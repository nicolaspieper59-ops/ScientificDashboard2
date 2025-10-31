// =================================================================
// FICHIER JS PARTIE 1/2 : gnss-dashboard-part1.js
// Contient constantes, variables d'état et fonctions utilitaires.
// DOIT ÊTRE CHARGÉ AVANT gnss-dashboard-part2.js.
// =================================================================

// --- CLÉS D'API & PROXY VERCEL (À METTRE À JOUR) ---
// 🚨 REMPLACER AVEC L'URL HTTPS VERCEL DE VOTRE PROXY NODE.JS
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app"; 
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
// 🚨 ENDPOINT À IMPLÉMENTER POUR UNE VRAIE SYNCHRONISATION HORAIRE
const SERVER_TIME_ENDPOINT = "https://votre-domaine-ici.com/api/timesync"; 

// --- CONSTANTES GLOBALES ET INITIALISATION ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458, R_E = 6371000, KMH_MS = 3.6; // Vitesse de la Lumière, Rayon Terre, Conversion Vitesse
const J1970 = 2440588, J2000 = 2451545; // Jours Juliens
const dayMs = 1000 * 60 * 60 * 24;      
const MIN_DT = 0.01; 
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};

// PARAMÈTRES AVANCÉS DU FILTRE DE KALMAN
const Q_NOISE = 0.01;       
const R_MIN = 0.05, R_MAX = 50.0; 
const MAX_ACC = 50, MIN_SPD = 0.001, ALT_TH = -50;
const NETHER_RATIO = 8; 

// FACTEURS ENVIRONNEMENTAUX POUR LA CORRECTION KALMAN
const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0 },
    'METAL': { R_MULT: 2.5 },      
    'FOREST': { R_MULT: 1.5 },     
    'CONCRETE': { R_MULT: 3.0 },   
};
const DOM_SLOW_UPDATE_MS = 1000; 

// --- VARIABLES D'ÉTAT (ACCESSIBLES GLOBALEMENT) ---
let wID = null, domID = null, lPos = null, lat = null, lon = null, sTime = null;
let distM = 0, distMStartOffset = 0, maxSpd = 0;
let kSpd = 0, kUncert = 1000; // État du filtre de Kalman

let lServH = null, lLocH = null; // Pour la synchronisation horaire serveur

// CHANGÉ À HIGH_FREQ PAR DÉFAUT
let currentGPSMode = 'HIGH_FREQ'; 
let emergencyStopActive = false;
let netherMode = false;
let selectedEnvironment = 'NORMAL'; 
let selectedWeather = 'CLEAR'; 

let lastP_hPa = null, lastT_K = null, lastH_perc = null; 

// --- REFERENCES DOM ---
const $ = id => document.getElementById(id);

// ===========================================
// FONCTIONS GÉO & UTILS
// ===========================================

/** Calcule la distance horizontale (Haversine). */
const dist = (lat1, lon1, lat2, lon2) => {
    const R = R_E, dLat = (lat2 - lat1) * D2R, dLon = (lon2 - lon1) * D2R;
    lat1 *= D2R; lat2 *= D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
};

/** Calcule la distance spatiale 3D. */
const dist3D = (lat1, lon1, alt1 = 0, lat2, lon2, alt2 = 0) => {
    const dH = dist(lat1, lon1, lat2, lon2);
    const dV = alt2 - alt1;                  
    return Math.sqrt(dH ** 2 + dV ** 2);     
};

const bearing = (lat1, lon1, lat2, lon2) => {
    lat1 *= D2R; lon1 *= D2R; lat2 *= D2R; lon2 *= D2R;
    const y = Math.sin(lon2 - lon1) * Math.cos(lat2);
    const x = Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1) * Math.cos(lat2) * Math.cos(lon2 - lon1);
    let b = Math.atan2(y, x) * R2D;
    return (b + 360) % 360; 
};


// ===========================================
// SYNCHRONISATION HORAIRE PAR SERVEUR
// ===========================================

async function syncH() { 
    if ($('local-time')) $('local-time').textContent = 'Synchronisation...';
    try {
        // NOTE: L'URL du serveur est très probablement non fonctionnelle et va lancer l'erreur.
        const response = await fetch(SERVER_TIME_ENDPOINT, { cache: "no-store", mode: "cors" });
        if (!response.ok) throw new Error(`Server time sync failed: ${response.statusText}`);
        const serverData = await response.json(); 
        const serverTimestamp = serverData.timestamp; 
        
        lServH = serverTimestamp; 
        lLocH = performance.now(); 
    } catch (error) {
        console.warn("Échec de la synchronisation de l'heure serveur. Utilisation de l'horloge locale.", error);
        lServH = Date.now(); // Fallback vers l'heure locale
        lLocH = performance.now();
        if ($('local-time')) $('local-time').textContent = 'N/A (SYNCHRO ÉCHOUÉE)';
    }
}

/** Retourne l'heure synchronisée (ou null si la synchro a échoué). */
function getCDate() { 
    if (lServH === null || lLocH === null) { return null; }
    const offsetSinceSync = performance.now() - lLocH;
    return new Date(lServH + offsetSinceSync); 
}

// ===========================================
// FILTRE DE KALMAN & FACTEUR R
// ===========================================

/** Applique le filtre de Kalman à la vitesse 3D. */
function kFilter(nSpd, dt, R_dyn) {
    if (dt === 0 || dt > 5) return kSpd; 
    const R = R_dyn ?? R_MAX, Q = Q_NOISE * dt; 
    let pSpd = kSpd, pUnc = kUncert + Q; 
    let K = pUnc / (pUnc + R); 
    kSpd = pSpd + K * (nSpd - pSpd); 
    kUncert = (1 - K) * pUnc; 
    return kSpd;
}

/** Calcule le Facteur R (Précision 3D Dynamique) du filtre de Kalman. */
function getKalmanR(acc, alt, P_hPa) {
    let R = acc ** 2; // Basé sur la précision horizontale au carré
    R = Math.max(R_MIN, Math.min(R_MAX, R));
    
    const envFactor = ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT;
    
    if (P_hPa !== null) {
        const pressureFactor = 1.0 + (1013.25 - P_hPa) / 1013.25 * 0.1;
        R *= Math.max(1.0, pressureFactor);
    }
    
    R *= envFactor;
    if (alt < ALT_TH) { R *= 2.0; } 

    R = Math.max(R_MIN, Math.min(R_MAX, R));
    
    return R;
}

// ===========================================
// RÉCUPÉRATION MÉTÉO (Proxy Vercel requis)
// ===========================================

async function fetchWeather(latA, lonA) {
    lastP_hPa = null; lastT_K = null; lastH_perc = null; selectedWeather = 'CLEAR';
    
    // Empêche l'exécution si l'URL est encore l'exemple
    if (!latA || !lonA || PROXY_BASE_URL.includes('scientific-dashboard2')) {
        if ($('env-factor')) $('env-factor').textContent = selectedEnvironment;
        return; 
    }

    try {
        const response = await fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${latA}&lon=${lonA}`);
        if (!response.ok) throw new Error(`Proxy error: ${response.statusText}`);
        
        const data = await response.json();
        
        if (data.main) {
            lastP_hPa = data.main.pressure; 
            lastT_K = data.main.temp + 273.15; 
            lastH_perc = data.main.humidity / 100; 
        }

        if (data.weather && data.weather.length > 0) {
            const condition = data.weather[0].main.toLowerCase(); 
            if (condition.includes('rain') || condition.includes('drizzle')) selectedWeather = 'RAIN';
            else if (condition.includes('snow')) selectedWeather = 'SNOW';
            else if (condition.includes('storm') || condition.includes('thunder')) selectedWeather = 'STORM';
            else selectedWeather = 'CLEAR';
        }
        
        // Mise à jour du DOM Météo
        if ($('temp-air')) $('temp-air').textContent = lastT_K ? `${(lastT_K - 273.15).toFixed(1)} °C` : 'N/A';
        if ($('pressure')) $('pressure').textContent = lastP_hPa ? `${lastP_hPa.toFixed(1)} hPa` : 'N/A';
        if ($('humidity')) $('humidity').textContent = lastH_perc ? `${(lastH_perc * 100).toFixed(0)} %` : 'N/A';
        if ($('env-factor')) $('env-factor').textContent = `${selectedEnvironment} (x${ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT})`;

    } catch (error) {
        console.error("Échec de la récupération des données météo:", error);
        if ($('env-factor')) $('env-factor').textContent = `${selectedEnvironment} (MÉTÉO ÉCHOUÉE)`;
    }
    }
// =================================================================
// FICHIER JS PARTIE 2/2 : gnss-dashboard-part2.js
// Contient la logique principale de mise à jour et les écouteurs d'événements.
// NÉCESSITE gnss-dashboard-part1.js
// =================================================================

// ===========================================
// FONCTIONS ASTRO
// ===========================================

function toDays(date) { return (date.valueOf() / dayMs - 0.5 + J1970) - J2000; }
function solarMeanAnomaly(d) { return D2R * (356.0470 + 0.9856002585 * d); }
function eclipticLongitude(M) {
    var C = D2R * (1.9148 * Math.sin(M) + 0.0200 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M)), 
        P = D2R * 102.9377;                                                                
    return M + C + P + Math.PI;
}

function getSolarTime(date, lon) {
    if (date === null || lon === null) return { TST: 'N/A', MST: 'N/A' };

    const d = toDays(date);
    const M = solarMeanAnomaly(d);
    const L = eclipticLongitude(M);

    // Équation du temps (EOT) en minutes
    const eot_min = 4 * R2D * (L - Math.sin(M) * 0.0167 * 2 - M - D2R * 102.9377);

    const msSinceMidnightUTC = (date.getUTCHours() * 3600 + date.getUTCMinutes() * 60 + date.getUTCSeconds()) * 1000 + date.getUTCMilliseconds();
    
    // Heure Solaire Moyenne (MST)
    // Correction de la formule : long * (24h / 360°) * (1 jour / 24h) = long * 4 minutes (mais on veut en ms)
    const mst_offset_ms = lon * dayMs / 360; 
    const mst_ms = (msSinceMidnightUTC + mst_offset_ms) % dayMs;

    // Heure Solaire Vraie (TST)
    const tst_ms = (mst_ms - eot_min * 60000 + dayMs) % dayMs;

    const toTimeString = (ms) => {
        let h = Math.floor(ms / 3600000);
        let m = Math.floor((ms % 3600000) / 60000);
        let s = Math.floor((ms % 60000) / 1000);
        return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
    };
    
    return { 
        TST: toTimeString(tst_ms),
        MST: toTimeString(mst_ms)
    };
}

function updateAstro(latA, lonA) {
    const now = getCDate(); 
    
    if (now === null) {
        return;
    }
    
    $('local-time').textContent = now.toLocaleTimeString();
    if (sTime) {
        const timeElapsed = (now.getTime() - sTime) / 1000;
        $('time-elapsed').textContent = `${timeElapsed.toFixed(2)} s`;
    }

    const solarTimes = getSolarTime(now, lonA);
    // SunCalc doit être chargé via un CDN/script
    const sunPos = window.SunCalc ? SunCalc.getPosition(now, latA, lonA) : null;
    const moonIllum = window.SunCalc ? SunCalc.getMoonIllumination(now) : null;

    if ($('time-solar-true')) $('time-solar-true').textContent = solarTimes.TST;
    if ($('culmination-lsm')) $('culmination-lsm').textContent = solarTimes.MST;
    if ($('sun-elevation')) $('sun-elevation').textContent = sunPos ? `${(sunPos.altitude * R2D).toFixed(2)} °` : 'N/A';
    if ($('lunar-phase-perc')) $('lunar-phase-perc').textContent = moonIllum ? `${(moonIllum.fraction * 100).toFixed(1)} %` : 'N/A';
}


// ===========================================
// FONCTIONS DE CONTRÔLE GPS
// ===========================================

function setGPSMode(mode) {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    currentGPSMode = mode;
    
    wID = navigator.geolocation.watchPosition(updateDisp, handleErr, GPS_OPTS[mode]);
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = `⏸️ PAUSE GPS`;
    // Assure que le sélecteur DOM est à jour
    if ($('freq-select')) $('freq-select').value = mode; 
}

function startGPS() {
    if (wID === null) {
        // Assure que le mode HIGH_FREQ (défaut dans part1.js) est appliqué
        if ($('freq-select')) $('freq-select').value = currentGPSMode; 
        setGPSMode(currentGPSMode);
    }
}

function stopGPS(resetButton = true) {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
    }
    if (resetButton) {
        if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = `▶️ MARCHE GPS`;
    }
}

function emergencyStop() {
    emergencyStopActive = true;
    stopGPS(false);
    if ($('emergency-stop-btn')) {
        $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: ACTIF 🔴";
        $('emergency-stop-btn').classList.add('active');
    }
    // Gèle l'affichage
    ['speed-stable', 'speed-3d-inst', 'distance-total-km', 'local-time'].forEach(id => {
        if ($(id)) $(id).textContent = 'ARRÊT D’URGENCE';
    });
}

function resetDisp() {
    distM = 0; maxSpd = 0; distMStartOffset = 0; kSpd = 0; kUncert = 1000;
    if ($('distance-total-km')) $('distance-total-km').textContent = `0.000 km | 0.00 m`;
    if ($('speed-max')) $('speed-max').textContent = `0.00000 km/h`;
}

function handleErr(err) {
    console.error(`Erreur GNSS (${err.code}): ${err.message}`);
    stopGPS(false);
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = `❌ ERREUR GPS`;
    emergencyStop(); 
}

// ===========================================
// FONCTION PRINCIPALE DE MISE À JOUR GPS
// ===========================================

function updateDisp(pos) {
    if (emergencyStopActive) return;

    lat = pos.coords.latitude; lon = pos.coords.longitude;
    const alt = pos.coords.altitude, acc = pos.coords.accuracy;
    const spd = pos.coords.speed, cTime = pos.timestamp; 

    const now = getCDate();
    if (now === null) { updateAstro(lat, lon); return; } 

    if (sTime === null) { sTime = now.getTime(); distMStartOffset = distM; }
    
    if (acc > MAX_ACC) { 
        if ($('gps-precision')) $('gps-precision').textContent = `❌ ${acc.toFixed(0)} m (Trop Imprécis)`; 
        if (lPos === null) lPos = pos; return; 
    }
    
    let spdH = spd ?? 0, spdV = 0; 
    const dt = lPos ? (cTime - lPos.timestamp) / 1000 : MIN_DT; 

    // Calcul de la vitesse à partir de la distance parcourue (plus fiable que pos.coords.speed, surtout si speed est null)
    if (lPos && dt > 0.05) { // Utiliser un seuil dt minimal pour éviter les divisions par zéro/très petits nombres
        const dH = dist(lPos.coords.latitude, lPos.coords.longitude, lat, lon); 
        spdH = dH / dt; 
        if (alt !== null && lPos.coords.altitude !== null) {
            spdV = (alt - lPos.coords.altitude) / dt; 
        }
    } else if (spd !== null) {
        spdH = spd; // Fallback si le calcul dH/dt n'est pas possible
    }
    
    // CALCUL DE LA VITESSE 3D
    const spd3D = Math.sqrt(spdH ** 2 + spdV ** 2);

    // FILTRAGE DE KALMAN AVEC FACTEUR R DYNAMIQUE
    const R_dyn = getKalmanR(acc, alt, lastP_hPa); 
    const fSpd = kFilter(spd3D, dt, R_dyn), sSpdFE = fSpd < MIN_SPD ? 0 : fSpd;
    
    // INTÉGRATION DE LA DISTANCE TOTALE 3D
    distM += sSpdFE * dt * (netherMode ? NETHER_RATIO : 1); 
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    
    // --- MISE À JOUR DU DOM (Affichage des données réalistes) ---
    
    if ($('latitude')) $('latitude').textContent = lat.toFixed(6);
    if ($('longitude')) $('longitude').textContent = lon.toFixed(6);
    if ($('altitude-gps')) $('altitude-gps').textContent = alt !== null ? `${alt.toFixed(2)} m` : 'N/A';
    
    if ($('gps-precision')) $('gps-precision').textContent = `${acc.toFixed(2)} m`; 
    if ($('speed-error-perc')) $('speed-error-perc').textContent = `${R_dyn.toFixed(3)} m² (R dyn)`; // FACTEUR R (Précision 3D Dynamique)
    if ($('speed-3d-inst')) $('speed-3d-inst').textContent = `${(spd3D * KMH_MS).toFixed(5)} km/h`; 
    if ($('speed-stable')) $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(5)} km/h`; 
    if ($('vertical-speed')) $('vertical-speed').textContent = `${spdV.toFixed(2)} m/s`;
    
    if ($('distance-total-km')) $('distance-total-km').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    if ($('speed-max')) $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    if ($('perc-speed-c')) $('perc-speed-c').textContent = `${(spd3D / C_L * 100).toExponential(2)}%`;

    // Gestion du mode nether
    if ($('mode-nether')) $('mode-nether').textContent = netherMode ? "ACTIVÉ (1:8) 🔥" : "DÉSACTIVÉ (1:1)";

    // Mise à jour de l'état "Sous-sol"
    if ($('underground-status')) $('underground-status').textContent = alt !== null && alt < ALT_TH ? `OUI (< ${ALT_TH}m)` : 'Non';

    // Récupération météo toutes les 60s
    if (Date.now() - (updateDisp.lastWeatherFetch ?? 0) > 60000) {
        fetchWeather(lat, lon); 
        updateDisp.lastWeatherFetch = Date.now();
    }
    
    lPos = pos; lPos.speedMS_3D = spd3D; lPos.timestamp = cTime; 
}


// ===========================================
// INITIALISATION DES ÉVÉNEMENTS DOM
// ===========================================

document.addEventListener('DOMContentLoaded', () => {
    
    const toggleGpsBtn = $('toggle-gps-btn');
    const resetDistBtn = $('reset-dist-btn');
    const resetMaxBtn = $('reset-max-btn');
    const resetAllBtn = $('reset-all-btn');
    const emergencyStopBtn = $('emergency-stop-btn');
    const freqSelect = $('freq-select'); 
    const netherToggleBtn = $('nether-toggle-btn');
    
    // Tentative de synchronisation horaire au démarrage (avec fallback local)
    syncH(); 

    // --- ÉVÉNEMENTS DE CONTRÔLE ---
    
    // Démarrage/Pause GPS
    if (toggleGpsBtn) toggleGpsBtn.addEventListener('click', () => wID === null ? startGPS() : stopGPS());

    // Sélecteur de fréquence
    if (freqSelect) freqSelect.addEventListener('change', (e) => setGPSMode(e.target.value));

    // Arrêt d'urgence
    if (emergencyStopBtn) emergencyStopBtn.addEventListener('click', emergencyStop);

    // Mode Nether
    if (netherToggleBtn) netherToggleBtn.addEventListener('click', () => {
        netherMode = !netherMode;
        if ($('mode-nether')) $('mode-nether').textContent = netherMode ? "ACTIVÉ (1:8) 🔥" : "DÉSACTIVÉ (1:1)";
    });
    
    // Sélecteur d'Environnement (Ajouté pour tester le facteur R du Kalman)
    const envSelect = document.createElement('select');
    envSelect.id = 'env-select';
    Object.keys(ENVIRONMENT_FACTORS).forEach(env => {
        const opt = document.createElement('option');
        opt.value = env; opt.textContent = env;
        envSelect.appendChild(opt);
    });
    envSelect.value = selectedEnvironment;
    envSelect.addEventListener('change', (e) => {
        selectedEnvironment = e.target.value;
        if ($('env-factor')) $('env-factor').textContent = `${selectedEnvironment} (x${ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT})`;
    });
    // Ajoutez le sélecteur d'environnement au bloc de contrôle
    const controlsSection = document.querySelector('.controls');
    if (controlsSection) {
        const envDiv = document.createElement('div');
        envDiv.className = 'data-item';
        envDiv.innerHTML = '<span class="label">Environnement Kalman</span>';
        envDiv.appendChild(envSelect);
        controlsSection.appendChild(envDiv);
    }

    // Réinitialisation
    if (resetDistBtn) resetDistBtn.addEventListener('click', () => { 
        distM = 0; distMStartOffset = 0; 
        if ($('distance-total-km')) $('distance-total-km').textContent = `0.000 km | 0.00 m`;
    });
    if (resetMaxBtn) resetMaxBtn.addEventListener('click', () => { 
        maxSpd = 0; 
        if ($('speed-max')) $('speed-max').textContent = `0.00000 km/h`;
    });
    if (resetAllBtn) resetAllBtn.addEventListener('click', () => {
        if (confirm("Êtes-vous sûr de vouloir tout réinitialiser (Distance, Max, etc.) ?")) {
            resetDisp();
        }
    });
    
    // --- INITIALISATION ---
    // Démarrage automatique du GPS
    startGPS(); 

    // Initialisation du DOM pour les mises à jour lentes (Astro)
    if (domID === null) {
        domID = setInterval(() => {
            if (lPos) updateAstro(lPos.coords.latitude, lPos.coords.longitude);
            // Si pas de position, mettre à jour l'heure uniquement
            else updateAstro(null, null); 
        }, DOM_SLOW_UPDATE_MS); 
    }
});
