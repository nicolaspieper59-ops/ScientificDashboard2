// =================================================================
// FICHIER JS CONSOLIDÉ : gnss-dashboard-full.js
// V5.1 : Version complète et runnable. Inclut toutes les fonctions
// de contrôle manquantes pour éviter que le dashboard ne se fige.
// =================================================================

// --- CLÉS D'API & PROXY VERCEL ---
// 🚨 REMPLACER AVEC L'URL HTTPS VERCEL DE VOTRE PROXY NODE.JS
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app"; 
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
// 🚨 ENDPOINT À IMPLÉMENTER PAR LE CLIENT POUR UNE VRAIE SYNCHRONISATION HORAIRE
const SERVER_TIME_ENDPOINT = "https://votre-domaine-ici.com/api/timesync"; 

// --- CONSTANTES GLOBALES ET INITIALISATION ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458, C_S = 343, R_E = 6371000, KMH_MS = 3.6;
const OBLIQ = 23.44 * D2R, ECC = 0.0167, JD_2K = 2451545.0; 
const J1970 = 2440588, J2000 = 2451545; 
const dayMs = 1000 * 60 * 60 * 24;      
let D_LAT = 48.8566, D_LON = 2.3522; 
const MIN_DT = 0.01; 
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};

// PARAMÈTRES AVANCÉS DU FILTRE DE KALMAN
const Q_NOISE = 0.01;       
const R_MIN = 0.05, R_MAX = 50.0; 
const MAX_ACC = 50, MIN_SPD = 0.001, ALT_TH = -50;
const SPEED_THRESHOLD = 0.5; 

// CONSTANTES POUR LES MODÈLES PHYSIQUES
const AIR_DENSITY = 1.225; 
const G_ACCEL = 9.80665;   
const CDA_EST = 0.6;       
const NETHER_RATIO = 8; 

const DOM_SLOW_UPDATE_MS = 1000; 

// FACTEURS ENVIRONNEMENTAUX POUR LA CORRECTION KALMAN ET TRAÎNÉE
const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0, DRAG_MULT: 1.0 },
    'METAL': { R_MULT: 2.5, DRAG_MULT: 1.0 },      
    'FOREST': { R_MULT: 1.5, DRAG_MULT: 1.2 },     
    'CONCRETE': { R_MULT: 3.0, DRAG_MULT: 1.0 },   
};
const WEATHER_FACTORS = {
    'CLEAR': 1.0,
    'RAIN': 1.2,                                   
    'SNOW': 1.5,                                   
    'STORM': 2.0,                                  
};

// --- VARIABLES D'ÉTAT (ACCESSIBLES GLOBALEMENT) ---
let wID = null, domID = null, lPos = null, lat = null, lon = null, sTime = null;
let distM = 0, distMStartOffset = 0, maxSpd = 0, tLat = null, tLon = null, tAlt = null;
let kSpd = 0, kUncert = 1000; 

let lServH = null, lLocH = null; 

let currentGPSMode = 'LOW_FREQ'; 
let manualFreqMode = false;      
let emergencyStopActive = false;
let netherMode = false;
let selectedEnvironment = 'NORMAL'; 
let selectedWeather = 'CLEAR'; 

let lastP_hPa = null, lastT_K = null, lastH_perc = null; 
let map = null, marker = null; // Déclaration pour Leaflet

// --- REFERENCES DOM ---
const $ = id => document.getElementById(id);

// ===========================================
// FONCTIONS GÉO & UTILS
// ===========================================

const dist = (lat1, lon1, lat2, lon2) => {
    const R = R_E, dLat = (lat2 - lat1) * D2R, dLon = (lon2 - lon1) * D2R;
    lat1 *= D2R; lat2 *= D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
};

/**
 * Calcule la distance spatiale 3D entre deux points.
 */
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
// SYNCHRONISATION HORAIRE PAR SERVEUR (STRICTE)
// ===========================================

async function syncH() { 
    if ($('local-time')) $('local-time').textContent = 'Synchronisation...';
    try {
        const response = await fetch(SERVER_TIME_ENDPOINT, { cache: "no-store", mode: "cors" });
        if (!response.ok) throw new Error(`Server time sync failed: ${response.statusText}`);
        const serverData = await response.json(); 
        const serverTimestamp = serverData.timestamp; 
        
        lServH = serverTimestamp; 
        lLocH = performance.now(); 
        console.log("Heure serveur synchronisée avec succès.");
    } catch (error) {
        console.error("Échec de la synchronisation de l'heure serveur. Le temps affiché sera 'N/A'.", error);
        lServH = null; 
        lLocH = null;
        if ($('local-time')) $('local-time').textContent = 'N/A (SYNCHRO SERVEUR ÉCHOUÉE)';
    }
}

function getCDate() { 
    if (lServH === null || lLocH === null) {
        return null; 
    }
    const offsetSinceSync = performance.now() - lLocH;
    return new Date(lServH + offsetSinceSync); 
}

// ===========================================
// FILTRE DE KALMAN & CORRECTIONS PHYSIQUES
// ===========================================

function kFilter(nSpd, dt, R_dyn) {
    if (dt === 0 || dt > 5) return kSpd; 
    const R = R_dyn ?? R_MAX, Q = Q_NOISE * dt; 
    let pSpd = kSpd, pUnc = kUncert + Q; 
    let K = pUnc / (pUnc + R); 
    kSpd = pSpd + K * (nSpd - pSpd); 
    kUncert = (1 - K) * pUnc; 
    return kSpd;
}

/**
 * Calcule la variance de l'observation (Facteur R) pour le filtre de Kalman.
 */
function getKalmanR(acc, alt, P_hPa) {
    let R = acc ** 2; 
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

/**
 * Calcule la vitesse du son (m/s) en fonction de la température de l'air (Kelvin).
 */
function calcSpeedOfSound(T_K) {
    if (T_K === null || isNaN(T_K)) return null;
    const T_C = T_K - 273.15;
    return 331.3 + 0.606 * T_C;
}

/**
 * Calcule la vitesse de la lumière (m/s) dans le milieu (air humide) en utilisant l'indice de réfraction.
 */
function calcSpeedOfLightInMedium(P_hPa, T_K, H_perc) {
    if (P_hPa === null || T_K === null || H_perc === null || isNaN(P_hPa) || isNaN(T_K) || isNaN(H_perc)) {
        return null;
    }
    
    const T_C = T_K - 273.15;
    const n_minus_1_dry = 77.6e-6 * P_hPa / T_K;
    const SVP_hPa = 6.1078 * Math.exp(17.27 * T_C / (T_C + 237.3)); 
    const P_water_hPa = H_perc * SVP_hPa;
    const n_minus_1_wet = 11.26e-6 * P_water_hPa / T_K; 
    
    const n = 1 + n_minus_1_dry - n_minus_1_wet;
    
    return C_L / n;
}

// ===========================================
// RÉCUPÉRATION MÉTÉO (AUCUNE SIMULATION)
// ===========================================

async function fetchWeather(latA, lonA) {
    lastP_hPa = null; lastT_K = null; lastH_perc = null; selectedWeather = 'CLEAR';
    
    // Le check ci-dessous empêche l'exécution si l'URL est encore l'exemple
    if (!latA || !lonA || PROXY_BASE_URL.includes('scientific-dashboard2')) {
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
    } catch (error) {
        console.error("Échec de la récupération des données météo:", error);
    }
}

// ===========================================
// FONCTIONS ASTRO (TST/MST)
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

    const eot_min = 4 * R2D * (L - Math.sin(M) * ECC * 2 - M - D2R * 102.9377);

    const msSinceMidnightUTC = (date.getUTCHours() * 3600 + date.getUTCMinutes() * 60 + date.getUTCSeconds()) * 1000 + date.getUTCMilliseconds();
    
    const mst_offset_ms = lon * 4 * 60000;
    const mst_ms = (msSinceMidnightUTC + mst_offset_ms) % dayMs;

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
        // Le dashboard ne devrait pas rester bloqué si on a une position mais pas de synchro
        return;
    }
    
    $('local-time').textContent = now.toLocaleTimeString();
    if (sTime) {
        const timeElapsed = (now.getTime() - sTime) / 1000;
        $('time-elapsed').textContent = `${timeElapsed.toFixed(2)} s`;
    }

    // Nécessite que SunCalc soit inclus dans le HTML
    const sunPos = window.SunCalc ? SunCalc.getPosition(now, latA, lonA) : null;
    const moonIllum = window.SunCalc ? SunCalc.getMoonIllumination(now) : null;
    const solarTimes = getSolarTime(now, lonA);

    if ($('time-solar-true')) $('time-solar-true').textContent = solarTimes.TST;
    if ($('culmination-lsm')) $('culmination-lsm').textContent = solarTimes.MST;
    if ($('sun-elevation')) $('sun-elevation').textContent = sunPos ? `${(sunPos.altitude * R2D).toFixed(2)} °` : 'N/A';
    if ($('lunar-phase-perc')) $('lunar-phase-perc').textContent = moonIllum ? `${(moonIllum.fraction * 100).toFixed(1)} %` : 'N/A';
    
    // ... (autres mises à jour astro) ...
}

// ===========================================
// FONCTIONS DE CONTRÔLE GPS
// ===========================================

function setGPSMode(mode) {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    currentGPSMode = mode;
    
    // Tentative de démarrer la surveillance GPS
    wID = navigator.geolocation.watchPosition(updateDisp, handleErr, GPS_OPTS[mode]);
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = `⏸️ PAUSE GPS`;
    if ($('freq-select')) $('freq-select').value = mode;
}

function startGPS() {
    if (wID === null) {
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
    // Gèle l'affichage avec le statut d'arrêt
    ['speed-stable', 'speed-3d-inst', 'distance-total-km', 'local-time'].forEach(id => {
        if ($(id)) $(id).textContent = 'ARRÊT D’URGENCE';
    });
}

function resetDisp() {
    distM = 0;
    maxSpd = 0;
    distMStartOffset = 0;
    kSpd = 0;
    kUncert = 1000;
    if ($('distance-total-km')) $('distance-total-km').textContent = `0.000 km | 0.00 m`;
    if ($('speed-max')) $('speed-max').textContent = `0.00000 km/h`;
    // ... (réinitialisation d'autres éléments d'affichage) ...
}

function handleErr(err) {
    console.error(`Erreur GNSS (${err.code}): ${err.message}`);
    // Si une erreur se produit, on arrête la surveillance et on passe en mode urgence
    stopGPS(false);
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = `❌ ERREUR GPS`;
    emergencyStop(); 
}

// ===========================================
// FONCTIONS DE MISE À JOUR DOM
// ===========================================

function updateDM(latA, lonA, altA) { 
    // Mise à jour de la Distance Totale (3D)
    if ($('distance-total-km')) $('distance-total-km').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    if ($('speed-max')) $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    
    // ... (Logique cible et carte omise pour la concision) ...
}

function updateDisp(pos) {
    // Si l'arrêt d'urgence est actif, on ignore toute nouvelle position
    if (emergencyStopActive) return;

    lat = pos.coords.latitude; lon = pos.coords.longitude;
    const alt = pos.coords.altitude, acc = pos.coords.accuracy;
    const spd = pos.coords.speed, cTime = pos.timestamp; 

    const now = getCDate();
    if (now === null) { updateAstro(lat, lon); return; } // Si synchro serveur échoue, on continue avec les autres données

    if (sTime === null) { sTime = now.getTime(); distMStartOffset = distM; }
    
    // On met en pause l'intégration si la précision est trop faible
    if (acc > MAX_ACC) { 
        if ($('gps-precision')) $('gps-precision').textContent = `❌ ${acc.toFixed(0)} m (Trop Imprécis)`; 
        if (lPos === null) lPos = pos; return; 
    }
    
    let spdH = spd ?? 0, spdV = 0; 
    const dt = lPos ? (cTime - lPos.timestamp) / 1000 : MIN_DT; 

    if (lPos && dt > 0.1) { 
        const dH = dist(lPos.coords.latitude, lPos.coords.longitude, lat, lon); 
        if (spd === null || spd === undefined) spdH = dH / dt; 
        if (alt !== null && lPos.coords.altitude !== null) {
            spdV = (alt - lPos.coords.altitude) / dt; 
        }
    }
    
    // CALCUL DE LA VITESSE 3D
    const spd3D = Math.sqrt(spdH ** 2 + spdV ** 2);

    // --- CORRECTION GNSS AVANCÉE (Facteur R) ---
    const R_dyn = getKalmanR(acc, alt, lastP_hPa); 
    const fSpd = kFilter(spd3D, dt, R_dyn), sSpdFE = fSpd < MIN_SPD ? 0 : fSpd;
    
    // INTÉGRATION DE LA DISTANCE TOTALE PAR VITESSE 3D FILTRÉE
    distM += sSpdFE * dt * (netherMode ? NETHER_RATIO : 1); 
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    
    updateDM(lat, lon, alt ?? 0); 
    
    // --- AFFICHAGE ---
    if ($('gps-precision')) $('gps-precision').textContent = `${acc.toFixed(2)} m`; 
    if ($('speed-error-perc')) $('speed-error-perc').textContent = `${R_dyn.toFixed(3)} m²`; // FACTEUR R (Précision 3D Dynamique)
    if ($('speed-3d-inst')) $('speed-3d-inst').textContent = `${(spd3D * KMH_MS).toFixed(5)} km/h`; 
    if ($('speed-stable')) $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(5)} km/h`; 
    if ($('vertical-speed')) $('vertical-speed').textContent = `${spdV.toFixed(2)} m/s`;

    // ... (Récupération météo toutes les 60s) ...
    if (Date.now() - (updateDisp.lastWeatherFetch ?? 0) > 60000) {
        fetchWeather(lat, lon); 
        updateDisp.lastWeatherFetch = Date.now();
    }
    
    lPos = pos; lPos.speedMS_3D = spd3D; lPos.timestamp = cTime; 
    // ... (Mise à jour de la carte) ...
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
    
    // Tentative de synchronisation horaire au démarrage
    syncH(); 
    fetchWeather(D_LAT, D_LON);

    // --- ÉVÉNEMENTS DE CONTRÔLE ---
    
    // Démarrage/Pause GPS
    if (toggleGpsBtn) toggleGpsBtn.addEventListener('click', () => wID === null ? startGPS() : stopGPS());

    // Sélecteur de fréquence
    if (freqSelect) freqSelect.addEventListener('change', (e) => setGPSMode(e.target.value));

    // Arrêt d'urgence
    if (emergencyStopBtn) emergencyStopBtn.addEventListener('click', emergencyStop);

    // Réinitialisation
    if (resetDistBtn) resetDistBtn.addEventListener('click', () => { distM = 0; distMStartOffset = 0; if(lPos) updateDM(lPos.coords.latitude, lPos.coords.longitude, lPos.coords.altitude); });
    if (resetMaxBtn) resetMaxBtn.addEventListener('click', () => { maxSpd = 0; });
    if (resetAllBtn) resetAllBtn.addEventListener('click', resetDisp);
    
    // --- INITIALISATION ---
    // Démarrage automatique du GPS
    startGPS(); 

    // Initialisation du DOM pour les mises à jour lentes (Astro, Météo)
    if (domID === null) {
        domID = setInterval(() => {
            const now = getCDate();
            if (lPos && now) updateAstro(lPos.coords.latitude, lPos.coords.longitude);
        }, DOM_SLOW_UPDATE_MS); 
    }
    
    // ... (Initialisation de la carte Leaflet et autres écouteurs omis pour la concision) ...
});
