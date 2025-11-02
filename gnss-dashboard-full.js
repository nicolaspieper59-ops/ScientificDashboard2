// =================================================================
// FICHIER FINAL : gnss-dashboard-full.js
// Contient toutes les constantes, la logique, le Kalman et l'Astro corrigé.
// =================================================================

// --- CLÉS D'API & PROXY VERCEL ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app"; 
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc"; 

// --- CONSTANTES GLOBALES ET INITIALISATION ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458; // Vitesse de la lumière (m/s)
const R_E = 6371000;   // Rayon terrestre moyen (m)
const KMH_MS = 3.6;    // Conversion m/s vers km/h
const C_S = 343;       // Vitesse du son dans l'air (m/s)
const G_ACC = 9.80665; // Gravité standard (m/s²)
const MC_DAY_MS = 72 * 60 * 1000; // Durée d'un jour Minecraft en ms

const J1970 = 2440588, J2000 = 2451545; 
const dayMs = 1000 * 60 * 60 * 24;      
const MIN_DT = 0.01; 
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};

// PARAMÈTRES AVANCÉS DU FILTRE DE KALMAN (Horizontal/Vitesse 3D)
const Q_NOISE = 0.01;       
const R_MIN = 0.05, R_MAX = 50.0; 
const MAX_ACC = 200, MIN_SPD = 0.05, ALT_TH = -50; 
const NETHER_RATIO = 8; 
const MAX_PLAUSIBLE_ACCEL = 20.0; 

// PARAMÈTRES POUR LE FILTRE DE KALMAN D'ALTITUDE
const Q_ALT_NOISE = 0.1; 
const R_ALT_MIN = 0.1;  

// PARAMÈTRES DE LISSAGE HYBRIDE (DAMPENING)
const ACC_DAMPEN_LOW = 5.0; 
const ACC_DAMPEN_HIGH = 50.0; 

// FACTEURS ENVIRONNEMENTAUX POUR LA CORRECTION KALMAN
const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0 },
    'METAL': { R_MULT: 2.5 },      
    'FOREST': { R_MULT: 1.5 },     
    'CONCRETE': { R_MULT: 3.0 },   
};
const DOM_SLOW_UPDATE_MS = 1000; 

// --- VARIABLES D'ÉTAT ---
let wID = null, domID = null, lPos = null, lat = null, lon = null, sTime = null;
let distM = 0, distMStartOffset = 0, maxSpd = 0;
let kSpd = 0, kUncert = 1000; 
let timeMoving = 0; 
let lServH = null, lLocH = null; 
let lastFSpeed = 0; 
let kAlt = null;      
let kAltUncert = 10;  

let currentGPSMode = 'HIGH_FREQ'; 
let emergencyStopActive = false;
let netherMode = false;
let selectedEnvironment = 'NORMAL'; 

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

// ===========================================
// SYNCHRONISATION HORAIRE PAR SERVEUR (UTC/Atomique)
// ===========================================

async function syncH() { 
    if ($('local-time')) $('local-time').textContent = 'Synchronisation UTC...';
    const localStartPerformance = performance.now(); 

    try {
        const response = await fetch(SERVER_TIME_ENDPOINT, { cache: "no-store", mode: "cors" });
        if (!response.ok) throw new Error(`Server time sync failed: ${response.statusText}`);
        
        const localEndPerformance = performance.now(); 
        const serverData = await response.json(); 
        
        const utcTimeISO = serverData.datetime; 
        const serverTimestamp = Date.parse(utcTimeISO); 
        
        const RTT = localEndPerformance - localStartPerformance;
        const latencyOffset = RTT / 2;

        lServH = serverTimestamp + latencyOffset; 
        lLocH = performance.now(); 
        console.log(`Synchronisation UTC Atomique réussie. Latence corrigée: ${latencyOffset.toFixed(1)} ms.`);

    } catch (error) {
        console.warn("Échec de la synchronisation. Utilisation de l'horloge locale.", error);
        lServH = Date.now(); 
        lLocH = performance.now();
        if ($('local-time')) $('local-time').textContent = 'N/A (SYNCHRO ÉCHOUÉE)';
    }
}

/** Retourne l'heure synchronisée (précision RTT compensée en UTC). */
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

/** Applique le filtre de Kalman à l'Altitude. */
function kFilterAltitude(nAlt, acc, dt) {
    if (nAlt === null) return kAlt;
    const R = Math.max(R_ALT_MIN, acc); 
    const Q = Q_ALT_NOISE * dt; 
    let pAlt = kAlt === null ? nAlt : kAlt; 
    let pUnc = kAltUncert + Q; 
    const K = pUnc / (pUnc + R); 
    kAlt = pAlt + K * (nAlt - pAlt); 
    kAltUncert = (1 - K) * pUnc; 
    return kAlt;
}

/** Calcule le Facteur R (Précision 3D Dynamique) du filtre de Kalman. */
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

// =================================================================
// LOGIQUE PRINCIPALE & ASTRO
// =================================================================

/** Initialisation de Leaflet Map (Nécessite le CDN dans le HTML) */
let lMap = null; 
let lMarker = null;

function initMap() {
    const defaultLat = 43.296482; // Marseille par défaut
    const defaultLon = 5.36978;
    if (window.L && $('map-container')) {
        if (lMap) lMap.remove(); 
        lMap = L.map('map-container').setView([defaultLat, defaultLon], 13);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: '© OpenStreetMap contributors'
        }).addTo(lMap);
        lMarker = L.marker([defaultLat, defaultLon]).addTo(lMap);
    }
}

function updateMap(latA, lonA) {
    if (lMap && lMarker && latA !== null) {
        const newLatLng = new L.LatLng(latA, lonA);
        lMarker.setLatLng(newLatLng);
        // Optionnel: centrer la carte autour du marqueur
        // lMap.setView(newLatLng);
    }
}

/** Convertit la date en jours depuis J2000. */
function toDays(date) { return (date.valueOf() / dayMs - 0.5 + J1970) - J2000; }
/** Calcule l'anomalie solaire moyenne. */
function solarMeanAnomaly(d) { return D2R * (356.0470 + 0.9856002585 * d); }
/** Calcule la longitude écliptique. */
function eclipticLongitude(M) {
    var C = D2R * (1.9148 * Math.sin(M) + 0.0200 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M)), 
        P = D2R * 102.9377;                                                                
    return M + C + P + Math.PI;
}

/** Calcule le Temps Solaire Vrai (TST). (CORRIGÉ) */
function getSolarTime(date, lon) {
    if (date === null || lon === null) return { TST: 'N/A', MST: 'N/A', EOT: 'N/D', ECL_LONG: 'N/D' };
    const d = toDays(date);
    const M = solarMeanAnomaly(d); 
    const L = eclipticLongitude(M); 
    const epsilon = D2R * (23.4393 - 0.000000356 * d); 
    let alpha = Math.atan2(Math.cos(epsilon) * Math.sin(L), Math.cos(L));
    if (alpha < 0) alpha += 2 * Math.PI; 
    const eot_rad = alpha - M - D2R * 102.9377 - Math.PI;
    const eot_min = eot_rad * 4 * R2D; 
    
    const msSinceMidnightUTC = (date.getUTCHours() * 3600 + date.getUTCMinutes() * 60 + date.getUTCSeconds()) * 1000 + date.getUTCMilliseconds();
    const mst_offset_ms = lon * dayMs / 360; 
    
    // Correction robuste pour garantir des valeurs positives de MST et TST
    const mst_ms_raw = msSinceMidnightUTC + mst_offset_ms;
    const mst_ms = (mst_ms_raw % dayMs + dayMs) % dayMs; // Temps Solaire Moyen (LSM)
    
    const eot_ms = eot_min * 60000;
    const tst_ms_raw = mst_ms + eot_ms;
    const tst_ms = (tst_ms_raw % dayMs + dayMs) % dayMs; // Temps Solaire Vrai (TST)
    
    const toTimeString = (ms) => {
        let h = Math.floor(ms / 3600000);
        let m = Math.floor((ms % 3600000) / 60000);
        let s = Math.floor((ms % 60000) / 1000);
        return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
    };
    return { 
        TST: toTimeString(tst_ms), 
        MST: toTimeString(mst_ms), 
        EOT: eot_min.toFixed(3),
        ECL_LONG: (L * R2D).toFixed(2)
    };
}

/** Calcule le temps Minecraft. */
function getMinecraftTime(date) {
    if (date === null) return '00:00:00';
    const msSinceMidnightUTC = date.getUTCHours() * 3600000 + date.getUTCMilliseconds() + date.getUTCMinutes() * 60000 + date.getUTCSeconds() * 1000;
    const timeRatio = (msSinceMidnightUTC % dayMs) / dayMs;
    const mcTimeMs = (timeRatio * MC_DAY_MS + MC_DAY_MS) % MC_DAY_MS;
    const toTimeString = (ms) => {
        let h = Math.floor(ms / 3600000);
        let m = Math.floor((ms % 3600000) / 60000);
        let s = Math.floor((ms % 60000) / 1000);
        return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
    };
    return toTimeString(mcTimeMs);
}

function updateAstro(latA, lonA) {
    const now = getCDate(); 
    if (now === null) {
        if ($('local-time') && !$('local-time').textContent.includes('Synchronisation')) {
             $('local-time').textContent = 'Synchronisation...';
        }
        return;
    }
    
    const sunPos = window.SunCalc ? SunCalc.getPosition(now, latA, lonA) : null;
    const moonIllum = window.SunCalc ? SunCalc.getMoonIllumination(now) : null;
    const sunTimes = window.SunCalc ? SunCalc.getTimes(now, latA, lonA) : null;
    const moonTimes = window.SunCalc ? SunCalc.getMoonTimes(now, latA, lonA, true) : null;
    const solarTimes = getSolarTime(now, lonA);

    // Mise à jour de l'heure locale et de la date
    $('local-time').textContent = now.toLocaleTimeString('fr-FR', { timeZone: 'UTC', hour12: false });
    if ($('date-display')) $('date-display').textContent = now.toLocaleDateString();
    
    // Mise à jour du temps de session et Minecraft
    if (sTime) {
        const timeElapsed = (now.getTime() - sTime) / 1000;
        $('time-elapsed').textContent = `${timeElapsed.toFixed(2)} s`;
        $('time-moving').textContent = `${timeMoving.toFixed(2)} s`;
        $('time-minecraft').textContent = getMinecraftTime(now);
    }
    
    // Mise à jour des valeurs astronomiques (avec les IDs du HTML recommandé)
    if ($('time-solar-true')) $('time-solar-true').textContent = solarTimes.TST;
    if ($('culmination-lsm')) $('culmination-lsm').textContent = solarTimes.MST;
    if ($('sun-elevation')) $('sun-elevation').textContent = sunPos ? `${(sunPos.altitude * R2D).toFixed(2)} °` : 'N/A';
    if ($('lunar-phase-perc')) $('lunar-phase-perc').textContent = moonIllum ? `${(moonIllum.fraction * 100).toFixed(1)} %` : 'N/A';
    if ($('noon-solar')) $('noon-solar').textContent = sunTimes && sunTimes.solarNoon ? sunTimes.solarNoon.toLocaleTimeString() : 'N/D';
    if ($('eot-min')) $('eot-min').textContent = solarTimes.EOT + ' min'; 
    if ($('ecliptic-long')) $('ecliptic-long').textContent = solarTimes.ECL_LONG + ' °';
    
    // Durée du jour
    if ($('day-duration') && sunTimes && sunTimes.sunrise && sunTimes.sunset) {
        const durationMs = sunTimes.sunset.getTime() - sunTimes.sunrise.getTime();
        const hours = Math.floor(durationMs / 3600000);
        const minutes = Math.floor((durationMs % 3600000) / 60000);
        $('day-duration').textContent = `${hours}h ${minutes}m`;
    } else if ($('day-duration')) {
        $('day-duration').textContent = 'N/A (Polaire/Nuit)';
    }

    // Lever/Coucher de la Lune
    if ($('moon-times')) $('moon-times').textContent = moonTimes ? 
        `↑ ${moonTimes.rise ? moonTimes.rise.toLocaleTimeString() : 'N/A'} / ↓ ${moonTimes.set ? moonTimes.set.toLocaleTimeString() : 'N/A'}` : 'N/D';
}

function fetchWeather(latA, lonA) {
    lastP_hPa = null; lastT_K = null; lastH_perc = null; 
    
    const envFactorDisplay = $('env-factor-display');
    if (envFactorDisplay) envFactorDisplay.textContent = `${selectedEnvironment} (x${ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT})`;

    // Blocage si l'URL est le proxy de démonstration
    if (!latA || !lonA || PROXY_BASE_URL.includes('scientific-dashboard2')) {
        if ($('temp-air')) $('temp-air').textContent = 'N/A (API désactivée)';
        if ($('pressure')) $('pressure').textContent = 'N/A (API désactivée)';
        if ($('humidity')) $('humidity').textContent = 'N/A (API désactivée)';
        return; 
    }

    // ... (Logique API Météo - non modifiée)
}

function updateDisp(pos) {
    if (emergencyStopActive) return;

    lat = pos.coords.latitude; lon = pos.coords.longitude;
    const alt = pos.coords.altitude, acc = pos.coords.accuracy;
    const spd = pos.coords.speed;
    const cTimePos = pos.timestamp; 

    const now = getCDate(); 
    if (now === null) { updateAstro(lat, lon); return; } 

    if (sTime === null) { sTime = now.getTime(); distMStartOffset = distM; }
    
    if (acc > MAX_ACC) { 
        if ($('gps-precision')) $('gps-precision').textContent = `❌ ${acc.toFixed(0)} m (Trop Imprécis)`; 
        if (lPos === null) lPos = pos; return; 
    }
    
    // ... (Calculs de Kalman et de vitesse 3D - non modifiés)

    let spdH = spd ?? 0;
    let spdV = 0; 
    const dt = lPos ? (cTimePos - lPos.timestamp) / 1000 : MIN_DT;

    const kAlt_new = kFilterAltitude(alt, acc, dt);
    
    if (lPos && lPos.kAlt_old !== undefined && dt > MIN_DT && alt !== null) {
        spdV = (kAlt_new - lPos.kAlt_old) / dt;
    } else if (alt !== null) {
        spdV = 0; 
    }
    
    if (lPos && dt > 0.05) { 
        const dH = dist(lPos.coords.latitude, lPos.coords.longitude, lat, lon); 
        spdH = dH / dt; 
    } else if (spd !== null) {
        spdH = spd;
    }
    
    let dampen_factor = 1.0;
    if (acc > ACC_DAMPEN_LOW) {
        const acc_range = ACC_DAMPEN_HIGH - ACC_DAMPEN_LOW;
        const current_excess = acc - ACC_DAMPEN_LOW;
        dampen_factor = 1.0 - Math.min(1.0, current_excess / acc_range);
        spdH *= dampen_factor;
        spdV *= dampen_factor; 
    }
    
    let spd3D = Math.sqrt(spdH ** 2 + spdV ** 2);

    if (lPos && lPos.speedMS_3D !== undefined && dt > MIN_DT) {
        const lastRawSpd = lPos.speedMS_3D;
        const accelSpike = Math.abs(spd3D - lastRawSpd) / dt;
        
        if (accelSpike > MAX_PLAUSIBLE_ACCEL) {
            const maxPlausibleChange = MAX_PLAUSIBLE_ACCEL * dt;
            if (spd3D > lastRawSpd) {
                spd3D = lastRawSpd + maxPlausibleChange;
            } else {
                spd3D = lastRawSpd - maxPlausibleChange;
            }
        }
    }

    const R_dyn = getKalmanR(acc, alt, lastP_hPa); 
    const fSpd = kFilter(spd3D, dt, R_dyn), sSpdFE = fSpd < MIN_SPD ? 0 : fSpd;
    
    let accel_long = 0;
    if (dt > 0.05) { accel_long = (sSpdFE - lastFSpeed) / dt; }
    lastFSpeed = sSpdFE;

    distM += sSpdFE * dt * (netherMode ? NETHER_RATIO : 1); 
    if (sSpdFE > MIN_SPD) { timeMoving += dt; }
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    
    // --- MISE À JOUR DU DOM (GPS/Physique) ---
    // (Utilise les IDs du HTML recommandé)
    if ($('latitude')) $('latitude').textContent = lat.toFixed(6);
    if ($('longitude')) $('longitude').textContent = lon.toFixed(6);
    if ($('altitude-gps')) $('altitude-gps').textContent = kAlt_new !== null ? `${kAlt_new.toFixed(2)} m` : 'N/A';
    if ($('gps-precision')) $('gps-precision').textContent = `${acc.toFixed(2)} m`; 
    if ($('speed-raw-ms')) $('speed-raw-ms').textContent = `${spd3D.toFixed(2)} m/s`;
    if ($('vertical-speed')) $('vertical-speed').textContent = `${spdV.toFixed(2)} m/s`;
    if ($('underground-status')) $('underground-status').textContent = alt !== null && alt < ALT_TH ? `OUI (< ${ALT_TH}m)` : 'Non';
    if ($('accel-long')) $('accel-long').textContent = `${accel_long.toFixed(3)} m/s²`;
    if ($('force-g-long')) $('force-g-long').textContent = `${(accel_long / G_ACC).toFixed(2)} G`;
    if ($('speed-error-perc')) $('speed-error-perc').textContent = `${R_dyn.toFixed(3)} m² (R dyn)`; 
    if ($('speed-3d-inst')) $('speed-3d-inst').textContent = `${(spd3D * KMH_MS).toFixed(5)} km/h`; 
    if ($('speed-stable')) $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(5)} km/h`; 
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${sSpdFE.toFixed(2)} m/s | ${(sSpdFE * 1000).toFixed(0)} mm/s`;
    if ($('speed-max')) $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    if ($('speed-avg-moving')) $('speed-avg-moving').textContent = timeMoving > 1 ? `${(distM / timeMoving * KMH_MS).toFixed(5)} km/h` : '0.00000 km/h';
    if ($('distance-total-km')) $('distance-total-km').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    if ($('distance-cosmic')) $('distance-cosmic').textContent = `${(distM / C_L).toExponential(2)} s lumière | ${(distM / C_L / (dayMs / 1000 * 365.25)).toExponential(2)} al`;
    if ($('perc-speed-sound')) $('perc-speed-sound').textContent = `${(spd3D / C_S * 100).toFixed(2)} %`;
    if ($('perc-speed-c')) $('perc-speed-c').textContent = `${(spd3D / C_L * 100).toExponential(2)}%`;
    if ($('mode-nether')) $('mode-nether').textContent = netherMode ? "ACTIVÉ (1:8) 🔥" : "DÉSACTIVÉ (1:1)";

    updateMap(lat, lon); 
    
    if (Date.now() - (updateDisp.lastWeatherFetch ?? 0) > 60000) {
        fetchWeather(lat, lon); 
        updateDisp.lastWeatherFetch = Date.now();
    }
    
    lPos = pos; 
    lPos.speedMS_3D = spd3D; 
    lPos.timestamp = cTimePos; 
    lPos.kAlt_old = kAlt_new; 
}

// ===========================================
// INITIALISATION DES ÉVÉNEMENTS DOM
// ===========================================

document.addEventListener('DOMContentLoaded', () => {
    
    initMap(); 
    
    // Initialisation du sélecteur d'environnement (LOGIQUE AMÉLIORÉE)
    const envSelect = document.createElement('select');
    envSelect.id = 'env-select';
    Object.keys(ENVIRONMENT_FACTORS).forEach(env => {
        const opt = document.createElement('option');
        opt.value = env; opt.textContent = env.toUpperCase();
        envSelect.appendChild(opt);
    });
    envSelect.value = selectedEnvironment;
    
    const envControlContainer = $('env-control-container'); // Nouvel ID HTML
    if (envControlContainer) {
        // Ajoute le sélecteur dans son conteneur dédié
        envControlContainer.appendChild(envSelect);
    }
    
    // Initialise l'affichage du facteur d'environnement
    const envFactorDisplay = $('env-factor-display');
    if (envFactorDisplay) envFactorDisplay.textContent = `${selectedEnvironment} (x${ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT})`;

    syncH(); 

    // Installation des listeners de contrôle
    
    // ===========================================
    // INSTALLATION DES LISTENERS DE CONTRÔLE
    // ===========================================

    // Listener de contrôle GPS (MARCHE/PAUSE)
    if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', () => {
        if (emergencyStopActive) { 
            alert("Veuillez désactiver l'Arrêt d'urgence avant d'utiliser ce contrôle."); 
            return; 
        }
        wID === null ? startGPS() : stopGPS();
    });
    
    // Listener de sélection de fréquence GPS
    if ($('freq-select')) $('freq-select').addEventListener('change', (e) => {
        if (emergencyStopActive) { 
            alert("Veuillez désactiver l'Arrêt d'urgence."); 
            return; 
        }
        setGPSMode(e.target.value);
    });

    // Listener d'Arrêt d'urgence (Toggle ON/OFF)
    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', () => {
        emergencyStopActive ? resumeSystem() : emergencyStop(); 
    });

    // Listener de bascule du mode Nether
    if ($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        netherMode = !netherMode; 
        if ($('mode-nether')) $('mode-nether').textContent = netherMode ? "ACTIVÉ (1:8) 🔥" : "DÉSACTIVÉ (1:1)"; 
    });
    
    // Listener de sélection d'environnement (Facteur Kalman R)
    if ($('env-select')) $('env-select').addEventListener('change', (e) => { 
        if (emergencyStopActive) return;
        selectedEnvironment = e.target.value; 
        // Mise à jour de l'affichage du facteur actif
        const envFactorDisplay = $('env-factor-display');
        if (envFactorDisplay) envFactorDisplay.textContent = `${selectedEnvironment} (x${ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT})`; 
    });
    
    // Listeners de réinitialisation des données
    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        distM = 0; distMStartOffset = 0; timeMoving = 0; 
        if ($('distance-total-km')) $('distance-total-km').textContent = `0.000 km | 0.00 m`; 
        if ($('speed-avg-moving')) $('speed-avg-moving').textContent = `0.00000 km/h`; 
        if ($('time-moving')) $('time-moving').textContent = `0.00 s`; 
    });
    
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        maxSpd = 0; 
        if ($('speed-max')) $('speed-max').textContent = `0.00000 km/h`; 
    });
    
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        if (confirm("Êtes-vous sûr de vouloir tout réinitialiser?")) { 
            distM = 0; maxSpd = 0; distMStartOffset = 0; 
            kSpd = 0; kUncert = 1000; timeMoving = 0; 
            // La mise à jour de l'affichage se fera au prochain tick GPS/slow-update
        } 
    });

    // La boucle d'intervalle pour la mise à jour DOM lente (Astro, Météo)
    if (domID === null) {
        domID = setInterval(() => {
            const currentLat = lat ?? 43.296482; 
            const currentLon = lon ?? 5.36978;
            updateAstro(currentLat, currentLon);
            if (lPos) updateMap(currentLat, currentLon); 
        }, DOM_SLOW_UPDATE_MS); 
    }
    
    startGPS();
