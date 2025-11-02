// =================================================================
// FICHIER JS PARTIE 1/2 : gnss-dashboard-part1.js
// Contient constantes, variables d'état, UTC/NTP et Kalman.
// =================================================================

// --- CLÉS D'API & PROXY VERCEL ---
// NOTE: L'URL du proxy Vercel est essentielle pour contourner les problèmes CORS avec l'API OpenWeatherMap.
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
// Vitesse minimale pour le mouvement (0.05 m/s = 0.18 km/h)
const MAX_ACC = 200, MIN_SPD = 0.05, ALT_TH = -50; 
const NETHER_RATIO = 8; 

// Filtre Anti-Spike de vitesse 3D (2G max d'accélération)
const MAX_PLAUSIBLE_ACCEL = 20.0; 

// PARAMÈTRES POUR LE FILTRE DE KALMAN D'ALTITUDE
const Q_ALT_NOISE = 0.1; // Bruit de processus de l'altitude
const R_ALT_MIN = 0.1;  // Erreur de mesure minimale de l'altitude

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
let kAlt = null;      // Altitude filtrée par Kalman
let kAltUncert = 10;  // Incertitude de l'altitude

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
    
    // R est basé sur la précision GPS (acc), mais avec un minimum R_ALT_MIN
    const R = Math.max(R_ALT_MIN, acc); 
    const Q = Q_ALT_NOISE * dt; 
    
    // Prédiction
    let pAlt = kAlt === null ? nAlt : kAlt; 
    let pUnc = kAltUncert + Q; 
    
    // Correction
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
    
    // (Cette partie est commentée pour éviter une erreur car P_hPa peut être null dans part2.js)
    /*
    if (P_hPa !== null) {
        const pressureFactor = 1.0 + (1013.25 - P_hPa) / 1013.25 * 0.1;
        R *= Math.max(1.0, pressureFactor);
    }
    */
    
    R *= envFactor;
    if (alt < ALT_TH) { R *= 2.0; } 

    R = Math.max(R_MIN, Math.min(R_MAX, R));
    
    return R;
                                          }
// =====
// =================================================================
// FICHIER JS PARTIE 2/2 : gnss-dashboard-part2.js (CORRIGÉ)
// =================================================================

// ... (fonctions toDays, solarMeanAnomaly, eclipticLongitude, getMinecraftTime restent inchangées)

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
    
    // Temps Solaire Moyen (Local)
    const mst_ms_raw = (msSinceMidnightUTC + mst_offset_ms);
    // Correction: Assure une valeur positive
    const mst_ms = (mst_ms_raw % dayMs + dayMs) % dayMs; 
    
    // Temps Solaire Vrai (TST)
    const eot_ms = eot_min * 60000;
    const tst_ms_raw = mst_ms + eot_ms;
    // Correction: Assure une valeur positive
    const tst_ms = (tst_ms_raw % dayMs + dayMs) % dayMs; 
    
    const toTimeString = (ms) => {
        let h = Math.floor(ms / 3600000);
        let m = Math.floor((ms % 3600000) / 60000);
        let s = Math.floor((ms % 60000) / 1000);
        // Note: La fonction est maintenant sûre car 'ms' est garanti d'être positif.
        return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
    };
    return { 
        TST: toTimeString(tst_ms), 
        MST: toTimeString(mst_ms), 
        EOT: eot_min.toFixed(3),
        ECL_LONG: (L * R2D).toFixed(2)
    };
}

function updateAstro(latA, lonA) {
    // ... (Logique SunCalc inchangée)
    // Remplacement des IDs pour correspondre aux libellés du tableau de bord:
    // $('time-solar-true') -> Heure Solaire Vraie (TST)
    // $('culmination-lsm') -> Temps Solaire Moyen Local (LSM)
    // $('eot-min') -> Équation du Temps (EOT)
    // $('noon-solar') -> Midi Solaire Local (UTC)
    
    const now = getCDate(); 
    // ... (check now === null, etc.)

    // ... (Calculs SunCalc et solarTimes)
    
    // Mise à jour de l'affichage de l'heure locale et de la date (qui était N/A)
    $('local-time').textContent = now.toLocaleTimeString('fr-FR', { timeZone: 'UTC', hour12: false });
    if ($('date-display')) $('date-display').textContent = now.toLocaleDateString();

    // ... (Affichage temps écoulé, temps de mouvement, temps Minecraft)

    // Mise à jour des valeurs Astro corrigées
    if ($('time-solar-true')) $('time-solar-true').textContent = solarTimes.TST;
    if ($('culmination-lsm')) $('culmination-lsm').textContent = solarTimes.MST;
    if ($('sun-elevation')) $('sun-elevation').textContent = sunPos ? `${(sunPos.altitude * R2D).toFixed(2)} °` : 'N/A';
    // ... (Autres valeurs SunCalc)
    if ($('eot-min')) $('eot-min').textContent = solarTimes.EOT + ' min'; // Corrected ID check
    if ($('ecliptic-long')) $('ecliptic-long').textContent = solarTimes.ECL_LONG + ' °';
    
    // ... (Logique Durée du Jour et Moon Times)
}

function fetchWeather(latA, lonA) {
    lastP_hPa = null; lastT_K = null; lastH_perc = null; 
    
    // Alerte sur l'API désactivée, comme indiqué dans le diagnostic
    if (!latA || !lonA || PROXY_BASE_URL.includes('scientific-dashboard2')) {
        if ($('env-factor-display')) $('env-factor-display').textContent = `${selectedEnvironment} (x${ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT})`;
        if ($('temp-air')) $('temp-air').textContent = 'N/A (API désactivée)';
        if ($('pressure')) $('pressure').textContent = 'N/A (API désactivée)';
        if ($('humidity')) $('humidity').textContent = 'N/A (API désactivée)';
        return; 
    }
    // ... (Logique API Météo)
}

document.addEventListener('DOMContentLoaded', () => {
    // ... (Logique de création du sélecteur d'environnement)
    // Mise à jour de la logique DOM pour le sélecteur d'environnement pour être moins agressive et utiliser les nouveaux IDs
    
    const envSelect = document.createElement('select');
    envSelect.id = 'env-select';
    Object.keys(ENVIRONMENT_FACTORS).forEach(env => {
        const opt = document.createElement('option');
        opt.value = env; opt.textContent = env.toUpperCase();
        envSelect.appendChild(opt);
    });
    envSelect.value = selectedEnvironment;
    
    const envControlContainer = $('env-control');
    if (envControlContainer) {
        envControlContainer.innerHTML = ''; // Vide l'ancien contenu
        envControlContainer.appendChild(envSelect);
    }
    
    // ... (Installation des listeners)

    if ($('env-select')) $('env-select').addEventListener('change', (e) => { 
        if (emergencyStopActive) return;
        selectedEnvironment = e.target.value; 
        // Met à jour le champ d'affichage dynamique de la valeur
        if ($('env-factor-display')) $('env-factor-display').textContent = `${selectedEnvironment} (x${ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT})`; 
    });

    // Initialise l'affichage du facteur d'environnement
    if ($('env-factor-display')) $('env-factor-display').textContent = `${selectedEnvironment} (x${ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT})`;

    // ... (Reste de la logique d'initialisation)
});
