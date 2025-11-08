// =================================================================
// FICHIER 1/3 : gnss-dashboard-part1.js
// CŒUR DU SYSTÈME (Constantes, Variables d'État, EKF, Physique)
// =================================================================

const $ = id => document.getElementById(id);

// --- PARTIE 1 : CONSTANTES GLOBALES (Physiques, GPS, Temps, Astro) ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;

// Constantes Physiques
const C_L = 299792458;          // Vitesse de la lumière (m/s)
const C_S = 343;                // Vitesse du son dans l'air (m/s)
const G_ACC_STD = 9.80665;      // Gravité standard de la Terre (m/s²)
const M_EARTH = 5.9722e24;      // Masse de la Terre (kg)
const G_CONST = 6.67430e-11;    // Constante gravitationnelle (N(m/kg)²)
const R_E = 6371000;            // Rayon moyen de la Terre (m)
const NETHER_RATIO = 8;         // Ratio Nether (1:8)

// Constantes de Conversion
const KMH_MS = 3.6; 
const KMS_MS = 1000; 
const NM_MS = 1e9;              
const AU_TO_M = 149597870700; 
const LIGHT_YEAR_TO_M = 9.461e15; 
const SEC_IN_MIN = 60;
const SEC_IN_HOUR = 3600;
const SEC_IN_DAY = 86400;
const SEC_IN_WEEK = 604800;
const SEC_IN_MONTH = 2629746;

// Constantes Temps / Astro
const MC_DAY_MS = 72 * 60 * 1000; // Durée d'un jour Minecraft en ms
const J1970 = 2440588, J2000 = 2451545; 
const dayMs = 1000 * 60 * 60 * 24;      
const MIN_DT = 0.01; 

// Paramètres EKF/GPS Avancés
const Q_ACCEL_NOISE = 0.05;     // Bruit de l'accéléromètre (process noise)
const R_MIN = 0.01;             // Plancher d'incertitude R (GPS)
const R_MAX = 50.0; 
const K_UNCERT_FLOOR = 0.01;    // Plancher d'incertitude EKF
const MIN_SPD = 0.05; 
const MAX_ACC = 200; 
const ALT_TH = -50; 
const ACC_DAMPEN_LOW = 5.0; 
const ACC_DAMPEN_HIGH = 50.0; 
const MAX_PLAUSIBLE_ACCEL = 20.0; 
const DOM_SLOW_UPDATE_MS = 1000; 
const P_RECORDS_KEY = 'gnss_precision_records';

// Endpoints
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app"; 
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc"; 


// --- PARTIE 2 : VARIABLES D'ÉTAT ---
let wID = null, domID = null, lPos = null, lat = null, lon = null, sTime = null;
let distM = 0, distMStartOffset = 0, maxSpd = 0;
let kSpd = 0, kUncert = 1000; 
let timeMoving = 0; 
let lServH = null, lLocH = null; 
let lastFSpeed = 0; 
let kAlt = null;      
let kAltUncert = 10;  
let initialAlt = null;

let currentGPSMode = 'HIGH_FREQ'; 
let emergencyStopActive = false;
let netherMode = false;
let selectedEnvironment = 'NORMAL'; 
let maxGForce = 0;

let lastP_hPa = null, lastT_K = null, lastH_perc = null; 

// Capteurs (IMU)
let a_sensor_longitudinal = 0; // m/s² (Capteur)
let a_sensor_vertical_imu = 0; // m/s² (Capteur)
let a_sensor_uncert = 0.5;     // Incertitude des capteurs (m/s²)

// Facteurs Environnementaux pour le Kalman
const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0 },
    'METAL': { R_MULT: 2.5 },      
    'FOREST': { R_MULT: 1.5 },     
    'CONCRETE': { R_MULT: 3.0 },   
};

// --- PARTIE 3 : FONCTIONS DE BASE ---

/** Calcule la distance horizontale (Haversine). */
const dist = (lat1, lon1, lat2, lon2) => {
    const R = R_E, dLat = (lat2 - lat1) * D2R, dLon = (lon2 - lon1) * D2R;
    lat1 *= D2R; lat2 *= D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
};

/** Calcule la gravité locale en fonction de l'altitude (m). */
function getGravityAtAltitude(altitude_m) {
    if (altitude_m === null) return G_ACC_STD;
    const distance_centre = R_E + altitude_m;
    return (G_CONST * M_EARTH) / (distance_centre * distance_centre);
}

// SYNCHRONISATION HORAIRE PAR SERVEUR (UTC/Atomique)
async function syncH() { 
    if ($('local-time')) $('local-time').textContent = 'Synchronisation UTC...';
    const localStartPerformance = performance.now(); 

    try {
        const response = await fetch(SERVER_TIME_ENDPOINT, { cache: "no-store", mode: "cors" });
        if (!response.ok) throw new Error(`Server time sync failed: ${response.statusText}`);
        
        const localEndPerformance = performance.now(); 
        const serverData = await response.json(); 
        
        const serverTimestamp = Date.parse(serverData.datetime); 
        const RTT = localEndPerformance - localStartPerformance;
        const latencyOffset = RTT / 2;

        lServH = serverTimestamp + latencyOffset; 
        lLocH = performance.now(); 
    } catch (error) {
        lServH = Date.now(); 
        lLocH = performance.now();
    }
}

/** Retourne l'heure synchronisée (précision RTT compensée en UTC). */
function getCDate() { 
    if (lServH === null || lLocH === null) { return null; }
    const offsetSinceSync = performance.now() - lLocH;
    return new Date(lServH + offsetSinceSync); 
}

// FILTRE DE KALMAN (EKF pour Vitesse)
/** * Applique le filtre de Kalman. 
 * Étape 1: PRÉDICTION (moteur: accélération capteur)
 * Étape 2: CORRECTION (mesure: vitesse GPS)
 */
function kFilter(nSpd_gps, dt, R_dyn, a_sensor_long, a_sensor_vert_imu) {
    if (dt === 0 || dt > 5) return kSpd; 
    
    // Accélération totale (horizontale + verticale) fournie par le capteur IMU pour la prédiction
    const a_total_sensor = Math.sqrt(a_sensor_long ** 2 + a_sensor_vert_imu ** 2);
    
    // 1. PRÉDICTION
    // La nouvelle vitesse est prédite à partir de l'ancienne vitesse et de l'accélération du capteur.
    let predictedSpd = kSpd + a_total_sensor * dt; 
    
    // Le bruit de processus (Q) est basé sur l'incertitude du capteur et l'intervalle de temps.
    const Q = Q_ACCEL_NOISE ** 2 * dt; 
    let pUnc = kUncert + Q; 

    // 2. CORRECTION
    // R est l'incertitude de la MESURE (GPS)
    const R = R_dyn ?? R_MAX; 
    const K = pUnc / (pUnc + R); 
    
    // Correction : Le GPS corrige la prédiction du capteur.
    kSpd = predictedSpd + K * (nSpd_gps - predictedSpd); 
    kUncert = (1 - K) * pUnc; 
    
    // Plancher d'incertitude EKF
    kUncert = Math.max(kUncert, K_UNCERT_FLOOR); 
    
    return kSpd;
}

/** Applique le filtre de Kalman à l'Altitude. */
function kFilterAltitude(nAlt, acc, dt) {
    if (nAlt === null) return kAlt;
    const R = Math.max(R_MIN, acc); 
    const Q = Q_ACCEL_NOISE * dt; // Utilisation d'un bruit similaire
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
// FICHIER 2A/3 : gnss-dashboard-part2a-astro.js
// Contient les fonctions de temps, Astro et la logique visuelle du cadran.
// NÉCESSITE gnss-dashboard-part1.js
// =================================================================

// ===========================================
// FONCTIONS ASTRO & TEMPS
// ===========================================

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

/** Calcule le Temps Solaire Vrai (TST). (Corrigé pour TST négatif) */
function getSolarTime(date, lon) {
    if (date === null || lon === null) return { TST: 'N/A', MST: 'N/A', EOT: 'N/D', ECL_LONG: 'N/D' };
    const d = toDays(date);
    const M = solarMeanAnomaly(d); 
    let L = eclipticLongitude(M); 
    
    // Correction Longitude Écliptique (enveloppement 0-360°)
    let L_wrapped = L % (2 * Math.PI);
    if (L_wrapped < 0) L_wrapped += 2 * Math.PI;

    const epsilon = D2R * (23.4393 - 0.000000356 * d); 
    let alpha = Math.atan2(Math.cos(epsilon) * Math.sin(L), Math.cos(L));
    if (alpha < 0) alpha += 2 * Math.PI; 
    const eot_rad = alpha - M - D2R * 102.9377 - Math.PI;
    const eot_min = eot_rad * 4 * R2D;
    const msSinceMidnightUTC = (date.getUTCHours() * 3600 + date.getUTCMinutes() * 60 + date.getUTCSeconds()) * 1000 + date.getUTCMilliseconds();
    const mst_offset_ms = lon * dayMs / 360; 
    const mst_ms = (msSinceMidnightUTC + mst_offset_ms + dayMs) % dayMs;
    const eot_ms = eot_min * 60000;
    
    // TST : Assure que le TST est toujours dans la plage positive (0 à 24h)
    const tst_ms = (mst_ms + eot_ms + dayMs) % dayMs; 
    
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
        ECL_LONG: (L_wrapped * R2D).toFixed(2)
    };
}
/** Calcule le temps Minecraft. */
function getMinecraftTime(date) {
    if (date === null) return { timeString: '00:00:00', ms: 0 };
    const msSinceMidnightUTC = date.getUTCHours() * 3600000 + date.getUTCMilliseconds() + date.getUTCMinutes() * 60000 + date.getUTCSeconds() * 1000;
    const timeRatio = (msSinceMidnightUTC % dayMs) / dayMs;
    const mcTimeMs = (timeRatio * MC_DAY_MS + MC_DAY_MS) % MC_DAY_MS; 
    const toTimeString = (ms) => {
        let h = Math.floor(ms / 3600000);
        let m = Math.floor((ms % 3600000) / 60000);
        let s = Math.floor((ms % 60000) / 1000);
        return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
    };
    return { timeString: toTimeString(mcTimeMs), ms: mcTimeMs };
}

/** * Convertit l'élévation Astro (0°=Horizon, 90°=Zenith) en angle de rotation visuel.
 * L'angle visuel doit être : Zenith (90°) = 12h (0° de rotation), Horizon (0°) = 3h/9h (90°/270° de rotation).
 */
function getAstroRotationAngle(elevation_rad) {
    const elevation_deg = elevation_rad * R2D;
    // Remap l'élévation de -90 à 90 en angle de 180° pour le côté visible du cadran (0° est le Zenith, 180° est le Nadir)
    let rotation = 90 - elevation_deg;
    
    // Le cadran HTML (CSS) est en 2D, on utilise donc une rotation standard.
    // L'angle 0° CSS correspond à 12h. L'angle 90° CSS correspond à 3h.
    // Zenih (90°) -> 0° CSS. Horizon (0°) -> 90° ou -90° CSS.
    return rotation;
}


function updateAstro(latA, lonA) {
    const now = getCDate(); 
    if (now === null) {
        if ($('local-time') && !$('local-time').textContent.includes('Synchronisation')) {
             $('local-time').textContent = 'Synchronisation...';
        }
        return;
    }
    
    // Utilisation de SunCalc pour l'élévation
    const sunPos = window.SunCalc ? SunCalc.getPosition(now, latA, lonA) : null;
    const moonPos = window.SunCalc ? SunCalc.getMoonPosition(now, latA, lonA) : null;
    const moonIllum = window.SunCalc ? SunCalc.getMoonIllumination(now) : null;
    const sunTimes = window.SunCalc ? SunCalc.getTimes(now, latA, lonA) : null;
    
    const solarTimes = getSolarTime(now, lonA);
    const mcTime = getMinecraftTime(now); 
    
    // --- MISE À JOUR DOM GÉNÉRALE ---
    $('local-time').textContent = now.toLocaleTimeString('fr-FR', { timeZone: 'UTC', hour12: false });
    if ($('date-display')) $('date-display').textContent = now.toLocaleDateString();
    if (sTime) {
        const timeElapsed = (now.getTime() - sTime) / 1000;
        $('time-elapsed').textContent = `${timeElapsed.toFixed(2)} s`;
        $('time-moving').textContent = `${timeMoving.toFixed(2)} s`;
        if ($('time-minecraft')) $('time-minecraft').textContent = mcTime.timeString; 
    }
    
    // --- MISE À JOUR DOM ASTRO ---
    if ($('time-solar-true')) $('time-solar-true').textContent = solarTimes.TST; 
    if ($('culmination-lsm')) $('culmination-lsm').textContent = solarTimes.MST;
    if ($('sun-elevation')) $('sun-elevation').textContent = sunPos ? `${(sunPos.altitude * R2D).toFixed(2)} °` : 'N/A';
    if ($('lunar-phase-perc')) $('lunar-phase-perc').textContent = moonIllum ? `${(moonIllum.fraction * 100).toFixed(1)} %` : 'N/A';
    if ($('noon-solar')) $('noon-solar').textContent = sunTimes && sunTimes.solarNoon ? sunTimes.solarNoon.toLocaleTimeString() : 'N/D';
    if ($('eot-min')) $('eot-min').textContent = solarTimes.EOT + ' min'; 
    if ($('ecliptic-long')) $('ecliptic-long').textContent = solarTimes.ECL_LONG + ' °';

    // --- LOGIQUE ASTRO VISUELLE AVEC ZENITH/HORIZON ---
    if ($('sun-element') && sunPos) {
        const sunRotationAngle = getAstroRotationAngle(sunPos.altitude);
        // Utilise l'azimut pour l'orientation latérale
        const azimuth_deg = sunPos.azimuth * R2D;
        const totalRotation = sunRotationAngle + azimuth_deg; 

        $('sun-element').style.transform = `rotate(${totalRotation}deg)`;
        
        // Simule l'opacité selon l'élévation pour montrer le lever/coucher
        $('sun-element').style.opacity = Math.max(0.2, Math.min(1.0, (sunPos.altitude * R2D + 15) / 30)).toFixed(2);
    }
    
    if ($('moon-element') && moonPos) {
        const moonRotationAngle = getAstroRotationAngle(moonPos.altitude);
        const azimuth_deg = moonPos.azimuth * R2D;
        const totalRotation = moonRotationAngle + azimuth_deg; 

        $('moon-element').style.transform = `rotate(${totalRotation}deg)`;
        $('moon-element').style.opacity = Math.max(0.2, Math.min(1.0, (moonPos.altitude * R2D + 15) / 30)).toFixed(2);
    }
    
    // Logique du ciel Minecraft (basée sur l'heure MC)
    const mcMs = mcTime.ms;
    const body = document.body;
    const hour = Math.floor(mcMs / 3600000); 
    
    body.classList.remove('sky-day', 'sky-sunset', 'sky-night', 'sky-night-light');
    if (hour >= 6 && hour < 10) { 
        body.classList.add('sky-day');
    } else if (hour >= 10 && hour < 16) { 
        body.classList.add('sky-day');
    } else if (hour >= 16 && hour < 20) { 
        body.classList.add('sky-sunset');
    } else { 
        body.classList.add('sky-night');
    }
            }
// =================================================================
// FICHIER 3/3 : gnss-dashboard-part2b-logic.js
// Contient la logique principale de mise à jour GPS, Contrôles et DOM.
// NÉCESSITE gnss-dashboard-part1.js ET gnss-dashboard-part2a-astro.js.
// =================================================================

// ===========================================
// PERSISTANCE LOCALE (Max G-Force)
    
// =================================================================
// FICHIER 3/3 : gnss-dashboard-part2b-logic.js
// Contient la logique principale de mise à jour GPS, Contrôles et DOM.
// NÉCESSITE gnss-dashboard-part1.js ET gnss-dashboard-part2a-astro.js.
// =================================================================

// ===========================================
// PERSISTANCE LOCALE & CONTRÔLES (inchangé)
// ===========================================

function loadPrecisionRecords() {
    try {
        const records = JSON.parse(localStorage.getItem(P_RECORDS_KEY));
        if (records && records.maxGForce) {
            maxGForce = records.maxGForce;
            if ($('max-g-force')) $('max-g-force').textContent = `${maxGForce.toFixed(2)} G`;
        }
    } catch (e) {
        console.error("Erreur lors du chargement des records:", e);
    }
}

function savePrecisionRecords() {
    try {
        const records = { maxGForce: maxGForce };
        localStorage.setItem(P_RECORDS_KEY, JSON.stringify(records));
    } catch (e) {
        console.error("Erreur lors de la sauvegarde des records:", e);
    }
}

// (setGPSMode, startGPS, stopGPS, emergencyStop, resumeSystem, handleErr, fetchWeather)

// --- NOUVEAU : FONCTION DE SIMULATION CAPTEURS (IMU) ---

function getSensorData(dt) {
    // === SIMULATION : Pour un environnement réel, remplacer par l'API DeviceMotionEvent ===
    
    // SIMULATION DE BRUIT ET ACCÉLÉRATION/DÉCÉLÉRATION LÉGÈRE
    const noise_long = (Math.random() - 0.5) * 0.05; // Bruit longitudinal ±0.025 m/s²
    const noise_vert = (Math.random() - 0.5) * 0.02; // Bruit vertical ±0.01 m/s²
    
    let a_long = 0;
    
    // Si la vitesse EKF est élevée, simuler une décélération naturelle (résistance de l'air/friction)
    if (kSpd > MIN_SPD) {
        a_long = -kSpd * 0.05 + noise_long; 
    } else {
        a_long = noise_long * 0.1; // Bruit très faible à l'arrêt
    }
    
    // Mise à jour des variables globales pour la prochaine itération EKF et l'affichage DOM
    a_sensor_longitudinal = a_long; 
    a_sensor_vertical_imu = noise_vert; 

    return { 
        a_long: a_long, 
        a_vert: noise_vert 
    };
}


// ===========================================
// FONCTION PRINCIPALE DE MISE À JOUR GPS 
// ===========================================

function updateDisp(pos) {
    if (emergencyStopActive) return;

    lat = pos.coords.latitude; lon = pos.coords.longitude;
    const alt = pos.coords.altitude, acc = pos.coords.accuracy;
    const spd_gps_raw = pos.coords.speed; // Vitesse GPS brute (mesure)
    const cTimePos = pos.timestamp; 

    const now = getCDate(); 
    if (now === null) { updateAstro(lat, lon); return; } 

    if (sTime === null) { sTime = now.getTime(); distMStartOffset = distM; }
    
    if (initialAlt === null && alt !== null) {
        initialAlt = alt;
        if ($('altitude-gps')) $('altitude-gps').textContent = `${initialAlt.toFixed(2)} m (Initial)`;
    }
    
    if (acc > MAX_ACC) { 
        if ($('gps-precision')) $('gps-precision').textContent = `❌ ${acc.toFixed(0)} m (Trop Imprécis)`; 
        if (lPos === null) lPos = pos; return; 
    }
    
    // Calcul du DT
    const dt = lPos ? (cTimePos - lPos.timestamp) / 1000 : MIN_DT;

    // --- Étape 1 : DONNÉES CAPTEURS (IMU) ---
    const sensorData = getSensorData(dt);
    const a_long_imu = a_sensor_longitudinal;
    const a_vert_imu = a_sensor_vertical_imu;
    
    // --- Étape 2 : MESURE GPS (pour la correction EKF) ---
    let spdV = 0; 
    const kAlt_new = kFilterAltitude(alt, acc, dt);
    if (lPos && lPos.kAlt_old !== undefined && dt > MIN_DT && alt !== null) {
        spdV = (kAlt_new - lPos.kAlt_old) / dt;
    }
    let spdH_gps_calc = spd_gps_raw ?? 0;
    if (lPos && dt > 0.05) { 
        const dH = dist(lPos.coords.latitude, lPos.coords.longitude, lat, lon); 
        spdH_gps_calc = dH / dt; 
    }
    
    // Vitesse 3D GPS BRUTE (Mesure de correction)
    let spd3D_gps_mesure = Math.sqrt(spdH_gps_calc ** 2 + spdV ** 2);
    
    // Logique d'amortissement (pour la mesure GPS seulement)
    if (acc > ACC_DAMPEN_LOW) {
        const acc_range = ACC_DAMPEN_HIGH - ACC_DAMPEN_LOW;
        const current_excess = acc - ACC_DAMPEN_LOW;
        const dampen_factor = 1.0 - Math.min(1.0, current_excess / acc_range);
        spd3D_gps_mesure *= dampen_factor;
    }

    // --- Étape 3 : FILTRE DE KALMAN (Capteur + GPS) ---
    const R_dyn = getKalmanR(acc, alt, lastP_hPa); 
    // L'EKF est piloté par le capteur, corrigé par le GPS (spd3D_gps_mesure)
    const fSpd = kFilter(spd3D_gps_mesure, dt, R_dyn, a_long_imu, a_vert_imu); 
    const sSpdFE = fSpd < MIN_SPD ? 0 : fSpd;
    
    // Calculs de l'accélération (maintenant basée sur le capteur IMU)
    // L'accélération longitudinale est l'accélération IMU lissée (ou simplement l'accélération IMU brute)
    const accel_long = a_long_imu; 
    
    // Mise à jour des records Max
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    
    // Calculs de distance/temps
    distM += sSpdFE * dt * (netherMode ? NETHER_RATIO : 1); 
    if (sSpdFE > MIN_SPD) { timeMoving += dt; }
    
    // Calcule la gravité locale
    const localGravity = getGravityAtAltitude(kAlt_new);

    // Calcule et persiste la G-Force Max (basée sur l'accélération du capteur)
    const currentGForce = Math.abs(accel_long) / localGravity;
    if (currentGForce > maxGForce) {
        maxGForce = currentGForce;
        savePrecisionRecords(); 
    }

    // --- MISE À JOUR DU DOM (GPS/Physique) ---
    
    // Vitesse
    if ($('speed-3d-inst')) $('speed-3d-inst').textContent = `${(spd3D_gps_mesure * KMH_MS).toFixed(5)} km/h`; // GPS Mesure
    if ($('speed-stable')) $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(5)} km/h`; // EKF (Capteur + GPS)
    if ($('speed-stable-km-s')) $('speed-stable-km-s').textContent = `${(sSpdFE / KMS_MS).toFixed(5)} km/s`; 
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${sSpdFE.toFixed(2)} m/s | ${(sSpdFE * NM_MS).toExponential(2)} nm/s`;
    
    // Dynamique du Véhicule (Maintenant basé sur l'IMU)
    if ($('accel-long')) $('accel-long').textContent = `${accel_long.toFixed(3)} m/s²`;
    if ($('force-g-long')) $('force-g-long').textContent = `${(accel_long / localGravity).toFixed(2)} G`;
    if ($('accel-vertical-imu')) $('accel-vertical-imu').textContent = `${a_vert_imu.toFixed(3)} m/s²`;
    if ($('force-g-vertical')) $('force-g-vertical').textContent = `${(a_vert_imu / localGravity).toFixed(2)} G`;
    if ($('gravity-local')) $('gravity-local').textContent = `${localGravity.toFixed(5)} m/s²`;

    // Distance
    const totalDist = distM * (netherMode ? NETHER_RATIO : 1);
    if ($('distance-total-km')) $('distance-total-km').textContent = `${(totalDist / 1000).toFixed(3)} km | ${totalDist.toFixed(2)} m`;
    
    // Distances Cosmiques (inchangé)
    const distSecLight = totalDist / C_L;
    if ($('dist-s-light')) $('dist-s-light').textContent = distSecLight.toExponential(2) + ' s lumière';
    if ($('dist-min-light')) $('dist-min-light').textContent = (distSecLight / SEC_IN_MIN).toExponential(2) + ' min lumière';
    if ($('dist-h-light')) $('dist-h-light').textContent = (distSecLight / SEC_IN_HOUR).toExponential(2) + ' h lumière';
    if ($('dist-j-light')) $('dist-j-light').textContent = (distSecLight / SEC_IN_DAY).toExponential(2) + ' j lumière';
    if ($('dist-sem-light')) $('dist-sem-light').textContent = (distSecLight / SEC_IN_WEEK).toExponential(2) + ' sem lumière';
    if ($('dist-mois-light')) $('dist-mois-light').textContent = (distSecLight / SEC_IN_MONTH).toExponential(2) + ' mois lumière';
    if ($('dist-au-al')) $('dist-au-al').textContent = `${(totalDist / AU_TO_M).toExponential(2)} UA | ${(totalDist / LIGHT_YEAR_TO_M).toExponential(2)} al`;
    
    // ... (Reste des mises à jour DOM inchangé)
    
    // SAUVEGARDE DES VALEURS POUR LA PROCHAINE ITÉRATION
    lPos = pos; 
    lPos.speedMS_3D = spd3D_gps_mesure; 
    lPos.timestamp = cTimePos; 
    lPos.kAlt_old = kAlt_new; 
}


// ===========================================
// INITIALISATION DES ÉVÉNEMENTS DOM (inchangé)
// ===========================================

document.addEventListener('DOMContentLoaded', () => {
    
    // ... (Logique d'initialisation et Event Listeners inchangés)

    // Boucle de mise à jour lente DOM (Énergie, Puissance, Astro)
    if (domID === null) {
        domID = setInterval(() => {
            if (lPos) updateAstro(lPos.coords.latitude, lPos.coords.longitude);
            else updateAstro(null, null); 

            // --- Logique de la Masse (Énergie et Puissance) ---
            const speed_ms = Math.abs(kSpd);
            const massElement = document.getElementById('mass-input');
            const mass = massElement ? parseFloat(massElement.value) : 70; 
            
            // L'accélération pour la puissance est maintenant a_sensor_longitudinal (capteur)
            if ($('kinetic-energy')) $('kinetic-energy').textContent = `${(0.5 * mass * speed_ms * speed_ms).toFixed(2)} J`;
            if ($('mechanical-power')) $('mechanical-power').textContent = `${(mass * a_sensor_longitudinal * speed_ms).toFixed(2)} W`;
            // --- FIN : Logique de la Masse ---
            
        }, DOM_SLOW_UPDATE_MS); 
    }
});
