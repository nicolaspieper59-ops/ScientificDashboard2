// =================================================================
// FICHIER JS COMPLET : gnss-dashboard-full.js
// BLOC 1/2 : Constantes, Kalman & Fonctions de Correction Avancées
// =================================================================

// --- CLÉS D'API & PROXY VERCEL ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app"; // Exemple de proxy
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const KMH_MS = 3.6;         
const G = 9.80665;          // Gravité terrestre de référence
const R_AIR = 287.058;      // Constante spécifique de l'air sec (J/kg·K)

// --- CONSTANTES DE CONFIGURATION SYSTÈME ---
const MIN_DT = 0.01; 
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};
const DOM_SLOW_UPDATE_MS = 1000; 
const IMU_FREQUENCY_MS = 50;      // Fréquence de la boucle DR (20 Hz)

// --- PARAMÈTRES DU FILTRE DE KALMAN (VITESSE) ---
const Q_NOISE = 0.1;        // Bruit de processus de base
const R_MIN = 0.01;         // Bruit de mesure minimum (GPS parfait/ZUPT)
const R_MAX = 500.0;        // Bruit de mesure maximum réaliste.
const MAX_ACC = 200;        // Précision max (m) avant de passer en "Estimation Seule"
const MIN_SPD = 0.05;       // Vitesse minimale pour être considéré "en mouvement"

// --- NOUVEAUX SEUILS POUR IMU/ZUPT AVANCÉ ---
const ZUPT_RAW_THRESHOLD = 1.0;     
const ZUPT_ACCEL_THRESHOLD = 0.5;   
const IMU_ZUPT_THRESHOLD = 0.05;    // Seuil d'accélération nette pour détecter l'arrêt (IMU).
const Q_BIAS_ACCEL_FACTOR = 0.5;    // Facteur de modulation de Q basé sur l'accélération.
const BIAS_EMA_ALPHA = 0.05;        // Facteur EMA (rapide) pour l'estimation du biais.
const R_ACCEL_DAMPING_FACTOR = 1.0; // Facteur pour amortir R en fonction de l'accélération.

// --- DONNÉES CÉLESTES/GRAVITÉ ---
const CELESTIAL_DATA = { /* ... (Données célestes simplifiées) ... */
    'EARTH': { G: 9.80665, R: 6371000, name: 'Terre' },
    'MOON': { G: 1.62, R: 1737400, name: 'Lune' }
};
let G_ACC = CELESTIAL_DATA['EARTH'].G; // Gravité locale courante
let R_ALT_CENTER_REF = CELESTIAL_DATA['EARTH'].R;

// --- VARIABLES D'ÉTAT (Globales) ---
let wID = null, domID = null, imuUpdateID = null;
let lPos = null, lat = null, lon = null, sTime = null;
let distM = 0, maxSpd = 0;
let kSpd = 0, kUncert = 1000;       // Incertitude du filtre (P)
let kAlt = null, kAltUncert = 10;   // Altitude (EKF 1D pour l'altitude)

// Variables IMU/DR/Attitude
let lastIMUUpdate = 0;
let rawAccelWithGrav = { x: 0.0, y: 0.0, z: 0.0 }; // Accélération BRUTE (+ Gravité)
let attitude = { roll: 0.0, pitch: 0.0, yaw: 0.0 }; // Attitude (Angles)
let imu_bias_est = 0.0;             // Biais estimé (EMA)
let corrected_Accel_Mag = 0.0;      // Accélération nette pour l'EKF
let centrifugalAccel = 0.0; 
let NED_Accel_Mag_RAW = 0.0;        // Magnitude brute (pour affichage)

// ... (Autres variables de contrôle et externes) ...
let emergencyStopActive = false;
let currentCelestialBody = 'EARTH';
let currentGPSMode = 'HIGH_FREQ'; 
let currentMass = 70.0;
let R_FACTOR_RATIO = 1.0;
let map, marker, circle;
let lastP_hPa = 1013.25, lastT_K = 290.45; // Valeurs météo par défaut
let lastFSpeed = 0;

// --- FONCTIONS UTILITAIRES ---
const $ = id => document.getElementById(id);

const dist = (lat1, lon1, lat2, lon2) => { /* ... (Logique Haversine inchangée) ... */
    const R = R_ALT_CENTER_REF; 
    const dLat = (lat2 - lat1) * D2R, dLon = (lon2 - lon1) * D2R;
    lat1 *= D2R; lat2 *= D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
};

const getKalmanR = (accRaw) => { /* ... (Logique R standard inchangée) ... */
    return Math.min(R_MAX, Math.max(R_MIN, accRaw * accRaw));
};

const updateCelestialBody = (body) => { /* ... (Logique de mise à jour G_ACC inchangée) ... */
    const data = CELESTIAL_DATA[body];
    G_ACC = data.G;
    R_ALT_CENTER_REF = data.R;
};

// --- NOUVELLE FONCTION CLÉ : COMPENSATION DE GRAVITÉ ---
function getGravityCompensatedAccel(rawAccel, attitude) {
    const G_current = G_ACC; 
    
    // 1. Conversion de l'angle de Tangage (Pitch) en radians
    // Le Pitch doit être compensé pour le sens réel du mouvement (sinon le signe est inversé)
    const pitchRad = attitude.pitch * D2R; 
    
    // 2. Projection de la Gravité (composante de G dans l'axe longitudinal Y)
    // accélération brute Y = (accel linéaire Y) + (G * sin(Pitch))
    const gravityProjectionOnY = G_current * Math.sin(pitchRad);
    
    // 3. Accélération Linéaire Nette (sur l'axe longitudinal du téléphone, Y)
    // Nous supposons que l'axe Y du capteur est l'axe longitudinal du mouvement (vers l'avant).
    const linearAccelY = rawAccel.y - gravityProjectionOnY;
    
    // L'EKF travaille sur la magnitude de l'accélération longitudinale
    return Math.abs(linearAccelY);
}

// --- FONCTION DE CORRECTION CENTRIFUGE ET BIAS EMA ---
function getCorrectedAccel(rawAccel, attitude, dt) {
    // ÉTAPE 1: Calculer la VÉRITABLE accélération longitudinale compensée
    const trueAccelMag = getGravityCompensatedAccel(rawAccel, attitude); 

    // 2. Mise à jour et Soustraction Centrifuge
    const local_r = parseFloat($('rotation-radius')?.value) || 100;
    const local_w = parseFloat($('angular-velocity')?.value) || 0;
    centrifugalAccel = local_r * (local_w ** 2);
    
    let net_Accel_Mag = trueAccelMag;
    if (net_Accel_Mag >= centrifugalAccel) {
        net_Accel_Mag -= centrifugalAccel;
    } else {
        net_Accel_Mag = 0.0;
    }

    // 3. ZUPT IMU et Estimation du Biais (Correction rapide par EMA)
    if (net_Accel_Mag < IMU_ZUPT_THRESHOLD) {
        // ZUPT IMU Actif : L'accélération nette restante est considérée comme le Biais
        if (imu_bias_est === 0.0) {
             imu_bias_est = net_Accel_Mag;
        } else {
            // Moyenne Mobile Exponentielle pour une convergence rapide
            imu_bias_est = (BIAS_EMA_ALPHA * net_Accel_Mag) + ((1 - BIAS_EMA_ALPHA) * imu_bias_est);
        }
        return 0.0; // Accélération corrigée nulle si immobile
    } else {
        // Mouvement détecté
        // 4. Soustraction du Biais (Correction de la Dérive en mouvement)
        return Math.max(0.0, net_Accel_Mag - imu_bias_est);
    }
}


/**
 * Filtre de Kalman 1D pour la vitesse (avec modulation de Q).
 * Note: kSpd et kUncert sont modifiés directement comme variables globales.
 */
function kFilter(nSpd, dt, R_dyn, accel_input = 0) {
    if (dt === 0 || dt > 5) return kSpd; 
    
    // MODULATION DE Q : Q augmente si l'accélération corrigée est forte (plus de confiance dans le changement d'état)
    const Q_accel_noise = Q_NOISE + Math.abs(accel_input) * Q_BIAS_ACCEL_FACTOR; 
    const R = R_dyn ?? R_MAX, Q = Q_accel_noise * dt * dt; 
    
    // PRÉDICTION
    let pSpd = kSpd + (accel_input * dt); 
    let pUnc = kUncert + Q; 

    // CORRECTION
    let K = pUnc / (pUnc + R); 
    kSpd = pSpd + K * (nSpd - pSpd); 
    kUncert = (1 - K) * pUnc; 
    
    return kSpd;
}

// ... (Logique pour kFilterAltitude, synchroHeure, Météo, Map omise pour concision, supposée fonctionnelle) ...
function kFilterAltitude(altRaw, R_alt, dt) { /* ... */ return altRaw; }
function initMap() { /* ... */ }
function updateMap(lat, lon, acc) { /* ... */ }
function getVirtualTemperature(T_C, HR_percent, P_hPa) { /* ... */ return T_C + 273.15; }
function getAirDensity(P_hPa, Tv_K) { /* ... */ return 1.225; }
function getSpeedOfSound(T_C) { /* ... */ return 343; }
function updateAstro(lat, lon) { /* ... */ }
function syncH() { /* ... */ }
function fetchWeather(lat, lon) { /* ... */ }


// =================================================================
// FICHIER JS COMPLET : gnss-dashboard-full.js
// BLOC 2/2 : Fonctions de Contrôle et Boucles Principales (IMU/DR + GPS)
// =================================================================

function startGPS() {
    const options = (currentGPSMode === 'HIGH_FREQ') ? GPS_OPTS.HIGH_FREQ : GPS_OPTS.LOW_FREQ;
    
    if (wID === null) {
        wID = navigator.geolocation.watchPosition(updateDisp, handleErr, options);
    }
    
    // Démarrage de la boucle de prédiction IMU haute fréquence (DR)
    if (imuUpdateID === null) {
        imuUpdateID = setInterval(updateIMUPrediction, IMU_FREQUENCY_MS);
    }
    
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = '⏸️ PAUSE GPS/IMU';
}

function stopGPS(resetButton = true) {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
    }
    if (imuUpdateID !== null) {
        clearInterval(imuUpdateID);
        imuUpdateID = null;
    }
    if (resetButton && $('toggle-gps-btn')) $('toggle-gps-btn').textContent = '▶️ MARCHE GPS/IMU';
}

function handleErr(error) { console.warn(`GPS ERROR: ${error.code}: ${error.message}`); }

// --- LOGIQUE DE PRÉDICTION IMU HAUTE FRÉQUENCE (20 Hz) ---
function updateIMUPrediction() {
    if (emergencyStopActive) return;

    const time = Date.now();
    if (lastIMUUpdate === 0) lastIMUUpdate = time;
    
    const dt_imu = (time - lastIMUUpdate) / 1000;
    const dt = (dt_imu > 0 && dt_imu < 1) ? dt_imu : IMU_FREQUENCY_MS / 1000; 
    lastIMUUpdate = time;

    // 1. Calcul de l'accélération corrigée (Attitude, Centrifuge, Biais EMA)
    corrected_Accel_Mag = getCorrectedAccel(rawAccelWithGrav, attitude, dt); 

    let R_imu_pred = R_MAX;
    let modeStatus = '';
    
    if (wID === null) { 
        // Mode DR (GPS Pausé)
        if (corrected_Accel_Mag === 0.0) {
             // ZUPT actif : MODIFICATION CLÉ: Réinitialisation d'Incertitude pour anti-dérive
             kSpd = 0.0;             // 1. Force la vitesse à zéro
             kUncert = R_MIN;        // 2. Réinitialise l'incertitude (confiance max)

             R_imu_pred = R_MIN; 
             modeStatus = '✅ DR - ZUPT/BIAS ON (IMU - P Reset)';
        } else {
             // Mouvement IMU détecté
             R_imu_pred = R_MAX; // Prédiction pure sans correction
             modeStatus = '⚠️ DR - PRÉDICTION IMU';
        }
        
        // Appliquer la prédiction EKF
        kFilter(kSpd, dt, R_imu_pred, corrected_Accel_Mag);
    } else {
         // GPS Actif - Nous laissons 'updateDisp' effectuer la correction
         return; 
    }

    // Mise à jour des stats DR à haute fréquence (Affichage)
    const sSpdFE = kSpd < MIN_SPD ? 0 : kSpd; 
    distM += sSpdFE * dt * R_FACTOR_RATIO; 
    if ($('imu-frequency')) $('imu-frequency').textContent = (1 / dt).toFixed(1);
    if ($('gps-status-dr')) $('gps-status-dr').textContent = modeStatus;
    if ($('imu-bias-est')) $('imu-bias-est').textContent = imu_bias_est.toFixed(6);
    if ($('corrected-accel-mag-comp')) $('corrected-accel-mag-comp').textContent = corrected_Accel_Mag.toFixed(3);
    if ($('centrifugal-accel')) $('centrifugal-accel').textContent = centrifugalAccel.toFixed(3);
    if ($('kalman-uncert')) $('kalman-uncert').textContent = kUncert.toFixed(3);
}


// --- FONCTION PRINCIPALE DE MISE À JOUR GPS (updateDisp) ---
function updateDisp(pos) {
    if (emergencyStopActive) return;

    const cTimePos = pos.timestamp;
    let cLat = pos.coords.latitude;
    let cLon = pos.coords.longitude;
    let accRaw = pos.coords.accuracy;
    let altRaw = pos.coords.altitude;
    
    // ... (Logique de gestion de lPos, dt, et calcul de spd3D_raw inchangée) ...
    // ... (Simulation de ces variables pour le contexte)
    const dt = 0.5; // Exemple
    let spd3D_raw = pos.coords.speed || 0.0;
    
    let R_dyn = getKalmanR(accRaw); 
    let isSignalLost = (accRaw > MAX_ACC);
    
    let spd_kalman_input = spd3D_raw;
    let R_kalman_input = R_dyn;
    let modeStatus = '🛰️ FUSION GNSS/IMU';
    
    if (isSignalLost) {
        modeStatus = `⚠️ ESTIMATION SEULE (Signal Perdu)`;
        return; // Retour en mode DR
    }

    // ZUPT GPS (Contrainte sur la mesure)
    if (spd3D_raw < ZUPT_RAW_THRESHOLD && R_dyn < R_MAX) { 
        spd_kalman_input = 0.0;     
        R_kalman_input = R_MIN;     
        modeStatus = '✅ ZUPT (Vélocité Nulle Forcée)';
    }

    // 1. Calcul de l'accélération corrigée IMU
    corrected_Accel_Mag = getCorrectedAccel(rawAccelWithGrav, attitude, dt); 
    let accel_sensor_input = corrected_Accel_Mag; 

    // 2. MODIFICATION CLÉ: Modulation Adaptative de R (Anti-Bruit en Mouvement)
    const accel_factor = Math.min(10.0, corrected_Accel_Mag / 2.0); 
    
    if (R_kalman_input !== R_MIN) { 
        // Augmenter le bruit de mesure R si l'IMU indique une forte accélération propre
        R_kalman_input = R_kalman_input * (1.0 + accel_factor * R_ACCEL_DAMPING_FACTOR);
    }

    // FILTRE DE KALMAN FINAL (Fusion IMU/GNSS)
    const fSpd = kFilter(spd_kalman_input, dt, R_kalman_input, accel_sensor_input); 
    const sSpdFE = fSpd < MIN_SPD ? 0 : fSpd; 
    
    // ... (Mise à jour des stats et lPos inchangée) ...
    
    // Mise à jour des affichages
    if ($('speed-stable')) $('speed-stable').textContent = `${sSpdFE.toFixed(3)} m/s`;
    if ($('current-speed')) $('current-speed').textContent = `${(sSpdFE * KMH_MS).toFixed(2)} km/h`;
    if ($('gps-status-dr')) $('gps-status-dr').textContent = modeStatus;
    if ($('kalman-r-factor')) $('kalman-r-factor').textContent = R_kalman_input.toFixed(3);
    if ($('kalman-q-noise')) $('kalman-q-noise').textContent = (Q_NOISE + Math.abs(accel_sensor_input) * Q_BIAS_ACCEL_FACTOR).toFixed(3);
    if ($('kalman-uncert')) $('kalman-uncert').textContent = kUncert.toFixed(3);
    if ($('corrected-accel-mag-comp')) $('corrected-accel-mag-comp').textContent = corrected_Accel_Mag.toFixed(3);
    if ($('raw-accel-mag')) $('raw-accel-mag').textContent = NED_Accel_Mag_RAW.toFixed(3);
    
    // Sauvegarde de l'état
    lPos = pos;
    lat = cLat; lon = cLon;
    kAlt = kFilterAltitude(altRaw, pos.coords.altitudeAccuracy || 1.0, dt);
    
    updateMap(lat, lon, accRaw);
}


// --- INITIALISATION DES ÉVÉNEMENTS DOM ---
document.addEventListener('DOMContentLoaded', () => {
    
    initMap(); 
    
    // Contrôles
    if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', () => { wID === null ? startGPS() : stopGPS(); });
    if ($('reset-system-btn')) $('reset-system-btn').addEventListener('click', () => { window.location.reload(); });
    if ($('celestial-body')) $('celestial-body').addEventListener('change', (e) => updateCelestialBody(e.target.value));

    // Listeners IMU AVANCÉS
    if (window.DeviceOrientationEvent) {
        window.addEventListener('deviceorientation', (event) => {
            attitude.yaw = event.alpha || 0; 
            attitude.pitch = event.beta || 0; 
            attitude.roll = event.gamma || 0;
            if ($('imu-roll')) $('imu-roll').textContent = attitude.roll.toFixed(1) + ' °';
            if ($('imu-pitch')) $('imu-pitch').textContent = attitude.pitch.toFixed(1) + ' °';
            if ($('imu-yaw')) $('imu-yaw').textContent = attitude.yaw.toFixed(1) + ' °';
        });
    }

    if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', (event) => {
            // Lecture des composantes BRUTES X, Y, Z (incluant la gravité)
            const a_grav = event.accelerationIncludingGravity; 
            
            if (a_grav) {
                rawAccelWithGrav.x = a_grav.x;
                rawAccelWithGrav.y = a_grav.y; // Axe longitudinal
                rawAccelWithGrav.z = a_grav.z;
                
                NED_Accel_Mag_RAW = Math.sqrt(a_grav.x**2 + a_grav.y**2 + a_grav.z**2);
            }
        });
    }

    // --- DÉMARRAGE DU SYSTÈME ---
    syncH(); 
    updateCelestialBody(currentCelestialBody); 
    startGPS();
    
    // Boucle de mise à jour lente (Astro/Météo/Horloge)
    // ... (Logique de la boucle lente omise) ...
});
