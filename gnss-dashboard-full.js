// =================================================================
// FICHIER JS COMPLET : gnss-dashboard-full.js
// BLOC 1/2 : Constantes, Kalman, Carte & Météo (CORRIGÉ IMU/Biais EMA)
// =================================================================

// --- CLÉS D'API & PROXY VERCEL ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const R_E_BASE = 6371000;   // Rayon terrestre moyen (m)
const KMH_MS = 3.6;         // Conversion m/s vers km/h
const C_S = 343;            // Vitesse du son (m/s)
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const R_AIR = 287.058;      // Constante spécifique de l'air sec (J/kg·K)
const G = 9.80665;          // Gravité terrestre de référence

// --- CONSTANTES DE TEMPS & CALENDRIER ---
const MC_DAY_MS = 72 * 60 * 1000; // Durée d'un jour Minecraft en ms
const J1970 = 2440588, J2000 = 2451545;
const dayMs = 1000 * 60 * 60 * 24;

// --- CONSTANTES DE CONFIGURATION SYSTÈME ---
const MIN_DT = 0.01; // Temps minimum (en sec) pour une mise à jour
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};
const DOM_SLOW_UPDATE_MS = 1000; // Rafraîchissement des données non-critiques (1 sec)
let lastMapUpdate = 0; 
const MAP_UPDATE_INTERVAL = 3000; // Rafraîchir la vue carte toutes les 3 sec si en mouvement
const IMU_FREQUENCY_MS = 50;      // NOUVEAU: 20 Hz (pour la boucle IMU)

// --- PARAMÈTRES DU FILTRE DE KALMAN (VITESSE) ---
const Q_NOISE = 0.1;        // Bruit de processus de base
const R_MIN = 0.01;         // Bruit de mesure minimum (GPS parfait)
const R_MAX = 500.0;        // Bruit de mesure maximum réaliste.
const MAX_ACC = 200;        // Précision max (m) avant de passer en "Estimation Seule"
const MIN_SPD = 0.05;       // Vitesse minimale pour être considéré "en mouvement"
const ALT_TH = -50;         // Seuil d'altitude pour détection "Sous-sol"
const MAX_PLAUSIBLE_ACCEL = 20.0; // Accélération max (m/s²) pour filtrage des "spikes"
const NETHER_RATIO = 8.0;   // Ratio pour le mode Nether/Ailleurs

// --- NOUVEAUX SEUILS POUR IMU/ZUPT (Zero Velocity Update) ---
const ZUPT_RAW_THRESHOLD = 1.0;     // Vitesse brute max (m/s) pour suspecter l'arrêt (GPS).
const ZUPT_ACCEL_THRESHOLD = 0.5;   // Accélération max (m/s²) pour suspecter l'arrêt (GPS).
const IMU_ZUPT_THRESHOLD = 0.05;    // NOUVEAU: Accélération nette max pour détecter l'arrêt (IMU).
const Q_BIAS_ACCEL_FACTOR = 0.5;    // NOUVEAU: Facteur d'augmentation de Q basé sur l'accélération.
const BIAS_EMA_ALPHA = 0.1;         // NOUVEAU: Facteur alpha pour l'EMA du Biais (0.1 = rapide).

// --- PARAMÈTRES EKF (ALTITUDE) ---
const Q_ALT_NOISE = 0.1;
const R_ALT_MIN = 0.1;

// --- FACTEURS ENVIRONNEMENTAUX (POUR R) ---
const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0, DISPLAY: 'Normal' },
    'FOREST': { R_MULT: 2.5, DISPLAY: 'Forêt' },
    'CONCRETE': { R_MULT: 7.0, DISPLAY: 'Grotte/Tunnel' },
    'METAL': { R_MULT: 5.0, DISPLAY: 'Métal/Bâtiment' },
};

// --- DONNÉES CÉLESTES/GRAVITÉ ---
const CELESTIAL_DATA = {
    'EARTH': { G: 9.80665, R: R_E_BASE, name: 'Terre' },
    'MOON': { G: 1.62, R: 1737400, name: 'Lune' },
    'MARS': { G: 3.71, R: 3389500, name: 'Mars' },
    'ROTATING': { G: 0.0, R: R_E_BASE, name: 'Station Spatiale' }
};
let G_ACC = CELESTIAL_DATA['EARTH'].G;
let R_ALT_CENTER_REF = CELESTIAL_DATA['EARTH'].R;

// --- VARIABLES D'ÉTAT (Globales) ---
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
let currentMass = 70.0; 
let R_FACTOR_RATIO = 1.0;   // Facteur de Rapport de Mouvement (MRF)
let currentCelestialBody = 'EARTH';
let rotationRadius = 100;
let angularVelocity = 0.0; 
let gpsAccuracyOverride = 0.0; 

// NOUVEAU: Variables IMU/DR
let imuUpdateID = null;             // ID pour la boucle IMU
let lastIMUUpdate = 0;
let NED_Accel_Mag = 0.0;            // Magnitude d'accélération brute
let imuData = { x: 0.0, y: 0.0, z: 0.0 }; 
let attitude = { roll: 0.0, pitch: 0.0, yaw: 0.0 }; 
let imu_bias_est = 0.0;             // Biais estimé (EMA)
let corrected_Accel_Mag = 0.0;      // Accélération nette pour l'EKF
let centrifugalAccel = 0.0; 

// Données externes
let lastP_hPa = null, lastT_K = null, lastH_perc = null; 

// Objets Map
let map, marker, circle;

// --- FONCTIONS UTILITAIRES ---
const $ = id => document.getElementById(id);

/** Calcule la distance de Haversine en mètres */
const dist = (lat1, lon1, lat2, lon2) => {
    const R = R_ALT_CENTER_REF; 
    const dLat = (lat2 - lat1) * D2R, dLon = (lon2 - lon1) * D2R;
    lat1 *= D2R; lat2 *= D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
};

// ... (fonctions syncH, getCDate, getGravityLocal, updateCelestialBody, calculateMRF inchangées) ...

// Nouvelle fonction utilitaire pour la correction d'accélération (Centrifuge & Biais)
function getCorrectedAccel(rawAccelMag, dt) {
    // 1. Mise à jour de la force Centrifuge
    const local_r = parseFloat($('rotation-radius')?.value) || 100;
    const local_w = parseFloat($('angular-velocity')?.value) || 0;
    centrifugalAccel = local_r * (local_w ** 2);
    
    // 2. Soustraction Centrifuge (Accélération nette de mouvement non-linéaire)
    let net_Accel_Mag = rawAccelMag;
    if (net_Accel_Mag >= centrifugalAccel) {
        net_Accel_Mag -= centrifugalAccel;
    } else {
        net_Accel_Mag = 0.0; // L'accélération est plafonnée à zéro si la force centrifuge est dominante
    }

    // 3. ZUPT IMU et Estimation du Biais (Correction de la Dérive)
    if (net_Accel_Mag < IMU_ZUPT_THRESHOLD) {
        // ZUPT IMU Actif : L'accélération nette restante est considérée comme le Biais (dérive)
        
        // MODIFICATION CLÉ: Moyenne Mobile Exponentielle (EMA) pour un effet "5 min" rapide
        if (imu_bias_est === 0.0) {
             imu_bias_est = net_Accel_Mag;
        } else {
            // EMA: Nouvel estimateur = alpha * Nouvelle mesure + (1-alpha) * Ancien estimateur
            imu_bias_est = (BIAS_EMA_ALPHA * net_Accel_Mag) + ((1 - BIAS_EMA_ALPHA) * imu_bias_est);
        }
        
        // L'accélération corrigée est nulle si immobile
        return 0.0; 
    } else {
        // Mouvement détecté
        // 4. Soustraction du Biais (Correction de la Dérive en mouvement)
        return Math.max(0.0, net_Accel_Mag - imu_bias_est);
    }
}


/**
 * Filtre de Kalman 1D pour la vitesse (avec modulation de Q).
 */
function kFilter(nSpd, dt, R_dyn, accel_input = 0) {
    if (dt === 0 || dt > 5) return kSpd; 
    
    // MODIFICATION: Q est modulé par l'accélération corrigée
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

// ... (fonctions kFilterAltitude, getKalmanR, initMap, updateMap, fetchWeather inchangées) ...

// =================================================================
// COUPE ARTIFICIELLE POUR COPIE. SUITE DU SCRIPT CI-DESSOUS.
// =================================================================
// =================================================================
// FICHIER JS COMPLET : gnss-dashboard-full.js
// BLOC 2/2 : Logique Astro, Contrôles & Boucle Principale (IMU/DR + EMA Bias)
// =================================================================

// ... (fonctions Astro/Météo inchangées) ...

// ===========================================
// FONCTIONS DE CONTRÔLE GPS & IMU
// ===========================================

function setGPSMode(mode) {
    currentGPSMode = mode;
    if (wID !== null || imuUpdateID !== null) {
        stopGPS(false); 
        startGPS();     
    }
    if ($('freq-select')) $('freq-select').value = mode; 
}

function startGPS() {
    
    const options = (currentGPSMode === 'HIGH_FREQ') ? GPS_OPTS.HIGH_FREQ : GPS_OPTS.LOW_FREQ;
    
    if (wID === null) {
        wID = navigator.geolocation.watchPosition(updateDisp, handleErr, options);
    }
    
    // Démarrage de la boucle de prédiction IMU haute fréquence (DR)
    if (imuUpdateID === null) {
        imuUpdateID = setInterval(updateIMUPrediction, IMU_FREQUENCY_MS);
    }
    
    if ($('toggle-gps-btn')) {
        $('toggle-gps-btn').textContent = '⏸️ PAUSE GPS/IMU';
        $('toggle-gps-btn').style.backgroundColor = '#ffc107'; 
    }
    
    if (lat && lon) fetchWeather(lat, lon);
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
    if (resetButton) {
        if ($('toggle-gps-btn')) {
            $('toggle-gps-btn').textContent = '▶️ MARCHE GPS/IMU';
            $('toggle-gps-btn').style.backgroundColor = '#28a745'; 
        }
    }
}

// ... (fonctions emergencyStop, resumeSystem, handleErr inchangées) ...

// --- LOGIQUE DE PRÉDICTION IMU HAUTE FRÉQUENCE (20 Hz) ---
function updateIMUPrediction() {
    if (emergencyStopActive) return;

    const now = getCDate();
    if (now === null) return;
    const time = now.getTime();
    if (lastIMUUpdate === 0) lastIMUUpdate = time;
    
    const dt_imu = (time - lastIMUUpdate) / 1000;
    const dt = (dt_imu > 0 && dt_imu < 1) ? dt_imu : IMU_FREQUENCY_MS / 1000; 
    
    lastIMUUpdate = time;

    // 1. Calcul de l'accélération corrigée (inclut la compensation centrifuge et l'EMA du Biais)
    corrected_Accel_Mag = getCorrectedAccel(NED_Accel_Mag, dt); 

    let R_imu_pred = R_MAX;
    let modeStatus = '';
    
    if (wID === null) { 
        // Mode DR (GPS Pausé)
        if (corrected_Accel_Mag === 0.0) {
             // ZUPT actif (grâce au bias EMA rapide, la vitesse converge vite vers 0)
             R_imu_pred = R_MIN; 
             modeStatus = '✅ DR - ZUPT/BIAS ON (IMU)';
        } else {
             // Mouvement IMU détecté
             R_imu_pred = R_MAX; // Pas de correction GPS, prédiction pure
             modeStatus = '⚠️ DR - PRÉDICTION IMU';
        }
    } else {
         // GPS Actif - Pas d'action, la correction sera faite par updateDisp
         return; 
    }

    // Le filtre de Kalman est appelé ici uniquement pour le mode DR
    kFilter(kSpd, dt, R_imu_pred, corrected_Accel_Mag);

    // Mise à jour de la distance, du temps de mouvement, etc.
    const sSpdFE = kSpd < MIN_SPD ? 0 : kSpd; 
    R_FACTOR_RATIO = calculateMRF(kAlt); 
    distM += sSpdFE * dt * R_FACTOR_RATIO; 
    if (sSpdFE > MIN_SPD) { timeMoving += dt; }
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    
    // Mise à jour de l'affichage à haute fréquence (pour IMU)
    if ($('gps-status-dr')) $('gps-status-dr').textContent = modeStatus;
    if ($('imu-bias-est')) $('imu-bias-est').textContent = imu_bias_est.toFixed(6);
    if ($('accel-imu-mag')) $('accel-imu-mag').textContent = corrected_Accel_Mag.toFixed(3);
    if ($('centrifugal-accel')) $('centrifugal-accel').textContent = centrifugalAccel.toFixed(3);
    if ($('speed-stable')) $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(5)} km/h`;
    if ($('distance-total-km')) $('distance-total-km').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
}


// ===========================================
// FONCTION PRINCIPALE DE MISE À JOUR GPS (updateDisp)
// ===========================================

function updateDisp(pos) {
    if (emergencyStopActive) return;

    const cTimePos = pos.timestamp;
    let cLat = pos.coords.latitude;
    let cLon = pos.coords.longitude;
    let altRaw = pos.coords.altitude;
    let accRaw = pos.coords.accuracy;
    let headingRaw = pos.coords.heading;

    const now = getCDate(); 
    if (now === null) { return; } 
    if (sTime === null) { sTime = now.getTime(); }
    
    if (gpsAccuracyOverride > 0.0) {
        accRaw = gpsAccuracyOverride;
    }

    let isSignalLost = (accRaw > MAX_ACC);
    let modeStatus = '';
    
    let R_dyn = getKalmanR(accRaw, kAlt, lastP_hPa); 
    const acc = accRaw; 

    // ... (Logique Signal Lost/Mode Status inchangée) ...
    if (isSignalLost) { 
        modeStatus = `⚠️ ESTIMATION SEULE (Signal Perdu/ARRÊTÉ)`;
        if ($('gps-precision')) $('gps-precision').textContent = `❌ ${accRaw.toFixed(0)} m (Signal Perdu/Estimation)`; 
        if (lPos === null) { lPos = pos; return; }
        cLat = lat;
        cLon = lon;
        altRaw = kAlt; 
    } else {
        if (R_dyn >= R_MAX * 0.75) {
             modeStatus = `🛰️ FUSION FAIBLE (Capteur Domine) ↑`; 
        } else if (R_dyn > R_MAX * 0.5) {
             modeStatus = `🏡 FUSION MOYENNE (Lissage Actif)`; 
        } else {
             modeStatus = `🚀 FUSION TOTALE (Équilibré)`;
        }
        lat = cLat; 
        lon = cLon;
        if ($('gps-precision')) $('gps-precision').textContent = `${accRaw.toFixed(2)} m`; 
    }
    
    let dt = 0;
    if (lPos) {
        dt = (cTimePos - lPos.timestamp) / 1000;
    } else {
        lPos = pos; 
        lPos.speedMS_3D = 0;
        lPos.kAlt_old = altRaw;
        kAlt = altRaw;
        updateMap(cLat, cLon, accRaw);
        return; 
    }
    
    if (dt < MIN_DT || dt > 10) { 
        lPos = pos; 
        return; 
    }

    // FILTRAGE DE L'ALTITUDE (via Kalman)
    const kAlt_new = kFilterAltitude(altRaw, pos.coords.altitudeAccuracy || R_ALT_MIN, dt);
    
    // ... (Calcul de la vitesse 3D brute et Correction Anti-Spike inchangés) ...
    const dist2D = dist(lPos.coords.latitude, lPos.coords.longitude, cLat, cLon);
    const dist3D = Math.sqrt(dist2D ** 2 + (kAlt_new - (lPos.kAlt_old || kAlt_new)) ** 2);
    let spd3D_raw = dist3D / dt; 
    const spdV = (kAlt_new - (lPos.kAlt_old || kAlt_new)) / dt; 
    let accel_long_provisional = 0;
    if (lPos && lPos.speedMS_3D !== undefined && dt > 0.05) { 
        accel_long_provisional = (spd3D_raw - lPos.speedMS_3D) / dt;
    }
    // ... (Correction Anti-Spike inchangée) ...

    
    // --- LOGIQUE ZUPT (Zero Velocity Update) GPS ---
    let spd_kalman_input = spd3D_raw;
    let R_kalman_input = R_dyn;
    
    const isPlausiblyStopped = (
        spd3D_raw < ZUPT_RAW_THRESHOLD && 
        Math.abs(accel_long_provisional) < ZUPT_ACCEL_THRESHOLD &&
        R_dyn < R_MAX 
    ); 
    
    if (isPlausiblyStopped) { 
        spd_kalman_input = 0.0;     // Forcer la mesure à 0 m/s
        R_kalman_input = R_MIN;     // Confiance maximale dans la mesure ZUPT
        modeStatus = '✅ ZUPT (Vélocité Nulle Forcée)';
    }

    // --- UTILISATION DE L'ACCÉLÉRATION CORRIGÉE IMU ---
    // On recalcule ici pour s'assurer que l'estimation du biais est prise en compte.
    corrected_Accel_Mag = getCorrectedAccel(NED_Accel_Mag, dt); 
    let accel_sensor_input = corrected_Accel_Mag; 


    // FILTRE DE KALMAN FINAL (Fusion IMU/GNSS)
    // Utilisation de l'accélération corrigée (Débiassée) comme entrée de prédiction
    const fSpd = kFilter(spd_kalman_input, dt, R_kalman_input, accel_sensor_input); 
    const sSpdFE = fSpd < MIN_SPD ? 0 : fSpd; 
    
    // Calculs d'accélération (final, basé sur la vitesse filtrée)
    let accel_long = 0;
    if (dt > 0.05) { 
        accel_long = (sSpdFE - lastFSpeed) / dt;
    }
    lastFSpeed = sSpdFE;

    R_FACTOR_RATIO = calculateMRF(kAlt_new); 
    distM += sSpdFE * dt * R_FACTOR_RATIO; 
    
    if (sSpdFE > MIN_SPD) { timeMoving += dt; }
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    
    // Calculs de physique (inchangés)
    const local_g = getGravityLocal(kAlt_new); 
    const kineticEnergy = 0.5 * currentMass * sSpdFE ** 2;
    const mechanicalPower = currentMass * sSpdFE * accel_long;
    const coriolis_force = 2 * currentMass * sSpdFE * OMEGA_EARTH * Math.sin(lat * D2R);

    // --- MISE À JOUR DU DOM (Affichage) ---
    
    // ... (Affichages Vitesse, Distance, GPS/Altitude inchangés) ...
    
    // Section IMU/DR Ajoutée
    if ($('centrifugal-accel')) $('centrifugal-accel').textContent = `${centrifugalAccel.toFixed(3)} m/s²`;
    if ($('imu-bias-est')) $('imu-bias-est').textContent = imu_bias_est.toFixed(6);
    if ($('accel-imu-mag')) $('accel-imu-mag').textContent = corrected_Accel_Mag.toFixed(3);
    if ($('gps-status-dr')) $('gps-status-dr').textContent = modeStatus;
    
    // Section Dynamique
    if ($('gravity-local')) $('gravity-local').textContent = `${local_g.toFixed(5)} m/s²`;
    if ($('accel-long')) $('accel-long').textContent = `${accel_long.toFixed(3)} m/s²`;
    if ($('force-g-long')) $('force-g-long').textContent = G_ACC > 0.1 ? `${(accel_long / local_g).toFixed(2)} G` : '0.00 G';
    if ($('vertical-speed')) $('vertical-speed').textContent = `${spdV.toFixed(2)} m/s`;
    
    // ... (Reste des affichages inchangés) ...

    // SAUVEGARDE DES VALEURS POUR LA PROCHAINE ITÉRATION
    lPos = pos; 
    lPos.speedMS_3D = spd3D_raw; 
    lPos.timestamp = cTimePos; 
    lPos.kAlt_old = kAlt_new; 

    // Mise à jour de la carte
    updateMap(lat, lon, accRaw);
}


// ===========================================
// INITIALISATION DES ÉVÉNEMENTS DOM
// ===========================================

document.addEventListener('DOMContentLoaded', () => {
    
    initMap(); 

    // ... (gestion des inputs utilisateur inchangée) ...

    if ($('toggle-gps-btn')) {
        $('toggle-gps-btn').addEventListener('click', () => { 
            if (emergencyStopActive) {
                alert("Veuillez désactiver l'Arrêt d'urgence avant d'utiliser ce contrôle.");
                return;
            }
            wID === null ? startGPS() : stopGPS(); 
        });
    }
    // ... (Reste des boutons de contrôle inchangés) ...
    
    // AJOUT: Listener IMU pour lecture des données
    if (window.DeviceOrientationEvent) {
        window.addEventListener('deviceorientation', (event) => {
            attitude.yaw = event.alpha || 0; 
            attitude.pitch = event.beta || 0; 
            attitude.roll = event.gamma || 0;
        });
    }

    window.addEventListener('devicemotion', (event) => {
        const a_lin = event.acceleration; 
        
        if (a_lin) {
            // Lecture directe de l'accélération linéaire pure
            imuData.x = a_lin.x; imuData.y = a_lin.y; imuData.z = a_lin.z;
            NED_Accel_Mag = Math.sqrt(a_lin.x**2 + a_lin.y**2 + a_lin.z**2);
        } else if (event.accelerationIncludingGravity) {
            // Estimation si seulement accélération + gravité est disponible
            const { x, y, z } = event.accelerationIncludingGravity;
            const accelMag = Math.sqrt(x*x + y*y + z*z);
            // Soustraction de la gravité locale (dépendante du corps céleste)
            NED_Accel_Mag = Math.abs(accelMag - G_ACC); 
        }
    });

    // --- DÉMARRAGE DU SYSTÈME ---
    syncH(); 
    updateCelestialBody(currentCelestialBody); 
    startGPS(); // Démarre à la fois le GPS et la boucle IMU/DR

    // ... (Boucle de mise à jour lente inchangée) ...
});
