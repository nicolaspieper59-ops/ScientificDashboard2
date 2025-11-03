// =================================================================
// FICHIER JS PARTIE 1 : gnss-dashboard-part1.js
// (Constantes, Utilitaires, Kalman, Astro, Météo, IMU, Lune & Ciel)
// =================================================================

const $ = (id) => document.getElementById(id);

// --- CONSTANTES GLOBALES (Physiques, GPS, Temps) ---
const C_L = 299792458; // Vitesse de la lumière (m/s)
const SPEED_SOUND = 343; // Vitesse du son (m/s) (Utilisé comme fallback)
const G_ACC = 9.80665; // Gravité standard (m/s²)
const KMH_MS = 3.6; // M/s vers Km/h
const R_E = 6371000; // Rayon moyen de la Terre (m)
const R2D = 180 / Math.PI;
const D2R = Math.PI / 180;
const W_EARTH = 7.2921E-5; // Vitesse angulaire de la Terre (rad/s)
const NETHER_RATIO = 1 / 8; 

// Constantes Temps / Astro
const dayMs = 86400000;
const J1970 = 2440588; 
const J2000 = 2451545; 
const DOM_SLOW_UPDATE_MS = 1000;
const WEATHER_UPDATE_MS = 30000; 
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc"; 

// Constantes GPS
const MIN_DT = 0.05; 
const MIN_SPD = 0.01; 
const MAX_ACC = 20; 
const SUBTERRANEAN_ACC_THRESHOLD = 50.0; 
const ALT_TH = -50; 
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 5000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 30000, timeout: 60000 }
};

// Constantes Kalman
const Q_NOISE = 0.001; 
const IMU_ONLY_R_NOISE = 100.0; 
let kSpd = 0; // Estimation vitesse (m/s)
let kUncert = 1000; // Incertitude vitesse (m/s)²
const ENVIRONMENT_FACTORS = {
    NORMAL: 1.0, 
    FOREST: 1.5, 
    CONCRETE: 3.0, 
    METAL: 2.5,
    AIRCRAFT: 8.0, 
    TRAIN: 4.0, // NOUVEAU: Bruit et blindage métallique
    MARINE: 2.0 // NOUVEAU: Mouvement d'eau (pitch/roll)
};
const Q_ALT_NOISE = 0.01; 
let kAlt = 0; // Estimation altitude (m)
let kAltUncert = 1000; // Incertitude altitude (m)²

// Constantes IMU
const ACCEL_FILTER_ALPHA = 0.8; 
const GRAVITY_CALIBRATION_ALPHA = 0.01; 
const ACCEL_MOVEMENT_THRESHOLD = 0.8; 
let kAccel = { x: 0, y: 0, z: 0 };
let G_STATIC_REF = { x: 0, y: 0, z: 0 }; // Référence G étalonnée
let latestVerticalAccelIMU = 0; 
let latestLinearAccelMagnitude = 0; 

// --- VARIABLES GLOBALES (État du système) ---
let wID = null;
let map = null;
let marker = null;
let tracePolyline = null;
let lat = 0, lon = 0;
let sTime = null; 
let distM = 0; 
let timeMoving = 0; 
let maxSpd = 0;
let lastFSpeed = 0;
let currentGPSMode = 'HIGH_FREQ';
let lPos = null; 
let emergencyStopActive = false;
let netherMode = false;
let selectedEnvironment = 'NORMAL';
let subterraneanMode = false; 
let lServH = 0; 
let lLocH = 0; 
let domID = null; 
let weatherID = null; 
let lastP_hPa = 1013.25; 
const DEFAULT_LAT = 43.296; 
const DEFAULT_LON = 5.378; 
const OWM_API_KEY = "VOTRE_CLE_API_OPENWEATHERMAP"; 
const OWM_API_URL = "https://api.openweathermap.org/data/2.5/weather";


// ===========================================
// FONCTIONS UTILITAIRES ET KALMAN
// ===========================================

/** Calcule la distance de Haversine entre deux points (m). */
function dist(lat1, lon1, lat2, lon2) { /* ... (implémentation omise) ... */ return 0; }

/** Filtre de Kalman 1D (Vitesse) */
function kFilter(z, dt, R_dyn) {
    const predSpd = kSpd;
    const predUncert = kUncert + Q_NOISE * dt;
    const K = predUncert / (predUncert + R_dyn);
    kSpd = predSpd + K * (z - predSpd);
    kUncert = (1 - K) * predUncert;
    return kSpd;
}

/** Calcule le bruit de mesure dynamique R (Intègre IMU linéaire) */
function getKalmanR(acc, alt, pressure, linearAccelMag) { 
    let R_raw = acc * acc; 
    const envFactor = ENVIRONMENT_FACTORS[selectedEnvironment] || ENVIRONMENT_FACTORS.NORMAL;
    let noiseMultiplier = envFactor;
    if (alt !== null && alt < 0) {
        noiseMultiplier += Math.abs(alt / 100); 
    }
    if (linearAccelMag > 0.5) { 
        noiseMultiplier += Math.pow(linearAccelMag, 1.5) * 0.5; 
    }
    return R_raw * noiseMultiplier;
}

/** Filtre de Kalman 1D pour l'Altitude */
function kFilterAltitude(z, acc, dt, u_accel = 0) { 
    if (z === null) return kAlt; 
    // Prédiction basée sur l'accélération verticale IMU
    const predAlt = kAlt + (0.5 * u_accel * dt * dt); 
    let predAltUncert = kAltUncert + Q_ALT_NOISE * dt;
    const R_alt = acc * acc * 2.0; 
    const K = predAltUncert / (predAltUncert + R_alt);
    kAlt = predAlt + K * (z - predAlt);
    kAltUncert = (1 - K) * predAltUncert;
    return kAlt;
}

// ... (fonctions getCDate, getSolarTime, calculateDewPoint, updateMoon, updateAstro, updateWeather, etc. omises) ...


/** Détermine la couleur de fond du body (ciel TST / Disque Jour-Nuit) */
function getSkyColor(elevation, date) { 
    const body = document.body;
    let newColor = '#f4f4f4'; 
    if (body.classList.contains('dark-mode')) { newColor = '#1a1a1a'; if (elevation > -12 * D2R) { newColor = '#2c2c2c'; } } 
    else {
        if (elevation < -18 * D2R) { newColor = '#333'; } 
        else if (elevation < -12 * D2R) { newColor = '#666'; } 
        else if (elevation < -6 * D2R) { newColor = '#999'; } 
        else if (elevation < 0) { newColor = '#ccc'; } 
        else if (elevation < 0.1 * D2R) { newColor = '#f0d0c0'; } 
        else { newColor = '#f4f4f4'; } 
    }
    body.style.backgroundColor = newColor;
}


// ===========================================
// FONCTIONS CAPTEURS INERTIELS (IMU) - ÉTALONNAGE CONTINU ADAPTÉ
// ===========================================

/** Gère les données de l'accéléromètre via DeviceMotionEvent. */
function handleDeviceMotion(event) {
    if (emergencyStopActive) return;
    const acc = event.accelerationIncludingGravity;
    if (acc.x === null) return; 

    // 1. Lissage des mesures brutes
    kAccel.x = ACCEL_FILTER_ALPHA * kAccel.x + (1 - ACCEL_FILTER_ALPHA) * acc.x;
    kAccel.y = ACCEL_FILTER_ALPHA * kAccel.y + (1 - ACCEL_FILTER_ALPHA) * acc.y; 
    kAccel.z = ACCEL_FILTER_ALPHA * kAccel.z + (1 - ACCEL_FILTER_ALPHA) * acc.z; 

    // 2. Calcul de l'accélération linéaire (par soustraction de la référence G étalonnée)
    const accel_lin_x = kAccel.x - G_STATIC_REF.x;
    const accel_lin_y = kAccel.y - G_STATIC_REF.y;
    const accel_lin_z = kAccel.z - G_STATIC_REF.z;

    // 3. Magnitude de l'accélération linéaire 3D
    const accel_3d_mag = Math.sqrt(accel_lin_x ** 2 + accel_lin_y ** 2 + accel_lin_z ** 2);
    latestLinearAccelMagnitude = accel_3d_mag;
    
    // 4. Étalonnage Continu de la Gravité (G-Freeze en mode Avion ou Bateau)
    // Désactiver la calibration si en mode AIRCRAFT ou MARINE (mouvement prolongé hors statique)
    const disableCalibration = selectedEnvironment === 'AIRCRAFT' || selectedEnvironment === 'MARINE';

    if (!disableCalibration && accel_3d_mag < ACCEL_MOVEMENT_THRESHOLD) {
        G_STATIC_REF.x = GRAVITY_CALIBRATION_ALPHA * kAccel.x + (1 - GRAVITY_CALIBRATION_ALPHA) * G_STATIC_REF.x;
        G_STATIC_REF.y = GRAVITY_CALIBRATION_ALPHA * kAccel.y + (1 - GRAVITY_CALIBRATION_ALPHA) * G_STATIC_REF.y;
        G_STATIC_REF.z = GRAVITY_CALIBRATION_ALPHA * kAccel.z + (1 - GRAVITY_CALIBRATION_ALPHA) * G_STATIC_REF.z;
    } 
    
    // 5. Mise à jour des valeurs DOM
    latestVerticalAccelIMU = accel_lin_z;
    
    if ($('accel-vertical-imu')) $('accel-vertical-imu').textContent = `${latestVerticalAccelIMU.toFixed(3)} m/s²`;
    if ($('force-g-vertical')) $('force-g-vertical').textContent = `${(latestVerticalAccelIMU / G_ACC).toFixed(2)} G`;
    if ($('accel-3d-mag')) $('accel-3d-mag').textContent = `${accel_3d_mag.toFixed(3)} m/s²`;
    if ($('force-g-3d-mag')) $('force-g-3d-mag').textContent = `${(accel_3d_mag / G_ACC).toFixed(2)} G`;
                    }
// =================================================================
// FICHIER JS PARTIE 2 : gnss-dashboard-part2.js
// (Logique Principale, Carte, Contrôles et Initialisation)
// =================================================================

// Les fonctions et variables globales sont définies dans part1.js.

// ... (fonctions initMap, updateMap, setGPSMode, etc. sont ici) ...

// ===========================================
// FONCTION PRINCIPALE DE MISE À JOUR (GPS, Kalman, Physique)
// ===========================================

function updateDisp(pos) {
    if (emergencyStopActive) return;
    lat = pos.coords.latitude; lon = pos.coords.longitude;
    const alt = pos.coords.altitude, acc = pos.coords.accuracy;
    const spd_raw_gps = pos.coords.speed;
    const cTimePos = pos.timestamp; 
    const now = getCDate(); 
    const MASS = 70.0; 
    
    // ... (Vérifications de temps et de sTime) ...

    let effectiveAcc = acc;
    const accOverride = parseFloat($('gps-accuracy-override').value);
    if (accOverride > 0) { effectiveAcc = accOverride; }

    let spdH = spd_raw_gps ?? 0; 
    const dt = lPos ? (cTimePos - lPos.timestamp) / 1000 : MIN_DT;

    // ----------------------------------------------------
    // LOGIQUE DU MODE SOUTERRAIN
    // ----------------------------------------------------
    let isGPSPoor = (effectiveAcc > SUBTERRANEAN_ACC_THRESHOLD) || (acc === null);

    if (isGPSPoor && !subterraneanMode) {
        subterraneanMode = true;
    } else if (!isGPSPoor && subterraneanMode) {
        subterraneanMode = false;
    }
    
    if ($('subterranean-status-display')) {
        $('subterranean-status-display').textContent = subterraneanMode ? 
            `🟢 ACTIF (IMU) | GPS Acc: ${effectiveAcc.toFixed(0)} m` : 
            `🔴 INACTIF (GPS+EKF) | GPS Acc: ${effectiveAcc.toFixed(0)} m`;
    }
    // ----------------------------------------------------
    
    let spd3D = 0;
    let R_dyn;
    let kAlt_new = kAlt; 
    const isAircraftMode = selectedEnvironment === 'AIRCRAFT';
    const isTrainMode = selectedEnvironment === 'TRAIN';
    const isMarineMode = selectedEnvironment === 'MARINE';

    if (subterraneanMode) {
        // Mode IMU SEULE (Dead Reckoning)
        const predictedSpeedFromIMU = kSpd + (latestLinearAccelMagnitude * dt);
        spd3D = predictedSpeedFromIMU; 
        R_dyn = IMU_ONLY_R_NOISE; 

        if (lPos) { lat = lPos.coords.latitude; lon = lPos.coords.longitude; }
        
    } else {
        // Mode GPS/IMU normal (ou AIRCRAFT/TRAIN/MARINE)
        
        // Vitesse Horizontale Calculée
        if (lPos && dt > 0.05) { 
            const dH = dist(lPos.coords.latitude, lPos.coords.longitude, lat, lon); 
            spdH = dH / dt; 
        } 
        
        // Filtrage de l'Altitude
        kAlt_new = kFilterAltitude(alt, effectiveAcc, dt, latestVerticalAccelIMU); 
        
        // Vitesse Verticale
        let spdV = 0; 
        if (lPos && lPos.kAlt_old !== undefined && dt > MIN_DT && alt !== null) { 
            spdV = (kAlt_new - lPos.kAlt_old) / dt; 
        } else if (alt !== null) { 
            spdV = 0; 
        }
        
        spd3D = Math.sqrt(spdH ** 2 + spdV ** 2);
        
        // Calcul de R_dyn basé sur la précision GPS et le facteur d'environnement
        R_dyn = getKalmanR(effectiveAcc, alt, lastP_hPa, latestLinearAccelMagnitude); 
    }
    
    // 4. FILTRE DE KALMAN FINAL (Vitesse 3D Stable)
    const fSpd = kFilter(spd3D, dt, R_dyn); 
    const sSpdFE = fSpd < MIN_SPD ? 0 : fSpd; 
    
    // CALCUL DE LA MARGE D'ERREUR (3-Sigma)
    const speedErrorMargin = 3 * Math.sqrt(kUncert);
    const speedErrorMarginKmh = speedErrorMargin * KMH_MS;

    // 5. CALCULS PHYSIQUES ET RELATIFS
    let accel_long = (dt > 0.05) ? (sSpdFE - lastFSpeed) / dt : 0;
    lastFSpeed = sSpdFE;
    distM += sSpdFE * dt * (netherMode ? NETHER_RATIO : 1); 
    if (sSpdFE > MIN_SPD) { timeMoving += dt; }
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    const avgSpdMoving = timeMoving > 0 ? (distM / timeMoving) : 0;
    
    const coriolusForce = 2 * MASS * sSpdFE * W_EARTH * Math.sin(lat * D2R);
    const kineticEnergy = 0.5 * MASS * sSpdFE ** 2;
    const mechanicalPower = accel_long * MASS * sSpdFE; 
    const v_c_ratio = sSpdFE / C_L;
    const lorentzFactor = 1 / Math.sqrt(Math.max(1 - v_c_ratio ** 2, 1e-15)); 
    
    // ... (Calculs de densité de l'air et pression dynamique omis) ...
    let airDensity = "N/A";
    const dynamicPressure = 0.0; // Simplifié

    // --- 6. MISE À JOUR DU DOM ---
    const sSpdFE_Kmh = (sSpdFE * KMH_MS);
    const distM_display = distM * (netherMode ? NETHER_RATIO : 1);

    // Vitesse/Distance
    if ($('speed-stable')) $('speed-stable').textContent = `${sSpdFE_Kmh.toFixed(5)} km/h`;
    if ($('speed-error-margin')) $('speed-error-margin').textContent = `± ${speedErrorMarginKmh.toFixed(3)} km/h`; 
    // ... (Autres mises à jour de vitesse) ...

    // Altitudes et Précision
    if ($('latitude')) $('latitude').textContent = lat.toFixed(6) + (subterraneanMode ? ' (DR)' : '');
    if ($('longitude')) $('longitude').textContent = lon.toFixed(6) + (subterraneanMode ? ' (DR)' : '');

    // AFFICHAGE SPÉCIFIQUE SELON LE MODE DE TRANSPORT
    if (isAircraftMode) {
        if ($('altitude-gps')) $('altitude-gps').textContent = `N/A (Cabine Pressurisée)`;
        if ($('altitude-msl')) $('altitude-msl').textContent = `N/A (Cabine Pressurisée)`;
        if ($('underground-status')) $('underground-status').textContent = 'N/A (Vol)';
        if ($('coriolis-force')) $('coriolis-force').textContent = `${coriolusForce.toExponential(2)} N (Pertinente)`; 
    } else if (isTrainMode) { 
        if ($('altitude-gps')) $('altitude-gps').textContent = subterraneanMode ? `N/A (IMU Souterrain)` : `${alt !== null ? alt.toFixed(2) : 'N/A'} m (Vibrations)`;
        if ($('altitude-msl')) $('altitude-msl').textContent = subterraneanMode ? `N/A (IMU Souterrain)` : `${kAlt_new !== null ? kAlt_new.toFixed(2) : 'N/A'} m (Vibrations)`;
        if ($('underground-status')) $('underground-status').textContent = (kAlt_new !== null && kAlt_new < ALT_TH) ? 'Oui (Tunnel)' : 'Non';
        if ($('coriolis-force')) $('coriolis-force').textContent = `${coriolusForce.toExponential(2)} N (Pertinente)`;
    } else if (isMarineMode) { 
        if ($('altitude-gps')) $('altitude-gps').textContent = subterraneanMode ? `N/A (IMU Souterrain)` : `${alt !== null ? alt.toFixed(2) : 'N/A'} m (Vagues/Marnage)`;
        if ($('altitude-msl')) $('altitude-msl').textContent = subterraneanMode ? `N/A (IMU Souterrain)` : `${kAlt_new !== null ? kAlt_new.toFixed(2) : 'N/A'} m (Vagues/Marnage)`;
        if ($('underground-status')) $('underground-status').textContent = 'N/A (Marin)';
        if ($('coriolis-force')) $('coriolis-force').textContent = `${coriolusForce.toExponential(2)} N (Essentielle)`; 
    } else { 
        if ($('altitude-gps')) $('altitude-gps').textContent = subterraneanMode ? `N/A (IMU Souterrain)` : `${alt !== null ? alt.toFixed(2) : 'N/A'} m`;
        if ($('altitude-msl')) $('altitude-msl').textContent = subterraneanMode ? `N/A (IMU Souterrain)` : `${kAlt_new !== null ? kAlt_new.toFixed(2) : 'N/A'} m`;
        if ($('underground-status')) $('underground-status').textContent = (kAlt_new !== null && kAlt_new < ALT_TH) ? 'Oui' : 'Non';
        if ($('coriolis-force')) $('coriolis-force').textContent = `${coriolusForce.toExponential(2)} N`;
    }
    
    if ($('gps-precision')) $('gps-precision').textContent = `${acc !== null ? acc.toFixed(2) : 'N/A'} m`;
    if ($('gps-accuracy-effective')) $('gps-accuracy-effective').textContent = `${effectiveAcc.toFixed(2)} m`;
    if ($('speed-error-perc')) $('speed-error-perc').textContent = `${R_dyn.toFixed(3)} m² (R dyn)`;
    
    // ... (Mises à jour Dynamique, Forces, Relativité, etc.) ...
    
    // --- 7. MISE À JOUR DES COMPOSANTS ET SAUVEGARDE ---
    updateAstro(lat, lon);
    
    if (!subterraneanMode) {
        updateMap(lat, lon);
    }
    
    lPos = { 
        coords: { 
            latitude: lat, 
            longitude: lon, 
            altitude: alt 
        }, 
        timestamp: cTimePos, 
        kAlt_old: kAlt_new, 
        kAltUncert_old: kAltUncert
    };
} 


// ... (INITIALISATION DES ÉVÉNEMENTS) ...
document.addEventListener('DOMContentLoaded', () => {
    // ... (initialisation de la carte, des boutons) ...
    const envSelect = $('environment-select');
    if (envSelect) {
        envSelect.addEventListener('change', (event) => {
            selectedEnvironment = event.target.value;
            // G-Freeze : Si le mode avion ou bateau est activé, nous réinitialisons G_STATIC_REF pour geler l'estimation de G
            if (selectedEnvironment === 'AIRCRAFT' || selectedEnvironment === 'MARINE') {
                G_STATIC_REF = { x: 0, y: 0, z: 0 };
            }
        });
    }
    // ... (démarrage du GPS et des écouteurs devicemotion) ...
});
