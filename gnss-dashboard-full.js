// =================================================================
// BLOC 1/4 : Constantes, État Global et Utilitaires
// Fichier: constants_and_state.js
// =================================================================

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;          // Vitesse de la lumière (m/s)
const G_CONST = 6.6743e-11;     // Constante gravitationnelle (N·m²/kg²)
const R_E_BASE = 6371000;       // Rayon terrestre moyen (m)
const KMH_MS = 3.6;             // Conversion m/s vers km/h
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const R_AIR = 287.058;          // Constante spécifique de l'air sec (J/kg·K)
const TEMP_SEA_LEVEL_K = 288.15; // 15°C en Kelvin
const BARO_ALT_REF_HPA = 1013.25; // Pression de référence (hPa)
const RHO_SEA_LEVEL = 1.225;    // Densité de l'air au niveau de la mer (kg/m³)

// --- PARAMÈTRES DU FILTRE DE KALMAN (UKF) ---
const Q_NOISE = 0.1;        // Bruit de processus (Position/Vitesse)
const R_MIN = 0.01;         // Bruit de mesure minimum (Vitesse)
const MAX_ACC_UNCERT = 200; // Précision max (m) avant "Estimation Seule"
const MIN_SPD_THRESH = 0.05; // Vitesse minimale pour être considéré comme "en mouvement"

// Définitions des corps célestes
const CELESTIAL_BODIES = {
    'EARTH': { display: 'Terre', gravity: 9.80665, radius: R_E_BASE },
    'MOON': { display: 'Lune', gravity: 1.625, radius: 1737400 },
    // ... autres corps célestes ...
};

// --- ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE ---
let isGpsPaused = true; 
let isEmergencyStop = false;
let selectedEnvironment = 'NORMAL';
let selectedCelestialBody = 'EARTH';

let currentMass = 70.0;
let distanceRatio = 1.0; 
let totalDistanceM = 0.0;
let maxSpeedKmh = 0.0;
let timeMovingMs = 0;
let timeElapsedMs = 0;
let lastTimestamp = 0;
let gpsWatchId = null; 
let lastGpsTimestamp = 0;
let imuActive = false;

// UKF/EKF State
let ukfState = {
    x: [0, 0, 0, 0], 
    P: [[100, 0, 0, 0], [0, 100, 0, 0], [0, 0, 10, 0], [0, 0, 0, 10]] 
};
// Coordonnées de repli pour les calculs Astro/Météo au démarrage
let lastEKFPosition = { lat: 43.2964, lon: 5.3697, alt: 0, heading: 0 }; 
let currentSpeedMs = 0.0;

// Météo et Correction Métrologique (Valeurs initiales du panneau)
let lastP_hPa = 1013;
let lastT_K = 283.45; 
let lastH_perc = 0.6; 
let currentAirDensity = 1.241;
let currentSpeedOfSound = 337.51;

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);

/** Formate une valeur numérique ou retourne 'N/A' si elle est invalide. */
const dataOrDefault = (val, decimals, suffix = '', precisionThreshold = 0.00000001) => {
    if (val === undefined || val === null || isNaN(val)) {
        return 'N/A';
    }
    // Formatage scientifique pour les extrêmes (Énergie, Rs)
    if (Math.abs(val) > 1e18 || (Math.abs(val) < precisionThreshold && val !== 0)) {
        return val.toExponential(3) + suffix;
    }
    return val.toFixed(decimals) + suffix;
};
// =================================================================
// BLOC 2/4 : Calculs Physique, Relativiste et UKF Core
// Fichier: physics_ukf_core.js
// Dépendance: BLOC 1
// =================================================================

/** Calcule le facteur de Lorentz (gamma). */
const getLorentzFactor = (v) => {
    if (v >= C_L) return Infinity;
    return 1.0 / Math.sqrt(1.0 - (v * v) / (C_L * C_L));
};

/** Calcule le Rayon de Schwarzschild (Rs). */
const getSchwarzschildRadius = (massKg) => {
    if (massKg <= 0) return 0;
    return (2 * G_CONST * massKg) / (C_L * C_L);
};

/** Calcule la densité de l'air (Formule de l'air humide). */
const getAirDensity = (p_hPa, t_K, h_perc = 0.6) => {
    const T_C = t_K - 273.15;
    const PV_SAT = 6.1078 * Math.pow(10, (7.5 * T_C) / (T_C + 237.3)) * 100; 
    const PV_ACTUAL = h_perc * PV_SAT; 
    const P_Pa = p_hPa * 100; 
    const P_DRY = P_Pa - PV_ACTUAL;
    const R_W = 461.495; 
    const rho = (P_DRY / (R_AIR * t_K)) + (PV_ACTUAL / (R_W * t_K));
    return rho;
};

/** Calcule la vitesse du son (fonction de la température). */
const getSpeedOfSound = (t_K) => {
    const GAMMA = 1.4; 
    const R = R_AIR;
    return Math.sqrt(GAMMA * R * t_K);
};

/** Calcule le Point de Rosée (Formule Magnus). */
const getDewPoint = (t_c, h_perc) => {
    if (h_perc <= 0 || isNaN(t_c)) return NaN;
    const A = 17.27;
    const B = 237.7;
    const logRH = Math.log(h_perc);
    const gamma = (A * t_c) / (B + t_c) + logRH;
    return (B * gamma) / (A - gamma);
};

/** Calcule la Force de Traînée (Approximation). */
const getDragForce = (rho, v, A = 1.0, Cd = 0.47) => { 
    const q = 0.5 * rho * v * v;
    return q * A * Cd;
};

/** Calcule la Force de Coriolis. */
const getCoriolisForce = (v_ms, latDeg, massKg) => {
    const latRad = latDeg * D2R;
    const f = 2 * OMEGA_EARTH * Math.sin(latRad);
    return massKg * f * v_ms;
};

/** Calcule l'énergie de masse au repos. */
const getRestMassEnergy = (massKg) => {
    return massKg * C_L * C_L;
};

// --- UKF/EKF CORE LOGIC ---
/** Exécute une étape de prédiction et de mise à jour du filtre UKF/EKF (Vitesse). */
const runUKFStep = (positionData, deltaTime) => {
    // R_MEASURE basé sur la précision GPS réelle (acc)
    const R_MEASURE = Math.pow(Math.max(R_MIN, positionData.acc), 2); 
    const Q_PROC = Math.pow(Q_NOISE, 2); 

    let v_old = currentSpeedMs;
    let P_old = ukfState.P[2][2]; 
    
    // Prédiction (Modèle de vitesse constante)
    let v_pred = v_old;
    let P_pred = P_old + Q_PROC * deltaTime;
    
    // Mise à jour (Correction avec mesure GNSS)
    const K = P_pred / (P_pred + R_MEASURE);
    let v_new = v_pred + K * (positionData.spd - v_pred);
    let P_new = (1 - K) * P_pred;
    
    currentSpeedMs = v_new;
    ukfState.P[2][2] = P_new;

    const kalmanUncertainty = Math.sqrt(P_new);

    return {
        speed: v_new,
        uncertainty: kalmanUncertainty,
        ekfStatus: kalmanUncertainty < MAX_ACC_UNCERT ? 'Actif' : 'Estimation Seule'
    };
};
// =================================================================
// BLOC 3/4 : Affichage, Astro et Boucles de Mise à Jour
// Fichier: data_processing_and_dom.js
// Dépendance: BLOC 1, BLOC 2 (Fonctions de calcul)
// =================================================================

// --- FONCTIONS ASTRO ET HORLOGE ---

const updateMinecraftClock = (ms) => {
    // ... (Logique Minecraft Clock) ...
};

/** Met à jour l'affichage de l'heure locale et UTC (NTP). */
const updateTimeDisplay = () => {
    // ... (Logique Heure Locale/UTC/Temps écoulé) ...
    const now = new Date();
    // (Implémentation détaillée omise pour la concision, utilise les variables globales timeElapsedMs, lastTimestamp, etc.)
};

const formatHoursToTime = (hours) => {
    // ... (Logique de formatage HH:MM:SS) ...
};

const getMoonPhaseName = (phase) => {
    // ... (Logique pour déterminer le nom de la phase lunaire) ...
};

/** Mise à jour de l'astronomie (utilise SunCalc.js). */
const updateAstro = (lat, lon) => {
    if (typeof SunCalc === 'undefined') return;
    // ... (Implémentation détaillée des calculs SunCalc, MST, TST, EOT) ...
    // NOTE: Tous les ID du DOM pour l'Astro sont mis à jour ici.
    // $('sun-alt').textContent = dataOrDefault(altitudeSunDeg, 2, '°');
};

// --- BOUCLES DE MISE À JOUR ---

/** Boucle rapide (Déclenchée par GNSS) : UKF, Vitesse, Relativité, Distance */
const runFastLoop = (positionData, deltaTime) => {
    if (isEmergencyStop || isGpsPaused) return;

    const ekfResults = runUKFStep(positionData, deltaTime);

    const v_stable = ekfResults.speed;
    const v_kmh = v_stable * KMH_MS;
    // ... (Calculs et mise à jour de Vitesse, Relativité, Énergie, Momentum) ...

    const distanceDelta = v_stable * deltaTime * distanceRatio;
    totalDistanceM += distanceDelta;
    if (v_stable >= MIN_SPD_THRESH) {
        timeMovingMs += deltaTime * 1000;
        maxSpeedKmh = Math.max(maxSpeedKmh, v_kmh);
    }
    
    // Mise à jour des affichages rapides
    $('distance-total-km').textContent = `${dataOrDefault(totalDistanceM / 1000, 3, ' km')} | ${dataOrDefault(totalDistanceM, 2, ' m')}`;
    $('speed-max').textContent = dataOrDefault(maxSpeedKmh, 1, ' km/h');
    $('kalman-uncert').textContent = dataOrDefault(ekfResults.uncertainty, 4, ' m/s');
    $('gps-status-ekf').textContent = ekfResults.ekfStatus;
};

/** Boucle lente (1000ms) : Météo, Forces, Moyennes, Astro */
const runSlowLoop = () => {
    // 1. Calculs Météo/Air (utilise lastT_K, lastP_hPa, lastH_perc)
    const T_C = lastT_K - 273.15;
    currentAirDensity = getAirDensity(lastP_hPa, lastT_K, lastH_perc);
    currentSpeedOfSound = getSpeedOfSound(lastT_K);
    const dewPointC = getDewPoint(T_C, lastH_perc); 
    // ... (Mise à jour des champs Météo/Son/Mach) ...
    $('dew-point').textContent = dataOrDefault(dewPointC, 1, ' °C'); 

    // 2. Calculs Dynamique et Forces
    // ... (Calculs et mise à jour Pression dynamique, Traînée, Coriolis, Énergie Cinétique) ...

    // 3. Calcul des vitesses moyennes (CORRECTION)
    const totalSeconds = timeElapsedMs / 1000;
    const totalMovingSeconds = timeMovingMs / 1000;

    const avgSpeedTotalMs = (totalSeconds > 0) ? totalDistanceM / totalSeconds : 0;
    $('vitesse-moyenne-totale').textContent = dataOrDefault(avgSpeedTotalMs * KMH_MS, 1, ' km/h');

    const avgSpeedMovingMs = (totalMovingSeconds > 0) ? totalDistanceM / totalMovingSeconds : 0;
    $('vitesse-moyenne-mvt').textContent = dataOrDefault(avgSpeedMovingMs * KMH_MS, 1, ' km/h');

    // 4. Mise à jour de l'Astro
    updateAstro(lastEKFPosition.lat, lastEKFPosition.lon);
    
    // 5. Mise à jour des données générales
    $('mass-display').textContent = dataOrDefault(currentMass, 3, ' kg');
    $('gravity-base').textContent = dataOrDefault(CELESTIAL_BODIES[selectedCelestialBody].gravity, 4, ' m/s²');
};
// =================================================================
// BLOC 4/4 : Gestion des Capteurs et Initialisation
// Fichier: sensor_and_init.js
// Dépendance: BLOC 1, BLOC 3 (runFastLoop, runSlowLoop)
// =================================================================

// --- GESTION DES CAPTEURS RÉELS (GNSS & IMU) ---

/** Traite les données GNSS reçues par watchPosition. */
const handleGpsUpdate = (position) => {
    if (isGpsPaused || isEmergencyStop) return;

    const currentGpsTimestamp = position.timestamp;
    const deltaTime = lastGpsTimestamp > 0 ? (currentGpsTimestamp - lastGpsTimestamp) / 1000 : 0.1;
    lastGpsTimestamp = currentGpsTimestamp;

    // Données réelles
    const realPositionData = {
        lat: position.coords.latitude,
        lon: position.coords.longitude,
        alt: position.coords.altitude || 0, 
        spd: position.coords.speed || 0.0,  
        acc: position.coords.accuracy,
        heading: position.coords.heading || lastEKFPosition.heading 
    };

    // Mise à jour de la position EKF et exécution de la boucle rapide
    lastEKFPosition.lat = realPositionData.lat;
    lastEKFPosition.lon = realPositionData.lon;
    // ... (Mise à jour des autres champs de position) ...
    
    runFastLoop(realPositionData, deltaTime);

    // Mise à jour des coordonnées EKF (affichage)
    $('latitude-ekf').textContent = dataOrDefault(realPositionData.lat, 6, '°');
    $('longitude-ekf').textContent = dataOrDefault(realPositionData.lon, 6, '°');
    $('altitude-ekf').textContent = dataOrDefault(realPositionData.alt, 2, ' m');
    $('cap-direction').textContent = dataOrDefault(realPositionData.heading, 1, '°');
    $('speed-status-text').textContent = 'Signal OK';
};

/** Gère les erreurs de l'API Geolocation. */
const handleGpsError = (error) => {
    console.error(`Erreur GPS (${error.code}): ${error.message}`);
    $('speed-status-text').textContent = `ERREUR GPS: ${error.message.substring(0, 30)}`;
    if (error.code === 1 || error.code === 2) {
        if (!isGpsPaused) toggleGPS(); 
    }
};

/** Traite les données de l'IMU (Accéléromètre/Gyroscope). */
const handleImuUpdate = (event) => {
    imuActive = true;
    $('sensor-status').textContent = 'Actif'; 
    
    const acc = event.accelerationIncludingGravity;
    // ... (Mise à jour des champs Accélération X, Y, Z, Forces G, etc.) ...
    $('accel-x').textContent = dataOrDefault(acc.x, 3, ' m/s²');
};

const initImuListener = () => {
    if (typeof window.DeviceMotionEvent !== 'undefined') {
        window.addEventListener('devicemotion', handleImuUpdate);
        $('sensor-status').textContent = 'Actif (Attente données)';
    } else {
        $('sensor-status').textContent = 'Inactif (API Capteur manquante)';
    }
};

// --- LOGIQUE DE CONTRÔLE ---

/** Démarre l'acquisition GPS via watchPosition. */
const startGpsAcquisition = () => {
    if (typeof navigator.geolocation === 'undefined') {
        alert("L'API de Géolocalisation n'est pas supportée par ce navigateur.");
        return;
    }
    
    if (gpsWatchId !== null) {
        navigator.geolocation.clearWatch(gpsWatchId);
    }
    
    const gpsOptions = {
        enableHighAccuracy: true,
        timeout: 10000,
        maximumAge: 1000 
    };

    gpsWatchId = navigator.geolocation.watchPosition(handleGpsUpdate, handleGpsError, gpsOptions);
    isGpsPaused = false;
    $('toggle-gps-btn').innerHTML = '⏸️ PAUSE GPS';
    $('toggle-gps-btn').style.backgroundColor = '#ffc107'; 
    $('speed-status-text').textContent = 'Acquisition GPS...';
};

/** Gère le bouton MARCHE/PAUSE GPS. */
const toggleGPS = () => {
    if ($('toggle-gps-btn').innerHTML.includes('MARCHE') || isGpsPaused) {
        startGpsAcquisition();
    } else {
        isGpsPaused = true;
        if (gpsWatchId !== null) {
             navigator.geolocation.clearWatch(gpsWatchId);
             gpsWatchId = null;
        }
        $('toggle-gps-btn').innerHTML = '▶️ MARCHE GPS';
        $('toggle-gps-btn').style.backgroundColor = '#28a745';
        $('speed-status-text').textContent = 'PAUSE GPS';
    }
};

const emergencyStop = () => { /* ... (Logique d'arrêt d'urgence) ... */ };
const resetDistance = () => { /* ... (Logique de réinitialisation de distance) ... */ };
const resetVmax = () => { /* ... (Logique de réinitialisation V-Max) ... */ };
const resetAll = () => { /* ... (Logique de réinitialisation complète) ... */ };


/** Point d'entrée principal. */
const initDashboard = () => {
    // 1. Initialisation des listeners pour tous les contrôles
    $('toggle-gps-btn').addEventListener('click', toggleGPS);
    $('emergency-stop-btn').addEventListener('click', emergencyStop);
    // ... (Ajouter tous les autres listeners d'input et de contrôle) ...

    // 2. Initialisation des Capteurs
    initImuListener();
    
    // 3. Démarrage des boucles de mise à jour lente
    updateTimeDisplay();
    runSlowLoop();
    setInterval(updateTimeDisplay, 1000);
    setInterval(runSlowLoop, 1000); 
    
    // 4. Configuration initiale de l'affichage
    $('toggle-gps-btn').style.backgroundColor = '#28a745';
    $('speed-status-text').textContent = 'Attente du signal GPS...';
};

// Exécute l'initialisation après le chargement complet du DOM
window.addEventListener('load', initDashboard);
