// =================================================================
// FICHIER JS PARTIE 1 : gnss-dashboard-part1.js (Constantes et Utilitaires)
// =================================================================

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
const G_ACC = 9.80665; // Accélération standard de la gravité (m/s²)
const C_L = 299792458; // Vitesse de la lumière dans le vide (m/s)
const R2D = 180 / Math.PI; // Radians to Degrees
const D2R = Math.PI / 180; // Degrees to Radians
const W_EARTH = 7.292115e-5; // Vitesse angulaire de rotation de la Terre (rad/s)
const KMH_MS = 3.6; // Multiplicateur pour m/s en km/h
const dayMs = 86400000; // Millisecondes par jour
const SPEED_SOUND = 340.29; // Vitesse du Son (m/s)

// --- CONSTANTES EKF ET DYNAMIQUE ---
const MIN_DT = 0.05; // Temps minimum pour les calculs de dérivée
const GPS_NOISE_FACTOR = 0.15; // Facteur pour convertir la précision en bruit de vitesse
const VEL_NOISE_FACTOR = 5.0; // Facteur pour seuil ZVU (m/s par mètre de bruit)
const MAX_ACC = 50.0; // Précision GPS maximale acceptable (m)
const GOOD_ACC_THRESHOLD = 5.0; // Précision GPS considérée comme bonne (m)
const ALT_TH = 10.0; // Seuil pour le mode souterrain (m)
const STATIC_ACCEL_THRESHOLD = 0.5; // Seuil pour l'IMU (m/s²)
const ACCEL_FILTER_ALPHA = 0.9; // Coefficient de lissage passe-bas de l'IMU (0.9 = 90% ancien)
const DOM_SLOW_UPDATE_MS = 1000; // Rafraîchissement lent pour l'heure/date
const WEATHER_UPDATE_MS = 30000; // Rafraîchissement météo (30 secondes)

// --- CONSTANTES MAGNÉTIQUES ---
const MAG_FIELD_THRESHOLD_UT = 100.0; // Seuil de champ magnétique (microTesla) pour considérer une interférence (Typique: 25-65 µT)
const MAG_NOISE_FACTOR = 0.05; // Facteur d'influence du champ magnétique sur le bruit EKF

// --- CONSTANTES NAVIGATION ET GÉODÉSIE ---
const TARGET_LAT = 48.8584; // Latitude de la Tour Eiffel (exemple de cible)
const TARGET_LON = 2.2945; // Longitude de la Tour Eiffel (exemple de cible)

// --- CONSTANTES SYSTÈME ET API ---
const NETHER_RATIO = 8; // Ratio de distance pour le mode Nether
const OWM_API_KEY = "VOTRE_CLE_API_OPENWEATHERMAP"; // Remplacer
const OWM_API_URL = "https://api.openweathermap.org/data/2.5/weather";
const GEOID_SEPARATION_M = 43.14; // Séparation géoïde EGM96 (exemple)

// Facteurs de bruit EKF par environnement
const R_NORMAL = 1.0; 
const R_FOREST = 1.5;
const R_CONCRETE = 3.0;
const R_METAL = 2.5; 
const R_GPS_DISABLED = 1000.0; // Bruit très élevé pour la vitesse si pas de GPS

// Options du GPS Watcher
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, timeout: 5000, maximumAge: 0 },
    LOW_FREQ: { enableHighAccuracy: false, timeout: 10000, maximumAge: 60000 }
};

// --- VARIABLES GLOBALES ---
let lat = 0.0, lon = 0.0;
let wID = null; // ID du GPS watchPosition
let lPos = null; // Dernière position enregistrée
let sTime = null; // Temps de début de session
let distM = 0.0; // Distance totale parcourue (m)
let maxSpd = 0.0; // Vitesse maximale (m/s)
let timeMoving = 0.0; // Temps passé au-dessus du seuil de vitesse
let lastFSpeed = 0.0; // Dernière vitesse filtrée (pour calcul accel.)
let currentGPSMode = 'HIGH_FREQ'; 
let selectedEnvironment = 'NORMAL';
let netherMode = false;
let emergencyStopActive = false;

// Variables du filtre Kalman (Vitesse Horizontale 3D)
let kSpd = 0.0; // Vitesse EKF filtrée (m/s)
let kUncert = 10.0; // Incertitude (variance) de la vitesse (m²/s²) - CORRIGÉ (était 1000.0)

// Variables du filtre Kalman (Altitude)
let kAlt = 0.0; // Altitude EKF filtrée (m)
let kAltUncert = 10.0; // Incertitude de l'altitude (m²) - CORRIGÉ (était 1000.0)

// Variables IMU
let global_pitch = 0.0; // Tangage (rad)
let global_roll = 0.0; // Roulis (rad)
let kAccel = { x: 0, y: 0, z: 0 }; // Accélération lissée (incluant gravité)
let latestLinearAccelMagnitude = 0.0; // Magnitude de l'accélération linéaire (sans gravité)
let latestVerticalAccelIMU = 0.0; // Accélération verticale linéaire (sans gravité)
let G_STATIC_REF = { x: 0, y: 0, z: 0 }; // Référence de gravité statique
let latestMagneticFieldMagnitude = 0.0; // Magnitude du champ magnétique (µT)
let currentHeading = 0.0; // Cap actuel de l'appareil (pour la boussole standard)

// Variables Météo
let lastP_hPa = 1013.25; // Pression atmosphérique (hPa)
let lastAirDensity = 1.225; // Densité de l'air (kg/m³)

// Variables Astro/Temps
let domID = null;
let weatherID = null;
let zvuLockTime = 0;
let isZVUActive = false;

// Variables Globe 3D (Three.js)
let scene, camera, renderer, controls;
let marker; 

// --- FONCTIONS UTILITAIRES ---

function $(id) { return document.getElementById(id); }

/** Convertit une date en jours juliens à partir de J2000. */
function toDays(date) { return (date.getTime() / dayMs) - 10957.5; }

/** Calcule l'anomalie solaire moyenne. */
function solarMeanAnomaly(d) { return (D2R * 356.047 + d * D2R * 0.98560028) % (2 * Math.PI); }

/** Calcule la longitude écliptique du Soleil. */
function eclipticLongitude(M) {
    const C = D2R * (1.914 * Math.sin(M) + 0.02 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M)); // Équation du centre
    const P = D2R * 102.9377; // Longitude du périhélie
    return (M + C + P + Math.PI) % (2 * Math.PI); 
}

/** Calcule le point de rosée (approximation Magnus-Tetens). */
function calculateDewPoint(tempC, humidity) {
    const A = 17.625;
    const B = 243.04;
    const alpha = Math.log(humidity / 100) + (A * tempC) / (B + tempC);
    return (B * alpha) / (A - alpha);
}

/** Renvoie le nom de la phase lunaire. */
function getMoonPhaseName(phase) {
    if (phase < 0.03 || phase > 0.97) return "Nouvelle Lune 🌑";
    if (phase < 0.22) return "Premier Croissant 🌒";
    if (phase < 0.28) return "Premier Quartier 🌓";
    if (phase < 0.47) return "Lune Gibbeuse Croissante 🌔";
    if (phase < 0.53) return "Pleine Lune 🌕";
    if (phase < 0.72) return "Lune Gibbeuse Décroissante 🌖";
    if (phase < 0.78) return "Dernier Quartier 🌗";
    return "Dernier Croissant 🌘";
}

/** Synchronise l'heure locale au démarrage. */
function syncH() {
    window.cDate = new Date();
    if ($('local-time')) $('local-time').textContent = window.cDate.toLocaleTimeString();
    if ($('date-display')) $('date-display').textContent = window.cDate.toLocaleDateString();
}

/** Renvoie l'objet Date actuel. */
function getCDate() { return window.cDate || new Date(); }

/** Formatte le temps en heures:minutes:secondes pour TST/MST. */
function formatTime(totalMinutes) {
    const mins = (totalMinutes + 1440) % 1440; 
    const hours = Math.floor(mins / 60);
    const minutes = Math.floor(mins % 60);
    const seconds = Math.floor((totalMinutes * 60) % 60);
    return `${String(hours).padStart(2, '0')}:${String(minutes).padStart(2, '0')}:${String(seconds).padStart(2, '0')}`;
}

/** Fonction de prédiction/mise à jour du filtre EKF (Vitesse). */
function kFilter(measurement, dt, R, accel_control) {
    // 1. Prediction (Modèle de mouvement simple : Accélération constante)
    kSpd += accel_control * dt; // Nouvelle vitesse prédite (basée sur l'IMU)
    kUncert += R * dt; // Augmentation de l'incertitude avec le temps
    
    // 2. Mise à jour (Correction avec la mesure GPS brute)
    const K = kUncert / (kUncert + R); // Gain de Kalman
    kSpd += K * (measurement - kSpd); // Correction de la vitesse
    kUncert *= (1 - K); // Réduction de l'incertitude
    
    // Clamping: La vitesse filtrée ne peut pas être négative.
    return Math.max(0, kSpd); 
}

/** Fonction de filtre EKF simplifiée pour l'Altitude. */
function kFilterAltitude(measurement, accuracy, dt, accel_control_V) {
    // Ce filtre simplifie en filtrant la position (altitude) directement
    
    // 1. Prediction
    let kAlt_v_pred = 0; // Vitesse verticale non filtrée
    if (lPos && lPos.kAltUncert_old !== undefined && dt > MIN_DT) {
        kAlt_v_pred = (kAlt - lPos.kAlt_old) / dt; // Approximation de la vitesse verticale
    }
    kAlt_v_pred += accel_control_V * dt; // Influence de l'accélération
    kAlt += kAlt_v_pred * dt; // Prédiction de la nouvelle altitude
    
    // Augmentation de l'incertitude
    kAltUncert += 1.0 * dt; 
    
    // 2. Mise à jour (Correction avec la mesure GPS brute si disponible)
    if (measurement !== null) {
        const R_alt = accuracy * accuracy; // Variance de la mesure GPS (R)
        const K = kAltUncert / (kAltUncert + R_alt); // Gain de Kalman
        kAlt += K * (measurement - kAlt); // Correction
        kAltUncert *= (1 - K); // Réduction de l'incertitude
    }
    
    return kAlt; 
}

/** Sélectionne le bruit R de l'EKF en fonction de l'environnement, altitude, et magnétisme. */
function getKalmanR(effectiveAcc, alt, pressurehPa) {
    const noiseFromGPS = effectiveAcc * effectiveAcc; 
    let baseR = noiseFromGPS * GPS_NOISE_FACTOR;

    // --- 1. Facteur Environnemental (Multiplicatif) ---
    switch (selectedEnvironment) {
        case 'FOREST': baseR *= R_FOREST; break;
        case 'CONCRETE': baseR *= R_CONCRETE; break;
        case 'METAL': baseR *= R_METAL; break;
        default: baseR *= R_NORMAL; 
    }
    
    // --- 2. Facteur Souterrain (Altitude) ---
    if (alt !== null && alt < ALT_TH) {
        baseR *= 1.5; // Augmente le bruit en souterrain
    }
    
    // --- 3. Variance Magnétique de Vitesse (VMV) (Additif) ---
    let magVariance = 0.0;
    if (latestMagneticFieldMagnitude > MAG_FIELD_THRESHOLD_UT) {
        const excessMag = latestMagneticFieldMagnitude - MAG_FIELD_THRESHOLD_UT;
        magVariance = excessMag * MAG_NOISE_FACTOR; 
        baseR += magVariance;
    }
    
    if ($('mag-variance-display')) {
         $('mag-variance-display').textContent = `+${magVariance.toFixed(3)} m² ${magVariance > 0.001 ? '(Interférence)' : ''}`;
    }
    
    // Le bruit minimal assure la stabilité du filtre même si l'erreur GPS est 0.
    return Math.max(baseR, 0.001); 
}

/** Calcule la distance entre deux points GPS (mètres) - Formule Haversine. */
function calculateDistanceHaversine(lat1, lon1, lat2, lon2) {
    const R = 6371e3; // Rayon moyen de la Terre en mètres
    const phi1 = lat1 * D2R;
    const phi2 = lat2 * D2R;
    const deltaPhi = (lat2 - lat1) * D2R;
    const deltaLambda = (lon2 - lon1) * D2R;

    const a = Math.sin(deltaPhi / 2) * Math.sin(deltaPhi / 2) +
              Math.cos(phi1) * Math.cos(phi2) *
              Math.sin(deltaLambda / 2) * Math.sin(deltaLambda / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

    return R * c;
    }
// =================================================================
// FICHIER
// =================================================================
// FICHIER JS PARTIE 2, BLOC A : gnss-dashboard-part2_blocA.js (Variables, EKF, APIs, Capteurs & updateDisp)
// =================================================================

// -------------------------------------------
// 1. VARIABLES GLOBALES (Déclarations initiales)
// -------------------------------------------

let wID = null; 
let domID = null; 
let weatherID = null; 
let lat = 0.0;
let lon = 0.0;
let kSpd = 0.0; 
let kAlt = 0.0; 
let kUncert = 10.0; 
let kAltUncert = 10.0; 
let lPos = null; 
let distM = 0.0; 
let maxSpd = 0.0;
let timeMoving = 0.0;
let sTime = new Date().getTime(); 

// IMU/Orientation
let kAccel = { x: 0, y: 0, z: 0 }; 
let global_pitch = 0; 
let global_roll = 0; 
let currentHeading = 0; 
let latestLinearAccelMagnitude = 0.0; 
let latestVerticalAccelIMU = 0.0; 
let latestMagneticFieldMagnitude = 0.0; 
let accel_long = 0.0; 

// Contrôles & État
let currentGPSMode = 'medium_accuracy';
let isGPSRunning = false;
let isGPSTracking = false;
let emergencyStopActive = false;
let isZVUActive = false;
let zvuLockTime = 0;
let netherMode = false;
let selectedEnvironment = 'Earth'; 
let lastFSpeed = 0.0; 

// Three.js (Déclarations nécessaires pour updateGlobe)
let marker; 
let speedVector, accelVector; 
let trajectoryLine; 
const MAX_TRAJECTORY_POINTS = 500; 
const R_GLOBE = 2.0; 

// -------------------------------------------
// 2. FONCTIONS UTILITAIRES (EKF et Mathématiques)
// -------------------------------------------

function $(id) { return document.getElementById(id); }
function getCDate() { return new Date(); }
function formatTime(totalMinutes) {
    const minutes = Math.round(totalMinutes % 60);
    const hours = Math.floor(totalMinutes / 60) % 24;
    return `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}`;
}
function calculateDistanceHaversine(lat1, lon1, lat2, lon2) {
    const R = 6371000; 
    const dLat = (lat2 - lat1) * D2R;
    const dLon = (lon2 - lon1) * D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1 * D2R) * Math.cos(lat2 * D2R) * Math.sin(dLon / 2) ** 2;
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R * c; 
}
// Fonctions EKF simulées (doivent être définies dans part1.js ou globales)
function getKalmanR(acc, alt, pressure) {
    let R_gps = acc * acc; 
    // R_dyn est une fonction de l'incertitude GPS et de la vitesse
    return R_gps * (1 + (Math.abs(kSpd) / 5) ** 2); 
}
function kFilter(z, dt, R, u) { 
    let Q = (u * dt) * (u * dt) / 2; 
    let P = kUncert + Q;
    let K = P / (P + R);
    let x = kSpd + K * (z - kSpd);
    kUncert = (1 - K) * P;
    kSpd = x;
    return x;
}
function kFilterAltitude(z_alt, R_alt, dt, u_alt) {
    let Q_alt = (u_alt * dt) * (u_alt * dt) / 2; 
    let P_alt = kAltUncert + Q_alt;
    let K_alt = P_alt / (P_alt + R_alt * R_alt); 
    let x_alt = kAlt + K_alt * (z_alt - kAlt);
    kAltUncert = (1 - K_alt) * P_alt;
    kAlt = x_alt;
    return x_alt;
}

//
// =================================================================
// FICHIER JS PARTIE 2, BLOC A : gnss-dashboard-part2_blocA.js (Variables, EKF, APIs, Capteurs & updateDisp)
// =================================================================

// -------------------------------------------
// 1. VARIABLES GLOBALES (Déclarations initiales)
// -------------------------------------------

let wID = null; // Watch ID pour Geolocation
let domID = null; // Intervalle DOM lent
let weatherID = null; // Intervalle Météo
let lat = 0.0;
let lon = 0.0;
let kSpd = 0.0; // Vitesse EKF stable (m/s)
let kAlt = 0.0; // Altitude EKF stable (m)
let kUncert = 10.0; // Incertitude EKF vitesse (m²/s²)
let kAltUncert = 10.0; // Incertitude EKF altitude (m²)
let lPos = null; // Dernière position reçue
let distM = 0.0; // Distance totale parcourue (m)
let maxSpd = 0.0;
let timeMoving = 0.0;
let sTime = new Date().getTime(); // Temps de démarrage de la session

// IMU/Orientation
let kAccel = { x: 0, y: 0, z: 0 }; // Accélération filtrée
let global_pitch = 0; // Tangage (rad)
let global_roll = 0; // Roulis (rad)
let currentHeading = 0; // Cap (degré)
let latestLinearAccelMagnitude = 0.0; // Accélération Linéaire 3D (m/s²)
let latestVerticalAccelIMU = 0.0; // Accélération Linéaire Verticale (m/s²)
let latestMagneticFieldMagnitude = 0.0; // Magnitude du champ Magnétique (µT)
let accel_long = 0.0; // Accélération longitudinale (calculée dans updateDisp)

// Contrôles & État
let currentGPSMode = 'medium_accuracy';
let isGPSRunning = false;
let isGPSTracking = false;
let emergencyStopActive = false;
let isZVUActive = false;
let zvuLockTime = 0;
let netherMode = false;
let selectedEnvironment = 'Earth'; 
let lastFSpeed = 0.0; 

// Three.js (Déclarations minimales pour updateDisp)
let marker, speedVector, accelVector, trajectoryLine; 
const MAX_TRAJECTORY_POINTS = 500; 
const R_GLOBE = 2.0; 
// Les autres variables Three.js (scene, camera, renderer, etc.) sont dans le Bloc B.


// -------------------------------------------
// 2. FONCTIONS UTILITAIRES (EKF et Mathématiques)
// -------------------------------------------

function $(id) { return document.getElementById(id); }
function getCDate() { return new Date(); }
function formatTime(totalMinutes) {
    const minutes = Math.round(totalMinutes % 60);
    const hours = Math.floor(totalMinutes / 60) % 24;
    return `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}`;
}
function calculateDistanceHaversine(lat1, lon1, lat2, lon2) {
    const R = 6371000; 
    const dLat = (lat2 - lat1) * D2R;
    const dLon = (lon2 - lon1) * D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1 * D2R) * Math.cos(lat2 * D2R) * Math.sin(dLon / 2) ** 2;
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R * c; 
}

// Fonctions EKF (doivent dépendre des constantes de part1.js)
function getKalmanR(acc, alt, pressure) {
    let R_gps = acc * acc; 
    return R_gps * (1 + (Math.abs(kSpd) / 5) ** 2); // Dynamique basée sur la vitesse
}
function kFilter(z, dt, R, u) { 
    let Q = (u * dt) * (u * dt) / 2; // Bruit de processus basé sur l'accélération
    let P = kUncert + Q;
    let K = P / (P + R);
    let x = kSpd + K * (z - kSpd);
    kUncert = (1 - K) * P;
    kSpd = x;
    return x;
}
function kFilterAltitude(z_alt, R_alt, dt, u_alt) {
    let Q_alt = (u_alt * dt) * (u_alt * dt) / 2; 
    let P_alt = kAltUncert + Q_alt;
    let K_alt = P_alt / (P_alt + R_alt * R_alt); 
    let x_alt = kAlt + K_alt * (z_alt - kAlt);
    kAltUncert = (1 - K_alt) * P_alt;
    kAlt = x_alt;
    return x_alt;
}

// -------------------------------------------
// 3. FONCTIONS ASTRO
// -------------------------------------------
// Fonctions utilitaires astronomiques (nécessitent suncalc et constantes)
function toDays(date) { return date.getUTCDate() + (date.getUTCHours() + date.getUTCMinutes() / 60 + date.getUTCSeconds() / 3600) / 24; }
function solarMeanAnomaly(d) { return D2R * (357.5291 + 0.98560028 * d) % (2 * Math.PI); }
function eclipticLongitude(M) {
    let L = M + D2R * (1.9148 * Math.sin(M) + 0.02 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M));
    return L % (2 * Math.PI);
}
function getMoonPhaseName(fraction) {
    if (fraction === 0) return 'Nouvelle Lune';
    if (fraction < 0.25) return 'Premier Croissant';
    if (fraction === 0.25) return 'Premier Quartier';
    if (fraction < 0.5) return 'Gibbeuse Croissante';
    if (fraction === 0.5) return 'Pleine Lune';
    if (fraction < 0.75) return 'Gibbeuse Décroissante';
    if (fraction === 0.75) return 'Dernier Quartier';
    return 'Dernier Croissant';
}
function getSolarTime(date, latA, lonA) {
    const d = toDays(date);
    const M = solarMeanAnomaly(d);
    const L = eclipticLongitude(M);

    const E = R2D * (M + 2 * Math.PI - L) / (2 * Math.PI) * 1440 / 360; 
    const UT = date.getUTCHours() + date.getUTCMinutes() / 60 + date.getUTCSeconds() / 3600;
    const LSM_hours = UT + lonA / 15;
    const LSM_mins = LSM_hours * 60;
    const TST_mins = LSM_mins + E;
    
    return { TST: TST_mins, LSM: LSM_mins, EOT: E, L: L };
}

function updateAstro(latA, lonA) {
    if (latA === 0.0 && lonA === 0.0) return;
    
    const date = getCDate();
    const times = suncalc.getTimes(date, latA, lonA);
    const moon = suncalc.getMoonIllumination(date);
    const sunPos = suncalc.getPosition(date, latA, lonA);
    const solarTime = getSolarTime(date, latA, lonA);
    
    if ($('tst')) $('tst').textContent = formatTime(solarTime.TST);
    if ($('moon-phase-display')) $('moon-phase-display').textContent = getMoonPhaseName(moon.fraction);
    // ... (Mise à jour des autres champs astro) ...
}

// -------------------------------------------
// 4. FONCTIONS MÉTÉO (Simulation)
// -------------------------------------------

async function updateWeather(latA, lonA) {
    if (latA === 0.0 && lonA === 0.0) return;
    
    const temp = 15 + Math.sin(Date.now() / 100000) * 10;
    const pressure = 1013.25 + Math.cos(Date.now() / 50000) * 10;
    const humidity = 50 + Math.sin(Date.now() / 70000) * 20;

    if ($('air-temp')) $('air-temp').textContent = `${temp.toFixed(1)} °C (Sim.)`;
    if ($('air-pressure')) $('air-pressure').textContent = `${pressure.toFixed(2)} hPa (Sim.)`;
    if ($('air-humidity')) $('air-humidity').textContent = `${humidity.toFixed(1)} % (Sim.)`;
}

// -------------------------------------------
// 5. FONCTIONS CAPTEURS INERTIELS (IMU, Mag)
// -------------------------------------------

function handleDeviceMotion(event) {
    if (emergencyStopActive) return;
    
    let acc_x_lin, acc_y_lin, acc_z_lin;

    if (event.acceleration && event.acceleration.x !== null) {
        // Option 1: Utiliser l'API acceleration (sans gravité)
        acc_x_lin = event.acceleration.x; acc_y_lin = event.acceleration.y; acc_z_lin = event.acceleration.z;
        kAccel.x = ACCEL_FILTER_ALPHA * kAccel.x + (1 - ACCEL_FILTER_ALPHA) * acc_x_lin;
        kAccel.y = ACCEL_FILTER_ALPHA * kAccel.y + (1 - ACCEL_FILTER_ALPHA) * acc_y_lin;
        kAccel.z = ACCEL_FILTER_ALPHA * kAccel.z + (1 - ACCEL_FILTER_ALPHA) * acc_z_lin;
        acc_x_lin = kAccel.x; acc_y_lin = kAccel.y; acc_z_lin = kAccel.z;

    } else if (event.accelerationIncludingGravity && event.accelerationIncludingGravity.x !== null) {
        // Option 2: Utiliser accelerationIncludingGravity et soustraire la gravité
        const acc_g = event.accelerationIncludingGravity;
        kAccel.x = ACCEL_FILTER_ALPHA * kAccel.x + (1 - ACCEL_FILTER_ALPHA) * acc_g.x;
        kAccel.y = ACCEL_FILTER_ALPHA * kAccel.y + (1 - ACCEL_FILTER_ALPHA) * acc_g.y;
        kAccel.z = ACCEL_FILTER_ALPHA * kAccel.z + (1 - ACCEL_FILTER_ALPHA) * acc_g.z;
        
        // Correction de l'Inclinaison (Gravité projetée)
        const G_CORR_X = -G_ACC * Math.sin(global_pitch); 
        const G_CORR_Y = G_ACC * Math.sin(global_roll) * Math.cos(global_pitch); 
        const G_CORR_Z = G_ACC * Math.cos(global_roll) * Math.cos(global_pitch); 
        
        acc_x_lin = kAccel.x - G_CORR_X;
        acc_y_lin = kAccel.y - G_CORR_Y;
        acc_z_lin = kAccel.z - G_CORR_Z; 
    } else { return; }

    const magnitude_lin = Math.sqrt(acc_x_lin ** 2 + acc_y_lin ** 2 + acc_z_lin ** 2);
    let accel_vertical_lin_raw = acc_z_lin; 
    
    // Application du seuil statique (ZVU IMU)
    if (magnitude_lin < STATIC_ACCEL_THRESHOLD) { 
        latestLinearAccelMagnitude = 0.0;
        latestVerticalAccelIMU = 0.0;
    } else {
        latestLinearAccelMagnitude = magnitude_lin;
        latestVerticalAccelIMU = accel_vertical_lin_raw;
    }

    // Affichage IMU
    if ($('accel-vertical-imu')) $('accel-vertical-imu').textContent = `${latestVerticalAccelIMU.toFixed(3)} m/s²`;
    if ($('accel-3d-imu')) $('accel-3d-imu').textContent = `${latestLinearAccelMagnitude.toFixed(3)} m/s²`;
}

function handleDeviceOrientation(event) {
    if (emergencyStopActive) return;
    if (event.alpha !== null) {
        global_pitch = event.beta * D2R; 
        global_roll = event.gamma * D2R;
    }
}

function handleMagnetometer(event) {
    if (emergencyStopActive) return;
    
    if (event.absolute && event.alpha !== null) {
        currentHeading = event.alpha; 
    }

    // Simulation de la magnitude magnétique
    latestMagneticFieldMagnitude = 50 + (Math.sin(Date.now() / 5000) * 5) + Math.random() * 2; 
    
    if ($('magnetic-field')) {
        $('magnetic-field').textContent = `${latestMagneticFieldMagnitude.toFixed(2)} µT`;
    }
    
    updateCompasses(lat, lon); 
}

function calculateBearing(lat1, lon1, lat2, lon2) {
    // ... (Logique de calcul du cap) ...
    return 0; // Simplifié pour l'exemple
}

function updateCompasses(latA, lonA) {
    if ($('compass-heading')) {
        $('compass-heading').textContent = `${currentHeading.toFixed(1)} °`;
        const needle = $('compass-needle');
        if (needle) { needle.style.transform = `rotate(${-currentHeading}deg)`; }
    }
    // ... (Logique de mise à jour des boussoles) ...
}


// -------------------------------------------
// 6. FONCTION PRINCIPALE DE MISE À JOUR (GPS/EKF)
// -------------------------------------------

function updateDisp(pos) {
    if (emergencyStopActive) return;
    
    const cTimePos = pos.timestamp; 
    lat = pos.coords.latitude; 
    lon = pos.coords.longitude;
    const alt_gps_raw = pos.coords.altitude; 
    const acc = pos.coords.accuracy;
    const spd_raw_gps = pos.coords.speed ?? 0;

    const dt = lPos ? (cTimePos - lPos.timestamp) / 1000 : 0.2;
    // ... (Logique EKF, ZVU, calcul d'accel_long, mise à jour de kSpd et kAlt) ...
    
    let effectiveAcc = acc;
    // ... (Calculs spdV_raw, spd3D_raw, initialisation EKF) ...

    // --- FILTRAGE EKF ---
    let accel_control_3D = latestLinearAccelMagnitude;
    let accel_control_V = latestVerticalAccelIMU;
    // (Logique ZVU)
    if (isZVUActive) { accel_control_3D = 0.0; accel_control_V = 0.0; }

    let R_dyn = getKalmanR(effectiveAcc, alt_gps_raw, 1013); 
    const sSpdFE = kFilter(spd3D_raw, dt, R_dyn, accel_control_3D); 
    const kAlt_new = kFilterAltitude(alt_gps_raw, effectiveAcc, dt, accel_control_V);

    accel_long = (dt > 0.05) ? (sSpdFE - lastFSpeed) / dt : 0;
    lastFSpeed = sSpdFE; 
    
    // ... (Calculs distance, maxSpd, énergie) ...

    // --- MISE À JOUR DU DOM ---
    // (Affichage des coordonnées, vitesses, accélérations EKF et IMU, énergies)
    
    // Satellites & DOP (Simulés)
    const numSatellites = Math.floor(Math.max(4, 12 - (acc / 10)));
    const pdop = Math.max(1.5, Math.sqrt(acc) * 0.5); 
    if ($('num-satellites')) $('num-satellites').textContent = `${numSatellites} (Sim.)`;
    if ($('pdop-value')) $('pdop-value').textContent = `${pdop.toFixed(2)} (Sim.)`;

    // --- MISE À JOUR VISUALISATIONS ET ASTRO ---
    updateGlobe(lat, lon, kAlt_new); // Appel de la fonction du Bloc B
    updateAstro(lat, lon);
    updateCompasses(lat, lon); 
    
    // --- SAUVEGARDE DES VALEURS ---
    lPos = pos;
    lPos.kAlt_old = kAlt_new;
    lPos.kSpd_old = sSpdFE; 
}
// =================================================================
// FICHIER JS PARTIE 2, BLOC B : gnss-dashboard-part2_blocB.js (Globe 3D, Contrôles & Initialisation)
// =================================================================

// -------------------------------------------
// 1. DÉCLARATIONS GLOBALES (Three.js)
// -------------------------------------------
// Références aux variables du Bloc A
let scene, camera, renderer, controls;
let earthMesh;
let trajectoryPoints; // Doit exister du Bloc A

// -------------------------------------------
// 2. FONCTIONS GLOBE 3D ET VISUALISATION (Three.js)
// -------------------------------------------

function initGlobe(latA, lonA, altA) {
    if (scene) return;
    
    const container = $('globe-container');
    if (!container) return;
    
    scene = new THREE.Scene();
    const width = container.clientWidth;
    const height = container.clientHeight;
    camera = new THREE.PerspectiveCamera(75, width / height, 0.1, 1000);
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(width, height);
    container.innerHTML = ''; 
    container.appendChild(renderer.domElement);
    
    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true; 
    
    // Habillage du Globe (Simulé)
    const textureLoader = new THREE.TextureLoader();
    const earthTexture = textureLoader.load('https://threejs.org/examples/textures/land_ocean_ice_cloud_2048.jpg'); 
    const geometry = new THREE.SphereGeometry(R_GLOBE, 60, 60); 
    const material = new THREE.MeshPhongMaterial({ map: earthTexture, specular: 0x333333, shininess: 15 });
    earthMesh = new THREE.Mesh(geometry, material);
    scene.add(earthMesh);
    
    // Lumières, Caméra.z
    
    // Marqueur GNSS (marker)
    const markerGeometry = new THREE.SphereGeometry(0.02, 16, 16);
    const markerMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000 });
    marker = new THREE.Mesh(markerGeometry, markerMaterial);
    scene.add(marker);

    // Vecteurs (speedVector, accelVector)
    const origin = new THREE.Vector3(0, 0, 0);
    const direction = new THREE.Vector3(1, 0, 0); 
    speedVector = new THREE.ArrowHelper(direction, origin, 0.5, 0x00ff00, 0.05, 0.025);
    scene.add(speedVector);
    accelVector = new THREE.ArrowHelper(direction, origin, 0.5, 0xffa500, 0.05, 0.025);
    scene.add(accelVector);
    
    // Tracé de Trajectoire (trajectoryLine)
    trajectoryPoints = []; // Initialisation ici si elle ne l'est pas dans le Bloc A
    const materialLine = new THREE.LineBasicMaterial({ color: 0xffff00 });
    const geometryLine = new THREE.BufferGeometry();
    trajectoryLine = new THREE.Line(geometryLine, materialLine);
    scene.add(trajectoryLine);
    
    animateGlobe();
}

function animateGlobe() {
    requestAnimationFrame(animateGlobe);
    controls.update(); 
    renderer.render(scene, camera);
}

function updateGlobe(latA, lonA, altA) {
    if (!scene) { initGlobe(latA, lonA, altA); return; }
    if (!marker || !speedVector || !accelVector || !trajectoryLine) return;

    // --- 1. Position du Marqueur ---
    const altitudeNormalized = kAlt / 100000; 
    const radius = R_GLOBE + altitudeNormalized;
    const phi = (90 - latA) * D2R; 
    const theta = (lonA + 180) * D2R; 
    
    marker.position.x = -(radius * Math.sin(phi) * Math.cos(theta));
    marker.position.z = (radius * Math.sin(phi) * Math.sin(theta));
    marker.position.y = (radius * Math.cos(phi));
    const currentPos = marker.position;
    
    // --- 2. Mise à jour du Tracé ---
    trajectoryPoints.push(currentPos.x, currentPos.y, currentPos.z);
    while (trajectoryPoints.length > MAX_TRAJECTORY_POINTS * 3) {
        trajectoryPoints.shift(); trajectoryPoints.shift(); trajectoryPoints.shift();
    }
    trajectoryLine.geometry.setAttribute('position', new THREE.Float32BufferAttribute(trajectoryPoints, 3));
    trajectoryLine.geometry.attributes.position.needsUpdate = true;

    // --- 3. Mise à jour des Vecteurs ---
    const currentSpeed = kSpd; 
    const currentAccel = Math.abs(accel_long); 
    
    const speedScale = currentSpeed * 0.05; 
    const speedLength = Math.min(speedScale, 2.0); 
    const headingRad = currentHeading * D2R; 
    const speedDir = new THREE.Vector3(-Math.sin(headingRad), 0, -Math.cos(headingRad));
    speedDir.normalize();

    speedVector.position.copy(currentPos);
    speedVector.setDirection(speedDir);
    speedVector.setLength(speedLength);
    
    const accelScale = currentAccel / G_ACC * 0.5;
    const accelLength = Math.min(accelScale, 1.0);
    let accelDir = speedDir.clone();
    if (accel_long < 0) { accelDir.negate(); }
    
    accelVector.position.copy(currentPos);
    accelVector.setDirection(accelDir);
    accelVector.setLength(accelLength);
}

// -------------------------------------------
// 3. FONCTIONS DE CONTRÔLE ET GESTION GPS
// -------------------------------------------

function syncH() {
    const now = getCDate();
    if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString('fr-FR');
    if ($('local-date')) $('local-date').textContent = now.toLocaleDateString('fr-FR');
}

function startGPS() {
    const GPS_OPTS = { 'low_accuracy': { enableHighAccuracy: false, maximumAge: 60000, timeout: 5000 },
                       'medium_accuracy': { enableHighAccuracy: true, maximumAge: 10000, timeout: 5000 },
                       'high_accuracy': { enableHighAccuracy: true, maximumAge: 1000, timeout: 3000 } };
    if (wID === null) {
        wID = navigator.geolocation.watchPosition(updateDisp, handleErr, GPS_OPTS[currentGPSMode]);
        isGPSTracking = true;
        if ($('toggle-gps-btn')) { $('toggle-gps-btn').textContent = "⏸️ PAUSE GPS"; $('toggle-gps-btn').style.backgroundColor = '#ffc107'; }
    }
}

function stopGPS() {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
        isGPSTracking = false;
        if ($('toggle-gps-btn')) { $('toggle-gps-btn').textContent = "▶️ MARCHE GPS"; $('toggle-gps-btn').style.backgroundColor = '#28a745'; }
    }
}

function toggleGPS() {
    isGPSTracking ? stopGPS() : startGPS();
}

function handleErr(err) { /* ... Gestion des erreurs GPS ... */ }

function emergencyStop() {
    emergencyStopActive = true;
    stopGPS();
    window.removeEventListener('devicemotion', handleDeviceMotion, true);
    window.removeEventListener('deviceorientation', handleDeviceOrientation, true);
    // ... (Suppression des autres listeners IMU/Mag) ...
    if ($('emergency-stop-btn')) { /* ... Mise à jour du bouton ... */ }
}

function resumeSystem() {
    emergencyStopActive = false;
    startGPS(); 
    window.addEventListener('devicemotion', handleDeviceMotion, true);
    window.addEventListener('deviceorientation', handleDeviceOrientation, true);
    // ... (Rétablissement des autres listeners IMU/Mag) ...
    if ($('emergency-stop-btn')) { /* ... Mise à jour du bouton ... */ }
}

function resetDist() { distM = 0.0; /* ... Mise à jour DOM ... */ }
function resetMax() { maxSpd = 0.0; /* ... Mise à jour DOM ... */ }

function resetTrajectory() {
    trajectoryPoints = [];
    if (trajectoryLine) {
        trajectoryLine.geometry.setAttribute('position', new THREE.Float32BufferAttribute(trajectoryPoints, 3));
        trajectoryLine.geometry.attributes.position.needsUpdate = true;
    }
}

function resetAll() {
    resetDist();
    resetMax();
    resetTrajectory();
    timeMoving = 0.0;
    sTime = new Date().getTime(); 
    // ... (Réinitialisation des variables EKF) ...
}

function toggleMode() { /* ... Bascule dark-mode ... */ }
function netherToggle() { netherMode = !netherMode; /* ... Mise à jour DOM ... */ }
function captureData() { /* ... Logique de téléchargement CSV ... */ }


// -------------------------------------------
// 4. INITIALISATION DES ÉVÉNEMENTS
// -------------------------------------------

document.addEventListener('DOMContentLoaded', () => {
    
    syncH(); 
    
    // --- Configuration des contrôles (Association des fonctions aux boutons) ---
    $('toggle-gps-btn')?.addEventListener('click', toggleGPS);
    $('freq-select')?.addEventListener('change', (e) => { 
        currentGPSMode = e.target.value; 
        if (isGPSTracking) { stopGPS(); startGPS(); }
    });
    $('emergency-stop-btn')?.addEventListener('click', emergencyStop);
    $('reset-dist-btn')?.addEventListener('click', resetDist);
    $('reset-max-btn')?.addEventListener('click', resetMax);
    $('reset-all-btn')?.addEventListener('click', resetAll);
    $('reset-trajectory-btn')?.addEventListener('click', resetTrajectory); 
    $('toggle-mode-btn')?.addEventListener('click', toggleMode);
    $('nether-toggle-btn')?.addEventListener('click', netherToggle);
    $('data-capture-btn')?.addEventListener('click', captureData);
    $('environment-select')?.addEventListener('change', (e) => { selectedEnvironment = e.target.value; });
    $('globe-toggle-btn')?.addEventListener('click', () => { /* ... Logique de visibilité du globe ... */ });

    // --- Démarrage des Capteurs (Ajout des Event Listeners) ---
    window.addEventListener('devicemotion', handleDeviceMotion, true);
    window.addEventListener('deviceorientation', handleDeviceOrientation, true); 
    if (window.DeviceOrientationAbsoluteEvent) {
        window.addEventListener('deviceorientationabsolute', handleMagnetometer, true);
    } else if (window.DeviceOrientationEvent) {
        window.addEventListener('deviceorientation', handleMagnetometer, true); 
    } 

    // --- Démarrage du Système de Traçage et Intervalles ---
    startGPS(); 
    const DOM_SLOW_UPDATE_MS = 1000;
    domID = setInterval(syncH, DOM_SLOW_UPDATE_MS);
    const WEATHER_UPDATE_MS = 30000;
    weatherID = setInterval(() => { if (lat !== 0 || lon !== 0) updateWeather(lat, lon); }, WEATHER_UPDATE_MS); 
    
    // Redimensionnement et Initialisation du Globe
    window.addEventListener('resize', () => {
        const container = $('globe-container');
        if (camera && renderer && container) {
            camera.aspect = container.clientWidth / container.clientHeight;
            camera.updateProjectionMatrix();
            renderer.setSize(container.clientWidth, container.clientHeight);
        }
    });

    // Initialisation du Globe
    initGlobe(lat, lon, kAlt); 

});
