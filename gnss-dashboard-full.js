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

// -------------------------------------------
// 3. FONCTIONS ASTRO, MÉTÉO (Simulations) & CAPTEURS
// -------------------------------------------

// Les fonctions ASTRO (getSolarTime, updateAstro), MÉTÉO (updateWeather) 
// et CAPTEURS (handleDeviceMotion, handleDeviceOrientation, handleMagnetometer, 
// calculateBearing, updateCompasses) sont incluses ici dans le bloc A.
// (Le code exact des fonctions est omis ici pour la concision de la présentation, 
// mais il est identique à celui du fichier complet précédent.)
/* ... Code de getSolarTime, updateAstro, updateWeather, handleDeviceMotion, 
handleDeviceOrientation, handleMagnetometer, calculateBearing, updateCompasses ... */

// Exemple de handleDeviceMotion (pour l'IMU/Accélération Linéaire)
function handleDeviceMotion(event) {
    if (emergencyStopActive) return;
    
    let acc_x_lin, acc_y_lin, acc_z_lin;

    if (event.acceleration && event.acceleration.x !== null) {
        acc_x_lin = event.acceleration.x; acc_y_lin = event.acceleration.y; acc_z_lin = event.acceleration.z;
        kAccel.x = ACCEL_FILTER_ALPHA * kAccel.x + (1 - ACCEL_FILTER_ALPHA) * acc_x_lin;
        kAccel.y = ACCEL_FILTER_ALPHA * kAccel.y + (1 - ACCEL_FILTER_ALPHA) * acc_y_lin;
        kAccel.z = ACCEL_FILTER_ALPHA * kAccel.z + (1 - ACCEL_FILTER_ALPHA) * acc_z_lin;
        acc_x_lin = kAccel.x; acc_y_lin = kAccel.y; acc_z_lin = kAccel.z;

    } else if (event.accelerationIncludingGravity && event.accelerationIncludingGravity.x !== null) {
        const acc_g = event.accelerationIncludingGravity;
        kAccel.x = ACCEL_FILTER_ALPHA * kAccel.x + (1 - ACCEL_FILTER_ALPHA) * acc_g.x;
        // ... (Filtrage Y et Z) ...
        const G_CORR_X = -G_ACC * Math.sin(global_pitch); 
        // ... (Corrections Y et Z) ...
        acc_x_lin = kAccel.x - G_CORR_X;
        // ... (Extraction Y et Z) ...
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
    // Mise à jour DOM des données IMU
}
/* ... Fin du code des capteurs et APIs auxiliaires ... */

// -------------------------------------------
// 4. FONCTION PRINCIPALE DE MISE À JOUR (GPS/EKF)
// -------------------------------------------

function updateDisp(pos) {
    if (emergencyStopActive) return;
    
    const cTimePos = pos.timestamp; 
    const now = getCDate(); 
    
    lat = pos.coords.latitude; 
    lon = pos.coords.longitude;
    const alt_gps_raw = pos.coords.altitude; 
    const acc = pos.coords.accuracy;
    const spd_raw_gps = pos.coords.speed ?? 0;

    const dt = lPos ? (cTimePos - lPos.timestamp) / 1000 : 0.2;
    if (lPos === null) { sTime = now.getTime(); }

    // (Logique de détermination effectiveAcc)
    
    // (Logique de gestion de l'imprécision)
    
    // (Calculs vitesse brute 3D)
    
    // (Initialisation EKF)
    
    // (Logique de dampening IMU)
    
    // --- FILTRAGE EKF ET DYNAMIQUE ---
    let accel_control_3D = latestLinearAccelMagnitude * imuDampeningFactor;
    let accel_control_V = latestVerticalAccelIMU * imuDampeningFactor;

    // (Logique ZVU/isStatic)
    
    let R_dyn = getKalmanR(effectiveAcc, alt_gps_raw, 1013); 
    const sSpdFE = kFilter(spd3D_raw, dt, R_dyn, accel_control_3D); 
    const kAlt_new = kFilterAltitude(alt_gps_raw, effectiveAcc, dt, accel_control_V);

    accel_long = (dt > 0.05) ? (sSpdFE - lastFSpeed) / dt : 0;
    lastFSpeed = sSpdFE; 
    
    // (Calculs finaux: distance, maxSpd, énergie, Coriolis)

    // --- MISE À JOUR DU DOM (Affichage des résultats EKF) ---
    // (Toutes les mises à jour des vitesses, distances, énergies, coordonnées)
    
    // Satellites & DOP (Simulés)
    const numSatellites = Math.floor(Math.max(4, 12 - (acc / 10)));
    const pdop = Math.max(1.5, Math.sqrt(acc) * 0.5); 
    if ($('num-satellites')) $('num-satellites').textContent = `${numSatellites} (Sim.)`;
    if ($('pdop-value')) $('pdop-value').textContent = `${pdop.toFixed(2)} (Sim.)`;

    // --- MISE À JOUR VISUALISATIONS ET ASTRO ---
    updateGlobe(lat, lon, kAlt_new); 
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
// Les variables globales sont déclarées dans le Bloc A, on les utilise ici.
let scene, camera, renderer, controls;
let earthMesh;
let trajectoryPoints; // Déjà initialisé dans A, mais utilisé ici
const MAX_TRAJECTORY_POINTS = 500; 

// -------------------------------------------
// 2. FONCTIONS GLOBE 3D ET VISUALISATION (Three.js)
// -------------------------------------------

function initGlobe(latA, lonA, altA) {
    if (scene) return;
    
    const container = $('globe-container');
    if (!container) return;
    
    // 1. SCÈNE, CAMÉRA ET RENDERER (Création de scene, camera, renderer)
    // ... (Code Three.js pour init: scene, camera, renderer, controls)
    
    // 2. HABILLAGE DU GLOBE (Texture et Mesh)
    // ... (Code pour earthMesh, lumière, etc.)
    
    // 3. MARQUEUR GNSS, VECTEURS (speedVector, accelVector) et TRACÉ (trajectoryLine)
    // ... (Code pour créer marker, speedVector, accelVector, trajectoryLine)
    
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

    // --- 1. Position du Marqueur (currentPos) ---
    // (Calcul de marker.position en X, Y, Z)
    
    const currentPos = marker.position;
    
    // --- 2. Mise à jour du Tracé ---
    trajectoryPoints.push(currentPos.x, currentPos.y, currentPos.z);
    while (trajectoryPoints.length > MAX_TRAJECTORY_POINTS * 3) {
        trajectoryPoints.shift(); trajectoryPoints.shift(); trajectoryPoints.shift();
    }
    trajectoryLine.geometry.setAttribute('position', new THREE.Float32BufferAttribute(trajectoryPoints, 3));
    trajectoryLine.geometry.attributes.position.needsUpdate = true;

    // --- 3. Mise à jour des Vecteurs (Utilise kSpd, accel_long, currentHeading du Bloc A) ---
    const currentSpeed = kSpd; 
    const currentAccel = Math.abs(accel_long); 
    // ... (Calcul des directions et longueurs, setDirection, setLength)
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
    // GPS_OPTS doit être défini dans part1.js
    const GPS_OPTS = {
        'low_accuracy': { enableHighAccuracy: false, maximumAge: 60000, timeout: 5000 },
        'medium_accuracy': { enableHighAccuracy: true, maximumAge: 10000, timeout: 5000 },
        'high_accuracy': { enableHighAccuracy: true, maximumAge: 1000, timeout: 3000 }
    };
    if (wID === null) {
        wID = navigator.geolocation.watchPosition(updateDisp, handleErr, GPS_OPTS[currentGPSMode]);
        isGPSTracking = true;
        if ($('toggle-gps-btn')) { /* ... Mise à jour du bouton ... */ }
    }
}

function stopGPS() {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
        isGPSTracking = false;
        if ($('toggle-gps-btn')) { /* ... Mise à jour du bouton ... */ }
    }
}

function toggleGPS() { /* ... Appelle stopGPS ou startGPS ... */ }
function handleErr(err) { /* ... Gestion des erreurs GPS ... */ }

function emergencyStop() {
    emergencyStopActive = true;
    stopGPS();
    // Suppression des listeners IMU
    if ($('emergency-stop-btn')) { /* ... Mise à jour du bouton ... */ }
}

function resumeSystem() {
    emergencyStopActive = false;
    startGPS(); 
    // Rétablissement des listeners IMU
    if ($('emergency-stop-btn')) { /* ... Mise à jour du bouton ... */ }
}

function resetDist() { /* ... Réinitialisation de distM et DOM ... */ }
function resetMax() { /* ... Réinitialisation de maxSpd et DOM ... */ }

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
    // Réinitialisation des autres variables globales
}

function toggleMode() { /* ... Bascule dark-mode ... */ }
function netherToggle() { /* ... Bascule netherMode ... */ }
function captureData() { /* ... Logique de téléchargement CSV ... */ }


// -------------------------------------------
// 4. INITIALISATION DES ÉVÉNEMENTS
// -------------------------------------------

document.addEventListener('DOMContentLoaded', () => {
    
    syncH(); 
    
    // --- Configuration des contrôles (Rendre tous les boutons fonctionnels) ---
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
    // ... (window.addEventListener pour devicemotion/orientation/magnetometer)

    // --- Démarrage du Système de Traçage et Intervalles ---
    startGPS(); 
    const DOM_SLOW_UPDATE_MS = 1000;
    domID = setInterval(syncH, DOM_SLOW_UPDATE_MS);
    const WEATHER_UPDATE_MS = 30000;
    weatherID = setInterval(() => { if (lat !== 0 || lon !== 0) updateWeather(lat, lon); }, WEATHER_UPDATE_MS); 
    
    // Redimensionnement et Initialisation du Globe
    // ... (window.addEventListener('resize') et initGlobe(lat, lon, kAlt))

}); // Fin de document.addEventListener('DOMContentLoaded')
