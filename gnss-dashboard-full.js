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
// FICHIER JS PARTIE 2, BLOC A : gnss-dashboard-part2_blocA.js (APIs, Logique Principale & EKF)
// =================================================================

// ===========================================
// FONCTIONS ASTRO (SunCalc & Temps Solaire)
// ===========================================

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
    if ($('lsm')) $('lsm').textContent = formatTime(solarTime.LSM);
    if ($('eot')) $('eot').textContent = `${solarTime.EOT.toFixed(2)} min`;
    
    const TST_seconds = (solarTime.TST % 1440) * 60;
    const TST_Minecraft = formatTime(TST_seconds / 60); 
    if ($('time-minecraft')) $('time-minecraft').textContent = TST_Minecraft;

    if ($('sunrise-time')) $('sunrise-time').textContent = times.sunrise ? times.sunrise.toLocaleTimeString('fr-FR', {timeZone: 'UTC'}) + ' UTC' : 'N/D';
    if ($('sunset-time')) $('sunset-time').textContent = times.sunset ? times.sunset.toLocaleTimeString('fr-FR', {timeZone: 'UTC'}) + ' UTC' : 'N/D';

    const solarNoonUTC = suncalc.getTimes(date, latA, lonA).solarNoon;
    if ($('noon-solar')) $('noon-solar').textContent = solarNoonUTC ? solarNoonUTC.toLocaleTimeString('fr-FR', {timeZone: 'UTC'}) + ' UTC' : 'N/D';
    
    if (times.sunset && times.sunrise) {
        const durationMs = times.sunset.getTime() - times.sunrise.getTime();
        const durationH = durationMs / 3600000;
        if ($('day-duration')) $('day-duration').textContent = `${Math.floor(durationH)}h ${Math.round((durationH % 1) * 60)}m`;
    }
    
    if ($('sun-elevation')) $('sun-elevation').textContent = `${(sunPos.altitude * R2D).toFixed(2)} °`;
    if ($('zenith-angle')) $('zenith-angle').textContent = `${(90 - sunPos.altitude * R2D).toFixed(2)} °`;
    if ($('ecliptic-long')) $('ecliptic-long').textContent = `${(solarTime.L * R2D).toFixed(2)} °`;
    if ($('sun-declination')) $('sun-declination').textContent = `${(sunPos.declination * R2D).toFixed(2)} °`;
    
    if ($('moon-phase-display')) $('moon-phase-display').textContent = getMoonPhaseName(moon.fraction);

    // Mettre à jour l'horloge visuelle Minecraft
}

// ===========================================
// FONCTIONS MÉTÉO (OpenWeatherMap)
// ===========================================

async function updateWeather(latA, lonA) {
    if (latA === 0.0 && lonA === 0.0) return;
    if (OWM_API_KEY === "VOTRE_CLE_API_OPENWEATHERMAP") {
        return;
    }
    // Logique de récupération météo OWM non détaillée ici pour la concision
}

// ===========================================
// FONCTIONS CAPTEURS INERTIELS (IMU, Mag)
// ===========================================

function handleDeviceMotion(event) {
    if (emergencyStopActive) return;
    const acc = event.accelerationIncludingGravity;
    if (acc.x === null) return; 

    // 1. Filtrage Passe-Bas
    kAccel.x = ACCEL_FILTER_ALPHA * kAccel.x + (1 - ACCEL_FILTER_ALPHA) * acc.x;
    kAccel.y = ACCEL_FILTER_ALPHA * kAccel.y + (1 - ACCEL_FILTER_ALPHA) * acc.y;
    kAccel.z = ACCEL_FILTER_ALPHA * kAccel.z + (1 - ACCEL_FILTER_ALPHA) * acc.z; 

    // 2. Correction de l'Inclinaison (Gravité projetée)
    const G_CORR_X = -G_ACC * Math.sin(global_pitch); 
    const G_CORR_Y = G_ACC * Math.sin(global_roll) * Math.cos(global_pitch); 
    const G_CORR_Z = G_ACC * Math.cos(global_roll) * Math.cos(global_pitch); 

    // 3. Extraction de l'Accélération Linéaire (corrigée)
    let linear_x = kAccel.x - G_CORR_X;
    let linear_y = kAccel.y - G_CORR_Y;
    let linear_z = kAccel.z - G_CORR_Z; 
    
    const magnitude_lin = Math.sqrt(linear_x ** 2 + linear_y ** 2 + linear_z ** 2);
    let accel_vertical_lin_raw = linear_z; 
    
    // 4. Application du seuil statique (ZVU IMU)
    if (magnitude_lin < STATIC_ACCEL_THRESHOLD) { 
        latestLinearAccelMagnitude = 0.0;
        latestVerticalAccelIMU = 0.0;
    } else {
        latestLinearAccelMagnitude = magnitude_lin;
        latestVerticalAccelIMU = accel_vertical_lin_raw;
    }

    // Affichage IMU
    if ($('accel-vertical-imu')) $('accel-vertical-imu').textContent = `${latestVerticalAccelIMU.toFixed(3)} m/s²`;
    if ($('force-g-vertical')) $('force-g-vertical').textContent = `${(latestVerticalAccelIMU / G_ACC).toFixed(2)} G`;
    if ($('accel-3d-imu')) $('accel-3d-imu').textContent = `${latestLinearAccelMagnitude.toFixed(3)} m/s²`;
    if ($('force-g-3d-imu')) $('force-g-3d-imu').textContent = `${(latestLinearAccelMagnitude / G_ACC).toFixed(2)} G`;
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

    // Placeholder de lecture de magnitude
    latestMagneticFieldMagnitude = 50 + (Math.sin(Date.now() / 5000) * 5) + Math.random() * 20 * (lat === 0.0 ? 1 : 0.1); 
    
    if ($('magnetic-field')) {
        $('magnetic-field').textContent = `${latestMagneticFieldMagnitude.toFixed(2)} µT`;
    }
    
    updateCompasses(lat, lon); 
}

/** Calcule le cap (azimut) entre deux points (Haversine Inverse). */
function calculateBearing(lat1, lon1, lat2, lon2) {
    const phi1 = lat1 * D2R;
    const phi2 = lat2 * D2R;
    const lambda1 = lon1 * D2R;
    const lambda2 = lon2 * D2R;

    const y = Math.sin(lambda2 - lambda1) * Math.cos(phi2);
    const x = Math.cos(phi1) * Math.sin(phi2) - 
              Math.sin(phi1) * Math.cos(phi2) * Math.cos(lambda2 - lambda1);
    
    const bearingRad = Math.atan2(y, x);
    return (bearingRad * R2D + 360) % 360; 
}

/** Met à jour les deux boussoles. */
function updateCompasses(latA, lonA) {
    // 1. Boussole Standard
    if ($('compass-heading')) {
        $('compass-heading').textContent = `${currentHeading.toFixed(1)} °`;
        const needle = $('compass-needle');
        if (needle) {
            needle.style.transform = `rotate(${-currentHeading}deg)`;
        }
    }
    
    // 2. Boussole de Cible
    if (latA !== 0 || lonA !== 0) {
        const targetBearing = calculateBearing(latA, lonA, TARGET_LAT, TARGET_LON);
        const distanceM = calculateDistanceHaversine(latA, lonA, TARGET_LAT, TARGET_LON);
        const distanceKm = distanceM / 1000;
        
        if ($('target-bearing')) $('target-bearing').textContent = `${targetBearing.toFixed(1)} °`;
        if ($('target-distance')) $('target-distance').textContent = `${distanceKm.toFixed(3)} km`;
        
        const relativeBearing = (targetBearing - currentHeading + 360) % 360;

        const targetNeedle = $('target-needle');
        if (targetNeedle) {
            targetNeedle.style.transform = `rotate(${relativeBearing}deg)`;
        }
    } else {
        if ($('target-bearing')) $('target-bearing').textContent = `N/A (GPS)`;
        if ($('target-distance')) $('target-distance').textContent = `N/A`;
    }
}


// ===========================================
// FONCTION PRINCIPALE DE MISE À JOUR (GPS CALLBACK)
// ===========================================

function updateDisp(pos) {
    if (emergencyStopActive) return;
    
    const cTimePos = pos.timestamp; 
    const now = getCDate(); 
    
    // Lecture des données brutes
    lat = pos.coords.latitude; 
    lon = pos.coords.longitude;
    const alt_gps_raw = pos.coords.altitude; 
    const acc = pos.coords.accuracy;
    const spd_raw_gps = pos.coords.speed ?? 0;
    const MASS = 70.0; 

    // Calcul du dt et conditions initiales
    const dt = lPos ? (cTimePos - lPos.timestamp) / 1000 : MIN_DT;
    if (lPos === null) { sTime = now.getTime(); }

    // Correction de l'imprécision forcée
    let effectiveAcc = acc;
    const accOverride = parseFloat($('gps-accuracy-override').value);
    if (accOverride > 0) { effectiveAcc = accOverride; }

    // Gestion de l'imprécision GPS
    if (acc > MAX_ACC) { 
        if ($('gps-precision')) $('gps-precision').textContent = `❌ ${acc.toFixed(0)} m (Trop Imprécis)`; 
        if (lPos === null) { lPos = pos; } 
        updateAstro(lat, lon);
        updateGlobe(lat, lon, kAlt);
        updateCompasses(lat, lon); 
        lPos.kAlt_old = kAlt;
        lPos.kSpd_old = kSpd;
        return; 
    }

    // Calcul de la vitesse 3D brute
    const spdH = spd_raw_gps; 
    let spdV_raw = alt_gps_raw !== null && lPos && lPos.altitude !== null ? (alt_gps_raw - lPos.altitude) / dt : 0;
    const spd3D_raw = Math.sqrt(spdH ** 2 + spdV_raw ** 2);
    
    // --- Initialisation EKF ---
    if (lPos === null && alt_gps_raw !== null && acc < MAX_ACC) {
        kSpd = spd3D_raw; 
        kAlt = alt_gps_raw; 
        kUncert = effectiveAcc * effectiveAcc; 
        kAltUncert = effectiveAcc * effectiveAcc; 
    }
    
    // --- FILTRAGE EKF ET DYNAMIQUE ---
    let imuDampeningFactor = 1.0; 
    if (effectiveAcc <= GOOD_ACC_THRESHOLD) { imuDampeningFactor = 0.5; } 
    else if (effectiveAcc >= MAX_ACC) { imuDampeningFactor = 1.0; } 
    else {
        const range = MAX_ACC - GOOD_ACC_THRESHOLD;
        const value = effectiveAcc - GOOD_ACC_THRESHOLD;
        imuDampeningFactor = 0.5 + 0.5 * (value / range); 
    }
    
    let accel_control_3D = latestLinearAccelMagnitude * imuDampeningFactor;
    let accel_control_V = latestVerticalAccelIMU * imuDampeningFactor;

    // LOGIQUE ZVU
    const velNoiseThreshold = effectiveAcc * VEL_NOISE_FACTOR; 
    const isStatic = (spd3D_raw < velNoiseThreshold);
    let isIMUOnlyMode = isStatic;

    if (isStatic) {
        isIMUOnlyMode = true; 
        isZVUActive = true;
        zvuLockTime += dt;
        accel_control_3D = 0.0; 
        accel_control_V = 0.0; 
        kSpd = 0.0; kUncert = 0.000001; lastFSpeed = 0.0; 
    } else {
        isZVUActive = false;
        zvuLockTime = 0;
    }
    
    let R_dyn = getKalmanR(effectiveAcc, alt_gps_raw, lastP_hPa);
    if (isIMUOnlyMode) { R_dyn = R_GPS_DISABLED; } 

    const sSpdFE = kFilter(spd3D_raw, dt, R_dyn, accel_control_3D); 
    const kAlt_new = kFilterAltitude(alt_gps_raw, effectiveAcc, dt, accel_control_V);

    // Calculs finaux
    let accel_long = (dt > 0.05) ? (sSpdFE - lastFSpeed) / dt : 0;
    lastFSpeed = sSpdFE; 
    
    const deltaD = sSpdFE * dt;
    distM += deltaD;
    
    let avgMovingSpeed = 0;
    if (sSpdFE > 0.1) { 
        timeMoving += dt;
        maxSpd = Math.max(maxSpd, sSpdFE); 
        avgMovingSpeed = distM / timeMoving; 
    } else {
        avgMovingSpeed = timeMoving > 0 ? (distM / timeMoving) : 0;
    }

    const kineticEnergy = 0.5 * MASS * sSpdFE * sSpdFE;
    const mechanicalPower = MASS * accel_long * sSpdFE;
    const dragForce = 0.5 * lastAirDensity * 0.5 * 2.0 * sSpdFE * sSpdFE; 
    const coriolisForce = 2 * MASS * sSpdFE * W_EARTH * Math.cos(lat * D2R);

    // --- MISE À JOUR DU DOM ---
    
    // Vitesse & Distance
    $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(5)} km/h`;
    $('speed-stable-ms').textContent = `${sSpdFE.toFixed(3)} m/s | ${(sSpdFE * 1000).toFixed(0)} mm/s`;
    $('perc-speed-sound').textContent = `${((sSpdFE / SPEED_SOUND) * 100).toFixed(2)} %`;
    $('perc-speed-c').textContent = `${((sSpdFE / C_L) * 100).toExponential(2)} %`;
    $('distance-total-km').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    $('speed-error-perc').textContent = `${R_dyn.toFixed(3)} m² (R dyn)`; 
    $('gps-accuracy-effective').textContent = `${kUncert.toFixed(3)} m²/s²`; 
    $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    $('speed-avg-moving').textContent = `${(avgMovingSpeed * KMH_MS).toFixed(5)} km/h`;
    $('speed-3d-inst').textContent = `${(spd3D_raw * KMH_MS).toFixed(5)} km/h`; 

    // GPS & Physique
    $('latitude').textContent = lat.toFixed(6);
    $('longitude').textContent = lon.toFixed(6);
    $('altitude-gps').textContent = alt_gps_raw !== null ? `${alt_gps_raw.toFixed(2)} m` : 'N/A';
    $('altitude-geoid').textContent = alt_gps_raw !== null ? `${(alt_gps_raw - GEOID_SEPARATION_M).toFixed(2)} m` : 'N/A';
    $('gps-precision').textContent = `${acc.toFixed(1)} m`;
    $('speed-raw-ms').textContent = `${spd3D_raw.toFixed(3)} m/s`;
    
    // Dynamique
    $('accel-long').textContent = `${accel_long.toFixed(3)} m/s²`;
    $('force-g-long').textContent = `${(accel_long / G_ACC).toFixed(2)} G`;
    $('vertical-speed').textContent = `${spdV_raw.toFixed(3)} m/s`;

    // Champs & Énergie
    $('kinetic-energy').textContent = `${kineticEnergy.toFixed(2)} J`;
    $('mechanical-power').textContent = `${mechanicalPower.toFixed(2)} W`;
    $('drag-force').textContent = `${dragForce.toFixed(3)} N`;
    $('coriolis-force').textContent = `${coriolisForce.toFixed(3)} N`;
    
    // Contrôles & Système
    $('time-elapsed').textContent = `${((now.getTime() - sTime) / 1000).toFixed(2)} s`;
    $('time-moving').textContent = `${timeMoving.toFixed(2)} s`;
    $('zvu-lock-status').textContent = isZVUActive ? 'VERROUILLÉ 🔒' : 'NON-VERROUILLÉ';
    $('zvu-lock-time').textContent = `${zvuLockTime.toFixed(1)} s`;

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
// FICHIER JS PARTIE 2, BLOC B : gnss-dashboard-part2_blocB.js (Contrôles, Globe 3D, Initialisation)
// =================================================================

// ===========================================
// FONCTIONS GLOBE 3D ET VISUALISATION (Three.js)
// ===========================================

/** Initialise la scène Three.js, le globe et les contrôles. */
function initGlobe(latA, lonA, altA) {
    if (scene) return;
    
    const container = $('globe-container');
    if (!container) return;
    
    // 1. SCÈNE, CAMÉRA ET RENDERER
    scene = new THREE.Scene();
    const width = container.clientWidth;
    const height = container.clientHeight;
    camera = new THREE.PerspectiveCamera(75, width / height, 0.1, 1000);
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(width, height);
    container.innerHTML = ''; 
    container.appendChild(renderer.domElement);
    
    // 2. CONTRÔLES
    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true; 
    
    // 3. CRÉATION DU GLOBE
    const geometry = new THREE.SphereGeometry(2, 60, 60); 
    const material = new THREE.MeshPhongMaterial({
        color: 0x3366ff,
        wireframe: true 
    });
    
    const earth = new THREE.Mesh(geometry, material);
    earth.name = "Earth";
    scene.add(earth);
    
    // 4. LUMIÈRE ET POSITION INITIALE
    const ambientLight = new THREE.AmbientLight(0x404040); 
    scene.add(ambientLight);
    const directionalLight = new THREE.DirectionalLight(0xffffff, 1);
    directionalLight.position.set(5, 3, 5);
    scene.add(directionalLight);

    camera.position.z = 4;
    
    // 5. MARQUEUR GNSS
    const markerGeometry = new THREE.SphereGeometry(0.02, 16, 16);
    const markerMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000 });
    marker = new THREE.Mesh(markerGeometry, markerMaterial);
    scene.add(marker);
    
    animateGlobe();
    
    console.log("Globe 3D Three.js initialisé.");
    $('globe-status-display').textContent = `Globe 3D Three.js: Actif`;
}

/** Boucle de rendu Three.js. */
function animateGlobe() {
    requestAnimationFrame(animateGlobe);
    controls.update(); 
    renderer.render(scene, camera);
}

/** Met à jour la position du marqueur GNSS. */
function updateGlobe(latA, lonA, altA) {
    if (!scene) { initGlobe(latA, lonA, altA); return; }
    if (!marker) return;

    // Conversion Lat/Lon en coordonnées cartésiennes (X, Y, Z)
    const R_GLOBE = 2.0;
    const altitudeNormalized = kAlt / 100000; 
    const radius = R_GLOBE + altitudeNormalized;

    const phi = (90 - latA) * D2R; 
    const theta = (lonA + 180) * D2R; 
    
    marker.position.x = -(radius * Math.sin(phi) * Math.cos(theta));
    marker.position.z = (radius * Math.sin(phi) * Math.sin(theta));
    marker.position.y = (radius * Math.cos(phi));
    
    if ($('globe-status-display')) {
        $('globe-status-display').textContent = `🛰️ Position: Lat ${latA.toFixed(6)}, Lon ${lonA.toFixed(6)}, Alt ${kAlt.toFixed(2)} m`;
    }
}


// ===========================================
// FONCTIONS DE CONTRÔLE ET GESTION GPS
// ===========================================

let isGPSRunning = false;
let isGPSTracking = false;

function startGPS() {
    if (wID === null) {
        wID = navigator.geolocation.watchPosition(updateDisp, handleErr, GPS_OPTS[currentGPSMode]);
        isGPSRunning = true;
        isGPSTracking = true;
        $('toggle-gps-btn').textContent = "⏸️ PAUSE GPS";
        $('toggle-gps-btn').style.backgroundColor = '#ffc107'; 
        console.log("GPS Démarré en mode:", currentGPSMode);
    }
}

function stopGPS() {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
        isGPSTracking = false;
        $('toggle-gps-btn').textContent = "▶️ MARCHE GPS";
        $('toggle-gps-btn').style.backgroundColor = '#28a745'; 
        console.log("GPS Arrêté.");
    }
}

function toggleGPS() {
    if (isGPSTracking) {
        stopGPS();
    } else {
        startGPS();
    }
}

function handleErr(err) {
    if (err.code === 1) {
        console.error("Erreur GPS: Accès refusé par l'utilisateur.");
        $('gps-precision').textContent = "Accès refusé";
    } else if (err.code === 2) {
        console.error("Erreur GPS: Position non disponible.");
        $('gps-precision').textContent = "Position non disponible";
    } else if (err.code === 3) {
        console.warn("Erreur GPS: Timeout.");
    } else {
        console.error(`Erreur GPS inconnue: ${err.message}`);
    }
}

function emergencyStop() {
    emergencyStopActive = true;
    stopGPS();
    window.removeEventListener('devicemotion', handleDeviceMotion, true);
    window.removeEventListener('deviceorientation', handleDeviceOrientation, true);
    window.removeEventListener('deviceorientationabsolute', handleMagnetometer, true);
    window.removeEventListener('deviceorientation', handleMagnetometer, true);
    
    $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: ACTIF 🔴";
    $('emergency-stop-btn').style.backgroundColor = '#dc3545';
    $('emergency-stop-btn').removeEventListener('click', emergencyStop);
    $('emergency-stop-btn').addEventListener('click', resumeSystem);
    console.warn("SYSTÈME EN ARRÊT D'URGENCE TOTAL.");
}

function resumeSystem() {
    emergencyStopActive = false;
    startGPS(); 
    
    window.addEventListener('devicemotion', handleDeviceMotion, true);
    window.addEventListener('deviceorientation', handleDeviceOrientation, true);
    if (window.DeviceOrientationAbsoluteEvent) {
        window.addEventListener('deviceorientationabsolute', handleMagnetometer, true);
    } else if (window.DeviceOrientationEvent) {
        window.addEventListener('deviceorientation', handleMagnetometer, true); 
    }
    
    $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: INACTIF 🟢";
    $('emergency-stop-btn').style.backgroundColor = ''; 
    $('emergency-stop-btn').removeEventListener('click', resumeSystem);
    $('emergency-stop-btn').addEventListener('click', emergencyStop);
    console.log("SYSTÈME RÉTABLI.");
}

function resetDist() {
    distM = 0.0;
    if ($('distance-total-km')) $('distance-total-km').textContent = "0.000 km | 0.00 m";
    console.log("Distance réinitialisée.");
}

function resetMax() {
    maxSpd = 0.0;
    if ($('speed-max')) $('speed-max').textContent = "0.00000 km/h";
    console.log("Vitesse max réinitialisée.");
}

function resetAll() {
    resetDist();
    resetMax();
    timeMoving = 0.0;
    sTime = new Date().getTime(); 
    zvuLockTime = 0;
    kSpd = 0.0; 
    kUncert = 10.0; 
    kAlt = lPos ? lPos.coords.altitude || 0 : 0;
    kAltUncert = 10.0;
    lastFSpeed = 0.0;
    console.log("Toutes les variables de session ont été réinitialisées.");
}

function toggleMode() {
    const isDarkMode = document.body.classList.toggle('dark-mode');
    $('toggle-mode-btn').textContent = isDarkMode ? '☀️ Mode Jour' : '🌗 Mode Nuit';
    console.log(`Mode d'affichage basculé vers ${isDarkMode ? 'Nuit' : 'Jour'}.`);
}

function netherToggle() {
    netherMode = !netherMode;
    $('mode-nether').textContent = netherMode ? `ACTIVÉ (1:${NETHER_RATIO})` : 'DÉSACTIVÉ (1:1)';
    $('nether-toggle-btn').textContent = netherMode ? 'Mode Overworld' : 'Mode Nether';
    console.log(`Mode Nether basculé : ${netherMode ? 'ACTIVÉ' : 'DÉSACTIVÉ'}`);
}

/** Enregistre les données actuelles de l'état du système dans un fichier CSV. */
function captureData() {
    const data = {
        Timestamp: getCDate().toISOString(),
        "Heure Locale": $('local-time').textContent,
        "Lat (EKF)": lat.toFixed(6),
        "Lon (EKF)": lon.toFixed(6),
        "Alt (EKF)": kAlt.toFixed(2) + ' m',
        "Vitesse Stable (km/h)": $('speed-stable').textContent,
        "Accel Long (m/s²)": $('accel-long').textContent.split(' ')[0],
        "Force G (Long)": $('force-g-long').textContent.split(' ')[0],
        "Distance Totale (m)": distM.toFixed(2),
        "Précision GPS (m)": $('gps-precision').textContent.split(' ')[0],
        "Vitesse Brute (m/s)": $('speed-raw-ms').textContent.split(' ')[0],
        "Cap Appareil (°)": currentHeading.toFixed(1),
        "Champ Magnétique (µT)": $('magnetic-field').textContent.split(' ')[0]
    };

    const headers = Object.keys(data).join(',');
    const values = Object.values(data).map(v => `"${v.replace(/"/g, '""')}"`).join(',');
    
    const csvContent = headers + "\n" + values;
    
    const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement("a");
    link.setAttribute("href", url);
    link.setAttribute("download", `GNSS_Capture_${new Date().toISOString().replace(/:/g, '-')}.csv`);
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
    
    console.log("Données capturées et téléchargement lancé.");
}


// ===========================================
// INITIALISATION DES ÉVÉNEMENTS ET INTERVALLES
// ===========================================

document.addEventListener('DOMContentLoaded', () => {
    
    syncH(); 
    
    // --- Configuration des contrôles ---
    $('toggle-gps-btn').addEventListener('click', toggleGPS);
    $('freq-select').addEventListener('change', (e) => { 
        currentGPSMode = e.target.value; 
        if (isGPSTracking) { stopGPS(); startGPS(); }
    });
    $('emergency-stop-btn').addEventListener('click', emergencyStop);
    $('reset-dist-btn').addEventListener('click', resetDist);
    $('reset-max-btn').addEventListener('click', resetMax);
    $('reset-all-btn').addEventListener('click', resetAll);
    $('toggle-mode-btn').addEventListener('click', toggleMode);
    $('nether-toggle-btn').addEventListener('click', netherToggle);
    $('data-capture-btn').addEventListener('click', captureData);
    $('environment-select').addEventListener('change', (e) => { 
        selectedEnvironment = e.target.value; 
        console.log("Environnement EKF:", selectedEnvironment);
    });

    // --- Démarrage des Capteurs IMU ---
    window.addEventListener('devicemotion', handleDeviceMotion, true);
    window.addEventListener('deviceorientation', handleDeviceOrientation, true); 
    
    // --- Démarrage du Capteur de Boussole/Magnétomètre ---
    if (window.DeviceOrientationAbsoluteEvent) {
        window.addEventListener('deviceorientationabsolute', handleMagnetometer, true);
    } else if (window.DeviceOrientationEvent) {
        window.addEventListener('deviceorientation', handleMagnetometer, true); 
    } 

    // --- Démarrage du Système de Traçage ---
    startGPS(); 

    // --- Intervalle pour l'horloge DOM lente (Heure/Date) ---
    domID = setInterval(syncH, DOM_SLOW_UPDATE_MS);

    // --- Intervalle pour la mise à jour Météo (30s) ---
    if (weatherID === null) {
        weatherID = setInterval(() => {
            if (lat !== 0 || lon !== 0) updateWeather(lat, lon);
        }, WEATHER_UPDATE_MS); 
    }
    
    // S'assurer que le globe est redimensionné
    window.addEventListener('resize', () => {
        if (camera && renderer) {
            const container = $('globe-container');
            camera.aspect = container.clientWidth / container.clientHeight;
            camera.updateProjectionMatrix();
            renderer.setSize(container.clientWidth, container.clientHeight);
        }
    });

}); // Fin de document.addEventListener('DOMContentLoaded')
