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
const ALT_TH = -50; 
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 5000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 30000, timeout: 60000 }
};

// Constantes Kalman
const Q_NOISE = 0.001; 
let kSpd = 0; // Estimation vitesse (m/s)
let kUncert = 1000; // Incertitude vitesse (m/s)²
const ENVIRONMENT_FACTORS = {
    NORMAL: 1.0, FOREST: 1.5, CONCRETE: 3.0, METAL: 2.5
};
const Q_ALT_NOISE = 0.01; 
let kAlt = 0; // Estimation altitude (m)
let kAltUncert = 1000; // Incertitude altitude (m)²

// Constantes IMU
const ACCEL_FILTER_ALPHA = 0.8; // LPF Alpha pour les mesures brutes
const GRAVITY_CALIBRATION_ALPHA = 0.01; // LPF Alpha lent pour l'estimation de G
const ACCEL_MOVEMENT_THRESHOLD = 0.8; // Seuil pour stopper le recalibrage de G
let kAccel = { x: 0, y: 0, z: 0 }; // Accélération brute filtrée (avec G)
let G_STATIC_REF = { x: 0, y: 0, z: 0 }; // Vecteur Gravité estimé (pour soustraction)
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
// (fonctions dist, kFilter, getKalmanR, kFilterAltitude, calculateDewPoint... sont présentes)
// ===========================================

// ... (fonctions utilitaires comme dist, kFilter, kFilterAltitude, calculateDewPoint, getKalmanR) ...

// ===========================================
// FONCTIONS ASTRO, TEMPS & COULEUR DU CIEL
// ===========================================
// ... (fonctions getCDate, syncH, getSolarTime, updateMoon, updateAstro... sont présentes) ...

/** Détermine la couleur de fond du body (ciel) en fonction de l'élévation solaire. (Disque Jour/Nuit) */
function getSkyColor(elevation, date) {
    const body = document.body;
    let newColor = '#f4f4f4'; // Jour (mode clair par défaut)

    if (body.classList.contains('dark-mode')) {
        // Mode nuit (couleurs sombres)
        newColor = '#1a1a1a'; // Nuit profonde
        if (elevation > -12 * D2R) { // Crépuscule nautique/civil
            newColor = '#2c2c2c';
        }
    } else {
        // Mode jour (couleurs claires)
        if (elevation < -18 * D2R) { // Nuit profonde (Astrologique)
            newColor = '#333';
        } else if (elevation < -12 * D2R) { // Crépuscule Astronomique
            newColor = '#666'; 
        } else if (elevation < -6 * D2R) { // Crépuscule Nautique
            newColor = '#999'; 
        } else if (elevation < 0) { // Crépuscule Civil
            newColor = '#ccc'; 
        } else if (elevation < 0.1 * D2R) { // Lever/Coucher (très bas)
            newColor = '#f0d0c0'; // Ton chaud
        } else {
            newColor = '#f4f4f4'; // Plein jour
        }
    }
    
    // Application de la couleur de fond (simule le ciel/disque TST)
    body.style.backgroundColor = newColor;
}

// ... (fonction updateWeather) ...

// ===========================================
// FONCTIONS CAPTEURS INERTIELS (IMU) - ÉTALONNAGE CONTINU
// ===========================================

/** Gère les données de l'accéléromètre via DeviceMotionEvent. */
function handleDeviceMotion(event) {
    if (emergencyStopActive) return;
    const acc = event.accelerationIncludingGravity;
    if (acc.x === null) return; 

    // 1. Lissage des mesures brutes (pour réduire le bruit)
    // Nous suivons les trois axes
    kAccel.x = ACCEL_FILTER_ALPHA * kAccel.x + (1 - ACCEL_FILTER_ALPHA) * acc.x;
    kAccel.y = ACCEL_FILTER_ALPHA * kAccel.y + (1 - ACCEL_FILTER_ALPHA) * acc.y; 
    kAccel.z = ACCEL_FILTER_ALPHA * kAccel.z + (1 - ACCEL_FILTER_ALPHA) * acc.z; 

    // 2. Calcul de l'accélération linéaire (par soustraction de la référence G)
    const accel_lin_x = kAccel.x - G_STATIC_REF.x;
    const accel_lin_y = kAccel.y - G_STATIC_REF.y;
    const accel_lin_z = kAccel.z - G_STATIC_REF.z;

    // 3. Magnitude de l'accélération linéaire 3D
    const accel_3d_mag = Math.sqrt(accel_lin_x ** 2 + accel_lin_y ** 2 + accel_lin_z ** 2);
    latestLinearAccelMagnitude = accel_3d_mag;
    
    // 4. Étalonnage Continu de la Gravité (comme si l'accéléromètre était parfait)
    // Mise à jour de G_STATIC_REF (le vecteur Gravité estimé) si l'accélération linéaire 
    // est inférieure à un seuil. Le LPF lent 'GRAVITY_CALIBRATION_ALPHA' garantit une correction progressive.
    if (accel_3d_mag < ACCEL_MOVEMENT_THRESHOLD) {
        G_STATIC_REF.x = GRAVITY_CALIBRATION_ALPHA * kAccel.x + (1 - GRAVITY_CALIBRATION_ALPHA) * G_STATIC_REF.x;
        G_STATIC_REF.y = GRAVITY_CALIBRATION_ALPHA * kAccel.y + (1 - GRAVITY_CALIBRATION_ALPHA) * G_STATIC_REF.y;
        G_STATIC_REF.z = GRAVITY_CALIBRATION_ALPHA * kAccel.z + (1 - GRAVITY_CALIBRATION_ALPHA) * G_STATIC_REF.z;
    } 
    
    // 5. Mise à jour des valeurs pour le DOM
    latestVerticalAccelIMU = accel_lin_z;
    
    if ($('accel-vertical-imu')) $('accel-vertical-imu').textContent = `${latestVerticalAccelIMU.toFixed(3)} m/s²`;
    if ($('force-g-vertical')) $('force-g-vertical').textContent = `${(latestVerticalAccelIMU / G_ACC).toFixed(2)} G`;
    
    // Affichage NOUVEAU: Accélération 3D (Magnitude Linéaire)
    if ($('accel-3d-mag')) $('accel-3d-mag').textContent = `${accel_3d_mag.toFixed(3)} m/s²`;
    if ($('force-g-3d-mag')) $('force-g-3d-mag').textContent = `${(accel_3d_mag / G_ACC).toFixed(2)} G`;
    }
// =================================================================
// FICHIER JS PARTIE 2 : gnss-dashboard-part2.js
// (Logique Principale, Carte, Contrôles et Initialisation)
// =================================================================

// Les fonctions et variables globales sont définies dans part1.js.

// ... (fonctions de la carte et de contrôle GPS) ...

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

    if (now === null) { updateAstro(lat, lon); return; } 
    if (sTime === null) { sTime = now.getTime(); }
    if (acc > MAX_ACC) { 
        if ($('gps-precision')) $('gps-precision').textContent = `❌ ${acc.toFixed(0)} m (Trop Imprécis)`; 
        if (lPos === null) lPos = pos; return; 
    }

    let effectiveAcc = acc;
    const accOverride = parseFloat($('gps-accuracy-override').value);
    if (accOverride > 0) { effectiveAcc = accOverride; }

    let spdH = spd_raw_gps ?? 0; 
    const dt = lPos ? (cTimePos - lPos.timestamp) / 1000 : MIN_DT;

    // 1. FILTRAGE DE L'ALTITUDE (via Kalman, utilise latestVerticalAccelIMU de Part 1)
    const kAlt_new = kFilterAltitude(alt, effectiveAcc, dt, latestVerticalAccelIMU); 
    
    // 2. VITESSE VERTICALE
    let spdV = 0; 
    if (lPos && lPos.kAlt_old !== undefined && dt > MIN_DT && alt !== null) { 
        spdV = (kAlt_new - lPos.kAlt_old) / dt; 
    } else if (alt !== null) { 
        spdV = 0; 
    }
    
    // 3. VITESSE HORIZONTALE CALCULÉE ET VITESSE 3D
    if (lPos && dt > 0.05) { 
        const dH = dist(lPos.coords.latitude, lPos.coords.longitude, lat, lon); 
        spdH = dH / dt; 
    } 
    let spd3D = Math.sqrt(spdH ** 2 + spdV ** 2);

    // 4. FILTRE DE KALMAN FINAL (Vitesse 3D Stable)
    // R_dyn utilise latestLinearAccelMagnitude (l'accélération 3D linéaire, plus précise)
    const R_dyn = getKalmanR(effectiveAcc, alt, lastP_hPa, latestLinearAccelMagnitude); 
    const fSpd = kFilter(spd3D, dt, R_dyn); 
    const sSpdFE = fSpd < MIN_SPD ? 0 : fSpd; 
    
    // CALCUL DE LA MARGE D'ERREUR (3-Sigma = 3 * Écart-Type)
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
    
    // CALCUL DENSITÉ DE L'AIR et Pression Dynamique
    let airDensity = "N/A";
    let tempCMatch = $('temp-air') ? $('temp-air').textContent.match(/(-?[\d.]+)\s*°C/) : null;
    
    if (tempCMatch) {
        const tempC = parseFloat(tempCMatch[1]);
        const tempK = tempC + 273.15; 
        const pressurePa = lastP_hPa * 100; 
        const R_specific = 287.058; 
        if (!isNaN(tempK) && tempK > 0 && pressurePa > 0) {
            airDensity = (pressurePa / (R_specific * tempK)).toFixed(3);
        }
    }
    const dynamicPressure = (airDensity !== "N/A" && sSpdFE > 0) ? (0.5 * parseFloat(airDensity) * sSpdFE ** 2) : 0.0;


    // --- 6. MISE À JOUR DU DOM ---
    const sSpdFE_Kmh = (sSpdFE * KMH_MS);
    const distM_display = distM * (netherMode ? NETHER_RATIO : 1);

    // Vitesse/Distance
    if ($('speed-stable')) $('speed-stable').textContent = `${sSpdFE_Kmh.toFixed(5)} km/h`;
    if ($('speed-error-margin')) $('speed-error-margin').textContent = `± ${speedErrorMarginKmh.toFixed(3)} km/h`; 
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${sSpdFE.toFixed(3)} m/s | ${(sSpdFE * 1000).toFixed(0)} mm/s`;
    if ($('speed-3d-inst')) $('speed-3d-inst').textContent = `${(spd3D * KMH_MS).toFixed(5)} km/h`;
    // ... (Autres mises à jour de distance et de vitesse) ...
    if ($('distance-total-km')) $('distance-total-km').textContent = `${(distM_display / 1000).toFixed(3)} km | ${distM_display.toFixed(2)} m`;


    // ... (Mises à jour des coordonnées et des altitudes) ...

    // Dynamique et Forces
    if ($('vertical-speed')) $('vertical-speed').textContent = `${spdV.toFixed(2)} m/s`;
    if ($('accel-long')) $('accel-long').textContent = `${accel_long.toFixed(3)} m/s²`;
    if ($('force-g-long')) $('force-g-long').textContent = `${(accel_long / G_ACC).toFixed(2)} G`;
    if ($('kinetic-energy')) $('kinetic-energy').textContent = `${kineticEnergy.toFixed(2)} J`;
    if ($('mechanical-power')) $('mechanical-power').textContent = `${mechanicalPower.toFixed(2)} W`;
    if ($('coriolis-force')) $('coriolis-force').textContent = `${coriolusForce.toExponential(2)} N`;
    
    // ... (Mises à jour Relativité, Fluides, etc.) ...
    if ($('dynamic-pressure')) $('dynamic-pressure').textContent = `${dynamicPressure.toFixed(2)} Pa`;
    if ($('air-density')) $('air-density').textContent = airDensity + (airDensity !== "N/A" ? ' kg/m³' : '');

    // Vitesse du Son Dynamique
    if (tempCMatch) {
        const tempC = parseFloat(tempCMatch[1]);
        const soundSpeed = 331.3 + 0.606 * tempC; 
        const percSound = (sSpdFE / soundSpeed) * 100;
        if ($('perc-speed-sound')) $('perc-speed-sound').textContent = `${percSound.toFixed(2)} %`;
    } else {
        if ($('perc-speed-sound')) $('perc-speed-sound').textContent = `${(sSpdFE / SPEED_SOUND * 100).toFixed(2)} %`;
    }

    // --- 7. MISE À JOUR DES COMPOSANTS ET SAUVEGARDE ---
    updateAstro(lat, lon);
    updateMap(lat, lon);
    
    lPos = { 
        coords: pos.coords, 
        timestamp: cTimePos, 
        kAlt_old: kAlt_new, 
        kAltUncert_old: kAltUncert
    };
} 


// ... (INITIALISATION DES ÉVÉNEMENTS) ...
