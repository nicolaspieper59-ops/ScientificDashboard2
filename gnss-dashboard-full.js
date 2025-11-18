// =================================================================
// BLOC 1/4 : Constantes, Fonctions Utilitaire et UKF (Structure)
// =================================================================

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const R_E_BASE = 6371000;   // Rayon terrestre moyen (m)
const KMH_MS = 3.6;         // Conversion m/s vers km/h
const C_S = 343;            // Vitesse du son (m/s)
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const R_AIR = 287.058;      // Constante spécifique de l'air sec (J/kg·K)
const BARO_ALT_REF_HPA = 1013.25;
const RHO_SEA_LEVEL = 1.225;

// --- PARAMÈTRES DU FILTRE (UKF) ---
const UKF_Q_NOISE = 0.1;        // Bruit de processus UKF
const UKF_R_MIN = 0.01;         // Bruit de mesure minimum
const UKF_R_MAX = 500.0;        // Bruit de mesure maximum
const MAX_ACC = 200;            // Précision max (m) avant "Estimation Seule"
const MIN_SPD = 0.05;           // Vitesse minimale "en mouvement"
const ALT_TH = -50;             // Seuil d'altitude "Sous-sol"
const MAX_PLAUSIBLE_ACCEL = 20.0; // Anti-spike (m/s²)
const NETHER_RATIO = 8.0;

// --- SEUILS ZUPT (Zero Velocity Update) ---
const ZUPT_RAW_THRESHOLD = 1.0;
const ZUPT_ACCEL_THRESHOLD = 0.5;

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

// --- UKF PLACEHOLDER (Doit être un fichier externe comme math.js) ---
class ProfessionalUKF {
    constructor() {
        // État initial [lat, lon, alt, vx, vy, vz, qx, qy, qz, qw, bias_ax, bias_ay, bias_az, bias_gx, bias_gy, bias_gz]
        this.state = { lat: 0.0, lon: 0.0, alt: 0.0, speed: 0.0, kUncert: 1000, kAltUncert: 10 };
    }
    // La fonction de prédiction utilise Accel et Gyro (IMU)
    predict(imuReadings, dt) {
        // Logique de prédiction mathématique basée sur le modèle cinématique (accel/gyro)
        // ... (Logique complexe omise) ...
        this.state.lat += imuReadings.gyro[0] * dt * 1e-5; // Placeholder simpliste pour mouvement
        this.state.lon += imuReadings.gyro[1] * dt * 1e-5;
        this.state.speed += imuReadings.accel[0] * dt;
        this.state.kUncert += UKF_Q_NOISE * dt;
        this.state.kAltUncert += Q_ALT_NOISE * dt;
    }
    // La fonction de correction utilise GPS
    update(gpsCoords, R_dyn) {
        // Logique de correction mathématique basée sur la mesure GPS
        // ... (Logique complexe omise) ...
        this.state.lat = gpsCoords.latitude; // Placeholder simpliste
        this.state.lon = gpsCoords.longitude;
        this.state.alt = gpsCoords.altitude || 0;
        this.state.speed = gpsCoords.speed || 0;
        this.state.kUncert = Math.max(UKF_R_MIN, this.state.kUncert / 2);
        this.state.kAltUncert = Math.max(R_ALT_MIN, this.state.kAltUncert / 2);
    }
    getState() { return this.state; }
}

// --- FONCTIONS UTILITAIRES ---

/** Formate les données pour l'affichage DOM */
function dataOrDefault(value, precision = 2, unit = '') {
    if (value === null || value === undefined || isNaN(value)) return `N/A${unit}`;
    return `${value.toFixed(precision)}${unit}`;
}
function dataOrDefaultExp(value, precision = 2, unit = '') {
    if (value === null || value === undefined || isNaN(value) || value === 0) return `0.00e+0${unit}`;
    return `${value.toExponential(precision)}${unit}`;
}
const $ = id => document.getElementById(id);


// --- FONCTIONS PHYSIQUES (Inchangées) ---

const dist = (lat1, lon1, lat2, lon2, R_ref) => { /* ... */ };
function getGravityLocal(alt, bodyKey, r_rot, omega_rot) { /* ... */ };
function updateCelestialBody(bodyKey, alt, r_rot, omega_rot) { 
    // Initialisation globale de G_ACC et R_ALT_CENTER_REF 
    const data = CELESTIAL_DATA[bodyKey];
    if (data) {
        G_ACC = data.G; 
        R_ALT_CENTER_REF = data.R;
    }
    // ... (Logique complète omise) ... 
    return { G_ACC: G_ACC, R_ALT_CENTER_REF: R_ALT_CENTER_REF };
};
function calculateMRF(alt, netherMode) { /* ... */ };
function getSpeedOfSound(tempK) { return 20.05 * Math.sqrt(tempK); }

/** Calcule le Facteur R (Confiance GPS) du filtre de Kalman. */
function getKalmanR(acc, alt, P_hPa, selectedEnv, R_FACTOR_RATIO) {
    let acc_effective = acc;
    if (acc > MAX_ACC) return 1e9; // Confiance nulle
    let R = acc_effective * acc_effective; 
    
    const envFactor = ENVIRONMENT_FACTORS[selectedEnv]?.R_MULT || 1.0;
    R *= envFactor;
    
    if (P_hPa !== null) {
        const pressureFactor = 1.0 + (BARO_ALT_REF_HPA - P_hPa) / BARO_ALT_REF_HPA * 0.1;
        R *= Math.max(1.0, pressureFactor); 
    }
    
    if (alt !== null && alt < ALT_TH) { 
        R *= 2.0;
    } 
    return Math.max(UKF_R_MIN, Math.min(UKF_R_MAX, R)) * R_FACTOR_RATIO;
    }
// =================================================================
// BLOC 2/4 : Filtres, Modèles Cinématiques et Quaternion (MISE À JOUR)
// =================================================================

// --- CLASSE QUATERNION (Pour gestion de l'IMU) ---
class Quaternion {
    constructor(w = 1, x = 0, y = 0, z = 0) {
        this.w = w; this.x = x; this.y = y; this.z = z;
    }
    // ... (Logique Quaternion complète omise pour la concision)
    static fromAcc(ax, ay, az) { return new Quaternion(); }
    toEuler() { return { roll: 0, pitch: 0, yaw: 0 }; }
}

// ===========================================
// CLASSE UKF PROFESSIONNELLE (Architecture 21 États)
// ===========================================
class ProfessionalUKF {
    constructor() {
        if (typeof math === 'undefined') {
            throw new Error("math.js n'est pas chargé. L'UKF 21 états ne peut pas fonctionner.");
        }
        this.N_STATES = 21; 
        this.x = math.zeros(this.N_STATES); 
        this.P = math.diag(Array(this.N_STATES).fill(1e-2)); 
        this.Q = math.diag(Array(this.N_STATES).fill(1e-6));
        this.isReady = true;
        // ... (Logique UKF complète omise pour la concision)
    }

    predict(imuData, dt) {
        if (!this.isReady) return;
        // PLACEHOLDER (Simulation simplifiée pour faire bouger les chiffres):
        const ax_corr = imuData.accel[0]; 
        let vN = this.x.get([3]) + ax_corr * dt; 
        if (Math.abs(vN) < MIN_SPD) vN = 0;
        this.x.set([3], vN); 
    }

    update(gpsData) {
        if (!this.isReady) return;
        // PLACEHOLDER : Correction directe
        this.x.set([0], gpsData.latitude * D2R);
        this.x.set([1], gpsData.longitude * D2R);
        this.x.set([2], gpsData.altitude);
        if (gpsData.speed !== null) {
            const oldSpeed = this.x.get([3]);
            this.x.set([3], oldSpeed * 0.8 + gpsData.speed * 0.2);
        }
    }
    
    getState() {
        if (!this.isReady) return { lat: 0, lon: 0, alt: 0, speed: 0, kUncert: 0, kAltUncert: 0 };
        const x_data = this.x.toArray();
        return {
            lat: x_data[0] * R2D,
            lon: x_data[1] * R2D,
            alt: x_data[2],
            vN: x_data[3], vE: x_data[4], vD: x_data[5],
            speed: Math.sqrt(x_data[3]**2 + x_data[4]**2 + x_data[5]**2),
            kUncert: this.P.get([3, 3]) + this.P.get([4, 4]) + this.P.get([5, 5]),
            kAltUncert: this.P.get([2, 2])
        };
    }
}


// --- FONCTIONS DE FILTRAGE ET DE MODÈLE (Altitude, Bruit, etc.) ---

function getKalmanR(accRaw, kAlt, kUncert, env, reactivityMode) {
    let R_gps_base = Math.min(accRaw, 100) ** 2; 
    if (accRaw > 100) { R_gps_base = 100 ** 2 + (accRaw - 100) * 10; }
    
    const env_mult = ENVIRONMENT_FACTORS[env]?.R_MULT || 1.0;
    let reactivity_mult = UKF_REACTIVITY_FACTORS[reactivityMode]?.MULT || 1.0;
    
    if (reactivityMode === 'AUTO' && accRaw !== null) {
        if (accRaw > 20) { reactivity_mult = 3.0; }
        else if (accRaw < 3) { reactivity_mult = 0.5; }
    }
    
    let R_dyn = Math.min(R_gps_base * env_mult * reactivity_mult, UKF_R_MAX);
    if (kUncert > 100) { R_dyn *= 1.1; }
    
    return Math.max(R_dyn, 0.1); 
}

function getBarometricAltitude(P_hPa, P_ref_hPa, T_K) {
    if (P_hPa === null || T_K === null) return null;
    const L = 0.0065;
    const g = G_ACC; 
    const R = R_AIR; 
    const alt = (T_K / L) * (1 - (P_hPa / P_ref_hPa)**(R * L / g));
    return alt;
}

function calculateMRF(alt) {
    if (alt < -20) { return 1.1; } 
    return 1.0;
}

function updateCelestialBody(body, alt, rotationRadius = 0, angularVelocity = 0) {
    let G_ACC_NEW = CELESTIAL_DATA['EARTH'].G;
    let R_ALT_CENTER_REF_NEW = WGS84_A; 
    const data = CELESTIAL_DATA[body];
    if (data) {
        G_ACC_NEW = data.G;
        R_ALT_CENTER_REF_NEW = data.R;
    }
    G_ACC = G_ACC_NEW;
    R_ALT_CENTER_REF = R_ALT_CENTER_REF_NEW;
    return { G_ACC: G_ACC_NEW, R_ALT_CENTER_REF: R_ALT_CENTER_REF_NEW };
            }
// =================================================================
// BLOC 3/4 : Logique Applicative Principale (IMU/GPS/Boucle Rapide) (CORRIGÉ)
// =================================================================

// --- CONSTANTES DE CONFIGURATION SYSTÈME ---
const MIN_DT = 0.01; 
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};
const DOM_SLOW_UPDATE_MS = 1000;
const IMU_UPDATE_RATE_MS = 20; // Taux de la boucle rapide (50 Hz)
const STANDBY_TIMEOUT_MS = 30000; // 30s avant de passer en LOW_FREQ
const MAP_UPDATE_INTERVAL = 3000; 

// --- VARIABLES D'ÉTAT (Globales) ---
let wID = null, domSlowID = null, domFastID = null, lPos = null, lat = 0.0, lon = 0.0, sTime = null;
let distM = 0, maxSpd = 0;
let kSpd = 0, kUncert = UKF_R_MAX, kAltUncert = 10; 
let timeMoving = 0, timeTotal = 0; 
let lastFSpeed = 0; 
let kAlt = 0.0;      
let ukf = null; 

let currentGPSMode = 'HIGH_FREQ'; 
let emergencyStopActive = false; 
let selectedEnvironment = 'NORMAL'; 
let currentMass = 70.0; 
let R_FACTOR_RATIO = 1.0;
let currentCelestialBody = 'EARTH';
let gpsAccuracyOverride = 0.0; 
let lastGPSPos = null;

let lServH = null, lLocH = null; 
let G_ACC = CELESTIAL_DATA.EARTH.G;
let R_ALT_CENTER_REF = R_E_BASE;

let lastP_hPa = BARO_ALT_REF_HPA, lastT_K = 288.15, currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound = C_S;

// DONNÉES IMU (Utilisées par la Fast Loop)
let accel = { x: 0, y: 0, z: 0 };
let gyro = { x: 0, y: 0, z: 0 };
let mag = { x: 0, y: 0, z: 0 }; // Magnétomètre non implémenté ici
let lastIMUTimestamp = 0;

let map = null, marker = null, circle = null; // Variables Leaflet
let gpsStandbyTimeoutID = null;    

// --- GESTION DES CAPTEURS (IMU) ---

/**
 * @function startIMUListeners
 * Utilise les APIs dédiées pour une meilleure performance et un accès clair aux données Gyro.
 */
function startIMUListeners() {
    if (emergencyStopActive) return;
    try {
        if ($('imu-status')) $('imu-status').textContent = "Activation...";
        
        if (typeof Accelerometer === 'undefined' || typeof Gyroscope === 'undefined') {
             throw new Error("API Sensor non supportée (Accéléromètre/Gyroscope manquant).");
        }
        
        // 1. ACCÉLÉROMÈTRE
        const accSensor = new Accelerometer({ frequency: 50 }); 
        accSensor.addEventListener('reading', () => {
            accel.x = accSensor.x;
            accel.y = accSensor.y;
            accel.z = accSensor.z;
        });
        accSensor.addEventListener('error', event => {
            if ($('imu-status')) $('imu-status').textContent = `Erreur Accél: ${event.error.name}`;
            console.error("Erreur Accéléromètre:", event.error);
        });
        accSensor.start();

        // 2. GYROSCOPE (CRITIQUE pour le filtre)
        const gyroSensor = new Gyroscope({ frequency: 50 });
        gyroSensor.addEventListener('reading', () => {
            gyro.x = gyroSensor.x;
            gyro.y = gyroSensor.y;
            gyro.z = gyroSensor.z;
        });
        gyroSensor.addEventListener('error', event => {
             // C'est ici que le NotReadableError apparaît
             if ($('imu-status')) $('imu-status').textContent = `Erreur Gyro: ${event.error.name}`;
             console.error("Erreur Gyroscope:", event.error);
        });
        gyroSensor.start();
        
        if ($('imu-status') && !$('imu-status').textContent.includes('Erreur')) {
            $('imu-status').textContent = "Actif (API Sensor)";
        }
        lastIMUTimestamp = performance.now();
        
    } catch (error) {
        let errMsg = error.message;
        if (error.name === 'SecurityError' || error.name === 'NotAllowedError') {
            errMsg = "Permission Capteurs Refusée.";
        } else if (error.name === 'NotReadableError') {
             errMsg = "Capteurs Verrouillés par l'OS (Mode économie ou usage externe).";
        }
        if ($('imu-status')) $('imu-status').textContent = `❌ ${errMsg}`;
        console.error("ERREUR CRITIQUE IMU:", error.name, errMsg);
    }
}

function stopIMUListeners() {
    // Dans une implémentation réelle, il faudrait arrêter les capteurs avec sensor.stop()
    if ($('imu-status')) $('imu-status').textContent = "Inactif";
    accel = { x: 0, y: 0, z: 0 };
    gyro = { x: 0, y: 0, z: 0 };
}

// --- GESTION GPS (CORRECTION UKF) ---
function startGPS(mode = currentGPSMode) { /* ... (Inchangé) ... */ }
function stopGPS(resetButton = true) { /* ... (Inchangé) ... */ }
function toggleGPS() { /* ... (Inchangé) ... */ }
function handleErr(err) { /* ... (Inchangé) ... */ }

/** * @function gpsUpdateCallback
 * Fonction appelée par le GPS : elle effectue la CORRECTION (Update) de l'UKF.
 */
function gpsUpdateCallback(pos) {
    if (emergencyStopActive || !ukf) return;
    
    lastGPSPos = pos; 
    const { accuracy } = pos.coords;
    
    const accRaw = gpsAccuracyOverride || accuracy || 100;
    
    // Le filtre UKF gère la confiance R_dyn, nous appelons TOUJOURS update.
    let R_dyn = getKalmanR(accRaw, kAlt, lastP_hPa, selectedEnvironment, R_FACTOR_RATIO); 
    
    // CORRECTION UKF (GPS)
    ukf.update(pos.coords, R_dyn);

    // Mise à jour de l'affichage de la précision GPS
    if ($('gps-precision')) {
        if (accRaw > 15) {
            $('gps-precision').textContent = `❌ ${accRaw.toFixed(0)} m (Estimation)`;
        } else {
            $('gps-precision').textContent = `✅ ${accRaw.toFixed(2)} m (Actif)`;
        }
    }
}

// --- MAP (Leaflet) ---
function initMap() { /* ... (Inchangé) ... */ }
function updateMap(lat, lon, acc) { /* ... (Inchangé) ... */ }

// --- ARRÊT D'URGENCE ---
function emergencyStop() { /* ... (Inchangé) ... */ }
function resumeSystem() { /* ... (Inchangé) ... */ }

/**
 * @function startFastLoop
 * BOUCLE RAPIDE (IMU) - Elle effectue la PRÉDICTION (Predict) de l'UKF.
 * Elle est indépendante de la fréquence GPS et tourne à 50Hz.
 */
function startFastLoop() {
    if (domFastID) return; 
    
    domFastID = setInterval(() => {
        if (emergencyStopActive || !ukf) return;
        
        const now = performance.now();
        const dt = (now - lastIMUTimestamp) / 1000.0;
        if (dt < MIN_DT) return; 
        lastIMUTimestamp = now;

        // --- 1. PRÉDICTION UKF ---
        // Cette étape utilise l'ACCÉLÉROMÈTRE ET LE GYROSCOPE
        const imuReadings = {
            accel: [accel.x, accel.y, accel.z], // Accélération
            gyro: [gyro.x, gyro.y, gyro.z]      // Vitesse Angulaire
        };
        ukf.predict(imuReadings, dt);

        // --- 2. EXTRACTION DE L'ÉTAT ---
        const estimatedState = ukf.getState();
        lat = estimatedState.lat;
        lon = estimatedState.lon;
        kAlt = estimatedState.alt;
        kSpd = estimatedState.speed;
        kUncert = estimatedState.kUncert;
        kAltUncert = estimatedState.kAltUncert;

        const sSpdFE = kSpd < MIN_SPD ? 0 : kSpd;
        const spd3D_raw_gps = (lastGPSPos && lastGPSPos.coords.speed !== null) ? lastGPSPos.coords.speed : 0;
        
        // --- 3. CALCULS AVANCÉS ---
        const accel_long = accel.x; // Accélération longitudinale (simplifié)
        R_FACTOR_RATIO = calculateMRF(kAlt); 
        distM += sSpdFE * dt * R_FACTOR_RATIO; 
        
        if (sSpdFE > MIN_SPD) { timeMoving += dt; }
        if (sTime) { timeTotal = (Date.now() - sTime) / 1000; }
        
        if (spd3D_raw_gps > maxSpd) maxSpd = spd3D_raw_gps; 
        
        // --- 4. MISE À JOUR DU DOM (Rapide) ---
        // (Toutes les mises à jour utilisant les données UKF/IMU)
        
        $('elapsed-time').textContent = dataOrDefault(timeTotal, 2, ' s');
        $('time-moving').textContent = dataOrDefault(timeMoving, 2, ' s');
        
        $('speed-stable').textContent = dataOrDefault(sSpdFE * KMH_MS, 2);
        $('speed-stable-ms').textContent = dataOrDefault(sSpdFE, 3, ' m/s');
        $('speed-stable-kms').textContent = dataOrDefaultExp(sSpdFE / 1000, 3, ' km/s');
        
        $('accel-long').textContent = dataOrDefault(accel_long, 3, ' m/s²'); 
        $('angular-speed').textContent = dataOrDefault(Math.sqrt(gyro.x**2 + gyro.y**2 + gyro.z**2) * R2D, 2, ' °/s');
        
        $('accel-x').textContent = dataOrDefault(accel.x, 2, ' m/s²');
        $('accel-y').textContent = dataOrDefault(accel.y, 2, ' m/s²');
        $('accel-z').textContent = dataOrDefault(accel.z, 2, ' m/s²');

        if (lat !== 0) $('lat-display').textContent = dataOrDefault(lat, 6, ' °');
        if (lon !== 0) $('lon-display').textContent = dataOrDefault(lon, 6, ' °');
        if (kAlt !== 0) $('alt-display').textContent = dataOrDefault(kAlt, 2, ' m');
        
        updateMap(lat, lon, (lastGPSPos ? lastGPSPos.coords.accuracy : 100));

    }, IMU_UPDATE_RATE_MS);
}
// ===========================================
// BLOC 4/4 : Initialisation DOM et Boucle Lente
// ===========================================

JavaScript

// ===========================================
// BLOC 4/4 : Initialisation DOM et Boucle Lente (CORRIGÉ POUR LE CLIC)
// ===========================================

function initSystem() {
    if (sTime !== null) return; // Empêche la réinitialisation si déjà démarré
    
    // 1. Initialisation UKF
    if (typeof ProfessionalUKF === 'undefined') {
        alert("Erreur critique: La classe ProfessionalUKF est manquante.");
        return;
    }
    ukf = new ProfessionalUKF();

    // 2. Démarrage des systèmes critiques
    sTime = Date.now();
    startGPS();        
    startIMUListeners(); 
    startFastLoop();

    // 3. Sync NTP en parallèle
    syncH(lServH, lLocH).then(newTimes => {
        lServH = newTimes.lServH;
        lLocH = newTimes.lLocH;
    });

    // 4. Démarrage de la boucle lente (Astro/Météo)
    const DOM_SLOW_UPDATE_MS = 3000;
    domSlowID = setInterval(() => {
        const currentLat = lat || 43.296; 
        const currentLon = lon || 5.37;

    }, DOM_SLOW_UPDATE_MS); 

    // Cache le bouton de démarrage
    if ($('init-system-btn')) $('init-system-btn').style.display = 'none';
    
    // Mettre à jour l'état GPS du bouton toggle-gps-btn
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = '⏸️ PAUSE GPS';
}


document.addEventListener('DOMContentLoaded', () => {
    
    initMap(); 
    
    // Les valeurs initiales doivent être mises en place avant le démarrage
    updateCelestialBody(currentCelestialBody, kAlt, 0, 0);

    // --- LIER LE BOUTON DÉMARRER LE SYSTÈME ---
    const startButton = $('init-system-btn'); // Assurez-vous que l'ID du bouton est 'init-system-btn'
    if (startButton) {
        startButton.addEventListener('click', initSystem, { once: true });
    } else {
         // Si le bouton n'existe pas, nous démarrons immédiatement (Fallback)
         initSystem();
    }
    
    // --- Autres écouteurs (Inchangés) ---
    if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', toggleGPS);
    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', toggleEmergencyStop);
    // ... (Ajouter ici tous les autres écouteurs de contrôles)
});
