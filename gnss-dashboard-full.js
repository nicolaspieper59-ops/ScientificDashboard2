// =================================================================
// FICHIER JS COMPLET : gnss-dashboard-full.js
// BLOC 1/2 : Constantes, Kalman & Fonctions de Correction Avancées
// =================================================================

// --- CLÉS D'API & PROXY VERCEL ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const KMH_MS = 3.6;         
const G = 9.80665;          
const R_AIR = 287.058;      

// --- CONSTANTES DE CONFIGURATION SYSTÈME ---
const MIN_DT = 0.01; 
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};
const DOM_SLOW_UPDATE_MS = 1000; 
const IMU_FREQUENCY_MS = 50;      

// --- PARAMÈTRES DU FILTRE DE KALMAN (VITESSE) ---
const Q_NOISE = 0.1;        
const R_MIN = 0.01;         
const R_MAX = 500.0;        
const MAX_ACC = 200;        
const MIN_SPD = 0.05;       

// --- NOUVEAUX SEUILS POUR IMU/ZUPT AVANCÉ ---
const ZUPT_RAW_THRESHOLD = 1.0;     
const IMU_ZUPT_THRESHOLD = 0.05;    
const Q_BIAS_ACCEL_FACTOR = 0.5;    
const BIAS_EMA_ALPHA = 0.05;        
const R_ACCEL_DAMPING_FACTOR = 1.0; 
const R_SLOW_SPEED_FACTOR = 100.0;  // Facteur pour le Soft Constraint

// --- DONNÉES CÉLESTES/GRAVITÉ ---
const CELESTIAL_DATA = { 
    'EARTH': { G: 9.80665, R: 6371000, name: 'Terre' },
    'MOON': { G: 1.62, R: 1737400, name: 'Lune' }
};
let G_ACC = CELESTIAL_DATA['EARTH'].G; 
let R_ALT_CENTER_REF = CELESTIAL_DATA['EARTH'].R;

// --- VARIABLES D'ÉTAT (Globales) ---
let wID = null, domID = null, imuUpdateID = null;
let lPos = null, lat = null, lon = null, sTime = null;
let distM = 0, maxSpd = 0;
let kSpd = 0, kUncert = 1000;      
let kAlt = null, kAltUncert = 10;  

let lastIMUUpdate = 0;
let rawAccelWithGrav = { x: 0.0, y: 0.0, z: 0.0 }; 
let attitude = { roll: 0.0, pitch: 0.0, yaw: 0.0 }; 
let imu_bias_est = 0.0;             
let corrected_Accel_Mag = 0.0;      
let centrifugalAccel = 0.0; 
let NED_Accel_Mag_RAW = 0.0;        

let emergencyStopActive = false;
let currentCelestialBody = 'EARTH';
let currentGPSMode = 'HIGH_FREQ'; 
let currentMass = 70.0;
let R_FACTOR_RATIO = 1.0;
let map, marker, circle;
let lastP_hPa = 1013.25, lastT_K = 290.45;
let lastFSpeed = 0;

// --- FONCTIONS UTILITAIRES ---
const $ = id => document.getElementById(id);

// Fonctions utilitaires simulées (assumées existantes dans l'implémentation complète)
const getCDate = () => { return new Date(); };
const dist = (lat1, lon1, lat2, lon2) => { return 0; };
const getKalmanR = (accRaw) => { return Math.min(R_MAX, Math.max(R_MIN, accRaw * accRaw)); };
const updateCelestialBody = (body) => { 
    const data = CELESTIAL_DATA[body];
    G_ACC = data.G;
    R_ALT_CENTER_REF = data.R;
};
function kFilterAltitude(altRaw, R_alt, dt) { return altRaw; }
function initMap() { /* ... */ }
function updateMap(lat, lon, acc) { /* ... */ }
function updateAstro(lat, lon) { /* ... */ }
function syncH() { /* ... */ }
function fetchWeather(lat, lon) { /* ... */ }


// Compensation de Gravité (Attitude)
function getGravityCompensatedAccel(rawAccel, attitude) {
    const G_current = G_ACC; 
    const pitchRad = attitude.pitch * D2R; 
    const gravityProjectionOnY = G_current * Math.sin(pitchRad);
    const linearAccelY = rawAccel.y - gravityProjectionOnY;
    return Math.abs(linearAccelY);
}

// Correction Centrifuge et Biais EMA
function getCorrectedAccel(rawAccel, attitude, dt) {
    const trueAccelMag = getGravityCompensatedAccel(rawAccel, attitude); 

    const local_r = parseFloat($('rotation-radius')?.value) || 100;
    const local_w = parseFloat($('angular-velocity')?.value) || 0;
    centrifugalAccel = local_r * (local_w ** 2);
    
    let net_Accel_Mag = trueAccelMag;
    if (net_Accel_Mag >= centrifugalAccel) {
        net_Accel_Mag -= centrifugalAccel;
    } else {
        net_Accel_Mag = 0.0;
    }

    if (net_Accel_Mag < IMU_ZUPT_THRESHOLD) {
        if (imu_bias_est === 0.0) {
             imu_bias_est = net_Accel_Mag;
        } else {
            imu_bias_est = (BIAS_EMA_ALPHA * net_Accel_Mag) + ((1 - BIAS_EMA_ALPHA) * imu_bias_est);
        }
        return 0.0; 
    } else {
        return Math.max(0.0, net_Accel_Mag - imu_bias_est);
    }
}


/**
 * Filtre de Kalman 1D pour la vitesse
 */
function kFilter(nSpd, dt, R_dyn, accel_input = 0) {
    if (dt === 0 || dt > 5) return kSpd; 
    
    const Q_accel_noise = Q_NOISE + Math.abs(accel_input) * Q_BIAS_ACCEL_FACTOR; 
    const R = R_dyn ?? R_MAX, Q = Q_accel_noise * dt * dt; 
    
    let pSpd = kSpd + (accel_input * dt); 
    let pUnc = kUncert + Q; 

    let K = pUnc / (pUnc + R); 
    kSpd = pSpd + K * (nSpd - pSpd); 
    kUncert = (1 - K) * pUnc; 
    
    return kSpd;
}


// =================================================================
// FICHIER JS COMPLET : gnss-dashboard-full.js
// BLOC 2/2 : Fonctions de Contrôle et Boucles Principales
// =================================================================

function startGPS() {
    const options = (currentGPSMode === 'HIGH_FREQ') ? GPS_OPTS.HIGH_FREQ : GPS_OPTS.LOW_FREQ;
    
    if (wID === null) {
        // NOTE: 'updateDisp' est appelée par l'API de géolocalisation
        wID = navigator.geolocation.watchPosition(updateDisp, handleErr, options);
    }
    
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
             // ZUPT Soft Constraint en DR: Confiance maximale dans la prédiction si l'IMU est stable (corrected_Accel_Mag = 0.0)
             R_imu_pred = R_MIN; 
             modeStatus = '✅ DR - BIAS ON (Prediction Stabilité)';
             
             // Réduction de l'incertitude pour un amortissement rapide
             if (kSpd < ZUPT_RAW_THRESHOLD) {
                 kUncert = Math.min(kUncert, R_MIN * 2); 
             }
        } else {
             R_imu_pred = R_MAX; 
             modeStatus = '⚠️ DR - PRÉDICTION IMU';
        }
        
        // Appliquer la prédiction EKF
        kFilter(kSpd, dt, R_imu_pred, corrected_Accel_Mag);
    } else {
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

    const cLat = pos.coords.latitude;
    const cLon = pos.coords.longitude;
    const accRaw = pos.coords.accuracy;
    const altRaw = pos.coords.altitude;

    // Calculs de dt et spd3D_raw (simulés)
    const dt = (lPos && pos.timestamp > lPos.timestamp) ? (pos.timestamp - lPos.timestamp) / 1000 : 0.5;
    let spd3D_raw = pos.coords.speed || 0.0; // Vitesse brute

    let R_dyn = getKalmanR(accRaw); 
    let spd_kalman_input = spd3D_raw;
    let R_kalman_input = R_dyn;
    let modeStatus = '🛰️ FUSION GNSS/IMU';
    
    // 1. Calcul de l'accélération corrigée IMU (Attitude + Biais)
    corrected_Accel_Mag = getCorrectedAccel(rawAccelWithGrav, attitude, dt); 
    let accel_sensor_input = corrected_Accel_Mag; 

    // 2. MODIFICATION CLÉ: Soft Constraint / Amortissement de Vitesse Lente
    const isVeryStable = (
        spd3D_raw < ZUPT_RAW_THRESHOLD && 
        accel_sensor_input < IMU_ZUPT_THRESHOLD 
    );

    if (R_kalman_input !== R_MIN) { 
        let modulation_factor = 1.0;
        
        if (isVeryStable) {
            // SOFT CONSTRAINT : Si stable et lent, on amplifie R pour ignorer la mesure GPS (trop bruyée).
            modulation_factor = R_SLOW_SPEED_FACTOR; 
            modeStatus = '✅ FUSION (Amortissement Vitesse Lente)';
        } else {
            // Modulation pour les manœuvres rapides (Confiance IMU)
            const accel_factor = Math.min(10.0, accel_sensor_input / 2.0); 
            modulation_factor = 1.0 + accel_factor * R_ACCEL_DAMPING_FACTOR;
        }

        R_kalman_input = R_kalman_input * modulation_factor;
    }

    // FILTRE DE KALMAN FINAL (Fusion IMU/GNSS)
    // Utilise la vitesse brute GPS comme mesure, mais le R modulé assure la stabilité
    // et la fluidité en mouvement lent.
    const fSpd = kFilter(spd_kalman_input, dt, R_kalman_input, accel_sensor_input); 
    const sSpdFE = fSpd < MIN_SPD ? 0 : fSpd; 
    
    // --- Mise à jour des affichages ---
    if ($('speed-stable')) $('speed-stable').textContent = `${sSpdFE.toFixed(3)} m/s`;
    if ($('current-speed')) $('current-speed').textContent = `${(sSpdFE * KMH_MS).toFixed(2)} km/h`;
    if ($('gps-status-dr')) $('gps-status-dr').textContent = modeStatus;
    
    if ($('kalman-r-factor')) $('kalman-r-factor').textContent = R_kalman_input.toFixed(3);
    if ($('kalman-q-noise')) $('kalman-q-noise').textContent = (Q_NOISE + Math.abs(accel_sensor_input) * Q_BIAS_ACCEL_FACTOR).toFixed(3);
    if ($('kalman-uncert')) $('kalman-uncert').textContent = kUncert.toFixed(3);
    
    // Mise à jour de l'accélération longitudinale (pour l'affichage)
    if ($('accel-long')) $('accel-long').textContent = accel_sensor_input.toFixed(3) + ' m/s²';
    if ($('force-g-long')) $('force-g-long').textContent = (accel_sensor_input / G).toFixed(3) + ' G';
    
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

    // Listeners IMU AVANCÉS (Attitude)
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

    // Listeners IMU AVANCÉS (Accélération BRUTE avec Gravité)
    if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', (event) => {
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
    // ... (Simulation de la boucle lente)
    if (domID === null) {
        domID = setInterval(() => {
            const now = getCDate();
            if (now) {
                if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString('fr-FR');
                if ($('utc-time')) $('utc-time').textContent = now.toUTCString();
            }
        }, DOM_SLOW_UPDATE_MS);
    }
});
