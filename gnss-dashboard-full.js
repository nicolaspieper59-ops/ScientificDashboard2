// ====================================================================
// FICHIER JS COMPLET : gnss-dashboard-full.js
// ====================================================================

// ====================================================================
// BLOC 1/3 : INITIALISATION, CONSTANTES & FILTRES SCIENTIFIQUES 
// ====================================================================

// --- Constantes de Conversion & Temps (inchangées) ---
const R2D = 180 / Math.PI;
const D2R = Math.PI / 180;
const G = 9.80665; 
const R_AIR = 287.05;
const GAMMA_AIR = 1.4;
const P_SEA_hPa = 1013.25;
const DOM_SLOW_UPDATE_MS = 1000;
const IMU_FREQUENCY_MS = 50; 

// --- Constantes du Filtre de Kalman & ZUPT ---
const MAX_ACC = 20;       
const R_MIN = 0.001;
const R_MAX = 100000.0;   
const Q_BASE_NOISE = 0.03; 
// MODIFIÉ: Facteur de bruit réduit (car on soustrait maintenant le biais)
const IMU_ACCEL_NOISE_FACTOR = 0.5; 
const ZUPT_RAW_THRESHOLD = 0.5; 
const ACCURACY_NOISE_FACTOR = 3.0; 
const MIN_DISPLAY_SPEED_MS = 0.0001; 
const IMU_ZUPT_THRESHOLD = 0.05; 

// --- Variables d'État Globales ---
let wID = null;
let domID = null;
let imuUpdateID = null; 
let lastIMUUpdate = 0; 
let lPos = {};
let kSpd = 0;
let kAlt = null;
let emergencyStopActive = false;
let totalDistance = 0; 
let kAltFilter;
let lastGPSPos = null; 

// NOUVEAU: Variables d'estimation du Biais (pour la dérive en DR)
let imu_bias_est = 0.0;
let imu_bias_count = 0;

// Données IMU 3D et Attitude
let imuData = { x: 0.0, y: 0.0, z: 0.0 }; 
let attitude = { roll: 0.0, pitch: 0.0, yaw: 0.0 }; 
let NED_Accel_Mag = 0.0; 
let corrected_Accel_Mag = 0.0; // Accélération nette pour l'EKF

// Variables de Manège (Manège rotatif)
let rotationRadius = 10.0; 
let angularVelocity = 0.0; 
let centrifugalAccel = 0.0;
let coriolisForce = 0.0; 

// Placeholders Météo (inchangés)
let currentTempC = 17.3;
let currentPressurehPa = 1015;
let currentHumidity = 72;

// --- Instanciation des Filtres (inchangée) ---
let kFilter = createKalmanFilter(Q_BASE_NOISE, R_MAX); 
kAltFilter = createSimpleKalmanFilter(0.1, R_MIN);

// --- Fonctions de Filtrage (inchangées) ---
function createSimpleKalmanFilter(Q, R) {
    let X = 0; let P = R; 
    return function(Z, R_new, dt) {
        let P_pred = P + Q * dt;
        let K = P_pred / (P_pred + R_new);
        X = X + K * (Z - X);
        P = (1 - K) * P_pred;
        return X;
    }
}

function createKalmanFilter(initialQ, initialR) {
    let X = [[0], [0]]; 
    let P = [[initialR, 0], [0, initialR]]; 
    
    return function(Z, dt, R_new, U_accel_sensor, isPredictionOnly = false) { 
        // 1. DÉFINITION DYNAMIQUE DE Q 
        const Q_dt_factor = Math.min(1.0, 1.0 / (dt * 10)); 
        const Q_noise = Q_BASE_NOISE * Q_dt_factor + (Math.abs(U_accel_sensor) * IMU_ACCEL_NOISE_FACTOR); 
        const Q_matrix = [[0.001, 0], [0, Q_noise * Q_noise]]; 
        
        // Prédiction (X_k = F * X_{k-1} + B * U)
        const F = [[1, dt], [0, 1]];
        const B = [[0.5 * dt * dt], [dt]]; 
        const U = [[U_accel_sensor]]; 
        
        X[0][0] = F[0][0] * X[0][0] + F[0][1] * X[1][0] + B[0][0] * U[0][0];
        X[1][0] = F[1][0] * X[0][0] + F[1][1] * X[1][0] + B[1][0] * U[0][0];
        
        // P_k = F * P_{k-1} * F^T + Q
        let P_pred = [[P[0][0] + dt * (P[0][1] + P[1][0]) + dt * dt * P[1][1], P[0][1] + dt * P[1][1]],
                      [P[1][0] + dt * P[1][1], P[1][1]]];
        
        P_pred[0][0] += Q_matrix[0][0]; P_pred[1][1] += Q_matrix[1][1];
        P = P_pred;

        if (isPredictionOnly) {
             kSpd = X[1][0];
             return { speed: X[1][0], Q_used: Q_noise, R_used: R_new };
        }
        
        // Mise à jour de la Mesure (Correction/Update Step)
        const R_new_matrix = [[R_new, 0], [0, R_new]];
        const Y = Z - X[1][0]; 
        const S = P[1][1] + R_new_matrix[1][1]; 
        const K = [[P[0][1] / S], [P[1][1] / S]]; 

        X[0][0] = X[0][0] + K[0][0] * Y; X[1][0] = X[1][0] + K[1][0] * Y;
        P[0][0] = P[0][0] - K[0][0] * P[1][0]; P[0][1] = P[0][1] - K[0][0] * P[1][1];
        P[1][0] = P[1][0] - K[1][0] * P[1][0]; P[1][1] = P[1][1] - K[1][0] * P[1][1];

        kSpd = X[1][0];
        return { speed: X[1][0], Q_used: Q_noise, R_used: R_new };
    }
}


// ====================================================================
// BLOC 2/3 : GESTION DES CAPTEURS, MÉTÉO & LOGIQUE DE FUSION 
// ====================================================================

// --- Fonctions utilitaires (omises) ---
function getVirtualTemperature(T_C, HR_percent, P_hPa) { 
    if (!P_hPa || !T_C || !HR_percent) return T_C + 273.15;
    const T_K = T_C + 273.15;
    const HR = HR_percent / 100;
    const Pvs_Pa = 611 * Math.exp((17.27 * T_C) / (237.3 + T_C));
    const Pv_Pa = HR * Pvs_Pa;
    const r = (0.622 * Pv_Pa) / (P_hPa * 100 - Pv_Pa);
    return T_K * (1 + 0.61 * r);
}
function getBarometricAltitude(P_hPa, Tv_K) { 
    if (P_hPa === null || isNaN(P_hPa)) return null;
    const exponent = (GAMMA_AIR - 1) / GAMMA_AIR;
    const alt = ((R_AIR * Tv_K) / (G * exponent)) * (1 - Math.pow(P_hPa / P_SEA_hPa, exponent));
    return Math.max(0.0, alt); 
}
function getSpeedOfSound(T_C) { return 331.3 * Math.sqrt(1 + T_C / 273.15); }
function getAirDensity(P_hPa, Tv_K) { return (P_hPa * 100) / (R_AIR * Tv_K); }
function getFrostPoint(T_C, HR_percent) {
    const A = 17.27; const B = 237.3;
    const alpha = (A * T_C) / (B + T_C) + Math.log(HR_percent / 100);
    return (B * alpha) / (A - alpha);
}
function getKalmanR(accRaw) {
    if (accRaw === null || accRaw > MAX_ACC) return R_MAX;
    return Math.max(R_MIN, accRaw * accRaw);
}
function getLocalSiderealTime(date, longitudeDeg) {
    return ((date.getUTCHours() + longitudeDeg / 15.0) % 24);
}

function startGPS() {
    // NOUVEAU: Réinitialisation du Bias IMU au démarrage du GPS
    imu_bias_est = 0.0;
    imu_bias_count = 0;
    
    if (wID === null) {
        wID = navigator.geolocation.watchPosition(updateGPSCorrection, handleError, { enableHighAccuracy: true, maximumAge: 500, timeout: 10000 });
        $('gps-pause-button').textContent = '⏸️ PAUSE GPS';
    }
    if (imuUpdateID === null) {
        imuUpdateID = setInterval(updateIMUPrediction, IMU_FREQUENCY_MS);
    }
}
function stopGPS() {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID); wID = null;
        $('gps-pause-button').textContent = '▶️ MARCHE GPS';
    }
    if (imuUpdateID !== null) {
        clearInterval(imuUpdateID); imuUpdateID = null;
    }
}
function handleError(error) { console.warn(`GPS ERROR: ${error.code}: ${error.message}`); }


// --- LOGIQUE DE PRÉDICTION IMU HAUTE FRÉQUENCE (20 Hz) ---
function updateIMUPrediction() {
    if (emergencyStopActive) return;

    const now = new Date();
    const time = now.getTime();
    if (lastIMUUpdate === 0) lastIMUUpdate = time;
    
    const dt_imu = (time - lastIMUUpdate) / 1000;
    const dt = (dt_imu > 0 && dt_imu < 1) ? dt_imu : IMU_FREQUENCY_MS / 1000; 
    
    lastIMUUpdate = time;

    // 1. Calcul des forces artificielles (Simulation)
    rotationRadius = parseFloat($('rotation-radius').value) || 0.0;
    angularVelocity = parseFloat($('angular-velocity').value) || 0.0;
    
    centrifugalAccel = rotationRadius * (angularVelocity ** 2);
    coriolisForce = 2 * angularVelocity * Math.abs(kSpd); 

    // 2. CORRECTION DE LA MAGNITUDE IMU
    
    // Étape A : Soustraction de l'Accélération Centrifuge (Force non-linéaire)
    let net_Accel_Mag = NED_Accel_Mag; 
    if (net_Accel_Mag >= centrifugalAccel) {
        net_Accel_Mag -= centrifugalAccel;
    } else {
        net_Accel_Mag = Q_BASE_NOISE; 
    }
    
    // Étape B : Soustraction du Biais Estimé (Correction essentielle contre la dérive)
    // L'accélération d'entrée pour l'EKF est l'accélération nette moins le biais du capteur.
    corrected_Accel_Mag = Math.max(0.0, net_Accel_Mag - imu_bias_est);


    // 3. LOGIQUE ZUPT et PRÉDICTION
    let kalmanResult;
    let R_corr = R_MAX; 

    if (wID === null) { 
        // Mode DR (GPS Pausé)
        
        if (corrected_Accel_Mag < IMU_ZUPT_THRESHOLD) {
            // ZUPT Actif : Correction forcée à Vitesse = 0 avec haute confiance
            kalmanResult = kFilter(0.0, dt, R_MIN, corrected_Accel_Mag, false);
            R_corr = R_MIN;
            $('gps-status').textContent = '✅ DR - ZUPT ON (IMU)';
            $('zupt-active').textContent = 'Oui (IMU/DR)';
            
            // NOUVEAU: Accumulation du Bias (la valeur nette restante est le bias)
            // Moyenne mobile simple : imu_bias_est est l'accélération résiduelle
            imu_bias_est = (imu_bias_est * imu_bias_count + net_Accel_Mag) / (imu_bias_count + 1);
            imu_bias_count++;
            
        } else {
            // Mouvement détecté : Prediction pure EKF
            kalmanResult = kFilter(kSpd, dt, R_MAX, corrected_Accel_Mag, true);
            $('gps-status').textContent = '⚠️ DR - PRÉDICTION IMU';
            $('zupt-active').textContent = 'Non';
            
            // NOUVEAU: Reset du compteur de bias
            imu_bias_count = 0;
        }
    } else {
        // GPS Actif - On ne fait que la prédiction. Correction dans updateGPSCorrection.
        kalmanResult = kFilter(kSpd, dt, R_MAX, corrected_Accel_Mag, true);
        $('gps-status').textContent = '✅ GPS OK - Prédiction';
        $('zupt-active').textContent = 'Non';
    }

    // Seuil minimal pour le clamping
    const sSpdAbs = Math.abs(kalmanResult.speed);
    const sSpdFE = sSpdAbs < MIN_DISPLAY_SPEED_MS ? 0 : sSpdAbs;
    
    if (totalDistance > 1000000) totalDistance = 0; 
    if (sSpdFE > MIN_DISPLAY_SPEED_MS) totalDistance += sSpdFE * dt;
    
    // Mise à jour du DOM à haute fréquence
    updateDOMHighFrequency(dt, kalmanResult.Q_used, R_corr, now, lastGPSPos);
}


// --- LOGIQUE DE CORRECTION GPS BASSE FRÉQUENCE (updateGPSCorrection) (Utilise corrected_Accel_Mag pour l'EKF)---
function updateGPSCorrection(pos) {
    if (emergencyStopActive || wID === null) return;
    
    lastGPSPos = pos;
    const now = new Date();
    const time = now.getTime();
    const { latitude: latRaw, longitude: lonRaw, altitude: altRaw, accuracy: accRaw } = pos.coords;

    // ... (Calcul de vitesse brute GPS inchangé) ...
    let spd3D_raw = 0;
    let dt_gps_raw = 0.05; 
    if (lPos.time && time > lPos.time) {
        dt_gps_raw = (time - lPos.time) / 1000;
        const R_EARTH = 6371e3;
        const dLat = (latRaw - lPos.lat) * D2R; const dLon = (lonRaw - lPos.lon) * D2R;
        const a = Math.sin(dLat / 2) ** 2 + Math.cos(lPos.lat * D2R) * Math.cos(latRaw * D2R) * Math.sin(dLon / 2) ** 2;
        const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        const dist2D = R_EARTH * c;
        const alt_delta = (kAlt || altRaw || 0) - (lPos.kAlt || 0);
        const dist3D = Math.sqrt(dist2D ** 2 + alt_delta ** 2);
        spd3D_raw = dist3D / dt_gps_raw;
    }
    
    const R_dyn = getKalmanR(accRaw);
    let isSignalLost = (accRaw > MAX_ACC);
    let spd_kalman_input, R_corr;   
    let ZUPT_DYNAMIC_THRESHOLD = 0;
    
    // 1. DÉTERMINATION DU MODE
    if (!isSignalLost) {
        ZUPT_DYNAMIC_THRESHOLD = ACCURACY_NOISE_FACTOR * accRaw / 0.05; 
        const isGpsPlausiblyStopped = (spd3D_raw < ZUPT_DYNAMIC_THRESHOLD); 
        
        if (isGpsPlausiblyStopped) { 
            spd_kalman_input = 0.0;     R_corr = R_MIN;     
            $('zupt-active').textContent = 'Oui (GPS/Dynamique)';
            $('gps-status').textContent = '✅ GPS OK - ZUPT';
        } else {
            spd_kalman_input = spd3D_raw; R_corr = R_dyn;       
            $('zupt-active').textContent = 'Non';
            $('gps-status').textContent = '✅ GPS OK';
        }
    } else {
        $('gps-status').textContent = '⚠️ GPS Perdu (DR)';
        return; 
    }

    // 2. CORRECTION DE L'ÉTAT EKF (U = Accel_corrigée)
    const kalmanResult = kFilter(spd_kalman_input, 0.001, R_corr, corrected_Accel_Mag, false); 
    
    // 3. Mise à jour de l'état global 
    const Tv_K = getVirtualTemperature(currentTempC, currentHumidity, currentPressurehPa);
    const altBaro = getBarometricAltitude(currentPressurehPa, Tv_K); 
    if (kAlt === null && altRaw !== null) kAlt = altRaw;
    const altToFilter = (altBaro !== null && altRaw === null) ? altBaro : altRaw;
    if (altToFilter !== null) kAlt = kAltFilter(altToFilter, pos.coords.altitudeAccuracy || R_MIN, dt_gps_raw); 

    lPos = { lat: latRaw, lon: lonRaw, time, kAlt, speedMS_3D: kSpd };

    updateDOMHighFrequency(dt_gps_raw, kalmanResult.Q_used, R_corr, now, pos);
}


// ====================================================================
// BLOC 3/3 : MISE À JOUR DU DOM, ASTRO & ÉVÉNEMENTS 
// ====================================================================

function $(id) { return document.getElementById(id); }
function msToHMS(ms) {
    const totalSeconds = Math.floor(ms / 1000);
    const h = Math.floor(totalSeconds / 3600);
    const m = Math.floor((totalSeconds % 3600) / 60);
    const s = totalSeconds % 60;
    return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
}

function updateDOMHighFrequency(dt, Q_used, R_kalman_input, now, pos) {
    const lat = pos?.coords?.latitude || lPos.lat || 43.284563; 
    const lon = pos?.coords?.longitude || lPos.lon || 5.358657;
    const accRaw = pos?.coords?.accuracy || MAX_ACC + 1;
    
    const sSpdAbs = Math.abs(kSpd);
    const sSpdFE = sSpdAbs < MIN_DISPLAY_SPEED_MS ? 0 : sSpdAbs;
    const sSpdKMH = sSpdFE * 3.6;
    
    const Tv_K = getVirtualTemperature(currentTempC, currentHumidity, currentPressurehPa);
    const C_S_TRUE = getSpeedOfSound(currentTempC);
    const airDensity = getAirDensity(currentPressurehPa, Tv_K);
    const frostPoint = getFrostPoint(currentTempC, currentHumidity);

    // Position & Vitesse
    $('current-lat').textContent = lat.toFixed(6);
    $('current-lon').textContent = lon.toFixed(6);
    $('altitude-gps').textContent = kAlt !== null ? `${kAlt.toFixed(2)} m` : 'N/A';
    $('stable-speed').textContent = `${sSpdFE.toFixed(3)} m/s`; 
    $('current-speed').textContent = `${sSpdKMH.toFixed(1)} km/h`;
    $('distance-total-km').textContent = `${(totalDistance / 1000).toFixed(3)} km`;
    
    // IMU & Forces
    $('accel-imu-x').textContent = imuData.x.toFixed(3);
    $('accel-imu-y').textContent = imuData.y.toFixed(3);
    $('accel-imu-z').textContent = imuData.z.toFixed(3);
    $('accel-imu-mag').textContent = corrected_Accel_Mag.toFixed(3); 
    
    // Affichage des forces simulées
    $('centrifugal-accel').textContent = centrifugalAccel.toFixed(3);
    $('coriolis-force').textContent = coriolisForce.toFixed(3); 

    $('imu-roll').textContent = attitude.roll.toFixed(1) + '°';
    $('imu-pitch').textContent = attitude.pitch.toFixed(1) + '°';
    $('imu-yaw').textContent = attitude.yaw.toFixed(1) + '°';

    // Affichage des paramètres du filtre
    $('kalman-q-noise').textContent = Q_used.toFixed(3);
    $('kalman-r-factor').textContent = R_kalman_input.toFixed(3); 
    // NOUVEAU: Affichage du Biais Estimé
    if ($('imu-bias-est')) $('imu-bias-est').textContent = imu_bias_est.toFixed(4);

    // Fréquence IMU
    $('gnss-frequency').textContent = (1 / dt).toFixed(1);
    $('update-time-dt').textContent = dt.toFixed(3);
    
    if (!$('gps-status').textContent.includes('GPS') && !$('gps-status').textContent.includes('DR')) {
        $('gps-status').textContent = 'INITIALISATION...';
    }
    
    if (pos) { 
        $('accuracy-raw').textContent = `${accRaw.toFixed(1)} m`;
        const MDS_DYNAMIC = ACCURACY_NOISE_FACTOR * accRaw / dt;
        $('mds-dynamic').textContent = MDS_DYNAMIC.toFixed(3);
    } else {
         $('mds-dynamic').textContent = 'N/A (IMU Pred)';
         $('accuracy-raw').textContent = 'N/A';
    }

    // Météo & Chimie (omises pour concision)
    const altBaro = getBarometricAltitude(currentPressurehPa, Tv_K); 
    $('altitude-baro').textContent = altBaro !== null ? `${altBaro.toFixed(2)} m` : 'N/A'; 
    $('temp-c').textContent = currentTempC.toFixed(1);
    $('pressure-hpa').textContent = currentPressurehPa.toFixed(2);
    $('humidity-perc').textContent = currentHumidity.toFixed(0);
    $('frost-point').textContent = frostPoint.toFixed(1); 
    $('speed-of-sound').textContent = C_S_TRUE.toFixed(2);
    $('air-density').textContent = airDensity.toFixed(3);
    
    $('perc-speed-light').textContent = `${(sSpdFE / 299792458 * 100).toExponential(2)} %`;
    $('perc-speed-sound').textContent = `${(sSpdFE / C_S_TRUE * 100).toFixed(2)} %`;
}


function updateAstro(lat, lon, kAlt, now) { /* ... (Code omis) */ }
function updateClockVisualization(lat, lon, kAlt, now) { /* ... (Code omis) */ }

// --- Événements et Démarrage (omises pour concision) ---
document.addEventListener('DOMContentLoaded', () => {
    
    $('gps-pause-button').addEventListener('click', () => {
        wID === null ? startGPS() : stopGPS();
    });
    
    // ... (Attitude et DeviceMotion Events) ...

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
            imuData.x = a_lin.x; imuData.y = a_lin.y; imuData.z = a_lin.z;
            NED_Accel_Mag = Math.sqrt(a_lin.x**2 + a_lin.y**2 + a_lin.z**2);
        } else if (event.accelerationIncludingGravity) {
            const { x, y, z } = event.accelerationIncludingGravity;
            const accelMag = Math.sqrt(x*x + y*y + z*z);
            NED_Accel_Mag = Math.abs(accelMag - G); 
        }
    });

    if (domID === null) {
        domID = setInterval(() => {
            const now = new Date();
            if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString('fr-FR');
            if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
            
            const currentLat = lPos.lat || 43.284563;
            const currentLon = lPos.lon || 5.358657;

            updateAstro(currentLat, currentLon, kAlt, now);
            
            if (wID === null) {
                 updateDOMHighFrequency(1.0, Q_BASE_NOISE, R_MAX, now, null);
            }

        }, DOM_SLOW_UPDATE_MS); 
    }
});
