// ====================================================================
// FICHIER JS COMPLET : gnss-dashboard-full.js
// ====================================================================

// ====================================================================
// BLOC 1/3 : INITIALISATION, CONSTANTES & FILTRES SCIENTIFIQUES (~80 Lignes)
// ====================================================================

// --- Constantes de Conversion & Temps ---
const R2D = 180 / Math.PI;
const D2R = Math.PI / 180;
const G = 9.80665; // Accélération gravitationnelle standard (m/s²)
const R_AIR = 287.05;
const GAMMA_AIR = 1.4;
const P_SEA_hPa = 1013.25;
const DOM_SLOW_UPDATE_MS = 1000;
const IMU_FREQUENCY_MS = 50; // 20 Hz pour la boucle IMU

// --- Constantes du Filtre de Kalman & ZUPT ---
const MAX_ACC = 20;       // Seuil de perte de signal GPS (m)
const R_MIN = 0.001;
const R_MAX = 100000.0;   
const Q_BASE_NOISE = 0.05;
const IMU_ACCEL_NOISE_FACTOR = 0.5; 
const ZUPT_RAW_THRESHOLD = 0.5; 
const ACCURACY_NOISE_FACTOR = 3.0; // FACTEUR MDS : Déplacement minimal réel

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

// NOUVEAU: Données IMU 3D et Attitude
let imuData = { x: 0.0, y: 0.0, z: 0.0 }; // Accélération linéaire X, Y, Z (m/s²)
let attitude = { roll: 0.0, pitch: 0.0, yaw: 0.0 }; // Roulis, Tangage, Cap (Degrés)
let NED_Accel_Mag = 0.0; // Magnitude de l'accélération linéaire pour l'EKF 1D

// Placeholders Météo (à remplacer par API)
let currentTempC = 17.3;
let currentPressurehPa = 1015;
let currentHumidity = 72;

// --- Instanciation des Filtres ---
let kFilter = createKalmanFilter(Q_BASE_NOISE, R_MAX); 
kAltFilter = createSimpleKalmanFilter(0.1, R_MIN);

// --- Fonctions de Filtrage ---
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
    let X = [[0], [0]]; // État: [position, vitesse]
    let P = [[initialR, 0], [0, initialR]]; // Covariance de l'erreur
    
    return function(Z, dt, R_new, U_accel_sensor, isPredictionOnly = false) { 
        // 1. DÉFINITION DYNAMIQUE DE Q 
        const Q_dt_factor = Math.min(1.0, 1.0 / (dt * 10)); 
        // U_accel_sensor est la magnitude 3D de l'accélération
        const Q_noise = Q_BASE_NOISE * Q_dt_factor + (Math.abs(U_accel_sensor) * IMU_ACCEL_NOISE_FACTOR); 
        const Q_matrix = [[0.001, 0], [0, Q_noise * Q_noise]]; 
        
        // Prédiction (X_k = F * X_{k-1} + B * U)
        const F = [[1, dt], [0, 1]];
        const B = [[0.5 * dt * dt], [dt]]; 
        const U = [[U_accel_sensor]]; 
        
        // On utilise la magnitude 3D de l'accélération pour prédire la VITESSE 1D
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

        // X = X_pred + K * Y; P = (I - K * H) * P_pred
        X[0][0] = X[0][0] + K[0][0] * Y; X[1][0] = X[1][0] + K[1][0] * Y;
        P[0][0] = P[0][0] - K[0][0] * P[1][0]; P[0][1] = P[0][1] - K[0][0] * P[1][1];
        P[1][0] = P[1][0] - K[1][0] * P[1][0]; P[1][1] = P[1][1] - K[1][0] * P[1][1];

        kSpd = X[1][0];
        return { speed: X[1][0], Q_used: Q_noise, R_used: R_new };
    }
}


// ====================================================================
// BLOC 2/3 : GESTION DES CAPTEURS, MÉTÉO & LOGIQUE DE FUSION (~80 Lignes)
// ====================================================================

// --- Fonctions Météo et GPS ---
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
    if (emergencyStopActive || wID === null) return;

    const now = new Date();
    const time = now.getTime();
    if (lastIMUUpdate === 0) lastIMUUpdate = time;
    
    const dt_imu = (time - lastIMUUpdate) / 1000;
    const dt = (dt_imu > 0 && dt_imu < 1) ? dt_imu : IMU_FREQUENCY_MS / 1000; 
    
    lastIMUUpdate = time;

    // ÉTAPE 1: Prédiction du nouvel état (sans correction)
    // U_accel_sensor est maintenant NED_Accel_Mag, la magnitude 3D de l'accélération linéaire.
    const kalmanResult = kFilter(kSpd, dt, R_MAX, NED_Accel_Mag, true); 
    
    // Mise à jour de la distance avec la vitesse prédite
    const sSpdFE = kalmanResult.speed < ZUPT_RAW_THRESHOLD * 0.5 ? 0 : kalmanResult.speed;
    if (totalDistance > 1000000) totalDistance = 0; 
    if (sSpdFE > 0.01) totalDistance += sSpdFE * dt;
    
    // Mise à jour du DOM à haute fréquence
    updateDOMHighFrequency(dt, kalmanResult.Q_used, kalmanResult.R_used, now, lastGPSPos);
}


// --- LOGIQUE DE CORRECTION GPS BASSE FRÉQUENCE (updateGPSCorrection) ---
function updateGPSCorrection(pos) {
    if (emergencyStopActive || wID === null) return;
    
    lastGPSPos = pos;
    const now = new Date();
    const time = now.getTime();
    const { latitude: latRaw, longitude: lonRaw, altitude: altRaw, accuracy: accRaw } = pos.coords;

    // CALCUL DE VITESSE BRUTE GPS
    let spd3D_raw = 0;
    let dt_gps_raw = 0.05; 
    if (lPos.time && time > lPos.time) {
        dt_gps_raw = (time - lPos.time) / 1000;
        const R = 6371e3;
        const dLat = (latRaw - lPos.lat) * D2R; const dLon = (lonRaw - lPos.lon) * D2R;
        const a = Math.sin(dLat / 2) ** 2 + Math.cos(lPos.lat * D2R) * Math.cos(latRaw * D2R) * Math.sin(dLon / 2) ** 2;
        const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        const dist2D = R * c;
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
        } else {
            spd_kalman_input = spd3D_raw; R_corr = R_dyn;       
            $('zupt-active').textContent = 'Non';
        }
    } else {
        spd_kalman_input = kSpd; 
        R_corr = R_MAX; 
        $('zupt-active').textContent = 'Non';
    }

    // 2. CORRECTION DE L'ÉTAT EKF
    // Utilise NED_Accel_Mag comme input U pour la correction également
    const kalmanResult = kFilter(spd_kalman_input, 0.001, R_corr, NED_Accel_Mag, false); 
    
    // 3. Mise à jour de l'état global (seulement les données GPS)
    const Tv_K = getVirtualTemperature(currentTempC, currentHumidity, currentPressurehPa);
    const altBaro = getBarometricAltitude(currentPressurehPa, Tv_K); 
    if (kAlt === null && altRaw !== null) kAlt = altRaw;
    const altToFilter = (altBaro !== null && altRaw === null) ? altBaro : altRaw;
    if (altToFilter !== null) kAlt = kAltFilter(altToFilter, pos.coords.altitudeAccuracy || R_MIN, dt_gps_raw); 

    lPos = { lat: latRaw, lon: lonRaw, time, kAlt, speedMS_3D: kSpd };

    updateDOMHighFrequency(dt_gps_raw, kalmanResult.Q_used, R_corr, now, pos);
}


// ====================================================================
// BLOC 3/3 : MISE À JOUR DU DOM, ASTRO & ÉVÉNEMENTS (~80 Lignes)
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
    const lat = pos?.coords?.latitude || lPos.lat || 43.284563; // Utilisation des valeurs initiales si non disponibles
    const lon = pos?.coords?.longitude || lPos.lon || 5.358657;
    const accRaw = pos?.coords?.accuracy || MAX_ACC + 1;
    const sSpdFE = kSpd < ZUPT_RAW_THRESHOLD * 0.5 ? 0 : kSpd;
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
    
    // IMU & Forces (Accélération 3D & Attitude)
    $('accel-imu-x').textContent = imuData.x.toFixed(3);
    $('accel-imu-y').textContent = imuData.y.toFixed(3);
    $('accel-imu-z').textContent = imuData.z.toFixed(3);
    $('accel-imu-mag').textContent = NED_Accel_Mag.toFixed(3);
    
    $('imu-roll').textContent = attitude.roll.toFixed(1) + '°';
    $('imu-pitch').textContent = attitude.pitch.toFixed(1) + '°';
    $('imu-yaw').textContent = attitude.yaw.toFixed(1) + '°';

    $('kalman-q-noise').textContent = Q_used.toFixed(3);
    $('kalman-r-factor').textContent = R_kalman_input.toFixed(3); 
    
    // Fréquence IMU
    $('gnss-frequency').textContent = (1 / dt).toFixed(1);
    $('update-time-dt').textContent = dt.toFixed(3);
    
    $('gps-status').textContent = accRaw > MAX_ACC ? '⚠️ GPS Perdu (DR)' : '✅ GPS OK';
    
    if (pos) { 
        $('accuracy-raw').textContent = `${accRaw.toFixed(1)} m`;
        const MDS_DYNAMIC = ACCURACY_NOISE_FACTOR * accRaw / dt;
        $('mds-dynamic').textContent = MDS_DYNAMIC.toFixed(3);
    } else if ($('mds-dynamic').textContent !== 'N/A (IMU Pred)') {
         $('mds-dynamic').textContent = 'N/A (IMU Pred)';
    }

    // Météo & Chimie
    const altBaro = getBarometricAltitude(currentPressurehPa, Tv_K); 
    $('altitude-baro').textContent = altBaro !== null ? `${altBaro.toFixed(2)} m` : 'N/A'; 
    $('temp-c').textContent = currentTempC.toFixed(1);
    $('pressure-hpa').textContent = currentPressurehPa.toFixed(2);
    $('humidity-perc').textContent = currentHumidity.toFixed(0);
    $('frost-point').textContent = frostPoint.toFixed(1); 
    $('speed-of-sound').textContent = C_S_TRUE.toFixed(2);
    $('air-density').textContent = airDensity.toFixed(3);
    
    // Rapports de Vitesse
    $('perc-speed-light').textContent = `${(sSpdFE / 299792458 * 100).toExponential(2)} %`;
    $('perc-speed-sound').textContent = `${(sSpdFE / C_S_TRUE * 100).toFixed(2)} %`;
}


function updateAstro(lat, lon, kAlt, now) {
    const LST_h = getLocalSiderealTime(now, lon);
    const LST_hms = msToHMS((LST_h % 24) * 3600000);
    $('local-sidereal-time').textContent = LST_hms;

    updateClockVisualization(lat, lon, kAlt, now);
    // (SunCalc integration omise pour concision)
}

function updateClockVisualization(lat, lon, kAlt, now) {
    // (Implémentation omise pour concision)
}

// --- Événements et Démarrage ---
document.addEventListener('DOMContentLoaded', () => {
    
    // Écouteur GPS/Pause
    $('gps-pause-button').addEventListener('click', () => {
        wID === null ? startGPS() : stopGPS();
    });
    
    // Écouteur Rayons X (omise pour concision)
    const toggleXrayButton = $('toggle-xray');
    const clockContainer = $('clock-container');
    if (toggleXrayButton && clockContainer) {
        toggleXrayButton.addEventListener('click', () => {
            const isXray = clockContainer.classList.toggle('xray-mode');
            toggleXrayButton.innerHTML = isXray ? 
                '<i class="fas fa-eye"></i> Astro Rayons X: ON' : 
                '<i class="fas fa-eye-slash"></i> Astro Rayons X: OFF';
        });
    }

    // NOUVEAU: Écouteur d'Attitude (Roll, Pitch, Yaw)
    if (window.DeviceOrientationEvent) {
        window.addEventListener('deviceorientation', (event) => {
            // alpha (Yaw/Cap): rotation autour de Z, beta (Pitch/Tangage): rotation autour de X, gamma (Roll/Roulis): rotation autour de Y
            attitude.yaw = event.alpha || 0; 
            attitude.pitch = event.beta || 0; 
            attitude.roll = event.gamma || 0;
        });
    }

    // MODIFIÉ: Écouteur IMU (Accélération Linéaire 3D)
    window.addEventListener('devicemotion', (event) => {
        // La propriété 'acceleration' fournit l'accélération linéaire (sans la gravité) dans le référentiel du dispositif. C'est l'idéal.
        const a_lin = event.acceleration; 
        
        if (a_lin) {
            imuData.x = a_lin.x;
            imuData.y = a_lin.y;
            imuData.z = a_lin.z;
            
            // Calcul de la magnitude 3D pour l'entrée de l'EKF 1D de vitesse
            NED_Accel_Mag = Math.sqrt(a_lin.x**2 + a_lin.y**2 + a_lin.z**2);
        } else if (event.accelerationIncludingGravity) {
            // Fallback (Moins précis): Si acceleration (linéaire) n'est pas disponible, 
            // on utilise l'approximation de la magnitude brute moins la gravité.
            const { x, y, z } = event.accelerationIncludingGravity;
            const accelMag = Math.sqrt(x*x + y*y + z*z);
            NED_Accel_Mag = Math.abs(accelMag - G); 
        }
    });

    // Démarrage de la boucle DOM lente (Horloge/Astro)
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
