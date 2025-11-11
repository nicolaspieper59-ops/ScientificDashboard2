// ====================================================================
// FICHIER JS COMPLET : gnss-dashboard-full.js
// ====================================================================

// ====================================================================
// BLOC 1/3 : INITIALISATION, CONSTANTES & FILTRES SCIENTIFIQUES 
// ====================================================================

// --- Constantes de Conversion & Temps (inchangées) ---
const R2D = 180 / Math.PI;
const D2R = Math.PI / 180;
// ... (autres constantes physiques)
const DOM_SLOW_UPDATE_MS = 1000;
const IMU_FREQUENCY_MS = 50; 

// --- Constantes du Filtre de Kalman & ZUPT ---
const MAX_ACC = 20;       
const R_MIN = 0.001;
const R_MAX = 100000.0;   
// MODIFIÉ: Bruit de base réduit pour la stabilité
const Q_BASE_NOISE = 0.03; 
// MODIFIÉ: Facteur de bruit accru pour une meilleure réaction à l'accélération/décélération
const IMU_ACCEL_NOISE_FACTOR = 0.8; 
const ZUPT_RAW_THRESHOLD = 0.5; 
const ACCURACY_NOISE_FACTOR = 3.0; 
const MIN_DISPLAY_SPEED_MS = 0.0001; 
// MODIFIÉ: Seuil ZUPT IMU abaissé
const IMU_ZUPT_THRESHOLD = 0.05; 

// --- Variables d'État Globales (inchangées) ---
// ... 

// NOUVEAU: Correction pour la Magnitude d'Accélération Corrigée
let corrected_Accel_Mag = 0.0;

// ... (Le reste du BLOC 1/3, y compris createKalmanFilter, est inchangé)

function createKalmanFilter(initialQ, initialR) {
    let X = [[0], [0]]; // État: [position, vitesse]
    let P = [[initialR, 0], [0, initialR]]; // Covariance de l'erreur
    
    // U_accel_sensor est maintenant la magnitude CORRIGÉE (corrected_Accel_Mag)
    return function(Z, dt, R_new, U_accel_sensor, isPredictionOnly = false) { 
        // 1. DÉFINITION DYNAMIQUE DE Q 
        const Q_dt_factor = Math.min(1.0, 1.0 / (dt * 10)); 
        // Q_noise utilise la MAGNITUDE de l'accélération CORRIGÉE pour définir le bruit
        const Q_noise = Q_BASE_NOISE * Q_dt_factor + (Math.abs(U_accel_sensor) * IMU_ACCEL_NOISE_FACTOR); 
        const Q_matrix = [[0.001, 0], [0, Q_noise * Q_noise]]; 
        
        // ... (Reste de la prédiction et mise à jour inchangée)
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

// ... (Fonctions utilitaires et GPS inchangées) ...

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

    // 2. CORRECTION DE LA MAGNITUDE IMU (Inchangée par rapport à l'étape précédente)
    corrected_Accel_Mag = NED_Accel_Mag;
    
    if (corrected_Accel_Mag >= centrifugalAccel) {
        corrected_Accel_Mag -= centrifugalAccel;
    } else {
        corrected_Accel_Mag = Q_BASE_NOISE; 
    }

    // 3. LOGIQUE ZUPT et PRÉDICTION
    let kalmanResult;
    let R_corr = R_MAX; 

    if (wID === null) { 
        // Mode DR (GPS Pausé) - Utilisation du ZUPT basé sur la MAGNITUDE CORRIGÉE
        
        if (corrected_Accel_Mag < IMU_ZUPT_THRESHOLD) {
            // ZUPT Actif : Correction forcée à Vitesse = 0 avec haute confiance
            kalmanResult = kFilter(0.0, dt, R_MIN, corrected_Accel_Mag, false);
            R_corr = R_MIN;
            $('gps-status').textContent = '✅ DR - ZUPT ON (IMU)';
            $('zupt-active').textContent = 'Oui (IMU/DR)';
        } else {
            // Mouvement détecté : Prediction pure EKF (U = Accel_corrigée)
            kalmanResult = kFilter(kSpd, dt, R_MAX, corrected_Accel_Mag, true);
            $('gps-status').textContent = '⚠️ DR - PRÉDICTION IMU';
            $('zupt-active').textContent = 'Non';
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
        // Signal GPS OK : Correction du filtre
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
        // Signal perdu. On sort, la boucle IMU gère le DR/ZUPT IMU
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

// ... (Fonctions du DOM et Listeners inchangés) ...

function updateDOMHighFrequency(dt, Q_used, R_kalman_input, now, pos) {
    // ... (Début de fonction inchangé) ...
    
    const sSpdAbs = Math.abs(kSpd);
    const sSpdFE = sSpdAbs < MIN_DISPLAY_SPEED_MS ? 0 : sSpdAbs;
    const sSpdKMH = sSpdFE * 3.6;
    
    // ... (Calculs Météo/Chimie inchangés) ...

    // IMU & Forces (Accélération 3D & Attitude)
    $('accel-imu-x').textContent = imuData.x.toFixed(3);
    $('accel-imu-y').textContent = imuData.y.toFixed(3);
    $('accel-imu-z').textContent = imuData.z.toFixed(3);
    
    // Affiche l'accélération Corrigée pour l'EKF
    $('accel-imu-mag').textContent = corrected_Accel_Mag.toFixed(3); 
    
    // ... (Affichage des forces et autres data points inchangés) ...
    $('kalman-q-noise').textContent = Q_used.toFixed(3);
    $('kalman-r-factor').textContent = R_kalman_input.toFixed(3); 
    // ... (Fin de fonction inchangée) ...
}

// ... (Fin du code) ...
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
            
