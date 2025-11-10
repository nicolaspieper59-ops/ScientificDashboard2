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
const IMU_FREQUENCY_MS = 50; // NOUVEAU: 20 Hz pour la boucle IMU

// --- Constantes du Filtre de Kalman & ZUPT ---
const MAX_ACC = 20;       // Seuil de perte de signal GPS (m)
const R_MIN = 0.001;
const R_MAX = 100000.0;   
const Q_BASE_NOISE = 0.05;
const IMU_ACCEL_NOISE_FACTOR = 0.5; 
const ZUPT_RAW_THRESHOLD = 0.5; 
const ZUPT_ACCEL_THRESHOLD = 0.5;
const ZUPT_ACCEL_THRESHOLD_IMU = 0.05; 
const ACCURACY_NOISE_FACTOR = 3.0; // FACTEUR MDS : Déplacement minimal réel

// --- Variables d'État Globales ---
let wID = null;
let domID = null;
let imuUpdateID = null; // NOUVEAU: ID pour la boucle de mise à jour IMU
let lastIMUUpdate = 0; // NOUVEAU: Dernier temps de mise à jour IMU/Prediction
let lPos = {};
let kSpd = 0;
let kAlt = null;
let emergencyStopActive = false;
let imuAccel = 0.0; 
let totalDistance = 0; 
let kAltFilter;

// Placeholders Météo (à remplacer par API)
let currentTempC = 17.3;
let currentPressurehPa = 1015;
let currentHumidity = 72;
let lastGPSPos = null; // NOUVEAU: Stocke la dernière position GPS brute

// --- Instanciation des Filtres ---
// MODIFIÉ: Le filtre EKF est maintenant adapté au mode Prediction/Correction
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
    
    // MODIFIÉ: Ajout de isPredictionOnly pour séparer la logique
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
// ... (Fonctions getVirtualTemperature, getBarometricAltitude, etc. inchangées) ...

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
        // Le GPS continue de fonctionner à basse fréquence pour la CORRECTION.
        wID = navigator.geolocation.watchPosition(updateGPSCorrection, handleError, { enableHighAccuracy: true, maximumAge: 500, timeout: 10000 });
        $('gps-pause-button').textContent = '⏸️ PAUSE GPS';
    }
    if (imuUpdateID === null) {
        // NOUVEAU: Démarrage de la boucle IMU haute fréquence
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


// --- NOUVEAU: LOGIQUE DE PRÉDICTION IMU HAUTE FRÉQUENCE (20 Hz) ---
function updateIMUPrediction() {
    if (emergencyStopActive || wID === null) return;

    const now = new Date();
    const time = now.getTime();
    if (lastIMUUpdate === 0) lastIMUUpdate = time;
    
    const dt_imu = (time - lastIMUUpdate) / 1000;
    
    // Correction de robustesse: Le dt_imu doit être minuscule
    const dt = (dt_imu > 0 && dt_imu < 1) ? dt_imu : IMU_FREQUENCY_MS / 1000; 
    
    lastIMUUpdate = time;

    // ÉTAPE 1: Prédiction du nouvel état (sans correction)
    // On utilise R_MAX (confiance minimale) car c'est une pure prédiction
    const kalmanResult = kFilter(kSpd, dt, R_MAX, imuAccel, true); 
    
    // Mise à jour de la distance avec la vitesse prédite
    const sSpdFE = kalmanResult.speed < ZUPT_RAW_THRESHOLD * 0.5 ? 0 : kalmanResult.speed;
    if (totalDistance > 1000000) totalDistance = 0; // Guard contre les débordements
    if (sSpdFE > 0.01) totalDistance += sSpdFE * dt;
    
    // Mise à jour du DOM à haute fréquence
    updateDOMHighFrequency(dt, kalmanResult.Q_used, kalmanResult.R_used, now, lastGPSPos);
}


// --- MODIFIÉ: LOGIQUE DE CORRECTION GPS BASSE FRÉQUENCE (updateGPSCorrection) ---
function updateGPSCorrection(pos) {
    // ÉTAPE 2: La mesure GPS corrige l'état prédit par l'IMU.
    if (emergencyStopActive || wID === null) return;
    
    lastGPSPos = pos;
    const now = new Date();
    const time = now.getTime();
    const { latitude: latRaw, longitude: lonRaw, altitude: altRaw, accuracy: accRaw } = pos.coords;

    // CALCUL DE VITESSE BRUTE GPS (spd3D_raw)
    let spd3D_raw = 0;
    let dt_gps_raw = 0.05; // Utilise dt de l'IMU pour le calcul de vitesse GPS si possible
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
        ZUPT_DYNAMIC_THRESHOLD = ACCURACY_NOISE_FACTOR * accRaw / 0.05; // Utilise dt IMU
        const isGpsPlausiblyStopped = (spd3D_raw < ZUPT_DYNAMIC_THRESHOLD); 
        
        if (isGpsPlausiblyStopped) { 
            spd_kalman_input = 0.0;     R_corr = R_MIN;     
            $('zupt-active').textContent = 'Oui (GPS/Dynamique)';
        } else {
            spd_kalman_input = spd3D_raw; R_corr = R_dyn;       
            $('zupt-active').textContent = 'Non';
        }
    } else {
        // DR mode
        spd_kalman_input = kSpd; 
        R_corr = R_MAX; 
        $('zupt-active').textContent = 'Non';
    }

    // 2. CORRECTION DE L'ÉTAT EKF
    // dt est 0.001 pour la correction, car l'IMU s'est déjà chargé de la propagation temporelle
    const kalmanResult = kFilter(spd_kalman_input, 0.001, R_corr, imuAccel, false); 
    
    // 3. Mise à jour de l'état global (seulement les données GPS)
    const Tv_K = getVirtualTemperature(currentTempC, currentHumidity, currentPressurehPa);
    const altBaro = getBarometricAltitude(currentPressurehPa, Tv_K); 
    if (kAlt === null && altRaw !== null) kAlt = altRaw;
    const altToFilter = (altBaro !== null && altRaw === null) ? altBaro : altRaw;
    if (altToFilter !== null) kAlt = kAltFilter(altToFilter, pos.coords.altitudeAccuracy || R_MIN, dt_gps_raw); 

    // Mise à jour de la dernière position GPS brute
    lPos = { lat: latRaw, lon: lonRaw, time, kAlt, speedMS_3D: kSpd };

    // Mise à jour du DOM qui dépend de GPS (accuracy, MDS)
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

// NOUVEAU: Fonction de mise à jour DOM qui s'exécute à haute fréquence
function updateDOMHighFrequency(dt, Q_used, R_kalman_input, now, pos) {
    // Si pos est null, on utilise les dernières données connues (mode prediction)
    const lat = pos?.coords?.latitude || lPos.lat || 'N/A';
    const lon = pos?.coords?.longitude || lPos.lon || 'N/A';
    const accRaw = pos?.coords?.accuracy || MAX_ACC + 1;
    const sSpdFE = kSpd < ZUPT_RAW_THRESHOLD * 0.5 ? 0 : kSpd;
    const sSpdKMH = sSpdFE * 3.6;
    
    const Tv_K = getVirtualTemperature(currentTempC, currentHumidity, currentPressurehPa);
    const C_S_TRUE = getSpeedOfSound(currentTempC);
    const airDensity = getAirDensity(currentPressurehPa, Tv_K);
    const frostPoint = getFrostPoint(currentTempC, currentHumidity);

    // Position & Vitesse
    if (lat !== 'N/A') {
        $('current-lat').textContent = lat.toFixed(6);
        $('current-lon').textContent = lon.toFixed(6);
    }
    $('altitude-gps').textContent = kAlt !== null ? `${kAlt.toFixed(2)} m` : 'N/A';
    $('stable-speed').textContent = `${sSpdFE.toFixed(3)} m/s`;
    $('current-speed').textContent = `${sSpdKMH.toFixed(1)} km/h`;
    $('distance-total-km').textContent = `${(totalDistance / 1000).toFixed(3)} km`;
    
    // IMU & Kalman
    $('accel-imu').textContent = imuAccel.toFixed(3);
    $('kalman-q-noise').textContent = Q_used.toFixed(3);
    // Le facteur R est toujours mis à jour pour refléter la dernière confiance
    $('kalman-r-factor').textContent = R_kalman_input.toFixed(3); 
    
    // CORRIGÉ: dt et Fréquence reflètent maintenant la boucle IMU (20Hz)
    $('gnss-frequency').textContent = (1 / dt).toFixed(1);
    $('update-time-dt').textContent = dt.toFixed(3);
    
    $('gps-status').textContent = accRaw > MAX_ACC ? '⚠️ GPS Perdu (DR)' : '✅ GPS OK';
    
    if (pos) { // Mises à jour spécifiques au GPS
        $('accuracy-raw').textContent = `${accRaw.toFixed(1)} m`;
        const MDS_DYNAMIC = ACCURACY_NOISE_FACTOR * accRaw / dt;
        $('mds-dynamic').textContent = MDS_DYNAMIC.toFixed(3);
    } else if ($('mds-dynamic').textContent !== 'N/A (DR)') {
         $('mds-dynamic').textContent = 'N/A (IMU Pred)';
    }


    // Météo & Chimie (Mise à jour lente ou correction du Baro)
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
    // ... (Fonctions updateAstro et updateClockVisualization inchangées) ...
    const LST_h = getLocalSiderealTime(now, lon);
    const LST_hms = msToHMS((LST_h % 24) * 3600000);
    $('local-sidereal-time').textContent = LST_hms;

    updateClockVisualization(lat, lon, kAlt, now);

    if (typeof SunCalc !== 'undefined') {
        const sunTimes = SunCalc.getTimes(now, lat, lon);
        if (sunTimes.sunrise) {
            $('sunrise-tst').textContent = `↑ ${sunTimes.sunrise.toLocaleTimeString('fr-FR')} / ↓ ${sunTimes.sunset.toLocaleTimeString('fr-FR')} (Local)`;
        }
    }
}

function updateClockVisualization(lat, lon, kAlt, now) {
    if (typeof SunCalc === 'undefined') return; 
    
    const sunPos = SunCalc.getPosition(now, lat, lon);
    const sunEl = $('sun-el');
    const DISC_RADIUS_PERCENT = 50; 
    const isNorthHemisphere = lat >= 0; 
    
    const calculateRadialDistance = (altDeg) => (altDeg / 90) * DISC_RADIUS_PERCENT; 

    if (sunPos) {
        const altDeg = sunPos.altitude * R2D;
        let aziDeg = sunPos.azimuth * R2D;
        const referenceAzimut = isNorthHemisphere ? 180 : 0; 
        let deltaAzimut = aziDeg - referenceAzimut;
        if (deltaAzimut > 180) deltaAzimut -= 360;
        if (deltaAzimut < -180) deltaAzimut += 360;
        const angleProjection = Math.min(90, Math.max(-90, deltaAzimut)); 
        const radialDistance = calculateRadialDistance(altDeg); 
        const thetaRad = angleProjection * D2R; 
        
        const translateXPercent = radialDistance * Math.sin(thetaRad);
        const translateYPercent = radialDistance * Math.cos(thetaRad) * -1;

        sunEl.style.transform = `translate(${translateXPercent}%, ${translateYPercent}%)`; 

        const isUnderHorizon = altDeg < -0.83;
        sunEl.classList.toggle('under-horizon', isUnderHorizon); 
    }
}

// --- Événements et Démarrage ---
document.addEventListener('DOMContentLoaded', () => {
    // Écouteurs GPS/Pause
    $('gps-pause-button').addEventListener('click', () => {
        wID === null ? startGPS() : stopGPS();
    });
    
    // Écouteur Rayons X
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
    
    // Écouteur IMU (Accélération)
    window.addEventListener('devicemotion', (event) => {
        if (event.accelerationIncludingGravity) {
            const { x, y, z } = event.accelerationIncludingGravity;
            const accelMag = Math.sqrt(x*x + y*y + z*z);
            imuAccel = Math.abs(accelMag - G); 
        }
    });

    // Démarrage de la boucle DOM lente (Horloge/Astro)
    if (domID === null) {
        domID = setInterval(() => {
            const now = new Date();
            if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString('fr-FR');
            if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
            
            const currentLat = lPos.lat || 43.296;
            const currentLon = lPos.lon || 5.370;

            updateAstro(currentLat, currentLon, kAlt, now);
            
            if (wID === null) {
                 // Mise à jour de l'affichage à vitesse nulle si le GPS est en pause
                 updateDOMHighFrequency(1.0, Q_BASE_NOISE, R_MAX, now, null);
            }

        }, DOM_SLOW_UPDATE_MS); 
    }
});
