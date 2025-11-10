// ====================================================================
// BLOC 1/3 : INITIALISATION, CONSTANTES & FONCTIONS SCIENTIFIQUES
// ====================================================================

// --- Constantes de Conversion & Temps ---
const R2D = 180 / Math.PI; // Radians to Degrees
const D2R = Math.PI / 180; // Degrees to Radians
const dayMs = 86400000;
const DOM_SLOW_UPDATE_MS = 1000; // Mise à jour lente du DOM (1s)

// --- Constantes de Physique/Météo ---
const C_S = 343;          // Vitesse du son (standard 20°C, 0% HR)
const C_LIGHT = 299792458; // Vitesse de la lumière (m/s)
const C_WATER = 461.5;    // Constante spécifique des gaz pour la vapeur d'eau
const R_AIR = 287.05;     // Constante spécifique des gaz pour l'air sec (J/kg·K)
const GAMMA_AIR = 1.4;    // Rapport des capacités thermiques pour l'air
const G = 9.80665;        // Accélération gravitationnelle standard (m/s²)
const P_SEA_hPa = 1013.25; // Pression atmosphérique standard au niveau de la mer

// --- Constantes du Filtre de Kalman & ZUPT ---
const MIN_ACC = 3;        // Précision minimale acceptable du GPS (m)
const MAX_ACC = 20;       // Seuil de perte de signal GPS (m)
const R_MIN = 0.001;      // Incertitude minimale (ZUPT ON)
const R_MAX = 500.0;      // Incertitude maximale (Signal faible)
const Q_NOISE = 0.1;      // Bruit du processus (modèle cinématique)
const R_ALT_MIN = 1.0;    // Incertitude minimale pour le filtre d'altitude
const ZUPT_RAW_THRESHOLD = 0.8; // Vitesse brute max pour ZUPT (m/s)
const ZUPT_ACCEL_THRESHOLD = 0.5; // Accélération max pour ZUPT (m/s²)
const ZUPT_ACCEL_THRESHOLD_IMU = 0.1; // Accélération max pour ZUPT en mode IMU Pur (m/s²)

// --- Variables d'État Globales ---
let wID = null;             // Watch Position ID (GPS)
let domID = null;           // Set Interval ID (DOM update)
let lastPosTime = 0;
let lastUpdate = 0;
let lPos = {};              // Dernier objet de position traité
let kSpd = 0;               // Vitesse estimée par Kalman
let kAlt = null;            // Altitude estimée par Kalman
let kAltFilter;             // Instance du filtre Kalman pour l'altitude
let lastP_hPa = null;       // Dernière pression atmosphérique pour les calculs
let emergencyStopActive = false;
let IMU_ONLY_MODE_FLAG = false; // Flag pour mode Dead Reckoning Pur
let currentTempC = 17.3;    // Placeholder Météo (à remplacer par une vraie API)
let currentPressurehPa = 1015;
let currentHumidity = 72;
let imuAccel = 0.0;         // Placeholder Accélération IMU (à mettre à jour par DeviceMotion)

// --- Instance de Filtres de Kalman (1D) ---
// [vitesse] = [[0], [0]]
let kFilter = createKalmanFilter(0.1, 100); 

function createKalmanFilter(Q = Q_NOISE, R = R_MAX) {
    let X = [[0], [0]]; // État: [position, vitesse] (en 1D pour la vitesse)
    let P = [[R, 0], [0, R]]; // Covariance de l'erreur

    return function(Z, dt, R_new, U_accel_sensor) {
        // Prédiction
        // F (Matrice de transition d'état)
        const F = [[1, dt], [0, 1]];
        // B (Matrice de contrôle)
        const B = [[0.5 * dt * dt], [dt]]; 
        // U (Contrôle/Entrée, accélération IMU)
        const U = [[U_accel_sensor]]; 
        
        // Nouvelle estimation de l'état (X_k = F * X_{k-1} + B * U)
        X[0][0] = F[0][0] * X[0][0] + F[0][1] * X[1][0] + B[0][0] * U[0][0];
        X[1][0] = F[1][0] * X[0][0] + F[1][1] * X[1][0] + B[1][0] * U[0][0];

        // Nouvelle covariance prédite (P_k = F * P_{k-1} * F^T + Q)
        const Q_matrix = [[0.001, 0], [0, Q * Q]]; 
        let P_pred = [[0, 0], [0, 0]];
        // F * P_{k-1} * F^T
        // P[0][0] = F[0][0]*(F[0][0]*P[0][0] + F[0][1]*P[1][0]) + F[0][1]*(F[0][0]*P[0][1] + F[0][1]*P[1][1]);
        P_pred[0][0] = P[0][0] + dt * (P[0][1] + P[1][0]) + dt * dt * P[1][1];
        P_pred[0][1] = P[0][1] + dt * P[1][1];
        P_pred[1][0] = P[1][0] + dt * P[1][1];
        P_pred[1][1] = P[1][1];
        // Ajout de Q (bruit du processus)
        P_pred[0][0] += Q_matrix[0][0];
        P_pred[1][1] += Q_matrix[1][1];
        P = P_pred;

        // Mise à jour de la Mesure
        const R_new_matrix = [[R_new, 0], [0, 1000]]; // Utiliser R_new pour la vitesse
        const H = [[0, 1]]; // La mesure (Z) est la vitesse (X[1])

        // Y (Innovation) = Z - H * X_pred
        const Y = Z - X[1][0]; 

        // S (Covariance de l'innovation) = H * P_pred * H^T + R
        const S = P[1][1] + R_new_matrix[1][1]; 

        // K (Gain de Kalman) = P_pred * H^T * S^-1
        const K = [[P[0][1] / S], [P[1][1] / S]]; 

        // Nouvel état estimé (X = X_pred + K * Y)
        X[0][0] = X[0][0] + K[0][0] * Y;
        X[1][0] = X[1][0] + K[1][0] * Y;

        // Nouvelle Covariance (P = (I - K * H) * P_pred)
        P[0][0] = P[0][0] - K[0][0] * P[1][0];
        P[0][1] = P[0][1] - K[0][0] * P[1][1];
        P[1][0] = P[1][0] - K[1][0] * P[1][0];
        P[1][1] = P[1][1] - K[1][0] * P[1][1];

        kSpd = X[1][0]; // Mise à jour de la vitesse globale
        return X[1][0]; // Vitesse filtrée (m/s)
    }
}

// Version simplifiée du filtre pour l'altitude (ne nécessite pas l'accélération)
function createSimpleKalmanFilter(Q, R) {
    let X = 0; // État
    let P = R; // Covariance

    return function(Z, R_new, dt) {
        // Prédiction (l'altitude ne bouge pas sans force)
        // X_pred = X (Pas de modèle de mouvement, on suppose l'état stable)
        // P_pred = P + Q
        let P_pred = P + Q * dt;

        // Mise à jour (Mesure)
        // K = P_pred / (P_pred + R_new)
        let K = P_pred / (P_pred + R_new);

        // X = X_pred + K * (Z - X_pred)
        X = X + K * (Z - X);

        // P = (1 - K) * P_pred
        P = (1 - K) * P_pred;
        
        return X;
    }
}

// Initialisation des filtres d'altitude
kAltFilter = createSimpleKalmanFilter(0.1, R_ALT_MIN);

/** Calcule l'incertitude R pour le filtre Kalman basé sur la précision GPS (accRaw). */
function getKalmanR(accRaw, alt, P_hPa) {
    if (accRaw === null || accRaw > MAX_ACC) return R_MAX;
    return Math.max(R_MIN, accRaw * accRaw); // R = σ²
}

// --- Fonctions Météo et Barométrie (avec correction d'humidité/température) ---

/** Calcule la Température Virtuelle (en K) pour les calculs d'altitude. */
function getVirtualTemperature(T_C, HR_percent, P_hPa) {
    if (!P_hPa || !T_C || !HR_percent) return T_C + 273.15; // Fallback à T air
    const T_K = T_C + 273.15;
    const HR = HR_percent / 100;
    const Pvs_Pa = 611 * Math.exp((17.27 * T_C) / (237.3 + T_C));
    const Pv_Pa = HR * Pvs_Pa;
    const r = (0.622 * Pv_Pa) / (P_hPa * 100 - Pv_Pa);
    return T_K * (1 + 0.61 * r);
}

/** Calcule l'Altitude Barométrique (en mètres) par la formule barométrique corrigée. */
function getBarometricAltitude(P_hPa, Tv_K) {
    if (P_hPa === null || isNaN(P_hPa)) return null;
    const exponent = (GAMMA_AIR - 1) / GAMMA_AIR;
    // Utilisation de la Température Virtuelle (Tv_K)
    const alt = ((R_AIR * Tv_K) / (G * exponent)) * (1 - Math.pow(P_hPa / P_SEA_hPa, exponent));
    return alt;
}

/** Calcule la Vitesse du Son (m/s) en fonction de la Température (Celsius). */
function getSpeedOfSound(T_C) {
    return 331.3 * Math.sqrt(1 + T_C / 273.15); 
}

/** Facteur de correction R pour GPS (influence atmosphérique). */
function getAtmosphericCorrectionFactor(P_hPa, T_C) {
    if (P_hPa === null || T_C === null || isNaN(P_hPa) || isNaN(T_C)) return 1.0; 
    const DENSITY_SEA_LEVEL = 1.225; // kg/m³
    const T_K = T_C + 273.15;
    const density = (P_hPa * 100) / (R_AIR * T_K); 
    // Si l'air est plus dense, l'incertitude est plus grande (facteur > 1)
    return Math.max(1.0, density / DENSITY_SEA_LEVEL);
}

// --- Fonctions Astro Avancées ---

/** Calcule l'Heure Sidérale Locale (LST) en heures décimales. */
function getLocalSiderealTime(date, longitudeDeg) {
    const JD_2000 = 2451545.0; 
    const timeMs = date.getTime();
    const JD = (timeMs / dayMs) + 2440587.5; 
    const D = JD - JD_2000;
    let GMST = 24110.54841 + 8640184.812866 * D + 0.093104 * D * D;
    GMST = (GMST / 3600.0) % 24; 
    if (GMST < 0) GMST += 24;
    let LST = GMST + (longitudeDeg / 15.0);
    LST = LST % 24;
    if (LST < 0) LST += 24;
    return LST;
        }
// ====================================================================
// BLOC 2/3 : GESTION DES CAPTEURS & LOGIQUE DE FUSION (updateDisp)
// ====================================================================

function startGPS() {
    if (wID === null) {
        wID = navigator.geolocation.watchPosition(updateDisp, handleError, {
            enableHighAccuracy: true,
            maximumAge: 500,
            timeout: 10000
        });
        $('gps-pause-button').textContent = '⏸️ PAUSE GPS';
    }
}

function stopGPS() {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
        $('gps-pause-button').textContent = '▶️ MARCHE GPS';
    }
}

function handleError(error) {
    console.warn(`GPS ERROR: ${error.code}: ${error.message}`);
    // Potentiellement basculer en mode IMU_ONLY_MODE_FLAG ici
}

function updateDisp(pos) {
    if (emergencyStopActive || wID === null) return;
    
    const now = new Date();
    const time = now.getTime();
    const dt = (time - lastUpdate) / 1000 || 0.001; // Delta temps (secondes)
    lastUpdate = time;

    // --- 1. CAPTEURS & ALTITUDE ---
    const latRaw = pos.coords.latitude;
    const lonRaw = pos.coords.longitude;
    const altRaw = pos.coords.altitude;
    const accRaw = pos.coords.accuracy;
    const lastP_hPa = currentPressurehPa;

    // Calcul de la Température Virtuelle (Tv) pour l'altimétrie précise
    const Tv_K = getVirtualTemperature(currentTempC, currentHumidity, currentPressurehPa);
    const altBaro = getBarometricAltitude(currentPressurehPa, Tv_K);

    // Initialisation de kAlt si nécessaire
    if (kAlt === null && altRaw !== null) kAlt = altRaw;

    // --- 2. SIGNAL GPS & CORRECTION R DYNAMIQUE ---
    const atmosphericFactor = getAtmosphericCorrectionFactor(currentPressurehPa, currentTempC);
    const R_dyn = getKalmanR(accRaw, kAlt, lastP_hPa) * atmosphericFactor;
    let isSignalLost = (accRaw > MAX_ACC);

    // FILTRAGE DE L'ALTITUDE (Fusion Baro/GPS)
    const altToFilter = (altBaro !== null && altRaw === null) ? altBaro : altRaw;
    if (altToFilter !== null) {
        kAlt = kAltFilter(altToFilter, pos.coords.altitudeAccuracy || R_ALT_MIN, dt); 
    }

    // --- 3. CALCUL VITESSE BRUTE & DISTANCE ---
    let lat = latRaw;
    let lon = lonRaw;
    
    // Logique de Dead Reckoning (DR) / Position Figée
    if (isSignalLost && lPos.lat) {
        // En cas de perte de signal, la position est figée à la dernière bonne
        lat = lPos.lat;
        lon = lPos.lon;
    }

    let dist2D = 0;
    let dist3D = 0;
    let spd3D_raw = 0;

    if (lPos.time && time > lPos.time) {
        const R = 6371e3; // Rayon de la Terre (m)
        const dLat = (lat - lPos.lat) * D2R;
        const dLon = (lon - lPos.lon) * D2R;
        const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
                  Math.cos(lPos.lat * D2R) * Math.cos(lat * D2R) *
                  Math.sin(dLon / 2) * Math.sin(dLon / 2);
        const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        dist2D = R * c; // Distance horizontale
        
        const kAlt_new = kAlt || altRaw || 0;
        const alt_delta = kAlt_new - (lPos.kAlt || 0);
        dist3D = Math.sqrt(dist2D ** 2 + alt_delta ** 2);
        spd3D_raw = dist3D / dt;
    }
    
    // --- 4. LOGIQUE ZUPT (Fusion/DR) ---
    let spd_kalman_input; 
    let R_kalman_input;   
    let modeStatus = '';
    
    // Le filtre utilise l'accélération IMU pour la prédiction (même si c'est 0.0)
    let accel_sensor_input = imuAccel; 

    if (isSignalLost) {
        // --- MODE DEAD RECKONING AIDE PAR IMU (GPS PERDU) ---
        const isImuStopped = imuAccel < ZUPT_ACCEL_THRESHOLD_IMU; 
        
        if (isImuStopped) {
            // ZUPT IMU : Forcer la vitesse à 0 m/s pour empêcher la dérive
            spd_kalman_input = 0.0;     
            R_kalman_input = R_MIN;     
            modeStatus = '✅ ZUPT IMU (Arrêt Forcé - DR)';
        } else {
            // Micro-mouvement réel : on se fie à l'intégration IMU (Prédiction)
            spd_kalman_input = kSpd;        // Maintien de la dernière vitesse estimée
            R_kalman_input = 100000;        // R très élevé pour se fier à la Prédiction (accel_sensor_input)
            modeStatus = `⚠️ DR: ESTIMATION SUR CAPTEURS (${accRaw.toFixed(0)}m) ↑`;
        }
    } else {
        // --- MODE FUSION GPS/IMU (GPS OK) ---
        const accel_long_provisional = (spd3D_raw - (lPos?.speedMS_3D || 0)) / dt;
        
        const isGpsPlausiblyStopped = (
            spd3D_raw < ZUPT_RAW_THRESHOLD && 
            Math.abs(accel_long_provisional) < ZUPT_ACCEL_THRESHOLD &&
            R_dyn < R_MAX 
        ); 
        
        if (isGpsPlausiblyStopped) { 
            spd_kalman_input = 0.0;     
            R_kalman_input = R_MIN;     
            modeStatus = '✅ ZUPT (Vélocité Nulle Forcée)';
        } else {
            spd_kalman_input = spd3D_raw; 
            R_kalman_input = R_dyn;       
            modeStatus = `🚀 FUSION TOTALE (${R_dyn.toFixed(1)} R)`;
        }
    }

    // --- 5. FILTRE DE KALMAN FINAL ---
    const fSpd = kFilter(spd_kalman_input, dt, R_kalman_input, accel_sensor_input); 
    const sSpdFE = fSpd < ZUPT_RAW_THRESHOLD * 0.5 ? 0 : fSpd; // Post-traitement anti-drift

    // Mise à jour de l'état global
    lPos = { lat, lon, time, kAlt, speedMS_3D: fSpd };

    // --- 6. MISE À JOUR DU DOM (Fonctions du BLOC 3) ---
    updateSpeedAndDistance(sSpdFE, dist3D, dt);
    updateAstro(lat, lon, kAlt, now);
    $('gps-status').textContent = modeStatus; 
    $('current-lat').textContent = lat.toFixed(6);
    $('current-lon').textContent = lon.toFixed(6);
    $('altitude-gps').textContent = kAlt !== null ? `${kAlt.toFixed(2)} m (vs Niveau de la Mer)` : 'N/A';
    $('altitude-baro').textContent = altBaro !== null ? `${altBaro.toFixed(2)} m` : 'N/A';
        }
// ====================================================================
// BLOC 3/3 : MISE À JOUR DU DOM & VISUALISATION
// ====================================================================

function $(id) { return document.getElementById(id); }

function msToHMS(ms) {
    const totalSeconds = Math.floor(ms / 1000);
    const h = Math.floor(totalSeconds / 3600);
    const m = Math.floor((totalSeconds % 3600) / 60);
    const s = totalSeconds % 60;
    return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
}

// Fonction placeholder (doit être implémentée avec l'API DeviceMotion)
function getImuAccelerationMagnitude() {
    // Retourne l'accélération linéaire totale (sans la gravité)
    return imuAccel; // Place d'un écouteur sur window.devicemotion
}

// --- Mise à jour de la Vitesse & Distance ---
function updateSpeedAndDistance(sSpdFE, dist3D, dt) {
    const sSpdKMH = sSpdFE * 3.6;
    const sSpdKMH_fixed = sSpdKMH.toFixed(1);
    const C_S_TRUE = getSpeedOfSound(currentTempC);

    $('current-speed').textContent = `${sSpdKMH_fixed} km/h`;
    $('stable-speed').textContent = `${sSpdFE.toFixed(3)} m/s`;
    
    // Mise à jour des grandeurs scientifiques
    $('perc-speed-light').textContent = `${(sSpdFE / C_LIGHT * 100).toExponential(2)} %`;
    $('perc-speed-sound').textContent = `${(sSpdFE / C_S_TRUE * 100).toFixed(2)} %`;
    $('speed-of-sound').textContent = `${C_S_TRUE.toFixed(2)} m/s`;
    // ... (autres mises à jour de vitesse/distance) ...
}


// --- Visualisation Astro (Modèle Minecraft) ---
function updateClockVisualization(lat, lon, altDeg, now) {
    const sunPos = SunCalc.getPosition(now, lat, lon);
    const moonPos = SunCalc.getMoonPosition(now, lat, lon);
    const sunTimes = SunCalc.getTimes(now, lat, lon);
    const sunEl = $('sun-el');
    const moonEl = $('moon-el');
    const MAX_RADIAL_PERCENT = 50; 
    const isNorthHemisphere = lat >= 0; 
    
    // Calcul de la distance radiale (Altitude 0° -> 0% ; Altitude 90° -> 50%)
    const calculateRadialDistance = (altDeg) => {
        const radialRatio = Math.min(1, Math.max(0, altDeg / 90));
        return radialRatio * MAX_RADIAL_PERCENT; 
    };

    // --- Position du Soleil ---
    if (sunPos) {
        const altDeg = sunPos.altitude * R2D;
        let aziDeg = sunPos.azimuth * R2D;
        
        // Référence Azimut : Sud (180°) au Nord (0°)
        const referenceAzimut = isNorthHemisphere ? 180 : 0; 
        let deltaAzimut = aziDeg - referenceAzimut;
        if (deltaAzimut > 180) deltaAzimut -= 360;
        if (deltaAzimut < -180) deltaAzimut += 360;
        
        const angleProjection = Math.min(90, Math.max(-90, deltaAzimut)); 
        
        const radialDistance = calculateRadialDistance(altDeg);
        const thetaRad = angleProjection * D2R; 
        
        // Coordonnées Cartésiennes (X, Y)
        const translateXPercent = radialDistance * Math.sin(thetaRad);
        const translateYPercent = radialDistance * Math.cos(thetaRad) * -1; // Y inversé
        
        sunEl.style.transform = `translate(${translateXPercent}%, ${translateYPercent}%)`; 
        sunEl.style.display = altDeg > -0.83 ? 'flex' : 'none'; 
    }

    // --- Position de la Lune (Logique similaire) ---
    if (moonPos) {
        // ... (Code similaire pour la lune) ...
    }
}


// --- Mise à jour Astro (TST, LST, Lever/Coucher) ---
function updateAstro(lat, lon, kAlt, now) {
    // ... (Logique TST, EOT et LST) ...
    const solarTimes = getSolarTime(now, lon); // Assume getSolarTime existe et retourne TST/EOT
    const sunTimes = SunCalc.getTimes(now, lat, lon);
    
    // Mise à jour de l'heure sidérale
    const LST_h = getLocalSiderealTime(now, lon);
    const LST_hms = msToHMS((LST_h % 24) * 3600000);
    $('local-sidereal-time').textContent = LST_hms;

    // Mise à jour de la visualisation Minecraft
    updateClockVisualization(lat, lon, kAlt, now);

    // ... (Mise à jour des heures de lever/coucher TST) ...
    if ($('sunrise-tst') && sunTimes.sunrise && solarTimes.TST !== 'N/A') {
        const msToTime = (ms) => { 
            let h = Math.floor(ms / 3600000); 
            let m = Math.floor((ms % 3600000) / 60000); 
            let s = Math.floor((ms % 60000) / 1000);
            return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
        };

        // Calcul du décalage (EOT) pour obtenir le TST
        const eotMs = solarTimes.EOT * 60000;
        
        // Lever/Coucher solaire en TST
        const sunriseTSTMs = (sunTimes.sunrise.getTime() % dayMs) - eotMs;
        const sunsetTSTMs = (sunTimes.sunset.getTime() % dayMs) - eotMs;
        
        $('sunrise-tst').textContent = `↑ ${msToTime(sunriseTSTMs)} / ↓ ${msToTime(sunsetTSTMs)} (TST)`;
    }
    // ... (Reste de la mise à jour Astro) ...
}


// --- Boucle de Mise à Jour du DOM (Démarrage) ---
document.addEventListener('DOMContentLoaded', () => {
    // Écouteur pour le bouton PAUSE/MARCHE
    $('gps-pause-button').addEventListener('click', () => {
        wID === null ? startGPS() : stopGPS();
    });

    // Démarrage de la boucle DOM lente
    if (domID === null) {
        domID = setInterval(() => {
            // Tentative de synchronisation de l'heure NTP si nécessaire...
            const now = new Date();
            if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString('fr-FR', { hour12: false });
            if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
            
            // Mise à jour de l'heure Minecraft (si nécessaire)
            // ...

        }, DOM_SLOW_UPDATE_MS); 
    }
    
    // Démarrage initial (Facultatif)
    // startGPS(); 
});
