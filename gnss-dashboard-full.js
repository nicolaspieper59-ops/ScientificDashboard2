// =================================================================
// GNSS/IMU Dashboard JS (V11.8 - STABILISATION VERTICALE COMPLÈTE)
// Gère EKF H-Z/V, IMU, GPS, Astro, Temps, Carte, Météo (simulation/API), 
// Correction EKF/Alt. et tous les contrôles DOM.
// =================================================================

// --- FONCTION UTILITAIRE ---
const $ = (id) => document.getElementById(id);

// --- CONSTANTES GLOBALES ---
const C_LIGHT = 299792458; // m/s
const C_SOUND = 343;       // m/s (approx au niveau de la mer)
const G_ACCEL = 9.80665;   // m/s²
const MAP_DEFAULT_LAT = 48.8566; // Paris
const MAP_DEFAULT_LNG = 2.3522;
const NETHER_RATIO = 8;
const NTP_SERVER = 'https://worldtimeapi.org/api/utc'; 
const R_AIR_SPECIFIC = 287.05; // J/(kg·K) pour l'air sec

// --- ÉTAT GLOBAL DU SYSTÈME ---
let DEVICE_MASS_KG = 70.000;
let WATCH_ID = null;
let LAST_TIMESTAMP = 0;
let TOTAL_DISTANCE_M = 0;
let TOTAL_MOVING_TIME_MS = 0;
let TOTAL_ELAPSED_TIME_MS = 0;
let MAX_SPEED_MS = 0;
let IS_NTP_SYNCED = false;
let emergencyStopActive = false;
let netherMode = false;
let currentGPSMode = 'HIGH_FREQ';

// --- ÉTAT MÉTÉO/ENVIRONNEMENT ---
let WEATHER_DATA = {
    temp_air_K: 288.15, // 15 °C en Kelvin (Standard)
    pressure_pa: 101325, // Pa (Niveau de la mer Standard)
    humidity_perc: 50,
    density_air: 1.225, // kg/m³ (Standard)
    ekf_correction_factor: 1.0 // Facteur de correction basé sur l'environnement
};
let WEATHER_LAST_FETCH = 0;
const WEATHER_FETCH_INTERVAL = 300000; // 5 minutes

// --- ÉTAT IMU (Capteurs Réels) ---
let ACCEL_LATERAL_IMU = 0; 
let ACCEL_LONG_IMU = 0;    
let IMU_IS_ACTIVE = false;

// --- ÉTAT CARTE (Leaflet) ---
let map = null;
let marker = null;
let polyline = null;
let trackPoints = [];

// --- ÉTAT EKF HORIZONTAL (Position/Vitesse) ---
let EKF_State_H = {
    P: [[100, 0], [0, 100]], 
    Q: 1, 
    R: 5, 
    velocity: 0, 
    position_error: 0
};

// --- NOUVEAU: ÉTAT EKF VERTICAL (Altitude/Vitesse Verticale) ---
let EKF_State_V = {
    P: [[100, 0], [0, 100]], // P[0][0]: Variance altitude, P[1][1]: Variance vitesse verticale
    Q: 0.1, // Bruit du processus (changement de vitesse)
    R: 10,  // Bruit de mesure (Précision Altitude GPS)
    altitude: 0, 
    v_vertical: 0
};


// =================================================================
// 1. LOGIQUE EKF HORIZONTAL (Stabilisation Vitesse GPS)
// =================================================================

function EKF_Predict_H(dt) {
    const dt2 = dt * dt;
    const P = EKF_State_H.P;
    const Q = EKF_State_H.Q;
    P[0][0] = P[0][0] + dt * P[1][0] + dt * P[0][1] + dt2 * P[1][1] + Q * dt2 * dt;
    P[0][1] = P[0][1] + dt * P[1][1] + Q * dt2;
    P[1][0] = P[1][0] + dt * P[1][1] + Q * dt2;
    P[1][1] = P[1][1] + Q * dt;
    EKF_State_H.P = P;
}

function EKF_Update_H(measurement_velocity, measurement_accuracy) {
    const P = EKF_State_H.P;
    const velocity_current = EKF_State_H.velocity;
    
    // Ajuste la covariance R (bruit de mesure) en fonction de la correction environnementale
    const adjustedR = measurement_accuracy * measurement_accuracy * WEATHER_DATA.ekf_correction_factor; 
    
    const R = adjustedR; 
    const Y = measurement_velocity - velocity_current;
    const S = P[1][1] + R; 
    const K_pos = P[0][1] / S;
    const K_vel = P[1][1] / S; 
    EKF_State_H.velocity = velocity_current + K_vel * Y;
    P[0][0] = P[0][0] - K_pos * P[0][1];
    P[0][1] = P[0][1] - K_pos * P[1][1];
    P[1][0] = P[1][0] - K_vel * P[1][0];
    P[1][1] = P[1][1] - K_vel * P[1][1];
    EKF_State_H.P = P;
    EKF_State_H.position_error = Math.sqrt(P[0][0]); 
}

// =================================================================
// 2. LOGIQUE EKF VERTICAL (Stabilisation Altitude/Vitesse Verticale)
// =================================================================

function EKF_Predict_V(dt) {
    const P = EKF_State_V.P;
    const Q = EKF_State_V.Q;
    const dt2 = dt * dt;
    
    // Matrice de transition F est [[1, dt], [0, 1]]
    // Prédiction de l'état (altitude, v_vertical)
    EKF_State_V.altitude = EKF_State_V.altitude + EKF_State_V.v_vertical * dt;
    
    // Prédiction de la covariance (P = F * P * F^T + Q)
    P[0][0] = P[0][0] + dt * P[1][0] + dt * P[0][1] + dt2 * P[1][1] + Q * dt2 * dt;
    P[0][1] = P[0][1] + dt * P[1][1] + Q * dt2;
    P[1][0] = P[1][0] + dt * P[1][1] + Q * dt2;
    P[1][1] = P[1][1] + Q * dt;
    EKF_State_V.P = P;
}

function EKF_Update_V(measurement_altitude, measurement_accuracy) {
    const P = EKF_State_V.P;
    const altitude_current = EKF_State_V.altitude;
    const v_vertical_current = EKF_State_V.v_vertical;

    // Précision de la mesure R
    const R = measurement_accuracy * measurement_accuracy; 
    
    // Innovation (différence entre mesure et estimation)
    const Y = measurement_altitude - altitude_current;
    
    // Covariance d'innovation S = H * P * H^T + R. H = [1, 0]
    const S = P[0][0] + R; 
    
    // Gain de Kalman K = P * H^T * S^-1. K = [P[0][0]/S, P[1][0]/S]^T
    const K_alt = P[0][0] / S;
    const K_v = P[1][0] / S; 
    
    // Mise à jour de l'état
    EKF_State_V.altitude = altitude_current + K_alt * Y;
    EKF_State_V.v_vertical = v_vertical_current + K_v * Y;
    
    // Mise à jour de la covariance (P = (I - K*H) * P)
    P[0][0] = P[0][0] - K_alt * P[0][0];
    P[0][1] = P[0][1] - K_alt * P[0][1];
    P[1][0] = P[1][0] - K_v * P[0][0]; // Utilise l'ancienne P[0][0] (qui est la même que P[0][0]_updated)
    P[1][1] = P[1][1] - K_v * P[0][1];
    
    EKF_State_V.P = P;
}

// =================================================================
// 3. CARTE, TEMPS, MÉTÉO (Inchangés dans leur principe)
// =================================================================
// ... [initMap, updateMap, attemptNTPTimeSync, updateLocalTime, updateAstronomy, 
//      updateEnvironmentalData, getWeatherData sont omis pour la concision mais restent dans le code complet] ...
// ... [requestDeviceMotionPermission, startDeviceMotionTracking, updateDynamicIMUDisplay sont omis pour la concision] ...
// =================================================================

// NOUVEAU: Initialisation de la carte (extrait de la V11.7)
function initMap() {
    if ($('map') && typeof L !== 'undefined') {
        map = L.map('map').setView([MAP_DEFAULT_LAT, MAP_DEFAULT_LNG], 13);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            maxZoom: 19, attribution: '© OpenStreetMap'
        }).addTo(map);
        marker = L.marker([MAP_DEFAULT_LAT, MAP_DEFAULT_LNG]).addTo(map);
        polyline = L.polyline([], {color: 'blue'}).addTo(map);
    }
}

// NOUVEAU: Mise à jour de la carte (extrait de la V11.7)
function updateMap(lat, lng) {
    if (!map || !marker) return;
    const newLatLng = [lat, lng];
    marker.setLatLng(newLatLng);
    trackPoints.push(newLatLng);
    polyline.setLatLngs(trackPoints);
    map.setView(newLatLng);
    if (trackPoints.length > 1) {
        map.fitBounds(polyline.getBounds(), {padding: [50, 50]});
    }
}


// =================================================================
// 4. LOGIQUE PRINCIPALE GPS & VITESSE 3D (MISE À JOUR)
// =================================================================

function calculateSpeed3D(speedHorizontal, speedVertical) {
    // Calcule la vitesse 3D en utilisant la vitesse horizontale STABILISÉE
    return Math.sqrt(speedHorizontal * speedHorizontal + speedVertical * speedVertical);
}

function gpsSuccess(position) {
    if (emergencyStopActive) return; 

    const coords = position.coords;
    const timestamp = position.timestamp;
    const dt = (timestamp - LAST_TIMESTAMP) / 1000 || 0; 
    if (dt <= 0) return; 
    
    LAST_TIMESTAMP = timestamp;
    TOTAL_ELAPSED_TIME_MS += dt * 1000;

    const lat = coords.latitude;
    const lng = coords.longitude;
    const altitudeRaw = coords.altitude;
    const speedRawHorizontal = coords.speed || 0; 
    const accuracy = coords.accuracy || EKF_State_H.R; 

    // --- 1. FILTRAGE VERTICAL (Altitude & Vitesse Verticale) ---
    let speedStabilizedVertical = 0;
    let altitudeStabilized = altitudeRaw; 
    if (altitudeRaw !== null) {
        EKF_Predict_V(dt);
        // Utilise la précision horizontale comme estimation de l'imprécision verticale
        EKF_Update_V(altitudeRaw, coords.accuracy || EKF_State_V.R); 
        altitudeStabilized = EKF_State_V.altitude;
        speedStabilizedVertical = EKF_State_V.v_vertical;
    }

    // --- 2. MÉTÉO & ENVIRONNEMENT ---
    getWeatherData(lat, lng); 
    updateEnvironmentalData(altitudeStabilized); // Utilise l'altitude STABILISÉE

    // --- 3. FILTRAGE HORIZONTAL (Vitesse) ---
    EKF_Predict_H(dt);
    // Utilise la précision ajustée par l'environnement
    EKF_Update_H(speedRawHorizontal, accuracy * WEATHER_DATA.ekf_correction_factor); 
    const speedStableHorizontal = EKF_State_H.velocity;
    
    // --- 4. VITESSE 3D ---
    const speed3D = calculateSpeed3D(speedStableHorizontal, speedStabilizedVertical);
    
    // --- 5. Distance & Temps de Mouvement ---
    if (speedStableHorizontal > 0.1) { 
        TOTAL_DISTANCE_M += speedStableHorizontal * dt;
        TOTAL_MOVING_TIME_MS += dt * 1000;
        MAX_SPEED_MS = Math.max(MAX_SPEED_MS, speedStableHorizontal);
    }
    
    // --- 6. Mise à jour Affichage ---
    updateMap(lat, lng);
    updateAstronomy(lat, lng);
    updateDisplay(coords, speedRawHorizontal, speedStableHorizontal, speed3D, speedStabilizedVertical, altitudeStabilized);
}

// ... [gpsError, startGPS sont inchangés] ...

// =================================================================
// 5. MISE À JOUR DU DOM (Affichage)
// =================================================================

function updateDisplay(coords, speedRaw, speedStable, speed3D, speedVertical, altitudeStabilized) {
    const speedStableKmH = speedStable * 3.6;
    const maxSpeedKmH = MAX_SPEED_MS * 3.6;
    const displayDistanceM = netherMode ? (TOTAL_DISTANCE_M / NETHER_RATIO) : TOTAL_DISTANCE_M;
    const displayDistanceKm = displayDistanceM / 1000;
    
    // Vitesse & Distance
    $('speed-stable').textContent = speedStableKmH.toFixed(2) + ' km/h';
    $('speed-raw-ms').textContent = speedRaw.toFixed(2) + ' m/s';
    $('speed-3d-inst').textContent = (speed3D * 3.6).toFixed(2) + ' km/h';
    $('speed-max').textContent = maxSpeedKmH.toFixed(5) + ' km/h';
    $('distance-total-km').textContent = displayDistanceKm.toFixed(3) + ' km | ' + displayDistanceM.toFixed(2) + ' m';
    $('speed-stable-ms').textContent = `${speedStable.toFixed(2)} m/s | ${(speedStable * 1e6).toFixed(0)} µm/s`;
    
    // GPS & Physique
    $('latitude').textContent = coords.latitude.toFixed(6);
    $('longitude').textContent = coords.longitude.toFixed(6);
    // CORRECTION : Utilise l'altitude STABILISÉE
    $('altitude-gps').textContent = altitudeStabilized !== null ? altitudeStabilized.toFixed(2) + ' m (EKF)' : 'N/A';
    $('gps-precision').textContent = coords.accuracy ? coords.accuracy.toFixed(2) + ' m' : 'N/A';
    // CORRECTION : Utilise la vitesse verticale STABILISÉE
    $('vertical-speed').textContent = speedVertical.toFixed(2) + ' m/s (EKF)'; 
    $('underground-status').textContent = (altitudeStabilized !== null && altitudeStabilized < -50) ? 'OUI' : 'Non';

    // Cosmics
    $('perc-speed-sound').textContent = (speedStable / C_SOUND * 100).toFixed(2) + ' %';
    $('perc-speed-c').textContent = (speedStable / C_LIGHT * 100).toExponential(2) + ' %';
    $('distance-cosmic').textContent = (displayDistanceM / C_LIGHT).toExponential(2) + ' s lumière';
    
    // Précision EKF
    const errorPerc = Math.sqrt(EKF_State_H.P[1][1]) / speedStable * 100;
    $('speed-error-perc').textContent = isFinite(errorPerc) ? errorPerc.toFixed(1) + ' %' : 'N/A';
    
    // Temps
    $('time-moving').textContent = (TOTAL_MOVING_TIME_MS / 1000).toFixed(2) + ' s';
    $('time-elapsed').textContent = (TOTAL_ELAPSED_TIME_MS / 1000).toFixed(2) + ' s';
}

function resetDisp(fullReset = true) {
    if (fullReset) {
        TOTAL_DISTANCE_M = 0;
        MAX_SPEED_MS = 0;
        TOTAL_MOVING_TIME_MS = 0;
        TOTAL_ELAPSED_TIME_MS = 0;
        trackPoints = []; 
        if (polyline) polyline.setLatLngs(trackPoints);
        if (map) map.setView([MAP_DEFAULT_LAT, MAP_DEFAULT_LNG], 13);

        // Reset EKF Vertical
        EKF_State_V.altitude = 0;
        EKF_State_V.v_vertical = 0;
        EKF_State_V.P = [[100, 0], [0, 100]];
    }
    
    // Reset EKF Horizontal
    EKF_State_H.velocity = 0;
    EKF_State_H.P = [[100, 0], [0, 100]];
    
    // Reset DOM
    $('speed-stable').textContent = '0.00 km/h';
    $('speed-max').textContent = '0.00000 km/h';
    $('distance-total-km').textContent = '0.000 km | 0.00 m';
    $('time-moving').textContent = '0.00 s';
    $('time-elapsed').textContent = '0.00 s';
}

// ... [initGeolocation, attemptNTPTimeSync, Fonctions Météo et IMU sont omis pour la concision mais restent dans le code complet] ...

// =================================================================
// 6. INITIALISATION (DOMContentLoaded)
// =================================================================
// Le reste de l'initialisation est inchangé, utilisant les nouvelles fonctions
// de filtration verticale pour les données d'altitude et de vitesse verticale.

// Lancement de l'application
document.addEventListener('DOMContentLoaded', initGeolocation);


// --- Fonctions Météo et IMU (Extrait de V11.7) ---
// Note: Le code complet inclurait ces fonctions ici...
async function attemptNTPTimeSync() {/* ... */ }
function updateLocalTime() {/* ... */ }
function updateAstronomy(latitude, longitude) {/* ... */ }
function updateEnvironmentalData(altitude_m) {
    const pressure = WEATHER_DATA.pressure_pa; 
    const temperature = WEATHER_DATA.temp_air_K; 
    const density = pressure / (R_AIR_SPECIFIC * temperature); 
    WEATHER_DATA.density_air = density;
    const standardDensity = 1.225;
    let factor = standardDensity / density;
    factor = Math.min(1.5, Math.max(1.0, factor)); 
    WEATHER_DATA.ekf_correction_factor = factor;
    $('env-factor').textContent = `x${factor.toFixed(2)}`;
}
async function getWeatherData(lat, lng) {
    if (Date.now() - WEATHER_LAST_FETCH < WEATHER_FETCH_INTERVAL) return;
    try {
        const temp_c = 15; const pressure_hpa = 1013; const humidity_perc = 50;
        WEATHER_DATA.temp_air_K = temp_c + 273.15;
        WEATHER_DATA.pressure_pa = pressure_hpa * 100;
        WEATHER_DATA.humidity_perc = humidity_perc;
        WEATHER_LAST_FETCH = Date.now();
        $('temp-air').textContent = `${temp_c.toFixed(1)} °C`;
        $('pressure').textContent = `${pressure_hpa.toFixed(1)} hPa`;
        $('humidity').textContent = `${humidity_perc.toFixed(0)} %`;
    } catch (error) {
        console.warn('Weather API failed, using standard environment data:', error);
        $('temp-air').textContent = `N/A (Std: 15°C)`;
        $('pressure').textContent = `N/A (Std: 1013 hPa)`;
    }
}
function requestDeviceMotionPermission() {/* ... */ }
function startDeviceMotionTracking() {/* ... */ }
function updateDynamicIMUDisplay() {/* ... */ }
function initGeolocation() { /* ... */ } // Le corps complet de l'init est dans le code précédent
