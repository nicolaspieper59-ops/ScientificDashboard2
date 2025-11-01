// =================================================================
// GNSS/IMU Dashboard JS (V11.9 - VITESSE 3D, MÉTÉO, NOUVEAUX BOUTONS)
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

// --- ÉTAT EKF VERTICAL (Altitude/Vitesse Verticale) ---
let EKF_State_V = {
    P: [[100, 0], [0, 100]], 
    Q: 0.1, 
    R: 10,  
    altitude: null, // CORRECTION : Initialisé à null pour indiquer qu'aucune mesure n'a été prise
    v_vertical: 0
};


// =================================================================
// 1. FILTRES DE KALMAN (Horizontal et Vertical)
// =================================================================

// ... [EKF_Predict_H, EKF_Update_H, EKF_Predict_V, EKF_Update_V - Fonctions de base inchangées] ...
// (Elles sont conservées ici pour la complétude du fichier.)

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

function EKF_Predict_V(dt) {
    const P = EKF_State_V.P;
    const Q = EKF_State_V.Q;
    const dt2 = dt * dt;
    
    if (EKF_State_V.altitude !== null) {
        EKF_State_V.altitude = EKF_State_V.altitude + EKF_State_V.v_vertical * dt;
    }
    
    P[0][0] = P[0][0] + dt * P[1][0] + dt * P[0][1] + dt2 * P[1][1] + Q * dt2 * dt;
    P[0][1] = P[0][1] + dt * P[1][1] + Q * dt2;
    P[1][0] = P[1][0] + dt * P[1][1] + Q * dt2;
    P[1][1] = P[1][1] + Q * dt;
    EKF_State_V.P = P;
}

function EKF_Update_V(measurement_altitude, measurement_accuracy) {
    // CORRECTION : Initialise l'altitude si c'est la première mesure
    if (EKF_State_V.altitude === null) {
        EKF_State_V.altitude = measurement_altitude;
        return; 
    }
    
    const P = EKF_State_V.P;
    const altitude_current = EKF_State_V.altitude;
    const v_vertical_current = EKF_State_V.v_vertical;

    const R = measurement_accuracy * measurement_accuracy; 
    const Y = measurement_altitude - altitude_current;
    const S = P[0][0] + R; 
    const K_alt = P[0][0] / S;
    const K_v = P[1][0] / S; 
    
    EKF_State_V.altitude = altitude_current + K_alt * Y;
    EKF_State_V.v_vertical = v_vertical_current + K_v * Y;
    
    P[0][0] = P[0][0] - K_alt * P[0][0];
    P[0][1] = P[0][1] - K_alt * P[0][1];
    P[1][0] = P[1][0] - K_v * P[0][0]; 
    P[1][1] = P[1][1] - K_v * P[0][1];
    
    EKF_State_V.P = P;
}


// =================================================================
// 2. TEMPS ET ASTRO (Correction)
// =================================================================

// ... [attemptNTPTimeSync, updateLocalTime - Inchangés] ...

/**
 * Mise à jour de l'astronomie (SunCalc).
 * @param {number} latitude 
 * @param {number} longitude 
 */
function updateAstronomy(latitude, longitude) {
    const now = new Date();
    // VÉRIFICATION : S'assurer que SunCalc est chargé
    if (typeof SunCalc === 'undefined') {
        $('time-solar-true').textContent = 'LIB. MANQUANTE';
        return;
    }

    const sunPos = SunCalc.getPosition(now, latitude, longitude);
    const moonIllumination = SunCalc.getMoonIllumination(now);
    const times = SunCalc.getTimes(now, latitude, longitude);

    const eclipticLongDeg = sunPos.eclipticLng * 180 / Math.PI;
    const sunElevationDeg = sunPos.altitude * 180 / Math.PI;
    const sunAzimuthDeg = (sunPos.azimuth * 180 / Math.PI + 180) % 360;

    let eotMinutes = 'N/D';
    if (times.solarNoon) {
        // Temps universel du midi solaire
        const solarNoonUTC = times.solarNoon.getTime();
        // Temps moyen local (midi local) - 12h UTC est le point de référence
        const noonUTC = new Date(now.getUTCFullYear(), now.getUTCMonth(), now.getUTCDate(), 12, 0, 0).getTime();
        // EOT = TST - TSM. Ici, TST est midi solaire (temps réel) et TSM est midi moyen (temps calculé)
        const eotOffsetMs = solarNoonUTC - noonUTC;
        eotMinutes = (eotOffsetMs / (1000 * 60)).toFixed(2);
    }
    
    // Mise à jour du DOM
    $('sun-elevation').textContent = sunElevationDeg.toFixed(2) + ' °';
    $('sun-azimuth').textContent = sunAzimuthDeg.toFixed(2) + ' °';
    $('ecliptic-long').textContent = eclipticLongDeg.toFixed(2) + ' °';
    $('eot-min').textContent = eotMinutes + ' min';
    $('lunar-phase-perc').textContent = (moonIllumination.fraction * 100).toFixed(1) + ' %';
    $('moon-times').textContent = times.moonrise ? times.moonrise.toLocaleTimeString('fr-FR') : 'N/D';
    
    // Le TST peut être plus complexe, affichons le temps de culmination pour le moment
    $('time-solar-true').textContent = times.solarNoon ? times.solarNoon.toLocaleTimeString('fr-FR') + ' (Culm.)' : 'N/D';
}


// =================================================================
// 3. MÉTÉO ET ENVIRONNEMENT (Correction)
// =================================================================

/**
 * Calcule la densité de l'air et le facteur de correction EKF.
 * @param {number|null} altitude_m - Altitude stabilisée en mètres.
 */
function updateEnvironmentalData(altitude_m) {
    // Si l'altitude n'est pas encore stabilisée, on utilise une altitude standard de 0m
    const altitude = altitude_m !== null ? altitude_m : 0; 
    
    // 1. Calcul de la Densité de l'Air (kg/m³)
    const pressure = WEATHER_DATA.pressure_pa; // Pa
    const temperature = WEATHER_DATA.temp_air_K; // K
    const density = pressure / (R_AIR_SPECIFIC * temperature); 
    WEATHER_DATA.density_air = density;

    // 2. Calcul du Facteur de Correction EKF (Impact de l'altitude/densité)
    const standardDensity = 1.225;
    let factor = standardDensity / density;
    factor = Math.min(1.5, Math.max(1.0, factor)); 

    WEATHER_DATA.ekf_correction_factor = factor;
    
    $('env-factor').textContent = `x${factor.toFixed(2)}`;
}

/**
 * Simule ou récupère les données météo.
 * @param {number} lat
 * @param {number} lng
 */
async function getWeatherData(lat, lng) {
    if (Date.now() - WEATHER_LAST_FETCH < WEATHER_FETCH_INTERVAL) {
        // S'assurer que l'affichage est mis à jour même sans nouveau fetch
        updateWeatherDisplay(); 
        return;
    }

    try {
        // --- SIMULATION (A remplacer par une API réelle) ---
        const temp_c = 15; 
        const pressure_hpa = 1013;
        const humidity_perc = 50;
        
        WEATHER_DATA.temp_air_K = temp_c + 273.15;
        WEATHER_DATA.pressure_pa = pressure_hpa * 100;
        WEATHER_DATA.humidity_perc = humidity_perc;
        
        WEATHER_LAST_FETCH = Date.now();
        
        // --- FIN SIMULATION ---
        
    } catch (error) {
        console.warn('Weather API failed, using standard environment data:', error);
    }
    updateWeatherDisplay();
}

/**
 * Met à jour l'affichage des données météo (appelé après chaque update ou fetch).
 */
function updateWeatherDisplay() {
    $('temp-air').textContent = `${(WEATHER_DATA.temp_air_K - 273.15).toFixed(1)} °C`;
    $('pressure').textContent = `${(WEATHER_DATA.pressure_pa / 100).toFixed(1)} hPa`;
    $('humidity').textContent = `${WEATHER_DATA.humidity_perc.toFixed(0)} %`;
}


// =================================================================
// 4. LOGIQUE PRINCIPALE GPS (Correction Vitesse 3D)
// =================================================================

function calculateSpeed3D(speedHorizontal, speedVertical) {
    // VÉRIFICATION : S'assurer que les valeurs ne sont pas nulles avant la racine carrée
    speedHorizontal = speedHorizontal || 0;
    speedVertical = speedVertical || 0;
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

    let speedStabilizedVertical = 0;
    let altitudeStabilized = null; 

    // --- 1. FILTRAGE VERTICAL ---
    if (altitudeRaw !== null) {
        EKF_Predict_V(dt);
        EKF_Update_V(altitudeRaw, coords.altitudeAccuracy || coords.accuracy || EKF_State_V.R); 
        altitudeStabilized = EKF_State_V.altitude;
        speedStabilizedVertical = EKF_State_V.v_vertical;
    }
    // Si l'altitude n'est pas disponible, on continue avec un EKF H et une vitesse 3D = vitesse H

    // --- 2. MÉTÉO & ENVIRONNEMENT ---
    getWeatherData(lat, lng); 
    updateEnvironmentalData(altitudeStabilized); 

    // --- 3. FILTRAGE HORIZONTAL (Vitesse) ---
    EKF_Predict_H(dt);
    EKF_Update_H(speedRawHorizontal, accuracy * WEATHER_DATA.ekf_correction_factor); 
    const speedStableHorizontal = EKF_State_H.velocity;
    
    // --- 4. VITESSE 3D (Utilise les valeurs stabilisées ou 0 si EKF-V non fonctionnel) ---
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

// ... [gpsError, startGPS, initMap, updateMap sont omis pour la concision] ...
// ... [updateDisplay est mis à jour pour afficher 'EKF' pour la vitesse verticale et l'altitude] ...


// =================================================================
// 5. NOUVEAUX BOUTONS & CONTROLS
// =================================================================

function handleCapture() {
    alert("Fonction CAPTURE/SNAPSHOT activée. Les données actuelles ont été loggées.");
    console.log("CAPTURE DES DONNÉES ACTUELLES:", {
        time: $('local-time').textContent,
        lat: $('latitude').textContent,
        lng: $('longitude').textContent,
        speedStable: $('speed-stable').textContent,
        altitude: $('altitude-gps').textContent,
        temp: $('temp-air').textContent
    });
}

function handleMaterials() {
    const currentMaterial = prompt("Entrez le type de matériau (ex: Acier, Polymère, Air) pour les calculs de frottement/traînée:", "Air");
    if (currentMaterial) {
        alert(`Matériau sélectionné: ${currentMaterial}. (La logique de calculs avancés des matériaux n'est pas implémentée dans cette version.)`);
    }
}

// =================================================================
// 6. INITIALISATION
// =================================================================

async function initGeolocation() {
    // ... (Logique d'initialisation de la V11.8) ...
    initMap();
    
    // Initialisation des listeners
    const $massInput = $('mass-input');
    if ($massInput) { /* ... */ }

    // --- ÉCOUTEURS DE BOUTONS (Mise à jour) ---
    $('toggle-gps-btn').addEventListener('click', startGPS);
    $('freq-select').addEventListener('change', (e) => {
        currentGPSMode = e.target.value;
        if (WATCH_ID) startGPS(); 
    });
    // Les autres écouteurs (reset, emergency, nether) sont conservés

    // NOUVEAU : Écouteurs pour les nouveaux boutons (doivent exister dans le HTML)
    const $captureBtn = $('capture-btn');
    if ($captureBtn) $captureBtn.addEventListener('click', handleCapture);

    const $materialsBtn = $('materials-btn');
    if ($materialsBtn) $materialsBtn.addEventListener('click', handleMaterials);

    // --- DÉMARRAGE DES SERVICES ---
    await attemptNTPTimeSync(); 

    setInterval(updateLocalTime, 1000);
    updateLocalTime();
    
    // Démarrage initial Météo et Astro avec position par défaut
    updateAstronomy(MAP_DEFAULT_LAT, MAP_DEFAULT_LNG); 
    getWeatherData(MAP_DEFAULT_LAT, MAP_DEFAULT_LNG);
    updateEnvironmentalData(0); 
    
    setInterval(() => updateAstronomy(MAP_DEFAULT_LAT, MAP_DEFAULT_LNG), 60000); 
    setInterval(() => getWeatherData(MAP_DEFAULT_LAT, MAP_DEFAULT_LNG), WEATHER_FETCH_INTERVAL);

    requestDeviceMotionPermission();
}

// Lancement de l'application
document.addEventListener('DOMContentLoaded', initGeolocation);


// --- Fonctions de soutien (Assurez-vous qu'elles sont incluses dans le fichier final) ---
function gpsError(error) { /* ... */ }
function startGPS() { /* ... */ }
function updateDisplay(coords, speedRaw, speedStable, speed3D, speedVertical, altitudeStabilized) { /* ... */ }
function resetDisp(fullReset = true) { /* ... */ }
function requestDeviceMotionPermission() { /* ... */ }
function startDeviceMotionTracking() { /* ... */ }
function updateDynamicIMUDisplay() { /* ... */ }
// ... (Les autres fonctions doivent être incluses dans le fichier .js)
