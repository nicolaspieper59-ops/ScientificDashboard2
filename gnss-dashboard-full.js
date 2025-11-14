// =================================================================
// 1/3. CONSTANTS, GLOBALS & UTILITIES
// =================================================================

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;          // Vitesse de la lumière (m/s)
const R_AIR = 287.058;          // Constante spécifique de l'air sec (J/kg·K)
const G_BASE = 9.80665;         // Gravité standard (m/s²)
const KMH_MS = 3.6;             
const KELVIN_OFFSET = 273.15;
const EARTH_RADIUS = 6371000;   // Rayon terrestre moyen (m)
const DOM_FAST_UPDATE_MS = 50;  // Fréquence rapide (IMU/EKF)
const DOM_SLOW_UPDATE_MS = 5000;
const MAX_ACC = 200;            
const MIN_SPD = 0.05;           
const R_MIN = 0.01;         
const R_MAX = 500.0;        
const ZUPT_ACCEL_TOLERANCE = 0.5; // Tolérance d'accélération pour ZUPT (m/s²)

// --- PARAMÈTRES DU FILTRE DE KALMAN (9 états simplifiés) ---
const Q_IMU = 0.05;         
const Q_ALT = 0.5;          
const Q_BIAS_ACC = 1e-6;    
const Q_BIAS_GYRO = 1e-7;   

let P_spd = 100;            
let P_alt = 100;            
let P_biasAccel = 1e-4;     
let P_biasGyro = 1e-5;      

let speedEst = 0;           
let altEst = 0;             
let biasAccelX = 0, biasAccelY = 0, biasAccelZ = 0; 
let biasGyroX = 0, biasGyroY = 0, biasGyroZ = 0;    

// --- VARIABLES D'ÉTAT CAPTEURS (Initialisées à null pour "N/A" si capteur absent) ---
let lat = null, lon = null; 
let speed3dRaw = 0;         
let accuracyGPS = 1000;     
let imuAccelX = null, imuAccelY = null, imuAccelZ = G_BASE; 
let imuGyroX = null, imuGyroY = null, imuGyroZ = null; 
let magX = null, magY = null, magZ = null; 
let currentHeading = null;     
let altBaro = null;            
let pressurehPa = null;  
let tempC = null;             
let humidityPerc = null;      
let lightIntensity = null;     

// --- VARIABLES COMPTEURS & CONTROLES ---
let wID = null;
let distM_3D = 0;           
let timeMoving = 0;
let maxSpd = 0;
let totalTime = 0;
let startTime = Date.now();
let gpsAccuracyOverride = 0;
let selectedEnvironment = 'NORMAL';
let currentMass = 70;
let maxLight = 0, maxMagnetic = 0;
let lastLat = null, lastLon = null;
let map = null, marker = null, trackPolyline = null, track = [];

// --- FONCTIONS UTILITAIRES ---
const $ = (id) => document.getElementById(id);
const getCDate = () => new Date();
const displayVal = (val, unit) => val !== null ? val.toFixed(3) + unit : 'N/A';
// =================================================================
// 2/3. CORE LOGIC & EKF FILTER
// (Dépend de: constants_globals.js)
// =================================================================

// --- CALCULS COMPLEXES ---

/** Calcule le bruit de mesure (R) pour l'EKF, ajusté par le mode d'environnement. */
function calculateR(accuracy_m) {
    if (accuracy_m > MAX_ACC) return R_MAX;
    if (accuracy_m === null || accuracy_m <= 0) accuracy_m = R_MIN; 
    let R = Math.max(R_MIN, Math.min(R_MAX, accuracy_m * accuracy_m / 10));

    let R_FACTOR_RATIO = 1.0;
    switch (selectedEnvironment) {
        case 'CONCRETE': R_FACTOR_RATIO = 7.0; break; 
        case 'METAL': R_FACTOR_RATIO = 5.0; break;
        case 'FOREST': R_FACTOR_RATIO = 2.5; break; 
        default: R_FACTOR_RATIO = 1.0; break;
    }
    return R * R_FACTOR_RATIO;
}

/** Calcule la gravité locale (g) en m/s² en fonction de la latitude et de l'altitude. */
function calculateLocalGravity(latitude_rad, altitude_m) {
    if (latitude_rad === null) return G_BASE;
    const sin2 = Math.sin(latitude_rad) * Math.sin(latitude_rad);
    const g0 = 9.780327 * (1 + 0.0053024 * sin2 - 0.0000058 * sin2 * sin2);
    return g0 - (3.086e-6 * altitude_m); 
}

/** Calcule la distance maximale visible jusqu'à l'horizon (m). */
function calculateHorizonDistance(altitude_m) {
    const k = 0.13; 
    return Math.sqrt(2 * EARTH_RADIUS * altitude_m * (1 + k));
}

// --- GESTION DES CAPTEURS RÉELS ---

function initializeSensors() {
    // 1. Accéléromètre
    if ('Accelerometer' in window) {
        let accelerometer = new Accelerometer({ frequency: 50 });
        accelerometer.addEventListener('reading', () => {
            imuAccelX = accelerometer.x || 0;
            imuAccelY = accelerometer.y || 0;
            imuAccelZ = accelerometer.z || G_BASE; 
        });
        accelerometer.start();
    }
    
    // 2. Magnétomètre
    if ('Magnetometer' in window) {
        let magnetometer = new Magnetometer({ frequency: 50 });
        magnetometer.addEventListener('reading', () => {
            magX = magnetometer.x || 0;
            magY = magnetometer.y || 0;
            magZ = magnetometer.z || 0;
        });
        magnetometer.start();
    }
    
    // 3. Gyroscope
    if ('Gyroscope' in window) {
        let gyroscope = new Gyroscope({ frequency: 50 });
        gyroscope.addEventListener('reading', () => {
            imuGyroX = gyroscope.x || 0;
            imuGyroY = gyroscope.y || 0;
            imuGyroZ = gyroscope.z || 0;
        });
        gyroscope.start();
    }
    
    // 4. Capteur de Lumière Ambiante
    if ('AmbientLightSensor' in window) {
        let lightSensor = new AmbientLightSensor();
        lightSensor.addEventListener('reading', () => {
            lightIntensity = lightSensor.illuminance || 0;
            maxLight = Math.max(maxLight, lightIntensity);
        });
        lightSensor.start();
    }
    
    if ($('imu-status') && !('Accelerometer' in window)) {
        $('imu-status').textContent = 'INACTIF (API IMU non supportée)';
    }
}


// --- FILTRE EKF AVANCÉ (9 États - Cœur) ---

/** Implémentation du Filtre de Kalman Étendu (EKF) 9 états (simplifié). */
function updateEKF(coords, speedMeasure3D, accuracy_m, dt) {
    const altMeasure = coords.altitude || altEst;
    
    // 1. DÉTECTION ZUPT (Zero Velocity Update) - Correction des Biais
    const imuTotalAccel = Math.sqrt((imuAccelX || 0)**2 + (imuAccelY || 0)**2 + (imuAccelZ || G_BASE)**2);
    const isStationary = Math.abs(imuTotalAccel - G_BASE) < ZUPT_ACCEL_TOLERANCE && (speedEst < MIN_SPD * 5); 

    if (isStationary) {
        // ZUPT Correction: Force speed to 0 et réduit l'incertitude
        speedEst = 0;
        P_spd = 0.1; 
        
        // Correction des Biais de l'Accéléromètre
        const R_bias_acc = 0.1;
        const K_bias_acc = P_biasAccel / (P_biasAccel + R_bias_acc);
        biasAccelX = biasAccelX + K_bias_acc * ((imuAccelX || 0) - biasAccelX);
        P_biasAccel = (1 - K_bias_acc) * P_biasAccel;
        
        // Correction des Biais du Gyroscope
        const K_bias_gyro = P_biasGyro / (P_biasGyro + R_bias_acc);
        biasGyroZ = biasGyroZ + K_bias_gyro * ((imuGyroZ || 0) - biasGyroZ);
        P_biasGyro = (1 - K_bias_gyro) * P_biasGyro;
        
        if ($('zupt-status')) $('zupt-status').textContent = 'ZUPT ACTIF (Biases corrigés)';
    } else {
        if ($('zupt-status')) $('zupt-status').textContent = 'N/A';
    }

    // 2. PRÉDICTION : Utilise l'accélération IMU CORRIGÉE
    const accelX_corr = (imuAccelX || 0) - biasAccelX;
    const accelY_corr = (imuAccelY || 0) - biasAccelY;
    
    const imuHorizontalAccel_corr = Math.sqrt(accelX_corr**2 + accelY_corr**2);
    const speedPred = speedEst + imuHorizontalAccel_corr * dt; 
    
    // Propagation de l'incertitude
    P_spd = P_spd + Q_IMU * dt; 
    P_biasAccel = P_biasAccel + Q_BIAS_ACC * dt;
    P_biasGyro = P_biasGyro + Q_BIAS_GYRO * dt;

    // 3. CORRECTION : Correction de Dérive GNSS.
    const R_spd = calculateR(accuracy_m);
    let K_spd = P_spd / (P_spd + R_spd);
    
    // Correction de la vitesse
    const speedInnovation = speedMeasure3D - speedPred;
    speedEst = speedPred + K_spd * speedInnovation; 
    P_spd = (1 - K_spd) * P_spd;
    
    // Correction des Biais (Propagation de la Correction de Vitesse)
    const K_bias_from_spd = K_spd * 0.01; 
    biasAccelX = biasAccelX + K_bias_from_spd * accelX_corr * speedInnovation;

    // EKF Altitude
    const altPred = altEst;
    P_alt = P_alt + Q_ALT * dt;
    const R_alt = R_spd; 
    let K_alt = P_alt / (P_alt + R_alt);
    altEst = altPred + K_alt * (altMeasure - altPred);
    P_alt = (1 - K_alt) * P_alt;
    
    // Mise à jour des compteurs et états globaux
    lastLat = lat; lastLon = lon;
    lat = coords.latitude || lat;
    lon = coords.longitude || lon;
    accuracyGPS = accuracy_m;
    
    if (speedEst >= MIN_SPD) {
        distM_3D += speedEst * dt;
        timeMoving += dt;
        maxSpd = Math.max(maxSpd, speedEst * KMH_MS);
    }
    
    const localGravity = calculateLocalGravity(lat * D2R, altEst);
    const airDensity = (pressurehPa !== null && tempC !== null) ? (pressurehPa * 100) / (R_AIR * (tempC + KELVIN_OFFSET)) : null;
    
    // Les fonctions d'affichage sont appelées ici mais définies dans app_flow_dom.js
    updateAdvancedPhysics(speedEst, localGravity, airDensity);
    
    totalTime = (Date.now() - startTime) / 1000;
    const speedAvgTotal = totalTime > 0 ? (distM_3D / totalTime) : 0;
    const speedAvgMoving = timeMoving > 0 ? (distM_3D / timeMoving) : 0;
    
    updateGPSDisplay(coords, speedMeasure3D, accuracy_m, speedAvgMoving, speedAvgTotal);
    updateEKFDisplay(R_spd, P_spd, altEst, P_alt, localGravity);
    
    // Affichage des biais EKF (Debug)
    if ($('bias-accel-x')) $('bias-accel-x').textContent = `${biasAccelX.toPrecision(3)} m/s²`;
    if ($('bias-gyro-z')) $('bias-gyro-z').textContent = `${biasGyroZ.toPrecision(3)} rad/s`;
    }
// =================================================================
// 3/3. APPLICATION FLOW & DOM UPDATES
// (Dépend de: constants_globals.js, core_logic_ekf.js)
// =================================================================

// --- LOGIQUE D'AFFICHAGE EKF / AVANCÉE ---

function updateEKFDisplay(R_spd, P_spd, altEst, P_alt, localGravity) {
    if ($('speed-stable')) $('speed-stable').textContent = `${(speedEst * KMH_MS).toFixed(1)} km/h`;
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${speedEst.toFixed(2)} m/s`;
    if ($('alt-ekf')) $('alt-ekf').textContent = altEst !== null ? `${altEst.toFixed(2)} m` : 'N/A';
    
    if ($('kalman-uncert')) $('kalman-uncert').textContent = `${P_spd.toPrecision(3)} (m/s)²`;
    if ($('alt-uncertainty')) $('alt-uncertainty').textContent = `${Math.sqrt(P_alt).toPrecision(3)} m`;
    if ($('speed-error-perc')) $('speed-error-perc').textContent = `${R_spd.toPrecision(3)}`;
    if ($('gravity-local')) $('gravity-local').textContent = `${localGravity.toFixed(5)} m/s²`;
    
    const percSpeedC = (speedEst / C_L) * 100;
    if ($('perc-speed-c')) $('perc-speed-c').textContent = `${percSpeedC.toExponential(2)} %`;

    if ($('distance-horizon')) $('distance-horizon').textContent = altEst !== null ? `${calculateHorizonDistance(altEst).toFixed(0)} m` : 'N/A';
}

function updateGPSDisplay(coords, speedMeasure3D, accuracy_m, speedAvgMoving, speedAvgTotal) {
    const latLonDisplay = (val) => val !== null ? val.toFixed(6) : 'N/A';
    if ($('lat-display')) $('lat-display').textContent = latLonDisplay(lat);
    if ($('lon-display')) $('lon-display').textContent = latLonDisplay(lon);
    if ($('gps-precision')) $('gps-precision').textContent = accuracy_m !== null ? `${accuracy_m.toFixed(2)} m` : 'N/A';
    
    if ($('speed-3d-inst')) $('speed-3d-inst').textContent = `${(speedMeasure3D * KMH_MS).toFixed(1)} km/h`;
    if ($('speed-raw-ms')) $('speed-raw-ms').textContent = `${(coords.speed || 0).toFixed(2)} m/s`;

    if ($('speed-max')) $('speed-max').textContent = `${maxSpd.toFixed(1)} km/h`;
    if ($('speed-avg-moving')) $('speed-avg-moving').textContent = `${(speedAvgMoving * KMH_MS).toFixed(1)} km/h`;
    if ($('speed-avg-total')) $('speed-avg-total').textContent = `${(speedAvgTotal * KMH_MS).toFixed(1)} km/h`;
    
    const distKM = distM_3D / 1000;
    if ($('distance-total-km')) $('distance-total-km').textContent = `${distKM.toFixed(3)} km | ${distM_3D.toFixed(0)} m`;
    
    // Distance lumière
    const distLightS = distM_3D / C_L;
    if ($('distance-light-s')) $('distance-light-s').textContent = `${distLightS.toExponential(2)} s`;
    if ($('distance-light-min')) $('distance-light-min').textContent = `${(distLightS / 60).toExponential(2)} min`;
    if ($('distance-light-h')) $('distance-light-h').textContent = `${(distLightS / 3600).toExponential(2)} h`;
    if ($('distance-light-j')) $('distance-light-j').textContent = `${(distLightS / 86400).toExponential(2)} j`;
    
    // Mises à jour de la carte
    if (map && lat !== null && lon !== null) {
        marker.setLatLng([lat, lon]);
        if (track.length === 0 || lastLat !== lat || lastLon !== lon) {
            track.push([lat, lon]);
            trackPolyline.setLatLngs(track);
            map.panTo([lat, lon]);
        }
    }
}

function updateAdvancedPhysics(speedEst, localGravity, airDensity) {
    const tempK = (tempC !== null) ? tempC + KELVIN_OFFSET : null;
    const speedOfSound = (tempK !== null) ? Math.sqrt(1.4 * R_AIR * tempK) : null;
    
    const lorentzFactor = 1 / Math.sqrt(1 - Math.pow(speedEst / C_L, 2));

    // Relativité
    const timeDilationVelocity = (lorentzFactor - 1) * 86400 * 1e9; 
    const restMassEnergy = currentMass * C_L * C_L;
    const relativisticEnergy = lorentzFactor * restMassEnergy;
    const schwarzschildRadius = 2 * localGravity * currentMass / Math.pow(C_L, 2);

    // Dynamique
    const dynamicPressure = (airDensity !== null) ? 0.5 * airDensity * speedEst * speedEst : null;
    const dragForce = (dynamicPressure !== null) ? dynamicPressure * 1 * 0.5 : null; 
    const coriolisForce = 2 * currentMass * (7.2921e-5) * speedEst * Math.sin((lat || 0) * D2R); 

    // Affichage des données avancées
    if ($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = speedOfSound !== null ? `${speedOfSound.toFixed(2)} m/s` : 'N/A';
    if ($('mach-number')) $('mach-number').textContent = speedOfSound !== null ? `${(speedEst / speedOfSound).toFixed(4)}` : 'N/A';
    if ($('time-dilation-velocity')) $('time-dilation-velocity').textContent = `${timeDilationVelocity.toFixed(2)} ns/j`;
    if ($('energy-relativistic')) $('energy-relativistic').textContent = `${relativisticEnergy.toExponential(2)} J`;
    if ($('schwarzschild-radius')) $('schwarzschild-radius').textContent = `${schwarzschildRadius.toExponential(2)} m`;
    if ($('dynamic-pressure')) $('dynamic-pressure').textContent = dynamicPressure !== null ? `${dynamicPressure.toFixed(2)} Pa` : 'N/A';
    if ($('drag-force')) $('drag-force').textContent = dragForce !== null ? `${dragForce.toFixed(2)} N` : 'N/A';
    if ($('coriolis-force')) $('coriolis-force').textContent = `${coriolisForce.toExponential(2)} N`;
    if ($('lorentz-factor')) $('lorentz-factor').textContent = `${lorentzFactor.toFixed(4)}`;
}

// --- GESTION DE LA CARTE, MÉTÉO, ASTRO ---

function initMap() {
    const initialLat = lat || 43.284540; 
    const initialLon = lon || 5.358558; 
    
    if ($('map') && !map && typeof L !== 'undefined') {
        map = L.map('map').setView([initialLat, initialLon], 14);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { 
            attribution: '© OpenStreetMap contributors',
            maxZoom: 19 
        }).addTo(map);

        marker = L.marker([initialLat, initialLon]).addTo(map);
        trackPolyline = L.polyline(track, {color: '#00ff66', weight: 3}).addTo(map);
    }
}

function fetchWeatherAndBaro() {
    // Les variables restent à null (N/A) car aucune API externe ou capteur n'est implémenté pour ces données.
    pressurehPa = null;  
    tempC = null;             
    humidityPerc = null;      
    altBaro = null;
    
    const airDensity = null;
    const dewPoint = null; 

    const displayN_A = (val) => val !== null ? val.toFixed(1) + ' °C' : 'N/A';
    const displayN_A_kg = (val) => val !== null ? val.toFixed(3) + ' kg/m³' : 'N/A';
    
    if ($('temp-air-2')) $('temp-air-2').textContent = displayN_A(tempC);
    if ($('pressure-2')) $('pressure-2').textContent = pressurehPa !== null ? pressurehPa.toFixed(0) + ' hPa' : 'N/A';
    if ($('humidity-2')) $('humidity-2').textContent = humidityPerc !== null ? humidityPerc.toFixed(0) + ' %' : 'N/A';
    if ($('air-density')) $('air-density').textContent = displayN_A_kg(airDensity);
    if ($('dew-point')) $('dew-point').textContent = displayN_A(dewPoint);
    if ($('alt-baro')) $('alt-baro').textContent = altBaro !== null ? altBaro.toFixed(1) + ' m' : 'N/A';
    if ($('alt-corrected-baro')) $('alt-corrected-baro').textContent = altBaro !== null ? altBaro.toFixed(1) + ' m' : 'N/A';
    
    if ($('temp-wet-bulb')) $('temp-wet-bulb').textContent = 'N/A';
    if ($('cape-index')) $('cape-index').textContent = 'N/A';
}

function updateAstro() {
    // Logique SunCalc (omis)
}

// --- GESTION GPS & FLUX D'APPLICATION ---

/** Gestionnaire de succès GPS. */
function gpsSuccess(position) {
    const dt = DOM_FAST_UPDATE_MS / 1000;
    
    const speedRaw = position.coords.speed || 0; 
    const newAccuracy = gpsAccuracyOverride || position.coords.accuracy;
    
    const altChange = (position.coords.altitude || altEst) - altEst;
    const speedVertical = altChange / dt; 
    const speedMeasure3D = Math.sqrt(speedRaw * speedRaw + speedVertical * speedVertical);

    updateEKF(position.coords, speedMeasure3D, newAccuracy, dt); 
    if ($('gps-status-dr')) $('gps-status-dr').textContent = 'ACTIF (FUSION IMU/GPS)';
    if ($('speed-status-text')) $('speed-status-text').textContent = 'Fusion EKF/IMU Active';
}

/** Gestionnaire d'erreur GPS (Active le Dead Reckoning IMU). */
function gpsError(error) {
    const imuStatus = (!('Accelerometer' in window) && !('Gyroscope' in window)) ? 'IMU NON DISPONIBLE' : 'IMU Seul (DR)';
    
    if ($('gps-status-dr')) $('gps-status-dr').textContent = `ERREUR GPS - DR: ${imuStatus}`;
    if ($('speed-status-text')) $('speed-status-text').textContent = `Mode Dead Reckoning (${imuStatus})`;
    
    const dt = DOM_FAST_UPDATE_MS / 1000;
    // L'EKF est mis à jour avec la dernière position connue et une incertitude maximale R_MAX.
    updateEKF({latitude: lat, longitude: lon, altitude: altEst}, speedEst, R_MAX, dt); 
}

function setGPSMode(mode) {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    wID = navigator.geolocation.watchPosition(gpsSuccess, gpsError, { 
        enableHighAccuracy: true, 
        timeout: mode === 'HIGH_FREQ' ? 1000 : 5000, 
        maximumAge: 0 
    });
}

function stopGPS() {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    wID = null;
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = '▶️ MARCHE GPS';
    if ($('gps-status-dr')) $('gps-status-dr').textContent = 'INACTIF (Arrêté)';
}

/** Boucle de mise à jour rapide (50ms). */
function domUpdateLoop() {
    const dt = DOM_FAST_UPDATE_MS / 1000;
    
    // Mises à jour des données IMU
    if ($('accel-x')) $('accel-x').textContent = displayVal(imuAccelX, ' m/s²');
    if ($('accel-y')) $('accel-y').textContent = displayVal(imuAccelY, ' m/s²');
    if ($('accel-z')) $('accel-z').textContent = imuAccelZ !== null ? imuAccelZ.toFixed(3) + ' m/s²' : 'N/A';
    
    // Gyroscope
    if ($('gyro-x')) $('gyro-x').textContent = displayVal(imuGyroX, ' rad/s');
    if ($('gyro-y')) $('gyro-y').textContent = displayVal(imuGyroY, ' rad/s');
    if ($('gyro-z')) $('gyro-z').textContent = displayVal(imuGyroZ, ' rad/s');
    
    // Si le GPS est arrêté ou non disponible, on force la logique Dead Reckoning
    if (wID === null) {
        gpsError(null); 
    }

    const now = getCDate();
    totalTime = (now.getTime() - startTime) / 1000;
    
    // Affichage du temps
    if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString('fr-FR', { hour12: false });
    if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
    if ($('elapsed-time')) $('elapsed-time').textContent = totalTime.toFixed(2) + ' s';
    if ($('time-moving')) $('time-moving').textContent = timeMoving.toFixed(2) + ' s';
    
    const mc_hours = Math.floor((now.getHours() * 1000 + now.getMinutes() * 100 / 6) / 1000) % 24;
    const mc_minutes = Math.floor((now.getMinutes() * 60) / 1000) * 10;
    if ($('time-minecraft')) $('time-minecraft').textContent = `${mc_hours.toString().padStart(2, '0')}:${mc_minutes.toString().padStart(2, '0')}`;

    // CALCUL DE DEBUG : Fréquence de Nyquist
    const nyquistFreq = 1 / (2 * dt); 
    if ($('nyquist-frequency')) {
        $('nyquist-frequency').textContent = `${nyquistFreq.toFixed(1)} Hz`;
    }
}

/** Initialise l'application et configure les boucles de mesure et d'écoute. */
function init() {
    if (!('geolocation' in navigator)) {
        console.error("La géolocalisation n'est pas supportée. Le tableau de bord ne peut fonctionner.");
    }
    
    initializeSensors(); 
    initMap();
    fetchWeatherAndBaro(); 
    updateAstro();
    setGPSMode('HIGH_FREQ');
    
    // Boucle rapide (IMU + DOM)
    setInterval(domUpdateLoop, DOM_FAST_UPDATE_MS); 

    // Boucle lente (Météo + Astro)
    setInterval(() => {
        fetchWeatherAndBaro(); 
        updateAstro();
    }, DOM_SLOW_UPDATE_MS);

    // TODO: Ajouter les listeners d'événements pour les boutons de contrôle
}

document.addEventListener('DOMContentLoaded', init);
