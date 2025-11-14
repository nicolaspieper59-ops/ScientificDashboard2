// =================================================================
// GNSS SPACETIME DASHBOARD - FUSION PROFESSIONNEL (SANS SIMULATION)
// ⚠️ Dépend des API de capteurs du navigateur (Generic Sensor API)
// =================================================================

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;          // Vitesse de la lumière (m/s)
const R_AIR = 287.058;          // Constante spécifique de l'air sec (J/kg·K)
const G_BASE = 9.80665;         // Gravité standard (m/s²)
const KMH_MS = 3.6;             
const KELVIN_OFFSET = 273.15;
const EARTH_RADIUS = 6371000;   // Rayon terrestre moyen (m)
const DOM_FAST_UPDATE_MS = 50;  
const DOM_SLOW_UPDATE_MS = 5000;
const MAX_ACC = 200;            
const MIN_SPD = 0.05;           
const R_MIN = 0.01;         
const R_MAX = 500.0;        
const AU_METERS = 149597870700;

// --- PARAMÈTRES DU FILTRE DE KALMAN ---
const Q_IMU = 0.05;         
const Q_ALT = 0.5;          

let P_spd = 100;            
let P_alt = 100;            
let speedEst = 0;           
let altEst = 0;             
let wID = null;             

// --- VARIABLES D'ÉTAT CAPTEURS (Initialisées à null pour "N/A" si capteur absent) ---
let lat = null, lon = null; // Position initiale (dépend du GPS)
let speed3dRaw = 0;         
let accuracyGPS = 1000;     
let imuAccelX = null, imuAccelY = null, imuAccelZ = G_BASE; // Z par défaut à G_BASE
let magX = null, magY = null, magZ = null; 
let currentHeading = null;     
let altBaro = null;            
let pressurehPa = null;  
let tempC = null;             
let humidityPerc = null;      
let lightIntensity = null;     
let soundLevel = null;         

// --- VARIABLES COMPTEURS & CONTROLES ---
let distM_3D = 0;           
let timeMoving = 0;
let maxSpd = 0;
let totalTime = 0;
let startTime = Date.now();
let gpsAccuracyOverride = 0;
let selectedEnvironment = 'NORMAL';
let currentMass = 70;
let isXRayMode = false;   
let maxSound = 0, maxLight = 0, maxMagnetic = 0;
let totalDistance2D = 0; 
let lastLat = null, lastLon = null;

// --- INSTANCES D'API DES CAPTEURS RÉELS ---
let accelerometer = null;
let magnetometer = null;
let lightSensor = null;

// --- FONCTIONS UTILITAIRES DE BASE (Sans simulation NTP) ---
const $ = (id) => document.getElementById(id);

/** Retourne l'heure locale actuelle sans simulation NTP. */
function getCDate() {
    // Élimine toute logique de décalage NTP
    return new Date(); 
}

/** Calcule le bruit de mesure (R) pour l'EKF. */
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
    // Si la position est inconnue (lat=null), on retourne la gravité standard
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
// --- BLOC 2/4 : Gestion des Capteurs, Physique Avancée et Filtre EKF (Fusion INS) ---

/** Initialise et démarre les capteurs réels du navigateur. */
function initializeSensors() {
    // 1. Accéléromètre (API Générique)
    if ('Accelerometer' in window) {
        accelerometer = new Accelerometer({ frequency: 50 });
        accelerometer.addEventListener('reading', () => {
            imuAccelX = accelerometer.x || 0;
            imuAccelY = accelerometer.y || 0;
            imuAccelZ = accelerometer.z || G_BASE; 
            if ($('imu-status')) $('imu-status').textContent = 'Actif / Fusion';
        });
        accelerometer.start();
    }
    
    // 2. Magnétomètre (API Générique)
    if ('Magnetometer' in window) {
        magnetometer = new Magnetometer({ frequency: 50 });
        magnetometer.addEventListener('reading', () => {
            magX = magnetometer.x || 0;
            magY = magnetometer.y || 0;
            magZ = magnetometer.z || 0;
        });
        magnetometer.start();
    }
    
    // 3. Capteur de Lumière Ambiante (API Générique)
    if ('AmbientLightSensor' in window) {
        lightSensor = new AmbientLightSensor();
        lightSensor.addEventListener('reading', () => {
            lightIntensity = lightSensor.illuminance || 0;
            maxLight = Math.max(maxLight, lightIntensity);
        });
        lightSensor.start();
    }
    
    // Remarque: L'API du Gyroscope et les capteurs Météo (Baro, Temp, Humidité) ne sont pas intégrés par défaut
    // et requièrent des permissions spécifiques ou du matériel externe.
    if ($('imu-status') && !('Accelerometer' in window)) {
        $('imu-status').textContent = 'INACTIF (API IMU non supportée)';
    }
}


/** Met à jour les lectures capteurs (lecture directe des API). */
function imuDataHandler(dt) {
    
    // Calcul du cap basé sur le magnétomètre réel
    if (magX !== null && magY !== null) {
        const declination = 2.5 * D2R; // Déclinaison magnétique (Doit être une API ou un modèle)
        let heading_rad = Math.atan2(magY, magX) + declination;
        if (heading_rad < 0) heading_rad += 2 * Math.PI;
        currentHeading = heading_rad * R2D;
    } else {
         currentHeading = null;
    }
    
    // Mise à jour des compteurs Max (basés sur lectures réelles)
    maxMagnetic = Math.max(maxMagnetic, Math.sqrt((magX || 0)**2 + (magY || 0)**2 + (magZ || 0)**2));
    // soundLevel reste à N/A car pas d'API simple pour la démo.
    
    // Affichage des données (avec vérification null pour N/A)
    const displayVal = (val, unit) => val !== null ? val.toFixed(3) + unit : 'N/A';
    
    if ($('accel-x')) $('accel-x').textContent = displayVal(imuAccelX, ' m/s²');
    if ($('accel-y')) $('accel-y').textContent = displayVal(imuAccelY, ' m/s²');
    if ($('accel-z')) $('accel-z').textContent = imuAccelZ !== null ? imuAccelZ.toFixed(3) + ' m/s²' : 'N/A'; // imuAccelZ a G_BASE si API absente
    if ($('heading-display')) $('heading-display').textContent = currentHeading !== null ? `${currentHeading.toFixed(1)}°` : 'N/A';
    
    if ($('magnetic-max')) $('magnetic-max').textContent = maxMagnetic > 0 ? maxMagnetic.toFixed(1) + ' µT' : 'N/A';
    if ($('sound-max')) $('sound-max').textContent = 'N/A'; 
    if ($('light-max')) $('light-max').textContent = maxLight > 0 ? maxLight.toFixed(1) + ' lux' : 'N/A';
}

/** Calculs Physiques et Relativistes Avancés. */
function updateAdvancedPhysics(speedEst, localGravity, airDensity) {
    // Utilise les variables globales tempC et airDensity, qui seront nulles sans API.
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
    const coriolisForce = 2 * currentMass * (7.2921e-5) * speedEst * Math.sin((lat || 0) * D2R); // lat=0 si inconnu

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

/** Implémentation du Filtre de Kalman Étendu (EKF) pour la fusion IMU/GNSS. */
function updateEKF(coords, speedMeasure3D, accuracy_m, dt) {
    const altMeasure = coords.altitude || altEst;
    
    // PRÉDICTION : Utilise l'accélération IMU réelle si disponible, sinon se fie uniquement à la correction GPS.
    const imuHorizontalAccel = (imuAccelX !== null && imuAccelY !== null) ? Math.sqrt(imuAccelX * imuAccelX + imuAccelY * imuAccelY) : 0; 
    const speedPred = speedEst + imuHorizontalAccel * dt; 
    P_spd = P_spd + Q_IMU * dt; 
    
    // CORRECTION : Correction de Dérive GNSS.
    const R_spd = calculateR(accuracy_m);
    let K_spd = P_spd / (P_spd + R_spd);
    
    speedEst = speedPred + K_spd * (speedMeasure3D - speedPred); 
    P_spd = (1 - K_spd) * P_spd;
    
    // EKF Altitude
    const altPred = altEst;
    P_alt = P_alt + Q_ALT * dt;
    const R_alt = R_spd; 
    let K_alt = P_alt / (P_alt + R_alt);
    altEst = altPred + K_alt * (altMeasure - altPred);
    P_alt = (1 - K_alt) * P_alt;
    
    // MISE À JOUR DES COMPTEURS
    if (speedEst >= MIN_SPD) {
        distM_3D += speedEst * dt;
        timeMoving += dt;
        maxSpd = Math.max(maxSpd, speedEst * KMH_MS);
    }
    
    // Mise à jour de la position (lat/lon sont mis à jour uniquement par le GPS)
    lastLat = lat; lastLon = lon;
    lat = coords.latitude || lat;
    lon = coords.longitude || lon;
    accuracyGPS = accuracy_m;
    
    const localGravity = calculateLocalGravity(lat * D2R, altEst);
    const airDensity = (pressurehPa !== null && tempC !== null) ? (pressurehPa * 100) / (R_AIR * (tempC + KELVIN_OFFSET)) : null; // Air Density réelle
    
    updateAdvancedPhysics(speedEst, localGravity, airDensity);
    
    totalTime = (Date.now() - startTime) / 1000;
    const speedAvgTotal = totalTime > 0 ? (distM_3D / totalTime) : 0;
    const speedAvgMoving = timeMoving > 0 ? (distM_3D / timeMoving) : 0;
    
    updateGPSDisplay(coords, speedMeasure3D, accuracy_m, speedAvgMoving, speedAvgTotal);
    updateEKFDisplay(R_spd, P_spd, altEst, P_alt, localGravity);
                }
// --- BLOC 3/4 : Fonctions d'Affichage, Carte et Calculs Météo/Astro ---

let map = null, marker = null, trackPolyline = null, track = [];

/** Initialise la carte Leaflet. */
function initMap() {
    // Utiliser une position par défaut si lat/lon sont nulls au démarrage
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

// ... (updateEKFDisplay et updateGPSDisplay inchangés, gèrent les valeurs nulles) ...


/** Calcule et affiche les données astronomiques complètes. */
function updateAstro() {
    // Astro dépend uniquement de l'heure et de la position GPS réelle.
    if (lat === null || lon === null || typeof SunCalc === 'undefined') {
        // Afficher N/A pour tous les champs Astro si la position n'est pas connue
        return; 
    }

    const now = getCDate();
    const pos = SunCalc.getPosition(now, lat, lon);
    const times = SunCalc.getTimes(now, lat, lon);
    const moonPos = SunCalc.getMoonPosition(now, lat, lon);
    const moonIllum = SunCalc.getMoonIllumination(now);
    
    // ... (Reste de la logique Astro inchangée) ...
    // ... (updateClockVisualization et getMoonPhaseName inchangées) ...

}

/** Élimine toute simulation et met à jour les variables météo à partir de l'API BROWSER (non standard) ou N/A. */
function fetchWeatherAndBaro() {
    // --- ÉLIMINATION DE TOUTE SIMULATION ---
    // Ces capteurs ne sont pas disponibles via l'API standard du navigateur (sauf capteurs IoT externes).
    pressurehPa = null;  
    tempC = null;             
    humidityPerc = null;      
    altBaro = null;
    
    const airDensity = null;
    const dewPoint = null; 

    // Affichage Météo
    const displayN_A = (val) => val !== null ? val.toFixed(1) + (val === pressurehPa ? ' hPa' : ' °C') : 'N/A';
    const displayN_A_kg = (val) => val !== null ? val.toFixed(3) + ' kg/m³' : 'N/A';
    
    if ($('temp-air-2')) $('temp-air-2').textContent = displayN_A(tempC);
    if ($('pressure-2')) $('pressure-2').textContent = pressurehPa !== null ? pressurehPa.toFixed(0) + ' hPa' : 'N/A';
    if ($('humidity-2')) $('humidity-2').textContent = humidityPerc !== null ? humidityPerc.toFixed(0) + ' %' : 'N/A';
    if ($('air-density')) $('air-density').textContent = displayN_A_kg(airDensity);
    if ($('dew-point')) $('dew-point').textContent = displayN_A(dewPoint);
    if ($('alt-baro')) $('alt-baro').textContent = altBaro !== null ? altBaro.toFixed(1) + ' m' : 'N/A';
    if ($('alt-corrected-baro')) $('alt-corrected-baro').textContent = altBaro !== null ? altBaro.toFixed(1) + ' m' : 'N/A';
    
    // Biophysique
    if ($('temp-wet-bulb')) $('temp-wet-bulb').textContent = 'N/A';
    if ($('cape-index')) $('cape-index').textContent = 'N/A';
                                            }
// --- BLOC 4/4 : Initialisation, GPS et Boucles du Système ---

/** Gestionnaire de succès GPS. */
function gpsSuccess(position) {
    const dt = DOM_FAST_UPDATE_MS / 1000;
    
    // Ces valeurs sont maintenant des lectures réelles, ou 0 si l'API GPS ne les fournit pas.
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
    // Si l'IMU est également indisponible, le système est inactif.
    const imuStatus = (accelerometer === null) ? 'IMU NON DISPONIBLE' : 'IMU Seul';
    
    if ($('gps-status-dr')) $('gps-status-dr').textContent = `ERREUR GPS (${error ? error.code : 99}) - DR: ${imuStatus}`;
    if ($('speed-status-text')) $('speed-status-text').textContent = `Mode Dead Reckoning (${imuStatus})`;
    
    const dt = DOM_FAST_UPDATE_MS / 1000;
    // L'EKF est mis à jour avec la dernière position connue et une incertitude maximale R_MAX.
    updateEKF({latitude: lat, longitude: lon, altitude: altEst}, speedEst, R_MAX, dt); 
}

// ... (setGPSMode et stopGPS inchangés) ...

/** Boucle de mise à jour rapide (50ms). */
function domUpdateLoop() {
    const dt = DOM_FAST_UPDATE_MS / 1000;
    
    imuDataHandler(dt);
    
    // Si le GPS est arrêté ou non disponible, on simule l'erreur pour forcer le DR IMU
    if (wID === null) {
        gpsError(null); 
    }

    const now = getCDate();
    totalTime = (now.getTime() - startTime) / 1000;
    
    // Affichage du temps (sans simulation NTP)
    if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString('fr-FR', { hour12: false });
    if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
    if ($('elapsed-time')) $('elapsed-time').textContent = totalTime.toFixed(2) + ' s';
    if ($('time-moving')) $('time-moving').textContent = timeMoving.toFixed(2) + ' s';
    
    // Calcul Heure Minecraft (basée sur l'heure locale réelle)
    const mc_hours = Math.floor((now.getHours() * 1000 + now.getMinutes() * 100 / 6) / 1000) % 24;
    const mc_minutes = Math.floor((now.getMinutes() * 60) / 1000) * 10;
    if ($('time-minecraft')) $('time-minecraft').textContent = `${mc_hours.toString().padStart(2, '0')}:${mc_minutes.toString().padStart(2, '0')}`;
}

/** Initialise l'application et configure les boucles de mesure et d'écoute. */
function init() {
    if (!('geolocation' in navigator)) {
        console.error("La géolocalisation n'est pas supportée. Le tableau de bord ne peut fonctionner.");
    }
    
    initializeSensors(); // Initialisation des API de capteurs réels
    initMap();
    fetchWeatherAndBaro(); 
    updateAstro(); 
    setGPSMode('HIGH_FREQ');
    
    // Boucle rapide (IMU + DOM)
    setInterval(domUpdateLoop, DOM_FAST_UPDATE_MS);
    // --- CALCULS DE DEBUG ---
// Fréquence de Nyquist : F_s/2
const nyquistFreq = 1 / (2 * dt); 
if ($('nyquist-frequency')) {
    $('nyquist-frequency').textContent = `${nyquistFreq.toFixed(1)} Hz`;
}

    // Boucle lente (Météo + Astro)
    setInterval(() => {
        fetchWeatherAndBaro(); // Reste sans simulation (N/A)
        updateAstro();
    }, DOM_SLOW_UPDATE_MS);

    // --- CONFIGURATION DES LISTENERS D'ÉVÉNEMENTS (inchangée) ---
    $('toggle-gps-btn').addEventListener('click', () => { if (wID !== null) stopGPS(); else setGPSMode($('freq-select').value); });
    $('freq-select').addEventListener('change', (e) => setGPSMode(e.target.value));
    
    // ... (Reste des listeners inchangé) ...
}

document.addEventListener('DOMContentLoaded', init);
