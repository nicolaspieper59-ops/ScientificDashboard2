// =================================================================
// SEGMENT 1/5 : UTILS, CONSTANTES ET ÉTAT GLOBAL (IIFE START)
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        return (decimals === 0 ? '0' : '0.00') + suffix;
    }
    return val.toFixed(decimals) + suffix;
};
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || val === Infinity || val === -Infinity) {
        const zeroDecimals = '0.' + Array(decimals).fill('0').join('');
        return zeroDecimals + 'e+0' + suffix;
    }
    return val.toExponential(decimals) + suffix;
};
const getCDate = () => lastNTPDate || new Date();

// --- CONSTANTES CRITIQUES ---
const C_L = 299792458; // Vitesse de la lumière (m/s)
const G_U = 6.67430e-11; // Constante gravitationnelle universelle
const R2D = 180 / Math.PI; // Radians to Degrees
const D2R = Math.PI / 180; // Degrees to Radians
const KMH_MS = 3.6; // m/s to km/h
const MIN_SPD = 0.5; // Vitesse minimale pour être considéré en mouvement (m/s)
const MAX_ACC = 50; // Précision GPS maximale acceptable (m)

// Constantes de temps et de mise à jour
const IMU_UPDATE_RATE_MS = 20; // 50 Hz pour la boucle rapide (UKF Predict)
const DOM_SLOW_UPDATE_MS = 1000; // 1 Hz pour Astro/Météo/NTP
const MAP_UPDATE_INTERVAL = 3000; // 3 secondes
const STANDBY_TIMEOUT_MS = 30000; // 30 secondes pour passer en LOW_FREQ
const MIN_DT = 0.005; 

// Constantes atmosphériques ISA (par défaut si hors ligne)
const TEMP_SEA_LEVEL_K = 288.15; 
const RHO_SEA_LEVEL = 1.225; 
const BARO_ALT_REF_HPA = 1013.25; 
let currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound = 340.29; 
let lastT_K = TEMP_SEA_LEVEL_K;
let lastP_hPa = BARO_ALT_REF_HPA;

// Constantes GPS et UKF
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 5000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 30000, timeout: 60000 }
};
const UKF_R_MAX = 1000.0; 
const R_ALT_MIN = 1.0; 
const ZUPT_RAW_THRESHOLD = 0.3; 
const ZUPT_ACCEL_THRESHOLD = 0.1; 

const ENVIRONMENT_FACTORS = {
    NORMAL: { R_MULT: 1.0, DISPLAY: 'Normal' },
    CITY: { R_MULT: 3.0, DISPLAY: 'Urbain Droit' },
    TUNNEL: { R_MULT: 10.0, DISPLAY: 'Tunnel/Indoor' },
    SEA: { R_MULT: 0.5, DISPLAY: 'Mer/Plaine' },
    GROTTO: { R_MULT: 100.0, DISPLAY: 'Grotte/No-GPS' }, 
};
let selectedEnvironment = 'NORMAL';
let currentUKFReactivity = 'AUTOMATIC'; // Correspond à l'ID 'ukf-reactivity-mode'

// Constantes du Modèle Gravitationnel
let G_ACC = 9.80665; 
let R_ALT_CENTER_REF = 6378137.0; 
let currentCelestialBody = 'EARTH';
let rotationRadius = 100.0;
let angularVelocity = 0.0;

// --- ÉTAT GLOBAL ET VARIABLES D'INSTANCE ---
let ukf = null; 
let wID = null; 
let domFastID = null; 
let domSlowID = null; 
let gpsStandbyTimeoutID = null; 
let sTime = null; 
let lastNTPDate = null; 
let lastGPSPos = null; 
let lPos = null; 
let lat = null, lon = null, kAlt = null, kSpd = 0; 
let kUncert = UKF_R_MAX, kAltUncert = 10; 
let distM = 0; 
let maxSpd = 0; 
let timeMoving = 0; 
let timeTotal = 0; 
let currentGPSMode = 'HIGH_FREQ';
let gpsAccuracyOverride = 0.0; 
let emergencyStopActive = false;
let distanceRatioMode = false; 
let R_FACTOR_RATIO = 1.0; 
let currentMass = 70.0; 
let sunAltitudeRad = 0; 

// --- ÉTAT DES CAPTEURS ---
let accel = { x: 0, y: 0, z: 0 };
let gyro = { x: 0, y: 0, z: 0 };
let mag = { x: 0, y: 0, z: 0 };
let lastIMUTimestamp = performance.now();
let imuActive = false;

// --- CARTE LEAFLET ---
let map = null;
let marker = null;
let circle = null;
let lastMapUpdate = 0;

// =================================================================
// DÉMARRAGE : Encapsulation de la logique UKF et État Global (IIFE)
// =================================================================
((window) => {
    // Vérification des dépendances critiques
    if (typeof math === 'undefined' || typeof L === 'undefined' || typeof SunCalc === 'undefined' || typeof turf === 'undefined') {
        document.body.innerHTML = "<h1>Erreur: Dépendances manquantes.</h1><p>Veuillez vérifier que math.min.js, leaflet.js, suncalc.js et turf.min.js sont correctement chargés (dans l'ordre).</p>";
        return;
    }
    
// --- SEGMENT 1 ENDS HERE ---
 // =================================================================
// SEGMENT 2/5 : LOGIQUE UKF, PHYSIQUE AVANCÉE ET CONTRÔLE GPS
// =================================================================

// --- LOGIQUE FILTRE UKF (Structure) ---
// NOTE: La classe réelle nécessite l'implémentation de 21 états et de matrices complexes.
class ProfessionalUKF {
    constructor() {
        this.state = { 
            lat: 43.296, lon: 5.37, alt: 0, speed: 0, vD: 0, // Position & Vitesse
            kUncert: UKF_R_MAX, kAltUncert: 10 
        };
        // Initialisation de la matrice de covariance (P) avec math.js
        this.P = math.identity(21).map(x => math.multiply(x, UKF_R_MAX)); 
    }
    predict(imuReadings, dt) {
        // Logique de PREDICTION UKF (Propagation du modèle IMU - Accélération/Gyro)
        if (dt > 0) this.state.speed += imuReadings.accel[0] * dt;
        this.state.speed = Math.max(0, this.state.speed);
    }
    update(gpsCoords, R_dyn) {
        // Logique de CORRECTION UKF (Fusion des mesures GPS)
        this.state.lat = gpsCoords.latitude;
        this.state.lon = gpsCoords.longitude;
        this.state.alt = gpsCoords.altitude || this.state.alt;
        this.state.speed = gpsCoords.speed || this.state.speed;
        this.state.kUncert = Math.max(1, this.state.kUncert - R_dyn * 0.1); // Simulation de la réduction d'incertitude
    }
    getState() { return this.state; }
}

// --- FONCTIONS DE PHYSIQUE ET DE KALMAN ---

const getSpeedOfSound = (tempK) => 20.04 * Math.sqrt(tempK); 

const getWGS84Gravity = (latDeg, altM) => {
    // Calcul précis de la gravité effective (WGS84 + correction d'altitude)
    const sin2Lat = Math.sin(latDeg * D2R)**2;
    const g0 = 9.780327 * (1 + 0.0053024 * sin2Lat - 0.0000058 * sin2Lat**2);
    return g0 * (1 - (altM * 2 / 6371000)); 
};

const calculateAdvancedPhysics = (speed, alt, mass, CdA, tempK, rhoAir, lat, altSigma, localG, accelLong) => {
    const E0 = mass * C_L**2;
    const speedOfSoundLocal = getSpeedOfSound(tempK);
    const machNumber = speed / speedOfSoundLocal;
    const lorentzFactor = 1 / Math.sqrt(1 - (speed / C_L)**2);
    
    return {
        E0: E0,
        energyRelativistic: E0 * lorentzFactor,
        momentum: mass * speed * lorentzFactor,
        Rs_object: 2 * G_U * mass / C_L**2,
        lorentzFactor: lorentzFactor,
        machNumber: machNumber,
        speedOfSoundLocal: speedOfSoundLocal,
        dynamicPressure: 0.5 * rhoAir * speed**2,
        dragForce: 0.5 * rhoAir * speed**2 * CdA,
        dragPower_kW: (0.5 * rhoAir * speed**2 * CdA * speed) / 1000,
        force_g_long: accelLong / localG,
        gravitationalDilation: 1000000000 * (1 - Math.sqrt(1 - 2 * localG * alt / C_L**2)) * 86400, // ns/j
        timeDilationSpeed: 1000000000 * (lorentzFactor - 1) * 86400, // ns/j
        altSigma: altSigma,
        geopotentialAltitude: alt * 6371000 / (6371000 + alt),
        radiationPressure: speed > 0 ? (rhoAir * speed**2 / C_L) : 0, 
        reynoldsNumber: 'N/A', // Calcul complexe non inclus
        coriolisForce: 2 * mass * speed * (Math.sin(lat * D2R)) * (2 * Math.PI / 86164), // Composante verticale (simple)
        nyquistFrequency: 1 / (2 * (IMU_UPDATE_RATE_MS / 1000)),
    };
};

const getKalmanR = (accRaw, alt, kUncert, environment, reactivity) => {
    let R_dyn = (accRaw || 100)**2;
    const factor = ENVIRONMENT_FACTORS[environment].R_MULT;
    
    if (reactivity === 'AUTOMATIC') {
        // Augmente R si l'incertitude UKF (P) est élevée pour éviter une surcorrection
        R_dyn = R_dyn * factor * (1 + kUncert / UKF_R_MAX); 
    } else {
        R_dyn = R_dyn * factor;
    }
    return Math.min(R_dyn, 10000); // Plafonner R pour éviter l'explosion
};

const calculateDistanceRatio = (altM) => {
    // Correction de distance pour la courbure/altitude ou le mode Nether (1:8)
    const EARTH_RADIUS = 6371000;
    if (distanceRatioMode) {
        return (EARTH_RADIUS + altM) / EARTH_RADIUS; // Ratio de circonférence
    }
    // Mode Nether (Non implémenté, donc 1.0)
    return 1.0;
};

const updateCelestialBody = (body, alt, rotationRadius, angularVelocity) => {
    if (body === 'EARTH') {
        G_ACC = 9.80665;
        R_ALT_CENTER_REF = 6378137.0;
    } else if (body === 'MOON') {
        G_ACC = 1.62;
        R_ALT_CENTER_REF = 1737400;
    } else if (body === 'ROTATING') {
        // G_ACC affiché sera la gravité effective (G - a_centrifuge)
        G_ACC = 9.80665 - (angularVelocity**2 * rotationRadius); 
        R_ALT_CENTER_REF = 6378137.0; 
    }
    return { G_ACC_NEW: G_ACC, R_ALT_CENTER_REF_NEW: R_ALT_CENTER_REF };
};

// --- LOGIQUE DE CONTRÔLE GPS ---

const onGpsSuccess = (pos) => {
    if (wID === null) return; 
    gpsUpdateCallback(pos); 
    const accRaw = pos.coords.accuracy || 100;
    if ($('gps-precision-acc')) $('gps-precision-acc').textContent = `${dataOrDefault(accRaw, 2)} m`;
    
    if (domFastID === null) {
        startFastLoop();
    }
};

const onGpsError = (err) => {
    console.warn(`ERREUR GPS (${err.code}): ${err.message}`);
    if ($('gps-status-dr')) $('gps-status-dr').textContent = `Erreur: ${err.message}`;
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = '⚠️ Erreur GPS';
};

const startGPS = (mode = currentGPSMode) => {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
    }
    currentGPSMode = mode;
    
    if (navigator.geolocation) {
        const opts = GPS_OPTS[currentGPSMode];
        wID = navigator.geolocation.watchPosition(onGpsSuccess, onGpsError, opts);
        
        $('toggle-gps-btn').innerHTML = currentGPSMode === 'HIGH_FREQ' ? '🔴 ARRÊTER GPS' : '🟡 VEILLE GPS';
        $('gps-status-dr').textContent = 'Acquisition...';
        // 'speed-status-text' est mis à jour dans la boucle rapide
    } else {
        alert('La géolocalisation n\'est pas supportée par ce navigateur.');
    }
};

const toggleGPS = () => {
    if (wID === null) {
        startGPS('HIGH_FREQ');
    } else {
        navigator.geolocation.clearWatch(wID);
        wID = null;
        if (domFastID) clearInterval(domFastID);
        domFastID = null;
        
        $('toggle-gps-btn').innerHTML = '▶️ DÉMARRER GPS';
        $('gps-status-dr').textContent = 'INACTIF';
    }
};

// --- SEGMENT 2 ENDS HERE ---
 // =================================================================
// SEGMENT 3/5 : GESTION DES CAPTEURS IMU, NTP ET CARTE
// =================================================================

// --- GESTION DES CAPTEURS IMU ---

const handleDeviceMotion = (event) => {
    // Utilisation de l'accélération pure (sans gravité) pour le filtre UKF si disponible
    const acceleration = event.acceleration || event.accelerationIncludingGravity;
    if (acceleration) {
        accel.x = acceleration.x || 0;
        accel.y = acceleration.y || 0;
        accel.z = acceleration.z || 0;
    }
    if (event.rotationRate) {
        gyro.x = event.rotationRate.alpha || 0; 
        gyro.y = event.rotationRate.beta || 0; 
        gyro.z = event.rotationRate.gamma || 0; 
    }
    imuActive = true;
    if ($('imu-status')) $('imu-status').textContent = 'Actif (Motion)';
};

const handleDeviceOrientation = (event) => {
    mag.x = event.webkitCompassHeading || event.alpha || 0; 
    if ($('imu-status').textContent !== 'Actif (Motion)') {
        $('imu-status').textContent = 'Actif (Orientation)';
    }
};

const startIMU = () => {
    if (window.DeviceMotionEvent) window.addEventListener('devicemotion', handleDeviceMotion);
    if (window.DeviceOrientationEvent) window.addEventListener('deviceorientation', handleDeviceOrientation);
    // Logique de demande de permission (iOS 13+):
    if (typeof DeviceOrientationEvent.requestPermission === 'function') {
        DeviceOrientationEvent.requestPermission()
            .then(permissionState => {
                if (permissionState === 'granted') startIMU(); 
                else $('imu-status').textContent = 'Inactif (Permission)';
            });
    }
    if (!imuActive) $('imu-status').textContent = 'Inactif';
};

// --- GESTION NTP (Heure Précise) ---

const syncH = async () => {
    $('local-time').textContent = 'Synchronisation...';
    try {
        const response = await fetch('https://worldtimeapi.org/api/timezone/Etc/UTC');
        const data = await response.json();
        
        const now = new Date(data.utc_datetime);
        lastNTPDate = now;
        $('local-time').textContent = now.toLocaleTimeString('fr-FR');
        $('date-display').textContent = now.toUTCString();
        
    } catch (e) {
        lastNTPDate = new Date();
        $('local-time').textContent = lastNTPDate.toLocaleTimeString('fr-FR') + ' (Local)';
        $('date-display').textContent = lastNTPDate.toUTCString() + ' (Local)';
        console.warn('Échec de la synchronisation NTP. Utilisation de Date() locale.', e);
    }
};

// --- GESTION MÉTÉO (Simulation d'API) ---

const fetchWeather = async (lat, lon) => {
    try {
        // Simuler un appel API réussi (pour la physique)
        const data = { tempC: 15.0, pressure_hPa: 1013.25, humidity_perc: 60.0,
            polluants: { NO2: 50, PM25: 15, PM10: 25, O3: 80 }
        };
        
        lastT_K = data.tempC + 273.15;
        lastP_hPa = data.pressure_hPa;
        currentAirDensity = (data.pressure_hPa * 100) / (287.058 * lastT_K); 
        currentSpeedOfSound = getSpeedOfSound(lastT_K);
        
        return data;
    } catch (e) {
        // En cas d'échec de l'API (mode hors ligne), utiliser les valeurs par défaut ISA
        currentAirDensity = RHO_SEA_LEVEL;
        currentSpeedOfSound = getSpeedOfSound(TEMP_SEA_LEVEL_K); 
        lastT_K = TEMP_SEA_LEVEL_K;
        lastP_hPa = BARO_ALT_REF_HPA;
        return null;
    }
};

// --- GESTION CARTE (LEAFLET) ---
const initMap = () => {
    try {
        if ($('map') && !map) { 
            map = L.map('map').setView([43.296, 5.37], 10);
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { 
                attribution: '© OpenStreetMap contributors', maxZoom: 20
            }).addTo(map);

            marker = L.marker([43.296, 5.37]).addTo(map);
            circle = L.circle([43.296, 5.37], { color: '#007bff', fillColor: '#007bff', fillOpacity: 0.3, radius: 10 }).addTo(map);
            setTimeout(() => map.invalidateSize(), 400); 
        }
    } catch (e) {
        if ($('map')) $('map').innerHTML = `Erreur d'initialisation de la carte: ${e.message}`;
    }
};

const updateMap = (lat, lon, acc) => {
    if (map && marker && lat && lon) {
        const latLng = [lat, lon];
        marker.setLatLng(latLng);
        circle.setLatLng(latLng).setRadius(acc * R_FACTOR_RATIO); 
        const now = Date.now();
        if (now - lastMapUpdate > MAP_UPDATE_INTERVAL) {
            // Centrer uniquement si le zoom est faible ou si l'utilisateur ne l'a pas bougé manuellement
            if (map.getZoom() < 18) {
                map.setView(latLng, Math.max(12, 18 - Math.log2(acc || 10))); 
            }
            lastMapUpdate = now;
        }
    }
};

// --- SEGMENT 3 ENDS HERE ---
 // =================================================================
// SEGMENT 4/5 : BOUCLES PRINCIPALES DE FUSION (GPS, EKF/UKF, DOM)
// =================================================================

/**
 * BOUCLE LENTE (Callback GPS) - Correction UKF
 */
const gpsUpdateCallback = (pos) => {
    if (emergencyStopActive || !ukf) return;
    
    lastGPSPos = pos; 
    const accRaw = pos.coords.accuracy || 100;
    
    // Priorité à l'override manuel si défini
    const accUsed = gpsAccuracyOverride > 0 ? gpsAccuracyOverride : accRaw; 
    
    let R_dyn = getKalmanR(accUsed, kAlt, kUncert, selectedEnvironment, currentUKFReactivity); 
    let isSignalPoor = (accRaw > MAX_ACC || R_dyn >= UKF_R_MAX * 0.9);

    const spd3D_raw_gps = pos.coords.speed || 0;
    const accel_long_provisional = Math.abs(accel.x); 
    
    // Logique ZUPT (Zero Update Velocity)
    const isPlausiblyStopped = (
        spd3D_raw_gps < ZUPT_RAW_THRESHOLD && 
        accel_long_provisional < ZUPT_ACCEL_THRESHOLD &&
        !isSignalPoor 
    ); 
    
    if (isSignalPoor) {
        if ($('gps-status-dr')) $('gps-status-dr').textContent = 'Drift (Prediction IMU)';
    } else if (isPlausiblyStopped) {
        if ($('gps-status-dr')) $('gps-status-dr').textContent = '✅ ZUPT (Vélocité Nulle)';
        let zuptData = { ...pos.coords, speed: 0 };
        ukf.update(zuptData, R_ALT_MIN); 
    } else {
        if ($('gps-status-dr')) $('gps-status-dr').textContent = 'Actif (Fusion UKF)';
        ukf.update(pos.coords, R_dyn);
    }
};

/**
 * BOUCLE RAPIDE (IMU/EKF Prédiction et DOM Fast Update) - 50Hz
 */
const startFastLoop = () => {
    if (domFastID) return; 
    
    domFastID = setInterval(() => {
        if (emergencyStopActive || !ukf) return;
        
        const now = performance.now();
        const dt = (now - lastIMUTimestamp) / 1000.0;
        if (dt < MIN_DT) return; 
        lastIMUTimestamp = now;

        // --- 1. PRÉDICTION UKF ---
        const imuReadings = { accel: [accel.x, accel.y, accel.z], gyro: [gyro.x, gyro.y, gyro.z] };
        ukf.predict(imuReadings, dt);

        // --- 2. EXTRACTION DE L'ÉTAT ---
        const estimatedState = ukf.getState();
        lat = estimatedState.lat; lon = estimatedState.lon; kAlt = estimatedState.alt; kSpd = estimatedState.speed; 
        kUncert = estimatedState.kUncert; kAltUncert = estimatedState.kAltUncert;
        const sSpdFE = kSpd < MIN_SPD ? 0 : kSpd;
        
        const spd3D_raw_gps = (lastGPSPos && lastGPSPos.coords.speed) ? lastGPSPos.coords.speed : 0;
        const accel_long = accel.x; 
        const local_g = getWGS84Gravity(lat, kAlt);

        // --- 3. CALCULS PHYSIQUES AVANCÉS ---
        const advancedPhysics = calculateAdvancedPhysics(sSpdFE, kAlt, currentMass, 0.5, lastT_K, currentAirDensity, lat, kAltUncert, local_g, accel_long);
        R_FACTOR_RATIO = calculateDistanceRatio(kAlt); 
        
        // Calcul de Distance 3D (Turf.js)
        if (lastGPSPos && lPos && typeof turf !== 'undefined') {
             const from = turf.point([lPos.coords.longitude, lPos.coords.latitude]);
             const to = turf.point([lastGPSPos.coords.longitude, lastGPSPos.coords.latitude]);
             const dist2D = turf.distance(from, to, { units: 'meters' });
             const altDiff = Math.abs((lastGPSPos.coords.altitude || 0) - (lPos.coords.altitude || 0));
             const dist3D_segment = Math.sqrt(dist2D**2 + altDiff**2);
             distM += dist3D_segment * R_FACTOR_RATIO;
        }
        lPos = lastGPSPos; 
        
        if (sSpdFE > MIN_SPD) { timeMoving += dt; }
        if (sTime) { timeTotal = (Date.now() - sTime) / 1000; }
        if (spd3D_raw_gps * KMH_MS > maxSpd) maxSpd = spd3D_raw_gps * KMH_MS; 
        
        // GESTION DE L'ÉNERGIE GPS AUTOMATIQUE
        if (kSpd < MIN_SPD * 2 && currentGPSMode === 'HIGH_FREQ' && !emergencyStopActive) {
            if (gpsStandbyTimeoutID === null) gpsStandbyTimeoutID = setTimeout(() => startGPS('LOW_FREQ'), STANDBY_TIMEOUT_MS);
        } else if (kSpd >= MIN_SPD * 2 && currentGPSMode === 'LOW_FREQ' && !emergencyStopActive) {
            startGPS('HIGH_FREQ');
            if (gpsStandbyTimeoutID) clearTimeout(gpsStandbyTimeoutID);
            gpsStandbyTimeoutID = null;
        }
        
        // --- 4. MISE À JOUR DU DOM (Rapide) ---
        
        // Vitesse
        $('speed-stable').textContent = dataOrDefault(sSpdFE * KMH_MS, 2);
        $('speed-stable-ms').textContent = dataOrDefault(sSpdFE, 3, ' m/s');
        $('speed-3d-inst').textContent = dataOrDefault(spd3D_raw_gps * KMH_MS, 2, ' km/h');
        $('speed-max').textContent = dataOrDefault(maxSpd, 2, ' km/h');
        $('mach-number').textContent = dataOrDefault(advancedPhysics.machNumber, 4);
        $('lorentz-factor').textContent = dataOrDefault(advancedPhysics.lorentzFactor, 8);
        
        // Distance
        $('distance-total-km').textContent = `${dataOrDefault(distM / 1000, 3)} km | ${dataOrDefault(distM, 2)} m`;
        $('distance-ratio').textContent = dataOrDefault(R_FACTOR_RATIO, 3);
        
        // Dynamique
        $('gravity-local').textContent = dataOrDefault(local_g, 4, ' m/s²');
        $('accel-long').textContent = dataOrDefault(accel_long, 3, ' m/s²');
        $('force-g-long').textContent = dataOrDefault(advancedPhysics.force_g_long, 2, ' G');
        $('vertical-speed').textContent = dataOrDefault(estimatedState.vD * -1, 2, ' m/s'); 
        $('dynamic-pressure').textContent = dataOrDefault(advancedPhysics.dynamicPressure, 2, ' Pa');
        $('drag-force').textContent = dataOrDefault(advancedPhysics.dragForce, 2, ' N');
        
        // EKF/Debug
        $('kalman-uncert').textContent = dataOrDefault(kUncert, 3, ' m²/s² (P)');
        $('alt-uncertainty').textContent = dataOrDefault(advancedPhysics.altSigma, 3, ' m (σ)');
        const R_dyn_display = getKalmanR(gpsAccuracyOverride || (lastGPSPos ? lastGPSPos.coords.accuracy : 100), kAlt, kUncert, selectedEnvironment, currentUKFReactivity);
        $('noise-r-dyn').textContent = dataOrDefault(R_dyn_display, 3, ' m² (R dyn)');
        
        // Position
        $('lat-display').textContent = dataOrDefault(lat, 6, ' °');
        $('lon-display').textContent = dataOrDefault(lon, 6, ' °');
        $('alt-display').textContent = dataOrDefault(kAlt, 2, ' m');
        
        // Carte et Niveau à bulle
        updateMap(lat, lon, (lastGPSPos ? gpsAccuracyOverride || lastGPSPos.coords.accuracy : 100));
        $('roll-pitch').textContent = `Roul: ${dataOrDefault(gyro.y * R2D, 1)}° / Tang: ${dataOrDefault(gyro.x * R2D, 1)}°`;
        
    }, IMU_UPDATE_RATE_MS);
};

/**
 * BOUCLE LENTE (Astro/Météo) - 1Hz
 */
const startSlowLoop = () => {
    if (domSlowID) return;
    
    const getMinecraftTime = (date) => {
        const h = date.getUTCHours(); const m = date.getUTCMinutes();
        const totalMinutes = h * 60 + m;
        const mcTime = (totalMinutes / 60 + 6) % 24; 
        const mcH = Math.floor(mcTime);
        const mcM = Math.floor((mcTime - mcH) * 60);
        return `${mcH.toString().padStart(2, '0')}:${mcM.toString().padStart(2, '0')}`;
    };
    
    const updateSlowData = async () => {
        const currentLat = lat || 43.296; 
        const currentLon = lon || 5.37;
        const now = getCDate();

        // 1. Mise à jour Astro
        if (typeof SunCalc !== 'undefined' && lat && lon) {
            try {
                const sunPos = SunCalc.getPosition(now, currentLat, currentLon);
                const moonIllum = SunCalc.getMoonIllumination(now);
                const sunTimes = SunCalc.getTimes(now, currentLat, currentLon);
                
                $('sun-alt').textContent = `${(sunPos.altitude * R2D).toFixed(2)}°`;
                $('moon-illuminated').textContent = `${(moonIllum.fraction * 100).toFixed(1)}%`;
                
                if (sunTimes.sunset && sunTimes.sunrise) {
                    const durationMs = sunTimes.sunset.getTime() - sunTimes.sunrise.getTime();
                    $('day-duration').textContent = `${Math.floor(durationMs / 3600000)}h ${Math.floor((durationMs % 3600000) / 60000)}m`;
                }
                $('time-minecraft').textContent = getMinecraftTime(now);
                
            } catch (e) { console.error("Erreur dans updateAstro:", e); }
        }

        // 2. Mise à jour Météo & Polluants
        const weatherData = await fetchWeather(currentLat, currentLon); 
        
        // Affichage des données météo simulées/mises à jour
        if (weatherData) {
            $('weather-status').textContent = `ACTIF (Simulé)`;
            $('temp-air-2').textContent = `${(lastT_K - 273.15).toFixed(1)} °C`;
            $('pressure-2').textContent = `${lastP_hPa.toFixed(0)} hPa`;
            $('air-density').textContent = `${currentAirDensity.toFixed(3)} kg/m³`;
            // Polluants (Simulation)
            $('no2-data').textContent = dataOrDefault(weatherData.polluants.NO2, 0); 
            $('pm25-data').textContent = dataOrDefault(weatherData.polluants.PM25, 0);
            $('pm10-data').textContent = dataOrDefault(weatherData.polluants.PM10, 0);
            $('o3-data').textContent = dataOrDefault(weatherData.polluants.O3, 0);
        } else {
            $('weather-status').textContent = `❌ HORS LIGNE`;
        }
        
        // 3. Mise à jour Heure
        if (lastNTPDate) {
            $('local-time').textContent = lastNTPDate.toLocaleTimeString('fr-FR');
        }
    };
    
    domSlowID = setInterval(updateSlowData, DOM_SLOW_UPDATE_MS);
    updateSlowData(); 
};

// --- SEGMENT 4 ENDS HERE ---
 // =================================================================
// SEGMENT 5/5 : INITIALISATION DU DOM, ÉVÉNEMENTS ET FIN DE L'IIFE
// =================================================================

const toggleEmergencyStop = () => {
    emergencyStopActive = !emergencyStopActive;
    const btn = $('emergency-stop-btn');
    if (emergencyStopActive) {
        btn.textContent = '🛑 Arrêt d\'urgence: ACTIF 🔴';
        btn.classList.add('active');
        if (wID !== null) toggleGPS(); 
    } else {
        btn.textContent = '🛑 Arrêt d\'urgence: INACTIF 🟢';
        btn.classList.remove('active');
    }
};

document.addEventListener('DOMContentLoaded', () => {
    
    // --- Initialisation des systèmes ---
    initMap(); 
    startIMU();
    syncH();
    updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity);
    
    // --- Écouteurs d'événements pour tous les contrôles ---
    
    $('toggle-gps-btn').addEventListener('click', () => {
        if (!ukf) { 
            ukf = new ProfessionalUKF(); 
            sTime = Date.now();
            startGPS(); 
            startSlowLoop(); 
        } else {
            toggleGPS(); 
        }
    });

    $('emergency-stop-btn').addEventListener('click', toggleEmergencyStop);
    
    $('toggle-mode-btn').addEventListener('click', () => {
        document.body.classList.toggle('dark-mode');
        $('toggle-mode-btn').innerHTML = document.body.classList.contains('dark-mode') ? '<i class="fas fa-sun"></i> Mode Jour' : '<i class="fas fa-moon"></i> Mode Nuit';
    });
    
    $('reset-dist-btn').addEventListener('click', () => { distM = 0; timeMoving = 0; });
    $('reset-max-btn').addEventListener('click', () => { maxSpd = 0; });
    $('reset-all-btn').addEventListener('click', () => {
        if (wID !== null) toggleGPS(); 
        distM = 0; maxSpd = 0; timeMoving = 0; timeTotal = 0; sTime = null;
        kSpd = 0; kUncert = UKF_R_MAX; kAlt = null; kAltUncert = 10; 
        ukf = null; 
        $('toggle-gps-btn').innerHTML = '▶️ DÉMARRER GPS';
    });
    
    $('capture-data-btn').addEventListener('click', () => {
        alert("Données loguées dans la console. Ouvrez F12.");
        console.log("--- DONNÉES CAPTURÉES ---", {
            etatUKF: ukf ? ukf.getState() : 'Inactif',
        });
    });
    
    $('xray-button').addEventListener('click', () => {
        // ID pour l'effet visuel X-Ray/Inversion
        $('minecraft-clock').classList.toggle('x-ray'); 
        $('xray-button').textContent = $('minecraft-clock').classList.contains('x-ray') ? 'X-Ray ON' : 'X-Ray OFF';
    });
    
    // --- Écouteurs pour les Inputs & Selects ---
    
    $('freq-select').addEventListener('change', (e) => startGPS(e.target.value));
    
    $('gps-accuracy-override').addEventListener('input', (e) => {
        gpsAccuracyOverride = parseFloat(e.target.value) || 0.0;
        if ($('gps-accuracy-override-display')) $('gps-accuracy-override-display').textContent = dataOrDefault(gpsAccuracyOverride, 1) + ' m';
    });
    
    $('environment-select').addEventListener('change', (e) => {
        selectedEnvironment = e.target.value;
        const factor = ENVIRONMENT_FACTORS[selectedEnvironment];
        $('env-factor').textContent = `${factor.DISPLAY} (x${factor.R_MULT.toFixed(1)})`;
    });
    
    $('mass-input').addEventListener('input', (e) => {
        currentMass = parseFloat(e.target.value) || 70.0;
        $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
    });

    $('celestial-body-select').addEventListener('change', (e) => {
        currentCelestialBody = e.target.value;
        const { G_ACC_NEW } = updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity);
        $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
    });
    
    const updateRotation = () => {
        rotationRadius = parseFloat($('rotation-radius').value) || 100;
        angularVelocity = parseFloat($('angular-velocity').value) || 0.0;
        if ($('rotation-radius-display')) $('rotation-radius-display').textContent = dataOrDefault(rotationRadius, 2) + ' m';
        if ($('angular-velocity-display')) $('angular-velocity-display').textContent = dataOrDefault(angularVelocity, 4) + ' rad/s';

        if (currentCelestialBody === 'ROTATING') {
            const { G_ACC_NEW } = updateCelestialBody('ROTATING', kAlt, rotationRadius, angularVelocity);
            $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
        }
    };
    $('rotation-radius').addEventListener('input', updateRotation);
    $('angular-velocity').addEventListener('input', updateRotation);
    updateRotation(); 
    
    $('distance-ratio-toggle-btn').addEventListener('click', () => {
        distanceRatioMode = !distanceRatioMode;
        const ratio = distanceRatioMode ? calculateDistanceRatio(kAlt || 0) : 1.0;
        $('distance-ratio-toggle-btn').textContent = `Rapport Distance: ${distanceRatioMode ? 'ALTITUDE' : 'SURFACE'} (${ratio.toFixed(3)})`;
    });
    
    $('ukf-reactivity-mode').addEventListener('change', (e) => currentUKFReactivity = e.target.value);
    
    // Mise à jour de l'affichage initial pour les constantes
    $('gravity-base').textContent = `${G_ACC.toFixed(4)} m/s²`;
    
}); // Fin de DOMContentLoaded

})(window);
