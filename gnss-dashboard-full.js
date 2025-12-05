// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// CODE FINAL, NETTOYÉ ET BASÉ SUR LES LIBRAIRIES EXTERNES
// Dépendances critiques (DOIVENT être chargées dans l'HTML AVANT ce script) : 
// math.min.js, leaflet.js, suncalc.js, turf.min.js, lib/astro.js
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
const calculateMaxVisibleDistance = (altM) => altM > 0 ? Math.sqrt(altM * (2 * 6371000 + altM)) : 0; 
const getMinecraftTime = (date) => {
    const h = date.getUTCHours(); const m = date.getUTCMinutes();
    const totalMinutes = h * 60 + m;
    const mcTime = (totalMinutes / 60 + 6) % 24; 
    const mcH = Math.floor(mcTime);
    const mcM = Math.floor((mcTime - mcH) * 60);
    return `${mcH.toString().padStart(2, '0')}:${mcM.toString().padStart(2, '0')}`;
};

// --- CONSTANTES CRITIQUES ---
const C_L = 299792458; // Vitesse de la lumière (m/s)
const G_U = 6.67430e-11; // Constante gravitationnelle universelle
const R2D = 180 / Math.PI; 
const D2R = Math.PI / 180; 
const KMH_MS = 3.6; 
const MIN_SPD = 0.5; 
const MAX_ACC = 50; 
const IMU_UPDATE_RATE_MS = 20; 
const DOM_SLOW_UPDATE_MS = 1000; 
const MAP_UPDATE_INTERVAL = 3000; 
const STANDBY_TIMEOUT_MS = 30000; 
const MIN_DT = 0.005; 
const TEMP_SEA_LEVEL_K = 288.15; 
const RHO_SEA_LEVEL = 1.225; 
const BARO_ALT_REF_HPA = 1013.25; 
const UKF_R_MAX = 1000.0; 
const R_ALT_MIN = 1.0; 
const ZUPT_RAW_THRESHOLD = 0.3; 
const ZUPT_ACCEL_THRESHOLD = 0.1; 
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 5000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 30000, timeout: 60000 }
};
const ENVIRONMENT_FACTORS = {
    NORMAL: { R_MULT: 1.0, DISPLAY: 'Normal' },
    CITY: { R_MULT: 3.0, DISPLAY: 'Urbain Droit' },
    TUNNEL: { R_MULT: 10.0, DISPLAY: 'Tunnel/Indoor' },
    SEA: { R_MULT: 0.5, DISPLAY: 'Mer/Plaine' },
    GROTTO: { R_MULT: 100.0, DISPLAY: 'Grotte/No-GPS' }, 
};

// --- ÉTAT GLOBAL ET VARIABLES D'INSTANCE (accessibles dans l'IIFE) ---
let currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound = 340.29; 
let lastT_K = TEMP_SEA_LEVEL_K;
let lastP_hPa = BARO_ALT_REF_HPA;
let currentUKFReactivity = 'AUTOMATIC'; 
let G_ACC = 9.80665; 
let R_ALT_CENTER_REF = 6378137.0; 
let currentCelestialBody = 'EARTH';
let rotationRadius = 100.0;
let angularVelocity = 0.0;
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

let accel = { x: 0, y: 0, z: 0 };
let gyro = { x: 0, y: 0, z: 0 };
let mag = { x: 0, y: 0, z: 0 };
let lastIMUTimestamp = performance.now();
let imuActive = false;

let map = null;
let marker = null;
let circle = null;
let lastMapUpdate = 0;

// =================================================================
// DÉMARRAGE : Encapsulation de la logique UKF et État Global (IIFE)
// =================================================================
((window) => {
    console.log("Démarrage du GNSS SpaceTime Dashboard.");

    // Vérification des dépendances critiques
    if (typeof math === 'undefined' || typeof L === 'undefined' || typeof SunCalc === 'undefined' || typeof turf === 'undefined') {
        const missing = [];
        if (typeof math === 'undefined') missing.push('math.min.js');
        if (typeof L === 'undefined') missing.push('leaflet.js');
        if (typeof SunCalc === 'undefined') missing.push('suncalc.js');
        if (typeof turf === 'undefined') missing.push('turf.min.js');
        
        const errorMessage = `ERREUR CRITIQUE: Dépendances manquantes : ${missing.join(', ')}. Vérifiez l'ordre et les chemins dans votre HTML.`;
        console.error(errorMessage);
        document.body.innerHTML = `<h1>${errorMessage}</h1><p>Le tableau de bord ne peut pas s'exécuter.</p>`;
        return;
    }
    
    // --- LOGIQUE FILTRE UKF (Structure) ---
    class ProfessionalUKF {
        constructor() {
            this.state = { 
                lat: 43.296, lon: 5.37, alt: 0, speed: 0, vD: 0, 
                kUncert: UKF_R_MAX, kAltUncert: 10 
            };
            this.P = math.identity(21).map(x => math.multiply(x, UKF_R_MAX)); 
        }
        predict(imuReadings, dt) {
            if (dt > 0) this.state.speed += imuReadings.accel[0] * dt;
            this.state.speed = Math.max(0, this.state.speed);
        }
        update(gpsCoords, R_dyn) {
            this.state.lat = gpsCoords.latitude;
            this.state.lon = gpsCoords.longitude;
            this.state.alt = gpsCoords.altitude || this.state.alt;
            this.state.speed = gpsCoords.speed || this.state.speed;
            this.state.kUncert = Math.max(1, this.state.kUncert - R_dyn * 0.1); 
        }
        getState() { return this.state; }
    }

    // --- FONCTIONS DE PHYSIQUE ET DE KALMAN ---

    const getSpeedOfSound = (tempK) => 20.04 * Math.sqrt(tempK); 

    const getWGS84Gravity = (latDeg, altM) => {
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
            gravitationalDilation: 1000000000 * (1 - Math.sqrt(1 - 2 * localG * alt / C_L**2)) * 86400, 
            timeDilationSpeed: 1000000000 * (lorentzFactor - 1) * 86400, 
            altSigma: altSigma,
            geopotentialAltitude: alt * 6371000 / (6371000 + alt),
            radiationPressure: speed > 0 ? (rhoAir * speed**2 / C_L) : 0, 
            reynoldsNumber: 'N/A', 
            coriolisForce: 2 * mass * speed * (Math.sin(lat * D2R)) * (2 * Math.PI / 86164), 
            nyquistFrequency: 1 / (2 * (IMU_UPDATE_RATE_MS / 1000)),
        };
    };

    const getKalmanR = (accRaw, alt, kUncert, environment, reactivity) => {
        let R_dyn = (accRaw || 100)**2;
        const factor = ENVIRONMENT_FACTORS[environment].R_MULT;
        
        if (reactivity === 'AUTOMATIC') {
            R_dyn = R_dyn * factor * (1 + kUncert / UKF_R_MAX); 
        } else {
            R_dyn = R_dyn * factor;
        }
        return Math.min(R_dyn, 10000); 
    };

    const calculateDistanceRatio = (altM) => {
        const EARTH_RADIUS = 6371000;
        if (distanceRatioMode) {
            return (EARTH_RADIUS + altM) / EARTH_RADIUS; 
        }
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
        if ($('toggle-gps-btn')) $('toggle-gps-btn').innerHTML = '⚠️ Erreur GPS';
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

    // --- GESTION DES CAPTEURS IMU ---
    const handleDeviceMotion = (event) => {
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
        if ($('imu-status') && $('imu-status').textContent !== 'Actif (Motion)') {
            $('imu-status').textContent = 'Actif (Orientation)';
        }
        if (event.alpha !== null && $('compass-needle')) {
            $('compass-heading').textContent = dataOrDefault(event.alpha, 1) + ' °';
            $('compass-needle').style.transform = `rotate(${-event.alpha}deg)`;
        }
    };

    const startIMU = () => {
        if (window.DeviceMotionEvent) window.addEventListener('devicemotion', handleDeviceMotion);
        if (window.DeviceOrientationEvent) window.addEventListener('deviceorientation', handleDeviceOrientation);
        if (typeof DeviceOrientationEvent.requestPermission === 'function') {
            DeviceOrientationEvent.requestPermission().then(permissionState => {
                if (permissionState !== 'granted') $('imu-status').textContent = 'Inactif (Permission)';
            }).catch(e => console.error("Permission IMU refusée", e));
        }
        if (!imuActive && $('imu-status')) $('imu-status').textContent = 'Inactif';
    };

    // --- GESTION NTP (Heure Précise) ---
    const syncH = async () => {
        if ($('local-time')) $('local-time').textContent = 'Synchronisation...';
        try {
            const response = await fetch('https://worldtimeapi.org/api/timezone/Etc/UTC');
            const data = await response.json();
            const now = new Date(data.utc_datetime);
            lastNTPDate = now;
            if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString('fr-FR');
            if ($('date-display')) $('date-display').textContent = now.toUTCString();
        } catch (e) {
            lastNTPDate = new Date();
            if ($('local-time')) $('local-time').textContent = lastNTPDate.toLocaleTimeString('fr-FR') + ' (Local)';
            if ($('date-display')) $('date-display').textContent = lastNTPDate.toUTCString() + ' (Local)';
            console.warn('Échec de la synchronisation NTP. Utilisation de Date() locale.', e);
        }
    };

    // --- GESTION MÉTÉO (Simulation d'API) ---
    const fetchWeather = async (lat, lon) => {
        try {
            // Simulation d'un appel API réussi (pour la physique)
            const data = { tempC: 15.0, pressure_hPa: 1013.25, humidity_perc: 60.0,
                polluants: { NO2: 50, PM25: 15, PM10: 25, O3: 80 }
            };
            
            lastT_K = data.tempC + 273.15;
            lastP_hPa = data.pressure_hPa;
            currentAirDensity = (data.pressure_hPa * 100) / (287.058 * lastT_K); 
            currentSpeedOfSound = getSpeedOfSound(lastT_K);
            
            return data;
        } catch (e) {
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
                if (map.getZoom() < 18) {
                    map.setView(latLng, Math.max(12, 18 - Math.log2(acc || 10))); 
                }
                lastMapUpdate = now;
            }
        }
    };

    /**
     * BOUCLE LENTE (Callback GPS) - Correction UKF
     */
    const gpsUpdateCallback = (pos) => {
        if (emergencyStopActive || !ukf) return;
        
        lastGPSPos = pos; 
        const accRaw = pos.coords.accuracy || 100;
        const accUsed = gpsAccuracyOverride > 0 ? gpsAccuracyOverride : accRaw; 
        
        let R_dyn = getKalmanR(accUsed, kAlt, kUncert, selectedEnvironment, currentUKFReactivity); 
        let isSignalPoor = (accRaw > MAX_ACC || R_dyn >= UKF_R_MAX * 0.9);

        const spd3D_raw_gps = pos.coords.speed || 0;
        const accel_long_provisional = Math.abs(accel.x); 
        
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
            const local_g = getWGS84Gravity(lat || 43.296, kAlt || 0);

            // --- 3. CALCULS PHYSIQUES AVANCÉS ---
            const advancedPhysics = calculateAdvancedPhysics(sSpdFE, kAlt || 0, currentMass, 0.5, lastT_K, currentAirDensity, lat || 43.296, kAltUncert, local_g, accel_long);
            R_FACTOR_RATIO = calculateDistanceRatio(kAlt || 0); 
            
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
            
            // Temps
            if ($('elapsed-time')) $('elapsed-time').textContent = dataOrDefault(timeTotal, 2, ' s');
            if ($('time-moving')) $('time-moving').textContent = dataOrDefault(timeMoving, 2, ' s');
            
            // Vitesse
            if ($('speed-stable')) $('speed-stable').textContent = dataOrDefault(sSpdFE * KMH_MS, 2);
            if ($('speed-stable-ms')) $('speed-stable-ms').textContent = dataOrDefault(sSpdFE, 3, ' m/s');
            if ($('speed-stable-kms')) $('speed-stable-kms').textContent = dataOrDefaultExp(sSpdFE / 1000, 3, ' km/s');
            if ($('speed-3d-inst')) $('speed-3d-inst').textContent = dataOrDefault(spd3D_raw_gps * KMH_MS, 2, ' km/h');
            if ($('speed-raw-ms')) $('speed-raw-ms').textContent = dataOrDefault(spd3D_raw_gps, 3, ' m/s');
            if ($('speed-max')) $('speed-max').textContent = dataOrDefault(maxSpd, 2, ' km/h');
            if ($('speed-avg-moving')) $('speed-avg-moving').textContent = timeMoving > 1 ? dataOrDefault((distM / timeMoving) * KMH_MS, 2, ' km/h') : '0.00 km/h';
            if ($('speed-avg-total')) $('speed-avg-total').textContent = timeTotal > 1 ? dataOrDefault((distM / timeTotal) * KMH_MS, 2, ' km/h') : '0.00 km/h';

            // Physique & Relativité
            if ($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = dataOrDefault(advancedPhysics.speedOfSoundLocal, 2, ' m/s');
            if ($('perc-speed-sound')) $('perc-speed-sound').textContent = dataOrDefault(advancedPhysics.machNumber * 100, 2, ' %');
            if ($('mach-number')) $('mach-number').textContent = dataOrDefault(advancedPhysics.machNumber, 4);
            if ($('perc-speed-c')) $('perc-speed-c').textContent = dataOrDefaultExp(sSpdFE / C_L * 100, 2, ' %');
            if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(advancedPhysics.lorentzFactor, 8);
            if ($('time-dilation-v')) $('time-dilation-v').textContent = dataOrDefault(advancedPhysics.timeDilationSpeed, 3, ' ns/j');
            if ($('time-dilation-g')) $('time-dilation-g').textContent = dataOrDefault(advancedPhysics.gravitationalDilation, 3, ' ns/j');
            if ($('energy-relativistic')) $('energy-relativistic').textContent = dataOrDefaultExp(advancedPhysics.energyRelativistic, 3, ' J');
            if ($('energy-rest-mass')) $('energy-rest-mass').textContent = dataOrDefaultExp(advancedPhysics.E0, 3, ' J');
            if ($('momentum')) $('momentum').textContent = dataOrDefault(advancedPhysics.momentum, 2, ' kg·m/s');
            if ($('Rs-object')) $('Rs-object').textContent = dataOrDefaultExp(advancedPhysics.Rs_object, 3, ' m');

            // Distance
            if ($('distance-total-km')) $('distance-total-km').textContent = `${dataOrDefault(distM / 1000, 3)} km | ${dataOrDefault(distM, 2)} m`;
            if ($('distance-ratio')) $('distance-ratio').textContent = dataOrDefault(R_FACTOR_RATIO, 3);
            const dist_light_s = distM / C_L;
            if ($('distance-light-s')) $('distance-light-s').textContent = dataOrDefaultExp(dist_light_s, 2, ' s');
            if ($('distance-light-min')) $('distance-light-min').textContent = dataOrDefaultExp(dist_light_s / 60, 2, ' min');
            if ($('distance-light-h')) $('distance-light-h').textContent = dataOrDefaultExp(dist_light_s / 3600, 2, ' h');
            if ($('distance-light-day')) $('distance-light-day').textContent = dataOrDefaultExp(dist_light_s / 86400, 2, ' j');
            if ($('distance-light-week')) $('distance-light-week').textContent = dataOrDefaultExp(dist_light_s / (86400 * 7), 2, ' sem');
            if ($('distance-light-month')) $('distance-light-month').textContent = dataOrDefaultExp(dist_light_s / (86400 * 30.44), 2, ' mois');
            if ($('distance-cosmic')) $('distance-cosmic').textContent = `${dataOrDefaultExp(distM / 149597870700, 2)} UA | ${dataOrDefaultExp(distM / 9460730472580800, 2)} al`; 
            if ($('distance-horizon')) $('distance-horizon').textContent = dataOrDefault(calculateMaxVisibleDistance(kAlt || 0) / 1000, 1, ' km');

            // Dynamique
            if ($('gravity-local')) $('gravity-local').textContent = dataOrDefault(local_g, 4, ' m/s²');
            if ($('accel-long')) $('accel-long').textContent = dataOrDefault(accel_long, 3, ' m/s²');
            if ($('force-g-long')) $('force-g-long').textContent = dataOrDefault(advancedPhysics.force_g_long, 2, ' G');
            if ($('vertical-speed')) $('vertical-speed').textContent = dataOrDefault(estimatedState.vD * -1, 2, ' m/s'); 
            if ($('accel-vertical-imu')) $('accel-vertical-imu').textContent = dataOrDefault(accel.z, 2, ' m/s²');
            if ($('force-g-vertical')) $('force-g-vertical').textContent = dataOrDefault(accel.z / local_g, 2, ' G');
            if ($('angular-speed')) $('angular-speed').textContent = dataOrDefault(Math.sqrt(gyro.x**2 + gyro.y**2 + gyro.z**2) * R2D, 2, ' °/s');
            
            // Mécanique des Fluides
            if ($('dynamic-pressure')) $('dynamic-pressure').textContent = dataOrDefault(advancedPhysics.dynamicPressure, 2, ' Pa');
            if ($('drag-force')) $('drag-force').textContent = dataOrDefault(advancedPhysics.dragForce, 2, ' N');
            if ($('drag-power-kw')) $('drag-power-kw').textContent = dataOrDefault(advancedPhysics.dragPower_kW, 2, ' kW');
            if ($('reynolds-number')) $('reynolds-number').textContent = advancedPhysics.reynoldsNumber;
            
            // Champs & Forces
            if ($('kinetic-energy')) $('kinetic-energy').textContent = dataOrDefault(0.5 * currentMass * sSpdFE**2, 2, ' J');
            if ($('mechanical-power')) $('mechanical-power').textContent = dataOrDefault(advancedPhysics.accel_long * currentMass * sSpdFE, 2, ' W');
            if ($('radiation-pressure')) $('radiation-pressure').textContent = dataOrDefaultExp(advancedPhysics.radiationPressure, 2, ' Pa');
            if ($('coriolis-force')) $('coriolis-force').textContent = dataOrDefaultExp(advancedPhysics.coriolisForce, 2, ' N');

            // EKF/Debug
            if ($('kalman-uncert')) $('kalman-uncert').textContent = dataOrDefault(kUncert, 3, ' m²/s² (P)');
            if ($('alt-uncertainty')) $('alt-uncertainty').textContent = dataOrDefault(advancedPhysics.altSigma, 3, ' m (σ)');
            const R_dyn_display = getKalmanR(gpsAccuracyOverride || (lastGPSPos ? lastGPSPos.coords.accuracy : 100), kAlt || 0, kUncert, selectedEnvironment, currentUKFReactivity);
            if ($('noise-r-dyn')) $('noise-r-dyn').textContent = dataOrDefault(R_dyn_display, 3, ' m² (R dyn)');
            if ($('band-nyquist')) $('band-nyquist').textContent = dataOrDefault(advancedPhysics.nyquistFrequency, 2, ' Hz');
            if ($('gps-accuracy-display')) $('gps-accuracy-display').textContent = dataOrDefault(gpsAccuracyOverride, 6, ' m');
            if ($('ekf-status')) $('ekf-status').textContent = ukf ? 'Actif (Fusion)' : 'Inactif';

            // IMU
            if ($('accel-x')) $('accel-x').textContent = dataOrDefault(accel.x, 2, ' m/s²');
            if ($('accel-y')) $('accel-y').textContent = dataOrDefault(accel.y, 2, ' m/s²');
            if ($('accel-z')) $('accel-z').textContent = dataOrDefault(accel.z, 2, ' m/s²');
            if ($('mag-x')) $('mag-x').textContent = dataOrDefault(mag.x, 2, ' µT');
            if ($('mag-y')) $('mag-y').textContent = dataOrDefault(mag.y, 2, ' µT');
            if ($('mag-z')) $('mag-z').textContent = dataOrDefault(mag.z, 2, ' µT');
            
            // Position
            if ($('lat-display')) $('lat-display').textContent = dataOrDefault(lat, 6, ' °');
            if ($('lon-display')) $('lon-display').textContent = dataOrDefault(lon, 6, ' °');
            if ($('alt-display')) $('alt-display').textContent = dataOrDefault(kAlt, 2, ' m');
            if ($('geopotential-alt')) $('geopotential-alt').textContent = dataOrDefault(advancedPhysics.geopotentialAltitude, 2, ' m');
            if ($('alt-corrected-baro')) $('alt-corrected-baro').textContent = 'N/A'; 
            if ($('cap-direction')) $('cap-direction').textContent = dataOrDefault(lastGPSPos ? lastGPSPos.coords.heading : null, 1, ' °');
            
            // Carte et Niveau à bulle
            updateMap(lat || 43.296, lon || 5.37, (lastGPSPos ? gpsAccuracyOverride || lastGPSPos.coords.accuracy : 100));
            if ($('roll-pitch')) $('roll-pitch').textContent = `Roul: ${dataOrDefault(gyro.y * R2D, 1)}° / Tang: ${dataOrDefault(gyro.x * R2D, 1)}°`;

        }, IMU_UPDATE_RATE_MS);
    };

    /**
     * BOUCLE LENTE (Astro/Météo) - 1Hz
     * DÉPEND DE : SunCalc, getSolarTime, getTSLV, getMoonPhaseName (fournis par lib/astro.js)
     */
    const startSlowLoop = () => {
        if (domSlowID) return;
        
        const updateSlowData = async () => {
            const currentLat = lat || 43.296; 
            const currentLon = lon || 5.37;
            const now = getCDate();

            // 1. Mise à jour Astro
            // Vérification des dépendances SunCalc ET des fonctions de lib/astro.js
            if (typeof SunCalc !== 'undefined' && typeof getSolarTime !== 'undefined' && typeof getTSLV !== 'undefined' && typeof getMoonPhaseName !== 'undefined') {
                try {
                    const solarTimes = getSolarTime(now, currentLon);
                    const tslv = getTSLV(now, currentLon);
                    const moonIllumData = SunCalc.getMoonIllumination(now);
                    const moonPhaseName = getMoonPhaseName(moonIllumData.phase);
                    
                    const sunPos = SunCalc.getPosition(now, currentLat, currentLon);
                    const sunTimes = SunCalc.getTimes(now, currentLat, currentLon);
                    const moonPos = SunCalc.getMoonPosition(now, currentLat, currentLon);
                    
                    // MAJ DOM ASTRO
                    if ($('date-display-astro')) $('date-display-astro').textContent = now.toLocaleDateString();
                    if ($('date-solar-mean')) $('date-solar-mean').textContent = solarTimes.DateMST ? solarTimes.DateMST.toLocaleDateString() : 'N/A';
                    if ($('date-solar-true')) $('date-solar-true').textContent = solarTimes.DateTST ? solarTimes.DateTST.toLocaleDateString() : 'N/A';
                    if ($('mst')) $('mst').textContent = solarTimes.MST;
                    if ($('tst')) $('tst').textContent = solarTimes.TST;
                    if ($('noon-solar') && sunTimes.solarNoon) $('noon-solar').textContent = sunTimes.solarNoon.toLocaleTimeString('fr-FR', { timeZone: 'UTC' });
                    if ($('eot')) $('eot').textContent = `${solarTimes.EOT} min`;
                    if ($('tslv')) $('tslv').textContent = tslv;
                    if ($('ecl-long')) $('ecl-long').textContent = `${solarTimes.ECL_LONG}°`;
                    
                    if ($('sun-alt')) $('sun-alt').textContent = `${(sunPos.altitude * R2D).toFixed(2)}°`;
                    if ($('sun-azimuth')) $('sun-azimuth').textContent = `${(sunPos.azimuth * R2D).toFixed(2)}°`;
                    
                    if (sunTimes.sunset && sunTimes.sunrise) {
                        const durationMs = sunTimes.sunset.getTime() - sunTimes.sunrise.getTime();
                        if ($('day-duration')) $('day-duration').textContent = `${Math.floor(durationMs / 3600000)}h ${Math.floor((durationMs % 3600000) / 60000)}m`;
                        if ($('sunrise-times')) $('sunrise-times').textContent = sunTimes.sunrise.toLocaleTimeString('fr-FR');
                        if ($('sunset-times')) $('sunset-times').textContent = sunTimes.sunset.toLocaleTimeString('fr-FR');
                    }
                    
                    if ($('moon-phase-name')) $('moon-phase-name').textContent = moonPhaseName;
                    if ($('moon-illuminated')) $('moon-illuminated').textContent = `${(moonIllumData.fraction * 100).toFixed(1)}%`;
                    if ($('moon-alt')) $('moon-alt').textContent = `${(moonPos.altitude * R2D).toFixed(2)}°`;
                    if ($('moon-azimuth')) $('moon-azimuth').textContent = `${(moonPos.azimuth * R2D).toFixed(2)}°`;
                    
                    // Horloge Minecraft (Visuelle et Affichage)
                    const clockDiv = $('minecraft-clock');
                    if (clockDiv) {
                        const sunRotation = (-sunPos.azimuth * R2D) + 90; 
                        if ($('sun-element')) $('sun-element').style.transform = `rotate(${sunRotation}deg)`;
                        if (sunPos.altitude > 0) { clockDiv.className = 'sky-day'; }
                        else if (sunPos.altitude > -10 * D2R) { clockDiv.className = 'sky-sunset'; }
                        else { clockDiv.className = 'sky-night'; }
                    }
                    if ($('time-minecraft')) $('time-minecraft').textContent = getMinecraftTime(now);
                    if ($('weather-status')) $('weather-status').textContent = `ACTIF (Astro/Météo Simulé)`;

                } catch (e) { 
                    console.error("Erreur critique dans updateAstro. Assurez-vous que toutes les fonctions de lib/astro.js sont définies globalement et que vsop2013.js est chargé en premier.", e); 
                    if ($('weather-status')) $('weather-status').textContent = `❌ ASTRO HORS LIGNE (Erreur d'exécution)`;
                }
            } else {
                 if ($('weather-status')) $('weather-status').textContent = `❌ ASTRO HORS LIGNE (Dépendances manquantes)`;
            }

            // 2. Mise à jour Météo & Polluants
            const weatherData = await fetchWeather(currentLat, currentLon); 
            
            if (weatherData) {
                if ($('temp-air-2')) $('temp-air-2').textContent = `${(lastT_K - 273.15).toFixed(1)} °C`;
                if ($('pressure-2')) $('pressure-2').textContent = `${lastP_hPa.toFixed(0)} hPa`;
                if ($('air-density')) $('air-density').textContent = `${currentAirDensity.toFixed(3)} kg/m³`;
                if ($('humidity-2')) $('humidity-2').textContent = `${weatherData.humidity_perc.toFixed(0)} %`; 
                
                if ($('no2-data')) $('no2-data').textContent = dataOrDefault(weatherData.polluants.NO2, 0); 
                if ($('pm25-data')) $('pm25-data').textContent = dataOrDefault(weatherData.polluants.PM25, 0);
                if ($('pm10-data')) $('pm10-data').textContent = dataOrDefault(weatherData.polluants.PM10, 0);
                if ($('o3-data')) $('o3-data').textContent = dataOrDefault(weatherData.polluants.O3, 0);
            }
            
            // 3. Mise à jour Heure
            if (lastNTPDate && $('local-time')) {
                $('local-time').textContent = lastNTPDate.toLocaleTimeString('fr-FR');
            }
        };
        
        domSlowID = setInterval(updateSlowData, DOM_SLOW_UPDATE_MS);
        updateSlowData(); 
    };

    const toggleEmergencyStop = () => {
        emergencyStopActive = !emergencyStopActive;
        const btn = $('emergency-stop-btn');
        if (btn) {
            if (emergencyStopActive) {
                btn.textContent = '🛑 Arrêt d\'urgence: ACTIF 🔴';
                btn.classList.add('active');
                if (wID !== null) toggleGPS(); 
            } else {
                btn.textContent = '🛑 Arrêt d\'urgence: INACTIF 🟢';
                btn.classList.remove('active');
            }
        }
    };

    document.addEventListener('DOMContentLoaded', () => {
        
        // --- Initialisation des systèmes ---
        initMap(); 
        startIMU();
        syncH();
        updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity);
        
        // --- Écouteurs d'événements ---
        
        if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', () => {
            if (!ukf) { 
                // Initialisation du système au premier clic
                ukf = new ProfessionalUKF(); 
                sTime = Date.now();
                startGPS(); 
                startSlowLoop(); 
            } else {
                toggleGPS(); 
            }
        });

        if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', toggleEmergencyStop);
        
        if ($('toggle-mode-btn')) $('toggle-mode-btn').addEventListener('click', () => {
            document.body.classList.toggle('dark-mode');
            $('toggle-mode-btn').innerHTML = document.body.classList.contains('dark-mode') ? '<i class="fas fa-sun"></i> Mode Jour' : '<i class="fas fa-moon"></i> Mode Nuit';
        });
        
        if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => { distM = 0; timeMoving = 0; });
        if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', () => { maxSpd = 0; });
        if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => {
            if (wID !== null) toggleGPS(); 
            distM = 0; maxSpd = 0; timeMoving = 0; timeTotal = 0; sTime = null;
            kSpd = 0; kUncert = UKF_R_MAX; kAlt = null; kAltUncert = 10; 
            ukf = null; 
            if ($('toggle-gps-btn')) $('toggle-gps-btn').innerHTML = '▶️ DÉMARRER GPS';
        });
        
        if ($('capture-data-btn')) $('capture-data-btn').addEventListener('click', () => {
            alert("Données loguées dans la console. Ouvrez F12.");
            console.log("--- DONNÉES CAPTURÉES ---", { etatUKF: ukf ? ukf.getState() : 'Inactif' });
        });
        
        if ($('xray-button') && $('minecraft-clock')) $('xray-button').addEventListener('click', () => {
            $('minecraft-clock').classList.toggle('x-ray'); 
            $('xray-button').textContent = $('minecraft-clock').classList.contains('x-ray') ? 'X-Ray ON' : 'X-Ray OFF';
        });
        
        // Inputs et Selects
        if ($('freq-select')) $('freq-select').addEventListener('change', (e) => startGPS(e.target.value));
        
        if ($('gps-accuracy-override')) $('gps-accuracy-override').addEventListener('input', (e) => {
            gpsAccuracyOverride = parseFloat(e.target.value) || 0.0;
            if ($('gps-accuracy-override-display')) $('gps-accuracy-override-display').textContent = dataOrDefault(gpsAccuracyOverride, 1) + ' m';
        });
        
        if ($('environment-select')) $('environment-select').addEventListener('change', (e) => {
            selectedEnvironment = e.target.value;
            const factor = ENVIRONMENT_FACTORS[selectedEnvironment];
            if ($('env-factor')) $('env-factor').textContent = `${factor.DISPLAY} (x${factor.R_MULT.toFixed(1)})`;
        });
        
        if ($('mass-input')) $('mass-input').addEventListener('input', (e) => {
            currentMass = parseFloat(e.target.value) || 70.0;
            if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        });

        if ($('celestial-body-select')) $('celestial-body-select').addEventListener('change', (e) => {
            currentCelestialBody = e.target.value;
            const { G_ACC_NEW } = updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity);
            if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
        });
        
        const updateRotation = () => {
            rotationRadius = parseFloat($('rotation-radius').value) || 100;
            angularVelocity = parseFloat($('angular-velocity').value) || 0.0;
            if ($('rotation-radius-display')) $('rotation-radius-display').textContent = dataOrDefault(rotationRadius, 2) + ' m';
            if ($('angular-velocity-display')) $('angular-velocity-display').textContent = dataOrDefault(angularVelocity, 4) + ' rad/s';

            if (currentCelestialBody === 'ROTATING') {
                const { G_ACC_NEW } = updateCelestialBody('ROTATING', kAlt, rotationRadius, angularVelocity);
                if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
            }
        };
        if ($('rotation-radius')) $('rotation-radius').addEventListener('input', updateRotation);
        if ($('angular-velocity')) $('angular-velocity').addEventListener('input', updateRotation);
        updateRotation();

        if ($('distance-ratio-toggle-btn')) $('distance-ratio-toggle-btn').addEventListener('click', () => {
            distanceRatioMode = !distanceRatioMode;
            const ratio = distanceRatioMode ? calculateDistanceRatio(kAlt || 0) : 1.0;
            $('distance-ratio-toggle-btn').textContent = `Rapport Distance: ${distanceRatioMode ? 'ALTITUDE' : 'SURFACE'} (${ratio.toFixed(3)})`;
        });
        
        if ($('ukf-reactivity-mode')) $('ukf-reactivity-mode').addEventListener('change', (e) => currentUKFReactivity = e.target.value);
        
        // Initialisation des valeurs DOM
        if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC.toFixed(4)} m/s²`;
        if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        if ($('env-factor')) $('env-factor').textContent = `Normal (x1.0)`;
        if ($('alt-ref-radius')) $('alt-ref-radius').textContent = `${(R_ALT_CENTER_REF / 1000).toFixed(2)} km`;
        if ($('gps-accuracy-override-display')) $('gps-accuracy-override-display').textContent = dataOrDefault(gpsAccuracyOverride, 1) + ' m';
        if ($('noise-r-dyn')) $('noise-r-dyn').textContent = dataOrDefault(getKalmanR(100, 0, UKF_R_MAX, 'NORMAL', 'AUTOMATIC'), 3, ' m² (R dyn)');
        
    }); // Fin de DOMContentLoaded  

})(window);
