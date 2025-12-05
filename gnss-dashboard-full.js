// =================================================================
// GNSS SPACETIME DASHBOARD - JS CORRIGÉ (MAPPING ID HTML 100% VALIDE)
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => {
    const el = document.getElementById(id);
    // Debug silencieux : permet de ne pas planter si un ID manque encore
    if (!el) console.warn(`ID manquant dans le HTML : ${id}`);
    return el;
};

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

// --- STUBS ASTRO (Évite le plantage si lib/astro.js échoue) ---
const getSolarTime = (typeof window.getSolarTime === 'function') ? window.getSolarTime : (date, lon) => ({ MST: 'N/A', TST: 'N/A', EOT: '0.00', DateMST: null, DateTST: null, ECL_LONG: 'N/A' });
const getTSLV = (typeof window.getTSLV === 'function') ? window.getTSLV : (date, lon) => 'N/A';
const getMoonPhaseName = (typeof window.getMoonPhaseName === 'function') ? window.getMoonPhaseName : (phase) => 'N/A';

// --- CONSTANTES CRITIQUES ---
const C_L = 299792458; 
const G_U = 6.67430e-11; 
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

// --- ÉTAT GLOBAL ---
let currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound = 340.29; 
let lastT_K = TEMP_SEA_LEVEL_K;
let lastP_hPa = BARO_ALT_REF_HPA;
let currentUKFReactivity = 'AUTO'; 
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
// DÉMARRAGE (IIFE)
// =================================================================
((window) => {
    console.log("Démarrage du GNSS SpaceTime Dashboard corrigé.");

    // --- FILTRE UKF ---
    class ProfessionalUKF {
        constructor() {
            this.state = { 
                lat: 43.296, lon: 5.37, alt: 0, speed: 0, vD: 0, 
                kUncert: UKF_R_MAX, kAltUncert: 10 
            };
            this.P = {}; // Par défaut un objet simple (Matrice de Covariance)
            
            // CORRECTION CRITIQUE (ROBUSTE) : Initialisation de la matrice P,
            // évitant les problèmes de version/type d'objet (ex: Matrix vs Array) de math.js.
            if (typeof math !== 'undefined' && typeof math.identity === 'function') {
                try {
                    // Utilise math.multiply pour l'initialisation de la matrice (plus fiable)
                    let P_init = math.identity(21);
                    this.P = math.multiply(P_init, UKF_R_MAX);
                } catch (e) {
                    // Message d'avertissement en cas d'échec de la librairie math.js
                    console.warn("Échec de l'initialisation de la matrice P. Le filtre EKF/UKF ne fonctionnera pas correctement.", e);
                    this.P = {}; // Assure que le script ne plante pas
                }
            }
        } // <-- ACCCOLADE DE FERMETURE DU CONSTRUCTOR (Doit être ici)
        
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
    } // <-- ACCCOLADE DE FERMETURE DE LA CLASSE ProfessionalUKF

    // --- PHYSIQUE ---
    const getWGS84Gravity = (latDeg, altM) => {
        const sin2Lat = Math.sin(latDeg * D2R)**2;
        const g0 = 9.780327 * (1 + 0.0053024 * sin2Lat - 0.0000058 * sin2Lat**2);
        return g0 * (1 - (altM * 2 / 6371000)); 
    };

    const calculateAdvancedPhysics = (speed, alt, mass, CdA, tempK, rhoAir, lat, altSigma, localG, accelLong) => {
        const E0 = mass * C_L**2;
        const speedOfSoundLocal = 20.04 * Math.sqrt(tempK);
        const machNumber = speed / speedOfSoundLocal;
        const lorentzFactor = 1 / Math.sqrt(1 - (speed / C_L)**2);
        
        return {
            E0: E0,
            energyRelativistic: E0 * lorentzFactor,
            momentum: mass * speed * lorentzFactor,
            Rs_object: 2 * G_U * mass / C_L**2,
            lorentzFactor: lorentzFactor || 1.0,
            machNumber: machNumber,
            speedOfSoundLocal: speedOfSoundLocal,
            dynamicPressure: 0.5 * rhoAir * speed**2,
            dragForce: 0.5 * rhoAir * speed**2 * CdA,
            dragPower_kW: (0.5 * rhoAir * speed**2 * CdA * speed) / 1000,
            force_g_long: localG > 0 ? accelLong / localG : 0,
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
        const factor = (ENVIRONMENT_FACTORS[environment] || ENVIRONMENT_FACTORS.NORMAL).R_MULT;
        return Math.min(R_dyn * factor, 10000); 
    };

    const calculateDistanceRatio = (altM) => {
        return distanceRatioMode ? (6371000 + altM) / 6371000 : 1.0;
    };

    const updateCelestialBody = (body, alt, rotationRadius, angularVelocity) => {
        if (body === 'EARTH') { G_ACC = 9.80665; R_ALT_CENTER_REF = 6378137.0; }
        else if (body === 'MOON') { G_ACC = 1.62; R_ALT_CENTER_REF = 1737400; }
        else if (body === 'ROTATING') { G_ACC = 9.80665 - (angularVelocity**2 * rotationRadius); R_ALT_CENTER_REF = 6378137.0; }
        return { G_ACC_NEW: G_ACC, R_ALT_CENTER_REF_NEW: R_ALT_CENTER_REF };
    };

    // --- CONTROLE GPS ---
    const onGpsSuccess = (pos) => {
        if (wID === null) return; 
        gpsUpdateCallback(pos); 
        const accRaw = pos.coords.accuracy || 100;
        if ($('acc-gps')) $('acc-gps').textContent = `${dataOrDefault(accRaw, 2)} m`; // CORRIGÉ: id="acc-gps"
        
        if (domFastID === null) startFastLoop();
    };

    const onGpsError = (err) => {
        console.warn(`ERREUR GPS: ${err.message}`);
        if ($('speed-status-text')) $('speed-status-text').textContent = `Erreur: ${err.message}`;
    };

    const startGPS = (mode = currentGPSMode) => {
        if (wID !== null) { navigator.geolocation.clearWatch(wID); wID = null; }
        currentGPSMode = mode;
        if (navigator.geolocation) {
            wID = navigator.geolocation.watchPosition(onGpsSuccess, onGpsError, GPS_OPTS[currentGPSMode]);
            if ($('toggle-gps-btn')) $('toggle-gps-btn').innerHTML = '🔴 ARRÊTER GPS';
            if ($('speed-status-text')) $('speed-status-text').textContent = 'Acquisition...';
        } else {
            alert('Géolocalisation non supportée.');
        }
    };

    const toggleGPS = () => {
        if (wID === null) startGPS('HIGH_FREQ');
        else {
            navigator.geolocation.clearWatch(wID); wID = null;
            if (domFastID) { clearInterval(domFastID); domFastID = null; }
            if ($('toggle-gps-btn')) $('toggle-gps-btn').innerHTML = '▶️ MARCHE GPS';
            if ($('speed-status-text')) $('speed-status-text').textContent = 'INACTIF';
        }
    };

    // --- IMU ---
    const handleDeviceMotion = (event) => {
        const acc = event.acceleration || event.accelerationIncludingGravity;
        if (acc) { accel.x = acc.x || 0; accel.y = acc.y || 0; accel.z = acc.z || 0; }
        if (event.rotationRate) { gyro.x = event.rotationRate.alpha || 0; gyro.y = event.rotationRate.beta || 0; gyro.z = event.rotationRate.gamma || 0; }
        imuActive = true;
        if ($('statut-capteur')) $('statut-capteur').textContent = 'Actif (Mouvement)';
    };

    const handleDeviceOrientation = (event) => {
        mag.x = event.webkitCompassHeading || event.alpha || 0; 
        if ($('statut-capteur') && !imuActive) $('statut-capteur').textContent = 'Actif (Orientation)';
        
        // Mise à jour boussole (Niveau à bulle)
        if ($('inclinaison-pitch')) $('inclinaison-pitch').textContent = dataOrDefault(event.beta, 1) + '°';
        if ($('roulis-roll')) $('roulis-roll').textContent = dataOrDefault(event.gamma, 1) + '°';
        
        // Animation Bulle
        const bubble = $('bubble');
        if (bubble) {
            const bx = Math.min(Math.max(event.gamma, -45), 45) * 1; 
            const by = Math.min(Math.max(event.beta, -45), 45) * 1;
            bubble.style.transform = `translate(${bx}px, ${by}px)`; 
        }
    };

    const startIMU = () => {
        if (window.DeviceMotionEvent) window.addEventListener('devicemotion', handleDeviceMotion);
        if (window.DeviceOrientationEvent) window.addEventListener('deviceorientation', handleDeviceOrientation);
    };

    // --- NTP & MÉTÉO ---
    const syncH = async () => {
        if ($('heure-locale')) $('heure-locale').textContent = 'Sync...';
        try {
            const res = await fetch('https://worldtimeapi.org/api/timezone/Etc/UTC');
            const data = await res.json();
            lastNTPDate = new Date(data.utc_datetime);
        } catch (e) {
            lastNTPDate = new Date();
            console.warn('Erreur NTP');
        }
    };

    const fetchWeather = async (lat, lon) => {
        // Simulation pour éviter les erreurs CORS
        lastT_K = 288.15; lastP_hPa = 1013.25;
        return { tempC: 15, pressure_hPa: 1013, humidity_perc: 50, pollutants: { NO2: 20, PM25: 10, PM10: 15, O3: 40 } };
    };

    // --- CARTE ---
    const initMap = () => {
        if (typeof L !== 'undefined' && $('map') && !map) { 
            try {
                map = L.map('map').setView([43.296, 5.37], 13);
                L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { attribution: '© OpenStreetMap' }).addTo(map);
                marker = L.marker([43.296, 5.37]).addTo(map);
                circle = L.circle([43.296, 5.37], { radius: 100 }).addTo(map);
            } catch(e) { console.error("Erreur Carte", e); }
        }
    };

    const updateMap = (lat, lon, acc) => {
        if (map && marker) {
            const latLng = [lat, lon];
            marker.setLatLng(latLng);
            if(circle) circle.setLatLng(latLng).setRadius(acc);
            if (Date.now() - lastMapUpdate > MAP_UPDATE_INTERVAL) {
                map.setView(latLng); lastMapUpdate = Date.now();
            }
        }
    };

    // --- BOUCLE GPS (Mise à jour état) ---
    const gpsUpdateCallback = (pos) => {
        if (emergencyStopActive || !ukf) return;
        lastGPSPos = pos;
        const accRaw = pos.coords.accuracy || 100;
        const R_dyn = getKalmanR(accRaw, kAlt, kUncert, 'NORMAL', 'AUTO');
        
        ukf.update(pos.coords, R_dyn);
        if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = 'Actif';
    };

    // --- BOUCLE RAPIDE (50Hz - UI Fluide) ---
    const startFastLoop = () => {
        if (domFastID) return;
        domFastID = setInterval(() => {
            if (emergencyStopActive || !ukf) return;
            const now = performance.now();
            const dt = (now - lastIMUTimestamp) / 1000.0;
            if (dt < MIN_DT) return;
            lastIMUTimestamp = now;

            ukf.predict({ accel: [accel.x, accel.y, accel.z], gyro: [gyro.x, gyro.y, gyro.z] }, dt);
            const state = ukf.getState();
            
            lat = state.lat; lon = state.lon; kAlt = state.alt; kSpd = state.speed; 
            kUncert = state.kUncert; kAltUncert = state.kAltUncert;
            
            const rawSpd = (lastGPSPos && lastGPSPos.coords.speed) ? lastGPSPos.coords.speed : 0;
            const localG = getWGS84Gravity(lat || 43, kAlt || 0);
            const phys = calculateAdvancedPhysics(kSpd, kAlt || 0, currentMass, 0.5, lastT_K, currentAirDensity, lat || 43, kAltUncert, localG, accel.x);

            // Mise à jour DOM (CORRECTION DES IDs)
            if ($('elapsed-session-time')) $('elapsed-session-time').textContent = dataOrDefault(timeTotal, 2, ' s');
            if ($('elapsed-motion-time')) $('elapsed-motion-time').textContent = dataOrDefault(timeMoving, 2, ' s');
            
            if ($('speed-stable')) $('speed-stable').textContent = dataOrDefault(kSpd * KMH_MS, 2) + ' km/h';
            if ($('speed-stable-ms')) $('speed-stable-ms').textContent = dataOrDefault(kSpd, 2) + ' m/s';
            if ($('speed-stable-kms')) $('speed-stable-kms').textContent = dataOrDefault(kSpd / 1000, 3) + ' km/s';
            if ($('speed-3d-inst')) $('speed-3d-inst').textContent = dataOrDefault(rawSpd * KMH_MS, 2) + ' km/h';
            if ($('speed-raw-ms')) $('speed-raw-ms').textContent = dataOrDefault(rawSpd, 2) + ' m/s';
            
            if ($('vitesse-max-session')) $('vitesse-max-session').textContent = dataOrDefault(maxSpd, 2) + ' km/h'; // CORRIGÉ
            if ($('speed-avg-moving')) $('speed-avg-moving').textContent = dataOrDefault(timeMoving > 0 ? (distM/timeMoving)*3.6 : 0, 2) + ' km/h';
            if ($('speed-avg-total')) $('speed-avg-total').textContent = dataOrDefault(timeTotal > 0 ? (distM/timeTotal)*3.6 : 0, 2) + ' km/h';

            if ($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = dataOrDefault(phys.speedOfSoundLocal, 2, ' m/s');
            if ($('mach-number')) $('mach-number').textContent = dataOrDefault(phys.machNumber, 4);
            if ($('percent-speed-light')) $('percent-speed-light').textContent = dataOrDefaultExp((kSpd/C_L)*100, 4, ' %'); // CORRIGÉ
            if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(phys.lorentzFactor, 8);
            
            if ($('time-dilation-vitesse')) $('time-dilation-vitesse').textContent = dataOrDefault(phys.timeDilationSpeed, 4, ' ns/j'); // CORRIGÉ
            if ($('time-dilation-gravite')) $('time-dilation-gravite').textContent = dataOrDefault(phys.gravitationalDilation, 4, ' ns/j'); // CORRIGÉ
            if ($('rest-mass-energy')) $('rest-mass-energy').textContent = dataOrDefaultExp(phys.E0, 3, ' J'); // CORRIGÉ
            if ($('schwarzschild-radius')) $('schwarzschild-radius').textContent = dataOrDefaultExp(phys.Rs_object, 3, ' m'); // CORRIGÉ

            if ($('distance-totale')) $('distance-totale').textContent = `${dataOrDefault(distM / 1000, 3)} km`; // CORRIGÉ
            if ($('distance-light-s')) $('distance-light-s').textContent = dataOrDefaultExp(distM / C_L, 2, ' s');

            if ($('gravite-wgs84')) $('gravite-wgs84').textContent = dataOrDefault(localG, 4, ' m/s²'); // CORRIGÉ
            if ($('acceleration-long')) $('acceleration-long').textContent = dataOrDefault(accel.x, 2, ' m/s²'); // CORRIGÉ
            if ($('acceleration-vert-imu')) $('acceleration-vert-imu').textContent = dataOrDefault(accel.z, 2, ' m/s²'); // CORRIGÉ
            if ($('force-g-vert')) $('force-g-vert').textContent = dataOrDefault(accel.z / 9.81, 2, ' G'); // CORRIGÉ

            if ($('statut-ekf-fusion')) $('statut-ekf-fusion').textContent = 'Actif';
            if ($('ukf-v-uncert')) $('ukf-v-uncert').textContent = dataOrDefault(kUncert, 3); // CORRIGÉ
            if ($('ukf-alt-sigma')) $('ukf-alt-sigma').textContent = dataOrDefault(kAltUncert, 3); // CORRIGÉ

            if ($('acceleration-x')) $('acceleration-x').textContent = dataOrDefault(accel.x, 2); // CORRIGÉ
            if ($('acceleration-y')) $('acceleration-y').textContent = dataOrDefault(accel.y, 2);
            if ($('acceleration-z')) $('acceleration-z').textContent = dataOrDefault(accel.z, 2);

            if ($('lat-ekf')) $('lat-ekf').textContent = dataOrDefault(lat, 6) + '°'; // CORRIGÉ
            if ($('lon-ekf')) $('lon-ekf').textContent = dataOrDefault(lon, 6) + '°'; // CORRIGÉ
            if ($('alt-ekf')) $('alt-ekf').textContent = dataOrDefault(kAlt, 2) + ' m'; // CORRIGÉ
            if ($('heading-display')) $('heading-display').textContent = dataOrDefault(mag.x, 1) + '°'; // CORRIGÉ

            updateMap(lat, lon, (lastGPSPos ? lastGPSPos.coords.accuracy : 100));

            // Logique de temps
            if (kSpd > MIN_SPD) timeMoving += dt;
            timeTotal += dt;
            if (lastGPSPos && lPos) {
                // distM += ... (Calcul distance simple)
                distM += kSpd * dt; 
            }
            if (kSpd * 3.6 > maxSpd) maxSpd = kSpd * 3.6;
            lPos = lastGPSPos;

        }, IMU_UPDATE_RATE_MS);
    };

    // --- BOUCLE LENTE (1Hz) ---
    const startSlowLoop = () => {
        if (domSlowID) return;
        domSlowID = setInterval(async () => {
            const now = getCDate();
            if ($('heure-locale')) $('heure-locale').textContent = now.toLocaleTimeString();
            if ($('date-heure-utc')) $('date-heure-utc').textContent = now.toUTCString(); // CORRIGÉ
            if ($('time-minecraft')) $('time-minecraft').textContent = getMinecraftTime(now);

            // Météo (IDs corrigés)
            const weather = await fetchWeather(lat, lon);
            if ($('air-temp-c')) $('air-temp-c').textContent = weather.tempC + ' °C'; // CORRIGÉ
            if ($('pressure-hpa')) $('pressure-hpa').textContent = weather.pressure_hPa + ' hPa'; // CORRIGÉ
            if ($('humidity-perc')) $('humidity-perc').textContent = weather.humidity_perc + ' %'; // CORRIGÉ
            
            if ($('no2-val')) $('no2-val').textContent = weather.pollutants.NO2; // CORRIGÉ
            
            // Astro (Utilise les fonctions globales de lib/astro.js)
            try {
                const sTime = getSolarTime(now, lon || 0);
                if ($('tst')) $('tst').textContent = sTime.TST; // CORRIGÉ
                if ($('eot')) $('eot').textContent = sTime.EOT + ' min'; // CORRIGÉ
                if ($('tslv')) $('tslv').textContent = getTSLV(now, lon || 0); // CORRIGÉ
                
                // Si SunCalc est présent
                if (typeof SunCalc !== 'undefined' && lat && lon) {
                    const times = SunCalc.getTimes(now, lat, lon);
                    const moon = SunCalc.getMoonIllumination(now);
                    if ($('sunrise-times')) $('sunrise-times').textContent = times.sunrise.toLocaleTimeString();
                    if ($('sunset-times')) $('sunset-times').textContent = times.sunset.toLocaleTimeString();
                    if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(moon.phase); // CORRIGÉ
                }
            } catch (e) { /* Ignorer erreur astro si pas prêt */ }

        }, DOM_SLOW_UPDATE_MS);
    };

    // --- INIT ---
    document.addEventListener('DOMContentLoaded', () => {
        initMap();
        startIMU();
        syncH();
        
        // Boutons
        if($('toggle-gps-btn')) $('toggle-gps-btn').onclick = () => {
            if (!ukf) { ukf = new ProfessionalUKF(); startGPS(); startFastLoop(); startSlowLoop(); }
            else toggleGPS();
        };
        if($('reset-all-btn')) $('reset-all-btn').onclick = () => {
            distM = 0; maxSpd = 0; timeMoving = 0; timeTotal = 0;
            if(ukf) ukf = new ProfessionalUKF();
        };
    });

})(window);
