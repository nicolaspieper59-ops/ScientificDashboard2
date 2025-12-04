// =================================================================
// BLOC 1/4 : CONSTANTES, ÉTAT GLOBAL, UTILITAIRES ET CLASSE UKF
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);
const D2R = Math.PI / 180, R2D = 180 / Math.PI; 
const KMH_MS = 3.6;         
const C_L = 299792458;      

const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || val === Infinity || val === -Infinity) {
        return (decimals === 0 ? '--' : ('--.' + Array(decimals).fill('-').join(''))) + suffix;
    }
    return val.toFixed(decimals) + suffix;
};
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || val === Infinity || val === -Infinity) {
        return 'N/A' + suffix;
    }
    return val.toExponential(decimals) + suffix;
};

// --- CONSTANTES WGS84 et GÉOPHYSIQUES ---
const WGS84_A = 6378137.0;        
const WGS84_F = 1 / 298.257223563; 
const WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F; 
const WGS84_G_EQUATOR = 9.780327; 
const WGS84_BETA = 0.0053024; 
const RHO_SEA_LEVEL = 1.225; 
const TEMP_SEA_LEVEL_K = 288.15; 
const BARO_ALT_REF_HPA = 1013.25; 
const NS_PER_DAY = 86400 * 1e9; // Facteur de conversion pour la dilatation du temps
let G_ACC = 9.80665; // Gravité locale (m/s²)

// --- ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE ---
let map = null;
let gpsWatchID = null;
let gpsIsActive = false;
let startTime = 0; // CRITIQUE : Temps écoulé session
let moveTime = 0; // CRITIQUE : Temps de mouvement
let isMoving = false;
let lastMoveTime = 0;
let lastServerTime = null;
let lastLocalTime = null;
let currentLat = 43.2964, currentLon = 5.3697, kAlt = 0; 
let currentSpeed = 0.0;
let maxSpeed = 0;
let totalDistance = 0;
let lastUKFPosition = { lat: currentLat, lon: currentLon, alt: kAlt }; 
let lastPolyline = null;
let lastKnownIMU = { ax: 0, ay: 0, az: 0, gx: 0, gy: 0, gz: 0, heading: 0 };
let ukf = null; 
let currentCelestialBody = 'EARTH'; 
let currentMass = 70.0; 
let currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound = 340.29; 
let currentUKFReactivity = 'NORMAL'; 
let netherMode = false;

const GPS_OPTIONS = { HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 } };

// --- CLASSE PROFESSIONALUKF (21 ÉTATS) ---
class ProfessionalUKF {
    constructor(nx = 21, nz = 6) {
        this.X = math.zeros(nx, 1);
        this.X.set([0, 0], currentLat * D2R);
        this.X.set([1, 0], currentLon * D2R);
        this.X.set([2, 0], -kAlt); 
        this.P = math.diag(Array(nx).fill(1e-2));
    }
    
    predict(dt, imuRaw) { 
        if (dt <= 0) return;
        const Accel_N = imuRaw.ax; 
        const Accel_E = imuRaw.ay;
        const Accel_D = G_ACC - imuRaw.az; 
        
        this.X.set([3, 0], this.X.get([3, 0]) + Accel_N * dt);
        this.X.set([4, 0], this.X.get([4, 0]) + Accel_E * dt);
        this.X.set([5, 0], this.X.get([5, 0]) + Accel_D * dt);
        
        this.X.set([0, 0], this.X.get([0, 0]) + this.X.get([3, 0]) * dt);
        this.X.set([1, 0], this.X.get([1, 0]) + this.X.get([4, 0]) * dt);
        this.X.set([2, 0], this.X.get([2, 0]) + this.X.get([5, 0]) * dt);
    }
    
    update(z_meas_ned, gps_accuracy) {
        // Logique d'intégration de la mesure GPS (simplifiée pour ce code)
        const K_P_pos = Math.min(0.5, 10 / (gps_accuracy + 1e-6)); 
        this.X.set([0, 0], this.X.get([0, 0]) * (1 - K_P_pos) + z_meas_ned.get([0, 0]) * K_P_pos);
        this.X.set([1, 0], this.X.get([1, 0]) * (1 - K_P_pos) + z_meas_ned.get([1, 0]) * K_P_pos);
        this.X.set([2, 0], this.X.get([2, 0]) * (1 - K_P_pos) + z_meas_ned.get([2, 0]) * K_P_pos);
    }

    getFilteredState() {
        const latRad = this.X.get([0, 0]);
        const lonRad = this.X.get([1, 0]);
        const altM = -this.X.get([2, 0]);
        const V_N = this.X.get([3, 0]); 
        const V_E = this.X.get([4, 0]); 
        const V_D = this.X.get([5, 0]);
        const speed = Math.sqrt(V_N**2 + V_E**2 + V_D**2); 
        const heading = V_N !== 0 || V_E !== 0 ? (Math.atan2(V_E, V_N) * R2D + 360) % 360 : 0;
        return {
            lat: latRad * R2D, lon: lonRad * R2D, alt: altM, speed: speed, heading: heading, V_N, V_E, V_D
        };
    }
}
// =================================================================
// BLOC 2/4 : MODÈLES PHYSIQUES ET ENVIRONNEMENTAUX (ISA, RELATIVITÉ, API)
// =================================================================

function calculateWGS84Gravity(latRad, altM) {
    const sin2 = Math.sin(latRad) ** 2;
    const g_lat = WGS84_G_EQUATOR * (1 + WGS84_BETA * sin2) / Math.sqrt(1 - WGS84_E2 * sin2); 
    const R_LAT = WGS84_A / Math.sqrt(1 - WGS84_E2 * sin2);
    // Correction d'altitude (Bouguer/Air Libre simplifiée)
    return g_lat * (1 - 2 * altM / R_LAT); 
}

function updateCelestialBody(body, altM, rotR = 100, angV = 0.0) {
    if (body === 'EARTH') {
        const latRad = currentLat * D2R;
        G_ACC = calculateWGS84Gravity(latRad, altM);
    } else if (body === 'ROTATING') {
        const g_base = 9.80665;
        const F_CENTRIFUGE = rotR * (angV**2);
        G_ACC = g_base - F_CENTRIFUGE;
    } else {
        G_ACC = 9.80665; 
    }
    return { G_ACC_NEW: G_ACC };
}

function calculateStandardISA(altM) {
    if (altM < 11000) { 
        const T_K = TEMP_SEA_LEVEL_K - 0.0065 * altM;
        const P_Pa = 101325 * Math.pow(T_K / TEMP_SEA_LEVEL_K, 5.256); 
        return { T_K, P_Pa, tempC: T_K - 273.15, pressure_hPa: P_Pa / 100, humidity_perc: 0.0 };
    }
    return { T_K: 216.65, P_Pa: 22632, tempC: -56.5, pressure_hPa: 226.32, humidity_perc: 0.0 }; // Stratosphère
}

function getAirDensity(tempK, pressure_hPa) {
    const pressure_Pa = pressure_hPa * 100;
    const R_SPECIFIC_AIR = 287.058;
    return pressure_Pa / (R_SPECIFIC_AIR * tempK);
}

function getSpeedOfSound(tempK) {
    const GAMMA_AIR = 1.4; 
    const R_SPECIFIC_AIR = 287.058;
    return Math.sqrt(GAMMA_AIR * R_SPECIFIC_AIR * tempK);
}

function calculateRelativityEffects(speed, altM) {
    const v_c_ratio = speed / C_L;
    const kinematic_dilation = 1.0 / Math.sqrt(1.0 - v_c_ratio * v_c_ratio);
    const G_U = 6.67430e-11; 
    const M_EARTH = 5.972e24; 
    const R_E_BASE = 6371000;
    // Taux de dilatation par gravité (par rapport à un point éloigné)
    const gravitational_dilation = 1.0 + (G_U * M_EARTH / (R_E_BASE * C_L**2) - G_U * M_EARTH / ((R_E_BASE + altM) * C_L**2)); 
    return { kinematic_dilation: kinematic_dilation, gravitational_dilation: gravitational_dilation };
}

/** NOUVEAU: Calcul des champs d'énergie et de momentum (Relativité) */
function calculatePhysics(speed, mass, gamma, altM) {
    const G_U = 6.67430e-11; 
    const v_c_ratio = speed / C_L;
    
    // Énergie et Momentum
    const E0 = mass * C_L**2; // Énergie de masse au repos
    const E = gamma * E0;     // Énergie relativiste
    const p = gamma * mass * speed; // Quantité de mouvement relativiste
    const kineticEnergy = 0.5 * mass * speed**2; // Énergie cinétique non relativiste (pour comparaison)
    
    // Rayon de Schwarzschild (Rs)
    const Rs = (2 * G_U * mass) / (C_L**2); 

    // Pression Dynamique et Force de Traînée (Simplifié)
    const drag_coeff = 0.47; // Coefficient de traînée (sphère)
    const ref_area = 1.0; // Surface de référence (1 m²)
    const dynamic_pressure = 0.5 * currentAirDensity * speed**2; // Pression dynamique (q)
    const drag_force = dynamic_pressure * ref_area * drag_coeff; 
    const drag_power = drag_force * speed / 1000; // Puissance en kW
    
    return { E, E0, p, Rs, kineticEnergy, dynamic_pressure, drag_force, drag_power, v_c_ratio };
}

function getCDate(serverTime, localTime) {
    if (serverTime && localTime) {
        const offset = serverTime.getTime() - localTime.getTime();
        return new Date(Date.now() + offset);
    }
    return new Date();
}

const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";
async function syncH() {
    const statusEl = $('local-time');
    if (statusEl) statusEl.textContent = 'Synchronisation...';
    try {
        const response = await fetch(SERVER_TIME_ENDPOINT);
        if (!response.ok) throw new Error('Échec de l\'API Time');
        const data = await response.json();
        lastServerTime = new Date(data.utc_datetime);
        lastLocalTime = new Date();
        if (statusEl) statusEl.textContent = lastLocalTime.toLocaleTimeString('fr-FR');
        if ($('date-utc')) $('date-utc').textContent = lastServerTime.toUTCString();
    } catch (error) {
        if (statusEl) statusEl.textContent = 'SYNCHRO ÉCHOUÉE (Local Time)';
        if ($('date-utc')) $('date-utc').textContent = 'N/A';
    }
}
// =================================================================
// BLOC 3/4 : ACQUISITION CAPTEURS, IMU ET CONTRÔLE SYSTÈME (CRITIQUE)
// =================================================================

// --- GESTION IMU (Device Motion) ---

function handleDeviceMotion(event) {
    const accel = event.accelerationIncludingGravity || event.acceleration; 
    const gyro = event.rotationRate;
    
    if (accel) {
        lastKnownIMU.ax = accel.x || 0;
        lastKnownIMU.ay = accel.y || 0;
        lastKnownIMU.az = accel.z || 0;
        
        if ($('acceleration-x')) $('acceleration-x').textContent = dataOrDefault(lastKnownIMU.ax, 2, ' m/s²');
        if ($('acceleration-y')) $('acceleration-y').textContent = dataOrDefault(lastKnownIMU.ay, 2, ' m/s²');
        if ($('acceleration-z')) $('acceleration-z').textContent = dataOrDefault(lastKnownIMU.az, 2, ' m/s²');
    }
    if (gyro) {
        lastKnownIMU.gx = gyro.alpha || 0;
        lastKnownIMU.gy = gyro.beta || 0;
        lastKnownIMU.gz = gyro.gamma || 0;
        if ($('angular-speed-gyro')) $('angular-speed-gyro').textContent = dataOrDefault(Math.sqrt(gyro.alpha**2 + gyro.beta**2 + gyro.gamma**2), 2, ' rad/s');
    }
}

function startIMUSensors() {
    let sensorStatus = $('statut-capteur');
    if (!sensorStatus) return;
    
    if (typeof DeviceMotionEvent.requestPermission === 'function') {
        sensorStatus.textContent = 'En attente d\'autorisation IMU... (iOS)';
        DeviceMotionEvent.requestPermission().then(permissionState => {
            if (permissionState === 'granted') {
                window.addEventListener('devicemotion', handleDeviceMotion);
                sensorStatus.textContent = 'Actif (IMU/Fusion)';
            } else {
                sensorStatus.textContent = '❌ IMU: Refusé';
            }
        }).catch(error => {
             sensorStatus.textContent = `❌ IMU: Erreur (${error.name})`;
        });
    } else if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', handleDeviceMotion);
        sensorStatus.textContent = 'Actif (IMU/Fusion)';
    } else {
        sensorStatus.textContent = '❌ IMU: Non supporté';
    }
}

// --- GESTION GPS (Geolocation API) ---

function startGPS() {
    const statusElement = $('statut-gps-acquisition');
    if (!('geolocation' in navigator)) {
        if (statusElement) statusElement.textContent = '❌ ERREUR: Geolocation non supportée.';
        return;
    }
    if (statusElement) statusElement.textContent = 'Acquisition GPS en cours...';

    gpsWatchID = navigator.geolocation.watchPosition(
        (position) => {
            const { latitude, longitude, altitude, accuracy } = position.coords;
            
            const newAlt = altitude !== null ? altitude : kAlt;
            if (ukf) ukf.update(math.matrix([[latitude * D2R], [longitude * D2R], [-newAlt]]), accuracy);

            if (map && lastPolyline) {
                lastPolyline.addLatLng(L.latLng(latitude, longitude));
            }

            if (statusElement) statusElement.textContent = `ACTIF (${dataOrDefault(accuracy, 1)}m)`;
            if ($('précision-gps-acc')) $('précision-gps-acc').textContent = `${dataOrDefault(accuracy, 1)} m`;

        }, 
        (error) => {
            let msg = '';
            if (error.code === error.PERMISSION_DENIED) {
                msg = "❌ ERREUR: Accès GPS refusé. (HTTPS/Localhost requis)";
            } else if (error.code === error.TIMEOUT) {
                msg = "⌛ ERREUR: Délai dépassé. (Mauvais signal)";
            } else {
                msg = `❌ ERREUR GPS: ${error.message}`;
            }
            if (statusElement) statusElement.textContent = msg;
        }, 
        GPS_OPTIONS.HIGH_FREQ
    );
}

function stopSystem() {
    if (gpsWatchID !== null) {
        navigator.geolocation.clearWatch(gpsWatchID);
        gpsWatchID = null;
    }
    window.removeEventListener('devicemotion', handleDeviceMotion);
    gpsIsActive = false;
    if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = 'Inactif';
    if ($('statut-capteur')) $('statut-capteur').textContent = 'Inactif';
            }
// =================================================================
// BLOC 4/4 : BOUCLES DE MISE À JOUR, CARTE & INITIALISATION FINALE
// =================================================================

function initMap() {
    if (typeof L === 'undefined') {
        if ($('map')) $('map').textContent = "Erreur: Leaflet n'est pas disponible.";
        return;
    }
    if (!map) {
        map = L.map('map', { center: [currentLat, currentLon], zoom: 13, layers: [ L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19 }) ] });
        lastPolyline = L.polyline([], {color: '#007bff', weight: 3}).addTo(map);
        if ($('map-status')) $('map-status').textContent = '';
    }
}

function updateAstro(lat, lon) {
    if (typeof SunCalc === 'undefined') return;
    const now = getCDate(lastServerTime, lastLocalTime);
    if (!now) return;
    const pos = SunCalc.getPosition(now, lat, lon);
    
    if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(pos.altitude * R2D, 2, '°');
    if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(pos.azimuth * R2D, 2, '°');
}

function updateWeatherDOM(data, isOffline = false) {
    if ($('temp-air')) $('temp-air').textContent = `${dataOrDefault(data.tempC, 1)} °C`;
    if ($('pressure-atmos')) $('pressure-atmos').textContent = `${dataOrDefault(data.pressure_hPa, 0)} hPa`;
    if ($('air-density')) $('air-density').textContent = `${dataOrDefault(currentAirDensity, 3)} kg/m³`;
    if ($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = `${dataOrDefault(currentSpeedOfSound, 2)} m/s`;
}

const DOM_SLOW_UPDATE_MS = 2000; 
function startSlowLoop() {
    if (window.slowLoopRunning) return;
    window.slowLoopRunning = true;
    
    setInterval(() => {
        updateAstro(currentLat, currentLon); 
        
        const isa = calculateStandardISA(kAlt);
        currentAirDensity = getAirDensity(isa.T_K, isa.pressure_hPa);
        currentSpeedOfSound = getSpeedOfSound(isa.T_K);
        updateWeatherDOM(isa, true);
        
        const now = getCDate(lastServerTime, lastLocalTime);
        if (now) {
            if ($('local-time') && !$('local-time').textContent.includes('SYNCHRO ÉCHOUÉE')) {
                $('local-time').textContent = now.toLocaleTimeString('fr-FR');
            }
        }
        
        const { G_ACC_NEW } = updateCelestialBody(currentCelestialBody, kAlt);
        if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
        
    }, DOM_SLOW_UPDATE_MS); 
}

/** Boucle Rapide (Animation Frame) : Propagation UKF et Mise à jour du DOM/Carte (Haute Fréquence). */
function startFastLoop() {
    if (window.fastLoopRunning) return;
    window.fastLoopRunning = true;
    let lastTime = performance.now();
    
    function fastLoop(time) {
        const now = performance.now();
        const dt = (now - lastTime) / 1000.0;
        lastTime = now;
        
        // GESTION DU TEMPS (Session/Mouvement)
        if (startTime === 0 && gpsIsActive) {
            startTime = now;
        }
        if (startTime > 0 && $('time-elapsed-session')) {
            const elapsed = (now - startTime) / 1000;
            $('time-elapsed-session').textContent = dataOrDefault(elapsed, 2, ' s');
        }
        
        let speedKmh = 0;
        let v_c_ratio = 0;
        let gamma = 1.0;

        if (gpsIsActive && ukf && dt > 0) { 
            // 1. Prédiction UKF 
            ukf.predict(dt, lastKnownIMU); 
            const filteredState = ukf.getFilteredState();
            
            // 2. Calcul de la distance 3D et Mise à jour de l'état
            const oldLat = lastUKFPosition.lat; 
            const oldLon = lastUKFPosition.lon;
            const oldAlt = lastUKFPosition.alt;

            currentLat = filteredState.lat;
            currentLon = filteredState.lon;
            kAlt = filteredState.alt;
            currentSpeed = filteredState.speed; 
            
            speedKmh = currentSpeed * KMH_MS; 
            maxSpeed = Math.max(maxSpeed, speedKmh);

            // CALCUL DE LA DISTANCE 3D HAUTE FRÉQUENCE 
            const dLat = (currentLat - oldLat) * D2R * WGS84_A; 
            const dLon = (currentLon - oldLon) * D2R * WGS84_A * Math.cos(currentLat * D2R);
            const dAlt = kAlt - oldAlt; 
            
            const distance3D_frame = Math.sqrt(dLat**2 + dLon**2 + dAlt**2);
            totalDistance += distance3D_frame;
            lastUKFPosition = { lat: currentLat, lon: currentLon, alt: kAlt };

            // GESTION DU TEMPS DE MOUVEMENT
            if (currentSpeed > 0.05) { // Seuil de 5 cm/s
                if (!isMoving) { isMoving = true; lastMoveTime = now; }
                moveTime += (now - lastMoveTime) / 1000;
                lastMoveTime = now;
            } else {
                isMoving = false;
                lastMoveTime = now; 
            }
            if ($('time-movement')) $('time-movement').textContent = dataOrDefault(moveTime, 2, ' s');

            // 3. Calcul Physique & Relativité
            const relEffects = calculateRelativityEffects(currentSpeed, kAlt);
            gamma = relEffects.kinematic_dilation;
            v_c_ratio = relEffects.v_c_ratio;
            const physics = calculatePhysics(currentSpeed, currentMass, gamma, kAlt);
            
            // DILATATION DU TEMPS
            const kinematic_dilation_rate = relEffects.kinematic_dilation - 1;
            const gravitational_dilation_rate = relEffects.gravitational_dilation - 1;
            if ($('time-dilation-speed')) $('time-dilation-speed').textContent = dataOrDefaultExp(kinematic_dilation_rate * NS_PER_DAY, 2, ' ns/j');
            if ($('time-dilation-gravity')) $('time-dilation-gravity').textContent = dataOrDefaultExp(gravitational_dilation_rate * NS_PER_DAY, 2, ' ns/j');
            
            // ÉNERGIE ET MOMENTUM
            if ($('energy-relativist')) $('energy-relativist').textContent = dataOrDefaultExp(physics.E, 3, ' J');
            if ($('energy-mass-rest')) $('energy-mass-rest').textContent = dataOrDefaultExp(physics.E0, 3, ' J');
            if ($('momentum')) $('momentum').textContent = dataOrDefaultExp(physics.p, 3, ' kg·m/s');
            if ($('schwarzschild-radius')) $('schwarzschild-radius').textContent = dataOrDefaultExp(physics.Rs, 3, ' m');
            if ($('kinetic-energy')) $('kinetic-energy').textContent = dataOrDefault(physics.kineticEnergy, 2, ' J');

            // ACCÉLÉRATION ET FORCES
            const g_long = Math.sqrt(lastKnownIMU.ax**2 + lastKnownIMU.ay**2) / G_ACC;
            const g_vert = (lastKnownIMU.az - G_ACC) / G_ACC; 
            if ($('force-g-long')) $('force-g-long').textContent = dataOrDefault(g_long, 2, ' G');
            if ($('accel-long')) $('accel-long').textContent = dataOrDefault(Math.sqrt(lastKnownIMU.ax**2 + lastKnownIMU.ay**2), 2, ' m/s²');
            if ($('force-g-vert')) $('force-g-vert').textContent = dataOrDefault(1 + g_vert, 2, ' G'); 
            if ($('accel-vert-imu')) $('accel-vert-imu').textContent = dataOrDefault(lastKnownIMU.az, 2, ' m/s²');
            
            // MÉCANIQUE DES FLUIDES
            if ($('dynamic-pressure')) $('dynamic-pressure').textContent = dataOrDefault(physics.dynamic_pressure, 2, ' Pa');
            if ($('drag-force')) $('drag-force').textContent = dataOrDefault(physics.drag_force, 2, ' N');
            if ($('drag-power')) $('drag-power').textContent = dataOrDefault(physics.drag_power, 2, ' kW');


        }
        
        // 4. Mise à jour du DOM (Indépendant du statut GPS/UKF)
        if ($('speed-current-kmh')) $('speed-current-kmh').textContent = dataOrDefault(speedKmh, 5, ' km/h');
        if ($('speed-max')) $('speed-max').textContent = dataOrDefault(maxSpeed, 5, ' km/h');
        if ($('distance-total-km')) $('distance-total-km').textContent = `${dataOrDefault(totalDistance / 1000, 3)} km | ${dataOrDefault(totalDistance, 2)} m`;
        if ($('current-lat')) $('current-lat').textContent = dataOrDefault(currentLat, 6);
        if ($('current-lon')) $('current-lon').textContent = dataOrDefault(currentLon, 6);
        if ($('current-alt')) $('current-alt').textContent = dataOrDefault(kAlt, 2, ' m');
        if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(gamma, 4);

        if ($('perc-speed-of-sound')) $('perc-speed-of-sound').textContent = dataOrDefault((currentSpeed / currentSpeedOfSound) * 100, 2, ' %');
        if ($('mach-number')) $('mach-number').textContent = dataOrDefault(currentSpeed / currentSpeedOfSound, 4);
        if ($('perc-speed-light')) $('perc-speed-light').textContent = dataOrDefaultExp(v_c_ratio * 100, 2, ' %');

        if (map) map.setView(L.latLng(currentLat, currentLon));
        if ($('statut-ekf-fusion')) $('statut-ekf-fusion').textContent = gpsIsActive ? `ACTIF (${dataOrDefault(dt*1000, 1)}ms)` : `PAUSE`;


        requestAnimationFrame(fastLoop);
    }
    requestAnimationFrame(fastLoop);
}

/** Gestionnaire principal d'activation/désactivation du système (CRITIQUE) */
function handleToggleGps() {
    const btn = $('toggle-gps-btn');
    
    if (gpsIsActive) {
        stopSystem();
        if (btn) btn.textContent = '▶️ MARCHE GPS';
    } else {
        startIMUSensors(); 
        startGPS(); 

        if (btn) btn.textContent = '⏸️ PAUSE GPS/IMU';
        gpsIsActive = true;
        
        if (ukf === null && typeof ProfessionalUKF !== 'undefined') {
             ukf = new ProfessionalUKF();
        }
        
        if (!window.fastLoopRunning) startFastLoop();
        
        if ($('statut-ekf-fusion')) $('statut-ekf-fusion').textContent = `INITIALISATION EKF...`;
    }
}

// Point d'entrée principal (garanti d'être exécuté après le chargement du DOM)
document.addEventListener('DOMContentLoaded', () => {
    try {
        // VÉRIFICATION CRITIQUE DES DÉPENDANCES ET INITIALISATION
        if (typeof math === 'undefined' || typeof L === 'undefined' || typeof SunCalc === 'undefined' || typeof turf === 'undefined') {
            const statusEl = $('statut-gps-acquisition') || document.body;
            statusEl.innerHTML = '<h2 style="color:red;">CRASH: Dépendances (math.js, leaflet.js, etc.) manquantes.</h2>';
            return;
        }

        initMap();
        syncH(); 
        startSlowLoop(); 
        updateCelestialBody(currentCelestialBody, kAlt); 
        
        if (ukf === null) ukf = new ProfessionalUKF();
        
        // ATTACHEMENT CRITIQUE DU BOUTON MARCHE GPS
        const toggleBtn = $('toggle-gps-btn');
        if (toggleBtn) {
            toggleBtn.addEventListener('click', handleToggleGps);
        } else {
            if ($('statut-gps-acquisition')) {
                $('statut-gps-acquisition').textContent = '❌ Erreur: Bouton Démarrer (ID: toggle-gps-btn) manquant.';
            }
        }
        
        // GESTIONNAIRES D'ÉVÉNEMENTS POUR LES CONTRÔLES (Masse, Mode Nuit, Reset, etc.)
        if ($('toggle-mode-btn')) $('toggle-mode-btn').addEventListener('click', () => { 
            document.body.classList.toggle('dark-mode'); 
            $('toggle-mode-btn').innerHTML = document.body.classList.contains('dark-mode') ? '<i class="fas fa-sun"></i> Mode Jour' : '<i class="fas fa-moon"></i> Mode Nuit';
        });

        if ($('reset-dashboard-btn')) $('reset-dashboard-btn').addEventListener('click', () => { 
            if (confirm("Voulez-vous vraiment réinitialiser le tableau de bord ?")) {
                stopSystem();
                totalDistance = 0; maxSpeed = 0; lastUKFPosition = { lat: currentLat, lon: currentLon, alt: kAlt };
                if (map) { 
                    map.removeLayer(lastPolyline); 
                    lastPolyline = L.polyline([], {color: '#007bff', weight: 3}).addTo(map);
                }
                startTime = 0; moveTime = 0; // Réinitialisation des compteurs de temps
                if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = '▶️ MARCHE GPS';
                if ($('distance-total-km')) $('distance-total-km').textContent = `0.000 km | 0.00 m`; 
                if ($('speed-max')) $('speed-max').textContent = `0.00000 km/h`; 
                if ($('time-elapsed-session')) $('time-elapsed-session').textContent = `0.00 s`;
            } 
        });
        
        // Mise à jour des affichages par défaut
        if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC.toFixed(4)} m/s² (WGS84)`;
        if ($('statut-capteur')) $('statut-capteur').textContent = `Inactif`;
        if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = `Prêt (HTTPS/Localhost requis)`;
        if ($('time-elapsed-session')) $('time-elapsed-session').textContent = `0.00 s`;
        if ($('time-movement')) $('time-movement').textContent = `0.00 s`;

    } catch (error) {
        console.error("DÉBOGAGE: ERREUR D'INITIALISATION MAJEURE (Avant le clic):", error);
        const statusEl = $('statut-gps-acquisition') || document.body;
        statusEl.innerHTML = `<h2 style="color:red;">CRASH INITIAL: ${error.name}</h2><p>Le script a planté avant le démarrage.</p>`;
    }
});
