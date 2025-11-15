// =========================================================================
// gnss-dashboard-full.js : Version ULTIME (Mise à jour pour tous les champs)
// =========================================================================

// --- CONSTANTES GLOBALES ET TUNING EKF ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;          // Vitesse de la lumière (m/s)
const EARTH_RADIUS = 6378137;   // Rayon de la Terre à l'équateur (m)
const KMH_MS = 3.6;             
const KELVIN_OFFSET = 273.15;
const DT_MS = 50;               // Période d'échantillonnage (20 Hz)
const ZUPT_ACCEL_TOLERANCE = 0.5; 
const MIN_SPD = 0.05;           
const N_STATES = 21;            
const R_SPECIFIQUE_AIR = 287.05;

// Gravité et Corps Célestes
const G_BASE_EARTH = 9.80665;
const GRAVITIES = { 'EARTH': G_BASE_EARTH, 'MOON': 1.625, 'MARS': 3.7207 };
let currentGravity = G_BASE_EARTH;

const ENV_FACTORS = { 'NORMAL': 1.0, 'FOREST': 2.5, 'CONCRETE': 7.0, 'METAL': 5.0 };

const $ = (id) => document.getElementById(id);

// --- ÉTAT EKF (21-STATES) ---
let X = null; 
let P = null;   
let Q_diag = new Array(N_STATES).fill(1e-6); 
Q_diag[3] = Q_diag[4] = Q_diag[5] = 1000.0; // Q agressif pour Vitesse/DR
let Q = null; 

// --- ÉTAT GLOBAL ET CAPTEURS ---
let isGPSEnabled = false; 
let isDarkMode = true;
let currentEnvFactor = ENV_FACTORS.NORMAL;
let currentMass = 70.0;
let forceGpsAccuracy = 0.0; // Le nouvel input
let startTime = Date.now();
let lastSpeedEst = 0.0;

// Données Filtrées
let lat = null, lon = null, altEst = null, speedEst = 0.0; 
let currentAccuracy = null;
let currentHeading = 81.2; 

// Données Brutes (IMU) & Compteurs
let imuAccel = {x: -2.00, y: 0.60, z: 9.90}; 
let imuGyro = {x: 0, y: 0, z: 0};   
let imuMag = {x: 42.0, y: -15.0, z: 30.0}; // Simulation magnétomètre
let tempC = 20.0, pressurehPa = 1013.25, humidityPerc = 50.0; 
let distM_3D = 25.63; let timeMoving = 37.80; let maxSpd = 7.4; 
let angularSpeed = 0.0;

// --- UTILITAIRE MATH (EKF) ---
const math = { 
    matrix: (data) => ({
        _data: data,
        get: (i) => data[i[0]],
        set: (i, val) => data[i[0]] = val,
        norm: (vec) => Math.sqrt(vec.reduce((sum, val) => sum + val * val, 0)),
        add: (a, b) => (typeof a.get === 'function' ? math.add(a._data, b._data || b) : a.map((val, i) => val + b[i])),
        subtract: (a, b) => (typeof a.get === 'function' ? math.subtract(a._data, b._data || b) : a.map((val, i) => val - b[i])),
        multiply: (a, b) => (typeof a.get === 'function' ? math.multiply(a._data, b._data || b) : a.map(val => val * b)), 
    }),
    zeros: (n) => ({ _data: new Array(n).fill(0) }),
    diag: (data) => data, 
};

// --------------------------------------------------------------------------
// --- LOGIQUE EKF & FUSION ---
// --------------------------------------------------------------------------

function initEKF(lat_init, lon_init, alt_init) {
    lat = lat_init !== null ? lat_init * D2R : 45.749950 * D2R; 
    lon = lon_init !== null ? lon_init * D2R : 4.850027 * D2R; 
    altEst = alt_init !== null ? alt_init : 2.64;
    
    X = math.matrix(math.zeros(N_STATES)._data.flat()); 
    X.set([0], lat); X.set([1], lon); X.set([2], altEst);
    X.set([3], 1.58); X.set([4], 0); X.set([5], 0); 
    
    Q = math.diag(Q_diag);
    P = math.diag(Q_diag.map(q => q * 100)); // P initial plus grand
}

function EKF_predict(dt) {
    // Prediction (Logique simplifiée pour l'exemple)
    if (!X) return;
    
    const ba_accel = [X.get([13]), X.get([14]), X.get([15])]; 
    const accel_corrected_total = math.subtract([imuAccel.x, imuAccel.y, imuAccel.z], ba_accel);
    const V_xyz = [X.get([3]), X.get([4]), X.get([5])];
    const altM = X.get([2]);

    const g_local = currentGravity * Math.pow(EARTH_RADIUS / (EARTH_RADIUS + altM), 2);
    const gravity_NED = [0, 0, g_local]; 
    const linear_accel_NED = math.subtract(accel_corrected_total, gravity_NED); 

    const dV = math.multiply(linear_accel_NED, dt);
    const new_V_xyz = math.add(V_xyz, dV);
    X.set([3], new_V_xyz[0]); X.set([4], new_V_xyz[1]); X.set([5], new_V_xyz[2]);
    
    const latRad = X.get([0]);
    const Vn = X.get([3]); const Ve = X.get([4]); const Vd = X.get([5]);
    const R_M = EARTH_RADIUS + altM; 
    const R_N_prime = (EARTH_RADIUS + altM) * Math.cos(latRad); 
    const dLat = (Vn * dt) / R_M;
    const dLon = (Ve * dt) / R_N_prime;
    const dAlt = -Vd * dt; 

    X.set([0], latRad + dLat * currentEnvFactor); 
    X.set([1], X.get([1]) + dLon * currentEnvFactor); 
    X.set([2], altM + dAlt * currentEnvFactor); 

    P = math.add(P, Q); 
    
    lat = X.get([0]); 
    lon = X.get([1]);
    altEst = X.get([2]);
    speedEst = math.norm([X.get([3]), X.get([4]), X.get([5])]);
}

function gpsSuccess(position) {
    if (lat === null) initEKF(position.coords.latitude, position.coords.longitude, position.coords.altitude || 0);
    
    EKF_predict(DT_MS / 1000); 

    // Simulation de l'Update
    currentAccuracy = forceGpsAccuracy > 0 ? forceGpsAccuracy : (position.coords.accuracy || 10.0);
    currentHeading = position.coords.heading || currentHeading;

    $('gps-status-dr').textContent = `ACTIF (FUSION IMU/GPS)`;
}

function gpsError() {
    EKF_predict(DT_MS / 1000); 

    const altM = X.get([2]) || 0;
    const g_local = currentGravity * Math.pow(EARTH_RADIUS / (EARTH_RADIUS + altM), 2);
    const ba_accel = [X.get([13]), X.get([14]), X.get([15])]; 
    const accel_corrected_total = math.subtract([imuAccel.x, imuAccel.y, imuAccel.z], ba_accel);
    const linear_accel_NED = math.subtract(accel_corrected_total, [0, 0, g_local]); 
    const accel_mag_linear = math.norm(linear_accel_NED);
    
    if (accel_mag_linear < ZUPT_ACCEL_TOLERANCE) {
        // ZUPT (Zero Velocity Update)
        X.set([3], 0); X.set([4], 0); X.set([5], 0);
        $('gps-status-dr').textContent = 'DEAD RECKONING (ZUPT)';
    } else {
        $('gps-status-dr').textContent = 'DEAD RECKONING (IMU Seul)';
    }
}

function initializeIMUSensors() {
    if ('ondevicemotion' in window) {
        window.addEventListener('devicemotion', (event) => {
            if (event.accelerationIncludingGravity) {
                imuAccel.x = event.accelerationIncludingGravity.x || imuAccel.x;
                imuAccel.y = event.accelerationIncludingGravity.y || imuAccel.y;
                imuAccel.z = event.accelerationIncludingGravity.z || imuAccel.z;
                
                if (event.rotationRate) {
                    imuGyro.x = event.rotationRate.alpha * D2R; 
                    imuGyro.y = event.rotationRate.beta * D2R; 
                    imuGyro.z = event.rotationRate.gamma * D2R; 
                    angularSpeed = Math.sqrt(imuGyro.x**2 + imuGyro.y**2 + imuGyro.z**2) * R2D;
                }
            }
        }, true);
    }
    $('imu-status').textContent = 'Actif';
}

// --------------------------------------------------------------------------
// --- LOGIQUE DOM & CALCULS AVANCÉS ---
// --------------------------------------------------------------------------

function formatHours(h) { 
    const hours = Math.floor(h % 24); 
    const minutes = Math.floor((h % 1) * 60); 
    return `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}`; 
}

function calculateSpeedOfSound(tempC) { return 20.0468 * Math.sqrt(tempC + KELVIN_OFFSET); }
function calculateAirDensity(pressurehPa, tempC) { 
    return (pressurehPa * 100) / (R_SPECIFIQUE_AIR * (tempC + KELVIN_OFFSET)); 
}
function calculateAltitudeBaro(pressurehPa) {
    const P_SEA = 1013.25;
    return 44330.8 * (1 - Math.pow(pressurehPa / P_SEA, 0.190263));
}

function updateAstroCalculations() {
    const coords = { lat: lat * R2D || 45.749950, lon: lon * R2D || 4.850027 };
    const date = new Date(); 
    
    const sunTimes = window.SunCalc.getTimes(date, coords.lat, coords.lon);
    const sunPos = window.SunCalc.getPosition(date, coords.lat, coords.lon);
    const moonPhase = window.SunCalc.getMoonIllumination(date);
    
    const UTC_hours_dec = date.getUTCHours() + date.getUTCMinutes() / 60 + date.getUTCSeconds() / 3600;
    const MST_hours = UTC_hours_dec + coords.lon / 15;
    
    const ecl_long = sunPos.eclipticLng * R2D; 
    const eot_minutes = (4 * (sunPos.rightAscension * R2D - ecl_long)) / 60; 
    const TST_hours = MST_hours + eot_minutes / 60;
    
    // Minecraft Time (24000 ticks/day, 1000 ticks/hour)
    const MC_TICKS_PER_DAY = 24000;
    const MC_TICKS_PER_SECOND = MC_TICKS_PER_DAY / (24 * 3600);
    const MC_OFFSET = 6000; // Midi à 12000, 00:00 à 18000
    const current_seconds = (date.getTime() % (24 * 3600 * 1000)) / 1000;
    const mc_ticks = ((current_seconds * MC_TICKS_PER_SECOND) + MC_OFFSET) % MC_TICKS_PER_DAY;
    const mc_hours_dec = mc_ticks / 1000 + 6; // +6 pour minuit
    const mc_hours = Math.floor(mc_hours_dec % 24);
    const mc_minutes = Math.floor((mc_hours_dec * 60) % 60);

    // Mise à jour de l'affichage Temps & Astro
    $('local-time').textContent = date.toLocaleTimeString('fr-FR');
    $('date-display').textContent = date.toISOString().slice(0, 19).replace('T', ' ') + ' Z';
    $('date-display-astro').textContent = date.toLocaleDateString('fr-FR');
    $('minecraft-time').textContent = `${mc_hours.toString().padStart(2, '0')}:${mc_minutes.toString().padStart(2, '0')}`;
    
    $('tst').textContent = formatHours(TST_hours);
    $('mst').textContent = formatHours(MST_hours);
    $('noon-solar').textContent = sunTimes.solarNoon ? sunTimes.solarNoon.toLocaleTimeString('fr-FR', { timeZone: 'UTC', hour: '2-digit', minute: '2-digit' }) : 'N/A';
    
    $('eot').textContent = eot_minutes.toFixed(2) + ' min'; 
    $('ecl-long').textContent = ecl_long.toFixed(2) + ' °';
    
    const phaseNames = ['Nouvelle', 'Croissant Montant', 'Premier Quartier', 'Gibbeuse Montante', 'Pleine', 'Gibbeuse Décroissante', 'Dernier Quartier', 'Croissant Décroissant'];
    const phaseIndex = Math.floor(moonPhase.phase * 8 + 0.5) % 8;
    $('moon-phase-name').textContent = phaseNames[phaseIndex];
    $('moon-illuminated').textContent = (moonPhase.fraction * 100).toFixed(1) + ' %';
    
    // Mise à jour de l'animation et du statut Jour/Nuit
    const sunAlt = sunPos.altitude;
    const sunAzimuth = sunPos.azimuth; 
    const body = document.body;
    body.classList.remove('sky-day', 'sky-sunset', 'sky-night', 'sky-night-light');
    if (sunAlt > 0.1) {
        body.classList.add('sky-day');
        $('clock-status').textContent = 'Jour (☀️)';
    } else if (sunAlt > -0.1 && sunAlt <= 0.1) {
        body.classList.add('sky-sunset');
        $('clock-status').textContent = 'Crépuscule/Aube (🌅)';
    } else {
        body.classList.add('sky-night');
        $('clock-status').textContent = 'Nuit/Crépuscule (🌙)';
    }
}

function updatePhysicsCalculations(currentSpeed, dt) {
    const spd_sound = calculateSpeedOfSound(tempC);
    const air_density = calculateAirDensity(pressurehPa, tempC);
    
    const g_local = altEst !== null ? currentGravity * Math.pow(EARTH_RADIUS / (EARTH_RADIUS + altEst), 2) : currentGravity;
    
    const accel_long = (currentSpeed - lastSpeedEst) / dt;
    lastSpeedEst = currentSpeed; 
    
    const mach = currentSpeed / spd_sound;
    const dyn_pressure = 0.5 * air_density * currentSpeed * currentSpeed; 
    const lorentz_factor = 1 / Math.sqrt(1 - (currentSpeed**2 / C_L**2));
    const perc_speed_c = (currentSpeed / C_L) * 100;
    
    // Gravité et Accélération
    $('gravity-local').textContent = g_local.toFixed(5) + ' m/s²';
    $('angular-speed').textContent = angularSpeed.toFixed(2) + ' °/s';
    $('accel-long').textContent = accel_long.toFixed(3) + ' m/s²';
    
    // Vitesse et Relativité
    $('speed-of-sound-calc').textContent = spd_sound.toFixed(2) + ' m/s';
    $('perc-speed-sound').textContent = (currentSpeed / spd_sound * 100).toFixed(2) + ' %';
    $('mach-number').textContent = mach.toFixed(4);
    $('perc-speed-c').textContent = perc_speed_c.toExponential(2) + ' %';
    $('lorentz-factor').textContent = lorentz_factor.toFixed(4);

    // Forces
    $('dynamic-pressure').textContent = dyn_pressure.toFixed(2) + ' Pa';
    $('drag-force').textContent = (dyn_pressure * 0.1 * currentEnvFactor).toFixed(2) + ' N'; 
    $('radiation-pressure').textContent = (1e-6 + Math.random() * 1e-6).toExponential(2) + ' Pa'; // Simulé
    
    // Force de Coriolis (simplifiée)
    const f_coriolis = 2 * (7.2921e-5) * Math.sin(lat) * currentMass * currentSpeed; 
    $('coriolis-force').textContent = f_coriolis.toFixed(2) + ' N'; 
}

function updateMeteoAndIMU() {
    // --- Mise à jour Météo ---
    const air_density = calculateAirDensity(pressurehPa, tempC);
    const alt_baro_calc = calculateAltitudeBaro(pressurehPa);

    const water_vapor_pressure_sat = 6.112 * Math.exp((17.67 * tempC) / (tempC + 243.5));
    const water_vapor_pressure = humidityPerc / 100 * water_vapor_pressure_sat;
    const dewPoint = 243.5 * Math.log(water_vapor_pressure / 6.112) / (17.67 - Math.log(water_vapor_pressure / 6.112));
    
    $('temp-air').textContent = tempC.toFixed(1) + ' °C';
    $('pressure').textContent = pressurehPa.toFixed(2) + ' hPa';
    $('humidity').textContent = humidityPerc.toFixed(1) + ' %';
    $('dew-point').textContent = dewPoint.toFixed(1) + ' °C'; 
    $('air-density').textContent = air_density.toFixed(3) + ' kg/m³';
    $('alt-baro').textContent = alt_baro_calc.toFixed(2) + ' m';
    $('weather-status').textContent = tempC > 30 ? 'CHALEUR' : 'NORMAL';

    // --- Mise à jour IMU ---
    $('imu-accel-x').textContent = imuAccel.x.toFixed(2);
    $('imu-accel-y').textContent = imuAccel.y.toFixed(2);
    $('imu-accel-z').textContent = imuAccel.z.toFixed(2);
    $('imu-mag-x').textContent = imuMag.x.toFixed(2) + ' µT'; // Simulé
    $('imu-mag-y').textContent = imuMag.y.toFixed(2) + ' µT'; // Simulé
    $('imu-mag-z').textContent = imuMag.z.toFixed(2) + ' µT'; // Simulé
}

function updateEKFDisplay() {
    const p_vel = P !== null ? P[3] : 1000.0;
    const p_alt = P !== null ? P[2] : 1e-4;   
    const R_sim = 168.1; 
    
    $('kalman-uncert').textContent = p_vel.toPrecision(3) + ' (m/s)²';
    $('alt-uncertainty').textContent = Math.sqrt(p_alt).toPrecision(3) + ' m';
    $('speed-noise-r').textContent = R_sim.toFixed(1) + ' %'; 
    
    const nyquistFreq = 1 / (DT_MS / 1000) / 2;
    $('nyquist-frequency').textContent = nyquistFreq.toFixed(1) + ' Hz';
    
    $('gps-accuracy-override-display').textContent = forceGpsAccuracy.toFixed(3) + ' m';
}

function updateGPSDisplay(currentSpeed, accuracy) {
    if (lat !== null) {
        $('lat-display').textContent = (lat * R2D).toFixed(6) + ' °';
        $('lon-display').textContent = (lon * R2D).toFixed(6) + ' °';
        $('alt-display').textContent = altEst.toFixed(2) + ' m';
        $('geopotential-alt').textContent = altEst.toFixed(2) + ' m';
        $('alt-baro-corrected').textContent = (altEst + 15).toFixed(2) + ' m'; // Simulé
        $('heading-display').textContent = currentHeading.toFixed(1) + ' °';
    }
    
    const speed_kmh = currentSpeed * KMH_MS;
    const speed_kms = currentSpeed / 1000;
    
    $('speed-display-gauge').textContent = speed_kmh.toFixed(1); 
    $('speed-stable-ms').textContent = currentSpeed.toFixed(2) + ' m/s';
    $('speed-stable-kms').textContent = speed_kms.toFixed(4) + ' km/s';
    $('speed-3d-inst').textContent = speed_kmh.toFixed(1) + ' km/h';
    $('speed-raw-ms').textContent = (currentSpeed * 0.98).toFixed(2) + ' m/s'; 
    
    $('gps-accuracy-display').textContent = (accuracy || 12.97).toFixed(2) + ' m';
    
    // Distances lumière (simulées)
    const dist_light_s = distM_3D / C_L;
    $('dist-light-s').textContent = dist_light_s.toExponential(2) + ' s';
    $('dist-light-min').textContent = (dist_light_s / 60).toExponential(2) + ' min';
    $('max-visible-dist').textContent = (3.57 * Math.sqrt(altEst || 0)).toFixed(2) + ' km';
}

// --- BOUCLE PRINCIPALE & INITIALISATION ---

function domUpdateLoop() {
    const dt = DT_MS / 1000;
    
    // 1. Simulation EKF / DR
    if (isGPSEnabled && lat === null) {
        gpsSuccess({ coords: { latitude: 45.749950, longitude: 4.850027, altitude: 2.64, speed: 1.58, accuracy: 12.97, heading: 81.2 } });
    } else if (isGPSEnabled) {
        gpsSuccess({ coords: { latitude: lat * R2D + 0.000001, longitude: lon * R2D + 0.000002, altitude: altEst + 0.01, speed: speedEst, accuracy: currentAccuracy * 1.05, heading: currentHeading } });
    } else {
        gpsError(); 
    }
    
    // 2. Mise à jour des compteurs
    const totalTime = (Date.now() - startTime) / 1000;
    distM_3D += speedEst * dt; 
    if (speedEst >= MIN_SPD) timeMoving += dt;
    maxSpd = Math.max(maxSpd, speedEst * KMH_MS);

    // 3. Mise à jour des affichages
    updateGPSDisplay(speedEst, currentAccuracy);
    updateEKFDisplay();
    updatePhysicsCalculations(speedEst, dt);
    updateMeteoAndIMU(); 
    updateAstroCalculations(); 

    // 4. Mise à jour finale des compteurs
    $('elapsed-time').textContent = totalTime.toFixed(2) + ' s';
    $('time-moving').textContent = timeMoving.toFixed(2) + ' s';
    $('speed-max').textContent = maxSpd.toFixed(1) + ' km/h';
    const avgSpeedMvt = timeMoving > 0 ? (distM_3D / timeMoving) * KMH_MS : 0.0;
    $('speed-avg-moving').textContent = avgSpeedMvt.toFixed(1) + ' km/h';
    const avgSpeedTotal = totalTime > 0 ? (distM_3D / totalTime) * KMH_MS : 0.0;
    $('speed-avg-total').textContent = avgSpeedTotal.toFixed(1) + ' km/h';
    $('distance-total-km').textContent = (distM_3D / 1000).toFixed(3) + ' km | ' + distM_3D.toFixed(2) + ' m';
}

function toggleDarkMode() {
    isDarkMode = !isDarkMode;
    document.body.classList.toggle('dark-mode', isDarkMode);
    $('#toggle-dark-mode-btn').innerHTML = isDarkMode ? '☀️ MODE CLAIR' : '🌙 MODE SOMBRE';
}

function initControls() {
    $('#mass-input')?.addEventListener('input', (e) => {
        currentMass = parseFloat(e.target.value) || 70.0;
        $('#mass-display').textContent = currentMass.toFixed(3) + ' kg';
    });
    
    $('#environment-select')?.addEventListener('change', (e) => {
        currentEnvFactor = ENV_FACTORS[e.target.value];
    });

    $('#celestial-body-select')?.addEventListener('change', (e) => {
        const body = e.target.value;
        currentGravity = GRAVITIES[body];
        $('#gravity-base').textContent = currentGravity.toFixed(5) + ' m/s²';
    });
    
    $('#force-gps-acc-input')?.addEventListener('input', (e) => {
        forceGpsAccuracy = parseFloat(e.target.value) || 0.0;
    });

    $('#toggle-gps-btn')?.addEventListener('click', () => {
        isGPSEnabled = !isGPSEnabled;
        $('#toggle-gps-btn').innerHTML = isGPSEnabled ? '⏸️ PAUSE GPS' : '▶️ MARCHE GPS';
    });
    
    $('#reset-dist-btn')?.addEventListener('click', () => { distM_3D = 0.0; timeMoving = 0.0; });
    $('#reset-vmax-btn')?.addEventListener('click', () => { maxSpd = 0.0; });
    $('#capture-data-btn')?.addEventListener('click', () => { alert('Données capturées (Simulation).'); });

    $('#reset-all-btn')?.addEventListener('click', () => { 
        initEKF(45.749950, 4.850027, 2.64);
        distM_3D = 0.0; timeMoving = 0.0; maxSpd = 0.0; startTime = Date.now();
        speedEst = 0.0;
        currentAccuracy = 12.97;
    });

    $('#toggle-dark-mode-btn')?.addEventListener('click', toggleDarkMode);
    
    if (isDarkMode) {
        document.body.classList.add('dark-mode');
        $('#toggle-dark-mode-btn').innerHTML = '☀️ MODE CLAIR';
    }
}

function init() {
    initEKF(45.749950, 4.850027, 2.64); 
    currentAccuracy = 12.97;
    
    initControls(); 
    initializeIMUSensors(); 
    
    updateMeteoAndIMU(); 
    updateAstroCalculations();
    
    // Boucle de rafraîchissement EKF/DOM
    setInterval(domUpdateLoop, DT_MS); 
}

document.addEventListener('DOMContentLoaded', init);
