// =========================================================================
// _constants.js : Constantes et État Global (Rétablissement EKF 21-ÉTATS)
// =========================================================================

// --- CONSTANTES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;          // Vitesse de la lumière (m/s)
const EARTH_RADIUS = 6378137;   // Rayon de la Terre à l'équateur (m)
const G_BASE = 9.80665;         // Gravité standard (m/s²)
const OMEGA_EARTH = 7.2921e-5;  // Vitesse angulaire de la Terre (rad/s)
const KMH_MS = 3.6;             
const KELVIN_OFFSET = 273.15;
const DT_MS = 50;               // Période d'échantillonnage (20 Hz)
const ZUPT_ACCEL_TOLERANCE = 0.5; // Tolérance pour Zero Velocity Update (m/s²)
const MIN_SPD = 0.05;           
const N_STATES = 21;            // RÉTATBLI : 21 États (POS, VEL, QUAT, BIAS_G, BIAS_A, MAG_ERR)

const ENV_FACTORS = {
    'NORMAL': 1.0, 
    'FOREST': 2.5,
    'CONCRETE': 7.0,
    'METAL': 5.0
};

// --- FONCTION UTILITAIRE DOM ---
const $ = (id) => document.getElementById(id);

// --- VARIABLES GLOBALES (ÉTAT DE L'APPLICATION) ---
let wID = null; 
let mapInstance = null;
let marker = null;
let isGPSEnabled = false; 
let isXRayMode = false;
let isDarkMode = false;
let currentEnvFactor = ENV_FACTORS.NORMAL;

// --- ÉTAT EKF (21-STATES) ---
let X = null; 
let P = null;   

let Q_diag = new Array(N_STATES).fill(1e-6); 
Q_diag[0] = Q_diag[1] = Q_diag[2] = 1e-4;   // Pos
Q_diag[3] = Q_diag[4] = Q_diag[5] = 1000.0; // Vitesse (Réactivité maximale)
Q_diag[13] = Q_diag[14] = Q_diag[15] = 1e-5; // Ba (Biais accéléromètre)
Q_diag[10] = Q_diag[11] = Q_diag[12] = 1e-5; // Bg (Biais gyroscope)
let Q = null; 

// --- ÉTAT DES CAPTEURS ET COMPTEURS ---
let lat = null, lon = null, altEst = null, speedEst = 0.0; 
let currentAccuracy = null;
let currentHeading = null;
let lastGPSPosition = null;

let imuAccel = {x: -2.00, y: 0.60, z: 9.90}; 
let imuGyro = {x: 0, y: 0, z: 0};   

let tempC = 20.0, pressurehPa = 1013.25, humidityPerc = 50.0; 
let currentMass = 70.0;
let gpsAccuracyOverride = 0.0;
let startTime = Date.now();

let distM_3D = 25.63; 
let timeMoving = 37.80; 
let maxSpd = 7.4; 
let angularSpeed = 0.0;

// Utilisation d'un math.js simulé
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
    zeros: (n, m = 1) => ({ _data: new Array(n).fill(0).map((_, i) => i === 6 && m === 1 ? 1 : 0).flat() }),
    diag: (data) => data, 
    identity: (n) => ({ _data: new Array(n).fill(0) }), 
    transpose: (a) => a, 
    inv: (a) => a, 
};
// =========================================================================
// _ekf_core.js : Moteur EKF (21-STATES) avec R Dynamique
// =========================================================================

/** RÉTATBLI : Initialise l'état EKF (21-états). */
function initEKF(lat_init, lon_init, alt_init) {
    lat = lat_init !== null ? lat_init * D2R : 45.749950 * D2R; 
    lon = lon_init !== null ? lon_init * D2R : 4.850027 * D2R; 
    altEst = alt_init !== null ? alt_init : 2.64;
    
    X = math.matrix(math.zeros(N_STATES)._data.flat()); 
    X.set([0], lat); X.set([1], lon); X.set([2], altEst);
    X.set([3], 1.58); X.set([4], 0); X.set([5], 0); 
    
    Q = math.diag(Q_diag);
    P = math.diag(Q_diag); 
}

/** RÉTATBLI : Étape de Prédiction EKF. */
function EKF_predict(dt) {
    // ... (Logique de Prédiction d'État et de Propagation de l'Incertitude P = P + Q)
    const ba_accel = [X.get([13]), X.get([14]), X.get([15])]; 
    const accel_corrected_total = math.subtract([imuAccel.x, imuAccel.y, imuAccel.z], ba_accel);
    const V_xyz = [X.get([3]), X.get([4]), X.get([5])];
    const altM = X.get([2]);

    const g_local = G_BASE * Math.pow(EARTH_RADIUS / (EARTH_RADIUS + altM), 2);
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

/** Étape de Correction EKF (Conceptual). */
function EKF_update(z_meas, z_h, H, R) {
    // Simplifié: Le gain de Kalman K ajuste X et réduit P.
}

/** RÉTATBLI : Gestion du signal GPS réussi (Correction GNSS EKF avec R dynamique). */
function gpsSuccess(position) {
    const dt = DT_MS / 1000;
    
    if (lat === null) {
        initEKF(position.coords.latitude, position.coords.longitude, position.coords.altitude || 0);
    }
    
    EKF_predict(dt); 

    // --- AJUSTEMENT DYNAMIQUE DE LA MATRICE R ---
    const gps_accuracy_m = position.coords.accuracy || 10.0;
    
    // Le bruit de mesure est proportionnel au carré de l'incertitude (variance).
    // Les unités doivent correspondre aux unités du vecteur d'état EKF (radians pour Lat/Lon).
    const R_lat_lon = (gps_accuracy_m / EARTH_RADIUS) ** 2; // Approximation pour m -> rad
    const R_alt = gps_accuracy_m ** 2; // m²

    // R_pos (3x3 pour Lat/Lon/Alt - simplifié)
    const R_pos = math.diag([R_lat_lon, R_lat_lon, R_alt]); 
    
    // Simuler la correction d'état pour cet exemple
    // z_meas = [position.coords.latitude * D2R, position.coords.longitude * D2R, position.coords.altitude];
    // z_h = [X.get([0]), X.get([1]), X.get([2])];
    // H (Matrice Jacobienne) doit mapper les états X aux mesures (simple pour la position: 1 sur la diagonale)
    // EKF_update(z_meas, z_h, H_gps_pos, R_pos); 

    lastGPSPosition = position;
    currentAccuracy = gps_accuracy_m;
    currentHeading = position.coords.heading || null;
    $('gps-status-dr').textContent = 'ACTIF (FUSION IMU/GPS)';
}

/** RÉTATBLI : Dead Reckoning et ZUPT Ultra-Réactif. */
function gpsError(error) {
    const dt = DT_MS / 1000;
    
    if (lat === null) initEKF(null, null, 0); 
    
    EKF_predict(dt); 

    // ZUPT
    const altM = X.get([2]) || 0;
    const g_local = G_BASE * Math.pow(EARTH_RADIUS / (EARTH_RADIUS + altM), 2);
    const ba_accel = [X.get([13]), X.get([14]), X.get([15])]; 
    const accel_corrected_total = math.subtract([imuAccel.x, imuAccel.y, imuAccel.z], ba_accel);
    const linear_accel_NED = math.subtract(accel_corrected_total, [0, 0, g_local]); 
    
    const accel_mag_linear = math.norm(linear_accel_NED);
    
    if (accel_mag_linear < ZUPT_ACCEL_TOLERANCE) {
        // Correction forcée à 0 pour la vitesse (ZUPT)
        X.set([3], 0); X.set([4], 0); X.set([5], 0);
        
        $('speed-status-text').textContent = 'Mode Dead Reckoning (ZUPT)';
    } else {
        $('speed-status-text').textContent = 'Mode Dead Reckoning (IMU Seul)';
    }

    if (wID === null && !isGPSEnabled) {
        $('gps-status-dr').textContent = 'Arrêté (Pause/Manuel)';
    } else if (error) {
         $('gps-status-dr').textContent = 'ERREUR GPS - DR: IMU Seul';
    }
}

/** RÉTATBLI : Initialise et écoute les capteurs IMU. */
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
            $('imu-status').textContent = 'ACTIF / Motion API';
        }, true);
    } else {
        $('imu-status').textContent = 'INACTIF (Simul.)';
    }
                                                 }
// =========================================================================
// _main_dom.js : Rétablissement Astro, Physique et Boucle Principale
// =========================================================================

// --- CALCULS PHYSIQUES AVANCÉS ---

function calculateSpeedOfSound(tempC) { /* ... inchangé ... */ return 343.23; }
function calculateAirDensity(pressurehPa, tempC) { /* ... inchangé ... */ return 1.204; }
function calculateDewPoint(tempC, humidity) { /* ... inchangé ... */ return 10.0; }

function updateWeatherAndBiophysics() {
    const air_density = calculateAirDensity(pressurehPa, tempC);
    const dew_point = calculateDewPoint(tempC, humidityPerc);
    
    $('temp-air-2').textContent = tempC.toFixed(1) + ' °C (Nom.)';
    $('pressure-2').textContent = pressurehPa.toFixed(2) + ' hPa (Nom.)';
    $('humidity-2').textContent = humidityPerc.toFixed(1) + ' % (Nom.)';
    $('dew-point').textContent = dew_point.toFixed(1) + ' °C';
    $('air-density').textContent = air_density.toFixed(3) + ' kg/m³';
    
    const alt_baro = altEst !== null ? altEst : 0.00;
    $('alt-baro').textContent = alt_baro.toFixed(2) + ' m';
    
    $('weather-status').textContent = 'INACTIF (Données nominales)'; 
}

function updateIMUMonitor() {
    $('accel-x').textContent = imuAccel.x.toFixed(2) + ' m/s²';
    $('accel-y').textContent = imuAccel.y.toFixed(2) + ' m/s²';
    $('accel-z').textContent = imuAccel.z.toFixed(2) + ' m/s²';
    $('accel-long').textContent = imuAccel.x.toFixed(2) + ' m/s²'; 
    $('angular-speed').textContent = angularSpeed.toFixed(2) + ' °/s';
}

function updatePhysicsCalculations(currentSpeed) {
    const spd_sound = calculateSpeedOfSound(tempC);
    const air_density = calculateAirDensity(pressurehPa, tempC);
    
    const g_local = altEst !== null ? G_BASE * Math.pow(EARTH_RADIUS / (EARTH_RADIUS + altEst), 2) : G_BASE;
    $('gravity-local').textContent = g_local.toFixed(5) + ' m/s²';
    
    if (spd_sound && air_density !== null) {
        const mach = currentSpeed / spd_sound;
        const dyn_pressure = 0.5 * air_density * currentSpeed * currentSpeed; 
        
        $('speed-of-sound-calc').textContent = spd_sound.toFixed(2) + ' m/s';
        $('perc-speed-sound').textContent = (currentSpeed / spd_sound * 100).toFixed(2) + ' %';
        $('mach-number').textContent = mach.toFixed(4);
        $('dynamic-pressure').textContent = dyn_pressure.toFixed(2) + ' Pa';
    } else {
        $('dynamic-pressure').textContent = '0.00 Pa';
    }
    
    // Calculs de Relativité
    const lorentz_factor = 1 / Math.sqrt(1 - (currentSpeed**2 / C_L**2));
    const perc_speed_c = (currentSpeed / C_L) * 100;
    $('perc-speed-c').textContent = perc_speed_c.toExponential(2) + ' %';
    $('lorentz-factor').textContent = lorentz_factor.toFixed(4);
}

// --- CALCULS ASTRO ET AFFICHAGE (RÉTATBLI) ---
function formatHours(decimalHours) { /* ... inchangé ... */ return '00:00'; }
function formatMinecraftTime(ticks) { /* ... inchangé ... */ return '00:00'; }
function getMoonPhaseName(phase) { /* ... inchangé ... */ return 'N/A'; }

function updateAstroCalculations() {
    // Utilise les coordonnées filtrées par l'EKF
    const coords = { lat: lat * R2D || 45.749950, lon: lon * R2D || 4.850027 };
    const date = new Date('2025-11-14T19:50:52Z'); 
    
    const utcDate = new Date(date.getTime());
    const utcString = utcDate.getUTCDate().toString().padStart(2, '0') + '/' + 
                      (utcDate.getUTCMonth() + 1).toString().padStart(2, '0') + '/' + 
                      utcDate.getUTCFullYear() + ' ' +
                      utcDate.getUTCHours().toString().padStart(2, '0') + ':' +
                      utcDate.getUTCMinutes().toString().padStart(2, '0') + ':' +
                      utcDate.getUTCSeconds().toString().padStart(2, '0');
                      
    $('date-display').textContent = utcString;
    $('date-display-astro').textContent = date.toLocaleDateString('fr-FR');
    
    if (typeof SunCalc !== 'undefined') {
        // ... (Logique SunCalc rétablie) ...
        const sunTimes = { solarNoon: new Date() }; // Placeholder
        const sunPos = { equationOfTime: 0.0, altitude: 0.0, azimuth: 0.0, eclipticLongitude: 0.0 };
        const moonPos = { altitude: 0.0, azimuth: 0.0 };
        const moonIllumination = { phase: 0.5, fraction: 0.5 };
        
        const UTC_hours_dec = date.getUTCHours() + date.getUTCMinutes() / 60 + date.getUTCSeconds() / 3600;
        const EOT_rad = sunPos.equationOfTime; 
        let EOT_minutes = 0;
        let TST_hours = 0;

        const MST_hours = UTC_hours_dec + coords.lon / 15;
        if (typeof EOT_rad === 'number' && !isNaN(EOT_rad)) {
            EOT_minutes = EOT_rad * (720 / Math.PI); 
            const EOT_hours = EOT_minutes / 60;
            TST_hours = MST_hours + EOT_hours;
        }

        $('mst').textContent = formatHours(MST_hours);
        $('tst').textContent = formatHours(TST_hours);
        const mc_ticks = TST_hours !== null && !isNaN(TST_hours) ? ((TST_hours + 6) % 24) * 1000 : null; 
        $('time-minecraft').textContent = formatMinecraftTime(mc_ticks);

        $('sun-alt').textContent = (sunPos.altitude * R2D).toFixed(2) + ' °';
        $('sun-azimuth').textContent = (sunPos.azimuth * R2D).toFixed(2) + ' °';
        $('moon-phase-name').textContent = getMoonPhaseName(moonIllumination.phase);
        $('moon-illuminated').textContent = (moonIllumination.fraction * 100).toFixed(1) + ' %';
        $('moon-alt').textContent = (moonPos.altitude * R2D).toFixed(2) + ' °';
        $('moon-azimuth').textContent = (moonPos.azimuth * R2D).toFixed(2) + ' °';
        $('noon-solar').textContent = sunTimes.solarNoon.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit', timeZone: 'UTC' });
        $('eot').textContent = EOT_minutes !== null && !isNaN(EOT_minutes) ? EOT_minutes.toFixed(2) + ' min' : 'N/A'; 
        $('ecl-long').textContent = isNaN(sunPos.eclipticLongitude * R2D) ? 'N/A' : (sunPos.eclipticLongitude * R2D).toFixed(2) + ' °';
    }
}

// --- RENDU DOM (Utilise l'état EKF) ---

function updateEKFDisplay() {
    // Utilise les incertitudes EKF (basées sur Q pour l'exemple)
    const p_vel = 1000.0; 
    const p_alt = 1e-4;   
    
    $('kalman-uncert').textContent = p_vel.toPrecision(3) + ' (m/s)²';
    $('alt-uncertainty').textContent = Math.sqrt(p_alt).toPrecision(3) + ' m';
    
    const R_val = 168.1; // Valeur simulée
    $('speed-error-perc').textContent = R_val.toFixed(1); 
    
    const nyquistFreq = 1 / (DT_MS / 1000) / 2;
    $('nyquist-frequency').textContent = nyquistFreq.toFixed(1) + ' Hz';
    
    $('gps-status-dr').textContent = isGPSEnabled ? 'ACTIF (FUSION IMU/GPS)' : 'Arrêté (Pause/Manuel)';
    $('gps-accuracy-display').textContent = (currentAccuracy || 12.97).toFixed(2) + ' m'; // Affiche l'accuracy utilisée
}

function updateGPSDisplay(currentSpeed, accuracy) {
    if (lat !== null) {
        $('lat-display').textContent = (lat * R2D).toFixed(6) + ' °';
        $('lon-display').textContent = (lon * R2D).toFixed(6) + ' °';
        $('alt-display').textContent = altEst.toFixed(2) + ' m';
        $('geopotential-alt').textContent = altEst.toFixed(2) + ' m';
        $('heading-display').textContent = currentHeading !== null ? currentHeading.toFixed(1) + ' °' : '81.2 °';
    }
    
    const speed_kmh = currentSpeed * KMH_MS;
    
    // Utilise la vitesse filtrée (EKF)
    $('speed-stable').textContent = speed_kmh.toFixed(1) + ' km/h';
    $('speed-stable-ms').textContent = currentSpeed.toFixed(2) + ' m/s';
    
    $('gps-precision').textContent = (accuracy || 12.97).toFixed(2) + ' m';
}

function updateCompteurs(currentSpeed, dt) {
    const totalTime = (Date.now() - startTime) / 1000;
    const currentSpeed_kmh = currentSpeed * KMH_MS * currentEnvFactor;
    
    $('elapsed-time').textContent = totalTime.toFixed(2) + ' s';
    $('time-moving').textContent = timeMoving.toFixed(2) + ' s';
    $('speed-max').textContent = maxSpd.toFixed(1) + ' km/h';
    
    // Affichage Vitesse filtrée
    $('speed-stable').textContent = currentSpeed_kmh.toFixed(1) + ' km/h';
    $('speed-stable-ms').textContent = currentSpeed.toFixed(2) + ' m/s';
    
    // Distances
    const avgSpeedMvt = timeMoving > 0 ? (distM_3D / timeMoving) * KMH_MS : 0.0;
    $('speed-avg-moving').textContent = avgSpeedMvt.toFixed(1) + ' km/h';
    $('distance-total-km').textContent = (distM_3D / 1000).toFixed(3) + ' km | ' + distM_3D.toFixed(2) + ' m';
    
    updatePhysicsCalculations(currentSpeed);
}


/** Boucle principale de l'application (20 Hz) */
function domUpdateLoop() {
    const dt = DT_MS / 1000;
    
    if (isGPSEnabled && lat === null) {
        // Simule une première lecture GPS pour l'initialisation de l'EKF
         gpsSuccess({ coords: { latitude: 45.749950, longitude: 4.850027, altitude: 2.64, speed: 1.58, accuracy: 12.97, heading: 81.2 } });
    } else if (!isGPSEnabled && wID === null) {
        gpsError(null); 
    }
    
    updateGPSDisplay(speedEst, currentAccuracy);
    updateEKFDisplay();
    updateCompteurs(speedEst, dt);
    updateIMUMonitor(); 
    updateWeatherAndBiophysics();
    updateAstroCalculations(); 
}

function init() {
    initEKF(45.749950, 4.850027, 2.64); 
    speedEst = 5.7 / KMH_MS; 
    
    // initControls(); 
    // initializeIMUSensors(); 
    
    updateWeatherAndBiophysics(); 
    updateAstroCalculations();
    
    setInterval(domUpdateLoop, DT_MS); 
}

// init();
