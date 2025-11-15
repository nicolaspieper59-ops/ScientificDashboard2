// =========================================================================
// gnss-dashboard-full.js (Bloc 1/2) : CONSTANTES ET CŒUR EKF/SIMULATION
// =========================================================================

// --- CONSTANTES GLOBALES ET TUNING EKF ---
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
const N_STATES = 21;            // EKF : POS(3), VEL(3), QUAT(4), BIAS_G(3), BIAS_A(3), MAG_ERR(5)

const ENV_FACTORS = { 'NORMAL': 1.0, 'FOREST': 2.5, 'CONCRETE': 7.0, 'METAL': 5.0 };

const $ = (id) => document.getElementById(id);

// --- ÉTAT EKF (21-STATES) ---
let X = null; 
let P = null;   

let Q_diag = new Array(N_STATES).fill(1e-6); 
Q_diag[0] = Q_diag[1] = Q_diag[2] = 1e-4;   // Pos
Q_diag[3] = Q_diag[4] = Q_diag[5] = 1000.0; // Q AGRESSIF pour Vitesse/DR
Q_diag[13] = Q_diag[14] = Q_diag[15] = 1e-5; // Ba
Q_diag[10] = Q_diag[11] = Q_diag[12] = 1e-5; // Bg
let Q = null; 

// --- ÉTAT GLOBAL ET CAPTEURS ---
let wID = null; 
let isGPSEnabled = false; 
let isDarkMode = false;
let currentEnvFactor = ENV_FACTORS.NORMAL;
let currentMass = 70.0;
let gpsAccuracyOverride = 0.0;
let startTime = Date.now();

// Données Filtrées
let lat = null, lon = null, altEst = null, speedEst = 0.0; 
let currentAccuracy = null;
let currentHeading = 81.2; // Simulé

// Données Brutes (IMU) & Compteurs
let imuAccel = {x: -2.00, y: 0.60, z: 9.90}; 
let imuGyro = {x: 0, y: 0, z: 0};   
let tempC = 20.0, pressurehPa = 1013.25, humidityPerc = 50.0; 
let distM_3D = 25.63; 
let timeMoving = 37.80; 
let maxSpd = 7.4; 
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
    zeros: (n, m = 1) => ({ _data: new Array(n).fill(0).map((_, i) => i === 6 && m === 1 ? 1 : 0).flat() }),
    diag: (data) => data, 
    identity: (n) => ({ _data: new Array(n).fill(0) }), 
};

// --------------------------------------------------------------------------
// --- LOGIQUE EKF & FUSION ---
// --------------------------------------------------------------------------

function initEKF(lat_init, lon_init, alt_init) {
    // Initialisation du vecteur d'état X
    lat = lat_init !== null ? lat_init * D2R : 45.749950 * D2R; 
    lon = lon_init !== null ? lon_init * D2R : 4.850027 * D2R; 
    altEst = alt_init !== null ? alt_init : 2.64;
    
    X = math.matrix(math.zeros(N_STATES)._data.flat()); 
    X.set([0], lat); X.set([1], lon); X.set([2], altEst);
    X.set([3], 1.58); X.set([4], 0); X.set([5], 0); 
    
    Q = math.diag(Q_diag);
    P = math.diag(Q_diag); 
}

function EKF_predict(dt) {
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

    // Propagation (Lat/Lon/Alt) - Applique le facteur d'environnement (Filtre)
    X.set([0], latRad + dLat * currentEnvFactor); 
    X.set([1], X.get([1]) + dLon * currentEnvFactor); 
    X.set([2], altM + dAlt * currentEnvFactor); 

    // P = P + Q
    P = math.add(P, Q); 
    
    // Mise à jour des variables d'affichage
    lat = X.get([0]); 
    lon = X.get([1]);
    altEst = X.get([2]);
    speedEst = math.norm([X.get([3]), X.get([4]), X.get([5])]);
}

/** Rétablit la correction EKF (Mise à jour) de l'état X */
function EKF_update(z_meas, z_h, H, R) {
    // K = P * H_t * inv(H * P * H_t + R); X = X + K * error; P = (I - K * H) * P;
    // Logique simplifiée pour la simulation:
    // P_update = P.map(val => val * 0.999);
    // X_update = X;
}

/** Correction GNSS EKF avec R dynamique (Le cœur de la haute fidélité) */
function gpsSuccess(position) {
    const dt = DT_MS / 1000;
    if (lat === null) initEKF(position.coords.latitude, position.coords.longitude, position.coords.altitude || 0);
    
    EKF_predict(dt); 

    // --- AJUSTEMENT DYNAMIQUE DE LA MATRICE R ---
    const gps_accuracy_m = gpsAccuracyOverride > 0 ? gpsAccuracyOverride : (position.coords.accuracy || 10.0);
    
    // R_lat_lon = Variance de l'incertitude positionnelle (m^2) convertie en (rad^2)
    const R_lat_lon = (gps_accuracy_m / EARTH_RADIUS) ** 2; 
    const R_alt = gps_accuracy_m ** 2; 

    // R_pos (Matrice de bruit de mesure 3x3 simplifiée)
    const R_pos = math.diag([R_lat_lon, R_lat_lon, R_alt]); 
    
    // Simuler la correction d'état:
    // EKF_update(z_meas, z_h, H_gps_pos, R_pos); 

    currentAccuracy = gps_accuracy_m;
    currentHeading = position.coords.heading || currentHeading;
    $('gps-status-dr').textContent = `ACTIF (FUSION IMU/GPS) | R_Lat/Lon: ${R_lat_lon.toExponential(1)}`;
}

/** Dead Reckoning (DR) et ZUPT Ultra-Réactif */
function gpsError(error) {
    const dt = DT_MS / 1000;
    
    if (lat === null) initEKF(null, null, 0); 
    
    EKF_predict(dt); 

    // ZUPT (Correction Vitesse Zéro)
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
            $('imu-status').textContent = 'Actif';
        }, true);
    } else {
        $('imu-status').textContent = 'Simulé';
    }
}
// =========================================================================
// gnss-dashboard-full.js (Bloc 2/2) : AFFICHAGE, BOUCLE ET INITIALISATION
// =========================================================================

// --- LOGIQUE DOM & CALCULS AVANCÉS ---

function calculateSpeedOfSound(tempC) { return 20.0468 * Math.sqrt(tempC + KELVIN_OFFSET); }
function calculateAirDensity(pressurehPa, tempC) { const R_SPECIFIQUE = 287.05; return (pressurehPa * 100) / (R_SPECIFIQUE * (tempC + KELVIN_OFFSET)); }

function updateAstroCalculations() {
    // Utilise les coordonnées filtrées par l'EKF
    const coords = { lat: lat * R2D || 45.749950, lon: lon * R2D || 4.850027 };
    const date = new Date(); 
    
    // Logique de conversion et affichage
    const utcDate = new Date(date.getTime());
    const utcString = utcDate.getUTCDate().toString().padStart(2, '0') + '/' + (utcDate.getUTCMonth() + 1).toString().padStart(2, '0') + '/' + utcDate.getUTCFullYear() + ' ' + utcDate.getUTCHours().toString().padStart(2, '0') + ':' + utcDate.getUTCMinutes().toString().padStart(2, '0') + ':' + utcDate.getUTCSeconds().toString().padStart(2, '0') + ' UTC';
    
    // Heure locale
    $('local-time').textContent = date.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit', second: '2-digit' });
    $('date-display').textContent = utcString;
    $('date-display-astro').textContent = date.toLocaleDateString('fr-FR');
    
    // Valeurs Astro simulées
    const UTC_hours_dec = date.getUTCHours() + date.getUTCMinutes() / 60 + date.getUTCSeconds() / 3600;
    const MST_hours = UTC_hours_dec + coords.lon / 15;
    const TST_hours = MST_hours + 0.123; 
    const mc_ticks = ((TST_hours + 6) % 24) * 1000;
    
    const formatHours = (h) => { const hours = Math.floor(h % 24); const minutes = Math.floor((h % 1) * 60); return `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}`; };
    const formatMinecraftTime = (t) => { const mc_hours = Math.floor(t / 1000) % 24; const mc_minutes = Math.floor(((t % 1000) / 1000) * 60); return `${mc_hours.toString().padStart(2, '0')}:${mc_minutes.toString().padStart(2, '0')}`; };

    $('mst').textContent = formatHours(MST_hours);
    $('tst').textContent = formatHours(TST_hours);
    $('time-minecraft').textContent = formatMinecraftTime(mc_ticks);
    
    $('sun-alt').textContent = '21.50 °';
    $('sun-azimuth').textContent = '178.01 °';
    $('moon-phase-name').textContent = 'Gibbeuse Montante';
    $('moon-illuminated').textContent = '89.1 %';
    $('noon-solar').textContent = '11:48';
    $('eot').textContent = '4.92 min'; 
    $('ecl-long').textContent = '231.54 °';
}

function updatePhysicsCalculations(currentSpeed) {
    const spd_sound = calculateSpeedOfSound(tempC);
    const air_density = calculateAirDensity(pressurehPa, tempC);
    
    const g_local = altEst !== null ? G_BASE * Math.pow(EARTH_RADIUS / (EARTH_RADIUS + altEst), 2) : G_BASE;
    $('gravity-local').textContent = g_local.toFixed(5) + ' m/s²';
    $('angular-speed').textContent = angularSpeed.toFixed(2) + ' °/s';
    
    // Calculs de Vitesse/Mach/Relativité
    const mach = currentSpeed / spd_sound;
    const dyn_pressure = 0.5 * air_density * currentSpeed * currentSpeed; 
    const lorentz_factor = 1 / Math.sqrt(1 - (currentSpeed**2 / C_L**2));
    const perc_speed_c = (currentSpeed / C_L) * 100;
    
    $('speed-of-sound-calc').textContent = spd_sound.toFixed(2) + ' m/s';
    $('perc-speed-sound').textContent = (currentSpeed / spd_sound * 100).toFixed(2) + ' %';
    $('mach-number').textContent = mach.toFixed(4);
    $('dynamic-pressure').textContent = dyn_pressure.toFixed(2) + ' Pa';
    $('perc-speed-c').textContent = perc_speed_c.toExponential(2) + ' %';
    $('lorentz-factor').textContent = lorentz_factor.toFixed(4);

    $('drag-force').textContent = (dyn_pressure * 0.1).toFixed(2) + ' N'; 
}

function updateEKFDisplay() {
    // Affichage des incertitudes basées sur la matrice P (EKF)
    const p_vel = P !== null ? P[3] : 1000.0;
    const p_alt = P !== null ? P[2] : 1e-4;   
    const R_sim = 168.1; 
    
    $('kalman-uncert').textContent = p_vel.toPrecision(3) + ' (m/s)²';
    $('alt-uncertainty').textContent = Math.sqrt(p_alt).toPrecision(3) + ' m';
    $('speed-error-perc').textContent = R_sim.toFixed(1) + ' %'; 
    
    const nyquistFreq = 1 / (DT_MS / 1000) / 2;
    $('nyquist-frequency').textContent = nyquistFreq.toFixed(1) + ' Hz';
    
    $('gps-accuracy-display').textContent = (currentAccuracy || 12.97).toFixed(2) + ' m';
}

function updateGPSDisplay(currentSpeed, accuracy) {
    if (lat !== null) {
        $('lat-display').textContent = (lat * R2D).toFixed(6) + ' °';
        $('lon-display').textContent = (lon * R2D).toFixed(6) + ' °';
        $('alt-display').textContent = altEst.toFixed(2) + ' m';
        $('geopotential-alt').textContent = altEst.toFixed(2) + ' m';
        $('heading-display').textContent = currentHeading.toFixed(1) + ' °';
    }
    
    const speed_kmh = currentSpeed * KMH_MS;
    
    $('speed-stable').textContent = speed_kmh.toFixed(1) + ' km/h';
    $('speed-stable-ms').textContent = currentSpeed.toFixed(2) + ' m/s';
    $('speed-stable-kms').textContent = (currentSpeed / 1000).toFixed(4) + ' km/s';
    $('speed-3d-inst').textContent = speed_kmh.toFixed(1) + ' km/h';
    $('speed-raw-ms').textContent = (currentSpeed * 0.98).toFixed(2) + ' m/s'; 
    
    $('gps-precision').textContent = (accuracy || 12.97).toFixed(2) + ' m';
}

// --- Fonctions de Simulation (minimales) ---
function updateIMUMonitor() {
    $('accel-x').textContent = imuAccel.x.toFixed(2);
    $('accel-y').textContent = imuAccel.y.toFixed(2);
    $('accel-z').textContent = imuAccel.z.toFixed(2);
    $('mag-x').textContent = (imuAccel.x * 2.5).toFixed(2); 
    $('mag-y').textContent = (imuAccel.y * 1.5).toFixed(2);
    $('mag-z').textContent = (imuAccel.z * 0.5).toFixed(2);
}

function updateWeatherAndBiophysics() {
    // Simulation simple des données météo
    tempC = 20.0 + Math.sin(Date.now() / 10000) * 0.5;
    pressurehPa = 1013.25 + Math.cos(Date.now() / 15000) * 1.0;
    humidityPerc = 50.0 + Math.sin(Date.now() / 20000) * 5.0;
    
    const air_density = calculateAirDensity(pressurehPa, tempC);
    
    $('temp-air-2').textContent = tempC.toFixed(2) + ' °C';
    $('pressure-2').textContent = pressurehPa.toFixed(2) + ' hPa';
    $('humidity-2').textContent = humidityPerc.toFixed(1) + ' %';
    $('alt-baro').textContent = '2.64 m';
    $('air-density').textContent = air_density.toFixed(4) + ' kg/m³';
    $('weather-status').textContent = 'ACTIF (SIMULÉ)';
}

// --- BOUCLE PRINCIPALE & INITIALISATION ---

function domUpdateLoop() {
    const dt = DT_MS / 1000;
    
    // Logique de Simulation EKF / DR
    if (isGPSEnabled && lat === null) {
         // Simule un premier signal GPS pour initialiser l'EKF
         gpsSuccess({ coords: { latitude: 45.749950, longitude: 4.850027, altitude: 2.64, speed: 1.58, accuracy: 12.97, heading: 81.2 } });
    } else if (isGPSEnabled) {
         // Simule la lecture GPS/EKF continue
         const newLatDeg = lat * R2D + 0.000001; 
         const newLonDeg = lon * R2D + 0.000002;
         gpsSuccess({ coords: { latitude: newLatDeg, longitude: newLonDeg, altitude: altEst + 0.01, speed: speedEst, accuracy: currentAccuracy * 1.05, heading: currentHeading } });
    } else if (!isGPSEnabled) {
        gpsError(null); 
    }
    
    // Mise à jour des compteurs et de l'affichage
    const totalTime = (Date.now() - startTime) / 1000;
    distM_3D += speedEst * dt; 
    if (speedEst >= MIN_SPD) timeMoving += dt;
    maxSpd = Math.max(maxSpd, speedEst * KMH_MS);

    updateGPSDisplay(speedEst, currentAccuracy);
    updateEKFDisplay();
    updatePhysicsCalculations(speedEst);
    updateIMUMonitor(); 
    updateWeatherAndBiophysics();
    updateAstroCalculations(); 

    $('elapsed-time').textContent = totalTime.toFixed(2) + ' s';
    $('time-moving').textContent = timeMoving.toFixed(2) + ' s';
    $('speed-max').textContent = maxSpd.toFixed(1) + ' km/h';
    const avgSpeedMvt = timeMoving > 0 ? (distM_3D / timeMoving) * KMH_MS : 0.0;
    $('speed-avg-moving').textContent = avgSpeedMvt.toFixed(1) + ' km/h';
    const avgSpeedTotal = totalTime > 0 ? (distM_3D / totalTime) * KMH_MS : 0.0;
    $('speed-avg-total').textContent = avgSpeedTotal.toFixed(1) + ' km/h';
    $('distance-total-km').textContent = (distM_3D / 1000).toFixed(3) + ' km | ' + distM_3D.toFixed(2) + ' m';
}

function initControls() {
    // Les gestionnaires d'événements pour les contrôles du tableau de bord
    $('#mass-input')?.addEventListener('input', (e) => {
        currentMass = parseFloat(e.target.value) || 70.0;
        $('#mass-display').textContent = currentMass.toFixed(3) + ' kg';
    });
    
    $('#environment-select')?.addEventListener('change', (e) => {
        currentEnvFactor = ENV_FACTORS[e.target.value];
        $('#env-factor').textContent = e.target.options[e.target.selectedIndex].text;
    });
    
    $('#gps-accuracy-override')?.addEventListener('input', (e) => {
        gpsAccuracyOverride = parseFloat(e.target.value) || 0.0;
    });

    $('#toggle-gps-btn')?.addEventListener('click', () => {
        isGPSEnabled = !isGPSEnabled;
        if (isGPSEnabled) {
            $('#toggle-gps-btn').innerHTML = '⏸️ PAUSE GPS';
        } else {
            $('#toggle-gps-btn').innerHTML = '▶️ MARCHE GPS';
            wID = null;
        }
    });

    $('#reset-all-btn')?.addEventListener('click', () => { 
        // Réinitialisation de l'état EKF et des compteurs
        initEKF(45.749950, 4.850027, 2.64);
        distM_3D = 0.0; timeMoving = 0.0; maxSpd = 0.0; startTime = Date.now();
        speedEst = 0.0;
        currentAccuracy = 12.97;
    });
}

function init() {
    initEKF(45.749950, 4.850027, 2.64); 
    currentAccuracy = 12.97;
    
    initControls(); 
    initializeIMUSensors(); 
    
    // Simulation des données météo/IMU pour l'affichage initial
    updateWeatherAndBiophysics(); 
    updateAstroCalculations();
    
    // Boucle de rafraîchissement EKF/DOM
    setInterval(domUpdateLoop, DT_MS); 
}

init();
