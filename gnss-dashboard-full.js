// =========================================================================
// _constants.js : Constantes et État Global
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
const MIN_SPD = 0.05;           // Vitesse minimale pour être considéré en mouvement (m/s)
const N_STATES = 21;            // Nombre d'états du vecteur EKF

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
// X: [0:lat, 1:lon, 2:alt, 3:vx, 4:vy, 5:vz, 6:q0, 7:q1, 8:q2, 9:q3, 10:bgx, 11:bgy, 12:bgz, 13:bax, 14:bay, 15:baz, 16:m0, 17:m1, 18:m2, 19:m3, 20:m4]
let X = math.matrix(math.zeros(N_STATES)._data.flat()); 
let P = math.diag(math.zeros(N_STATES)._data.flat());   

let Q_diag = new Array(N_STATES).fill(1e-6); 
Q_diag[0] = Q_diag[1] = Q_diag[2] = 1e-4; // Pos
Q_diag[3] = Q_diag[4] = Q_diag[5] = 10.0; // Vel: Augmenté pour gérer le drift en Dead Reckoning
let Q = math.diag(Q_diag); // Matrice de covariance du bruit de processus

// --- ÉTAT DES CAPTEURS ET COMPTEURS ---
let lat = null, lon = null, altEst = null, speedEst = 0.0; 
let currentAccuracy = null;
let currentHeading = null;
let lastGPSPosition = null;

let imuAccel = {x: 0, y: 0, z: 0};  
let imuGyro = {x: 0, y: 0, z: 0};   

// Valeurs nominales statiques par défaut
let tempC = 20.0, pressurehPa = 1013.25, humidityPerc = 50.0; 
let currentMass = 70.0;
let gpsAccuracyOverride = 0.0;
let startTime = Date.now();

let distM_3D = 0.0;
let timeMoving = 0.0;
let maxSpd = 0.0;

let magFieldMax = 0.0; 
let angularSpeed = 0.0;
// =========================================================================
// _ekf_core.js : Moteur EKF, Prédiction, Correction et Gestion Capteurs
// =========================================================================

/** Initialise l'état EKF. */
function initEKF(lat_init, lon_init, alt_init) {
    lat = lat_init !== null ? lat_init * D2R : 45.75 * D2R; 
    lon = lon_init !== null ? lon_init * D2R : 4.85 * D2R; 
    altEst = alt_init !== null ? alt_init : 0.0;
    
    X.set([0], lat); X.set([1], lon); X.set([2], altEst);
    P = math.diag(Q_diag);
}

/** Étape de Prédiction de l'EKF. */
function EKF_predict(dt) {
    // Correction: Utiliser les indices corrects pour les biais d'accéléromètre (13, 14, 15)
    const ba_accel = [X.get([13]), X.get([14]), X.get([15])]; 
    // Correction de l'accélération par le biais du biais
    const accel_corrected_total = math.subtract([imuAccel.x, imuAccel.y, imuAccel.z], ba_accel);

    // 1. Vitesse (Propagation)
    const V_xyz = [X.get([3]), X.get([4]), X.get([5])];
    const altM = X.get([2]);

    // Calcul de la gravité locale basée sur l'altitude EKF (dynamique)
    const g_local = G_BASE * Math.pow(EARTH_RADIUS / (EARTH_RADIUS + altM), 2);
    
    // Accélération linéaire en NED (assumant alignement body=NED, Z=Down)
    // a_linear = a_IMU_total - g_NED. Gravity is [0, 0, g_local] en NED (Down est Z positif)
    const gravity_NED = [0, 0, g_local]; 
    const linear_accel_NED = math.subtract(accel_corrected_total, gravity_NED); 

    const dV = math.multiply(linear_accel_NED, dt);
    const new_V_xyz = math.add(V_xyz, dV);
    X.set([3], new_V_xyz[0]); X.set([4], new_V_xyz[1]); X.set([5], new_V_xyz[2]);
    
    // 2. Position (Propagation Géodétique Corrigée)
    const latRad = X.get([0]);
    // const altM est déjà calculée
    
    const Vn = X.get([3]); // V North
    const Ve = X.get([4]); // V East
    const Vd = X.get([5]); // V Down (Down = -Vz)
    
    const R_M = EARTH_RADIUS + altM; // Rayon méridien
    const R_N_prime = (EARTH_RADIUS + altM) * Math.cos(latRad); // Rayon normal prime
    
    const dLat = (Vn * dt) / R_M;
    const dLon = (Ve * dt) / R_N_prime;
    const dAlt = -Vd * dt; // Alt augmente si Vd est négatif (Up)

    // Propagation (Lat/Lon/Alt) - Applique le facteur d'environnement
    X.set([0], latRad + dLat * currentEnvFactor); 
    X.set([1], X.get([1]) + dLon * currentEnvFactor); 
    X.set([2], altM + dAlt * currentEnvFactor); 

    // 3. Mise à jour de la Covariance (Simplifié: P = P + Q)
    P = math.add(P, Q); 
    
    // Mise à jour des variables d'affichage
    lat = X.get([0]); 
    lon = X.get([1]);
    altEst = X.get([2]);
    speedEst = math.norm([X.get([3]), X.get([4]), X.get([5])]);
}

/** Étape de Correction (Mise à jour) de l'EKF. */
function EKF_update(z_meas, z_h, H, R) {
    const H_t = math.transpose(H);
    const S = math.add(math.multiply(H, P, H_t), R);
    const K = math.multiply(P, H_t, math.inv(S));

    const error = math.subtract(z_meas, z_h);
    const correction = math.multiply(K, error);
    X = math.add(X, correction);

    const I = math.identity(N_STATES);
    const I_KH = math.subtract(I, math.multiply(K, H));
    // Formule Joseph (stabliisée)
    P = math.multiply(I_KH, P, math.transpose(I_KH), math.multiply(K, R, math.transpose(K))); 
}

/** Gestion du signal GPS réussi (Correction GNSS de l'EKF). */
function gpsSuccess(position) {
    const dt = DT_MS / 1000;
    
    if (lat === null) {
        initEKF(position.coords.latitude, position.coords.longitude, position.coords.altitude || 0);
    }
    
    EKF_predict(dt); 

    let R_val_base = (gpsAccuracyOverride > 0) ? gpsAccuracyOverride : (position.coords.accuracy || 10);
    let R_val = R_val_base * currentEnvFactor; 
    
    currentAccuracy = R_val;
    currentHeading = position.coords.heading;
    
    const speedRaw = position.coords.speed || 0;
    const headingRad = (currentHeading !== null) ? currentHeading * D2R : 0;
    
    const V_North_Approx = speedRaw * Math.cos(headingRad);
    const V_East_Approx = speedRaw * Math.sin(headingRad);
    
    // Mesure (z_meas): [lat, lon, alt, vn, ve, vd]
    const z_gnss = math.matrix([
        position.coords.latitude * D2R, 
        position.coords.longitude * D2R,
        position.coords.altitude || altEst,
        V_North_Approx,
        V_East_Approx,
        0 // V_down n'est pas fiable sur GPS standard
    ]);
    
    // États estimés (z_h)
    const z_h_gnss = math.matrix([X.get([0]), X.get([1]), X.get([2]), X.get([3]), X.get([4]), X.get([5])]);
    
    // Matrice d'Observation (H)
    let H_gnss = math.zeros(6, N_STATES);
    for(let i=0; i<6; i++) H_gnss.set([i, i], 1); 
    
    // Matrice de Covariance des Mesures (R)
    const R_gps_val = R_val * R_val;
    const R_gps = math.diag([R_gps_val, R_gps_val, R_gps_val, 1, 1, 1]); 
    
    EKF_update(z_gnss, z_h_gnss, H_gnss, R_gps); 

    lastGPSPosition = position;
    $('gps-status-dr').textContent = 'ACTIF (FUSION IMU/GPS)';
}

/** Gestion de l'erreur GPS (Dead Reckoning et ZUPT). */
function gpsError(error) {
    const dt = DT_MS / 1000;
    
    if (lat === null) initEKF(null, null, 0); 
    
    EKF_predict(dt); 

    // --- CALCUL DE L'ACCÉLÉRATION LINÉAIRE pour ZUPT ---
    const altM = X.get([2]) || 0;
    const g_local = G_BASE * Math.pow(EARTH_RADIUS / (EARTH_RADIUS + altM), 2);
    // Correction: Utiliser les indices corrects pour les biais d'accéléromètre (13, 14, 15)
    const ba_accel = [X.get([13]), X.get([14]), X.get([15])]; 
    const accel_corrected_total = math.subtract([imuAccel.x, imuAccel.y, imuAccel.z], ba_accel);
    // Accélération linéaire (gravité soustraite)
    const linear_accel_NED = math.subtract(accel_corrected_total, [0, 0, g_local]); 
    
    // ZUPT (Correction Vitesse Zéro) - Utiliser l'accélération linéaire
    const accel_mag_linear = math.norm(linear_accel_NED);
    
    // Correction: ZUPT est appliqué si l'accélération linéaire est faible. 
    // La vérification de 'speedEst < MIN_SPD' est retirée.
    if (accel_mag_linear < ZUPT_ACCEL_TOLERANCE) {
        // L'objet est immobile: Vitesse réelle = 0
        const z_zupt = math.matrix([0, 0, 0]); 
        const z_h_zupt = math.matrix([X.get([3]), X.get([4]), X.get([5])]); 
        
        let H_zupt = math.zeros(3, N_STATES);
        H_zupt.set([0, 3], 1); // Vx
        H_zupt.set([1, 4], 1); // Vy
        H_zupt.set([2, 5], 1); // Vz
        
        const R_zupt = math.diag([0.01, 0.01, 0.01]); 
        EKF_update(z_zupt, z_h_zupt, H_zupt, R_zupt); 
        
        $('speed-status-text').textContent = 'Mode Dead Reckoning (ZUPT)';
        currentAccuracy = Math.sqrt(P.get([0, 0])) * R2D * EARTH_RADIUS;
    } else {
        $('speed-status-text').textContent = 'Mode Dead Reckoning (IMU Seul)';
        currentAccuracy = null;
    }

    // Correction du statut GPS pour refléter l'arrêt explicite du GPS
    if (wID === null && !isGPSEnabled) {
        $('gps-status-dr').textContent = 'Arrêté (Pause/Manuel)';
    } else {
         $('gps-status-dr').textContent = 'ERREUR GPS - DR: IMU Seul';
    }
}

/** Initialise et écoute les capteurs IMU (Accéléromètre et Gyroscope). */
function initializeIMUSensors() {
    if ('ondevicemotion' in window) {
        window.addEventListener('devicemotion', (event) => {
            if (event.accelerationIncludingGravity) {
                // Met à jour les variables d'état imuAccel et imuGyro
                imuAccel.x = event.accelerationIncludingGravity.x || 0;
                imuAccel.y = event.accelerationIncludingGravity.y || 0;
                imuAccel.z = event.accelerationIncludingGravity.z || 0;
                
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
// _main_dom.js : Logique Physique, Astro, Rendu DOM et Boucle Principale
// =========================================================================

// --- CALCULS PHYSIQUES AVANCÉS ---

function calculateSpeedOfSound(tempC) {
    if (tempC === null || isNaN(tempC)) return null; 
    return 20.0468 * Math.sqrt(tempC + KELVIN_OFFSET);
}

function calculateAirDensity(pressurehPa, tempC) {
    if (pressurehPa === null || tempC === null) return null; 
    const R_SPECIFIQUE = 287.05; 
    return (pressurehPa * 100) / (R_SPECIFIQUE * (tempC + KELVIN_OFFSET));
}

function calculateDewPoint(tempC, humidity) {
    const A = 17.27;
    const B = 237.7;
    const alfa = (A * tempC) / (B + tempC) + Math.log(humidity / 100);
    return (B * alfa) / (A - alfa);
}

/** Mise à jour des données Météo (utilise des valeurs nominales si pas de capteur réel). */
function updateWeatherAndBiophysics() {
    // Les valeurs sont nominales statiques
    const air_density = calculateAirDensity(pressurehPa, tempC);
    const dew_point = calculateDewPoint(tempC, humidityPerc);
    
    // Rendu DOM
    $('temp-air-2').textContent = tempC.toFixed(1) + ' °C (Nom.)';
    $('pressure-2').textContent = pressurehPa.toFixed(2) + ' hPa (Nom.)';
    $('humidity-2').textContent = humidityPerc.toFixed(1) + ' % (Nom.)';
    $('dew-point').textContent = dew_point !== null ? dew_point.toFixed(1) + ' °C' : 'N/A';
    $('air-density').textContent = air_density !== null ? air_density.toFixed(3) + ' kg/m³' : 'N/A';
    
    // Correction: Altitude barométrique affichée comme l'altitude EKF
    const alt_baro = altEst !== null ? altEst : null;
    $('alt-baro').textContent = alt_baro !== null ? alt_baro.toFixed(2) + ' m' : 'N/A';
    
    $('weather-status').textContent = 'INACTIF (Données nominales)'; 
}

/** Mise à jour des indicateurs IMU (appelé dans la boucle principale pour la cohérence) */
function updateIMUMonitor() {
    // Assure la cohérence des valeurs d'accélération (X et Long.)
    $('accel-x').textContent = imuAccel.x.toFixed(2) + ' m/s²';
    $('accel-y').textContent = imuAccel.y.toFixed(2) + ' m/s²';
    $('accel-z').textContent = imuAccel.z.toFixed(2) + ' m/s²';
    
    // Accélération Longitudinale (basée sur l'axe X du dispositif)
    $('accel-long').textContent = imuAccel.x !== 0 ? imuAccel.x.toFixed(2) + ' m/s²' : '0.00 m/s²'; 
}

function updatePhysicsCalculations(currentSpeed) {
    const spd_sound = calculateSpeedOfSound(tempC);
    const air_density = calculateAirDensity(pressurehPa, tempC);
    
    // Gravité locale basée sur l'altitude EKF
    const g_local = altEst !== null ? G_BASE * Math.pow(EARTH_RADIUS / (EARTH_RADIUS + altEst), 2) : G_BASE;
    $('gravity-local').textContent = g_local.toFixed(5) + ' m/s²';

    $('angular-speed').textContent = angularSpeed !== 0 ? angularSpeed.toFixed(2) + ' °/s' : '0.00 °/s';
    
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

// --- CALCULS ASTRO ET AFFICHAGE ---

function formatHours(decimalHours) {
    if (decimalHours === null || isNaN(decimalHours)) return 'N/A';
    decimalHours = (decimalHours % 24 + 24) % 24;
    const hours = Math.floor(decimalHours);
    const minutes = Math.floor((decimalHours % 1) * 60);
    return `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}`;
}

function formatMinecraftTime(ticks) {
    if (ticks === null || isNaN(ticks)) return 'N/A';
    ticks = (ticks % 24000 + 24000) % 24000; 
    let mc_hours = Math.floor(ticks / 1000) % 24;
    let mc_minutes = Math.floor(((ticks % 1000) / 1000) * 60);
    return `${mc_hours.toString().padStart(2, '0')}:${mc_minutes.toString().padStart(2, '0')}`;
}

function getMoonPhaseName(phase) {
    if (phase < 0.03 || phase > 0.97) return 'Nouvelle Lune';
    if (phase < 0.22) return 'Croissant Mince';
    if (phase < 0.28) return 'Premier Quartier';
    if (phase < 0.47) return 'Gibbeuse Montante';
    if (phase < 0.53) return 'Pleine Lune';
    if (phase < 0.72) return 'Gibbeuse Décroissante';
    if (phase < 0.78) return 'Dernier Quartier';
    return 'Vieux Croissant';
}

function updateAstroCalculations() {
    const coords = { lat: lat * R2D || 45.75, lon: lon * R2D || 4.85 };
    const date = new Date();
    
    // Correction: Rétablir la synchronisation GMT (assuré dans l'itération précédente, reconfirmé)
    $('local-time').textContent = date.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit', second: '2-digit' });
    
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
        const sunTimes = SunCalc.getTimes(date, coords.lat, coords.lon);
        const sunPos = SunCalc.getPosition(date, coords.lat, coords.lon);
        const moonPos = SunCalc.getMoonPosition(date, coords.lat, coords.lon);
        const moonIllumination = SunCalc.getMoonIllumination(date);
        
        // Calcul des Temps Solaires
        const UTC_hours_dec = date.getUTCHours() + date.getUTCMinutes() / 60 + date.getUTCSeconds() / 3600;
        
        const EOT_rad = sunPos.equationOfTime; 
        let EOT_minutes = null;
        let TST_hours = null;

        const MST_hours = UTC_hours_dec + coords.lon / 15;

        // Correction: L'EOT est en radians dans SunCalc.getPosition. 
        // Conversion Radian -> Minutes (1 rad = 3.8197 heures = 229.18 minutes)
        if (typeof EOT_rad === 'number' && !isNaN(EOT_rad)) {
            EOT_minutes = EOT_rad * (240 / Math.PI); 
            const EOT_hours = EOT_minutes / 60;
            TST_hours = MST_hours + EOT_hours;
        }

        // Affichage des temps solaires (gère le null/NaN via formatHours -> N/A)
        $('mst').textContent = formatHours(MST_hours);
        $('tst').textContent = formatHours(TST_hours);
        
        // Temps Minecraft
        const mc_ticks = TST_hours !== null && !isNaN(TST_hours) ? (TST_hours * 1000 - 6000) % 24000 : null;
        $('time-minecraft').textContent = formatMinecraftTime(mc_ticks);

        // Affichage Astro
        $('sun-alt').textContent = (sunPos.altitude * R2D).toFixed(2) + ' °';
        $('sun-azimuth').textContent = (sunPos.azimuth * R2D).toFixed(2) + ' °';
        $('moon-alt').textContent = (moonPos.altitude * R2D).toFixed(2) + ' °';
        $('moon-azimuth').textContent = (moonPos.azimuth * R2D).toFixed(2) + ' °';
        $('moon-phase-name').textContent = getMoonPhaseName(moonIllumination.phase);
        $('moon-illuminated').textContent = (moonIllumination.fraction * 100).toFixed(1) + ' %';
        
        // EOT et Longitude Écliptique (Gère le null/NaN pour l'affichage)
        $('noon-solar').textContent = sunTimes.solarNoon.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit', timeZone: 'UTC' });
        $('eot').textContent = EOT_minutes !== null && !isNaN(EOT_minutes) ? EOT_minutes.toFixed(2) + ' min' : 'N/A'; 
        $('ecl-long').textContent = isNaN(sunPos.eclipticLongitude * R2D) ? 'N/A' : (sunPos.eclipticLongitude * R2D).toFixed(2) + ' °';
        
        $('date-solar-mean').textContent = date.toLocaleDateString('fr-FR');
        $('date-solar-true').textContent = date.toLocaleDateString('fr-FR');
        
        updateMinecraftClockAnimation(sunPos.altitude, sunPos.azimuth);
    }
}

function updateMinecraftClockAnimation(sunAltitudeRad, sunAzimuthRad) {
    const totalAngle = (-sunAzimuthRad * R2D) + 90; 
    const sunElement = $('sun-element');
    const moonElement = $('moon-element');
    const clockDiv = $('minecraft-clock');
    const sunAltitudeDeg = sunAltitudeRad * R2D;

    if (sunElement && moonElement) {
        sunElement.style.transform = `rotate(${totalAngle}deg) translateX(0) translateY(-50%)`;
        moonElement.style.transform = `rotate(${totalAngle - 180}deg) translateX(0) translateY(-50%)`; 
        
        clockDiv.classList.remove('sky-day', 'sky-sunset', 'sky-night');
        if (sunAltitudeDeg > 5) {
            clockDiv.classList.add('sky-day');
            $('clock-status').textContent = 'Jour (☀️)';
        } else if (sunAltitudeDeg < -15) {
            clockDiv.classList.add('sky-night');
            $('clock-status').textContent = 'Nuit (🌙)';
        } else {
            clockDiv.classList.add('sky-sunset');
            $('clock-status').textContent = 'Crépuscule (🌅)';
        }
        
        clockDiv.classList.toggle('x-ray', isXRayMode);
    }
    $('nether-mode-status').textContent = currentEnvFactor.toFixed(3);
}

// --- RENDU DOM ET CARTE ---

/** Initialise la carte Leaflet. */
function initMap() {
    if (typeof L !== 'undefined') {
        mapInstance = L.map('map').setView([45.75, 4.85], 10); 
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            maxZoom: 19,
            attribution: '© OpenStreetMap'
        }).addTo(mapInstance);
        $('map').textContent = ''; 
    } else {
        $('map').textContent = 'Erreur: Bibliothèque Leaflet non chargée.';
    }
}

/** Met à jour la position du marqueur EKF sur la carte. */
function updateMap(newLatDeg, newLonDeg, accuracy) {
    if (mapInstance && newLatDeg !== null) {
        const newLatLng = [newLatDeg, newLonDeg];
        
        if (isGPSEnabled || distM_3D === 0) {
            mapInstance.setView(newLatLng, mapInstance.getZoom() > 14 ? mapInstance.getZoom() : 15, { animate: true });
        }
        
        if (marker) {
            mapInstance.removeLayer(marker);
        }
        
        marker = L.circle(newLatLng, {
            color: 'red',
            fillColor: '#f03',
            fillOpacity: 0.5,
            radius: accuracy || 10 
        }).addTo(mapInstance);
    }
}

/** Met à jour les compteurs de distance, vitesse max et appels aux calculs physiques. */
function updateCompteurs(currentSpeed, dt) {
    const totalTime = (Date.now() - startTime) / 1000;
    const currentSpeed_kmh = currentSpeed * KMH_MS * currentEnvFactor;
    
    $('elapsed-time').textContent = totalTime.toFixed(2) + ' s';
    
    if (currentSpeed >= MIN_SPD) {
        distM_3D += currentSpeed * dt * currentEnvFactor; 
        timeMoving += dt;
        maxSpd = Math.max(maxSpd, currentSpeed_kmh);
    }
    
    // Affichage Vitesse
    $('speed-stable').textContent = currentSpeed_kmh.toFixed(1) + ' km/h';
    $('speed-stable-ms').textContent = (currentSpeed * currentEnvFactor).toFixed(2) + ' m/s';
    $('speed-stable-kms').textContent = (currentSpeed * currentEnvFactor / 1000).toFixed(4) + ' km/s';
    
    // Distances et Max
    $('time-moving').textContent = timeMoving.toFixed(2) + ' s';
    $('speed-max').textContent = maxSpd.toFixed(1) + ' km/h';
    $('distance-total-km').textContent = (distM_3D / 1000).toFixed(3) + ' km | ' + distM_3D.toFixed(2) + ' m';
    
    const avgSpeedMvt = timeMoving > 0 ? (distM_3D / timeMoving) * KMH_MS : 0.0;
    const avgSpeedTotal = totalTime > 0 ? (distM_3D / totalTime) * KMH_MS : 0.0;
    $('speed-avg-moving').textContent = avgSpeedMvt.toFixed(1) + ' km/h';
    $('speed-avg-total').textContent = avgSpeedTotal.toFixed(1) + ' km/h';
    
    // Distance Maximale Visible (Horizon)
    const horizon_dist = altEst !== null && altEst > 0 ? Math.sqrt(2 * EARTH_RADIUS * altEst) : null;
    $('distance-horizon').textContent = (horizon_dist !== null) ? (horizon_dist / 1000).toFixed(1) + ' km' : 'N/A';
    
    updatePhysicsCalculations(speedEst * currentEnvFactor);
}

/** Met à jour les données de position EKF et la vitesse brute. */
function updateGPSDisplay(coords) {
    if (lat !== null) {
        $('lat-display').textContent = (lat * R2D).toFixed(6) + ' °';
        $('lon-display').textContent = (lon * R2D).toFixed(6) + ' °';
        $('alt-display').textContent = altEst.toFixed(2) + ' m';
        $('geopotential-alt').textContent = altEst.toFixed(2) + ' m';
        $('heading-display').textContent = currentHeading !== null ? currentHeading.toFixed(1) + ' °' : 'N/A';
    }
    
    const speed_3d_inst = (coords && coords.speed || 0) * KMH_MS;
    const speed_raw_ms = coords && coords.speed || 0;
    $('speed-3d-inst').textContent = speed_3d_inst.toFixed(1) + ' km/h';
    $('speed-raw-ms').textContent = speed_raw_ms.toFixed(2) + ' m/s';

    $('gps-precision').textContent = currentAccuracy !== null ? currentAccuracy.toFixed(2) + ' m' : 'N/A';
    
    updateMap(lat * R2D, lon * R2D, currentAccuracy);
}

/** Met à jour les informations de débogage EKF. */
function updateEKFDisplay() {
    const p_vel = P.get([3, 3]);
    const p_alt = P.get([2, 2]);
    
    $('kalman-uncert').textContent = p_vel !== null ? `${p_vel.toPrecision(3)} (m/s)²` : 'N/A';
    $('alt-uncertainty').textContent = p_alt !== null ? `${Math.sqrt(p_alt).toPrecision(3)} m` : 'N/A';
    
    const R_val = currentAccuracy !== null ? currentAccuracy * currentAccuracy : null;
    $('speed-error-perc').textContent = R_val !== null ? R_val.toFixed(1) : 'N/A'; 
    
    const nyquistFreq = 1 / (DT_MS / 1000) / 2;
    $('nyquist-frequency').textContent = nyquistFreq.toFixed(1) + ' Hz';
    
    $('gps-accuracy-display').textContent = `${gpsAccuracyOverride.toFixed(6)} m`;
}

// --- GESTION DES CONTRÔLES ---

function startGPS() {
    if (wID === null && 'geolocation' in navigator) {
        wID = navigator.geolocation.watchPosition(gpsSuccess, gpsError, {
            enableHighAccuracy: true,
            timeout: 5000,
            maximumAge: 0
        });
        isGPSEnabled = true;
        $('toggle-gps-btn').textContent = '⏸️ PAUSE GPS';
        $('gps-status-dr').textContent = 'Initialisation du signal...';
    } else {
         alert("La géolocalisation n'est pas supportée ou n'a pas été autorisée.");
    }
}

function stopGPS() {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
    }
    isGPSEnabled = false;
    $('toggle-gps-btn').textContent = '▶️ MARCHE GPS';
    $('gps-status-dr').textContent = 'Arrêté (Pause/Manuel)';
    $('speed-status-text').textContent = 'GPS en Pause / Dead Reckoning Arrêté.';
}

function initControls() {
    $('toggle-gps-btn').addEventListener('click', () => isGPSEnabled ? stopGPS() : startGPS());
    $('emergency-stop-btn').addEventListener('click', () => { 
        stopGPS();
        $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: ACTIF 🔴";
    });
    $('reset-dist-btn').addEventListener('click', () => { distM_3D = 0; timeMoving = 0; });
    $('reset-max-btn').addEventListener('click', () => { 
        maxSpd = 0; 
        magFieldMax = 0;
    });
    $('reset-all-btn').addEventListener('click', () => { window.location.reload(); });
    
    $('toggle-mode-btn').addEventListener('click', () => {
        isDarkMode = !isDarkMode;
        document.body.classList.toggle('dark-mode', isDarkMode);
        $('toggle-mode-btn').innerHTML = isDarkMode ? '<i class="fas fa-sun"></i> Mode Jour' : '<i class="fas fa-moon"></i> Mode Nuit';
    });
    
    $('xray-button').addEventListener('click', () => {
        isXRayMode = !isXRayMode;
        $('minecraft-clock').classList.toggle('x-ray', isXRayMode);
        $('xray-button').textContent = isXRayMode ? 'ON' : 'X';
    });

    $('gps-accuracy-override').addEventListener('input', (e) => {
        gpsAccuracyOverride = parseFloat(e.target.value) || 0;
    });
    $('mass-input').addEventListener('input', (e) => {
        currentMass = parseFloat(e.target.value) || 70;
        $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
    });
    $('environment-select').addEventListener('change', (e) => {
        const envKey = e.target.value;
        currentEnvFactor = ENV_FACTORS[envKey] || ENV_FACTORS.NORMAL;
        $('env-factor').textContent = e.target.options[e.target.selectedIndex].text + ' (x' + currentEnvFactor.toFixed(1) + ')';
    });
}

// --- BOUCLES ET INITIALISATION ---

/** Boucle principale de l'application (20 Hz) */
function domUpdateLoop() {
    const dt = DT_MS / 1000;
    
    // Logique EKF/DR
    if (!isGPSEnabled && wID === null) {
        // Appelle gpsError pour faire le Dead Reckoning (DR) et ZUPT lorsque le GPS est en pause/inactif
        gpsError(null); 
    }
    
    // Rendu DOM
    if (lastGPSPosition) {
        updateGPSDisplay(lastGPSPosition.coords);
    } else {
        updateGPSDisplay(null); 
    }
    
    updateEKFDisplay();
    updateCompteurs(speedEst, dt);
    updateIMUMonitor(); // Mise à jour centralisée de l'IMU (Accélération X et Long.)
}

/** Point d'entrée de l'application */
function init() {
    initControls();
    initializeIMUSensors();
    initEKF(null, null, 0); 
    initMap();
    
    // Les appels conservent les valeurs nominales statiques
    updateWeatherAndBiophysics(); 
    updateAstroCalculations();
    
    // Boucle rapide (EKF Predict + DOM)
    setInterval(domUpdateLoop, DT_MS); 
    
    // Boucle lente (Astro, Météo (si non temps réel))
    setInterval(updateAstroCalculations, 10000);
    setInterval(updateWeatherAndBiophysics, 60000); 
}

document.addEventListener('DOMContentLoaded', init);
