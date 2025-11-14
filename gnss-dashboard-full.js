// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;          // Vitesse de la lumière (m/s)
const G_CONSTANT = 6.67430e-11; // Constante gravitationnelle (N·(m/kg)²)
const EARTH_MASS = 5.972e24;    // Masse de la Terre (kg)
const EARTH_RADIUS = 6378137;   // Rayon de la Terre à l'équateur (m)
const OMEGA_EARTH = 7.2921e-5;  // Vitesse angulaire de la Terre (rad/s)
const LIGHT_YEAR_M = 9.461e15;  // Mètres par année-lumière
const AU_M = 1.496e11;          // Mètres par Unité Astronomique
const KMH_MS = 3.6;             
const KELVIN_OFFSET = 273.15;
const G_BASE = 9.80665;         // Gravité standard (m/s²)
const DT_MS = 50;               // Période d'échantillonnage (20 Hz)
const ZUPT_ACCEL_TOLERANCE = 0.5; // Tolérance pour Zero Velocity Update (m/s²)
const MIN_SPD = 0.05;           // Vitesse minimale pour être considéré en mouvement (m/s)
const NETHER_RATIO = 8.0;       // Rapport distance Nether:Overworld (8:1)
const N_STATES = 21;            // Nombre d'états du vecteur EKF

// --- FONCTION UTILITAIRE DOM ---
const $ = (id) => document.getElementById(id);

// --- VARIABLES GLOBALES (ÉTAT DE L'APPLICATION) ---
let wID = null; 
let mapInstance = null;
let marker = null;
let isGPSEnabled = false; 
let isXRayMode = false;
let isDarkMode = false;
let isNetherMode = false;

// --- ÉTAT EKF (21-STATES) ---
let X = math.matrix(math.zeros(N_STATES)._data.flat()); // Vecteur d'état
let P = math.diag(math.zeros(N_STATES)._data.flat());   // Matrice de Covariance
let Q_diag = new Array(N_STATES).fill(1e-6); 
Q_diag[0] = Q_diag[1] = Q_diag[2] = 1e-4; 
Q_diag[3] = Q_diag[4] = Q_diag[5] = 1e-3; 
let Q = math.diag(Q_diag);

// --- ÉTAT DES CAPTEURS ET COMPTEURS ---
let lat = null, lon = null, altEst = null, speedEst = 0.0; 
let currentAccuracy = null;
let lastGPSPosition = null;
let imuAccel = {x: 0, y: 0, z: 0};  // Accélération (m/s²)

let tempC = 20.0, pressurehPa = 1013.25, humidityPerc = 50.0; // Données Météo (initiales/baro)
let magFieldMax = 0.0;
let soundLevelMax = 0.0;
let luminosityMax = 0.0;
let distM_3D = 0.0;
let timeMoving = 0.0;
let maxSpd = 0.0;
let currentMass = 70.0;
let gpsAccuracyOverride = 0.0;
let startTime = Date.now();
/** Initialise l'état EKF et les variables globales. */
function initEKF(lat_init, lon_init, alt_init) {
    lat = lat_init !== null ? lat_init * D2R : 45.0 * D2R; 
    lon = lon_init !== null ? lon_init * D2R : 5.0 * D2R; 
    altEst = alt_init !== null ? alt_init : 0.0;
    X.set([0], lat); X.set([1], lon); X.set([2], altEst);
    P = math.diag(Q_diag);
}

/** Étape de Prédiction de l'EKF. */
function EKF_predict(dt) {
    // ... [Logique de prédiction EKF] ...
    const bg = [X.get([9]), X.get([10]), X.get([11])]; 
    const ba = [X.get([12]), X.get([13]), X.get([14])]; 
    const gyro_corrected = math.subtract([0, 0, 0], bg); // Gyro Simp.
    const accel_corrected = math.subtract([imuAccel.x, imuAccel.y, imuAccel.z], ba);

    const V_xyz = [X.get([3]), X.get([4]), X.get([5])];
    const NED_Accel = [accel_corrected[0], accel_corrected[1], accel_corrected[2]]; 
    const gravity = [0, 0, G_BASE]; 

    const dV = math.multiply(math.subtract(NED_Accel, gravity), dt);
    const new_V_xyz = math.add(V_xyz, dV);
    X.set([3], new_V_xyz[0]); X.set([4], new_V_xyz[1]); X.set([5], new_V_xyz[2]);

    let nether_factor = isNetherMode ? NETHER_RATIO : 1.0; 
    const P_xyz = [X.get([0]), X.get([1]), X.get([2])];
    const dP = math.multiply(V_xyz, dt * nether_factor); // Facteur Nether appliqué ici (pour la DR)
    const new_P_xyz = math.add(P_xyz, dP);
    X.set([0], new_P_xyz[0]); X.set([1], new_P_xyz[1]); X.set([2], new_P_xyz[2]);

    P = math.add(P, Q); 
    
    lat = X.get([0]) * R2D; 
    lon = X.get([1]) * R2D;
    altEst = X.get([2]);
    speedEst = math.norm([X.get([3]), X.get([4]), X.get([5])]);
}

/** Étape de Correction (Mise à jour) de l'EKF. */
function EKF_update(z_meas, z_h, H, R) {
    // ... [Calcul du gain K et mise à jour de X et P] ...
    const H_t = math.transpose(H);
    const S = math.add(math.multiply(H, P, H_t), R);
    const K = math.multiply(P, H_t, math.inv(S));
    const error = math.subtract(z_meas, z_h);
    const correction = math.multiply(K, error);
    X = math.add(X, correction);
    const I = math.identity(N_STATES);
    const I_KH = math.subtract(I, math.multiply(K, H));
    P = math.multiply(I_KH, P);
}

/** Gestion du signal GPS réussi (Correction GNSS de l'EKF). */
function gpsSuccess(position) {
    const dt = DT_MS / 1000;
    if (lat === null) initEKF(position.coords.latitude, position.coords.longitude, position.coords.altitude || 0);
    EKF_predict(dt); 

    let R_val = (gpsAccuracyOverride > 0) ? gpsAccuracyOverride : (position.coords.accuracy || 10);
    currentAccuracy = R_val;
    
    // ... [Mise à jour GNSS] ...
    const speedRaw = position.coords.speed || 0;
    const headingRad = (position.coords.heading !== null) ? position.coords.heading * D2R : 0;
    
    const z_gnss = math.matrix([
        position.coords.latitude * D2R, 
        position.coords.longitude * D2R,
        position.coords.altitude || altEst,
        speedRaw * Math.sin(headingRad), 
        speedRaw * Math.cos(headingRad), 
        0 
    ]);
    
    const z_h_gnss = math.matrix([X.get([0]), X.get([1]), X.get([2]), X.get([3]), X.get([4]), X.get([5])]);
    let H_gnss = math.zeros(6, N_STATES);
    for(let i=0; i<6; i++) H_gnss.set([i, i], 1); 
    
    const R_gps_val = R_val * R_val;
    const R_gps = math.diag([R_gps_val, R_gps_val, R_gps_val, R_gps_val, R_gps_val, R_gps_val]);

    EKF_update(z_gnss, z_h_gnss, H_gnss, R_gps); 
    lastGPSPosition = position;
    $('statut-gps-ekf').textContent = 'ACTIF (FUSION IMU/GPS)';
}

/** Gestion de l'erreur GPS (Dead Reckoning et ZUPT). */
function gpsError(error) {
    const dt = DT_MS / 1000;
    if (lat === null) initEKF(null, null, 0); 
    EKF_predict(dt); 

    const accel_mag = Math.sqrt(imuAccel.x*imuAccel.x + imuAccel.y*imuAccel.y + (imuAccel.z - G_BASE)*(imuAccel.z - G_BASE));
    
    if (accel_mag < ZUPT_ACCEL_TOLERANCE && speedEst < MIN_SPD) {
        // ZUPT Correction
        const z_zupt = math.matrix([0, 0, 0]); 
        const z_h_zupt = math.matrix([X.get([3]), X.get([4]), X.get([5])]); 
        let H_zupt = math.zeros(3, N_STATES);
        for(let i=0; i<3; i++) H_zupt.set([i, i+3], 1); 
        const R_zupt = math.diag([0.01, 0.01, 0.01]); 
        EKF_update(z_zupt, z_h_zupt, H_zupt, R_zupt); 
        $('speed-status-text').textContent = 'Mode Dead Reckoning (ZUPT)';
        $('statut-zupt').textContent = 'ZUPT ACTIF'; 
        currentAccuracy = Math.sqrt(P.get([0, 0])); 
    } else {
        $('speed-status-text').textContent = 'Mode Dead Reckoning (IMU Seul)';
        $('statut-zupt').textContent = 'N/A';
        currentAccuracy = null;
    }
    $('statut-gps-ekf').textContent = 'ERREUR GPS - DR: IMU Seul';
}

/** Initialise et écoute les capteurs IMU (Accéléromètre et Gyroscope). */
function initializeIMUSensors() {
    if ('ondevicemotion' in window) {
        window.addEventListener('devicemotion', (event) => {
            if (event.accelerationIncludingGravity) {
                imuAccel.x = event.accelerationIncludingGravity.x || 0;
                imuAccel.y = event.accelerationIncludingGravity.y || 0;
                imuAccel.z = event.accelerationIncludingGravity.z || 0;

                // Proxy pour le champ magnétique (μT)
                magFieldMax = Math.max(magFieldMax, event.rotationRate ? event.rotationRate.alpha * 100 : 0); 
                
                // Gyroscope omis pour simplifier le 21-états, mais on pourrait ajouter ici :
                // imuGyro.x = event.rotationRate.alpha * D2R; 

                $('sensor-status').textContent = 'Actif / Motion API';
            }
        }, true);
    } else {
        $('sensor-status').textContent = 'Inactif (Simul.)';
    }
    // Simulation simple pour les autres capteurs max (ajustez si vous avez des APIs réelles)
    soundLevelMax = 85.0; 
    luminosityMax = 12000.0;
    }
// --- LOGIQUE MÉTÉO/BIOPHYSIQUE (Assumant que weather.js met à jour les globales) ---

/** Placeholder pour la logique Météo/Biophysique (Simulée) */
function updateWeatherSim() {
    // Cette fonction devrait être implémentée dans weather.js.
    // Elle met à jour les variables globales (tempC, pressurehPa, humidityPerc).
    
    // Si la fonction n'est pas définie dans weather.js, on utilise des valeurs statiques/simulées:
    if (typeof fetchWeather !== 'function') { 
        tempC = 20.0 + Math.sin(Date.now() / 100000) * 5; 
        pressurehPa = 1013.25 + Math.cos(Date.now() / 50000) * 2;
        humidityPerc = 50.0 + Math.sin(Date.now() / 80000) * 10;
    }
    
    const air_density = calculateAirDensity(pressurehPa, tempC);
    const dew_point = calculateDewPoint(tempC, humidityPerc);
    const wet_bulb_temp = calculateWetBulbTemp(tempC, pressurehPa, humidityPerc);
    
    $('temp-air').textContent = tempC.toFixed(1) + ' °C';
    $('pressure-hpa').textContent = pressurehPa.toFixed(2) + ' hPa';
    $('humidity-perc').textContent = humidityPerc.toFixed(1) + ' %';
    $('air-density').textContent = air_density !== null ? air_density.toFixed(3) + ' kg/m³' : 'N/A';
    $('dew-point').textContent = dew_point !== null ? dew_point.toFixed(1) + ' °C' : 'N/A';
    
    // Biophysique
    $('temp-thermometre-mouille').textContent = wet_bulb_temp !== null ? wet_bulb_temp.toFixed(1) + ' °C' : 'N/A';
    $('absolute-humidity').textContent = dew_point !== null ? (absoluteHumidityFromDewPoint(dew_point)).toFixed(3) + ' g/m³' : 'N/A';
    $('o2-saturation-theo').textContent = (100 - (humidityPerc / 2)).toFixed(1) + ' %'; // Théorique
    $('photosynthesis-sim').textContent = (luminosityMax > 10000 ? 'Élevée' : 'Normale');
    $('indice-de-stabilité').textContent = '0 J/kg'; // Simplifié
}

function calculateAirDensity(pressurehPa, tempC) {
    if (pressurehPa === null || tempC === null) return null; 
    const R_SPECIFIQUE = 287.05; 
    return (pressurehPa * 100) / (R_SPECIFIQUE * (tempC + KELVIN_OFFSET));
}

function calculateDewPoint(tempC, humidity) {
    // Formule Magnus (approximation)
    const A = 17.27;
    const B = 237.7;
    const alfa = (A * tempC) / (B + tempC) + Math.log(humidity / 100);
    return (B * alfa) / (A - alfa);
}

function absoluteHumidityFromDewPoint(dewPointC) {
    const SVP = 6.112 * Math.exp((17.67 * dewPointC) / (dewPointC + 243.5)); // Pression de vapeur saturante (hPa)
    return (2167 * SVP) / (KELVIN_OFFSET + tempC); // Humidité absolue (g/m³)
}

function calculateWetBulbTemp(tempC, pressurehPa, humidity) {
    // Approximation Stull (simplifiée)
    const TW = tempC * Math.atan(0.151977 * Math.sqrt(humidity + 8.313659)) + 
               Math.atan(tempC + humidity) - Math.atan(humidity - 1.676331) + 
               0.00391838 * Math.pow(humidity, 1.5) * Math.atan(0.023101 * humidity) - 4.686035;
    return TW;
}


// --- LOGIQUE PHYSIQUE AVANCÉE ---

/** Calcule la dilatation du temps due à la gravité. */
function calculateTimeDilationGravity(altM) {
    if (altM === null) return 0;
    const factor = (G_BASE * altM) / (C_L * C_L);
    return factor * (24 * 3600 * 1e9); // ns/day
}

/** Effectue tous les calculs physiques et relativistes. */
function updatePhysicsCalculations(currentSpeed) {
    const spd_sound = 20.0468 * Math.sqrt(tempC + KELVIN_OFFSET);
    $('vitesse-du-son-locale').textContent = spd_sound.toFixed(2) + ' m/s';
    
    // ... [Calculs de Mach, Pression Dynamique, Force de Traînée, Coriolis] ...

    const air_density = calculateAirDensity(pressurehPa, tempC);
    if (air_density !== null) {
        const dynamic_pressure = 0.5 * air_density * currentSpeed * currentSpeed;
        const force_trainee = dynamic_pressure * 0.5 * 1.0; 
        $('pression-dynamique').textContent = dynamic_pressure.toFixed(2) + ' Pa';
        $('force-de-trainee').textContent = force_trainee.toFixed(2) + ' N';
    }
    
    // Relativité
    const v_c_ratio = currentSpeed / C_L;
    const LORENTZ_FACTOR = 1 / Math.sqrt(1 - v_c_ratio * v_c_ratio);
    
    $('temps-dilation-vitesse').textContent = ((LORENTZ_FACTOR - 1) * (24 * 3600 * 1e9)).toPrecision(2) + ' ns/j';
    $('temps-dilation-gravite').textContent = calculateTimeDilationGravity(altEst).toPrecision(2) + ' ns/j';
    $('rayon-schwarzschild').textContent = (2 * G_CONSTANT * currentMass / (C_L * C_L)).toPrecision(3) + ' m';
    $('lorentz-factor').textContent = LORENTZ_FACTOR.toFixed(4);
    
    // Gravité
    const g_local = altEst !== null ? G_BASE * Math.pow(EARTH_RADIUS / (EARTH_RADIUS + altEst), 2) : null;
    $('gravite-locale').textContent = g_local !== null ? g_local.toFixed(5) + ' m/s²' : 'N/A';
    
    // Distances-Lumière Complètes
    const total_dist_m = distM_3D;
    $('distance-light-s').textContent = (total_dist_m / C_L).toPrecision(2) + ' s';
    $('distance-light-min').textContent = (total_dist_m / (C_L * 60)).toPrecision(2) + ' min';
    $('distance-light-h').textContent = (total_dist_m / (C_L * 3600)).toPrecision(2) + ' h'; // NOUVEAU
    $('distance-light-day').textContent = (total_dist_m / (C_L * 3600 * 24)).toPrecision(2) + ' j';
    $('distance-light-week').textContent = (total_dist_m / (C_L * 3600 * 24 * 7)).toPrecision(2) + ' sem'; // NOUVEAU
    $('distance-light-month').textContent = (total_dist_m / (C_L * 3600 * 24 * 30.4375)).toPrecision(2) + ' mois'; // NOUVEAU
}

/** Met à jour les calculs astronomiques. */
function updateAstroCalculations() {
    const coords = { lat: lat * R2D || 45.0, lon: lon * R2D || 5.0 };
    const date = new Date();
    
    // Affichage des heures locales/UTC (corrigé avec les nouveaux IDs)
    $('local-time-ntp').textContent = date.toLocaleTimeString('fr-FR', { timeZone: 'Europe/Paris' });
    $('date-display-utc').textContent = date.toLocaleDateString('fr-FR', { timeZone: 'UTC' });
    $('date-display-astro').textContent = date.toLocaleDateString('fr-FR');
    
    if (typeof SunCalc !== 'undefined') {
        const sunTimes = SunCalc.getTimes(date, coords.lat, coords.lon);
        const sunPos = SunCalc.getPosition(date, coords.lat, coords.lon);
        
        // ... [Calculs TST, EOT, etc.] ...
        const TST_hours = date.getUTCHours() + date.getUTCMinutes() / 60 + date.getUTCSeconds() / 3600;
        const TST_corrected = (TST_hours + coords.lon / 15); // Simplifié
        
        $('heure-solaire-vraie').textContent = formatHours(TST_corrected % 24);
        $('midi-solaire-local').textContent = sunTimes.solarNoon.toLocaleTimeString('fr-FR', { timeZone: 'UTC' });
        
        // Calcul Heure Minecraft (24000 ticks = 24h)
        const mc_ticks = ((TST_corrected % 24) * 1000) - 6000;
        $('heure-minecraft').textContent = formatMinecraftTime(mc_ticks);
        
        updateMinecraftClockAnimation(sunPos.altitude, sunPos.azimuth);
    }
}

function formatMinecraftTime(ticks) {
    ticks = (ticks % 24000 + 24000) % 24000; // Modulo pour 0-23999
    let mc_hours = Math.floor(ticks / 1000);
    let mc_minutes = Math.floor(((ticks % 1000) / 1000) * 60);
    return `${mc_hours.toString().padStart(2, '0')}:${mc_minutes.toString().padStart(2, '0')}`;
}

function updateMinecraftClockAnimation(sunAltitudeRad, sunAzimuthRad) {
    // ... [Logique d'animation] ...
    const sunAltitudeDeg = sunAltitudeRad * R2D;
    const clockDiv = $('minecraft-clock');

    if (sunAltitudeDeg > 5) {
        clockDiv.style.backgroundColor = 'var(--sky-day)';
    } else if (sunAltitudeDeg < -15) {
        clockDiv.style.backgroundColor = 'var(--sky-night)';
    } else {
        clockDiv.style.backgroundColor = 'var(--sky-sunset)';
    }
    
    // Affichage Nether Mode
    $('nether-mode-display').textContent = isNetherMode ? `ACTIF (1:${NETHER_RATIO})` : 'DÉSACTIVÉ (1:1)';
}
/** Met à jour les compteurs de distance, vitesse max et appels aux calculs physiques. */
function updateCompteurs(currentSpeed, dt) {
    const totalTime = (Date.now() - startTime) / 1000;
    const currentSpeed_kmh = currentSpeed * KMH_MS;
    
    $('elapsed-time').textContent = totalTime.toFixed(2) + ' s';

    let nether_factor = isNetherMode ? NETHER_RATIO : 1.0; 
    
    if (currentSpeed >= MIN_SPD) {
        distM_3D += currentSpeed * dt * nether_factor; // Distance amplifiée en mode Nether
        timeMoving += dt;
        maxSpd = Math.max(maxSpd, currentSpeed_kmh * nether_factor);
    }
    
    // Affichage principal
    $('speed-stable').textContent = (currentSpeed_kmh * nether_factor).toFixed(1) + ' km/h'; // Affichage de la vitesse effective
    
    // Vitesse Moyenne Totale (NOUVEAU)
    const avgSpeedTotal = totalTime > 0 ? (distM_3D / totalTime) * KMH_MS : 0.0;
    $('speed-avg-total').textContent = avgSpeedTotal.toFixed(1) + ' km/h';

    // Mettre à jour les max
    $('magnetic-max-counter').textContent = magFieldMax.toFixed(3) + ' μT';
    $('sound-max-counter').textContent = soundLevelMax.toFixed(1) + ' dB';
    $('light-max-counter').textContent = luminosityMax.toFixed(1) + ' Lux';

    updatePhysicsCalculations(currentSpeed * nether_factor);
}

/** Met à jour la position EKF et la carte. */
function updateGPSDisplay(coords) {
    // ... [Logique d'affichage EKF] ...
    if (lat !== null) {
        $('lat-display').textContent = (lat * R2D).toFixed(6) + ' °';
        $('lon-display').textContent = (lon * R2D).toFixed(6) + ' °';
        $('alt-ekf').textContent = altEst.toFixed(2) + ' m';
    }
    
    const speed_3d_inst = (coords && coords.speed || 0) * KMH_MS;
    $('speed-3d-inst').textContent = speed_3d_inst.toFixed(1) + ' km/h';
    $('gps-precision').textContent = currentAccuracy !== null ? currentAccuracy.toFixed(2) + ' m' : 'N/A';
    
    updateMap(lat * R2D, lon * R2D, currentAccuracy);
}

/** Met à jour les informations de débogage EKF. */
function updateEKFDisplay() {
    // ... [Logique d'affichage EKF P et Biais] ...
    const p_vel = P.get([3, 3]);
    const p_alt = P.get([2, 2]);
    $('incertitude-vitesse-p').textContent = p_vel !== null ? `${p_vel.toPrecision(3)} (m/s)²` : 'N/A';
    $('incertitude-alt-sigma').textContent = p_alt !== null ? `${Math.sqrt(p_alt).toPrecision(3)} m` : 'N/A';
    $('accel-biases').textContent = `${X.get([12]).toPrecision(3)}, ${X.get([13]).toPrecision(3)}, ${X.get([14]).toPrecision(3)}`;
    $('gyro-biases').textContent = `${X.get([9]).toPrecision(3)}, ${X.get([10]).toPrecision(3)}, ${X.get([11]).toPrecision(3)}`;
    
    // Fréquence
    const nyquistFreq = 1 / (DT_MS / 1000) / 2;
    $('nyquist-frequency').textContent = nyquistFreq.toFixed(1) + ' Hz';
    $('high-freq-status').textContent = `${(1000 / DT_MS).toFixed(0)} Hz`;
}

// --- GESTION DES CONTRÔLES ---
function initControls() {
    $('toggle-gps-btn').addEventListener('click', () => isGPSEnabled ? stopGPS() : startGPS());
    $('emergency-stop-btn').addEventListener('click', () => { 
        stopGPS();
        $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: ACTIF 🔴";
    });
    $('reset-dist-btn').addEventListener('click', () => { distM_3D = 0; timeMoving = 0; });
    $('reset-max-btn').addEventListener('click', () => { 
        maxSpd = 0; magFieldMax = 0; soundLevelMax = 0; luminosityMax = 0; 
    });
    $('reset-all-btn').addEventListener('click', () => { window.location.reload(); });
    
    // NOUVEAU: Mode Nuit (Dark Mode)
    $('toggle-dark-mode').addEventListener('click', () => {
        isDarkMode = !isDarkMode;
        document.body.classList.toggle('dark-mode', isDarkMode);
        $('toggle-dark-mode').textContent = isDarkMode ? 'Mode Jour' : 'Mode Nuit';
    });

    $('gps-accuracy-override').addEventListener('input', (e) => {
        gpsAccuracyOverride = parseFloat(e.target.value) || 0;
        $('forcer-precision-gps-display-label').textContent = `${gpsAccuracyOverride.toFixed(6)} m`;
    });
    $('mass-input').addEventListener('input', (e) => {
        currentMass = parseFloat(e.target.value) || 70;
        $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
    });

    // NOUVEAU: Logique du Mode Nether (via l'environnement select)
    $('environment-select').addEventListener('change', (e) => {
        isNetherMode = e.target.value === 'SOUTERRAIN'; // Utilisez Souterrain comme proxy pour Nether Mode
    });
}

// --- BOUCLES ET INITIALISATION ---
function domUpdateLoop() {
    const dt = DT_MS / 1000;
    
    if (!isGPSEnabled && wID === null) {
        gpsError(null); 
    }
    
    if (lastGPSPosition) {
        updateGPSDisplay(lastGPSPosition.coords);
    } else {
        updateGPSDisplay(null); 
    }
    
    updateEKFDisplay();
    updateCompteurs(speedEst, dt);

    // Intégration de la Météo (si weather.js existe, il remplacera cette fonction)
    if (typeof updateWeatherSim === 'function') {
        updateWeatherSim();
    }
}

function init() {
    initControls();
    initializeIMUSensors();
    initEKF(null, null, 0); 
    initMap();
    
    setInterval(domUpdateLoop, DT_MS); 
    setInterval(updateAstroCalculations, 10000);

    updateAstroCalculations();
}

document.addEventListener('DOMContentLoaded', init);
