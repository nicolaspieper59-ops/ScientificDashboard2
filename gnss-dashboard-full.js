// =========================================================================
// _constants.js : Constantes Physiques, Utilitaires et État Global de l'EKF
// =========================================================================

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
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

// Facteurs de correction d'environnement pour l'EKF (simule l'augmentation du bruit R)
const ENV_FACTORS = {
    'NORMAL': 1.0, 
    'FOREST': 2.5,  // Augmentation modérée de l'incertitude
    'CONCRETE': 7.0, // Forte augmentation (tunnels, canyons urbains)
    'METAL': 5.0    // Forte augmentation (grands bâtiments)
};

// --- FONCTION UTILITAIRE DOM ---
const $ = (id) => document.getElementById(id);

// --- VARIABLES GLOBALES (ÉTAT DE L'APPLICATION) ---
let wID = null; // Watcher ID pour Geolocation
let mapInstance = null;
let marker = null;
let isGPSEnabled = false; 
let isXRayMode = false;
let isDarkMode = false;
let currentEnvFactor = ENV_FACTORS.NORMAL;

// --- ÉTAT EKF (21-STATES) ---
// X: [lat, lon, alt, vx, vy, vz, q0, q1, q2, q3, bgx, bgy, bgz, bax, bay, baz, m0, m1, m2, m3, m4]
let X = math.matrix(math.zeros(N_STATES)._data.flat()); // Vecteur d'état
let P = math.diag(math.zeros(N_STATES)._data.flat());   // Matrice de Covariance

// Matrice de Bruit Processus (Q)
let Q_diag = new Array(N_STATES).fill(1e-6); 
Q_diag[0] = Q_diag[1] = Q_diag[2] = 1e-4; // Pos
Q_diag[3] = Q_diag[4] = Q_diag[5] = 1e-3; // Vel
let Q = math.diag(Q_diag);

// --- ÉTAT DES CAPTEURS ET COMPTEURS ---
let lat = null, lon = null, altEst = null, speedEst = 0.0; 
let currentAccuracy = null;
let currentHeading = null;
let lastGPSPosition = null;

let imuAccel = {x: 0, y: 0, z: 0};  // Accélération (m/s²)
let imuGyro = {x: 0, y: 0, z: 0};   // Vitesse angulaire (rad/s)

// Données Météo (initiales/baro)
let tempC = 20.0, pressurehPa = 1013.25, humidityPerc = 50.0; 
let currentMass = 70.0;
let gpsAccuracyOverride = 0.0;
let startTime = Date.now();

let distM_3D = 0.0;
let timeMoving = 0.0;
let maxSpd = 0.0;

// Variables pour l'affichage (simulées si pas de capteurs réels)
let magFieldMax = 0.0; 
let angularSpeed = 0.0;
// =========================================================================
// CORRECTION CRITIQUE DANS _ekf_core.js
// =========================================================================

/**
 * Étape de Prédiction de l'EKF (Propagation INS) via IMU corrigée.
 */
function EKF_predict(dt) {
    const ba = [X.get([12]), X.get([13]), X.get([14])]; 
    const accel_corrected = math.subtract([imuAccel.x, imuAccel.y, imuAccel.z], ba);

    // 1. Vitesse (Propagation)
    const V_xyz = [X.get([3]), X.get([4]), X.get([5])];
    const NED_Accel = [accel_corrected[0], accel_corrected[1], accel_corrected[2]]; 
    const gravity = [0, 0, G_BASE]; 

    const dV = math.multiply(math.subtract(NED_Accel, gravity), dt);
    const new_V_xyz = math.add(V_xyz, dV);
    X.set([3], new_V_xyz[0]); X.set([4], new_V_xyz[1]); X.set([5], new_V_xyz[2]);
    
    // --- FIX CRITIQUE: Propagation de la position en coordonnées géodétiques ---
    const latRad = X.get([0]);
    const lonRad = X.get([1]);
    const altM = X.get([2]);

    const Vn = X.get([3]); // V North
    const Ve = X.get([4]); // V East
    const Vd = X.get([5]); // V Down
    
    // Rayons de courbure (simplifié: rayon équatorial)
    const R_M = EARTH_RADIUS + altM; 
    // R_N_prime est le rayon du parallèle (dépend de la latitude)
    const R_N_prime = (EARTH_RADIUS + altM) * Math.cos(latRad); 

    // Changement de position (radians/mètres)
    const dLat = (Vn * dt) / R_M;
    const dLon = (Ve * dt) / R_N_prime;
    const dAlt = -Vd * dt; // dAlt est l'opposé de Vd (Down)

    // Propagation (Lat/Lon/Alt) - Applique le facteur d'environnement
    X.set([0], latRad + dLat * currentEnvFactor); 
    X.set([1], lonRad + dLon * currentEnvFactor); 
    X.set([2], altM + dAlt * currentEnvFactor); 

    // 3. Mise à jour de la Covariance (Simplifié: P = P + Q)
    P = math.add(P, Q); 
    
    // Mise à jour des variables d'affichage
    lat = X.get([0]); 
    lon = X.get([1]);
    altEst = X.get([2]);
    speedEst = math.norm([X.get([3]), X.get([4]), X.get([5])]);
}

// Le reste du fichier _ekf_core.js reste inchangé.


/**
 * Étape de Correction (Mise à jour) de l'EKF.
 */
function EKF_update(z_meas, z_h, H, R) {
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

/**
 * Gestion du signal GPS réussi (Correction GNSS de l'EKF).
 */
function gpsSuccess(position) {
    const dt = DT_MS / 1000;
    
    if (lat === null) {
        initEKF(position.coords.latitude, position.coords.longitude, position.coords.altitude || 0);
    }
    
    // Prédiction avant la mesure
    EKF_predict(dt); 

    // Ajustement de la covariance de mesure R avec le facteur d'environnement
    let R_val_base = (gpsAccuracyOverride > 0) ? gpsAccuracyOverride : (position.coords.accuracy || 10);
    let R_val = R_val_base * currentEnvFactor; // Pénaliser la confiance dans la mesure GPS
    
    currentAccuracy = R_val;
    currentHeading = position.coords.heading;
    
    // Mesure (z_meas): [lat, lon, alt, vx, vy, vz]
    const speedRaw = position.coords.speed || 0;
    const headingRad = (currentHeading !== null) ? currentHeading * D2R : 0;
    
    const z_gnss = math.matrix([
        position.coords.latitude * D2R, 
        position.coords.longitude * D2R,
        position.coords.altitude || altEst,
        speedRaw * Math.sin(headingRad), 
        speedRaw * Math.cos(headingRad), 
        0 
    ]);
    
    // Correction GNSS
    const z_h_gnss = math.matrix([X.get([0]), X.get([1]), X.get([2]), X.get([3]), X.get([4]), X.get([5])]);
    let H_gnss = math.zeros(6, N_STATES);
    for(let i=0; i<6; i++) H_gnss.set([i, i], 1); 
    
    const R_gps_val = R_val * R_val;
    const R_gps = math.diag([R_gps_val, R_gps_val, R_gps_val, R_gps_val, R_gps_val, R_gps_val]);

    EKF_update(z_gnss, z_h_gnss, H_gnss, R_gps); 

    lastGPSPosition = position;
    $('gps-status-dr').textContent = 'ACTIF (FUSION IMU/GPS)';
}

/**
 * Gestion de l'erreur GPS (Dead Reckoning et ZUPT).
 */
function gpsError(error) {
    const dt = DT_MS / 1000;
    
    if (lat === null) initEKF(null, null, 0); 
    
    // Prédiction (Dead Reckoning)
    EKF_predict(dt); 

    // ZUPT (Correction Vitesse Zéro)
    const accel_vertical = imuAccel.z - G_BASE; // Accélération verticale corrigée par la gravité
    const accel_horizontal = Math.sqrt(imuAccel.x*imuAccel.x + imuAccel.y*imuAccel.y);
    const accel_mag = Math.sqrt(accel_horizontal*accel_horizontal + accel_vertical*accel_vertical);
    
    if (accel_mag < ZUPT_ACCEL_TOLERANCE && speedEst < MIN_SPD) {
        const z_zupt = math.matrix([0, 0, 0]); 
        const z_h_zupt = math.matrix([X.get([3]), X.get([4]), X.get([5])]); 
        
        let H_zupt = math.zeros(3, N_STATES);
        for(let i=0; i<3; i++) H_zupt.set([i, i+3], 1); 
        
        const R_zupt = math.diag([0.01, 0.01, 0.01]); // Haute confiance dans l'arrêt
        EKF_update(z_zupt, z_h_zupt, H_zupt, R_zupt); 
        
        $('speed-status-text').textContent = 'Mode Dead Reckoning (ZUPT)';
        currentAccuracy = Math.sqrt(P.get([0, 0])); // Précision estimée par la covariance
    } else {
        $('speed-status-text').textContent = 'Mode Dead Reckoning (IMU Seul)';
        currentAccuracy = null;
    }

    $('gps-status-dr').textContent = 'ERREUR GPS - DR: IMU Seul';
}

/**
 * Initialise et écoute les capteurs IMU (Accéléromètre et Gyroscope) via DeviceMotion.
 */
function initializeIMUSensors() {
    if ('ondevicemotion' in window) {
        window.addEventListener('devicemotion', (event) => {
            if (event.accelerationIncludingGravity) {
                // Stockage des accélérations brutes (avec gravité)
                imuAccel.x = event.accelerationIncludingGravity.x || 0;
                imuAccel.y = event.accelerationIncludingGravity.y || 0;
                imuAccel.z = event.accelerationIncludingGravity.z || 0;
                
                // Mettre à jour les données brutes IMU
                $('accel-x').textContent = imuAccel.x.toFixed(2) + ' m/s²';
                $('accel-y').textContent = imuAccel.y.toFixed(2) + ' m/s²';
                $('accel-z').textContent = imuAccel.z.toFixed(2) + ' m/s²';

                if (event.rotationRate) {
                    imuGyro.x = event.rotationRate.alpha * D2R; 
                    imuGyro.y = event.rotationRate.beta * D2R; 
                    imuGyro.z = event.rotationRate.gamma * D2R; 
                    angularSpeed = Math.sqrt(imuGyro.x**2 + imuGyro.y**2 + imuGyro.z**2) * R2D;
                    
                    // Calcul simplifié du champ magnétique (simulé ici par une fonction de rotation)
                    magFieldMax = Math.max(magFieldMax, Math.abs(imuGyro.x) * 100); 
                }
            }
            $('imu-status').textContent = 'ACTIF / Motion API';
        }, true);
    } else {
        $('imu-status').textContent = 'INACTIF (Simul.)';
    }
    }
// =========================================================================
// _physics_astro_weather.js : Physique Avancée, Astronomie et Météo
// =========================================================================

// --- FONCTIONS DE BASE ---

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

function calculateTimeDilationGravity(altM) {
    if (altM === null || altM < 0) return 0;
    // Approche simplifiée pour l'affichage
    const g_local = G_BASE * Math.pow(EARTH_RADIUS / (EARTH_RADIUS + altM), 2);
    const factor = (G_BASE - g_local) * altM / (C_L * C_L);
    return factor * (24 * 3600 * 1e9); 
}

// --- LOGIQUE MÉTÉO/BIOPHYSIQUE ---

function updateWeatherAndBiophysics() {
    // Simulation des variations si aucune API externe n'est disponible
    tempC = 20.0 + Math.sin(Date.now() / 100000) * 5; 
    pressurehPa = 1013.25 + Math.cos(Date.now() / 50000) * 2;
    humidityPerc = 50.0 + Math.sin(Date.now() / 80000) * 10;

    const air_density = calculateAirDensity(pressurehPa, tempC);
    const dew_point = calculateDewPoint(tempC, humidityPerc);
    
    // Rendu DOM (Utilisation des IDs du HTML)
    $('temp-air-2').textContent = tempC.toFixed(1) + ' °C';
    $('pressure-2').textContent = pressurehPa.toFixed(2) + ' hPa';
    $('humidity-2').textContent = humidityPerc.toFixed(1) + ' %';
    $('dew-point').textContent = dew_point !== null ? dew_point.toFixed(1) + ' °C' : 'N/A';
    $('air-density').textContent = air_density !== null ? air_density.toFixed(3) + ' kg/m³' : 'N/A';
    $('alt-baro').textContent = altEst !== null ? (altEst + (1013.25 - pressurehPa) * 8.5).toFixed(2) + ' m' : 'N/A';
    $('weather-status').textContent = 'SIMULÉ';
}

// --- LOGIQUE PHYSIQUE AVANCÉE ---

function updatePhysicsCalculations(currentSpeed) {
    const spd_sound = calculateSpeedOfSound(tempC);
    const air_density = calculateAirDensity(pressurehPa, tempC);
    
    // Gravité Locale
    const g_local = altEst !== null ? G_BASE * Math.pow(EARTH_RADIUS / (EARTH_RADIUS + altEst), 2) : null;
    $('gravity-local').textContent = g_local !== null ? g_local.toFixed(5) + ' m/s²' : 'N/A';

    // Mécanique des Fluides
    if (spd_sound) {
        const mach = currentSpeed / spd_sound;
        $('speed-of-sound-calc').textContent = spd_sound.toFixed(2) + ' m/s';
        $('perc-speed-sound').textContent = (currentSpeed / spd_sound * 100).toFixed(2) + ' %';
        $('mach-number').textContent = mach.toFixed(4);
    } 
    
    if (air_density !== null) {
        const dynamic_pressure = 0.5 * air_density * currentSpeed * currentSpeed;
        const force_trainee = dynamic_pressure * 0.5 * 1.0; 
        $('dynamic-pressure').textContent = dynamic_pressure.toFixed(2) + ' Pa';
        $('drag-force').textContent = force_trainee.toFixed(2) + ' N';
        $('radiation-pressure').textContent = (dynamic_pressure * 1e-6).toPrecision(2) + ' Pa'; // Placeholder
    }

    // Force de Coriolis
    const coriol_lat = lat !== null ? lat * R2D : 45.0; 
    const coriolis_force = currentMass * 2 * OMEGA_EARTH * Math.sin(coriol_lat * D2R) * currentSpeed;
    $('coriolis-force').textContent = coriolis_force.toPrecision(3) + ' N';
    
    // Vitesse angulaire et Accélération longitudinale
    $('angular-speed').textContent = angularSpeed.toFixed(2) + ' °/s';
    $('accel-long').textContent = imuAccel.x.toFixed(2) + ' m/s²'; 

    // Relativité
    const v_c_ratio = currentSpeed / C_L;
    const LORENTZ_FACTOR = 1 / Math.sqrt(1 - v_c_ratio * v_c_ratio);
    
    $('lorentz-factor').textContent = LORENTZ_FACTOR.toFixed(4);
    // Affichage des % de vitesse de la lumière (utilise le même ID que l'HTML)
    $('perc-speed-c').textContent = (v_c_ratio * 100).toPrecision(3) + ' %'; 
}

// --- LOGIQUE ASTRO ---

function formatHours(decimalHours) {
    const hours = Math.floor(decimalHours);
    const minutes = Math.floor((decimalHours % 1) * 60);
    return `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}`;
}

function formatMinecraftTime(ticks) {
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
    const coords = { lat: lat * R2D || 45.0, lon: lon * R2D || 5.0 };
    const date = new Date();
    
    // Heures locales/UTC
    $('local-time').textContent = date.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit', second: '2-digit' });
    $('date-display').textContent = date.toLocaleDateString('fr-FR', { timeZone: 'UTC' });
    $('date-display-astro').textContent = date.toLocaleDateString('fr-FR');
    
    if (typeof SunCalc !== 'undefined') {
        const sunTimes = SunCalc.getTimes(date, coords.lat, coords.lon);
        const sunPos = SunCalc.getPosition(date, coords.lat, coords.lon);
        const moonPos = SunCalc.getMoonPosition(date, coords.lat, coords.lon);
        const moonIllumination = SunCalc.getMoonIllumination(date);
        
        // Calcul TST et Minecraft
        const TST_hours = date.getUTCHours() + date.getUTCMinutes() / 60 + date.getUTCSeconds() / 3600;
        const TST_corrected = (TST_hours + coords.lon / 15); 
        
        $('tst').textContent = formatHours(TST_corrected % 24);
        $('noon-solar').textContent = sunTimes.solarNoon.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit', timeZone: 'UTC' });
        
        const mc_ticks = ((TST_corrected % 24) * 1000) - 6000;
        $('time-minecraft').textContent = formatMinecraftTime(mc_ticks);

        // Affichage Astro
        $('sun-alt').textContent = (sunPos.altitude * R2D).toFixed(2) + ' °';
        $('sun-azimuth').textContent = (sunPos.azimuth * R2D).toFixed(2) + ' °';
        $('moon-alt').textContent = (moonPos.altitude * R2D).toFixed(2) + ' °';
        $('moon-azimuth').textContent = (moonPos.azimuth * R2D).toFixed(2) + ' °';
        $('moon-phase-name').textContent = getMoonPhaseName(moonIllumination.phase);
        $('moon-illuminated').textContent = (moonIllumination.fraction * 100).toFixed(1) + ' %';
        
        // Placeholders pour les autres IDs astro
        $('date-solar-mean').textContent = date.toLocaleDateString('fr-FR');
        $('date-solar-true').textContent = date.toLocaleDateString('fr-FR');
        $('mst').textContent = formatHours(TST_hours % 24);
        $('eot').textContent = (sunPos.equationOfTime / 60).toFixed(2) + ' min'; // EOT en minutes
        $('ecl-long').textContent = (sunPos.eclipticLongitude * R2D).toFixed(2) + ' °';
        
        updateMinecraftClockAnimation(sunPos.altitude, sunPos.azimuth);
    }
}

function updateMinecraftClockAnimation(sunAltitudeRad, sunAzimuthRad) {
    const totalAngle = (-sunAzimuthRad * R2D) + 90; 
    const sunElement = $('sun-element');
    const moonElement = $('moon-element');
    const clockDiv = $('minecraft-clock');
    const biomeHalf = $('biome-half');
    const sunAltitudeDeg = sunAltitudeRad * R2D;

    if (sunElement && moonElement) {
        sunElement.style.transform = `rotate(${totalAngle}deg) translateX(0) translateY(-50%)`;
        moonElement.style.transform = `rotate(${totalAngle - 180}deg) translateX(0) translateY(-50%)`; 
        
        // Ajustement de la couleur du ciel
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
        
        // Mode X-Ray pour le biome
        if (isXRayMode) {
            clockDiv.classList.add('x-ray');
        } else {
            clockDiv.classList.remove('x-ray');
        }
    }
    // Affichage du Facteur d'environnement
    $('nether-mode-status').textContent = currentEnvFactor.toFixed(3);
}
// =========================================================================
// _main_dom.js : Contrôles, Boucle Principale et Rendu DOM/Carte
// =========================================================================

// --- CARTE LEAFLET ---

/** Initialise la carte Leaflet. */
function initMap() {
    if (typeof L !== 'undefined') {
        mapInstance = L.map('map').setView([45.0, 5.0], 10);
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
        mapInstance.setView(newLatLng, 15, { animate: true });
        
        if (marker) {
            mapInstance.removeLayer(marker);
        }
        
        // Marqueur de position (cercle de précision)
        marker = L.circle(newLatLng, {
            color: 'red',
            fillColor: '#f03',
            fillOpacity: 0.5,
            radius: accuracy || 10 
        }).addTo(mapInstance);
    }
}

// --- MISE À JOUR DU DOM ---

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
    const horizon_dist = altEst !== null ? Math.sqrt(2 * EARTH_RADIUS * altEst) : null;
    $('distance-horizon').textContent = (horizon_dist !== null && horizon_dist > 0) ? (horizon_dist / 1000).toFixed(1) + ' km' : 'N/A';
    
    // Rapport de Distance (MRF)
    const altRatio = (EARTH_RADIUS + (altEst || 0)) / EARTH_RADIUS;
    $('distance-ratio').textContent = altRatio.toFixed(3);

    // Relativité Cosmique (utilise les mêmes IDs que le HTML)
    const total_dist_m = distM_3D;
    $('distance-light-s').textContent = (total_dist_m / C_L).toPrecision(2) + ' s';
    $('distance-light-min').textContent = (total_dist_m / (C_L * 60)).toPrecision(2) + ' min';

    // Appel au BLOC 3 pour la physique
    updatePhysicsCalculations(speedEst * currentEnvFactor);
}

/** Met à jour les données de position EKF et la vitesse brute. */
function updateGPSDisplay(coords) {
    if (lat !== null) {
        $('lat-display').textContent = (lat * R2D).toFixed(6) + ' °';
        $('lon-display').textContent = (lon * R2D).toFixed(6) + ' °';
        $('alt-display').textContent = altEst.toFixed(2) + ' m';
        $('geopotential-alt').textContent = altEst.toFixed(2) + ' m'; // Simplification
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
    $('speed-error-perc').textContent = R_val !== null ? R_val.toFixed(1) : 'N/A'; // Utilise l'ID pour Bruit Mesure
    
    // Fréquence
    const nyquistFreq = 1 / (DT_MS / 1000) / 2;
    $('nyquist-frequency').textContent = nyquistFreq.toFixed(1) + ' Hz';
    
    // Affichage de la précision forcée
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
    $('gps-status-dr').textContent = 'Arrêté';
    $('speed-status-text').textContent = 'GPS en Pause / Dead Reckoning Arrêté.';
}

function initControls() {
    // Boutons de contrôle
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
    
    // Mode Nuit
    $('toggle-mode-btn').addEventListener('click', () => {
        isDarkMode = !isDarkMode;
        document.body.classList.toggle('dark-mode', isDarkMode);
        $('toggle-mode-btn').innerHTML = isDarkMode ? '<i class="fas fa-sun"></i> Mode Jour' : '<i class="fas fa-moon"></i> Mode Nuit';
    });
    
    // Mode X-Ray pour la carte
    $('xray-button').addEventListener('click', () => {
        isXRayMode = !isXRayMode;
        $('minecraft-clock').classList.toggle('x-ray', isXRayMode);
        $('xray-button').textContent = isXRayMode ? 'ON' : 'X';
    });

    // Inputs et Sélections
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
        $('env-factor').textContent = e.target.options[e.target.selectedIndex].text;
    });
}

// --- BOUCLES ET INITIALISATION ---

/** Boucle principale de l'application (20 Hz) */
function domUpdateLoop() {
    const dt = DT_MS / 1000;
    
    // Si GPS est en pause, forcer l'EKF à passer en mode DR/ZUPT
    if (!isGPSEnabled && wID === null) {
        gpsError(null); 
    }
    
    // Mise à jour de l'affichage
    if (lastGPSPosition) {
        updateGPSDisplay(lastGPSPosition.coords);
    } else {
        updateGPSDisplay(null); 
    }
    
    updateEKFDisplay();
    updateCompteurs(speedEst, dt);
    
    // Mise à jour de la météo (rapide)
    if (typeof updateWeatherAndBiophysics === 'function') {
        updateWeatherAndBiophysics();
    }
}

/** Point d'entrée de l'application */
function init() {
    initControls();
    initializeIMUSensors();
    initEKF(null, null, 0); 
    initMap();
    
    // Boucle rapide (EKF Predict + DOM)
    setInterval(domUpdateLoop, DT_MS); 
    
    // Boucle lente (Astro, Météo (si non temps réel))
    setInterval(updateAstroCalculations, 10000);

    updateAstroCalculations();
}

document.addEventListener('DOMContentLoaded', init);
