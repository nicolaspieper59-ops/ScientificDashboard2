// =========================================================================
// _constants.js : Constantes Physiques, Utilitaires et État Global de l'EKF
// =========================================================================

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;          // Vitesse de la lumière (m/s)
const G_CONSTANT = 6.67430e-11; // Constante gravitationnelle (N·(m/kg)²)
const EARTH_MASS = 5.972e24;    // Masse de la Terre (kg)
const EARTH_RADIUS = 6378137;   // Rayon de la Terre à l'équateur (m)
const G_BASE = 9.80665;         // Gravité standard (m/s²)
const OMEGA_EARTH = 7.2921e-5;  // Vitesse angulaire de la Terre (rad/s)
const KMH_MS = 3.6;             
const KELVIN_OFFSET = 273.15;
const DT_MS = 50;               // Période d'échantillonnage (20 Hz)
const ZUPT_ACCEL_TOLERANCE = 0.5; // Tolérance pour Zero Velocity Update (m/s²)
const MIN_SPD = 0.05;           // Vitesse minimale pour être considéré en mouvement (m/s)
const NETHER_RATIO = 8.0;       // Rapport distance Nether:Overworld (8:1)
const N_STATES = 21;            // Nombre d'états du vecteur EKF

// Constantes Lumineuses/Cosmiques
const LIGHT_YEAR_M = 9.461e15; 
const AU_M = 1.496e11;          

// --- FONCTION UTILITAIRE DOM ---
const $ = (id) => document.getElementById(id);

// --- VARIABLES GLOBALES (ÉTAT DE L'APPLICATION) ---
let wID = null; // Watcher ID pour Geolocation
let mapInstance = null;
let marker = null;
let isGPSEnabled = false; 
let isXRayMode = false;
let isDarkMode = false;
let isNetherMode = false;

// --- ÉTAT EKF (21-STATES) ---
// X: [lat, lon, alt, vx, vy, vz, q0, q1, q2, q3, bgx, bgy, bgz, bax, bay, baz, m0, m1, m2, m3, m4]
let X = math.matrix(math.zeros(N_STATES)._data.flat()); // Vecteur d'état
let P = math.diag(math.zeros(N_STATES)._data.flat());   // Matrice de Covariance

// Matrice de Bruit Processus (Q) - Initialisation des incertitudes
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

// Données Météo (initiales/baro) - Mises à jour par _physics_astro_weather.js
let tempC = 20.0, pressurehPa = 1013.25, humidityPerc = 50.0; 
let magFieldMax = 0.0;
let soundLevelMax = 0.0;
let luminosityMax = 0.0;

let distM_3D = 0.0;
let timeMoving = 0.0;
let maxSpd = 0.0;
let currentMass = 70.0;
let gpsAccuracyOverride = 0.0;
let startTime = Date.now();
// =========================================================================
// _ekf_core.js : Moteur EKF (21-States), Géolocalisation et IMU
// =========================================================================

/**
 * Initialise l'état EKF et les variables globales avec une position GPS initiale.
 */
function initEKF(lat_init, lon_init, alt_init) {
    lat = lat_init !== null ? lat_init * D2R : 45.0 * D2R; 
    lon = lon_init !== null ? lon_init * D2R : 5.0 * D2R; 
    altEst = alt_init !== null ? alt_init : 0.0;
    
    // Initialisation du vecteur d'état X (seulement Pos/Alt)
    X.set([0], lat); X.set([1], lon); X.set([2], altEst);
    
    // Initialisation de la matrice de Covariance P
    P = math.diag(Q_diag);
}

/**
 * Étape de Prédiction de l'EKF (Propagation INS) via IMU corrigée.
 */
function EKF_predict(dt) {
    const bg = [X.get([9]), X.get([10]), X.get([11])]; 
    const ba = [X.get([12]), X.get([13]), X.get([14])]; 
    const accel_corrected = math.subtract([imuAccel.x, imuAccel.y, imuAccel.z], ba);

    // 1. Vitesse (V_k+1 = V_k + (R * a_corr - g) * dt)
    const V_xyz = [X.get([3]), X.get([4]), X.get([5])];
    const NED_Accel = [accel_corrected[0], accel_corrected[1], accel_corrected[2]]; 
    const gravity = [0, 0, G_BASE]; 

    const dV = math.multiply(math.subtract(NED_Accel, gravity), dt);
    const new_V_xyz = math.add(V_xyz, dV);
    X.set([3], new_V_xyz[0]); X.set([4], new_V_xyz[1]); X.set([5], new_V_xyz[2]);

    // 2. Position (P_k+1 = P_k + V_k * dt) - Applique le facteur Nether à la distance
    let nether_factor = isNetherMode ? NETHER_RATIO : 1.0; 
    const P_xyz = [X.get([0]), X.get([1]), X.get([2])];
    const dP = math.multiply(V_xyz, dt * nether_factor); 
    const new_P_xyz = math.add(P_xyz, dP);
    X.set([0], new_P_xyz[0]); X.set([1], new_P_xyz[1]); X.set([2], new_P_xyz[2]);

    // 3. Mise à jour de la Covariance (P = F * P * F' + Q) - Simplifié: P = P + Q
    P = math.add(P, Q); 
    
    // Mise à jour des variables d'affichage
    lat = X.get([0]); 
    lon = X.get([1]);
    altEst = X.get([2]);
    speedEst = math.norm([X.get([3]), X.get([4]), X.get([5])]);
}

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
    
    EKF_predict(dt); 

    let R_val = (gpsAccuracyOverride > 0) ? gpsAccuracyOverride : (position.coords.accuracy || 10);
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
    
    // Matrice H et R
    const z_h_gnss = math.matrix([X.get([0]), X.get([1]), X.get([2]), X.get([3]), X.get([4]), X.get([5])]);
    let H_gnss = math.zeros(6, N_STATES);
    for(let i=0; i<6; i++) H_gnss.set([i, i], 1); 
    
    const R_gps_val = R_val * R_val;
    const R_gps = math.diag([R_gps_val, R_gps_val, R_gps_val, R_gps_val, R_gps_val, R_gps_val]);

    EKF_update(z_gnss, z_h_gnss, H_gnss, R_gps); 

    lastGPSPosition = position;
    $('statut-gps-ekf').textContent = 'ACTIF (FUSION IMU/GPS)';
}

/**
 * Gestion de l'erreur GPS (Dead Reckoning et ZUPT).
 */
function gpsError(error) {
    const dt = DT_MS / 1000;
    
    if (lat === null) initEKF(null, null, 0); 
    EKF_predict(dt); 

    // ZUPT (Correction Vitesse Zéro)
    const accel_mag = Math.sqrt(imuAccel.x*imuAccel.x + imuAccel.y*imuAccel.y + (imuAccel.z - G_BASE)*(imuAccel.z - G_BASE));
    
    if (accel_mag < ZUPT_ACCEL_TOLERANCE && speedEst < MIN_SPD) {
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

/**
 * Initialise et écoute les capteurs IMU (Accéléromètre et Gyroscope) via DeviceMotion.
 */
function initializeIMUSensors() {
    if ('ondevicemotion' in window) {
        window.addEventListener('devicemotion', (event) => {
            if (event.accelerationIncludingGravity) {
                imuAccel.x = event.accelerationIncludingGravity.x || 0;
                imuAccel.y = event.accelerationIncludingGravity.y || 0;
                imuAccel.z = event.accelerationIncludingGravity.z || 0;
                
                // Estimation de la magnitude du Champ Magnétique
                if (event.rotationRate) {
                    magFieldMax = Math.max(magFieldMax, Math.abs(event.rotationRate.alpha) * 100); 
                    imuGyro.x = event.rotationRate.alpha * D2R; 
                    imuGyro.y = event.rotationRate.beta * D2R; 
                    imuGyro.z = event.rotationRate.gamma * D2R; 
                }
            }
            $('sensor-status').textContent = 'Actif / Motion API';
        }, true);
    } else {
        $('sensor-status').textContent = 'Inactif (Simul.)';
    }
    // Simulation simple des capteurs Max si non disponible
    soundLevelMax = 85.0; 
    luminosityMax = 12000.0;
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

function absoluteHumidityFromDewPoint(dewPointC) {
    const SVP = 6.112 * Math.exp((17.67 * dewPointC) / (dewPointC + 243.5)); 
    return (2167 * SVP) / (KELVIN_OFFSET + tempC); 
}

function calculateWetBulbTemp(tempC, pressurehPa, humidity) {
    // Approximation Stull simplifiée
    const TW = tempC * Math.atan(0.151977 * Math.sqrt(humidity + 8.313659)) + 
               Math.atan(tempC + humidity) - Math.atan(humidity - 1.676331) + 
               0.00391838 * Math.pow(humidity, 1.5) * Math.atan(0.023101 * humidity) - 4.686035;
    return TW;
}

function calculateTimeDilationGravity(altM) {
    if (altM === null || altM < 0) return 0;
    const factor = (G_BASE * altM) / (C_L * C_L);
    return factor * (24 * 3600 * 1e9); 
}

// --- LOGIQUE MÉTÉO/BIOPHYSIQUE ---

/**
 * Mise à jour de la Météo (Simulée si pas de fichier weather.js)
 * Si une fonction 'fetchWeather' est définie dans weather.js, elle la remplacera.
 */
function updateWeatherAndBiophysics() {
    // Simulation des variations si aucune API externe n'est disponible
    if (typeof fetchWeather !== 'function') { 
        tempC = 20.0 + Math.sin(Date.now() / 100000) * 5; 
        pressurehPa = 1013.25 + Math.cos(Date.now() / 50000) * 2;
        humidityPerc = 50.0 + Math.sin(Date.now() / 80000) * 10;
    }

    const air_density = calculateAirDensity(pressurehPa, tempC);
    const dew_point = calculateDewPoint(tempC, humidityPerc);
    const wet_bulb_temp = calculateWetBulbTemp(tempC, pressurehPa, humidityPerc);
    
    // Rendu DOM
    $('temp-air').textContent = tempC.toFixed(1) + ' °C';
    $('pressure-hpa').textContent = pressurehPa.toFixed(2) + ' hPa';
    $('humidity-perc').textContent = humidityPerc.toFixed(1) + ' %';
    $('air-density').textContent = air_density !== null ? air_density.toFixed(3) + ' kg/m³' : 'N/A';
    $('dew-point').textContent = dew_point !== null ? dew_point.toFixed(1) + ' °C' : 'N/A';
    $('absolute-humidity').textContent = dew_point !== null ? (absoluteHumidityFromDewPoint(dew_point)).toFixed(3) + ' g/m³' : 'N/A';
    $('temp-thermometre-mouille').textContent = wet_bulb_temp !== null ? wet_bulb_temp.toFixed(1) + ' °C' : 'N/A';
    
    // Simplifications Biophysiques
    $('o2-saturation-theo').textContent = (100 - (humidityPerc / 2)).toFixed(1) + ' %'; 
    $('photosynthesis-sim').textContent = (luminosityMax > 10000 ? 'Élevée' : 'Normale');
    $('indice-de-stabilité').textContent = '0 J/kg'; 
    $('taux-d-oxygene-o2').textContent = '20.9 % vol';
}

// --- LOGIQUE PHYSIQUE AVANCÉE ---

/** Effectue tous les calculs physiques et relativistes. */
function updatePhysicsCalculations(currentSpeed) {
    const spd_sound = calculateSpeedOfSound(tempC);
    const air_density = calculateAirDensity(pressurehPa, tempC);
    
    // Gravité Locale
    const g_local = altEst !== null ? G_BASE * Math.pow(EARTH_RADIUS / (EARTH_RADIUS + altEst), 2) : null;
    $('gravite-locale').textContent = g_local !== null ? g_local.toFixed(5) + ' m/s²' : 'N/A';

    // Mécanique des Fluides
    if (spd_sound) {
        const mach = currentSpeed / spd_sound;
        $('vitesse-du-son-locale').textContent = spd_sound.toFixed(2) + ' m/s';
        $('perc-speed-sound').textContent = (currentSpeed / spd_sound * 100).toFixed(2) + ' %';
        $('nombre-de-mach').textContent = mach.toFixed(4);
    } 
    
    if (air_density !== null) {
        const dynamic_pressure = 0.5 * air_density * currentSpeed * currentSpeed;
        const force_trainee = dynamic_pressure * 0.5 * 1.0; 
        $('pression-dynamique').textContent = dynamic_pressure.toFixed(2) + ' Pa';
        $('force-de-trainee').textContent = force_trainee.toFixed(2) + ' N';
    }

    // Force de Coriolis
    const coriol_lat = lat !== null ? lat * R2D : 45.0; 
    const coriolis_force = currentMass * 2 * OMEGA_EARTH * Math.sin(coriol_lat * D2R) * currentSpeed;
    $('force-de-coriolis').textContent = coriolis_force.toPrecision(3) + ' N';

    // Relativité
    const v_c_ratio = currentSpeed / C_L;
    const LORENTZ_FACTOR = 1 / Math.sqrt(1 - v_c_ratio * v_c_ratio);
    
    $('lorentz-factor').textContent = LORENTZ_FACTOR.toFixed(4);
    $('temps-dilation-vitesse').textContent = ((LORENTZ_FACTOR - 1) * (24 * 3600 * 1e9)).toPrecision(2) + ' ns/j';
    $('temps-dilation-gravite').textContent = calculateTimeDilationGravity(altEst).toPrecision(2) + ' ns/j';
    $('energie-masse-au-repos').textContent = (currentMass * C_L * C_L).toPrecision(3) + ' J';
    $('energie-relativiste').textContent = (currentMass * C_L * C_L * LORENTZ_FACTOR).toPrecision(3) + ' J';
    $('rayon-schwarzschild').textContent = (2 * G_CONSTANT * currentMass / (C_L * C_L)).toPrecision(3) + ' m';
    $('perc-speed-c').textContent = (v_c_ratio * 100).toPrecision(3) + ' %';
    $('momentum').textContent = (currentMass * currentSpeed * LORENTZ_FACTOR).toPrecision(3) + ' kg·m/s';
}

// --- LOGIQUE ASTRO ---

function formatHours(decimalHours) {
    const hours = Math.floor(decimalHours);
    const minutes = Math.floor((decimalHours % 1) * 60);
    return `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}`;
}

function formatMinecraftTime(ticks) {
    ticks = (ticks % 24000 + 24000) % 24000; 
    let mc_hours = Math.floor(ticks / 1000);
    let mc_minutes = Math.floor(((ticks % 1000) / 1000) * 60);
    return `${mc_hours.toString().padStart(2, '0')}:${mc_minutes.toString().padStart(2, '0')}`;
}

function updateAstroCalculations() {
    const coords = { lat: lat * R2D || 45.0, lon: lon * R2D || 5.0 };
    const date = new Date();
    
    // Heures locales/UTC (NTP non simulé, utilise l'heure système)
    $('local-time-ntp').textContent = date.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit', second: '2-digit' });
    $('date-display-utc').textContent = date.toLocaleDateString('fr-FR', { timeZone: 'UTC' });
    $('date-display-astro').textContent = date.toLocaleDateString('fr-FR');
    
    if (typeof SunCalc !== 'undefined') {
        const sunTimes = SunCalc.getTimes(date, coords.lat, coords.lon);
        const sunPos = SunCalc.getPosition(date, coords.lat, coords.lon);
        const moonIllumination = SunCalc.getMoonIllumination(date);
        
        // TST (Heure Solaire Vraie)
        const TST_hours = date.getUTCHours() + date.getUTCMinutes() / 60 + date.getUTCSeconds() / 3600;
        const TST_corrected = (TST_hours + coords.lon / 15); 
        
        $('heure-solaire-vraie').textContent = formatHours(TST_corrected % 24);
        $('midi-solaire-local').textContent = sunTimes.solarNoon.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit', timeZone: 'UTC' });
        
        // Heure Minecraft
        const mc_ticks = ((TST_corrected % 24) * 1000) - 6000;
        $('heure-minecraft').textContent = formatMinecraftTime(mc_ticks);

        // Lune et Soleil
        $('moon-phase-name').textContent = getMoonPhaseName(moonIllumination.phase);
        $('moon-phase-perc').textContent = (moonIllumination.fraction * 100).toFixed(1) + ' %';
        $('lever-coucher-soleil-locale').textContent = `${sunTimes.sunrise.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit' })} / ${sunTimes.sunset.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit' })}`;
        
        updateMinecraftClockAnimation(sunPos.altitude, sunPos.azimuth);
    }
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

function updateMinecraftClockAnimation(sunAltitudeRad, sunAzimuthRad) {
    const totalAngle = (-sunAzimuthRad * R2D) + 90; 
    const sunElement = $('sun-element');
    const moonElement = $('moon-element');
    const clockDiv = $('minecraft-clock');
    const horizonDiv = document.getElementById('minecraft-clock-horizon');
    const sunAltitudeDeg = sunAltitudeRad * R2D;

    if (sunElement && moonElement) {
        sunElement.style.transform = `rotate(${totalAngle}deg) translateX(0) translateY(-100px)`;
        moonElement.style.transform = `rotate(${totalAngle - 180}deg) translateX(0) translateY(-100px)`; 
        
        const altitudeOffset = Math.sin(sunAltitudeRad) * 50; 
        sunElement.style.top = `calc(50% - 25px - ${altitudeOffset}px)`;
        moonElement.style.top = `calc(50% - 25px + ${altitudeOffset}px)`;
        
        // Ciel
        if (sunAltitudeDeg > 5) {
            $('clock-status').textContent = 'Jour (☀️)';
        } else if (sunAltitudeDeg < -15) {
            $('clock-status').textContent = 'Nuit (🌙)';
        } else {
            $('clock-status').textContent = 'Crépuscule (🌅)';
        }

        horizonDiv.style.opacity = isXRayMode ? 0.2 : 1.0; 
    }
    // Affichage Nether Mode
    $('nether-mode-display').textContent = isNetherMode ? `ACTIF (1:${NETHER_RATIO})` : 'DÉSACTIVÉ (1:1)';
        }
// =========================================================================
// _main_dom.js : Contrôles, Boucle Principale et Rendu DOM/Carte
// =========================================================================

// --- CARTE LEAFLET ---

/** Initialise la carte Leaflet. */
function initMap() {
    if (typeof L !== 'undefined') {
        mapInstance = L.map('map-container').setView([45.0, 5.0], 10);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            maxZoom: 19,
            attribution: '© OpenStreetMap'
        }).addTo(mapInstance);
        $('map-container').textContent = ''; 
    } else {
        $('map-container').textContent = 'Erreur: Bibliothèque Leaflet non chargée.';
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
    let nether_factor = isNetherMode ? NETHER_RATIO : 1.0; 
    const currentSpeed_kmh = currentSpeed * KMH_MS;
    const effectiveSpeed_kmh = currentSpeed_kmh * nether_factor;
    
    $('elapsed-time').textContent = totalTime.toFixed(2) + ' s';
    
    if (currentSpeed >= MIN_SPD) {
        distM_3D += currentSpeed * dt * nether_factor; // Distance effective
        timeMoving += dt;
        maxSpd = Math.max(maxSpd, effectiveSpeed_kmh);
    }
    
    // Affichage Vitesse
    $('speed-stable').textContent = effectiveSpeed_kmh.toFixed(1) + ' km/h';
    $('speed-stable-ms').textContent = (currentSpeed * nether_factor).toFixed(2) + ' m/s';
    $('speed-stable-kms').textContent = (currentSpeed * nether_factor / 1000).toFixed(4) + ' km/s';
    
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
    $('distance-max-horizon').textContent = (horizon_dist !== null && horizon_dist > 0) ? (horizon_dist / 1000).toFixed(1) + ' km' : 'N/A';
    
    // Rapport de Distance (MRF)
    const altRatio = (EARTH_RADIUS + (altEst || 0)) / EARTH_RADIUS;
    $('distance-ratio-mrf').textContent = altRatio.toFixed(3);

    // Distances-Lumière Complètes
    const total_dist_m = distM_3D;
    $('distance-light-s').textContent = (total_dist_m / C_L).toPrecision(2) + ' s';
    $('distance-light-min').textContent = (total_dist_m / (C_L * 60)).toPrecision(2) + ' min';
    $('distance-light-h').textContent = (total_dist_m / (C_L * 3600)).toPrecision(2) + ' h';
    $('distance-light-day').textContent = (total_dist_m / (C_L * 3600 * 24)).toPrecision(2) + ' j';
    $('distance-light-week').textContent = (total_dist_m / (C_L * 3600 * 24 * 7)).toPrecision(2) + ' sem';
    $('distance-light-month').textContent = (total_dist_m / (C_L * 3600 * 24 * 30.4375)).toPrecision(2) + ' mois';
    $('distance-cosmic').textContent = (total_dist_m / AU_M).toPrecision(2) + ' UA | ' + (total_dist_m / LIGHT_YEAR_M).toPrecision(2) + ' al';

    // Mettre à jour les max
    $('magnetic-max-counter').textContent = magFieldMax.toFixed(3) + ' μT';
    $('sound-max-counter').textContent = soundLevelMax.toFixed(1) + ' dB';
    $('light-max-counter').textContent = luminosityMax.toFixed(1) + ' Lux';
    
    // Appel au BLOC 3 pour la physique
    updatePhysicsCalculations(currentSpeed * nether_factor);
}

/** Met à jour les données de position EKF et la vitesse brute. */
function updateGPSDisplay(coords) {
    if (lat !== null) {
        $('lat-display').textContent = (lat * R2D).toFixed(6) + ' °';
        $('lon-display').textContent = (lon * R2D).toFixed(6) + ' °';
        $('alt-ekf').textContent = altEst.toFixed(2) + ' m';
        $('heading-display').textContent = currentHeading !== null ? currentHeading.toFixed(1) + ' °' : 'N/A';
        $('geopotential-alt').textContent = altEst.toFixed(2) + ' m'; // Simplification
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
    
    $('incertitude-vitesse-p').textContent = p_vel !== null ? `${p_vel.toPrecision(3)} (m/s)²` : 'N/A';
    $('incertitude-alt-sigma').textContent = p_alt !== null ? `${Math.sqrt(p_alt).toPrecision(3)} m` : 'N/A';
    
    const R_val = currentAccuracy !== null ? currentAccuracy * currentAccuracy : null;
    $('bruit-mesure-vitesse-r').textContent = R_val !== null ? R_val.toFixed(1) : 'N/A';
    
    // Biais IMU Estimé (depuis X)
    $('accel-biases').textContent = `${X.get([12]).toPrecision(3)}, ${X.get([13]).toPrecision(3)}, ${X.get([14]).toPrecision(3)}`;
    $('gyro-biases').textContent = `${X.get([9]).toPrecision(3)}, ${X.get([10]).toPrecision(3)}, ${X.get([11]).toPrecision(3)}`;
    
    // Fréquence
    const nyquistFreq = 1 / (DT_MS / 1000) / 2;
    $('nyquist-frequency').textContent = nyquistFreq.toFixed(1) + ' Hz';
    $('high-freq-status').textContent = `${(1000 / DT_MS).toFixed(0)} Hz`;
    $('accel-long').textContent = imuAccel.x.toFixed(2) + ' m/s²'; // Accélération X est le proxy Longitudinale
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
        $('statut-gps-ekf').textContent = 'Initialisation du signal...';
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
    $('statut-gps-ekf').textContent = 'Arrêté';
    $('speed-status-text').textContent = 'GPS en Pause / Dead Reckoning Arrêté.';
}

function initControls() {
    // Boutons de contrôle
    $('toggle-gps-btn').addEventListener('click', () => isGPSEnabled ? stopGPS() : startGPS());
    $('emergency-stop-btn').addEventListener('click', () => { 
        stopGPS();
        alert("Arrêt d'urgence: GPS et Boucle principale stoppés.");
        $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: ACTIF 🔴";
    });
    $('reset-dist-btn').addEventListener('click', () => { distM_3D = 0; timeMoving = 0; });
    $('reset-max-btn').addEventListener('click', () => { 
        maxSpd = 0; magFieldMax = 0; soundLevelMax = 0; luminosityMax = 0; 
    });
    $('reset-all-btn').addEventListener('click', () => { window.location.reload(); });
    
    // Mode Nuit
    $('toggle-dark-mode').addEventListener('click', () => {
        isDarkMode = !isDarkMode;
        document.body.classList.toggle('dark-mode', isDarkMode);
        $('toggle-dark-mode').textContent = isDarkMode ? 'Mode Jour' : 'Mode Nuit';
    });
    // Mode X-Ray
    $('sun-moon-xray-btn').addEventListener('click', () => {
        isXRayMode = !isXRayMode;
        $('sun-moon-xray-btn').textContent = `Mode X-Ray: ${isXRayMode ? 'ACTIF' : 'DÉSACTIVÉ'}`;
    });

    // Inputs
    $('gps-accuracy-override').addEventListener('input', (e) => {
        gpsAccuracyOverride = parseFloat(e.target.value) || 0;
        $('forcer-precision-gps-display-label').textContent = `${gpsAccuracyOverride.toFixed(6)} m`;
    });
    $('mass-input').addEventListener('input', (e) => {
        currentMass = parseFloat(e.target.value) || 70;
        $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
    });

    // Logique du Mode Nether (via l'environnement select)
    $('environment-select').addEventListener('change', (e) => {
        // Le mode Souterrain (x0.5) est utilisé comme proxy pour le Nether
        isNetherMode = e.target.value === 'SOUTERRAIN'; 
        $('env-factor').textContent = $('environment-select').options[$('environment-select').selectedIndex].text;
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
    
    // Appel à la logique Météo/Biophysique
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
