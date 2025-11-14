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
let X = math.matrix(math.zeros(N_STATES)._data.flat());
let P = math.diag(math.zeros(N_STATES)._data.flat());

// Matrice de Bruit Processus (Q)
let Q_diag = new Array(N_STATES).fill(1e-6); 
Q_diag[0] = Q_diag[1] = Q_diag[2] = 1e-4;
Q_diag[3] = Q_diag[4] = Q_diag[5] = 1e-3;
let Q = math.diag(Q_diag);

// --- ÉTAT DES CAPTEURS ET COMPTEURS ---
let lat = null, lon = null, altEst = null, speedEst = 0.0; 
let currentAccuracy = null;
let currentHeading = null;
let lastGPSPosition = null;

let imuAccel = {x: 0, y: 0, z: 0};
let imuGyro = {x: 0, y: 0, z: 0};

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
// _ekf_core.js : Moteur EKF (21-States), Géolocalisation et IMU (Corrigé)
// =========================================================================

/**
 * Initialise l'état EKF et les variables globales avec une position GPS initiale.
 */
function initEKF(lat_init, lon_init, alt_init) {
    lat = lat_init !== null ? lat_init * D2R : 45.0 * D2R; 
    lon = lon_init !== null ? lon_init * D2R : 5.0 * D2R; 
    altEst = alt_init !== null ? alt_init : 0.0;
    
    X.set([0], lat); X.set([1], lon); X.set([2], altEst);
    P = math.diag(Q_diag);
}

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
    
    const R_M = EARTH_RADIUS + altM; 
    const R_N_prime = (EARTH_RADIUS + altM) * Math.cos(latRad); 

    const dLat = (Vn * dt) / R_M;
    const dLon = (Ve * dt) / R_N_prime;
    const dAlt = -Vd * dt;

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

// Les fonctions gpsSuccess, gpsError et initializeIMUSensors restent inchangées.

function gpsSuccess(position) { /* ... (Fonction inchangée) ... */ }
function gpsError(error) { /* ... (Fonction inchangée) ... */ }
function initializeIMUSensors() { /* ... (Fonction inchangée) ... */ }
// =========================================================================
// _physics_astro_weather.js : Physique Avancée, Astronomie et Météo (Corrigé)
// =========================================================================

// --- FONCTIONS DE BASE (Inchangées) ---

function calculateSpeedOfSound(tempC) { /* ... (Inchangée) ... */ return 20.0468 * Math.sqrt(tempC + KELVIN_OFFSET); }
function calculateAirDensity(pressurehPa, tempC) { /* ... (Inchangée) ... */ }
function calculateDewPoint(tempC, humidity) { /* ... (Inchangée) ... */ }
function calculateTimeDilationGravity(altM) { /* ... (Inchangée) ... */ }
function updateWeatherAndBiophysics() { /* ... (Inchangée) ... */ }
function updatePhysicsCalculations(currentSpeed) { 
    // ... (Code robuste inchangé) 
    const g_local = altEst !== null ? G_BASE * Math.pow(EARTH_RADIUS / (EARTH_RADIUS + altEst), 2) : null;
    $('gravity-local').textContent = g_local !== null ? g_local.toFixed(5) + ' m/s²' : G_BASE.toFixed(4) + ' m/s² (Base)';
    $('angular-speed').textContent = angularSpeed !== 0 ? angularSpeed.toFixed(2) + ' °/s' : '0.00 °/s';
    $('accel-long').textContent = imuAccel.x !== 0 ? imuAccel.x.toFixed(2) + ' m/s²' : '0.00 m/s²'; 
    // ...
}

// --- LOGIQUE ASTRO (Corrigée) ---

function formatHours(decimalHours) {
    // Normalise l'heure décimale entre 0 et 24
    decimalHours = (decimalHours % 24 + 24) % 24;
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

function getMoonPhaseName(phase) { /* ... (Inchangée) ... */ }

function updateAstroCalculations() {
    const coords = { lat: lat * R2D || 45.0, lon: lon * R2D || 5.0 };
    const date = new Date();
    
    // --- SYNCHRONISATION GMT / AFFICHAGE HEURE ---
    $('local-time').textContent = date.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit', second: '2-digit' });
    // Affichage UTC (GMT) pour la synchronisation
    $('date-display').textContent = date.toLocaleDateString('fr-FR', { timeZone: 'UTC' }) + ' ' + 
                                     date.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit', second: '2-digit', timeZone: 'UTC' });
    
    $('date-display-astro').textContent = date.toLocaleDateString('fr-FR');
    
    if (typeof SunCalc !== 'undefined') {
        const sunTimes = SunCalc.getTimes(date, coords.lat, coords.lon);
        const sunPos = SunCalc.getPosition(date, coords.lat, coords.lon);
        const moonPos = SunCalc.getMoonPosition(date, coords.lat, coords.lon);
        const moonIllumination = SunCalc.getMoonIllumination(date);
        
        // --- CALCULS TEMPS SOLAIRE CORRIGÉS ---
        
        // 1. UTC Time en heures décimales
        const UTC_hours_dec = date.getUTCHours() + date.getUTCMinutes() / 60 + date.getUTCSeconds() / 3600;
        
        // 2. Équation du Temps (EOT) en minutes
        const EOT_seconds = sunPos.equationOfTime || 0;
        const EOT_minutes = EOT_seconds / 60; 
        const EOT_hours = EOT_minutes / 60;
        
        // 3. Heure Solaire Moyenne (MST) : UTC + Longitude (deg) / 15
        const MST_hours = UTC_hours_dec + coords.lon / 15;
        
        // 4. Heure Solaire Vraie (TST) : MST + EOT (en heures)
        const TST_hours = MST_hours + EOT_hours;

        // Affichage des temps solaires
        $('mst').textContent = formatHours(MST_hours);
        $('tst').textContent = formatHours(TST_hours);
        
        // Calcul Minecraft (basé sur le TST)
        const mc_ticks = (TST_hours * 1000) - 6000;
        $('time-minecraft').textContent = formatMinecraftTime(mc_ticks);

        // Affichage Astro
        $('sun-alt').textContent = (sunPos.altitude * R2D).toFixed(2) + ' °';
        $('sun-azimuth').textContent = (sunPos.azimuth * R2D).toFixed(2) + ' °';
        $('moon-alt').textContent = (moonPos.altitude * R2D).toFixed(2) + ' °';
        $('moon-azimuth').textContent = (moonPos.azimuth * R2D).toFixed(2) + ' °';
        $('moon-phase-name').textContent = getMoonPhaseName(moonIllumination.phase);
        $('moon-illuminated').textContent = (moonIllumination.fraction * 100).toFixed(1) + ' %';
        
        // EOT et Longitude Écliptique (maintenant fiables)
        $('noon-solar').textContent = sunTimes.solarNoon.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit', timeZone: 'UTC' });
        $('eot').textContent = EOT_minutes.toFixed(2) + ' min'; 
        $('ecl-long').textContent = (sunPos.eclipticLongitude * R2D).toFixed(2) + ' °';
        
        // Placeholders non calculés
        $('date-solar-mean').textContent = date.toLocaleDateString('fr-FR');
        $('date-solar-true').textContent = date.toLocaleDateString('fr-FR');
        
        updateMinecraftClockAnimation(sunPos.altitude, sunPos.azimuth);
    }
}

function updateMinecraftClockAnimation(sunAltitudeRad, sunAzimuthRad) { /* ... (Inchangée) ... */ }
// =========================================================================
// _main_dom.js : Contrôles, Boucle Principale et Rendu DOM/Carte (Inchangé)
// =========================================================================

// --- CARTE LEAFLET (Inchangée) ---
function initMap() { /* ... (Inchangée) ... */ }
function updateMap(newLatDeg, newLonDeg, accuracy) { /* ... (Inchangée) ... */ }

// --- MISE À JOUR DU DOM (Inchangée) ---
function updateCompteurs(currentSpeed, dt) { /* ... (Inchangée) ... */ }
function updateGPSDisplay(coords) { /* ... (Inchangée) ... */ }
function updateEKFDisplay() { /* ... (Inchangée) ... */ }

// --- GESTION DES CONTRÔLES (Inchangée) ---
function startGPS() { /* ... (Inchangée) ... */ }
function stopGPS() { /* ... (Inchangée) ... */ }
function initControls() { /* ... (Inchangée) ... */ }

// --- BOUCLES ET INITIALISATION (Mise à jour pour forcer la météo/astro) ---

/** Boucle principale de l'application (20 Hz) */
function domUpdateLoop() { /* ... (Inchangée) ... */ }

/** Point d'entrée de l'application */
function init() {
    initControls();
    initializeIMUSensors();
    initEKF(null, null, 0); 
    initMap();
    
    // Appel initial pour remplir le DOM
    if (typeof updateWeatherAndBiophysics === 'function') {
        updateWeatherAndBiophysics();
    }
    updateAstroCalculations(); // Appel immédiat pour les temps solaires
    
    // Boucle rapide (EKF Predict + DOM)
    setInterval(domUpdateLoop, DT_MS); 
    
    // Boucle lente (Astro, Météo (si non temps réel))
    setInterval(updateAstroCalculations, 10000);
}

document.addEventListener('DOMContentLoaded', init);
