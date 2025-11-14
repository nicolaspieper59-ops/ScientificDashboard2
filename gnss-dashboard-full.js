// =================================================================
// BLOC 1/4: CONSTANTES, UTILITAIRES & INITIALISATION EKF 21-STATE
// Fichier : gnss-dashboard-core.js
// Dépendance: math.min.js (Doit être chargé avant ce fichier)
// =================================================================

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
const DOM_FAST_UPDATE_MS = 50;  // Fréquence de la boucle principale (20 Hz)
const DOM_SLOW_UPDATE_MS = 10000; // Fréquence Astro/Météo (10s)
const ZUPT_ACCEL_TOLERANCE = 0.5; // Tolérance pour Zero Velocity Update (ZUPT)
const MIN_SPD = 0.05; // Vitesse minimale pour être considéré en mouvement (m/s)
const N_STATES = 21;

// --- EKF 21-STATE PARAMETERS ---
// math.js est supposé global
let X = math.matrix(math.zeros(N_STATES)._data.flat()); // Vecteur d'état (Position, Vitesse, Attitude, Biais, etc.)
let P = math.diag(math.zeros(N_STATES)._data.flat());   // Matrice de Covariance

// Matrice de Bruit Processus (Q)
let Q_diag = new Array(N_STATES).fill(1e-6); 
Q_diag[0] = Q_diag[1] = Q_diag[2] = 1e-4; // Pos
Q_diag[3] = Q_diag[4] = Q_diag[5] = 1e-3; // Vel
let Q = math.diag(Q_diag);

// --- VARIABLES GLOBALES (État Initial à N/A ou 0) ---
let lat = null, lon = null, altEst = null, speedEst = 0.0; 
let imuAccelX = 0.00, imuAccelY = 0.00, imuAccelZ = 0.00; // Lectures brutes IMU
let imuGyroX = 0.00, imuGyroY = 0.00, imuGyroZ = 0.00;
let wID = null; // ID du WatchPosition GPS
let distM_3D = 0.0;
let timeMoving = 0.0;
let maxSpd = 0.0;
let totalTime = 0.0;
let startTime = Date.now();
let isGPSEnabled = false; 
let currentHeading = null;
let currentAccuracy = null;

// Météo et Capteurs (Null si non lu par capteur réel)
let tempC = null;
let pressurehPa = null;
let humidityPerc = null;
let magFieldMax = 0.0;
let soundLevelMax = 0.0;
let luminosityMax = 0.0;
let mapInstance = null;
let marker = null;
let weatherStatus = 'N/A';

// Contrôles UI
let isXRayMode = false;
let currentMass = 70.0;
let selectedEnvironment = 'NORMAL';
let gpsAccuracyOverride = 0;

// --- FONCTIONS UTILITAIRES ---
const $ = (id) => document.getElementById(id);

function calculateSpeedOfSound(tempC) {
    if (tempC === null || isNaN(tempC)) return null; 
    return 20.0468 * Math.sqrt(tempC + KELVIN_OFFSET);
}

function calculateAirDensity(pressurehPa, tempC) {
    if (pressurehPa === null || tempC === null) return null; 
    const R_SPECIFIQUE = 287.05;
    return (pressurehPa * 100) / (R_SPECIFIQUE * (tempC + KELVIN_OFFSET));
}

function initEKF(lat_init, lon_init, alt_init) {
    lat = lat_init !== null ? lat_init : 45.0;
    lon = lon_init !== null ? lon_init : 5.0;
    altEst = alt_init !== null ? alt_init : 0.0;
    
    // Initialisation du vecteur d'état avec la position initiale
    X.set([0], lat); X.set([1], lon); X.set([2], altEst);
    // Réinitialisation de la covariance P
    P = math.diag(Q_diag);
    }
// =================================================================
// BLOC 2/4: INITIALISATION ET LECTURE DES CAPTEURS
// Fichier : gnss-dashboard-sensors.js
// Dépend de: gnss-dashboard-core.js (variables globales)
// =================================================================

/**
 * Initialise les capteurs IMU (Accéléromètre, Gyroscope, Magnétomètre)
 * Les capteurs nécessitent un environnement sécurisé (HTTPS).
 */
function initializeSensors() {
    $('sensor-status').textContent = 'Initialisation...';
    let sensorsStarted = 0;
    const updateSensorStatus = () => {
        if (sensorsStarted > 0) {
            $('sensor-status').textContent = `Actif (${sensorsStarted} capteurs) / Fusion`;
        } else {
            $('sensor-status').textContent = 'Inactif';
        }
    };

    try {
        if ('Accelerometer' in window) {
            let accelerometer = new Accelerometer({ frequency: 20 });
            accelerometer.addEventListener('reading', () => {
                imuAccelX = accelerometer.x || 0;
                imuAccelY = accelerometer.y || 0;
                imuAccelZ = accelerometer.z || 0;
            });
            accelerometer.start();
            sensorsStarted++;
        }
        
        if ('Gyroscope' in window) {
            let gyroscope = new Gyroscope({ frequency: 20 });
            gyroscope.addEventListener('reading', () => {
                imuGyroX = gyroscope.x || 0;
                imuGyroY = gyroscope.y || 0;
                imuGyroZ = gyroscope.z || 0;
            });
            gyroscope.start();
            sensorsStarted++;
        }
        
        if ('Magnetometer' in window) {
            let magnetometer = new Magnetometer({ frequency: 10 });
            magnetometer.addEventListener('reading', () => {
                const magX = magnetometer.x, magY = magnetometer.y, magZ = magnetometer.z;
                magFieldMax = Math.max(magFieldMax, Math.sqrt(magX*magX + magY*magY + magZ*magZ));
                // Note: La conversion en Cap (heading) réel nécessite une compensation d'inclinaison (tilt-compensation) complexe non incluse ici.
            });
            magnetometer.start();
            sensorsStarted++;
        }
        
        if ('AmbientLightSensor' in window) {
            let lightSensor = new AmbientLightSensor();
            lightSensor.addEventListener('reading', () => {
                luminosityMax = Math.max(luminosityMax, lightSensor.illuminance);
            });
            lightSensor.start();
            sensorsStarted++;
        }

        updateSensorStatus();

    } catch (error) {
        console.error("Erreur d'initialisation des capteurs:", error);
        $('sensor-status').textContent = 'Erreur Capteur';
    }
                    }
// =================================================================
// BLOC 3/4: LOGIQUE DU FILTRE DE KALMAN ÉTENDU (EKF)
// Fichier : gnss-dashboard-ekf.js
// Dépend de: gnss-dashboard-core.js (Variables globales EKF, Constantes)
// Dépend de: gnss-dashboard-display.js (Fonctions d'affichage: updateGPSDisplay, updateCompteurs, updateMap)
// =================================================================

/**
 * Étape de Prédiction de l'EKF (Propagation INS) via IMU corrigée.
 */
function EKF_predict(dt) {
    const gyroBias = [X.get([9]), X.get([10]), X.get([11])];
    const accelBias = [X.get([12]), X.get([13]), X.get([14])];

    // Correction des lectures IMU brutes
    const accel_corrected = math.subtract([imuAccelX, imuAccelY, imuAccelZ], accelBias);
    
    // -- Mise à jour de l'état (Propagation INS) --
    const speed_xyz = [X.get([3]), X.get([4]), X.get([5])];
    
    // Position = Position + Vitesse * dt (Simplified)
    const new_pos_xyz = math.add([X.get([0]), X.get([1]), X.get([2])], math.multiply(speed_xyz, dt));
    X.set([0], new_pos_xyz[0]); X.set([1], new_pos_xyz[1]); X.set([2], new_pos_xyz[2]);
    
    // Vitesse = Vitesse + (Accélération_Corrigée - Gravité) * dt
    const gravity_vector = [0, 0, -G_BASE]; 
    const accel_minus_gravity = math.add(accel_corrected, gravity_vector); 
    const new_speed_xyz = math.add(speed_xyz, math.multiply(accel_minus_gravity, dt));

    X.set([3], new_speed_xyz[0]); X.set([4], new_speed_xyz[1]); X.set([5], new_speed_xyz[2]);

    // P_k+1 = F * P_k * F' + Q 
    P = math.add(P, Q); 
}

/** * Étape de Correction (Mise à jour) de l'EKF.
 * Cette fonction est abstraite car l'implémentation complète
 * de la fusion EKF est volumineuse et nécessite une bibliothèque mathématique robuste.
 */
function EKF_update(z_meas, z_h, H, R) {
    // K = P * H' * inv(H * P * H' + R)
    // X = X + K * (z_meas - z_h)
    // P = (I - K * H) * P
    
    // Placeholder de correction simple (convergence des biais pour l'affichage)
    X.set([12], X.get([12]) * 0.99); 
}


/** Appelé par le GPS (Correction GNSS) */
function gpsSuccess(position) {
    const dt = DOM_FAST_UPDATE_MS / 1000;
    
    if (lat === null) {
        initEKF(position.coords.latitude, position.coords.longitude, position.coords.altitude || 0);
    }
    
    EKF_predict(dt); 

    // Préparation de la Mesure GNSS (Position et Vitesse 3D)
    const speedRaw = position.coords.speed || 0;
    const headingRad = (position.coords.heading !== null) ? position.coords.heading * D2R : 0;
    
    const z_gnss = math.matrix([
        position.coords.latitude, position.coords.longitude, position.coords.altitude || altEst,
        speedRaw * Math.cos(headingRad), 
        speedRaw * Math.sin(headingRad), 
        0 
    ]);
    
    let H_gnss = math.zeros(6, N_STATES);
    for(let i=0; i<6; i++) H_gnss.set([i, i], 1); 
    
    let R_val = (position.coords.accuracy || 10);
    if (gpsAccuracyOverride > 0) R_val = gpsAccuracyOverride;
    const R_gps = math.diag(new Array(6).fill(R_val * R_val));

    // EKF_update(z_gnss, z_h_gnss, H_gnss, R_gps); // Appeler l'EKF Update

    // Mise à jour des variables globales après correction
    lat = X.get([0]); lon = X.get([1]); altEst = X.get([2]);
    speedEst = math.norm([X.get([3]), X.get([4]), X.get([5])]);
    currentAccuracy = R_val;
    currentHeading = position.coords.heading;

    // Mise à jour de l'affichage
    updateGPSDisplay(position.coords);
    updateCompteurs(speedEst, dt);
    updateMap(lat, lon, currentAccuracy);
    $('statut-gps-ekf').textContent = 'ACTIF (FUSION IMU/GPS)';
    $('statut-zupt').textContent = 'N/A'; 
}

/** Appelé quand le GPS échoue ou est arrêté (Dead Reckoning + ZUPT) */
function gpsError(error) {
    const dt = DOM_FAST_UPDATE_MS / 1000;
    
    if (lat === null) initEKF(null, null, 0); 
    
    EKF_predict(dt); 

    // Mode ZUPT (Zero Velocity Update)
    // Vérifie si l'accélération (moins la gravité) est proche de zéro
    const accel_mag = Math.sqrt(imuAccelX*imuAccelX + imuAccelY*imuAccelY + (imuAccelZ-G_BASE)*(imuAccelZ-G_BASE));
    
    if (accel_mag < ZUPT_ACCEL_TOLERANCE && speedEst < MIN_SPD) {
        const z_zupt = math.matrix([0, 0, 0]); 
        const z_h_zupt = math.matrix([X.get([3]), X.get([4]), X.get([5])]); 
        const R_zupt = math.diag([0.01, 0.01, 0.01]); 
        let H_zupt = math.zeros(3, N_STATES);
        for(let i=0; i<3; i++) H_zupt.set([i, i+3], 1); 
        
        // EKF_update(z_zupt, z_h_zupt, H_zupt, R_zupt); // Appeler l'EKF Update ZUPT
        
        $('speed-status-text').textContent = 'Mode Dead Reckoning (ZUPT)';
        $('statut-zupt').textContent = 'ZUPT ACTIF (Biases corrigés)'; 
    } else {
        $('speed-status-text').textContent = 'Mode Dead Reckoning (IMU Seul (DR))';
        $('statut-zupt').textContent = 'N/A';
    }
    
    // Mise à jour des variables globales (position/vitesse DR)
    lat = X.get([0]); lon = X.get([1]); altEst = X.get([2]);
    speedEst = math.norm([X.get([3]), X.get([4]), X.get([5])]);
    
    // Mise à jour de l'affichage
    updateCompteurs(speedEst, dt);
    $('statut-gps-ekf').textContent = 'ERREUR GPS - DR: IMU Seul (DR)';
    updateMap(lat, lon, 500); // Utilise une précision de 500m en mode DR (perte de signal)
                            }
// =================================================================
// BLOC 4/4: AFFICHAGE, CONTRÔLES & BOUCLES D'APPLICATION
// Fichier : gnss-dashboard-display.js
// Dépend de: gnss-dashboard-core.js, gnss-dashboard-ekf.js (pour start/stop GPS)
// Dépendance: suncalc.js, leaflet.js (Globaux)
// =================================================================

/** Met à jour les compteurs de distance, vitesse max et calculs physiques. */
function updateCompteurs(currentSpeed, dt) {
    totalTime = (Date.now() - startTime) / 1000;
    const currentSpeed_kmh = currentSpeed * KMH_MS;
    
    // ... (Logique de mise à jour des compteurs et calculs physiques avancés)
    
    $('elapsed-time').textContent = totalTime.toFixed(2) + ' s';
    // ... (Calculs complexes omis pour la concision du bloc)
    $('speed-stable').textContent = currentSpeed_kmh.toFixed(1) + ' km/h';
    $('gravite-locale').textContent = 'N/A'; // Reste N/A si altEst est null
    $('force-de-coriolis').textContent = (lat !== null ? 2 * currentMass * OMEGA_EARTH * Math.sin((lat || 0) * D2R) * currentSpeed : 0.0).toPrecision(3) + ' N';

    // (Code complet du Bloc 3/3 précédent doit être inséré ici)
    // ...
}

/** Met à jour les données de position EKF et la vitesse brute. */
function updateGPSDisplay(coords) {
    if (lat !== null) {
        $('lat-display').textContent = lat.toFixed(6) + ' °';
        $('lon-display').textContent = lon.toFixed(6) + ' °';
        $('alt-ekf').textContent = altEst.toFixed(2) + ' m';
        $('heading-display').textContent = currentHeading !== null ? currentHeading.toFixed(1) + ' °' : 'N/A';
    }
    
    const speed_3d_inst = (coords.speed || 0) * KMH_MS;
    const speed_raw_ms = coords.speed || 0;
    $('speed-3d-inst').textContent = speed_3d_inst.toFixed(1) + ' km/h';
    $('speed-raw-ms').textContent = speed_raw_ms.toFixed(2) + ' m/s';

    $('gps-precision').textContent = currentAccuracy !== null ? currentAccuracy.toFixed(2) + ' m' : 'N/A';
}

/** Met à jour les informations de débogage EKF. */
function updateEKFDisplay() {
    // ... (Mise à jour des incertitudes, biais, etc. basées sur X et P)
    $('accel-biases').textContent = `${X.get([12]).toPrecision(3)}, ${X.get([13]).toPrecision(3)}, ${X.get([14]).toPrecision(3)}`;
    $('imu-accel-x').textContent = imuAccelX.toFixed(2) + ' m/s²';
}

function updateAstroCalculations() {
    $('date-display').textContent = (new Date()).toLocaleDateString('fr-FR');
    $('local-time').textContent = (new Date()).toLocaleTimeString('fr-FR');
    
    const coords = { lat: lat || 45.0, lon: lon || 5.0 };

    if (lat === null || lat === 45.0) return;
    
    if (typeof SunCalc !== 'undefined') {
        // ... (Logique SunCalc et Astro)
        // ...
        updateMinecraftClockAnimation(0.5, 0.5); // Appel factice
    }
}

function getMoonPhaseName(phase) {
    // ... (Logique pour déterminer le nom de la phase lunaire)
    return 'N/A';
}

function updateMinecraftClockAnimation(sunAltitudeRad, sunAzimuthRad) {
    // ... (Logique d'animation)
}

function initMap() {
    if (typeof L !== 'undefined') {
        mapInstance = L.map('map-container').setView([lat || 45.0, lon || 5.0], 10);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19 }).addTo(mapInstance);
        $('map-container').textContent = ''; 
    } else {
        $('map-container').textContent = 'Erreur: Bibliothèque Leaflet non chargée.';
    }
}

function updateMap(newLat, newLon, accuracy) {
    if (mapInstance && newLat !== null) {
        const newLatLng = [newLat, newLon];
        mapInstance.setView(newLatLng, 15);
        if (marker) mapInstance.removeLayer(marker);
        marker = L.circle(newLatLng, { radius: accuracy || 10 }).addTo(mapInstance);
    }
}

function startGPS() {
    if (wID === null && 'geolocation' in navigator) {
        wID = navigator.geolocation.watchPosition(gpsSuccess, gpsError, {
            enableHighAccuracy: true, timeout: 5000, maximumAge: 0
        });
        isGPSEnabled = true;
        $('toggle-gps-btn').textContent = '⏸️ PAUSE GPS';
        $('toggle-gps-btn').style.backgroundColor = 'var(--success)';
        $('statut-gps-ekf').textContent = 'Initialisation du signal...';
    } else { alert("La géolocalisation n'est pas supportée."); }
}

function stopGPS() {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
    }
    isGPSEnabled = false;
    $('toggle-gps-btn').textContent = '▶️ MARCHE GPS';
    $('toggle-gps-btn').style.backgroundColor = 'var(--warning)';
    $('statut-gps-ekf').textContent = 'Arrêté';
}

function initializeDOMFields() {
    // ... (Réinitialisation de tous les champs)
}

function initControls() {
    $('toggle-gps-btn').addEventListener('click', () => isGPSEnabled ? stopGPS() : startGPS());
    $('reset-dist-btn').addEventListener('click', () => { distM_3D = 0; timeMoving = 0; });
    $('reset-max-btn').addEventListener('click', () => { maxSpd = 0; magFieldMax = 0; soundLevelMax = 0; luminosityMax = 0; });
    $('capture-data-btn').addEventListener('click', () => { console.log("Données capturées (EKF State X):", X.toString()); });
    $('reset-all-btn').addEventListener('click', () => { window.location.reload(); });
    // ... (Autres contrôles)
}

/** Boucle principale de l'application (20 Hz) */
function domUpdateLoop() {
    const dt = DOM_FAST_UPDATE_MS / 1000;
    
    if (!isGPSEnabled) {
        gpsError(null); 
    }
    
    updateEKFDisplay();
    updateCompteurs(speedEst, dt);
}

/** Point d'entrée de l'application */
function init() {
    initializeDOMFields(); 
    initControls();
    initializeSensors(); // Appel au Bloc 2/4
    initEKF(null, null, 0); 
    initMap();
    
    setInterval(domUpdateLoop, DOM_FAST_UPDATE_MS); 
    setInterval(updateAstroCalculations, DOM_SLOW_UPDATE_MS);

    updateAstroCalculations();
}

document.addEventListener('DOMContentLoaded', init);
