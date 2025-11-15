// =================================================================
// BLOC 1/4 : ekf_logic.js
// Constantes de base, filtres EKF (Vitesse/Altitude) et fonctions de calcul physique/mathématique.
// =================================================================

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const R_E_BASE = 6371000;   // Rayon terrestre moyen (m)
const KMH_MS = 3.6;         // Conversion m/s vers km/h
const C_S_BASE = 343.0;     // Vitesse du son de base (m/s)
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const R_AIR = 287.058;      // Constante spécifique de l'air sec (J/kg·K)
const KELVIN_OFFSET = 273.15; // Décalage vers Kelvin

// --- PARAMÈTRES DU FILTRE DE KALMAN (TUNING AVIONIQUE) ---
const Q_NOISE = 0.5;        // Bruit de processus (Augmenté pour dynamiques rapides)
const R_MIN = 0.01;         // Bruit de mesure minimum
const R_MAX = 500.0;        // Bruit de mesure maximum
const MAX_ACC = 500.0;      // Précision max (m) avant "Estimation Seule" (Tolérance avion)
const MIN_SPD = 0.05;       // Vitesse minimale "en mouvement" (m/s)
const ZUPT_RAW_THRESHOLD = 0.1; // Seuil de vitesse pour ZUPT (Zero Velocity Update)
const ZUPT_ACCEL_TOLERANCE = 0.5; // Tolérance d'accélération pour ZUPT (m/s²)
const MIN_DT = 0.01;        // Intervalle de temps minimum pour la mise à jour (s)
const ALT_TH = 10.0;        // Seuil d'altitude pour le statut souterrain (m)
const NETHER_RATIO = 8.0;   // Ratio pour le mode Nether (Minecraft)

// --- ÉTAT EKF (Variable globale accessible) ---
let currentEKFState = {
    lat: 0.0, lon: 0.0, alt: 0.0, 
    acc_est: 10.0, // Précision estimée par l'EKF (Valeur initiale corrigée)
    vel_n: 0.0, vel_e: 0.0, vel_d: 0.0 // Vitesses ENU
};

// --- STUBS DES FONCTIONS EKF ---

function initEKF(lat, lon, alt, acc) {
    currentEKFState.lat = lat;
    currentEKFState.lon = lon;
    currentEKFState.alt = alt;
    currentEKFState.acc_est = acc; 
    currentEKFState.vel_n = currentEKFState.vel_e = currentEKFState.vel_d = 0.0;
    console.log(`EKF Initialisé à ${lat}, ${lon}, Alt: ${alt}m. Acc: ${acc}m.`);
}

function predictEKF(dt, acc_meas, gyro_meas, G_ACC, R_ALT_CENTER_REF) {
    // Phase de prédiction: Propagation de l'incertitude.
    currentEKFState.acc_est *= (1.0 + Q_NOISE * dt * 0.1); 
}

function updateEKF_ZUPT() {
    // Mise à jour de vitesse nulle (Zero Velocity Update)
    currentEKFState.vel_n = currentEKFState.vel_e = currentEKFState.vel_d = 0.0;
    currentEKFState.acc_est *= 0.95; 
}

function updateEKF_GNSS(gnss_pos, gnss_vel, accRaw, altAcc) {
    // Phase de correction: Fusion simplifiée pour les coordonnées
    currentEKFState.lat = gnss_pos.lat;
    currentEKFState.lon = gnss_pos.lon;
    currentEKFState.alt = gnss_pos.alt;
    currentEKFState.acc_est = Math.min(accRaw, currentEKFState.acc_est * 0.95);
}

function updateEKFReadableState() {
    // Met à jour les valeurs EKF simplifiées
}

function getEKFVelocity3D() {
    return Math.sqrt(currentEKFState.vel_n ** 2 + currentEKFState.vel_e ** 2 + currentEKFState.vel_d ** 2) || 0.0;
}

function getVelocityUncertainty() {
    // Simulation d'une incertitude de vitesse cohérente avec la précision
    return currentEKFState.acc_est * 0.01; 
}
// =================================================================
// BLOC 2/4 : utils_astro_weather.js
// Fonctions utilitaires, horloge, météo, astro.
// =================================================================

// --- FONCTIONS UTILITAIRES DE BASE ---

/** Référence rapide au DOM */
function $(id) {
    return document.getElementById(id);
}

// --- GESTION DE L'HORLOGE ET NTP ---

/**
 * Synchronise l'horloge locale avec un serveur NTP simulé.
 * @returns {Promise<{lServH: number, lLocH: number}>} Horloge Serveur et Locale.
 */
function syncH(lServH, lLocH) {
    return new Promise((resolve, reject) => {
        // Simulation d'une tentative de synchro NTP
        if (Math.random() < 0.2) { 
             return reject(new Error("Erreur de réseau simulée ou serveur NTP non trouvé."));
        }
        
        const now = Date.now();
        const serverTime = now + 50; 
        
        if ($('local-time')) $('local-time').textContent = new Date(now).toLocaleTimeString('fr-FR');

        resolve({
            lServH: serverTime, 
            lLocH: now          
        });
    });
}

/**
 * Calcule l'heure corrigée en utilisant le delta NTP.
 * @returns {Date | null} L'objet Date corrigé.
 */
function getCDate(lServH, lLocH) {
    if (lServH === null || lLocH === null) {
        return new Date(); 
    }
    const delta = lServH - lLocH;
    const now = Date.now();
    return new Date(now + delta);
}

// --- CALCULS MÉTÉO/SON ---

/** Calcule la vitesse du son locale et le nombre de Mach. */
function calculateLocalSpeed(tempC, speedMS) {
    const tempK = tempC + KELVIN_OFFSET;
    const C_S_local = Math.sqrt(1.4 * R_AIR * tempK); 
    const mach = speedMS / C_S_local;
    return { C_S_local, mach };
}

/** Simule la récupération des données météo. */
async function fetchWeather(lat, lon) {
    if (lat === 0 && lon === 0) return null;

    try {
        await new Promise(resolve => setTimeout(resolve, 100)); 
        
        const tempC = 19.0; 
        const pressure_hPa = 1012.0;
        const humidity_perc = 82.0;

        const tempK = tempC + KELVIN_OFFSET;
        const air_density = (pressure_hPa * 100) / (R_AIR * tempK); 
        
        return {
            tempC, tempK, pressure_hPa, humidity_perc, air_density,
            dew_point: 15.9,
            status: "Dégagé" 
        };
    } catch (e) {
        return null;
    }
}

// --- CALCULS ASTRONOMIQUES (Stubs) ---

function updateAstro(lat, lon, lServH, lLocH) {
    if (lat === 0 && lon === 0) return null;
    
    const now = getCDate(lServH, lLocH);
    
    // Stubs (basés sur les valeurs de l'état initial)
    const sunPos = { altitude: 0.414, azimuth: -0.465 }; 
    const moonPos = { altitude: 0.705, azimuth: 0.601 }; 
    const moonIllum = { fraction: 0.202, phase: 0.8 }; 
    const sunTimes = { sunriseEnd: new Date(now.getTime() - 3600000), sunsetStart: new Date(now.getTime() + 3600000), dusk: new Date(now.getTime() + 7200000), dawn: new Date(now.getTime() - 7200000) };
    const solarTimes = { TST: '10:17:48', MST: '10:02:15', EOT: 15.55, ECL_LONG: 231.32, NoonSolar: '12:00:00' };

    return { now, sunPos, moonIllum, moonPos, sunTimes, solarTimes };
}

function getMinecraftTime(now) {
    const hours = now.getHours();
    const minutes = now.getMinutes();
    return `${(hours + 6) % 24}:${minutes.toString().padStart(2, '0')}`;
}

function getMoonPhaseName(phase) {
    if (phase < 0.0625 || phase >= 0.9375) return 'Nouvelle Lune';
    if (phase < 0.1875) return 'Premier Croissant';
    if (phase < 0.3125) return 'Premier Quartier';
    if (phase < 0.4375) return 'Lune Gibbeuse Croissante';
    if (phase < 0.5625) return 'Pleine Lune';
    if (phase < 0.6875) return 'Lune Gibbeuse Décroissante';
    if (phase < 0.8125) return 'Dernier Quartier';
    return 'Dernier Croissant 🌘';
                 }
// =================================================================
// BLOC 3/4 : celestial_data.js
// Données des corps célestes et fonctions de calcul de gravité/distance.
// =================================================================

const CELESTIAL_DATA = {
    'EARTH': { G: 9.80665, R: 6371000, ROTATION_RATE: OMEGA_EARTH, DISPLAY: 'Terre' },
    'MOON': { G: 1.625, R: 1737400, ROTATION_RATE: 0, DISPLAY: 'Lune' },
    'MARS': { G: 3.7207, R: 3389500, ROTATION_RATE: 7.0882e-5, DISPLAY: 'Mars' },
    'ROTATING': { G: 9.80665, R: 6371000, ROTATION_RATE: 0, DISPLAY: 'Rotation Perso' }
};

const ENVIRONMENT_FACTORS = {
    'NORMAL': { DISPLAY: 'Normal', R_MULT: 1.0 },
    'FOREST': { DISPLAY: 'Forêt', R_MULT: 2.5 },
    'CONCRETE': { DISPLAY: 'Urbain', R_MULT: 4.0 }, 
    'TUNNEL': { DISPLAY: 'Tunnel', R_MULT: 10.0 }
};

/** Calcule la gravité locale (ajustée pour l'altitude et la rotation). */
function getGravityLocal(altitude, body, rotationRadius, angularVelocity) {
    const data = CELESTIAL_DATA[body];
    const G_base = data.G;
    const R_body = data.R;
    
    const R_total = R_body + altitude;
    const G_alt = G_base * (R_body / R_total) ** 2;
    
    if (body === 'ROTATING') {
        return G_alt - rotationRadius * angularVelocity ** 2;
    }
    
    return G_alt;
}

/** Met à jour les constantes globales G_ACC et R_ALT_CENTER_REF. */
function updateCelestialBody(body, alt, rotationRadius, angularVelocity) {
    const data = CELESTIAL_DATA[body];
    let G_ACC = data.G;
    let R_ALT_CENTER_REF = data.R;

    if (body === 'ROTATING') {
        G_ACC = CELESTIAL_DATA['EARTH'].G - rotationRadius * angularVelocity ** 2;
        R_ALT_CENTER_REF = R_E_BASE;
    }
    
    // Note: currentCelestialBody est une variable globale dans AppController
    
    return { G_ACC, R_ALT_CENTER_REF };
}

/** Calcule le Rapport de Distance (MRF : Map Ratio Factor) pour le mode Nether. */
function calculateMRF(altitude, netherMode) {
    let R_FACTOR_RATIO = 1.0;
    if (netherMode) {
        R_FACTOR_RATIO = NETHER_RATIO;
    } else if (altitude < 0) {
        R_FACTOR_RATIO = 1.0; 
    }
    return R_FACTOR_RATIO;
        }
// =================================================================
// BLOC 4A/4 : AppController.js (Partie 1: Contrôles et Utilitaires)
// État Global, Gestion des Capteurs, Fonctions de Contrôle GPS/Carte.
// =================================================================

// --- CONSTANTES DE CONFIGURATION SYSTÈME ---
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};
const DOM_SLOW_UPDATE_MS = 1000; 
let lastMapUpdate = 0; 
const MAP_UPDATE_INTERVAL = 3000; 
const DEFAULT_INIT_LAT = 45.749950;
const DEFAULT_INIT_LON = 4.850027;
const DEFAULT_INIT_ALT = 2.64;

// --- VARIABLES D'ÉTAT (Globales) ---
let wID = null, domID = null, lPos = null, sTime = null;
let distM_3D = 0, maxSpd = 0;
let timeMoving = 0; 
let lServH = null, lLocH = null;    // Horloge NTP
let lastFSpeed = 0; 

// État Physique et Contrôles
let currentGPSMode = 'HIGH_FREQ'; 
let emergencyStopActive = false;
let netherMode = false; 
let selectedEnvironment = 'NORMAL'; 
let currentMass = 70.0; 
let R_FACTOR_RATIO = 1.0;
let currentCelestialBody = 'EARTH';
let rotationRadius = 100;
let angularVelocity = 0.0; 
let gpsAccuracyOverride = 0.0; 
let G_ACC = CELESTIAL_DATA['EARTH'].G;      
let R_ALT_CENTER_REF = R_E_BASE;            
let currentEnvFactor = 1.0;

// Données externes et IMU
let lastP_hPa = null, lastT_K = null, lastH_perc = null; 
let real_accel_x = 0, real_accel_y = 0, real_accel_z = 0;
let lastAccelLong = 0;

// Objets Map (Leaflet)
let map, marker, circle;


// --------------------------------------------------------------------------
// --- GESTION DES CAPTEURS IMU ---
// --------------------------------------------------------------------------

function imuMotionHandler(event) {
    if (event.acceleration) {
        real_accel_x = event.acceleration.x || 0;
        real_accel_y = event.acceleration.y || 0;
        real_accel_z = event.acceleration.z || 0;
        if ($('imu-status')) $('imu-status').textContent = "Actif (Sans Gravité)";
    } 
    else if (event.accelerationIncludingGravity) {
        real_accel_x = event.accelerationIncludingGravity.x || 0;
        real_accel_y = event.accelerationIncludingGravity.y || 0;
        real_accel_z = event.accelerationIncludingGravity.z || 0;
        if ($('imu-status')) $('imu-status').textContent = "Actif (Avec Gravité)";
    } else {
        if ($('imu-status')) $('imu-status').textContent = "Erreur (Capteur N/A)";
    }
}

function startIMUListeners() {
    if (window.DeviceMotionEvent) {
        if (typeof DeviceMotionEvent.requestPermission === 'function') {
            DeviceMotionEvent.requestPermission().then(permissionState => {
                if (permissionState === 'granted') {
                    window.addEventListener('devicemotion', imuMotionHandler);
                }
            }).catch(console.error);
        } else {
            window.addEventListener('devicemotion', imuMotionHandler);
        }
    } else {
         if ($('imu-status')) $('imu-status').textContent = "Non supporté";
    }
}

function stopIMUListeners() {
    if (window.DeviceMotionEvent) {
        window.removeEventListener('devicemotion', imuMotionHandler);
    }
    if ($('imu-status')) $('imu-status').textContent = "Inactif";
}

// --------------------------------------------------------------------------
// --- GESTION DES CARTES ---
// --------------------------------------------------------------------------

function initMap() {
    try {
        if ($('map') && typeof L !== 'undefined') { 
            map = L.map('map').setView([DEFAULT_INIT_LAT, DEFAULT_INIT_LON], 12);
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                attribution: '© OpenStreetMap contributors'
            }).addTo(map);
            marker = L.marker([DEFAULT_INIT_LAT, DEFAULT_INIT_LON]).addTo(map);
            circle = L.circle([DEFAULT_INIT_LAT, DEFAULT_INIT_LON], { color: 'red', fillColor: '#f03', fillOpacity: 0.5, radius: 10 }).addTo(map);
        }
    } catch (e) {
        console.error("Erreur d'initialisation de Leaflet (Carte):", e);
        if ($('map')) $('map').innerHTML = "Erreur d'initialisation de la carte. (Leaflet N/A)";
    }
}

function updateMap(lat, lon, acc) {
    if (map && marker) {
        marker.setLatLng([lat, lon]);
        circle.setLatLng([lat, lon]).setRadius(acc * R_FACTOR_RATIO); 
        const now = Date.now();
        if (now - lastMapUpdate > MAP_UPDATE_INTERVAL && getEKFVelocity3D() > MIN_SPD) {
            map.setView([lat, lon], map.getZoom() > 10 ? map.getZoom() : 16); 
            lastMapUpdate = now;
        } else if (map.getZoom() < 10 && (Date.now() - lastMapUpdate > 5000)) {
            map.setView([lat, lon], 12);
            lastMapUpdate = now;
        }
    }
}

// --------------------------------------------------------------------------
// --- FONCTIONS DE CONTRÔLE GPS & SYSTÈME ---
// --------------------------------------------------------------------------

function setGPSMode(mode) {
    currentGPSMode = mode;
    if (wID !== null) {
        stopGPS(false); 
        startGPS();     
    }
    if ($('freq-select')) $('freq-select').value = mode; 
}

function startGPS() {
    if (wID !== null) return; 
    
    const options = (currentGPSMode === 'HIGH_FREQ') ? GPS_OPTS.HIGH_FREQ : GPS_OPTS.LOW_FREQ;
    
    wID = navigator.geolocation.watchPosition(updateDisp, handleErr, options); 
    startIMUListeners(); 
    
    if ($('toggle-gps-btn')) {
        $('toggle-gps-btn').textContent = '⏸️ PAUSE GPS';
        $('toggle-gps-btn').style.backgroundColor = '#ffc107'; 
    }
}

function stopGPS(resetButton = true) {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
    }
    stopIMUListeners(); 
    
    if (resetButton) {
        if ($('toggle-gps-btn')) {
            $('toggle-gps-btn').textContent = '▶️ MARCHE GPS';
            $('toggle-gps-btn').style.backgroundColor = '#28a745'; 
        }
    }
}

function emergencyStop() {
    emergencyStopActive = true;
    stopGPS(false);
    if ($('emergency-stop-btn')) {
        $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: ACTIF 🔴";
        $('emergency-stop-btn').classList.add('active');
    }
    ['speed-stable', 'speed-3d-inst', 'distance-total-km', 'local-time'].forEach(id => {
        if ($(id)) $(id).textContent = 'ARRÊT D’URGENCE';
    });
}

function resumeSystem() {
    emergencyStopActive = false;
    if ($('emergency-stop-btn')) {
        $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: INACTIF 🟢";
        $('emergency-stop-btn').classList.remove('active');
    }
    startGPS();
}

function handleErr(err) {
    console.warn(`ERREUR GPS (${err.code}): ${err.message}`);
    if ($('gps-precision')) $('gps-precision').textContent = `Erreur: ${err.message}`;
    
    if (err.code === 1) { 
        stopGPS();
        alert("Accès à la géolocalisation refusé. Veuillez l'activer.");
    }
    }
// =================================================================
// BLOC 4B/4 : AppController.js (Partie 2: Logique Principale, EKF et Initialisation)
// Fonctions de Mise à Jour (EKF, DOM Lent) et Gestion du Démarrage.
// Dépendances: Toutes les variables globales et fonctions de 4A/4, plus les Blocs 1/4, 2/4 et 3/4.
// =================================================================

// --------------------------------------------------------------------------
// --- FONCTION PRINCIPALE DE MISE À JOUR GPS/EKF (updateDisp) ---
// --------------------------------------------------------------------------

function updateDisp(pos) {
    if (emergencyStopActive) return;

    // --- 1. ACQUISITION DES DONNÉES ET INITIALISATION ---
    const cTimePos = pos.timestamp;
    const now = getCDate(lServH, lLocH); 
    
    if (now === null) return; 
    if (sTime === null) sTime = now.getTime();
    
    let accRaw = pos.coords.accuracy;
    if (gpsAccuracyOverride > 0.0) accRaw = gpsAccuracyOverride;

    let dt = 0;
    if (lPos) {
        dt = (cTimePos - lPos.timestamp) / 1000;
    } else {
        // Première position: Initialisation de l'EKF
        lPos = pos; 
        initEKF(pos.coords.latitude, pos.coords.longitude, pos.coords.altitude || DEFAULT_INIT_ALT, accRaw);
        updateMap(currentEKFState.lat, currentEKFState.lon, currentEKFState.acc_est);
        return; 
    }
    
    if (dt < MIN_DT || dt > 10) { 
        lPos = pos; 
        return; 
    }
    
    // 2. PRÉDICTION EKF (Basée sur l'IMU et le temps)
    predictEKF(dt, [real_accel_x, real_accel_y, real_accel_z], [0, 0, 0], G_ACC, R_ALT_CENTER_REF);
    
    // 3. LOGIQUE ZUPT
    const V_ekf = getEKFVelocity3D();
    const isPlausiblyStopped = (
        V_ekf < ZUPT_RAW_THRESHOLD && 
        Math.abs(lastAccelLong) < ZUPT_ACCEL_TOLERANCE
    ); 
    
    if (isPlausiblyStopped) { 
        updateEKF_ZUPT();
    }
    
    // 4. CORRECTION EKF (Fusion GNSS)
    const isSignalLost = (accRaw > MAX_ACC);
    currentEnvFactor = ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT;
    
    if (!isSignalLost) {
        const gnss_pos = { lat: pos.coords.latitude, lon: pos.coords.longitude, alt: pos.coords.altitude || currentEKFState.alt };
        const gnss_vel = { V_n: pos.coords.velocity, V_e: 0, V_d: 0 }; 
        
        updateEKF_GNSS(gnss_pos, gnss_vel, accRaw, pos.coords.altitudeAccuracy || 5.0);
    }
    
    // 5. MISE À JOUR DE L'ÉTAT LISIBLE ET CALCULS PHYSIQUES
    updateEKFReadableState(); 
    const sSpdFE = V_ekf < MIN_SPD ? 0 : V_ekf; 
    
    let accel_long = 0;
    if (dt > 0.05) { 
        accel_long = (sSpdFE - lastFSpeed) / dt;
    }
    lastFSpeed = sSpdFE;
    lastAccelLong = accel_long;

    R_FACTOR_RATIO = calculateMRF(currentEKFState.alt, netherMode); 
    distM_3D += sSpdFE * dt * R_FACTOR_RATIO; 
    
    if (sSpdFE > MIN_SPD) { timeMoving += dt; }
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    
    const local_g = getGravityLocal(currentEKFState.alt, currentCelestialBody, rotationRadius, angularVelocity); 
    const coriolis_force = 2 * currentMass * sSpdFE * OMEGA_EARTH * Math.sin(currentEKFState.lat * D2R);
    
    let mach_info = { C_S_local: C_S_BASE, mach: 0 };
    if (lastT_K !== null) {
        mach_info = calculateLocalSpeed(lastT_K - KELVIN_OFFSET, sSpdFE);
    }

    // 6. MISE À JOUR DU DOM (Affichage Rapide)
    const acc_est_m = currentEKFState.acc_est;
    const kalman_V_uncert = getVelocityUncertainty();
    const R_dyn = currentEnvFactor * accRaw ** 2; 
    const altStatusTxt = currentEKFState.alt < ALT_TH ? `OUI (< ${ALT_TH}m)` : 'Non';
    const status_mode = isPlausiblyStopped ? '✅ ZUPT' : (isSignalLost ? '⚠️ EST. SEULE' : '🚀 FUSION');

    if ($('time-elapsed')) $('time-elapsed').textContent = `${((Date.now() - sTime) / 1000).toFixed(2)} s`;
    if ($('time-moving')) $('time-moving').textContent = `${timeMoving.toFixed(2)} s`;
    if ($('mode-ratio')) $('mode-ratio').textContent = `${R_FACTOR_RATIO.toFixed(3)} (Ratio)`;
    if ($('env-factor')) $('env-factor').textContent = `${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY} (x${currentEnvFactor.toFixed(1)})`;
    if ($('speed-stable')) $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(5)}`;
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${sSpdFE.toFixed(3)} m/s | ${(sSpdFE * 1e6).toFixed(0)} µm/s | ${(sSpdFE * 1e9).toFixed(0)} nm/s`;
    if ($('speed-max')) $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    if ($('distance-total-km')) $('distance-total-km').textContent = `${(distM_3D / 1000).toFixed(3)} km | ${distM_3D.toFixed(2)} m`;
    if ($('latitude')) $('latitude').textContent = `${currentEKFState.lat.toFixed(6)} °`;
    if ($('longitude')) $('longitude').textContent = `${currentEKFState.lon.toFixed(6)} °`;
    if ($('altitude-gps')) $('altitude-gps').textContent = currentEKFState.alt !== null ? `${currentEKFState.alt.toFixed(2)} m` : 'N/A';
    if ($('heading-display')) $('heading-display').textContent = pos.coords.heading !== null ? `${pos.coords.heading.toFixed(1)} °` : 'N/A';
    if ($('underground-status')) $('underground-status').textContent = `Souterrain: ${altStatusTxt} (${status_mode} | Acc GPS: ${accRaw.toFixed(1)}m)`;
    if ($('gravity-local')) $('gravity-local').textContent = `${local_g.toFixed(5)} m/s²`;
    if ($('accel-long')) $('accel-long').textContent = `${accel_long.toFixed(3)} m/s²`;
    if ($('imu-accel-x')) $('imu-accel-x').textContent = `${real_accel_x.toFixed(2)} m/s²`;
    if ($('kalman-uncert')) $('kalman-uncert').textContent = `${kalman_V_uncert.toFixed(3)} m²/s² (P)`;
    if ($('speed-error-perc')) $('speed-error-perc').textContent = `${R_dyn.toFixed(3)} m² (R dyn)`;
    if ($('gps-precision')) $('gps-precision').textContent = `${acc_est_m.toFixed(2)} m (Est.)`;
    if ($('coriolis-force')) $('coriolis-force').textContent = `${coriolis_force.toExponential(2)} N`;
    if ($('speed-sound-local')) $('speed-sound-local').textContent = `${mach_info.C_S_local.toFixed(2)} m/s`;
    if ($('mach-number')) $('mach-number').textContent = `${mach_info.mach.toFixed(4)}`;
    
    // 7. SAUVEGARDE & MISE À JOUR CARTE
    lPos = pos; 
    lPos.timestamp = cTimePos; 
    updateMap(currentEKFState.lat, currentEKFState.lon, acc_est_m);
}

// --------------------------------------------------------------------------
// --- FONCTIONS DOM VISUELLES ET LENTES ---
// --------------------------------------------------------------------------

function updateClockVisualization(now, sunPos, moonPos, sunTimes) {
    const sunEl = $('sun-element');
    const moonEl = $('moon-element');
    const clockEl = $('minecraft-clock'); 

    if (!sunEl || !moonEl || !clockEl || !sunPos || !moonPos || !sunTimes) return;

    const sunIcon = sunEl.querySelector('.sun-icon');
    const moonIcon = moonEl.querySelector('.moon-icon');

    const altDegSun = sunPos.altitude * R2D;
    const aziDegSun = (sunPos.azimuth * R2D + 180) % 360; 
    sunEl.style.transform = `rotate(${aziDegSun}deg)`;
    const radialPercentSun = Math.min(50, Math.max(0, 50 * (90 - altDegSun) / 90));
    const altitudeOffsetPercentSun = 50 - radialPercentSun; 
    if (sunIcon) sunIcon.style.transform = `translateY(calc(-50% + ${altitudeOffsetPercentSun}%) )`; 
    sunEl.style.display = altDegSun > -0.83 ? 'flex' : 'none'; 

    const altDegMoon = moonPos.altitude * R2D;
    const aziDegMoon = (moonPos.azimuth * R2D + 180) % 360; 
    moonEl.style.transform = `rotate(${aziDegMoon}deg)`;
    const radialPercentMoon = Math.min(50, Math.max(0, 50 * (90 - altDegMoon) / 90));
    const altitudeOffsetPercentMoon = 50 - radialPercentMoon; 
    if (moonIcon) moonIcon.style.transform = `translateY(calc(-50% + ${altitudeOffsetPercentMoon}%) )`;
    moonEl.style.display = altDegMoon > 0 ? 'flex' : 'none';

    const body = document.body;
    body.classList.remove('sky-day', 'sky-sunset', 'sky-night', 'dark-mode', 'light-mode');
    clockEl.classList.remove('sky-day', 'sky-sunset', 'sky-night');

    const nowMs = now.getTime();
    let bodyClass;
    if (sunTimes.sunriseEnd && sunTimes.sunsetStart && sunTimes.dusk && sunTimes.dawn) {
        if (nowMs >= sunTimes.sunriseEnd.getTime() && nowMs < sunTimes.sunsetStart.getTime()) {
            bodyClass = 'sky-day';
        } else if (nowMs >= sunTimes.dusk.getTime() || nowMs < sunTimes.dawn.getTime()) {
            bodyClass = 'sky-night';
        } else {
            bodyClass = 'sky-sunset';
        }
    } else {
        bodyClass = 'sky-night';
    }
    
    body.classList.add(bodyClass);
    body.classList.add(bodyClass === 'sky-day' ? 'light-mode' : 'dark-mode');
    clockEl.classList.add(bodyClass); 

    $('clock-status').textContent = altDegSun > 0 ? 'Jour Solaire (☀️)' : 'Nuit/Crépuscule (🌙)';
}


// --------------------------------------------------------------------------
// --- INITIALISATION DES ÉVÉNEMENTS DOM ---
// --------------------------------------------------------------------------

document.addEventListener('DOMContentLoaded', () => {
    
    initMap(); 
    // CORRECTION EKF: Précision initiale de 10.0m
    initEKF(DEFAULT_INIT_LAT, DEFAULT_INIT_LON, DEFAULT_INIT_ALT, 10.0);

    // --- Initialisation des Contrôles ---
    const massInput = $('mass-input'); 
    if (massInput) {
        massInput.addEventListener('input', () => { 
            currentMass = parseFloat(massInput.value) || 70.0; 
            if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        });
        currentMass = parseFloat(massInput.value); 
        if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
    }

    if ($('celestial-body-select')) {
        $('celestial-body-select').addEventListener('change', (e) => { 
            const newVals = updateCelestialBody(e.target.value, currentEKFState.alt, rotationRadius, angularVelocity);
            G_ACC = newVals.G_ACC;
            R_ALT_CENTER_REF = newVals.R_ALT_CENTER_REF;
            currentCelestialBody = e.target.value;
            if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC.toFixed(4)} m/s²`;
        });
    }

    const updateRotation = () => {
        rotationRadius = parseFloat($('rotation-radius')?.value) || 100;
        angularVelocity = parseFloat($('angular-velocity')?.value) || 0;
        if (currentCelestialBody === 'ROTATING') {
            const newVals = updateCelestialBody('ROTATING', currentEKFState.alt, rotationRadius, angularVelocity);
            G_ACC = newVals.G_ACC;
            if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC.toFixed(4)} m/s²`;
        }
    };
    if ($('rotation-radius')) $('rotation-radius').addEventListener('input', updateRotation);
    if ($('angular-velocity')) $('angular-velocity').addEventListener('input', updateRotation);

    if ($('environment-select')) {
        $('environment-select').addEventListener('change', (e) => { 
            if (emergencyStopActive) return;
            selectedEnvironment = e.target.value; 
            if ($('env-factor')) $('env-factor').textContent = `${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY} (x${ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT.toFixed(1)})`; 
        });
        if ($('env-factor')) $('env-factor').textContent = `${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY} (x${ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT.toFixed(1)})`;
    }
    
    if ($('gps-accuracy-override')) {
        $('gps-accuracy-override').addEventListener('change', (e) => {
            gpsAccuracyOverride = parseFloat(e.target.value) || 0.0;
        });
    }

    // --- Boutons Principaux ---
    if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', () => { 
        if (emergencyStopActive) { alert("Veuillez désactiver l'Arrêt d'urgence."); return; }
        wID === null ? startGPS() : stopGPS(); 
    });
    if ($('freq-select')) $('freq-select').addEventListener('change', (e) => setGPSMode(e.target.value));
    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', () => { 
        !emergencyStopActive ? emergencyStop() : resumeSystem(); 
    });
    if ($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        netherMode = !netherMode; 
        if ($('mode-nether')) $('mode-nether').textContent = netherMode ? `ACTIVÉ (1:${NETHER_RATIO}) 🔥` : "DÉSACTIVÉ (1:1)"; 
    });
    
    // --- Réinitialisations ---
    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        distM_3D = 0; timeMoving = 0; 
    });
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        maxSpd = 0; 
    });
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        if (confirm("Réinitialiser toutes les données de session, y compris l'EKF ?")) { 
            distM_3D = 0; maxSpd = 0; timeMoving = 0; lPos = null; sTime = null;
            initEKF(DEFAULT_INIT_LAT, DEFAULT_INIT_LON, DEFAULT_INIT_ALT, 10.0);
        } 
    });
    if ($('toggle-mode-btn')) $('toggle-mode-btn').addEventListener('click', () => {
        document.body.classList.toggle('dark-mode');
    });
    
    // --- DÉMARRAGE DU SYSTÈME ---
    
    const initVals = updateCelestialBody(currentCelestialBody, currentEKFState.alt, rotationRadius, angularVelocity);
    G_ACC = initVals.G_ACC;
    R_ALT_CENTER_REF = initVals.R_ALT_CENTER_REF;
    
    // CORRECTION NTP: Gestion de l'échec et démarrage garanti du GPS.
    syncH(lServH, lLocH).then(newTimes => {
        lServH = newTimes.lServH;
        lLocH = newTimes.lLocH;
        startGPS(); 
    }).catch(err => {
        console.warn("Échec de la synchronisation NTP. Démarrage du GPS avec heure locale.", err);
        if ($('local-time')) $('local-time').textContent = 'SYNCHRO ÉCHOUÉE';
        startGPS(); 
    });

    // Boucle de mise à jour lente (Astro/Météo/Horloge)
    if (domID === null) {
        domID = setInterval(async () => {
            const currentLat = currentEKFState.lat; 
            const currentLon = currentEKFState.lon;
            
            // 1. Météo : Récupération et Affichage
            if (currentLat !== 0 && currentLon !== 0 && !emergencyStopActive) {
                const data = await fetchWeather(currentLat, currentLon);
                
                if (data) {
                    lastP_hPa = data.pressure_hPa;
                    lastT_K = data.tempK;
                    lastH_perc = data.humidity_perc / 100.0;
                    
                    if ($('temp-air-2')) $('temp-air-2').textContent = `${data.tempC.toFixed(1)} °C`;
                    if ($('pressure-2')) $('pressure-2').textContent = `${data.pressure_hPa.toFixed(0)} hPa`;
                    if ($('humidity-2')) $('humidity-2').textContent = `${data.humidity_perc} %`;
                    if ($('air-density')) $('air-density').textContent = `${data.air_density.toFixed(3)} kg/m³`;
                    if ($('dew-point')) $('dew-point').textContent = `${data.dew_point.toFixed(1)} °C`;
                    if ($('weather-status')) $('weather-status').textContent = data.status; 
                } else {
                    if ($('weather-status')) $('weather-status').textContent = 'ÉCHEC RÉCUPÉRATION (Réseau ou API)';
                    lastP_hPa = lastT_K = lastH_perc = null; 
                }
            }
            
            // 2. Astro : Calculs et Affichage
            const astroData = updateAstro(currentLat, currentLon, lServH, lLocH);

            if (astroData) {
                const { now, sunPos, moonIllum, moonPos, sunTimes, solarTimes } = astroData;

                if ($('time-minecraft')) $('time-minecraft').textContent = getMinecraftTime(now);
                if ($('date-display-astro')) $('date-display-astro').textContent = now.toLocaleDateString('fr-FR');
                if ($('tst')) $('tst').textContent = solarTimes.TST;
                if ($('mst')) $('mst').textContent = solarTimes.MST;
                if ($('eot')) $('eot').textContent = solarTimes.EOT + ' min'; 
                if ($('ecl-long')) $('ecl-long').textContent = solarTimes.ECL_LONG + ' °'; 
                if ($('sun-alt')) $('sun-alt').textContent = `${(sunPos.altitude * R2D).toFixed(2)} °`;
                if ($('sun-azimuth')) $('sun-azimuth').textContent = `${(sunPos.azimuth * R2D).toFixed(2)} ° (S-O)`;
                if ($('moon-alt')) $('moon-alt').textContent = `${(moonPos.altitude * R2D).toFixed(2)} °`;
                if ($('moon-azimuth')) $('moon-azimuth').textContent = `${(moonPos.azimuth * R2D).toFixed(2)} ° (S-O)`;
                if ($('moon-illuminated')) $('moon-illuminated').textContent = `${(moonIllum.fraction * 100).toFixed(1)} %`;
                if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(moonIllum.phase);
                if ($('noon-solar')) $('noon-solar').textContent = solarTimes.NoonSolar;
                
                updateClockVisualization(now, sunPos, moonPos, sunTimes);
            } else {
                $('clock-status').textContent = 'Astro (Attente GPS/SunCalc)...';
            }
            
            // 3. Horloge NTP : Resynchronisation toutes les minutes
            if (Math.floor(Date.now() / 1000) % 60 === 0) {
                 syncH(lServH, lLocH).then(newTimes => {
                    lServH = newTimes.lServH;
                    lLocH = newTimes.lLocH;
                 }).catch(() => { /* Échec géré au démarrage */ });
            }
            
            // 4. Horloge locale (NTP)
            const nowTime = getCDate(lServH, lLocH);
            if (nowTime) {
                if ($('local-time') && !$('local-time').textContent.includes('SYNCHRO ÉCHOUÉE')) {
                    $('local-time').textContent = nowTime.toLocaleTimeString('fr-FR');
                }
                if ($('date-display')) $('date-display').textContent = nowTime.toLocaleDateString('fr-FR');
            }
            
        }, DOM_SLOW_UPDATE_MS); 
    }
});
