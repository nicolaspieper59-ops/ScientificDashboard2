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
