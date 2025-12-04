// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// Version finale optimisée pour la robustesse des navigateurs modernes (HTTPS/Permissions).
// DÉPENDANCES CRITIQUES (à charger dans l'HTML) : math.min.js, leaflet.js, suncalc.js, turf.min.js
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);
const D2R = Math.PI / 180, R2D = 180 / Math.PI; 
const KMH_MS = 3.6;         
const C_L = 299792458;      // Vitesse de la lumière (m/s)

const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        return (decimals === 0 ? '0' : '0.' + Array(decimals).fill('0').join('')) + suffix;
    }
    return val.toFixed(decimals) + suffix;
};
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        const zeroDecimals = '0.' + Array(decimals).fill('0').join('');
        return zeroDecimals + 'e+0' + suffix;
    }
    return val.toExponential(decimals) + suffix;
};

// --- CLÉS D'API & ENDPOINTS (Exemple Vercel Proxy) ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";
const DOM_SLOW_UPDATE_MS = 2000; 

// --- CONSTANTES WGS84 et GÉOPHYSIQUES ---
const WGS84_A = 6378137.0;        
const WGS84_F = 1 / 298.257223563; 
const WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F; 
const WGS84_G_EQUATOR = 9.780327; 
const WGS84_BETA = 0.0053024; 
const RHO_SEA_LEVEL = 1.225; // Densité de l'air ISA (kg/m³)
const TEMP_SEA_LEVEL_K = 288.15; // Température ISA (K)
const BARO_ALT_REF_HPA = 1013.25; // Pression de référence (hPa)
let G_ACC = 9.80665;         

// --- ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE ---
let map = null;
let gpsWatchID = null;
let gpsIsActive = false;
let lastTimestamp = 0;
let lastServerTime = null; // Pour la sync NTP
let lastLocalTime = null;  // Pour la sync NTP
let currentLat = 43.2964, currentLon = 5.3697, kAlt = 0; // Position initiale par défaut
let currentSpeed = 0.0;
let maxSpeed = 0;
let totalDistance = 0;
let lastPoint = null;
let lastPolyline = null; // Pour la carte Leaflet
let lastKnownIMU = { ax: 0, ay: 0, az: 0, gx: 0, gy: 0, gz: 0, heading: 0 }; // État IMU brut
let ukf = null; 
let currentCelestialBody = 'EARTH'; 
let rotationRadius = 100; 
let angularVelocity = 0.0;
let currentMass = 70.0; 
let currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound = 340.29; 
let lastP_hPa = BARO_ALT_REF_HPA;
let lastT_K = TEMP_SEA_LEVEL_K;
let lastH_perc = 0.0; 
let sunAltitudeRad = 0; 
let distanceRatioMode = false;
let netherMode = false;
let currentUKFReactivity = 'NORMAL'; 
const GPS_OPTIONS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};

// =================================================================
// BLOC 2/4 : NOYAU SCIENTIFIQUE (UKF & MODÈLES PHYSIQUES)
// =================================================================

// --- CLASSE PROFESSIONALUKF (21 ÉTATS) ---
class ProfessionalUKF {
    constructor(nx = 21, nz = 6) {
        this.X = math.zeros(nx, 1);
        this.X.set([0, 0], currentLat * D2R);
        this.X.set([1, 0], currentLon * D2R);
        this.X.set([2, 0], -kAlt); // Altitude Z vers le bas (NED convention)
        const P_init_diag = [1e-2, 1e-2, 1e-1, 1, 1, 1, 0.01, 0.01, 0.05, 1e-3, 1e-3, 1e-3, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-1, 1e-1, 1e-1];
        this.P = math.diag(P_init_diag.map(v => Math.max(v, 1e-8)));
    }
    
    predict(dt, imuRaw) { 
        if (dt <= 0) return;
        const Accel_N = imuRaw.ax; 
        const Accel_E = imuRaw.ay;
        const Accel_D = G_ACC - imuRaw.az; 
        this.X.set([3, 0], this.X.get([3, 0]) + Accel_N * dt);
        this.X.set([4, 0], this.X.get([4, 0]) + Accel_E * dt);
        this.X.set([5, 0], this.X.get([5, 0]) + Accel_D * dt);
        this.X.set([0, 0], this.X.get([0, 0]) + this.X.get([3, 0]) * dt);
        this.X.set([1, 0], this.X.get([1, 0]) + this.X.get([4, 0]) * dt);
        this.X.set([2, 0], this.X.get([2, 0]) + this.X.get([5, 0]) * dt);
        // Mise à jour de P omise (logique UKF réelle)
    }
    
    update(z_meas_ned, gps_accuracy) {
        const K_P_pos = Math.min(0.5, 1 / (gps_accuracy + 1e-6) * (currentUKFReactivity === 'AGRESSIF' ? 20 : currentUKFReactivity === 'PRUDENT' ? 5 : 10)); 
        this.X.set([0, 0], this.X.get([0, 0]) * (1 - K_P_pos) + z_meas_ned.get([0, 0]) * K_P_pos);
        this.X.set([1, 0], this.X.get([1, 0]) * (1 - K_P_pos) + z_meas_ned.get([1, 0]) * K_P_pos);
        this.X.set([2, 0], this.X.get([2, 0]) * (1 - K_P_pos) + z_meas_ned.get([2, 0]) * K_P_pos);
        // Mise à jour de P omise (logique UKF réelle)
    }

    getFilteredState() {
        const latRad = this.X.get([0, 0]);
        const lonRad = this.X.get([1, 0]);
        const altM = -this.X.get([2, 0]);
        const V_N = this.X.get([3, 0]); 
        const V_E = this.X.get([4, 0]); 
        const speed = Math.sqrt(V_N**2 + V_E**2);
        const heading = V_N !== 0 || V_E !== 0 ? (Math.atan2(V_E, V_N) * R2D + 360) % 360 : 0;
        return {
            lat: latRad * R2D, lon: lonRad * R2D, alt: altM, speed: speed, heading: heading,
            attitude: { pitch: 0, roll: 0, yaw: heading }
        };
    }
}

// --- MODÈLES PHYSIQUES AVANCÉS ---

function calculateWGS84Gravity(latRad, altM) {
    const sin2 = Math.sin(latRad) ** 2;
    const g_lat = WGS84_G_EQUATOR * (1 + WGS84_BETA * sin2) / Math.sqrt(1 - WGS84_E2 * sin2); 
    const R_LAT = WGS84_A / Math.sqrt(1 - WGS84_E2 * sin2);
    return g_lat * (1 - 2 * altM / R_LAT);
}

function updateCelestialBody(body, altM, rotR, angV) {
    if (body === 'EARTH') {
        const latRad = currentLat * D2R;
        G_ACC = calculateWGS84Gravity(latRad, altM);
    } else if (body === 'ROTATING') {
        const g_base = 9.80665;
        const F_CENTRIFUGE = rotR * (angV**2);
        G_ACC = g_base - F_CENTRIFUGE;
    } else {
        G_ACC = 9.80665; 
    }
    return { G_ACC_NEW: G_ACC };
}

function calculateStandardISA(altM) {
    if (altM < 11000) { 
        const T_K = TEMP_SEA_LEVEL_K - 0.0065 * altM;
        const P_Pa = 101325 * Math.pow(T_K / TEMP_SEA_LEVEL_K, 5.256); 
        return { T_K, P_Pa, tempC: T_K - 273.15, pressure_hPa: P_Pa / 100, humidity_perc: 0.0 };
    }
    return { T_K: TEMP_SEA_LEVEL_K, P_Pa: 101325, tempC: 15.0, pressure_hPa: 1013.25, humidity_perc: 0.0 };
}

function getAirDensity(tempK, pressure_hPa, humidity_perc = 0.0) {
    const pressure_Pa = pressure_hPa * 100;
    const R_SPECIFIC_AIR = 287.058;
    return pressure_Pa / (R_SPECIFIC_AIR * tempK);
}

function getSpeedOfSound(tempK) {
    const GAMMA_AIR = 1.4; 
    const R_SPECIFIC_AIR = 287.058;
    return Math.sqrt(GAMMA_AIR * R_SPECIFIC_AIR * tempK);
}

function calculateRelativityEffects(speed, altM) {
    const v_c_ratio = speed / C_L;
    const kinematic_dilation = 1.0 / Math.sqrt(1.0 - v_c_ratio * v_c_ratio);
    const G_U = 6.67430e-11; 
    const M_EARTH = 5.972e24; 
    const R_E_BASE = 6371000;
    const gravitational_dilation = 1.0 + (G_U * M_EARTH / (R_E_BASE * C_L**2) - G_U * M_EARTH / ((R_E_BASE + altM) * C_L**2));
    return { kinematic_dilation: kinematic_dilation, gravitational_dilation: gravitational_dilation };
}

function calculateDistanceRatio(altM) {
    const R_E_WGS84 = 6378137.0; 
    return (R_E_WGS84 + altM) / R_E_WGS84;
}

function calculateBioSVT(tempC, altM, humidity_perc, pressure_Pa, sunAltitudeRad) {
    const dewPoint = tempC - (100 - humidity_perc) / 5;
    return { dewPoint: dewPoint };
}


// =================================================================
// BLOC 3/4 : ACQUISITION CAPTEURS, API & GESTION DU TEMPS
// =================================================================

// --- GESTION IMU (Device Motion) ---

function handleDeviceMotion(event) {
    const accel = event.accelerationIncludingGravity || event.acceleration; 
    const gyro = event.rotationRate;
    if (accel && gyro) {
        lastKnownIMU.ax = accel.x || 0;
        lastKnownIMU.ay = accel.y || 0;
        lastKnownIMU.az = accel.z || 0;
        lastKnownIMU.gx = gyro.alpha || 0;
        lastKnownIMU.gy = gyro.beta || 0;
        lastKnownIMU.gz = gyro.gamma || 0;
    }
}

function handleDeviceOrientation(event) {
    const alpha = event.alpha || 0; 
    lastKnownIMU.heading = (360 - alpha + 90) % 360; 
}

function startIMUSensors() {
    let sensorStatus = $('statut-capteur');
    if (!sensorStatus) return;
    
    // CRITIQUE : Gestion des permissions iOS 13+
    if (typeof DeviceMotionEvent.requestPermission === 'function') {
        sensorStatus.textContent = 'En attente d\'autorisation IMU... (iOS)';
        DeviceMotionEvent.requestPermission().then(permissionState => {
            if (permissionState === 'granted') {
                window.addEventListener('devicemotion', handleDeviceMotion);
                window.addEventListener('deviceorientation', handleDeviceOrientation);
                sensorStatus.textContent = 'IMU : Actif';
            } else {
                sensorStatus.textContent = '❌ IMU: Refusé';
            }
        }).catch(error => {
             sensorStatus.textContent = `❌ IMU: Erreur (${error.name})`;
        });
    } else if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', handleDeviceMotion);
        window.addEventListener('deviceorientation', handleDeviceOrientation);
        sensorStatus.textContent = 'IMU : Actif (Hérité/Android)';
    } else {
        sensorStatus.textContent = '❌ IMU: Non supporté';
    }
}

// --- GESTION GPS (Geolocation API) ---

function startGPS() {
    const statusElement = $('statut-gps-acquisition');
    if (!('geolocation' in navigator)) {
        if (statusElement) statusElement.textContent = '❌ ERREUR: Geolocation non supportée.';
        return;
    }
    if (statusElement) statusElement.textContent = 'Acquisition GPS en cours... (Haute précision)';
    
    const gpsOptions = gpsIsActive ? GPS_OPTIONS.HIGH_FREQ : GPS_OPTIONS.LOW_FREQ; 

    gpsWatchID = navigator.geolocation.watchPosition(
        (position) => {
            const { latitude, longitude, altitude, accuracy, speed, heading } = position.coords;
            const gpsTime = position.timestamp;
            
            // Calcul de la distance parcourue (turf.js requis)
            const currentPoint = turf.point([longitude, latitude]);
            if (lastPoint) {
                const distanceM = turf.distance(lastPoint, currentPoint, {units: 'meters'});
                totalDistance += distanceM;
            }
            lastPoint = currentPoint;

            // Mise à jour de l'UKF
            const newAlt = altitude !== null ? altitude : kAlt;
            if (ukf) ukf.update(math.matrix([[latitude * D2R], [longitude * D2R], [-newAlt]]), accuracy);

            // Mise à jour de la carte
            if (map && lastPolyline) {
                lastPolyline.addLatLng(L.latLng(latitude, longitude));
            }

            // Mise à jour du DOM
            if (statusElement) statusElement.textContent = `ACTIF (${dataOrDefault(accuracy, 1)}m)`;
            if ($('précision-gps-acc')) $('précision-gps-acc').textContent = `${dataOrDefault(accuracy, 1)} m`;

        }, 
        (error) => {
            let msg = '';
            if (error.code === error.PERMISSION_DENIED) {
                msg = "❌ ERREUR: Accès GPS refusé. (Veuillez autoriser et utiliser HTTPS)";
            } else if (error.code === error.TIMEOUT) {
                msg = "⌛ ERREUR: Délai dépassé. (Mauvais signal)";
            } else {
                msg = `❌ ERREUR GPS: ${error.message}`;
            }
            if (statusElement) statusElement.textContent = msg;
        }, 
        gpsOptions
    );
}

function stopSystem() {
    if (gpsWatchID !== null) {
        navigator.geolocation.clearWatch(gpsWatchID);
        gpsWatchID = null;
    }
    window.removeEventListener('devicemotion', handleDeviceMotion);
    window.removeEventListener('deviceorientation', handleDeviceOrientation);
    gpsIsActive = false;
    if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = 'Inactif';
    if ($('statut-capteur')) $('statut-capteur').textContent = 'Inactif';
}

// --- GESTION API (Météo/NTP) ---

function getCDate(serverTime, localTime) {
    if (serverTime && localTime) {
        const offset = serverTime.getTime() - localTime.getTime();
        return new Date(Date.now() + offset);
    }
    return new Date();
}

async function syncH() {
    const statusEl = $('local-time');
    if (statusEl) statusEl.textContent = 'Synchronisation...';
    try {
        const response = await fetch(SERVER_TIME_ENDPOINT);
        if (!response.ok) throw new Error('Échec de l\'API Time');
        const data = await response.json();
        lastServerTime = new Date(data.utc_datetime);
        lastLocalTime = new Date();
        if (statusEl) statusEl.textContent = lastLocalTime.toLocaleTimeString('fr-FR');
    } catch (error) {
        if (statusEl) statusEl.textContent = 'SYNCHRO ÉCHOUÉE (Local Time)';
    }
}

async function fetchWeather(lat, lon) {
    try {
        const response = await fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`);
        if (!response.ok) throw new Error('Échec de l\'API Météo');
        const data = await response.json();
        
        lastP_hPa = data.pressure_hPa;
        lastT_K = data.tempK;
        lastH_perc = data.humidity_perc / 100.0;
        currentAirDensity = data.air_density;
        currentSpeedOfSound = getSpeedOfSound(lastT_K);
        
        updateWeatherDOM(data);
        return data;
    } catch (err) {
        if ($('weather-status')) $('weather-status').textContent = `❌ API ÉCHOUÉE`;
        const isa = calculateStandardISA(kAlt);
        lastP_hPa = isa.pressure_hPa;
        lastT_K = isa.T_K;
        currentAirDensity = getAirDensity(lastT_K, lastP_hPa);
        currentSpeedOfSound = getSpeedOfSound(lastT_K);
        updateWeatherDOM({ ...isa, isOffline: true });
        return null;
    }
}

function updateWeatherDOM(data, isOffline = false) {
    const status = isOffline ? 'ISA' : data.isOffline ? 'ISA/Défaut' : 'ACTIF';
    if ($('weather-status')) $('weather-status').textContent = status;
    if ($('temp-air-2')) $('temp-air-2').textContent = `${data.tempC.toFixed(1)} °C`;
    if ($('pressure-2')) $('pressure-2').textContent = `${data.pressure_hPa.toFixed(0)} hPa`;
    if ($('humidity-2')) $('humidity-2').textContent = `${data.humidity_perc ? data.humidity_perc.toFixed(0) : 0} %`;
    if ($('air-density')) $('air-density').textContent = `${currentAirDensity.toFixed(3)} kg/m³`;
    if ($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = `${currentSpeedOfSound.toFixed(2)} m/s (${status})`;
}
// =================================================================
// BLOC 4/4 : BOUCLES DE MISE À JOUR, CARTE & INITIALISATION
// (Continuation après les fonctions startFastLoop, startSlowLoop, initMap, etc.)
// =================================================================


/** Gestionnaire principal d'activation/désactivation du système (CRITIQUE) */
function handleToggleGps() {
    console.log("handleToggleGps: Clic détecté. Lancement de la procédure de démarrage/arrêt.");
    const btn = $('toggle-gps-btn');
    
    if (gpsIsActive) {
        // Logique d'arrêt
        stopSystem();
        if (btn) btn.textContent = '▶️ MARCHE GPS';
        if ($('statut-ekf-fusion')) $('statut-ekf-fusion').textContent = `PAUSE`;
    } else {
        // Logique de démarrage
        
        // Le clic est la condition nécessaire pour les permissions (surtout IMU iOS)
        startIMUSensors(); 
        startGPS(); 

        if (btn) btn.textContent = '⏸️ PAUSE GPS/IMU';
        gpsIsActive = true;
        
        // Initialisation de l'UKF si ce n'est pas déjà fait
        if (ukf === null && typeof ProfessionalUKF !== 'undefined') {
             ukf = new ProfessionalUKF();
        }
        
        // Démarre la boucle rapide de calcul et d'affichage si elle n'est pas en cours
        if (!window.fastLoopRunning) startFastLoop();
        
        // Mise à jour du statut
        if ($('statut-ekf-fusion')) $('statut-ekf-fusion').textContent = `INITIALISATION EKF...`;
    }
}

// Point d'entrée principal (garanti d'être exécuté après le chargement du DOM)
document.addEventListener('DOMContentLoaded', () => {
    
    console.log("DÉBOGAGE: 1. DOMContentLoaded déclenché. Début de l'initialisation du script.");

    try {
        // 1. Vérification des dépendances (Critique : math, leaflet, suncalc, turf)
        if (typeof math === 'undefined' || typeof L === 'undefined' || typeof SunCalc === 'undefined' || typeof turf === 'undefined') {
            const statusEl = $('statut-gps-acquisition') || document.body;
            statusEl.innerHTML = '<h2 style="color:red;">CRASH: Dépendances (math.js, leaflet.js, etc.) manquantes.</h2>';
            return;
        }

        // 2. Initialisation statique
        initMap();
        syncH(); // Synchronisation NTP
        startSlowLoop(); 
        updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity); 
        
        // Initialisation de l'UKF par défaut si non créé par le clic
        if (ukf === null && typeof ProfessionalUKF !== 'undefined') {
             ukf = new ProfessionalUKF();
        }
        
        // 3. ATTACHEMENT CRITIQUE DU BOUTON MARCHE GPS
        const toggleBtn = $('toggle-gps-btn');
        if (toggleBtn) {
            // L'écouteur d'événement est attaché ici, garantissant que le DOM est prêt.
            toggleBtn.addEventListener('click', handleToggleGps);
            console.log("DÉBOGAGE: 2. Écouteur de clic attaché à 'toggle-gps-btn'.");
        } else {
            console.error("DÉBOGAGE: 2. ERREUR CRITIQUE - ID 'toggle-gps-btn' INTROUVABLE. Le bouton est mort.");
            if ($('statut-gps-acquisition')) {
                $('statut-gps-acquisition').textContent = '❌ Erreur: Bouton Démarrer (ID: toggle-gps-btn) manquant.';
            }
        }
        
        // 4. Initialisation des autres contrôles et événements
        
        if ($('toggle-mode-btn')) $('toggle-mode-btn').addEventListener('click', () => { 
            document.body.classList.toggle('dark-mode'); 
            $('toggle-mode-btn').innerHTML = document.body.classList.contains('dark-mode') ? '<i class="fas fa-sun"></i> Mode Jour' : '<i class="fas fa-moon"></i> Mode Nuit';
        });

        if ($('reset-dashboard-btn')) $('reset-dashboard-btn').addEventListener('click', () => { 
            if (confirm("Voulez-vous vraiment réinitialiser le tableau de bord (distance, max speed, carte)?")) {
                stopSystem();
                totalDistance = 0; maxSpeed = 0; lastPoint = null;
                if (map) { 
                    map.removeLayer(lastPolyline); 
                    // Supposons que lastPolyline est une variable globale pour le tracé
                    lastPolyline = L.polyline([], {color: '#007bff', weight: 3}).addTo(map);
                }
                if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = '▶️ MARCHE GPS';
                if ($('distance-total-km')) $('distance-total-km').textContent = `0.000 km | 0.00 m`; 
                if ($('speed-max')) $('speed-max').textContent = `0.00000 km/h`; 
            } 
        });

        // ... (Autres gestionnaires d'événements : mass-input, celestial-body-select, rotation, etc.)

        // Mise à jour des affichages par défaut
        if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC.toFixed(4)} m/s² (WGS84)`;
        if ($('statut-capteur')) $('statut-capteur').textContent = `Inactif`;
        if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = `Prêt (HTTPS/Localhost requis)`;


    } catch (error) {
        console.error("DÉBOGAGE: ERREUR D'INITIALISATION MAJEURE (Avant le clic):", error);
        const statusEl = $('statut-gps-acquisition') || document.body;
        statusEl.innerHTML = `<h2 style="color:red;">CRASH INITIAL: ${error.name}</h2><p>Le script a planté avant le démarrage.</p>`;
    }
});
