// =================================================================
// BLOC 1/4 : Constantes Globales et Configuration
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        return (decimals === 0 ? '0' : '0.00') + suffix;
    }
    return val.toFixed(decimals) + suffix;
};
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        return (decimals === 0 ? '0.00e+0' : val.toExponential(decimals)) + suffix;
    }
    return val.toExponential(decimals) + suffix;
};

// --- CONSTANTES MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI; 
const KMH_MS = 3.6;         
const C_L = 299792458;      

// --- CONSTANTES GÉOPHYSIQUES (WGS84) ---
let G_ACC = 9.80665;         
let R_ALT_CENTER_REF = 6371000; 
const OMEGA_EARTH = 7.2921159e-5; 
const WGS84_A = 6378137.0;  
const WGS84_F = 1 / 298.257223563; 
const WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F; 
const WGS84_G_EQUATOR = 9.780327; 
const WGS84_BETA = 0.0053024; 

// --- CONSTANTES ATMOSPHÉRIQUES (ISA Standard) ---
const BARO_ALT_REF_HPA = 1013.25;
const RHO_SEA_LEVEL = 1.225;
const R_AIR = 287.058;
const GAMMA = 1.4;

// --- PARAMÈTRES DU FILTRE UKF ---
const UKF_R_MAX = 500.0;     
const UKF_Q_SPD = 0.5;       
const KAPPA = 0.0;           
const MIN_SPD = 0.01;        
const R_ALT_MIN = 1.0;
const MAX_PLAUSIBLE_ACCEL_GPS = 19.62;

// --- FACTEURS DE RÉACTIVITÉ UKF ---
const UKF_REACTIVITY_FACTORS = {
    'AUTO': { MULT: 1.0, DISPLAY: 'Automatique', DESC: 'Ajuste R basé sur la précision GPS.' },
    'NORMAL': { MULT: 1.0, DISPLAY: 'Normal', DESC: 'Correction UKF standard.' },
    'FAST': { MULT: 0.2, DISPLAY: 'Rapide', DESC: 'Fait très confiance au GPS brut.' },
    'STABLE': { MULT: 2.5, DISPLAY: 'Microscopique', DESC: 'Fait peu confiance au GPS (Très stable).' },
};

// --- CONFIGURATION SYSTÈME ---
const MIN_DT = 0.01;        
const MAP_UPDATE_INTERVAL = 3000;
const IMU_UPDATE_RATE_MS = 20; 
const DOM_SLOW_UPDATE_MS = 2000; // Fréquence de rafraîchissement lente
const STANDBY_TIMEOUT_MS = 300000; 

// OPTIONS GPS ASSOUPLIES (TIMEOUT ÉTENDU À 20s)
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 500, timeout: 20000 }, 
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 60000 } 
};

// --- FACTEURS D'ENVIRONNEMENT ---
const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0, DISPLAY: 'Normal' },
    'FOREST': { R_MULT: 2.5, DISPLAY: 'Forêt' },
    'CONCRETE': { R_MULT: 7.0, DISPLAY: 'Grotte/Tunnel' },
    'METAL': { R_MULT: 5.0, DISPLAY: 'Métal/Bâtiment' },
};

// --- DONNÉES CÉLESTES/GRAVITÉ ---
const CELESTIAL_DATA = {
    'EARTH': { G: 9.80665, R: WGS84_A, name: 'Terre' },
    'MOON': { G: 1.62, R: 1737400, name: 'Lune' },
    'MARS': { G: 3.71, R: 3389500, name: 'Mars' },
    'ROTATING': { G: 0.0, R: WGS84_A, name: 'Station Spatiale' }
};
// =================================================================
// BLOC 2/4 : Filtres, Modèles Cinématiques et Quaternion
// =================================================================

// --- CLASSE QUATERNION (Pour gestion de l'IMU) ---
class Quaternion {
    constructor(w = 1, x = 0, y = 0, z = 0) {
        this.w = w; this.x = x; this.y = y; this.z = z;
    }
    // ... (Logique complète de fromAcc et toEuler omise pour la concision)
    static fromAcc(ax, ay, az) { return new Quaternion(1, 0, 0, 0); /* Placeholder */ }
    toEuler() { return { roll: 0, pitch: 0, yaw: 0 }; /* Placeholder */ }
}

// ===========================================
// CLASSE UKF PROFESSIONNELLE (Architecture 21 États)
// ===========================================
class ProfessionalUKF {
    constructor() {
        if (typeof math === 'undefined') {
            // Cette alerte sera déclenchée si math.js n'est pas chargé
            throw new Error("math.js n'est pas chargé. Le filtre UKF 21 états ne peut pas fonctionner.");
        }
        this.N_STATES = 21; 
        this.x = math.zeros(this.N_STATES); 
        this.P = math.diag(Array(this.N_STATES).fill(1e-2)); 
        this.Q = math.diag(Array(this.N_STATES).fill(1e-6));

        this.ALPHA = 1e-3;
        this.BETA = 2;
        this.KAPPA = 0;
        this.lambda = this.ALPHA * this.ALPHA * (this.N_STATES + this.KAPPA) - this.N_STATES;
    }

    /** Étape de PRÉDICTION : Modèle de Mouvement Inertielles (Strapdown) */
    predict(imuData, dt) {
        // Logique simplifiée/placeholder pour la démonstration:
        const ax_corr = imuData.accel[0]; 
        
        let vN = this.x.get([3]) + ax_corr * dt; 
        let vE = this.x.get([4]);
        let vD = this.x.get([5]);
        
        if (Math.abs(vN) < MIN_SPD) vN = 0; // ZUPT 

        this.x.set([3], vN); 
    }

    /** Étape de MISE À JOUR : Correction par le GPS */
    update(gpsData, R_factor) {
        // Logique simplifiée/placeholder pour la démonstration:
        this.x.set([0], gpsData.latitude * D2R);
        this.x.set([1], gpsData.longitude * D2R);
        this.x.set([2], gpsData.altitude);
        
        if (gpsData.speed !== null) {
            const currentSpeed = Math.sqrt(this.x.get([3])**2 + this.x.get([4])**2 + this.x.get([5])**2);
            // Correction douce de la vitesse
            const alpha = 0.2; 
            const newSpeed = currentSpeed * (1 - alpha) + gpsData.speed * alpha;
            this.x.set([3], newSpeed); // Approximation pour la démo
        }
    }
    
    getState() {
        const x_data = this.x.toArray();
        return {
            lat: x_data[0] * R2D,
            lon: x_data[1] * R2D,
            alt: x_data[2],
            vN: x_data[3], vE: x_data[4], vD: x_data[5],
            speed: Math.sqrt(x_data[3]**2 + x_data[4]**2 + x_data[5]**2),
            kUncert: this.P.get([3, 3]) + this.P.get([4, 4]) + this.P.get([5, 5]),
            kAltUncert: this.P.get([2, 2])
        };
    }
}


// --- FONCTIONS DE FILTRAGE ET DE MODÈLE (Altitude, Bruit, etc.) ---

function getKalmanR(accRaw, kAlt, kUncert, env, reactivityMode) {
    let R_gps_base = Math.min(accRaw, 100) ** 2; 
    if (accRaw > 100) { R_gps_base = 100 ** 2 + (accRaw - 100) * 10; }
    
    const env_mult = ENVIRONMENT_FACTORS[env]?.R_MULT || 1.0;
    let reactivity_mult = UKF_REACTIVITY_FACTORS[reactivityMode]?.MULT || 1.0;
    
    // AUTOMATISATION DU FACTEUR R
    if (reactivityMode === 'AUTO' && accRaw !== null) {
        if (accRaw > 20) { 
            reactivity_mult = 3.0; 
        } else if (accRaw < 3) { 
            reactivity_mult = 0.5; 
        }
    }
    
    let R_dyn = Math.min(R_gps_base * env_mult * reactivity_mult, UKF_R_MAX);
    if (kUncert > 100) { R_dyn *= 1.1; }
    
    return Math.max(R_dyn, R_ALT_MIN); 
}

function updateCelestialBody(body, alt, rotationRadius, angularVelocity) {
    let G_ACC_NEW = CELESTIAL_DATA['EARTH'].G;
    let R_ALT_CENTER_REF_NEW = WGS84_A; 
    const data = CELESTIAL_DATA[body];
    if (data) {
        G_ACC_NEW = data.G;
        R_ALT_CENTER_REF_NEW = data.R;
    }
    G_ACC = G_ACC_NEW;
    R_ALT_CENTER_REF = R_ALT_CENTER_REF_NEW;
    return { G_ACC: G_ACC_NEW, R_ALT_CENTER_REF: R_ALT_CENTER_REF_NEW };
            }
// =================================================================
// BLOC 3/4 : Services Externes & Calculs Astro/Physique (CORRIGÉ PROXY)
// =================================================================

// CORRECTION CRITIQUE DE L'URL DU PROXY
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app"; 
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

const J1970 = 2440588, J2000 = 2451545.0;
const dayMs = 1000 * 60 * 60 * 24;
const MC_DAY_MS = 72 * 60 * 1000; 

async function syncH(lServH_in, lLocH_in) {
    let lServH = lServH_in;
    let lLocH = lLocH_in;
    
    if ($('local-time')) $('local-time').textContent = 'Synchronisation...';
    
    try {
        const response = await fetch(SERVER_TIME_ENDPOINT, { cache: "no-store", mode: "cors" });
        if (!response.ok) throw new Error(`Server time sync failed: ${response.statusText}`);
        
        const serverData = await response.json(); 
        const utcTimeISO = serverData.utc_datetime; 
        lServH = Date.parse(utcTimeISO); 
        lLocH = performance.now(); 
        
        if ($('local-time')) $('local-time').textContent = '✅ SYNCHRO NTP ACTIVE';

    } catch (error) {
        console.warn("Échec de la synchronisation. Utilisation de l'horloge locale.", error);
        lServH = Date.now(); 
        lLocH = performance.now();
        if ($('local-time')) $('local-time').textContent = '❌ SYNCHRO ÉCHOUÉE (Local)';
    }
    return { lServH, lLocH };
}

function getCDate(lServH, lLocH) {
    if (lServH === null || lLocH === null) { return new Date(); }
    const offset = performance.now() - lLocH;
    return new Date(lServH + offset);
}

function getSpeedOfSound(tempK) {
    return Math.sqrt(GAMMA * R_AIR * tempK);
}

async function fetchWeather(lat, lon) {
    if (lat === null || lon === null) {
         if ($('weather-status')) $('weather-status').textContent = 'En attente de coordonnées GPS...';
         return null;
    }
    const url = `${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`;
    try {
        const response = await fetch(url);
        if (!response.ok) throw new Error(`Erreur HTTP ${response.status} de la source proxy.`);
        const data = await response.json();

        if (data.main) {
            const P_hPa = data.main.pressure;
            const T_C = data.main.temp;
            const T_K = T_C + 273.15;
            const H_perc = data.main.humidity;
            const P_Pa = P_hPa * 100; 
            const air_density = P_Pa / (R_AIR * T_K);
            
            if ($('weather-status')) $('weather-status').textContent = 'Météo Actif';

            // ... (Logique de calcul du point de rosée et renvoi des données)
            return {
                pressure_hPa: P_hPa,
                tempC: T_C,
                tempK: T_K,
                humidity_perc: H_perc,
                air_density: air_density,
                dew_point: 0.0 // Placeholder pour la concision
            };
        }
    } catch (e) {
        console.warn("Erreur de récupération météo:", e.message);
        if ($('weather-status')) $('weather-status').textContent = `❌ ${e.message}`;
        throw e; 
    }
    return null;
}

function getMinecraftTime(date) {
    // ... (Logique complète de calcul de l'heure Minecraft omise pour la concision)
    return '00:00'; 
}

function updateAstro(lat, lon, lServH, lLocH) {
    // ... (Logique complète Astro omise pour la concision)
}
// =================================================================
// BLOC 4/4 : Logique Applicative Principale (Core Loop & DOM Init) (FINAL AVEC DÉBOGAGE ET SIMULATION)
// =================================================================

// --- VARIABLES D'ÉTAT (Globales) ---
let wID = null, domSlowID = null, domFastID = null, lPos = null, lat = null, lon = null, sTime = null;
let distM = 0, maxSpd = 0;
let kSpd = 0, kUncert = UKF_R_MAX, kAltUncert = 10; 
let timeMoving = 0, timeTotal = 0; 
let lastFSpeed = 0; 
let kAlt = null;      
let ukf = null; 

let currentGPSMode = 'HIGH_FREQ'; 
let emergencyStopActive = false; 
let selectedEnvironment = 'NORMAL'; 
let currentMass = 70.0; 
let currentCelestialBody = 'EARTH';
let gpsAccuracyOverride = 0.0; 
let lastGPSPos = null;

let lServH = null, lLocH = null; // Variables pour la synchro NTP

let lastP_hPa = BARO_ALT_REF_HPA, lastT_K = 288.15, currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound = 343;

let accel = { x: 0, y: 0, z: 0 };
let gyro = { x: 0, y: 0, z: 0 };
let mag = { x: 0, y: 0, z: 0 };
let lastIMUTimestamp = 0;
let lastMapUpdate = 0;

let currentUKFReactivity = 'AUTO'; 
let gpsStandbyTimeoutID = null;    
let simulationMode = false; 

// --- GESTION DES CAPTEURS (IMU) ---
function startIMUListeners() {
    if (emergencyStopActive) return;
    try {
        if ($('imu-status')) $('imu-status').textContent = "Activation...";
        
        if (typeof Accelerometer === 'undefined' || typeof Gyroscope === 'undefined') {
             throw new Error("API Sensor non supportée (Accéléromètre/Gyroscope manquant).");
        }
        
        const accSensor = new Accelerometer({ frequency: 50 }); 
        accSensor.addEventListener('reading', () => {
            accel.x = accSensor.x;
            accel.y = accSensor.y;
            accel.z = accSensor.z;
        });
        accSensor.addEventListener('error', event => {
            if ($('imu-status')) $('imu-status').textContent = `Erreur Accél: ${event.error.name}`;
            console.error("Erreur Accéléromètre:", event.error);
        });
        accSensor.start();

        const gyroSensor = new Gyroscope({ frequency: 50 });
        gyroSensor.addEventListener('reading', () => {
            gyro.x = gyroSensor.x;
            gyro.y = gyroSensor.y;
            gyro.z = gyroSensor.z;
        });
        gyroSensor.addEventListener('error', event => {
             if ($('imu-status')) $('imu-status').textContent = `Erreur Gyro: ${event.error.name}`;
             console.error("Erreur Gyroscope:", event.error);
        });
        gyroSensor.start();
        
        if ($('imu-status')) $('imu-status').textContent = "Actif (API Sensor)";
        lastIMUTimestamp = performance.now();
        
        startFastLoop();

    } catch (error) {
        let errMsg = error.message;

        if (error.name === 'SecurityError' || error.name === 'NotAllowedError') {
            // ERREUR CLÉ : Permission refusée par manque de clic explicite
            errMsg = "Permission Capteurs Refusée. Cliquez explicitement sur le bouton de simulation/arrêt.";
        } else if (error.name === 'NotReadableError') {
             errMsg = "Capteurs Verrouillés par l'OS (Mode économie ou usage externe).";
        }

        if ($('imu-status')) $('imu-status').textContent = `❌ ${errMsg}`;
        console.error("ERREUR CRITIQUE IMU:", error.name, errMsg);
    }
}

function stopIMUListeners() {
    // ... (Logique d'arrêt de l'IMU omise pour la concision)
}

// --- GESTION GPS (DÉBOGAGE GPS AMÉLIORÉ) ---
function startGPS(mode = currentGPSMode) {
    if (emergencyStopActive) return;
    if (wID !== null) { navigator.geolocation.clearWatch(wID); wID = null; }
    
    currentGPSMode = mode;
    const options = GPS_OPTS[mode];
    
    // Tentative de démarrage du GPS
    wID = navigator.geolocation.watchPosition(gpsUpdateCallback, handleErr, options);
    
    if ($('gps-precision')) $('gps-precision').textContent = "Attente du signal GPS...";

    // ... (Logique de mise à jour du bouton omise pour la concision)
}

function stopGPS(resetButton = true) {
    if (wID !== null) { navigator.geolocation.clearWatch(wID); wID = null; }
    // ... (Logique d'arrêt du timer et de mise à jour du bouton omise)
}

function toggleGPS() {
    if (emergencyStopActive) { alert("Système en arrêt d'urgence."); return; }
    wID === null ? startGPS('HIGH_FREQ') : stopGPS();
}

// GESTION DES ERREURS GPS (AFFICHAGE CLAIR DE LA CAUSE)
function handleErr(err) {
    let errMsg = `Erreur GPS (Code ${err.code}): `;

    switch (err.code) {
        case 1:
            errMsg += "Permission refusée. Vérifiez les réglages de confidentialité.";
            stopGPS(); 
            break;
        case 2:
            errMsg += "Position indisponible (Mauvais signal ou GPS désactivé).";
            break;
        case 3:
            errMsg += "Timeout (20s) : Le fix GPS est trop lent. Réessayez.";
            break;
        default:
            errMsg += `Erreur inconnue: ${err.message}`;
    }

    if ($('gps-precision')) $('gps-precision').textContent = `❌ ${errMsg}`;
    console.error("ERREUR CRITIQUE GPS:", errMsg);
}

// GESTION DE LA MISE À JOUR GPS (LIAISON UKF)
function gpsUpdateCallback(pos) {
    lastGPSPos = pos;
    const { latitude, longitude, altitude, speed, accuracy } = pos.coords;
    
    // Mettre à jour l'UKF avec la mesure GPS
    if (ukf) {
        ukf.update({
            latitude: latitude,
            longitude: longitude,
            altitude: altitude || 0,
            speed: speed || null, 
            accuracy: accuracy
        }, kUncert); // Utiliser l'incertitude UKF pour déterminer R
    }
    
    // ... (Logique de mise à jour du DOM GPS/Carte/Statut omise pour la concision)
}

// --- FONCTION DE SIMULATION (Point d'entrée pour les permissions) ---
function toggleSimulation() {
    simulationMode = !simulationMode;
    const btn = $('toggle-sim-btn');
    const imuStatus = $('imu-status');

    if (simulationMode) {
        stopGPS(false); 
        stopIMUListeners();

        if (btn) {
            btn.textContent = "⏹️ SIMULATION ACTIVE";
            btn.style.backgroundColor = '#dc3545';
        }
        if (imuStatus) imuStatus.textContent = "MODE SIMULATION";

        // Injection d'une position fake pour démarrer les calculs astro/météo
        lat = 43.296; 
        lon = 5.37;
        kAlt = 0;
        lastGPSPos = { coords: { latitude: lat, longitude: lon, speed: 0, heading: 0, accuracy: 5 } };
        
        // Assurer que la boucle rapide démarre
        if (!domFastID) startFastLoop(); 

    } else {
        // --- Désactivation : Tenter d'activer les vrais capteurs (via CLIC UTILISATEUR) ---
        if (btn) {
            btn.textContent = "▶️ DÉMARRER SIMULATION IMU";
            btn.style.backgroundColor = '#007bff';
        }
        
        // C'est ici que l'interaction utilisateur débloque les capteurs
        startIMUListeners(); 
        startGPS('HIGH_FREQ'); 
    }
}

/**
 * BOUCLE RAPIDE (IMU) - Prédiction UKF et Affichage
 */
function startFastLoop() {
    if (domFastID) return; 
    
    domFastID = setInterval(() => {
        if (emergencyStopActive || !ukf) return;
        
        const now = performance.now();
        const dt = (now - lastIMUTimestamp) / 1000.0;
        if (dt < MIN_DT) return; 
        lastIMUTimestamp = now;
        
        // Logique de simulation (si active)
        let simAccel = { x: accel.x, y: accel.y, z: accel.z };
        if (simulationMode) {
             const time = (now - sTime) / 1000.0;
             simAccel.x = 2.0 * Math.sin(time / 5); 
             simAccel.z = G_ACC; 
             // ... (Mise à jour des affichages SIM)
        } 
        
        // 1. PRÉDICTION UKF
        const imuReadings = {
            accel: [simAccel.x, simAccel.y, simAccel.z],
            gyro: [gyro.x, gyro.y, gyro.z] // Utilise le vrai gyro ou 0
        };
        ukf.predict(imuReadings, dt);

        // 2. MISE À JOUR DE LA VITESSE (Calculs de forces et d'affichage)
        const state = ukf.getState();
        kSpd = state.speed;
        kUncert = state.kUncert;
        kAltUncert = state.kAltUncert;
        // ... (Reste des calculs et de l'affichage rapide)

    }, IMU_UPDATE_RATE_MS);
}


// ... (Fonctions initMap, updateMap et autres utilitaires omises pour la concision)


// ===========================================
// INITIALISATION DOM (DÉMARRAGE) 
// ===========================================
document.addEventListener('DOMContentLoaded', () => {
    
    // ... (Initialisation de la carte et du filtre UKF)
    
    // --- Initialisation des contrôles (Liée aux clics) ---
    if ($('toggle-sim-btn')) $('toggle-sim-btn').addEventListener('click', toggleSimulation);
    if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', toggleGPS);
    // ... (Autres écouteurs omis pour la concision)

    // --- DÉMARRAGE DU SYSTÈME (Sync NTP) ---
    updateCelestialBody(currentCelestialBody, kAlt, 0, 0); // Initialise la gravité
    
    syncH(lServH, lLocH).then(newTimes => {
        lServH = newTimes.lServH;
        lLocH = newTimes.lLocH;
    }).finally(() => { 
        sTime = Date.now();
        // ATTENTION : Le GPS et l'IMU ne DÉMARRENT PAS ici. Ils attendent le clic sur le bouton de simulation/arrêt.

        // Démarrage de la boucle lente (Astro/Météo)
        // ... (Logique de la boucle lente omise)
    });
});
