// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// BLOC 1/5 : Constantes, Utilitaires, État Global, et Classe TQ
// =ignée pour index_complet.html
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
        // Assure que le format '0.00e+0' est respecté pour le nombre de décimales
        const zeroDecimals = '0.' + Array(decimals).fill('0').join('');
        return zeroDecimals + 'e+0' + suffix;
    }
    return val.toExponential(decimals) + suffix;
};

// --- CONSTANTES GÉNÉRALES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI; 
const KMH_MS = 3.6;          
const DOM_SLOW_UPDATE_MS = 2000; 

// --- CLÉS D'API & ENDPOINTS (Exemple) ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const PROXY_CELESTIAL_ENDPOINT = `${PROXY_BASE_URL}/api/celestial-ephem`; 
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES FONDAMENTALES ---
const C_L = 299792458;          // Vitesse de la lumière (m/s)
const AU_TO_METERS = 149597870700; // 1 Unité Astronomique en mètres
const R_E_BASE = 6371000;       // Rayon terrestre moyen (m)
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const G_EARTH = 9.80665;        // Gravité standard (m/s²)
const R_AIR = 287.058;          // Constante spécifique de l'air sec (J/kg·K)
const BARO_ALT_REF_HPA = 1013.25; // Pression au niveau de la mer (hPa)
const TEMP_SEA_LEVEL_K = 288.15;  // Température au niveau de la mer (15°C)
const RHO_SEA_LEVEL = 1.225;      // Densité de l'air au niveau de la mer (kg/m³)

// --- CONSTANTES DE CACHE ET CORPS CÉLESTES ---
const CELESTIAL_CACHE_KEY = 'celestialEphemerisData';
const CELESTIAL_CACHE_DURATION_MS = 24 * 60 * 60 * 1000; // 1 jour de validité
const CELESTIAL_STEP_SIZE = '1 s'; 
const REQUIRED_CELESTIAL_BODIES = ['SUN', 'MOON', 'MARS']; 
const WEATHER_CACHE_KEY = 'weatherForecastData';
const WEATHER_CACHE_DURATION_MS = 12 * 60 * 60 * 1000; // 12 heures de validité

// --- ÉTAT GLOBAL ET CACHES ---
let isGpsPaused = false;
let currentPosition = { lat: 43.2964, lon: 5.3697, acc: 10.0, spd: 0.0, alt: 0.0 }; // Marseille par défaut
let currentIMU = { acc_x: 0, acc_y: 0, acc_z: G_EARTH, gyro_roll: 0, gyro_pitch: 0, gyro_yaw: 0 };
let currentMass = 70.0;
let kAlt = 0; // Altitude du GPS/EKF
let currentCelestialBody = 'EARTH';
let rotationRadius = 100;
let angularVelocity = 0.0;
let distanceRatioMode = false;
let currentUKFReactivity = 'NORMAL';

let lastT_K = TEMP_SEA_LEVEL_K;
let lastP_hPa = BARO_ALT_REF_HPA;
let currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound; 
let lServH = 0; // Heure serveur pour synchro NTP
let lLocH = 0;  // Heure locale de la synchro NTP

// --- UKF/EKF DUMMY OBJECTS (Simulent l'état du filtre) ---
const kf_alt = { x: [0], P: [[100]] }; 
const kf_vel = { x: [0], P: [[100]] };

// --- DONNÉES DE SECOURS (Mars) ---
const CELESTIAL_EPHEM_MARS_FALLBACK = [
    [new Date('2025-11-28T00:00:00Z').getTime(), 2.42379158261092, 255.50120833, -23.408167],
    [new Date('2025-11-29T00:00:00Z').getTime(), 2.42385075850788, 256.29791667, -23.488194]
]; 

let celestialEphemerisCache = {
    'MARS': { 
        expires: 0, 
        data: typeof CELESTIAL_EPHEM_MARS_FALLBACK !== 'undefined' ? CELESTIAL_EPHEM_MARS_FALLBACK : []
    },
    'SUN': { expires: 0, data: [] },
    'MOON': { expires: 0, data: [] }
};
let weatherForecastCache = { expires: 0, data: [] }; 

const ENVIRONMENT_FACTORS = {
    EARTH: { G: G_EARTH, DISPLAY: 'Terre (1.00 G)', MULT: 1.0 },
    MOON: { G: 1.625, DISPLAY: 'Lune (0.16 G)', MULT: 0.165 },
    MARS: { G: 3.72, DISPLAY: 'Mars (0.38 G)', MULT: 0.38 },
    ROTATING: { G: 9.80665, DISPLAY: 'Centrifuge (Défaut)', MULT: 1.0 }
};

// --- CLASSE TheoreticalQuantities (TQ) ---
class TheoreticalQuantities {
    constructor() {
        this.q = {
            'gravity-base': { name: 'Gravité locale (Base)', value: G_EARTH, unit: 'm/s²', critical: true },
            'gravity-centrifugal': { name: 'Force centrifuge (Estimée)', value: 0, unit: 'm/s²' },
            'speed-of-sound-calc': { name: 'Vitesse du Son (ISA)', value: 340.29, unit: 'm/s' }, // Valeur ISA par défaut
            'air-density': { name: 'Densité de l\'air', value: RHO_SEA_LEVEL, unit: 'kg/m³' },
            'current-alt': { name: 'Altitude (GPS/EKF)', value: 0, unit: 'm', critical: true },
            'time-dilation': { name: 'Dilatation du Temps (rel.)', value: 1.0000000000, unit: '' }
        };
    }
    // La mise à jour du DOM est gérée par la boucle d'intervalle dans le BLOC 5.
    set(id, value) {
        if (this.q[id]) {
            this.q[id].value = value;
        }
    }
    // Le contenu de populateDOM est défini dans le script de l'HTML
    populateDOM() {} 
}
const TQ = new TheoreticalQuantities();
// =================================================================
// BLOC 2/5 : Modèles Physiques & Fonctions d'Interpolation UKF
// =================================================================

/**
 * Calcule la densité de l'air (kg/m³) en utilisant la formule des gaz parfaits.
 */
function getAirDensity(P_hPa, T_K) {
    // P_Pa = P_hPa * 100
    // rho = P_Pa / (R_AIR * T_K)
    return (P_hPa * 100) / (R_AIR * T_K); 
}

/**
 * Calcule la vitesse du son (m/s) en fonction de la température (K).
 */
function getSpeedOfSound(T_K) {
    const GAMMA = 1.4; 
    return Math.sqrt(GAMMA * R_AIR * T_K);
}

/**
 * Convertit les coordonnées sphériques (Distance, RA, DEC) en Cartésiennes (X, Y, Z).
 */
function sphericalToCartesian(dist, raDeg, decDeg) {
    const raRad = raDeg * D2R; 
    const decRad = decDeg * D2R;

    const x = dist * Math.cos(decRad) * Math.cos(raRad);
    const y = dist * Math.cos(decRad) * Math.sin(raRad);
    const z = dist * Math.sin(decRad);

    return { x, y, z };
}

/**
 * Obtient le vecteur de position d'un astre à un instant donné par interpolation linéaire (1s).
 */
function getCelestialPositionAt(bodyId, date) {
    const t = date.getTime();
    const bodyCache = celestialEphemerisCache[bodyId];

    if (!bodyCache || bodyCache.data.length < 2) {
        return { x: 0, y: 0, z: 0 }; 
    }
    const ephemerisData = bodyCache.data;

    let p1 = null;
    let p2 = null;
    
    for (let i = 0; i < ephemerisData.length - 1; i++) {
        if (t >= ephemerisData[i][0] && t <= ephemerisData[i + 1][0]) {
            p1 = ephemerisData[i];
            p2 = ephemerisData[i + 1];
            break;
        }
    }
    
    if (!p1 || !p2) {
        p1 = p2 = ephemerisData[ephemerisData.length - 1]; 
    }

    const t1 = p1[0];
    const t2 = p2[0];
    const dt = (t - t1) / (t2 - t1); 
    
    const deltaAU = p1[1] + (p2[1] - p1[1]) * dt;
    const raDeg = p1[2] + (p2[2] - p1[2]) * dt;
    const decDeg = p1[3] + (p2[3] - p1[3]) * dt;

    const distMeters = deltaAU * AU_TO_METERS;

    return sphericalToCartesian(distMeters, raDeg, decDeg);
}

/**
 * Obtient les valeurs météorologiques (Pression, Température) par interpolation linéaire (1h).
 */
function getUKFWeatherAt(date) {
    const t = date.getTime();
    const forecast = weatherForecastCache.data;

    if (!forecast || forecast.length < 2) {
        return { P_hPa: BARO_ALT_REF_HPA, T_K: TEMP_SEA_LEVEL_K, H_perc: 0.5 };
    }
    
    let p1 = null;
    let p2 = null;

    for (let i = 0; i < forecast.length - 1; i++) {
        // [0]=timestamp, [1]=P, [2]=T, [3]=H
        if (t >= forecast[i][0] && t <= forecast[i + 1][0]) {
            p1 = forecast[i];
            p2 = forecast[i + 1];
            break;
        }
    }
    
    if (!p1 || !p2) {
        p1 = p2 = forecast[forecast.length - 1]; 
    }

    const t1 = p1[0];
    const t2 = p2[0];
    const dt = (t - t1) / (t2 - t1); 
    
    const P_hPa = p1[1] + (p2[1] - p1[1]) * dt;
    const T_K = p1[2] + (p2[2] - p1[2]) * dt;
    const H_perc = p1[3] + (p2[3] - p1[3]) * dt;

    return { P_hPa, T_K, H_perc };
}
// =================================================================
// BLOC 3/5 : Logique de Caching et Initialisation des Données Hors Ligne
// =================================================================

async function fetchAndCacheCelestialEphemeris(bodyId) {
    if (!navigator.onLine) return;

    try {
        const startTime = new Date().toISOString().split('T')[0];
        const stopDate = new Date(Date.now() + CELESTIAL_CACHE_DURATION_MS);
        const stopTime = stopDate.toISOString().split('T')[0];
        
        const url = `${PROXY_CELESTIAL_ENDPOINT}?body=${bodyId}&start=${startTime}&stop=${stopTime}&step=${CELESTIAL_STEP_SIZE}`;

        const response = await fetch(url);
        if (!response.ok) throw new Error(`Erreur HTTP: ${response.status}`);
        
        const data = await response.json();
        
        if (data && data.ephemeris && data.ephemeris.length > 0) {
            const expirationTime = Date.now() + CELESTIAL_CACHE_DURATION_MS;
            celestialEphemerisCache[bodyId] = {
                expires: expirationTime,
                data: data.ephemeris 
            };
            
            // ATTENTION: Risque de QuotaExceededError pour ~28 Mo.
            localStorage.setItem(CELESTIAL_CACHE_KEY, JSON.stringify(celestialEphemerisCache));
            console.log(`✅ Ephémérides ${bodyId} mises en cache (1s/1j).`);
        } else {
            throw new Error("Données d'éphémérides invalides.");
        }
        
    } catch (error) {
        console.error(`❌ Erreur de récupération/caching des éphémérides ${bodyId}.`, error);
    }
}

async function fetchAndCacheWeatherForecast() {
    if (!navigator.onLine) return;

    try {
        const { lat, lon } = currentPosition;
        if (lat === 0 && lon === 0) return;
        
        const url = `${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}&duration=24h&step=1h`;

        const response = await fetch(url);
        if (!response.ok) throw new Error(`Erreur HTTP: ${response.status}`);
        
        const data = await response.json();
        
        if (data && data.forecast && data.forecast.length > 0) {
            const expirationTime = Date.now() + WEATHER_CACHE_DURATION_MS;
            
            weatherForecastCache = {
                expires: expirationTime,
                data: data.forecast 
            };
            
            localStorage.setItem(WEATHER_CACHE_KEY, JSON.stringify(weatherForecastCache));
            console.log(`✅ Prévisions météo mises en cache (1h/24h).`);
        } else {
            throw new Error("Données de prévisions invalides reçues.");
        }
        
    } catch (error) {
        console.error("❌ Erreur de récupération/caching des prévisions météo :", error);
    }
}

function initializeCelestialData() {
    const cachedData = localStorage.getItem(CELESTIAL_CACHE_KEY);
    if (cachedData) {
        try {
            const parsedCache = JSON.parse(cachedData);
            // Fusionne le cache avec les données de secours pour MARS (si nécessaire)
            celestialEphemerisCache = { ...celestialEphemerisCache, ...parsedCache };
        } catch (e) {
            console.error("Erreur de parsing du cache d'éphémérides. Cache effacé.", e);
            localStorage.removeItem(CELESTIAL_CACHE_KEY);
        }
    }

    REQUIRED_CELESTIAL_BODIES.forEach(bodyId => {
        const bodyCache = celestialEphemerisCache[bodyId];
        const isExpired = !bodyCache || bodyCache.expires < Date.now();
        
        if (isExpired) {
            fetchAndCacheCelestialEphemeris(bodyId);
        }
    });
}

function initializeWeatherCache() {
    const cachedData = localStorage.getItem(WEATHER_CACHE_KEY);
    if (cachedData) {
        try {
            weatherForecastCache = JSON.parse(cachedData);
        } catch (e) {
            console.error("Erreur de parsing du cache météo.", e);
            localStorage.removeItem(WEATHER_CACHE_KEY);
        }
    }
    const isExpired = weatherForecastCache.expires < Date.now();
    
    // Ne lance le fetch que si la position par défaut a été mise à jour par le GPS
    if (isExpired && (currentPosition.lat !== 43.2964 || currentPosition.lon !== 5.3697)) {
        fetchAndCacheWeatherForecast();
    }
            }
// =================================================================
// BLOC 4/5 : Fonctions de Mise à Jour UKF/DOM
// =================================================================

/**
 * Obtient l'heure synchronisée (NTP ou locale par défaut). (ASSUMÉE)
 */
function getCDate(serverTimeMs, localTimeMs) {
    if (serverTimeMs > 0 && localTimeMs > 0) {
        const offset = Date.now() - localTimeMs;
        return new Date(serverTimeMs + offset);
    }
    return new Date();
}

/**
 * [UKF ENTRY POINT] Met à jour les facteurs environnementaux pour le filtre.
 */
function updateUKFEnvironmentalFactors() {
    const now = getCDate(lServH, lLocH); 
    
    // 1. Mise à jour Météo (Interpolation à la seconde)
    const weather = getUKFWeatherAt(now);
    lastP_hPa = weather.P_hPa;
    lastT_K = weather.T_K;
    
    // Mise à jour des variables physiques
    currentAirDensity = getAirDensity(lastP_hPa, lastT_K); 
    currentSpeedOfSound = getSpeedOfSound(lastT_K); 
    
    // Mise à jour des quantités théoriques (TQ)
    TQ.set('speed-of-sound-calc', currentSpeedOfSound);
    TQ.set('air-density', currentAirDensity);

    // 2. Récupération des Vecteurs Célestes (Interpolation à la seconde)
    const sunPos = getCelestialPositionAt('SUN', now);
    const moonPos = getCelestialPositionAt('MOON', now);
    const marsPos = getCelestialPositionAt('MARS', now);
    
    // 3. Mise à jour du DOM Météo/Air (Affichage de la donnée interpolée)
    if ($('temp-air-2')) $('temp-air-2').textContent = `${(lastT_K - 273.15).toFixed(1)} °C`;
    if ($('pressure-2')) $('pressure-2').textContent = `${lastP_hPa.toFixed(0)} hPa`;
    
    // 4. Mise à jour du DOM Heure
    if (now) {
        if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString('fr-FR');
        if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
    }
}

/**
 * Met à jour la gravité de base et les facteurs environnementaux selon le corps céleste.
 */
function updateCelestialBody(body, alt, rotRadius, rotVel) {
    const factor = ENVIRONMENT_FACTORS[body];
    const G_BASE = factor.G;
    let G_ACC_NEW = G_BASE;

    if (body === 'ROTATING') {
        const a_cent = rotRadius * rotVel * rotVel;
        G_ACC_NEW = G_BASE + a_cent; 
        TQ.set('gravity-centrifugal', a_cent);
    } else {
        TQ.set('gravity-centrifugal', 0);
    }

    TQ.set('gravity-base', G_ACC_NEW);
    if ($('env-factor')) $('env-factor').textContent = `${factor.DISPLAY} (x${factor.MULT.toFixed(1)})`;
    return { G_ACC_NEW };
}

/**
 * Fonction fictive de l'UKF (Unscented Kalman Filter)
 * (Le code mathématique complet du filtre est omis)
 */
function runUKFStep(dt) {
    // X(k+1) = f(X(k), U(k), V(k))
    // U(k) est l'entrée IMU (acc, gyro), V(k) est le bruit.
    // L'UKF calcule 21 états (position, vitesse, attitude, biais capteur, etc.)
}

/**
 * Calcule le rapport de distance pour le mode "ALTITUDE"
 */
function calculateDistanceRatio(altMeters) {
    // Calcul de l'effet relativiste ou autre correction basée sur l'altitude
    // Exemple simple: 1 - (2*G_EARTH*altMeters / C_L^2) 
    return 1.0 + (altMeters / R_E_BASE); 
}

// --- DUMMY FUNCTIONS ---
function syncH() {
    lServH = Date.now();
    lLocH = Date.now();
}

function updateMap() {
    // Logique de mise à jour de la carte Leaflet
    if ($('lat-val')) $('lat-val').textContent = dataOrDefault(currentPosition.lat, 6);
    if ($('lon-val')) $('lon-val').textContent = dataOrDefault(currentPosition.lon, 6);
    if ($('alt-val')) $('alt-val').textContent = dataOrDefault(currentPosition.alt, 2);
    if ($('acc-val')) $('acc-val').textContent = dataOrDefault(currentPosition.acc, 1);
    if ($('speed-val')) $('speed-val').textContent = dataOrDefault(currentPosition.spd, 2);
    
    TQ.set('current-alt', currentPosition.alt);
}
// =================================================================
// BLOC 5/5 : DÉMARRAGE : Encapsulation de la logique UKF et État Global (IIFE)
// =================================================================
((window) => {
    
    // --- VÉRIFICATION DES DÉPENDANCES CRITIQUES ---
    if (typeof math === 'undefined' || typeof L === 'undefined' || typeof SunCalc === 'undefined' || typeof turf === 'undefined') {
        const missing = [
            (typeof math === 'undefined' ? "math.min.js" : ""),
            (typeof L === 'undefined' ? "leaflet.js" : ""),
            (typeof SunCalc === 'undefined' ? "suncalc.js" : ""),
            (typeof turf === 'undefined' ? "turf.min.js" : "")
        ].filter(Boolean).join(', ');
        if (missing) {
             console.error(`DÉPENDANCES CRITIQUES MANQUANTES: ${missing}. Veuillez vous assurer qu'elles sont chargées dans l'HTML.`);
        }
    }

    // --- LOGIQUE DE GESTION DES INPUTS ET ÉVÉNEMENTS ---

    $('mass-input').addEventListener('input', (e) => {
        currentMass = parseFloat(e.target.value) || 70.0;
        $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
    });
    
    $('celestial-body-select').addEventListener('change', (e) => {
        currentCelestialBody = e.target.value;
        const { G_ACC_NEW } = updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity);
        $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
    });
    
    const updateRotation = () => {
        rotationRadius = parseFloat($('rotation-radius').value) || 100;
        angularVelocity = parseFloat($('angular-velocity').value) || 0.0;
        if (currentCelestialBody === 'ROTATING') {
            const { G_ACC_NEW } = updateCelestialBody('ROTATING', kAlt, rotationRadius, angularVelocity);
            $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
        }
    };
    $('rotation-radius').addEventListener('input', updateRotation);
    $('angular-velocity').addEventListener('input', updateRotation);
    
    $('distance-ratio-toggle-btn').addEventListener('click', () => {
        distanceRatioMode = !distanceRatioMode;
        const ratio = distanceRatioMode ? calculateDistanceRatio(kAlt || 0) : 1.0;
        $('distance-ratio-toggle-btn').textContent = `Rapport Distance: ${distanceRatioMode ? 'ALTITUDE' : 'SURFACE'} (${ratio.toExponential(4)})`;
    });
    
    $('ukf-reactivity-mode').addEventListener('change', (e) => currentUKFReactivity = e.target.value);
    
    // --- GESTION DU CAPTEUR GPS (REAL) ---
    function handleGpsUpdate(position) {
        if (isGpsPaused) return;
        
        const newLat = position.coords.latitude;
        const newLon = position.coords.longitude;
        const newAlt = position.coords.altitude || 0.0;
        const newAcc = position.coords.accuracy || 10.0;
        const newSpd = position.coords.speed || 0.0;

        currentPosition = { lat: newLat, lon: newLon, acc: newAcc, spd: newSpd, alt: newAlt };
        kAlt = newAlt;
        
        initializeCelestialData(); 
        initializeWeatherCache();  
        
        // runUKFStep(dt) 
        updateMap(); // Met à jour l'affichage GPS/EKF
    }
    
    if (navigator.geolocation) {
        navigator.geolocation.watchPosition(handleGpsUpdate, (error) => {
            console.error("Erreur GPS:", error.message);
        }, { enableHighAccuracy: true, maximumAge: 0, timeout: 5000 });
    }

    // --- DÉMARRAGE DU SYSTÈME ---
    
    // Démarrer la synchro NTP
    syncH(); 
    
    // Initialiser le cache Astro/Météo
    initializeCelestialData();
    initializeWeatherCache(); 
    
    // Initialiser les valeurs de physique
    currentAirDensity = getAirDensity(lastP_hPa, lastT_K); 
    currentSpeedOfSound = getSpeedOfSound(lastT_K); 
    
    // Mise à jour initiale de l'affichage
    const { G_ACC_NEW } = updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity);
    if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
    
    // Boucle de mise à jour haute fréquence (10 Hz)
    setInterval(() => {
        // Mise à jour des facteurs environnementaux (utilise l'interpolation 1s)
        updateUKFEnvironmentalFactors(); 
        // Mise à jour des affichages des quantités théoriques
        TQ.populateDOM(); 
        
    }, 100); 

})(window);
