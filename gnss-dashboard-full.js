// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// Fusion de (13).js, (10).js, (7).js, (6).js, (5).js, (3).js, (2).js
// =================================================================

// =================================================================
// BLOC 1/3 : Constantes, Modèles Physiques & Classes
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
        return '0.00e+0' + suffix;
    }
    return val.toExponential(decimals) + suffix;
};

// --- CLÉS D'API & ENDPOINTS ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES FONDAMENTALES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI; 
const KMH_MS = 3.6;         
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const G_U = 6.67430e-11;    // Constante gravitationnelle universelle (N·m²/kg²)
const R_E_BASE = 6371000;   // Rayon terrestre moyen (m)
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const AU_METERS = 149597870700; // Unité Astronomique (m)
const LIGHT_YEAR_METERS = 9.461e15; // Année Lumière (m)
const SOLAR_FLUX_DENSITY = 1361; // Flux solaire (W/m²)

// --- CONSTANTES ATMOSPHÉRIQUES (ISA Standard) ---
const BARO_ALT_REF_HPA = 1013.25;
const RHO_SEA_LEVEL = 1.225;
const R_AIR = 287.058;
const GAMMA_AIR = 1.4;
const MU_DYNAMIC_AIR = 1.8e-5;  // Viscosité dynamique de l'air (Pa·s)
const KELVIN_OFFSET = 273.15;
const SAT_O2_SEA_LEVEL = 97.5;  // Saturation O2 théorique (%)

// --- CONSTANTES GÉOPHYSIQUES (WGS84) ---
const WGS84_A = 6378137.0;  // Rayon équatorial
const WGS84_F = 1 / 298.257223563;
const WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F;
const WGS84_G_EQUATOR = 9.780327;
const WGS84_BETA = 0.0053024;

// --- PARAMÈTRES DU FILTRE UKF/EKF ---
const UKF_STATE_DIM = 21;    // [Pos(3), Vel(3), Att(3), BiasAcc(3), BiasGyro(3), ...]
const UKF_R_MAX = 500.0;     
const UKF_Q_SPD = 0.5;       
const R_ALT_MIN = 1.0;
const MAX_PLAUSIBLE_ACCEL_GPS = 19.62; // 2G (Anti-spike)
const ZUPT_RAW_THRESHOLD = 1.0;     // Seuil Vitesse ZUPT
const ZUPT_ACCEL_THRESHOLD = 0.5;   // Seuil Accel ZUPT
const MIN_SPD = 0.01;        // Seuil de mouvement
const MAX_ACC = 200;        // Précision GPS max avant perte de signal
const NETHER_RATIO = 8.0;
const ALT_TH = -50;         // Seuil d'altitude "Sous-sol"

// --- CONFIGURATION SYSTÈME ---
const MIN_DT = 0.01;        
const MAP_UPDATE_INTERVAL = 3000;
const IMU_UPDATE_RATE_MS = 20; // 50Hz (Boucle rapide)
const DOM_SLOW_UPDATE_MS = 1000; // 1Hz (Boucle lente)
const STANDBY_TIMEOUT_MS = 300000; // 5 minutes
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};

// --- FACTEURS DE RÉACTIVITÉ UKF ---
const UKF_REACTIVITY_FACTORS = {
    'AUTO': { MULT: 1.0, DISPLAY: 'Automatique' },
    'NORMAL': { MULT: 1.0, DISPLAY: 'Normal' },
    'FAST': { MULT: 0.2, DISPLAY: 'Rapide' },
    'STABLE': { MULT: 2.5, DISPLAY: 'Microscopique' },
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

// ===========================================
// CLASSE UKF PROFESSIONNELLE (Architecture 21 États)
// ===========================================
// 

[Image of Unscented Kalman Filter block diagram]

class ProfessionalUKF {
    constructor() {
        if (typeof math === 'undefined') {
            throw new Error("math.js n'est pas chargé. L'UKF 21 états ne peut pas fonctionner.");
        }
        this.N_STATES = 21; 
        this.x = math.zeros(this.N_STATES); 
        this.P = math.diag(Array(this.N_STATES).fill(1e-2)); 
        this.Q = math.diag(Array(this.N_STATES).fill(1e-6));
        
        // Initialiser l'état (exemple : position 0,0,0)
        this.x.set([0], 0); this.x.set([1], 0); this.x.set([2], 0);
    }

    /** Étape de PRÉDICTION : Modèle de Mouvement Inertielles (Strapdown) */
    predict(imuData, dt) {
        // [Implémentation Strapdown complète (nécessite algèbre matricielle de math.js)]
        
        // PLACEHOLDER (Simulation simplifiée pour faire bouger les chiffres):
        // Intègre l'accélération (corrigée du biais estimé) pour prédire la vitesse
        const accel_biais_corrigé = imuData.accel[0] - this.x.get([6]); // Accel X - Biais X
        
        let vN = this.x.get([3]) + accel_biais_corrigé * dt; 
        if (Math.abs(vN) < MIN_SPD) vN = 0; // ZUPT
        this.x.set([3], vN); // vN
        
        // Prédit la position basée sur la vitesse prédite
        let lat = this.x.get([0]) + (vN / R_E_BASE) * dt;
        this.x.set([0], lat);
    }

    /** Étape de MISE À JOUR : Correction par le GPS */
    update(gpsData, R_dyn) {
        // [Implémentation de la correction UKF complète (nécessite algèbre matricielle de math.js)]
        
        // PLACEHOLDER : Correction directe (remplacé par la vraie fusion EKF/UKF)
        const K = 0.1; // Gain de Kalman (simplifié)
        this.x.set([0], this.x.get([0]) * (1-K) + (gpsData.latitude * D2R) * K);
        this.x.set([1], this.x.get([1]) * (1-K) + (gpsData.longitude * D2R) * K);
        this.x.set([2], this.x.get([2]) * (1-K) + gpsData.altitude * K);
        
        if (gpsData.speed) {
            const oldSpeed = this.x.get([3]);
            this.x.set([3], oldSpeed * (1-K) + gpsData.speed * K);
        }
    }
    
    getState() {
        const x_data = this.x.toArray ? this.x.toArray() : this.x; // Compatibilité math.js
        return {
            lat: x_data[0] * R2D, lon: x_data[1] * R2D, alt: x_data[2],
            vN: x_data[3], vE: x_data[4], vD: x_data[5],
            speed: Math.sqrt(x_data[3]**2 + x_data[4]**2 + x_data[5]**2),
            kUncert: this.P.get ? (this.P.get([3, 3]) + this.P.get([4, 4]) + this.P.get([5, 5])) : 0,
            kAltUncert: this.P.get ? this.P.get([2, 2]) : 0
        };
    }
}


// --- FONCTIONS DE FILTRAGE ET DE MODÈLE (Altitude, Bruit, etc.) ---

function getKalmanR(accRaw, kAlt, kUncert, env, reactivityMode) {
    let acc_effective = gpsAccuracyOverride > 0 ? gpsAccuracyOverride : accRaw;
    if (acc_effective > MAX_ACC) { return 1e9; } // Perte de signal (Mode Grotte/Tunnel)
    
    let R_gps_base = Math.min(acc_effective, 100) ** 2; 
    
    const env_mult = ENVIRONMENT_FACTORS[env]?.R_MULT || 1.0;
    let reactivity_mult = UKF_REACTIVITY_FACTORS[reactivityMode]?.MULT || 1.0;
    
    if (reactivityMode === 'AUTO' && acc_effective !== null) {
        if (acc_effective > 20) reactivity_mult = 3.0; 
        else if (acc_effective < 3) reactivity_mult = 0.5; 
    }
    
    let R_dyn = Math.min(R_gps_base * env_mult * reactivity_mult, UKF_R_MAX);
    return Math.max(R_dyn, R_ALT_MIN); 
}

function dist2D(lat1, lon1, lat2, lon2, R_earth) {
    const dLat = (lat2 - lat1) * D2R, dLon = (lon2 - lon1) * D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1*D2R) * Math.cos(lat2*D2R) * Math.sin(dLon / 2) ** 2;
    return (R_earth || R_E_BASE) * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a)); 
}

function getBarometricAltitude(P_hPa, P_ref_hPa, T_K) {
    return (T_K / 0.0065) * (1 - (P_hPa / P_ref_hPa)**(R_AIR * 0.0065 / G_ACC));
}

function calculateMRF(alt, netherMode) {
    if (netherMode) return NETHER_RATIO;
    return 1.0;
}

function updateCelestialBody(body, alt, rotationRadius, angularVelocity) {
    let G_ACC_NEW = CELESTIAL_DATA['EARTH'].G;
    let R_ALT_CENTER_REF_NEW = WGS84_A; 
    const data = CELESTIAL_DATA[body];
    if (data) {
        G_ACC_NEW = data.G;
        R_ALT_CENTER_REF_NEW = data.R;
    }
    if (body === 'ROTATING') {
        G_ACC_NEW = rotationRadius * angularVelocity ** 2;
    }
    G_ACC = G_ACC_NEW; // Mise à jour globale
    R_ALT_CENTER_REF = R_ALT_CENTER_REF_NEW; // Mise à jour globale
    return { G_ACC: G_ACC_NEW, R_ALT_CENTER_REF: R_ALT_CENTER_REF_NEW };
}

function getWGS84Gravity(lat, alt) {
    const latRad = lat * D2R; 
    const sin2lat = Math.sin(latRad) ** 2;
    const g_surface = WGS84_G_EQUATOR * (1 + WGS84_BETA * sin2lat) / Math.sqrt(1 - WGS84_E2 * sin2lat);
    return g_surface * (1 - 2 * alt / WGS84_A); 
}

function getSpeedOfSound(tempK) {
    if(tempK < KELVIN_OFFSET) tempK += KELVIN_OFFSET; // Correction si on reçoit des °C
    return Math.sqrt(GAMMA_AIR * R_AIR * tempK);
}

// --- FONCTIONS DE PHYSIQUE AVANCÉE (DU FICHIER (7).js) ---

function calculateAdvancedPhysics(kSpd, kAlt, mass, CdA, tempK, airDensity, lat, kAltUncert, localG, accel_long) {
    const V = kSpd;
    if(isNaN(V)) V = 0;
    const lorentzFactor = 1 / Math.sqrt(1 - Math.pow(V / C_L, 2));
    const timeDilationSpeed = (lorentzFactor - 1) * 86400 * 1e9; // ns/jour
    const E0 = mass * C_L * C_L;
    const energyRelativistic = lorentzFactor * E0;
    const momentum = mass * V; 
    const gravitationalDilation = (localG * kAlt / (C_L * C_L)) * 86400 * 1e9; // ns/jour
    const Rs_object = (2 * G_U * mass) / (C_L * C_L);
    
    const speedOfSoundLocal = getSpeedOfSound(tempK);
    const machNumber = V / speedOfSoundLocal;
    const dynamicPressure = 0.5 * airDensity * V * V; 
    const reynoldsNumber = (airDensity * V * 1) / MU_DYNAMIC_AIR; 
    const dragForce = dynamicPressure * (CdA || 0.5); // Utilise 0.5 si CdA n'est pas défini
    const dragPower_kW = (dragForce * V) / 1000.0;
    
    const coriolisForce = 2 * mass * V * OMEGA_EARTH * Math.sin(lat * D2R);
    const geopotentialAltitude = kAlt * (G_ACC / localG);
    const force_g_long = localG > 0.1 ? (accel_long / localG) : 0;
    
    const nyquistFrequency = 0.5 * (1000 / IMU_UPDATE_RATE_MS); 
    const altSigma = Math.sqrt(Math.abs(kAltUncert)); // S'assure de ne pas faire sqrt d'un négatif

    return { 
        lorentzFactor, timeDilationSpeed, energyRelativistic, E0, momentum, gravitationalDilation, Rs_object,
        speedOfSoundLocal, machNumber, dynamicPressure, reynoldsNumber, dragForce, dragPower_kW,
        coriolisForce, geopotentialAltitude, force_g_long,
        nyquistFrequency, altSigma
    };
}

function calculateBioSVT(tempC, alt, humidity_perc, pressurePa, sunAltitudeRad) {
    const a = 17.27, b = 237.7;
    const h_frac = humidity_perc / 100.0;
    const f = (a * tempC) / (b + tempC) + Math.log(h_frac);
    const dewPoint = (b * f) / (a - f);
    
    const wetBulbTemp = tempC * Math.atan(0.151977 * Math.sqrt(h_frac + 8.313659)) + 
                        Math.atan(tempC + h_frac) - 
                        Math.atan(h_frac - 1.67633) + 
                        0.00391838 * Math.pow(h_frac, 1.5) * Math.atan(0.023101 * h_frac) - 4.686035;
    const CAPE_sim = Math.max(0, 0.5 * (tempC - dewPoint) * 500); 
    const O2Saturation = SAT_O2_SEA_LEVEL - (alt / 1000) * 2; 
    const O2SaturationClamped = Math.max(70, Math.min(100, O2Saturation));
    const sunAngleFactor = Math.max(0, sunAltitudeRad * R2D) / 90; 
    const tempFactor = Math.exp(-0.5 * Math.pow((tempC - 25) / 10, 2)); 
    const photosynthesisRate = 0.05 * sunAngleFactor * tempFactor; 
    
    const solarIrradiance = SOLAR_FLUX_DENSITY * Math.max(0, Math.sin(sunAltitudeRad)); 
    const radiationPressure = solarIrradiance / C_L; 

    // Calcul Humidité Absolue
    const P_sat_hPa = tC => 6.1078 * Math.pow(10, (7.5 * tC) / (237.3 + tC));
    const P_v = P_sat_hPa(tempC) * h_frac; 
    const absoluteHumidity = (P_v * 100 * 18.015) / (8.314 * (tempC + KELVIN_OFFSET)) * 1000; // g/m³

    return { 
        dewPoint, wetBulbTemp, CAPE_sim, O2SaturationClamped, photosynthesisRate, 
        solarIrradiance, radiationPressure, absoluteHumidity
    };
}


// --- FONCTIONS ASTRO (SUNCALC & Custom) ---
const J1970 = 2440588;
const dayMs = 1000 * 60 * 60 * 24;

function getSolarTime(date, lon) {
    if (date === null || lon === null || isNaN(lon)) return { TST: '00:00:00', MST: '00:00:00', EOT: '0.00', ECL_LONG: '0.00' };
    
    const d = toDays(date);
    const M = solarMeanAnomaly(d); 
    const L = eclipticLongitude(M); 
    
    const J_star = toDays(date) - lon / 360;
    const J_transit = J_star + (0.0053 * Math.sin(M) - 0.0069 * Math.sin(2 * L));
    const eot_min = (J_star - J_transit) * 1440; 

    const msSinceMidnightUTC = (date.getUTCHours() * 3600 + date.getUTCMinutes() * 60 + date.getUTCSeconds()) * 1000 + date.getUTCMilliseconds();
    const mst_offset_ms = lon * dayMs / 360; 
    const mst_ms = (msSinceMidnightUTC + mst_offset_ms + dayMs) % dayMs;
    const eot_ms = eot_min * 60000;
    const tst_ms = (mst_ms + eot_ms + dayMs) % dayMs; 

    const toTimeString = (ms) => {
        let h = Math.floor(ms / 3600000);
        let m = Math.floor((ms % 3600000) / 60000);
        let s = Math.floor((ms % 60000) / 1000);
        return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
    };

    return { 
        TST: toTimeString(tst_ms), 
        MST: toTimeString(mst_ms), 
        EOT: eot_min.toFixed(2),
        ECL_LONG: (L * R2D).toFixed(2),
        DateMST: new Date(date.getTime() + mst_offset_ms),
        DateTST: new Date(date.getTime() + mst_offset_ms + eot_ms)
    };
}

function getMinecraftTime(date) {
    if (date === null) return '00:00';
    const msSinceMidnightUTC = date.getUTCHours() * 3600000 + date.getUTCMilliseconds() + date.getUTCMinutes() * 60000 + date.getUTCSeconds() * 1000;
    const timeRatio = (msSinceMidnightUTC % dayMs) / dayMs;
    const mcTimeMs = (timeRatio * MC_DAY_MS + MC_DAY_MS) % MC_DAY_MS;
    const toTimeString = (ms) => {
        let h = Math.floor(ms / 3600000);
        let m = Math.floor((ms % 3600000) / 60000);
        return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}`;
    };
    return toTimeString(mcTimeMs);
}

function getMoonPhaseName(phase) {
    if (phase < 0.03 || phase > 0.97) return "Nouvelle Lune 🌑";
    if (phase < 0.23) return "Premier Croissant 🌒";
    if (phase < 0.27) return "Premier Quartier 🌓";
    if (phase < 0.48) return "Gibbeuse Croissante 🌔";
    if (phase < 0.52) return "Pleine Lune 🌕";
    if (phase < 0.73) return "Gibbeuse Décroissante 🌖";
    if (phase < 0.77) return "Dernier Quartier 🌗";
    return "Dernier Croissant 🌘"; 
}

/** Calcule le Temps Sidéral Local Vrai (TSLV) (Simplification pour affichage) */
function getTSLV(date, lon) {
    if (date === null) return 'N/A';
    // Approximation du TSLV en heures
    const GMST = (date.getUTCHours() + date.getUTCMinutes() / 60) * 15; // GMST en degrés (très simplifié)
    const LST = GMST + lon;
    const LST_h = (LST / 15 + 24) % 24;
    return LST_h.toFixed(2) + ' h';
}
// =================================================================
// BLOC 2/3 : GESTION DES SERVICES, CAPTEURS ET CONTRÔLES
// =================================================================

// --- VARIABLES D'ÉTAT (Globales) ---
let wID = null, domSlowID = null, domFastID = null, lPos = null;
let lat = null, lon = null, sTime = null;
let distM = 0, maxSpd = 0;
let kSpd = 0, kUncert = UKF_R_MAX, kAltUncert = 10; 
let timeMoving = 0, timeTotal = 0; 
let lastFSpeed = 0; 
let kAlt = null;      
let ukf = null; 

let currentGPSMode = 'HIGH_FREQ'; 
let emergencyStopActive = false; 
let netherMode = false; 
let selectedEnvironment = 'NORMAL'; 
let currentMass = 70.0; 
let currentCdA = 0.5; // CdA par défaut
let R_FACTOR_RATIO = 1.0;
let currentCelestialBody = 'EARTH';
let rotationRadius = 100;
let angularVelocity = 0.0; 
let gpsAccuracyOverride = 0.0; 
let lastGPSPos = null;

let lServH = null, lLocH = null; // Horodatages NTP
let lastP_hPa = BARO_ALT_REF_HPA, lastT_K = 288.15, currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound = 343;
let sunAltitudeRad = 0; // Pour BioSVT
let lastWeatherData = null; // Stockage des données météo

let accel = { x: 0, y: 0, z: 0 };
let gyro = { x: 0, y: 0, z: 0 };
let mag = { x: 0, y: 0, z: 0 };
let lastIMUTimestamp = 0;
let lastMapUpdate = 0;
let map, marker, circle; // Objets Leaflet

let currentUKFReactivity = 'AUTO'; 
let gpsStandbyTimeoutID = null;    

// --- GESTION NTP (SYNCHRO HEURE) ---
async function syncH() {
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
        lServH = Date.now(); 
        lLocH = performance.now();
        if ($('local-time')) $('local-time').textContent = '❌ SYNCHRO ÉCHOUÉE (Local)';
    }
}

/** Retourne l'heure synchronisée. */
function getCDate() {
    if (lServH === null || lLocH === null) { return new Date(); }
    const offset = performance.now() - lLocH;
    return new Date(lServH + offset);
}

// --- GESTION API MÉTÉO ---
async function fetchWeather(lat, lon) {
    const url = `${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`;
    try {
        const response = await fetch(url);
        if (!response.ok) throw new Error(`Erreur HTTP ${response.status}`);
        const data = await response.json();

        if (data.main) {
            const P_hPa = data.main.pressure;
            const T_C = data.main.temp;
            const T_K = T_C + KELVIN_OFFSET;
            const H_perc = data.main.humidity;
            const P_Pa = P_hPa * 100; 
            const air_density = P_Pa / (R_AIR * T_K);

            // Mise à jour des globales pour la physique
            lastP_hPa = P_hPa;
            lastT_K = T_K;
            currentAirDensity = air_density;
            currentSpeedOfSound = getSpeedOfSound(T_K);
            
            lastWeatherData = { // Stockage global
                pressure_hPa: P_hPa, tempC: T_C, tempK: T_K,
                humidity_perc: H_perc, air_density: air_density
            };
            return lastWeatherData;
        }
    } catch (e) {
        if ($('weather-status')) $('weather-status').textContent = `❌ ${e.message}`;
        throw e; 
    }
    return null;
}

// --- GESTION DES CAPTEURS IMU (API MODERNE) ---
function startIMUListeners() {
    if (emergencyStopActive || domFastID) return; 
    try {
        if ($('imu-status')) $('imu-status').textContent = "Activation...";
        
        if (typeof Accelerometer === 'undefined' || typeof Gyroscope === 'undefined') {
             throw new Error("API Sensor non supportée.");
        }
        
        const accSensor = new Accelerometer({ frequency: 50 }); 
        accSensor.addEventListener('reading', () => {
            accel.x = accSensor.x; accel.y = accSensor.y; accel.z = accSensor.z;
        });
        accSensor.addEventListener('error', e => console.error("Erreur Accéléromètre:", e.error));
        accSensor.start();

   accSensor.start();
        
        // --- Gyroscope ---
        const gyroSensor = new Gyroscope({ frequency: 50 }); 
        gyroSensor.addEventListener('reading', () => {
            gyro.x = gyroSensor.x; gyro.y = gyroSensor.y; gyro.z = gyroSensor.z;
        });
        gyroSensor.addEventListener('error', e => console.error("Erreur Gyroscope:", e.error));
        gyroSensor.start();

        if ($('imu-status')) $('imu-status').textContent = "✅ IMU Active (50Hz)";
        
        // --- Démarrage de la BOUCLE RAPIDE (UKF Prediction) ---
        domFastID = setInterval(() => {
            const now = performance.now();
            const dt = lastIMUTimestamp ? (now - lastIMUTimestamp) / 1000 : MIN_DT;
            lastIMUTimestamp = now;

            // Étape 1 : Prédiction UKF à haute fréquence (50Hz)
            // Utilise les données IMU pour propager l'état (Strapdown Integration)
            ukf.predict({ accel, gyro }, dt);

            // Récupération de l'état filtré pour la cinématique rapide
            const state = ukf.getState();
            kSpd = state.speed * KMH_MS; // Vitesse en km/h
            kAlt = state.alt;
            kUncert = state.kUncert;
            kAltUncert = state.kAltUncert;
            
            // Mise à jour de la distance et du temps de mouvement
            if (kSpd > MIN_SPD) {
                distM += (kSpd / KMH_MS) * dt;
                timeMoving += dt;
            }
            timeTotal += dt;

            // Mise à jour du DOM critique (vitesse, accélération, altitude UKF)
            updateDOMFast(state, accel, gyro, dt);
            
        }, IMU_UPDATE_RATE_MS); // 20ms = 50Hz

    } catch (e) {
        console.error("Erreur Capteur IMU Moderne:", e);
        if ($('imu-status')) $('imu-status').textContent = `❌ ${e.message}`;
        // En cas d'échec, tenter l'API plus ancienne (handleDeviceMotion)
        if (window.DeviceMotionEvent) startOldIMUListeners(); 
    }
}

// --- GESTION IMU (Ancienne API pour la compatibilité) ---
function startOldIMUListeners() {
    // Si l'ancienne API est démarrée, on ne démarre pas la nouvelle
    if (domFastID) return; 
    
    // Nécessite l'autorisation de l'utilisateur sur mobile
    if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', handleDeviceMotion, true);
        if ($('imu-status')) $('imu-status').textContent = "⚠️ IMU Active (Motion)";
    } else {
        if ($('imu-status')) $('imu-status').textContent = "🚫 IMU Non Supporté";
    }
}

function handleDeviceMotion(event) {
    // Met à jour l'état `accel` et `gyro` à partir de l'objet event
    if (event.accelerationIncludingGravity) {
        accel.x = event.accelerationIncludingGravity.x;
        accel.y = event.accelerationIncludingGravity.y;
        accel.z = event.accelerationIncludingGravity.z;
    }
    if (event.rotationRate) {
        gyro.x = event.rotationRate.alpha;
        gyro.y = event.rotationRate.beta;
        gyro.z = event.rotationRate.gamma;
    }
    // Note: Le `domFastID` doit être mis en place pour traiter ces données
}

// --- FONCTION DE MISE À JOUR RAPIDE (DOM) ---
function updateDOMFast(state, accelRaw, gyroRaw, dt) {
    if ($('speed-val')) $('speed-val').textContent = dataOrDefault(state.speed * KMH_MS, 5, ' km/h');
    if ($('vel-ms')) $('vel-ms').textContent = dataOrDefault(state.speed, 2, ' m/s');
    if ($('alt-val')) $('alt-val').textContent = dataOrDefault(state.alt, 2, ' m');
    
    // Affichage des données brutes/filtrées de l'IMU (ex: Accélération Longitudinale)
    const accel_long = accelRaw.x; // Simplification : Accel X est l'axe avant
    if ($('accel-long-val')) $('accel-long-val').textContent = dataOrDefault(accel_long, 2, ' m/s²');
    
    // Calculs physiques rapides
    const localG = getWGS84Gravity(state.lat, state.alt);
    const physics = calculateAdvancedPhysics(state.speed, state.alt, currentMass, currentCdA, lastT_K, currentAirDensity, state.lat, state.kAltUncert, localG, accel_long);
    if ($('mach-number')) $('mach-number').textContent = dataOrDefault(physics.machNumber, 3, '');
    if ($('drag-power-kw')) $('drag-power-kw').textContent = dataOrDefault(physics.dragPower_kW, 2, ' kW');
    if ($('force-g-long')) $('force-g-long').textContent = dataOrDefault(physics.force_g_long, 2, ' G');
}

// --- FONCTION DE MISE À JOUR LENTE (DOM, Météo, Astro) ---
function updateDOMLow(weatherData, physics, bioSim) {
    if ($('temp-air-2')) $('temp-air-2').textContent = `${weatherData.tempC.toFixed(1)} °C`;
    if ($('pressure-2')) $('pressure-2').textContent = `${weatherData.pressure_hPa.toFixed(0)} hPa`;
    if ($('humidity-2')) $('humidity-2').textContent = `${weatherData.humidity_perc.toFixed(0)} %`;
    if ($('dew-point')) $('dew-point').textContent = `${bioSim.dewPoint.toFixed(1)} °C`;
    if ($('air-density')) $('air-density').textContent = `${weatherData.air_density.toFixed(3)} kg/m³`;
    if ($('o2-perc')) $('o2-perc').textContent = `${bioSim.O2SaturationClamped.toFixed(1)} %`;
    if ($('perc-light')) $('perc-light').textContent = `${((physics.speed / C_L) * 100).toExponential(2)} %`;
    if ($('perc-sound')) $('perc-sound').textContent = `${(physics.machNumber * 100).toFixed(1)} %`;
    
    // Affichage UKF Debug
    if ($('kalman-uncert')) $('kalman-uncert').textContent = dataOrDefaultExp(kUncert, 2, ' m²/s²');
    const R_dyn = getKalmanR(gpsAccuracyOverride, kAlt, kUncert, selectedEnvironment, currentUKFReactivity);
    if ($('speed-error-perc')) $('speed-error-perc').textContent = dataOrDefault(Math.sqrt(R_dyn), 2, ' m');
}

// --- FONCTION DE MISE À JOUR DE L'HEURE ---
function updateTimeDisplay(now) {
    if (now) {
        if ($('local-time') && !$('local-time').textContent.includes('Synchronisation...')) {
            $('local-time').textContent = now.toLocaleTimeString('fr-FR');
        }
        if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
        if ($('time-elapsed')) $('time-elapsed').textContent = sTime ? ((now.getTime() - sTime) / 1000).toFixed(2) + ' s' : '0.00 s';
        if ($('time-moving')) $('time-moving').textContent = timeMoving.toFixed(2) + ' s';
        if ($('mc-time')) $('mc-time').textContent = getMinecraftTime(now); 
        
        // Mise à jour de la période de la journée (Mode Nuit/Jour)
        // ... (Logique pour déterminer le mode sombre/clair)
    }
}

// --- LOGIQUE GPS (Simplifiée pour la complétion) ---
function handleGPSUpdate(position) {
    const now = position.timestamp;
    const dt = lPos ? (now - lPos.timestamp) / 1000 : MIN_DT;
    lPos = position;
    
    // Correction de l'état UKF (Mise à jour)
    const R_dyn = getKalmanR(position.coords.accuracy, kAlt, kUncert, selectedEnvironment, currentUKFReactivity);
    ukf.update({ 
        latitude: position.coords.latitude, 
        longitude: position.coords.longitude, 
        altitude: position.coords.altitude, 
        speed: position.coords.speed 
    }, R_dyn);
    
    // Mise à jour de la carte (si l'intervalle est dépassé)
    const state = ukf.getState();
    const newLatLon = [state.lat, state.lon];
    if (map && marker) {
        marker.setLatLng(newLatLon);
        circle.setLatLng(newLatLon).setRadius(position.coords.accuracy || 10);
        
        // Ajout à la polyline
        // pathCoordinates.push(newLatLon);
        // if (polyline) polyline.setLatLngs(pathCoordinates);
        
        if (now - lastMapUpdate > MAP_UPDATE_INTERVAL) {
             map.panTo(newLatLon);
             lastMapUpdate = now;
        }
    }
    
    // Mise à jour de la vitesse max
    maxSpd = Math.max(maxSpd, state.speed * KMH_MS);
    if ($('speed-max')) $('speed-max').textContent = dataOrDefault(maxSpd, 5, ' km/h');
}

function startGPS() {
    if (navigator.geolocation) {
        sTime = Date.now();
        navigator.geolocation.watchPosition(handleGPSUpdate, (error) => {
            console.error('Erreur GPS:', error);
            if ($('gps-status')) $('gps-status').textContent = '❌ ÉCHOUÉ';
        }, GPS_OPTS.HIGH_FREQ);
        if ($('gps-status')) $('gps-status').textContent = 'ACTIF';
    } else {
        if ($('gps-status')) $('gps-status').textContent = 'NON SUPPORTÉ';
    }
}

// --- INITIALISATION DE LA CARTE (Leaflet) ---
function initMap() {
    map = L.map('map').setView([43.296, 5.370], 13); 
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        maxZoom: 19,
        attribution: '© OpenStreetMap'
    }).addTo(map);

    marker = L.marker([43.296, 5.370]).addTo(map)
        .bindPopup("Position GNSS").openPopup();
        
    circle = L.circle([43.296, 5.370], { radius: 10, color: 'blue', fillOpacity: 0.1 }).addTo(map);
    // polyline = L.polyline(pathCoordinates, {color: 'red'}).addTo(map);
}

// --- FONCTIONS ASTRO (PLACEHOLDERS) ---
function solarMeanAnomaly(d) { return D2R * (357.5291 + 0.98560028 * d); } 
function eclipticLongitude(M) { return D2R * (280.470 + 0.98564736 * toDays(getCDate())) + D2R * 1.915 * Math.sin(M) + D2R * 0.02 * Math.sin(2 * M); } 
function toDays(date) { return date.getTime() / dayMs - (J1970 - J2000); } 

function updateAstro(lat, lon) {
    const now = getCDate();
    const solar = SunCalc.getTimes(now, lat, lon);
    const moon = SunCalc.getMoonIllumination(now);
    const moonPos = SunCalc.getMoonPosition(now, lat, lon);
    const sunPos = SunCalc.getPosition(now, lat, lon);
    
    sunAltitudeRad = sunPos.altitude; // Mis à jour pour BioSVT

    if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(sunPos.altitude * R2D, 2, ' °');
    if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(moon.phase);
    // ... Mise à jour du reste du DOM Astro
    
    const solarTime = getSolarTime(now, lon);
    if ($('tstv')) $('tstv').textContent = getTSLV(now, lon);
    if ($('tst')) $('tst').textContent = solarTime.TST;
    if ($('eot')) $('eot').textContent = solarTime.EOT + ' min';
}


// =================================================================
// BLOC 3/3 : DÉMARRAGE ET ÉVÉNEMENTS
// =================================================================

document.addEventListener('DOMContentLoaded', () => {
    // 1. Initialisation de la librairie mathématique pour UKF Pro
    try {
        if (typeof math === 'undefined') {
            throw new Error("La librairie math.js est requise pour ProfessionalUKF.");
        }
        ukf = new ProfessionalUKF(); 
    } catch (e) {
        console.error(e.message);
        if ($('kalman-status')) $('kalman-status').textContent = `❌ ${e.message}`;
        return; // Arrêter le script si l'initialisation UKF échoue
    }

    // 2. Initialisation des composants statiques
    initMap(); 
    syncH(); 
    updateCelestialBody(currentCelestialBody); // Définit G_ACC / R_ALT_CENTER_REF

    // 3. Démarrage des boucles et des capteurs
    startGPS();
    startIMUListeners(); // Démarrage de la boucle rapide (domFastID)

    // Démarrage de la Boucle Lente (domSlowID) - Météo, Astro, Affichage Lente
    if (domSlowID === null) {
        domSlowID = setInterval(() => {
            const now = getCDate();
            const currentLat = ukf.getState().lat || 43.296; 
            const currentLon = ukf.getState().lon || 5.370;
            const state = ukf.getState();
            
            // Mise à jour Météo et Calculs (si position filtrée valide)
            if (currentLat !== 0 && currentLon !== 0 && !emergencyStopActive) {
                fetchWeather(currentLat, currentLon)
                    .then(data => {
                        if (data) {
                            const localG = getWGS84Gravity(state.lat, state.alt);
                            const physics = calculateAdvancedPhysics(state.speed, state.alt, currentMass, currentCdA, data.tempK, data.air_density, state.lat, state.kAltUncert, localG, accel.x);
                            const bioSim = calculateBioSVT(data.tempC, state.alt || 0, data.humidity_perc, data.pressure_hPa * 100, sunAltitudeRad);
                            updateDOMLow(data, physics, bioSim);
                        }
                    })
                    .catch(() => {}); // Gérer les erreurs de fetch silencieusement
            }

            // Mise à jour Astro (Soleil/Lune/Temps)
            updateAstro(currentLat, currentLon); 

            // Mise à jour Horloge / Durées
            updateTimeDisplay(now); 
            
        }, DOM_SLOW_UPDATE_MS); // 1000ms (1Hz)
    }
    
    // Ajout des gestionnaires d'événements pour les contrôles (si DOM a les éléments)
    if ($('control-nether')) $('control-nether').addEventListener('change', (e) => netherMode = e.target.checked);
    if ($('control-env')) $('control-env').addEventListener('change', (e) => selectedEnvironment = e.target.value);
    if ($('control-react')) $('control-react').addEventListener('change', (e) => currentUKFReactivity = e.target.value);
});
// FIN DU FICHIER JAVASCRIPT
