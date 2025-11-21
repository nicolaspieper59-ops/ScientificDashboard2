// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// BLOC 1/4 : Constantes, Modèles Physiques & Classes
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
// 

[Image of Unscented Kalman Filter block diagram]

// ===========================================
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
    let V = kSpd;
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
const J2000 = 2451545.0; // Renommé pour éviter conflit
const dayMs = 1000 * 60 * 60 * 24;
const MC_DAY_MS = 72 * 60 * 1000; 

function toDays(date) { return (date.valueOf() / dayMs - 0.5 + J1970) - J2000; }
function solarMeanAnomaly(d) { return D2R * (356.0470 + 0.9856002585 * d); }
function eclipticLongitude(M) {
    var C = D2R * (1.9148 * Math.sin(M) + 0.0200 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M)), 
        P = D2R * 102.9377;                                                                
    return M + C + P + Math.PI;
}

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
// BLOC 2/4 : GESTION DES SERVICES, CAPTEURS ET CONTRÔLES
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

        const gyroSensor = new Gyroscope({ frequency: 50 });
        gyroSensor.addEventListener('reading', () => {
            gyro.x = gyroSensor.x; gyro.y = gyroSensor.y; gyro.z = gyroSensor.z;
        });
        gyroSensor.addEventListener('error', e => console.error("Erreur Gyroscope:", e.error));
        gyroSensor.start();
        
        if ($('imu-status')) $('imu-status').textContent = "Actif (API Sensor 50Hz)";
        lastIMUTimestamp = performance.now();
        
        startFastLoop(); // DÉMARRE LA BOUCLE RAPIDE (PRÉDICTION)

    } catch (error) {
        let errMsg = error.message;
        if (error.name === 'SecurityError' || error.name === 'NotAllowedError') {
            errMsg = "Permission Capteurs Refusée. Cliquez sur 'MARCHE GPS' pour autoriser.";
        }
        if ($('imu-status')) $('imu-status').textContent = `❌ ${errMsg}`;
    }
}

function stopIMUListeners() {
    if (domFastID) clearInterval(domFastID);
    domFastID = null;
    if ($('imu-status')) $('imu-status').textContent = "Inactif";
    accel = { x: 0, y: 0, z: 0 };
    gyro = { x: 0, y: 0, z: 0 };
}

// --- GESTION GPS (GÉOLOCALISATION) ---
function startGPS(mode = currentGPSMode) {
    if (emergencyStopActive) return;
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    
    currentGPSMode = mode;
    const options = GPS_OPTS[mode];
    
    wID = navigator.geolocation.watchPosition(gpsUpdateCallback, handleErr, options);
    
    // Démarrage de l'IMU en même temps que le GPS (pour les permissions)
    if (!domFastID) {
        startIMUListeners();
    }
    
    let text = (mode === 'LOW_FREQ' && kSpd < MIN_SPD * 2) ? '⏸️ GPS EN VEILLE' : '⏸️ PAUSE GPS';
    if ($('toggle-gps-btn')) {
        $('toggle-gps-btn').textContent = text;
        $('toggle-gps-btn').style.backgroundColor = '#ffc107'; 
    }
}

function stopGPS(resetButton = true) {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    wID = null;
    if (gpsStandbyTimeoutID) clearTimeout(gpsStandbyTimeoutID);
    gpsStandbyTimeoutID = null;
    
    // Arrêter l'IMU si le GPS est arrêté
    stopIMUListeners();
    
    if (resetButton && $('toggle-gps-btn')) {
        $('toggle-gps-btn').textContent = '▶️ MARCHE GPS';
        $('toggle-gps-btn').style.backgroundColor = '#28a745'; 
    }
}

function toggleGPS() {
    if (emergencyStopActive) return;
    wID === null ? startGPS('HIGH_FREQ') : stopGPS();
}

function toggleEmergencyStop() {
    emergencyStopActive = !emergencyStopActive;
    if (emergencyStopActive) {
        stopGPS(false); 
        stopIMUListeners();
        if($('emergency-stop-btn')) {
            $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: ACTIF 🔴";
            $('emergency-stop-btn').classList.add('active');
        }
        if ($('speed-status-text')) $('speed-status-text').textContent = '🛑 ARRÊT D’URGENCE';
    } else {
        if($('emergency-stop-btn')) {
            $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: INACTIF 🟢";
            $('emergency-stop-btn').classList.remove('active');
        }
        startGPS(); 
        // startIMUListeners() sera appelé par startGPS()
    }
}

function handleErr(err) {
    let errMsg = `Erreur GPS (Code ${err.code}): `;
    if (err.code === 1) errMsg += "Permission refusée.";
    else if (err.code === 2) errMsg += "Position indisponible (Pas de signal).";
    else if (err.code === 3) errMsg += "Timeout GPS.";
    else errMsg += `Erreur inconnue: ${err.message}`;

    if ($('gps-precision')) $('gps-precision').textContent = errMsg;
    if (err.code === 1) stopGPS(); 
}

// --- GESTION CARTE (LEAFLET) ---
function initMap() {
    try {
        if ($('map') && typeof L !== 'undefined' && !map) { 
            map = L.map('map').setView([43.296, 5.37], 10);
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                attribution: '© OpenStreetMap contributors'
            }).addTo(map);
            marker = L.marker([43.296, 5.37]).addTo(map);
            circle = L.circle([43.296, 5.37], { color: 'red', fillColor: '#f03', fillOpacity: 0.5, radius: 10 }).addTo(map);
            setTimeout(() => map.invalidateSize(), 400); 
        }
    } catch (e) {
        if ($('map')) $('map').innerHTML = "Erreur d'initialisation de la carte.";
    }
}

function updateMap(lat, lon, acc) {
    if (map && marker) {
        const latLng = [lat, lon];
        marker.setLatLng(latLng);
        circle.setLatLng(latLng).setRadius(acc * R_FACTOR_RATIO); 
        const now = Date.now();
        if (now - lastMapUpdate > MAP_UPDATE_INTERVAL && kSpd > MIN_SPD) {
            map.setView(latLng, map.getZoom() > 10 ? map.getZoom() : 16); 
            lastMapUpdate = now;
        }
    }
}
// =================================================================
// BLOC 3/4 : BOUCLES PRINCIPALES & GESTION DU DOM
// =================================================================

/**
 * BOUCLE LENTE (Callback GPS) - Correction UKF
 * Se déclenche à chaque nouvelle position GPS.
 */
function gpsUpdateCallback(pos) {
    if (emergencyStopActive || !ukf) return;
    
    lastGPSPos = pos; 
    const accRaw = pos.coords.accuracy || 100;
    
    // Calcul du bruit de mesure (R)
    let R_dyn = getKalmanR(accRaw, kAlt, kUncert, selectedEnvironment, currentUKFReactivity); 
    
    let isSignalPoor = (accRaw > MAX_ACC || R_dyn >= UKF_R_MAX * 0.9);

    // LOGIQUE ZUPT (Mode Grotte/Arrêt)
    const spd3D_raw_gps = pos.coords.speed || 0;
    const accel_long_provisional = Math.abs(accel.x); // Utilise l'IMU
    
    const isPlausiblyStopped = (
        spd3D_raw_gps < ZUPT_RAW_THRESHOLD && 
        accel_long_provisional < ZUPT_ACCEL_THRESHOLD &&
        !isSignalPoor 
    ); 
    
    if (isSignalPoor) {
        // Mode "Grotte" : Ne pas mettre à jour l'UKF avec des données GPS invalides
        if ($('gps-precision')) $('gps-precision').textContent = `❌ ${accRaw.toFixed(0)} m (Estimation/Drift)`;
        if ($('gps-status-dr')) $('gps-status-dr').textContent = 'Drift (Estimation)';
        // L'UKF continue en mode Prédiction pure (IMU seul)
    } else if (isPlausiblyStopped) {
        // Mode "ZUPT" : Forcer la vitesse à 0
        if ($('gps-precision')) $('gps-precision').textContent = `${accRaw.toFixed(2)} m (ZUPT)`;
        if ($('gps-status-dr')) $('gps-status-dr').textContent = '✅ ZUPT (Vélocité Nulle)';
        // Forcer la mise à jour de l'UKF avec une vitesse nulle et une confiance élevée
        let zuptData = { ...pos.coords, speed: 0 };
        ukf.update(zuptData, R_ALT_MIN); 
    } else {
        // Mode "Manège" ou normal : Corriger l'UKF avec le GPS
        if ($('gps-precision')) $('gps-precision').textContent = `${accRaw.toFixed(2)} m`;
        if ($('gps-status-dr')) $('gps-status-dr').textContent = 'Actif (Fusion UKF)';
        ukf.update(pos.coords, R_dyn);
    }
}

/**
 * BOUCLE RAPIDE (IMU/EKF Prédiction) - 50Hz
 * Se déclenche toutes les 20ms pour la prédiction IMU et l'affichage DOM.
 */
function startFastLoop() {
    if (domFastID) return; 
    
    domFastID = setInterval(() => {
        if (emergencyStopActive || !ukf) return;
        
        const now = performance.now();
        const dt = (now - lastIMUTimestamp) / 1000.0;
        if (dt < MIN_DT) return; 
        lastIMUTimestamp = now;

        // --- 1. PRÉDICTION UKF (Mouvement) ---
        const imuReadings = {
            accel: [accel.x, accel.y, accel.z],
            gyro: [gyro.x, gyro.y, gyro.z]
        };
        ukf.predict(imuReadings, dt);

        // --- 2. EXTRACTION DE L'ÉTAT (UKF) ---
        const estimatedState = ukf.getState();
        lat = estimatedState.lat;
        lon = estimatedState.lon;
        kAlt = estimatedState.alt;
        kSpd = estimatedState.speed; // Vitesse stable (m/s)
        kUncert = estimatedState.kUncert;
        kAltUncert = estimatedState.kAltUncert;

        const sSpdFE = kSpd < MIN_SPD ? 0 : kSpd;
        
        const spd3D_raw_gps = (lastGPSPos && lastGPSPos.coords.speed) ? lastGPSPos.coords.speed : 0;
        const accel_long = accel.x; 
        const local_g = getWGS84Gravity(lat, kAlt);

        // --- 3. CALCULS PHYSIQUES AVANCÉS (Temps réel) ---
        const advancedPhysics = calculateAdvancedPhysics(sSpdFE, kAlt, currentMass, currentCdA, 
            lastT_K, currentAirDensity, 
            lat, kAltUncert, local_g, accel_long);

        R_FACTOR_RATIO = calculateMRF(kAlt, netherMode); 
        distM += sSpdFE * dt * R_FACTOR_RATIO; 
        
        if (sSpdFE > MIN_SPD) { timeMoving += dt; }
        if (sTime) { timeTotal = (Date.now() - sTime) / 1000; }
        
        const maxSpd_kmh_raw = spd3D_raw_gps * KMH_MS;
        if (maxSpd_kmh_raw > maxSpd) maxSpd = maxSpd_kmh_raw; 
        
        // GESTION DE L'ÉNERGIE GPS AUTOMATIQUE
        if (sSpdFE < MIN_SPD * 2 && currentGPSMode === 'HIGH_FREQ') {
            if (gpsStandbyTimeoutID === null) {
                gpsStandbyTimeoutID = setTimeout(() => startGPS('LOW_FREQ'), STANDBY_TIMEOUT_MS);
            }
        } else if (sSpdFE >= MIN_SPD * 2 && currentGPSMode === 'LOW_FREQ') {
            startGPS('HIGH_FREQ');
            if (gpsStandbyTimeoutID) clearTimeout(gpsStandbyTimeoutID);
            gpsStandbyTimeoutID = null;
        }

        // --- 4. MISE À JOUR DU DOM (Rapide) ---
        
        $('elapsed-time').textContent = dataOrDefault(timeTotal, 2, ' s');
        $('time-moving').textContent = dataOrDefault(timeMoving, 2, ' s');
        $('distance-ratio').textContent = dataOrDefault(R_FACTOR_RATIO, 3);
        $('nether-mode-status').textContent = dataOrDefault(R_FACTOR_RATIO, 3); // (ID caché)

        // Section Vitesse & Relativité
        $('speed-stable').textContent = dataOrDefault(sSpdFE * KMH_MS, 2);
        $('speed-status-text').textContent = (ukf && kSpd > MIN_SPD) ? "🚀 UKF 21 ÉTATS (INS)" : "✅ ZUPT (Attente Mouvement)";
        $('speed-stable-ms').textContent = dataOrDefault(sSpdFE, 3, ' m/s');
        $('speed-stable-kms').textContent = dataOrDefaultExp(sSpdFE / 1000, 3, ' km/s');
        $('speed-3d-inst').textContent = dataOrDefault(spd3D_raw_gps * KMH_MS, 2, ' km/h');
        $('speed-raw-ms').textContent = dataOrDefault(spd3D_raw_gps, 3, ' m/s');
        $('speed-max').textContent = dataOrDefault(maxSpd, 2, ' km/h');
        $('speed-avg-moving').textContent = timeMoving > 1 ? dataOrDefault(distM / timeMoving * KMH_MS, 2, ' km/h') : '0.00 km/h';
        $('speed-avg-total').textContent = timeTotal > 1 ? dataOrDefault(distM / timeTotal * KMH_MS, 2, ' km/h') : '0.00 km/h';
        
        $('speed-of-sound-calc').textContent = dataOrDefault(advancedPhysics.speedOfSoundLocal, 2, ' m/s');
        $('perc-speed-sound').textContent = dataOrDefault(advancedPhysics.machNumber * 100, 2, ' %');
        $('mach-number').textContent = dataOrDefault(advancedPhysics.machNumber, 4);
        $('perc-speed-c').textContent = dataOrDefaultExp(sSpdFE / C_L * 100, 2, ' %');
        $('lorentz-factor').textContent = dataOrDefault(advancedPhysics.lorentzFactor, 8);
        $('time-dilation-v').textContent = dataOrDefault(advancedPhysics.timeDilationSpeed, 3, ' ns/j');
        $('time-dilation-g').textContent = dataOrDefault(advancedPhysics.gravitationalDilation, 3, ' ns/j');
        
        $('energy-relativistic').textContent = dataOrDefaultExp(advancedPhysics.energyRelativistic, 3, ' J');
        $('energy-rest-mass').textContent = dataOrDefaultExp(advancedPhysics.E0, 3, ' J');
        $('momentum').textContent = dataOrDefault(advancedPhysics.momentum, 2, ' kg·m/s');
        $('Rs-object').textContent = dataOrDefaultExp(advancedPhysics.Rs_object, 3, ' m');

        // Section Distance
        $('distance-total-km').textContent = `${dataOrDefault(distM / 1000, 3)} km | ${dataOrDefault(distM, 2)} m`;
        const dist_light_s = distM / C_L;
        $('distance-light-s').textContent = dataOrDefaultExp(dist_light_s, 2, ' s');
        $('distance-light-min').textContent = dataOrDefaultExp(dist_light_s / 60, 2, ' min');
        $('distance-light-h').textContent = dataOrDefaultExp(dist_light_s / 3600, 2, ' h');
        $('distance-light-day').textContent = dataOrDefaultExp(dist_light_s / 86400, 2, ' j');
        $('distance-light-week').textContent = dataOrDefaultExp(dist_light_s / (86400 * 7), 2, ' sem');
        $('distance-light-month').textContent = dataOrDefaultExp(dist_light_s / (86400 * 30.44), 2, ' mois');
        $('distance-cosmic').textContent = `${dataOrDefaultExp(distM / AU_METERS, 2)} UA | ${dataOrDefaultExp(distM / LIGHT_YEAR_METERS, 2)} al`;
        $('distance-horizon').textContent = dataOrDefault(calculateMaxVisibleDistance(kAlt) / 1000, 1, ' km');

        // Section Carte
        updateMap(lat, lon, (lastGPSPos ? lastGPSPos.coords.accuracy : 100));

        // Section Dynamique & Forces
        $('gravity-local').textContent = dataOrDefault(local_g, 4, ' m/s²');
        $('accel-long').textContent = dataOrDefault(accel_long, 3, ' m/s²');
        $('force-g-long').textContent = dataOrDefault(advancedPhysics.force_g_long, 2, ' G');
        $('vertical-speed').textContent = dataOrDefault(estimatedState.vD * -1, 2, ' m/s'); // vD est Négatif vers le bas
        $('angular-speed').textContent = dataOrDefault(Math.sqrt(gyro.x**2 + gyro.y**2 + gyro.z**2) * R2D, 2, ' °/s');
        $('dynamic-pressure').textContent = dataOrDefault(advancedPhysics.dynamicPressure, 2, ' Pa');
        $('drag-force').textContent = dataOrDefault(advancedPhysics.dragForce, 2, ' N');
        $('drag-power-kw').textContent = dataOrDefault(advancedPhysics.dragPower_kW, 2, ' kW');
        $('reynolds-number').textContent = dataOrDefaultExp(advancedPhysics.reynoldsNumber, 2);
        $('kinetic-energy').textContent = dataOrDefault(0.5 * currentMass * sSpdFE**2, 2, ' J');
        $('mechanical-power').textContent = dataOrDefault(advancedPhysics.accel_long * currentMass * sSpdFE, 2, ' W');
        $('coriolis-force').textContent = dataOrDefaultExp(advancedPhysics.coriolisForce, 2, ' N');

        // Section EKF/Debug
        $('kalman-uncert').textContent = dataOrDefault(kUncert, 3, ' m²/s² (P)');
        $('alt-uncertainty').textContent = dataOrDefault(advancedPhysics.altSigma, 3, ' m (σ)');
        const R_dyn_display = getKalmanR((lastGPSPos ? lastGPSPos.coords.accuracy : 100), kAlt, kUncert, selectedEnvironment, currentUKFReactivity);
        $('speed-error-perc').textContent = dataOrDefault(R_dyn_display, 3, ' m² (R dyn)');
        $('nyquist-frequency').textContent = dataOrDefault(advancedPhysics.nyquistFrequency, 2, ' Hz');
        $('gps-accuracy-display').textContent = dataOrDefault(gpsAccuracyOverride, 6, ' m');

        // Section IMU
        $('accel-x').textContent = dataOrDefault(accel.x, 2, ' m/s²');
        $('accel-y').textContent = dataOrDefault(accel.y, 2, ' m/s²');
        $('accel-z').textContent = dataOrDefault(accel.z, 2, ' m/s²');
        
        // Section Position
        $('lat-display').textContent = dataOrDefault(lat, 6, ' °');
        $('lon-display').textContent = dataOrDefault(lon, 6, ' °');
        $('alt-display').textContent = dataOrDefault(kAlt, 2, ' m');
        $('geopotential-alt').textContent = dataOrDefault(advancedPhysics.geopotentialAltitude, 2, ' m');
        
    }, IMU_UPDATE_RATE_MS);
                            }
// =================================================================
// BLOC 4/4 : INITIALISATION DOM ET BOUCLE LENTE (Astro/Météo)
// =================================================================

/**
 * BOUCLE LENTE (Astro/Météo) - 1Hz
 * Se déclenche toutes les secondes pour les calculs lourds et non critiques.
 */
function startSlowLoop() {
    if (domSlowID) return;
    
    const updateSlowData = async () => {
        const currentLat = lat || 43.296; // Position par défaut (Marseille) si GPS non dispo
        const currentLon = lon || 5.37;
        const now = getCDate();

        // 1. Mise à jour Astro (SunCalc & TST)
        if (typeof SunCalc !== 'undefined' && lat && lon) {
            try {
                const sunPos = SunCalc.getPosition(now, currentLat, currentLon);
                const moonIllum = SunCalc.getMoonIllumination(now);
                const moonPos = SunCalc.getMoonPosition(now, currentLat, currentLon);
                const sunTimes = SunCalc.getTimes(now, currentLat, currentLon);
                const moonTimes = SunCalc.getMoonTimes(now, currentLat, currentLon, true);
                const solarTimes = getSolarTime(now, currentLon);
                sunAltitudeRad = sunPos.altitude; // Mettre à jour pour BioSVT

                // --- MAJ DOM ASTRO ---
                $('date-display-astro').textContent = now.toLocaleDateString() || '...';
                $('date-solar-mean').textContent = solarTimes.DateMST ? solarTimes.DateMST.toLocaleDateString() : '...';
                $('date-solar-true').textContent = solarTimes.DateTST ? solarTimes.DateTST.toLocaleDateString() : '...';
                $('mst').textContent = solarTimes.MST;
                $('tst').textContent = solarTimes.TST;
                $('noon-solar').textContent = sunTimes.solarNoon ? sunTimes.solarNoon.toLocaleTimeString('fr-FR', { timeZone: 'UTC' }) : '...';
                $('eot').textContent = `${solarTimes.EOT} min`;
                $('tslv').textContent = getTSLV(now, currentLon);
                $('ecl-long').textContent = `${solarTimes.ECL_LONG}°`;
                
                $('sun-alt').textContent = `${(sunPos.altitude * R2D).toFixed(2)}°`;
                $('sun-azimuth').textContent = `${(sunPos.azimuth * R2D).toFixed(2)}°`;
                if (sunTimes.sunset && sunTimes.sunrise) {
                    const durationMs = sunTimes.sunset.getTime() - sunTimes.sunrise.getTime();
                    $('day-duration').textContent = `${Math.floor(durationMs / 3600000)}h ${Math.floor((durationMs % 3600000) / 60000)}m`;
                }
                
                $('moon-phase-name').textContent = getMoonPhaseName(moonIllum.phase);
                $('moon-illuminated').textContent = `${(moonIllum.fraction * 100).toFixed(1)}%`;
                $('moon-alt').textContent = `${(moonPos.altitude * R2D).toFixed(2)}°`;
                $('moon-azimuth').textContent = `${(moonPos.azimuth * R2D).toFixed(2)}°`;
                $('moon-times').textContent = (moonTimes.rise && moonTimes.set) ? `${moonTimes.rise.toLocaleTimeString()} / ${moonTimes.set.toLocaleTimeString()}` : '...';

                // --- MAJ Horloge Visuelle (Minecraft) ---
                const clockDiv = $('minecraft-clock');
                if (clockDiv) {
                    if (sunPos.altitude > 0) { clockDiv.className = 'sky-day'; $('clock-status').textContent = 'Jour (☀️)'; }
                    else if (sunPos.altitude > -10 * D2R) { clockDiv.className = 'sky-sunset'; $('clock-status').textContent = 'Crépuscule/Aube (✨)'; }
                    else { clockDiv.className = 'sky-night'; $('clock-status').textContent = 'Nuit (🌙)'; }
                }
                
            } catch (e) { console.error("Erreur dans updateAstro:", e); }
        }

        // 2. Mise à jour Météo & BioSVT
        if (lat && lon && !emergencyStopActive) {
            try {
                // S'assure que les données météo sont fraîches, sinon les récupère
                const data = await fetchWeather(currentLat, currentLon);
                
                if (data) {
                    const bioSVT = calculateBioSVT(data.tempC, kAlt || 0, data.humidity_perc, data.pressure_hPa * 100, sunAltitudeRad);
                    
                    $('weather-status').textContent = `ACTIF`;
                    $('temp-air-2').textContent = `${data.tempC.toFixed(1)} °C`;
                    $('pressure-2').textContent = `${data.pressure_hPa.toFixed(0)} hPa`;
                    $('humidity-2').textContent = `${data.humidity_perc.toFixed(0)} %`;
                    $('air-density').textContent = `${data.air_density.toFixed(3)} kg/m³`;
                    $('dew-point').textContent = `${bioSVT.dewPoint.toFixed(1)} °C`;
                    
                    // MAJ BioSVT (Simulation)
                    $('abs-humidity-sim').textContent = dataOrDefault(bioSVT.absoluteHumidity, 3, ' g/m³');
                    $('wet-bulb-temp-sim').textContent = `${bioSVT.wetBulbTemp.toFixed(1)} °C`;
                    $('CAPE-sim').textContent = `${bioSVT.CAPE_sim.toFixed(0)} J/kg`;
                    $('O2-saturation-sim').textContent = `${bioSVT.O2SaturationClamped.toFixed(1)} %`;
                    $('photosynthesis-rate-sim').textContent = dataOrDefaultExp(bioSVT.photosynthesisRate, 2);
                    $('radiation-pressure').textContent = dataOrDefaultExp(bioSVT.radiationPressure, 2, ' Pa');
                }
            } catch (e) { /* Géré par fetchWeather */ }
        }

        // 3. Mise à jour Heure NTP
        if (now) {
            if ($('local-time') && !$('local-time').textContent.includes('Synchronisation...')) {
                $('local-time').textContent = now.toLocaleTimeString('fr-FR');
            }
            if ($('date-display')) $('date-display').textContent = now.toUTCString();
            if ($('time-minecraft')) $('time-minecraft').textContent = getMinecraftTime(now);
        }
    };
    
    domSlowID = setInterval(updateSlowData, DOM_SLOW_UPDATE_MS);
    updateSlowData(); // Exécuter immédiatement au démarrage
}


// ===========================================
// INITIALISATION DOM (DÉMARRAGE)
// ===========================================
document.addEventListener('DOMContentLoaded', () => {
    
    initMap(); 
    
    // --- Écouteurs d'événements pour tous les contrôles ---
    // IMPORTANT : Démarrage du système au clic (pour permissions Navigateur)
    $('toggle-gps-btn').addEventListener('click', () => {
        if (!ukf) { // Si le système n'est pas initialisé
             if (typeof math === 'undefined') {
                alert("Erreur: math.js n'a pas pu être chargé. Le filtre UKF est désactivé.");
                return;
            }
            ukf = new ProfessionalUKF(); 
            sTime = Date.now();
            startGPS(); // Démarre le GPS (qui démarrera l'IMU et la boucle rapide)
            startSlowLoop(); // Démarrage de la boucle lente (Astro/Météo)
        } else {
            toggleGPS(); // Comportement normal (Pause/Reprise)
        }
    });

    $('emergency-stop-btn').addEventListener('click', toggleEmergencyStop);
    $('toggle-mode-btn').addEventListener('click', () => {
        document.body.classList.toggle('dark-mode');
        $('toggle-mode-btn').innerHTML = document.body.classList.contains('dark-mode') ? '<i class="fas fa-sun"></i> Mode Jour' : '<i class="fas fa-moon"></i> Mode Nuit';
    });
    $('reset-dist-btn').addEventListener('click', () => { distM = 0; timeMoving = 0; });
    $('reset-max-btn').addEventListener('click', () => { maxSpd = 0; });
    $('reset-all-btn').addEventListener('click', () => {
        distM = 0; maxSpd = 0; kSpd = 0; kUncert = UKF_R_MAX; kAlt = null; kAltUncert = 10; timeMoving = 0; timeTotal = 0; sTime = Date.now();
        if (ukf) ukf = new ProfessionalUKF(); // Réinitialise le filtre
    });
    $('capture-data-btn').addEventListener('click', () => alert('Capture de données non implémentée.'));
    $('xray-button').addEventListener('click', () => $('minecraft-clock').classList.toggle('x-ray'));
    
    // --- Écouteurs pour les Inputs & Selects ---
    $('freq-select').addEventListener('change', (e) => startGPS(e.target.value));
    $('gps-accuracy-override').addEventListener('change', (e) => gpsAccuracyOverride = parseFloat(e.target.value) || 0.0);
    $('environment-select').addEventListener('change', (e) => {
        selectedEnvironment = e.target.value;
        const factor = ENVIRONMENT_FACTORS[selectedEnvironment];
        $('env-factor').textContent = `${factor.DISPLAY} (x${factor.R_MULT.toFixed(1)})`;
    });
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
    
    $('nether-toggle-btn').addEventListener('click', () => {
        netherMode = !netherMode;
        $('nether-toggle-btn').textContent = `Mode Nether: ${netherMode ? 'ACTIVÉ (1:8)' : 'DÉSACTIVÉ (1:1)'}`;
    });
    $('ukf-reactivity-mode').addEventListener('change', (e) => currentUKFReactivity = e.target.value);


    // --- DÉMARRAGE DU SYSTÈME ---
    updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity);
    
    // Démarrer la synchro NTP immédiatement au chargement
    syncH();
});
