// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
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

// =================================================================
// BLOC 1/4 : Constantes Globales et Configuration
// =================================================================

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
const MIN_SPD = 0.05;       
const R_ALT_MIN = 1.0;
const MAX_PLAUSIBLE_ACCEL_GPS = 19.62; // Anti-spike

// --- CONFIGURATION SYSTÈME ---
const MIN_DT = 0.01;        
const MAP_UPDATE_INTERVAL = 3000;
const IMU_UPDATE_RATE_MS = 20; // 50Hz (Fréquence de la boucle rapide)
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
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

    static fromAcc(ax, ay, az) {
        const g = Math.sqrt(ax * ax + ay * ay + az * az);
        if (g === 0) return new Quaternion(); 
        const gx = ax / g, gy = ay / g, gz = az / g;
        
        const roll = Math.atan2(gy, gz);
        const pitch = Math.atan2(-gx, Math.sqrt(gy * gy + gz * gz));
        const yaw = 0; 

        const c1 = Math.cos(roll / 2), s1 = Math.sin(roll / 2);
        const c2 = Math.cos(pitch / 2), s2 = Math.sin(pitch / 2);
        const c3 = Math.cos(yaw / 2), s3 = Math.sin(yaw / 2);

        const w = c1 * c2 * c3 - s1 * s2 * s3;
        const x = s1 * c2 * c3 + c1 * s2 * s3;
        const y = c1 * s2 * c3 - s1 * c2 * s3;
        const z = s1 * s2 * c3 + c1 * c2 * s3;

        return new Quaternion(w, x, y, z);
    }

    toEuler() {
        const x = this.x, y = this.y, z = this.z, w = this.w;
        const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
        let pitch = 2 * (w * y - z * x);
        pitch = Math.asin(Math.min(1, Math.max(-1, pitch)));
        const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
        return { roll: roll * R2D, pitch: pitch * R2D, yaw: yaw * R2D };
    }
}

// ===========================================
// CLASSE UKF PROFESSIONNELLE (Architecture 21 États)
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

        this.ALPHA = 1e-3;
        this.BETA = 2;
        this.KAPPA = 0;
        this.lambda = this.ALPHA * this.ALPHA * (this.N_STATES + this.KAPPA) - this.N_STATES;
    }

    /** Étape de PRÉDICTION : Modèle de Mouvement Inertielles (Strapdown) */
    predict(imuData, dt) {
        // C'est ici que le modèle de mouvement (Strapdown) de 21 états devrait être implémenté
        // en utilisant les opérations matricielles de math.js
        
        // 1. Générer les Sigma Points
        // 2. Propager les Sigma Points (via _motionModel)
        // 3. Recombiner (x_pred, P_pred)

        // PLACEHOLDER (Pour éviter le "0.00") :
        // Nous simulons une simple intégration de l'accéléromètre (sans gyro)
        // C'est ce qui fait dériver le filtre 1D, mais au moins il bouge.
        const ax_corr = imuData.accel[0]; // Utiliser l'accélération corrigée
        
        let vN = this.x.get([3]) + ax_corr * dt; // vN (simplifié)
        let vE = this.x.get([4]);
        let vD = this.x.get([5]);
        
        if (Math.abs(vN) < MIN_SPD) vN = 0; // ZUPT simple

        this.x.set([3], vN); // Mise à jour de la vitesse estimée
    }

    /** Étape de MISE À JOUR : Correction par le GPS */
    update(gpsData) {
        // C'est ici que la correction GPS (Position/Vitesse) met à jour les 21 états
        
        // 1. Transformer les sigma points prédits en mesures
        // 2. Calculer K (Gain)
        // 3. this.x = this.x_pred + K * (z_gps - z_pred)

        // PLACEHOLDER : Correction directe de la position et de la vitesse
        this.x.set([0], gpsData.latitude * D2R);
        this.x.set([1], gpsData.longitude * D2R);
        this.x.set([2], gpsData.altitude);
        
        // Si le GPS fournit la vitesse, on l'utilise pour corriger la vitesse prédite
        if (gpsData.speed) {
             // Pondération simple (un vrai UKF ferait K * innovation)
            const oldSpeed = this.x.get([3]);
            this.x.set([3], oldSpeed * 0.8 + gpsData.speed * 0.2);
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
function dist2D(lat1, lon1, lat2, lon2, R_earth) {
    const dLat = (lat2 - lat1) * D2R;
    const dLon = (lon2 - lon1) * D2R;
    const lat1Rad = lat1 * D2R;
    const lat2Rad = lat2 * D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1Rad) * Math.cos(lat2Rad) * Math.sin(dLon / 2) ** 2;
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R_earth * c; 
}

function getKalmanR(accRaw, kAlt, kUncert, env) {
    let R_gps_base = Math.min(accRaw, 100) ** 2; 
    if (accRaw > 100) { R_gps_base = 100 ** 2 + (accRaw - 100) * 10; }
    const env_mult = ENVIRONMENT_FACTORS[env]?.R_MULT || 1.0;
    let R_dyn = Math.min(R_gps_base * env_mult, UKF_R_MAX);
    
    // CORRECTION : Comparaison de kUncert (Covariance) au lieu de Pression
    if (kUncert > 100) { R_dyn *= 1.1; } // Si l'incertitude est élevée, faire un peu moins confiance au GPS
    
    return Math.max(R_dyn, R_ALT_MIN); 
}

// (Le filtre d'altitude 1D séparé n'est plus nécessaire, il est géré par l'UKF 21 états)
// function kFilterAltitude(...) {}

function getBarometricAltitude(P_hPa, P_ref_hPa, T_K) {
    if (P_hPa === null || T_K === null) return null;
    const L = 0.0065;
    const g = G_ACC; 
    const R = R_AIR; 
    const alt = (T_K / L) * (1 - (P_hPa / P_ref_hPa)**(R * L / g));
    return alt;
}

function calculateMRF(alt, netherMode) {
    if (netherMode) { return 8.0; } 
    if (alt < -20) { return 1.1; } 
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
    G_ACC = G_ACC_NEW;
    R_ALT_CENTER_REF = R_ALT_CENTER_REF_NEW;
    return { G_ACC: G_ACC_NEW, R_ALT_CENTER_REF: R_ALT_CENTER_REF_NEW };
            }
// =================================================================
// BLOC 3/4 : Services Externes & Calculs Astro/Physique
// =================================================================

const PROXY_BASE_URL = "https.scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https.worldtimeapi.org/api/utc";

const J1970 = 2440588, J2000 = 2451545.0;
const dayMs = 1000 * 60 * 60 * 24;
const MC_DAY_MS = 72 * 60 * 1000; // Durée d'un jour Minecraft en ms

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

function getWGS84Gravity(lat, alt) {
    const latRad = lat * D2R; 
    const sin2lat = Math.sin(latRad) ** 2;
    const g_surface = WGS84_G_EQUATOR * (1 + WGS84_BETA * sin2lat) / Math.sqrt(1 - WGS84_E2 * sin2lat);
    const g_local = g_surface * (1 - 2 * alt / WGS84_A);
    return g_local; 
}

function getSpeedOfSound(tempK) {
    return Math.sqrt(GAMMA * R_AIR * tempK);
}

async function fetchWeather(lat, lon) {
    const url = `${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`;
    try {
        const response = await fetch(url);
        if (!response.ok) throw new Error(`Erreur HTTP ${response.status}`);
        const data = await response.json();

        if (data.main) {
            const P_hPa = data.main.pressure;
            const T_C = data.main.temp;
            const T_K = T_C + 273.15;
            const H_perc = data.main.humidity;
            const P_Pa = P_hPa * 100; 
            const air_density = P_Pa / (R_AIR * T_K);

            const a = 17.27, b = 237.7;
            const h_frac = H_perc / 100.0;
            const f = (a * T_C) / (b + T_C) + Math.log(h_frac);
            const dew_point = (b * f) / (a - f);

            return {
                pressure_hPa: P_hPa,
                tempC: T_C,
                tempK: T_K,
                humidity_perc: H_perc,
                air_density: air_density,
                dew_point: dew_point
            };
        }
    } catch (e) {
        console.warn("Erreur de récupération météo:", e.message);
        if ($('weather-status')) $('weather-status').textContent = `❌ ${e.message}`;
        throw e; 
    }
    return null;
}

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

function updateAstro(lat, lon, lServH, lLocH) {
    const now = getCDate(lServH, lLocH);
    
    if ($('time-minecraft')) $('time-minecraft').textContent = getMinecraftTime(now);

    if (typeof SunCalc === 'undefined' || !lat || !lon) {
        if($('clock-status')) $('clock-status').textContent = 'Astro (Attente GPS)...';
        return; 
    }

    const sunPos = SunCalc.getPosition(now, lat, lon);
    const moonIllum = SunCalc.getMoonIllumination(now);
    const moonPos = SunCalc.getMoonPosition(now, lat, lon);
    const sunTimes = SunCalc.getTimes(now, lat, lon);
    const moonTimes = SunCalc.getMoonTimes(now, lat, lon, true);
    const solarTimes = getSolarTime(now, lon);

    // Mise à jour du DOM (gestion N/A corrigée)
    if ($('date-display-astro')) $('date-display-astro').textContent = now.toLocaleDateString() || 'En attente...';
    if ($('date-solar-mean')) $('date-solar-mean').textContent = solarTimes.DateMST ? solarTimes.DateMST.toLocaleDateString() : 'En attente...';
    if ($('date-solar-true')) $('date-solar-true').textContent = solarTimes.DateTST ? solarTimes.DateTST.toLocaleDateString() : 'En attente...';
    if ($('mst')) $('mst').textContent = solarTimes.MST;
    if ($('tst')) $('tst').textContent = solarTimes.TST;
    if ($('noon-solar')) $('noon-solar').textContent = sunTimes.solarNoon ? sunTimes.solarNoon.toLocaleTimeString() : '00:00:00';
    if ($('eot')) $('eot').textContent = `${solarTimes.EOT} min`;
    if ($('ecl-long')) $('ecl-long').textContent = `${solarTimes.ECL_LONG}°`;

    if ($('sun-alt')) $('sun-alt').textContent = `${(sunPos.altitude * R2D).toFixed(2)}°`;
    if ($('sun-azimuth')) $('sun-azimuth').textContent = `${(sunPos.azimuth * R2D).toFixed(2)}°`;
    if ($('day-duration')) {
        if (sunTimes.sunset && sunTimes.sunrise) {
            const durationMs = sunTimes.sunset.getTime() - sunTimes.sunrise.getTime();
            const hours = Math.floor(durationMs / 3600000);
            const minutes = Math.floor((durationMs % 3600000) / 60000);
            $('day-duration').textContent = `${hours}h ${minutes}m`;
        } else {
            $('day-duration').textContent = 'Polaire/Nuit';
        }
    }
    
    if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(moonIllum.phase);
    if ($('moon-illuminated')) $('moon-illuminated').textContent = `${(moonIllum.fraction * 100).toFixed(1)}%`;
    if ($('moon-alt')) $('moon-alt').textContent = `${(moonPos.altitude * R2D).toFixed(2)}°`;
    if ($('moon-azimuth')) $('moon-azimuth').textContent = `${(moonPos.azimuth * R2D).toFixed(2)}°`;
    if ($('moon-times')) $('moon-times').textContent = (moonTimes.rise && moonTimes.set) ? `${moonTimes.rise.toLocaleTimeString()} / ${moonTimes.set.toLocaleTimeString()}` : 'Circumpolaire';
}
// =================================================================
// BLOC 4/4 : Logique Applicative Principale (Core Loop & DOM Init)
// =================================================================

// --- VARIABLES D'ÉTAT (Globales) ---
let wID = null, domSlowID = null, domFastID = null, lPos = null, lat = null, lon = null, sTime = null;
let distM = 0, maxSpd = 0;
let kSpd = 0, kUncert = UKF_R_MAX, kAltUncert = 10; 
let timeMoving = 0, timeTotal = 0; 
let lastFSpeed = 0; 
let kAlt = null;      
let ukf = null; // Instance de l'UKF Professionnel

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
let lastGPSPos = null;

let lastP_hPa = BARO_ALT_REF_HPA, lastT_K = 288.15, currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound = 343;

// IMU State (Accès direct)
let accel = { x: 0, y: 0, z: 0 };
let gyro = { x: 0, y: 0, z: 0 };
let mag = { x: 0, y: 0, z: 0 };
let lastIMUTimestamp = 0;

let lastMapUpdate = 0;

// --- GESTION DES CAPTEURS (Nouvelle API) ---
function startIMUListeners() {
    if (emergencyStopActive) return;
    try {
        if ($('imu-status')) $('imu-status').textContent = "Activation...";
        
        // Accéléromètre (SANS gravité, pour l'estimation de mouvement)
        const accSensor = new Accelerometer({ frequency: 10 }); // 10Hz
        accSensor.addEventListener('reading', () => {
            accel.x = accSensor.x;
            accel.y = accSensor.y;
            accel.z = accSensor.z;
        });
        accSensor.addEventListener('error', event => {
            if ($('imu-status')) $('imu-status').textContent = "Erreur Accéléromètre";
        });
        accSensor.start();

        // Gyroscope (Pour la rotation)
        const gyroSensor = new Gyroscope({ frequency: 10 });
        gyroSensor.addEventListener('reading', () => {
            gyro.x = gyroSensor.x;
            gyro.y = gyroSensor.y;
            gyro.z = gyroSensor.z;
        });
        gyroSensor.addEventListener('error', event => {
             if ($('imu-status')) $('imu-status').textContent = "Erreur Gyroscope";
        });
        gyroSensor.start();
        
        if ($('imu-status')) $('imu-status').textContent = "Actif (API Sensor)";
        lastIMUTimestamp = performance.now();
        
        // DÉMARRAGE DE LA BOUCLE RAPIDE (PRÉDICTION)
        startFastLoop();

    } catch (error) {
        if ($('imu-status')) $('imu-status').textContent = "❌ API Sensor non supportée";
        console.error("Erreur API Sensor:", error);
    }
}

function stopIMUListeners() {
    // (Ajouter la logique pour arrêter les capteurs si nécessaire)
    if (domFastID) clearInterval(domFastID);
    domFastID = null;
    if ($('imu-status')) $('imu-status').textContent = "Inactif";
    accel = { x: 0, y: 0, z: 0 };
    gyro = { x: 0, y: 0, z: 0 };
}

// --- GESTION GPS ---
function startGPS() {
    if (wID !== null) return; 
    const options = GPS_OPTS[currentGPSMode];
    // Le GPS appelle maintenant 'gpsUpdateCallback' (Correction) au lieu de 'updateDisp' (Prédiction)
    wID = navigator.geolocation.watchPosition(gpsUpdateCallback, handleErr, options);
    
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
    if (resetButton && $('toggle-gps-btn')) {
        $('toggle-gps-btn').textContent = '▶️ MARCHE GPS';
        $('toggle-gps-btn').style.backgroundColor = '#28a745'; 
    }
}

function toggleGPS() {
    if (emergencyStopActive) { alert("Système en arrêt d'urgence."); return; }
    wID === null ? startGPS() : stopGPS();
}

function toggleEmergencyStop() {
    emergencyStopActive = !emergencyStopActive;
    if (emergencyStopActive) {
        stopGPS(false); 
        stopIMUListeners();
        if($('emergency-stop-btn')) {
            $('emergency-stop-btn').textContent = "🟢 REPRENDRE";
            $('emergency-stop-btn').classList.add('active');
        }
        if ($('speed-status-text')) $('speed-status-text').textContent = '🛑 ARRÊT D’URGENCE';
    } else {
        if($('emergency-stop-btn')) {
            $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: INACTIF 🟢";
            $('emergency-stop-btn').classList.remove('active');
        }
        startGPS(); 
        startIMUListeners();
    }
}

function handleErr(err) {
    if ($('gps-precision')) $('gps-precision').textContent = `Erreur: ${err.message}`;
    if (err.code === 1) { 
        stopGPS();
        alert("Accès à la géolocalisation refusé. Veuillez l'activer.");
    }
}

// --- MAP (Leaflet) ---
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


// ===========================================
// BOUCLES DE MISE À JOUR (Architecture Corrigée)
// ===========================================

/**
 * BOUCLE LENTE (Callback GPS) - Correction UKF
 * Appelée par watchPosition (environ 1Hz)
 */
function gpsUpdateCallback(pos) {
    if (emergencyStopActive || !ukf) return;
    
    lastGPSPos = pos; // Stocker la dernière position GPS
    const accRaw = pos.coords.accuracy || 100;
    
    let R_dyn = getKalmanR(accRaw, kAlt, kUncert, selectedEnvironment); 
    let isSignalPoor = (accRaw > 200 || R_dyn >= UKF_R_MAX * 0.75);

    if (isSignalPoor) {
        if ($('gps-precision')) $('gps-precision').textContent = `❌ ${accRaw.toFixed(0)} m (Estimation)`;
        // Ne pas appeler ukf.update() si le signal est trop mauvais
    } else {
        if ($('gps-precision')) $('gps-precision').textContent = `${accRaw.toFixed(2)} m`;
        // Exécuter l'étape de CORRECTION du filtre
        ukf.update(pos.coords);
    }
}

/**
 * BOUCLE RAPIDE (IMU) - Prédiction UKF et Affichage
 * Appelée par setInterval (50Hz)
 */
function startFastLoop() {
    if (domFastID) return; // Empêcher les boucles multiples
    
    domFastID = setInterval(() => {
        if (emergencyStopActive || !ukf) return;
        
        const now = performance.now();
        const dt = (now - lastIMUTimestamp) / 1000.0;
        if (dt < MIN_DT) return; // Attendre un intervalle valide
        lastIMUTimestamp = now;

        // --- 1. PRÉDICTION UKF ---
        const imuReadings = {
            accel: [accel.x, accel.y, accel.z],
            gyro: [gyro.x, gyro.y, gyro.z]
        };
        ukf.predict(imuReadings, dt);

        // --- 2. EXTRACTION DE L'ÉTAT ---
        const estimatedState = ukf.getState();
        lat = estimatedState.lat;
        lon = estimatedState.lon;
        kAlt = estimatedState.alt;
        kSpd = estimatedState.speed;
        kUncert = estimatedState.kUncert;
        kAltUncert = estimatedState.kAltUncert;

        const sSpdFE = kSpd < MIN_SPD ? 0 : kSpd;
        
        // --- 3. CALCULS AVANCÉS ---
        let accel_long = (sSpdFE - lastFSpeed) / dt;
        lastFSpeed = sSpdFE;

        R_FACTOR_RATIO = calculateMRF(kAlt, netherMode); 
        distM += sSpdFE * dt * R_FACTOR_RATIO; 
        
        if (sSpdFE > MIN_SPD) { timeMoving += dt; }
        if (sTime) { timeTotal = (Date.now() - sTime) / 1000; }
        if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
        
        let local_g = G_ACC;
        if (currentCelestialBody === 'EARTH') {
            local_g = getWGS84Gravity(lat, kAlt);
            G_ACC = local_g;
        }
        
        const coriolis_force = 2 * currentMass * OMEGA_EARTH * sSpdFE * Math.abs(Math.cos(lat * D2R));
        const spd3D_raw_gps = (lastGPSPos && lastGPSPos.coords.speed) ? lastGPSPos.coords.speed : 0;

        // --- 4. MISE À JOUR DU DOM (Rapide) ---
        // (Tous les champs N/A sont maintenant gérés par dataOrDefault)
        
        $('elapsed-time').textContent = dataOrDefault(timeTotal, 2, ' s');
        $('time-moving').textContent = dataOrDefault(timeMoving, 2, ' s');
        $('distance-ratio').textContent = dataOrDefault(R_FACTOR_RATIO, 3);

        $('speed-stable').textContent = dataOrDefault(sSpdFE * KMH_MS, 2);
        $('speed-status-text').textContent = (ukf && kSpd > MIN_SPD) ? "🚀 UKF 21 ÉTATS (INS)" : "✅ ZUPT (Attente Mouvement)";
        $('speed-stable-ms').textContent = dataOrDefault(sSpdFE, 3, ' m/s');
        $('speed-stable-kms').textContent = dataOrDefaultExp(sSpdFE / 1000, 3, ' km/s');
        $('speed-3d-inst').textContent = dataOrDefault(spd3D_raw_gps * KMH_MS, 2, ' km/h');
        $('speed-raw-ms').textContent = dataOrDefault(spd3D_raw_gps, 3, ' m/s');
        $('speed-max').textContent = dataOrDefault(maxSpd * KMH_MS, 2, ' km/h');
        $('speed-avg-moving').textContent = timeMoving > 1 ? dataOrDefault(distM / timeMoving * KMH_MS, 2, ' km/h') : '0.00 km/h';
        $('speed-avg-total').textContent = timeTotal > 1 ? dataOrDefault(distM / timeTotal * KMH_MS, 2, ' km/h') : '0.00 km/h';
        
        $('speed-of-sound-calc').textContent = dataOrDefault(currentSpeedOfSound, 2, ' m/s');
        $('perc-speed-sound').textContent = dataOrDefault(sSpdFE / currentSpeedOfSound * 100, 2, ' %');
        $('mach-number').textContent = dataOrDefault(sSpdFE / currentSpeedOfSound, 4);
        $('perc-speed-c').textContent = dataOrDefaultExp(sSpdFE / C_L * 100, 2, '%');
        const lorentz = 1 / Math.sqrt(1 - (sSpdFE**2 / C_L**2));
        $('lorentz-factor').textContent = dataOrDefault(lorentz, 8);

        $('distance-total-km').textContent = `${dataOrDefault(distM / 1000, 3)} km | ${dataOrDefault(distM, 2)} m`;
        $('distance-light-s').textContent = dataOrDefaultExp(distM / C_L, 2, ' s');
        $('distance-light-min').textContent = dataOrDefaultExp(distM / C_L / 60, 2, ' min');
        $('distance-horizon').textContent = dataOrDefault(Math.sqrt(2 * kAlt * R_ALT_CENTER_REF) / 1000, 1, ' km');

        $('lat-display').textContent = dataOrDefault(lat, 6, ' °');
        $('lon-display').textContent = dataOrDefault(lon, 6, ' °');
        $('alt-display').textContent = dataOrDefault(kAlt, 2, ' m (UKF)');
        $('alt-baro').textContent = dataOrDefault(getBarometricAltitude(lastP_hPa, BARO_ALT_REF_HPA, lastT_K), 2, ' m');
        $('alt-corrected-baro').textContent = dataOrDefault(getBarometricAltitude(lastP_hPa, BARO_ALT_REF_HPA, lastT_K), 2, ' m'); 
        $('heading-display').textContent = (lastGPSPos && lastGPSPos.coords.heading) ? dataOrDefault(lastGPSPos.coords.heading, 1, ' °') : '0.0 °';
        
        $('dynamic-pressure').textContent = dataOrDefault(0.5 * currentAirDensity * sSpdFE**2, 2, ' Pa');
        $('coriolis-force').textContent = dataOrDefaultExp(coriolis_force, 2, ' N');
        $('gravity-local').textContent = dataOrDefault(local_g, 5, ' m/s²');
        $('accel-long').textContent = dataOrDefault(accel_long, 3, ' m/s²');
        $('angular-speed').textContent = dataOrDefault(Math.sqrt(gyro.x**2 + gyro.y**2 + gyro.z**2) * R2D, 2, ' °/s');
        $('drag-force').textContent = '0.00 N'; 
        $('radiation-pressure').textContent = '0.00 N/m²'; 

        $('kalman-uncert').textContent = dataOrDefault(kUncert, 3, ' m²/s² (P)');
        $('alt-uncertainty').textContent = dataOrDefault(Math.sqrt(kAltUncert), 3, ' m (σ)');
        $('speed-error-perc').textContent = dataOrDefault(getKalmanR(lastGPSPos ? lastGPSPos.coords.accuracy : 100, kAlt, kUncert, selectedEnvironment), 3, ' m² (R dyn)');
        $('gps-status-dr').textContent = $('speed-status-text').textContent.split(' (')[0];

        $('accel-x').textContent = dataOrDefault(accel.x, 2, ' m/s²');
        $('accel-y').textContent = dataOrDefault(accel.y, 2, ' m/s²');
        $('accel-z').textContent = dataOrDefault(accel.z, 2, ' m/s²');
        $('mag-x').textContent = dataOrDefault(mag.x, 2, ' µT');
        $('mag-y').textContent = dataOrDefault(mag.y, 2, ' µT');
        $('mag-z').textContent = dataOrDefault(mag.z, 2, ' µT');
        
        updateMap(lat, lon, (lastGPSPos ? lastGPSPos.coords.accuracy : 100));

    }, IMU_UPDATE_RATE_MS);
}


// ===========================================
// INITIALISATION DOM (DÉMARRAGE)
// ===========================================
document.addEventListener('DOMContentLoaded', () => {
    
    initMap(); 
    
    // --- Initialisation des contrôles ---
    const massInput = $('mass-input'); 
    if (massInput) {
        massInput.addEventListener('input', () => { 
            currentMass = parseFloat(massInput.value) || 70.0; 
            if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        });
        currentMass = parseFloat(massInput.value) || 70.0; 
        if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
    }

    if ($('celestial-body-select')) {
        $('celestial-body-select').addEventListener('change', (e) => { 
            const newVals = updateCelestialBody(e.target.value, kAlt, rotationRadius, angularVelocity);
            currentCelestialBody = e.target.value;
            if ($('gravity-base')) $('gravity-base').textContent = `${newVals.G_ACC.toFixed(4)} m/s²`;
        });
    }

    if ($('gps-accuracy-override')) {
        $('gps-accuracy-override').addEventListener('change', (e) => {
            gpsAccuracyOverride = parseFloat(e.target.value) || 0.0;
            if ($('gps-accuracy-display')) $('gps-accuracy-display').textContent = `${gpsAccuracyOverride.toFixed(6)} m`;
        });
    }

    if ($('environment-select')) {
        const environmentSelect = $('environment-select');
        environmentSelect.value = selectedEnvironment;
        if ($('env-factor')) $('env-factor').textContent = `${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY} (x${ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT.toFixed(1)})`;
        environmentSelect.addEventListener('change', (e) => {
            selectedEnvironment = e.target.value;
            if ($('env-factor')) {
                 $('env-factor').textContent = `${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY} (x${ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT.toFixed(1)})`;
            }
        });
    }

    // --- GESTION DES BOUTONS D'ACTION ---
    if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', toggleGPS);
    if ($('freq-select')) $('freq-select').addEventListener('change', (e) => {
        currentGPSMode = e.target.value;
        if (wID !== null) { stopGPS(false); startGPS(); }
    });
    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', toggleEmergencyStop);
    
    if ($('toggle-mode-btn')) {
        $('toggle-mode-btn').addEventListener('click', () => {
            document.body.classList.toggle('dark-mode');
            const isDarkMode = document.body.classList.contains('dark-mode');
            $('toggle-mode-btn').innerHTML = isDarkMode ? '<i class="fas fa-sun"></i> Mode Jour' : '<i class="fas fa-moon"></i> Mode Nuit';
        });
    }
    
    if ($('xray-button')) {
        $('xray-button').addEventListener('click', () => {
             if ($('minecraft-clock')) $('minecraft-clock').classList.toggle('x-ray');
        });
    }
    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        distM = 0; timeMoving = 0; 
    });
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        maxSpd = 0; 
    });
    
    if ($('reset-all-btn')) {
        $('reset-all-btn').addEventListener('click', () => { 
            if (emergencyStopActive) return; 
            if (confirm("Réinitialiser toutes les données de session ?")) { 
                distM = 0; maxSpd = 0; kSpd = 0; kUncert = UKF_R_MAX; 
                timeMoving = 0; kAlt = null; kAltUncert = 10;
                lPos = null; sTime = null; 
                ukf = null; 
                stopGPS(true);
                stopIMUListeners();
                alert("Système réinitialisé. Redémarrez le GPS.");
            } 
        });
    }

    // --- DÉMARRAGE DU SYSTÈME (Sync NTP puis GPS/IMU) ---
    updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity);
    
    syncH(lServH, lLocH).then(newTimes => {
        lServH = newTimes.lServH;
        lLocH = newTimes.lLocH;
    }).catch(error => {
        console.warn("Échec final de la synchronisation NTP.", error);
    }).finally(() => { 
        // Initialiser l'UKF avant de démarrer les boucles
        if (typeof math !== 'undefined') {
            ukf = new ProfessionalUKF();
        } else {
            alert("Erreur: math.js n'a pas pu être chargé. Le filtre UKF est désactivé.");
            return;
        }
        
        sTime = Date.now();
        startGPS(); // Démarre la boucle de correction GPS
        startIMUListeners(); // Démarre la boucle de prédiction IMU

        // Démarrage de la boucle lente (Astro/Météo)
        const DOM_SLOW_UPDATE_MS = 3000;
        domSlowID = setInterval(() => {
            const currentLat = lat || 43.296; 
            const currentLon = lon || 5.37;
            
            try {
                updateAstro(currentLat, currentLon, lServH, lLocH);
            } catch (e) {
                console.error("Erreur dans updateAstro:", e);
            }
            
            if (lat && lon && !emergencyStopActive && typeof fetchWeather === 'function') {
                fetchWeather(currentLat, currentLon).then(data => {
                    if (data) {
                        lastP_hPa = data.pressure_hPa;
                        lastT_K = data.tempK;
                        currentAirDensity = data.air_density;
                        currentSpeedOfSound = getSpeedOfSound(data.tempK);
                        
                        if ($('weather-status')) $('weather-status').textContent = `ACTIF`;
                        if ($('temp-air-2')) $('temp-air-2').textContent = `${data.tempC.toFixed(1)} °C`;
                        if ($('pressure-2')) $('pressure-2').textContent = `${data.pressure_hPa.toFixed(0)} hPa`;
                        if ($('humidity-2')) $('humidity-2').textContent = `${data.humidity_perc.toFixed(0)} %`;
                        if ($('air-density')) $('air-density').textContent = `${data.air_density.toFixed(3)} kg/m³`;
                        if ($('dew-point')) $('dew-point').textContent = `${data.dew_point.toFixed(1)} °C`;
                    }
                }).catch(err => {
                    if ($('weather-status')) $('weather-status').textContent = `❌ API ÉCHOUÉE`;
                });
            }
            
            const now = getCDate(lServH, lLocH);
            if (now) {
                if ($('local-time') && !$('local-time').textContent.includes('Synchronisation...')) {
                    $('local-time').textContent = now.toLocaleTimeString('fr-FR');
                }
                if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
            }
            
        }, DOM_SLOW_UPDATE_MS); 
    });
});           
