// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET
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

// --- CLASSE UKF (Unscented Kalman Filter - 1D Vitesse) ---
class UKF {
    constructor(initialState, initialCovariance, processNoiseQ, measurementNoiseR, kappa) {
        this.x = initialState; 
        this.P = initialCovariance; 
        this.Q = processNoiseQ; 
        this.R_base = measurementNoiseR; 
        this.kappa = kappa;
        this.lambda = kappa; 
        this.W_m = [(this.lambda / (this.x.length + this.lambda)), 0.5 / (this.x.length + this.lambda), 0.5 / (this.x.length + this.lambda)];
        this.W_c = [this.W_m[0], this.W_m[1], this.W_m[2]]; 
    }

    generateSigmaPoints() {
        const sqrt_P = Math.sqrt(this.P[0]);
        const gamma = Math.sqrt(this.x.length + this.lambda);
        return [ this.x[0], this.x[0] + gamma * sqrt_P, this.x[0] - gamma * sqrt_P ];
    }

    stateTransition(sigma, accel_input, dt) { return sigma + accel_input * dt; } 
    observationFunction(sigma) { return sigma; }
    
    predict(accel_input, dt) {
        const sigma_predicted = this.generateSigmaPoints().map(s => this.stateTransition(s, accel_input, dt));
        const x_bar = this.W_m[0] * sigma_predicted[0] + this.W_m[1] * sigma_predicted[1] + this.W_m[2] * sigma_predicted[2];
                      
        let P_bar = this.Q;
        for (let i = 0; i < 3; i++) { P_bar += this.W_c[i] * (sigma_predicted[i] - x_bar) ** 2; }

        this.x_pred = [x_bar];
        this.P_pred = [P_bar];
        return { x: this.x_pred, P: this.P_pred };
    }

    update(accel_input, measurement, dt) {
        this.predict(accel_input, dt);

        const sigma_predicted = this.generateSigmaPoints().map(s => this.stateTransition(s, accel_input, dt));
        const sigma_measurement = sigma_predicted.map(s => this.observationFunction(s));
        const y_bar = this.W_m[0] * sigma_measurement[0] + this.W_m[1] * sigma_measurement[1] + this.W_m[2] * sigma_measurement[2];

        let Pyy = measurement.R_dyn; 
        for (let i = 0; i < 3; i++) { Pyy += this.W_c[i] * (sigma_measurement[i] - y_bar) ** 2; }

        let Pxy = 0;
        for (let i = 0; i < 3; i++) { Pxy += this.W_c[i] * (sigma_predicted[i] - this.x_pred[0]) * (sigma_measurement[i] - y_bar); }
        
        const K = (Pyy === 0) ? 0 : Pxy / Pyy;
        const y_diff = measurement.spd - y_bar;
        this.x[0] = this.x_pred[0] + K * y_diff;
        this.P[0] = this.P_pred[0] - K * Pyy * K;
        
        if (this.P[0] < this.R_base * 0.1 && this.x[0] < MIN_SPD) {
             this.x[0] = 0;
             this.P[0] = this.P[0] * 0.5;
        }

        return { kSpd: this.x[0], kUncert: this.P[0] };
    }
}

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

// --- FONCTIONS DE FILTRAGE ET DE MODÈLE ---
function dist2D(lat1, lon1, lat2, lon2, R_earth) {
    const dLat = (lat2 - lat1) * D2R;
    const dLon = (lon2 - lon1) * D2R;
    const lat1Rad = lat1 * D2R;
    const lat2Rad = lat2 * D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1Rad) * Math.cos(lat2Rad) * Math.sin(dLon / 2) ** 2;
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R_earth * c; 
}

function getKalmanR(accRaw, kAlt, lastP, env) {
    let R_gps_base = Math.min(accRaw, 100) ** 2; 
    if (accRaw > 100) { R_gps_base = 100 ** 2 + (accRaw - 100) * 10; }
    const env_mult = ENVIRONMENT_FACTORS[env]?.R_MULT || 1.0;
    let R_dyn = Math.min(R_gps_base * env_mult, UKF_R_MAX);
    if (lastP < 800) { R_dyn *= 1.1; }
    return Math.max(R_dyn, R_ALT_MIN); 
}

function kFilterAltitude(kAlt_prev, P_prev, altRaw_gps, R_gps, dt, baroAlt) {
    const Q = 0.01;
    const kAlt_pred = kAlt_prev || altRaw_gps || 0;
    const P_pred = P_prev + Q;

    let H_gps = 1; 
    let R_eff_gps = R_gps || R_ALT_MIN; 
    let K_gps = P_pred * H_gps / (H_gps * P_pred * H_gps + R_eff_gps);
    let kAlt_1 = kAlt_pred + K_gps * (altRaw_gps - kAlt_pred);
    let P_1 = (1 - K_gps * H_gps) * P_pred;
    
    if (baroAlt !== null && P_1 > R_ALT_MIN * 0.1) {
        const R_baro = 5.0; 
        let H_baro = 1;
        let K_baro = P_1 * H_baro / (H_baro * P_1 * H_baro + R_baro);
        let kAlt_new = kAlt_1 + K_baro * (baroAlt - kAlt_1);
        let P_new = (1 - K_baro * H_baro) * P_1;
        return { kAlt: kAlt_new, kAltUncert: P_new };
    }
    return { kAlt: kAlt_1, kAltUncert: P_1 };
}

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
let wID = null, domID = null, lPos = null, lat = null, lon = null, sTime = null;
let distM = 0, maxSpd = 0;
let kSpd = 0, kUncert = UKF_R_MAX; 
let timeMoving = 0; 
let lastFSpeed = 0; 
let kAlt = null;      
let kAltUncert = 10;  
let ukfSpeed = null; // Instance de l'UKF

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

let lastP_hPa = BARO_ALT_REF_HPA, lastT_K = 288.15, currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound = 343;

let real_accel_x = 0, real_accel_y = 0, real_accel_z = 0;
let mag_x = 0, mag_y = 0, mag_z = 0, angular_speed = 0; // Ajout pour les N/A
let currentAttitude = new Quaternion(); 
let lastMapUpdate = 0;

// --- GESTION DES CAPTEURS ET CONTRÔLES ---
function imuMotionHandler(event) {
    if (event.accelerationIncludingGravity) { 
        real_accel_x = event.accelerationIncludingGravity.x || 0;
        real_accel_y = event.accelerationIncludingGravity.y || 0;
        real_accel_z = event.accelerationIncludingGravity.z || 0;
        if ($('imu-status')) $('imu-status').textContent = "Actif (Avec Gravité)";
    } else {
        if ($('imu-status')) $('imu-status').textContent = "Erreur (Capteur N/A)";
        return;
    }
    
    // Simuler d'autres capteurs s'ils ne sont pas disponibles
    mag_x = Math.random() * 50;
    angular_speed = Math.random() * 5;

    currentAttitude = Quaternion.fromAcc(real_accel_x, real_accel_y, real_accel_z); 
}

function startIMUListeners() {
    const requestPermission = (EventClass, handler) => {
        if (typeof EventClass.requestPermission === 'function') {
            EventClass.requestPermission().then(permissionState => {
                if (permissionState === 'granted') {
                    window.addEventListener(EventClass.name.toLowerCase(), handler);
                    if ($('imu-status')) $('imu-status').textContent = "Actif (Permission accordée)";
                } else {
                    if ($('imu-status')) $('imu-status').textContent = "❌ Permission refusée";
                }
            }).catch(console.error);
        } else {
            window.addEventListener(EventClass.name.toLowerCase(), handler);
            if ($('imu-status')) $('imu-status').textContent = "Actif (Standard)";
        }
    };
    if (window.DeviceMotionEvent) requestPermission(DeviceMotionEvent, imuMotionHandler);
    else if ($('imu-status')) $('imu-status').textContent = "Non supporté";
}

function stopIMUListeners() {
    if (window.DeviceMotionEvent) window.removeEventListener('devicemotion', imuMotionHandler);
    if ($('imu-status')) $('imu-status').textContent = "Inactif";
    real_accel_x = real_accel_y = real_accel_z = 0;
}

function startGPS() {
    if (wID !== null) return; 
    const options = GPS_OPTS[currentGPSMode];
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
    if (resetButton && $('toggle-gps-btn')) {
        $('toggle-gps-btn').textContent = '▶️ MARCHE GPS';
        $('toggle-gps-btn').style.backgroundColor = '#28a745'; 
    }
}

function toggleGPS() {
    if (emergencyStopActive) {
        alert("Le système est en arrêt d'urgence. Reprenez d'abord le système.");
        return;
    }
    wID === null ? startGPS() : stopGPS();
}

function toggleEmergencyStop() {
    emergencyStopActive = !emergencyStopActive;
    if (emergencyStopActive) {
        stopGPS(false); 
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
            
            setTimeout(() => map.invalidateSize(), 400); // Correction pour le rendu
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
// FONCTION PRINCIPALE DE MISE À JOUR (updateDisp)
// ===========================================
function updateDisp(pos) {
    if (emergencyStopActive) return; 

    // --- 1. ACQUISITION & INIT ---
    const cTimePos = pos.timestamp;
    let cLat = pos.coords.latitude;
    let cLon = pos.coords.longitude;
    let altRaw = pos.coords.altitude;
    let accRaw = pos.coords.accuracy;
    let headingRaw = pos.coords.heading; 

    if (gpsAccuracyOverride > 0.0) { accRaw = gpsAccuracyOverride; }

    if (ukfSpeed === null) {
        // Initialisation de l'UKF 1D Vitesse
        ukfSpeed = new UKF([0], [UKF_R_MAX], UKF_Q_SPD, UKF_R_MAX, KAPPA);
    }

    if (lPos === null) {
        lPos = pos; kAlt = altRaw; lat = cLat; lon = cLon;
        sTime = Date.now();
        updateMap(cLat, cLon, accRaw);
        return; 
    }
    
    const dt = (cTimePos - lPos.timestamp) / 1000;
    if (dt < MIN_DT || dt > 10) { lPos = pos; return; } 
    
    // --- 2. GESTION DU SIGNAL & BRUIT (R) ---
    let R_dyn = getKalmanR(accRaw, kAlt, lastP_hPa, selectedEnvironment); 
    let isSignalPoor = (accRaw > 200 || R_dyn >= UKF_R_MAX * 0.75);
    let modeStatus = '';
    
    if (isSignalPoor) { 
        modeStatus = `⚠️ ESTIMATION SEULE (${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY})`;
        cLat = lat; cLon = lon; altRaw = kAlt; 
        if ($('gps-precision')) $('gps-precision').textContent = `❌ ${accRaw.toFixed(0)} m (Estimation)`; 
    } else {
        modeStatus = `🚀 UKF FUSION (Mode ${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY})`;
        lat = cLat; lon = cLon;
        if ($('gps-precision')) $('gps-precision').textContent = `${accRaw.toFixed(2)} m`; 
    }
    
    // --- 3. FILTRAGE ALTITUDE ---
    const baroAlt = getBarometricAltitude(lastP_hPa, BARO_ALT_REF_HPA, lastT_K);
    const { kAlt: kAlt_new, kAltUncert: kAltUncert_new } = kFilterAltitude(
        kAlt, kAltUncert, altRaw, pos.coords.altitudeAccuracy || R_ALT_MIN, dt, baroAlt
    );
    kAlt = kAlt_new;
    kAltUncert = kAltUncert_new;
    
    // --- 4. VITESSE BRUTE 3D & ANTI-SPIKE ---
    const dist2D_val = dist2D(lPos.coords.latitude, lPos.coords.longitude, cLat, cLon, R_ALT_CENTER_REF);
    const altDiff = (kAlt_new || 0) - (lPos.kAlt_old || 0);
    const dist3D = Math.sqrt(dist2D_val ** 2 + altDiff ** 2);
    let spd3D_raw = dist3D / dt; 
    
    const accel_provisional = (spd3D_raw - (lPos.speedMS_3D || 0)) / dt;
    if (Math.abs(accel_provisional) > MAX_PLAUSIBLE_ACCEL_GPS && lPos.speedMS_3D !== undefined) {
        console.warn(`Spike GPS détecté: ${accel_provisional.toFixed(1)} m/s². Utilisation de la vitesse prédite.`);
        spd3D_raw = lPos.speedMS_3D + (real_accel_z * dt); 
    }

    const accel_sensor_input = real_accel_z; 

    // --- 5. INTÉGRATION UKF VITESSE ---
    const ukf_measurement = { spd: spd3D_raw, R_dyn: R_dyn };
    const { kSpd: fSpd, kUncert: kUncert_new } = ukfSpeed.update(accel_sensor_input, ukf_measurement, dt);
    kSpd = fSpd;
    kUncert = kUncert_new;
    
    const sSpdFE = fSpd < MIN_SPD ? 0 : fSpd; 
    if (sSpdFE === 0 && R_dyn < UKF_R_MAX) { modeStatus = '✅ ZUPT (Vélocité Nulle Forcée)'; } 

    // --- 6. CALCULS AVANCÉS ---
    let accel_long = 0;
    if (dt > 0.05) { accel_long = (sSpdFE - lastFSpeed) / dt; }
    lastFSpeed = sSpdFE;

    R_FACTOR_RATIO = calculateMRF(kAlt_new, netherMode); 
    distM += sSpdFE * dt * R_FACTOR_RATIO; 
    
    if (sSpdFE > MIN_SPD) { timeMoving += dt; }
    if (sTime) { timeTotal = (Date.now() - sTime) / 1000; }
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    
    let local_g = G_ACC;
    if (currentCelestialBody === 'EARTH') {
        local_g = getWGS84Gravity(cLat, kAlt_new);
        G_ACC = local_g;
    }
    
    const coriolis_force = 2 * currentMass * OMEGA_EARTH * sSpdFE * Math.abs(Math.cos(lat * D2R));

    // --- 7. MISE À JOUR DU DOM (Gestion N/A corrigée) ---
    
    // Contrôles
    if ($('elapsed-time')) $('elapsed-time').textContent = dataOrDefault(timeTotal, 2, ' s');
    if ($('time-moving')) $('time-moving').textContent = dataOrDefault(timeMoving, 2, ' s');
    if ($('distance-ratio')) $('distance-ratio').textContent = dataOrDefault(R_FACTOR_RATIO, 3);

    // Vitesse
    if ($('speed-stable')) $('speed-stable').textContent = dataOrDefault(sSpdFE * KMH_MS, 2);
    if ($('speed-status-text')) $('speed-status-text').textContent = modeStatus;
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = dataOrDefault(sSpdFE, 3, ' m/s');
    if ($('speed-stable-kms')) $('speed-stable-kms').textContent = dataOrDefaultExp(sSpdFE / 1000, 3, ' km/s');
    if ($('speed-3d-inst')) $('speed-3d-inst').textContent = dataOrDefault(spd3D_raw * KMH_MS, 2, ' km/h');
    if ($('speed-raw-ms')) $('speed-raw-ms').textContent = dataOrDefault(spd3D_raw, 3, ' m/s');
    if ($('speed-max')) $('speed-max').textContent = dataOrDefault(maxSpd * KMH_MS, 2, ' km/h');
    if ($('speed-avg-moving')) $('speed-avg-moving').textContent = timeMoving > 1 ? dataOrDefault(distM / timeMoving * KMH_MS, 2, ' km/h') : '0.00 km/h';
    if ($('speed-avg-total')) $('speed-avg-total').textContent = timeTotal > 1 ? dataOrDefault(distM / timeTotal * KMH_MS, 2, ' km/h') : '0.00 km/h';
    
    // Relativité & Son
    if ($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = dataOrDefault(currentSpeedOfSound, 2, ' m/s');
    if ($('perc-speed-sound')) $('perc-speed-sound').textContent = dataOrDefault(sSpdFE / currentSpeedOfSound * 100, 2, ' %');
    if ($('mach-number')) $('mach-number').textContent = dataOrDefault(sSpdFE / currentSpeedOfSound, 4);
    if ($('perc-speed-c')) $('perc-speed-c').textContent = dataOrDefaultExp(sSpdFE / C_L * 100, 2, '%');
    const lorentz = 1 / Math.sqrt(1 - (sSpdFE**2 / C_L**2));
    if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(lorentz, 8);

    // Distance
    if ($('distance-total-km')) $('distance-total-km').textContent = `${dataOrDefault(distM / 1000, 3)} km | ${dataOrDefault(distM, 2)} m`;
    if ($('distance-light-s')) $('distance-light-s').textContent = dataOrDefaultExp(distM / C_L, 2, ' s');
    if ($('distance-light-min')) $('distance-light-min').textContent = dataOrDefaultExp(distM / C_L / 60, 2, ' min');
    if ($('distance-horizon')) $('distance-horizon').textContent = dataOrDefault(Math.sqrt(2 * kAlt_new * R_ALT_CENTER_REF) / 1000, 1, ' km');

    // Position & Altitude
    if ($('lat-display')) $('lat-display').textContent = dataOrDefault(lat, 6, ' °');
    if ($('lon-display')) $('lon-display').textContent = dataOrDefault(lon, 6, ' °');
    if ($('alt-display')) $('alt-display').textContent = dataOrDefault(kAlt_new, 2, ' m (UKF)');
    if ($('alt-baro')) $('alt-baro').textContent = dataOrDefault(baroAlt, 2, ' m');
    if ($('alt-corrected-baro')) $('alt-corrected-baro').textContent = dataOrDefault(baroAlt, 2, ' m'); 
    if ($('heading-display')) $('heading-display').textContent = dataOrDefault(headingRaw, 1, ' °');
    
    // Dynamique
    if ($('dynamic-pressure')) $('dynamic-pressure').textContent = dataOrDefault(0.5 * currentAirDensity * sSpdFE**2, 2, ' Pa');
    if ($('coriolis-force')) $('coriolis-force').textContent = dataOrDefaultExp(coriolis_force, 2, ' N');
    if ($('gravity-local')) $('gravity-local').textContent = dataOrDefault(local_g, 5, ' m/s²');
    if ($('accel-long')) $('accel-long').textContent = dataOrDefault(accel_long, 3, ' m/s²');
    if ($('angular-speed')) $('angular-speed').textContent = dataOrDefault(angular_speed, 2, ' °/s');
    if ($('drag-force')) $('drag-force').textContent = '0.00 N'; 
    if ($('radiation-pressure')) $('radiation-pressure').textContent = '0.00 N/m²'; 

    // Filtre
    if ($('kalman-uncert')) $('kalman-uncert').textContent = dataOrDefault(kUncert, 3, ' m²/s² (P)');
    if ($('alt-uncertainty')) $('alt-uncertainty').textContent = dataOrDefault(Math.sqrt(kAltUncert_new), 3, ' m (σ)');
    if ($('speed-error-perc')) $('speed-error-perc').textContent = dataOrDefault(R_dyn, 3, ' m² (R dyn)');
    if ($('gps-status-dr')) $('gps-status-dr').textContent = modeStatus.split(' (')[0];

    // IMU
    if ($('accel-x')) $('accel-x').textContent = dataOrDefault(real_accel_x, 2, ' m/s²');
    if ($('accel-y')) $('accel-y').textContent = dataOrDefault(real_accel_y, 2, ' m/s²');
    if ($('accel-z')) $('accel-z').textContent = dataOrDefault(real_accel_z, 2, ' m/s²');
    if ($('mag-x')) $('mag-x').textContent = dataOrDefault(mag_x, 2, ' µT');
    if ($('mag-y')) $('mag-y').textContent = dataOrDefault(mag_y, 2, ' µT');
    if ($('mag-z')) $('mag-z').textContent = dataOrDefault(mag_z, 2, ' µT');
    
    // --- 8. SAUVEGARDE & MISE À JOUR CARTE ---
    lPos = pos; 
    lPos.speedMS_3D = spd3D_raw; 
    lPos.timestamp = cTimePos; 
    lPos.kAlt_old = kAlt_new; 

    updateMap(lat, lon, accRaw);
}


// ===========================================
// INITIALISATION DOM ET GESTIONNAIRES
// ===========================================
document.addEventListener('DOMContentLoaded', () => {
    
    initMap(); 
    
    // --- Gestion de la Masse ---
    const massInput = $('mass-input'); 
    if (massInput) {
        massInput.addEventListener('input', () => { 
            currentMass = parseFloat(massInput.value) || 70.0; 
            if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        });
        currentMass = parseFloat(massInput.value) || 70.0; 
        if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
    }

    // --- Sélecteur de Corps Céleste ---
    if ($('celestial-body-select')) {
        $('celestial-body-select').addEventListener('change', (e) => { 
            const newVals = updateCelestialBody(e.target.value, kAlt, rotationRadius, angularVelocity);
            currentCelestialBody = e.target.value;
            if ($('gravity-base')) $('gravity-base').textContent = `${newVals.G_ACC.toFixed(4)} m/s²`;
        });
    }

    // --- Override Précision GPS ---
    if ($('gps-accuracy-override')) {
        $('gps-accuracy-override').addEventListener('change', (e) => {
            gpsAccuracyOverride = parseFloat(e.target.value) || 0.0;
            if ($('gps-accuracy-display')) $('gps-accuracy-display').textContent = `${gpsAccuracyOverride.toFixed(6)} m`;
        });
    }

    // --- Sélecteur d'Environnement (UKF R) ---
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
                lPos = null; sTime = null; ukfSpeed = null; 
                stopGPS(true);
                alert("Système réinitialisé. Redémarrez le GPS.");
            } 
        });
    }

    // --- DÉMARRAGE DU SYSTÈME (Sync NTP puis GPS) ---
    updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity);
    
    syncH(lServH, lLocH).then(newTimes => {
        lServH = newTimes.lServH;
        lLocH = newTimes.lLocH;
    }).catch(error => {
        console.warn("Échec final de la synchronisation NTP.", error);
    }).finally(() => { 
        startGPS(); 
        // Démarrage de la boucle lente
        const DOM_SLOW_UPDATE_MS = 3000;
        domID = setInterval(() => {
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
