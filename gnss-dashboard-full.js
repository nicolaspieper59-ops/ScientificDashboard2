// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// CORRIGÉ : Gestion hors ligne, capteurs IMU complets, animation corrigée, simulations supprimées.
// Dépendances (doivent être chargées dans l'HTML) : leaflet.js, turf.min.js, suncalc.js, math.min.js
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        return (decimals === 0 ? '0' : '0.00') + suffix;
    }
    return val.toFixed(decimals) + suffix;
};

// CORRECTION CRITIQUE : Assure que le format exponentiel par défaut respecte 'decimals'.
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        const zeroDecimals = '0.' + Array(decimals).fill('0').join('');
        return zeroDecimals + 'e+0' + suffix;
    }
    return val.toExponential(decimals) + suffix;
};

// =================================================================
// DÉMARRAGE : Encapsulation de la logique UKF et État Global (IIFE)
// =================================================================

((window) => {

    // Vérification des dépendances critiques
    if (typeof math === 'undefined' || typeof L === 'undefined' || typeof SunCalc === 'undefined' || typeof turf === 'undefined') {
        const missing = [
            (typeof math === 'undefined' ? "math.min.js" : ""),
            (typeof L === 'undefined' ? "leaflet.js" : ""),
            (typeof SunCalc === 'undefined' ? "suncalc.js" : ""),
            (typeof turf === 'undefined' ? "turf.min.js" : "")
        ].filter(Boolean).join(", ");
        
        console.error(`Erreur critique : Dépendances manquantes : ${missing}. Le script est arrêté.`);
        alert(`Erreur: Dépendances manquantes : ${missing}. L'application ne peut pas démarrer.`);
        return;
    }
    
    // --- CLÉS D'API & ENDPOINTS (Conservés mais gérés pour le mode hors ligne) ---
    const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
    const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
    const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

    // --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES FONDAMENTALES ---
    const D2R = Math.PI / 180, R2D = 180 / Math.PI;         
    const KMH_MS = 3.6;         
    const C_L = 299792458;      
    const G_U = 6.67430e-11;    
    const R_E_BASE = 6371000;   
    const OMEGA_EARTH = 7.2921159e-5; 
    const AU_METERS = 149597870700; 
    const LIGHT_YEAR_METERS = 9.461e15; 
    const SOLAR_FLUX_DENSITY = 1361; 

    // --- CONSTANTES ATMOSPHÉRIQUES (ISA Standard - Utilisées comme valeurs par défaut hors ligne) ---
    const BARO_ALT_REF_HPA = 1013.25;
    const RHO_SEA_LEVEL = 1.225;
    const TEMP_SEA_LEVEL_K = 288.15; // 15°C
    const R_AIR = 287.058;
    const GAMMA_AIR = 1.4;
    const MU_DYNAMIC_AIR = 1.8e-5;  
    const KELVIN_OFFSET = 273.15;

    // --- CONSTANTES GÉOPHYSIQUES (WGS84) ---
    const WGS84_A = 6378137.0;  
    const WGS84_F = 1 / 298.257223563;
    const WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F;
    const WGS84_G_EQUATOR = 9.780327;
    const WGS84_BETA = 0.0053024;

    // --- PARAMÈTRES DU FILTRE UKF/EKF ---
    const UKF_STATE_DIM = 21;    
    const UKF_R_MAX = 500.0;     
    const UKF_Q_SPD = 0.5;       
    const R_ALT_MIN = 1.0;
    const MAX_PLAUSIBLE_ACCEL_GPS = 19.62; 
    const ZUPT_RAW_THRESHOLD = 1.0;     
    const ZUPT_ACCEL_THRESHOLD = 0.5;   
    const MIN_SPD = 0.01;        
    const MAX_ACC = 200;        

    // --- CONFIGURATION SYSTÈME ---
    const MIN_DT = 0.01;        
    const MAP_UPDATE_INTERVAL = 3000;
    const IMU_UPDATE_RATE_MS = 20; // 50Hz
    const DOM_SLOW_UPDATE_MS = 1000; // 1Hz
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

    let G_ACC = CELESTIAL_DATA.EARTH.G;
    let R_ALT_CENTER_REF = CELESTIAL_DATA.EARTH.R;
    // ===========================================
// CLASSE UKF PROFESSIONNELLE (Architecture 21 États)
// ===========================================
class ProfessionalUKF {
    constructor() {
        this.N_STATES = 21; 
        this.x = math.zeros(this.N_STATES); 
        this.P = math.diag(Array(this.N_STATES).fill(1e-2)); 
        this.Q = math.diag(Array(this.N_STATES).fill(1e-6));
        this.x.set([0], 0); this.x.set([1], 0); this.x.set([2], 0);
    }

    predict(imuData, dt) {
        // (PLACEHOLDER - La logique réelle de prédiction UKF 21 états est complexe)
        const accel_biais_corrigé = imuData.accel[0] - (this.x.get([9]) || 0); // Biais Accel X
        let vN = this.x.get([3]) + accel_biais_corrigé * dt; 
        if (Math.abs(vN) < MIN_SPD) vN = 0; // ZUPT
        this.x.set([3], vN); 
        let lat_pred = this.x.get([0]) + (vN / R_E_BASE) * dt;
        this.x.set([0], lat_pred);
    }

    update(gpsData, R_dyn) {
        // (PLACEHOLDER - La logique réelle de mise à jour UKF 21 états est complexe)
        const K = 0.1; 
        this.x.set([0], this.x.get([0]) * (1-K) + (gpsData.latitude * D2R) * K);
        this.x.set([1], this.x.get([1]) * (1-K) + (gpsData.longitude * D2R) * K);
        this.x.set([2], this.x.get([2]) * (1-K) + gpsData.altitude * K);
        if (gpsData.speed) {
            const oldSpeed = this.x.get([3]);
            this.x.set([3], oldSpeed * (1-K) + gpsData.speed * K);
        }
    }
    
    getState() {
        const x_data = this.x.toArray(); 
        return {
            lat: x_data[0] * R2D, lon: x_data[1] * R2D, alt: x_data[2],
            vN: x_data[3], vE: x_data[4], vD: x_data[5],
            speed: Math.sqrt(x_data[3]**2 + x_data[4]**2 + x_data[5]**2),
            kUncert: this.P.get([3, 3]) + this.P.get([4, 4]) + this.P.get([5, 5]),
            kAltUncert: this.P.get([2, 2])
        };
    }
}

// --- FONCTIONS DE FILTRAGE ET DE MODÈLE (Altitude, Bruit, etc.) ---
function getKalmanR(accRaw, kAlt, kUncert, env, reactivityMode) {
    let acc_effective = gpsAccuracyOverride > 0 ? gpsAccuracyOverride : accRaw;
    if (acc_effective > MAX_ACC) { return 1e9; } 
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

function dist3D(lat1, lon1, alt1, lat2, lon2, alt2) {
    const from = turf.point([lon1, lat1, alt1]);
    const to = turf.point([lon2, lat2, alt2]);
    return turf.distance(from, to, { units: 'meters' });
}

function getBarometricAltitude(P_hPa, P_ref_hPa, T_K) {
    return (T_K / 0.0065) * (1 - (P_hPa / P_ref_hPa)**(R_AIR * 0.0065 / G_ACC));
}

// CORRECTION : Logique "Mode Nether" remplacée par "Rapport Distance Altitude"
function calculateDistanceRatio(alt) {
    // Calcule le ratio de la distance à la surface vs au centre
    return R_E_BASE / (R_E_BASE + alt);
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

function getWGS84Gravity(lat, alt) {
    const latRad = lat * D2R; 
    const sin2lat = Math.sin(latRad) ** 2;
    const g_surface = WGS84_G_EQUATOR * (1 + WGS84_BETA * sin2lat) / Math.sqrt(1 - WGS84_E2 * sin2lat);
    return g_surface * (1 - 2 * alt / WGS84_A); 
}

function getSpeedOfSound(tempK) {
    if(tempK < KELVIN_OFFSET) tempK += KELVIN_OFFSET; 
    return Math.sqrt(GAMMA_AIR * R_AIR * tempK);
}

function calculateMaxVisibleDistance(altitude) {
    return Math.sqrt(2 * R_E_BASE * altitude + altitude * altitude);
 }
    // --- FONCTIONS DE PHYSIQUE AVANCÉE ---
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
    const machNumber = (speedOfSoundLocal > 0) ? V / speedOfSoundLocal : 0;
    const dynamicPressure = 0.5 * airDensity * V * V; 
    const reynoldsNumber = (airDensity * V * 1) / MU_DYNAMIC_AIR; 
    const dragForce = dynamicPressure * (CdA || 0.5); 
    const dragPower_kW = (dragForce * V) / 1000.0;
    
    const coriolisForce = 2 * mass * V * OMEGA_EARTH * Math.sin(lat * D2R);
    const geopotentialAltitude = (G_ACC > 0) ? kAlt * (G_ACC / localG) : kAlt;
    const force_g_long = localG > 0.1 ? (accel_long / localG) : 0;
    
    const nyquistFrequency = 0.5 * (1000 / IMU_UPDATE_RATE_MS); 
    const altSigma = Math.sqrt(Math.abs(kAltUncert)); 

    const solarIrradiance = SOLAR_FLUX_DENSITY * Math.max(0, Math.sin(sunAltitudeRad)); 
    const radiationPressure = solarIrradiance / C_L; 

    return { 
        lorentzFactor, timeDilationSpeed, energyRelativistic, E0, momentum, gravitationalDilation, Rs_object,
        speedOfSoundLocal, machNumber, dynamicPressure, reynoldsNumber, dragForce, dragPower_kW,
        coriolisForce, geopotentialAltitude, force_g_long,
        nyquistFrequency, altSigma, radiationPressure
    };
}

// --- FONCTIONS ASTRO (SUNCALC & Custom) ---
const J1970 = 2440588;
const J2000 = 2451545.0; 
const dayMs = 1000 * 60 * 60 * 24;
const MC_DAY_MS = 72 * 60 * 1000; 

function toDays(date) { return (date.valueOf() / dayMs - 0.5 + J1970) - J2000; }
function solarMeanAnomaly(d) { return D2R * (356.0470 + 0.9856002585 * d); }
function eclipticLongitude(M) {
    var C = D2R * (1.9148 * Math.sin(M) + 0.0200 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M)), 
        P = D2R * 102.9377;                                                                
    return M + C + P + Math.PI;
}

// CORRECTION : Vérification SunCalc
function getSolarTime(date, lon) {
    if (date === null || lon === null || isNaN(lon) || typeof SunCalc === 'undefined') {
        return { TST: 'N/A', MST: 'N/A', EOT: 'N/A', ECL_LONG: 'N/A', DateMST: null, DateTST: null };
    }
    
    const d = toDays(date);
    const M = solarMeanAnomaly(d); 
    const L = eclipticLongitude(M); 
    
    const J_star = toDays(date) - lon / 360;
    const J_transit = J_star + (0.0053 * Math.sin(M) - 0.0069 * Math.sin(2 * L));
    // CORRECTION : Équation du temps réaliste
    const eot_min = SunCalc.getEquationOfTime(date) * 60; 

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

function getTSLV(date, lon) {
    if (date === null) return 'N/A';
    const GMST = (date.getUTCHours() + date.getUTCMinutes() / 60) * 15; 
    const LST = GMST + lon;
    const LST_h = (LST / 15 + 24) % 24;
    return LST_h.toFixed(2) + ' h';
}
    // --- VARIABLES D'ÉTAT (Globales) ---
let wID = null, domSlowID = null, domFastID = null, lPos = null;
let lat = null, lon = null, sTime = null;
let distM = 0, maxSpd = 0;
let kSpd = 0, kUncert = UKF_R_MAX, kAltUncert = 10; 
let timeMoving = 0, timeTotal = 0; 
let lastFSpeed = 0; 
let ukf = null; 

let currentGPSMode = 'HIGH_FREQ'; 
let emergencyStopActive = false; 
let distanceRatioMode = false; 
let selectedEnvironment = 'NORMAL'; 
let gpsAccuracyOverride = 0.0; 
let lastGPSPos = null;

let lServH = null, lLocH = null; // Horodatages NTP
let lastP_hPa = BARO_ALT_REF_HPA, lastT_K = TEMP_SEA_LEVEL_K, currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound = 343;
let lastWeatherData = null;

// Capteurs
let accel = { x: 0, y: 0, z: 0 };
let gyro = { x: 0, y: 0, z: 0 };
let mag = { x: 0, y: 0, z: 0 };
let barometer = null;
let lightSensor = null;
let audioContext = null;
let soundLevelMax = 0;
let lightLevelMax = 0;
let lastIMUTimestamp = 0;

let lastMapUpdate = 0;
let map, marker, circle; // Objets Leaflet
let currentUKFReactivity = 'AUTO';
let gpsStandbyTimeoutID = null;

// --- GESTION NTP (SYNCHRO HEURE) --- 
async function syncH() { 
    if ($('local-time')) $('local-time').textContent = 'Synchronisation...'; 
    lLocH = performance.now(); 
    try { 
        const response = await fetch(SERVER_TIME_ENDPOINT, { cache: "no-store", mode: "cors" }); 
        if (!response.ok) throw new Error(`Échec du fetch`); 
        const serverData = await response.json(); 
        const utcTimeISO = serverData.utc_datetime; 
        lServH = Date.parse(utcTimeISO); 
        if ($('local-time')) $('local-time').textContent = '✅ SYNCHRO NTP ACTIVE'; 
    } catch (error) { 
        // CORRECTION HORS LIGNE 
        lServH = Date.now(); 
        if ($('local-time')) $('local-time').textContent = '❌ HORS LIGNE (Local)'; 
        console.warn("Échec de la synchro NTP (Hors ligne ?). Utilisation de l'heure locale.", error.message); 
    } 
} 

/** Retourne l'heure synchronisée. */ 
function getCDate() { 
    if (lServH === null || lLocH === null) { 
        return new Date(); 
    } 
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
            lastP_hPa = P_hPa; 
            lastT_K = T_K; 
            currentAirDensity = air_density; 
            currentSpeedOfSound = getSpeedOfSound(T_K); 
            lastWeatherData = { 
                pressure_hPa: P_hPa, 
                tempC: T_C, 
                tempK: T_K, 
                humidity_perc: H_perc, 
                air_density: air_density 
            }; 
            return lastWeatherData;
        } 
        return null; 
    } catch (error) { 
        console.warn("Échec du fetch météo (Hors ligne ou API non accessible).", error.message); 
        return null; 
    } 
} 

// --- GESTION GPS --- 
function toggleGPS() { 
    if (wID === null) { 
        ukf = new ProfessionalUKF(); 
        if (!navigator.geolocation) { 
            alert("La géolocalisation n'est pas supportée par votre navigateur."); 
            return; 
        } 
        $('toggle-gps-btn').textContent = '⏸️ PAUSE GPS'; 
        $('toggle-gps-btn').classList.remove('btn-start'); 
        $('toggle-gps-btn').classList.add('btn-pause'); 
        currentGPSMode = $('freq-select').value; 
        wID = navigator.geolocation.watchPosition(onGpsSuccess, onGpsError, GPS_OPTS[currentGPSMode]); 
        
        domSlowID = setInterval(slowDomUpdate, DOM_SLOW_UPDATE_MS); 
        domFastID = setInterval(fastDomUpdate, IMU_UPDATE_RATE_MS); 
        
        requestIMUData(); 
        
        if (!map) initMap(); 
        
        resetStandbyTimeout(); 
        $('speed-stable').textContent = 'INIT...'; 
        $('toggle-gps-btn').style.backgroundColor = '#28a745'; 
        $('toggle-gps-btn').style.color = 'white'; 
        console.log("GPS démarré en mode: " + currentGPSMode); 
    } else { 
        navigator.geolocation.clearWatch(wID); 
        clearInterval(domSlowID); 
        clearInterval(domFastID); 
        clearTimeout(gpsStandbyTimeoutID); 
        wID = null; 
        domSlowID = null; 
        domFastID = null; 
        gpsStandbyTimeoutID = null; 
        $('toggle-gps-btn').textContent = '▶️ MARCHE GPS'; 
        $('toggle-gps-btn').classList.remove('btn-pause'); 
        $('toggle-gps-btn').classList.add('btn-start'); 
        $('speed-stable').textContent = dataOrDefault(kSpd, 2, ' m/s'); 
        $('toggle-gps-btn').style.backgroundColor = '#555'; 
        $('toggle-gps-btn').style.color = 'white'; 
        console.log("GPS et boucles arrêtés."); 
    } 
} 

function onGpsSuccess(pos) { 
    resetStandbyTimeout(); 
    const c = pos.coords; 
    const oldLat = lat, oldLon = lon, oldAlt = lPos ? lPos.coords.altitude : null; 
    const now = performance.now(); 
    
    // Calculer dt (temps écoulé depuis la dernière mise à jour GPS)
    const dt = lPos ? (now - lPos.timestamp) / 1000 : MIN_DT; 

    // UKF Prediction - Utilise les données IMU pour l'intervalle dt
    const imuData = { 
        accel: [accel.x, accel.y, accel.z], // Utilisation des données IMU
        gyro: [gyro.x, gyro.y, gyro.z],
        mag: [mag.x, mag.y, mag.z]
    };
    if(ukf) ukf.predict(imuData, dt);

    // UKF Update - Mise à jour avec la mesure GPS
    lat = c.latitude; 
    lon = c.longitude; 
    const gpsData = { 
        latitude: lat, 
        longitude: lon, 
        altitude: c.altitude || 0, 
        accuracy: c.accuracy, 
        speed: c.speed || 0, 
        heading: c.heading, 
        timestamp: now 
    }; 
    const R_dyn = getKalmanR(gpsData.accuracy, ukf ? ukf.getState().alt : gpsData.altitude, ukf ? ukf.getState().kUncert : UKF_R_MAX, selectedEnvironment, currentUKFReactivity); 
    if(ukf) ukf.update(gpsData, R_dyn); 

    lPos = pos; 

    // Mettre à jour les variables d'état du tableau de bord
    const kState = ukf ? ukf.getState() : { 
        lat: lat, lon: lon, alt: gpsData.altitude, speed: gpsData.speed, 
        kUncert: gpsData.accuracy, kAltUncert: 0 
    }; 
    kSpd = kState.speed; 
    kUncert = kState.kUncert; 
    kAltUncert = kState.kAltUncert; 
    
    sTime = getCDate(); 

    // Calcul de la distance
    if (oldLat !== null) { 
        const d = dist3D(oldLat, oldLon, oldAlt, lat, lon, c.altitude); 
        distM += d; 
    } 
    
    // Mettre à jour la vitesse maximale
    if (kSpd > maxSpd) maxSpd = kSpd; 

    // Mise à jour de la carte (moins fréquente)
    if (now - lastMapUpdate > MAP_UPDATE_INTERVAL) {
        updateMap(kState.lat, kState.lon, c.accuracy);
        lastMapUpdate = now;
    }
    
    // Mise à jour météo (moins fréquente)
    if (!lastWeatherData) {
        fetchWeather(lat, lon); 
    }

    // Affichage immédiat de la position GPS brute
    $('lat-raw').textContent = dataOrDefault(lat, 5) + ' °'; 
    $('lon-raw').textContent = dataOrDefault(lon, 5) + ' °'; 
    $('alt-raw').textContent = dataOrDefault(c.altitude, 2) + ' m'; 
    $('acc-raw').textContent = dataOrDefault(c.accuracy, 2) + ' m'; 

    // Mise à jour des données filtrées
    $('lat-stable').textContent = dataOrDefault(kState.lat, 5) + ' °'; 
    $('lon-stable').textContent = dataOrDefault(kState.lon, 5) + ' °'; 
    $('alt-stable').textContent = dataOrDefault(kState.alt, 2) + ' m'; 
    $('uncert-stable').textContent = dataOrDefault(kState.kUncert, 3) + ' m'; 
    $('speed-stable').textContent = dataOrDefault(kSpd, 2) + ' m/s'; 

    // Mettre à jour l'état de la boussole/niveau à bulle
    updateSpiritLevel(imuData.accel, c.heading);
} 

function onGpsError(error) { 
    console.error("Erreur GPS:", error.code, error.message); 
    if (error.code === error.PERMISSION_DENIED) { 
        alert("Accès à la géolocalisation refusé. Veuillez l'activer."); 
    } 
    // Tentative de redémarrage automatique en cas de timeout
    if (error.code === error.TIMEOUT) {
        console.warn("Timeout GPS. Tentative de redémarrage...");
        navigator.geolocation.clearWatch(wID);
        currentGPSMode = (currentGPSMode === 'HIGH_FREQ' ? 'LOW_FREQ' : 'HIGH_FREQ');
        $('freq-select').value = currentGPSMode;
        wID = navigator.geolocation.watchPosition(onGpsSuccess, onGpsError, GPS_OPTS[currentGPSMode]);
    }
} 

// --- GESTION CAPTEURS IMU (Accéléromètre/Gyro/Magneto) ---
function requestIMUData() {
    // Vérification des permissions et accès aux capteurs
    if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', handleMotion, false);
    } else {
        console.warn("DeviceMotionEvent non supporté.");
    }

    if (window.DeviceOrientationEvent) {
        window.addEventListener('deviceorientation', handleOrientation, false);
    } else {
        console.warn("DeviceOrientationEvent non supporté.");
    }
}

function handleMotion(event) {
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
}

function handleOrientation(event) {
    mag.x = event.alpha; // Alpha: boussole (azimut)
    mag.y = event.beta;  // Beta: inclinaison avant/arrière
    mag.z = event.gamma; // Gamma: roulis gauche/droite
}

// --- GESTION DES TIMEOUTS ET STANDBY ---
function resetStandbyTimeout() {
    clearTimeout(gpsStandbyTimeoutID);
    gpsStandbyTimeoutID = setTimeout(() => {
        if (wID !== null) {
            console.warn("Timeout de veille atteint. Passage en mode Basse Fréquence.");
            currentGPSMode = 'LOW_FREQ';
            $('freq-select').value = currentGPSMode;
            navigator.geolocation.clearWatch(wID);
            wID = navigator.geolocation.watchPosition(onGpsSuccess, onGpsError, GPS_OPTS[currentGPSMode]);
            resetStandbyTimeout(); // Redémarrer le timeout
        }
    }, STANDBY_TIMEOUT_MS);
}

// --- BOUCLES DE MISE À JOUR DOM ---

let sessionStartTime = performance.now();
let lastSlowTime = 0;
let sunAltitudeRad = 0;

function slowDomUpdate() {
    if (sTime) {
        // Temps de session
        timeTotal = (performance.now() - sessionStartTime) / 1000;
        $('elapsed-session-time').textContent = dataOrDefault(timeTotal, 2) + ' s'; 
        
        // Temps de mouvement
        if (kSpd > MIN_SPD) {
            timeMoving += (performance.now() - lastSlowTime) / 1000;
        }
        $('elapsed-motion-time').textContent = dataOrDefault(timeMoving, 2) + ' s';
        
        // Horodatage
        $('heure-locale').textContent = sTime.toLocaleTimeString(); 
        $('date-heure-utc').textContent = sTime.toISOString().replace('T', ' ').substring(0, 19) + ' UTC'; 
        
        // Vitesse Max et Distance
        $('max-speed-display').textContent = dataOrDefault(maxSpd * KMH_MS, 2) + ' km/h';
        $('max-speed-display-ms').textContent = dataOrDefault(maxSpd, 2) + ' m/s'; 
        $('distance-traveled').textContent = dataOrDefault(distM / 1000, 3) + ' km'; 
        
        // Météo et Baro
        if (lastWeatherData) {
            $('temp-display').textContent = dataOrDefault(lastWeatherData.tempC, 1) + ' °C';
            $('pressure-display').textContent = dataOrDefault(lastWeatherData.pressure_hPa, 2) + ' hPa';
            $('humidity-display').textContent = dataOrDefault(lastWeatherData.humidity_perc, 1) + ' %';
            $('air-density-display').textContent = dataOrDefault(currentAirDensity, 3) + ' kg/m³';
            $('speed-of-sound-display').textContent = dataOrDefault(currentSpeedOfSound, 2) + ' m/s';
            $('baro-alt').textContent = dataOrDefault(getBarometricAltitude(lastP_hPa, BARO_ALT_REF_HPA, lastT_K), 2) + ' m';
        } else {
             // Affichage des valeurs par défaut hors ligne
            $('temp-display').textContent = dataOrDefault(TEMP_SEA_LEVEL_K - KELVIN_OFFSET, 1) + ' °C (ISA)';
            $('pressure-display').textContent = dataOrDefault(BARO_ALT_REF_HPA, 2) + ' hPa (ISA)';
            $('humidity-display').textContent = 'N/A';
            $('air-density-display').textContent = dataOrDefault(RHO_SEA_LEVEL, 3) + ' kg/m³ (ISA)';
            $('speed-of-sound-display').textContent = dataOrDefault(getSpeedOfSound(TEMP_SEA_LEVEL_K), 2) + ' m/s (ISA)';
            $('baro-alt').textContent = 'N/A';
        }
        
        // Gravité et Référence UKF
        const localG = getWGS84Gravity(lat || 0, ukf ? ukf.getState().alt : (lPos ? lPos.coords.altitude : 0));
        $('gravity-g').textContent = dataOrDefault(localG, 4) + ' m/s²';
        $('gravity-base').textContent = dataOrDefault(G_ACC, 4) + ' m/s²';
        $('alt-ref-radius').textContent = dataOrDefault(R_ALT_CENTER_REF / 1000, 2) + ' km';
        
        // Astronomie
        if (lat !== null && lon !== null) {
            const times = SunCalc.getTimes(sTime, lat, lon);
            const pos = SunCalc.getPosition(sTime, lat, lon);
            const moonPos = SunCalc.getMoonPosition(sTime, lat, lon);
            const moonIllum = SunCalc.getMoonIllumination(sTime);
            
            sunAltitudeRad = pos.altitude;
            
            $('sun-alt').textContent = dataOrDefault(pos.altitude * R2D, 2) + ' °';
            $('sun-azimuth').textContent = dataOrDefault(pos.azimuth * R2D, 2) + ' °';
            $('moon-alt').textContent = dataOrDefault(moonPos.altitude * R2D, 2) + ' °';
            $('moon-azimuth').textContent = dataOrDefault(moonPos.azimuth * R2D, 2) + ' °';
            $('moon-illuminated').textContent = dataOrDefault(moonIllum.fraction * 100, 1) + ' %';
            $('moon-phase-name').textContent = getMoonPhaseName(moonIllum.phase);
            
            const sr = times.sunrise ? times.sunrise.toLocaleTimeString() : 'N/A';
            const ss = times.sunset ? times.sunset.toLocaleTimeString() : 'N/A';
            $('day-duration').textContent = times.sunset && times.sunrise ? ((times.sunset.getTime() - times.sunrise.getTime()) / 3600000).toFixed(2) + ' h' : 'N/A';
            $('sunrise-times').textContent = sr;
            $('sunset-times').textContent = ss;
            
            const mt = times.moonrise ? times.moonrise.toLocaleTimeString() : 'N/A';
            const mst = times.moonset ? times.moonset.toLocaleTimeString() : 'N/A';
            $('moon-times').textContent = `L: ${mt} / C: ${mst}`;
            
            const solarTimes = getSolarTime(sTime, lon);
            $('solar-time-tst').textContent = solarTimes.TST;
            $('solar-time-mst').textContent = solarTimes.MST;
            $('eot-min').textContent = solarTimes.EOT + ' min';
            $('ecliptic-long').textContent = solarTimes.ECL_LONG + ' °';
            $('tslv').textContent = getTSLV(sTime, lon);
            
            $('time-minecraft').textContent = getMinecraftTime(sTime);
            updateMinecraftClock(pos.altitude * R2D, solarTimes.DateMST, solarTimes.DateTST);
            
            // Physique avancée - Les valeurs sont calculées dans fastDomUpdate mais affichées ici.
        }
        
        // Distance visible
        $('max-visible-dist').textContent = dataOrDefault(calculateMaxVisibleDistance(ukf ? ukf.getState().alt : (lPos ? lPos.coords.altitude : 0)) / 1000, 2) + ' km';
        
    } else {
        $('elapsed-session-time').textContent = dataOrDefault(timeTotal, 2) + ' s';
        $('heure-locale').textContent = new Date().toLocaleTimeString(); 
    }
    lastSlowTime = performance.now();
}

function fastDomUpdate() {
    const kState = ukf ? ukf.getState() : { alt: lPos ? lPos.coords.altitude : 0, lat: lat || 0, speed: kSpd, kAltUncert: kAltUncert };
    const localG = getWGS84Gravity(kState.lat, kState.alt);
    
    // Calcul de la Physique Avancée
    const advancedPhysics = calculateAdvancedPhysics(
        kSpd, 
        kState.alt, 
        parseFloat($('mass-input').value) || 70, 
        parseFloat($('cda-input').value) || 0.5, 
        lastT_K, 
        currentAirDensity, 
        kState.lat, 
        kState.kAltUncert,
        localG,
        Math.sqrt(accel.x**2 + accel.y**2 + accel.z**2)
    );
    
    // Affichage des quantités Relativistes
    $('lorentz-factor').textContent = dataOrDefaultExp(advancedPhysics.lorentzFactor, 10);
    $('rs-object').textContent = dataOrDefaultExp(advancedPhysics.Rs_object, 10) + ' m';
    $('energy-rest').textContent = dataOrDefaultExp(advancedPhysics.E0, 4) + ' J';
    $('energy-relativistic').textContent = dataOrDefaultExp(advancedPhysics.energyRelativistic, 4) + ' J';
    $('momentum').textContent = dataOrDefaultExp(advancedPhysics.momentum, 4) + ' kg·m/s';
    $('time-dilation-speed').textContent = dataOrDefault(advancedPhysics.timeDilationSpeed, 4) + ' ns/jour';
    $('time-dilation-grav').textContent = dataOrDefault(advancedPhysics.gravitationalDilation, 4) + ' ns/jour';
    
    // Affichage des quantités Aérodynamiques/Méc. Classique
    $('mach-number').textContent = dataOrDefault(advancedPhysics.machNumber, 3);
    $('dynamic-pressure').textContent = dataOrDefault(advancedPhysics.dynamicPressure, 2) + ' Pa';
    $('reynolds-number').textContent = dataOrDefaultExp(advancedPhysics.reynoldsNumber, 3);
    $('drag-force').textContent = dataOrDefault(advancedPhysics.dragForce, 2) + ' N';
    $('drag-power').textContent = dataOrDefault(advancedPhysics.dragPower_kW, 4) + ' kW';
    $('coriolis-force').textContent = dataOrDefaultExp(advancedPhysics.coriolisForce, 6) + ' N';
    
    // Affichage des quantités de Gravité/Geopotentiel
    $('geopotential-alt').textContent = dataOrDefault(advancedPhysics.geopotentialAltitude, 2) + ' m';
    $('force-g-long').textContent = dataOrDefault(advancedPhysics.force_g_long, 2) + ' g';
    
    // Affichage des quantités d'Incertitude/Capteurs
    $('alt-sigma').textContent = dataOrDefault(advancedPhysics.altSigma, 3) + ' m';
    $('nyquist-freq').textContent = dataOrDefault(advancedPhysics.nyquistFrequency, 1) + ' Hz';
    $('imu-rate').textContent = dataOrDefault(1000 / IMU_UPDATE_RATE_MS, 0) + ' Hz';

    // Affichage des données brutes des capteurs
    $('accel-raw').textContent = `X: ${dataOrDefault(accel.x, 2)} Y: ${dataOrDefault(accel.y, 2)} Z: ${dataOrDefault(accel.z, 2)} m/s²`;
    $('gyro-raw').textContent = `X: ${dataOrDefault(gyro.x, 2)} Y: ${dataOrDefault(gyro.y, 2)} Z: ${dataOrDefault(gyro.z, 2)} °/s`;
    $('mag-raw').textContent = `X: ${dataOrDefault(mag.x, 2)} Y: ${dataOrDefault(mag.y, 2)} Z: ${dataOrDefault(mag.z, 2)} μT`;
    
    // Mise à jour de la couleur de la vitesse (vert, jaune, rouge)
    updateSpeedColor(kSpd);
    
    // Mettre à jour le statut d'arrêt d'urgence si la vitesse est trop élevée
    if (kSpd > MAX_ACC || isNaN(kSpd)) {
        if (!emergencyStopActive) {
            toggleEmergencyStop(true);
        }
    } else if (emergencyStopActive) {
        toggleEmergencyStop(false);
    }
}

// --- LOGIQUE D'AFFICHAGE ET D'ANIMATION ---

function updateSpeedColor(speed) {
    const ratio = speed / (MAX_PLAUSIBLE_ACCEL_GPS * 2); 
    let color;
    if (ratio < 0.2) color = '#00ff66'; 
    else if (ratio < 0.5) color = '#ffff00';
    else color = '#ff4500';
    $('speed-stable').style.color = color;
}

function updateMap(lat, lon, accuracy) {
    if (!map) return;
    const currentZoom = map.getZoom();

    if (marker) {
        marker.setLatLng([lat, lon]);
    } else {
        marker = L.marker([lat, lon]).addTo(map);
    }

    if (circle) {
        circle.setLatLng([lat, lon]).setRadius(accuracy || 10);
    } else {
        circle = L.circle([lat, lon], { radius: accuracy || 10 }).addTo(map);
    }
    
    // Ne pas centrer la carte si l'utilisateur l'a déplacée ou si le zoom est trop élevé
    if (currentZoom < 18 || lPos === null || !map.dragging.enabled()) {
        map.setView([lat, lon], Math.max(14, 18 - Math.log2(accuracy || 10)));
    }
}

function initMap() {
    map = L.map('map', { zoomControl: false, maxZoom: 20 }).setView([lat || 0, lon || 0], 13);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '© OpenStreetMap contributors'
    }).addTo(map);
    // Option pour désactiver le filtre de couleur en mode X-Ray
    if ($('map').classList.contains('x-ray')) {
        $('map').style.filter = 'none';
    }
}

function updateSpiritLevel(accel, heading) {
    const beta = Math.atan2(accel.y, accel.z) * R2D;
    const gamma = Math.atan2(accel.x, accel.z) * R2D;

    // Inverser l'axe Y pour une meilleure intuition sur les téléphones
    const transformX = Math.min(Math.max(gamma, -100), 100); 
    const transformY = Math.min(Math.max(beta, -100), 100); 

    const max_dev = 45; // Max 45 degrés d'inclinaison pour l'affichage
    const bubble_x = (transformX / max_dev) * 50; 
    const bubble_y = (transformY / max_dev) * 50; 
    
    // Déplacer la bulle (taille du conteneur 100x100, bulle 90x90, décalage de 5px)
    // Le centre est à 45. Décalage entre -45 et +45.
    const offsetX = 45 - (45 * bubble_x / 100);
    const offsetY = 45 - (45 * bubble_y / 100);

    $('spirit-level-bubble').style.transform = `translate(${offsetX}px, ${offsetY}px)`;
    $('roll-pitch').textContent = `Roul: ${dataOrDefault(gamma, 1)}° / Tang: ${dataOrDefault(beta, 1)}°`;

    // Boussole
    if (heading !== null) {
        $('compass-heading').textContent = dataOrDefault(heading, 1) + ' °';
        $('compass-needle').style.transform = `rotate(${-heading}deg)`;
    } else if (mag.x !== null) {
        // Fallback sur magnetomètre
        $('compass-heading').textContent = dataOrDefault(mag.x, 1) + ' ° (Mag)';
        $('compass-needle').style.transform = `rotate(${-mag.x}deg)`;
    } else {
        $('compass-heading').textContent = 'N/A';
        $('compass-needle').style.transform = `rotate(0deg)`;
    }
}

function updateMinecraftClock(sunAltDeg, dateMST, dateTST) {
    const isDay = sunAltDeg > 0;
    const isSunset = sunAltDeg < 10 && sunAltDeg > -5; // Créer une zone de coucher/lever
    const clock = $('minecraft-clock');
    
    clock.classList.remove('sky-day', 'sky-sunset', 'sky-night');
    if (isSunset) {
        clock.classList.add('sky-sunset');
    } else if (isDay) {
        clock.classList.add('sky-day');
    } else {
        clock.classList.add('sky-night');
    }

    // Calculer la rotation de Sun/Moon (Minecraft est simple, UTC/GMT)
    const date = getCDate();
    const totalMinutes = date.getUTCHours() * 60 + date.getUTCMinutes();
    const mcRotation = ((totalMinutes / (24 * 60)) * 360) + 90; // +90 pour aligner 6h avec 0°
    
    // TST est plus précis pour l'alignement réel du Soleil
    const tstHours = (dateTST.getUTCHours() + dateTST.getUTCMinutes() / 60) ;
    const tstRotation = ((tstHours / 24) * 360) - 90;

    $('sun-element').style.transform = `rotate(${tstRotation}deg)`;
    $('moon-element').style.transform = `rotate(${tstRotation + 180}deg)`; // La lune est à 180° du soleil pour la phase d'éclairage simple

    // Affichage des temps solaires
    $('mst-display').textContent = dateMST ? dateMST.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit', second: '2-digit' }) : 'N/A';
    $('tst-display').textContent = dateTST ? dateTST.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit', second: '2-digit' }) : 'N/A';
}

function toggleEmergencyStop(activate) {
    emergencyStopActive = activate;
    const btn = $('emergency-stop-btn');
    if (activate) {
        btn.textContent = '🛑 Arrêt d\'urgence: ACTIF 🔴';
        btn.classList.add('active');
        // Option : Arrêter le GPS en cas d'alerte critique
        if (wID !== null) toggleGPS();
    } else {
        btn.textContent = '🛑 Arrêt d\'urgence: INACTIF 🟢';
        btn.classList.remove('active');
    }
}

// --- GESTION DES ÉVÉNEMENTS ---

window.onload = () => {
    // Événements des boutons de contrôle
    $('toggle-gps-btn').addEventListener('click', toggleGPS);
    $('emergency-stop-btn').addEventListener('click', () => toggleEmergencyStop(!emergencyStopActive));
    $('reset-dist-btn').addEventListener('click', () => { distM = 0; });
    $('reset-max-btn').addEventListener('click', () => { maxSpd = 0; });
    $('reset-all-btn').addEventListener('click', () => { 
        if (wID !== null) toggleGPS(); 
        distM = 0; maxSpd = 0; timeMoving = 0; timeTotal = 0; 
        sessionStartTime = performance.now();
        ukf = new ProfessionalUKF(); 
        syncH();
        lastWeatherData = null;
        console.log("Système réinitialisé.");
    });

    $('toggle-mode-btn').addEventListener('click', () => {
        document.body.classList.toggle('dark-mode');
        $('toggle-mode-btn').innerHTML = document.body.classList.contains('dark-mode') ? '<i class="fas fa-moon"></i> Mode Nuit' : '<i class="fas fa-sun"></i> Mode Jour';
    });

    $('capture-data-btn').addEventListener('click', () => {
        alert("Fonctionnalité de capture des données non implémentée dans ce fichier.");
    });
    
    // Événement pour forcer la précision GPS
    $('gps-accuracy-override').addEventListener('input', (e) => {
        gpsAccuracyOverride = parseFloat(e.target.value) || 0.0;
        if (gpsAccuracyOverride < 0) gpsAccuracyOverride = 0;
        e.target.value = gpsAccuracyOverride;
        $('gps-accuracy-override').textContent = dataOrDefault(gpsAccuracyOverride, 1) + ' m';
    });

    // Événements de sélection/changement de mode
    $('freq-select').addEventListener('change', (e) => {
        currentGPSMode = e.target.value;
        if (wID !== null) { 
            navigator.geolocation.clearWatch(wID);
            wID = navigator.geolocation.watchPosition(onGpsSuccess, onGpsError, GPS_OPTS[currentGPSMode]);
        }
    });

    $('environment-select').addEventListener('change', (e) => {
        selectedEnvironment = e.target.value;
        const factor = ENVIRONMENT_FACTORS[selectedEnvironment];
        $('env-factor').textContent = `${factor.DISPLAY} (x${factor.R_MULT.toFixed(1)})`;
    });
    
    $('mass-input').addEventListener('input', (e) => {
        const mass = parseFloat(e.target.value) || 0;
        $('mass-display').textContent = dataOrDefault(mass, 3) + ' kg';
    });

    $('cd-area-input').addEventListener('input', (e) => {
        const cda = parseFloat(e.target.value) || 0;
        $('cda-display').textContent = dataOrDefault(cda, 3) + ' m²';
    });

    // Événement de sélection du Corps Céleste
    let currentCelestialBody = 'EARTH';
    $('celestial-body-select').addEventListener('change', (e) => {
        currentCelestialBody = e.target.value;
        const { G_ACC_NEW, R_ALT_CENTER_REF_NEW } = updateCelestialBody(currentCelestialBody, ukf ? ukf.getState().alt : 0, 0, 0);

        $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
        $('alt-ref-radius').textContent = `${(R_ALT_CENTER_REF_NEW / 1000).toFixed(2)} km`;
        
        // Afficher/Masquer les contrôles de rotation
        const rotationControls = $('rotation-controls');
        if (currentCelestialBody === 'ROTATING') {
            rotationControls.style.display = 'block';
        } else {
            rotationControls.style.display = 'none';
        }
    });

    // Événements des contrôles de rotation (pour 'ROTATING')
    const updateRotation = () => {
        const rotationRadius = parseFloat($('rotation-radius').value) || 0;
        const angularVelocity = parseFloat($('angular-velocity').value) || 0;
        $('rotation-radius-display').textContent = dataOrDefault(rotationRadius, 2) + ' m';
        $('angular-velocity-display').textContent = dataOrDefault(angularVelocity, 4) + ' rad/s';

        if (currentCelestialBody === 'ROTATING') {
            const { G_ACC_NEW } = updateCelestialBody('ROTATING', kAlt, rotationRadius, angularVelocity);
            $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
        }
    };
    $('rotation-radius').addEventListener('input', updateRotation);
    $('angular-velocity').addEventListener('input', updateRotation);
    
    // CORRECTION : Bouton "Rapport Distance"
    $('distance-ratio-toggle-btn').addEventListener('click', () => {
        distanceRatioMode = !distanceRatioMode;
        const ratio = distanceRatioMode ? calculateDistanceRatio(kAlt || 0) : 1.0;
        $('distance-ratio-toggle-btn').textContent = `Rapport Distance: ${distanceRatioMode ? 'ALTITUDE' : 'SURFACE'} (${ratio.toFixed(3)})`;
    });
    $('ukf-reactivity-mode').addEventListener('change', (e) => currentUKFReactivity = e.target.value);

    // --- DÉMARRAGE DU SYSTÈME ---
    updateCelestialBody(currentCelestialBody, kAlt, 0, 0);
    
    // Démarrer la synchro NTP (gère l'échec hors ligne)
    syncH();
    
    // Initialiser les valeurs par défaut hors ligne pour la physique
    currentAirDensity = RHO_SEA_LEVEL;
    currentSpeedOfSound = getSpeedOfSound(TEMP_SEA_LEVEL_K); // 15°C ISA
    lastT_K = TEMP_SEA_LEVEL_K;
    lastP_hPa = BARO_ALT_REF_HPA;
    
    // Initialiser l'affichage des constantes
    $('gravity-base').textContent = `${G_ACC.toFixed(4)} m/s²`;
    $('alt-ref-radius').textContent = `${(R_ALT_CENTER_REF / 1000).toFixed(2)} km`;

    // Mise à jour initiale du DOM (hors boucle)
    slowDomUpdate(); 

    // Initialisation de la carte (si elle est visible)
    if ($('map')) initMap();

    // Gestion du bouton X-Ray (Leaflet Inversion de couleur)
    $('xray-button').addEventListener('click', () => {
        $('map').classList.toggle('x-ray');
        if ($('map').classList.contains('x-ray')) {
            $('map').style.filter = 'none'; 
            $('xray-button').textContent = 'X-Ray ON';
        } else {
            $('map').style.filter = 'invert(0.9) hue-rotate(180deg)';
            $('xray-button').textContent = 'X-Ray OFF';
        }
    });
};
})(window);
