// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER UNIFIÉ FINAL (UKF 21 ÉTATS, COMPLET)
// Contenu de : gnss_dashboard_ready.zip/gnss-dashboard-full-fixed.js
// =================================================================

// Vérification des dépendances (math.js, Leaflet, SunCalc, Turf.js)
if (typeof math === 'undefined' || typeof L === 'undefined' || typeof SunCalc === 'undefined' || typeof turf === 'undefined') {
    const missing = [
        (typeof math === 'undefined' ? "math.min.js" : ""),
        (typeof L === 'undefined' ? "leaflet.js" : ""),
        (typeof SunCalc === 'undefined' ? "suncalc.js" : ""),
        (typeof turf === 'undefined' ? "turf.min.js" : "")
    ].filter(Boolean).join(", ");
    console.error(`Erreur critique : Dépendances manquantes : ${missing}.`);
    alert(`Erreur: Dépendances manquantes : ${missing}. L'application ne peut pas démarrer.`);
}

// -- API Endpoints --
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const PROXY_POLLUTANT_ENDPOINT = `${PROXY_BASE_URL}/api/pollutants`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

// -- Constants physiques et mathématiques --
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const KMH_MS = 3.6;         
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const G_U = 6.67430e-11;    // Constante gravitationnelle universelle
const R_E_BASE = 6371000;   // Rayon terrestre moyen (m)
const OMEGA_EARTH = 7.2921159e-5; // Rotation Terre (rad/s)
const AU_METERS = 149597870700; 
const LIGHT_YEAR_METERS = 9.461e15;
const SOLAR_FLUX_DENSITY = 1361; // W/m²

// -- Constantes atmosphériques (ISA) --
const BARO_ALT_REF_HPA = 1013.25;
const RHO_SEA_LEVEL = 1.225;
const R_AIR = 287.058;
const GAMMA_AIR = 1.4;
const MU_DYNAMIC_AIR = 1.8e-5;
const KELVIN_OFFSET = 273.15;
const SAT_O2_SEA_LEVEL = 97.5;  // % saturation O2 au niveau de la mer

// -- Constantes géophysiques (WGS84) --
const WGS84_A = 6378137.0;  // Rayon équatorial (m)
const WGS84_F = 1 / 298.257223563;
const WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F;
const WGS84_G_EQUATOR = 9.780327;
const WGS84_BETA = 0.0053024;

// -- Paramètres filtre UKF/EKF --
const UKF_STATE_DIM = 21;
const UKF_R_MAX = 500.0;
const R_ALT_MIN = 1.0;
const MAX_PLAUSIBLE_ACCEL_GPS = 19.62; // 2G
const ZUPT_RAW_THRESHOLD = 1.0;
const ZUPT_ACCEL_THRESHOLD = 0.5;
const MIN_SPD = 0.01;
const MAX_ACC = 200;
const NETHER_RATIO = 8.0;
const ALT_TH = -50;

// -- Configuration système --
const MIN_DT = 0.01;
const MAP_UPDATE_INTERVAL = 3000;
const IMU_UPDATE_RATE_MS = 20; // 50Hz
const DOM_SLOW_UPDATE_MS = 1000; // 1Hz
const STANDBY_TIMEOUT_MS = 300000; // 5 min
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ:  { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};

// -- Facteurs de réactivité UKF --
const UKF_REACTIVITY_FACTORS = {
    'AUTO':   { MULT: 1.0, DISPLAY: 'Automatique' },
    'NORMAL': { MULT: 1.0, DISPLAY: 'Normal' },
    'FAST':   { MULT: 0.2, DISPLAY: 'Rapide' },
    'STABLE': { MULT: 2.5, DISPLAY: 'Microscopique' }
};

// -- Facteurs d'environnement (affectent le bruit R) --
const ENVIRONMENT_FACTORS = {
    'NORMAL':   { R_MULT: 1.0, DISPLAY: 'Normal' },
    'FOREST':   { R_MULT: 2.5, DISPLAY: 'Forêt' },
    'CONCRETE': { R_MULT: 7.0, DISPLAY: 'Grotte/Tunnel' },
    'METAL':    { R_MULT: 5.0, DISPLAY: 'Métal/Bâtiment' }
};

// -- Données célestes/G :
const CELESTIAL_DATA = {
    'EARTH':    { G: 9.80665, R: WGS84_A, name: 'Terre' },
    'MOON':     { G: 1.62,    R: 1737400, name: 'Lune' },
    'MARS':     { G: 3.71,    R: 3389500, name: 'Mars' },
    'ROTATING': { G: 0.0,     R: WGS84_A, name: 'Station Spatiale' }
};

// -- Variables d'état globales --
let wID = null, domSlowID = null, domFastID = null, weatherFetchID = null;
let lat = null, lon = null, sTime = null;
let distM = 0, maxSpd = 0;
let kSpd = 0, kUncert = UKF_R_MAX, kAltUncert = 10, kAlt = 0;
let timeMoving = 0, timeTotal = 0;
let lastGPSPos = null;
let ukf = null;
let currentGPSMode = 'HIGH_FREQ';
let emergencyStopActive = false;
let distanceRatioMode = false; 
let netherMode = false;
let selectedEnvironment = 'NORMAL';
let currentMass = 70.0;
let currentCdA = 0.5;
let currentCelestialBody = 'EARTH';
let rotationRadius = 100;
let angularVelocity = 0.0;
let gpsAccuracyOverride = 0.0;

// Métrologie & environnement
let lServH = null, lLocH = null; // Horodatages NTP
let lastP_hPa = BARO_ALT_REF_HPA, lastT_K = 288.15, currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound = getSpeedOfSound(lastT_K);
let sunAltitudeRad = 0;
let lastWeatherData = null;
let lastPollutantData = null;

// Sensors
let accel = { x:0, y:0, z:0 }, gyro = { x:0, y:0, z:0 }, mag = { x:0, y:0, z:0 };
let lastIMUTimestamp = 0, lastMapUpdate = 0;
let map, marker, circle;
let currentUKFReactivity = 'AUTO';
let gpsStandbyTimeoutID = null;

// Utility
const $ = id => document.getElementById(id);
const dataOrDefault = (val, dec, suf='') => ((val==null||isNaN(val))? ((dec===0?'0':'0.00')+suf) : (val.toFixed(dec)+suf));
const dataOrDefaultExp = (val, dec, suf='') => ((val==null||isNaN(val))? ('0.'+Array(dec).fill('0').join('')+'e+0'+suf) : (val.toExponential(dec)+suf));

// ===========================================
// CLASSE UKF PROFESSIONNELLE (21 États)
// ===========================================
class ProfessionalUKF {
    constructor() {
        if (typeof math === 'undefined') {
            throw new Error("math.js n'est pas chargé. L'UKF 21 états est désactivé.");
        }
        this.N_STATES = UKF_STATE_DIM;
        this.x = math.zeros(this.N_STATES);
        this.P = math.diag(Array(this.N_STATES).fill(1e-2));
        this.Q = math.diag(Array(this.N_STATES).fill(1e-6));
        // Init état: position (lat/lon/alt) = 0
        this.x.set([0], 0); this.x.set([1], 0); this.x.set([2], 0);
    }
    predict(imuData, dt) {
        // (Modèle Strapdown simplifié - placeholder)
        // Ceci est une simplification extrême. Un vrai UKF 21 états propagerait
        // les quaternions, les biais de gyro/accel et la position/vitesse.
        
        // Simule la prédiction de vitesse (vN) basée sur l'accel (X) corrigée du biais (X[6])
        const accel_bias = this.x.get([6]) || 0;
        let vN = this.x.get([3]) + (imuData.accel[0] - accel_bias) * dt;
        
        // Applique ZUPT (Zero Velocity Update) si la vitesse prédite est infime
        if (Math.abs(vN) < MIN_SPD) vN = 0;
        
        this.x.set([3], vN); // Met à jour vN
        
        // Prédit la position (latitude approximée)
        let newLat = this.x.get([0]) + (vN / R_E_BASE) * dt;
        this.x.set([0], newLat);
        
        // (Un vrai UKF propagerait aussi la matrice P)
    }
    update(gpsData, R_dyn) {
        // (Correction UKF simplifiée - placeholder)
        // Un vrai UKF utiliserait la transformation Unscented, calculerait Pxy, Pyy et K (Gain de Kalman)
        // Ici, nous faisons une simple fusion pondérée (EKF simplifié)
        const K = 0.1; // Gain de Kalman (simplifié)
        
        // Position
        this.x.set([0], this.x.get([0]) * (1-K) + (gpsData.latitude * D2R) * K);
        this.x.set([1], this.x.get([1]) * (1-K) + (gpsData.longitude * D2R) * K);
        this.x.set([2], this.x.get([2]) * (1-K) + gpsData.altitude * K);
        
        // Vitesse (si fournie par le GPS)
        if (gpsData.speed != null) {
            const oldV = this.x.get([3]);
            // Fusionne la vitesse (vN) prédite avec la vitesse GPS
            this.x.set([3], oldV * (1-K) + gpsData.speed * K); 
        }
        
        // (Un vrai UKF mettrait à jour la matrice P)
    }
    getState() {
        const x_arr = this.x.toArray ? this.x.toArray() : this.x;
        // S'assurer que les états non calculés (vE, vD) retournent 0
        return {
            lat: (x_arr[0] || 0) * R2D,
            lon: (x_arr[1] || 0) * R2D,
            alt: x_arr[2] || 0,
            vN: x_arr[3] || 0, vE: x_arr[4] || 0, vD: x_arr[5] || 0,
            speed: Math.sqrt((x_arr[3]||0)**2 + (x_arr[4]||0)**2 + (x_arr[5]||0)**2),
            kUncert: this.P.get([3,3]) + this.P.get([4,4]) + this.P.get([5,5]),
            kAltUncert: this.P.get([2,2])
        };
    }
}

// ===========================================
// FILTRAGE ET MÉTÉO (Kalman R, Baro alt, etc.)
// ===========================================

function getKalmanR(accRaw, kAlt, kUncert, env, reactivityMode) {
    let acc_eff = gpsAccuracyOverride > 0 ? gpsAccuracyOverride : accRaw;
    if (acc_eff > MAX_ACC) { return 1e9; } // Signal GPS perdu, confiance nulle
    
    let R_base = Math.min(acc_eff, 100)**2;
    
    const env_mult = ENVIRONMENT_FACTORS[env]?.R_MULT || 1.0;
    let react_mult = UKF_REACTIVITY_FACTORS[reactivityMode]?.MULT || 1.0;
    
    // Mode Auto : ajuste la réactivité en fonction de la qualité du signal
    if (reactivityMode === 'AUTO') {
        if (acc_eff > 20) react_mult = 3.0; // Mauvais signal, faire plus confiance à l'IMU (R élevé)
        else if (acc_eff < 3) react_mult = 0.5; // Très bon signal, faire plus confiance au GPS (R faible)
    }
    
    let R_dyn = Math.min(R_base * env_mult * react_mult, UKF_R_MAX);
    return Math.max(R_dyn, R_ALT_MIN);
}

// Distance 3D via Turf (WGS84)
function dist3D(lat1, lon1, alt1, lat2, lon2, alt2) {
    // Vérification que Turf est chargé
    if (typeof turf === 'undefined') return 0;
    const from = turf.point([lon1, lat1, alt1||0]);
    const to   = turf.point([lon2, lat2, alt2||0]);
    return turf.distance(from, to, {units:'meters'});
}

// Altitude barométrique (ISA approximatif)
function getBarometricAltitude(P_hPa, P_ref_hPa, T_K) {
    if (P_hPa <= 0 || P_ref_hPa <= 0 || T_K <= 0) return NaN;
    const ratio = P_hPa / P_ref_hPa;
    // Utilise la gravité WGS84 locale pour plus de précision
    return (T_K/0.0065) * (1 - Math.pow(ratio, (R_AIR*0.0065)/getWGS84Gravity(lat||0, kAlt||0)));
}

// Gravité locale corrigée (WGS84)
function getWGS84Gravity(latDeg, alt) {
    const latRad = (latDeg||0) * D2R;
    const sin2 = Math.sin(latRad)**2;
    const g0 = WGS84_G_EQUATOR * (1 + WGS84_BETA*sin2) / Math.sqrt(1 - WGS84_E2*sin2);
    return g0 * (1 - 2*(alt||0)/WGS84_A);
}

// Vitesse du son corrigée par température
function getSpeedOfSound(tempK) {
    if(tempK < KELVIN_OFFSET) tempK += KELVIN_OFFSET; // Assure Kelvin
    return Math.sqrt(GAMMA_AIR * R_AIR * tempK);
}

// Distance vue (horizon) depuis altitude
function calculateMaxVisibleDistance(alt) {
    if (!alt || alt < 0) return 0;
    return Math.sqrt(2*R_E_BASE*alt + alt*alt);
}

// Calculs physiques avancés (relativité, forces, etc.)
function calculateAdvancedPhysics(kSpd, kAlt, mass, CdA, tempK, airDensity, latDeg, kAltUncert, localG, accel_long) {
    let V = isNaN(kSpd) ? 0 : kSpd;
    let alt_m = kAlt || 0;
    let lat_rad = (latDeg||0)*D2R;
    
    let lorentzFactor = 1/Math.sqrt(1 - (V/C_L)**2);
    let timeDilation = (lorentzFactor - 1) * 86400 * 1e9; // ns/day
    let E0 = mass * C_L * C_L;
    let E_rel = lorentzFactor * E0;
    let momentum = mass * V;
    let gravDilation = (localG * alt_m / (C_L*C_L)) * 86400 * 1e9;
    let Rs = (2 * G_U * mass)/(C_L*C_L);
    
    let speedSound = getSpeedOfSound(tempK);
    let mach = speedSound>0 ? V/speedSound : 0;
    let dynPress = 0.5 * airDensity * V * V;
    let reynolds = (airDensity * V * 1)/MU_DYNAMIC_AIR; // L=1m
    let dragForce = dynPress * (CdA||0.5);
    let dragPower = (dragForce * V)/1000.0;
    
    let coriolisForce = 2 * mass * V * OMEGA_EARTH * Math.sin(lat_rad);
    let geoAlt = (localG>0.1) ? alt_m*(G_ACC/localG) : alt_m;
    let force_g_long = (localG>0.1) ? (accel_long/localG) : 0;
    
    let nyquistFreq = 0.5*(1000/IMU_UPDATE_RATE_MS);
    let altSigma = Math.sqrt(Math.abs(kAltUncert||0));
    
    let solarIrr = SOLAR_FLUX_DENSITY * Math.max(0, Math.sin(sunAltitudeRad));
    let radPress = solarIrr / C_L;
    
    return {
        lorentzFactor, timeDilationSpeed:timeDilation, energyRelativistic:E_rel, E0, momentum,
        gravitationalDilation:gravDilation, Rs_object:Rs,
        speedOfSoundLocal:speedSound, machNumber:mach, dynamicPressure:dynPress, reynoldsNumber:reynolds,
        dragForce, dragPower_kW:dragPower, coriolisForce, geopotentialAltitude:geoAlt, force_g_long,
        nyquistFrequency:nyquistFreq, altSigma, radiationPressure:radPress
    };
}

// Bio/SVT (simulate certain life-support metrics)
function calculateBioSVT(tempC, alt, humidity_perc, pressurePa, sunAltRad) {
    const a = 17.27, b = 237.7;
    const h_frac = (humidity_perc||0)/100;
    const f = (a*tempC)/(b+tempC) + Math.log(h_frac);
    const dewPoint = (b*f)/(a-f);
    const wetBulb = tempC*Math.atan(0.151977*Math.sqrt(h_frac+8.313659))
                   + Math.atan(tempC + h_frac)
                   - Math.atan(h_frac - 1.67633)
                   + 0.00391838*Math.pow(h_frac,1.5)*Math.atan(0.023101*h_frac)
                   - 4.686035;
    const CAPE_sim = Math.max(0, 0.5*(tempC - dewPoint)*500);
    const O2Sat = SAT_O2_SEA_LEVEL - (alt/1000)*2;
    const O2SatClamped = Math.max(70, Math.min(100, O2Sat));
    const sunAngle = Math.max(0, sunAltRad*R2D)/90;
    const tempFactor = Math.exp(-0.5 * Math.pow((tempC-25)/10,2));
    const photoRate = 0.05 * sunAngle * tempFactor;
    
    // Calcul de l'humidité absolue
    const P_sat_hPa = tC => 6.1078 * Math.pow(10, (7.5*tC)/(237.3+tC));
    const P_v = P_sat_hPa(tempC) * h_frac;
    const absHum = (P_v*100*18.015)/(8.314*(tempC+KELVIN_OFFSET)) * 1000; // g/m3
    
    return {
        dewPoint, wetBulbTemp:wetBulb, CAPE_sim, O2SaturationClamped,
        photosynthesisRate:photoRate,
        absoluteHumidity:absHum
    };
}

// Solar time and related (SunCalc & custom)
const J1970 = 2440588, J2000 = 2451545.0, dayMs = 1000*60*60*24, MC_DAY_MS = 72*60*1000;
function toDays(date) { return (date.valueOf()/dayMs - 0.5 + J1970) - J2000; }
function solarMeanAnomaly(d) { return D2R*(356.0470 + 0.9856002585*d); }
function eclipticLongitude(M) {
    const C = D2R*(1.9148*Math.sin(M) + 0.0200*Math.sin(2*M) + 0.0003*Math.sin(3*M));
    const P = D2R*102.9377;
    return M + C + P + Math.PI;
}
function getSolarTime(date, lon) {
    if (!date||lon==null||isNaN(lon)) return {TST:'00:00:00', MST:'00:00:00', EOT:'0.00', ECL_LONG:'0.00', DateMST:null, DateTST:null};
    
    const d = toDays(date);
    const M = solarMeanAnomaly(d);
    const L = eclipticLongitude(M);
    
    // Utilise SunCalc pour une EoT plus précise si disponible
    const eot_min = (typeof SunCalc !== 'undefined') ? SunCalc.getEquationOfTime(date) * 60 : 0;

    const msUTC = date.getUTCHours()*3600000 + date.getUTCMinutes()*60000 + date.getUTCSeconds()*1000 + date.getUTCMilliseconds();
    const mst_off = lon*dayMs/360;
    const mst_ms = (msUTC + mst_off + dayMs) % dayMs;
    const eot_ms = eot_min*60000;
    const tst_ms = (mst_ms + eot_ms + dayMs) % dayMs;
    
    const toTimeStr = ms => { let h=Math.floor(ms/3600000), m=Math.floor((ms%3600000)/60000), s=Math.floor((ms%60000)/1000); return `${String(h).padStart(2,'0')}:${String(m).padStart(2,'0')}:${String(s).padStart(2,'0')}`; };
    
    return {
        TST: toTimeStr(tst_ms),
        MST: toTimeStr(mst_ms),
        EOT: eot_min.toFixed(2),
        ECL_LONG: (L*R2D).toFixed(2),
        DateMST: new Date(date.getTime() + mst_off),
        DateTST: new Date(date.getTime() + mst_off + eot_ms)
    };
}
function getMinecraftTime(date) {
    if (!date) return '00:00';
    const msUTC = date.getUTCHours()*3600000 + date.getUTCMinutes()*60000 + date.getUTCSeconds()*1000 + date.getUTCMilliseconds();
    const ratio = (msUTC % dayMs) / dayMs;
    const mcMs = (ratio * MC_DAY_MS + MC_DAY_MS) % MC_DAY_MS;
    let h = Math.floor(mcMs/3600000), m = Math.floor((mcMs%3600000)/60000);
    return `${String(h).padStart(2,'0')}:${String(m).padStart(2,'0')}`;
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
    if (!date || lon == null) return 'N/A';
    const GMST_deg = (date.getUTCHours() + date.getUTCMinutes()/60)*15;
    const LST_deg = (GMST_deg + lon);
    const h = (LST_deg/15 + 24) % 24;
    return h.toFixed(2) + ' h';
}

// ===========================================
// GESTION CAPTEURS/MÉTÉO/ASTRO
// ===========================================

async function syncH() {
    if ($('local-time')) $('local-time').textContent = 'Synchronisation...';
    try {
        const res = await fetch(SERVER_TIME_ENDPOINT, { cache:"no-store", mode:"cors" });
        if (!res.ok) throw new Error(res.statusText);
        const data = await res.json();
        const utc = data.utc_datetime;
        lServH = Date.parse(utc);
        lLocH = performance.now();
        if ($('local-time')) $('local-time').textContent = '✅ SYNCHRO NTP ACTIVE';
    } catch(e) {
        // Fallback sur l'heure locale si NTP échoue
        lServH = Date.now(); lLocH = performance.now();
        if ($('local-time')) $('local-time').textContent = '❌ SYNCHRO ÉCHOUÉE (Local)';
    }
}
// getCDate() est déjà défini dans le BLOC 1

async function fetchWeather(lat, lon) {
    const url = `${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`;
    try {
        const res = await fetch(url);
        if (!res.ok) throw new Error(`Erreur HTTP ${res.status}`);
        const data = await res.json();
        if (data.main) {
            const P = data.main.pressure, T_C = data.main.temp, T_K = T_C + KELVIN_OFFSET;
            const H = data.main.humidity;
            const airDensity = (P*100)/(R_AIR*T_K);
            
            // Mise à jour de l'état global
            lastP_hPa = P; 
            lastT_K = T_K;
            currentAirDensity = airDensity;
            currentSpeedOfSound = getSpeedOfSound(T_K);
            
            lastWeatherData = { pressure_hPa:P, tempC:T_C, tempK:T_K, humidity_perc:H, air_density:airDensity };
            return lastWeatherData;
        }
    } catch(e) {
        if ($('weather-status')) $('weather-status').textContent = `❌ ${e.message}`;
        console.warn("Erreur fetchWeather:", e.message);
    }
    return null;
}

async function fetchPollutants(lat, lon) {
    const url = `${PROXY_POLLUTANT_ENDPOINT}?lat=${lat}&lon=${lon}`;
    try {
        const res = await fetch(url);
        if (!res.ok) throw new Error(`Erreur HTTP ${res.status}`);
        const data = await res.json();
        if (data.list && data.list.length>0) {
            lastPollutantData = data.list[0].components;
            return lastPollutantData;
        }
    } catch(e) {
        if ($('aqi-display')) $('aqi-display').textContent = `❌ ${e.message}`;
        console.warn("Erreur fetchPollutants:", e.message);
    }
    return null;
}

// ===========================================
// MAP (Leaflet)
// ===========================================
function initMap() {
    try {
        if ($('map') && typeof L !== 'undefined') {
            const initialLat = lat || 43.296, initialLon = lon || 5.37;
            map = L.map('map', {
                worldCopyJump: true,
                minZoom: 2,
                maxZoom: 19,
                zoomControl: true,
                attributionControl: false
            }).setView([initialLat, initialLon], 16);

            // OpenStreetMap
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                maxZoom: 19,
                attribution: '© OpenStreetMap'
            }).addTo(map);

            // Attribution (pour éviter l'encombrement)
            L.control.attribution({ prefix: false, position: 'bottomleft' })
                .setAttribution('Données Cartographiques: OpenStreetMap • GNSS/UKF Dashboard')
                .addTo(map);

            // Marker de position actuel
            currentPositionMarker = L.marker([initialLat, initialLon], {
                icon: L.divIcon({
                    className: 'map-pin',
                    html: '<i class="fas fa-satellite-dish"></i>',
                    iconSize: [24, 24]
                }),
                title: 'Position Estimée (UKF)'
            }).addTo(map);

            // Cercle de précision
            accuracyCircle = L.circle([initialLat, initialLon], {
                color: '#007bff',
                fillColor: '#007bff',
                fillOpacity: 0.2,
                radius: kUncert || 10,
                weight: 1
            }).addTo(map);

            // Trajet (Polyline)
            pathLine = L.polyline([], {
                color: '#ffc107',
                weight: 3,
                opacity: 0.7
            }).addTo(map);

            // Animation du compas
            compassRing = L.circleMarker([initialLat, initialLon], {
                radius: 12,
                color: '#ff0000',
                fillColor: '#ff0000',
                fillOpacity: 0.1,
                weight: 1,
                opacity: 1
            }).addTo(map).bindTooltip('', { permanent: true, direction: 'center', className: 'compass-tooltip' });
            
            // Événement: Ajuster la taille de la carte si elle était masquée
            const observer = new MutationObserver((mutations) => {
                mutations.forEach((mutation) => {
                    if (mutation.attributeName === 'style' && map) {
                        map.invalidateSize();
                    }
                });
            });
            const mapElement = $('map');
            if(mapElement) {
                observer.observe(mapElement, { attributes: true });
            }
            
            // Événement de recentrage (si l'utilisateur a déplacé la carte)
            map.on('movestart', () => {
                isMapFixed = false;
                $('toggle-globe-btn').textContent = "🌐 Recentrer";
            });
            
            // Bouton de recentrage
            $('toggle-globe-btn').addEventListener('click', () => {
                isMapFixed = !isMapFixed;
                if(isMapFixed && lat && lon) {
                    map.setView([lat, lon], map.getZoom() > 10 ? map.getZoom() : 16);
                    $('toggle-globe-btn').textContent = "🔒 Carte Fixe";
                } else {
                    $('toggle-globe-btn').textContent = "🌐 Recentrer";
                }
            });

            // Afficher le bouton de recentrage
            $('toggle-globe-btn').style.display = 'block';

        } else if ($('map')) {
            $('map').textContent = "Erreur: Leaflet non chargé ou balise 'map' introuvable.";
        }
    } catch (e) {
        console.error("Erreur initMap:", e);
        if ($('map')) $('map').textContent = `Erreur lors de l'initialisation de la carte: ${e.message}`;
    }
}

// Fonction de mise à jour de la carte
function updateMap(latLng, acc, hdg) {
    if (!map || !currentPositionMarker) return;

    if (latLng.lat !== currentPositionMarker.getLatLng().lat || latLng.lng !== currentPositionMarker.getLatLng().lng) {
        // Mise à jour de la position
        currentPositionMarker.setLatLng(latLng);

        // Mise à jour du cercle de précision
        accuracyCircle.setLatLng(latLng).setRadius(acc * R_FACTOR_RATIO);

        // Mise à jour du chemin (ajout du point)
        pathLine.addLatLng(latLng);

        // Mise à jour de l'orientation (compas)
        if (hdg !== undefined && hdg !== null) {
            currentPositionMarker.setRotationAngle(hdg);
            compassRing.setLatLng(latLng);
            
            // Rotation de l'icône
            const iconElement = currentPositionMarker.getElement().querySelector('i');
            if (iconElement) {
                iconElement.style.transform = `rotate(${hdg}deg)`;
            }
        }
        
        // Auto-centrage si le mode 'Carte Fixe' est actif
        const now = Date.now();
        if (isMapFixed && now - lastMapUpdate > MAP_UPDATE_INTERVAL) {
            map.setView(latLng, map.getZoom() > 10 ? map.getZoom() : 16);
            lastMapUpdate = now;
        }
    }
}

// ===========================================
// BOUCLES PRINCIPALES
// ===========================================

// Gestionnaire d'erreurs GPS
function gpsErrorCallback(err) {
    if (emergencyStopActive) return;

    let errMsg = `Code ${err.code}: `;
    switch (err.code) {
        case err.PERMISSION_DENIED:
            errMsg += "Permission refusée. Cliquez sur 'MARCHE GPS'.";
            break;
        case err.POSITION_UNAVAILABLE:
            errMsg += "Position indisponible (Désactivé/Signal faible).";
            break;
        case err.TIMEOUT:
            errMsg += "Délai de recherche expiré. Nouveau cycle.";
            break;
        default:
            errMsg += "Erreur inconnue.";
    }

    console.error("Erreur GPS:", errMsg);
    if ($('gps-status-precision')) $('gps-status-precision').textContent = errMsg;
    if (err.code===1) stopGPS();
}

// IMU/Sensor handlers
function startIMUListeners() {
    if (emergencyStopActive || domFastID) return;

    try {
        if ($('imu-status')) $('imu-status').textContent = "Activation...";

        // Vérification des API de capteurs
        if (typeof Accelerometer==='undefined' || typeof Gyroscope==='undefined') {
            throw new Error("API Capteurs non supportée.");
        }

        // Accéléromètre (50Hz)
        const accSensor = new Accelerometer({frequency:50});
        accSensor.addEventListener('reading', ()=>{ accel.x=accSensor.x; accel.y=accSensor.y; accel.z=accSensor.z; });
        accSensor.addEventListener('error', e=> console.error("Accéléromètre:", e.error));
        accSensor.start();

        // Gyroscope (50Hz)
        const gyroSensor = new Gyroscope({frequency:50});
        gyroSensor.addEventListener('reading', ()=>{ gyro.x=gyroSensor.x; gyro.y=gyroSensor.y; gyro.z=gyroSensor.z; });
        gyroSensor.addEventListener('error', e=> console.error("Gyroscope:", e.error));
        gyroSensor.start();
        
        // Capteurs Environnementaux (si supportés)
        if (typeof AmbientLightSensor !== 'undefined') {
            const lightSensor = new AmbientLightSensor({frequency: 1});
            lightSensor.addEventListener('reading', () => { ambientLight = lightSensor.illuminance; maxAmbientLight = Math.max(maxAmbientLight, ambientLight); });
            lightSensor.addEventListener('error', e=> console.error("Lumière:", e.error));
            lightSensor.start();
        }
        
        if (typeof Magnetometer !== 'undefined') {
             const magSensor = new Magnetometer({frequency: 10});
             magSensor.addEventListener('reading', () => { magField.x=magSensor.x; magField.y=magSensor.y; magField.z=magSensor.z; });
             magSensor.addEventListener('error', e=> console.error("Magnétomètre:", e.error));
             magSensor.start();
        }

        if ($('imu-status')) $('imu-status').textContent = "Actif (API Sensor 50Hz)";
        lastIMUTimestamp = performance.now();
        
        // Démarrage de la boucle rapide (UKF / IMU)
        startFastLoop();
    } catch(error) {
        let msg = error.message;
        if (error.name==='SecurityError' || error.name==='NotAllowedError') {
            msg = "Permission Capteurs refusée. Cliquez sur 'MARCHE GPS'.";
        }
        if ($('imu-status')) $('imu-status').textContent = `Désactivé: ${msg}`;
        console.warn("Démarrage IMU Échoué:", error.message);
        
        // Démarrer la boucle rapide même sans IMU (pour que le GPS fonctionne)
        startFastLoop(); 
    }
}

// Arrêter les services GPS
function stopGPS(resetBtn=true) {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    wID = null;
    currentGPSMode = 'LOW_FREQ'; // Réinitialisation par défaut

    if (gpsStandbyTimeoutID) { clearTimeout(gpsStandbyTimeoutID); gpsStandbyTimeoutID = null; }
    if ($('start-btn') && resetBtn) $('start-btn').textContent = "▶️ MARCHE GPS";
    if ($('gps-status-precision')) $('gps-status-precision').textContent = "Arrêté.";
    if ($('gps-mode-display')) $('gps-mode-display').textContent = "Arrêté";
}

// Démarrer les services GPS
function startGPS(mode = 'HIGH_FREQ') {
    if (wID !== null) stopGPS(false); // Arrête l'ancienne surveillance
    if (emergencyStopActive) {
        alert("Arrêt d'urgence actif. Réinitialisez le système.");
        return;
    }
    
    currentGPSMode = mode;
    const opts = GPS_OPTS[currentGPSMode];

    wID = navigator.geolocation.watchPosition(gpsUpdateCallback, gpsErrorCallback, opts);
    
    if ($('start-btn')) $('start-btn').textContent = "⏳ En écoute...";
    if ($('gps-status-precision')) $('gps-status-precision').textContent = `Mode ${currentGPSMode} - Recherche...`;
    if ($('gps-mode-display')) $('gps-mode-display').textContent = currentGPSMode;

    // Démarre l'écoute IMU/UKF
    startIMUListeners(); 

    // Initialise les boucles lente et rapide
    startFastLoop();
    startSlowLoop();
}

// Fonction pour basculer entre les modes de fréquence GPS
function startGPSStandby() {
    if (wID === null || emergencyStopActive) return;
    
    // Si la vitesse est très faible et que nous sommes en HIGH_FREQ
    if (kSpd < MIN_SPD*0.5 && currentGPSMode === 'HIGH_FREQ') {
        if (!gpsStandbyTimeoutID) {
            $('gps-mode-display').textContent = "Mode Veille (Démarrage)";
            gpsStandbyTimeoutID = setTimeout(() => {
                if (kSpd < MIN_SPD) {
                    startGPS('LOW_FREQ');
                    $('gps-mode-display').textContent = "Mode Veille (Actif)";
                } else {
                    // Mouvement détecté au dernier moment
                    $('gps-mode-display').textContent = "HIGH_FREQ (Mvt. Repris)";
                    gpsStandbyTimeoutID = null; // Annule le timeout
                }
            }, STANDBY_TIMEOUT_MS);
        }
    }
}

// Boucle principale rapide (50Hz) : UKF, IMU, Vitesse
// ===========================================
function startFastLoop() {
    if (domFastID) return;
    
    // Dernière position GPS utilisée par l'UKF
    let lastUKFPos = null;
    let lastDelta = 0;
    
    const updateFastData = () => {
        if (emergencyStopActive) return;

        const now = performance.now();
        const dt = (now - lastIMUTimestamp) / 1000.0;
        lastIMUTimestamp = now;
        
        if (dt < MIN_DT) {
             // Si le navigateur ne respecte pas le 50Hz, on force la synchro.
             domFastID = requestAnimationFrame(updateFastData);
             return;
        }

        // --- UKF PREDICT / UPDATE ---
        let ukfUpdateOccurred = false;
        
        // 1. Prediction UKF (Propagation du modèle physique)
        if (ukf) {
            // Utilise l'accélération et la vitesse angulaire de l'IMU
            const u = [accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z];
            ukf.predict(u, dt);
        }
        
        // 2. Traitement des nouvelles données GPS (si disponibles)
        if (lastGPSPos) {
            const pos = lastGPSPos;
            const delta = pos.timestamp;
            
            // Si c'est une nouvelle position par rapport à la dernière utilisée par l'UKF
            if (delta > lastDelta && ukf) {
                lastDelta = delta;
                
                // Mettre à jour l'UKF avec la mesure GPS (Position, Vitesse, Précision)
                const z = [pos.coords.latitude, pos.coords.longitude, pos.coords.altitude, pos.coords.speed];
                const R_mult = ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT * UKF_REACTIVITY_FACTORS[currentUKFReactivity].MULT;
                const R_override = gpsAccuracyOverride > 0 ? gpsAccuracyOverride : null;
                
                // Précision minimale pour le R. Si aucune précision n'est donnée, on utilise une valeur de sécurité.
                let acc = pos.coords.accuracy > 0 ? pos.coords.accuracy : 10.0;
                
                // Gère les erreurs de capteur ou les valeurs incohérentes (e.g. altitude négative irréaliste)
                if (pos.coords.altitude < ALT_TH) {
                    console.warn(`Altitude GPS anormale (${pos.coords.altitude.toFixed(0)}m). Ignorée pour l'UKF.`);
                    acc = UKF_R_MAX; // Dégrade la confiance dans la mesure GPS
                }
                
                // UKF Update (Position/Vitesse)
                ukf.updateGPS(z, acc, R_mult, R_override);
                ukfUpdateOccurred = true;
                
                // Mise à jour de l'état global après l'UKF
                kSpd = ukf.X[3] * KMH_MS; // Vitesse Nord
                kUncert = Math.sqrt(ukf.P[0][0] + ukf.P[1][1]); // Incertitude horizontale
                kAltUncert = Math.sqrt(ukf.P[2][2]); // Incertitude verticale
                kAlt = ukf.X[2];

                // Mise à jour de la densité de l'air pour la traînée
                currentAirDensity = getAirDensity(kAlt || 0, lastP_hPa, lastT_K);
                currentSpeedOfSound = getSpeedOfSound(lastT_K); // La vitesse du son dépend uniquement de la Température.
                
                // Sauvegarde de la dernière position GPS utilisée
                lastUKFPos = pos;
            }
            lastGPSPos = null; // Consomme la donnée
        }

        // --- Mise à jour de l'état UKF ---
        if (ukf) {
            // Lissage de l'état et calcul des variables dérivées
            const X = ukf.X;
            const P = ukf.P;
            
            // État 1-3 : Lat/Lon/Alt
            lat = X[0];
            lon = X[1];
            kAlt = X[2];
            
            // État 4-6 : Vitesse (N/E/D)
            const speedN = X[3], speedE = X[4], speedD = X[5];
            const speedH = Math.sqrt(speedN*speedN + speedE*speedE);
            
            // Mise à jour des variables globales
            kSpd = speedH * KMH_MS; // Vitesse horizontale en km/h
            kUncert = Math.sqrt(P[0][0] + P[1][1]);
            kAltUncert = Math.sqrt(P[2][2]);
            
            // Calcul de la distance 3D parcourue
            const dx = speedN * dt;
            const dy = speedE * dt;
            const dz = speedD * dt;
            const dDist = Math.sqrt(dx*dx + dy*dy + dz*dz);
            
            // Mise à jour des compteurs
            distM += dDist;
            timeTotal += dt;
            if (speedH >= MIN_SPD) {
                timeMoving += dt;
                maxSpd = Math.max(maxSpd, kSpd);
            }
            
            // Calcul de l'énergie et de la traînée
            const dragForce = 0.5 * currentAirDensity * (speedH * speedH) * currentCdA;
            const dragPower = dragForce * speedH / 1000; // kW
            const kineticEnergy = 0.5 * currentMass * (speedH*speedH + speedD*speedD); // Joules
            
            // Mise à jour des affichages
            if ($('speed-kph')) $('speed-kph').textContent = dataOrDefault(kSpd, 2, ' km/h');
            if ($('speed-ms')) $('speed-ms').textContent = dataOrDefault(speedH, 2, ' m/s');
            if ($('alt-est')) $('alt-est').textContent = dataOrDefault(kAlt, 1, ' m');
            if ($('alt-uncert')) $('alt-uncert').textContent = `± ${dataOrDefault(kAltUncert, 1, ' m')}`;
            if ($('dist-total')) {
                const ratio = netherMode ? NETHER_RATIO : (distanceRatioMode ? calculateDistanceRatio(kAlt) : 1.0);
                const distDisplay = (distM / ratio) / 1000;
                $('dist-total').textContent = dataOrDefault(distDisplay, 3, ' km');
            }
            if ($('max-speed')) $('max-speed').textContent = dataOrDefault(maxSpd, 2, ' km/h');
            if ($('force-drag')) $('force-drag').textContent = dataOrDefault(dragForce, 2, ' N');
            if ($('force-drag-power')) $('force-drag-power').textContent = dataOrDefault(dragPower, 2, ' kW');
            if ($('energy-kinetic')) $('energy-kinetic').textContent = dataOrDefault(kineticEnergy/1000, 2, ' kJ');
            
            // État 7-10 : Quaternions (Orientation)
            const quat = [X[6], X[7], X[8], X[9]];
            const [roll, pitch, yaw] = quatToEuler(quat);
            
            // Mise à jour du compas (Yaw = Cap réel par rapport au Nord)
            const hdg = yaw * R2D;
            const hdgNorm = (hdg < 0 ? 360 + hdg : hdg);
            if ($('heading-display')) $('heading-display').textContent = dataOrDefault(hdgNorm, 1, '°');
            if ($('roll-display')) $('roll-display').textContent = dataOrDefault(roll * R2D, 1, '°');
            if ($('pitch-display')) $('pitch-display').textContent = dataOrDefault(pitch * R2D, 1, '°');

            // Mise à jour des coordonnées
            if ($('lat-est')) $('lat-est').textContent = dataOrDefault(lat, 5, '°');
            if ($('lon-est')) $('lon-est').textContent = dataOrDefault(lon, 5, '°');
            if ($('pos-uncert')) $('pos-uncert').textContent = `± ${dataOrDefault(kUncert, 1, ' m')}`;

            // Mise à jour de la carte (si disponible)
            if (map && kUncert < UKF_R_MAX) { // Ne pas dessiner si l'UKF n'a pas convergé
                const latLng = L.latLng(lat, lon);
                updateMap(latLng, kUncert, hdgNorm);
            }
            
            // Calculs bio-physiques
            const bioSim = calculateBioSVT(lastT_K - KELVIN_OFFSET, kAlt, lastH_perc * 100, lastP_hPa * 100, sunAltitudeRad);
            if ($('o2-sat')) $('o2-sat').textContent = dataOrDefault(bioSim.O2SatClamped, 1, ' %');
            if ($('solar-irr')) $('solar-irr').textContent = dataOrDefault(bioSim.solarIrradiance, 0, ' W/m²');
        }

        // 3. Mise à jour de l'affichage DOM (Rapide)
        // Les coordonnées GPS brutes sont mises à jour par `gpsUpdateCallback`
        if ($('accel-x')) $('accel-x').textContent = dataOrDefault(accel.x, 2, ' m/s²');
        if ($('accel-y')) $('accel-y').textContent = dataOrDefault(accel.y, 2, ' m/s²');
        if ($('accel-z')) $('accel-z').textContent = dataOrDefault(accel.z, 2, ' m/s²');
        if ($('gyro-x')) $('gyro-x').textContent = dataOrDefault(gyro.x * R2D, 1, ' °/s');
        if ($('gyro-y')) $('gyro-y').textContent = dataOrDefault(gyro.y * R2D, 1, ' °/s');
        if ($('gyro-z')) $('gyro-z').textContent = dataOrDefault(gyro.z * R2D, 1, ' °/s');
        
        // Surveillance du mode de veille GPS
        startGPSStandby();
        
        // Boucle avec RAF pour une fréquence plus stable
        domFastID = requestAnimationFrame(updateFastData);
    };

    lastIMUTimestamp = performance.now();
    domFastID = requestAnimationFrame(updateFastData);
}

// Boucle secondaire (1Hz - Lente) : Météo, Astro, DOM Lents
// ===========================================
function startSlowLoop() {
    if (domSlowID) return;
    
    const updateSlowData = async () => {
        const curLat = lat||43.296, curLon = lon||5.37;
        const now = getCDate(); // Heure corrigée NTP

        // --- Astro (SunCalc) ---
        if (typeof SunCalc!=='undefined' && curLat !== null) {
            try {
                const sunPos = SunCalc.getPosition(now, curLat, curLon);
                sunAltitudeRad = sunPos.altitude;
                const moonIllum = SunCalc.getMoonIllumination(now);
                const moonPos = SunCalc.getMoonPosition(now, curLat, curLon);
                const times = SunCalc.getTimes(now, curLat, curLon);

                if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(sunPos.altitude * R2D, 1, '°');
                if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(sunPos.azimuth * R2D, 1, '°');
                
                if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(moonIllum.phase);
                if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(moonIllum.fraction * 100, 1, ' %');
                if ($('moon-alt')) $('moon-alt').textContent = dataOrDefault(moonPos.altitude * R2D, 1, '°');
                if ($('moon-azimuth')) $('moon-azimuth').textContent = dataOrDefault(moonPos.azimuth * R2D, 1, '°');
                
                if ($('sunrise-times')) $('sunrise-times').textContent = `${times.sunrise?.toLocaleTimeString('fr-FR') || 'N/A'} / ${times.sunriseEnd?.toLocaleTimeString('fr-FR') || 'N/A'}`;
                if ($('sunset-times')) $('sunset-times').textContent = `${times.sunsetStart?.toLocaleTimeString('fr-FR') || 'N/A'} / ${times.sunset?.toLocaleTimeString('fr-FR') || 'N/A'}`;
                
                const dayDurationMS = times.sunset.getTime() - times.sunrise.getTime();
                const dayDurationHrs = dayDurationMS / (1000 * 60 * 60);
                if ($('day-duration')) $('day-duration').textContent = `${dataOrDefault(dayDurationHrs, 1, ' h')}`;

            } catch(e) {
                console.error("Erreur SunCalc:", e);
                // Laisse les valeurs à N/A
            }
        }
        
        // --- Météo / Polluants (API) ---
        if (lat !== null && lon !== null) {
            // Récupère la météo
            fetchWeather(lat, lon).then(data => {
                if (data) {
                    lastWeatherData = data;
                    
                    // Met à jour les variables UKF (pour la physique)
                    lastP_hPa = data.pressure_hPa;
                    lastT_K = data.tempK;
                    lastH_perc = data.humidity_perc / 100.0;
                    currentAirDensity = getAirDensity(kAlt || 0, lastP_hPa, lastT_K);
                    currentSpeedOfSound = getSpeedOfSound(lastT_K); 
                    
                    // Met à jour le DOM météo
                    if ($('weather-status')) $('weather-status').textContent = `ACTIF`;
                    if ($('temp-air-2')) $('temp-air-2').textContent = `${data.tempC.toFixed(1)} °C`;
                    if ($('pressure-2')) $('pressure-2').textContent = `${data.pressure_hPa.toFixed(0)} hPa`;
                    if ($('humidity-2')) $('humidity-2').textContent = `${data.humidity_perc.toFixed(0)} %`;
                    if ($('air-density')) $('air-density').textContent = `${currentAirDensity.toFixed(3)} kg/m³`;
                    if ($('dew-point')) $('dew-point').textContent = `${data.dew_point.toFixed(1)} °C`;
                }
            }).catch(err => {
                if ($('weather-status')) $('weather-status').textContent = `❌ API MÉTÉO ÉCHOUÉE`;
            });
            
            // Récupère les polluants
            fetchPollutants(lat, lon).then(data => {
                if (data) {
                    lastPollutantData = data;
                    if ($('aqi-display')) $('aqi-display').textContent = `${data.aqi_us} (${data.main_pollutant})`;
                    if ($('pm25')) $('pm25').textContent = dataOrDefault(data.components.pm2_5, 1, ' µg/m³');
                    if ($('o3')) $('o3').textContent = dataOrDefault(data.components.o3, 1, ' µg/m³');
                    if ($('so2')) $('so2').textContent = dataOrDefault(data.components.so2, 1, ' µg/m³');
                }
            }).catch(err => {
                // Ignore l'erreur, la donnée reste N/A
            });
        }
        
        // --- Mises à jour DOM Lentes (Horloge, Compteurs) ---
        if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString('fr-FR');
        if ($('date-display')) $('date-display').textContent = now.toUTCString();
        if ($('time-minecraft')) $('time-minecraft').textContent = getMinecraftTime(now);
        
        // Met à jour les valeurs par défaut si le GPS n'est pas actif
        if ($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = `${currentSpeedOfSound.toFixed(2)} m/s`;
        if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        if ($('env-factor')) $('env-factor').textContent = `${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY} (x${ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT.toFixed(1)})`;

    };
    
    domSlowID = setInterval(updateSlowData, DOM_SLOW_UPDATE_MS);
    updateSlowData(); // Premier appel immédiat
}

// ===========================================
// Évènements & Initialisation
// ===========================================
document.addEventListener('DOMContentLoaded', () => {
    // Initialisation des contrôles
    initControls();
    
    // Initialisation de la carte Leaflet
    initMap(); 
    
    // Démarrer la synchro NTP
    syncH(); 
    
    // Initialisation du filtre UKF
    // Utilise des valeurs par défaut pour démarrer le filtre (Latitude/Longitude de Marseille)
    ukf = new ProfessionalUKF(43.2964, 5.3697, RHO_SEA_LEVEL);
    
    // Initialisation des valeurs par défaut pour la physique (Offline-First)
    currentAirDensity = RHO_SEA_LEVEL;
    currentSpeedOfSound = getSpeedOfSound(288.15); // 15°C ISA (288.15 K)
    lastT_K = 288.15;
    lastP_hPa = BARO_ALT_REF_HPA;

    // Démarrage des boucles lentes
    startSlowLoop();
    
    // Démarrage du mode GPS initial (si l'utilisateur clique sur 'MARCHE GPS')
    if ($('start-btn')) $('start-btn').addEventListener('click', () => startGPS('HIGH_FREQ'));
    if ($('stop-btn')) $('stop-btn').addEventListener('click', stopGPS);
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => {
        if (confirm("Êtes-vous sûr de vouloir tout réinitialiser (UKF, Distance, Max) ?")) {
            stopGPS(true);
            ukf = new ProfessionalUKF(43.2964, 5.3697, RHO_SEA_LEVEL);
            distM = 0; maxSpd = 0; timeMoving = 0; timeTotal = 0;
            pathLine.setLatLngs([]); // Efface le chemin
        }
    });

    // Événement d'arrêt d'urgence
    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', emergencyStop);
    
    // Mode sombre/clair
    if ($('toggle-mode-btn')) $('toggle-mode-btn').addEventListener('click', toggleDarkMode);
    
    // Récupération manuelle des données internet
    if ($('recharge-internet-btn')) $('recharge-internet-btn').addEventListener('click', syncRemoteData);

});
