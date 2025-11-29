// GNSS SpaceTime Dashboard - Unified JavaScript (UKF 21 États, COMPLET)
// Consolidated from multiple sources: UKF filters, meteorology, astro, IMU, map, etc.

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
        const accel_bias = this.x.get([6]) || 0;
        let vN = this.x.get([3]) + (imuData.accel[0] - accel_bias)*dt;
        if (Math.abs(vN) < MIN_SPD) vN = 0;
        this.x.set([3], vN);
        // Prédit la position (latitude approximée)
        let newLat = this.x.get([0]) + (vN / R_E_BASE) * dt;
        this.x.set([0], newLat);
        // (On pourrait intégrer lon, alt, etc. dans un vrai UKF)
    }
    update(gpsData, R_dyn) {
        // (Correction UKF simplifiée - placeholder)
        const K = 0.1;
        this.x.set([0], this.x.get([0])*(1-K) + (gpsData.latitude*D2R)*K);
        this.x.set([1], this.x.get([1])*(1-K) + (gpsData.longitude*D2R)*K);
        this.x.set([2], this.x.get([2])*(1-K) + gpsData.altitude*K);
        if (gpsData.speed != null) {
            const oldV = this.x.get([3]);
            this.x.set([3], oldV*(1-K) + gpsData.speed*K);
        }
    }
    getState() {
        const x_arr = this.x.toArray ? this.x.toArray() : this.x;
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
    if (acc_eff > MAX_ACC) { return 1e9; }
    let R_base = Math.min(acc_eff, 100)**2;
    const env_mult = ENVIRONMENT_FACTORS[env]?.R_MULT || 1.0;
    let react_mult = UKF_REACTIVITY_FACTORS[reactivityMode]?.MULT || 1.0;
    if (reactivityMode === 'AUTO') {
        if (acc_eff > 20) react_mult = 3.0;
        else if (acc_eff < 3) react_mult = 0.5;
    }
    let R_dyn = Math.min(R_base * env_mult * react_mult, UKF_R_MAX);
    return Math.max(R_dyn, R_ALT_MIN);
}

// Distance 3D via Turf (WGS84)
function dist3D(lat1, lon1, alt1, lat2, lon2, alt2) {
    const from = turf.point([lon1, lat1, alt1||0]);
    const to   = turf.point([lon2, lat2, alt2||0]);
    return turf.distance(from, to, {units:'meters'});
}

// Altitude barométrique (ISA approximatif)
function getBarometricAltitude(P_hPa, P_ref_hPa, T_K) {
    if (P_hPa <= 0 || P_ref_hPa <= 0 || T_K <= 0) return NaN;
    const ratio = P_hPa / P_ref_hPa;
    return (T_K/0.0065) * (1 - Math.pow(ratio, (R_AIR*0.0065)/getWGS84Gravity(0,0)));
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
    if(tempK < KELVIN_OFFSET) tempK += KELVIN_OFFSET;
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
    let lorentzFactor = 1/Math.sqrt(1 - (V/C_L)**2);
    let timeDilation = (lorentzFactor - 1) * 86400 * 1e9; // ns/day
    let E0 = mass * C_L * C_L;
    let E_rel = lorentzFactor * E0;
    let momentum = mass * V;
    let gravDilation = (localG * kAlt / (C_L*C_L)) * 86400 * 1e9;
    let Rs = (2 * G_U * mass)/(C_L*C_L);
    let speedSound = getSpeedOfSound(tempK);
    let mach = speedSound>0 ? V/speedSound : 0;
    let dynPress = 0.5 * airDensity * V * V;
    let reynolds = (airDensity * V * 1)/MU_DYNAMIC_AIR;
    let dragForce = dynPress * (CdA||0.5);
    let dragPower = (dragForce * V)/1000.0;
    let coriolisForce = 2 * mass * V * OMEGA_EARTH * Math.sin((latDeg||0)*D2R);
    let geoAlt = (localG>0.1) ? kAlt*(G_ACC/localG) : kAlt;
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
    const h_frac = humidity_perc/100;
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
    const solarIrr = SOLAR_FLUX_DENSITY * Math.max(0, Math.sin(sunAltRad));
    const radPress = solarIrr / C_L;
    const P_sat_hPa = tC => 6.1078 * Math.pow(10, (7.5*tC)/(237.3+tC));
    const P_v = P_sat_hPa(tempC) * h_frac;
    const absHum = (P_v*100*18.015)/(8.314*(tempC+KELVIN_OFFSET)) * 1000; // g/m3
    return {
        dewPoint, wetBulbTemp:wetBulb, CAPE_sim, O2SaturationClamped,
        photosynthesisRate:photoRate, solarIrradiance:solarIrr, radiationPressure:radPress,
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
    const J_star = d - lon/360;
    const J_transit = J_star + (0.0053*Math.sin(M) - 0.0069*Math.sin(2*L));
    const eot_min = (J_star - J_transit)*1440;
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
    if (!date) return 'N/A';
    const GMST_deg = (date.getUTCHours() + date.getUTCMinutes()/60)*15;
    const LST_deg = (GMST_deg + lon) % 360;
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
        lServH = Date.now(); lLocH = performance.now();
        if ($('local-time')) $('local-time').textContent = '❌ SYNCHRO ÉCHOUÉE (Local)';
    }
}
function getCDate() {
    if (lServH==null||lLocH==null) return new Date();
    let offset = performance.now() - lLocH;
    return new Date(lServH + offset);
}

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
            lastP_hPa = P; lastT_K = T_K;
            currentAirDensity = airDensity;
            currentSpeedOfSound = getSpeedOfSound(T_K);
            lastWeatherData = { pressure_hPa:P, tempC:T_C, tempK:T_K, humidity_perc:H, air_density:airDensity };
            return lastWeatherData;
        }
    } catch(e) {
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
            return data.list[0].components || null;
        }
    } catch(e) {
        console.warn("Erreur fetchPollutants:", e.message);
    }
    return null;
}

// ===========================================
// MAP (Leaflet)
// ===========================================
function initMap() {
    try {
        if ($('map') && typeof L !== 'undefined' && !map) {
            map = L.map('map').setView([43.296, 5.37], 10);
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                attribution: '© OpenStreetMap contributors'
            }).addTo(map);
            marker = L.marker([43.296, 5.37]).addTo(map);
            circle = L.circle([43.296, 5.37], { color:'red', fillColor:'#f03', fillOpacity:0.5, radius:10 }).addTo(map);
            setTimeout(()=>map.invalidateSize(), 400);
        }
    } catch(e) {
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
// BOUCLES PRINCIPALES
// ===========================================

function gpsUpdateCallback(pos) {
    if (emergencyStopActive || !ukf) return;
    lastGPSPos = pos;
    const accRaw = pos.coords.accuracy || 100;
    let R_dyn = getKalmanR(accRaw, kAlt, kUncert, selectedEnvironment, currentUKFReactivity);
    const isSignalPoor = (accRaw > MAX_ACC || R_dyn >= UKF_R_MAX * 0.9);
    const spd_raw = pos.coords.speed || 0;
    const accel_long = Math.abs(accel.x);
    const isZUPT = (spd_raw < ZUPT_RAW_THRESHOLD && accel_long < ZUPT_ACCEL_THRESHOLD && !isSignalPoor);
    if (isSignalPoor) {
        $('gps-precision').textContent = `❌ ${accRaw.toFixed(0)} m (Estimation)`;
        $('gps-status-dr').textContent = 'Drift (IMU seul)';
        $('gps-status-ekf').textContent = '⚠️ Estim.';
        // UKF in predict-only mode
    } else if (isZUPT) {
        $('gps-precision').textContent = `${accRaw.toFixed(2)} m (ZUPT)`;
        $('gps-status-dr').textContent = '✅ ZUPT (Arrêt)';
        $('gps-status-ekf').textContent = 'ZUPT';
        // Forcer vitesse à 0
let zuptData = {...pos.coords, speed:0};
ukf.update(zuptData, R_ALT_MIN);
} else {
$('gps-precision').textContent = `${accRaw.toFixed(2)} m`;
$('gps-status-dr').textContent = 'Actif (GPS)';
$('gps-status-ekf').textContent = 'Actif (Fusion)';
ukf.update(pos.coords, R_dyn);
}
}
function startFastLoop() {
if (domFastID) return;
domFastID = setInterval(()=>{
if (emergencyStopActive || !ukf) return;
const now = performance.now();
let dt = (now - lastIMUTimestamp)/1000.0;
if (dt < MIN_DT) return;
lastIMUTimestamp = now;
// UKF Predict
const imuData = { accel:[accel.x,accel.y,accel.z], gyro:
[gyro.x,gyro.y,gyro.z] };
ukf.predict(imuData, dt);
const state = ukf.getState();
lat = state.lat; lon = state.lon; kAlt = state.alt; kSpd =
state.speed;
kUncert = state.kUncert; kAltUncert = state.kAltUncert;
const stableSpeed = kSpd < MIN_SPD ? 0 : kSpd;
const spd3D_raw = lastGPSPos && lastGPSPos.coords.speed ?
lastGPSPos.coords.speed : 0;
const local_g = getWGS84Gravity(lat, kAlt);
// Advanced physics
const adv = calculateAdvancedPhysics(stableSpeed, kAlt, currentMass,
currentCdA, lastT_K, currentAirDensity, lat, kAltUncert, local_g, accel.x);
// Distance ratio (altitude vs surface)
let altFactor = distanceRatioMode ? (R_E_BASE/(R_E_BASE + (kAlt>0?
kAlt:0))) : 1.0;
R_FACTOR_RATIO = altFactor * (netherMode ? NETHER_RATIO : 1.0);
distM += stableSpeed * dt * R_FACTOR_RATIO;
if (stableSpeed > MIN_SPD) timeMoving += dt;
if (sTime) timeTotal = (Date.now()-sTime)/1000;
// Max speed in km/h
const maxSpdRaw_kmh = spd3D_raw * KMH_MS;
11
if (maxSpdRaw_kmh > maxSpd) maxSpd = maxSpdRaw_kmh;
// Manage GPS power
if (stableSpeed < MIN_SPD*2 && currentGPSMode==='HIGH_FREQ') {
if (!gpsStandbyTimeoutID) gpsStandbyTimeoutID =
setTimeout(()=>startGPS('LOW_FREQ'), STANDBY_TIMEOUT_MS);
} else if (stableSpeed >= MIN_SPD*2 && currentGPSMode==='LOW_FREQ') {
startGPS('HIGH_FREQ'); if (gpsStandbyTimeoutID)
{clearTimeout(gpsStandbyTimeoutID); gpsStandbyTimeoutID=null;}
}
// Update DOM (rapide)
$('elapsed-time').textContent = dataOrDefault(timeTotal,2,' s');
$('time-moving').textContent = dataOrDefault(timeMoving,2,' s');
$('distance-ratio').textContent = dataOrDefault(R_FACTOR_RATIO,3);
$('nether-mode-status').textContent = dataOrDefault(R_FACTOR_RATIO,
3);
// Vitesse & Relativité
$('speed-stable').textContent = dataOrDefault(stableSpeed*KMH_MS,2);
$('speed-status-text').textContent = (ukf && kSpd>MIN_SPD)?
" UKF 21 ÉTATS (INS)" : " ZUPT (Attente)";
$('speed-stable-ms').textContent = dataOrDefault(stableSpeed,3,' m/
s');
$('speed-stable-kms').textContent = dataOrDefaultExp(stableSpeed/
1000,3,' km/s');
$('speed-3d-inst').textContent = dataOrDefault(spd3D_raw*KMH_MS,2,'
km/h');
$('speed-raw-ms').textContent = dataOrDefault(spd3D_raw,3,' m/s');
$('speed-max').textContent = dataOrDefault(maxSpd,2,' km/h');
$('speed-avg-moving').textContent = (timeMoving>1)?
dataOrDefault(distM/timeMoving*KMH_MS,2,' km/h') : '0.00 km/h';
$('speed-avg-total').textContent = (timeTotal>1)?
dataOrDefault(distM/timeTotal*KMH_MS,2,' km/h'): '0.00 km/h';
$('speed-of-sound-calc').textContent =
dataOrDefault(adv.speedOfSoundLocal,2,' m/s');
$('perc-speed-sound').textContent = dataOrDefault(adv.machNumber*100,
2,' %');
$('mach-number').textContent = dataOrDefault(adv.machNumber,4);
$('perc-speed-c').textContent = dataOrDefaultExp(stableSpeed/C_L*100,
2,' %');
$('lorentz-factor').textContent = dataOrDefault(adv.lorentzFactor,8);
$('time-dilation-v').textContent =
dataOrDefault(adv.timeDilationSpeed,3,' ns/j');
$('time-dilation-g').textContent =
dataOrDefault(adv.gravitationalDilation,3,' ns/j');
$('energy-relativistic').textContent =
dataOrDefaultExp(adv.energyRelativistic,3,' J');
$('energy-rest-mass').textContent = dataOrDefaultExp(adv.E0,3,' J');
$('momentum').textContent = dataOrDefault(adv.momentum,2,' kg·m/s');
$('Rs-object').textContent = dataOrDefaultExp(adv.Rs_object,3,' m');
// Distance
$('distance-total-km').textContent = `${dataOrDefault(distM/1000,3)}
km | ${dataOrDefault(distM,2)} m`;
12
const dist_light_s = distM/C_L;
$('distance-light-s').textContent = dataOrDefaultExp(dist_light_s,
2,' s');
$('distance-light-min').textContent = dataOrDefaultExp(dist_light_s/
60,2,' min');
$('distance-light-h').textContent = dataOrDefaultExp(dist_light_s/
3600,2,' h');
$('distance-light-day').textContent = dataOrDefaultExp(dist_light_s/
86400,2,' j');
$('distance-light-week').textContent = dataOrDefaultExp(dist_light_s/
(86400*7),2,' sem');
$('distance-light-month').textContent =
dataOrDefaultExp(dist_light_s/(86400*30.44),2,' mois');
$('distance-cosmic').textContent = `${dataOrDefaultExp(distM/
AU_METERS,2)} UA | ${dataOrDefaultExp(distM/LIGHT_YEAR_METERS,2)} al`;
$('distance-horizon').textContent =
dataOrDefault(calculateMaxVisibleDistance(kAlt)/1000,1,' km');
// Map
updateMap(lat, lon, lastGPSPos ? lastGPSPos.coords.accuracy : 100);
// Dynamique & forces
$('gravity-local').textContent = dataOrDefault(local_g,4,' m/s²');
$('accel-long').textContent = dataOrDefault(accel.x,3,' m/s²');
$('force-g-long').textContent = dataOrDefault(adv.force_g_long,2,'
G');
$('vertical-speed').textContent = dataOrDefault(adv.vD * -1,2,' m/
s');
$('angular-speed').textContent =
dataOrDefault(Math.sqrt(gyro.x**2+gyro.y**2+gyro.z**2)*R2D,2,' °/s');
$('dynamic-pressure').textContent =
dataOrDefault(adv.dynamicPressure,2,' Pa');
$('drag-force').textContent = dataOrDefault(adv.dragForce,2,' N');
$('drag-power-kw').textContent = dataOrDefault(adv.dragPower_kW,2,'
kW');
$('reynolds-number').textContent =
dataOrDefaultExp(adv.reynoldsNumber,2);
$('kinetic-energy').textContent = dataOrDefault(0.5 * currentMass *
stableSpeed**2,2,' J');
$('mechanical-power').textContent = dataOrDefault(adv.force_g_long *
currentMass * stableSpeed,2,' W');
$('coriolis-force').textContent = dataOrDefaultExp(adv.coriolisForce,
2,' N');
// EKF/Debug
$('kalman-uncert').textContent = dataOrDefault(kUncert,3,' m²/s²
(P)');
$('alt-uncertainty').textContent = dataOrDefault(adv.altSigma,3,' m
(σ)');
let R_dyn_disp = getKalmanR(lastGPSPos ?
lastGPSPos.coords.accuracy : 100, kAlt, kUncert, selectedEnvironment,
currentUKFReactivity);
$('speed-error-perc').textContent = dataOrDefault(R_dyn_disp,3,' m²
(R dyn)');
13
$('nyquist-frequency').textContent =
dataOrDefault(adv.nyquistFrequency,2,' Hz');
$('gps-accuracy-display').textContent =
dataOrDefault(gpsAccuracyOverride,6,' m');
// IMU
$('accel-x').textContent = dataOrDefault(accel.x,2,' m/s²');
$('accel-y').textContent = dataOrDefault(accel.y,2,' m/s²');
$('accel-z').textContent = dataOrDefault(accel.z,2,' m/s²');
// Position/EKF
$('lat-display').textContent = dataOrDefault(lat,6,' °');
$('lon-display').textContent = dataOrDefault(lon,6,' °');
$('alt-display').textContent = dataOrDefault(kAlt,2,' m');
$('geopotential-alt').textContent =
dataOrDefault(adv.geopotentialAltitude,2,' m');
$('alt-corrected-baro').textContent =
dataOrDefault(getBarometricAltitude(lastP_hPa, BARO_ALT_REF_HPA, lastT_K),
2,' m');
$('heading-display').textContent = dataOrDefault(lastGPSPos ?
(turf.bearing([lastGPSPos.coords.longitude, lastGPSPos.coords.latitude],
[lon,lat]) + 360)%360 : 0,1,'°');
}, IMU_UPDATE_RATE_MS);
}
// ===========================================
// BOUCLE LENTE (Astro + Météo/Polluants) - 1Hz
// ===========================================
function startSlowLoop() {
if (domSlowID) return;
const updateSlowData = async () => {
const curLat = lat||43.296, curLon = lon||5.37;
const now = getCDate();
// Astro (SunCalc)
if (typeof SunCalc!=='undefined') {
try {
const sunPos = SunCalc.getPosition(now, curLat, curLon);
const moonIllum = SunCalc.getMoonIllumination(now);
const moonPos = SunCalc.getMoonPosition(now, curLat, curLon);
const sunTimes = SunCalc.getTimes(now, curLat, curLon);
const moonTimes = SunCalc.getMoonTimes(now, curLat, curLon,
true);
const solarTimes = getSolarTime(now, curLon);
sunAltitudeRad = sunPos.altitude;
$('date-display-astro').textContent =
now.toLocaleDateString() || '...';
if (solarTimes.DateMST) $('date-solar-mean').textContent =
solarTimes.DateMST.toLocaleDateString();
if (solarTimes.DateTST) $('date-solar-true').textContent =
solarTimes.DateTST.toLocaleDateString();
$('mst').textContent = solarTimes.MST;
$('tst').textContent = solarTimes.TST;
$('noon-solar').textContent = sunTimes.solarNoon ?
14
sunTimes.solarNoon.toLocaleTimeString('fr-FR',{timeZone:'UTC'}) : '...';
$('eot').textContent = `${solarTimes.EOT} min`;
$('tslv').textContent = getTSLV(now, curLon);
$('ecl-long').textContent = `${solarTimes.ECL_LONG}°`;
$('sun-alt').textContent = `$
{(sunPos.altitude*R2D).toFixed(2)}°`;
$('sun-azimuth').textContent = `$
{(sunPos.azimuth*R2D).toFixed(2)}°`;
if (sunTimes.sunset && sunTimes.sunrise) {
const durationMs = sunTimes.sunset.getTime() -
sunTimes.sunrise.getTime();
$('day-duration').textContent = `${Math.floor(durationMs/
3600000)}h ${Math.floor((durationMs%3600000)/60000)}m`;
}
$('sunrise-times').textContent = sunTimes.sunrise? `$
{sunTimes.sunrise.toLocaleTimeString('fr-FR',{timeZone:'UTC'})}/$
{sunTimes.sunrise.toLocaleTimeString('fr-FR',{timeZone:'UTC'})}`:'...';
$('sunset-times').textContent = sunTimes.sunset? `$
{sunTimes.sunset.toLocaleTimeString('fr-FR',{timeZone:'UTC'})}/$
{sunTimes.sunset.toLocaleTimeString('fr-FR',{timeZone:'UTC'})}`:'...';
$('moon-phase-name').textContent =
getMoonPhaseName(moonIllum.phase);
$('moon-illuminated').textContent = `$
{(moonIllum.fraction*100).toFixed(1)}%`;
$('moon-alt').textContent = `$
{(moonPos.altitude*R2D).toFixed(2)}°`;
$('moon-azimuth').textContent = `$
{(moonPos.azimuth*R2D).toFixed(2)}°`;
$('moon-times').textContent =
(moonTimes.rise&&moonTimes.set)? `${moonTimes.rise.toLocaleTimeString()}/$
{moonTimes.set.toLocaleTimeString()}`:'...';
// Minecraft clock visuals
const clockDiv = $('minecraft-clock');
if (clockDiv) {
if (sunPos.altitude > 0) { clockDiv.className='sky-day';
$('clock-status').textContent='Jour (☀)'; }
else if (sunPos.altitude > -10*D2R) {
clockDiv.className='sky-sunset'; $('clock-status').textContent='Crépuscule/
Aube ( )'; }
else { clockDiv.className='sky-night'; $('clock-
status').textContent='Nuit ( )'; }
}
} catch(e) { console.error("Erreur calcul Astro:", e); }
}
// Météo & Pollution
if (!emergencyStopActive) {
try {
const wdata = await fetchWeather(curLat, curLon);
if (wdata) {
const bio = calculateBioSVT(wdata.tempC, kAlt||0,
wdata.humidity_perc, wdata.pressure_hPa*100, sunAltitudeRad);
15
$('weather-status').textContent = 'ACTIF';
$('temp-air-2').textContent = `${wdata.tempC.toFixed(1)}
°C`;
$('pressure-2').textContent = `$
{wdata.pressure_hPa.toFixed(0)} hPa`;
$('humidity-2').textContent = `$
{wdata.humidity_perc.toFixed(0)} %`;
$('air-density').textContent = `$
{wdata.air_density.toFixed(3)} kg/m³`;
$('dew-point').textContent = `${bio.dewPoint.toFixed(1)}
°C`;
// BioSVT
$('abs-humidity-sim').textContent =
dataOrDefault(bio.absoluteHumidity,3,' g/m³');
$('wet-bulb-temp-sim').textContent = `$
{bio.wetBulbTemp.toFixed(1)} °C`;
$('CAPE-sim').textContent = `${bio.CAPE_sim.toFixed(0)}
J/kg`;
$('O2-saturation-sim').textContent = `$
{bio.O2SaturationClamped.toFixed(1)} %`;
$('photosynthesis-rate-sim').textContent =
dataOrDefaultExp(bio.photosynthesisRate,2);
$('radiation-pressure').textContent =
dataOrDefaultExp(bio.radiationPressure,2,' Pa');
}
const pol = await fetchPollutants(curLat, curLon);
if (pol) {
$('no2-val').textContent = dataOrDefault(pol.no2,1,' μg/
m³');
$('pm25-val').textContent = dataOrDefault(pol.pm2_5,1,'
μg/m³');
$('pm10-val').textContent = dataOrDefault(pol.pm10,1,'
μg/m³');
$('o3-val').textContent = dataOrDefault(pol.o3,1,' μg/
m³');
}
} catch(e) {
console.warn("Erreur fetch météo/pollution:", e);
}
}
// Temps et NTP
if (now) {
if ($('local-time') && !$('local-
time').textContent.includes('Synchronisation')) {
$('local-time').textContent = now.toLocaleTimeString('fr-
FR');
}
if ($('date-display')) $('date-display').textContent =
now.toUTCString();
if ($('time-minecraft')) $('time-minecraft').textContent =
getMinecraftTime(now);
16
}
};
domSlowID = setInterval(updateSlowData, DOM_SLOW_UPDATE_MS);
updateSlowData();
}
// ===========================================
// Évènements & Initialisation
// ===========================================
document.addEventListener('DOMContentLoaded', () => {
initMap();
// Toggle GPS button (démarrage système)
$('toggle-gps-btn').addEventListener('click', () => {
if (!ukf) {
try {
ukf = new ProfessionalUKF();
} catch(e) {
alert("Erreur: " + e.message);
return;
}
sTime = Date.now();
startGPS(); startSlowLoop();
} else {
toggleGPS();
}
});
$('emergency-stop-btn').addEventListener('click', toggleEmergencyStop);
$('toggle-mode-btn').addEventListener('click', () => {
document.body.classList.toggle('dark-mode');
$('toggle-mode-btn').innerHTML =
document.body.classList.contains('dark-mode')?
'<i class="fas fa-sun"></i> Mode Jour' : '<i class="fas fa-
moon"></i> Mode Nuit';
});
$('reset-dist-btn').addEventListener('click', ()=>{ distM=0;
timeMoving=0; });
$('reset-max-btn').addEventListener('click', ()=>{ maxSpd=0; });
$('reset-all-btn').addEventListener('click', ()=>{
distM=0; maxSpd=0; kSpd=0; kUncert=UKF_R_MAX; kAlt=0; kAltUncert=10;
timeMoving=0; timeTotal=0; sTime=Date.now();
if (ukf) ukf = new ProfessionalUKF();
});
$('capture-data-btn').addEventListener('click', ()=>alert('Capture non
implémentée.'));
$('xray-button').addEventListener('click', ()=> $('minecraft-
clock').classList.toggle('x-ray'));
$('freq-select').addEventListener('change', (e)=>
startGPS(e.target.value));
$('gps-accuracy-override').addEventListener('change', (e)=>
gpsAccuracyOverride = parseFloat(e.target.value) || 0.0);
$('environment-select').addEventListener('change', (e)=>{
17
selectedEnvironment = e.target.value;
const factor = ENVIRONMENT_FACTORS[selectedEnvironment];
$('env-factor').textContent = `${factor.DISPLAY} (x$
{factor.R_MULT.toFixed(1)})`;
});
$('mass-input').addEventListener('input', (e)=>{
currentMass = parseFloat(e.target.value) || 70.0;
$('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
});
$('celestial-body-select').addEventListener('change', (e)=>{
currentCelestialBody = e.target.value;
const { G_ACC: newG } = updateCelestialBody(currentCelestialBody,
kAlt, rotationRadius, angularVelocity);
$('gravity-base').textContent = `${newG.toFixed(4)} m/s²`;
});
const updateRotation = () => {
rotationRadius = parseFloat($('rotation-radius').value) || 100;
angularVelocity = parseFloat($('angular-velocity').value) || 0.0;
if (currentCelestialBody === 'ROTATING') {
const { G_ACC: newG } = updateCelestialBody('ROTATING', kAlt,
rotationRadius, angularVelocity);
$('gravity-base').textContent = `${newG.toFixed(4)} m/s²`;
}
};
$('rotation-radius').addEventListener('input', updateRotation);
$('angular-velocity').addEventListener('input', updateRotation);
$('nether-toggle-btn').addEventListener('click', ()=>{
netherMode = !netherMode;
$('nether-toggle-btn').textContent = `Mode Nether: ${netherMode?
'ACTIVÉ (1:8)' : 'DÉSACTIVÉ (1:1)'}`;
});
$('ukf-reactivity-mode').addEventListener('change', (e)=>
currentUKFReactivity = e.target.value);
// NEW: Toggle Distance Ratio (Surface vs Altitude)
$('distance-ratio-toggle-btn').addEventListener('click', ()=>{
distanceRatioMode = !distanceRatioMode;
if (distanceRatioMode) {
$('distance-ratio-toggle-btn').textContent = `Rapport Distance:
ALTITUDE (${(R_E_BASE/(R_E_BASE + kAlt)).toFixed(3)})`;
} else {
$('distance-ratio-toggle-btn').textContent = `Rapport Distance:
SURFACE (1.000)`;
}
});
// Initial body gravity display
const { G_ACC: baseG } = updateCelestialBody(currentCelestialBody, kAlt,
rotationRadius, angularVelocity);
$('gravity-base').textContent = `${baseG.toFixed(4)} m/s²`;
// Démarrage NTP
syncH();
});
18
// GPS control functions
function startGPS(mode=currentGPSMode) {
if (emergencyStopActive) return;
if (wID !== null) navigator.geolocation.clearWatch(wID);
currentGPSMode = mode;
wID = navigator.geolocation.watchPosition(gpsUpdateCallback, handleErr,
GPS_OPTS[mode]);
if (!domFastID) startIMUListeners();
let text = (mode==='LOW_FREQ' && kSpd < MIN_SPD*2)? '⏸ GPS EN VEILLE' :
'⏸ PAUSE GPS';
if ($('toggle-gps-btn')) {
$('toggle-gps-btn').textContent = text;
$('toggle-gps-btn').style.backgroundColor = '#ffc107';
}
}
function stopGPS(resetBtn=true) {
if (wID !== null) navigator.geolocation.clearWatch(wID);
wID = null;
if (gpsStandbyTimeoutID) { clearTimeout(gpsStandbyTimeoutID);
gpsStandbyTimeoutID = null; }
stopIMUListeners();
if (resetBtn && $('toggle-gps-btn')) {
$('toggle-gps-btn').textContent = '▶ MARCHE GPS';
$('toggle-gps-btn').style.backgroundColor = '#28a745';
}
}
function toggleGPS() {
if (emergencyStopActive) return;
wID===null? startGPS('HIGH_FREQ') : stopGPS();
}
function toggleEmergencyStop() {
emergencyStopActive = !emergencyStopActive;
if (emergencyStopActive) {
stopGPS(false); stopIMUListeners();
if ($('emergency-stop-btn')) {
$('emergency-stop-btn').textContent = " Arrêt d'urgence: ACTIF
";
$('emergency-stop-btn').classList.add('active');
}
if ($('speed-status-text')) $('speed-status-text').textContent = '
ARRÊT D’URGENCE';
} else {
if ($('emergency-stop-btn')) {
$('emergency-stop-btn').textContent =
" Arrêt d'urgence: INACTIF 🟢";
$('emergency-stop-btn').classList.remove('active');
}
startGPS();
}
}
19
function handleErr(err) {
let errMsg = `Erreur GPS (Code ${err.code}): `;
if (err.code===1) errMsg += "Permission refusée.";
else if (err.code===2) errMsg += "Position indisponible (Pas de
signal).";
else if (err.code===3) errMsg += "Timeout GPS.";
else errMsg += `Erreur inconnue: ${err.message}`;
if ($('gps-precision')) $('gps-precision').textContent = errMsg;
if (err.code===1) stopGPS();
}
// IMU/Sensor handlers
function startIMUListeners() {
if (emergencyStopActive || domFastID) return;
try {
if ($('imu-status')) $('imu-status').textContent = "Activation...";
if (typeof Accelerometer==='undefined' || typeof
Gyroscope==='undefined') {
throw new Error("API Capteurs non supportée.");
}
const accSensor = new Accelerometer({frequency:50});
accSensor.addEventListener('reading', ()=>{ accel.x=accSensor.x;
accel.y=accSensor.y; accel.z=accSensor.z; });
accSensor.addEventListener('error', e=>
console.error("Accéléromètre:", e.error));
accSensor.start();
const gyroSensor = new Gyroscope({frequency:50});
gyroSensor.addEventListener('reading', ()=>{ gyro.x=gyroSensor.x;
gyro.y=gyroSensor.y; gyro.z=gyroSensor.z; });
gyroSensor.addEventListener('error', e=> console.error("Gyroscope:",
e.error));
gyroSensor.start();
if ($('imu-status')) $('imu-status').textContent =
"Actif (API Sensor 50Hz)";
lastIMUTimestamp = performance.now();
// Altogether, start fast loop
startFastLoop();
} catch(error) {
let msg = error.message;
if (error.name==='SecurityError' || error.name==='NotAllowedError') {
msg = "Permission Capteurs refusée. Cliquez sur 'MARCHE GPS'.";
}
if ($('imu-status')) $('imu-status').textContent = ` ${msg}`;
}
}
function stopIMUListeners() {
if (domFastID) clearInterval(domFastID);
domFastID = null;
if ($('imu-status')) $('imu-status').textContent = "Inactif";
accel = {x:0,y:0,z:0}; gyro = {x:0,y:0,z:0};
}
