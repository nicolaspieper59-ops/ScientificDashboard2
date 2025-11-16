// =================================================================
// BLOC 1/3 : Modèles Avancés (UKF, Quaternion, WGS84) & Filtres
// =================================================================

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const KMH_MS = 3.6;         // Conversion m/s vers km/h
const R_AIR = 287.058;      // Constante spécifique de l'air sec (J/kg·K)
const BARO_ALT_REF_HPA = 1013.25; // Pression atmosphérique standard au niveau de la mer
const RHO_SEA_LEVEL = 1.225; // Densité de l'air standard (kg/m³)
const ALT_TH = -50;         // Seuil d'altitude "Sous-sol"

// WGS84 Geodesy Parameters
const WGS84_A = 6378137.0;  // Rayon équatorial (m)
const WGS84_F = 1 / 298.257223563; // Inverse de l'aplatissement
const WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F; // Excentricité au carré
const WGS84_G_EQUATOR = 9.780327; // Gravité Standard à l'équateur (g0)
const WGS84_BETA = 0.0053024; // Différence de gravité
let G_ACC = WGS84_G_EQUATOR; // Gravité locale (variable globale, sera mise à jour)
let R_ALT_CENTER_REF = WGS84_A; // Rayon de référence (variable globale)

// --- PARAMÈTRES DU FILTRE DE KALMAN NON LINÉAIRE (UKF) ---
const UKF_Q_SPD = 0.001;    // Bruit de processus vitesse
const UKF_R_MIN = 0.001;    // Bruit de mesure minimum
const UKF_R_MAX = 500.0;    // Bruit de mesure maximum
const MIN_SPD = 0.05;       // Vitesse minimale "en mouvement"
const ZUPT_RAW_THRESHOLD = 0.1; // Vitesse brute max (m/s) pour ZUPT (plus strict)
const KAPPA = 0; // Paramètre UKF
const Q_ALT_NOISE = 0.01;
const R_ALT_MIN = 0.1;

// Facteurs Environnementaux (pour ajuster le bruit R)
const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0, DISPLAY: 'Normal' },
    'FOREST': { R_MULT: 2.5, DISPLAY: 'Forêt' },
    'CONCRETE': { R_MULT: 7.0, DISPLAY: 'Grotte/Tunnel' },
    'METAL': { R_MULT: 5.0, DISPLAY: 'Métal/Bâtiment' },
};
// Corps Célestes (pour ajuster la gravité et le rayon)
const CELESTIAL_DATA = {
    'EARTH': { G: WGS84_G_EQUATOR, R: WGS84_A, name: 'Terre' },
    'MOON': { G: 1.62, R: 1737400, name: 'Lune' },
    'MARS': { G: 3.71, R: 3389500, name: 'Mars' },
    'ROTATING': { G: 0.0, R: WGS84_A, name: 'Station Spatiale' }
};

// --- CLASSE QUATERNION (POUR L'ATTITUDE) ---
class Quaternion {
    constructor(w = 1, x = 0, y = 0, z = 0) {
        this.w = w; this.x = x; this.y = y; this.z = z;
    }
    normalize() {
        let mag = Math.sqrt(this.w ** 2 + this.x ** 2 + this.y ** 2 + this.z ** 2);
        if (mag > 1e-6) {
            this.w /= mag; this.x /= mag; this.y /= mag; this.z /= mag;
        }
        return this;
    }
    toEuler() { 
        const q = this.normalize();
        const w = q.w, x = q.x, y = q.y, z = q.z;
        // Roll (Rotation autour de X)
        const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x ** 2 + y ** 2));
        // Pitch (Rotation autour de Y)
        let sinp = 2 * (w * y - z * x);
        const pitch = Math.abs(sinp) >= 1 ? Math.sign(sinp) * Math.PI / 2 : Math.asin(sinp);
        // Yaw (Rotation autour de Z) - Non fiable sans magnétomètre
        const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y ** 2 + z ** 2));
        return { roll: roll * R2D, pitch: pitch * R2D, yaw: yaw * R2D };
    }
    static fromAcc(accX, accY, accZ) { 
        // Approximation d'attitude statique à partir de l'accélération (assumant accZ aligné avec la gravité)
        let roll = Math.atan2(accY, accZ);
        let pitch = Math.atan2(-accX, Math.sqrt(accY ** 2 + accZ ** 2));
        const cr = Math.cos(roll * 0.5), sr = Math.sin(roll * 0.5);
        const cp = Math.cos(pitch * 0.5), sp = Math.sin(pitch * 0.5);
        const cy = 1, sy = 0; // Yaw (Lacet) est arbitrairement à 0, car non mesurable sans magnétomètre
        const w = cr * cp * cy + sr * sp * sy;
        const x = sr * cp * cy - cr * sp * sy;
        const y = cr * sp * cy + sr * cp * sy;
        const z = cr * cp * sy - sr * sp * cy;
        return new Quaternion(w, x, y, z).normalize();
    }
}

// --- CLASSE UKF (UNSCENTED KALMAN FILTER) 1D ---
class UKF {
    constructor(initialState, initialCovariance, processNoise, measurementNoise, kappa) {
        this.x = initialState[0]; 
        this.P = initialCovariance[0]; 
        this.Q = processNoise; 
        this.R = measurementNoise; 
        this.n = 1; 
        this.kappa = kappa;
        this.lambda = kappa; 
        this.gamma = Math.sqrt(this.n + this.lambda);
        this.weightsM = []; this.weightsC = [];
        this._calculateWeights();
    }
    _calculateWeights() {
        this.weightsM[0] = this.lambda / (this.n + this.lambda);
        this.weightsC[0] = this.lambda / (this.n + this.lambda);
        for (let i = 1; i <= 2 * this.n; i++) {
            this.weightsM[i] = 1 / (2 * (this.n + this.lambda));
            this.weightsC[i] = 1 / (2 * (this.n + this.lambda));
        }
    }
    _generateSigmaPoints() {
        const sqrtP = Math.sqrt(this.P);
        return [this.x, this.x + this.gamma * sqrtP, this.x - this.gamma * sqrtP];
    }
    f(x, u, dt) { return x + u * dt; } // Transition d'état (Dead Reckoning + Accélération)
    h(x) { return x; } // Fonction de mesure
    update(u, z, dt) {
        // Mise à jour dynamique de R (bruit de mesure)
        this.R = Math.max(UKF_R_MIN, Math.min(UKF_R_MAX, z.R_dyn));
        
        // Prédiction (Propagation des points Sigma)
        let X_sigma = this._generateSigmaPoints();
        let X_star = X_sigma.map(x_point => this.f(x_point, u, dt));
        let x_bar = X_star.reduce((sum, x_point, i) => sum + this.weightsM[i] * x_point, 0);
        let P_bar = X_star.reduce((sum, x_point, i) => sum + this.weightsC[i] * (x_point - x_bar) ** 2, 0) + this.Q * dt; 
        
        // Mise à jour (Correction)
        let sigmaP_sqrt = Math.sqrt(P_bar);
        let X_bar_sigma = [x_bar, x_bar + this.gamma * sigmaP_sqrt, x_bar - this.gamma * sigmaP_sqrt];
        let Y_star = X_bar_sigma.map(x_point => this.h(x_point));
        let y_bar = Y_star.reduce((sum, y_point, i) => sum + this.weightsM[i] * y_point, 0);
        let Pyy = Y_star.reduce((sum, y_point, i) => sum + this.weightsC[i] * (y_point - y_bar) ** 2, 0) + this.R; 
        let Pxy = X_bar_sigma.reduce((sum, x_point, i) => sum + this.weightsC[i] * (x_point - x_bar) * (Y_star[i] - y_bar), 0);
        let K = Pxy / Pyy;
        
        this.x = x_bar + K * (z.spd - y_bar); 
        this.P = P_bar - K * Pyy * K;
        
        // Zero-Velocity Update (ZUPT)
        if (z.spd < ZUPT_RAW_THRESHOLD && z.R_dyn < UKF_R_MAX) { this.x = 0; }
        if (this.P < 0) this.P = UKF_R_MIN;

        return { kSpd: this.x, kUncert: this.P };
    }
}

// --- FONCTIONS PHYSIQUES ET DE FILTRAGE (COMMUNES) ---
function dist2D(lat1, lon1, lat2, lon2, R_ref) {
    const R = R_ref || WGS84_A; 
    const dLat = (lat2 - lat1) * D2R, dLon = (lon2 - lon1) * D2R;
    lat1 *= D2R; lat2 *= D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R * c;
}

function kFilterAltitude(kAlt_in, kAltUncert_in, nAlt, acc, dt, baroAlt = null) {
    let R_alt = Math.max(R_ALT_MIN, acc * acc); 
    let alt_meas = nAlt;
    
    // Fusion de la mesure Baro (si disponible)
    if (baroAlt !== null && nAlt !== null) {
        const R_baro = 1.0; 
        if (R_alt > R_baro) {
            alt_meas = (nAlt * R_baro + baroAlt * R_alt) / (R_baro + R_alt);
            R_alt = (R_alt * R_baro) / (R_alt + R_baro); 
        }
    }
    
    if (alt_meas === null) return { kAlt: kAlt_in, kAltUncert: kAltUncert_in };

    const Q_alt = Q_ALT_NOISE * dt; 
    let pAlt = kAlt_in === null ? alt_meas : kAlt_in; 
    let pAltUncert = kAlt_in === null ? 1000 : kAltUncert_in + Q_alt;
    
    let K_alt = pAltUncert / (pAltUncert + R_alt);
    let kAlt = pAlt + K_alt * (alt_meas - pAlt);
    let kAltUncert = (1 - K_alt) * pAltUncert;
    
    return { kAlt, kAltUncert };
}

function updateCelestialBody(bodyKey, kAlt, r_rot, omega_rot) {
    let G_ACC_local = WGS84_G_EQUATOR;
    let R_ALT_CENTER_REF_local = WGS84_A;

    if (bodyKey === 'ROTATING') {
        // Gravité effective est l'accélération centripète (ou 0 si dans un environnement non gravitationnel)
        const centripetal_accel = r_rot * omega_rot ** 2;
        G_ACC_local = centripetal_accel;
        R_ALT_CENTER_REF_local = WGS84_A; // On garde Terre pour la carte
    } else if (bodyKey === 'EARTH') {
         G_ACC_local = WGS84_G_EQUATOR; 
         R_ALT_CENTER_REF_local = WGS84_A;
    } else {
        const data = CELESTIAL_DATA[bodyKey];
        if (data) {
            G_ACC_local = data.G;
            R_ALT_CENTER_REF_local = data.R;
        }
    }
    
    return { G_ACC: G_ACC_local, R_ALT_CENTER_REF: R_ALT_CENTER_REF_local };
}

function getKalmanR(acc, alt, P_hPa, selectedEnv) {
    let acc_effective = acc;
    if (acc > 200) { return UKF_R_MAX; } // Dégradation sévère
    
    let R = acc_effective * acc_effective; 
    const envFactor = ENVIRONMENT_FACTORS[selectedEnv]?.R_MULT || 1.0;
    R *= envFactor;
    
    // Influence de la pression atmosphérique (proxy d'interférence)
    if (P_hPa !== null) {
        const pressureFactor = 1.0 + (1013.25 - P_hPa) / 1013.25 * 0.1;
        R *= Math.max(1.0, pressureFactor); 
    }
    
    // Pénalité souterraine
    if (alt !== null && alt < ALT_TH) { 
        R *= 2.0;
    } 

    return Math.max(UKF_R_MIN, Math.min(UKF_R_MAX, R)); 
            }
// =================================================================
// BLOC 2/3 : Logique Géophysique & Astro & Services (Météo/NTP)
// =================================================================

// --- CLÉS D'API & PROXY VERCEL ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

// --- CONSTANTES DE TEMPS & CALENDRIER ---
const MC_DAY_MS = 72 * 60 * 1000; 
const J1970 = 2440588, J2000 = 2451545.0; // Jours juliens
const dayMs = 1000 * 60 * 60 * 24;

// --- FONCTIONS GÉOPHYSIQUES AVANCÉES ---
function getWGS84Gravity(latDeg, altM) {
    if (latDeg === null || isNaN(latDeg)) return 9.80665;
    const lat = latDeg * D2R; 
    const sin2lat = Math.sin(lat) ** 2;
    // Formule internationale de la gravité (1980) à la surface
    const g_surface = WGS84_G_EQUATOR * (1 + WGS84_BETA * sin2lat) / Math.sqrt(1 - WGS84_E2 * sin2lat);
    // Correction d'altitude (approximation)
    const g_local = g_surface * (1 - 2 * altM / WGS84_A);
    return g_local;
}

function getTrueAirspeed(filteredGroundSpeedMS, airDensity) {
    if (!airDensity || airDensity <= 0) return filteredGroundSpeedMS;
    // La True Airspeed (TAS) corrige la vitesse sol (GS) pour la densité de l'air
    const TAS_Correction_Factor = Math.sqrt(RHO_SEA_LEVEL / airDensity);
    return filteredGroundSpeedMS * TAS_Correction_Factor;
}

function getBarometricAltitude(P_hPa, P_ref_hPa = BARO_ALT_REF_HPA, T_K = 288.15) {
    if (P_hPa === null || T_K === null) return null;
    const P = P_hPa * 100; const P_ref = P_ref_hPa * 100; 
    const L = 0.0065; // Taux de gradient de température standard (K/m)
    const T0 = 288.15; // Température standard au niveau de la mer (K)
    const g = G_ACC; 
    // Formule hypsométrique (atmosphére standard)
    const alt = (T0 / L) * (1 - (P / P_ref) ** (R_AIR * L / g));
    return alt;
}

function getSpeedOfSound(T_K) {
    // Vitesse du son dans l'air sec (formule simplifiée)
    return 20.05 * Math.sqrt(T_K || 273.15);
}

function calculateMRF(alt, netherMode) {
    const NETHER_RATIO = 8.0; 
    if (netherMode) { return 1.0 / NETHER_RATIO; }
    if (alt !== null && alt < ALT_TH) { return 0.5; } // Réduction du facteur si sous-sol
    return 1.0;
}

// --- FONCTIONS DE TEMPS (NTP) ---
async function syncH(lServH_in, lLocH_in) {
    let lServH = lServH_in;
    let lLocH = lLocH_in;
    const $ = id => document.getElementById(id);

    try {
        const response = await fetch(SERVER_TIME_ENDPOINT, { cache: "no-store", mode: "cors" });
        const serverData = await response.json(); 
        const serverTimestamp = Date.parse(serverData.utc_datetime); 

        lServH = serverTimestamp; 
        lLocH = performance.now(); 

    } catch (error) {
        lServH = Date.now(); 
        lLocH = performance.now();
        if ($('local-time')) $('local-time').textContent = 'N/A (SYNCHRO ÉCHOUÉE)';
    }
    return { lServH, lLocH };
}

function getCDate(lServH, lLocH) { 
    if (lServH === null || lLocH === null) { return null; }
    const offsetSinceSync = performance.now() - lLocH;
    return new Date(lServH + offsetSinceSync); 
}

// --- FONCTION MÉTÉO (Proxy OpenWeatherMap) ---
async function fetchWeather(latA, lonA) {
    if (!latA || !lonA) return null; 
    const apiUrl = `${PROXY_WEATHER_ENDPOINT}?lat=${latA}&lon=${lonA}`;
    let weatherData = null;
    
    try {
        const response = await fetch(apiUrl);
        if (!response.ok) throw new Error(`Erreur HTTP: ${response.status}`);
        const data = await response.json();
        
        if (data.main) {
            const tempC = data.main.temp;
            const pressure_hPa = data.main.pressure;
            const humidity_perc = data.main.humidity;
            const tempK = tempC + 273.15;
            const pressure_pa = pressure_hPa * 100;
            
            // Calcul de la Densité de l'Air (Loi des gaz parfaits)
            const air_density = pressure_pa / (R_AIR * tempK); 
            
            // Calcul du Point de Rosée (Formule Magnus)
            const a = 17.27, b = 237.7;
            const h_frac = humidity_perc / 100.0;
            const f = (a * tempC) / (b + tempC) + Math.log(h_frac);
            const dew_point = (b * f) / (a - f);
            
            weatherData = {
                tempC: tempC,
                pressure_hPa: pressure_hPa,
                humidity_perc: humidity_perc,
                tempK: tempK,
                air_density: air_density,
                dew_point: dew_point
            };
        } else {
             throw new Error(data.message || 'Données météo incomplètes');
        }
    } catch (err) {
        // console.warn("Erreur de récupération météo:", err.message);
    }
    return weatherData; 
}


// --- FONCTIONS ASTRO (SUNCALC) ---
function toDays(date) { return (date.valueOf() / dayMs - 0.5 + J1970) - J2000; }
function solarMeanAnomaly(d) { return D2R * (356.0470 + 0.9856002585 * d); }
function eclipticLongitude(M) {
    var C = D2R * (1.9148 * Math.sin(M) + 0.0200 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M)), 
        P = D2R * 102.9377;                                                                
    return M + C + P + Math.PI;
}
function getSolarTime(date, lon) {
    if (date === null || lon === null || isNaN(lon)) return { TST: 'N/A', MST: 'N/A', EOT: 'N/D', ECL_LONG: 'N/D' };
    
    const d = toDays(date);
    const M = solarMeanAnomaly(d); 
    const L = eclipticLongitude(M); 
    
    // Calcul de l'équation du temps (EOT) et du transit solaire (midi)
    const J_star = toDays(date) - lon / 360;
    const J_transit = J_star + (0.0053 * Math.sin(M) - 0.0069 * Math.sin(2 * L));
    const eot_min = (J_star - J_transit) * 1440; 

    // Heure UTC en millisecondes depuis minuit
    const msSinceMidnightUTC = (date.getUTCHours() * 3600 + date.getUTCMinutes() * 60 + date.getUTCSeconds()) * 1000 + date.getUTCMilliseconds();
    
    // Mean Solar Time (MST)
    const mst_offset_ms = lon * dayMs / 360; 
    const mst_ms = (msSinceMidnightUTC + mst_offset_ms + dayMs) % dayMs;
    
    // True Solar Time (TST)
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
        NoonSolar: toTimeString(J_transit * dayMs)
    };
}

function getMinecraftTime(date) {
    if (date === null) return '00:00:00';
    const msSinceMidnightUTC = date.getUTCHours() * 3600000 + date.getUTCMilliseconds() + date.getUTCMinutes() * 60000 + date.getUTCSeconds() * 1000;
    const timeRatio = (msSinceMidnightUTC % dayMs) / dayMs;
    const MC_DAY_MS = 72 * 60 * 1000; 
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

function updateClockVisualization(now, sunPos, moonPos, sunTimes) {
    const $ = id => document.getElementById(id);
    const sunEl = $('sun-element');
    const moonEl = $('moon-element');
    const clockEl = $('minecraft-clock'); 

    if (!sunEl || !moonEl || !clockEl) return;

    const sunIcon = sunEl.querySelector('.sun-icon');
    const moonIcon = moonEl.querySelector('.moon-icon');

    // Mettre à jour la position du Soleil
    if (sunPos) {
        const altDeg = sunPos.altitude * R2D;
        const aziDeg = (sunPos.azimuth * R2D + 180) % 360; 
        sunEl.style.transform = `rotate(${aziDeg}deg)`;
        const radialPercent = Math.min(50, Math.max(0, 50 * (90 - altDeg) / 90));
        const altitudeOffsetPercent = 50 - radialPercent; 
        sunIcon.style.transform = `translateY(calc(-50% + ${altitudeOffsetPercent}%) )`; 
        sunEl.style.display = altDeg > -0.83 ? 'flex' : 'none'; 
    } else {
        sunEl.style.display = 'none';
    }

    // Mettre à jour la position de la Lune
    if (moonPos) {
        const altDeg = moonPos.altitude * R2D;
        const aziDeg = (moonPos.azimuth * R2D + 180) % 360; 
        moonEl.style.transform = `rotate(${aziDeg}deg)`;
        const radialPercent = Math.min(50, Math.max(0, 50 * (90 - altDeg) / 90));
        const altitudeOffsetPercent = 50 - radialPercent; 
        moonIcon.style.transform = `translateY(calc(-50% + ${altitudeOffsetPercent}%) )`;
        moonEl.style.display = altDeg > 0 ? 'flex' : 'none';
    } else {
        moonEl.style.display = 'none';
    }
    
    // Mettre à jour le fond et le mode jour/nuit
    const body = document.body;
    body.classList.remove('sky-day', 'sky-sunset', 'sky-night', 'light-mode', 'dark-mode');
    clockEl.classList.remove('sky-day', 'sky-sunset', 'sky-night');

    if (sunTimes) {
        const nowMs = now.getTime();
        let bodyClass;
        if (nowMs >= sunTimes.sunriseEnd.getTime() && nowMs < sunTimes.sunsetStart.getTime()) {
            bodyClass = 'sky-day';
        } else if (nowMs >= sunTimes.dusk.getTime() || nowMs < sunTimes.dawn.getTime()) {
            bodyClass = 'sky-night';
        } else {
            bodyClass = 'sky-sunset';
        }
        
        body.classList.add(bodyClass);
        body.classList.add(bodyClass === 'sky-day' ? 'light-mode' : 'dark-mode');
        clockEl.classList.add(bodyClass); 

        $('clock-status').textContent = sunPos && sunPos.altitude > 0 ? 'Jour Solaire (☀️)' : 'Nuit/Crépuscule (🌙)';
    } else {
        $('clock-status').textContent = 'Position Solaire Indisponible';
    }
}

function updateAstro(latA, lonA, lServH, lLocH) {
    const $ = id => document.getElementById(id);
    const now = getCDate(lServH, lLocH); 
    
    if (now === null) return;
    
    if ($('time-minecraft')) $('time-minecraft').textContent = getMinecraftTime(now);

    if (typeof SunCalc === 'undefined' || !latA || !lonA) {
        $('clock-status').textContent = 'Astro (Attente GPS)...';
        return;
    }
    
    if ($('date-display-astro')) $('date-display-astro').textContent = now.toLocaleDateString('fr-FR');

    const sunPos = SunCalc.getPosition(now, latA, lonA);
    const moonIllum = SunCalc.getMoonIllumination(now);
    const moonPos = SunCalc.getMoonPosition(now, latA, lonA);
    const sunTimes = SunCalc.getTimes(now, latA, lonA);
    const solarTimes = getSolarTime(now, lonA);
    
    // Mise à jour des données Astro
    if ($('tst')) $('tst').textContent = solarTimes.TST;
    if ($('lsm')) $('lsm').textContent = solarTimes.MST;
    if ($('eot')) $('eot').textContent = solarTimes.EOT + ' min'; 
    if ($('ecliptic-long')) $('ecliptic-long').textContent = solarTimes.ECL_LONG + ' °';
    if ($('sun-alt')) $('sun-alt').textContent = `${(sunPos.altitude * R2D).toFixed(2)} °`;
    if ($('moon-illuminated')) $('moon-illuminated').textContent = `${(moonIllum.fraction * 100).toFixed(1)} %`;
    if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(moonIllum.phase);
    
    if ($('day-duration') && sunTimes.sunrise && sunTimes.sunset) {
        const durationMs = sunTimes.sunset.getTime() - sunTimes.sunrise.getTime();
        const hours = Math.floor(durationMs / 3600000);
        const minutes = Math.floor((durationMs % 3600000) / 60000);
        $('day-duration').textContent = `${hours}h ${minutes}m`;
    } 

    updateClockVisualization(now, sunPos, moonPos, sunTimes);
    }
// =================================================================
// BLOC 3/3 : Logique Applicative Principale (updateDisp & DOM/Init)
// =================================================================

// --- CONSTANTES DE CONFIGURATION SYSTÈME ---
const MIN_DT = 0.01; 
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};
const MAP_UPDATE_INTERVAL = 3000; 

// --- VARIABLES D'ÉTAT (Globales) ---
let wID = null, domID = null, lPos = null, lat = null, lon = null, sTime = null;
let distM = 0, maxSpd = 0;
let kSpd = 0, kUncert = 1000; 
let timeMoving = 0; 
let lServH = null, lLocH = null; 
let lastFSpeed = 0; 
let kAlt = null;      
let kAltUncert = 10;  
let ukfSpeed = null; 

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

// Données externes
let lastP_hPa = BARO_ALT_REF_HPA, lastT_K = 288.15, currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound = 343;

// IMU/Quaternion State
let real_accel_x = 0, real_accel_y = 0, real_accel_z = 0;
let currentAttitude = new Quaternion();
let lastAccelMagnitude = 0;

// Objets Map (Leaflet)
let map, marker, circle;
let lastMapUpdate = 0;

// --- FONCTION UTILITAIRE DOM ---
const $ = id => document.getElementById(id);


// --- GESTION DES CAPTEURS IMU (QUATERNION) ---
function imuMotionHandler(event) {
    let accX = 0, accY = 0, accZ = 0;

    if (event.accelerationIncludingGravity) {
        accX = event.accelerationIncludingGravity.x || 0;
        accY = event.accelerationIncludingGravity.y || 0;
        accZ = event.accelerationIncludingGravity.z || 0;
        if ($('imu-status')) $('imu-status').textContent = "Actif (Gravité incluse)";
    } 
    else if (event.acceleration) {
         accX = event.acceleration.x || 0;
         accY = event.acceleration.y || 0;
         accZ = event.acceleration.z || 0;
         if ($('imu-status')) $('imu-status').textContent = "Actif (Sans Gravité)";
    } else {
        if ($('imu-status')) $('imu-status').textContent = "Erreur (Capteur N/A)";
        return;
    }
    
    // Convertir en m/s² si les données sont en g
    if (Math.abs(accZ) > 30) { 
        accX /= 9.81; accY /= 9.81; accZ /= 9.81; 
    }

    real_accel_x = accX;
    real_accel_y = accY;
    real_accel_z = accZ;

    lastAccelMagnitude = Math.sqrt(accX**2 + accY**2 + accZ**2);
    
    // Le filtre UKF utilise real_accel_z (l'accélération verticale, direction du mouvement pour un corps en déplacement) comme input.
    // L'attitude est calculée à partir de l'accélération, incluant la gravité.
    currentAttitude = Quaternion.fromAcc(accX, accY, accZ);
}

function startIMUListeners() {
    const requestPermission = (EventClass, handler) => {
        if (typeof EventClass.requestPermission === 'function') {
            EventClass.requestPermission().then(permissionState => {
                if (permissionState === 'granted') {
                    window.addEventListener(EventClass.name.toLowerCase(), handler);
                }
            });
        } else {
            window.addEventListener(EventClass.name.toLowerCase(), handler);
        }
    };
    if (window.DeviceMotionEvent) requestPermission(DeviceMotionEvent, imuMotionHandler);
    if ($('imu-status')) $('imu-status').textContent = "Capteurs en attente...";
}

function stopIMUListeners() {
    if (window.DeviceMotionEvent) window.removeEventListener('devicemotion', imuMotionHandler);
    if ($('imu-status')) $('imu-status').textContent = "Inactif";
    real_accel_x = real_accel_y = real_accel_z = 0;
}


// --- Fonctions Carte ---
function initMap() {
    try {
        if ($('map') && typeof L !== 'undefined' && !map) { 
            map = L.map('map').setView([0, 0], 2);
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                attribution: '© OpenStreetMap contributors'
            }).addTo(map);
            marker = L.marker([0, 0]).addTo(map);
            circle = L.circle([0, 0], { color: 'red', fillColor: '#f03', fillOpacity: 0.5, radius: 10 }).addTo(map);
        }
    } catch (e) {
        if ($('map')) $('map').innerHTML = "Erreur d'initialisation de la carte (Leaflet non chargé).";
    }
}

function updateMap(lat, lon, acc) {
    if (map && marker) {
        marker.setLatLng([lat, lon]);
        // Utiliser le rayon ajusté par le facteur MRF (Nether/Souterrain)
        circle.setLatLng([lat, lon]).setRadius(acc * R_FACTOR_RATIO); 
        const now = Date.now();
        if (now - lastMapUpdate > MAP_UPDATE_INTERVAL && kSpd > MIN_SPD) {
            map.setView([lat, lon], map.getZoom() > 10 ? map.getZoom() : 16); 
            lastMapUpdate = now;
        } else if (map.getZoom() < 10 && (Date.now() - lastMapUpdate > 5000)) {
            map.setView([lat, lon], 12);
            lastMapUpdate = now;
        }
    }
}

// --- FONCTIONS DE CONTRÔLE GPS & Erreur ---
function setGPSMode(mode) {
    currentGPSMode = mode;
    if (wID !== null) {
        stopGPS(false); 
        startGPS();     
    }
    if ($('freq-select')) $('freq-select').value = mode; 
}

function startGPS() {
    if (wID !== null) return; 
    
    const options = (currentGPSMode === 'HIGH_FREQ') ? GPS_OPTS.HIGH_FREQ : GPS_OPTS.LOW_FREQ;
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
    
    if (resetButton) {
        if ($('toggle-gps-btn')) {
            $('toggle-gps-btn').textContent = '▶️ MARCHE GPS';
            $('toggle-gps-btn').style.backgroundColor = '#28a745'; 
        }
    }
}

function emergencyStop() {
    emergencyStopActive = true;
    stopGPS(false);
    if ($('emergency-stop-btn')) {
        $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: ACTIF 🔴";
        $('emergency-stop-btn').classList.add('active');
    }
}

function resumeSystem() {
    emergencyStopActive = false;
    if ($('emergency-stop-btn')) {
        $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: INACTIF 🟢";
        $('emergency-stop-btn').classList.remove('active');
    }
    startGPS();
}

function handleErr(err) {
    if ($('gps-precision')) $('gps-precision').textContent = `Erreur: ${err.message}`;
    if (err.code === 1) { 
        stopGPS();
        alert("Accès à la géolocalisation refusé. Veuillez l'activer.");
    }
}


// ===========================================
// FONCTION PRINCIPALE DE MISE À JOUR GPS (updateDisp)
// ===========================================

function updateDisp(pos) {
    // Vérifie l'état d'arrêt d'urgence
    if (emergencyStopActive) return;

    // --- 1. ACQUISITION DES DONNÉES & INITIALISATION ---
    const cTimePos = pos.timestamp;
    let cLat = pos.coords.latitude;
    let cLon = pos.coords.longitude;
    let altRaw = pos.coords.altitude;
    let accRaw = pos.coords.accuracy;

    if (gpsAccuracyOverride > 0.0) { accRaw = gpsAccuracyOverride; }

    if (ukfSpeed === null) {
        // Initialise l'UKF à la première position
        ukfSpeed = new UKF([0], [UKF_R_MAX], UKF_Q_SPD, UKF_R_MAX, KAPPA);
    }

    if (lPos === null) {
        // Sauvegarde la première position
        lPos = pos; kAlt = altRaw; lat = cLat; lon = cLon;
        sTime = Date.now();
        updateMap(cLat, cLon, accRaw);
        return; 
    }
    
    const dt = (cTimePos - lPos.timestamp) / 1000;
    if (dt < MIN_DT || dt > 10) { lPos = pos; return; } // Ignore les mises à jour trop rapides ou trop lentes
    
    // --- 2. GESTION DU SIGNAL & BRUIT (R) ---
    // Calcul de R_dyn basé sur la précision GPS et l'environnement
    let R_dyn = getKalmanR(accRaw, kAlt, lastP_hPa, selectedEnvironment); 
    let isSignalPoor = (accRaw > 200 || R_dyn >= UKF_R_MAX * 0.75);
    let modeStatus = '';
    
    if (isSignalPoor) { 
        // Si le signal est trop faible, on fait du dead reckoning (estimation seule)
        modeStatus = `⚠️ ESTIMATION SEULE (Signal Faible/DR)`;
        cLat = lat; 
        cLon = lon;
        altRaw = kAlt; 
        if ($('gps-precision')) $('gps-precision').textContent = `❌ ${accRaw.toFixed(0)} m (Signal Faible/Estimation)`; 
    } else {
        modeStatus = `🚀 UKF WGS84 FUSION TOTALE`;
        lat = cLat; 
        lon = cLon;
        if ($('gps-precision')) $('gps-precision').textContent = `${accRaw.toFixed(2)} m`; 
    }
    
    // --- 3. FILTRAGE UKF ALTITUDE (Fusion Baro/GPS/EKF-like) ---
    const baroAlt = getBarometricAltitude(lastP_hPa, BARO_ALT_REF_HPA, lastT_K);
    const { kAlt: kAlt_new, kAltUncert: kAltUncert_new } = kFilterAltitude(
        kAlt, kAltUncert, altRaw, pos.coords.altitudeAccuracy || R_ALT_MIN, dt, baroAlt
    );
    kAlt = kAlt_new;
    kAltUncert = kAltUncert_new;
    
    // --- 4. CALCUL VITESSE BRUTE 3D ---
    const dist2D_val = dist2D(lPos.coords.latitude, lPos.coords.longitude, cLat, cLon, R_ALT_CENTER_REF);
    const altDiff = (kAlt_new || 0) - (lPos.kAlt_old || 0);
    const dist3D = Math.sqrt(dist2D_val ** 2 + altDiff ** 2);
    let spd3D_raw = dist3D / dt; 
    const spdV = altDiff / dt; 

    // --- 5. LOGIQUE UKF VITESSE & ZUPT ---
    // Utilise l'accélération verticale (Z) comme input cinématique (propulsion/freinage vertical simulé)
    const accel_sensor_input = real_accel_z; 

    const ukf_measurement = { spd: spd3D_raw, R_dyn: R_dyn };
    const { kSpd: fSpd, kUncert: kUncert_new } = ukfSpeed.update(accel_sensor_input, ukf_measurement, dt);
    kSpd = fSpd;
    kUncert = kUncert_new;
    
    const sSpdFE = fSpd < MIN_SPD ? 0 : fSpd; 
    if (sSpdFE === 0 && R_dyn < UKF_R_MAX) { modeStatus = '✅ ZUPT (Vélocité Nulle Forcée)'; } 

    // --- 6. CALCULS AVANCÉS (Gravité, TAS, Distance) ---
    let accel_long = 0;
    if (dt > 0.05) { accel_long = (sSpdFE - lastFSpeed) / dt; }
    lastFSpeed = sSpdFE;

    R_FACTOR_RATIO = calculateMRF(kAlt_new, netherMode); 
    distM += sSpdFE * dt * R_FACTOR_RATIO; 
    
    if (sSpdFE > MIN_SPD) { timeMoving += dt; }
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    
    // Mise à jour de la gravité locale (dépend du corps céleste sélectionné)
    let local_g = G_ACC;
    if (currentCelestialBody === 'EARTH') {
        local_g = getWGS84Gravity(cLat, kAlt_new);
        G_ACC = local_g;
    } else if (currentCelestialBody === 'ROTATING') {
        local_g = rotationRadius * angularVelocity ** 2;
        G_ACC = local_g;
    }
    
    const tas_ms = getTrueAirspeed(sSpdFE, currentAirDensity);

    // --- 7. MISE À JOUR DU DOM (Affichage UKF/WGS84/Quaternion) ---
    
    // Vitesse & Distance
    if ($('speed-stable')) $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(5)}`;
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${sSpdFE.toFixed(3)} m/s`;
    if ($('speed-max')) $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    if ($('speed-3d-inst')) $('speed-3d-inst').textContent = `${(spd3D_raw * KMH_MS).toFixed(5)} km/h`; 
    if ($('perc-speed-sound')) $('perc-speed-sound').textContent = `${(sSpdFE / currentSpeedOfSound * 100).toFixed(2)} %`;
    if ($('distance-total-km')) $('distance-total-km').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    if ($('tas-display')) $('tas-display').textContent = `${tas_ms.toFixed(3)} m/s`;

    // GPS & Physique
    if ($('latitude')) $('latitude').textContent = `${lat.toFixed(6)} °`;
    if ($('longitude')) $('longitude').textContent = `${lon.toFixed(6)} °`;
    if ($('altitude-gps')) $('altitude-gps').textContent = kAlt_new !== null ? `${kAlt_new.toFixed(2)} m (UKF/Baro Fusion)` : 'N/A';
    if ($('underground-status')) {
        $('underground-status').textContent = `Souterrain: ${kAlt_new !== null && kAlt_new < ALT_TH ? 'OUI' : 'Non'} (${modeStatus} | Acc: ${accRaw.toFixed(1)}m | R: ${R_dyn.toExponential(1)})`;
    }
    if ($('gravity-local')) $('gravity-local').textContent = `${local_g.toFixed(5)} m/s²`;
    if ($('accel-long')) $('accel-long').textContent = `${accel_long.toFixed(3)} m/s²`;
    if ($('force-g-long')) $('force-g-long').textContent = local_g > 0.1 ? `${(accel_long / local_g).toFixed(2)} G` : '0.00 G';
    if ($('vertical-speed')) $('vertical-speed').textContent = `${spdV.toFixed(2)} m/s`;
    
    // IMU (QUATERNION)
    const euler = currentAttitude.toEuler();
    if ($('imu-accel-x')) $('imu-accel-x').textContent = `${real_accel_x.toFixed(2)} m/s²`;
    if ($('imu-accel-y')) $('imu-accel-y').textContent = `${real_accel_y.toFixed(2)} m/s²`;
    if ($('imu-accel-z')) $('imu-accel-z').textContent = `${real_accel_z.toFixed(2)} m/s²`;
    if ($('attitude-roll')) $('attitude-roll').textContent = `${euler.roll.toFixed(2)} °`;
    if ($('attitude-pitch')) $('attitude-pitch').textContent = `${euler.pitch.toFixed(2)} °`;
    if ($('attitude-yaw')) $('attitude-yaw').textContent = `${euler.yaw.toFixed(2)} ° (Approximation)`;


    // Kalman (UKF)
    if ($('kalman-uncert')) $('kalman-uncert').textContent = `${kUncert.toFixed(5)} m²/s² (P)`;
    if ($('ukf-uncert-alt')) $('ukf-uncert-alt').textContent = `${kAltUncert_new.toFixed(3)} m² (P)`; 
    if ($('speed-error-perc')) $('speed-error-perc').textContent = `${R_dyn.toFixed(3)} m² (R dyn)`;
    
    // --- 8. SAUVEGARDE & MISE À JOUR CARTE ---
    lPos = pos; 
    lPos.speedMS_3D = spd3D_raw; 
    lPos.timestamp = cTimePos; 
    lPos.kAlt_old = kAlt_new; 

    updateMap(lat, lon, accRaw);
}


// --- INITIALISATION DOM ET BOUCLE LENTE ---
document.addEventListener('DOMContentLoaded', () => {
    
    initMap(); 

    // --- Initialisation des Contrôles (Mass, Corps Céleste, Environnement, etc.) ---
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
            G_ACC = newVals.G_ACC;
            R_ALT_CENTER_REF = newVals.R_ALT_CENTER_REF;
            currentCelestialBody = e.target.value;
            if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC.toFixed(4)} m/s²`;
        });
    }

    const updateRotation = () => {
        rotationRadius = parseFloat($('rotation-radius')?.value) || 100;
        angularVelocity = parseFloat($('angular-velocity')?.value) || 0;
        if (currentCelestialBody === 'ROTATING') {
            const newVals = updateCelestialBody('ROTATING', kAlt, rotationRadius, angularVelocity);
            G_ACC = newVals.G_ACC;
            if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC.toFixed(4)} m/s²`;
        }
    };
    if ($('rotation-radius')) $('rotation-radius').addEventListener('input', updateRotation);
    if ($('angular-velocity')) $('angular-velocity').addEventListener('input', updateRotation);

    if ($('environment-select')) {
        $('environment-select').addEventListener('change', (e) => { 
            if (emergencyStopActive) return;
            selectedEnvironment = e.target.value; 
            if ($('env-factor')) $('env-factor').textContent = `${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY} (x${ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT.toFixed(1)})`; 
        });
        if ($('env-factor')) $('env-factor').textContent = `${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY} (x${ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT.toFixed(1)})`;
    }
    
    if ($('gps-accuracy-override')) {
        $('gps-accuracy-override').addEventListener('change', (e) => {
            gpsAccuracyOverride = parseFloat(e.target.value) || 0.0;
        });
    }

    // --- Boutons Principaux ---
    if ($('toggle-gps-btn')) {
        $('toggle-gps-btn').addEventListener('click', () => { 
            if (emergencyStopActive) {
                alert("Veuillez désactiver l'Arrêt d'urgence avant d'utiliser ce contrôle.");
                return;
            }
            wID === null ? startGPS() : stopGPS(); 
        });
    }
    if ($('freq-select')) $('freq-select').addEventListener('change', (e) => setGPSMode(e.target.value));

    if ($('emergency-stop-btn')) {
        $('emergency-stop-btn').addEventListener('click', () => { 
            if (!emergencyStopActive) { emergencyStop(); } else { resumeSystem(); }
        });
    }
    
    if ($('nether-toggle-btn')) {
        $('nether-toggle-btn').addEventListener('click', () => { 
            if (emergencyStopActive) return; 
            netherMode = !netherMode; 
            if ($('mode-nether')) $('mode-nether').textContent = netherMode ? `ACTIVÉ (1:8) 🔥` : "DÉSACTIVÉ (1:1)"; 
        });
    }
    
    // Réinitialisation
    if ($('reset-dist-btn')) {
        $('reset-dist-btn').addEventListener('click', () => { 
            if (emergencyStopActive) return; 
            distM = 0; timeMoving = 0; 
        });
    }
    
    if ($('reset-max-btn')) {
        $('reset-max-btn').addEventListener('click', () => { 
            if (emergencyStopActive) return; 
            maxSpd = 0; 
        });
    }
    
    if ($('reset-all-btn')) {
        $('reset-all-btn').addEventListener('click', () => { 
            if (emergencyStopActive) return; 
            if (confirm("Réinitialiser toutes les données de session ?")) { 
                distM = 0; maxSpd = 0; 
                kSpd = 0; kUncert = 1000; 
                timeMoving = 0; 
                kAlt = null; kAltUncert = 10;
                lPos = null; sTime = null;
                ukfSpeed = null; 
                if ($('distance-total-km')) $('distance-total-km').textContent = `0.000 km | 0.00 m`; 
                if ($('speed-max')) $('speed-max').textContent = `0.00000 km/h`; 
                alert("Système réinitialisé. Redémarrez le GPS pour recommencer.");
            } 
        });
    }
    
    if ($('toggle-mode-btn')) {
        $('toggle-mode-btn').addEventListener('click', () => {
            document.body.classList.toggle('dark-mode');
            document.body.classList.toggle('light-mode');
        });
    }
    
    // --- DÉMARRAGE DU SYSTÈME ---
    const initVals = updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity);
    G_ACC = initVals.G_ACC;
    R_ALT_CENTER_REF = initVals.R_ALT_CENTER_REF;
    
    syncH(lServH, lLocH).then(newTimes => {
        lServH = newTimes.lServH;
        lLocH = newTimes.lLocH;
        startGPS(); 
    });

    // Boucle de mise à jour lente (Astro/Météo/Horloge)
    const DOM_SLOW_UPDATE_MS = 1000;
    if (domID === null) {
        domID = setInterval(() => {
            const currentLat = lat || 43.296; 
            const currentLon = lon || 5.370;
            
            if (typeof updateAstro === 'function') {
                updateAstro(currentLat, currentLon, lServH, lLocH);
            }
            
            // Resynchronisation NTP toutes les 60 secondes
            if (Math.floor(Date.now() / 1000) % 60 === 0) {
                 syncH(lServH, lLocH).then(newTimes => {
                    lServH = newTimes.lServH;
                    lLocH = newTimes.lLocH;
                 });
     
