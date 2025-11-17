// =================================================================
// BLOC 1/4 : Modèles Avancés (UKF, Quaternion, WGS84) & Filtres
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
// BLOC 2/4 : Logique Géophysique & Astro & Services (Météo/NTP)
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
    if ($('local-time')) $('local-time').textContent = 'Synchronisation...';
    const localStartPerformance = performance.now(); 

    try {
        const response = await fetch(SERVER_TIME_ENDPOINT, { cache: "no-store", mode: "cors" });
        if (!response.ok) throw new Error(`Server time sync failed: ${response.statusText}`);
        
        const localEndPerformance = performance.now(); 
        const serverData = await response.json(); 
        
        const utcTimeISO = serverData.utc_datetime; 
        const serverTimestamp = Date.parse(utcTimeISO); 
        
        const RTT = localEndPerformance - localStartPerformance;
        const latencyOffset = RTT / 2;

        lServH = serverTimestamp + latencyOffset; 
        lLocH = performance.now(); 
        // console.log(`Synchro UTC réussie. Latence: ${latencyOffset.toFixed(1)} ms.`);
        
        const now = getCDate(lServH, lLocH);
        if (now) {
            if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString('fr-FR');
            if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
        }

    } catch (error) {
        // console.warn("Échec de la synchronisation. Utilisation de l'horloge locale.", error);
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
    const moonTimes = SunCalc.getMoonTimes(now, latA, lonA, true);
    const solarTimes = getSolarTime(now, lonA);
    
    // Mise à jour des données Astro
    if ($('tst')) $('tst').textContent = solarTimes.TST;
    if ($('lsm')) $('lsm').textContent = solarTimes.MST;
    if ($('eot')) $('eot').textContent = solarTimes.EOT + ' min'; 
    if ($('ecliptic-long')) $('ecliptic-long').textContent = solarTimes.ECL_LONG + ' °';
    if ($('sun-alt')) $('sun-alt').textContent = `${(sunPos.altitude * R2D).toFixed(2)} °`;
    if ($('sun-azimuth')) $('sun-azimuth').textContent = `${(sunPos.azimuth * R2D).toFixed(2)} ° (Azi)`;
    if ($('moon-alt')) $('moon-alt').textContent = `${(moonPos.altitude * R2D).toFixed(2)} °`;
    if ($('moon-azimuth')) $('moon-azimuth').textContent = `${(moonPos.azimuth * R2D).toFixed(2)} ° (Azi)`;
    if ($('moon-illuminated')) $('moon-illuminated').textContent = `${(moonIllum.fraction * 100).toFixed(1)} %`;
    if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(moonIllum.phase);
    
    if ($('day-duration') && sunTimes.sunrise && sunTimes.sunset) {
        const durationMs = sunTimes.sunset.getTime() - sunTimes.sunrise.getTime();
        const hours = Math.floor(durationMs / 3600000);
        const minutes = Math.floor((durationMs % 3600000) / 60000);
        $('day-duration').textContent = `${hours}h ${minutes}m`;
    } 
    
    if ($('moon-times')) $('moon-times').textContent = moonTimes ? 
        `↑ ${moonTimes.rise ? moonTimes.rise.toLocaleTimeString() : 'N/A'} / ↓ ${moonTimes.set ? moonTimes.set.toLocaleTimeString() : 'N/A'}` : 'N/D';

    updateClockVisualization(now, sunPos, moonPos, sunTimes);
    }
// =================================================================
// BLOC 4.1/4 : Modèles & Filtres (UKF, EKF Altitude, Constantes)
// =================================================================

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES GLOBALES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const KMH_MS = 3.6;          // Conversion m/s vers km/h
const G_ACC = 9.80665;       // Accélération standard de la gravité (m/s²)
const R_ALT_CENTER_REF = 6371000; // Rayon terrestre de référence (m)
const BARO_ALT_REF_HPA = 1013.25; // Pression atmosphérique standard au niveau de la mer (hPa)
const RHO_SEA_LEVEL = 1.225; // Masse volumique de l'air au niveau de la mer (kg/m³)

// --- PARAMÈTRES DU FILTRE DE KALMAN NON PARFUMÉ (UKF) VITESSE ---
const UKF_R_MAX = 500.0;     // Bruit de mesure maximum (R) pour la vitesse GPS
const UKF_Q_SPD = 0.5;       // Bruit de processus (Q) pour la vitesse (basé sur une accélération max supposée)
const KAPPA = 0.0;           // Paramètre de mise à l'échelle pour les points Sigma (UKF)
const MIN_SPD = 0.05;        // Vitesse minimale pour être considéré "en mouvement"
const R_ALT_MIN = 1.0;       // Bruit de mesure minimum pour l'altitude (m²)

// --- FACTEURS D'ENVIRONNEMENT (Modélisation du bruit GPS/Magnétique) ---
const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0, DISPLAY: 'Surface Standard' },
    'URBAN': { R_MULT: 1.5, DISPLAY: 'Urbain Denser' },
    'MOUNTAIN': { R_MULT: 1.2, DISPLAY: 'Montagne' },
    'SUBTERRANEAN': { R_MULT: 3.0, DISPLAY: 'Souterrain/Tunnel' }
};

// ===========================================
// CLASSE UKF (Unscented Kalman Filter)
// Note: Implémentation simplifiée pour le modèle de vitesse 1D
// ===========================================

class UKF {
    constructor(initialState, initialCovariance, processNoiseQ, measurementNoiseR, kappa) {
        // État (vitesse [0])
        this.x = initialState;
        // Covariance de l'état (P)
        this.P = initialCovariance;
        // Bruit de processus (Q)
        this.Q = processNoiseQ;
        // Bruit de mesure (R - sera dynamique)
        this.R_base = measurementNoiseR;
        // Paramètre de l'UKF
        this.kappa = kappa;
        // Poids des points Sigma
        this.lambda = kappa; 
        this.W_m = [(this.lambda / (this.x.length + this.lambda)), 0.5 / (this.x.length + this.lambda), 0.5 / (this.x.length + this.lambda)];
        this.W_c = [this.W_m[0], this.W_m[1], this.W_m[2]]; 
    }

    // Génère 2n+1 points sigma (pour 1D : 3 points)
    generateSigmaPoints() {
        const sqrt_P = Math.sqrt(this.P[0]);
        const gamma = Math.sqrt(this.x.length + this.lambda);

        const sigma_points = [
            this.x[0],
            this.x[0] + gamma * sqrt_P,
            this.x[0] - gamma * sqrt_P
        ];
        return sigma_points;
    }

    // Fonction de transition d'état (Modèle cinématique 1D)
    // f(x, u, dt) = x + u * dt (Vitesse = Vitesse + Accélération * dt)
    stateTransition(sigma, u, dt) {
        // u = accélération réelle (input du capteur IMU)
        return sigma + u * dt;
    }

    // Fonction d'observation (Mesure directe de la vitesse)
    // h(x) = x
    observationFunction(sigma) {
        return sigma; 
    }
    
    // Étape de prédiction
    predict(accel_input, dt) {
        const sigma_points = this.generateSigmaPoints();
        const sigma_predicted = sigma_points.map(s => this.stateTransition(s, accel_input, dt));

        // Calcul de la moyenne prédite (x_bar)
        const x_bar = this.W_m[0] * sigma_predicted[0] + 
                      this.W_m[1] * sigma_predicted[1] + 
                      this.W_m[2] * sigma_predicted[2];
                      
        // Calcul de la covariance prédite (P_bar)
        let P_bar = this.Q; // Ajout du bruit de processus Q
        for (let i = 0; i < 3; i++) {
            P_bar += this.W_c[i] * (sigma_predicted[i] - x_bar) ** 2;
        }

        this.x_pred = [x_bar];
        this.P_pred = [P_bar];
        
        return { x: this.x_pred, P: this.P_pred };
    }

    // Étape de mise à jour (Correction)
    update(accel_input, measurement, dt) {
        this.predict(accel_input, dt);

        const sigma_predicted = this.generateSigmaPoints().map(s => this.stateTransition(s, accel_input, dt));

        // Transformer les points sigma prédits à travers la fonction d'observation (h)
        const sigma_measurement = sigma_predicted.map(s => this.observationFunction(s));
        
        // Calcul de la moyenne de mesure prédite (y_bar)
        const y_bar = this.W_m[0] * sigma_measurement[0] + 
                      this.W_m[1] * sigma_measurement[1] + 
                      this.W_m[2] * sigma_measurement[2];

        // Calcul de la covariance de mesure (Pyy)
        let Pyy = measurement.R_dyn; // Bruit de mesure R_dyn
        for (let i = 0; i < 3; i++) {
            Pyy += this.W_c[i] * (sigma_measurement[i] - y_bar) ** 2;
        }

        // Calcul de la covariance croisée (Pxy)
        let Pxy = 0;
        for (let i = 0; i < 3; i++) {
            Pxy += this.W_c[i] * (sigma_predicted[i] - this.x_pred[0]) * (sigma_measurement[i] - y_bar);
        }
        
        // Calcul du Gain de Kalman (K)
        const K = Pxy / Pyy;

        // Correction de l'état (x) et de la covariance (P)
        const y_diff = measurement.spd - y_bar;
        this.x[0] = this.x_pred[0] + K * y_diff;
        this.P[0] = this.P_pred[0] - K * Pyy * K;
        
        // Si la vitesse est très faible et l'incertitude faible, force la ZUPT (Zero Velocity Update)
        if (this.P[0] < this.R_base * 0.1 && this.x[0] < MIN_SPD) {
             this.x[0] = 0;
             this.P[0] = this.P[0] * 0.5; // Réduction d'incertitude
        }

        return { kSpd: this.x[0], kUncert: this.P[0] };
    }
}


// ===========================================
// CLASSE QUATERNION (Pour gestion de l'IMU)
// ===========================================

class Quaternion {
    constructor(w = 1, x = 0, y = 0, z = 0) {
        this.w = w; this.x = x; this.y = y; this.z = z;
    }

    static fromAcc(ax, ay, az) {
        // Calcule l'orientation à partir du vecteur d'accélération (assumant que l'accélération statique est la gravité - non-filtré)
        const g = Math.sqrt(ax * ax + ay * ay + az * az);
        if (g === 0) return new Quaternion(); // Éviter la division par zéro

        const gx = ax / g, gy = ay / g, gz = az / g;

        // Simplification: le pitch et le roll sont faciles à obtenir
        const roll = Math.atan2(gy, gz);
        const pitch = Math.atan2(-gx, Math.sqrt(gy * gy + gz * gz));
        const yaw = 0; // Le cap (Yaw) ne peut pas être déterminé par la seule accélération

        const c1 = Math.cos(roll / 2);
        const s1 = Math.sin(roll / 2);
        const c2 = Math.cos(pitch / 2);
        const s2 = Math.sin(pitch / 2);
        const c3 = Math.cos(yaw / 2);
        const s3 = Math.sin(yaw / 2);

        const w = c1 * c2 * c3 - s1 * s2 * s3;
        const x = s1 * c2 * c3 + c1 * s2 * s3;
        const y = c1 * s2 * c3 - s1 * c2 * s3;
        const z = s1 * s2 * c3 + c1 * c2 * s3;

        return new Quaternion(w, x, y, z);
    }

    toEuler() {
        const x = this.x, y = this.y, z = this.z, w = this.w;

        // Roll (rotation autour de l'axe X)
        const sinr_cosp = 2 * (w * x + y * z);
        const cosr_cosp = 1 - 2 * (x * x + y * y);
        const roll = Math.atan2(sinr_cosp, cosr_cosp);

        // Pitch (rotation autour de l'axe Y)
        let pitch = 2 * (w * y - z * x);
        if (pitch > 1) pitch = 1;
        if (pitch < -1) pitch = -1;
        pitch = Math.asin(pitch);

        // Yaw (rotation autour de l'axe Z)
        const siny_cosp = 2 * (w * z + x * y);
        const cosy_cosp = 1 - 2 * (y * y + z * z);
        const yaw = Math.atan2(siny_cosp, cosy_cosp);

        return { roll: roll * R2D, pitch: pitch * R2D, yaw: yaw * R2D };
    }
}


// ===========================================
// FONCTIONS MODÈLES ET MATHÉMATIQUES
// ===========================================

/**
 * Calcul de la distance 2D entre deux points (Formule Haversine).
 * @param {number} lat1 Latitude 1
 * @param {number} lon1 Longitude 1
 * @param {number} lat2 Latitude 2
 * @param {number} lon2 Longitude 2
 * @param {number} R_earth Rayon terrestre moyen (m)
 * @returns {number} Distance en mètres
 */
function dist2D(lat1, lon1, lat2, lon2, R_earth) {
    const dLat = (lat2 - lat1) * D2R;
    const dLon = (lon2 - lon1) * D2R;
    const lat1Rad = lat1 * D2R;
    const lat2Rad = lat2 * D2R;

    const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
              Math.cos(lat1Rad) * Math.cos(lat2Rad) *
              Math.sin(dLon / 2) * Math.sin(dLon / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R_earth * c; 
}


/**
 * Détermine la covariance de mesure dynamique (R) pour l'UKF Vitesse.
 * R est ajusté en fonction de la précision GPS (accRaw) et de l'environnement sélectionné.
 * @param {number} accRaw Précision GPS (m)
 * @param {number} kAlt Altitude actuelle filtrée (m)
 * @param {number} lastP Pression atmosphérique (hPa)
 * @param {string} env Environnement sélectionné (clé dans ENVIRONMENT_FACTORS)
 * @returns {number} R dynamique (m²/s²)
 */
function getKalmanR(accRaw, kAlt, lastP, env) {
    // R_min est basé sur la précision GPS au carré (m²)
    let R_gps_base = Math.min(accRaw, 100) ** 2; 

    // Ajustement basé sur la qualité du signal (si R_gps_base devient trop grand, il est saturé)
    if (accRaw > 100) {
        R_gps_base = 100 ** 2 + (accRaw - 100) * 10;
    }
    
    // Facteur d'environnement (multiplicateur)
    const env_mult = ENVIRONMENT_FACTORS[env]?.R_MULT || 1.0;
    
    // R dynamique est une combinaison de la variance de position et de l'incertitude environnementale
    // On divise par une valeur pour obtenir R en m²/s² (R = sigma_v^2 * dt^2 / dt^2), mais ici on simplifie
    // en supposant que R = sigma_position^2. On utilise UKF_R_MAX pour plafonner.
    let R_dyn = Math.min(R_gps_base * env_mult, UKF_R_MAX);

    // Si la pression est très basse (haute altitude), on augmente légèrement l'incertitude
    if (lastP < 800) { R_dyn *= 1.1; }

    return Math.max(R_dyn, R_ALT_MIN); // R ne doit jamais être trop petit
}


/**
 * Filtre de Kalman (EKF-like) pour l'altitude (fusion GPS + Baromètre).
 * @param {number} kAlt_prev Altitude filtrée précédente (m)
 * @param {number} P_prev Covariance précédente (m²)
 * @param {number} altRaw_gps Mesure GPS brute (m)
 * @param {number} R_gps Covariance de mesure GPS (m²)
 * @param {number} dt Delta temps (s)
 * @param {number} baroAlt Mesure barométrique corrigée (m)
 * @returns {{kAlt: number, kAltUncert: number}} Nouvelle altitude et covariance
 */
function kFilterAltitude(kAlt_prev, P_prev, altRaw_gps, R_gps, dt, baroAlt) {
    // Modèle: x = x (altitude ne change pas sans raison)
    const Q = 0.01; // Bruit de processus (changement lent de l'altitude)

    // 1. Prédiction (simple car le modèle est statique : x_pred = x_prev)
    const kAlt_pred = kAlt_prev || altRaw_gps || 0;
    const P_pred = P_prev + Q; // La covariance augmente

    // 2. Fusion (Correction)
    // Nous avons deux mesures: GPS et Baromètre. Nous les fusionnons séquentiellement.

    // Mesure 1: GPS
    let H_gps = 1; // Matrice d'observation: z = Hx
    let R_eff_gps = R_gps || R_ALT_MIN; 
    let K_gps = P_pred * H_gps / (H_gps * P_pred * H_gps + R_eff_gps);
    let kAlt_1 = kAlt_pred + K_gps * (altRaw_gps - kAlt_pred);
    let P_1 = (1 - K_gps * H_gps) * P_pred;
    
    // Mesure 2: Baromètre (si disponible et valide)
    if (baroAlt !== null && P_1 > R_ALT_MIN * 0.1) {
        const R_baro = 5.0; // Le baromètre est assez précis (faible R)
        let H_baro = 1;
        let K_baro = P_1 * H_baro / (H_baro * P_1 * H_baro + R_baro);
        
        let kAlt_new = kAlt_1 + K_baro * (baroAlt - kAlt_1);
        let P_new = (1 - K_baro * H_baro) * P_1;
        
        return { kAlt: kAlt_new, kAltUncert: P_new };
    }

    return { kAlt: kAlt_1, kAltUncert: P_1 };
}


/**
 * Calcule l'altitude barométrique corrigée.
 * @param {number} P_hPa Pression actuelle (hPa)
 * @param {number} P_ref_hPa Pression de référence (hPa)
 * @param {number} T_K Température de l'air (K)
 * @returns {number|null} Altitude barométrique (m) ou null
 */
function getBarometricAltitude(P_hPa, P_ref_hPa, T_K) {
    if (P_hPa === null || T_K === null) return null;
    
    // Formule basée sur l'atmosphère standard ISA
    // H = T0 / L * (1 - (P/P0)^(L*R / g)) 
    // T0 = 288.15 K, L = -0.0065 K/m (gradient temp.), g = 9.80665 m/s², R = 287.058 J/kg·K (air)
    
    const T0 = 288.15;
    const L = 0.0065;
    const g = G_ACC; 
    const R = 287.058;
    
    // L'altitude est calculée par la température réelle pour une meilleure précision
    const alt = (T_K / L) * (1 - (P_hPa / P_ref_hPa)**(R * L / g));
    
    return alt;
}

/**
 * Calcule le Facteur de Référence Multidimensionnel (MRF).
 * Utilisé pour ajuster la distance parcourue dans des environnements modifiés (ex: Nether/Souterrain).
 * @param {number} alt Altitude filtrée (m)
 * @param {boolean} netherMode Mode Nether activé
 * @returns {number} Multiplicateur de distance
 */
function calculateMRF(alt, netherMode) {
    if (netherMode) {
        return 8.0; // Facteur du Nether (8:1)
    }
    // Facteur d'ajustement souterrain léger (par exemple, si Alt < 0)
    if (alt < -20) {
        return 1.1; 
    }
    return 1.0; // Facteur standard
}

/**
 * Met à jour les constantes physiques pour un corps céleste différent.
 * @param {string} body Nom du corps céleste
 * @param {number} alt Altitude actuelle (m)
 * @param {number} rotationRadius Rayon de rotation (pour ROTATING)
 * @param {number} angularVelocity Vitesse angulaire (rad/s)
 * @returns {{G_ACC: number, R_ALT_CENTER_REF: number}} Nouvelles constantes
 */
function updateCelestialBody(body, alt, rotationRadius, angularVelocity) {
    let G_ACC_NEW = 9.80665;
    let R_ALT_CENTER_REF_NEW = 6371000;

    switch (body) {
        case 'MARS':
            G_ACC_NEW = 3.721; 
            R_ALT_CENTER_REF_NEW = 3389500;
            break;
        case 'MOON':
            G_ACC_NEW = 1.62; 
            R_ALT_CENTER_REF_NEW = 1737400;
            break;
        case 'ROTATING':
            // Calcule l'accélération centripète comme gravité effective
            G_ACC_NEW = rotationRadius * angularVelocity ** 2;
            R_ALT_CENTER_REF_NEW = 10000; // Rayon arbitraire pour un système rotatif
            break;
        case 'EARTH':
        default:
            G_ACC_NEW = 9.80665;
            R_ALT_CENTER_REF_NEW = 6371000;
            break;
    }
    
    // Mettre à jour la constante globale G_ACC (pour la gravité WGS84, par exemple)
    // NOTE: La fonction `getWGS84Gravity` doit utiliser la constante mise à jour.
    return { G_ACC: G_ACC_NEW, R_ALT_CENTER_REF: R_ALT_CENTER_REF_NEW };
    }
// =================================================================
// BLOC 4.2/4 : Services Externes & Calculs Astro/Physique
// =================================================================

// --- CONSTANTES DE SERVICE ET PHYSIQUE ---
const NTP_API_URL = 'https://worldtimeapi.org/api/ip'; 
const WEATHER_API_KEY = 'YOUR_WEATHER_API_KEY_HERE'; // CLÉ API REQUISE
const WEATHER_API_URL = 'https://api.openweathermap.org/data/2.5/weather'; 
const R_AIR = 287.058;      // Constante spécifique de l'air sec (J/kg·K)
const GAMMA = 1.4;          // Indice adiabatique de l'air

// --- Variables d'état partagées (doivent être déclarées dans app_core.js) ---
// Note: G_ACC est utilisée ici pour les calculs WGS84 et la météo.
// Si elle est mise à jour dans ukf_models.js, elle est disponible globalement.

// ===========================================
// GESTION DU TEMPS ET NTP
// ===========================================

/**
 * Synchronise l'heure locale avec une heure de service (NTP).
 * @param {number|null} lServH Heure de service précédente (ms)
 * @param {number|null} lLocH Heure locale précédente (ms)
 * @returns {Promise<{lServH: number, lLocH: number}>} Nouveau temps de référence
 */
async function syncH(lServH, lLocH) {
    if (lServH === null) {
        // Première synchronisation, utilise une API publique
        try {
            const response = await fetch(NTP_API_URL);
            const data = await response.json();
            
            if (data.unixtime) {
                const now_ms = Date.now();
                const serv_ms = data.unixtime * 1000;
                
                if (Math.abs(serv_ms - now_ms) > 1000) {
                     // Si la différence est supérieure à 1s, on corrige.
                     if (document.getElementById('local-time')) {
                         document.getElementById('local-time').textContent = `🕒 CORRECTION: ${(serv_ms - now_ms) / 1000}s`;
                     }
                }
                return { lServH: serv_ms, lLocH: now_ms };
            }
        } catch (e) {
            console.error("Erreur de synchronisation NTP:", e);
            if (document.getElementById('local-time')) {
                document.getElementById('local-time').textContent = '❌ SYNCHRO ÉCHOUÉE (Local)';
            }
            // Retourne le temps local comme fallback
            return { lServH: Date.now(), lLocH: Date.now() };
        }
    }
    // Synchronisation subséquente, utilise simplement le décalage calculé
    return { lServH: lServH, lLocH: lLocH };
}

/**
 * Calcule l'heure actuelle corrigée par NTP.
 * @param {number|null} lServH Heure de service de référence (ms)
 * @param {number|null} lLocH Heure locale de référence (ms)
 * @returns {Date|null} Objet Date corrigé
 */
function getCDate(lServH, lLocH) {
    if (lServH === null || lLocH === null) {
        return new Date(); // Fallback
    }
    const offset = lServH - lLocH;
    return new Date(Date.now() + offset);
}

// ===========================================
// CALCULS GÉOPHYSIQUES ET MÉTÉOROLOGIQUES
// ===========================================

/**
 * Calcule l'accélération gravitationnelle locale selon le modèle WGS84.
 * @param {number} lat Latitude (degrés)
 * @param {number} alt Altitude (m)
 * @returns {number} Gravité locale (m/s²)
 */
function getWGS84Gravity(lat, alt) {
    // Formule WGS84 (gravité à la surface)
    const latRad = lat * D2R;
    const g0 = 9.780327 * (1 + 0.0053024 * Math.sin(latRad)**2 - 0.0000058 * Math.sin(2 * latRad)**2);
    
    // Correction d'altitude (Free-Air Correction - FAC)
    const FAC = 3.086e-6 * alt; // 3.086 µGal/m
    
    // Correction de Plateau (Bouguer Correction - BC) - Simplifiée/Ignorée ici
    // g = g0 - FAC
    return g0 - FAC; 
}


/**
 * Calcule la Vitesse Vraie de l'Air (True Airspeed - TAS).
 * @param {number} spdMS Vitesse sol (m/s)
 * @param {number} rhoAir Masse volumique de l'air (kg/m³)
 * @returns {number} TAS (m/s) - Note: Simplification majeure, TAS = IAS * sqrt(rho_0 / rho)
 */
function getTrueAirspeed(spdMS, rhoAir) {
    // TAS ≈ IAS (Indicated Airspeed) * sqrt(rho_0 / rho)
    // spdMS est ici l'IAS (vitesse sol)
    const rho0 = RHO_SEA_LEVEL;
    
    if (rhoAir === 0 || rhoAir === null) return spdMS; // Éviter la division par zéro
    
    // Si la masse volumique de l'air est plus faible, la TAS est plus élevée que la vitesse sol.
    return spdMS * Math.sqrt(rho0 / rhoAir);
}

/**
 * Calcule la Vitesse du Son dans l'air.
 * @param {number} tempK Température de l'air (Kelvin)
 * @returns {number} Vitesse du son (m/s)
 */
function getSpeedOfSound(tempK) {
    // c = sqrt(gamma * R * T)
    return Math.sqrt(GAMMA * R_AIR * tempK);
}


// ===========================================
// SERVICES MÉTÉO (OpenWeatherMap)
// ===========================================

/**
 * Récupère les données météorologiques actuelles.
 * @param {number} lat Latitude
 * @param {number} lon Longitude
 * @returns {Promise<{pressure_hPa: number, tempC: number, tempK: number, air_density: number}|null>} Données météo
 */
async function fetchWeather(lat, lon) {
    if (!WEATHER_API_KEY.includes('YOUR_WEATHER_API_KEY')) {
        const url = `${WEATHER_API_URL}?lat=${lat}&lon=${lon}&units=metric&appid=${WEATHER_API_KEY}`;
        try {
            const response = await fetch(url);
            const data = await response.json();

            if (data.main) {
                const P_hPa = data.main.pressure;
                const T_C = data.main.temp;
                const T_K = T_C + 273.15;
                
                // Calcul de la masse volumique de l'air (Rho)
                // Équation d'état des gaz parfaits: P = Rho * R_specific * T
                // Rho = P / (R_specific * T)
                const P_Pa = P_hPa * 100; // Pression en Pascals
                const R_specific = R_AIR;
                
                const air_density = P_Pa / (R_specific * T_K);

                return {
                    pressure_hPa: P_hPa,
                    tempC: T_C,
                    tempK: T_K,
                    air_density: air_density
                };
            }
        } catch (e) {
            console.error("Erreur de récupération météo:", e);
        }
    }
    return null;
}

// ===========================================
// CALCULS ASTRONOMIQUES (Simplifié)
// ===========================================

/**
 * Met à jour l'affichage des données astronomiques (Soleil/Lune).
 * @param {number} lat Latitude
 * @param {number} lon Longitude
 * @param {number} lServH Heure de service
 * @param {number} lLocH Heure locale
 */
function updateAstro(lat, lon, lServH, lLocH) {
    // Calcul de l'heure actuelle corrigée
    const now = getCDate(lServH, lLocH);
    
    // --- Calcul Heure Solaire Moyenne (MST) ---
    // MST = UTC + Longitude / 15
    const utcHours = now.getUTCHours() + now.getUTCMinutes() / 60 + now.getUTCSeconds() / 3600;
    const mstHours = utcHours + lon / 15;
    
    // Conversion en format HH:MM:SS
    const h = Math.floor(mstHours % 24);
    const m = Math.floor((mstHours * 60) % 60);
    const s = Math.floor((mstHours * 3600) % 60);
    
    if (document.getElementById('mst')) {
        document.getElementById('mst').textContent = 
            `${h.toString().padStart(2, '0')}:${m.toString().padStart(2, '0')}:${s.toString().padStart(2, '0')}`;
    }

    // --- Placeholder (Les calculs de position céleste sont trop complexes pour un seul fichier) ---
    // Les fonctions de calcul précis de l'azimut/altitude du Soleil/Lune nécessitent une librairie complète
    // (ex: SunCalc ou Ephemeris) ou un grand nombre de formules d'éphémérides.
    // Nous mettons à jour l'affichage avec un placeholder.
    if (document.getElementById('sun-alt')) document.getElementById('sun-alt').textContent = `~Calcul en cours...`;
    if (document.getElementById('moon-phase-name')) document.getElementById('moon-phase-name').textContent = `~Calcul en cours...`;
    }
