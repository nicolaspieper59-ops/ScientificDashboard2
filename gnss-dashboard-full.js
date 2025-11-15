// =================================================================
// BLOC 1/3 : CoreUtils.js
// Constantes, Calculs Physiques/Mathématiques, NTP et Services Astro/Météo
// =================================================================

const $ = id => document.getElementById(id); // Utilitaires pour le DOM

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const R_E_BASE = 6371000;   // Rayon terrestre moyen (m)
const KMH_MS = 3.6;         // Conversion m/s vers km/h
const C_S_BASE = 343;       // Vitesse du son de base (m/s)
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const R_AIR = 287.058;      // Constante spécifique de l'air sec (J/kg·K)
const KELVIN_OFFSET = 273.15; // Conversion Celsius vers Kelvin

// --- PARAMÈTRES DE CONTRÔLE ET ENVIRONNEMENT ---
const MAX_ACC = 200;        // Précision max (m) avant "Estimation Seule"
const MIN_SPD = 0.05;       // Vitesse minimale "en mouvement"
const ALT_TH = -50;         // Seuil d'altitude "Sous-sol"
const NETHER_RATIO = 8.0;   // Ratio Nether
const MIN_DT = 0.01;        // Temps minimum (en sec) pour une mise à jour
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0, DISPLAY: 'Normal' },
    'FOREST': { R_MULT: 2.5, DISPLAY: 'Forêt' },
    'CONCRETE': { R_MULT: 7.0, DISPLAY: 'Grotte/Tunnel' },
    'METAL': { R_MULT: 5.0, DISPLAY: 'Métal/Bâtiment' },
};
const CELESTIAL_DATA = {
    'EARTH': { G: 9.80665, R: R_E_BASE, name: 'Terre' },
    'MOON': { G: 1.62, R: 1737400, name: 'Lune' },
    'MARS': { G: 3.71, R: 3389500, name: 'Mars' },
    'ROTATING': { G: 0.0, R: R_E_BASE, name: 'Station Spatiale' }
};

// --- CONSTANTES DE TEMPS & CALENDRIER ---
const MC_DAY_MS = 72 * 60 * 1000; 
const J1970 = 2440588, J2000 = 2451545;
const dayMs = 1000 * 60 * 60 * 24;


// --------------------------------------------------------------------------
// --- UTILITAIRES MATHÉMATIQUES ET PHYSIQUES ---
// --------------------------------------------------------------------------

/** Calcule la distance de Haversine en mètres */
const dist = (lat1, lon1, lat2, lon2, R_ref) => {
    const R = R_ref || R_E_BASE; 
    const dLat = (lat2 - lat1) * D2R, dLon = (lon2 - lon1) * D2R;
    lat1 *= D2R; lat2 *= D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R * c;
};

/** Calcule l'accélération gravitationnelle locale ou artificielle. */
function getGravityLocal(alt, bodyKey, r_rot, omega_rot) {
    if (bodyKey === 'ROTATING') {
        return r_rot * omega_rot ** 2; 
    }
    
    if (alt === null) alt = 0;
    const g_base = CELESTIAL_DATA[bodyKey].G;
    const R_base = CELESTIAL_DATA[bodyKey].R;
    
    return g_base * (R_base / (R_base + alt)) ** 2;
}

/** Met à jour les constantes physiques globales lors du changement de corps céleste. */
function updateCelestialBody(bodyKey, alt, r_rot, omega_rot) {
    let G_ACC_local = 0;
    let R_ALT_CENTER_REF_local = R_E_BASE;

    if (bodyKey === 'ROTATING') {
        G_ACC_local = getGravityLocal(alt, bodyKey, r_rot, omega_rot);
    } else {
        const data = CELESTIAL_DATA[bodyKey];
        if (data) {
            G_ACC_local = data.G;
            R_ALT_CENTER_REF_local = data.R;
        }
    }
    
    return { G_ACC: G_ACC_local, R_ALT_CENTER_REF: R_ALT_CENTER_REF_local };
}

/** Calcule le Facteur de Rapport de Mouvement (MRF). */
function calculateMRF(alt, netherMode) {
    if (netherMode) return 1.0 / NETHER_RATIO;
    if (alt !== null && alt < ALT_TH) return 0.5; 
    return 1.0;
}

/** Calcule la vitesse du son locale et le nombre de Mach */
function calculateLocalSpeed(tempC, speedMS) {
    const tempK = tempC + KELVIN_OFFSET;
    // C_S = sqrt(gamma * R_air * T) ; gamma = 1.4 pour l'air
    const C_S_local = Math.sqrt(1.4 * R_AIR * tempK); 
    const mach = speedMS / C_S_local;
    return { C_S_local, mach };
}

// --------------------------------------------------------------------------
// --- LOGIQUE DE SERVICES (NTP) ---
// --------------------------------------------------------------------------

/** Retourne l'heure synchronisée (précision RTT compensée en UTC). */
function getCDate(lServH, lLocH) { 
    if (lServH === null || lLocH === null) { return null; }
    const offsetSinceSync = performance.now() - lLocH;
    return new Date(lServH + offsetSinceSync); 
}

/** Synchronise l'horloge interne avec un serveur de temps (UTC/Atomique) */
async function syncH(lServH_in, lLocH_in) {
    let lServH = lServH_in;
    let lLocH = lLocH_in;
    
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
        console.log(`Synchronisation UTC Atomique réussie. Latence corrigée: ${latencyOffset.toFixed(1)} ms.`);
        
    } catch (error) {
        console.warn("Échec de la synchronisation. Utilisation de l'horloge locale.", error);
        lServH = Date.now(); 
        lLocH = performance.now();
        if ($('local-time')) $('local-time').textContent = 'SYNCHRO ÉCHOUÉE';
    }
    return { lServH, lLocH };
}

// --------------------------------------------------------------------------
// --- LOGIQUE DE SERVICES (MÉTÉO) ---
// --------------------------------------------------------------------------

/** Récupère et traite les données météo via l'API */
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
            const tempK = tempC + KELVIN_OFFSET;
            
            const pressure_pa = pressure_hPa * 100;
            // Densité de l'air (Loi des gaz parfaits) : rho = P / (R_air * T)
            const air_density = pressure_pa / (R_AIR * tempK);
            
            // Calcul du Point de Rosée (approximation Magnus-Tetens)
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
                dew_point: dew_point,
                status: 'ACTIF' // Correction du statut Météo
            };
        } else {
             throw new Error(data.message || 'Données météo incomplètes');
        }
    } catch (err) {
        console.warn("Erreur de récupération météo:", err.message);
        if ($('weather-status')) $('weather-status').textContent = 'ÉCHEC RÉCUPÉRATION';
    }
    return weatherData; 
}


// --------------------------------------------------------------------------
// --- LOGIQUE DE SERVICES (ASTRO) ---
// --------------------------------------------------------------------------

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
    
    // CORRECTION: Longitude Écliptique contrainte entre 0 et 360 degrés
    const eclipticLongitudeDeg = (L * R2D) % 360;
    if (eclipticLongitudeDeg < 0) eclipticLongitudeDeg += 360;

    return { 
        TST: toTimeString(tst_ms), 
        MST: toTimeString(mst_ms), 
        EOT: eot_min.toFixed(2),
        ECL_LONG: eclipticLongitudeDeg.toFixed(2),
        NoonSolar: toTimeString(J_transit * dayMs)
    };
}

function getMinecraftTime(date) {
    if (date === null) return '00:00:00';
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

/** Fonction principale de mise à jour Astro (Nécessite SunCalc global) */
function updateAstro(latA, lonA, lServH, lLocH) {
    const now = getCDate(lServH, lLocH); 
    if (now === null) return null; 
    
    if (typeof SunCalc === 'undefined' || !latA || !lonA) return null;
    
    // SunCalc doit être importé dans le HTML pour que ces fonctions existent
    const sunPos = SunCalc.getPosition(now, latA, lonA);
    const moonIllum = SunCalc.getMoonIllumination(now);
    const moonPos = SunCalc.getMoonPosition(now, latA, lonA);
    const sunTimes = SunCalc.getTimes(now, latA, lonA);
    const moonTimes = SunCalc.getMoonTimes(now, latA, lonA, true);
    const solarTimes = getSolarTime(now, lonA);
    
    return { now, sunPos, moonIllum, moonPos, sunTimes, moonTimes, solarTimes };
}
// =================================================================
// BLOC 2/3 : EKF_21State.js
// Architecture et Logique du Filtre de Kalman Étendu 21 États (EKF INS/GNSS)
// =================================================================

// --- CONSTANTES SPÉCIFIQUES EKF ---
const N_STATES = 21;            // EKF : POS(3), VEL(3), QUAT(4), BIAS_G(3), BIAS_A(3), MAG_ERR(5)
const ZUPT_ACCEL_TOLERANCE = 0.5; // Tolérance pour Zero Velocity Update (m/s²)
const ZUPT_RAW_THRESHOLD = 1.0;     // Vitesse brute max (m/s)

// --- VARIABLES D'ÉTAT EKF ---
// X: [lat, lon, alt, V_n, V_e, V_d, q_w, q_x, q_y, q_z, bias_g_x, bias_g_y, bias_g_z, bias_a_x, bias_a_y, bias_a_z, mag_err_x, mag_err_y, mag_err_z, mag_err_h, mag_err_v]
let EKF_state = new Array(N_STATES).fill(0.0); 
// P: Matrice de Covariance (N_STATES x N_STATES)
let EKF_P = new Array(N_STATES * N_STATES).fill(0.0);
let currentEnvFactor = 1.0; // Facteur R de l'environnement (1.0 = Normal)

// --- Fonctions de Matrices Simplifiées (Placeholders pour math.js) ---

// Les fonctions de manipulation matricielle (add, mult, transpose, inverse)
// sont généralement fournies par une librairie (comme math.js). 
// Elles sont ici simplifiées pour illustrer la structure de l'EKF.
const identityMatrix = (n) => {
    const I = new Array(n * n).fill(0.0);
    for (let i = 0; i < n; i++) I[i * n + i] = 1.0;
    return I;
};

// --------------------------------------------------------------------------
// --- LOGIQUE CORE EKF ---
// --------------------------------------------------------------------------

/** Initialise l'état EKF et la matrice de covariance P */
function initEKF(lat_init, lon_init, alt_init, acc_init) {
    console.log(`Initialisation EKF 21 états à Lat: ${lat_init.toFixed(4)}`);
    
    // 1. Initialisation de l'État X
    EKF_state[0] = lat_init * D2R; // rad
    EKF_state[1] = lon_init * D2R; // rad
    EKF_state[2] = alt_init;       // m
    
    // Vitesse (4, 5, 6) à zéro
    // Quaternion (7, 8, 9, 10) à [1, 0, 0, 0] (pas d'erreur d'attitude initiale)
    EKF_state[7] = 1.0; 
    
    // Biais (11-19) et Erreurs Magnéto (20-24) à zéro par défaut
    
    // 2. Initialisation de la Matrice de Covariance P
    EKF_P = identityMatrix(N_STATES);
    
    // Position (Lat/Lon/Alt) - Erreur basée sur l'incertitude GNSS (acc_init)
    EKF_P[0 * N_STATES + 0] = (acc_init / R_E_BASE) ** 2; // Lat (rad)²
    EKF_P[1 * N_STATES + 1] = (acc_init / R_E_BASE) ** 2; // Lon (rad)²
    EKF_P[2 * N_STATES + 2] = acc_init ** 2;             // Alt (m)²

    // Vitesse (Tuning agressif pour la confiance INS)
    EKF_P[3 * N_STATES + 3] = 100.0;
    EKF_P[4 * N_STATES + 4] = 100.0;
    EKF_P[5 * N_STATES + 5] = 100.0;
    
    // Biais et Erreurs des capteurs à des valeurs conservatives
    EKF_P[11 * N_STATES + 11] = 1e-4; // Gyroscope Bias (rad/s)²
    EKF_P[14 * N_STATES + 14] = 1e-2; // Accelerometer Bias (m/s²)²
    
    // Mise à jour de l'état d'affichage pour l'Application.js
    currentEKFState = {
        lat: lat_init, lon: lon_init, alt: alt_init,
        V_n: 0, V_e: 0, V_d: 0,
        acc_est: Math.sqrt(EKF_P[0*N_STATES+0]) * R_E_BASE // Estime de l'incertitude position
    };
}

/** * Étape de PRÉDICTION (Propagation Inertielle)
 * Simule la propagation de l'état EKF basé sur les mesures IMU (acc, gyro)
 * et la matrice de covariance P.
 */
function predictEKF(dt, acc_meas, gyro_meas, g_local, R_ref) {
    if (dt === 0) return;
    
    // 1. Calcul de la Matrice de Transition d'État (F) et de Bruit (Q)
    // F et Q dépendent de l'état actuel et de dt. (Omise ici pour la simplicité)
    
    // 2. Propagation de l'État (Phi, Omega, Kinématique)
    // EKF_state_new = f(EKF_state_old, acc_meas, gyro_meas, dt, g_local)
    
    // Simulation simplifiée de la propagation de la position/vitesse (pour les tests)
    EKF_state[0] += EKF_state[3] / R_ref * dt; // lat += V_n / R * dt
    EKF_state[1] += EKF_state[4] / R_ref * dt / Math.cos(EKF_state[0]); // lon += V_e / (R*cos(lat)) * dt
    EKF_state[2] += EKF_state[5] * dt; // alt += V_d * dt
    
    // 3. Propagation de la Covariance P (P = F*P*F^T + Q)
    // P_new = math.add(math.multiply(F, math.multiply(EKF_P, math.transpose(F))), Q)
    
    // Augmentation de l'incertitude (Q) simplifiée:
    const Q_pos_factor = currentEnvFactor * dt;
    EKF_P[0 * N_STATES + 0] += Q_pos_factor;
    EKF_P[1 * N_STATES + 1] += Q_pos_factor;
    EKF_P[2 * N_STATES + 2] += Q_pos_factor;
    // Bruit de vitesse agressif pour compenser
    EKF_P[3 * N_STATES + 3] += 1000.0 * dt; 
    EKF_P[4 * N_STATES + 4] += 1000.0 * dt; 
    EKF_P[5 * N_STATES + 5] += 1000.0 * dt; 
}


/** * Étape de CORRECTION (Mise à jour GNSS)
 * Fusionne les mesures GNSS (position, vitesse si disponible) avec l'état prédit.
 */
function updateEKF_GNSS(gnss_pos, gnss_vel, gnss_acc, alt_acc) {
    
    // 1. Définir le Vecteur de Mesure (Z) et la Matrice de Covariance de Mesure (R)
    // Z = [lat_gnss, lon_gnss, alt_gnss]
    const R_pos = gnss_acc ** 2 * currentEnvFactor; 
    const R_alt = alt_acc ** 2 * currentEnvFactor;

    // 2. Calculer le Gain de Kalman (K = P * H^T * (H * P * H^T + R)^-1)
    // H est la Matrice d'Observation (Omise ici)
    
    // 3. Calculer le Résidu (Y = Z - h(X))
    // h(X) est la fonction d'observation (extraction de la position de l'état EKF)
    
    // 4. Mettre à jour l'État (X_new = X + K * Y)
    // EKF_state = math.add(EKF_state, math.multiply(K, Y))
    
    // 5. Mettre à jour la Covariance (P_new = (I - K * H) * P)
    // EKF_P = math.multiply(math.subtract(identityMatrix(N_STATES), math.multiply(K, H)), EKF_P)
    
    // Simulation simplifiée de la correction de la position (lissage)
    const K_sim = 0.05; // Gain de correction (très simplifié)
    EKF_state[0] = (1 - K_sim) * EKF_state[0] + K_sim * gnss_pos.lat * D2R;
    EKF_state[1] = (1 - K_sim) * EKF_state[1] + K_sim * gnss_pos.lon * D2R;
    EKF_state[2] = (1 - K_sim) * EKF_state[2] + K_sim * gnss_pos.alt;
}

/** * Étape de CORRECTION ZUPT (Zero Velocity Update)
 * Force l'état de vitesse à zéro si le véhicule est immobile.
 */
function updateEKF_ZUPT() {
    // 1. Mesure Z = [0, 0, 0] (Vitesse N, E, D = 0)
    
    // 2. Matrice d'Observation H (H extrait les indices de vitesse 3, 4, 5)
    
    // 3. Matrice de Covariance de Mesure R (Très faible pour forcer la correction)
    
    // 4. Mise à jour de l'État (X_new = X + K * Y) et de la Covariance (P_new)
    
    // Simulation : Forcer la vitesse à zéro et réduire son incertitude
    EKF_state[3] = 0.0;
    EKF_state[4] = 0.0;
    EKF_state[5] = 0.0;
    
    // Réinitialiser la covariance de vitesse à une valeur minimale
    EKF_P[3 * N_STATES + 3] = 0.01;
    EKF_P[4 * N_STATES + 4] = 0.01;
    EKF_P[5 * N_STATES + 5] = 0.01;
}

// --------------------------------------------------------------------------
// --- UTILITAIRES DE LECTURE EKF ---
// --------------------------------------------------------------------------

let currentEKFState = { lat: 0, lon: 0, alt: 0, V_n: 0, V_e: 0, V_d: 0, acc_est: 1000 };

/** Retourne la vitesse 3D estimée par l'EKF */
function getEKFVelocity3D() {
    const V_n = EKF_state[3];
    const V_e = EKF_state[4];
    const V_d = EKF_state[5];
    return Math.sqrt(V_n ** 2 + V_e ** 2 + V_d ** 2);
}

/** Met à jour l'état lisible pour l'affichage DOM */
function updateEKFReadableState() {
    currentEKFState.lat = EKF_state[0] * R2D;
    currentEKFState.lon = EKF_state[1] * R2D;
    currentEKFState.alt = EKF_state[2];
    currentEKFState.V_n = EKF_state[3];
    currentEKFState.V_e = EKF_state[4];
    currentEKFState.V_d = EKF_state[5];
    // Erreur de position estimée (position RMS en mètres)
    currentEKFState.acc_est = Math.sqrt(EKF_P[0 * N_STATES + 0]) * R_E_BASE; 
}

/** Calcule l'incertitude de la vitesse (Diagonale P[3,3], P[4,4], P[5,5]) */
function getVelocityUncertainty() {
    return (EKF_P[3 * N_STATES + 3] + EKF_P[4 * N_STATES + 4] + EKF_P[5 * N_STATES + 5]) / 3.0;
        }
// =================================================================
// BLOC 3/3 : AppController.js
// État Global, Gestion des Capteurs, Boucle de Mise à Jour (EKF), et DOM
// =================================================================

// --- CONSTANTES DE CONFIGURATION SYSTÈME ---
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};
const DOM_SLOW_UPDATE_MS = 1000; 
let lastMapUpdate = 0; 
const MAP_UPDATE_INTERVAL = 3000; 
const DEFAULT_INIT_LAT = 45.749950;
const DEFAULT_INIT_LON = 4.850027;
const DEFAULT_INIT_ALT = 2.64;

// --- VARIABLES D'ÉTAT (Globales) ---
let wID = null, domID = null, lPos = null, sTime = null;
let distM_3D = 0, maxSpd = 0;
let timeMoving = 0; 
let lServH = null, lLocH = null;    // Horloge NTP
let lastFSpeed = 0; 

// État Physique et Contrôles
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
let G_ACC = CELESTIAL_DATA['EARTH'].G;      
let R_ALT_CENTER_REF = R_E_BASE;            

// Données externes et IMU
let lastP_hPa = null, lastT_K = null, lastH_perc = null; 
let real_accel_x = 0, real_accel_y = 0, real_accel_z = 0;
let lastAccelLong = 0;

// Objets Map (Leaflet)
let map, marker, circle;


// --------------------------------------------------------------------------
// --- GESTION DES CAPTEURS IMU ---
// --------------------------------------------------------------------------

function imuMotionHandler(event) {
    // Les mesures brutes sont stockées. L'EKF les utilisera pour la PRÉDICTION.
    if (event.acceleration) {
        real_accel_x = event.acceleration.x || 0;
        real_accel_y = event.acceleration.y || 0;
        real_accel_z = event.acceleration.z || 0;
        if ($('imu-status')) $('imu-status').textContent = "Actif (Sans Gravité)";
    } 
    else if (event.accelerationIncludingGravity) {
        real_accel_x = event.accelerationIncludingGravity.x || 0;
        real_accel_y = event.accelerationIncludingGravity.y || 0;
        real_accel_z = event.accelerationIncludingGravity.z || 0;
        if ($('imu-status')) $('imu-status').textContent = "Actif (Avec Gravité)";
    } else {
        if ($('imu-status')) $('imu-status').textContent = "Erreur (Capteur N/A)";
    }
}

function startIMUListeners() {
    if (window.DeviceMotionEvent) {
        if (typeof DeviceMotionEvent.requestPermission === 'function') {
            DeviceMotionEvent.requestPermission().then(permissionState => {
                if (permissionState === 'granted') {
                    window.addEventListener('devicemotion', imuMotionHandler);
                }
            }).catch(console.error);
        } else {
            window.addEventListener('devicemotion', imuMotionHandler);
        }
    } else {
         if ($('imu-status')) $('imu-status').textContent = "Non supporté";
    }
}

function stopIMUListeners() {
    if (window.DeviceMotionEvent) {
        window.removeEventListener('devicemotion', imuMotionHandler);
    }
    if ($('imu-status')) $('imu-status').textContent = "Inactif";
}

// --------------------------------------------------------------------------
// --- GESTION DES CARTES ---
// --------------------------------------------------------------------------

function initMap() {
    try {
        if ($('map') && typeof L !== 'undefined') { 
            map = L.map('map').setView([DEFAULT_INIT_LAT, DEFAULT_INIT_LON], 12);
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                attribution: '© OpenStreetMap contributors'
            }).addTo(map);
            marker = L.marker([DEFAULT_INIT_LAT, DEFAULT_INIT_LON]).addTo(map);
            circle = L.circle([DEFAULT_INIT_LAT, DEFAULT_INIT_LON], { color: 'red', fillColor: '#f03', fillOpacity: 0.5, radius: 10 }).addTo(map);
        }
    } catch (e) {
        console.error("Erreur d'initialisation de Leaflet (Carte):", e);
        if ($('map')) $('map').innerHTML = "Erreur d'initialisation de la carte.";
    }
}

function updateMap(lat, lon, acc) {
    if (map && marker) {
        marker.setLatLng([lat, lon]);
        circle.setLatLng([lat, lon]).setRadius(acc * R_FACTOR_RATIO); 
        const now = Date.now();
        if (now - lastMapUpdate > MAP_UPDATE_INTERVAL && getEKFVelocity3D() > MIN_SPD) {
            map.setView([lat, lon], map.getZoom() > 10 ? map.getZoom() : 16); 
            lastMapUpdate = now;
        } else if (map.getZoom() < 10 && (Date.now() - lastMapUpdate > 5000)) {
            map.setView([lat, lon], 12);
            lastMapUpdate = now;
        }
    }
}

// --------------------------------------------------------------------------
// --- FONCTIONS DE CONTRÔLE GPS & SYSTÈME ---
// --------------------------------------------------------------------------

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
    ['speed-stable', 'speed-3d-inst', 'distance-total-km', 'local-time'].forEach(id => {
        if ($(id)) $(id).textContent = 'ARRÊT D’URGENCE';
    });
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
    console.warn(`ERREUR GPS (${err.code}): ${err.message}`);
    if ($('gps-precision')) $('gps-precision').textContent = `Erreur: ${err.message}`;
    
    if (err.code === 1) { 
        stopGPS();
        alert("Accès à la géolocalisation refusé. Veuillez l'activer.");
    }
}

// --------------------------------------------------------------------------
// --- FONCTION PRINCIPALE DE MISE À JOUR GPS/EKF (updateDisp) ---
// --------------------------------------------------------------------------

function updateDisp(pos) {
    if (emergencyStopActive) return;

    // --- 1. ACQUISITION DES DONNÉES ET INITIALISATION ---
    const cTimePos = pos.timestamp;
    const now = getCDate(lServH, lLocH); 
    
    if (now === null) return; 
    if (sTime === null) sTime = now.getTime();
    
    let accRaw = pos.coords.accuracy;
    if (gpsAccuracyOverride > 0.0) accRaw = gpsAccuracyOverride;

    let dt = 0;
    if (lPos) {
        dt = (cTimePos - lPos.timestamp) / 1000;
    } else {
        // Première position: Initialisation de l'EKF
        lPos = pos; 
        initEKF(pos.coords.latitude, pos.coords.longitude, pos.coords.altitude || DEFAULT_INIT_ALT, accRaw);
        updateMap(currentEKFState.lat, currentEKFState.lon, currentEKFState.acc_est);
        return; 
    }
    
    if (dt < MIN_DT || dt > 10) { 
        lPos = pos; 
        return; 
    }
    
    // 2. PRÉDICTION EKF (Basée sur l'IMU et le temps)
    // acc_meas et gyro_meas devraient être lus des variables globales IMU
    predictEKF(dt, [real_accel_x, real_accel_y, real_accel_z], [0, 0, 0], G_ACC, R_ALT_CENTER_REF);
    
    // 3. LOGIQUE ZUPT
    const V_ekf = getEKFVelocity3D();
    const isPlausiblyStopped = (
        V_ekf < ZUPT_RAW_THRESHOLD && 
        Math.abs(lastAccelLong) < ZUPT_ACCEL_TOLERANCE
    ); 
    
    if (isPlausiblyStopped) { 
        updateEKF_ZUPT();
    }
    
    // 4. CORRECTION EKF (Fusion GNSS)
    const isSignalLost = (accRaw > MAX_ACC);
    currentEnvFactor = ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT;
    
    if (!isSignalLost) {
        const gnss_pos = { lat: pos.coords.latitude, lon: pos.coords.longitude, alt: pos.coords.altitude || currentEKFState.alt };
        const gnss_vel = { V_n: pos.coords.velocity, V_e: 0, V_d: 0 }; // Vitesse GNSS (si disponible)
        
        // La fonction de correction intègre R_pos et R_alt via l'accRaw/alt_acc
        updateEKF_GNSS(gnss_pos, gnss_vel, accRaw, pos.coords.altitudeAccuracy || 5.0);
    }
    
    // 5. MISE À JOUR DE L'ÉTAT LISIBLE ET CALCULS PHYSIQUES
    updateEKFReadableState(); // Met à jour currentEKFState
    const sSpdFE = V_ekf < MIN_SPD ? 0 : V_ekf; 
    
    let accel_long = 0;
    if (dt > 0.05) { 
        accel_long = (sSpdFE - lastFSpeed) / dt;
    }
    lastFSpeed = sSpdFE;
    lastAccelLong = accel_long;

    R_FACTOR_RATIO = calculateMRF(currentEKFState.alt, netherMode); 
    distM_3D += sSpdFE * dt * R_FACTOR_RATIO; 
    
    if (sSpdFE > MIN_SPD) { timeMoving += dt; }
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    
    const local_g = getGravityLocal(currentEKFState.alt, currentCelestialBody, rotationRadius, angularVelocity); 
    const kineticEnergy = 0.5 * currentMass * sSpdFE ** 2;
    const mechanicalPower = currentMass * sSpdFE * accel_long;
    const coriolis_force = 2 * currentMass * sSpdFE * OMEGA_EARTH * Math.sin(currentEKFState.lat * D2R);
    
    // Calculs Mach (Nécessite lastT_K de la météo)
    let mach_info = { C_S_local: C_S_BASE, mach: 0 };
    if (lastT_K !== null) {
        mach_info = calculateLocalSpeed(lastT_K - KELVIN_OFFSET, sSpdFE);
    }

    // 6. MISE À JOUR DU DOM (Affichage Rapide)
    const acc_est_m = currentEKFState.acc_est;
    const kalman_V_uncert = getVelocityUncertainty();
    const R_dyn = currentEnvFactor * accRaw ** 2; // Représentation simplifiée du R dynamique
    const altStatusTxt = currentEKFState.alt < ALT_TH ? `OUI (< ${ALT_TH}m)` : 'Non';
    const status_mode = isPlausiblyStopped ? '✅ ZUPT' : (isSignalLost ? '⚠️ EST. SEULE' : '🚀 FUSION');

    if ($('time-elapsed')) $('time-elapsed').textContent = `${((Date.now() - sTime) / 1000).toFixed(2)} s`;
    if ($('time-moving')) $('time-moving').textContent = `${timeMoving.toFixed(2)} s`;
    if ($('mode-ratio')) $('mode-ratio').textContent = `${R_FACTOR_RATIO.toFixed(3)} (Ratio)`;
    if ($('env-factor')) $('env-factor').textContent = `${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY} (x${currentEnvFactor.toFixed(1)})`;
    if ($('speed-stable')) $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(5)}`;
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${sSpdFE.toFixed(3)} m/s | ${(sSpdFE * 1e6).toFixed(0)} µm/s | ${(sSpdFE * 1e9).toFixed(0)} nm/s`;
    if ($('speed-max')) $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    if ($('distance-total-km')) $('distance-total-km').textContent = `${(distM_3D / 1000).toFixed(3)} km | ${distM_3D.toFixed(2)} m`;
    if ($('latitude')) $('latitude').textContent = `${currentEKFState.lat.toFixed(6)} °`;
    if ($('longitude')) $('longitude').textContent = `${currentEKFState.lon.toFixed(6)} °`;
    if ($('altitude-gps')) $('altitude-gps').textContent = currentEKFState.alt !== null ? `${currentEKFState.alt.toFixed(2)} m` : 'N/A';
    if ($('heading-display')) $('heading-display').textContent = pos.coords.heading !== null ? `${pos.coords.heading.toFixed(1)} °` : 'N/A';
    if ($('underground-status')) $('underground-status').textContent = `Souterrain: ${altStatusTxt} (${status_mode} | Acc GPS: ${accRaw.toFixed(1)}m)`;
    if ($('gravity-local')) $('gravity-local').textContent = `${local_g.toFixed(5)} m/s²`;
    if ($('accel-long')) $('accel-long').textContent = `${accel_long.toFixed(3)} m/s²`;
    if ($('imu-accel-x')) $('imu-accel-x').textContent = `${real_accel_x.toFixed(2)} m/s²`;
    if ($('kalman-uncert')) $('kalman-uncert').textContent = `${kalman_V_uncert.toFixed(3)} m²/s² (P)`;
    if ($('speed-error-perc')) $('speed-error-perc').textContent = `${R_dyn.toFixed(3)} m² (R dyn)`;
    if ($('gps-precision')) $('gps-precision').textContent = `${acc_est_m.toFixed(2)} m (Est.)`;
    if ($('coriolis-force')) $('coriolis-force').textContent = `${coriolis_force.toExponential(2)} N`;
    if ($('speed-sound-local')) $('speed-sound-local').textContent = `${mach_info.C_S_local.toFixed(2)} m/s`;
    if ($('mach-number')) $('mach-number').textContent = `${mach_info.mach.toFixed(4)}`;
    
    // 7. SAUVEGARDE & MISE À JOUR CARTE
    lPos = pos; 
    lPos.timestamp = cTimePos; 
    updateMap(currentEKFState.lat, currentEKFState.lon, acc_est_m);
}

// --------------------------------------------------------------------------
// --- FONCTIONS DOM VISUELLES ET LENTES ---
// --------------------------------------------------------------------------

function updateClockVisualization(now, sunPos, moonPos, sunTimes) {
    const sunEl = $('sun-element');
    const moonEl = $('moon-element');
    const clockEl = $('minecraft-clock'); 

    if (!sunEl || !moonEl || !clockEl || !sunPos || !moonPos || !sunTimes) return;

    // ... (Logique visuelle complète du Bloc 2/2 précédent)
    const sunIcon = sunEl.querySelector('.sun-icon');
    const moonIcon = moonEl.querySelector('.moon-icon');

    const altDegSun = sunPos.altitude * R2D;
    const aziDegSun = (sunPos.azimuth * R2D + 180) % 360; 
    sunEl.style.transform = `rotate(${aziDegSun}deg)`;
    const radialPercentSun = Math.min(50, Math.max(0, 50 * (90 - altDegSun) / 90));
    const altitudeOffsetPercentSun = 50 - radialPercentSun; 
    sunIcon.style.transform = `translateY(calc(-50% + ${altitudeOffsetPercentSun}%) )`; 
    sunEl.style.display = altDegSun > -0.83 ? 'flex' : 'none'; 

    const altDegMoon = moonPos.altitude * R2D;
    const aziDegMoon = (moonPos.azimuth * R2D + 180) % 360; 
    moonEl.style.transform = `rotate(${aziDegMoon}deg)`;
    const radialPercentMoon = Math.min(50, Math.max(0, 50 * (90 - altDegMoon) / 90));
    const altitudeOffsetPercentMoon = 50 - radialPercentMoon; 
    moonIcon.style.transform = `translateY(calc(-50% + ${altitudeOffsetPercentMoon}%) )`;
    moonEl.style.display = altDegMoon > 0 ? 'flex' : 'none';

    const body = document.body;
    body.classList.remove('sky-day', 'sky-sunset', 'sky-night', 'dark-mode', 'light-mode');
    clockEl.classList.remove('sky-day', 'sky-sunset', 'sky-night');

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

    $('clock-status').textContent = altDegSun > 0 ? 'Jour Solaire (☀️)' : 'Nuit/Crépuscule (🌙)';
}


// --------------------------------------------------------------------------
// --- INITIALISATION DES ÉVÉNEMENTS DOM ---
// --------------------------------------------------------------------------

document.addEventListener('DOMContentLoaded', () => {
    
    initMap(); 
    initEKF(DEFAULT_INIT_LAT, DEFAULT_INIT_LON, DEFAULT_INIT_ALT, 1000.0);

    // --- Initialisation des Contrôles ---
    const massInput = $('mass-input'); 
    if (massInput) {
        massInput.addEventListener('input', () => { 
            currentMass = parseFloat(massInput.value) || 70.0; 
            if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        });
        currentMass = parseFloat(massInput.value); 
        if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
    }

    if ($('celestial-body-select')) {
        $('celestial-body-select').addEventListener('change', (e) => { 
            const newVals = updateCelestialBody(e.target.value, currentEKFState.alt, rotationRadius, angularVelocity);
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
            const newVals = updateCelestialBody('ROTATING', currentEKFState.alt, rotationRadius, angularVelocity);
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
    if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', () => { 
        if (emergencyStopActive) { alert("Veuillez désactiver l'Arrêt d'urgence."); return; }
        wID === null ? startGPS() : stopGPS(); 
    });
    if ($('freq-select')) $('freq-select').addEventListener('change', (e) => setGPSMode(e.target.value));
    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', () => { 
        !emergencyStopActive ? emergencyStop() : resumeSystem(); 
    });
    if ($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        netherMode = !netherMode; 
        if ($('mode-nether')) $('mode-nether').textContent = netherMode ? `ACTIVÉ (1:${NETHER_RATIO}) 🔥` : "DÉSACTIVÉ (1:1)"; 
    });
    
    // --- Réinitialisations ---
    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        distM_3D = 0; timeMoving = 0; 
    });
        if (emergencyStopActive) return; 
        maxSpd = 0; 
    });
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        if (confirm("Réinitialiser toutes les données de session, y compris l'EKF ?")) { 
            distM_3D = 0; maxSpd = 0; timeMoving = 0; lPos = null; sTime = null;
            // Réinitialise l'EKF avec les valeurs par défaut
            initEKF(DEFAULT_INIT_LAT, DEFAULT_INIT_LON, DEFAULT_INIT_ALT, 1000.0);
        } 
    });
    
    if ($('toggle-mode-btn')) $('toggle-mode-btn').addEventListener('click', () => {
        document.body.classList.toggle('dark-mode');
    });
    
    // --- DÉMARRAGE DU SYSTÈME ---
    
    // Initialisation des valeurs de gravité/rayon en fonction du corps céleste sélectionné
    const initVals = updateCelestialBody(currentCelestialBody, currentEKFState.alt, rotationRadius, angularVelocity);
    G_ACC = initVals.G_ACC;
    R_ALT_CENTER_REF = initVals.R_ALT_CENTER_REF;
    
    // Synchronisation NTP initiale pour l'horloge
    syncH(lServH, lLocH).then(newTimes => {
        lServH = newTimes.lServH;
        lLocH = newTimes.lLocH;
        startGPS(); // Démarre le GPS une fois l'horloge synchronisée
    });

    // Boucle de mise à jour lente (Astro/Météo/Horloge)
    if (domID === null) {
        domID = setInterval(async () => {
            const currentLat = currentEKFState.lat; 
            const currentLon = currentEKFState.lon;
            
            // 1. Météo : Récupération et Affichage
            // (La fonction fetchWeather et ses variables sont supposées être définies dans BLOC 1/3)
            if (currentLat !== 0 && currentLon !== 0 && !emergencyStopActive) {
                const data = await fetchWeather(currentLat, currentLon);
                if (data) {
                    // Stocke les valeurs pour le filtre EKF
                    lastP_hPa = data.pressure_hPa;
                    lastT_K = data.tempK;
                    lastH_perc = data.humidity_perc / 100.0;
                    
                    // Met à jour le DOM météo
                    if ($('temp-air-2')) $('temp-air-2').textContent = `${data.tempC.toFixed(1)} °C`;
                    if ($('pressure-2')) $('pressure-2').textContent = `${data.pressure_hPa.toFixed(0)} hPa`;
                    if ($('humidity-2')) $('humidity-2').textContent = `${data.humidity_perc} %`;
                    if ($('air-density')) $('air-density').textContent = `${data.air_density.toFixed(3)} kg/m³`;
                    if ($('dew-point')) $('dew-point').textContent = `${data.dew_point.toFixed(1)} °C`;
                    if ($('weather-status')) $('weather-status').textContent = data.status; 
                }
            }
            
            // 2. Astro : Calculs et Affichage
            // (La fonction updateAstro est supposée être définie dans BLOC 2/3)
            const astroData = updateAstro(currentLat, currentLon, lServH, lLocH);

            if (astroData) {
                const { now, sunPos, moonIllum, moonPos, sunTimes, solarTimes } = astroData;

                if ($('time-minecraft')) $('time-minecraft').textContent = getMinecraftTime(now);
                if ($('date-display-astro')) $('date-display-astro').textContent = now.toLocaleDateString('fr-FR');
                if ($('tst')) $('tst').textContent = solarTimes.TST;
                if ($('mst')) $('mst').textContent = solarTimes.MST;
                if ($('eot')) $('eot').textContent = solarTimes.EOT + ' min'; 
                if ($('ecl-long')) $('ecl-long').textContent = solarTimes.ECL_LONG + ' °'; 
                if ($('sun-alt')) $('sun-alt').textContent = `${(sunPos.altitude * R2D).toFixed(2)} °`;
                if ($('sun-azimuth')) $('sun-azimuth').textContent = `${(sunPos.azimuth * R2D).toFixed(2)} ° (S-O)`;
                if ($('moon-alt')) $('moon-alt').textContent = `${(moonPos.altitude * R2D).toFixed(2)} °`;
                if ($('moon-azimuth')) $('moon-azimuth').textContent = `${(moonPos.azimuth * R2D).toFixed(2)} ° (S-O)`;
                if ($('moon-illuminated')) $('moon-illuminated').textContent = `${(moonIllum.fraction * 100).toFixed(1)} %`;
                if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(moonIllum.phase);
                if ($('noon-solar')) $('noon-solar').textContent = solarTimes.NoonSolar;
                
                updateClockVisualization(now, sunPos, moonPos, sunTimes);
            } else {
                $('clock-status').textContent = 'Astro (Attente GPS/SunCalc)...';
            }
            
            // 3. Horloge NTP : Resynchronisation toutes les minutes
            if (Math.floor(Date.now() / 1000) % 60 === 0) {
                 syncH(lServH, lLocH).then(newTimes => {
                    lServH = newTimes.lServH;
                    lLocH = newTimes.lLocH;
                 });
            }
            
            // 4. Horloge locale (NTP)
            const now = getCDate(lServH, lLocH);
            if (now) {
                if ($('local-time') && !$('local-time').textContent.includes('SYNCHRO ÉCHOUÉE')) {
                    $('local-time').textContent = now.toLocaleTimeString('fr-FR');
                }
                if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
            }
            
        }, DOM_SLOW_UPDATE_MS); 
    }
}); // Fermeture finale du document.addEventListener('DOMContentLoaded', ...)
