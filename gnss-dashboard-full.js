// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// BLOC 1/5 : Constantes, Utilitaires, et État Global
// CORRIGÉ : Fonctions métrologiques et offline-first
// =================================================================
// =================================================================
// MODIFICATIONS CRITIQUES DE L'ÉTAT GLOBAL (À PLACER EN HAUT DU FICHIER)
// =================================================================

// --- ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE ---
let isGpsPaused = false; // MODIFICATION 1 : La pause GPS est désactivée par défaut

let currentPosition = { 
    // MODIFICATION 2 : Initialisation avec des coordonnées de travail (ex: Marseille)
    // REMPLACER ces valeurs par votre emplacement si nécessaire
    lat: 43.2964,   // Latitude (ex: pour débloquer Astro/Météo)
    lon: 5.3697,    // Longitude
    acc: 10.0,      // Précision initiale (pour le filtre)
    spd: 0.0        // Vitesse initiale (pour le filtre)
};

// ... autres variables globales ...
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

    // --- CLÉS D'API & ENDPOINTS ---
    const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
    const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
    const PROXY_POLLUTANT_ENDPOINT = `${PROXY_BASE_URL}/api/pollutants`; 
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

    // --- CONSTANTES ATMOSPHÉRIQUES (ISA Standard) ---
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
    const R_ALT_MIN = 1.0;
    const MAX_PLAUSIBLE_ACCEL_GPS = 19.62; 
    const ZUPT_RAW_THRESHOLD = 1.0;     
    const ZUPT_ACCEL_THRESHOLD = 0.5;   
    const MIN_SPD = 0.01;        
    const MAX_ACC = 200;        
    const NETHER_RATIO = 8.0; // Ratio 1:8
    const ALT_TH = -50;         // Seuil d'altitude "Sous-sol"

    // --- CONFIGURATION SYSTÈME ---
    const MIN_DT = 0.01;        
    const MAP_UPDATE_INTERVAL = 3000;
    const IMU_UPDATE_RATE_MS = 20; // 50Hz
    const DOM_SLOW_UPDATE_MS = 1000; // 1Hz
    const WEATHER_FETCH_INTERVAL = 600000; // 10 minutes
    const STANDBY_TIMEOUT_MS = 300000; // 5 minutes
    const GPS_OPTS = {
        HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
        LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
    };

    // --- FACTEURS DE RÉACTIVITÉ UKF ---
    const UKF_REACTIVITY_FACTORS = {
        'AUTO': { MULT: 1.0, DISPLAY: 'Automatique' }, 'NORMAL': { MULT: 1.0, DISPLAY: 'Normal' },
        'FAST': { MULT: 0.2, DISPLAY: 'Rapide' }, 'STABLE': { MULT: 2.5, DISPLAY: 'Microscopique' },
    };

    // --- FACTEURS D'ENVIRONNEMENT ---
    const ENVIRONMENT_FACTORS = {
        'NORMAL': { MULT: 1.0, DISPLAY: 'Normal' }, 'FOREST': { MULT: 2.5, DISPLAY: 'Forêt' },
        'CONCRETE': { MULT: 7.0, DISPLAY: 'Grotte/Tunnel' }, 'METAL': { MULT: 5.0, DISPLAY: 'Métal/Bâtiment' },
    };

    // --- DONNÉES CÉLESTES/GRAVITÉ ---
    const CELESTIAL_DATA = {
        'EARTH': { G: 9.80665, R: WGS84_A, name: 'Terre' }, 'MOON': { G: 1.62, R: 1737400, name: 'Lune' },
        'MARS': { G: 3.71, R: 3389500, name: 'Mars' }, 'ROTATING': { G: 0.0, R: WGS84_A, name: 'Station Spatiale' }
    };

    // --- VARIABLES D'ÉTAT (Globales) ---
    let wID = null, domSlowID = null, domFastID = null, weatherFetchID = null; 
    let lat = null, lon = null, sTime = null; 
    let distM = 0, maxSpd = 0; 
    let kSpd = 0, kUncert = UKF_R_MAX, kAltUncert = 10, kAlt = 0; 
    let lastP_hPa = BARO_ALT_REF_HPA, lastT_K = TEMP_SEA_LEVEL_K, lastH_perc = 0;
    let currentAirDensity = RHO_SEA_LEVEL;
    let currentSpeedOfSound = 343.0; // Vitesse du son par défaut (m/s)
    let currentMass = 70.0; // Masse de l'objet (kg)
    let G_ACC = CELESTIAL_DATA.EARTH.G;
    let R_ALT_CENTER_REF = WGS84_A;
    let selectedEnvironment = 'NORMAL';
    let ukf_reactivity_factor = UKF_REACTIVITY_FACTORS.NORMAL.MULT;
    let currentRawGPSData = null;
    let lastRawGPSData = null;
    let lastGPSTime = null;
    let lastWeatherUpdate = 0;
    let gpsStandbyTimeoutID = null;
    let lastLocationForWeather = { lat: 0, lon: 0 };
    let lastKnownWeather = null;
    let lastKnownPollutants = null;
    let currentWakeLock = null; 

    // --- ÉTAT IMU ---
    let accel = { x: 0, y: 0, z: 0 };
    let gyro = { x: 0, y: 0, z: 0 };
    let orientation = { alpha: 0, beta: 0, gamma: 0 };
    let sensorsStarted = false;

    // --- ÉTAT DU FILTRE UKF ---
    let ukf = null; 
    
    // --- MAP (Leaflet) ---
    let map = null, marker = null, circle = null, trail = null;
    let trailPoints = [];
    let lastMapUpdate = 0;

    // --- HORLOGE & SYNCHRONISATION ---
    let lServH = null, lLocH = null; // Heure Serveur et Heure Locale pour le décalage
    const dayMs = 1000 * 60 * 60 * 24; 
    const hourMs = 1000 * 60 * 60;
   // =================================================================
// BLOC 2/5 : Fonctions de Calcul Physique, Métrologique et Astro
// =================================================================

// --- FONCTIONS MATHÉMATIQUES ET GÉODÉSIQUES ---

/** Calcule la distance de Haversine en mètres. */
const dist = (lat1, lon1, lat2, lon2, R_ref) => {
    const R = R_ref || R_ALT_CENTER_REF; 
    const dLat = (lat2 - lat1) * D2R, dLon = (lon2 - lon1) * D2R;
    lat1 *= D2R; lat2 *= D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R * c;
};

/** Calcule l'altitude barométrique selon la formule de l'atmosphère standard. */
function calculateAltitudeBarometric(P_hPa, P_ref_HPA, T_K) {
    // P_hPa: Pression actuelle (mesurée ou par défaut) en hPa
    // P_ref_HPA: Pression de référence (niveau de la mer) en hPa
    // T_K: Température actuelle (mesurée ou par défaut) en Kelvin
    if (P_hPa <= 0 || P_ref_HPA <= 0 || T_K <= 0) return NaN; // Évite les divisions par zéro/log de négatif

    // Formule standard d'altitude barométrique
    const pressure_ratio = P_hPa / P_ref_HPA;

    // Utilise G_ACC (gravité du corps céleste) et R_AIR (constante de l'air)
    const exponent = (G_ACC * 0.0065) / (R_AIR * 0.0065 - G_ACC * 0.0065); // Simplifié, 0.0065 est le lapse rate standard (K/m)
    const altitude = (T_K / 0.0065) * (Math.pow(pressure_ratio, (G_ACC * R_AIR) / (G_ACC * 0.0065)) - 1); 
    
    return altitude; 
}

/** Calcule la vitesse du son (m/s) en fonction de la température (K). */
function getSpeedOfSound(tempK) {
    // Vitesse du son dans l'air sec : c = sqrt(gamma * R_air * T)
    return Math.sqrt(GAMMA_AIR * R_AIR * tempK);
}

/** Calcule la densité de l'air (kg/m³) selon l'équation des gaz parfaits. */
function getAirDensity(P_Pa, T_K) {
    // P = rho * R_air * T -> rho = P / (R_air * T)
    const P_Pa = P_hPa * 100; // Conversion hPa en Pascal
    return P_Pa / (R_AIR * T_K);
}

/** Calcule la position, vitesse et accélération relativiste. */
function calculateRelativisticEffects(speed_ms, mass_kg, height_m) {
    // 1. Facteur de Lorentz (v/c)
    const v_c = speed_ms / C_L;
    const v_c2 = v_c * v_c;
    const lorentzFactor = 1 / Math.sqrt(1 - v_c2);
    
    // 2. Dilatation du Temps (Vitesse) : ns/jour (jours = 86400s)
    // Dilatation = (gamma - 1) * temps de repos
    const timeDilationSpeed = (lorentzFactor - 1) * 86400 * 1e9; 
    
    // 3. Énergie relativiste et masse au repos (E = m * c^2)
    const energyRestMass = mass_kg * C_L * C_L; // E_0
    const energyRelativistic = lorentzFactor * energyRestMass; // E_total

    // 4. Dilatation du Temps (Gravité) : ns/jour
    // g_potential = G * M_earth / R (avec R = R_base + alt)
    const G_CONST = G_U; // Constante de gravitation universelle
    const M_EARTH = 5.972e24; // Masse de la Terre (kg)
    const gravitationalPotential = -G_CONST * M_EARTH / (WGS84_A + height_m);
    
    // Formule simplifiée pour la dilatation gravitationnelle : dt/t ~ G*M/(R*c^2)
    // Facteur d'altitude/gravité (simplifié pour la Terre)
    const factor_g = gravitationalPotential / (C_L * C_L);
    const timeDilationGravity = -factor_g * 86400 * 1e9; // ns/jour
    
    return {
        v_c: v_c,
        lorentzFactor: lorentzFactor,
        timeDilationSpeed: timeDilationSpeed,
        timeDilationGravity: timeDilationGravity,
        energyRestMass: energyRestMass,
        energyRelativistic: energyRelativistic
    };
}

/** Calcule les forces non-gravitationnelles (traînée, Coriolis). */
function calculateExternalForces(V, mass, height_m, CdA = 0.5) {
    const P_Pa = lastP_hPa * 100;
    const T_K = lastT_K;
    const airDensity = currentAirDensity; // Utilise la densité calculée ou par défaut

    // 1. Traînée (Drag)
    const dynamicPressure = 0.5 * airDensity * V * V;
    const reynoldsNumber = (airDensity * V * 1) / MU_DYNAMIC_AIR; // Longueur de référence = 1m
    const dragForce = dynamicPressure * (CdA || 0.5); // CdA (Coefficient de traînée * Aire) par défaut 0.5
    const dragPower_kW = (dragForce * V) / 1000.0; // Puissance de la traînée (kW)

    // 2. Force de Coriolis (simplifiée) - F_c = 2 * m * v * Omega_Earth
    // Cette force est généralement faible et est mieux modélisée par l'UKF/EKF, mais peut être estimée
    const coriolisForce = 2 * mass * V * OMEGA_EARTH; 

    return {
        dragForce: dragForce,
        dragPower_kW: dragPower_kW,
        coriolisForce: coriolisForce,
        dynamicPressure: dynamicPressure,
        reynoldsNumber: reynoldsNumber
    };
}

// --- FONCTIONS ASTRO (SUNCALC & HEURES) ---

/** Synchronise l'horloge interne avec un serveur de temps (UTC/Atomique). */
async function syncH() {
    // Si l'heure serveur est déjà définie, on ne resynchronise pas souvent
    if (lServH && performance.now() - lServH < hourMs) return; 

    if ($('local-time')) $('local-time').textContent = 'Synchronisation...';
    const localStartPerformance = performance.now();

    try {
        const response = await fetchWithBackoff(SERVER_TIME_ENDPOINT);
        const serverTime = new Date(response.datetime); 
        const localEndPerformance = performance.now();
        const latency = (localEndPerformance - localStartPerformance) / 2; // Latence estimée

        lServH = serverTime.valueOf() + latency; // Temps serveur corrigé
        lLocH = Date.now(); // Temps local exact au moment de la réception

        console.log(`Synchro UTC réussie. Latence: ${latency.toFixed(1)}ms. Décalage: ${lServH - lLocH}ms`);

    } catch (err) {
        console.error("Échec de la synchro NTP:", err.message);
        lServH = null; 
        lLocH = null;
        if ($('local-time')) $('local-time').textContent = 'SYNCHRO ÉCHOUÉE (Local)';
    }
}

/** Retourne la date/heure corrigée. */
function getCDate(lServH_in, lLocH_in) {
    if (lServH_in === null || lLocH_in === null) {
        return new Date(); // Retourne l'heure locale non synchronisée
    }
    const currentLocalTime = Date.now();
    const correctedTime = lServH_in + (currentLocalTime - lLocH_in);
    return new Date(correctedTime);
}

/** Calcule les heures solaires (TST, MST, EOT, etc.). */
function calculateSolarTimes(now, lat, lon) {
    // Temps Solaire Vrai (True Solar Time - TST)
    const sunPos = SunCalc.getPosition(now, lat, lon);
    const sunTime = SunCalc.getTimes(now, lat, lon);
    const solarNoon = sunTime.solarNoon;
    
    // Équation du Temps (EOT)
    // EOT = TST - MST (Temps Solaire Moyen)
    const eot_min = solarNoon ? (solarNoon.getUTCHours() * 60 + solarNoon.getUTCMinutes() + solarNoon.getUTCSeconds() / 60) - (12 * 60) : NaN;

    // Calcul de TST et MST en ms
    const tst_ms = now.valueOf() + (sunPos.azimuth * R2D / 15 * hourMs); // Approximation TST (non canonique)
    const mst_ms = now.valueOf() + (lon / 15 * hourMs); // Approximation MST

    // Longitude écliptique du Soleil (L)
    const L = sunPos.altitude; // Approximation de L (à améliorer si besoin)

    // Fonction d'aide pour formater l'heure
    const toTimeString = (t) => {
        const date = new Date(t);
        const h = date.getUTCHours();
        const m = date.getUTCMinutes();
        const s = date.getUTCSeconds();
        return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
    };

    return { 
        TST: toTimeString(tst_ms), 
        MST: toTimeString(mst_ms), 
        EOT: eot_min.toFixed(2), 
        ECL_LONG: (L * R2D).toFixed(2)
    };
}

/** Retourne le TSLV (Temps Solaire Local Vrai) à l'instant t pour la longitude lon. */
function getTSLV(t, lon) {
    const date = new Date(t);
    const offset = date.getTimezoneOffset() * 60000;
    const utcTime = date.getTime() + offset;
    // TSLV = UTC + Longitude / 15°
    const TSLV_ms = utcTime + (lon / 15) * hourMs; 
    
    const tslv_date = new Date(TSLV_ms);
    const h = tslv_date.getHours();
    const m = tslv_date.getMinutes();
    const s = tslv_date.getSeconds();

    return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
}

// --- FONCTIONS MÉTÉO ET POLLUANTS ---

/** Fetch les données météo et polluants. */
async function fetchWeatherData(latA, lonA) {
    if (latA === null || lonA === null) return null;
    
    const now = performance.now();
    // Limite le fetch à 10 minutes (WEATHER_FETCH_INTERVAL)
    if (now - lastWeatherUpdate < WEATHER_FETCH_INTERVAL) {
        return lastKnownWeather;
    }

    lastWeatherUpdate = now;
    lastLocationForWeather = { lat: latA, lon: lonA };
    
    let weatherData = null;
    try {
        const response = await fetchWithBackoff(`${PROXY_WEATHER_ENDPOINT}?lat=${latA}&lon=${lonA}`);
        weatherData = {
            tempC: response.main.temp,
            tempK: response.main.temp + KELVIN_OFFSET,
            pressure_hPa: response.main.pressure,
            humidity_perc: response.main.humidity,
            wind_speed_ms: response.wind.speed,
            wind_deg: response.wind.deg,
            dew_point: response.dewPoint, // Calculé par le proxy
            air_density: response.airDensity // Calculé par le proxy
        };
        lastKnownWeather = weatherData;
    } catch (err) {
        console.warn("Erreur de récupération météo:", err.message);
    }
    return weatherData;
}

async function fetchPollutantData(latA, lonA) {
    if (latA === null || lonA === null) return null;

    try {
        const response = await fetchWithBackoff(`${PROXY_POLLUTANT_ENDPOINT}?lat=${latA}&lon=${lonA}`);
        lastKnownPollutants = {
            aqi: response.aqi,
            co: response.co,
            no2: response.no2,
            o3: response.o3,
            pm2_5: response.pm2_5
        };
        return lastKnownPollutants;
    } catch (err) {
        console.warn("Erreur de récupération polluants:", err.message);
    }
    return null;
 }
   // =================================================================
// BLOC 3/5 : Logique Principale UKF (Unscented Kalman Filter)
// =================================================================

/**
 * Implémentation simplifiée d'un UKF (Unscented Kalman Filter) 
 * à 21 états pour la fusion GPS/IMU (Position, Vitesse, Attitude, Biais).
 * * État (x - 21 dimensions):
 * x(0-2): Position (Lat, Lon, Alt)
 * x(3-5): Vitesse (vN, vE, vD - Nord, Est, Bas)
 * x(6-8): Attitude (Roll, Pitch, Yaw - RPY)
 * x(9-11): Biais Accéléromètre (Bias_a)
 * x(12-14): Biais Gyroscope (Bias_g)
 * x(15-17): Erreur de Position (e_p)
 * x(18-20): Erreur de Vitesse (e_v)
 */
class UKF_Engine {
    constructor(lat_init, lon_init, alt_init, acc_init, reactivityFactor) {
        // Initialisation de l'état (math.matrix)
        this.x = math.matrix(Array(UKF_STATE_DIM).fill(0).map((_, i) => [i < 3 ? (i === 0 ? lat_init * D2R : i === 1 ? lon_init * D2R : alt_init) : 0]));
        
        // Matrice de Covariance (P - Incertitude)
        this.P = math.diag(UKF_STATE_DIM).map((val, [i, j]) => {
            if (i < 2) return (acc_init / WGS84_A) ** 2 * 10; // Position (Lat/Lon)
            if (i === 2) return acc_init ** 2 * 10; // Altitude
            if (i >= 3 && i <= 5) return 0.5 ** 2; // Vitesse
            if (i >= 6 && i <= 8) return (5 * D2R) ** 2; // Attitude (5 deg init)
            if (i >= 9 && i <= 14) return 0.1 ** 2; // Biais (Acc/Gyro)
            return 0.01; // Autres erreurs
        });
        
        // Matrice de Bruit de Processus (Q) et de Mesure (R) - Doivent être définies dynamiquement
        this.Q_base = math.diag(UKF_STATE_DIM).map((val, [i, j]) => {
            if (i < 3) return 1e-4; // Pos
            if (i >= 3 && i <= 5) return 1e-3; // Vitesse
            if (i >= 6 && i <= 8) return 1e-5; // Attitude
            if (i >= 9 && i <= 14) return 1e-6; // Biais
            return 1e-4;
        });
        this.R_GPS_base = math.diag([acc_init ** 2, acc_init ** 2, 10 ** 2, 1 ** 2]); // R (Position(Lat, Lon, Alt), Vitesse)
        
        this.lastTime = performance.now() / 1000;
        this.reactivityFactor = reactivityFactor;
    }

    // --- Fonctions de transformation UKF (f et h) ---

    /** Fonction d'état (f): Modèle de propagation (Prédiction) */
    f(x_in, accel_corrige, gyro_corrige, dt) {
        const x_out = x_in.clone();

        // 1. Mise à jour de la Vitesse (x(3-5))
        // Simplifié : v_new = v_old + (accel - Biais_a - Gravité) * dt
        // Utilisation de la rotation (x(6-8)) pour projeter l'accélération
        const roll = x_in.get([6]), pitch = x_in.get([7]), yaw = x_in.get([8]);
        
        // Accélération corrigée (IMU dans le repère corps vers repère local de navigation)
        // Simplification : On assume acc_x = N, acc_y = E, acc_z = D dans ce modèle 21 états
        // (Modèle simplifié car la transformation de coordonnées est complexe sans matrice de rotation complète)
        
        const accel_local_n = accel_corrige.x;
        const accel_local_e = accel_corrige.y;
        const accel_local_d = accel_corrige.z - G_ACC; // Accélération vers le bas (Gravité soustraite)

        // Mise à jour de la vitesse
        x_out.set([3], x_in.get([3]) + accel_local_n * dt); // vN
        x_out.set([4], x_in.get([4]) + accel_local_e * dt); // vE
        x_out.set([5], x_in.get([5]) + accel_local_d * dt); // vD

        // Appliquer ZUPT (Zero Velocity Update) si la vitesse est très faible
        if (Math.abs(x_out.get([3])) < MIN_SPD) x_out.set([3], 0);
        if (Math.abs(x_out.get([4])) < MIN_SPD) x_out.set([4], 0);
        if (Math.abs(x_out.get([5])) < MIN_SPD) x_out.set([5], 0);

        // 2. Mise à jour de la Position (x(0-2))
        // Position new = Position old + Vitesse * dt
        const R_N = WGS84_A * (1 - WGS84_E2) / Math.pow(1 - WGS84_E2 * Math.sin(x_in.get([0])) ** 2, 1.5); // Rayon de courbure méridien (Latitude)
        const R_E = WGS84_A / Math.sqrt(1 - WGS84_E2 * Math.sin(x_in.get([0])) ** 2); // Rayon de courbure transverse (Longitude)
        
        const lat_pred = x_in.get([0]) + (x_out.get([3]) / (R_N + x_in.get([2]))) * dt; // Lat = Lat + vN / R_N * dt
        const lon_pred = x_in.get([1]) + (x_out.get([4]) / ((R_E + x_in.get([2])) * Math.cos(x_in.get([0])))) * dt; // Lon = Lon + vE / (R_E * cos(Lat)) * dt
        const alt_pred = x_in.get([2]) - x_out.get([5]) * dt; // Alt = Alt - vD * dt (vD est vers le bas)
        
        x_out.set([0], lat_pred);
        x_out.set([1], lon_pred);
        x_out.set([2], alt_pred);
        
        // 3. Mise à jour de l'Attitude (x(6-8))
        // Attitude new = Attitude old + Taux de rotation (Gyro corrigé) * dt
        const roll_pred = x_in.get([6]) + gyro_corrige.x * dt;
        const pitch_pred = x_in.get([7]) + gyro_corrige.y * dt;
        const yaw_pred = x_in.get([8]) + gyro_corrige.z * dt;
        
        x_out.set([6], roll_pred);
        x_out.set([7], pitch_pred);
        x_out.set([8], yaw_pred);
        
        // 4. Les Biais (x(9-14)) et Erreurs (x(15-20)) sont modélisés comme des random walks, donc ne changent pas lors de la prédiction sans un modèle de dynamique plus complexe.
        // x_out(9-20) = x_in(9-20);

        return x_out;
    }

    /** Fonction d'observation (h): Modèle de mesure (Lat, Lon, Alt, Spd) */
    h(x_in) {
        // Renvoie l'état sous forme de mesure attendue (Position + Vitesse 3D)
        const lat_rad = x_in.get([0]);
        const lon_rad = x_in.get([1]);
        const alt = x_in.get([2]);
        const vN = x_in.get([3]);
        const vE = x_in.get([4]);
        const vD = x_in.get([5]);
        
        // Vitesse 3D (Magnitude)
        const speed_3d = Math.sqrt(vN * vN + vE * vE + vD * vD);

        // La mesure attendue est [Lat (rad), Lon (rad), Alt (m), Speed_3D (m/s)]
        return math.matrix([[lat_rad], [lon_rad], [alt], [speed_3d]]);
    }

    /** Exécute le pas de prédiction et de mise à jour de l'UKF. */
    update(raw_gps, accel_raw, gyro_raw, baro_alt, dt) {
        // --- 1. PRÉDICTION (Propager l'état et la covariance) ---
        // Les fonctions de prédiction f(x, u, dt) et les étapes UKF (Sigma points, propagation, recombinaison) 
        // sont trop complexes pour être implémentées ici manuellement.
        // On utilise l'approximation : x_pred = f(x_prev, u, dt) et P_pred = F*P*F' + Q

        // a. Pré-traitement de l'entrée (u - Commandes/Mesures brutes)
        const dt_ukf = dt < MIN_DT ? MIN_DT : dt;
        
        // Correction de l'IMU par le Biais
        const accel_corrige = {
            x: accel_raw.x - this.x.get([9]),
            y: accel_raw.y - this.x.get([10]),
            z: accel_raw.z - this.x.get([11])
        };
        const gyro_corrige = {
            x: gyro_raw.x - this.x.get([12]),
            y: gyro_raw.y - this.x.get([13]),
            z: gyro_raw.z - this.x.get([14])
        };

        // Simplification du UKF par un EKF Linéarisé autour de x
        const F = math.identity(UKF_STATE_DIM); // Matrice de transition d'état F (simplifiée à identité pour petit dt)
        
        // Prédiction de l'état (utilisation de la fonction f simplifiée pour l'état)
        const x_pred = this.f(this.x, accel_corrige, gyro_corrige, dt_ukf);
        
        // Prédiction de la Covariance (P_pred = F * P * F' + Q)
        const Q_scaled = math.multiply(this.Q_base, dt_ukf * this.reactivityFactor);
        const P_pred = math.add(math.multiply(F, this.P, math.transpose(F)), Q_scaled); 
        
        this.x = x_pred;
        this.P = P_pred;

        // --- 2. MISE À JOUR (Correction par Mesure GPS) ---
        if (raw_gps) {
            // z_gps = Mesure GPS [Lat(rad), Lon(rad), Alt(m), Spd(m/s)]
            const z_gps = math.matrix([[raw_gps.lat * D2R], [raw_gps.lon * D2R], [raw_gps.alt], [raw_gps.spd]]);
            
            // h(x_pred) = Mesure attendue à partir de l'état prédit
            const h_x_pred = this.h(x_pred);

            // Innovation (y) = Mesure - Attendu
            const y = math.subtract(z_gps, h_x_pred);
            
            // Matrice Jacobienne d'Observation (H)
            const H = math.zeros(4, UKF_STATE_DIM); 
            H.set([0, 0], 1); // Lat
            H.set([1, 1], 1); // Lon
            H.set([2, 2], 1); // Alt
            
            // Vitesse (H[3, 3-5] dépend de la vitesse 3D)
            const vN = x_pred.get([3]), vE = x_pred.get([4]), vD = x_pred.get([5]);
            const speed_3d = Math.max(MIN_SPD, h_x_pred.get([3])); // Éviter la division par zéro
            H.set([3, 3], vN / speed_3d); 
            H.set([3, 4], vE / speed_3d); 
            H.set([3, 5], vD / speed_3d);

            // Matrice de Covariance de Mesure (R) - Dynamique
            let R_GPS_dynamic = this.R_GPS_base.clone();
            const raw_acc_rad = raw_gps.acc / WGS84_A; 
            R_GPS_dynamic.set([0, 0], Math.max(raw_acc_rad ** 2, 1e-6)); // Lat
            R_GPS_dynamic.set([1, 1], Math.max(raw_acc_rad ** 2, 1e-6)); // Lon
            R_GPS_dynamic.set([2, 2], Math.min(raw_gps.acc ** 2, UKF_R_MAX ** 2)); // Alt
            R_GPS_dynamic.set([3, 3], (raw_gps.spd_uncert || 1.0) ** 2); // Vitesse
            
            // Covariance d'innovation (S) = H * P_pred * H' + R
            const H_T = math.transpose(H);
            const S = math.add(math.multiply(H, P_pred, H_T), R_GPS_dynamic);
            
            // Gain de Kalman (K) = P_pred * H' * inv(S)
            const K = math.multiply(P_pred, H_T, math.inv(S));

            // Mise à jour de l'état (x_new = x_pred + K * y)
            this.x = math.add(x_pred, math.multiply(K, y));

            // Mise à jour de la Covariance (P_new = (I - K * H) * P_pred)
            const I = math.identity(UKF_STATE_DIM);
            this.P = math.multiply(math.subtract(I, math.multiply(K, H)), P_pred);
        }

        // --- 3. MISE À JOUR (Correction par Mesure Barométrique) ---
        if (!isNaN(baro_alt) && baro_alt !== null) {
            // L'altitude barométrique est la seule mesure. Elle n'affecte que l'état x(2) Alt.
            const z_baro = math.matrix([[baro_alt]]);
            const h_baro = math.matrix([[this.x.get([2])]]);
            const y_baro = math.subtract(z_baro, h_baro);
            
            const H_baro = math.zeros(1, UKF_STATE_DIM);
            H_baro.set([0, 2], 1); // La mesure est seulement l'altitude (x(2))

            const R_baro = math.matrix([[R_ALT_MIN ** 2]]); // Incertitude Baro (très faible)
            
            const H_baro_T = math.transpose(H_baro);
            const S_baro = math.add(math.multiply(H_baro, this.P, H_baro_T), R_baro);
            const K_baro = math.multiply(this.P, H_baro_T, math.inv(S_baro));

            this.x = math.add(this.x, math.multiply(K_baro, y_baro));
            this.P = math.multiply(math.subtract(math.identity(UKF_STATE_DIM), math.multiply(K_baro, H_baro)), this.P);
        }

        // Normaliser l'attitude (Roll/Pitch)
        this.x.set([6], this.x.get([6]) % (2 * Math.PI));
        this.x.set([7], this.x.get([7]) % (2 * Math.PI));

        // Mettre à jour l'heure du dernier update
        this.lastTime = performance.now() / 1000;
    }

    /** Retourne l'état estimé de manière lisible. */
    getState() {
        // Position
        const lat_rad = this.x.get([0]);
        const lon_rad = this.x.get([1]);
        const alt = this.x.get([2]);

        // Vitesse
        const vN = this.x.get([3]), vE = this.x.get([4]), vD = this.x.get([5]);
        const speed = Math.sqrt(vN * vN + vE * vE + vD * vD);

        // Attitude (convertie en degrés)
        const roll = this.x.get([6]) * R2D;
        const pitch = this.x.get([7]) * R2D;
        const yaw = this.x.get([8]) * R2D;
        
        // Incertitude (approximation: racine de la diagonale de P pour Pos/Alt/Spd)
        const kUncert = Math.sqrt(this.P.get([0, 0]) * WGS84_A * WGS84_A + this.P.get([1, 1]) * WGS84_A * WGS84_A);
        const kAltUncert = Math.sqrt(this.P.get([2, 2]));

        return {
            lat: lat_rad * R2D,
            lon: lon_rad * R2D,
            alt: alt,
            speed: speed,
            roll: roll,
            pitch: pitch,
            yaw: yaw,
            kUncert: kUncert,
            kAltUncert: kAltUncert,
            bias_accel: { x: this.x.get([9]), y: this.x.get([10]), z: this.x.get([11]) },
            bias_gyro: { x: this.x.get([12]), y: this.x.get([13]), z: this.x.get([14]) }
        };
    }
    
    /** Met à jour les facteurs d'environnement/réactivité. */
    updateFactors(env_factor, ukf_reactivity_mult) {
        // Adaptation de R et Q selon l'environnement et la réactivité
        this.reactivityFactor = ukf_reactivity_mult; 
        const env_mult = ENVIRONMENT_FACTORS[env_factor].MULT;
        
        // R est augmenté en fonction de l'environnement (moins fiable = plus d'incertitude)
        this.R_GPS_base = math.diag([WGS84_A ** 2 * env_mult, WGS84_A ** 2 * env_mult, 10 ** 2 * env_mult, 1 ** 2 * env_mult]);
    }
    }
   // =================================================================
// BLOC 4/5 : Acquisition GPS/IMU (Boucle Rapide)
// =================================================================

// --- FONCTIONS DE CONTRÔLE GPS/WAKELOCK ---

/** Tente d'obtenir un verrouillage de l'écran (WakeLock) pour maintenir l'application active. */
async function requestWakeLock() {
    if ('wakeLock' in navigator) {
        try {
            currentWakeLock = await navigator.wakeLock.request('screen');
            currentWakeLock.addEventListener('release', () => {
                console.log('Wake Lock libéré.');
            });
            console.log('Wake Lock acquis.');
        } catch (err) {
            console.warn(`Wake Lock Échec: ${err.name}, ${err.message}`);
        }
    }
}

/** Libère le verrouillage de l'écran. */
function releaseWakeLock() {
    if (currentWakeLock) {
        currentWakeLock.release()
            .then(() => {
                currentWakeLock = null;
                console.log('Wake Lock libéré avec succès.');
            });
    }
}

/** Lance l'acquisition GPS via navigator.geolocation.watchPosition(). */
function startGPS() {
    if (wID !== null) return; // Déjà lancé
    
    // Demander le WakeLock
    requestWakeLock();

    // Démarrer l'acquisition (Haute Fréquence)
    wID = navigator.geolocation.watchPosition(
        (position) => {
            // Succès : Récupérer les données brutes
            const { latitude, longitude, altitude, speed, accuracy } = position.coords;
            
            // Mettre à jour l'état global et alimenter le filtre UKF
            currentRawGPSData = { 
                lat: latitude, 
                lon: longitude, 
                alt: altitude, 
                spd: speed, 
                acc: accuracy,
                spd_uncert: position.coords.speedAccuracy || 1.0 // Incertitude vitesse
            };
            lastGPSTime = position.timestamp;
            
            // Réinitialiser le timeout de veille
            if (gpsStandbyTimeoutID) clearTimeout(gpsStandbyTimeoutID);
            gpsStandbyTimeoutID = setTimeout(() => {
                stopGPS(false); // Passer en mode "pause douce" après timeout
                console.log("GPS mis en veille après 5 minutes d'inactivité.");
            }, STANDBY_TIMEOUT_MS);

            // Mettre à jour la position globale pour l'Astro/Météo
            currentPosition.lat = latitude;
            currentPosition.lon = longitude;
            currentPosition.acc = accuracy;

            // Mise à jour de l'affichage rapide
            updateDisp(currentRawGPSData, position.timestamp);
        },
        (error) => {
            console.error(`Erreur GPS (${error.code}): ${error.message}`);
            // Gérer les erreurs (ex: permission refusée)
            if (error.code === 1) { // PERMISSION_DENIED
                alert("Erreur: L'accès à la géolocalisation est refusé.");
                stopGPS();
            }
        },
        GPS_OPTS.HIGH_FREQ // Options de haute précision
    );
    
    // Mise à jour de l'interface utilisateur
    if ($('toggle-gps-btn')) {
        $('toggle-gps-btn').textContent = "Pause (GPS ACTIF)";
        $('toggle-gps-btn').style.backgroundColor = '#28a745';
    }
    console.log("Acquisition GPS démarrée.");
}

/** Arrête l'acquisition GPS. */
function stopGPS(resetButton = true) {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    wID = null;
    
    if (gpsStandbyTimeoutID) clearTimeout(gpsStandbyTimeoutID);
    gpsStandbyTimeoutID = null;
    
    releaseWakeLock(); // Libérer le WakeLock

    if (resetButton && $('toggle-gps-btn')) {
        $('toggle-gps-btn').textContent = "Démarrer GPS";
        $('toggle-gps-btn').style.backgroundColor = '#007bff';
    } else if ($('toggle-gps-btn')) {
        $('toggle-gps-btn').textContent = "Reprendre GPS (Veille)";
        $('toggle-gps-btn').style.backgroundColor = '#ffc107';
    }
    console.log("Acquisition GPS arrêtée.");
}

// --- LOGIQUE IMU (ACCÉLÉROMÈTRE/GYROSCOPE) ---

/** Initialise et démarre la lecture des capteurs IMU. */
function startIMUSensors() {
    if (sensorsStarted) return;
    sensorsStarted = true; // Flag pour s'assurer que l'IMU répond

    try {
        if ($('imu-status')) $('imu-status').textContent = "Activation...";

        // 1. Accéléromètre (Haute Fréquence)
        if (typeof Accelerometer === 'undefined') throw new Error("API Accelerometer non supportée.");
        const accSensor = new Accelerometer({ frequency: 50 });
        accSensor.addEventListener('reading', () => {
            accel.x = accSensor.x;
            accel.y = accSensor.y;
            accel.z = accSensor.z;
        });
        accSensor.addEventListener('error', e => console.warn("Erreur Accéléromètre:", e.message));
        accSensor.start();

        // 2. Gyroscope (Haute Fréquence)
        if (typeof Gyroscope === 'undefined') throw new Error("API Gyroscope non supportée.");
        const gyrSensor = new Gyroscope({ frequency: 50 });
        gyrSensor.addEventListener('reading', () => {
            gyro.x = gyrSensor.x;
            gyro.y = gyrSensor.y;
            gyro.z = gyrSensor.z;
        });
        gyrSensor.addEventListener('error', e => console.warn("Erreur Gyroscope:", e.message));
        gyrSensor.start();

        // 3. Orientation (Pitch/Roll/Yaw)
        if (typeof AbsoluteOrientationSensor === 'undefined') throw new Error("API Orientation non supportée.");
        const orientSensor = new AbsoluteOrientationSensor({ frequency: 50 });
        orientSensor.addEventListener('reading', () => {
            // Conversion du quaternion en Euler (Simplification pour l'affichage)
            const q = orientSensor.quaternion;
            if (q) {
                // Conversion quaternion -> Roll/Pitch/Yaw (Simplification pour le dashboard)
                const sinr = 2.0 * (q[0] * q[1] + q[2] * q[3]);
                const cosr = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);
                orientation.roll = Math.atan2(sinr, cosr) * R2D;

                const sinp = 2.0 * (q[0] * q[2] - q[3] * q[1]);
                orientation.pitch = sinp >= 1 ? Math.PI / 2 * R2D : sinp <= -1 ? -Math.PI / 2 * R2D : Math.asin(sinp) * R2D;
                
                const siny = 2.0 * (q[0] * q[3] + q[1] * q[2]);
                const cosy = 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]);
                orientation.yaw = Math.atan2(siny, cosy) * R2D;
                orientation.alpha = (orientation.yaw + 360) % 360; // Azimut (0-360)
            }
        });
        orientSensor.addEventListener('error', e => console.warn("Erreur Orientation:", e.message));
        orientSensor.start();

        if ($('imu-status')) $('imu-status').textContent = "ACTIF (50Hz)";

        // Démarrage de la boucle rapide (UKF Update)
        domFastID = setInterval(() => {
            const now = performance.now();
            const dt = (now / 1000) - ukf.lastTime; // Delta time en secondes

            // 1. Déterminer l'altitude barométrique corrigée (si baromètre disponible)
            const alt_baro = calculateAltitudeBarometric(lastP_hPa, BARO_ALT_REF_HPA, lastT_K);
            
            // 2. Mettre à jour l'UKF (UKF_Engine)
            ukf.update(currentRawGPSData, accel, gyro, alt_baro, dt);
            
            // 3. Récupérer et afficher les états du filtre
            const estimatedState = ukf.getState();
            kAlt = estimatedState.alt;
            kSpd = estimatedState.speed;
            kUncert = estimatedState.kUncert;
            kAltUncert = estimatedState.kAltUncert;

            // Mise à jour du DOM (affichage rapide)
            const sSpdFE = kSpd < MIN_SPD ? 0 : kSpd;
            
            // Vitesse
            if ($('speed-stable-kmh')) $('speed-stable-kmh').textContent = `${(sSpdFE * KMH_MS).toFixed(3)} km/h`;
            if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${sSpdFE.toFixed(3)} m/s`;
            
            // Altitude
            if ($('alt-stable')) $('alt-stable').textContent = `${kAlt.toFixed(2)} m`;
            if ($('alt-uncert')) $('alt-uncert').textContent = `${kAltUncert.toFixed(2)} m`;
            
            // Incertitude position (horizontal)
            if ($('acc-stable')) $('acc-stable').textContent = `${kUncert.toFixed(2)} m`;

            // Attitude (IMU/Filtre)
            if ($('pitch')) $('pitch').textContent = `${estimatedState.pitch.toFixed(1)}°`;
            if ($('roll')) $('roll').textContent = `${estimatedState.roll.toFixed(1)}°`;

            // Dessin de la bulle de niveau (Pitch/Roll)
            const maxBubbleMove = 45; 
            const maxAngle = 45; // Max angle to display
            const bubbleX = Math.max(-maxBubbleMove, Math.min(maxBubbleMove, (estimatedState.roll / maxAngle) * maxBubbleMove));
            const bubbleY = Math.max(-maxBubbleMove, Math.min(maxBubbleMove, (estimatedState.pitch / maxAngle) * maxBubbleMove));
            if ($('bubble')) {
                $('bubble').style.transform = `translate(${bubbleX}px, ${bubbleY}px)`;
            }

            // Calculs physiques avancés (Relativité, Drag, etc.)
            const advancedPhysics = calculateRelativisticEffects(sSpdFE, currentMass, kAlt);
            
            // Affichage Relativité
            if ($('v-c-ratio')) $('v-c-ratio').textContent = dataOrDefault(advancedPhysics.v_c / C_L * 100, 2, ' %');
            if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(advancedPhysics.lorentzFactor, 8);
            if ($('time-dilation-v')) $('time-dilation-v').textContent = dataOrDefault(advancedPhysics.timeDilationSpeed, 3, ' ns/j');
            if ($('time-dilation-g')) $('time-dilation-g').textContent = dataOrDefault(advancedPhysics.gravitationalDilation, 3, ' ns/j');
            
            // Affichage Forces
            const externalForces = calculateExternalForces(sSpdFE, currentMass, kAlt);
            if ($('drag-force')) $('drag-force').textContent = `${externalForces.dragForce.toFixed(3)} N`;
            if ($('drag-power')) $('drag-power').textContent = `${externalForces.dragPower_kW.toFixed(3)} kW`;

        }, IMU_UPDATE_RATE_MS);
    } catch (e) {
        console.error("Erreur de démarrage IMU:", e.message);
        if ($('imu-status')) $('imu-status').textContent = "INACTIF (Non supporté)";
    }
}
   // =================================================================
// BLOC 5/5 : Boucle Lente (Astro/Météo) et Initialisation
// =================================================================

// --- Fonctions d'aide pour l'affichage Météo/Polluants (Hors ligne) ---
function updateWeatherDOM(data, isOffline = false) {
    const suffix = isOffline ? ' (Hors ligne)' : '';
    if ($('weather-status')) $('weather-status').textContent = `ACTIF ${suffix}`;
    
    if ($('temp-air-2')) $('temp-air-2').textContent = `${data.tempC.toFixed(1)} °C`;
    if ($('pressure-2')) $('pressure-2').textContent = `${data.pressure_hPa.toFixed(0)} hPa`;
    if ($('humidity-2')) $('humidity-2').textContent = `${data.humidity_perc} %`;
    if ($('air-density')) $('air-density').textContent = `${data.air_density.toFixed(3)} kg/m³`;
    if ($('dew-point')) $('dew-point').textContent = `${data.dew_point.toFixed(1)} °C`;

    if ($('wind-speed')) $('wind-speed').textContent = `${data.wind_speed_ms.toFixed(1)} m/s`;
    if ($('wind-direction')) $('wind-direction').textContent = `${data.wind_deg}° (${getWindDirectionName(data.wind_deg)})`;
}

function updatePollutantsDOM(data, isOffline = false) {
    if ($('pollutant-status')) $('pollutant-status').textContent = `ACTIF ${isOffline ? ' (Hors ligne)' : ''}`;
    if ($('aqi-value')) $('aqi-value').textContent = `${data.aqi} (${getAirQualityIndexName(data.aqi)})`;
    if ($('co-level')) $('co-level').textContent = `${data.co.toFixed(1)} µg/m³`;
    if ($('no2-level')) $('no2-level').textContent = `${data.no2.toFixed(1)} µg/m³`;
    if ($('o3-level')) $('o3-level').textContent = `${data.o3.toFixed(1)} µg/m³`;
    if ($('pm25-level')) $('pm25-level').textContent = `${data.pm2_5.toFixed(1)} µg/m³`;
}

// --- Fonctions utilitaires diverses ---

/** Retourne le nom de la direction du vent. */
function getWindDirectionName(deg) {
    const directions = ['N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE', 'S', 'SSO', 'SO', 'OSO', 'O', 'ONO', 'NO', 'NNO'];
    const index = Math.round((deg % 360) / (360 / directions.length)) % directions.length;
    return directions[index];
}

/** Retourne le nom de l'indice de qualité de l'air. */
function getAirQualityIndexName(aqi) {
    if (aqi === 1) return 'Bon';
    if (aqi === 2) return 'Passable';
    if (aqi === 3) return 'Modéré';
    if (aqi === 4) return 'Mauvais';
    if (aqi === 5) return 'Très Mauvais';
    return 'Inconnu';
}

/** Met à jour les constantes physiques globales lors du changement de corps céleste. */
function updateCelestialBody(bodyKey, alt, rotR, rotV) {
    const body = CELESTIAL_DATA[bodyKey];
    if (!body) return;
    
    if (bodyKey === 'ROTATING') {
        // Mode station spatiale : Gravité artificielle (centripète)
        const centripetal = rotR > 0 ? rotR * rotV * rotV : 0;
        G_ACC = centripetal;
    } else {
        // Gravité standard (basée sur l'altitude)
        const R_base = body.R;
        const g_base = body.G;
        // Formule de gravité standard g(h) = g_base * (R_base / (R_base + alt))^2
        G_ACC = g_base * (R_base / (R_base + alt)) ** 2;
    }

    R_ALT_CENTER_REF = body.R; // Mise à jour du rayon de référence pour les calculs de distance/horizon

    if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC.toFixed(4)} m/s²`;
}

// --- FONCTIONS ASTRO (SUNCALC) ---

/** Fonction principale de mise à jour Astro (appelée par la boucle lente) */
function updateAstro(latA, lonA, lServH, lLocH) {
    const now = getCDate(lServH, lLocH);
    if (now === null || latA === null || lonA === null) {
        if ($('clock-status')) $('clock-status').textContent = 'Position Solaire Indisponible';
        return;
    }
    
    const sunTimes = SunCalc.getTimes(now, latA, lonA);
    const solarTimes = calculateSolarTimes(now, latA, lonA);
    const sunPos = SunCalc.getPosition(now, latA, lonA);
    const moonPos = SunCalc.getMoonPosition(now, latA, lonA);
    const moonIllumination = SunCalc.getMoonIllumination(now);
    
    // 1. Statut Jour/Nuit
    if (now > sunTimes.sunset || now < sunTimes.sunrise) {
        if ($('clock-status')) $('clock-status').textContent = 'Nuit/Crépuscule (🌙)';
    } else {
        if ($('clock-status')) $('clock-status').textContent = 'Jour (☀️)';
    }

    // 2. Affichage Solaire
    if ($('mst')) $('mst').textContent = solarTimes.MST;
    if ($('tst')) $('tst').textContent = solarTimes.TST;
    if ($('noon-solar')) $('noon-solar').textContent = sunTimes.solarNoon ? sunTimes.solarNoon.toLocaleTimeString('fr-FR', { timeZone: 'UTC' }) : '...';
    if ($('eot')) $('eot').textContent = `${solarTimes.EOT} min`;
    if ($('tslv')) $('tslv').textContent = getTSLV(now, lonA);
    if ($('ecl-long')) $('ecl-long').textContent = `${solarTimes.ECL_LONG}°`;
    if ($('sun-alt')) $('sun-alt').textContent = `${(sunPos.altitude * R2D).toFixed(2)}°`;
    if ($('sun-azimuth')) $('sun-azimuth').textContent = `${(sunPos.azimuth * R2D + 180).toFixed(1)}°`;

    // 3. Affichage Lunaire
    if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(moonIllumination.phase);
    if ($('moon-illuminated')) $('moon-illuminated').textContent = `${(moonIllumination.fraction * 100).toFixed(1)} %`;
    if ($('moon-alt')) $('moon-alt').textContent = `${(moonPos.altitude * R2D).toFixed(2)}°`;
    if ($('moon-azimuth')) $('moon-azimuth').textContent = `${(moonPos.azimuth * R2D + 180).toFixed(1)}°`;
}

// --- FONCTIONS DE GESTION DU DOM ET INITIALISATION ---

/** Met à jour l'affichage rapide (DOM) avec les données GPS brutes. */
function updateDisp(raw_gps, timestamp) {
    if (raw_gps) {
        // Heure brute (Timestamp)
        if ($('time-raw')) $('time-raw').textContent = new Date(timestamp).toLocaleTimeString('fr-FR');
        
        // Position brute
        if ($('lat-raw')) $('lat-raw').textContent = raw_gps.lat.toFixed(6) + '°';
        if ($('lon-raw')) $('lon-raw').textContent = raw_gps.lon.toFixed(6) + '°';
        if ($('alt-raw')) $('alt-raw').textContent = `${raw_gps.alt ? raw_gps.alt.toFixed(2) : 'N/A'} m`;
        if ($('acc-raw')) $('acc-raw').textContent = `${raw_gps.acc.toFixed(2)} m`;
        
        // Vitesse brute
        if ($('speed-raw-kmh')) $('speed-raw-kmh').textContent = `${(raw_gps.spd * KMH_MS).toFixed(3)} km/h`;
        if ($('speed-raw-ms')) $('speed-raw-ms').textContent = `${raw_gps.spd.toFixed(3)} m/s`;
        
        // Mise à jour de la carte Leaflet
        if (map && marker && circle) {
            const latlng = [raw_gps.lat, raw_gps.lon];
            marker.setLatLng(latlng);
            circle.setLatLng(latlng).setRadius(raw_gps.acc);
            
            // Suivi de la trace uniquement si en mouvement
            if (raw_gps.spd > MIN_SPD || currentRawGPSData) {
                if (trailPoints.length === 0 || dist(trailPoints[trailPoints.length - 1][0], trailPoints[trailPoints.length - 1][1], raw_gps.lat, raw_gps.lon, R_ALT_CENTER_REF) > 1.0) {
                    trailPoints.push(latlng);
                    if (trailPoints.length > 500) trailPoints.shift(); // Limite la taille de la trace
                    trail.setLatLngs(trailPoints);
                }
            }
            
            // Recenter la carte seulement au début ou si l'UKF est loin
            if (map.getCenter().lat === 0 && map.getCenter().lng === 0 || kUncert > 100) {
                map.setView(latlng, 15);
            }
        }
    }
}

/** Initialise la carte Leaflet. */
function initMap() {
    if (typeof L !== 'undefined' && $('map')) {
        map = L.map('map').setView([currentPosition.lat, currentPosition.lon], 15);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: '© OpenStreetMap contributors'
        }).addTo(map);
        marker = L.marker([currentPosition.lat, currentPosition.lon]).addTo(map);
        circle = L.circle([currentPosition.lat, currentPosition.lon], { 
            color: 'red', 
            fillColor: '#f03', 
            fillOpacity: 0.1, 
            radius: currentPosition.acc 
        }).addTo(map);
        trail = L.polyline(trailPoints, { color: 'yellow', weight: 3, opacity: 0.7 }).addTo(map);
    }
}


/** Initialise la Boucle Lente (Horloge, Astro, Météo). */
function startSlowLoop() {
    domSlowID = setInterval(async () => {
        // 1. Synchronisation NTP/UTC
        await syncH();
        const now = getCDate(lServH, lLocH);

        // 2. Mise à jour Horloge
        if (now) {
            if ($('local-time') && !$('local-time').textContent.includes('SYNCHRO ÉCHOUÉE')) {
                $('local-time').textContent = now.toLocaleTimeString('fr-FR');
            }
            if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
        }

        // 3. Mise à jour Astro (utilise la position actuelle ou par défaut)
        const latA = currentPosition.lat;
        const lonA = currentPosition.lon;
        updateAstro(latA, lonA, lServH, lLocH);
        
        // 4. Météo et Polluants
        if (latA !== null && lonA !== null) {
            // Fetch Météo et Polluants (avec intervalle de 10 min)
            const weatherData = await fetchWeatherData(latA, lonA);
            const pollutantData = await fetchPollutantData(latA, lonA);

            if (weatherData) {
                updateWeatherDOM(weatherData);
                // Sauve les valeurs pour le filtre EKF (Altitude barométrique)
                lastP_hPa = weatherData.pressure_hPa;
                lastT_K = weatherData.tempK;
                lastH_perc = weatherData.humidity_perc / 100.0;
                currentAirDensity = weatherData.air_density;
                currentSpeedOfSound = getSpeedOfSound(lastT_K);
                if($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = `${currentSpeedOfSound.toFixed(2)} m/s`;
            }

            if (pollutantData) {
                updatePollutantsDOM(pollutantData);
            }
        }
        
        // 5. Mise à jour de la distance totale (en utilisant l'UKF pour la position)
        if (ukf) {
             const estimatedState = ukf.getState();
             const lastLat = lat;
             const lastLon = lon;
             lat = estimatedState.lat;
             lon = estimatedState.lon;
             
             if (lastLat !== null && lastLon !== null) {
                 // Calculer la distance parcourue depuis le dernier update lent
                 const d = dist(lastLat, lastLon, lat, lon, R_ALT_CENTER_REF);
                 distM += d;
             }
             
             // Affichage de la distance
             const distKm = distM / 1000.0;
             if ($('distance-total-km')) $('distance-total-km').textContent = `${distKm.toFixed(3)} km | ${distM.toFixed(0)} m`;

             // Mise à jour des données géophysiques
             updateCelestialBody('EARTH', estimatedState.alt, 0, 0); // Gravité locale corrigée
        }

    }, DOM_SLOW_UPDATE_MS); 
}

// --- INITIALISATION GLOBALE ---

document.addEventListener('DOMContentLoaded', () => {
    // 1. Initialiser la carte et le tableau de bord
    initMap(); 
    
    // 2. Initialiser le Filtre UKF avec la position par défaut
    ukf = new UKF_Engine(currentPosition.lat, currentPosition.lon, currentPosition.acc, currentPosition.acc, ukf_reactivity_factor);
    
    // 3. Initialiser les contrôles (Mass/Reactivity/Environment)
    const massInput = $('mass-input');
    if (massInput) {
        massInput.value = currentMass;
        massInput.addEventListener('input', () => {
            currentMass = parseFloat(massInput.value) || 70.0;
            if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        });
    }

    const reactivitySelect = $('reactivity-select');
    if (reactivitySelect) {
        Object.keys(UKF_REACTIVITY_FACTORS).forEach(key => {
            const option = document.createElement('option');
            option.value = key;
            option.textContent = UKF_REACTIVITY_FACTORS[key].DISPLAY;
            reactivitySelect.appendChild(option);
        });
        reactivitySelect.value = 'NORMAL';
        reactivitySelect.addEventListener('change', (e) => {
            const factor = UKF_REACTIVITY_FACTORS[e.target.value];
            ukf_reactivity_factor = factor.MULT;
            ukf.updateFactors(selectedEnvironment, ukf_reactivity_factor);
            if ($('ukf-reactivity')) $('ukf-reactivity').textContent = factor.DISPLAY;
        });
        if ($('ukf-reactivity')) $('ukf-reactivity').textContent = UKF_REACTIVITY_FACTORS.NORMAL.DISPLAY;
    }

    const envSelect = $('environment-select');
    if (envSelect) {
        Object.keys(ENVIRONMENT_FACTORS).forEach(key => {
            const option = document.createElement('option');
            option.value = key;
            option.textContent = ENVIRONMENT_FACTORS[key].DISPLAY;
            envSelect.appendChild(option);
        });
        envSelect.value = 'NORMAL';
        envSelect.addEventListener('change', (e) => {
            selectedEnvironment = e.target.value;
            ukf.updateFactors(selectedEnvironment, ukf_reactivity_factor);
            const factor = ENVIRONMENT_FACTORS[selectedEnvironment];
            if ($('env-factor')) $('env-factor').textContent = `${factor.DISPLAY} (x${factor.MULT.toFixed(1)})`;
        });
        const factor = ENVIRONMENT_FACTORS[selectedEnvironment];
        if ($('env-factor')) $('env-factor').textContent = `${factor.DISPLAY} (x${factor.MULT.toFixed(1)})`;
    }
    
    // 4. Initialiser les valeurs par défaut (Correction Métrologique)
    if (lastKnownWeather) {
        updateWeatherDOM(lastKnownWeather, true);
        lastP_hPa = lastKnownWeather.pressure_hPa;
        lastT_K = lastKnownWeather.tempK;
        currentAirDensity = lastKnownWeather.air_density;
        currentSpeedOfSound = getSpeedOfSound(lastT_K);
    } else {
        currentAirDensity = RHO_SEA_LEVEL;
        currentSpeedOfSound = getSpeedOfSound(TEMP_SEA_LEVEL_K); 
        lastT_K = TEMP_SEA_LEVEL_K;
        lastP_hPa = BARO_ALT_REF_HPA;
    }
    if (lastKnownPollutants) {
        updatePollutantsDOM(lastKnownPollutants, true);
    }
    
    // Mettre à jour les affichages par défaut
    if($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = `${currentSpeedOfSound.toFixed(2)} m/s (Défaut)`;
    if($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;

    // 5. Démarrer les boucles principales
    startIMUSensors(); 
    startSlowLoop();

    // 6. Gestion du bouton GPS
    $('toggle-gps-btn').addEventListener('click', () => {
        if (wID !== null) {
            stopGPS();
        } else {
            startGPS();
        }
    });

    // 7. Gestion du mode sombre
    $('toggle-mode-btn').addEventListener('click', () => {
        document.body.classList.toggle('dark-mode');
        if (document.body.classList.contains('dark-mode')) {
             $('toggle-mode-btn').innerHTML = '<i class="fas fa-sun"></i> Mode Jour';
        } else {
             $('toggle-mode-btn').innerHTML = '<i class="fas fa-moon"></i> Mode Nuit';
        }
    });
    
    // Initialiser le mode sombre
    if (window.matchMedia && window.matchMedia('(prefers-color-scheme: dark)').matches) {
        document.body.classList.add('dark-mode');
        $('toggle-mode-btn').innerHTML = '<i class="fas fa-sun"></i> Mode Jour';
    } else {
        $('toggle-mode-btn').innerHTML = '<i class="fas fa-moon"></i> Mode Nuit';
    }

    console.log('Dashboard initialisé (UKF 21 États).');
});

})(window);
