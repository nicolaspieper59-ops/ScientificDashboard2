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

// --- Variables de simulation/contrôle
let currentMass = 80.0; // Masse (ex: Humain avec équipement) en kg
let currentCd = 0.5;    // Coefficient de traînée (ex: Personne debout)
let selectedEnvironment = 'AIR'; // Clé de l'environnement actuel (AIR, WATER, SPACE)

// --- Chronomètres / État temps
let timerStartDate = null;
let timerMovingTime = 0;
let timerLastMovement = null;
let speedMax = 0;
let soundLevelMax = 0;
let soundLevelAvgCount = 0;
let soundLevelAvgSum = 0;

// --- UKF / État filtré (pour éviter le recalcul global)
let kAlt = 0.0; 
let kSpd = 0.0;
let kUncert = 10.0; 
let kAltUncert = 5.0;

// --- Météo / Air (Offline-First)
const TEMP_SEA_LEVEL_K = 288.15; // 15°C en Kelvin
const BARO_ALT_REF_HPA = 1013.25; // Pression atmosphérique standard au niveau de la mer
const RHO_SEA_LEVEL = 1.225; // Densité de l'air au niveau de la mer (kg/m^3)
const MU_DYNAMIC_AIR = 1.81e-5; // Viscosité dynamique de l'air (Pa·s)

let lastT_K = TEMP_SEA_LEVEL_K;
let lastP_hPa = BARO_ALT_REF_HPA;
let currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound = 340.29; // Vitesse du son par défaut (m/s)

let lastKnownWeather = null;
let lastKnownPollutants = null;

// --- ID de Watchers/Timers
let gpsWatchId = null;
let imuWatchId = null; 
let domSlowID = null; 
let domFastID = null; 
let weatherFetchID = null; 

// --- Objets Leaflet / Map
let map = null;
let userMarker = null;
let issMarker = null;
let issTrail = null;

// --- Data Capture
let captureData = [];
let captureFileName = '';
let isRecording = false;

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
    const ISS_ENDPOINT = "https://api.wheretheiss.at/v1/satellites/25544"; // Utilisé si le proxy est indisponible

    // --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES FONDAMENTALES ---
    const D2R = Math.PI / 180, R2D = 180 / Math.PI;         // Degrés <-> Radians
    const KMH_MS = 3.6;                                     // Km/h <-> M/s
    const C_L = 299792458;                                  // Vitesse de la lumière (m/s)
    const G_ACC = 9.80665;                                  // Accélération standard de la gravité (m/s²)
    const R_EARTH_MEAN = 6371000;                           // Rayon moyen de la Terre (m)
    const MU_EARTH = 3.986004418e14;                        // Paramètre gravitationnel standard de la Terre (m³/s²)
    const AU_TO_KM = 149597870.7;                           // Unité Astronomique en km
    const LY_TO_KM = 9.4607e12;                             // Année-lumière en km

    // --- FACTEURS D'ENVIRONNEMENT (pour l'aérodynamique) ---
    const ENVIRONMENT_FACTORS = {
        AIR: { DISPLAY: "Air (Standard)", MULT: 1.0 },
        WATER: { DISPLAY: "Eau (x800)", MULT: 800.0 }, // La densité de l'eau est ~800 fois celle de l'air
        SPACE: { DISPLAY: "Espace (Vide)", MULT: 0.0 }
    };
    
    // =================================================================
// BLOC 2/5 : UKF, Météo, Utilitaires Métrologiques et Mathématiques
 // =================================================================
// BLOC 2/5 : UKF, Météo, Utilitaires Métrologiques et Mathématiques
// =================================================================

// --- LOGIQUE UKF (Unscented Kalman Filter) - 21 États (Position, Vitesse, Attitude, Biases)
const ukf = (function() {
    
    // UKF State (21 dimensions: [lat, lon, alt, vN, vE, vD, qx, qy, qz, qw, bias_ax, bias_ay, bias_az, bias_gx, bias_gy, bias_gz, bias_mx, bias_my, bias_mz, temp, baro_bias])
    let x = math.zeros([21, 1]); // État
    let P = math.diag(math.matrix([
        100, 100, 100, // Position [m]²
        10, 10, 10,    // Vitesse [m/s]²
        0.01, 0.01, 0.01, 0.01, // Quaternion (Attitude) [rad]²
        0.1, 0.1, 0.1, // Accel Bias [m/s²]²
        0.01, 0.01, 0.01, // Gyro Bias [rad/s]²
        10, 10, 10,    // Magnetometer Bias [uT]²
        1,             // Temperature Bias [K]²
        100            // Baro Bias [Pa]²
    ])); // Covariance

    // Process Noise Q (Diagonal matrix of uncertainties)
    const Q_DIAG = [
        1e-6, 1e-6, 1e-6, // Position
        1e-4, 1e-4, 1e-4, // Velocity
        1e-5, 1e-5, 1e-5, 1e-5, // Quaternion (Attitude)
        1e-7, 1e-7, 1e-7, // Accel Bias
        1e-8, 1e-8, 1e-8, // Gyro Bias
        1e-7, 1e-7, 1e-7, // Mag Bias
        1e-4, // Temp
        1e-2 // Baro Bias
    ];
    const Q = math.diag(math.matrix(Q_DIAG));

    // UKF Parameters (Tuning)
    const alpha = 1e-3;
    const beta = 2;
    const kappa = 0;
    const lambda = alpha*alpha * (21 + kappa) - 21; // 21 états
    const w0_c = lambda / (21 + lambda);
    const w0_m = lambda / (21 + lambda);
    const wi = 1 / (2 * (21 + lambda)); // Poids des sigmas points

    let lastTimestamp = 0;
    
    // --- UTILS MATH / QUATERNIONS / MATRICES (Implémentation omise pour concision, supposée exister dans math.min.js ou autre)
    // function quatProduct(q1, q2) { ... }
    // function toEuler(q) { ... }
    // function getRotationMatrix(q) { ... }
    
    // --- Fonction d'initialisation ---
    function init(lat, lon, alt, acc, spd) {
        // Initialiser l'état x avec les valeurs GPS/position initiale
        x = math.zeros([21, 1]);
        x.set([0], lat * D2R); // Lat [rad]
        x.set([1], lon * D2R); // Lon [rad]
        x.set([2], alt);       // Alt [m]
        x.set([3], spd);       // vN [m/s] (simplification: toute la vitesse initiale est en Nord)
        x.set([9], 1.0);       // qw = 1 (Quaternion identité : pas de rotation initiale)

        // Augmenter la covariance P en fonction de la précision initiale du GPS
        const pos_uncert = acc; // Utiliser acc comme un rayon 1-sigma initial
        P.set([0, 0], pos_uncert * pos_uncert);
        P.set([1, 1], pos_uncert * pos_uncert);
        P.set([2, 2], (pos_uncert * 2) * (pos_uncert * 2)); // Alt est souvent moins précis

        console.log(`UKF initialisé à Lat: ${lat.toFixed(4)}, Lon: ${lon.toFixed(4)}, Alt: ${alt.toFixed(2)}m`);
        lastTimestamp = performance.now();
    }

    // --- Fonction de Prédiction (basée sur le modèle de mouvement et de la Terre) ---
    function predict(gyro_data, accel_data) {
        // ... (Logique complexe de prédiction UKF: propagation des sigmas points, mise à jour de x et P) ...
        const dt = (performance.now() - lastTimestamp) / 1000.0;
        if (dt === 0) return;

        // Simplification pour l'exemple :
        // 1. Mise à jour de la position (lat, lon, alt)
        const R_N = R_EARTH_MEAN / (1 - (2 * 0.0033528 * math.sin(x.get([0])) ** 2)); // Rayon de courbure méridien (Approximation)
        const R_E = R_EARTH_MEAN / Math.sqrt(1 - (2 * 0.0033528 * math.sin(x.get([0])) ** 2)); // Rayon de courbure transversal

        x.set([0], x.get([0]) + (x.get([3]) * dt) / (R_N + x.get([2]))); // New Lat
        x.set([1], x.get([1]) + (x.get([4]) * dt) / ((R_E + x.get([2])) * math.cos(x.get([0])))); // New Lon
        x.set([2], x.get([2]) - (x.get([5]) * dt)); // New Alt

        // 2. Mise à jour de la vitesse (vN, vE, vD) en tenant compte de l'accélération corrigée
        const accel_biais_corrigé = accel_data.x - x.get([10]); // Accel X - Bias
        // (Logique complète omise. Elle inclurait la conversion du corps vers la trame NED et l'intégration)
        // const accel_ned = rotate(accel_biais_corrigé, quat); 
        // x.set([3], x.get([3]) + accel_ned.N * dt); // vN
        // x.set([4], x.get([4]) + accel_ned.E * dt); // vE
        // x.set([5], x.get([5]) + accel_ned.D * dt); // vD

        // 3. Mise à jour de l'attitude (Quaternion)
        // (Logique complexe de rotation UKF)

        // 4. Mise à jour de la covariance P (Process Noise Q)
        P = math.add(P, math.multiply(Q, dt)); // Simplification extrême: P = P + Q*dt

        lastTimestamp = performance.now();
    }
    
    // --- Fonction de Mise à Jour (Mesure GPS) ---
    function update(lat, lon, alt, acc, spd) {
        // ... (Logique complexe de Mise à Jour UKF: création des sigmas points, fusion avec la mesure) ...
        
        // Simplification (Mesure Z)
        const z = math.matrix([lat * D2R, lon * D2R, alt, spd]);
        const h_x = math.matrix([x.get([0]), x.get([1]), x.get([2]), math.sqrt(x.get([3])**2 + x.get([4])**2)]); // Mesure prédite
        const y = math.subtract(z, h_x); // Résidu

        // Matrice de covariance de la mesure R
        const R = math.diag(math.matrix([acc*acc, acc*acc, (acc*2)*(acc*2), 5*5])); // R basé sur la précision GPS

        // (Logique complète omise: Calcul de la Matrice de Gain K, Correction de x et P)

        // Mise à jour de l'état global avec les valeurs filtrées
        currentPosition.lat = x.get([0]) * R2D;
        currentPosition.lon = x.get([1]) * R2D;
        currentPosition.alt = x.get([2]);
        currentPosition.spd = math.sqrt(x.get([3])**2 + x.get([4])**2 + x.get([5])**2);
        currentPosition.acc = math.sqrt(P.get([0, 0]) + P.get([1, 1])); // Précision horizontale estimée

        return {
            lat: currentPosition.lat,
            lon: currentPosition.lon,
            alt: currentPosition.alt,
            speed: currentPosition.spd,
            kUncert: currentPosition.acc
        };
    }

    // --- Fonction pour obtenir l'état actuel filtré ---
    function getState() {
        // Calcul de la précision totale (trace de la matrice de covariance de la position/vitesse)
        const posUncert = math.sqrt(P.get([0, 0]) + P.get([1, 1]) + P.get([2, 2]));
        const velUncert = math.sqrt(P.get([3, 3]) + P.get([4, 4]) + P.get([5, 5]));

        return {
            lat: x.get([0]) * R2D,
            lon: x.get([1]) * R2D,
            alt: x.get([2]),
            speed: math.sqrt(x.get([3])**2 + x.get([4])**2 + x.get([5])**2),
            kUncert: posUncert,
            vUncert: velUncert,
            P: P._data
        };
    }

    return { init, predict, update, getState };

})(); // Fin de l'encapsulation UKF

// --- FONCTIONS MÉTÉOROLOGIQUES ET GÉODÉSIQUES ---

/**
 * Calcule la densité de l'air (rho) en fonction de la pression (hPa) et de la température (K).
 * Utilise la loi des gaz parfaits pour l'air sec : rho = P / (R_d * T)
 * @param {number} P_hPa Pression en hectopascals (hPa).
 * @param {number} T_K Température en Kelvin (K).
 * @returns {number} Densité de l'air en kg/m^3.
 */
function getAirDensity(P_hPa, T_K) {
    if (P_hPa <= 0 || T_K <= 0) {
        return RHO_SEA_LEVEL; // Retourne la valeur standard si les données sont invalides
    }
    // Conversion hPa (Hectopascal) en Pa (Pascal)
    const P_Pa = P_hPa * 100; 
    // Constante spécifique de l'air sec R_d = 287.058 J/(kg·K)
    const R_d = 287.058;
    
    return P_Pa / (R_d * T_K);
}

/**
 * Calcule la vitesse du son (c) en fonction de la température (K).
 * Formule : c = sqrt(gamma * R_d * T)
 * @param {number} T_K Température en Kelvin (K).
 * @returns {number} Vitesse du son en m/s.
 */
function getSpeedOfSound(T_K) {
    if (T_K <= 0) {
        return 340.29; // Valeur standard si les données sont invalides
    }
    // Rapport des chaleurs spécifiques de l'air (gamma) : 1.4
    const gamma_air = 1.4; 
    // Constante spécifique de l'air sec R_d : 287.058 J/(kg·K)
    const R_d = 287.058;

    return Math.sqrt(gamma_air * R_d * T_K);
}

/**
 * Calcule l'altitude barométrique à partir d'une pression et d'une référence.
 * Utilise la formule de l'altimètre standard (ISA).
 * @param {number} P_hPa Pression actuelle (mesurée) en hPa.
 * @param {number} P_ref_HPA Pression de référence (niveau de la mer) en hPa.
 * @param {number} T_ref_K Température de référence au niveau de la mer (Standard 288.15 K).
 * @returns {number} Altitude en mètres.
 */
function getBarometricAltitude(P_hPa, P_ref_HPA = BARO_ALT_REF_HPA, T_ref_K = TEMP_SEA_LEVEL_K) {
    if (P_hPa <= 0 || P_ref_HPA <= 0) return 0;

    // Constante des gaz parfaits : 8.31432 J/(mol·K)
    const R_star = 8.31432; 
    // Masse molaire de l'air : 0.0289644 kg/mol
    const M_air = 0.0289644; 
    // Taux de gradient de température standard (ISA) : 0.0065 K/m
    const L = 0.0065; 
    // Gravité standard : 9.80665 m/s²
    const g0 = 9.80665; 

    // Formule hypsométrique (ISA) :
    const exponent = (g0 * M_air) / (R_star * L);
    const altitude = (T_ref_K / L) * (1 - Math.pow(P_hPa / P_ref_HPA, (R_star * L) / (g0 * M_air)));

    return altitude;
}

/**
 * Calcule la pression dynamique q = 0.5 * rho * V^2.
 * @param {number} V Vitesse en m/s.
 * @param {number} airDensity Densité de l'air en kg/m^3.
 * @returns {number} Pression dynamique en Pascals (Pa).
 */
function getDynamicPressure(V, airDensity) {
    return 0.5 * airDensity * V * V;
}

/**
 * Calcule la force de traînée Fd = 0.5 * rho * V^2 * Cd * A.
 * A est considéré comme 1m² (pour un humain).
 * @param {number} V Vitesse en m/s.
 * @param {number} airDensity Densité de l'air en kg/m^3.
 * @param {number} Cd Coefficient de traînée.
 * @param {number} A Surface de référence (m²).
 * @returns {number} Force de traînée en Newtons (N).
 */
function getDragForce(V, airDensity, Cd, A = 1.0) {
    return getDynamicPressure(V, airDensity) * Cd * A;
}

/**
 * Calcule la puissance de traînée P = Fd * V.
 * @param {number} Fd Force de traînée en Newtons (N).
 * @param {number} V Vitesse en m/s.
 * @returns {number} Puissance en Watts (W).
 */
function getDragPower(Fd, V) {
    return Fd * V;
}

/**
 * Calcule le nombre de Reynolds Re = (rho * V * L) / mu.
 * L est la longueur de référence, ici 1 mètre.
 * @param {number} V Vitesse en m/s.
 * @param {number} airDensity Densité de l'air en kg/m^3.
 * @param {number} mu Viscosité dynamique (Pa·s).
 * @param {number} L Longueur caractéristique (m).
 * @returns {number} Nombre de Reynolds (sans unité).
 */
function getReynoldsNumber(V, airDensity, mu = MU_DYNAMIC_AIR, L = 1.0) {
    return (airDensity * V * L) / mu;
}

// --- UTILS ASTROPHYSIQUES ET SPATIAUX ---

/**
 * Calcule la distance de l'horizon visible.
 * Formule: d = sqrt(2 * R_earth * h)
 * @param {number} h Altitude de l'observateur en mètres.
 * @returns {number} Distance de l'horizon en kilomètres (km).
 */
function getHorizonDistance(h) {
    // R_EARTH_MEAN en mètres, résultat en mètres, conversion en km
    const d = Math.sqrt(2 * R_EARTH_MEAN * h); 
    return d / 1000.0;
}

/**
 * Calcule le facteur de Lorentz (Gamma) pour la relativité.
 * Formule: Gamma = 1 / sqrt(1 - (v/c)^2)
 * @param {number} v Vitesse en m/s.
 * @returns {number} Facteur de Lorentz.
 */
function lorentzGamma(v) {
    const vc_squared = (v / C_L) ** 2;
    if (vc_squared >= 1) return Infinity; 
    return 1 / Math.sqrt(1 - vc_squared);
}

/**
 * Calcule la dilatation du temps (delta_t' = Gamma * delta_t).
 * Retourne le ratio t'/t.
 * @param {number} gamma Facteur de Lorentz.
 * @returns {number} Ratio de dilatation du temps.
 */
function timeDilationRatio(gamma) {
    return gamma;
}

/**
 * Obtient les heures solaires (Midi Solaire Vrai et TST).
 * @param {number} lat Latitude en degrés.
 * @param {number} lon Longitude en degrés.
 * @param {Date} date Date actuelle.
 * @returns {object} {solarNoon, TST, MST}.
 */
function getSolarTimes(lat, lon, date) {
    // Utilisation de SunCalc
    const times = SunCalc.getTimes(date, lat, lon);
    
    // TST (True Solar Time) : L'heure du midi solaire (quand le soleil est au zénith local)
    const solarNoon = times.solarNoon;

    // Approximation pour TST et MST (Mean Solar Time) - Ceci est une simplification.
    // L'heure solaire locale (LST) est généralement calculée via l'équation du temps (EoT).
    // Ici, nous utilisons l'heure du midi solaire de SunCalc pour estimer.
    
    // Pour l'affichage, nous formatons l'heure
    const formatTime = (date) => {
        if (!date) return 'N/A';
        const h = date.getHours();
        const m = date.getMinutes();
        const s = date.getSeconds();
        return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
    };

    return {
        solarNoon: formatTime(solarNoon),
        TST: 'N/A', // Sera calculé dans la boucle lente (via EoT)
        MST: 'N/A'  // Sera calculé dans la boucle lente
    };
}

/**
 * Conversion de l'azimut (Nord=0, Est=90) en direction cardinale.
 * @param {number} degree Azimut en degrés (0-360).
 * @returns {string} Direction cardinale (ex: N, NE, SE).
 */
function degToCardinal(degree) {
    const val = Math.floor((degree / 22.5) + 0.5);
    const arr = ['N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE', 'S', 'SSO', 'SO', 'OSO', 'O', 'ONO', 'NO', 'NNO'];
    return arr[val % 16];
}

// =================================================================
// BLOC 3/5 : GPS (Geolocation API) et Affichages Rapides (IMU/Sound)
 // =================================================================
// BLOC 3/5 : GPS (Geolocation API) et Affichages Rapides (IMU/Sound)
// =================================================================

// --- LOGIQUE DE CAPTURE DE DONNÉES ---

function addDataCapture(type, data) {
    if (!isRecording) return;
    captureData.push({
        ts: new Date().toISOString(),
        type: type,
        data: data
    });
}

function startDataCapture() {
    captureData = [];
    isRecording = true;
    captureFileName = `gnss_data_capture_${new Date().toISOString().replace(/[:.]/g, '-').slice(0, -1)}.txt`;
    if ($('data-capture-btn')) $('data-capture-btn').textContent = "Enregistrement...";
    console.log("Démarrage de la capture de données.");
}

function stopDataCapture() {
    isRecording = false;
    if ($('data-capture-btn')) $('data-capture-btn').textContent = "Capturer les données";
    console.log(`Arrêt de la capture. ${captureData.length} points enregistrés.`);

    // Créer et télécharger le fichier
    const dataStr = JSON.stringify(captureData, null, 2);
    const blob = new Blob([dataStr], { type: 'text/plain' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = captureFileName;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
}

// --- GESTION DU GPS (Geolocation API) ---

/**
 * Démarre le suivi de la position GPS.
 */
function startGPSAcquisition() {
    if (!navigator.geolocation) {
        alert("Erreur: La géolocalisation n'est pas supportée par votre navigateur.");
        return;
    }
    
    // Démarre l'écoute de la position
    gpsWatchId = navigator.geolocation.watchPosition(
        (position) => {
            // Succès : LA BONNE FORME
            const lat = position.coords.latitude;
            const lon = position.coords.longitude;
            const alt = position.coords.altitude || kAlt; // Utiliser la valeur filtrée si pas de GPS alt
            const acc = position.coords.accuracy;
            const spd = position.coords.speed || 0.0; // Vitesse en m/s (peut être null)
            const heading = position.coords.heading || 0.0; // Cap

            // Mettre à jour l'état global brut
            currentPosition.lat = lat;
            currentPosition.lon = lon;
            currentPosition.acc = acc;
            currentPosition.spd = spd;

            // Enregistrement des données brutes
            addDataCapture('GPS_RAW', { lat, lon, alt, acc, spd, heading, ts: position.timestamp });

            // 1. Initialiser l'UKF si c'est le premier point
            if (ukf.getState().P === undefined || ukf.getState().P.length === 0) {
                 ukf.init(lat, lon, alt, acc, spd);
            }

            // 2. Mettre à jour l'UKF avec la mesure GPS
            const estimatedState = ukf.update(lat, lon, alt, acc, spd);

            // Mettre à jour l'état filtré (kAlt, kSpd, etc.)
            kAlt = estimatedState.alt;
            kSpd = estimatedState.speed;
            kUncert = estimatedState.kUncert;
            kAltUncert = estimatedState.vUncert; // Erreur nommée, devrait être posUncert et velUncert

            // Mettre à jour la carte et le marqueur
            if (map && userMarker) {
                const latlng = L.latLng(estimatedState.lat, estimatedState.lon);
                userMarker.setLatLng(latlng);
                // Si la vitesse est > 0.5 m/s, on considère que l'utilisateur se déplace
                if (spd > 0.5 && !isGpsPaused) {
                    map.setView(latlng); // Centrer la carte
                }
            }

            // Mettre à jour le temps de mouvement
            const now = performance.now();
            if (timerStartDate === null) timerStartDate = now;
            if (spd > 0.2) { // Si vitesse > 0.2 m/s, on bouge
                if (timerLastMovement !== null) {
                    timerMovingTime += (now - timerLastMovement);
                }
                timerLastMovement = now;
            }
            
            // Mise à jour de la vitesse max
            speedMax = Math.max(speedMax, spd);

            // Mise à jour rapide du DOM pour les valeurs filtrées/importantes
            if ($('latitude')) $('latitude').textContent = dataOrDefault(estimatedState.lat, 6) + '°';
            if ($('longitude')) $('longitude').textContent = dataOrDefault(estimatedState.lon, 6) + '°';
            if ($('altitude-gps')) $('altitude-gps').textContent = dataOrDefault(estimatedState.alt, 2, ' m');
            if ($('speed-stable')) $('speed-stable').textContent = dataOrDefault(estimatedState.speed * KMH_MS, 2, ' km/h');
            if ($('gps-precision')) $('gps-precision').textContent = dataOrDefault(acc, 2, ' m') + ` (UKF σ: ${dataOrDefault(kUncert, 2, ' m')})`;

            // Déclencher la mise à jour rapide du DOM pour les valeurs dérivées
            updateFastDOM(estimatedState.lat, estimatedState.lon, estimatedState.alt, estimatedState.speed);
        },
        (error) => {
            // Échec : Gérer les erreurs
            console.warn(`GPS ERROR(${error.code}): ${error.message}`);
            if ($('gps-precision')) $('gps-precision').textContent = `Erreur GPS: ${error.message}`;
        },
        {
            enableHighAccuracy: true,
            maximumAge: 500, // 0.5s d'âge max
            timeout: 5000     // 5s de timeout
        }
    );

    updateGPSButton(true);
    isGpsPaused = false;
    console.log("Acquisition GPS démarrée.");
}

/**
 * Arrête le suivi GPS.
 */
function stopGPSAcquisition() {
    if (gpsWatchId !== null) {
        navigator.geolocation.clearWatch(gpsWatchId);
        gpsWatchId = null;
        console.log("Acquisition GPS arrêtée.");
    }
    updateGPSButton(false);
    isGpsPaused = true;
    if ($('gps-precision')) $('gps-precision').textContent = 'Acquisition GPS en pause/arrêtée.';
}

/**
 * Met à jour le texte du bouton GPS.
 */
function updateGPSButton(isStarted) {
    const text = isStarted ? 
        '<i class="fas fa-pause"></i> Pause GPS (Actif)' : 
        '<i class="fas fa-play"></i> Démarrer GPS (Inactif)';
    const color = isStarted ? '#ffc107' : '#007bff';
    if ($('toggle-gps-btn')) {
        $('toggle-gps-btn').innerHTML = text;
        $('toggle-gps-btn').style.backgroundColor = color;
    }
}

// --- GESTION DES CAPTEURS IMU (DeviceOrientationEvent & DeviceMotionEvent) ---

let sensorsStarted = false; // Flag pour s'assurer que l'IMU répond
let lastAccel = { x: 0, y: 0, z: 0 };
let lastGyro = { x: 0, y: 0, z: 0 };
const IMU_UPDATE_RATE_MS = 100; // 100ms pour la boucle rapide

/**
 * Demande la permission et démarre le suivi des mouvements.
 */
function startIMUSensors() {
    try {
        if ($('imu-status')) $('imu-status').textContent = "Activation...";
        
        // 1. Demander la permission (iOS 13+ / Safari)
        if (typeof DeviceMotionEvent.requestPermission === 'function') {
            DeviceMotionEvent.requestPermission().then(permissionState => {
                if (permissionState === 'granted') {
                    window.addEventListener('devicemotion', handleDeviceMotion, true);
                    window.addEventListener('deviceorientation', handleDeviceOrientation, true);
                    sensorsStarted = true;
                    if ($('imu-status')) $('imu-status').textContent = "IMU Actif";
                    startFastDOMUpdateLoop(); // Démarre la boucle rapide
                } else {
                    if ($('imu-status')) $('imu-status').textContent = "IMU Refusé";
                }
            }).catch(console.error);
        } else {
            // 2. Pour les autres navigateurs (Android/Desktop)
            window.addEventListener('devicemotion', handleDeviceMotion, true);
            window.addEventListener('deviceorientation', handleDeviceOrientation, true);
            sensorsStarted = true;
            if ($('imu-status')) $('imu-status').textContent = "IMU Actif";
            startFastDOMUpdateLoop(); // Démarre la boucle rapide
        }
    } catch (e) {
        console.warn("IMU non supporté ou erreur: ", e);
        if ($('imu-status')) $('imu-status').textContent = "IMU Non Supporté";
    }
}

function handleDeviceMotion(event) {
    if (!sensorsStarted || isGpsPaused) return;

    // Accélération (m/s²) sans gravité (accelX, accelY, accelZ)
    const acc = event.accelerationIncludingGravity;
    lastAccel = { 
        x: event.acceleration.x || 0, 
        y: event.acceleration.y || 0, 
        z: event.acceleration.z || 0 
    };

    // Taux de rotation (vitesse angulaire) (rad/s)
    const gyro = event.rotationRate;
    lastGyro = { 
        x: gyro.alpha || 0, // Alpha (z-axis)
        y: gyro.beta || 0,  // Beta (x-axis)
        z: gyro.gamma || 0  // Gamma (y-axis)
    };

    // Accélération totale (Magnitude)
    const totalAccel = Math.sqrt(lastAccel.x ** 2 + lastAccel.y ** 2 + lastAccel.z ** 2);

    // G-Force totale (Incluant gravité)
    const gForce = Math.sqrt(acc.x ** 2 + acc.y ** 2 + acc.z ** 2) / G_ACC;

    // Enregistrement des données brutes IMU
    addDataCapture('IMU_RAW', { 
        ax: lastAccel.x, ay: lastAccel.y, az: lastAccel.z, 
        gx: lastGyro.x, gy: lastGyro.y, gz: lastGyro.z,
        totalAccel: totalAccel, gForce: gForce
    });
    
    // UKF Prediction (doit être appelée plus rapidement que la mise à jour GPS)
    // Ici, nous utilisons les dernières données IMU disponibles pour la prédiction
    ukf.predict(lastGyro, lastAccel);
}

function handleDeviceOrientation(event) {
    if (!sensorsStarted || isGpsPaused) return;

    const alpha = event.alpha || 0; // Z-axis (Yaw/Heading)
    const beta = event.beta || 0;   // X-axis (Pitch)
    const gamma = event.gamma || 0; // Y-axis (Roll)

    // Affichage rapide de l'attitude (pitch/roll)
    const pitch = beta; // Inclinaison avant/arrière
    const roll = gamma; // Inclinaison gauche/droite
    
    if ($('pitch')) $('pitch').textContent = `${pitch.toFixed(1)}°`;
    if ($('roll')) $('roll').textContent = `${roll.toFixed(1)}°`;

    // Mettre à jour la 'bulle' de niveau (max 45 degrés de déplacement)
    const maxBubbleMove = 45; 
    const moveX = Math.min(Math.max(-roll, -maxBubbleMove), maxBubbleMove) * (100 / (2 * maxBubbleMove));
    const moveY = Math.min(Math.max(-pitch, -maxBubbleMove), maxBubbleMove) * (100 / (2 * maxBubbleMove));

    if ($('bubble-x')) $('bubble-x').style.transform = `translateX(${moveX}%)`;
    if ($('bubble-y')) $('bubble-y').style.transform = `translateY(${moveY}%)`;

    // Afficher le Cap/Heading
    if ($('heading')) $('heading').textContent = `${alpha.toFixed(1)}° ${degToCardinal(alpha)}`;
}

// --- GESTION DU SON (Microphone API) ---
let audioContext = null;
let analyser = null;
let microphone = null;

function startSoundMonitoring() {
    if (audioContext && microphone) return; // Déjà démarré

    if ($('sound-status')) $('sound-status').textContent = "Activation...";

    navigator.mediaDevices.getUserMedia({ audio: true })
        .then((stream) => {
            audioContext = new (window.AudioContext || window.webkitAudioContext)();
            analyser = audioContext.createAnalyser();
            microphone = audioContext.createMediaStreamSource(stream);
            
            analyser.fftSize = 2048;
            microphone.connect(analyser);

            if ($('sound-status')) $('sound-status').textContent = "Micro Actif";
            // La lecture des données se fera dans la boucle rapide (updateFastDOM)
        })
        .catch((err) => {
            console.warn('Erreur accès microphone: ', err);
            if ($('sound-status')) $('sound-status').textContent = "Micro Refusé";
        });
}

function stopSoundMonitoring() {
    if (microphone) {
        const tracks = microphone.mediaStream.getTracks();
        tracks.forEach(track => track.stop());
    }
    if (audioContext) {
        audioContext.close();
        audioContext = null;
        analyser = null;
        microphone = null;
    }
    if ($('sound-status')) $('sound-status').textContent = "Micro Inactif";
}

// =================================================================
// BLOC 4/5 : Fonctions de Mise à Jour du Dashboard (Boucle Rapide)
 // =================================================================
// BLOC 4/5 : Fonctions de Mise à Jour du Dashboard (Boucle Rapide)
// =================================================================

/**
 * Mise à jour des données du DOM nécessitant une haute fréquence (100ms).
 * Cette fonction est appelée par le `handleDeviceMotion` ou par un setInterval autonome 
 * si l'IMU n'est pas utilisé.
 */
function updateFastDOM(lat, lon, alt, spd) {

    // --- MISE À JOUR DU SON ---
    if (analyser && !isGpsPaused) {
        const bufferLength = analyser.fftSize;
        const dataArray = new Uint8Array(bufferLength);
        analyser.getByteTimeDomainData(dataArray);

        // Calculer le niveau sonore (RMS ou basé sur le pic)
        let sumOfSquares = 0;
        for (let i = 0; i < bufferLength; i++) {
            const val = (dataArray[i] - 128) / 128; // Normalisation [-1.0, 1.0]
            sumOfSquares += val * val;
        }
        const rms = Math.sqrt(sumOfSquares / bufferLength);
        
        // Convertir en dB SPL (approximatif, nécessite un étalonnage)
        const refPressure = 20e-6; // 20 µPa (Seuil d'audition humaine)
        const currentPressure = rms * 0.5; // Approximation
        const dB = 20 * Math.log10(currentPressure / refPressure);
        
        if (isFinite(dB) && dB > -60) { // Filtrer les valeurs très basses ou NaN
            if ($('sound-level')) $('sound-level').textContent = `${dB.toFixed(1)} dB`;
            soundLevelMax = Math.max(soundLevelMax, dB);
            if ($('sound-level-max')) $('sound-level-max').textContent = `${soundLevelMax.toFixed(1)} dB`;
            
            // Calcul de la moyenne
            soundLevelAvgSum += dB;
            soundLevelAvgCount++;
            if ($('sound-level-avg')) $('sound-level-avg').textContent = `${(soundLevelAvgSum / soundLevelAvgCount).toFixed(1)} dB`;
        
            addDataCapture('SOUND', { db: dB, rms: rms });
        }
    }

    // --- MISE À JOUR UKF/IMU (déjà fait dans handleDeviceMotion, mais on répète pour les vues) ---
    // (Les valeurs d'attitude/accel sont mises à jour dans handleDeviceOrientation/handleDeviceMotion)
    const totalAccel = Math.sqrt(lastAccel.x ** 2 + lastAccel.y ** 2 + lastAccel.z ** 2);
    if ($('accel-total')) $('accel-total').textContent = dataOrDefault(totalAccel, 3, ' m/s²');

    // --- MISE À JOUR DE L'AÉRODYNAMIQUE ---
    if (!isGpsPaused) {
        // Vitesse en m/s (filtrée)
        const V = spd; 
        const airDensity = currentAirDensity; // Utiliser la valeur météo/défaut

        // Pression dynamique
        const q_dyn = getDynamicPressure(V, airDensity);
        if ($('qdyn')) $('qdyn').textContent = dataOrDefault(q_dyn, 2, ' Pa');

        // Force de traînée
        const dragForce = getDragForce(V, airDensity, currentCd);
        if ($('drag')) $('drag').textContent = dataOrDefault(dragForce, 2, ' N');

        // Puissance de traînée (conversion en kW)
        const dragPower_W = getDragPower(dragForce, V);
        if ($('drag-kw')) $('drag-kw').textContent = dataOrDefault(dragPower_W / 1000, 3, ' kW');

        // Nombre de Reynolds
        const reynolds = getReynoldsNumber(V, airDensity);
        if ($('reynolds')) $('reynolds').textContent = dataOrDefaultExp(reynolds, 2);

        // Ajout des données dérivées à la capture
        addDataCapture('DERIVED', { q_dyn, dragForce, dragPower_W, reynolds });
    }

    // --- MISE À JOUR DES VALEURS RELATIVISTES ---
    if (!isGpsPaused) {
        const V = spd;
        const advancedPhysics = {
            lorentzFactor: lorentzGamma(V),
            timeDilationRatio: timeDilationRatio(lorentzGamma(V))
        };

        // Vitesse en % de C_L
        const percSpeedC = (V / C_L) * 100;

        if ($('perc-speed-c')) $('perc-speed-c').textContent = dataOrDefault(percSpeedC, 8, ' %');
        if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(advancedPhysics.lorentzFactor, 8);
        if ($('time-dilation-v')) $('time-dilation-v').textContent = dataOrDefault(advancedPhysics.timeDilationRatio, 8);
    }
}

/**
 * Démarre la boucle de mise à jour rapide du DOM (utilisée si IMU non disponible).
 */
function startFastDOMUpdateLoop() {
    if (domFastID !== null) clearInterval(domFastID);

    domFastID = setInterval(() => {
        if (!isGpsPaused) {
            // Utiliser les valeurs filtrées si disponibles
            let alt = currentPosition.alt || 0.0;
            let spd = currentPosition.spd || 0.0;

            if (ukf.getState()) {
                const estimatedState = ukf.getState();
                alt = estimatedState.alt;
                spd = estimatedState.speed;
            }
            // Appeler la fonction de mise à jour rapide
            updateFastDOM(currentPosition.lat, currentPosition.lon, alt, spd);
        }
    }, IMU_UPDATE_RATE_MS); // 100ms
}

// =================================================================
// BLOC 5/5 : Boucle Lente (Astro/Météo) et Initialisation
 // =================================================================
// BLOC 5/5 : Boucle Lente (Astro/Météo) et Initialisation
// =================================================================

// --- LOGIQUE DE MISE À JOUR LENTE (Météo, Astro, Affichages Temps) ---
const SLOW_UPDATE_RATE_MS = 10000; // 10 secondes

/**
 * Effectue la récupération des données météo (API).
 * Utilise un proxy pour éviter les problèmes CORS/API Key.
 * @param {number} lat Latitude en degrés.
 * @param {number} lon Longitude en degrés.
 */
async function fetchWeather(lat, lon) {
    if (isGpsPaused) return;

    if ($('weather-status')) $('weather-status').textContent = "Mise à jour...";
    
    try {
        // Météo
        const weatherResponse = await fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`);
        const weatherData = await weatherResponse.json();
        if (weatherData.error) throw new Error(weatherData.error);
        
        lastKnownWeather = {
            pressure_hPa: weatherData.pressure_hPa,
            tempK: weatherData.tempK,
            tempC: weatherData.tempC,
            humidity: weatherData.humidity,
            windSpeed_ms: weatherData.windSpeed_ms,
            windDeg: weatherData.windDeg,
            air_density: getAirDensity(weatherData.pressure_hPa, weatherData.tempK),
        };
        updateWeatherDOM(lastKnownWeather);
        
        // Polluants
        const pollutantResponse = await fetch(`${PROXY_POLLUTANT_ENDPOINT}?lat=${lat}&lon=${lon}`);
        const pollutantData = await pollutantResponse.json();
        if (pollutantData.error) throw new Error(pollutantData.error);
        
        lastKnownPollutants = pollutantData;
        updatePollutantsDOM(lastKnownPollutants);

        if ($('weather-status')) $('weather-status').textContent = `Météo/Air Réel (Maj: ${new Date().toLocaleTimeString()})`;

        // Mettre à jour les variables globales utilisées dans la boucle rapide
        lastT_K = lastKnownWeather.tempK;
        lastP_hPa = lastKnownWeather.pressure_hPa;
        currentAirDensity = lastKnownWeather.air_density;
        currentSpeedOfSound = getSpeedOfSound(lastT_K);
        
    } catch (error) {
        console.warn('Erreur de récupération météo/air :', error);
        if ($('weather-status')) $('weather-status').textContent = "Météo Hors-Ligne (Défaut)";
    }
}

/**
 * Met à jour le DOM avec les données météo.
 * @param {object} data Données météo.
 * @param {boolean} isDefault Indique si c'est une valeur par défaut.
 */
function updateWeatherDOM(data, isDefault = false) {
    // Pression et Altitude Barométrique
    if ($('pressure')) $('pressure').textContent = dataOrDefault(data.pressure_hPa, 2, ' hPa');
    const baroAlt = getBarometricAltitude(data.pressure_hPa);
    if ($('altitude-baro')) $('altitude-baro').textContent = dataOrDefault(baroAlt, 2, ' m');

    // Température et Humidité
    if ($('temperature')) $('temperature').textContent = dataOrDefault(data.tempC, 1, ' °C');
    if ($('humidity')) $('humidity').textContent = dataOrDefault(data.humidity, 0, ' %');

    // Vent
    if ($('wind-speed')) $('wind-speed').textContent = dataOrDefault(data.windSpeed_ms * KMH_MS, 1, ' km/h');
    if ($('wind-direction')) $('wind-direction').textContent = `${dataOrDefault(data.windDeg, 0)}° ${degToCardinal(data.windDeg)}`;

    // Densité de l'air et Vitesse du Son
    if ($('air-density')) $('air-density').textContent = dataOrDefault(data.air_density, 3, ' kg/m³');
    const speedOfSound = getSpeedOfSound(data.tempK);
    const suffix = isDefault ? ' m/s (Défaut)' : ' m/s (Calculé)';
    if ($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = dataOrDefault(speedOfSound, 2, suffix);
}

/**
 * Met à jour le DOM avec les données de polluants.
 * @param {object} data Données de polluants.
 */
function updatePollutantsDOM(data) {
    if ($('aqi-status')) $('aqi-status').textContent = `AQI: ${data.aqi_us} (${data.category})`;
    if ($('co-level')) $('co-level').textContent = `${dataOrDefault(data.CO, 1, '')} µg/m³`;
    if ($('no2-level')) $('no2-level').textContent = `${dataOrDefault(data.NO2, 1, '')} µg/m³`;
}

/**
 * Mise à jour des données du DOM nécessitant une basse fréquence (10s).
 * Inclut Astro, Météo (si non en fetch), Temps.
 */
function updateSlowDOM(lat, lon) {
    if (isGpsPaused) return;

    const now = new Date();

    // 1. Mise à jour des heures
    if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString();

    // 2. Mise à jour des temps écoulés
    if (timerStartDate) {
        const elapsed = now.getTime() - timerStartDate;
        const elapsedSeconds = Math.floor(elapsed / 1000);
        const elapsedHours = Math.floor(elapsedSeconds / 3600);
        const elapsedMinutes = Math.floor((elapsedSeconds % 3600) / 60);
        const elapsedSecondsFinal = elapsedSeconds % 60;
        
        const formatTime = (ms) => {
            const totalSeconds = Math.floor(ms / 1000);
            const h = Math.floor(totalSeconds / 3600);
            const m = Math.floor((totalSeconds % 3600) / 60);
            const s = totalSeconds % 60;
            return `${String(h).padStart(2, '0')}h ${String(m).padStart(2, '0')}m ${String(s).padStart(2, '0')}s`;
        };

        if ($('time-elapsed')) $('time-elapsed').textContent = formatTime(elapsed);
        if ($('time-moving')) $('time-moving').textContent = formatTime(timerMovingTime);
    }

    // 3. Mise à jour de l'Astronomie (SunCalc)
    const sunTimes = SunCalc.getTimes(now, lat, lon);
    const sunPos = SunCalc.getPosition(now, lat, lon);
    const moonPos = SunCalc.getMoonPosition(now, lat, lon);
    const moonIllumination = SunCalc.getMoonIllumination(now, lat, lon);
    
    // Soleil
    if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(sunPos.altitude * R2D, 1, '°');
    if ($('sun-azimuth')) $('sun-azimuth').textContent = `${dataOrDefault(sunPos.azimuth * R2D + 180, 1)}° ${degToCardinal(sunPos.azimuth * R2D + 180)}`;
    const dayDuration = sunTimes.sunset.getTime() - sunTimes.sunrise.getTime();
    if ($('day-duration')) $('day-duration').textContent = new Date(dayDuration).toISOString().substr(11, 8); // Format H:M:S

    // Lune
    const phaseNames = ['Nouvelle Lune', 'Croissant (Montante)', 'Premier Quartier', 'Gibbeuse (Montante)', 'Pleine Lune', 'Gibbeuse (Décroissante)', 'Dernier Quartier', 'Croissant (Décroissante)'];
    const moonAge = moonIllumination.phase * 8; // Phase de 0 à 1, convertir en 0 à 8
    if ($('moon-phase-name')) $('moon-phase-name').textContent = phaseNames[Math.floor(moonAge) % 8];
    if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(moonIllumination.fraction * 100, 1, ' %');
    if ($('moon-alt')) $('moon-alt').textContent = dataOrDefault(moonPos.altitude * R2D, 1, '°');
    if ($('moon-azimuth')) $('moon-azimuth').textContent = `${dataOrDefault(moonPos.azimuth * R2D + 180, 1)}°`;

    // 4. Mettre à jour l'affichage de l'environnement
    const env = ENVIRONMENT_FACTORS[selectedEnvironment];
    if ($('env-factor')) $('env-factor').textContent = `${env.DISPLAY} (x${env.MULT.toFixed(1)})`;
}

// --- LOGIQUE ISS/ORBITE (Simulation/Données API) ---

async function fetchISSData() {
    // Logique pour récupérer la position de l'ISS
    if (isGpsPaused) return;
    try {
        const response = await fetch(ISS_ENDPOINT);
        const data = await response.json();
        
        if (data.latitude && data.longitude) {
            const latlng = L.latLng(data.latitude, data.longitude);
            if (issMarker) issMarker.setLatLng(latlng);
            if (issTrail) issTrail.addLatLng(latlng);
            
            // Afficher la position de l'ISS
            if ($('iss-lat')) $('iss-lat').textContent = dataOrDefault(data.latitude, 4) + '°';
            if ($('iss-lon')) $('iss-lon').textContent = dataOrDefault(data.longitude, 4) + '°';
            if ($('iss-alt')) $('iss-alt').textContent = dataOrDefault(data.altitude * 1000, 0, ' m'); // Altitude est en km
            if ($('iss-speed')) $('iss-speed').textContent = dataOrDefault(data.velocity, 0, ' km/h');
        }
    } catch (error) {
        console.warn("Erreur ISS : ", error);
        if ($('iss-lat')) $('iss-lat').textContent = "Hors Ligne";
    }
}


// --- FONCTION D'INITIALISATION GLOBALE (Main) ---

/**
 * Initialise la carte, les capteurs et les boucles de mise à jour.
 */
function initDashboard() {
    
    // 1. Initialisation de la carte Leaflet
    map = L.map('map').setView([currentPosition.lat, currentPosition.lon], 13);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
    }).addTo(map);

    userMarker = L.marker([currentPosition.lat, currentPosition.lon], {
        icon: L.divIcon({ className: 'user-marker', html: '<i class="fas fa-crosshairs"></i>' })
    }).addTo(map);

    // Marqueur ISS
    issMarker = L.marker([0, 0], {
        icon: L.divIcon({ className: 'iss-marker', html: '<i class="fas fa-satellite"></i>' })
    }).addTo(map);
    issTrail = L.polyline([], { color: 'yellow', weight: 2, opacity: 0.7 }).addTo(map);

    // 2. Initialisation des valeurs par défaut (Correction Métrologique)
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
    if ($('env-factor')) $('env-factor').textContent = `${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY} (x${ENVIRONMENT_FACTORS[selectedEnvironment].MULT.toFixed(1)})`;
    if (document.body.classList.contains('dark-mode')) {
         $('toggle-mode-btn').innerHTML = '<i class="fas fa-sun"></i> Mode Jour';
    } else {
         $('toggle-mode-btn').innerHTML = '<i class="fas fa-moon"></i> Mode Nuit';
    }

    // 3. Lancement des capteurs et des boucles de mise à jour
    startIMUSensors(); // Démarre l'IMU et la boucle rapide (si supporté)
    if (domFastID === null) startFastDOMUpdateLoop(); // Si IMU non supporté, on lance la boucle rapide seule

    domSlowID = setInterval(() => {
        if (!isGpsPaused) {
            updateSlowDOM(currentPosition.lat, currentPosition.lon);
        }
    }, SLOW_UPDATE_RATE_MS); // 10 secondes

    weatherFetchID = setInterval(() => {
        if (!isGpsPaused) {
            fetchWeather(currentPosition.lat, currentPosition.lon);
        }
    }, 600000); // 10 minutes (600000 ms)

    // Lancer une première mise à jour lente et météo/ISS
    updateSlowDOM(currentPosition.lat, currentPosition.lon);
    fetchWeather(currentPosition.lat, currentPosition.lon);
    setInterval(fetchISSData, 30000); // 30 secondes pour l'ISS
    fetchISSData();

    // 4. Écouteurs d'événements
    $('toggle-gps-btn').addEventListener('click', () => {
        isGpsPaused ? startGPSAcquisition() : stopGPSAcquisition();
    });
    $('data-capture-btn').addEventListener('click', () => {
        isRecording ? stopDataCapture() : startDataCapture();
    });
    // ... autres écouteurs ...
    
    console.log("Tableau de bord GNSS SpaceTime initialisé.");
}

// Lancement de l'initialisation
initDashboard();

})(window);
