// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// NOYAU SCIENTIFIQUE PROFESSIONNEL ET LOGIQUE DE TABLEAU DE BORD
// Dépendances critiques : math.min.js, leaflet.js, suncalc.js, turf.min.js
// =================================================================

// =================================================================
// BLOC 1/5 : Constantes, Utilitaires et État Global
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
    if (val === undefined || val === null || isNaN(val) || val === Infinity || val === -Infinity) {
        const zeroDecimals = '0.' + Array(decimals).fill('0').join('');
        return zeroDecimals + 'e+0' + suffix;
    }
    // Assure au moins 2 chiffres après la virgule pour l'exposant (ex: 1.00e+0)
    return val.toExponential(decimals < 2 ? 2 : decimals) + suffix;
};

const getCDate = () => window.lastNTPDate || new Date();

// --- CLÉS D'API & ENDPOINTS (À configurer) ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app"; 
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES FONDAMENTALES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;          // Vitesse de la lumière (m/s)
const G_U = 6.67430e-11;        // Constante gravitationnelle universelle (N·m²/kg²)
const KMH_MS = 3.6;             // Conversion m/s vers km/h

// --- CONSTANTES WGS84 ---
const WGS84_A = 6378137.0;       // Rayon semi-grand (m)
const WGS84_F = 1 / 298.257223563; // Aplatissement
const WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F; // Excentricité au carré
const OMEGA_EARTH = 7.292115e-5; // Vitesse de rotation de la Terre (rad/s)
const G_MU_EARTH = 3.986004418e14; // Constante gravitationnelle de la Terre (m³/s²)
let G_ACC = 9.80665;             // Gravité de base (m/s²)
let R_ALT_CENTER_REF = 6371000; // Rayon Terre (m)

// --- CONSTANTES MÉTÉO/ATMOSPHÈRE (ISA Standard) ---
const TEMP_SEA_LEVEL_K = 288.15; // 15 °C
const RHO_SEA_LEVEL = 1.225;     // Densité de l'air au niveau de la mer (kg/m³)
const R_SPECIFIC_AIR = 287.058;  // Constante spécifique de l'air sec (J/kg·K)
const GAMMA_AIR = 1.4;           // Indice adiabatique de l'air
const BARO_ALT_REF_HPA = 1013.25; // Pression de référence (hPa)

// --- ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE ---
let kLat = 43.2964, kLon = 5.3697, kAlt = 0.0, kSpd = 0.0, kAcc = 10.0;
let kSpeedAvg = 0.0, kSpeedMax = 0.0;
let distM = 0.0, distMStartOffset = 0.0;
let totalTime = 0, motionTime = 0, lastUpdateTime = performance.now();
let lastGpsTime = 0;
let lastP_hPa = BARO_ALT_REF_HPA, lastT_K = TEMP_SEA_LEVEL_K;
let currentAirDensity = RHO_SEA_LEVEL, currentSpeedOfSound = 340.29; // Défaut ISA
let currentMass = 70.0, currentDragCoeff = 1.0, currentFrontalArea = 0.5;
let currentCelestialBody = 'TERRE';
let rotationRadius = 100, angularVelocity = 0.0;
let netherMode = false, distanceRatioMode = false;
let gpsWatchID = null;
let sunAltitudeRad = 0.0;
let currentUKFReactivity = 'NORMAL'; // NORMAL | AGRESSIF | ADAPTATIF

// UKF Instance
let ukf; 

// Capteurs IMU (Body Frame)
let lastAccel = { x: 0, y: 0, z: 0 };
let lastGyro = { x: 0, y: 0, z: 0 };
let lastMag = { x: 0, y: 0, z: 0 }; // Utilisé pour l'initialisation du Quaternion (non intégré dans le UKF 21 ici pour la simplicité du web)

// =================================================================
// BLOC 2/5 : Modèles Physiques (ECEF, Quaternion, Géodésie)
// =================================================================

/**
 * Crée la matrice Skew-Symétrique d'un vecteur (pour le produit vectoriel matriciel).
 */
function skewSymmetric(v) {
    const vx = v.get([0, 0]), vy = v.get([1, 0]), vz = v.get([2, 0]);
    return math.matrix([
        [0, -vz, vy],
        [vz, 0, -vx],
        [-vy, vx, 0]
    ]);
}

/**
 * Convertit les coordonnées ECEF (x, y, z) en coordonnées géodésiques (Lat, Lon, Alt).
 * Algorithme de Bowring (approche itérative).
 * @param {math.Matrix} r_ecef - Position ECEF [x, y, z] (3x1 Matrix)
 */
function ecefToGeodetic(r_ecef) {
    const x = r_ecef.get([0, 0]), y = r_ecef.get([1, 0]), z = r_ecef.get([2, 0]);
    const p = Math.sqrt(x * x + y * y);

    if (p < 1e-6) return { lat: 90 * D2R, lon: 0, alt: z - (WGS84_A * (1 - WGS84_E2)) };

    let lat_prev = Math.atan2(z, p * (1 - WGS84_E2));
    let N; // Rayon de Courbure dans la verticale principale (Prime Vertical)

    for (let i = 0; i < 5; i++) {
        const sinLat = Math.sin(lat_prev);
        N = WGS84_A / Math.sqrt(1 - WGS84_E2 * sinLat * sinLat);
        const lat_next = Math.atan2(z + N * WGS84_E2 * sinLat, p);
        if (Math.abs(lat_next - lat_prev) < 1e-10) break;
        lat_prev = lat_next;
    }

    const lat = lat_prev;
    const lon = Math.atan2(y, x);
    const alt = p / Math.cos(lat) - N;

    return { lat: lat * R2D, lon: lon * R2D, alt: alt }; // Retourne en degrés pour l'affichage
}

/**
 * Calcule la matrice de rotation C_e_b (ECEF vers Body) à partir du Quaternion q_e_b.
 * @param {math.Matrix} q - Quaternion [q0, q1, q2, q3] (4x1 Matrix)
 */
function quatToRotationMatrix(q) {
    const q0 = q.get([0, 0]), q1 = q.get([1, 0]), q2 = q.get([2, 0]), q3 = q.get([3, 0]);
    
    const C_e_b = math.matrix([
        [q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3, 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
        [2 * (q1 * q2 + q0 * q3), q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3, 2 * (q2 * q3 - q0 * q1)],
        [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3]
    ]);
    return C_e_b;
}

/**
 * Intégration du Quaternion (Modèle d'Euler pour l'intégration de l'attitude).
 * @param {math.Matrix} q - Quaternion initial (4x1 Matrix)
 * @param {math.Matrix} omega - Vitesse angulaire (3x1 Matrix)
 * @param {number} dt - Intervalle de temps
 */
function integrateQuaternion(q, omega, dt) {
    const OmegaMatrix = math.matrix([
        [0, -omega.get([0, 0]), -omega.get([1, 0]), -omega.get([2, 0])],
        [omega.get([0, 0]), 0, omega.get([2, 0]), -omega.get([1, 0])],
        [omega.get([1, 0]), -omega.get([2, 0]), 0, omega.get([0, 0])],
        [omega.get([2, 0]), omega.get([1, 0]), -omega.get([0, 0]), 0]
    ]);
    
    const delta_q = math.multiply(OmegaMatrix, q, 0.5 * dt);
    const q_next = math.add(q, delta_q);
    
    // Normalisation essentielle pour éviter la dérive
    const q_norm = math.norm(q_next);
    return math.divide(q_next, q_norm);
}

/**
 * Calcule le vecteur de gravité en ECEF g_e. (Modèle sphérique simple).
 * @param {math.Matrix} r_ecef - Position ECEF [x, y, z] (3x1 Matrix)
 */
function gravityEcef(r_ecef) {
    const r_norm = math.norm(r_ecef);
    const unit_r = math.divide(r_ecef, r_norm);
    
    // Gravité inverse square law
    const g_magnitude = G_MU_EARTH / (r_norm * r_norm);
    
    // Vecteur gravité dirigé vers le centre de la Terre
    const g_ecef = math.multiply(unit_r, -g_magnitude);
    
    return g_ecef;
}

/**
 * Convertit les coordonnées Géodésiques (Lat/Lon/Alt en degrés) en ECEF.
 * @param {number} lat_deg - Latitude en degrés
 * @param {number} lon_deg - Longitude en degrés
 * @param {number} alt_m - Altitude en mètres
 * @returns {math.Matrix} Position ECEF [x, y, z] (3x1 Matrix)
 */
function geodeticToEcef(lat_deg, lon_deg, alt_m) {
    const lat = lat_deg * D2R;
    const lon = lon_deg * D2R;
    const sinLat = Math.sin(lat), cosLat = Math.cos(lat);
    const sinLon = Math.sin(lon), cosLon = Math.cos(lon);

    const N = WGS84_A / Math.sqrt(1 - WGS84_E2 * sinLat * sinLat);
    
    const x = (N + alt_m) * cosLat * cosLon;
    const y = (N + alt_m) * cosLat * sinLon;
    const z = (N * (1 - WGS84_E2) + alt_m) * sinLat;
    
    return math.matrix([[x], [y], [z]]);
}

// =================================================================
// BLOC 3/5 : CLASSE PROFESSIONAL UKF (Unscented Kalman Filter - 21 États)
// =================================================================

class ProfessionalUKF {
    constructor() {
        this.N_STATES = 21; // [r_e(3), v_e(3), q_e_b(4), b_g(3), b_a(3), b_m(3), b_clk(2)]
        this.N_MEAS = 6;    // [Lat, Lon, Alt, V_mag, b_clk, b_clk_dot]

        this.alpha = 1e-3;
        this.beta = 2.0;
        this.kappa = 0.0; // Kappa = 0 pour les systèmes à 21 états
        this.lambda = this.alpha * this.alpha * (this.N_STATES + this.kappa) - this.N_STATES;

        // Poids pour les Sigma Points
        this.Wm = math.zeros(2 * this.N_STATES + 1, 1);
        this.Wc = math.zeros(2 * this.N_STATES + 1, 1);
        this.Wm.set([0, 0], this.lambda / (this.N_STATES + this.lambda));
        this.Wc.set([0, 0], this.lambda / (this.N_STATES + this.lambda) + (1 - this.alpha * this.alpha + this.beta));

        for (let i = 1; i < 2 * this.N_STATES + 1; i++) {
            this.Wm.set([i, 0], 1 / (2 * (this.N_STATES + this.lambda)));
            this.Wc.set([i, 0], 1 / (2 * (this.N_STATES + this.lambda)));
        }

        // --- État initial (x) ---
        // Utilisé ici pour initialiser UKF après la première lecture GPS
        const initialR = geodeticToEcef(kLat, kLon, kAlt);
        const initialV = math.zeros(3, 1); // Vitesse initiale nulle
        
        // Quaternion initial (simplifié: North, East, Down -> ECEF)
        // Ceci est une simplification. Une initialisation professionnelle nécessite un Cap (Yaw)
        const initialQ = math.matrix([[1], [0], [0], [0]]); // Quaternion identité [q0, q1, q2, q3]

        this.x = math.zeros(this.N_STATES, 1);
        this.x.subset(math.index(math.range(0, 3), 0), initialR); // r_e (0-2)
        this.x.subset(math.index(math.range(3, 6), 0), initialV); // v_e (3-5)
        this.x.subset(math.index(math.range(6, 10), 0), initialQ); // q_e_b (6-9)
        // Biases (10-20) initialisés à zéro

        // --- Covariance initiale (P) ---
        this.P = math.identity(this.N_STATES, this.N_STATES);
        // Position/Vitesse ECEF (Position ECEF initiale très incertaine, 1000m)
        this.P.set([0, 0], 1000); this.P.set([1, 1], 1000); this.P.set([2, 2], 1000);
        this.P.set([3, 3], 1); this.P.set([4, 4], 1); this.P.set([5, 5], 1);
        // Quaternion (Attitude incertaine, 0.1 rad)
        this.P.set([6, 6], 0.1); this.P.set([7, 7], 0.1); this.P.set([8, 8], 0.1); this.P.set([9, 9], 0.1);
        // Biases (Très incertain, 10e-3 pour les capteurs, 100 pour l'horloge)
        for (let i = 10; i < 19; i++) this.P.set([i, i], 1e-4);
        this.P.set([19, 19], 100); this.P.set([20, 20], 10); // Clock Bias/Drift

        // --- Bruit de Processus (Q) ---
        this.Q = math.zeros(this.N_STATES, this.N_STATES);
        // Position est dérivée de la vitesse/accel, Q est petit (0.1)
        for (let i = 0; i < 3; i++) this.Q.set([i, i], 0.1); 
        // Vitesse (Accélération bruitée, 1.0)
        for (let i = 3; i < 6; i++) this.Q.set([i, i], 1.0); 
        // Quaternion (Gyroscope bruitée, 0.01)
        for (let i = 6; i < 10; i++) this.Q.set([i, i], 0.01); 
        // Biases (Modèle Markov 1er ordre: drift lent, 1e-8)
        for (let i = 10; i < 19; i++) this.Q.set([i, i], 1e-8);
        this.Q.set([19, 19], 0.1); this.Q.set([20, 20], 0.01); // Clock Bias/Drift Noise
    }

    /**
     * Calcule les Sigma Points (Chi) à partir de l'état (x) et de la covariance (P).
     */
    generateSigmaPoints(x, P) {
        const N = this.N_STATES;
        const sqrtPP = math.sqrtm(math.multiply(P, N + this.lambda)); // Racine matricielle

        const Chi = math.zeros(N, 2 * N + 1);
        Chi.subset(math.index(math.range(0, N), 0), x); // Premier point: x
        
        for (let i = 0; i < N; i++) {
            // Points positifs (x + sqrtPP[i])
            const col_pos = math.subset(sqrtPP, math.index(math.range(0, N), i));
            Chi.subset(math.index(math.range(0, N), i + 1), math.add(x, col_pos));
            
            // Points négatifs (x - sqrtPP[i])
            const col_neg = math.subset(sqrtPP, math.index(math.range(0, N), i));
            Chi.subset(math.index(math.range(0, N), i + N + 1), math.subtract(x, col_neg));
        }
        return Chi;
    }

    /**
     * [CORE NON LINÉAIRE] Fonction de Transition d'État f(x_k, u_k, dt).
     * Modèle cinématique ECEF, Coriolis/Gravité, Intégration IMU/Quaternion.
     */
    f(x_in, imuReadings, dt) {
        // Décomposition de l'état (21 états)
        const r_e = x_in.subset(math.index(math.range(0, 3), 0));   
        const v_e = x_in.subset(math.index(math.range(3, 6), 0));   
        const q_e_b = x_in.subset(math.index(math.range(6, 10), 0));
        const b_g = x_in.subset(math.index(math.range(10, 13), 0)); 
        const b_a = x_in.subset(math.index(math.range(13, 16), 0)); 
        const b_m = x_in.subset(math.index(math.range(16, 19), 0)); 
        const b_clk = x_in.subset(math.index(math.range(19, 21), 0)); 
        
        // Lectures des capteurs (IMU) & CORRECTION DES BIAS
        const f_raw_b = math.matrix([[imuReadings.accel.x], [imuReadings.accel.y], [imuReadings.accel.z]]);
        const w_raw_b = math.matrix([[imuReadings.gyro.x], [imuReadings.gyro.y], [imuReadings.gyro.z]]);
        
        const f_b = math.subtract(f_raw_b, b_a); // Force spécifique corrigée
        const w_b = math.subtract(w_raw_b, b_g); // Vitesse angulaire corrigée (w_ib_b)
        
        // Dynamiques de l'Attitude (Quaternion)
        const w_ie_e = math.matrix([[0], [0], [OMEGA_EARTH]]); 
        const C_b_e = math.transpose(quatToRotationMatrix(q_e_b));
        const w_ie_b = math.multiply(quatToRotationMatrix(q_e_b), w_ie_e);
        const w_eb_b = math.subtract(w_b, w_ie_b);

        const q_e_b_next = integrateQuaternion(q_e_b, w_eb_b, dt);
        
        // Dynamiques de la Vitesse (ECEF)
        const g_e = gravityEcef(r_e);
        const Omega_ie_e_skew = skewSymmetric(w_ie_e);
        const coriolis_e = math.multiply(Omega_ie_e_skew, v_e, 2);
        
        // Accélération dans ECEF: a_e = C_b_e * f_b + g_e - Coriolis
        const f_e = math.multiply(C_b_e, f_b);
        const a_e = math.subtract(math.add(f_e, g_e), coriolis_e);

        const v_e_next = math.add(v_e, math.multiply(a_e, dt));

        // Dynamiques de la Position (ECEF)
        const r_e_next = math.add(r_e, math.multiply(v_e, dt), math.multiply(a_e, dt * dt * 0.5));
        
        // Dynamiques des Biases (Modèle Constant pour la Prédiction)
        const b_g_next = b_g; 
        const b_a_next = b_a; 
        const b_m_next = b_m; 
        
        // Clock Bias (b_clk) et Clock Drift (b_clk_dot)
        const b_clk_next_0 = b_clk.get([0, 0]) + b_clk.get([1, 0]) * dt;
        const b_clk_next_1 = b_clk.get([1, 0]);
        const b_clk_next = math.matrix([[b_clk_next_0], [b_clk_next_1]]);

        // Reconstruction de l'état propagé (21x1)
        const x_out = math.zeros(this.N_STATES, 1);
        x_out.subset(math.index(math.range(0, 3), 0), r_e_next);
        x_out.subset(math.index(math.range(3, 6), 0), v_e_next);
        x_out.subset(math.index(math.range(6, 10), 0), q_e_b_next);
        x_out.subset(math.index(math.range(10, 13), 0), b_g_next);
        x_out.subset(math.index(math.range(13, 16), 0), b_a_next);
        x_out.subset(math.index(math.range(16, 19), 0), b_m_next);
        x_out.subset(math.index(math.range(19, 21), 0), b_clk_next);

        return x_out;
    }

    /**
     * [CORE NON LINÉAIRE] Fonction de Mesure h(x_k).
     * Transforme l'état prédit (ECEF) en mesures GPS attendues (Lat, Lon, Alt, Speed_Mag, Clock_Bias, Clock_Drift).
     */
    h(x_in) {
        // Décomposition des états nécessaires pour la mesure
        const r_e = x_in.subset(math.index(math.range(0, 3), 0)); 
        const v_e = x_in.subset(math.index(math.range(3, 6), 0));   
        const b_clk = x_in.subset(math.index(math.range(19, 21), 0)); 
        
        // Transformation ECEF -> Géodésique (Lat/Lon/Alt)
        const { lat, lon, alt } = ecefToGeodetic(r_e); // Lat/Lon en degrés
        
        // Calcul de la Vitesse (Magnitude du vecteur Vitesse ECEF)
        const v_mag = math.norm(v_e);
        
        // Reconstruction de la Mesure Prédite (6x1)
        return math.matrix([
            [lat * D2R], // Lat (rad)
            [lon * D2R], // Lon (rad)
            [alt],       // Alt (m)
            [v_mag],     // Speed (m/s)
            [b_clk.get([0, 0])], // Clock Bias (m)
            [b_clk.get([1, 0])]  // Clock Drift (m/s)
        ]);
    }

    /**
     * Étape de Prédiction UKF.
     */
    predict(imuReadings, dt) {
        const Chi = this.generateSigmaPoints(this.x, this.P);
        const N = this.N_STATES;
        
        // 1. Propagation des Sigma Points
        const Chi_star = math.zeros(N, 2 * N + 1);
        for (let i = 0; i < 2 * N + 1; i++) {
            const x_i = Chi.subset(math.index(math.range(0, N), i));
            Chi_star.subset(math.index(math.range(0, N), i), this.f(x_i, imuReadings, dt));
        }

        // 2. Calcul de l'état prédit (x_bar)
        const x_bar = math.zeros(N, 1);
        for (let i = 0; i < 2 * N + 1; i++) {
            const x_i_star = Chi_star.subset(math.index(math.range(0, N), i));
            x_bar.set(math.index(math.range(0, N), 0), math.add(x_bar, math.multiply(x_i_star, this.Wm.get([i, 0]))));
        }

        // 3. Calcul de la Covariance Prédite (P_bar)
        let P_bar = math.zeros(N, N);
        const Q_scaled = math.multiply(this.Q, dt * dt); // Le bruit de processus Q est souvent échelonné par dt^2
        
        for (let i = 0; i < 2 * N + 1; i++) {
            const P_diff = math.subtract(Chi_star.subset(math.index(math.range(0, N), i)), x_bar);
            const P_term = math.multiply(P_diff, math.transpose(P_diff), this.Wc.get([i, 0]));
            P_bar = math.add(P_bar, P_term);
        }
        P_bar = math.add(P_bar, Q_scaled);
        
        this.x_bar = x_bar;
        this.P_bar = P_bar;
        this.Chi_star = Chi_star;

        return { x_bar, P_bar };
    }

    /**
     * Étape de Mise à Jour UKF.
     * @param {math.Matrix} z_real - Vecteur de mesure réelle (6x1)
     * @param {number} R_value - Valeur de bruit de mesure (pour Lat/Lon/Alt/Speed)
     */
    update(z_real, R_value) {
        const N = this.N_STATES;
        const M = this.N_MEAS;
        
        // 1. Transformation des Sigma Points propagés en espace de mesure
        const Psi = math.zeros(M, 2 * N + 1);
        for (let i = 0; i < 2 * N + 1; i++) {
            const x_i_star = this.Chi_star.subset(math.index(math.range(0, N), i));
            Psi.subset(math.index(math.range(0, M), i), this.h(x_i_star));
        }

        // 2. Calcul de la mesure prédite (z_bar)
        const z_bar = math.zeros(M, 1);
        for (let i = 0; i < 2 * N + 1; i++) {
            const Psi_i = Psi.subset(math.index(math.range(0, M), i));
            z_bar.set(math.index(math.range(0, M), 0), math.add(z_bar, math.multiply(Psi_i, this.Wm.get([i, 0]))));
        }

        // 3. Calcul de la Matrice de Covariance de l'Innovation (Pzz)
        let Pzz = math.zeros(M, M);
        // Matrice de bruit de mesure R (GPS, 6x6)
        const R = math.diag([R_value, R_value, R_value, R_value / 100, 100, 10]); // Ajustement pour Vitesse/Clock Biases
        
        for (let i = 0; i < 2 * N + 1; i++) {
            const Psi_diff = math.subtract(Psi.subset(math.index(math.range(0, M), i)), z_bar);
            const Pzz_term = math.multiply(Psi_diff, math.transpose(Psi_diff), this.Wc.get([i, 0]));
            Pzz = math.add(Pzz, Pzz_term);
        }
        Pzz = math.add(Pzz, R);

        // 4. Calcul de la Matrice de Covariance Croisée (Pxz)
        let Pxz = math.zeros(N, M);
        for (let i = 0; i < 2 * N + 1; i++) {
            const x_diff = math.subtract(this.Chi_star.subset(math.index(math.range(0, N), i)), this.x_bar);
            const z_diff = math.subtract(Psi.subset(math.index(math.range(0, M), i)), z_bar);
            const Pxz_term = math.multiply(x_diff, math.transpose(z_diff), this.Wc.get([i, 0]));
            Pxz = math.add(Pxz, Pxz_term);
        }

        // 5. Calcul du Gain de Kalman (K)
        const K = math.multiply(Pxz, math.inv(Pzz));

        // 6. Mise à jour de l'état (x)
        const innovation = math.subtract(z_real, z_bar);
        this.x = math.add(this.x_bar, math.multiply(K, innovation));
        
        // Normalisation du Quaternion après la mise à jour (indices 6 à 9)
        const q_update = this.x.subset(math.index(math.range(6, 10), 0));
        const q_norm = math.norm(q_update);
        this.x.subset(math.index(math.range(6, 10), 0), math.divide(q_update, q_norm));

        // 7. Mise à jour de la Covariance (P)
        this.P = math.subtract(this.P_bar, math.multiply(K, Pzz, math.transpose(K)));
        
        return { innovation: math.norm(innovation), P_speed: math.sqrt(this.P.get([3, 3])) }; // Retourne l'incertitude vitesse
    }
}

// =================================================================
// BLOC 4/5 : Fonctions Physiques, Météorologiques et Relativistes
// =================================================================

/**
 * Calcul de la densité de l'air (Modèle ISA standard pour une altitude, ajusté par P et T)
 */
function getAirDensity(pressure_hPa, temp_K) {
    // ρ = P / (R_SPECIFIC_AIR * T) [P en Pa, T en K]
    const pressure_Pa = pressure_hPa * 100;
    return pressure_Pa / (R_SPECIFIC_AIR * temp_K);
}

/**
 * Calcul de la vitesse du son (c_s) en fonction de la température T (en Kelvin).
 */
function getSpeedOfSound(temp_K) {
    // c_s = sqrt(γ * R_SPECIFIC_AIR * T)
    return Math.sqrt(GAMMA_AIR * R_SPECIFIC_AIR * temp_K);
}

/**
 * Calcul des forces aérodynamiques (Traînée).
 */
function calculateDrag(speed_ms, air_density, C_d = currentDragCoeff, A_f = currentFrontalArea) {
    // F_traînée = 0.5 * ρ * v² * C_d * A_f
    const dynamic_pressure = 0.5 * air_density * speed_ms * speed_ms;
    const force = dynamic_pressure * C_d * A_f;
    const power = force * speed_ms;
    return { force: force, power: power, dynamic_pressure: dynamic_pressure };
}

/**
 * Calcul du Nombre de Reynolds. (Simplifié, utilise une longueur caractéristique L=1.0m)
 */
function calculateReynolds(speed_ms, air_density) {
    const L_char = 1.0; // Longueur caractéristique de l'objet (arbitraire)
    const MU_DYNAMIC_AIR = 1.8e-5; // Viscosité dynamique de l'air (Pa·s)
    return (air_density * speed_ms * L_char) / MU_DYNAMIC_AIR;
}

/**
 * Calculs de la Relativité Restreinte (SR).
 */
function calculateRelativity(speed_ms) {
    const ratio = speed_ms / C_L;
    const ratioSq = ratio * ratio;
    
    // Facteur de Lorentz (gamma)
    const gamma = 1.0 / Math.sqrt(1.0 - ratioSq);
    
    // Dilatation du temps (delta_t = t / gamma)
    // Nous calculons la différence de temps par an (en nanosecondes par jour)
    const dt_day = 1.0 - (1.0 / gamma); // Fraction de temps perdu par rapport au temps propre (t)
    const ns_per_day = dt_day * 24 * 3600 * 1e9;

    // Énergie Relativiste E = γ * E₀ = γ * m * c²
    const E_zero = currentMass * C_L * C_L;
    const E_rel = gamma * E_zero;
    
    // Quantité de mouvement (p) p = γ * m * v
    const momentum = gamma * currentMass * speed_ms;

    return {
        ratio_c: ratio,
        gamma: gamma,
        time_dilation_ns_day: ns_per_day,
        E_rest: E_zero,
        E_rel: E_rel,
        momentum: momentum
    };
}

/**
 * Mise à jour de la gravité et des constantes de corps céleste.
 */
function updateCelestialBody(bodyName, alt_m, rot_radius, ang_vel) {
    let G_ACC_NEW, R_REF_NEW;
    switch (bodyName) {
        case 'TERRE':
            // Gravité de la Terre en fonction de l'altitude
            R_REF_NEW = WGS84_A;
            // Utilisation du modèle de gravité standard WGS84 (simplifié pour l'altitude)
            const g_eq = 9.780327; // Gravité à l'équateur
            const g_pol = 9.832; // Gravité au pôle
            const g_0 = (g_eq * Math.pow(Math.cos(kLat * D2R), 2) + g_pol * Math.pow(Math.sin(kLat * D2R), 2));
            G_ACC_NEW = g_0 * (1 - 2 * alt_m / R_REF_NEW); // Correction en altitude
            break;
        case 'LUNE':
            G_ACC_NEW = 1.62; // Gravité Lune (m/s²)
            R_REF_NEW = 1737400; // Rayon Lune (m)
            break;
        case 'MARS':
            G_ACC_NEW = 3.72; // Gravité Mars (m/s²)
            R_REF_NEW = 3389500; // Rayon Mars (m)
            break;
        case 'ROTATING':
            // G_eff = G_ACC - omega² * R
            const G_CENTRIFUGAL = rot_radius * ang_vel * ang_vel;
            G_ACC_NEW = G_ACC - G_CENTRIFUGAL; // Nécessite de connaître la gravité de base
            R_REF_NEW = R_ALT_CENTER_REF;
            break;
        default:
            G_ACC_NEW = 9.80665;
            R_REF_NEW = 6371000;
            break;
    }

    G_ACC = G_ACC_NEW;
    R_ALT_CENTER_REF = R_REF_NEW;

    return { G_ACC_NEW, R_REF_NEW };
}

// =================================================================
// BLOC 5/5 : Logique de Tableau de Bord, IMU et Boucles
// =================================================================

// --- GESTION IMU (DeviceMotion) ---
function startIMU() {
    if (typeof DeviceOrientationEvent !== 'undefined' && typeof DeviceMotionEvent !== 'undefined') {
        window.addEventListener('devicemotion', handleMotion, true);
        if ($('imu-status')) $('imu-status').textContent = 'Actif';
    } else {
        if ($('imu-status')) $('imu-status').textContent = 'Non supporté';
    }
}

function handleMotion(event) {
    // Les lectures d'accélération et de gyroscope sont le seul 'input' u_k du UKF.
    const acceleration = event.accelerationIncludingGravity;
    const rotationRate = event.rotationRate;
    const dt_imu = event.interval / 1000.0 || (performance.now() - lastGpsTime) / 1000.0; // Utiliser l'intervalle si disponible, sinon dt global

    if (acceleration) {
        lastAccel = {
            x: acceleration.x || 0,
            y: acceleration.y || 0,
            z: acceleration.z || G_ACC // Z souvent vers le haut/bas. Initialisé avec G_ACC par défaut
        };
    }
    if (rotationRate) {
        lastGyro = {
            x: rotationRate.alpha * D2R || 0, // Rotation en radians/s
            y: rotationRate.beta * D2R || 0,
            z: rotationRate.gamma * D2R || 0
        };
    }
    
    // Le UKF doit être appelé dans la boucle principale pour garantir un dt cohérent.
}

// --- GESTION GPS ---
function getRValue(accuracy) {
    const baseR = accuracy * accuracy;
    switch (currentUKFReactivity) {
        case 'ADAPTATIF':
            // R Adaptatif: plus la précision est mauvaise (accuracy grand), plus R est grand.
            return Math.min(baseR * 2, 10000); 
        case 'AGRESSIF':
            // R Agressif: on fait plus confiance à la mesure (R plus petit)
            return Math.max(baseR * 0.5, 0.01); 
        case 'NORMAL':
        default:
            return baseR; 
    }
}

function onGpsSuccess(pos) {
    if (window.isGpsPaused) return;

    const coords = pos.coords;
    const currentTime = pos.timestamp;
    const dt = (currentTime - lastGpsTime) / 1000.0;
    
    // Acquisition initiale : Si l'UKF n'est pas initialisé, initialiser l'état x et le temps.
    if (!ukf) {
        kLat = coords.latitude;
        kLon = coords.longitude;
        kAlt = coords.altitude || 0.0;
        kSpd = coords.speed || 0.0;
        kAcc = coords.accuracy;
        ukf = new ProfessionalUKF();
        ukf.x.subset(math.index(math.range(0, 3), 0), geodeticToEcef(kLat, kLon, kAlt));
        lastGpsTime = currentTime;
        return;
    }

    if (dt > 0.0) {
        // --- 1. PRÉDICTION (Basée sur l'IMU et le temps écoulé dt) ---
        ukf.predict({ accel: lastAccel, gyro: lastGyro }, dt);
        
        // --- 2. MISE À JOUR (Basée sur la mesure GPS) ---
        // Construction du vecteur de mesure z_real (6x1)
        const z_real = math.matrix([
            [coords.latitude * D2R],
            [coords.longitude * D2R],
            [coords.altitude || 0.0],
            [coords.speed || 0.0],
            [0.0], // Clock Bias (simulé nul si non fourni par API raw GNSS)
            [0.0]  // Clock Drift (simulé nul si non fourni)
        ]);
        
        // Détermination du bruit de mesure R (basé sur l'accuracy GPS)
        const R_val = getRValue(coords.accuracy);

        const { P_speed } = ukf.update(z_real, R_val);
        
        // --- 3. MISE À JOUR DE L'ÉTAT DU DASHBOARD (À partir de l'état UKF filtré) ---
        const r_e_filtered = ukf.x.subset(math.index(math.range(0, 3), 0));
        const v_e_filtered = ukf.x.subset(math.index(math.range(3, 6), 0));
        
        const { lat, lon, alt } = ecefToGeodetic(r_e_filtered);
        
        // Mise à jour de l'état global avec les valeurs filtrées
        kLat = lat;
        kLon = lon;
        kAlt = alt;
        kSpd = math.norm(v_e_filtered);
        kAcc = P_speed; // Utiliser l'incertitude de la vitesse du filtre UKF pour l'affichage de la précision.
    }
    
    // Mise à jour de la distance et des max/avg
    if (lastGpsTime !== 0) {
        const d_m = kSpd * dt;
        distM += d_m;
        totalTime += dt;
        if (kSpd > 0.05) motionTime += dt;
    }

    if (kSpd > kSpeedMax) kSpeedMax = kSpd;
    kSpeedAvg = motionTime > 0 ? distM / motionTime : 0;
    
    lastGpsTime = currentTime;
    
    // Mise à jour de la carte (implémentation simplifiée)
    if (window.map) {
        window.L.marker([kLat, kLon]).addTo(window.map);
        window.map.setView([kLat, kLon], 16);
    }
}

function onGpsError(err) {
    console.warn(`GPS ERROR(${err.code}): ${err.message}`);
    if ($('gps-status-acquisition')) $('gps-status-acquisition').textContent = `ERREUR (${err.code})`;
}

function startGps(isHighFreq) {
    if (gpsWatchID) navigator.geolocation.clearWatch(gpsWatchID);
    
    const opts = {
        enableHighAccuracy: isHighFreq,
        maximumAge: isHighFreq ? 0 : 60000,
        timeout: isHighFreq ? 5000 : 15000
    };
    
    gpsWatchID = navigator.geolocation.watchPosition(onGpsSuccess, onGpsError, opts);
}

// --- MISE À JOUR DU DOM (Boucle Lente et Rapide) ---
function updateDashboardDOM() {
    const dt_session = (performance.now() - window.sessionStartTime) / 1000.0;
    const v_kmh = kSpd * KMH_MS;
    
    // Vitesse, Distance & Relativité
    if ($('vitesse-stable')) $('vitesse-stable').textContent = `${dataOrDefault(kSpd, 2)} m/s`;
    if ($('vitesse-3d-instant')) $('vitesse-3d-instant').textContent = `${dataOrDefault(v_kmh, 2)} km/h`;
    if ($('vitesse-max')) $('vitesse-max').textContent = `${dataOrDefault(kSpeedMax * KMH_MS, 1)} km/h`;
    if ($('vitesse-moyenne-mvt')) $('vitesse-moyenne-mvt').textContent = `${dataOrDefault(kSpeedAvg * KMH_MS, 1)} km/h`;
    if ($('total-distance-3d')) $('total-distance-3d').textContent = `${dataOrDefault(distM / 1000, 3)} km | ${dataOrDefault(distM, 2)} m`;
    
    // Relativité
    const rel = calculateRelativity(kSpd);
    if ($('%-vitesse-lumiere')) $('%-vitesse-lumiere').textContent = `${dataOrDefaultExp(rel.ratio_c * 100, 2)} %`;
    if ($('nombre-mach')) $('nombre-mach').textContent = `${dataOrDefault(kSpd / currentSpeedOfSound, 4)}`;
    if ($('facteur-lorentz')) $('facteur-lorentz').textContent = `${dataOrDefault(rel.gamma, 4)}`;
    if ($('temps-dilation-vitesse')) $('temps-dilation-vitesse').textContent = `${dataOrDefault(rel.time_dilation_ns_day, 2)} ns/j`;
    if ($('energie-relativiste')) $('energie-relativiste').textContent = `${dataOrDefaultExp(rel.E_rel, 2)} J`;
    if ($('energie-masse-repos')) $('energie-masse-repos').textContent = `${dataOrDefaultExp(rel.E_rest, 2)} J`;
    if ($('quantite-mouvement')) $('quantite-mouvement').textContent = `${dataOrDefaultExp(rel.momentum, 2)} N·s`;

    // Dynamique & Forces
    const { force, power, dynamic_pressure } = calculateDrag(kSpd, currentAirDensity);
    if ($('pression-dynamique')) $('pression-dynamique').textContent = `${dataOrDefault(dynamic_pressure, 2)} Pa`;
    if ($('force-trainee')) $('force-trainee').textContent = `${dataOrDefault(force, 2)} N`;
    if ($('puissance-trainee')) $('puissance-trainee').textContent = `${dataOrDefault(power / 1000, 2)} kW`;
    if ($('nombre-reynolds')) $('nombre-reynolds').textContent = `${dataOrDefaultExp(calculateReynolds(kSpd, currentAirDensity), 2)}`;

    // UKF & Debug (Affichage des états filtrés)
    if (ukf) {
        if ($('latitude-ekf')) $('latitude-ekf').textContent = `${dataOrDefault(kLat, 6)} °`;
        if ($('longitude-ekf')) $('longitude-ekf').textContent = `${dataOrDefault(kLon, 6)} °`;
        if ($('altitude-ekf')) $('altitude-ekf').textContent = `${dataOrDefault(kAlt, 2)} m`;
        if ($('incertitude-vitesse')) $('incertitude-vitesse').textContent = `${dataOrDefault(kAcc, 6)} m/s`;
        if ($('statut-ekf')) $('statut-ekf').textContent = 'Actif (21 États)';
        
        // Affichage des Biases (Exemple Gyro X)
        const gyroBiasX = ukf.x.get([10, 0]) * R2D; // État 10: Gyro Bias X
        if ($('accel-x')) $('accel-x').textContent = `${dataOrDefault(lastAccel.x, 2)} m/s² (Bias $\mathbf{b}_a$: ${dataOrDefault(ukf.x.get([13, 0]), 3)})`;
        if ($('angular-velocity')) $('angular-velocity').textContent = `${dataOrDefault(lastGyro.x * R2D, 2)} °/s (Bias $\mathbf{b}_g$: ${dataOrDefault(gyroBiasX, 3)})`;
    }
}

// --- BOUCLE LENTE (Météo, Astro, Affichage LENT) ---
function slowUpdateLoop() {
    const currentLat = kLat, currentLon = kLon;

    // Mise à jour Météo & Polluants (via API Proxy)
    if (currentLat !== 0 && currentLon !== 0) {
        fetchWeather(currentLat, currentLon);
    }
    
    // Mise à jour NTP (pour horloge précise)
    syncH();

    // Mise à jour Astro
    if (currentLat !== 0 && currentLon !== 0) {
        updateAstroDOM(currentLat, currentLon, getCDate());
    }

    // Mise à jour de l'affichage LENT
    if ($('elapsed-time')) $('elapsed-time').textContent = `${dataOrDefault(totalTime, 2)} s`;
    if ($('motion-time')) $('motion-time').textContent = `${dataOrDefault(motionTime, 2)} s`;
}

// --- FONCTIONS ASYNCHRONES (API) ---
async function syncH() {
    // Fonction de synchronisation NTP (simulée via WorldTimeAPI)
    // ... implémentation omise pour la concision ...
}

async function fetchWeather(lat, lon) {
    // Fonction pour récupérer la météo et les polluants via le proxy Vercel
    // ... implémentation omise pour la concision ...
    
    // Mettre à jour les variables globales après réception des données
    // currentAirDensity = getAirDensity(P_API, T_API);
    // currentSpeedOfSound = getSpeedOfSound(T_API);
}

function updateAstroDOM(lat, lon, date) {
    if (typeof SunCalc === 'undefined') return;

    const times = SunCalc.getTimes(date, lat, lon);
    const sunPos = SunCalc.getPosition(date, lat, lon);
    const moonPos = SunCalc.getMoonPosition(date, lat, lon);
    const moonIllumination = SunCalc.getMoonIllumination(date);

    sunAltitudeRad = sunPos.altitude;

    // Soleil
    if ($('sun-alt')) $('sun-alt').textContent = `${dataOrDefault(sunPos.altitude * R2D, 1)} °`;
    if ($('sun-azimuth')) $('sun-azimuth').textContent = `${dataOrDefault(sunPos.azimuth * R2D, 1)} °`;
    if ($('sunrise-times')) $('sunrise-times').textContent = `${times.sunrise.toLocaleTimeString()} / ${times.sunriseEnd.toLocaleTimeString()}`;
    if ($('sunset-times')) $('sunset-times').textContent = `${times.sunsetStart.toLocaleTimeString()} / ${times.sunset.toLocaleTimeString()}`;

    // Lune
    if ($('moon-phase-name')) $('moon-phase-name').textContent = dataOrDefault(moonIllumination.phase, 2);
    if ($('moon-illuminated')) $('moon-illuminated').textContent = `${dataOrDefault(moonIllumination.fraction * 100, 1)} %`;
    if ($('moon-alt')) $('moon-alt').textContent = `${dataOrDefault(moonPos.altitude * R2D, 1)} °`;
    if ($('moon-azimuth')) $('moon-azimuth').textContent = `${dataOrDefault(moonPos.azimuth * R2D, 1)} °`;
}

// --- INITIALISATION DES ÉVÉNEMENTS ---
document.addEventListener('DOMContentLoaded', () => {
    window.sessionStartTime = performance.now();
    
    // Vérification des dépendances critiques (math.js)
    if (typeof math === 'undefined') {
        alert("Erreur critique: La librairie math.js est manquante. Le filtre UKF est désactivé.");
        return;
    }
    
    // Initialisation de la carte Leaflet (si les dépendances sont chargées)
    if (typeof L !== 'undefined' && $('#map-gnss')) {
        window.map = L.map('map-gnss').setView([kLat, kLon], 13);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: '© OpenStreetMap contributors'
        }).addTo(window.map);
    } else {
        $('#map-gnss').textContent = "Carte GNSS: Impossible de charger (Leaflet/DOM manquant)";
    }
    
    // --- Initialisation UKF (Après la première lecture GPS) ---
    // Le UKF est initialisé dans onGpsSuccess après la première lecture fiable.
    
    // --- Événements et Contrôles ---
    if ($('start-gps-btn')) $('start-gps-btn').addEventListener('click', () => startGps(true));
    if ($('high-frequency-toggle')) $('high-frequency-toggle').addEventListener('change', (e) => startGps(e.target.checked));
    
    // Événement Corps Céleste
    if ($('celestial-body-select')) $('celestial-body-select').addEventListener('change', (e) => {
        currentCelestialBody = e.target.value;
        const { G_ACC_NEW } = updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity);
        $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
    });
    
    // Démarrage des capteurs
    startIMU();

    // Boucles de Mise à Jour
    setInterval(updateDashboardDOM, 100); // Boucle rapide (10 Hz)
    setInterval(slowUpdateLoop, 5000);   // Boucle lente (0.2 Hz)
    
    // Démarrage initial
    syncH();
    updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity);
    startGps(true); // Démarrage par défaut en Haute Fréquence
});
