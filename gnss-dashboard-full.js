// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// CORRIGÉ : Encapsulation (IIFE) et Robustesse de Formatage.
// Dépendances (doivent être chargées dans l'HTML) : Leaflet, turf, suncalc, mathjs
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES (hors IIFE, car souvent utilisées par les scripts inclus) ---
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
        // Crée une chaîne de zéros dynamiques pour respecter 'decimals' (ex: '0.000e+0' pour decimals=3)
        const zeroDecimals = '0.' + Array(decimals).fill('0').join('');
        return zeroDecimals + 'e+0' + suffix;
    }
    return val.toExponential(decimals) + suffix;
};

// =================================================================
// DÉMARRAGE : Encapsulation de la logique UKF et État Global (IIFE)
// =================================================================

((window) => {

    // Vérifie si la bibliothèque math.js est chargée
    if (typeof math === 'undefined') {
        console.error("Erreur: La bibliothèque 'math.js' n'est pas chargée. Le UKF ne peut pas fonctionner.");
        return;
    }
    
    // --- CLÉS D'API & ENDPOINTS ---
    // Note: Utilisation d'un proxy Vercel ou d'une API locale pour contourner les restrictions CORS
    const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
    const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
    const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

    // --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES FONDAMENTALES ---
    const D2R = Math.PI / 180, R2D = 180 / Math.PI;
    const KMH_MS = 3.6;         
    const C_L = 299792458;      // Vitesse de la lumière (m/s)
    const C_S_STD = 343;        // Vitesse du son standard (m/s)
    const G_U = 6.67430e-11;    // Constante gravitationnelle universelle (N·m²/kg²)
    const R_SPECIFIC_AIR = 287.058; // Constante spécifique de l'air sec (J/kg·K)
    const GAMMA_AIR = 1.4;      // Indice adiabatique de l'air
    const MU_DYNAMIC_AIR = 1.8e-5; // Viscosité dynamique de l'air (Pa·s)

    // --- CONSTANTES GÉOPHYSIQUES (WGS84) ---
    const G_EARTH = 9.80665;         // Gravité standard à la surface (m/s²)
    const R_E_BASE = 6371000;      // Rayon terrestre moyen (m)
    const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
    const M_EARTH = 5.972e24;       // Masse de la Terre (kg)

    // --- CONSTANTES DE TEMPS & CALENDRIER ---
    const MC_DAY_MS = 72 * 60 * 1000; // Durée d'un jour Minecraft en ms
    const J1970 = 2440588;          // Jour Julien pour 1970-01-01
    const DOM_SLOW_UPDATE_MS = 2000; // Fréquence de rafraîchissement lent (ms)

    // --- CONSTANTES GPS & KALMAN ---
    const NETHER_RATIO = 1 / 8; 
    const MIN_DT = 0.05;        // Durée minimale entre deux mises à jour GPS (s)
    const MAX_ACC = 200;        // Précision max (m) avant "Estimation Seule"
    const MIN_SPD = 0.05;       // Vitesse minimale "en mouvement" (m/s)
    
    // --- UKF PARAMÈTRES ET DIMENSIONS ---
    const N_STATES = 21; // Position(3), Vitesse(3), Attitude(3), Biais Acc(3), Biais Gyro(3), Biais Mag(3), Erreur Environnement(3)
    const N_SIGMA = 2 * N_STATES + 1; // Nombre de points Sigma
    const ALPHA = 1e-3, BETA = 2, KAPPA = 0; // Paramètres de l'UKF

    // --- ÉTATS GLOBALES (Variables d'état et filtres) ---
    let map = null;
    let lastMarker = null;
    let path = [];
    let watchID = null;
    let emergencyStopActive = false;
    let gpsRunning = false;
    let sTime = Date.now();
    let lastTime = 0;
    let timeMoving = 0;
    let distTotal = 0;
    let speedMax = 0;
    let totalSpeedMS = 0;
    let speedUpdates = 0;
    let domID = null;
    let weatherID = null;

    // État Physique
    let currentMass = 70.0;
    let currentCelestialBody = 'EARTH';
    let netherMode = false;
    let lastAlt = 0;
    let lat = 0, lon = 0; // Dernière position GPS brute
    let kAlt = 0; // Altitude estimée par Kalman
    let currentAirDensity = 1.225;
    let currentSpeedOfSound = C_S_STD;
    let sunAltitudeRad = 0;

    // Données Météo/Sync
    let lastP_hPa = 1013.25; // Pression de référence
    let lastT_K = 288.15;    // Température de référence
    let lastH_perc = 0.5;    // Humidité de référence
    let lServH = 0;
    let lLocH = 0;

    // --- VARIABLES UKF (À INITIALISER) ---
    let ukfState = math.matrix(math.zeros(N_STATES)); // Vecteur d'état (x)
    // Initialisation de la Matrice de Covariance (P)
    let P_UKF_COV = math.diag(math.zeros(N_STATES).map(v => 1e-4)); 
    let Q_PROCESS = math.diag(math.zeros(N_STATES).map((v, i) => i < 6 ? 1e-3 : 1e-5)); // Bruit de Processus
    let R_GPS = math.diag(math.zeros(3).map(v => 5.0)); // Bruit de Mesure GPS (Lat, Lon, Alt)
    let currentUKFReactivity = 'AUTO';


    // =================================================================
    // BLOC 2/4 : Fonctions du Filtre de Kalman Non-Parfumé (UKF)
    // =================================================================

    /**
     * Calcule les poids pour la moyenne et la covariance (W_m et W_c).
     */
    const getUKFWeights = () => {
        const lambda = ALPHA * ALPHA * (N_STATES + KAPPA) - N_STATES;
        const W_m = math.zeros(N_SIGMA);
        const W_c = math.zeros(N_SIGMA);
        
        // Poids pour le premier point Sigma (le point moyen)
        W_m.set([0], lambda / (N_STATES + lambda));
        W_c.set([0], lambda / (N_STATES + lambda) + (1 - ALPHA * ALPHA + BETA));

        // Poids pour les autres 2*N_STATES points
        const W_rest = 1 / (2 * (N_STATES + lambda));
        for (let i = 1; i < N_SIGMA; i++) {
            W_m.set([i], W_rest);
            W_c.set([i], W_rest);
        }
        return { W_m, W_c, lambda };
    };

    /**
     * Génère les points Sigma à partir de l'état (x) et de la covariance (P).
     * @param {math.Matrix} x - Vecteur d'état (21x1).
     * @param {math.Matrix} P - Matrice de covariance (21x21).
     * @returns {math.Matrix} Les 2N+1 points Sigma (21 x N_SIGMA).
     */
    const generateSigmaPoints = (x, P) => {
        // sqrtP = Cholesky( (N_STATES + lambda) * P )
        const { lambda } = getUKFWeights();
        const factor = Math.sqrt(N_STATES + lambda);
        const sqrtP = math.multiply(math.cholesky(P), factor); // Matrice triangulaire
        
        const sigmaPoints = math.zeros(N_STATES, N_SIGMA);

        // 1. Point moyen (sigmaPoints[0] = x)
        sigmaPoints.set(math.range(0, N_STATES), 0, x); 

        // 2. et 3. Les paires de points (x +/- sqrtP[i])
        for (let i = 0; i < N_STATES; i++) {
            const sqrtP_i = math.column(sqrtP, i);
            
            // Point positif (indice i+1)
            sigmaPoints.set(math.range(0, N_STATES), i + 1, math.add(x, sqrtP_i));
            
            // Point négatif (indice i+1+N_STATES)
            sigmaPoints.set(math.range(0, N_STATES), i + 1 + N_STATES, math.subtract(x, sqrtP_i));
        }

        return sigmaPoints;
    };

    /**
     * Fonction de prédiction du modèle d'état non-linéaire (f).
     * @param {math.Matrix} x - L'état actuel [pos, vel, att, biases...].
     * @param {number} dt - Intervalle de temps (s).
     * @param {math.Matrix} u - Vecteur de contrôle (Accélération, Taux de rotation de l'IMU).
     * @returns {math.Matrix} Le nouvel état prédit.
     */
    const f = (x, dt, u) => {
        // Implémentation du modèle de mouvement cinématique pour les 21 états
        // (Position, Vitesse, Attitude, Biases)
        // Ceci est une simplification pour l'exemple. La version réelle est très longue.
        
        const pos = math.subset(x, math.index(math.range(0, 3))); // x[0:2]
        const vel = math.subset(x, math.index(math.range(3, 6))); // x[3:5]
        // const accBias = math.subset(x, math.index(math.range(9, 12))); // x[9:11]

        // Nouvelle Position: P_k+1 = P_k + V_k * dt + 0.5 * A_k * dt^2
        const new_pos = math.add(pos, math.multiply(vel, dt));
        // Simplification de la Nouvelle Vitesse: V_k+1 = V_k + A_k * dt
        // Dans un vrai UKF, cela inclurait la gravité, Coriolis, et la rotation.
        const new_vel = math.add(vel, math.multiply(u, dt)); 

        const x_new = x.clone(); // Cloner l'état précédent pour les états non mis à jour
        x_new.set(math.range(0, 3), new_pos);
        x_new.set(math.range(3, 6), new_vel);
        
        return x_new;
    };

    /**
     * Fonction de mesure non-linéaire (h) - Transforme l'état en mesures GPS.
     * @param {math.Matrix} x - L'état prédit.
     * @returns {math.Matrix} La mesure GPS prédite [Lat, Lon, Alt].
     */
    const h = (x) => {
        // L'UKF estime les coordonnées ECEF (x, y, z) en interne.
        // La fonction h doit convertir l'état ECEF de x en Lat/Lon/Alt
        // L'état UKF [0:2] = Position (probablement ECEF)
        
        // Ceci est la mesure prédite par le modèle.
        const pos_ecef = math.subset(x, math.index(math.range(0, 3)));
        
        // SIMPLIFICATION : Conversion ECEF -> GPS (Lat, Lon, Alt)
        // Pour cet exemple, on suppose que l'état [0:2] est déjà [Lat, Lon, Alt]
        // C'est une simplification courante pour les exemples de démonstration.
        return pos_ecef; // [Lat, Lon, Alt] prédits
    };

    /**
     * Étape de PRÉDICTION UKF (Propagation des points Sigma et de la Covariance).
     * @param {math.Matrix} x - État précédent.
     * @param {math.Matrix} P - Covariance précédente.
     * @param {number} dt - Intervalle de temps.
     * @param {math.Matrix} u - Vecteur de contrôle (IMU/Forces).
     * @returns {{x_pred: math.Matrix, P_pred: math.Matrix}} État et Covariance prédits.
     */
    const predictUKF = (x, P, dt, u) => {
        const { W_m, W_c } = getUKFWeights();
        
        // 1. Générer les points Sigma
        const sigmaPoints = generateSigmaPoints(x, P);
        
        // 2. Propager chaque point Sigma à travers la fonction de transition d'état f
        const sigmaPoints_pred = math.zeros(N_STATES, N_SIGMA);
        for (let i = 0; i < N_SIGMA; i++) {
            const x_i = math.column(sigmaPoints, i);
            sigmaPoints_pred.set(math.range(0, N_STATES), i, f(x_i, dt, u));
        }
        
        // 3. Reconstruire l'état prédit (x_pred) à partir des points propagés
        let x_pred = math.zeros(N_STATES);
        for (let i = 0; i < N_SIGMA; i++) {
            x_pred = math.add(x_pred, math.multiply(math.column(sigmaPoints_pred, i), W_m.get([i])));
        }
        
        // 4. Reconstruire la covariance prédite (P_pred)
        let P_pred = Q_PROCESS.clone(); // P_pred = Q + Σ W_c[i] * (σ[i] - x_pred) * (σ[i] - x_pred)^T
        for (let i = 0; i < N_SIGMA; i++) {
            const sigma_i = math.column(sigmaPoints_pred, i);
            const deviation = math.subtract(sigma_i, x_pred);
            const cov_i = math.multiply(deviation, math.transpose(deviation));
            P_pred = math.add(P_pred, math.multiply(cov_i, W_c.get([i])));
        }

        return { x_pred, P_pred };
    };

    /**
     * Étape de MISE À JOUR UKF (Incorporation de la Mesure GPS).
     * @param {math.Matrix} x_pred - État prédit.
     * @param {math.Matrix} P_pred - Covariance prédite.
     * @param {math.Matrix} z_gps - Mesure GPS [Lat, Lon, Alt].
     * @returns {{x_new: math.Matrix, P_new: math.Matrix}} Nouvel état et Covariance mis à jour.
     */
    const updateUKF = (x_pred, P_pred, z_gps) => {
        const { W_m, W_c } = getUKFWeights();
        
        // 1. Régénérer les points Sigma (sur l'état prédit)
        const sigmaPoints_pred = generateSigmaPoints(x_pred, P_pred);
        
        // 2. Propager les points Sigma à travers la fonction de mesure h
        const sigmaZ = math.zeros(3, N_SIGMA); // 3 dimensions de mesure : Lat, Lon, Alt
        for (let i = 0; i < N_SIGMA; i++) {
            const x_i = math.column(sigmaPoints_pred, i);
            sigmaZ.set(math.index(math.range(0, 3), i), h(x_i));
        }

        // 3. Reconstruire la moyenne prédite des mesures (z_pred)
        let z_pred = math.zeros(3);
        for (let i = 0; i < N_SIGMA; i++) {
            z_pred = math.add(z_pred, math.multiply(math.column(sigmaZ, i), W_m.get([i])));
        }

        // 4. Calculer la Covariance de l'innovation (Pzz) et la Covariance croisée (Pxz)
        let Pzz = R_GPS.clone(); // Pzz = R + Σ W_c[i] * (Z[i] - z_pred) * (Z[i] - z_pred)^T
        let Pxz = math.zeros(N_STATES, 3); // Pxz = Σ W_c[i] * (X[i] - x_pred) * (Z[i] - z_pred)^T

        for (let i = 0; i < N_SIGMA; i++) {
            const z_i = math.column(sigmaZ, i);
            const x_i = math.column(sigmaPoints_pred, i);
            
            const innovationZ = math.subtract(z_i, z_pred);
            const innovationX = math.subtract(x_i, x_pred);
            
            Pzz = math.add(Pzz, math.multiply(innovationZ, math.transpose(innovationZ), W_c.get([i])));
            Pxz = math.add(Pxz, math.multiply(innovationX, math.transpose(innovationZ), W_c.get([i])));
        }

        // 5. Calculer le Gain de Kalman (K) : K = Pxz * Pzz⁻¹
        const K = math.multiply(Pxz, math.inv(Pzz));

        // 6. Mettre à jour l'état (x_new) et la covariance (P_new)
        const y = math.subtract(z_gps, z_pred); // Innovation
        const x_new = math.add(x_pred, math.multiply(K, y));
        const P_new = math.subtract(P_pred, math.multiply(K, Pzz, math.transpose(K)));

        // Stocker la matrice K pour l'affichage de debug
        if ($('kalman-gain')) $('kalman-gain').textContent = `${K.get([0, 0]).toExponential(2)} (K[0,0])`;

        return { x_new, P_new };
    };

    // =================================================================
    // BLOC 3/4 : Fonctions de Calcul Physique, Météo & Astro
    // =================================================================

    const getGravityLocal = (altM) => {
        // Gravité locale g = G_U * M_Terre / (R_Terre + alt)^2
        const G_ACC_BASE = G_U * M_EARTH / Math.pow(R_E_BASE, 2);
        return G_ACC_BASE * Math.pow(R_E_BASE / (R_E_BASE + altM), 2);
    };
    
    const updateCelestialBody = (body, altM, rotationRadius = 100, angularVelocity = 0.0) => {
        let G_ACC_NEW = G_EARTH;
        
        switch (body) {
            case 'MOON': 
                G_ACC_NEW = 1.625; 
                R_ALT_CENTER_REF = 1737400; 
                break;
            case 'MARS': 
                G_ACC_NEW = 3.721; 
                R_ALT_CENTER_REF = 3389500; 
                break;
            case 'ROTATING': // Gravité modifiée par la force centrifuge (pour simulation)
                const centrifugalAcc = angularVelocity * angularVelocity * rotationRadius;
                G_ACC_NEW = G_EARTH - centrifugalAcc;
                break;
            case 'EARTH':
            default:
                G_ACC_NEW = getGravityLocal(altM); 
                R_ALT_CENTER_REF = R_E_BASE;
                break;
        }
        
        // Mise à jour de la gravité globale pour les calculs physiques
        window.G_ACC = G_ACC_NEW; 
        
        return { G_ACC_NEW };
    };

    /**
     * Calcule la densité de l'air (rho) en fonction de Pression (Pa), Température (K) et Humidité (H).
     * @param {number} P_Pa - Pression atmosphérique en Pascal.
     * @param {number} T_K - Température en Kelvin.
     * @param {number} H_perc - Humidité relative (0.0 à 1.0).
     * @returns {number} Densité de l'air en kg/m³.
     */
    const calculateAirDensity = (P_Pa, T_K, H_perc) => {
        // Formule de l'air humide (simplifiée)
        const R_V = 461.5; // Constante spécifique de la vapeur d'eau (J/kg·K)
        
        // Pression de saturation de la vapeur d'eau (formule de Magnus-Tetens, simplifiée)
        const T_C = T_K - 273.15;
        const P_sat = 6.1078 * Math.pow(10, (7.5 * T_C) / (T_C + 237.3)) * 100; // en Pa

        const P_v = H_perc * P_sat; // Pression partielle de la vapeur d'eau
        const P_d = P_Pa - P_v;     // Pression partielle de l'air sec
        
        // Densité: rho = (P_d / (R_SPECIFIC_AIR * T_K)) + (P_v / (R_V * T_K))
        return (P_d / (R_SPECIFIC_AIR * T_K)) + (P_v / (R_V * T_K));
    };

    const getSpeedOfSound = (tempK) => {
        // Vitesse du son c = sqrt(gamma * R_specifique * T)
        return Math.sqrt(GAMMA_AIR * R_SPECIFIC_AIR * tempK);
    };

    /**
     * Tente de synchroniser l'heure locale avec l'heure du serveur (NTP Like).
     */
    // gnss-dashboard-full.js

function syncH() {
    lLocH = Date.now();
    
    // Tente de se connecter au serveur NTP
    fetch(SERVER_TIME_ENDPOINT)
    .then(res => {
        // CORRECTION: Assurer que l'état du réseau est vérifié
        if (!res.ok) throw new Error("Erreur de statut réseau ou hors ligne"); 
        return res.json();
    })
    .then(data => {
        const serverTimeMs = new Date(data.utc_datetime).getTime();
        lServH = serverTimeMs;
        if ($('local-time')) $('local-time').textContent = 'Synchronisé';
    })
    .catch(err => {
        // FALLBACK HORS LIGNE
        lServH = lLocH; // Utilise l'heure locale comme référence
        if ($('local-time')) $('local-time').textContent = 'SYNCHRO HORS LIGNE ⚠️'; 
        console.warn("Synchronisation de l'heure échouée. Utilisation de l'heure système locale.");
        
        // CORRECTION CRITIQUE: Si getCDate utilise lServH et lLocH, il faut s'assurer qu'ils sont initialisés. C'est fait ici.
    });
          }

    /**
     * Obtient l'heure synchronisée en utilisant l'offset.
     * @returns {Date | null} Date synchronisée.
     */
    function getCDate() {
        if (lServH === 0) return new Date(); // Retourne l'heure locale si non synchronisé
        const offset = lServH - lLocH;
        return new Date(Date.now() + offset);
    }

    /**
     * Récupère les données météorologiques de l'API externe.
     */
    function fetchWeather(lat, lon) {
        return fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`)
            .then(res => {
                if (!res.ok) throw new Error(`API Météo indisponible: ${res.status}`);
                return res.json();
            })
            .then(data => {
                const tempK = data.tempC + 273.15;
                const airDensity = calculateAirDensity(data.pressure_hPa * 100, tempK, data.humidity_perc / 100.0);
                
                // Calcul du point de rosée
                const A = 17.27, B = 237.7;
                const alpha = ((A * data.tempC) / (B + data.tempC)) + Math.log(data.humidity_perc / 100.0);
                const dewPoint = (B * alpha) / (A - alpha);
                
                return { 
                    ...data, 
                    tempK, 
                    air_density: airDensity,
                    dew_point: dewPoint
                };
            });
    }

    /**
     * Met à jour les données astronomiques (Soleil/Lune)
     */
    function updateAstro(lat, lon) {
        const now = getCDate();
        if (!now) return;
        
        const times = SunCalc.getTimes(now, lat, lon);
        const pos = SunCalc.getPosition(now, lat, lon);
        const moonPos = SunCalc.getMoonPosition(now, lat, lon);
        const moonIllumination = SunCalc.getMoonIllumination(now);
        
        sunAltitudeRad = pos.altitude; // Stockage de l'altitude du soleil pour les calculs bio
        
        // Mise à jour DOM
        if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(pos.altitude * R2D, 2) + ' °';
        if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(pos.azimuth * R2D, 2) + ' °';
        if ($('moon-alt')) $('moon-alt').textContent = dataOrDefault(moonPos.altitude * R2D, 2) + ' °';
        if ($('moon-azimuth')) $('moon-azimuth').textContent = dataOrDefault(moonPos.azimuth * R2D, 2) + ' °';
        if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(moonIllumination.fraction * 100, 1) + ' %';
        
        // ... (Autres mises à jour astro) ...
    }


    // =================================================================
    // BLOC 4/4 : Logique GPS, IMU, DOM & Initialisation
    // =================================================================

        @param {GeolocationPosition} pos
    
function updateGPSData(pos) {
    const currentLat = pos.coords.latitude;
    const currentLon = pos.coords.longitude;
    // Utilise l'altitude estimée par UKF (kAlt) si l'altitude GPS est manquante (courant)
    const currentAlt = pos.coords.altitude || kAlt; 
    const currentAcc = pos.coords.accuracy || 10;
    const currentSpd = pos.coords.speed || 0; // Vitesse 2D (m/s)

    const dt = (pos.timestamp - lastTime) / 1000;
    if (dt < MIN_DT) {
        // console.log("Trop rapide:", dt.toFixed(3), "s"); // Décommenter pour debug
        return;
    }
    lastTime = pos.timestamp;

    // Mise à jour de la distance parcourue et des statistiques (nécessite turf.js)
    if (lat !== 0 && lon !== 0) {
        const from = turf.point([lastLatLon.lon, lastLatLon.lat]);
        const to = turf.point([currentLon, currentLat]);
        // Calcul de la distance 2D en mètres
        const distance = turf.distance(from, to, { units: 'meters' });
        distTotal += distance;
        
        if (currentSpd >= MIN_SPD) {
            timeMoving += dt;
            totalSpeedMS += currentSpd;
            speedUpdates++;
        }
    }
    
    // --- MISE À JOUR UKF (Fusion) ---
    // Le vecteur de mesure Z (observable) est [Lat, Lon, Alt]
    const z_gps = math.matrix([currentLat, currentLon, currentAlt]); 
    // Vecteur de contrôle U (IMU/Accélération) - Simulée à zéro ici.
    const u_imu = math.matrix(math.zeros(3)); 
    
    if (ukfState.get([0]) === 0) { // Initialisation du UKF
        ukfState.set(math.range(0, 3), z_gps); // Initialise la position
        if ($('ukf-status')) $('ukf-status').textContent = 'INITIALISÉ';
    } else {
        // Étape 1 : PRÉDICTION
        const { x_pred, P_pred } = predictUKF(ukfState, P_UKF_COV, dt, u_imu);
        
        // Étape 2 : MISE À JOUR (si la précision GPS est acceptable)
        if (currentAcc < MAX_ACC && currentSpd > 0) {
            const { x_new, P_new } = updateUKF(x_pred, P_pred, z_gps);
            ukfState = x_new;
            P_UKF_COV = P_new;
            if ($('ukf-status')) $('ukf-status').textContent = 'FUSION';
        } else {
            // Seulement PRÉDICTION si les données GPS sont mauvaises ou manquantes
            ukfState = x_pred;
            P_UKF_COV = P_pred;
            if ($('ukf-status')) $('ukf-status').textContent = 'PRÉDICTION';
        }
    }
    
    // Extraction des états UKF pour l'affichage (Indices 0 à 5)
    const ukfStateArray = ukfState.toArray();
    const [kLat, kLon, kAltEst] = ukfStateArray.slice(0, 3);
    const [kSpeedX, kSpeedY, kSpeedZ] = ukfStateArray.slice(3, 6);
    const kSpeed3D = Math.sqrt(kSpeedX*kSpeedX + kSpeedY*kSpeedY + kSpeedZ*kSpeedZ);
    kAlt = kAltEst; // Met à jour l'altitude globale UKF
    
    // --- MISE À JOUR DE L'AFFICHAGE ---
    updateSpeedDisplay(currentSpd, kSpeed3D); 
    
    // Mise à jour de la carte (utilise la position UKF pour un tracé plus fluide)
    if (map) {
        const newPoint = [kLat, kLon];
        map.setView(newPoint, map.getZoom());
        if (lastMarker) map.removeLayer(lastMarker);
        lastMarker = L.marker(newPoint).addTo(map).bindPopup("Position UKF").openPopup();
        
        // Limiter la taille du chemin pour des raisons de performance
        if (path.length > 500) path.shift(); 
        path.push(newPoint);
        L.polyline(path, { color: 'cyan' }).addTo(map);
    }

    // Mise à jour des variables d'état GPS brutes et des statistiques
    lat = currentLat;
    lon = currentLon;
    lastLatLon = { lat: currentLat, lon: currentLon };
    speedMax = Math.max(speedMax, kSpeed3D);

    // Affichage des données Lat/Lon/Alt UKF
    if ($('lat-display')) $('lat-display').textContent = dataOrDefault(kLat, 6) + ' °';
    if ($('lon-display')) $('lon-display').textContent = dataOrDefault(kLon, 6) + ' °';
    if ($('alt-display')) $('alt-display').textContent = dataOrDefault(kAlt, 2) + ' m';
    
    // Affichage de l'incertitude de la position Z (indice 2 de la matrice de covariance P)
    if ($('ukf-alt-uncert')) $('ukf-alt-uncert').textContent = dataOrDefault(P_UKF_COV.get([2, 2]), 3, ' m²');
    // Affichage de l'incertitude de la vitesse X (indice 3 de P)
    if ($('ukf-speed-uncert')) $('ukf-speed-uncert').textContent = dataOrDefault(P_UKF_COV.get([3, 3]), 3, ' (m/s)²');
}

/**
 * Met à jour l'affichage de la vitesse, de la dynamique et les données relativistes.
 */
function updateSpeedDisplay(currentSpd, kSpeed3D) {
    const speedKMH = kSpeed3D * KMH_MS;
    const percSpeedC = (kSpeed3D / C_L) * 100;
    const percSpeedSound = (kSpeed3D / currentSpeedOfSound) * 100;
    
    if ($('speed-display-ms')) $('speed-display-ms').textContent = dataOrDefault(kSpeed3D, 2) + ' m/s';
    if ($('speed-display-kmh')) $('speed-display-kmh').textContent = dataOrDefault(speedKMH, 2) + ' km/h';
    if ($('speed-max')) $('speed-max').textContent = dataOrDefault(speedMax * KMH_MS, 5) + ' km/h';

    // Relativité
    if ($('perc-light')) $('perc-light').textContent = dataOrDefaultExp(percSpeedC, 2, ' %');
    if ($('perc-sound')) $('perc-sound').textContent = dataOrDefault(percSpeedSound, 2, ' %');

    // Dynamique
    const dragForce = 0.5 * currentAirDensity * kSpeed3D * kSpeed3D * 0.8 * 1.0; 
    const dragPower = dragForce * kSpeed3D;
    if ($('drag-power-kw')) $('drag-power-kw').textContent = dataOrDefault(dragPower / 1000, 2) + ' kW';
    
    // Énergie Cinétique E = 0.5 * m * v^2
    const kineticEnergy = 0.5 * currentMass * kSpeed3D * kSpeed3D;
    if ($('kinetic-energy')) $('kinetic-energy').textContent = dataOrDefault(kineticEnergy / 1000, 2) + ' kJ';
}

/**
 * Démarre la géolocalisation.
 */
function startGPS() {
    if (watchID === null) {
        gpsRunning = true;
        const options = { enableHighAccuracy: true, timeout: 5000, maximumAge: 0 };
        watchID = navigator.geolocation.watchPosition(updateGPSData, (error) => {
            console.error("Erreur GPS:", error.message);
            if ($('ukf-status')) $('ukf-status').textContent = `❌ GPS ÉCHOUÉ (${error.code})`;
        }, options);
        if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = '⏸️ PAUSE GPS';
    }
}

/**
 * Arrête la géolocalisation.
 */
function stopGPS() {
    if (watchID !== null) {
        gpsRunning = false;
        navigator.geolocation.clearWatch(watchID);
        watchID = null;
        if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = '▶️ MARCHE GPS';
    }
}

// --- ÉVÉNEMENTS & INITIALISATION DE LA PAGE ---
document.addEventListener('DOMContentLoaded', () => {
    
    // Initialisation Leaflet et Carte
    map = L.map('map').setView([43.296, 5.37], 13);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19 }).addTo(map);

    // --- LISTENERS ---
    if ($('toggle-gps-btn')) {
        $('toggle-gps-btn').addEventListener('click', () => {
            if (gpsRunning) stopGPS(); else startGPS();
        });
    }
    
    // Listener pour la masse
    if ($('mass-input')) {
        $('mass-input').addEventListener('input', (e) => {
            currentMass = parseFloat(e.target.value) || 70.0;
            if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        });
    }
    
    // Listener pour le corps céleste
    if ($('celestial-body-select')) {
        $('celestial-body-select').addEventListener('change', (e) => {
            currentCelestialBody = e.target.value;
            const { G_ACC_NEW } = updateCelestialBody(currentCelestialBody, kAlt || 0); 
            if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
        });
    }
    
    // Listener pour le mode Nether
    if ($('nether-toggle-btn')) {
        $('nether-toggle-btn').addEventListener('click', () => {
            netherMode = !netherMode;
            $('nether-indicator').textContent = netherMode ? 'ACTIVÉ (1:8)' : 'DÉSACTIVÉ (1:1)';
            $('nether-toggle-btn').textContent = `Mode Nether: ${netherMode ? 'ACTIVÉ' : 'DÉSACTIVÉ'}`;
        });
    }
    
    // --- DÉMARRAGE DU SYSTÈME ---
    syncH(); 
    updateCelestialBody(currentCelestialBody, 0); // Initialise la gravité
    startGPS(); 

    // Boucle de mise à jour lente (Astro/Météo/Horloge/Temps de parcours)
    if (domID === null) {
        domID = setInterval(() => {
            const now = getCDate();
            if (now) {
                // Temps local synchronisé
                if ($('local-time') && !$('local-time').textContent.includes('Synchronisation...')) {
                    $('local-time').textContent = now.toLocaleTimeString('fr-FR');
                }
                if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
                
                // Temps écoulé
                if ($('time-elapsed')) $('time-elapsed').textContent = sTime ? ((now.getTime() - sTime) / 1000).toFixed(2) + ' s' : '0.00 s';
                if ($('time-moving')) $('time-moving').textContent = timeMoving.toFixed(2) + ' s';
                if ($('distance-total-km')) $('distance-total-km').textContent = `${dataOrDefault(distTotal / 1000, 3)} km`;
                
                // Temps Minecraft (Calcul du cycle jour/nuit)
                const mcTimeMs = now.getTime() % MC_DAY_MS;
                const totalTicks = Math.floor(mcTimeMs / 50); 
                const h = Math.floor((totalTicks / 1000) % 24);
                const m = Math.floor((totalTicks / 16.666) % 60);
                const s = Math.floor((totalTicks % 16.666) * 0.06);
                if ($('mc-time')) $('mc-time').textContent = `${h.toString().padStart(2, '0')}:${m.toString().padStart(2, '0')}:${s.toString().padStart(2, '0')}`;
            }

            // Mise à jour Astro (utilise la dernière position UKF ou GPS brute)
            if (lat !== 0 && lon !== 0) updateAstro(lat, lon); 
            
            // Vérification de la synchronisation de l'heure toutes les minutes
            if (Math.floor(Date.now() / 1000) % 60 === 0) {
                syncH();
            }

            // Lancement de la mise à jour Météo (si on a une position valide)
            if (lat !== 0 && lon !== 0 && !emergencyStopActive) {
                fetchWeather(lat, lon).then(data => {
                    if (data) {
                        // Stocke les valeurs pour les calculs UKF/Dynamique
                        lastP_hPa = data.pressure_hPa;
                        lastT_K = data.tempK;
                        lastH_perc = data.humidity_perc / 100.0;
                        currentAirDensity = data.air_density;
                        currentSpeedOfSound = getSpeedOfSound(data.tempK);
                        
                        // Met à jour le DOM météo
                        if ($('weather-status')) $('weather-status').textContent = `ACTIF`;
                        if ($('temp-air-2')) $('temp-air-2').textContent = `${data.tempC.toFixed(1)} °C`;
                        if ($('pressure-2')) $('pressure-2').textContent = `${data.pressure_hPa.toFixed(0)} hPa`;
                        if ($('humidity-2')) $('humidity-2').textContent = `${data.humidity_perc.toFixed(0)} %`;
                        if ($('air-density')) $('air-density').textContent = `${data.air_density.toFixed(3)} kg/m³`;
                        if ($('dew-point')) $('dew-point').textContent = `${data.dew_point.toFixed(1)} °C`;
                    }
                }).catch(err => {
                    if ($('weather-status')) $('weather-status').textContent = `❌ API ÉCHOUÉE`;
                    console.error("Erreur Fetch Météo:", err);
                });
            }
        }, DOM_SLOW_UPDATE_MS); 
    }
});    
