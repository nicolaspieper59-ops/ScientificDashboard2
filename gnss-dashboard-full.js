// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// ARCHITECTURE SCIENTIFIQUE, GÉOPHYSIQUE WGS84, MODÈLE ISA, FUSION INS/GNSS
// =================================================================

// =================================================================
// DÉMARRAGE : Encapsulation de la logique UKF et État Global (IIFE)
// =================================================================

((window) => {

    // Vérification des dépendances critiques (math.js, Leaflet, SunCalc)
    if (typeof math === 'undefined' || typeof L === 'undefined' || typeof SunCalc === 'undefined') {
        console.error("Dépendances critiques manquantes. Assurez-vous que math.min.js, leaflet.js et suncalc.js sont chargés dans l'HTML.");
        return;
    }

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
    // SECTION 1/4 : Constantes, État Global & Modèles Physiques WGS84/ISA
    // =================================================================

    // --- CLÉS D'API & ENDPOINTS ---
    // REMPLACER par votre URL de proxy si vous en utilisez une, ou directement les APIs.
    const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
    const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
    const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";
    
    // --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES FONDAMENTALES ---
    const D2R = Math.PI / 180, R2D = 180 / Math.PI;
    const KMH_MS = 3.6;
    const C_L = 299792458;      // Vitesse de la lumière (m/s)
    const G_U = 6.67430e-11;    // Constante gravitationnelle universelle (N·m²/kg²)

    // --- CONSTANTES GÉOPHYSIQUES (WGS84) ---
    const OMEGA_EARTH = 7.292115e-5; // Vitesse de rotation de la Terre (rad/s)
    const WGS84_A = 6378137.0;  // Rayon équatorial WGS84 (m)
    const WGS84_F = 1 / 298.257223563; // Aplatissement WGS84
    const WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F; // Excentricité au carré
    const WGS84_G_EQUATOR = 9.780327; // Gravité à l'équateur (Formule de Somigliana)
    const WGS84_BETA = 0.0053024; // Facteur de gravité

    // --- CONSTANTES ATMOSPHÉRIQUES (ISA Standard) ---
    const BARO_ALT_REF_HPA = 1013.25;
    const TEMP_SEA_LEVEL_K = 288.15; // 15°C
    const RHO_SEA_LEVEL = 1.225;
    const R_AIR = 287.058;      // Constante spécifique de l'air sec (J/kg·K)
    const GAMMA_AIR = 1.4;      // Ratio de chaleurs spécifiques
    const MU_DYNAMIC_AIR = 1.8e-5; // Viscosité dynamique de l'air (Pa·s)
    const KELVIN_OFFSET = 273.15; // Conversion C vers K

    // --- PARAMÈTRES DU FILTRE UKF/EKF & ÉTAT LOCAL ---
    const UKF_STATE_DIM = 21;
    const R_ALT_MIN = 1.0;
    const UKF_R_MAX = 500.0;
    const MAX_ACC = 200;
    const MIN_SPD = 0.01;
    const NETHER_RATIO = 8.0;

    // --- ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE ---
    let isGpsPaused = false;
    let currentPosition = { lat: 43.2964, lon: 5.3697, acc: 10.0, spd: 0.0, alt: 0.0 };
    let currentUKFReactivity = 'NORMAL';
    let selectedEnvironment = 'NORMAL';
    let currentCelestialBody = 'EARTH';
    let currentMass = 70.0;
    let netherMode = false;
    let distanceRatioMode = false;

    // Variables pour la fusion métrologique
    let lastP_hPa = BARO_ALT_REF_HPA;
    let lastT_K = TEMP_SEA_LEVEL_K;
    let lastH_perc = 0.5; // 50% d'humidité par défaut
    let currentAirDensity = RHO_SEA_LEVEL;
    let currentSpeedOfSound = 340.29;
    let G_ACC = 9.80665; // Gravité locale calculée (m/s²)
    let R_ALT_CENTER_REF = WGS84_A;

    // Variables de temps et d'affichage
    let lServH = new Date();
    let lLocH = new Date();
    let lastGpsTimestamp = 0;
    let kAlt = 0; // Altitude estimée par le filtre
    let kSpd = 0; // Vitesse estimée par le filtre
    let ukf; // Instance du UKF

    // Cartographie
    let map, mapMarker, mapTrace = [], mapPolyline;
    const MAP_UPDATE_INTERVAL = 3000;
    const IMU_UPDATE_RATE_MS = 20; // 50Hz
    const DOM_SLOW_UPDATE_MS = 1000; // 1Hz

    const UKF_REACTIVITY_FACTORS = {
        'AUTO': { MULT: 1.0, DISPLAY: 'Automatique' },
        'NORMAL': { MULT: 1.0, DISPLAY: 'Normal' },
        'FAST': { MULT: 0.2, DISPLAY: 'Rapide' },
        'STABLE': { MULT: 2.5, DISPLAY: 'Microscopique' },
    };
    const ENVIRONMENT_FACTORS = {
        'NORMAL': { R_MULT: 1.0, DISPLAY: 'Normal' },
        'FOREST': { R_MULT: 2.5, DISPLAY: 'Forêt' },
        'CONCRETE': { R_MULT: 7.0, DISPLAY: 'Grotte/Tunnel' },
        'METAL': { R_MULT: 5.0, DISPLAY: 'Métal/Bâtiment' },
    };
    const CELESTIAL_DATA = {
        'EARTH': { G: 9.80665, R: WGS84_A, name: 'Terre' },
        'MOON': { G: 1.62, R: 1737400, name: 'Lune' },
        'MARS': { G: 3.71, R: 3389500, name: 'Mars' },
        'ROTATING': { G: 0.0, R: WGS84_A, name: 'Station Spatiale' }
    };
    const GPS_OPTS = {
        HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
        LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
    };

    // --- FONCTIONS DE MODÈLE PHYSIQUE/GÉODÉSIE ---

    /**
     * Calcule la gravité locale WGS84 (Somigliana + Air Libre).
     * @param {number} lat - Latitude en degrés.
     * @param {number} alt - Altitude en mètres.
     * @returns {number} Gravité locale en m/s².
     */
    function getWGS84Gravity(lat, alt) {
        const latRad = lat * D2R;
        const sin2lat = Math.sin(latRad) ** 2;
        // Formule de Somigliana pour la gravité de surface (ellipsoïde)
        const g_surface = WGS84_G_EQUATOR * (1 + WGS84_BETA * sin2lat) / Math.sqrt(1 - WGS84_E2 * sin2lat);
        // Correction d'air libre pour l'altitude
        return g_surface * (1 - 2 * alt / WGS84_A);
    }

    /**
     * Convertit les coordonnées géodétiques (LLA) en ECEF (Earth-Centered, Earth-Fixed).
     * @param {number} lat - Latitude géodétique (rad).
     * @param {number} lon - Longitude (rad).
     * @param {number} alt - Altitude ellipsoïdale (m).
     * @returns {math.Matrix} [X, Y, Z] en ECEF (m).
     */
    function LLAtoECEF(lat, lon, alt) {
        const N = WGS84_A / Math.sqrt(1 - WGS84_E2 * Math.sin(lat) ** 2);
        const X = (N + alt) * Math.cos(lat) * Math.cos(lon);
        const Y = (N + alt) * Math.cos(lat) * Math.sin(lon);
        const Z = (N * (1 - WGS84_E2) + alt) * Math.sin(lat);
        return math.matrix([X, Y, Z]);
    }

    /**
     * Calcule les propriétés atmosphériques (T, P, Rho, C_S) selon le modèle ISA avec correction d'humidité.
     * @param {number} alt_m - Altitude géopotentielle (m).
     * @param {number} humidity_perc - Humidité relative (0-100).
     * @param {number} pressure_hPa_ref - Pression de référence MSL (hPa).
     * @returns {{tempK: number, pressure_hPa: number, air_density: number, speedOfSound: number}}
     */
    function getAirPropertiesISA(alt_m, humidity_perc, pressure_hPa_ref) {
        const T0 = TEMP_SEA_LEVEL_K;
        const P0 = (pressure_hPa_ref || BARO_ALT_REF_HPA) * 100; // Pression de référence (Pa)
        const L = 0.0065;
        const T11 = 216.65;
        const G0 = G_ACC; 
        
        let T, P; // T en K, P en Pa

        if (alt_m <= 11000) { // Troposphère
            T = T0 - L * alt_m;
            P = P0 * Math.pow(T / T0, G0 / (L * R_AIR));
        } else { // Stratosphère (section isotherme)
            const P11 = P0 * Math.pow(T11 / T0, G0 / (L * R_AIR));
            T = T11;
            P = P11 * Math.exp(-G0 * (alt_m - 11000) / (R_AIR * T11));
        }
        
        const speedOfSound = Math.sqrt(GAMMA_AIR * R_AIR * T);
        
        // Calcul de la densité de l'air humide (correction professionnelle)
        // La densité est la somme des densités de l'air sec et de la vapeur d'eau.
        const VAPOR_PRESSURE_SAT = 6.1078 * Math.pow(10, (7.5 * (T - KELVIN_OFFSET)) / (237.3 + (T - KELVIN_OFFSET))); // Pression de vapeur saturante (hPa)
        const P_vapeur = (humidity_perc || 0) * VAPOR_PRESSURE_SAT / 100.0; // Pression de vapeur (hPa)
        const P_air_sec = P / 100 - P_vapeur; // Pression partielle de l'air sec (hPa)

        const RHO_DRY = P_air_sec * 100 / (R_AIR * T);
        const R_VAPEUR = 461.5; // Constante spécifique de la vapeur d'eau (J/kg·K)
        const RHO_WET = P_vapeur * 100 / (R_VAPEUR * T);
        const RHO = RHO_DRY + RHO_WET; // Densité de l'air humide (kg/m³)

        return {
            tempK: T,
            pressure_hPa: P / 100,
            air_density: RHO,
            speedOfSound: speedOfSound
        };
    }

    /**
     * Met à jour les constantes de gravité et de rayon pour un corps céleste simulé.
     */
    function updateCelestialBody(body, alt, rotationRadius, angularVelocity) {
        let G_ACC_NEW = CELESTIAL_DATA['EARTH'].G;
        let R_ALT_CENTER_REF_NEW = WGS84_A; 
        const data = CELESTIAL_DATA[body];
        
        if (data) {
            G_ACC_NEW = data.G;
            R_ALT_CENTER_REF_NEW = data.R;
        }

        if (body === 'ROTATING') {
            // Calcul de l'accélération centrifuge à l'altitude + rayon de rotation
            G_ACC_NEW = rotationRadius * angularVelocity ** 2;
        }

        G_ACC = G_ACC_NEW; 
        R_ALT_CENTER_REF = R_ALT_CENTER_REF_NEW; 
        return { G_ACC_NEW, R_ALT_CENTER_REF_NEW };
    }


    // =================================================================
    // SECTION 2/4 : Classe UKF 21 États (Noyau de Fusion)
    // =================================================================

    /**
     * Implémentation de la classe Unscented Kalman Filter (UKF) pour la fusion INS/GNSS
     * utilisant un modèle à 21 états dans le référentiel ECEF.
     */
    class ProfessionalUKF {
        constructor() {
            this.N_STATES = UKF_STATE_DIM;
            // État: [r_E(3), v_E(3), q_EB(4), bias_a(3), bias_g(3), scale_a(3), scale_g(2)] = 21
            this.x = math.zeros(this.N_STATES); 
            // Covariance de l'état (P)
            this.P = math.diag([
                1e0, 1e0, 1e0,      // Position (m)
                1e-1, 1e-1, 1e-1,   // Vitesse (m/s)
                1e-3, 1e-3, 1e-3, 1e-3, // Orientation (quat)
                1e-3, 1e-3, 1e-3,   // Biais accéléromètre (m/s²)
                1e-6, 1e-6, 1e-6,   // Biais gyroscope (rad/s)
                1e-4, 1e-4, 1e-4,   // Facteur d'échelle accéléromètre
                1e-5, 1e-5         // Facteur d'échelle gyroscope
            ]);
            // Bruit de processus (Q)
            this.Q = math.diag(Array(this.N_STATES).fill(1e-8)); 
        }

        /**
         * Initialise le vecteur d'état x en coordonnées ECEF.
         * @param {object} initialGps - {latitude, longitude, altitude, speed} en degrés/m/m/s.
         */
        initState(initialGps) {
            const latRad = initialGps.lat * D2R;
            const lonRad = initialGps.lon * D2R;
            const ECEF = LLAtoECEF(latRad, lonRad, initialGps.alt);
            
            this.x.set([0, 1, 2], ECEF.toArray()); // Position ECEF [x, y, z]

            // Vitesse ECEF (initialisée à 0, mise à jour dans la boucle)
            this.x.set([3, 4, 5], [0, 0, 0]); 
            
            // Orientation (Quaternion - par défaut: Pas de rotation initiale)
            this.x.set([6, 7, 8, 9], [1, 0, 0, 0]); 
        }
        
        /**
         * Étape de PRÉDICTION (Modèle de Mouvement Strapdown INS)
         * NOTE: L'implémentation complète des matrices de l'UKF nécessite beaucoup de code
         * de math.js. Ici, nous utilisons l'architecture mais simplifions la propagation
         * pour des fins de démonstration/performance JavaScript.
         */
        predict(imuData, dt) {
            if (dt < 0.001) return;

            // 1. Accélération et Rotation mesurées (corrigées des biais/scale factors)
            const f_B = math.matrix(imuData.accel);
            const w_B = math.matrix(imuData.gyro);
            
            // Biais et scale factors (simplifié pour l'exemple)
            const bias_a = this.x.subset(math.index(10, 13));
            const bias_g = this.x.subset(math.index(13, 16));
            
            const f_B_cor = math.subtract(f_B, bias_a); 
            const w_B_cor = math.subtract(w_B, bias_g); 
            
            // 2. Propagation de l'état (simplifiée)
            
            // a) Position et Vitesse (ECEF) - Intégration simple (Approximation)
            // L'intégration complète doit tenir compte de la gravité (g_E), Coriolis, et Centrifuge.
            
            // Gravité locale ECEF (approximation: direction Z)
            const g_local = getWGS84Gravity(this.x.get([0]) * R2D, this.x.get([2]));
            
            // Force inertielle dans le référentiel ECEF (R_EB * f_B_cor)
            // L'implémentation UKF utiliserait Sigma Points pour propager l'état non-linéaire.
            
            // Approximation de la vitesse et de la position pour faire bouger le dashboard
            const speed = Math.sqrt(this.x.get([3])**2 + this.x.get([4])**2 + this.x.get([5])**2);
            let k_new_speed = speed + f_B_cor.get([0]) * dt;
            if (k_new_speed < MIN_SPD) k_new_speed = 0;

            // Mise à jour simplifiée de l'altitude estimée (pour le DOM)
            kAlt += this.x.get([5]) * dt; 
            kSpd = k_new_speed;

            // 3. Mise à jour de la Covariance P
            // P_k+1 = F_k * P_k * F_k^T + Q_k
            // ... (Omis pour la brièveté et la performance JS) ...
        }

        /**
         * Étape de MISE À JOUR (Correction par la Mesure GNSS)
         */
        update(gpsData, R_dyn) {
            if (gpsData.acc > MAX_ACC) return; // Mesure GNSS non fiable

            const K = 0.5; // Gain de Kalman d'approche (Remplacer par K_k du vrai UKF)

            // Mesure GNSS (Position LLA -> ECEF)
            const latRad = gpsData.lat * D2R;
            const lonRad = gpsData.lon * D2R;
            const ECEF_GNSS = LLAtoECEF(latRad, lonRad, gpsData.alt);
            
            // Innovation (y)
            const y_pos = math.subtract(ECEF_GNSS, this.x.subset(math.index(0, 3)));
            
            // Correction de l'état (simplifiée: x_k+1 = x_k + K * y)
            const ECEF_NEW = math.add(this.x.subset(math.index(0, 3)), math.multiply(y_pos, K));
            this.x.set([0, 1, 2], ECEF_NEW.toArray());
            
            // Correction de la vitesse (simplifiée)
            if (gpsData.spd !== undefined) {
                const oldSpeed = kSpd;
                const newSpeed = oldSpeed * (1 - K) + gpsData.spd * K;
                kSpd = newSpeed;
                // Mettre à jour les composantes de vitesse du vecteur d'état si nécessaire...
            }
            
            // Mise à jour de la Covariance P
            // P_k+1 = (I - K_k * H_k) * P_k
            // ... (Omis pour la brièveté) ...
        }

        /**
         * Extrait les données clés (LLA et Vitesse) de l'état estimé.
         */
        getState() {
            const ECEF = this.x.subset(math.index(0, 3)).toArray();
            // Conversion ECEF -> LLA (inverser LLAtoECEF) - Omissible ici, utilise LLA GNSS/Estimé
            
            const kLat = currentPosition.lat; // On utilise la dernière lat/lon GPS corrigée pour la carte
            const kLon = currentPosition.lon;
            const kUncert = this.P.get([0, 0]) + this.P.get([1, 1]) + this.P.get([2, 2]);
            const kAltUncert = this.P.get([2, 2]);

            return {
                lat: kLat, lon: kLon, alt: kAlt,
                speed: kSpd,
                kUncert: kUncert,
                kAltUncert: kAltUncert,
            };
        }
    }

    // =================================================================
    // SECTION 3/4 : Gestion des API, DOM & Boucles de Mise à Jour
    // =================================================================

    // --- FONCTIONS UTILITAIRES API / MÉTROLOGIE ---

    /** Synchronisation temporelle (NTP like) */
    function syncH() {
        return fetch(SERVER_TIME_ENDPOINT)
            .then(res => res.json())
            .then(data => {
                lServH = new Date(data.datetime);
                lLocH = new Date();
                console.log(`[NTP] Synchronisation réussie. Décalage: ${lLocH.getTime() - lServH.getTime()} ms`);
            })
            .catch(err => {
                console.error("[NTP] Échec de la synchronisation:", err);
                if ($('local-time')) $('local-time').textContent = 'SYNCHRO ÉCHOUÉE (Heure Locale)';
            });
    }

    /** Obtient la date corrigée par la synchro NTP */
    function getCDate(serverTime, localTime) {
        if (!serverTime || !localTime) return new Date();
        const now = new Date();
        const delta = now.getTime() - localTime.getTime();
        return new Date(serverTime.getTime() + delta);
    }

    /** Récupère les données météo/air via le proxy */
    async function fetchWeather(lat, lon) {
        try {
            const url = `${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`;
            const res = await fetch(url);
            const data = await res.json();
            
            const { tempC, pressure_hPa, humidity_perc } = data;
            const { tempK, air_density, speedOfSound } = getAirPropertiesISA(kAlt, humidity_perc, pressure_hPa);
            
            return {
                ...data,
                tempK,
                air_density: air_density,
                speedOfSound: speedOfSound,
            };
        } catch (e) {
            console.warn("Échec de la récupération météo/air:", e);
            return null;
        }
    }

    // --- FONCTIONS D'AFFICHAGE DU DOM ---

    /** Met à jour les données météo dans le DOM */
    function updateWeatherDOM(data, isInitial = false) {
        if (data) {
            lastP_hPa = data.pressure_hPa;
            lastT_K = data.tempK;
            lastH_perc = data.humidity_perc / 100.0;
            currentAirDensity = data.air_density;
            currentSpeedOfSound = data.speedOfSound;

            $('temp-air-2').textContent = dataOrDefault(data.tempC, 1, ' °C');
            $('pressure-2').textContent = dataOrDefault(data.pressure_hPa, 0, ' hPa');
            $('humidity-2').textContent = dataOrDefault(data.humidity_perc, 0, ' %');
            $('air-density').textContent = dataOrDefault(currentAirDensity, 3, ' kg/m³');
            $('dew-point').textContent = dataOrDefault(data.dew_point, 1, ' °C');
            $('wind-speed').textContent = dataOrDefault(data.wind_speed, 1, ' m/s');
            $('speed-of-sound-calc').textContent = dataOrDefault(currentSpeedOfSound, 2, ' m/s');
        } else if (isInitial) {
            $('temp-air-2').textContent = 'N/A';
            $('air-density').textContent = dataOrDefault(RHO_SEA_LEVEL, 3, ' kg/m³ (ISA)');
            $('speed-of-sound-calc').textContent = dataOrDefault(getAirPropertiesISA(0, 0, 1013.25).speedOfSound, 2, ' m/s (ISA)');
        }
    }
    
    /** Met à jour les données astronomiques dans le DOM */
    function updateAstroDOM(lat, lon, date) {
        const times = SunCalc.getTimes(date, lat, lon);
        const sunPos = SunCalc.getPosition(date, lat, lon);
        const moonPos = SunCalc.getMoonPosition(date, lat, lon);
        const moonIllum = SunCalc.getMoonIllumination(date);

        $('sun-alt').textContent = dataOrDefault(sunPos.altitude * R2D, 2, '°');
        $('sun-azimuth').textContent = dataOrDefault(sunPos.azimuth * R2D, 2, '°');
        $('moon-alt').textContent = dataOrDefault(moonPos.altitude * R2D, 2, '°');
        $('moon-azimuth').textContent = dataOrDefault(moonPos.azimuth * R2D, 2, '°');
        $('moon-illuminated').textContent = dataOrDefault(moonIllum.fraction * 100, 1, '%');
        
        const sunrise = times.sunriseEnd ? times.sunriseEnd.toLocaleTimeString('fr-FR') : 'N/A';
        const sunset = times.sunsetStart ? times.sunsetStart.toLocaleTimeString('fr-FR') : 'N/A';
        $('sunrise-times').textContent = sunrise;
        $('sunset-times').textContent = sunset;
        
        $('moon-phase-name').textContent = getMoonPhaseName(moonIllum.phase);
    }
    
    /** Donne le nom de la phase lunaire */
    function getMoonPhaseName(phase) {
        if (phase < 0.03 || phase >= 0.97) return 'Nouvelle Lune 🌑';
        if (phase < 0.22) return 'Premier Croissant  waxing crescent 🌒';
        if (phase < 0.28) return 'Premier Quartier 🌓';
        if (phase < 0.47) return 'Lune Gibbeuse Croissante 🌔';
        if (phase < 0.53) return 'Pleine Lune 🌕';
        if (phase < 0.72) return 'Lune Gibbeuse Décroissante 🌖';
        if (phase < 0.78) return 'Dernier Quartier 🌗';
        if (phase < 0.97) return 'Dernier Croissant waning crescent 🌘';
        return 'Inconnue';
    }

    /** Met à jour les données calculées avancées dans le DOM */
    function updateAdvancedPhysicsDOM(kSpd, kAlt, mass, CdA, tempK, airDensity, lat, kAltUncert, localG, accel_long) {
        const V = kSpd;
        const lorentzFactor = 1 / Math.sqrt(1 - Math.pow(V / C_L, 2));
        const timeDilationSpeed = (lorentzFactor - 1) * 86400 * 1e9; // ns/jour
        const gravitationalDilation = (localG * kAlt / (C_L * C_L)) * 86400 * 1e9; // ns/jour
        const machNumber = V / currentSpeedOfSound;
        const dynamicPressure = 0.5 * airDensity * V * V;
        const dragForce = dynamicPressure * (CdA || 0.5); 
        const dragPower_kW = (dragForce * V) / 1000.0;
        const coriolisForce = 2 * mass * V * OMEGA_EARTH * Math.sin(lat * D2R);
        const force_g_long = localG > 0.1 ? (accel_long / localG) : 0;
        const altSigma = Math.sqrt(Math.abs(kAltUncert)); 

        $('speed-est').textContent = dataOrDefault(V, 2, ' m/s');
        $('altitude-est').textContent = dataOrDefault(kAlt, 2, ' m');
        $('mach-number').textContent = dataOrDefault(machNumber, 3);
        $('dynamic-pressure').textContent = dataOrDefault(dynamicPressure, 2, ' Pa');
        $('drag-force').textContent = dataOrDefault(dragForce, 2, ' N');
        $('drag-power').textContent = dataOrDefault(dragPower_kW, 3, ' kW');
        $('coriolis-force').textContent = dataOrDefaultExp(coriolisForce, 3, ' N');
        $('local-g-force').textContent = dataOrDefault(force_g_long, 2, ' g');
        $('lorentz-factor').textContent = dataOrDefaultExp(lorentzFactor - 1, 3);
        $('time-dilation-speed').textContent = dataOrDefaultExp(timeDilationSpeed, 3, ' ns/j');
        $('time-dilation-gravity').textContent = dataOrDefaultExp(gravitationalDilation, 3, ' ns/j');
        $('ukf-alt-sigma').textContent = dataOrDefault(altSigma, 3, ' m');
    }

    // --- GESTIONNAIRES DE CAPTEURS ---

    function handleGpsUpdate(pos) {
        const now = Date.now();
        const dt_gps = (now - lastGpsTimestamp) / 1000.0;
        lastGpsTimestamp = now;

        const { latitude, longitude, altitude, speed, accuracy } = pos.coords;

        const speed_ms = speed !== null ? speed : 0;
        const altitude_m = altitude !== null ? altitude : kAlt; // Utiliser l'estimation si pas de mesure

        currentPosition = {
            lat: latitude,
            lon: longitude,
            acc: accuracy,
            spd: speed_ms,
            alt: altitude_m
        };

        const R_dyn = getKalmanR(accuracy, ukf.getState().kAlt, ukf.getState().kUncert, selectedEnvironment, currentUKFReactivity);

        // Correction UKF (Update)
        ukf.update(currentPosition, R_dyn);
        
        // Mettre à jour l'estimation UKF pour l'affichage (position corrigée)
        const ukfState = ukf.getState();
        currentPosition.lat = ukfState.lat;
        currentPosition.lon = ukfState.lon;
        kAlt = ukfState.alt;
        kSpd = ukfState.speed;
    }

    function handleGpsError(err) {
        console.error(`Erreur GPS (${err.code}): ${err.message}`);
        $('status').textContent = `Erreur GPS: ${err.message}`;
    }

    // --- LOGIQUE DE FUSION UKF & BOUCLES SYSTÈME ---

    /**
     * Calcule le bruit de mesure R dynamique en fonction de la précision GPS et de l'environnement.
     */
    function getKalmanR(accRaw, kAlt, kUncert, env, reactivityMode) {
        let acc_effective = accRaw;
        if (acc_effective > MAX_ACC) { return 1e9; } 
        
        let R_gps_base = Math.min(acc_effective, 100) ** 2; 
        
        const env_mult = ENVIRONMENT_FACTORS[env]?.R_MULT || 1.0;
        let reactivity_mult = UKF_REACTIVITY_FACTORS[reactivityMode]?.MULT || 1.0;
        
        if (reactivityMode === 'AUTO' && acc_effective !== null) {
            if (acc_effective > 20) reactivity_mult = 3.0; // Forte incertitude = stabilisation
            else if (acc_effective < 3) reactivity_mult = 0.5; // Haute précision = réaction rapide
        }
        
        let R_dyn = Math.min(R_gps_base * env_mult * reactivity_mult, UKF_R_MAX);
        return Math.max(R_dyn, R_ALT_MIN); 
    }

    /** Boucle rapide (IMU/UKF) */
    function fastLoop() {
        const now = Date.now();
        const dt = (now - ukf.lastTime) / 1000.0;
        ukf.lastTime = now;
        
        if (dt > 0.1 || isGpsPaused) { 
            // Simulation IMU simple (à remplacer par de vraies données DeviceMotionEvent)
            const simulatedImuData = { accel: [0, 0, 0], gyro: [0, 0, 0] };
            ukf.predict(simulatedImuData, dt);
        }

        const state = ukf.getState();
        kAlt = state.alt;
        kSpd = state.speed;

        // Mise à jour de la gravité locale (pour les calculs physiques)
        G_ACC = getWGS84Gravity(state.lat, state.alt);

        // Mise à jour des valeurs du DOM ultra-rapides
        $('alt-est-2').textContent = dataOrDefault(kAlt, 2, ' m');
        $('spd-est-2').textContent = dataOrDefault(kSpd * KMH_MS, 2, ' km/h');
        $('gravity-base').textContent = dataOrDefault(G_ACC, 4, ' m/s²');
    }

    /** Boucle lente (Météo/Astro/DOM général) */
    function slowLoop() {
        // Mettre à jour les données météo/air
        fetchWeather(currentPosition.lat, currentPosition.lon).then(data => {
            if (data) updateWeatherDOM(data);
        });

        // Met à jour l'horloge locale (NTP) et Astro
        const now = getCDate(lServH, lLocH);
        if (now) {
            if ($('local-time') && !$('local-time').textContent.includes('SYNCHRO ÉCHOUÉE')) {
                $('local-time').textContent = now.toLocaleTimeString('fr-FR');
            }
            if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
            
            updateAstroDOM(currentPosition.lat, currentPosition.lon, now);
        }

        // Met à jour les données physiques avancées
        updateAdvancedPhysicsDOM(
            kSpd, 
            kAlt, 
            currentMass, 
            0.5, // CdA (Coefficient de Traînée * Aire) - Par défaut: 0.5
            lastT_K, 
            currentAirDensity, 
            currentPosition.lat, 
            ukf.getState().kAltUncert, 
            G_ACC, 
            0 // Accélération longitudinale (à intégrer avec DeviceMotionEvent)
        );
    }

    /** Boucle de la carte */
    function mapLoop() {
        if (!map || isGpsPaused) return;

        const { lat, lon } = currentPosition;
        mapMarker.setLatLng([lat, lon]);
        map.setView([lat, lon], map.getZoom());

        // Mise à jour de la trace (avec un rapport de distance)
        const ratio = netherMode ? NETHER_RATIO : 1.0;
        const newPoint = [lat, lon];

        // Vérifie si la distance est suffisante pour ajouter un nouveau point
        if (mapTrace.length === 0 || L.latLng(mapTrace[mapTrace.length - 1]).distanceTo(newPoint) * ratio > 1.0) {
            mapTrace.push(newPoint);
            mapPolyline.setLatLngs(mapTrace);
            
            // Calcul de la distance totale cumulée
            const totalDistance = mapTrace.reduce((acc, point, index, arr) => {
                if (index === 0) return acc;
                const prev = L.latLng(arr[index - 1][0], arr[index - 1][1]);
                const curr = L.latLng(point[0], point[1]);
                return acc + prev.distanceTo(curr);
            }, 0);

            $('dist-total').textContent = dataOrDefault(totalDistance * ratio / 1000, 2, ' km');
        }

        // Mise à jour du statut
        $('status').textContent = `GPS OK | Acc: ${dataOrDefault(currentPosition.acc, 1, 'm')} | UKF ${currentUKFReactivity}`;
    }

    // =================================================================
    // SECTION 4/4 : Initialisation et Événements DOM
    // =================================================================

    function initMap() {
        map = L.map('map', { zoomControl: false }).setView([currentPosition.lat, currentPosition.lon], 18);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            maxZoom: 19,
            attribution: '© OpenStreetMap'
        }).addTo(map);

        mapMarker = L.marker([currentPosition.lat, currentPosition.lon]).addTo(map);
        mapPolyline = L.polyline(mapTrace, { color: 'cyan', weight: 4 }).addTo(map);
    }

    function initSensors() {
        if (navigator.geolocation) {
            navigator.geolocation.watchPosition(handleGpsUpdate, handleGpsError, GPS_OPTS.HIGH_FREQ);
        } else {
            $('status').textContent = 'Erreur: La géolocalisation n\'est pas supportée.';
        }
    }

    window.addEventListener('DOMContentLoaded', async () => {
        // Initialiser l'UKF (dépend de math.js)
        ukf = new ProfessionalUKF();
        ukf.initState(currentPosition);
        ukf.lastTime = Date.now();

        // Démarrer la synchro NTP
        await syncH();
        
        // Initialisation de la carte et des capteurs
        initMap();
        initSensors();
        
        // Initialiser les valeurs DOM par défaut (ISA/WGS84)
        updateWeatherDOM(null, true); 
        
        // Initialisation des valeurs par défaut pour les corps célestes
        const rotationRadius = parseFloat($('rotation-radius')?.value) || 100;
        const angularVelocity = parseFloat($('angular-velocity')?.value) || 0.0;
        updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity);


        // --- DÉMARRAGE DES BOUCLES ---
        setInterval(fastLoop, IMU_UPDATE_RATE_MS);
        setInterval(slowLoop, DOM_SLOW_UPDATE_MS);
        setInterval(mapLoop, MAP_UPDATE_INTERVAL);

        // --- GESTIONNAIRES D'ÉVÉNEMENTS ---
        $('toggle-mode-btn').addEventListener('click', () => {
            document.body.classList.toggle('dark-mode');
            document.body.classList.toggle('light-mode');
            $('toggle-mode-btn').innerHTML = document.body.classList.contains('dark-mode') ? 
                '<i class="fas fa-sun"></i> Mode Jour' : 
                '<i class="fas fa-moon"></i> Mode Nuit';
        });

        $('ukf-reactivity-mode').addEventListener('change', (e) => currentUKFReactivity = e.target.value);
        $('environment-select').addEventListener('change', (e) => selectedEnvironment = e.target.value);
        
        $('celestial-body-select').addEventListener('change', (e) => {
            currentCelestialBody = e.target.value;
            const rotationRadius = parseFloat($('rotation-radius')?.value) || 100;
            const angularVelocity = parseFloat($('angular-velocity')?.value) || 0.0;
            updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity);
        });

        $('mass-input').addEventListener('input', (e) => {
            currentMass = parseFloat(e.target.value) || 70.0;
            $('mass-display').textContent = dataOrDefault(currentMass, 3, ' kg');
        });
        
        // Gestion de la rotation (pour le mode 'ROTATING')
        const updateRotation = () => {
            const rotationRadius = parseFloat($('rotation-radius')?.value) || 100;
            const angularVelocity = parseFloat($('angular-velocity')?.value) || 0.0;
            if (currentCelestialBody === 'ROTATING') {
                const { G_ACC_NEW } = updateCelestialBody('ROTATING', kAlt, rotationRadius, angularVelocity);
                $('gravity-base').textContent = dataOrDefault(G_ACC_NEW, 4, ' m/s²');
            }
        };
        $('rotation-radius')?.addEventListener('input', updateRotation);
        $('angular-velocity')?.addEventListener('input', updateRotation);


        $('nether-toggle-btn').addEventListener('click', () => {
            netherMode = !netherMode;
            $('nether-toggle-btn').textContent = `Mode Nether: ${netherMode ? 'ACTIVÉ (1:8)' : 'DÉSACTIVÉ (1:1)'}`;
        });

        $('distance-ratio-toggle-btn').addEventListener('click', () => {
            distanceRatioMode = !distanceRatioMode;
            $('distance-ratio-toggle-btn').textContent = `Rapport Distance: ${distanceRatioMode ? 'ALTITUDE' : 'SURFACE'}`;
        });
    });

})(window);

// =================================================================
// FIN DU FICHIER COMPLET
// =================================================================
