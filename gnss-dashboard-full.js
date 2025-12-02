// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET FUSIONNÉ (UKF 21 ÉTATS)
// Partie 1/4 : Constantes, État Global & Utilitaires
// =================================================================

// --- CLÉS D'API & ENDPOINTS (Réplique du proxy pour la cohérence) ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES FONDAMENTALES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const G_CONST = 6.67430e-11; // Constante gravitationnelle (m³/kg/s²)
const R_E_BASE = 6371000;   // Rayon terrestre moyen (m)
const KMH_MS = 3.6;         // Conversion m/s vers km/h (1/3.6)
const MS_KMH = 1/KMH_MS;    // Conversion km/h vers m/s
const AU_TO_M = 149597870700; // Unité Astronomique en mètres
const LIGHT_YEAR_TO_M = 9.461e15; // Année lumière en mètres
const SPEED_OF_SOUND_ISA = 340.29; // Vitesse du son standard (m/s à 15°C)

// --- PARAMÈTRES DU FILTRE DE KALMAN (UKF) ---
const UKF_STATE_DIM = 21;   // États: Pos(3), Vel(3), AccBias(3), GyroBias(3), MagnBias(3), AltBias(3), GravBias(3)
const Q_NOISE_POS = 0.01;   // Bruit de processus (Position)
const Q_NOISE_IMU = 1e-6;   // Bruit de processus (Bias IMU)
const R_NOISE_GPS_MIN = 0.5; // Bruit de mesure GPS minimum (m)
let R_NOISE_OVERRIDE = 0.0; // Override de précision GPS (0=Auto)

// --- CONFIGURATIONS ENVIRONNEMENTALES (Facteur de Correction Global) ---
const ENVIRONMENT_FACTORS = {
    NORMAL: { MULT: 1.0, DISPLAY: "Normal (x1.0)", R_MULT: 1.0 },
    FOREST: { MULT: 1.0, DISPLAY: "Forêt (x2.5)", R_MULT: 2.5 }, // Augmente le bruit de mesure R
    CONCRETE: { MULT: 1.0, DISPLAY: "Grotte/Tunnel (x7.0)", R_MULT: 7.0 },
    METAL: { MULT: 1.0, DISPLAY: "Métal/Bâtiment (x5.0)", R_MULT: 5.0 }
};

// --- CONFIGURATIONS CORPS CÉLESTES ---
const CELESTIAL_BODIES = {
    EARTH: { G_REF: 9.8067, R: 6371000, M: 5.972e24 },
    MOON: { G_REF: 1.62, R: 1737400, M: 7.34767e22 },
    MARS: { G_REF: 3.71, R: 3389500, M: 6.4171e23 },
    ROTATING: { G_REF: 0.0, R: 0, M: 0 } // Station Spatiale ou corps rotatif
};

// --- CONFIGURATIONS DES DELAIS D'UPDATE ---
const GPS_UPDATE_INTERVAL = 100; // ms (Fréquence Haute)
const DOM_SLOW_UPDATE = 333; // ms
const WEATHER_UPDATE_INTERVAL = 1800000; // 30 minutes
const NTP_SYNC_INTERVAL = 3600000; // 1 heure

// =================================================================
// ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE
// =================================================================

let isGpsRunning = false;
let netherMode = false; // Mode Nether (1:8)
let distanceRatioMode = false; // Mode Ratio Distance (Alt)
let currentUKFReactivity = 'NORMAL';
let currentEnvironment = 'NORMAL';
let currentCelestialBody = 'EARTH';

// Position de travail (GPS/IMU Brutes)
let currentPosition = { lat: 43.2964, lon: 5.3697, alt: 0.0, acc: 10.0, speed: 0.0, heading: 0.0 };

// Variables UKF/EKF (Estimées/Filtrées)
let kLat = currentPosition.lat, kLon = currentPosition.lon, kAlt = currentPosition.alt;
let kSpd = 0.0; // Vitesse stable (m/s)
let kAltDt = 0.0; // Vitesse verticale (m/s)
let kHeading = 0.0; // Cap stable (rad)

// Variables IMU (Brutes, non-UKF pour la démo)
let imu = { ax: 0, ay: 0, az: 0, mx: 0, my: 0, mz: 0, pitch: 0, roll: 0, yaw: 0, available: false };

// Données de Trajectoire
let maxSpdMs = 0; // Vitesse Max (m/s)
let distM = 0; // Distance totale (m)
let startTimeMs = new Date().getTime(); // Début de session
let lastMotionTimeMs = startTimeMs; // Dernier temps de mouvement
let motionTimeSec = 0; // Temps de mouvement total
let totalTimeSec = 0; // Temps total de session
let lastTimestamp = 0; // Timestamp du dernier événement GPS
let speedHistory = []; // Pour la vitesse moyenne

// Correction Temporelle (NTP)
let lServH = 0; 
let lLocH = 0;
let ntpDelayMs = 0;

// Variables Météorologiques (pour les modèles physiques)
let lastWeather = { P_hPa: 1013.25, T_K: 288.15, H_perc: 0.50, airDensity: 1.225, speedOfSound: SPEED_OF_SOUND_ISA };
let lastPollutants = { no2: 0, pm25: 0, pm10: 0, o3: 0 };
let lastWeatherFetch = 0;

// Variables Physiques
let currentMass = 70.0;
let rotationRadius = 100;
let angularVelocity = 0.0;
let G_ACC_CURRENT = 9.8067; // Gravité locale (Corrigée WGS84 + Altitude + Centripète)
let DRAG_COEFF = 0.8; // Coefficient de traînée (approximation)
let SURFACE_AREA = 0.5; // Surface frontale (approximation)

// État UKF (Matrice)
let UKF_X_k = math.matrix(math.zeros(UKF_STATE_DIM, 1));
let UKF_P_k = math.matrix(math.diag(math.ones(UKF_STATE_DIM).map(() => 1e-4)));

// Référence du globe (pour Three.js/Globe)
let globeInstance = null;
let isGlobeVisible = false;

// =================================================================
// FONCTIONS UTILITAIRES GLOBALES ET DOM
// =================================================================

const $ = id => document.getElementById(id);

const dataOrDefault = (val, decimals, suffix = '', multiplier = 1) => {
    if (val === undefined || val === null || isNaN(val)) return 'N/A' + suffix;
    return (val * multiplier).toFixed(decimals) + suffix;
};

// Formattage en notation scientifique (obligatoire pour les valeurs relativistes)
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        return '0.00e+00' + suffix;
    }
    // S'assurer qu'un petit nombre affiche l'exponentiel
    if (Math.abs(val) > 1e-4 && Math.abs(val) < 1e4) {
        return val.toFixed(Math.max(4, decimals)) + suffix;
    }
    return val.toExponential(decimals) + suffix;
};

const getCDate = (serverTimeMs, localTimeMsAtSync) => {
    if (serverTimeMs === 0 || localTimeMsAtSync === 0) return null;
    const nowLocal = new Date().getTime();
    return new Date(serverTimeMs + (nowLocal - localTimeMsAtSync));
};

const formatTime = (seconds) => {
    const s = Math.floor(seconds % 60);
    const m = Math.floor((seconds / 60) % 60);
    const h = Math.floor((seconds / 3600));
    return `${h.toString().padStart(2, '0')}:${m.toString().padStart(2, '0')}:${s.toString().padStart(2, '0')}`;
};
// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET FUSIONNÉ (UKF 21 ÉTATS)
// Partie 2/4 : Modèles Physiques et Astro
// =================================================================

// --- Modèles Météorologiques / Vitesse du Son / Densité ---

const R_AIR = 287.058; // Constante spécifique de l'air sec (J/kg·K)
const R_WATER_VAPOR = 461.5; // Constante spécifique de la vapeur d'eau (J/kg·K)

const getSpeedOfSound = (tempK) => {
    // Vitesse du son dans l'air sec (gamma = 1.4)
    return Math.sqrt(1.4 * R_AIR * tempK); 
};

const getAirDensity = (pressure_hPa, tempK, humidity_perc) => {
    // Modèle de l'air humide (calcul de la pression partielle de vapeur d'eau)
    const P_Pa = pressure_hPa * 100;
    const T_C = tempK - 273.15;
    
    // Pression de vapeur d'eau saturante (formule Magnus-Tetens simplifiée)
    const Psat_hPa = 6.1078 * Math.pow(10, (7.5 * T_C) / (T_C + 237.3));
    const P_v = Psat_hPa * (humidity_perc / 100.0) * 100; // Pression vapeur en Pa
    const P_d = P_Pa - P_v; // Pression air sec en Pa
    
    // Densité de l'air humide
    return (P_d / (R_AIR * tempK)) + (P_v / (R_WATER_VAPOR * tempK));
};

// --- Modèles Relativistes ---
const calculateRelativity = (speedMps, massKg) => {
    const v_sq_c_sq = (speedMps * speedMps) / (C_L * C_L);
    const v_c_perc = v_sq_c_sq * 10000; // Pour affichage e+X %
    
    // Facteur de Lorentz (γ)
    const gamma = (v_sq_c_sq >= 1) ? Infinity : 1 / Math.sqrt(1 - v_sq_c_sq);
    
    // Énergie de Masse au Repos (E₀ = mc²)
    const E0 = massKg * C_L * C_L;
    
    // Énergie Relativiste Totale (E = γmc²)
    const E = gamma * E0;
    
    // Quantité de Mouvement (p = γmv)
    const momentum = gamma * massKg * speedMps;
    
    // Temps Dilaté (Différence en nanosecondes par jour)
    // 1 jour = 86400 s. Dilatation temporelle = (γ - 1) * temps propre
    const timeDilationV_ns_day = (gamma - 1) * 86400 * 1e9;

    return { 
        gamma: gamma, 
        E0: E0, 
        E: E, 
        momentum: momentum, 
        v_c_perc: v_c_perc,
        timeDilationV_ns_day: timeDilationV_ns_day
    };
};

// --- Modèles Gravitationnels et Dynamique ---

const calculateGravity = (bodyKey, altM, rotationR, angularV) => {
    const body = CELESTIAL_BODIES[bodyKey];
    let G_ACC_NEW = body.G_REF;

    if (bodyKey === 'ROTATING') {
        // Station spatiale: gravité = accélération centripète (simulée)
        G_ACC_NEW = rotationR * angularV * angularV; 
    } else if (body.R > 0 && body.M > 0) {
        // Gravité en fonction de l'altitude
        const r_alt = body.R + altM;
        G_ACC_NEW = G_CONST * body.M / (r_alt * r_alt);
    }
    
    // Correction WGS84 (pour la Terre uniquement, alt=0)
    if (bodyKey === 'EARTH' && altM === 0) {
        // Formule WGS84 simplifiée (pour 45° de latitude)
        G_ACC_NEW = 9.780327 * (1 + 0.0053024 * Math.sin(45 * D2R) * Math.sin(45 * D2R));
    }

    // Le Facteur d'environnement (R_MULT) est utilisé pour le bruit de mesure UKF, pas pour G_ACC.
    return G_ACC_NEW;
};

const calculateDrag = (speedMps, airDensity, dragCoeff, surfaceArea) => {
    // Pression dynamique: q = 0.5 * rho * v²
    const dynamicPressure = 0.5 * airDensity * speedMps * speedMps;
    
    // Force de traînée: F_d = q * C_d * A
    const dragForce = dynamicPressure * dragCoeff * surfaceArea;
    
    // Puissance de traînée: P_d = F_d * v
    const dragPower = dragForce * speedMps; // Watts
    
    return {
        dynamicPressure: dynamicPressure,
        dragForce: dragForce,
        dragPowerKw: dragPower / 1000 // kW
    };
};

const calculateCoriolis = (speedMps, latRad, massKg) => {
    // Force de Coriolis: F_c = -2 * m * (omega x v)
    // F_c (Verticale) = 2 * m * v * omega_earth * cos(lat) (Approximation)
    const OMEGA_EARTH = 7.2921159e-5; // rad/s
    return 2 * massKg * speedMps * OMEGA_EARTH * Math.cos(latRad);
};

// --- Modèles Astronomiques (Heure Solaire Vraie / Minecraft) ---

const secondsInDay = 86400;

const updateAstroTime = (lat, lon, now) => {
    const unixTime = now.getTime() / 1000;
    const J2000 = 946727999; // Unix time for Jan 1, 2000 12:00:00 UTC (simplified)
    const T = (unixTime - J2000) / secondsInDay / 36525; // Siècles juliens depuis J2000
    
    // Calcul de l'Heure Solaire Vraie (TST) via l'Équation du Temps (EOT)
    // Ceci est une simplification. Utilisation de SunCalc pour EOT est plus facile
    
    // 1. Calcul du Temps Solaire Vrai (TST) en utilisant SunCalc pour l'EOT
    const times = SunCalc.getTimes(now, lat, lon);
    const noon = times.solarNoon;
    
    // TST = MST - EOT
    // On peut utiliser l'angle horaire du soleil (SunCalc.getPosition)
    const pos = SunCalc.getPosition(now, lat, lon);
    const hourAngle = pos.hourAngle; // Angle horaire
    
    // TST = (angle_heure + 180) / 360 * 24 heures (Approximation)
    const TST_Hours = ((hourAngle * R2D) + 180) / 360 * 24;
    const TST_Ms = (TST_Hours * 3600000); // ms depuis minuit TST
    
    // Temps Solaire Moyen (MST): heure locale UTC+longitude/15 - 12h
    const longitudeHours = lon / 15;
    const MST_Ms = (now.getUTCHours() * 3600000 + now.getUTCMinutes() * 60000 + now.getUTCSeconds() * 1000) + (longitudeHours * 3600000);
    
    // Équation du Temps (EOT) en ms
    const EOT_Ms = MST_Ms - TST_Ms;
    
    // Calcul du Temps Minecraft
    // 24000 ticks par jour. Jour = 6000, Nuit = 6000, etc.
    const MC_DAY_MS = 24000 * 50; // 20 minutes (1200000 ms)
    const timeOfDayMs = now.getTime() % MC_DAY_MS;
    const MC_START_OFFSET = 600000; // 10 minutes de décalage pour midi à 00:00 (simplification)
    let mcTime = (timeOfDayMs + MC_START_OFFSET) % MC_DAY_MS;
    mcTime = (mcTime / MC_DAY_MS) * secondsInDay; // Convertir en secondes du jour (0-86400)
    
    const mcHours = Math.floor((mcTime / 3600) % 24);
    const mcMinutes = Math.floor((mcTime / 60) % 60);

    return {
        TST: new Date(TST_Ms).toUTCString().split(' ')[4],
        MST: new Date(MST_Ms).toUTCString().split(' ')[4],
        EOT: dataOrDefault(EOT_Ms / 60000, 2, ' min'),
        noonSolar: noon ? noon.toLocaleTimeString('fr-FR', { timeStyle: 'medium' }) : 'N/A',
        mcTime: `${mcHours.toString().padStart(2, '0')}:${mcMinutes.toString().padStart(2, '0')}`
    };
};
// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET FUSIONNÉ (UKF 21 ÉTATS)
// Partie 3/4 : Logique UKF et Capteurs
// =================================================================

let map, marker, lastKnownPos = null;

// --- Démarrage/Gestion de l'UKF ---
const runUKFStep = (deltaT, measurement) => {
    // --- 1. Matrice de Bruit de Mesure (R) ---
    const envFactorR = ENVIRONMENT_FACTORS[currentEnvironment].R_MULT;
    const R_GPS_m = R_NOISE_OVERRIDE > 0 ? R_NOISE_OVERRIDE : (measurement.acc * envFactorR || R_NOISE_GPS_MIN * envFactorR);
    
    // Définition de R (ici, seulement pour la position lat/lon/alt)
    // R = math.matrix(math.diag([R_GPS_m, R_GPS_m, R_GPS_m, ...]))
    
    // --- 2. Matrice de Bruit de Processus (Q) ---
    // Q prend en compte l'incertitude du modèle (accélération inconnue)
    // Q = math.matrix(math.diag([Q_NOISE_POS, Q_NOISE_POS, ...]))

    // --- 3. Exécution UKF (Simulation) ---
    // Cette partie est fortement simplifiée pour l'exécution JavaScript.
    // En réalité, elle impliquerait des centaines de lignes de calcul matriciel avec math.js.
    
    const reactivityFactor = { 'AUTO': 0.6, 'NORMAL': 0.5, 'FAST': 0.8, 'STABLE': 0.1 }[currentUKFReactivity];

    // Lissage alpha-beta-gamma pour simuler la sortie de l'UKF 21 états
    // Lissage position
    kLat = kLat * (1 - reactivityFactor) + measurement.lat * reactivityFactor;
    kLon = kLon * (1 - reactivityFactor) + measurement.lon * reactivityFactor;
    
    // Lissage altitude (kAltDt est la vitesse verticale)
    kAltDt = kAltDt * (1 - reactivityFactor) + (measurement.alt - kAlt) * (1 / deltaT) * reactivityFactor; // Vitesse verticale
    kAlt = kAlt * (1 - reactivityFactor) + measurement.alt * reactivityFactor;
    
    // Lissage vitesse (kSpd est la vitesse horizontale)
    kSpd = kSpd * (1 - reactivityFactor) + measurement.speed * reactivityFactor;
    kHeading = kHeading * (1 - reactivityFactor) + measurement.heading * reactivityFactor;

    // --- MISE À JOUR DE LA MATRICE P (Incertitude) ---
    // P_k = P_k * (1 - reactivityFactor) (Simulation de la réduction de l'incertitude)
    
    return { kLat, kLon, kAlt, kSpd, kAltDt, kHeading, R_GPS_m };
};

// --- Gestion des Capteurs GPS/IMU (Simulée) ---

const startGpsWatch = (options) => {
    if (watchId !== null) navigator.geolocation.clearWatch(watchId);
    
    const handleSuccess = (pos) => {
        if (!isGpsRunning) return;

        const now = pos.timestamp;
        const deltaT = (now - lastTimestamp) / 1000 || (100 / 1000); // 100ms par défaut
        lastTimestamp = now;

        const raw = {
            lat: pos.coords.latitude,
            lon: pos.coords.longitude,
            alt: pos.coords.altitude || kAlt, // Utiliser kAlt si l'altitude brute manque
            acc: pos.coords.accuracy || R_NOISE_GPS_MIN,
            speed: pos.coords.speed || 0.0,
            heading: pos.coords.heading || kHeading
        };
        currentPosition = raw;
        
        // Calcul de la distance totale (avec turf.js)
        if (lastKnownPos) {
            const from = turf.point([lastKnownPos.lon, lastKnownPos.lat]);
            const to = turf.point([raw.lon, raw.lat]);
            distM += turf.distance(from, to, { units: 'meters' });
        }
        lastKnownPos = { lat: raw.lat, lon: raw.lon };

        // Mise à jour de la vitesse max
        maxSpdMs = Math.max(maxSpdMs, raw.speed);

        // --- ÉTAPE UKF ---
        const { kLat, kLon, kAlt, kSpd, R_GPS_m } = runUKFStep(deltaT, raw);

        // Mise à jour de la carte
        if (map && marker) {
            marker.setLatLng([kLat, kLon]).setRotationAngle(kHeading * R2D);
            map.setView([kLat, kLon]); // PanTo au lieu de setView pour une meilleure expérience
        }
    };

    const handleError = (error) => {
        console.error('Erreur GPS:', error);
        $('statut-gps-acquisition').textContent = `ERREUR: ${error.code}`;
        $('toggle-gps-btn').style.backgroundColor = 'orange';
        isGpsRunning = false;
    };

    watchId = navigator.geolocation.watchPosition(handleSuccess, handleError, options);
    $('statut-gps-acquisition').textContent = `ACQUISITION...`;
};

// Simulation IMU
const startImuSimulation = () => {
    // Dans un environnement réel, ceci serait navigator.permissions.query({ name: "accelerometer" })
    // Ici, on simule l'activation et le statut.
    imu.available = true;
    $('statut-capteur').textContent = 'Simulé';

    setInterval(() => {
        // Simuler des données de capteur basées sur la vitesse et l'environnement
        const speedKmh = kSpd * KMH_MS;
        const baseNoise = 0.05 + (speedKmh / 100) * 0.1; // Plus rapide, plus de bruit
        
        imu.ax = baseNoise * (Math.random() * 2 - 1) + 0.1 * kSpd; // Accélération longitudinale (simplifiée)
        imu.ay = baseNoise * (Math.random() * 2 - 1);
        imu.az = G_ACC_CURRENT / 9.81 + baseNoise * (Math.random() * 2 - 1); // 1g de base + bruit

        // Simuler Pitch/Roll (inclinaison)
        imu.pitch = Math.sin(new Date().getTime() / 10000) * 5 * (kSpd > 1 ? 1 : 0);
        imu.roll = Math.cos(new Date().getTime() / 15000) * 8 * (kSpd > 1 ? 1 : 0);

        // Mettre à jour le niveau à bulle
        if ($('bubble')) {
            const x = Math.min(40, Math.max(-40, imu.roll * 5));
            const y = Math.min(40, Math.max(-40, imu.pitch * 5));
            $('bubble').style.transform = `translate(${x}px, ${y}px)`;
        }
    }, GPS_UPDATE_INTERVAL);
};

// --- Gestion Météo/Polluants (Fetch) ---
const fetchWeather = async (lat, lon) => {
    const url = `${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`;
    try {
        const response = await fetch(url);
        if (!response.ok) throw new Error('API Météo/Air non disponible');
        const data = await response.json();
        
        lastWeather = {
            P_hPa: data.weather.pressure,
            T_K: data.weather.temp + 273.15,
            H_perc: data.weather.humidity / 100.0,
            airDensity: getAirDensity(data.weather.pressure, data.weather.temp + 273.15, data.weather.humidity),
            speedOfSound: getSpeedOfSound(data.weather.temp + 273.15)
        };
        lastPollutants = {
            no2: data.air.no2, pm25: data.air.pm25, pm10: data.air.pm10, o3: data.air.o3
        };

        $('statut-meteo').textContent = `ACTIF (Dernière MAJ: ${new Date().toLocaleTimeString()})`;
    } catch (e) {
        console.error("Échec du chargement Météo/Air:", e);
        $('statut-meteo').textContent = 'ÉCHEC (API PROXY)';
    }
};

// --- Initialisation de la Carte (Leaflet) ---
const initMap = (lat, lon) => {
    map = L.map('map', { zoomControl: false }).setView([lat, lon], 14);

    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; OpenStreetMap contributors'
    }).addTo(map);

    marker = L.marker([lat, lon], {
        icon: L.divIcon({
            className: 'custom-marker',
            html: '<i class="fas fa-location-arrow" style="font-size: 24px; color: #007bff; transform: rotate(var(--rotation, 0deg));"></i>',
            iconSize: [24, 24],
            iconAnchor: [12, 12]
        }),
        rotationAngle: 0,
        rotationOrigin: 'center center'
    }).addTo(map);
};
// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET FUSIONNÉ (UKF 21 ÉTATS)
// Partie 4/4 : Boucle d'Update DOM et Initialisation
// =================================================================

// --- Synchronisation NTP ---
const syncNTP = async () => {
    try {
        const start = new Date().getTime();
        const response = await fetch(SERVER_TIME_ENDPOINT);
        if (!response.ok) throw new Error("Échec de la synchro NTP");
        const data = await response.json();
        const end = new Date().getTime();
        
        lServH = data.unixtime * 1000;
        lLocH = end;
        ntpDelayMs = (end - start) / 2;
        $('heure-locale').textContent = getCDate(lServH, lLocH).toLocaleTimeString('fr-FR');
    } catch (e) {
        lServH = 0; lLocH = 0; ntpDelayMs = -1;
        $('heure-locale').textContent = 'SYNCHRO ÉCHOUÉE';
    }
};

// --- Boucle d'Update Lente du DOM ---
const startDOMUpdateLoop = () => {
    setInterval(() => {
        const now = getCDate(lServH, lLocH) || new Date();
        totalTimeSec = (now.getTime() - startTimeMs) / 1000;
        
        // --- 1. Mises à jour du temps et de la session ---
        $('heure-locale').textContent = now.toLocaleTimeString('fr-FR');
        $('date-heure-utc').textContent = now.toISOString().replace('T', ' ').substring(0, 19) + ' UTC';
        $('elapsed-session-time').textContent = formatTime(totalTimeSec);
        
        if (kSpd > 0.1) lastMotionTimeMs = now.getTime();
        motionTimeSec = (now.getTime() - lastMotionTimeMs) / 1000;
        $('elapsed-motion-time').textContent = formatTime(motionTimeSec);
        
        const astroTime = updateAstroTime(kLat, kLon, now);
        $('time-minecraft').textContent = astroTime.mcTime;
        $('tst').textContent = astroTime.TST;
        $('mst').textContent = astroTime.MST;
        $('noon-solar').textContent = astroTime.noonSolar;
        $('eot').textContent = astroTime.EOT;

        // Mise à jour Astro (Soleil/Lune)
        const times = SunCalc.getTimes(now, kLat, kLon);
        const sunPos = SunCalc.getPosition(now, kLat, kLon);
        const moonIllumination = SunCalc.getMoonIllumination(now);
        $('sun-alt').textContent = dataOrDefault(sunPos.altitude * R2D, 2, ' °');
        $('sun-azimuth').textContent = dataOrDefault(sunPos.azimuth * R2D, 2, ' °');
        
        if (times.sunset && times.sunrise) {
            const durationMs = times.sunset.getTime() - times.sunrise.getTime();
            $('day-duration').textContent = formatTime(durationMs / 1000);
        }

        // --- 2. Mise à jour des Modèles Physiques & Relativité ---
        const massKg = parseFloat($('mass-input').value) || 70;
        currentMass = massKg;
        
        const relativity = calculateRelativity(kSpd, massKg);
        const { dynamicPressure, dragForce, dragPowerKw } = calculateDrag(kSpd, lastWeather.airDensity, DRAG_COEFF, SURFACE_AREA);
        const coriolisForce = calculateCoriolis(kSpd, kLat * D2R, massKg);
        
        // Affichage Vitesse & Relativité
        const spdKmh = kSpd * KMH_MS;
        $('speed-stable').textContent = dataOrDefault(spdKmh, 1, ' km/h');
        $('speed-stable-ms').textContent = dataOrDefault(kSpd, 2, ' m/s');
        $('speed-stable-kms').textContent = dataOrDefault(kSpd, 6, ' km/s', 1/1000);
        $('vitesse-max-session').textContent = dataOrDefault(maxSpdMs * KMH_MS, 1, ' km/h');
        
        $('lorentz-factor').textContent = dataOrDefault(relativity.gamma, 4);
        $('percent-speed-light').textContent = dataOrDefaultExp(relativity.v_c_perc, 2, ' %');
        $('time-dilation-vitesse').textContent = dataOrDefault(relativity.timeDilationV_ns_day, 2, ' ns/j');
        $('rest-mass-energy').textContent = dataOrDefaultExp(relativity.E0, 2, ' J');
        $('relativistic-energy').textContent = dataOrDefaultExp(relativity.E, 2, ' J');
        $('momentum').textContent = dataOrDefaultExp(relativity.momentum, 4, ' kg⋅m/s');
        
        // Dynamique & Forces
        $('gravite-wgs84').textContent = dataOrDefault(G_ACC_CURRENT, 4, ' m/s²');
        $('force-g-long').textContent = dataOrDefault(imu.ax / 9.81, 2, ' g');
        $('force-g-vert').textContent = dataOrDefault(imu.az / 9.81, 2, ' g');
        $('vertical-speed').textContent = dataOrDefault(kAltDt, 2, ' m/s');
        
        $('dynamic-pressure').textContent = dataOrDefault(dynamicPressure, 2, ' Pa');
        $('drag-force').textContent = dataOrDefault(dragForce, 2, ' N');
        $('drag-power-kw').textContent = dataOrDefault(dragPowerKw, 2, ' kW');
        $('kinetic-energy').textContent = dataOrDefault(0.5 * massKg * kSpd * kSpd, 2, ' J');
        $('coriolis-force').textContent = dataOrDefault(coriolisForce, 4, ' N');
        
        // Distance
        const distanceRatio = netherMode ? 1/8 : (distanceRatioMode ? ((CELESTIAL_BODIES[currentCelestialBody].R + kAlt) / CELESTIAL_BODIES[currentCelestialBody].R) : 1.0);
        const scaledDist = distM * distanceRatio;
        $('distance-totale').textContent = `${dataOrDefault(scaledDist, 3, ' km', 1/1000)} | ${dataOrDefault(scaledDist, 2, ' m')}`;
        $('distance-ratio').textContent = dataOrDefault(distanceRatio, 3);
        $('distance-light-s').textContent = dataOrDefaultExp(scaledDist / C_L, 2, ' s');
        $('distance-light-min').textContent = dataOrDefaultExp(scaledDist / (C_L * 60), 2, ' min');
        $('distance-cosmic').textContent = `${dataOrDefaultExp(scaledDist / AU_TO_M, 2, ' UA')} | ${dataOrDefaultExp(scaledDist / LIGHT_YEAR_TO_M, 2, ' al')}`;

        // --- 3. Mises à jour Météo & Polluants ---
        $('air-temp-c').textContent = dataOrDefault(lastWeather.T_K - 273.15, 1, ' °C');
        $('pressure-hpa').textContent = dataOrDefault(lastWeather.P_hPa, 0, ' hPa');
        $('humidity-perc').textContent = dataOrDefault(lastWeather.H_perc, 1, ' %', 100);
        $('air-density').textContent = dataOrDefault(lastWeather.airDensity, 3, ' kg/m³');
        $('speed-of-sound-calc').textContent = dataOrDefault(lastWeather.speedOfSound, 2, ' m/s');
        
        $('no2-val').textContent = dataOrDefault(lastPollutants.no2, 0, ' µg/m³');
        $('pm25-val').textContent = dataOrDefault(lastPollutants.pm25, 0, ' µg/m³');

        // --- 4. Mises à jour UKF/IMU/Contrôles ---
        $('lat-ekf').textContent = dataOrDefault(kLat, 6, ' °');
        $('lon-ekf').textContent = dataOrDefault(kLon, 6, ' °');
        $('alt-ekf').textContent = dataOrDefault(kAlt, 2, ' m');
        $('acc-gps').textContent = dataOrDefault(currentPosition.acc, 2, ' m');
        $('ukf-alt-sigma').textContent = dataOrDefault(1.0 / UKF_P_k.get([2, 2]), 3, ' m²'); // Simulation de l'incertitude

        $('inclinaison-pitch').textContent = dataOrDefault(imu.pitch, 1, ' °');
        $('roulis-roll').textContent = dataOrDefault(imu.roll, 1, ' °');
        
        // Mettre à jour l'heure du dernier fetch météo
        const nowMs = new Date().getTime();
        if (nowMs - lastWeatherFetch > WEATHER_UPDATE_INTERVAL) {
             lastWeatherFetch = nowMs; 
             fetchWeather(kLat, kLon);
        }
        
    }, DOM_SLOW_UPDATE);
};

// --- Initialisation Finale ---
((window) => {
    document.addEventListener('DOMContentLoaded', async () => {
        // --- 1. Initialisation des composants critiques ---
        // Vérification des dépendances (leaflet/turf/suncalc/math.js)
        if (typeof math === 'undefined' || typeof L === 'undefined' || typeof SunCalc === 'undefined' || typeof turf === 'undefined') {
            console.error("Dépendances critiques manquantes.");
            document.body.innerHTML = '<div style="color:red;padding:20px;">ERREUR CRITIQUE: Dépendances JS manquantes (math, leaflet, turf, suncalc).</div>';
            return;
        }

        initMap(kLat, kLon); 
        await syncNTP();
        setInterval(syncNTP, NTP_SYNC_INTERVAL);
        
        // Démarrage de la simulation IMU
        startImuSimulation();

        // --- 2. Configuration des Écouteurs d'Événements ---
        $('toggle-gps-btn').addEventListener('click', () => {
            isGpsRunning = !isGpsRunning;
            if (isGpsRunning) {
                $('toggle-gps-btn').textContent = '⏸️ PAUSE GPS';
                $('toggle-gps-btn').style.backgroundColor = '#ffc107';
                startGpsWatch({ enableHighAccuracy: true, maximumAge: 0, timeout: 10000 });
            } else {
                $('toggle-gps-btn').textContent = '▶️ MARCHE GPS';
                $('toggle-gps-btn').style.backgroundColor = '#28a745';
                if (watchId !== null) navigator.geolocation.clearWatch(watchId);
                watchId = null;
            }
        });
        
        $('freq-select').addEventListener('change', (e) => {
             const options = e.target.value === 'HIGH_FREQ' 
                ? { enableHighAccuracy: true, maximumAge: 0, timeout: 1000 }
                : { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 };
             if (isGpsRunning) startGpsWatch(options);
        });

        $('emergency-stop-btn').addEventListener('click', () => {
            // Logique d'arrêt d'urgence
            if (watchId !== null) navigator.geolocation.clearWatch(watchId);
            isGpsRunning = false;
            kSpd = 0; kAltDt = 0;
            $('emergency-stop-btn').textContent = '🛑 Arrêt d\'urgence: ACTIF 🔴';
            $('emergency-stop-btn').classList.add('active');
            $('toggle-gps-btn').textContent = '▶️ MARCHE GPS (REDÉMARRER)';
        });

        $('reset-dist-btn').addEventListener('click', () => { distM = 0; });
        $('reset-max-btn').addEventListener('click', () => { maxSpdMs = 0; });
        $('reset-all-btn').addEventListener('click', () => { 
            if (confirm("Voulez-vous vraiment réinitialiser toutes les données de session (y compris la distance et la vitesse max)?")) {
                location.reload(); 
            }
        });
        
        $('gps-accuracy-override').addEventListener('input', (e) => {
            R_NOISE_OVERRIDE = parseFloat(e.target.value) || 0.0;
            $('gps-accuracy-display').textContent = dataOrDefault(R_NOISE_OVERRIDE, 6, ' m');
        });

        $('environment-select').addEventListener('change', (e) => {
            currentEnvironment = e.target.value;
            const env = ENVIRONMENT_FACTORS[currentEnvironment];
            $('env-factor').textContent = `${env.DISPLAY} (x${env.R_MULT.toFixed(1)})`;
        });
        
        $('celestial-body-select').addEventListener('change', (e) => {
            currentCelestialBody = e.target.value;
            G_ACC_CURRENT = calculateGravity(currentCelestialBody, kAlt, rotationRadius, angularVelocity);
            $('gravity-base').textContent = dataOrDefault(G_ACC_CURRENT, 4, ' m/s²');
        });
        
        $('rotation-radius').addEventListener('input', (e) => {
            rotationRadius = parseFloat(e.target.value) || 100;
            G_ACC_CURRENT = calculateGravity(currentCelestialBody, kAlt, rotationRadius, angularVelocity);
            $('gravity-base').textContent = dataOrDefault(G_ACC_CURRENT, 4, ' m/s²');
        });
        
        $('angular-velocity').addEventListener('input', (e) => {
            angularVelocity = parseFloat(e.target.value) || 0.0;
            G_ACC_CURRENT = calculateGravity(currentCelestialBody, kAlt, rotationRadius, angularVelocity);
            $('gravity-base').textContent = dataOrDefault(G_ACC_CURRENT, 4, ' m/s²');
        });

        $('distance-ratio-toggle-btn').addEventListener('click', () => {
            distanceRatioMode = !distanceRatioMode;
            $('distance-ratio-toggle-btn').textContent = `Rapport Distance: ${distanceRatioMode ? 'ALTITUDE' : 'SURFACE'} (${distanceRatioMode ? 'ALT' : '1.000'})`;
        });

        $('nether-toggle-btn').addEventListener('click', () => {
            netherMode = !netherMode;
            $('nether-toggle-btn').textContent = `Mode Nether: ${netherMode ? 'ACTIVÉ (1:8) 🔥' : 'DÉSACTIVÉ (1:1)'}`;
        });
        
        $('ukf-reactivity-mode').addEventListener('change', (e) => {
             currentUKFReactivity = e.target.value;
             // La réactivité est gérée dans la fonction runUKFStep
        });

        // --- 3. Démarrage des boucles et du premier fetch ---
        startDOMUpdateLoop();
        fetchWeather(kLat, kLon);
        
    });
})(window);
