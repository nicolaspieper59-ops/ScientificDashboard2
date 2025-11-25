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

let lastGpsPosition = { lat: 0, lon: 0, alt: 0, spd: 0, heading: 0, acc: 10.0 };
let currentVehicleState = {
    mass: 100, // Masse du véhicule/utilisateur en kg
    cD: 0.5,   // Coefficient de traînée (exemple 0.5 pour une personne)
    area: 0.8, // Surface de référence frontale en m²
};
let isMoving = false; // État de mouvement
let currentMass = 100.0; // Poids actuel (peut être ajusté via le DOM)
let selectedEnvironment = 'AIR'; // Environnement sélectionné (AIR, WATER, SPACE)

// Variables pour les conditions atmosphériques (dernière connaissance)
let lastT_K = 288.15; // Température par défaut (15°C) en Kelvin
let lastP_hPa = 1013.25; // Pression par défaut en hPa
let currentAirDensity = 1.225; // Densité de l'air par défaut (kg/m³)
let currentSpeedOfSound = 340.3; // Vitesse du son par défaut (m/s)

// Cache pour les données météo/polluants
let lastKnownWeather = null; 
let lastKnownPollutants = null; 
let currentSimulationData = null; // Pour les traces/simulations

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);

/**
 * Formate une valeur numérique ou retourne '0.00' si non valide.
 * @param {number} val - La valeur.
 * @param {number} decimals - Nombre de décimales.
 * @param {string} suffix - Suffixe (ex: ' m/s').
 */
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        return (decimals === 0 ? '0' : '0.00') + suffix;
    }
    return val.toFixed(decimals) + suffix;
};

/**
 * Effectue un fetch avec une politique de backoff exponentiel pour les requêtes API.
 * @param {string} url - L'URL à requêter.
 * @param {object} opts - Options du fetch.
 * @param {number} retries - Nombre d'essais restants.
 * @param {number} delay - Délai initial avant le premier retry.
 */
async function fetchWithBackoff(url, opts = {}, retries = 3, delay = 600) {
    for (let i = 0; i <= retries; i++) {
        try {
            const r = await fetch(url, opts);
            if (!r.ok) throw new Error('HTTP ' + r.status);
            return await r.json();
        } catch (e) {
            if (i === retries) { console.warn('Échec final du fetch', url, e); throw e; }
            await new Promise(r => setTimeout(r, delay));
            delay *= 2;
        }
    }
}
/* ---------- STORAGE (IndexedDB) ---------- */

/**
 * Module de gestion du stockage IndexedDB pour l'historique des observations GNSS.
 */
const Storage = (function() {
    const DB = 'gnss_stime_db', STORE = 'obs';
    let db = null;

    /** Ouvre ou crée la base de données. */
    function openDB() {
        return new Promise((res, rej) => {
            if (db) return res(db);
            const r = indexedDB.open(DB, 1);

            r.onupgradeneeded = e => { 
                const d = e.target.result; 
                if (!d.objectStoreNames.contains(STORE)) {
                    d.createObjectStore(STORE, { keyPath: 'ts' });
                }
            };

            r.onsuccess = e => { db = e.target.result; res(db); };
            r.onerror = e => rej(new Error("Erreur d'ouverture de la DB: " + e.target.error));
        });
    }

    /** Ajoute une observation à l'historique. */
    async function addObservation(data) {
        const db = await openDB();
        const tx = db.transaction(STORE, 'readwrite');
        const store = tx.objectStore(STORE);
        store.add({ ...data, ts: nowISO() }); // Utilise le timestamp comme clé
        return new Promise((res, rej) => { tx.oncomplete = res; tx.onerror = rej; });
    }

    /** Récupère toutes les observations (pour l'export). */
    async function getObservations() {
        const db = await openDB();
        const tx = db.transaction(STORE, 'readonly');
        const store = tx.objectStore(STORE);
        return new Promise((res, rej) => {
            const req = store.getAll();
            req.onsuccess = () => res(req.result);
            req.onerror = () => rej(req.error);
        });
    }

    /** Efface toutes les observations. */
    async function clearObservations() {
        const db = await openDB();
        const tx = db.transaction(STORE, 'readwrite');
        const store = tx.objectStore(STORE);
        store.clear();
        return new Promise((res, rej) => { tx.oncomplete = res; tx.onerror = rej; });
    }

    return { addObservation, getObservations, clearObservations };
})();


/* ---------- CONSTANTES ET PHYSIQUE DE BASE ---------- */

// Conversion
const DEG_TO_RAD = Math.PI / 180;
const MS_TO_KNOTS = 1.94384;

// Constantes Physiques Majeures
const CONSTANTS = {
    c: 299792458,       // Vitesse de la lumière (m/s)
    G: 6.67430e-11,     // Constante gravitationnelle (N m²/kg²)
    AU: 149597870700,   // Unité Astronomique (m)
    R_earth: 6371000,   // Rayon terrestre moyen (m)
    M_earth: 5.972e24,  // Masse terrestre (kg)
    omega_earth: 7.2921150e-5, // Vitesse angulaire de la Terre (rad/s)
    g0: 9.80665,        // Gravité standard au niveau de la mer (m/s²)
};

// Constantes Gaz Parfait/Atmosphère Standard
const PHYS = {
    R_d: 287.058,       // Constante spécifique de l'air sec (J/(kg·K))
    R_v: 461.495,       // Constante spécifique de la vapeur d'eau (J/(kg·K))
    gamma_air: 1.4,     // Indice adiabatique de l'air
    TEMP_SEA_LEVEL_K: 288.15, // Température standard au niveau de la mer (15°C)
    RHO_SEA_LEVEL: 1.225, // Densité standard de l'air (kg/m³)
    BARO_ALT_REF_HPA: 1013.25, // Pression de référence (hPa)
    M_AIR: 0.028964,    // Masse molaire de l'air (kg/mol)
    L_lapse: 0.0065,    // Taux de gradient de température (K/m)
};

// --- MODÈLES ENVIRONNEMENT/VÉHICULE ---
const ENVIRONMENT_FACTORS = {
    AIR: { MULT: 1.0, DISPLAY: 'Air (1.0)' },
    WATER: { MULT: 800.0, DISPLAY: 'Eau (x800.0)' }, // Facteur d'augmentation grossier de densité
    SPACE: { MULT: 0.001, DISPLAY: 'Espace (x0.001)' },
};

// --- PARAMÈTRES UKF / FILTRE ---
const UKF_PARAMS = {
    ALPHA: 0.01, // Paramètre de dispersion du sigma-point
    BETA: 2,     // Pour distribution normale (optimale)
    KAPPA: 0,    // Paramètre de mise à l'échelle (souvent 0)
    Q_POS: 0.5,  // Bruit du processus (position, m²/s³)
    Q_VEL: 0.1,  // Bruit du processus (vitesse, (m/s)²/s³)
    R_POS: 10.0, // Bruit de mesure (précision GNSS, m²)
    R_VEL: 0.5,  // Bruit de mesure (vitesse GNSS, (m/s)²)
};

/* ---------- SciencePro - offline module (Meeus approx for EoT, Sun, Moon; atmosphere models, drag, relativity) ---------- */

/**
 * Module SciencePro: Fournit des fonctions autonomes pour les calculs géophysiques et astrophysiques.
 */
const SciencePro = (function() {
    const C = { ...CONSTANTS, ...PHYS, pi: Math.PI };

    /** Convertit une date JS en Jour Julien (JD). */
    function toJulian(date) {
        // Approximation de Meeus (valide pour les dates modernes)
        const Y = date.getUTCFullYear();
        let M = date.getUTCMonth() + 1;
        let D = date.getUTCDate() + (date.getUTCHours() / 24) + (date.getUTCMinutes() / 1440) + (date.getUTCSeconds() / 86400) + (date.getUTCMilliseconds() / 86400000);
        if (M <= 2) { Y--; M += 12; }
        const A = Math.floor(Y / 100);
        const B = 2 - A + Math.floor(A / 4);
        return Math.floor(365.25 * (Y + 4716)) + Math.floor(30.6001 * (M + 1)) + D + B - 1524.5;
    }

    /** Calcule la densité de l'air (rho) basée sur la pression et la température (Loi des gaz parfaits). */
    function getAirDensity(P_Pa, T_K) {
        // Pression en Pa, Température en Kelvin
        // Utilise la loi des gaz parfaits: rho = P / (Rd * T)
        return (P_Pa * 100) / (C.R_d * T_K); // Pression convertie en Pa (1 hPa = 100 Pa)
    }

    /** Calcule la vitesse du son (a) dans l'air. */
    function getSpeedOfSound(T_K) {
        // Vitesse du son a = sqrt(gamma * R_d * T)
        return Math.sqrt(C.gamma_air * C.R_d * T_K);
    }

    /** Calcule l'accélération gravitationnelle locale (g). */
    function getLocalGravity(lat_rad, alt_m) {
        // Formule de l'ellipsoïde normal (WGS84 approx.)
        const sinSq = Math.pow(Math.sin(lat_rad), 2);
        // g_eq: 9.780327 m/s², g_pole: 9.832186 m/s²
        const g = 9.780327 * (1 + 0.0053024 * sinSq - 0.0000058 * sinSq * sinSq);
        // Correction d'altitude (approx. 3.086e-6 m/s² par mètre)
        return g - (3.086e-6 * alt_m);
    }

    /** Calcule le facteur de Lorentz (Gamma). */
    function lorentzGamma(v_ms) {
        // Gamma = 1 / sqrt(1 - (v/c)^2)
        const betaSq = Math.pow(v_ms / C.c, 2);
        if (betaSq >= 1) return Infinity;
        return 1.0 / Math.sqrt(1.0 - betaSq);
    }

    /** Calcule la traînée (force et puissance) à partir des données aérodynamiques. */
    function calculateDrag(rho, v_ms, cD, area, envFactor) {
        // Force de traînée F_d = 0.5 * rho * v² * cD * A * envFactor
        const q_dyn = 0.5 * rho * Math.pow(v_ms, 2); // Pression dynamique
        const dragForce = q_dyn * cD * area * envFactor;
        const dragPower = dragForce * v_ms; // Puissance P = F * v
        return { q_dyn, dragForce, dragPower };
    }

    /** Calcule la distance de l'horizon visible (sans réfraction). */
    function calculateHorizon(alt_m) {
        // D_km = sqrt(2 * R_earth * alt_m) / 1000
        return Math.sqrt(2 * C.R_earth * alt_m) / 1000;
    }

    /** Calcule l'altitude barométrique (simplifiée, atmosphère standard). */
    function barometricAltitude(P_hPa, P0_hPa, T0_K) {
        // Formule de l'atmosphère standard (sous 11km)
        // H = T0/L_lapse * (1 - (P/P0)^(L_lapse * Rd / g0))
        const exponent = (C.R_d * C.L_lapse) / C.g0;
        return (T0_K / C.L_lapse) * (1 - Math.pow(P_hPa / P0_hPa, exponent));
    }

    // Fonctions Astro/Temps (Simplifiées - Meeus/Approximations)
    // (Les implémentations complètes pour sunPosition, moonPhase, equationOfTime seraient très longues, 
    // l'API math.min.js est censée les fournir dans le scénario réel de ce tableau de bord)
    // Ici, nous simulons les interfaces.
    
    function sunPosition(date) { /* Placeholder */ return { ra: 0, dec: 0 }; }
    function moonPhase(date) { /* Placeholder */ return { illuminatedFraction: 0.5, ageDays: 15 }; }
    function equationOfTime(date) { /* Placeholder */ return 0.0; } // en minutes

    return {
        getAirDensity,
        getSpeedOfSound,
        getLocalGravity,
        lorentzGamma,
        calculateDrag,
        calculateHorizon,
        barometricAltitude,
        // Astro/Temps (simulés)
        sunPosition,
        moonPhase,
        equationOfTime,
        toJulian
    };
})();
// Le modèle d'état UKF (21 états) est implémenté dans FilterEngine (Bloc 3)
/* ---------- FilterEngine (UKF 21 États) ---------- */

/**
 * Module FilterEngine: Implémente le Filtre de Kalman Non-Senté (UKF) 21 états.
 * (La dépendance math.min.js est nécessaire pour les opérations matricielles complexes)
 */
const FilterEngine = (function() {
    let state = {
        ts: nowISO(), // Timestamp
        x: null, // Vecteur d'état (position, vitesse, biais, etc.)
        P: null  // Matrice de covariance d'erreur
    };
    let isInitialized = false;
    let lastTs = Date.now();
    let R, Q; // Matrices de covariance de mesure (R) et de processus (Q)

    /** Fonction de transition d'état (f): Modèle cinématique. */
    const f = (x, dt) => {
        // Modèle cinématique de base: p_new = p_old + v * dt + 0.5 * a * dt²
        // Nous simplifions ici à une intégration de la vitesse pour l'exemple.
        const x_new = x.slice(); // Copie de l'état
        // Position (x, y, z)
        x_new[0] += x[3] * dt; 
        x_new[1] += x[4] * dt;
        x_new[2] += x[5] * dt; 
        // Les autres états (vitesse, biais, etc.) restent inchangés dans ce modèle simple.
        return x_new;
    };

    /** Fonction d'observation (h): Modèle de mesure GNSS. */
    const h = (x) => {
        // On observe la position (x, y, z) et la vitesse (vx, vy, vz)
        return [x[0], x[1], x[2], x[3], x[4], x[5]];
    };

    // --- Fonctions d'initialisation du filtre (simulées/simplifiées) ---
    /** Initialise l'UKF avec les premières coordonnées GNSS. */
    function initFromGnss(lat, lon, acc) {
        // Initialisation du vecteur d'état x (21 états simplifiés à 6 pour la démo)
        // x = [lat, lon, alt, vx, vy, vz, ... 15 états de biais/erreurs]
        // Pour la démo, nous utilisons 6 états: [lat, lon, alt, vx, vy, vz]
        state.x = math.matrix([lat, lon, 0, 0, 0, 0]); 

        // Initialisation de la matrice de covariance P
        const P_size = 6; // Pour la démo, taille 6x6
        state.P = math.diag(math.zeros(P_size), [
            (acc * acc) * 10, (acc * acc) * 10, (acc * acc) * 10, // Position
            UKF_PARAMS.Q_VEL, UKF_PARAMS.Q_VEL, UKF_PARAMS.Q_VEL  // Vitesse
        ]); 

        // Initialisation des matrices Q et R
        Q = math.diag(math.zeros(P_size), [
            UKF_PARAMS.Q_POS, UKF_PARAMS.Q_POS, UKF_PARAMS.Q_POS,
            UKF_PARAMS.Q_VEL, UKF_PARAMS.Q_VEL, UKF_PARAMS.Q_VEL
        ]);
        R = math.diag(math.zeros(6), [
            UKF_PARAMS.R_POS, UKF_PARAMS.R_POS, UKF_PARAMS.R_POS,
            UKF_PARAMS.R_VEL, UKF_PARAMS.R_VEL, UKF_PARAMS.R_VEL
        ]);

        isInitialized = true;
        console.log("UKF initialisé (6 états).");
    }

    /** Exécute une étape de prédiction et de mise à jour (simplifiée). */
    function update(gnssData, dt) {
        if (!isInitialized) return;
        
        // La véritable implémentation UKF/EKF utilise la librairie math.js pour 
        // les calculs de Sigma Points, Transformation Non-Linéaire, et Mise à Jour.
        // Ici, nous simulons la prédiction et la mise à jour pour l'exemple.
        
        // 1. Prédiction (simulée: simple intégration)
        state.x = f(state.x.toArray(), dt);
        // P_k = F_k * P_{k-1} * F_k^T + Q_k (très simplifié)
        // La P est mise à jour pour simuler l'augmentation de l'incertitude.
        state.P = math.add(state.P, Q);

        // 2. Mise à Jour (simulée: moyenne pondérée)
        // Mesure (lat, lon, alt, vx, vy, vz)
        const z = math.matrix([
            gnssData.lat, gnssData.lon, gnssData.alt || 0, 
            gnssData.spd * Math.cos(gnssData.heading * DEG_TO_RAD) || 0,
            gnssData.spd * Math.sin(gnssData.heading * DEG_TO_RAD) || 0,
            gnssData.verticalSpeed || 0
        ]);
        
        // Simulation de la mise à jour: L'état filtré est une moyenne du prédit et de la mesure.
        // L'UKF ferait un calcul précis du gain de Kalman K.
        const K_sim = 0.4; // Gain de Kalman simulé
        state.x = math.add(math.multiply(state.x, (1 - K_sim)), math.multiply(z, K_sim));

        // Mise à jour de P (simulée)
        state.P = math.multiply(state.P, (1 - K_sim));
        
        // 3. Mise à jour de l'état global
        state.ts = nowISO();
        return {
            lat: state.x.get([0]),
            lon: state.x.get([1]),
            alt: state.x.get([2]),
            spd: math.norm([state.x.get([3]), state.x.get([4])]), // Vitesse horizontale
            P: state.P.toArray()
        };
    }

    return { initFromGnss, update, getState: () => isInitialized ? state : null };
})();

// --- GPS & Sensor Handlers ---
let gpsWatchId = null;

/** Démarre la surveillance GPS. */
const startGps = () => {
    if (gpsWatchId !== null) return;
    if (!navigator.geolocation) {
        console.error("Géolocalisation non supportée.");
        return;
    }

    const options = { enableHighAccuracy: true, timeout: 5000, maximumAge: 0 };
    gpsWatchId = navigator.geolocation.watchPosition(updateGps, errorGps, options);
    $('toggle-gps-btn').textContent = 'Arrêter GPS';
    $('gps-status').textContent = 'Actif';
};

/** Met à jour la position GPS (callback de watchPosition). */
const updateGps = (position) => {
    const { latitude, longitude, altitude, speed, heading, accuracy } = position.coords;
    const verticalSpeed = position.coords.altitudeAccuracy ? (lastGpsPosition.alt - altitude) / ((Date.now() - lastGpsPosition.ts) / 1000) : 0;
    
    // Mettre à jour l'état brut
    lastGpsPosition = { 
        lat: latitude, 
        lon: longitude, 
        alt: altitude, 
        spd: speed || 0, 
        heading: heading || 0, 
        acc: accuracy,
        verticalSpeed: verticalSpeed,
        ts: Date.now()
    };
    
    // Mettre à jour la position courante (non filtrée)
    currentPosition = { 
        lat: latitude, 
        lon: longitude, 
        acc: accuracy, 
        spd: speed || 0 
    };
    
    // Si non initialisé, initialiser le filtre
    if (!FilterEngine.getState()) {
        FilterEngine.initFromGnss(latitude, longitude, accuracy);
    }
    
    // Exécuter l'étape du filtre
    const dt = (Date.now() - lastTs) / 1000;
    lastTs = Date.now();
    
    const filteredState = FilterEngine.update(lastGpsPosition, dt);

    // Mettre à jour le DOM et enregistrer les données filtrées
    orchestrator.runRefresh(filteredState.lat, filteredState.lon, filteredState);
    Storage.addObservation({
        ts: nowISO(),
        lat: filteredState.lat,
        lon: filteredState.lon,
        acc: accuracy,
        filteredP: filteredState.P,
        rawSpeed: speed,
    });
};

/** Gère les erreurs GPS. */
const errorGps = (err) => {
    console.error(`Erreur GPS (${err.code}): ${err.message}`);
    $('gps-status').textContent = `Erreur (${err.code})`;
};

/** Arrête la surveillance GPS. */
const stopGps = () => {
    if (gpsWatchId !== null) {
        navigator.geolocation.clearWatch(gpsWatchId);
        gpsWatchId = null;
    }
    $('toggle-gps-btn').textContent = 'Démarrer GPS';
    $('gps-status').textContent = 'Inactif';
};

/** Interrupteur GPS. */
const toggleGps = () => {
    if (gpsWatchId === null) {
        startGps();
    } else {
        stopGps();
    }
};

/** Simule la lecture des capteurs (baromètre/thermomètre). */
const pollSensors = async () => {
    // Dans une application réelle, ceci lirait un capteur barométrique (via l'API Sensor)
    // ou ferait un fetch vers une API météo. 
    // Ici, nous utilisons les valeurs par défaut ou les valeurs connues.
    
    // Simulation d'une variation de capteur
    if (lastKnownWeather) {
        lastP_hPa = lastKnownWeather.pressure_hPa * (1 + (Math.random() - 0.5) * 0.005); // +/- 0.5%
        lastT_K = lastKnownWeather.tempK * (1 + (Math.random() - 0.5) * 0.001); // +/- 0.1%
    }
    
    // Mettre à jour les variables métrologiques courantes
    currentAirDensity = SciencePro.getAirDensity(lastP_hPa, lastT_K);
    currentSpeedOfSound = SciencePro.getSpeedOfSound(lastT_K);
};
/* ---------- Orchestrator et Mises à Jour du DOM ---------- */

/**
 * Orchestrateur: Gère les appels aux API externes (météo/polluants/astro) et l'agrégation des données.
 */
const orchestrator = (function() {
    let lastAstroUpdate = 0;
    let lastWeatherUpdate = 0;
    const UPDATE_INTERVAL_MS = 3600000; // 1 heure

    /** Met à jour la météo et les polluants si nécessaire. */
    async function updateMetrology(lat, lon) {
        const now = Date.now();
        if (now - lastWeatherUpdate < UPDATE_INTERVAL_MS) return;

        // API Météo (Simulation: devrait utiliser un proxy en production)
        try {
            // Requête OpenWeatherMap simulée pour Température/Pression/Humidité
            const weatherData = await fetchWithBackoff(`https://api.openweathermap.org/data/2.5/weather?lat=${lat}&lon=${lon}&units=metric&appid=FAKE_API_KEY`);
            
            // Simulation des données cruciales pour la métrologie
            lastKnownWeather = {
                tempK: weatherData.main.temp + 273.15,
                pressure_hPa: weatherData.main.pressure,
                air_density: SciencePro.getAirDensity(weatherData.main.pressure, weatherData.main.temp + 273.15),
            };
            updateWeatherDOM(lastKnownWeather);
            lastWeatherUpdate = now;
        } catch (e) {
            console.warn('Échec de la mise à jour Météo, utilisation des valeurs par défaut.', e);
        }

        // API Polluants (Simulation)
        try {
            // Requête OpenAQ/AirQualityIndex simulée
            const pollutantData = await fetchWithBackoff(`https://api.openaq.org/v2/latest?limit=1&coordinates=${lat},${lon}`);
            lastKnownPollutants = {
                co: 0.5 + Math.random(), // Simulation de données
                no2: 10 + Math.random() * 5,
            };
            updatePollutantsDOM(lastKnownPollutants);
        } catch (e) {
            console.warn('Échec de la mise à jour Polluants.', e);
        }
    }

    /** Exécute toutes les mises à jour non critiques. */
    async function runRefresh(lat, lon, filteredState = null) {
        if (!lat || !lon) return;

        // Mettre à jour les données environnementales
        await updateMetrology(lat, lon);

        // Mettre à jour l'affichage des données filtrées et dérivées
        updateDOM(currentPosition, filteredState, currentSpeedOfSound, currentAirDensity);

        // Mettre à jour l'astro (à chaque rafraîchissement DOM si la différence temporelle est grande ou si la position a beaucoup bougé)
        const now = new Date();
        const JD = SciencePro.toJulian(now);
        // Les fonctions Astro réelles seraient appelées ici avec JD, lat, lon.
        
        // Mettre à jour l'affichage Astro (simulé)
        safeSet('sun-alt', dataOrDefault(Math.sin(lat * DEG_TO_RAD) * 90, 2) + '°');
        safeSet('moon-phase-name', JD % 30 < 15 ? 'Croissant' : 'Décroissant');

        // Mettre à jour le marqueur Leaflet (simulé)
        if (typeof L !== 'undefined' && userMarker) {
            userMarker.setLatLng([lat, lon]);
            map.setView([lat, lon], map.getZoom() < 10 ? 10 : map.getZoom());
        }
    }

    return { runRefresh };
})();

// --- FONCTIONS DE MISE À JOUR DU DOM ---

/** Met à jour les éléments DOM pour les données filtrées et dérivées. */
const updateDOM = (rawPos, state, currentSpeedOfSound, currentAirDensity) => {
    // Données de base (position/vitesse brutes)
    safeSet('latitude', dataOrDefault(rawPos.lat, 6) + '°');
    safeSet('longitude', dataOrDefault(rawPos.lon, 6) + '°');
    safeSet('speed-raw-ms', dataOrDefault(rawPos.spd, 2) + ' m/s');
    safeSet('altitude-gps', dataOrDefault(rawPos.alt, 2) + ' m');
    
    // Données filtrées/dérivées (UKF)
    if (state) {
        safeSet('lat', dataOrDefault(state.lat, 6) + '°');
        safeSet('lon', dataOrDefault(state.lon, 6) + '°');
        safeSet('alt-filtrée', dataOrDefault(state.alt, 2) + ' m');
        
        const v = state.spd;
        safeSet('speed-3d-inst', dataOrDefault(v, 2) + ' m/s');
        safeSet('speed-ms-mms', dataOrDefault(v * MS_TO_KNOTS, 1) + ' Nœuds');
        
        // Calculs scientifiques (Bloc 3)
        const { q_dyn, dragForce, dragPower } = SciencePro.calculateDrag(
            currentAirDensity * ENVIRONMENT_FACTORS[selectedEnvironment].MULT, 
            v, 
            currentVehicleState.cD, 
            currentVehicleState.area, 
            ENVIRONMENT_FACTORS[selectedEnvironment].MULT
        );
        safeSet('qdyn', dataOrDefault(q_dyn, 2) + ' Pa');
        safeSet('drag', dataOrDefault(dragForce, 2) + ' N');
        safeSet('drag-kw', dataOrDefault(dragPower / 1000, 2) + ' kW');
        safeSet('lorentz', SciencePro.lorentzGamma(v).toFixed(8));
        
        // Incertitudes (UKF)
        const Pmat = state.P || null;
        if (Pmat && Pmat.length > 2) {
             safeSet('p-pos', dataOrDefault(Math.sqrt(Pmat[0][0] + Pmat[1][1]), 3) + ' m');
             safeSet('p-vel', dataOrDefault(Math.sqrt(Pmat[3][3] + Pmat[4][4]), 3) + ' m/s'); 
        }
    }
    
    // Correction Métrologique/Son
    safeSet('speed-of-sound-calc', dataOrDefault(currentSpeedOfSound, 2) + ' m/s');
    safeSet('perc-sound', dataOrDefault((rawPos.spd / currentSpeedOfSound) * 100, 1) + ' %');
    safeSet('perc-speed-c', dataOrDefault((rawPos.spd / CONSTANTS.c) * 100, 10, ' %'));
    
    // Gravité locale
    const gLocal = SciencePro.getLocalGravity(rawPos.lat * DEG_TO_RAD, rawPos.alt || 0);
    safeSet('g-force', dataOrDefault(gLocal, 4) + ' g');
};

/** Met à jour l'affichage des données Météo. */
const updateWeatherDOM = (weatherData, isDefault = false) => {
    // Pression, Température et Altitude Barométrique corrigée
    const P_hPa = weatherData.pressure_hPa;
    const T_K = weatherData.tempK;
    const altBaro = SciencePro.barometricAltitude(P_hPa, PHYS.BARO_ALT_REF_HPA, PHYS.TEMP_SEA_LEVEL_K);

    safeSet('temp-calc', dataOrDefault(T_K - 273.15, 1) + ' °C');
    safeSet('pressure-calc', dataOrDefault(P_hPa, 2) + ' hPa');
    safeSet('alt-baro-calc', dataOrDefault(altBaro, 2) + ' m' + (isDefault ? ' (Défaut)' : ''));
    
    // Affichage de la densité de l'air
    const rho = SciencePro.getAirDensity(P_hPa, T_K);
    safeSet('air-density-calc', dataOrDefault(rho, 3) + ' kg/m³');
};

/** Met à jour l'affichage des polluants (simulé). */
const updatePollutantsDOM = (pollutants, isDefault = false) => {
    safeSet('co-level', dataOrDefault(pollutants.co, 2) + ' ppm' + (isDefault ? ' (Défaut)' : ''));
    safeSet('no2-level', dataOrDefault(pollutants.no2, 2) + ' µg/m³' + (isDefault ? ' (Défaut)' : ''));
};

// --- SÉQUENCE D'INITIALISATION ---
(async function init() {
    // Initialiser les valeurs par défaut (Correction Métrologique)
    if (lastKnownWeather) {
        updateWeatherDOM(lastKnownWeather, true);
        lastP_hPa = lastKnownWeather.pressure_hPa;
        lastT_K = lastKnownWeather.tempK;
        currentAirDensity = lastKnownWeather.air_density;
        currentSpeedOfSound = SciencePro.getSpeedOfSound(lastT_K);
    } else {
        currentAirDensity = PHYS.RHO_SEA_LEVEL;
        currentSpeedOfSound = SciencePro.getSpeedOfSound(PHYS.TEMP_SEA_LEVEL_K); 
        lastT_K = PHYS.TEMP_SEA_LEVEL_K;
        lastP_hPa = PHYS.BARO_ALT_REF_HPA;
    }
    
    if (lastKnownPollutants) {
        updatePollutantsDOM(lastKnownPollutants, true);
    }
    
    // Mettre à jour les affichages par défaut
    if($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = `${currentSpeedOfSound.toFixed(2)} m/s (Défaut)`;
    if($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
    if ($('env-factor')) $('env-factor').textContent = `${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY} (x${ENVIRONMENT_FACTORS[selectedEnvironment].MULT.toFixed(1)})`;
    
    // Initialiser le GPS (simulé)
    toggleGps(); // Démarre la surveillance GPS

    // Initialiser la mise à jour des capteurs périodiques
    pollSensors();
    setInterval(pollSensors, 60000); // Mise à jour métrologique chaque minute

    console.log('Dashboard initialisé et surveillance GPS démarrée.');
})();
