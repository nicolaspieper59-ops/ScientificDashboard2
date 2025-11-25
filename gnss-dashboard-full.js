// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// BLOC 1/5 : État Global, Constantes de Base, et Utilitaires
// =================================================================

// --- ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE ---
let isGpsPaused = false; 
let isRunning = true;         // État de fonctionnement du tableau de bord

// IDs pour l'arrêt d'urgence et le contrôle du cycle de vie
let gpsWatchId = null;       // ID pour navigator.geolocation.watchPosition
let refreshIntervalId = null; // ID pour l'intervalle de rafraîchissement principal (Orchestrator/DOM)
let sensorPollIntervalId = null; // ID pour l'intervalle d'interrogation des capteurs

let currentPosition = { 
    // Coordonnées de travail (ex: Marseille)
    lat: 43.2964,   
    lon: 5.3697,    
    acc: 10.0,      
    spd: 0.0        
};

let lastGpsPosition = { lat: 0, lon: 0, alt: 0, spd: 0, heading: 0, acc: 10.0, ts: Date.now() };
let currentVehicleState = {
    mass: 100, // Masse du véhicule/utilisateur en kg
    cD: 0.5,   // Coefficient de traînée
    area: 0.8, // Surface de référence frontale en m²
};
let isMoving = false; 
let currentMass = 100.0; 
let selectedEnvironment = 'AIR'; 

// Variables pour les conditions atmosphériques
let lastT_K = 288.15; // Température par défaut (15°C) en Kelvin
let lastP_hPa = 1013.25; // Pression par défaut en hPa
let currentAirDensity = 1.225; // Densité de l'air par défaut (kg/m³)
let currentSpeedOfSound = 340.3; // Vitesse du son par défaut (m/s)

// Cache pour les données API
let lastKnownWeather = null; 
let lastKnownPollutants = null; 
let currentSimulationData = null; 

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);

/** Retourne un timestamp ISO pour IndexedDB. */
const nowISO = () => new Date().toISOString();

/** Met à jour le contenu textuel d'un élément DOM en toute sécurité. */
const safeSet = (id, txt) => { const e = $(id); if(e) e.textContent = txt; };

/**
 * Formate une valeur numérique ou retourne '0.00' si non valide.
 */
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        return (decimals === 0 ? '0' : '0.00') + suffix;
    }
    return val.toFixed(decimals) + suffix;
};

/**
 * Effectue un fetch avec une politique de backoff exponentiel.
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
// =================================================================
// GNSS SPACETIME DASHBOARD 
// BLOC 2/5 : Stockage IndexedDB, Constantes Physiques et UKF
// Dépendances : nowISO, isRunning (Bloc 1)
// =================================================================

/* ---------- STORAGE (IndexedDB) ---------- */

/**
 * Module de gestion du stockage IndexedDB pour l'historique des observations GNSS.
 */
const Storage = (function() {
    const DB = 'gnss_stime_db', STORE = 'obs';
    let db = null;

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

    async function addObservation(data) {
        if(!isRunning) return; // Ne pas enregistrer si arrêté
        const db = await openDB();
        const tx = db.transaction(STORE, 'readwrite');
        const store = tx.objectStore(STORE);
        store.add({ ...data, ts: nowISO() }); 
        return new Promise((res, rej) => { tx.oncomplete = res; tx.onerror = rej; });
    }

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

// Constantes Gaz Parfait/Atmosphère Standard (PHYS)
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
    WATER: { MULT: 800.0, DISPLAY: 'Eau (x800.0)' }, 
    SPACE: { MULT: 0.001, DISPLAY: 'Espace (x0.001)' },
};

// --- PARAMÈTRES UKF / FILTRE ---
const UKF_PARAMS = {
    ALPHA: 0.01, 
    BETA: 2,     
    KAPPA: 0,    
    Q_POS: 0.5,  // Bruit du processus (position)
    Q_VEL: 0.1,  // Bruit du processus (vitesse)
    R_POS: 10.0, // Bruit de mesure (position GNSS)
    R_VEL: 0.5,  // Bruit de mesure (vitesse GNSS)
};
// =================================================================
// GNSS SPACETIME DASHBOARD 
// BLOC 3/5 : Module SciencePro (Science Géophysique et Relativité)
// Dépendances : CONSTANTS, PHYS, DEG_TO_RAD (Bloc 2)
// =================================================================

/**
 * Module SciencePro: Fournit des fonctions autonomes pour les calculs géophysiques et astrophysiques.
 */
const SciencePro = (function() {
    const C = { ...CONSTANTS, ...PHYS, pi: Math.PI };

    /** Convertit une date JS en Jour Julien (JD). */
    function toJulian(date) {
        const Y = date.getUTCFullYear();
        let M = date.getUTCMonth() + 1;
        let D = date.getUTCDate() + (date.getUTCHours() / 24) + (date.getUTCMinutes() / 1440) + (date.getUTCSeconds() / 86400) + (date.getUTCMilliseconds() / 86400000);
        if (M <= 2) { Y--; M += 12; }
        const A = Math.floor(Y / 100);
        const B = 2 - A + Math.floor(A / 4);
        return Math.floor(365.25 * (Y + 4716)) + Math.floor(30.6001 * (M + 1)) + D + B - 1524.5;
    }

    /** Calcule la densité de l'air (rho) basée sur la pression et la température (Loi des gaz parfaits). */
    function getAirDensity(P_hPa, T_K) {
        // Pression convertie en Pa (1 hPa = 100 Pa)
        return (P_hPa * 100) / (C.R_d * T_K); 
    }

    /** Calcule la vitesse du son (a) dans l'air. */
    function getSpeedOfSound(T_K) {
        // a = sqrt(gamma * R_d * T)
        return Math.sqrt(C.gamma_air * C.R_d * T_K);
    }

    /** Calcule l'accélération gravitationnelle locale (g). */
    function getLocalGravity(lat_rad, alt_m) {
        // Formule de l'ellipsoïde normal (WGS84 approx.)
        const sinSq = Math.pow(Math.sin(lat_rad), 2);
        const g = 9.780327 * (1 + 0.0053024 * sinSq - 0.0000058 * sinSq * sinSq);
        // Correction d'altitude 
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
        return Math.sqrt(2 * C.R_earth * alt_m) / 1000;
    }

    /** Calcule l'altitude barométrique (simplifiée, atmosphère standard). */
    function barometricAltitude(P_hPa, P0_hPa, T0_K) {
        const exponent = (C.R_d * C.L_lapse) / C.g0;
        return (T0_K / C.L_lapse) * (1 - Math.pow(P_hPa / P0_hPa, exponent));
    }

    // Fonctions Astro/Temps (Simulées)
    function sunPosition(date) { /* Placeholder */ return { ra: 0, dec: 0 }; }
    function moonPhase(date) { /* Placeholder */ return { illuminatedFraction: 0.5, ageDays: 15 }; }
    function equationOfTime(date) { /* Placeholder */ return 0.0; } 

    return {
        getAirDensity,
        getSpeedOfSound,
        getLocalGravity,
        lorentzGamma,
        calculateDrag,
        calculateHorizon,
        barometricAltitude,
        sunPosition,
        moonPhase,
        equationOfTime,
        toJulian
    };
})();
// =================================================================
// GNSS SPACETIME DASHBOARD 
// BLOC 4/5 : FilterEngine (UKF Simplifié), Capteurs, et Contrôle d'Arrêt
// Dépendances : math.js (externe), UKF_PARAMS, Storage, orchestrator (Bloc 5),
//               currentPosition, lastGpsPosition, isRunning (Bloc 1)
// =================================================================

/* ---------- FilterEngine (UKF 6 États Simplifié pour Démo) ---------- */

/**
 * Module FilterEngine: Implémente le Filtre de Kalman Non-Senté (UKF) ou EKF.
 * REQUIERT : math.min.js (pour les opérations matricielles)
 */
const FilterEngine = (function() {
    let state = { ts: nowISO(), x: null, P: null };
    let isInitialized = false;
    let lastTs = Date.now();
    let R, Q; 

    // Modèle cinématique f: p_new = p_old + v * dt
    const f = (x, dt) => {
        const x_new = x.slice(); 
        x_new[0] += x[3] * dt; // lat
        x_new[1] += x[4] * dt; // lon
        x_new[2] += x[5] * dt; // alt 
        return x_new;
    };

    /** Initialise l'UKF avec les premières coordonnées GNSS. */
    function initFromGnss(lat, lon, acc) {
        // [lat, lon, alt, vx, vy, vz]
        state.x = math.matrix([lat, lon, 0, 0, 0, 0]); 

        const P_size = 6; 
        state.P = math.diag(math.zeros(P_size), [
            (acc * acc) * 10, (acc * acc) * 10, (acc * acc) * 10, 
            UKF_PARAMS.Q_VEL, UKF_PARAMS.Q_VEL, UKF_PARAMS.Q_VEL 
        ]); 

        Q = math.diag(math.zeros(P_size), [
            UKF_PARAMS.Q_POS, UKF_PARAMS.Q_POS, UKF_PARAMS.Q_POS,
            UKF_PARAMS.Q_VEL, UKF_PARAMS.Q_VEL, UKF_PARAMS.Q_VEL
        ]);
        R = math.diag(math.zeros(6), [
            UKF_PARAMS.R_POS, UKF_PARAMS.R_POS, UKF_PARAMS.R_POS,
            UKF_PARAMS.R_VEL, UKF_PARAMS.R_VEL, UKF_PARAMS.R_VEL
        ]);

        isInitialized = true;
    }

    /** Exécute une étape de prédiction et de mise à jour (simulée/simplifiée). */
    function update(gnssData, dt) {
        if (!isInitialized || !isRunning) return null;
        
        // 1. Prédiction
        state.x = f(state.x.toArray(), dt);
        state.P = math.add(state.P, Q); 

        // 2. Mise à Jour (Mesure GPS)
        const z = math.matrix([
            gnssData.lat, gnssData.lon, gnssData.alt || 0, 
            gnssData.spd * Math.cos(gnssData.heading * DEG_TO_RAD) || 0,
            gnssData.spd * Math.sin(gnssData.heading * DEG_TO_RAD) || 0,
            gnssData.verticalSpeed || 0
        ]);
        
        const K_sim = 0.4; // Gain de Kalman simulé
        state.x = math.add(math.multiply(state.x, (1 - K_sim)), math.multiply(z, K_sim));
        state.P = math.multiply(state.P, (1 - K_sim));
        
        // 3. Retour de l'état filtré
        state.ts = nowISO();
        return {
            lat: state.x.get([0]),
            lon: state.x.get([1]),
            alt: state.x.get([2]),
            spd: math.norm([state.x.get([3]), state.x.get([4])]), 
            P: state.P.toArray()
        };
    }

    return { initFromGnss, update, getState: () => isInitialized ? state : null };
})();


/* ---------- GPS & Sensor Handlers et Contrôle d'Arrêt ---------- */

/** Démarre la surveillance GPS et capture l'ID du watcher. */
function initGnssWatch(successCallback, errorCallback, options) {
    if (gpsWatchId !== null) return;
    if (!navigator.geolocation) {
        console.error("Géolocalisation non supportée.");
        return;
    }
    gpsWatchId = navigator.geolocation.watchPosition(successCallback, errorCallback, options);
    safeSet('toggle-gps-btn', 'Arrêter GPS');
    safeSet('gps-status', 'Actif');
}

/** Arrête la surveillance GPS. */
const stopGps = () => {
    if (gpsWatchId !== null) {
        navigator.geolocation.clearWatch(gpsWatchId);
        gpsWatchId = null;
    }
    safeSet('toggle-gps-btn', 'Démarrer GPS');
    safeSet('gps-status', 'Inactif');
};

/** Met à jour la position GPS (callback de watchPosition). */
const updateGps = (position) => {
    if (!isRunning) return; 

    const { latitude, longitude, altitude, speed, heading, accuracy } = position.coords;
    const verticalSpeed = position.coords.altitudeAccuracy ? (lastGpsPosition.alt - altitude) / ((Date.now() - lastGpsPosition.ts) / 1000) : 0;
    
    // Mettre à jour l'état brut
    lastGpsPosition = { 
        lat: latitude, lon: longitude, alt: altitude, spd: speed || 0, 
        heading: heading || 0, acc: accuracy, verticalSpeed: verticalSpeed, ts: Date.now()
    };
    currentPosition = { lat: latitude, lon: longitude, acc: accuracy, spd: speed || 0 };
    
    if (!FilterEngine.getState()) {
        FilterEngine.initFromGnss(latitude, longitude, accuracy);
    }
    
    // Exécuter l'étape du filtre
    const dt = (Date.now() - lastTs) / 1000;
    lastTs = Date.now();
    
    const filteredState = FilterEngine.update(lastGpsPosition, dt);

    if (filteredState) {
        orchestrator.runRefresh(filteredState.lat, filteredState.lon, filteredState);
        Storage.addObservation({
            ts: nowISO(), lat: filteredState.lat, lon: filteredState.lon, acc: accuracy,
            filteredP: filteredState.P, rawSpeed: speed,
        });
    }
};

/** Gère les erreurs GPS. */
const errorGps = (err) => {
    console.error(`Erreur GPS (${err.code}): ${err.message}`);
    safeSet('gps-status', `Erreur (${err.code})`);
};

/** Interrupteur GPS. */
const toggleGps = () => {
    if (gpsWatchId === null) {
        const options = { enableHighAccuracy: true, timeout: 5000, maximumAge: 0 };
        initGnssWatch(updateGps, errorGps, options);
    } else {
        stopGps();
    }
};

/** Simule la lecture des capteurs (baromètre/thermomètre). */
const pollSensors = async () => {
    if (!isRunning) return;

    // Simulation d'une variation de capteur
    if (lastKnownWeather) {
        lastP_hPa = lastKnownWeather.pressure_hPa * (1 + (Math.random() - 0.5) * 0.005); 
        lastT_K = lastKnownWeather.tempK * (1 + (Math.random() - 0.5) * 0.001); 
    }
    
    currentAirDensity = SciencePro.getAirDensity(lastP_hPa, lastT_K);
    currentSpeedOfSound = SciencePro.getSpeedOfSound(lastT_K);
};

/**
 * Arrête toutes les mises à jour en temps réel (GPS, intervalles) et les boucles de l'application.
 * FONCTION D'ARRÊT D'URGENCE.
 */
function stopDashboardUpdates() {
    if (!isRunning) return; 

    stopGps();

    if (refreshIntervalId !== null) {
        clearInterval(refreshIntervalId);
        refreshIntervalId = null;
    }

    if (sensorPollIntervalId !== null) {
        clearInterval(sensorPollIntervalId);
        sensorPollIntervalId = null;
    }

    isRunning = false;
    
    // Mise à jour de l'interface utilisateur
    safeSet('fix-type', 'ARRÊT D\'URGENCE'); 
    safeSet('speed-3d-inst', '0.000 m/s');
    
    const stopBtn = $('emergency-stop-btn');
    if (stopBtn) {
        stopBtn.textContent = 'Mises à jour arrêtées';
        stopBtn.disabled = true;
    }

    console.log('Tableau de bord GNSS arrêté.');
            }
// =================================================================
// GNSS SPACETIME DASHBOARD 
// BLOC 5/5 : Orchestrateur, Mises à Jour du DOM, et Séquence d'Initialisation
// Dépendances : Tous les blocs précédents.
// =================================================================

/* ---------- Orchestrator et Mises à Jour du DOM ---------- */

/**
 * Orchestrateur: Gère les appels aux API externes et l'agrégation des données.
 */
const orchestrator = (function() {
    const UPDATE_INTERVAL_MS = 3600000; // 1 heure

    /** Met à jour la météo et les polluants si nécessaire. */
    async function updateMetrology(lat, lon) {
        if (!isRunning) return;
        const now = Date.now();
        // Le code de fetchWithBackoff pour les API météo et polluants irait ici.
        // Il est conservé tel quel (simulé) pour des raisons de complétude.
        
        // ... (Simulations d'appels API)
        
        // Exemple de mise à jour forcée (pour la démo)
        lastKnownWeather = lastKnownWeather || { 
            tempK: PHYS.TEMP_SEA_LEVEL_K, 
            pressure_hPa: PHYS.BARO_ALT_REF_HPA, 
            air_density: PHYS.RHO_SEA_LEVEL
        };
        updateWeatherDOM(lastKnownWeather);
        lastKnownPollutants = lastKnownPollutants || { co: 0.0, no2: 0.0 };
        updatePollutantsDOM(lastKnownPollutants);
    }

    /** Exécute toutes les mises à jour non critiques. */
    async function runRefresh(lat, lon, filteredState = null) {
        if (!lat || !lon || !isRunning) return;

        await updateMetrology(lat, lon);
        updateDOM(currentPosition, filteredState, currentSpeedOfSound, currentAirDensity);

        // Mettre à jour l'astro (simulé)
        const now = new Date();
        const JD = SciencePro.toJulian(now);
        safeSet('sun-alt', dataOrDefault(Math.sin(lat * DEG_TO_RAD) * 90, 2) + '°');
        safeSet('moon-phase-name', JD % 30 < 15 ? 'Croissant' : 'Décroissant');

        // Mettre à jour le marqueur Leaflet (s'assurer que la carte et le marqueur existent)
        // if (typeof L !== 'undefined' && typeof map !== 'undefined' && typeof userMarker !== 'undefined') {
        //     userMarker.setLatLng([lat, lon]);
        //     map.setView([lat, lon], map.getZoom() < 10 ? 10 : map.getZoom());
        // }
    }

    return { runRefresh };
})();

// --- FONCTIONS DE MISE À JOUR DU DOM ---

/** Met à jour les éléments DOM pour les données filtrées et dérivées. */
const updateDOM = (rawPos, state, currentSpeedOfSound, currentAirDensity) => {
    // Données de base
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
        const envFactor = ENVIRONMENT_FACTORS[selectedEnvironment].MULT;
        const { q_dyn, dragForce, dragPower } = SciencePro.calculateDrag(
            currentAirDensity * envFactor, 
            v, 
            currentVehicleState.cD, 
            currentVehicleState.area, 
            envFactor
        );
        safeSet('qdyn', dataOrDefault(q_dyn, 2) + ' Pa');
        safeSet('drag', dataOrDefault(dragForce, 2) + ' N');
        safeSet('drag-kw', dataOrDefault(dragPower / 1000, 2) + ' kW');
        safeSet('lorentz', SciencePro.lorentzGamma(v).toFixed(8));
        
        // Incertitudes (UKF)
        const Pmat = state.P || null;
        if (Pmat && Pmat.length > 2 && Array.isArray(Pmat[0])) {
             safeSet('p-pos', dataOrDefault(Math.sqrt(Pmat[0][0] + Pmat[1][1]), 3) + ' m');
             safeSet('p-vel', dataOrDefault(Math.sqrt(Pmat[3][3] + Pmat[4][4]), 3) + ' m/s'); 
             safeSet('filter-type', 'UKF-6 (Défaut)');
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
    const P_hPa = weatherData.pressure_hPa;
    const T_K = weatherData.tempK;
    const altBaro = SciencePro.barometricAltitude(P_hPa, PHYS.BARO_ALT_REF_HPA, PHYS.TEMP_SEA_LEVEL_K);

    safeSet('temp-calc', dataOrDefault(T_K - 273.15, 1) + ' °C');
    safeSet('pressure-calc', dataOrDefault(P_hPa, 2) + ' hPa');
    safeSet('alt-baro-calc', dataOrDefault(altBaro, 2) + ' m' + (isDefault ? ' (Défaut)' : ''));
    
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
    
    // 1. Initialiser les valeurs par défaut (Correction Métrologique)
    lastKnownWeather = { 
        tempK: PHYS.TEMP_SEA_LEVEL_K, 
        pressure_hPa: PHYS.BARO_ALT_REF_HPA, 
        air_density: PHYS.RHO_SEA_LEVEL
    }; 

    currentAirDensity = lastKnownWeather.air_density;
    currentSpeedOfSound = SciencePro.getSpeedOfSound(lastKnownWeather.tempK); 
    updateWeatherDOM(lastKnownWeather, true);
    updatePollutantsDOM({ co: 0.0, no2: 0.0 }, true);
    
    // Mettre à jour les affichages par défaut (mass, env, etc.)
    safeSet('mass-display', `${currentMass.toFixed(3)} kg`);
    safeSet('env-factor', `${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY}`);

    // 2. Initialiser le GPS 
    toggleGps(); 

    // 3. Initialiser les mises à jour périodiques et CAPTURER les IDs
    sensorPollIntervalId = setInterval(pollSensors, 60000); // Mise à jour métrologique chaque minute
    refreshIntervalId = setInterval(() => { 
        if (FilterEngine.getState()) {
            orchestrator.runRefresh(currentPosition.lat, currentPosition.lon, FilterEngine.getState());
        }
    }, 7000); // Rafraîchissement d'appoint toutes les 7 secondes

    // 4. Lier l'événement d'arrêt d'urgence
    const stopBtn = $('emergency-stop-btn');
    if (stopBtn) {
        stopBtn.addEventListener('click', stopDashboardUpdates);
    }

    console.log('Dashboard initialisé et surveillance GNSS démarrée.');
})();
