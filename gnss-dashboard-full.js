// =================================================================
// BLOC 1/5 : Constantes, Utilitaires, Grandeurs Théoriques (TQ)
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);
const toRad = deg => deg * (Math.PI / 180);
const toDeg = rad => rad * (180 / Math.PI);

/**
 * Formate une valeur numérique ou retourne une chaîne par défaut.
 * @param {number} val - Valeur à formater.
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
 * Formate une valeur en notation exponentielle ou retourne une chaîne par défaut.
 * @param {number} val - Valeur à formater.
 * @param {number} decimals - Nombre de décimales.
 * @param {string} suffix - Suffixe.
 */
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        const zeroDecimals = '0.' + Array(decimals).fill('0').join('');
        return zeroDecimals + 'e+0' + suffix;
    }
    return val.toExponential(decimals) + suffix;
};


// --- CONSTANTES PHYSIQUES ET ISA (Atmosphère Standard Internationale) ---
const C_LIGHT = 299792458;          // Vitesse de la lumière (m/s)
const G_CONST = 6.67430e-11;        // Constante gravitationnelle universelle (m³/kg/s²)
const G_ACC_EARTH = 9.80665;        // Gravité standard à la surface de la Terre (m/s²)
const R_EARTH = 6371000;            // Rayon terrestre moyen (m)
const TEMP_SEA_LEVEL_K = 288.15;    // Température au niveau de la mer (K)
const BARO_ALT_REF_HPA = 1013.25;   // Pression atmosphérique standard au niveau de la mer (hPa)
const RHO_SEA_LEVEL = 1.225;        // Densité de l'air au niveau de la mer (kg/m³)
const GAS_CONST_AIR = 287.05;       // Constante spécifique des gaz pour l'air sec (J/kg/K)


// --- CLASSE ET INITIALISATION DES GRANDEURS THÉORIQUES (TQ) ---

/**
 * Classe pour gérer les constantes théoriques, extraite du HTML.
 */
class TheoreticalQuantities {
    constructor() {
        this.q = {};
    }

    add(id, name, value, unit, critical = false) {
        this.q[id] = { name, value, unit, critical };
    }

    // Met à jour les éléments DOM qui existent déjà (ex: c, G)
    populateDOM() {
        for (const k in this.q) {
            const el = document.getElementById(k);
            if (el) {
                // Utilisation de la notation exponentielle pour les constantes SI
                const textContent = TQ.q[k].value.toExponential(10) + (TQ.q[k].unit ? (' ' + TQ.q[k].unit) : '');
                el.textContent = textContent;
            }
        }
    }
}

// Instanciation de la classe et ajout des constantes
const TQ = new TheoreticalQuantities();
TQ.add('c', 'Vitesse de la lumière', C_LIGHT, 'm/s', true);
TQ.add('G', 'Gravitation Universelle (G)', G_CONST, 'm³/kg/s²', false);
TQ.add('planck-h', 'Constante de Planck (h)', 6.62607015e-34, 'J·s', false);
TQ.add('boltzmann-kB', 'Constante de Boltzmann (kB)', 1.380649e-23, 'J/K', false);
TQ.add('gas-constant-R', 'Constante des Gaz Parfaits (R)', 8.314462618, 'J/mol/K', false);
TQ.add('stefan-boltzmann', 'Constante de Stefan-Boltzmann', 5.670374419e-8, 'W/m²/K⁴', false);
TQ.add('avogadro-NA', "Nombre d'Avogadro (NA)", 6.02214076e23, 'mol⁻¹', false);

// Assure l'initialisation du panneau TQ au chargement du DOM
document.addEventListener('DOMContentLoaded', () => {
    // Logique pour créer le panneau TQ dynamique (si non présent dans l'HTML)
    const column3 = document.querySelector('.column-3');
    if (column3) {
        const container = document.createElement('section');
        container.className = 'panel';
        const header = document.createElement('h2');
        header.innerHTML = '<i class="fas fa-flask"></i> Grandeurs Mesurables & Théoriques (Calculées)';
        container.appendChild(header);
        const list = document.createElement('div');
        list.className = 'theoretical-list';
        
        // Populate TQ list (excluding those already in column-2)
        for (const k in TQ.q) {
            if (!document.getElementById(k)) {
                const dp = document.createElement('div');
                // ... (code de construction de la ligne TQ) ...
            }
        }

        container.appendChild(list);
        const levelSection = document.querySelector('#level-heading').closest('section');
        if (levelSection) {
             column3.insertBefore(container, levelSection);
        } else {
             column3.appendChild(container);
        }
    }
    
    // Mettre à jour les valeurs des constantes déjà présentes dans le DOM
    TQ.populateDOM();
});
// =================================================================
// FIN BLOC 1/5
// =================================================================
// =================================================================
// BLOC 2/5 : DÉMARRAGE, ÉTAT GLOBAL & UKF
// =================================================================

((window) => {

    // --- VERIFICATION DES DÉPENDANCES CRITIQUES ---
    if (typeof math === 'undefined' || typeof L === 'undefined' || typeof SunCalc === 'undefined' || typeof turf === 'undefined') {
        const missing = [
            (typeof math === 'undefined' ? "math.min.js" : ""),
            (typeof L === 'undefined' ? "leaflet.js" : ""),
            (typeof SunCalc === 'undefined' ? "suncalc.js" : ""),
            (typeof turf === 'undefined' ? "turf.min.js" : "")
        ].filter(n => n !== "");

        const errorMsg = `Erreur critique : Dépendances manquantes : ${missing.join(', ')}. Le script est arrêté.`;
        console.error(errorMsg);
        document.body.innerHTML = `<div style="padding: 20px; color: white; background: #8B0000;">${errorMsg}</div>`;
        return; // Arrêt du script
    }
    
    // --- VARIABLES D'ÉTAT GLOBAL ---
    let isGPSRunning = false;
    let watchID = null;
    let fastLoopID = null;
    let slowLoopID = null;
    let startTime = 0;
    let timeMoving = 0;
    let lastTime = Date.now();
    let currentSpeedMax = 0;
    let distanceTotal = 0;
    let distanceRatioMode = false; // true = Alt-Corrigé, false = Surface
    
    let currentMass = parseFloat($('mass-input').value) || 70.0;
    let currentCelestialBody = $('celestial-body-select').value || 'EARTH';
    let currentGravityAcc = G_ACC_EARTH;
    let rotationRadius = parseFloat($('rotation-radius').value) || 100;
    let angularVelocity = parseFloat($('angular-velocity').value) || 0.0;
    
    // Données des capteurs
    let lastKnownPos = null;
    let lastKnownVelocity = null;
    let lastKnownAccel = { x: 0, y: 0, z: 0 };
    let lastKnownMagnetic = { x: 0, y: 0, z: 0 };
    let lastKnownOrientation = { alpha: 0, beta: 0, gamma: 0 };

    // Données atmosphériques par défaut (ISA)
    let currentAirDensity = RHO_SEA_LEVEL;
    let currentSpeedOfSound = 340.29; // Vitesse du son à 15°C
    let lastT_K = TEMP_SEA_LEVEL_K;
    let lastP_hPa = BARO_ALT_REF_HPA;

    // État UKF (Unscented Kalman Filter)
    const UKF_STATE_DIM = 21; // x, y, z, vx, vy, vz, ax, ay, az, roll, pitch, yaw, d_roll, d_pitch, d_yaw, bias_ax, bias_ay, bias_az, bias_gx, bias_gy, bias_gz
    let UKF_X = math.zeros(UKF_STATE_DIM)._data; // Vecteur d'état
    let UKF_P = math.multiply(math.eye(UKF_STATE_DIM), 10)._data; // Matrice de covariance

    // Facteurs d'environnement pour ajuster l'UKF
    const ENVIRONMENT_FACTORS = {
        'NORMAL': { MULT: 1.0, DISPLAY: "Normal" },
        'FOREST': { MULT: 2.5, DISPLAY: "Forêt" },
        'CONCRETE': { MULT: 7.0, DISPLAY: "Grotte/Tunnel" },
        'METAL': { MULT: 5.0, DISPLAY: "Métal/Bâtiment" },
    };
    let selectedEnvironment = $('environment-select').value || 'NORMAL';
    let UKF_PROCESS_NOISE_MULT = ENVIRONMENT_FACTORS[selectedEnvironment].MULT;
    
    // État de la carte
    let map = null;
    let marker = null;
    let trackPolyline = null;
    let trackPoints = [];
    let isMapInitialized = false;

// =================================================================
// FIN BLOC 2/5
// =================================================================
 // =================================================================
// BLOC 3/5 : Logique du Filtre UKF (21 États) & Physique
// REMARQUE : L'implémentation complète des matrices F, H, Q, R de l'UKF n'est pas détaillée ici,
// mais est représentée par les fonctions d'initialisation, de prédiction et de mise à jour.
// =================================================================

// --- FONCTIONS PHYSIQUES ET ATMOSPHÉRIQUES ---

/** Calcule la vitesse du son (m/s) dans l'air à partir de la température (K). */
const getSpeedOfSound = (T_K) => {
    // Formule: a = sqrt(gamma * R * T) où gamma (ratio des chaleurs spécifiques) est ~1.4 pour l'air sec
    return Math.sqrt(1.4 * GAS_CONST_AIR * T_K);
};

/** Calcule la densité de l'air (kg/m³) à partir de la pression (hPa) et de la température (K). */
const calculateAirDensity = (P_hPa, T_K) => {
    // Formule: rho = (P * 100) / (R * T)
    const P_Pa = P_hPa * 100; // Pression en Pascals
    return P_Pa / (GAS_CONST_AIR * T_K);
};

/** Calcule le facteur de Lorentz (gamma). */
const calculateLorentzFactor = (v) => {
    if (v >= C_LIGHT) return Infinity;
    return 1 / Math.sqrt(1 - (v * v) / (C_LIGHT * C_LIGHT));
};

/** Calcule la dilatation temporelle (en nanosecondes par jour) due à la vitesse. */
const calculateTimeDilationVel = (gamma) => {
    if (gamma === 1) return 0;
    const s_in_day = 86400;
    // Dilation T = (gamma - 1) * temps de référence
    const dilation_seconds = (gamma - 1) * s_in_day;
    return dilation_seconds * 1e9; // ns/jour
};

/** Calcule la dilatation temporelle (en nanosecondes par jour) due à la gravité. */
const calculateTimeDilationGrav = (alt, R_body, G_acc) => {
    const s_in_day = 86400;
    // Formule simplifiée pour la surface terrestre: delta_t = (G_acc * alt / C²) * t
    const dilation_seconds = (G_acc * alt / (C_LIGHT * C_LIGHT)) * s_in_day;
    return dilation_seconds * 1e9; // ns/jour
};

/** Calcule le rayon de Schwarzschild (m) pour une masse donnée. */
const calculateSchwarzschildRadius = (mass) => {
    // Formule: Rs = 2 * G * M / c²
    return (2 * G_CONST * mass) / (C_LIGHT * C_LIGHT);
};

/** Met à jour les paramètres du corps céleste (Gravité et Rotation). */
const updateCelestialBody = (bodyKey, altitude, rotationR, angularV) => {
    let G_ACC_NEW = G_ACC_EARTH;

    switch (bodyKey) {
        case 'EARTH':
            // Gravité en fonction de l'altitude
            G_ACC_NEW = G_ACC_EARTH * (R_EARTH * R_EARTH) / Math.pow(R_EARTH + altitude, 2);
            break;
        case 'MOON':
            G_ACC_NEW = 1.62; // Lune
            break;
        case 'MARS':
            G_ACC_NEW = 3.71; // Mars
            break;
        case 'ROTATING':
            // Calcule l'accélération centrifuge à la circonférence
            const a_centrifuge = rotationR * angularV * angularV;
            // La gravité est la simulation de la force centrifuge
            G_ACC_NEW = a_centrifuge; 
            break;
    }
    currentGravityAcc = G_ACC_NEW;
    return { G_ACC_NEW };
};


// --- UKF : FONCTIONS CŒUR ---

/** Initialise l'UKF avec une position GPS initiale. */
const initializeUKF = (lat, lon, alt) => {
    // x = [x, y, z, vx, vy, vz, ax, ay, az, roll, pitch, yaw, d_roll, d_pitch, d_yaw, bias_ax, bias_ay, bias_az, bias_gx, bias_gy, bias_gz]
    // Initialise la position (x, y, z) avec la première mesure
    // (Conversion de Lat/Lon/Alt en coordonnées ECEF (simplifié pour math.js))
    UKF_X = math.zeros(UKF_STATE_DIM)._data;
    // Ici, nous supposons que UKF_X[0], UKF_X[1], UKF_X[2] représentent une position
    // UKF_X[0] = lat, UKF_X[1] = lon, UKF_X[2] = alt (Simplification)
    UKF_X[0] = lat;
    UKF_X[1] = lon;
    UKF_X[2] = alt;

    // Initialisation de la matrice de covariance P
    UKF_P = math.multiply(math.eye(UKF_STATE_DIM), 10)._data;
    // Plus de confiance dans la position initiale
    UKF_P[0][0] = UKF_P[1][1] = UKF_P[2][2] = 1; 

    console.log("UKF Initialisé.");
};

/** Étape de prédiction UKF. */
const predictUKF = (dt) => {
    // F: Matrice de transition d'état (basée sur le modèle cinématique)
    // Q: Matrice de bruit de processus (ajustée par UKF_PROCESS_NOISE_MULT)
    // (Implémentation math.js non montrée mais supposée existante)
    // UKF_X = F * UKF_X
    // UKF_P = F * UKF_P * F' + Q 
};

/** Étape de mise à jour UKF (avec mesures GPS et IMU). */
const updateUKF = (currentPosition, currentVelocity, currentIMU) => {
    // H: Matrice d'observation
    // R: Matrice de bruit de mesure (basée sur l'exactitude GPS et les facteurs d'environnement)
    // (Implémentation math.js non montrée mais supposée existante)
    // K: Gain de Kalman = P * H' * (H * P * H' + R)^-1
    // UKF_X = UKF_X + K * (Z - H * UKF_X)
    // UKF_P = (I - K * H) * UKF_P
};

// =================================================================
// FIN BLOC 3/5
// =================================================================
 // =================================================================
// BLOC 4/5 : Gestion des Capteurs, Géolocalisation & Carte (Leaflet)
// =================================================================

// --- LOGIQUE DE LA CARTE (LEAFLET) ---

/** Initialise la carte Leaflet. */
const startMap = () => {
    if (isMapInitialized) return;

    map = L.map('map').setView([45, 0], 2); // Position par défaut

    // Utilisation d'une tuile standard (OpenStreetMap)
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        maxZoom: 19,
        attribution: '© OpenStreetMap contributors'
    }).addTo(map);

    trackPolyline = L.polyline([], { color: '#007bff', weight: 3 }).addTo(map);
    marker = L.circleMarker([0, 0], { color: '#ff0000', radius: 6 }).addTo(map);

    isMapInitialized = true;
};

/** Met à jour la carte avec la position stable. */
const updateMap = (lat, lon) => {
    if (!isMapInitialized) startMap();
    if (!lat || !lon) return;

    const latlng = [lat, lon];
    
    // Mettre à jour la ligne de trajectoire (Turf est utilisé pour des analyses complexes)
    trackPoints.push(latlng);
    trackPolyline.setLatLngs(trackPoints);

    // Mettre à jour le marqueur
    marker.setLatLng(latlng);
    
    // Zoomer sur le dernier point si c'est le premier ou si l'utilisateur n'a pas bougé la carte.
    if (trackPoints.length <= 1) {
        map.setView(latlng, 15);
    }
};

// --- HANDLERS GPS ---

/** Succès de l'API de géolocalisation. */
const getLocationSuccess = (position) => {
    const { latitude: lat, longitude: lon, altitude: alt, accuracy, altitudeAccuracy, speed, heading } = position.coords;
    const timestamp = position.timestamp;
    
    const kLat = lat;
    const kLon = lon;
    const kAlt = alt || 0;
    
    // Si l'UKF n'est pas initialisé, le faire maintenant
    if (UKF_X[0] === 0 && trackPoints.length === 0) {
        initializeUKF(kLat, kLon, kAlt);
    }
    
    // --- UKF MISE À JOUR ---
    // Les mesures d'observation (Z) sont construites ici
    const currentIMU = { accel: lastKnownAccel, orientation: lastKnownOrientation }; // Données IMU
    updateUKF({ lat: kLat, lon: kLon, alt: kAlt }, { speed, heading }, currentIMU);

    // Mettre à jour les données stables avec l'état UKF (positions 0, 1, 2)
    lastKnownPos = { lat: UKF_X[0], lon: UKF_X[1], alt: UKF_X[2] };
    
    // Vitesse (UKF positions 3, 4, 5)
    const vx = UKF_X[3];
    const vy = UKF_X[4];
    const vz = UKF_X[5];
    lastKnownVelocity = { 
        speed: Math.sqrt(vx*vx + vy*vy + vz*vz), 
        bearing: toDeg(Math.atan2(vx, vy)) 
    };
    
    // Mettre à jour le DOM et la carte
    updateMap(lastKnownPos.lat, lastKnownPos.lon);
    
    // Enregistrer les données brutes
    lastKnownPos.accuracy = accuracy;
    lastKnownPos.hdop = position.coords.hdop || 1; // HDOP et VDOP ne sont souvent pas disponibles dans l'API standard
    lastKnownPos.vdop = position.coords.vdop || 1;
    lastKnownPos.satCount = position.coords.satellites || 0;
};

/** Erreur de l'API de géolocalisation. */
const getLocationError = (error) => {
    console.warn(`Erreur GPS (${error.code}): ${error.message}`);
    $('speed-status-text').textContent = 'Erreur GPS ou Accès Refusé.';
};

// --- HANDLERS CAPTEURS IMU ET ENVIRONNEMENTAUX ---

/** Gestionnaire des événements DeviceMotion (Accéléromètre/G-Force) */
const handleDeviceMotion = (event) => {
    const { x, y, z } = event.accelerationIncludingGravity;
    lastKnownAccel = { x, y, z };
    
    // Mettre à jour l'état IMU
    $('imu-status').textContent = 'Actif';

    // Calcul de la G-Force
    const gTotal = Math.sqrt(x*x + y*y + z*z) / G_ACC_EARTH;
    $('g-force-x').textContent = dataOrDefault(x / G_ACC_EARTH, 4, ' g');
    $('g-force-y').textContent = dataOrDefault(y / G_ACC_EARTH, 4, ' g');
    $('g-force-z').textContent = dataOrDefault(z / G_ACC_EARTH, 4, ' g');
    $('g-force-total').textContent = dataOrDefault(gTotal, 4, ' g');
};

/** Gestionnaire des événements DeviceOrientation (Magnétomètre/Orientation) */
const handleDeviceOrientation = (event) => {
    lastKnownOrientation = { alpha: event.alpha, beta: event.beta, gamma: event.gamma };

    // Mettre à jour l'inclinaison/tangage
    $('roll').textContent = dataOrDefault(event.gamma, 2, '°'); // Roll (gamma)
    $('pitch').textContent = dataOrDefault(event.beta, 2, '°');  // Pitch (beta)
};

/** Gestionnaire du Niveau de Lumière Ambiante */
const handleAmbientLight = (event) => {
    const lux = event.value;
    $('ambient-light').textContent = dataOrDefault(lux, 2, ' Lux');
    
    const maxLux = parseFloat($('ambient-light-max').textContent) || 0;
    if (lux > maxLux) {
        $('ambient-light-max').textContent = dataOrDefault(lux, 2, ' Lux');
    }
};

// =================================================================
// FIN BLOC 4/5
// =================================================================
// =================================================================
// BLOC 5/5 : Boucles, Mise à Jour DOM & Événements (Initialisation)
// =================================================================

// --- FONCTIONS DE BOUCLE ---

/** Boucle rapide (20 Hz) pour les calculs cinématiques et d'affichage. */
const fastLoop = () => {
    const now = Date.now();
    const dt = (now - lastTime) / 1000; // Delta temps en secondes
    lastTime = now;
    
    // 1. Prédiction UKF (basée sur le modèle cinématique et l'accélération)
    predictUKF(dt);

    // 2. Mises à jour du temps et de la distance
    const speed = lastKnownVelocity ? lastKnownVelocity.speed : 0;
    
    if (isGPSRunning) {
        $('elapsed-time').textContent = dataOrDefault((now - startTime) / 1000, 2, ' s');
        
        if (speed > 0.5) { // Est en mouvement
            timeMoving += dt;
            distanceTotal += speed * dt;
        }
        $('time-moving').textContent = dataOrDefault(timeMoving, 2, ' s');
        
        // Vitesse Max
        if (speed > currentSpeedMax) currentSpeedMax = speed;
        $('speed-max').textContent = dataOrDefault(currentSpeedMax * 3.6, 1, ' km/h');
        
        // Mises à jour Vitesse/Cinématique
        $('speed-stable').textContent = dataOrDefault(speed * 3.6, 1, ' km/h');
        $('speed-stable-ms').textContent = dataOrDefault(speed, 2, ' m/s');
        $('speed-stable-kms').textContent = dataOrDefault(speed / 1000, 4, ' km/s');
        
        // Relativité
        const gamma = calculateLorentzFactor(speed);
        $('lorentz-factor').textContent = dataOrDefault(gamma, 4);
        $('perc-speed-c').textContent = dataOrDefaultExp((speed / C_LIGHT) * 100, 2, ' %');
        $('time-dilation-vel').textContent = dataOrDefault(calculateTimeDilationVel(gamma), 2, ' ns/j');
        
        // Poids/Énergie
        const restEnergy = currentMass * C_LIGHT * C_LIGHT;
        const relativisticEnergy = currentMass * gamma * C_LIGHT * C_LIGHT;
        $('rest-energy').textContent = dataOrDefaultExp(restEnergy, 4, ' J');
        $('relativistic-energy').textContent = dataOrDefaultExp(relativisticEnergy, 4, ' J');
        
        // Distance
        const totalKm = distanceTotal / 1000;
        $('distance-total').textContent = `${dataOrDefault(totalKm, 3, ' km')} | ${dataOrDefault(distanceTotal, 2, ' m')}`;

    } else {
        $('speed-status-text').textContent = 'GPS en Pause.';
    }

    fastLoopID = requestAnimationFrame(fastLoop);
};

/** Boucle lente (1 Hz) pour les mises à jour non critiques (Astro, Heure). */
const slowLoop = () => {
    // Astro & Heure
    const date = new Date();
    $('date-display').textContent = date.toISOString().replace('T', ' ').substring(0, 19) + ' UTC';
    
    if (lastKnownPos) {
        // SunCalc
        const times = SunCalc.getTimes(date, lastKnownPos.lat, lastKnownPos.lon);
        const sunPos = SunCalc.getPosition(date, lastKnownPos.lat, lastKnownPos.lon);
        
        $('sun-zenith').textContent = dataOrDefault(toDeg(sunPos.altitude), 2, '°'); // Zenith/Altitude
        $('sun-azimuth').textContent = dataOrDefault(toDeg(sunPos.azimuth) + 180, 2, '°'); // Azimuth
        $('sunrise-sunset').textContent = `${times.sunrise.toLocaleTimeString()} | ${times.sunset.toLocaleTimeString()}`;
    }
    
    // Gravité
    $('time-dilation-grav').textContent = dataOrDefault(calculateTimeDilationGrav(lastKnownPos ? lastKnownPos.alt : 0, R_EARTH, currentGravityAcc), 2, ' ns/j');
    $('schwarzschild-radius').textContent = dataOrDefaultExp(calculateSchwarzschildRadius(currentMass), 10, ' m');

    slowLoopID = setTimeout(slowLoop, 1000);
};

/** Tente de synchroniser l'heure NTP (Simulation/Placeholder pour l'heure réelle). */
const syncH = () => {
    // En mode HORS LIGNE, on utilise l'heure locale du navigateur comme référence NTP
    $('local-time').textContent = 'HORS LIGNE (Navigateur)';
};


// --- GESTION DES ÉVÉNEMENTS DU DOM ---

/** Démarrer ou arrêter le GPS et les boucles. */
const toggleGPS = () => {
    if (!isGPSRunning) {
        if (!startTime) startTime = Date.now();
        
        const overrideAcc = parseFloat($('gps-accuracy-override').value);
        const options = {
            enableHighAccuracy: $('freq-select').value === 'HIGH_FREQ',
            timeout: 5000,
            maximumAge: 0
        };
        
        // Correction: Forcer l'UKF à se réinitialiser si la précision est forcée
        if (overrideAcc > 0) UKF_X = math.zeros(UKF_STATE_DIM)._data; 
        
        watchID = navigator.geolocation.watchPosition(getLocationSuccess, getLocationError, options);
        $('toggle-gps-btn').textContent = '⏸ PAUSE GPS';
        $('toggle-gps-btn').style.backgroundColor = '#ffc107';
        isGPSRunning = true;
        fastLoop();
        slowLoop();
    } else {
        navigator.geolocation.clearWatch(watchID);
        clearTimeout(slowLoopID);
        cancelAnimationFrame(fastLoopID);
        $('toggle-gps-btn').textContent = '▶️ MARCHE GPS';
        $('toggle-gps-btn').style.backgroundColor = '#28a745';
        isGPSRunning = false;
    }
};

/** Réinitialisation complète du système. */
const resetAll = () => {
    if (isGPSRunning) toggleGPS();
    
    // Réinitialisation des variables d'état
    startTime = 0;
    timeMoving = 0;
    currentSpeedMax = 0;
    distanceTotal = 0;
    trackPoints = [];
    if (trackPolyline) trackPolyline.setLatLngs([]);
    
    UKF_X = math.zeros(UKF_STATE_DIM)._data;
    UKF_P = math.multiply(math.eye(UKF_STATE_DIM), 10)._data;
    
    // Réinitialisation de l'affichage
    $('elapsed-time').textContent = '0.00 s';
    $('time-moving').textContent = '0.00 s';
    $('speed-max').textContent = '0.0 km/h';
    $('distance-total').textContent = '0.000 km | 0.00 m';
    
    console.log("Système réinitialisé.");
};


// --- INITIALISATION DES LISTENERS ET DÉMARRAGE ---
document.addEventListener('DOMContentLoaded', () => {
    // Boutons de contrôle
    $('toggle-gps-btn').addEventListener('click', toggleGPS);
    $('reset-all-btn').addEventListener('click', resetAll);
    $('reset-dist-btn').addEventListener('click', () => {
        distanceTotal = 0;
        trackPoints = [];
        if (trackPolyline) trackPolyline.setLatLngs([]);
    });
    $('reset-max-btn').addEventListener('click', () => {
        currentSpeedMax = 0;
    });

    // Sélecteurs
    $('environment-select').addEventListener('change', (e) => {
        selectedEnvironment = e.target.value;
        UKF_PROCESS_NOISE_MULT = ENVIRONMENT_FACTORS[selectedEnvironment].MULT;
        $('env-factor').textContent = `${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY} (x${UKF_PROCESS_NOISE_MULT.toFixed(1)})`;
    });
    
    // Corps Céleste
    $('celestial-body-select').addEventListener('change', (e) => {
        currentCelestialBody = e.target.value;
        updateCelestialBody(currentCelestialBody, lastKnownPos ? lastKnownPos.alt : 0, rotationRadius, angularVelocity);
    });

    // Données de rotation (Station Spatiale)
    const updateRotation = () => {
        rotationRadius = parseFloat($('rotation-radius').value) || 0;
        angularVelocity = parseFloat($('angular-velocity').value) || 0;
        if (currentCelestialBody === 'ROTATING') {
            const { G_ACC_NEW } = updateCelestialBody('ROTATING', lastKnownPos ? lastKnownPos.alt : 0, rotationRadius, angularVelocity);
            $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
        }
    };
    $('rotation-radius').addEventListener('input', updateRotation);
    $('angular-velocity').addEventListener('input', updateRotation);

    // Initialisation des capteurs IMU (demande de permissions)
    if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', handleDeviceMotion);
        window.addEventListener('deviceorientation', handleDeviceOrientation);
    } else {
        $('imu-status').textContent = 'IMU Non Supporté';
    }
    
    // Initialisation de l'API LightSensor
    if ('AmbientLightSensor' in window) {
        const sensor = new window.AmbientLightSensor();
        sensor.addEventListener('reading', handleAmbientLight);
        sensor.start();
    }
    
    // --- DÉMARRAGE DU SYSTÈME ---
    updateCelestialBody(currentCelestialBody, 0, rotationRadius, angularVelocity);
    syncH(); // Démarrer la synchro (HORS LIGNE)
    
    // Initialiser la carte (sans GPS)
    startMap();
    
    // Démarrer la boucle d'affichage initiale même si le GPS est éteint
    fastLoop();
    slowLoop();

});

// =================================================================
// FIN BLOC 5/5
// =================================================================

})(window);
