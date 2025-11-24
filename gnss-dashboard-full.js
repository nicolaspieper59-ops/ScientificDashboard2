// =================================================================
// BLOC 1/4 : Constantes, État Global et Utilitaires
// Fichier: constants_and_state.js
// Description: Définit les constantes fondamentales, les paramètres du système
//              et les variables d'état globales persistantes.
// =================================================================

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458.0;        // Vitesse de la lumière (m/s)
const G_CONST = 6.67430e-11;    // Constante gravitationnelle (N·m²/kg²)
const R_E_BASE = 6371000.0;     // Rayon terrestre moyen (m)
const KMH_MS = 3.6;             // Conversion m/s vers km/h
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const R_AIR = 287.058;          // Constante spécifique de l'air sec (J/kg·K)
const TEMP_SEA_LEVEL_K = 288.15; // 15°C en Kelvin
const BARO_ALT_REF_HPA = 1013.25; // Pression de référence (hPa)
const RHO_SEA_LEVEL = 1.225;    // Densité de l'air au niveau de la mer (kg/m³)
const DOM_SLOW_UPDATE_MS = 1000; // Fréquence de la boucle lente (1 Hz)

// --- PARAMÈTRES DU FILTRE DE KALMAN (UKF) ---
const Q_NOISE = 0.1;        // Bruit de processus (Position/Vitesse)
const R_MIN = 0.01;         // Bruit de mesure minimum (Vitesse)
const MAX_ACC_UNCERT = 200.0; // Précision max (m) avant "Estimation Seule"
const MIN_SPD_THRESH = 0.05; // Vitesse minimale pour être considéré comme "en mouvement"

const ENVIRONMENT_FACTORS = {
    'NORMAL': { MULT: 1.0, DISPLAY: 'Normal' },
    'URBAN': { MULT: 0.8, DISPLAY: 'Urbain' },
    'DEGRADED': { MULT: 0.5, DISPLAY: 'Dégradé' },
};

const CELESTIAL_BODIES = {
    'EARTH': { display: 'Terre', gravity: 9.80665, radius: R_E_BASE, mass: 5.972e24 },
    'MOON': { display: 'Lune', gravity: 1.625, radius: 1737400, mass: 7.347e22 },
};

// --- ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE ---
let isGpsPaused = true; 
let isEmergencyStop = false;
let selectedEnvironment = 'NORMAL';
let selectedCelestialBody = 'EARTH';
let selectedUKFMode = 'AUTO'; 

let currentMass = 70.0;
let distanceRatio = 1.0; 
let totalDistanceM = 0.0;
let maxSpeedKmh = 0.0;
let timeMovingMs = 0;
let timeElapsedMs = 0;
let lastTimestamp = 0;
let gpsWatchId = null; 
let lastGpsTimestamp = 0;
let imuActive = false;
let minecraftTimeMs = 0; // Ajout de l'état Minecraft Clock

// UKF/EKF State (Simplifié pour la démo - Vitesse 1D dans P[3][3])
let ukfState = {
    x: [0.0, 0.0, 0.0, 0.0], 
    P: [[100.0, 0, 0, 0], [0, 100.0, 0, 0], [0, 0, 10.0, 0], [0, 0, 0, 10.0]] 
};
let lastEKFPosition = { lat: 43.2964, lon: 5.3697, alt: 0.0, heading: 0.0 }; 
let currentSpeedMs = 0.0;

// Météo et Correction Métrologique
let lastP_hPa = BARO_ALT_REF_HPA;
let lastT_K = TEMP_SEA_LEVEL_K;
let lastH_perc = 0.6; 
let currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound = 343.0; // Vitesse du son par défaut
let currentMap = null; // Instance Leaflet.js
let currentMarker = null; // Marqueur Leaflet.js
let localNTPOffsetMs = 0; // Simulation d'offset NTP

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);

/** Formate une valeur numérique ou retourne 'N/A' si elle est invalide. */
const dataOrDefault = (val, decimals, suffix = '', precisionThreshold = 0.00000001) => {
    if (val === undefined || val === null || isNaN(val)) {
        return 'N/A';
    }
    const absVal = Math.abs(val);
    
    if (absVal > 1e18 || (absVal < precisionThreshold && absVal !== 0)) {
        return val.toExponential(decimals) + suffix;
    }
    
    return val.toFixed(decimals) + suffix;
};
// =================================================================
// BLOC 2/4 : Calculs Physique, Relativiste et UKF Core
// Fichier: physics_ukf_core.js
// Description: Contient les modèles mathématiques (Relativité, Météo, Forces)
//              et la logique du filtre de Kalman.
// Dépendance: BLOC 1
// =================================================================

// --- CALCULS PHYSIQUES ET RELATIVISTES ---

/** Calcule le facteur de Lorentz (gamma). */
const getLorentzFactor = (v) => {
    if (v >= C_L) return Infinity;
    const ratioSq = (v * v) / (C_L * C_L);
    return 1.0 / Math.sqrt(1.0 - ratioSq);
};

/** Calcule la dilatation temporelle due à la vitesse (en nanosecondes/jour). */
const getTimeDilationSpeed = (v) => {
    if (v < MIN_SPD_THRESH) return 0.0;
    const gamma = getLorentzFactor(v);
    const daySeconds = 86400.0;
    return (gamma - 1.0) * daySeconds * 1e9;
};

/** Calcule la dilatation temporelle due à la gravité (en nanosecondes/jour). */
const getTimeDilationGravity = (altM, massKg = CELESTIAL_BODIES.EARTH.mass, radiusM = R_E_BASE) => {
    // Calcul basé sur l'approximation de faible champ
    const r = radiusM + altM;
    const schwarzschildFactor = G_CONST * massKg / (r * C_L * C_L);
    const daySeconds = 86400.0;
    return schwarzschildFactor * daySeconds * 1e9;
};

/** Calcule le Rayon de Schwarzschild (Rs). */
const getSchwarzschildRadius = (massKg) => {
    if (massKg <= 0) return 0.0;
    return (2 * G_CONST * massKg) / (C_L * C_L);
};

/** Calcule la densité de l'air (air humide). */
const getAirDensity = (p_hPa, t_K, h_perc = 0.6) => {
    const T_C = t_K - 273.15;
    const PV_SAT = 6.1078 * Math.pow(10, (7.5 * T_C) / (T_C + 237.3)) * 100; // Pa
    const PV_ACTUAL = h_perc * PV_SAT; // Pa
    const P_Pa = p_hPa * 100; // Pa
    const P_DRY = P_Pa - PV_ACTUAL;
    const R_W = 461.495; 
    const rho = (P_DRY / (R_AIR * t_K)) + (PV_ACTUAL / (R_W * t_K));
    return rho;
};

/** Calcule la vitesse du son. */
const getSpeedOfSound = (t_K) => {
    const GAMMA = 1.4; 
    return Math.sqrt(GAMMA * R_AIR * t_K);
};

/** Calcule la Force de Traînée. */
const getDragForce = (rho, v, A = 1.0, Cd = 0.47) => { 
    const q = 0.5 * rho * v * v;
    return q * A * Cd;
};

/** Calcule la Force de Coriolis. */
const getCoriolisForce = (v_ms, latDeg, massKg) => {
    const latRad = latDeg * D2R;
    const f = 2 * OMEGA_EARTH * Math.sin(latRad);
    return massKg * f * v_ms;
};

/** Calcule l'énergie cinétique (Classique). */
const getKineticEnergy = (massKg, v_ms) => {
    return 0.5 * massKg * v_ms * v_ms;
};

/** Calcule la puissance de Traînée (mécanique). */
const getDragPower = (dragForce, v_ms) => {
    return dragForce * v_ms; // Watts
};


// --- UKF/EKF CORE LOGIC (VITESSE SEULE) ---
/** Exécute une étape de prédiction et de mise à jour du filtre UKF/EKF (Vitesse). */
const runUKFStep = (positionData, deltaTime) => {
    // Le facteur d'environnement module l'incertitude perçue.
    const envFactor = ENVIRONMENT_FACTORS[selectedEnvironment].MULT;
    const R_MEASURE = Math.pow(Math.max(R_MIN, positionData.acc * envFactor), 2); 
    const Q_PROC = Math.pow(Q_NOISE * envFactor, 2); 
    
    // Simplification 1D pour la vitesse EKF (utilise l'index 3 de la matrice P)
    let v_old = currentSpeedMs;
    let P_old = ukfState.P[3][3]; 
    
    // Prédiction
    let v_pred = v_old;
    let P_pred = P_old + Q_PROC * deltaTime;
    
    // Mise à jour (Correction avec mesure GNSS)
    const K = P_pred / (P_pred + R_MEASURE);
    let v_new = v_pred + K * (positionData.spd - v_pred);
    let P_new = (1 - K) * P_pred;
    
    currentSpeedMs = v_new;
    ukfState.P[3][3] = P_new; 

    const kalmanUncertainty = Math.sqrt(P_new);

    // Bruit de mesure de la vitesse utilisé (pour debug)
    const speedR = R_MEASURE;

    return {
        speed: v_new,
        uncertainty: kalmanUncertainty,
        ekfStatus: kalmanUncertainty < MAX_ACC_UNCERT ? 'Actif' : 'Estimation Seule',
        R_noise: speedR,
    };
};
// =================================================================
// BLOC 3/4 : Affichage, Astro et Boucles de Mise à Jour
// Fichier: data_processing_and_dom.js
// Description: Gère le flux de données après les calculs et met à jour le DOM.
// Dépendance: BLOC 1, BLOC 2
// =================================================================

// --- FONCTIONS ASTRO ET HORLOGE ---

/** Calcule la date et l'heure corrigées par NTP (simulées par un offset). */
const getCorrectedTime = () => {
    // Utilisez Date.now() pour des données réelles
    return new Date(Date.now() + localNTPOffsetMs);
};

/** Met à jour l'affichage de l'heure locale, UTC, Minecraft et temps écoulé. */
const updateTimeDisplay = () => {
    const now = getCorrectedTime();
    const nowUTC = now.toUTCString().split(' ')[4];
    
    // Affichage de l'heure/date
    if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString('fr-FR');
    if ($('date-display')) $('date-display').textContent = `${now.toLocaleDateString('fr-FR')} ${nowUTC}`;
    if ($('date-astro')) $('date-astro').textContent = now.toLocaleDateString('fr-FR');

    // Calcul du temps écoulé et temps en mouvement
    const currentMs = Date.now();
    if (lastTimestamp === 0) {
        lastTimestamp = currentMs;
    }
    const deltaRealTime = currentMs - lastTimestamp;
    timeElapsedMs += deltaRealTime;
    lastTimestamp = currentMs;
    
    const formatTime = (ms) => {
        const totalSeconds = Math.floor(ms / 1000);
        const h = String(Math.floor(totalSeconds / 3600)).padStart(2, '0');
        const m = String(Math.floor((totalSeconds % 3600) / 60)).padStart(2, '0');
        const s = String(totalSeconds % 60).padStart(2, '0');
        return `${h}:${m}:${s}`;
    };

    $('elapsed-time').textContent = formatTime(timeElapsedMs);
    $('time-moving').textContent = formatTime(timeMovingMs);
    
    // Mise à jour de l'horloge Minecraft
    updateMinecraftClock(deltaRealTime);
};

/** Met à jour l'affichage de l'heure Minecraft. */
const updateMinecraftClock = (ms) => {
    const TICKS_PER_DAY = 24000;
    const MS_PER_TICK = 50;
    const TIME_SCALE = 72;
    
    minecraftTimeMs += ms * TIME_SCALE;
    minecraftTimeMs %= (TICKS_PER_DAY * MS_PER_TICK);
    
    const ticks = minecraftTimeMs / MS_PER_TICK;
    const hours = Math.floor((ticks / 1000) + 6) % 24; 
    const minutes = Math.floor((ticks % 1000) / (1000 / 60));
    const timeStr = `${String(hours).padStart(2, '0')}:${String(minutes).padStart(2, '0')}`;
    
    if ($('time-minecraft')) $('time-minecraft').textContent = timeStr;
};

/** Convertit des heures décimales en format HH:MM:SS. */
const formatHoursToTime = (hours) => {
    let h = Math.floor(hours);
    const sign = h >= 0 ? 1 : -1;
    h = Math.abs(h) % 24 * sign;
    if (h < 0) h += 24;
    const m = Math.floor((hours * 60) % 60);
    const s = Math.floor((hours * 3600) % 60) % 60;
    
    return `${String(Math.floor(h)).padStart(2, '0')}:${String(Math.floor(m)).padStart(2, '0')}:${String(Math.floor(s)).padStart(2, '0')}`;
};

/** Détermine le nom de la phase lunaire. */
const getMoonPhaseName = (phase) => {
    if (phase < 0.03 || phase >= 0.97) return 'Nouvelle Lune';
    if (phase < 0.22) return 'Croissant Montant';
    if (phase < 0.28) return 'Premier Quartier';
    if (phase < 0.47) return 'Lune Gibbeuse Montante';
    if (phase < 0.53) return 'Pleine Lune';
    if (phase < 0.72) return 'Lune Gibbeuse Décroissante';
    if (phase < 0.78) return 'Dernier Quartier';
    return 'Croissant Décroissant';
};


/** Mise à jour de l'astronomie (Utilise SunCalc.js). */
const updateAstro = (lat, lon) => {
    const now = getCorrectedTime();
    
    // NOTE: SunCalc.js est une librairie externe requise pour ces calculs.
    if (typeof SunCalc === 'undefined') {
        // Mettre à jour tous les champs Astro en N/A si SunCalc est absent
        const astroFields = ['sun-alt', 'sun-azimuth', 'moon-phase-name', 'moon-illuminated', 'sunrise-times', 'sunset-times', 'eot', 'mst', 'tst', 'noon-solar', 'day-duration'];
        astroFields.forEach(id => { if ($(id)) $(id).textContent = 'N/A (Lib. Astro manquante)'; });
        return; 
    }

    const times = SunCalc.getTimes(now, lat, lon);
    const sunPos = SunCalc.getPosition(now, lat, lon);
    const moonIllumination = SunCalc.getMoonIllumination(now);
    
    // 1. Soleil et Lune
    const altitudeSunDeg = sunPos.altitude * R2D;
    const azimuthSunDeg = (sunPos.azimuth * R2D + 180) % 360; 
    
    $('sun-alt').textContent = dataOrDefault(altitudeSunDeg, 2, '°');
    $('sun-azimuth').textContent = dataOrDefault(azimuthSunDeg, 2, '°');
    $('moon-phase-name').textContent = getMoonPhaseName(moonIllumination.phase);
    $('moon-illuminated').textContent = dataOrDefault(moonIllumination.fraction * 100, 1, ' %');
    $('sunrise-times').textContent = times.sunrise ? times.sunrise.toLocaleTimeString('fr-FR') : 'N/A';
    $('sunset-times').textContent = times.sunset ? times.sunset.toLocaleTimeString('fr-FR') : 'N/A';
    
    // 2. Temps Solaire Vrai (TST), Temps Solaire Moyen (MST) et Équation du Temps (EOT)
    const nowHours = now.getUTCHours() + now.getUTCMinutes() / 60.0 + now.getUTCSeconds() / 3600.0;
    const lonHours = lon / 15.0;
    
    // Utiliser SunCalc pour obtenir l'équation du temps plus précisément (différence entre midi solaire vrai et midi solaire moyen)
    const solarNoon = times.solarNoon.getUTCHours() + times.solarNoon.getUTCMinutes() / 60.0;
    const meanNoon = 12.0 - lonHours;
    const EOT_hours = solarNoon - meanNoon;
    const EOT_min = EOT_hours * 60;
    
    const MST_hours = nowHours + lonHours;
    const TST_hours = nowHours + lonHours + EOT_hours; // TST = UTC + L + EOT
    
    $('eot').textContent = dataOrDefault(EOT_min, 2, ' min');
    $('mst').textContent = formatHoursToTime(MST_hours);
    $('tst').textContent = formatHoursToTime(TST_hours);
    $('noon-solar').textContent = formatHoursToTime(12.0 - lonHours);

    // Durée du Jour
    if(times.sunrise && times.sunset) {
        const durationMs = times.sunset.getTime() - times.sunrise.getTime();
        const h = Math.floor(durationMs / 3600000);
        const m = Math.floor((durationMs % 3600000) / 60000);
        $('day-duration').textContent = `${h}h ${String(m).padStart(2, '0')}m`;
    }
};


// --- BOUCLES DE MISE À JOUR ---

/** Boucle rapide (Déclenchée par GNSS) : UKF, Vitesse, Relativité, Distance */
const runFastLoop = (positionData, deltaTime) => {
    if (isEmergencyStop || isGpsPaused) return;

    const ekfResults = runUKFStep(positionData, deltaTime);
    const v_stable = ekfResults.speed;
    const v_kmh = v_stable * KMH_MS;
    const v_raw = positionData.spd;
    
    // Calculs Physiques
    const gamma = getLorentzFactor(v_stable);
    const restEnergy = currentMass * C_L * C_L;
    const totalEnergy = restEnergy * gamma;
    const momentum = currentMass * v_stable * gamma;
    const dilationSpeed = getTimeDilationSpeed(v_stable);
    const dilationGravity = getTimeDilationGravity(lastEKFPosition.alt, CELESTIAL_BODIES[selectedCelestialBody].mass, CELESTIAL_BODIES[selectedCelestialBody].radius);

    // Distance et Vitesse Max
    const distanceDelta = v_stable * deltaTime * distanceRatio;
    totalDistanceM += distanceDelta;
    if (v_stable >= MIN_SPD_THRESH) {
        timeMovingMs += deltaTime * 1000;
        maxSpeedKmh = Math.max(maxSpeedKmh, v_kmh);
    }
    
    // Mise à jour des affichages rapides (Vitesse, Relativité, EKF)
    $('speed-stable').textContent = dataOrDefault(v_kmh, 1, ' km/h');
    $('speed-stable-ms').textContent = dataOrDefault(v_stable, 2, ' m/s');
    $('vitesse-3d-inst').textContent = dataOrDefault(v_raw * KMH_MS, 1, ' km/h');
    $('vitesse-brute-ms').textContent = dataOrDefault(v_raw, 2, ' m/s');
    $('speed-max').textContent = dataOrDefault(maxSpeedKmh, 1, ' km/h');
    
    $('perc-speed-c').textContent = dataOrDefault(v_stable / C_L * 100, 2, ' %', 1e-10);
    $('lorentz-factor').textContent = dataOrDefault(gamma, 4);
    $('energy-relativistic').textContent = dataOrDefault(totalEnergy, 2, ' J', 1e-18);
    $('energy-rest-mass').textContent = dataOrDefault(restEnergy, 2, ' J', 1e-18);
    $('momentum').textContent = dataOrDefault(momentum, 2, ' kg·m/s');
    $('Rs-object').textContent = dataOrDefault(getSchwarzschildRadius(currentMass), 3, ' m', 1e-25);
    $('time-dilation-speed').textContent = dataOrDefault(dilationSpeed, 2, ' ns/j');
    $('time-dilation-gravity').textContent = dataOrDefault(dilationGravity, 2, ' ns/j');
    $('distance-total-km').textContent = `${dataOrDefault(totalDistanceM / 1000, 3, ' km')} | ${dataOrDefault(totalDistanceM, 2, ' m')}`;
    
    $('gps-precision').textContent = dataOrDefault(positionData.acc, 2, ' m');
    $('kalman-uncert').textContent = dataOrDefault(ekfResults.uncertainty, 4, ' m/s');
    $('gps-status-ekf').textContent = ekfResults.ekfStatus;
    $('bruit-mesure-vitesse-r').textContent = dataOrDefault(ekfResults.R_noise, 4, ' m²/s²');
};

/** Boucle lente (1000ms) : Météo, Forces, Moyennes, Astro */
const runSlowLoop = () => {
    // 1. Calculs Météo/Air et Mach
    const T_C = lastT_K - 273.15;
    currentAirDensity = getAirDensity(lastP_hPa, lastT_K, lastH_perc);
    currentSpeedOfSound = getSpeedOfSound(lastT_K);
    const machNumber = currentSpeedMs / currentSpeedOfSound;
    
    $('temp-air-2').textContent = dataOrDefault(T_C, 1, ' °C');
    $('pressure-2').textContent = dataOrDefault(lastP_hPa, 0, ' hPa');
    $('humidity-2').textContent = dataOrDefault(lastH_perc * 100, 0, ' %');
    $('air-density-calc').textContent = dataOrDefault(currentAirDensity, 3, ' kg/m³');
    $('speed-of-sound-calc').textContent = dataOrDefault(currentSpeedOfSound, 2, ' m/s (Cor.)');
    $('mach-number').textContent = dataOrDefault(machNumber, 4);

    // 2. Calculs Dynamique et Forces
    const q_dyn = 0.5 * currentAirDensity * currentSpeedMs * currentSpeedMs;
    const dragForce = getDragForce(currentAirDensity, currentSpeedMs, 1.0, 0.47); 
    const dragPower = getDragPower(dragForce, currentSpeedMs) / 1000; // kW
    const kineticEnergy = getKineticEnergy(currentMass, currentSpeedMs);
    const coriolisForce = getCoriolisForce(currentSpeedMs, lastEKFPosition.lat, currentMass);
    
    $('dynamic-pressure').textContent = dataOrDefault(q_dyn, 2, ' Pa');
    $('drag-force').textContent = dataOrDefault(dragForce, 2, ' N');
    $('drag-power-kw').textContent = dataOrDefault(dragPower, 2, ' kW');
    $('kinetic-energy').textContent = dataOrDefault(kineticEnergy, 2, ' J');
    $('coriolis-force').textContent = dataOrDefault(coriolisForce, 2, ' N');
    
    // 3. Vitesses moyennes
    const totalSeconds = timeElapsedMs / 1000;
    const totalMovingSeconds = timeMovingMs / 1000;
    const avgSpeedTotalMs = (totalSeconds > 0) ? totalDistanceM / totalSeconds : 0;
    const avgSpeedMovingMs = (totalMovingSeconds > 0) ? totalDistanceM / totalMovingSeconds : 0;
    $('vitesse-moyenne-totale').textContent = dataOrDefault(avgSpeedTotalMs * KMH_MS, 1, ' km/h');
    $('vitesse-moyenne-mvt').textContent = dataOrDefault(avgSpeedMovingMs * KMH_MS, 1, ' km/h');

    // 4. Mise à jour de l'Astro
    updateAstro(lastEKFPosition.lat, lastEKFPosition.lon);
};
// =================================================================
// BLOC 4/4 : Gestion des Capteurs et Initialisation
// Fichier: sensor_and_init.js
// Description: Gère l'interface avec les capteurs réels (Geolocation, IMU)
//              et initialise le tableau de bord.
// Dépendance: BLOC 1, BLOC 3
// =================================================================

// --- GESTION DE LA CARTE (Leaflet.js) ---

/** Initialise la carte Leaflet. */
const initMap = () => {
    if (typeof L === 'undefined') {
        $('globe-x').textContent = 'Erreur: Leaflet.js non chargé.';
        return;
    }

    if (currentMap) {
        currentMap.remove();
    }
    
    currentMap = L.map('map-container').setView([lastEKFPosition.lat, lastEKFPosition.lon], 13);
    
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; OpenStreetMap',
        maxZoom: 19,
    }).addTo(currentMap);
    
    currentMarker = L.marker([lastEKFPosition.lat, lastEKFPosition.lon]).addTo(currentMap)
        .bindPopup("Position initiale EKF").openPopup();
        
    $('globe-x').textContent = 'Carte GNSS/Globe Interactif'; // Remplacer le message de chargement
};


/** Met à jour la carte Leaflet. */
const updateMap = (lat, lon, heading) => {
    if (currentMap && currentMarker) {
        const newLatLng = new L.LatLng(lat, lon);
        currentMarker.setLatLng(newLatLng).update();
        currentMap.setView(newLatLng, currentMap.getZoom(), {animate: true, duration: 0.5}); 
    }
};

// --- GESTION DES CAPTEURS RÉELS (GNSS & IMU) ---

/** Traite les données GNSS reçues par watchPosition. */
const handleGpsUpdate = (position) => {
    if (isGpsPaused || isEmergencyStop) return;

    const currentGpsTimestamp = position.timestamp;
    const deltaTime = lastGpsTimestamp > 0 ? (currentGpsTimestamp - lastGpsTimestamp) / 1000 : 0.1;
    lastGpsTimestamp = currentGpsTimestamp;

    const realPositionData = {
        lat: position.coords.latitude,
        lon: position.coords.longitude,
        alt: position.coords.altitude || 0.0, 
        spd: position.coords.speed || 0.0,  
        acc: position.coords.accuracy,
        heading: position.coords.heading || lastEKFPosition.heading 
    };

    // Mise à jour de l'état EKF et de la boucle rapide
    lastEKFPosition.lat = realPositionData.lat;
    lastEKFPosition.lon = realPositionData.lon;
    lastEKFPosition.alt = realPositionData.alt;
    lastEKFPosition.heading = realPositionData.heading;
    
    runFastLoop(realPositionData, deltaTime);
    
    // Mise à jour de la carte et du DOM
    updateMap(realPositionData.lat, realPositionData.lon, realPositionData.heading);
    $('latitude-ekf').textContent = dataOrDefault(realPositionData.lat, 6, '°');
    $('longitude-ekf').textContent = dataOrDefault(realPositionData.lon, 6, '°');
    $('altitude-ekf').textContent = dataOrDefault(realPositionData.alt, 2, ' m');
    $('cap-direction').textContent = dataOrDefault(realPositionData.heading, 1, '°');
    $('gps-precision-acc').textContent = dataOrDefault(realPositionData.acc, 2, ' m');
    $('speed-status-text').textContent = 'Signal OK';
};

/** Gère les erreurs de l'API Geolocation. */
const handleGpsError = (error) => {
    console.error(`Erreur GPS (${error.code}): ${error.message}`);
    $('speed-status-text').textContent = `ERREUR GPS: ${error.message.substring(0, 30)}`;
    if (error.code === 1 || error.code === 2) {
        if (!isGpsPaused) toggleGPS(); 
    }
};

/** Traite les données de l'IMU (Accéléromètre/Gyroscope). */
const handleImuUpdate = (event) => {
    imuActive = true;
    $('sensor-status').textContent = 'Actif'; 
    
    const acc = event.accelerationIncludingGravity || { x: NaN, y: NaN, z: NaN };
    const rot = event.rotationRate || { alpha: NaN, beta: NaN, gamma: NaN };
    const gravity = CELESTIAL_BODIES[selectedCelestialBody].gravity;

    // Accélération et Force G
    $('accel-x').textContent = dataOrDefault(acc.x, 3, ' m/s²');
    $('accel-y').textContent = dataOrDefault(acc.y, 3, ' m/s²');
    $('accel-z').textContent = dataOrDefault(acc.z, 3, ' m/s²');
    $('accel-vertical-imu').textContent = dataOrDefault(acc.z, 3, ' m/s²');
    
    const longAcc = Math.sqrt(acc.x * acc.x + acc.y * acc.y);
    $('force-g-long').textContent = dataOrDefault(longAcc / gravity, 2, ' G');
    $('force-g-verticale').textContent = dataOrDefault((acc.z - gravity) / gravity, 2, ' G'); 
    
    const angularSpeed = Math.sqrt(rot.alpha * rot.alpha + rot.beta * rot.beta + rot.gamma * rot.gamma);
    $('angular-velocity-gyro').textContent = dataOrDefault(angularSpeed, 2, ' rad/s');
    
    // Note: Le magnétomètre nécessite DeviceOrientationEvent (non inclus dans cet exemple).
};

const initImuListener = () => {
    if (typeof window.DeviceMotionEvent !== 'undefined') {
        window.addEventListener('devicemotion', handleImuUpdate);
        $('sensor-status').textContent = 'Actif (Attente données)';
    } else {
        $('sensor-status').textContent = 'Inactif (API Capteur manquante)';
    }
};

// --- LOGIQUE DE CONTRÔLE ET INITIALISATION ---

/** Démarre l'acquisition GPS via watchPosition. */
const startGpsAcquisition = () => {
    if (typeof navigator.geolocation === 'undefined') {
        alert("L'API de Géolocalisation n'est pas supportée par ce navigateur.");
        return;
    }
    
    if (gpsWatchId !== null) {
        navigator.geolocation.clearWatch(gpsWatchId);
    }
    
    const gpsOptions = {
        enableHighAccuracy: true,
        timeout: 10000,
        maximumAge: 1000 
    };

    gpsWatchId = navigator.geolocation.watchPosition(handleGpsUpdate, handleGpsError, gpsOptions);
    isGpsPaused = false;
    $('toggle-gps-btn').innerHTML = '⏸️ PAUSE GPS';
    $('toggle-gps-btn').style.backgroundColor = '#ffc107'; 
    $('speed-status-text').textContent = 'Acquisition GPS...';
};

/** Gère le bouton MARCHE/PAUSE GPS. */
const toggleGPS = () => {
    if (isGpsPaused) {
        startGpsAcquisition();
    } else {
        isGpsPaused = true;
        if (gpsWatchId !== null) {
             navigator.geolocation.clearWatch(gpsWatchId);
             gpsWatchId = null;
        }
        $('toggle-gps-btn').innerHTML = '▶️ MARCHE GPS';
        $('toggle-gps-btn').style.backgroundColor = '#28a745';
        $('speed-status-text').textContent = 'PAUSE GPS';
    }
};

const resetDistance = () => {
    totalDistanceM = 0.0;
    $('distance-total-km').textContent = `${dataOrDefault(0, 3, ' km')} | ${dataOrDefault(0, 2, ' m')}`;
};
const resetVmax = () => {
    maxSpeedKmh = 0.0;
    $('speed-max').textContent = dataOrDefault(0, 1, ' km/h');
};
const resetAll = () => {
    resetDistance();
    resetVmax();
    timeMovingMs = 0;
    timeElapsedMs = 0;
    currentSpeedMs = 0.0;
    ukfState = {
        x: [0.0, 0.0, 0.0, 0.0], 
        P: [[100.0, 0, 0, 0], [0, 100.0, 0, 0], [0, 0, 10.0, 0], [0, 0, 0, 10.0]] 
    };
    $('elapsed-time').textContent = '00:00:00';
    $('time-moving').textContent = '00:00:00';
    $('kalman-uncert').textContent = dataOrDefault(Math.sqrt(ukfState.P[3][3]), 4, ' m/s');
    // Forcer la mise à jour complète du DOM
    runSlowLoop(); 
};


/** Point d'entrée principal. */
const initDashboard = () => {
    // 1. Initialisation des listeners
    $('toggle-gps-btn').addEventListener('click', toggleGPS);
    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', resetDistance);
    if ($('reset-vmax-btn')) $('reset-vmax-btn').addEventListener('click', resetVmax);
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', resetAll);

    // 2. Initialisation des Capteurs et de la Carte
    initImuListener();
    initMap();
    
    // 3. Démarrage des boucles
    updateTimeDisplay();
    runSlowLoop();
    setInterval(updateTimeDisplay, DOM_SLOW_UPDATE_MS);
    setInterval(runSlowLoop, DOM_SLOW_UPDATE_MS); 
    
    // 4. Affichage initial des valeurs par défaut
    $('toggle-gps-btn').style.backgroundColor = '#28a745';
    $('speed-status-text').textContent = 'Attente du signal GPS...';
    $('mass-display').textContent = dataOrDefault(currentMass, 3, ' kg');
    $('gravity-base').textContent = dataOrDefault(CELESTIAL_BODIES[selectedCelestialBody].gravity, 4, ' m/s²');
    $('kalman-uncert').textContent = dataOrDefault(Math.sqrt(ukfState.P[3][3]), 4, ' m/s');
};

window.addEventListener('load', initDashboard);
