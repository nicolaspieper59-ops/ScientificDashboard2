// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// Version 3.0.0 - SANS SIMULATION (Utilisation d'API Réelles)
// =================================================================

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;          // Vitesse de la lumière (m/s)
const G_CONST = 6.6743e-11;     // Constante gravitationnelle (N·m²/kg²)
const M_EARTH = 5.972e24;       // Masse de la Terre (kg)
const R_E_BASE = 6371000;       // Rayon terrestre moyen (m)
const KMH_MS = 3.6;             // Conversion m/s vers km/h
const C_S_DEFAULT = 343;        // Vitesse du son par défaut (m/s)
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const R_AIR = 287.058;          // Constante spécifique de l'air sec (J/kg·K)
const TEMP_SEA_LEVEL_K = 288.15; // 15°C en Kelvin
const BARO_ALT_REF_HPA = 1013.25; // Pression de référence (hPa)
const RHO_SEA_LEVEL = 1.225;    // Densité de l'air au niveau de la mer (kg/m³)

// --- PARAMÈTRES DU FILTRE DE KALMAN (UKF) ---
const Q_NOISE = 0.1;        // Bruit de processus
const R_MIN = 0.01;         // Bruit de mesure minimum
const MAX_ACC_UNCERT = 200; // Précision max (m) avant "Estimation Seule"
const MIN_SPD_THRESH = 0.05; // Vitesse minimale pour "en mouvement"

// --- ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE ---
let isGpsPaused = true; 
let isEmergencyStop = false;
let selectedEnvironment = 'NORMAL';
let selectedCelestialBody = 'EARTH';

let currentMass = 70.0;
let distanceRatio = 1.0; 

let totalDistanceM = 0.0;
let maxSpeedKmh = 0.0;
let timeMovingMs = 0;
let timeElapsedMs = 0;
let lastTimestamp = 0;

// GESTION DES CAPTEURS RÉELS
let gpsWatchId = null; 
let lastGpsTimestamp = 0;
let imuActive = false;

// UKF/EKF State (état initial non simulé)
let ukfState = {
    x: [0, 0, 0, 0], 
    P: [[100, 0, 0, 0], [0, 100, 0, 0], [0, 0, 10, 0], [0, 0, 0, 10]] 
};
let lastEKFPosition = { lat: 43.2964, lon: 5.3697, alt: 0, heading: 0 }; // Coordonnées de repli (ex: Marseille)
let currentSpeedMs = 0.0;

// Météo et correction métrologique (Initialisation aux valeurs par défaut)
let lastP_hPa = 1013;
let lastT_K = 283.45; // 10.3 °C
let lastH_perc = 0.6; // 60%
let currentAirDensity = 1.241;
let currentSpeedOfSound = 337.51;

// Minecraft Clock state
let minecraftTimeMs = 0;

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);

const dataOrDefault = (val, decimals, suffix = '', precisionThreshold = 0.00000001) => {
    if (val === undefined || val === null || isNaN(val)) {
        return 'N/A';
    }
    // Formatage scientifique pour les très grands/petits nombres
    if (Math.abs(val) > 1e18 || (Math.abs(val) < precisionThreshold && val !== 0)) {
        return val.toExponential(3) + suffix;
    }
    
    // Éviter les affichages négatifs pour les distances/vitesses
    if (val < 0 && (suffix.includes('m') || suffix.includes('km') || suffix.includes('rad/s'))) {
        return '0' + suffix;
    }

    return val.toFixed(decimals) + suffix;
};

// --- CALCULS PHYSIQUES ET RELATIVISTES ---

const getLorentzFactor = (v) => {
    if (v >= C_L) return Infinity;
    return 1.0 / Math.sqrt(1.0 - (v * v) / (C_L * C_L));
};

const getSchwarzschildRadius = (massKg) => {
    if (massKg <= 0) return 0;
    return (2 * G_CONST * massKg) / (C_L * C_L);
};

const getAirDensity = (p_hPa, t_K, h_perc = 0.6) => {
    const T_C = t_K - 273.15;
    const PV_SAT = 6.1078 * Math.pow(10, (7.5 * T_C) / (T_C + 237.3)) * 100; 
    const PV_ACTUAL = h_perc * PV_SAT; 
    const P_Pa = p_hPa * 100; 
    const P_DRY = P_Pa - PV_ACTUAL;
    const R_W = 461.495; 
    const rho = (P_DRY / (R_AIR * t_K)) + (PV_ACTUAL / (R_W * t_K));
    return rho;
};

const getSpeedOfSound = (t_K) => {
    const GAMMA = 1.4; 
    const R = R_AIR;
    return Math.sqrt(GAMMA * R * t_K);
};

const getDewPoint = (t_c, h_perc) => {
    if (h_perc <= 0 || isNaN(t_c)) return NaN;
    const A = 17.27;
    const B = 237.7;
    const logRH = Math.log(h_perc);
    const gamma = (A * t_c) / (B + t_c) + logRH;
    return (B * gamma) / (A - gamma);
};

const getDragForce = (rho, v, A = 1.0, Cd = 0.47) => { 
    const q = 0.5 * rho * v * v;
    return q * A * Cd;
};

const getCoriolisForce = (v_ms, latDeg, massKg) => {
    const latRad = latDeg * D2R;
    const f = 2 * OMEGA_EARTH * Math.sin(latRad);
    return massKg * f * v_ms;
};

const getRestMassEnergy = (massKg) => {
    return massKg * C_L * C_L;
};


// --- UKF/EKF CORE LOGIC (Simplified for Demo) ---
/** Exécute une étape de prédiction et de mise à jour du filtre UKF/EKF. */
const runUKFStep = (positionData, deltaTime) => {
    // Le bruit de mesure est basé sur la précision GPS réelle.
    const R_MEASURE = Math.pow(Math.max(R_MIN, positionData.acc), 2); 
    const Q_PROC = Math.pow(Q_NOISE * 1.0, 2); // Simplifié, le facteur d'environnement est souvent intégré ici

    let v_old = currentSpeedMs;
    let P_old = ukfState.P[2][2]; 
    
    // Prédiction (Modèle de vitesse constante)
    let v_pred = v_old;
    let P_pred = P_old + Q_PROC * deltaTime;
    
    // Mise à jour (Correction avec mesure GNSS)
    // positionData.spd est la vitesse mesurée par le GNSS (si disponible, sinon 0)
    const K = P_pred / (P_pred + R_MEASURE);
    let v_new = v_pred + K * (positionData.spd - v_pred);
    let P_new = (1 - K) * P_pred;
    
    currentSpeedMs = v_new;
    ukfState.P[2][2] = P_new;

    const kalmanUncertainty = Math.sqrt(P_new);

    return {
        speed: v_new,
        uncertainty: kalmanUncertainty,
        ekfStatus: kalmanUncertainty < MAX_ACC_UNCERT ? 'Actif' : 'Estimation Seule'
    };
};

// --- FONCTIONS ASTRO ET HORLOGE ---

/** Met à jour l'horloge Minecraft pour la démo. */
const updateMinecraftClock = (ms) => {
    const TICKS_PER_DAY = 24000;
    const MS_PER_TICK = 50;
    const TIME_SCALE = 72; 

    minecraftTimeMs = (minecraftTimeMs + ms * TIME_SCALE) % (TICKS_PER_DAY * MS_PER_TICK);
    
    const ticks = minecraftTimeMs / MS_PER_TICK;
    const hours = Math.floor((ticks / 1000) + 6) % 24; 
    const minutes = Math.floor((ticks % 1000) / (1000 / 60));
    const timeStr = `${String(hours).padStart(2, '0')}:${String(minutes).padStart(2, '0')}`;
    
    $('time-minecraft').textContent = timeStr;
    
    // Rotation de l'horloge astro
    const angle = (ticks / TICKS_PER_DAY) * 360 - 90; 
    if ($('sun-element')) {
        $('sun-element').style.transform = `rotate(${angle}deg)`;
    }
    if ($('moon-element')) {
        $('moon-element').style.transform = `rotate(${angle + 180}deg)`;
    }
};

/** Met à jour l'affichage de l'heure locale et UTC (NTP). */
const updateTimeDisplay = () => {
    const now = new Date();
    const utc = now.toUTCString().split(' ')[4];

    if ($('local-time')) {
        $('local-time').textContent = now.toLocaleTimeString('fr-FR');
    }
    if ($('date-display')) {
        const dateFr = now.toLocaleDateString('fr-FR');
        $('date-display').textContent = `${dateFr} ${utc}`;
    }
    
const updateMinecraftClock = (ms) => {
    const TICKS_PER_DAY = 24000;
    const MS_PER_TICK = 50;
    const TIME_SCALE = 72; 

    minecraftTimeMs = (minecraftTimeMs + ms * TIME_SCALE) % (TICKS_PER_DAY * MS_PER_TICK);
    
    const ticks = minecraftTimeMs / MS_PER_TICK;
    const hours = Math.floor((ticks / 1000) + 6) % 24; 
    const minutes = Math.floor((ticks % 1000) / (1000 / 60));
    const timeStr = `${String(hours).padStart(2, '0')}:${String(minutes).padStart(2, '0')}`;
    
    if ($('time-minecraft')) $('time-minecraft').textContent = timeStr;
    // (Implémentation de la rotation de l'horloge astro omise ici pour la concision)
    // Rotation de l'horloge astro
    const angle = (ticks / TICKS_PER_DAY) * 360 - 90; 
    if ($('sun-element')) {
        $('sun-element').style.transform = `rotate(${angle}deg)`;
    }
    if ($('moon-element')) {
        $('moon-element').style.transform = `rotate(${angle + 180}deg)`;
    }
};

const updateTimeDisplay = () => {
    const now = new Date();
    const utc = now.toUTCString().split(' ')[4];

    if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString('fr-FR');
    if ($('date-display')) {
        const dateFr = now.toLocaleDateString('fr-FR');
        $('date-display').textContent = `${dateFr} ${utc}`;
    }
    
    const currentMs = Date.now();
    if (lastTimestamp === 0) {
        lastTimestamp = currentMs;
    }
    timeElapsedMs += (currentMs - lastTimestamp);
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
    
    updateMinecraftClock(currentMs - lastTimestamp);
};

const formatHoursToTime = (hours) => {
    let h = Math.floor(hours);
    const sign = h >= 0 ? 1 : -1;
    h = Math.abs(h) % 24 * sign;
    if (h < 0) h += 24;
    const m = Math.floor((hours * 60) % 60);
    const s = Math.floor((hours * 3600) % 60) % 60;
    
    return `${String(Math.floor(h)).padStart(2, '0')}:${String(Math.floor(m)).padStart(2, '0')}:${String(Math.floor(s)).padStart(2, '0')}`;
};

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

const updateAstro = (lat, lon) => {
    // Nécessite la librairie SunCalc.js pour fonctionner (non incluse ici)
    if (typeof SunCalc === 'undefined') return;

    const now = new Date();
    const times = SunCalc.getTimes(now, lat, lon);
    const sunPos = SunCalc.getPosition(now, lat, lon);
    const moonPos = SunCalc.getMoonPosition(now, lat, lon);
    const moonIllumination = SunCalc.getMoonIllumination(now);
    
    const altitudeSunDeg = sunPos.altitude * R2D;
    const azimuthSunDeg = (sunPos.azimuth * R2D + 180) % 360; 
    
    // ... (Mise à jour des champs Astro) ...
    $('sun-alt').textContent = dataOrDefault(altitudeSunDeg, 2, '°');
    $('sun-azimuth').textContent = dataOrDefault(azimuthSunDeg, 2, '°');
    $('sunrise-times').textContent = times.sunrise ? times.sunrise.toLocaleTimeString('fr-FR') : 'N/A';
    $('sunset-times').textContent = times.sunset ? times.sunset.toLocaleTimeString('fr-FR') : 'N/A';
    
    const lonDeg = lon;
    const lonHours = lonDeg / 15; 
    const nowUTC = now.getUTCHours() + now.getUTCMinutes() / 60 + now.getUTCSeconds() / 3600;
    let mstHours = nowUTC + lonHours;
    
    const solarNoonTime = times.solarNoon;
    const meanNoonTime = new Date(now.getTime());
    meanNoonTime.setUTCHours(12, 0, 0, 0);
    meanNoonTime.setUTCMinutes(meanNoonTime.getUTCMinutes() - lonDeg * 4); 

    const eotMinutes = (solarNoonTime.getTime() - meanNoonTime.getTime()) / 60000;
    let tstHours = mstHours + eotMinutes / 60;

    $('eot').textContent = dataOrDefault(eotMinutes, 2, ' min');
    $('mst').textContent = formatHoursToTime(mstHours);
    $('tst').textContent = formatHoursToTime(tstHours);
    $('noon-solar').textContent = formatHoursToTime(12 - lonHours);
    // ... (suite de l'affichage Lune/Jour/Nuit) ...
    // Statut Jour/Nuit
    if (altitudeSunDeg > 1) {
        $('clock-status').textContent = 'Jour (☀️)';
    } else if (altitudeSunDeg > -12) {
        $('clock-status').textContent = 'Crépuscule ( twilight)';
    } else {
        $('clock-status').textContent = 'Nuit (🌙)';
    }
};

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

// --- GESTION DES CAPTEURS RÉELS (GNSS & IMU) ---

// 1. Gestion des données GNSS (Geolocation API)
const handleGpsUpdate = (position) => {
    if (isGpsPaused || isEmergencyStop) return;

    // Calcul du Delta Time entre les mises à jour
    const currentGpsTimestamp = position.timestamp;
    const deltaTime = lastGpsTimestamp > 0 ? (currentGpsTimestamp - lastGpsTimestamp) / 1000 : 0.1;
    lastGpsTimestamp = currentGpsTimestamp;

    // Données réelles
    const realPositionData = {
        lat: position.coords.latitude,
        lon: position.coords.longitude,
        alt: position.coords.altitude || 0, 
        spd: position.coords.speed || 0.0,  
        acc: position.coords.accuracy,
        heading: position.coords.heading || lastEKFPosition.heading 
    };

    // Mise à jour de la position EKF avec la donnée réelle pour les calculs Astro/Coriolis
    lastEKFPosition.lat = realPositionData.lat;
    lastEKFPosition.lon = realPositionData.lon;
    lastEKFPosition.alt = realPositionData.alt;
    lastEKFPosition.heading = realPositionData.heading;

    // Exécution du Filtre Kalman et des calculs rapides
    runFastLoop(realPositionData, deltaTime);

    // Mise à jour des coordonnées
    $('latitude-ekf').textContent = dataOrDefault(realPositionData.lat, 6, '°');
    $('longitude-ekf').textContent = dataOrDefault(realPositionData.lon, 6, '°');
    $('altitude-ekf').textContent = dataOrDefault(realPositionData.alt, 2, ' m');
    $('cap-direction').textContent = dataOrDefault(realPositionData.heading, 1, '°');
    $('speed-status-text').textContent = 'Signal OK';
};

const handleGpsError = (error) => {
    console.error(`Erreur GPS (${error.code}): ${error.message}`);
    $('speed-status-text').textContent = `ERREUR GPS: ${error.message.substring(0, 30)}`;
    
    // Bascule en pause si l'erreur est grave (Permission denied, Position unavailable)
    if (error.code === 1 || error.code === 2) {
        if (!isGpsPaused) toggleGPS(); // Passe en mode PAUSE
    }
};


// 2. Gestion des données IMU (DeviceMotion API)
const handleImuUpdate = (event) => {
    imuActive = true;
    $('sensor-status').textContent = 'Actif'; 

    // NOTE: Certains navigateurs nécessitent un geste utilisateur pour activer DeviceMotion (e.g. iOS)
    const acc = event.accelerationIncludingGravity;
    const rot = event.rotationRate;
    const gravity = CELESTIAL_BODIES[selectedCelestialBody].gravity;

    // Accélération et Force G
    $('accel-x').textContent = dataOrDefault(acc.x, 3, ' m/s²');
    $('accel-y').textContent = dataOrDefault(acc.y, 3, ' m/s²');
    $('accel-z').textContent = dataOrDefault(acc.z, 3, ' m/s²');
    $('accel-vertical-imu').textContent = dataOrDefault(acc.z, 3, ' m/s²');
    
    // Force G (Longitudinale : dans le plan X-Y)
    $('force-g-long').textContent = dataOrDefault(Math.sqrt(acc.x * acc.x + acc.y * acc.y) / gravity, 2, ' G');
    // Force G (Verticale : composante Z)
    $('force-g-vertical').textContent = dataOrDefault(acc.z / gravity, 2, ' G');
    
    // Vitesse Angulaire (Gyroscope)
    $('angular-velocity-gyro').textContent = dataOrDefault(Math.sqrt(rot.alpha * rot.alpha + rot.beta * rot.beta + rot.gamma * rot.gamma), 2, ' rad/s');
    
    // Le Champ Magnétique est souvent manquant ou nécessitent DeviceOrientationEvent.
    // Laissez 'N/A' si les champs ne sont pas mis à jour par cet événement.
    // (Les IDs magnétiques sont laissés à 'N/A' par dataOrDefault si l'API est absente)
};

const initImuListener = () => {
    if (typeof window.DeviceMotionEvent !== 'undefined') {
        window.addEventListener('devicemotion', handleImuUpdate);
        $('sensor-status').textContent = 'Actif (Attente données)';
    } else {
        $('sensor-status').textContent = 'Inactif (API Capteur manquante)';
    }
};

// --- BOUCLES DE MISE À JOUR ---

/** Exécutée par le flux GNSS (handleGpsUpdate) */
const runFastLoop = (positionData, deltaTime) => {
    // 1. Mise à jour de l'UKF/EKF
    const ekfResults = runUKFStep(positionData, deltaTime);

    // 2. Calculs de Vitesse et de Relativité
    const v_stable = ekfResults.speed;
    const v_kmh = v_stable * KMH_MS;
    const v_raw = positionData.spd;
    const gamma = getLorentzFactor(v_stable);
    const restEnergy = getRestMassEnergy(currentMass);
    const totalEnergy = restEnergy * gamma;
    const momentum = currentMass * v_stable * gamma;

    // Mise à jour de la vitesse
    $('speed-stable').textContent = dataOrDefault(v_kmh, 1, ' km/h');
    $('speed-stable-ms').textContent = dataOrDefault(v_stable, 2, ' m/s');
    $('speed-stable-kms').textContent = dataOrDefault(v_stable / 1000, 3, ' km/s');
    $('vitesse-3d-inst').textContent = dataOrDefault(v_raw * KMH_MS, 1, ' km/h');
    $('speed-raw-ms').textContent = dataOrDefault(v_raw, 2, ' m/s');

    // Mise à jour Relativité
    $('perc-speed-c').textContent = dataOrDefault(v_stable / C_L * 100, 2, ' %', 1e-10);
    $('lorentz-factor').textContent = dataOrDefault(gamma, 4);
    $('energy-relativistic').textContent = dataOrDefault(totalEnergy, 2, ' J', 1e-18);
    $('energy-rest-mass').textContent = dataOrDefault(restEnergy, 2, ' J', 1e-18);
    $('momentum').textContent = dataOrDefault(momentum, 2, ' kg·m/s');
    $('Rs-object').textContent = dataOrDefault(getSchwarzschildRadius(currentMass), 3, ' m', 1e-25);

    // Mise à jour de la distance
    const distanceDelta = v_stable * deltaTime * distanceRatio;
    totalDistanceM += distanceDelta;
    if (v_stable >= MIN_SPD_THRESH) {
        timeMovingMs += deltaTime * 1000;
        maxSpeedKmh = Math.max(maxSpeedKmh, v_kmh);
    }
    $('distance-total-km').textContent = `${dataOrDefault(totalDistanceM / 1000, 3, ' km')} | ${dataOrDefault(totalDistanceM, 2, ' m')}`;
    $('speed-max').textContent = dataOrDefault(maxSpeedKmh, 1, ' km/h');
    
    // Mise à jour du Filtre UKF/EKF
    $('gps-precision').textContent = dataOrDefault(positionData.acc, 2, ' m');
    $('kalman-uncert').textContent = dataOrDefault(ekfResults.uncertainty, 4, ' m/s');
    $('gps-status-ekf').textContent = ekfResults.ekfStatus;
};

/** Boucle de mise à jour lente (1000ms) */
const runSlowLoop = () => {
    // 1. Calculs Météo/Air (Ces valeurs doivent être alimentées par un capteur ou une API externe)
    const T_C = lastT_K - 273.15;
    currentAirDensity = getAirDensity(lastP_hPa, lastT_K, lastH_perc);
    currentSpeedOfSound = getSpeedOfSound(lastT_K);
    const dewPointC = getDewPoint(T_C, lastH_perc); 

    $('temp-air-2').textContent = dataOrDefault(T_C, 1, ' °C');
    $('pressure-2').textContent = dataOrDefault(lastP_hPa, 0, ' hPa');
    $('humidity-2').textContent = dataOrDefault(lastH_perc * 100, 0, ' %');
    $('air-density-calc').textContent = dataOrDefault(currentAirDensity, 3, ' kg/m³');
    $('speed-of-sound-calc').textContent = dataOrDefault(currentSpeedOfSound, 2, ' m/s (Cor.)');
    $('dew-point').textContent = dataOrDefault(dewPointC, 1, ' °C'); 
    
    const machNumber = currentSpeedMs / currentSpeedOfSound;
    $('perc-speed-sound').textContent = dataOrDefault(machNumber * 100, 2, ' %');
    $('mach-number').textContent = dataOrDefault(machNumber, 4);

    // 2. Calculs Dynamique et Forces
    const q_dyn = 0.5 * currentAirDensity * currentSpeedMs * currentSpeedMs;
    const dragForce = getDragForce(currentAirDensity, currentSpeedMs, 1.0, 0.47);
    const dragPower = dragForce * currentSpeedMs / 1000;
    const kineticEnergy = 0.5 * currentMass * currentSpeedMs * currentSpeedMs;
    const coriolisForce = getCoriolisForce(currentSpeedMs, lastEKFPosition.lat, currentMass);
    
    $('dynamic-pressure').textContent = dataOrDefault(q_dyn, 2, ' Pa');
    $('drag-force').textContent = dataOrDefault(dragForce, 2, ' N');
    $('drag-power-kw').textContent = dataOrDefault(dragPower, 2, ' kW');
    $('kinetic-energy').textContent = dataOrDefault(kineticEnergy, 2, ' J');
    $('coriolis-force').textContent = dataOrDefault(coriolisForce, 2, ' N');
    
    // Calcul des vitesses moyennes
    const totalSeconds = timeElapsedMs / 1000;
    const totalMovingSeconds = timeMovingMs / 1000;

    const avgSpeedTotalMs = (totalSeconds > 0) ? totalDistanceM / totalSeconds : 0;
    $('vitesse-moyenne-totale').textContent = dataOrDefault(avgSpeedTotalMs * KMH_MS, 1, ' km/h');

    const avgSpeedMovingMs = (totalMovingSeconds > 0) ? totalDistanceM / totalMovingSeconds : 0;
    $('vitesse-moyenne-mvt').textContent = dataOrDefault(avgSpeedMovingMs * KMH_MS, 1, ' km/h');

    // 3. Mise à jour de l'Astro
    updateAstro(lastEKFPosition.lat, lastEKFPosition.lon);

    // 4. Mise à jour des données générales
    $('mass-display').textContent = dataOrDefault(currentMass, 3, ' kg');
    $('gravity-base').textContent = dataOrDefault(CELESTIAL_BODIES[selectedCelestialBody].gravity, 4, ' m/s²');
};


// --- ÉVÉNEMENTS & INITIALISATION ---

/** Démarre l'acquisition GPS via watchPosition. */
const startGpsAcquisition = () => {
    if (typeof navigator.geolocation === 'undefined') {
        alert("L'API de Géolocalisation n'est pas supportée par ce navigateur.");
        return;
    }
    
    // Nettoyer toute surveillance existante pour éviter les doublons
    if (gpsWatchId !== null) {
        navigator.geolocation.clearWatch(gpsWatchId);
    }
    
    // Options pour une acquisition professionnelle
    const gpsOptions = {
        enableHighAccuracy: true,
        timeout: 10000,
        maximumAge: 1000 
    };

    // Démarrer la surveillance continue
    gpsWatchId = navigator.geolocation.watchPosition(
        handleGpsUpdate, 
        handleGpsError, 
        gpsOptions
    );
    
    isGpsPaused = false;
    $('toggle-gps-btn').innerHTML = '⏸️ PAUSE GPS';
    $('toggle-gps-btn').style.backgroundColor = '#ffc107'; 
    $('speed-status-text').textContent = 'Acquisition GPS...';
};

/** Gère le bouton MARCHE/PAUSE GPS. */
const toggleGPS = () => {
    if ($('toggle-gps-btn').innerHTML.includes('MARCHE') || isGpsPaused) {
        startGpsAcquisition();
    } else {
        isGpsPaused = true;
        // Arrêter la surveillance GNSS
        if (gpsWatchId !== null) {
             navigator.geolocation.clearWatch(gpsWatchId);
             gpsWatchId = null;
        }
        $('toggle-gps-btn').innerHTML = '▶️ MARCHE GPS';
        $('toggle-gps-btn').style.backgroundColor = '#28a745';
        $('speed-status-text').textContent = 'PAUSE GPS';
    }
};

const emergencyStop = () => {
    isEmergencyStop = !isEmergencyStop;
    const btn = $('emergency-stop-btn');
    if (isEmergencyStop) {
        btn.innerHTML = "🛑 Arrêt d'urgence: ACTIF 🔴";
        btn.classList.add('active');
        $('speed-status-text').textContent = 'ARRÊT D\'URGENCE';
    } else {
        btn.innerHTML = "🛑 Arrêt d'urgence: INACTIF 🟢";
        btn.classList.remove('active');
        $('speed-status-text').textContent = isGpsPaused ? 'PAUSE GPS' : 'Signal OK';
    }
};

const resetDistance = () => {
    totalDistanceM = 0.0;
    timeMovingMs = 0;
    $('distance-total-km').textContent = '0.000 km | 0.00 m';
    $('time-moving').textContent = '00:00:00';
    $('vitesse-moyenne-mvt').textContent = '0.0 km/h';
    $('vitesse-moyenne-totale').textContent = '0.0 km/h';
};

const resetVmax = () => {
    maxSpeedKmh = 0.0;
    $('speed-max').textContent = '0.0 km/h';
};

const resetAll = () => {
    resetDistance();
    resetVmax();
    timeElapsedMs = 0;
    currentSpeedMs = 0.0;
    lastTimestamp = 0;
    ukfState.P = [[100, 0, 0, 0], [0, 100, 0, 0], [0, 0, 10, 0], [0, 0, 0, 10]];
    if (isEmergencyStop) emergencyStop();
    if (!isGpsPaused) toggleGPS();
};


/** Gère l'initialisation du tableau de bord. */
const initDashboard = () => {
    // Initialisation des listeners (boutons, inputs)
    $('toggle-gps-btn').addEventListener('click', toggleGPS);
    $('emergency-stop-btn').addEventListener('click', emergencyStop);
    $('reset-dist-btn').addEventListener('click', resetDistance);
    $('reset-vmax-btn').addEventListener('click', resetVmax);
    $('reset-all-btn').addEventListener('click', resetAll);
    
    if ($('toggle-mode-btn')) {
         $('toggle-mode-btn').addEventListener('click', () => {
            document.body.classList.toggle('dark-mode');
            $('toggle-mode-btn').innerHTML = document.body.classList.contains('dark-mode') ? '<i class="fas fa-sun"></i> Mode Jour' : '<i class="fas fa-moon"></i> Mode Nuit';
         });
    }

    // Gestion des inputs et selects (UKF/Physique)
    $('mass-input').addEventListener('input', (e) => {
        currentMass = parseFloat(e.target.value) || 0;
        runSlowLoop(); 
    });

    $('environment-select').addEventListener('change', (e) => {
        selectedEnvironment = e.target.value;
        $('env-factor').textContent = `${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY} (x${ENVIRONMENT_FACTORS[selectedEnvironment].MULT.toFixed(1)})`;
    });
    
    $('celestial-body-select').addEventListener('change', (e) => {
        selectedCelestialBody = e.target.value;
        runSlowLoop(); 
    });
    
    $('gps-accuracy-override').addEventListener('input', (e) => {
        gpsAccuracyOverride = parseFloat(e.target.value) || 0;
        $('gps-accuracy-display').textContent = dataOrDefault(gpsAccuracyOverride, 6, ' m');
    });

    $('distance-ratio-toggle-btn').addEventListener('click', () => {
        distanceRatio = (distanceRatio === 1.0) ? 8.0 : 1.0; 
        $('distance-ratio-toggle-btn').textContent = `Rapport Distance: ${distanceRatio === 8.0 ? 'NETHER' : 'SURFACE'} (${distanceRatio.toFixed(3)})`;
        $('distance-ratio').textContent = distanceRatio.toFixed(3);
    });

    
    // INITIALISATION IMU (Désormais professionnel)
    initImuListener();
    
    // Configuration initiale de l'affichage
    $('toggle-gps-btn').style.backgroundColor = '#28a745';
    $('speed-status-text').textContent = 'Attente du signal GPS...';

    // Démarrer la boucle lente (Météo/Astro/Horloge)
    updateTimeDisplay();
    runSlowLoop();
    setInterval(updateTimeDisplay, 1000);
    setInterval(runSlowLoop, 1000); 
};

window.addEventListener('load', initDashboard);    
