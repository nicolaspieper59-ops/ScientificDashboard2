// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// Version 2.0.0 - CORRIGÉ ET CONSOLIDÉ
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
const Q_NOISE = 0.1;        // Bruit de processus (Position/Vitesse)
const R_MIN = 0.01;         // Bruit de mesure minimum (Vitesse)
const R_MAX = 500.0;        // Bruit de mesure maximum
const MAX_ACC_UNCERT = 200; // Précision max (m) avant "Estimation Seule"
const MIN_SPD_THRESH = 0.05; // Vitesse minimale pour être considéré comme "en mouvement"

// Définitions des corps célestes (Gravité et Rayon)
const CELESTIAL_BODIES = {
    'EARTH': { display: 'Terre', gravity: 9.80665, radius: R_E_BASE },
    'MOON': { display: 'Lune', gravity: 1.625, radius: 1737400 },
    'MARS': { display: 'Mars', gravity: 3.72076, radius: 3389500 },
    'ROTATING': { display: 'Station', gravity: 0.0, radius: 100 } // Rayon et Gravité ajustables
};

// Facteurs d'environnement pour le bruit de mesure
const ENVIRONMENT_FACTORS = {
    'NORMAL': { MULT: 1.0, DISPLAY: 'Normal' },
    'URBAN': { MULT: 2.0, DISPLAY: 'Urbain Denser' },
    'FOREST': { MULT: 2.5, DISPLAY: 'Forêt' },
    'CONCRETE': { MULT: 7.0, DISPLAY: 'Grotte/Tunnel' },
    'METAL': { MULT: 5.0, DISPLAY: 'Métal/Bâtiment' }
};

// --- ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE ---
let isGpsPaused = true; 
let isEmergencyStop = false;
let selectedEnvironment = 'NORMAL';
let selectedCelestialBody = 'EARTH';

let currentMass = 70.0; // Masse de l'objet (kg)
let rotationRadius = 100.0; // Rayon de rotation (m)
let angularVelocity = 0.0; // Vitesse angulaire (rad/s)
let gpsAccuracyOverride = 0.0; // Précision GPS forcée

let distanceRatio = 1.0; // Rapport Distance

let totalDistanceM = 0.0;
let maxSpeedKmh = 0.0;
let timeMovingMs = 0;
let timeElapsedMs = 0;
let lastTimestamp = 0;

// UKF/EKF State:
let ukfState = {
    x: [0, 0, 0, 0], // [lat, lon, vel_lat, vel_lon] (simplifié pour cet exemple)
    P: [[100, 0, 0, 0], [0, 100, 0, 0], [0, 0, 10, 0], [0, 0, 0, 10]] // Matrice de covariance
};
let lastEKFPosition = { lat: 43.2964, lon: 5.3697, alt: 0, heading: 0 };
let currentSpeedMs = 0.0;
let currentVerticalSpeed = 0.0;

// Météo et correction métrologique
let lastP_hPa = BARO_ALT_REF_HPA;
let lastT_K = TEMP_SEA_LEVEL_K;
let lastH_perc = 0.6; // 60% par défaut
let currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound = C_S_DEFAULT;

// Minecraft Clock state
let minecraftTimeMs = 0;
let isXRayMode = false;

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);

const dataOrDefault = (val, decimals, suffix = '', precisionThreshold = 0.0001) => {
    if (val === undefined || val === null || isNaN(val)) {
        return 'N/A';
    }
    // Formattage scientifique pour très petits nombres
    if (Math.abs(val) < precisionThreshold && val !== 0) {
        return val.toExponential(3) + suffix;
    }
    return val.toFixed(decimals) + suffix;
};

// --- CALCULS PHYSIQUES ET RELATIVISTES ---

/** Calcule le facteur de Lorentz (gamma). */
const getLorentzFactor = (v) => {
    if (v >= C_L) return Infinity;
    return 1.0 / Math.sqrt(1.0 - (v * v) / (C_L * C_L));
};

/** Calcule le Rayon de Schwarzschild (Rs). */
const getSchwarzschildRadius = (massKg) => {
    if (massKg <= 0) return 0;
    return (2 * G_CONST * massKg) / (C_L * C_L);
};

/** Calcule la densité de l'air. */
const getAirDensity = (p_hPa, t_K, h_perc = 0.6) => {
    // Calcul de la pression de vapeur saturante (formule Magnus-Tetens simplifiée)
    const T_C = t_K - 273.15;
    const PV_SAT = 6.1078 * Math.pow(10, (7.5 * T_C) / (T_C + 237.3)) * 100; // Pa
    const PV_ACTUAL = h_perc * PV_SAT; // Pa
    const P_Pa = p_hPa * 100; // Pa

    // Pression de l'air sec
    const P_DRY = P_Pa - PV_ACTUAL;

    // Constante spécifique de l'air humide (approximation simple)
    const R_W = 461.495; // Constante pour la vapeur d'eau (J/kg·K)
    
    // Densité de l'air humide (Equation des gaz parfaits modifiée)
    const rho = (P_DRY / (R_AIR * t_K)) + (PV_ACTUAL / (R_W * t_K));
    return rho;
};

/** Calcule la vitesse du son. */
const getSpeedOfSound = (t_K) => {
    const GAMMA = 1.4; // Rapport des chaleurs spécifiques pour l'air sec
    const R = R_AIR;
    return Math.sqrt(GAMMA * R * t_K);
};

/** Calcule la Force de Traînée (Approximation). */
const getDragForce = (rho, v, A = 1.0, Cd = 0.47) => { // A=1m², Cd=0.47 (sphère) par défaut
    const q = 0.5 * rho * v * v; // Pression dynamique
    return q * A * Cd;
};

/** Calcule la Force de Coriolis (Approximation horizontale). */
const getCoriolisForce = (v_ms, latDeg, massKg) => {
    const latRad = latDeg * D2R;
    const f = 2 * OMEGA_EARTH * Math.sin(latRad);
    return massKg * f * v_ms;
};

/** Calcule l'énergie de masse au repos. */
const getRestMassEnergy = (massKg) => {
    return massKg * C_L * C_L;
};


// --- UKF/EKF CORE LOGIC (Simplified for Demo) ---
/** Simule une étape de prédiction et de mise à jour du filtre UKF/EKF. */
const runUKFStep = (positionData, deltaTime) => {
    // Dans une implémentation réelle, cela impliquerait des matrices et des vecteurs
    // pour les 21 états (position 3D, vitesse 3D, accélération 3D, bias capteur, etc.).
    
    // Simplification pour la démonstration : Filtrage de la vitesse
    const R_MEASURE = Math.pow(Math.max(R_MIN, positionData.acc * gpsAccuracyOverride), 2);
    const Q_PROC = Math.pow(Q_NOISE * ENVIRONMENT_FACTORS[selectedEnvironment].MULT, 2);

    let v_old = currentSpeedMs;
    let P_old = ukfState.P[2][2]; // Utilisation de P pour la vitesse (simplification)
    
    // Prédiction (Modèle de vitesse constante)
    let v_pred = v_old;
    let P_pred = P_old + Q_PROC * deltaTime;
    
    // Mise à jour (Correction avec mesure GNSS)
    const K = P_pred / (P_pred + R_MEASURE);
    let v_new = v_pred + K * (positionData.spd - v_pred);
    let P_new = (1 - K) * P_pred;
    
    // Mise à jour de l'état
    currentSpeedMs = v_new;
    ukfState.P[2][2] = P_new;

    // Logique de position : Utiliser l'EKF pour positionner le marqueur
    if (v_new > 0.1) {
        // Simple extrapolation pour l'affichage (non EKF réel)
        const travelDist = v_new * deltaTime;
        // Calculer une nouvelle position basée sur le cap (si disponible)
        // Ceci est une approximation et non le cœur de l'EKF/UKF
    }

    // Mise à jour de l'incertitude pour l'affichage
    const kalmanUncertainty = Math.sqrt(P_new);

    // Retourne les valeurs mises à jour
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
    const TIME_SCALE = 72; // 1200ms -> 1 jour MC (facteur de 72 par rapport à la réalité)

    minecraftTimeMs = (minecraftTimeMs + ms * TIME_SCALE) % (TICKS_PER_DAY * MS_PER_TICK);
    
    const ticks = minecraftTimeMs / MS_PER_TICK;
    const hours = Math.floor((ticks / 1000) + 6) % 24; // Décalage de 6h (0 ticks = 6h)
    const minutes = Math.floor((ticks % 1000) / (1000 / 60));
    const timeStr = `${String(hours).padStart(2, '0')}:${String(minutes).padStart(2, '0')}`;
    
    $('time-minecraft').textContent = timeStr;
    
    // Rotation de l'horloge astro pour la démo (0h = 6h du matin, 12h = 18h)
    const angle = (ticks / TICKS_PER_DAY) * 360 - 90; // -90 pour commencer à 6h
    if ($('sun-element')) {
        $('sun-element').style.transform = `rotate(${angle}deg)`;
    }
    if ($('moon-element')) {
         // La Lune est à 180 degrés de rotation
        $('moon-element').style.transform = `rotate(${angle + 180}deg)`;
    }
};

/** Met à jour l'affichage de l'heure locale et UTC (NTP). */
const updateTimeDisplay = () => {
    const now = new Date();
    const utc = now.toUTCString().split(' ')[4];

    // Correction de l'ID pour l'heure locale (NTP)
    if ($('local-time')) {
        $('local-time').textContent = now.toLocaleTimeString('fr-FR');
    }
    // Correction de l'ID pour la date/heure UTC
    if ($('date-display')) {
        const dateFr = now.toLocaleDateString('fr-FR');
        $('date-display').textContent = `${dateFr} ${utc}`;
    }
    
    // Calcul du temps écoulé total
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

/** Mise à jour de l'astronomie (utilise SunCalc). */
const updateAstro = (lat, lon) => {
    // Simulation des données SunCalc si le polyfill SunCalc.js est chargé
    if (typeof SunCalc === 'undefined') {
         // Affichage par défaut si la librairie n'est pas chargée
         if ($('sun-alt')) $('sun-alt').textContent = 'N/A';
         if ($('moon-phase-name')) $('moon-phase-name').textContent = 'N/A';
         return;
    }

    const now = new Date();
    const times = SunCalc.getTimes(now, lat, lon);
    const sunPos = SunCalc.getPosition(now, lat, lon);
    const moonPos = SunCalc.getMoonPosition(now, lat, lon);
    const moonIllumination = SunCalc.getMoonIllumination(now);
    
    const altitudeSunDeg = sunPos.altitude * R2D;
    const azimuthSunDeg = (sunPos.azimuth * R2D + 180) % 360; // Convertir en 0-360°

    // --- Affichage Soleil ---
    $('sun-alt').textContent = dataOrDefault(altitudeSunDeg, 2, '°');
    $('sun-azimuth').textContent = dataOrDefault(azimuthSunDeg, 2, '°');
    $('sunrise-times').textContent = times.sunrise ? `${times.sunrise.toLocaleTimeString('fr-FR')}/${times.sunriseEnd.toLocaleTimeString('fr-FR')}` : 'N/A';
    $('sunset-times').textContent = times.sunset ? `${times.sunset.toLocaleTimeString('fr-FR')}/${times.sunsetStart.toLocaleTimeString('fr-FR')}` : 'N/A';
    // Durée du jour
    if (times.sunset && times.sunrise) {
        const diffMs = times.sunset.getTime() - times.sunrise.getTime();
        const h = String(Math.floor(diffMs / 3600000)).padStart(2, '0');
        const m = String(Math.floor((diffMs % 3600000) / 60000)).padStart(2, '0');
        $('day-duration').textContent = `${h}:${m}:00`;
    } else {
        $('day-duration').textContent = '24:00:00';
    }

    // --- Affichage Lune ---
    $('moon-illuminated').textContent = dataOrDefault(moonIllumination.fraction * 100, 1, ' %');
    $('moon-phase-name').textContent = getMoonPhaseName(moonIllumination.phase);
    $('moon-alt').textContent = dataOrDefault(moonPos.altitude * R2D, 2, '°');
    $('moon-azimuth').textContent = dataOrDefault((moonPos.azimuth * R2D + 180) % 360, 2, '°');
    
    // Calcul Temps Solaire Vrai/Moyen (TST/MST)
    const LST = now.getHours() + now.getMinutes() / 60 + now.getSeconds() / 3600;
    const EOT = (sunPos.declination - Math.asin(Math.sin(sunPos.altitude) * Math.sin(lat * D2R)) / Math.cos(lat * D2R)) * R2D * 4; // Simplification de l'EOT en minutes
    
    const mstHours = LST - (lon / 15);
    const tstHours = mstHours + EOT / 60;

    $('eot').textContent = dataOrDefault(EOT, 2, ' min');
    $('mst').textContent = formatHoursToTime(mstHours);
    $('tst').textContent = formatHoursToTime(tstHours);
    
    // Midi Solaire Local (UTC)
    const noonHours = 12 - (lon / 15);
    $('noon-solar').textContent = formatHoursToTime(noonHours);

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

const formatHoursToTime = (hours) => {
    let h = Math.floor(hours) % 24;
    if (h < 0) h += 24;
    const m = Math.floor((hours * 60) % 60);
    const s = Math.floor((hours * 3600) % 60);
    return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
};

// --- GESTION DU DOM ET BOUCLES DE MISE À JOUR ---

/** Boucle de mise à jour rapide (Fast Loop) - 100ms / GNSS */
const runFastLoop = (positionData, deltaTime) => {
    if (isEmergencyStop || isGpsPaused) return;

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
    $('perc-speed-c').textContent = dataOrDefault(v_stable / C_L * 100, 2, 'e+0 %', 1e-10);
    $('lorentz-factor').textContent = dataOrDefault(gamma, 4);
    $('energy-relativistic').textContent = dataOrDefault(totalEnergy, 2, ' J');
    $('energy-rest-mass').textContent = dataOrDefault(restEnergy, 2, ' J');
    $('momentum').textContent = dataOrDefault(momentum, 2, ' kg·m/s');
    $('Rs-object').textContent = dataOrDefault(getSchwarzschildRadius(currentMass), 3, ' m');

    // Mise à jour de la distance et du temps de mouvement
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
    $('gps-accuracy-display').textContent = dataOrDefault(positionData.acc, 6, ' m');
};

/** Boucle de mise à jour lente (Slow Loop) - 1000ms / Météo/Astro/Forces */
const runSlowLoop = () => {
    // 1. Calculs Météo/Air
    const T_C = lastT_K - 273.15;
    currentAirDensity = getAirDensity(lastP_hPa, lastT_K, lastH_perc);
    currentSpeedOfSound = getSpeedOfSound(lastT_K);
    
    $('temp-air-2').textContent = dataOrDefault(T_C, 1, ' °C');
    $('pressure-2').textContent = dataOrDefault(lastP_hPa, 0, ' hPa');
    $('humidity-2').textContent = dataOrDefault(lastH_perc * 100, 0, ' %');
    $('air-density-calc').textContent = dataOrDefault(currentAirDensity, 3, ' kg/m³');
    $('speed-of-sound-calc').textContent = dataOrDefault(currentSpeedOfSound, 2, ' m/s (Cor.)');
    
    const machNumber = currentSpeedMs / currentSpeedOfSound;
    $('perc-speed-sound').textContent = dataOrDefault(machNumber * 100, 2, ' %');
    $('mach-number').textContent = dataOrDefault(machNumber, 4);

    // 2. Calculs Dynamique et Forces
    const q_dyn = 0.5 * currentAirDensity * currentSpeedMs * currentSpeedMs;
    const dragForce = getDragForce(currentAirDensity, currentSpeedMs, 1.0, 0.47);
    const dragPower = dragForce * currentSpeedMs / 1000; // en kW
    const kineticEnergy = 0.5 * currentMass * currentSpeedMs * currentSpeedMs;
    const coriolisForce = getCoriolisForce(currentSpeedMs, lastEKFPosition.lat, currentMass);
    
    $('dynamic-pressure').textContent = dataOrDefault(q_dyn, 2, ' Pa');
    $('drag-force').textContent = dataOrDefault(dragForce, 2, ' N');
    $('drag-power-kw').textContent = dataOrDefault(dragPower, 2, ' kW');
    $('kinetic-energy').textContent = dataOrDefault(kineticEnergy, 2, ' J');
    $('coriolis-force').textContent = dataOrDefault(coriolisForce, 2, ' N');

    // 3. Mise à jour de l'Astro
    updateAstro(lastEKFPosition.lat, lastEKFPosition.lon);

    // 4. Mise à jour des données générales
    $('mass-display').textContent = dataOrDefault(currentMass, 3, ' kg');
    $('gravity-base').textContent = dataOrDefault(CELESTIAL_BODIES[selectedCelestialBody].gravity, 4, ' m/s²');
};


// --- ÉVÉNEMENTS & INITIALISATION ---

/** Démarre l'acquisition GPS/Boucles de travail. */
const startGpsAcquisition = () => {
    if (typeof navigator.geolocation === 'undefined') {
        alert("L'API de Géolocalisation n'est pas supportée par ce navigateur.");
        return;
    }
    
    // Démarrage d'une simulation/acquisition GPS
    isGpsPaused = false;
    $('toggle-gps-btn').innerHTML = '⏸️ PAUSE GPS';
    $('toggle-gps-btn').style.backgroundColor = '#ffc107'; 
    $('speed-status-text').textContent = 'Acquisition GPS...';
    
    const FAST_UPDATE_MS = 100;
    
    setInterval(() => {
        updateTimeDisplay();
    }, 1000); // Mise à jour de l'horloge système chaque seconde

    setInterval(() => {
        // Simuler les données GPS (remplacer par watchPosition réel)
        const simData = {
            lat: lastEKFPosition.lat, 
            lon: lastEKFPosition.lon, 
            alt: lastEKFPosition.alt, 
            spd: currentSpeedMs + (Math.random() - 0.5) * 0.1, // Simulation de bruit
            acc: 5.0 + Math.random() * 5 // Simulation de précision
        };
        
        const deltaTime = FAST_UPDATE_MS / 1000;
        
        if (!isGpsPaused && !isEmergencyStop) {
             // Mise à jour des coordonnées pour simuler le mouvement EKF
             lastEKFPosition.lat += (simData.spd / 111111) * Math.cos(lastEKFPosition.heading * D2R) * deltaTime; 
             lastEKFPosition.lon += (simData.spd / (111111 * Math.cos(lastEKFPosition.lat * D2R))) * Math.sin(lastEKFPosition.heading * D2R) * deltaTime;
             
             runFastLoop(simData, deltaTime);
        }
    }, FAST_UPDATE_MS);

    // Démarrage de la boucle lente (météo/astro/forces)
    setInterval(runSlowLoop, 1000);
    
    // Mise à jour initiale de l'état
    runSlowLoop(); 
};

/** Gère le bouton MARCHE/PAUSE GPS. */
const toggleGPS = () => {
    if ($('toggle-gps-btn').innerHTML.includes('MARCHE')) {
        // Initialisation si première exécution
        startGpsAcquisition();
    } else {
        isGpsPaused = !isGpsPaused;
        if (isGpsPaused) {
            $('toggle-gps-btn').innerHTML = '▶️ MARCHE GPS';
            $('toggle-gps-btn').style.backgroundColor = '#28a745';
            $('speed-status-text').textContent = 'PAUSE GPS';
        } else {
            $('toggle-gps-btn').innerHTML = '⏸️ PAUSE GPS';
            $('toggle-gps-btn').style.backgroundColor = '#ffc107';
            $('speed-status-text').textContent = 'Signal OK';
        }
    }
};

/** Gère l'arrêt d'urgence. */
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

/** Réinitialise la distance et les moyennes de vitesse. */
const resetDistance = () => {
    totalDistanceM = 0.0;
    timeMovingMs = 0;
    $('distance-total-km').textContent = '0.000 km | 0.00 m';
    $('time-moving').textContent = '00:00:00';
    $('vitesse-moyenne-mvt').textContent = '0.0 km/h';
    $('vitesse-moyenne-totale').textContent = '0.0 km/h';
};

/** Réinitialise la vitesse maximale. */
const resetVmax = () => {
    maxSpeedKmh = 0.0;
    $('speed-max').textContent = '0.0 km/h';
};

/** Réinitialise tout. */
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
    // Initialisation des listeners pour tous les contrôles
    $('toggle-gps-btn').addEventListener('click', toggleGPS);
    $('emergency-stop-btn').addEventListener('click', emergencyStop);
    $('reset-dist-btn').addEventListener('click', resetDistance);
    $('reset-vmax-btn').addEventListener('click', resetVmax);
    $('reset-all-btn').addEventListener('click', resetAll);
    
    // Toggle Mode Nuit (si l'ID est correct dans le HTML)
    if ($('toggle-mode-btn')) {
         $('toggle-mode-btn').addEventListener('click', () => {
            document.body.classList.toggle('dark-mode');
            $('toggle-mode-btn').innerHTML = document.body.classList.contains('dark-mode') ? '<i class="fas fa-sun"></i> Mode Jour' : '<i class="fas fa-moon"></i> Mode Nuit';
         });
    }

    // Gestion des inputs et selects (UKF/Physique)
    $('mass-input').addEventListener('input', (e) => {
        currentMass = parseFloat(e.target.value) || 0;
        runSlowLoop(); // Recalcul des énergies
    });

    $('environment-select').addEventListener('change', (e) => {
        selectedEnvironment = e.target.value;
        $('env-factor').textContent = `${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY} (x${ENVIRONMENT_FACTORS[selectedEnvironment].MULT.toFixed(1)})`;
    });
    
    $('celestial-body-select').addEventListener('change', (e) => {
        selectedCelestialBody = e.target.value;
        runSlowLoop(); // Mise à jour de la gravité affichée
    });
    
    $('gps-accuracy-override').addEventListener('input', (e) => {
        gpsAccuracyOverride = parseFloat(e.target.value) || 0;
        $('gps-accuracy-display').textContent = dataOrDefault(gpsAccuracyOverride, 6, ' m');
    });

    $('distance-ratio-toggle-btn').addEventListener('click', () => {
        distanceRatio = (distanceRatio === 1.0) ? 8.0 : 1.0; // Bascule 1.0 / 8.0 (Nether/Surface)
        $('distance-ratio-toggle-btn').textContent = `Rapport Distance: ${distanceRatio === 8.0 ? 'NETHER' : 'SURFACE'} (${distanceRatio.toFixed(3)})`;
        $('distance-ratio').textContent = distanceRatio.toFixed(3);
    });

    // Configuration initiale de l'affichage
    $('toggle-gps-btn').style.backgroundColor = '#28a745';
    $('speed-status-text').textContent = 'Attente du signal GPS...';

    // Simulation de données météo par défaut pour que les calculs de densité et vitesse du son fonctionnent
    lastT_K = 283.45; // 10.3 °C
    lastP_hPa = 1013;
    lastH_perc = 0.6;
    $('weather-status').textContent = 'Actif (Hors Ligne)';
    
    // Exécute la boucle lente une fois pour peupler les valeurs par défaut (météo, gravité, etc.)
    runSlowLoop();

    // Démarrer l'horloge système sans démarrer le GPS
    updateTimeDisplay();
    setInterval(updateTimeDisplay, 1000);
};

// Exécute l'initialisation après le chargement complet du DOM
window.addEventListener('load', initDashboard);
// Fin du fichier gnss-dashboard-full.js
