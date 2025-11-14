// =================================================================
// BLOC 1/4 : Constantes, Variables d'État et Fonctions Utilitaire de Base
// Rôle : Initialisation du système et des paramètres EKF/IMU.
// =================================================================

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const R_AIR = 287.058;      // Constante spécifique de l'air sec (J/kg·K)
const G_BASE = 9.80665;     // Gravité standard au niveau de la mer (m/s²)
const KMH_MS = 3.6;         // Conversion m/s vers km/h
const KELVIN_OFFSET = 273.15; 
const DOM_FAST_UPDATE_MS = 50; // Haute fréquence (IMU/EKF)
const DOM_SLOW_UPDATE_MS = 5000; // Basse fréquence (Météo/Astro)

// --- PARAMÈTRES DU FILTRE DE KALMAN (VITESSE & ALTITUDE) ---
const Q_IMU = 0.05;         // Bruit de processus de l'IMU (FAIBLE: Forte confiance en la prédiction IMU)
const Q_ALT = 0.5;          // Bruit de processus (Altitude)
const R_MIN = 0.01;         // Bruit de mesure minimum (Vitesse GPS)
const R_MAX = 500.0;        // Bruit de mesure maximum (Vitesse GPS)
const MAX_ACC = 200;        // Précision max (m)
const MIN_SPD = 0.05;       // Vitesse minimale "en mouvement" (m/s)

let P_spd = 100;            // Incertitude initiale de la vitesse
let P_alt = 100;            // Incertitude initiale de l'altitude
let speedEst = 0;           // Vitesse estimée par l'EKF (m/s)
let lastSpeed = 0;

// --- VARIABLES D'ÉTAT IMU ---
let imuAccelX = 0, imuAccelY = 0, imuAccelZ = 9.81; // Accélérations (m/s²)
let imuGyroZ = 0;       // Rotation (rad/s)
let currentHeading = 0; // Cap estimé/fusionné (0-360)

// --- VARIABLES D'ÉTAT GNSS/EKF & Contrôles ---
let lat = null, lon = null, altEst = 0; 
let accuracy = 1000;        
let wID = null;             
let currentGPSMode = 'HIGH_FREQ'; 
let gpsAccuracyOverride = 0;
let emergencyStopActive = false;
let selectedEnvironment = 'NORMAL'; 
let currentCelestialBody = 'EARTH'; 
let currentMass = 70;       
let isXRayMode = false;     

let distM = 0;              
let timeMoving = 0;         
let maxSpd = 0;             

// Variables Météo/Physique
let airDensity = 1.225;     
let localGravity = G_BASE;  
let speedOfSound = 343;     
let tempK = 288.15;         
let lastP_hPa = 1013.25;    

// --- FONCTIONS UTILITAIRES DE BASE ---
const $ = (id) => document.getElementById(id);
let lServH, lLocH; // Placeholders pour la synchronisation NTP simulée

/** Synchronise l'heure locale avec une heure serveur simulée (NTP simulé). */
function getCDate(lServH, lLocH) {
    if (lServH && lLocH) {
        const offset = lServH.getTime() - lLocH.getTime();
        return new Date(new Date().getTime() + offset);
    }
    return new Date(); 
}

/** Calcule le bruit de mesure (R) pour l'EKF basé sur la précision GPS et l'environnement. */
function calculateR(accuracy_m) {
    if (accuracy_m > MAX_ACC) return R_MAX;
    if (accuracy_m <= 0) accuracy_m = 0.001; 
    
    let R = Math.max(R_MIN, Math.min(R_MAX, accuracy_m * accuracy_m / 10));

    let R_FACTOR_RATIO = 1.0;
    switch (selectedEnvironment) {
        case 'FOREST': R_FACTOR_RATIO = 2.5; break;
        case 'CONCRETE': R_FACTOR_RATIO = 7.0; break;
        case 'METAL': R_FACTOR_RATIO = 5.0; break;
        default: R_FACTOR_RATIO = 1.0; break;
    }
    
    return R * R_FACTOR_RATIO;
}

// --- FONCTIONS DE PHYSIQUE ET MÉTÉO (Stubs pour la clarté du bloc) ---

function calculateAirDensity(tempK, pressure_hPa) {
    const P_Pa = pressure_hPa * 100;
    return P_Pa / (R_AIR * tempK); 
}
function calculateSpeedOfSound(tempK) {
    const GAMMA = 1.4; 
    return Math.sqrt(GAMMA * R_AIR * tempK);
}
function calculateLorentzFactor(speed_ms) {
    const ratio = speed_ms / C_L;
    return 1 / Math.sqrt(1 - (ratio * ratio));
}
function calculateLocalGravity(latitude_rad, altitude_m) {
    const sin2 = Math.sin(latitude_rad) * Math.sin(latitude_rad);
    const g0 = 9.780327 * (1 + 0.0053024 * sin2 - 0.0000058 * sin2 * sin2);
    return g0 - (3.086e-6 * altitude_m); 
}
function getMoonPhaseName(phase_0_1) {
    if (phase_0_1 < 0.03 || phase_0_1 >= 0.97) return "Nouvelle Lune";
    if (phase_0_1 < 0.22) return "Premier Croissant";
    if (phase_0_1 < 0.28) return "Premier Quartier";
    if (phase_0_1 < 0.47) return "Lune Gibbeuse Croissante";
    if (phase_0_1 < 0.53) return "Pleine Lune";
    if (phase_0_1 < 0.72) return "Lune Gibbeuse Décroissante";
    if (phase_0_1 < 0.78) return "Dernier Quartier";
    return "Dernier Croissant";
}
function fetchWeatherAndAirDensity() {
    return new Promise(resolve => { /* Simulation d'appel météo */ });
}


// =================================================================
// BLOC 2/4 : Gestion des Capteurs (IMU, GNSS) et Filtre EKF (Fusion)
// Rôle : Mise à jour de l'état (vitesse/position) par la fusion IMU/GNSS.
// =================================================================

/** * Gestionnaire de données IMU (simulation de la lecture des capteurs).
 * Appelée à haute fréquence pour le Dead Reckoning.
 */
function imuDataHandler(dt) {
    // --- SIMULATION DES DONNÉES IMU (m/s² et rad/s) ---
    const movementNoise = speedEst * 0.02 + 0.1; 
    imuAccelX = (Math.random() - 0.5) * movementNoise; 
    imuAccelY = (Math.random() - 0.5) * movementNoise; 
    imuAccelZ = localGravity + (Math.random() - 0.5) * 0.05; 

    imuGyroZ += (Math.random() - 0.5) * 0.01 * dt; 
    currentHeading = (currentHeading + imuGyroZ * R2D * dt + 360) % 360;

    // Mise à jour de l'affichage IMU
    if ($('accel-x')) $('accel-x').textContent = imuAccelX.toFixed(3) + ' m/s²';
    if ($('accel-y')) $('accel-y').textContent = imuAccelY.toFixed(3) + ' m/s²';
    if ($('accel-z')) $('accel-z').textContent = imuAccelZ.toFixed(3) + ' m/s²';
    if ($('heading-display')) $('heading-display').textContent = `${currentHeading.toFixed(1)}°`;
    if ($('imu-status')) $('imu-status').textContent = 'Actif / Fusion';
}

/** * Implémentation du Filtre de Kalman Étendu pour la fusion vitesse (IMU + GNSS).
 * L'IMU est le moteur de PRÉDICTION, le GNSS est le correcteur de DÉRIVE.
 */
function updateEKF(coords, speedMeasure, accuracy_m, dt) {
    const altMeasure = coords.altitude || altEst;

    // --- 1. PRÉDICTION (Poids IMU renforcé) ---
    const imuHorizontalAccel = Math.sqrt(imuAccelX * imuAccelX + imuAccelY * imuAccelY); 

    const speedPred = speedEst + imuHorizontalAccel * dt; 
    P_spd = P_spd + Q_IMU * dt; // P augmente seulement avec le faible bruit IMU
    
    // --- 2. CORRECTION (Correction de dérive GNSS) ---
    const R_spd = calculateR(accuracy_m);
    let K_spd = P_spd / (P_spd + R_spd);
    
    // Le GNSS (Correction) corrige la dérive accumulée par l'IMU (Prédiction)
    speedEst = speedPred + K_spd * (speedMeasure - speedPred); 
    P_spd = (1 - K_spd) * P_spd;
    
    // EKF Altitude (correction simple)
    const altPred = altEst;
    P_alt = P_alt + Q_ALT * dt;
    const R_alt = accuracy_m * accuracy_m; 
    let K_alt = P_alt / (P_alt + R_alt);
    altEst = altPred + K_alt * (altMeasure - altPred);
    P_alt = (1 - K_alt) * P_alt;

    // --- MISE À JOUR DES COMPTEURS ET CALCULS ---
    if (speedEst >= MIN_SPD) {
        distM += speedEst * dt;
        timeMoving += dt;
        maxSpd = Math.max(maxSpd, speedEst * KMH_MS);
    }
    lastSpeed = speedEst;
    
    // Mise à jour des variables d'état
    lat = coords.latitude;
    lon = coords.longitude;
    accuracy = accuracy_m;

    // Calculs Physiques Avancés
    localGravity = calculateLocalGravity(lat * D2R, altEst);
    speedOfSound = calculateSpeedOfSound(tempK);
    const machNumber = speedEst / speedOfSound;
    const lorentzFactor = calculateLorentzFactor(speedEst);
    
    // Mise à jour de tous les affichages
    updateGPSDisplay(coords, speedMeasure, accuracy_m, dt);
    updateEKFDisplay(speedMeasure, R_spd, K_spd, P_spd, altEst, P_alt, localGravity, machNumber, lorentzFactor);
}

/** Gestionnaire de succès GPS. */
function gpsSuccess(position) {
    const dt = DOM_FAST_UPDATE_MS / 1000;
    const speedRaw = position.coords.speed || 0;
    const newAccuracy = gpsAccuracyOverride || position.coords.accuracy;

    updateEKF(position.coords, speedRaw, newAccuracy, dt); 
}

/** Gestionnaire d'erreur GPS. */
function gpsError(error) {
    console.error(`Erreur GPS (${error.code}): ${error.message}`);
    if ($('gps-status-dr')) $('gps-status-dr').textContent = `ERREUR (${error.code}) - DR: IMU Seul`;
}

/** Démarre ou change le mode de surveillance GPS. */
function setGPSMode(mode) {
    stopGPS(); 
    currentGPSMode = mode;
    
    const options = {
        enableHighAccuracy: mode === 'HIGH_FREQ',
        timeout: 5000,
        maximumAge: mode === 'HIGH_FREQ' ? 0 : 3000,
    };
    
    wID = navigator.geolocation.watchPosition(gpsSuccess, gpsError, options);
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = '⏸️ PAUSE GPS';
    if ($('gps-status-dr')) $('gps-status-dr').textContent = 'ACTIF (FUSION IMU/GPS)';
}

/** Arrête la surveillance GPS. */
function stopGPS() {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
    }
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = '▶️ MARCHE GPS';
    if ($('gps-status-dr')) $('gps-status-dr').textContent = 'ARRÊTÉ';
                           }
// =================================================================
// BLOC 3/4 : Fonctions d'Affichage, Carte et Calculs Météo/Astro
// Rôle : Rendu de l'information et fonctions de calculs auxiliaires.
// =================================================================

let map = null, marker = null, track = [];

/** Initialise la carte Leaflet. */
function initMap() {
    if ($('map') && !map && typeof L !== 'undefined') {
        map = L.map('map').setView([43.296, 5.370], 13);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { attribution: '© OpenStreetMap contributors' }).addTo(map);
        marker = L.marker([0, 0]).addTo(map);
    }
}

/** Met à jour l'affichage des données GPS et des compteurs. */
function updateGPSDisplay(coords, speedRaw, accuracy_m) {
    if ($('speed-stable')) $('speed-stable').textContent = `${(speedEst * KMH_MS).toFixed(1)} km/h`;
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${speedEst.toFixed(2)} m/s`;
    if ($('speed-raw-ms')) $('speed-raw-ms').textContent = `${speedRaw.toFixed(2)} m/s`;
    
    if (lat !== null) {
        if ($('lat-display')) $('lat-display').textContent = lat.toFixed(6) + '°';
        if ($('lon-display')) $('lon-display').textContent = lon.toFixed(6) + '°';
        if ($('alt-display')) $('alt-display').textContent = altEst.toFixed(2) + ' m';
    }
    
    if ($('distance-total-km')) $('distance-total-km').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(1)} m`;
    if ($('speed-max')) $('speed-max').textContent = `${maxSpd.toFixed(1)} km/h`;

    if (map && marker && lat !== null) {
        const latLng = [coords.latitude, coords.longitude];
        marker.setLatLng(latLng);
        map.setView(latLng, map.getZoom() > 14 ? map.getZoom() : 14);
        track.push(latLng);
    }
    if ($('gps-precision')) $('gps-precision').textContent = `${accuracy_m.toFixed(2)} m`;
}

/** Met à jour l'affichage des données EKF et physiques avancées. */
function updateEKFDisplay(speedMeasure, R_spd, K_spd, P_spd, altEst, P_alt, localGravity, machNumber, lorentzFactor) {
    if ($('kalman-uncert')) $('kalman-uncert').textContent = `${P_spd.toFixed(4)}`;
    if ($('alt-uncertainty')) $('alt-uncertainty').textContent = `${Math.sqrt(P_alt).toFixed(2)} m`;
    if ($('speed-error-perc')) $('speed-error-perc').textContent = `${R_spd.toFixed(4)}`;
    
    if ($('gravity-local')) $('gravity-local').textContent = `${localGravity.toFixed(4)} m/s²`;
    if ($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = `${speedOfSound.toFixed(2)} m/s`;
    if ($('mach-number')) $('mach-number').textContent = `${machNumber.toFixed(4)}`;
    if ($('perc-speed-sound')) $('perc-speed-sound').textContent = `${(machNumber * 100).toFixed(2)} %`;
    if ($('lorentz-factor')) $('lorentz-factor').textContent = `${lorentzFactor.toFixed(4)}`;
    if ($('perc-speed-c')) $('perc-speed-c').textContent = `${(speedEst / C_L * 100).toExponential(2)} %`;
}

/** Met à jour la visualisation de l'horloge astronomique. */
function updateClockVisualization(now, sunPos, moonPos, sunTimes) {
    const clock = $('minecraft-clock');
    if (clock) {
        clock.className = ''; 
        // Logique de changement de classe (day/night/etc.)
    }
    const sunEl = $('sun-element');
    const moonEl = $('moon-element');
    // Mouvements Soleil/Lune
    if (sunEl) sunEl.style.transform = `rotate(${sunPos.altitude * R2D + 90}deg)`;
    if (moonEl) moonEl.style.transform = `rotate(${moonPos.altitude * R2D + 90}deg)`;
}

/** Calcule et affiche les données astronomiques. */
function updateAstro(latA, lonA, lServH, lLocH) {
    const now = getCDate(lServH, lLocH);
    
    if (typeof SunCalc === 'undefined' || latA === null || lonA === null) return;
    
    const sunPos = SunCalc.getPosition(now, latA, lonA);
    const moonPos = SunCalc.getMoonPosition(now, latA, lonA);
    const moonIllum = SunCalc.getMoonIllumination(now);
    const sunTimes = SunCalc.getTimes(now, latA, lonA);

    // Les fonctions getSolarTime et autres dépendent de librairies non incluses (stubs)
    // if ($('tst')) $('tst').textContent = getSolarTime(now, lonA).TST;
    
    if ($('sun-alt')) $('sun-alt').textContent = `${(sunPos.altitude * R2D).toFixed(2)}°`;
    if ($('sun-azimuth')) $('sun-azimuth').textContent = `${((sunPos.azimuth * R2D + 180) % 360).toFixed(2)}°`;
    if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(moonIllum.phase);
    if ($('moon-illuminated')) $('moon-illuminated').textContent = `${(moonIllum.fraction * 100).toFixed(1)}%`;
    
    updateClockVisualization(now, sunPos, moonPos, sunTimes);
}

/** Simule l'acquisition de données météo et met à jour les variables globales. */
function fetchWeatherAndAirDensity() {
    return new Promise(resolve => {
        const pressure_hPa = lastP_hPa + (Math.random() - 0.5) * 0.5;
        const tempC = tempK - KELVIN_OFFSET + (Math.random() - 0.5) * 0.2;
        const tempK_new = tempC + KELVIN_OFFSET;
        const humidity_perc = 60 + (Math.random() - 0.5) * 5;
        
        const air_density = calculateAirDensity(tempK_new, pressure_hPa);
        const dew_point = tempC - ((100 - humidity_perc) / 5); 

        resolve({
            tempC: tempC,
            tempK: tempK_new,
            pressure_hPa: pressure_hPa,
            air_density: air_density,
            dew_point: dew_point
        });
    });
}
// =================================================================
// BLOC 4/4 : Boucles d'Intervalle et Initialisation du Système
// Rôle : Démarrage du tableau de bord et gestion des interactions utilisateur.
// =================================================================

/**
 * Boucle de mise à jour rapide (50ms).
 * Gère le Dead Reckoning IMU et l'affichage du temps.
 */
function domUpdateLoop() {
    const dt = DOM_FAST_UPDATE_MS / 1000;
    
    // 1. Mise à jour IMU (Dead Reckoning) - haute fréquence
    imuDataHandler(dt);
    
    // 2. Mise à jour de l'affichage du temps
    const now = getCDate(lServH, lLocH);
    if (now) {
        if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString('fr-FR', { hour12: false });
        if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
    }
    
    if ($('time-moving')) $('time-moving').textContent = timeMoving.toFixed(2) + ' s';
}

/**
 * Initialise l'application et configure les boucles de mesure et d'écoute.
 */
function init() {
    initMap();
    setGPSMode(currentGPSMode);
    
    // Boucle rapide (IMU + DOM)
    setInterval(domUpdateLoop, DOM_FAST_UPDATE_MS); 

    // Boucle lente (Météo + Astro)
    setInterval(() => {
        if (lat !== null && lon !== null) {
            fetchWeatherAndAirDensity().then(data => {
                tempK = data.tempK;
                lastP_hPa = data.pressure_hPa;
                airDensity = data.air_density;

                // Mise à jour du DOM Météo
                if ($('temp-air-2')) $('temp-air-2').textContent = `${data.tempC.toFixed(1)} °C`;
                if ($('pressure-2')) $('pressure-2').textContent = `${data.pressure_hPa.toFixed(0)} hPa`;
                if ($('air-density')) $('air-density').textContent = `${data.air_density.toFixed(3)} kg/m³`;
                if ($('dew-point')) $('dew-point').textContent = `${data.dew_point.toFixed(1)} °C`;
            });

            updateAstro(lat, lon, lServH, lLocH);
        }
    }, DOM_SLOW_UPDATE_MS);

    // --- CONFIGURATION DES LISTENERS D'ÉVÉNEMENTS (Contrôle du système) ---
    $('toggle-gps-btn').addEventListener('click', () => { if (wID !== null) stopGPS(); else setGPSMode(currentGPSMode); });
    $('freq-select').addEventListener('change', (e) => setGPSMode(e.target.value));
    $('emergency-stop-btn').addEventListener('click', () => {
        emergencyStopActive = !emergencyStopActive;
        $('emergency-stop-btn').textContent = emergencyStopActive ? '🚨 Arrêt d\'urgence: ACTIF 🔴' : '🛑 Arrêt d\'urgence: INACTIF 🟢';
        $('emergency-stop-btn').classList.toggle('active', emergencyStopActive);
    });
    $('toggle-mode-btn').addEventListener('click', () => {
        document.body.classList.toggle('dark-mode');
        const mapEl = document.getElementById('map');
        if (mapEl) {
            const isDarkMode = document.body.classList.contains('dark-mode');
            mapEl.style.filter = isDarkMode ? 'invert(0.9) hue-rotate(180deg)' : 'none';
        }
    });

    $('reset-dist-btn').addEventListener('click', () => { distM = 0; timeMoving = 0; maxSpd = 0; });
    $('reset-max-btn').addEventListener('click', () => { maxSpd = 0; });
    $('reset-all-btn').addEventListener('click', () => { window.location.reload(); });
    
    $('gps-accuracy-override').addEventListener('input', (e) => { 
        gpsAccuracyOverride = parseFloat(e.target.value); 
        if ($('gps-accuracy-display')) $('gps-accuracy-display').textContent = `${gpsAccuracyOverride.toFixed(6)} m`;
    });
    $('mass-input').addEventListener('input', (e) => { currentMass = parseFloat(e.target.value); $('mass-display').textContent = currentMass.toFixed(3) + ' kg'; });
    $('celestial-body-select').addEventListener('change', (e) => { currentCelestialBody = e.target.value; });
    $('environment-select').addEventListener('change', (e) => { 
        selectedEnvironment = e.target.value; 
        if ($('env-factor')) $('env-factor').textContent = e.target.options[e.target.selectedIndex].text;
    });
    
    $('xray-button').addEventListener('click', () => { 
        isXRayMode = !isXRayMode;
        document.getElementById('minecraft-clock').classList.toggle('x-ray', isXRayMode);
        $('xray-button').textContent = isXRayMode ? 'ON' : 'X';
    });
}

document.addEventListener('DOMContentLoaded', init);
