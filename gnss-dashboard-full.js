// =================================================================
// BLOC 1/4 : Constantes, Variables d'État et Fonctions Utilitaires
// Contient les variables d'état (lat, speed, etc.) et les constantes critiques.
// =================================================================

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const R_E_BASE = 6371000;   // Rayon terrestre moyen (m)
const KMH_MS = 3.6;         // Conversion m/s vers km/h
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const R_AIR = 287.058;      // Constante spécifique de l'air sec (J/kg·K)
const G_BASE = 9.80665;     // Gravité standard au niveau de la mer (m/s²)
const KELVIN_OFFSET = 273.15; // 0°C en Kelvin
const DOM_FAST_UPDATE_MS = 50; // Intervalle de rafraîchissement DOM rapide (ms)
const DOM_SLOW_UPDATE_MS = 5000; // Intervalle de rafraîchissement DOM lent (ms) (Météo/Astro)

// --- PARAMÈTRES DU FILTRE DE KALMAN (Vitesse & Altitude) ---
const Q_NOISE = 0.1;        // Bruit de processus (Vitesse)
const Q_ALT = 0.5;          // Bruit de processus (Altitude)
const R_MIN = 0.01;         // Bruit de mesure minimum (Vitesse)
const R_MAX = 500.0;        // Bruit de mesure maximum (Vitesse)
const MAX_ACC = 200;        // Précision max (m) avant "Estimation Seule"
const MIN_SPD = 0.05;       // Vitesse minimale "en mouvement" (m/s)

let P_spd = 100;            // Incertitude initiale de la vitesse (P)
let P_alt = 100;            // Incertitude initiale de l'altitude (P)
let speedEst = 0;           // Vitesse estimée par l'EKF (m/s)
let altEst = 0;             // Altitude estimée par l'EKF (m)
let lastSpeed = 0;
let lastAltitude = 0;

// --- VARIABLES D'ÉTAT (DONNÉES MESURÉES/CALCULÉES) ---
let lat = null, lon = null, alt = null; // Coordonnées GPS EKF
let accuracy = 1000;        // Précision GPS brute (m)
let wID = null;             // ID du watcher Geolocation
let currentGPSMode = 'HIGH_FREQ'; 

let distM = 0;              // Distance totale parcourue (m)
let timeMoving = 0;         // Temps passé en mouvement (s)
let maxSpd = 0;             // Vitesse maximale (km/h)

// Variables Météo/Physique
let airDensity = 1.225;     // Densité de l'air (kg/m³) au niveau de la mer
let localGravity = G_BASE;  // Gravité locale (m/s²)
let speedOfSound = 343;     // Vitesse du son (m/s)
let tempK = 288.15;         // Température en Kelvin (15°C)
let lastP_hPa = 1013.25;    // Pression en hPa

// Variables de contrôle
let emergencyStopActive = false;
let selectedEnvironment = 'NORMAL'; 
let currentCelestialBody = 'EARTH'; 
let currentMass = 70;       
let isXRayMode = false;     
let gpsAccuracyOverride = 0;

// --- FONCTION UTILITAIRE ET BASE ---

/** Récupère un élément du DOM par son ID. */
const $ = (id) => document.getElementById(id);

/** Synchronise l'heure locale avec une heure serveur simulée (NTP simulé). */
function getCDate(lServH, lLocH) {
    // Si la synchro n'est pas établie, retourne l'heure locale
    if (lServH && lLocH) {
        const offset = lServH.getTime() - lLocH.getTime();
        return new Date(new Date().getTime() + offset);
    }
    return new Date(); 
}

/** Calcule le bruit de mesure (R) pour l'EKF basé sur la précision GPS. */
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
// =================================================================
// BLOC 2/4 : Fonctions de Physique, Météo et Calculs Secondaires
// Définit les fonctions de calcul des facteurs environnementaux et physiques.
// =================================================================

/** Calcule la densité de l'air (rho) selon la loi des gaz parfaits. */
function calculateAirDensity(tempK, pressure_hPa, humidity_perc) {
    // Pression en Pascals (Pa)
    const P_Pa = pressure_hPa * 100;
    
    // Équation pour l'air sec (simple) : ρ = P / (R_AIR * T)
    // Pour une précision professionnelle, utiliser l'air humide (plus complexe) :
    
    // Fraction de vapeur d'eau (ignorée ici pour simplifier, on prend l'air sec)
    // const Rv = 461.5; // Constante pour la vapeur d'eau
    // const Td = 0; // Température du point de rosée (nécessaire pour la pression de vapeur)
    
    const rho = P_Pa / (R_AIR * tempK); // Formule simple air sec
    return rho;
}

/** Calcule la vitesse du son dans l'air (m/s) en fonction de la température (K). */
function calculateSpeedOfSound(tempK) {
    const GAMMA = 1.4; // Coefficient adiabatique de l'air
    // a = sqrt(gamma * R_AIR * T)
    return Math.sqrt(GAMMA * R_AIR * tempK);
}

/** Calcule le facteur de Lorentz (γ) pour la relativité restreinte. */
function calculateLorentzFactor(speed_ms) {
    const ratio = speed_ms / C_L;
    return 1 / Math.sqrt(1 - (ratio * ratio));
}

/** Calcule la gravité locale selon le modèle de l'ellipsoïde terrestre (WGS84). */
function calculateLocalGravity(latitude_rad, altitude_m) {
    // Formule d'approximation pour la gravité standard à la latitude phi (m/s²)
    const sin2 = Math.sin(latitude_rad) * Math.sin(latitude_rad);
    const g0 = 9.780327 * (1 + 0.0053024 * sin2 - 0.0000058 * sin2 * sin2);
    
    // Correction pour l'altitude (approximation linéaire pour petites altitudes)
    const g_alt = g0 - (3.086e-6 * altitude_m); 
    
    return g_alt;
}

/** Calcule les données de temps solaire (TST, MST, EOT). */
function getSolarTime(date, longitude_deg) {
    // NOTE: C'est un calcul astronomique complexe simplifié
    // Heure solaire vraie (TST), Équation du temps (EOT), Longitude écliptique (ECL_LONG)
    return {
        TST: "N/A", // Pour l'implémentation complète
        MST: "N/A",
        EOT: "N/A",
        ECL_LONG: "N/A"
    };
}

/** Détermine le nom de la phase de la lune. */
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

/**
 * Simule l'acquisition de données météo (devrait être remplacé par un appel API réel).
 * Les données de pression et température sont essentielles pour l'EKF d'altitude.
 */
function fetchWeatherAndAirDensity(lat, lon) {
    return new Promise(resolve => {
        // En production, cette fonction appellerait une API Météo externe
        // Ici, nous utilisons des valeurs fixes pour permettre le calcul de la densité
        const pressure_hPa = lastP_hPa + (Math.random() - 0.5) * 0.5;
        const tempC = tempK - KELVIN_OFFSET + (Math.random() - 0.5) * 0.2;
        const tempK_new = tempC + KELVIN_OFFSET;
        const humidity_perc = 60 + (Math.random() - 0.5) * 5;
        
        const air_density = calculateAirDensity(tempK_new, pressure_hPa, humidity_perc);
        const dew_point = tempC - ((100 - humidity_perc) / 5); // Simplifié

        resolve({
            tempC: tempC,
            tempK: tempK_new,
            pressure_hPa: pressure_hPa,
            humidity_perc: humidity_perc,
            air_density: air_density,
            dew_point: dew_point
        });
    });
            }
// =================================================================
// BLOC 3/4 : Acquisition de Données Réelles, EKF et Mise à Jour du DOM
// Contient la gestion GPS, le calcul EKF, et les fonctions de rafraîchissement d'affichage.
// =================================================================

/** Implémentation du Filtre de Kalman Étendu pour la vitesse et l'altitude. */
function updateEKF(coords, speedMeasure, accuracy_m, dt) {
    // 1. Prediction (Vitesse et Altitude)
    const altMeasure = coords.altitude || altEst;
    
    // EKF Vitesse
    const speedPred = speedEst;
    P_spd = P_spd + Q_NOISE * dt;
    const R_spd = calculateR(accuracy_m);
    let K_spd = P_spd / (P_spd + R_spd);
    speedEst = speedPred + K_spd * (speedMeasure - speedPred);
    P_spd = (1 - K_spd) * P_spd;
    
    // EKF Altitude (simplifié)
    const altPred = altEst;
    P_alt = P_alt + Q_ALT * dt;
    const R_alt = accuracy_m * accuracy_m; // R basé sur la précision GPS verticale
    let K_alt = P_alt / (P_alt + R_alt);
    altEst = altPred + K_alt * (altMeasure - altPred);
    P_alt = (1 - K_alt) * P_alt;

    // --- MISE À JOUR DES COMPTEURS ET STATISTIQUES ---
    const speedDelta = Math.abs(speedEst - lastSpeed);
    if (speedEst >= MIN_SPD) {
        distM += speedEst * dt;
        timeMoving += dt;
        maxSpd = Math.max(maxSpd, speedEst * KMH_MS);
    }
    lastSpeed = speedEst;
    
    // Mise à jour de toutes les variables d'état
    lat = coords.latitude;
    lon = coords.longitude;
    alt = altEst; // On utilise l'altitude EKF
    accuracy = accuracy_m;

    // --- CALCULS PHYSIQUES AVANCÉS ---
    localGravity = calculateLocalGravity(lat * D2R, altEst);
    speedOfSound = calculateSpeedOfSound(tempK);
    const machNumber = speedEst / speedOfSound;
    const lorentzFactor = calculateLorentzFactor(speedEst);
    
    // Mise à jour de tous les affichages
    updateGPSDisplay(coords, speedMeasure, accuracy_m, dt);
    updateEKFDisplay(speedMeasure, R_spd, K_spd, P_spd, accuracy_m, altEst, P_alt, localGravity, machNumber, lorentzFactor);
}

/** Gestionnaire de succès GPS - Appelé à chaque mise à jour de position. */
function gpsSuccess(position) {
    const dt = DOM_FAST_UPDATE_MS / 1000;
    const speedRaw = position.coords.speed || 0;
    const newAccuracy = gpsAccuracyOverride || position.coords.accuracy;

    updateEKF(position.coords, speedRaw, newAccuracy, dt);
}

/** Gestionnaire d'erreurs GPS. */
function gpsError(error) {
    console.error(`Erreur GPS (${error.code}): ${error.message}`);
    if ($('gps-status-dr')) $('gps-status-dr').textContent = `ERREUR (${error.code})`;
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
    if ($('gps-status-dr')) $('gps-status-dr').textContent = 'ACTIF (GPS)';
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

// --- FONCTIONS DE MISE À JOUR DU DOM ---

let map = null, marker = null, track = [];
function initMap() {
    if ($('map') && !map) {
        map = L.map('map').setView([43.296, 5.370], 13);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { attribution: '© OpenStreetMap contributors' }).addTo(map);
        marker = L.marker([0, 0]).addTo(map);
    }
}

function updateGPSDisplay(coords, speedRaw, accuracy_m, dt) {
    if ($('speed-stable')) $('speed-stable').textContent = `${(speedEst * KMH_MS).toFixed(1)} km/h`;
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${speedEst.toFixed(2)} m/s`;
    if ($('speed-raw-ms')) $('speed-raw-ms').textContent = `${speedRaw.toFixed(2)} m/s`;
    
    // Position EKF
    if ($('lat-display')) $('lat-display').textContent = lat.toFixed(6) + '°';
    if ($('lon-display')) $('lon-display').textContent = lon.toFixed(6) + '°';
    if ($('alt-display')) $('alt-display').textContent = altEst.toFixed(2) + ' m';
    
    // Distances et Max
    if ($('distance-total-km')) $('distance-total-km').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(1)} m`;
    if ($('speed-max')) $('speed-max').textContent = `${maxSpd.toFixed(1)} km/h`;

    // Carte
    if (map && marker) {
        const latLng = [coords.latitude, coords.longitude];
        marker.setLatLng(latLng);
        map.setView(latLng, map.getZoom() > 14 ? map.getZoom() : 14);
        track.push(latLng);
        // La logique complète du tracé de la piste est omise ici
    }
    if ($('gps-precision')) $('gps-precision').textContent = `${accuracy_m.toFixed(2)} m`;
}

function updateEKFDisplay(speedMeasure, R_spd, K_spd, P_spd, accuracy_m, altEst, P_alt, localGravity, machNumber, lorentzFactor) {
    // EKF/Debug
    if ($('kalman-uncert')) $('kalman-uncert').textContent = `${P_spd.toFixed(4)}`;
    if ($('alt-uncertainty')) $('alt-uncertainty').textContent = `${Math.sqrt(P_alt).toFixed(2)} m`;
    if ($('speed-error-perc')) $('speed-error-perc').textContent = `${R_spd.toFixed(4)}`;
    if ($('gps-accuracy-display')) $('gps-accuracy-display').textContent = `${accuracy_m.toFixed(2)} m`;
    
    // Dynamique & Forces
    if ($('gravity-local')) $('gravity-local').textContent = `${localGravity.toFixed(4)} m/s²`;
    if ($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = `${speedOfSound.toFixed(2)} m/s`;
    if ($('mach-number')) $('mach-number').textContent = `${machNumber.toFixed(4)}`;
    if ($('perc-speed-sound')) $('perc-speed-sound').textContent = `${(machNumber * 100).toFixed(2)} %`;
    if ($('lorentz-factor')) $('lorentz-factor').textContent = `${lorentzFactor.toFixed(4)}`;
    if ($('perc-speed-c')) $('perc-speed-c').textContent = `${(speedEst / C_L * 100).toExponential(2)} %`;
    // La force de Coriolis et la pression dynamique nécessitent des données de vitesse et d'accélération
}

function updateClockVisualization(now, sunPos, moonPos, sunTimes) {
    // ... Logique de rotation et de couleur du ciel ... (omise pour la concision)
    const skyState = 'day'; // Placeholder
    const clock = $('minecraft-clock');
    if (clock) {
        clock.className = ''; 
        if (skyState === 'day') clock.classList.add('sky-day');
    }
    const sunEl = $('sun-element');
    const moonEl = $('moon-element');
    if (sunEl) sunEl.style.transform = `rotate(${sunPos.altitude * R2D + 90}deg)`;
    if (moonEl) moonEl.style.transform = `rotate(${moonPos.altitude * R2D + 90}deg)`;
}

function updateAstro(latA, lonA, lServH, lLocH) {
    const now = getCDate(lServH, lLocH);
    
    const sunPos = SunCalc.getPosition(now, latA, lonA);
    const moonPos = SunCalc.getMoonPosition(now, latA, lonA);
    const moonIllum = SunCalc.getMoonIllumination(now);
    const sunTimes = SunCalc.getTimes(now, latA, lonA);
    const solarTimes = getSolarTime(now, lonA); 
    
    // Mise à jour du DOM Astro
    if ($('tst')) $('tst').textContent = solarTimes.TST;
    // ... autres mises à jour Astro (eot, sun-alt, moon-phase-name, etc.) ...

    if ($('sun-alt')) $('sun-alt').textContent = `${(sunPos.altitude * R2D).toFixed(2)}°`;
    if ($('sun-azimuth')) $('sun-azimuth').textContent = `${((sunPos.azimuth * R2D + 180) % 360).toFixed(2)}°`;
    if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(moonIllum.phase);
    if ($('moon-illuminated')) $('moon-illuminated').textContent = `${(moonIllum.fraction * 100).toFixed(1)}%`;
    
    updateClockVisualization(now, sunPos, moonPos, sunTimes);
            }
// =================================================================
// BLOC 4/4 : Initialisation de l'Application et Boucles d'Intervalle
// Contient la logique de démarrage du programme.
// =================================================================

/**
 * Boucle de mise à jour rapide (Fast Update Loop).
 * Met à jour l'affichage du temps local et de la session.
 */
function domUpdateLoop() {
    // Obtient l'heure synchronisée
    const now = getCDate(lServH, lLocH);
    if (now) {
        if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString('fr-FR', { hour12: false });
        if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
    }
    
    // Mise à jour de l'affichage de la durée de mouvement
    if ($('time-moving')) $('time-moving').textContent = timeMoving.toFixed(2) + ' s';
}

/**
 * Initialise l'application et configure les boucles de mesure et de calcul réel.
 */
function init() {
    // Initialise la carte Leaflet
    initMap();
    
    // Démarre l'acquisition des données GPS/Geolocalisation réelle
    setGPSMode(currentGPSMode);
    
    // Boucle rapide pour la mise à jour du DOM (temps local, etc.)
    setInterval(domUpdateLoop, DOM_FAST_UPDATE_MS); 

    // Boucle lente pour les calculs Astro et la Météo
    setInterval(() => {
        // Acquisition des données météo/environnementales
        if (lat !== null && lon !== null) {
            fetchWeatherAndAirDensity(lat, lon).then(data => {
                // Met à jour les variables globales (tempK, lastP_hPa, airDensity)
                tempK = data.tempK;
                lastP_hPa = data.pressure_hPa;
                airDensity = data.air_density;

                // Met à jour le DOM météo
                if ($('temp-air-2')) $('temp-air-2').textContent = `${data.tempC.toFixed(1)} °C`;
                if ($('pressure-2')) $('pressure-2').textContent = `${data.pressure_hPa.toFixed(0)} hPa`;
                if ($('air-density')) $('air-density').textContent = `${data.air_density.toFixed(3)} kg/m³`;
                if ($('dew-point')) $('dew-point').textContent = `${data.dew_point.toFixed(1)} °C`;
            });

            // Calculs astronomiques
            updateAstro(lat, lon, lServH, lLocH);
        }
    }, DOM_SLOW_UPDATE_MS);

    // --- CONFIGURATION DES LISTENERS D'ÉVÉNEMENTS (Contrôle du système de mesure) ---
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

    // Reset des compteurs de mesure
    $('reset-dist-btn').addEventListener('click', () => { distM = 0; timeMoving = 0; maxSpd = 0; });
    $('reset-max-btn').addEventListener('click', () => { maxSpd = 0; });
    $('reset-all-btn').addEventListener('click', () => { window.location.reload(); });
    
    // Paramètres EKF/Physique
    $('gps-accuracy-override').addEventListener('input', (e) => { gpsAccuracyOverride = parseFloat(e.target.value); });
    $('mass-input').addEventListener('input', (e) => { currentMass = parseFloat(e.target.value); $('mass-display').textContent = currentMass.toFixed(3) + ' kg'; });
    $('celestial-body-select').addEventListener('change', (e) => { currentCelestialBody = e.target.value; });
    $('environment-select').addEventListener('change', (e) => { selectedEnvironment = e.target.value; });
    
    // Bouton Rayons X (Visualisation Astro)
    $('xray-button').addEventListener('click', () => { 
        isXRayMode = !isXRayMode;
        document.getElementById('minecraft-clock').classList.toggle('x-ray', isXRayMode);
        $('xray-button').textContent = isXRayMode ? 'ON' : 'X';
    });
}

// Démarre l'application après le chargement complet du HTML
document.addEventListener('DOMContentLoaded', init);
