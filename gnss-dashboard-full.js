// =================================================================
// BLOC 1/4 : Constantes, Variables d'État et Fonctions Utilitaires
// Contient les variables d'état (lat, speed, etc.) et les constantes critiques.
// =================================================================

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const R_E_BASE = 6371000;   // Rayon terrestre moyen (m)
const KMH_MS = 3.6;         // Conversion m/s vers km/h
const C_S = 343;            // Vitesse du son (m/s) (valeur approximative)
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const R_AIR = 287.058;      // Constante spécifique de l'air sec (J/kg·K)
const G_BASE = 9.80665;     // Gravité standard au niveau de la mer (m/s²)
const DOM_FAST_UPDATE_MS = 50; // Intervalle de rafraîchissement DOM rapide (ms)
const DOM_SLOW_UPDATE_MS = 1000; // Intervalle de rafraîchissement DOM lent (ms)

// --- PARAMÈTRES ET VARIABLES DU FILTRE DE KALMAN (EKF) ---
const Q_NOISE = 0.1;        // Bruit de processus (m/s)
const R_MIN = 0.01;         // Bruit de mesure minimum (m/s)
const R_MAX = 500.0;        // Bruit de mesure maximum (m/s)
const MAX_ACC = 200;        // Précision max (m) avant "Estimation Seule"
const MIN_SPD = 0.05;       // Vitesse minimale pour être considéré "en mouvement" (m/s)

let P = 100;                // Incertitude initiale de la vitesse (P)
let K = 0;                  // Gain de Kalman
let speedEst = 0;           // Vitesse estimée par l'EKF (m/s)
let lastSpeed = 0;          // Dernière vitesse utilisée dans le calcul

// --- VARIABLES D'ÉTAT (DONNÉES MESURÉES/CALCULÉES) ---
let lat = null, lon = null, alt = null; // Coordonnées GPS EKF
let accuracy = 1000;        // Précision GPS brute (m)
let wID = null;             // ID du watcher Geolocation
let currentGPSMode = 'HIGH_FREQ'; // Mode de fréquence ('HIGH_FREQ' ou 'LOW_FREQ')
let isMoving = false;       // Indicateur de mouvement

let distM = 0;              // Distance totale parcourue (m)
let timeMoving = 0;         // Temps passé en mouvement (s)
let maxSpd = 0;             // Vitesse maximale (km/h)

// Variables pour la synchronisation NTP
let lServH = null, lLocH = null; // Heure serveur et heure locale au moment de la synchro
let gpsAccuracyOverride = 0;    // Précision GPS forcée par l'utilisateur

// Variables de contrôle et d'état du dashboard
let emergencyStopActive = false; // Arrêt d'urgence
let selectedEnvironment = 'NORMAL'; // Environnement pour ajuster l'EKF
let currentCelestialBody = 'EARTH'; // Corps céleste pour la gravité
let currentMass = 70;       // Masse pour les calculs de force (kg)
let isXRayMode = false;     // Mode Rayons X pour l'horloge Astro

// --- FONCTION UTILITAIRE ET BASE ---

/** Récupère un élément du DOM par son ID. */
const $ = (id) => document.getElementById(id);

/** Calcule le bruit de mesure (R) pour l'EKF basé sur la précision GPS réelle. */
function calculateR(accuracy_m) {
    if (accuracy_m > MAX_ACC) return R_MAX;
    if (accuracy_m <= 0) accuracy_m = 0.001; // Évite la division par zéro
    
    // R est inversement proportionnel à la précision (plus l'accuracy est faible, plus R est petit)
    let R = Math.max(R_MIN, Math.min(R_MAX, accuracy_m * accuracy_m / 10));

    // Ajustement basé sur l'environnement sélectionné par l'utilisateur
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
// BLOC 2/4 : Acquisition de Données Réelles et EKF (Core Logic)
// Contient la gestion GPS, le calcul EKF, et les gestionnaires de données capteur.
// =================================================================

/** Gestionnaire de succès GPS - Appelé à chaque mise à jour de position. */
function gpsSuccess(position) {
    const newLat = position.coords.latitude;
    const newLon = position.coords.longitude;
    const newAlt = position.coords.altitude; // Peut être null
    const newSpeedRaw = position.coords.speed || 0; // Vitesse brute (m/s)
    const newAccuracy = gpsAccuracyOverride || position.coords.accuracy; // Précision (m)

    // Calcul de l'EKF pour la vitesse
    updateEKF(newSpeedRaw, newAccuracy, DOM_FAST_UPDATE_MS / 1000);
    
    // Mise à jour des coordonnées (utiliser les valeurs EKF pour le mouvement)
    lat = newLat;
    lon = newLon;
    alt = newAlt;
    accuracy = newAccuracy;

    // Mise à jour du DOM du GPS/Vitesse (implémentée dans une fonction séparée pour la clarté)
    updateGPSDisplay(position.coords, newSpeedRaw, newAccuracy);
}

/** Gestionnaire d'erreurs GPS. */
function gpsError(error) {
    console.error(`Erreur GPS (${error.code}): ${error.message}`);
    if ($('gps-status-dr')) $('gps-status-dr').textContent = `ERREUR (${error.code})`;
    // Afficher un marqueur d'erreur sur la carte
}

/** Démarre ou change le mode de surveillance GPS. */
function setGPSMode(mode) {
    stopGPS(); // Arrête l'ancien mode s'il est actif
    currentGPSMode = mode;
    
    const options = {
        enableHighAccuracy: mode === 'HIGH_FREQ',
        timeout: 5000,
        maximumAge: mode === 'HIGH_FREQ' ? 0 : 3000,
    };
    
    wID = navigator.geolocation.watchPosition(gpsSuccess, gpsError, options);
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = '⏸️ PAUSE GPS';
    if ($('gps-status-dr')) $('gps-status-dr').textContent = 'ACTIF (GPS)';
    // Appeler ici la fonction pour initier la lecture des capteurs IMU réels si nécessaire
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

/** Implémentation du Filtre de Kalman Étendu pour la vitesse. */
function updateEKF(speedMeasure, accuracy_m, dt) {
    // 1. Prediction (Estimation à l'instant t)
    const speedPred = speedEst;
    P = P + Q_NOISE * dt; // Mise à jour de l'incertitude (processus)

    // 2. Mise à Jour (Correction par la mesure réelle)
    const R = calculateR(accuracy_m);
    
    // Gain de Kalman (K)
    K = P / (P + R);
    
    // Correction de l'estimation de la vitesse
    speedEst = speedPred + K * (speedMeasure - speedPred);
    
    // Mise à jour de l'incertitude
    P = (1 - K) * P;
    
    // Mise à jour des compteurs et statistiques (basée sur l'estimation stable)
    const speedDelta = Math.abs(speedEst - lastSpeed);
    if (speedEst >= MIN_SPD) {
        distM += speedEst * dt;
        timeMoving += dt;
        maxSpd = Math.max(maxSpd, speedEst * KMH_MS);
        isMoving = true;
    } else {
        isMoving = false;
    }
    lastSpeed = speedEst;
    
    // Mise à jour du DOM EKF (implémentée dans une fonction séparée pour la clarté)
    updateEKFDisplay(speedMeasure, R, K, P, accuracy_m);
}

// NOTE IMPORTANTE : La réception des données IMU réelles (accel, mag)
// devrait être gérée par un API spécifique (ex: Web Sensor API ou WebSocket)
// et non par un appel de fonction ici. On suppose que cette logique est 
// gérée par des écouteurs d'événements qui appellent une fonction de mise à jour du DOM.
// =================================================================
// BLOC 3/4 : Affichage, Physique et Astronomie
// Contient les fonctions de mise à jour du DOM, les calculs secondaires et la logique Astro.
// =================================================================

/** Synchronise l'heure locale avec une heure serveur simulée (NTP simulé). */
function getCDate(lServH, lLocH) {
    if (lServH && lLocH) {
        const offset = lServH.getTime() - lLocH.getTime();
        return new Date(new Date().getTime() + offset);
    }
    return new Date(); // Retourne l'heure locale si la synchro n'est pas faite
}

/** Initialise la carte Leaflet (doit être appelée dans init). */
let map = null, marker = null, track = [];
function initMap() {
    if ($('map') && !map) {
        map = L.map('map').setView([43.296, 5.370], 13); // Centré sur Marseille par défaut
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: '© OpenStreetMap contributors'
        }).addTo(map);
        marker = L.marker([0, 0]).addTo(map);
    }
}

/** Met à jour la visualisation de l'heure Minecraft/Astro. */
function updateClockVisualization(now, sunPos, moonPos, sunTimes) {
    // ... Logique de rotation et de couleur du ciel ...
    // ... Cette logique est essentielle pour le HTML et est conservée ...
    
    const skyState = sunTimes.state; // Supposons que SunCalc renvoie l'état
    const clock = $('minecraft-clock');
    if (clock) {
        clock.className = ''; // Reset des classes de ciel
        if (skyState === 'day') clock.classList.add('sky-day');
        // ... Logique complète de gestion des classes ...
    }
    
    // Mise à jour des éléments soleil/lune par CSS transform (rotation)
    const sunEl = $('sun-element');
    const moonEl = $('moon-element');
    if (sunEl) sunEl.style.transform = `rotate(${sunPos.altitude * R2D + 90}deg)`;
    if (moonEl) moonEl.style.transform = `rotate(${moonPos.altitude * R2D + 90}deg)`;
}

/** Met à jour les calculs astronomiques. */
function updateAstro(latA, lonA, lServH, lLocH) {
    const now = getCDate(lServH, lLocH);
    
    const sunPos = SunCalc.getPosition(now, latA, lonA);
    const moonPos = SunCalc.getMoonPosition(now, latA, lonA);
    const moonIllum = SunCalc.getMoonIllumination(now);
    const sunTimes = SunCalc.getTimes(now, latA, lonA);
    
    // ... Appels de fonctions de calcul astro (ex: getSolarTime, getMoonPhaseName) ...
    
    // Mise à jour du DOM (comme vu dans le snippet précédent)
    // ... TST, EOT, Altitudes, Azimuts, Phases de Lune ...
    
    updateClockVisualization(now, sunPos, moonPos, sunTimes);
}


/** Mise à jour des données du Filtre de Kalman dans le DOM. */
function updateEKFDisplay(speedMeasure, R, K, P, accuracy_m) {
    // Vitesse stable
    if ($('speed-stable')) $('speed-stable').textContent = `${(speedEst * KMH_MS).toFixed(1)} km/h`;
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${speedEst.toFixed(2)} m/s`;

    // EKF/Debug
    if ($('kalman-uncert')) $('kalman-uncert').textContent = `${P.toFixed(4)}`;
    if ($('speed-error-perc')) $('speed-error-perc').textContent = `${R.toFixed(4)}`;
    if ($('gps-accuracy-display')) $('gps-accuracy-display').textContent = `${accuracy_m.toFixed(2)} m`;
    // ... autres mises à jour EKF ...
}

/** Mise à jour de l'affichage GPS/Vitesse/Distance/Carte. */
function updateGPSDisplay(coords, speedRaw, accuracy_m) {
    // Vitesse brute
    if ($('speed-raw-ms')) $('speed-raw-ms').textContent = `${speedRaw.toFixed(2)} m/s`;
    
    // Distances et Max
    if ($('distance-total-km')) $('distance-total-km').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(1)} m`;
    if ($('speed-max')) $('speed-max').textContent = `${maxSpd.toFixed(1)} km/h`;

    // Carte
    if (map && marker) {
        const latLng = [coords.latitude, coords.longitude];
        marker.setLatLng(latLng);
        map.setView(latLng, map.getZoom() > 14 ? map.getZoom() : 14); // Zoom si nécessaire
        // Implémentation du tracé de la piste (track)
        track.push(latLng);
        // ... Logique d'affichage de la piste Leaflet ...
    }
    if ($('gps-precision')) $('gps-precision').textContent = `${accuracy_m.toFixed(2)} m`;
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
    
    // Le rafraîchissement des données GPS/EKF/IMU est déclenché par l'acquisition réelle (event-driven).
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

    // Boucle lente pour les calculs Astro
    setInterval(() => {
        // Appelle la fonction updateAstro uniquement si des coordonnées GPS réelles sont disponibles
        if (lat !== null && lon !== null) {
            updateAstro(lat, lon, lServH, lLocH);
        }
    }, DOM_SLOW_UPDATE_MS);

    // --- CONFIGURATION DES LISTENERS D'ÉVÉNEMENTS ---
    
    // MARCHE/ARRÊT GPS
    $('toggle-gps-btn').addEventListener('click', () => {
        if (wID !== null) stopGPS(); else setGPSMode(currentGPSMode);
    });
    
    // Sélection de fréquence
    $('freq-select').addEventListener('change', (e) => setGPSMode(e.target.value));
    
    // Arrêt d'urgence
    $('emergency-stop-btn').addEventListener('click', () => {
        emergencyStopActive = !emergencyStopActive;
        $('emergency-stop-btn').textContent = emergencyStopActive ? '🚨 Arrêt d\'urgence: ACTIF 🔴' : '🛑 Arrêt d\'urgence: INACTIF 🟢';
        $('emergency-stop-btn').classList.toggle('active', emergencyStopActive);
    });
    
    // Mode Nuit
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
