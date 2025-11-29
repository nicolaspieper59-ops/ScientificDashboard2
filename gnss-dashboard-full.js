// =================================================================
// BLOC 1/4 : Utilitaires, Constantes et État Global (Initialisation)
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);

/** Formate une valeur numérique. */
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || val === Infinity) {
        return (decimals === 0 ? '0' : '0.00') + suffix;
    }
    return val.toFixed(decimals) + suffix;
};

// --- CLÉS D'API & ENDPOINTS ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

// --- CONSTANTES PHYSIQUES ET ISA (Atmosphère Standard Internationale) ---
const KMH_MS = 3.6;             // Conversion m/s vers km/h
const TEMP_SEA_LEVEL_K = 288.15; // 15°C en Kelvin
const RHO_SEA_LEVEL = 1.225;    // Densité de l'air au niveau de la mer (kg/m³)

// --- ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE ---
let gpsWatcherID = null;    // ID de la surveillance GPS
let map = null;             // Instance Leaflet
let domFastID = null;       // ID du setInterval pour la fastLoop (UKF)
let domSlowID = null;       // ID du setInterval pour la slowLoop (DOM/Astro)
let lastIMUTimestamp = 0;   // Horodatage pour le calcul du dt UKF
let totalElapsedTime = 0;   // Temps total écoulé depuis le démarrage

// Initialisation critique avec des coordonnées de travail (Marseille par défaut)
let currentPosition = { 
    lat: 43.2964,   
    lon: 5.3697,    
    alt: 0.0,
    acc: 10.0,      
    spd: 0.0,       
    timestamp: Date.now()
};

// Variables des capteurs bruts (utilisés par l'UKF)
let accel = { x: 0, y: 0, z: 0 };
let gyro = { x: 0, y: 0, z: 0 };

// Variables EKF/UKF (Simplifié)
let currentSpeedEKF = 0.0;
let currentAltitudeEKF = 0.0;
let currentAirDensity = RHO_SEA_LEVEL;

// --- Vérification des dépendances critiques (à ajouter si elles ne sont pas dans une IIFE) ---
if (typeof math === 'undefined' || typeof L === 'undefined' || typeof SunCalc === 'undefined' || typeof turf === 'undefined') {
    console.error(`Erreur critique : Dépendances (math, leaflet, suncalc, turf) manquantes.`);
}

// =================================================================
// BLOC 2/4 : Gestion des Capteurs, Démarrage du Moteur & Cartographie
// =================================================================

// --- FONCTIONS DE DÉMARRAGE DU MOTEUR UKF (CRITIQUE) ---

function startFastLoop() {
    // Correction 2 : S'assurer que la boucle UKF ne démarre qu'UNE SEULE fois
    if (domFastID === null) { 
        const FAST_UPDATE_MS = 20; // 50 Hz pour UKF/IMU
        domFastID = setInterval(fastLoop, FAST_UPDATE_MS); 
        console.log("UKF/IMU Fast Loop Démarrée à 50Hz.");
        
        // Lance la boucle LENTE pour l'affichage si ce n'est pas déjà fait
        if (domSlowID === null) {
             startSlowLoop(); 
        }
    }
}

// --- GESTION GPS ---

const GPS_OPTIONS = {
    enableHighAccuracy: true,
    timeout: 30000,
    maximumAge: 500
};

function onPositionSuccess(pos) {
    // Correction 3 : Démarre l'UKF/FastLoop au premier signal GPS (Mécanisme de secours/Fallback)
    if (domFastID === null) {
        startFastLoop(); 
    }
    
    // Mise à jour de la position globale
    currentPosition.lat = pos.coords.latitude;
    currentPosition.lon = pos.coords.longitude;
    currentPosition.alt = pos.coords.altitude || 0.0;
    currentPosition.acc = pos.coords.accuracy;
    currentPosition.spd = pos.coords.speed || 0.0;
    currentPosition.timestamp = pos.timestamp;
    
    // ... Logique de mise à jour/Correction de l'état UKF (UKF_Update) ...
    
    // Mise à jour du DOM
    if ($('gps-precision')) $('gps-precision').textContent = `${dataOrDefault(currentPosition.acc, 2)} m`;
    if ($('gps-toggle-btn')) {
        $('gps-toggle-btn').textContent = "🟢 MARCHE GPS";
        $('gps-toggle-btn').classList.remove('btn-warning', 'btn-danger');
        $('gps-toggle-btn').classList.add('btn-success');
    }
}

function startGPS() {
    if (gpsWatcherID === null) {
        if ($('gps-toggle-btn')) $('gps-toggle-btn').textContent = "🟡 Acquisition GPS...";
        
        // Lance la surveillance GPS
        gpsWatcherID = navigator.geolocation.watchPosition(
            onPositionSuccess,
            onPositionError,
            GPS_OPTIONS
        );
        
        // Correction 1 : Lance l'IMU avec le même geste utilisateur (CRITIQUE)
        startIMUListeners(); 
        startMap();
    }
}

function onPositionError(err) {
    console.error(`Erreur GPS (${err.code}): ${err.message}`);
    const errMsg = (err.code === 1) ? "🔴 Accès refusé." : `🔴 ERREUR GPS (${err.code})`;
    if ($('gps-toggle-btn')) {
        $('gps-toggle-btn').textContent = errMsg;
        $('gps-toggle-btn').classList.remove('btn-success');
        $('gps-toggle-btn').classList.add('btn-danger');
    }
    if ($('gps-precision')) $('gps-precision').textContent = `Erreur: ${err.message}`;
}


// --- GESTION IMU (Accéléromètre/Gyroscope) ---
// Utilisation des API Sensor (plus modernes et plus précis)
function startIMUListeners() {
    if (domFastID) return; // IMU est déjà actif via FastLoop
    try {
        if (typeof Accelerometer === 'undefined' || typeof Gyroscope === 'undefined') {
            throw new Error("API Capteurs non supportée.");
        }
        
        if ($('imu-status')) $('imu-status').textContent = "Activation...";

        // Initialisation de l'accéléromètre
        const accSensor = new Accelerometer({ frequency: 50 });
        accSensor.addEventListener('reading', () => { 
            accel.x = accSensor.x;
            accel.y = accSensor.y;
            accel.z = accSensor.z;
        });
        accSensor.addEventListener('error', e => console.error("Accéléromètre:", e.error));
        accSensor.start();

        // Initialisation du gyroscope
        const gyroSensor = new Gyroscope({ frequency: 50 });
        gyroSensor.addEventListener('reading', () => { 
            gyro.x = gyroSensor.x;
            gyro.y = gyroSensor.y;
            gyro.z = gyroSensor.z;
        });
        gyroSensor.addEventListener('error', e => console.error("Gyroscope:", e.error));
        gyroSensor.start();

        if ($('imu-status')) $('imu-status').textContent = "Actif (API Sensor 50Hz)";
        lastIMUTimestamp = performance.now();
        
        // Démarre le moteur après succès de l'initialisation du capteur
        startFastLoop(); 
    } catch (error) {
        let msg = error.message;
        // Correction 4 : Message d'erreur clair pour les problèmes de sécurité
        if (error.name === 'SecurityError' || error.name === 'NotAllowedError') {
            msg = "Permission Capteurs refusée. Recliquez ou vérifiez les paramètres du navigateur.";
        }
        console.error("Erreur IMU:", error);
        if ($('imu-status')) $('imu-status').textContent = `IMU ÉCHOUÉ : ${msg}`;
        // La fastLoop démarrera via le Fallback GPS si la permission est refusée ici.
    }
}

// --- CARTOGRAPHIE ---
function startMap() {
    if (map === null && $('#map-gnss')) { 
        map = L.map('map-gnss').setView([currentPosition.lat, currentPosition.lon], 13);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19 }).addTo(map);
        if ($('map-status')) $('map-status').textContent = "Carte chargée.";
        // ... Logique pour le marker et le suivi
    }
}

// =================================================================
// BLOC 3/4 : Le Moteur (FastLoop UKF) et Boucles Lentes (Affichage/API)
// =================================================================

// --- BOUCLE HAUTE FRÉQUENCE (UKF/IMU) ---
function fastLoop() {
    // Correction 5 : Calcul précis du temps écoulé (dt) pour l'UKF
    const now = performance.now();
    let dt = (now - lastIMUTimestamp) / 1000; // dt en secondes
    lastIMUTimestamp = now;
    
    // Clamp de dt pour éviter les valeurs extrêmes après un blocage
    if (dt > 0.1 || dt < 0.001) dt = 0.02; 
    
    // Mise à jour des temps de session
    totalElapsedTime += dt;

    // ... Logique de Prédiction UKF (basée sur dt, accel, gyro, etc.) ...
    // UKF_Predict(dt, accel, gyro, ...); 
    
    // Mise à jour de la vitesse stable (affichage rapide)
    if ($('stable-speed-kmh')) $('stable-speed-kmh').textContent = `${dataOrDefault(currentSpeedEKF * KMH_MS, 2)} km/h`;
    if ($('elapsed-time')) $('elapsed-time').textContent = `${dataOrDefault(totalElapsedTime, 2)} s`;
}


// --- BOUCLE BASSE FRÉQUENCE (Affichage / Astro / Météo) ---
function startSlowLoop() {
    if (domSlowID === null) { 
        const DOM_SLOW_UPDATE_MS = 1000; // 1 Hz
        domSlowID = setInterval(slowLoop, DOM_SLOW_UPDATE_MS); 
    }
}

function slowLoop() {
    // 1. Mise à jour Horloge NTP (Synchronisation)
    // syncH(); // Assumer que syncH existe et appelle l'API de temps NTP
    
    // 2. Mise à jour Météo & Pollution (Uniquement si la position n'est pas par défaut)
    if (currentPosition.lat !== 43.2964) {
        // fetchWeather(currentPosition.lat, currentPosition.lon).catch(() => console.warn("Météo Échouée"));
        // fetchPollutants(currentPosition.lat, currentPosition.lon).catch(() => console.warn("Pollution Échouée"));
    }
    
    // 3. Mise à jour de l'Astro
    // updateAstro(currentPosition.lat, currentPosition.lon); // Assumer que updateAstro existe

    // ... autres mises à jour DOM et de l'altitude EKF
    if ($('ekf-altitude')) $('ekf-altitude').textContent = `${dataOrDefault(currentAltitudeEKF, 2)} m`;
}


// =================================================================
// BLOC 4/4 : Logique de Contrôle et Initialisation Finale
// =================================================================

function stopGPS() {
    if (gpsWatcherID !== null) {
        navigator.geolocation.clearWatch(gpsWatcherID);
        gpsWatcherID = null;
    }
    // Note: L'arrêt des boucles Fast/Slow doit être géré séparément ou dans une fonction d'arrêt totale
}

/** 🚀 Fonction d'initialisation finale lancée après le chargement du DOM */
window.addEventListener('load', () => {
    // 1. Gestion du bouton GPS (le point de départ unique)
    const gpsBtn = $('gps-toggle-btn');
    if (gpsBtn) {
        gpsBtn.addEventListener('click', () => {
            if (gpsWatcherID === null) {
                startGPS();
            } else {
                stopGPS();
                gpsBtn.textContent = "▶️ MARCHE GPS";
            }
        });
    }

    // 2. Démarrage initial de la partie statique (Astro, Météo ISA)
    if (domSlowID === null) {
        startSlowLoop();
    }
    
    // 3. Initialisation de la carte et synchro NTP
    // syncH(); // Démarrage initial de l'horloge NTP
    startMap(); 
}); 
