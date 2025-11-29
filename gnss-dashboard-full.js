// =================================================================
// BLOC 1/4 : Utilitaires, Constantes, État Global & Vérification des Dépendances
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);
const toRad = deg => deg * (Math.PI / 180);
const toDeg = rad => rad * (180 / Math.PI);

/** Formate une valeur numérique. */
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || val === Infinity) {
        return (decimals === 0 ? '0' : '0.00') + suffix;
    }
    return val.toFixed(decimals) + suffix;
};

/** Formate en notation exponentielle. */
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || val === Infinity) {
        const zeroDecimals = '0.' + Array(decimals).fill('0').join('');
        return zeroDecimals + 'e+0' + suffix;
    }
    return val.toExponential(decimals) + suffix;
};

// --- CLÉS D'API & ENDPOINTS ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

// --- CONSTANTES PHYSIQUES ET ISA (Atmosphère Standard Internationale) ---
const C_L = 299792458;          // Vitesse de la lumière (m/s)
const KMH_MS = 3.6;             // Conversion m/s vers km/h
const TEMP_SEA_LEVEL_K = 288.15; // 15°C en Kelvin
const BARO_ALT_REF_HPA = 1013.25; // Pression de référence (hPa)
const RHO_SEA_LEVEL = 1.225;    // Densité de l'air au niveau de la mer (kg/m³)

// --- ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE ---
let gpsWatcherID = null;    // ID de la surveillance GPS
let map = null;             // Instance Leaflet
let domFastID = null;       // ID du setInterval pour la fastLoop (UKF)
let domSlowID = null;       // ID du setInterval pour la slowLoop (DOM/Astro)
let lastKnownPos = null;    // Dernière position GPS brute
let lastIMUTimestamp = 0;   // Horodatage pour le calcul du dt UKF
let totalElapsedTime = 0;   // Temps total écoulé depuis le démarrage
let totalMovementTime = 0;  // Temps passé en mouvement
let lastMovementTimeUpdate = 0; // Dernier temps d'actualisation du mouvement

let currentPosition = { 
    // Coordonnées initiales de travail (ex: Marseille)
    lat: 43.2964,   
    lon: 5.3697,    
    alt: 0.0,
    acc: 10.0,      
    spd: 0.0,       
    timestamp: Date.now()
};

// --- Variables EKF/UKF (Simplifié pour l'exemple) ---
// *Dans la version complète, X_UKF, P_UKF, et les fonctions math.js devraient être ici.*
let currentSpeedEKF = 0.0;
let currentAltitudeEKF = 0.0;
let currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound = 340.29; // Vitesse du son par défaut

// --- Vérification des dépendances critiques (à exécuter au chargement) ---
if (typeof math === 'undefined' || typeof L === 'undefined' || typeof SunCalc === 'undefined' || typeof turf === 'undefined') {
    const missing = [
        (typeof math === 'undefined' ? "math.min.js" : ""),
        (typeof L === 'undefined' ? "leaflet.js" : ""),
        (typeof SunCalc === 'undefined' ? "suncalc.js" : ""),
        (typeof turf === 'undefined' ? "turf.min.js" : "")
    ].filter(Boolean).join(", ");
    console.error(`Erreur critique : Dépendances manquantes : ${missing}.`);
    alert(`Erreur: Dépendances manquantes : ${missing}. L'application ne peut pas démarrer.`);
}
// Fin BLOC 1/4
// =================================================================

// =================================================================
// BLOC 2/4 : Gestion des Capteurs, Démarrage du Moteur & Cartographie
// =================================================================

// --- FONCTIONS DE DÉMARRAGE DU MOTEUR (fastLoop/slowLoop) ---

function startFastLoop() {
    // CORRECTION CRITIQUE A : S'assurer que la boucle UKF ne démarre qu'UNE SEULE fois
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

function startSlowLoop() {
    if (domSlowID === null) { 
        const DOM_SLOW_UPDATE_MS = 1000; // 1 Hz pour l'affichage lent
        domSlowID = setInterval(slowLoop, DOM_SLOW_UPDATE_MS); 
    }
}

// --- GESTION GPS ---

const GPS_OPTIONS = {
    enableHighAccuracy: true,
    timeout: 30000,
    maximumAge: 500
};

function onPositionSuccess(pos) {
    // CORRECTION CRITIQUE B : Démarre l'UKF/FastLoop au premier signal GPS (Mécanisme de secours)
    if (domFastID === null) {
        startFastLoop(); 
    }
    
    // Mise à jour de la position globale
    currentPosition.lat = pos.coords.latitude;
    currentPosition.lon = pos.coords.longitude;
    currentPosition.alt = pos.coords.altitude || 0.0;
    currentPosition.acc = pos.coords.accuracy;
    currentPosition.spd = pos.coords.speed || 0.0; // Vitesse GPS brute
    currentPosition.timestamp = pos.timestamp;
    
    // ... Logique de mise à jour/Correction de l'état UKF (UKF_Update) ...
    
    // Mise à jour de la carte
    updateMapMarker(currentPosition.lat, currentPosition.lon, currentPosition.acc);
    if ($('gps-precision')) $('gps-precision').textContent = `${dataOrDefault(currentPosition.acc, 2)} m`;
    
    $('gps-toggle-btn').textContent = "🟢 MARCHE GPS";
    $('gps-toggle-btn').classList.remove('btn-warning');
    $('gps-toggle-btn').classList.add('btn-success');
}

function onPositionError(err) {
    console.error(`Erreur GPS (${err.code}): ${err.message}`);
    $('gps-toggle-btn').textContent = "🔴 ERREUR GPS";
    $('gps-toggle-btn').classList.remove('btn-success');
    $('gps-toggle-btn').classList.add('btn-danger');
    
    if (err.code === 1) { // PERMISSION_DENIED
        $('gps-precision').textContent = "Accès refusé. Veuillez autoriser la géolocalisation.";
    } else {
        $('gps-precision').textContent = `Erreur: ${err.message}`;
    }
}

function startGPS() {
    if (gpsWatcherID === null) {
        $('gps-toggle-btn').textContent = "🟡 Acquisition GPS...";
        
        // Lance la surveillance GPS
        gpsWatcherID = navigator.geolocation.watchPosition(
            onPositionSuccess,
            onPositionError,
            GPS_OPTIONS
        );
        
        // CORRECTION CRITIQUE C : Lance l'IMU avec le même geste utilisateur
        startIMUListeners(); 
        startMap();
    }
}

// --- GESTION IMU (Accéléromètre/Gyroscope) ---
let accel = { x: 0, y: 0, z: 0 };
let gyro = { x: 0, y: 0, z: 0 };

function handleAccelerometerReading() {
    // Logique de traitement des données de l'accéléromètre
    // ... Mise à jour des variables 'accel'
    if ($('imu-accel-x')) $('imu-accel-x').textContent = `${dataOrDefault(accel.x, 3)} m/s²`;
    // ...
}

function startIMUListeners() {
    if (domFastID) return; // IMU est déjà actif via FastLoop
    try {
        if (typeof Accelerometer === 'undefined' || typeof Gyroscope === 'undefined') {
            throw new Error("API Capteurs non supportée.");
        }
        
        const accSensor = new Accelerometer({ frequency: 50 });
        accSensor.addEventListener('reading', handleAccelerometerReading);
        // ... Logique d'erreur
        accSensor.start();

        // ... Logique pour Gyroscope ...

        if ($('imu-status')) $('imu-status').textContent = "Actif (API Sensor 50Hz)";
        lastIMUTimestamp = performance.now();
        
        // Démarre le moteur après succès de l'initialisation du capteur
        startFastLoop(); 
    } catch (error) {
        let msg = error.message;
        if (error.name === 'SecurityError' || error.name === 'NotAllowedError') {
            msg = "Permission Capteurs refusée ou nécessite un geste utilisateur.";
        }
        console.error("Erreur IMU:", error);
        if ($('imu-status')) $('imu-status').textContent = `IMU ÉCHOUÉ : ${msg}`;
        // Laisse la fastLoop être démarrée par le GPS (Fallback)
    }
}

// --- CARTOGRAPHIE ---

function startMap() {
    if (map === null && $('#map-gnss')) { 
        map = L.map('map-gnss').setView([currentPosition.lat, currentPosition.lon], 13);
        
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            maxZoom: 19,
            attribution: '© OpenStreetMap'
        }).addTo(map);

        if ($('map-status')) $('map-status').textContent = "Carte chargée.";
        // Si la carte est dans un onglet non visible, il faut parfois appeler map.invalidateSize()
        map.on('locationfound', onPositionSuccess);
    }
}

// Fin BLOC 2/4
// =================================================================

// =================================================================
// BLOC 3/4 : Le Moteur (FastLoop UKF) et Mise à Jour des Variables DOM
// =================================================================

// --- BOUCLE HAUTE FRÉQUENCE (UKF/IMU) ---
function fastLoop() {
    // CRITIQUE D : Calcul du temps écoulé (dt) pour l'UKF
    const now = performance.now();
    let dt = (now - lastIMUTimestamp) / 1000; // dt en secondes
    lastIMUTimestamp = now;

    // Évite les dt aberrants
    if (dt > 0.1 || dt < 0.001) dt = 0.02; // Clamp à 50Hz
    
    // Mise à jour des temps de session
    totalElapsedTime += dt;
    if (currentSpeedEKF > 0.05) { // Vitesse supérieure à 0.05 m/s (minimum de mouvement)
        totalMovementTime += dt;
    }

    // ... Logique de Prédiction UKF (basée sur dt et accel/gyro) ...
    // ... Mise à jour de currentSpeedEKF, currentAltitudeEKF ...
    
    // Mise à jour de la vitesse stable (affichage rapide)
    if ($('stable-speed-kmh')) $('stable-speed-kmh').textContent = `${dataOrDefault(currentSpeedEKF * KMH_MS, 2)} km/h`;
    
    // Mise à jour du temps écoulé
    if ($('elapsed-time')) $('elapsed-time').textContent = `${dataOrDefault(totalElapsedTime, 2)} s`;
    if ($('movement-time')) $('movement-time').textContent = `${dataOrDefault(totalMovementTime, 2)} s`;
}


// --- BOUCLE BASSE FRÉQUENCE (Affichage / Astro / Météo) ---
function slowLoop() {
    // 1. Mise à jour de l'Astro
    if (currentPosition.lat !== 43.2964) { // Uniquement si la position n'est pas par défaut
        updateAstro(currentPosition.lat, currentPosition.lon);
    }

    // 2. Mise à jour Météo & Pollution
    if (currentPosition.lat !== 43.2964) {
        fetchWeather(currentPosition.lat, currentPosition.lon);
        fetchPollutants(currentPosition.lat, currentPosition.lon);
    }

    // 3. Mise à jour Horloge NTP (Synchronisation)
    syncH();
    
    // 4. Mise à jour des données physiques (qui dépendent de currentAirDensity/currentSpeedOfSound)
    // ...

    // 5. Mise à jour de l'affichage de l'altitude EKF
    if ($('ekf-altitude')) $('ekf-altitude').textContent = `${dataOrDefault(currentAltitudeEKF, 2)} m`;
    // ... autres mises à jour DOM
}

// --- FONCTION ASTRO (CRITIQUE : Affichage Lune) ---
function updateAstro(lat, lon) {
    const date = new Date();
    const times = SunCalc.getTimes(date, lat, lon);
    const moon = SunCalc.getMoonTimes(date, lat, lon);
    const moonIllumination = SunCalc.getMoonIllumination(date);
    
    // Mises à jour Soleil (similaire à vos autres éléments)
    if ($('sun-alt')) $('sun-alt').textContent = `${dataOrDefault(times.solarAltitude, 2)}°`;
    
    // CORRECTION CRITIQUE E : S'assurer que 'moon-times' reçoit les deux valeurs (Lever/Coucher Lune)
    let moonTimesText = "N/A";
    if (moon.rise && moon.set) {
        moonTimesText = `${moon.rise.toLocaleTimeString('fr-FR')} / ${moon.set.toLocaleTimeString('fr-FR')}`;
    }
    if ($('moon-times')) $('moon-times').textContent = moonTimesText;
    
    // Mise à jour Illumination
    if ($('moon-illuminated')) $('moon-illuminated').textContent = `${dataOrDefault(moonIllumination.fraction * 100, 1)} %`;
    if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(moonIllumination.phase); // Fonction à définir
}


// --- FONCTION MÉTÉO (Simplement pour ne pas bloquer le code) ---
async function fetchWeather(lat, lon) {
    try {
        const response = await fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`);
        const data = await response.json();
        if (data.tempK) {
            // Mettez à jour les variables globales (critique pour l'UKF)
            // lastP_hPa = data.pressure_hPa;
            // lastT_K = data.tempK;
            // currentAirDensity = data.air_density;
            // currentSpeedOfSound = getSpeedOfSound(data.tempK);
            if ($('weather-status')) $('weather-status').textContent = "ACTIF";
        }
    } catch (error) {
        console.warn("Échec de l'API Météo (Défaut ISA actif):", error);
        if ($('weather-status')) $('weather-status').textContent = "SYNCHRO ÉCHOUÉE (Défaut ISA)";
    }
}

// ... syncH() et fetchPollutants() similaires ...

// Fin BLOC 3/4
// =================================================================

// =================================================================
// BLOC 4/4 : Logique de Contrôle (Boutons) et Initialisation Finale
// =================================================================

function stopGPS() {
    if (gpsWatcherID !== null) {
        navigator.geolocation.clearWatch(gpsWatcherID);
        gpsWatcherID = null;
    }
    // Arrêter aussi les boucles rapides/lentes si le GPS est la seule source active
    // ... Logique d'arrêt des boucles
}

// --- Initialisation des Événements ---
window.addEventListener('load', () => {
    // 1. Gestion du bouton GPS (le point de départ unique)
    const gpsBtn = $('gps-toggle-btn');
    if (gpsBtn) {
        gpsBtn.addEventListener('click', () => {
            if (gpsWatcherID === null) {
                startGPS();
            } else {
                // Vous pouvez ajouter une logique de pause/arrêt complet ici
                stopGPS();
                gpsBtn.textContent = "▶️ MARCHE GPS";
            }
        });
    }

    // 2. Gestion des autres boutons/sliders
    // (Ex: Réinitialiser la distance, changer l'environnement, etc.)
    // ... $('reset-dist-btn').addEventListener('click', resetDistance); ...

    // 3. Démarrage initial de la partie statique
    // Initialiser l'affichage Astro/Météo même sans GPS
    if (domSlowID === null) {
        startSlowLoop();
    }
    // La carte est lancée par startGPS(), mais peut être lancée ici pour l'affichage initial
    startMap(); 
});

// Fin BLOC 4/4
// =================================================================
