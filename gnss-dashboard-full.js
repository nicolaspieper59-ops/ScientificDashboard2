// =================================================================
// BLOC 1/4 : Constantes, État Global & Filtres EKF (Core Math)
// =================================================================

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES FONDAMENTALES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;          // Vitesse de la lumière dans le vide (m/s)
const C_S_STD = 343;            // Vitesse du son standard (m/s)
const R_E_BASE = 6371000;       // Rayon terrestre moyen (m)

// --- ÉTAT GLOBAL DU SYSTÈME ET DES FILTRES ---
let kAlt = 0;               // Altitude estimée par EKF
let kSpd = 0;               // Vitesse estimée par EKF
let kLat = 0, kLon = 0;     // Position estimée par EKF
let P_KF = 0.1, P_ALT_KF = 50.0; // Covariances EKF
let C_AIR = C_L;            // Vitesse de la lumière corrigée
let C_S_DYNAMIC = C_S_STD;  // Vitesse du son dynamique
let lastTimestamp = 0;      // Dernier horodatage GPS
let watchId = null;         // ID de l'écouteur GPS
let wakeLock = null;        // Verrouillage de l'écran

// Variables de données externes (Remplies par API/Capteurs)
let lastWeatherData = null;
let groundAltitudeM = null; // Altitude du sol par API
let magneticDeclinationDeg = 0; // Déclinaison magnétique par API

const $ = (id) => document.getElementById(id); // Fonction utilitaire

// Filtre de Kalman (Vitesse)
function kalmanFilterSpeed(z_speed, dt, gps_accuracy) {
    const Q_NOISE = 0.1; 
    let R = Math.max(0.1, gps_accuracy * 0.1); // R augmente avec l'imprécision du GPS
    
    // Prédiction (on suppose une accélération nulle si pas d'IMU)
    let P_predict = P_KF + Q_NOISE * dt;
    let x_predict = kSpd;
    
    // Correction (Mise à jour par la mesure GPS)
    let K = P_predict / (P_predict + R); 
    kSpd = x_predict + K * (z_speed - x_predict);
    P_KF = (1 - K) * P_predict;

    return { speed: kSpd, P: P_KF };
}

// Filtre de Kalman (Altitude)
function kalmanFilterAlt(z_gps_alt, dt, gps_accuracy) {
    const Q_ALT = 0.5;
    let R_GPS = Math.max(5.0, gps_accuracy); 

    let P_alt_predict = P_ALT_KF + Q_ALT * dt;
    let x_alt_predict = kAlt;
    
    let K_gps = P_alt_predict / (P_alt_predict + R_GPS);
    kAlt = x_alt_predict + K_gps * (z_gps_alt - x_alt_predict);
    P_ALT_KF = (1 - K_gps) * P_alt_predict;

    return { altitude: kAlt, P_alt: P_ALT_KF };
}
// =================================================================
// BLOC 2/4 : map_handler.js (Carte Leaflet et Zoom Auto)
// =================================================================

let map = null, marker = null, polyline = null;
let isFirstPos = true;
let trackPoints = [];

function initMap(initialLat = 0, initialLon = 0) {
    if (map) map.remove();
    map = L.map('map').setView([initialLat, initialLon], 10);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19 }).addTo(map);
    marker = L.marker([initialLat, initialLon]).addTo(map);
    polyline = L.polyline([], {color: '#3498db', weight: 3}).addTo(map);
    trackPoints = [];
    isFirstPos = true;
}

// Mise à jour de la carte avec Zoom Auto
function updateMap(lat, lon, accuracy) {
    if (!map) return;

    const newLatLng = [lat, lon];
    marker.setLatLng(newLatLng);

    // Ajout du point à la trace (condition de distance pour éviter le bruit)
    if (accuracy < 50 && (trackPoints.length === 0 || L.latLng(trackPoints[trackPoints.length - 1]).distanceTo(newLatLng) > 1)) {
        trackPoints.push(newLatLng);
        polyline.setLatLngs(trackPoints);
    }

    if (isFirstPos) {
        map.setView(newLatLng, 15);
        isFirstPos = false;
    } else if (trackPoints.length > 1) {
        // Zoom automatique sur l'ensemble de la trajectoire
        const bounds = polyline.getBounds();
        if (bounds.isValid()) {
            map.fitBounds(bounds, { padding: [10, 10], maxZoom: 17 });
        }
    } else {
        // Centrage simple
        map.setView(newLatLng, map.getZoom());
    }
                    }
// =================================================================
// BLOC 3/4 : physics_astro_api.js (Physique Avancée, Astro & API)
// =================================================================

// --- CORRECTION VITESSE DE LA LUMIÈRE C_air (Indice de Réfraction) ---
// Formule basée sur Pression et Température
function calculateRefractiveIndex(P_Pa, T_K) {
    const P_hPa = P_Pa / 100.0;
    // n = 1 + N * 10^-6 (N est la refractivité)
    const N_dry = 77.6 * P_hPa / T_K; 
    return 1.0 + (N_dry * 1e-6); 
}

// Calcule le facteur de Lorentz (gamma) - utilise C_AIR global
function calculateLorentzFactor(speed) {
    if (speed >= C_AIR) return Infinity; 
    return 1.0 / Math.sqrt(1.0 - (speed * speed) / (C_AIR * C_AIR));
}

// Calcule la vitesse du son (C_S) dynamique (basé sur la température)
function calculateSpeedOfSound(tempC) {
    // Cs = 331.4 + 0.6 * T (en mètres par seconde)
    return 331.4 + 0.6 * tempC;
}

// --- INTEGRATION API EXTERNE (PLACEHOLDERS) ---

// PLACEHOLDER: Récupère l'altitude du sol (Alt. DEM)
async function getGroundAltitudeAPI(lat, lon) {
    // EN PRODUCTION: Remplacez ceci par un VRAI appel API (ex: Google Elevation)
    console.warn(`[API] Appel d'altitude du sol requis pour ${lat.toFixed(2)}, ${lon.toFixed(2)}. (NON IMPLÉMENTÉ)`);
    // Retourne une valeur par défaut pour permettre au reste du code de fonctionner
    return 50.0; 
}

// PLACEHOLDER: Récupère la Déclinaison Magnétique
async function getMagneticDeclinationAPI(lat, lon, date) {
    // EN PRODUCTION: Remplacez ceci par un VRAI appel API WMM (World Magnetic Model)
    console.warn(`[API] Appel de déclinaison magnétique requis. (NON IMPLÉMENTÉ)`);
    // Retourne une valeur par défaut
    return 1.0; 
}

// PLACEHOLDER: Récupère les données météo
async function getWeatherData(lat, lon) {
    // EN PRODUCTION: Remplacez ceci par un VRAI appel API Météo (ex: OpenWeatherMap, Meteo France)
    console.warn(`[API] Appel météo requis pour les corrections C/CS. (NON IMPLÉMENTÉ)`);
    
    // Retourne des données par défaut pour les corrections C/CS et Météo
    return {
        tempC: 15.0,
        tempK: 288.15,
        pressure_hPa: 1013.25,
        humidity_perc: 60,
        speedOfSound: 340.4
    };
            }
// =================================================================
// BLOC 4/4 : main_logic.js (Acquisition GPS, DOM & API)
// =================================================================

const DOM_SLOW_UPDATE_MS = 2000;

// --- GESTION DU WAKE LOCK (Écran allumé) ---
async function requestWakeLock() {
    if ('wakeLock' in navigator) {
        try {
            wakeLock = await navigator.wakeLock.request('screen');
            console.log('Wake Lock acquis. Écran forcé allumé.');
        } catch (err) {
            console.error('Échec de l\'acquisition du Wake Lock.');
        }
    }
}

function releaseWakeLock() {
    if (wakeLock) {
        wakeLock.release();
        wakeLock = null;
        console.log('Wake Lock relâché.');
    }
}

// --- ACQUISITION GPS EN TEMPS RÉEL (NON SIMULÉE) ---

function onGpsSuccess(position) {
    const now = new Date(position.timestamp);
    const coords = position.coords;

    const lat = coords.latitude;
    const lon = coords.longitude;
    const gpsAlt = coords.altitude || kAlt; 
    const gpsSpd = coords.speed || 0;
    const gpsAccuracy = coords.accuracy;

    // Calcul du Delta Time
    const dt = lastTimestamp ? (position.timestamp - lastTimestamp) / 1000 : 0;
    lastTimestamp = position.timestamp;

    // Mise à jour EKF
    const filteredSpeedData = kalmanFilterSpeed(gpsSpd, dt, gpsAccuracy);
    const filteredAltData = kalmanFilterAlt(gpsAlt, dt, gpsAccuracy); 

    kLat = lat; kLon = lon;
    
    updateMap(lat, lon, gpsAccuracy);
    updateFastData(lat, lon, filteredSpeedData.speed, filteredAltData.altitude, gpsAccuracy, now, dt);
}

function onGpsError(error) {
    console.error(`Erreur GPS (${error.code}): ${error.message}`);
    if ($('gps-status-dr')) $('gps-status-dr').textContent = `Erreur GPS ${error.code}`;
}

// --- MISE À JOUR RAPIDE DU DOM ---

function updateFastData(lat, lon, kSpdMS, kAltM, gpsAccuracy, date, dt) {
    const objectMass = parseFloat($('object-mass').value) || 70;
    const kSpdKMH = kSpdMS * 3.6;

    // 1. CORRECTION MÉTÉO DE C et CS
    if (lastWeatherData) {
        const n_air = calculateRefractiveIndex(lastWeatherData.pressure_hPa * 100, lastWeatherData.tempK);
        C_AIR = C_L / n_air;
        C_S_DYNAMIC = calculateSpeedOfSound(lastWeatherData.tempC);
        
        if ($('refraction-factor')) $('refraction-factor').textContent = n_air.toFixed(6);
        if ($('speed-of-sound')) $('speed-of-sound').textContent = C_S_DYNAMIC.toFixed(1) + ' m/s';
    }

    // 2. Calculs Avancés
    const lorentz = calculateLorentzFactor(kSpdMS);
    const verticalDistance = groundAltitudeM !== null ? kAltM - groundAltitudeM : null;

    // 3. Mise à jour du DOM
    if ($('gps-accuracy')) $('gps-accuracy').textContent = gpsAccuracy.toFixed(1) + ' m';
    if ($('altitude-display')) $('altitude-display').textContent = kAltM.toFixed(1) + ' m';
    if ($('speed-display')) $('speed-display').textContent = kSpdKMH.toFixed(1) + ' km/h';
    if ($('light-speed-perc')) $('light-speed-perc').textContent = ((kSpdMS / C_AIR) * 100).toPrecision(2) + ' %';
    if ($('mach-number')) $('mach-number').textContent = (kSpdMS / C_S_DYNAMIC).toFixed(4);
    if ($('lorentz-factor')) $('lorentz-factor').textContent = lorentz.toFixed(4);
    if ($('kalman-uncert')) $('kalman-uncert').textContent = P_KF.toFixed(2);
    if ($('alt-uncertainty')) $('alt-uncertainty').textContent = P_ALT_KF.toFixed(2) + ' m';

    // Mise à jour Distance Verticale Sol/Ciel
    if (verticalDistance !== null) {
        let altStatus = verticalDistance >= 0 ? 'Ciel' : 'Sous Sol';
        if ($('vertical-distance-ground')) $('vertical-distance-ground').textContent = `${Math.abs(verticalDistance).toFixed(1)} m (${altStatus})`;
    } else {
        if ($('vertical-distance-ground')) $('vertical-distance-ground').textContent = `N/A (Alt Sol Req.)`;
    }
    
    // Mise à jour Déclinaison
    if ($('mag-declination')) $('mag-declination').textContent = magneticDeclinationDeg.toFixed(2) + ' °'; 
    if ($('heading-display')) $('heading-display').textContent = coords.heading ? coords.heading.toFixed(0) + ' ° Vrai' : 'N/A';
}

// --- GESTION DU MONITORING ---

async function startMonitoring() {
    if (watchId) {
        console.warn("Monitoring déjà actif.");
        return;
    }

    requestWakeLock();
    initMap(kLat, kLon);

    const options = {
        enableHighAccuracy: true,
        timeout: 5000,
        maximumAge: 0
    };
    watchId = navigator.geolocation.watchPosition(onGpsSuccess, onGpsError, options);
    
    // Boucle de mise à jour lente (API Météo, Altitude, Magnétisme)
    setInterval(async () => {
        if (kLat !== 0 && kLon !== 0) {
            // Appels API externes (placeholders)
            lastWeatherData = await getWeatherData(kLat, kLon);
            groundAltitudeM = await getGroundAltitudeAPI(kLat, kLon);
            magneticDeclinationDeg = await getMagneticDeclinationAPI(kLat, kLon, new Date());
        }
        
        if ($('local-time')) $('local-time').textContent = new Date().toLocaleTimeString('fr-FR');
        if ($('date-display')) $('date-display').textContent = new Date().toLocaleDateString('fr-FR');

    }, DOM_SLOW_UPDATE_MS); 
    
    console.log("Monitoring GPS en temps réel démarré.");
}

function stopMonitoring() {
    if (watchId) {
        navigator.geolocation.clearWatch(watchId);
        watchId = null;
        releaseWakeLock();
        // Réinitialisation simple de la carte et de l'état
        if ($('gps-status-dr')) $('gps-status-dr').textContent = 'Arrêté';
        console.log("Monitoring GPS arrêté.");
    }
}

// --- CAPTURE DE DONNÉES ---

function captureData() {
    const data = {
        timestamp: new Date().toISOString(),
        latitude: kLat,
        longitude: kLon,
        altitude_ekf: kAlt,
        speed_ekf: kSpd,
        speed_kmh: kSpd * 3.6,
        lorentz_factor: calculateLorentzFactor(kSpd),
        C_air: C_AIR,
        C_sound: C_S_DYNAMIC,
        ground_alt: groundAltitudeM,
        vertical_distance: kAlt - groundAltitudeM,
        mag_declination: magneticDeclinationDeg,
        weather: lastWeatherData
    };
    const jsonStr = JSON.stringify(data, null, 2);
    
    // Création du fichier JSON téléchargeable
    const blob = new Blob([jsonStr], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `gnss_data_capture_${Date.now()}.json`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
    
    console.log("Données capturées et exportées au format JSON.");
}

// Démarrage de l'application
document.addEventListener('DOMContentLoaded', () => {
    initMap(0, 0); 

    // Écouteurs de boutons
    $('start-button').addEventListener('click', startMonitoring);
    $('stop-button').addEventListener('click', stopMonitoring);
    $('capture-button').addEventListener('click', captureData);
});
