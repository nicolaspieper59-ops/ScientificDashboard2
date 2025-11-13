// =================================================================
// BLOC 1/4 : Constantes, État Global & Filtres EKF (Core Math)
// =================================================================

// --- CLÉS D'API & PROXY ---
// Ces URLs sont des placeholders. Elles doivent être remplacées par des endpoints fonctionnels.
const PROXY_BASE_URL = "https://your-api-proxy.com"; 
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const PROXY_ELEVATION_ENDPOINT = `${PROXY_BASE_URL}/api/elevation`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES FONDAMENTALES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;          // Vitesse de la lumière (m/s)
const C_S_STD = 343;            // Vitesse du son standard (m/s)
const G_U = 6.67430e-11;        // Constante gravitationnelle universelle (N·m²/kg²)
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const R_E_BASE = 6371000;       // Rayon terrestre moyen (m)
const KMH_MS = 3.6;             // Conversion m/s vers km/h

// --- ÉTAT GLOBAL DU SYSTÈME ET DES FILTRES ---
let kAlt = 0;               // Altitude estimée par EKF
let kSpd = 0;               // Vitesse estimée par EKF
let kLat = 0, kLon = 0;     // Position estimée par EKF
let P_KF = 0.1, P_ALT_KF = 50.0; // Covariances EKF
let lastTimestamp = 0;      // Dernier horodatage GPS
let watchId = null;         // ID de l'écouteur GPS
let wakeLock = null;        // Verrouillage de l'écran

// Données accumulées
let totalDistance = 0;      // Distance 3D totale parcourue
let totalTime = 0;          // Temps total écoulé
let movementTime = 0;       // Temps de mouvement (vitesse > min)
let maxSpeed = 0;
let isPaused = false;

// Variables de données externes / capteurs
let lastWeatherData = null;
let groundAltitudeM = null; // Altitude du sol par API
let baroAltitude = null;    // Altitude corrigée par Baromètre
let magneticDeclinationDeg = 0; // Déclinaison magnétique par API
let imuAccel = { x: 0, y: 0, z: 0 }; // Données IMU
let maxNoise = 0, avgNoise = 0; // Compteur sonore

const $ = (id) => document.getElementById(id); // Fonction utilitaire

// Filtre de Kalman (Vitesse)
function kalmanFilterSpeed(z_speed, dt, gps_accuracy, acc_x, acc_y) {
    const Q_NOISE = 0.1;
    let R = Math.max(0.1, gps_accuracy * 0.1); 
    const accel_mag = Math.sqrt(acc_x * acc_x + acc_y * acc_y);

    let P_predict = P_KF + Q_NOISE * dt;
    let x_predict = kSpd + accel_mag * dt; // Utilise l'accélération IMU pour la prédiction
    
    let K = P_predict / (P_predict + R); 
    kSpd = x_predict + K * (z_speed - x_predict);
    P_KF = (1 - K) * P_predict;

    return { speed: kSpd, P: P_KF };
}

// Filtre de Kalman (Altitude - Fusion GPS/Baro)
function kalmanFilterAlt(z_gps_alt, z_baro_alt, dt, gps_accuracy) {
    const Q_ALT = 0.5;
    let R_GPS = Math.max(5.0, gps_accuracy);
    const R_BARO = 5.0; // Précision relative du baromètre (généralement bonne)

    let P_alt_predict = P_ALT_KF + Q_ALT * dt;
    let x_alt_predict = kAlt;
    
    // Correction 1: GPS (altitude MSL)
    let K_gps = P_alt_predict / (P_alt_predict + R_GPS);
    let alt_temp = x_alt_predict + K_gps * (z_gps_alt - x_alt_predict);
    let P_temp = (1 - K_gps) * P_alt_predict;

    // Correction 2: Baromètre (altitude relative)
    if (z_baro_alt !== null) {
        let K_baro = P_temp / (P_temp + R_BARO);
        kAlt = alt_temp + K_baro * (z_baro_alt - alt_temp);
        P_ALT_KF = (1 - K_baro) * P_temp;
    } else {
        kAlt = alt_temp;
        P_ALT_KF = P_temp;
    }

    return { altitude: kAlt, P_alt: P_ALT_KF };
                                 }
// =================================================================
// BLOC 2/4 : map_handler & Astro/Visu Logic
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
    if (!map || isPaused) return;

    const newLatLng = [lat, lon];
    marker.setLatLng(newLatLng);

    // Ajout du point si la précision est bonne (< 50m) et distance minimale
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
        map.setView(newLatLng, map.getZoom());
    }
}

// --- LOGIQUE ASTRO AVANCÉE (PLACEHOLDERS) ---

function updateAstroVisualizer(lat, lon, date, sunAlt, moonAlt, moonPhasePerc) {
    const sunDisk = $('sun-disk');
    const moonDisk = $('moon-disk');
    const container = $('astro-visualizer');

    if (!sunDisk || !container) return;

    // L'altitude est convertie en une position verticale (0% = Horizon, 100% = Zénith)
    // 90° = Zénith (top du disque), 0° = Horizon (milieu de la div)
    const containerHeight = container.offsetHeight;
    const center = containerHeight / 2;
    const maxMovement = containerHeight / 2; // Mouvement max de 0° à 90°

    // Position verticale (0 = Horizon, 1 = Zénith)
    const sunPosNorm = Math.min(1, Math.max(-1, sunAlt / 90));
    const moonPosNorm = Math.min(1, Math.max(-1, moonAlt / 90));

    // Y position dans le conteneur (Y = 0 est en haut)
    const sunY = center - (sunPosNorm * maxMovement);
    const moonY = center - (moonPosNorm * maxMovement);

    // X position (simplifié pour l'exemple, devrait dépendre de l'azimut)
    const sunX = (date.getHours() % 24) * 100 / 24; // Simple cycle quotidien
    const moonX = ((date.getHours() + 12) % 24) * 100 / 24;

    sunDisk.style.top = `${sunY}px`;
    sunDisk.style.left = `${sunX}%`;
    sunDisk.style.transform = `translate(-50%, -50%)`;

    moonDisk.style.top = `${moonY}px`;
    moonDisk.style.left = `${moonX}%`;
    moonDisk.style.transform = `translate(-50%, -50%)`;

    // Visibilité / Opacité
    sunDisk.style.opacity = sunAlt > -10 ? 1 : 0.2;
    moonDisk.style.opacity = moonAlt > -10 ? 1 : 0.2;
    
    // LOGIQUE MC (partie inférieure transparente/biome)
    const biomeColor = lastWeatherData && lastWeatherData.biomeColor ? lastWeatherData.biomeColor : '#3498db'; // Exemple de couleur de biome
    document.documentElement.style.setProperty('--biome-color', biomeColor);
}

// PLACEHOLDER: Globe Interactif 3D
function initGlobe() {
    // EN PRODUCTION : Initialisation de Three.js, WebGL ou Cesium ici.
    console.log("Initialisation du Globe 3D Interactive...");
                         }
// =================================================================
// BLOC 3/4 : API & Sensor Handlers
// =================================================================

// --- GESTION DU WAKE LOCK (Écran allumé) ---
async function requestWakeLock() {
    if ('wakeLock' in navigator) {
        try {
            wakeLock = await navigator.wakeLock.request('screen');
            console.log('Wake Lock acquis.');
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

// --- LOGIQUE CAPTEURS IMU/MAGNÉTOMÈTRE (NON SIMULÉE) ---

function initSensorHandlers() {
    if (window.DeviceOrientationEvent) {
        window.addEventListener('deviceorientation', (event) => {
            // Placeholder pour la fusion des données d'orientation et de magnétisme
            // Cap (Direction) doit être corrigé par la Déclinaison Magnétique
        });
    }

    if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', (event) => {
            if (event.accelerationIncludingGravity) {
                imuAccel.x = event.accelerationIncludingGravity.x;
                imuAccel.y = event.accelerationIncludingGravity.y;
                imuAccel.z = event.accelerationIncludingGravity.z;
                $('accel-x').textContent = imuAccel.x.toFixed(2) + ' m/s²';
                $('accel-y').textContent = imuAccel.y.toFixed(2) + ' m/s²';
                $('accel-z').textContent = imuAccel.z.toFixed(2) + ' m/s²';
                $('sensor-status-imu').textContent = 'Actif';
            }
        });
    }
}

// --- LOGIQUE API EXTERNE (PLACEHOLDERS) ---

// PLACEHOLDER: Récupère l'altitude du sol (Alt. DEM)
async function getGroundAltitudeAPI(lat, lon) {
    // DOIT être un VRAI appel API
    console.warn(`[API] Appel d'altitude du sol requis.`);
    // Placeholder : 100m
    return 100.0; 
}

// PLACEHOLDER: Récupère la Déclinaison Magnétique
async function getMagneticDeclinationAPI(lat, lon, date) {
    // DOIT être un VRAI appel API WMM
    console.warn(`[API] Appel de déclinaison magnétique requis.`);
    // Placeholder : 1 degré
    return 1.0; 
}

// PLACEHOLDER: Récupère les données météo (T, P, H) et altitude barométrique
async function getWeatherData(lat, lon, altitudeMSL) {
    // DOIT être un VRAI appel API Météo (station la plus proche)
    console.warn(`[API] Appel météo/barométrique requis.`);
    
    // Placeholder de données réalistes
    const tempC = 17.3;
    const pressure_hPa = 1021.0;
    const humidity_perc = 81;
    const air_density = 1.225;
    const dew_point = 14.0;
    
    // Calcul de l'altitude barométrique corrigée (pour la fusion EKF)
    // Pression au niveau de la mer (QNH) est corrigée par la pression API
    const alt_baro_corrected = altitudeMSL + (1013.25 - pressure_hPa) * 8.3; // Formule simplifiée

    return {
        tempC, tempK: tempC + 273.15, pressure_hPa, humidity_perc, 
        air_density, dew_point, alt_baro: alt_baro_corrected,
        speedOfSound: 331.4 + 0.6 * tempC,
        biomeColor: '#27ae60' // Couleur de biome pour l'animation MC
    };
}
// =================================================================
// BLOC 4/4 : Main Logic & DOM
// =================================================================

const DOM_SLOW_UPDATE_MS = 2000;
const MIN_SPD_FOR_MOVEMENT = 0.5; // m/s

// Fonction de succès de l'acquisition GPS (NON SIMULÉE)
function onGpsSuccess(position) {
    if (isPaused) return;

    const coords = position.coords;

    const lat = coords.latitude;
    const lon = coords.longitude;
    // Altitude EKF initialisée par GPS Alt (MSL)
    const gpsAlt = coords.altitude || kAlt; 
    const gpsSpd = coords.speed || 0; // Vitesse Brute (m/s)
    const gpsAccuracy = coords.accuracy;

    const now = new Date(position.timestamp);

    // Calcul du Delta Time
    const dt = lastTimestamp ? (position.timestamp - lastTimestamp) / 1000 : 0.01;
    lastTimestamp = position.timestamp;

    // Mise à jour EKF (utilise l'accélération IMU si disponible)
    const filteredSpeedData = kalmanFilterSpeed(gpsSpd, dt, gpsAccuracy, imuAccel.x, imuAccel.y);
    // Utilise l'altitude barométrique corrigée si disponible, sinon seulement GPS
    const baro_alt_input = lastWeatherData ? lastWeatherData.alt_baro : null;
    const filteredAltData = kalmanFilterAlt(gpsAlt, baro_alt_input, dt, gpsAccuracy); 

    kLat = lat; kLon = lon;
    
    // Accumulation des statistiques
    if (kSpd > MIN_SPD_FOR_MOVEMENT) {
        totalDistance += kSpd * dt;
        movementTime += dt;
    }
    totalTime += dt;
    maxSpeed = Math.max(maxSpeed, filteredSpeedData.speed);

    updateMap(lat, lon, gpsAccuracy);
    updateFastData(lat, lon, filteredSpeedData.speed, filteredAltData.altitude, gpsSpd, gpsAccuracy, now, dt);
}

function onGpsError(error) {
    console.error(`Erreur GPS (${error.code}): ${error.message}`);
    if ($('gps-status-dr')) $('gps-status-dr').textContent = `Erreur GPS ${error.code}`;
}

// --- MISE À JOUR RAPIDE DU DOM ---

function updateFastData(lat, lon, kSpdMS, gpsSpdBrute, kAltM, gpsAccuracy, date, dt) {
    const objectMass = parseFloat($('object-mass').value) || 70;
    const kSpdKMH = kSpdMS * KMH_MS;

    // 1. CORRECTION C/CS (utilisée par la physique)
    const C_AIR = lastWeatherData ? C_L / calculateRefractiveIndex(lastWeatherData.pressure_hPa * 100, lastWeatherData.tempK) : C_L;
    const C_S_DYNAMIC = lastWeatherData ? lastWeatherData.speedOfSound : C_S_STD;

    // 2. Calculs Relativité / Physique
    const lorentz = calculateLorentzFactor(kSpdMS, C_AIR);
    const timeDilatKin = (lorentz - 1) * 86400 * 1e9; // ns/jour
    const timeDilatGrav = kAltM * 100 * 86400 * 1e9 / (C_L * C_L); // Simplifié ns/jour
    const E = objectMass * C_L * C_L * lorentz;
    const q = 0.5 * (lastWeatherData ? lastWeatherData.air_density : 1.225) * kSpdMS * kSpdMS;
    
    // 3. LOGIQUE NETHER (Rapport de Distance Centre Terre)
    const radiusNow = R_E_BASE + kAltM;
    const distanceRatioMRF = radiusNow / R_E_BASE;

    // 4. Distance Verticale Sol/Ciel
    const verticalDistance = (groundAltitudeM !== null) ? kAltM - groundAltitudeM : null;

    // 5. DOM
    $('speed-display').textContent = kSpdKMH.toFixed(1) + ' km/h';
    $('speed-ms-display').textContent = kSpdMS.toFixed(2) + ' m/s';
    $('speed-ks-display').textContent = (kSpdMS / 1000).toFixed(4) + ' km/s';
    $('speed-brute-ms').textContent = gpsSpdBrute.toFixed(2) + ' m/s';
    $('speed-max').textContent = (maxSpeed * KMH_MS).toFixed(1) + ' km/h';
    $('speed-avg-mvt').textContent = movementTime > 0 ? ((totalDistance / movementTime) * KMH_MS).toFixed(1) + ' km/h' : '0.0 km/h';
    $('speed-avg-total').textContent = totalTime > 0 ? ((totalDistance / totalTime) * KMH_MS).toFixed(1) + ' km/h' : '0.0 km/h';

    $('sound-speed-perc').textContent = ((kSpdMS / C_S_DYNAMIC) * 100).toFixed(2) + ' %';
    $('mach-number').textContent = (kSpdMS / C_S_DYNAMIC).toFixed(4);
    $('light-speed-perc').textContent = ((kSpdMS / C_AIR) * 100).toPrecision(2) + ' %';
    $('lorentz-factor').textContent = lorentz.toFixed(4);
    
    $('total-distance').textContent = `${(totalDistance / 1000).toFixed(3)} km | ${totalDistance.toFixed(2)} m`;
    $('distance-ratio-mrf').textContent = distanceRatioMRF.toFixed(3);
    $('dist-light-s').textContent = (totalDistance / C_L).toPrecision(2) + ' s';
    // Les autres distances lumières/UA nécessitent des constantes trop longues.
    
    $('latitude-display').textContent = lat.toFixed(6) + ' °';
    $('longitude-display').textContent = lon.toFixed(6) + ' °';
    $('altitude-display').textContent = kAltM.toFixed(1) + ' m';
    
    // Altitude Sol/Ciel
    if (verticalDistance !== null) {
        let altStatus = verticalDistance >= 0 ? 'Ciel' : 'Sous Sol';
        $('vertical-distance-ground').textContent = `${Math.abs(verticalDistance).toFixed(1)} m (${altStatus})`;
    } else {
        $('vertical-distance-ground').textContent = `N/A (Alt Sol Req.)`;
    }

    $('speed-of-sound').textContent = C_S_DYNAMIC.toFixed(1) + ' m/s';
    $('time-dilat-kin').textContent = timeDilatKin.toFixed(2) + ' ns/j';
    $('time-dilat-grav').textContent = timeDilatGrav.toFixed(2) + ' ns/j';
    $('relativistic-energy').textContent = E.toPrecision(3) + ' J';
    $('dynamic-pressure').textContent = q.toFixed(2) + ' Pa';

    // Mise à jour de l'animation Astro
    // PLACEHOLDER: sunAlt/moonAlt doivent être calculés par une librairie Astro
    updateAstroVisualizer(lat, lon, date, 45, -10, 50); 
}

// --- GESTION DU MONITORING ET DES BOUTONS ---

async function startMonitoring() {
    if (watchId) {
        console.warn("Monitoring déjà actif.");
        return;
    }

    requestWakeLock();
    initMap(kLat, kLon);
    initGlobe(); // Placeholder 3D
    initSensorHandlers(); // Capteurs IMU/Mag

    const options = { enableHighAccuracy: true, timeout: 5000, maximumAge: 0 };
    watchId = navigator.geolocation.watchPosition(onGpsSuccess, onGpsError, options);
    
    // Boucle de mise à jour lente (API et Astro)
    setInterval(async () => {
        if (kLat !== 0 && kLon !== 0 && !isPaused) {
            // Appels API externes (placeholders)
            const altMSL = $('altitude-display').textContent.replace(' m', '');
            lastWeatherData = await getWeatherData(kLat, kLon, parseFloat(altMSL));
            groundAltitudeM = await getGroundAltitudeAPI(kLat, kLon);
            magneticDeclinationDeg = await getMagneticDeclinationAPI(kLat, kLon, new Date());
            
            // PLACEHOLDER ASTRO AVANCÉ
            $('true-solar-time').textContent = '12:00:00';
            $('mean-solar-time').textContent = '12:00:00';
            $('solar-noon-utc').textContent = '11:58:34 UTC';
            $('eot-value').textContent = '+15.93 min';
        }
        
        // Mise à jour de l'heure
        const now = new Date();
        $('local-time').textContent = now.toLocaleTimeString('fr-FR');
        $('date-display').textContent = now.toLocaleDateString('fr-FR');
        $('session-time').textContent = totalTime.toFixed(2) + ' s';
        $('movement-time').textContent = movementTime.toFixed(2) + ' s';
        
    }, DOM_SLOW_UPDATE_MS); 
    
    $('gps-status-dr').textContent = 'Actif';
    console.log("Monitoring GPS en temps réel démarré.");
}

function stopMonitoring() {
    if (watchId) {
        navigator.geolocation.clearWatch(watchId);
        watchId = null;
        releaseWakeLock();
        $('gps-status-dr').textContent = 'Arrêté';
        console.log("Monitoring GPS arrêté.");
    }
}

function captureData() {
    // ... (Logique de capture de données JSON complète) ...
    const data = { /* ... */ };
    const jsonStr = JSON.stringify(data, null, 2);
    
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
    initGlobe();
    
    // Écouteurs de boutons
    $('start-button').addEventListener('click', startMonitoring);
    $('stop-button').addEventListener('click', stopMonitoring);
    $('capture-button').addEventListener('click', captureData);
    $('pause-button').addEventListener('click', () => {
        isPaused = !isPaused;
        $('pause-button').textContent = isPaused ? '▶️ REPRENDRE GPS' : '⏸️ PAUSE GPS';
        $('gps-status-dr').textContent = isPaused ? 'PAUSE' : 'Actif';
    });
    $('night-mode').addEventListener('click', () => document.body.classList.toggle('night-mode'));
    // Les autres boutons (Reset, Emergency Stop, etc.) nécessitent une implémentation de leur logique de réinitialisation/état.
});
