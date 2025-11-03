// =================================================================
// FICHIER JS PARTIE 1 : gnss-dashboard-part1.js (Initialisation & État Global)
// =================================================================

// --- 1. FONCTION UTILITAIRE DE RÉFÉRENCE ---
/** Shorthand pour document.getElementById */
function $(id) {
    return document.getElementById(id);
}

// --- 2. CONSTANTES DE SYSTÈME ET PHYSIQUE ---
const KMH_MS = 3.6;          // 1 m/s = 3.6 km/h
const C_L = 299792458;       // Vitesse de la lumière (m/s)
const G_ACC = 9.80665;       // Force de gravité (m/s²)
const W_EARTH = 7.2921e-5;   // Vitesse angulaire de la Terre (rad/s)
const NETHER_RATIO = 8;      // Ratio Minecraft Nether (1 bloc du monde réel = 8 dans le Nether)
const MIN_SPD = 0.1;         // Seuil minimal de vitesse pour considérer un mouvement (m/s)
const MIN_DT = 0.05;         // Intervalle de temps minimal pour les calculs (secondes)
const ACCEL_FILTER_ALPHA = 0.5; // Coefficient pour le filtre IIR d'accélération
const ACCEL_MOVEMENT_THRESHOLD = 0.5; // Seuil pour détecter le mouvement (m/s²)
const MSL_CORRECTION = 48.0; // Correction du géoïde (Niveau Moyen de la Mer) pour l'altitude (Marseille, exemple statique)

// Constantes GNSS
const DEFAULT_LAT = 43.2965;
const DEFAULT_LON = 5.3698;
const MAX_ACC = 50;          // Précision max acceptable (m)

// Synchronisation et Météo
const OWM_API_URL = "https://api.openweathermap.org/data/2.5/weather";
const OWM_API_KEY = "VOTRE_CLE_API_OPENWEATHERMAP"; // <-- REMPLACER PAR VOTRE CLÉ RÉELLE
const SERVER_TIME_ENDPOINT = "http://worldtimeapi.org/api/timezone/Etc/UTC";
const WEATHER_SYNC_INTERVAL_LOW = 300000; // 5 minutes en basse fréquence (ms)

// Constantes Astro (pour getSolarTime)
const dayMs = 86400000;
const J1970 = 2440588; // Jour Julien pour le 1er jan 1970
const J2000 = 2451545; // Jour Julien pour J2000
const D2R = Math.PI / 180; // Degrés en radians
const R2D = 180 / Math.PI; // Radians en degrés

const GPS_OPTS = {
    // Options de l'API Geolocation
    HIGH_FREQ: { enableHighAccuracy: true, timeout: 5000, maximumAge: 0 },
    LOW_FREQ: { enableHighAccuracy: false, timeout: 10000, maximumAge: 60000 }
};

// --- 3. VARIABLES D'ÉTAT GLOBALES ---
let wID = null;             // ID du watchPosition
let lPos = null;            // Dernière position GPS enregistrée
let sTime = null;           // Heure de début de session (timestamp)
let lat = DEFAULT_LAT;      // Latitude courante
let lon = DEFAULT_LON;      // Longitude courante
let maxSpd = 0;             // Vitesse maximale (m/s)
let distM = 0;              // Distance totale parcourue (m)
let timeMoving = 0;         // Temps passé en mouvement (secondes)
let lastFSpeed = 0;         // Vitesse filtrée précédente (m/s)
let lastP_hPa = 1013.25;    // Dernière pression atmosphérique (hPa)
let currentGPSMode = 'HIGH_FREQ'; 
let netherMode = false;     // État du mode Nether
let nightMode = false;      // État du mode Nuit

// Variables de synchronisation horaire
let lServH = 0;             // Heure du serveur UTC (ms)
let lLocH = 0;              // performance.now() lors de la synchro

// Variables d'arrêt d'urgence et de Wake Lock
let emergencyStopActive = false;
let wakeLock = null;

// Variables pour le filtre IIR
let iirSpdRaw = 0;
let iirSpd3D = 0;

// Variables pour l'accéléromètre
let latestVerticalAccelIMU = 0.0;
let latestLinearAccelMagnitude = 0.0;
const G_STATIC_REF = { x: 0, y: 0, z: G_ACC };
let kAccel = { x: 0, y: 0, z: G_ACC }; // Accélération filtrée IIR

// Variables pour le Filtre de Kalman (Vitesse)
let kFilterState = { speed: 0, uncertainty: 10 }; 

// Variables pour le Filtre de Kalman (Altitude)
let kAltState = { altitude: 0, uncertainty: 50 }; 
let kAltUncert = 50; 

// Variables pour la Batterie
let batteryManager = null;

// Variables Leaflet
let map = null;
let marker = null;
let tracePolyline = null;
let lastWeatherUpdate = 0;

// --- 4. UTILITAIRES MATHÉMATIQUES ---
/** Formate un nombre avec une précision donnée */
function formatNumber(num, precision) {
    return (num || 0).toFixed(precision);
}

/** Calcule la distance entre deux points GPS (formule de Haversine simplifiée) */
function dist(lat1, lon1, lat2, lon2) {
    const R = 6371000; // Rayon de la Terre en mètres
    const dLat = (lat2 - lat1) * D2R;
    const dLon = (lon2 - lon1) * D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1 * D2R) * Math.cos(lat2 * D2R) * Math.sin(dLon / 2) ** 2;
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R * c;
}

/** Calcule le point de rosée à partir de la température et de l'humidité */
function calculateDewPoint(tempC, humidity) {
    // Formule Magnus (approximation)
    const a = 17.27;
    const b = 237.7;
    const alpha = (a * tempC) / (b + tempC) + Math.log(humidity / 100);
    return (b * alpha) / (a - alpha);
}

// --- 5. GESTIONNAIRES D'ÉVÉNEMENTS (RÉINITIALISATION & MODES) ---

function resetAll() {
    distM = 0; timeMoving = 0; maxSpd = 0; lastFSpeed = 0; sTime = null;
    kFilterState = { speed: 0, uncertainty: 10 };
    kAltState = { altitude: 0, uncertainty: 50 };
    iirSpdRaw = 0; iirSpd3D = 0;
    
    // Réinitialisation de l'UI (Distance/Vitesse Max)
    $('distance-total-km').textContent = '0.000 km | 0.00 m';
    $('time-moving').textContent = '0.00 s';
    $('speed-max').textContent = '0.00000 km/h';
    $('speed-avg-moving').textContent = '0.00000 km/h';
    
    // Réinitialisation du tracé de la carte
    if (tracePolyline) {
        tracePolyline.setLatLngs([]);
    }
    console.log('Système entièrement réinitialisé.');
}

function resetDistance() {
    distM = 0; timeMoving = 0;
    $('distance-total-km').textContent = '0.000 km | 0.00 m';
    $('time-moving').textContent = '0.00 s';
    console.log('Distance et Temps réinitialisés.');
}

function resetMaxSpeed() {
    maxSpd = 0;
    $('speed-max').textContent = '0.00000 km/h';
    console.log('Vitesse Max réinitialisée.');
}

function toggleNetherMode() {
    netherMode = !netherMode;
    const ratio = netherMode ? ' (1:8)' : ' (1:1)';
    const status = netherMode ? 'ACTIVÉ' : 'DÉSACTIVÉ';
    $('mode-nether').textContent = `Mode Nether ${status}${ratio}`;
    console.log(`Mode Nether: ${status}`);
}

function toggleNightMode() {
    nightMode = !nightMode;
    document.body.classList.toggle('night-mode', nightMode);
    console.log(`Mode Nuit: ${nightMode ? 'ON' : 'OFF'}`);
    }
// =================================================================
// FICHIER JS PARTIE 2 : gnss-dashboard-part2.js (Logique d'Exécution)
// Dépend de gnss-dashboard-part1.js
// =================================================================

// --- 1. FONCTIONS DE FILTRAGE (SIMULATION EKF) ---

/** Filtre Passe-Bas IIR (utilisé pour les données brutes) */
function iirFilter(current, previous, alpha = 0.8) {
    if (previous === 0) return current; // Évite un lissage initial trop lent
    return alpha * current + (1 - alpha) * previous;
}

/** Filtre de Kalman pour la Vitesse 3D (EKF simplifié) */
function kFilter(measuredSpeed, dt, measurementUncertainty) {
    // Phase de prédiction
    const Q = 0.1; // Bruit du modèle (ajusté pour des mouvements rapides)
    kFilterState.uncertainty += Q * dt;

    // Phase de correction
    const R = measurementUncertainty; 
    const K = kFilterState.uncertainty / (kFilterState.uncertainty + R); // Gain de Kalman

    kFilterState.speed += K * (measuredSpeed - kFilterState.speed);
    kFilterState.uncertainty = (1 - K) * kFilterState.uncertainty;

    return kFilterState.speed;
}

/** Filtre de Kalman pour l'Altitude */
function kFilterAltitude(measuredAlt, acc, dt, latestVerticalAccelIMU) {
    // Phase de prédiction (État = Alt précédente + Vitesse Verticale * dt)
    // Ici, nous utilisons une vitesse verticale simplifiée pour la simulation
    const V_vertical_sim = kAltState.altitude === 0 ? 0 : latestVerticalAccelIMU * dt;
    kAltState.altitude += V_vertical_sim; 
    
    const Q_alt = 0.5; // Bruit du modèle d'altitude
    kAltState.uncertainty += Q_alt * dt;

    // Phase de correction
    const R_alt = acc * 2; // Incertitude de la mesure GPS d'altitude
    const K = kAltState.uncertainty / (kAltState.uncertainty + R_alt); // Gain de Kalman

    kAltState.altitude += K * (measuredAlt - kAltState.altitude);
    kAltState.uncertainty = (1 - K) * kAltState.uncertainty;
    kAltUncert = kAltState.uncertainty;

    return kAltState.altitude;
}

/** Calcule le bruit de mesure dynamique (R) pour le filtre de vitesse */
function getKalmanR(effectiveAcc, alt, lastP_hPa, latestLinearAccelMagnitude) {
    let R = effectiveAcc + 0.5; // Base sur la précision GPS
    R += latestLinearAccelMagnitude * 5; // Augmente l'incertitude lors d'accélérations (IMU)
    R = Math.max(1, Math.min(100, R)); // Plafonnement
    return R;
}

// --- 2. FONCTIONS DE L'INTERFACE (Wake Lock, Batterie, Chart) ---
// (Les fonctions `initWakeLock`, `updateBatteryStatus`, `initChart`, `updateChart` sont définies ici pour ne pas dépasser la limite de 500 lignes pour le fichier 1)

/** Tente d'acquérir un Wake Lock. */
async function initWakeLock() {
    // ... implémentation omise par souci de concision du bloc ...
    if ('wakeLock' in navigator) {
        try {
            wakeLock = await navigator.wakeLock.request('screen');
            wakeLock.addEventListener('release', () => { console.log('Wake Lock relâché.'); });
            console.log('Wake Lock acquis.');
        } catch (err) {
            console.warn(`Échec d'acquisition du Wake Lock: ${err.name}`);
        }
    }
}

/** Met à jour l'affichage du niveau de batterie. */
function updateBatteryStatus() {
    // ... implémentation omise par souci de concision du bloc ...
    const $display = $('battery-level');
    if (!$display) return;
    if (!batteryManager && 'getBattery' in navigator) {
        navigator.getBattery().then(manager => {
            batteryManager = manager;
            manager.addEventListener('levelchange', updateBatteryStatus);
            manager.addEventListener('chargingchange', updateBatteryStatus);
            updateBatteryStatus(); 
        });
        return;
    }

    if (batteryManager) {
        const level = (batteryManager.level * 100).toFixed(0);
        const charging = batteryManager.charging;
        const status = charging ? '⚡ Charge' : '🔋 Décharge';
        $display.textContent = `${level} % (${status})`;
    } else {
        $display.textContent = 'N/A (API non supportée)';
    }
}

let speedChart = null;
const MAX_CHART_POINTS = 50;
let chartLabels = [];
let chartData = [];

/** Initialise le graphique de vitesse. */
function initChart() {
    const ctx = $('speed-chart').getContext('2d');
    // ... implémentation Chart.js omise par souci de concision du bloc ...
    speedChart = new Chart(ctx, { /* ... options de graphique ... */ });
    console.log("Graphique Chart.js initialisé.");
}

/** Met à jour le graphique avec de nouvelles données. */
function updateChart(timeLabel, speedKmh) {
    if (!speedChart) { initChart(); return; }
    chartLabels.push(timeLabel);
    chartData.push(speedKmh);
    if (chartLabels.length > MAX_CHART_POINTS) {
        chartLabels.shift();
        chartData.shift();
    }
    speedChart.update('none'); // Pas d'animation pour les updates rapides
}


// --- 3. FONCTIONS ASTRO & TEMPS ---

/** Obtient l'heure courante synchronisée (si syncH a réussi) */
function getCDate() {
    if (lLocH === 0) return new Date();
    const currentLocTime = performance.now();
    const offset = currentLocTime - lLocH;
    return new Date(lServH + offset);
}

/** Synchronisation horaire par serveur (UTC/Atomique) */
async function syncH() { 
    // ... implémentation omise par souci de concision du bloc ...
    $('local-time').textContent = 'Synchronisation UTC...';
    try {
        const response = await fetch(SERVER_TIME_ENDPOINT, { cache: "no-store", mode: "cors" });
        const localEndPerformance = performance.now(); 
        const serverData = await response.json(); 
        const RTT = localEndPerformance - performance.now(); // RTT = temps écoulé
        const latencyOffset = RTT / 2;
        lServH = Date.parse(serverData.datetime) + latencyOffset; 
        lLocH = performance.now(); 
        console.log(`Synchronisation UTC Atomique réussie.`);
    } catch (error) {
        lServH = Date.now(); 
        lLocH = performance.now();
        console.warn("Échec de la synchronisation. Utilisation de l'horloge locale.", error);
    }
}

/** Calcule le Temps Solaire Vrai (TST) normalisé. */
function getSolarTime(date, lon) {
    if (date === null || lon === null) return { TST: 'N/A', MST: 'N/A', EOT: 'N/D', ECL_LONG: 'N/D' };
    const d = (date.valueOf() / dayMs - 0.5 + J1970) - J2000;
    const M = D2R * (356.0470 + 0.9856002585 * d); 
    const L = M + D2R * (1.9148 * Math.sin(M) + 0.0200 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M)) + D2R * 102.9377 + Math.PI; 
    const epsilon = D2R * (23.4393 - 0.000000356 * d); 
    let alpha = Math.atan2(Math.cos(epsilon) * Math.sin(L), Math.cos(L));
    if (alpha < 0) alpha += 2 * Math.PI; 
    
    const meanLongitude = M + D2R * 102.9377 + Math.PI;
    let eot_rad_raw = alpha - meanLongitude; 
    eot_rad_raw = eot_rad_raw % (2 * Math.PI);
    if (eot_rad_raw > Math.PI) eot_rad_raw -= 2 * Math.PI; else if (eot_rad_raw < -Math.PI) eot_rad_raw += 2 * Math.PI;
    const eot_min = eot_rad_raw * 4 * R2D; 
    let ecl_long_deg = (L * R2D) % 360; 
    const final_ecl_long = ecl_long_deg < 0 ? ecl_long_deg + 360 : ecl_long_deg;
    
    const msSinceMidnightUTC = (date.getUTCHours() * 3600 + date.getUTCMinutes() * 60 + date.getUTCSeconds()) * 1000 + date.getUTCMilliseconds();
    const mst_offset_ms = lon * dayMs / 360; 
    const mst_ms_raw = msSinceMidnightUTC + mst_offset_ms;
    const mst_ms = (mst_ms_raw % dayMs + dayMs) % dayMs; 
    const eot_ms = eot_min * 60000;
    const tst_ms_raw = mst_ms + eot_ms;
    const tst_ms = (tst_ms_raw % dayMs + dayMs) % dayMs; 
    
    const toTimeString = (ms) => {
        let h = Math.floor(ms / 3600000);
        let m = Math.floor((ms % 3600000) / 60000);
        let s = Math.floor((ms % 60000) / 1000);
        return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
    };
    return { TST: toTimeString(tst_ms), TST_MS: tst_ms, MST: toTimeString(mst_ms), EOT: eot_min.toFixed(3), ECL_LONG: final_ecl_long.toFixed(2) };
}

/** Met à jour les valeurs Astro, TST et Batterie sur le DOM */
function updateAstro(latA, lonA) {
    const now = getCDate(); 
    if (now === null) return;
    
    updateBatteryStatus();

    const sunPos = window.SunCalc ? SunCalc.getPosition(now, latA, lonA) : null;
    const solarTimes = getSolarTime(now, lonA);
    const elevation_deg = sunPos ? (sunPos.altitude * R2D) : 0;
    
    $('local-time').textContent = now.toLocaleTimeString('fr-FR', { timeZone: 'UTC', hour12: false });
    $('date-display').textContent = now.toLocaleDateString();
    
    if (sTime) {
        const timeElapsed = (now.getTime() - sTime) / 1000;
        $('time-elapsed').textContent = `${timeElapsed.toFixed(2)} s`;
        $('time-moving').textContent = `${timeMoving.toFixed(2)} s`;
    }
    
    $('time-minecraft').textContent = solarTimes.TST; 
    $('tst').textContent = solarTimes.TST;
    $('lsm').textContent = solarTimes.MST;
    $('sun-elevation').textContent = sunPos ? `${elevation_deg.toFixed(2)} °` : 'N/A';
    $('eot').textContent = solarTimes.EOT + ' min'; 
    $('ecliptic-long').textContent = solarTimes.ECL_LONG + ' °';

    // Mise à jour simplifiée des autres champs astro (nécessitent SunCalc)
    if (window.SunCalc) {
        const times = SunCalc.getTimes(now, latA, lonA);
        const dayDurationMs = times.sunset.getTime() - times.sunrise.getTime();
        const dayDurationSec = dayDurationMs / 1000;
        const h = Math.floor(dayDurationSec / 3600);
        const m = Math.floor((dayDurationSec % 3600) / 60);
        $('day-duration').textContent = `${h}h ${m}min`;
    }
}

// --- 4. FONCTIONS API MÉTÉO ---

/** Récupère et met à jour les données météo via OpenWeatherMap. */
async function updateWeather(latA, lonA) {
    // ... implémentation omise par souci de concision du bloc ...
    if (!OWM_API_KEY || OWM_API_KEY === "VOTRE_CLE_API_OPENWEATHERMAP") {
        $('temp-air').textContent = 'API CLÉ MANQUANTE';
        return;
    }
    
    try {
        const url = `${OWM_API_URL}?lat=${latA.toFixed(4)}&lon=${lonA.toFixed(4)}&units=metric&appid=${OWM_API_KEY}`;
        const response = await fetch(url);
        const data = await response.json();
        
        const tempC = data.main.temp;
        const pressurehPa = data.main.pressure;
        const humidity = data.main.humidity;
        const windSpeedMs = data.wind.speed; 
        const windDeg = data.wind.deg;
        
        lastP_hPa = pressurehPa; 

        const dewPointC = calculateDewPoint(tempC, humidity);
        // Utilisation d'une formule simplifiée pour le ressenti
        const windChillC = tempC - (windSpeedMs * 1.5); 
        
        const directions = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSO", "SO", "OSO", "O", "ONO", "NO", "NNO"];
        const windDirection = directions[Math.round((windDeg / 22.5) + 0.5) % 16];

        $('temp-air').textContent = `${tempC.toFixed(1)} °C`;
        $('pressure').textContent = `${pressurehPa.toFixed(0)} hPa | ${(pressurehPa * 0.75006).toFixed(1)} mmHg`;
        $('humidity').textContent = `${humidity.toFixed(0)} %`;
        $('dew-point').textContent = `${dewPointC.toFixed(1)} °C`;
        $('wind-speed-ms').textContent = `${windSpeedMs.toFixed(2)} m/s | ${(windSpeedMs * 3.6).toFixed(1)} km/h`;
        $('wind-direction').textContent = `${windDirection} (${windDeg} °)`;
        $('temp-feels-like').textContent = `${windChillC.toFixed(1)} °C`;
        $('visibility').textContent = data.visibility ? `${(data.visibility / 1000).toFixed(1)} km` : 'N/A (API)';
        
    } catch (error) {
        console.error("Erreur lors de la récupération des données météo:", error);
        $('temp-air').textContent = 'API ERREUR';
    }
}

// --- 5. FONCTIONS CAPTEURS INERTIELS & CARTE ---

/** Gère les données de l'accéléromètre via DeviceMotionEvent. */
function handleDeviceMotion(event) {
    if (emergencyStopActive) return;
    const acc = event.accelerationIncludingGravity;
    if (acc.x === null) return; 

    // Filtrage IIR pour la stabilité
    kAccel.x = iirFilter(acc.x, kAccel.x, ACCEL_FILTER_ALPHA);
    kAccel.y = iirFilter(acc.y, kAccel.y, ACCEL_FILTER_ALPHA);
    kAccel.z = iirFilter(acc.z, kAccel.z, ACCEL_FILTER_ALPHA); 

    const kAccel_mag = Math.sqrt(kAccel.x ** 2 + kAccel.y ** 2 + kAccel.z ** 2);

    if (Math.abs(kAccel_mag - G_ACC) < ACCEL_MOVEMENT_THRESHOLD) {
        // En mode statique, met à jour la référence de gravité
        G_STATIC_REF.x = kAccel.x;
        G_STATIC_REF.y = kAccel.y;
        G_STATIC_REF.z = kAccel.z;
        latestVerticalAccelIMU = 0.0;
        latestLinearAccelMagnitude = 0.0;
    } else {
        // Calcul de l'accélération linéaire
        const linear_x = kAccel.x - G_STATIC_REF.x;
        const linear_y = kAccel.y - G_STATIC_REF.y;
        const linear_z = kAccel.z - G_STATIC_REF.z;
        latestVerticalAccelIMU = linear_z;
        latestLinearAccelMagnitude = Math.sqrt(linear_x ** 2 + linear_y ** 2 + linear_z ** 2); 
    }
    
    if ($('accel-vertical-imu')) $('accel-vertical-imu').textContent = `${latestVerticalAccelIMU.toFixed(3)} m/s²`;
    if ($('force-g-vertical')) $('force-g-vertical').textContent = `${(1 + latestVerticalAccelIMU / G_ACC).toFixed(2)} G`;
}

/** Initialise la carte Leaflet. */
function initMap(latA, lonA) {
    if (map) return;
    try {
        map = L.map('map-container').setView([latA, lonA], 15);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            maxZoom: 19, attribution: '© OpenStreetMap'
        }).addTo(map);

        marker = L.marker([latA, lonA]).addTo(map);
        tracePolyline = L.polyline([], { color: 'red' }).addTo(map);
        console.log("Carte Leaflet initialisée.");
    } catch (e) {
        console.error("Échec de l'initialisation de la carte Leaflet:", e);
    }
}

/** Met à jour la carte. */
function updateMap(latA, lonA) {
    if (!map || !marker || !tracePolyline) { initMap(latA, lonA); return; }
    
    const newLatLng = [latA, lonA];
    marker.setLatLng(newLatLng);
    tracePolyline.addLatLng(newLatLng);
    map.setView(newLatLng);
}

// Fonctions de contrôle GPS (startGPS, stopGPS, setGPSMode, handleErr, emergencyStop, resumeSystem)
// ... Elles sont définies ici et utilisées par les écouteurs d'événements de Part 1 ...

function setGPSMode(mode) {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    currentGPSMode = mode;
    wID = navigator.geolocation.watchPosition(updateDisp, handleErr, GPS_OPTS[mode]);
    $('toggle-gps-btn').textContent = `⏸️ PAUSE GPS`;
}

function startGPS() {
    if (wID === null) {
        setGPSMode(currentGPSMode);
        sTime = null; 
        initWakeLock(); 
    }
}

function stopGPS(resetButton = true) {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
    }
    if (resetButton) { $('toggle-gps-btn').textContent = `▶️ MARCHE GPS`; }
    if (wakeLock) { wakeLock.release(); wakeLock = null; }
}

function emergencyStop() {
    emergencyStopActive = true; stopGPS(false);
    $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: ACTIF 🔴";
    $('emergency-stop-btn').classList.add('active');
}

function resumeSystem() {
    emergencyStopActive = false;
    $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: INACTIF 🟢";
    $('emergency-stop-btn').classList.remove('active');
    startGPS();
}

function handleErr(err) {
    console.error(`Erreur GNSS (${err.code}): ${err.message}`);
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = `❌ ERREUR GPS`;
}

// --- 6. FONCTION PRINCIPALE DE MISE À JOUR ---

function updateDisp(pos) {
    if (emergencyStopActive) return;
    lat = pos.coords.latitude; lon = pos.coords.longitude;
    const alt = pos.coords.altitude, acc = pos.coords.accuracy;
    const spd_raw_gps = pos.coords.speed;
    const cTimePos = pos.timestamp; 
    const now = getCDate(); 
    const MASS = 70.0; 

    if (now === null) { updateAstro(lat, lon); return; } 
    if (sTime === null) { sTime = now.getTime(); }
    if (acc > MAX_ACC) { 
        if ($('gps-precision')) $('gps-precision').textContent = `❌ ${acc.toFixed(0)} m (Trop Imprécis)`; 
        if (lPos === null) lPos = pos; return; 
    }

    let effectiveAcc = acc;
    const accOverride = parseFloat($('gps-accuracy-override').value);
    if (accOverride > 0) { effectiveAcc = accOverride; }

    let spdH = spd_raw_gps ?? 0; 
    const dt = lPos ? (cTimePos - lPos.timestamp) / 1000 : MIN_DT;

    // --- 1. FILTRAGE & CALCULS ---
    iirSpdRaw = iirFilter(spd_raw_gps || 0, iirSpdRaw);

    const kAlt_new = kFilterAltitude(alt || 0, effectiveAcc, dt, latestVerticalAccelIMU); 
    const altitude_msl = kAlt_new - MSL_CORRECTION;
    
    let spdV = 0;
    if (lPos && lPos.kAlt_old !== undefined && dt > MIN_DT && alt !== null) { 
        spdV = (kAlt_new - lPos.kAlt_old) / dt; 
    }
    
    if (lPos && dt > 0.05) { 
        const dH = dist(lPos.coords.latitude, lPos.coords.longitude, lat, lon); 
        spdH = dH / dt; 
    } 

    let spd3D = Math.sqrt(spdH ** 2 + spdV ** 2);
    iirSpd3D = iirFilter(spd3D, iirSpd3D);

    const R_dyn = getKalmanR(effectiveAcc, alt, lastP_hPa, latestLinearAccelMagnitude); 
    const fSpd = kFilter(spd3D, dt, R_dyn); 
    const sSpdFE = fSpd < MIN_SPD ? 0 : fSpd; 

    // --- CALCULS PHYSIQUES ---
    let accel_long = (dt > 0.05) ? (sSpdFE - lastFSpeed) / dt : 0;
    lastFSpeed = sSpdFE;
    distM += sSpdFE * dt * (netherMode ? NETHER_RATIO : 1); 
    if (sSpdFE > MIN_SPD) { timeMoving += dt; }
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    const avgSpdMoving = timeMoving > 0 ? (distM / timeMoving) : 0;
    
    const coriolusForce = 2 * MASS * sSpdFE * W_EARTH * Math.sin(lat * D2R);
    const kineticEnergy = 0.5 * MASS * sSpdFE ** 2;
    const mechanicalPower = accel_long * MASS * sSpdFE; 
    
    let airDensity = 1.225; // Valeur par défaut
    let dynamicPressure = 0.0;
    const tempElement = $('temp-air').textContent;
    const tempCMatch = tempElement.match(/(-?[\d.]+)\s*°C/);
    
    if (tempCMatch && lastP_hPa) {
        const tempC = parseFloat(tempCMatch[1]);
        const tempK = tempC + 273.15; 
        const pressurePa = lastP_hPa * 100; 
        const R_specific = 287.058; 

        if (!isNaN(tempK) && tempK > 0 && pressurePa > 0) {
            airDensity = (pressurePa / (R_specific * tempK));
            dynamicPressure = 0.5 * airDensity * sSpdFE ** 2; // q = 1/2 * rho * v²
            $('air-density').textContent = airDensity.toFixed(3) + ' kg/m³';
        }
    }
    $('dynamic-pressure').textContent = dynamicPressure.toFixed(2) + ' Pa';

    const v_c_ratio = sSpdFE / C_L;
    const lorentzFactor = 1 / Math.sqrt(1 - (v_c_ratio * v_c_ratio));


    // ... suite de la fonction updateDisp(pos) ...

    // --- 7. MISE À JOUR DU DOM ---
    const sSpdFE_Kmh = (sSpdFE * KMH_MS);
    $('speed-stable').textContent = `${sSpdFE_Kmh.toFixed(5)} km/h`;
    $('speed-stable-ms').textContent = `${sSpdFE.toFixed(3)} m/s | ${(sSpdFE * 1000).toFixed(0)} mm/s`;
    $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    $('speed-avg-moving').textContent = `${(avgSpdMoving * KMH_MS).toFixed(5)} km/h`;
    $('speed-raw-ms').textContent = `${iirSpdRaw.toFixed(3)} m/s`;
    $('speed-3d-inst').textContent = `${(iirSpd3D * KMH_MS).toFixed(5)} km/h`;
    
    $('accel-long').textContent = `${accel_long.toFixed(3)} m/s²`;
    $('force-g-long').textContent = `${(accel_long / G_ACC).toFixed(2)} G`;

    const distM_display = distM * (netherMode ? NETHER_RATIO : 1);
    $('distance-total-km').textContent = `${(distM_display / 1000).toFixed(3)} km | ${distM_display.toFixed(2)} m`;
    
    $('vertical-speed').textContent = `${spdV.toFixed(2)} m/s`;
    $('altitude-gps').textContent = alt !== null ? `${alt.toFixed(2)} m` : 'N/A';
    $('altitude-msl').textContent = `${altitude_msl.toFixed(2)} m`; 
    $('underground-status').textContent = (altitude_msl < -50) ? 'OUI (Seuil -50m)' : 'Non';

    $('latitude').textContent = formatNumber(lat, 6);
    $('longitude').textContent = formatNumber(lon, 6);
    $('gps-precision').textContent = `${acc.toFixed(3)} m`;
    $('gps-accuracy-effective').textContent = `${effectiveAcc.toFixed(3)} m`;
    $('speed-error-perc').textContent = `${(R_dyn / 10).toFixed(2)} %`; 
    
    $('kinetic-energy').textContent = `${kineticEnergy.toFixed(2)} J`;
    $('mechanical-power').textContent = `${mechanicalPower.toFixed(2)} W`;
    $('coriolis-force').textContent = `${coriolusForce.toFixed(4)} N`;
    $('perc-speed-c').textContent = `${(v_c_ratio * 100).toFixed(8)} %`;
    $('lorentz-factor').textContent = `γ = ${lorentzFactor.toFixed(6)}`; 
    
    const tempElement = $('temp-air').textContent;
    const tempCMatch = tempElement.match(/(-?[\d.]+)\s*°C/);

    if (tempCMatch) {
        const tempC = parseFloat(tempCMatch[1]);
        const soundSpeed = 331.3 + 0.606 * tempC; 
        const percSound = (sSpdFE / soundSpeed) * 100;
        $('perc-speed-sound').textContent = `${percSound.toFixed(2)} %`;
    }

    // --- 8. MISE À JOUR DES COMPOSANTS ---
    updateAstro(lat, lon);
    if (sSpdFE_Kmh > 0.1) {
        const timeLabel = new Date().toLocaleTimeString('fr-FR');
        updateChart(timeLabel, sSpdFE_Kmh);
    }
    updateMap(lat, lon);
    
    if (currentGPSMode === 'LOW_FREQ' && (now.getTime() - lastWeatherUpdate > WEATHER_SYNC_INTERVAL_LOW)) {
        updateWeather(lat, lon);
        lastWeatherUpdate = now.getTime();
    }


    // --- 9. SAUVEGARDE DE LA DERNIÈRE POSITION ---
    lPos = { 
        coords: pos.coords, 
        timestamp: cTimePos, 
        kAlt_old: kAlt_new, // Sauvegarde l'altitude filtrée pour le calcul de vitesse verticale au prochain cycle
        kAltUncert_old: kAltUncert
    };
} // Fin de la fonction updateDisp(pos)


// ===========================================
// 10. INITIALISATION GLOBALE ET ÉVÉNEMENTS
// ===========================================

/** Point d'entrée après le chargement du DOM et de Part 1 */
document.addEventListener('DOMContentLoaded', () => {
    // Initialisation des options de la carte et de l'accéléromètre
    window.addEventListener('devicemotion', handleDeviceMotion);
    
    // Liaison des événements de contrôle
    $('toggle-gps-btn').addEventListener('click', () => {
        if (wID === null) {
            startGPS();
        } else {
            stopGPS();
        }
    });

    $('emergency-stop-btn').addEventListener('click', () => {
        if (emergencyStopActive) {
            resumeSystem();
        } else {
            emergencyStop();
        }
    });

    $('reset-all-btn').addEventListener('click', resetAll);
    $('reset-dist-btn').addEventListener('click', resetDistance);
    $('reset-max-btn').addEventListener('click', resetMaxSpeed);
    
    $('freq-select').addEventListener('change', (e) => {
        setGPSMode(e.target.value);
    });

    $('nether-toggle-btn').addEventListener('click', toggleNetherMode);
    $('toggle-mode-btn').addEventListener('click', toggleNightMode); 
    
    // Synchronisation horaire initiale et récupération météo
    syncH();
    // Tente d'utiliser une position par défaut si le GPS n'est pas encore démarré
    updateWeather(DEFAULT_LAT, DEFAULT_LON); 
    updateBatteryStatus();
    
    // Initialisation des données au démarrage
    updateAstro(DEFAULT_LAT, DEFAULT_LON);
    initMap(DEFAULT_LAT, DEFAULT_LON);
    
    // Initialisation des valeurs par défaut dans l'UI
    $('freq-select').value = currentGPSMode; 
    $('mass-display').textContent = '70.000 kg'; // Masse par défaut
    $('o2-level').textContent = '20.9 % vol (Fixe)'; // O2 par défaut

    console.log("Système prêt. Attente de la commande MARCHE GPS.");
});
