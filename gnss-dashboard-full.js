// =================================================================
// FICHIER JS PARTIE 1 : gnss-dashboard-part1.js (Constantes, Filtres & Kalman)
// =================================================================

const $ = (id) => document.getElementById(id);

// --- CONSTANTES GLOBALES (Physiques, GPS, Temps) ---
const C_L = 299792458; // Vitesse de la lumière (m/s)
const SPEED_SOUND = 343; // Vitesse du son (m/s)
const G_ACC = 9.80665; // Gravité standard (m/s²)
const KMH_MS = 3.6; 
const R_E = 6371000; // Rayon moyen de la Terre (m)
const R2D = 180 / Math.PI;
const D2R = Math.PI / 180;
const W_EARTH = 7.2921E-5; // Vitesse angulaire de la Terre (rad/s)
const NETHER_RATIO = 1 / 8; 

// Constantes Temps / Astro
const dayMs = 86400000;
const J1970 = 2440588; 
const J2000 = 2451545; 
const DOM_SLOW_UPDATE_MS = 1000;
const WEATHER_UPDATE_MS = 30000; 
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc"; 

// Constantes GPS
const MIN_DT = 0.05; 
const MIN_SPD = 0.01; 
const MAX_ACC = 20; 
const ALT_TH = -50; 
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 5000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 30000, timeout: 60000 }
};

// Constantes Kalman (Vitesse 3D)
const Q_NOISE = 0.001; 
let kSpd = 0; // Estimation vitesse (m/s)
let kUncert = 1000; // Incertitude vitesse (m/s)²
const ENVIRONMENT_FACTORS = {
    NORMAL: 1.0, FOREST: 1.5, CONCRETE: 3.0, METAL: 2.5
};

// Constantes Kalman (Altitude)
const Q_ALT_NOISE = 0.01; 
let kAlt = 0; // Estimation altitude (m)
let kAltUncert = 1000; // Incertitude altitude (m)²

// Constantes IMU (Accéléromètre)
const ACCEL_FILTER_ALPHA = 0.8; 
const ACCEL_MOVEMENT_THRESHOLD = 0.5; 
let kAccel = { x: 0, y: 0, z: 0 };
let G_STATIC_REF = { x: 0, y: 0, z: 0 };
let latestVerticalAccelIMU = 0; // Accélération verticale pour filtre altitude
let latestLinearAccelMagnitude = 0; // Magnitude de l'accélération pour bruit EKF

// --- VARIABLES GLOBALES (État du système) ---
let wID = null;
let map = null;
let marker = null;
let tracePolyline = null;
let lat = 0, lon = 0;
let sTime = null; 
let distM = 0; 
let timeMoving = 0; 
let maxSpd = 0;
let lastFSpeed = 0;
let currentGPSMode = 'HIGH_FREQ';
let lPos = null; 
let emergencyStopActive = false;
let netherMode = false;
let selectedEnvironment = 'NORMAL';
let lServH = 0; 
let lLocH = 0; 
let domID = null; 
let weatherID = null; 
let lastP_hPa = 1013.25; 
let wakeLock = null; // Wake Lock API
let batteryManager = null; // Battery Status API

// --- VARIABLES FILTRAGE IIR ---
const IIR_ALPHA = 0.2; // Facteur de lissage IIR (plus petit = plus lisse)
let iirSpdRaw = 0; // Vitesse brute lissée
let iirSpd3D = 0; // Vitesse 3D lissée

// --- CONSTANTES API MÉTÉO ---
const OWM_API_KEY = "VOTRE_CLE_API_OPENWEATHERMAP"; // <-- REMPLACEZ CECI PAR VOTRE CLÉ API OPENWEATHERMAP
const OWM_API_URL = "https://api.openweathermap.org/data/2.5/weather";


// ===========================================
// FONCTIONS UTILITAIRES ET KALMAN
// ===========================================

/** Calcule la distance de Haversine entre deux points (m). */
function dist(lat1, lon1, lat2, lon2) {
    const dLat = (lat2 - lat1) * D2R;
    const dLon = (lon2 - lon1) * D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1 * D2R) * Math.cos(lat2 * D2R) * Math.sin(dLon / 2) ** 2;
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R_E * c;
}

/** Filtre IIR (Infinite Impulse Response) pour lisser les données. */
function iirFilter(current, previous, alpha = IIR_ALPHA) {
    return alpha * current + (1 - alpha) * previous;
}

/** Filtre de Kalman 1D (Vitesse) */
function kFilter(z, dt, R_dyn) {
    // 1. Prediction
    const predSpd = kSpd;
    const predUncert = kUncert + Q_NOISE * dt;

    // 2. Mesure
    const K = predUncert / (predUncert + R_dyn);
    kSpd = predSpd + K * (z - predSpd);
    kUncert = (1 - K) * predUncert;
    return kSpd;
}

/** Calcule le bruit de mesure dynamique R (Intègre IMU linéaire) */
function getKalmanR(acc, alt, pressure, linearAccelMag) { 
    let R_raw = acc * acc; 
    const envFactor = ENVIRONMENT_FACTORS[selectedEnvironment] || ENVIRONMENT_FACTORS.NORMAL;
    
    // 1. Base Noise + Correction d'Altitude
    let noiseMultiplier = envFactor;
    if (alt !== null && alt < 0) {
        noiseMultiplier += Math.abs(alt / 100); 
    }
    
    // 2. Correction par l'Accélération Linéaire (IMU)
    if (linearAccelMag > 0.5) { 
        noiseMultiplier += Math.pow(linearAccelMag, 1.5) * 0.5; 
    }
    
    return R_raw * noiseMultiplier;
}

/** Filtre de Kalman 1D pour l'Altitude (Utilise u_accel IMU) */
function kFilterAltitude(z, acc, dt, u_accel = 0) { 
    if (z === null) return kAlt; 

    // 1. Prediction (avec Accélération IMU comme entrée de contrôle)
    const predAlt = kAlt + (0.5 * u_accel * dt * dt); 
    let predAltUncert = kAltUncert + Q_ALT_NOISE * dt;

    // 2. Mesure (R_alt est basé sur la précision brute du GPS)
    const R_alt = acc * acc * 2.0; 
    const K = predAltUncert / (predAltUncert + R_alt);
    kAlt = predAlt + K * (z - predAlt);
    kAltUncert = (1 - K) * predAltUncert;

    return kAlt;
}

/** Calcule le point de rosée (utilitaire pour la météo) */
function calculateDewPoint(tempC, humidity) {
    const a = 17.27;
    const b = 237.7;
    const alpha = (a * tempC) / (b + tempC) + Math.log(humidity / 100);
    return (b * alpha) / (a - alpha);
        }
// =================================================================
// FICHIER JS PARTIE 2 : gnss-dashboard-part2.js (Logique d'Exécution)
// Dépend de gnss-dashboard-part1.js
// =================================================================

/** Obtient l'heure courante synchronisée (si syncH a réussi) */
function getCDate() {
    if (lLocH === 0) return new Date();
    const currentLocTime = performance.now();
    const offset = currentLocTime - lLocH;
    return new Date(lServH + offset);
}

// ===========================================
// FONCTIONS DE L'INTERFACE (Wake Lock, Batterie, Chart)
// ===========================================

/** Tente d'acquérir un Wake Lock pour empêcher l'écran de se verrouiller. */
async function initWakeLock() {
    if ('wakeLock' in navigator) {
        try {
            wakeLock = await navigator.wakeLock.request('screen');
            wakeLock.addEventListener('release', () => {
                console.log('Wake Lock relâché.');
            });
            console.log('Wake Lock acquis.');
        } catch (err) {
            console.warn(`Échec d'acquisition du Wake Lock: ${err.name}, ${err.message}`);
        }
    } else {
        console.warn("Wake Lock API non supportée.");
    }
}

/** Met à jour l'affichage du niveau de batterie. */
function updateBatteryStatus() {
    const $display = $('battery-level');
    if (!batteryManager) {
        if ('getBattery' in navigator) {
            navigator.getBattery().then(manager => {
                batteryManager = manager;
                manager.addEventListener('levelchange', updateBatteryStatus);
                manager.addEventListener('chargingchange', updateBatteryStatus);
                updateBatteryStatus(); 
            });
        } else {
            $display.textContent = 'N/A (API non supportée)';
            return;
        }
    }

    if (batteryManager) {
        const level = (batteryManager.level * 100).toFixed(0);
        const charging = batteryManager.charging;
        const status = charging ? '⚡ Charge' : '🔋 Décharge';
        $display.textContent = `${level} % (${status})`;
        
        // Coloriage
        $display.classList.remove('quality-green', 'quality-orange', 'quality-red');
        if (level > 70) $display.classList.add('quality-green');
        else if (level > 20) $display.classList.add('quality-orange');
        else $display.classList.add('quality-red');
    }
}

let speedChart = null;
const MAX_CHART_POINTS = 50;
let chartLabels = [];
let chartData = [];

/** Initialise le graphique de vitesse. */
function initChart() {
    const ctx = $('speed-chart').getContext('2d');
    speedChart = new Chart(ctx, {
        type: 'line',
        data: {
            labels: chartLabels,
            datasets: [{
                label: 'Vitesse Stable (km/h)',
                data: chartData,
                borderColor: 'rgb(75, 192, 192)',
                tension: 0.1,
                pointRadius: 0
            }]
        },
        options: {
            animation: false,
            responsive: true,
            maintainAspectRatio: false,
            scales: {
                x: { display: false },
                y: { beginAtZero: true, title: { display: true, text: 'Vitesse (km/h)' } }
            }
        }
    });
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

    speedChart.update();
}


// ===========================================
// FONCTIONS ASTRO & TEMPS
// ===========================================

/** Synchronisation horaire par serveur (UTC/Atomique) */
async function syncH() { 
    // (Fonction syncH identique à la version précédente)
    if ($('local-time')) $('local-time').textContent = 'Synchronisation UTC...';
    const localStartPerformance = performance.now(); 
    try {
        const response = await fetch(SERVER_TIME_ENDPOINT, { cache: "no-store", mode: "cors" });
        if (!response.ok) throw new Error(`Server time sync failed: ${response.statusText}`);
        const localEndPerformance = performance.now(); 
        const serverData = await response.json(); 
        const RTT = localEndPerformance - localStartPerformance;
        const latencyOffset = RTT / 2;
        lServH = Date.parse(serverData.datetime) + latencyOffset; 
        lLocH = performance.now(); 
        console.log(`Synchronisation UTC Atomique réussie.`);
    } catch (error) {
        console.warn("Échec de la synchronisation. Utilisation de l'horloge locale.", error);
        lServH = Date.now(); 
        lLocH = performance.now();
        if ($('local-time')) $('local-time').textContent = 'N/A (SYNCHRO ÉCHOUÉE)';
    }
}

/** Convertit la date en jours depuis J2000. */
function toDays(date) { return (date.valueOf() / dayMs - 0.5 + J1970) - J2000; }
/** Calcule l'anomalie solaire moyenne. */
function solarMeanAnomaly(d) { return D2R * (356.0470 + 0.9856002585 * d); }
/** Calcule la longitude écliptique. */
function eclipticLongitude(M) {
    var C = D2R * (1.9148 * Math.sin(M) + 0.0200 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M)), 
        P = D2R * 102.9377;                                                                
    return M + C + P + Math.PI;
}

/** Calcule le Temps Solaire Vrai (TST) normalisé. */
function getSolarTime(date, lon) {
    // (Fonction getSolarTime identique à la version précédente)
    if (date === null || lon === null) return { TST: 'N/A', MST: 'N/A', EOT: 'N/D', ECL_LONG: 'N/D' };
    const d = toDays(date);
    const M = solarMeanAnomaly(d); 
    const L = eclipticLongitude(M); 
    const epsilon = D2R * (23.4393 - 0.000000356 * d); 
    let alpha = Math.atan2(Math.cos(epsilon) * Math.sin(L), Math.cos(L));
    if (alpha < 0) alpha += 2 * Math.PI; 
    
    const meanLongitude = M + D2R * 102.9377 + Math.PI;
    let eot_rad_raw = alpha - meanLongitude; 
    eot_rad_raw = eot_rad_raw % (2 * Math.PI);
    if (eot_rad_raw > Math.PI) { eot_rad_raw -= 2 * Math.PI; } else if (eot_rad_raw < -Math.PI) { eot_rad_raw += 2 * Math.PI; }
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
}

// ===========================================
// FONCTIONS API MÉTÉO (Identique)
// ===========================================

/** Récupère et met à jour les données météo via OpenWeatherMap. */
async function updateWeather(latA, lonA) {
    if (!OWM_API_KEY || OWM_API_KEY === "VOTRE_CLE_API_OPENWEATHERMAP") {
        $('temp-air').textContent = 'API CLÉ MANQUANTE';
        return;
    }
    
    // (Reste de la fonction updateWeather est identique à la version précédente)
    if ($('temp-air')) $('temp-air').textContent = 'Chargement...';

    try {
        const url = `${OWM_API_URL}?lat=${latA.toFixed(4)}&lon=${lonA.toFixed(4)}&units=metric&appid=${OWM_API_KEY}`;
        const response = await fetch(url);
        if (!response.ok) throw new Error(`Weather API failed: ${response.statusText}`);
        const data = await response.json();
        
        const tempC = data.main.temp;
        const pressurehPa = data.main.pressure;
        const humidity = data.main.humidity;
        const windSpeedMs = data.wind.speed; 
        const windDeg = data.wind.deg;
        
        lastP_hPa = pressurehPa; 

        const dewPointC = calculateDewPoint(tempC, humidity);
        const windChillC = 13.12 + 0.6215 * tempC - 11.37 * (windSpeedMs**0.16) + 0.3965 * tempC * (windSpeedMs**0.16);
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


// ===========================================
// FONCTIONS CAPTEURS INERTIELS & CARTE (Identique)
// ===========================================

/** Gère les données de l'accéléromètre via DeviceMotionEvent. */
function handleDeviceMotion(event) {
    // (Fonction handleDeviceMotion est identique à la version précédente)
    if (emergencyStopActive) return;
    const acc = event.accelerationIncludingGravity;
    if (acc.x === null) return; 

    kAccel.x = ACCEL_FILTER_ALPHA * kAccel.x + (1 - ACCEL_FILTER_ALPHA) * acc.x;
    kAccel.y = ACCEL_FILTER_ALPHA * kAccel.y + (1 - ACCEL_FILTER_ALPHA) * acc.y;
    kAccel.z = ACCEL_FILTER_ALPHA * kAccel.z + (1 - ACCEL_FILTER_ALPHA) * acc.z; 

    const kAccel_mag = Math.sqrt(kAccel.x ** 2 + kAccel.y ** 2 + kAccel.z ** 2);
    let accel_vertical_lin = 0.0;
    let linear_x = 0.0, linear_y = 0.0, linear_z = 0.0;

    if (Math.abs(kAccel_mag - G_ACC) < ACCEL_MOVEMENT_THRESHOLD) {
        G_STATIC_REF.x = kAccel.x;
        G_STATIC_REF.y = kAccel.y;
        G_STATIC_REF.z = kAccel.z;
        accel_vertical_lin = 0.0; 
    } else {
        linear_x = kAccel.x - G_STATIC_REF.x;
        linear_y = kAccel.y - G_STATIC_REF.y;
        linear_z = kAccel.z - G_STATIC_REF.z;
        accel_vertical_lin = linear_z;
    }

    latestVerticalAccelIMU = accel_vertical_lin; 
    latestLinearAccelMagnitude = Math.sqrt(linear_x ** 2 + linear_y ** 2 + linear_z ** 2); 
    
    if ($('accel-vertical-imu')) $('accel-vertical-imu').textContent = `${accel_vertical_lin.toFixed(3)} m/s²`;
    if ($('force-g-vertical')) $('force-g-vertical').textContent = `${(accel_vertical_lin / G_ACC).toFixed(2)} G`;
}

/** Initialise la carte Leaflet. */
function initMap(latA, lonA) {
    // (Fonction initMap est identique à la version précédente)
    if (map) return;
    try {
        map = L.map('map-container').setView([latA, lonA], 15);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            maxZoom: 19,
            attribution: '© OpenStreetMap'
        }).addTo(map);

        marker = L.marker([latA, lonA]).addTo(map);
        tracePolyline = L.polyline([], { color: 'red' }).addTo(map);
        console.log("Carte Leaflet initialisée.");
    } catch (e) {
        console.error("Échec de l'initialisation de la carte Leaflet:", e);
        if ($('map-container')) $('map-container').textContent = 'Erreur d\'initialisation de la carte.';
    }
}

/** Met à jour la carte. */
function updateMap(latA, lonA) {
    // (Fonction updateMap est identique à la version précédente)
    if (!map || !marker || !tracePolyline) { initMap(latA, lonA); return; }
    
    const newLatLng = [latA, lonA];
    marker.setLatLng(newLatLng);
    tracePolyline.addLatLng(newLatLng);
    map.setView(newLatLng);
}

function setGPSMode(mode) {
    // (Fonction setGPSMode est identique à la version précédente)
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    currentGPSMode = mode;
    wID = navigator.geolocation.watchPosition(updateDisp, handleErr, GPS_OPTS[mode]);
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = `⏸️ PAUSE GPS`;
    if ($('freq-select')) $('freq-select').value = mode; 
}

function startGPS() {
    // (Fonction startGPS est identique à la version précédente)
    if (wID === null) {
        if ($('freq-select')) $('freq-select').value = currentGPSMode; 
        setGPSMode(currentGPSMode);
        sTime = null; 
        initWakeLock(); 
    }
}

function stopGPS(resetButton = true) {
    // (Fonction stopGPS est identique à la version précédente)
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
    }
    if (resetButton) {
        if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = `▶️ MARCHE GPS`;
    }
    if (wakeLock) {
        wakeLock.release();
        wakeLock = null;
        console.log('Wake Lock relâché à l\'arrêt du GPS.');
    }
}

function emergencyStop() {
    // (Fonction emergencyStop est identique à la version précédente)
    emergencyStopActive = true;
    stopGPS(false);
    if ($('emergency-stop-btn')) {
        $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: ACTIF 🔴";
        $('emergency-stop-btn').classList.add('active');
    }
}

function resumeSystem() {
    // (Fonction resumeSystem est identique à la version précédente)
    emergencyStopActive = false;
    if ($('emergency-stop-btn')) {
        $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: INACTIF 🟢";
        $('emergency-stop-btn').classList.remove('active');
    }
    startGPS();
}

function handleErr(err) {
    // (Fonction handleErr est identique à la version précédente)
    console.error(`Erreur GNSS (${err.code}): ${err.message}`);
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = `❌ ERREUR GPS`;
}


// ===========================================
// FONCTION PRINCIPALE DE MISE À JOUR (GPS, Kalman, Physique)
// ===========================================

function updateDisp(pos) {
    if (emergencyStopActive) return;
    lat = pos.coords.latitude; lon = pos.coords.longitude;
    const alt = pos.coords.altitude, acc = pos.coords.accuracy;
    const spd_raw_gps = pos.coords.speed;
    const heading = pos.coords.heading;
    const cTimePos = pos.timestamp; 
    const now = getCDate(); 
    const MASS = 70.0; 
    const MSL_CORRECTION = 48.0; // Exemple de correction géoïde pour MSL (Approximation statique, devrait être dynamique)

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

    // --- 1. FILTRAGE IIR (Lissage de la vitesse brute) ---
    iirSpdRaw = iirFilter(spd_raw_gps || 0, iirSpdRaw);

    // 2. FILTRAGE DE L'ALTITUDE (via Kalman) - Utilise l'IMU verticale
    const kAlt_new = kFilterAltitude(alt, effectiveAcc, dt, latestVerticalAccelIMU); 
    const altitude_msl = kAlt_new !== null ? kAlt_new - MSL_CORRECTION : null;
    
    // 3. VITESSE VERTICALE FILTRÉE ET INCERTITUDE 
    let spdV = 0; 
    let verticalSpeedUncert = 0;

    if (lPos && lPos.kAlt_old !== undefined && dt > MIN_DT && alt !== null) { 
        spdV = (kAlt_new - lPos.kAlt_old) / dt; 
        
        const kAltUncert_old = lPos.kAltUncert_old !== undefined ? lPos.kAltUncert_old : kAltUncert;
        verticalSpeedUncert = Math.sqrt(kAltUncert ** 2 + kAltUncert_old ** 2) / dt;
        verticalSpeedUncert = Math.min(20, verticalSpeedUncert); 
    } else if (alt !== null) { 
        spdV = 0; 
    }
    
    // 4. VITESSE HORIZONTALE CALCULÉE
    if (lPos && dt > 0.05) { 
        const dH = dist(lPos.coords.latitude, lPos.coords.longitude, lat, lon); 
        spdH = dH / dt; 
    } 

    // 5. VITESSE 3D ET FILTRAGE IIR
    let spd3D = Math.sqrt(spdH ** 2 + spdV ** 2);
    iirSpd3D = iirFilter(spd3D, iirSpd3D);

    // 6. FILTRE DE KALMAN FINAL (Vitesse 3D Stable) - Utilise l'IMU linéaire pour R_dyn
    const R_dyn = getKalmanR(effectiveAcc, alt, lastP_hPa, latestLinearAccelMagnitude); 
    const fSpd = kFilter(spd3D, dt, R_dyn); 
    const sSpdFE = fSpd < MIN_SPD ? 0 : fSpd; 

    // --- CALCULS PHYSIQUES AVANCÉS ---
    let accel_long = (dt > 0.05) ? (sSpdFE - lastFSpeed) / dt : 0;
    lastFSpeed = sSpdFE;
    distM += sSpdFE * dt * (netherMode ? NETHER_RATIO : 1); 
    if (sSpdFE > MIN_SPD) { timeMoving += dt; }
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    const avgSpdMoving = timeMoving > 0 ? (distM / timeMoving) : 0;
    
    const coriolusForce = 2 * MASS * sSpdFE * W_EARTH * Math.sin(lat * D2R);
    const kineticEnergy = 0.5 * MASS * sSpdFE ** 2;
    const mechanicalPower = accel_long * MASS * sSpdFE; 
    
    // CALCUL DENSITÉ AIR & PRESSION DYNAMIQUE
    let airDensity = 0.0;
    let dynamicPressure = 0.0;
    const tempElement = $('temp-air').textContent;
    const tempCMatch = tempElement.match(/(-?[\d.]+)\s*°C/);
    
    if (tempCMatch) {
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


    // CALCUL FACTEUR DE LORENTZ ($\gamma$) - Était le dernier bloc du code fourni
    const v_c_ratio = sSpdFE / C_L;
    const lorentzFactor = 1 / Math.sqrt(1 - (v_c_ratio * v_c_ratio));


    // --- MISE À JOUR DU DOM (GPS/Physique) ---
    const sSpdFE_Kmh = (sSpdFE * KMH_MS);
    
    // Vitesse & Accélération
    $('speed-stable').textContent = `${sSpdFE_Kmh.toFixed(5)} km/h`;
    $('speed-stable-ms').textContent = `${sSpdFE.toFixed(3)} m/s | ${(sSpdFE * 1000).toFixed(0)} mm/s`;
    $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    $('speed-avg-moving').textContent = `${(avgSpdMoving * KMH_MS).toFixed(5)} km/h`;
    $('speed-raw-ms').textContent = `${iirSpdRaw.toFixed(3)} m/s`;
    $('speed-3d-inst').textContent = `${(iirSpd3D * KMH_MS).toFixed(5)} km/h`;
    
    $('accel-long').textContent = `${accel_long.toFixed(3)} m/s²`;
    $('force-g-long').textContent = `${(accel_long / G_ACC).toFixed(2)} G`;

    // Distance et Temps
    const distM_display = distM * (netherMode ? NETHER_RATIO : 1); // Applique le ratio Nether
    $('distance-total-km').textContent = `${(distM_display / 1000).toFixed(3)} km | ${distM_display.toFixed(2)} m`;
    
    // Vertical & Altitude
    $('vertical-speed').textContent = `${spdV.toFixed(2)} m/s`;
    $('altitude-gps').textContent = alt !== null ? `${alt.toFixed(2)} m` : 'N/A';
    // Assumant que 'altitude-msl' est un ID existant
    $('altitude-msl').textContent = altitude_msl !== null ? `${altitude_msl.toFixed(2)} m` : 'N/A'; 
    $('underground-status').textContent = (altitude_msl !== null && altitude_msl < -50) ? 'OUI (Seuil -50m)' : 'Non';

    // Précision et EKF
    $('gps-precision').textContent = `${acc.toFixed(3)} m`;
    $('gps-accuracy-effective').textContent = `${effectiveAcc.toFixed(3)} m`;
    // Assumant que 'speed-error-perc' est l'incertitude du Kalman (R_dyn)
    $('speed-error-perc').textContent = `${(R_dyn * 10).toFixed(2)} %`; 
    
    // Physique avancée et Relativité
    $('kinetic-energy').textContent = `${kineticEnergy.toFixed(2)} J`;
    $('mechanical-power').textContent = `${mechanicalPower.toFixed(2)} W`;
    $('coriolis-force').textContent = `${coriolusForce.toFixed(4)} N`;
    $('perc-speed-c').textContent = `${(v_c_ratio * 100).toFixed(8)} %`;
    // Assumant que 'lorentz-factor' est un ID existant
    $('lorentz-factor').textContent = `γ = ${lorentzFactor.toFixed(6)}`; 
    
    // Vitesse du son et pourcentage (Utilise la température récupérée par la météo)
    const tempElement = $('temp-air').textContent;
    const tempCMatch = tempElement.match(/(-?[\d.]+)\s*°C/);

    if (tempCMatch) {
        const tempC = parseFloat(tempCMatch[1]);
        // Formule de la vitesse du son (approximation standard)
        const soundSpeed = 331.3 + 0.606 * tempC; 
        const percSound = (sSpdFE / soundSpeed) * 100;
        $('perc-speed-sound').textContent = `${percSound.toFixed(2)} %`;
    }

    // --- MISE À JOUR DES COMPOSANTS ---
    updateAstro(lat, lon);
    if (sSpdFE_Kmh > 0.1) {
        const timeLabel = new Date().toLocaleTimeString('fr-FR');
        updateChart(timeLabel, sSpdFE_Kmh);
    }
    updateMap(lat, lon);
    
    // Si nous sommes en mode GPS Basse Fréquence, forcer la synchro météo
    if (currentGPSMode === 'LOW_FREQ' && (now.getTime() - lastWeatherUpdate > WEATHER_SYNC_INTERVAL_LOW)) {
        updateWeather(lat, lon);
        lastWeatherUpdate = now.getTime();
    }


    // --- SAUVEGARDE DE LA DERNIÈRE POSITION ---
    lPos = { 
        coords: pos.coords, 
        timestamp: cTimePos, 
        kAlt_old: kAlt_new, 
        kAltUncert_old: kAltUncert
    };
} // Fin de la fonction updateDisp(pos)


// ===========================================
// INITIALISATION GLOBALE ET ÉVÉNEMENTS
// ===========================================

/** Point d'entrée après le chargement du DOM et de Part 1 */
document.addEventListener('DOMContentLoaded', () => {
    // Initialisation des options de la carte et de l'accéléromètre
    window.addEventListener('devicemotion', handleDeviceMotion);
    
    // Liaison des boutons de contrôle définis dans Part 1 avec les fonctions de Part 2
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
    console.log("Système prêt. Attente de la commande MARCHE GPS.");
});
