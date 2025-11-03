// =================================================================
// FICHIER JS PARTIE 1 : gnss-dashboard-part1.js (Constantes & Kalman)
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
let lPos = null; // Dernière position
let emergencyStopActive = false;
let netherMode = false;
let selectedEnvironment = 'NORMAL';
let lServH = 0; // Heure serveur
let lLocH = 0; // Temps performance local
let domID = null; 
let weatherID = null; 
let lastP_hPa = 1013.25; 

// --- CONSTANTES API MÉTÉO ---
const OWM_API_KEY = "VOTRE_CLE_API_OPENWEATHERMAP"; // <-- N'OUBLIEZ PAS DE REMPLACER VOTRE CLÉ
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

/** Calcule le bruit de mesure dynamique R basé sur la précision GPS et l'environnement */
function getKalmanR(acc, alt, pressure) {
    let R_raw = acc * acc; 
    const envFactor = ENVIRONMENT_FACTORS[selectedEnvironment] || ENVIRONMENT_FACTORS.NORMAL;
    
    let noiseMultiplier = envFactor;
    if (alt !== null && alt < 0) {
        noiseMultiplier += Math.abs(alt / 100); 
    }
    
    return R_raw * noiseMultiplier;
}

/** Filtre de Kalman 1D pour l'Altitude */
function kFilterAltitude(z, acc, dt) {
    if (z === null) return kAlt; 

    // 1. Prediction
    const predAlt = kAlt;
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
// FONCTIONS ASTRO & TEMPS
// ===========================================

/** Synchronisation horaire par serveur (UTC/Atomique) */
async function syncH() { 
    // [Logique de synchronisation NTP/WorldTimeAPI]
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
    // [Logique de calcul TST]
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

/** Met à jour les valeurs Astro et TST sur le DOM */
function updateAstro(latA, lonA) {
    // [Logique de mise à jour DOM pour l'Astro]
    const now = getCDate(); 
    if (now === null) return;
    
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
// FONCTIONS API MÉTÉO
// ===========================================

/** Récupère et met à jour les données météo via OpenWeatherMap. */
async function updateWeather(latA, lonA) {
    // [Logique de l'API Météo]
    if (!OWM_API_KEY || OWM_API_KEY === "VOTRE_CLE_API_OPENWEATHERMAP") {
        $('temp-air').textContent = 'API CLÉ MANQUANTE';
        return;
    }
    
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
// FONCTIONS CAPTEURS INERTIELS (IMU)
// ===========================================

/** Gère les données de l'accéléromètre via DeviceMotionEvent. */
function handleDeviceMotion(event) {
    // [Logique de l'IMU]
    if (emergencyStopActive) return;
    const acc = event.accelerationIncludingGravity;
    if (acc.x === null) return; 

    kAccel.x = ACCEL_FILTER_ALPHA * kAccel.x + (1 - ACCEL_FILTER_ALPHA) * acc.x;
    kAccel.y = ACCEL_FILTER_ALPHA * kAccel.y + (1 - ACCEL_FILTER_ALPHA) * acc.y;
    kAccel.z = ACCEL_FILTER_ALPHA * kAccel.z + (1 - ACCEL_FILTER_ALPHA) * acc.z; 

    const kAccel_mag = Math.sqrt(kAccel.x ** 2 + kAccel.y ** 2 + kAccel.z ** 2);
    let accel_vertical_lin = 0.0;

    if (Math.abs(kAccel_mag - G_ACC) < ACCEL_MOVEMENT_THRESHOLD) {
        G_STATIC_REF.x = kAccel.x;
        G_STATIC_REF.y = kAccel.y;
        G_STATIC_REF.z = kAccel.z;
        accel_vertical_lin = 0.0; 
    } else {
        const linear_z = kAccel.z - G_STATIC_REF.z;
        accel_vertical_lin = linear_z;
    }
    
    if ($('accel-vertical-imu')) $('accel-vertical-imu').textContent = `${accel_vertical_lin.toFixed(3)} m/s²`;
    if ($('force-g-vertical')) $('force-g-vertical').textContent = `${(accel_vertical_lin / G_ACC).toFixed(2)} G`;
}

// ===========================================
// FONCTIONS CARTE ET CONTRÔLE GPS
// ===========================================

/** Initialise la carte Leaflet. */
function initMap(latA, lonA) {
    // [Logique d'initialisation de la carte]
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
    // [Logique de mise à jour de la carte]
    if (!map || !marker || !tracePolyline) { initMap(latA, lonA); return; }
    
    const newLatLng = [latA, lonA];
    marker.setLatLng(newLatLng);
    tracePolyline.addLatLng(newLatLng);
    map.setView(newLatLng);
}

function setGPSMode(mode) {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    currentGPSMode = mode;
    wID = navigator.geolocation.watchPosition(updateDisp, handleErr, GPS_OPTS[mode]);
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = `⏸️ PAUSE GPS`;
    if ($('freq-select')) $('freq-select').value = mode; 
}

function startGPS() {
    if (wID === null) {
        if ($('freq-select')) $('freq-select').value = currentGPSMode; 
        setGPSMode(currentGPSMode);
        sTime = null; 
    }
}

function stopGPS(resetButton = true) {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
    }
    if (resetButton) {
        if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = `▶️ MARCHE GPS`;
    }
}

function emergencyStop() {
    emergencyStopActive = true;
    stopGPS(false);
    if ($('emergency-stop-btn')) {
        $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: ACTIF 🔴";
        $('emergency-stop-btn').classList.add('active');
    }
}

function resumeSystem() {
    emergencyStopActive = false;
    if ($('emergency-stop-btn')) {
        $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: INACTIF 🟢";
        $('emergency-stop-btn').classList.remove('active');
    }
    startGPS();
}

function handleErr(err) {
    console.error(`Erreur GNSS (${err.code}): ${err.message}`);
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = `❌ ERREUR GPS`;
}


// ===========================================
// FONCTION PRINCIPALE DE MISE À JOUR (GPS, Kalman, Physique)
// ===========================================

function updateDisp(pos) {
    // [Logique principale de traitement des données (Vitesse 3D, Incertitude Verticale, Coriolis)]
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

    // 1. FILTRAGE DE L'ALTITUDE (via Kalman)
    const kAlt_new = kFilterAltitude(alt, effectiveAcc, dt);
    
    // 2. VITESSE VERTICALE FILTRÉE ET INCERTITUDE (Propagation d'erreur)
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
    
    // 3. VITESSE HORIZONTALE CALCULÉE
    if (lPos && dt > 0.05) { 
        const dH = dist(lPos.coords.latitude, lPos.coords.longitude, lat, lon); 
        spdH = dH / dt; 
    } 

    // 4. VITESSE 3D
    let spd3D = Math.sqrt(spdH ** 2 + spdV ** 2);
    
    // 5. FILTRE DE KALMAN FINAL (Vitesse 3D Stable)
    const R_dyn = getKalmanR(effectiveAcc, alt, lastP_hPa); 
    const fSpd = kFilter(spd3D, dt, R_dyn); 
    const sSpdFE = fSpd < MIN_SPD ? 0 : fSpd; 

    // Calculs d'accélération et distance
    let accel_long = (dt > 0.05) ? (sSpdFE - lastFSpeed) / dt : 0;
    lastFSpeed = sSpdFE;
    distM += sSpdFE * dt * (netherMode ? NETHER_RATIO : 1); 
    if (sSpdFE > MIN_SPD) { timeMoving += dt; }
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    const avgSpdMoving = timeMoving > 0 ? (distM / timeMoving) : 0;
    
    // CALCUL DE LA FORCE DE CORIOLIS
    const coriolusForce = 2 * MASS * sSpdFE * W_EARTH * Math.sin(lat * D2R);
    
    // CALCULS DE PHYSIQUE
    const kineticEnergy = 0.5 * MASS * sSpdFE ** 2;
    const mechanicalPower = accel_long * MASS * sSpdFE; 

    // --- MISE À JOUR DU DOM (GPS/Physique) ---
    $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(5)} km/h`; 
    $('speed-3d-inst').textContent = `${(spd3D * KMH_MS).toFixed(5)} km/h`;
    $('vertical-speed').textContent = `${spdV.toFixed(2)} m/s`;
    $('vertical-speed-uncert').textContent = `${verticalSpeedUncert.toFixed(2)} m/s`; 
    $('accel-long').textContent = `${accel_long.toFixed(3)} m/s²`;
    $('force-g-long').textContent = `${(accel_long / G_ACC).toFixed(2)} G`;
    $('coriolis-force').textContent = `${coriolusForce.toExponential(2)} N`;
    
    // [Autres mises à jour DOM...]
    $('speed-stable-ms').textContent = `${sSpdFE.toFixed(3)} m/s | ${(sSpdFE * 1000).toFixed(0)} mm/s`; 
    $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    $('speed-avg-moving').textContent = `${(avgSpdMoving * KMH_MS).toFixed(5)} km/h`;
    $('perc-speed-c').textContent = `${(spd3D / C_L * 100).toExponential(2)}%`;
    $('perc-speed-sound').textContent = `${(spd3D / SPEED_SOUND * 100).toFixed(2)} %`;
    $('distance-total-km').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    $('distance-cosmic').textContent = `${(distM / C_L).toExponential(2)} s lumière | ${(distM / C_L / (3600*24*365.25)).toExponential(2)} al`;
    $('latitude').textContent = lat.toFixed(6);
    $('longitude').textContent = lon.toFixed(6);
    $('altitude-gps').textContent = kAlt_new !== null ? `${kAlt_new.toFixed(2)} m` : 'N/A';
    $('gps-precision').textContent = `${acc.toFixed(2)} m`;
    $('gps-accuracy-effective').textContent = `${effectiveAcc.toFixed(2)} m`;
    $('speed-raw-ms').textContent = spd_raw_gps !== null ? `${spd_raw_gps.toFixed(2)} m/s` : 'N/A';
    $('underground-status').textContent = (kAlt_new !== null && kAlt_new < ALT_TH) ? 'Oui' : 'Non';
    $('speed-error-perc').textContent = `${R_dyn.toFixed(3)} m² (R dyn)`; 
    $('kinetic-energy').textContent = `${kineticEnergy.toFixed(2)} J`;
    $('mechanical-power').textContent = `${mechanicalPower.toFixed(2)} W`;
    
    updateMap(lat, lon);
    
    // SAUVEGARDE DES VALEURS POUR LA PROCHAINE ITÉRATION
    lPos = pos; 
    lPos.timestamp = cTimePos; 
    lPos.kAlt_old = kAlt_new; 
    lPos.kAltUncert_old = kAltUncert; 
}


// ===========================================
// INITIALISATION DES ÉVÉNEMENTS ET INTERVALLES
// ===========================================

document.addEventListener('DOMContentLoaded', () => {
    // [Initialisation des contrôles et des écouteurs d'événements]
    if ($('environment-select')) {
        $('environment-select').value = selectedEnvironment;
        $('environment-select').addEventListener('change', (e) => { 
            if (emergencyStopActive) return;
            selectedEnvironment = e.target.value; 
        });
    }
    
    syncH(); 

    if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', () => {
        if (emergencyStopActive) return;
        wID === null ? startGPS() : stopGPS();
    });
    if ($('freq-select')) $('freq-select').addEventListener('change', (e) => {
        if (emergencyStopActive) return;
        setGPSMode(e.target.value);
    });
    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', () => {
        emergencyStopActive ? resumeSystem() : emergencyStop(); 
    });
    if ($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        netherMode = !netherMode; 
        if ($('mode-nether')) $('mode-nether').textContent = netherMode ? "ACTIVÉ (1:8) 🔥" : "DÉSACTIVÉ (1:1)"; 
    });
    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => { if (emergencyStopActive) return; distM = 0; timeMoving = 0; });
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', () => { if (emergencyStopActive) return; maxSpd = 0; });
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        if (confirm("Êtes-vous sûr de vouloir tout réinitialiser?")) { 
            distM = 0; maxSpd = 0; kSpd = 0; kUncert = 1000; timeMoving = 0; lastFSpeed = 0;
            if (tracePolyline) tracePolyline.setLatLngs([]); 
        } 
    });


    if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', handleDeviceMotion, true);
        console.log("Capteur DeviceMotion (IMU) activé.");
    } else {
        console.warn("DeviceMotion n'est pas supporté ou activé sur cet appareil/navigateur.");
    } 

    startGPS(); 

    // Intervalle lent pour les mises à jour Astro (1s)
    if (domID === null) {
        domID = setInterval(() => {
            if (lPos) updateAstro(lPos.coords.latitude, lPos.coords.longitude);
            else updateAstro(null, null); 
        }, DOM_SLOW_UPDATE_MS); 
    }
    
    // Intervalle pour la mise à jour Météo (30s)
    if (weatherID === null) {
        weatherID = setInterval(() => {
            if (lPos) updateWeather(lPos.coords.latitude, lPos.coords.longitude);
        }, WEATHER_UPDATE_MS); 
    }
});
