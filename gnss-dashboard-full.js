// =================================================================
// FICHIER JS PARTIE 1 : gnss-dashboard-constants-kalman.js
// Contient les Constantes, Variables d'État Globales, et le Moteur EKF.
// =================================================================

const $ = (id) => document.getElementById(id);

// --- CONSTANTES GLOBALES (Physiques, GPS, Temps) ---
const C_L = 299792458; 
const SPEED_SOUND = 343; 
const G_ACC = 9.80665; 
const KMH_MS = 3.6; 
const R_E = 6371000; 
const R2D = 180 / Math.PI;
const D2R = Math.PI / 180;
const W_EARTH = 7.2921E-5; 
const NETHER_RATIO = 1 / 8; 

// Constantes Temps / API
const DOM_SLOW_UPDATE_MS = 1000;
const WEATHER_UPDATE_MS = 30000; 
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc"; 
const OWM_API_KEY = "VOTRE_CLE_API_OPENWEATHERMAP"; // <-- REMPLACEZ CECI
const OWM_API_URL = "https://api.openweathermap.org/data/2.5/weather"; 

// Constantes GPS / IMU
const MIN_DT = 0.05; 
const MAX_ACC = 20; 
const GOOD_ACC_THRESHOLD = 3.0; 
const ALT_TH = -50; 
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 5000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 30000, timeout: 60000 }
};

// Constantes Kalman
const Q_NOISE = 0.001; 
const Q_ALT_NOISE = 0.0005; 
const ENVIRONMENT_FACTORS = {
    NORMAL: 1.0, FOREST: 1.5, CONCRETE: 3.0, METAL: 2.5
};
const MIN_SPD = 0.0000000001; 
const R_GPS_DISABLED = 100000.0; 
const VEL_NOISE_FACTOR = 4.5; 
const STATIC_ACCEL_THRESHOLD = 0.05; 
const LOW_SPEED_THRESHOLD = 1.0; 
const ACCEL_FILTER_ALPHA = 0.8; 
const ACCEL_MOVEMENT_THRESHOLD = 0.5; 

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
let currentMass = 70.0; 
let latestWindSpeedMs = 0; 

// Variables Kalman (EKF)
let kSpd = 0; 
let kUncert = 1000; 
let kAlt = 0; 
let kAltUncert = 1000; 
let recordUncertMin = 1000; 
let kAccel = { x: 0, y: 0, z: 0 }; 
let G_STATIC_REF = { x: 0, y: 0, z: 0 }; 
let latestVerticalAccelIMU = 0; 
let latestLinearAccelMagnitude = 0; 

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
function kFilter(z, dt, R_dyn, u_accel = 0) {
    const predSpd = kSpd + u_accel * dt; 
    const predUncert = kUncert + Q_NOISE * dt;
    const K = predUncert / (predUncert + R_dyn);
    kSpd = predSpd + K * (z - predSpd);
    kUncert = (1 - K) * predUncert;
    if (kUncert < recordUncertMin) {
        recordUncertMin = kUncert;
    }
    return kSpd;
}

/** Calcule le bruit de mesure dynamique R */
function getKalmanR(acc, alt) { 
    let R_raw = acc * acc; 
    const envFactor = ENVIRONMENT_FACTORS[selectedEnvironment] || ENVIRONMENT_FACTORS.NORMAL;
    const MASS_PROXY = 0.05; 

    let noiseMultiplier = envFactor;
    if (alt !== null && alt < 0) {
        noiseMultiplier += Math.abs(alt / 100); 
    }
    
    const kSpd_squared = kSpd * kSpd;
    const speedFactor = 1 / (1 + MASS_PROXY * kSpd_squared); 

    let lowSpeedBoost = 1.0;
    if (kSpd > MIN_SPD && kSpd < LOW_SPEED_THRESHOLD) {
        const speed_ratio = (LOW_SPEED_THRESHOLD - kSpd) / LOW_SPEED_THRESHOLD; 
        lowSpeedBoost = 1.0 + speed_ratio * 1.0; 
    }
    
    let R_dyn = R_raw * noiseMultiplier * speedFactor * lowSpeedBoost; 
    R_dyn = Math.max(R_dyn, 0.01); 

    return R_dyn;
}

/** Filtre de Kalman 1D pour l'Altitude */
function kFilterAltitude(z, acc, dt, u_accel = 0) { 
    if (z === null) return kAlt; 

    const predAlt = kAlt + (0.5 * u_accel * dt * dt); 
    let predAltUncert = kAltUncert + Q_ALT_NOISE * dt;
    const R_alt = acc * acc * 2.0; 
    const K = predAltUncert / (predAltUncert + R_alt);
    kAlt = predAlt + K * (z - predAlt);
    kAltUncert = (1 - K) * predAltUncert;

    return kAlt;
}

/** Calcule le point de rosée */
function calculateDewPoint(tempC, humidity) {
    if (tempC === null || humidity === null) return null;
    const a = 17.27;
    const b = 237.7;
    const alpha = (a * tempC) / (b + tempC) + Math.log(humidity / 100);
    return (b * alpha) / (a - alpha);
}

/** Obtient l'heure courante synchronisée */
function getCDate() {
    if (lLocH === 0) return new Date();
    const currentLocTime = performance.now();
    const offset = currentLocTime - lLocH;
    return new Date(lServH + offset);
}

/** Formatage HH:MM:SS */
function formatTime(minutes) {
    if (isNaN(minutes)) return "N/D";
    let totalMinutes = minutes % 1440;
    if (totalMinutes < 0) totalMinutes += 1440;
    
    const h = Math.floor(totalMinutes / 60);
    const m = Math.floor(totalMinutes % 60);
    const s = Math.floor((totalMinutes * 60) % 60);

    const pad = (num) => num.toString().padStart(2, '0');
    return `${pad(h)}:${pad(m)}:${pad(s)}`;
}
// =================================================================
// FICHIER JS PARTIE 2 : gnss-dashboard-astro-weather.js
// Contient les fonctions d'Heure, Astro et Météo.
// Dépend de gnss-dashboard-constants-kalman.js.
// =================================================================

/** Synchronisation horaire par serveur (UTC/Atomique) */
async function syncH() { 
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
    } catch (error) {
        lServH = Date.now(); 
        lLocH = performance.now();
        if ($('local-time')) $('local-time').textContent = 'N/A (SYNCHRO ÉCHOUÉE)';
    }
}

function getMoonPhaseName(phase) {
    if (phase < 0.03 || phase > 0.97) return "Nouvelle Lune 🌑";
    if (phase < 0.22) return "Premier Croissant 🌒";
    if (phase < 0.28) return "Premier Quartier 🌓";
    if (phase < 0.47) return "Gibbeuse Croissante 🌔";
    if (phase < 0.53) return "Pleine Lune 🌕";
    if (phase < 0.72) return "Gibbeuse Décroissante 🌖";
    if (phase < 0.78) return "Dernier Quartier 🌗";
    return "Dernier Croissant 🌘";
}

function updateMinecraftClock(TST_minutes, elevation) {
    const $clock = $('minecraft-clock');
    if (!$clock) return;
    
    const $status = $('clock-status');
    const $tstDisplay = $('time-minecraft');
    
    $tstDisplay.textContent = formatTime(TST_minutes);
    $status.textContent = `TST (Élévation: ${elevation.toFixed(2)}°)`;
    
    const TST_hour = TST_minutes / 60;
    const TST_normalized = TST_hour % 24; 
    const TST_angle_deg_raw = (TST_normalized - 12) * 15;
    const TST_angle_deg = (TST_angle_deg_raw + 360) % 360; 

    let sunEl = $clock.querySelector('.sun-element');
    if (!sunEl) { sunEl = document.createElement('div'); sunEl.className = 'sun-element'; $clock.appendChild(sunEl); }
    sunEl.style.transform = `rotate(${TST_angle_deg}deg) translateY(-45px)`; 
    sunEl.textContent = '☀️'; 

    let moonEl = $clock.querySelector('.moon-element');
    if (!moonEl) { moonEl = document.createElement('div'); moonEl.className = 'moon-element'; $clock.appendChild(moonEl); }
    const moon_TST_angle_deg = (TST_angle_deg + 180) % 360;
    moonEl.style.transform = `rotate(${moon_TST_angle_deg}deg) translateY(-45px)`;
    
    const moonIllum = SunCalc.getMoonIllumination(getCDate());
    const phase = moonIllum.phase;
    moonEl.textContent = getMoonPhaseName(phase).split(' ')[1];

    let sky_color = '#3f51b5';
    let opacity = 1; 
    if (elevation > 10) { sky_color = '#87ceeb'; opacity = 0; }
    else if (elevation > -6) { sky_color = '#ff9800'; opacity = 0.5; }
    else if (elevation > -18) { sky_color = '#00008b'; opacity = 0.8; }

    const style = document.createElement('style');
    style.innerHTML = `#minecraft-clock::before { background-color: ${sky_color} !important; opacity: ${opacity}; }`;
    let oldStyle = $clock.querySelector('style');
    if (oldStyle) { $clock.removeChild(oldStyle); }
    $clock.appendChild(style);
}

function updateAstro(latA, lonA) {
    const cDate = getCDate(); 
    if (!cDate || isNaN(latA) || isNaN(lonA) || !window.SunCalc) { return; }

    const pos = SunCalc.getPosition(cDate, latA, lonA);
    const times = SunCalc.getTimes(cDate, latA, lonA);
    const moonIllum = SunCalc.getMoonIllumination(cDate);
    
    const sunElevationDeg = pos.altitude * R2D;
    const solarNoonLocal = times.solarNoon;
    const solarNoonUTCMs = solarNoonLocal ? solarNoonLocal.getTime() + (solarNoonLocal.getTimezoneOffset() * 60000) : null;
    const solarNoonUTC = solarNoonUTCMs ? new Date(solarNoonUTCMs) : null; 
    
    const minutesSinceMidnight = cDate.getHours() * 60 + cDate.getMinutes() + cDate.getSeconds() / 60;
    const solarNoonMinutes = solarNoonLocal ? (solarNoonLocal.getHours() * 60 + solarNoonLocal.getMinutes() + solarNoonLocal.getSeconds() / 60) : 720; 
    
    const TST_minutes = minutesSinceMidnight + (720 - solarNoonMinutes);
    const EOT_minutes = 720 - solarNoonMinutes + (cDate.getTimezoneOffset());
    
    let dayDurationH = "N/A";
    if (times.sunrise && times.sunset) {
        dayDurationH = (times.sunset.getTime() - times.sunrise.getTime()) / (1000 * 3600);
    }
    
    $('sun-elevation').textContent = `${sunElevationDeg.toFixed(2)} °`;
    $('tst').textContent = formatTime(TST_minutes);
    $('noon-solar').textContent = solarNoonUTC ? solarNoonUTC.toTimeString().split(' ')[0] + ' UTC' : 'N/D'; 
    $('day-duration').textContent = typeof dayDurationH === 'number' ? `${dayDurationH.toFixed(2)} h` : dayDurationH; 
    
    $('eot').textContent = `${EOT_minutes.toFixed(3)} min`; 
    $('moon-phase-display').textContent = getMoonPhaseName(moonIllum.phase);
    
    updateMinecraftClock(TST_minutes, sunElevationDeg); 
}

async function updateWeather(latA, lonA) {
    if (!OWM_API_KEY || OWM_API_KEY === "VOTRE_CLE_API_OPENWEATHERMAP") {
        $('temp-air').textContent = 'API CLÉ MANQUANTE';
        return;
    }
    
    if ($('temp-air')) $('temp-air').textContent = 'Chargement...';

    try {
        const url = `${OWM_API_URL}?lat=${latA.toFixed(4)}&lon=${lonA.toFixed(4)}&units=metric&appid=${OWM_API_KEY}&lang=fr`;
        const response = await fetch(url);
        if (!response.ok) throw new Error(`Weather API failed: ${response.statusText}`);
        const data = await response.json();
        
        const tempC = data.main.temp;
        const pressurehPa = data.main.pressure;
        const humidity = data.main.humidity;
        const windSpeedMs = data.wind.speed; 
        const windDeg = data.wind.deg;
        latestWindSpeedMs = windSpeedMs;
        
        lastP_hPa = pressurehPa; 
        const dewPointC = calculateDewPoint(tempC, humidity);

        let windChillC = tempC;
        if (tempC < 10) { 
             windChillC = 13.12 + 0.6215 * tempC - 11.37 * (windSpeedMs**0.16) + 0.3965 * tempC * (windSpeedMs**0.16);
        }

        const directions = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSO", "SO", "OSO", "O", "ONO", "NO", "NNO"];
        const windDirection = directions[Math.round((windDeg / 22.5) + 0.5) % 16];

        $('temp-air').textContent = `${tempC.toFixed(1)} °C`;
        $('pressure').textContent = `${pressurehPa.toFixed(0)} hPa | ${(pressurehPa * 0.75006).toFixed(1)} mmHg`;
        $('humidity').textContent = `${humidity.toFixed(0)} %`;
        $('dew-point').textContent = dewPointC ? `${dewPointC.toFixed(1)} °C` : 'N/A';
        $('wind-speed-ms').textContent = `${windSpeedMs.toFixed(2)} m/s | ${(windSpeedMs * 3.6).toFixed(1)} km/h`;
        $('wind-direction').textContent = `${windDirection} (${windDeg} °)`;
        $('temp-feels-like').textContent = `${windChillC.toFixed(1)} °C`;
        
    } catch (error) {
        $('temp-air').textContent = 'API ERREUR';
    }
}
// =================================================================
// FICHIER JS PARTIE 3 : gnss-dashboard-core-gps-imu-init.js
// Contient la logique GPS/IMU, la Carte et l'Initialisation.
// Dépend de gnss-dashboard-constants-kalman.js et gnss-dashboard-astro-weather.js.
// =================================================================

// ===========================================
// FONCTIONS CAPTEURS INERTIELS (IMU)
// ===========================================

function handleDeviceMotion(event) {
    if (emergencyStopActive) return;
    const acc = event.accelerationIncludingGravity;
    if (acc.x === null) return; 

    // Filtrage simple (Alpha-filter)
    kAccel.x = ACCEL_FILTER_ALPHA * kAccel.x + (1 - ACCEL_FILTER_ALPHA) * acc.x;
    kAccel.y = ACCEL_FILTER_ALPHA * kAccel.y + (1 - ACCEL_FILTER_ALPHA) * acc.y;
    kAccel.z = ACCEL_FILTER_ALPHA * kAccel.z + (1 - ACCEL_FILTER_ALPHA) * acc.z; 

    const kAccel_mag = Math.sqrt(kAccel.x ** 2 + kAccel.y ** 2 + kAccel.z ** 2);
    let accel_vertical_lin = 0.0;
    let linear_x = 0.0, linear_y = 0.0, linear_z = 0.0;

    // Détection de l'état statique pour calibrer la référence de gravité
    if (Math.abs(kAccel_mag - G_ACC) < ACCEL_MOVEMENT_THRESHOLD) { 
        G_STATIC_REF.x = kAccel.x;
        G_STATIC_REF.y = kAccel.y;
        G_STATIC_REF.z = kAccel.z;
        accel_vertical_lin = 0.0; 
    } else {
        // Soustraction de la gravité pour obtenir l'accélération linéaire
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

// ===========================================
// FONCTION PRINCIPALE DE MISE À JOUR (GPS/EKF)
// ===========================================

function updateDisp(pos) {
    if (emergencyStopActive) return;
    lat = pos.coords.latitude; lon = pos.coords.longitude;
    const alt = pos.coords.altitude, acc = pos.coords.accuracy;
    const spd_raw_gps = pos.coords.speed;
    const cTimePos = pos.timestamp; 
    const now = getCDate(); 
    const MASS = parseFloat($('mass-input').value) || 70.0; 
    currentMass = MASS; 

    if (now === null || acc > MAX_ACC) { 
        if ($('gps-precision')) $('gps-precision').textContent = acc > MAX_ACC ? `❌ ${acc.toFixed(0)} m (Trop Imprécis)` : 'N/A';
        if (lPos === null) lPos = pos; return; 
    }

    let effectiveAcc = parseFloat($('gps-accuracy-override').value) > 0 ? parseFloat($('gps-accuracy-override').value) : acc;

    let spdH = spd_raw_gps ?? 0; 
    const dt = lPos ? (cTimePos - lPos.timestamp) / 1000 : MIN_DT;

    // Calculs de facteurs IMU/ZVU
    let imuDampeningFactor = effectiveAcc <= GOOD_ACC_THRESHOLD ? 0.5 : (effectiveAcc >= MAX_ACC ? 1.0 : (0.5 + 0.5 * (effectiveAcc - GOOD_ACC_THRESHOLD) / (MAX_ACC - GOOD_ACC_THRESHOLD)));
    let accel_control_3D = latestLinearAccelMagnitude * imuDampeningFactor;
    let accel_control_V = latestVerticalAccelIMU * imuDampeningFactor;

    const GPS_NOISE_SPEED = effectiveAcc / VEL_NOISE_FACTOR; 
    const spd3D_raw = Math.sqrt(spdH ** 2 + (lPos && lPos.kAlt_old !== undefined && dt > MIN_DT && alt !== null ? ((kAlt - lPos.kAlt_old) / dt) ** 2 : 0));
    const isStatic = (latestLinearAccelMagnitude < STATIC_ACCEL_THRESHOLD) && (spd3D_raw < GPS_NOISE_SPEED);
    
    let isIMUOnlyMode = false; 
    let spd3D = spd3D_raw; 

    if (isStatic) {
        spd3D = 0.0; isIMUOnlyMode = true; accel_control_3D = 0.0; accel_control_V = 0.0; 
        kSpd = 0.0; kUncert = 0.000001; lastFSpeed = 0.0; 
    }
    
    const isHighNoise = effectiveAcc > 10.0 || selectedEnvironment === 'CONCRETE' || selectedEnvironment === 'METAL';
    const isInterior = isStatic && isHighNoise;

    // Filtrage Kalman
    const kAlt_new = kFilterAltitude(alt, effectiveAcc, dt, accel_control_V); 
    let spdV = (lPos && lPos.kAlt_old !== undefined && dt > MIN_DT && alt !== null) ? (kAlt_new - lPos.kAlt_old) / dt : 0;
    if (isStatic) spdV = 0.0; 
    
    if (lPos && dt > 0.05) { 
        const dH = dist(lPos.coords.latitude, lPos.coords.longitude, lat, lon); 
        spdH = dH / dt; 
    } 
    
    let R_dyn = getKalmanR(effectiveAcc, alt); 
    if (isIMUOnlyMode) R_dyn = R_GPS_DISABLED;

    const fSpd = kFilter(spd3D, dt, R_dyn, accel_control_3D); 
    const sSpdFE = fSpd < MIN_SPD ? 0 : fSpd; 

    // Métriques
    const speedForMetrics = spd3D < MIN_SPD ? 0 : spd3D; 
    let accel_long = (dt > 0.05) ? (sSpdFE - lastFSpeed) / dt : 0;
    lastFSpeed = sSpdFE;

    distM += speedForMetrics * dt * (netherMode ? NETHER_RATIO : 1); 
    if (speedForMetrics > MIN_SPD) { timeMoving += dt; }
    if (speedForMetrics > maxSpd) maxSpd = speedForMetrics; 
    
    const coriolusForce = 2 * MASS * sSpdFE * W_EARTH * Math.sin(lat * D2R); 
    const kineticEnergy = 0.5 * MASS * sSpdFE ** 2;
    const mechanicalPower = accel_long * MASS * sSpdFE; 
    
    let airDensity = "N/A";
    const tempElement = $('temp-air').textContent;
    const tempCMatch = tempElement.match(/(-?[\d.]+)\s*°C/);
    if (tempCMatch) {
        const tempC = parseFloat(tempCMatch[1]);
        const tempK = tempC + 273.15; 
        const pressurePa = lastP_hPa * 100; 
        const R_specific = 287.058; 
        if (!isNaN(tempK) && tempK > 0 && pressurePa > 0) { airDensity = (pressurePa / (R_specific * tempK)).toFixed(3); }
    }


    // --- MISE À JOUR DU DOM ---
    $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(5)} km/h`; 
    $('speed-stable-ms').textContent = `${sSpdFE.toFixed(12)} m/s | ${(sSpdFE * 1e9).toExponential(2)} nm/s`; 
    $('latitude').textContent = lat.toFixed(6);
    $('longitude').textContent = lon.toFixed(6);
    $('altitude-gps').textContent = kAlt_new !== null ? `${kAlt_new.toFixed(2)} m` : 'N/A';
    $('gps-precision').textContent = `${acc.toFixed(2)} m`;
    $('mass-display').textContent = `${MASS.toFixed(3)} kg`;
    $('accel-long').textContent = `${accel_long.toFixed(3)} m/s²`;
    $('force-g-long').textContent = `${(accel_long / G_ACC).toFixed(2)} G`;
    $('kinetic-energy').textContent = `${kineticEnergy.toFixed(2)} J`;
    $('mechanical-power').textContent = `${mechanicalPower.toFixed(2)} W`;
    $('coriolis-force').textContent = `${coriolusForce.toExponential(2)} N`;
    $('air-density').textContent = airDensity + (airDensity !== "N/A" ? ' kg/m³' : '');
    $('speed-3d-inst').textContent = `${(spd3D * KMH_MS).toFixed(5)} km/h`;
    $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    $('distance-total-km').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    $('distance-cosmic').textContent = `${(distM / C_L).toExponential(2)} s lumière | ${(distM / C_L / (3600*24*365.25)).toExponential(2)} al`;
    $('perc-speed-c').textContent = `${(spd3D / C_L * 100).toExponential(2)}%`; 
    $('perc-speed-sound').textContent = `${(spd3D / SPEED_SOUND * 100).toFixed(2)} %`; 
    $('gps-accuracy-effective').textContent = `${effectiveAcc.toFixed(2)} m`;
    $('speed-raw-ms').textContent = spd_raw_gps !== null ? `${spd_raw_gps.toFixed(2)} m/s` : 'N/A';
    $('vertical-speed').textContent = `${spdV.toFixed(2)} m/s`;
    $('speed-error-perc').textContent = `${R_dyn.toFixed(3)} m² (R dyn)`; 
    $('uncert-horizontal-ekf').textContent = `${Math.sqrt(kUncert).toFixed(4)} m`;
    $('uncert-vertical-ekf').textContent = `${Math.sqrt(kAltUncert).toFixed(4)} m`;

    const isSubterranean = (kAlt_new !== null && kAlt_new < ALT_TH); 
    let statusText = isSubterranean ? `🟢 ACTIF (SOUTERRAIN/IMU)` : isInterior ? `🏡 INTÉRIEUR (GPS Élevé)` : `🔴 GPS+EKF`;
    if (isStatic) { statusText += ` | 🔒 ZVU ON`; }
    $('underground-status').textContent = isSubterranean ? `Oui (${statusText})` : `Non (${statusText})`;

    updateMap(lat, lon);
    updateAstro(lat, lon);
    
    lPos = pos; 
    lPos.timestamp = cTimePos; 
    lPos.kAlt_old = kAlt_new; 
    lPos.kAltUncert_old = kAltUncert; 
}


// ===========================================
// FONCTIONS CARTE ET CONTRÔLE GPS
// ===========================================

function initMap(latA, lonA) {
    if (map) return;
    try {
        map = L.map('map-container').setView([latA, lonA], 15);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19, attribution: '© OpenStreetMap' }).addTo(map);
        marker = L.marker([latA, lonA]).addTo(map);
        tracePolyline = L.polyline([], { color: 'red' }).addTo(map);
    } catch (e) {
        if ($('map-container')) $('map-container').textContent = 'Erreur d\'initialisation de la carte.';
    }
}

function updateMap(latA, lonA) {
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
    
    // Démarre la météo après le début de la géolocalisation
    if (weatherID === null) {
        updateWeather(lat, lon);
        weatherID = setInterval(() => {
            if (lat !== 0 && lon !== 0) updateWeather(lat, lon);
        }, WEATHER_UPDATE_MS); 
    }
}

/** Fonction critique de démarrage (nécessite interaction utilisateur pour IMU/GPS) */
function startGPS() {
    if (wID !== null) return;
    
    const startIMU = () => {
        if (window.DeviceMotionEvent) {
            window.addEventListener('devicemotion', handleDeviceMotion, true);
        }
    };
    
    // Demande de permission DeviceMotion (iOS 13+ support)
    if (typeof DeviceMotionEvent !== 'undefined' && typeof DeviceMotionEvent.requestPermission === 'function') {
        DeviceMotionEvent.requestPermission().then(response => {
            if (response === 'granted') { startIMU(); } 
            setGPSMode(currentGPSMode);
            sTime = null; 
        }).catch(() => {
            setGPSMode(currentGPSMode);
            sTime = null;
        });
    } else {
        startIMU();
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
    if (weatherID !== null) {
        clearInterval(weatherID);
        weatherID = null;
    }
}

function emergencyStop() {
    emergencyStopActive = true;
    stopGPS(false);
    if ($('emergency-stop-btn')) { $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: ACTIF 🔴"; }
}

function resumeSystem() {
    emergencyStopActive = false;
    if ($('emergency-stop-btn')) { $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: INACTIF 🟢"; }
    startGPS();
}

function handleErr(err) {
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = `❌ ERREUR GPS`;
}


// ===========================================
// INITIALISATION DES ÉVÉNEMENTS
// ===========================================

document.addEventListener('DOMContentLoaded', () => {
    
    syncH();
    
    const $massInput = $('mass-input');
    if ($massInput) {
        $massInput.value = currentMass.toFixed(3);
        $massInput.addEventListener('change', (e) => {
            currentMass = parseFloat(e.target.value) || 70.0;
        });
    }
    
    if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', () => {
        if (emergencyStopActive) return;
        wID === null ? startGPS() : stopGPS();
    });
    
    if ($('freq-select')) $('freq-select').addEventListener('change', (e) => {
        if (emergencyStopActive) return;
        if (wID !== null) setGPSMode(e.target.value); 
    });
    
    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', () => {
        emergencyStopActive ? resumeSystem() : emergencyStop(); 
    });
    
    if ($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        netherMode = !netherMode; 
        if ($('mode-nether')) $('mode-nether').textContent = netherMode ? "ACTIVÉ (1:8) 🔥" : "DÉSACTIVÉ (1:1)"; 
    });
    
    if ($('environment-select')) {
        $('environment-select').value = selectedEnvironment;
        $('environment-select').addEventListener('change', (e) => { 
            if (emergencyStopActive) return;
            selectedEnvironment = e.target.value; 
        });
    }

    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => { distM = 0; timeMoving = 0; if ($('distance-total-km')) $('distance-total-km').textContent = "0.000 km | 0.00 m"; });
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', () => { maxSpd = 0; if ($('speed-max')) $('speed-max').textContent = "0.00000 km/h"; });
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { 
        if (confirm("Êtes-vous sûr de vouloir tout réinitialiser?")) { 
            distM = 0; maxSpd = 0; timeMoving = 0; lastFSpeed = 0;
            kSpd = 0; kUncert = 1000; kAlt = 0; kAltUncert = 1000; recordUncertMin = 1000;
            if (tracePolyline) tracePolyline.setLatLngs([]); 
        } 
    });
    
    if ($('toggle-mode-btn')) $('toggle-mode-btn').addEventListener('click', () => {
        document.body.classList.toggle('dark-mode');
        const isDarkMode = document.body.classList.contains('dark-mode');
        $('toggle-mode-btn').textContent = isDarkMode ? "☀️ Mode Jour" : "🌗 Mode Nuit";
    });

    if (domID === null) {
        domID = setInterval(() => {
            const now = getCDate();
            if (now) {
                $('local-time').textContent = now.toLocaleTimeString();
                $('date-display').textContent = now.toLocaleDateString();
                $('time-elapsed').textContent = sTime ? ((now.getTime() - sTime) / 1000).toFixed(2) + ' s' : '0.00 s';
                $('time-moving').textContent = timeMoving.toFixed(2) + ' s';
            }
            
            if (lat !== 0 && lon !== 0) updateAstro(lat, lon); 
            else updateMinecraftClock(0, -90);
        }, DOM_SLOW_UPDATE_MS); 
    }
});
