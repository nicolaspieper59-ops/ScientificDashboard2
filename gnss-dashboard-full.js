// =================================================================
// FICHIER JS PARTIE 1 : gnss-dashboard-part1.js (Constantes & Utilitaires)
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
const AU_M = 149597870700;

// Constantes Temps / Astro
const dayMs = 86400000;
const J1970 = 2440588; 
const J2000 = 2451545; 
const DOM_SLOW_UPDATE_MS = 1000;
const WEATHER_UPDATE_MS = 30000; 
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc"; 

// Constantes GPS
const MIN_DT = 0.05; 
const MAX_ACC = 20; 
const GOOD_ACC_THRESHOLD = 6.0; 
const ALT_TH = -50; 
// AJOUT : Correction pour l'altitude MSL (Niveau de la mer) vs WGS84 (GPS)
// Basé sur 100.11m (EKF/GPS) - 54m (Vraie Altitude) = 46.11m
const GEOID_SEPARATION_M = 46.11; 

const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 5000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 30000, timeout: 60000 }
};

// Constantes Kalman
const Q_NOISE = 0.001; 
const Q_ALT_NOISE = 0.0005; 
let kSpd = 0; 
let kUncert = 1000; 
let kAlt = 0; 
let kAltUncert = 1000; 
const ENVIRONMENT_FACTORS = {
    NORMAL: 1.0, FOREST: 1.5, CONCRETE: 3.0, METAL: 2.5
};

// Constantes ZVU / IMU (Mises à jour)
const MIN_SPD = 0.00001; 
const R_GPS_DISABLED = 100000.0; 
const VEL_NOISE_FACTOR = 2.0; 
const STATIC_ACCEL_THRESHOLD = 0.01; 
const LOW_SPEED_THRESHOLD = 1.0; 
const ACCEL_FILTER_ALPHA = 0.95; 
const ACCEL_MOVEMENT_THRESHOLD = 0.2; 
let kAccel = { x: 0, y: 0, z: 0 };
let G_STATIC_REF = { x: 0, y: 0, z: 0 };
let latestVerticalAccelIMU = 0; 
let latestLinearAccelMagnitude = 0; 
let zvuLockTime = 0; 
let isZVUActive = false; 

// Variables globales IMU Avancées
let global_roll = 0; 
let global_pitch = 0; 

// Variables globales d'état
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
let lastAirDensity = 1.225; 

// Constantes API Météo
const OWM_API_KEY = "VOTRE_CLE_API_OPENWEATHERMAP"; 
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
function kFilter(z, dt, R_dyn, u_accel = 0) {
    const predSpd = kSpd + u_accel * dt; 
    const predUncert = kUncert + Q_NOISE * dt;
    const K = predUncert / (predUncert + R_dyn);
    kSpd = predSpd + K * (z - predSpd);
    kUncert = (1 - K) * predUncert;
    return kSpd;
}

/** Calcule le bruit de mesure dynamique R */
function getKalmanR(acc, alt, pressure) { 
    let R_raw = acc * acc; 
    const envFactor = ENVIRONMENT_FACTORS[selectedEnvironment] || ENVIRONMENT_FACTORS.NORMAL;
    const MASS_PROXY = 0.05; 

    let noiseMultiplier = envFactor;
    if (alt !== null && alt < 0) { noiseMultiplier += Math.abs(alt / 100); }
    
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

/** Calcule le point de rosée (utilitaire pour la météo) */
function calculateDewPoint(tempC, humidity) {
    if (tempC === null || humidity === null) return null;
    const a = 17.27;
    const b = 237.7;
    const alpha = (a * tempC) / (b + tempC) + Math.log(humidity / 100);
    return (b * alpha) / (a - alpha);
}

/** Calcule le nom de la phase de la Lune. */
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

// ===========================================
// FONCTIONS ASTRO UTILITAIRES ET SYNCHRO
// ===========================================

/** Synchronise l'heure locale avec une source NTP (API). */
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
        console.log(`Synchronisation UTC Atomique réussie.`);
    } catch (error) {
        console.warn("Échec de la synchronisation. Utilisation de l'horloge locale.", error);
        lServH = Date.now(); 
        lLocH = performance.now();
        if ($('local-time')) $('local-time').textContent = 'N/A (SYNCHRO ÉCHOUÉE)';
    }
}

/** Obtient l'heure courante synchronisée. */
function getCDate() {
    if (lLocH === 0) return new Date();
    const currentLocTime = performance.now();
    const offset = currentLocTime - lLocH;
    return new Date(lServH + offset);
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

/** Formate un nombre de minutes en HH:MM:SS. */
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
// FICHIER JS PARTIE 2 : gnss-dashboard-part2.js (Logique Principale & UI)
// =================================================================

// ... (Nécessite toutes les constantes et utilitaires du Bloc 1)

// ===========================================
// FONCTIONS API MÉTÉO
// ===========================================

async function updateWeather(latA, lonA) {
    if (!OWM_API_KEY || OWM_API_KEY === "VOTRE_CLE_API_OPENWEATHERMAP") {
        if ($('temp-air')) $('temp-air').textContent = 'API CLÉ MANQUANTE';
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
        
        lastP_hPa = pressurehPa; 
        const dewPointC = calculateDewPoint(tempC, humidity);

        let windChillC = tempC;
        if (tempC < 10) { 
             windChillC = 13.12 + 0.6215 * tempC - 11.37 * (windSpeedMs**0.16) + 0.3965 * tempC * (windSpeedMs**0.16);
        }

        // Calcul de la densité de l'air
        const R_SPECIFIQUE_AIR = 287.05; 
        const tempK = tempC + 273.15;
        const pressurePa = pressurehPa * 100;
        const density = pressurePa / (R_SPECIFIQUE_AIR * tempK);
        lastAirDensity = density; 
        
        const directions = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSO", "SO", "OSO", "O", "ONO", "NO", "NNO"];
        const windDirection = directions[Math.round((windDeg / 22.5) + 0.5) % 16];

        if ($('temp-air')) $('temp-air').textContent = `${tempC.toFixed(1)} °C`;
        if ($('pressure')) $('pressure').textContent = `${pressurehPa.toFixed(0)} hPa | ${(pressurehPa * 0.75006).toFixed(1)} mmHg`;
        if ($('humidity')) $('humidity').textContent = `${humidity.toFixed(0)} %`;
        if ($('dew-point')) $('dew-point').textContent = dewPointC ? `${dewPointC.toFixed(1)} °C` : 'N/A';
        if ($('wind-speed-ms')) $('wind-speed-ms').textContent = `${windSpeedMs.toFixed(2)} m/s | ${(windSpeedMs * 3.6).toFixed(1)} km/h`;
        if ($('wind-direction')) $('wind-direction').textContent = `${windDirection} (${windDeg} °)`;
        if ($('temp-feels-like')) $('temp-feels-like').textContent = `${windChillC.toFixed(1)} °C`;
        if ($('visibility')) $('visibility').textContent = data.visibility ? `${(data.visibility / 1000).toFixed(1)} km` : 'N/A (API)';
        if ($('air-density')) $('air-density').textContent = `${density.toFixed(3)} kg/m³`; 

        if ($('precipitation-rate')) $('precipitation-rate').textContent = data.rain && data.rain['1h'] ? `${data.rain['1h']} mm/h` : (data.snow && data.snow['1h'] ? `${data.snow['1h']} mm/h (neige)` : '0 mm/h');
        
    } catch (error) {
        console.error("Erreur lors de la récupération des données météo:", error);
        if ($('temp-air')) $('temp-air').textContent = 'API ERREUR';
    }
}

// ===========================================
// FONCTIONS ASTRO (TST/MST/EOT)
// ===========================================

/** Calcule le Temps Solaire Vrai (TST) et l'Équation du Temps (EOT). */
function getSolarTime(date, lon) {
    if (date === null || lon === null) return { TST: 'N/A', MST: 'N/A', EOT: 'N/D', ECL_LONG: 'N/D', TST_MS: 0, DECL: 'N/D' }; 
    const d = toDays(date);
    const M = solarMeanAnomaly(d); 
    const L = eclipticLongitude(M); 
    const epsilon = D2R * (23.4393 - 0.000000356 * d); 
    
    const declination_rad = Math.asin(Math.sin(epsilon) * Math.sin(L));
    const final_declination = declination_rad * R2D; 

    let alpha = Math.atan2(Math.cos(epsilon) * Math.sin(L), Math.cos(L));
    if (alpha < 0) alpha += 2 * Math.PI; 
    
    const meanLongitude = M + D2R * 102.9377 + Math.PI;
    let eot_rad_raw = alpha - meanLongitude; 
    eot_rad_raw = eot_rad_raw % (2 * Math.PI);
    if (eot_rad_raw > Math.PI) { eot_rad_raw -= 2 * Math.PI; } else if (eot_rad_raw < -Math.PI) { eot_rad_raw += 2 * Math.PI; }
    
    const eot_min = eot_rad_raw * R2D * 4; 
    
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
    return { 
        TST: toTimeString(tst_ms), 
        TST_MS: tst_ms, 
        MST: toTimeString(mst_ms), 
        EOT: eot_min.toFixed(3), 
        ECL_LONG: final_ecl_long.toFixed(2), 
        DECL: final_declination.toFixed(2) 
    };
}

/** Met à jour les données Astro. (Lever/Coucher de soleil ajoutés) */
function updateAstro(latA, lonA) {
    const cDate = getCDate(); 
    if (!cDate || isNaN(latA) || isNaN(lonA) || !window.SunCalc) { return; }

    const { TST, TST_MS, MST, EOT, ECL_LONG, DECL } = getSolarTime(cDate, lonA); 

    const pos = SunCalc.getPosition(cDate, latA, lonA);
    const times = SunCalc.getTimes(cDate, latA, lonA);
    const moonIllum = SunCalc.getMoonIllumination(cDate);
    
    const sunElevationDeg = pos.altitude * R2D;
    const zenithAngleDeg = 90 - sunElevationDeg;

    const solarNoonLocal = times.solarNoon;
    const solarNoonUTCMs = solarNoonLocal.getTime() + (solarNoonLocal.getTimezoneOffset() * 60000);
    const solarNoonUTC = new Date(solarNoonUTCMs); 
    
    let dayDurationH = "N/A";
    if (times.sunrise && times.sunset) {
        dayDurationH = (times.sunset.getTime() - times.sunrise.getTime()) / (1000 * 3600);
    }
    
    // Formatage Lever/Coucher de soleil en UTC
    const formatUTCDate = (dateObj) => dateObj.toUTCString().match(/(\d{2}:\d{2}:\d{2})/)[0];
    const sunriseUTC = times.sunrise ? formatUTCDate(times.sunrise) : "N/A";
    const sunsetUTC = times.sunset ? formatUTCDate(times.sunset) : "N/A";

    // --- MISE À JOUR DU DOM (Astro) ---
    $('sun-elevation').textContent = `${sunElevationDeg.toFixed(2)} °`;
    $('zenith-angle').textContent = `${zenithAngleDeg.toFixed(2)} °`;
    $('tst').textContent = TST; 
    $('lsm').textContent = MST; 
    
    $('noon-solar').textContent = solarNoonUTC.toTimeString().split(' ')[0] + ' UTC'; 
    $('noon-solar-mst').textContent = '12:00:00 MST';
    
    $('day-duration').textContent = typeof dayDurationH === 'number' ? `${dayDurationH.toFixed(2)} h` : dayDurationH; 
    
    $('ecliptic-long').textContent = `${ECL_LONG} °`; 
    $('eot').textContent = `${EOT} min`;
    $('moon-phase-display').textContent = getMoonPhaseName(moonIllum.phase);
    $('sun-declination').textContent = `${DECL} °`; 
    
    // NOUVEAU : Affichage Lever/Coucher
    $('sunrise-time').textContent = `${sunriseUTC} UTC`;
    $('sunset-time').textContent = `${sunsetUTC} UTC`;
    
    updateMinecraftClock(TST_MS / 60000, sunElevationDeg, parseFloat(DECL)); 
}

/** Met à jour l'affichage de l'horloge stylisée du Soleil avec logique saisonnière. */
function updateMinecraftClock(TST_minutes, elevation, sunDeclination) {
    const $clock = $('minecraft-clock');
    if (!$clock) return;
    
    const $status = $('clock-status');
    
    if ($('time-minecraft')) {
        $('time-minecraft').textContent = formatTime(TST_minutes); 
    }
    
    $status.textContent = `TST (Élévation: ${elevation.toFixed(2)}°)`;
    
    const TST_hour = TST_minutes / 60;
    const TST_normalized = TST_hour % 24; 
    const TST_angle_deg_raw = (TST_normalized - 12) * 15;
    const TST_angle_deg = (TST_angle_deg_raw + 360) % 360; 

    // --- LOGIQUE SAISONNIÈRE ---
    const MAX_DECL = 23.44; 
    const normalizedDecl = Math.abs(sunDeclination) / MAX_DECL; 
    const sunSizeFactor = 1.0 + (normalizedDecl * 0.2); 
    const moonVerticalShift = -(sunDeclination / MAX_DECL) * 5; 

    // --- MISE À JOUR DES ÉLÉMENTS CÉLESTES ---
    let sunEl = $clock.querySelector('.sun-element');
    if (!sunEl) { sunEl = document.createElement('div'); sunEl.className = 'sun-element'; $clock.appendChild(sunEl); }
    sunEl.style.fontSize = `${1.0 * sunSizeFactor}em`; 
    sunEl.style.transform = `rotate(${TST_angle_deg}deg) translateY(-45px)`; 
    sunEl.textContent = '☀️'; 

    let moonEl = $clock.querySelector('.moon-element');
    if (!moonEl) { moonEl = document.createElement('div'); moonEl.className = 'moon-element'; $clock.appendChild(moonEl); }
    const moon_TST_angle_deg = (TST_angle_deg + 180) % 360;
    moonEl.style.transform = `rotate(${moon_TST_angle_deg}deg) translateY(${-45 + moonVerticalShift}px)`; 
    
    const moonIllum = SunCalc.getMoonIllumination(getCDate());
    const phase = moonIllum.phase;
    moonEl.textContent = phase < 0.03 || phase > 0.97 ? '🌑' : 
                         phase < 0.22 ? '🌒' :
                         phase < 0.28 ? '🌓' :
                         phase < 0.47 ? '🌔' :
                         phase < 0.53 ? '🌕' :
                         phase < 0.72 ? '🌖' :
                         phase < 0.78 ? '🌗' : '🌘';

    // Logique de couleur du ciel dynamique
    let sky_color = '#3f51b5';
    let opacity = 1; 
    if (elevation > 10) { sky_color = '#87ceeb'; opacity = 0; }
    else if (elevation > -6) { sky_color = '#ff9800'; opacity = 0.5; }
    else if (elevation > -18) { sky_color = '#00008b'; opacity = 0.8; }

    const style = document.createElement('style');
    style.innerHTML = `
        #minecraft-clock::before { 
            background-color: ${sky_color} !important; 
            opacity: ${opacity};
        }
    `;
    let oldStyle = $clock.querySelector('style');
    if (oldStyle) { $clock.removeChild(oldStyle); }
    $clock.appendChild(style);
}

// ===========================================
// FONCTIONS CAPTEURS INERTIELS (IMU) : CORRIGÉE
// ===========================================

function handleDeviceMotion(event) {
    if (emergencyStopActive) return;
    const acc = event.accelerationIncludingGravity;
    if (acc.x === null) return; 

    // --- 1. Filtrage Passe-Bas (lissage renforcé) ---
    kAccel.x = ACCEL_FILTER_ALPHA * kAccel.x + (1 - ACCEL_FILTER_ALPHA) * acc.x;
    kAccel.y = ACCEL_FILTER_ALPHA * kAccel.y + (1 - ACCEL_FILTER_ALPHA) * acc.y;
    kAccel.z = ACCEL_FILTER_ALPHA * kAccel.z + (1 - ACCEL_FILTER_ALPHA) * acc.z; 

    // --- 2. Correction de l'Inclinaison (Gravité projetée) ---
    const G_CORR_X = -G_ACC * Math.sin(global_pitch); 
    const G_CORR_Y = G_ACC * Math.sin(global_roll) * Math.cos(global_pitch); 
    const G_CORR_Z = G_ACC * Math.cos(global_roll) * Math.cos(global_pitch); 

    // --- 3. Extraction de l'Accélération Linéaire (corrigée) ---
    let accel_vertical_lin = 0.0;
    let linear_x = 0.0, linear_y = 0.0, linear_z = 0.0;

    linear_x = kAccel.x - G_CORR_X;
    linear_y = kAccel.y - G_CORR_Y;
    linear_z = kAccel.z - G_CORR_Z; 
    accel_vertical_lin = linear_z; 

    const magnitude_lin = Math.sqrt(linear_x ** 2 + linear_y ** 2 + linear_z ** 2);

    if (magnitude_lin < ACCEL_MOVEMENT_THRESHOLD) { 
        G_STATIC_REF.x = G_CORR_X;
        G_STATIC_REF.y = G_CORR_Y;
        G_STATIC_REF.z = G_CORR_Z;
        accel_vertical_lin = 0.0; 
        latestLinearAccelMagnitude = 0.0;
    } else {
        latestLinearAccelMagnitude = magnitude_lin;
    }

    latestVerticalAccelIMU = accel_vertical_lin; 
    
    if ($('accel-vertical-imu')) $('accel-vertical-imu').textContent = `${accel_vertical_lin.toFixed(3)} m/s²`;
    if ($('force-g-vertical')) $('force-g-vertical').textContent = `${(accel_vertical_lin / G_ACC).toFixed(2)} G`;
}

/** Met à jour les variables globales d'orientation (roulis et tangage) */
function handleDeviceOrientation(event) {
    if (emergencyStopActive) return;
    global_pitch = event.beta * D2R; 
    global_roll = event.gamma * D2R; 
}


// ===========================================
// FONCTION PRINCIPALE DE MISE À JOUR (GPS CALLBACK)
// ===========================================

function updateDisp(pos) {
    if (emergencyStopActive) return;
    lat = pos.coords.latitude; lon = pos.coords.longitude;
    const alt = pos.coords.altitude, acc = pos.coords.accuracy;
    const spd_raw_gps = pos.coords.speed;
    const cTimePos = pos.timestamp; 
    const now = getCDate(); 
    const MASS = 70.0; 

    if (now === null) { updateAstro(lat, lon); return; } 
    
    // CORRECTION : Réinitialise timeMoving avec sTime
    if (sTime === null) { 
        sTime = now.getTime(); 
        timeMoving = 0.0; 
    }

    // CORRECTION (NaN) : Gestion de l'imprécision GPS et initialisation de l'altitude
    if (acc > MAX_ACC) { 
        if ($('gps-precision')) $('gps-precision').textContent = `❌ ${acc.toFixed(0)} m (Trop Imprécis)`; 
        if (lPos === null) {
            lPos = pos; 
            if (alt !== null) {
                kAlt = alt; 
                lPos.kAlt_old = kAlt;
            }
        }
        return; 
    }

    let effectiveAcc = acc;
    const accOverride = parseFloat($('gps-accuracy-override').value);
    if (accOverride > 0) { effectiveAcc = accOverride; }

    let spdH = spd_raw_gps ?? 0; 
    const dt = lPos ? (cTimePos - lPos.timestamp) / 1000 : MIN_DT;

    if (lPos && lPos.kAlt_old === undefined) {
        lPos.kAlt_old = kAlt; 
    }

    // Facteur d'amortissement IMU
    let imuDampeningFactor = 1.0; 
    if (effectiveAcc <= GOOD_ACC_THRESHOLD) { imuDampeningFactor = 0.5; } 
    else if (effectiveAcc >= MAX_ACC) { imuDampeningFactor = 1.0; } 
    else {
        const range = MAX_ACC - GOOD_ACC_THRESHOLD;
        const value = effectiveAcc - GOOD_ACC_THRESHOLD;
        imuDampeningFactor = 0.5 + 0.5 * (value / range); 
    }

    let accel_control_3D = latestLinearAccelMagnitude * imuDampeningFactor;
    let accel_control_V = latestVerticalAccelIMU * imuDampeningFactor;

    // Vitesse Verticale Brute
    let spdV_raw = 0; 
    if (lPos && lPos.kAlt_old !== undefined && dt > MIN_DT && alt !== null) { 
        spdV_raw = (kAlt - lPos.kAlt_old) / dt; 
    } 
    let spd3D_raw = Math.sqrt(spdH ** 2 + spdV_raw ** 2);

    // LOGIQUE ZÉRO-VITESSE DYNAMIQUE (ZVU)
    const GPS_NOISE_SPEED = effectiveAcc / VEL_NOISE_FACTOR; 
    const isStaticByIMU = latestLinearAccelMagnitude < STATIC_ACCEL_THRESHOLD; 
    const isStaticByDynamicSpeed = spd3D_raw < GPS_NOISE_SPEED;
    
    const isStatic = isStaticByIMU && isStaticByDynamicSpeed; 
    let isIMUOnlyMode = false; 
    let spd3D = spd3D_raw; 

    if (isStatic) {
        spd3D = 0.0; 
        isIMUOnlyMode = true; 
        isZVUActive = true;
        zvuLockTime += dt;
        accel_control_3D = 0.0; accel_control_V = 0.0; 
        kSpd = 0.0; kUncert = 0.000001; lastFSpeed = 0.0; 
    } else {
        isZVUActive = false;
        zvuLockTime = 0;
    }
    
    const HIGH_NOISE_ACC_THRESHOLD = 10.0; 
    const isInterior = isStatic && (effectiveAcc > HIGH_NOISE_ACC_THRESHOLD || selectedEnvironment === 'CONCRETE' || selectedEnvironment === 'METAL');

    // Filtrage Kalman de la Vitesse et de l'Altitude
    let R_dyn = getKalmanR(effectiveAcc, alt, lastP_hPa);
    if (isIMUOnlyMode) { R_dyn = R_GPS_DISABLED; }

    const sSpdFE = kFilter(spd3D, dt, R_dyn, accel_control_3D);
    const kAlt_new = kFilterAltitude(alt, effectiveAcc, dt, accel_control_V);
    
    // NOUVEAU : Calcul de l'Altitude Géoïde (MSL)
    const kAlt_Geoid = kAlt_new - GEOID_SEPARATION_M;

    // Correction Distance : utilise spd3D_raw (vitesse brute)
    const speedForMetrics = spd3D_raw < MIN_SPD ? 0 : spd3D_raw; 

    const spd_kms = sSpdFE / 1000;
    const spd_mach = sSpdFE / SPEED_SOUND; 

    let accel_long = (dt > 0.05) ? (sSpdFE - lastFSpeed) / dt : 0;
    lastFSpeed = sSpdFE;

    distM += speedForMetrics * dt * (netherMode ? NETHER_RATIO : 1); 
    
    if (speedForMetrics > MIN_SPD) { timeMoving += dt; }
    if (speedForMetrics > maxSpd) maxSpd = speedForMetrics; 
    const avgSpdMoving = timeMoving > 0 ? (distM / timeMoving) : 0;
    
    const dist_s_light = distM / C_L;
    const dist_ua = distM / AU_M;
    
    const coriolusForce = 2 * MASS * sSpdFE * W_EARTH * Math.sin(lat * D2R); 
    const kineticEnergy = 0.5 * MASS * sSpdFE ** 2;
    const mechanicalPower = accel_long * MASS * sSpdFE; 
    
    const C_DRAG = 0.4; 
    const A_REF = 1.0; 
    const dragForce = 0.5 * lastAirDensity * sSpdFE * sSpdFE * C_DRAG * A_REF;

    // --- MISE À JOUR DU DOM (GPS/Physique) ---
    $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(5)} km/h`; 
    $('speed-stable-ms').textContent = `${sSpdFE.toFixed(3)} m/s`; 
    $('speed-stable-kms').textContent = `${spd_kms.toFixed(5)} km/s`; 
    $('speed-mach').textContent = `${spd_mach.toFixed(5)} Mach`; 
    $('speed-3d-inst').textContent = `${(spd3D_raw * KMH_MS).toFixed(5)} km/h`; 
    $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    $('speed-avg-moving').textContent = `${(avgSpdMoving * KMH_MS).toFixed(5)} km/h`;
    $('perc-speed-c').textContent = `${(sSpdFE / C_L * 100).toExponential(2)}%`; 
    $('perc-speed-sound').textContent = `${(sSpdFE / SPEED_SOUND * 100).toFixed(2)} %`; 
    
    $('distance-total-km').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    $('distance-ua').textContent = `${dist_ua.toExponential(3)} UA`; 
    $('distance-s-light').textContent = `${dist_s_light.toExponential(3)} s lumière`; 
    $('distance-j-light').textContent = `${(dist_s_light / (24 * 3600)).toExponential(3)} j lumière`; 
    $('distance-cosmic-al').textContent = `${(dist_s_light / (3600*24*365.25)).toExponential(3)} al`; 
    
    $('latitude').textContent = lat.toFixed(6);
    $('longitude').textContent = lon.toFixed(6);
    $('altitude-gps').textContent = `${kAlt_new.toFixed(2)} m (WGS84)`; // Précise WGS84
    $('altitude-geoid').textContent = `${kAlt_Geoid.toFixed(2)} m (MSL)`; // NOUVELLE ALTITUDE
    $('gps-precision').textContent = `${acc.toFixed(2)} m`;
    $('gps-accuracy-effective').textContent = `${effectiveAcc.toFixed(2)} m`;
    $('speed-raw-ms').textContent = spd_raw_gps !== null ? `${spd_raw_gps.toFixed(2)} m/s` : 'N/A';
    
    $('vertical-speed').textContent = `${spdV_raw.toFixed(2)} m/s`;
    $('accel-long').textContent = `${accel_long.toFixed(3)} m/s²`;
    $('force-g-long').textContent = `${(accel_long / G_ACC).toFixed(2)} G`;
    $('speed-error-perc').textContent = `${R_dyn.toFixed(3)} m² (R dyn)`; 
    
    $('kinetic-energy').textContent = `${kineticEnergy.toFixed(2)} J`;
    $('mechanical-power').textContent = `${mechanicalPower.toFixed(2)} W`;
    $('coriolis-force').textContent = `${coriolusForce.toExponential(2)} N`;
    $('drag-force').textContent = `${dragForce.toFixed(3)} N`; 
    
        
    // --- MISE À JOUR DU DOM (Physique - suite) ---
    // ... (Votre code précédent de mise à jour du DOM se termine ici) ...
    // Note : La nouvelle variable $('altitude-geoid') est utilisée ici pour la première fois.

    // MISE À JOUR DU STATUT DÉTAILLÉ 
    const isSubterranean = (kAlt_new !== null && kAlt_new < ALT_TH); 
    let statusText;
    
    if (isSubterranean) {
        statusText = `🟢 ACTIF (SOUTERRAIN/IMU) | GPS Acc: ${effectiveAcc.toFixed(0)} m`;
    } else {
        if (isInterior) {
            statusText = `🏡 INTÉRIEUR (GPS Élevé) | GPS Acc: ${effectiveAcc.toFixed(0)} m`;
        } else {
            // Complète la ligne de statut GPS/EKF
            statusText = `🔴 GPS+EKF | GPS Acc: ${effectiveAcc.toFixed(0)} m`; 
        }
        
        // Ajout de l'information ZVU (Zéro Vitesse) si statique
        if (isStatic) {
             const GPS_NOISE_SPEED_DISPLAY = effectiveAcc / VEL_NOISE_FACTOR; 
             statusText += ` | 🔒 ZVU ON (Seuil Dyn: ${(GPS_NOISE_SPEED_DISPLAY * KMH_MS).toFixed(2)} km/h)`;
             
             // Si le mode est IMU-Only (ZVU activé), on le reflète dans le statut
             if (isIMUOnlyMode) {
                 statusText = statusText.replace('GPS+EKF', 'IMU-Only/EKF');
             }
        }
    }
    
    // --- MISE À JOUR DU DOM (Statut) ---
    $('underground-status').textContent = isSubterranean ? `Oui (${statusText})` : `Non (${statusText})`;
    $('zvu-lock-status').textContent = isZVUActive ? `VERROUILLÉ` : `NON-VERROUILLÉ`;
    $('zvu-lock-time').textContent = `${zvuLockTime.toFixed(1)} s`;
    
    // Mise à jour de la carte et de l'astronomie
    updateMap(lat, lon);
    updateAstro(lat, lon);
    
    // SAUVEGARDE DES VALEURS POUR LA PROCHAINE ITÉRATION
    lPos = pos; 
    lPos.timestamp = cTimePos; 
    lPos.kAlt_old = kAlt_new; 
    lPos.kAltUncert_old = kAltUncert; 
}


// ===========================================
// FONCTIONS CARTE ET CONTRÔLE GPS
// ===========================================

// ... (Le reste du fichier, y compris les fonctions initMap, updateMap, 
// setGPSMode, startGPS, stopGPS, emergencyStop, resumeSystem, 
// handleErr, et le bloc document.addEventListener('DOMContentLoaded'), doit suivre ici, 
// tel que dans la dernière version fournie.)
