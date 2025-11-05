// =================================================================
// FICHIER JS PARTIE 1 : gnss-dashboard-part1.js (Constantes & Utilitaires)
// =================================================================

const $ = (id) => document.getElementById(id);

// --- CONSTANTES GLOBALES (Physiques, GPS, Temps) ---
const C_L = 299792458; // Vitesse de la lumière (m/s)
const SPEED_SOUND = 343; // Vitesse du son (m/s)
const G_ACC = 9.80665; // Gravité standard (m/s²)
const KMH_MS = 3.6; // Conversion
const R_E = 6371000; // Rayon moyen de la Terre (m)
const R2D = 180 / Math.PI; // Radians to Degrees
const D2R = Math.PI / 180; // Degrees to Radians
const W_EARTH = 7.2921E-5; // Vitesse angulaire de la Terre (rad/s)
const NETHER_RATIO = 1 / 8; 

// Constante Cosmique : Unité Astronomique (UA) en mètres
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
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 5000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 30000, timeout: 60000 }
};

// Constantes Kalman (Vitesse 3D & Altitude)
const Q_NOISE = 0.001; 
const Q_ALT_NOISE = 0.0005; 
let kSpd = 0; // Estimation vitesse (m/s)
let kUncert = 1000; // Incertitude vitesse (m/s)²
let kAlt = 0; // Estimation altitude (m)
let kAltUncert = 1000; // Incertitude altitude (m)²
const ENVIRONMENT_FACTORS = {
    NORMAL: 1.0, FOREST: 1.5, CONCRETE: 3.0, METAL: 2.5
};

// Constantes ZVU / IMU-Only / Bruit (MISES À JOUR pour sensibilité et stabilité)
const MIN_SPD = 0.00001; 
const R_GPS_DISABLED = 100000.0; 
const VEL_NOISE_FACTOR = 2.0; // Augmente le seuil ZVU dynamique pour la stabilité
const STATIC_ACCEL_THRESHOLD = 0.01; // Rend l'IMU plus sensible aux mm/s²
const LOW_SPEED_THRESHOLD = 1.0; 
const ACCEL_FILTER_ALPHA = 0.95; // Lissage fort (anti-bruit)
const ACCEL_MOVEMENT_THRESHOLD = 0.2; 
let kAccel = { x: 0, y: 0, z: 0 };
let G_STATIC_REF = { x: 0, y: 0, z: 0 };
let latestVerticalAccelIMU = 0; 
let latestLinearAccelMagnitude = 0; 
let zvuLockTime = 0; 
let isZVUActive = false; 

// --- VARIABLES GLOBALES IMU AVANCÉES (POUR CORRECTION INCLINAISON) ---
let global_roll = 0; // Angle de roulis (rad), mis à jour par DeviceOrientationEvent
let global_pitch = 0; // Angle de tangage (rad), mis à jour par DeviceOrientationEvent

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
let lServH = 0; // Heure du serveur (NTP)
let lLocH = 0; // Heure locale du navigateur lors de la dernière synchro
let domID = null; 
let weatherID = null; 
let lastP_hPa = 1013.25; 
let lastAirDensity = 1.225; 

// --- CONSTANTES API MÉTÉO ---
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

/** Filtre de Kalman 1D (Vitesse) avec entrée de contrôle Accélération IMU */
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

/** Filtre de Kalman 1D pour l'Altitude (Utilise u_accel IMU) */
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
    try {
        const response = await fetch(SERVER_TIME_ENDPOINT);
        if (!response.ok) throw new Error("Erreur synchro NTP");
        const data = await response.json();
        lServH = new Date(data.utc_datetime).getTime();
        lLocH = performance.now();
    } catch (error) {
        console.warn("Échec de la synchro NTP, utilisation de l'horloge locale.", error);
        lServH = new Date().getTime();
        lLocH = performance.now();
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

/** Met à jour la "Minecraft Clock". */
function updateMinecraftClock(tst_min, elevation, declination) {
    if (elevation < ALT_TH || isNaN(elevation)) { return; } 
    
    // Position du Soleil pour Minecraft (0.0 = minuit, 1.0 = fin de journée)
    let mc_time_ratio = (tst_min - 120) / 1440; // Début à 04:00 (6:00 dans le jeu)
    mc_time_ratio = mc_time_ratio % 1;
    if (mc_time_ratio < 0) mc_time_ratio += 1;
    
    // Logique de jour/nuit (simple)
    const time_of_day = mc_time_ratio * 24; 
    let icon = "☀️";
    if (time_of_day < 6 || time_of_day >= 18) icon = "🌙";
    
    const h = Math.floor(time_of_day);
    const m = Math.floor((time_of_day * 60) % 60);

    const pad = (num) => num.toString().padStart(2, '0');

    if ($('mc-clock')) $('mc-clock').textContent = `${icon} ${pad(h)}:${pad(m)}`;
    if ($('sun-moon-phase')) $('sun-moon-phase').textContent = `${icon} ${elevation.toFixed(2)}°`;
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
        const R_SPECIFIQUE_AIR = 287.05; // J/(kg·K)
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
    
    // Équation du Temps (EOT) : Signe correct TST - MST
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

/** Met à jour les données Astro. (Midi Solaire MST ajouté) */
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
    
    // --- MISE À JOUR DU DOM (Astro) ---
    $('sun-elevation').textContent = `${sunElevationDeg.toFixed(2)} °`;
    $('zenith-angle').textContent = `${zenithAngleDeg.toFixed(2)} °`;
    $('tst').textContent = TST; 
    $('lsm').textContent = MST; 
    
    $('noon-solar').textContent = solarNoonUTC.toTimeString().split(' ')[0] + ' UTC'; 
    $('noon-solar-mst').textContent = '12:00:00 MST'; // Midi Solaire Moyen
    
    $('day-duration').textContent = typeof dayDurationH === 'number' ? `${dayDurationH.toFixed(2)} h` : dayDurationH; 
    
    $('ecliptic-long').textContent = `${ECL_LONG} °`; 
    $('eot').textContent = `${EOT} min`; // EOT signé
    $('moon-phase-display').textContent = getMoonPhaseName(moonIllum.phase);
    $('sun-declination').textContent = `${DECL} °`; 
    
    updateMinecraftClock(TST_MS / 60000, sunElevationDeg, parseFloat(DECL)); 
}

// ===========================================
// FONCTIONS CAPTEURS INERTIELS (IMU) : CORRIGÉE
// ===========================================
// =================================================================
// BLOC 2/3 : gnss-dashboard-part2.js (Astro, IMU, updateDisp)
// =================================================================

// ... (Les fonctions getSolarTime, updateAstro, updateMinecraftClock, 
//      handleDeviceMotion, handleDeviceOrientation, updateWeather 
//      restent inchangées par rapport à la version précédente) ...

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
    
    // CORRECTION ERREUR 2 : Réinitialise timeMoving en même temps que sTime
    if (sTime === null) { 
        sTime = now.getTime(); 
        timeMoving = 0.0; // Réinitialise le temps de mouvement avec la session
    }

    // CORRECTION ERREUR 1 (NaN) : Gestion de l'imprécision GPS
    if (acc > MAX_ACC) { 
        if ($('gps-precision')) $('gps-precision').textContent = `❌ ${acc.toFixed(0)} m (Trop Imprécis)`; 
        
        // Si lPos n'a jamais été défini (premier lancement), 
        // nous devons initialiser l'altitude pour éviter NaN.
        if (lPos === null) {
            lPos = pos; 
            // Initialise kAlt avec la première mesure d'altitude valide
            if (alt !== null) {
                kAlt = alt; 
                lPos.kAlt_old = kAlt;
            }
        }
        return; // Sortie sécurisée
    }

    let effectiveAcc = acc;
    const accOverride = parseFloat($('gps-accuracy-override').value);
    if (accOverride > 0) { effectiveAcc = accOverride; }

    let spdH = spd_raw_gps ?? 0; 
    const dt = lPos ? (cTimePos - lPos.timestamp) / 1000 : MIN_DT;

    // Assure que kAlt_old est défini si lPos existe mais n'a pas encore kAlt_old
    if (lPos && lPos.kAlt_old === undefined) {
        lPos.kAlt_old = kAlt; 
    }

    // Détermination du facteur d'amortissement IMU
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

    // Filtrage Kalman de la Vitesse et de l'Altitude
    let R_dyn = getKalmanR(effectiveAcc, alt, lastP_hPa);
    if (isIMUOnlyMode) { R_dyn = R_GPS_DISABLED; }

    const sSpdFE = kFilter(spd3D, dt, R_dyn, accel_control_3D);
    const kAlt_new = kFilterAltitude(alt, effectiveAcc, dt, accel_control_V);

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
    
    // NOUVELLES DISTANCES
    const dist_s_light = distM / C_L;
    const dist_ua = distM / AU_M;
    
    const coriolusForce = 2 * MASS * sSpdFE * W_EARTH * Math.sin(lat * D2R); 
    const kineticEnergy = 0.5 * MASS * sSpdFE ** 2;
    const mechanicalPower = accel_long * MASS * sSpdFE; 
    
    const C_DRAG = 0.4; 
    const A_REF = 1.0; 
    const dragForce = 0.5 * lastAirDensity * sSpdFE * sSpdFE * C_DRAG * A_REF;

    // --- MISE À JOUR DU DOM (GPS/Physique) ---
    // (Cette section met à jour l'affichage avec les valeurs calculées)
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
    $('altitude-gps').textContent = kAlt_new !== null ? `${kAlt_new.toFixed(2)} m` : 'N/A';
    $('gps-precision').textContent = `${acc.toFixed(2)} m`;
    $('gps-accuracy-effective').textContent = `${effectiveAcc.toFixed(2)} m`;
    $('speed-raw-ms').textContent = spd_raw_gps !== null ? `${spd_raw_gps.toFixed(2)} m/s` : 'N/A';
    
    $('vertical-speed').textContent = `${spdV.toFixed(2)} m/s`;
    $('accel-long').textContent = `${accel_long.toFixed(3)} m/s²`;
    $('force-g-long').textContent = `${(accel_long / G_ACC).toFixed(2)} G`;
    $('speed-error-perc').textContent = `${R_dyn.toFixed(3)} m² (R dyn)`; 
    
    $('kinetic-energy').textContent = `${kineticEnergy.toFixed(2)} J`;
    $('mechanical-power').textContent = `${mechanicalPower.toFixed(2)} W`;
    $('coriolis-force').textContent = `${coriolusForce.toExponential(2)} N`;
    $('drag-force').textContent = `${dragForce.toFixed(3)} N`; 
    
    // MISE À JOUR DU STATUT DÉTAILLÉ 
    const isSubterranean = (kAlt_new !== null && kAlt_new < ALT_TH); 
    let statusText;
    
    if (isSubterranean) {
        statusText = `🟢 ACTIF (SOUTERRAIN/IMU) | GPS Acc: ${effectiveAcc.toFixed(0)} m`;
    } else {
        if (isInterior) {
            statusText = `🏡 INTÉRIEUR (GPS Élevé) | GPS Acc: ${effectiveAcc.toFixed(0)} m`;
        } else {
            statusText = `🔴 GPS+EKF | GPS Acc: ${effectiveAcc.toFixed(0)} m`;
        }
        
        if (isStatic) {
             const GPS_NOISE_SPEED_DISPLAY = effectiveAcc / VEL_NOISE_FACTOR; 
             statusText += ` | 🔒 ZVU ON (Seuil Dyn: ${(GPS_NOISE_SPEED_DISPLAY * KMH_MS).toFixed(2)} km/h)`;
             
             if (isIMUOnlyMode) {
                 statusText = statusText.replace('GPS+EKF', 'IMU-Only/EKF');
             }
        }
    }
    
    $('underground-status').textContent = isSubterranean ? `Oui (${statusText})` : `Non (${statusText})`;
    $('zvu-lock-status').textContent = isZVUActive ? `VERROUILLÉ` : `NON-VERROUILLÉ`;
    $('zvu-lock-time').textContent = `${zvuLockTime.toFixed(1)} s`;
    
    updateMap(lat, lon);
    updateAstro(lat, lon);
    
    // SAUVEGARDE DES VALEURS POUR LA PROCHAINE ITÉRATION
    lPos = pos; 
    lPos.timestamp = cTimePos; 
    lPos.kAlt_old = kAlt_new; 
    lPos.kAltUncert_old = kAltUncert; 
}


// ... (Le reste du fichier : updateWeather, handleDeviceMotion, handleDeviceOrientation, 
//      fonctions de Carte/GPS, et DOMContentLoaded reste identique à la version précédente) ...


// =================================================================
// BLOC 3/3 : gnss-dashboard-part2.js (Contrôles & Événements)
// =================================================================

// ... (Incluez ici les fonctions de Contrôle GPS/Carte : initMap, startGPS, emergencyStop, etc.)

// ===========================================
// INITIALISATION DES ÉVÉNEMENTS ET INTERVALLES
// ===========================================

document.addEventListener('DOMContentLoaded', () => {
    
    // --- Initialisation des contrôles et de l'état ---
    if ($('environment-select')) {
        $('environment-select').value = selectedEnvironment;
        $('environment-select').addEventListener('change', (e) => { 
            if (emergencyStopActive) return;
            selectedEnvironment = e.target.value; 
        });
    }

    // --- Écouteurs pour les boutons et contrôles ---
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

    // --- Écouteurs pour la réinitialisation des métriques ---
    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        distM = 0; timeMoving = 0; 
        if ($('distance-total-km')) $('distance-total-km').textContent = "0.000 km | 0.00 m";
    });
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        maxSpd = 0; 
        if ($('speed-max')) $('speed-max').textContent = "0.00000 km/h";
    });
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        if (confirm("Êtes-vous sûr de vouloir tout réinitialiser? (Distance, Max, Kalman)")) { 
            distM = 0; maxSpd = 0; kSpd = 0; kUncert = 1000; kAlt = 0; kAltUncert = 1000; timeMoving = 0; lastFSpeed = 0;
            if (tracePolyline) tracePolyline.setLatLngs([]); 
        } 
    });
    
    if ($('data-capture-btn')) $('data-capture-btn').addEventListener('click', () => {
        alert("Données actuelles capturées (logique de sauvegarde à implémenter)!");
    });
    
    if ($('toggle-mode-btn')) $('toggle-mode-btn').addEventListener('click', () => {
        document.body.classList.toggle('dark-mode');
        const isDarkMode = document.body.classList.contains('dark-mode');
        $('toggle-mode-btn').textContent = isDarkMode ? "☀️ Mode Jour" : "🌗 Mode Nuit";
    });

    // --- Démarrage des Capteurs IMU (Accélération) ---
    if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', handleDeviceMotion, true);
    } else {
        console.warn("DeviceMotion n'est pas supporté ou activé sur cet appareil/navigateur.");
    } 
    
    // --- Démarrage des Capteurs d'Orientation (pour correction verticale) ---
    if (window.DeviceOrientationEvent) {
        window.addEventListener('deviceorientation', handleDeviceOrientation, true);
    } else {
        console.warn("DeviceOrientation n'est pas supporté. La correction de l'accélération verticale ne sera pas appliquée.");
    }

    // --- Initialisation du Système ---
    syncH(); 
    startGPS(); 

    // --- Intervalle lent pour les mises à jour (Astro/Temps) ---
    if (domID === null) {
        domID = setInterval(() => {
            const now = getCDate();
            if (now) {
                if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString();
                if ($('date-display')) $('date-display').textContent = now.toLocaleDateString();
                if ($('time-elapsed')) $('time-elapsed').textContent = sTime ? ((now.getTime() - sTime) / 1000).toFixed(2) + ' s' : '0.00 s';
                if ($('time-moving')) $('time-moving').textContent = timeMoving.toFixed(2) + ' s';
            }
            
            if (lat !== 0 && lon !== 0) updateAstro(lat, lon); 
        }, DOM_SLOW_UPDATE_MS); 
    }
    
    // --- Intervalle pour la mise à jour Météo (30s) ---
    if (weatherID === null) {
        weatherID = setInterval(() => {
            if (lat !== 0 && lon !== 0) updateWeather(lat, lon);
        }, WEATHER_UPDATE_MS); 
    }
});
