// =================================================================
// BLOC 1/3 : gnss-dashboard-part1.js (Constantes & Variables Globales)
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
let kSpd = 0; 
let kUncert = 1000; 
let kAlt = 0; 
let kAltUncert = 1000; 
const ENVIRONMENT_FACTORS = {
    NORMAL: 1.0, FOREST: 1.5, CONCRETE: 3.0, METAL: 2.5
};

// Constantes ZVU / IMU-Only / Bruit (MISES À JOUR)
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
let global_roll = 0; // Angle de roulis (rad)
let global_pitch = 0; // Angle de tangage (rad)

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
let lastAirDensity = 1.225; 

// --- CONSTANTES API MÉTÉO ---
const OWM_API_KEY = "VOTRE_CLE_API_OPENWEATHERMAP"; 
const OWM_API_URL = "https://api.openweathermap.org/data/2.5/weather"; 

// ===========================================
// FONCTIONS UTILITAIRES ET KALMAN (INCHANGÉES)
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
// FONCTIONS ASTRO UTILITAIRES (INCHANGÉES)
// ===========================================

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
// BLOC 2/3 : gnss-dashboard-part2.js (Astro, IMU, updateDisp)
// =================================================================

// ... (Nécessite toutes les constantes et utilitaires du Bloc 1)

/** Calcule le Temps Solaire Vrai (TST) normalisé. */
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
    
    // Équation du Temps (EOT) : TST - MST = (Ascension Droite - Long. Moyenne) * 4 minutes/degré
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
    $('noon-solar-mst').textContent = '12:00:00 MST'; // NOUVEL AFFICHAGE DEMANDÉ
    $('day-duration').textContent = typeof dayDurationH === 'number' ? `${dayDurationH.toFixed(2)} h` : dayDurationH; 
    
    $('ecliptic-long').textContent = `${ECL_LONG} °`; 
    $('eot').textContent = `${EOT} min`; // EOT affiché avec le signe correct (ex: +16.416 min)
    $('moon-phase-display').textContent = getMoonPhaseName(moonIllum.phase);
    $('sun-declination').textContent = `${DECL} °`; 
    
    updateMinecraftClock(TST_MS / 60000, sunElevationDeg, parseFloat(DECL)); 
}

// ... (Inclure ici la fonction updateMinecraftClock et d'autres fonctions Astro non modifiées)

/** Fonction Principale de Mise à Jour (utilise les nouvelles constantes ZVU/IMU) */
function updateDisp(pos) {
    // ... (Logique EKF/ZVU inchangée, utilise les nouvelles constantes du Bloc 1)
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

    let spdV_raw = 0; 
    if (lPos && lPos.kAlt_old !== undefined && dt > MIN_DT && alt !== null) { 
        spdV_raw = (kAlt - lPos.kAlt_old) / dt; 
    } 
    let spd3D_raw = Math.sqrt(spdH ** 2 + spdV_raw ** 2);

    // 3. LOGIQUE ZÉRO-VITESSE DYNAMIQUE (ZVU) avec les nouveaux facteurs de stabilité
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
    
    // ... (suite de la logique updateDisp, filtrage Kalman, mise à jour DOM inchangée) ...
    // ...
}


// ===========================================
// FONCTIONS CAPTEURS INERTIELS (IMU) : CORRIGÉE
// ===========================================

function handleDeviceMotion(event) {
    if (emergencyStopActive) return;
    const acc = event.accelerationIncludingGravity;
    if (acc.x === null) return; 

    // --- 1. Filtrage Passe-Bas (lissage) ---
    kAccel.x = ACCEL_FILTER_ALPHA * kAccel.x + (1 - ACCEL_FILTER_ALPHA) * acc.x;
    kAccel.y = ACCEL_FILTER_ALPHA * kAccel.y + (1 - ACCEL_FILTER_ALPHA) * acc.y;
    kAccel.z = ACCEL_FILTER_ALPHA * kAccel.z + (1 - ACCEL_FILTER_ALPHA) * acc.z; 

    // --- 2. Correction de l'Inclinaison (Gravité projetée) ---
    // Utilise global_roll et global_pitch mis à jour par handleDeviceOrientation
    const G_CORR_X = -G_ACC * Math.sin(global_pitch); // Tangage
    const G_CORR_Y = G_ACC * Math.sin(global_roll) * Math.cos(global_pitch); // Roulis
    const G_CORR_Z = G_ACC * Math.cos(global_roll) * Math.cos(global_pitch); // Composante verticale

    // --- 3. Extraction de l'Accélération Linéaire (corrigée) ---
    let accel_vertical_lin = 0.0;
    let linear_x = 0.0, linear_y = 0.0, linear_z = 0.0;

    linear_x = kAccel.x - G_CORR_X;
    linear_y = kAccel.y - G_CORR_Y;
    linear_z = kAccel.z - G_CORR_Z; 
    accel_vertical_lin = linear_z; // Accélération verticale nette

    const magnitude_lin = Math.sqrt(linear_x ** 2 + linear_y ** 2 + linear_z ** 2);

    if (magnitude_lin < ACCEL_MOVEMENT_THRESHOLD) { 
        // Si quasi-statique, la référence est la composante de gravité corrigée
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

// ... (Inclure ici les fonctions Météo : updateWeather)
// =================================================================
// BLOC 3/3 : gnss-dashboard-part2.js (Contrôles & Événements)
// =================================================================

// ... (Inclure ici les fonctions de Contrôle GPS/Carte : syncH, initMap, startGPS, emergencyStop, etc.)

// --- GESTION DE L'ORIENTATION (POUR LA CORRECTION IMU) ---
function handleDeviceOrientation(event) {
    if (emergencyStopActive) return;
    
    // Convertit les angles Beta (tangage) et Gamma (roulis) en radians
    // Nécessaire pour la projection de la gravité dans handleDeviceMotion
    global_pitch = event.beta * D2R; 
    global_roll = event.gamma * D2R; 
}


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
    
    // --- Intervalle pour la mise à jour Météo (30s) --
    if (weatherID === null) {
        weatherID = setInterval(() => {
            if (lat !== 0 && lon !== 0) updateWeather(lat, lon);
        }, WEATHER_UPDATE_MS); 
    }
});
