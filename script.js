// =================================================================
// gnss-dashboard-part1.js : Initialisation, Constantes & Utils
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
const MAX_ACC = 20; 
const GOOD_ACC_THRESHOLD = 3.0; 
const ALT_TH = -50; 
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 5000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 30000, timeout: 60000 }
};

// --- CONSTANTES ET VARIABLES KALMAN (EKF) ET PERSISTANCE ---
const Q_NOISE = 0.001; // Bruit de processus Vitesse
const Q_ALT_NOISE = 0.0005; // Bruit de processus Altitude

// Planche d'incertitude théorique: Incertitude minimale atteignable.
const MIN_UNCERT_FLOOR = Q_NOISE * MIN_DT; // 0.001 * 0.05 = 5e-5

let kSpd = 0; // Estimation vitesse (m/s)
let kUncert = 1000; // Incertitude vitesse (m/s)²
let kAlt = 0; // Estimation altitude (m)
let kAltUncert = 1000; // Incertitude altitude (m)²

// Variables de Persistance
const P_RECORDS_KEY = 'gnss_precision_records'; 
let P_RECORDS = {
    max_kUncert_min: 1000, 
    max_acc_min: 1000,     
    max_g_force_max: 0     
};

const ENVIRONMENT_FACTORS = {
    NORMAL: 1.0, FOREST: 1.5, CONCRETE: 3.0, METAL: 2.5
};

// Constantes ZVU / IMU-Only / Bruit
const MIN_SPD = 0.000000001; // Le VRAI seuil de vitesse minimale (1 nm/s)
const R_GPS_DISABLED = 100000.0; 
const VEL_NOISE_FACTOR = 4.5; 
const STATIC_ACCEL_THRESHOLD = 0.05; 
const LOW_SPEED_THRESHOLD = 1.0; 
const ACCEL_FILTER_ALPHA = 0.8; 
const ACCEL_MOVEMENT_THRESHOLD = 0.5; 
let kAccel = { x: 0, y: 0, z: 0 };
let G_STATIC_REF = { x: 0, y: 0, z: 0 };
let latestVerticalAccelIMU = 0; 
let latestLinearAccelMagnitude = 0; 

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

// --- CONSTANTES API MÉTÉO ---
const OWM_API_KEY = "VOTRE_CLE_API_OPENWEATHERMAP"; // <-- REMPLACEZ CECI
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
    R_dyn = Math.max(R_dyn, MIN_UNCERT_FLOOR); 
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

/** Calcule le nom de la phase de la Lune à partir du coefficient de phase (0.0 à 1.0) */
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

// --- FONCTIONS ASTRO UTILITAIRES ---
function getCDate() {
    if (lLocH === 0) return new Date();
    const currentLocTime = performance.now();
    const offset = currentLocTime - lLocH;
    return new Date(lServH + offset);
}

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

function toDays(date) { return (date.valueOf() / dayMs - 0.5 + J1970) - J2000; }
function solarMeanAnomaly(d) { return D2R * (356.0470 + 0.9856002585 * d); }
function eclipticLongitude(M) {
    var C = D2R * (1.9148 * Math.sin(M) + 0.0200 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M)), 
        P = D2R * 102.9377;                                                                
    return M + C + P + Math.PI;
}

function getSolarTime(date, lon) {
    if (date === null || lon === null) return { TST: 'N/A', MST: 'N/A', EOT: 'N/D', ECL_LONG: 'N/D', TST_MS: 0 };
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
// =================================================================
// gnss-dashboard-part2.js : Logique EKF, Mouvement & Persistance
// (Dépend de gnss-dashboard-part1.js)
// =================================================================

// --- PERSISTANCE DES RECORDS ---

/** Charge les records de précision depuis localStorage au démarrage. */
function loadPrecisionRecords() {
    try {
        const stored = localStorage.getItem(P_RECORDS_KEY);
        if (stored) {
            const loaded = JSON.parse(stored);
            P_RECORDS.max_kUncert_min = loaded.max_kUncert_min || 1000;
            P_RECORDS.max_acc_min = loaded.max_acc_min || 1000;
            P_RECORDS.max_g_force_max = loaded.max_g_force_max || 0;
            maxSpd = loaded.maxSpd || 0; 
        }
        
        kUncert = Math.max(kUncert, P_RECORDS.max_kUncert_min, MIN_UNCERT_FLOOR); 
        kAltUncert = Math.max(kAltUncert, MIN_UNCERT_FLOOR); 
        
        if ($('record-uncert-min')) $('record-uncert-min').textContent = P_RECORDS.max_kUncert_min.toExponential(4) + ' m²';

    } catch (e) {
        console.error("Erreur lors du chargement des records de précision:", e);
    }
}

/** Sauvegarde les records de précision dans localStorage. */
function savePrecisionRecords() {
    try {
        const recordsToSave = {
             max_kUncert_min: P_RECORDS.max_kUncert_min,
             max_acc_min: P_RECORDS.max_acc_min,
             max_g_force_max: P_RECORDS.max_g_force_max,
             maxSpd: maxSpd
        };
        localStorage.setItem(P_RECORDS_KEY, JSON.stringify(recordsToSave));
    } catch (e) {
        console.error("Erreur lors de la sauvegarde des records de précision:", e);
    }
}


// ===========================================
// FONCTIONS CAPTEURS INERTIELS (IMU)
// ===========================================

function handleDeviceMotion(event) {
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
    
    // Mise à jour du record de G-Force max
    const totalGForce = latestLinearAccelMagnitude / G_ACC;
    if (totalGForce > P_RECORDS.max_g_force_max) {
        P_RECORDS.max_g_force_max = totalGForce; 
    }

    if ($('accel-vertical-imu')) $('accel-vertical-imu').textContent = `${accel_vertical_lin.toFixed(3)} m/s²`;
    if ($('force-g-vertical')) $('force-g-vertical').textContent = `${(accel_vertical_lin / G_ACC).toFixed(2)} G`;
    // ID 'max-g-force' manquant dans index.html, mais la variable P_RECORDS est mise à jour.
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

    // --- 1. LOGIQUE DE CORRECTION AUTOMATIQUE À LA FIN DE LA DÉRIVE (Correction Théorique) ---
    const hasGPSFix = acc !== null && acc < MAX_ACC; 
    const wasDrifting = kUncert > 10.0; 

    if (wasDrifting && hasGPSFix && kUncert > P_RECORDS.max_kUncert_min * 10) { 
        kUncert = P_RECORDS.max_kUncert_min; 
        kAltUncert = MIN_UNCERT_FLOOR; 
    }

    // 2. DÉTERMINATION DU FACTEUR D'AMORTISSEMENT IMU
    let imuDampeningFactor = 1.0; 
    if (effectiveAcc <= GOOD_ACC_THRESHOLD) { imuDampeningFactor = 0.5; } 
    else if (effectiveAcc >= MAX_ACC) { imuDampeningFactor = 1.0; } 
    else {
        const range = MAX_ACC - GOOD_ACC_THRESHOLD;
        const value = effectiveAcc - GOOD_ACC_THRESHOLD;
        imuDampeningFactor = 0.5 + 0.5 * (value / range); 
    }

    // 3. Entrées de Contrôle Amorties
    let accel_control_3D = latestLinearAccelMagnitude * imuDampeningFactor;
    let accel_control_V = latestVerticalAccelIMU * imuDampeningFactor;

    // 4. VITESSE 3D INSTANTANÉE BRUTE
    let spdV_raw = 0; 
    if (lPos && lPos.kAlt_old !== undefined && dt > MIN_DT && alt !== null) { 
        spdV_raw = (kAlt - lPos.kAlt_old) / dt; 
    } 
    let spd3D_raw = Math.sqrt(spdH ** 2 + spdV_raw ** 2);

    // 5. LOGIQUE DE MISE À JOUR ZÉRO-VITESSE DYNAMIQUE (ZVU)
    const GPS_NOISE_SPEED = effectiveAcc / VEL_NOISE_FACTOR; 
    const isStaticByIMU = latestLinearAccelMagnitude < STATIC_ACCEL_THRESHOLD;
    const isStaticByDynamicSpeed = spd3D_raw < GPS_NOISE_SPEED;
    
    const isStatic = isStaticByIMU && isStaticByDynamicSpeed;
    
    let isIMUOnlyMode = false; 
    let spd3D = spd3D_raw; 

    if (isStatic) {
        spd3D = 0.0; 
        isIMUOnlyMode = true; 
        
        accel_control_3D = 0.0; 
        accel_control_V = 0.0; 
        
        kSpd = 0.0;
        kUncert = Math.min(kUncert, MIN_UNCERT_FLOOR); 
        kAltUncert = Math.min(kAltUncert, MIN_UNCERT_FLOOR);
        lastFSpeed = 0.0; 
    }
    
    // Détection du Mode Intérieur/Haute Poursuite
    const HIGH_NOISE_ACC_THRESHOLD = 10.0; 
    const isHighNoise = effectiveAcc > HIGH_NOISE_ACC_THRESHOLD || selectedEnvironment === 'CONCRETE' || selectedEnvironment === 'METAL';
    const isInterior = isStatic && isHighNoise;

    // 6. FILTRAGE DE L'ALTITUDE (via Kalman)
    const kAlt_new = kFilterAltitude(alt, effectiveAcc, dt, accel_control_V); 
    
    // 7. VITESSE VERTICALE et HORIZONTALE
    let spdV = 0; 
    if (lPos && lPos.kAlt_old !== undefined && dt > MIN_DT && alt !== null) { 
        spdV = (kAlt_new - lPos.kAlt_old) / dt; 
        if (isStatic) spdV = 0.0; 
    } 
    
    if (lPos && dt > 0.05) { 
        const dH = dist(lPos.coords.latitude, lPos.coords.longitude, lat, lon); 
        spdH = dH / dt; 
    } 
    
    // 8. FILTRE DE KALMAN FINAL (Vitesse 3D Stable)
    let R_dyn = getKalmanR(effectiveAcc, alt, lastP_hPa); 
    
    if (isIMUOnlyMode) {
        R_dyn = R_GPS_DISABLED; 
    }

    const fSpd = kFilter(spd3D, dt, R_dyn, accel_control_3D); 
    const sSpdFE = fSpd < MIN_SPD ? 0 : fSpd; 

    // --- CALCUL DES MÉTRIQUES ---
    const speedForMetrics = spd3D < MIN_SPD ? 0 : spd3D; 

    let accel_long = (dt > 0.05) ? (sSpdFE - lastFSpeed) / dt : 0;
    lastFSpeed = sSpdFE;

    distM += speedForMetrics * dt * (netherMode ? NETHER_RATIO : 1); 
    
    if (speedForMetrics > MIN_SPD) { timeMoving += dt; }

    if (speedForMetrics > maxSpd) maxSpd = speedForMetrics; 
    const avgSpdMoving = timeMoving > 0 ? (distM / timeMoving) : 0;
    
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
        if (!isNaN(tempK) && tempK > 0 && pressurePa > 0) {
            airDensity = (pressurePa / (R_specific * tempK)).toFixed(3);
        }
    }
    
    // --- MISE À JOUR DES RECORDS DE PRÉCISION ---
    if (acc !== null && acc < P_RECORDS.max_acc_min) {
        P_RECORDS.max_acc_min = acc;
    }
    if (kUncert < P_RECORDS.max_kUncert_min) {
        P_RECORDS.max_kUncert_min = kUncert;
    }


    // --- MISE À JOUR DU DOM (SCIENTIFIQUE & VITESSE HAUTE PRÉCISION) ---
    const kmh_stable = sSpdFE * KMH_MS;
    
    // Vitesse en nm/s (12 décimales pour le m/s pour garantir la résolution)
    $('speed-stable').textContent = `${kmh_stable.toFixed(12)} km/h`; 
    $('speed-stable-ms').textContent = `${sSpdFE.toFixed(12)} m/s | ${(sSpdFE * 1e9).toExponential(4)} nm/s`; 
    
    // Incertitudes EKF (Horizontal et Vertical)
    $('uncert-horizontal-ekf').textContent = `${(Math.sqrt(kUncert)).toFixed(6)} m`;
    $('uncert-vertical-ekf').textContent = `${(Math.sqrt(kAltUncert)).toFixed(6)} m`;
    $('record-uncert-min').textContent = `${P_RECORDS.max_kUncert_min.toExponential(4)} m²`; 

    // --- MISE À JOUR DU DOM (Reste) ---
    $('speed-3d-inst').textContent = `${(spd3D * KMH_MS).toFixed(5)} km/h`;
    $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    $('speed-avg-moving').textContent = `${(avgSpdMoving * KMH_MS).toFixed(5)} km/h`;
    $('perc-speed-c').textContent = `${(spd3D / C_L * 100).toExponential(2)}%`; 
    $('perc-speed-sound').textContent = `${(spd3D / SPEED_SOUND * 100).toFixed(2)} %`; 
    $('distance-total-km').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    $('distance-cosmic').textContent = `${(distM / C_L).toExponential(2)} s lumière | ${(distM / C_L / (3600*24*365.25)).toExponential(2)} al`;
    
    $('latitude').textContent = lat.toFixed(6);
    $('longitude').textContent = lon.toFixed(6);
    $('altitude-gps').textContent = kAlt_new !== null ? `${kAlt_new.toFixed(2)} m (EKF)` : 'N/A';
    $('gps-precision').textContent = `${acc.toFixed(2)} m`;
    $('gps-accuracy-effective').textContent = `${effectiveAcc.toFixed(2)} m`;
    $('speed-raw-ms').textContent = spd_raw_gps !== null ? `${spd_raw_gps.toFixed(2)} m/s` : 'N/A';
    
    $('vertical-speed').textContent = `${spdV.toFixed(2)} m/s`;
    $('accel-long').textContent = `${accel_long.toFixed(3)} m/s²`;
    $('force-g-long').textContent = `${(accel_long / G_ACC).toFixed(2)} G`;
    $('speed-error-perc').textContent = `${R_dyn.toExponential(3)} m² (R dyn)`; 
    
    $('kinetic-energy').textContent = `${kineticEnergy.toFixed(2)} J`;
    $('mechanical-power').textContent = `${mechanicalPower.toFixed(2)} W`;
    $('coriolis-force').textContent = `${coriolusForce.toExponential(2)} N`;
    
    $('air-density').textContent = airDensity + (airDensity !== "N/A" ? ' kg/m³' : '');
    
    const isSubterranean = (kAlt_new !== null && kAlt_new < ALT_TH); 
    const isHighNoise = effectiveAcc > HIGH_NOISE_ACC_THRESHOLD || selectedEnvironment === 'CONCRETE' || selectedEnvironment === 'METAL';
    const isInterior = isStatic && isHighNoise;
    
    let statusText;
    if (isSubterranean) { statusText = `🟢 ACTIF (SOUTERRAIN/IMU) | GPS Acc: ${effectiveAcc.toFixed(0)} m`; } 
    else {
        statusText = isInterior ? `🏡 INTÉRIEUR (GPS Élevé) | GPS Acc: ${effectiveAcc.toFixed(0)} m` : `🔴 GPS+EKF | GPS Acc: ${effectiveAcc.toFixed(0)} m`;
        if (isStatic) {
             const GPS_NOISE_SPEED_DISPLAY = effectiveAcc / VEL_NOISE_FACTOR; 
             statusText += ` | 🔒 ZVU ON (Seuil Dyn: ${(GPS_NOISE_SPEED_DISPLAY * KMH_MS).toFixed(2)} km/h)`;
             if (isIMUOnlyMode) { statusText = statusText.replace('GPS+EKF', 'IMU-Only/EKF'); }
        }
    }
    
    $('underground-status').textContent = isSubterranean ? `Oui (${statusText})` : `Non (${statusText})`;
    
    updateMap(lat, lon);
    updateAstro(lat, lon);
    
    lPos = pos; 
    lPos.timestamp = cTimePos; 
    lPos.kAlt_old = kAlt_new; 
    lPos.kAltUncert_old = kAltUncert; 
        }
// =================================================================
// gnss-dashboard-part3.js : Astro, Map, Controls & Initialisation Finale
// (Dépend de gnss-dashboard-part1.js et gnss-dashboard-part2.js)
// =================================================================

// --- FONCTIONS ASTRO UTILITAIRES ---
/** * Convertit un nombre de minutes depuis minuit (0 à 1440) en format HH:MM:SS. */
function formatTime(minutes) {
    if (isNaN(minutes)) return "N/D";
    let totalMinutes = minutes % 1440;
    if (totalMinutes < 0) totalMinutes += 1440;
    
    const h = Math.floor(totalMinutes / 60);
    const m = Math.floor((totalMinutes % 60) / 60);
    const s = Math.floor((totalMinutes * 60) % 60);

    const pad = (num) => num.toString().padStart(2, '0');
    return `${pad(h)}:${pad(m).padStart(2, '0')}:${pad(s).padStart(2, '0')}`;
}


// ===========================================
// FONCTIONS ASTRONOMIE (TST / SunCalc)
// ===========================================

function updateAstro(latA, lonA) {
    const cDate = getCDate(); 
    if (!cDate || isNaN(latA) || isNaN(lonA) || !window.SunCalc) { return; }

    const pos = SunCalc.getPosition(cDate, latA, lonA);
    const times = SunCalc.getTimes(cDate, latA, lonA);
    const moonIllum = SunCalc.getMoonIllumination(cDate);
    
    const sunElevationDeg = pos.altitude * R2D;
    const solarNoonLocal = times.solarNoon;
    const solarNoonUTCMs = solarNoonLocal.getTime() + (solarNoonLocal.getTimezoneOffset() * 60000);
    const solarNoonUTC = new Date(solarNoonUTCMs); 
    
    const minutesSinceMidnight = cDate.getHours() * 60 + cDate.getMinutes() + cDate.getSeconds() / 60;
    const solarNoonMinutes = solarNoonLocal.getHours() * 60 + solarNoonLocal.getMinutes() + solarNoonLocal.getSeconds() / 60;
    
    const TST_minutes = minutesSinceMidnight + (720 - solarNoonMinutes);
    const EOT_minutes = 720 - solarNoonMinutes + (cDate.getTimezoneOffset());
    
    let dayDurationH = "N/A";
    if (times.sunrise && times.sunset) {
        dayDurationH = (times.sunset.getTime() - times.sunrise.getTime()) / (1000 * 3600);
    }

    $('sun-elevation').textContent = `${sunElevationDeg.toFixed(2)} °`;
    $('tst').textContent = formatTime(TST_minutes);
    $('lsm').textContent = formatTime(TST_minutes - EOT_minutes); 
    $('noon-solar').textContent = solarNoonUTC.toTimeString().split(' ')[0] + ' UTC'; 
    $('day-duration').textContent = typeof dayDurationH === 'number' ? `${dayDurationH.toFixed(2)} h` : dayDurationH; 
    
    const sunEcliptic = SunCalc.getEclipticPosition(cDate);
    $('ecliptic-long').textContent = `${(sunEcliptic.lon * R2D).toFixed(2)} °`; 
    $('eot').textContent = `${EOT_minutes.toFixed(3)} min`; 
    
    updateMinecraftClock(TST_minutes, sunElevationDeg); 
}

/** * Met à jour l'affichage de l'horloge stylisée du Soleil. (Logique omise pour brevité) */
function updateMinecraftClock(TST_minutes, elevation) {
    // Logique de mise à jour de l'horloge...
}

// ===========================================
// FONCTIONS API MÉTÉO (Logique omise pour brevité)
// ===========================================

async function updateWeather(latA, lonA) {
    // Logique de mise à jour météo...
}


// ===========================================
// FONCTIONS CARTE ET CONTRÔLE GPS
// ===========================================

/** Initialise la carte Leaflet. */
function initMap(latA, lonA) {
    if (map) return;
    try {
        map = L.map('map-container').setView([latA, lonA], 15);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            maxZoom: 19,
            attribution: '© OpenStreetMap'
        }).addTo(map);

        marker = L.marker([latA, lonA]).addTo(map);
        tracePolyline = L.polyline([], { color: 'red' }).addTo(map);
    } catch (e) {
        if ($('map-container')) $('map-container').textContent = 'Erreur d\'initialisation de la carte.';
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
    }
}

function resumeSystem() {
    emergencyStopActive = false;
    if ($('emergency-stop-btn')) {
        $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: INACTIF 🟢";
    }
    startGPS();
}

function handleErr(err) {
    console.error(`Erreur GNSS (${err.code}): ${err.message}`);
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = `❌ ERREUR GPS`;
}

/** Simule la correction GPS théorique parfaite et initialise l'EKF/Records. */
function simulateBestCorrection() {
    // 1. Charge les records pour obtenir le meilleur plancher connu
    loadPrecisionRecords(); 
    
    // 2. Initialise l'EKF à ce meilleur plancher pour un démarrage rapide et stable.
    kUncert = P_RECORDS.max_kUncert_min; 
    kAltUncert = MIN_UNCERT_FLOOR; 

    const mockBestCorrectionPos = {
        coords: {
            latitude: lat || 0,
            longitude: lon || 0,
            altitude: kAlt, 
            accuracy: 0.00001, // Précision simulée idéale
            speed: kSpd, 
            altitudeAccuracy: 0.00001
        },
        timestamp: new Date().getTime()
    };

    updateDisp(mockBestCorrectionPos); 
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
    
    if ($('nether-toggle-btn')) {
        $('nether-toggle-btn').textContent = "Corriger EKF (Théorique)"; 
        $('nether-toggle-btn').addEventListener('click', simulateBestCorrection);
    }
    
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
            localStorage.removeItem(P_RECORDS_KEY);
            loadPrecisionRecords();
        } 
    });
    
    if ($('data-capture-btn')) $('data-capture-btn').addEventListener('click', () => {
        alert("Données actuelles capturées (logique de sauvegarde à implémenter)!");
    });
    
    // --- Écouteur pour le Mode Nuit ---
    if ($('toggle-mode-btn')) $('toggle-mode-btn').addEventListener('click', () => {
        document.body.classList.toggle('dark-mode');
        const isDarkMode = document.body.classList.contains('dark-mode');
        $('toggle-mode-btn').textContent = isDarkMode ? "☀️ Mode Jour" : "🌗 Mode Nuit";
    });

    // --- Démarrage des Capteurs IMU ---
    if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', handleDeviceMotion, true);
    } 

    // --- Initialisation du Système ---
    syncH(); 
    loadPrecisionRecords(); 
    simulateBestCorrection(); // Correction théorique pour un démarrage stable
    startGPS(); 

    // --- Intervalle lent pour les mises à jour (Astro/Temps) ---
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
        }, DOM_SLOW_UPDATE_MS); 
    }
    
    // --- Intervalle pour la mise à jour Météo (30s) ---
    if (weatherID === null) {
        weatherID = setInterval(() => {
            if (lat !== 0 && lon !== 0) updateWeather(lat, lon);
        }, WEATHER_UPDATE_MS); 
    }
    
    // PERSISTANCE (SAUVEGARDE)
    window.addEventListener('beforeunload', savePrecisionRecords);
    window.addEventListener('pagehide', savePrecisionRecords); 
});
