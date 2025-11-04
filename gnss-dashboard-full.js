// =================================================================
// FICHIER JS COMPLET : gnss-dashboard-full.js
// (Intègre Astro Millénaire, ZVU Dynamique, Mode IMU-Only)
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
const J2000 = 2451545.0; // Précision J2000.0
const DOM_SLOW_UPDATE_MS = 1000;
const WEATHER_UPDATE_MS = 30000; 
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc"; 

// CONSTANTES POUR CALCULS ASTRONOMIQUES À LONG TERME (basées sur Meeus)
const L_MEAN_COEFF = [280.4664567, 36000.76983, 0.0003032]; 
const G_MEAN_COEFF = [357.5291092, 35999.05034, -0.0001537]; 
const E_OBLIQUITY_COEFF = [23.439291, -0.0130042, -0.00000016]; 

// Constantes GPS
const MIN_DT = 0.05; 
const MIN_SPD = 0.01; 
const MAX_ACC = 20; 
const ALT_TH = -50; 
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 5000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 30000, timeout: 60000 }
};

// Constantes pour la Détection Statique Dynamique (ZVU)
const STATIC_ACCEL_THRESHOLD = 0.05; // m/s² 
const VEL_NOISE_FACTOR = 5.0; 

// Constantes Kalman (Vitesse 3D)
const Q_NOISE = 0.001; 
const R_GPS_DISABLED = 10000.0; // NOUVEAU: R très élevé pour ignorer la mesure GPS
let kSpd = 0; 
let kUncert = 1000; 
const ENVIRONMENT_FACTORS = {
    NORMAL: 1.0, 
    FOREST: 1.5, 
    CONCRETE: 3.0, 
    METAL: 5.0 
};

// Constantes Kalman (Altitude)
const Q_ALT_NOISE = 0.01; 
let kAlt = 0; 
let kAltUncert = 1000; 

// Constantes IMU (Accéléromètre)
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

// --- CONSTANTES API MÉTÉO (Placeholders) ---
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
    
    // 1. Base Noise (influence IMU/Environnement sur la précision GPS)
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

/** Filtre de Kalman 1D pour l'Altitude */
function kFilterAltitude(z, acc, dt, u_accel = 0) { 
    if (z === null) return kAlt; 

    // 1. Prediction
    const predAlt = kAlt + (0.5 * u_accel * dt * dt); 
    let predAltUncert = kAltUncert + Q_ALT_NOISE * dt;

    // 2. Mesure
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

// FONCTIONS ASTRONOMIQUES À LONG TERME (PRÉCISION MILLÉNAIRE)
/** Calcule le Jour Julien (JD) pour la date donnée. */
function dateToJD(date) {
    return date.getTime() / dayMs + J1970;
}

/** Calcule le nombre de siècles Juliens (T) écoulés depuis l'équinoxe J2000.0. */
function jdToCenturies(jd) {
    return (jd - J2000) / 36525.0; 
}

/** Calcule la valeur d'un élément astronomique à long terme en utilisant une série polynomiale. */
function calculateLongTermElement(T, coeffs) {
    let value = 0;
    if (coeffs[0] !== undefined) value += coeffs[0]; 
    if (coeffs[1] !== undefined) value += coeffs[1] * T;
    if (coeffs[2] !== undefined) value += coeffs[2] * T * T;
    return value;
}

/** Normalise un angle en degrés à l'intervalle [0, 360). */
function normalizeAngle(angle) {
    return angle - 360 * Math.floor(angle / 360);
}

/** Calcule les éléments solaires moyens avec la précision millénaire requise. */
function getSolarMeanElements(date) {
    const JD = dateToJD(date);
    const T = jdToCenturies(JD);

    let L0 = calculateLongTermElement(T, L_MEAN_COEFF);
    L0 = normalizeAngle(L0) * D2R; 

    let M = calculateLongTermElement(T, G_MEAN_COEFF);
    M = normalizeAngle(M) * D2R; 

    let e_obliquity = calculateLongTermElement(T, E_OBLIQUITY_COEFF);
    e_obliquity *= D2R; 

    return { 
        L0: L0, 
        M: M,   
        obliquity: e_obliquity,
        T: T 
    };
}


// =================================================================
// FICHIER JS PARTIE 2 : Logique d'Exécution
// =================================================================

/** Obtient l'heure courante synchronisée */
function getCDate() {
    if (lLocH === 0) return new Date();
    const currentLocTime = performance.now();
    const offset = currentLocTime - lLocH;
    return new Date(lServH + offset);
}

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

/** Calcule le Temps Solaire Vrai (TST) normalisé. */
function getSolarTime(date, lon) {
    if (date === null || lon === null) return { TST: 'N/A', MST: 'N/A', EOT: 'N/D', ECL_LONG: 'N/D' };
    
    const meanElements = getSolarMeanElements(date);
    const L0 = meanElements.L0;
    const M = meanElements.M;
    const epsilon = meanElements.obliquity;
    const T = meanElements.T;

    const C_deg = (1.9148 - 0.004817 * T * Math.cos(M) - 0.00014 * T * Math.sin(M)) * Math.sin(M)
                + (0.02 * Math.cos(2 * M) - 0.0001 * Math.sin(2 * M)) * Math.sin(2 * M);
    
    const trueLongitude = L0 + C_deg * D2R;
    
    let alpha = Math.atan2(Math.cos(epsilon) * Math.sin(trueLongitude), Math.cos(trueLongitude));
    if (alpha < 0) alpha += 2 * Math.PI; 

    const meanLongitude = L0; 

    let eot_rad_raw = alpha - meanLongitude; 
    eot_rad_raw = eot_rad_raw % (2 * Math.PI);
    if (eot_rad_raw > Math.PI) { eot_rad_raw -= 2 * Math.PI; } else if (eot_rad_raw < -Math.PI) { eot_rad_raw += 2 * Math.PI; }
    const eot_min = eot_rad_raw * 4 * R2D; 
    
    let ecl_long_deg = (trueLongitude * R2D) % 360; 
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

/** Gère les données de l'accéléromètre via DeviceMotionEvent. */
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
    
    if ($('accel-vertical-imu')) $('accel-vertical-imu').textContent = `${accel_vertical_lin.toFixed(3)} m/s²`;
    if ($('force-g-vertical')) $('force-g-vertical').textContent = `${(accel_vertical_lin / G_ACC).toFixed(2)} G`;
}

// ... (Les fonctions updateWeather, initMap, updateMap, setGPSMode, startGPS, stopGPS, emergencyStop, resumeSystem, handleErr sont omises pour la concision) ...

// ===========================================
// FONCTION PRINCIPALE DE MISE À JOUR (GPS, Kalman, Physique)
// Intègre le ZVU Dynamique et le Mode IMU-Only
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

    // 1. FILTRAGE DE L'ALTITUDE (via Kalman)
    const kAlt_new = kFilterAltitude(alt, effectiveAcc, dt, latestVerticalAccelIMU); 
    
    // ... (Calcul de spdV et spdH omis) ...
    if (lPos && dt > 0.05) { 
        const dH = dist(lPos.coords.latitude, lPos.coords.longitude, lat, lon); 
        spdH = dH / dt; 
    } 

    // 4. VITESSE 3D
    let spd3D = Math.sqrt(spdH ** 2 + spdV ** 2);

    // LOGIQUE DE MISE À JOUR ZÉRO-VITESSE DYNAMIQUE (ZVU)
    const GPS_NOISE_SPEED = effectiveAcc / VEL_NOISE_FACTOR; 
    const isStaticByIMU = latestLinearAccelMagnitude < STATIC_ACCEL_THRESHOLD;
    const isStaticByDynamicSpeed = spd3D < GPS_NOISE_SPEED;
    
    const isStatic = isStaticByIMU && isStaticByDynamicSpeed;
    
    let isIMUOnlyMode = false; // Indicateur pour l'affichage et la logique R

    if (isStatic) {
        // ZVU : La mesure est forcée à zéro.
        spd3D = 0.0;
        
        // ACTIVATION DU MODE IMU-ONLY (GPS ignoré)
        isIMUOnlyMode = true; 
    }

    // Détection du Mode Intérieur
    let isInterior = false;
    const HIGH_NOISE_ACC_THRESHOLD = 10.0; 
    const isHighNoise = effectiveAcc > HIGH_NOISE_ACC_THRESHOLD || selectedEnvironment === 'CONCRETE' || selectedEnvironment === 'METAL';

    if (isStatic && isHighNoise) {
        isInterior = true;
    }


    // 5. FILTRE DE KALMAN FINAL (Vitesse 3D Stable)
    let R_dyn = getKalmanR(effectiveAcc, alt, lastP_hPa, latestLinearAccelMagnitude); 
    
    if (isIMUOnlyMode) {
        // NOUVEAU: Si en mode ZVU/IMU-Only, on écrase R_dyn avec une valeur très haute.
        // Cela signifie: "N'aie aucune confiance en la mesure spd3D=0, mais fais confiance à ta prédiction (kSpd + acc * dt)"
        // La prédiction sera kSpd + 0 * dt, donc kSpd se stabilisera à 0.
        R_dyn = R_GPS_DISABLED; 
    }

    const fSpd = kFilter(spd3D, dt, R_dyn); 
    const sSpdFE = fSpd < MIN_SPD ? 0 : fSpd; 

    // ... (Calculs Physiques inchangés) ...
    let accel_long = (dt > 0.05) ? (sSpdFE - lastFSpeed) / dt : 0;
    lastFSpeed = sSpdFE;
    distM += sSpdFE * dt * (netherMode ? NETHER_RATIO : 1); 
    if (sSpdFE > MIN_SPD) { timeMoving += dt; }
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    
    const coriolusForce = 2 * MASS * sSpdFE * W_EARTH * Math.sin(lat * D2R);
    const kineticEnergy = 0.5 * MASS * sSpdFE ** 2;
    const mechanicalPower = accel_long * MASS * sSpdFE; 
    
    // ... (Calcul de la Densité de l'Air omis) ...

    // --- MISE À JOUR DU DOM (GPS/Physique) ---
    $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(5)} km/h`; 
    $('speed-stable-ms').textContent = `${sSpdFE.toFixed(3)} m/s | ${(sSpdFE * 1000).toFixed(0)} mm/s`; 
    // ... (Mises à jour DOM de position et précision omises) ...
    $('speed-error-perc').textContent = `${R_dyn.toFixed(3)} m² (R dyn)`; 
    
    // MISE À JOUR DU STATUT DÉTAILLÉ (SOUTERRAIN, INTÉRIEUR, ZVU/IMU-Only)
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
                 statusText = statusText.replace('GPS+EKF', 'IMU-Only/EKF').replace('(GPS Élevé)', '(IMU-Only)');
             }
        }
    }
    
    if ($('subterranean-status-display')) { 
        $('subterranean-status-display').textContent = statusText;
    }
    $('underground-status').textContent = isSubterranean ? 'Oui' : 'Non';

    
    // ... (Mise à jour Map et sauvegarde des valeurs pour la prochaine itération omises) ...
    lPos = pos; 
    lPos.timestamp = cTimePos; 
    lPos.kAlt_old = kAlt_new; 
    lPos.kAltUncert_old = kAltUncert; 
}

// ... (Le reste des fonctions et de l'initialisation est inchangé) ...

document.addEventListener('DOMContentLoaded', () => {
    
    // ... (Logique d'initialisation et d'écoute d'événements omise) ...

    if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', handleDeviceMotion, true);
    } else {
        console.warn("DeviceMotion n'est pas supporté ou activé sur cet appareil/navigateur.");
    } 

    // Les fonctions startGPS/stopGPS et les gestionnaires de boutons sont inchangés par rapport à la version précédente.
    startGPS(); 

    if (domID === null) {
        domID = setInterval(() => {
            if (lPos) updateAstro(lPos.coords.latitude, lPos.coords.longitude);
            else updateAstro(null, null); 
        }, DOM_SLOW_UPDATE_MS); 
    }
    
    if (weatherID === null) {
        weatherID = setInterval(() => {
            if (lPos) updateWeather(lPos.coords.latitude, lPos.coords.longitude);
        }, WEATHER_UPDATE_MS); 
    }
});
