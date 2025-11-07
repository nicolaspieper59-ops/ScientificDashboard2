// =================================================================
// BLOC A : CONSTANTES, ÉTAT GLOBAL, FILTRES DE KALMAN & IMU
// Dépendances : AUCUNE
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
let kSpd = 0; 
let kUncert = 1000; 
const ENVIRONMENT_FACTORS = {
    NORMAL: 1.0, FOREST: 1.5, CONCRETE: 3.0, METAL: 2.5
};

// Constantes Kalman (Altitude)
const Q_ALT_NOISE = 0.01; 
let kAlt = 0; 
let kAltUncert = 1000; 

// Constantes IMU (Accéléromètre)
const ACCEL_FILTER_ALPHA = 0.8; 
let kAccel = { x: 0, y: 0, z: 0 };
let latestVerticalAccelIMU = 0; 
let latestLinearAccelMagnitude = 0; 

// --- VARIABLES D'ÉTAT GLOBALES ---
let wID = null;
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
let lastP_hPa = 1013.25; 

// --- VARIABLES GLOBALES IMU & MAP ---
let global_pitch = 0; // Tangage (en Radians)
let global_roll = 0; // Roulis (en Radians)
const trajectoryPoints = []; 

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

/** Filtre de Kalman 1D (Vitesse) */
function kFilter(z, dt, R_dyn) {
    const predSpd = kSpd;
    const predUncert = kUncert + Q_NOISE * dt;
    const K = predUncert / (predUncert + R_dyn);
    kSpd = predSpd + K * (z - predSpd);
    kUncert = (1 - K) * predUncert;
    return kSpd;
}

/** Calcule le bruit de mesure dynamique R (Intègre IMU linéaire) */
function getKalmanR(acc, alt, pressure, linearAccelMag) { 
    let R_raw = acc * acc; 
    const envFactor = ENVIRONMENT_FACTORS[selectedEnvironment] || ENVIRONMENT_FACTORS.NORMAL;
    let noiseMultiplier = envFactor;
    if (alt !== null && alt < 0) { noiseMultiplier += Math.abs(alt / 100); }
    if (linearAccelMag > 0.5) { noiseMultiplier += Math.pow(linearAccelMag, 1.5) * 0.5; }
    return R_raw * noiseMultiplier;
}

/** Filtre de Kalman 1D pour l'Altitude (Utilise u_accel IMU) */
function kFilterAltitude(z, acc, dt, u_accel = 0) { 
    if (z === null) return kAlt; 
    
    // Correction pour l'initialisation (gérée dans updateDisp)
    if (kAlt === 0 && z !== null) kAlt = z; 
    
    const predAlt = kAlt + (0.5 * u_accel * dt * dt); 
    let predAltUncert = kAltUncert + Q_ALT_NOISE * dt;
    const R_alt = acc * acc * 2.0; 
    const K = predAltUncert / (predAltUncert + R_alt);
    kAlt = predAlt + K * (z - predAlt);
    kAltUncert = (1 - K) * predAltUncert;
    return kAlt;
}

// ===========================================
// FONCTIONS CAPTEURS INERTIELS (IMU)
// ===========================================

/** Gère les angles de l'appareil via DeviceOrientationEvent (Pitch et Roll). */
function handleDeviceOrientation(event) {
    if (emergencyStopActive) return;
    
    // Assumons beta = Pitch (Tangage), gamma = Roll (Roulis)
    const roll_deg = event.gamma || 0; 
    const pitch_deg = event.beta || 0; 

    // Mise à jour des variables globales pour la correction dans handleDeviceMotion
    global_roll = roll_deg * D2R;
    global_pitch = pitch_deg * D2R;

    // Mise à jour de l'affichage (sans la mention "Est.")
    if ($('pitch-angle')) $('pitch-angle').textContent = `${pitch_deg.toFixed(1)} °`;
    if ($('roll-angle')) $('roll-angle').textContent = `${roll_deg.toFixed(1)} °`;
}

/** Gère les données de l'accéléromètre, avec correction de gravité robuste. */
function handleDeviceMotion(event) {
    if (emergencyStopActive) return;
    const acc_g_raw = event.accelerationIncludingGravity;
    if (acc_g_raw.x === null) return; 

    // 1. ÉTALONNAGE & LISSAGE
    kAccel.x = ACCEL_FILTER_ALPHA * kAccel.x + (1 - ACCEL_FILTER_ALPHA) * acc_g_raw.x;
    kAccel.y = ACCEL_FILTER_ALPHA * kAccel.y + (1 - ACCEL_FILTER_ALPHA) * acc_g_raw.y;
    kAccel.z = ACCEL_FILTER_ALPHA * kAccel.z + (1 - ACCEL_FILTER_ALPHA) * acc_g_raw.z; 

    // FALLBACK : Si DeviceOrientationEvent n'a pas initialisé les angles, les estimer ici.
    if (global_pitch === 0 && global_roll === 0) {
        global_pitch = Math.atan2(kAccel.x, Math.sqrt(kAccel.y * kAccel.y + kAccel.z * kAccel.z));
        global_roll = Math.atan2(kAccel.y, kAccel.z);
        
        if ($('pitch-angle')) $('pitch-angle').textContent = `${(global_pitch * R2D).toFixed(1)} °`;
        if ($('roll-angle')) $('roll-angle').textContent = `${(global_roll * R2D).toFixed(1)} °`;
    }

    // 2. CORRECTION DE L'INCLINAISON (Projection de G)
    const phi = global_roll; 
    const theta = global_pitch; 
    const g_local = G_ACC; 

    const G_x_proj = g_local * Math.sin(theta);        
    const G_y_proj = -g_local * Math.sin(phi) * Math.cos(theta); 
    const G_z_proj_abs = g_local * Math.cos(phi) * Math.cos(theta);  
    
    // 3. ACCÉLÉRATION LINÉAIRE CORRIGÉE
    let acc_lin_t_x = kAccel.x - G_x_proj;
    let acc_lin_t_y = kAccel.y - G_y_proj;
    let acc_lin_t_z = 0;

    // LOGIQUE DYNAMIQUE ROBUSTE Z :
    let acc_lin_temp = kAccel.z - G_z_proj_abs; 

    if (Math.abs(acc_lin_temp) > 0.8 * G_ACC) { 
        acc_lin_t_z = kAccel.z + G_z_proj_abs; 
    } else {
        acc_lin_t_z = acc_lin_temp;
    }

    if (Math.abs(acc_lin_t_z) > 0.8 * G_ACC) {
        acc_lin_t_z = -acc_lin_t_z;
    }
    
    // CALCUL DES MAGNITUDES
    const acc_lin_horizontal = Math.sqrt(acc_lin_t_x ** 2 + acc_lin_t_y ** 2);
    latestVerticalAccelIMU = acc_lin_t_z; 
    latestLinearAccelMagnitude = Math.sqrt(acc_lin_t_x ** 2 + acc_lin_t_y ** 2 + acc_lin_t_z ** 2);
    
    // --- MISE À JOUR DU DOM (Horizontal et Vertical Corrigés) ---
    if ($('accel-horizontal-imu')) $('accel-horizontal-imu').textContent = `${acc_lin_horizontal.toFixed(3)} m/s²`;
    if ($('force-g-horizontal')) $('force-g-horizontal').textContent = `${(acc_lin_horizontal / G_ACC).toFixed(2)} G`;
    
    if ($('accel-vertical-imu')) $('accel-vertical-imu').textContent = `${acc_lin_t_z.toFixed(3)} m/s²`;
    if ($('force-g-vertical')) $('force-g-vertical').textContent = `${(acc_lin_t_z / G_ACC).toFixed(2)} G`;
    if ($('accel-linear-3d')) $('accel-linear-3d').textContent = `${latestLinearAccelMagnitude.toFixed(3)} m/s²`;
    if ($('force-g-3d-linear')) $('force-g-3d-linear').textContent = `${(latestLinearAccelMagnitude / G_ACC).toFixed(2)} G`;
}
// =================================================================
// BLOC B : BOUCLE PRINCIPALE (updateDisp), ASTRO & MÉTÉO
// Dépendances : BLOC A (Constantes, Globales, Filtres)
// =================================================================

// --- Fonctions utilitaires du Temps ---
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

/** Fonctions Astro (dépendent de SunCalc) */
function toDays(date) { return (date.valueOf() / dayMs - 0.5 + J1970) - J2000; }
function solarMeanAnomaly(d) { return D2R * (356.0470 + 0.9856002585 * d); }
function eclipticLongitude(M) {
    var C = D2R * (1.9148 * Math.sin(M) + 0.0200 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M)), 
        P = D2R * 102.9377;                                                                
    return M + C + P + Math.PI;
}

/** Calcule le Temps Solaire Vrai (TST) normalisé. */
function getSolarTime(date, lon) {
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
    // Retourne TST_MS pour l'horloge
    return { TST: toTimeString(tst_ms), TST_MS: tst_ms, MST: toTimeString(mst_ms), EOT: eot_min.toFixed(3), ECL_LONG: final_ecl_long.toFixed(2) };
}

/** Met à jour les valeurs Astro et TST sur le DOM (avec horloge Minecraft) */
function updateAstro(latA, lonA) {
    const now = getCDate(); 
    if (now === null || latA === 0 || lonA === 0) return; 
    
    const times = window.SunCalc ? SunCalc.getTimes(now, latA, lonA) : null;
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
    
    // --- MISE À JOUR DE L'HORLOGE TST MINECRAFT ---
    const tst_ms = solarTimes.TST_MS;
    const percentOfDay = tst_ms / dayMs; 
    const rotationDeg = (percentOfDay * 360) - 180; 
    
    const clockFace = $('minecraft-clock-face');
    if (clockFace) {
        clockFace.style.transform = `rotate(${rotationDeg}deg)`;
    }
    
    // Logique Saisons/Ciel
    if ($('clock-status')) {
         if (elevation_deg > 0) {
             $('clock-status').textContent = `Jour (Soleil visible)`;
         } else if (elevation_deg > -18) {
             $('clock-status').textContent = `Crépuscule / Aube`;
         } else {
             $('clock-status').textContent = `Nuit (Lune visible)`;
         }
    }
    // --- FIN MISE À JOUR HORLOGE ---

    $('time-minecraft').textContent = solarTimes.TST; 
    $('tst').textContent = solarTimes.TST;
    $('lsm').textContent = solarTimes.MST;
    $('sun-elevation').textContent = sunPos ? `${elevation_deg.toFixed(2)} °` : 'N/A';
    $('eot').textContent = solarTimes.EOT + ' min'; 
    $('ecliptic-long').textContent = solarTimes.ECL_LONG + ' °';

    if ($('date-display-astro')) $('date-display-astro').textContent = now.toLocaleDateString();

    if (times) {
        $('noon-solar').textContent = times.solarNoon.toLocaleTimeString('fr-FR', { timeZone: 'UTC' });
        const dayDurationMs = times.sunset.getTime() - times.sunrise.getTime();
        if (dayDurationMs > 0) {
            const hours = Math.floor(dayDurationMs / 3600000);
            const minutes = Math.floor((dayDurationMs % 3600000) / 60000);
            $('day-duration').textContent = `${hours}h ${minutes}m`;
        } else {
            $('day-duration').textContent = "Nuit polaire";
        }
    }
}


/** Calcule le point de rosée (utilitaire pour la météo) */
function calculateDewPoint(tempC, humidity) {
    const a = 17.27;
    const b = 237.7;
    const alpha = (a * tempC) / (b + tempC) + Math.log(humidity / 100);
    return (b * alpha) / (a - alpha);
}

/** Récupère et met à jour les données météo via OpenWeatherMap. */
async function updateWeather(latA, lonA) {
    if (!OWM_API_KEY || OWM_API_KEY === "VOTRE_CLE_API_OPENWEATHERMAP") {
        if ($('temp-air')) $('temp-air').textContent = 'API CLÉ MANQUANTE';
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
        const directions = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSO", "SO", "OSO", "O", "ONO", "NO", "NNO"];
        const windDirection = directions[Math.round((windDeg / 22.5) + 0.5) % 16];

        $('temp-air').textContent = `${tempC.toFixed(1)} °C`;
        $('pressure').textContent = `${pressurehPa.toFixed(0)} hPa | ${(pressurehPa * 0.75006).toFixed(1)} mmHg`;
        $('humidity').textContent = `${humidity.toFixed(0)} %`;
        $('dew-point').textContent = `${dewPointC.toFixed(1)} °C`;
        $('wind-speed-ms').textContent = `${windSpeedMs.toFixed(2)} m/s | ${(windSpeedMs * 3.6).toFixed(1)} km/h`;
        $('wind-direction').textContent = `${windDirection} (${windDeg} °)`;
        
    } catch (error) {
        $('temp-air').textContent = 'API ERREUR';
    }
}

/** * FONCTION PRINCIPALE DE MISE À JOUR (GPS, Kalman, Physique)
 * Appelée à chaque nouvelle position GPS
 */
function updateDisp(pos) {
    if (emergencyStopActive) return;
    lat = pos.coords.latitude; lon = pos.coords.longitude;
    const alt = pos.coords.altitude, acc = pos.coords.accuracy;
    const spd_raw_gps = pos.coords.speed;
    const cTimePos = pos.timestamp; 
    const now = getCDate(); 
    const MASS = 70.0; 

    // Initialisation du temps ou vérification des conditions
    if (now === null) { updateAstro(lat, lon); return; } 
    
    if (sTime === null) { 
        sTime = now.getTime(); 
        
        // <-- CORRECTION : Initialisation de l'Altitude EKF -->
        if (alt !== null && kAlt === 0) {
            kAlt = alt;
            kAltUncert = acc * acc * 2.0; 
        }
    }
    
    if (acc > MAX_ACC) { 
        if ($('gps-precision')) $('gps-precision').textContent = `❌ ${acc.toFixed(0)} m (Trop Imprécis)`; 
        if (lPos === null) lPos = pos; return; 
    }

    let effectiveAcc = acc;
    const accOverride = parseFloat($('gps-accuracy-override').value);
    if (accOverride > 0) { effectiveAcc = accOverride; }

    let spdH = spd_raw_gps ?? 0; 
    const dt = lPos ? (cTimePos - lPos.timestamp) / 1000 : MIN_DT;

    // 1. FILTRAGE DE L'ALTITUDE (via Kalman) - Utilise l'IMU verticale
    const kAlt_new = kFilterAltitude(alt, effectiveAcc, dt, latestVerticalAccelIMU); 
    
    // 2. VITESSE VERTICALE FILTRÉE
    let spdV = 0; 
    if (lPos && lPos.kAlt_old !== undefined && dt > MIN_DT && alt !== null) { 
        spdV = (kAlt_new - lPos.kAlt_old) / dt; 
        let verticalSpeedUncert = Math.sqrt(kAltUncert ** 2 + lPos.kAltUncert_old ** 2) / dt;
        if ($('vertical-speed-uncert')) $('vertical-speed-uncert').textContent = `${Math.min(20, verticalSpeedUncert).toFixed(2)} m/s`;
    } 
    
    // 3. VITESSE HORIZONTALE CALCULÉE
    if (lPos && dt > 0.05) { 
        const dH = dist(lPos.coords.latitude, lPos.coords.longitude, lat, lon); 
        spdH = dH / dt; 
    } 

    // 4. VITESSE 3D
    let spd3D = Math.sqrt(spdH ** 2 + spdV ** 2);

    // 5. FILTRE DE KALMAN FINAL (Vitesse 3D Stable) - Utilise l'IMU linéaire pour R_dyn
    const R_dyn = getKalmanR(effectiveAcc, alt, lastP_hPa, latestLinearAccelMagnitude); 
    const fSpd = kFilter(spd3D, dt, R_dyn); 
    const sSpdFE = fSpd < MIN_SPD ? 0 : fSpd; 

    // Calculs Physiques
    let accel_long = (dt > 0.05) ? (sSpdFE - lastFSpeed) / dt : 0;
    lastFSpeed = sSpdFE;
    distM += sSpdFE * dt * (netherMode ? NETHER_RATIO : 1); 
    if (sSpdFE > MIN_SPD) { timeMoving += dt; }
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    const avgSpdMoving = timeMoving > 0 ? (distM / timeMoving) : 0;
    
    const coriolusForce = 2 * MASS * sSpdFE * W_EARTH * Math.sin(lat * D2R);
    const kineticEnergy = 0.5 * MASS * sSpdFE ** 2;
    const mechanicalPower = accel_long * MASS * sSpdFE; 
    
    // DENSITÉ DE L'AIR
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

    // --- MISE À JOUR DU DOM (GPS/Physique) ---
    $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(5)} km/h`; 
    $('speed-stable-ms').textContent = `${sSpdFE.toFixed(3)} m/s | ${(sSpdFE * 1000).toFixed(0)} mm/s`; 
    $('speed-3d-inst').textContent = `${(spd3D * KMH_MS).toFixed(5)} km/h`;
    $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    $('speed-avg-moving').textContent = `${(avgSpdMoving * KMH_MS).toFixed(5)} km/h`;
    $('perc-speed-c').textContent = `${(spd3D / C_L * 100).toExponential(2)}%`;
    $('perc-speed-sound').textContent = `${(spd3D / SPEED_SOUND * 100).toFixed(2)} %`;
    $('distance-total-km').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    
    $('latitude').textContent = lat.toFixed(6);
    $('longitude').textContent = lon.toFixed(6);
    if ($('altitude-ekf')) $('altitude-ekf').textContent = kAlt_new !== null ? `${kAlt_new.toFixed(2)} m` : 'N/A';
    $('altitude-gps').textContent = pos.coords.altitude !== null ? `${pos.coords.altitude.toFixed(2)} m` : 'N/A';
    $('gps-precision').textContent = `${acc.toFixed(2)} m`;
    $('gps-accuracy-effective').textContent = `${effectiveAcc.toFixed(2)} m`;
    $('speed-raw-ms').textContent = spd_raw_gps !== null ? `${spd_raw_gps.toFixed(2)} m/s` : 'N/A';
    $('underground-status').textContent = (kAlt_new !== null && kAlt_new < ALT_TH) ? 'Oui' : 'Non';
    
    $('vertical-speed').textContent = `${spdV.toFixed(2)} m/s`;
    $('accel-long').textContent = `${accel_long.toFixed(3)} m/s²`;
    $('force-g-long').textContent = `${(accel_long / G_ACC).toFixed(2)} G`;
    $('speed-error-perc').textContent = `${R_dyn.toFixed(3)} m² (R dyn)`; 
    
    $('kinetic-energy').textContent = `${kineticEnergy.toFixed(2)} J`;
    $('mechanical-power').textContent = `${mechanicalPower.toFixed(2)} W`;
    $('coriolis-force').textContent = `${coriolusForce.toExponential(2)} N`;
    $('air-density').textContent = airDensity + (airDensity !== "N/A" ? ' kg/m³' : '');
    
    // Appel de la fonction de la carte (BLOC C)
    if (typeof updateMap === 'function') {
        updateMap(lat, lon);
    }
    
    // SAUVEGARDE DES VALEURS POUR LA PROCHAINE ITÉRATION
    lPos = pos; 
    lPos.timestamp = cTimePos; 
    lPos.kAlt_old = kAlt_new; 
    lPos.kAltUncert_old = kAltUncert; 
        }
// =================================================================
// BLOC C : CARTE (Leaflet), CONTRÔLES & GESTIONNAIRES D'ÉVÉNEMENTS
// Dépendances : BLOC A (Globales, Constantes), BLOC B (updateDisp, syncH, updateAstro, updateWeather)
// =================================================================

let map = null;
let marker = null;
let tracePolyline = null;
let domID = null; // Interval ID pour les mises à jour DOM (Astro)
let weatherID = null; // Interval ID pour les mises à jour Météo

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

        marker = L.circle([latA, lonA], {
            color: '#ffc107',
            fillColor: '#ffc107',
            fillOpacity: 0.5,
            radius: kUncert / 2.0 
        }).addTo(map);
        tracePolyline = L.polyline([], { color: '#007bff', weight: 3 }).addTo(map);
    } catch (e) {
        if ($('map-container')) $('map-container').textContent = 'Erreur d\'initialisation de la carte.';
    }
}

/** Met à jour la carte. (Appelée par updateDisp dans BLOC B) */
function updateMap(latA, lonA) {
    if (!map || !marker || !tracePolyline) { 
        // Initialiser la carte avec la première position GPS
        if (latA !== 0 || lonA !== 0) initMap(latA, lonA); 
        return; 
    }
    
    const newLatLng = [latA, lonA];
    trajectoryPoints.push(newLatLng);
    tracePolyline.setLatLngs(trajectoryPoints);
    marker.setLatLng(newLatLng);
    marker.setRadius(kUncert / 2.0);
    
    if (!map.getBounds().contains(newLatLng)) {
        map.panTo(newLatLng);
    }
}

/** Réinitialise le tracé de la trajectoire. */
function resetTrajectory() {
    trajectoryPoints.length = 0; 
    if (tracePolyline) {
        tracePolyline.setLatLngs([]); 
    }
    if (map && marker) {
        map.setView(marker.getLatLng(), 17);
    }
}

function handleErr(err) {
    console.error(`Erreur GNSS (${err.code}): ${err.message}`);
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = `❌ ERREUR GPS`;
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


// ===========================================
// INITIALISATION DES ÉVÉNEMENTS
// ===========================================
    
document.addEventListener('DOMContentLoaded', () => {
    
    syncH(); // Synchronisation horaire

    // Gestionnaire du Facteur Kalman Environnement
    if ($('environment-select')) {
        $('environment-select').value = selectedEnvironment;
        $('environment-select').addEventListener('change', (e) => { 
            if (emergencyStopActive) return;
            selectedEnvironment = e.target.value; 
        });
    }

    // Démarrage/Arrêt GPS
    if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', () => {
        if (emergencyStopActive) return;
        wID === null ? startGPS() : stopGPS();
    });
    
    // Sélecteur de Fréquence GPS
    if ($('freq-select')) $('freq-select').addEventListener('change', (e) => {
        if (emergencyStopActive) return;
        setGPSMode(e.target.value);
    });
    
    // Arrêt d'Urgence
    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', () => {
        emergencyStopActive ? resumeSystem() : emergencyStop(); 
    });
    
    // Mode Nether
    if ($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        netherMode = !netherMode; 
        if ($('mode-nether')) $('mode-nether').textContent = netherMode ? "ACTIVÉ (1:8) 🔥" : "DÉSACTIVÉ (1:1)"; 
    });

    // Boutons de Réinitialisation
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
        if (confirm("Êtes-vous sûr de vouloir tout réinitialiser? (Distance, Max, Kalman, Trajectoire)")) { 
            distM = 0; maxSpd = 0; kSpd = 0; kUncert = 1000; timeMoving = 0; lastFSpeed = 0;
            resetTrajectory(); 
        } 
    });
    
    // Gestionnaire du Mode Nuit (toggle-mode-btn)
    if ($('toggle-mode-btn')) $('toggle-mode-btn').addEventListener('click', () => {
        document.body.classList.toggle('dark-mode');
        const isDarkMode = document.body.classList.contains('dark-mode');
        $('toggle-mode-btn').textContent = isDarkMode ? "☀️ Mode Jour" : "🌗 Mode Nuit";
    });
    
    // Événements IMU et Orientation (attachés ici)
    if (window.DeviceOrientationEvent) {
        window.addEventListener('deviceorientation', handleDeviceOrientation, true);
    } else {
        console.warn("DeviceOrientation n'est pas supporté. Les angles seront estimés via DeviceMotion.");
    }
    
    if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', handleDeviceMotion, true);
    } else {
        console.warn("DeviceMotion n'est pas supporté.");
    } 

    startGPS(); // Démarrage initial du GPS

    // Intervalle lent pour les mises à jour Astro (1s)
    if (domID === null) {
        domID = setInterval(() => {
            // Assurer que la lat/lon existe avant d'appeler updateAstro
            const currentLat = (lPos) ? lPos.coords.latitude : 43.28; // Défaut Marseille si lPos est nul
            const currentLon = (lPos) ? lPos.coords.longitude : 5.35;
            updateAstro(currentLat, currentLon);
        }, DOM_SLOW_UPDATE_MS); 
    }
    
    // Intervalle pour la mise à jour Météo (30s)
    if (weatherID === null) {
        weatherID = setInterval(() => {
            if (lPos) {
                updateWeather(lPos.coords.latitude, lPos.coords.longitude);
            }
        }, WEATHER_UPDATE_MS); 
    }
});
