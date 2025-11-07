// =================================================================
// FICHIER JS PARTIE 1 : gnss-dashboard-part1.js
// CONSTANTES, ÉTAT GLOBAL, FILTRES DE KALMAN & IMU
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

// Constantes Kalman & IMU
const Q_NOISE = 0.001; 
let kSpd = 0; 
let kUncert = 1000; 
const ENVIRONMENT_FACTORS = {
    NORMAL: 1.0, FOREST: 1.5, CONCRETE: 3.0, METAL: 2.5
};
const Q_ALT_NOISE = 0.01; 
let kAlt = 0; 
let kAltUncert = 1000; 
const ACCEL_FILTER_ALPHA = 0.8; 
let kAccel = { x: 0, y: 0, z: 0 };
let latestVerticalAccelIMU = 0; 
let latestLinearAccelMagnitude = 0; 
let latestAccelLongEKF = 0; 

// Constantes Astro / UI
const CLOCK_RADIUS_PX = 45; 
const MOON_ICON_SIZE_PX = 15;
const MOON_ORBIT_RADIUS_PX = CLOCK_RADIUS_PX - (MOON_ICON_SIZE_PX / 2); 
const OWM_API_KEY = "VOTRE_CLE_API_OPENWEATHERMAP"; 
const OWM_API_URL = "https://api.openweathermap.org/data/2.5/weather";

// --- VARIABLES D'ÉTAT GLOBALES ---
let wID = null;
let lat = 43.2965; 
let lon = 5.3698;  
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
let global_pitch = 0; 
let global_roll = 0; 
const trajectoryPoints = []; 

// Variables pour la carte (Doit être dans un fichier global ou ici)
let map = null;
let marker = null;
let tracePolyline = null;
let domID = null; 
let weatherID = null; 

// ===========================================
// FONCTIONS UTILITAIRES ET KALMAN
// ===========================================

function dist(lat1, lon1, lat2, lon2) {
    const dLat = (lat2 - lat1) * D2R;
    const dLon = (lon2 - lon1) * D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1 * D2R) * Math.cos(lat2 * D2R) * Math.sin(dLon / 2) ** 2;
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R_E * c;
}

function kFilter(z, dt, R_dyn) {
    const predSpd = kSpd;
    const predUncert = kUncert + Q_NOISE * dt;
    const K = predUncert / (predUncert + R_dyn);
    kSpd = predSpd + K * (z - predSpd);
    kUncert = (1 - K) * predUncert;
    return kSpd;
}

/** CALCULE L'INCERTITUDE DYNAMIQUE R_dyn (LOGIQUE RENFORCÉE EKF/IMU)
 * **CORRECTION VITESSE APPLIQUÉE ICI**
 * @param acc Précision GPS (m)
 * @param alt Altitude (m)
 * @param pressure Pression Atmosphérique (hPa)
 * @param linearAccelMag Magnitude de l'Accélération Linéaire (IMU) (m/s²)
 * @param accelLongEKF Accélération Longitudinale filtrée (EKF) (m/s²)
 */
function getKalmanR(acc, alt, pressure, linearAccelMag, accelLongEKF) { 
    let R_gps_base = acc * acc; 
    if (R_gps_base < 0.1) R_gps_base = 0.1;

    const envFactor = ENVIRONMENT_FACTORS[selectedEnvironment] || ENVIRONMENT_FACTORS.NORMAL;
    let dynamicMultiplier = envFactor;
    
    // 1. Influence de l'altitude / Underground
    if (alt !== null && alt < 0) { 
        dynamicMultiplier += Math.abs(alt / 50); 
    }
    
    // 2. RENFORCEMENT EKF/IMU : Influence de l'Accélération Longitudinale (EKF)
    const accel_influence = Math.pow(Math.abs(accelLongEKF), 2) * 1.5; // Facteur plus agressif
    dynamicMultiplier += accel_influence;
    
    // 3. Influence de l'accélération linéaire totale (IMU) - Bruit de choc/vibration
    const imu_noise_influence = Math.pow(linearAccelMag, 2) * 0.1;
    dynamicMultiplier += imu_noise_influence;
    
    // 4. Ajout d'une influence de base pour que l'IMU corrige toujours un minimum
    dynamicMultiplier += 0.5; // Planche pour l'influence IMU

    return R_gps_base * dynamicMultiplier;
}

function kFilterAltitude(z, acc, dt, u_accel = 0) { 
    if (z === null) return kAlt; 
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

function handleDeviceOrientation(event) {
    if (emergencyStopActive) return;
    const roll_deg = event.gamma || 0; 
    const pitch_deg = event.beta || 0; 
    global_roll = roll_deg * D2R;
    global_pitch = pitch_deg * D2R;
    if ($('pitch-angle')) $('pitch-angle').textContent = `${pitch_deg.toFixed(1)} °`;
    if ($('roll-angle')) $('roll-angle').textContent = `${roll_deg.toFixed(1)} °`;
}

function handleDeviceMotion(event) {
    if (emergencyStopActive) return;
    const acc_g_raw = event.accelerationIncludingGravity;
    if (acc_g_raw.x === null) return; 

    kAccel.x = ACCEL_FILTER_ALPHA * kAccel.x + (1 - ACCEL_FILTER_ALPHA) * acc_g_raw.x;
    kAccel.y = ACCEL_FILTER_ALPHA * kAccel.y + (1 - ACCEL_FILTER_ALPHA) * acc_g_raw.y;
    kAccel.z = ACCEL_FILTER_ALPHA * kAccel.z + (1 - ACCEL_FILTER_ALPHA) * acc_g_raw.z; 

    if (global_pitch === 0 && global_roll === 0) {
        global_pitch = Math.atan2(kAccel.x, Math.sqrt(kAccel.y * kAccel.y + kAccel.z * kAccel.z));
        global_roll = Math.atan2(kAccel.y, kAccel.z);
        if ($('pitch-angle')) $('pitch-angle').textContent = `${(global_pitch * R2D).toFixed(1)} °`;
        if ($('roll-angle')) $('roll-angle').textContent = `${(global_roll * R2D).toFixed(1)} °`;
    }

    const phi = global_roll; 
    const theta = global_pitch; 
    const g_local = G_ACC; 

    const G_x_proj = g_local * Math.sin(theta);        
    const G_y_proj = -g_local * Math.sin(phi) * Math.cos(theta); 
    const G_z_proj_abs = g_local * Math.cos(phi) * Math.cos(theta);  
    
    let acc_lin_t_x = kAccel.x - G_x_proj;
    let acc_lin_t_y = kAccel.y - G_y_proj;
    let acc_lin_t_z = 0;

    let acc_lin_temp = kAccel.z - G_z_proj_abs; 
    if (Math.abs(acc_lin_temp) > 0.8 * G_ACC) { 
        acc_lin_t_z = kAccel.z + G_z_proj_abs; 
    } else {
        acc_lin_t_z = acc_lin_temp;
    }
    if (Math.abs(acc_lin_t_z) > 0.8 * G_ACC) {
        acc_lin_t_z = -acc_lin_t_z;
    }
    
    latestVerticalAccelIMU = acc_lin_t_z; 
    latestLinearAccelMagnitude = Math.sqrt(acc_lin_t_x ** 2 + acc_lin_t_y ** 2 + acc_lin_t_z ** 2); 
    
    if ($('accel-vertical-imu')) $('accel-vertical-imu').textContent = `${acc_lin_t_z.toFixed(3)} m/s²`;
    if ($('force-g-vertical')) $('force-g-vertical').textContent = `${(acc_lin_t_z / G_ACC).toFixed(2)} G`;
}
// =================================================================
// FICHIER JS PARTIE 2 : gnss-dashboard-part2.js
// BOUCLE ASTRO, MÉTÉO, FOND DE CIEL & MISE À JOUR PRINCIPALE (updateDisp)
// Dépend de gnss-dashboard-part1.js
// =================================================================

// --- Fonctions utilitaires du Temps ---
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

// --- Fonctions Astro ---
function toDays(date) { return (date.valueOf() / dayMs - 0.5 + J1970) - J2000; }
function solarMeanAnomaly(d) { return D2R * (356.0470 + 0.9856002585 * d); }
function eclipticLongitude(M) {
    var C = D2R * (1.9148 * Math.sin(M) + 0.0200 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M)), 
        P = D2R * 102.9377;                                                                
    return M + C + P + Math.PI;
}

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
    return { TST: toTimeString(tst_ms), TST_MS: tst_ms, MST: toTimeString(mst_ms), EOT: eot_min.toFixed(3), ECL_LONG: final_ecl_long.toFixed(2) };
}

function getClosestMoonPhaseSymbol(phase) {
    const phases = [
        { threshold: 0.00, symbol: '🌑', name: 'Nouvelle Lune' },
        { threshold: 0.07, symbol: '🌒', name: 'Nouveau Croissant' }, 
        { threshold: 0.22, symbol: '🌓', name: 'Premier Quartier' }, 
        { threshold: 0.35, symbol: '🌔', name: 'Gibbeuse Croissante' }, 
        { threshold: 0.50, symbol: '🌕', name: 'Pleine Lune' }, 
        { threshold: 0.65, symbol: '🌖', name: 'Gibbeuse Décroissante' }, 
        { threshold: 0.88, symbol: '🌗', name: 'Dernier Quartier' }, 
        { threshold: 0.93, symbol: '🌘', name: 'Vieux Croissant' }, 
        { threshold: 1.00, symbol: '🌑', name: 'Nouvelle Lune' },
    ];
    let bestPhase = phases[0];
    let minDiff = Infinity;
    for (let i = 0; i < phases.length; i++) {
        const diff = Math.abs(phase - phases[i].threshold);
        if (diff < minDiff) {
            minDiff = diff;
            bestPhase = phases[i];
        }
    }
    return bestPhase;
}

/** Met à jour le dégradé du ciel et simule la trajectoire saisonnière.
 * **CORRECTION DÉGRADÉ DU CIEL APPLIQUÉE ICI**
 */
function updateSkyGradient(elevation_deg, now) {
    const skyEl = $('sky-background');
    if (!skyEl) return;

    const month = now.getMonth(); // 0 (Jan) to 11 (Dec)
    
    // Couleurs de base pour le ciel
    const nightTop = '#00001a'; // Noir/Bleu Nuit
    const nightBottom = '#00004d'; // Bleu Nuit
    const blueDay = '#87ceeb'; // Bleu Clair
    const whiteDay = '#e0ffff'; // Blanc Très Clair
    const deepOrange = '#ff4500'; 
    
    // Ajustement saisonnier pour le crépuscule
    const seasonalOrange = (month >= 9 || month <= 2) ? deepOrange : '#ffa500';

    if (elevation_deg > 10) { 
        // Plein jour (Altitude > 10°)
        skyEl.style.background = `linear-gradient(to top, ${whiteDay}, ${blueDay})`;
        skyEl.style.opacity = 1;
    } else if (elevation_deg > 0) { 
        // Jour faible / Crépuscule civil (0° < Altitude ≤ 10°)
        const factor = elevation_deg / 10;
        const colorMiddle = `rgba(255, 165, 0, ${1 - factor})`; 

        skyEl.style.background = `linear-gradient(to top, ${whiteDay} 0%, ${colorMiddle} 50%, ${blueDay} 100%)`;
        skyEl.style.opacity = 1;
    } else if (elevation_deg > -18) { 
        // Crépuscule nautique et astronomique (-18° < Altitude ≤ 0°)
        const factor = (elevation_deg + 18) / 18; 
        
        const darkPurple = Math.round(75 * (1 - factor)); 
        const colorMiddle = `rgb(${darkPurple}, 0, 100)`; 

        // Transition nuit -> crépuscule
        skyEl.style.background = `linear-gradient(to top, ${seasonalOrange} 0%, ${colorMiddle} 30%, ${nightTop} 80%)`;
        skyEl.style.opacity = Math.max(0.5, factor); 
    } else { 
        // Nuit (Altitude ≤ -18°)
        skyEl.style.background = `linear-gradient(to top, ${nightBottom}, ${nightTop})`;
        skyEl.style.opacity = 1;
    }
    
    // SIMULATION DE TRAJECTOIRE SAISONNIÈRE (Inclinaison de l'horizon)
    const seasonalEclipticTilt = Math.sin((month - 3) * (Math.PI / 6)) * 15; 
    const tilt = 18 + seasonalEclipticTilt; 
    skyEl.style.transform = `perspective(1000px) rotateX(${tilt}deg)`; 
}


function updateAstro(latA, lonA) {
    const now = getCDate(); 
    if (now === null || latA === 0 || lonA === 0) return; 
    
    const times = window.SunCalc ? SunCalc.getTimes(now, latA, lonA) : null;
    const sunPos = window.SunCalc ? SunCalc.getPosition(now, latA, lonA) : null;
    const moonIllum = window.SunCalc ? SunCalc.getMoonIllumination(now) : null;
    const moonPos = window.SunCalc ? SunCalc.getMoonPosition(now, latA, lonA) : null;
    
    const solarTimes = getSolarTime(now, lonA);
    const elevation_deg = sunPos ? (sunPos.altitude * R2D) : 0;
    
    // MISE À JOUR DU FOND DE CIEL SAISONNIER
    updateSkyGradient(elevation_deg, now);
    
    $('local-time').textContent = now.toLocaleTimeString('fr-FR', { timeZone: 'UTC', hour12: false });
    $('date-display').textContent = now.toLocaleDateString();
    
    if (sTime) {
        const timeElapsed = (now.getTime() - sTime) / 1000;
        $('time-elapsed').textContent = `${timeElapsed.toFixed(2)} s`;
        $('time-moving').textContent = `${timeMoving.toFixed(2)} s`;
    }
    
    const tst_ms = solarTimes.TST_MS;
    if (isNaN(tst_ms)) return; 
    
    const percentOfDay = tst_ms / dayMs; 
    const rotationDeg = (percentOfDay * 360) - 180; 
    const clockFace = $('minecraft-clock-face');
    
    if (clockFace && times) {
        const dayDurationMs = times.sunset.getTime() - times.sunrise.getTime();
        const dayPercent = (dayDurationMs / dayMs) * 100;
        const sunriseTST = getSolarTime(times.sunrise, lonA).TST_MS;
        const sunrisePercent = (sunriseTST / dayMs) * 100;
        const startAngle = (sunrisePercent * 360) - 90; 

        clockFace.style.background = `conic-gradient(from ${startAngle}deg, #f7dc00 0% ${dayPercent}%, #3777d1 ${dayPercent}% 100%)`;
        clockFace.style.transform = `rotate(${rotationDeg}deg)`;
    }
    
    if ($('clock-status')) {
         if (elevation_deg > 0) {
             $('clock-status').textContent = `Jour (Soleil visible)`;
         } else if (elevation_deg > -18) {
             $('clock-status').textContent = `Crépuscule / Aube`;
         } else {
             $('clock-status').textContent = `Nuit (Lune visible)`;
         }
    }
    
    const moonIconEl = $('moon-icon');
    if (moonPos && moonIllum && moonIconEl) {
        
        const angleOffsetRad = moonIllum.angle; 
        const angleOffsetDeg = angleOffsetRad * R2D;
        
        let moonRotationDeg = rotationDeg + angleOffsetDeg;

        const rotationAngleAroundCenter = (moonRotationDeg + 180); 
        
        moonIconEl.style.transform = `translate(-50%, -50%) rotate(${rotationAngleAroundCenter}deg) translateX(${MOON_ORBIT_RADIUS_PX}px) rotate(${-rotationAngleAroundCenter}deg)`;

        const currentPhase = getClosestMoonPhaseSymbol(moonIllum.phase);
        moonIconEl.textContent = currentPhase.symbol;
        moonIconEl.title = `${currentPhase.name} - Illum: ${(moonIllum.fraction * 100).toFixed(1)} %`;

        $('moon-phase').textContent = `${(moonIllum.fraction * 100).toFixed(1)} % (${currentPhase.name})`;
        $('moon-elevation').textContent = `${(moonPos.altitude * R2D).toFixed(2)} °`;
        $('moon-azimuth').textContent = `${(moonPos.azimuth * R2D).toFixed(2)} °`;
    } else if (moonIconEl) {
        moonIconEl.textContent = '';
        $('moon-phase').textContent = 'N/A';
        $('moon-elevation').textContent = '-- °';
        $('moon-azimuth').textContent = '-- °';
    }

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

function calculateDewPoint(tempC, humidity) {
    const a = 17.27;
    const b = 237.7;
    const alpha = (a * tempC) / (b + tempC) + Math.log(humidity / 100);
    return (b * alpha) / (a - alpha);
}

async function updateWeather(latA, lonA) {
    if (!OWM_API_KEY || OWM_API_KEY === "VOTRE_CLE_API_OPENWEATHERMAP") {
        if ($('temp-air')) $('temp-air').textContent = 'API CLÉ MANQUANTE';
        return;
    }
    if (!latA || !lonA) return; 
    
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


/** FONCTION PRINCIPALE DE MISE À JOUR (GPS, Kalman, Physique) */
function updateDisp(pos) {
    if (emergencyStopActive) return;
    
    let alt, acc, spd_raw_gps, cTimePos;
    
    if (pos) {
        lat = pos.coords.latitude; 
        lon = pos.coords.longitude;
        alt = pos.coords.altitude;
        acc = pos.coords.accuracy;
        spd_raw_gps = pos.coords.speed;
        cTimePos = pos.timestamp;
    } else {
        alt = null;
        acc = 99.0;
        spd_raw_gps = 0;
        cTimePos = Date.now();
    }
    
    const now = getCDate(); 
    const MASS = 70.0; 

    if (now === null) { updateAstro(lat, lon); return; } 
    
    if (sTime === null) { 
        sTime = now.getTime(); 
        if (alt !== null && kAlt === 0) {
            kAlt = alt;
            kAltUncert = acc * acc * 2.0; 
        }
    }
    
    if (acc > MAX_ACC && pos) { 
        if ($('gps-precision')) $('gps-precision').textContent = `❌ ${acc.toFixed(0)} m (Trop Imprécis)`; 
        if (lPos === null) lPos = { coords: { latitude: lat, longitude: lon }}; 
        return; 
    }

    let effectiveAcc = acc;
    const accOverride = parseFloat($('gps-accuracy-override').value);
    if (accOverride > 0) { effectiveAcc = accOverride; }

    let spdH = spd_raw_gps ?? 0; 
    const dt = lPos ? (cTimePos - lPos.timestamp) / 1000 : MIN_DT;

    const kAlt_new = kFilterAltitude(alt, effectiveAcc, dt, latestVerticalAccelIMU); 
    
    let spdV = 0; 
    if (lPos && lPos.kAlt_old !== undefined && dt > MIN_DT && alt !== null) { 
        spdV = (kAlt_new - lPos.kAlt_old) / dt; 
        let verticalSpeedUncert = Math.sqrt(kAltUncert ** 2 + (lPos.kAltUncert_old || kAltUncert) ** 2) / dt;
        if ($('vertical-speed-uncert')) $('vertical-speed-uncert').textContent = `${Math.min(20, verticalSpeedUncert).toFixed(2)} m/s`;
    } 
    
    if (lPos && dt > 0.05 && pos) { 
        const dH = dist(lPos.coords.latitude, lPos.coords.longitude, lat, lon); 
        spdH = dH / dt; 
    } 

    let spd3D = Math.sqrt(spdH ** 2 + spdV ** 2);

    const R_dyn = getKalmanR(effectiveAcc, alt, lastP_hPa, latestLinearAccelMagnitude, latestAccelLongEKF); 
    const fSpd = kFilter(spd3D, dt, R_dyn); 
    const sSpdFE = fSpd < MIN_SPD ? 0 : fSpd; 

    let accel_long = (dt > 0.05) ? (sSpdFE - lastFSpeed) / dt : 0;
    
    latestAccelLongEKF = accel_long; 

    lastFSpeed = sSpdFE;
    if (pos) { 
        distM += sSpdFE * dt * (netherMode ? NETHER_RATIO : 1); 
    }
    if (sSpdFE > MIN_SPD) { timeMoving += dt; }
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
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
    $('altitude-gps').textContent = (alt !== null) ? `${alt.toFixed(2)} m` : 'N/A';
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
    
    if (typeof updateMap === 'function') {
        updateMap(lat, lon); 
    }
    
    if (pos) { 
        lPos = pos; 
        lPos.timestamp = cTimePos; 
    } else if (!lPos) { 
        lPos = { coords: { latitude: lat, longitude: lon }, timestamp: cTimePos };
    }
    lPos.kAlt_old = kAlt_new; 
    lPos.kAltUncert_old = kAltUncert; 
        }
// =================================================================
// FICHIER JS PARTIE 3 : gnss-dashboard-part3.js
// CARTE (Leaflet), CONTRÔLES & GESTIONNAIRES D'ÉVÉNEMENTS
// Dépend de gnss-dashboard-part1.js et gnss-dashboard-part2.js
// =================================================================

/** Initialise la carte Leaflet. */
function initMap(latA, lonA) {
    // Vérifie si Leaflet (L) est chargé
    if (typeof L === 'undefined') {
        if ($('map-container')) $('map-container').textContent = 'Erreur: Librairie Leaflet non chargée.';
        return;
    }

    if (map) { 
        map.setView([latA, lonA], 15);
        return;
    }
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

/** Met à jour la carte. */
function updateMap(latA, lonA) {
    if (typeof L === 'undefined' || !map || !marker || !tracePolyline) { 
        if (latA !== 0 || lonA !== 0) initMap(latA, lonA); 
        return; 
    }
    
    const newLatLng = [latA, lonA];
    
    if (wID !== null) {
        trajectoryPoints.push(newLatLng);
        tracePolyline.setLatLngs(trajectoryPoints);
    }
    
    marker.setLatLng(newLatLng);
    marker.setRadius(kUncert / 2.0);
    
    if (!map.getBounds().contains(newLatLng)) {
        map.panTo(newLatLng);
    }
}

function resetTrajectory() {
    trajectoryPoints.length = 0; 
    if (tracePolyline) {
        tracePolyline.setLatLngs([]); 
    }
    if (map && marker) {
        map.setView(marker.getLatLng(), 17);
    }
}

function applyManualCoords() {
    const latInput = parseFloat($('override-lat').value);
    const lonInput = parseFloat($('override-lon').value);

    if (isNaN(latInput) || isNaN(lonInput)) {
        alert("Coordonnées manuelles invalides.");
        return;
    }

    lat = latInput;
    lon = lonInput;
    
    if (typeof updateAstro === 'function') updateAstro(lat, lon);
    if (typeof updateWeather === 'function') updateWeather(lat, lon);

    initMap(lat, lon);
    
    lPos = null;
    sTime = null; 
    
    if (typeof updateDisp === 'function') updateDisp(null); 
}


function handleErr(err) {
    console.error(`Erreur GNSS (${err.code}): ${err.message}`);
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = `❌ ERREUR GPS`;
    
    $('override-lat').disabled = false;
    $('override-lon').disabled = false;
    $('apply-coords-btn').disabled = false;

    if (sTime === null) { 
        applyManualCoords(); 
    }
}

function setGPSMode(mode) {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    currentGPSMode = mode;
    
    if (typeof updateDisp === 'function') {
        wID = navigator.geolocation.watchPosition(updateDisp, handleErr, GPS_OPTS[mode]);
    } else {
        wID = navigator.geolocation.watchPosition(console.log, handleErr, GPS_OPTS[mode]);
    }
    
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = `⏸️ PAUSE GPS`;
    if ($('freq-select')) $('freq-select').value = mode; 
    
    $('override-lat').disabled = true;
    $('override-lon').disabled = true;
    $('apply-coords-btn').disabled = true;
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
    $('override-lat').disabled = false;
    $('override-lon').disabled = false;
    $('apply-coords-btn').disabled = false;
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
    
document.addEventListener('DOMContentLoaded', () => {
    
    // syncH est dans Part 2
    if (typeof syncH === 'function') syncH(); 

    if ($('environment-select')) {
        $('environment-select').value = selectedEnvironment;
        $('environment-select').addEventListener('change', (e) => { 
            if (emergencyStopActive) return;
            selectedEnvironment = e.target.value; 
        });
    }

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
            kAlt = 0; kAltUncert = 1000; lPos = null; sTime = null;
            resetTrajectory(); 
        } 
    });
    
    if ($('toggle-mode-btn')) $('toggle-mode-btn').addEventListener('click', () => {
        document.body.classList.toggle('dark-mode');
        const isDarkMode = document.body.classList.contains('dark-mode');
        $('toggle-mode-btn').textContent = isDarkMode ? "☀️ Mode Jour" : "🌗 Mode Nuit";
    });

    if ($('apply-coords-btn')) {
        $('apply-coords-btn').addEventListener('click', applyManualCoords);
    }
    
    // handleDeviceOrientation et handleDeviceMotion sont dans Part 1
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

    startGPS(); 

    // updateAstro et updateWeather sont dans Part 2
    if (typeof updateAstro === 'function') {
        if (domID === null) {
            domID = setInterval(() => {
                updateAstro(lat, lon); 
            }, DOM_SLOW_UPDATE_MS); 
        }
    }
    
    if (typeof updateWeather === 'function') {
        if (weatherID === null) {
            weatherID = setInterval(() => {
                if (lat !== 0 && lon !== 0) { 
                    updateWeather(lat, lon);
                }
            }, WEATHER_UPDATE_MS); 
        }
    }
});
