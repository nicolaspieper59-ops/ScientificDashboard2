// =================================================================
// FICHIER JS PARTIE 1 : gnss-dashboard-part1.js
// Contient les constantes, les variables d'état et les fonctions de base (Kalman, Math).
// Doit être chargé en premier.
// =================================================================

// --- CLÉS D'API & PROXY VERCEL ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app"; 
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc"; 

// --- CONSTANTES GLOBALES ET INITIALISATION ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458; // Vitesse de la lumière (m/s)
const R_E = 6371000;   // Rayon terrestre moyen (m)
const KMH_MS = 3.6;    // Conversion m/s vers km/h
const C_S = 343;       // Vitesse du son dans l'air (m/s)
const G_ACC = 9.80665; // Gravité standard (m/s²)
const MC_DAY_MS = 72 * 60 * 1000; // Durée d'un jour Minecraft en ms
const J1970 = 2440588, J2000 = 2451545; 
const dayMs = 1000 * 60 * 60 * 24;      
const MIN_DT = 0.01; 
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};

// PARAMÈTRES AVANCÉS DU FILTRE DE KALMAN (Horizontal/Vitesse 3D)
const Q_NOISE = 0.01;       
const R_MIN = 0.05, R_MAX = 50.0; 
const MAX_ACC = 200, MIN_SPD = 0.05, ALT_TH = -50; 
const NETHER_RATIO = 8; 
const MAX_PLAUSIBLE_ACCEL = 20.0; 
const Q_ALT_NOISE = 0.1; 
const R_ALT_MIN = 0.1;  
const ACC_DAMPEN_LOW = 5.0; 
const ACC_DAMPEN_HIGH = 50.0; 
const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0 }, 'METAL': { R_MULT: 2.5 },      
    'FOREST': { R_MULT: 1.5 }, 'CONCRETE': { R_MULT: 3.0 },   
};
const DOM_SLOW_UPDATE_MS = 1000; 

// --- VARIABLES D'ÉTAT GLOBALES ---
let wID = null, domID = null, lPos = null, lat = null, lon = null, sTime = null;
let distM = 0, distMStartOffset = 0, maxSpd = 0;
let kSpd = 0, kUncert = 1000; 
let timeMoving = 0; 
let lServH = null, lLocH = null; 
let lastFSpeed = 0; 
let kAlt = null;      
let kAltUncert = 10;  
let currentGPSMode = 'HIGH_FREQ'; 
let emergencyStopActive = false;
let netherMode = false;
let selectedEnvironment = 'NORMAL'; 
let lastP_hPa = null, lastT_K = null, lastH_perc = null; 

// --- VARIABLES POUR CAPTEURS INERTIELS (ACCÉLÉROMÈTRE) ---
let kAccel = { x: 0, y: 0, z: 0 }; 
const ACCEL_FILTER_ALPHA = 0.8; 
let spdH_raw_current = 0; 
let G_STATIC_REF = { x: 0, y: 0, z: G_ACC }; // Référence de la gravité statique
const ACCEL_MOVEMENT_THRESHOLD = 0.3; // Seuil pour détecter un mouvement linéaire réel (m/s²)

// --- REFERENCES DOM ---
const $ = id => document.getElementById(id);

// ===========================================
// FONCTIONS DE BASE PARTAGÉES (MATH & KALMAN)
// ===========================================

/** Calcule la distance horizontale (Haversine). */
const dist = (lat1, lon1, lat2, lon2) => {
    const R = R_E, dLat = (lat2 - lat1) * D2R, dLon = (lon2 - lon1) * D2R;
    lat1 *= D2R; lat2 *= D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
};

/** Applique le filtre de Kalman à la vitesse 3D. */
function kFilter(nSpd, dt, R_dyn) {
    if (dt === 0 || dt > 5) return kSpd; 
    const R = R_dyn ?? R_MAX, Q = Q_NOISE * dt; 
    let pSpd = kSpd, pUnc = kUncert + Q; 
    let K = pUnc / (pUnc + R); 
    kSpd = pSpd + K * (nSpd - pSpd); 
    kUncert = (1 - K) * pUnc; 
    return kSpd;
}

/** Applique le filtre de Kalman à l'Altitude. */
function kFilterAltitude(nAlt, acc, dt) {
    if (nAlt === null) return kAlt;
    const R = Math.max(R_ALT_MIN, acc); 
    const Q = Q_ALT_NOISE * dt; 
    let pAlt = kAlt === null ? nAlt : kAlt; 
    let pUnc = kAltUncert + Q; 
    const K = pUnc / (pUnc + R); 
    kAlt = pAlt + K * (nAlt - pAlt); 
    kAltUncert = (1 - K) * pUnc; 
    return kAlt;
}

/** Calcule le Facteur R (Précision 3D Dynamique) du filtre de Kalman. */
function getKalmanR(acc, alt, P_hPa) {
    let R = acc ** 2; 
    R = Math.max(R_MIN, Math.min(R_MAX, R));
    const envFactor = ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT;
    R *= envFactor;
    if (alt < ALT_TH) { R *= 2.0; } 
    R = Math.max(R_MIN, Math.min(R_MAX, R));
    return R;
               }
// =================================================================
// FICHIER JS PARTIE 2 : gnss-dashboard-part2.js
// Contient la logique principale, les API (Astro/IMU) et les mises à jour DOM.
// Doit être chargé après gnss-dashboard-part1.js.
// =================================================================

// ===========================================
// SYNCHRONISATION HORAIRE PAR SERVEUR (UTC/Atomique)
// ===========================================

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

/** Retourne l'heure synchronisée (précision RTT compensée en UTC). */
function getCDate() { 
    if (lServH === null || lLocH === null) { return null; }
    const offsetSinceSync = performance.now() - lLocH;
    return new Date(lServH + offsetSinceSync); 
}

// ===========================================
// FONCTIONS CAPTEURS INERTIELS (Accéléromètre)
// ===========================================

/** Gère les données de l'accéléromètre via DeviceMotionEvent. (CORRIGÉE) */
function handleDeviceMotion(event) {
    if (emergencyStopActive) return;
    const acc = event.accelerationIncludingGravity;
    // 1. Application du filtre passe-bas sur l'ACCÉLÉRATION TOTALE (incluant G)
    if (acc.x !== null) {
        kAccel.x = ACCEL_FILTER_ALPHA * kAccel.x + (1 - ACCEL_FILTER_ALPHA) * acc.x;
        kAccel.y = ACCEL_FILTER_ALPHA * kAccel.y + (1 - ACCEL_FILTER_ALPHA) * acc.y;
        kAccel.z = ACCEL_FILTER_ALPHA * kAccel.z + (1 - ACCEL_FILTER_ALPHA) * acc.z; // Stocke A_total
    }
    const kAccel_mag = Math.sqrt(kAccel.x ** 2 + kAccel.y ** 2 + kAccel.z ** 2);
    let accel_transverse_lin = 0.0;
    let accel_vertical_lin = 0.0;
    // 2. CORRECTION D'INCLINAISON : Détection de l'état statique
    if (Math.abs(kAccel_mag - G_ACC) < ACCEL_MOVEMENT_THRESHOLD) {
        // Appareil statique : Mettre à jour la référence de gravité (G_STATIC_REF).
        G_STATIC_REF.x = kAccel.x;
        G_STATIC_REF.y = kAccel.y;
        G_STATIC_REF.z = kAccel.z;
        accel_transverse_lin = 0.0; 
        accel_vertical_lin = 0.0; 
    } else {
        // Appareil en mouvement : Accélération linéaire = A_total - G_STATIC_REF
        const linear_x = kAccel.x - G_STATIC_REF.x;
        const linear_y = kAccel.y - G_STATIC_REF.y;
        const linear_z = kAccel.z - G_STATIC_REF.z;
        accel_transverse_lin = Math.sqrt(linear_x ** 2 + linear_y ** 2);
        accel_vertical_lin = linear_z;
    }
    // Mise à jour des valeurs du DOM
    if ($('accel-transverse')) $('accel-transverse').textContent = `${accel_transverse_lin.toFixed(3)} m/s²`;
    if ($('force-g-transverse')) $('force-g-transverse').textContent = `${(accel_transverse_lin / G_ACC).toFixed(2)} G`;
    if ($('accel-vertical-imu')) $('accel-vertical-imu').textContent = `${accel_vertical_lin.toFixed(3)} m/s²`;
    if ($('force-g-vertical')) $('force-g-vertical').textContent = `${(accel_vertical_lin / G_ACC).toFixed(2)} G`;
}

// ===========================================
// FONCTIONS ASTRO & TEMPS (CORRIGÉES TST/EOT/LONGITUDE)
// ===========================================

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
    if (date === null || lon === null) return { TST: 'N/A', MST: 'N/A', EOT: 'N/D', ECL_LONG: 'N/D' };
    const d = toDays(date);
    const M = solarMeanAnomaly(d); 
    const L = eclipticLongitude(M); 
    const epsilon = D2R * (23.4393 - 0.000000356 * d); 
    let alpha = Math.atan2(Math.cos(epsilon) * Math.sin(L), Math.cos(L));
    if (alpha < 0) alpha += 2 * Math.PI; 
    // --- CORRECTION EOT ---
    const meanLongitude = M + D2R * 102.9377 + Math.PI;
    let eot_rad_raw = alpha - meanLongitude; 
    eot_rad_raw = eot_rad_raw % (2 * Math.PI);
    if (eot_rad_raw > Math.PI) { eot_rad_raw -= 2 * Math.PI; } else if (eot_rad_raw < -Math.PI) { eot_rad_raw += 2 * Math.PI; }
    const eot_min = eot_rad_raw * 4 * R2D; // EOT en minutes de temps
    // --- CORRECTION LONGITUDE ÉCLIPTIQUE ---
    let ecl_long_deg = (L * R2D) % 360; 
    const final_ecl_long = ecl_long_deg < 0 ? ecl_long_deg + 360 : ecl_long_deg;
    // Calculs TST 
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

// ===========================================
// FONCTIONS DE MISE À JOUR VISUELLE (TST Clock & Astro)
// ===========================================

/** Met à jour la visualisation du cadran TST */
function updateTSTClock(tst_ms, elevation_deg) {
    const isDay = elevation_deg > 0;
    const rotationPercent = tst_ms / dayMs;
    const angle = (rotationPercent * 360 - 180); 
    const skyElement = $('tst-sky');
    if (skyElement) { skyElement.style.transform = `rotate(${angle}deg)`; }
    const sunElement = $('tst-sun');
    const moonElement = $('tst-moon');
    const radius = 60; 
    if (sunElement) {
        const sunAngle = (rotationPercent * 360); 
        const x = radius * Math.sin(sunAngle * D2R);
        const y = -radius * Math.cos(sunAngle * D2R); 
        sunElement.style.transform = `translate(${x}px, ${y}px)`;
        sunElement.style.opacity = isDay ? 1 : 0.2; 
        sunElement.style.display = 'block';
    }
    if (moonElement) {
        const moonAngle = (rotationPercent * 360 + 180);
        const x = radius * Math.sin(moonAngle * D2R);
        const y = -radius * Math.cos(moonAngle * D2R);
        moonElement.style.transform = `translate(${x}px, ${y}px)`;
        moonElement.style.opacity = isDay ? 0.2 : 1; 
        moonElement.style.display = 'block';
    }
    const statusElement = $('clock-status');
    if (statusElement) { statusElement.textContent = `TST (Élévation: ${elevation_deg.toFixed(2)}°)`; }
}

function updateAstro(latA, lonA) {
    const now = getCDate(); 
    if (now === null) {
        if ($('local-time') && !$('local-time').textContent.includes('Synchronisation')) { $('local-time').textContent = 'Synchronisation...'; }
        return;
    }
    const sunPos = window.SunCalc ? SunCalc.getPosition(now, latA, lonA) : null;
    const solarTimes = getSolarTime(now, lonA);
    const elevation_deg = sunPos ? (sunPos.altitude * R2D) : 0;
    if (solarTimes.TST_MS !== undefined && latA !== null) { updateTSTClock(solarTimes.TST_MS, elevation_deg); } 
    else if ($('clock-status')) { $('clock-status').textContent = 'Position requise pour TST'; }

    $('local-time').textContent = now.toLocaleTimeString('fr-FR', { timeZone: 'UTC', hour12: false });
    if ($('date-display')) $('date-display').textContent = now.toLocaleDateString();
    if (sTime) {
        const timeElapsed = (now.getTime() - sTime) / 1000;
        $('time-elapsed').textContent = `${timeElapsed.toFixed(2)} s`;
        $('time-moving').textContent = `${timeMoving.toFixed(2)} s`;
    }
    // Mise à jour de l'Heure Solaire Vraie (Contrôles et Astro)
    if ($('time-solar-true')) $('time-solar-true').textContent = solarTimes.TST;
    if ($('tst')) $('tst').textContent = solarTimes.TST;
    if ($('culmination-lsm')) $('culmination-lsm').textContent = solarTimes.MST;
    if ($('sun-elevation')) $('sun-elevation').textContent = sunPos ? `${elevation_deg.toFixed(2)} °` : 'N/A';
    if ($('eot-min')) $('eot-min').textContent = solarTimes.EOT + ' min'; 
    if ($('ecliptic-long')) $('ecliptic-long').textContent = solarTimes.ECL_LONG + ' °';
}

// ===========================================
// FONCTIONS DE CONTRÔLE GPS & MÉTÉO 
// ===========================================

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
    emergencyStop(); 
}

async function fetchWeather(latA, lonA) {
    lastP_hPa = null; lastT_K = null; lastH_perc = null; 
    if (!latA || !lonA || PROXY_BASE_URL.includes('scientific-dashboard2')) {
        if ($('env-factor')) $('env-factor').textContent = `${selectedEnvironment} (x${ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT})`;
        return; 
    }
    try {
        const response = await fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${latA}&lon=${lonA}`);
        const data = await response.json();
        if (data.main) {
            lastP_hPa = data.main.pressure; 
            lastT_K = data.main.temp + 273.15; 
            lastH_perc = data.main.humidity / 100; 
        }
    } catch (error) {
        console.error("Échec de la récupération des données météo:", error);
    }
}

// ===========================================
// FONCTION PRINCIPALE DE MISE À JOUR GPS 
// ===========================================

function updateDisp(pos) {
    if (emergencyStopActive) return;
    lat = pos.coords.latitude; lon = pos.coords.longitude;
    const alt = pos.coords.altitude, acc = pos.coords.accuracy;
    const spd = pos.coords.speed;
    const cTimePos = pos.timestamp; 
    const now = getCDate(); 
    if (now === null) { updateAstro(lat, lon); return; } 
    if (sTime === null) { sTime = now.getTime(); distMStartOffset = distM; }
    if (acc > MAX_ACC) { 
        if ($('gps-precision')) $('gps-precision').textContent = `❌ ${acc.toFixed(0)} m (Trop Imprécis)`; 
        if (lPos === null) lPos = pos; return; 
    }
    let spdH = spd ?? 0;
    let spdV = 0; 
    const dt = lPos ? (cTimePos - lPos.timestamp) / 1000 : MIN_DT;

    // 1. FILTRAGE DE L'ALTITUDE (via Kalman)
    const kAlt_new = kFilterAltitude(alt, acc, dt);
    // 2. CALCUL DE LA VITESSE VERTICALE FILTRÉE
    if (lPos && lPos.kAlt_old !== undefined && dt > MIN_DT && alt !== null) { spdV = (kAlt_new - lPos.kAlt_old) / dt; } else if (alt !== null) { spdV = 0; }
    // 3. CALCUL DE LA VITESSE HORIZONTALE BRUTE
    if (lPos && dt > 0.05) { 
        const dH = dist(lPos.coords.latitude, lPos.coords.longitude, lat, lon); 
        spdH = dH / dt; 
        spdH_raw_current = spdH;
    } else if (spd !== null) {
        spdH = spd;
        spdH_raw_current = spdH;
    }
    // 4. LOGIQUE HYBRIDE D'AMORTISSEMENT 3D
    let dampen_factor = 1.0;
    if (acc > ACC_DAMPEN_LOW) {
        const acc_range = ACC_DAMPEN_HIGH - ACC_DAMPEN_LOW;
        const current_excess = acc - ACC_DAMPEN_LOW;
        dampen_factor = 1.0 - Math.min(1.0, current_excess / acc_range);
        spdH *= dampen_factor; spdV *= dampen_factor; 
    }
    let spd3D = Math.sqrt(spdH ** 2 + spdV ** 2);
    // 5. CORRECTION ANTI-SPIKE DE VITESSE
    if (lPos && lPos.speedMS_3D !== undefined && dt > MIN_DT) {
        const lastRawSpd = lPos.speedMS_3D;
        const accelSpike = Math.abs(spd3D - lastRawSpd) / dt;
        if (accelSpike > MAX_PLAUSIBLE_ACCEL) {
            const maxPlausibleChange = MAX_PLAUSIBLE_ACCEL * dt;
            spd3D = spd3D > lastRawSpd ? (lastRawSpd + maxPlausibleChange) : (lastRawSpd - maxPlausibleChange);
        }
    }
    // 6. FILTRE DE KALMAN FINAL 
    const R_dyn = getKalmanR(acc, alt, lastP_hPa); 
    const fSpd = kFilter(spd3D, dt, R_dyn), sSpdFE = fSpd < MIN_SPD ? 0 : fSpd;
    // Calculs d'accélération et distance
    let accel_long = (dt > 0.05) ? (sSpdFE - lastFSpeed) / dt : 0;
    lastFSpeed = sSpdFE;
    distM += sSpdFE * dt * (netherMode ? NETHER_RATIO : 1); 
    if (sSpdFE > MIN_SPD) { timeMoving += dt; }
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    
    // --- MISE À JOUR DU DOM (GPS/Physique) ---
    if ($('latitude')) $('latitude').textContent = lat.toFixed(6);
    if ($('longitude')) $('longitude').textContent = lon.toFixed(6);
    if ($('altitude-gps')) $('altitude-gps').textContent = kAlt_new !== null ? `${kAlt_new.toFixed(2)} m` : 'N/A';
    if ($('gps-precision')) $('gps-precision').textContent = `${acc.toFixed(2)} m`; 
    if ($('speed-raw-ms')) $('speed-raw-ms').textContent = `${spd3D.toFixed(2)} m/s`;
    if ($('vertical-speed')) $('vertical-speed').textContent = `${spdV.toFixed(2)} m/s`;
    if ($('accel-long')) $('accel-long').textContent = `${accel_long.toFixed(3)} m/s²`;
    if ($('force-g-long')) $('force-g-long').textContent = `${(accel_long / G_ACC).toFixed(2)} G`;
    if ($('speed-error-perc')) $('speed-error-perc').textContent = `${R_dyn.toFixed(3)} m² (R dyn)`; 
    if ($('speed-stable')) $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(5)} km/h`; 
    if ($('speed-max')) $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    if ($('distance-total-km')) $('distance-total-km').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    if ($('perc-speed-c')) $('perc-speed-c').textContent = `${(spd3D / C_L * 100).toExponential(2)}%`;
    if ($('env-factor')) $('env-factor').textContent = `${selectedEnvironment} (x${ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT})`;

    if (Date.now() - (updateDisp.lastWeatherFetch ?? 0) > 60000) {
        fetchWeather(lat, lon); 
        updateDisp.lastWeatherFetch = Date.now();
    }
    
    // SAUVEGARDE DES VALEURS POUR LA PROCHAINE ITÉRATION
    lPos = pos; 
    lPos.speedMS_3D = spd3D; 
    lPos.timestamp = cTimePos; 
    lPos.kAlt_old = kAlt_new; 
}

// ===========================================
// INITIALISATION DES ÉVÉNEMENTS DOM 
// ===========================================

document.addEventListener('DOMContentLoaded', () => {
    // Initialisation des sélecteurs et boutons
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

    // Événements de réinitialisation
    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => { if (emergencyStopActive) return; distM = 0; distMStartOffset = 0; timeMoving = 0; });
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', () => { if (emergencyStopActive) return; maxSpd = 0; });
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        if (confirm("Êtes-vous sûr de vouloir tout réinitialiser?")) { 
            distM = 0; maxSpd = 0; distMStartOffset = 0; kSpd = 0; kUncert = 1000; timeMoving = 0; 
        } 
    });

    // --- Initialisation du Capteur de Mouvement (IMU) ---
    if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', handleDeviceMotion, true);
    } 

    startGPS(); 

    // Boucle de mise à jour lente (Astro/Temps)
    if (domID === null) {
        domID = setInterval(() => {
            if (lPos) updateAstro(lPos.coords.latitude, lPos.coords.longitude);
            else updateAstro(null, null); 
        }, DOM_SLOW_UPDATE_MS); 
    }
});
