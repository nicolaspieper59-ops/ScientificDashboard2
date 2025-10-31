// =================================================================
// FICHIER JS PARTIE 1/2 : gnss-dashboard-part1.js (Final V4.0 - IMU/GPS/Astro)
// =================================================================

// --- CLÉS D'API & PROXY VERCEL ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app"; 
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc"; 

// --- CONSTANTES GLOBALES ET INITIALISATION ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458; 
const R_E = 6371000;   
const KMH_MS = 3.6;    
const C_S = 343;       
const G_ACC = 9.80665; 
const MC_DAY_MS = 72 * 60 * 1000; 
const MSL_OFFSET_M = 40; // Correction locale moyenne par défaut WGS84 -> MSL

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

// Anti-Pic d'Accélération RIGOUREUX (Mode Zéro Bruit Statique)
const MAX_PLAUSIBLE_ACCEL = 0.5; 

// PARAMÈTRES POUR LE FILTRE DE KALMAN D'ALTITUDE
const Q_ALT_NOISE = 0.1; 
const R_ALT_MIN = 0.1;  

// PARAMÈTRES DE LISSAGE HYBRIDE (DAMPENING)
const ACC_DAMPEN_LOW = 5.0; 
const ACC_DAMPEN_HIGH = 50.0; 

// FACTEURS ENVIRONNEMENTAUX POUR LA CORRECTION KALMAN
const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0 },
    'METAL': { R_MULT: 2.5 },      
    'FOREST': { R_MULT: 1.5 },     
    'CONCRETE': { R_MULT: 3.0 },   
};
const DOM_SLOW_UPDATE_MS = 1000; 

// --- VARIABLES D'ÉTAT ---
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

// --- VARIABLES D'ÉTAT DES CAPTEURS ---
let lastAccel = { x: 0, y: 0, z: 0 }; 
let lastHeading = 0; 
let mslOffset = MSL_OFFSET_M; // Stocke la correction MSL (si disponible via l'API, sinon la valeur par défaut)

// --- REFERENCES DOM ---
const $ = id => document.getElementById(id);

// ===========================================
// FONCTIONS GÉO & UTILS
// ===========================================

/** Calcule la distance horizontale (Haversine). */
const dist = (lat1, lon1, lat2, lon2) => {
    const R = R_E, dLat = (lat2 - lat1) * D2R, dLon = (lon2 - lon1) * D2R;
    lat1 *= D2R; lat2 *= D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
};

// ===========================================
// SYNCHRONISATION HORAIRE PAR SERVEUR (UTC/Atomique)
// (Inchangé)
// ===========================================

async function syncH() { 
    if ($('local-time')) $('local-time').textContent = 'Synchronisation UTC...';
    const localStartPerformance = performance.now(); 

    try {
        const response = await fetch(SERVER_TIME_ENDPOINT, { cache: "no-store", mode: "cors" });
        if (!response.ok) throw new Error(`Server time sync failed: ${response.statusText}`);
        
        const localEndPerformance = performance.now(); 
        const serverData = await response.json(); 
        
        const utcTimeISO = serverData.datetime; 
        const serverTimestamp = Date.parse(utcTimeISO); 
        
        const RTT = localEndPerformance - localStartPerformance;
        const latencyOffset = RTT / 2;

        lServH = serverTimestamp + latencyOffset; 
        lLocH = performance.now(); 
        console.log(`Synchronisation UTC Atomique réussie. Latence corrigée: ${latencyOffset.toFixed(1)} ms.`);

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
// FILTRE DE KALMAN & FACTEUR R
// (Inchangé)
// ===========================================

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
    
    if (P_hPa !== null) {
        const pressureFactor = 1.0 + (1013.25 - P_hPa) / 1013.25 * 0.1;
        R *= Math.max(1.0, pressureFactor);
    }
    
    R *= envFactor;
    if (alt < ALT_TH) { R *= 2.0; } 

    R = Math.max(R_MIN, Math.min(R_MAX, R));
    
    return R;
}

/** 🌐 Corrige l'altitude WGS84 (GPS) en Altitude MSL (Niveau de la mer) */
function correctAltitudeToMSL(wgs84Alt, geoidHeight) {
    if (wgs84Alt === null) return null;
    // La hauteur du géoïde (geoidHeight) est la différence entre l'ellipsoïde WGS84 et le géoïde (MSL).
    // Altitude MSL = Altitude WGS84 - Hauteur du géoïde.
    // Cette valeur est typiquement fournie dans l'objet GPS (si l'appareil est récent), sinon on utilise une valeur par défaut.
    return wgs84Alt - (geoidHeight ?? mslOffset);
    }
// =================================================================
// FICHIER JS PARTIE 2/2 : gnss-dashboard-part2.js (AVEC FUSION CAPTEUR & ASTRO)
// NÉCESSITE gnss-dashboard-part1.js et suncalc.js
// =================================================================

// ===========================================
// FONCTIONS ASTRO & TEMPS 
// ===========================================

/** Convertit la date en jours depuis J2000. (Inchangé) */
function toDays(date) { return (date.valueOf() / dayMs - 0.5 + J1970) - J2000; }

/** Calcule l'anomalie solaire moyenne (en radians). (Inchangé) */
function solarMeanAnomaly(d) { return D2R * (356.0470 + 0.9856002585 * d); }

/** Calcule le Temps Solaire Vrai (TST) et l'Équation du Temps (EOT). (Inchangé) */
function getSolarTime(date, lon) {
    if (date === null || lon === null) return { TST: 'N/A', MST: 'N/A', EOT: 'N/D', ECL_LONG: 'N/D' };
    
    const d = toDays(date);
    const M = solarMeanAnomaly(d); 
    
    var C = D2R * (1.9148 * Math.sin(M) + 0.0200 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M)); 
    var P = D2R * 102.9377; 
    let L = M + C + P + Math.PI;
    L = L % (2 * Math.PI); 
    if (L < 0) L += 2 * Math.PI;

    let L_mean = M + P + Math.PI;
    L_mean = L_mean % (2 * Math.PI); 
    if (L_mean < 0) L_mean += 2 * Math.PI;
    
    const epsilon = D2R * (23.4393 - 0.000000356 * d); 
    
    let alpha = Math.atan2(Math.cos(epsilon) * Math.sin(L), Math.cos(L));
    if (alpha < 0) alpha += 2 * Math.PI; 
    
    let alpha_mean = Math.atan2(Math.cos(epsilon) * Math.sin(L_mean), Math.cos(L_mean));
    if (alpha_mean < 0) alpha_mean += 2 * Math.PI; 
    
    let eot_rad = alpha_mean - alpha;
    
    if (eot_rad > Math.PI) eot_rad -= 2 * Math.PI;
    if (eot_rad < -Math.PI) eot_rad += 2 * Math.PI;
    
    const eot_min = eot_rad * R2D * 4; 
    
    const msSinceMidnightUTC = (date.getUTCHours() * 3600 + date.getUTCMinutes() * 60 + date.getUTCSeconds()) * 1000 + date.getUTCMilliseconds();
    const mst_offset_ms = lon * dayMs / 360; 
    const mst_ms = (msSinceMidnightUTC + mst_offset_ms + dayMs) % dayMs; 

    const eot_ms = eot_min * 60000;
    const tst_ms = (mst_ms + eot_ms + dayMs) % dayMs; 

    const toTimeString = (ms) => {
        let h = Math.floor(ms / 3600000);
        let m = Math.floor((ms % 3600000) / 60000);
        let s = Math.floor((ms % 60000) / 1000);
        return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
    };
    
    return { 
        TST: toTimeString(tst_ms), 
        MST: toTimeString(mst_ms), 
        EOT: eot_min.toFixed(3),
        ECL_LONG: (L * R2D).toFixed(2) 
    };
}

/** 🌞 Mise à jour complète des données Astro (Soleil/Lune) */
function updateAstro(latA, lonA) {
    const now = getCDate(); 
    if (now === null) {
        if ($('local-time') && !$('local-time').textContent.includes('Synchronisation')) {
             $('local-time').textContent = 'Synchronisation...';
        }
        if ($('date-display')) $('date-display').textContent = 'N/A';
        return;
    }
    
    // --- CALCULS SunCalc ---
    const sunPos = window.SunCalc ? SunCalc.getPosition(now, latA, lonA) : null;
    const moonPos = window.SunCalc ? SunCalc.getMoonPosition(now, latA, lonA) : null;
    const moonIllum = window.SunCalc ? SunCalc.getMoonIllumination(now) : null;
    const sunTimes = window.SunCalc ? SunCalc.getTimes(now, latA, lonA) : null;
    const solarTimes = getSolarTime(now, lonA); 

    if ($('date-display')) $('date-display').textContent = now.toLocaleDateString();
    
    // --- TEMPS & DISTANCE ---
    $('local-time').textContent = now.toLocaleTimeString('fr-FR', { timeZone: 'UTC', hour12: false });
    if (sTime) {
        const timeElapsed = (now.getTime() - sTime) / 1000;
        $('time-elapsed').textContent = `${timeElapsed.toFixed(2)} s`;
        $('time-moving').textContent = `${timeMoving.toFixed(2)} s`;
    }
    
    // --- SOLEIL ---
    if ($('time-solar-true')) $('time-solar-true').textContent = solarTimes.TST;
    if ($('culmination-lsm')) $('culmination-lsm').textContent = solarTimes.MST;
    if ($('eot-min')) $('eot-min').textContent = solarTimes.EOT + ' min'; 
    if ($('ecliptic-long')) $('ecliptic-long').textContent = solarTimes.ECL_LONG + ' °';
    if ($('sun-elevation')) $('sun-elevation').textContent = sunPos ? `${(sunPos.altitude * R2D).toFixed(2)} °` : 'N/A';
    if ($('sun-azimuth')) $('sun-azimuth').textContent = sunPos ? `${(sunPos.azimuth * R2D + 180).toFixed(2) % 360} °` : 'N/A';
    if ($('noon-solar')) $('noon-solar').textContent = sunTimes && sunTimes.solarNoon ? sunTimes.solarNoon.toLocaleTimeString() : 'N/D';

    // --- LUNE ---
    if ($('lunar-phase-perc')) $('lunar-phase-perc').textContent = moonIllum ? `${(moonIllum.fraction * 100).toFixed(1)} %` : 'N/A';
    if ($('moon-elevation')) $('moon-elevation').textContent = moonPos ? `${(moonPos.altitude * R2D).toFixed(2)} °` : 'N/A';
    if ($('moon-azimuth')) $('moon-azimuth').textContent = moonPos ? `${(moonPos.azimuth * R2D + 180).toFixed(2) % 360} °` : 'N/A';
    
    if ($('day-duration') && sunTimes && sunTimes.sunrise && sunTimes.sunset) {
        const durationMs = sunTimes.sunset.getTime() - sunTimes.sunrise.getTime();
        const hours = Math.floor(durationMs / 3600000);
        const minutes = Math.floor((durationMs % 3600000) / 60000);
        $('day-duration').textContent = `${hours}h ${minutes}m`;
    } else if ($('day-duration')) {
        $('day-duration').textContent = 'N/A (Polaire/Nuit)';
    }
}

// ===========================================
// FONCTIONS DE CONTRÔLE GPS & MÉTÉO 
// (Inchangé)
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

/** Fonction d'arrêt d'urgence : bloque l'exécution et arrête le GPS */
function emergencyStop() {
    emergencyStopActive = true;
    stopGPS(false);
    if ($('emergency-stop-btn')) {
        $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: ACTIF 🔴";
        $('emergency-stop-btn').classList.add('active');
    }
    ['speed-stable', 'speed-3d-inst', 'distance-total-km', 'local-time'].forEach(id => {
        if ($(id)) $(id).textContent = 'ARRÊT D’URGENCE';
    });
}

/** Fonction de reprise : réactive le système et redémarre le GPS */
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

        if ($('temp-air')) $('temp-air').textContent = lastT_K ? `${(lastT_K - 273.15).toFixed(1)} °C` : 'N/A';
        if ($('pressure')) $('pressure').textContent = lastP_hPa ? `${lastP_hPa.toFixed(1)} hPa` : 'N/A';
        if ($('humidity')) $('humidity').textContent = lastH_perc ? `${(lastH_perc * 100).toFixed(0)} %` : 'N/A';
        if ($('env-factor')) $('env-factor').textContent = `${selectedEnvironment} (x${ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT})`;

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
    const alt_wgs84 = pos.coords.altitude, acc = pos.coords.accuracy;
    const geoidHeight = pos.coords.altitudeAccuracy; // Utilisé comme hauteur du géoïde si disponible
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

    // --- CORRECTION ALTITUDE MSL ---
    const alt_msl = correctAltitudeToMSL(alt_wgs84, geoidHeight);

    // 1. FILTRAGE DE L'ALTITUDE (via Kalman)
    const kAlt_new = kFilterAltitude(alt_msl, acc, dt);
    
    // 2. CALCUL DE LA VITESSE VERTICALE FILTRÉE
    if (lPos && lPos.kAlt_old !== undefined && dt > MIN_DT && alt_msl !== null) {
        spdV = (kAlt_new - lPos.kAlt_old) / dt;
    } else if (alt_msl !== null) {
        spdV = 0; 
    }
    
    // 3. CALCUL DE LA VITESSE HORIZONTALE BRUTE
    if (lPos && dt > 0.05) { 
        const dH = dist(lPos.coords.latitude, lPos.coords.longitude, lat, lon); 
        spdH = dH / dt; 
    } else if (spd !== null) {
        spdH = spd;
    }
    
    // 4. LOGIQUE HYBRIDE D'AMORTISSEMENT 3D BASÉE SUR LA PRÉCISION (ACC)
    let dampen_factor = 1.0;
    
    if (acc > ACC_DAMPEN_LOW) {
        const acc_range = ACC_DAMPEN_HIGH - ACC_DAMPEN_LOW;
        const current_excess = acc - ACC_DAMPEN_LOW;
        dampen_factor = 1.0 - Math.min(1.0, current_excess / acc_range);
        spdH *= dampen_factor;
        spdV *= dampen_factor; 
    }
    
    let spd3D = Math.sqrt(spdH ** 2 + spdV ** 2);

    // 5. CORRECTION DU LAG PAR CAPTEUR (FUSION IMU)
    if (lPos && lPos.speedMS_3D !== undefined && dt > MIN_DT) {
        const lastRawSpd = lPos.speedMS_3D;
        
        // Accélération horizontale mesurée par l'IMU (Non-simulée)
        const accel_mesuree_horiz = Math.sqrt(lastAccel.x ** 2 + lastAccel.y ** 2);
        
        // Estimation de la vitesse par intégration inertielle (Rattrapage du lag)
        const spd_inertielle_estimee = lastRawSpd + accel_mesuree_horiz * dt;
        
        // Fusion : Forte pondération pour l'inertiel (0.8) pour la réactivité
        const spd3D_fusion = (0.2 * spd3D) + (0.8 * spd_inertielle_estimee);
        
        // Anti-Spike sur la vitesse FUSIONNÉE
        const accelSpike = Math.abs(spd3D_fusion - lastRawSpd) / dt;
        
        if (accelSpike > MAX_PLAUSIBLE_ACCEL) {
            const maxPlausibleChange = MAX_PLAUSIBLE_ACCEL * dt;
            spd3D = spd3D_fusion > lastRawSpd 
                    ? lastRawSpd + maxPlausibleChange 
                    : lastRawSpd - maxPlausibleChange;
        } else {
             spd3D = spd3D_fusion; 
        }
    }

    // 6. FILTRE DE KALMAN FINAL
    const R_dyn = getKalmanR(acc, alt_msl, lastP_hPa); 
    const fSpd = kFilter(spd3D, dt, R_dyn), sSpdFE = fSpd < MIN_SPD ? 0 : fSpd;
    
    let accel_long = 0;
    if (dt > 0.05) {
        accel_long = (sSpdFE - lastFSpeed) / dt;
    }
    lastFSpeed = sSpdFE;

    distM += sSpdFE * dt * (netherMode ? NETHER_RATIO : 1); 
    if (sSpdFE > MIN_SPD) { timeMoving += dt; }
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    
    // --- MISE À JOUR DU DOM (GPS/Physique) ---
    if ($('latitude')) $('latitude').textContent = lat.toFixed(6);
    if ($('longitude')) $('longitude').textContent = lon.toFixed(6);
    // Affichage de l'Altitude MSL Filtrée
    if ($('altitude-gps')) $('altitude-gps').textContent = kAlt_new !== null ? `${kAlt_new.toFixed(2)} m (MSL)` : 'N/A';
    // Affichage de la correction Geoïde
    if ($('geoid-correction')) $('geoid-correction').textContent = geoidHeight !== undefined ? `${geoidHeight.toFixed(2)} m` : `${mslOffset.toFixed(2)} m (Est.)`;
    
    if ($('gps-precision')) $('gps-precision').textContent = `${acc.toFixed(2)} m`; 
    if ($('speed-raw-ms')) $('speed-raw-ms').textContent = `${spd3D.toFixed(2)} m/s`;
    if ($('vertical-speed')) $('vertical-speed').textContent = `${spdV.toFixed(2)} m/s`;
    if ($('underground-status')) $('underground-status').textContent = alt_msl !== null && alt_msl < ALT_TH ? `OUI (< ${ALT_TH}m)` : 'Non';
    if ($('accel-long')) $('accel-long').textContent = `${accel_long.toFixed(3)} m/s²`;
    if ($('force-g-long')) $('force-g-long').textContent = `${(accel_long / G_ACC).toFixed(2)} G`;
    if ($('speed-error-perc')) $('speed-error-perc').textContent = `${R_dyn.toFixed(3)} m² (R dyn)`; 
    if ($('speed-3d-inst')) $('speed-3d-inst').textContent = `${(spd3D * KMH_MS).toFixed(5)} km/h`; 
    if ($('speed-stable')) $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(5)} km/h`; 
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${sSpdFE.toFixed(2)} m/s | ${(sSpdFE * 1000).toFixed(0)} mm/s`;
    if ($('speed-max')) $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    if ($('speed-avg-moving')) $('speed-avg-moving').textContent = timeMoving > 1 ? `${(distM / timeMoving * KMH_MS).toFixed(5)} km/h` : '0.00000 km/h';
    if ($('distance-total-km')) $('distance-total-km').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    if ($('distance-cosmic')) $('distance-cosmic').textContent = `${(distM / C_L).toExponential(2)} s lumière | ${(distM / C_L / (dayMs / 1000 * 365.25)).toExponential(2)} al`;
    if ($('perc-speed-sound')) $('perc-speed-sound').textContent = `${(spd3D / C_S * 100).toFixed(2)} %`;
    if ($('perc-speed-c')) $('perc-speed-c').textContent = `${(spd3D / C_L * 100).toExponential(2)}%`;
    if ($('mode-nether')) $('mode-nether').textContent = netherMode ? "ACTIVÉ (1:8) 🔥" : "DÉSACTIVÉ (1:1)";

    if (Date.now() - (updateDisp.lastWeatherFetch ?? 0) > 60000) {
        fetchWeather(lat, lon); 
        updateDisp.lastWeatherFetch = Date.now();
    }
    
    // SAUVEGARDE
    lPos = pos; 
    lPos.speedMS_3D = spd3D; 
    lPos.timestamp = cTimePos; 
    lPos.kAlt_old = kAlt_new; 
}

// ===========================================
// FONCTIONS DE CAPTEURS (OPÉRATIONNELLES)
// ===========================================

/** 💡 Initialise les écouteurs pour les capteurs d'accélération */
function initSensors() {
    if ('ondevicemotion' in window) {
        window.addEventListener('devicemotion', (event) => {
            const acc = event.acceleration; // Accélération linéaire (sans gravité)
            const accG = event.accelerationIncludingGravity; // Accélération totale (avec gravité)
            
            if (acc) {
                // Utilise l'accélération linéaire si possible pour le calcul de mouvement
                lastAccel.x = acc.x ?? 0;
                lastAccel.y = acc.y ?? 0;
                lastAccel.z = acc.z ?? 0;
            } else if (accG) {
                // Sinon, utilise l'accélération totale (nécessiterait une correction de gravité/orientation)
                lastAccel.x = accG.x ?? 0;
                lastAccel.y = accG.y ?? 0;
                lastAccel.z = accG.z ?? 0;
            }
        }, true);
        console.log("Capteurs de mouvement (Accéléromètre) initialisés.");
    } else {
        console.warn("Capteurs de mouvement (IMU) non supportés/activés. Utilisation du GPS seul.");
    }
}


// ===========================================
// INITIALISATION DES ÉVÉNEMENTS DOM
// ===========================================

document.addEventListener('DOMContentLoaded', () => {
    
    const envSelect = document.createElement('select');
    envSelect.id = 'env-select';
    Object.keys(ENVIRONMENT_FACTORS).forEach(env => {
        const opt = document.createElement('option');
        opt.value = env; opt.textContent = env.toUpperCase();
        envSelect.appendChild(opt);
    });
    envSelect.value = selectedEnvironment;
    
    const envFactorElement = $('env-factor');
    if (envFactorElement) {
        const parent = envFactorElement.parentNode;
        parent.innerHTML = '<span class="label">Facteur Kalman Environnement</span>';
        parent.appendChild(envSelect);
    }
    
    syncH(); 

    if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', () => {
        if (emergencyStopActive) {
            alert("Veuillez désactiver l'Arrêt d'urgence avant d'utiliser ce contrôle.");
            return;
        }
        wID === null ? startGPS() : stopGPS();
    });
    if ($('freq-select')) $('freq-select').addEventListener('change', (e) => {
        if (emergencyStopActive) {
            alert("Veuillez désactiver l'Arrêt d'urgence.");
            return;
        }
        setGPSMode(e.target.value);
    });

    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', () => {
        if (emergencyStopActive) {
            resumeSystem(); 
        } else {
            emergencyStop(); 
        }
    });

    if ($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        netherMode = !netherMode; 
        if ($('mode-nether')) $('mode-nether').textContent = netherMode ? "ACTIVÉ (1:8) 🔥" : "DÉSACTIVÉ (1:1)"; 
    });
    if ($('env-select')) $('env-select').addEventListener('change', (e) => { 
        if (emergencyStopActive) return;
        selectedEnvironment = e.target.value; 
        if ($('env-factor')) $('env-factor').textContent = `${selectedEnvironment} (x${ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT})`; 
    });
    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => { if (emergencyStopActive) return; distM = 0; distMStartOffset = 0; timeMoving = 0; if ($('distance-total-km')) $('distance-total-km').textContent = `0.000 km | 0.00 m`; if ($('speed-avg-moving')) $('speed-avg-moving').textContent = `0.00000 km/h`; if ($('time-moving')) $('time-moving').textContent = `0.00 s`; });
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', () => { if (emergencyStopActive) return; maxSpd = 0; if ($('speed-max')) $('speed-max').textContent = `0.00000 km/h`; });
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { if (emergencyStopActive) return; if (confirm("Êtes-vous sûr de vouloir tout réinitialiser?")) { distM = 0; maxSpd = 0; distMStartOffset = 0; kSpd = 0; kUncert = 1000; timeMoving = 0; } });

    // Initialisation des capteurs IMU
    initSensors(); 
    
    startGPS(); 

    if (domID === null) {
        domID = setInterval(() => {
            if (lPos) updateAstro(lPos.coords.latitude, lPos.coords.longitude);
            else updateAstro(null, null); 
        }, DOM_SLOW_UPDATE_MS); 
    }
});
