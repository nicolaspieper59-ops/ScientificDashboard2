// =================================================================
// FICHIER JS PARTIE 1/2 : gnss-dashboard-part1.js
// Contient constantes, variables d'état, UTC/NTP, Kalman, Astro, Map et INS Core.
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

// Filtre Anti-Spike de vitesse 3D (2G max d'accélération)
const MAX_PLAUSIBLE_ACCEL = 20.0; 

// PARAMÈTRES POUR LE FILTRE DE KALMAN D'ALTITUDE
const Q_ALT_NOISE = 0.1; 
const R_ALT_MIN = 0.1;  

// PARAMÈTRES INS (ACCÉLÉROMÈTRE)
const ACCEL_NOISE = 0.05; // Bruit du capteur d'accélération
const K_ACCEL_ALPHA = 0.3; // Facteur de lissage pour l'accélération (filtre passe-bas)

// PARAMÈTRES DE LISSAGE HYBRIDE (DAMPENING)
const ACC_DAMPEN_LOW = 5.0; 
const ACC_DAMPEN_HIGH = 50.0; 

// --- CORRECTION EMPIRIQUE DE VITESSE (CALIBRATION) ---
// 1.0 = Aucune correction. Utiliser > 1.0 si vitesse sous-estimée, < 1.0 si surestimée.
const EMPIRICAL_CORR_FACTOR = 1.0; 

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

// --- VARIABLES INS ---
let kAccel = 0; // Accélération inertielle filtrée (m/s²)
let lastAccelTime = null;

// --- MAP VARIABLES ---
let map = null;
let marker = null;
let accuracyCircle = null;


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
// SYNCHRONISATION HORAIRE
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
function getCDate() { 
    if (lServH === null || lLocH === null) { return null; }
    const offsetSinceSync = performance.now() - lLocH;
    return new Date(lServH + offsetSinceSync); 
}

// ===========================================
// FILTRE DE KALMAN & FUSION INS
// ===========================================

/** Applique le filtre de Kalman à la vitesse 3D (Fusion INS/GNSS). */
function kFilter(nSpd, dt, R_dyn, accel_inertial) {
    if (dt === 0 || dt > 5) return kSpd; 
    let pSpd = kSpd + accel_inertial * dt; 
    const R_accel = ACCEL_NOISE; 
    const Q = Q_NOISE * dt + R_accel * dt; 
    let pUnc = kUncert + Q; 
    const R = R_dyn ?? R_MAX; 
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

// ===========================================
// FONCTIONS ASTRO & TEMPS 
// ===========================================

function toDays(date) { return (date.valueOf() / dayMs - 0.5 + J1970) - J2000; }
function solarMeanAnomaly(d) { return D2R * (356.0470 + 0.9856002585 * d); }
function eclipticLongitude(M) {
    var C = D2R * (1.9148 * Math.sin(M) + 0.0200 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M)), P = D2R * 102.9377;                                                                
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
    const eot_rad = alpha - M - D2R * 102.9377 - Math.PI;
    const eot_min = eot_rad * 4 * R2D; 
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
function getMinecraftTime(date) {
    if (date === null) return '00:00:00';
    const msSinceMidnightUTC = date.getUTCHours() * 3600000 + date.getUTCMilliseconds() + date.getUTCMinutes() * 60000 + date.getUTCSeconds() * 1000;
    const timeRatio = (msSinceMidnightUTC % dayMs) / dayMs;
    const mcTimeMs = (timeRatio * MC_DAY_MS + MC_DAY_MS) % MC_DAY_MS;
    const toTimeString = (ms) => {
        let h = Math.floor(ms / 3600000);
        let m = Math.floor((ms % 3600000) / 60000);
        let s = Math.floor((ms % 60000) / 1000);
        return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
    };
    return toTimeString(mcTimeMs);
}
function updateAstro(latA, lonA) {
    const now = getCDate(); 
    if (now === null) {
        if ($('local-time') && !$('local-time').textContent.includes('Synchronisation')) {
             $('local-time').textContent = 'Synchronisation...';
        }
        return;
    }
    const sunPos = window.SunCalc ? SunCalc.getPosition(now, latA, lonA) : null;
    const moonIllum = window.SunCalc ? SunCalc.getMoonIllumination(now) : null;
    const sunTimes = window.SunCalc ? SunCalc.getTimes(now, latA, lonA) : null;
    const solarTimes = getSolarTime(now, lonA);
    $('local-time').textContent = now.toLocaleTimeString('fr-FR', { timeZone: 'UTC', hour12: false });
    if ($('date-display')) $('date-display').textContent = now.toLocaleDateString();
    if (sTime) {
        const timeElapsed = (now.getTime() - sTime) / 1000;
        $('time-elapsed').textContent = `${timeElapsed.toFixed(2)} s`;
        $('time-moving').textContent = `${timeMoving.toFixed(2)} s`;
        $('time-minecraft').textContent = getMinecraftTime(now);
    }
    if ($('tst')) $('tst').textContent = solarTimes.TST;
    if ($('lsm')) $('lsm').textContent = solarTimes.MST;
    if ($('sun-elevation')) $('sun-elevation').textContent = sunPos ? `${(sunPos.altitude * R2D).toFixed(2)} °` : 'N/A';
    if ($('noon-solar')) $('noon-solar').textContent = sunTimes && sunTimes.solarNoon ? sunTimes.solarNoon.toLocaleTimeString() : 'N/D';
    if ($('eot')) $('eot').textContent = solarTimes.EOT + ' min'; 
    if ($('ecliptic-long')) $('ecliptic-long').textContent = solarTimes.ECL_LONG + ' °';
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
// FONCTIONS CARTE (LEAFLET)
// ===========================================

function mapInit(latA = 43.3, lonA = 5.4) { 
    if (map === null && document.getElementById('map-container')) {
        map = L.map('map-container').setView([latA, lonA], 14);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
        }).addTo(map);
        marker = L.marker([latA, lonA]).addTo(map)
            .bindPopup("Dernière Position").openPopup();
        accuracyCircle = L.circle([latA, lonA], 0).addTo(map);
    }
}
function updateMap(latA, lonA, acc) {
    if (map && marker) {
        const newLatLng = new L.LatLng(latA, lonA);
        marker.setLatLng(newLatLng);
        if (accuracyCircle) {
             accuracyCircle.setLatLng(newLatLng).setRadius(acc).redraw();
        }
        if (map.getZoom() > 10) {
            map.setView(newLatLng, map.getZoom());
        }
    }
}

// ===========================================
// GESTIONNAIRES CAPTEURS INERTIELS (INS)
// ===========================================

/** Gère les événements d'accélération (pour la fusion INS). */
function handleDeviceMotion(event) {
    const ax = event.accelerationIncludingGravity.x ?? 0;
    const ay = event.accelerationIncludingGravity.y ?? 0;
    const az = event.accelerationIncludingGravity.z ?? 0;
    const rawAccel = Math.sqrt(ax * ax + ay * ay + az * az) - G_ACC; 
    const accel_move = Math.max(0, rawAccel); 
    
    kAccel = kAccel * (1 - K_ACCEL_ALPHA) + accel_move * K_ACCEL_ALPHA;
    lastAccelTime = performance.now();
    
    if ($('accel-inertial')) $('accel-inertial').textContent = `${kAccel.toFixed(3)} m/s²`;
}

/** Demande l'accès et démarre les écouteurs de capteurs inertiels. */
function startInertialSensors() {
    if (window.DeviceMotionEvent) {
        if (typeof DeviceMotionEvent.requestPermission === 'function') {
            DeviceMotionEvent.requestPermission().then(permissionState => {
                if (permissionState === 'granted') {
                    window.addEventListener('devicemotion', handleDeviceMotion);
                    console.log("Accès aux capteurs inertiels accordé.");
                } else {
                    console.warn("Accès aux capteurs inertiels refusé.");
                }
            }).catch(console.error);
        } else {
            window.addEventListener('devicemotion', handleDeviceMotion);
            console.log("Capteurs inertiels démarrés.");
        }
    } else {
        console.warn("API DeviceMotionEvent non supportée. Fusion INS désactivée.");
        if ($('accel-inertial')) $('accel-inertial').textContent = 'N/A (API non supportée)';
    }
    }
// =================================================================
// FICHIER JS PARTIE 2/2 : gnss-dashboard-part2.js
// Contient les fonctions de contrôle GPS, l'API Météo et la boucle principale (updateDisp).
// =================================================================

// Les variables et fonctions core sont définies dans gnss-dashboard-part1.js

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
    if (!latA || !lonA) { return; }
    try {
        const response = await fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${latA}&lon=${lonA}`);
        const data = await response.json();
        
        if (data.main) {
            lastP_hPa = data.main.pressure; 
            lastT_K = data.main.temp + 273.15; // Température en Kelvin
            lastH_perc = data.main.humidity / 100; 
        }

        if ($('temp-air')) $('temp-air').textContent = lastT_K ? `${(lastT_K - 273.15).toFixed(1)} °C` : 'N/A';
        if ($('pressure')) $('pressure').textContent = lastP_hPa ? `${lastP_hPa.toFixed(1)} hPa` : 'N/A';
        if ($('humidity')) $('humidity').textContent = lastH_perc ? `${(lastH_perc * 100).toFixed(0)} %` : 'N/A';
        
        // Calcul du Point de Rosée (Dew Point)
        if (lastT_K !== null && lastH_perc !== null) {
            const T_c = lastT_K - 273.15;
            const RH = lastH_perc;
            const a = 17.27, b = 237.7;
            const alpha = a * T_c / (b + T_c) + Math.log(RH);
            const dewPoint_c = b * alpha / (a - alpha);
            if ($('dew-point')) $('dew-point').textContent = `${dewPoint_c.toFixed(1)} °C`;
        } else {
             if ($('dew-point')) $('dew-point').textContent = 'N/A';
        }
        
        if (data.wind && $('wind-direction')) {
             $('wind-direction').textContent = data.wind.deg ? `${data.wind.deg}°` : 'N/A'; 
        }
        
        if (data.wind && $('wind-speed-ms')) {
             $('wind-speed-ms').textContent = data.wind.speed ? `${data.wind.speed.toFixed(2)} m/s` : 'N/A'; 
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
    if (lPos && lPos.kAlt_old !== undefined && dt > MIN_DT && alt !== null) {
        spdV = (kAlt_new - lPos.kAlt_old) / dt;
    } else if (alt !== null) {
        spdV = 0; 
    }
    
    // 3. CALCUL DE LA VITESSE HORIZONTALE BRUTE
    if (lPos && dt > 0.05) { 
        const dH = dist(lPos.coords.latitude, lPos.coords.longitude, lat, lon); 
        spdH = dH / dt; 
    } else if (spd !== null) {
        spdH = spd;
    }
    
    // 4. LOGIQUE HYBRIDE D'AMORTISSEMENT 3D
    let dampen_factor = 1.0;
    if (acc > ACC_DAMPEN_LOW) {
        const acc_range = ACC_DAMPEN_HIGH - ACC_DAMPEN_LOW;
        const current_excess = acc - ACC_DAMPEN_LOW;
        dampen_factor = 1.0 - Math.min(1.0, current_excess / acc_range);
        spdH *= dampen_factor;
        spdV *= dampen_factor; 
    }
    
    let spd3D = Math.sqrt(spdH ** 2 + spdV ** 2);

    // 5. CORRECTION ANTI-SPIKE DE VITESSE (BRUTE)
    if (lPos && lPos.speedMS_3D !== undefined && dt > MIN_DT) {
        const lastRawSpd = lPos.speedMS_3D;
        const accelSpike = Math.abs(spd3D - lastRawSpd) / dt;
        
        if (accelSpike > MAX_PLAUSIBLE_ACCEL) {
            const maxPlausibleChange = MAX_PLAUSIBLE_ACCEL * dt;
            if (spd3D > lastRawSpd) {
                spd3D = lastRawSpd + maxPlausibleChange;
            } else {
                spd3D = lastRawSpd - maxPlausibleChange;
            }
        }
    }

    // 6. FILTRE DE KALMAN FINAL (Stabilité temporelle) - AVEC PRÉDICTION INS (kAccel)
    const R_dyn = getKalmanR(acc, alt, lastP_hPa); 
    const fSpd = kFilter(spd3D, dt, R_dyn, kAccel); 
    
    // NOUVEAU: Application du facteur de correction empirique (Calibration)
    let sSpdFE = fSpd * EMPIRICAL_CORR_FACTOR;
    if (sSpdFE < MIN_SPD) sSpdFE = 0;
    
    // Calculs d'accélération et distance
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
    if ($('altitude-gps')) $('altitude-gps').textContent = kAlt_new !== null ? `${kAlt_new.toFixed(2)} m` : 'N/A';
    if ($('gps-precision')) $('gps-precision').textContent = `${acc.toFixed(2)} m`; 
    if ($('speed-raw-ms')) $('speed-raw-ms').textContent = `${spd3D.toFixed(2)} m/s`;
    if ($('vertical-speed')) $('vertical-speed').textContent = `${spdV.toFixed(2)} m/s`;
    if ($('underground-status')) $('underground-status').textContent = alt !== null && alt < ALT_TH ? `OUI (< ${ALT_TH}m)` : 'Non';
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

    // --- MISE À JOUR DE LA CARTE ---
    updateMap(lat, lon, acc); 

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
    
    const envSelect = document.createElement('select');
    envSelect.id = 'environment-select'; 
    Object.keys(ENVIRONMENT_FACTORS).forEach(env => {
        const opt = document.createElement('option');
        opt.value = env; opt.textContent = `${env.toUpperCase()} (x${ENVIRONMENT_FACTORS[env].R_MULT})`;
        envSelect.appendChild(opt);
    });
    envSelect.value = selectedEnvironment;
    
    const envContainer = document.querySelector('.control-input-full label[for="environment-select"]').parentNode;
    if (envContainer) {
        const existingSelect = envContainer.querySelector('#environment-select');
        if (existingSelect) {
            existingSelect.remove();
        }
        envContainer.appendChild(envSelect);
    }
    
    mapInit(); 
    syncH(); 
    startInertialSensors(); 

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
    if ($('environment-select')) $('environment-select').addEventListener('change', (e) => { 
        if (emergencyStopActive) return;
        selectedEnvironment = e.target.value; 
    });
    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => { if (emergencyStopActive) return; distM = 0; distMStartOffset = 0; timeMoving = 0; if ($('distance-total-km')) $('distance-total-km').textContent = `0.000 km | 0.00 m`; if ($('speed-avg-moving')) $('speed-avg-moving').textContent = `0.00000 km/h`; if ($('time-moving')) $('time-moving').textContent = `0.00 s`; });
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', () => { if (emergencyStopActive) return; maxSpd = 0; if ($('speed-max')) $('speed-max').textContent = `0.00000 km/h`; });
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { if (emergencyStopActive) return; if (confirm("Êtes-vous sûr de vouloir tout réinitialiser?")) { distM = 0; maxSpd = 0; distMStartOffset = 0; kSpd = 0; kUncert = 1000; timeMoving = 0; } });


    startGPS(); 

    if (domID === null) {
        domID = setInterval(() => {
            if (lPos) updateAstro(lPos.coords.latitude, lPos.coords.longitude);
            else updateAstro(null, null); 
        }, DOM_SLOW_UPDATE_MS); 
    }
});
