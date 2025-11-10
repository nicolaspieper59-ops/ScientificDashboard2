// =================================================================
// FICHIER JS PARTIE 1/3 : gnss-dashboard-part1.js
// Contient constantes, variables d'état, UTC/NTP et Helpers.
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
// Q_NOISE AUGMENTÉ pour améliorer la réactivité aux accélérations/décélérations
const Q_NOISE = 0.05;       
// R_MIN DIMINUÉ pour donner plus de confiance aux précisions très faibles
const R_MIN = 0.01, R_MAX = 50.0; 
const MAX_ACC = 200, MIN_SPD = 0.05, ALT_TH = -50; 
const NETHER_RATIO = 8; 

// Filtre Anti-Spike de vitesse 3D (2G max d'accélération)
const MAX_PLAUSIBLE_ACCEL = 20.0; 

// PARAMÈTRES POUR LE FILTRE DE KALMAN D'ALTITUDE
const Q_ALT_NOISE = 0.1; // Bruit de processus de l'altitude
const R_ALT_MIN = 0.1;  // Erreur de mesure minimale de l'altitude

// PARAMÈTRES DE LISSAGE HYBRIDE (DAMPENING)
const ACC_DAMPEN_LOW = 5.0; 
const ACC_DAMPEN_HIGH = 50.0; 

// FACTEURS ENVIRONNEMENTAUX POUR LA CORRECTION KALMAN (Rendue plus agressive)
const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0 },
    'METAL': { R_MULT: 5.0 },      
    'FOREST': { R_MULT: 2.5 },     
    'CONCRETE': { R_MULT: 7.0 },   
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
// =================================================================
// FIN FICHIER JS PARTIE 1/3 : gnss-dashboard-part1.js
// =================================================================
// =================================================================
// FICHIER JS PARTIE 2/3 : gnss-dashboard-part2.js
// Contient les filtres de Kalman, l'Astro, les Controles et la Météo.
// NÉCESSITE gnss-dashboard-part1.js
// =================================================================

// ===========================================
// FILTRE DE KALMAN & FACTEUR R
// ===========================================

/** Applique le filtre de Kalman à la vitesse 3D en utilisant l'accélération pour la prédiction. */
function kFilter(nSpd, dt, R_dyn, accel_input = 0) { 
    if (dt === 0 || dt > 5) return kSpd; 
    const R = R_dyn ?? R_MAX, Q = Q_NOISE * dt; 

    // Prédiction améliorée: Vitesse précédente + (Accélération * temps)
    // Cela améliore la réactivité aux décélérations/accélérations (meilleure harmonie).
    let pSpd = kSpd + accel_input * dt; 
    let pUnc = kUncert + Q; 

    // Correction
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
    
    // Si la pression est disponible, elle affecte R
    if (P_hPa !== null) {
        const pressureFactor = 1.0 + (1013.25 - P_hPa) / 1013.25 * 0.1;
        R *= Math.max(1.0, pressureFactor);
    }
    
    // Facteur environnemental (simule la dégradation du signal)
    R *= envFactor;
    if (alt < ALT_TH) { R *= 2.0; } 

    R = Math.max(R_MIN, Math.min(R_MAX, R));
    
    return R;
}

// ===========================================
// FONCTIONS ASTRO & TEMPS
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
/** Calcule le Temps Solaire Vrai (TST). */
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
/** Calcule le temps Minecraft. */
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

// Fonction pour déterminer le nom de la phase lunaire (0.0 à 1.0)
function getMoonPhaseName(phase) {
    if (phase === 0 || phase === 1) return "Nouvelle Lune (🌑)";
    if (phase > 0 && phase < 0.25) return "Premier Croissant (🌒)";
    if (phase === 0.25) return "Premier Quartier (🌓)";
    if (phase > 0.25 && phase < 0.5) return "Gibbeuse Croissante (🌔)";
    if (phase === 0.5) return "Pleine Lune (🌕)";
    if (phase > 0.5 && phase < 0.75) return "Gibbeuse Décroissante (🌖)";
    if (phase === 0.75) return "Dernier Quartier (🌗)";
    if (phase > 0.75 && phase < 1) return "Dernier Croissant (🌘)";
    return "N/A";
}

// Mise à jour de l'horloge visuelle et des couleurs du corps (Day/Night)
function updateClockVisualization(now, sunPos, moonPos, sunTimes) {
    const sunEl = $('sun-element');
    const sunIcon = sunEl.querySelector('.sun-icon');
    const moonEl = $('moon-element');
    const moonIcon = moonEl.querySelector('.moon-icon');

    // 1. Mise à jour de la position du Soleil
    if (sunPos) {
        const altDeg = sunPos.altitude * R2D;
        const aziDeg = (sunPos.azimuth * R2D + 180) % 360; 
        sunEl.style.transform = `rotate(${aziDeg}deg)`;
        const radialPercent = Math.min(50, Math.max(0, 50 * (90 - altDeg) / 90));
        const altitudeOffsetPercent = 50 - radialPercent; 
        sunIcon.style.transform = `translateY(calc(-50% + ${altitudeOffsetPercent}%) )`; 
        sunEl.style.display = altDeg > -0.83 ? 'flex' : 'none'; 
    } else {
        sunEl.style.display = 'none';
    }

    // 2. Mise à jour de la position de la Lune
    if (moonPos) {
        const altDeg = moonPos.altitude * R2D;
        const aziDeg = (moonPos.azimuth * R2D + 180) % 360; 
        moonEl.style.transform = `rotate(${aziDeg}deg)`;
        const radialPercent = Math.min(50, Math.max(0, 50 * (90 - altDeg) / 90));
        const altitudeOffsetPercent = 50 - radialPercent; 
        moonIcon.style.transform = `translateY(calc(-50% + ${altitudeOffsetPercent}%) )`;
        moonEl.style.display = altDeg > 0 ? 'flex' : 'none';
    } else {
        moonEl.style.display = 'none';
    }
    
    // 3. Mise à jour du fond du corps et du statut
    const body = document.body;
    body.classList.remove('sky-day', 'sky-sunset', 'sky-night', 'sky-night-light', 'dark-mode');

    if (sunTimes) {
        const nowMs = now.getTime();
        let bodyClass;
        if (nowMs >= sunTimes.sunriseEnd.getTime() && nowMs < sunTimes.sunsetStart.getTime()) {
            bodyClass = 'sky-day';
        } else if (nowMs >= sunTimes.dusk.getTime() || nowMs < sunTimes.dawn.getTime()) {
            bodyClass = 'sky-night';
        } else {
            bodyClass = 'sky-sunset';
        }
        
        body.classList.add(bodyClass);
        body.classList.add(bodyClass === 'sky-day' ? 'light-mode' : 'dark-mode');

        $('clock-status').textContent = sunPos && sunPos.altitude > 0 ? 'Jour Solaire (☀️)' : 'Nuit/Crépuscule (🌙)';
    } else {
        $('clock-status').textContent = 'Position Solaire Indisponible';
    }
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
    const moonPos = window.SunCalc ? SunCalc.getMoonPosition(now, latA, lonA) : null;
    const sunTimes = window.SunCalc ? SunCalc.getTimes(now, latA, lonA) : null;
    const moonTimes = window.SunCalc ? SunCalc.getMoonTimes(now, latA, lonA, true) : null;
    const solarTimes = getSolarTime(now, lonA);

    $('local-time').textContent = now.toLocaleTimeString('fr-FR', { timeZone: 'UTC', hour12: false });
    if ($('date-display')) $('date-display').textContent = now.toLocaleDateString();
    if (sTime) {
        const timeElapsed = (now.getTime() - sTime) / 1000;
        $('time-elapsed').textContent = `${timeElapsed.toFixed(2)} s`;
        $('time-moving').textContent = `${timeMoving.toFixed(2)} s`;
        $('time-minecraft').textContent = getMinecraftTime(now);
    }
    
    if ($('time-solar-true')) $('time-solar-true').textContent = solarTimes.TST;
    
    // --- NOUVELLES DONNÉES ASTRO ---
    if ($('sun-altitude')) $('sun-altitude').textContent = sunPos ? `${(sunPos.altitude * R2D).toFixed(2)} °` : 'N/A';
    if ($('sun-azimuth')) $('sun-azimuth').textContent = sunPos ? `${(sunPos.azimuth * R2D).toFixed(2)} ° (S-O)` : 'N/A';
    
    if ($('moon-altitude')) $('moon-altitude').textContent = moonPos ? `${(moonPos.altitude * R2D).toFixed(2)} °` : 'N/A';
    if ($('moon-azimuth')) $('moon-azimuth').textContent = moonPos ? `${(moonPos.azimuth * R2D).toFixed(2)} ° (S-O)` : 'N/A';
    
    if ($('moon-illum-fraction')) $('moon-illum-fraction').textContent = moonIllum ? `${(moonIllum.fraction * 100).toFixed(1)} %` : 'N/A';
    if ($('moon-phase-name')) $('moon-phase-name').textContent = moonIllum ? getMoonPhaseName(moonIllum.phase) : 'N/A';
    
    if (window.SunCalc && sunPos && moonPos) {
        updateClockVisualization(now, sunPos, moonPos, sunTimes);
    } else if ($('clock-status')) {
        $('clock-status').textContent = 'Initialisation TST...';
    }

    if ($('culmination-lsm')) $('culmination-lsm').textContent = solarTimes.MST;
    if ($('noon-solar')) $('noon-solar').textContent = sunTimes && sunTimes.solarNoon ? sunTimes.solarNoon.toLocaleTimeString() : 'N/D';
    if ($('eot-min')) $('eot-min').textContent = solarTimes.EOT + ' min'; 
    if ($('ecliptic-long')) $('ecliptic-long').textContent = solarTimes.ECL_LONG + ' °';

    if ($('day-duration') && sunTimes && sunTimes.sunrise && sunTimes.sunset) {
        const durationMs = sunTimes.sunset.getTime() - sunTimes.sunrise.getTime();
        const hours = Math.floor(durationMs / 3600000);
        const minutes = Math.floor((durationMs % 3600000) / 60000);
        $('day-duration').textContent = `${hours}h ${minutes}m`;
    } else if ($('day-duration')) {
        $('day-duration').textContent = 'N/A (Polaire/Nuit)';
    }

    if ($('moon-times')) $('moon-times').textContent = moonTimes ? 
        `↑ ${moonTimes.rise ? moonTimes.rise.toLocaleTimeString() : 'N/A'} / ↓ ${moonTimes.set ? moonTimes.set.toLocaleTimeString() : 'N/A'}` : 'N/D';
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
// =================================================================
// FIN FICHIER JS PARTIE 2/3 : gnss-dashboard-part2.js
// =================================================================
// =================================================================
// FICHIER JS PARTIE 3/3 : gnss-dashboard-part3.js
// Contient la boucle principale de mise à jour GPS et l'initialisation DOM.
// NÉCESSITE gnss-dashboard-part1.js et gnss-dashboard-part2.js
// =================================================================

// ===========================================
// FONCTION PRINCIPALE DE MISE À JOUR GPS 
// ===========================================

function updateDisp(pos) {
    if (emergencyStopActive) return;

    // Récupération des données brutes
    let currentLat = pos.coords.latitude, currentLon = pos.coords.longitude;
    const alt = pos.coords.altitude, acc = pos.coords.accuracy;
    const spd = pos.coords.speed;
    const cTimePos = pos.timestamp; // Heure de la mesure GPS

    const now = getCDate(); 
    if (now === null) { updateAstro(currentLat, currentLon); return; } 

    if (sTime === null) { sTime = now.getTime(); distMStartOffset = distM; }
    
    // --- DÉBUT LOGIQUE NAVIGATION MORTE / HARMONIE ---
    let isSignalLost = false;
    let kAlt_new = kAlt; // Initialisation par défaut pour la perte de signal
    
    if (acc > MAX_ACC) { 
        if ($('gps-precision')) $('gps-precision').textContent = `❌ ${acc.toFixed(0)} m (Signal Perdu/Trop Imprécis)`; 
        isSignalLost = true;
        // Si signal perdu, on conserve la dernière position connue pour le calcul dt
        if (lPos === null) lPos = pos; 
        currentLat = lat; currentLon = lon; // Utiliser les dernières coordonnées fiables
    } else {
        // Mettre à jour les coordonnées globales si le signal est bon
        lat = currentLat; lon = currentLon;
        // 1. FILTRAGE DE L'ALTITUDE (via Kalman) - Seulement si le signal est bon
        kAlt_new = kFilterAltitude(alt, acc, dt);
    }

    let spdH = spd ?? 0;
    let spdV = 0; 
    const dt = lPos ? (cTimePos - lPos.timestamp) / 1000 : MIN_DT;

    // 2. CALCUL DE LA VITESSE VERTICALE FILTRÉE
    if (lPos && lPos.kAlt_old !== undefined && dt > MIN_DT && alt !== null) {
        spdV = (kAlt_new - lPos.kAlt_old) / dt;
    } else if (alt !== null) {
        spdV = 0; 
    }
    
    // 3. CALCUL DE LA VITESSE HORIZONTALE BRUTE
    if (lPos && dt > 0.05) { 
        const dH = dist(lPos.coords.latitude, lPos.coords.longitude, currentLat, currentLon); 
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

    // 5. CORRECTION ANTI-SPIKE DE VITESSE
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

    // Calcul de l'accélération 3D brute pour la prédiction Kalman (IMU simulé)
    let raw_accel_3d = 0;
    if (lPos && lPos.speedMS_3D !== undefined && dt > MIN_DT) {
        raw_accel_3d = (spd3D - lPos.speedMS_3D) / dt;
        // Cap l'accélération brute pour éviter d'introduire un bruit trop fort dans la prédiction.
        raw_accel_3d = Math.min(MAX_PLAUSIBLE_ACCEL, Math.max(-MAX_PLAUSIBLE_ACCEL, raw_accel_3d));
    }

    // Détermine la vitesse de mesure (nSpd) à fournir au filtre
    const nSpd = isSignalLost ? kSpd : spd3D; 
    
    // 6. FILTRE DE KALMAN FINAL (Stabilité temporelle)
    let R_dyn = getKalmanR(acc, alt, lastP_hPa); 
    
    if (isSignalLost) {
        // Si signal perdu, on augmente R pour que le filtre ignore complètement le GPS
        R_dyn = R_MAX * 10; 
        if ($('speed-stable')) $('speed-stable').textContent = `~ ${(kSpd * KMH_MS).toFixed(5)} km/h (ESTIMATION)`; 
    } else {
        // Réinitialiser l'affichage si le signal est revenu
        if ($('speed-stable') && $('speed-stable').textContent.includes('(ESTIMATION)')) {
            $('speed-stable').textContent = `...`; 
        }
    }

    // Appel du filtre avec l'accélération 3D brute pour la prédiction
    const fSpd = kFilter(nSpd, dt, R_dyn, raw_accel_3d), sSpdFE = fSpd < MIN_SPD ? 0 : fSpd;
    
    // --- FIN LOGIQUE NAVIGATION MORTE / HARMONIE ---

    // Calculs d'accélération et distance (utilisent sSpdFE, la vitesse la plus stable)
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
    if ($('gps-precision') && !isSignalLost) $('gps-precision').textContent = `${acc.toFixed(2)} m`; 
    if ($('speed-raw-ms')) $('speed-raw-ms').textContent = `${spd3D.toFixed(2)} m/s`;
    if ($('vertical-speed')) $('vertical-speed').textContent = `${spdV.toFixed(2)} m/s`;
    if ($('underground-status')) $('underground-status').textContent = alt !== null && alt < ALT_TH ? `OUI (< ${ALT_TH}m)` : 'Non';
    if ($('accel-long')) $('accel-long').textContent = `${accel_long.toFixed(3)} m/s²`;
    if ($('force-g-long')) $('force-g-long').textContent = `${(accel_long / G_ACC).toFixed(2)} G`;
    if ($('speed-error-perc')) $('speed-error-perc').textContent = `${R_dyn.toFixed(3)} m² (R dyn)`; 
    if ($('speed-3d-inst')) $('speed-3d-inst').textContent = `${(spd3D * KMH_MS).toFixed(5)} km/h`; 
    if ($('speed-stable') && !isSignalLost) $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(5)} km/h`; 
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
    
    // SAUVEGARDE DES VALEURS POUR LA PROCHAINE ITÉRATION (Seulement si le signal est bon)
    if (!isSignalLost) {
        lPos = pos; 
        lPos.speedMS_3D = spd3D; 
        lPos.timestamp = cTimePos; 
        lPos.kAlt_old = kAlt_new; 
    }
}


// ===========================================
// INITIALISATION DES ÉVÉNEMENTS DOM
// ===========================================

document.addEventListener('DOMContentLoaded', () => {
    
    // Logique pour injecter la sélection d'environnement
    const envSelect = document.createElement('select');
    envSelect.id = 'env-select';
    Object.keys(ENVIRONMENT_FACTORS).forEach(env => {
        const opt = document.createElement('option');
        opt.value = env; opt.textContent = env.toUpperCase();
        envSelect.appendChild(opt);
    });
    envSelect.value = selectedEnvironment;
    
    const envFactorElement = $('environment-select');
    if (envFactorElement) {
        const parent = envFactorElement.parentNode;
        parent.replaceChild(envSelect, envFactorElement);
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
// =================================================================
// FIN FICHIER JS PARTIE 3/3 : gnss-dashboard-part3.js
// =================================================================
