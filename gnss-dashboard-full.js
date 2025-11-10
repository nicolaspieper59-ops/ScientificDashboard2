// =================================================================
// GNSS/IMU DASHBOARD - EKF FUSION CONTINUE (FICHIER COMPLET CORRIGÉ)
// =================================================================

// --- CLÉS D'API & PROXY VERCEL ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app"; 
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc"; 

// --- CONSTANTES GLOBALES ET PHYSIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458; 
const R_E_BASE = 6371000;   // Rayon terrestre de base
const KMH_MS = 3.6;    
const C_S = 343;       
const MC_DAY_MS = 72 * 60 * 1000; 

const J1970 = 2440588, J2000 = 2451545; 
const dayMs = 1000 * 60 * 60 * 24;      
const MIN_DT = 0.01; 
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};

// --- PARAMÈTRES EKF (Filtre de Kalman) ---
const Q_NOISE = 0.01; // Bruit de prédiction (confiance dans l'IMU)
const R_MIN = 0.05;   // Bruit de mesure GPS minimum (GPS parfait)
const R_MAX = 50.0;   // Bruit de mesure GPS maximum (avant rejet)
const MAX_ACC = 200;  // Précision GPS max (m) avant le mode Estimation Seule
const MIN_SPD = 0.05; // Seuil de vitesse (m/s)
const ALT_TH = -50;   // Seuil de sous-sol (m)
const MAX_PLAUSIBLE_ACCEL = 20.0; // Anti-spike (m/s²)

// PARAMÈTRES EKF D'ALTITUDE
const Q_ALT_NOISE = 0.1; 
const R_ALT_MIN = 0.1;  

// FACTEURS ENVIRONNEMENTAUX (POUR R)
const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0, DISPLAY: 'Normal' },
    'METAL': { R_MULT: 2.5, DISPLAY: 'Métal/Bâtiment' },      
    'FOREST': { R_MULT: 1.5, DISPLAY: 'Forêt' },     
    'CONCRETE': { R_MULT: 3.0, DISPLAY: 'Grotte/Tunnel' },   
};
const DOM_SLOW_UPDATE_MS = 1000; 

// --- DONNÉES CÉLESTES/GRAVITÉ (Logique MRF/Artificielle) ---
const CELESTIAL_DATA = {
    'EARTH': { G: 9.80665, R: R_E_BASE, name: 'Terre' },
    'MOON': { G: 1.62, R: 1737400, name: 'Lune' },
    'MARS': { G: 3.71, R: 3389500, name: 'Mars' },
    'ROTATING': { G: 0.0, R: R_E_BASE, name: 'Station Spatiale' }
};
let G_ACC = CELESTIAL_DATA['EARTH'].G; // Gravité effective (m/s²)
let R_ALT_CENTER_REF = CELESTIAL_DATA['EARTH'].R; // Rayon de référence pour MRF

// --- VARIABLES D'ÉTAT ---
let wID = null, domID = null, lPos = null, lat = null, lon = null, sTime = null;
let distM = 0, maxSpd = 0;
let kSpd = 0, kUncert = 1000; 
let timeMoving = 0; 
let lServH = null, lLocH = null; 
let lastFSpeed = 0; 
let kAlt = null;      
let kAltUncert = 10;  
let currentGPSMode = 'HIGH_FREQ'; 
let emergencyStopActive = false;
let selectedEnvironment = 'NORMAL'; 
let currentMass = 70.0; // Assuré d'être un nombre
let R_FACTOR_RATIO = 1.0;   // Facteur de Rapport de Mouvement (MRF)
let currentCelestialBody = 'EARTH';
let rotationRadius = 100;
let angularVelocity = 0.0;
let lastP_hPa = null, lastT_K = null, lastH_perc = null; 
let map, marker, circle, lastMapUpdate = 0;

// --- REFERENCES DOM ---
const $ = id => document.getElementById(id);

// ===========================================
// FONCTIONS GÉO & UTILS
// ===========================================
const dist = (lat1, lon1, lat2, lon2) => {
    const R = R_ALT_CENTER_REF; // Utilise le rayon du corps céleste
    const dLat = (lat2 - lat1) * D2R, dLon = (lon2 - lon1) * D2R;
    lat1 *= D2R; lat2 *= D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
};

// ===========================================
// SYNCHRONISATION HORAIRE (NTP)
// ===========================================
async function syncH() { 
    if ($('local-time')) $('local-time').textContent = 'Synchronisation UTC...';
    try {
        const response = await fetch(SERVER_TIME_ENDPOINT, { cache: "no-store", mode: "cors" });
        if (!response.ok) throw new Error(`Server time sync failed: ${response.statusText}`);
        const serverData = await response.json(); 
        const serverTimestamp = Date.parse(serverData.utc_datetime); // Correction : utilise utc_datetime
        lServH = serverTimestamp; 
        lLocH = Date.now(); // Utilise Date.now() pour la synchro
    } catch (error) {
        console.warn("Échec de la synchronisation. Utilisation de l'horloge locale.", error);
        lServH = Date.now(); 
        lLocH = Date.now();
    }
}

function getCDate() { 
    if (lServH === null || lLocH === null) { return null; }
    const offsetSinceSync = Date.now() - lLocH;
    return new Date(lServH + offsetSinceSync); 
}

// ===========================================
// FILTRE DE KALMAN & LOGIQUE DE FUSION
// ===========================================

/** * Applique le filtre de Kalman à la vitesse 3D (Fusion IMU/GNSS). 
 * C'est ici que le "bruit gps" est corrigé par les "capteurs".
 */
function kFilter(nSpd, dt, R_dyn, accel_input = 0) {
    if (dt === 0 || dt > 5) return kSpd; 
    const R = R_dyn ?? R_MAX, Q = Q_NOISE * dt; 
    
    // Étape de PRÉDICTION (Modèle + Accéléromètre IMU)
    // pSpd = dernière vitesse filtrée + accélération mesurée par les capteurs
    let pSpd = kSpd + (accel_input * dt); 
    let pUnc = kUncert + Q; 
    
    // Calcul du Gain de Kalman (K)
    // Si R (GPS bruité) est grand, K est petit.
    // Si R (GPS précis) est petit, K est grand.
    let K = pUnc / (pUnc + R); 
    
    // Étape de CORRECTION (Fusion : combine prédiction et mesure GPS)
    // Vitesse = Prédiction + K * (Erreur de mesure GPS)
    kSpd = pSpd + K * (nSpd - pSpd); 
    kUncert = (1 - K) * pUnc; 
    return kSpd;
}

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

/** * Calcule le Facteur R (Confiance GPS) du filtre de Kalman.
 * Gère le mode "Estimation Seule" (Dead Reckoning).
 */
function getKalmanR(acc, alt, P_hPa) {
    
    // CORRECTION : Mode Estimation Seule (Dead Reckoning)
    // Si le GPS est arrêté ou perdu (MAX_ACC = 200m)
    if (acc > MAX_ACC) {
        return 1e9; // R = infini. K (Gain) = 0. Force le filtre à ignorer le GPS.
    }
    
    let R = acc ** 2; // Le bruit est le carré de la précision
    R = Math.max(R_MIN, Math.min(R_MAX, R));
    
    const envFactor = ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT;
    
    if (P_hPa !== null) {
        const pressureFactor = 1.0 + (1013.25 - P_hPa) / 1013.25 * 0.1;
        R *= Math.max(1.0, pressureFactor);
    }
    
    R *= envFactor;
    if (alt < ALT_TH) { R *= 2.0; } // Pénalité sous-sol

    return Math.max(R_MIN, R);
}

// ===========================================
// LOGIQUE MRF & GRAVITÉ ARTIFICIELLE
// ===========================================

/** Calcule le Facteur de Rapport de Mouvement (MRF) */
function calculateMRF(kAltA) {
    if (kAltA === null || R_ALT_CENTER_REF === null) return 1.0;
    const R_current = R_ALT_CENTER_REF + kAltA; 
    const R_ref = R_ALT_CENTER_REF;
    const ratio = R_current / R_ref;
    return Math.max(0.001, Math.min(1.5, ratio)); // Plancher pour éviter 0
}

/** Calcule la Gravité Artificielle (Force Centrifuge) */
function calculateArtificialGravity() {
    if (angularVelocity === 0 || rotationRadius === 0) return 0;
    // g_art = omega^2 * r
    return angularVelocity ** 2 * rotationRadius;
}

/** Met à jour les constantes physiques pour le corps céleste sélectionné. */
function updateCelestialBody(body) {
    currentCelestialBody = body;
    let newG = 0;
    let newR = R_E_BASE;

    if (body === 'ROTATING') {
        const g_art = calculateArtificialGravity();
        newG = g_art; 
        newR = R_E_BASE; 
        if ($('gravity-local')) $('gravity-local').textContent = `${newG.toFixed(4)} m/s² (Centrifuge)`;
    } else {
        const data = CELESTIAL_DATA[body];
        newG = data.G;
        newR = data.R;
        if ($('gravity-local')) $('gravity-local').textContent = `${newG.toFixed(4)} m/s²`;
    }
    
    G_ACC = newG;
    R_ALT_CENTER_REF = newR; // Le rayon du corps est la référence pour le MRF
    kAlt = null; kAltUncert = 10;
    console.log(`Corps céleste réglé sur ${body}. G_ACC: ${G_ACC}, R_REF: ${R_ALT_CENTER_REF}`);
}

// ===========================================
// FONCTIONS ASTRO & TEMPS 
// ===========================================

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
    if (now === null || !latA || !lonA) {
        // Ne pas mettre à jour l'astro si les données de base manquent
        return;
    }
    
    const sunPos = window.SunCalc ? SunCalc.getPosition(now, latA, lonA) : null;
    const moonIllum = window.SunCalc ? SunCalc.getMoonIllumination(now) : null;
    const sunTimes = window.SunCalc ? SunCalc.getTimes(now, latA, lonA) : null;
    const moonTimes = window.SunCalc ? SunCalc.getMoonTimes(now, latA, lonA, true) : null;
    const solarTimes = getSolarTime(now, lonA);

    if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString('fr-FR', { timeZone: 'UTC', hour: '2-digit', minute: '2-digit', second: '2-digit' });
    if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
    
    if (sTime) {
        const timeElapsed = (now.getTime() - sTime) / 1000;
        $('time-elapsed').textContent = `${timeElapsed.toFixed(2)} s`;
        $('time-moving').textContent = `${timeMoving.toFixed(2)} s`;
        $('time-minecraft').textContent = getMinecraftTime(now); // Corrigé : utilise 'time-minecraft'
    }
    
    if ($('tst')) $('tst').textContent = solarTimes.TST; // Corrigé : ID TST
    if ($('lsm')) $('lsm').textContent = solarTimes.MST; // Corrigé : ID LSM
    if ($('sun-elevation')) $('sun-elevation').textContent = sunPos ? `${(sunPos.altitude * R2D).toFixed(2)} °` : 'N/A';
    if ($('moon-phase-display')) $('moon-phase-display').textContent = moonIllum ? `${(moonIllum.fraction * 100).toFixed(1)} %` : 'N/A';
    if ($('noon-solar')) $('noon-solar').textContent = sunTimes && sunTimes.solarNoon ? sunTimes.solarNoon.toLocaleTimeString('fr-FR') : 'N/D';
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

    // Mises à jour de l'horloge visuelle
    if (sunPos && moonIllum) {
        updateClockVisual(sunPos.altitude * R2D, sunPos.azimuth * R2D, moonIllum.phase);
    }
}

// MISE À JOUR : Logique de l'horloge visuelle (Astro)
function updateClockVisual(elevation, azimuth, moonPhase) {
    const sun = $('sun-element'), moonEl = $('moon-element');
    const moonIcon = $('moon-icon-display'); // Assurez-vous que cet ID existe dans l'HTML
    const clockStatus = $('clock-status');

    if (!sun || !moonEl) return;

    // Azimut 0°=N, 90°=E, 180°=S, 270°=W
    // Rotation CSS 0°=Haut, 90°=Droite, 180°=Bas
    // On veut 180° (Sud, Soleil au plus haut) en HAUT (0°)
    const sunRotation = azimuth - 180;
    const moonRotation = azimuth; // Lune opposée (azimut + 180 - 180)
    
    sun.style.transform = `rotate(${sunRotation.toFixed(1)}deg)`; 
    moonEl.style.transform = `rotate(${moonRotation.toFixed(1)}deg)`; 

    sun.style.opacity = elevation > -5 ? 1 : 0; // Soleil visible au-dessus de -5°
    moonEl.style.opacity = elevation <= 5 ? 1 : 0; // Lune visible la nuit
    
    // Mise à jour de l'icône de la Lune
    let moonIconChar = '🌑';
    if (moonPhase > 0.97) moonIconChar = '🌕';
    else if (moonPhase > 0.77) moonIconChar = '🌘';
    else if (moonPhase > 0.73) moonIconChar = '🌗';
    else if (moonPhase > 0.52) moonIconChar = '🌖';
    else if (moonPhase > 0.48) moonIconChar = '🌕';
    else if (moonPhase > 0.27) moonIconChar = '🌔';
    else if (moonPhase > 0.23) moonIconChar = '🌓';
    else if (moonPhase > 0.03) moonIconChar = '🌒';
    
    if (moonIcon) moonIcon.textContent = moonIconChar;

    if (clockStatus) clockStatus.textContent = elevation > 0 ? `Jour (Élévation: ${elevation.toFixed(1)}°)` : `Nuit`;

    // Mettre à jour le fond du ciel
    let skyClass = 'sky-day';
    if (elevation < 0) {
        skyClass = 'sky-night-light';
        if (elevation < -18) skyClass = 'sky-night';
    } else if (elevation < 5) {
        skyClass = 'sky-sunset'; 
    }
    // Appliquer uniquement si la classe a changé
    if (!document.body.classList.contains(skyClass)) {
        document.body.className = ''; // Nettoyer
        document.body.classList.add(skyClass);
        if (document.body.classList.contains('dark-mode')) document.body.classList.add('dark-mode');
    }
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
    if (sTime === null) sTime = Date.now();
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
    ['speed-stable', 'distance-total-km', 'local-time'].forEach(id => {
        if ($(id)) $(id).textContent = 'ARRÊT D’URGENCE';
    });
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
    if (err.code === 1) { // Permission refusée
        alert("Permission GPS refusée. L'application ne peut pas fonctionner.");
    }
    // Ne pas déclencher l'arrêt d'urgence pour une erreur GPS standard (ex: timeout)
    // emergencyStop(); 
}

async function fetchWeather(latA, lonA) {
    if (!latA || !lonA) return; 

    try {
        const response = await fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${latA}&lon=${lonA}`);
        const data = await response.json();
        
        if (data.cod && data.cod != 200) {
            throw new Error(data.message || 'Clé API invalide ou requête échouée');
        }

        if (data.main) {
            lastP_hPa = data.main.pressure; 
            lastT_K = data.main.temp_min; // Utilise la temp_min (en K) fournie par le proxy
            lastH_perc = data.main.humidity / 100; 
        }

        if ($('temp-air')) $('temp-air').textContent = lastT_K ? `${(lastT_K - 273.15).toFixed(1)} °C` : 'N/A';
        if ($('pressure')) $('pressure').textContent = lastP_hPa ? `${lastP_hPa.toFixed(1)} hPa` : 'N/A';
        if ($('humidity')) $('humidity').textContent = lastH_perc ? `${(lastH_perc * 100).toFixed(0)} %` : 'N/A';

    } catch (error) {
        console.error("Échec de la récupération des données météo:", error);
        if ($('temp-air')) $('temp-air').textContent = 'API CLÉ MANQUANTE';
    }
}


// ===========================================
// FONCTION PRINCIPALE DE MISE À JOUR GPS (updateDisp)
// ===========================================

function updateDisp(pos) {
    if (emergencyStopActive) return;

    lat = pos.coords.latitude; lon = pos.coords.longitude;
    const alt = pos.coords.altitude, acc = pos.coords.accuracy;
    const spd = pos.coords.speed;
    const cTimePos = pos.timestamp;
    const gps_heading = pos.coords.heading; // Cap GPS

    const now = getCDate(); 
    if (now === null) { return; } // Attend la synchro NTP

    if (sTime === null) { sTime = now.getTime(); }
    
    // CORRECTION : Ne pas s'arrêter si acc > MAX_ACC, juste pénaliser R (géré dans getKalmanR)
    if (acc > MAX_ACC) { 
        if ($('gps-precision')) $('gps-precision').textContent = `❌ ${acc.toFixed(0)} m (Signal Perdu/Estimation)`; 
    } else {
        if ($('gps-precision')) $('gps-precision').textContent = `${acc.toFixed(2)} m`; 
    }
    
    let spdH = spd ?? 0;
    let spdV = 0; 
    const dt = lPos ? (cTimePos - lPos.timestamp) / 1000 : MIN_DT;

    if (dt < MIN_DT || dt > 10) { // Si le dt est invalide (ex: 0 ou trop long)
        lPos = pos; // Réi
    // ... (Code précédent de la fonction DOMContentLoaded)

    // Contrôles d'arrêt et de réinitialisation
    if ($('reset-dist-btn')) {
        $('reset-dist-btn').addEventListener('click', () => { 
            if (emergencyStopActive) return; 
            distM = 0; timeMoving = 0; 
            // Mise à jour immédiate du DOM pour le reset
            if ($('distance-total-km')) $('distance-total-km').textContent = `0.000 km | 0.00 m`;
            if ($('speed-avg-moving')) $('speed-avg-moving').textContent = `0.00000 km/h`;
            if ($('time-moving')) $('time-moving').textContent = `0.00 s`;
        });
    }
    if ($('reset-max-btn')) {
        $('reset-max-btn').addEventListener('click', () => { 
            if (emergencyStopActive) return; 
            maxSpd = 0; 
            if ($('speed-max')) $('speed-max').textContent = `0.00000 km/h`;
        });
    }
    if ($('reset-all-btn')) {
        $('reset-all-btn').addEventListener('click', () => { 
            if (emergencyStopActive) return; 
            if (confirm("Êtes-vous sûr de vouloir tout réinitialiser?")) { 
                distM = 0; maxSpd = 0; timeMoving = 0; 
                kSpd = 0; kUncert = 1000; 
                kAlt = null; kAltUncert = 10;
                // Réinitialiser l'affichage
                if ($('distance-total-km')) $('distance-total-km').textContent = `0.000 km | 0.00 m`;
                if ($('speed-avg-moving')) $('speed-avg-moving').textContent = `0.00000 km/h`;
                if ($('time-moving')) $('time-moving').textContent = `0.00 s`;
                if ($('speed-max')) $('speed-max').textContent = `0.00000 km/h`;
            } 
        });
    }
    
    // Contrôle du mode Nuit/Jour (Esthétique)
    if ($('toggle-mode-btn')) {
        $('toggle-mode-btn').addEventListener('click', () => {
            document.body.classList.toggle('dark-mode');
        });
    }
    
    // --- DÉMARRAGE DU SYSTÈME ---
    
    // 1. Initialise la synchro NTP
    syncH(); 
    
    // 2. Initialise la gravité (basée sur la sélection par défaut)
    updateCelestialBody(currentCelestialBody); 
    
    // 3. Démarre la surveillance GPS (qui lancera updateDisp)
    startGPS(); 

    // 4. Démarrage de la boucle d'affichage lent (Astro/Météo)
    if (domID === null) {
        domID = setInterval(() => {
            // Utilise les dernières coordonnées connues (ou un fallback) pour l'astro
            const fallbackLat = lat || 43.296; // Marseille fallback
            const fallbackLon = lon || 5.370;
            updateAstro(fallbackLat, fallbackLon);
            
            // Mettre à jour la météo (si la position est connue)
            if (lat && lon) {
                fetchWeather(lat, lon);
            }
        }, DOM_SLOW_UPDATE_MS); 
    }
});
