// =================================================================
// FICHIER JS PARTIE 1/2 : gnss-dashboard-part1.js (Final V11.0)
// CORE, KALMAN, CONSTANTES
// =================================================================

// --- CLÉS D'API & PROXY VERCEL ---
// Note: Utilisation d'un proxy pour éviter les problèmes CORS/clés API dans un environnement local.
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app"; 
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc"; 

// --- CONSTANTES GLOBALES ET INITIALISATION ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI; // Degrés/Radians
const C_L = 299792458; // Vitesse de la lumière (m/s)
const R_E = 6371000;   // Rayon terrestre moyen (m)
const KMH_MS = 3.6;    // Facteur m/s -> km/h
const C_S = 343;       // Vitesse du son standard (m/s)
const G_ACC = 9.80665; // Accélération de la gravité (m/s²)
const dayMs = 1000 * 60 * 60 * 24; // Millisecondes par jour
const J1970 = 2440588, J2000 = 2451545; // Jours Juliens de référence
const MIN_DT = 0.01; // Delta T minimum pour éviter la division par zéro

// PARAMÈTRES AVANCÉS DU FILTRE DE KALMAN
const Q_NOISE = 0.01;       // Bruit du processus EKF Vitesse
const R_MIN = 0.05, R_MAX = 50.0; // Bornes du bruit de mesure EKF Vitesse
const ALT_TH = -50; // Seuil d'altitude pour le statut 'Sous-sol'
const NETHER_RATIO = 8; // Ratio de distance du mode Nether
const MSL_OFFSET_M = 40; // Correction Géoïde par défaut (estimation)
const DOM_SLOW_UPDATE_MS = 1000; 

const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0 },
    'METAL': { R_MULT: 2.5 },      
    'FOREST': { R_MULT: 1.5 },     
    'CONCRETE': { R_MULT: 3.0 },   
};

// Options GPS
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};

// --- VARIABLES D'ÉTAT ---
let wID = null, domID = null, lPos = null; // IDs et dernière position GPS brute
let lat = null, lon = null, sTime = null; // Position actuelle et temps de session
let distM = 0, distMStartOffset = 0, maxSpd = 0; // Distance et max speed
let kSpd = 0, kUncert = 1000; // État et incertitude EKF Vitesse
let timeMoving = 0, timeElapsed = 0; // Temps de mouvement et total
let lServH = null, lLocH = null; // Heure serveur et locale
let lastFSpeed = 0; // Dernière vitesse filtrée
let mslOffset = MSL_OFFSET_M; // Correction Géoïde estimée
let lastP_hPa = null, lastT_K = null, lastH_perc = null; // Données Météo
let emergencyStopActive = false;
let netherMode = false;
let selectedEnvironment = 'NORMAL'; 
let currentGPSMode = 'HIGH_FREQ'; 

const $ = id => document.getElementById(id);


// ===========================================
// FONCTIONS GÉO & UTILS
// ===========================================

/** ⌚ Obtient la date/heure corrigée (NTP si synchro ok) */
function getCDate() { 
    return sTime ? new Date(Date.now() - (lLocH.getTime() - lServH.getTime())) : new Date();
}

/** Calcule la distance 3D Haversine entre deux positions. */
function dist(pos1, pos2) {
    const lat1 = pos1.coords.latitude * D2R;
    const lat2 = pos2.coords.latitude * D2R;
    const dLat = (pos2.coords.latitude - pos1.coords.latitude) * D2R;
    const dLon = (pos2.coords.longitude - pos1.coords.longitude) * D2R;

    const a = Math.sin(dLat / 2) ** 2 +
              Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    
    // Distance 2D
    const distance2D = R_E * c;
    
    // Correction 3D (altitude)
    const dAlt = (pos2.coords.altitude || 0) - (pos1.coords.altitude || 0);
    return Math.sqrt(distance2D ** 2 + dAlt ** 2);
}

/** 🌐 Corrige l'altitude WGS84 (GPS) en Altitude MSL (Niveau de la mer) */
function correctAltitudeToMSL(wgs84Alt, geoidHeight) {
    if (wgs84Alt === null) return null;
    return wgs84Alt - (geoidHeight ?? mslOffset);
}

/** Calcule le facteur R (Précision 3D Dynamique) du filtre de Kalman. */
function getKalmanR(acc, alt, P_hPa) {
    // R base est basé sur l'incertitude reportée par le GPS (acc^2)
    let R = acc ** 2; 
    R = Math.max(R_MIN, Math.min(R_MAX, R));
    
    // 1. Facteur environnemental (manuel)
    const envFactor = ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT;
    
    // 2. CORRECTION MÉTÉO (Si la pression est loin du standard, augmente l'incertitude)
    if (P_hPa !== null) {
        const pressureDeviation = Math.abs(1013.25 - P_hPa);
        const pressureFactor = 1.0 + pressureDeviation / 1013.25 * 0.1; // Max 10% d'augmentation de R
        R *= Math.max(1.0, pressureFactor);
    }
    
    R *= envFactor;

    // 3. Facteur sous-sol
    if (alt < ALT_TH) { R *= 2.0; } 

    return Math.max(R_MIN, Math.min(R_MAX, R));
}

// ===========================================
// FILTRE DE KALMAN (EKF) VITESSE
// ===========================================

/** Applique le filtre de Kalman à la vitesse 3D. */
function kFilter(z, dt, acc, alt, P_hPa) {
    if (isNaN(z) || z === null || dt < MIN_DT) {
        // Si données invalides, on confie au modèle (prédiction seule)
        kUncert += Q_NOISE * dt;
        return kSpd;
    }
    
    // 1. Prédiction
    const kSpdPred = kSpd + lastFSpeed * dt; // Simple: v(t) = v(t-1) + a*dt (a=0 ici)
    const kUncertPred = kUncert + Q_NOISE * dt;

    // 2. Mise à jour (Correction)
    const R_dyn = getKalmanR(acc, alt, P_hPa);
    const K = kUncertPred / (kUncertPred + R_dyn);
    
    kSpd = kSpdPred + K * (z - kSpdPred);
    kUncert = (1 - K) * kUncertPred;

    lastFSpeed = (kSpd - kSpdPred) / dt; // Vitesse de changement (accélération virtuelle)
    return kSpd;
}

// ===========================================
// FONCTIONS DE CONTRÔLE GPS & INITIALISATION
// (startGPS, stopGPS, resetDisp, syncNTPTime, initGeolocation)
// ... (Ces fonctions sont habituellement dans la partie 2 pour la logique DOM/Event)
// ===========================================
// =================================================================
// FICHIER JS PARTIE 2/2 : gnss-dashboard-part2.js (Final V11.1)
// ASTRO (EOT Précis), MÉTÉO, LOGIQUE DOM & INITIALISATION
// =================================================================

// ===========================================
// FONCTIONS ASTRO
// ===========================================

/** Convertit la date en jours depuis J2000. */
function toDays(date) { return (date.valueOf() / dayMs - 0.5 + J1970) - J2000; }

/** Calcule l'anomalie solaire moyenne (en radians). */
function solarMeanAnomaly(d) { return D2R * (356.0470 + 0.98560028 * d); }

/**
 * ☀️ CORRIGÉ (V11.1) : Calcule TST, LSM, EOT Total et ses Composantes (Eccentricité et Obliquité).
 * Utilise une éphéméride de haute précision (Meeus's method).
 */
function getSolarTime(date, lon) {
    if (date === null || lon === null) return { TST: 'N/A', MST: 'N/A', EOT: 'N/D', ECL_LONG: 'N/D' };

    const d = toDays(date);
    const M = solarMeanAnomaly(d); 
    const P = D2R * 102.9377;      

    // 1. Composante Eccentricité (Équation du Centre - C)
    const C_rad = D2R * (1.9148 * Math.sin(M) + 0.0200 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M));
    const eot_ecc_min = C_rad * R2D * 4; 
    
    // 2. Longitude Ecliptique Vraie (L)
    let L = M + C_rad + P + Math.PI;
    L = L % (2 * Math.PI); 
    if (L < 0) L += 2 * Math.PI;

    // 3. Longitude Ecliptique Moyenne (L_mean)
    let L_mean = M + P + Math.PI;
    L_mean = L_mean % (2 * Math.PI); 
    if (L_mean < 0) L_mean += 2 * Math.PI;

    // 4. Obliquité et Ascension Droite Vraie (alpha)
    const epsilon = D2R * (23.4393 - 0.000000356 * d);
    let alpha = Math.atan2(Math.cos(epsilon) * Math.sin(L), Math.cos(L));
    if (alpha < 0) alpha += 2 * Math.PI;

    // 5. Équation du Temps (EOT) (Total)
    let eot_rad = L_mean - alpha;
    eot_rad = eot_rad % (2 * Math.PI);
    if (eot_rad > Math.PI) eot_rad -= 2 * Math.PI;
    if (eot_rad < -Math.PI) eot_rad += 2 * Math.PI;
    
    const eot_min = eot_rad * R2D * 4; 

    // 6. Composante Obliquité (par soustraction)
    const eot_obliq_min = eot_min - eot_ecc_min;

    // 7. Temps Solaire Moyen Local (LSM/MST) et Temps Solaire Vrai (TST)
    const msSinceMidnightUTC = (date.getUTCHours() * 3600 + date.getUTCMinutes() * 60 + date.getUTCSeconds()) * 1000 + date.getUTCMilliseconds();
    const mst_offset_ms = (lon / 15) * 3600000; 
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
        EOT: eot_min.toFixed(4), 
        ECL_LONG: (L * R2D).toFixed(4),
        EOT_ECC: eot_ecc_min.toFixed(4),
        EOT_OBLIQ: eot_obliq_min.toFixed(4)
    };
}

// ===========================================
// FONCTIONS MÉTÉO & SYNCHRONISATION
// ===========================================

let lastWeatherFetch = 0;

/** Récupère les données météo via l'API (pour correction Kalman/Affichage) */
async function fetchWeather(latA, lonA) {
    if (Date.now() - lastWeatherFetch < 60000) return; // Limite à 1 appel/minute
    if (latA === null || lonA === null) return;
    
    lastWeatherFetch = Date.now();
    
    try {
        const response = await fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${latA}&lon=${lonA}`);
        const data = await response.json();

        if (data.error || !data.main) throw new Error(data.error || 'Données météo invalides');

        // Conversion et stockage
        lastP_hPa = data.main.pressure;
        lastT_K = data.main.temp + 273.15; // De Celsius à Kelvin
        lastH_perc = data.main.humidity / 100;

        if ($('temp-air')) $('temp-air').textContent = `${data.main.temp.toFixed(1)} °C`;
        if ($('pressure')) $('pressure').textContent = `${lastP_hPa.toFixed(0)} hPa`;
        if ($('humidity')) $('humidity').textContent = `${data.main.humidity.toFixed(0)} %`;
        
    } catch (e) {
        console.warn("Erreur de récupération météo:", e.message);
        if ($('temp-air')) $('temp-air').textContent = 'N/A (Erreur)';
    }
}

/** Synchronise l'horloge système avec un serveur NTP/UTC */
async function syncNTPTime() {
    try {
        const response = await fetch(SERVER_TIME_ENDPOINT);
        const data = await response.json();

        if (!data.utc_datetime) throw new Error('Format de date NTP invalide');

        lLocH = new Date();
        lServH = new Date(data.utc_datetime);
        sTime = lServH; // Heure du serveur comme temps de référence
        
        console.log("Synchronisation NTP réussie.");
    } catch (e) {
        console.warn("Erreur de synchronisation NTP. Utilisation de l'horloge locale.");
        lLocH = null;
        lServH = null;
        sTime = null;
    }
}


// ===========================================
// BOUCLES ET MISE À JOUR DOM
// ===========================================

let lastUpdateTS = 0;

/** Mise à jour des affichages DOM qui nécessitent une haute fréquence */
function updateDOMFast() {
    const now = getCDate();
    const dt_sec = (now.getTime() - lastUpdateTS) / 1000;
    
    if (!lastUpdateTS || dt_sec > 1) { // Sécurité si l'intervalle est manqué
        lastUpdateTS = now.getTime();
        return;
    }
    
    timeElapsed += dt_sec;

    // Mise à jour des temps
    const formatTime = (totalSeconds) => {
        const h = Math.floor(totalSeconds / 3600);
        const m = Math.floor((totalSeconds % 3600) / 60);
        const s = (totalSeconds % 60).toFixed(2);
        return `${String(h).padStart(2, '0')}h ${String(m).padStart(2, '0')}m ${String(Math.floor(s)).padStart(2, '0')}s`;
    };
    
    if (kSpd >= getDynamicMinSpd()) timeMoving += dt_sec;

    if ($('time-elapsed')) $('time-elapsed').textContent = formatTime(timeElapsed);
    if ($('time-moving')) $('time-moving').textContent = formatTime(timeMoving);
    
    // Vitesse ultra-précise (µm/s)
    const microMS = (kSpd * 1e6).toFixed(0);
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${kSpd.toFixed(2)} m/s | ${microMS} µm/s`;

    lastUpdateTS = now.getTime();
}

/** Mise à jour des données astronomiques */
function updateAstro(latA, lonA) {
    const now = getCDate(); 
    if (now === null || latA === null || lonA === null) {
        if ($('date-display')) $('date-display').textContent = 'N/A';
        return;
    }

    if ($('date-display')) $('date-display').textContent = now.toLocaleDateString();
    
    // --- 1. Calculs Astro Précis (V11.1) ---
    const solarTimes = getSolarTime(now, lonA); 
    const sunPos = window.SunCalc ? SunCalc.getPosition(now, latA, lonA) : null;
    const moonPos = window.SunCalc ? SunCalc.getMoonPosition(now, latA, lonA) : null;
    const moonIllum = window.SunCalc ? SunCalc.getMoonIllumination(now) : null;
    const sunTimes = window.SunCalc ? SunCalc.getTimes(now, latA, lonA) : null;
    
    // --- 2. Affichage Soleil (EOT Composantes) ---
    if ($('time-solar-true')) $('time-solar-true').textContent = solarTimes.TST;
    if ($('culmination-lsm')) $('culmination-lsm').textContent = solarTimes.MST;
    if ($('eot-min')) $('eot-min').textContent = solarTimes.EOT + ' min'; 
    if ($('ecliptic-long')) $('ecliptic-long').textContent = solarTimes.ECL_LONG + ' °';

    // NOUVEAUTÉ V11.1 : Affichage des composantes
    if ($('eot-ecc-comp')) $('eot-ecc-comp').textContent = solarTimes.EOT_ECC + ' min';
    if ($('eot-obliq-comp')) $('eot-obliq-comp').textContent = solarTimes.EOT_OBLIQ + ' min';

    if (sunPos) {
        if ($('sun-elevation')) $('sun-elevation').textContent = (sunPos.altitude * R2D).toFixed(2) + ' °';
        if ($('sun-azimuth')) $('sun-azimuth').textContent = (sunPos.azimuth * R2D).toFixed(2) + ' °';
    }
    
    // Durée du Jour
    if (sunTimes && sunTimes.sunrise && sunTimes.sunset) {
        const durationMs = sunTimes.sunset.getTime() - sunTimes.sunrise.getTime();
        const h = Math.floor(durationMs / 3600000);
        const m = Math.floor((durationMs % 3600000) / 60000);
        if ($('day-duration')) $('day-duration').textContent = `${h}h ${String(m).padStart(2, '0')}m`;
    }

    // --- 3. Affichage Lune ---
    if (moonPos && moonIllum) {
        if ($('lunar-phase-perc')) $('lunar-phase-perc').textContent = (moonIllum.fraction * 100).toFixed(1) + ' %';
        if ($('moon-elevation')) $('moon-elevation').textContent = (moonPos.altitude * R2D).toFixed(2) + ' °';
        if ($('moon-azimuth')) $('moon-azimuth').textContent = (moonPos.azimuth * R2D).toFixed(2) + ' °';
    }
    
    if (window.SunCalc && latA !== null) {
        const moonRise = SunCalc.getMoonTimes(now, latA, lonA).rise;
        const moonSet = SunCalc.getMoonTimes(now, latA, lonA).set;
        const riseStr = moonRise ? moonRise.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit' }) : '--:--';
        const setStr = moonSet ? moonSet.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit' }) : '--:--';
        if ($('moon-times')) $('moon-times').textContent = `↑ ${riseStr} / ↓ ${setStr}`;
    }
}

/** Mise à jour des affichages DOM et lancement des fonctions externes */
function updateDisp(pos) {
    if (emergencyStopActive) return;

    const now = getCDate();
    let dt = 0;
    let speedRawMS = 0;
    
    lat = pos.coords.latitude;
    lon = pos.coords.longitude;
    
    const alt_gps_wgs = pos.coords.altitude || 0;
    const geoid_correction = mslOffset; // Estimation statique pour l'instant
    const alt_msl = correctAltitudeToMSL(alt_gps_wgs, geoid_correction);
    
    // 1. Calcul des vitesses et distances
    if (lPos) {
        dt = (pos.timestamp - lPos.timestamp) / 1000;
        const dist_step = dist(lPos, pos);
        
        if (dt > MIN_DT) {
            speedRawMS = dist_step / dt;
            distM += dist_step;
        }
    }
    lPos = pos;
    
    // 2. Filtre EKF et Accélération
    const accuracy = pos.coords.accuracy || 5; 
    const kSpd_prev = kSpd;
    kSpd = kFilter(speedRawMS, dt, accuracy, alt_msl, lastP_hPa);
    
    const accel_long = (kSpd - kSpd_prev) / dt || 0;
    
    // 3. Distance Nether
    const dist_total_final = netherMode ? distM / NETHER_RATIO : distM;
    
    // 4. Mise à jour Max/Avg
    maxSpd = Math.max(maxSpd, kSpd);
    
    // 5. Mise à jour de l'affichage
    if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString('fr-FR');
    if ($('speed-stable')) $('speed-stable').textContent = (kSpd * KMH_MS).toFixed(5);
    if ($('speed-3d-inst')) $('speed-3d-inst').textContent = (speedRawMS * KMH_MS).toFixed(5);
    if ($('speed-max')) $('speed-max').textContent = (maxSpd * KMH_MS).toFixed(5);
    if ($('speed-raw-ms')) $('speed-raw-ms').textContent = speedRawMS.toFixed(2) + ' m/s';
    
    if ($('latitude')) $('latitude').textContent = lat.toFixed(6);
    if ($('longitude')) $('longitude').textContent = lon.toFixed(6);
    if ($('altitude-gps')) $('altitude-gps').textContent = alt_msl.toFixed(2) + ' m (MSL)';
    if ($('geoid-correction')) $('geoid-correction').textContent = geoid_correction.toFixed(2) + ' m';
    if ($('gps-precision')) $('gps-precision').textContent = accuracy.toFixed(2) + ' m';
    
    if ($('accel-long')) $('accel-long').textContent = accel_long.toFixed(3) + ' m/s²';
    if ($('force-g-long')) $('force-g-long').textContent = (accel_long / G_ACC).toFixed(3) + ' G';

    // Affichage des ratios de vitesse
    if ($('perc-speed-sound')) $('perc-speed-sound').textContent = (kSpd / C_S * 100).toFixed(2) + ' %';
    if ($('perc-speed-c')) $('perc-speed-c').textContent = (kSpd / C_L * 100).toExponential(2) + '%';
    
    if ($('distance-total-km')) $('distance-total-km').textContent = `${(dist_total_final / 1000).toFixed(3)} km | ${dist_total_final.toFixed(2)} m`;
    if ($('distance-cosmic')) $('distance-cosmic').textContent = (dist_total_final / C_L).toExponential(2) + ' s lumière';
    
    if ($('underground-status')) $('underground-status').textContent = alt_msl < ALT_TH ? 'Oui' : 'Non';
    
    const R_dyn = getKalmanR(accuracy, alt_msl, lastP_hPa);
    if ($('speed-error-perc')) $('speed-error-perc').textContent = `${(kUncert / R_dyn * 100).toFixed(1)} % (${R_dyn.toFixed(2)})`;
    
    // Lancement des fonctions lentes
    updateAstro(lat, lon);
    fetchWeather(lat, lon);
}

// ... (fonctions errorCallback, startGPS, stopGPS, resetDisp, initGeolocation, etc.) ...

function getDynamicMinSpd() {
    return 0.1; // Vitesse minimale en m/s pour être considéré en mouvement
}

function handleGeolocationError(error) {
    console.error("Erreur GPS:", error);
    // Affichage des messages d'erreur dans une zone dédiée
    // ...
}

function startGPS() {
    if ('geolocation' in navigator) {
        if (wID !== null) stopGPS(false);
        
        // Choisir les options en fonction du mode
        const options = GPS_OPTS[currentGPSMode];

        wID = navigator.geolocation.watchPosition(updateDisp, handleGeolocationError, options);
        domID = setInterval(updateDOMFast, 17); // 60 FPS
        
        syncNTPTime();
        
        if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = '⏸️ PAUSE GPS';
    } else {
        alert("La géolocalisation n'est pas supportée par ce navigateur.");
    }
}

function stopGPS(shouldReset = true) {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
    }
    if (domID !== null) {
        clearInterval(domID);
        domID = null;
    }

    if (shouldReset) {
        resetDisp(true);
    }
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = '▶️ MARCHE GPS';
}

function resetDisp(fullReset = false) {
    // Réinitialisation des compteurs
    if (fullReset) {
        distM = 0; 
        maxSpd = 0;
        timeElapsed = 0;
        timeMoving = 0;
    }
    // Réinitialisation EKF
    kSpd = 0; 
    kUncert = 1000;
    lastFSpeed = 0;
    
    // Mise à jour de l'affichage à 0
    if ($('speed-stable')) $('speed-stable').textContent = "0.00000";
    if ($('speed-max')) $('speed-max').textContent = "0.00000";
    // ... (autres réinitialisations DOM)
}

// ===========================================
// INITIALISATION ET ÉVÉNEMENTS
// ===========================================

function initGeolocation() {
    if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', () => wID === null ? startGPS() : stopGPS(false));
    
    if ($('freq-select')) $('freq-select').addEventListener('change', (e) => {
        currentGPSMode = e.target.value;
        if (wID !== null) startGPS(); // Redémarre avec les nouvelles options
    });
    
    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', () => {
        emergencyStopActive = !emergencyStopActive;
        const btn = $('emergency-stop-btn');
        btn.textContent = emergencyStopActive ? '🛑 Arrêt d\'urgence: ACTIF 🔴' : '🛑 Arrêt d\'urgence: INACTIF 🟢';
        if (emergencyStopActive) stopGPS(false);
    });

    // Événements de réinitialisation
    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => { distM = 0; distMStartOffset = 0; maxSpd = 0; });
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', () => { maxSpd = 0; });
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { if (confirm("Tout réinitialiser?")) stopGPS(true); });
    
    if ($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', () => {
        netherMode = !netherMode;
        $('mode-nether').textContent = netherMode ? 'ACTIVÉ (1:8) 🔥' : 'DÉSACTIVÉ (1:1)';
    });

    // Initialisation du sélecteur d'environnement (si implémenté via JS)
    // Ici, on utilise la valeur par défaut 'NORMAL' définie dans part1.js
    
    syncNTPTime(); // Synchronisation initiale
}

document.addEventListener('DOMContentLoaded', initGeolocation);
