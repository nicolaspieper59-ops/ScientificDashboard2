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

// Constantes Unités Cosmologiques
const AU_M = 149597870700; // Unité Astronomique (m)
const DAY_SEC = 86400; // Secondes par jour
const C_L_DAY = C_L * DAY_SEC; // Distance lumière/jour (m)
const C_L_MONTH = C_L_DAY * 30.4375; // Distance lumière/mois (m, moyenne)
const C_L_YEAR = C_L_DAY * 365.25; // Distance lumière/année (m)

// Constantes Temps / Astro
const dayMs = 86400000;
const J1970 = 2440588; 
const J2000 = 2451545; 
const DOM_SLOW_UPDATE_MS = 1000;
const WEATHER_UPDATE_MS = 30000; 
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc"; 

// Constantes GPS
const MIN_DT = 0.05; 
const MIN_SPD = 0.005; // Seuil de vitesse minimale (5 mm/s)
const MAX_ACC = 20; 
const ALT_TH = -50; // Seuil d'altitude pour le mode souterrain (m)
const ACCEL_IMMOBILITY_THRESHOLD = 0.05; // Seuil d'accélération IMU pour détecter l'immobilité (m/s²)

const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 5000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 30000, timeout: 60000 }
};

// Constantes Kalman (Vitesse 3D)
const Q_NOISE = 0.001; 
let kSpd = 0; // État interne de la vitesse EKF
let kUncert = 1000; 
const ENVIRONMENT_FACTORS = {
    NORMAL: 1.0, FOREST: 1.5, CONCRETE: 3.0, METAL: 2.5
};

// Constantes Kalman (Altitude)
const Q_ALT_NOISE = 0.01; 
let kAlt = 0; // État interne de l'altitude EKF
let kAltUncert = 1000; 

// Constantes IMU (Accéléromètre)
const ACCEL_FILTER_ALPHA = 0.8; 
const ACCEL_MOVEMENT_THRESHOLD = 0.5; 
let kAccel = { x: 0, y: 0, z: 0 };
let G_STATIC_REF = { x: 0, y: 0, z: 0 };
let latestVerticalAccelIMU = 0; 
let latestLinearAccelMagnitude = 0; 
let latestAccelLongIMU = 0; 
let latestAccelLatIMU = 0; 
let maxGForce = 0; 

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
let G_ACC_LOCAL = G_ACC; 

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
    if (alt !== null && alt < 0) {
        noiseMultiplier += Math.abs(alt / 100); 
    }
    
    if (linearAccelMag > 0.5) { 
        noiseMultiplier += Math.pow(linearAccelMag, 1.5) * 0.5; 
    }
    
    return R_raw * noiseMultiplier;
}

/** Filtre de Kalman 1D pour l'Altitude (Utilise u_accel IMU).
 * Fournit l'altitude en souterrain via Dead Reckoning si z est null.
 */
function kFilterAltitude(z, acc, dt, u_accel = 0) { 
    // 1. Prediction (toujours exécutée - utilise l'IMU pour estimer le mouvement)
    const predAlt = kAlt + (0.5 * u_accel * dt * dt); 
    let predAltUncert = kAltUncert + Q_ALT_NOISE * dt;

    // 2. Update (correction par mesure GPS si disponible)
    if (z !== null) { 
        const R_alt = acc * acc * 2.0; 
        const K = predAltUncert / (predAltUncert + R_alt);
        kAlt = predAlt + K * (z - predAlt);
        kAltUncert = (1 - K) * predAltUncert;
    } else {
        // Si aucune mesure GPS, la prédiction devient le nouvel état
        kAlt = predAlt;
        kAltUncert = predAltUncert;
    }

    return kAlt;
}

/** Calcule le point de rosée (utilitaire pour la météo) */
function calculateDewPoint(tempC, humidity) {
    const a = 17.27;
    const b = 237.7;
    const alpha = (a * tempC) / (b + tempC) + Math.log(humidity / 100);
    return (b * alpha) / (a - alpha);
}

/** Calcule l'accélération de la gravité locale (IGF 1980) en fonction de la latitude (m/s²). */
function calculateGravity(latitude) {
    const latRad = latitude * D2R;
    const sinLat = Math.sin(latRad);
    const sin2Lat = Math.sin(2 * latRad);
    const g0 = 9.780327 * (1 + 0.0053024 * sinLat * sinLat - 0.0000058 * sin2Lat * sin2Lat);
    return g0;
        }
// ===========================================
// FONCTIONS UTILITAIRES ASTRO & TEMPS
// ===========================================

/** Obtient l'heure courante synchronisée (si syncH a réussi) */
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
        console.warn("Échec de la synchronisation. Utilisation de l'horloge locale.", error);
        lServH = Date.now(); 
        lLocH = performance.now();
        if ($('local-time')) $('local-time').textContent = 'N/A (SYNCHRO ÉCHOUÉE)';
    }
}

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

/** Helper pour déterminer le nom de la phase lunaire. */
function getMoonPhaseName(phaseDeg) {
    if (phaseDeg < 22.5) return 'Nouvelle Lune';
    if (phaseDeg < 67.5) return 'Croissant Ascendant';
    if (phaseDeg < 112.5) return 'Premier Quartier';
    if (phaseDeg < 157.5) return 'Gibbeuse Ascendante';
    if (phaseDeg < 202.5) return 'Pleine Lune';
    if (phaseDeg < 247.5) return 'Gibbeuse Descendante';
    if (phaseDeg < 292.5) return 'Dernier Quartier';
    if (phaseDeg < 337.5) return 'Vieux Croissant';
    return 'Nouvelle Lune';
}

/** Met à jour les valeurs Astro, Lune et Couleur du Ciel. */
function updateAstro(latA, lonA) {
    const now = getCDate(); 
    if (now === null) return;
    
    // Calculs Solaires et Lunaires (SunCalc)
    const sunPos = window.SunCalc ? SunCalc.getPosition(now, latA, lonA) : null;
    const solarTimes = getSolarTime(now, lonA);
    const moonIllumination = window.SunCalc ? SunCalc.getMoonIllumination(now) : null;
    
    const moonPhaseDeg = moonIllumination ? moonIllumination.phase * 360 : 0;
    const moonPhaseName = getMoonPhaseName(moonPhaseDeg);

    // ... (Mise à jour du DOM pour Astro) ...
    // Note: Les mises à jour du DOM sont omises ici pour la concision mais sont incluses dans le fichier JS réel.
}

// ===========================================
// FONCTIONS API MÉTÉO
// ===========================================

/** Récupère et met à jour les données météo via OpenWeatherMap. */
async function updateWeather(latA, lonA) {
    if (!OWM_API_KEY || OWM_API_KEY === "VOTRE_CLE_API_OPENWEATHERMAP") {
        if ($('temp-air')) $('temp-air').textContent = 'API CLÉ MANQUANTE';
        return;
    }
    
    // ... (Logique de récupération et de mise à jour du DOM pour la météo) ...
    // Note: Le corps de cette fonction est omis pour la concision mais est inclus dans le fichier JS réel.
}


// ===========================================
// FONCTIONS CAPTEURS INERTIELS (IMU)
// ===========================================

/** Gère les données de l'accéléromètre via DeviceMotionEvent. */
function handleDeviceMotion(event) {
    if (emergencyStopActive) return;
    const acc = event.accelerationIncludingGravity;
    if (acc.x === null) return; 

    // Filtrage pour la stabilité (Complémentary Filter)
    kAccel.x = ACCEL_FILTER_ALPHA * kAccel.x + (1 - ACCEL_FILTER_ALPHA) * acc.x;
    kAccel.y = ACCEL_FILTER_ALPHA * kAccel.y + (1 - ACCEL_FILTER_ALPHA) * acc.y;
    kAccel.z = ACCEL_FILTER_ALPHA * kAccel.z + (1 - ACCEL_FILTER_ALPHA) * acc.z; 

    const kAccel_mag = Math.sqrt(kAccel.x ** 2 + kAccel.y ** 2 + kAccel.z ** 2);
    
    // Détermination de la gravité statique (pour extraire l'accélération linéaire)
    if (Math.abs(kAccel_mag - G_ACC) < ACCEL_MOVEMENT_THRESHOLD) { 
        G_STATIC_REF.x = kAccel.x;
        G_STATIC_REF.y = kAccel.y;
        G_STATIC_REF.z = kAccel.z;
    } 

    // Accélération linéaire (Mouvement pur)
    const linear_x = kAccel.x - G_STATIC_REF.x;
    const linear_y = kAccel.y - G_STATIC_REF.y;
    const linear_z = kAccel.z - G_STATIC_REF.z;

    // Mise à jour des globales IMU
    latestVerticalAccelIMU = linear_z; 
    latestAccelLongIMU = linear_x; 
    latestAccelLatIMU = linear_y; 
    latestLinearAccelMagnitude = Math.sqrt(linear_x ** 2 + linear_y ** 2 + linear_z ** 2); 
    
    const totalGForce = latestLinearAccelMagnitude / G_ACC_LOCAL; 
    if (totalGForce > maxGForce) {
        maxGForce = totalGForce;
    }
    
    // ... (Mise à jour du DOM pour l'IMU) ...
}
// ===========================================
// FONCTIONS CARTE ET CONTRÔLE GPS
// ===========================================

/** Initialise la carte Leaflet. */
function initMap(latA, lonA) {
    if (map) return;
    try {
        map = L.map('map').setView([latA, lonA], 15);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            maxZoom: 19,
            attribution: '© OpenStreetMap'
        }).addTo(map);

        marker = L.marker([latA, lonA]).addTo(map);
        tracePolyline = L.polyline([], { color: 'red' }).addTo(map);
    } catch (e) {
        if ($('map')) $('map').textContent = 'Erreur d\'initialisation de la carte.';
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

/** Met à jour le style de la carte pour simuler une vue "Rayons X" souterraine. */
function updateMapStyle(altitude) {
    const mapContainer = $('map');
    // Le seuil ALT_TH est fixé à -50 m
    const isUnderground = (altitude !== null && altitude < ALT_TH);

    if (mapContainer) {
        if (isUnderground) {
            // Activer l'effet X-Ray : niveaux de gris, assombrissement, teinte bleue
            mapContainer.style.filter = 'grayscale(100%) brightness(50%) hue-rotate(240deg) invert(10%)';
            
            // Simuler le zoom sur le lieu pour une vue en coupe "3D" plus rapprochée
            if (map && map.getZoom() < 17) map.setZoom(17); 
            $('underground-status').textContent = 'Oui (Mode X-Ray Actif ☢️)';
        } else {
            // Désactiver l'effet X-Ray
            mapContainer.style.filter = 'none';
            // Retourner au zoom par défaut
            if (map && map.getZoom() > 15) map.setZoom(15); 
            $('underground-status').textContent = 'Non';
        }
    }
}

function setGPSMode(mode) {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    currentGPSMode = mode;
    wID = navigator.geolocation.watchPosition(updateDisp, handleErr, GPS_OPTS[mode]);
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
}

function emergencyStop() {
    emergencyStopActive = true;
    stopGPS(false);
    if ($('emergency-stop-btn')) {
        $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: ACTIF 🔴";
        $('emergency-stop-btn').classList.remove('green');
        $('emergency-stop-btn').classList.add('red');
        $('emergency-stop-btn').classList.add('active');
    }
}

function resumeSystem() {
    emergencyStopActive = false;
    if ($('emergency-stop-btn')) {
        $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: INACTIF 🟢";
        $('emergency-stop-btn').classList.remove('red');
        $('emergency-stop-btn').classList.add('green');
        $('emergency-stop-btn').classList.remove('active');
    }
    startGPS();
}

function handleErr(err) {
    console.error(`Erreur GNSS (${err.code}): ${err.message}`);
    const btn = $('toggle-gps-btn');
    if (btn) {
        btn.innerHTML = '❌ ERREUR GPS';
        btn.classList.remove('green');
        btn.classList.add('red');
    }
}


// ===========================================
// FONCTION PRINCIPALE DE MISE À JOUR (GPS, Kalman, Physique)
// ===========================================

function updateDisp(pos) {
    if (emergencyStopActive) return;
    lat = pos.coords.latitude; lon = pos.coords.longitude;
    const alt_raw = pos.coords.altitude, acc = pos.coords.accuracy; 
    const spd_raw_gps = pos.coords.speed;
    const cTimePos = pos.timestamp; 
    const now = getCDate(); 
    
    // Initialisation
    const G_local = calculateGravity(lat);
    G_ACC_LOCAL = G_local; 
    let MASS = 70.0; // Lecture de la masse
    
    if (sTime === null) { sTime = now.getTime(); }

    let effectiveAcc = acc;
    const accOverride = parseFloat($('gps-accuracy-override').value);
    if (accOverride > 0) { effectiveAcc = accOverride; }

    let spdH = spd_raw_gps ?? 0; 
    const dt = lPos ? (cTimePos - lPos.timestamp) / 1000 : MIN_DT;

    // 1. FILTRAGE DE L'ALTITUDE (via Kalman et IMU pour le Dead Reckoning)
    let kAlt_new = kFilterAltitude(alt_raw, effectiveAcc, dt, latestVerticalAccelIMU); 
    
    // 2. VITESSE VERTICALE
    let spdV = 0; 
    let verticalSpeedUncert = 0;
    if (lPos && lPos.kAlt_old !== undefined && dt > MIN_DT && alt_raw !== null) { 
        spdV = (kAlt_new - lPos.kAlt_old) / dt; 
        const kAltUncert_old = lPos.kAltUncert_old !== undefined ? lPos.kAltUncert_old : kAltUncert;
        verticalSpeedUncert = Math.sqrt(kAltUncert ** 2 + kAltUncert_old ** 2) / dt;
        verticalSpeedUncert = Math.min(20, verticalSpeedUncert); 
    }
    
    // 3. VITESSE HORIZONTALE CALCULÉE (Haversine)
    if (lPos && dt > 0.05) { 
        const dH = dist(lPos.coords.latitude, lPos.coords.longitude, lat, lon); 
        spdH = dH / dt; 
    } 

    // 4. VITESSE 3D INSTANTANÉE
    let spd3D = Math.sqrt(spdH ** 2 + spdV ** 2);

    // 5. FILTRE DE KALMAN FINAL (Vitesse 3D Stable)
    const R_dyn = getKalmanR(effectiveAcc, alt_raw, lastP_hPa, latestLinearAccelMagnitude); 
    let fSpd = kFilter(spd3D, dt, R_dyn); 
    
    // LOGIQUE D'IMMOBILITÉ (Fusion GPS/IMU)
    let sSpdFE = fSpd < MIN_SPD ? 0 : fSpd; 
    const isImmobileByIMU = latestLinearAccelMagnitude < ACCEL_IMMOBILITY_THRESHOLD;
    
    if (isImmobileByIMU) {
        sSpdFE = 0;
        fSpd = 0; 
        kSpd = 0; 
        // Gèle l'altitude EKF lorsque l'IMU détecte l'immobilité
        if (lPos && lPos.kAlt_old !== undefined) {
            kAlt_new = lPos.kAlt_old;
            kAlt = lPos.kAlt_old; 
        }
    }

    // Calculs Physiques et de Distance
    let accel_long_ekf = (dt > 0.05) ? (sSpdFE - lastFSpeed) / dt : 0;
    lastFSpeed = sSpdFE;
    distM += sSpdFE * dt * (netherMode ? NETHER_RATIO : 1); 
    
    if (sSpdFE > MIN_SPD) { timeMoving += dt; }
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    
    // ... (Calculs d'énergie, force de Coriolis, etc.) ...
    
    // --- MISE À JOUR DU DOM ---
    // ... (Mise à jour des nombreuses valeurs dans les colonnes 1, 2 et 3) ...

    // --- LOGIQUE X-RAY : MISE À JOUR DU STYLE DE LA CARTE ---
    updateMapStyle(kAlt_new); 
    
    // CONDITION POUR LA CARTE : Ne mettez à jour la carte que si l'appareil est en mouvement.
    if (!isImmobileByIMU) {
        updateMap(lat, lon);
    }
    
    // SAUVEGARDE DES VALEURS POUR LA PROCHAINE ITÉRATION
    lPos = pos; 
    lPos.timestamp = cTimePos; 
    lPos.kAlt_old = kAlt_new; 
    lPos.kAltUncert_old = kAltUncert; 
}

// ===========================================
// ÉCOUTEURS ET INITIALISATION
// ===========================================

window.addEventListener('load', () => {
    syncH(); // Synchronisation initiale de l'heure
    setInterval(syncH, 3600000); // Resynchronisation toutes les heures
    setInterval(() => { if (wID !== null) updateAstro(lat, lon); }, DOM_SLOW_UPDATE_MS); 
    weatherID = setInterval(() => updateWeather(lat, lon), WEATHER_UPDATE_MS);
    
    if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', handleDeviceMotion, false);
    } else {
        console.warn("DeviceMotion n'est pas supporté. L'IMU sera inactive.");
    }
    
    // Initialiser avec les paramètres par défaut
    if ($('freq-select')) $('freq-select').value = currentGPSMode; 
    if ($('environment-select')) $('environment-select').value = selectedEnvironment; 
});
