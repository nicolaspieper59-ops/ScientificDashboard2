// =================================================================
// FICHIER JS PARTIE 1 : gnss-dashboard-part1.js
// CONSTANTES, ÉTAT GLOBAL, FILTRES DE KALMAN & IMU
// =================================================================

const $ = (id) => document.getElementById(id);

// --- CONSTANTES GLOBALES (Physiques, GPS, Temps) ---
const C_L = 299792458; 
const SPEED_SOUND = 343; 
const G_ACC = 9.80665; 
const KMH_MS = 3.6; 
const R_E = 6371000; 
const R2D = 180 / Math.PI;
const D2R = Math.PI / 180;
const W_EARTH = 7.2921E-5; 
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
const Q_NOISE = 0.005; // Augmentation du bruit de processus IMU (pour qu'il dérive un peu)
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
let latestAccelLatIMU = 0; // NOUVEAU: Accélération Latérale IMU
let latestLinearAccelMagnitude = 0; 
let latestAccelLongEKF = 0; 
let latestAccelLongIMU = 0; 
let maxGForce = 0; // NOUVEAU: Force G Max

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

/** NOUVEAU FILTRE DE KALMAN FUSIONNÉ (IMU pour Prédiction, GPS pour Correction)
 * @param z Mesure GPS de la vitesse (m/s)
 * @param dt Intervalle de temps (s)
 * @param R_dyn Bruit dynamique de la mesure GPS (m²)
 * @param u_accel Accélération longitudinale IMU (m/s²) - Input de contrôle
 */
function kFilter(z, dt, R_dyn, u_accel) {
    // 1. PREDICTION (Utilise l'IMU)
    const predSpd = kSpd + u_accel * dt;
    const Q_PROC = Q_NOISE * dt; // Bruit de processus (incertitude de la mesure accélérométrique)
    const predUncert = kUncert + Q_PROC; // L'incertitude accumule le bruit de l'IMU

    // 2. CORRECTION (Utilise le GPS)
    // R_dyn: Bruit de la mesure GPS (incertitude latérale, environnement, etc.)
    const K = predUncert / (predUncert + R_dyn);

    // KSpd est la vitesse corrigée (Fusion des deux)
    kSpd = predSpd + K * (z - predSpd);
    
    // 3. MISE À JOUR DE L'INCERTITUDE
    kUncert = (1 - K) * predUncert;
    
    return kSpd;
}


/** CALCULE L'INCERTITUDE DYNAMIQUE R_dyn */
function getKalmanR(acc, alt, pressure, linearAccelMag, accelLongEKF) { 
    let R_gps_base;
    
    // STRATÉGIE CLÉ : Pénalité agressive pour la faible précision GPS
    if (acc > 10.0) { // Si la précision est mauvaise (> 10m)
        R_gps_base = acc * acc * 100.0; // Multiplicateur très agressif pour ignorer le GPS
    } else if (acc > 5.0) {
        R_gps_base = acc * acc * 5.0; // Multiplicateur agressif
    } else {
        R_gps_base = acc * acc; 
    }
    if (R_gps_base < 0.1) R_gps_base = 0.1;

    const envFactor = ENVIRONMENT_FACTORS[selectedEnvironment] || ENVIRONMENT_FACTORS.NORMAL;
    let dynamicMultiplier = envFactor;
    
    // ... (Influence de l'altitude/Underground) ...
    if (alt !== null && alt < 0) { dynamicMultiplier += Math.abs(alt / 50); }
    
    // ... (Influence de l'EKF/Accélération Longitudinale) ...
    const accel_influence = Math.pow(Math.abs(accelLongEKF), 2) * 1.5; 
    dynamicMultiplier += accel_influence;
    
    // ... (Influence de l'accélération linéaire totale) ...
    const imu_noise_influence = Math.pow(linearAccelMag, 2) * 0.1;
    dynamicMultiplier += imu_noise_influence;
    
    dynamicMultiplier += 0.5; // Planche pour l'influence IMU

    return R_gps_base * dynamicMultiplier;
}

// ... (kFilterAltitude, handleDeviceOrientation omis) ...

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

    // Filtrage des accélérations brutes (IMU)
    kAccel.x = ACCEL_FILTER_ALPHA * kAccel.x + (1 - ACCEL_FILTER_ALPHA) * acc_g_raw.x;
    kAccel.y = ACCEL_FILTER_ALPHA * kAccel.y + (1 - ACCEL_FILTER_ALPHA) * acc_g_raw.y;
    kAccel.z = ACCEL_FILTER_ALPHA * kAccel.z + (1 - ACCEL_FILTER_ALPHA) * acc_g_raw.z; 

    // ... (Logique de projection de la gravité) ...
    const phi = global_roll; 
    const theta = global_pitch; 
    const g_local = G_ACC; 

    const G_x_proj = g_local * Math.sin(theta);        
    const G_y_proj = -g_local * Math.sin(phi) * Math.cos(theta); 
    const G_z_proj_abs = g_local * Math.cos(phi) * Math.cos(theta);  
    
    // Accélération linéaire (sans gravité)
    let acc_lin_t_x = kAccel.x - G_x_proj;
    let acc_lin_t_y = kAccel.y - G_y_proj; // NOUVEAU: Accélération latérale (axe Y)
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
    latestAccelLatIMU = acc_lin_t_y; // Stocke l'accélération latérale
    
    // Accélération linéaire totale (pour le calcul du G-Force Total)
    latestLinearAccelMagnitude = Math.sqrt(acc_lin_t_x ** 2 + acc_lin_t_y ** 2 + acc_lin_t_z ** 2); 
    
    // L'accélération longitudinale est utilisée comme input de contrôle pour l'EKF
    latestAccelLongIMU = acc_lin_t_x; 
    
    // MISE À JOUR DES FORCES G
    const currentGTotal = Math.sqrt(acc_lin_t_x ** 2 + acc_lin_t_y ** 2 + acc_lin_t_z ** 2) / G_ACC;
    if (currentGTotal > maxGForce) {
        maxGForce = currentGTotal;
    }

    if ($('accel-vertical-imu')) $('accel-vertical-imu').textContent = `${acc_lin_t_z.toFixed(3)} m/s²`;
    if ($('force-g-vertical')) $('force-g-vertical').textContent = `${(acc_lin_t_z / G_ACC).toFixed(2)} G`;
    
    // NOUVEAU: Mise à jour de l'accélération latérale
    if ($('accel-lateral')) $('accel-lateral').textContent = `${acc_lin_t_y.toFixed(3)} m/s²`;
    if ($('force-g-lateral')) $('force-g-lateral').textContent = `${(acc_lin_t_y / G_ACC).toFixed(2)} G`;
    if ($('force-g-total')) $('force-g-total').textContent = `${currentGTotal.toFixed(2)} G`;
    }
    
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
// FICHIER JS PARTIE 2 : gnss-dashboard-part2.js
// BOUCLE ASTRO, MÉTÉO, FOND DE CIEL & MISE À JOUR PRINCIPALE (updateDisp)
// Dépend de gnss-dashboard-part1.js
// =================================================================

// ... (fonctions astro et météo omises) ...

/** FONCTION PRINCIPALE DE MISE À JOUR (GPS, Kalman, Physique) */
function updateDisp(pos) {
    if (emergencyStopActive) return;
    
    // ... (Logique d'initialisation des coordonnées et du temps) ...
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
    const MASS = parseFloat($('mass-input') ? $('mass-input').value : 70.0); // Récupère la masse utilisateur
    
    // ... (Logique d'initialisation sTime/kAlt) ...
    if (sTime === null) { sTime = now.getTime(); }
    
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

    // --- MISE À JOUR EKF FUSION IMU/GPS ---
    const R_dyn = getKalmanR(effectiveAcc, alt, lastP_hPa, latestLinearAccelMagnitude, latestAccelLongEKF); 
    
    // Utilise la nouvelle fonction kFilter avec l'accélération IMU comme input de contrôle
    const fSpd = kFilter(spd3D, dt, R_dyn, latestAccelLongIMU); 
    const sSpdFE = fSpd < MIN_SPD ? 0 : fSpd; 

    // L'accélération EKF (pour la boucle de R_dyn) est toujours calculée à partir de la vitesse EKF
    let accel_long_ekf_calc = (dt > 0.05) ? (sSpdFE - lastFSpeed) / dt : 0;
    
    // Mise à jour de l'EKF pour la boucle suivante
    if (effectiveAcc < 10.0 && Math.abs(accel_long_ekf_calc) < 50.0) {
        latestAccelLongEKF = accel_long_ekf_calc; 
    }

    lastFSpeed = sSpdFE;
    
    // ... (Logique de mise à jour distM, timeMoving, maxSpd, avgSpdMoving) ...
    if (pos) { 
        distM += sSpdFE * dt * (netherMode ? NETHER_RATIO : 1); 
    }
    if (sSpdFE > MIN_SPD) { timeMoving += dt; }
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    const avgSpdMoving = timeMoving > 0 ? (distM / timeMoving) : 0;
    
    // L'accélération finale affichée est l'accélération IMU (la plus précise pour la dynamique)
    const accel_long_final = latestAccelLongIMU; 
    
    const coriolusForce = 2 * MASS * sSpdFE * W_EARTH * Math.sin(lat * D2R);
    const kineticEnergy = 0.5 * MASS * sSpdFE ** 2;
    const mechanicalPower = accel_long_final * MASS * sSpdFE; 
    
    // ... (Logique de calcul airDensity) ...

    // --- MISE À JOUR DU DOM (GPS/Physique) ---
    $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(5)} km/h`; 
    $('speed-stable-ms').textContent = `${sSpdFE.toFixed(3)} m/s | ${(sSpdFE * 1000).toFixed(0)} mm/s`; 
    $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    $('distance-total-km').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    $('latitude').textContent = lat.toFixed(6);
    $('longitude').textContent = lon.toFixed(6);
    $('gps-precision').textContent = `${acc.toFixed(2)} m`;
    $('gps-accuracy-effective').textContent = `${effectiveAcc.toFixed(2)} m`;
    
    // Affichage des données de Force G
    $('accel-long').textContent = `${accel_long_final.toFixed(3)} m/s²`;
    $('force-g-long').textContent = `${(accel_long_final / G_ACC).toFixed(2)} G`;
    $('speed-error-perc').textContent = `${R_dyn.toFixed(3)} m² (R dyn)`; 
    $('kinetic-energy').textContent = `${kineticEnergy.toFixed(2)} J`;
    $('mechanical-power').textContent = `${mechanicalPower.toFixed(2)} W`;
    $('coriolis-force').textContent = `${coriolusForce.toExponential(2)} N`;
    
    if ($('force-g-max')) $('force-g-max').textContent = `${maxGForce.toFixed(2)} G`;
    
    // ... (Reste de la mise à jour du DOM et de lPos) ...
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
