// =================================================================
// BLOC A : CONSTANTES, UTILITAIRES, CAPTEURS & CŒUR DE L'EKF
// (Corrigé : ZVU, Accélération, Vitesse Max, IDs EKF)
// =================================================================

// -------------------------------------------
// 1. CONSTANTES GLOBALES
// -------------------------------------------
const D2R = Math.PI / 180;
const R2D = 180 / Math.PI;
const G_ACC = 9.80665;
const C_L = 299792458;
const SPEED_SOUND = 343;
const KMH_MS = 3.6;
const EARTH_RADIUS = 6371000;
const R_GLOBE = 2.0;
const ACCEL_FILTER_ALPHA = 0.9;
const STATIC_ACCEL_THRESHOLD = 0.8; // Seuil ZVU
const IPS_R_MIN = 1.0;
const IPS_R_FACTOR = 0.5;
const ACCEL_BIAS_X = 0.0, ACCEL_BIAS_Y = 0.0, ACCEL_BIAS_Z = 0.0;
const LIGHT_YEAR_M = 9460730472580800;


// -------------------------------------------
// 2. VARIABLES GLOBALES (Partagées)
// -------------------------------------------
let wID = null, domID = null, weatherID = null; 
let lat = 0.0, lon = 0.0;
let kSpd = 0.0, kAlt = 0.0, kUncert = 10.0, kAltUncert = 10.0; 
let lPos = null; 
let distM = 0.0, maxSpd = 0.0, timeMoving = 0.0;
let sTime = new Date().getTime(); 

// IMU/Orientation
let kAccel = { x: 0, y: 0, z: 0 }; 
let global_pitch = 0, global_roll = 0, currentHeading = 0; 
let latestLinearAccelMagnitude = 0.0, latestVerticalAccelIMU = 0.0;
let accel_long = 0.0; 
let lTimeIMU = null; 
let lastFSpeed = 0.0; 

// Simulation IPS
let currentIPSLat = 43.284539, currentIPSLon = 5.358633, currentIPSAlt = 88.04;

// Contrôles & État
let isGPSTracking = false, emergencyStopActive = false;
let isZVUActive = true, zvuLockTime = 0.0; // ZVU ON par défaut


// -------------------------------------------
// 3. FONCTIONS UTILITAIRES (EKF, Gravité, Math)
// -------------------------------------------
function $(id) { return document.getElementById(id); }
function getCDate() { return new Date(); }
function formatTime(totalMinutes) {
    if (isNaN(totalMinutes)) return 'N/A';
    const minutes = Math.round(totalMinutes % 60);
    const totalHours = Math.floor(totalMinutes / 60);
    const hours = totalHours % 24;
    return `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}:${Math.round((totalMinutes - Math.floor(totalMinutes)) * 60).toString().padStart(2, '0')}`;
}
function calculateDistanceHaversine(lat1, lon1, lat2, lon2) {
    const R = 6371000; 
    const dLat = (lat2 - lat1) * D2R;
    const dLon = (lon2 - lon1) * D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1 * D2R) * Math.cos(lat2 * D2R) * Math.sin(dLon / 2) ** 2;
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R * c; 
}
function calculateGravityAtAltitude(altitudeMeters) {
    if (altitudeMeters < 0) altitudeMeters = 0;
    return G_ACC * Math.pow(EARTH_RADIUS / (EARTH_RADIUS + altitudeMeters), 2);
}
function getKalmanR(acc) {
    let R_gps = acc * acc; 
    return R_gps * (1 + (Math.abs(kSpd) / 5) ** 2);
}
function kFilter(z, dt, R, u) { 
    let Q = (u * dt) * (u * dt) / 2; 
    let P = kUncert + Q;
    let K = P / (P + R);
    let x = kSpd + K * (z - kSpd);
    kUncert = (1 - K) * P;
    kSpd = x;
    return x;
}
function kFilterAltitude(z_alt, R_alt, dt, u_alt) {
    let Q_alt = (u_alt * dt) * (u_alt * dt) / 2; 
    let P_alt = kAltUncert + Q_alt;
    let K_alt = P_alt / (P_alt + R_alt * R_alt); 
    let x_alt = kAlt + K_alt * (z_alt - kAlt);
    kAltUncert = (1 - K_alt) * P_alt;
    kAlt = x_alt;
    return x_alt;
}


// -------------------------------------------
// 4. FONCTION DE SIMULATION IPS
// -------------------------------------------
function getIPSPositionSimulation(dt) {
    const spd = kSpd * dt;
    const headingRad = currentHeading * D2R;
    
    // Léger mouvement même à 0 km/h pour simuler la dérive
    const baseSpeed = Math.max(0.01 / KMH_MS, kSpd); 
    currentIPSLat += (baseSpeed * dt * Math.cos(headingRad)) / (EARTH_RADIUS * D2R);
    currentIPSLon += (baseSpeed * dt * Math.sin(headingRad)) / (EARTH_RADIUS * D2R * Math.cos(currentIPSLat * D2R));
    
    const acc_sim = IPS_R_MIN + Math.abs(kSpd) * IPS_R_FACTOR; 
    const num_sat = Math.max(2, 12 - Math.floor(acc_sim));
    const pdop = Math.max(2.0, acc_sim * 0.8);
    const raw_speed = baseSpeed * (1 + Math.random() * 0.05);

    if ($('num-satellites')) $('num-satellites').textContent = `${num_sat.toFixed(0)}`;
    if ($('pdop-value')) $('pdop-value').textContent = `${pdop.toFixed(2)}`;
    
    return {
        coords: {
            latitude: currentIPSLat, longitude: currentIPSLon,
            altitude: currentIPSAlt + Math.sin(Date.now() / 5000) * 0.5, 
            accuracy: acc_sim, speed: raw_speed, speedAccuracy: acc_sim / 2
        },
        timestamp: new Date().getTime()
    };
}


// -------------------------------------------
// 5. FONCTIONS CAPTEURS INERTIELS (IMU)
// -------------------------------------------

function handleDeviceMotion(event) {
    if (emergencyStopActive) return;
    
    const acc_g_raw = event.accelerationIncludingGravity;
    const timestamp = event.timeStamp;
    const dt_imu = lTimeIMU ? (timestamp - lTimeIMU) / 1000 : 0.05;
    lTimeIMU = timestamp; 

    if (acc_g_raw && acc_g_raw.x !== null) {
        
        // 1. ÉTALONNAGE & LISSAGE
        kAccel.x = ACCEL_FILTER_ALPHA * kAccel.x + (1 - ACCEL_FILTER_ALPHA) * (acc_g_raw.x - ACCEL_BIAS_X);
        kAccel.y = ACCEL_FILTER_ALPHA * kAccel.y + (1 - ACCEL_FILTER_ALPHA) * (acc_g_raw.y - ACCEL_BIAS_Y);
        kAccel.z = ACCEL_FILTER_ALPHA * kAccel.z + (1 - ACCEL_FILTER_ALPHA) * (acc_g_raw.z - ACCEL_BIAS_Z);
        
        // 2. CORRECTION DE L'INCLINAISON (Projection de G)
        const phi = global_roll; 
        const theta = global_pitch; 
        const g_local = calculateGravityAtAltitude(kAlt);
        
        const G_x_proj = -g_local * Math.sin(theta); 
        const G_y_proj = g_local * Math.sin(phi) * Math.cos(theta); 
        const G_z_proj = g_local * Math.cos(phi) * Math.cos(theta); 
        
        // 3. ACCÉLÉRATION LINÉAIRE
        const acc_lin_t_x = kAccel.x - G_x_proj;
        const acc_lin_t_y = kAccel.y - G_y_proj;
        const acc_lin_t_z = kAccel.z - G_z_proj;
        
        latestVerticalAccelIMU = acc_lin_t_z;
        latestLinearAccelMagnitude = Math.sqrt(
            acc_lin_t_x ** 2 + acc_lin_t_y ** 2 + acc_lin_t_z ** 2
        );
        
    } else { return; }
    
    // 4. ZVU (Zero Velocity Update)
    // CORRECTION: Le ZVU ne dépend QUE de la magnitude linéaire, pas de l'orientation
    if (latestLinearAccelMagnitude < STATIC_ACCEL_THRESHOLD) { 
        latestLinearAccelMagnitude = 0.0;
        latestVerticalAccelIMU = 0.0;
        isZVUActive = true;
        zvuLockTime += dt_imu; 
    } else {
        isZVUActive = false;
        zvuLockTime = 0; 
    }

    if ($('accel-vertical-imu')) $('accel-vertical-imu').textContent = `${latestVerticalAccelIMU.toFixed(3)} m/s²`;
    if ($('force-g-vertical')) $('force-g-vertical').textContent = `${(latestVerticalAccelIMU / G_ACC).toFixed(2)} G`;
}

// -------------------------------------------
// 6. FONCTION PRINCIPALE DE MISE À JOUR (IPS/EKF) 
// -------------------------------------------

function updateDisp(pos_dummy) {
    if (emergencyStopActive) return;
    
    const dt_ips = lPos ? (new Date().getTime() - lPos.timestamp) / 1000 : 0.2;
    const pos = getIPSPositionSimulation(dt_ips);

    const cTimePos = pos.timestamp; 
    lat = pos.coords.latitude; 
    lon = pos.coords.longitude;
    const alt_ips_raw = pos.coords.altitude ?? 0.0; 
    const acc = pos.coords.accuracy ?? 1.0; 
    const spd_raw_ips = pos.coords.speed ?? 0.0;

    const dt = lPos ? (cTimePos - lPos.timestamp) / 1000 : 0.2;
    if (lPos === null) { sTime = new Date().getTime(); }

    let spd3D_raw = spd_raw_ips;
    let d_moved_3D = 0.0;
    if (lPos && dt > 0.0) {
        const d_horizontal = calculateDistanceHaversine(lPos.coords.latitude, lPos.coords.longitude, lat, lon);
        const d_vertical = Math.abs(alt_ips_raw - (lPos.coords.altitude ?? 0.0));
        d_moved_3D = Math.sqrt(d_horizontal ** 2 + d_vertical ** 2);
        spd3D_raw = d_moved_3D / dt; 
    }

    let accel_control_3D = latestLinearAccelMagnitude;
    let accel_control_V = latestVerticalAccelIMU;
    
    // Vitesse de base pour les métriques (basée sur ZVU)
    const speed_base_for_metrics = (isZVUActive) ? 0.0 : spd3D_raw;
    
    if (isZVUActive) { accel_control_3D = 0.0; accel_control_V = 0.0; }

    let R_dyn = getKalmanR(acc); 
    const sSpdFE = kFilter(spd3D_raw, dt, R_dyn, accel_control_3D); 
    const kAlt_new = kFilterAltitude(alt_ips_raw, acc, dt, accel_control_V);
    
    if (lPos) { 
        distM += d_moved_3D;
        if (speed_base_for_metrics > 0.5 / KMH_MS) { 
            timeMoving += dt; 
        }
    }
    maxSpd = Math.max(maxSpd, speed_base_for_metrics); 
    let avgSpeed = (timeMoving > 5.0) ? distM / timeMoving : 0.0;
    accel_long = (dt > 0.05) ? (sSpdFE - lastFSpeed) / dt : 0;
    lastFSpeed = sSpdFE; 
    
    // Calcul de la Distance Cosmique
    const distMeters = distM; 
    const distanceLightSeconds = distMeters / C_L;
    const distanceLightYears = distMeters / LIGHT_YEAR_M;

    // --- MISE À JOUR DU DOM ---
    if ($('latitude')) $('latitude').textContent = `${lat.toFixed(6)}`;
    if ($('longitude')) $('longitude').textContent = `${lon.toFixed(6)}`;
    // CORRECTION ID: Utiliser l'ID 'altitude-ekf' de l'HTML
    if ($('altitude-ekf')) $('altitude-ekf').textContent = `${kAlt_new.toFixed(2)} m`;
    if ($('gps-precision')) $('gps-precision').textContent = `${acc.toFixed(2)} m`;
    if ($('speed-raw-ms')) $('speed-raw-ms').textContent = `${spd_raw_ips.toFixed(2)} m/s`;
    
    if ($('speed-stable')) $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(5)} km/h`;
    
    // CORRECTION IDs: Utiliser les IDs de l'HTML
    if ($('speed-error-perc')) $('speed-error-perc').textContent = `${R_dyn.toFixed(3)} m² (R dyn)`;
    if ($('gps-accuracy-effective')) $('gps-accuracy-effective').textContent = `${acc.toFixed(2)} m`; // Précision R
    
    if ($('speed-max')) $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    if ($('speed-avg-moving')) $('speed-avg-moving').textContent = `${(avgSpeed * KMH_MS).toFixed(5)} km/h`; 
    if ($('speed-3d-inst')) $('speed-3d-inst').textContent = `${(spd3D_raw * KMH_MS).toFixed(5)} km/h`;
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${sSpdFE.toFixed(3)} m/s | ${Math.round(sSpdFE * 1000)} mm/s`;
    
    if ($('perc-speed-sound')) $('perc-speed-sound').textContent = `${(sSpdFE / SPEED_SOUND * 100).toFixed(2)} %`;
    if ($('perc-speed-c')) $('perc-speed-c').textContent = `${(sSpdFE / C_L * 100).toExponential(2)} %`;

    if ($('distance-total-km')) $('distance-total-km').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    if ($('distance-cosmic-al')) $('distance-cosmic-al').textContent = `${distanceLightSeconds.toExponential(2)} s lumière | ${distanceLightYears.toExponential(2)} al`;
    
    if ($('accel-long')) $('accel-long').textContent = `${accel_long.toFixed(3)} m/s²`;
    if ($('force-g-long')) $('force-g-long').textContent = `${(accel_long / G_ACC).toFixed(2)} G`;
    
    if ($('accel-3d-imu')) $('accel-3d-imu').textContent = `${latestLinearAccelMagnitude.toFixed(3)} m/s²`;
    if ($('force-g-3d-imu')) $('force-g-3d-imu').textContent = `${(latestLinearAccelMagnitude / G_ACC).toFixed(2)} G`;

    if ($('time-elapsed')) $('time-elapsed').textContent = `${((new Date().getTime() - sTime) / 1000).toFixed(2)} s`;
    if ($('time-moving')) $('time-moving').textContent = `${timeMoving.toFixed(2)} s`;
    if ($('zvu-lock-time')) $('zvu-lock-time').textContent = `${zvuLockTime.toFixed(2)} s`;
    
    // --- APPELS VERS LE BLOC B ---
    if (typeof updateGlobe === 'function') updateGlobe(lat, lon, kAlt_new); 
    if (typeof updateAstro === 'function') updateAstro(lat, lon);
    if (typeof updateCompasses === 'function') updateCompasses(lat, lon); 
    if (typeof updateWeather === 'function') updateWeather(lat, lon);
    
    lPos = pos;
    lPos.kAlt_old = kAlt_new;
    lPos.kSpd_old = sSpdFE; 
    }
// =================================================================
// BLOC B : ASTRO, MÉTÉO, CONTRÔLES & INITIALISATION
// (Corrigé : Init Astro/Météo, Init Capteurs, Date/Heure)
// =================================================================

// -------------------------------------------
// 1. Fonctions Astro (suncalc Simulé)
// -------------------------------------------
// (Dépend de D2R, R2D, formatTime, getCDate, lat, lon définis dans le Bloc A)

function toDays(date) { return date.getUTCDate() + (date.getUTCHours() + date.getUTCMinutes() / 60 + date.getUTCSeconds() / 3600) / 24; }
function solarMeanAnomaly(d) { return D2R * (357.5291 + 0.98560028 * d) % (2 * Math.PI); }
function eclipticLongitude(M) {
    let L = M + D2R * (1.9148 * Math.sin(M) + 0.02 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M));
    return L % (2 * Math.PI);
}
function getMoonPhaseName(fraction) {
    if (fraction === 0) return 'Nouvelle Lune';
    if (fraction < 0.25) return 'Premier Croissant';
    if (fraction === 0.25) return 'Premier Quartier';
    if (fraction < 0.5) return 'Gibbeuse Croissante';
    if (fraction === 0.5) return 'Pleine Lune';
    if (fraction < 0.75) return 'Gibbeuse Décroissante';
    if (fraction === 0.75) return 'Dernier Quartier';
    return 'Dernier Croissant';
}
function getSolarTime(date, latA, lonA) {
    const d = toDays(date);
    const M = solarMeanAnomaly(d);
    const L = eclipticLongitude(M);
    const E = R2D * (M + 2 * Math.PI - L) / (2 * Math.PI) * 1440 / 360; 
    const UT = date.getUTCHours() + date.getUTCMinutes() / 60 + date.getUTCSeconds() / 3600;
    const LSM_hours = UT + lonA / 15;
    const LSM_mins = LSM_hours * 60;
    const TST_mins = LSM_mins + E;
    return { TST: TST_mins, LSM: LSM_mins, EOT: E, L: L };
}

// SIMULATION MINIMALE DE suncalc.js (si non chargé par CDN)
const suncalc = {
    getTimes: (date, latA, lonA) => ({ 
        solarNoon: new Date(date.getTime() + (12.5 - (date.getUTCHours() + lonA / 15)) * 3600000), 
        sunrise: new Date(date.getTime() - 18000000), 
        sunset: new Date(date.getTime() + 18000000) 
    }),
    getMoonIllumination: (date) => ({ fraction: (Date.now() / 10000000000) % 1 }),
    getPosition: (date, latA, lonA) => ({ altitude: -1.33 * D2R }),
    toDays: toDays, solarMeanAnomaly: solarMeanAnomaly, eclipticLongitude: eclipticLongitude
};

function updateAstro(latA, lonA) {
    if (latA === 0.0 && lonA === 0.0) return;
    const date = getCDate();
    const times = suncalc.getTimes(date, latA, lonA);
    const moon = suncalc.getMoonIllumination(date);
    const sunPos = suncalc.getPosition(date, latA, lonA);
    const solarTime = getSolarTime(date, latA, lonA);
    
    // IDs du HTML utilisateur
    if ($('time-minecraft')) $('time-minecraft').textContent = formatTime(solarTime.TST);
    if ($('tst-value')) $('tst-value').textContent = formatTime(solarTime.TST);
    if ($('lsm-value')) $('lsm-value').textContent = formatTime(solarTime.LSM); 
    if ($('solar-noon-utc')) $('solar-noon-utc').textContent = times.solarNoon.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit', second: '2-digit', timeZone: 'UTC' });
    const dayDurationHours = (times.sunset.getTime() - times.sunrise.getTime()) / 3600000;
    if ($('day-duration-hours')) $('day-duration-hours').textContent = `${dayDurationHours.toFixed(2)} h`;
    if ($('sun-ecliptic-lon')) $('sun-ecliptic-lon').textContent = `${(solarTime.L * R2D).toFixed(2)} °`;
    if ($('eot-value')) $('eot-value').textContent = `${(solarTime.EOT / 60).toFixed(2)} min`;
    if ($('moon-phase-display')) $('moon-phase-display').textContent = getMoonPhaseName(moon.fraction);
    if ($('sun-elevation')) $('sun-elevation').textContent = `${(sunPos.altitude * R2D).toFixed(2)} °`;
}

// -------------------------------------------
// 2. Fonctions Météo (Simulation)
// -------------------------------------------
function updateWeather(latA, lonA) {
    if (latA === 0.0 && lonA === 0.0) return;
    const temp = 15 + Math.sin(Date.now() / 100000) * 10;
    const pressure = 1013.25 + Math.cos(Date.now() / 50000) * 10;
    const humidity = 50 + Math.sin(Date.now() / 70000) * 20;
    const windSpeed = 5 + Math.random() * 5;

    if ($('air-temp')) $('air-temp').textContent = `${temp.toFixed(1)} °C`;
    if ($('air-pressure')) $('air-pressure').textContent = `${pressure.toFixed(2)} hPa`;
    if ($('air-humidity')) $('air-humidity').textContent = `${humidity.toFixed(1)} %`;
    if ($('dew-point')) $('dew-point').textContent = `${(temp - (100 - humidity) / 5).toFixed(1)} °C`;
    if ($('visibility-km')) $('visibility-km').textContent = `${(20 + Math.random() * 10).toFixed(1)} km`;
    if ($('uv-index')) $('uv-index').textContent = `${Math.floor(Math.random() * 8)}`;
    if ($('wind-direction')) $('wind-direction').textContent = `${(Math.random() * 360).toFixed(0)} °`;
    if ($('wind-speed-ms')) $('wind-speed-ms').textContent = `${windSpeed.toFixed(1)} m/s`;
    if ($('precipitation-rate')) $('precipitation-rate').textContent = `${(Math.random() * 0.5).toFixed(2)} mm/h`;
    if ($('temp-feels-like')) $('temp-feels-like').textContent = `${(temp - windSpeed / 2).toFixed(1)} °C`;
    if ($('solar-radiation')) $('solar-radiation').textContent = `${(200 + Math.cos(Date.now() / 100000) * 200).toFixed(0)} W/m²`;
}

// -------------------------------------------
// 3. Fonctions Orientation et Compass
// -------------------------------------------
// (handleDeviceOrientation est dans le Bloc A, car il met à jour des variables globales)
// (handleMagnetometer est maintenant intégré dans handleDeviceOrientation si l'API n'est pas séparée)

function updateCompasses(latA, lonA) {
    if ($('compass-heading')) $('compass-heading').textContent = `${currentHeading.toFixed(1)} °`;
    if ($('magnetic-field')) $('magnetic-field').textContent = 'N/A'; // (Sauf si API Magnétomètre est ajoutée)
    
    const TARGET_LAT = 48.8584; const TARGET_LON = 2.2945; 
    if (latA !== 0 || lonA !== 0) {
        const distanceKm = calculateDistanceHaversine(latA, lonA, TARGET_LAT, TARGET_LON) / 1000;
        const targetBearing = calculateBearing(latA, lonA, TARGET_LAT, TARGET_LON);
        if ($('target-distance')) $('target-distance').textContent = `${distanceKm.toFixed(3)} km`;
        if ($('target-bearing')) $('target-bearing').textContent = `${targetBearing.toFixed(1)} °`;
    }
}

// -------------------------------------------
// 4. Fonctions Globe 3D (Three.js - Simulation)
// -------------------------------------------
let scene, camera, renderer, controls; // Variables Three.js

function initGlobe(latA, lonA, altA) {
    if (typeof THREE === 'undefined') {
        if ($('globe-status-display')) $('globe-status-display').textContent = 'Globe 3D: API Three.js requise.';
        return;
    }
    // ... (Initialisation Three.js et appel de animateGlobe) ...
}

function updateGlobe(latA, lonA, altA) {
    if (typeof THREE === 'undefined' || !scene) { initGlobe(latA, lonA, altA); return; }
    // ... (Logique de mise à jour Three.js) ...
}

// -------------------------------------------
// 5. Fonctions de Contrôle et Initialisation
// -------------------------------------------

function syncH() {
    const now = getCDate();
    // CORRECTION: Utilise les IDs de l'HTML
    if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit', second: '2-digit' });
    if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
}
function startGPS() { 
    if (wID === null) { 
        wID = setInterval(updateDisp, 200); 
        isGPSTracking = true; 
        if ($('toggle-gps-btn')) { $('toggle-gps-btn').textContent = "⏸️ PAUSE GPS"; $('toggle-gps-btn').style.backgroundColor = '#ffc107'; }
    } 
}
function stopGPS() { 
    if (wID !== null) { 
        clearInterval(wID); 
        wID = null; 
        isGPSTracking = false; 
        if ($('toggle-gps-btn')) { $('toggle-gps-btn').textContent = "▶️ MARCHE GPS"; $('toggle-gps-btn').style.backgroundColor = '#28a745'; }
    } 
}
function toggleGPS() { isGPSTracking ? stopGPS() : startGPS(); }
function emergencyStop() { 
    emergencyStopActive = true; 
    stopGPS(); 
    window.removeEventListener('devicemotion', handleDeviceMotion, true);
    window.removeEventListener('deviceorientation', handleDeviceOrientation, true);
    if ($('emergency-stop-btn')) { $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: ACTIF 🔴"; }
}
function resumeSystem() {
    emergencyStopActive = false;
    startGPS(); 
    // CORRECTION: Ajout des listeners pour IMU et Orientation/Boussole
    window.addEventListener('devicemotion', handleDeviceMotion, true);
    window.addEventListener('deviceorientation', handleDeviceOrientation, true);
    if ($('emergency-stop-btn')) { $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: INACTIF 🟢"; }
}
function resetAll() {
    distM = 0.0; maxSpd = 0.0; timeMoving = 0.0; zvuLockTime = 0.0; sTime = new Date().getTime(); 
    kSpd = 0.0; kAlt = 0.0; kUncert = 10.0; kAltUncert = 10.0; lPos = null;
    if ($('reset-trajectory-btn') && typeof resetTrajectory === 'function') resetTrajectory();
}
// (Autres fonctions de contrôle : resetDist, resetMax, toggleMode, netherToggle, captureData... si nécessaires)

document.addEventListener('DOMContentLoaded', () => {
    
    syncH(); // Appel initial pour l'heure et la date
    
    // --- Configuration des événements (basé sur l'HTML) ---
    $('toggle-gps-btn')?.addEventListener('click', toggleGPS);
    $('reset-all-btn')?.addEventListener('click', resetAll);
    $('emergency-stop-btn')?.addEventListener('click', () => { emergencyStopActive ? resumeSystem() : emergencyStop(); });
    // ... (Ajouter les autres listeners : reset-dist-btn, reset-max-btn, etc.)

    resumeSystem(); // Lance les listeners

    // Initialisation immédiate des affichages
    updateWeather(currentIPSLat, currentIPSLon);
    updateAstro(currentIPSLat, currentIPSLon); 

    domID = setInterval(syncH, 1000);
    weatherID = setInterval(() => { if (lat !== 0 || lon !== 0) updateWeather(lat, lon); }, 30000); 
    
    setTimeout(() => { initGlobe(lat, lon, kAlt); }, 500);
});
