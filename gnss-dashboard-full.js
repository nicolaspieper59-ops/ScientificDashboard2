// =================================================================
// BLOC A : CONSTANTES, UTILITAIRES, CAPTEURS & CŒUR DE L'EKF (FINAL CORRIGÉ)
// =================================================================

// 1. CONSTANTES GLOBALES
const D2R = Math.PI / 180;
const R2D = 180 / Math.PI;
const G_ACC = 9.80665;
const C_L = 299792458;
const SPEED_SOUND = 343;
const KMH_MS = 3.6;
const EARTH_RADIUS = 6371000;
const IPS_R_MIN = 1.0;
const IPS_R_FACTOR = 0.5;
const ACCEL_FILTER_ALPHA = 0.9;
const STATIC_ACCEL_THRESHOLD = 0.8; 
const ACCEL_BIAS_X = 0.0, ACCEL_BIAS_Y = 0.0, ACCEL_BIAS_Z = 0.0;
const LIGHT_YEAR_M = 9460730472580800;
const UNDERGROUND_THRESHOLD = -50.0; 

// 2. VARIABLES GLOBALES
let wID = null, domID = null, weatherID = null; 
let lat = 0.0, lon = 0.0;
let kSpd = 0.0, kAlt = 0.0, kUncert = 10.0, kAltUncert = 10.0; 
let lPos = null; 
let distM = 0.0, maxSpd = 0.0, timeMoving = 0.0;
let sTime = new Date().getTime(); 
let kAccel = { x: 0, y: 0, z: 0 }; 
let global_pitch = 0, global_roll = 0, currentHeading = 0; 
let latestLinearAccelMagnitude = 0.0, latestVerticalAccelIMU = 0.0;
let accel_long = 0.0; 
let lTimeIMU = null; 
let lastFSpeed = 0.0; 
let currentIPSLat = 43.284539, currentIPSLon = 5.358633, currentIPSAlt = 88.04;
let isGPSTracking = false, emergencyStopActive = false;
let isZVUActive = true, zvuLockTime = 0.0; 

// 3. FONCTIONS UTILITAIRES
function $(id) { return document.getElementById(id); }
function getCDate() { return new Date(); }
function formatTime(totalMinutes) {
    if (isNaN(totalMinutes)) return 'N/A';
    const minutes = Math.floor(totalMinutes % 60);
    const totalHours = Math.floor(totalMinutes / 60);
    const hours = totalHours % 24;
    const seconds = Math.round((totalMinutes - Math.floor(totalMinutes)) * 60);
    return `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}`;
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
function isUnderground(altitudeMeters) {
    return altitudeMeters < UNDERGROUND_THRESHOLD;
}

// 4. FONCTION DE SIMULATION IPS
function getIPSPositionSimulation(dt) {
    const speedDrift = (isZVUActive) ? (0.01 / KMH_MS) : kSpd;
    const headingRad = currentHeading * D2R;
    
    currentIPSLat += (speedDrift * dt * Math.cos(headingRad)) / (EARTH_RADIUS * D2R);
    currentIPSLon += (speedDrift * dt * Math.sin(headingRad)) / (EARTH_RADIUS * D2R * Math.cos(currentIPSLat * D2R));
    
    const acc_sim = IPS_R_MIN + Math.abs(kSpd) * IPS_R_FACTOR; 
    const num_sat = Math.max(2, 12 - Math.floor(acc_sim));
    const pdop = Math.max(2.0, acc_sim * 0.8);
    const raw_speed = (isZVUActive) ? 0.0 : speedDrift * (1 + Math.random() * 0.05);

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

// 5. FONCTIONS CAPTEURS INERTIELS (IMU)
// ... (Dans gnss-dashboard-partA.js)

// 5. FONCTIONS CAPTEURS INERTIELS (IMU)
function handleDeviceMotion(event) {
    if (emergencyStopActive) return;
    
    // ... (Reste du code inchangé) ...
    
    // 2. CORRECTION DE L'INCLINAISON (Projection de G)
    const phi = global_roll; // Roll (gamma) en RADIANS
    const theta = global_pitch; // Pitch (beta) en RADIANS
    const g_local = calculateGravityAtAltitude(kAlt);

    // TEMPORAIRE : Vérification des angles
    console.log(`Angles (R/P): ${Math.round(phi * R2D)}° / ${Math.round(theta * R2D)}°`); 
    
    // CORRECTION FINALE : ... (Reste inchangé)
    // ...
    }
    
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
        const phi = global_roll; // Roll (gamma) en RADIANS
        const theta = global_pitch; // Pitch (beta) en RADIANS
        const g_local = calculateGravityAtAltitude(kAlt);
        
        // CORRECTION FINALE : Projection de G. 
        // L'inversion des signes de projection (par rapport à la théorie standard) est faite 
        // pour correspondre à la convention inhabituelle de certains navigateurs/OS mobiles 
        // (où kAccel.z est positif à plat, +9.81m/s²).
        
        const G_x_proj = g_local * Math.sin(theta);        // Inversion pour annuler kAccel.x
        const G_y_proj = -g_local * Math.sin(phi) * Math.cos(theta); // Inversion pour annuler kAccel.y
        const G_z_proj = g_local * Math.cos(phi) * Math.cos(theta);  // Inversion pour annuler kAccel.z
        
        // 3. ACCÉLÉRATION LINÉAIRE
        // A_lin = A_raw - G_proj
        const acc_lin_t_x = kAccel.x - G_x_proj;
        const acc_lin_t_y = kAccel.y - G_y_proj;
        const acc_lin_t_z = kAccel.z - G_z_proj;
        
        latestVerticalAccelIMU = acc_lin_t_z;
        latestLinearAccelMagnitude = Math.sqrt(
            acc_lin_t_x ** 2 + acc_lin_t_y ** 2 + acc_lin_t_z ** 2
        );
        
    } else { return; }
    
    // 4. ZVU (Zero Velocity Update)
    if (latestLinearAccelMagnitude < STATIC_ACCEL_THRESHOLD) { 
        latestLinearAccelMagnitude = 0.0;
        latestVerticalAccelIMU = 0.0;
        isZVUActive = true;
        zvuLockTime += dt_imu; 
    } else {
        isZVUActive = false;
        zvuLockTime = 0; 
    }
    
    if ($('zvu-lock-status')) $('zvu-lock-status').textContent = isZVUActive ? 'VERROUILLÉ 🟢' : 'NON-VERROUILLÉ 🔴';
    if ($('zvu-lock-time')) $('zvu-lock-time').textContent = `${zvuLockTime.toFixed(2)} s`;

    if ($('accel-vertical-imu')) $('accel-vertical-imu').textContent = `${latestVerticalAccelIMU.toFixed(3)} m/s²`;
    if ($('force-g-vertical')) $('force-g-vertical').textContent = `${(latestVerticalAccelIMU / G_ACC).toFixed(2)} G`;
}

// 6. FONCTION PRINCIPALE DE MISE À JOUR (IPS/EKF) 
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
    
    // --- MISE À JOUR DU DOM (IDs Finalisés) ---
    if ($('latitude')) $('latitude').textContent = `${lat.toFixed(6)}`;
    if ($('longitude')) $('longitude').textContent = `${lon.toFixed(6)}`;
    if ($('altitude-gps')) $('altitude-gps').textContent = `${alt_ips_raw.toFixed(2)} m`;
    if ($('gps-precision')) $('gps-precision').textContent = `${acc.toFixed(2)} m`;
    
    if ($('altitude-ekf')) $('altitude-ekf').textContent = `${kAlt_new.toFixed(2)} m`;
    
    if ($('speed-stable')) $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(5)} km/h`;
    
    if ($('ekf-precision-rdyn')) $('ekf-precision-rdyn').textContent = `${R_dyn.toFixed(3)} m² (R dyn)`; 
    if ($('gps-accuracy-effective')) $('gps-accuracy-effective').textContent = `${acc.toFixed(2)} m`;
    
    if ($('speed-max')) $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    if ($('speed-3d-inst')) $('speed-3d-inst').textContent = `${(spd3D_raw * KMH_MS).toFixed(5)} km/h`;
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${sSpdFE.toFixed(3)} m/s | ${Math.round(sSpdFE * 1000)} mm/s`;
    
    if ($('perc-speed-sound')) $('perc-speed-sound').textContent = `${(sSpdFE / SPEED_SOUND * 100).toFixed(2)} %`;
    if ($('perc-speed-c')) $('perc-speed-c').textContent = `${(sSpdFE / C_L * 100).toExponential(2)} %`;

    if ($('distance-total-km')) $('distance-total-km').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    
    if ($('accel-long')) $('accel-long').textContent = `${accel_long.toFixed(3)} m/s²`;
    if ($('force-g-long')) $('force-g-long').textContent = `${(accel_long / G_ACC).toFixed(2)} G`;
    
    if ($('accel-3d-imu')) $('accel-3d-imu').textContent = `${latestLinearAccelMagnitude.toFixed(3)} m/s²`;
    if ($('force-g-3d-imu')) $('force-g-3d-imu').textContent = `${(latestLinearAccelMagnitude / G_ACC).toFixed(2)} G`;

    if ($('time-elapsed')) $('time-elapsed').textContent = `${((new Date().getTime() - sTime) / 1000).toFixed(2)} s`;
    if ($('time-moving')) $('time-moving').textContent = `${timeMoving.toFixed(2)} s`;

    if ($('underground-status')) $('underground-status').textContent = isUnderground(kAlt_new) ? 'Oui' : 'Non';
    
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
// BLOC B : ASTRO, MÉTÉO, CONTRÔLES & INITIALISATION (FINAL)
// =================================================================

// 1. Fonctions Astro (Dépend des utilitaires et constantes du Bloc A)
function toDays(date) { return date.getUTCDate() + (date.getUTCHours() + date.getUTCMinutes() / 60 + date.getUTCSeconds() / 3600) / 24; }
function solarMeanAnomaly(d) { return D2R * (357.5291 + 0.98560028 * d) % (2 * Math.PI); }
function eclipticLongitude(M) {
    let L = M + D2R * (1.9148 * Math.sin(M) + 0.02 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M));
    return L % (2 * Math.PI);
}
function getMoonPhaseName(fraction) {
    if (fraction < 0.05 || fraction > 0.95) return 'Nouvelle Lune';
    if (fraction < 0.25) return 'Premier Croissant';
    if (fraction < 0.5) return 'Gibbeuse Croissante';
    if (fraction < 0.75) return 'Gibbeuse Décroissante';
    return 'Dernier Croissant';
}
function getSolarTime(date, latA, lonA) {
    const d = toDays(date);
    const M = solarMeanAnomaly(d);
    const L = eclipticLongitude(M);
    const EOT_mins = R2D * (M + 2 * Math.PI - L) / (2 * Math.PI) * 1440 / 360; 
    const UT = date.getUTCHours() + date.getUTCMinutes() / 60 + date.getUTCSeconds() / 3600;
    const LSM_mins = (UT + lonA / 15) * 60;
    const TST_mins = LSM_mins + EOT_mins;
    return { TST: TST_mins, LSM: LSM_mins, EOT: EOT_mins, L: L };
}

// SIMULATION MINIMALE DE suncalc.js
const suncalc = {
    getTimes: (date, latA, lonA) => {
        const offset = lonA / 15 * 3600000;
        const noon = new Date(date.getTime() + (12 * 3600000) - offset);
        return { 
            solarNoon: noon, 
            sunrise: new Date(noon.getTime() - 6 * 3600000), 
            sunset: new Date(noon.getTime() + 6 * 3600000)   
        };
    },
    getMoonIllumination: (date) => ({ fraction: (Date.now() / 10000000000) % 1 }),
    getPosition: (date, latA, lonA) => ({ 
        altitude: -1.33 * D2R, 
        azimuth: (Date.now() / 100000) % (2 * Math.PI) 
    })
};

function updateAstro(latA, lonA) {
    if (latA === 0.0 && lonA === 0.0) return;
    const date = getCDate();
    const times = suncalc.getTimes(date, latA, lonA);
    const moon = suncalc.getMoonIllumination(date);
    const sunPos = suncalc.getPosition(date, latA, lonA);
    const solarTime = getSolarTime(date, latA, lonA);
    
    // Heures Solaires
    if ($('tst-value')) $('tst-value').textContent = formatTime(solarTime.TST);
    if ($('lsm-value')) $('lsm-value').textContent = formatTime(solarTime.LSM); 
    
    // Position Soleil
    if ($('sun-elevation')) $('sun-elevation').textContent = `${(sunPos.altitude * R2D).toFixed(2)} °`;
    if ($('zenith-angle')) $('zenith-angle').textContent = `${(90 - sunPos.altitude * R2D).toFixed(2)} °`;
    
    // Lever/Coucher (UTC)
    if ($('sunrise-utc')) $('sunrise-utc').textContent = times.sunrise.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit', second: '2-digit', timeZone: 'UTC' });
    if ($('sunset-utc')) $('sunset-utc').textContent = times.sunset.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit', second: '2-digit', timeZone: 'UTC' });
    
    // Lune
    if ($('moon-phase-display')) $('moon-phase-display').textContent = getMoonPhaseName(moon.fraction);
}

// 2. Fonctions Météo (Simulées)
function updateWeather(latA, lonA) {
    if (latA === 0.0 && lonA === 0.0) return;
    const temp = 24.6 + Math.sin(Date.now() / 100000) * 2;
    const pressure = 1004.94 + Math.cos(Date.now() / 50000) * 2;
    const humidity = 69.4 + Math.sin(Date.now() / 70000) * 5;
    const windSpeed = 9.3 + Math.random() * 2;

    if ($('air-temp')) $('air-temp').textContent = `${temp.toFixed(1)} °C`;
    if ($('air-pressure')) $('air-pressure').textContent = `${pressure.toFixed(2)} hPa`;
    if ($('air-humidity')) $('air-humidity').textContent = `${humidity.toFixed(1)} %`;
    if ($('wind-speed-ms')) $('wind-speed-ms').textContent = `${windSpeed.toFixed(1)} m/s`;
}

// 3. Fonctions Orientation et Compass
// ... (Dans gnss-dashboard-partB.js)

// 3. Fonctions Orientation et Compass
function handleDeviceOrientation(event) {
    if (emergencyStopActive) return;
    if (event.alpha !== null) {
        // Enregistre en RADIANS pour la soustraction de G dans le Bloc A
        global_pitch = event.beta ? event.beta * D2R : 0; 
        global_roll = event.gamma ? event.gamma * D2R : 0; 
        currentHeading = event.alpha ?? 0;

        // TEMPORAIRE : Vérification des données brutes
        console.log(`Orientation brute: A=${event.alpha.toFixed(0)}, P=${event.beta.toFixed(0)}, R=${event.gamma.toFixed(0)}`);
    }
}
// ...
        // Enregistre en RADIANS pour la soustraction de G dans le Bloc A
        global_pitch = event.beta ? event.beta * D2R : 0; 
        global_roll = event.gamma ? event.gamma * D2R : 0; 
        currentHeading = event.alpha ?? 0;
    }
}
function calculateBearing(lat1, lon1, lat2, lon2) {
    const phi1 = lat1 * D2R; const phi2 = lat2 * D2R; const lambda1 = lon1 * D2R; const lambda2 = lon2 * D2R;
    const y = Math.sin(lambda2 - lambda1) * Math.cos(phi2);
    const x = Math.cos(phi1) * Math.sin(phi2) - Math.sin(phi1) * Math.cos(phi2) * Math.cos(lambda2 - lambda1);
    const bearingRad = Math.atan2(y, x);
    return (bearingRad * R2D + 360) % 360; 
}
function updateCompasses(latA, lonA) {
    if ($('compass-heading')) $('compass-heading').textContent = `${currentHeading.toFixed(1)} °`;
    
    const latestMagneticFieldMagnitude = 50 + (Math.sin(Date.now() / 5000) * 5) + Math.random() * 2; 
    if ($('magnetic-field')) $('magnetic-field').textContent = `${latestMagneticFieldMagnitude.toFixed(2)} µT`;
    
    const TARGET_LAT = 48.8584; const TARGET_LON = 2.2945; 
    if (latA !== 0 || lonA !== 0) {
        const distanceKm = calculateDistanceHaversine(latA, lonA, TARGET_LAT, TARGET_LON) / 1000;
        const targetBearing = calculateBearing(latA, lonA, TARGET_LAT, TARGET_LON);
        
        if ($('target-distance')) $('target-distance').textContent = `${distanceKm.toFixed(3)} km`;
        if ($('target-bearing')) $('target-bearing').textContent = `${targetBearing.toFixed(1)} °`;
    } else {
        if ($('target-distance')) $('target-distance').textContent = 'N/A';
        if ($('target-bearing')) $('target-bearing').textContent = 'N/A';
    }
}

// 4. Fonctions de Contrôle et Initialisation 

function syncH() {
    const now = getCDate();
    // Synchronisation Heure Locale et Date Locale
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
    window.addEventListener('devicemotion', handleDeviceMotion, true);
    window.addEventListener('deviceorientation', handleDeviceOrientation, true);
    if ($('emergency-stop-btn')) { $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: INACTIF 🟢"; }
}
function resetAll() {
    distM = 0.0; maxSpd = 0.0; timeMoving = 0.0; zvuLockTime = 0.0; sTime = new Date().getTime(); 
    kSpd = 0.0; kAlt = 0.0; kUncert = 10.0; kAltUncert = 10.0; lPos = null;
    
    if ($('speed-stable')) $('speed-stable').textContent = '0.00000 km/h';
    if ($('speed-max')) $('speed-max').textContent = '0.00000 km/h';
    if ($('distance-total-km')) $('distance-total-km').textContent = '0.000 km | 0.00 m';
    if ($('time-moving')) $('time-moving').textContent = '0.00 s';
    
    if ($('reset-trajectory-btn') && typeof resetTrajectory === 'function') resetTrajectory();
}

document.addEventListener('DOMContentLoaded', () => {
    
    // Initialisation du temps
    syncH(); 
    
    // Configuration des événements des boutons
    $('toggle-gps-btn')?.addEventListener('click', toggleGPS);
    $('reset-all-btn')?.addEventListener('click', resetAll);
    $('emergency-stop-btn')?.addEventListener('click', () => { emergencyStopActive ? resumeSystem() : emergencyStop(); });

    // Démarrage initial des capteurs
    resumeSystem(); 

    // Initialisation des données pour éviter les N/A au démarrage
    updateWeather(currentIPSLat, currentIPSLon);
    updateAstro(currentIPSLat, currentIPSLon); 
    updateCompasses(currentIPSLat, currentIPSLon);

    // Boucles de mise à jour (Heure et Météo)
    domID = setInterval(syncH, 1000);
    weatherID = setInterval(() => { if (lat !== 0 || lon !== 0) updateWeather(lat, lon); }, 30000); 
    
    setTimeout(() => { /* initGlobe(lat, lon, kAlt); */ }, 500);
});
