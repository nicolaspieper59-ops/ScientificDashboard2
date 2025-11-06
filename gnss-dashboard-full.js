// =================================================================
// FICHIER JS PARTIE 1 : gnss-dashboard-part1.js (Constantes)
// =================================================================

// -------------------------------------------
// Constantes Mathématiques & Unités
// -------------------------------------------
const D2R = Math.PI / 180; // Conversion Degrés vers Radians
const R2D = 180 / Math.PI; // Conversion Radians vers Degrés

// -------------------------------------------
// Gravité et Modèle Terrestre
// -------------------------------------------
const G_ACC = 9.80665; // Accélération standard de la gravité (m/s²)
const EARTH_RADIUS = 6371000; // Rayon moyen de la Terre (m)

// -------------------------------------------
// Constantes Physiques Générales
// -------------------------------------------
const KMH_MS = 3.6; // Facteur de conversion m/s vers km/h
const SPEED_SOUND = 343; // Vitesse du son (m/s)
const C_L = 299792458; // Vitesse de la lumière (m/s)

// -------------------------------------------
// Constantes IPS (Indoor Positioning System)
// -------------------------------------------
const IPS_R_MIN = 0.5; // Précision minimale simulée pour l'IPS (m)
const IPS_R_FACTOR = 3.0; // Facteur de bruit (incertitude augmente avec la vitesse EKF)

// -------------------------------------------
// Paramètres d'Étalonnage IMU (Biais/Offset)
// -------------------------------------------
const ACCEL_BIAS_X = 0.05; // Biais sur l'accélération X (m/s²)
const ACCEL_BIAS_Y = -0.03; // Biais sur l'accélération Y (m/s²)
const ACCEL_BIAS_Z = 0.1; // Biais sur l'accélération Z (m/s²)
const GYRO_BIAS_X = 0.001; // Biais sur le gyroscope X (rad/s)
const GYRO_BIAS_Y = -0.002; // Biais sur le gyroscope Y (rad/s)
const GYRO_BIAS_Z = 0.003; // Biais sur le gyroscope Z (rad/s)

// -------------------------------------------
// Paramètres de Filtrage EKF / IMU
// -------------------------------------------
const ACCEL_FILTER_ALPHA = 0.8; // Lissage IMU
const STATIC_ACCEL_THRESHOLD = 0.8; // Seuil ZVU (m/s²). Corrigé (ni 0.5 ni 3.5)

// -------------------------------------------
// Paramètres de Visualisation 3D (Three.js)
// -------------------------------------------
const R_GLOBE = 2.0; 
const MAX_TRAJECTORY_POINTS = 500;
// =========
// =================================================================
// FICHIER JS PARTIE 2, BLOC A : gnss-dashboard-part2_blocA.js
// (Intègre ZVU, Accélération Corrigée, Vitesse Max/Moyenne/Distance sur Vitesse Instantanée)
// =================================================================

// -------------------------------------------
// 1. VARIABLES GLOBALES
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
let latestLinearAccelMagnitude = 0.0, latestVerticalAccelIMU = 0.0, latestMagneticFieldMagnitude = 0.0; 
let accel_long = 0.0; 
let lTimeIMU = null; // Timestamp local pour l'IMU (Correction ZVU)

// Simulation IPS
let currentIPSLat = 43.284573, currentIPSLon = 5.358806, currentIPSAlt = 66.10;

// Contrôles & État
let currentGPSMode = 'medium_accuracy';
let isGPSRunning = false, isGPSTracking = false, emergencyStopActive = false;
let isZVUActive = false, zvuLockTime = 0.0;
let netherMode = false, selectedEnvironment = 'Earth'; 
let lastFSpeed = 0.0; 


// -------------------------------------------
// 2. FONCTIONS UTILITAIRES (EKF, Gravité, Math)
// -------------------------------------------
function $(id) { return document.getElementById(id); }
function getCDate() { return new Date(); }
function formatTime(totalMinutes) {
    const minutes = Math.round(totalMinutes % 60);
    const hours = Math.floor(totalMinutes / 60) % 24;
    return `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}`;
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
// 3. FONCTION DE SIMULATION IPS
// -------------------------------------------

function getIPSPositionSimulation(dt) {
    const spd = kSpd * dt;
    const headingRad = currentHeading * D2R;
    currentIPSLat += (spd * Math.cos(headingRad) * (Math.random() * 0.05 + 0.95)) / (EARTH_RADIUS * D2R);
    currentIPSLon += (spd * Math.sin(headingRad) * (Math.random() * 0.05 + 0.95)) / (EARTH_RADIUS * D2R * Math.cos(currentIPSLat * D2R));
    const acc_sim = IPS_R_MIN + Math.abs(kSpd) * IPS_R_FACTOR; 
    const num_sat = Math.max(2, 10 - Math.floor(acc_sim));
    const pdop = Math.max(2.0, acc_sim * 0.8);

    if ($('num-satellites')) $('num-satellites').textContent = `${num_sat.toFixed(0)} (IPS Sim.)`;
    if ($('pdop-value')) $('pdop-value').textContent = `${pdop.toFixed(2)} (IPS Sim.)`;
    if ($('gps-precision')) $('gps-precision').textContent = `${acc_sim.toFixed(2)} m`;
    
    return {
        coords: {
            latitude: currentIPSLat, longitude: currentIPSLon,
            altitude: currentIPSAlt + Math.sin(Date.now() / 5000) * 0.5, 
            accuracy: acc_sim, speed: kSpd, speedAccuracy: acc_sim / 2
        },
        timestamp: new Date().getTime()
    };
}

// -------------------------------------------
// 4. FONCTIONS ASTRO
// -------------------------------------------
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
    if (typeof suncalc === 'undefined') return { TST: 0, LSM: 0, EOT: 0, L: 0 };
    
    const d = suncalc.toDays(date);
    const M = suncalc.solarMeanAnomaly(d);
    const L = suncalc.eclipticLongitude(M);

    const E = R2D * (M + 2 * Math.PI - L) / (2 * Math.PI) * 1440 / 360; 
    const UT = date.getUTCHours() + date.getUTCMinutes() / 60 + date.getUTCSeconds() / 3600;
    const LSM_hours = UT + lonA / 15;
    const LSM_mins = LSM_hours * 60;
    const TST_mins = LSM_mins + E;
    
    return { TST: TST_mins, LSM: LSM_mins, EOT: E, L: L };
}

function updateAstro(latA, lonA) {
    if (latA === 0.0 && lonA === 0.0 || typeof suncalc === 'undefined') return;
    
    const date = getCDate();
    const times = suncalc.getTimes(date, latA, lonA);
    const moon = suncalc.getMoonIllumination(date);
    const sunPos = suncalc.getPosition(date, latA, lonA);
    const solarTime = getSolarTime(date, latA, lonA);

    // Mise à jour TST (Système et Astro)
    if ($('time-minecraft')) $('time-minecraft').textContent = formatTime(solarTime.TST);
    if ($('tst')) $('tst').textContent = formatTime(solarTime.TST);
    
    if ($('moon-phase-display')) $('moon-phase-display').textContent = getMoonPhaseName(moon.fraction);
    if ($('sun-elevation')) $('sun-elevation').textContent = `${(sunPos.altitude * R2D).toFixed(2)} ° (Sim.)`;
    if ($('zenith-angle')) $('zenith-angle').textContent = `${(90 - sunPos.altitude * R2D).toFixed(2)} ° (Sim.)`;
    if ($('lsm')) $('lsm').textContent = formatTime(solarTime.LSM);
    if ($('sunrise-time')) $('sunrise-time').textContent = times.sunrise.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit', second: '2-digit', timeZone: 'UTC' }) + ' (Sim.)';
    if ($('sunset-time')) $('sunset-time').textContent = times.sunset.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit', second: '2-digit', timeZone: 'UTC' }) + ' (Sim.)';
    if ($('tst-display')) $('tst-display').textContent = formatTime(solarTime.TST);
}

// -------------------------------------------
// 5. FONCTIONS MÉTÉO (Simulation)
// -------------------------------------------

async function updateWeather(latA, lonA) {
    if (latA === 0.0 && lonA === 0.0) return;
    
    const temp = 15 + Math.sin(Date.now() / 100000) * 10;
    const pressure = 1013.25 + Math.cos(Date.now() / 50000) * 10;
    const humidity = 50 + Math.sin(Date.now() / 70000) * 20;

    if ($('air-temp')) $('air-temp').textContent = `${temp.toFixed(1)} °C (Sim.)`;
    if ($('air-pressure')) $('air-pressure').textContent = `${pressure.toFixed(2)} hPa (Sim.)`;
    if ($('air-humidity')) $('air-humidity').textContent = `${humidity.toFixed(1)} % (Sim.)`;
    if ($('wind-speed-ms')) $('wind-speed-ms').textContent = `${(5 + Math.random() * 5).toFixed(1)} m/s (Sim.)`;
}

// -------------------------------------------
// 6. FONCTIONS CAPTEURS INERTIELS (IMU, Mag) (CORRECTION INCLINAISON)
// -------------------------------------------

function handleDeviceMotion(event) {
    if (emergencyStopActive) return;
    
    const acc_g_raw = event.accelerationIncludingGravity;
    const timestamp = event.timeStamp;
    // CORRECTION ZVU: Utilise le timestamp IMU pour le dt
    const dt_imu = lTimeIMU ? (timestamp - lTimeIMU) / 1000 : 0.05;
    lTimeIMU = timestamp; 

    if (acc_g_raw && acc_g_raw.x !== null) {
        
        // 1. ÉTALONNAGE (Soustraction des Biais)
        const acc_g_calibrated_x = acc_g_raw.x - ACCEL_BIAS_X;
        const acc_g_calibrated_y = acc_g_raw.y - ACCEL_BIAS_Y;
        const acc_g_calibrated_z = acc_g_raw.z - ACCEL_BIAS_Z;

        // 2. LISSAGE
        kAccel.x = ACCEL_FILTER_ALPHA * kAccel.x + (1 - ACCEL_FILTER_ALPHA) * acc_g_calibrated_x;
        kAccel.y = ACCEL_FILTER_ALPHA * kAccel.y + (1 - ACCEL_FILTER_ALPHA) * acc_g_calibrated_y;
        kAccel.z = ACCEL_FILTER_ALPHA * kAccel.z + (1 - ACCEL_FILTER_ALPHA) * acc_g_calibrated_z;
        
        // 3. TRANSFORMATION DE REPÈRE (CORRECTION DE L'INCLINAISON)
        
        // Angles en radians
        const phi = global_roll; // Roulis (Roll)
        const theta = global_pitch; // Tangage (Pitch)

        const g_local = calculateGravityAtAltitude(kAlt);

        // Composantes de G dans le repère de l'appareil (t)
        const g_proj_x = -g_local * Math.sin(theta); 
        const g_proj_y = g_local * Math.sin(phi) * Math.cos(theta); 
        const g_proj_z = g_local * Math.cos(phi) * Math.cos(theta); 

        // Accélération linéaire réelle dans le repère de l'appareil (t)
        const acc_lin_t_x = kAccel.x - g_proj_x;
        const acc_lin_t_y = kAccel.y - g_proj_y;
        const acc_lin_t_z = kAccel.z - g_proj_z;

        // VRAI ACCÉLÉRATION LINÉAIRE 3D MAGNITUDE
        latestLinearAccelMagnitude = Math.sqrt(
            acc_lin_t_x ** 2 + acc_lin_t_y ** 2 + acc_lin_t_z ** 2
        );
        
        // ACCÉLÉRATION VERTICALE
        latestVerticalAccelIMU = acc_lin_t_z;
        
    } else { return; }
    
    // 4. ZVU (Zero Velocity Update) - Basé sur la magnitude linéaire 3D
    if (latestLinearAccelMagnitude < STATIC_ACCEL_THRESHOLD) { 
        latestLinearAccelMagnitude = 0.0;
        latestVerticalAccelIMU = 0.0;
        isZVUActive = true;
        zvuLockTime += dt_imu; 
    } else {
        isZVUActive = false;
        zvuLockTime = 0; 
    }

    // Affichage des données IMU
    if ($('accel-vertical-imu')) $('accel-vertical-imu').textContent = `${latestVerticalAccelIMU.toFixed(3)} m/s²`;
    if ($('accel-3d-imu')) $('accel-3d-imu').textContent = `${latestLinearAccelMagnitude.toFixed(3)} m/s²`;
    if ($('force-g-3d-imu')) $('force-g-3d-imu').textContent = `${(latestLinearAccelMagnitude / G_ACC).toFixed(3)} G`;
    if ($('zvu-lock-status')) $('zvu-lock-status').textContent = isZVUActive ? 'VERROUILLÉ 🟢' : 'NON-VERROUILLÉ 🔴';
    if ($('zvu-lock-time')) $('zvu-lock-time').textContent = `${zvuLockTime.toFixed(1)} s`;
}

function handleDeviceOrientation(event) {
    if (emergencyStopActive) return;
    if (event.alpha !== null) {
        global_pitch = event.beta * D2R; 
        global_roll = event.gamma * D2R;
    }
}

function handleMagnetometer(event) {
    if (emergencyStopActive) return;
    
    if (event.absolute && event.alpha !== null) {
        currentHeading = event.alpha; 
    } else if (event.alpha !== null) {
        currentHeading = event.alpha;
    }

    latestMagneticFieldMagnitude = 50 + (Math.sin(Date.now() / 5000) * 5) + Math.random() * 2; 
    
    if ($('magnetic-field')) {
        $('magnetic-field').textContent = `${latestMagneticFieldMagnitude.toFixed(2)} µT`;
    }
    
    updateCompasses(lat, lon); 
}

function calculateBearing(lat1, lon1, lat2, lon2) {
    const phi1 = lat1 * D2R;
    const phi2 = lat2 * D2R;
    const lambda1 = lon1 * D2R;
    const lambda2 = lon2 * D2R;
    const y = Math.sin(lambda2 - lambda1) * Math.cos(phi2);
    const x = Math.cos(phi1) * Math.sin(phi2) - Math.sin(phi1) * Math.cos(phi2) * Math.cos(lambda2 - lambda1);
    const bearingRad = Math.atan2(y, x);
    return (bearingRad * R2D + 360) % 360; 
}

function updateCompasses(latA, lonA) {
    if ($('compass-heading')) {
        $('compass-heading').textContent = `${currentHeading.toFixed(1)} °`;
        const needle = $('compass-needle');
        if (needle) { needle.style.transform = `translateX(-50%) rotate(${-currentHeading}deg)`; }
    }
    
    const TARGET_LAT = 48.8584; 
    const TARGET_LON = 2.2945; 

    if (latA !== 0 || lonA !== 0) {
        const targetBearing = calculateBearing(latA, lonA, TARGET_LAT, TARGET_LON);
        const distanceKm = calculateDistanceHaversine(latA, lonA, TARGET_LAT, TARGET_LON) / 1000;
        
        if ($('target-bearing')) $('target-bearing').textContent = `${targetBearing.toFixed(1)} °`;
        if ($('target-distance')) $('target-distance').textContent = `${distanceKm.toFixed(3)} km`;
        
        const relativeBearing = (targetBearing - currentHeading + 360) % 360;
        const targetNeedle = $('target-needle');
        if (targetNeedle) {
            targetNeedle.style.transform = `rotate(${relativeBearing}deg)`;
        }
    }
}


// -------------------------------------------
// 7. FONCTION PRINCIPALE DE MISE À JOUR (IPS/EKF) (Vitesse Max/Moyenne/Distance sur Vitesse Instantanée)
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
        spd3D_raw = d_moved_3D / dt; // Vitesse 3D Instantanée
    }

    let accel_control_3D = latestLinearAccelMagnitude;
    let accel_control_V = latestVerticalAccelIMU;
    
    // Vitesse de base pour les métriques de session (Max/Moyenne/Distance)
    const speed_base_for_metrics = (isZVUActive) ? 0.0 : spd3D_raw;
    
    if (isZVUActive) { accel_control_3D = 0.0; accel_control_V = 0.0; } // L'EKF est corrigé par le ZVU

    let R_dyn = getKalmanR(acc); 
    const sSpdFE = kFilter(spd3D_raw, dt, R_dyn, accel_control_3D); // L'EKF filtre toujours spd3D_raw
    const kAlt_new = kFilterAltitude(alt_ips_raw, acc, dt, accel_control_V);
    
    // ********************************************************
    // CALCUL DES MÉTRIQUES BASÉ SUR LA VITESSE 3D INSTANTANÉE
    // ********************************************************
    if (lPos) { 
        // 1. Distance Totale
        distM += d_moved_3D;

        // 2. Temps de Mouvement
        if (speed_base_for_metrics > 0.5) { 
            timeMoving += dt; 
        }
    }
    
    // 3. Vitesse Max
    maxSpd = Math.max(maxSpd, speed_base_for_metrics); 

    // 4. Vitesse Moyenne
    let avgSpeed = 0.0;
    if (timeMoving > 5.0) { 
        avgSpeed = distM / timeMoving;
    }
    // ********************************************************

    accel_long = (dt > 0.05) ? (sSpdFE - lastFSpeed) / dt : 0;
    lastFSpeed = sSpdFE; 
    

    // --- MISE À JOUR DU DOM ---
    if ($('latitude')) $('latitude').textContent = `${lat.toFixed(6)} ° (IPS)`;
    if ($('longitude')) $('longitude').textContent = `${lon.toFixed(6)} ° (IPS)`;
    if ($('altitude-gps')) $('altitude-gps').textContent = `${alt_ips_raw.toFixed(2)} m (IPS)`;
    if ($('speed-stable')) $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(4)} km/h`;
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${sSpdFE.toFixed(3)} m/s`;
    
    // Affichage des métriques instantanées
    if ($('speed-max')) $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(4)} km/h`;
    if ($('speed-average')) $('speed-average').textContent = `${(avgSpeed * KMH_MS).toFixed(4)} km/h`; 
    if ($('distance-total-km')) $('distance-total-km').textContent = `${(distM / 1000).toFixed(3)} km`;
    
    if ($('accel-long')) $('accel-long').textContent = `${accel_long.toFixed(3)} m/s²`;
    if ($('force-g-long')) $('force-g-long').textContent = `${(accel_long / G_ACC).toFixed(3)} G`;
    if ($('time-elapsed')) $('time-elapsed').textContent = `${((new Date().getTime() - sTime) / 1000).toFixed(1)} s`;
    if ($('time-moving')) $('time-moving').textContent = `${timeMoving.toFixed(1)} s`;
    if ($('speed-3d-inst')) $('speed-3d-inst').textContent = `${(spd3D_raw * KMH_MS).toFixed(4)} km/h`;
    if ($('speed-raw-ms')) $('speed-raw-ms').textContent = `${spd_raw_ips.toFixed(3)} m/s (IPS)`;
    if ($('perc-speed-sound')) $('perc-speed-sound').textContent = `${(sSpdFE / SPEED_SOUND * 100).toFixed(2)} %`;
    if ($('perc-speed-c')) $('perc-speed-c').textContent = `${(sSpdFE / C_L * 100).toPrecision(2)} %`;

    // --- MISE À JOUR VISUALISATIONS ET ASTRO ---
    if (typeof updateGlobe === 'function') updateGlobe(lat, lon, kAlt_new); 
    updateAstro(lat, lon);
    updateCompasses(lat, lon); 
    
    // --- SAUVEGARDE DES VALEURS ---
    lPos = pos;
    lPos.kAlt_old = kAlt_new;
    lPos.kSpd_old = sSpdFE; 
}
// =================================================================
// FICHIER JS PARTIE 2, BLOC B : gnss-dashboard-part2_blocB.js (Globe 3D, Contrôles & Initialisation)
// (Intègre corrections Date Locale, Météo/Astro Init.)
// =================================================================

// -------------------------------------------
// 1. DÉCLARATIONS GLOBALES (Three.js)
// -------------------------------------------
let scene, camera, renderer, controls;
let earthMesh;
let marker, speedVector, accelVector, trajectoryLine; 
let trajectoryPoints = []; 

// -------------------------------------------
// 2. FONCTIONS GLOBE 3D ET VISUALISATION (Three.js)
// -------------------------------------------

function initGlobe(latA, lonA, altA) {
    if (scene) return;
    
    const container = $('globe-container');
    if (!container) return;
    
    scene = new THREE.Scene();
    const width = container.clientWidth;
    const height = container.clientHeight;
    camera = new THREE.PerspectiveCamera(75, width / height, 0.1, 1000);
    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setSize(width, height);
    container.innerHTML = ''; 
    container.appendChild(renderer.domElement);
    
    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true; 
    
    const textureLoader = new THREE.TextureLoader();
    // Utilisation d'une texture de placeholder sécurisée si non disponible localement
    const earthTexture = textureLoader.load('https://threejs.org/examples/textures/land_ocean_ice_cloud_2048.jpg'); 
    const geometry = new THREE.SphereGeometry(R_GLOBE, 60, 60); 
    const material = new THREE.MeshPhongMaterial({ map: earthTexture, specular: 0x333333, shininess: 15 });
    earthMesh = new THREE.Mesh(geometry, material);
    scene.add(earthMesh);
    camera.position.set(2.5, 0.5, 2.5);
    scene.add(new THREE.AmbientLight(0x404040));
    scene.add(new THREE.DirectionalLight(0xffffff, 0.8));
    
    const markerGeometry = new THREE.SphereGeometry(0.02, 16, 16);
    const markerMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000 });
    marker = new THREE.Mesh(markerGeometry, markerMaterial);
    scene.add(marker);

    const direction = new THREE.Vector3(1, 0, 0); 
    speedVector = new THREE.ArrowHelper(direction, new THREE.Vector3(0, 0, 0), 0.5, 0x00ff00, 0.05, 0.025);
    scene.add(speedVector);
    accelVector = new THREE.ArrowHelper(direction, new THREE.Vector3(0, 0, 0), 0.5, 0xffa500, 0.05, 0.025);
    scene.add(accelVector);
    
    const materialLine = new THREE.LineBasicMaterial({ color: 0xffff00 });
    const geometryLine = new THREE.BufferGeometry();
    trajectoryLine = new THREE.Line(geometryLine, materialLine);
    scene.add(trajectoryLine);
    
    animateGlobe();
    if ($('globe-status-display')) $('globe-status-display').textContent = 'Globe 3D Three.js: Actif 🟢';
}

function animateGlobe() {
    requestAnimationFrame(animateGlobe);
    if(controls) controls.update(); 
    if(renderer && scene && camera) renderer.render(scene, camera);
}

function updateGlobe(latA, lonA, altA) {
    if (!scene) { initGlobe(latA, lonA, altA); return; }
    if (!marker || !speedVector || !accelVector || !trajectoryLine) return;

    const altitudeNormalized = kAlt / 100000; 
    const radius = R_GLOBE + altitudeNormalized;
    const phi = (90 - latA) * D2R; 
    const theta = (lonA + 180) * D2R; 
    
    marker.position.x = -(radius * Math.sin(phi) * Math.cos(theta));
    marker.position.z = (radius * Math.sin(phi) * Math.sin(theta));
    marker.position.y = (radius * Math.cos(phi));
    const currentPos = marker.position;
    
    if (!isZVUActive) {
        trajectoryPoints.push(currentPos.x, currentPos.y, currentPos.z);
    }
    while (trajectoryPoints.length > MAX_TRAJECTORY_POINTS * 3) {
        trajectoryPoints.shift(); trajectoryPoints.shift(); trajectoryPoints.shift();
    }
    trajectoryLine.geometry.setAttribute('position', new THREE.Float32BufferAttribute(trajectoryPoints, 3));
    trajectoryLine.geometry.attributes.position.needsUpdate = true;

    const currentSpeed = kSpd; 
    const currentAccel = Math.abs(accel_long); 
    
    const speedLength = Math.min(currentSpeed * 0.05, 2.0); 
    const headingRad = currentHeading * D2R; 
    const speedDir = new THREE.Vector3(-Math.sin(headingRad), 0, -Math.cos(headingRad));
    speedDir.normalize();

    speedVector.position.copy(currentPos);
    speedVector.setDirection(speedDir);
    speedVector.setLength(speedLength);
    
    const accelLength = Math.min(currentAccel / G_ACC * 0.5, 1.0);
    let accelDir = speedDir.clone();
    if (accel_long < 0) { accelDir.negate(); }
    
    accelVector.position.copy(currentPos);
    accelVector.setDirection(accelDir);
    accelVector.setLength(accelLength);
}

// -------------------------------------------
// 3. FONCTIONS DE CONTRÔLE ET GESTION IPS/IMU
// -------------------------------------------

function syncH() {
    const now = getCDate();
    if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString('fr-FR');
    // CORRECTION: Met à jour la date locale
    if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
}

function startGPS() { // Simule le bouton de démarrage de la fonction de positionnement
    if (wID === null) {
        // Démarre l'appel à updateDisp (qui utilise la simulation IPS/IMU)
        wID = setInterval(updateDisp, 200); // 5 Hz pour l'IPS/EKF
        isGPSTracking = true;
        if ($('toggle-gps-btn')) { $('toggle-gps-btn').textContent = "⏸️ PAUSE IPS/IMU"; $('toggle-gps-btn').style.backgroundColor = '#ffc107'; }
    }
}

function stopGPS() {
    if (wID !== null) {
        clearInterval(wID);
        wID = null;
        isGPSTracking = false;
        if ($('toggle-gps-btn')) { $('toggle-gps-btn').textContent = "▶️ MARCHE IPS/IMU"; $('toggle-gps-btn').style.backgroundColor = '#28a745'; }
    }
}

function toggleGPS() {
    isGPSTracking ? stopGPS() : startGPS();
}

function handleErr(err) { 
    console.warn('Note: La fonction handleErr (GPS) est maintenant désactivée car le système utilise l\'IPS simulé.');
}

function emergencyStop() {
    emergencyStopActive = true;
    stopGPS();
    window.removeEventListener('devicemotion', handleDeviceMotion, true);
    window.removeEventListener('deviceorientation', handleDeviceOrientation, true);
    window.removeEventListener('deviceorientationabsolute', handleMagnetometer, true);
    if ($('emergency-stop-btn')) { $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: ACTIF 🔴"; $('emergency-stop-btn').style.backgroundColor = '#dc3545'; }
}

function resumeSystem() {
    emergencyStopActive = false;
    startGPS(); 
    window.addEventListener('devicemotion', handleDeviceMotion, true);
    window.addEventListener('deviceorientation', handleDeviceOrientation, true);
    if (window.DeviceOrientationAbsoluteEvent) {
        window.addEventListener('deviceorientationabsolute', handleMagnetometer, true);
    } else if (window.DeviceOrientationEvent) {
        window.addEventListener('deviceorientation', handleMagnetometer, true); 
    } 
    if ($('emergency-stop-btn')) { $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: INACTIF 🟢"; $('emergency-stop-btn').style.backgroundColor = '#28a745'; }
}

function resetDist() { distM = 0.0; if ($('distance-total-km')) $('distance-total-km').textContent = '0.000 km'; }
function resetMax() { maxSpd = 0.0; if ($('speed-max')) $('speed-max').textContent = '0.0000 km/h'; }

function resetTrajectory() {
    trajectoryPoints = [];
    if (trajectoryLine) {
        trajectoryLine.geometry.setAttribute('position', new THREE.Float32BufferAttribute(trajectoryPoints, 3));
        trajectoryLine.geometry.attributes.position.needsUpdate = true;
    }
}

function resetAll() {
    resetDist();
    resetMax();
    resetTrajectory();
    timeMoving = 0.0;
    zvuLockTime = 0.0;
    sTime = new Date().getTime(); 
    kSpd = 0.0; kAlt = 0.0; kUncert = 10.0; kAltUncert = 10.0;
    lPos = null;
    if ($('time-elapsed')) $('time-elapsed').textContent = '0.0 s';
    if ($('time-moving')) $('time-moving').textContent = '0.0 s';
}

function toggleMode() {
    document.body.classList.toggle('dark-mode');
}

function netherToggle() {
    netherMode = !netherMode;
    if ($('mode-nether')) $('mode-nether').textContent = netherMode ? 'ACTIVÉ (1:8) 🟠' : 'DÉSACTIVÉ (1:1)';
}

function captureData() {
    alert('Capture de données simulée. Une implémentation complète nécessiterait une fonction de journalisation.');
}

// -------------------------------------------
// 4. INITIALISATION DES ÉVÉNEMENTS
// -------------------------------------------

document.addEventListener('DOMContentLoaded', () => {
    
    syncH(); // Appel initial pour l'heure et la date
    
    // --- Configuration des contrôles ---
    $('toggle-gps-btn')?.addEventListener('click', toggleGPS);
    $('freq-select')?.addEventListener('change', (e) => { 
        currentGPSMode = e.target.value; 
    });
    $('emergency-stop-btn')?.addEventListener('click', () => { emergencyStopActive ? resumeSystem() : emergencyStop(); });
    $('reset-dist-btn')?.addEventListener('click', resetDist);
    $('reset-max-btn')?.addEventListener('click', resetMax);
    $('reset-all-btn')?.addEventListener('click', resetAll);
    $('reset-trajectory-btn')?.addEventListener('click', resetTrajectory); 
    $('toggle-mode-btn')?.addEventListener('click', toggleMode);
    $('nether-toggle-btn')?.addEventListener('click', netherToggle);
    $('data-capture-btn')?.addEventListener('click', captureData);
    $('environment-select')?.addEventListener('change', (e) => { selectedEnvironment = e.target.value; });

    // --- Démarrage des Capteurs ---
    resumeSystem(); // Lance les listeners IMU/Orientation et startGPS (IPS/EKF)

    // CORRECTION: Appel initial pour la météo et l'astro (pour éviter N/A au démarrage)
    updateWeather(currentIPSLat, currentIPSLon);
    updateAstro(currentIPSLat, currentIPSLon);

    domID = setInterval(syncH, 1000);
    weatherID = setInterval(() => { if (lat !== 0 || lon !== 0) updateWeather(lat, lon); }, 30000); 
    
    // Redimensionnement et Initialisation du Globe
    window.addEventListener('resize', () => {
        const container = $('globe-container');
        if (camera && renderer && container) {
            camera.aspect = container.clientWidth / container.clientHeight;
            camera.updateProjectionMatrix();
            renderer.setSize(container.clientWidth, container.clientHeight);
        }
    });

    setTimeout(() => {
        if (!scene) {
            initGlobe(lat, lon, kAlt);
        }
    }, 500);
});
