// =================================================================
// FICHIER JS PARTIE 1 : gnss-dashboard-part1.js (Constantes)
// =================================================================

// -------------------------------------------
// Constantes Mathématiques & Unités
// -------------------------------------------
const D2R = Math.PI / 180; // Conversion Degrés vers Radians
const R2D = 180 / Math.PI; // Conversion Radians vers Degrés

// -------------------------------------------
// Gravité et Modèle Terrestre (pour correction EKF/IMU)
// -------------------------------------------
const G_ACC = 9.80665; // Accélération standard de la gravité au niveau de la mer (m/s²)
const EARTH_RADIUS = 6371000; // Rayon moyen de la Terre (m). Utilisé pour la correction de gravité dynamique en fonction de l'altitude.

// -------------------------------------------
// Constantes Physiques Générales
// -------------------------------------------
const KMH_MS = 3.6; // Facteur de conversion de m/s en km/h
const SPEED_SOUND = 343; // Vitesse du son dans l'air sec à 20°C (m/s)
const C_L = 299792458; // Vitesse de la lumière dans le vide (m/s)

// -------------------------------------------
// Paramètres de Filtrage EKF / IMU
// -------------------------------------------
const ACCEL_FILTER_ALPHA = 0.8; // Alpha pour le filtre passe-bas sur l'IMU (lissage des accélérations brutes)
const STATIC_ACCEL_THRESHOLD = 0.5; // Seuil d'accélération pour l'activation du ZVU (Zero Velocity Update) (m/s²)

// -------------------------------------------
// Paramètres de Visualisation 3D (Three.js)
// -------------------------------------------
const R_GLOBE = 2.0; // Rayon de la sphère représentant la Terre dans la scène 3D
const MAX_TRAJECTORY_POINTS = 500; // Nombre maximum de points à afficher pour le tracé de la trajectoire
// =================================================================
// FICHIER JS PARTIE 2, BLOC A : gnss-dashboard-part2_blocA.js (Variables, EKF, Capteurs & updateDisp)
// =================================================================

// -------------------------------------------
// 1. VARIABLES GLOBALES
// -------------------------------------------

let wID = null; 
let domID = null; 
let weatherID = null; 
let lat = 0.0;
let lon = 0.0;
let kSpd = 0.0; // Vitesse EKF stable (m/s)
let kAlt = 0.0; // Altitude EKF stable (m)
let kUncert = 10.0; // Incertitude EKF vitesse (m²/s²)
let kAltUncert = 10.0; // Incertitude EKF altitude (m²)
let lPos = null; 
let distM = 0.0; 
let maxSpd = 0.0;
let timeMoving = 0.0;
let sTime = new Date().getTime(); 

// IMU/Orientation
let kAccel = { x: 0, y: 0, z: 0 }; 
let global_pitch = 0; 
let global_roll = 0; 
let currentHeading = 0; 
let latestLinearAccelMagnitude = 0.0; 
let latestVerticalAccelIMU = 0.0; 
let latestMagneticFieldMagnitude = 0.0; 
let accel_long = 0.0; 

// Contrôles & État
let currentGPSMode = 'medium_accuracy';
let isGPSRunning = false;
let isGPSTracking = false;
let emergencyStopActive = false;
let isZVUActive = false;
let zvuLockTime = 0;
let netherMode = false;
let selectedEnvironment = 'Earth'; 
let lastFSpeed = 0.0; 


// -------------------------------------------
// 2. FONCTIONS UTILITAIRES (EKF et Mathématiques)
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

// Fonction de Gravité Dynamique
function calculateGravityAtAltitude(altitudeMeters) {
    if (altitudeMeters < 0) altitudeMeters = 0;
    // Utilise G_ACC et EARTH_RADIUS de gnss-dashboard-part1.js
    return G_ACC * Math.pow(EARTH_RADIUS / (EARTH_RADIUS + altitudeMeters), 2);
}

// Fonctions EKF
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
// 3. FONCTIONS ASTRO
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

function updateAstro(latA, lonA) {
    if (latA === 0.0 && lonA === 0.0) return;
    
    const date = getCDate();
    const times = suncalc.getTimes(date, latA, lonA);
    const moon = suncalc.getMoonIllumination(date);
    const sunPos = suncalc.getPosition(date, latA, lonA);
    const solarTime = getSolarTime(date, latA, lonA);

    if ($('local-date')) $('local-date').textContent = date.toLocaleDateString('fr-FR');
    if ($('tst')) $('tst').textContent = formatTime(solarTime.TST);
    if ($('moon-phase-display')) $('moon-phase-display').textContent = getMoonPhaseName(moon.fraction);
    if ($('sun-elevation')) $('sun-elevation').textContent = `${(sunPos.altitude * R2D).toFixed(2)} °`;
    if ($('zenith-angle')) $('zenith-angle').textContent = `${(90 - sunPos.altitude * R2D).toFixed(2)} °`;
    if ($('lsm')) $('lsm').textContent = formatTime(solarTime.LSM);
    if ($('sunrise-time')) $('sunrise-time').textContent = times.sunrise.toLocaleTimeString('fr-FR');
    if ($('sunset-time')) $('sunset-time').textContent = times.sunset.toLocaleTimeString('fr-FR');
    if ($('tst-display')) $('tst-display').textContent = formatTime(solarTime.TST);
}

// -------------------------------------------
// 4. FONCTIONS MÉTÉO (Simulation)
// -------------------------------------------

async function updateWeather(latA, lonA) {
    if (latA === 0.0 && lonA === 0.0) return;
    
    // Simulations basiques
    const temp = 15 + Math.sin(Date.now() / 100000) * 10;
    const pressure = 1013.25 + Math.cos(Date.now() / 50000) * 10;
    const humidity = 50 + Math.sin(Date.now() / 70000) * 20;

    if ($('air-temp')) $('air-temp').textContent = `${temp.toFixed(1)} °C (Sim.)`;
    if ($('air-pressure')) $('air-pressure').textContent = `${pressure.toFixed(2)} hPa (Sim.)`;
    if ($('air-humidity')) $('air-humidity').textContent = `${humidity.toFixed(1)} % (Sim.)`;
    if ($('wind-speed-ms')) $('wind-speed-ms').textContent = `${(5 + Math.random() * 5).toFixed(1)} m/s (Sim.)`;
}

// -------------------------------------------
// 5. FONCTIONS CAPTEURS INERTIELS (IMU, Mag)
// -------------------------------------------

function handleDeviceMotion(event) {
    if (emergencyStopActive) return;
    
    let acc_x_lin, acc_y_lin, acc_z_lin;
    const acc_g = event.accelerationIncludingGravity;
    const timestamp = event.timeStamp;
    const dt_imu = lPos ? (timestamp - lPos.timestamp) / 1000 : 0.05;

    // Calcul de la gravité locale avec l'altitude EKF stable (kAlt)
    const g_local = calculateGravityAtAltitude(kAlt);
    
    if (acc_g && acc_g.x !== null) {
        // Lissage des accélérations brutes
        kAccel.x = ACCEL_FILTER_ALPHA * kAccel.x + (1 - ACCEL_FILTER_ALPHA) * acc_g.x;
        kAccel.y = ACCEL_FILTER_ALPHA * kAccel.y + (1 - ACCEL_FILTER_ALPHA) * acc_g.y;
        kAccel.z = ACCEL_FILTER_ALPHA * kAccel.z + (1 - ACCEL_FILTER_ALPHA) * acc_g.z;
        
        // Soustraction de la gravité basée sur l'orientation (utilise g_local)
        const G_CORR_X = -g_local * Math.sin(global_pitch); 
        const G_CORR_Y = g_local * Math.sin(global_roll) * Math.cos(global_pitch); 
        const G_CORR_Z = g_local * Math.cos(global_roll) * Math.cos(global_pitch); 
        
        acc_x_lin = kAccel.x - G_CORR_X;
        acc_y_lin = kAccel.y - G_CORR_Y;
        acc_z_lin = kAccel.z - G_CORR_Z; 
    } else { return; }

    const magnitude_lin = Math.sqrt(acc_x_lin ** 2 + acc_y_lin ** 2 + acc_z_lin ** 2);
    let accel_vertical_lin_raw = acc_z_lin; 
    
    // ZVU (Zero Velocity Update) 
    if (magnitude_lin < STATIC_ACCEL_THRESHOLD) { 
        latestLinearAccelMagnitude = 0.0;
        latestVerticalAccelIMU = 0.0;
        isZVUActive = true;
        zvuLockTime += dt_imu;
    } else {
        latestLinearAccelMagnitude = magnitude_lin;
        latestVerticalAccelIMU = accel_vertical_lin_raw;
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
    
    // Mise à jour de l'affichage boussole
    updateCompasses(lat, lon); 
}

function calculateBearing(lat1, lon1, lat2, lon2) {
    // Calcule l'azimut (cap) entre deux points
    const phi1 = lat1 * D2R;
    const phi2 = lat2 * D2R;
    const lambda1 = lon1 * D2R;
    const lambda2 = lon2 * D2R;

    const y = Math.sin(lambda2 - lambda1) * Math.cos(phi2);
    const x = Math.cos(phi1) * Math.sin(phi2) - 
              Math.sin(phi1) * Math.cos(phi2) * Math.cos(lambda2 - lambda1);
    
    const bearingRad = Math.atan2(y, x);
    return (bearingRad * R2D + 360) % 360; 
}

function updateCompasses(latA, lonA) {
    if ($('compass-heading')) {
        $('compass-heading').textContent = `${currentHeading.toFixed(1)} °`;
        const needle = $('compass-needle');
        if (needle) { needle.style.transform = `translateX(-50%) rotate(${-currentHeading}deg)`; }
    }
    
    // Cible : Tour Eiffel (Exemple)
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
            // Rotation pour la boussole de la cible
            targetNeedle.style.transform = `rotate(${relativeBearing}deg)`;
        }
    }
}


// -------------------------------------------
// 6. FONCTION PRINCIPALE DE MISE À JOUR (GPS/EKF)
// -------------------------------------------

function updateDisp(pos) {
    if (emergencyStopActive) return;
    
    const cTimePos = pos.timestamp; 
    lat = pos.coords.latitude; 
    lon = pos.coords.longitude;
    const alt_gps_raw = pos.coords.altitude ?? 0.0; 
    const acc = pos.coords.accuracy ?? 10.0;
    const spd_raw_gps = pos.coords.speed ?? 0.0;

    const dt = lPos ? (cTimePos - lPos.timestamp) / 1000 : 0.2;
    if (lPos === null) { sTime = new Date().getTime(); }

    let spd3D_raw = spd_raw_gps;
    if (lPos && dt > 0.0) {
        const d_horizontal = calculateDistanceHaversine(lPos.coords.latitude, lPos.coords.longitude, lat, lon);
        const d_vertical = Math.abs(alt_gps_raw - (lPos.coords.altitude ?? 0.0));
        const d_3D = Math.sqrt(d_horizontal ** 2 + d_vertical ** 2);
        if (pos.coords.speedAccuracy === undefined || pos.coords.speedAccuracy > acc * 2) {
             spd3D_raw = d_3D / dt;
        }
    }

    // EKF
    let accel_control_3D = latestLinearAccelMagnitude;
    let accel_control_V = latestVerticalAccelIMU;
    if (isZVUActive) { accel_control_3D = 0.0; accel_control_V = 0.0; spd3D_raw = 0.0; } 

    let R_dyn = getKalmanR(acc); 
    const sSpdFE = kFilter(spd3D_raw, dt, R_dyn, accel_control_3D); 
    const kAlt_new = kFilterAltitude(alt_gps_raw, acc, dt, accel_control_V);
    
    // Distance et Vitesse Max
    if (lPos && !isZVUActive) { 
        const d_moved = calculateDistanceHaversine(lPos.coords.latitude, lPos.coords.longitude, lat, lon);
        distM += d_moved;
        if (sSpdFE > 0.5) { timeMoving += dt; }
    }

    accel_long = (dt > 0.05) ? (sSpdFE - lastFSpeed) / dt : 0;
    lastFSpeed = sSpdFE; 
    maxSpd = Math.max(maxSpd, sSpdFE);

    // --- MISE À JOUR DU DOM ---
    if ($('latitude')) $('latitude').textContent = `${lat.toFixed(6)} °`;
    if ($('longitude')) $('longitude').textContent = `${lon.toFixed(6)} °`;
    if ($('altitude-gps')) $('altitude-gps').textContent = `${alt_gps_raw.toFixed(2)} m`;
    if ($('gps-precision')) $('gps-precision').textContent = `${acc.toFixed(2)} m`;
    if ($('speed-stable')) $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(4)} km/h`;
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${sSpdFE.toFixed(3)} m/s`;
    if ($('speed-max')) $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(4)} km/h`;
    if ($('accel-long')) $('accel-long').textContent = `${accel_long.toFixed(3)} m/s²`;
    if ($('force-g-long')) $('force-g-long').textContent = `${(accel_long / G_ACC).toFixed(3)} G`;
    if ($('distance-total-km')) $('distance-total-km').textContent = `${(distM / 1000).toFixed(3)} km`;
    if ($('time-elapsed')) $('time-elapsed').textContent = `${((new Date().getTime() - sTime) / 1000).toFixed(1)} s`;
    if ($('time-moving')) $('time-moving').textContent = `${timeMoving.toFixed(1)} s`;
    if ($('speed-3d-inst')) $('speed-3d-inst').textContent = `${(spd3D_raw * KMH_MS).toFixed(4)} km/h`;
    if ($('speed-raw-ms')) $('speed-raw-ms').textContent = `${spd_raw_gps.toFixed(3)} m/s`;
    if ($('num-satellites')) $('num-satellites').textContent = `${Math.floor(Math.max(4, 12 - (acc / 10)))} (Sim.)`; // Simulé
    if ($('pdop-value')) $('pdop-value').textContent = `${Math.max(1.5, Math.sqrt(acc) * 0.5).toFixed(2)} (Sim.)`; // Simulé
    if ($('perc-speed-sound')) $('perc-speed-sound').textContent = `${(sSpdFE / SPEED_SOUND * 100).toFixed(2)} %`;
    if ($('perc-speed-c')) $('perc-speed-c').textContent = `${(sSpdFE / C_L * 100).toPrecision(2)} %`;

    // --- MISE À JOUR VISUALISATIONS ET ASTRO ---
    if (typeof updateGlobe === 'function') updateGlobe(lat, lon, kAlt_new); 
    updateAstro(lat, lon);
    
    // --- SAUVEGARDE DES VALEURS ---
    lPos = pos;
    lPos.kAlt_old = kAlt_new;
    lPos.kSpd_old = sSpdFE; 
}
// =================================================================
// FICHIER JS PARTIE 2, BLOC B : gnss-dashboard-part2_blocB.js (Globe 3D, Contrôles & Initialisation)
// =================================================================

// -------------------------------------------
// 1. DÉCLARATIONS GLOBALES (Three.js)
// -------------------------------------------
let scene, camera, renderer, controls;
let earthMesh;
// Les variables marker, speedVector, etc. sont déclarées dans le Bloc A mais sont utilisées ici
// let marker, speedVector, accelVector, trajectoryLine; 
// let trajectoryPoints = []; 

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

    // 1. Position du Marqueur
    const altitudeNormalized = kAlt / 100000; 
    const radius = R_GLOBE + altitudeNormalized;
    const phi = (90 - latA) * D2R; 
    const theta = (lonA + 180) * D2R; 
    
    marker.position.x = -(radius * Math.sin(phi) * Math.cos(theta));
    marker.position.z = (radius * Math.sin(phi) * Math.sin(theta));
    marker.position.y = (radius * Math.cos(phi));
    const currentPos = marker.position;
    
    // 2. Mise à jour du Tracé
    if (!isZVUActive) {
        trajectoryPoints.push(currentPos.x, currentPos.y, currentPos.z);
    }
    while (trajectoryPoints.length > MAX_TRAJECTORY_POINTS * 3) {
        trajectoryPoints.shift(); trajectoryPoints.shift(); trajectoryPoints.shift();
    }
    trajectoryLine.geometry.setAttribute('position', new THREE.Float32BufferAttribute(trajectoryPoints, 3));
    trajectoryLine.geometry.attributes.position.needsUpdate = true;

    // 3. Mise à jour des Vecteurs
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
// 3. FONCTIONS DE CONTRÔLE ET GESTION GPS
// -------------------------------------------

function syncH() {
    const now = getCDate();
    if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString('fr-FR');
}

function startGPS() {
    const GPS_OPTS = { 
        'low_accuracy': { enableHighAccuracy: false, maximumAge: 60000, timeout: 5000 },
        'medium_accuracy': { enableHighAccuracy: true, maximumAge: 10000, timeout: 5000 },
        'high_accuracy': { enableHighAccuracy: true, maximumAge: 1000, timeout: 3000 } 
    };
    if (wID === null) {
        // updateDisp est définie dans le Bloc A
        wID = navigator.geolocation.watchPosition(updateDisp, handleErr, GPS_OPTS[currentGPSMode]); 
        isGPSTracking = true;
        if ($('toggle-gps-btn')) { $('toggle-gps-btn').textContent = "⏸️ PAUSE GPS"; $('toggle-gps-btn').style.backgroundColor = '#ffc107'; }
    }
}

function stopGPS() {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
        isGPSTracking = false;
        if ($('toggle-gps-btn')) { $('toggle-gps-btn').textContent = "▶️ MARCHE GPS"; $('toggle-gps-btn').style.backgroundColor = '#28a745'; }
    }
}

function toggleGPS() {
    isGPSTracking ? stopGPS() : startGPS();
}

function handleErr(err) { 
    console.error('Erreur GPS:', err.code, err.message);
    if ($('globe-status-display')) $('globe-status-display').textContent = `Erreur GPS: ${err.message} (Code: ${err.code})`;
    if (err.code === 3) {
        stopGPS();
        setTimeout(startGPS, 1000);
    }
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
    // Les fonctions handleDeviceMotion, etc. sont définies dans le Bloc A
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
function resetMax() { maxSpd = 0.0; if ($('speed-max')) $('speed-max').textContent = '0.00000 km/h'; }

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
    if ($('time-elapsed')) $('time-elapsed').textContent = '0.00 s';
    if ($('time-moving')) $('time-moving').textContent = '0.00 s';
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
    
    syncH(); 
    
    // --- Configuration des contrôles ---
    $('toggle-gps-btn')?.addEventListener('click', toggleGPS);
    $('freq-select')?.addEventListener('change', (e) => { 
        currentGPSMode = e.target.value; 
        if (isGPSTracking) { stopGPS(); startGPS(); }
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
    resumeSystem(); // Lance les listeners IMU/Orientation

    // --- Démarrage du Système de Traçage et Intervalles ---
    startGPS(); // Lance la surveillance GPS
    domID = setInterval(syncH, 1000);
    // updateWeather est définie dans le Bloc A
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

    // Initialisation du Globe au démarrage (après un court délai)
    setTimeout(() => {
        if (!scene) {
            initGlobe(lat, lon, kAlt);
        }
    }, 500);
});
