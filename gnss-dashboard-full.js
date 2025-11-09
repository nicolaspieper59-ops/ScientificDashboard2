// =================================================================
// BLOC 1/2 : CŒUR DU SYSTÈME (EKF, VARIABLES D'ÉTAT, CAPTEURS)
// =================================================================

const $ = (id) => document.getElementById(id);

// --- PARTIE 1 : CONSTANTES GLOBALES ET PHYSIQUES ---
const C_L = 299792458; 
const SPEED_SOUND = 343; 
const G_ACC_STD = 9.80665; 
const R2D = 180 / Math.PI;
const D2R = Math.PI / 180;

// Minecraft/Temps
const NETHER_RATIO = 1 / 8; 
const MC_DAY_MS = 1200000; 
const REAL_DAY_MS = 86400000; 
const MC_START_OFFSET_MS = 6 * 3600 * 1000; 

// Constantes GPS et EKF
const KMH_MS = 3.6; 
const MIN_DT = 0.05; 
const Q_NOISE_MIN_BASE = 0.0001; // Bruit minimal de base
const Q_NOISE_MAX = 0.0005; // Bruit maximal (Faible confiance)
let Q_NOISE = Q_NOISE_MAX; 
const MIN_SPD = 0.001; 
const MIN_UNCERT_FLOOR = Q_NOISE_MIN_BASE * MIN_DT; 
const ALT_TH = -50; 
const R_E = 6371000; 
const G_CONST = 6.67430e-11; 
const M_EARTH = 5.972e24; 

const MAX_GPS_ACCURACY_FOR_USE = 50.0; 
const KUNCERT_MAX = 0.05; 
const KUNCERT_FACTOR_MIN = 0.85; // Amortissement minimal appliqué (Réactivité EKF)
const SMOOTHING_TIME_CONSTANT = 0.1; // Lissage réduit (Réactivité d'affichage)

// Endpoints
const OWM_API_KEY = "VOTRE_CLE_API_OPENWEATHERMAP"; 
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc"; 

// --- PARTIE 2 : VARIABLES D'ÉTAT ---
let lat = 0, lon = 0, alt = 0;
let kSpd = 0, kUncert = 0.01; 
let lastTS = 0, lastFSpeed = 0, distM = 0;
let lastPos = null, lastAlt = 0; 
let sTime = null, timeMoving = 0, maxSpd = 0; 
let maxGForce = 0;
let maxGForceLat = 0;
let wID = null, domID = null, weatherID = null;

let currentGPSMode = 'HIGH_FREQ'; 
let emergencyStopActive = false;
let netherMode = false;
let G_ACC_LOCAL = G_ACC_STD; 
const P_RECORDS_KEY = 'gnss_precision_records'; 
let serverOffset = 0; 

let latestIMULinearAccel = 0; 
let lastHeading = 0; 
let imuHeading = 0; 
let imuPitch = 0; 
let imuRoll = 0; 
let verticalSpeedRaw = 0; 
let smoothedSpeed = 0; 

// --- PARTIE 3 : FONCTIONS UTILITAIRES ET PERSISTANCE ---

function toReadableScientific(num) {
    if (num === 0 || isNaN(num) || Math.abs(num) < 1e-6) return "0.00e+00";
    const exponent = Math.floor(Math.log10(Math.abs(num)));
    const mantissa = num / Math.pow(10, exponent);
    return `${mantissa.toFixed(2)}e+${exponent > 0 ? '+' : ''}${exponent}`;
}

function getCDate() {
    return new Date(Date.now() + serverOffset);
}

function calculateLocalGravity(altitude) {
    if (altitude === null || isNaN(altitude)) {
        G_ACC_LOCAL = G_ACC_STD; 
        return G_ACC_STD; 
    }
    const g_local = G_CONST * M_EARTH / Math.pow(R_E + altitude, 2);
    G_ACC_LOCAL = g_local; 
    return g_local;
}

/**
 * Retourne le facteur de pénalité de la covariance de mesure R basé sur l'environnement ET la fréquence GPS.
 */
function getKalmanRFactor() {
    const env = $('environment-select').value;
    let rFactor = 1.0;

    // Pénalité basée sur l'environnement
    switch (env) {
        case 'FOREST': rFactor = 1.5; break; 
        case 'METAL': rFactor = 3.0; break; 
        case 'CONCRETE': rFactor = 2.0; break; 
        default: rFactor = 1.0; 
    }

    // AJUSTEMENT CRITIQUE POUR LA HAUTE FRÉQUENCE
    if (currentGPSMode === 'HIGH_FREQ') {
        // Augmente R, disant à l'EKF : "Fais moins confiance à cette mesure GPS rapide, elle est probablement bruité."
        rFactor *= 1.2; 
    }
    
    return rFactor;
}


/**
 * Retourne un facteur de modification pour le Bruit du Modèle EKF (Q) basé sur l'environnement.
 */
function getKalmanQNoiseFactor() {
    const env = $('environment-select').value;
    switch (env) {
        case 'OPEN': return 0.7; // Moins de bruit IMU supposé
        case 'FOREST': return 1.0; 
        case 'CONCRETE': return 1.2; 
        case 'METAL': return 1.5; 
        default: return 1.0; 
    }
}


function loadPrecisionRecords() {
    try {
        const stored = localStorage.getItem(P_RECORDS_KEY);
        if (stored) {
            const loaded = JSON.parse(stored);
            maxGForce = loaded.max_g_force_max || 0;
            maxGForceLat = loaded.max_g_force_lat || 0; 
            const forceEl = $('force-g-long');
            if (forceEl) forceEl.textContent = `0.00 G | Max: ${maxGForce.toFixed(2)} G`;
        }
    } catch (e) { console.error("Erreur de chargement des records:", e); }
}

function savePrecisionRecords() {
    try {
        const records = { max_g_force_max: maxGForce, max_g_force_lat: maxGForceLat };
        localStorage.setItem(P_RECORDS_KEY, JSON.stringify(records));
    } catch (e) { console.error("Erreur de sauvegarde des records:", e); }
}

// --- PARTIE 4 : GESTIONNAIRE EKF ET GPS (Modèle IMU Pur) ---

/** Handler des données GPS et du filtre EKF. */
function updateDisp(pos) {
    if (emergencyStopActive) return;

    const acc = pos.coords.accuracy; 
    
    // --- MISE À JOUR DES POSITIONS ET TEMPS ---
    const gpsLat = pos.coords.latitude; 
    const gpsLon = pos.coords.longitude;
    const currentAlt = pos.coords.altitude; 
    
    const now = getCDate();
    
    // Initialisation au premier signal valide
    if (lat === 0 && lon === 0) {
        if (acc < MAX_GPS_ACCURACY_FOR_USE && gpsLat !== null && gpsLon !== null) {
            lat = gpsLat;
            lon = gpsLon;
            lastPos = { latitude: lat, longitude: lon };
            if (currentAlt !== null) lastAlt = currentAlt;
            lastTS = now.getTime();
            return; 
        } else {
            return; 
        }
    }
    
    if (lastTS === 0) lastTS = now.getTime(); 
    const dt = (now.getTime() - lastTS) / 1000;
    lastTS = now.getTime();
    
    if (dt < MIN_DT) return; 

    const g_dynamic = calculateLocalGravity(currentAlt);
    
    // --- CALCUL VITESSE VERTICALE (Altitude) ---
    if (currentAlt !== null && lastAlt !== 0 && dt > 0) {
        verticalSpeedRaw = (currentAlt - lastAlt) / dt;
    } else {
        verticalSpeedRaw = 0;
    }
    
    // --- GESTION DYNAMIQUE Q_NOISE ET NETHER ---
    const accuracyPenaltyFactor = Math.min(1.0, pos.coords.accuracy / MAX_GPS_ACCURACY_FOR_USE);
    const envQFactor = getKalmanQNoiseFactor();
    
    // Q_NOISE adapte le bruit du modèle
    Q_NOISE = Q_NOISE_MIN_BASE * envQFactor + (Q_NOISE_MAX - Q_NOISE_MIN_BASE) * accuracyPenaltyFactor;
    Q_NOISE = Math.min(Q_NOISE_MAX, Q_NOISE); 
    
    let currentSpeedSound = SPEED_SOUND;
    let currentSpeedLight = C_L;

    if (netherMode) {
        currentSpeedSound = SPEED_SOUND * NETHER_RATIO;
        currentSpeedLight = C_L * NETHER_RATIO; 
        Q_NOISE = Math.min(Q_NOISE_MAX, Q_NOISE * 1.5);
    }

    // --- Détermination de la validité de la position GPS ---
    let gpsPositionValid = false;
    const rFactor = getKalmanRFactor(); // Utilisation du R-Factor ajusté selon la fréquence
    
    if (acc < MAX_GPS_ACCURACY_FOR_USE * rFactor && lat !== 0 && lon !== 0) {
        gpsPositionValid = true;
    }

    // --- EKF (IMU-Only avec ZVU) ---
    const predictedSpd = kSpd + latestIMULinearAccel * dt; 
    kUncert = kUncert + Q_NOISE * dt; 
    const predictedSpdPositive = Math.max(0, predictedSpd); 
    kSpd = predictedSpdPositive;
    
    // ZVU (Zero Velocity Update)
    if (latestIMULinearAccel < 0.05) { 
         kSpd = 0;
         kUncert = MIN_UNCERT_FLOOR;
    }
    // ZVU ÉTALONNAGE
    if (kSpd < 0.05) { 
        kSpd = 0; 
        kUncert = MIN_UNCERT_FLOOR; 
    }
    
    // --- ACCÉLÉRATION HORIZONTALE (EKF Stable) ---
    let sSpdHorizFE = Math.abs(kSpd); 
    if (sSpdHorizFE > maxSpd) maxSpd = sSpdHorizFE; 
    
    // ÉTALONNAGE EN FONCTION DE KUNCERT
    let uncertFactor = 1.0;
    if (kUncert > MIN_UNCERT_FLOOR) {
        const normalizedUncert = Math.min(1, (kUncert - MIN_UNCERT_FLOOR) / (KUNCERT_MAX - MIN_UNCERT_FLOOR));
        uncertFactor = 1.0 - (normalizedUncert * (1.0 - KUNCERT_FACTOR_MIN));
    }
    sSpdHorizFE *= uncertFactor; 
    
    // Calcul d'accélération
    const accel_ekf = (dt > MIN_DT) ? (sSpdHorizFE - lastFSpeed) / dt : 0;
    const accel_long = accel_ekf; 
    
    lastFSpeed = sSpdHorizFE; 

    const currentGForceLong = Math.abs(accel_long / g_dynamic); 
    if (currentGForceLong > maxGForce) maxGForce = currentGForceLong; 

    // --- LOGIQUE DE MISE À JOUR DE LA POSITION (CORRECTION DE DÉRIVE GPS + DR) ---
    
    const env = $('environment-select').value;
    const trustIMUHeading = (env === 'OPEN' || env === 'FOREST');
    let headingSource = 'N/A'; 

    if (gpsPositionValid) {
        if (pos.coords.heading !== null && pos.coords.heading >= 0) {
             lastHeading = pos.coords.heading; 
             headingSource = 'GPS';
        } else {
             headingSource = 'GPS (No Hdg)';
        }
        lat = gpsLat;
        lon = gpsLon;
        lastPos = { latitude: lat, longitude: lon };
    }
    
    // 2. Dead Reckoning
    if (!gpsPositionValid && sSpdHorizFE > MIN_SPD && lastPos && dt > 0) {
        
        if (imuHeading !== 0 && trustIMUHeading) {
            lastHeading = imuHeading; 
            headingSource = 'IMU (DR)';
        } else if (headingSource !== 'GPS') {
            headingSource = 'DR (Last Hdg)';
        }
        
        const d_horiz_ekf = sSpdHorizFE * dt;
        const bearingRad = lastHeading * D2R; 
        
        const d_lat_m = d_horiz_ekf * Math.cos(bearingRad);
        const d_lon_m = d_horiz_ekf * Math.sin(bearingRad);
        
        const d_lat_deg = d_lat_m / R_E * R2D;
        const d_lon_deg = d_lon_m / (R_E * Math.cos(lat * D2R)) * R2D;
        
        lat += d_lat_deg;
        lon += d_lon_deg;
        
        lastPos = { latitude: lat, longitude: lon };
    }
    
    // --- CALCUL VITESSE 3D STABLE ---
    const sSpdFE_3D = Math.sqrt(sSpdHorizFE * sSpdHorizFE + verticalSpeedRaw * verticalSpeedRaw);
    
    // LISSAGE TEMPOREL (pour l'affichage)
    const alpha = dt / (SMOOTHING_TIME_CONSTANT + dt);
    smoothedSpeed = (alpha * sSpdFE_3D) + ((1 - alpha) * smoothedSpeed);
    const finalDisplaySpeed = smoothedSpeed;

    // Calcul de la distance 3D totale
    if (lastPos && sSpdFE_3D > MIN_SPD) { 
        const d_3d = sSpdFE_3D * dt; 
        distM += d_3d; 
        timeMoving += dt;
    } 
    
    if (currentAlt !== null) lastAlt = currentAlt; 
    alt = currentAlt; 

    // --- MISE À JOUR DOM ---
    
    // Vitesse (Utilise finalDisplaySpeed)
    $('speed-stable').textContent = `${(finalDisplaySpeed * KMH_MS).toFixed(4)} km/h (3D)`;
    $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(4)} km/h`;
    $('speed-stable-ms').textContent = `${finalDisplaySpeed.toFixed(3)} m/s (3D)`; 

    // Localisation & Temps
    $('distance-total-km').textContent = `${(distM/1000).toFixed(3)} km | ${distM.toFixed(2)} m (3D)`;
    $('time-moving').textContent = timeMoving.toFixed(2) + ' s';
    $('latitude').textContent = lat.toFixed(6);
    $('longitude').textContent = lon.toFixed(6);
    $('altitude-gps').textContent = currentAlt !== null ? `${currentAlt.toFixed(2)} m` : 'N/A';
    $('gps-precision').textContent = acc !== null ? `${acc.toFixed(3)} m` : 'N/A';
    $('underground-status').textContent = currentAlt !== null && currentAlt < ALT_TH ? 'OUI' : 'Non';
    $('vertical-speed').textContent = `${verticalSpeedRaw.toFixed(2)} m/s`;
    
    // Accélération & G-Forces
    $('accel-long').textContent = `${(accel_long).toFixed(3)} m/s ²`; 
    $('force-g-long').textContent = `${(accel_long / g_dynamic).toFixed(2)} G | Max: ${maxGForce.toFixed(2)} G`;
    
    // Diagnostics EKF
    $('kalman-uncert').textContent = `${kUncert.toFixed(5)} m² (Horiz.) | Q: ${Q_NOISE.toExponential(2)}`;
    $('kalman-r-dyn').textContent = `DR Mode (IMU) | R-Factor: ${rFactor.toFixed(1)}`; 
    
    // Cap (Heading)
    $('current-heading').textContent = `${lastHeading.toFixed(1)} ° (${headingSource})`;
    
    // Vitesse Cosmique
    $('perc-speed-c').textContent = `${(finalDisplaySpeed / currentSpeedLight * 100).toExponential(2)}%`;
    $('perc-speed-sound').textContent = `${(finalDisplaySpeed / currentSpeedSound * 100).toFixed(2)}%`;
    
    // Énergie & Puissance (CORRIGÉ : Assure que les variables sont utilisées)
    const mass = parseFloat($('mass-input').value) || 70;
    const kineticEnergy = 0.5 * mass * finalDisplaySpeed * finalDisplaySpeed;
    const mechanicalPower = mass * accel_long * finalDisplaySpeed;

    $('kinetic-energy').textContent = `${kineticEnergy.toFixed(2)} J`; 
    $('mechanical-power').textContent = `${mechanicalPower.toFixed(2)} W`; 
    
    savePrecisionRecords();
    if (lat !== 0 && lon !== 0) updateMap(lat, lon); 
    }
// Note: Le BLOC 2/2 reste inchangé, car seule la fonction getKalmanQNoiseFactor a été ajoutée au BLOC 1.
// =================================================================
// BLOC 2/2 : FONCTIONS SECONDAIRES (ASTRO/MÉTÉO/CARTE) & INITIALISATION
// =================================================================

// --- FONCTIONS SECONDAIRES (Astro, Météo, Carte, Horloge) ---

function getEOT(date) { 
    // Équation du Temps (Approximation)
    const J = date.getTime() / 86400000 + 2440587.5; 
    const n = J - 2451545.0;
    const L = (280.460 + 0.98564736 * n) % 360;
    const g = (357.528 + 0.98560030 * n) % 360;
    const lambda = L + 1.915 * Math.sin(g * D2R) + 0.020 * Math.sin(2 * g * D2R);
    const epsilon = 23.439 - 0.0000004 * n;
    const RA = Math.atan2(Math.sin(lambda * D2R) * Math.cos(epsilon * D2R) , Math.cos(lambda * D2R)) * R2D;
    const EOT = L - RA;
    return EOT * 4; 
}

function updateAstro(latitude, longitude) {
    if (latitude === 0 && longitude === 0) return;

    const now = getCDate();

    // Calcul du temps Minecraft
    const totalMsToday = (now.getHours() * 3600 + now.getMinutes() * 60 + now.getSeconds()) * 1000 + now.getMilliseconds();
    let msSinceMcStart = (totalMsToday - MC_START_OFFSET_MS) % REAL_DAY_MS;
    if (msSinceMcStart < 0) msSinceMcStart += REAL_DAY_MS;

    const mcTimeMs = (msSinceMcStart / REAL_DAY_MS) * MC_DAY_MS;

    const mcTotalSeconds = Math.floor(mcTimeMs / 1000);
    const mcMinutesTotal = Math.floor(mcTotalSeconds / 60);
    const mcHoursDisplay = Math.floor(mcMinutesTotal / 60) % 24; 
    const mcMinutesDisplay = mcMinutesTotal % 60;
    const mcSecondsDisplay = mcTotalSeconds % 60;

    const mcTimeStr = `${mcHoursDisplay.toString().padStart(2, '0')}:${mcMinutesDisplay.toString().padStart(2, '0')}:${mcSecondsDisplay.toString().padStart(2, '0')}`;
    
    if ($('mc-time')) $('mc-time').textContent = mcTimeStr;

    let clockRotation = (mcTimeMs / MC_DAY_MS) * 360; 
    const sunEl = $('sun-element');
    if (sunEl) sunEl.style.transform = `translate(-50%, -120%) rotate(${clockRotation}deg)`; 

    // Nécessite la librairie SunCalc
    if (typeof SunCalc !== 'undefined') {
        const pos = SunCalc.getPosition(now, latitude, longitude);
        const moonIllumination = SunCalc.getMoonIllumination(now);
        const moonPos = SunCalc.getMoonPosition(now, latitude, longitude);
    
        const eotMinutes = getEOT(now);
        const UTCHours = now.getUTCHours() + now.getUTCMinutes() / 60 + now.getUTCSeconds() / 3600;
        const TSTHours = UTCHours + (eotMinutes / 60) + (longitude / 15);
        const tstHours24 = TSTHours % 24;
        const h = Math.floor(tstHours24);
        const m = Math.floor((tstHours24 - h) * 60);
        const s = Math.floor(((tstHours24 - h) * 60 - m) * 60);

        $('tst').textContent = `${h.toString().padStart(2, '0')}:${m.toString().padStart(2, '0')}:${s.toString().padStart(2, '0')}`;
        const moonEl = $('moon-element');
        if (moonEl) moonEl.style.transform = `translate(-50%, -120%) rotate(${(moonPos.azimuth + Math.PI) * R2D}deg)`; 
        if ($('moon-phase-display')) $('moon-phase-display').textContent = `Phase: ${(moonIllumination.phase * 100).toFixed(1)}%`;
    } else {
        $('tst').textContent = `N/A (SunCalc)`;
    }
}

function getWeather() {
    if (lat === 0 || lon === 0) return;
    if (OWM_API_KEY === "VOTRE_CLE_API_OPENWEATHERMAP") return; 
    
    const url = `https://api.openweathermap.org/data/2.5/weather?lat=${lat}&lon=${lon}&appid=${OWM_API_KEY}&units=metric`;
    fetch(url)
        .then(response => response.json())
        .then(data => {
            if (data.main) {
                if ($('temp-air')) $('temp-air').textContent = `${data.main.temp.toFixed(1)} °C`;
                if ($('pressure')) $('pressure').textContent = `${data.main.pressure.toFixed(0)} hPa`;
                if ($('wind-speed-ms')) $('wind-speed-ms').textContent = `${data.wind.speed.toFixed(1)} m/s`;
            }
        })
        .catch(err => console.error("Erreur de récupération météo :", err));
}

let map = null;
let marker = null;

function initMap() {
    if (typeof L === 'undefined') return;
    map = L.map('map-container').setView([0, 0], 2);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
    }).addTo(map);
    marker = L.marker([0, 0]).addTo(map).bindPopup("Position Actuelle");
}
function updateMap(latitude, longitude) {
    if (map && marker) {
        map.setView([latitude, longitude], map.getZoom() < 12 ? 12 : map.getZoom());
        marker.setLatLng([latitude, longitude]);
    }
}
function syncH() {
    const localStart = Date.now();
    fetch(SERVER_TIME_ENDPOINT)
        .then(response => response.json())
        .then(data => {
            const serverTime = new Date(data.utc_datetime).getTime();
            const localEnd = Date.now();
            const rtt = localEnd - localStart;
            serverOffset = serverTime - (localEnd - rtt / 2); 
        })
        .catch(err => {
            serverOffset = 0;
            console.warn("Erreur de synchronisation horaire, utilisation de l'heure locale.");
        });
}

// --- GESTION DES CAPTEURS IMU (Partie 5) ---

function handleDeviceOrientation(event) {
    if (emergencyStopActive) return;

    if (event.beta !== null && event.gamma !== null) {
        imuPitch = event.beta; 
        imuRoll = event.gamma; 
    }
    
    let heading = null;
    if (event.alpha !== null) {
        heading = event.alpha; 
    } else if (event.webkitCompassHeading !== null) {
        heading = event.webkitCompassHeading;
    }

    if (heading !== null) {
        imuHeading = (360 - heading) % 360; 
        if ($('imu-heading')) $('imu-heading').textContent = `${imuHeading.toFixed(1)} °`;
    }
    
    if ($('pitch-angle')) $('pitch-angle').textContent = imuPitch.toFixed(1) + ' °';
    if ($('roll-angle')) $('roll-angle').textContent = imuRoll.toFixed(1) + ' °';
}

function handleDeviceMotion(event) {
    if (emergencyStopActive) return;

    const accel = event.accelerationIncludingGravity;
    const g_dynamic = G_ACC_LOCAL; 
    const linearAccel = event.acceleration;
    
    let correctedLinearAccelX = 0;
    let correctedLinearAccelY = 0;
    let correctedLinearAccelZ = 0;
    
    if (linearAccel && linearAccel.x !== null) {
        correctedLinearAccelX = linearAccel.x;
        correctedLinearAccelY = linearAccel.y;
        correctedLinearAccelZ = linearAccel.z;
    } else if (accel && accel.x !== null) {
        // Correction manuelle de la gravité à partir des angles
        
        const pitchRad = imuPitch * D2R; 
        const rollRad = imuRoll * D2R;
        
        const gravX = g_dynamic * Math.sin(rollRad);
        const gravY = g_dynamic * Math.sin(pitchRad); 
        const gravZ = g_dynamic * Math.cos(rollRad) * Math.cos(pitchRad); 

        correctedLinearAccelX = accel.x - gravX;
        correctedLinearAccelY = accel.y - gravY;
        correctedLinearAccelZ = accel.z - gravZ;
        
        if ($('gravity-calculated')) {
            $('gravity-calculated').textContent = `X:${gravX.toFixed(2)}, Y:${gravY.toFixed(2)}, Z:${gravZ.toFixed(2)}`;
        }
    }
    
    const latestLinearAccelMagnitude = Math.sqrt(
        correctedLinearAccelX * correctedLinearAccelX + 
        correctedLinearAccelY * correctedLinearAccelY + 
        correctedLinearAccelZ * correctedLinearAccelZ
    );
    
    latestIMULinearAccel = latestLinearAccelMagnitude; 
    
    // --- Calcul et mise à jour de la Force G Latérale ---
    const accelHorizX = correctedLinearAccelX;
    const accelHorizY = correctedLinearAccelY;
    
    // Accélération latérale (virage)
    const accelLat = Math.sqrt(accelHorizX * accelHorizX + accelHorizY * accelHorizY);
    
    const currentGForceLat = accelLat / G_ACC_LOCAL;
    if (currentGForceLat > maxGForceLat) maxGForceLat = currentGForceLat;
    
    if ($('force-g-lat')) {
        $('force-g-lat').textContent = `${currentGForceLat.toFixed(2)} G | Max: ${maxGForceLat.toFixed(2)} G`;
    }
    // -----------------------------------------------------------------

    if ($('accel-imu-raw')) {
         $('accel-imu-raw').textContent = `${latestLinearAccelMagnitude.toFixed(3)} m/s²`;
    }
    if ($('accel-linear-corrected')) {
        $('accel-linear-corrected').textContent = `${latestLinearAccelMagnitude.toFixed(3)} m/s²`;
    }

    const latestAccelZ = accel.z || 0; 
    const accelVerticalCorrigee = correctedLinearAccelZ;

    if ($('accel-vertical-imu')) $('accel-vertical-imu').textContent = `${accelVerticalCorrigee.toFixed(3)} m/s²`;
    if ($('force-g-vertical')) $('force-g-vertical').textContent = `${(latestAccelZ / g_dynamic).toFixed(2)} G`;
}

function continueGPSStart() {
    const opts = { enableHighAccuracy: currentGPSMode === 'HIGH_FREQ', timeout: 5000, maximumAge: 0 }; 
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    wID = navigator.geolocation.watchPosition(updateDisp, (error) => {
        console.warn(`ERREUR GPS(${error.code}): ${error.message}`);
    }, opts);
    const freqSelect = $('freq-select');
    if (freqSelect) freqSelect.value = currentGPSMode; 
}

function requestIMUPermissionAndStart() {
    if (typeof DeviceOrientationEvent !== 'undefined' && typeof DeviceOrientationEvent.requestPermission === 'function') {
        DeviceOrientationEvent.requestPermission()
            .then(permissionState => {
                if (permissionState === 'granted') {
                    window.addEventListener('devicemotion', handleDeviceMotion, true);
                    window.addEventListener('deviceorientation', handleDeviceOrientation, true);
                } else {
                    console.warn("Permission DeviceOrientation/Motion refusée. Les données IMU ne seront pas utilisées.");
                }
                continueGPSStart(); 
            })
            .catch(err => {
                console.error("Erreur d'autorisation DeviceMotion/Orientation:", err);
                continueGPSStart(); 
            });
    } else {
        if (window.DeviceMotionEvent) {
             window.addEventListener('devicemotion', handleDeviceMotion, true);
        }
        if (window.DeviceOrientationEvent) { 
             window.addEventListener('deviceorientation', handleDeviceOrientation, true);
        }
        continueGPSStart();
    }
}

function startGPS() {
    if (wID === null) {
        sTime = sTime === null ? getCDate() : sTime; 
        requestIMUPermissionAndStart(); 
    }
    const toggleBtn = $('toggle-gps-btn');
    if (toggleBtn) {
        toggleBtn.textContent = '⏸️ PAUSE GPS';
        toggleBtn.style.backgroundColor = '#dc3545';
    }
}
function stopGPS() {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
    }
    const toggleBtn = $('toggle-gps-btn');
    if (toggleBtn) {
        toggleBtn.textContent = '▶️ MARCHE GPS';
        toggleBtn.style.backgroundColor = '#28a745';
    }
}

// --- PARTIE 6 : INITIALISATION DU DOM (EventListener) ---

document.addEventListener('DOMContentLoaded', () => {
    loadPrecisionRecords();
    initMap();
    syncH(); 
    
    // Boucle de mise à jour DOM/Temps (toutes les secondes)
    domID = setInterval(() => {
        const now = getCDate();
        if (now) {
            if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString();
            if ($('time-elapsed')) $('time-elapsed').textContent = sTime ? ((now.getTime() - sTime.getTime()) / 1000).toFixed(2) + ' s' : '0.00 s';
            if ($('time-moving')) $('time-moving').textContent = timeMoving.toFixed(2) + ' s';
        }
        
        if (lat !== 0 && lon !== 0 && typeof SunCalc !== 'undefined') updateAstro(lat, lon); 
    }, 1000); 
    
    // Boucle de mise à jour Météo (toutes les 30 secondes)
    weatherID = setInterval(getWeather, 30000); 

    // --- ÉCOUTEURS D'ÉVÉNEMENTS ---
    
    const gpsBtn = $('toggle-gps-btn');
    if (gpsBtn) gpsBtn.addEventListener('click', () => wID === null ? startGPS() : stopGPS() );
    
    const stopBtn = $('emergency-stop-btn');
    if (stopBtn) stopBtn.addEventListener('click', () => {
        emergencyStopActive = !emergencyStopActive;
        stopBtn.textContent = emergencyStopActive ? '🛑 Arrêt d\'urgence: ACTIF 🔴' : '🛑 Arrêt d\'urgence: INACTIF 🟢';
        stopBtn.style.backgroundColor = emergencyStopActive ? '#f8d7da' : '#dc3545';
        if (emergencyStopActive) stopGPS();
    });
    
    const netherBtn = $('nether-toggle-btn');
    const netherModeEl = $('mode-nether');
    if (netherBtn && netherModeEl) netherBtn.addEventListener('click', () => {
        netherMode = !netherMode;
        netherModeEl.textContent = netherMode ? `ACTIF (1:${1/NETHER_RATIO})` : 'DÉSACTIVÉ (1:1)';
    });

    const resetDistBtn = $('reset-dist-btn');
    if (resetDistBtn) resetDistBtn.addEventListener('click', () => { distM = 0; timeMoving = 0; });
    
    const resetMaxBtn = $('reset-max-btn');
    if (resetMaxBtn) resetMaxBtn.addEventListener('click', () => { maxSpd = 0; maxGForce = 0; maxGForceLat = 0; savePrecisionRecords(); });
    
    const toggleModeBtn = $('toggle-mode-btn');
    if (toggleModeBtn) {
        toggleModeBtn.addEventListener('click', () => {
            document.body.classList.toggle('dark-mode');
            toggleModeBtn.classList.toggle('active');
        });
    }

    const freqSelect = $('freq-select');
    if (freqSelect) freqSelect.addEventListener('change', (e) => {
        currentGPSMode = e.target.value;
        if (wID !== null) {
            stopGPS();
            startGPS();
        }
    });

    const envSelect = $('environment-select');
    if (envSelect) envSelect.addEventListener('change', () => {
        // Le changement d'environnement impacte Q et R et sera pris en compte
        // lors du prochain cycle updateDisp via getKalmanRFactor et getKalmanQNoiseFactor.
        console.log(`Environnement changé à: ${envSelect.value}`);
    });
    
    // Démarrage initial
    startGPS();
});
