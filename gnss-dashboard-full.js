// =================================================================
// BLOC 1/2 : CŒUR DU SYSTÈME (EKF FINAL)
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
const Q_NOISE_MIN_BASE = 0.0001; 
const Q_NOISE_MAX = 0.0005; 
let Q_NOISE = Q_NOISE_MAX; 
const MIN_SPD = 0.001; 
const MIN_UNCERT_FLOOR = Q_NOISE_MIN_BASE * MIN_DT; 
const ALT_TH = -50; 
const R_E = 6371000; 
const G_CONST = 6.67430e-11; 
const M_EARTH = 5.972e24; 

const MAX_GPS_ACCURACY_FOR_USE = 50.0; 
const KUNCERT_MAX = 0.05; 
const KUNCERT_FACTOR_MIN = 0.85; 
const SMOOTHING_TIME_CONSTANT = 0.1; 
const ACCEL_FILTER_ALPHA = 0.5; // Constante de lissage (DOIT correspondre à celle du Bloc 2/2)

// CONSTANTES DE CLAMPING DE SÉCURITÉ ANTI-DÉRIVE
const MAX_IMU_ACCEL_INTEGRATION = 30.0; // Limite d'accélération (3G)
const MAX_EKF_SPEED = 41.7; // Limite max de vitesse EKF (150 km/h)

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

let rawIMULinearAccel = 0; // Accélération pure brute (avant lissage)
let filteredIMULinearAccel = 0; // Accélération pure filtrée (utilisée par EKF)
let latestIMULinearAccel = 0; // Utilise désormais filteredIMULinearAccel

let lastHeading = 0; 
let imuHeading = 0; 
let imuPitch = 0; 
let imuRoll = 0; 
let verticalSpeedRaw = 0; 
let smoothedSpeed = 0; 

// --- PARTIE 3 : FONCTIONS UTILITAIRES ET PERSISTANCE (Inchangé) ---
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

function getKalmanRFactor() {
    const env = $('environment-select').value;
    let rFactor = 1.0;

    switch (env) {
        case 'FOREST': rFactor = 1.5; break; 
        case 'METAL': rFactor = 3.0; break; 
        case 'CONCRETE': rFactor = 2.0; break; 
        default: rFactor = 1.0; 
    }

    if (currentGPSMode === 'HIGH_FREQ') {
        rFactor *= 1.2; 
    }
    
    return rFactor;
}

function getKalmanQNoiseFactor() {
    const env = $('environment-select').value;
    switch (env) {
        case 'OPEN': return 0.7; 
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

// --- PARTIE 4 : GESTIONNAIRE EKF ET GPS ---

/** Handler des données GPS et du filtre EKF. */
function updateDisp(pos) {
    if (emergencyStopActive) return;

    const acc = pos.coords.accuracy; 
    
    // ... (Calculs initiaux du temps, du bruit Q, etc., inchangés)
    const gpsLat = pos.coords.latitude; 
    const gpsLon = pos.coords.longitude;
    const currentAlt = pos.coords.altitude; 
    const now = getCDate();
    
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
    
    if (currentAlt !== null && lastAlt !== 0 && dt > 0) {
        verticalSpeedRaw = (currentAlt - lastAlt) / dt;
    } else {
        verticalSpeedRaw = 0;
    }
    
    const accuracyPenaltyFactor = Math.min(1.0, pos.coords.accuracy / MAX_GPS_ACCURACY_FOR_USE);
    const envQFactor = getKalmanQNoiseFactor();
    
    Q_NOISE = Q_NOISE_MIN_BASE * envQFactor + (Q_NOISE_MAX - Q_NOISE_MIN_BASE) * accuracyPenaltyFactor;
    Q_NOISE = Math.min(Q_NOISE_MAX, Q_NOISE); 
    
    let currentSpeedSound = SPEED_SOUND;
    let currentSpeedLight = C_L;

    if (netherMode) {
        currentSpeedSound = SPEED_SOUND * NETHER_RATIO;
        currentSpeedLight = C_L * NETHER_RATIO; 
        Q_NOISE = Math.min(Q_NOISE_MAX, Q_NOISE * 1.5);
    }

    let gpsPositionValid = false;
    const rFactor = getKalmanRFactor(); 
    
    if (acc < MAX_GPS_ACCURACY_FOR_USE * rFactor && lat !== 0 && lon !== 0) {
        gpsPositionValid = true;
    }

    // --- LOGIQUE EKF (PRÉDICTION + CORRECTION) ---

    // 🛡️ CLAMPING DE L'ACCÉLÉRATION IMU FILTRÉE POUR ÉVITER LE BRUIT
    // latestIMULinearAccel est maintenant l'accélération FILTRÉE par le passe-bas (Bloc 2/2)
    const clampedAccel = Math.min(latestIMULinearAccel, MAX_IMU_ACCEL_INTEGRATION);
    
    // 1. Mise à jour de l'état (vitesse) et de la covariance (incertitude) par le modèle (IMU)
    kSpd = kSpd + clampedAccel * dt; 
    kUncert = kUncert + Q_NOISE * dt; 
    
    // ⬆️ ZVU AMÉLIORÉ (Seuil : 0.10 m/s²) : Si l'accélération pure est négligeable, la vitesse est zéro
    if (clampedAccel < 0.10) { 
        kSpd = 0;
        kUncert = MIN_UNCERT_FLOOR;
    }

    // --- EKF: ÉTAPE DE CORRECTION (Mesure GPS de Vitesse) ---
    if (pos.coords.speed !== null && !isNaN(pos.coords.speed) && gpsPositionValid) {
        
        const measuredSpd = pos.coords.speed;
        
        const R_MEASUREMENT_BASE = Math.max(1.0, pos.coords.accuracy);
        const R_MEASUREMENT = Math.pow(R_MEASUREMENT_BASE, 2) * rFactor; 

        const innovation = measuredSpd - kSpd; 
        const innovationCovariance = kUncert + R_MEASUREMENT; 
        const kalmanGain = kUncert / innovationCovariance;
        
        kSpd = kSpd + kalmanGain * innovation;
        kUncert = (1 - kalmanGain) * kUncert;
    }
    
    // 🛑 SÉCURITÉ FINALE : CLAMPING DE LA VITESSE EKF
    kSpd = Math.max(0, kSpd);
    kSpd = Math.min(kSpd, MAX_EKF_SPEED); 
    kUncert = Math.max(MIN_UNCERT_FLOOR, kUncert);

    // --- ACCÉLÉRATION HORIZONTALE (EKF Stable) et LOGIQUE DR (Le reste est inchangé) ---
    let sSpdHorizFE = Math.abs(kSpd); 
    if (sSpdHorizFE > maxSpd) maxSpd = sSpdHorizFE; 
    
    let uncertFactor = 1.0;
    if (kUncert > MIN_UNCERT_FLOOR) {
        const normalizedUncert = Math.min(1, (kUncert - MIN_UNCERT_FLOOR) / (KUNCERT_MAX - MIN_UNCERT_FLOOR));
        uncertFactor = 1.0 - (normalizedUncert * (1.0 - KUNCERT_FACTOR_MIN));
    }
    sSpdHorizFE *= uncertFactor; 
    
    const accel_ekf = (dt > MIN_DT) ? (sSpdHorizFE - lastFSpeed) / dt : 0;
    const accel_long = accel_ekf; 
    lastFSpeed = sSpdHorizFE; 

    const currentGForceLong = Math.abs(accel_long / g_dynamic); 
    if (currentGForceLong > maxGForce) maxGForce = currentGForceLong; 

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
    
    const sSpdFE_3D = Math.sqrt(sSpdHorizFE * sSpdHorizFE + verticalSpeedRaw * verticalSpeedRaw);
    
    const alpha = dt / (SMOOTHING_TIME_CONSTANT + dt);
    smoothedSpeed = (alpha * sSpdFE_3D) + ((1 - alpha) * smoothedSpeed);
    const finalDisplaySpeed = smoothedSpeed;

    if (lastPos && sSpdFE_3D > MIN_SPD) { 
        const d_3d = sSpdFE_3D * dt; 
        distM += d_3d; 
        timeMoving += dt;
    } 
    
    if (currentAlt !== null) lastAlt = currentAlt; 
    alt = currentAlt; 

    // --- MISE À JOUR DOM (Inchangé) ---
    
    $('speed-stable').textContent = `${(finalDisplaySpeed * KMH_MS).toFixed(4)} km/h (3D)`;
    $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(4)} km/h`;
    $('speed-stable-ms').textContent = `${finalDisplaySpeed.toFixed(3)} m/s (3D)`; 

    $('distance-total-km').textContent = `${(distM/1000).toFixed(3)} km | ${distM.toFixed(2)} m (3D)`;
    $('time-moving').textContent = timeMoving.toFixed(2) + ' s';
    $('latitude').textContent = lat.toFixed(6);
    $('longitude').textContent = lon.toFixed(6);
    $('altitude-gps').textContent = currentAlt !== null ? `${currentAlt.toFixed(2)} m` : 'N/A';
    $('gps-precision').textContent = acc !== null ? `${acc.toFixed(3)} m` : 'N/A';
    $('underground-status').textContent = currentAlt !== null && currentAlt < ALT_TH ? 'OUI' : 'Non';
    $('vertical-speed').textContent = `${verticalSpeedRaw.toFixed(2)} m/s`;
    
    $('accel-long').textContent = `${(accel_long).toFixed(3)} m/s ²`; 
    $('force-g-long').textContent = `${(accel_long / g_dynamic).toFixed(2)} G | Max: ${maxGForce.toFixed(2)} G`;
    
    $('kalman-uncert').textContent = `${kUncert.toFixed(5)} m² (Horiz.) | Q: ${Q_NOISE.toExponential(2)}`;
    $('kalman-r-dyn').textContent = `DR Mode (IMU) | R-Factor: ${rFactor.toFixed(1)}`; 
    
    $('current-heading').textContent = `${lastHeading.toFixed(1)} ° (${headingSource})`;
    
    $('perc-speed-c').textContent = `${(finalDisplaySpeed / currentSpeedLight * 100).toExponential(2)}%`;
    $('perc-speed-sound').textContent = `${(finalDisplaySpeed / currentSpeedSound * 100).toFixed(2)}%`;
    
    const mass = parseFloat($('mass-input').value) || 70;
    const kineticEnergy = 0.5 * mass * finalDisplaySpeed * finalDisplaySpeed;
    const mechanicalPower = mass * accel_long * finalDisplaySpeed;

    $('kinetic-energy').textContent = `${kineticEnergy.toFixed(2)} J`; 
    $('mechanical-power').textContent = `${mechanicalPower.toFixed(2)} W`; 
    
    savePrecisionRecords();
    if (lat !== 0 && lon !== 0) updateMap(lat, lon); 
    }
// =================================================================
// BLOC 2/2 : FONCTIONS SECONDAIRES & IMU (AVEC FILTRE PASSE-BAS)
// =================================================================

// --- FONCTIONS SECONDAIRES (Astro, Météo, Carte, Horloge) ---

function getEOT(date) { /* ... (Inchangé) */ }
function updateAstro() { /* ... (Inchangé) */ }
function getWeather() { /* ... (Inchangé) */ }
function initMap() { /* ... (Inchangé) */ }
function updateMap(lat, lon) { /* ... (Inchangé) */ }
function syncH() { /* ... (Inchangé) */ }
function startGPS() { /* ... (Inchangé) */ }
function stopGPS() { /* ... (Inchangé) */ }
function requestIMUPermissionAndStart() { /* ... (Inchangé) */ }
function continueGPSStart() { /* ... (Inchangé) */ }

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
    
    // Si l'API fournit l'accélération linéaire sans gravité, on l'utilise directement
    if (linearAccel && linearAccel.x !== null) {
        correctedLinearAccelX = linearAccel.x;
        correctedLinearAccelY = linearAccel.y;
        correctedLinearAccelZ = linearAccel.z;
    } else if (accel && accel.x !== null) {
        
        // --- CORRECTION DE LA GRAVITÉ PAR ANGLES (NIVEAU À BULLE) ---
        const pitchRad = imuPitch * D2R; 
        const rollRad = imuRoll * D2R;
        
        const gravX = g_dynamic * Math.sin(rollRad);
        const gravY = g_dynamic * Math.sin(pitchRad); 
        const gravZ = g_dynamic * Math.cos(rollRad) * Math.cos(pitchRad); 

        // Accélération pure (corrigée)
        correctedLinearAccelX = accel.x - gravX;
        correctedLinearAccelY = accel.y - gravY;
        correctedLinearAccelZ = accel.z - gravZ;
        
        if ($('gravity-calculated')) {
            $('gravity-calculated').textContent = `X:${gravX.toFixed(2)}, Y:${gravY.toFixed(2)}, Z:${gravZ.toFixed(2)}`;
        }
    }
    
    // --- Calcul de l'accélération linéaire 3D (Magnitude) ---
    const latestLinearAccelMagnitude = Math.sqrt(
        correctedLinearAccelX * correctedLinearAccelX + 
        correctedLinearAccelY * correctedLinearAccelY + 
        correctedLinearAccelZ * correctedLinearAccelZ
    );
    
    rawIMULinearAccel = latestLinearAccelMagnitude; // Stockage de la valeur brute

    // 🌟 FILTRE PASSE-BAS TEMPOREL SUR L'ACCÉLÉRATION (Anti-Vibration)
    // Utilise la constante ACCEL_FILTER_ALPHA définie dans le Bloc 1/2
    filteredIMULinearAccel = (ACCEL_FILTER_ALPHA * rawIMULinearAccel) + 
                             ((1 - ACCEL_FILTER_ALPHA) * filteredIMULinearAccel);

    latestIMULinearAccel = filteredIMULinearAccel; // LA NOUVELLE ACCÉLÉRATION FILTRÉE
    
    // --- Calcul et mise à jour de la Force G Latérale ---
    const accelHorizX = correctedLinearAccelX;
    const accelHorizY = correctedLinearAccelY;
    
    const accelLat = Math.sqrt(accelHorizX * accelHorizX + accelHorizY * accelHorizY);
    
    const currentGForceLat = accelLat / G_ACC_LOCAL;
    if (currentGForceLat > maxGForceLat) maxGForceLat = currentGForceLat;
    
    if ($('force-g-lat')) {
        $('force-g-lat').textContent = `${currentGForceLat.toFixed(2)} G | Max: ${maxGForceLat.toFixed(2)} G`;
    }
    // -----------------------------------------------------------------

    if ($('accel-imu-raw')) {
         $('accel-imu-raw').textContent = `${rawIMULinearAccel.toFixed(3)} m/s²`;
    }
    if ($('accel-linear-corrected')) {
        $('accel-linear-corrected').textContent = `${filteredIMULinearAccel.toFixed(3)} m/s² (Filtré)`;
    }

    const latestAccelZ = accel.z || 0; 
    const accelVerticalCorrigee = correctedLinearAccelZ;

    if ($('accel-vertical-imu')) $('accel-vertical-imu').textContent = `${accelVerticalCorrigee.toFixed(3)} m/s²`;
    if ($('force-g-vertical')) $('force-g-vertical').textContent = `${(latestAccelZ / g_dynamic).toFixed(2)} G`;
}

// ... (Le reste du Bloc 2/2 est inchangé : handleDeviceMotion, initMap, DOMContentLoaded, etc.)
// =================================================================
// BLOC 2/2 : SUITE DES FONCTIONS SECONDAIRES & INITIALISATION
// (Le code IMU, y compris handleDeviceMotion, se trouve juste avant cette partie)
// =================================================================

// --- FONCTIONS SECONDAIRES (Astro, Météo, Carte, Horloge) ---

// Équation du temps (Approximation pour la correction solaire)
function getEOT(date) {
    const dayOfYear = Math.floor((date - new Date(date.getFullYear(), 0, 0)) / 86400000);
    const B = (2 * Math.PI / 365) * (dayOfYear - 81);
    return 9.87 * Math.sin(2 * B) - 7.53 * Math.cos(B) - 1.5 * Math.sin(B); // en minutes
}

function updateAstro() {
    // Calcul de l'heure solaire locale
    if (lat !== 0 && lon !== 0) {
        const now = getCDate();
        const eotMinutes = getEOT(now);
        const timeZoneOffset = now.getTimezoneOffset(); // Décalage local en minutes
        
        // Correction de la longitude pour obtenir l'heure solaire
        const solarTimeMinutes = now.getHours() * 60 + now.getMinutes() + now.getSeconds() / 60 + eotMinutes + (lon * 4) + timeZoneOffset;
        
        const hours = Math.floor((solarTimeMinutes / 60) % 24);
        const minutes = Math.floor(solarTimeMinutes % 60);

        $('solar-time').textContent = `${String(hours).padStart(2, '0')}:${String(minutes).padStart(2, '0')}`;
    }
}

function getWeather() {
    if (lat === 0 || lon === 0) return;

    const url = `https://api.openweathermap.org/data/2.5/weather?lat=${lat}&lon=${lon}&appid=${OWM_API_KEY}&units=metric&lang=fr`;

    fetch(url)
        .then(response => response.json())
        .then(data => {
            if (data.main) {
                $('temp-air').textContent = `${data.main.temp.toFixed(1)} °C`;
                $('pressure-baro').textContent = `${data.main.pressure} hPa`;
                $('humidity-air').textContent = `${data.main.humidity} %`;
            }
            if (data.wind) {
                $('wind-speed').textContent = `${(data.wind.speed * KMH_MS).toFixed(1)} km/h`;
                $('wind-dir').textContent = `${data.wind.deg}°`;
            }
            if (data.weather && data.weather.length > 0) {
                $('weather-status').textContent = data.weather[0].description;
            }
        })
        .catch(error => {
            console.error("Erreur de récupération météo:", error);
            $('weather-status').textContent = 'Météo indisponible';
        });
}

// Fonction de synchronisation de l'heure serveur (pour la précision temporelle)
function syncH() {
    fetch(SERVER_TIME_ENDPOINT)
        .then(response => response.json())
        .then(data => {
            const serverTime = new Date(data.utc_datetime);
            const localTime = new Date();
            serverOffset = serverTime.getTime() - localTime.getTime();
            $('clock-utc').textContent = serverTime.toUTCString().slice(17, 25);
            console.log(`Synchronisation horaire effectuée. Décalage: ${serverOffset} ms`);
        })
        .catch(error => {
            console.error("Erreur de synchronisation horaire:", error);
            $('clock-utc').textContent = 'UTC: N/A';
        });
}

// --- GESTION GPS & IMU (Initialisation et Boucles) ---

function startGPS() {
    if (wID) return;

    const options = {
        enableHighAccuracy: true,
        timeout: 5000,
        maximumAge: 0
    };

    wID = navigator.geolocation.watchPosition(updateDisp, (error) => {
        console.error('Erreur GPS:', error);
        $('gps-precision').textContent = `Erreur: ${error.code}`;
    }, options);
    
    // Démarrage des boucles de mise à jour DOM et Météo/Astro
    domID = setInterval(updateAstro, 5000); 
    weatherID = setInterval(getWeather, 300000); // Mise à jour météo toutes les 5 minutes
    
    // Tentative de synchronisation horaire au démarrage
    syncH(); 
    
    // Tentative de démarrage des capteurs IMU (si non démarré par permission)
    if (window.DeviceOrientationEvent) {
        window.addEventListener('deviceorientation', handleDeviceOrientation, true);
    }
    if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', handleDeviceMotion, true);
    }
}

function stopGPS() {
    if (wID) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
    }
    if (domID) clearInterval(domID);
    if (weatherID) clearInterval(weatherID);
    
    // Arrêt des écouteurs IMU
    if (window.DeviceOrientationEvent) {
        window.removeEventListener('deviceorientation', handleDeviceOrientation, true);
    }
    if (window.DeviceMotionEvent) {
        window.removeEventListener('devicemotion', handleDeviceMotion, true);
    }

    emergencyStopActive = true;
    $('emergency-stop').textContent = 'RESTART';
    $('emergency-stop').classList.add('error');
}

function requestIMUPermissionAndStart() {
    if (typeof DeviceOrientationEvent.requestPermission === 'function') {
        DeviceOrientationEvent.requestPermission()
            .then(response => {
                if (response === 'granted') {
                    window.addEventListener('deviceorientation', handleDeviceOrientation, true);
                    window.addEventListener('devicemotion', handleDeviceMotion, true);
                    continueGPSStart();
                } else {
                    alert("Permission IMU refusée. Le dashboard fonctionnera sans accélération ni cap.");
                    continueGPSStart();
                }
            })
            .catch(error => {
                console.error("Erreur de permission IMU:", error);
                continueGPSStart();
            });
    } else {
        // Navigateurs ne nécessitant pas de demande explicite (ex: Android)
        continueGPSStart();
    }
}

function continueGPSStart() {
    // Tentative de démarrer le GPS après la permission IMU (ou si non nécessaire)
    startGPS();
}

// --- GESTION ÉVÉNEMENTS & DOM (Carte) ---

let map;
let marker;

function initMap() {
    // Initialise Google Maps (remplacer par votre implémentation si nécessaire)
    const mapElement = document.getElementById('map');
    if (!mapElement) return;
    
    map = new google.maps.Map(mapElement, {
        center: { lat: 0, lng: 0 },
        zoom: 2,
        mapTypeId: 'satellite'
    });
    
    marker = new google.maps.Marker({
        position: { lat: 0, lng: 0 },
        map: map,
        title: 'Position Actuelle'
    });
}

function updateMap(lat, lon) {
    if (!map || !marker) return;
    
    const newLatLng = new google.maps.LatLng(lat, lon);
    marker.setPosition(newLatLng);
    map.setCenter(newLatLng);
    
    // Ajustement dynamique du zoom en fonction de la vitesse (simple approximation)
    const currentSpeedKmH = kSpd * KMH_MS;
    let newZoom = 15;
    if (currentSpeedKmH > 100) newZoom = 13;
    else if (currentSpeedKmH > 300) newZoom = 10;
    
    map.setZoom(newZoom);
}

// --- POINT D'ENTRÉE ---

document.addEventListener('DOMContentLoaded', (event) => {
    // Boutons de contrôle
    $('start-gnss').addEventListener('click', requestIMUPermissionAndStart);
    $('emergency-stop').addEventListener('click', () => {
        if (emergencyStopActive) {
            emergencyStopActive = false;
            $('emergency-stop').textContent = 'STOP';
            $('emergency-stop').classList.remove('error');
            requestIMUPermissionAndStart(); // Redémarrer l'ensemble
        } else {
            stopGPS();
        }
    });
    
    // Toggle Nether Mode
    $('nether-mode-toggle').addEventListener('change', (e) => {
        netherMode = e.target.checked;
        $('mode-display').textContent = netherMode ? 'NETHER' : 'OVERWORLD';
    });

    // Reset Max Speed/G-Force
    $('reset-metrics').addEventListener('click', () => {
        maxSpd = 0;
        maxGForce = 0;
        maxGForceLat = 0;
        distM = 0;
        timeMoving = 0;
        savePrecisionRecords();
    });

    // Chargement des records locaux
    loadPrecisionRecords();

    // Initialisation de la carte après le chargement du DOM
    initMap(); 
});
