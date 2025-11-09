// =================================================================
// FICHIER FINAL ET COMPLET : gnss-dashboard-full.js (V9.1 - Vitesse 3D)
// Fusion GNSS/IMU avec EKF optimisé pour la marche, Cap Dynamique et 3D.
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
const Q_NOISE = 0.0002; // Bruit du processus EKF (optimisé pour la marche)
const MIN_SPD = 0.001; 
const MIN_UNCERT_FLOOR = Q_NOISE * MIN_DT; 
const ALT_TH = -50; 
const R_E = 6371000; 
const G_CONST = 6.67430e-11; 
const M_EARTH = 5.972e24; 

const MAX_GPS_ACCURACY_FOR_USE = 50.0; 
const SIGMA_DRIFT_THRESHOLD = 3.0; 

// Endpoints
const OWM_API_KEY = "VOTRE_CLE_API_OPENWEATHERMAP"; 
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc"; 

// --- PARTIE 2 : VARIABLES D'ÉTAT ---
let lat = 0, lon = 0, alt = 0;
let kSpd = 0, kUncert = 0.01; // Vitesse EKF (Horizontale)
let lastTS = 0, lastFSpeed = 0, distM = 0;
let lastPos = null, lastAlt = 0; 
let sTime = null, timeMoving = 0, maxSpd = 0; 
let maxGForce = 0;
let wID = null, domID = null, weatherID = null;

let currentGPSMode = 'HIGH_FREQ'; 
let emergencyStopActive = false;
let netherMode = false;
let G_ACC_LOCAL = G_ACC_STD; 
let serverOffset = 0; 
const P_RECORDS_KEY = 'gnss_precision_records'; 

let latestIMULinearAccel = 0; 
let lastHeading = 0; 
let imuHeading = 0; // Cap IMU brut
let verticalSpeedRaw = 0; // Vitesse verticale (nouvelle variable)

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

function distanceCalc(lat1, lon1, lat2, lon2) {
    if (lat1 === 0 && lon1 === 0) return 0;
    const R = R_E; 
    const dLat = (lat2 - lat1) * D2R;
    const dLon = (lon2 - lon1) * D2R;
    const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
              Math.cos(lat1 * D2R) * Math.cos(lat2 * D2R) * Math.sin(dLon / 2) * Math.sin(dLon / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    
    let distance = R * c;
    if (netherMode) distance *= (1 / NETHER_RATIO); 
    
    return distance;
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

function getKalmanR(accuracy) {
    const env = $('environment-select').value;
    let factor = 1.0;
    
    // Pénalise la mesure GPS pour donner plus de poids au modèle IMU/EKF à basse vitesse
    let basePenalty = 1.25; 
    
    switch (env) {
        case 'FOREST': factor = 1.5; break;
        case 'METAL': factor = 2.5; break;
        case 'CONCRETE': factor = 3.0; break; 
        default: factor = 1.0; break;
    }
    return accuracy * accuracy * factor * basePenalty; 
}

function loadPrecisionRecords() {
    try {
        const stored = localStorage.getItem(P_RECORDS_KEY);
        if (stored) {
            const loaded = JSON.parse(stored);
            maxGForce = loaded.max_g_force_max || 0;
            if ($('force-g-long')) $('force-g-long').textContent = `0.00 G | Max: ${maxGForce.toFixed(2)} G`;
        }
    } catch (e) { console.error("Erreur de chargement des records:", e); }
}

function savePrecisionRecords() {
    try {
        const records = { max_g_force_max: maxGForce };
        localStorage.setItem(P_RECORDS_KEY, JSON.stringify(records));
    } catch (e) { console.error("Erreur de sauvegarde des records:", e); }
}

// --- PARTIE 4 : GESTIONNAIRE EKF ET GPS ---

/** Handler des données GPS et du filtre EKF. */
function updateDisp(pos) {
    if (emergencyStopActive) return;

    const acc = pos.coords.accuracy; 
    
    // --- MISE À JOUR DES POSITIONS ET TEMPS ---
    const gpsLat = pos.coords.latitude; 
    const gpsLon = pos.coords.longitude;
    const currentAlt = pos.coords.altitude; 
    
    const now = getCDate();
    
    if (lastTS === 0) lastTS = now.getTime(); 
    const dt = (now.getTime() - lastTS) / 1000;
    lastTS = now.getTime();
    
    if (dt < MIN_DT) return; 

    const g_dynamic = calculateLocalGravity(currentAlt);
    
    // --- CALCUL VITESSE VERTICALE (Altitude) ---
    // Utilise la variation d'altitude sur l'intervalle de temps.
    if (currentAlt !== null && lastAlt !== 0 && dt > 0) {
        verticalSpeedRaw = (currentAlt - lastAlt) / dt;
    } else {
        verticalSpeedRaw = 0;
    }
    
    // --- Calcul Vitesse Brute (Mesure GPS pour la correction) ---
    let speedRaw = 0;
    let gpsMeasurementValid = false;
    let gpsPositionValid = false;

    // GPS Position Validée : bonne précision ET déjà initialisé
    if (acc < MAX_GPS_ACCURACY_FOR_USE && lat !== 0 && lon !== 0) {
        gpsPositionValid = true;
    }

    if (pos.coords.speed !== null && pos.coords.speed >= 0 && gpsPositionValid) {
        speedRaw = pos.coords.speed;
        gpsMeasurementValid = true;
    } 
    else if (lastPos && dt > 0 && gpsPositionValid) {
        const d_horiz_raw = distanceCalc(lastPos.latitude, lastPos.longitude, gpsLat, gpsLon);
        speedRaw = d_horiz_raw / dt;
        gpsMeasurementValid = true;
    }
    
    // --- EKF (Filtre de Kalman pour la vitesse 2D) ---
    
    // 1. Prédiction (utilise l'IMU)
    // On utilise l'accélération IMU LÉGÈREMENT FILTRÉE pour une meilleure prédiction
    const predictedSpd = kSpd + latestIMULinearAccel * dt; 
    kUncert = kUncert + Q_NOISE * dt; 
    
    const predictedSpdPositive = Math.max(0, predictedSpd); 
    
    if (gpsMeasurementValid) {
        let kR = getKalmanR(acc); 

        // --- DÉTECTION DE DÉRIVE / OUTLIER (Innovation Check) ---
        const totalUncertainty = kUncert + kR;
        const innovation = speedRaw - predictedSpdPositive;
        
        if (Math.abs(innovation) > SIGMA_DRIFT_THRESHOLD * Math.sqrt(totalUncertainty)) {
             console.warn(`[EKF] Dérive/Anomalie détectée: Mesure GPS rejetée.`);
             gpsMeasurementValid = false; 
        }
        
        if (gpsMeasurementValid) {
            // 2. Correction GPS (Utilisation normale)
            let kGain = kUncert / totalUncertainty;
            kSpd = predictedSpdPositive + kGain * innovation; 
            kUncert = (1 - kGain) * kUncert; 
            kUncert = Math.max(kUncert, MIN_UNCERT_FLOOR); 
        }
    } 
    
    if (!gpsMeasurementValid) {
        // 2. Dead Reckoning (mode de dérive sans correction)
        kSpd = predictedSpdPositive;
        
        // ZVU basé sur l'accélération IMU si en Dead Reckoning (arrêt réaliste)
        if (latestIMULinearAccel < 0.05) { 
             kSpd = 0;
             kUncert = MIN_UNCERT_FLOOR;
        }
    }
    
    // --- ZVU ÉTALONNAGE (Nettoyage à l'arrêt) ---
    if (kSpd < 0.05) { 
        kSpd = 0; 
        kUncert = MIN_UNCERT_FLOOR; 
    }
    
    // --- ACCÉLÉRATION HORIZONTALE (EKF Stable) ---
    let sSpdHorizFE = Math.abs(kSpd); // Vitesse horizontale filtrée (stable)
    if (sSpdHorizFE > maxSpd) maxSpd = sSpdHorizFE; 
    
    const accel_ekf = (dt > MIN_DT) ? (sSpdHorizFE - lastFSpeed) / dt : 0;
    const accel_long = accel_ekf; 
    
    lastFSpeed = sSpdHorizFE; 

    const currentGForceLong = Math.abs(accel_long / g_dynamic); 
    if (currentGForceLong > maxGForce) maxGForce = currentGForceLong; 

    // --- LOGIQUE DE MISE À JOUR DE LA POSITION (DR AMÉLIORÉ AVEC CONFIANCE CAP) ---
    
    const env = $('environment-select').value;
    // Confiance Dynamique : On ne fait confiance à l'IMU que dans un environnement non métallique/béton
    const trustIMUHeading = (env === 'OPEN' || env === 'FOREST');

    if (gpsPositionValid) {
        // 1. Cap GPS (Priorité absolue)
        if (pos.coords.heading !== null && pos.coords.heading >= 0) {
             lastHeading = pos.coords.heading; 
        }
        lat = gpsLat;
        lon = gpsLon;
        lastPos = { latitude: lat, longitude: lon };
    }
    
    // 2. Cap IMU conditionnel (Si GPS perdu ET on fait confiance à l'IMU)
    else if (!gpsPositionValid && imuHeading !== 0 && trustIMUHeading) {
        lastHeading = imuHeading; 
    }
    // Si pas de confiance IMU, lastHeading conserve sa dernière valeur (cap constant).
    
    // Dead Reckoning : Intégration de la vitesse pour estimer la position
    if (!gpsPositionValid && sSpdHorizFE > MIN_SPD && lastPos && dt > 0) {
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
    // Utilise la vitesse horizontale filtrée par l'EKF et la vitesse verticale brute.
    const sSpdFE_3D = Math.sqrt(sSpdHorizFE * sSpdHorizFE + verticalSpeedRaw * verticalSpeedRaw);

    // Calcul de la distance 3D totale (utilise la vitesse filtrée 3D)
    if (lastPos && sSpdFE_3D > MIN_SPD) { 
        const d_3d = sSpdFE_3D * dt; 
        distM += d_3d; 
        timeMoving += dt;
    } 
    
    if (currentAlt !== null) lastAlt = currentAlt; 
    alt = currentAlt; // Mise à jour de la variable alt globale

    // --- MISE À JOUR DOM ---
    
    // Affichage de la Vitesse 3D Stable
    $('speed-stable').textContent = `${(sSpdFE_3D * KMH_MS).toFixed(4)} km/h (3D)`;
    $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(4)} km/h`;
    $('speed-stable-ms').textContent = `${sSpdFE_3D.toFixed(3)} m/s (3D)`; 

    // Affichage de la Distance 3D Totale
    $('distance-total-km').textContent = `${(distM/1000).toFixed(3)} km | ${distM.toFixed(2)} m (3D)`;
    
    $('perc-speed-c').textContent = `${(sSpdFE_3D / C_L * 100).toExponential(2)}%`;
    $('perc-speed-sound').textContent = `${(sSpdFE_3D / SPEED_SOUND * 100).toFixed(2)}%`;

    $('kalman-uncert').textContent = `${kUncert.toFixed(5)} m² (Horiz.)`;
    $('kalman-r-dyn').textContent = `${gpsMeasurementValid ? getKalmanR(acc).toFixed(5) : 'DR Mode'} m²`;
    $('accel-long').textContent = `${(accel_long).toFixed(3)} m/s ²`; 
    $('force-g-long').textContent = `${(accel_long / g_dynamic).toFixed(2)} G | Max: ${maxGForce.toFixed(2)} G`;
    $('vertical-speed').textContent = `${verticalSpeedRaw.toFixed(2)} m/s`;
    
    $('latitude').textContent = lat.toFixed(6);
    $('longitude').textContent = lon.toFixed(6);
    $('altitude-gps').textContent = currentAlt !== null ? `${currentAlt.toFixed(2)} m` : 'N/A';
    $('gps-precision').textContent = acc !== null ? `${acc.toFixed(3)} m` : 'N/A';
    $('underground-status').textContent = currentAlt !== null && currentAlt < ALT_TH ? 'OUI' : 'Non';
    
    savePrecisionRecords();

    if (lat !== 0 && lon !== 0) updateMap(lat, lon); 
    
    const mass = parseFloat($('mass-input').value) || 70;
    $('kinetic-energy').textContent = `${(0.5 * mass * sSpdFE_3D * sSpdFE_3D).toFixed(2)} J`; // Utilisation de la vitesse 3D
    $('mechanical-power').textContent = `${(mass * accel_long * sSpdHorizFE).toFixed(2)} W`; // La puissance reste horizontale/longitudinale
}

// --- PARTIE 5 & 6 : GESTION DES CAPTEURS ET INITIALISATION (Non Modifiée) ---

function handleDeviceOrientation(event) {
    if (emergencyStopActive) return;

    let heading = null;
    if (event.alpha !== null) {
        heading = event.alpha; 
    } else if (event.webkitCompassHeading !== null) {
        heading = event.webkitCompassHeading;
    }

    if (heading !== null) {
        // Conversion pour obtenir l'azimut du nord magnétique
        imuHeading = (360 - heading) % 360; 
        if ($('imu-heading')) $('imu-heading').textContent = `${imuHeading.toFixed(1)} °`;
    }
}

/** Handler des données IMU (accéléromètre) */
function handleDeviceMotion(event) {
    if (emergencyStopActive) return;

    const accel = event.accelerationIncludingGravity;
    const g_dynamic = G_ACC_LOCAL; 
    
    const linearAccel = event.acceleration;
    if (linearAccel) {
        const latestLinearAccelMagnitude = Math.sqrt(linearAccel.x*linearAccel.x + linearAccel.y*linearAccel.y + linearAccel.z*linearAccel.z);
        latestIMULinearAccel = latestLinearAccelMagnitude; 
        
        if ($('accel-imu-raw')) {
             $('accel-imu-raw').textContent = `${latestLinearAccelMagnitude.toFixed(3)} m/s²`;
        }
    }
    
    const latestAccelZ = accel.z || 0; 
    const accelVerticalCorrigee = latestAccelZ - g_dynamic;

    if ($('accel-vertical-imu')) $('accel-vertical-imu').textContent = `${accelVerticalCorrigee.toFixed(3)} m/s²`;
    if ($('force-g-vertical')) $('force-g-vertical').textContent = `${(latestAccelZ / g_dynamic).toFixed(2)} G`;
}

function continueGPSStart() {
    const opts = { enableHighAccuracy: currentGPSMode === 'HIGH_FREQ', timeout: 5000, maximumAge: 0 }; 
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    wID = navigator.geolocation.watchPosition(updateDisp, (error) => {
        console.warn(`ERREUR GPS(${error.code}): ${error.message}`);
    }, opts);
    if ($('freq-select')) $('freq-select').value = currentGPSMode; 
}

function requestIMUPermissionAndStart() {
    if (typeof DeviceOrientationEvent !== 'undefined' && typeof DeviceOrientationEvent.requestPermission === 'function') {
        DeviceOrientationEvent.requestPermission()
            .then(permissionState => {
                if (permissionState === 'granted') {
                    window.addEventListener('devicemotion', handleDeviceMotion, true);
                    window.addEventListener('deviceorientation', handleDeviceOrientation, true);
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
    $('toggle-gps-btn').textContent = '⏸️ PAUSE GPS';
    $('toggle-gps-btn').style.backgroundColor = '#dc3545';
}
function stopGPS() {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
    }
    $('toggle-gps-btn').textContent = '▶️ MARCHE GPS';
    $('toggle-gps-btn').style.backgroundColor = '#28a745';
}

let map = null;
let marker = null;

// --- FONCTIONS SECONDAIRES (Astro, Météo, Carte, Horloge) ---

function getEOT(date) { 
    // ... (Logique EOT)
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

    // Calcul de l'heure Minecraft
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
    
    // Note: l'élément 'clock-status' n'est pas dans le HTML, on le désactive pour éviter une erreur
    // if ($('clock-status')) $('clock-status').textContent = `MC: ${mcTimeStr}`; 
    
    let clockRotation = (mcTimeMs / MC_DAY_MS) * 360; 
    const sunEl = $('sun-element');
    // Note: On retire la transformation translate du CSS si elle n'est pas nécessaire pour la rotation
    if (sunEl) sunEl.style.transform = `translate(-50%, -120%) rotate(${clockRotation}deg)`; 

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
    // Rotation basée sur l'azimut de la lune (position par rapport au Nord)
    if (moonEl) moonEl.style.transform = `translate(-50%, -120%) rotate(${(moonPos.azimuth + Math.PI) * R2D}deg)`; 
    
    // Note: l'élément 'sun-elevation' n'est pas dans le HTML
    // $('sun-elevation').textContent = `${(pos.altitude * R2D).toFixed(2)} °`;
    $('moon-phase-display').textContent = `Phase: ${(moonIllumination.phase * 100).toFixed(1)}%`;
}

function getWeather() {
    if (lat === 0 || lon === 0) return;
    if (OWM_API_KEY === "VOTRE_CLE_API_OPENWEATHERMAP") return; 
    
    const url = `https://api.openweathermap.org/data/2.5/weather?lat=${lat}&lon=${lon}&appid=${OWM_API_KEY}&units=metric`;
    fetch(url)
        .then(response => response.json())
        .then(data => {
            if (data.main) {
                $('temp-air').textContent = `${data.main.temp.toFixed(1)} °C`;
                $('pressure').textContent = `${data.main.pressure.toFixed(0)} hPa`;
                $('wind-speed-ms').textContent = `${data.wind.speed.toFixed(1)} m/s`;
            }
        })
        .catch(err => console.error("Erreur de récupération météo :", err));
}

function initMap() {
    if (typeof L === 'undefined') return;
    map = L.map('map-container').setView([0, 0], 2);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
    }).addTo(map);
    marker = L.marker([0, 0]).addTo(map).bindPopup("Position Actuelle");
}
function updateMap(latitude, longitude) {
    if (map) {
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
        });
}

document.addEventListener('DOMContentLoaded', () => {
    loadPrecisionRecords();
    initMap();
    syncH(); 
    
    // Boucle de mise à jour DOM/Temps (toutes les secondes)
    domID = setInterval(() => {
        const now = getCDate();
        if (now) {
            $('local-time').textContent = now.toLocaleTimeString();
            // $('date-display').textContent = now.toLocaleDateString();
            $('time-elapsed').textContent = sTime ? ((now.getTime() - sTime.getTime()) / 1000).toFixed(2) + ' s' : '0.00 s';
            $('time-moving').textContent = timeMoving.toFixed(2) + ' s';
        }
        
        // SunCalc doit être chargé pour cette fonction
        if (lat !== 0 && lon !== 0 && typeof SunCalc !== 'undefined') updateAstro(lat, lon); 
    }, 1000); 
    
    // Boucle de mise à jour Météo (toutes les 30 secondes)
    weatherID = setInterval(getWeather, 30000); 

    // --- ÉCOUTEURS D'ÉVÉNEMENTS ---
    
    $('toggle-gps-btn').addEventListener('click', () => wID === null ? startGPS() : stopGPS() );
    
    $('emergency-stop-btn').addEventListener('click', () => {
        emergencyStopActive = !emergencyStopActive;
        $('emergency-stop-btn').textContent = emergencyStopActive ? '🛑 Arrêt d\'urgence: ACTIF 🔴' : '🛑 Arrêt d\'urgence: INACTIF 🟢';
        $('emergency-stop-btn').style.backgroundColor = emergencyStopActive ? '#f8d7da' : '#dc3545';
        if (emergencyStopActive) stopGPS();
    });
    
    $('nether-toggle-btn').addEventListener('click', () => {
        netherMode = !netherMode;
        $('mode-nether').textContent = netherMode ? `ACTIF (1:${1/NETHER_RATIO})` : 'DÉSACTIVÉ (1:1)';
    });

    $('reset-dist-btn').addEventListener('click', () => { distM = 0; timeMoving = 0; });
    $('reset-max-btn').addEventListener('click', () => { maxSpd = 0; maxGForce = 0; savePrecisionRecords(); });
    
    const toggleModeBtn = $('toggle-mode-btn');
    if (toggleModeBtn) {
        toggleModeBtn.addEventListener('click', () => {
            document.body.classList.toggle('dark-mode');
            toggleModeBtn.classList.toggle('active');
        });
    }

    $('freq-select').addEventListener('change', (e) => {
        currentGPSMode = e.target.value;
        if (wID !== null) {
            stopGPS();
            startGPS();
        }
    });
    
    $('environment-select').addEventListener('change', () => {
        // La simple sélection met à jour R dans getKalmanR()
    });
});
