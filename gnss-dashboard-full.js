// =================================================================
// GNSS/IMU Dashboard JS - UTILS & EKF CORE LOGIC (Ancienne PART 1)
// =================================================================

// Fonction utilitaire pour récupérer les éléments DOM
const $ = (id) => document.getElementById(id);

// --- CONSTANTES ---
const C_LIGHT = 299792458; // m/s
const C_SOUND = 343;       // m/s (approx à 20°C)
const G_ACCEL = 9.80665;   // m/s² (Gravité standard)
const MAP_DEFAULT_LAT = 48.8566; // Position par défaut (Paris)
const MAP_DEFAULT_LNG = 2.3522;

// --- EKF STATE (Filtre de Kalman Étendu - Pour la stabilisation GPS) ---
let EKF_State = {
    x: 0, // Position (dans le modèle simplifié)
    P: [[100, 0], [0, 100]], // Matrice de covariance
    Q: 1, // Bruit de processus (Dynamique)
    R: 5, // Bruit de mesure (GPS/Brute)
    velocity: 0, // Vitesse stable (m/s)
    position_error: 0 // Erreur de position estimée
};

// --- GPS/IMU GLOBAL STATE ---
let DEVICE_MASS_KG = 70.000;
let WATCH_ID = null;
let LAST_TIMESTAMP = 0;
let TOTAL_DISTANCE_M = 0;
let TOTAL_MOVING_TIME_MS = 0;
let MAX_SPEED_MS = 0;
let ACCEL_LATERAL_IMU = 0; // Mesure IMU (Réel)
let ACCEL_LONG_IMU = 0;    // Mesure IMU (Réel)
let IMU_IS_ACTIVE = false;
let LAST_KNOWN_SYNC_UTC; // Pour la référence de temps hors ligne


// EKF CORE FUNCTION - Prédiction
function EKF_Predict(dt) {
    const dt2 = dt * dt;
    const P = EKF_State.P;
    const Q = EKF_State.Q;

    // Mise à jour de la Covariance P
    P[0][0] = P[0][0] + dt * P[1][0] + dt * P[0][1] + dt2 * P[1][1] + Q * dt2 * dt;
    P[0][1] = P[0][1] + dt * P[1][1] + Q * dt2;
    P[1][0] = P[1][0] + dt * P[1][1] + Q * dt2;
    P[1][1] = P[1][1] + Q * dt;
    
    EKF_State.P = P;
}

// EKF CORE FUNCTION - Mise à Jour (Correction)
function EKF_Update(measurement_velocity, measurement_accuracy) {
    const P = EKF_State.P;
    const velocity_current = EKF_State.velocity;
    const R = measurement_accuracy * measurement_accuracy; 

    // 1. Innovation (Différence entre mesure brute et estimation actuelle)
    const Y = measurement_velocity - velocity_current;

    // 2. Innovation Covariance (S)
    const S = P[1][1] + R; 

    // 3. Gain de Kalman (K)
    const K_pos = P[0][1] / S;
    const K_vel = P[1][1] / S; 

    // 4. Mise à jour de l'État (Vitesse stable)
    EKF_State.velocity = velocity_current + K_vel * Y;
    
    // 5. Mise à jour de la Covariance
    P[0][0] = P[0][0] - K_pos * P[0][1];
    P[0][1] = P[0][1] - K_pos * P[1][1];
    P[1][0] = P[1][0] - K_vel * P[1][0];
    P[1][1] = P[1][1] - K_vel * P[1][1];
    
    EKF_State.P = P;
    EKF_State.position_error = Math.sqrt(P[0][0]); 
}

// =================================================================
// GNSS/IMU Dashboard JS - DOM, SENSORS, TIME & ASTRO (Ancienne PART 2)
// =================================================================

// --- MAP & TRACKING ---
let map = null;
let marker = null;
let polyline = null;
let trackPoints = [];

function initMap() {
    if ($('map') && typeof L !== 'undefined') {
        map = L.map('map').setView([MAP_DEFAULT_LAT, MAP_DEFAULT_LNG], 13);
        
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            maxZoom: 19,
            attribution: '© OpenStreetMap'
        }).addTo(map);

        marker = L.marker([MAP_DEFAULT_LAT, MAP_DEFAULT_LNG]).addTo(map);
        polyline = L.polyline([], {color: 'red'}).addTo(map);
    }
}

function updateMap(lat, lng, accuracy) {
    if (!map || !marker) return;

    const newLatLng = [lat, lng];
    marker.setLatLng(newLatLng);
    
    trackPoints.push(newLatLng);
    polyline.setLatLngs(trackPoints);

    map.setView(newLatLng);
    if (trackPoints.length > 1) {
        // Ajuste le zoom pour voir tout le tracé
        map.fitBounds(polyline.getBounds(), {padding: [50, 50]});
    }
}


// --- TIME SYNCHRONIZATION (OFFLINE GMT/UTC) ---

function updateLocalTime() {
    const now = new Date();
    
    // Afficher l'heure en format UTC (équivalent GMT)
    const utcTime = now.toLocaleTimeString('fr-FR', {
        timeZone: 'UTC', 
        hour: '2-digit', 
        minute: '2-digit', 
        second: '2-digit'
    });
    
    // Affichage de l'heure
    $('local-time').textContent = utcTime + ' UTC (Horloge interne)';
    
    // Affichage de la date locale
    $('date-display').textContent = now.toLocaleDateString('fr-FR', {
        year: 'numeric', month: 'long', day: 'numeric'
    });
}

// --- IMU / DEVICEMOTION (Mesure d'Accélération Réelle) ---

function requestDeviceMotionPermission() {
    // Demander la permission sur iOS 13+
    if (typeof DeviceOrientationEvent !== 'undefined' && typeof DeviceOrientationEvent.requestPermission === 'function') {
        DeviceOrientationEvent.requestPermission()
            .then(permissionState => {
                if (permissionState === 'granted') {
                    startDeviceMotionTracking();
                } else {
                    document.querySelector('.warning-note').style.display = 'block'; 
                }
            })
            .catch(console.error);
    } else {
        // Autres navigateurs (Android, etc.)
        startDeviceMotionTracking();
    }
}

function startDeviceMotionTracking() {
    window.addEventListener('devicemotion', (event) => {
        const accelerationSource = event.acceleration || event.accelerationIncludingGravity;

        if (accelerationSource) {
            // Axe X (Longitudinal)
            ACCEL_LONG_IMU = accelerationSource.x || 0;
            // Axe Y (Latéral)
            ACCEL_LATERAL_IMU = accelerationSource.y || 0;

            IMU_IS_ACTIVE = true;
            updateDynamicIMUDisplay();
        }
    }, true);
}

function updateDynamicIMUDisplay() {
    // 1. Mise à jour de la masse actuelle
    $('current-mass').textContent = DEVICE_MASS_KG.toFixed(3);

    // 2. Accélération latérale RÉELLE (IMU)
    const accelLat = ACCEL_LATERAL_IMU;

    // 3. Force Centrifuge (Calculé à partir de la MESURE réelle)
    const forceCentrifugal = DEVICE_MASS_KG * Math.abs(accelLat); 
    const forceG = Math.abs(accelLat) / G_ACCEL; 

    // Mise à jour des affichages
    $('accel-lat').textContent = accelLat.toFixed(3) + ' m/s² (MESURÉE)';
    $('force-centrifugal').textContent = forceCentrifugal.toFixed(2) + ' N (MESURÉE)';
    $('force-centrifugal-g').textContent = forceG.toFixed(2) + ' G (MESURÉE)';
    $('accel-long').textContent = ACCEL_LONG_IMU.toFixed(3) + ' m/s² (MESURÉE)';
    
    // Masquer l'avertissement une fois que l'IMU est fonctionnel
    const warningNote = document.querySelector('.warning-note');
    if (warningNote) warningNote.style.display = 'none';
}


// --- ASTRO / SUNCALC (Méthode PlanetCalc/Standard) ---

function updateAstronomy(latitude, longitude) {
    const now = new Date();
    
    if (typeof SunCalc === 'undefined') {
        return;
    }

    const sunPos = SunCalc.getPosition(now, latitude, longitude);
    const moonIllumination = SunCalc.getMoonIllumination(now);
    const times = SunCalc.getTimes(now, latitude, longitude);

    const eclipticLongDeg = sunPos.eclipticLng * 180 / Math.PI;
    const sunElevationDeg = sunPos.altitude * 180 / Math.PI;
    const sunAzimuthDeg = sunPos.azimuth * 180 / Math.PI + 180;

    let eotMinutes = 'N/D';
    if (times.solarNoon) {
        const localNoon = new Date(now.getFullYear(), now.getMonth(), now.getDate(), 12, 0, 0).getTime();
        const solarNoonTime = times.solarNoon.getTime();
        
        // Convertir l'heure de midi solaire en heure locale basée sur le fuseau horaire du système
        const solarNoonLocalTime = new Date(solarNoonTime + now.getTimezoneOffset() * 60000); 
        const eotOffsetMs = solarNoonLocalTime.getTime() - localNoon;
        eotMinutes = (eotOffsetMs / (1000 * 60)).toFixed(2);
    }
    
    $('sun-elevation').textContent = sunElevationDeg.toFixed(2) + ' °';
    $('sun-azimuth').textContent = sunAzimuthDeg.toFixed(2) + ' °';
    $('ecliptic-long').textContent = eclipticLongDeg.toFixed(2) + ' °';
    $('eot-min').textContent = eotMinutes + ' min';
    
    $('lunar-phase-perc').textContent = (moonIllumination.fraction * 100).toFixed(1) + ' %';
    
    if (times.moonrise && times.moonset) {
        $('moon-times').textContent = times.moonrise.toLocaleTimeString('fr-FR') + ' / ' + times.moonset.toLocaleTimeString('fr-FR');
    } else {
         $('moon-times').textContent = 'N/D';
    }
}


// --- GPS CORE FUNCTION ---

function gpsSuccess(position) {
    const coords = position.coords;
    const timestamp = position.timestamp;
    const dt = (timestamp - LAST_TIMESTAMP) / 1000 || 0; 
    LAST_TIMESTAMP = timestamp;

    const lat = coords.latitude;
    const lng = coords.longitude;
    const speedRaw = coords.speed || 0; 
    const accuracy = coords.accuracy || EKF_State.R; 

    // 1. EKF Prediction & Update
    if (dt > 0) {
        EKF_Predict(dt);
        EKF_Update(speedRaw, accuracy);
    }
    const speedStable = EKF_State.velocity;
    
    // 2. Distance Calculation
    if (speedStable > 0.05) { 
        TOTAL_DISTANCE_M += speedStable * dt;
        TOTAL_MOVING_TIME_MS += dt * 1000;
        MAX_SPEED_MS = Math.max(MAX_SPEED_MS, speedStable);
    }
    
    // 3. Update Map & Astro
    updateMap(lat, lng, accuracy);
    updateAstronomy(lat, lng);
    
    // 4. Display Update
    updateDisplay(coords, speedRaw, speedStable);
}

function gpsError(error) {
    console.warn(`GPS ERROR(${error.code}): ${error.message}`);
    $('gps-precision').textContent = 'ERREUR GPS';
}


function startGPS() {
    if (WATCH_ID) {
        navigator.geolocation.clearWatch(WATCH_ID);
        WATCH_ID = null;
        $('toggle-gps-btn').textContent = '▶️ MARCHE GPS';
    } else {
        const options = {
            enableHighAccuracy: ($('freq-select').value === 'HIGH_FREQ'),
            timeout: 5000,
            maximumAge: 0
        };
        WATCH_ID = navigator.geolocation.watchPosition(gpsSuccess, gpsError, options);
        $('toggle-gps-btn').textContent = '⏸️ PAUSE GPS';
        
        // S'assurer que le tracking IMU démarre avec le GPS
        requestDeviceMotionPermission();
    }
}

function updateDisplay(coords, speedRaw, speedStable) {
    const speedStableKmH = speedStable * 3.6;
    const maxSpeedKmH = MAX_SPEED_MS * 3.6;
    const distanceKm = TOTAL_DISTANCE_M / 1000;
    
    // Vitesse & Distance
    $('speed-stable').textContent = speedStableKmH.toFixed(2) + ' km/h';
    $('speed-raw-ms').textContent = speedRaw.toFixed(2) + ' m/s';
    $('speed-max').textContent = maxSpeedKmH.toFixed(5) + ' km/h';
    $('distance-total-km').textContent = distanceKm.toFixed(3) + ' km | ' + TOTAL_DISTANCE_M.toFixed(2) + ' m';
    
    // Si l'IMU est actif, la fonction updateDynamicIMUDisplay prend la priorité
    if (!IMU_IS_ACTIVE) {
         // Fallback EKF
         $('accel-long').textContent = '0.000 m/s² (ESTIMÉE)';
         // Re-afficher l'avertissement car l'IMU n'est pas utilisé
         const warningNote = document.querySelector('.warning-note');
         if (warningNote) warningNote.style.display = 'block';
    }
    
    // GPS & Physique
    $('latitude').textContent = coords.latitude.toFixed(6);
    $('longitude').textContent = coords.longitude.toFixed(6);
    $('altitude-gps').textContent = coords.altitude ? coords.altitude.toFixed(2) + ' m' : 'N/A';
    $('gps-precision').textContent = coords.accuracy ? coords.accuracy.toFixed(2) + ' m' : 'N/A';
    $('vertical-speed').textContent = coords.altitude ? (coords.altitude * 0).toFixed(2) + ' m/s' : '0.00 m/s'; 

    // Cosmics
    $('perc-speed-sound').textContent = (speedStable / C_SOUND * 100).toFixed(2) + ' %';
    $('perc-speed-c').textContent = (speedStable / C_LIGHT * 100).toExponential(2) + ' %';
    $('distance-cosmic').textContent = (TOTAL_DISTANCE_M / C_LIGHT).toExponential(2) + ' s lumière';
    
    // EKF Accuracy
    const errorPerc = Math.sqrt(EKF_State.P[1][1]) / speedStable * 100;
    $('speed-error-perc').textContent = isFinite(errorPerc) ? errorPerc.toFixed(1) + ' %' : 'N/A';
    
    // Mouvement Time
    $('time-moving').textContent = (TOTAL_MOVING_TIME_MS / 1000).toFixed(2) + ' s';
}

// --- INITIALIZATION ---

function initGeolocation() {
    initMap();
    
    // Écouteur pour la masse (précision au gramme)
    const $massInput = $('mass-input');
    if ($massInput) {
        $massInput.value = DEVICE_MASS_KG.toFixed(3); 
        
        $massInput.addEventListener('change', (e) => {
            const newMass = parseFloat(e.target.value);
            if (!isNaN(newMass) && newMass >= 0.001) { 
                DEVICE_MASS_KG = newMass;
            } else {
                e.target.value = DEVICE_MASS_KG.toFixed(3); 
            }
            $('current-mass').textContent = DEVICE_MASS_KG.toFixed(3); 
            if (IMU_IS_ACTIVE) updateDynamicIMUDisplay(); 
        });
    }

    // Boutons de contrôle
    $('toggle-gps-btn').addEventListener('click', startGPS);
    $('reset-all-btn').addEventListener('click', () => { window.location.reload(); });
    // TODO: Ajouter la logique des autres boutons de contrôle ici.

    // Initialisation de la dernière synchronisation connue (GMT/UTC)
    LAST_KNOWN_SYNC_UTC = new Date().toUTCString();

    // Démarrage de la mise à jour de l'heure
    setInterval(updateLocalTime, 1000);
    updateLocalTime();
    
    // Démarrage de la mise à jour des données astronomiques 
    updateAstronomy(MAP_DEFAULT_LAT, MAP_DEFAULT_LNG); 
    setInterval(() => updateAstronomy(MAP_DEFAULT_LAT, MAP_DEFAULT_LNG), 60000); 
    
    // Tentative de démarrage immédiat des capteurs IMU
    requestDeviceMotionPermission();
}

document.addEventListener('DOMContentLoaded', initGeolocation);
