// =================================================================
// GNSS/IMU Dashboard JS (V11.4 COMPLET)
// Gère EKF, IMU (Réel), GPS, Astro (SunCalc), Temps (UTC), 
// Carte (Leaflet) et tous les contrôles DOM.
// =================================================================

// --- FONCTION UTILITAIRE ---
const $ = (id) => document.getElementById(id);

// --- CONSTANTES GLOBALES ---
const C_LIGHT = 299792458; // m/s
const C_SOUND = 343;       // m/s
const G_ACCEL = 9.80665;   // m/s²
const MAP_DEFAULT_LAT = 48.8566; // Paris
const MAP_DEFAULT_LNG = 2.3522;
const NETHER_RATIO = 8;

// --- ÉTAT GLOBAL DU SYSTÈME ---
let DEVICE_MASS_KG = 70.000;
let WATCH_ID = null;
let LAST_TIMESTAMP = 0;
let TOTAL_DISTANCE_M = 0;
let TOTAL_MOVING_TIME_MS = 0;
let TOTAL_ELAPSED_TIME_MS = 0;
let MAX_SPEED_MS = 0;
let LAST_KNOWN_SYNC_UTC;
let emergencyStopActive = false;
let netherMode = false;
let currentGPSMode = 'HIGH_FREQ';

// --- ÉTAT IMU (Capteurs Réels) ---
let ACCEL_LATERAL_IMU = 0; 
let ACCEL_LONG_IMU = 0;    
let IMU_IS_ACTIVE = false;

// --- ÉTAT CARTE (Leaflet) ---
let map = null;
let marker = null;
let polyline = null;
let trackPoints = [];

// --- ÉTAT EKF (Filtre de Kalman pour stabiliser la vitesse GPS) ---
let EKF_State = {
    P: [[100, 0], [0, 100]], 
    Q: 1, 
    R: 5, 
    velocity: 0, 
    position_error: 0
};

// =================================================================
// 1. LOGIQUE EKF (Stabilisation Vitesse GPS)
// =================================================================

function EKF_Predict(dt) {
    const dt2 = dt * dt;
    const P = EKF_State.P;
    const Q = EKF_State.Q;

    P[0][0] = P[0][0] + dt * P[1][0] + dt * P[0][1] + dt2 * P[1][1] + Q * dt2 * dt;
    P[0][1] = P[0][1] + dt * P[1][1] + Q * dt2;
    P[1][0] = P[1][0] + dt * P[1][1] + Q * dt2;
    P[1][1] = P[1][1] + Q * dt;
    
    EKF_State.P = P;
}

function EKF_Update(measurement_velocity, measurement_accuracy) {
    const P = EKF_State.P;
    const velocity_current = EKF_State.velocity;
    const R = measurement_accuracy * measurement_accuracy; 
    const Y = measurement_velocity - velocity_current;
    const S = P[1][1] + R; 
    const K_pos = P[0][1] / S;
    const K_vel = P[1][1] / S; 

    EKF_State.velocity = velocity_current + K_vel * Y;
    
    P[0][0] = P[0][0] - K_pos * P[0][1];
    P[0][1] = P[0][1] - K_pos * P[1][1];
    P[1][0] = P[1][0] - K_vel * P[1][0];
    P[1][1] = P[1][1] - K_vel * P[1][1];
    
    EKF_State.P = P;
    EKF_State.position_error = Math.sqrt(P[0][0]); 
}

// =================================================================
// 2. CARTE (Leaflet)
// =================================================================

function initMap() {
    if ($('map') && typeof L !== 'undefined') {
        map = L.map('map').setView([MAP_DEFAULT_LAT, MAP_DEFAULT_LNG], 13);
        
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            maxZoom: 19,
            attribution: '© OpenStreetMap'
        }).addTo(map);

        marker = L.marker([MAP_DEFAULT_LAT, MAP_DEFAULT_LNG]).addTo(map);
        polyline = L.polyline([], {color: 'blue'}).addTo(map);
    }
}

function updateMap(lat, lng) {
    if (!map || !marker) return;

    const newLatLng = [lat, lng];
    marker.setLatLng(newLatLng);
    
    trackPoints.push(newLatLng);
    polyline.setLatLngs(trackPoints);

    map.setView(newLatLng);
    if (trackPoints.length > 1) {
        map.fitBounds(polyline.getBounds(), {padding: [50, 50]});
    }
}

// =================================================================
// 3. TEMPS (UTC Hors Ligne) & ASTRO (SunCalc)
// =================================================================

function updateLocalTime() {
    const now = new Date();
    
    const utcTime = now.toLocaleTimeString('fr-FR', {
        timeZone: 'UTC', hour: '2-digit', minute: '2-digit', second: '2-digit'
    });
    
    $('local-time').textContent = utcTime + ' UTC (Horloge interne)';
    $('date-display').textContent = now.toLocaleDateString('fr-FR', {
        year: 'numeric', month: 'long', day: 'numeric'
    });
}

function updateAstronomy(latitude, longitude) {
    const now = new Date();
    if (typeof SunCalc === 'undefined') return;

    const sunPos = SunCalc.getPosition(now, latitude, longitude);
    const moonIllumination = SunCalc.getMoonIllumination(now);
    const times = SunCalc.getTimes(now, latitude, longitude);

    const eclipticLongDeg = sunPos.eclipticLng * 180 / Math.PI;
    const sunElevationDeg = sunPos.altitude * 180 / Math.PI;
    const sunAzimuthDeg = (sunPos.azimuth * 180 / Math.PI + 180) % 360; // Azimut 0-360°

    let eotMinutes = 'N/D';
    if (times.solarNoon) {
        const localNoon = new Date(now.getFullYear(), now.getMonth(), now.getDate(), 12, 0, 0).getTime();
        const solarNoonTime = times.solarNoon.getTime();
        
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

// =================================================================
// 4. CAPTEURS IMU (Accélération Réelle)
// =================================================================

function requestDeviceMotionPermission() {
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
        startDeviceMotionTracking();
    }
}

function startDeviceMotionTracking() {
    window.addEventListener('devicemotion', (event) => {
        const accelerationSource = event.acceleration || event.accelerationIncludingGravity;

        if (accelerationSource) {
            ACCEL_LONG_IMU = accelerationSource.x || 0;
            ACCEL_LATERAL_IMU = accelerationSource.y || 0;
            IMU_IS_ACTIVE = true;
            updateDynamicIMUDisplay();
        }
    }, true);
}

function updateDynamicIMUDisplay() {
    $('current-mass').textContent = DEVICE_MASS_KG.toFixed(3);
    const accelLat = ACCEL_LATERAL_IMU;
    const forceCentrifugal = DEVICE_MASS_KG * Math.abs(accelLat); 
    const forceG = Math.abs(accelLat) / G_ACCEL; 

    $('accel-lat').textContent = accelLat.toFixed(3) + ' m/s² (MESURÉE)';
    $('force-centrifugal').textContent = forceCentrifugal.toFixed(2) + ' N (MESURÉE)';
    $('force-centrifugal-g').textContent = forceG.toFixed(2) + ' G (MESURÉE)';
    $('accel-long').textContent = ACCEL_LONG_IMU.toFixed(3) + ' m/s² (MESURÉE)';
    
    const warningNote = document.querySelector('.warning-note');
    if (warningNote) warningNote.style.display = 'none';
}

// =================================================================
// 5. GPS & LOGIQUE PRINCIPALE
// =================================================================

function gpsSuccess(position) {
    if (emergencyStopActive) return; // Arrêt d'urgence

    const coords = position.coords;
    const timestamp = position.timestamp;
    const dt = (timestamp - LAST_TIMESTAMP) / 1000 || 0; 
    if (dt <= 0) return; // Ignorer si le timestamp est invalide ou dupliqué
    
    LAST_TIMESTAMP = timestamp;
    TOTAL_ELAPSED_TIME_MS += dt * 1000;

    const lat = coords.latitude;
    const lng = coords.longitude;
    const speedRaw = coords.speed || 0; 
    const accuracy = coords.accuracy || EKF_State.R; 

    // 1. EKF (Stabilisation Vitesse)
    EKF_Predict(dt);
    EKF_Update(speedRaw, accuracy);
    const speedStable = EKF_State.velocity;
    
    // 2. Distance & Temps de Mouvement
    if (speedStable > 0.1) { // Seuil de mouvement
        TOTAL_DISTANCE_M += speedStable * dt;
        TOTAL_MOVING_TIME_MS += dt * 1000;
        MAX_SPEED_MS = Math.max(MAX_SPEED_MS, speedStable);
    }
    
    // 3. Mise à jour Carte & Astro
    updateMap(lat, lng);
    updateAstronomy(lat, lng);
    
    // 4. Mise à jour Affichage
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
            enableHighAccuracy: (currentGPSMode === 'HIGH_FREQ'),
            timeout: 5000,
            maximumAge: 0
        };
        WATCH_ID = navigator.geolocation.watchPosition(gpsSuccess, gpsError, options);
        $('toggle-gps-btn').textContent = '⏸️ PAUSE GPS';
        
        // Demander les capteurs IMU au démarrage du GPS
        requestDeviceMotionPermission();
    }
}

// =================================================================
// 6. MISE À JOUR DU DOM
// =================================================================

function updateDisplay(coords, speedRaw, speedStable) {
    const speedStableKmH = speedStable * 3.6;
    const maxSpeedKmH = MAX_SPEED_MS * 3.6;
    
    // Application du ratio Nether si actif
    const displayDistanceM = netherMode ? (TOTAL_DISTANCE_M / NETHER_RATIO) : TOTAL_DISTANCE_M;
    const displayDistanceKm = displayDistanceM / 1000;
    
    // Vitesse & Distance
    $('speed-stable').textContent = speedStableKmH.toFixed(2) + ' km/h';
    $('speed-raw-ms').textContent = speedRaw.toFixed(2) + ' m/s';
    $('speed-max').textContent = maxSpeedKmH.toFixed(5) + ' km/h';
    $('distance-total-km').textContent = displayDistanceKm.toFixed(3) + ' km | ' + displayDistanceM.toFixed(2) + ' m';
    $('speed-stable-ms').textContent = `${speedStable.toFixed(2)} m/s | ${(speedStable * 1e6).toFixed(0)} µm/s`;

    // Si l'IMU n'est PAS actif, afficher l'avertissement et les données EKF
    if (!IMU_IS_ACTIVE) {
         // Fallback EKF (estimation)
         $('accel-long').textContent = 'N/A (Calculé)';
         const warningNote = document.querySelector('.warning-note');
         if (warningNote) warningNote.style.display = 'block';
    }
    
    // GPS & Physique
    $('latitude').textContent = coords.latitude.toFixed(6);
    $('longitude').textContent = coords.longitude.toFixed(6);
    $('altitude-gps').textContent = coords.altitude ? coords.altitude.toFixed(2) + ' m' : 'N/A';
    $('gps-precision').textContent = coords.accuracy ? coords.accuracy.toFixed(2) + ' m' : 'N/A';
    $('vertical-speed').textContent = coords.altitudeAccuracy ? (coords.altitudeAccuracy * 0).toFixed(2) + ' m/s' : '0.00 m/s'; // Placeholder, GPS seul ne donne pas vSpeed fiable
    $('underground-status').textContent = (coords.altitude && coords.altitude < -50) ? 'OUI' : 'Non';

    // Cosmics
    $('perc-speed-sound').textContent = (speedStable / C_SOUND * 100).toFixed(2) + ' %';
    $('perc-speed-c').textContent = (speedStable / C_LIGHT * 100).toExponential(2) + ' %';
    $('distance-cosmic').textContent = (displayDistanceM / C_LIGHT).toExponential(2) + ' s lumière';
    
    // Précision EKF
    const errorPerc = Math.sqrt(EKF_State.P[1][1]) / speedStable * 100;
    $('speed-error-perc').textContent = isFinite(errorPerc) ? errorPerc.toFixed(1) + ' %' : 'N/A';
    
    // Temps
    $('time-moving').textContent = (TOTAL_MOVING_TIME_MS / 1000).toFixed(2) + ' s';
    $('time-elapsed').textContent = (TOTAL_ELAPSED_TIME_MS / 1000).toFixed(2) + ' s';
}

function resetDisp(fullReset = true) {
    if (fullReset) {
        TOTAL_DISTANCE_M = 0;
        MAX_SPEED_MS = 0;
        TOTAL_MOVING_TIME_MS = 0;
        TOTAL_ELAPSED_TIME_MS = 0;
        trackPoints = []; 
        if (polyline) polyline.setLatLngs(trackPoints);
        if (map) map.setView([MAP_DEFAULT_LAT, MAP_DEFAULT_LNG], 13);
    }
    
    EKF_State.velocity = 0;
    EKF_State.P = [[100, 0], [0, 100]];
    
    // Reset DOM
    $('speed-stable').textContent = '0.00 km/h';
    $('speed-max').textContent = '0.00000 km/h';
    $('distance-total-km').textContent = '0.000 km | 0.00 m';
    $('time-moving').textContent = '0.00 s';
    $('time-elapsed').textContent = '0.00 s';
}

// =================================================================
// 7. INITIALISATION (DOMContentLoaded)
// =================================================================

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

    // --- ÉCOUTEURS DE BOUTONS (Corrigé) ---
    $('toggle-gps-btn').addEventListener('click', startGPS);
    
    $('freq-select').addEventListener('change', (e) => {
        currentGPSMode = e.target.value;
        if (WATCH_ID) startGPS(); // Redémarre le GPS avec la nouvelle fréquence
    });
    
    $('emergency-stop-btn').addEventListener('click', () => {
        emergencyStopActive = !emergencyStopActive;
        const btn = $('emergency-stop-btn');
        btn.textContent = emergencyStopActive ? '🛑 Arrêt d'urgence: ACTIF 🔴' : '🛑 Arrêt d'urgence: INACTIF 🟢';
        if (emergencyStopActive && WATCH_ID) {
            navigator.geolocation.clearWatch(WATCH_ID);
            WATCH_ID = null;
            $('toggle-gps-btn').textContent = '▶️ MARCHE GPS';
        }
    });
    
    $('nether-toggle-btn').addEventListener('click', () => {
        netherMode = !netherMode;
        $('mode-nether').textContent = netherMode ? 'ACTIVÉ (1:8) 🔥' : 'DÉSACTIVÉ (1:1)';
    });
    
    $('reset-dist-btn').addEventListener('click', () => {
        TOTAL_DISTANCE_M = 0;
        TOTAL_MOVING_TIME_MS = 0;
        trackPoints = [];
        if (polyline) polyline.setLatLngs(trackPoints);
        $('distance-total-km').textContent = '0.000 km | 0.00 m';
    });
    
    $('reset-max-btn').addEventListener('click', () => {
        MAX_SPEED_MS = 0;
        $('speed-max').textContent = '0.00000 km/h';
    });

    $('reset-all-btn').addEventListener('click', () => {
        if (confirm("Réinitialiser toutes les données de session ?")) {
            resetDisp(true);
        }
    });

    // Initialisation du temps UTC
    LAST_KNOWN_SYNC_UTC = new Date().toUTCString();
    setInterval(updateLocalTime, 1000);
    updateLocalTime();
    
    // Initialisation de l'astro
    updateAstronomy(MAP_DEFAULT_LAT, MAP_DEFAULT_LNG); 
    setInterval(() => updateAstronomy(MAP_DEFAULT_LAT, MAP_DEFAULT_LNG), 60000); 
    
    // Tentative de démarrage des capteurs IMU
    requestDeviceMotionPermission();
}

// Lancement de l'application
document.addEventListener('DOMContentLoaded', initGeolocation);
