// =================================================================
// GNSS/IMU Dashboard JS (V11.6 - COMPLET & SYNC HYBRIDE)
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
const NTP_SERVER = 'https://worldtimeapi.org/api/utc'; // Serveur de temps

// --- ÉTAT GLOBAL DU SYSTÈME ---
let DEVICE_MASS_KG = 70.000;
let WATCH_ID = null;
let LAST_TIMESTAMP = 0;
let TOTAL_DISTANCE_M = 0;
let TOTAL_MOVING_TIME_MS = 0;
let TOTAL_ELAPSED_TIME_MS = 0;
let MAX_SPEED_MS = 0;
let LAST_KNOWN_SYNC_UTC;
let IS_NTP_SYNCED = false; // NOUVEAU: Suit l'état de la synchro
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
// [Fonctions EKF_Predict et EKF_Update - Inchangées]
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
            maxZoom: 19, attribution: '© OpenStreetMap'
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
// 3. TEMPS (SYNCHRO HYBRIDE) & ASTRO (SunCalc)
// =================================================================

async function attemptNTPTimeSync() {
    // Tente de contacter l'API (mode EN LIGNE)
    try {
        const response = await fetch(NTP_SERVER);
        if (!response.ok) throw new Error('NTP fetch failed');
        
        const data = await response.json();
        LAST_KNOWN_SYNC_UTC = new Date(data.utc_datetime);
        IS_NTP_SYNCED = true;
        console.log("NTP Sync Success (EN LIGNE):", LAST_KNOWN_SYNC_UTC);

    } catch (error) {
        // Mode HORS LIGNE ou erreur réseau
        console.warn("NTP Sync Failed (HORS LIGNE/Erreur):", error.message);
        LAST_KNOWN_SYNC_UTC = new Date(); // Fallback sur l'horloge locale
        IS_NTP_SYNCED = false;
    }
}

function updateLocalTime() {
    const now = new Date();
    const utcTime = now.toLocaleTimeString('fr-FR', {
        timeZone: 'UTC', hour: '2-digit', minute: '2-digit', second: '2-digit'
    });
    
    // CORRECTION : Affiche le statut de la synchro
    $('local-time').textContent = utcTime + (IS_NTP_SYNCED ? ' UTC (Synchro)' : ' UTC (Horloge Interne)');
    
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
    const sunAzimuthDeg = (sunPos.azimuth * 180 / Math.PI + 180) % 360;

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
    $('moon-times').textContent = times.moonrise ? times.moonrise.toLocaleTimeString('fr-FR') : 'N/D';
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
    if (emergencyStopActive) return; 

    const coords = position.coords;
    const timestamp = position.timestamp;
    const dt = (timestamp - LAST_TIMESTAMP) / 1000 || 0; 
    if (dt <= 0) return; 
    
    LAST_TIMESTAMP = timestamp;
    TOTAL_ELAPSED_TIME_MS += dt * 1000;

    const lat = coords.latitude;
    const lng = coords.longitude;
    const speedRaw = coords.speed || 0; 
    const accuracy = coords.accuracy || EKF_State.R; 

    EKF_Predict(dt);
    EKF_Update(speedRaw, accuracy);
    const speedStable = EKF_State.velocity;
    
    if (speedStable > 0.1) { 
        TOTAL_DISTANCE_M += speedStable * dt;
        TOTAL_MOVING_TIME_MS += dt * 1000;
        MAX_SPEED_MS = Math.max(MAX_SPEED_MS, speedStable);
    }
    
    updateMap(lat, lng);
    updateAstronomy(lat, lng);
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
        requestDeviceMotionPermission();
    }
}

// =================================================================
// 6. MISE À JOUR DU DOM (Affichage)
// =================================================================

function updateDisplay(coords, speedRaw, speedStable) {
    const speedStableKmH = speedStable * 3.6;
    const maxSpeedKmH = MAX_SPEED_MS * 3.6;
    const displayDistanceM = netherMode ? (TOTAL_DISTANCE_M / NETHER_RATIO) : TOTAL_DISTANCE_M;
    const displayDistanceKm = displayDistanceM / 1000;
    
    // Vitesse & Distance
    $('speed-stable').textContent = speedStableKmH.toFixed(2) + ' km/h';
    $('speed-raw-ms').textContent = speedRaw.toFixed(2) + ' m/s';
    $('speed-3d-inst').textContent = (speedRaw * 3.6).toFixed(2) + ' km/h';
    $('speed-max').textContent = maxSpeedKmH.toFixed(5) + ' km/h';
    $('distance-total-km').textContent = displayDistanceKm.toFixed(3) + ' km | ' + displayDistanceM.toFixed(2) + ' m';
    $('speed-stable-ms').textContent = `${speedStable.toFixed(2)} m/s | ${(speedStable * 1e6).toFixed(0)} µm/s`;
    
    // Moyenne
    const avgSpeed = (TOTAL_MOVING_TIME_MS > 0) ? (TOTAL_DISTANCE_M / (TOTAL_MOVING_TIME_MS / 1000)) : 0;
    $('speed-avg-moving').textContent = (avgSpeed * 3.6).toFixed(5) + ' km/h';

    // GPS & Physique
    $('latitude').textContent = coords.latitude.toFixed(6);
    $('longitude').textContent = coords.longitude.toFixed(6);
    $('altitude-gps').textContent = coords.altitude ? coords.altitude.toFixed(2) + ' m' : 'N/A';
    $('gps-precision').textContent = coords.accuracy ? coords.accuracy.toFixed(2) + ' m' : 'N/A';
    $('vertical-speed').textContent = '0.00 m/s'; 
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

async function initGeolocation() {
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
        if (WATCH_ID) startGPS(); 
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

    // --- DÉMARRAGE DES SERVICES ---
    
    // Tente la synchronisation NTP (en ligne) avant de démarrer l'horloge
    await attemptNTPTimeSync(); 

    // Démarrage de la mise à jour de l'heure
    setInterval(updateLocalTime, 1000);
    updateLocalTime(); // Premier appel
    
    // Démarrage de l'astro (position par défaut)
    updateAstronomy(MAP_DEFAULT_LAT, MAP_DEFAULT_LNG); 
    setInterval(() => updateAstronomy(MAP_DEFAULT_LAT, MAP_DEFAULT_LNG), 60000); 
    
    // Tentative de démarrage des capteurs IMU
    requestDeviceMotionPermission();
}

// Lancement de l'application
document.addEventListener('DOMContentLoaded', initGeolocation);
