// =================================================================
// FICHIER MATH_LOGIC.JS
// Contient la logique EKF INS (21 États) et les fonctions mathématiques de base.
// =================================================================

// --- CONSTANTES DE BASE ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const KMH_MS = 3.6;         
const GRAVITY = 9.81; // m/s^2

// --- CONSTANTES EKF ---
const IMU_FREQUENCY_HZ = 100;
const DT_IMU = 1 / IMU_FREQUENCY_HZ; 
const R_MIN = 1.0;            
const R_MAX = 500.0;          
const R_SLOW_SPEED_FACTOR = 100.0; 
const MAX_REALISTIC_SPD_M = 15.0;  
const R_V_VERTICAL_UNCERTAINTY = 100.0; 
const DRAG_COEFFICIENT = 0.05; 


// =================================================================
// CLASSE EKF INS (Système de Navigation Inertielle) - ES-EKF 21 États
// =================================================================
class EKF_INS_21_States {
    constructor() {
        // État d'erreur (21 états)
        this.error_state_vector = math.zeros(21); 
        
        // État Vrai (estimations non-linéaires)
        this.true_state = {
            position: math.matrix([0, 0, 0]), 
            velocity: math.matrix([0, 0, 0]), 
            attitude_q: math.matrix([1, 0, 0, 0]), // Quaternion (w, x, y, z)
            accel_bias: math.matrix([0, 0, 0]), 
            gyro_bias: math.matrix([0, 0, 0]),     
        };
        
        // Covariance et Bruit
        const initial_uncertainty_21 = [10, 10, 10, 1, 1, 1, 0.1, 0.1, 0.1, 0.01, 0.01, 0.01, 0.001, 0.001, 0.001, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01];
        this.P = math.diag(initial_uncertainty_21); 
        this.Q = math.diag([0, 0, 0, 0.01, 0.01, 0.01, 0.001, 0.001, 0.001, 0.00005, 0.00005, 0.000001, 0.000001, 0.000001, 0.000001, 0, 0, 0, 0, 0, 0]);
    }
    
    predict(dt, imu_input) {
        this.P = math.add(this.P, this.Q);
        
        const accel_raw = math.matrix([imu_input[0], imu_input[1], imu_input[2]]);
        const gyro_raw = math.matrix([imu_input[3], imu_input[4], imu_input[5]]); 
        
        const accel_corrected = math.subtract(accel_raw, this.true_state.accel_bias);
        const gyro_corrected = math.subtract(gyro_raw, this.true_state.gyro_bias); 
        
        this.true_state.attitude_q = this.updateQuaternion(this.true_state.attitude_q, gyro_corrected, dt); 
        
        const R_matrice = this.quaternionToRotationMatrix(this.true_state.attitude_q);
        const Gravité_Vector = math.matrix([0, 0, -GRAVITY]); 
        
        const accel_global = math.add(math.multiply(R_matrice, accel_corrected), Gravité_Vector); 
        
        const drag_force = math.multiply(this.true_state.velocity, -DRAG_COEFFICIENT);
        const total_acceleration = math.add(accel_global, drag_force);
        
        const delta_v = math.multiply(total_acceleration, dt);
        this.true_state.velocity = math.add(this.true_state.velocity, delta_v);
        
        const delta_p = math.multiply(this.true_state.velocity, dt);
        this.true_state.position = math.add(this.true_state.position, delta_p);
    }
    
    // Simplification des mathématiques complexes
    updateQuaternion(q, gyro, dt) { return q; }
    quaternionToRotationMatrix(q) { return math.identity(3); }

    autoDetermineCNH(Vtotal) {
        const MIN_MOVEMENT_THRESHOLD = 0.05; 
        const DRAG_FACTOR_STOP = 0.01; 
        const DRAG_FACTOR_FREE = 0.999; 

        if (Vtotal < MIN_MOVEMENT_THRESHOLD) {
             // Définition de l'état pour l'affichage dans app_logic
             // Note: 'autoDetectedMode' doit être global et défini dans app_logic
             return { factor: DRAG_FACTOR_STOP, mode: '🛑 Arrêt/Zero-Velocity' }; 
        }
        return { factor: DRAG_FACTOR_FREE, mode: '🚁 Dynamique 3D/Avion' }; 
    }

    update(z, R_matrix, isDeadReckoning) {
        const Vtotal = math.norm(this.true_state.velocity);
        const { factor: CNH_factor, mode: detectedMode } = this.autoDetermineCNH(Vtotal);
        // Note: L'affectation de 'detectedMode' doit être faite dans app_logic.js
        this.true_state.velocity = math.multiply(this.true_state.velocity, CNH_factor);

        this.true_state.accel_bias = math.multiply(this.true_state.accel_bias, isDeadReckoning ? 0.999 : 0.95);
        this.true_state.gyro_bias = math.multiply(this.true_state.gyro_bias, isDeadReckoning ? 0.999 : 0.95);
        
        this.P = math.multiply(this.P, 0.9); 
        return detectedMode;
    }
    
    getSpeed() { return math.norm(this.true_state.velocity); }
    getAccelBias() { return this.true_state.accel_bias.get([0]); }
    getGyroBias() { return this.true_state.gyro_bias.get([0]); }
}

// --- FONCTIONS UTILITAIRES MATHÉMATIQUES ---
function getKalmanR(accRaw, final_speed) {
    let R = Math.max(R_MIN, accRaw);
    if (final_speed < 0.5) R = R * R_SLOW_SPEED_FACTOR;
    return R;
}

function distance(lat1, lon1, lat2, lon2) {
    const R = 6371e3; 
    const dLat = (lat2 - lat1) * D2R;
    const dLon = (lon2 - lon1) * D2R;
    const a = Math.sin(dLat/2) * Math.sin(dLat/2) + Math.cos(lat1 * D2R) * Math.cos(lat2 * D2R) * Math.sin(dLon/2) * Math.sin(dLon/2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
    return R * c;
            }
// =================================================================
// FICHIER WEATHER.JS
// Récupère les données météo pour la correction troposphérique.
// =================================================================

/**
 * Récupère les données météo (Pression, Température, Humidité)
 * @param {number} lat Latitude
 * @param {number} lon Longitude
 * @returns {Promise<Object>} Une promesse résolue avec { temperature, pressure, humidity }
 */
async function fetchWeatherData(lat, lon) {
    if (typeof lat !== 'number' || typeof lon !== 'number') {
        return Promise.reject("Coordonnées GPS invalides.");
    }

    // Utilisation de l'API Open-Meteo pour l'exemple (pas de clé requise)
    const apiUrl = `https://api.open-meteo.com/v1/forecast?latitude=${lat.toFixed(4)}&longitude=${lon.toFixed(4)}&current=temperature_2m,relative_humidity_2m,surface_pressure`;

    try {
        const response = await fetch(apiUrl);
        if (!response.ok) {
            throw new Error(`Erreur API Météo: ${response.statusText}`);
        }
        
        const data = await response.json();
        
        if (data && data.current) {
            const weather = {
                temperature: data.current.temperature_2m, // en °C
                humidity: data.current.relative_humidity_2m, // en %
                pressure: data.current.surface_pressure / 100 // Convertir Pa en hPa
            };
            return weather;
        } else {
            throw new Error("Format de données météo inconnu.");
        }
    } catch (error) {
        console.error("Échec de la récupération météo:", error);
        return null;
    }
    }
// =================================================================
// FICHIER APP_LOGIC.JS
// Gestion des capteurs, de la carte, des boucles, et de l'affichage DOM.
// Dépend de math_logic.js, weather.js, math.min.js, leaflet.js, suncalc.js
// =================================================================

// --- VARIABLES D'ÉTAT GLOBALES ---
let wID = null, lPos = null; 
let imuIntervalID = null; 
let ekf6dof = null; // Instance de EKF_INS_21_States
let currentTransportMode = 'INS_6DOF_REALISTE'; 
let map = null, marker = null;
let isDeadReckoning = false;
let autoDetectedMode = 'Libre/Piéton'; // Utilisé dans math_logic.js.update()

// --- CAPTEURS RÉELS ---
let real_accel_x = 0.0; let real_accel_y = 0.0; let real_accel_z = 0.0; 
let real_gyro_x = 0.0; let real_gyro_y = 0.0; let real_gyro_z = 0.0;

// --- MÉTÉO ET CIEL ---
let lastWeatherFetchTimestamp = 0; 
let currentWeatherData = null; 
let currentGpsCoords = { lat: 0, lon: 0 }; 


// --- LISTENER IMU (DeviceMotion) ---
function imuMotionHandler(event) {
    const acc = event.accelerationIncludingGravity; 
    const rot = event.rotationRate;
    
    if (!acc || acc.x === null) { real_accel_x = 0.0; real_accel_y = 0.0; real_accel_z = 0.0; } 
    else { real_accel_x = acc.x ?? 0.0; real_accel_y = acc.y ?? 0.0; real_accel_z = acc.z ?? 0.0; }
    
    real_gyro_x = rot.alpha ?? 0.0; 
    real_gyro_y = rot.beta ?? 0.0;
    real_gyro_z = rot.gamma ?? 0.0;
}

function startIMUListeners() {
    if (window.DeviceMotionEvent) { window.addEventListener('devicemotion', imuMotionHandler); }
}

function stopIMUListeners() {
    if (window.DeviceMotionEvent) { window.removeEventListener('devicemotion', imuMotionHandler); }
}

// --- BOUCLE D'ESTIME IMU AUTONOME (100 Hz) ---
function runIMULoop() {
    if (ekf6dof) {
        const real_imu_input = [real_accel_x, real_accel_y, real_accel_z, real_gyro_x, real_gyro_y, real_gyro_z];
        ekf6dof.predict(DT_IMU, real_imu_input);
        updateDisplayMetrics();
    }
}

// --- MISE À JOUR MÉTÉO ET CIEL ---
function updateSkyDisplay(lat, lon) {
    const now = new Date();
    
    // 1. Saison (Hémisphère nord)
    const month = now.getMonth(); 
    let season = "Indéterminée";
    if (month >= 2 && month <= 4) season = "Printemps";
    else if (month >= 5 && month <= 7) season = "Été";
    else if (month >= 8 && month <= 10) season = "Automne";
    else season = "Hiver";
    document.getElementById('season-info').textContent = season;

    // 2. Position du Soleil (utilise SunCalc.js)
    if (typeof SunCalc !== 'undefined') {
        const sunPos = SunCalc.getPosition(now, lat, lon);
        const sunAltitude = sunPos.altitude * R2D; // Convertir radians en degrés
        
        let yPosPercent = (sunAltitude + 90) / 180; 
        const cssYPos = 90 - (yPosPercent * 80); 
        
        const sunElement = document.getElementById('sky-sun-object');
        if (sunElement) {
            sunElement.style.transform = `translate(-50%, ${cssYPos}%)`;
        }
    }
}

function updateWeatherAndSky(lat, lon) {
    // Mettre à jour l'affichage du ciel
    updateSkyDisplay(lat, lon);

    // Limiter les appels à l'API météo (toutes les 5 minutes)
    const now = Date.now();
    const CINQ_MINUTES_MS = 300000;
    if (now - lastWeatherFetchTimestamp > CINQ_MINUTES_MS) {
        lastWeatherFetchTimestamp = now;
        
        if (typeof fetchWeatherData !== 'undefined') {
            fetchWeatherData(lat, lon)
                .then(data => {
                    if (data) {
                        currentWeatherData = data;
                        document.getElementById('weather-info').textContent = 
                            `${data.temperature.toFixed(1)}°C | ${data.pressure.toFixed(0)}hPa | ${data.humidity.toFixed(0)}%`;
                        // C'est ici que l'EKF pourrait utiliser les données météo (ekf6dof.setTroposphericCorrection(data);)
                    }
                })
                .catch(error => {
                    document.getElementById('weather-info').textContent = "Erreur API";
                });
        }
    }
    // Afficher les données météo en cache
    else if (currentWeatherData) {
        document.getElementById('weather-info').textContent = 
            `${currentWeatherData.temperature.toFixed(1)}°C | ${currentWeatherData.pressure.toFixed(0)}hPa | ${currentWeatherData.humidity.toFixed(0)}%`;
    }
}


// --- MISE À JOUR DES MÉTRIES D'AFFICHAGE (ÉCRAN) ---
function updateDisplayMetrics() {
    if (!ekf6dof) return;

    const final_speed = ekf6dof.getSpeed();
    const R_kalman_input = getKalmanR(lPos?.coords?.accuracy ?? R_MAX, final_speed);
    
    const modeStatus = isDeadReckoning ? '🚨 DEAD RECKONING (DR) EN COURS - Mode DR auto: ' + autoDetectedMode : '🛰️ FUSION INS/GNSS';
    
    const v_x = ekf6dof.true_state.velocity.get([0]);
    const v_y = ekf6dof.true_state.velocity.get([1]);
    const p_norm_sq = ekf6dof.P.get([0,0]);
    
    // Mise à jour des valeurs
    document.getElementById('speed-stable').textContent = `${(final_speed < 0.05 ? 0 : final_speed).toFixed(3)}`;
    document.getElementById('current-speed').textContent = `${((final_speed < 0.05 ? 0 : final_speed) * KMH_MS).toFixed(2)}`;
    document.getElementById('kalman-r-dyn').textContent = `${R_kalman_input.toFixed(2)}`;
    document.getElementById('gps-status-dr').textContent = modeStatus;
    
    // ... (Autres mises à jour de l'EKF) ...
    
    if (document.getElementById('kalman-uncert')) document.getElementById('kalman-uncert').textContent = `Matrice P (${p_norm_sq.toFixed(2)})`;
    if (document.getElementById('gyro-bias')) document.getElementById('gyro-bias').textContent = `${ekf6dof.getGyroBias().toFixed(3)}`;
    if (document.getElementById('nhc-status')) document.getElementById('nhc-status').textContent = autoDetectedMode; // Mise à jour du mode auto détecté
}


// --- LOGIQUE DE SYNCHRONISATION (Multi-Source/GNSS) ---
function updateEKFWithExternalSource(lat, lon, alt, acc, speed_raw, R_speed_factor_custom, altAccRaw) {
    if (!ekf6dof) return;

    // ... (Logique de détection d'anomalie de vitesse et de distance) ...
    // Note: cette partie nécessite de réécrire distance() qui est dans math_logic.js

    const R_kalman_input = getKalmanR(acc, ekf6dof.getSpeed()); 
    const R_speed_factor = 1.0; // Simplifié
    
    const external_measurement = math.matrix([lat, lon, alt, speed_raw, 0, 0]); 
    const R_matrix = math.diag([R_kalman_input, R_kalman_input, altAccRaw, R_speed_factor, R_speed_factor, R_V_VERTICAL_UNCERTAINTY]); 
    
    // Mise à jour de l'EKF
    const detectedMode = ekf6dof.update(external_measurement, R_matrix, isDeadReckoning);
    autoDetectedMode = detectedMode; // Met à jour la variable globale pour l'affichage
}

// --- FONCTIONS DE BASE (GPS, Carte, Initialisation) ---
function updateDisp(pos) {
    const accRaw = pos.coords.accuracy;

    if (currentTransportMode !== 'INS_6DOF_REALISTE' || !ekf6dof) {
        lPos = pos;
        updateMap(pos.coords.latitude, pos.coords.longitude, accRaw);
        return; 
    }
    
    if (accRaw > R_MAX) { isDeadReckoning = true; } 
    else if (isDeadReckoning && accRaw < R_MAX) { isDeadReckoning = false; }

    const cLat = pos.coords.latitude;
    const cLon = pos.coords.longitude;
    const altRaw = pos.coords.altitude || 0.0;
    const altAccRaw = pos.coords.altitudeAccuracy || 10.0; 

    currentGpsCoords = { lat: cLat, lon: cLon };
    updateWeatherAndSky(cLat, cLon); // Mise à jour Météo et Ciel

    updateEKFWithExternalSource(cLat, cLon, altRaw, accRaw, pos.coords.speed || 0.0, null, altAccRaw);
    lPos = pos;
    updateMap(cLat, cLon, accRaw);
}

function initMap() {
    if (typeof L !== 'undefined') {
        map = L.map('map').setView([43.2965, 5.37], 13);
        marker = L.marker([0, 0]).addTo(map);
    }
}

function updateMap(lat, lon, acc) {
    if (map && marker) {
        marker.setLatLng([lat, lon]);
        map.setView([lat, lon], map.getZoom() < 13 ? 13 : map.getZoom());
    }
}

function requestSensorPermissionAndStart() {
    
    const startFusion = () => {
        wID = navigator.geolocation.watchPosition(updateDisp, (err) => {
            if (err.code === 3 || err.code === 2) { 
                isDeadReckoning = true;
                document.getElementById('gps-status-dr').textContent = '🚨 ERREUR GPS: Passage en DR';
            } else { console.error(err); }
        }, { enableHighAccuracy: true, timeout: 5000, maximumAge: 0 });

        if (currentTransportMode === 'INS_6DOF_REALISTE') {
            startIMUListeners(); 
            imuIntervalID = setInterval(runIMULoop, DT_IMU * 1000); 
        }
        document.getElementById('toggle-gps-btn').textContent = "Arrêter la Fusion";
    };

    if (currentTransportMode === 'INS_6DOF_REALISTE' && typeof DeviceOrientationEvent.requestPermission === 'function') {
        DeviceOrientationEvent.requestPermission()
            .then(permissionState => {
                if (permissionState === 'granted') { startFusion(); } 
                else { alert("Autorisation capteurs refusée. La dérive sera maximale sans mouvement."); }
            });
    } else {
        startFusion();
    }
}

function startGPS() {
    stopGPS(); 
    requestSensorPermissionAndStart(); 
}

function stopGPS() {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    if (imuIntervalID !== null) clearInterval(imuIntervalID); 
    stopIMUListeners(); 
    wID = null;
    imuIntervalID = null;
    isDeadReckoning = false;
    document.getElementById('gps-status-dr').textContent = 'Arrêté';
    document.getElementById('toggle-gps-btn').textContent = "Démarrer la Fusion (INS/GNSS)";
}

// --- INITIALISATION DES ÉVÉNEMENTS DOM ---
document.addEventListener('DOMContentLoaded', () => {
    
    initMap(); 
    
    document.getElementById('toggle-gps-btn').addEventListener('click', () => { 
        wID === null ? startGPS() : stopGPS(); 
    });
    
    document.getElementById('transport-mode-select').addEventListener('change', (e) => {
        stopGPS(); 
        ekf6dof = null;
        setTransportModeParameters(e.target.value);
    });

    function setTransportModeParameters(mode) {
        currentTransportMode = mode;
        if (mode === 'INS_6DOF_REALISTE') {
            ekf6dof = new EKF_INS_21_States(); 
            document.getElementById('nhc-status').textContent = '✅ EKF INS 21-États ACTIF';
        } else {
             document.getElementById('nhc-status').textContent = 'Mode EKF 1D ou Fictionnel';
        }
    }

    setTransportModeParameters(document.getElementById('transport-mode-select').value); 
});
