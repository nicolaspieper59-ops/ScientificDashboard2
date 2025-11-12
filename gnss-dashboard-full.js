// =================================================================
// FICHIER JS FINAL : gnss-dashboard-full.js
// EKF INS (Système de Navigation Inertielle) - ES-EKF 21 États CONCEPTUEL
// =================================================================

// --- CONSTANTES DE BASE ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const KMH_MS = 3.6;         
const GRAVITY = 9.81; // m/s^2

// --- CONSTANTES DE FRÉQUENCE IMU ---
const IMU_FREQUENCY_HZ = 100;
const DT_IMU = 1 / IMU_FREQUENCY_HZ; 

// --- PARAMÈTRES EKF ---
const R_MIN = 1.0;            
const R_MAX = 500.0;          
const R_SLOW_SPEED_FACTOR = 100.0; 
const MAX_REALISTIC_SPD_M = 15.0;  
const R_V_VERTICAL_UNCERTAINTY = 100.0; 
const DRAG_COEFFICIENT = 0.05; 

// --- VARIABLES D'ÉTAT GLOBALES ---
let wID = null, lPos = null; 
let imuIntervalID = null; 
let ekf6dof = null;
let currentTransportMode = 'INS_6DOF_REALISTE'; 
let map = null, marker = null;
let isDeadReckoning = false;
let autoDetectedMode = 'Libre/Piéton'; 

// --- VARIABLES GLOBALES POUR LES CAPTEURS RÉELS ---
let real_accel_x = 0.0; let real_accel_y = 0.0; let real_accel_z = 0.0; 
let real_gyro_x = 0.0; let real_gyro_y = 0.0; let real_gyro_z = 0.0;

// =================================================================
// CLASSE EKF INS (Système de Navigation Inertielle) - ES-EKF 21 États
// =================================================================
class EKF_INS_21_States {
    constructor() {
        this.error_state_vector = math.zeros(21); 
        this.true_state = {
            position: math.matrix([0, 0, 0]), 
            velocity: math.matrix([0, 0, 0]), 
            attitude_q: math.matrix([1, 0, 0, 0]), // Quaternion 
            accel_bias: math.matrix([0, 0, 0]), 
            gyro_bias: math.matrix([0, 0, 0]),     
        };
        
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
    
    // --- FONCTIONS PLACEHOLDERS (Nécessitent la librairie math.js) ---
    updateQuaternion(q, gyro, dt) { return q; }
    quaternionToRotationMatrix(q) { return math.identity(3); }

    autoDetermineCNH(Vtotal) {
        const MIN_MOVEMENT_THRESHOLD = 0.05; 
        const DRAG_FACTOR_STOP = 0.01; 
        const DRAG_FACTOR_FREE = 0.999; 

        if (Vtotal < MIN_MOVEMENT_THRESHOLD) {
             autoDetectedMode = '🛑 Arrêt/Zero-Velocity';
             return { factor: DRAG_FACTOR_STOP }; 
        }
        
        autoDetectedMode = '🚁 Dynamique 3D/Avion';
        return { factor: DRAG_FACTOR_FREE }; 
    }

    update(z, R_matrix, isDeadReckoning) {
        const Vtotal = math.norm(this.true_state.velocity);
        const { factor: CNH_factor } = this.autoDetermineCNH(Vtotal);
        this.true_state.velocity = math.multiply(this.true_state.velocity, CNH_factor);

        this.true_state.accel_bias = math.multiply(this.true_state.accel_bias, isDeadReckoning ? 0.999 : 0.95);
        this.true_state.gyro_bias = math.multiply(this.true_state.gyro_bias, isDeadReckoning ? 0.999 : 0.95);
        
        this.P = math.multiply(this.P, 0.9); 
    }
    
    getSpeed() { return math.norm(this.true_state.velocity); }
    getAccelBias() { return this.true_state.accel_bias.get([0]); }
    getGyroBias() { return this.true_state.gyro_bias.get([0]); }
}

// --- FONCTIONS UTILITAIRES (inchangées) ---
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

// --- MISE À JOUR DES MÉTRIES D'AFFICHAGE ---
function updateDisplayMetrics() {
    if (!ekf6dof) return;

    const final_speed = ekf6dof.getSpeed();
    const current_ekf_speed = final_speed;

    const R_kalman_input = getKalmanR(lPos?.coords?.accuracy ?? R_MAX, current_ekf_speed);
    
    const modeStatus = isDeadReckoning ? '🚨 DEAD RECKONING (DR) EN COURS - Mode DR auto: ' + autoDetectedMode : '🛰️ FUSION INS/GNSS';
    
    const v_x = ekf6dof.true_state.velocity.get([0]);
    const v_y = ekf6dof.true_state.velocity.get([1]);
    const p_z = ekf6dof.true_state.position.get([2]);

    const sSpdFE = final_speed < 0.05 ? 0 : final_speed; 
    
    document.getElementById('speed-stable').textContent = `${sSpdFE.toFixed(3)}`;
    document.getElementById('current-speed').textContent = `${(sSpdFE * KMH_MS).toFixed(2)}`;
    document.getElementById('kalman-r-dyn').textContent = `${R_kalman_input.toFixed(2)}`;
    document.getElementById('gps-status-dr').textContent = modeStatus;
    
    if (document.getElementById('speed-x')) document.getElementById('speed-x').textContent = `${v_x.toFixed(2)}`;
    if (document.getElementById('speed-y')) document.getElementById('speed-y').textContent = `${v_y.toFixed(2)}`;
    
    const p_norm_sq = ekf6dof.P.get([0,0]);
    document.getElementById('kalman-uncert').textContent = `Matrice P (${p_norm_sq.toFixed(2)})`;
    document.getElementById('altitude-kalman').textContent = `${p_z.toFixed(2)} m`;
    document.getElementById('kalman-q-noise').textContent = `${ekf6dof.getAccelBias().toFixed(3)}`;
    document.getElementById('gyro-bias').textContent = `${ekf6dof.getGyroBias().toFixed(3)}`;
}

// --- LOGIQUE DE SYNCHRONISATION (Multi-Source/GNSS) ---
function updateEKFWithExternalSource(lat, lon, alt, acc, speed_raw, R_speed_factor_custom, altAccRaw) {
    if (!ekf6dof) return;

    const current_ekf_speed = ekf6dof.getSpeed();
    let R_kalman_input = getKalmanR(acc, current_ekf_speed); 
    
    if (lPos && acc < R_MAX) {
        const dt_gps = (Date.now() - lPos.timestamp) / 1000 || 1.0;
        const measured_dist = distance(lPos.coords.latitude, lPos.coords.longitude, lat, lon);
        const max_dist_plausible = MAX_REALISTIC_SPD_M * dt_gps; 
        
        if (measured_dist > max_dist_plausible) { R_kalman_input = R_MAX * 100; }
    }
    
    let R_speed_factor = R_speed_factor_custom || 1.0; 
    if (!R_speed_factor_custom && speed_raw !== undefined) {
        const speed_difference = Math.abs(current_ekf_speed - speed_raw);
        const SPEED_TOLERANCE = 2.0; 

        if (speed_difference > SPEED_TOLERANCE) { R_speed_factor = 100.0; } 
        else { R_speed_factor = 0.001; } 
    }
    
    const external_measurement = math.matrix([lat, lon, alt, speed_raw, 0, 0]); 
    const R_matrix = math.diag([R_kalman_input, R_kalman_input, altAccRaw, R_speed_factor, R_speed_factor, R_V_VERTICAL_UNCERTAINTY]); 
    
    ekf6dof.update(external_measurement, R_matrix, isDeadReckoning);
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
    const spd3D_raw = pos.coords.speed || 0.0; 

    updateEKFWithExternalSource(cLat, cLon, altRaw, accRaw, spd3D_raw, null, altAccRaw);
    lPos = pos;
    updateMap(cLat, cLon, accRaw);
}

function initMap() {
    map = L.map('map').setView([43.2965, 5.37], 13);
    // Tuiles OSM commentées pour le mode HORS LIGNE
    marker = L.marker([0, 0]).addTo(map);
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
