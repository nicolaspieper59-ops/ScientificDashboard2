// =================================================================
// FICHIER JS COMPLET : gnss-dashboard-full.js
// EKF 6-DOF (15 États) OPTIMISÉ POUR UN RÉALISME DE VITESSE MAXIMAL AVEC SYNCHRONISATION MULTI-SOURCES
// =================================================================

// --- CONSTANTES DE BASE ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const KMH_MS = 3.6;         
const MIN_DT = 0.0001;      

// --- CONSTANTES DE FRÉQUENCE IMU ---
const IMU_FREQUENCY_HZ = 100;
const DT_IMU = 1 / IMU_FREQUENCY_HZ; // 0.01 secondes

// --- PARAMÈTRES EKF et ANTI-SAUT ---
const R_MIN = 1.0;            
const R_MAX = 500.0;          
const R_SLOW_SPEED_FACTOR = 100.0; 
const MAX_REALISTIC_SPD_M = 15.0;  
const R_V_VERTICAL_UNCERTAINTY = 100.0; 
const DRAG_COEFFICIENT = 0.05; // Ajustez pour la glace (0.001) ou l'eau (0.05)

// --- VARIABLES D'ÉTAT GLOBALES ---
let wID = null, lPos = null;
let imuIntervalID = null; 
let ekf6dof = null;
let currentTransportMode = 'INS_6DOF_REALISTE'; 
let map = null, marker = null;
let isDeadReckoning = false;
let autoDetectedMode = 'Libre/Piéton'; 

// --- VARIABLES GLOBALES POUR LES CAPTEURS RÉELS ---
let real_accel_x = 0.0;
let real_accel_y = 0.0;
let real_accel_z = 0.0; 
let real_gyro_x = 0.0;
let real_gyro_y = 0.0;
let real_gyro_z = 0.0;

// --- CLASSE CONCEPTUELLE EKF 6-DOF (15 états simplifiés) ---
class EKF_6DoF {
    constructor() {
        this.error_state_vector = math.zeros(15); 
        const initial_uncertainty = [100, 100, 100, 1, 1, 1, 0.01, 0.01, 0.01, 0.001, 0.001, 0.001, 0.0001, 0.0001, 0.0001];
        this.P = math.diag(initial_uncertainty); 
        
        // Q: Bruit de Processus 
        this.Q = math.diag([0, 0, 0, 0.01, 0.01, 0.01, 0.001, 0.001, 0.001, 0.00005, 0.00005, 0.000001, 0.000001, 0.000001, 0.000001]);
        
        this.true_state = {
            position: math.matrix([0, 0, 0]), 
            velocity: math.matrix([0, 0, 0]), 
            accel_bias: math.matrix([0, 0, 0]), 
        };
    }
    
    predict(dt, imu_input) {
        // P_k = F * P_{k-1} * F^T + Q
        const F = math.identity(15); 
        this.P = math.add(math.multiply(F, math.multiply(this.P, math.transpose(F))), this.Q);
        
        let accel_raw = math.matrix([imu_input[0], imu_input[1], imu_input[2]]);
        // Correction du biais estimé
        let accel_corrected = math.subtract(accel_raw, this.true_state.accel_bias);

        // FORCE DE TRAÎNÉE (DAMPING) : Réalisme de la décélération
        let drag_force = math.multiply(this.true_state.velocity, -DRAG_COEFFICIENT);
        let total_acceleration = math.add(accel_corrected, drag_force);
        
        // Intégration de la Vitesse (a * dt)
        let delta_v = math.multiply(total_acceleration, dt);
        this.true_state.velocity = math.add(this.true_state.velocity, delta_v);
        
        // Intégration de la Position (v * dt)
        let delta_p = math.multiply(this.true_state.velocity, dt);
        this.true_state.position = math.add(this.true_state.position, delta_p);
    }
    
    // Logique CNH (Contrainte Non-Holonomique) pour le mode Dead Reckoning
    autoDetermineCNH(Vx, Vy, Vz, Vtotal) {
        const MIN_MOVEMENT_THRESHOLD = 0.05; 
        const DRAG_FACTOR_STOP = 0.01; 
        const DRAG_FACTOR_FREE = 0.999; 

        if (Vtotal < MIN_MOVEMENT_THRESHOLD) {
             autoDetectedMode = '🛑 Arrêt/Zero-Velocity';
             // Force la vitesse proche de 0 (correction CNH agressive)
             this.true_state.velocity = math.multiply(this.true_state.velocity, DRAG_FACTOR_STOP); 
             return { Vx_corr: DRAG_FACTOR_STOP, Vy_corr: DRAG_FACTOR_STOP, Vz_corr: DRAG_FACTOR_STOP }; 
        }
        
        if (Math.abs(Vz) > Vtotal * 0.8) {
             autoDetectedMode = '⏫ Ascenseur/Vertical';
             return { Vx_corr: 0.90, Vy_corr: 0.90, Vz_corr: 0.99 }; 
        }

        autoDetectedMode = '🚁 Libre/Drone/Piéton';
        // En mouvement, on utilise un facteur très doux (Drag Factor fait le travail)
        return { Vx_corr: DRAG_FACTOR_FREE, Vy_corr: DRAG_FACTOR_FREE, Vz_corr: DRAG_FACTOR_FREE }; 
    }

    update(z, R_k, isDeadReckoning) {
        // Réduction de l'incertitude globale (P) et du biais pour une meilleure stabilité
        this.P = math.multiply(this.P, 0.9); 
        const BIAS_STABILITY_FACTOR = isDeadReckoning ? 0.99 : 0.95; 
        this.true_state.accel_bias = math.multiply(this.true_state.accel_bias, BIAS_STABILITY_FACTOR); 
        
        // Application des CNH
        const Vx = this.true_state.velocity.get([0]);
        const Vy = this.true_state.velocity.get([1]);
        const Vz = this.true_state.velocity.get([2]);
        const Vtotal = math.norm(this.true_state.velocity);

        const { Vx_corr, Vy_corr, Vz_corr } = this.autoDetermineCNH(Vx, Vy, Vz, Vtotal);
        
        this.true_state.velocity.set([0], Vx * Vx_corr);
        this.true_state.velocity.set([1], Vy * Vy_corr);
        this.true_state.velocity.set([2], Vz * Vz_corr);
    }
    
    getSpeed() {
        return math.norm(this.true_state.velocity);
    }
    
    getAccelBias() {
        return this.true_state.accel_bias.get([0]); 
    }
}

// --- FONCTIONS UTILITAIRES (non modifiées) ---
function getKalmanR(accRaw, final_speed) {
    let R = Math.max(R_MIN, accRaw);
    if (final_speed < 0.5) R = R * R_SLOW_SPEED_FACTOR;
    return R;
}

function distance(lat1, lon1, lat2, lon2) {
    const R = 6371e3; 
    const dLat = (lat2 - lat1) * D2R;
    const dLon = (lon2 - lon1) * D2R;
    const a = Math.sin(dLat/2) * Math.sin(dLat/2) +
              Math.cos(lat1 * D2R) * Math.cos(lat2 * D2R) *
              Math.sin(dLon/2) * Math.sin(dLon/2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
    return R * c;
}

// --- LISTENER IMU (SANS BOUCLE) ---
function imuMotionHandler(event) {
    const acc = event.acceleration; 
    const rot = event.rotationRate;
    
    if (!acc || acc.x === null) {
        real_accel_x = 0.0;
        real_accel_y = 0.0;
        real_accel_z = 0.0; 
    } else {
        real_accel_x = acc.x ?? 0.0;
        real_accel_y = acc.y ?? 0.0;
        real_accel_z = acc.z ?? 0.0; 
    }
    
    real_gyro_x = rot.alpha ?? 0.0; 
    real_gyro_y = rot.beta ?? 0.0;
    real_gyro_z = rot.gamma ?? 0.0;
}

function startIMUListeners() {
    if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', imuMotionHandler);
    }
}

function stopIMUListeners() {
    if (window.DeviceMotionEvent) {
        window.removeEventListener('devicemotion', imuMotionHandler);
    }
}

// --- BOUCLE D'ESTIME IMU AUTONOME (100 Hz) ---
function runIMULoop() {
    if (ekf6dof) {
        const real_imu_input = [real_accel_x, real_accel_y, real_accel_z, real_gyro_x, real_gyro_y, real_gyro_z];
        ekf6dof.predict(DT_IMU, real_imu_input);
        updateDisplayMetrics();
    }
}

// --- MISE À JOUR DES MÉTRIES D'AFFICHAGE (non modifiée) ---
function updateDisplayMetrics() {
    if (!ekf6dof) return;

    const final_speed = ekf6dof.getSpeed();
    const current_ekf_speed = final_speed;

    const R_kalman_input = getKalmanR(lPos?.coords?.accuracy ?? R_MAX, current_ekf_speed);
    
    const modeStatus = isDeadReckoning 
        ? '🚨 DEAD RECKONING (DR) EN COURS - Mode DR auto: ' + autoDetectedMode
        : '🛰️ FUSION GNSS/IMU';
    
    const v_x = ekf6dof.true_state.velocity.get([0]);
    const v_y = ekf6dof.true_state.velocity.get([1]);
    const p_x = ekf6dof.true_state.position.get([0]);
    const p_y = ekf6dof.true_state.position.get([1]);
    const p_z = ekf6dof.true_state.position.get([2]);

    const sSpdFE = final_speed < 0.05 ? 0 : final_speed; 
    document.getElementById('speed-stable').textContent = `${sSpdFE.toFixed(3)}`;
    document.getElementById('current-speed').textContent = `${(sSpdFE * KMH_MS).toFixed(2)}`;
    document.getElementById('kalman-r-dyn').textContent = `${R_kalman_input.toFixed(2)}`;
    document.getElementById('gps-status-dr').textContent = modeStatus;
    
    if (document.getElementById('speed-x')) document.getElementById('speed-x').textContent = `${v_x.toFixed(2)}`;
    if (document.getElementById('speed-y')) document.getElementById('speed-y').textContent = `${v_y.toFixed(2)}`;
    if (document.getElementById('pos-x')) document.getElementById('pos-x').textContent = `${p_x.toFixed(2)}`;
    if (document.getElementById('pos-y')) document.getElementById('pos-y').textContent = `${p_y.toFixed(2)}`;
    
    const p_norm_sq = ekf6dof.P.get([0,0]);
    document.getElementById('kalman-uncert').textContent = `Matrice P (${p_norm_sq.toFixed(2)})`;
    document.getElementById('altitude-kalman').textContent = `${p_z.toFixed(2)} m`;
    document.getElementById('kalman-q-noise').textContent = `${ekf6dof.getAccelBias().toFixed(3)}`;
}

// =================================================================
// FONCTION DE SYNCHRONISATION UNIVERSELLE (POINT D'ENTRÉE)
// Appelé par n'importe quelle source de mesure (GNSS, Wi-Fi RTT, Odometry, etc.)
// =================================================================
function updateEKFWithExternalSource(lat, lon, alt, acc, speed_raw, R_speed_factor_custom, altAccRaw) {
    if (!ekf6dof) return;

    const current_ekf_speed = ekf6dof.getSpeed();
    let R_kalman_input = getKalmanR(acc, current_ekf_speed); 
    
    // --- LOGIQUE ANTI-SAUT GPS/GNSS (seulement si lPos est défini et acc est réaliste) ---
    if (lPos && acc < R_MAX) {
        const dt_gps = (Date.now() - lPos.timestamp) / 1000 || 1.0;
        const measured_dist = distance(lPos.coords.latitude, lPos.coords.longitude, lat, lon);
        const max_dist_plausible = MAX_REALISTIC_SPD_M * dt_gps; 
        
        if (measured_dist > max_dist_plausible) {
            R_kalman_input = R_MAX * 100; // Augmenter R pour rejeter la mesure.
        }
    }
    
    // --- ÉTALONNAGE DYNAMIQUE DE LA VITESSE (Utilise R_speed_factor_custom s'il est fourni) ---
    let R_speed_factor = R_speed_factor_custom || 1.0; 
    
    // Si la vitesse n'est pas fournie, on utilise la logique GNSS agressive par défaut
    if (!R_speed_factor_custom && speed_raw !== undefined) {
        const speed_difference = Math.abs(current_ekf_speed - speed_raw);
        const SPEED_TOLERANCE = 2.0; 

        if (speed_difference > SPEED_TOLERANCE) {
            R_speed_factor = 100.0; // GPS/Source non fiable
        } else {
            R_speed_factor = 0.001; // Correction EXTRÊMEMENT agressive.
        }
    }
    
    // Exécution de l'étape UPDATE (Correction 6D par la mesure externe)
    const external_measurement = math.matrix([lat, lon, alt, speed_raw, 0, 0]); 
    
    const R_matrix = math.diag([
        R_kalman_input,             // R - Pos X (Lat)
        R_kalman_input,             // R - Pos Y (Lon)
        altAccRaw,                  // R - Pos Z (Altitude)
        R_speed_factor,             // R - Vitesse X (étalonnée)
        R_speed_factor,             // R - Vitesse Y (étalonnée)
        R_V_VERTICAL_UNCERTAINTY    // R - Vitesse Z (très incertaine)
    ]); 
    
    ekf6dof.update(external_measurement, R_matrix, isDeadReckoning);

    // Mettre à jour lPos ici pour le calcul de l'Anti-Saut
    // Si vous utilisez une autre source que le GNSS, vous devrez gérer l'historique différemment.
}


// --- FONCTION PRINCIPALE DE MISE À JOUR GPS/GNSS (Appelle la fonction Universelle) ---
function updateDisp(pos) {
    const accRaw = pos.coords.accuracy;
    
    if (currentTransportMode !== 'INS_6DOF_REALISTE' || !ekf6dof) {
        lPos = pos;
        updateMap(pos.coords.latitude, pos.coords.longitude, accRaw);
        return; 
    }
    
    // Détermination du Dead Reckoning (DR) par la fiabilité GNSS
    if (accRaw > R_MAX) {
        isDeadReckoning = true;
    } else if (isDeadReckoning && accRaw < R_MAX) {
        isDeadReckoning = false;
    }

    const cLat = pos.coords.latitude;
    const cLon = pos.coords.longitude;
    const altRaw = pos.coords.altitude || 0.0;
    const altAccRaw = pos.coords.altitudeAccuracy || 10.0; 
    const spd3D_raw = pos.coords.speed || 0.0; 

    // Appel de la fonction universelle
    updateEKFWithExternalSource(cLat, cLon, altRaw, accRaw, spd3D_raw, null, altAccRaw);

    // Mettre à jour lPos pour le calcul de l'Anti-Saut du GPS/GNSS
    lPos = pos;
    updateMap(cLat, cLon, accRaw);
}


// --- GESTION DE LA CARTE (non modifiée) ---
function initMap() {
    map = L.map('map').setView([43.2965, 5.37], 13);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; OpenStreetMap contributors'
    }).addTo(map);
    marker = L.marker([0, 0]).addTo(map);
}

function updateMap(lat, lon, acc) {
    if (map && marker) {
        marker.setLatLng([lat, lon]);
        map.setView([lat, lon], map.getZoom() < 13 ? 13 : map.getZoom());
    }
}

// --- GESTION DU DÉMARRAGE AVEC AUTORISATION DES CAPTEURS (non modifiée) ---
function requestSensorPermissionAndStart() {
    
    const startFusion = () => {
        // Le wID est maintenant dédié à l'écoute GNSS (source principale)
        wID = navigator.geolocation.watchPosition(updateDisp, (err) => {
            if (err.code === 3 || err.code === 2) { 
                isDeadReckoning = true;
                document.getElementById('gps-status-dr').textContent = '🚨 ERREUR GPS: Passage en DR';
            } else {
                console.error(err);
            }
        }, {
            enableHighAccuracy: true,
            timeout: 5000,
            maximumAge: 0
        });

        if (currentTransportMode === 'INS_6DOF_REALISTE') {
            startIMUListeners(); 
            imuIntervalID = setInterval(runIMULoop, DT_IMU * 1000); 
        }
        document.getElementById('toggle-gps-btn').textContent = "Arrêter la Fusion";
    };

    if (currentTransportMode === 'INS_6DOF_REALISTE' && typeof DeviceOrientationEvent.requestPermission === 'function') {
        DeviceOrientationEvent.requestPermission()
            .then(permissionState => {
                if (permissionState === 'granted') {
                    startFusion();
                } else {
                    console.error("Autorisation des capteurs refusée.");
                    alert("Autorisation capteurs refusée. La dérive sera maximale sans mouvement.");
                }
            })
            .catch(error => console.error("Erreur lors de la demande d'autorisation :", error));
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
    document.getElementById('toggle-gps-btn').textContent = "Démarrer la Fusion (GPS/IMU)";
}

// --- INITIALISATION DES ÉVÉNEMENTS DOM (non modifiée) ---
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
            ekf6dof = new EKF_6DoF();
            document.getElementById('nhc-status').textContent = '✅ EKF 6-DoF ACTIF (Fusion)';
        } else {
             document.getElementById('nhc-status').textContent = 'Mode EKF 1D ou Fictionnel';
        }
    }

    setTransportModeParameters(document.getElementById('transport-mode-select').value); 
});
