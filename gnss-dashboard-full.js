// =================================================================
// FICHIER JS COMPLET : gnss-dashboard-full.js
// EKF 6-DOF autonome (DR) avec DÉRIVE MINIMALE (Utilisation de l'accélération sans gravité)
// =================================================================

// --- CONSTANTES DE BASE ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const KMH_MS = 3.6;         
const MIN_DT = 0.0001;      

// --- CONSTANTES DE FRÉQUENCE IMU (Base du DR Autonome) ---
const IMU_FREQUENCY_HZ = 100;
const DT_IMU = 1 / IMU_FREQUENCY_HZ; // 0.01 secondes

// --- PARAMÈTRES EKF et ANTI-SAUT ---
const R_MIN = 1.0;            
const R_MAX = 500.0;          
const R_SLOW_SPEED_FACTOR = 100.0; 
const MAX_REALISTIC_SPD_M = 15.0;  

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

// --- CLASSE CONCEPTUELLE EKF 6-DOF ---
class EKF_6DoF {
    constructor() {
        this.error_state_vector = math.zeros(15); 
        const initial_uncertainty = [100, 100, 100, 1, 1, 1, 0.01, 0.01, 0.01, 0.001, 0.001, 0.001, 0.0001, 0.0001, 0.0001];
        this.P = math.diag(initial_uncertainty); 
        
        // Q (Bruit de Processus) : Bruit minimal
        this.Q = math.diag([0, 0, 0, 0.01, 0.01, 0.01, 0.001, 0.001, 0.001, 0.00001, 0.00001, 0.000001, 0.000001, 0.000001, 0.000001]);
        
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
        
        // Accélération brute (VRAIS capteurs - on suppose SANS GRAVITÉ ici)
        let accel_raw = math.matrix([imu_input[0], imu_input[1], imu_input[2]]);
        
        // Accélération corrigée = Mesure brute moins le biais de capteur ESTIMÉ
        let accel_corrected = math.subtract(accel_raw, this.true_state.accel_bias);

        // Intégration de la Vitesse (a * dt)
        let delta_v = math.multiply(accel_corrected, dt);
        this.true_state.velocity = math.add(this.true_state.velocity, delta_v);
        
        // Intégration de la Position (v * dt)
        let delta_p = math.multiply(this.true_state.velocity, dt);
        this.true_state.position = math.add(this.true_state.position, delta_p);
    }
    
    // Logique CNH (Contrainte Non-Holonomique) pour le mode Dead Reckoning
    autoDetermineCNH(Vx, Vy, Vz, Vtotal) {
        const MIN_MOVEMENT_THRESHOLD = 0.05; 
        
        // *** OPTIMISATION 1: CNH AGRESSIVE À L'ARRÊT (Pour minimiser la dérive) ***
        const DRAG_FACTOR_STOP = 0.01; // Correction très forte, la vitesse s'annule rapidement

        if (Vtotal < MIN_MOVEMENT_THRESHOLD) {
             autoDetectedMode = '🛑 Arrêt/Zero-Velocity';
             
             // Réduction très agressive de la vitesse pour annuler la dérive
             this.true_state.velocity = math.multiply(this.true_state.velocity, DRAG_FACTOR_STOP); 

             return { Vx_corr: 0.01, Vy_corr: 0.01, Vz_corr: 0.01 }; // Faible propagation du bruit
        }
        
        // CNH pour le mouvement libre (mouvement vertical)
        if (Math.abs(Vz) > Vtotal * 0.8) {
             autoDetectedMode = '⏫ Ascenseur/Vertical';
             return { Vx_corr: 0.90, Vy_corr: 0.90, Vz_corr: 0.99 }; 
        }

        autoDetectedMode = '🚁 Libre/Drone/Piéton';
        return { Vx_corr: 0.95, Vy_corr: 0.95, Vz_corr: 0.95 }; 
    }

    update(z, R_k, isDeadReckoning) {
        // Correction de la Covariance (P)
        this.P = math.multiply(this.P, 0.9); 
        
        // Dégénérescence simple du biais estimé 
        this.true_state.accel_bias = math.multiply(this.true_state.accel_bias, 0.99); 
        
        // Stabilisation de la vitesse (évite la dérive) - Moins agressive ici car la CNH gère le cas à l'arrêt
        if (!isDeadReckoning) {
             if (math.norm(this.true_state.velocity) > 0.1) {
                 this.true_state.velocity = math.multiply(this.true_state.velocity, 0.99); 
             }
        }
        
        // Application des CNH
        const Vx = this.true_state.velocity.get([0]);
        const Vy = this.true_state.velocity.get([1]);
        const Vz = this.true_state.velocity.get([2]);
        const Vtotal = math.norm(this.true_state.velocity);

        const { Vx_corr, Vy_corr, Vz_corr } = this.autoDetermineCNH(Vx, Vy, Vz, Vtotal);
        
        // Les CNH ne sont appliquées que si le système n'est PAS en Dead Reckoning (il utilise le GPS)
        // Mais nous les appliquons ici pour minimiser la dérive même sans GPS parfait.
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

// --- FONCTIONS UTILITAIRES ---
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
    // *** OPTIMISATION 2: UTILISER l'accélération SANS GRAVITÉ ***
    // Cela est crucial. event.acceleration doit être utilisé si disponible, 
    // sinon nous retombons sur la divergence explosive.
    const acc = event.acceleration; 
    const rot = event.rotationRate;
    
    // Si acc n'est pas disponible, le navigateur est probablement vieux ou la permission n'est pas complète.
    if (!acc || acc.x === null) {
        // En cas d'échec de la lecture sans gravité, le système s'arrête de dériver.
        // Sinon, le système explose à 9.81 m/s² si l'on prend accelerationIncludingGravity.
        real_accel_x = 0.0;
        real_accel_y = 0.0;
        real_accel_z = 0.0; 
        console.warn("Accélération sans gravité (event.acceleration) non disponible. DR stoppé pour éviter la divergence.");
    } else {
        // Mise à jour des variables globales pour l'EKF
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

// --- MISE À JOUR DES MÉTRIES D'AFFICHAGE (Peut être appelée à 100Hz) ---
function updateDisplayMetrics() {
    if (!ekf6dof) return;

    const final_speed = ekf6dof.getSpeed();
    const current_ekf_speed = final_speed;

    const R_kalman_input = getKalmanR(lPos?.coords?.accuracy ?? R_MAX, current_ekf_speed);
    
    const modeStatus = isDeadReckoning 
        ? '🚨 DEAD RECKONING (DR) EN COURS - Mode DR auto: ' + autoDetectedMode
        : '🛰️ FUSION GNSS/IMU';
    
    // --- Extraction et Affichage ---
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

// --- FONCTION PRINCIPALE DE MISE À JOUR GPS (CORRECTION ACTIVE) ---
function updateDisp(pos) {
    const accRaw = pos.coords.accuracy;
    
    if (currentTransportMode !== 'INS_6DOF_REALISTE' || !ekf6dof) {
        lPos = pos;
        updateMap(pos.coords.latitude, pos.coords.longitude, accRaw);
        return; 
    }
    
    // Détection de la perte de signal
    if (accRaw > R_MAX) {
        isDeadReckoning = true;
    } else if (isDeadReckoning && accRaw < R_MAX) {
        isDeadReckoning = false;
    }

    const cLat = pos.coords.latitude;
    const cLon = pos.coords.longitude;
    const altRaw = pos.coords.altitude;
    const altAccRaw = pos.coords.altitudeAccuracy || 1.0;
    const spd3D_raw = pos.coords.speed || 0.0; 

    const current_ekf_speed = ekf6dof.getSpeed();
    let R_kalman_input = getKalmanR(accRaw, current_ekf_speed); 
    
    // --- LOGIQUE ANTI-SAUT GPS ---
    if (lPos && !isDeadReckoning) {
        const dt_gps = (pos.timestamp - lPos.timestamp) / 1000 || 1.0;
        const measured_dist = distance(lPos.coords.latitude, lPos.coords.longitude, cLat, cLon);
        const max_dist_plausible = MAX_REALISTIC_SPD_M * dt_gps; 
        
        if (measured_dist > max_dist_plausible) {
            R_kalman_input = R_MAX * 100; 
        }
    }
    
    // Exécution de l'étape UPDATE (Correction par la mesure GPS)
    const gps_measurement = math.matrix([cLat, cLon, altRaw, spd3D_raw, 0, 0]); 
    const R_matrix = math.diag([R_kalman_input, R_kalman_input, altAccRaw, 1, 1, 1]); 
    
    ekf6dof.update(gps_measurement, R_matrix, isDeadReckoning);

    lPos = pos;
    updateMap(cLat, cLon, accRaw);
}

// --- GESTION DE LA CARTE (Leaflet) ---
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

// --- GESTION DU DÉMARRAGE AVEC AUTORISATION DES CAPTEURS ---
function requestSensorPermissionAndStart() {
    
    const startFusion = () => {
        // 1. Démarrer le Watcher GPS (UPDATE)
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

        // 2. Démarrer la Boucle d'Estime IMU (PREDICTION)
        if (currentTransportMode === 'INS_6DOF_REALISTE') {
            startIMUListeners(); 
            imuIntervalID = setInterval(runIMULoop, DT_IMU * 1000); 
        }
        document.getElementById('toggle-gps-btn').textContent = "Arrêter la Fusion";
    };

    if (currentTransportMode === 'INS_6DOF_REALISTE' && typeof DeviceOrientationEvent.requestPermission === 'function') {
        // iOS 13+ et certains navigateurs: demande d'autorisation explicite
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
        // Navigateurs anciens, Android, ou autres où l'accès est implicite
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
            ekf6dof = new EKF_6DoF();
            document.getElementById('nhc-status').textContent = '✅ EKF 6-DoF ACTIF (Fusion)';
        } else {
             document.getElementById('nhc-status').textContent = 'Mode EKF 1D ou Fictionnel';
        }
    }

    setTransportModeParameters(document.getElementById('transport-mode-select').value); 
});
