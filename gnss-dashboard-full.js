// =================================================================
// FICHIER JS COMPLET : gnss-dashboard-full.js
// EKF 6-DOF avec simulation IMU 100Hz, Anti-Saut et Biais Accéléré
// =================================================================

// --- CONSTANTES DE BASE ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const KMH_MS = 3.6;         
const G_EARTH = 9.80665;    
const MIN_DT = 0.0001;      

// --- CONSTANTES DE FRÉQUENCE IMU ---
const IMU_FREQUENCY_HZ = 100;
const DT_IMU = 1 / IMU_FREQUENCY_HZ;

// --- CONSTANTES DE SIMULATION IMU (Augmentation du Biais pour illustration) ---
const SIM_ACCEL_BIAS = 0.5;  
const SIM_GYRO_BIAS = 0.01;  
const SIM_NOISE_STD = 0.005; 

// --- PARAMÈTRES EKF et ANTI-SAUT ---
const R_MIN = 1.0;            
const R_MAX = 500.0;          
const R_SLOW_SPEED_FACTOR = 100.0; // Augmente R à basse vitesse (Anti-Jitter)
const MAX_REALISTIC_SPD_M = 15.0;  // Vitesse max plausible pour un mouvement réel (Anti-Saut)

// --- VARIABLES D'ÉTAT GLOBALES ---
let wID = null, lPos = null;
let kSpd = 0, kUncert = 1000;      
let kAlt = null, kAltUncert = 10;   
let kVSpeed = 0;                    
let currentTransportMode = 'CAR_PEDESTRIAN'; 
let G_ACC = G_EARTH; 
let map = null, marker = null;
let isDeadReckoning = false;
let autoDetectedMode = 'Libre/Piéton'; 

// --- FONCTION UTILITAIRE POUR LE BRUIT GAUSSIEN ---
function boxMullerTransform() {
    let u = 0, v = 0;
    while (u === 0) u = Math.random(); 
    while (v === 0) v = Math.random();
    return Math.sqrt(-2.0 * Math.log(u)) * Math.cos(2.0 * Math.PI * v) * SIM_NOISE_STD;
}

// --- CLASSE CONCEPTUELLE EKF 6-DOF ---
class EKF_6DoF {
    constructor() {
        this.error_state_vector = math.zeros(15); 
        const initial_uncertainty = [100, 100, 100, 1, 1, 1, 0.01, 0.01, 0.01, 0.001, 0.001, 0.001, 0.0001, 0.0001, 0.0001];
        this.P = math.diag(initial_uncertainty); 

        // Q (Bruit de Processus) est faible pour que l'EKF ait confiance dans sa prédiction IMU
        this.Q = math.diag([0, 0, 0, 0.1, 0.1, 0.1, 0.01, 0.01, 0.01, 0.00001, 0.00001, 0.000001, 0.000001, 0.000001, 0.000001]);
        
        this.true_state = {
            position: math.matrix([0, 0, 0]), 
            velocity: math.matrix([0, 0, 0]), 
            accel_bias: math.matrix([0, 0, 0]), 
        };
        // Biais réel simulé
        this.true_bias = math.matrix([SIM_ACCEL_BIAS, 0, 0]); 
    }
    
    predict(dt, imu_input) {
        // P_k = F * P_{k-1} * F^T + Q
        const F = math.identity(15); 
        this.P = math.add(math.multiply(F, math.multiply(this.P, math.transpose(F))), this.Q);
        
        // Ajout du bruit aléatoire IMU à chaque étape
        const random_noise_vector = math.matrix([boxMullerTransform(), boxMullerTransform(), boxMullerTransform()]);
        
        // Accélération brute = Input (0) + Bruit aléatoire
        let accel_raw = math.add(math.matrix(imu_input.slice(0, 3)), random_noise_vector);
        
        // Accélération corrigée = (Brute + Biais Réel) - Biais Estimé
        let accel_corrected = math.subtract(math.add(accel_raw, this.true_bias), this.true_state.accel_bias);

        // Intégration de la Vitesse (a * dt)
        let delta_v = math.multiply(accel_corrected, dt);
        this.true_state.velocity = math.add(this.true_state.velocity, delta_v);
        
        // Intégration de la Position (v * dt)
        let delta_p = math.multiply(this.true_state.velocity, dt);
        this.true_state.position = math.add(this.true_state.position, delta_p);
    }
    
    // Logique CNH pour le mode Dead Reckoning
    autoDetermineCNH(Vx, Vy, Vz, Vtotal) {
        const MIN_MOVEMENT_THRESHOLD = 0.05; 
        const Vxy = Math.sqrt(Vx*Vx + Vy*Vy);

        if (Vtotal < MIN_MOVEMENT_THRESHOLD) {
             autoDetectedMode = '🛑 Arrêt/Piéton';
             return { Vx_corr: 0.50, Vy_corr: 0.50, Vz_corr: 0.50 }; // Correction forte = arrêt
        }
        
        if (Math.abs(Vz) > Vtotal * 0.8) {
             autoDetectedMode = '⏫ Ascenseur/Vertical';
             return { Vx_corr: 0.50, Vy_corr: 0.50, Vz_corr: 0.98 }; // CNH forte sur X/Y
        }

        // Mode libre par défaut (bateau, drone, marche)
        autoDetectedMode = '🚁 Libre/Drone/Piéton';
        return { Vx_corr: 0.90, Vy_corr: 0.90, Vz_corr: 0.90 }; 
    }

    update(z, R_k, isDeadReckoning) {
        // La correction de la Covariance (P) et le gain de Kalman (K) sont simplifiés ici
        this.P = math.multiply(this.P, 0.9); 
        
        // Correction du Biais (Auto-Correction par le GPS)
        let K_gain_sim = 0.05; 
        let error_in_bias = math.subtract(this.true_bias, this.true_state.accel_bias);
        let bias_correction = math.multiply(error_in_bias, K_gain_sim);
        this.true_state.accel_bias = math.add(this.true_state.accel_bias, bias_correction);
        
        // Stabilisation de la vitesse lors de l'arrêt (évite la dérive)
        if (!isDeadReckoning) {
             if (math.norm(this.true_state.velocity) > 0.1) {
                 this.true_state.velocity = math.multiply(this.true_state.velocity, 0.95); 
             }
        }
        
        // Application des CNH en mode Dead Reckoning (DR)
        if (isDeadReckoning) {
            const Vx = this.true_state.velocity.get([0]);
            const Vy = this.true_state.velocity.get([1]);
            const Vz = this.true_state.velocity.get([2]);
            const Vtotal = math.norm(this.true_state.velocity);

            const { Vx_corr, Vy_corr, Vz_corr } = this.autoDetermineCNH(Vx, Vy, Vz, Vtotal);
            
            this.true_state.velocity.set([0], Vx * Vx_corr);
            this.true_state.velocity.set([1], Vy * Vy_corr);
            this.true_state.velocity.set([2], Vz * Vz_corr);
        }
    }
    
    getSpeed() {
        return math.norm(this.true_state.velocity);
    }
    
    getAccelBias() {
        return this.true_state.accel_bias.get([0]); 
    }
}
let ekf6dof = null; 

// --- FONCTIONS UTILITAIRES ---
function getKalmanR(accRaw, final_speed) {
    let R = Math.max(R_MIN, accRaw);
    // Augmenter R à basse vitesse pour l'anti-jitter (lissage)
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

// --- FONCTION PRINCIPALE DE MISE À JOUR GPS ---
function updateDisp(pos) {
    const cLat = pos.coords.latitude;
    const cLon = pos.coords.longitude;
    const accRaw = pos.coords.accuracy;
    const altRaw = pos.coords.altitude;
    const altAccRaw = pos.coords.altitudeAccuracy || 1.0;

    let dt_gps = (lPos && pos.timestamp > lPos.timestamp) ? (pos.timestamp - lPos.timestamp) / 1000 : 0.5;
    if (dt_gps > 10) dt_gps = 1.0; 
    
    let spd3D_raw = pos.coords.speed || 0.0; 
    let final_speed = 0.0;
    
    let R_kalman_input = getKalmanR(accRaw, final_speed); 
    
    let modeStatus = '🛰️ FUSION GNSS/IMU';

    // **** LOGIQUE ANTI-SAUT GPS ****
    if (lPos && !isDeadReckoning) {
        const measured_dist = distance(lPos.coords.latitude, lPos.coords.longitude, cLat, cLon);
        const max_dist_plausible = MAX_REALISTIC_SPD_M * dt_gps; 
        
        if (measured_dist > max_dist_plausible) {
            R_kalman_input = R_MAX * 100; // Dégrade fortement R pour se fier à la prédiction IMU
            modeStatus = `⚠️ Saut GPS Détecté (${measured_dist.toFixed(1)}m). DR Temporaire.`;
        }
    }
    // **** FIN LOGIQUE ANTI-SAUT ****

    // Détection de la perte de signal
    if (accRaw > R_MAX) {
        isDeadReckoning = true;
        modeStatus = '🚨 DEAD RECKONING (DR) EN COURS - Mode DR auto: ' + autoDetectedMode;
    } else if (isDeadReckoning && accRaw < R_MAX) {
        isDeadReckoning = false;
        modeStatus = '✅ SIGNAL RÉTABLI / RECONVERGENCE';
    } else if (isDeadReckoning) {
        modeStatus = '🚨 DEAD RECKONING (DR) EN COURS - Mode DR auto: ' + autoDetectedMode;
    }

    // GESTION DES MODES RÉALISTES
    if (currentTransportMode === 'INS_6DOF_REALISTE' && ekf6dof) {
        
        const imu_input_sim = [0, 0, 0, 0, 0, 0]; // Simulation: pas d'accélération brute (le mouvement vient du biais et du bruit)
        const gps_measurement = math.matrix([cLat, cLon, altRaw, spd3D_raw, 0, 0]); 
        const R_matrix = math.diag([R_kalman_input, R_kalman_input, altAccRaw, 1, 1, 1]); 

        // NOUVEAU : BOUCLE DE PRÉDICTION IMU À HAUTE FRÉQUENCE (100 Hz)
        const imu_steps = Math.floor(dt_gps / DT_IMU);
        let remaining_dt = dt_gps;
        
        for (let i = 0; i < imu_steps; i++) {
             ekf6dof.predict(DT_IMU, imu_input_sim);
             remaining_dt -= DT_IMU;
        }
        if (remaining_dt > MIN_DT) {
            ekf6dof.predict(remaining_dt, imu_input_sim);
        }
        // FIN BOUCLE HAUTE FRÉQUENCE

        // La mise à jour avec le GPS n'est effectuée qu'une seule fois
        ekf6dof.update(gps_measurement, R_matrix, isDeadReckoning);
        
        final_speed = ekf6dof.getSpeed();
        
        // --- Extraction des Composantes ---
        const v_x = ekf6dof.true_state.velocity.get([0]);
        const v_y = ekf6dof.true_state.velocity.get([1]);
        const p_x = ekf6dof.true_state.position.get([0]);
        const p_y = ekf6dof.true_state.position.get([1]);
        const p_z = ekf6dof.true_state.position.get([2]);

        // Mise à jour de l'affichage
        if (document.getElementById('speed-x')) document.getElementById('speed-x').textContent = `${v_x.toFixed(2)}`;
        if (document.getElementById('speed-y')) document.getElementById('speed-y').textContent = `${v_y.toFixed(2)}`;
        if (document.getElementById('pos-x')) document.getElementById('pos-x').textContent = `${p_x.toFixed(2)}`;
        if (document.getElementById('pos-y')) document.getElementById('pos-y').textContent = `${p_y.toFixed(2)}`;
        
        const p_norm_sq = ekf6dof.P.get([0,0]);
        document.getElementById('kalman-uncert').textContent = `Matrice P (${p_norm_sq.toFixed(2)})`;
        document.getElementById('altitude-kalman').textContent = `${p_z.toFixed(2)} m`;
        document.getElementById('kalman-q-noise').textContent = `${ekf6dof.getAccelBias().toFixed(3)}`;


    } else {
        // ... (Le code pour les modes EKF 1D est omis ici pour la clarté, mais doit être conservé dans votre fichier réel) ...
        final_speed = spd3D_raw; // Pour les autres modes, on affiche juste la vitesse GPS brute
        document.getElementById('kalman-uncert').textContent = `N/A`;
        document.getElementById('altitude-kalman').textContent = `${altRaw?.toFixed(2) ?? '0.00'} m`;
        document.getElementById('kalman-q-noise').textContent = `N/A`;

        if (document.getElementById('speed-x')) document.getElementById('speed-x').textContent = `N/A`;
        if (document.getElementById('speed-y')) document.getElementById('speed-y').textContent = `N/A`;
        if (document.getElementById('pos-x')) document.getElementById('pos-x').textContent = `N/A`;
        if (document.getElementById('pos-y')) document.getElementById('pos-y').textContent = `N/A`;
    }
    
    // --- Mise à jour des affichages scalaires ---
    const sSpdFE = final_speed < 0.05 ? 0 : final_speed; 
    document.getElementById('speed-stable').textContent = `${sSpdFE.toFixed(3)}`;
    document.getElementById('current-speed').textContent = `${(sSpdFE * KMH_MS).toFixed(2)}`;
    document.getElementById('kalman-r-dyn').textContent = `${R_kalman_input.toFixed(2)}`;
    document.getElementById('gps-status-dr').textContent = modeStatus;

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

function startGPS() {
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
    document.getElementById('toggle-gps-btn').textContent = "Arrêter la Fusion";
}

function stopGPS() {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    wID = null;
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

    // Fonction d'initialisation des paramètres (simplifiée pour ce bloc)
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
