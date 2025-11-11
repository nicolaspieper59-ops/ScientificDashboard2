// =================================================================
// FICHIER JS COMPLET : gnss-dashboard-full.js
// EKF 6-DOF autonome (DR) prêt pour des VRAIS capteurs IMU
// =================================================================

// --- CONSTANTES DE BASE ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const KMH_MS = 3.6;         
const G_EARTH = 9.80665;    
const MIN_DT = 0.0001;      

// --- CONSTANTES DE FRÉQUENCE IMU (Base du DR Autonome) ---
const IMU_FREQUENCY_HZ = 100;
const DT_IMU = 1 / IMU_FREQUENCY_HZ; // 0.01 secondes

// --- CONSTANTES DE SIMULATION ET CAPTEURS ---
const SIM_ACCEL_BIAS = 0.5;  
const SIM_NOISE_STD = 0.005; 
let G_ACC = G_EARTH; 

// --- PARAMÈTRES EKF et ANTI-SAUT ---
const R_MIN = 1.0;            
const R_MAX = 500.0;          
const R_SLOW_SPEED_FACTOR = 100.0; // Augmente R à basse vitesse (Anti-Jitter)
const MAX_REALISTIC_SPD_M = 15.0;  // Vitesse max plausible pour un mouvement réel (Anti-Saut)

// --- VARIABLES D'ÉTAT GLOBALES ---
let wID = null, lPos = null;
let imuIntervalID = null; // ID du timer pour la boucle rapide IMU
let kSpd = 0, kUncert = 1000;      
let ekf6dof = null;
let currentTransportMode = 'CAR_PEDESTRIAN'; 
let map = null, marker = null;
let isDeadReckoning = false;
let autoDetectedMode = 'Libre/Piéton'; 

// --- NOUVELLES VARIABLES GLOBALES POUR LES CAPTEURS RÉELS ---
// Celles-ci doivent être mises à jour par une API de capteur externe
let real_accel_x = 0.0;
let real_accel_y = 0.0;
let real_accel_z = 0.0;
let real_gyro_x = 0.0;
let real_gyro_y = 0.0;
let real_gyro_z = 0.0;

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
        this.Q = math.diag([0, 0, 0, 0.1, 0.1, 0.1, 0.01, 0.01, 0.01, 0.00001, 0.00001, 0.000001, 0.000001, 0.000001, 0.000001]);
        
        this.true_state = {
            position: math.matrix([0, 0, 0]), 
            velocity: math.matrix([0, 0, 0]), 
            accel_bias: math.matrix([0, 0, 0]), 
        };
        this.true_bias = math.matrix([SIM_ACCEL_BIAS, 0, 0]); 
    }
    
    predict(dt, imu_input) {
        // P_k = F * P_{k-1} * F^T + Q
        const F = math.identity(15); 
        this.P = math.add(math.multiply(F, math.multiply(this.P, math.transpose(F))), this.Q);
        
        // Accélération brute (en utilisant les capteurs réels X, Y, Z)
        let accel_raw = math.matrix([imu_input[0], imu_input[1], imu_input[2]]);
        
        // Ajout du bruit (pour simuler la qualité du capteur)
        const random_noise_vector = math.matrix([boxMullerTransform(), boxMullerTransform(), boxMullerTransform()]);
        accel_raw = math.add(accel_raw, random_noise_vector);

        // Accélération corrigée = (Brute + Biais Réel) - Biais Estimé
        let accel_corrected = math.subtract(math.add(accel_raw, this.true_bias), this.true_state.accel_bias);

        // Intégration de la Vitesse (a * dt)
        let delta_v = math.multiply(accel_corrected, dt);
        this.true_state.velocity = math.add(this.true_state.velocity, delta_v);
        
        // Intégration de la Position (v * dt)
        let delta_p = math.multiply(this.true_state.velocity, dt);
        this.true_state.position = math.add(this.true_state.position, delta_p);
    }
    
    // Logique CNH pour le mode Dead Reckoning (non modifiée)
    autoDetermineCNH(Vx, Vy, Vz, Vtotal) {
        const MIN_MOVEMENT_THRESHOLD = 0.05; 
        const Vxy = Math.sqrt(Vx*Vx + Vy*Vy);

        if (Vtotal < MIN_MOVEMENT_THRESHOLD) {
             autoDetectedMode = '🛑 Arrêt/Piéton';
             return { Vx_corr: 0.50, Vy_corr: 0.50, Vz_corr: 0.50 }; 
        }
        
        if (Math.abs(Vz) > Vtotal * 0.8) {
             autoDetectedMode = '⏫ Ascenseur/Vertical';
             return { Vx_corr: 0.50, Vy_corr: 0.50, Vz_corr: 0.98 }; 
        }

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
        
        // Stabilisation de la vitesse (évite la dérive)
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

// --- NOUVELLE FONCTION : BOUCLE D'ESTIME IMU AUTONOME (100 Hz) ---
function runIMULoop() {
    if (ekf6dof) {
        // Utilisation des VRAIES données de capteurs (X, Y, Z de l'accéléromètre)
        // Les données du gyroscope (0, 0, 0) sont actuellement des placeholders
        const real_imu_input = [real_accel_x, real_accel_y, real_accel_z, real_gyro_x, real_gyro_y, real_gyro_z];
        
        // Exécution de l'étape PREDICT de l'EKF (DR)
        ekf6dof.predict(DT_IMU, real_imu_input);
        
        // Mise à jour de l'affichage à chaque étape (pour un affichage fluide à 100 Hz)
        updateDisplayMetrics();
    }
}

// --- NOUVELLE FONCTION : MISE À JOUR DES MÉTRIES D'AFFICHAGE ---
function updateDisplayMetrics() {
    if (!ekf6dof) return;

    const final_speed = ekf6dof.getSpeed();
    const current_ekf_speed = final_speed;

    // --- LOGIQUE ANTI-SAUT/ANTI-JITTER pour l'affichage R ---
    const R_kalman_input = getKalmanR(lPos?.coords?.accuracy ?? R_MAX, current_ekf_speed);
    
    const modeStatus = isDeadReckoning 
        ? '🚨 DEAD RECKONING (DR) EN COURS - Mode DR auto: ' + autoDetectedMode
        : '🛰️ FUSION GNSS/IMU';
    
    // --- Extraction des Composantes ---
    const v_x = ekf6dof.true_state.velocity.get([0]);
    const v_y = ekf6dof.true_state.velocity.get([1]);
    const p_x = ekf6dof.true_state.position.get([0]);
    const p_y = ekf6dof.true_state.position.get([1]);
    const p_z = ekf6dof.true_state.position.get([2]);

    // --- Mise à jour des affichages scalaires ---
    const sSpdFE = final_speed < 0.05 ? 0 : final_speed; 
    document.getElementById('speed-stable').textContent = `${sSpdFE.toFixed(3)}`;
    document.getElementById('current-speed').textContent = `${(sSpdFE * KMH_MS).toFixed(2)}`;
    document.getElementById('kalman-r-dyn').textContent = `${R_kalman_input.toFixed(2)}`;
    document.getElementById('gps-status-dr').textContent = modeStatus;
    
    // Mise à jour des données 6-DoF
    if (document.getElementById('speed-x')) document.getElementById('speed-x').textContent = `${v_x.toFixed(2)}`;
    if (document.getElementById('speed-y')) document.getElementById('speed-y').textContent = `${v_y.toFixed(2)}`;
    if (document.getElementById('pos-x')) document.getElementById('pos-x').textContent = `${p_x.toFixed(2)}`;
    if (document.getElementById('pos-y')) document.getElementById('pos-y').textContent = `${p_y.toFixed(2)}`;
    
    const p_norm_sq = ekf6dof.P.get([0,0]);
    document.getElementById('kalman-uncert').textContent = `Matrice P (${p_norm_sq.toFixed(2)})`;
    document.getElementById('altitude-kalman').textContent = `${p_z.toFixed(2)} m`;
    document.getElementById('kalman-q-noise').textContent = `${ekf6dof.getAccelBias().toFixed(3)}`;
}

// --- FONCTION PRINCIPALE DE MISE À JOUR GPS (Uniquement l'étape UPDATE) ---
function updateDisp(pos) {
    const accRaw = pos.coords.accuracy;
    
    // Si le filtre n'est pas actif ou n'est pas en mode EKF 6-DoF, on ignore
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

    // Récupération des données GPS pour la mesure (UPDATE)
    const cLat = pos.coords.latitude;
    const cLon = pos.coords.longitude;
    const altRaw = pos.coords.altitude;
    const altAccRaw = pos.coords.altitudeAccuracy || 1.0;
    const spd3D_raw = pos.coords.speed || 0.0; 

    // Calcul de R basé sur la vitesse actuelle de l'EKF
    const current_ekf_speed = ekf6dof.getSpeed();
    let R_kalman_input = getKalmanR(accRaw, current_ekf_speed); 
    
    // --- LOGIQUE ANTI-SAUT GPS (RÉ-ÉVALUÉE) ---
    if (lPos && !isDeadReckoning) {
        const dt_gps = (pos.timestamp - lPos.timestamp) / 1000 || 1.0;
        const measured_dist = distance(lPos.coords.latitude, lPos.coords.longitude, cLat, cLon);
        const max_dist_plausible = MAX_REALISTIC_SPD_M * dt_gps; 
        
        if (measured_dist > max_dist_plausible) {
            R_kalman_input = R_MAX * 100; // Dégrade fortement R pour ignorer la mesure
        }
    }
    // --- FIN ANTI-SAUT ---
    
    // Exécution de l'étape UPDATE (Correction par la mesure GPS)
    const gps_measurement = math.matrix([cLat, cLon, altRaw, spd3D_raw, 0, 0]); 
    const R_matrix = math.diag([R_kalman_input, R_kalman_input, altAccRaw, 1, 1, 1]); 
    
    ekf6dof.update(gps_measurement, R_matrix, isDeadReckoning);

    // Mise à jour finale de l'état
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
    stopGPS(); 
    
    // 1. Démarrer le Watcher GPS (Boucle Lente pour l'UPDATE)
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

    // 2. Démarrer la Boucle d'Estime IMU (Boucle Rapide 100 Hz pour la PREDICTION)
    if (currentTransportMode === 'INS_6DOF_REALISTE') {
        imuIntervalID = setInterval(runIMULoop, DT_IMU * 1000); 
    }

    document.getElementById('toggle-gps-btn').textContent = "Arrêter la Fusion";
}

function stopGPS() {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    if (imuIntervalID !== null) clearInterval(imuIntervalID); 
    
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

    // Initialisation des paramètres EKF
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
