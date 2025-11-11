// =================================================================
// FICHIER JS COMPLET : gnss-dashboard-full.js
// EKF de RÉALITÉ ADAPTATIF : Modes Réalistes (INS 6-DoF Conceptuel) et Fictionnels
// (Ajout de la simulation IMU haute fréquence dans le 6-DoF)
// =================================================================

// --- CONSTANTES DE BASE ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const KMH_MS = 3.6;         
const G_EARTH = 9.80665;    
const MIN_DT = 0.0001;      
const SPACETIME_JUMP_THRESHOLD_M = 500000; 

// --- NOUVELLES CONSTANTES DE SIMULATION IMU ---
const SIM_ACCEL_LAT = 0.2;  // Accélération latérale simulée (m/s²)
const SIM_GYRO_YAW = 0.05;  // Taux de lacet simulé (rad/s)

// --- PARAMÈTRES EKF (Ajustés par le mode de transport) ---
let Q_NOISE_H = 0.5;        
let Q_NOISE_V = 0.05;       
let R_MIN = 1.0;            
let R_MAX = 500.0;          
let G_NULL_FACTOR = 1.0;    
let R_SLOW_SPEED_FACTOR = 100.0; 
let AIRCRAFT_GPS_DEGRADE_THRESHOLD = 50.0; 
let AIRCRAFT_DEGRADE_R_FACTOR = 5000.0;   

// --- CONSTANTES PHYSIQUES ET FICTIONNELLES ---
const DIMENSIONAL_CONSTANTS = {
    'STANDARD': { G: G_EARTH, AIR_DENSITY: 1.225 },
    'HYPER_SPACE': { G: 0.0, AIR_DENSITY: 0.0 }, 
    'CARTOON': { G: 0.1, AIR_DENSITY: 1.2 },        
    'MAGICAL_AURA': { G: 0.5, AIR_DENSITY: 0.5 } 
};
let G_ACC = G_EARTH; 

// --- VARIABLES D'ÉTAT GLOBALES ---
let wID = null, lPos = null;
let kSpd = 0, kUncert = 1000;      
let kAlt = null, kAltUncert = 10;   
let kVSpeed = 0;                    
let currentTransportMode = 'CAR_PEDESTRIAN'; 
let rawAccelWithGrav = { x: 0.0, y: 0.0, z: 0.0 };
let attitude = { roll: 0.0, pitch: 0.0, yaw: 0.0 }; 
let corrected_Accel_Mag = 0.0;      
let lateralAccelIMU = 0.0; 
let map = null, marker = null;
let isDeadReckoning = false;

// --- CLASSE CONCEPTUELLE EKF 6-DOF ---
class EKF_6DoF {
    constructor() {
        this.error_state_vector = math.zeros(15); 
        const initial_uncertainty = [100, 100, 100, 1, 1, 1, 0.01, 0.01, 0.01, 0.001, 0.001, 0.001, 0.0001, 0.0001, 0.0001];
        this.P = math.diag(initial_uncertainty); 

        this.Q = math.diag([0, 0, 0, 0.1, 0.1, 0.1, 0.01, 0.01, 0.01, 0.00001, 0.00001, 0.00001, 0.000001, 0.000001, 0.000001]);
        
        this.true_state = {
            position: math.matrix([0, 0, 0]), 
            velocity: math.matrix([0, 0, 0]), 
            orientation: [1, 0, 0, 0], 
            accel_bias: [0, 0, 0],
            gyro_bias: [0, 0, 0]
        };
    }
    
    predict(dt, imu_input) {
        // Propagation P_k = F * P_{k-1} * F^T + Q
        const F = math.identity(15); 
        this.P = math.add(math.multiply(F, math.multiply(this.P, math.transpose(F))), this.Q);
        
        // Propagation de la Vitesse Basée sur l'IMU (a * dt)
        const accel_corrected = math.matrix(imu_input.slice(0, 3)); 
        let delta_v = math.multiply(accel_corrected, dt);
        this.true_state.velocity = math.add(this.true_state.velocity, delta_v);
        
        // Propagation de la Position (v * dt)
        let delta_p = math.multiply(this.true_state.velocity, dt);
        this.true_state.position = math.add(this.true_state.position, delta_p);
    }
    
    update(z, R_k) {
        const H = math.zeros(6, 15); H.subset(math.index([0, 1, 2, 3, 4, 5], [0, 1, 2, 3, 4, 5]), math.identity(6)); 
        this.P = math.multiply(this.P, 0.9); 
    }
    
    getSpeed() {
        return math.norm(this.true_state.velocity);
    }
}
let ekf6dof = null; 

// --- FONCTIONS DE GESTION DES MODES ET DES CONSTANTES ---
function setDimensionalConstants(dimension) {
    const data = DIMENSIONAL_CONSTANTS[dimension];
    if (data) G_ACC = data.G; 
    document.getElementById('nhc-status').textContent = `Gravité Locale: ${G_ACC.toFixed(2)} m/s²`;
}

function setTransportModeParameters(mode) {
    currentTransportMode = mode;
    
    // Réinitialisation aux valeurs standard réalistes
    Q_NOISE_H = 0.5; Q_NOISE_V = 0.05; R_MIN = 1.0; R_MAX = 500.0;
    R_SLOW_SPEED_FACTOR = 100.0; G_NULL_FACTOR = 1.0; 
    setDimensionalConstants('STANDARD'); 
    let status_text = 'INACTIF';
    isDeadReckoning = false;

    switch (mode) {
        case 'INS_6DOF_REALISTE':
            if (!ekf6dof) ekf6dof = new EKF_6DoF();
            // Réinitialiser les états 6-DoF pour une nouvelle simulation
            ekf6dof.true_state.position = math.matrix([0, 0, 0]);
            ekf6dof.true_state.velocity = math.matrix([0, 0, 0]);

            status_text = '✅ EKF 6-DoF ACTIF (Conceptuel)';
            break;
        case 'TRAIN_METRO': Q_NOISE_H = 0.05; R_SLOW_SPEED_FACTOR = 200.0; status_text = 'ACTIF (Rail/NHC)'; break;
        case 'AIRCRAFT': R_SLOW_SPEED_FACTOR = 10.0; Q_NOISE_V = 0.001; Q_NOISE_H = 0.08; status_text = 'ACTIF (Aéronef/NHC)'; break;
        
        // --- Modes Fiction ---
        case 'CARTOON_PHYSICS': G_NULL_FACTOR = 0.5; Q_NOISE_H = 50.0; Q_NOISE_V = 50.0; setDimensionalConstants('CARTOON'); status_text = '🤪 LOIS CARTOON (G=0.1)'; break;
        case 'SPACETIME_DRIVE': G_NULL_FACTOR = 0.001; Q_NOISE_H = 10000.0; Q_NOISE_V = 10000.0; status_text = '⚡️ VOYAGE TEMPOREL (Δt Ignoré)'; break;
        case 'MAGICAL_REALITY': G_NULL_FACTOR = 0.0; Q_NOISE_H = 100000.0; Q_NOISE_V = 100000.0; R_MIN = 0.0001; setDimensionalConstants('MAGICAL_AURA'); status_text = '✨ MAGICAL REALITY (Observation Pure)'; break;
        case 'HYPERDIMENSIONAL_DRIVE': G_NULL_FACTOR = 0.0; Q_NOISE_H = 50000.0; Q_NOISE_V = 50000.0; setDimensionalConstants('HYPER_SPACE'); status_text = '🚀 HYPER-ESPACE (Vitesse Lumière+)'; break;
        
        case 'CAR_PEDESTRIAN':
        default: break;
    }
    document.getElementById('nhc-status').textContent = `Contraintes de Mouvement: ${status_text}`;
}

// --- EKF 1D (pour les modes non 6-DoF) ---
function kFilter(nSpd, dt, R_dyn, accel_input = 0) {
    if (dt === 0 || dt > 5) return kSpd; 
    
    const effective_accel_input = accel_input * G_NULL_FACTOR; 
    let Q_accel_noise = Q_NOISE_H + Math.abs(effective_accel_input) * 0.5; 
    
    const R_eff = isDeadReckoning ? R_MAX * 10 : R_dyn;

    const R = R_eff ?? R_MAX, Q = Q_accel_noise * dt * dt; 
    
    let pSpd = kSpd + (effective_accel_input * dt); 
    let pUnc = kUncert + Q; 

    let K = pUnc / (pUnc + R); 
    kSpd = pSpd + K * (nSpd - pSpd); 
    kUncert = (1 - K) * pUnc; 
    
    return kSpd;
}

function kFilterAltitude(altRaw, altAccRaw, dt) { 
    if (kAlt === null) { kAlt = altRaw; return kAlt; }
    if (dt === 0 || dt > 5) return kAlt; 

    const accel_vertical_raw = rawAccelWithGrav.z - G_ACC; 
    const effective_accel_vertical = accel_vertical_raw * G_NULL_FACTOR;

    const Q_V_current = Q_NOISE_V * dt * dt * 0.1; 
    const R_alt = isDeadReckoning ? R_MAX * 10 : (altAccRaw * altAccRaw); 

    let pAlt = kAlt + (kVSpeed * dt) + (0.5 * effective_accel_vertical * dt * dt); 
    let pVSpeed = kVSpeed + (effective_accel_vertical * dt); 
    
    kAltUncert = kAltUncert + Q_V_current; 

    let K = kAltUncert / (kAltUncert + R_alt); 
    kAlt = pAlt + K * (altRaw - pAlt); 
    kAltUncert = (1 - K) * kAltUncert; 
    kVSpeed = pVSpeed + K * ((altRaw - pAlt) / dt);

    return kAlt;
}

// --- FONCTIONS UTILITAIRES ---
function getKalmanR(accRaw) {
    let R = Math.max(R_MIN, accRaw);
    if (kSpd < 0.5 && corrected_Accel_Mag < 0.05) R = R * R_SLOW_SPEED_FACTOR;
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

    let dt = (lPos && pos.timestamp > lPos.timestamp) ? (pos.timestamp - lPos.timestamp) / 1000 : 0.5;
    let spd3D_raw = pos.coords.speed || 0.0; 
    
    let R_kalman_input = getKalmanR(accRaw);
    corrected_Accel_Mag = spd3D_raw > 0 ? 0.1 : 0.0; 
    
    let modeStatus = '🛰️ FUSION GNSS/IMU';
    let final_speed = 0.0;

    // Détection de la perte de signal ou de la réacquisition
    if (accRaw > R_MAX) {
        isDeadReckoning = true;
        modeStatus = '🚨 DEAD RECKONING (DR) EN COURS';
    } else if (isDeadReckoning && accRaw < R_MAX) {
        isDeadReckoning = false;
        modeStatus = '✅ SIGNAL RÉTABLI / RECONVERGENCE';
    } else if (isDeadReckoning) {
        modeStatus = '🚨 DEAD RECKONING (DR) EN COURS';
    }


    // 1. GESTION DES MODES DE FICTION
    const isFictionMode = ['SPACETIME_DRIVE', 'MAGICAL_REALITY', 'HYPERDIMENSIONAL_DRIVE'].includes(currentTransportMode);

    if (isFictionMode && lPos !== null) {
        const d_space = distance(lPos.coords.latitude, lPos.coords.longitude, cLat, cLon);
        if (d_space > SPACETIME_JUMP_THRESHOLD_M) {
            dt = MIN_DT; 
            kUncert = R_MAX; 
            kSpd = spd3D_raw; 
            modeStatus = '✨ MANIFESTATION PARANORMALE!';
            if (currentTransportMode === 'MAGICAL_REALITY') {
                const keys = Object.keys(DIMENSIONAL_CONSTANTS);
                setDimensionalConstants(keys[Math.floor(Math.random() * keys.length)]);
            }
        }
    }
    
    // 2. GESTION DES MODES RÉALISTES
    if (currentTransportMode === 'INS_6DOF_REALISTE' && ekf6dof) {
        
        // **** ENTRÉES IMU HAUTE FRÉQUENCE SIMULÉES ****
        const imu_input_sim = [0, SIM_ACCEL_LAT, 0, 0, 0, SIM_GYRO_YAW]; 
        
        const gps_measurement = math.matrix([cLat, cLon, altRaw, spd3D_raw, 0, 0]); 
        const R_matrix = math.diag([R_kalman_input, R_kalman_input, altAccRaw, 1, 1, 1]); 

        ekf6dof.predict(dt, imu_input_sim);
        if (!isDeadReckoning) {
            ekf6dof.update(gps_measurement, R_matrix);
        }
        
        final_speed = ekf6dof.getSpeed();
        
        // Affichage des données 6-DoF
        const p_norm_sq = ekf6dof.P.get([0,0]); // Utiliser l'incertitude sur la première composante de position
        document.getElementById('kalman-uncert').textContent = `Matrice P (${p_norm_sq.toFixed(2)})`;
        document.getElementById('altitude-kalman').textContent = `${ekf6dof.true_state.position.get([2]).toFixed(2)} m`;


    } else {
        // EKF 1D (Modes Simples et Fictionnels)
        const fSpd = kFilter(spd3D_raw, dt, R_kalman_input, corrected_Accel_Mag); 
        kAlt = kFilterAltitude(altRaw, altAccRaw, dt); 
        final_speed = fSpd;

        document.getElementById('kalman-uncert').textContent = `${kUncert.toFixed(2)}`;
        document.getElementById('altitude-kalman').textContent = `${(kAlt ?? altRaw).toFixed(2)}`;
    }
    
    // --- Mise à jour des affichages ---
    const sSpdFE = final_speed < 0.05 ? 0 : final_speed; 
    document.getElementById('speed-stable').textContent = `${sSpdFE.toFixed(3)}`;
    document.getElementById('current-speed').textContent = `${(sSpdFE * KMH_MS).toFixed(2)}`;
    document.getElementById('kalman-q-noise').textContent = `${Q_NOISE_H.toFixed(3)}`;
    document.getElementById('kalman-r-dyn').textContent = `${R_kalman_input.toFixed(2)}`;
    document.getElementById('gps-status-dr').textContent = modeStatus;

    // Sauvegarde de l'état
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
        kSpd = 0; kUncert = 1000; lPos = null; 
        ekf6dof = null;
        setTransportModeParameters(e.target.value);
    });

    setTransportModeParameters(document.getElementById('transport-mode-select').value); 
});
