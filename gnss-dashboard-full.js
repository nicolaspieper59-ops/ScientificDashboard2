// =================================================================
// FICHIER JS COMPLET : gnss-dashboard-full.js
// EKF de RÉALITÉ ADAPTATIF : Modes Réalistes (INS 6-DoF Conceptuel) et Fictionnels
// =================================================================

// --- CONSTANTES DE BASE ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const KMH_MS = 3.6;         
const G_EARTH = 9.80665;    
const MIN_DT = 0.0001;      // Temps minimal pour éviter division par zéro
const SPACETIME_JUMP_THRESHOLD_M = 500000; 

// --- PARAMÈTRES EKF (Ajustés par le mode de transport) ---
let Q_NOISE_H = 0.1;        
let Q_NOISE_V = 0.01;       
let R_MIN = 0.01;           
let R_MAX = 500.0;          
let G_NULL_FACTOR = 1.0;    // Facteur d'annulation d'inertie (0.0 pour l'OVNI/Magie)
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

// --- VARIABLES D'ÉTAT GLOBALES (pour le EKF 1D) ---
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

// --- CLASSE CONCEPTUELLE EKF 6-DOF ---
class EKF_6DoF {
    constructor() {
        // État du filtre d'erreur (15x1)
        this.error_state_vector = math.zeros(15); 
        const initial_uncertainty = [100, 100, 100, 1, 1, 1, 0.01, 0.01, 0.01, 0.001, 0.001, 0.001, 0.0001, 0.0001, 0.0001];
        this.P = math.diag(initial_uncertainty); // Covariance (15x15)

        // Q simplifié pour le mode réaliste
        this.Q = math.diag([0, 0, 0, 0.1, 0.1, 0.1, 0.01, 0.01, 0.01, 0.00001, 0.00001, 0.00001, 0.000001, 0.000001, 0.000001]);
        
        // État réel (non-linéaire) - Position, Vitesse, Quaternion
        this.true_state = {
            position: math.matrix([0, 0, 0]), 
            velocity: math.matrix([0, 0, 0]), 
            orientation: [1, 0, 0, 0], 
            accel_bias: [0, 0, 0],
            gyro_bias: [0, 0, 0]
        };
        console.log("EKF 6-DoF initialisé. Utilise mathjs.");
    }
    
    // --- METHODES PRINCIPALES (Squelette Logique) ---
    predict(dt, imu_input) {
        // En mode réel, F et la propagation du true_state seraient calculées ici.
        // P_k = F * P_{k-1} * F^T + Q
        // true_state_k = f(true_state_{k-1}, imu_input)
        const F = math.identity(15); // Placeholder F
        this.P = math.add(math.multiply(F, math.multiply(this.P, math.transpose(F))), this.Q);
    }
    
    update(z, R_k) {
        // Correction de l'état avec la mesure GPS (z)
        const H = math.zeros(6, 15); H.subset(math.index([0, 1, 2, 3, 4, 5], [0, 1, 2, 3, 4, 5]), math.identity(6)); // Simplifié H (mesure p, v)
        // K = P * H^T * inv(H * P * H^T + R)
        // ... calcul du Gain de Kalman K ...
        // P = (I - K * H) * P
        // this.true_state est corrigé.
    }
    
    // Renvoie la vitesse 3D pour l'affichage
    getSpeed() {
        return math.norm(this.true_state.velocity);
    }
}
let ekf6dof = null; // Instance globale pour le 6-DoF

// --- FONCTIONS DE GESTION DES MODES ET DES CONSTANTES ---
function setDimensionalConstants(dimension) {
    const data = DIMENSIONAL_CONSTANTS[dimension];
    if (data) G_ACC = data.G; 
    document.getElementById('nhc-status').textContent = `Gravité Locale: ${G_ACC.toFixed(2)} m/s²`;
}

function setTransportModeParameters(mode) {
    currentTransportMode = mode;
    
    // Réinitialisation de base
    Q_NOISE_H = 0.1; Q_NOISE_V = 0.01; R_SLOW_SPEED_FACTOR = 100.0; G_NULL_FACTOR = 1.0; 
    setDimensionalConstants('STANDARD'); 
    let status_text = 'INACTIF';

    switch (mode) {
        case 'INS_6DOF_REALISTE':
            if (!ekf6dof) ekf6dof = new EKF_6DoF(); // Créer l'instance 6-DoF
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
    
    // Application de la G-Nullification
    const effective_accel_input = accel_input * G_NULL_FACTOR; 
    
    let Q_accel_noise = Q_NOISE_H + Math.abs(effective_accel_input) * 0.5; 
    
    // Contrainte Latérale Avion
    if (currentTransportMode === 'AIRCRAFT' && Math.abs(lateralAccelIMU) < 0.5) Q_accel_noise *= 0.1;
    
    const R = R_dyn ?? R_MAX, Q = Q_accel_noise * dt * dt; 
    
    // Prédiction et Correction
    let pSpd = kSpd + (effective_accel_input * dt); 
    let pUnc = kUncert + Q; 

    let K = pUnc / (pUnc + R); 
    kSpd = pSpd + K * (nSpd - pSpd); 
    kUncert = (1 - K) * pUnc; 
    
    return kSpd;
}

function kFilterAltitude(altRaw, R_alt_raw, dt) { 
    if (kAlt === null) { kAlt = altRaw; return kAlt; }
    if (dt === 0 || dt > 5) return kAlt; 

    const accel_vertical_raw = rawAccelWithGrav.z - G_ACC; 
    const effective_accel_vertical = accel_vertical_raw * G_NULL_FACTOR;

    const Q_V_current = Q_NOISE_V * dt * dt * 0.1; 
    const R_alt = R_alt_raw * R_alt_raw; 

    // Prédiction
    let pAlt = kAlt + (kVSpeed * dt) + (0.5 * effective_accel_vertical * dt * dt); 
    let pVSpeed = kVSpeed + (effective_accel_vertical * dt); 
    
    kAltUncert = kAltUncert + Q_V_current; 

    // Correction
    let K = kAltUncert / (kAltUncert + R_alt); 
    kAlt = pAlt + K * (altRaw - pAlt); 
    kAltUncert = (1 - K) * kAltUncert; 
    kVSpeed = pVSpeed + K * ((altRaw - pAlt) / dt);

    return kAlt;
}

// --- FONCTIONS UTILITAIRES ---
function getKalmanR(accRaw) {
    let R = Math.max(R_MIN, accRaw);
    // Soft Constraint
    if (kSpd < 0.5 && corrected_Accel_Mag < 0.05) R = R * R_SLOW_SPEED_FACTOR;
    return R;
}
function distance(lat1, lon1, lat2, lon2) {
    const R = 6371e3; // Rayon de la Terre en mètres
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
    corrected_Accel_Mag = spd3D_raw > 0 ? 0.1 : 0.0; // Accél. simplifiée
    
    let modeStatus = '🛰️ FUSION GNSS/IMU';
    let final_speed = 0.0;

    // 1. GESTION DES MODES DE FICTION ET DES SAUTS SPATIO-TEMPORELS
    const isFictionMode = ['SPACETIME_DRIVE', 'MAGICAL_REALITY', 'HYPERDIMENSIONAL_DRIVE'].includes(currentTransportMode);

    if (isFictionMode && lPos !== null) {
        const d_space = distance(lPos.coords.latitude, lPos.coords.longitude, cLat, cLon);
        
        if (d_space > SPACETIME_JUMP_THRESHOLD_M) {
            // Saut Temporel/Dimensionnel détecté
            dt = MIN_DT; 
            kUncert = R_MAX; 
            kSpd = spd3D_raw; 
            kAlt = altRaw; 
            modeStatus = '✨ MANIFESTATION PARANORMALE!';
            
            // Changement aléatoire de dimension dans le mode MAGICAL_REALITY
            if (currentTransportMode === 'MAGICAL_REALITY') {
                const keys = Object.keys(DIMENSIONAL_CONSTANTS);
                const randomDim = keys[Math.floor(Math.random() * keys.length)];
                setDimensionalConstants(randomDim);
            }
        }
    }
    
    // 2. GESTION DES MODES RÉALISTES
    if (currentTransportMode === 'AIRCRAFT' && accRaw > AIRCRAFT_GPS_DEGRADE_THRESHOLD) {
        R_kalman_input *= AIRCRAFT_DEGRADE_R_FACTOR;
        modeStatus = '⚠️ GPS DÉGRADÉ (Couloir/DR) !';
    }

    if (currentTransportMode === 'INS_6DOF_REALISTE' && ekf6dof) {
        // En mode 6-DoF conceptuel, nous simulons les entrées IMU
        const imu_input_sim = [0, 0, 0, 0, 0, 0]; 
        const gps_measurement = [cLat, cLon, altRaw, spd3D_raw, 0, 0]; // Simplifié
        const R_matrix = math.diag([accRaw, accRaw, altAccRaw, 1, 1, 1]); // Matrice de bruit GPS (6x6)

        // Propagation et Correction 6-DoF
        ekf6dof.predict(dt, imu_input_sim);
        ekf6dof.update(gps_measurement, R_matrix);
        
        final_speed = ekf6dof.getSpeed();
        // L'affichage de l'altitude, de l'incertitude P, etc. nécessiterait de décoder l'état du 6-DoF.

    } else {
        // EKF 1D (Modes Simples et Fictionnels)
        const fSpd = kFilter(spd3D_raw, dt, R_kalman_input, corrected_Accel_Mag); 
        kAlt = kFilterAltitude(altRaw, altAccRaw, dt); 
        final_speed = fSpd;
    }
    
    // --- Mise à jour des affichages ---
    const sSpdFE = final_speed < 0.05 ? 0 : final_speed; 
    document.getElementById('speed-stable').textContent = `${sSpdFE.toFixed(3)}`;
    document.getElementById('current-speed').textContent = `${(sSpdFE * KMH_MS).toFixed(2)}`;
    document.getElementById('altitude-kalman').textContent = `${(kAlt ?? altRaw).toFixed(2)}`;
    document.getElementById('kalman-uncert').textContent = `${kUncert.toFixed(2)}`;
    document.getElementById('kalman-q-noise').textContent = `${Q_NOISE_H.toFixed(3)}`;
    document.getElementById('kalman-r-dyn').textContent = `${R_kalman_input.toFixed(2)}`;
    document.getElementById('gps-status-dr').textContent = modeStatus;


    // Sauvegarde de l'état
    lPos = pos;
    updateMap(cLat, cLon, accRaw);
}

// --- GESTION DE LA CARTE (Leaflet) ---
function initMap() {
    map = L.map('map').setView([43.2965, 5.37], 13); // Centré sur Marseille
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
    wID = navigator.geolocation.watchPosition(updateDisp, (err) => {console.error(err)}, {
        enableHighAccuracy: true,
        timeout: 5000,
        maximumAge: 0
    });
    document.getElementById('toggle-gps-btn').textContent = "Arrêter la Fusion";
}

function stopGPS() {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    wID = null;
    document.getElementById('toggle-gps-btn').textContent = "Démarrer la Fusion (GPS/IMU)";
}

// --- INITIALISATION DES ÉVÉNEMENTS DOM ---
document.addEventListener('DOMContentLoaded', () => {
    
    initMap(); 
    
    document.getElementById('toggle-gps-btn').addEventListener('click', () => { wID === null ? startGPS() : stopGPS(); });
    
    // Listener pour le changement de mode de transport
    document.getElementById('transport-mode-select').addEventListener('change', (e) => {
        stopGPS(); // Arrêter la position avant de changer de mode pour réinitialiser l'état
        kSpd = 0; kUncert = 1000; lPos = null; 
        ekf6dof = null; // Réinitialiser l'instance 6-DoF
        setTransportModeParameters(e.target.value);
    });

    setTransportModeParameters(document.getElementById('transport-mode-select').value); 
});
