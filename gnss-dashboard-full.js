// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        return (decimals === 0 ? '0' : '0.00') + suffix;
    }
    return val.toFixed(decimals) + suffix;
};
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        return (decimals === 0 ? '0.00e+0' : val.toExponential(decimals)) + suffix;
    }
    return val.toExponential(decimals) + suffix;
};

// =================================================================
// BLOC 1/4 : Constantes Globales et Configuration (MISE À JOUR)
// =================================================================

// --- CONSTANTES MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI; 
const KMH_MS = 3.6;         
const C_L = 299792458;      

// --- CONSTANTES GÉOPHYSIQUES (WGS84) ---
let G_ACC = 9.80665;         
const OMEGA_EARTH = 7.2921159e-5;
// ... autres constantes WGS84

// --- CONSTANTES ATMOSPHÉRIQUES (ISA Standard) ---
const BARO_ALT_REF_HPA = 1013.25;
const RHO_SEA_LEVEL = 1.225;
const R_AIR = 287.058;
const GAMMA = 1.4;

// --- NOUVEAU : CONSTANTES HYPERLOOP/DYNAMIQUE ---
const RHO_VACUUM = 0.00001; // Densité d'air très basse (1e-5 kg/m³) pour Hyperloop
const REFERENCE_DRAG_AREA = 0.1; // Surface de référence pour la Traînée (m²)
const DRAG_COEFFICIENT = 0.2; // Coefficient de Traînée (hypothèse)
const MASS_KG = 70.0; // Masse de l'objet (pour le calcul des forces)

// --- VARIABLES D'ÉTAT ---
let ukf = null;
let lastGPSPos = null;
let kSpd = 0; // Vitesse lissée EKF (m/s)
let kAlt = 0; // Altitude lissée EKF (m)
let kUncert = 0.01; // Incertitude EKF Vitesse (m²/s²)
let kAltUncert = 1.0; // Incertitude EKF Altitude (m)
let accel = { x: 0, y: 0, z: 0 }; // Accélération IMU brute
let emergencyStopActive = false;
let currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound = 340.29; // Vitesse du son initiale (m/s)
let isHyperloopActive = false; // NOUVEAU: État Hyperloop
let isolationStatus = "NON"; // NOUVEAU: Statut d'isolation (Avion/Souterrain)
let lat = 0, lon = 0; // Position EKF
let timeMoving = 0, distM = 0, maxSpd = 0;
let lastTimestamp = performance.now();
// ... autres variables (Temps, Météo, Carte)

// =================================================================
// BLOC 2/4 : Filtres, Modèles Cinématiques et Quaternion (MISE À JOUR)
// =================================================================

// --- CLASSE UKF PROFESSIONNELLE (Architecture 21 États) ---
class ProfessionalUKF {
    constructor() {
        // ... (Initialisation UKF math.js)
    }
    predict(imuData, dt) {
        // PLACEHOLDER: Simulation simplifiée de prédiction
        const ax_corr = imuData.accel[0] || 0; 
        let vN = this.x.get([3]) + ax_corr * dt; 
        
        // Mise à jour simplifiée de kSpd pour la démo
        kSpd = Math.abs(vN); 
    }
    update(gpsData) {
        // PLACEHOLDER: Correction directe de la position et de l'incertitude
        kAlt = gpsData.altitude; 
        kUncert = 1.0 + (gpsData.accuracy || 100) / 100;
        kAltUncert = 1.0 + (gpsData.verticalAccuracy || gpsData.accuracy || 100) / 10;
        lat = gpsData.latitude;
        lon = gpsData.longitude;
    }
    getState() {
        return {
            lat: lat, lon: lon, alt: kAlt, speed: kSpd,
            kUncert: kUncert, kAltUncert: kAltUncert
        };
    }
}


// --- FONCTIONS ATMOSPHÉRIQUES (MODIFIÉES pour Hyperloop) ---

// Fonction pour calculer la densité de l'air
function getAirDensity(P_hPa, T_K) {
    if (isHyperloopActive) {
        return RHO_VACUUM;
    }
    if (P_hPa === null || T_K === null) return RHO_SEA_LEVEL;
    const P_Pa = P_hPa * 100;
    return P_Pa / (R_AIR * T_K);
}

// Fonction pour calculer la Vitesse du Son
function getSpeedOfSound(T_K) {
    if (T_K === null) {
        return 340.29; 
    }
    // La vitesse du son dans le vide n'est pas applicable, on garde la valeur basée sur la température
    return Math.sqrt(GAMMA * R_AIR * T_K);
}

// --- FONCTIONS DYNAMIQUES (NOUVELLES) ---

function calculateDynamicMetrics(speed_ms, airDensity, soundSpeed) {
    const mach = speed_ms / soundSpeed;
    // Pression dynamique: q = 1/2 * rho * V²
    const dynamicPressure = 0.5 * airDensity * speed_ms * speed_ms; 
    // Force de traînée: Fd = q * A * Cd
    const dragForce = dynamicPressure * REFERENCE_DRAG_AREA * DRAG_COEFFICIENT; 

    // Accélération Longitudinale (basée sur l'accélération IMU x, ou une estimation EKF)
    // En mode réel, on utiliserait l'accélération EKF corrigée.
    const accelLong = accel.x; 

    // Placeholder pour la Force de Coriolis (simple)
    const coriolisForce = 2 * MASS_KG * speed_ms * OMEGA_EARTH * Math.cos(lat * D2R); 

    return { mach, dynamicPressure, dragForce, accelLong, coriolisForce };
}

// ... (Autres fonctions : dist2D, getKalmanR, updateCelestialBody, etc.)

// =================================================================
// BLOC 3/4 : Gestion GPS, IMU, Carte et Météo (MISE À JOUR)
// =================================================================

// ... (Fonctions d'initialisation, startGPS, stopGPS, etc.)

/**
 * BOUCLE RAPIDE (Loop IMU) - Prédiction UKF et Mise à Jour DOM Dynamique
 */
function mainLoop() {
    const now = performance.now();
    const dt = (now - lastTimestamp) / 1000;
    lastTimestamp = now;

    if (emergencyStopActive || !ukf || dt < 0.01) return;
    
    // 1. PRÉDICTION UKF
    ukf.predict({ accel: [accel.x, accel.y, accel.z] }, dt);
    const state = ukf.getState();
    kSpd = state.speed;
    kUncert = state.kUncert;
    kAltUncert = state.kAltUncert;
    kAlt = state.alt;

    // 2. CALCULS DYNAMIQUES (NOUVEAU)
    const currentDensity = isHyperloopActive ? RHO_VACUUM : currentAirDensity;
    const { mach, dynamicPressure, dragForce, accelLong, coriolisForce } = calculateDynamicMetrics(
        kSpd, 
        currentDensity, 
        currentSpeedOfSound
    );

    // 3. MISE À JOUR DU DOM (Rapide)
    $('speed-stable').textContent = dataOrDefault(kSpd * KMH_MS, 2);
    $('speed-stable-ms').textContent = dataOrDefault(kSpd, 3, ' m/s');
    $('lat-display').textContent = dataOrDefault(state.lat, 6, ' °');
    $('lon-display').textContent = dataOrDefault(state.lon, 6, ' °');
    
    // Nouveaux champs EKF et Dynamique
    $('alt-display').textContent = dataOrDefault(kAlt, 2, ' m');
    $('kalman-uncert').textContent = dataOrDefault(kUncert, 3, ' m²/s²');
    $('alt-uncertainty').textContent = dataOrDefault(kAltUncert, 3, ' m');
    $('gps-status-dr').textContent = isolationStatus; // Statut d'isolation
    
    $('mach-number').textContent = dataOrDefault(mach, 4);
    $('dynamic-pressure').textContent = dataOrDefault(dynamicPressure, 2, ' Pa');
    $('drag-force').textContent = dataOrDefault(dragForce, 2, ' N');
    $('accel-long').textContent = dataOrDefault(accelLong, 3, ' m/s²');
    $('coriolis-force').textContent = dataOrDefault(coriolisForce, 2, ' N');
    $('speed-sound').textContent = dataOrDefault(currentSpeedOfSound, 2, ' m/s');
    // ... Mise à jour des autres champs de vitesse/distance
}


// ... (Fonction gpsUpdateCallback : Appelle ukf.update(pos.coords))

// ... (Fonction fetchWeather : Met à jour currentAirDensity et currentSpeedOfSound)
// Assurez-vous d'appeler getAirDensity(P_hPa, T_K) qui tient compte de isHyperloopActive

// =================================================================
// BLOC 4/4 : Event Listeners et Démarrage (MISE À JOUR)
// =================================================================
document.addEventListener('DOMContentLoaded', () => {

    // ... (Autres Event Listeners : init, reset, toggle-mode, etc.)

    // NOUVEAU : CONTRÔLE HYPERLOOP
    const hyperloopBtn = $('hyperloop-toggle-btn');
    if (hyperloopBtn) {
        hyperloopBtn.addEventListener('click', () => {
            isHyperloopActive = !isHyperloopActive;
            if (isHyperloopActive) {
                hyperloopBtn.textContent = 'HYPERLOOP ACTIF (VIDE)';
                hyperloopBtn.classList.add('active');
                currentAirDensity = RHO_VACUUM; // Force la densité minimale
            } else {
                hyperloopBtn.textContent = 'HYPERLOOP INACTIF';
                hyperloopBtn.classList.remove('active');
                // Déclencher une mise à jour météo pour retrouver une densité réelle
                // if (lat && lon) fetchWeather(lat, lon); // Décommenter si fetchWeather est implémenté
                currentAirDensity = RHO_SEA_LEVEL; // Placeholder si fetchWeather n'est pas là
            }
            if ($('weather-status')) $('weather-status').textContent = isHyperloopActive ? "FORCÉ (VIDE)" : "EN LIGNE";
        });
    }

    // ... (Démarrage du système, boucles d'intervalles)
});
