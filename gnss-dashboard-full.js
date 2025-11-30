// =================================================================
// BLOC 1/4 : CONSTANTES, VARIABLES D'ÉTAT & PLACEHOLDERS SYSTÈME
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);
const KMH_MS = 3.6;             // Conversion m/s -> km/h
const C_L = 299792458;          // Vitesse de la lumière (m/s)
const G_ACCEL = 9.80665;        // Gravité standard (m/s²)
const RHO_SEA_LEVEL = 1.225;    // Densité de l'air ISA (kg/m³)
const TEMP_SEA_LEVEL_K = 288.15; // 15°C en Kelvin

// Helper pour afficher N/A ou la valeur formatée
const dataOrDefault = (val, decimals, suffix = '', na = 'N/A') => {
    if (val === undefined || val === null || isNaN(val)) return na;
    return val.toFixed(decimals) + suffix;
};
const dataOrDefaultExp = (val, decimals, suffix = '', na = 'N/A') => {
    if (val === undefined || val === null || isNaN(val)) return na;
    return val.toExponential(decimals) + suffix;
};

// --- CONFIGURATIONS GPS ---
const GPS_OPTS = {
    'HIGH_FREQ': { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    'LOW_FREQ': { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};

// --- VARIABLES D'ÉTAT CRITIQUES (UKF et Session) ---
let wID = null;             // ID de watchPosition (CLÉ du toggle MARCHE/PAUSE)
let domFastID = null;       // ID pour la boucle d'affichage rapide
let sessionStartTime = Date.now();
let emergencyStopActive = false;
let currentMass = 70.0;
let currentCelestialBody = 'Terre';
let rotationRadius = 100;
let angularVelocity = 0.0;
let netherMode = false;
let currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound = 343.20; // Vitesse du son de référence

// Données de session
let distM = 0.0;            // Distance totale parcourue (m)
let maxSpd = 0.0;           // Vitesse max (m/s)
let timeMoving = 0.0;       // Temps de mouvement (s)

// Données EKF/UKF (estimées)
let kSpd = 0.0;             // Vitesse estimée par UKF (m/s)
let kAlt = 0.0;             // Altitude estimée par UKF (m)
let lastT_K = TEMP_SEA_LEVEL_K; // Température pour calculs métrologiques

// --- PLACEHOLDERS ESSENTIELS (Simulations / Modules) ---

// UKF Placeholder
class ProfessionalUKF {
    constructor(lat = 0, lon = 0, rho = RHO_SEA_LEVEL) {
        // Initialisation fictive
        this.speed = 0.0;
        this.altitude = 0.0;
    }
    update(position, imuData) {
        // Logique de fusion (fictive)
        this.speed = position.coords ? position.coords.speed || 0 : 0;
        this.altitude = position.coords ? position.coords.altitude || 0 : 0;
    }
}
let ukf = new ProfessionalUKF(43.2964, 5.3697, RHO_SEA_LEVEL); // Utilise les coordonnées par défaut de Marseille

// Fonctions Placeholders
function gpsSuccess(position) { 
    // Mettez à jour la vitesse estimée par le filtre Kalman/UKF
    ukf.update(position, {}); 
    kSpd = ukf.speed; 
    kAlt = ukf.altitude;
    // Mise à jour de la distance, max speed, etc.
}
function gpsError(error) { console.error("Erreur GPS:", error.code, error.message); }
function syncH() { /* Logique de Synchro NTP */ console.log("Synchro NTP tentée."); }
function startSlowLoop() { /* Boucle pour Météo/Astro/Logging */ console.log("Boucle lente démarrée."); }
function startIMUListeners() { 
    if ($('imu-status')) $('imu-status').textContent = "Actif (API Sensor 50Hz)";
}
function stopIMUListeners() {
    if ($('imu-status')) $('imu-status').textContent = "Inactif";
}
function updateCelestialBody(body, alt, radius, angular) { return { G_ACC_NEW: G_ACCEL }; }
function getSpeedOfSound(tempK) { return 331.3 + 0.606 * (tempK - 273.15); }
// =================================================================
// BLOC 2/4 : LOGIQUE DE CONTRÔLE GPS & IMU (startGPS/stopGPS)
// =CHEK-POINT-CORRECTION-CRITIQUE-GPS-TOGGLE-
// =================================================================

/** Démarre l'acquisition GPS, les capteurs IMU et la boucle d'affichage rapide. */
function startGPS(mode = 'HIGH_FREQ') {
    // Si déjà actif ou Arrêt d'Urgence actif, sortir
    if (wID !== null || emergencyStopActive) return; 
    
    // 1. Démarrer la géolocalisation
    wID = navigator.geolocation.watchPosition(gpsSuccess, gpsError, GPS_OPTS[mode]);
    
    // 2. Démarrer les capteurs IMU
    startIMUListeners(); 

    // 3. Mettre à jour l'affichage du bouton
    if ($('start-btn')) $('start-btn').innerHTML = '⏸️ PAUSE GPS'; 
    if ($('gps-status')) $('gps-status').textContent = `Actif (Mode ${mode})`;
    
    // 4. Assurer que la boucle d'affichage rapide est lancée
    if (domFastID === null) startFastLoop();
}

/** Arrête l'acquisition GPS, les capteurs IMU et la boucle d'affichage rapide. */
function stopGPS(isManualReset = false) {
    if (wID !== null) { 
        navigator.geolocation.clearWatch(wID); 
        wID = null; // Désactive l'état GPS
    }
    
    // 1. Arrêter les capteurs IMU
    stopIMUListeners();
    
    // 2. Arrêter la boucle d'affichage rapide
    if (domFastID) { cancelAnimationFrame(domFastID); domFastID = null; }

    // 3. Mettre à jour l'affichage du bouton et des statuts
    if ($('start-btn')) $('start-btn').innerHTML = '▶️ MARCHE GPS';
    if ($('gps-status')) $('gps-status').textContent = isManualReset ? "INACTIF (Manuel)" : "INACTIF";
    if ($('imu-status')) $('imu-status').textContent = "Inactif";
}
// =================================================================
// BLOC 3/4 : BOUCLE D'AFFICHAGE RAPIDE (startFastLoop)
// Mise à jour des valeurs dynamiques à haute fréquence.
// =================================================================

/** Boucle d'affichage rapide basée sur requestAnimationFrame */
function startFastLoop() {
    const loop = () => {
        
        // --- CALCULS DE BASE ---
        const currentSpeedKmH = kSpd * KMH_MS; // Vitesse UKF en km/h
        maxSpd = Math.max(maxSpd, kSpd);
        
        // Facteur de Lorentz (γ)
        const lorentzFactor = 1 / Math.sqrt(1 - Math.pow(kSpd / C_L, 2));
        
        // Énergie Cinétique (J)
        const kineticEnergy = 0.5 * currentMass * Math.pow(kSpd, 2);
        
        // Pression Dynamique
        const dynamicPressure = 0.5 * currentAirDensity * Math.pow(kSpd, 2);

        // --- MISE À JOUR DU DOM (VITESSE, DISTANCE & RELATIVITÉ) ---
        
        // Vitesse
        if ($('speed-instant')) $('speed-instant').textContent = dataOrDefault(currentSpeedKmH, 2, ' km/h', '--.- km/h');
        if ($('speed-stable-ms')) $('speed-stable-ms').textContent = dataOrDefault(kSpd, 2, ' m/s', '-- m/s');
        if ($('speed-max')) $('speed-max').textContent = dataOrDefault(maxSpd * KMH_MS, 5, ' km/h');
        
        // Distance/Temps
        if ($('temps-ecoule-session')) $('temps-ecoule-session').textContent = dataOrDefault((Date.now() - sessionStartTime) / 1000, 2, ' s');
        if ($('time-moving')) $('time-moving').textContent = dataOrDefault(timeMoving, 2, ' s', '0.00 s');
        if ($('distance-total-km')) $('distance-total-km').textContent = `${dataOrDefault(distM / 1000, 3, ' km')} | ${dataOrDefault(distM, 2, ' m')}`;

        // Relativité
        if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(lorentzFactor, 4);
        if ($('speed-of-light-perc')) $('speed-of-light-perc').textContent = dataOrDefaultExp((kSpd / C_L) * 100, 2, ' %');
        if ($('energy-cinetique')) $('energy-cinetique').textContent = dataOrDefaultExp(kineticEnergy, 2, ' J', 'N/A');
        
        // Dynamique
        if ($('dynamic-pressure')) $('dynamic-pressure').textContent = dataOrDefault(dynamicPressure, 2, ' Pa');

        // Demande la prochaine frame
        domFastID = requestAnimationFrame(loop);
    };
    
    // Lancement initial de la boucle
    if (domFastID === null) domFastID = requestAnimationFrame(loop);
}
// =================================================================
// BLOC 4/4 : INITIALISATION DES CONTRÔLES SYSTÈME (INIT)
// =================================================================

/** Configure tous les écouteurs d'événements pour les boutons et les inputs. */
function initControls() {
    
    // 🚩 CORRECTION CRITIQUE : GESTION DU BOUTON MARCHE/PAUSE (Toggle)
    const startBtn = $('start-btn');
    if (startBtn) {
        startBtn.addEventListener('click', () => {
            // Si wID existe (non-null), le GPS est actif -> on le met en pause (stopGPS).
            if (wID !== null) {
                stopGPS(true); 
            } else {
                // Sinon, le GPS est inactif -> on le démarre (startGPS).
                startGPS('HIGH_FREQ'); 
            }
        });
    }
    
    // Contrôle : Arrêt d'Urgence
    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', () => {
        emergencyStopActive = !emergencyStopActive;
        // Met à jour l'affichage d'urgence
        if ($('emergency-status')) $('emergency-status').textContent = emergencyStopActive ? 'ACTIF 🔴' : 'INACTIF 🟢';
        if (emergencyStopActive) stopGPS(true); // Arrêter immédiatement en cas d'urgence
    });

    // Contrôle : TOUT RÉINITIALISER
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { 
        if (confirm("Êtes-vous sûr de vouloir TOUT réinitialiser ?")) {
            stopGPS(true); 
            // Réinitialisation des variables de session et rechargement
            distM = 0; maxSpd = 0; timeMoving = 0; 
            window.location.reload(); 
        }
    });

    // Contrôle : Réinitialiser Distance
    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => {
        if (!emergencyStopActive) { distM = 0; timeMoving = 0; }
    });
    
    // Contrôle : Réinitialiser Vitesse Max
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', () => {
        if (!emergencyStopActive) { maxSpd = 0.0; }
    });
    
    // Contrôle : Masse de l'objet (kg)
    if ($('mass-input')) $('mass-input').addEventListener('input', (e) => {
        currentMass = parseFloat(e.target.value) || 70.0;
        if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
    });
    
    // Contrôle : Corps Céleste
    if ($('celestial-body-select')) $('celestial-body-select').addEventListener('change', (e) => {
        currentCelestialBody = e.target.value;
        const { G_ACC_NEW } = updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity);
        if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
    });
    
    // Contrôle : Mode Nether
    if ($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', () => {
        netherMode = !netherMode;
        if ($('mode-nether')) $('mode-nether').textContent = netherMode ? 'ACTIF (1:8)' : 'DÉSACTIVÉ (1:1)';
    });

    // ... Ajoutez ici les autres écouteurs d'événements ...
}

/** Fonction d'initialisation principale */
function init() {
    // 1. Démarrage des boucles lentes (Météo/Astro/NTP)
    syncH(); 
    startSlowLoop(); 
    
    // 2. Démarrage de la boucle d'affichage rapide (pour les valeurs '0.00' par défaut)
    startFastLoop(); 
    
    // 3. Initialisation des gestionnaires d'événements
    initControls(); 
}

// Lancement du système au chargement complet de la page
document.addEventListener('DOMContentLoaded', init);
