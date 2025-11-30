// =================================================================
// BLOC 1/4 : CONSTANTES, VARIABLES D'ÉTAT & UTILITAIRES ESSENTIELS
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES (Pour accéder rapidement aux éléments DOM) ---
const $ = id => document.getElementById(id);
const KMH_MS = 3.6; 
const C_L = 299792458; // Vitesse de la lumière (m/s)

// Helper pour afficher N/A ou la valeur formatée
const dataOrDefault = (val, decimals, suffix = '', na = 'N/A') => {
    if (val === undefined || val === null || isNaN(val)) {
        return na;
    }
    return val.toFixed(decimals) + suffix;
};
const dataOrDefaultExp = (val, decimals, suffix = '', na = 'N/A') => {
    if (val === undefined || val === null || isNaN(val)) {
        return na;
    }
    return val.toExponential(decimals) + suffix;
};

// --- VARIABLES D'ÉTAT CRITIQUES ---
let wID = null;             // ID de watchPosition (null = GPS inactif). CLÉ du bouton MARCHE/PAUSE
let domFastID = null;       // ID pour la boucle d'affichage rapide (requestAnimationFrame)
let sessionStartTime = Date.now(); // Début de la session
let emergencyStopActive = false;
let currentMass = 70.0;
let currentCelestialBody = 'Terre';

// Données de session
let distM = 0.0;            // Distance totale parcourue (m)
let maxSpd = 0.0;           // Vitesse max (m/s)
let timeMoving = 0.0;       // Temps de mouvement (s)

// Données EKF/UKF (minimales)
let kSpd = 0.0;             // Vitesse estimée par UKF (m/s)
let kAlt = 0.0;             // Altitude estimée par UKF (m)
let currentSpeedOfSound = 343.20; // m/s
let currentAirDensity = 1.225; // kg/m³

// --- CONFIGURATIONS & OPTIONS ---
const GPS_OPTS = {
    'HIGH_FREQ': { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    'LOW_FREQ': { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};

// --- CLASSES ET FONCTIONS PLACEHOLDERS (Obligatoires pour éviter des erreurs ReferenceError) ---
// Remplacez ces corps de fonctions par vos implémentations complètes.
class ProfessionalUKF {
    constructor(lat = 0, lon = 0, rho = 1.225) { /* UKF Init */ this.speed = 0; }
    update(gpsData, imuData) { /* Logique de fusion */ this.speed = gpsData.speed || 0; }
}
let ukf = new ProfessionalUKF();

function updateCelestialBody(body, alt, radius, angular) { return { G_ACC_NEW: 9.8067 }; }
function getSpeedOfSound(tempK) { return 331.3 + 0.606 * (tempK - 273.15); }

// GPS Success/Error minimal
function gpsSuccess(position) { 
    console.log("Nouvelle position GPS reçue."); 
    // Ici, vous mettriez à jour l'UKF: ukf.update(position, latestIMUData);
    kSpd = position.coords.speed || 0; // Mise à jour simplifiée pour débloquer la vitesse
}
function gpsError(error) { console.error("Erreur GPS:", error.code, error.message); }

function syncH() { /* Tente la synchro NTP */ }
function startSlowLoop() { /* Boucle pour Météo/Astro */ }
// =================================================================
// BLOC 2/4 : LOGIQUE DE CONTRÔLE GPS & IMU
// Contient la correction critique du bouton MARCHE/PAUSE.
// =================================================================

/** Démarre les capteurs IMU (Accéléromètre et Gyroscope) */
function startIMUListeners() {
    // 🚩 IMPORTANT : Votre code IMU doit être ici (ex: new Accelerometer().start())
    if ($('imu-status')) $('imu-status').textContent = "Actif (API Sensor 50Hz)";
}

/** Arrête les capteurs IMU */
function stopIMUListeners() {
    // 🚩 IMPORTANT : Votre code d'arrêt IMU doit être ici
    if ($('imu-status')) $('imu-status').textContent = "Inactif";
}

/** Démarre l'acquisition GPS et les capteurs IMU. */
function startGPS(mode = 'HIGH_FREQ') {
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

/** Arrête l'acquisition GPS et les capteurs IMU. */
function stopGPS(isManualReset = false) {
    if (wID !== null) { 
        navigator.geolocation.clearWatch(wID); 
        wID = null; 
    }
    
    // 1. Arrêter les capteurs IMU
    stopIMUListeners();
    
    // 2. Arrêter la boucle d'affichage rapide (optionnel si vous voulez laisser l'horloge tourner)
    if (domFastID) { cancelAnimationFrame(domFastID); domFastID = null; }

    // 3. Mettre à jour l'affichage du bouton et des statuts
    if ($('start-btn')) $('start-btn').innerHTML = '▶️ MARCHE GPS';
    if ($('gps-status')) $('gps-status').textContent = isManualReset ? "INACTIF (Manuel)" : "INACTIF";
    if ($('imu-status')) $('imu-status').textContent = "Inactif";
}
// =================================================================
// BLOC 3/4 : BOUCLE D'AFFICHAGE RAPIDE (DOM)
// =================================================================

/** Met à jour les éléments du DOM qui nécessitent une haute fréquence (Vitesse, Relativité, Dynamique). */
function startFastLoop() {
    const loop = (timestamp) => {
        
        const currentSpeedKmH = kSpd * KMH_MS; // Vitesse UKF en km/h
        
        // --- VITESSE, DISTANCE & RELATIVITÉ ---
        if ($('speed-instant')) $('speed-instant').textContent = dataOrDefault(currentSpeedKmH, 2, ' km/h', '--.- km/h');
        if ($('speed-stable-ms')) $('speed-stable-ms').textContent = dataOrDefault(kSpd, 2, ' m/s', '-- m/s');
        
        maxSpd = Math.max(maxSpd, kSpd);
        if ($('speed-max')) $('speed-max').textContent = dataOrDefault(maxSpd * KMH_MS, 5, ' km/h');

        // Facteur de Lorentz (γ)
        const lorentzFactor = 1 / Math.sqrt(1 - Math.pow(kSpd / C_L, 2));
        if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(lorentzFactor, 4);
        if ($('speed-of-light-perc')) $('speed-of-light-perc').textContent = dataOrDefaultExp((kSpd / C_L) * 100, 2, ' %');
        
        // Énergie Cinétique (J)
        const kineticEnergy = 0.5 * currentMass * Math.pow(kSpd, 2);
        if ($('kinetic-energy')) $('kinetic-energy').textContent = dataOrDefault(kineticEnergy, 2, ' J');

        if ($('time-elapsed-session')) $('time-elapsed-session').textContent = dataOrDefault((Date.now() - sessionStartTime) / 1000, 2, ' s');
        if ($('distance-total-km')) $('distance-total-km').textContent = `${dataOrDefault(distM / 1000, 3, ' km')} | ${dataOrDefault(distM, 2, ' m')}`;

        // --- MÉCANIQUE DES FLUIDES ---
        const dynamicPressure = 0.5 * currentAirDensity * Math.pow(kSpd, 2);
        if ($('dynamic-pressure')) $('dynamic-pressure').textContent = dataOrDefault(dynamicPressure, 2, ' Pa');

        // Demande la prochaine frame
        domFastID = requestAnimationFrame(loop);
    };
    
    // Lancement initial de la boucle
    if (domFastID === null) domFastID = requestAnimationFrame(loop);
}
// =================================================================
// BLOC 4/4 : INITIALISATION DES CONTRÔLES SYSTÈME & DÉMARRAGE (INIT)
// =================================================================

/** Configure tous les écouteurs d'événements pour les boutons et les inputs. */
function initControls() {
    
    // 🚩 CORRECTION CRITIQUE : GESTION DU BOUTON MARCHE/PAUSE (Toggle)
    const startBtn = $('start-btn');
    if (startBtn) {
        startBtn.addEventListener('click', () => {
            // Si wID n'est pas null, le GPS est actif -> on le met en pause (stopGPS).
            if (wID !== null) {
                stopGPS(true); 
            } else {
                // Sinon, le GPS est inactif -> on le démarre (startGPS).
                startGPS('HIGH_FREQ'); 
            }
        });
    }

    // Contrôle : TOUT RÉINITIALISER
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { 
        if (confirm("Êtes-vous sûr de vouloir TOUT réinitialiser ?")) {
            stopGPS(true); // Arrête le GPS et les capteurs
            window.location.reload(); // Rechargement total pour un reset complet
        }
    });

    // Contrôle : Réinitialiser Distance
    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => {
        distM = 0; timeMoving = 0; 
    });
    
    // Contrôle : Réinitialiser Vitesse Max
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', () => {
        maxSpd = 0.0;
    });
    
    // Contrôle : Masse de l'objet (kg)
    if ($('mass-input')) $('mass-input').addEventListener('input', (e) => {
        currentMass = parseFloat(e.target.value) || 70.0;
        if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
    });
    
    // Contrôle : Corps Céleste (Mise à jour de la Gravité)
    if ($('celestial-body-select')) $('celestial-body-select').addEventListener('change', (e) => {
        currentCelestialBody = e.target.value;
        const { G_ACC_NEW } = updateCelestialBody(currentCelestialBody, kAlt, 100, 0);
        if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
    });
    
    // ... Ajoutez ici les autres écouteurs (Mode Nether, Facteur d'environnement, etc.) ...
}

/** Fonction d'initialisation principale */
function init() {
    // 1. Initialiser la synchro temps et les boucles lentes (Météo/Astro)
    syncH(); 
    startSlowLoop(); 
    
    // 2. Démarrage de la boucle d'affichage rapide (pour les valeurs '0.00' par défaut)
    startFastLoop(); 
    
    // 3. Initialisation des gestionnaires d'événements
    initControls(); 
}

// Lancement du système au chargement complet de la page
document.addEventListener('DOMContentLoaded', init);
