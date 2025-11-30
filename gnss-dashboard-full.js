// =================================================================
// BLOC 1/4 : CONSTANTES, VARIABLES D'ÉTAT & UTILITAIRES
// Assure que les dépendances globales sont définies avant utilisation.
// =================================================================

// --- FONCTION UTILITAIRE (Doit toujours être définie en premier) ---
const $ = id => document.getElementById(id);

// --- VARIABLES D'ÉTAT CRITIQUES ---
let wID = null;             // Identifiant de la session watchPosition (null = GPS inactif)
let domFastID = null;       // Identifiant pour la boucle d'affichage rapide (requestAnimationFrame)
let emergencyStopActive = false; // Statut de l'arrêt d'urgence

// Données de session
let distM = 0;              // Distance totale parcourue (m)
let maxSpd = 0.0;           // Vitesse max (m/s)
let timeMoving = 0;         // Temps de mouvement (s)

// Variables EKF/Physique (minimales pour l'exécution)
let kAlt = 0;               // Altitude estimée par EKF
let currentMass = 70;       // Masse de l'objet (kg)
let currentCelestialBody = 'Terre';
let rotationRadius = 100;
let angularVelocity = 0.0;
let netherMode = false;

// --- CONSTANTES & OPTIONS ---
const GPS_OPTS = {
    'HIGH_FREQ': { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    'LOW_FREQ': { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};

// --- CLASSES ET FONCTIONS PLACEHOLDERS (Pour garantir l'exécution) ---
// Remplacez ces classes/fonctions par vos implémentations complètes.
class ProfessionalUKF {
    constructor(lat = 0, lon = 0, rho = 1.225) {
        console.log("UKF Initialisé.");
    }
}
let ukf = new ProfessionalUKF();
function updateCelestialBody(body, alt, radius, angular) { return { G_ACC_NEW: 9.8067 }; }
function gpsSuccess(position) { console.log("Nouvelle position GPS reçue."); }
function gpsError(error) { console.error("Erreur GPS:", error.code); }
function syncH() { /* Logique de Synchro NTP */ }
function startSlowLoop() { /* Logique Météo/Astro */ }
// =================================================================
// BLOC 2/4 : LOGIQUE DE CONTRÔLE GPS & IMU
// Contient la correction critique du bouton MARCHE/PAUSE.
// =================================================================

function startIMUListeners() {
    // 🚩 CORRECTION : Démarrez les capteurs ici (ex: new Accelerometer().start())
    // Si l'IMU est activé, mettez à jour l'affichage.
    if ($('imu-status')) $('imu-status').textContent = "Actif (API Sensor 50Hz)";
}

function stopIMUListeners() {
    // 🚩 CORRECTION : Arrêtez les capteurs ici (ex: accelerometer.stop())
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
    
    // 2. Arrêter la boucle d'affichage rapide
    if (domFastID) { cancelAnimationFrame(domFastID); domFastID = null; }

    // 3. Mettre à jour l'affichage du bouton et des statuts
    if ($('start-btn')) $('start-btn').innerHTML = '▶️ MARCHE GPS';
    if ($('gps-status')) $('gps-status').textContent = isManualReset ? "INACTIF (Manuel)" : "INACTIF";
    if ($('imu-status')) $('imu-status').textContent = "Inactif";
                                              }
// =================================================================
// BLOC 3/4 : BOUCLES D'AFFICHAGE & MISE À JOUR DOM
// La boucle rapide met à jour la vitesse et les données dynamiques.
// =================================================================

/** Boucle d'affichage rapide basée sur requestAnimationFrame */
function startFastLoop() {
    const loop = () => {
        // --- LOGIQUE DE MISE À JOUR DU DOM (VITESSE, PHYSIQUE, EKF) ---
        
        // Exemple : Affichage de la vitesse et de la distance (si UKF a été mis à jour)
        const currentSpeedKmH = (ukf.speed || 0) * 3.6; // Utiliser la vitesse EKF/UKF
        if ($('speed-instant')) $('speed-instant').textContent = `${currentSpeedKmH.toFixed(2)} km/h`;
        if ($('distance-total-km')) $('distance-total-km').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
        if ($('temps-ecoule-session')) $('temps-ecoule-session').textContent = `${((Date.now() - sessionStartTime) / 1000).toFixed(2)} s`;

        // Exemple : Mise à jour de l'état du capteur IMU (le statut 'Actif' vient de startIMUListeners)
        if ($('imu-status')) {
             if (wID === null) $('imu-status').textContent = "Inactif"; // Réinitialisation au cas où
        }
        
        // Mettez à jour ici tous les champs dynamiques (Mach, Lorentz, Énergie, etc.)
        
        // Demande la prochaine frame (haute fréquence)
        if (wID !== null || domFastID !== null) { // Continuer si GPS actif ou si on veut juste la clock
            domFastID = requestAnimationFrame(loop);
        } else {
             domFastID = null; // Stoppe si GPS arrêté
        }
    };
    
    // Lancement initial de la boucle
    if (domFastID === null) domFastID = requestAnimationFrame(loop);
}
// =================================================================
// BLOC 4/4 : INITIALISATION DES CONTRÔLES SYSTÈME (INIT)
// Point d'entrée de l'application.
// =================================================================

/**
 * Configure tous les écouteurs d'événements pour les boutons et les inputs.
 */
function initControls() {
    // 🚩 CORRECTION CRITIQUE : Logique de bascule (toggle) pour le bouton MARCHE/PAUSE GPS
    const startBtn = $('start-btn');
    if (startBtn) {
        startBtn.addEventListener('click', () => {
            // Si wID existe (non-null), le GPS est ACTIF -> on le met en pause.
            if (wID !== null) {
                stopGPS(true);
            } else {
                // Sinon, le GPS est inactif -> on le démarre.
                startGPS('HIGH_FREQ'); 
            }
        });
    }

    // Contrôle : Arrêt d'Urgence
    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', () => {
        emergencyStopActive = !emergencyStopActive;
        // La mise à jour de l'affichage de l'arrêt d'urgence doit se faire ici ou dans une fonction dédiée.
        if (emergencyStopActive) stopGPS(true);
    });

    // Contrôle : Réinitialiser Distance
    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => {
        if (!emergencyStopActive) { distM = 0; timeMoving = 0; }
    });
    
    // Contrôle : Réinitialiser Vitesse Max
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', () => {
        if (!emergencyStopActive) { maxSpd = 0.0; }
    });
    
    // Contrôle : TOUT RÉINITIALISER
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { 
        if (confirm("Êtes-vous sûr de vouloir TOUT réinitialiser ?")) {
            stopGPS(true); 
            localStorage.clear(); // Optionnel: effacer les données persistantes
            window.location.reload(); 
        }
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
        if ($('mode-nether')) $('mode-nether').textContent = netherMode ? 'ACTIVÉ (1:8)' : 'DÉSACTIVÉ (1:1)';
    });
    
    // ... Ajoutez ici les autres écouteurs d'événements ...
}

/** Fonction d'initialisation principale */
function init() {
    // 1. Initialiser la synchro temps et les boucles lentes (Météo/Astro)
    syncH(); 
    startSlowLoop(); 
    
    // 2. Démarrer la boucle d'affichage rapide (pour les valeurs statiques/par défaut)
    startFastLoop(); 
    
    // 3. Initialisation des gestionnaires d'événements
    initControls(); 
    
    // 4. Initialisation de la carte (si elle n'est pas déjà gérée par un autre bloc)
    // initMap(); 
}

// Lancement du système au chargement complet de la page
document.addEventListener('DOMContentLoaded', init);
