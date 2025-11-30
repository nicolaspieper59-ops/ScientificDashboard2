// =================================================================
// BLOC 1/5 : CONSTANTES, UTILITAIRES ET ÉTAT GLOBAL
// Base Physique, Constantes et Variables de Session (UKF/GNSS)
// =================================================================

const $ = id => document.getElementById(id);

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const KMH_MS = 3.6;         
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const G_ACCEL = 9.80665;    // Gravité standard (m/s²)
const RHO_SEA_LEVEL = 1.225; // Densité de l'air ISA (kg/m³)
const TEMP_SEA_LEVEL_K = 288.15; // 15°C en Kelvin

// API Endpoints (si nécessaire)
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

// --- CONFIGURATIONS GPS ---
const GPS_OPTS = {
    'HIGH_FREQ': { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    'LOW_FREQ': { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};

// --- VARIABLES D'ÉTAT CRITIQUES ---
let wID = null;             // ID de watchPosition (CLÉ du toggle MARCHE/PAUSE)
let domFastID = null;       // ID pour la boucle d'affichage rapide (requestAnimationFrame)
let sessionStartTime = Date.now();
let emergencyStopActive = false;
let currentMass = 70.0;
let currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound = 343.20; 
let lastT_K = TEMP_SEA_LEVEL_K;
let lastP_hPa = 1013.25;

// Données de session
let distM = 0.0;            // Distance totale parcourue (m)
let maxSpd = 0.0;           // Vitesse max (m/s)
let timeMoving = 0.0;       // Temps de mouvement (s)
let maxGForce = 0.0;

// Données EKF/UKF (estimées)
let kSpd = 0.0;             // Vitesse estimée par UKF (m/s)
let kAlt = 0.0;             // Altitude estimée par UKF (m)

// --- FONCTIONS UTILITAIRES ---
const dataOrDefault = (val, decimals, suffix = '', na = 'N/A') => {
    if (val === undefined || val === null || isNaN(val)) return na;
    return val.toFixed(decimals) + suffix;
};
const dataOrDefaultExp = (val, decimals, suffix = '', na = 'N/A') => {
    if (val === undefined || val === null || isNaN(val)) return na;
    return val.toExponential(decimals) + suffix;
};
// =================================================================
// BLOC 2/5 : MODÈLES PHYSIQUES & FONCTIONS SYSTÈME (UKF, ASTRO, MÉTÉO)
// =================================================================

// --- CLASSE UKF (UKF 21 États - Logique Fictive pour exécution) ---
class ProfessionalUKF {
    constructor(lat = 0, lon = 0, rho = RHO_SEA_LEVEL) {
        // Initialisation fictive (dans la version complète, ceci est math.js)
        this.speed = 0.0;
        this.altitude = 0.0;
        this.uncertainty = 100.0;
        this.x = [lat, lon, 0, 0, 0, 0]; // État minimal: Lat, Lon, V_N, V_E, Alt, V_Alt
    }
    // La fonction update doit exister pour être appelée par gpsSuccess
    update(position, imuData) {
        // Logique de fusion (fictive, utilise données brutes du GPS)
        this.speed = position.coords ? position.coords.speed || 0 : 0;
        this.altitude = position.coords ? position.coords.altitude || 0 : 0;
        this.uncertainty = position.coords ? position.coords.accuracy || 10 : 10;
        // La mise à jour de l'état (this.x) se ferait ici dans la vraie version
    }
}
let ukf = new ProfessionalUKF(43.2964, 5.3697, RHO_SEA_LEVEL);

// --- FONCTIONS ASTRO / MÉTÉO / GÉO (Élémentaires ou Fictives) ---

/** Synchronisation NTP (fictive / simplifiée) */
function syncH() { 
    console.log("Synchro NTP tentée.");
    // Logique réelle pour mettre à jour 'systemClockOffsetMS'
}

/** Démarrage de la boucle lente (Météo, Astro, Logging) */
function startSlowLoop() { 
    console.log("Boucle lente démarrée.");
    // Logique réelle : Mise à jour de l'heure et des données météo toutes les 30s
    // Exemple d'appel toutes les 30s:
    // setInterval(() => { fetchWeather(currentPosition.lat, currentPosition.lon); /* ... */ }, 30000);
}

/** Fonction pour calculer la vitesse du son (m/s) à partir de la température (K) */
function getSpeedOfSound(tempK) { 
    return 331.3 + 0.606 * (tempK - 273.15); 
}

/** Fonction de mise à jour du corps céleste (simulée) */
function updateCelestialBody(body, alt, radius, angular) { 
    console.log(`Changement de corps céleste vers ${body}`);
    // Logique réelle pour recalculer G_ACCEL
    return { G_ACC_NEW: G_ACCEL }; 
}

/** Fonction de récupération de la météo (fictive pour éviter erreur de référence) */
async function fetchWeather(lat, lon) {
    console.log(`Tentative de fetch météo à ${lat}, ${lon}`);
    // Logique réelle pour fetch depuis PROXY_WEATHER_ENDPOINT
    return { pressure_hPa: lastP_hPa, tempK: lastT_K, tempC: lastT_K - 273.15, air_density: currentAirDensity };
}
// =================================================================
// BLOC 3/5 : LOGIQUE DE CONTRÔLE GPS & IMU (startGPS/stopGPS)
// =================================================================

// --- GESTION DES CAPTEURS ---

function gpsSuccess(position) { 
    // Mise à jour de l'UKF avec la nouvelle position
    ukf.update(position, {}); 
    kSpd = ukf.speed; 
    kAlt = ukf.altitude;

    // Logique de calcul de distance et max speed...
}

function gpsError(error) { 
    console.error("Erreur GPS:", error.code, error.message);
    // Gérer l'erreur (ex: passage en mode estimation seule)
}

function startIMUListeners() { 
    // Logique réelle pour démarrer Accelerometer, Gyroscope, etc. (API Sensor)
    if ($('imu-status')) $('imu-status').textContent = "Actif (API Sensor 50Hz)";
}

function stopIMUListeners() {
    // Logique réelle pour arrêter les capteurs
    if ($('imu-status')) $('imu-status').textContent = "Inactif";
}


// --- GESTION DU GPS (START/STOP) ---

/** Démarre l'acquisition GPS, les capteurs IMU et la boucle d'affichage rapide. */
function startGPS(mode = 'HIGH_FREQ') {
    if (wID !== null || emergencyStopActive) return; 
    
    // 1. Démarrer la géolocalisation
    wID = navigator.geolocation.watchPosition(gpsSuccess, gpsError, GPS_OPTS[mode]);
    
    // 2. Démarrer les capteurs IMU
    startIMUListeners(); 

    // 3. Mettre à jour l'affichage
    if ($('start-btn')) $('start-btn').innerHTML = '⏸️ PAUSE GPS'; 
    if ($('gps-status')) $('gps-status').textContent = `Actif (Mode ${mode})`;
    
    // 4. Assurer que la boucle d'affichage rapide est lancée
    if (domFastID === null) startFastLoop();
}

/** Arrête l'acquisition GPS, les capteurs IMU et la boucle d'affichage. */
function stopGPS(isManualReset = false) {
    if (wID !== null) { 
        navigator.geolocation.clearWatch(wID); 
        wID = null; // Désactive l'état GPS
    }
    
    // 1. Arrêter les capteurs IMU
    stopIMUListeners();
    
    // 2. Arrêter la boucle d'affichage rapide
    if (domFastID) { cancelAnimationFrame(domFastID); domFastID = null; }

    // 3. Mettre à jour l'affichage
    if ($('start-btn')) $('start-btn').innerHTML = '▶️ MARCHE GPS';
    if ($('gps-status')) $('gps-status').textContent = isManualReset ? "INACTIF (Manuel)" : "INACTIF";
}
// =================================================================
// BLOC 4/5 : BOUCLE D'AFFICHAGE RAPIDE (requestAnimationFrame)
// =================================================================

/** Boucle d'affichage rapide basée sur requestAnimationFrame */
function startFastLoop() {
    const loop = () => {
        
        // --- CALCULS EN TEMPS RÉEL ---
        const currentSpeedKmH = kSpd * KMH_MS; // Vitesse UKF en km/h
        maxSpd = Math.max(maxSpd, kSpd);
        
        // Facteur de Lorentz (γ)
        const lorentzFactor = 1 / Math.sqrt(1 - Math.pow(kSpd / C_L, 2));
        
        // Énergie Cinétique (J)
        const kineticEnergy = 0.5 * currentMass * Math.pow(kSpd, 2);
        
        // Pression Dynamique et Mach
        const dynamicPressure = 0.5 * currentAirDensity * Math.pow(kSpd, 2);
        const machNumber = kSpd / currentSpeedOfSound;

        // --- MISE À JOUR DU DOM ---
        
        // Vitesse
        if ($('speed-instant')) $('speed-instant').textContent = dataOrDefault(currentSpeedKmH, 2, ' km/h', '--.- km/h');
        if ($('speed-stable-ms')) $('speed-stable-ms').textContent = dataOrDefault(kSpd, 2, ' m/s', '-- m/s');
        if ($('speed-max')) $('speed-max').textContent = dataOrDefault(maxSpd * KMH_MS, 5, ' km/h');
        
        // Distance/Temps
        if ($('temps-ecoule-session')) $('temps-ecoule-session').textContent = dataOrDefault((Date.now() - sessionStartTime) / 1000, 2, ' s');
        if ($('time-moving')) $('time-moving').textContent = dataOrDefault(timeMoving, 2, ' s', '0.00 s');
        if ($('distance-total-km')) $('distance-total-km').textContent = `${dataOrDefault(distM / 1000, 3, ' km')} | ${dataOrDefault(distM, 2, ' m')}`;

        // Relativité / Dynamique
        if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(lorentzFactor, 4);
        if ($('speed-of-light-perc')) $('speed-of-light-perc').textContent = dataOrDefaultExp((kSpd / C_L) * 100, 2, ' %');
        if ($('energy-cinetique')) $('energy-cinetique').textContent = dataOrDefaultExp(kineticEnergy, 2, ' J', 'N/A');
        if ($('dynamic-pressure')) $('dynamic-pressure').textContent = dataOrDefault(dynamicPressure, 2, ' Pa');
        if ($('mach-number')) $('mach-number').textContent = dataOrDefault(machNumber, 4, '', '0.0000');
        if ($('force-g-max')) $('force-g-max').textContent = dataOrDefault(maxGForce, 3, ' G', '0.000 G');

        // Demande la prochaine frame
        domFastID = requestAnimationFrame(loop);
    };
    
    // Lancement initial de la boucle
    if (domFastID === null) domFastID = requestAnimationFrame(loop);
            }
// =================================================================
// BLOC 5/5 : INITIALISATION DES CONTRÔLES SYSTÈME (INIT)
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
        if ($('emergency-status')) $('emergency-status').textContent = emergencyStopActive ? 'ACTIF 🔴' : 'INACTIF 🟢';
        if (emergencyStopActive) stopGPS(true); 
    });

    // Contrôle : TOUT RÉINITIALISER
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { 
        if (confirm("Êtes-vous sûr de vouloir TOUT réinitialiser ?")) {
            stopGPS(true); 
            // Réinitialisation de l'UKF et des variables de session
            ukf = new ProfessionalUKF(43.2964, 5.3697, RHO_SEA_LEVEL);
            distM = 0; maxSpd = 0; timeMoving = 0; maxGForce = 0;
            window.location.reload(); 
        }
    });

    // Contrôle : Réinitialiser Distance
    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => {
        if (!emergencyStopActive) { distM = 0; timeMoving = 0; }
    });
    
    // Contrôle : Réinitialiser Vitesse Max
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', () => {
        if (!emergencyStopActive) { maxSpd = 0.0; maxGForce = 0.0; }
    });
    
    // Contrôle : Masse de l'objet (kg)
    if ($('mass-input')) $('mass-input').addEventListener('input', (e) => {
        currentMass = parseFloat(e.target.value) || 70.0;
        if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
    });
    
    // Contrôle : Corps Céleste
    if ($('celestial-body-select')) $('celestial-body-select').addEventListener('change', (e) => {
        currentCelestialBody = e.target.value;
        const { G_ACC_NEW } = updateCelestialBody(currentCelestialBody, kAlt, 0, 0);
        if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
    });
    
    // Contrôle : Mode Nether
    if ($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', () => {
        netherMode = !netherMode;
        if ($('mode-nether')) $('mode-nether').textContent = netherMode ? 'ACTIF (1:8) 🔥' : 'DÉSACTIVÉ (1:1) 🌍';
    });
    
    // Initialisation de l'affichage par défaut
    if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
    if ($('gravity-base')) $('gravity-base').textContent = `${G_ACCEL.toFixed(4)} m/s²`;
}

/** Fonction d'initialisation principale */
function init() {
    // 1. Démarrage des boucles lentes (Météo/Astro/NTP)
    syncH(); 
    startSlowLoop(); 
    
    // 2. Démarrage de la boucle d'affichage rapide (pour les valeurs par défaut)
    startFastLoop(); 
    
    // 3. Initialisation des gestionnaires d'événements
    initControls(); 
}

// Lancement du système au chargement complet de la page
document.addEventListener('DOMContentLoaded', init);
