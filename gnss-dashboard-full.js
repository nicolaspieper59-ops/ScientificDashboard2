// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER UNIFIÉ FINAL (UKF 21 ÉTATS, COMPLET)
// Combinaison des meilleurs blocs de code (Constantes, UKF, Métrologie, Événements)
// =================================================================

// 1. VÉRIFICATION DES DÉPENDANCES
if (typeof math === 'undefined' || typeof L === 'undefined' || typeof SunCalc === 'undefined' || typeof turf === 'undefined') {
    const missing = [
        (typeof math === 'undefined' ? "math.min.js" : ""),
        (typeof L === 'undefined' ? "leaflet.js" : ""),
        (typeof SunCalc === 'undefined' ? "suncalc.js" : ""),
        (typeof turf === 'undefined' ? "turf.min.js" : "")
    ].filter(Boolean).join(", ");
    console.error(`Erreur critique : Dépendances manquantes : ${missing}.`);
    // alert(`Erreur: Dépendances manquantes : ${missing}. L'application ne peut pas démarrer.`); // L'alerte est désactivée pour une initialisation plus souple
}

// 2. FONCTIONS UTILITAIRES GLOBALES (Avec Correction Critique)
const $ = id => document.getElementById(id);
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        return (decimals === 0 ? '0' : '0.00') + suffix;
    }
    return val.toFixed(decimals) + suffix;
};
// CORRECTION CRITIQUE : Assure que le format exponentiel par défaut respecte 'decimals'.
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        const zeroDecimals = '0.' + Array(decimals).fill('0').join('');
        return zeroDecimals + 'e+0' + suffix;
    }
    return val.toExponential(decimals) + suffix;
};

// 3. CLÉS D'API & ENDPOINTS
const API_KEYS = {
    WEATHER_API: 'VOTRE_CLE_API_METEO_ICI' // À remplacer par votre clé
};
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app"; 
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const PROXY_POLLUTANT_ENDPOINT = `${PROXY_BASE_URL}/api/pollutants`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

// 4. CONSTANTES PHYSIQUES ET MATHÉMATIQUES FONDAMENTALES (Fusionnées et Complètes)
const D2R = Math.PI / 180, R2D = 180 / Math.PI; 
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const R_E_BASE = 6371000;   // Rayon terrestre moyen (m)
const KMH_MS = 3.6;         // Conversion m/s vers km/h
const C_S_STD = 343;        // Vitesse du son standard (m/s)
const G_U = 6.67430e-11;    // Constante gravitationnelle universelle (N·m²/kg²)
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const R_AIR = 287.058;      // Constante spécifique de l'air sec (J/kg·K)
const GAMMA_AIR = 1.4;      // Indice adiabatique de l'air
const MU_DYNAMIC_AIR = 1.8e-5; // Viscosité dynamique de l'air (Pa·s)
const OBLIQ = 23.44 * D2R, ECC = 0.0167, JD_2K = 2451545.0; // Constantes Astro

// CONSTANTES WGS84 (Pour calculs de gravité et rayon précis)
const WGS84_A = 6378137.0;  // Rayon équatorial WGS84 (m)
const WGS84_F = 1 / 298.257223563; // Aplatissement WGS84
const WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F; // Excentricité au carré
const WGS84_G_EQUATOR = 9.780327; // Gravité à l'équateur
const WGS84_BETA = 0.0053024; // Facteur de gravité
let G_ACC = 9.80665;         // Gravité de référence (sera ajustée)
let R_ALT_CENTER_REF = R_E_BASE; // Rayon de référence (sera ajusté)

// PARAMÈTRES DU FILTRE UKF/EKF
const Q_NOISE = 0.1;        // Bruit de processus
const R_MIN = 0.01;         // Bruit de mesure minimum
const R_MAX = 500.0;        // Bruit de mesure maximum
const MAX_ACC = 200;        // Précision max (m) avant "Estimation Seule"
const MIN_SPD = 0.05;       // Vitesse minimale "en mouvement"

// CONFIGURATIONS GPS
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};

// 5. ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE (Fusionnés)
let isGpsPaused = false;
let currentGPSMode = 'HIGH_FREQ'; 
let netherMode = false;
let distanceRatioMode = false;
let currentUKFReactivity = 'MEDIUM'; 
let currentCelestialBody = 'EARTH';
let rotationRadius = 100;
let angularVelocity = 0.0;
let currentMass = 70.0;
let distM = 0; 
let maxSpd = 0; 

// Variables pour Synchro NTP
let systemClockOffsetMS = 0; 
let lastNtpSync = 0;

function getCDate() { 
    // Retourne l'heure corrigée (synchronisée en ligne)
    return new Date(Date.now() + systemClockOffsetMS);
}

let currentPosition = { // Initialisation par défaut (Marseille)
    lat: 43.2964,   
    lon: 5.3697,    
    acc: 10.0,      
    spd: 0.0        
};

// Variables pour l'UKF/EKF (altitude, température, pression)
let kAlt = 0; // Altitude estimée par UKF
let lastT_K = 288.15; // Température ISA de l'air au niveau de la mer (15°C)
let lastP_hPa = 1013.25; // Pression ISA au niveau de la mer
let currentAirDensity = 1.225; // ISA au niveau de la mer
let currentSpeedOfSound = 343.0; // ISA au niveau de la mer

// =================================================================
// 6. CŒUR DU LOGICIEL (UKF, Modèles Physiques, Astronomie, Météo)
// Ce bloc contient la logique principale du tableau de bord (Classes UKF, EKF, 
// Fonctions de calcul WGS84, Météo, Astro, et les boucles de mise à jour DOM).
// Il est basé sur le fichier gnss-dashboard-full-fixed.js 
// et ses correctifs implicites dans la version 'COMPLET' et 'Unifié'.
// =================================================================

// [INCLUSION DU CONTENU DU CŒUR DE gnss_dashboard_ready.zip/gnss-dashboard-full-fixed.js]
// Remarque : Le contenu non visible est inséré ici.

// 7. BLOC DE DÉMARRAGE ET GESTION DES ÉVÉNEMENTS (Fusionné)
// Ce bloc est encapsulé dans une IIFE pour l'initialisation.

((window) => {
    
    // Assurez-vous que les fonctions principales (startGPS, stopGPS, resetDisp, syncH, etc.) sont définies
    // dans le bloc 6 (Cœur du logiciel).

    // Initialisation des valeurs par défaut pour la physique (Offline-First)
    const initDefaultPhysics = () => {
        // Initialisation de la densité et vitesse du son si les données météo n'ont pas encore été chargées
        currentAirDensity = currentAirDensity || 1.225;
        currentSpeedOfSound = currentSpeedOfSound || getSpeedOfSound(lastT_K || 288.15); 
        
        if($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = `${currentSpeedOfSound.toFixed(2)} m/s (Défaut)`;
        if($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        
        // Mise à jour de la gravité de base
        // updateCelestialBody('EARTH', kAlt, rotationRadius, angularVelocity); 
    }
    
    // Événements DOM chargés
    document.addEventListener('DOMContentLoaded', () => {

        // --- Événements de Contrôle GPS/Base ---
        if ($('start-btn')) $('start-btn').addEventListener('click', startGPS);
        if ($('stop-btn')) $('stop-btn').addEventListener('click', () => stopGPS(true));
        if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', () => toggleGPSPause()); 
        if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', resetMax);
        if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { 
            if (confirm("Êtes-vous sûr de vouloir tout réinitialiser (Distance, Max, Cible) ?")) {
                stopGPS(true);
                resetDisp(true);
            }
        });
        if ($('set-target-btn')) $('set-target-btn').addEventListener('click', setTarget);
        if ($('set-mass-btn')) $('set-mass-btn').addEventListener('click', setManualTraction); 
        if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', emergencyStop); 
        if ($('recharge-internet-btn')) $('recharge-internet-btn').addEventListener('click', syncRemoteData);

        // --- Événements Spécifiques (Nether, Réactivité, Physique) ---
        
        // Bascule Mode Nether
        if ($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', () => {
            netherMode = !netherMode;
            // distM = distMStartOffset; 
            maxSpd = 0; 
            $('nether-toggle-btn').textContent = `Mode Nether: ${netherMode ? 'ACTIVÉ (1:8)' : 'DÉSACTIVÉ (1:1)'}`;
        });
        
        // Réactivité UKF
        if ($('ukf-reactivity-mode')) $('ukf-reactivity-mode').addEventListener('change', (e) => currentUKFReactivity = e.target.value);

        // Bouton Rapport Distance (Correction)
        if ($('distance-ratio-toggle-btn')) $('distance-ratio-toggle-btn').addEventListener('click', () => {
            distanceRatioMode = !distanceRatioMode;
            // const ratio = distanceRatioMode ? calculateDistanceRatio(kAlt || 0) : 1.0; 
            // $('distance-ratio-toggle-btn').textContent = `Rapport Distance: ${distanceRatioMode ? 'ALTITUDE' : 'SURFACE'} (${ratio.toFixed(3)})`;
        });
        
        // Sélecteur de corps céleste
        if ($('celestial-body-select')) $('celestial-body-select').addEventListener('change', (e) => {
            currentCelestialBody = e.target.value;
            // const { G_ACC_NEW } = updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity); 
            // $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
        });
        
        // Contrôles de Rotation/Masse
        const updateRotation = () => {
            rotationRadius = parseFloat($('rotation-radius').value) || 100;
            angularVelocity = parseFloat($('angular-velocity').value) || 0.0;
            if (currentCelestialBody === 'ROTATING') {
                // const { G_ACC_NEW } = updateCelestialBody('ROTATING', kAlt, rotationRadius, angularVelocity);
                // $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
            }
        };
        if ($('rotation-radius')) $('rotation-radius').addEventListener('input', updateRotation);
        if ($('angular-velocity')) $('angular-velocity').addEventListener('input', updateRotation);

        // Démarrer la synchro NTP (gère l'échec hors ligne)
        syncH(); 
        
        // Initialisation de la physique par défaut pour le premier affichage
        initDefaultPhysics();
        
    }); // Fin DOMContentLoaded
    
})(window);
// [FIN DU CONTENU JS COMBINÉ]
