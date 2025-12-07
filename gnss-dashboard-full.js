// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// VERSION 4.0 : FIX DÉFINITIF (ID, DOMContentLoaded, Logging Amélioré)
// =================================================================

// 🚨 DEBUG CRITIQUE 1 : Vérifie si le fichier est lu par le navigateur.
console.log(">>> V4.0 SCRIPT CHARGÉ. Le fichier est lu.");

// --- BLOC 1 : CONSTANTES ET UTILITAIRES DE BASE ---

const $ = id => document.getElementById(id);

const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const KMH_MS = 3.6;         
const C_L = 299792458;      
const G_U = 6.67430e-11;    
const G_STD = 9.8067;       
const DOM_FAST_UPDATE_MS = 100; // Intervalle de rafraîchissement rapide (0.1s)
const DOM_SLOW_UPDATE_MS = 2000; 

const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || val === Infinity || val === -Infinity || Math.abs(val) < 1e-9) { 
        return (decimals === 0 ? '--' : '--.--') + suffix; 
    }
    return val.toFixed(decimals) + suffix;
};

const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || val === Infinity || val === -Infinity) {
        return 'N/A' + suffix;
    }
    return val.toExponential(decimals) + suffix;
};


// --- BLOC 2 : ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE ---

let isGpsRunning = false;
let timeTotalSeconds = 0;
let timeMovingSeconds = 0;
let lastNTPDate = null; 
let lastLocalTime = null; 

let currentPosition = { 
    lat: 43.2964,   // Default: Marseille
    lon: 5.3697,    
    alt: 0.0,
    acc: 10.0,      
    spd: 0.0        
};

let ukf = null;
let kAlt = 0.0;     
let kSpd = 0.0;     
let kVVert = 0.0;   


// --- BLOC 3 : FONCTIONS DE TEMPS ---

const syncH = async () => {
    // ID vérifié: 'heure-locale'
    if ($('heure-locale')) $('heure-locale').textContent = 'SYNCHRO...';
    try {
        const response = await fetch(SERVER_TIME_ENDPOINT);
        if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`);
        const data = await response.json();
        
        lastNTPDate = new Date(data.utc_datetime); 
        lastLocalTime = Date.now(); 
        // ID vérifié: 'date-heure-utc'
        if ($('date-heure-utc')) $('date-heure-utc').textContent = lastNTPDate.toTimeString().split(' ')[0] + ' UTC';
        
    } catch (e) {
        console.warn("🔴 Échec de la synchronisation NTP. Utilisation de l'horloge locale.", e.message);
        if ($('heure-locale')) $('heure-locale').textContent = 'SYNCHRO ÉCHOUÉE';
        if ($('date-heure-utc')) $('date-heure-utc').textContent = 'HORLOGE LOCALE';
    }
};

const getCDate = () => {
    if (!lastNTPDate || !lastLocalTime) {
        return new Date(); 
    }
    const localTimeDifference = Date.now() - lastLocalTime;
    return new Date(lastNTPDate.getTime() + localTimeDifference);
};


// --- BLOC 4 : GESTION DES CAPTEURS ET GPS ---

const activateDeviceMotion = () => {
    console.warn("🟡 La fonction 'activateDeviceMotion' n'est pas implémentée ou chargée.");
    // ID vérifié: 'statut-capteur'
    if ($('statut-capteur')) $('statut-capteur').textContent = 'IMU Non implémenté';
};

const handleGeolocation = (pos) => {
    const { latitude, longitude, altitude, accuracy, speed } = pos.coords;
    
    currentPosition = { 
        lat: latitude, 
        lon: longitude, 
        alt: altitude, 
        acc: accuracy, 
        spd: speed || 0.0,
        time: pos.timestamp 
    };
    
    try {
        // VÉRIFIE si ProfessionalUKF a été défini par ukf-lib.js
        if (window.ukf && typeof window.ukf.update === 'function') {
            window.ukf.update(currentPosition); 
        }
    } catch (ukfError) {
        console.error("🔴 Échec de la mise à jour UKF (Fonction update) :", ukfError.message);
    }
    
    isGpsRunning = true;
    // ID vérifié: 'statut-gps-acquisition'
    if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = 'Actif 🟢';
    // ID vérifié: 'speed-raw-ms'
    if ($('speed-raw-ms')) $('speed-raw-ms').textContent = dataOrDefault(currentPosition.spd, 2, ' m/s');
    // ID vérifié: 'acc-gps'
    if ($('acc-gps')) $('acc-gps').textContent = dataOrDefault(currentPosition.acc, 2, ' m');
};

const initGPS = () => {
    if ('geolocation' in navigator) {
        navigator.geolocation.watchPosition(handleGeolocation, (error) => {
            console.error("🔴 ERREUR GPS:", error.code, error.message);
            isGpsRunning = false;
            if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = 'Erreur/Refus 🔴';
        }, { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 });
    } else {
        if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = 'Non supporté 🟡';
    }
    if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = 'Acquisition en cours...';
};


// --- BLOC 5 : MISES À JOUR PÉRIODIQUES DU DOM ---

const updateDOMFast = () => {
    try { 
        // --- TEMPS ÉCOULÉ (DOIT S'INCRÉMENTER) ---
        timeTotalSeconds += (DOM_FAST_UPDATE_MS / 1000); 
        // ID vérifié: 'elapsed-session-time'
        if ($('elapsed-session-time')) $('elapsed-session-time').textContent = `${timeTotalSeconds.toFixed(2)} s`;
        // ID vérifié: 'elapsed-motion-time'
        if ($('elapsed-motion-time')) $('elapsed-motion-time').textContent = `${timeMovingSeconds.toFixed(2)} s`;
        
        // --- VITESSE & RELATIVITÉ ---
        const instVitesseKmH = currentPosition.spd * KMH_MS;
        // ID vérifié: 'speed-3d-inst'
        if ($('speed-3d-inst')) $('speed-3d-inst').textContent = dataOrDefault(instVitesseKmH, 1, ' km/h');
        
        // ID vérifié: 'lorentz-factor'
        const gamma = 1 / Math.sqrt(1 - Math.pow(currentPosition.spd / C_L, 2));
        if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(gamma, 4);
        
        // --- PHYSIQUE STATIQUE ---
        // ID vérifié: 'const-c'
        if ($('const-c')) $('const-c').textContent = `${C_L.toFixed(0)} m/s`;
        // ID vérifié: 'const-G'
        if ($('const-G')) $('const-G').textContent = dataOrDefaultExp(G_U, 5, ' m³/kg/s²');
        // ID vérifié: 'gravity-base'
        if ($('gravity-base')) $('gravity-base').textContent = `${G_STD.toFixed(4)} m/s²`;
        

    } catch (e) {
        console.error("🔴 ERREUR NON GÉRÉE dans updateDOMFast (La boucle continue)", e.message);
    }
    
    setTimeout(updateDOMFast, DOM_FAST_UPDATE_MS);
};


const updateDOMSlow = () => {
    try { 

        // --- HORLOGE ET DATE ---
        const now = getCDate(); 
        if (now) {
            // ID vérifié: 'heure-locale'
            if ($('heure-locale') && !$('heure-locale').textContent.includes('SYNCHRO ÉCHOUÉE')) {
                $('heure-locale').textContent = now.toLocaleTimeString('fr-FR');
            }
            // ID vérifié: 'date-heure-utc'
            if ($('date-heure-utc')) $('date-heure-utc').textContent = now.toUTCString().split(' ')[4] + ' UTC';
            // ID vérifié: 'date-display-astro' (Vérifier si cet ID existe dans l'HTML)
            if ($('date-display-astro')) $('date-display-astro').textContent = now.toLocaleDateString('fr-FR');
        }
        
        // ... (Logique Astro et Météo) ...

    } catch (e) {
        console.error("🔴 ERREUR NON GÉRÉE dans updateDOMSlow (La boucle continue)", e.message);
    }
    
    setTimeout(updateDOMSlow, DOM_SLOW_UPDATE_MS);
};


// --- BLOC 6 : INITIALISATION DU SYSTÈME (Fonction) ---
const initializeDashboard = () => {
    
    // 🚨 DEBUG CRITIQUE 2 : Confirme que la fonction d'initialisation est appelée.
    console.log(">>> V4.0 INITIALIZATION START. Starting checks.");

    // 1. Initialisation Conditionnelle de l'UKF
    if (typeof math === 'undefined') {
        console.error("🔴 ERREUR : math.min.js est manquant. Le filtre UKF ne peut pas démarrer.");
        // Remplacez 'statut-ekf-fusion' par l'ID utilisé pour le statut UKF dans votre HTML
        if ($('statut-ekf-fusion')) $('statut-ekf-fusion').textContent = 'ERREUR (math.js manquant) 🔴';
        
    } else if (typeof ProfessionalUKF !== 'undefined') { 
        try {
            // C'est la ligne CRITIQUE qui appelle votre code dans ukf-lib.js
            window.ukf = new ProfessionalUKF(); 
            console.log("UKF 21 États Initialisé. 🟢");
            if ($('statut-ekf-fusion')) $('statut-ekf-fusion').textContent = 'Initialisé 🟢';
        } catch (e) {
            // Cette erreur se produit si une erreur est dans le constructeur de ProfessionalUKF
            console.error("🔴 ÉCHEC D'INITIALISATION UKF DANS LE CONSTRUCTEUR: " + e.message);
            if ($('statut-ekf-fusion')) $('statut-ekf-fusion').textContent = 'ERREUR CONSTRUCTEUR 🔴';
        }
    } else {
        // Cela se produit si ukf-lib.js est chargé, mais la classe n'est pas définie
        console.error("🔴 ÉCHEC CRITIQUE : La classe ProfessionalUKF n'est pas définie. Vérifiez lib/ukf-lib.js.");
        if ($('statut-ekf-fusion')) $('statut-ekf-fusion').textContent = 'ERREUR (Classe manquante) 🔴';
    }
    
    // 2. Initialisation des capteurs (GPS et IMU)
    initGPS(); 
    
    const activateButton = document.getElementById('activate-sensors-btn');
    if (activateButton && typeof activateDeviceMotion === 'function') {
        activateButton.addEventListener('click', activateDeviceMotion); 
    }
    
    // 3. Démarrage de la synchronisation NTP (réseau)
    syncH(); 

    // 4. Démarrage des boucles de rafraîchissement (CRITICAL STEP)
    updateDOMFast(); // Le compteur de temps s'incrémente ici
    updateDOMSlow();
    
    // 🚨 DEBUG CRITIQUE 3 : Confirme que les boucles de rafraîchissement ont été appelées.
    console.log(">>> V4.0 INITIALIZATION COMPLETE. Loops started.");
};


// --- BLOC 7 : POINT D'ENTRÉE DU SCRIPT (Le plus robuste) ---

// S'assure que le script se lance dès que le HTML est prêt.
document.addEventListener('DOMContentLoaded', initializeDashboard);

// Fallback: Si le script est chargé après l'événement DOMContentLoaded.
if (document.readyState === 'complete' || document.readyState === 'interactive') {
    setTimeout(initializeDashboard, 10); 
    }
