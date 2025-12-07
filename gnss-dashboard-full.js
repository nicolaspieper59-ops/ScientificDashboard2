// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// VERSION 2.0 : SÉCURITÉ MAXIMALE ANTI-SYNTAXE ET ANTI-CRASH
// =================================================================

// --- BLOC 1 : CONSTANTES ET UTILITAIRES DE BASE ---

const $ = id => document.getElementById(id);

const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const KMH_MS = 3.6;         
const C_L = 299792458;      
const G_U = 6.67430e-11;    // Constante gravitationnelle universelle
const G_STD = 9.8067;       // Gravité de Base
const DOM_SLOW_UPDATE_MS = 2000; 

const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

// Formatage des données (Anti-NaN/Null/Inf)
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


// --- BLOC 3 : FONCTIONS DE TEMPS (Incassables) ---

const syncH = async () => {
    if ($('local-time')) $('local-time').textContent = 'SYNCHRO...';
    try {
        const response = await fetch(SERVER_TIME_ENDPOINT);
        if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`);
        const data = await response.json();
        
        lastNTPDate = new Date(data.utc_datetime); 
        lastLocalTime = Date.now(); 
        if ($('date-gmt')) $('date-gmt').textContent = lastNTPDate.toTimeString().split(' ')[0] + ' UTC';
        
    } catch (e) {
        console.warn("🔴 Échec de la synchronisation NTP. Utilisation de l'horloge locale.", e.message);
        if ($('local-time')) $('local-time').textContent = 'SYNCHRO ÉCHOUÉE';
        if ($('date-gmt')) $('date-gmt').textContent = 'HORLOGE LOCALE';
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

// ⚠️ PLACEHOLDER IMU (Garantie que le script ne plante pas si la fonction est manquante)
const activateDeviceMotion = () => {
    console.warn("🟡 La fonction 'activateDeviceMotion' n'est pas implémentée ou chargée.");
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
    
    // 🚨 PROTECTION UKF : Appeler ukf.update doit être protégé
    try {
        if (window.ukf && typeof window.ukf.update === 'function') {
            window.ukf.update(currentPosition); 
        }
    } catch (ukfError) {
        console.error("🔴 Échec de la mise à jour UKF (Fonction update) :", ukfError.message);
    }
    
    isGpsRunning = true;
    if ($('gps-status')) $('gps-status').textContent = 'Actif 🟢';
    if ($('vitesse-brute-ms')) $('vitesse-brute-ms').textContent = dataOrDefault(currentPosition.spd, 2, ' m/s');
    if ($('precision-gps')) $('precision-gps').textContent = dataOrDefault(currentPosition.acc, 2, ' m');
};

const initGPS = () => {
    if ('geolocation' in navigator) {
        navigator.geolocation.watchPosition(handleGeolocation, (error) => {
            console.error("🔴 ERREUR GPS:", error.code, error.message);
            isGpsRunning = false;
            if ($('gps-status')) $('gps-status').textContent = 'Erreur/Refus 🔴';
        }, { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 });
    } else {
        if ($('gps-status')) $('gps-status').textContent = 'Non supporté 🟡';
    }
    if ($('gps-status')) $('gps-status').textContent = 'Acquisition en cours...';
};


// --- BLOC 5 : MISES À JOUR PÉRIODIQUES DU DOM ---

const updateDOMFast = () => {
    try { // ⬅️ PROTECTION ANTI-CRASH
        
        // --- TEMPS ÉCOULÉ (Devrait s'incrémenter même sans GPS) ---
        timeTotalSeconds += 0.1;
        if ($('time-total')) $('time-total').textContent = `${timeTotalSeconds.toFixed(2)} s`;
        if ($('time-moving')) $('time-moving').textContent = `${timeMovingSeconds.toFixed(2)} s`;
        
        // --- VITESSE BRUTE & EKF (Mise à jour pour affichage) ---
        const instVitesseKmH = currentPosition.spd * KMH_MS;
        if ($('vitesse-inst-kmh')) $('vitesse-inst-kmh').textContent = dataOrDefault(instVitesseKmH, 1, ' km/h');
        
        // --- PHYSIQUE STATIQUE ---
        if ($('vitesse-lumiere')) $('vitesse-lumiere').textContent = `${C_L.toFixed(0)} m/s`;
        if ($('gravitation-universelle')) $('gravitation-universelle').textContent = dataOrDefaultExp(G_U, 5, ' m³/kg/s²');
        if ($('gravity-base')) $('gravity-base').textContent = `${G_STD.toFixed(4)} m/s²`;
        
        // --- RELATIVITÉ (Se met à jour avec la vitesse actuelle) ---
        const gamma = 1 / Math.sqrt(1 - Math.pow(currentPosition.spd / C_L, 2));
        if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(gamma, 4);

    } catch (e) {
        console.error("🔴 ERREUR NON GÉRÉE dans updateDOMFast (La boucle continue)", e.message);
    }
    
    // 🚨 Le setTimeout garantit la récurrence, MÊME en cas d'erreur.
    setTimeout(updateDOMFast, 100);
};

const updateDOMSlow = () => {
    try { // ⬅️ PROTECTION ANTI-CRASH

        // --- HORLOGE ET DATE ---
        const now = getCDate(); 
        if (now) {
            if ($('local-time') && !$('local-time').textContent.includes('SYNCHRO ÉCHOUÉE')) {
                $('local-time').textContent = now.toLocaleTimeString('fr-FR');
            }
            if ($('date-display-utc')) $('date-display-utc').textContent = now.toUTCString().split(' ')[4] + ' UTC';
            if ($('date-astro')) $('date-astro').textContent = now.toLocaleDateString('fr-FR');
        }

        // --- ASTRO (Requiert lat/lon et les dépendances lib/astro.js) ---
        const lat = currentPosition.lat;
        const lon = currentPosition.lon;
        
        if (typeof calculateAstroDataHighPrec === 'function' && lat !== 'N/A') {
            try { 
                // ⚠️ La logique Astro complète doit être ici. (Ex: calculateAstroDataHighPrec(lat, lon, now); )
                
            } catch (astroError) {
                console.error("🔴 ERREUR DANS LA LOGIQUE ASTRO : ", astroError.message);
                if ($('tst')) $('tst').textContent = `ERREUR: ${astroError.message.substring(0, 10)}...`;
            }

        } else if (typeof calculateAstroDataHighPrec !== 'function') {
            if ($('tst')) $('tst').textContent = 'N/A (Astro.js manquant)';
        }

        // --- MÉTÉO (Requiert API et fetchWeather) ---
        if ($('meteo-status')) $('meteo-status').textContent = 'INACTIF (API requise)';

    } catch (e) {
        console.error("🔴 ERREUR NON GÉRÉE dans updateDOMSlow (La boucle continue)", e.message);
    }
    
    // Le setTimeout garantit la récurrence.
    setTimeout(updateDOMSlow, DOM_SLOW_UPDATE_MS);
};


// --- BLOC 6 : INITIALISATION DU SYSTÈME (window.onload) ---

window.onload = () => {
    
    // 1. Initialisation Conditionnelle de l'UKF (Non-bloquant)
    if (typeof math === 'undefined') {
        console.error("🔴 ERREUR : math.min.js est manquant. Le filtre UKF ne peut pas démarrer.");
        if ($('ekf-status')) $('ekf-status').textContent = 'ERREUR (math.js manquant) 🔴';
        
    } else if (typeof ProfessionalUKF !== 'undefined') { 
        try {
            window.ukf = new ProfessionalUKF(); 
            console.log("UKF 21 États Initialisé. 🟢");
            if ($('ekf-status')) $('ekf-status').textContent = 'Initialisé 🟢';
        } catch (e) {
            console.error("🔴 ÉCHEC D'INITIALISATION UKF DANS LE CONSTRUCTEUR: " + e.message);
            if ($('ekf-status')) $('ekf-status').textContent = 'ERREUR CONSTRUCTEUR 🔴';
        }
    } else {
        console.error("🔴 ÉCHEC CRITIQUE : La classe ProfessionalUKF n'est pas définie. Chargez lib/ukf-lib.js.");
        if ($('ekf-status')) $('ekf-status').textContent = 'ERREUR (Classe manquante) 🔴';
    }
    
    // 2. Initialisation des capteurs (GPS et IMU)
    initGPS(); 
    
    const activateButton = document.getElementById('activate-sensors-btn');
    if (activateButton) {
        // VÉRIFICATION DÉFENSIVE : Assure que la fonction existe avant d'ajouter l'écouteur.
        if (typeof activateDeviceMotion === 'function') {
            activateButton.addEventListener('click', activateDeviceMotion); 
        } else {
             console.warn("🟡 AVERTISSEMENT : La fonction 'activateDeviceMotion' n'est pas définie. Le bouton IMU est inactif.");
        }
    } 
    
    // 3. Démarrage de la synchronisation NTP (réseau)
    syncH(); 

    // 4. Démarrage des boucles de rafraîchissement (DOIT DÉMARRER QUOI QU'IL ARRIVE)
    updateDOMFast();
    updateDOMSlow();
};
