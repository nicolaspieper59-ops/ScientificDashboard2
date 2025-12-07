// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// VERSION 3.3 : CORRECTION D'ID & DÉMARRAGE FORCÉ (DOMContentLoaded)
// =================================================================

// --- BLOC 1 : CONSTANTES ET UTILITAIRES DE BASE ---

const $ = id => document.getElementById(id);

const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const KMH_MS = 3.6;         
const C_L = 299792458;      
const G_U = 6.67430e-11;    
const G_STD = 9.8067;       
// La mise à jour rapide doit rester à 100ms (0.1s)
const DOM_FAST_UPDATE_MS = 100;
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
    if ($('heure-locale')) $('heure-locale').textContent = 'SYNCHRO...';
    try {
        const response = await fetch(SERVER_TIME_ENDPOINT);
        if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`);
        const data = await response.json();
        
        lastNTPDate = new Date(data.utc_datetime); 
        lastLocalTime = Date.now(); 
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
        if (window.ukf && typeof window.ukf.update === 'function') {
            window.ukf.update(currentPosition); 
        }
    } catch (ukfError) {
        console.error("🔴 Échec de la mise à jour UKF (Fonction update) :", ukfError.message);
    }
    
    isGpsRunning = true;
    if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = 'Actif 🟢';
    if ($('speed-raw-ms')) $('speed-raw-ms').textContent = dataOrDefault(currentPosition.spd, 2, ' m/s');
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
        timeTotalSeconds += (DOM_FAST_UPDATE_MS / 1000); // Utiliser la vraie durée
        if ($('elapsed-session-time')) $('elapsed-session-time').textContent = `${timeTotalSeconds.toFixed(2)} s`;
        if ($('elapsed-motion-time')) $('elapsed-motion-time').textContent = `${timeMovingSeconds.toFixed(2)} s`;
        
        // --- VITESSE & RELATIVITÉ ---
        const instVitesseKmH = currentPosition.spd * KMH_MS;
        if ($('speed-3d-inst')) $('speed-3d-inst').textContent = dataOrDefault(instVitesseKmH, 1, ' km/h');
        
        const gamma = 1 / Math.sqrt(1 - Math.pow(currentPosition.spd / C_L, 2));
        if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(gamma, 4);
        
        // --- PHYSIQUE STATIQUE ---
        if ($('const-c')) $('const-c').textContent = `${C_L.toFixed(0)} m/s`;
        if ($('const-G')) $('const-G').textContent = dataOrDefaultExp(G_U, 5, ' m³/kg/s²');
        if ($('gravity-base')) $('gravity-base').textContent = `${G_STD.toFixed(4)} m/s²`;
        

    } catch (e) {
        console.error("🔴 ERREUR NON GÉRÉE dans updateDOMFast (La boucle continue)", e.message);
    }
    
    // Le DOM_FAST_UPDATE_MS assure une récurrence correcte de 0.1s
    setTimeout(updateDOMFast, DOM_FAST_UPDATE_MS); 
};


const updateDOMSlow = () => {
    try { 

        // --- HORLOGE ET DATE ---
        const now = getCDate(); 
        if (now) {
            if ($('heure-locale') && !$('heure-locale').textContent.includes('SYNCHRO ÉCHOUÉE')) {
                $('heure-locale').textContent = now.toLocaleTimeString('fr-FR');
            }
            if ($('date-heure-utc')) $('date-heure-utc').textContent = now.toUTCString().split(' ')[4] + ' UTC';
            if ($('date-display-astro')) $('date-display-astro').textContent = now.toLocaleDateString('fr-FR');
        }

        // --- ASTRO (Mise à jour lente) ---
        const lat = currentPosition.lat;
        const lon = currentPosition.lon;
        
        if (typeof calculateAstroDataHighPrec === 'function' && lat !== 43.2964) { 
            try { 
                const astroData = calculateAstroDataHighPrec(now, lat, lon);
                
                // MAPPING ASTRONOMIE
                if ($('tst')) $('tst').textContent = astroData.TST_HRS;
                if ($('mst')) $('mst').textContent = astroData.MST_HRS;
                if ($('eot')) $('eot').textContent = astroData.EOT_MIN + ' min';
                if ($('ecl-long')) $('ecl-long').textContent = astroData.ECL_LONG + '°';
                if ($('noon-solar')) $('noon-solar').textContent = astroData.NOON_SOLAR_UTC.toTimeString().split(' ')[0] + ' UTC';
                
                if (typeof getTSLV === 'function' && $('tslv')) {
                    $('tslv').textContent = getTSLV(now, lon);
                }

                // SOLEIL
                if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(astroData.sun.altitude * R2D, 2, '°'); 
                if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(astroData.sun.azimuth * R2D, 2, '°'); 
                
                if (astroData.sun.sunrise && astroData.sun.sunset && $('day-duration')) {
                     const diffMs = astroData.sun.sunset.getTime() - astroData.sun.sunrise.getTime();
                     const diffH = Math.floor(diffMs / (1000 * 60 * 60));
                     const diffM = Math.floor((diffMs % (1000 * 60 * 60)) / (1000 * 60));
                     $('day-duration').textContent = `${diffH}h ${diffM}m`;
                }

                if ($('sunrise-times')) $('sunrise-times').textContent = astroData.sun.sunrise ? astroData.sun.sunrise.toLocaleTimeString('fr-FR') : 'N/A';
                if ($('sunset-times')) $('sunset-times').textContent = astroData.sun.sunset ? astroData.sun.sunset.toLocaleTimeString('fr-FR') : 'N/A';
                
                // LUNE
                if ($('moon-phase-name') && typeof getMoonPhaseName === 'function') {
                    $('moon-phase-name').textContent = getMoonPhaseName(astroData.moon.illumination.phase);
                }
                if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(astroData.moon.illumination.fraction * 100, 1, ' %');
                if ($('moon-alt')) $('moon-alt').textContent = dataOrDefault(astroData.moon.position.altitude * R2D, 2, '°');
                if ($('moon-azimuth')) $('moon-azimuth').textContent = dataOrDefault(astroData.moon.position.azimuth * R2D, 2, '°');
                if ($('moon-distance')) $('moon-distance').textContent = dataOrDefault(astroData.moon.position.distance / 1000, 0, ' km');
                if ($('moon-times')) $('moon-times').textContent = 'N/A (Calcul complexe)';
                
            } catch (astroError) {
                console.error("🔴 ERREUR DANS LA LOGIQUE ASTRO : ", astroError.message);
                if ($('tst')) $('tst').textContent = `ASTRO ERREUR: ${astroError.message.substring(0, 20)}...`;
            }

        } else if (typeof calculateAstroDataHighPrec !== 'function') {
            if ($('tst')) $('tst').textContent = 'N/A (Astro.js manquant)';
        }

        if ($('statut-meteo')) $('statut-meteo').textContent = 'INACTIF (API requise)';

    } catch (e) {
        console.error("🔴 ERREUR NON GÉRÉE dans updateDOMSlow (La boucle continue)", e.message);
    }
    
    setTimeout(updateDOMSlow, DOM_SLOW_UPDATE_MS);
};


// --- BLOC 6 : INITIALISATION DU SYSTÈME (Fonction) ---
const initializeDashboard = () => {
    
    console.log("DEBUG: Initialization function called. Starting checks.");

    // 1. Initialisation Conditionnelle de l'UKF
    if (typeof math === 'undefined') {
        console.error("🔴 ERREUR : math.min.js est manquant. Le filtre UKF ne peut pas démarrer.");
        if ($('statut-ekf-fusion')) $('statut-ekf-fusion').textContent = 'ERREUR (math.js manquant) 🔴';
        
    } else if (typeof ProfessionalUKF !== 'undefined') { 
        try {
            window.ukf = new ProfessionalUKF(); 
            console.log("UKF 21 États Initialisé. 🟢");
            if ($('statut-ekf-fusion')) $('statut-ekf-fusion').textContent = 'Initialisé 🟢';
        } catch (e) {
            console.error("🔴 ÉCHEC D'INITIALISATION UKF DANS LE CONSTRUCTEUR: " + e.message);
            if ($('statut-ekf-fusion')) $('statut-ekf-fusion').textContent = 'ERREUR CONSTRUCTEUR 🔴';
        }
    } else {
        console.error("🔴 ÉCHEC CRITIQUE : La classe ProfessionalUKF n'est pas définie. Chargez lib/ukf-lib.js.");
        if ($('statut-ekf-fusion')) $('statut-ekf-fusion').textContent = 'ERREUR (Classe manquante) 🔴';
    }
    
    // 2. Initialisation des capteurs (GPS et IMU)
    initGPS(); 
    
    const activateButton = document.getElementById('activate-sensors-btn');
    if (activateButton) {
        if (typeof activateDeviceMotion === 'function') {
            activateButton.addEventListener('click', activateDeviceMotion); 
        } else {
             console.warn("🟡 AVERTISSEMENT : La fonction 'activateDeviceMotion' est manquante ou non définie. Le bouton IMU est inactif.");
        }
    } 
    
    // 3. Démarrage de la synchronisation NTP (réseau)
    syncH(); 

    // 4. Démarrage des boucles de rafraîchissement (CRITICAL STEP)
    updateDOMFast();
    updateDOMSlow();
    console.log("DEBUG: Initialization complete. Loops started.");
};


// --- BLOC 7 : POINT D'ENTRÉE DU SCRIPT (Le plus robuste) ---
console.log("DEBUG: Script gnss-dashboard-full.js CHARGÉ.");

// Utiliser DOMContentLoaded pour s'assurer que le HTML est prêt, mais sans attendre les ressources externes.
document.addEventListener('DOMContentLoaded', initializeDashboard);

// Fallback : Si l'événement est déjà passé (car le script est chargé très tardivement)
if (document.readyState === 'complete' || document.readyState === 'interactive') {
    // Si le code n'a pas encore été initialisé, appelez-le immédiatement.
    setTimeout(initializeDashboard, 10); 
}
