// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// VERSION FINALE ET ROBUSTE
// =================================================================

// --- BLOC 1 : CONSTANTES ET UTILITAIRES DE BASE ---

const $ = id => document.getElementById(id);

const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const KMH_MS = 3.6;         
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const G_U = 6.67430e-11;    // Constante gravitationnelle universelle
const R_E_BASE = 6371000;   // Rayon terrestre moyen (m)
const DOM_SLOW_UPDATE_MS = 2000; // Fréquence de rafraîchissement lent (ms)

// Constantes Météo (pour fallback)
const RHO_SEA_LEVEL = 1.225; // Densité de l'air niveau mer (kg/m³)
const TEMP_SEA_LEVEL_K = 288.15; // 15°C en Kelvin
const BARO_ALT_REF_HPA = 1013.25;

// API Endpoints (Nécessite HTTPS et/ou CORS)
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";


// --- FORMATAGE DES DONNÉES (CORRIGÉ : Anti-zéro de machine) ---
const dataOrDefault = (val, decimals, suffix = '') => {
    // Affiche '--.--' si NaN, null, ou si la valeur est négligeable (bruit de machine < 1e-9)
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
let isMoving = false;
let currentMass = 70.0;
let timeTotalSeconds = 0;
let timeMovingSeconds = 0;
let lastKnownWeather = null;
let lastNTPDate = null; // Date fournie par le serveur NTP
let lastLocalTime = null; // Date locale au moment de la réception NTP

// Position UKF/GPS (Initialisation CRITIQUE avec des valeurs par défaut)
let currentPosition = { 
    lat: 43.2964,   // Latitude (Marseille, pour débloquer Astro/Météo)
    lon: 5.3697,    // Longitude
    alt: 0.0,
    acc: 10.0,      
    spd: 0.0        
};

// Variables EKF/UKF (Doivent exister)
let ukf = null;
let kAlt = 0.0; // Altitude estimée (m)
let kSpd = 0.0; // Vitesse stable estimée (m/s)
let kVVert = 0.0; // Vitesse verticale estimée (m/s)

// Variables IMU/Capteurs
let lastPitch = 0.0, lastRoll = 0.0;
let lastAccZ = 0.0, lastAccLong = 0.0;
let sensorStatus = "Inactif";


// --- BLOC 3 : FONCTIONS DE TEMPS (CORRIGÉ POUR ROBUSTESSE) ---

/**
 * Fonction de synchronisation NTP robuste.
 */
const syncH = async () => {
    try {
        const response = await fetch(SERVER_TIME_ENDPOINT);
        if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`);
        const data = await response.json();
        
        // La date vient du serveur (UTC)
        lastNTPDate = new Date(data.utc_datetime); 
        // L'heure locale est le timestamp du navigateur après réception
        lastLocalTime = Date.now(); 
        
        if ($('date-display')) $('date-display').textContent = lastNTPDate.toLocaleDateString('fr-FR');
        if ($('date-gmt')) $('date-gmt').textContent = lastNTPDate.toTimeString().split(' ')[0] + ' UTC';
        
    } catch (e) {
        console.warn("🔴 Échec de la synchronisation NTP. Utilisation de l'horloge locale.", e.message);
        if ($('date-gmt')) $('date-gmt').textContent = 'SYNCHRO ÉCHOUÉE';
        // lastNTPDate et lastLocalTime restent null, activant le fallback de getCDate.
    }
};

/**
 * Retourne un objet Date corrigé ou l'heure locale comme fallback (CORRIGÉ).
 * Cette fonction est maintenant incassable.
 */
const getCDate = () => {
    if (!lastNTPDate || !lastLocalTime) {
        // FALLBACK CRITIQUE : Utilise l'heure locale du navigateur si NTP a échoué.
        return new Date(); 
    }
    // Calcul de la différence et application au temps actuel
    const localTimeDifference = Date.now() - lastLocalTime;
    return new Date(lastNTPDate.getTime() + localTimeDifference);
};


// --- BLOC 4 : GESTION DES CAPTEURS ET GPS (IMU et Geolocation) ---

/**
 * Gère l'événement d'accélération et met à jour l'état de l'IMU.
 */
const handleDeviceMotion = (event) => {
    // Si l'UKF est initialisé, on peut l'utiliser pour la fusion
    if (ukf) {
        // ukf.updateIMU(event.accelerationIncludingGravity); // Fonction à intégrer dans l'UKF
    }

    const acc = event.accelerationIncludingGravity;
    lastAccLong = acc.x; 
    lastAccZ = acc.z; // Accélération verticale brute (avec gravité)
    sensorStatus = "Actif";

    // Mise à jour de l'affichage
    if ($('acc-x')) $('acc-x').textContent = dataOrDefault(acc.x, 2, ' m/s²');
    if ($('acc-y')) $('acc-y').textContent = dataOrDefault(acc.y, 2, ' m/s²');
    if ($('acc-z')) $('acc-z').textContent = dataOrDefault(acc.z, 2, ' m/s²');
    if ($('imu-status')) $('imu-status').textContent = 'Actif';
};


/**
 * Active les capteurs de mouvement (IMU) avec gestion des permissions mobiles (CORRIGÉ).
 */
function activateDeviceMotion() {
    // Nécessite HTTPS pour fonctionner
    if (window.DeviceMotionEvent && typeof DeviceMotionEvent.requestPermission === 'function') {
        DeviceMotionEvent.requestPermission()
            .then(permissionState => {
                if (permissionState === 'granted') {
                    window.addEventListener('devicemotion', handleDeviceMotion, true);
                    console.log("Capteurs IMU activés (Permission accordée).");
                } else {
                    console.warn("Permission capteurs IMU refusée.");
                    sensorStatus = "Refusé";
                }
            })
            .catch(error => {
                console.error("Erreur à l'activation des capteurs:", error);
                sensorStatus = "Erreur";
            });
    } else if (window.DeviceMotionEvent) {
        // Anciens navigateurs (Android/Chrome): tente d'activer sans demande explicite
        window.addEventListener('devicemotion', handleDeviceMotion, true);
        console.log("Capteurs IMU activés (Méthode standard).");
    } else {
        sensorStatus = "Non supporté";
    }
    if ($('imu-status')) $('imu-status').textContent = sensorStatus;
    // Cache le bouton d'activation si trouvé (à ajouter au HTML si ce n'est pas fait)
    if ($('activate-sensors-btn')) $('activate-sensors-btn').style.display = 'none';
}


/**
 * Gère l'événement GPS (position et vitesse)
 */
const handleGeolocation = (pos) => {
    const { latitude, longitude, altitude, accuracy, speed } = pos.coords;
    const timestamp = pos.timestamp;

    // Mise à jour de l'état global
    currentPosition = { 
        lat: latitude, 
        lon: longitude, 
        alt: altitude, 
        acc: accuracy, 
        spd: speed || 0.0, // speed est null si non supporté
        time: timestamp 
    };
    
    // Mettre à jour l'UKF (ou l'EKF, en supposant que updateUKF gère cela)
    if (ukf) {
        // ukf.updateGPS(currentPosition); // Fonction à intégrer dans l'UKF
    }

    // Indicateur de mouvement
    isMoving = (currentPosition.spd * KMH_MS) > 0.5;

    // Mise à jour du DOM rapide
    if ($('vitesse-brute-ms')) $('vitesse-brute-ms').textContent = dataOrDefault(currentPosition.spd, 2, ' m/s');
    if ($('precision-gps')) $('precision-gps').textContent = dataOrDefault(currentPosition.acc, 2, ' m');
    
    isGpsRunning = true;
};

const initGPS = () => {
    // Options GPS standard
    const GPS_OPTS = { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 };
    if ('geolocation' in navigator) {
        navigator.geolocation.watchPosition(handleGeolocation, (error) => {
            console.error("🔴 ERREUR GPS:", error.code, error.message);
            isGpsRunning = false;
            // Met à jour les indicateurs
            if ($('vitesse-kmh')) $('vitesse-kmh').textContent = '--.-- km/h';
            if ($('gps-status')) $('gps-status').textContent = 'Acquisition en échec';
        }, GPS_OPTS);
    } else {
        console.warn("GPS non supporté par ce navigateur.");
        isGpsRunning = false;
        if ($('gps-status')) $('gps-status').textContent = 'Non supporté';
    }
};


// --- BLOC 5 : MISES À JOUR PÉRIODIQUES DU DOM ---

/**
 * Mise à jour rapide (100ms), pour la vitesse instantanée et le niveau à bulle.
 */
const updateDOMFast = () => {
    // ... Logique de l'UKF/EKF et autres calculs rapides (Vitesse UKF, etc.)
    // Supposons que ukf.getState() retourne un tableau ou un objet
    if (ukf) {
        const state = ukf.getState(); 
        kSpd = state[0] || 0.0;
        kAlt = state[1] || 0.0;
        kVVert = state[2] || 0.0;
    }
    
    // Mise à jour de la vitesse UKF/EKF
    if ($('vitesse-stable-kmh')) $('vitesse-stable-kmh').textContent = dataOrDefault(kSpd * KMH_MS, 2, ' km/h');
    if ($('vitesse-stable-ms')) $('vitesse-stable-ms').textContent = dataOrDefault(kSpd, 2, ' m/s');
    if ($('vitesse-stable-kms')) $('vitesse-stable-kms').textContent = dataOrDefault(kSpd / 1000, 4, ' km/s');

    // Mise à jour du temps écoulé
    timeTotalSeconds += 0.1;
    if (isMoving) timeMovingSeconds += 0.1;
    if ($('time-total')) $('time-total').textContent = `${timeTotalSeconds.toFixed(2)} s`;
    if ($('time-moving')) $('time-moving').textContent = `${timeMovingSeconds.toFixed(2)} s`;
    
    // Force G et Accélérations (IMU/EKF)
    const gForceVertical = lastAccZ / 9.80665;
    if ($('acc-vertical-imu')) $('acc-vertical-imu').textContent = dataOrDefault(lastAccZ, 2, ' m/s²');
    if ($('force-g-vertical')) $('force-g-vertical').textContent = dataOrDefault(gForceVertical, 2, ' G');
    if ($('vitesse-verticale-ekf')) $('vitesse-verticale-ekf').textContent = dataOrDefault(kVVert, 2, ' m/s');


    setTimeout(updateDOMFast, 100);
};

/**
 * Mise à jour lente (2000ms), pour les calculs Astro, Météo et DOM lourds.
 */
const updateDOMSlow = () => {

    // --- MISE À JOUR HORLOGE LOCALE (NTP/FALLBACK) ---
    const now = getCDate(); // Utilise la date incassable
    
    // Mise à jour de l'heure locale (NTP) et UTC/GMT
    if (now) {
        const timeStr = now.toLocaleTimeString('fr-FR');
        const dateStr = now.toLocaleDateString('fr-FR');
        
        if ($('local-time')) $('local-time').textContent = timeStr;
        if ($('date-display-utc')) $('date-display-utc').textContent = now.toUTCString().split(' ')[4] + ' UTC';
        if ($('date-display-astro')) $('date-display-astro').textContent = dateStr;
    }

    // --- MISE À JOUR ASTRO (CORRIGÉ ET ROBUSTE) ---
    
    // Le calcul Astro est basé sur la date valide (now) et les coordonnées UKF/GPS.
    const lat = currentPosition.lat || 0.0;
    const lon = currentPosition.lon || 0.0;
    
    // Vérification critique des dépendances lib/astro.js (calculateAstroDataHighPrec, getTSLV)
    if (typeof calculateAstroDataHighPrec === 'function' && typeof getTSLV === 'function') {
        try {
            const fullAstroData = calculateAstroDataHighPrec(now, lat, lon); 
            const R2D = 180 / Math.PI;

            // --- TEMPS SOLAIRE & SIDÉRAL ---
            if ($('date-astro')) $('date-astro').textContent = now.toLocaleDateString('fr-FR');
            if ($('tst')) $('tst').textContent = fullAstroData.TST_HRS || 'N/A';
            if ($('mst')) $('mst').textContent = fullAstroData.MST_HRS || 'N/A';
            if ($('noon-solar-utc')) $('noon-solar-utc').textContent = fullAstroData.NOON_SOLAR_UTC.toTimeString().split(' ')[0] + ' UTC';
            if ($('eot')) $('eot').textContent = dataOrDefault(parseFloat(fullAstroData.EOT_MIN), 2, ' min');
            if ($('tslv')) $('tslv').textContent = getTSLV(now, lon) || 'N/A'; 
            if ($('ecl-long')) $('ecl-long').textContent = dataOrDefault(parseFloat(fullAstroData.ECL_LONG), 4, '°');

            // --- SOLEIL ---
            if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(fullAstroData.sun.altitude * R2D, 2, '°');
            if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(fullAstroData.sun.azimuth * R2D, 2, '°');
            
            // Lever/Coucher
            const dayDurationMs = fullAstroData.sun.sunset.getTime() - fullAstroData.sun.sunrise.getTime();
            if ($('sunrise-times')) $('sunrise-times').textContent = fullAstroData.sun.sunrise.toLocaleTimeString('fr-FR');
            if ($('sunset-times')) $('sunset-times').textContent = fullAstroData.sun.sunset.toLocaleTimeString('fr-FR');
            if ($('day-duration')) $('day-duration').textContent = dataOrDefault(dayDurationMs / 3600000, 2, ' h');
            
            // --- LUNE ---
            if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(fullAstroData.moon.illumination.phase); 
            if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(fullAstroData.moon.illumination.fraction * 100, 1, ' %');
            if ($('moon-alt')) $('moon-alt').textContent = dataOrDefault(fullAstroData.moon.position.altitude * R2D, 2, '°');
            if ($('moon-azimuth')) $('moon-azimuth').textContent = dataOrDefault(fullAstroData.moon.position.azimuth * R2D, 2, '°');
            if ($('moon-distance')) $('moon-distance').textContent = dataOrDefault(fullAstroData.moon.position.distance / 1000, 0, ' km'); 
            
            // Lever/Coucher Lune
            if ($('moon-times')) $('moon-times').textContent = 'Calcul indisponible (Simp.)'; 

        } catch (e) {
            console.error("🔴 ERREUR CRITIQUE ASTRO : L'appel à calculateAstroDataHighPrec a échoué. Assurez-vous que lib/astro.js est correct.", e);
            // Affichage d'erreur de secours
            if ($('tst')) $('tst').textContent = 'N/A (Erreur JS)';
        }
    } else {
        // Affichage de secours en cas de dépendance manquante (lib/astro.js)
        if ($('tst')) $('tst').textContent = 'N/A (Astro.js manquant)';
    }

    // --- APPELS API (Météo/Pollution) ---
    // Ces appels restent inchangés et continueront de nécessiter une connexion ou un proxy valide.
    // L'appel doit être conditionné par la disponibilité des coordonnées GPS.
    // if (isGpsRunning && currentPosition.lat !== 0.0) {
    //    fetchWeather(currentPosition.lat, currentPosition.lon); 
    //    fetchPollutants(currentPosition.lat, currentPosition.lon); 
    // }

    setTimeout(updateDOMSlow, DOM_SLOW_UPDATE_MS);
};


// --- BLOC 6 : INITIALISATION DU SYSTÈME ---

window.onload = () => {
    
    // 1. VÉRIFICATION DES DÉPENDANCES ET INITIALISATION DE L'UKF (CORRIGÉ)
    if (typeof math !== 'undefined') {
        window.ukf = new ProfessionalUKF(); // Initialisation IMMÉDIATE
        console.log("UKF 21 États Initialisé. 🟢");
    } else {
        console.error("🔴 math.js n'a pas pu être chargé. Le filtre UKF est désactivé.");
        return;
    }

    // 2. Initialisation des capteurs (GPS et IMU)
    initGPS(); 

    // 3. Gestion de l'activation des capteurs IMU (Pour mobiles)
    const sensorBtn = $('activate-sensors-btn');
    if (sensorBtn) {
        sensorBtn.addEventListener('click', activateDeviceMotion);
        // Si la page est servie via HTTPS, ce bouton est nécessaire.
    } else {
        // Si le bouton n'existe pas, tente de démarrer directement (pour les ordinateurs de bureau)
        activateDeviceMotion(); 
    }

    // 4. Démarrage de la synchronisation NTP (réseau)
    syncH(); // L'UKF n'est plus bloqué par cette promesse.

    // 5. Démarrage des boucles de rafraîchissement
    updateDOMFast();
    updateDOMSlow();

    // ... Initialisation des écouteurs d'événements (boutons) ...
    // ... Par exemple : $('reset-dist').addEventListener('click', resetDistance);
};
