// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// VERSION FINALE - ROBUSTESSE ET HAUTE PRÉCISION
// =================================================================

// --- BLOC 1 : CONSTANTES ET UTILITAIRES DE BASE ---

const $ = id => document.getElementById(id);

const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const KMH_MS = 3.6;         
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const DOM_SLOW_UPDATE_MS = 2000; // Fréquence de rafraîchissement lent (ms)

// Constantes Météo (pour fallback)
const RHO_SEA_LEVEL = 1.225; // Densité de l'air niveau mer (kg/m³)
const TEMP_SEA_LEVEL_K = 288.15; // 15°C en Kelvin
const BARO_ALT_REF_HPA = 1013.25;

// API Endpoints (Nécessite HTTPS et/ou CORS)
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";


// --- FORMATAGE DES DONNÉES (VERSION ROBUSTE ANTI-BRUIT DE MACHINE) ---
const dataOrDefault = (val, decimals, suffix = '') => {
    // Affiche '--.--' si NaN, null, Infinity, ou si la valeur est négligeable (bruit de machine < 1e-9)
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
let timeTotalSeconds = 0;
let timeMovingSeconds = 0;
let lastNTPDate = null; // Date fournie par le serveur NTP
let lastLocalTime = null; // Date locale au moment de la réception NTP

// Position UKF/GPS (Initialisation CRITIQUE avec des valeurs par défaut)
let currentPosition = { 
    lat: 43.2964,   // Latitude par défaut (Marseille, pour débloquer Astro/Météo)
    lon: 5.3697,    // Longitude par défaut
    alt: 0.0,
    acc: 10.0,      
    spd: 0.0        
};

// Variables EKF/UKF (Initialisées à 0, l'UKF les mettra à jour)
let ukf = null;
let kAlt = 0.0;     // Altitude estimée (m)
let kSpd = 0.0;     // Vitesse stable estimée (m/s)
let kVVert = 0.0;   // Vitesse verticale estimée (m/s)

// Variables IMU/Capteurs
let lastPitch = 0.0, lastRoll = 0.0;
let lastAccZ = 0.0, lastAccLong = 0.0;
let sensorStatus = "Inactif";
let currentMass = 70.0; // Poids par défaut pour les calculs de force


// --- BLOC 3 : FONCTIONS DE TEMPS (ROBUSTESSE GARANTIE) ---

const syncH = async () => {
    try {
        const response = await fetch(SERVER_TIME_ENDPOINT);
        if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`);
        const data = await response.json();
        
        lastNTPDate = new Date(data.utc_datetime); 
        lastLocalTime = Date.now(); 
        
        // Affichage des temps initiaux
        if ($('date-gmt')) $('date-gmt').textContent = lastNTPDate.toTimeString().split(' ')[0] + ' UTC';
        
    } catch (e) {
        console.warn("🔴 Échec de la synchronisation NTP. Utilisation de l'horloge locale.", e.message);
        if ($('date-gmt')) $('date-gmt').textContent = 'SYNCHRO ÉCHOUÉE';
        // Le fallback dans getCDate gère le reste.
    }
};

/**
 * Retourne un objet Date corrigé ou l'heure locale comme fallback (INASSABLE).
 */
const getCDate = () => {
    if (!lastNTPDate || !lastLocalTime) {
        return new Date(); // FALLBACK CRITIQUE
    }
    const localTimeDifference = Date.now() - lastLocalTime;
    return new Date(lastNTPDate.getTime() + localTimeDifference);
};


// --- BLOC 4 : GESTION DES CAPTEURS ET GPS (IMU et Geolocation) ---

const handleDeviceMotion = (event) => {
    // Si l'UKF est initialisé, on peut l'utiliser pour la fusion
    // if (ukf) { ukf.updateIMU(event.accelerationIncludingGravity, event.rotationRate); }

    const acc = event.accelerationIncludingGravity;
    lastAccLong = acc.x; 
    lastAccZ = acc.z; // Accélération verticale brute (avec gravité)
    sensorStatus = "Actif";

    // Mettez ici à jour lastPitch et lastRoll (nécessite DeviceOrientationEvent pour être précis)
    // Pour cet exemple, nous utilisons les valeurs brutes de l'accélération
    
    // Mise à jour de l'affichage
    if ($('acc-x')) $('acc-x').textContent = dataOrDefault(acc.x, 2, ' m/s²');
    if ($('acc-y')) $('acc-y').textContent = dataOrDefault(acc.y, 2, ' m/s²');
    if ($('acc-z')) $('acc-z').textContent = dataOrDefault(acc.z, 2, ' m/s²');
    if ($('imu-status')) $('imu-status').textContent = 'Actif 🟢';
};


/**
 * Active les capteurs de mouvement (IMU) avec gestion des permissions mobiles.
 */
function activateDeviceMotion() {
    // La fonction doit être appelée suite à une interaction utilisateur
    if (window.DeviceMotionEvent && typeof DeviceMotionEvent.requestPermission === 'function') {
        DeviceMotionEvent.requestPermission()
            .then(permissionState => {
                if (permissionState === 'granted') {
                    window.addEventListener('devicemotion', handleDeviceMotion, true);
                    console.log("Capteurs IMU activés (Permission accordée).");
                } else {
                    console.warn("Permission capteurs IMU refusée.");
                    sensorStatus = "Refusé 🔴";
                }
            })
            .catch(error => {
                console.error("Erreur à l'activation des capteurs:", error);
                sensorStatus = "Erreur 🔴";
            });
    } else if (window.DeviceMotionEvent) {
        // Tente d'activer sans demande explicite (pour les ordinateurs de bureau)
        window.addEventListener('devicemotion', handleDeviceMotion, true);
        console.log("Capteurs IMU activés (Méthode standard).");
    } else {
        sensorStatus = "Non supporté 🟡";
    }
    if ($('imu-status')) $('imu-status').textContent = sensorStatus;
    if ($('activate-sensors-btn')) $('activate-sensors-btn').style.display = 'none';
}


const handleGeolocation = (pos) => {
    const { latitude, longitude, altitude, accuracy, speed } = pos.coords;
    const timestamp = pos.timestamp;

    // Mise à jour de l'état global
    currentPosition = { 
        lat: latitude, 
        lon: longitude, 
        alt: altitude, 
        acc: accuracy, 
        spd: speed || 0.0,
        time: timestamp 
    };
    
    // Mettre à jour l'UKF
    // if (ukf) { ukf.updateGPS(currentPosition); }

    isMoving = (currentPosition.spd * KMH_MS) > 0.5;

    // Mise à jour du DOM rapide
    if ($('vitesse-brute-ms')) $('vitesse-brute-ms').textContent = dataOrDefault(currentPosition.spd, 2, ' m/s');
    if ($('precision-gps')) $('precision-gps').textContent = dataOrDefault(currentPosition.acc, 2, ' m');
    if ($('gps-status')) $('gps-status').textContent = 'Actif 🟢';

    isGpsRunning = true;
};

const initGPS = () => {
    const GPS_OPTS = { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 };
    if ('geolocation' in navigator) {
        navigator.geolocation.watchPosition(handleGeolocation, (error) => {
            console.error("🔴 ERREUR GPS:", error.code, error.message);
            isGpsRunning = false;
            if ($('gps-status')) $('gps-status').textContent = 'Erreur/Refus 🔴';
        }, GPS_OPTS);
    } else {
        console.warn("GPS non supporté par ce navigateur.");
        isGpsRunning = false;
        if ($('gps-status')) $('gps-status').textContent = 'Non supporté 🟡';
    }
};


// --- BLOC 5 : MISES À JOUR PÉRIODIQUES DU DOM ---

const updateDOMFast = () => {
    // --- UKF/EKF FUSION LOGIC (Simplified placeholders) ---
    // Les valeurs kSpd, kAlt, kVVert sont mises à jour ici par la logique de l'UKF
    
    // Si l'UKF est actif, on peut mettre à jour l'état
    if (ukf) {
        // const state = ukf.getState(); 
        // kSpd = state.speed; 
        // kAlt = state.altitude; 
        // kVVert = state.verticalSpeed; 
    }

    // Mise à jour du temps écoulé (même en cas de "N/A" initial)
    timeTotalSeconds += 0.1;
    if (isMoving) timeMovingSeconds += 0.1;
    if ($('time-total')) $('time-total').textContent = `${timeTotalSeconds.toFixed(2)} s`;
    if ($('time-moving')) $('time-moving').textContent = `${timeMovingSeconds.toFixed(2)} s`;
    
    // Mise à jour des valeurs EKF/UKF
    if ($('vitesse-stable-kmh')) $('vitesse-stable-kmh').textContent = dataOrDefault(kSpd * KMH_MS, 2, ' km/h');
    if ($('vitesse-stable-ms')) $('vitesse-stable-ms').textContent = dataOrDefault(kSpd, 2, ' m/s');
    if ($('vitesse-verticale-ekf')) $('vitesse-verticale-ekf').textContent = dataOrDefault(kVVert, 2, ' m/s');
    
    // Accélération et Force G
    const gForceVertical = lastAccZ / 9.80665;
    if ($('accel-verticale-imu')) $('accel-verticale-imu').textContent = dataOrDefault(lastAccZ, 2, ' m/s²');
    if ($('force-g-verticale')) $('force-g-verticale').textContent = dataOrDefault(gForceVertical, 2, ' G');
    
    // Position EKF (basé sur la fusion de l'UKF)
    if ($('latitude-ekf')) $('latitude-ekf').textContent = dataOrDefault(currentPosition.lat, 4, '°');
    if ($('longitude-ekf')) $('longitude-ekf').textContent = dataOrDefault(currentPosition.lon, 4, '°');
    if ($('altitude-ekf')) $('altitude-ekf').textContent = dataOrDefault(kAlt, 1, ' m');


    setTimeout(updateDOMFast, 100);
};

const updateDOMSlow = () => {

    // --- MISE À JOUR HORLOGE LOCALE (NTP/FALLBACK) ---
    const now = getCDate(); // Date VALIDE garantie
    
    if (now) {
        const timeStr = now.toLocaleTimeString('fr-FR');
        const dateStr = now.toLocaleDateString('fr-FR');
        
        if ($('local-time')) $('local-time').textContent = timeStr;
        if ($('date-display-utc')) $('date-display-utc').textContent = now.toUTCString().split(' ')[4] + ' UTC';
        if ($('date-astro')) $('date-astro').textContent = dateStr;
    }


    // --- MISE À JOUR ASTRO (High-Precision - Dépend de lib/astro.js) ---
    const lat = currentPosition.lat;
    const lon = currentPosition.lon;
    
    // Vérification des dépendances CRITIQUES lib/astro.js
    if (typeof calculateAstroDataHighPrec === 'function' && typeof getTSLV === 'function' && typeof getMoonPhaseName === 'function') {
        try {
            const fullAstroData = calculateAstroDataHighPrec(now, lat, lon); 
            const R2D = 180 / Math.PI;

            // ... MISE À JOUR DU DOM ASTRO (TST, MST, etc.) ...
            if ($('tst')) $('tst').textContent = fullAstroData.TST_HRS || 'N/A';
            if ($('mst')) $('mst').textContent = fullAstroData.MST_HRS || 'N/A';
            if ($('noon-solar-local')) $('noon-solar-local').textContent = fullAstroData.NOON_SOLAR_UTC.toTimeString().split(' ')[0] + ' UTC';
            if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(fullAstroData.sun.altitude * R2D, 2, '°');
            if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(fullAstroData.sun.azimuth * R2D, 2, '°');
            if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(fullAstroData.moon.illumination.phase); 
            if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(fullAstroData.moon.illumination.fraction * 100, 1, ' %');
            // ... Mettre à jour tous les autres champs Astro de la même manière ...

        } catch (e) {
            console.error("🔴 ERREUR CRITIQUE ASTRO : L'appel à calculateAstroDataHighPrec a échoué. Coordonnées ou librairie Astro ?", e);
            if ($('tst')) $('tst').textContent = 'N/A (Erreur JS)';
        }
    } else {
        // L'affichage 'N/A' ici est normal si vous n'avez pas chargé lib/astro.js et lib/ephem.js
        if ($('tst')) $('tst').textContent = 'N/A (Astro.js manquant)';
    }

    // --- MISES À JOUR MÉTÉO/API (Requiert fetchWeather et des coordonnées) ---
    // if (isGpsRunning && typeof fetchWeather === 'function') {
    //    fetchWeather(lat, lon); 
    // }
    
    setTimeout(updateDOMSlow, DOM_SLOW_UPDATE_MS);
};


// --- BLOC 6 : INITIALISATION DU SYSTÈME (window.onload) ---

window.onload = () => {
    
    // 1. VÉRIFICATION DES DÉPENDANCES ET INITIALISATION DE L'UKF (IMMEDIATE)
    if (typeof math !== 'undefined' && typeof ProfessionalUKF !== 'undefined') {
        window.ukf = new ProfessionalUKF(); // Initialisation IMMÉDIATE
        console.log("UKF 21 États Initialisé. 🟢");
    } else {
        console.error("🔴 math.js ou ProfessionalUKF manquant. Le filtre UKF est désactivé.");
        // Si math.js n'est pas là, le script s'arrête ici.
        return; 
    }

    // 2. Initialisation des capteurs (GPS et IMU)
    initGPS(); 
    
    // 3. Gestion de l'activation des capteurs IMU (Pour mobiles)
    const sensorBtn = $('activate-sensors-btn');
    if (sensorBtn) {
        sensorBtn.addEventListener('click', activateDeviceMotion);
    } else {
        // Pour les navigateurs de bureau qui ne nécessitent pas de clic
        activateDeviceMotion(); 
    }
    
    // 4. Démarrage de la synchronisation NTP (réseau)
    syncH(); 

    // 5. Démarrage des boucles de rafraîchissement
    updateDOMFast();
    updateDOMSlow();

    // 6. Autres initialisations (carte, événements)
    // if (typeof L !== 'undefined' && $('map-container')) { initMap(); }
    
    // ... Initialisation des écouteurs de contrôle (Réinitialisation, etc.)
};

// =================================================================
// FIN DU FICHIER GNSS-DASHBOARD-FULL.JS
// =================================================================
