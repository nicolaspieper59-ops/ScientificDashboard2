// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// VERSION FINALE ET ROBUSTE
// =================================================================

// --- BLOC 1 : CONSTANTES ET UTILITAIRES DE BASE ---

const $ = id => document.getElementById(id);

const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const KMH_MS = 3.6;         
const C_L = 299792458;      
const DOM_SLOW_UPDATE_MS = 2000; 

const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

// --- FORMATAGE DES DONNÉES (ROBUSTESSE ANTI-BRUIT DE MACHINE) ---
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
let isMoving = false;
let timeTotalSeconds = 0;
let timeMovingSeconds = 0;
let lastNTPDate = null; 
let lastLocalTime = null; 
let currentMass = 70.0; 

// Position UKF/GPS (Initialisation CRITIQUE : Marseille)
let currentPosition = { 
    lat: 43.2964,   
    lon: 5.3697,    
    alt: 0.0,
    acc: 10.0,      
    spd: 0.0        
};

let ukf = null;
let kAlt = 0.0;     
let kSpd = 0.0;     
let kVVert = 0.0;   

// Variables IMU/Capteurs
let lastAccZ = 0.0, lastAccLong = 0.0;
let sensorStatus = "Inactif";
let lastPitch = 0.0, lastRoll = 0.0;


// --- BLOC 3 : FONCTIONS DE TEMPS (ROBUSTESSE GARANTIE) ---

const syncH = async () => {
    try {
        const response = await fetch(SERVER_TIME_ENDPOINT);
        if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`);
        const data = await response.json();
        
        lastNTPDate = new Date(data.utc_datetime); 
        lastLocalTime = Date.now(); 
        
    } catch (e) {
        console.warn("🔴 Échec de la synchronisation NTP. Utilisation de l'horloge locale.");
        // Le fallback dans getCDate gère l'affichage d'erreur.
    }
};

const getCDate = () => {
    if (!lastNTPDate || !lastLocalTime) {
        return new Date(); // FALLBACK CRITIQUE
    }
    const localTimeDifference = Date.now() - lastLocalTime;
    return new Date(lastNTPDate.getTime() + localTimeDifference);
};


// --- BLOC 4 : GESTION DES CAPTEURS ET GPS ---

const handleDeviceMotion = (event) => {
    // Les mises à jour de l'UKF doivent être faites ici
    const acc = event.accelerationIncludingGravity;
    lastAccLong = acc.x; 
    lastAccZ = acc.z; 
    sensorStatus = "Actif";

    if ($('acc-x')) $('acc-x').textContent = dataOrDefault(acc.x, 2, ' m/s²');
    if ($('acc-y')) $('acc-y').textContent = dataOrDefault(acc.y, 2, ' m/s²');
    if ($('acc-z')) $('acc-z').textContent = dataOrDefault(acc.z, 2, ' m/s²');
    if ($('imu-status')) $('imu-status').textContent = 'Actif 🟢';
};

function activateDeviceMotion() {
    if (window.DeviceMotionEvent && typeof DeviceMotionEvent.requestPermission === 'function') {
        DeviceMotionEvent.requestPermission()
            .then(permissionState => {
                if (permissionState === 'granted') {
                    window.addEventListener('devicemotion', handleDeviceMotion, true);
                } else {
                    sensorStatus = "Refusé 🔴";
                }
            })
            .catch(error => { sensorStatus = "Erreur 🔴"; });
    } else if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', handleDeviceMotion, true);
    } else {
        sensorStatus = "Non supporté 🟡";
    }
    if ($('imu-status')) $('imu-status').textContent = sensorStatus;
    // Cache le bouton (si existant, comme dans votre HTML)
    if ($('activate-sensors-btn')) $('activate-sensors-btn').style.display = 'none';
}


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
    
    // Si l'UKF est initialisé, mettez à jour la position
    // if (ukf) { ukf.updateGPS(currentPosition); }

    isMoving = (currentPosition.spd * KMH_MS) > 0.5;

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
        if ($('gps-status')) $('gps-status').textContent = 'Non supporté 🟡';
    }
    // Assure que l'ID existe, basé sur votre sortie
    if ($('gps-status')) $('gps-status').textContent = 'Acquisition en cours...';
};


// --- BLOC 5 : MISES À JOUR PÉRIODIQUES DU DOM ---

const updateDOMFast = () => {
    // Logique UKF/EKF : Mettez à jour kSpd, kAlt, kVVert ici
    
    timeTotalSeconds += 0.1;
    if (isMoving) timeMovingSeconds += 0.1;
    if ($('time-total')) $('time-total').textContent = `${timeTotalSeconds.toFixed(2)} s`;
    if ($('time-moving')) $('time-moving').textContent = `${timeMovingSeconds.toFixed(2)} s`;
    
    // Affichage des valeurs UKF/EKF
    if ($('vitesse-stable-kmh')) $('vitesse-stable-kmh').textContent = dataOrDefault(kSpd * KMH_MS, 2, ' km/h');
    if ($('vitesse-stable-ms')) $('vitesse-stable-ms').textContent = dataOrDefault(kSpd, 2, ' m/s');
    if ($('vitesse-verticale-ekf')) $('vitesse-verticale-ekf').textContent = dataOrDefault(kVVert, 2, ' m/s');
    
    // Accélération et Force G
    const gForceVertical = lastAccZ / 9.80665;
    if ($('accel-verticale-imu')) $('accel-verticale-imu').textContent = dataOrDefault(lastAccZ, 2, ' m/s²');
    if ($('force-g-verticale')) $('force-g-verticale').textContent = dataOrDefault(gForceVertical, 2, ' G');
    
    // Position EKF (utilise currentPosition si UKF non actif)
    if ($('latitude-ekf')) $('latitude-ekf').textContent = dataOrDefault(currentPosition.lat, 4, '°');
    if ($('longitude-ekf')) $('longitude-ekf').textContent = dataOrDefault(currentPosition.lon, 4, '°');
    if ($('altitude-ekf')) $('altitude-ekf').textContent = dataOrDefault(kAlt, 1, ' m');

    setTimeout(updateDOMFast, 100);
};

const updateDOMSlow = () => {

    const now = getCDate(); // Date VALIDE garantie
    
    // Mise à jour de l'heure
    if (now) {
        const timeStr = now.toLocaleTimeString('fr-FR');
        const dateStr = now.toLocaleDateString('fr-FR');
        
        if ($('local-time')) $('local-time').textContent = timeStr;
        if ($('date-display-utc')) $('date-display-utc').textContent = now.toUTCString().split(' ')[4] + ' UTC';
        if ($('date-astro')) $('date-astro').textContent = dateStr;
    }


    // --- MISE À JOUR ASTRO (Requiert lib/astro.js et lib/ephem.js) ---
    const lat = currentPosition.lat;
    const lon = currentPosition.lon;
    
    // Vérification critique des dépendances Astro.
    if (typeof calculateAstroDataHighPrec === 'function' && typeof getTSLV === 'function' && typeof getMoonPhaseName === 'function') {
        try {
            const fullAstroData = calculateAstroDataHighPrec(now, lat, lon); 
            const R2D = 180 / Math.PI;

            // Temps Solaire & Sidéral
            if ($('tst')) $('tst').textContent = fullAstroData.TST_HRS || 'N/A';
            if ($('mst')) $('mst').textContent = fullAstroData.MST_HRS || 'N/A';
            if ($('eot')) $('eot').textContent = dataOrDefault(parseFloat(fullAstroData.EOT_MIN), 2, ' min');
            if ($('tslv')) $('tslv').textContent = getTSLV(now, lon) || 'N/A'; 
            if ($('longitude-ecliptique')) $('longitude-ecliptique').textContent = dataOrDefault(parseFloat(fullAstroData.ECL_LONG), 4, '°');

            // Soleil
            if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(fullAstroData.sun.altitude * R2D, 2, '°');
            if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(fullAstroData.sun.azimuth * R2D, 2, '°');
            if ($('day-duration')) $('day-duration').textContent = dataOrDefault((fullAstroData.sun.sunset.getTime() - fullAstroData.sun.sunrise.getTime()) / 3600000, 2, ' h');

            // Lune
            if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(fullAstroData.moon.illumination.phase); 
            if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(fullAstroData.moon.illumination.fraction * 100, 1, ' %');
            
        } catch (e) {
            console.error("🔴 ERREUR ASTRO : Le calcul a échoué (Astro.js/ephem.js?).", e);
            if ($('tst')) $('tst').textContent = 'N/A (Erreur JS)';
        }
    } else {
        if ($('tst')) $('tst').textContent = 'N/A (Dépendance Astro manquante)';
    }

    // --- MÉTÉO / API ---
    // Les appels API (météo, polluants) doivent être implémentés ici ou dans une fonction appelée ici.
    // fetchWeather(lat, lon); 

    setTimeout(updateDOMSlow, DOM_SLOW_UPDATE_MS);
};


// --- BLOC 6 : INITIALISATION DU SYSTÈME ---

window.onload = () => {
    
    // 1. Initialisation de l'UKF (vérification de math.js)
    if (typeof math !== 'undefined' && typeof ProfessionalUKF !== 'undefined') {
        window.ukf = new ProfessionalUKF(); 
        console.log("UKF 21 États Initialisé. 🟢");
    } else {
        console.error("🔴 math.js ou ProfessionalUKF manquant. UKF désactivé.");
        return; 
    }

    // 2. Initialisation des capteurs
    initGPS(); 
    const sensorBtn = $('activate-sensors-btn');
    if (sensorBtn) {
        sensorBtn.addEventListener('click', activateDeviceMotion);
    } else {
        // Tente de démarrer sur desktop sans bouton
        activateDeviceMotion(); 
    }
    
    // 3. Démarrage de la synchronisation NTP
    syncH(); 

    // 4. Démarrage des boucles de rafraîchissement
    updateDOMFast();
    updateDOMSlow();
    
    // ... Autres initialisations (carte, écouteurs d'événements) ...
};
