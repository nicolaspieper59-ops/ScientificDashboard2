// =================================================================
// BLOC 1/4 : Utilitaires, Constantes Fondamentales & État Global
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);
const toRad = deg => deg * (Math.PI / 180);
const toDeg = rad => rad * (180 / Math.PI);

/** Formate une valeur numérique. */
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        return (decimals === 0 ? '0' : '0.00') + suffix;
    }
    return val.toFixed(decimals) + suffix;
};
/** Formate en notation exponentielle. */
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        const zeroDecimals = '0.' + Array(decimals).fill('0').join('');
        return zeroDecimals + 'e+0' + suffix;
    }
    return val.toExponential(decimals) + suffix;
};
// Inclure ici les autres utilitaires (toKmH, getCDate, etc.)

// --- Vérification des dépendances critiques (math.js, Leaflet, SunCalc, Turf.js) ---
if (typeof math === 'undefined' || typeof L === 'undefined' || typeof SunCalc === 'undefined' || typeof turf === 'undefined') {
    const missing = ["math.min.js", "leaflet.js", "suncalc.js", "turf.min.js"].filter(f => typeof eval(f.replace('.js', '').replace('.', '')) === 'undefined').join(", ");
    console.error(`Erreur critique : Dépendances manquantes : ${missing}.`);
    alert(`Erreur: Dépendances manquantes : ${missing}. L'application ne peut pas démarrer.`);
}

// --- Encapsulation de la logique UKF et État Global (IIFE) ---
((window) => {
    
    // --- CLÉS D'API & ENDPOINTS ---
    const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
    const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
    const PROXY_POLLUTANT_ENDPOINT = `${PROXY_BASE_URL}/api/pollutants`;
    const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";
    const GPS_OPTIONS = { enableHighAccuracy: true, maximumAge: 500, timeout: 15000 };

    // --- CONSTANTES PHYSIQUES FONDAMENTALES ---
    const C_L = 299792458;          // Vitesse de la lumière (m/s)
    const G_ACC_STD = 9.80665;      // Gravité standard (m/s²)
    const G_UNIVERSAL = 6.67430e-11;// Constante gravitationnelle (m³/kg/s²)
    const OMEGA_EARTH = 7.2921159e-5;// Vitesse de rotation Terre (rad/s)
    const TEMP_SEA_LEVEL_K = 288.15; // 15°C ISA (K)
    const RHO_SEA_LEVEL = 1.225;    // Densité Air (kg/m³) ISA

    // --- Variables d'État Globales ---
    let gpsWatcherID = null;    
    let domSlowID = null;       
    let domFastID = null;       
    let lastIMUTimestamp = 0;   
    let currentLat = 43.2964, currentLon = 5.3697; // Coordonnées par défaut
    let currentAlt = 0.0;
    let accel = { x: 0, y: 0, z: 0 }; 
    let gyro = { x: 0, y: 0, z: 0 };  
    let currentMass = 70.0;
    let currentCelestialBody = 'EARTH';
    let currentMap; // Objet Leaflet
    let currentUKFReactivity = 'AUTO';
    let lastKnownWeather = null;
    // PLACEHOLDER : Variables d'état du Filtre (UKF 21 états)
    let ukfState = null; 
    
    // --- PLACEHOLDERS Pour Fonctions de Calcul ---
    const getGravity = (lat, alt) => G_ACC_STD; // Placeholder pour WGS84
    const getAirDensity = (T_K, P_hPa, H_perc) => RHO_SEA_LEVEL;
    const getSpeedOfSound = (T_K) => 340.29; // Placeholder pour T_K

// =================================================================
// COUPURE ARTIFICIELLE N°1
// =================================================================

### BLOC 2/4 : Moteur de Fusion (UKF) & Handlers Capteurs

Ce bloc contient la logique essentielle pour le traitement des données brutes GPS et IMU, incluant la correction critique des permissions sur iOS.

```javascript
// =================================================================
// BLOC 2/4 : Moteur de Fusion (UKF) & Handlers Capteurs
// =================================================================

    // --- LOGIQUE EKF/UKF (PLACEHOLDERS pour le code Math.js) ---
    /** Initialisation du filtre UKF (21 états). */
    function initEKF(lat, lon, alt) {
        console.log(`UKF initialisé à Lat: ${lat.toFixed(4)}, Alt: ${alt.toFixed(1)}m. (21 états)`);
        // ukfState = math.zeros([21, 1]); // Ex: Initialisation de la matrice d'état
    }
    /** Étape de Prédiction EKF/UKF. */
    function predictEKF(dt, acc, gyro) {
        // Logique de prédiction (Propagation des états et de la covariance)
        // updateUKFDOM(ukfState); // Mise à jour du DOM EKF
    }
    /** Étape de Mise à Jour EKF/UKF (GPS/Baro/Mag). */
    function updateEKF(position, weather) {
        // Logique de mise à jour (Correction via mesures GPS et Capteurs)
    }

    // --- Handlers IMU/Device Motion ---
    function handleDeviceMotion(event) {
        // Enregistrement des données d'accélération pour l'EKF
        accel.x = event.accelerationIncludingGravity.x;
        accel.y = event.accelerationIncludingGravity.y;
        accel.z = event.accelerationIncludingGravity.z;
        if ($('accel-x')) $('accel-x').textContent = dataOrDefault(accel.x, 3, ' m/s²');
        if ($('accel-y')) $('accel-y').textContent = dataOrDefault(accel.y, 3, ' m/s²');
        if ($('accel-z')) $('accel-z').textContent = dataOrDefault(accel.z, 3, ' m/s²');
    }
    // Inclure ici 'handleDeviceOrientation' si nécessaire.

    // --- Handlers GPS ---
    function onPositionSuccess(position) {
        currentLat = position.coords.latitude;
        currentLon = position.coords.longitude;
        currentAlt = position.coords.altitude || currentAlt;
        
        if ($('gps-toggle-btn')) $('gps-toggle-btn').textContent = "🟢 ACTIF GPS";
        updateEKF(position, lastKnownWeather); 
        // Logique de mise à jour du DOM GPS/Carte ici
    }

    function onPositionError(err) {
        let errMsg = `Erreur GPS (${err.code}): ${err.message}`;
        if (err.code === 1) {
            errMsg = "🔴 Accès GPS refusé (Code 1). L'estimation EKF/IMU reste active (si autorisée).";
        }
        if ($('gps-precision')) $('gps-precision').textContent = errMsg;
    }

    /** 🔑 CRITIQUE : Fonction maîtresse pour le démarrage IMU (Gestion robuste des permissions HTTPS/iOS) */
    function startIMUDeviceMotionListeners() {
        if (domFastID) return; 
        
        const activateListeners = () => {
            if ($('imu-status')) $('imu-status').textContent = "Actif (DeviceMotion)";
            window.addEventListener('devicemotion', handleDeviceMotion);
            lastIMUTimestamp = performance.now();
            // Démarrage de la boucle rapide (EKF)
            startFastLoop(); 
        };

        // --- STRATÉGIE iOS/SAFARI : Demande de permission explicite ---
        if (typeof DeviceOrientationEvent !== 'undefined' && typeof DeviceOrientationEvent.requestPermission === 'function') {
            DeviceOrientationEvent.requestPermission()
                .then(permissionState => {
                    if (permissionState === 'granted') {
                        activateListeners();
                    } else {
                        if ($('imu-status')) $('imu-status').textContent = 'IMU ÉCHOUÉ : Permission refusée par iOS.';
                    }
                })
                .catch(e => console.error("IMU Permission Error:", e));
        } 
        // --- Stratégie Android/Chrome/Autres : Activation directe ---
        else if (window.DeviceMotionEvent) {
            activateListeners();
        } else {
            if ($('imu-status')) $('imu-status').textContent = 'IMU Non Supporté';
        }
    }


// =================================================================
// COUPURE ARTIFICIELLE N°2
// =================================================================

### BLOC 3/4 : Réseau, Astro & Contrôleurs de Boucle

Ce bloc gère les communications asynchrones (APIs, NTP), les boucles de rafraîchissement (lent/rapide) et la fonction de démarrage principale.

```javascript
// =================================================================
// BLOC 3/4 : Réseau, Astro & Contrôleurs de Boucle
// =================================================================

    // --- Fonctions Réseau & Astro ---
    /** ⌚ Synchronisation Heure NTP. */
    async function syncH() {
        if ($('local-time')) $('local-time').textContent = "SYNCHRO...";
        // PLACEHOLDER pour la logique fetch(SERVER_TIME_ENDPOINT)
    }
    
    /** ☁️ Récupération données Météo. */
    async function fetchWeather(lat, lon) {
        if ($('statut-meteo')) $('statut-meteo').textContent = "FETCH...";
        // PLACEHOLDER pour la logique fetch(PROXY_WEATHER_ENDPOINT)
    }
    
    /** 🌍 Mise à jour données Astro. */
    function updateAstro(lat, lon) {
        // PLACEHOLDER pour la logique SunCalc (Soleil/Lune)
    }
    
    /** ⚙️ Boucle de Mise à Jour Lente (1Hz) */
    function slowLoop() {
        if (domSlowID) return;
        domSlowID = setInterval(() => {
            syncH();
            fetchWeather(currentLat, currentLon);
            // Mise à jour de l'affichage du temps de session/mouvement
        }, 1000); 
    }
    
    /** ⚡ Boucle de Mise à Jour Rapide (50Hz - EKF/DOM) */
    function fastLoop() {
        if (domFastID) return;
        domFastID = setInterval(() => {
            const now = performance.now();
            const dt = (now - lastIMUTimestamp) / 1000;
            if (lastIMUTimestamp !== 0) {
                 predictEKF(dt, accel, gyro); // Prédiction UKF
            }
            lastIMUTimestamp = now;
            
            // Mise à jour DOM rapide (Vitesse, Accélération, EKF Debug)
            if ($('vitesse-kmh')) $('vitesse-kmh').textContent = dataOrDefault(Math.random() * 10, 2, ' km/h'); // Simule
        }, 20); // 50 Hz (1000ms / 20ms)
    }

    /** 🛰️ Fonction principale unifiée appelée par le bouton (Le Démarreur) */
    function startSensors() {
        if (gpsWatcherID === null) {
            if ($('gps-toggle-btn')) $('gps-toggle-btn').textContent = "🟡 Acquisition...";
            
            // 1. Lance la surveillance GPS
            gpsWatcherID = navigator.geolocation.watchPosition(
                onPositionSuccess,
                onPositionError,
                GPS_OPTIONS
            );
            
            // 2. Lance l'IMU/DeviceMotion avec la demande de permission critique
            startIMUDeviceMotionListeners(); 
            
            // 3. Démarre la carte et les boucles de mise à jour
            startMap();
            slowLoop(); 
            
            initEKF(currentLat, currentLon, currentAlt); // Initialisation du filtre
        }
    }

// =================================================================
// COUPURE ARTIFICIELLE N°3
// =================================================================

    // --- Fonctions de Contrôle ---
    function startMap() {
        if (!currentMap) {
             // PLACEHOLDER : Initialisation de la carte Leaflet
             // currentMap = L.map('mapid', { /* options */ }).setView([currentLat, currentLon], 13);
             if ($('map-status')) $('map-status').textContent = "Carte chargée.";
        }
    }

    function stopSensors(clearGPS = true) {
        if (clearGPS && gpsWatcherID !== null) {
            navigator.geolocation.clearWatch(gpsWatcherID);
            gpsWatcherID = null;
            if ($('gps-toggle-btn')) $('gps-toggle-btn').textContent = "▶️ MARCHE GPS";
        }
        if (domFastID !== null) { clearInterval(domFastID); domFastID = null; }
        if (domSlowID !== null) { clearInterval(domSlowID); domSlowID = null; }
        window.removeEventListener('devicemotion', handleDeviceMotion);
        if ($('imu-status')) $('imu-status').textContent = 'Inactif';
        // Réinitialisation des compteurs et de l'état EKF
    }
    
    // --- Initialisation du Système à la Fin du Chargement de la Page ---
    window.addEventListener('load', () => {
        
        // 1. Logique du bouton de démarrage (Point de contrôle unique)
        const gpsBtn = $('gps-toggle-btn');
        if (gpsBtn) {
            gpsBtn.addEventListener('click', () => {
                if (gpsWatcherID === null) {
                    startSensors(); // Déclenche le démarrage unifié (GPS, IMU, Boucles)
                } else {
                    stopSensors(true); 
                }
            });
        } else {
             console.error("ERREUR CRITIQUE: Bouton 'gps-toggle-btn' introuvable.");
             alert("ERREUR CRITIQUE: Le bouton DÉMARRER est manquant. Vérifiez l'ID 'gps-toggle-btn' dans index.html.");
        }

        // 2. Écouteurs d'événements pour les contrôles UI (UKF, Physique)
        $('mass-input').addEventListener('input', (e) => {
            currentMass = parseFloat(e.target.value) || 70.0;
            $('mass-display').textContent = dataOrDefault(currentMass, 3, ' kg');
        });
        $('celestial-body-select').addEventListener('change', (e) => {
            currentCelestialBody = e.target.value;
            // Logic: updateCelestialBody(currentCelestialBody, currentAlt, rotationRadius, angularVelocity);
        });
        $('ukf-reactivity-mode').addEventListener('change', (e) => currentUKFReactivity = e.target.value);
        $('reset-all-btn').addEventListener('click', () => { 
            stopSensors(true); 
            // Reset distance, max speed, EKF state... 
        });

        // 3. Initialisations de base (dès le chargement)
        syncH(); // Tentative de synchro NTP initiale
        // initEKF(currentLat, currentLon, currentAlt); // Initialisation de l'état
        updateAstro(currentLat, currentLon);
    });

})(window); // <-- Fermeture de l'IIFE

// =================================================================
// COUPURE ARTIFICIELLE N°4 (FIN DU FICHIER)
// =================================================================
