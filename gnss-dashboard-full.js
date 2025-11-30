// =================================================================
// GNSS SPACETIME DASHBOARD - VRAIE VERSION COMPLÈTE (4 BLOCS)
// BLOC 1/4 : Utilitaires, Dépendances & État Global
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES (Hors IIFE) ---
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

// --- Vérification des dépendances critiques ---
if (typeof math === 'undefined' || typeof L === 'undefined' || typeof SunCalc === 'undefined' || typeof turf === 'undefined') {
    const missing = ["math.min.js", "leaflet.js", "suncalc.js", "turf.min.js"].filter(f => typeof eval(f.replace('.js', '').replace('.', '')) === 'undefined').join(", ");
    console.error(`Erreur critique : Dépendances manquantes : ${missing}.`);
    alert(`Erreur: Dépendances manquantes : ${missing}. L'application ne peut pas démarrer.`);
}

// --- Encapsulation de la logique UKF et État Global (IIFE) ---
((window) => {
    
    // --- CONSTANTES DE L'APPLICATION ---
    const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
    const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";
    const GPS_OPTIONS = { enableHighAccuracy: true, maximumAge: 500, timeout: 15000 };
    // PLACEZ ICI VOS CONSTANTES PHYSIQUES (WGS84, ISA, EKF_TUNING, etc.)
    const G_ACC = 9.80665;
    const TEMP_SEA_LEVEL_K = 288.15; // 15°C

    // --- Variables d'État Globales ---
    let gpsWatcherID = null;    // ID du GPS watch (null = arrêté)
    let domSlowID = null;       // ID de l'intervalle lent
    let domFastID = null;       // ID de l'intervalle rapide (EKF)
    let lastIMUTimestamp = 0;   // Timestamp de la dernière mesure IMU
    let currentLat = 43.2964, currentLon = 5.3697; // Coordonnées par défaut
    let accel = { x: 0, y: 0, z: 0 }; // Données Accélération IMU
    let gyro = { x: 0, y: 0, z: 0 };  // Données Gyroscope IMU
    // PLACEZ ICI VOTRE ÉTAT EKF/UKF (EKF_STATE, COVARIANCE_MATRIX, lastKnownPos, etc.)
    // ...

// =================================================================
// COUPURE ARTIFICIELLE N°1
// =================================================================

### BLOC 2/4 : Moteur de Fusion & Handlers Capteurs (IMU/GPS)

Ce bloc contient les fonctions fondamentales de capture de données et les placeholders pour le moteur de filtrage.

```javascript
// =================================================================
// BLOC 2/4 : Moteur de Fusion & Handlers Capteurs
// =================================================================

    // --- Logique EKF/UKF ---
    // PLACEZ ICI VOTRE CODE COMPLET pour l'initialisation EKF/UKF.
    function initEKF(lat, lon, alt) { /* ... */ }
    function predictEKF(dt, acc, gyro) { /* ... */ }
    function updateEKF(position) { /* ... */ }

    // --- Handlers IMU/Device Motion ---
    function handleDeviceMotion(event) {
        // Enregistrement des données d'accélération pour l'EKF
        accel.x = event.accelerationIncludingGravity.x;
        accel.y = event.accelerationIncludingGravity.y;
        accel.z = event.accelerationIncludingGravity.z;
        if ($('imu-status')) $('imu-status').textContent = `Actif (A:${accel.x.toFixed(2)}, G:N/A)`;
    }
    // Inclure ici 'handleDeviceOrientation' si vous l'utilisez pour le cap.

    // --- Handlers GPS ---
    function onPositionSuccess(position) {
        currentLat = position.coords.latitude;
        currentLon = position.coords.longitude;
        if ($('gps-toggle-btn')) $('gps-toggle-btn').textContent = "🟢 ACTIF GPS";
        updateEKF(position); // Envoi direct des données au filtre
        // Mise à jour de la carte, du DOM GPS, etc.
    }

    function onPositionError(err) {
        let errMsg = `Erreur GPS (${err.code}): ${err.message}`;
        if (err.code === 1) {
            errMsg = "🔴 Accès GPS refusé (Code 1). L'estimation EKF/IMU reste active (si autorisée).";
        }
        if ($('gps-precision')) $('gps-precision').textContent = errMsg;
    }

    /** 🔑 Fonction maîtresse pour le démarrage IMU (Gestion robuste des permissions HTTPS/iOS) */
    function startIMUDeviceMotionListeners() {
        if (domFastID) return; // Déjà actif
        
        const activateListeners = () => {
            if ($('imu-status')) $('imu-status').textContent = "Actif (DeviceMotion Fallback)";
            // Attache les écouteurs d'événements de capteur
            window.addEventListener('devicemotion', handleDeviceMotion);
            lastIMUTimestamp = performance.now();
            // Démarrage de la boucle rapide (EKF)
            startFastLoop(); 
        };

        // --- STRATÉGIE iOS/SAFARI : Demande de permission explicite (CRITIQUE) ---
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

Ce bloc gère les communications externes (Météo, NTP) et définit le point d'entrée unique du système.

```javascript
// =================================================================
// BLOC 3/4 : Réseau, Astro & Contrôleurs de Boucle
// =================================================================

    // --- Fonctions Réseau (NTP, Météo, Pollution) ---
    function syncH() { /* PLACEZ ICI VOTRE CODE NTP/WorldTimeAPI */ }
    function fetchWeather(lat, lon) { /* PLACEZ ICI VOTRE CODE FETCH MÉTÉO */ }
    // Inclure ici fetchPollutants()

    // --- Fonctions Astro ---
    function updateAstro(lat, lon) { /* PLACEZ ICI VOTRE CODE SUNCALC */ }
    
    /** ⚙️ Boucle de Mise à Jour Lente (1Hz) */
    function slowLoop() {
        if (domSlowID) return;
        domSlowID = setInterval(() => {
            syncH();
            fetchWeather(currentLat, currentLon);
            updateAstro(currentLat, currentLon);
            // Mise à jour DOM lent (heure, statut, etc.)
        }, 1000); 
    }
    
    /** ⚡ Boucle de Mise à Jour Rapide (50Hz - EKF/DOM) */
    function fastLoop() {
        if (domFastID) return;
        domFastID = setInterval(() => {
            const now = performance.now();
            let dt = (now - lastIMUTimestamp) / 1000;
            if (dt > 0.1) dt = 0.02; // Limite le saut temporel
            
            predictEKF(dt, accel, gyro); // Prédiction EKF/UKF
            lastIMUTimestamp = now;
            
            // Mise à jour DOM rapide (vitesse estimée, altitude, accélération)
        }, 20); // 50 Hz
    }

    /** 🛰️ Fonction principale unifiée appelée par le bouton (Le Démarreur) */
    function startSensors() {
        if (gpsWatcherID === null) {
            if ($('gps-toggle-btn')) $('gps-toggle-btn').textContent = "🟡 Acquisition...";
            
            // 1. Lance la surveillance GPS (demande de permission 1)
            gpsWatcherID = navigator.geolocation.watchPosition(
                onPositionSuccess,
                onPositionError,
                GPS_OPTIONS
            );
            
            // 2. Lance l'IMU/DeviceMotion (demande de permission 2)
            startIMUDeviceMotionListeners(); 
            
            // 3. Démarre la carte et les boucles de mise à jour
            startMap();
            slowLoop(); 
            // fastLoop() est démarrée dans startIMUDeviceMotionListeners()
            
            initEKF(currentLat, currentLon, 0); // Initialisation du filtre
        }
    }

// =================================================================
// COUPURE ARTIFICIELLE N°3
// =================================================================



    // --- Fonctions de Contrôle ---
    function startMap() {
        // PLACEZ ICI VOTRE LOGIQUE D'INITIALISATION DE LA CARTE LEAFLET
    }

    function stopSensors(clearGPS = true) {
        if (clearGPS && gpsWatcherID !== null) {
            navigator.geolocation.clearWatch(gpsWatcherID);
            gpsWatcherID = null;
            if ($('gps-toggle-btn')) $('gps-toggle-btn').textContent = "▶️ DÉMARRER LE SYSTÈME";
        }
        if (domFastID !== null) {
            clearInterval(domFastID);
            domFastID = null;
        }
        if (domSlowID !== null) {
            clearInterval(domSlowID);
            domSlowID = null;
        }
        window.removeEventListener('devicemotion', handleDeviceMotion);
        // PLACEZ ICI VOTRE LOGIQUE DE RÉINITIALISATION D'ÉTAT EKF
    }
    
    // Inclure ici les autres fonctions de contrôle (handleToggleMode, updateRotation, etc.)

    // --- Initialisation du Système à la Fin du Chargement de la Page ---
    window.addEventListener('load', () => {
        // 1. Attachement de l'événement de clic au bouton (Le seul geste utilisateur requis)
        const gpsBtn = $('gps-toggle-btn');
        if (gpsBtn) {
            gpsBtn.addEventListener('click', () => {
                if (gpsWatcherID === null) {
                    startSensors(); // Déclenche le démarrage unifié
                } else {
                    stopSensors(true); // Logique d'arrêt
                }
            });
        } else {
             console.error("ERREUR CRITIQUE: Bouton 'gps-toggle-btn' introuvable.");
             alert("ERREUR CRITIQUE: Le tableau de bord ne peut pas démarrer. Le bouton GPS est manquant.");
        }

        // 2. Initialisations de base (non dépendantes du clic)
        syncH(); // Tentative de synchro NTP initiale
        // initEKF(currentLat, currentLon, 0); // Peut être initialisé ici ou dans startSensors()

        // PLACEZ ICI VOS AUTRES INITIALISATIONS (Ex: initContrôles UI)
    });

})(window); // <-- Fermeture de l'IIFE

// =================================================================
// COUPURE ARTIFICIELLE N°4
// =================================================================
