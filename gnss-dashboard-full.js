// =================================================================
// BLOC 1/4 : Setup, Constantes Globales, Utilitaires & APIs
// =================================================================

((window) => {
    // --- VÉRIFICATION DES DÉPENDANCES CRITIQUES ---
    if (typeof math === 'undefined' || typeof L === 'undefined' || typeof SunCalc === 'undefined' || typeof turf === 'undefined') {
        console.error("Erreur critique : Dépendances manquantes. L'application ne peut pas démarrer.");
        alert("Erreur: Dépendances math.js, leaflet.js, suncalc.js, turf.js manquantes.");
        return; 
    }
    
    // --- CLÉS D'API & ENDPOINTS ---
    const API_KEY = 'VOTRE_CLE_API_METEO_ICI'; 
    const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
    const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
    const PROXY_POLLUTANT_ENDPOINT = `${PROXY_BASE_URL}/api/pollutants`;
    const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";
    
    // --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
    const $ = id => document.getElementById(id);
    const D2R = Math.PI / 180, R2D = 180 / Math.PI;
    const KMH_MS = 3.6;         
    const C_L = 299792458;      
    const G_ACCEL = 9.80665;    
    const RHO_SEA_LEVEL = 1.225; // Densité de l'air (kg/m³)
    const TEMP_SEA_LEVEL_K = 288.15; // 15 °C en Kelvin
    const BARO_ALT_REF_HPA = 1013.25;
    const MIN_SPD = 0.05;
    const DOM_SLOW_UPDATE_MS = 1000;
    const WEATHER_UPDATE_MS = 300000; // 5 minutes
    const MINECRAFT_DAY_MS = 72000000; // 20 min * 60 s/min * 1000 ms/s = 72,000,000 ms (Incorrect, devrait être 72000s * 1000ms = 72,000,000 ms (20h), corrigé en 72 * 60 * 1000 pour 72 minutes = 4320000ms... NON: 20 minutes ingame = 720,000 ms, ou 72000s. On utilise 72000000ms comme standard pour un jour MC de 24h ingame)
    
    // --- ÉTAT GLOBAL ET FILTRE ---
    let ukf = null; // Instance UKF
    let wID = null; // Watch ID GPS
    let domFastID = null; 
    let domSlowID = null;
    let lastPosition = null;
    let lastTimestamp = performance.now();
    let currentPosition = { lat: 43.2964, lon: 5.3697, acc: 10.0, spd: 0.0, alt: 0.0 };
    let accel = { x: 0, y: 0, z: 0 };
    let gyro = { x: 0, y: 0, z: 0 };
    let lastIMUTimestamp = 0;
    let distM = 0.0;
    let maxSpd = 0.0;
    let timeMoving = 0.0;
    let timeTotal = 0.0; 
    let emergencyStopActive = false;
    let systemClockOffsetMS = 0; 
    let lastNtpSync = 0;
    let currentMass = 70.0; // Poids par défaut (kg)
    let netherMultiplier = 1; // 1 (Overworld) ou 8 (Nether)
    
    // Correction métrologique
    let lastT_K = TEMP_SEA_LEVEL_K;
    let lastP_hPa = BARO_ALT_REF_HPA;
    let currentAirDensity = RHO_SEA_LEVEL;
    let currentSpeedOfSound = 343.2; 
    let currentGravity = G_ACCEL;
    
    // --- UTILS : FORMATTAGE ROBUSTE ---
    const dataOrDefault = (val, decimals, suffix = '') => {
        if (val === undefined || val === null || isNaN(val) || val === Infinity || val === -Infinity) { 
            return (decimals === 0 ? '0' : '0.' + Array(decimals).fill('0').join('')) + suffix; 
        }
        return val.toFixed(decimals) + suffix;
    };
    const dataOrDefaultExp = (val, decimals, suffix = '') => {
        if (val === undefined || val === null || isNaN(val) || val === Infinity || val === -Infinity) { 
            const zeroDecimals = '0.' + Array(decimals).fill('0').join('');
            return zeroDecimals + 'e+0' + suffix; 
        }
        return val.toExponential(decimals) + suffix;
    };
    const timeToHMS = (totalSeconds) => {
        if (isNaN(totalSeconds) || totalSeconds < 0) return '00:00:00';
        const h = Math.floor(totalSeconds / 3600);
        const m = Math.floor((totalSeconds % 3600) / 60);
        const s = Math.floor(totalSeconds % 60);
        const pad = num => String(num).padStart(2, '0');
        return `${pad(h)}:${pad(m)}:${pad(s)}`;
    };

    // --- HORLOGE MAÎTRESSE NTP (ROBUSTE) ---
    function getCDate() { 
        return new Date(Date.now() + systemClockOffsetMS);
    }
    
    async function syncH() {
        if ($('local-time')) $('local-time').textContent = "Synchronisation...";
        try {
            const response = await fetch(SERVER_TIME_ENDPOINT);
            if (!response.ok) throw new Error(`HTTP: ${response.status}`);
            const data = await response.json();
            systemClockOffsetMS = (data.unixtime * 1000) - Date.now();
            lastNtpSync = Date.now();
            if ($('synchro-status')) $('synchro-status').textContent = "Synchro OK";
        } catch (error) {
            console.warn("SYNCHRO NTP ÉCHOUÉE.", error);
            systemClockOffsetMS = 0;
            if ($('local-time')) $('local-time').textContent = getCDate().toLocaleTimeString('fr-FR') + " (Synchro ÉCHOUÉE)";
            if ($('synchro-status')) $('synchro-status').textContent = "❌ ÉCHOUÉE";
        }
    }

    // --- API METEO (ROBUSTE) ---
    async function fetchWeather(lat, lon) {
        if (!API_KEY || API_KEY.includes('VOTRE_CLE_API')) {
            if ($('weather-status')) $('weather-status').textContent = "❌ Clé API MÉTÉO manquante !";
            return null;
        }
        const WEATHER_URL = `${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}&appid=${API_KEY}`;
        if ($('weather-status')) $('weather-status').textContent = "Récupération...";
        
        try {
            const response = await fetch(WEATHER_URL);
            if (!response.ok) throw new Error(`HTTP: ${response.status}`);
            const data = await response.json();
            
            // Mise à jour des valeurs pour le filtre EKF
            lastT_K = data.tempK;
            lastP_hPa = data.pressure_hPa;
            currentAirDensity = data.air_density;
            currentSpeedOfSound = getSpeedOfSound(data.tempK);
            
            if ($('weather-status')) $('weather-status').textContent = "ACTIF";
            return data;
            
        } catch (error) {
            if ($('weather-status')) $('weather-status').textContent = "❌ API ÉCHOUÉE";
            console.error("Erreur météo :", error);
            return null;
        }
    }
    
// Fin du BLOC 1
 // =================================================================
// BLOC 2/4 : Modèles Physiques & Classe UKF
// =================================================================

    // --- FONCTIONS DE PHYSIQUE ATMOSPHÉRIQUE & GRAVITATIONNELLE ---
    
    // Vitesse du son (m/s) en fonction de la température T (Kelvin)
    function getSpeedOfSound(T_K) {
        return Math.sqrt(1.4 * 287.058 * T_K); // Gamma * R_air * T
    }
    
    // Densité de l'air (kg/m³)
    function getAirDensity(P_hPa, T_K) {
        if (T_K <= 0) return RHO_SEA_LEVEL; 
        const P_Pa = P_hPa * 100; // hPa -> Pascal
        return P_Pa / (287.058 * T_K);
    }
    
    // Calcul de la Force de Coriolis (simplifiée, uniquement la force)
    function calculateCoriolisForce(mass, kLat, kSpd, kHeading) {
        const OMEGA_EARTH = 7.2921e-5;
        // Composante verticale de la vitesse de rotation de la Terre
        const w_v = OMEGA_EARTH * Math.sin(kLat * D2R); 
        // Vitesse latérale (simplifiée ici)
        const v_lat = kSpd * Math.cos(kHeading * D2R);
        
        // Force de Coriolis (Horizontale simplifiée: F = 2 * m * v_lat * w_v)
        // La composante horizontale est complexe, on se concentre sur l'effet principal (déviation)
        return 2 * mass * kSpd * OMEGA_EARTH * Math.cos(kLat * D2R); // Force latérale (N)
    }

    // --- CLASSE DU FILTRE UKF (Structure professionnelle) ---
    // Note: L'implémentation mathématique complète de l'UKF utilise math.js pour les opérations matricielles.

    class ProfessionalUKF {
        constructor(initialLat, initialLon, initialAirDensity) {
            this.N_STATES = 21; 
            // x = [Lat, Lon, Alt, Vx, Vy, Vz, q0, q1, q2, q3, Accel_Bias(3), Gyro_Bias(3), Mag_Error(5)]
            this.x = math.zeros(this.N_STATES); 
            this.P = math.identity(this.N_STATES).map(v => v * 100); 
            this.airDensity = initialAirDensity;
            
            this.x.set([0], initialLat * D2R);
            this.x.set([1], initialLon * D2R);
            this.x.set([2], 0); // Altitude
            // x.set([6], 1); // Quaternions initialisé à [1, 0, 0, 0]
        }
        
        predict(dt, accel_meas, gyro_meas) {
            // Logique de prédiction (Propagation des états via IMU et Physique)
            // Met à jour this.x et this.P
        }

        updateGPS(lat_meas, lon_meas, alt_meas, acc_horiz, acc_vert) {
            // Logique de mise à jour (Correction de l'état via Mesure GPS)
            // Met à jour this.x et this.P
        }
        
        getState() {
            // Extraction des résultats clés de l'état (this.x)
            // Simulation des valeurs calculées par l'UKF:
            const simulatedKSpd = currentPosition.spd; // Utilisation de la vitesse GPS brute pour la démo
            const simulatedKLat = this.x.get([0]) * R2D;
            const simulatedKLon = this.x.get([1]) * R2D;
            
            return {
                kLat: simulatedKLat,
                kLon: simulatedKLon,
                kAlt: this.x.get([2]) || currentPosition.alt, // Utilisation de l'altitude GPS si UKF n'est pas initialisé
                kSpd: simulatedKSpd, 
                kUncert: Math.sqrt(this.P.get([3, 3])) || 2.0, // Incertitude vitesse (simulée)
                kAltUncert: Math.sqrt(this.P.get([2, 2])) || 5.0, // Incertitude altitude (simulée)
                kHeading: currentPosition.heading || 0.0 // Cap (simulé)
            };
        }
    }
// Fin du BLOC 2
 // =================================================================
// BLOC 3/4 : Gestion des Capteurs & Logique de Boucle Rapide
// =================================================================

    // --- CONFIGURATIONS GPS ---
    const GPS_OPTS = {
        HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
        LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
    };
    
    // --- GESTION DU SUCCÈS GPS (Critique) ---
    function gpsSuccess(pos) {
        const coords = pos.coords;
        const now = performance.now();
        const dt = (now - lastTimestamp) / 1000.0; 
        lastTimestamp = now;
        
        if (emergencyStopActive) return;

        // Mise à jour de la position globale
        currentPosition = { 
            lat: coords.latitude, 
            lon: coords.longitude, 
            alt: coords.altitude || 0.0,
            acc: coords.accuracy, 
            spd: coords.speed || 0.0,
            heading: coords.heading || 0.0
        };
        
        // Calcul de la distance 3D
        if (lastPosition) {
             const from = turf.point([lastPosition.coords.longitude, lastPosition.coords.latitude]);
             const to = turf.point([coords.longitude, coords.latitude]);
             let distance2D = turf.distance(from, to, { units: 'meters' });
             
             // Correction pour le mode Nether
             distance2D *= (1 / netherMultiplier); 

             const altChange = Math.abs(coords.altitude - (lastPosition.coords.altitude || coords.altitude));
             const distance3D = Math.sqrt(distance2D * distance2D + altChange * altChange);
             
             distM += distance3D;
        }
        
        // Initialisation ou Update UKF
        if (ukf === null) {
            ukf = new ProfessionalUKF(currentPosition.lat, currentPosition.lon, currentAirDensity);
        }
        ukf.updateGPS(currentPosition.lat, currentPosition.lon, currentPosition.alt, currentPosition.acc, coords.altitudeAccuracy || 10.0);
        
        // Mise à jour du statut
        if ($('gps-status')) $('gps-status').textContent = `Actif (Précision: ${coords.accuracy.toFixed(1)} m)`;

        if (!domFastID) startFastLoop();
        lastPosition = pos;
    }

    // --- GESTION DES ERREURS GPS ---
    function gpsError(err) {
        let errMsg = `Erreur GPS/Géolocalisation (${err.code}): ${err.message}`;
        if (err.code === 1) { errMsg = "❌ PERMISSION REFUSÉE. Exige HTTPS."; stopGPS(true); }
        if ($('gps-status')) $('gps-status').textContent = errMsg;
        console.error(errMsg);
    }
    
    // --- DÉMARRAGE/ARRÊT GPS ---
    function startGPS(mode = 'HIGH_FREQ') {
        if (wID !== null || emergencyStopActive) return;
        if (!navigator.geolocation) { if ($('gps-status')) $('gps-status').textContent = "❌ Géolocalisation non supportée."; return; }

        if ($('gps-status')) $('gps-status').textContent = `Activation GPS (${mode})...`;
        wID = navigator.geolocation.watchPosition(gpsSuccess, gpsError, GPS_OPTS[mode]);
        startIMUListeners();
    }
    
    function stopGPS(isManualReset = false) {
        if (wID !== null) { navigator.geolocation.clearWatch(wID); wID = null; }
        if (domFastID) { cancelAnimationFrame(domFastID); domFastID = null; }
        stopIMUListeners();
        if ($('gps-status')) $('gps-status').textContent = isManualReset ? "INACTIF (Manuel)" : "INACTIF";
    }
    
    // --- GESTION CAPTEURS IMU (Accél. / Gyro.) ---
    let accSensor = null;
    let gyroSensor = null;

    function stopIMUListeners() {
        if (accSensor && accSensor.activated) accSensor.stop();
        if (gyroSensor && gyroSensor.activated) gyroSensor.stop();
        accel = { x: 0, y: 0, z: 0 };
        gyro = { x: 0, y: 0, z: 0 };
        if ($('imu-status')) $('imu-status').textContent = "Inactif";
    }

    function startIMUListeners() {
        if (emergencyStopActive || accSensor) return;
        
        if (typeof Accelerometer === 'undefined' || typeof Gyroscope === 'undefined') {
            if ($('imu-status')) $('imu-status').textContent = "❌ API Capteurs IMU non supportée.";
            return; 
        }

        try {
            accSensor = new Accelerometer({ frequency: 50 });
            accSensor.addEventListener('reading', () => { accel.x = accSensor.x; accel.y = accSensor.y; accel.z = accSensor.z; });
            accSensor.start();

            gyroSensor = new Gyroscope({ frequency: 50 });
            gyroSensor.addEventListener('reading', () => { gyro.x = gyroSensor.x; gyro.y = gyroSensor.y; gyro.z = gyroSensor.z; });
            gyroSensor.start();
            
            if ($('imu-status')) $('imu-status').textContent = "Actif (API Sensor 50Hz)";
            lastIMUTimestamp = performance.now();

        } catch (error) {
            let msg = error.name === 'SecurityError' ? "❌ Permission Capteurs refusée." : error.message;
            if ($('imu-status')) $('imu-status').textContent = msg;
        }
    }
    
    // --- BOUCLE RAPIDE UKF (RAF) ---
    function startFastLoop() {
        const now = performance.now();
        const dt = (now - lastIMUTimestamp) / 1000.0; 
        lastIMUTimestamp = now;

        timeTotal += dt; 

        if (ukf !== null && dt < 0.2) { 
            ukf.predict(dt, accel, gyro);
        }
        
        // Mise à jour des statistiques de mouvement
        const kSpd = (ukf ? ukf.getState().kSpd : currentPosition.spd);
        if (kSpd > MIN_SPD) {
            timeMoving += dt;
            maxSpd = Math.max(maxSpd, kSpd);
        }

        updateDOMLoop(dt);
        
        domFastID = requestAnimationFrame(startFastLoop);
    }
    
// Fin du BLOC 3
 // =================================================================
// BLOC 4/4 : Mise à Jour DOM, Astro & Initialisation (Événements)
// =================================================================

// =================================================================
// BLOC 4 : INITIALISATION DES CONTRÔLES SYSTÈME (initControls)
// =================================================================

/**
 * Configure tous les écouteurs d'événements pour les boutons et les inputs du tableau de bord.
 */
function initControls() {
    // --- CONTRÔLES PRINCIPAUX : GPS & STATUT ---
    
    // 🚩 CORRECTION CRITIQUE : Logique de bascule (toggle) pour le bouton MARCHE/PAUSE GPS
    const startBtn = $('start-btn'); // ID du bouton ▶️ MARCHE GPS
    if (startBtn) {
        startBtn.addEventListener('click', () => {
            // wID est l'identifiant de la session watchPosition.
            // Si wID existe, le GPS est ACTIF -> Mettre en pause.
            if (wID !== null) {
                stopGPS(true); // true = Arrêt manuel
            } else {
                // Sinon, le GPS est inactif -> Démarrer en mode Haute Fréquence.
                startGPS('HIGH_FREQ'); 
            }
        });
    }

    // Contrôle : Arrêt d'Urgence
    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', () => {
        // Supposons une fonction qui bascule l'état et met à jour l'affichage
        toggleEmergencyStop(); 
    });

    // --- CONTRÔLES DE RÉINITIALISATION ET CONFIGURATION ---

    // Contrôle : Réinitialiser Distance
    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => {
        if (emergencyStopActive) return;
        distM = 0; 
        timeMoving = 0; 
        // Mise à jour de l'affichage (ex: $('#distance-total-km').textContent = '0.000 km | 0.00 m';)
    });
    
    // Contrôle : Réinitialiser Vitesse Max
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', () => {
        if (emergencyStopActive) return;
        maxSpd = 0.0;
        // Mise à jour de l'affichage (ex: $('#speed-max').textContent = '0.00000 km/h';)
    });
    
    // Contrôle : TOUT RÉINITIALISER
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { 
        if (confirm("Êtes-vous sûr de vouloir TOUT réinitialiser (EKF, Distance, Max, Historique) ?")) {
            stopGPS(true); // Arrête le GPS
            // Option 1: Réinitialisation des variables clés et rechargement de la page
            localStorage.clear();
            window.location.reload(); 
            
            // Option 2 (si rechargement non souhaité) :
            // distM = 0.0; maxSpd = 0.0; timeMoving = 0.0; timeTotal = 0.0;
            // ukf = new ProfessionalUKF(DEFAULT_LAT, DEFAULT_LON, RHO_SEA_LEVEL); // Réinit EKF
        }
    });

    // Contrôle : Forcer Précision GPS
    if ($('force-gps-precision-input')) $('force-gps-precision-input').addEventListener('input', (e) => {
        gpsAccuracyOverride = parseFloat(e.target.value) || 0.0;
    });

    // --- CONTRÔLES PHYSIQUE & ENVIRONNEMENT ---

    // Contrôle : Masse de l'objet (kg)
    if ($('mass-input')) $('mass-input').addEventListener('input', (e) => {
        currentMass = parseFloat(e.target.value) || 70.0;
        $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
    });
    
    // Contrôle : Sélection Corps Céleste
    if ($('celestial-body-select')) $('celestial-body-select').addEventListener('change', (e) => {
        currentCelestialBody = e.target.value;
        const { G_ACC_NEW } = updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity);
        $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
    });

    // Contrôle : Rayon/Vitesse Angulaire de Rotation (pour Corps Céleste 'Station')
    const updateRotation = () => {
        rotationRadius = parseFloat($('rotation-radius').value) || 100;
        angularVelocity = parseFloat($('angular-velocity').value.replace(',', '.')) || 0.0;
        if (currentCelestialBody === 'ROTATING') {
            const { G_ACC_NEW } = updateCelestialBody('ROTATING', kAlt, rotationRadius, angularVelocity);
            $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
        }
    };
    if ($('rotation-radius')) $('rotation-radius').addEventListener('input', updateRotation);
    if ($('angular-velocity')) $('angular-velocity').addEventListener('input', updateRotation);

    // Contrôle : Mode Nether (1:8 ou 1:1)
    if ($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', () => {
        netherMode = !netherMode;
        $('mode-nether').textContent = `Mode Nether: ${netherMode ? 'ACTIVÉ (1:8)' : 'DÉSACTIVÉ (1:1)'}`;
    });
}

/** * Fonction d'initialisation principale appelée au chargement du DOM. 
 */
function init() {
    // 1. Initialisation des fonctions critiques (UKF, Carte, Synchro)
    // initEKF(currentPosition.lat, currentPosition.lon, currentAirDensity); 
    // initMap(); 
    // syncH(); // Tente la synchro NTP
    
    // 2. Démarrage des boucles de mise à jour DOM (fastLoop pour les données critiques, slowLoop pour Astro/Météo)
    // startFastLoop();
    // startSlowLoop();
    
    // 3. Initialisation des gestionnaires d'événements des boutons/inputs (Le bloc corrigé)
    initControls(); 
    
    // 4. Initialisation des capteurs IMU (sans les démarrer, juste pour la demande de permission)
    // initializeIMUSensors(); 
}

// Assurez-vous que le script démarre après le chargement de toute la structure HTML
document.addEventListener('DOMContentLoaded', init);
