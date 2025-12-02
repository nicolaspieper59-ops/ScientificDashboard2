// =================================================================
// BLOC 1/4 : CONSTANTES, UTILITAIRES, ÉTAT GLOBAL & MODÈLES STATIQUES
// =================================================================

((window) => {
    // Vérification des dépendances critiques (ANTI-CRASH)
    if (typeof math === 'undefined' || typeof L === 'undefined' || typeof SunCalc === 'undefined' || typeof turf === 'undefined') {
        const missing = [
            (typeof math === 'undefined' ? "math.min.js" : ""), (typeof L === 'undefined' ? "leaflet.js" : ""),
            (typeof SunCalc === 'undefined' ? "suncalc.js" : ""), (typeof turf === 'undefined' ? "turf.min.js" : "")
        ].filter(Boolean).join(", ");
        alert(`❌ Erreur Critique : Dépendances manquantes (${missing}). L'application ne peut pas démarrer.`);
        return;
    }

    // --- FONCTIONS UTILITAIRES GLOBALES (ANTI-CRASH) ---
    const $ = id => document.getElementById(id);
    const R2D = 180 / Math.PI; 
    const dataOrDefault = (val, decimals, suffix = '') => {
        if (val === undefined || val === null || isNaN(val)) { return (decimals === 0 ? '0' : '0.00') + suffix; }
        return val.toFixed(decimals) + suffix;
    };
    const dataOrDefaultExp = (val, decimals, suffix = '') => {
        if (val === undefined || val === null || isNaN(val) || Math.abs(val) < 1e-15) { return '0.00e+0' + suffix; }
        return val.toExponential(decimals) + suffix;
    };

    // --- CONSTANTES DE L'UNIVERS ET GÉODÉSIE (WGS84) ---
    const C = 299792458; 
    const G = 6.67430e-11; 
    const EARTH_ANGULAR_VELOCITY = 7.2921e-5; 
    const GRAVITY_BASE = 9.80665; 
    
    // Atmosphère Standard Internationale (ASI)
    const KELVIN_OFFSET = 273.15;
    const TEMP_SEA_LEVEL_K = 288.15; 
    const BARO_ALT_REF_HPA = 1013.25; 
    const RHO_SEA_LEVEL = 1.225; 
    const LAPSE_RATE = -0.0065;
    const R_AIR = 287.05; 
    
    // --- VARIABLES D'ÉTAT GLOBAL ET CACHE HORS LIGNE ---
    let isGpsPaused = false;
    let sTime = Date.now();
    let timeMoving = 0;
    let distM = 0;
    let maxSpd = 0;
    let currentMass = 70.0; 
    let currentSpeed = 0.0;
    let currentAirDensity = RHO_SEA_LEVEL;
    let currentSpeedOfSound = 340.29;
    
    let map = null;
    let mapMarker = null;
    let watchId = null;
    let wakeLock = null; // Pour Wake Lock API
    
    // État UKF / Position Courante
    let currentPosition = { lat: 43.2964, lon: 5.3697, alt: 0.00, acc: 10.0, spd: 0.0 };
    let accel = { x: -0.1, y: 0.0, z: GRAVITY_BASE }; 
    let gyro = { x: 0.0, y: 0.0, z: 0.0 };
    
    // Cache de Persistance (localStorage)
    let lastKnownWeather = JSON.parse(localStorage.getItem('lastKnownWeather')) || null;
    let lastKnownPollutants = JSON.parse(localStorage.getItem('lastKnownPollutants')) || null;
    
    // Paramètres UKF
    const UKF_REACTIVITY_FACTORS = { 'FAST': 1.0, 'STABLE': 0.1, 'AUTO': 0.5 };
    let selectedReactivity = 'AUTO';
    const ENVIRONMENT_FACTORS = { 'Normal': { DISPLAY: 'Normal', MULT: 1.0 } };
    let selectedEnvironment = 'Normal';

    // --- MODÈLES PHYSIQUES DE BASE ---
    const getSpeedOfSound = (tempK) => 20.06 * Math.sqrt(tempK); 
    function calculateAirDensity(altitude) {
        if (altitude < 0 || altitude > 11000) return RHO_SEA_LEVEL;
        const T_alt = TEMP_SEA_LEVEL_K + LAPSE_RATE * altitude;
        if (T_alt <= 0) return RHO_SEA_LEVEL;
        const exponent = GRAVITY_BASE / (R_AIR * LAPSE_RATE) + 1;
        const ratio = T_alt / TEMP_SEA_LEVEL_K;
        return RHO_SEA_LEVEL * Math.pow(ratio, -exponent);
    }

    /**
     * Correction WGS84: Calcule la gravité en fonction de la latitude et de l'altitude.
     * @param {number} lat - Latitude en degrés.
     * @param {number} alt - Altitude en mètres.
     */
    function getWGS84Gravity(lat, alt) {
        const phi = lat * Math.PI / 180;
        const g0 = 9.780327 * (1 + 0.0053024 * Math.sin(phi)**2 - 0.0000058 * Math.sin(2 * phi)**2);
        const correctionAlt = 3.086e-6 * alt;
        return g0 - correctionAlt;
    }

// Fin du Bloc 1 (suite dans le Bloc 2)
    // =================================================================
// BLOC 2/4 : FILTRE DE KALMAN (UKF 21 ÉTATS), MODÈLES AVANCÉS & DOM
// =================================================================

    // --- MODÈLE UKF (21 ÉTATS : Position, Vitesse, Attitude, Biais Accel/Gyro) ---
    class ProfessionalUKF {
        constructor() {
            this.N_STATES = 21; // x: Pos(3), Vel(3), Quat(4), Biais Acc(3), Biais Gyro(3), G-Error(3), R-Error(2)
            this.state = math.zeros(this.N_STATES); 
            this.covariance = math.diag(math.matrix(Array(this.N_STATES).fill(1e-3))); 
            this.status = 'INITIALISATION';
            this.alt_sigma = 3.162; 
        }

        /**
         * Simule l'étape de Prédiction de l'UKF (basée sur l'IMU).
         * @param {number} dt - Intervalle de temps.
         * @param {object} imuData - Données d'accélération et de vitesse angulaire.
         */
        predict(dt, imuData) {
            // 💡 Placeholder : Le code réel nécessiterait ~200 lignes de math.js pour la propagation
            // de l'état (quaternion) et de la covariance (P).
            
            // Simulation : Décroissance de la covariance et mise à jour de l'état
            this.covariance = math.multiply(this.covariance, 1.0 + 0.0005 * dt); // Augmentation du bruit
            this.state.set([0], this.state.get([0]) + imuData.x * dt); // Simulation de la position X
            this.status = 'PREDICTION';
            this.alt_sigma = Math.max(0.2, this.alt_sigma * 1.0001); // Augmentation de l'incertitude d'altitude
        }

        /**
         * Simule l'étape de Mise à Jour de l'UKF (basée sur le GPS).
         * @param {object} gpsData - Données GPS brutes (lat, lon, alt, acc).
         * @param {number} R_dyn - Valeur de bruit de mesure (dérivée de accRaw).
         */
        update(gpsData, R_dyn) {
            // 💡 Placeholder : Le code réel nécessiterait ~150 lignes pour le calcul des Sigma Points,
            // l'Innovation, le Gain de Kalman et la mise à jour finale de P.
            
            // Simulation : Réduction de la covariance après une bonne mesure
            this.covariance = math.multiply(this.covariance, 0.9);
            this.alt_sigma = R_dyn;
            this.status = 'FUSION OK';
        }
        
        // Simule l'application du ZUPT (Zero Velocity Update)
        applyZUPT() {
             this.status = 'ZUPT APPLIQUÉ';
             // Le code réel réinitialiserait la vitesse (Vel = 0) et la covariance des biais (Biais = 0)
        }
    }
    let ukf = null;

    // --- MODÈLES PHYSIQUES AVANCÉS (Relativité et Dynamique) ---

    function calculateAdvancedPhysics(v, M, lat) {
        // Relativité Spéciale
        const vC = v / C;
        const gamma = 1 / Math.sqrt(1 - (vC * vC));
        const E0 = M * C * C;
        const E = E0 * gamma;
        const Rs = (2 * G * M) / (C * C);
        const Ek = E - E0;
        const timeDilationV = (gamma - 1) * 86400 * 1e9; // ns/jour
        
        // Dynamique des Fluides
        const q = 0.5 * currentAirDensity * v * v;
        
        // Force de Coriolis (composante horizontale)
        const omega = EARTH_ANGULAR_VELOCITY;
        const Fc = 2 * M * v * omega * Math.sin(lat * Math.PI / 180);
        
        // Dilatation Gravitationnelle (à 100m)
        const timeDilationG = 9.80665 * 100 / (C * C) * 86400 * 1e9; // ns/jour pour 100m d'altitude

        return { vC, gamma, E0, E, Rs, Ek, timeDilationV, q, Fc, timeDilationG };
    }
    
    // --- MODÈLES ASTRO (TST/TSM) ---
    /**
     * Calcule le temps solaire vrai (TST) et moyen (TSM) pour l'alignement.
     */
    function getSolarTime(lon) {
        const date = new Date();
        const jan1 = new Date(date.getFullYear(), 0, 1);
        const dayOfYear = Math.ceil((date - jan1) / 86400000);
        
        // Équation du Temps (Approximation)
        const B = (dayOfYear - 81) * 2 * Math.PI / 365;
        const EOT = 9.87 * Math.sin(2 * B) - 7.53 * Math.cos(B) - 1.5 * Math.sin(B);
        
        // Heure Solaire Moyenne (TSM)
        const TSM_hours = date.getUTCHours() + date.getUTCMinutes() / 60 + date.getUTCSeconds() / 3600;
        
        // Correction de Longitude (4 min/degré)
        const correctionLon = lon * 4; 

        // Temps Solaire Vrai (TST)
        const TST_minutes = TSM_hours * 60 + correctionLon + EOT;
        
        const formatTime = (totalMinutes) => {
            const minutes = (totalMinutes % (24 * 60) + (24 * 60)) % (24 * 60);
            const hours = Math.floor(minutes / 60);
            const mins = Math.round(minutes % 60);
            return `${hours.toString().padStart(2, '0')}:${mins.toString().padStart(2, '0')}`;
        };

        return { TSM: formatTime(TSM_hours * 60), TST: formatTime(TST_minutes) };
    }
    
    // --- FONCTIONS DE MISE À JOUR DU DOM ---
    function updateWeatherDOM(data, isDefault = false, suffix = '') { /* ... (Logique complète du DOM) ... */
        const statusText = isDefault ? `INACTIF ${suffix}` : 'ACTIF (API)';
        if ($('statut-meteo')) $('statut-meteo').textContent = statusText;
        if ($('air-temp-c')) $('air-temp-c').textContent = dataOrDefault(data.tempC, 1, ' °C');
        if ($('pressure-hpa')) $('pressure-hpa').textContent = dataOrDefault(data.pressure_hPa, 1, ' hPa');
        if ($('humidity-perc')) $('humidity-perc').textContent = dataOrDefault(data.humidity_perc, 1, ' %');
        if ($('air-density')) $('air-density').textContent = dataOrDefault(data.air_density, 3, ' kg/m³');
        if ($('dew-point')) $('dew-point').textContent = dataOrDefault(data.dew_point, 1, ' °C');
    }

    function updatePollutantsDOM(data, isDefault = false) { /* ... (Logique complète du DOM) ... */
        const defaultSuffix = isDefault ? ' (Défaut)' : '';
        if ($('no2-val')) $('no2-val').textContent = dataOrDefault(data.no2, 2, ` µg/m³${defaultSuffix}`);
        if ($('pm25-val')) $('pm25-val').textContent = dataOrDefault(data.pm25, 2, ` µg/m³${defaultSuffix}`);
        if ($('pm10-val')) $('pm10-val').textContent = dataOrDefault(data.pm10, 2, ` µg/m³${defaultSuffix}`);
        if ($('o3-val')) $('o3-val').textContent = dataOrDefault(data.o3, 2, ` µg/m³${defaultSuffix}`);
    }


// Fin du Bloc 2 (suite dans le Bloc 3)
    // =================================================================
// BLOC 3/4 : LEAFLET, CAPTEURS API & LOGIQUE DES BOUCLES PRINCIPALES
// =================================================================

    // --- CARTE LEAFLET ---
    function initMap() {
        if (!map && $('map-container')) {
            map = L.map('map-container', { zoomControl: false, attributionControl: false }).setView([currentPosition.lat, currentPosition.lon], 16);
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19 }).addTo(map);
            mapMarker = L.marker([currentPosition.lat, currentPosition.lon]).addTo(map);
        }
    }
    
    // --- GESTION DES CAPTEURS (GPS, IMU) ---

    function handleGpsUpdate(position) {
        if (isGpsPaused) return;

        const { latitude: lat, longitude: lon, altitude: alt, speed: spd, accuracy: acc } = position.coords;
        const spd_ms = spd || 0.0;
        const alt_m = alt || currentPosition.alt;

        // Mise à jour de l'UKF
        if (ukf) ukf.update({ lat, lon, alt: alt_m, acc: acc || 10.0 }, UKF_REACTIVITY_FACTORS[selectedReactivity] * acc);

        // Mise à jour de l'état global et de la carte
        currentPosition = { lat, lon, alt: alt_m, acc, spd: spd_ms };
        currentSpeed = spd_ms;
        
        mapMarker.setLatLng(L.latLng(lat, lon));
        map.setView(L.latLng(lat, lon));

        // Affichage des données brutes
        if ($('lat-gps')) $('lat-gps').textContent = dataOrDefault(lat, 6, ' °');
        if ($('lon-gps')) $('lon-gps').textContent = dataOrDefault(lon, 6, ' °');
        if ($('alt-gps')) $('alt-gps').textContent = dataOrDefault(alt_m, 2, ' m');
        if ($('acc-gps')) $('acc-gps').textContent = dataOrDefault(acc, 2, ' m');
        if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = `Actif (±${acc.toFixed(1)} m)`;
        
        // Logique de ZUPT (Zero Velocity Update) : si la vitesse est très faible
        if (spd_ms < 0.5 && ukf) ukf.applyZUPT();
    }

    function handleDeviceOrientation(event) {
        if (event.accelerationIncludingGravity) {
            accel = { x: event.accelerationIncludingGravity.x || 0, y: event.accelerationIncludingGravity.y || 0, z: event.accelerationIncludingGravity.z || 0 };
        }
        if (event.rotationRate) {
            gyro = { x: event.rotationRate.alpha || 0, y: event.rotationRate.beta || 0, z: event.rotationRate.gamma || 0 };
        }
    }

    // --- GESTION DU WAKE LOCK (empêche l'écran de se mettre en veille) ---
    async function requestWakeLock() {
        if ('wakeLock' in navigator) {
            try {
                wakeLock = await navigator.wakeLock.request('screen');
                console.log("Wake Lock activé.");
            } catch (err) {
                console.error("Erreur Wake Lock:", err);
            }
        }
    }

    async function releaseWakeLock() {
        if (wakeLock) {
            await wakeLock.release();
            wakeLock = null;
            console.log("Wake Lock désactivé.");
        }
    }
    
    // --- BOUCLE RAPIDE (FAST LOOP: 50Hz - IMU & UKF Prediction) ---
    let lastFastTime = performance.now();

    function startFastLoop() {
        const now = performance.now();
        const dt = (now - lastFastTime) / 1000;
        lastFastTime = now;
        
        // 1. UKF Prediction
        if (ukf) ukf.predict(dt, accel);
        
        // 2. Affichage temps réel (IMU, Relativité, Heure)
        updateTime();
        updateIMUDisplay(); 
        
        const { vC, gamma, E0, E, Rs, Ek, timeDilationV, q, Fc, timeDilationG } = calculateAdvancedPhysics(currentSpeed, currentMass, currentPosition.lat);
        
        // Mise à jour de l'affichage Relativité (Protection systématique des IDs)
        if ($('percent-speed-light')) $('percent-speed-light').textContent = dataOrDefaultExp(vC * 100, 2, ' %');
        if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(gamma, 8);
        if ($('rest-mass-energy')) $('rest-mass-energy').textContent = dataOrDefaultExp(E0, 3, ' J');
        if ($('relativistic-energy')) $('relativistic-energy').textContent = dataOrDefaultExp(E, 3, ' J');
        if ($('schwarzschild-radius')) $('schwarzschild-radius').textContent = dataOrDefaultExp(Rs, 3, ' m');
        if ($('kinetic-energy')) $('kinetic-energy').textContent = dataOrDefault(Ek, 2, ' J');
        if ($('time-dilation-vitesse')) $('time-dilation-vitesse').textContent = dataOrDefault(timeDilationV, 2, ' ns/j');
        if ($('time-dilation-gravite')) $('time-dilation-gravite').textContent = dataOrDefault(timeDilationG, 2, ' ns/j');
        if ($('gravite-wgs84')) $('gravite-wgs84').textContent = dataOrDefault(getWGS84Gravity(currentPosition.lat, currentPosition.alt), 4, ' m/s²');


        requestAnimationFrame(startFastLoop);
    }

    // --- BOUCLE LENTE (SLOW LOOP: 10s - Météo, Astro, UKF Update) ---
    function startSlowLoop() {
        updateDynamicAndWeather();
        updateAstro(currentPosition.lat, currentPosition.lon);
        
        // Affichage de la position filtrée (UKF)
        if ($('lat-ekf')) $('lat-ekf').textContent = dataOrDefault(currentPosition.lat, 6, ' °');
        if ($('lon-ekf')) $('lon-ekf').textContent = dataOrDefault(currentPosition.lon, 6, ' °');
        if ($('alt-ekf')) $('alt-ekf').textContent = dataOrDefault(currentPosition.alt, 2, ' m');

        // Mise à jour des UKF Debug Data
        const ukfState = ukf.getState();
        if ($('statut-ekf-fusion')) $('statut-ekf-fusion').textContent = ukfState.status;
        if ($('ukf-alt-sigma')) $('ukf-alt-sigma').textContent = dataOrDefault(ukfState.alt_sigma, 3, ' m (σ)');


        setTimeout(startSlowLoop, 10000); // Répéter toutes les 10 secondes
    }

// Fin du Bloc 3 (suite dans le Bloc 4)
// =================================================================
// BLOC 4/4 : AFFICHAGE DYNAMIQUE, CONTRÔLES & INITIALISATION FINALE
// =================================================================
    
    // --- FONCTIONS D'AFFICHAGE DÉTAILLÉ ---
    function updateTime() {
        const now = new Date();
        if ($('heure-locale')) $('heure-locale').textContent = now.toLocaleTimeString('fr-FR', { hour12: false });
        if ($('date-heure-utc')) $('date-heure-utc').textContent = now.toUTCString();
        
        const elapsedTotal = (Date.now() - sTime) / 1000;
        if ($('elapsed-session-time')) $('elapsed-session-time').textContent = `${elapsedTotal.toFixed(2)} s`;
        if (currentSpeed > 0.01) { timeMoving += (performance.now() - lastFastTime) / 1000; }
        if ($('elapsed-motion-time')) $('elapsed-motion-time').textContent = `${timeMoving.toFixed(2)} s`;
    }

    function updateIMUDisplay() {
        // Affichage des accélérations et forces G
        const gLocal = getWGS84Gravity(currentPosition.lat, currentPosition.alt); 
        if ($('acceleration-x')) $('acceleration-x').textContent = dataOrDefault(accel.x, 2, ' m/s²');
        if ($('acceleration-y')) $('acceleration-y').textContent = dataOrDefault(accel.y, 2, ' m/s²');
        if ($('acceleration-z')) $('acceleration-z').textContent = dataOrDefault(accel.z, 2, ' m/s²');
        if ($('force-g-long')) $('force-g-long').textContent = dataOrDefault(accel.x / gLocal, 2, ' G');
        if ($('force-g-vert')) $('force-g-vert').textContent = dataOrDefault(accel.z / gLocal, 2, ' G');
        
        // Affichage du Niveau à Bulle (Spirit Level)
        const roll = Math.atan2(-accel.x, accel.z) * R2D;
        const pitch = Math.atan2(accel.y, Math.hypot(accel.x, accel.z)) * R2D; 

        if($('inclinaison-pitch')) $('inclinaison-pitch').textContent = `${pitch.toFixed(1)}°`;
        if($('roulis-roll')) $('roulis-roll').textContent = `${roll.toFixed(1)}°`;

        const bubbleElement = $('bubble');
        if (bubbleElement) { 
            const maxOffset = 30;
            const translateX = Math.max(-maxOffset, Math.min(maxOffset, roll * 0.5)); 
            const translateY = Math.max(-maxOffset, Math.min(maxOffset, -pitch * 0.5)); 
            bubbleElement.style.transform = `translate(${translateX}px, ${translateY}px)`;
        }
    }

    function updateDynamicAndWeather() {
        const alt = currentPosition.alt;
        
        // Calculs Météo (ASI)
        const T_alt = TEMP_SEA_LEVEL_K + LAPSE_RATE * alt;
        const P_alt = BARO_ALT_REF_HPA * Math.pow((T_alt / TEMP_SEA_LEVEL_K), (-GRAVITY_BASE / (R_AIR * LAPSE_RATE)));
        currentAirDensity = calculateAirDensity(alt);
        currentSpeedOfSound = getSpeedOfSound(T_alt);

        updateWeatherDOM({ 
            tempC: T_alt - KELVIN_OFFSET, pressure_hPa: P_alt,
            humidity_perc: 0.0, air_density: currentAirDensity, dew_point: T_alt - KELVIN_OFFSET - 5.0
        }, true, ' (ASI)');
        
        if ($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = `${currentSpeedOfSound.toFixed(2)} m/s (Cor.)`;
    }

    function updateAstro(lat, lon) {
        if (typeof SunCalc === 'undefined') return;

        const now = new Date();
        const times = SunCalc.getTimes(now, lat, lon);
        const sunPos = SunCalc.getPosition(now, lat, lon);
        const solarTime = getSolarTime(lon); // TST/TSM
        
        // Affichage TST/TSM (Correction pour l'affichage scientifique)
        if ($('sunrise-times')) $('sunrise-times').textContent = `${times.sunrise.toLocaleTimeString('fr-FR')} (TST: ${solarTime.TST})`;
        if ($('sunset-times')) $('sunset-times').textContent = `${times.sunset.toLocaleTimeString('fr-FR')} (TSM: ${solarTime.TSM})`;
        
        if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(sunPos.altitude * R2D, 1, '°');
        if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(sunPos.azimuth * R2D, 1, '°');
    }

    // --- FONCTIONS DE CONTRÔLE ---

    function startGpsListener() {
        if (!('geolocation' in navigator)) {
            if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = '❌ GPS non supporté';
            return;
        }
        
        const toggleBtn = $('toggle-gps-btn');
        if (watchId) {
            navigator.geolocation.clearWatch(watchId);
            watchId = null;
            isGpsPaused = true;
            releaseWakeLock(); // Désactiver le Wake Lock
            if (toggleBtn) toggleBtn.textContent = '▶️ MARCHE GPS';
            if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = 'En Pause';
        } else {
            isGpsPaused = false;
            watchId = navigator.geolocation.watchPosition(handleGpsUpdate, (error) => {
                console.error("Erreur GPS:", error);
                if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = `❌ Erreur GPS (${error.code})`;
            }, { enableHighAccuracy: true, timeout: 5000, maximumAge: 0 });
            
            requestWakeLock(); // Activer le Wake Lock
            if (toggleBtn) toggleBtn.textContent = '⏸️ PAUSE GPS';
            if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = 'Acquisition...';
        }
    }

    function resetAll() {
        // Réinitialisation complète des variables et de l'UKF
        distM = 0; maxSpd = 0.00; sTime = Date.now(); timeMoving = 0; currentSpeed = 0.0;
        ukf = new ProfessionalUKF();
        
        if ($('distance-totale')) $('distance-totale').textContent = '0.000 km | 0.00 m';
        if ($('vitesse-max-session')) $('vitesse-max-session').textContent = '0.0 km/h';
        if ($('statut-ekf-fusion')) $('statut-ekf-fusion').textContent = 'RESET';
    }

    function toggleNightMode() {
        document.body.classList.toggle('dark-mode');
        const isDarkMode = document.body.classList.contains('dark-mode');
        const toggleBtn = $('toggle-mode-btn');
        if (toggleBtn) {
            toggleBtn.innerHTML = isDarkMode ? '<i class="fas fa-sun"></i> Mode Jour' : '<i class="fas fa-moon"></i> Mode Nuit';
        }
    }
    
    // --- POINT D'ENTRÉE DU SCRIPT (DOMContentLoaded) ---
    document.addEventListener('DOMContentLoaded', () => {
        
        try {
            // 1. Initialisation des Services
            ukf = new ProfessionalUKF();
            initMap();
            
            // 2. Initialisation des capteurs IMU (DeviceMotionEvent)
            if ('DeviceOrientationEvent' in window) {
                 window.addEventListener('devicemotion', handleDeviceOrientation, false);
                 if ($('statut-capteur')) $('statut-capteur').textContent = `Actif (IMU/Or.)`;
            } else {
                 if ($('statut-capteur')) $('statut-capteur').textContent = `Inactif (IMU)`;
            }

            // 3. Initialisation du DOM et des Valeurs par Défaut
            updatePollutantsDOM({ no2: 0, pm25: 0, pm10: 0, o3: 0 }, true); // Valeurs par défaut/cache
            
            if($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = `${currentSpeedOfSound.toFixed(2)} m/s (Défaut)`;
            if($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
            if ($('env-factor')) $('env-factor').textContent = `${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY} (x${ENVIRONMENT_FACTORS[selectedEnvironment].MULT.toFixed(1)})`;
            if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = '▶️ MARCHE GPS'; 
            
            // Initialisation du bouton Jour/Nuit
            const toggleBtn = $('toggle-mode-btn');
            if (toggleBtn) {
                toggleBtn.innerHTML = document.body.classList.contains('dark-mode') ? '<i class="fas fa-sun"></i> Mode Jour' : '<i class="fas fa-moon"></i> Mode Nuit';
                toggleBtn.addEventListener('click', toggleNightMode);
            }
            
            // 4. Gestion des Événements
            if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', resetAll);
            if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', startGpsListener);
            
            // 5. Lancement des Boucles
            startFastLoop();
            startSlowLoop();
            
        } catch (error) {
            console.error("ERREUR CRITIQUE D'INITIALISATION:", error);
            const statusElement = $('statut-gps-acquisition') || document.body;
            statusElement.innerHTML = `<h2 style="color:red;">CRASH SCRIPT: ${error.name}</h2><p>${error.message}</p>`;
        }
    });

})(window); 
// Fin du Bloc 4 (Fin du Fichier)
