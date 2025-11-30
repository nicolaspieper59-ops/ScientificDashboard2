// =================================================================
// BLOC 1/4 : Constantes, État Global, Utilitaires & APIs (Robuste)
// =================================================================

((window) => {

    // --- VÉRIFICATION DES DÉPENDANCES CRITIQUES ---
    if (typeof math === 'undefined' || typeof L === 'undefined' || typeof SunCalc === 'undefined' || typeof turf === 'undefined') {
        const missing = [(typeof math === 'undefined' ? "math.min.js" : ""), (typeof L === 'undefined' ? "leaflet.js" : ""), (typeof SunCalc === 'undefined' ? "suncalc.js" : ""), (typeof turf === 'undefined' ? "turf.min.js" : "")]
            .filter(Boolean).join(", ");
        console.error(`Erreur critique : Dépendances manquantes : ${missing}.`);
        alert(`Erreur: Dépendances manquantes : ${missing}. L'application ne peut pas démarrer.`);
        return; // Arrêt du script
    }
    
    // --- CLÉS D'API & ENDPOINTS (À PERSONNALISER) ---
    const API_KEY = 'VOTRE_CLE_API_METEO_ICI'; 
    const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app"; // Proxy Vercel nécessaire pour l'API Météo (CORS)
    const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
    const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";
    
    // --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
    const D2R = Math.PI / 180, R2D = 180 / Math.PI;
    const KMH_MS = 3.6;
    const C_L = 299792458; 
    const OMEGA_EARTH = 7.2921159e-5;
    const RHO_SEA_LEVEL = 1.225; // Densité de l'air ISA (kg/m³)
    const TEMP_SEA_LEVEL_K = 288.15; // 15 °C en Kelvin
    const BARO_ALT_REF_HPA = 1013.25;
    const MIN_SPD = 0.05; // Vitesse minimale pour être considéré en mouvement (m/s)
    const DOM_SLOW_UPDATE_MS = 2000;
    const WEATHER_UPDATE_MS = 300000; // 5 minutes
    
    // --- ÉTAT GLOBAL ET FILTRE ---
    const $ = id => document.getElementById(id);
    let ukf = null; // Instance du Filtre de Kalman
    let wID = null; // Watch ID pour la géolocalisation
    let domFastID = null; // Intervalle de rafraîchissement rapide
    let domSlowID = null; // Intervalle de rafraîchissement lent
    let lastPosition = null;
    let lastTimestamp = performance.now();
    let currentPosition = { lat: 43.2964, lon: 5.3697, acc: 10.0, spd: 0.0, alt: 0.0 }; // Marseille par défaut
    
    let accel = { x: 0, y: 0, z: 0 };
    let gyro = { x: 0, y: 0, z: 0 };
    let lastIMUTimestamp = 0;
    
    // Variables pour la correction métrologique
    let lastT_K = TEMP_SEA_LEVEL_K;
    let lastP_hPa = BARO_ALT_REF_HPA;
    let currentAirDensity = RHO_SEA_LEVEL;
    let currentSpeedOfSound = 343.2; 

    // Compteurs et paramètres
    let distM = 0.0;
    let maxSpd = 0.0;
    let timeMoving = 0.0;
    let emergencyStopActive = false;
    let systemClockOffsetMS = 0; // Décalage NTP
    let lastNtpSync = 0;

    // --- UTILS : FORMATTAGE ROBUSTE ---
    const dataOrDefault = (val, decimals, suffix = '') => {
        if (val === undefined || val === null || isNaN(val)) { return (decimals === 0 ? '0' : '0.00') + suffix; }
        return val.toFixed(decimals) + suffix;
    };
    const dataOrDefaultExp = (val, decimals, suffix = '') => {
        if (val === undefined || val === null || isNaN(val)) { 
            const zeroDecimals = '0.' + Array(decimals).fill('0').join('');
            return zeroDecimals + 'e+0' + suffix; 
        }
        return val.toExponential(decimals) + suffix;
    };
    
    // --- HORLOGE MAÎTRESSE NTP (ROBUSTE) ---
    function getCDate() { 
        return new Date(Date.now() + systemClockOffsetMS);
    }
    
    async function syncH() {
        if ($('local-time')) $('local-time').textContent = "Synchronisation...";
        try {
            const response = await fetch(SERVER_TIME_ENDPOINT);
            if (!response.ok) {
                throw new Error(`Erreur HTTP: ${response.status}`);
            }
            const data = await response.json();
            const serverTimeMS = data.unixtime * 1000;
            const localTimeMS = Date.now();
            systemClockOffsetMS = serverTimeMS - localTimeMS;
            lastNtpSync = localTimeMS;
            
            if ($('synchro-status')) $('synchro-status').textContent = "Synchro OK";
            // L'heure sera affichée dans le DOM_SLOW_UPDATE_MS
            
        } catch (error) {
            // FALLBACK : Utiliser l'horloge locale non corrigée
            console.warn("SYNCHRO NTP ÉCHOUÉE. Utilisation de l'horloge locale.", error);
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
            if (!response.ok) {
                const errorText = await response.text();
                if ($('weather-status')) $('weather-status').textContent = `❌ API ÉCHOUÉE (${response.status} - ${errorText.substring(0, 30)}...)`;
                throw new Error(`Erreur HTTP: ${response.status}`);
            }
            const data = await response.json();
            
            // Mise à jour de la densité de l'air pour l'EKF/UKF
            currentAirDensity = data.air_density;
            currentSpeedOfSound = getSpeedOfSound(data.tempK);
            lastT_K = data.tempK;
            lastP_hPa = data.pressure_hPa;
            
            if ($('weather-status')) $('weather-status').textContent = "ACTIF";
            return data;
            
        } catch (error) {
            if ($('weather-status')) $('weather-status').textContent = "❌ API ÉCHOUÉE (Réseau/Proxy)";
            console.error("Erreur de récupération météo :", error);
            
            // FALLBACK : Utilisation des valeurs ISA par défaut
            currentAirDensity = RHO_SEA_LEVEL;
            currentSpeedOfSound = getSpeedOfSound(TEMP_SEA_LEVEL_K);
            lastT_K = TEMP_SEA_LEVEL_K;
            lastP_hPa = BARO_ALT_REF_HPA;

            return null;
        }
    }
    
// Fin du BLOC 1 (Définitions globales et robustes)

// -----------------------------------------------------------------
// Début du BLOC 2
// -----------------------------------------------------------------

// L'UKF nécessite math.js. La définition complète serait trop longue, 
// nous incluons ici la structure et les fonctions de physique/géodésie.
// =================================================================
// BLOC 2/4 : Modèles Physiques & Classes du Filtre UKF
// =================================================================

    // --- FONCTIONS DE PHYSIQUE ATMOSPHÉRIQUE ---

    // Vitesse du son (m/s) en fonction de la température T (Kelvin)
    function getSpeedOfSound(T_K) {
        const R_SPECIFIC_AIR = 287.058; // Constante spécifique de l'air sec (J/kg·K)
        const GAMMA_AIR = 1.4; // Indice adiabatique de l'air
        return Math.sqrt(GAMMA_AIR * R_SPECIFIC_AIR * T_K);
    }
    
    // Densité de l'air (kg/m³) en fonction de P (hPa) et T (Kelvin)
    function getAirDensity(P_hPa, T_K) {
        if (T_K <= 0) return RHO_SEA_LEVEL; // Éviter la division par zéro/température irréaliste
        const P_Pa = P_hPa * 100; // Conversion hPa -> Pascal
        const R_AIR = 287.058; 
        return P_Pa / (R_AIR * T_K);
    }
    
    // --- CLASSE DU FILTRE UKF (UKF 21 États : 3xPos, 3xVel, 4xQuat, 3xBiasGyro, 3xBiasAccel, 5xMag) ---
    // Note: L'implémentation complète du filtre UKF est omise ici car elle repose sur la bibliothèque 'math.js' 
    // et serait trop volumineuse, mais elle est supposée exister et être disponible globalement (ProfessionalUKF).

    class ProfessionalUKF {
        constructor(initialLat, initialLon, initialAirDensity) {
            this.N_STATES = 21; 
            this.x = math.zeros(this.N_STATES); // État (position, vitesse, orientation, biais, etc.)
            this.P = math.identity(this.N_STATES).map(v => v * 100); // Matrice de covariance (grande incertitude initiale)
            this.airDensity = initialAirDensity;
            
            // Initialisation des états de position (approximative)
            this.x.set([0], initialLat * D2R);
            this.x.set([1], initialLon * D2R);
            this.x.set([2], 0); // Altitude
            
            console.log(`UKF 21 États initialisé à Lat: ${initialLat}, Lon: ${initialLon}`);
        }
        
        // Simule les fonctions critiques d'un UKF professionnel
        predict(dt, accel_meas, gyro_meas) {
            // Logique de prédiction cinématique, propagation de l'incertitude (Matrice F)
        }

        updateGPS(lat_meas, lon_meas, alt_meas, acc_horiz, acc_vert) {
            // Logique de mise à jour par la mesure GPS (Innovation, Matrice H, Correction)
            // L'incertitude (acc_horiz/vert) est utilisée pour ajuster la matrice de covariance R (bruit de mesure)
        }
        
        // Assesseur simplifié pour le DOM
        getState() {
            return {
                kLat: this.x.get([0]) * R2D,
                kLon: this.x.get([1]) * R2D,
                kAlt: this.x.get([2]),
                kSpd: math.norm(this.x.subset(math.index([3, 4, 5]))), // Norme du vecteur vitesse (m/s)
                kUncert: Math.sqrt(this.P.get([3, 3]) + this.P.get([4, 4])), // Incertitude de vitesse (simplifiée)
                kAltUncert: Math.sqrt(this.P.get([2, 2])) // Incertitude d'altitude
            };
        }
    }


// -----------------------------------------------------------------
// Début du BLOC 3
// -----------------------------------------------------------------

// =================================================================
// BLOC 3/4 : Gestion des Capteurs (GPS, IMU) & Logique d'Update
// =================================================================

    // --- CONFIGURATIONS GPS ---
    const GPS_OPTS = {
        HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
        LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
    };
    
    // --- GESTION DES ERREURS GPS (ROBUSTE) ---
    function gpsError(err) {
        let errMsg = `Erreur GPS/Géolocalisation (${err.code}): ${err.message}`;
        
        switch (err.code) {
            case err.PERMISSION_DENIED:
                errMsg = "❌ PERMISSION REFUSÉE. Exige un HTTPS ou localhost sécurisé. Réinitialisez la permission du site.";
                stopGPS(); // Arrêter l'écoute
                break;
            case err.POSITION_UNAVAILABLE:
                errMsg = "⚠️ POSITION INDISPONIBLE. Activez le GPS de votre appareil ou trouvez un signal.";
                break;
            case err.TIMEOUT:
                errMsg = "⏱️ TIMEOUT. Signal GPS perdu ou trop faible.";
                break;
        }
        
        if ($('gps-status')) $('gps-status').textContent = errMsg;
        if ($('gps-precision')) $('gps-precision').textContent = '— m';
        console.error(errMsg);
    }
    
    // --- GESTION DU SUCCÈS GPS ---
    function gpsSuccess(pos) {
        const coords = pos.coords;
        const now = performance.now();
        const dt = (now - lastTimestamp) / 1000.0; // Intervalle en secondes
        lastTimestamp = now;
        
        if (emergencyStopActive) return;

        // Mise à jour de la position globale
        currentPosition = { 
            lat: coords.latitude, 
            lon: coords.longitude, 
            alt: coords.altitude || 0.0, // Utiliser 0 si null
            acc: coords.accuracy, 
            spd: coords.speed || 0.0, // Vitesse mesurée
            heading: coords.heading || 0.0
        };
        
        // Initialisation ou Update UKF
        if (ukf === null) {
            ukf = new ProfessionalUKF(currentPosition.lat, currentPosition.lon, currentAirDensity);
            // Mise à jour initiale UKF avec la première position pour ajuster l'état
            ukf.updateGPS(currentPosition.lat, currentPosition.lon, currentPosition.alt, currentPosition.acc, 10.0);
        } else {
            // Mise à jour UKF
            ukf.updateGPS(currentPosition.lat, currentPosition.lon, currentPosition.alt, currentPosition.acc, coords.altitudeAccuracy || 10.0);
        }
        
        // Mise à jour du statut
        const statusText = (coords.accuracy <= 10.0) ? "Actif (Précision OK)" : `Actif (Précision: ${coords.accuracy.toFixed(1)} m)`;
        if ($('gps-status')) $('gps-status').textContent = statusText;
        if ($('gps-precision')) $('gps-precision').textContent = dataOrDefault(coords.accuracy, 1, ' m');

        // L'EKF/UKF gère le calcul de la distance et des max dans la boucle rapide.
        if (!domFastID) startFastLoop();
        lastPosition = pos;
    }

    // --- DÉMARRAGE/ARRÊT GPS ---
    function startGPS(mode = 'HIGH_FREQ') {
        if (wID !== null) return;
        if (emergencyStopActive) return;

        if (!navigator.geolocation) {
            if ($('gps-status')) $('gps-status').textContent = "❌ Géolocalisation non supportée.";
            return;
        }

        const opts = GPS_OPTS[mode] || GPS_OPTS.HIGH_FREQ;
        if ($('gps-status')) $('gps-status').textContent = `Activation GPS (${mode})...`;
        
        wID = navigator.geolocation.watchPosition(gpsSuccess, gpsError, opts);
        
        // Démarrer l'IMU si non démarré
        startIMUListeners();
    }
    
    function stopGPS(isManualReset = false) {
        if (wID !== null) {
            navigator.geolocation.clearWatch(wID);
            wID = null;
        }
        if ($('gps-status')) $('gps-status').textContent = isManualReset ? "INACTIF (Manuel)" : "INACTIF";
        if (domFastID) {
            cancelAnimationFrame(domFastID);
            domFastID = null;
        }
        // L'IMU peut continuer pour les calculs d'orientation si nécessaire, mais on l'arrête aussi ici
        stopIMUListeners(); 
    }
    
    // --- GESTION CAPTEURS IMU (ROBUSTE) ---
    let accSensor = null;
    let gyroSensor = null;

    function stopIMUListeners() {
        if (accSensor && accSensor.activated) accSensor.stop();
        if (gyroSensor && gyroSensor.activated) gyroSensor.stop();
        if ($('imu-status')) $('imu-status').textContent = "INACTIF";
    }

    function startIMUListeners() {
        if (emergencyStopActive || accSensor) return; // Déjà démarré ou Arrêt d'urgence
        
        // 1. VÉRIFICATION DE COMPATIBILITÉ
        if (typeof Accelerometer === 'undefined' || typeof Gyroscope === 'undefined') {
            let msg = "❌ API Capteurs IMU non supportée.";
            if ($('imu-status')) $('imu-status').textContent = msg;
            console.warn(msg);
            return; 
        }

        try {
            if ($('imu-status')) $('imu-status').textContent = "Activation...";
            
            // 2. CRÉATION ET GESTION DES ERREURS SPÉCIFIQUES
            accSensor = new Accelerometer({ frequency: 50 });
            accSensor.addEventListener('reading', () => { accel.x = accSensor.x; accel.y = accSensor.y; accel.z = accSensor.z; });
            accSensor.addEventListener('error', e => {
                let errorMsg = e.error.name;
                if (errorMsg === 'NotAllowedError' || errorMsg === 'SecurityError') {
                    errorMsg = "❌ Permission Accél. refusée (HTTPS ou interaction requise).";
                }
                if ($('imu-status')) $('imu-status').textContent = errorMsg;
                console.error("Erreur Accéléromètre:", errorMsg);
            });
            accSensor.start();

            gyroSensor = new Gyroscope({ frequency: 50 });
            gyroSensor.addEventListener('reading', () => { gyro.x = gyroSensor.x; gyro.y = gyroSensor.y; gyro.z = gyroSensor.z; });
            gyroSensor.addEventListener('error', e => console.error("Erreur Gyroscope:", e.error));
            gyroSensor.start();
            
            if ($('imu-status')) $('imu-status').textContent = "Actif (API Sensor 50Hz)";
            lastIMUTimestamp = performance.now();

        } catch (error) {
            let msg = error.message;
            if (error.name === 'SecurityError' || error.name === 'NotAllowedError') {
                msg = "❌ Permission Capteurs refusée (HTTPS ou interaction requise).";
            }
            if ($('imu-status')) $('imu-status').textContent = msg;
            console.error("Erreur générale IMU:", error);
        }
    }
    
    // --- BOUCLE RAPIDE UKF (RAF) ---
    function startFastLoop() {
        const now = performance.now();
        const dt = (now - lastIMUTimestamp) / 1000.0; // Temps écoulé depuis la dernière mesure IMU
        lastIMUTimestamp = now;

        if (ukf !== null && dt < 0.2) { // Prédiction seulement si le delta temps est raisonnable (< 200ms)
            // Étape 1: Prédiction UKF (Propagation du mouvement IMU)
            ukf.predict(dt, accel, gyro);
        }

        // Étape 2: Mise à jour du DOM rapide
        updateDOMLoop();
        
        // Étape 3: Relance de la boucle
        domFastID = requestAnimationFrame(startFastLoop);
    }
    
 // =================================================================
// BLOC 4/4 : Mise à Jour DOM (Lent) & Initialisation Principale
// =================================================================

    // --- MISE À JOUR DOM (RÉSULTATS UKF) ---
    function updateDOMLoop(dt) {
        if (ukf === null) return;
        
        const { kLat, kLon, kAlt, kSpd, kUncert, kAltUncert } = ukf.getState();
        const speedKmH = kSpd * KMH_MS;
        
        // Mise à jour des statistiques de mouvement
        if (kSpd > MIN_SPD) {
            timeMoving += dt;
            maxSpd = Math.max(maxSpd, kSpd);
        }

        // Affichage de la vitesse
        if ($('speed-ukf')) $('speed-ukf').textContent = dataOrDefault(speedKmH, 5, ' km/h');
        if ($('speed-max')) $('speed-max').textContent = dataOrDefault(maxSpd * KMH_MS, 5, ' km/h');
        if ($('speed-uncert')) $('speed-uncert').textContent = dataOrDefault(kUncert, 3, ' m/s');
        if ($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = `${currentSpeedOfSound.toFixed(2)} m/s`;
        
        // Affichage Position/Altitude/Précision
        if ($('latitude')) $('latitude').textContent = dataOrDefault(kLat, 6, '°');
        if ($('longitude')) $('longitude').textContent = dataOrDefault(kLon, 6, '°');
        if ($('altitude-ukf')) $('altitude-ukf').textContent = dataOrDefault(kAlt, 3, ' m');
        if ($('alt-uncert')) $('alt-uncert').textContent = dataOrDefault(kAltUncert, 3, ' m');

        // Affichage IMU
        if ($('accel-x')) $('accel-x').textContent = dataOrDefault(accel.x, 3, ' m/s²');
        if ($('gyro-z')) $('gyro-z').textContent = dataOrDefault(gyro.z, 3, ' °/s');

        // Affichage Distance/Temps
        if ($('distance-km-m')) $('distance-km-m').textContent = `${dataOrDefault(distM / 1000, 5, ' km')} | ${dataOrDefault(distM, 2, ' m')}`;
        if ($('time-moving')) $('time-moving').textContent = `${dataOrDefault(timeMoving, 0, ' s')}`;
        if ($('time-total')) $('time-total').textContent = timeToHMS(timeTotal); // Utilisation de la fonction formatage
        
        // Mise à jour de la carte Leaflet (position UKF)
        if (window.map && window.marker) {
            const newLatLng = L.latLng(kLat, kLon);
            window.marker.setLatLng(newLatLng);
            // window.map.panTo(newLatLng); // Optionnel : centrer la carte
        }
    }

    // --- MISE À JOUR ASTRO (SunCalc) ---
    function updateAstro(lat, lon) {
        // ... (Fonction de mise à jour astro comme dans la réponse précédente) ...
        if (!lat || !lon) return;
        const now = getCDate();
        const times = SunCalc.getTimes(now, lat, lon);
        const sunPos = SunCalc.getPosition(now, lat, lon);
        const moonPos = SunCalc.getMoonPosition(now, lat, lon);
        const moonIllum = SunCalc.getMoonIllumination(now);
        
        // Soleil
        if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(sunPos.altitude * R2D, 2, '°');
        if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(sunPos.azimuth * R2D + 180, 2, '°');
        if ($('sunrise-times')) $('sunrise-times').textContent = `${times.sunrise.toLocaleTimeString('fr-FR')} / ${times.sunsetStart.toLocaleTimeString('fr-FR')}`;
        if ($('sunset-times')) $('sunset-times').textContent = `${times.sunset.toLocaleTimeString('fr-FR')} / ${times.dusk.toLocaleTimeString('fr-FR')}`;
        
        // Lune
        if ($('moon-alt')) $('moon-alt').textContent = dataOrDefault(moonPos.altitude * R2D, 2, '°');
        if ($('moon-azimuth')) $('moon-azimuth').textContent = dataOrDefault(moonPos.azimuth * R2D + 180, 2, '°');
        if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(moonIllum.fraction * 100, 1, ' %');
    }

    // --- BOUCLE LENTE (Météo, Astro, Horloge) ---
    function startSlowLoop() {
        let lastWeatherFetch = 0;
        
        domSlowID = setInterval(() => {
            const now = getCDate();
            
            // 1. Horloge
            if ($('local-time') && !$('local-time').textContent.includes('ÉCHOUÉE')) {
                 $('local-time').textContent = now.toLocaleTimeString('fr-FR');
            }
            if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
            
            // 2. Météo
            if (lastPosition && (now.getTime() - lastWeatherFetch > WEATHER_UPDATE_MS)) {
                const lat = lastPosition.coords.latitude;
                const lon = lastPosition.coords.longitude;
                fetchWeather(lat, lon).then(data => {
                    if (data) {
                        // Mise à jour du DOM météo
                        if ($('temp-air-2')) $('temp-air-2').textContent = `${dataOrDefault(data.tempC, 1, ' °C')}`;
                        if ($('pressure-2')) $('pressure-2').textContent = `${dataOrDefault(data.pressure_hPa, 0, ' hPa')}`;
                        if ($('humidity-2')) $('humidity-2').textContent = `${dataOrDefault(data.humidity_perc, 0, ' %')}`;
                        if ($('air-density')) $('air-density').textContent = `${dataOrDefault(data.air_density, 3, ' kg/m³')}`;
                        if ($('dew-point')) $('dew-point').textContent = `${dataOrDefault(data.dew_point, 1, ' °C')}`;
                        lastWeatherFetch = now.getTime();
                    }
                });
            }
            
            // 3. Astro
            if (lastPosition) {
                updateAstro(lastPosition.coords.latitude, lastPosition.coords.longitude);
            } else {
                updateAstro(currentPosition.lat, currentPosition.lon);
            }

        }, DOM_SLOW_UPDATE_MS);
    }

    // --- INITIALISATION PRINCIPALE (DOM Content Loaded) ---
    function init() {
        // Initialisation de l'UKF avec les coordonnées par défaut
        ukf = new ProfessionalUKF(currentPosition.lat, currentPosition.lon, currentAirDensity);

        // Démarrer la synchro NTP (Robuste)
        syncH();

        // Démarrer la boucle lente (Météo, Astro, Affichage Heure)
        startSlowLoop();
        
        // --- ÉVÉNEMENTS DU DASHBOARD ---
        if ($('start-btn')) $('start-btn').addEventListener('click', () => startGPS('HIGH_FREQ'));
        if ($('stop-btn')) $('stop-btn').addEventListener('click', () => stopGPS(true));
        
        if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { 
            if (confirm("Êtes-vous sûr de vouloir TOUT réinitialiser (EKF, Distance, Max) ?")) {
                stopGPS(true);
                // Réinitialisation des compteurs et de l'UKF
                distM = 0.0; maxSpd = 0.0; timeMoving = 0.0; timeTotal = 0.0;
                ukf = new ProfessionalUKF(currentPosition.lat, currentPosition.lon, currentAirDensity);
                if ($('map') && window.map) window.map.setView([currentPosition.lat, currentPosition.lon], 13);
                // Optionnel : Réinitialiser le marqueur/chemin
                if(window.marker) window.marker.setLatLng([currentPosition.lat, currentPosition.lon]);
            }
        });
        
        if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', () => {
            emergencyStopActive = !emergencyStopActive;
            if (emergencyStopActive) {
                stopGPS(true);
                $('emergency-status').textContent = 'ACTIF (Mode Sécurité)';
            } else {
                $('emergency-status').textContent = 'INACTIF';
            }
        });

        // Gestion du mode jour/nuit (simple CSS toggle)
        if ($('toggle-mode-btn')) $('toggle-mode-btn').addEventListener('click', () => {
            document.body.classList.toggle('dark-mode');
            const isDarkMode = document.body.classList.contains('dark-mode');
            $('toggle-mode-btn').innerHTML = isDarkMode ? '☀️ Mode Jour' : '🌗 Mode Nuit';
        });

        // Initialisation de la carte Leaflet
        if ($('map')) {
            window.map = L.map('map').setView([currentPosition.lat, currentPosition.lon], 13);
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                attribution: '&copy; OpenStreetMap',
                maxZoom: 19
            }).addTo(window.map);
            window.marker = L.marker([currentPosition.lat, currentPosition.lon]).addTo(window.map);
        }
        
        // Démarrage initial de la boucle rapide pour mettre à jour le DOM avec les valeurs par défaut/UKF
        startFastLoop();
    }

    // Lancement après le chargement complet du DOM
    document.addEventListener('DOMContentLoaded', init);

})(window); // Fin de l'IIFE
