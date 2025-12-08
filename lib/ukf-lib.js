// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER CORE V6 (RÉTABLISS. DE L'UKF COMPLET)
// DÉPENDANCE CRITIQUE : La classe ProfessionalUKF DOIT ÊTRE définie dans ukf-lib.js
// =================================================================

// ⚠️ DÉPENDANCES CRITIQUES (à charger avant ce fichier dans l'HTML) :
// - math.min.js
// - lib/ukf-lib.js (DOIT contenir la classe ProfessionalUKF à 21 États)
// - lib/astro.js (si vous utilisez les fonctions astronomiques)
// =================================================================

((window) => {

    // --- BLOC 1 : CONSTANTES ET UTILITAIRES DE BASE ---

    // Vérification des dépendances critiques
    if (typeof math === 'undefined') {
        console.error("🔴 ERREUR CRITIQUE: math.js n'a pas pu être chargé. Le filtre UKF est désactivé.");
    }
    if (typeof ProfessionalUKF === 'undefined') {
        console.error("🔴 ERREUR CRITIQUE: ProfessionalUKF n'est pas définie. Vérifiez que lib/ukf-lib.js est chargé.");
    }
    
    const $ = id => document.getElementById(id);
    const dataOrDefault = (val, decimals, suffix = '') => {
        if (val === undefined || val === null || isNaN(val)) {
            return (decimals === 0 ? '0' : '0.00') + suffix;
        }
        return val.toFixed(decimals) + suffix;
    };
    const dataOrDefaultExp = (val, decimals, suffix = '') => {
        if (val === undefined || val === null || isNaN(val)) {
            return '0.00e+0' + suffix;
        }
        return val.toExponential(decimals) + suffix;
    };

    // --- CLÉS D'API & ENDPOINTS ---
    const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
    const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
    const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

    // --- CONSTANTES PHYSIQUES (Basées sur votre code complet) ---
    const D2R = Math.PI / 180, R2D = 180 / Math.PI;
    const C_L = 299792458;
    const G_U = 6.67430e-11;
    const TEMP_SEA_LEVEL_K = 288.15; // 15°C
    const RHO_SEA_LEVEL = 1.225; // kg/m³
    const BARO_ALT_REF_HPA = 1013.25;

    // --- ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE ---
    let ukf = typeof ProfessionalUKF !== 'undefined' ? new ProfessionalUKF() : null;
    let currentPosition = { lat: 43.2964, lon: 5.3697, acc: 10.0, spd: 0.0, alt: 0.0 };
    let currentMass = 70.0;
    let isGpsPaused = false;
    let isIMUActive = false;
    let totalDistance = 0.0;
    let maxSpeed = 0.0;
    let kAlt = 0.0; // Altitude filtrée (UKF)
    let gpsWatchID = null;

    // Variables de synchronisation Temps/Météo
    let lServH = null;
    let lLocH = new Date();
    let initTime = Date.now();
    let ntpSyncSuccess = false;
    let currentSpeedOfSound = 340.29; 
    let currentAirDensity = RHO_SEA_LEVEL;
    let lastKnownWeather = null;
    let isWeatherAPIFailing = true;
    let imuAccels = { x: 0, y: 0, z: 0 };
    let imuAngles = { pitch: 0, roll: 0 };
    let lastLat = currentPosition.lat;
    let lastLon = currentPosition.lon;


    // --- BLOC 2 : MODÈLES ET FONCTIONS SYSTÈME ---

    function getSpeedOfSound(T_K) {
        if (T_K <= 0 || isNaN(T_K)) return 340.29;
        return 20.0468 * Math.sqrt(T_K);
    }

    function getSchwarzschildRadius(mass_kg) {
        return (2 * G_U * mass_kg) / (C_L ** 2);
    }
    
    // Fallback NTP/Heure Locale
    async function syncH() {
        try {
            const response = await fetch(SERVER_TIME_ENDPOINT);
            const data = await response.json();
            lServH = new Date(data.utc_datetime);
            lLocH = new Date();
            ntpSyncSuccess = true;
        } catch (e) {
            console.warn("Échec de la synchro NTP. Utilisation de l'heure locale.");
            lServH = new Date();
            lLocH = new Date();
            ntpSyncSuccess = false;
        }
    }

    function getCDate(serverTime, localTimeAtSync) {
        if (!serverTime || !localTimeAtSync) return new Date();
        const diff_ms = new Date().getTime() - localTimeAtSync.getTime();
        return new Date(serverTime.getTime() + diff_ms);
    }

    async function fetchWeather(lat, lon) {
        try {
            const response = await fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`);
            if (!response.ok) throw new Error("Erreur Proxy ou API Météo");
            const data = await response.json();
            const tempK = data.tempC + 273.15;
            isWeatherAPIFailing = false;
            lastKnownWeather = {
                tempC: data.tempC,
                pressure_hPa: data.pressure_hPa,
                humidity_perc: data.humidity_perc,
                air_density: (data.pressure_hPa * 100) / (287.058 * tempK),
                tempK: tempK
            };
            return lastKnownWeather;
        } catch (e) {
            isWeatherAPIFailing = true;
            return lastKnownWeather || { // Fallback ISA
                tempC: 15.0, pressure_hPa: BARO_ALT_REF_HPA, humidity_perc: 50.0,
                air_density: RHO_SEA_LEVEL, tempK: TEMP_SEA_LEVEL_K
            };
        }
    }

    // --- BLOC 3 : FUSION GNSS / UKF / IMU ---

    // 🌟 CORRECTION CRITIQUE : Paramètres GPS plus TOLÉRANTS (V5)
    const GPS_OPTS = {
        enableHighAccuracy: true,
        maximumAge: 3000,     // Accepte une position de 3 secondes max (plus stable)
        timeout: 15000        // Donne 15 secondes pour trouver une position (moins de timeouts)
    };

    function initGPS() {
        if (gpsWatchID) navigator.geolocation.clearWatch(gpsWatchID);
        let lastTimestamp = 0;

        gpsWatchID = navigator.geolocation.watchPosition(
            (position) => {
                if (isGpsPaused) return;

                const timestamp = position.timestamp;
                const dt = (timestamp - lastTimestamp) / 1000 || 0;
                lastTimestamp = timestamp;

                const gpsData = {
                    lat: position.coords.latitude,
                    lon: position.coords.longitude,
                    alt: position.coords.altitude || 0.0,
                    speed: position.coords.speed || 0.0,
                    acc: position.coords.accuracy,
                    bearing: position.coords.heading || 0.0, // Utiliser le heading comme bearing
                    timestamp: timestamp
                };

                // 1. UKF Predict / Update (Dépend de ukf-lib.js)
                if (ukf) {
                    if (dt > 0 && ukf.x) {
                        ukf.predict(dt); 
                    }
                    ukf.update(gpsData);

                    const ukfState = ukf.getState();
                    currentPosition.lat = ukfState.lat;
                    currentPosition.lon = ukfState.lon;
                    currentPosition.alt = ukfState.alt;
                    currentPosition.spd = ukfState.speed;
                    currentPosition.acc = ukfState.kUncert; // Incertitude EKF/UKF

                } else {
                    // Fallback sur données brutes
                    currentPosition.lat = gpsData.lat;
                    currentPosition.lon = gpsData.lon;
                    currentPosition.alt = gpsData.alt;
                    currentPosition.spd = gpsData.speed;
                    currentPosition.acc = gpsData.acc;
                }
                
                kAlt = currentPosition.alt;

                // 2. Mise à jour de la Distance Totale
                if (lastLat !== null && lastLon !== null && currentPosition.spd > 0.1) {
                    // Calcul de la distance 3D simple (Turf.js serait préférable ici)
                    // Simplifié pour ne pas dépendre de Turf dans ce fichier core
                    const R_EARTH = 6371000;
                    const dLat = (currentPosition.lat - lastLat) * D2R;
                    const dLon = (currentPosition.lon - lastLon) * D2R;
                    const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) + 
                              Math.cos(lastLat * D2R) * Math.cos(currentPosition.lat * D2R) * Math.sin(dLon / 2) * Math.sin(dLon / 2);
                    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
                    const distance2D = R_EARTH * c;
                    const distance3D = Math.sqrt(distance2D**2 + (currentPosition.alt - kAlt)**2);
                    totalDistance += distance3D; 
                }
                lastLat = currentPosition.lat;
                lastLon = currentPosition.lon;

                // 3. Mise à jour Vitesse Max
                const speedKmh = currentPosition.spd * 3.6;
                if (speedKmh > maxSpeed) maxSpeed = speedKmh;

                // 4. Mise à jour Météo (toutes les 10s)
                if (Date.now() % 10000 < 500) {
                    fetchWeather(currentPosition.lat, currentPosition.lon).then(data => {
                        currentAirDensity = data.air_density;
                        currentSpeedOfSound = getSpeedOfSound(data.tempK);
                    });
                }
            },
            (error) => {
                console.error("Erreur GPS:", error);
                if ($('gps-status-indicator')) $('gps-status-indicator').textContent = `ERREUR GPS (${error.code}) 🔴`;
                currentPosition.spd = 0.0;
            },
            GPS_OPTS
        );
    }

    function handleDeviceMotion(event) {
        // Accélération linéaire (Accéléromètre pur, sans gravité)
        if (event.acceleration) {
            imuAccels.x = event.acceleration.x || 0;
            imuAccels.y = event.acceleration.y || 0;
            imuAccels.z = event.acceleration.z || 0;
        }

        // Pitch/Roll (Calculé à partir de l'accélération + gravité)
        if (event.accelerationIncludingGravity) {
            const ax = event.accelerationIncludingGravity.x;
            const ay = event.accelerationIncludingGravity.y;
            const az = event.accelerationIncludingGravity.z;
            imuAngles.pitch = Math.atan2(ax, Math.sqrt(ay*ay + az*az)) * R2D;
            imuAngles.roll = Math.atan2(ay, Math.sqrt(ax*ax + az*az)) * R2D;
        }
        
        // Gyroscope
        if (event.rotationRate && ukf) {
            // Logique d'intégration Gyro dans l'UKF (dans ukf-lib.js:predict)
            // L'UKF gère le bruit de mesure (R) pour le Gyro/Accel
        }
    }

    // 🌟 CORRECTION CRITIQUE IMU : Gestion des permissions mobiles (V5)
    function activateDeviceMotion() {
        const statusEl = $('imu-status');
        if (typeof DeviceOrientationEvent !== 'undefined' && typeof DeviceOrientationEvent.requestPermission === 'function') {
            DeviceOrientationEvent.requestPermission()
                .then(permissionState => {
                    if (permissionState === 'granted') {
                        window.addEventListener('devicemotion', handleDeviceMotion, true);
                        isIMUActive = true;
                        if (statusEl) statusEl.textContent = 'Actif 🟢';
                    } else {
                        if (statusEl) statusEl.textContent = 'Refusé 🔴 (Permission requise)';
                    }
                })
                .catch(e => {
                    if (statusEl) statusEl.textContent = 'Erreur 🔴 (API non fonctionnelle)';
                });
        } else if (typeof window.DeviceMotionEvent !== 'undefined') {
            window.addEventListener('devicemotion', handleDeviceMotion, true);
            isIMUActive = true;
            if (statusEl) statusEl.textContent = 'Actif 🟢';
        } else {
            if (statusEl) statusEl.textContent = 'Non supporté 🔴';
        }
    }


    // --- BLOC 4 : MISE À JOUR DU DOM (Affichage) ---

    function updateDashboardDOM() {
        // ... (Calculs physiques et relativistes)
        const speed3D = currentPosition.spd; 
        const speedKmh = speed3D * 3.6;
        const mach = speed3D / currentSpeedOfSound;
        const lightPerc = (speed3D / C_L) * 100;
        const lorentzFactor = 1 / Math.sqrt(1 - (speed3D / C_L) ** 2);
        const restMassEnergy = currentMass * C_L ** 2; 
        const schwarzschildRadius = getSchwarzschildRadius(currentMass);
        const elapsedTime = (Date.now() - initTime) / 1000;
        const ukfState = ukf ? ukf.getState() : currentPosition; // Utilise l'état UKF complet

        // 1. TEMPS ET DATE (avec Fallback)
        const now = getCDate(lServH, lLocH);
        if ($('temps-ecoule-session')) $('temps-ecoule-session').textContent = `${dataOrDefault(elapsedTime, 2)} s`;
        if ($('local-time-ntp')) {
            let ntpText = now.toLocaleTimeString('fr-FR');
            if (!ntpSyncSuccess) ntpText += ' (Local)';
            $('local-time-ntp').textContent = ntpText;
        }
        if ($('date-utc-gmt')) {
             let dateText = now.toLocaleDateString('fr-FR') + ' ' + now.toLocaleTimeString('fr-FR');
             dateText += ntpSyncSuccess ? ' UTC' : ' LOCAL (Échec UTC)';
             $('date-utc-gmt').textContent = dateText;
        }
        
        // 2. MÉTÉO (avec Fallback ISA)
        let weatherData = lastKnownWeather;
        if (isWeatherAPIFailing && !weatherData) {
            weatherData = { tempC: 15.0, pressure_hPa: BARO_ALT_REF_HPA, air_density: RHO_SEA_LEVEL };
            if ($('weather-status')) $('weather-status').textContent = 'API ÉCHOUÉE (ISA) 🟡';
        } else if (!isWeatherAPIFailing) {
            if ($('weather-status')) $('weather-status').textContent = 'Actif 🟢';
        }
        if ($('temp-air')) $('temp-air').textContent = dataOrDefault(weatherData ? weatherData.tempC : null, 2, ' °C');
        if ($('pressure-atm')) $('pressure-atm').textContent = dataOrDefault(weatherData ? weatherData.pressure_hPa : null, 0, ' hPa');
        if ($('air-density-rho')) $('air-density-rho').textContent = dataOrDefault(weatherData ? weatherData.air_density : null, 3, ' kg/m³');


        // 3. VITESSE/RELATIVITÉ
        if ($('vitesse-son-locale')) $('vitesse-son-locale').textContent = `${dataOrDefault(currentSpeedOfSound, 4)} m/s`;
        if ($('schwarzschild-radius')) $('schwarzschild-radius').textContent = dataOrDefaultExp(schwarzschildRadius, 4, ' m');
        if ($('vitesse-inst')) $('vitesse-inst').textContent = `${dataOrDefault(speedKmh, 1)} km/h`;
        if ($('vitesse-brute-ms')) $('vitesse-brute-ms').textContent = `${dataOrDefault(speed3D, 2)} m/s`;
        if ($('vitesse-max-session')) $('vitesse-max-session').textContent = `${dataOrDefault(maxSpeed, 1)} km/h`;
        if ($('mach-number')) $('mach-number').textContent = dataOrDefault(mach, 4);
        if ($('perc-speed-light')) $('perc-speed-light').textContent = dataOrDefaultExp(lightPerc, 2, ' %');
        if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(lorentzFactor, 4);
        if ($('energie-masse-repos')) $('energie-masse-repos').textContent = dataOrDefaultExp(restMassEnergy, 4, ' J');
        if ($('dist-total-3d')) $('dist-total-3d').textContent = `${dataOrDefault(totalDistance / 1000, 3)} km | ${dataOrDefault(totalDistance, 2)} m`;
        
        // 4. POSITION EKF/UKF
        if ($('lat-ekf')) $('lat-ekf').textContent = dataOrDefault(ukfState.lat, 6);
        if ($('lon-ekf')) $('lon-ekf').textContent = dataOrDefault(ukfState.lon, 6);
        if ($('alt-ekf')) $('alt-ekf').textContent = dataOrDefault(ukfState.alt, 2) + ' m';
        if ($('gps-accuracy')) $('gps-accuracy').textContent = dataOrDefault(currentPosition.acc, 2) + ' m';
        
        // 5. IMU
        if ($('accel-x')) $('accel-x').textContent = dataOrDefault(imuAccels.x, 2) + ' m/s²';
        if ($('accel-y')) $('accel-y').textContent = dataOrDefault(imuAccels.y, 2) + ' m/s²';
        if ($('accel-z')) $('accel-z').textContent = dataOrDefault(imuAccels.z, 2) + ' m/s²';
        if ($('pitch-imu')) $('pitch-imu').textContent = dataOrDefault(imuAngles.pitch, 1) + '°';
        if ($('roll-imu')) $('roll-imu').textContent = dataOrDefault(imuAngles.roll, 1) + '°';
        if ($('imu-status')) $('imu-status').textContent = isIMUActive ? 'Actif 🟢' : 'Inactif';

        // 6. Statut UKF/GPS
        if ($('ukf-status')) $('ukf-status').textContent = ukf ? 'Actif' : 'UKF N/A 🔴';
        if ($('ukf-uncertainty')) $('ukf-uncertainty').textContent = dataOrDefault(ukfState.kUncert, 4);
    }


    // --- BLOC 5 : CONTRÔLES ET INITIALISATION ---
    
    function toggleGpsPause() {
        isGpsPaused = !isGpsPaused;
        const btn = $('toggle-gps-btn');
        if (btn) btn.innerHTML = isGpsPaused ? '▶️ MARCHE GPS' : '⏸️ PAUSE GPS';
        
        if (isGpsPaused) {
            if (gpsWatchID) navigator.geolocation.clearWatch(gpsWatchID);
            gpsWatchID = null;
            if ($('gps-status-indicator')) $('gps-status-indicator').textContent = 'PAUSE ⏸️';
        } else {
            initGPS();
            if ($('gps-status-indicator')) $('gps-status-indicator').textContent = 'Recherche... 🟡';
        }
    }
    
    // Ajout d'une fonction d'initialisation des événements (basé sur le fichier complet)
    function setupEventListeners() {
        // Boutons de contrôle
        if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', toggleGpsPause);
        if ($('activate-sensors-btn')) $('activate-sensors-btn').addEventListener('click', activateDeviceMotion);
        if ($('full-reset-btn')) $('full-reset-btn').addEventListener('click', () => { 
            if(confirm("Voulez-vous vraiment TOUT RÉINITIALISER ?")) { location.reload(); }
        });
        
        // Initialisation de l'affichage du bouton de pause
        if ($('toggle-gps-btn') && isGpsPaused === false) $('toggle-gps-btn').innerHTML = '⏸️ PAUSE GPS';
        
        // Événements de configuration (Gravité, Masse, etc. - si implémentés)
        // Note: Laissez les EventListeners de vos fichiers complets pour ces éléments.
    }


    window.addEventListener('load', () => {
        
        // 1. Initialisation des systèmes critiques
        syncH(); // Démarrer la synchro NTP
        initGPS(); // Démarrer le GPS avec options stables
        setupEventListeners(); // Attacher les contrôles

        // 2. Premier rafraîchissement des valeurs de Fallback
        updateDashboardDOM();

        // 3. Boucle principale de rafraîchissement
        setInterval(updateDashboardDOM, 250);
    });

})(window);
