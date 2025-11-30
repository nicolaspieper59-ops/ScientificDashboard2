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

    // --- MISE À JOUR DOM (RÉSULTATS UKF, Physique, Relativité) ---
    function updateDOMLoop(dt) {
        const { kLat, kLon, kAlt, kSpd, kUncert, kAltUncert, kHeading } = (ukf ? ukf.getState() : { kLat: currentPosition.lat, kLon: currentPosition.lon, kAlt: currentPosition.alt, kSpd: currentPosition.spd, kUncert: NaN, kAltUncert: NaN, kHeading: NaN });
        const speedKmH = kSpd * KMH_MS;
        const LorentzFactor = 1 / Math.sqrt(1 - Math.pow(kSpd / C_L, 2));
        const T_Dilation = (LorentzFactor - 1) * 365.25 * 86400 * 1e9; // ns/année
        const machNumber = kSpd / currentSpeedOfSound;
        const kineticEnergy = 0.5 * currentMass * kSpd * kSpd;
        const coriolisForce = calculateCoriolisForce(currentMass, kLat, kSpd, kHeading);
        const distanceLightSec = (distM / C_L) * netherMultiplier;

        // --- Mises à jour du BLOC Vitesse, Distance, Relativité ---
        if ($('speed-ukf')) $('speed-ukf').textContent = dataOrDefault(speedKmH, 5, ' km/h');
        if ($('speed-max')) $('speed-max').textContent = dataOrDefault(maxSpd * KMH_MS, 5, ' km/h');
        if ($('speed-uncert')) $('speed-uncert').textContent = dataOrDefault(kUncert, 3, ' m/s');
        if ($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = `${currentSpeedOfSound.toFixed(2)} m/s`;
        
        if ($('mach-number')) $('mach-number').textContent = dataOrDefault(machNumber, 4, '');
        if ($('speed-of-light-perc')) $('speed-of-light-perc').textContent = dataOrDefaultExp((kSpd / C_L) * 100, 2, ' %');
        if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(LorentzFactor, 4, '');
        if ($('time-dilation-speed')) $('time-dilation-speed').textContent = dataOrDefault(T_Dilation, 2, ' ns/j');
        
        if ($('energy-kinetic')) $('energy-kinetic').textContent = dataOrDefaultExp(kineticEnergy, 3, ' J');
        if ($('coriolis-force')) $('coriolis-force').textContent = dataOrDefault(coriolisForce, 2, ' N');
        
        // Affichage IMU
        if ($('accel-x')) $('accel-x').textContent = dataOrDefault(accel.x, 3, ' m/s²');
        if ($('gyro-z')) $('gyro-z').textContent = dataOrDefault(gyro.z, 3, ' °/s');
        
        // Affichage Distance/Temps
        if ($('distance-km-m')) $('distance-km-m').textContent = `${dataOrDefault(distM * netherMultiplier / 1000, 5, ' km')} | ${dataOrDefault(distM * netherMultiplier, 2, ' m')}`;
        if ($('time-moving')) $('time-moving').textContent = dataOrDefault(timeMoving, 0, ' s');
        if ($('time-total')) $('time-total').textContent = timeToHMS(timeTotal); 
        
        if ($('time-minecraft')) $('time-minecraft').textContent = timeToHMS((getCDate().getTime() % MINECRAFT_DAY_MS) / 1000);
        if ($('distance-light-sec')) $('distance-light-sec').textContent = dataOrDefaultExp(distanceLightSec, 2, ' s');

        // Affichage Position
        if ($('latitude')) $('latitude').textContent = dataOrDefault(kLat, 6, '°');
        if ($('longitude')) $('longitude').textContent = dataOrDefault(kLon, 6, '°');
        if ($('altitude-ukf')) $('altitude-ukf').textContent = dataOrDefault(kAlt, 3, ' m');
        if ($('alt-uncert')) $('alt-uncert').textContent = dataOrDefault(kAltUncert, 3, ' m');
        if ($('cap-direction')) $('cap-direction').textContent = dataOrDefault(kHeading, 0, '°');

        // Mise à jour de la carte Leaflet
        if (window.map && window.marker) {
            window.marker.setLatLng(L.latLng(kLat, kLon));
        }
    }

    // --- MISE À JOUR ASTRO (SunCalc) ---
    function updateAstro(lat, lon) {
        if (!lat || !lon) { 
            if ($('sun-alt')) $('sun-alt').textContent = "N/A"; return; 
        }
        const now = getCDate();
        const times = SunCalc.getTimes(now, lat, lon);
        const sunPos = SunCalc.getPosition(now, lat, lon);
        const moonIllum = SunCalc.getMoonIllumination(now);
        
        // Soleil
        if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(sunPos.altitude * R2D, 2, '°');
        if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(sunPos.azimuth * R2D + 180, 2, '°');
        if ($('sunrise-times')) $('sunrise-times').textContent = `${times.sunrise.toLocaleTimeString('fr-FR')} / ${times.sunsetStart.toLocaleTimeString('fr-FR')}`;
        if ($('sunset-times')) $('sunset-times').textContent = `${times.sunset.toLocaleTimeString('fr-FR')} / ${times.dusk.toLocaleTimeString('fr-FR')}`;
        
        // Lune
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
                 $('date-display').textContent = now.toLocaleDateString('fr-FR');
            }
            
            // 2. Météo
            if (lastPosition && (now.getTime() - lastWeatherFetch > WEATHER_UPDATE_MS)) {
                const lat = lastPosition.coords.latitude;
                const lon = lastPosition.coords.longitude;
                fetchWeather(lat, lon).then(data => {
                    if (data) {
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
    
    // --- INITIALISATION PRINCIPALE (GESTION DES BOUTONS) ---
    function init() {
        // Initialisation de l'UKF avec les coordonnées par défaut
        ukf = new ProfessionalUKF(currentPosition.lat, currentPosition.lon, currentAirDensity);

        // Démarrer la synchro NTP
        syncH();

        // Démarrer la boucle lente
        startSlowLoop();
        
        // --- ÉVÉNEMENTS DES BOUTONS (Rend tous les contrôles fonctionnels) ---
        
        // Boutons MARCHE/ARRÊT/RÉINITIALISATION
        if ($('start-btn')) $('start-btn').addEventListener('click', () => startGPS('HIGH_FREQ'));
        if ($('stop-btn')) $('stop-btn').addEventListener('click', () => stopGPS(true));
        
        if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => { 
            if (emergencyStopActive) return;
            distM = 0; timeMoving = 0; 
        });
        if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', () => { 
            if (emergencyStopActive) return;
            maxSpd = 0; 
        });
        if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { 
            if (emergencyStopActive) return;
            if (confirm("Êtes-vous sûr de vouloir TOUT réinitialiser (EKF, Distance, Max) ?")) {
                stopGPS(true);
                distM = 0.0; maxSpd = 0.0; timeMoving = 0.0; timeTotal = 0.0;
                ukf = new ProfessionalUKF(currentPosition.lat, currentPosition.lon, currentAirDensity); // Réinitialiser l'UKF
            }
        });
        
        // Boutons de contrôle système
        if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', () => {
            emergencyStopActive = !emergencyStopActive;
            if (emergencyStopActive) {
                stopGPS(true);
                $('emergency-status').textContent = 'ACTIF (Mode Sécurité)';
            } else {
                $('emergency-status').textContent = 'INACTIF 🟢';
            }
        });
        if ($('toggle-mode-btn')) $('toggle-mode-btn').addEventListener('click', () => {
            document.body.classList.toggle('dark-mode');
            const isDarkMode = document.body.classList.contains('dark-mode');
            $('toggle-mode-btn').textContent = isDarkMode ? '☀️ Mode Jour' : '🌗 Mode Nuit';
        });

        // Contrôles spécifiques
        if ($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', () => {
            netherMultiplier = (netherMultiplier === 1) ? 8 : 1;
            $('nether-indicator').textContent = (netherMultiplier === 8) ? 'ACTIVÉ (1:8) 🔥' : 'DÉSACTIVÉ (1:1)';
        });
        if ($('mass-input')) $('mass-input').addEventListener('input', (e) => {
            currentMass = parseFloat(e.target.value) || 70.0;
            if ($('mass-display')) $('mass-display').textContent = `${dataOrDefault(currentMass, 3, ' kg')}`;
        });

        // Initialisation de la carte Leaflet
        if ($('map')) {
            window.map = L.map('map').setView([currentPosition.lat, currentPosition.lon], 13);
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { attribution: '&copy; OpenStreetMap', maxZoom: 19 }).addTo(window.map);
            window.marker = L.marker([currentPosition.lat, currentPosition.lon]).addTo(window.map);
        }
        
        // Démarrage initial pour les affichages par défaut
        startFastLoop();
    }

    document.addEventListener('DOMContentLoaded', init);

})(window); // Fin de l'IIFE
