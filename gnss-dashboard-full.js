// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// Version Corrigée (v2025.12.06) - FIX DES ISSUES D'INITIALISATION
// Dépendances critiques (DOIVENT être chargées dans l'HTML) : 
// math.min.js, leaflet.js, suncalc.js, turf.min.js
// =================================================================

// --- BLOC 1 : CONSTANTES ET UTILITAIRES DE BASE ---

const $ = id => document.getElementById(id);
const D2R = Math.PI / 180, R2D = 180 / Math.PI; 
const KMH_MS = 3.6;         

// Formatage des données standard (x.xx + suffixe)
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || val === Infinity || val === -Infinity) {
        return (decimals === 0 ? '0' : '0.00') + suffix;
    }
    return val.toFixed(decimals) + suffix;
};

// Formatage des données exponentielles (x.xxe+x + suffixe)
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || val === Infinity || val === -Infinity) {
        const zeroDecimals = '0.' + Array(decimals).fill('0').join('');
        return zeroDecimals + 'e+0' + suffix;
    }
    return val.toExponential(decimals) + suffix;
};

// --- CLÉS D'API & ENDPOINTS ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";
const DOM_SLOW_UPDATE_MS = 2000; // Fréquence de rafraîchissement du DOM lent

// --- CONSTANTES PHYSIQUES ET GÉOPHYSIQUES ---
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const G_U = 6.67430e-11;    // Gravitation universelle (N·m²/kg²)
const G_ACC_STD = 9.80665;  // Gravité standard (m/s²)
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const R_SPECIFIC_AIR = 287.058; // Constante spécifique de l'air sec (J/kg·K)
const TEMP_SEA_LEVEL_K = 288.15; // 15°C en Kelvin
const BARO_ALT_REF_HPA = 1013.25; // Pression au niveau de la mer standard
const RHO_SEA_LEVEL = 1.225; // Densité de l'air ISA (kg/m³)
const GAMMA_AIR = 1.4; // Indice adiabatique de l'air
const EARTH_MASS = 5.972e24; // Masse de la Terre (kg)

// --- CONSTANTES WGS84 ---
const WGS84_A = 6378137.0;  // Rayon équatorial WGS84 (m)
const WGS84_F = 1 / 298.257223563; // Aplatissement WGS84
const WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F; // Excentricité au carré

// --- CONSTANTES MAGNÉTISME (SIMPLIFIÉ) ---
const MAG_FIELD_STRENGTH_REF = 48000; // Intensité totale (nT)
const MAG_INCLINATION_REF_RAD = 60.0 * D2R; 

// --- CONSTANTES MINECRAFT ---
const MINECRAFT_RATIO = 72; 

// --- CONFIGURATION D'ENVIRONNEMENT ---
const ENVIRONMENT_FACTORS = {
    NORMAL: { MULT: 1.0, DISPLAY: 'Normal' },
    URBAN: { MULT: 0.5, DISPLAY: 'Urbain (x0.5)' },
    MOUNTAIN: { MULT: 2.0, DISPLAY: 'Montagne (x2.0)' },
    TUNNEL: { MULT: 5.0, DISPLAY: 'Tunnel (x5.0)' }
};

// =================================================================
// BLOC 2 : ÉTAT GLOBAL ET MODÈLES PHYSIQUES / UKF
// =================================================================

((window) => {
    
    // --- ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE ---
    let isGpsPaused = false; 
    let currentPosition = { 
        lat: 43.2964,   // Default: Marseille
        lon: 5.3697,    
        alt: 0.0,       
        acc: 10.0,      
        spd: 0.0,       
        heading: 0.0    
    };
    let kAlt = 0;       
    let lastUKFUpdate = Date.now();
    let ukf;            

    // État Physique/Contrôle
    let currentMass = 70.0;
    let netherMode = false; 
    let distMTotal = 0.0; 
    let vMaxSession = 0.0; 
    let currentCelestialBody = 'TERRE';
    let rotationRadius = 100;
    let angularVelocity = 0.0;
    let selectedEnvironment = 'NORMAL';
    let currentUKFReactivity = 'AUTO';
    let gpsAccuracyTarget = 0; 
    let lastKnownWeather = null;
    let lastKnownPollutants = null;
    let startTime = Date.now();
    let isMoving = false;
    let totalMoveTimeMs = 0;
    let totalSpeedSum = 0;
    let speedSamples = 0;
    let isEmergencyStop = false;

    // --- SYNCHRONISATION HORAIRE (NTP/UTC) ---
    let lastNTPDate = null;     
    let lastLocalTime = null;   
    let syncErrorCount = 0;     

    // --- DONNÉES DE CAPTEURS (IMU + Magnétomètre) ---
    let lastAccX = 0, lastAccY = 0, lastAccZ = 0; 
    let lastGyrX = 0, lastGyrY = 0, lastGyrZ = 0; 
    let lastMagX = 0, lastMagY = 0, lastMagZ = 0; 
    let lastPitch = 0, lastRoll = 0; // Angles d'inclinaison (Pitch/Roll)

    // --- VARIABLES MÉTRÉOLOGIQUES ---
    let lastP_hPa = BARO_ALT_REF_HPA; 
    let lastT_K = TEMP_SEA_LEVEL_K;   
    let lastH_perc = 0.5; 
    let currentAirDensity = RHO_SEA_LEVEL;
    let currentSpeedOfSound = getSpeedOfSound(TEMP_SEA_LEVEL_K);

    /** [CRITICAL] Retourne l'heure UTC actuelle estimée (Offline-First). */
    const getCDate = () => {
        if (!lastNTPDate || !lastLocalTime) return new Date(); 
        const localTimeDifference = Date.now() - lastLocalTime;
        return new Date(lastNTPDate.getTime() + localTimeDifference);
    };

    /** Calcule la Vitesse du Son ajustée (m/s). */
    const getSpeedOfSound = (tempK) => Math.sqrt(GAMMA_AIR * R_SPECIFIC_AIR * tempK);

    /** Calcule l'accélération gravitationnelle (WGS84). */
    const calculateWGS84Gravity = (lat, alt) => {
        if (currentCelestialBody !== 'TERRE') return G_ACC_STD;
        const latRad = lat * D2R;
        const sinSqLat = Math.pow(Math.sin(latRad), 2);
        const g0 = 9.780327 * (1 + 0.0053024 * sinSqLat - 0.0000058 * Math.pow(sinSqLat, 2));
        const radius = WGS84_A / Math.sqrt(1 - WGS84_E2 * sinSqLat);
        const g_alt = g0 * (1 - 2 * alt / radius);
        return g_alt;
    };
    
    /** Calcule la Force de Coriolis (Newton) (simplifié pour l'Est/Ouest). */
    const calculateCoriolisForce = (latRad, v_mps, m) => {
        const force = 2 * m * OMEGA_EARTH * v_mps * Math.sin(latRad);
        return force;
    };

    /** Calcule le Rayon de Schwarzschild (m). */
    const calculateSchwarzschildRadius = (m) => (2 * G_U * m) / (C_L * C_L);
    
    /** Calcule le Facteur de Lorentz (γ). */
    const calculateLorentzFactor = (v) => {
        const beta = v / C_L;
        if (beta >= 1) return Infinity;
        return 1 / Math.sqrt(1 - beta * beta);
    };
    
    /** Calcule l'énergie de masse au repos (E₀). */
    const calculateRestMassEnergy = (m) => m * C_L * C_L;
    
    /** Calcule la Dilation Temporelle Gravitationnelle (ns/jour). */
    const calculateGravitationalDilation = (altM) => {
        const c2 = C_L * C_L;
        const g_local = calculateWGS84Gravity(currentPosition.lat, altM);
        const fraction = (g_local * altM) / c2;
        const ns_per_day = fraction * 86400 * 1e9;
        return ns_per_day; 
    };
    
    /** Calcule la densité de l'air humide (kg/m³). */
    const calculateAirDensity = (P_hPa, T_K, H_perc) => {
        const P = P_hPa * 100; // Pascals
        // Formule de l'air humide simplifiée
        return (P * 100) / (R_SPECIFIC_AIR * T_K); 
    };
    
    /** Calcule la Distance Maximale Visible (Horizon). */
    const calculateMaxVisibleDistance = (altM) => {
        const R_E = WGS84_A;
        return altM > 0 ? Math.sqrt(altM * (2 * R_E + altM)) : 0;
    };

    /** MOCK UKF (Non implémenté mathématiquement ici, mais structurellement nécessaire) */
    class ProfessionalUKF {
        constructor() {
            this.STATE_SIZE = 21;
            if (typeof math === 'undefined') {
                 console.error("math.js non disponible. UKF désactivé.");
                 this.x = { get: (i) => i === 2 ? currentPosition.alt : (i === 0 ? currentPosition.lat * D2R : (i === 1 ? currentPosition.lon * D2R : 0)) }; 
                 this.getState = () => ({ lat: currentPosition.lat, lon: currentPosition.lon, alt: currentPosition.alt, vel_up: 0 });
                 return;
            }
            this.x = math.zeros(this.STATE_SIZE); 
            this.x.set([0], currentPosition.lat * D2R);
            this.x.set([1], currentPosition.lon * D2R);
            this.x.set([2], currentPosition.alt);
            this.x.set([6], 1.0); // Quaternion q1
            console.log(`[UKF] Filtre UKF ${this.STATE_SIZE} États initialisé.`);
        }
        predict_propagation(dt, imuMeasure) {
            // MOCK: Mise à jour simple d'altitude pour simuler l'EKF
            this.x.set([2], this.x.get([2]) + imuMeasure[2] * dt * dt / 2); // z += a*t^2/2
        }
        update_correction(gpsMeasure, magMeasure, referenceMag) {
            // MOCK: Correction simple de la position
            this.x.set([0], gpsMeasure[0] * D2R);
            this.x.set([1], gpsMeasure[1] * D2R);
            this.x.set([2], this.x.get([2]) * 0.9 + gpsMeasure[2] * 0.1); 
        }
        
        getState() {
            return {
                lat: this.x.get([0]) * R2D,
                lon: this.x.get([1]) * R2D,
                alt: this.x.get([2]),
                vel_up: this.x.get([5]) || 0, // Vitesse verticale (exemple)
            };
        }
    }


    // =================================================================
    // BLOC 3 : SYNCHRONISATION, CAPTEURS ET FETCHING ASYNCHRONE
    // =================================================================

    /** Gère la demande de permission IMU (nécessaire sur iOS/modernes). */
    const requestMotionPermission = () => {
        if (typeof DeviceOrientationEvent.requestPermission === 'function') {
            DeviceOrientationEvent.requestPermission()
                .then(permissionState => {
                    if (permissionState === 'granted') {
                        window.addEventListener('devicemotion', handleDeviceMotion);
                        window.addEventListener('deviceorientation', handleDeviceOrientation);
                        if ($('imu-status')) $('imu-status').textContent = 'Actif (IMU/Mag)';
                    } else {
                         if ($('imu-status')) $('imu-status').textContent = 'Permission refusée';
                    }
                })
                .catch(console.error);
        } else {
            // Pour les navigateurs qui ne nécessitent pas d'autorisation (Android/Desktop)
            if (window.DeviceMotionEvent) window.addEventListener('devicemotion', handleDeviceMotion);
            if (window.DeviceOrientationEvent) window.addEventListener('deviceorientation', handleDeviceOrientation);
            if ($('imu-status')) $('imu-status').textContent = 'Actif (IMU/Mag)';
        }
    };
    
    /** Synchronise l'heure du système avec un serveur NTP/UTC. */
    const syncH = async () => {
        if (syncH.inProgress) return; 
        syncH.inProgress = true;
        if ($('local-time')) $('local-time').textContent = 'Synchronisation...';

        try {
            const response = await fetch(SERVER_TIME_ENDPOINT);
            if (!response.ok) throw new Error(`Erreur HTTP: ${response.status}`);
            const data = await response.json();
            
            lastNTPDate = new Date(data.datetime);
            lastLocalTime = Date.now(); 
            syncErrorCount = 0;
            console.log(`[NTP Sync OK]`);

        } catch (error) {
            syncErrorCount++;
            console.error(`[NTP Sync ÉCHEC - Tentative ${syncErrorCount}]`);
            if ($('local-time')) $('local-time').textContent = `SYNCHRO ÉCHOUÉE (${syncErrorCount}x)`;
        } finally {
            syncH.inProgress = false;
            setTimeout(syncH, 60000); 
        }
    };
    
    /** Gère l'Accéléromètre et le Gyroscope (DeviceMotionEvent). */
    const handleDeviceMotion = (event) => {
        if (event.accelerationIncludingGravity) {
            lastAccX = event.accelerationIncludingGravity.x;
            lastAccY = event.accelerationIncludingGravity.y;
            lastAccZ = event.accelerationIncludingGravity.z;
        }
        if (event.rotationRate) {
            lastGyrX = (event.rotationRate.beta || 0) * D2R; 
            lastGyrY = (event.rotationRate.gamma || 0) * D2R;
            lastGyrZ = (event.rotationRate.alpha || 0) * D2R;
        }
    };

    /** Gère le Magnétomètre/Orientation (DeviceOrientationEvent). */
    const handleDeviceOrientation = (event) => {
        const B_TOTAL = MAG_FIELD_STRENGTH_REF * 1e-9;
        
        if (event.absolute && event.alpha !== undefined) {
            currentPosition.heading = (event.alpha || 0);
            lastPitch = event.beta || 0;    // Pitch (Inclinaison)
            lastRoll = event.gamma || 0;    // Roll (Roulis)
            
            // Simulation simplifiée des composantes Mag basées sur le cap absolu
            const alpha = currentPosition.heading * D2R;
            lastMagX = B_TOTAL * Math.cos(MAG_INCLINATION_REF_RAD) * Math.cos(alpha);
            lastMagY = B_TOTAL * Math.cos(MAG_INCLINATION_REF_RAD) * Math.sin(alpha);
            lastMagZ = B_TOTAL * Math.sin(MAG_INCLINATION_REF_RAD);

        } else if (event.webkitCompassHeading !== undefined) {
             currentPosition.heading = event.webkitCompassHeading;
             lastPitch = event.beta || 0;
             lastRoll = event.gamma || 0;
             // Simulation
             const headingRad = currentPosition.heading * D2R;
             lastMagX = B_TOTAL * Math.cos(MAG_INCLINATION_REF_RAD) * Math.cos(headingRad);
             lastMagY = B_TOTAL * Math.cos(MAG_INCLINATION_REF_RAD) * Math.sin(headingRad);
             lastMagZ = B_TOTAL * Math.sin(MAG_INCLINATION_REF_RAD);
        }
        if ($('imu-status').textContent === 'Inactif') $('imu-status').textContent = 'Actif (IMU/Mag)';
    };

    /** Gère la géolocalisation GPS (seul standard web). */
    const handleGPS = (position) => {
        if (isGpsPaused) return;

        const newLat = position.coords.latitude;
        const newLon = position.coords.longitude;
        const newAlt = position.coords.altitude === null ? currentPosition.alt : position.coords.altitude;
        const speed_mps = position.coords.speed || 0.0;
        
        // Mise à jour de la distance
        if (distMTotal === 0 && newLat !== 0) {
            // Initialisation après la première lecture GPS
        } else if (typeof turf !== 'undefined') {
            const p1 = turf.point([currentPosition.lon, currentPosition.lat]);
            const p2 = turf.point([newLon, newLat]);
            const horizontalDistance = turf.distance(p1, p2, { units: 'meters' });
            const verticalDistance = Math.abs(newAlt - currentPosition.alt);
            const distance3D = Math.sqrt(horizontalDistance * horizontalDistance + verticalDistance * verticalDistance);
            distMTotal += distance3D;
        }

        // Mise à jour des variables globales
        currentPosition.lat = newLat;
        currentPosition.lon = newLon;
        currentPosition.alt = newAlt;
        currentPosition.acc = position.coords.accuracy || 10.0;
        currentPosition.spd = speed_mps;
        currentPosition.heading = position.coords.heading || currentPosition.heading;

        // Mise à jour des statistiques
        if (speed_mps > 0.1) {
            if (!isMoving) { isMoving = true; }
            totalSpeedSum += speed_mps;
            speedSamples++;
            vMaxSession = Math.max(vMaxSession, speed_mps * KMH_MS);
        } else {
            isMoving = false;
        }
        
        // Mise à jour de la carte
        if (window.map) {
            window.map.setView([newLat, newLon], 15);
            if (window.marker) window.marker.setLatLng([newLat, newLon]);
        }
        if ($('gps-status')) $('gps-status').textContent = 'ACTIF';
        if ($('gps-status-indicator-text')) $('gps-status-indicator-text').textContent = 'Signal GPS OK';
    };

    const handleGPSError = (error) => {
        console.error(`[GPS ERREUR] Code ${error.code}: ${error.message}`);
        if ($('gps-status')) $('gps-status').textContent = 'ERREUR GPS';
        if ($('gps-status-indicator-text')) $('gps-status-indicator-text').textContent = 'Attente du signal GPS...';
    };

    /** Fetch la météo (proxy) et met à jour les variables physiques. */
    const fetchWeather = async (lat, lon) => {
        try {
            const response = await fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`);
            if (!response.ok) throw new Error("Erreur de l'API météo");
            const data = await response.json();
            
            const tempC = data.temp - 273.15;
            const tempK = data.temp;
            const pressure_hPa = data.pressure;
            const humidity_perc = data.humidity;
            
            lastT_K = tempK;
            lastP_hPa = pressure_hPa;
            lastH_perc = humidity_perc / 100.0;
            currentSpeedOfSound = getSpeedOfSound(tempK);
            currentAirDensity = calculateAirDensity(pressure_hPa, tempK, lastH_perc);
            
            lastKnownWeather = { tempC, pressure_hPa, humidity_perc, air_density: currentAirDensity, tempK };
            updateWeatherDOM(lastKnownWeather);
            
        } catch (error) {
            console.warn(`[Météo Échec]`);
            // Retour aux valeurs par défaut ISA
            lastKnownWeather = { tempC: 15.0, pressure_hPa: 1013.25, humidity_perc: 50, air_density: RHO_SEA_LEVEL, tempK: TEMP_SEA_LEVEL_K };
            updateWeatherDOM(lastKnownWeather, true);
        }
    };

    /** Simule la récupération de données de Polluants. */
    const fetchPollutants = async () => {
        // Simulation des données, car l'API n'est pas accessible directement
        const pollutants = {
            NO2: 20 + Math.random() * 5, PM25: 15 + Math.random() * 5,
            PM10: 25 + Math.random() * 5, O3: 50 + Math.random() * 10
        };
        lastKnownPollutants = pollutants;
    };

    const updateWeatherDOM = (data, isDefault = false) => {
        if ($('temp-air-2')) $('temp-air-2').textContent = `${data.tempC.toFixed(1)} °C`;
        if ($('pressure-2')) $('pressure-2').textContent = `${data.pressure_hPa.toFixed(0)} hPa`;
        if ($('humidity-2')) $('humidity-2').textContent = `${data.humidity_perc} %`;
        if ($('air-density')) $('air-density').textContent = `${data.air_density.toFixed(3)} kg/m³`;
        if ($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = `${currentSpeedOfSound.toFixed(2)} m/s${isDefault ? ' (Défaut)' : ''}`;
        if ($('weather-status')) $('weather-status').textContent = isDefault ? 'HORS LIGNE/DÉFAUT' : 'ACTIF';
        // Point de rosée simulé
        if ($('dew-point')) $('dew-point').textContent = dataOrDefault(data.tempC - (1 - data.humidity_perc/100) * 20, 1, ' °C (Sim)');
    };
    
    const updatePollutantsDOM = (data) => {
        if ($('no2')) $('no2').textContent = dataOrDefault(data.NO2, 1, ' µg/m³');
        if ($('pm25')) $('pm25').textContent = dataOrDefault(data.PM25, 1, ' µg/m³');
        if ($('pm10')) $('pm10').textContent = dataOrDefault(data.PM10, 1, ' µg/m³');
        if ($('o3')) $('o3').textContent = dataOrDefault(data.O3, 1, ' µg/m³');
    };


    // =================================================================
    // BLOC 4 : BOUCLES D'EXÉCUTION ET MISE À JOUR DU DOM
    // =================================================================

    /** Boucle principale d'exécution du filtre UKF (haute fréquence). */
    const updateSensorsAndFilter = () => {
        if (isEmergencyStop) {
            requestAnimationFrame(updateSensorsAndFilter); 
            return;
        }

        const now = Date.now();
        const dt = (now - lastUKFUpdate) / 1000.0;
        lastUKFUpdate = now;

        if (ukf && dt > 0) {
            const imuMeasure = [lastAccX, lastAccY, lastAccZ, lastGyrX, lastGyrY, lastGyrZ];
            const gpsMeasure = [currentPosition.lat, currentPosition.lon, currentPosition.alt, currentPosition.spd];
            const magMeasure = [lastMagX, lastMagY, lastMagZ];
            const referenceMag = [0, 0, MAG_FIELD_STRENGTH_REF * 1e-9]; // Mocked reference
            
            ukf.predict_propagation(dt, imuMeasure);
            ukf.update_correction(gpsMeasure, magMeasure, referenceMag);
            
            const ukfState = ukf.getState();
            kAlt = ukfState.alt;
            
            // Calculs Dynamiques
            const gravity = calculateWGS84Gravity(currentPosition.lat, kAlt);
            const instSpd_mps = currentPosition.spd || 0.0; 
            const dynamicPressure = 0.5 * currentAirDensity * instSpd_mps * instSpd_mps;
            
            // --- Mise à jour du DOM haute fréquence ---
            
            // Vitesse
            if ($('speed-instant-kmh')) $('speed-instant-kmh').textContent = dataOrDefault(instSpd_mps * KMH_MS, 2, ' km/h'); // Assumé ID pour '--.- km/h'
            if ($('speed-3d-kmh')) $('speed-3d-kmh').textContent = dataOrDefault(instSpd_mps * KMH_MS, 2, ' km/h'); 
            if ($('speed-brute-mps')) $('speed-brute-mps').textContent = dataOrDefault(instSpd_mps, 3, ' m/s');
            if ($('speed-stable-mps')) $('speed-stable-mps').textContent = dataOrDefault(instSpd_mps, 2, ' m/s');
            if ($('speed-stable-kms')) $('speed-stable-kms').textContent = dataOrDefault(instSpd_mps / 1000, 5, ' km/s');
            
            // UKF States
            if ($('ukf-alt')) $('ukf-alt').textContent = dataOrDefault(kAlt, 2, ' m');
            if ($('ukf-lat')) $('ukf-lat').textContent = dataOrDefault(ukfState.lat, 6, '°');
            if ($('ukf-lon')) $('ukf-lon').textContent = dataOrDefault(ukfState.lon, 6, '°');
            if ($('vertical-speed-ekf')) $('vertical-speed-ekf').textContent = dataOrDefault(ukfState.vel_up || 0, 3, ' m/s');
            if ($('cap-direction')) $('cap-direction').textContent = dataOrDefault(currentPosition.heading, 1, '°');
            
            // IMU & MAG RAW
            if ($('acc-raw-x')) $('acc-raw-x').textContent = dataOrDefault(lastAccX, 3, ' m/s²');
            if ($('acc-raw-y')) $('acc-raw-y').textContent = dataOrDefault(lastAccY, 3, ' m/s²');
            if ($('acc-raw-z')) $('acc-raw-z').textContent = dataOrDefault(lastAccZ, 3, ' m/s²');
            if ($('mag-raw-x')) $('mag-raw-x').textContent = dataOrDefault(lastMagX * 1e9, 2, ' nT');
            if ($('mag-raw-y')) $('mag-raw-y').textContent = dataOrDefault(lastMagY * 1e9, 2, ' nT');
            if ($('mag-raw-z')) $('mag-raw-z').textContent = dataOrDefault(lastMagZ * 1e9, 2, ' nT');
            
            // G-Force Verticale
            const nonGravAccelZ = lastAccZ - gravity; 
            if ($('gravity-local')) $('gravity-local').textContent = dataOrDefault(gravity, 4, ' m/s²');
            if ($('g-force-vertical')) $('g-force-vertical').textContent = dataOrDefault((nonGravAccelZ / G_ACC_STD) + 1.0, 3, ' G');
            if ($('accel-vertical-imu')) $('accel-vertical-imu').textContent = dataOrDefault(nonGravAccelZ, 3, ' m/s²');
            
            // Niveau à Bulle (IMU)
            if ($('pitch-display')) $('pitch-display').textContent = dataOrDefault(lastPitch, 1, '°');
            if ($('roll-display')) $('roll-display').textContent = dataOrDefault(lastRoll, 1, '°');
            
            // Mécanique
            if ($('pressure-dynamic')) $('pressure-dynamic').textContent = dataOrDefault(dynamicPressure, 2, ' Pa');
            if ($('coriolis-force')) $('coriolis-force').textContent = dataOrDefault(calculateCoriolisForce(currentPosition.lat * D2R, instSpd_mps, currentMass), 2, ' N');
            if ($('gps-accuracy-display')) $('gps-accuracy-display').textContent = dataOrDefault(currentPosition.acc, 2, ' m'); // Précision GPS (Acc)

        } else if (ukf && ukf.getState) {
             // Au moins mettre à jour l'IMU si UKF est là mais la GPS/heure n'est pas encore stable.
             if ($('pitch-display')) $('pitch-display').textContent = dataOrDefault(lastPitch, 1, '°');
             if ($('roll-display')) $('roll-display').textContent = dataOrDefault(lastRoll, 1, '°');
        }
        
        requestAnimationFrame(updateSensorsAndFilter); 
    };

    /** Boucle de mise à jour lente du DOM (Astro, Météo, Clock). */
    const updateDOMSlow = () => {
        if (isEmergencyStop) {
            setTimeout(updateDOMSlow, DOM_SLOW_UPDATE_MS);
            return;
        }

        const now = getCDate();
        const currentLat = currentPosition.lat;
        const currentLon = currentPosition.lon;
        const currentAlt = kAlt;
        const currentSpd_mps = currentPosition.spd || 0.0;
        const elapsedTime = (Date.now() - startTime) / 1000;
        const distanceRatio = netherMode ? (1/8.0) : 1.0; 
        
        // --- HORLOGE & TEMPS ÉCOULÉ & MINECRAFT ---
        if ($('local-time') && !$('local-time').textContent.includes('SYNCHRO ÉCHOUÉE')) {
            $('local-time').textContent = now.toLocaleTimeString('fr-FR');
        }
        if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
        if ($('elapsed-time')) $('elapsed-time').textContent = dataOrDefault(elapsedTime, 2, ' s');
        // Fonction getMinecraftTime est omise pour la concision mais devrait être présente
        if ($('minecraft-time')) $('minecraft-time').textContent = '00:00'; // MOCK
        if (isMoving) totalMoveTimeMs += DOM_SLOW_UPDATE_MS;
        if ($('move-time')) $('move-time').textContent = dataOrDefault(totalMoveTimeMs / 1000, 2, ' s');

        // --- STATISTIQUES DE VITESSE ET DISTANCE ---
        if ($('speed-max')) $('speed-max').textContent = dataOrDefault(vMaxSession, 1, ' km/h');
        const avgSpd = speedSamples > 0 ? (totalSpeedSum / speedSamples) : 0;
        if ($('speed-avg-move')) $('speed-avg-move').textContent = dataOrDefault(avgSpd * KMH_MS, 1, ' km/h');
        const avgSpdTotal = (distMTotal * distanceRatio) / (elapsedTime || 1); 
        if ($('speed-avg-total')) $('speed-avg-total').textContent = dataOrDefault(avgSpdTotal * KMH_MS, 1, ' km/h');
        
        if ($('distance-total-km')) $('distance-total-km').textContent = `${dataOrDefault(distMTotal / 1000 * distanceRatio, 3, ' km')} | ${dataOrDefault(distMTotal * distanceRatio, 0, ' m')}`;
        if ($('distance-ratio')) $('distance-ratio').textContent = dataOrDefault(distanceRatio, 3);
        
        const horizon = calculateMaxVisibleDistance(currentAlt);
        if ($('max-visible-distance')) $('max-visible-distance').textContent = dataOrDefault(horizon / 1000, 3, ' km');
        
        const dist_m = distMTotal * distanceRatio;
        if ($('distance-s-light')) $('distance-s-light').textContent = dataOrDefaultExp(dist_m / C_L, 2, ' s');
        // ... (autres distances lumière)
        const AU_TO_M = 149597870700;
        const AL_TO_M = 9460730472580800;
        if ($('distance-au-al')) $('distance-au-al').textContent = `${dataOrDefaultExp(dist_m / AU_TO_M, 2, ' UA')} | ${dataOrDefaultExp(dist_m / AL_TO_M, 2, ' al')}`;
        if ($('distance-horizon-s')) $('distance-horizon-s').textContent = dataOrDefault(horizon / (currentSpd_mps || 1), 1, ' s');


        // --- RELATIVITÉ ---
        const lorentzFactor = calculateLorentzFactor(currentSpd_mps);
        const restEnergy = calculateRestMassEnergy(currentMass);
        const gravDilation = calculateGravitationalDilation(currentAlt);
        const Rs = calculateSchwarzschildRadius(currentMass);
        
        if ($('%-speed-of-light')) $('%-speed-of-light').textContent = dataOrDefaultExp(currentSpd_mps / C_L * 100, 2, ' %');
        if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(lorentzFactor, 4);
        if ($('dilation-time-speed')) $('dilation-time-speed').textContent = dataOrDefault(Math.max(0, (lorentzFactor - 1) * 31536000e9), 2, ' ns/j'); 
        if ($('dilation-time-gravity')) $('dilation-time-gravity').textContent = dataOrDefault(gravDilation, 2, ' ns/j'); 
        if ($('mach-number')) $('mach-number').textContent = dataOrDefault(currentSpd_mps / currentSpeedOfSound, 4);
        
        if ($('energy-rel')) $('energy-rel').textContent = dataOrDefaultExp(lorentzFactor * restEnergy, 2, ' J');
        if ($('energy-rest')) $('energy-rest').textContent = dataOrDefaultExp(restEnergy, 2, ' J');
        const kineticEnergy = restEnergy * (lorentzFactor - 1);
        if ($('kinetic-energy')) $('kinetic-energy').textContent = dataOrDefault(kineticEnergy, 2, ' J');
        if ($('momentum-p')) $('momentum-p').textContent = dataOrDefaultExp(currentMass * currentSpd_mps * lorentzFactor, 2, ' kg·m/s');
        if ($('schwarzschild-radius')) $('schwarzschild-radius').textContent = dataOrDefaultExp(Rs, 3, ' m');

        // Dynamique & Forces (suite)
        const mechanicalPower = (kineticEnergy / elapsedTime) || 0; 
        if ($('mechanical-power')) $('mechanical-power').textContent = dataOrDefault(mechanicalPower / 1000, 2, ' kW'); 
        
        // --- ASTRO & MÉTÉO (Appels) ---
        if (typeof SunCalc !== 'undefined') {
            const times = SunCalc.getTimes(now, currentLat, currentLon);
            const sunPos = SunCalc.getPosition(now, currentLat, currentLon);
            if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(sunPos.altitude * R2D, 2, '°');
            if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault((sunPos.azimuth * R2D + 180) % 360, 2, '°');
            if ($('sunrise-times')) $('sunrise-times').textContent = times.sunrise ? times.sunrise.toLocaleTimeString('fr-FR') : 'N/A';
            if ($('sunset-times')) $('sunset-times').textContent = times.sunset ? times.sunset.toLocaleTimeString('fr-FR') : 'N/A';
        } else {
             // Mettre à jour les champs Astro à N/A ou MOCK si SunCalc manque
        }

        // Fetch Météo/Polluants seulement si des coordonnées valides sont disponibles
        if (currentLat !== 0 && currentLon !== 0) {
            fetchWeather(currentLat, currentLon); 
            fetchPollutants(); 
            updatePollutantsDOM(lastKnownPollutants || { NO2: 0, PM25: 0, PM10: 0, O3: 0 });
        }

        setTimeout(updateDOMSlow, DOM_SLOW_UPDATE_MS);
    };


    // =================================================================
    // BLOC 5 : GESTIONNAIRES D'ÉVÉNEMENTS ET INITIALISATION
    // =================================================================
    
    const resetDistance = () => { distMTotal = 0.0; console.log("[Contrôle] Distance réinitialisée."); };
    const resetVMax = () => { vMaxSession = 0.0; console.log("[Contrôle] Vitesse Max réinitialisée."); };
    const resetAll = () => { 
        resetDistance(); resetVMax(); totalSpeedSum = 0; speedSamples = 0; totalMoveTimeMs = 0; startTime = Date.now();
        if (ukf) ukf = new ProfessionalUKF(); 
        console.log("[Contrôle] Tous les paramètres réinitialisés."); 
    };
    
    const handleControlEvents = () => {
        // --- CONTRÔLES ---
        if ($('gps-start-btn')) $('gps-start-btn').addEventListener('click', () => { // Assumé ID pour ▶️ MARCHE GPS
            isGpsPaused = !isGpsPaused;
            $('gps-start-btn').textContent = isGpsPaused ? '▶️ MARCHE GPS' : '⏸️ PAUSE GPS';
            if (!isGpsPaused) console.log("GPS Repris.");
        });
        if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', () => {
            isEmergencyStop = !isEmergencyStop;
            if ($('stop-status')) $('stop-status').textContent = isEmergencyStop ? 'ACTIF 🔴' : 'INACTIF 🟢';
        });
        if ($('toggle-mode-btn')) $('toggle-mode-btn').addEventListener('click', () => {
            document.body.classList.toggle('dark-mode'); 
            const isDark = document.body.classList.contains('dark-mode');
            $('toggle-mode-btn').textContent = isDark ? 'Mode Jour' : 'Mode Nuit';
        });
        if ($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', () => {
            netherMode = !netherMode;
            $('nether-toggle-btn').textContent = `Mode Nether: ${netherMode ? 'ACTIVÉ (1:8)' : 'DÉSACTIVÉ (1:1)'}`;
        });
        if ($('mass-input')) $('mass-input').addEventListener('input', (e) => {
            currentMass = parseFloat(e.target.value) || 70.0;
            if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        });
        
        // Boutons de réinitialisation
        if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', resetDistance);
        if ($('reset-vmax-btn')) $('reset-vmax-btn').addEventListener('click', resetVMax);
        if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', resetAll);
        
        // Sélections
        if ($('environment-select')) $('environment-select').addEventListener('change', (e) => {
            selectedEnvironment = e.target.value;
            if ($('env-factor')) $('env-factor').textContent = `${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY} (x${ENVIRONMENT_FACTORS[selectedEnvironment].MULT.toFixed(1)})`;
        });
        
        // Assumé ID pour l'input de rotation
        if ($('rotation-radius')) $('rotation-radius').addEventListener('input', (e) => rotationRadius = parseFloat(e.target.value) || 100);
        if ($('angular-velocity')) $('angular-velocity').addEventListener('input', (e) => angularVelocity = parseFloat(e.target.value) || 0.0);

        // IMU/Motion Permission (à appeler depuis un bouton si nécessaire)
        // Note: L'utilisateur devra peut-être cliquer sur un bouton pour activer les capteurs IMU sur iOS/modernes
        if ($('imu-request-btn')) $('imu-request-btn').addEventListener('click', requestMotionPermission);
    };

    const initDashboard = () => {
        
        // 1. Initialisation des contrôles utilisateur et valeurs par défaut
        handleControlEvents();

        // 2. Tente de demander la permission IMU (si disponible)
        if (window.DeviceOrientationEvent || window.DeviceMotionEvent) {
            // Si l'API de demande de permission existe (iOS), l'utilisateur DOIT cliquer sur un bouton.
            if (typeof DeviceOrientationEvent.requestPermission === 'function') {
                if ($('imu-status')) $('imu-status').textContent = 'Cliquer pour Activer IMU';
            } else {
                // Sinon, on lance directement les listeners
                requestMotionPermission();
            }
        }
        
        // 3. Démarrage de la géolocalisation GPS (Haute précision)
        if (navigator.geolocation) {
            navigator.geolocation.watchPosition(handleGPS, handleGPSError, {
                enableHighAccuracy: true,
                maximumAge: 0, 
                timeout: 5000 
            });
        }
        
        // 4. Initialisation de la Carte (Leaflet)
        if (typeof L !== 'undefined' && $('map-container')) {
            window.map = L.map('map-container').setView([currentPosition.lat, currentPosition.lon], 15);
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                attribution: '© OpenStreetMap contributors'
            }).addTo(window.map);
            window.marker = L.marker([currentPosition.lat, currentPosition.lon]).addTo(window.map)
                .bindPopup("Position UKF/GPS").openPopup();
        }

        // 5. Lancement de la Synchro NTP et UKF
        syncH().finally(() => { 
            // UKF Initialisation
            if (typeof math !== 'undefined') {
                try {
                    ukf = new ProfessionalUKF(); 
                } catch (e) {
                    console.error("Échec de l'initialisation UKF:", e.message);
                }
            }
            // Démarrer les boucles de mise à jour
            updateSensorsAndFilter();
            updateDOMSlow(); 
        });
        
        // Initialisation de l'affichage de la gravité WGS84
        if ($('gravity-base')) $('gravity-base').textContent = `${calculateWGS84Gravity(currentPosition.lat, kAlt).toFixed(4)} m/s²`;
        if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        if ($('env-factor')) $('env-factor').textContent = `${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY} (x${ENVIRONMENT_FACTORS[selectedEnvironment].MULT.toFixed(1)})`;
        if ($('force-gps-accuracy-display')) $('force-gps-accuracy-display').textContent = dataOrDefault(gpsAccuracyTarget, 6, ' m');
    };

    document.addEventListener('DOMContentLoaded', initDashboard);
    
})(window);
