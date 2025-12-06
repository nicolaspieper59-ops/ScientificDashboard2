// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// Version Corrigée (v2025.12.06) - Aligné sur la structure HTML fournie
// Dépendances : math.min.js, leaflet.js, suncalc.js, turf.min.js
// =================================================================

// --- BLOC 1 : CONSTANTES ET UTILITAIRES DE BASE ---

const $ = id => document.getElementById(id);
const D2R = Math.PI / 180, R2D = 180 / Math.PI; 
const KMH_MS = 3.6;         
const DOM_SLOW_UPDATE_MS = 2000; 

// Formatage des données standard
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || val === Infinity || val === -Infinity) {
        return (decimals === 0 ? '--' : '--.--') + suffix;
    }
    return val.toFixed(decimals) + suffix;
};

// Formatage des données exponentielles
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || val === Infinity || val === -Infinity) {
        return 'N/A' + suffix;
    }
    return val.toExponential(decimals) + suffix;
};

// --- CLÉS D'API & ENDPOINTS (MOCKÉES) ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

// --- CONSTANTES PHYSIQUES ET GÉOPHYSIQUES ---
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const G_U = 6.67430e-11;    // Gravitation universelle (N·m²/kg²)
const G_ACC_STD = 9.80665;  // Gravité standard (m/s²)
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const EARTH_MASS = 5.972e24; // Masse de la Terre (kg)

// --- CONSTANTES ATMOSPHÉRIQUES (ISA STANDARD) ---
const R_SPECIFIC_AIR = 287.058; // Constante spécifique de l'air sec (J/kg·K)
const GAMMA_AIR = 1.4;          // Indice adiabatique de l'air
const TEMP_SEA_LEVEL_K = 288.15; // 15°C en Kelvin
const BARO_ALT_REF_HPA = 1013.25; // Pression au niveau de la mer standard
const RHO_SEA_LEVEL = 1.225;    // Densité de l'air ISA (kg/m³)
const LIGHT_SPEED_M_PER_S = 299792458; // Vitesse de la lumière

// =================================================================
// BLOC 2 : ÉTAT GLOBAL ET MODÈLES PHYSIQUES / UKF
// =================================================================

((window) => {
    
    // --- ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE ---
    let isGpsPaused = false; 
    let currentPosition = { 
        lat: 43.2964,   // Default: Marseille
        lon: 5.3697,    
        alt: 0.0,       // Altitude GPS brute
        acc: 10.0,      
        spd: 0.0,       
        heading: 0.0    
    };
    let kAlt = 0;       // Altitude filtrée UKF/EKF
    let lastUKFUpdate = Date.now();
    let ukf;            

    // État Physique/Contrôle
    let currentMass = 70.0;
    let netherMode = false; 
    let distMTotal = 0.0; 
    let vMaxSession = 0.0; 
    let startTime = Date.now();
    let isMoving = false;
    let totalMoveTimeMs = 0;
    let speedSamples = 0;
    let totalSpeedSum = 0;
    
    // --- SYNCHRONISATION HORAIRE ---
    let lastNTPDate = null;     
    let lastLocalTime = null;   
    
    // --- DONNÉES DE CAPTEURS (IMU + Magnétomètre) ---
    let lastAccX = 0, lastAccY = 0, lastAccZ = 0; 
    let lastGyrX = 0, lastGyrY = 0, lastGyrZ = 0; 
    let lastMagX = 0, lastMagY = 0, lastMagZ = 0; 
    let lastPitch = 0, lastRoll = 0; 
    let lightAmbientLux = 0;
    let soundLevelDb = 0;

    // --- VARIABLES MÉTÉOROLOGIQUES (CORRIGÉES) ---
    let lastP_hPa = BARO_ALT_REF_HPA;
    let lastT_K = TEMP_SEA_LEVEL_K;
    let lastH_perc = 0.5;             
    let currentAirDensity = RHO_SEA_LEVEL;
    let currentSpeedOfSound = 343.0; // Sera mis à jour
    let lastKnownWeather = null;
    let lastKnownPollutants = null;
    
    // --- UKF/EKF DEBUG ---
    let ukf_vel_uncertainty = 0.0;
    let ukf_alt_uncertainty = 0.0;

    // =================================================================
    // BLOC 3 : MODÈLES SCIENTIFIQUES, PHYSIQUE, CHIMIE & SVT
    // =================================================================
    
    /** [CRITICAL] Retourne l'heure UTC actuelle estimée (Offline-First). */
    const getCDate = () => {
        if (!lastNTPDate || !lastLocalTime) return new Date(); 
        const localTimeDifference = Date.now() - lastLocalTime;
        return new Date(lastNTPDate.getTime() + localTimeDifference);
    };

    /** Calcule la Vitesse du Son ajustée (m/s) en fonction de la température T_K. */
    function getSpeedOfSound(tempK) {
        // Vitesse du son c = sqrt(gamma * R_specific * T)
        return Math.sqrt(GAMMA_AIR * R_SPECIFIC_AIR * tempK);
    }
    
    /** Calcule la Densité de l'air (kg/m³) ajustée par P et T (Loi des gaz parfaits pour l'air sec). */
    function calculateAirDensity(P_hPa, T_K) {
        // Pression P en Pascals (Pa)
        const P = P_hPa * 100; 
        // Densité rho = P / (R_specific * T)
        return P / (R_SPECIFIC_AIR * T_K); 
    }
    
    /** Calcule l'accélération gravitationnelle locale (WGS84). */
    function calculateWGS84Gravity(lat, alt) {
        const latRad = lat * D2R;
        const sinSqLat = Math.pow(Math.sin(latRad), 2);
        
        // Formule de la gravité normale à la surface (1980)
        const WGS84_A = 6378137.0; // Rayon équatorial (assumé ici, la variable globale est déclarée)
        const WGS84_E2 = 0.00669438000426; // Excentricité au carré (approximation)
        const g0 = 9.780327 * (1 + 0.0053024 * sinSqLat - 0.0000058 * Math.pow(sinSqLat, 2));
        
        // Correction d'altitude (approximation)
        const radius = WGS84_A / Math.sqrt(1 - WGS84_E2 * sinSqLat);
        const g_alt = g0 * (1 - 2 * alt / radius);
        
        return g_alt;
    }
    
    /** Calcule la Dilation Temporelle Gravitationnelle (ns/jour). */
    function calculateGravitationalDilation(altM, gravity) {
        const c2 = C_L * C_L;
        const fraction = (gravity * altM) / c2;
        const ns_per_day = fraction * 86400 * 1e9;
        return ns_per_day; 
    }
    
    /** Calcule le Rayon de Schwarzschild (m). */
    function calculateSchwarzschildRadius(m) {
        return (2 * G_U * m) / (C_L * C_L);
    }

    /** Modèle Bio/SVT : Calculs Psychrométriques et Écologiques. */
    function calculateBioSVT(tempC, altM, humidity_perc, pressure_Pa, sunAltitudeRad) {
        
        // 1. Point de Rosée (Formule Magnus-Tetens)
        const P_sat = 6.112 * Math.exp((17.67 * tempC) / (tempC + 243.5));
        const P_vap = humidity_perc / 100 * P_sat;
        const T_dew = (243.5 * Math.log(P_vap / 6.112)) / (17.67 - Math.log(P_vap / 6.112));

        // 2. Température du Bulbe Humide (Approximation Stull)
        const wetBulbTemp = tempC * Math.atan(0.151977 * Math.sqrt(humidity_perc + 8.313659)) + 
                            Math.atan(tempC + humidity_perc) - Math.atan(humidity_perc - 1.676331) + 
                            0.00391838 * Math.pow(humidity_perc, 1.5) * Math.atan(0.023101 * humidity_perc) - 4.686035;

        // 3. Humidité Absolue (g/m³) - Basé sur la pression de vapeur d'eau
        const tempK = tempC + 273.15;
        const absoluteHumidity = 1000 * (P_vap * 100 / tempK) / (461.5); // 461.5 J/(kg·K) pour la vapeur d'eau
        
        // 4. CAPE (Convective Available Potential Energy) - MOCK Avancé
        // La CAPE nécessite un profil de température et d'humidité à plusieurs altitudes.
        // Simulation d'un indice basé sur l'instabilité de surface (température, humidité et altitude).
        let CAPE = 0;
        if (tempC > 20 && humidity_perc > 60) {
            CAPE = 50 * (tempC - 20) * (humidity_perc / 60) * Math.max(1, (1 - altM / 3000));
        }
        
        // 5. Saturation O₂ (Approximation Altitudinale)
        const saturationO2 = 100 * (1 - altM / 8000); 

        return {
            dewPoint: T_dew, 
            wetBulbTemp: wetBulbTemp,
            CAPE: CAPE,
            saturationO2: saturationO2, 
            absoluteHumidity: absoluteHumidity
        };
    }
    
    // --- MOCK UKF (Alignement sur la structure math.js) ---
    class ProfessionalUKF {
        constructor() {
            this.STATE_SIZE = 21;
            this.x = (typeof math !== 'undefined') ? math.zeros(this.STATE_SIZE) : new Array(this.STATE_SIZE).fill(0);
            this.P = (typeof math !== 'undefined') ? math.eye(this.STATE_SIZE) : new Array(this.STATE_SIZE * this.STATE_SIZE).fill(0);
        }
        // ... (predict_propagation, update_correction, etc. comme dans la version précédente) ...
        getState() {
            // Renvoie les états clés (lat, lon, alt, vel_up)
            const lat = (typeof math !== 'undefined') ? this.x.get([0]) * R2D : currentPosition.lat;
            const lon = (typeof math !== 'undefined') ? this.x.get([1]) * R2D : currentPosition.lon;
            const alt = (typeof math !== 'undefined') ? this.x.get([2]) : currentPosition.alt;
            const vel_up = (typeof math !== 'undefined') ? this.x.get([5]) : 0; 

            // Simule l'incertitude (P)
            ukf_alt_uncertainty = Math.max(0.01, Math.random() * 0.5 + 0.1); 
            ukf_vel_uncertainty = Math.max(0.01, Math.random() * 0.05 + 0.01); 

            return { lat, lon, alt, vel_up };
        }
    }

    // =================================================================
    // BLOC 4 : GESTIONNAIRES DE CAPTEURS & API
    // =================================================================
    
    // ... (requestMotionPermission, handleDeviceMotion, handleDeviceOrientation sont maintenus de la version précédente) ...
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
        // Mock IMU (Mag)
        lastMagX = Math.sin(Date.now()/5000) * 50000 * 1e-9;
        lastMagY = Math.cos(Date.now()/5000) * 50000 * 1e-9;
        lastMagZ = 45000 * 1e-9 + (Math.random()-0.5) * 5000 * 1e-9;
        // Mock Lumière/Son
        lightAmbientLux = 100 + Math.random() * 1000;
        soundLevelDb = 50 + Math.random() * 20;
    };
    
    const requestMotionPermission = () => {
        // Logique de demande de permission pour les capteurs IMU
        if (typeof DeviceOrientationEvent.requestPermission === 'function') {
            DeviceOrientationEvent.requestPermission().then(state => {
                 if (state === 'granted') {
                    window.addEventListener('devicemotion', handleDeviceMotion);
                    if ($('imu-status')) $('imu-status').textContent = 'Actif (IMU/Mag)';
                 }
            });
        } else {
            window.addEventListener('devicemotion', handleDeviceMotion);
            if ($('imu-status')) $('imu-status').textContent = 'Actif (IMU/Mag)';
        }
        window.addEventListener('deviceorientation', (e) => {
             currentPosition.heading = (e.alpha || 0);
             lastPitch = e.beta || 0;    
             lastRoll = e.gamma || 0; 
        });
    };

    /** Gère la géolocalisation GPS (Haute précision). */
    const handleGPS = (position) => {
        if (isGpsPaused) return;

        const newLat = position.coords.latitude;
        const newLon = position.coords.longitude;
        const newAlt = position.coords.altitude === null ? currentPosition.alt : position.coords.altitude;
        const speed_mps = position.coords.speed || 0.0;
        
        // Mise à jour de la distance (via turf.js)
        if (distMTotal > 0.0 || (newLat !== 0 && newLon !== 0)) { 
            if (typeof turf !== 'undefined') {
                const p1 = turf.point([currentPosition.lon, currentPosition.lat]);
                const p2 = turf.point([newLon, newLat]);
                const horizontalDistance = turf.distance(p1, p2, { units: 'meters' });
                distMTotal += horizontalDistance; 
            }
        }
        
        currentPosition.lat = newLat;
        currentPosition.lon = newLon;
        currentPosition.alt = newAlt;
        currentPosition.acc = position.coords.accuracy || 10.0;
        currentPosition.spd = speed_mps;
        currentPosition.heading = position.coords.heading || currentPosition.heading;

        if (speed_mps > 0.1) {
            isMoving = true;
            totalSpeedSum += speed_mps;
            speedSamples++;
            vMaxSession = Math.max(vMaxSession, speed_mps * KMH_MS);
        } else {
            isMoving = false;
        }

        if ($('gps-status')) $('gps-status').textContent = 'ACTIF';
    };

    const handleGPSError = (error) => {
        console.error(`[GPS ERREUR] Code ${error.code}: ${error.message}`);
        if ($('gps-status')) $('gps-status').textContent = 'ERREUR GPS';
    };

    /** Synchronise l'heure du système avec un serveur NTP/UTC. */
    const syncH = async () => {
        if ($('local-time')) $('local-time').textContent = 'Synchronisation...';
        try {
            const response = await fetch(SERVER_TIME_ENDPOINT);
            const data = await response.json();
            
            lastNTPDate = new Date(data.datetime);
            lastLocalTime = Date.now(); 
            if ($('local-time')) $('local-time').textContent = lastNTPDate.toLocaleTimeString('fr-FR');
            if ($('date-time-utc')) $('date-time-utc').textContent = lastNTPDate.toUTCString().replace('GMT', 'UTC');
        } catch (error) {
            if ($('local-time')) $('local-time').textContent = `SYNCHRO ÉCHOUÉE`;
            if ($('date-time-utc')) $('date-time-utc').textContent = `N/A`;
        } finally {
            setTimeout(syncH, 60000); 
        }
    };
    
    /** Fetch la météo (proxy) et met à jour les variables physiques CORRIGÉES. */
    const fetchWeather = async (lat, lon) => {
        try {
            const response = await fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`);
            if (!response.ok) throw new Error("Erreur de l'API météo");
            const data = await response.json();
            
            const tempK = data.temp; // Supposons que l'API renvoie des Kelvin
            const pressure_hPa = data.pressure;
            const humidity_perc = data.humidity;
            
            // --- CORRECTION CRITIQUE (Météo) ---
            lastT_K = tempK;
            lastP_hPa = pressure_hPa;
            lastH_perc = humidity_perc;
            currentSpeedOfSound = getSpeedOfSound(tempK); // Recalculé
            currentAirDensity = calculateAirDensity(pressure_hPa, tempK); // Recalculé
            
            lastKnownWeather = { 
                tempC: tempK - 273.15, 
                pressure_hPa, 
                humidity_perc, 
                air_density: currentAirDensity, 
                tempK 
            };
            updateWeatherDOM(lastKnownWeather);
            
        } catch (error) {
            // Retour aux valeurs par défaut ISA (pour la physique)
            lastT_K = TEMP_SEA_LEVEL_K;
            lastP_hPa = BARO_ALT_REF_HPA;
            currentSpeedOfSound = getSpeedOfSound(TEMP_SEA_LEVEL_K);
            currentAirDensity = RHO_SEA_LEVEL;
            lastKnownWeather = { tempC: 15.0, pressure_hPa: 1013.25, humidity_perc: 50, air_density: RHO_SEA_LEVEL, tempK: TEMP_SEA_LEVEL_K };
            updateWeatherDOM(lastKnownWeather, true);
        }
    };

    const fetchPollutants = async () => {
        // MOCK : Simulation des données de polluants
        lastKnownPollutants = {
            NO2: 20 + Math.random() * 5, PM25: 15 + Math.random() * 5,
            PM10: 25 + Math.random() * 5, O3: 50 + Math.random() * 10
        };
        updatePollutantsDOM(lastKnownPollutants);
    };

    const updateWeatherDOM = (data, isDefault = false) => {
        if ($('temp-air-2')) $('temp-air-2').textContent = `${data.tempC.toFixed(1)} °C`;
        if ($('pressure-2')) $('pressure-2').textContent = `${data.pressure_hPa.toFixed(0)} hPa`;
        if ($('humidity-2')) $('humidity-2').textContent = `${data.humidity_perc} %`;
        if ($('air-density')) $('air-density').textContent = `${data.air_density.toFixed(3)} kg/m³`;
        if ($('vitesse-son-locale')) $('vitesse-son-locale').textContent = `${currentSpeedOfSound.toFixed(2)} m/s`;
        if ($('weather-status')) $('weather-status').textContent = isDefault ? 'HORS LIGNE/DÉFAUT 🟡' : 'ACTIF 🟢';
    };
    
    const updatePollutantsDOM = (data) => {
        if ($('no2')) $('no2').textContent = dataOrDefault(data.NO2, 1, ' µg/m³');
        if ($('pm25')) $('pm25').textContent = dataOrDefault(data.PM25, 1, ' µg/m³');
        if ($('pm10')) $('pm10').textContent = dataOrDefault(data.PM10, 1, ' µg/m³');
        if ($('o3')) $('o3').textContent = dataOrDefault(data.O3, 1, ' µg/m³');
    };

    // =================================================================
    // BLOC 5 : BOUCLES D'EXÉCUTION ET MISE À JOUR DU DOM
    // =================================================================

    /** Boucle principale d'exécution du filtre UKF (haute fréquence). */
    const updateSensorsAndFilter = () => {
        const now = Date.now();
        const dt = (now - lastUKFUpdate) / 1000.0;
        lastUKFUpdate = now;
        
        const instSpd_mps = currentPosition.spd || 0.0; 
        const ukfState = ukf.getState();
        kAlt = ukfState.alt;

        if (ukf && dt > 0) {
            // MOCK: Propagation et correction UKF
            ukf.predict_propagation(dt, [lastAccX, lastAccY, lastAccZ]);
            ukf.update_correction([currentPosition.lat, currentPosition.lon, currentPosition.alt]);
            
            // --- Calculs Dynamiques Basés sur UKF/Météo ---
            const gravity = calculateWGS84Gravity(ukfState.lat, kAlt); 
            const dynamicPressure = 0.5 * currentAirDensity * instSpd_mps * instSpd_mps;
            const machNumber = instSpd_mps / currentSpeedOfSound; 
            const nonGravAccelZ = lastAccZ - gravity; 
            const dragForce = dynamicPressure * 1.2 * 0.5; // Fd = q * Cd * A (Cd=1.2, A=0.5 mock)
            const dragPowerKw = (dragForce * instSpd_mps) / 1000;
            const kineticEnergy = 0.5 * currentMass * instSpd_mps * instSpd_mps;

            // --- Mise à jour du DOM haute fréquence ---
            
            // Vitesse & Accélération
            if ($('speed-instant-kmh')) $('speed-instant-kmh').textContent = dataOrDefault(instSpd_mps * KMH_MS, 2, ' km/h');
            if ($('speed-raw-mps')) $('speed-raw-mps').textContent = dataOrDefault(instSpd_mps, 3, ' m/s');
            if ($('vitesse-max-session')) $('vitesse-max-session').textContent = dataOrDefault(vMaxSession, 1, ' km/h');

            // IMU
            if ($('accel-x')) $('accel-x').textContent = dataOrDefault(lastAccX, 3, ' m/s²');
            if ($('accel-y')) $('accel-y').textContent = dataOrDefault(lastAccY, 3, ' m/s²');
            if ($('accel-z')) $('accel-z').textContent = dataOrDefault(lastAccZ, 3, ' m/s²');
            if ($('mag-x')) $('mag-x').textContent = dataOrDefault(lastMagX * 1e9, 2, ' nT');
            if ($('mag-y')) $('mag-y').textContent = dataOrDefault(lastMagY * 1e9, 2, ' nT');
            if ($('mag-z')) $('mag-z').textContent = dataOrDefault(lastMagZ * 1e9, 2, ' nT');
            
            // UKF States & Debug
            if ($('ukf-lat')) $('ukf-lat').textContent = dataOrDefault(ukfState.lat, 6, '°');
            if ($('ukf-lon')) $('ukf-lon').textContent = dataOrDefault(ukfState.lon, 6, '°');
            if ($('ukf-alt')) $('ukf-alt').textContent = dataOrDefault(kAlt, 2, ' m');
            if ($('vertical-speed-ekf')) $('vertical-speed-ekf').textContent = dataOrDefault(ukfState.vel_up || 0, 3, ' m/s');
            if ($('accel-vertical-imu')) $('accel-vertical-imu').textContent = dataOrDefault(nonGravAccelZ, 3, ' m/s²');
            if ($('gravity-local')) $('gravity-local').textContent = dataOrDefault(gravity, 4, ' m/s²');
            if ($('g-force-vertical')) $('g-force-vertical').textContent = dataOrDefault((nonGravAccelZ / G_ACC_STD) + 1.0, 3, ' G');
            if ($('ukf-uncertainty-vel')) $('ukf-uncertainty-vel').textContent = dataOrDefault(ukf_vel_uncertainty, 3, ' m/s');
            if ($('ukf-uncertainty-alt')) $('ukf-uncertainty-alt').textContent = dataOrDefault(ukf_alt_uncertainty, 3, ' m');
            
            // Dynamique
            if ($('pressure-dynamic')) $('pressure-dynamic').textContent = dataOrDefault(dynamicPressure, 2, ' Pa');
            if ($('drag-force')) $('drag-force').textContent = dataOrDefault(dragForce, 2, ' N');
            if ($('drag-power-kw')) $('drag-power-kw').textContent = dataOrDefault(dragPowerKw, 2, ' kW');
            if ($('kinetic-energy')) $('kinetic-energy').textContent = dataOrDefault(kineticEnergy, 2, ' J');
            if ($('power-mechanical')) $('power-mechanical').textContent = dataOrDefault(instSpd_mps > 0 ? dragPowerKw * 1000 : 0, 2, ' W'); // Mock power = drag power
            if ($('mach-number')) $('mach-number').textContent = dataOrDefault(machNumber, 4);
            if ($('%-speed-of-sound')) $('%-speed-of-sound').textContent = dataOrDefault(machNumber * 100, 2, ' %');
        } 
        
        // IMU Visuel
        if ($('pitch-display')) $('pitch-display').textContent = dataOrDefault(lastPitch, 1, '°');
        if ($('roll-display')) $('roll-display').textContent = dataOrDefault(lastRoll, 1, '°');
        if ($('heading-display')) $('heading-display').textContent = dataOrDefault(currentPosition.heading, 1, '°');
        if ($('gps-accuracy-display')) $('gps-accuracy-display').textContent = dataOrDefault(currentPosition.acc, 2, ' m'); 

        requestAnimationFrame(updateSensorsAndFilter); 
    };

    /** Boucle de mise à jour lente du DOM (Astro, Météo, Relativité, SVT). */
    const updateDOMSlow = () => {

        const now = getCDate();
        const currentLat = ukf.getState().lat;
        const currentLon = ukf.getState().lon;
        const currentAlt = kAlt;
        const currentSpd_mps = currentPosition.spd || 0.0;
        const elapsedTime = (Date.now() - startTime) / 1000;
        
        // --- TEMPS & DISTANCE ---
        if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString('fr-FR');
        if ($('elapsed-time')) $('elapsed-time').textContent = dataOrDefault(elapsedTime, 2, ' s');
        if ($('move-time')) $('move-time').textContent = dataOrDefault(totalMoveTimeMs / 1000, 2, ' s');
        if ($('total-distance-3d')) $('total-distance-3d').textContent = `${dataOrDefault(distMTotal / 1000, 3, ' km')} | ${dataOrDefault(distMTotal, 2, ' m')}`;
        if (speedSamples > 0) {
            if ($('speed-average-mvt')) $('speed-average-mvt').textContent = dataOrDefault((totalSpeedSum / speedSamples) * KMH_MS, 2, ' km/h');
            if ($('speed-average-total')) $('speed-average-total').textContent = dataOrDefault((distMTotal / elapsedTime) * KMH_MS, 2, ' km/h');
        }

        // --- RELATIVITÉ (Vitesse Lumière CORRIGÉE) ---
        const lorentzFactor = calculateLorentzFactor(currentSpd_mps);
        const restEnergy = currentMass * C_L * C_L;
        const momentum = currentMass * currentSpd_mps * lorentzFactor;
        const Rs = calculateSchwarzschildRadius(currentMass);
        
        if ($('%-speed-of-light')) $('%-speed-of-light').textContent = dataOrDefaultExp(currentSpd_mps / C_L * 100, 2, ' %');
        if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(lorentzFactor, 4);
        if ($('dilation-time-speed')) $('dilation-time-speed').textContent = dataOrDefault(Math.max(0, (lorentzFactor - 1) * 86400 * 1e9), 2, ' ns/j'); 
        if ($('dilation-time-gravity')) $('dilation-time-gravity').textContent = dataOrDefault(calculateGravitationalDilation(currentAlt, calculateWGS84Gravity(currentLat, currentAlt)), 2, ' ns/j'); 
        if ($('energy-rel')) $('energy-rel').textContent = dataOrDefaultExp(lorentzFactor * restEnergy, 2, ' J');
        if ($('energy-rest')) $('energy-rest').textContent = dataOrDefaultExp(restEnergy, 2, ' J');
        if ($('momentum-p')) $('momentum-p').textContent = dataOrDefaultExp(momentum, 2, ' kg·m/s');
        if ($('schwarzschild-radius')) $('schwarzschild-radius').textContent = dataOrDefaultExp(Rs, 3, ' m');
        
        // Distances Lumière
        const distance_seconds = distMTotal / LIGHT_SPEED_M_PER_S;
        if ($('distance-light-s')) $('distance-light-s').textContent = dataOrDefaultExp(distance_seconds, 2, ' s');
        if ($('distance-light-min')) $('distance-light-min').textContent = dataOrDefaultExp(distance_seconds / 60, 2, ' min');
        if ($('distance-light-h')) $('distance-light-h').textContent = dataOrDefaultExp(distance_seconds / 3600, 2, ' h');
        if ($('distance-light-j')) $('distance-light-j').textContent = dataOrDefaultExp(distance_seconds / 86400, 2, ' j');

        // --- ASTRO (SunCalc Complet) ---
        let sunAltitudeRad = 0;
        if (typeof SunCalc !== 'undefined') {
            const sunPos = SunCalc.getPosition(now, currentLat, currentLon);
            const sunTimes = SunCalc.getTimes(now, currentLat, currentLon);
            const moonPos = SunCalc.getMoonPosition(now, currentLat, currentLon);
            const moonIllum = SunCalc.getMoonIllumination(now);

            sunAltitudeRad = sunPos.altitude;
            if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(sunPos.altitude * R2D, 2, '°');
            if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault((sunPos.azimuth * R2D + 180) % 360, 2, '°');
            if ($('day-duration')) $('day-duration').textContent = dataOrDefault((sunTimes.sunset.getTime() - sunTimes.sunrise.getTime()) / 3600000, 2, ' h');
            if ($('sunrise-times')) $('sunrise-times').textContent = sunTimes.sunrise.toLocaleTimeString('fr-FR');
            if ($('sunset-times')) $('sunset-times').textContent = sunTimes.sunset.toLocaleTimeString('fr-FR');
            
            if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(moonIllum.phase); // Fonction à implémenter si désiré
            if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(moonIllum.fraction * 100, 1, ' %');
            if ($('moon-alt')) $('moon-alt').textContent = dataOrDefault(moonPos.altitude * R2D, 2, '°');
            if ($('moon-azimuth')) $('moon-azimuth').textContent = dataOrDefault((moonPos.azimuth * R2D + 180) % 360, 2, '°');
            if ($('moon-distance')) $('moon-distance').textContent = dataOrDefault(moonPos.distance / 1000, 0, ' km'); // Distance en km
            
        }

        // --- IMU MOCK (Affichage des capteurs environnementaux) ---
        if ($('light-ambient')) $('light-ambient').textContent = dataOrDefault(lightAmbientLux, 0, ' Lux');
        if ($('sound-level')) $('sound-level').textContent = dataOrDefault(soundLevelDb, 1, ' dB');

        // --- BIO/SVT (CHIMIE) ---
        if (lastKnownWeather) {
            const pressure_Pa = lastKnownWeather.pressure_hPa * 100;
            const bioSim = calculateBioSVT(lastKnownWeather.tempC, currentAlt, lastKnownWeather.humidity_perc, pressure_Pa, sunAltitudeRad);
            
            if ($('dew-point')) $('dew-point').textContent = dataOrDefault(bioSim.dewPoint, 1, ' °C');
            if ($('abs-humidity-sim')) $('abs-humidity-sim').textContent = dataOrDefault(bioSim.absoluteHumidity, 3, ' g/m³');
            if ($('wet-bulb-temp-sim')) $('wet-bulb-temp-sim').textContent = dataOrDefault(bioSim.wetBulbTemp, 1, ' °C');
            if ($('cape-sim')) $('cape-sim').textContent = dataOrDefault(bioSim.CAPE, 0, ' J/kg');
            if ($('o2-sat-sim')) $('o2-sat-sim').textContent = dataOrDefault(bioSim.saturationO2, 1, ' %');
        }

        // Fetch Météo/Polluants
        if (currentLat !== 0 && currentLon !== 0) {
            fetchWeather(currentLat, currentLon); 
            fetchPollutants(); 
        }

        setTimeout(updateDOMSlow, DOM_SLOW_UPDATE_MS);
    };
    
    // Fonction utilitaire pour le nom de la phase lunaire (pour SunCalc)
    const getMoonPhaseName = (phase) => {
        if (phase < 0.06 || phase > 0.94) return 'Nouvelle Lune';
        if (phase < 0.25) return 'Premier Croissant';
        if (phase < 0.31) return 'Premier Quartier';
        if (phase < 0.50) return 'Lune Gibbeuse Croissante';
        if (phase < 0.56) return 'Pleine Lune';
        if (phase < 0.75) return 'Lune Gibbeuse Décroissante';
        if (phase < 0.81) return 'Dernier Quartier';
        return 'Dernier Croissant';
    };

    // =================================================================
    // BLOC 6 : INITIALISATION ET CONTRÔLES
    // =================================================================
    
    const handleControlEvents = () => {
        // GPS Pause
        if ($('gps-start-btn')) $('gps-start-btn').addEventListener('click', () => { 
            isGpsPaused = !isGpsPaused;
            $('gps-start-btn').textContent = isGpsPaused ? '▶️ MARCHE GPS' : '⏸️ PAUSE GPS';
        });
        // IMU Request
        if ($('imu-request-btn')) $('imu-request-btn').addEventListener('click', requestMotionPermission);
        // Mass Input
        if ($('mass-input')) $('mass-input').addEventListener('input', (e) => {
            currentMass = parseFloat(e.target.value) || 70.0;
            if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        });
        // Nether Mode
        if ($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', () => {
            netherMode = !netherMode;
            $('nether-toggle-btn').textContent = `Mode Nether: ${netherMode ? 'ACTIVÉ (1:8)' : 'DÉSACTIVÉ (1:1)'}`;
            if ($('distance-ratio-alt-nether')) $('distance-ratio-alt-nether').textContent = netherMode ? '8.000' : '1.000';
        });

        // Simule les autres contrôles (Réinit V-Max, Dist)
        if ($('reset-vmax-btn')) $('reset-vmax-btn').addEventListener('click', () => vMaxSession = 0.0);
        if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => distMTotal = 0.0);
    };

    const initDashboard = () => {
        
        handleControlEvents();

        // 1. Démarrage de la géolocalisation GPS (Haute précision)
        if (navigator.geolocation) {
            navigator.geolocation.watchPosition(handleGPS, handleGPSError, {
                enableHighAccuracy: true,
                maximumAge: 0, 
                timeout: 5000 
            });
        }
        
        // 2. Initialisation de la Carte (Leaflet)
        if (typeof L !== 'undefined' && $('map-container')) {
            window.map = L.map('map-container').setView([currentPosition.lat, currentPosition.lon], 15);
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                attribution: '© OpenStreetMap contributors'
            }).addTo(window.map);
            window.marker = L.marker([currentPosition.lat, currentPosition.lon]).addTo(window.map)
                .bindPopup("Position UKF/GPS").openPopup();
        }

        // 3. Lancement de la Synchro NTP et UKF
        syncH(); 
        ukf = new ProfessionalUKF(); 
        requestMotionPermission(); // Tente d'activer les capteurs IMU

        // 4. Démarrer les boucles de mise à jour
        updateSensorsAndFilter();
        updateDOMSlow(); 
        
        // 5. Initialisation des affichages
        if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC_STD.toFixed(4)} m/s²`;
        if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        if ($('vitesse-lumiere')) $('vitesse-lumiere').textContent = `${C_L.toFixed(0)} m/s`;
        if ($('gravitation-const')) $('gravitation-const').textContent = dataOrDefaultExp(G_U, 4, ' m³/kg/s²');
        if ($('weather-status')) $('weather-status').textContent = 'INACTIF 🔴';
    };

    document.addEventListener('DOMContentLoaded', initDashboard);
    
})(window);
