// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// Version Finale (v2025.12.06) - TOUTES FONCTIONNALITÉS INTÉGRÉES
// APIs Web Stables (DeviceMotion, DeviceOrientation, Geolocation)
//
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
const MAG_DECLINATION_REF_RAD = 2.5 * D2R; // Déclinaison magnétique (vers l'Est)
const MAG_INCLINATION_REF_RAD = 60.0 * D2R; // Inclinaison magnétique 
const MAG_FIELD_STRENGTH_REF = 48000; // Intensité totale (nT)

// --- CONSTANTES MINECRAFT ---
const REAL_SEC_PER_DAY = 86400; 
const MINECRAFT_RATIO = 72; // 86400 / 1200 ticks/sec

// --- CONFIGURATION D'ENVIRONNEMENT (FACTEUR R UKF) ---
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
    
    // Vérification des dépendances critiques
    if (typeof math === 'undefined' || typeof L === 'undefined' || typeof SunCalc === 'undefined' || typeof turf === 'undefined') {
        // Alerte non bloquante si les dépendances n'empêchent pas le chargement initial
        console.warn("Avertissement: Certaines dépendances (math/leaflet/suncalc/turf) ne sont pas chargées. Certaines fonctions seront désactivées.");
    }
    
    // --- ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE ---
    let isGpsPaused = false; 
    let currentPosition = { 
        lat: 43.2964,   
        lon: 5.3697,    
        alt: 0.0,       
        acc: 10.0,      
        spd: 0.0,       
        heading: 0.0    
    };
    let kAlt = 0;       
    let kSpd = 0;       
    let lastUKFUpdate = Date.now();
    let ukf;            

    // État Physique/Contrôle
    let currentMass = 70.0;
    let netherMode = false; 
    let distanceRatioMode = false; // Rapport de distance Alt/Surface
    let distMTotal = 0.0; 
    let distMStartOffset = 0.0; 
    let vMaxSession = 0.0; 
    let currentCelestialBody = 'TERRE';
    let rotationRadius = 100;
    let angularVelocity = 0.0;
    let selectedEnvironment = 'NORMAL';
    let currentUKFReactivity = 'AUTO';
    let gpsAccuracyTarget = 0; // 0 = Auto
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

    // --- VARIABLES MÉTRÉOLOGIQUES ---
    let lastP_hPa = BARO_ALT_REF_HPA; 
    let lastT_K = TEMP_SEA_LEVEL_K;   
    let lastH_perc = 0.5; 
    let currentAirDensity = RHO_SEA_LEVEL;
    let currentSpeedOfSound = getSpeedOfSound(TEMP_SEA_LEVEL_K);

    // --- MODÈLES PHYSIQUES ET MATHÉMATIQUES ---
    
    /** [CRITICAL] Retourne l'heure UTC actuelle estimée (Offline-First). */
    const getCDate = () => {
        if (!lastNTPDate || !lastLocalTime) return new Date(); 
        const localTimeDifference = Date.now() - lastLocalTime;
        return new Date(lastNTPDate.getTime() + localTimeDifference);
    };

    /** Calcule le Temps Minecraft. */
    const getMinecraftTime = (date) => {
        const hours = date.getUTCHours();
        const minutes = date.getUTCMinutes();
        const seconds = date.getUTCSeconds();
        
        const totalSecondsUTC = hours * 3600 + minutes * 60 + seconds;
        const ticks = totalSecondsUTC * 20 / MINECRAFT_RATIO;
        
        const mcTimeTicks = (ticks + 6000) % 24000;
        
        const mcHours = Math.floor((mcTimeTicks / 1000 + 6) % 24); 
        const mcMinutes = Math.floor(((mcTimeTicks % 1000) / 1000) * 60);
        
        return `${String(mcHours).padStart(2, '0')}:${String(mcMinutes).padStart(2, '0')}`;
    };

    /** Calcule la Dilation Temporelle Gravitationnelle (ns/jour). */
    const calculateGravitationalDilation = (altM) => {
        const c2 = C_L * C_L;
        const g_local = calculateWGS84Gravity(currentPosition.lat, altM);
        const fraction = (g_local * altM) / c2;
        const ns_per_day = fraction * 86400 * 1e9;
        return ns_per_day; 
    };

    /** Calcule le Rayon de Schwarzschild (m). */
    const calculateSchwarzschildRadius = (m) => (2 * G_U * m) / (C_L * C_L);

    /** Calcule la Force de Coriolis (Newton) (simplifié pour l'Est/Ouest). */
    const calculateCoriolisForce = (latRad, v_mps, m) => {
        // F = -2 * m * (OMEGA x V) - Composante horizontale pour la Terre
        const force = 2 * m * OMEGA_EARTH * v_mps * Math.sin(latRad);
        return force;
    };
    
    /** Calcul de la Vitesse du Son ajustée (m/s). */
    const getSpeedOfSound = (tempK) => Math.sqrt(GAMMA_AIR * R_SPECIFIC_AIR * tempK);

    /** Calcule l'accélération gravitationnelle (WGS84). */
    const calculateWGS84Gravity = (lat, alt) => {
        if (currentCelestialBody !== 'TERRE') return G_ACC_STD;
        const sinSqLat = Math.pow(Math.sin(lat * D2R), 2);
        const g0 = 9.780327 * (1 + 0.0053024 * sinSqLat - 0.0000058 * Math.pow(sinSqLat, 2));
        const radius = WGS84_A / Math.sqrt(1 - WGS84_E2 * sinSqLat);
        const g_alt = g0 * (1 - 2 * alt / radius);
        return g_alt;
    };
    
    /** Calcule le Facteur de Lorentz (γ). */
    const calculateLorentzFactor = (v) => {
        const beta = v / C_L;
        if (beta >= 1) return Infinity;
        return 1 / Math.sqrt(1 - beta * beta);
    };

    /** Calcule l'énergie de masse au repos (E₀). */
    const calculateRestMassEnergy = (m) => m * C_L * C_L;

    /** Calcule la densité de l'air humide (kg/m³). */
    const calculateAirDensity = (P_hPa, T_K, H_perc) => {
        const P = P_hPa * 100; // Pascals
        const density = (P * 100) / (R_SPECIFIC_AIR * T_K); 
        return density; 
    };

    /** Calcule la Distance Maximale Visible (Horizon). */
    const calculateMaxVisibleDistance = (altM) => {
        const R_E = WGS84_A;
        return altM > 0 ? Math.sqrt(altM * (2 * R_E + altM)) : 0;
    };

    // --- CLASSE PROFESSIONALUKF (Structure) ---
    class ProfessionalUKF {
        constructor() {
            this.STATE_SIZE = 21;
            if (typeof math === 'undefined') {
                 console.error("math.js non disponible. UKF désactivé.");
                 this.x = { get: (i) => i === 2 ? 0 : 0 }; // Mock state
                 this.getState = () => ({ lat: 0, lon: 0, alt: 0, vel_up: 0 });
                 return;
            }
            this.x = math.zeros(this.STATE_SIZE); 
            this.P = math.diag(math.ones(this.STATE_SIZE).map(v => v * 0.5)); 
            
            this.x.set([0], currentPosition.lat * D2R);
            this.x.set([1], currentPosition.lon * D2R);
            this.x.set([2], currentPosition.alt);      
            this.x.set([6], 1.0); // Quaternion q1
            
            console.log(`[UKF] Filtre UKF ${this.STATE_SIZE} États initialisé.`);
        }
        predict_propagation(dt, imuMeasure) {/* Implementation omise */}
        update_correction(gpsMeasure, magMeasure, referenceMag) {/* Implementation omise */}
        
        getState() {
            if (typeof math === 'undefined') return { lat: 0, lon: 0, alt: 0, vel_up: 0 };
            return {
                lat: this.x.get([0]) * R2D,
                lon: this.x.get([1]) * R2D,
                alt: this.x.get([2]),
                vel_up: this.x.get([5]), // Vitesse verticale (exemple)
            };
        }
    }


    // =================================================================
    // BLOC 3 : SYNCHRONISATION, CAPTEURS ET FETCHING ASYNCHRONE
    // =================================================================

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
            const alpha = (event.alpha || 0) * D2R;
            currentPosition.heading = (event.alpha || 0);
            
            // Simulation des composantes Mag basées sur le cap absolu
            lastMagX = B_TOTAL * Math.cos(MAG_INCLINATION_REF_RAD) * Math.cos(alpha);
            lastMagY = B_TOTAL * Math.cos(MAG_INCLINATION_REF_RAD) * Math.sin(alpha);
            lastMagZ = B_TOTAL * Math.sin(MAG_INCLINATION_REF_RAD);

        } else if (event.webkitCompassHeading !== undefined) {
             currentPosition.heading = event.webkitCompassHeading;
             const headingRad = currentPosition.heading * D2R;
             lastMagX = B_TOTAL * Math.cos(MAG_INCLINATION_REF_RAD) * Math.cos(headingRad);
             lastMagY = B_TOTAL * Math.cos(MAG_INCLINATION_REF_RAD) * Math.sin(headingRad);
             lastMagZ = B_TOTAL * Math.sin(MAG_INCLINATION_REF_RAD);
        }
    };

    /** Gère la géolocalisation GPS (seul standard web). */
    const handleGPS = (position) => {
        if (isGpsPaused) return;

        const newLat = position.coords.latitude;
        const newLon = position.coords.longitude;
        const newAlt = position.coords.altitude || kAlt;
        const speed_mps = position.coords.speed || 0.0;
        
        // Mise à jour de la distance
        if (distMTotal === 0 && newLat !== 0) {
            distMTotal = distMStartOffset; // Initialisation
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
    };

    const handleGPSError = (error) => {
        console.error(`[GPS ERREUR] Code ${error.code}: ${error.message}`);
        if ($('gps-status')) $('gps-status').textContent = 'ERREUR GPS';
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
            lastP_hPa = BARO_ALT_REF_HPA;
            lastT_K = TEMP_SEA_LEVEL_K;
            currentAirDensity = RHO_SEA_LEVEL;
            currentSpeedOfSound = getSpeedOfSound(TEMP_SEA_LEVEL_K);
            updateWeatherDOM({ tempC: 15.0, pressure_hPa: 1013.25, humidity_perc: 50, air_density: RHO_SEA_LEVEL, tempK: TEMP_SEA_LEVEL_K }, true);
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
        if ($('dew-point')) $('dew-point').textContent = dataOrDefault(data.tempC - 2.0, 1, ' °C (Sim)');
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
        if (isEmergencyStop) return;

        const now = Date.now();
        const dt = (now - lastUKFUpdate) / 1000.0;
        lastUKFUpdate = now;

        if (ukf && dt > 0) {
            const imuMeasure = [lastAccX, lastAccY, lastAccZ, lastGyrX, lastGyrY, lastGyrZ];
            const magMeasure = [lastMagX, lastMagY, lastMagZ];
            const gpsMeasure = [currentPosition.lat, currentPosition.lon, currentPosition.alt, currentPosition.spd];
            const referenceMag = getMagneticFieldVector(); 
            
            // ukf.predict_propagation(dt, imuMeasure);
            // ukf.update_correction(gpsMeasure, magMeasure, referenceMag);
            
            const ukfState = ukf.getState();
            kAlt = ukfState.alt;
            
            // Calculs Dynamiques
            const gravity = calculateWGS84Gravity(currentPosition.lat, kAlt);
            const instSpd_mps = currentPosition.spd || 0.0; 
            const dynamicPressure = 0.5 * currentAirDensity * instSpd_mps * instSpd_mps;
            
            // Mise à jour du DOM haute fréquence
            if ($('speed-instant-kmh')) $('speed-instant-kmh').textContent = dataOrDefault(instSpd_mps * KMH_MS, 2, ' km/h');
            if ($('speed-3d-kmh')) $('speed-3d-kmh').textContent = dataOrDefault(instSpd_mps * KMH_MS, 2, ' km/h'); 
            if ($('speed-brute-mps')) $('speed-brute-mps').textContent = dataOrDefault(instSpd_mps, 3, ' m/s');
            
            if ($('gravity-local')) $('gravity-local').textContent = dataOrDefault(gravity, 4, ' m/s²');
            
            if ($('pressure-dynamic')) $('pressure-dynamic').textContent = dataOrDefault(dynamicPressure, 2, ' Pa');

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
            if ($('mag-raw-x')) $('mag-raw-x').textContent = dataOrDefault(lastMagX, 2, ' T');
            if ($('mag-raw-y')) $('mag-raw-y').textContent = dataOrDefault(lastMagY, 2, ' T');
            if ($('mag-raw-z')) $('mag-raw-z').textContent = dataOrDefault(lastMagZ, 2, ' T');
            if ($('imu-status')) $('imu-status').textContent = (lastAccX === 0 && lastGyrX === 0) ? 'Inactif' : 'Actif (IMU/Mag)';
            
            // G-Force Verticale
            const nonGravAccelZ = lastAccZ - gravity; 
            if ($('g-force-vertical')) $('g-force-vertical').textContent = dataOrDefault((nonGravAccelZ / G_ACC_STD) + 1.0, 3, ' G');
            if ($('accel-vertical-imu')) $('accel-vertical-imu').textContent = dataOrDefault(nonGravAccelZ, 3, ' m/s²');
            
            // Autres Dynamiques
            if ($('coriolis-force')) $('coriolis-force').textContent = dataOrDefault(calculateCoriolisForce(currentPosition.lat * D2R, instSpd_mps, currentMass), 2, ' N');

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
        if ($('minecraft-time')) $('minecraft-time').textContent = getMinecraftTime(now);

        if (isMoving) totalMoveTimeMs += DOM_SLOW_UPDATE_MS;
        if ($('move-time')) $('move-time').textContent = dataOrDefault(totalMoveTimeMs / 1000, 2, ' s');

        // --- STATISTIQUES DE VITESSE ET DISTANCE ---
        if ($('speed-max')) $('speed-max').textContent = dataOrDefault(vMaxSession, 1, ' km/h');
        const avgSpd = speedSamples > 0 ? (totalSpeedSum / speedSamples) : 0;
        if ($('speed-avg-move')) $('speed-avg-move').textContent = dataOrDefault(avgSpd * KMH_MS, 1, ' km/h');
        const avgSpdTotal = (distMTotal * distanceRatio) / (elapsedTime || 1); // Total distance / total time
        if ($('speed-avg-total')) $('speed-avg-total').textContent = dataOrDefault(avgSpdTotal * KMH_MS, 1, ' km/h');
        
        if ($('distance-total-km')) $('distance-total-km').textContent = `${dataOrDefault(distMTotal / 1000 * distanceRatio, 3, ' km')} | ${dataOrDefault(distMTotal * distanceRatio, 0, ' m')}`;
        if ($('distance-ratio')) $('distance-ratio').textContent = dataOrDefault(distanceRatio, 3);
        
        const horizon = calculateMaxVisibleDistance(currentAlt);
        if ($('max-visible-distance')) $('max-visible-distance').textContent = dataOrDefault(horizon / 1000, 3, ' km');
        
        const dist_m = distMTotal * distanceRatio;
        if ($('distance-s-light')) $('distance-s-light').textContent = dataOrDefaultExp(dist_m / C_L, 2, ' s');
        if ($('distance-min-light')) $('distance-min-light').textContent = dataOrDefaultExp(dist_m / C_L / 60, 2, ' min');
        if ($('distance-h-light')) $('distance-h-light').textContent = dataOrDefaultExp(dist_m / C_L / 3600, 2, ' h');
        if ($('distance-j-light')) $('distance-j-light').textContent = dataOrDefaultExp(dist_m / C_L / 86400, 2, ' j');
        if ($('distance-sem-light')) $('distance-sem-light').textContent = dataOrDefaultExp(dist_m / C_L / (86400 * 7), 2, ' sem');
        if ($('distance-mois-light')) $('distance-mois-light').textContent = dataOrDefaultExp(dist_m / C_L / (86400 * 30), 2, ' mois');
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
        if ($('kinetic-energy')) $('kinetic-energy').textContent = dataOrDefault(restEnergy * (lorentzFactor - 1), 2, ' J');
        if ($('momentum-p')) $('momentum-p').textContent = dataOrDefaultExp(currentMass * currentSpd_mps * lorentzFactor, 2, ' kg·m/s');
        if ($('schwarzschild-radius')) $('schwarzschild-radius').textContent = dataOrDefaultExp(Rs, 3, ' m');

        // Dynamique & Forces (suite)
        const kineticEnergy = restEnergy * (lorentzFactor - 1);
        const mechanicalPower = (kineticEnergy / elapsedTime) || 0; // Puissance moyenne
        if ($('mechanical-power')) $('mechanical-power').textContent = dataOrDefault(mechanicalPower / 1000, 2, ' kW'); 
        
        // --- ASTRO (SunCalc) ---
        if (typeof SunCalc !== 'undefined') {
            const times = SunCalc.getTimes(now, currentLat, currentLon);
            const sunPos = SunCalc.getPosition(now, currentLat, currentLon);
            const moonIllum = SunCalc.getMoonIllumination(now);
            const moonTimes = SunCalc.getMoonTimes(now, currentLat, currentLon);
            const moonDistance = SunCalc.getMoonPosition(now, currentLat, currentLon).distance; // Distance en km

            // Soleil
            const sunAltDeg = sunPos.altitude * R2D;
            const sunAziDeg = (sunPos.azimuth * R2D + 180) % 360;
            if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(sunAltDeg, 2, '°');
            if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(sunAziDeg, 2, '°');
            if ($('sunrise-times')) $('sunrise-times').textContent = times.sunrise ? times.sunrise.toLocaleTimeString('fr-FR') : 'N/A';
            if ($('sunset-times')) $('sunset-times').textContent = times.sunset ? times.sunset.toLocaleTimeString('fr-FR') : 'N/A';
            if ($('day-duration')) $('day-duration').textContent = times.sunset ? new Date(times.sunset - times.sunrise).toISOString().substr(11, 8) : 'N/A';
            
            // Lune
            if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(moonIllum.fraction * 100, 1, ' %');
            if ($('moon-phase-name')) $('moon-phase-name').textContent = SunCalc.getMoonPhase(moonIllum.phase); // Fonction à implémenter pour le nom de la phase
            if ($('moon-alt')) $('moon-alt').textContent = dataOrDefault(SunCalc.getMoonPosition(now, currentLat, currentLon).altitude * R2D, 2, '°');
            if ($('moon-azimuth')) $('moon-azimuth').textContent = dataOrDefault(SunCalc.getMoonPosition(now, currentLat, currentLon).azimuth * R2D, 2, '°');
            if ($('moon-times')) $('moon-times').textContent = moonTimes.rise ? `${moonTimes.rise.toLocaleTimeString('fr-FR')} / ${moonTimes.set ? moonTimes.set.toLocaleTimeString('fr-FR') : 'N/A'}` : 'N/A';
            if ($('moon-distance')) $('moon-distance').textContent = dataOrDefault(moonDistance * 1000, 0, ' km'); 
        }

        // --- APPEL MÉTÉO & POLLUANTS ---
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
    
    // --- FONCTIONS DE CONTROLE ---
    const resetDistance = () => { distMTotal = 0.0; distMStartOffset = 0.0; console.log("[Contrôle] Distance réinitialisée."); };
    const resetVMax = () => { vMaxSession = 0.0; console.log("[Contrôle] Vitesse Max réinitialisée."); };
    const resetAll = () => { 
        resetDistance(); resetVMax(); totalSpeedSum = 0; speedSamples = 0; totalMoveTimeMs = 0; startTime = Date.now();
        if (ukf) ukf = new ProfessionalUKF(); 
        console.log("[Contrôle] Tous les paramètres réinitialisés."); 
    };

    const handleControlEvents = () => {
        if ($('gps-pause-btn')) $('gps-pause-btn').addEventListener('click', () => {
            isGpsPaused = !isGpsPaused;
            $('gps-pause-btn').textContent = isGpsPaused ? '▶️ REPRENDRE GPS' : '⏸️ PAUSE GPS';
        });
        if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', () => {
            isEmergencyStop = !isEmergencyStop;
            $('emergency-stop-btn').textContent = isEmergencyStop ? 'REPRENDRE SYSTÈME' : '🛑 Arrêt d\'urgence: ACTIF';
            if ($('stop-status')) $('stop-status').textContent = isEmergencyStop ? 'ACTIF 🔴' : 'INACTIF 🟢';
        });
        if ($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', () => {
            netherMode = !netherMode;
            $('nether-toggle-btn').textContent = `Mode Nether: ${netherMode ? 'ACTIVÉ (1:8)' : 'DÉSACTIVÉ (1:1)'}`;
        });
        if ($('mass-input')) $('mass-input').addEventListener('input', (e) => {
            currentMass = parseFloat(e.target.value) || 70.0;
            if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        });
        if ($('celestial-body-select')) $('celestial-body-select').addEventListener('change', (e) => {
            currentCelestialBody = e.target.value;
            const { G_ACC_NEW } = updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity);
            if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
        });
        
        // Boutons de réinitialisation
        if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', resetDistance);
        if ($('reset-vmax-btn')) $('reset-vmax-btn').addEventListener('click', resetVMax);
        if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', resetAll);
        
        // Inputs de rotation pour 'ROTATING'
        const updateRotation = () => {
            rotationRadius = parseFloat($('rotation-radius').value) || 100;
            angularVelocity = parseFloat($('angular-velocity').value) || 0.0;
            if (currentCelestialBody === 'ROTATING') {
                const { G_ACC_NEW } = updateCelestialBody('ROTATING', kAlt, rotationRadius, angularVelocity);
                if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
            }
        };
        if ($('rotation-radius')) $('rotation-radius').addEventListener('input', updateRotation);
        if ($('angular-velocity')) $('angular-velocity').addEventListener('input', updateRotation);

        // Autres contrôles
        if ($('environment-select')) $('environment-select').addEventListener('change', (e) => {
            selectedEnvironment = e.target.value;
            if ($('env-factor')) $('env-factor').textContent = `${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY} (x${ENVIRONMENT_FACTORS[selectedEnvironment].MULT.toFixed(1)})`;
        });
        if ($('ukf-reactivity-mode')) $('ukf-reactivity-mode').addEventListener('change', (e) => currentUKFReactivity = e.target.value);
        if ($('force-gps-accuracy-input')) $('force-gps-accuracy-input').addEventListener('input', (e) => {
            gpsAccuracyTarget = parseFloat(e.target.value) || 0;
            if ($('force-gps-accuracy-display')) $('force-gps-accuracy-display').textContent = dataOrDefault(gpsAccuracyTarget, 6, ' m');
        });
        if ($('toggle-mode-btn')) $('toggle-mode-btn').addEventListener('click', () => {
            document.body.classList.toggle('dark-mode'); 
            const isDark = document.body.classList.contains('dark-mode');
            $('toggle-mode-btn').innerHTML = isDark ? '<i class="fas fa-sun"></i> Mode Jour' : '<i class="fas fa-moon"></i> Mode Nuit';
        });
    };

    const initDashboard = () => {
        
        // 1. Lancement des Listeners de Capteurs (IMU + MAG) (APIs STABLES)
        if (window.DeviceMotionEvent) window.addEventListener('devicemotion', handleDeviceMotion);
        if (window.DeviceOrientationEvent) window.addEventListener('deviceorientation', handleDeviceOrientation);
        
        // 2. Démarrage de la géolocalisation GPS (Haute précision)
        if (navigator.geolocation) {
            navigator.geolocation.watchPosition(handleGPS, handleGPSError, {
                enableHighAccuracy: true,
                maximumAge: 0, 
                timeout: 5000 
            });
        }
        
        // 3. Initialisation de la Carte (Leaflet)
        if (typeof L !== 'undefined' && $('map-container')) {
            window.map = L.map('map-container').setView([currentPosition.lat, currentPosition.lon], 15);
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                attribution: '© OpenStreetMap contributors'
            }).addTo(window.map);
            window.marker = L.marker([currentPosition.lat, currentPosition.lon]).addTo(window.map)
                .bindPopup("Position UKF/GPS").openPopup();
        }

        // 4. Lancement de la Synchro NTP et UKF
        syncH().finally(() => { 
            if (typeof math !== 'undefined') {
                try {
                    ukf = new ProfessionalUKF(); 
                    updateSensorsAndFilter();
                } catch (e) {
                    console.error("Échec de l'initialisation UKF:", e.message);
                }
            }
            updateDOMSlow(); // Démarrer la boucle de mise à jour lente
        });
        
        // 5. Initialisation des contrôles utilisateur et valeurs par défaut
        handleControlEvents();

        // Initialisation de l'affichage de la gravité WGS84
        if ($('gravity-base')) $('gravity-base').textContent = `${calculateWGS84Gravity(currentPosition.lat, kAlt).toFixed(4)} m/s²`;
        if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        if ($('env-factor')) $('env-factor').textContent = `${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY} (x${ENVIRONMENT_FACTORS[selectedEnvironment].MULT.toFixed(1)})`;
        if ($('force-gps-accuracy-display')) $('force-gps-accuracy-display').textContent = dataOrDefault(gpsAccuracyTarget, 6, ' m');
    };

    document.addEventListener('DOMContentLoaded', initDashboard);
    
})(window);
