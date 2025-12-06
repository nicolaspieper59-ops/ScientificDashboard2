
// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// Version Corrigée (v2025.12.06) - Synthèse complète
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

// --- CLÉS D'API & ENDPOINTS (MOCKÉES) ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";
const DOM_SLOW_UPDATE_MS = 2000; // Fréquence de rafraîchissement du DOM lent

// --- CONSTANTES PHYSIQUES ET GÉOPHYSIQUES ---
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const G_U = 6.67430e-11;    // Gravitation universelle (N·m²/kg²)
const G_ACC_STD = 9.80665;  // Gravité standard (m/s²)
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const EARTH_MASS = 5.972e24; // Masse de la Terre (kg)

// --- CONSTANTES ATMOSPHÉRIQUES (ISA STANDARD) ---
const R_SPECIFIC_AIR = 287.058; // Constante spécifique de l'air sec (J/kg·K)
const GAMMA_AIR = 1.4;          // Indice adiabatique de l'air (air sec)
const TEMP_SEA_LEVEL_K = 288.15; // 15°C en Kelvin
const BARO_ALT_REF_HPA = 1013.25; // Pression au niveau de la mer standard
const RHO_SEA_LEVEL = 1.225;    // Densité de l'air ISA (kg/m³)
const MU_DYNAMIC_AIR = 1.8e-5;  // Viscosité dynamique de l'air (Pa·s)

// --- CONSTANTES WGS84 ---
const WGS84_A = 6378137.0;  // Rayon équatorial WGS84 (m)
const WGS84_F = 1 / 298.257223563; // Aplatissement WGS84
const WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F; // Excentricité au carré

// --- CONFIGURATION D'ENVIRONNEMENT ---
const ENVIRONMENT_FACTORS = {
    NORMAL: { MULT: 1.0, DISPLAY: 'Normal' },
    URBAN: { MULT: 0.5, DISPLAY: 'Urbain (x0.5)' },
    MOUNTAIN: { MULT: 2.0, DISPLAY: 'Montagne (x2.0)' },
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
    let dragCoefficient = 1.2; // Coefficient de traînée (Exemple)
    let frontalArea = 0.5; // Surface frontale (m²)
    let netherMode = false; 
    let distMTotal = 0.0; 
    let vMaxSession = 0.0; 
    let startTime = Date.now();
    let isMoving = false;
    let totalMoveTimeMs = 0;
    let totalSpeedSum = 0;
    let speedSamples = 0;
    let isEmergencyStop = false;
    let currentCelestialBody = 'TERRE';
    let rotationRadius = 100;
    let angularVelocity = 0.0;
    
    // --- SYNCHRONISATION HORAIRE ---
    let lastNTPDate = null;     
    let lastLocalTime = null;   
    
    // --- DONNÉES DE CAPTEURS (IMU + Magnétomètre) ---
    let lastAccX = 0, lastAccY = 0, lastAccZ = 0; 
    let lastGyrX = 0, lastGyrY = 0, lastGyrZ = 0; 
    let lastMagX = 0, lastMagY = 0, lastMagZ = 0; 
    let lastPitch = 0, lastRoll = 0; 

    // --- VARIABLES MÉTÉOROLOGIQUES (CORRIGÉES) ---
    let lastP_hPa = BARO_ALT_REF_HPA; // Pression actuelle (hPa)
    let lastT_K = TEMP_SEA_LEVEL_K;   // Température actuelle (Kelvin)
    let lastH_perc = 0.5;             // Humidité relative (0.0 à 1.0)
    let currentAirDensity = RHO_SEA_LEVEL;
    let currentSpeedOfSound = getSpeedOfSound(TEMP_SEA_LEVEL_K);
    let lastKnownWeather = null;
    let lastKnownPollutants = null;
    
    // =================================================================
    // BLOC 3 : MODÈLES SCIENTIFIQUES & PHYSIQUES
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
    
    /** Calcule l'accélération gravitationnelle (WGS84). */
    function calculateWGS84Gravity(lat, alt) {
        const latRad = lat * D2R;
        const sinSqLat = Math.pow(Math.sin(latRad), 2);
        
        // Formule de la gravité normale à la surface (1980)
        const g0 = 9.780327 * (1 + 0.0053024 * sinSqLat - 0.0000058 * Math.pow(sinSqLat, 2));
        
        // Correction d'altitude (approximation)
        const radius = WGS84_A / Math.sqrt(1 - WGS84_E2 * sinSqLat);
        const g_alt = g0 * (1 - 2 * alt / radius);
        
        return g_alt;
    }
    
    /** Calcule la Force de Coriolis (Newton). */
    function calculateCoriolisForce(latRad, v_mps, m) {
        // Force Coriolis = 2 * m * Omega * V * sin(lat) (Composante latérale, simplifiée)
        return 2 * m * OMEGA_EARTH * v_mps * Math.sin(latRad);
    }

    /** Calcule le Facteur de Lorentz (γ). */
    function calculateLorentzFactor(v) {
        const beta = v / C_L;
        if (beta >= 1) return Infinity;
        return 1 / Math.sqrt(1 - beta * beta);
    }
    
    /** Calcule la Dilation Temporelle Gravitationnelle (ns/jour). */
    function calculateGravitationalDilation(altM, gravity) {
        // Dilation temps gravitationnelle = (g*h / c^2) * Jours/seconde * 1e9 (ns)
        const c2 = C_L * C_L;
        const fraction = (gravity * altM) / c2;
        const ns_per_day = fraction * 86400 * 1e9;
        return ns_per_day; 
    }
    
    /** Calcule le Rayon de Schwarzschild (m). */
    function calculateSchwarzschildRadius(m) {
        return (2 * G_U * m) / (C_L * C_L);
    }

    /** Modèle Bio/SVT : Calculs Psychrométriques et Écologiques Simples. */
    function calculateBioSVT(tempC, altM, humidity_perc, pressure_Pa, sunAltitudeRad) {
        
        // 1. Point de Rosée (Formule Magnus-Tetens, approximation)
        const P_sat = 6.11 * Math.pow(10, (7.5 * tempC) / (237.7 + tempC));
        const P_vap = humidity_perc / 100 * P_sat;
        const B = 237.7;
        const C_magnus = 17.27;
        const dewPoint = (B * Math.log(P_vap / 6.11)) / (C_magnus - Math.log(P_vap / 6.11));

        // 2. Température du Bulbe Humide (Formule simplifiée/Approximation)
        const wetBulbTemp = tempC * Math.atan(0.151977 * Math.sqrt(humidity_perc + 8.313659)) + 
                            Math.atan(tempC + humidity_perc) - Math.atan(humidity_perc - 1.676331) + 
                            0.00391838 * Math.pow(humidity_perc, 1.5) * Math.atan(0.023101 * humidity_perc) - 4.686035;

        // 3. CAPE (Convective Available Potential Energy) - MOCK
        // CAPE est complexe, nous simulons une valeur basse ou nulle
        const CAPE = tempC > 25 && humidity_perc > 70 ? 1000 * (tempC / 30) * (humidity_perc / 100) : 0;
        
        // 4. Taux de Photosynthèse (MOCK)
        const lightMax = 100000; // Lux max pour le soleil (arbitraire)
        const lightAmbient = Math.max(0, Math.sin(sunAltitudeRad)) * lightMax;
        const photoRate = (lightAmbient / 10000) * (tempC > 5 && tempC < 35 ? 1 : 0.5); 

        // 5. Humidité Absolue (MOCK, plus complexe, basé sur la densité de vapeur d'eau)
        const absoluteHumidity = (P_vap / (R_SPECIFIC_AIR * (tempC + 273.15))) * 1000; // g/m³ (Approximation)

        return {
            dewPoint, 
            wetBulbTemp,
            CAPE: CAPE,
            saturationO2: 100 * (1 - altM / 8000), // Simulation de la baisse avec l'altitude
            photosynthesisRate: photoRate,
            absoluteHumidity: absoluteHumidity
        };
    }

    // --- MOCK UKF (Doit utiliser la bibliothèque math.js) ---
    class ProfessionalUKF {
        constructor() {
            this.STATE_SIZE = 21;
            if (typeof math === 'undefined') {
                 console.error("math.js non disponible. UKF désactivé/Mocké.");
                 this.x = { get: (i) => i === 2 ? currentPosition.alt : (i === 0 ? currentPosition.lat * D2R : (i === 1 ? currentPosition.lon * D2R : 0)) }; 
                 this.getState = () => ({ lat: currentPosition.lat, lon: currentPosition.lon, alt: currentPosition.alt, vel_up: 0 });
                 return;
            }
            this.x = math.zeros(this.STATE_SIZE); 
            // Initialisation des états de position et d'attitude (quaternion q1=1)
            this.x.set([0], currentPosition.lat * D2R);
            this.x.set([1], currentPosition.lon * D2R);
            this.x.set([2], currentPosition.alt);
            this.x.set([6], 1.0); 
        }
        predict_propagation(dt, imuMeasure) {
            // MOCK: Propagation simplifiée (ajoute l'accélération)
            const accZ_gravity_compensated = imuMeasure[2] - G_ACC_STD; // Acc Z brute - Gravité
            this.x.set([2], this.x.get([2]) + this.x.get([5]) * dt + accZ_gravity_compensated * dt * dt / 2); 
            this.x.set([5], this.x.get([5]) + accZ_gravity_compensated * dt); // MOCK: Vitesse verticale
        }
        update_correction(gpsMeasure) {
            // MOCK: Correction simple de la position
            this.x.set([0], this.x.get([0]) * 0.99 + gpsMeasure[0] * D2R * 0.01);
            this.x.set([1], this.x.get([1]) * 0.99 + gpsMeasure[1] * D2R * 0.01);
            this.x.set([2], this.x.get([2]) * 0.9 + gpsMeasure[2] * 0.1); 
        }
        getState() {
            return {
                lat: this.x.get([0]) * R2D,
                lon: this.x.get([1]) * R2D,
                alt: this.x.get([2]),
                vel_up: this.x.get([5]) || 0, 
            };
        }
    }

    // =================================================================
    // BLOC 4 : GESTIONNAIRES DE CAPTEURS & API
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
    
    /** Gère l'Accéléromètre et le Gyroscope (DeviceMotionEvent). */
    const handleDeviceMotion = (event) => {
        // Enregistre les accélérations brutes (avec gravité) et les vitesses angulaires
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
        // Enregistre le cap (alpha) et les angles de roulis/tangage (gamma/beta)
        if (event.absolute && event.alpha !== undefined) {
            currentPosition.heading = (event.alpha || 0);
            lastPitch = event.beta || 0;    
            lastRoll = event.gamma || 0;    
        } else if (event.webkitCompassHeading !== undefined) {
             currentPosition.heading = event.webkitCompassHeading;
             lastPitch = event.beta || 0;
             lastRoll = event.gamma || 0;
        }
        if ($('imu-status').textContent === 'Inactif') $('imu-status').textContent = 'Actif (IMU/Mag)';
        
        // MOCK: Mise à jour des champs magnétiques (pour l'affichage)
        lastMagX = Math.random() * 50000 * 1e-9;
        lastMagY = Math.random() * 50000 * 1e-9;
        lastMagZ = Math.random() * 50000 * 1e-9;
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
            } else {
                 // Fallback simple: distance euclidienne (très imprécis)
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
        
        // Mise à jour de la carte (Leaflet)
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

    /** Synchronise l'heure du système avec un serveur NTP/UTC. */
    const syncH = async () => {
        if ($('local-time')) $('local-time').textContent = 'Synchronisation...';
        try {
            const response = await fetch(SERVER_TIME_ENDPOINT);
            const data = await response.json();
            
            lastNTPDate = new Date(data.datetime);
            lastLocalTime = Date.now(); 
            console.log(`[NTP Sync OK]`);
        } catch (error) {
            console.error(`[NTP Sync ÉCHEC]`);
            if ($('local-time')) $('local-time').textContent = `SYNCHRO ÉCHOUÉE`;
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
            lastH_perc = humidity_perc / 100.0;
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
            console.warn(`[Météo Échec] Retour aux valeurs ISA par défaut.`);
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
        if ($('weather-status')) $('weather-status').textContent = isDefault ? 'HORS LIGNE/DÉFAUT' : 'ACTIF';
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
        if (isEmergencyStop) {
            requestAnimationFrame(updateSensorsAndFilter); 
            return;
        }

        const now = Date.now();
        const dt = (now - lastUKFUpdate) / 1000.0;
        lastUKFUpdate = now;
        
        const instSpd_mps = currentPosition.spd || 0.0; 

        if (ukf && dt > 0) {
            // Entrées : IMU (Accélération), GPS (Position/Vitesse)
            const imuMeasure = [lastAccX, lastAccY, lastAccZ, lastGyrX, lastGyrY, lastGyrZ];
            const gpsMeasure = [currentPosition.lat, currentPosition.lon, currentPosition.alt, instSpd_mps];
            
            ukf.predict_propagation(dt, imuMeasure);
            ukf.update_correction(gpsMeasure);
            
            const ukfState = ukf.getState();
            kAlt = ukfState.alt;
            
            // --- Calculs Dynamiques Basés sur UKF/Météo ---
            const gravity = calculateWGS84Gravity(ukfState.lat, kAlt); // G corrigée par WGS84
            const dynamicPressure = 0.5 * currentAirDensity * instSpd_mps * instSpd_mps;
            const coriolisForce = calculateCoriolisForce(ukfState.lat * D2R, instSpd_mps, currentMass);
            const machNumber = instSpd_mps / currentSpeedOfSound; // Mach corrigé par Météo
            const nonGravAccelZ = lastAccZ - gravity; 
            
            // --- Mise à jour du DOM haute fréquence ---
            
            // Vitesse & Accélération
            if ($('speed-instant-kmh')) $('speed-instant-kmh').textContent = dataOrDefault(instSpd_mps * KMH_MS, 2, ' km/h');
            if ($('accel-vertical-imu')) $('accel-vertical-imu').textContent = dataOrDefault(nonGravAccelZ, 3, ' m/s²');
            
            // UKF States
            if ($('ukf-lat')) $('ukf-lat').textContent = dataOrDefault(ukfState.lat, 6, '°');
            if ($('ukf-lon')) $('ukf-lon').textContent = dataOrDefault(ukfState.lon, 6, '°');
            if ($('ukf-alt')) $('ukf-alt').textContent = dataOrDefault(kAlt, 2, ' m');
            if ($('vertical-speed-ekf')) $('vertical-speed-ekf').textContent = dataOrDefault(ukfState.vel_up || 0, 3, ' m/s');
            
            // Dynamique/Forces
            if ($('gravity-local')) $('gravity-local').textContent = dataOrDefault(gravity, 4, ' m/s²');
            if ($('g-force-vertical')) $('g-force-vertical').textContent = dataOrDefault((nonGravAccelZ / G_ACC_STD) + 1.0, 3, ' G');
            if ($('pressure-dynamic')) $('pressure-dynamic').textContent = dataOrDefault(dynamicPressure, 2, ' Pa');
            if ($('drag-force')) $('drag-force').textContent = dataOrDefault(dynamicPressure * dragCoefficient * frontalArea, 2, ' N');
            if ($('coriolis-force')) $('coriolis-force').textContent = dataOrDefault(coriolisForce, 2, ' N');
            
            // IMU & Niveau à Bulle
            if ($('pitch-display')) $('pitch-display').textContent = dataOrDefault(lastPitch, 1, '°');
            if ($('roll-display')) $('roll-display').textContent = dataOrDefault(lastRoll, 1, '°');
            if ($('acc-raw-z')) $('acc-raw-z').textContent = dataOrDefault(lastAccZ, 3, ' m/s²');
            if ($('mag-raw-z')) $('mag-raw-z').textContent = dataOrDefault(lastMagZ * 1e9, 2, ' nT');
            if ($('gps-accuracy-display')) $('gps-accuracy-display').textContent = dataOrDefault(currentPosition.acc, 2, ' m'); // Précision GPS (Acc)

            // Mach Number
            if ($('mach-number')) $('mach-number').textContent = dataOrDefault(machNumber, 4);

        } else if (ukf && ukf.getState) {
             // Mise à jour minimale si math.js est en échec mais IMU est actif
             if ($('pitch-display')) $('pitch-display').textContent = dataOrDefault(lastPitch, 1, '°');
             if ($('roll-display')) $('roll-display').textContent = dataOrDefault(lastRoll, 1, '°');
             if ($('gps-accuracy-display')) $('gps-accuracy-display').textContent = dataOrDefault(currentPosition.acc, 2, ' m');
        }
        
        requestAnimationFrame(updateSensorsAndFilter); 
    };

    /** Boucle de mise à jour lente du DOM (Astro, Météo, Relativité, SVT). */
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
        
        // Mise à jour du temps écoulé
        if ($('elapsed-time')) $('elapsed-time').textContent = dataOrDefault(elapsedTime, 2, ' s');
        if (isMoving) totalMoveTimeMs += DOM_SLOW_UPDATE_MS;
        if ($('move-time')) $('move-time').textContent = dataOrDefault(totalMoveTimeMs / 1000, 2, ' s');

        // --- RELATIVITÉ (Vitesse Lumière CORRIGÉE) ---
        const lorentzFactor = calculateLorentzFactor(currentSpd_mps);
        const restEnergy = currentMass * C_L * C_L;
        const kineticEnergy = restEnergy * (lorentzFactor - 1);
        const Rs = calculateSchwarzschildRadius(currentMass);
        
        if ($('%-speed-of-light')) $('%-speed-of-light').textContent = dataOrDefaultExp(currentSpd_mps / C_L * 100, 2, ' %');
        if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(lorentzFactor, 4);
        if ($('dilation-time-speed')) $('dilation-time-speed').textContent = dataOrDefault(Math.max(0, (lorentzFactor - 1) * 86400 * 1e9), 2, ' ns/j'); 
        if ($('dilation-time-gravity')) $('dilation-time-gravity').textContent = dataOrDefault(calculateGravitationalDilation(currentAlt, calculateWGS84Gravity(currentLat, currentAlt)), 2, ' ns/j'); 
        
        if ($('energy-rel')) $('energy-rel').textContent = dataOrDefaultExp(lorentzFactor * restEnergy, 2, ' J');
        if ($('energy-rest')) $('energy-rest').textContent = dataOrDefaultExp(restEnergy, 2, ' J');
        if ($('kinetic-energy')) $('kinetic-energy').textContent = dataOrDefault(kineticEnergy, 2, ' J');
        if ($('momentum-p')) $('momentum-p').textContent = dataOrDefaultExp(currentMass * currentSpd_mps * lorentzFactor, 2, ' kg·m/s');
        if ($('schwarzschild-radius')) $('schwarzschild-radius').textContent = dataOrDefaultExp(Rs, 3, ' m');

        // --- ASTRO & BIO/SVT ---
        let sunAltitudeRad = 0;
        if (typeof SunCalc !== 'undefined') {
            const sunPos = SunCalc.getPosition(now, currentLat, currentLon);
            sunAltitudeRad = sunPos.altitude;
            if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(sunPos.altitude * R2D, 2, '°');
            if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault((sunPos.azimuth * R2D + 180) % 360, 2, '°');
            // ... (autres mises à jour Astro)
        } 
        
        // --- BIO/SVT (CHIMIE) ---
        if (lastKnownWeather) {
            const pressure_Pa = lastKnownWeather.pressure_hPa * 100;
            const bioSim = calculateBioSVT(lastKnownWeather.tempC, currentAlt, lastKnownWeather.humidity_perc, pressure_Pa, sunAltitudeRad);
            
            if ($('dew-point')) $('dew-point').textContent = dataOrDefault(bioSim.dewPoint, 1, ' °C');
            if ($('abs-humidity-sim')) $('abs-humidity-sim').textContent = dataOrDefault(bioSim.absoluteHumidity, 3, ' g/m³');
            if ($('wet-bulb-temp-sim')) $('wet-bulb-temp-sim').textContent = dataOrDefault(bioSim.wetBulbTemp, 1, ' °C');
            if ($('cape-sim')) $('cape-sim').textContent = dataOrDefault(bioSim.CAPE, 0, ' J/kg');
            if ($('o2-sat-sim')) $('o2-sat-sim').textContent = dataOrDefault(bioSim.saturationO2, 1, ' %');
            if ($('photo-rate-sim')) $('photo-rate-sim').textContent = dataOrDefault(bioSim.photosynthesisRate, 2, ' %/h');
        }

        // Fetch Météo/Polluants (mis à jour à chaque boucle lente)
        if (currentLat !== 0 && currentLon !== 0) {
            fetchWeather(currentLat, currentLon); 
            fetchPollutants(); 
        }

        setTimeout(updateDOMSlow, DOM_SLOW_UPDATE_MS);
    };

    // =================================================================
    // BLOC 6 : INITIALISATION ET CONTRÔLES
    // =================================================================
    
    const handleControlEvents = () => {
        // [Contrôles]
        if ($('gps-start-btn')) $('gps-start-btn').addEventListener('click', () => { 
            isGpsPaused = !isGpsPaused;
            $('gps-start-btn').textContent = isGpsPaused ? '▶️ MARCHE GPS' : '⏸️ PAUSE GPS';
        });
        if ($('mass-input')) $('mass-input').addEventListener('input', (e) => {
            currentMass = parseFloat(e.target.value) || 70.0;
            if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        });
        if ($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', () => {
            netherMode = !netherMode;
            $('nether-toggle-btn').textContent = `Mode Nether: ${netherMode ? 'ACTIVÉ (1:8)' : 'DÉSACTIVÉ (1:1)'}`;
        });
        
        // IMU/Motion Permission (bouton pour iOS/modernes)
        if ($('imu-request-btn')) $('imu-request-btn').addEventListener('click', requestMotionPermission);
        if ($('imu-status').textContent === 'Inactif') {
             // Tente de lancer les listeners sans demander la permission si possible
             requestMotionPermission();
        }
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
        if (typeof math !== 'undefined') {
            ukf = new ProfessionalUKF(); 
        } else {
            console.warn("Erreur: math.js n'a pas pu être chargé. Le filtre UKF est désactivé et mocké.");
        }
        
        // 4. Démarrer les boucles de mise à jour
        updateSensorsAndFilter();
        updateDOMSlow(); 
        
        // 5. Initialisation des affichages (pour éviter les N/A au chargement)
        if ($('gravity-base')) $('gravity-base').textContent = `${calculateWGS84Gravity(currentPosition.lat, kAlt).toFixed(4)} m/s²`;
        if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        if ($('vitesse-son-locale')) $('vitesse-son-locale').textContent = `${currentSpeedOfSound.toFixed(2)} m/s`;
    };

    document.addEventListener('DOMContentLoaded', initDashboard);
    
})(window);
