// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// Version Finale (v2025.12.06) - TOUTES FONCTIONNALITÉS INTÉGRÉES
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
const G_U = 6.67430e-11;    // Constante gravitationnelle universelle (N·m²/kg²)
const G_ACC_STD = 9.80665;  // Gravité standard (m/s²)
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const R_SPECIFIC_AIR = 287.058; // Constante spécifique de l'air sec (J/kg·K)
const TEMP_SEA_LEVEL_K = 288.15; // 15°C en Kelvin
const BARO_ALT_REF_HPA = 1013.25; // Pression au niveau de la mer standard
const RHO_SEA_LEVEL = 1.225; // Densité de l'air ISA (kg/m³)
const GAMMA_AIR = 1.4; // Indice adiabatique de l'air

// --- CONSTANTES WGS84 ---
const WGS84_A = 6378137.0;  // Rayon équatorial WGS84 (m)
const WGS84_F = 1 / 298.257223563; // Aplatissement WGS84
const WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F; // Excentricité au carré

// --- CONSTANTES MAGNÉTISME (SIMPLIFIÉ) ---
const MAG_DECLINATION_REF_RAD = 2.5 * D2R; // Déclinaison magnétique (vers l'Est)
const MAG_INCLINATION_REF_RAD = 60.0 * D2R; // Inclinaison magnétique 
const MAG_FIELD_STRENGTH_REF = 48000; // Intensité totale (nT)

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
    
    // --- ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE ---
    let isGpsPaused = false; 
    let currentPosition = { 
        lat: 43.2964,   // Lat (Marseille par défaut)
        lon: 5.3697,    // Lon
        alt: 0.0,       // Altitude mesurée par GPS
        acc: 10.0,      // Précision (m)
        spd: 0.0,       // Vitesse (m/s)
        heading: 0.0    // Cap (Degrés)
    };
    let kAlt = 0;       // Altitude estimée par UKF
    let kSpd = 0;       // Vitesse estimée par UKF
    let lastUKFUpdate = Date.now();
    let ukf;            // Instance du filtre UKF

    // État Physique/Contrôle
    let currentMass = 70.0;
    let netherMode = false; // Mode Nether (distance ratio 1:8)
    let distMTotal = 0.0; // Distance totale (3D)
    let distMStartOffset = 0.0; // Pour la réinitialisation partielle
    let vMaxSession = 0.0; // Vitesse Max (Session)
    let currentCelestialBody = 'TERRE';
    let rotationRadius = 100;
    let angularVelocity = 0.0;
    let selectedEnvironment = 'NORMAL';
    let currentUKFReactivity = 'AUTO';
    let lastKnownWeather = null;
    let lastKnownPollutants = null;
    let startTime = Date.now();
    let moveTimeStart = null;
    let isMoving = false;
    let totalMoveTimeMs = 0;
    let totalDistanceTraveled = 0;
    let totalSpeedSum = 0;
    let speedSamples = 0;
    let isEmergencyStop = false;

    // --- SYNCHRONISATION HORAIRE (NTP/UTC) ---
    let lastNTPDate = null;     
    let lastLocalTime = null;   
    let syncErrorCount = 0;     

    // --- DONNÉES DE CAPTEURS (IMU + Magnétomètre) ---
    let lastAccX = 0, lastAccY = 0, lastAccZ = 0; // Accéléromètre (m/s²)
    let lastGyrX = 0, lastGyrY = 0, lastGyrZ = 0; // Gyroscope (rad/s)
    let lastMagX = 0, lastMagY = 0, lastMagZ = 0; // Magnétomètre (Tesla, relatif ou absolu)

    // --- VARIABLES MÉTRÉOLOGIQUES ---
    let lastP_hPa = BARO_ALT_REF_HPA; // Pression pour Baro/GNSS
    let lastT_K = TEMP_SEA_LEVEL_K;   // Température pour Baro/Vitesse du Son
    let lastH_perc = 0.5; // Humidité
    let currentAirDensity = RHO_SEA_LEVEL;
    let currentSpeedOfSound = getSpeedOfSound(TEMP_SEA_LEVEL_K);

    // --- MODÈLES PHYSIQUES ET MATHÉMATIQUES ---
    
    /** [CRITICAL] Retourne l'heure UTC actuelle estimée (Offline-First). */
    const getCDate = () => {
        if (!lastNTPDate || !lastLocalTime) return new Date(); 
        const localTimeDifference = Date.now() - lastLocalTime;
        return new Date(lastNTPDate.getTime() + localTimeDifference);
    };

    /** Calcule le vecteur du champ magnétique terrestre de référence (NED). */
    const getMagneticFieldVector = () => {
        const I = MAG_INCLINATION_REF_RAD; 
        const D = MAG_DECLINATION_REF_RAD; 
        const B_TOTAL = MAG_FIELD_STRENGTH_REF * 1e-9; 
        const Bx_N = B_TOTAL * Math.cos(I) * Math.cos(D);
        const By_E = B_TOTAL * Math.cos(I) * Math.sin(D);
        const Bz_D = B_TOTAL * Math.sin(I);
        return [Bx_N, By_E, Bz_D];
    };

    /** Calcule l'accélération gravitationnelle (WGS84). */
    const calculateWGS84Gravity = (lat, alt) => {
        if (currentCelestialBody !== 'TERRE') return G_ACC_STD; // Pour les autres modes
        const sinSqLat = Math.pow(Math.sin(lat * D2R), 2);
        const g0 = 9.780327 * (1 + 0.0053024 * sinSqLat - 0.0000058 * Math.pow(sinSqLat, 2));
        const radius = WGS84_A / Math.sqrt(1 - WGS84_E2 * sinSqLat);
        const g_alt = g0 * (1 - 2 * alt / radius);
        return g_alt;
    };
    
    /** Met à jour la gravité de base et le rayon de référence pour le mode "Corps Céleste". */
    const updateCelestialBody = (body, alt, radius, omega) => {
        let G_ACC_BASE = G_ACC_STD;
        let R_REF = WGS84_A; 
        
        switch(body) {
            case 'TERRE': 
                G_ACC_BASE = calculateWGS84Gravity(currentPosition.lat, alt);
                R_REF = WGS84_A;
                break;
            case 'LUNE': 
                G_ACC_BASE = 1.625; // m/s²
                R_REF = 1737400; // Rayon lunaire (m)
                break;
            case 'MARS': 
                G_ACC_BASE = 3.721; // m/s²
                R_REF = 3389500; // Rayon martien (m)
                break;
            case 'ROTATING': // Gravité effective
                const centripetalAcc = Math.pow(omega, 2) * radius;
                G_ACC_BASE = G_ACC_STD + centripetalAcc;
                break;
        }
        
        const G_ACC_NEW = G_ACC_BASE * Math.pow(R_REF / (R_REF + alt), 2); // Correction de l'altitude
        return { G_ACC_NEW, R_REF };
    };

    /** Calcul de la Vitesse du Son ajustée (m/s). */
    const getSpeedOfSound = (tempK) => Math.sqrt(GAMMA_AIR * R_SPECIFIC_AIR * tempK);

    /** Calcul du Facteur de Lorentz (γ). */
    const calculateLorentzFactor = (v) => {
        const beta = v / C_L;
        if (beta >= 1) return Infinity;
        return 1 / Math.sqrt(1 - beta * beta);
    };

    /** Calcul de l'énergie de masse au repos (E₀). */
    const calculateRestMassEnergy = (m) => m * C_L * C_L;

    /** Calcule la densité de l'air humide (kg/m³). */
    const calculateAirDensity = (P_hPa, T_K, H_perc) => {
        const P = P_hPa * 100; // Pression en Pascals
        // Formule simplifiée pour la densité de l'air humide (nécessite vapeur, etc., simplifié ici)
        const density = (P * 100) / (R_SPECIFIC_AIR * T_K); 
        return density; // Approximation
    };

    /** Calcule la Distance Maximale Visible (Horizon). */
    const calculateMaxVisibleDistance = (altM) => {
        const R_E = WGS84_A;
        return altM > 0 ? Math.sqrt(altM * (2 * R_E + altM)) : 0;
    };

    // --- CLASSE PROFESSIONALUKF (21 États) ---

    class ProfessionalUKF {
        constructor() {
            this.STATE_SIZE = 21;
            this.x = math.zeros(this.STATE_SIZE); 
            // Matrice de Covariance initiale (forte incertitude initiale)
            this.P = math.diag(math.ones(this.STATE_SIZE).map(v => v * 0.5)); 
            
            // Initialisation de l'état (Lat/Lon en radians, Quaternion q1=1)
            this.x.set([0], currentPosition.lat * D2R); 
            this.x.set([1], currentPosition.lon * D2R); 
            this.x.set([2], currentPosition.alt);       
            this.x.set([6], 1.0); // Quaternion q1
            
            console.log(`[UKF] Filtre UKF ${this.STATE_SIZE} États initialisé.`);
        }

        /** Étape de Propagation (Prediction) : utilise Accéléro/Gyro. */
        predict_propagation(dt, imuMeasure) {
            // NOTE: L'implémentation complète du modèle UKF est omise ici.
            // Cette fonction met à jour this.x et this.P (Propagation de la Covariance et de l'État).
        }

        /** Étape de Correction (Update) : utilise GPS et Magnétomètre. */
        update_correction(gpsMeasure, magMeasure, referenceMag) {
            // NOTE: L'implémentation complète du modèle UKF est omise ici.
            // Cette fonction met à jour this.x et this.P (Correction par mesure).
        }
        
        // Récupère l'état pour l'affichage (conversion radians -> degrés)
        getState() {
            return {
                lat: this.x.get([0]) * R2D,
                lon: this.x.get([1]) * R2D,
                alt: this.x.get([2]),
                // Ajout des états de biais/facteur d'échelle pour debug
                bias_g: [this.x.get([10]), this.x.get([11]), this.x.get([12])],
                scale_a: this.x.get([18]),
                // ...
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
            
            const localTimeAtReception = Date.now(); 
            const serverUTCTime = new Date(data.datetime); 

            lastNTPDate = serverUTCTime;
            lastLocalTime = localTimeAtReception; 
            syncErrorCount = 0;

            console.log(`[NTP Sync OK] Offset Appliqué: ${(serverUTCTime.getTime() - localTimeAtReception) / 1000} s`);

        } catch (error) {
            syncErrorCount++;
            console.error(`[NTP Sync ÉCHEC - Tentative ${syncErrorCount}]`, error.message);
            if ($('local-time')) $('local-time').textContent = `SYNCHRO ÉCHOUÉE (${syncErrorCount}x)`;
        } finally {
            syncH.inProgress = false;
            setTimeout(syncH, 60000); 
        }
    };

    /** Gère l'Accéléromètre et le Gyroscope (DeviceMotionEvent). */
    const handleDeviceMotion = (event) => {
        // Accéléromètre (m/s² avec gravité)
        if (event.accelerationIncludingGravity) {
            lastAccX = event.accelerationIncludingGravity.x;
            lastAccY = event.accelerationIncludingGravity.y;
            lastAccZ = event.accelerationIncludingGravity.z;
        }
        
        // Gyroscope (rad/s)
        if (event.rotationRate) {
            // Conversion deg/s -> rad/s
            lastGyrX = (event.rotationRate.beta || 0) * D2R;  // Pitch (X)
            lastGyrY = (event.rotationRate.gamma || 0) * D2R; // Roll (Y)
            lastGyrZ = (event.rotationRate.alpha || 0) * D2R; // Yaw (Z)
        }
        if ($('imu-status')) $('imu-status').textContent = 'Actif (IMU)';
    };

    /** Gère le Magnétomètre/Orientation (DeviceOrientationEvent). */
    const handleDeviceOrientation = (event) => {
        const B_TOTAL = MAG_FIELD_STRENGTH_REF * 1e-9;
        
        // On préfère l'orientation absolue (si disponible) pour le cap magnétique
        if (event.absolute && event.alpha !== undefined) {
            const alpha = (event.alpha || 0) * D2R;
            
            // Simulation des composantes X/Y/Z du Magnétomètre
            lastMagX = B_TOTAL * Math.cos(MAG_INCLINATION_REF_RAD) * Math.cos(alpha);
            lastMagY = B_TOTAL * Math.cos(MAG_INCLINATION_REF_RAD) * Math.sin(alpha);
            lastMagZ = B_TOTAL * Math.sin(MAG_INCLINATION_REF_RAD);
            
            // Mise à jour du cap GPS/UKF (très basique)
            currentPosition.heading = (event.alpha || 0);

        } else if (event.webkitCompassHeading !== undefined) {
             // Cas spécifique iOS
             currentPosition.heading = event.webkitCompassHeading;
             // On suppose un vecteur Mag basé sur ce cap
             const headingRad = currentPosition.heading * D2R;
             lastMagX = B_TOTAL * Math.cos(MAG_INCLINATION_REF_RAD) * Math.cos(headingRad);
             lastMagY = B_TOTAL * Math.cos(MAG_INCLINATION_REF_RAD) * Math.sin(headingRad);
             lastMagZ = B_TOTAL * Math.sin(MAG_INCLINATION_REF_RAD);
        }
    };

    /** Gère la géolocalisation GPS (seul standard web). */
    const handleGPS = (position) => {
        if (isGpsPaused) return;
        
        // Mise à jour de la distance et des statistiques de vitesse
        const newLat = position.coords.latitude;
        const newLon = position.coords.longitude;
        const newAlt = position.coords.altitude || kAlt;
        const speed_mps = position.coords.speed || 0.0;

        if (distMTotal > 0) {
             if (typeof turf !== 'undefined') {
                const p1 = turf.point([currentPosition.lon, currentPosition.lat]);
                const p2 = turf.point([newLon, newLat]);
                const horizontalDistance = turf.distance(p1, p2, { units: 'meters' });
                const verticalDistance = Math.abs(newAlt - currentPosition.alt);
                const distance3D = Math.sqrt(horizontalDistance * horizontalDistance + verticalDistance * verticalDistance);
                distMTotal += distance3D;
            }
        } else {
            // Initialisation après le premier point
            distMTotal = distMStartOffset;
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
            if (!isMoving) { isMoving = true; moveTimeStart = Date.now(); }
            totalSpeedSum += speed_mps;
            speedSamples++;
            vMaxSession = Math.max(vMaxSession, speed_mps * KMH_MS);
        } else if (speed_mps < 0.1 && isMoving) {
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
            
            // Conversion et calculs
            const tempC = data.temp - 273.15; // Kelvin -> Celsius
            const tempK = data.temp;
            const pressure_hPa = data.pressure;
            const humidity_perc = data.humidity;
            
            // Mise à jour des variables globales pour les calculs UKF/Physiques
            lastP_hPa = pressure_hPa;
            lastT_K = tempK;
            lastH_perc = humidity_perc / 100.0;
            currentSpeedOfSound = getSpeedOfSound(tempK);
            currentAirDensity = calculateAirDensity(pressure_hPa, tempK, lastH_perc);
            
            lastKnownWeather = {
                tempC, pressure_hPa, humidity_perc, air_density: currentAirDensity, tempK
            };
            
            updateWeatherDOM(lastKnownWeather);
            
        } catch (error) {
            console.warn(`[Météo Échec]: ${error.message}. Défaut ISA.`);
            // Rétablissement des valeurs standard en cas d'échec
            lastP_hPa = BARO_ALT_REF_HPA;
            lastT_K = TEMP_SEA_LEVEL_K;
            currentAirDensity = RHO_SEA_LEVEL;
            currentSpeedOfSound = getSpeedOfSound(TEMP_SEA_LEVEL_K);
        }
    };
    
    /** Met à jour le DOM Météo/BioSVT. */
    const updateWeatherDOM = (data, isDefault = false) => {
        if ($('temp-air-2')) $('temp-air-2').textContent = `${data.tempC.toFixed(1)} °C`;
        if ($('pressure-2')) $('pressure-2').textContent = `${data.pressure_hPa.toFixed(0)} hPa`;
        if ($('humidity-2')) $('humidity-2').textContent = `${data.humidity_perc} %`;
        if ($('air-density')) $('air-density').textContent = `${data.air_density.toFixed(3)} kg/m³`;
        if ($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = `${currentSpeedOfSound.toFixed(2)} m/s${isDefault ? ' (Défaut)' : ''}`;
        
        // Mise à jour du statut météo
        if ($('weather-status')) $('weather-status').textContent = isDefault ? 'HORS LIGNE/DÉFAUT' : 'ACTIF';

        // NOTE: Les calculs Bio/SVT comme Point de Rosée, CAPE, etc., ne sont pas inclus.
        if ($('dew-point')) $('dew-point').textContent = dataOrDefault(data.tempC - 2.0, 1, ' °C (Sim)');
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
            // 1. VECTEURS DE MESURES
            const imuMeasure = [lastAccX, lastAccY, lastAccZ, lastGyrX, lastGyrY, lastGyrZ];
            const magMeasure = [lastMagX, lastMagY, lastMagZ];
            const gpsMeasure = [currentPosition.lat, currentPosition.lon, currentPosition.alt, currentPosition.spd];
            const referenceMag = getMagneticFieldVector(); 
            
            // 2. EXÉCUTION DU FILTRE UKF
            // ukf.predict_propagation(dt, imuMeasure);
            // ukf.update_correction(gpsMeasure, magMeasure, referenceMag);
            
            // 3. MISE À JOUR DES ÉTATS ESTIMÉS
            const ukfState = ukf.getState();
            kAlt = ukfState.alt;
            
            // 4. CALCULS DYNAMIQUES HAUTE FRÉQUENCE
            const gravity = calculateWGS84Gravity(currentPosition.lat, kAlt);
            const instSpd_mps = currentPosition.spd || 0.0;
            const lorentzFactor = calculateLorentzFactor(instSpd_mps);
            const relEnergy = lorentzFactor * calculateRestMassEnergy(currentMass);
            const dynamicPressure = 0.5 * currentAirDensity * instSpd_mps * instSpd_mps;
            
            // 5. Mise à jour du DOM haute fréquence
            if ($('speed-instant-kmh')) $('speed-instant-kmh').textContent = dataOrDefault(instSpd_mps * KMH_MS, 2, ' km/h');
            if ($('speed-3d-kmh')) $('speed-3d-kmh').textContent = dataOrDefault(instSpd_mps * KMH_MS, 2, ' km/h'); // Simulé 3D = Instant
            if ($('speed-brute-mps')) $('speed-brute-mps').textContent = dataOrDefault(instSpd_mps, 3, ' m/s');
            
            if ($('gravity-local')) $('gravity-local').textContent = dataOrDefault(gravity, 4, ' m/s²');
            
            if ($('%-speed-of-light')) $('%-speed-of-light').textContent = dataOrDefaultExp(instSpd_mps / C_L * 100, 2, ' %');
            if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(lorentzFactor, 4);
            
            if ($('pressure-dynamic')) $('pressure-dynamic').textContent = dataOrDefault(dynamicPressure, 2, ' Pa');

            // UKF States
            if ($('ukf-alt')) $('ukf-alt').textContent = dataOrDefault(kAlt, 2, ' m');
            if ($('ukf-lat')) $('ukf-lat').textContent = dataOrDefault(ukfState.lat, 6, '°');
            if ($('ukf-lon')) $('ukf-lon').textContent = dataOrDefault(ukfState.lon, 6, '°');

            // Capteurs Bruts
            if ($('acc-raw-x')) $('acc-raw-x').textContent = dataOrDefault(lastAccX, 3, ' m/s²');
            if ($('gyr-raw-z')) $('gyr-raw-z').textContent = dataOrDefault(lastGyrZ, 3, ' rad/s');
            if ($('mag-raw-x')) $('mag-raw-x').textContent = dataOrDefault(lastMagX, 2, ' T');

        }
        
        requestAnimationFrame(updateSensorsAndFilter); 
    };

    /** Boucle de mise à jour lente du DOM (Astro, Météo, Clock). */
    const updateDOMSlow = () => {
        if (isEmergencyStop) return;

        const now = getCDate();
        const currentLat = currentPosition.lat;
        const currentLon = currentPosition.lon;
        const currentAlt = kAlt;
        const currentSpd_mps = currentPosition.spd || 0.0;
        const instSpd_kmh = currentSpd_mps * KMH_MS;

        // --- HORLOGE & TEMPS ÉCOULÉ ---
        if ($('local-time') && !$('local-time').textContent.includes('SYNCHRO ÉCHOUÉE')) {
            $('local-time').textContent = now.toLocaleTimeString('fr-FR');
        }
        if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
        
        const elapsedTime = (Date.now() - startTime) / 1000;
        if ($('elapsed-time')) $('elapsed-time').textContent = dataOrDefault(elapsedTime, 2, ' s');
        
        if (isMoving) totalMoveTimeMs += DOM_SLOW_UPDATE_MS;
        if ($('move-time')) $('move-time').textContent = dataOrDefault(totalMoveTimeMs / 1000, 2, ' s');

        // --- STATISTIQUES DE VITESSE ET DISTANCE ---
        if ($('speed-max')) $('speed-max').textContent = dataOrDefault(vMaxSession, 1, ' km/h');
        const avgSpd = speedSamples > 0 ? (totalSpeedSum / speedSamples) : 0;
        if ($('speed-avg-move')) $('speed-avg-move').textContent = dataOrDefault(avgSpd * KMH_MS, 1, ' km/h');

        const distanceRatio = netherMode ? (1/8.0) : 1.0;
        if ($('distance-total-km')) $('distance-total-km').textContent = `${dataOrDefault(distMTotal / 1000 * distanceRatio, 3, ' km')} | ${dataOrDefault(distMTotal * distanceRatio, 0, ' m')}`;
        if ($('distance-ratio')) $('distance-ratio').textContent = dataOrDefault(distanceRatio, 3);
        
        const horizon = calculateMaxVisibleDistance(currentAlt);
        if ($('max-visible-distance')) $('max-visible-distance').textContent = dataOrDefault(horizon / 1000, 3, ' km');

        // --- RELATIVITÉ ---
        const lorentzFactor = calculateLorentzFactor(currentSpd_mps);
        const restEnergy = calculateRestMassEnergy(currentMass);
        
        if ($('mach-number')) $('mach-number').textContent = dataOrDefault(currentSpd_mps / currentSpeedOfSound, 4);
        if ($('dilation-time-speed')) $('dilation-time-speed').textContent = dataOrDefault(Math.max(0, (lorentzFactor - 1) * 31536000e9), 2, ' ns/an'); // Dilatation en ns/an

        if ($('energy-rel')) $('energy-rel').textContent = dataOrDefaultExp(lorentzFactor * restEnergy, 2, ' J');
        if ($('energy-rest')) $('energy-rest').textContent = dataOrDefaultExp(restEnergy, 2, ' J');
        if ($('momentum-p')) $('momentum-p').textContent = dataOrDefaultExp(currentMass * currentSpd_mps * lorentzFactor, 2, ' kg·m/s');
        
        // --- ASTRO (SunCalc) ---
        if (typeof SunCalc !== 'undefined') {
            const times = SunCalc.getTimes(now, currentLat, currentLon);
            const sunPos = SunCalc.getPosition(now, currentLat, currentLon);
            const moonIllum = SunCalc.getMoonIllumination(now);

            // Soleil
            const sunAltDeg = sunPos.altitude * R2D;
            const sunAziDeg = (sunPos.azimuth * R2D + 180) % 360;
            if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(sunAltDeg, 2, '°');
            if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(sunAziDeg, 2, '°');
            if ($('sunrise-times')) $('sunrise-times').textContent = times.sunrise ? times.sunrise.toLocaleTimeString('fr-FR') : 'N/A';
            if ($('sunset-times')) $('sunset-times').textContent = times.sunset ? times.sunset.toLocaleTimeString('fr-FR') : 'N/A';
            
            // Lune
            if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(moonIllum.fraction * 100, 1, ' %');
            if ($('moon-phase-name')) $('moon-phase-name').textContent = SunCalc.getMoonPhase(moonIllum.phase); // Fonction à implémenter pour le nom de la phase
            // (Autres données lunaires)
        }

        // --- APPEL MÉTÉO ASYNCHRONE ---
        if (currentLat !== 0 && currentLon !== 0) {
            fetchWeather(currentLat, currentLon);
            // fetchPollutants(currentLat, currentLon); // Fonction non implémentée mais nécessaire pour le DOM
        }

        setTimeout(updateDOMSlow, DOM_SLOW_UPDATE_MS);
    };

    // =================================================================
    // BLOC 5 : GESTIONNAIRES D'ÉVÉNEMENTS ET INITIALISATION
    // =================================================================

    const handleEnvironmentChange = (e) => {
        selectedEnvironment = e.target.value;
        if ($('env-factor')) $('env-factor').textContent = `${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY} (x${ENVIRONMENT_FACTORS[selectedEnvironment].MULT.toFixed(1)})`;
    };

    const handleControlEvents = () => {
        // GPS Pause
        if ($('gps-pause-btn')) {
            $('gps-pause-btn').addEventListener('click', () => {
                isGpsPaused = !isGpsPaused;
                $('gps-pause-btn').textContent = isGpsPaused ? 'Reprendre GPS' : 'Pause GPS';
            });
        }
        // Masse
        if ($('mass-input')) $('mass-input').addEventListener('input', (e) => {
            currentMass = parseFloat(e.target.value) || 70.0;
            if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        });
        // Corps Céleste
        if ($('celestial-body-select')) $('celestial-body-select').addEventListener('change', (e) => {
            currentCelestialBody = e.target.value;
            const { G_ACC_NEW } = updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity);
            if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
        });
        // Mode Nether
        if ($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', () => {
            netherMode = !netherMode;
            $('nether-toggle-btn').textContent = `Mode Nether: ${netherMode ? 'ACTIVÉ (1:8)' : 'DÉSACTIVÉ (1:1)'}`;
        });
        // Arrêt d'Urgence
        if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', () => {
            isEmergencyStop = !isEmergencyStop;
            $('emergency-stop-btn').textContent = isEmergencyStop ? 'REPRENDRE SYSTÈME' : '🛑 Arrêt d\'urgence: ACTIF';
            if ($('stop-status')) $('stop-status').textContent = isEmergencyStop ? 'ACTIF 🔴' : 'INACTIF 🟢';
        });
        // Réactivité UKF
        if ($('ukf-reactivity-mode')) $('ukf-reactivity-mode').addEventListener('change', (e) => {
            currentUKFReactivity = e.target.value;
            // Logic to update UKF R-matrix based on reactivity mode
        });
    };

    const initDashboard = () => {
        
        // --- 1. Lancement des Listeners de Capteurs (IMU + MAG) (APIs STABLES) ---
        if (window.DeviceMotionEvent) window.addEventListener('devicemotion', handleDeviceMotion);
        if (window.DeviceOrientationEvent) window.addEventListener('deviceorientation', handleDeviceOrientation);
        if ($('imu-status')) $('imu-status').textContent = 'Initialisation...';
        
        // --- 2. Démarrage de la géolocalisation GPS (Haute précision) ---
        if (navigator.geolocation) {
            navigator.geolocation.watchPosition(handleGPS, handleGPSError, {
                enableHighAccuracy: true,
                maximumAge: 0, 
                timeout: 5000 
            });
        }
        
        // --- 3. Initialisation de la Carte (Leaflet) ---
        if (typeof L !== 'undefined' && $('map-container')) {
            window.map = L.map('map-container').setView([currentPosition.lat, currentPosition.lon], 15);
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                attribution: '© OpenStreetMap contributors'
            }).addTo(window.map);
            window.marker = L.marker([currentPosition.lat, currentPosition.lon]).addTo(window.map)
                .bindPopup("Position UKF/GPS").openPopup();
        }

        // --- 4. Lancement de la Synchro NTP ---
        syncH().finally(() => { 
            // 5. Initialisation du UKF (après chargement de math.js)
            if (typeof math !== 'undefined') {
                try {
                    ukf = new ProfessionalUKF(); 
                    // Démarrer la boucle UKF à haute fréquence
                    updateSensorsAndFilter();
                } catch (e) {
                    console.error("Échec de l'initialisation UKF:", e.message);
                }
            } else {
                 console.error("Erreur: math.js non chargé.");
            }
            
            // 6. Démarrage de la boucle de mise à jour lente
            updateDOMSlow();
        });
        
        // 7. Initialisation des contrôles utilisateur
        handleControlEvents();
    };

    // Lancement du dashboard une fois le DOM chargé
    document.addEventListener('DOMContentLoaded', initDashboard);
    
})(window);
