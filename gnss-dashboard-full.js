// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// Version HIGH-PRECISION - ADAPTÉE POUR lib/ephem.js et lib/astro.js
// Dépendances : math.min.js, leaflet.js, turf.min.js, lib/ephem.js, lib/astro.js
// =================================================================

// --- BLOC 1 : CONSTANTES ET UTILITAIRES DE BASE ---

const $ = id => document.getElementById(id);
const D2R = Math.PI / 180, R2D = 180 / Math.PI; 
const KMH_MS = 3.6;         
const DOM_SLOW_UPDATE_MS = 2000; 
const LIGHT_SPEED_M_PER_S = 299792458; // C_L

// Formatage des données standard
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || val === Infinity || val === -Infinity || Math.abs(val) < 1e-9) {
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
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

// --- CONSTANTES PHYSIQUES ET GÉOPHYSIQUES ---
const C_L = LIGHT_SPEED_M_PER_S;      
const G_U = 6.67430e-11;    
const G_ACC_STD = 9.80665;  
const R_SPECIFIC_AIR = 287.058; 
const GAMMA_AIR = 1.4;          
const TEMP_SEA_LEVEL_K = 288.15; 
const BARO_ALT_REF_HPA = 1013.25; 
const RHO_SEA_LEVEL = 1.225;    
const WGS84_A = 6378137.0; 
const WGS84_F = 1 / 298.257223563; 
const WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F; 

// =================================================================
// BLOC 2 : ÉTAT GLOBAL ET MODÈLES PHYSIQUES / UKF (SIMULÉS)
// =================================================================

((window) => {
    
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
    let lastUKFUpdate = Date.now();
    let ukf = null;     
    let currentMass = 70.0;
    let distMTotal = 0.0; 
    let vMaxSession = 0.0; 
    let startTime = Date.now();
    let isMoving = false;
    let totalMoveTimeMs = 0;
    let speedSamples = 0;
    let totalSpeedSum = 0;
    let lastNTPDate = null;     
    let lastLocalTime = null;   
    let lastAccX = 0, lastAccY = 0, lastAccZ = 0; 
    let lastPitch = 0, lastRoll = 0; 
    let lightAmbientLux = 0;
    let soundLevelDb = 0;
    let lastP_hPa = BARO_ALT_REF_HPA;
    let lastT_K = TEMP_SEA_LEVEL_K;
    let lastH_perc = 0.5;             
    let currentAirDensity = RHO_SEA_LEVEL;
    let currentSpeedOfSound = 343.0; 
    let lastKnownWeather = null;
    let lastKnownPollutants = null;
    let ukf_vel_uncertainty = 0.0;
    let ukf_alt_uncertainty = 0.0;

    // --- CLASSE UKF (Simplifiée si math.js est présent) ---
    class ProfessionalUKF {
        constructor() {
            this.STATE_SIZE = 21;
            this.x = (typeof math !== 'undefined') ? math.zeros(this.STATE_SIZE) : new Array(this.STATE_SIZE).fill(0);
        }
        predict_propagation(dt, imu_accel) {
             if (typeof math !== 'undefined') {
                this.x = this.x.map((val, index) => {
                    if (index < 3) return val + (Math.random() - 0.5) * 1e-6; 
                    if (index === 5) return imu_accel[2] * dt; 
                    return val;
                });
            }
        }
        update_correction(gps_measurement) {
            if (typeof math !== 'undefined') {
                const gpsAlt = gps_measurement[2];
                const currentAlt = this.x.get([2]);
                const newAlt = currentAlt + (gpsAlt - currentAlt) * 0.1;
                this.x.set([2], newAlt);
            }
        }
        getState() {
            const lat = (typeof math !== 'undefined') ? this.x.get([0]) * R2D : currentPosition.lat;
            const lon = (typeof math !== 'undefined') ? this.x.get([1]) * R2D : currentPosition.lon;
            const alt = (typeof math !== 'undefined') ? this.x.get([2]) : currentPosition.alt;
            const vel_up = (typeof math !== 'undefined') ? this.x.get([5]) : 0; 
            ukf_alt_uncertainty = Math.max(0.01, Math.random() * 0.5 + 0.1); 
            ukf_vel_uncertainty = Math.max(0.01, Math.random() * 0.05 + 0.01); 
            return { lat, lon, alt, vel_up };
        }
    }

    // --- FONCTIONS UTILITAIRES PHYSIQUES/ASTRO (Inchangées) ---

    const getCDate = () => {
        if (!lastNTPDate || !lastLocalTime) return new Date(); 
        const localTimeDifference = Date.now() - lastLocalTime;
        return new Date(lastNTPDate.getTime() + localTimeDifference);
    };

    function getSpeedOfSound(tempK) {
        return Math.sqrt(GAMMA_AIR * R_SPECIFIC_AIR * tempK);
    }
    
    function calculateAirDensity(P_hPa, T_K) {
        const P = P_hPa * 100; // Pa
        return P / (R_SPECIFIC_AIR * T_K); 
    }
    
    function calculateWGS84Gravity(lat, alt) {
        const latRad = lat * D2R;
        const sinSqLat = Math.pow(Math.sin(latRad), 2);
        const g0 = 9.780327 * (1 + 0.0053024 * sinSqLat - 0.0000058 * Math.pow(sinSqLat, 2));
        const radius = WGS84_A / Math.sqrt(1 - WGS84_E2 * sinSqLat);
        const g_alt = g0 * (1 - 2 * alt / radius);
        return g_alt;
    }
    
    function calculateLorentzFactor(v_mps) {
        if (v_mps >= C_L) return Infinity;
        return 1.0 / Math.sqrt(1.0 - Math.pow(v_mps / C_L, 2));
    }
    
    function calculateGravitationalDilation(altM, gravity) {
        const c2 = C_L * C_L;
        const fraction = (gravity * altM) / c2;
        const ns_per_day = fraction * 86400 * 1e9;
        return ns_per_day; 
    }
    
    function calculateSchwarzschildRadius(m) {
        return (2 * G_U * m) / (C_L * C_L);
    }
    
    function calculateBioSVT(tempC, altM, humidity_perc, pressure_Pa) {
        const P_sat = 6.112 * Math.exp((17.67 * tempC) / (tempC + 243.5));
        const P_vap = humidity_perc / 100 * P_sat;
        const T_dew = (243.5 * Math.log(P_vap / 6.112)) / (17.67 - Math.log(P_vap / 6.112));

        const wetBulbTemp = tempC * Math.atan(0.151977 * Math.sqrt(humidity_perc + 8.313659)) + 
                            Math.atan(tempC + humidity_perc) - Math.atan(humidity_perc - 1.676331) + 
                            0.00391838 * Math.pow(humidity_perc, 1.5) * Math.atan(0.023101 * humidity_perc) - 4.686035;

        const tempK = tempC + 273.15;
        const absoluteHumidity = 1000 * (P_vap * 100 / tempK) / 461.5; 
        
        let CAPE = 0;
        if (tempC > 15 && humidity_perc > 50) {
            CAPE = 10 * (tempC - 15) * (humidity_perc / 100) * Math.max(1, (1 - altM / 3000));
        }
        
        const saturationO2 = 100 * (1 - altM / 8000); 

        return {
            dewPoint: T_dew, wetBulbTemp: wetBulbTemp,
            CAPE: CAPE, saturationO2: saturationO2, 
            absoluteHumidity: absoluteHumidity
        };
    }
    
    // NOTE: Utilise getMoonPhaseName du script astro.js de l'utilisateur si disponible, sinon utilise une version simple.
    const getMoonPhaseName = (phase) => {
        if (typeof window.getMoonPhaseName === 'function') {
            return window.getMoonPhaseName(phase); // Utilise la fonction de l'utilisateur (si définie)
        }
        // Fallback simple
        if (phase < 0.06 || phase > 0.94) return 'Nouvelle Lune';
        if (phase < 0.25) return 'Premier Croissant';
        if (phase < 0.31) return 'Premier Quartier';
        if (phase < 0.50) return 'Lune Gibbeuse Croissante';
        if (phase < 0.56) return 'Pleine Lune';
        if (phase < 0.75) return 'Lune Gibbeuse Décroissante';
        if (phase < 0.81) return 'Dernier Quartier';
        return 'Dernier Croissant';
    };


    // --- GESTIONNAIRES DE CAPTEURS & API (MOCKÉES/SIMULÉES) ---
    
    const handleDeviceMotion = (event) => {
        if (event.accelerationIncludingGravity) {
            lastAccX = event.accelerationIncludingGravity.x;
            lastAccY = event.accelerationIncludingGravity.y;
            lastAccZ = event.accelerationIncludingGravity.z;
        }
        const now = Date.now();
        lastPitch = (event.rotationRate ? event.rotationRate.beta : 0) || 0;    
        lastRoll = (event.rotationRate ? event.rotationRate.gamma : 0) || 0; 
        lightAmbientLux = 100 + Math.abs(Math.sin(now / 10000)) * 900;
        soundLevelDb = 40 + Math.abs(Math.cos(now / 8000)) * 40;
    };
    
    const requestMotionPermission = () => {
        // ... (Logique de permission IMU) ...
    };

    const handleGPS = (position) => {
        // ... (Logique de mise à jour GPS/Distance) ...
        if (isGpsPaused) return;

        const newLat = position.coords.latitude;
        const newLon = position.coords.longitude;
        const newAlt = position.coords.altitude === null ? currentPosition.alt : position.coords.altitude;
        const speed_mps = position.coords.speed || 0.0;
        const now = Date.now();

        // Distance & Vitesse
        if (speed_mps > 0.1) {
            if (!isMoving) totalMoveTimeMs += now - lastUKFUpdate;
            isMoving = true;
            totalSpeedSum += speed_mps;
            speedSamples++;
            vMaxSession = Math.max(vMaxSession, speed_mps * KMH_MS);
        } else {
            isMoving = false;
        }
        
        if (typeof turf !== 'undefined' && distMTotal > 0.0 || (newLat !== 0 && newLon !== 0)) { 
            const p1 = turf.point([currentPosition.lon, currentPosition.lat]);
            const p2 = turf.point([newLon, newLat]);
            const horizontalDistance = turf.distance(p1, p2, { units: 'meters' });
            distMTotal += horizontalDistance; 
        } else if (typeof turf === 'undefined' && (newLat !== 0 && newLon !== 0)) {
            distMTotal += speed_mps * (now - lastUKFUpdate) / 1000;
        }
        
        currentPosition.lat = newLat;
        currentPosition.lon = newLon;
        currentPosition.alt = newAlt;
        currentPosition.acc = position.coords.accuracy || 10.0;
        currentPosition.spd = speed_mps;
        currentPosition.heading = position.coords.heading || currentPosition.heading;

        if ($('statut-gps')) $('statut-gps').textContent = 'ACTIF 🟢';
        if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = 'Acquisition OK';
        if ($('positionnement-ekf')) $('positionnement-ekf').textContent = 'GPS Actif'; 
    };

    const handleGPSError = (error) => {
        // ... (Logique d'erreur GPS/Fallback) ...
        console.error(`[GPS ERREUR CRITIQUE] Code ${error.code}: ${error.message}`);
        if ($('statut-gps')) $('statut-gps').textContent = 'ERREUR GPS 🔴';
        if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = 'ERREUR ACQUISITION';
        if ($('positionnement-ekf')) $('positionnement-ekf').textContent = 'Coordonnées par défaut'; 
        
        if (ukf) {
            ukf.update_correction([currentPosition.lat * D2R, currentPosition.lon * D2R, currentPosition.alt]);
        }
    };
    
    const syncH = async () => {
        // ... (Logique de Synchro NTP) ...
        if ($('heure-locale')) $('heure-locale').textContent = 'Synchronisation...';
        try {
            const response = await fetch(SERVER_TIME_ENDPOINT);
            const data = await response.json();
            
            lastNTPDate = new Date(data.datetime);
            lastLocalTime = Date.now(); 
            if ($('heure-locale')) $('heure-locale').textContent = lastNTPDate.toLocaleTimeString('fr-FR');
            if ($('date-heure-utc')) $('date-heure-utc').textContent = lastNTPDate.toUTCString().replace('GMT', 'UTC');
        } catch (error) {
            console.error("[NTP ERREUR] Échec de la synchronisation NTP. Le temps local du PC est utilisé.");
            if ($('heure-locale')) $('heure-locale').textContent = `SYNCHRO ÉCHOUÉE 🟡`;
            if ($('date-heure-utc')) $('date-heure-utc').textContent = new Date().toUTCString().replace('GMT', 'UTC'); 
        } finally {
            setTimeout(syncH, 60000); 
        }
    };
    
    const fetchWeather = async (lat, lon) => {
        // MOCK de la réponse API (SIMULATION)
        const mockData = {
            temp: TEMP_SEA_LEVEL_K + (Math.random() - 0.5) * 10, 
            pressure: BARO_ALT_REF_HPA + (Math.random() - 0.5) * 10, 
            humidity: 50 + (Math.random() - 0.5) * 20, 
            weather_status: 'CLAIRE'
        };
        const data = mockData;
        
        const tempK = data.temp;
        const pressure_hPa = data.pressure;
        const humidity_perc = data.humidity;
        
        lastT_K = tempK;
        lastP_hPa = pressure_hPa;
        lastH_perc = humidity_perc;
        currentSpeedOfSound = getSpeedOfSound(tempK); 
        currentAirDensity = calculateAirDensity(pressure_hPa, tempK); 
        
        lastKnownWeather = { 
            tempC: tempK - 273.15, 
            pressure_hPa, 
            humidity_perc, 
            air_density: currentAirDensity, 
            tempK 
        };
        updateWeatherDOM(lastKnownWeather);
    };

    const fetchPollutants = async () => {
        // MOCK de la réponse API (SIMULATION)
        lastKnownPollutants = {
            NO2: 20 + Math.random() * 5, PM25: 15 + Math.random() * 5,
            PM10: 25 + Math.random() * 5, O3: 50 + Math.random() * 10
        };
        updatePollutantsDOM(lastKnownPollutants);
    };

    const updateWeatherDOM = (data, isDefault = false) => {
        // ... (Logique de mise à jour DOM Météo) ...
        if ($('temp-air')) $('temp-air').textContent = dataOrDefault(data.tempC, 1, ' °C');
        if ($('pressure-atm')) $('pressure-atm').textContent = dataOrDefault(data.pressure_hPa, 0, ' hPa');
        if ($('humidity-rel')) $('humidity-rel').textContent = dataOrDefault(data.humidity_perc, 0, ' %');
        if ($('densite-air')) $('densite-air').textContent = dataOrDefault(data.air_density, 3, ' kg/m³');
        if ($('statut-meteo')) $('statut-meteo').textContent = isDefault ? 'HORS LIGNE/DÉFAUT 🟡' : 'ACTIF 🟢';
        if ($('vitesse-son-locale')) $('vitesse-son-locale').textContent = dataOrDefault(currentSpeedOfSound, 2, ' m/s');
        if ($('point-rosee')) $('point-rosee').textContent = dataOrDefault(calculateBioSVT(data.tempC, kAlt, data.humidity_perc, data.pressure_hPa * 100).dewPoint, 1, ' °C');
    };
    
    const updatePollutantsDOM = (data) => {
        // ... (Logique de mise à jour DOM Polluants) ...
        if ($('no2')) $('no2').textContent = dataOrDefault(data.NO2, 1, ' µg/m³');
        if ($('pm-25')) $('pm-25').textContent = dataOrDefault(data.PM25, 1, ' µg/m³');
        if ($('pm-10')) $('pm-10').textContent = dataOrDefault(data.PM10, 1, ' µg/m³');
        if ($('o3')) $('o3').textContent = dataOrDefault(data.O3, 1, ' µg/m³');
    };


    // =================================================================
    // BLOC 3 : BOUCLES D'EXÉCUTION ET MISE À JOUR DU DOM
    // =================================================================

    const updateSensorsAndFilter = () => {
        // ... (Logique de haute fréquence) ...
        const now = Date.now();
        const dt = (now - lastUKFUpdate) / 1000.0;
        lastUKFUpdate = now;
        
        const instSpd_mps = currentPosition.spd || 0.0; 
        
        let ukfState = { lat: currentPosition.lat, lon: currentPosition.lon, alt: currentPosition.alt, vel_up: 0.0 };

        if (ukf && typeof math !== 'undefined' && dt > 0) {
            ukf.predict_propagation(dt, [lastAccX, lastAccY, lastAccZ]);
            ukf.update_correction([currentPosition.lat * D2R, currentPosition.lon * D2R, currentPosition.alt]);
            ukfState = ukf.getState();
            kAlt = ukfState.alt;
            if ($('statut-ekf-fusion')) $('statut-ekf-fusion').textContent = 'Fusion Active 🟢';
        } else if (ukf === null) {
            kAlt = currentPosition.alt;
            if ($('statut-ekf-fusion')) $('statut-ekf-fusion').textContent = 'Désactivé (math.js manquant) 🔴';
        }
        
        // ... (Mise à jour DOM Haute Fréquence : Vitesse, IMU, UKF, Dynamique) ...
        if (dt > 0) {
            const gravity = calculateWGS84Gravity(ukfState.lat, kAlt); 
            const dynamicPressure = 0.5 * currentAirDensity * instSpd_mps * instSpd_mps;
            const machNumber = instSpd_mps / currentSpeedOfSound; 
            const nonGravAccelZ = lastAccZ - gravity; 
            const dragForce = dynamicPressure * 1.2 * 0.5; // Coeffs de traînée simplifiés
            const dragPowerKw = (dragForce * instSpd_mps) / 1000;
            const kineticEnergy = 0.5 * currentMass * instSpd_mps * instSpd_mps;
            const gForceVertical = (nonGravAccelZ / G_ACC_STD) + 1.0;

            if ($('vitesse-3d-instantanee')) $('vitesse-3d-instantanee').textContent = dataOrDefault(instSpd_mps * KMH_MS, 2, ' km/h');
            if ($('vitesse-brute-mps')) $('vitesse-brute-mps').textContent = dataOrDefault(instSpd_mps, 3, ' m/s');
            if ($('acceleration-x')) $('acceleration-x').textContent = dataOrDefault(lastAccX, 3, ' m/s²');
            if ($('acceleration-y')) $('acceleration-y').textContent = dataOrDefault(lastAccY, 3, ' m/s²');
            if ($('acceleration-z')) $('acceleration-z').textContent = dataOrDefault(lastAccZ, 3, ' m/s²');
            if ($('latitude-ekf')) $('latitude-ekf').textContent = dataOrDefault(ukfState.lat, 6, '°');
            if ($('longitude-ekf')) $('longitude-ekf').textContent = dataOrDefault(ukfState.lon, 6, '°');
            if ($('altitude-ekf')) $('altitude-ekf').textContent = dataOrDefault(kAlt, 2, ' m');
            if ($('vitesse-verticale-ekf')) $('vitesse-verticale-ekf').textContent = dataOrDefault(ukfState.vel_up || 0, 3, ' m/s');
            if ($('accel-verticale-imu')) $('accel-verticale-imu').textContent = dataOrDefault(nonGravAccelZ, 3, ' m/s²');
            if ($('gravity-locale')) $('gravity-locale').textContent = dataOrDefault(gravity, 4, ' m/s²');
            if ($('force-g-verticale')) $('force-g-verticale').textContent = dataOrDefault(gForceVertical, 3, ' G');
            if ($('incertitude-vitesse')) $('incertitude-vitesse').textContent = dataOrDefault(ukf_vel_uncertainty, 3, ' m/s');
            if ($('incertitude-alt')) $('incertitude-alt').textContent = dataOrDefault(ukf_alt_uncertainty, 3, ' m');
            if ($('pression-dynamique')) $('pression-dynamique').textContent = dataOrDefault(dynamicPressure, 2, ' Pa');
            if ($('force-trainee')) $('force-trainee').textContent = dataOrDefault(dragForce, 2, ' N');
            if ($('puissance-trainee-kw')) $('puissance-trainee-kw').textContent = dataOrDefault(dragPowerKw, 2, ' kW');
            if ($('energie-cinetique')) $('energie-cinetique').textContent = dataOrDefault(kineticEnergy, 2, ' J');
            if ($('puissance-mecanique')) $('puissance-mecanique').textContent = dataOrDefault(instSpd_mps > 0 ? dragPowerKw * 1000 : 0, 2, ' W'); 
            if ($('nombre-mach')) $('nombre-mach').textContent = dataOrDefault(machNumber, 4);
            if ($('pourcentage-vitesse-son')) $('pourcentage-vitesse-son').textContent = dataOrDefault(machNumber * 100, 2, ' %'); 
        } 
        
        if ($('inclinaison-pitch')) $('inclinaison-pitch').textContent = dataOrDefault(lastPitch, 1, '°');
        if ($('roulis-roll')) $('roulis-roll').textContent = dataOrDefault(lastRoll, 1, '°');
        if ($('cap-direction')) $('cap-direction').textContent = dataOrDefault(currentPosition.heading, 1, '°');
        if ($('precision-gps-acc')) $('precision-gps-acc').textContent = dataOrDefault(currentPosition.acc, 2, ' m'); 

        requestAnimationFrame(updateSensorsAndFilter); 
    };

    const updateDOMSlow = () => {

        const now = getCDate();
        const currentLat = currentPosition.lat; 
        const currentLon = currentPosition.lon; 
        const currentAlt = kAlt;
        const currentSpd_mps = currentPosition.spd || 0.0;
        const elapsedTime = (Date.now() - startTime) / 1000;
        const totalMoveTimeS = totalMoveTimeMs / 1000;
        
        // --- TEMPS & DISTANCE / RELATIVITÉ (Inchangé) ---
        if ($('temps-ecoule-session')) $('temps-ecoule-session').textContent = dataOrDefault(elapsedTime, 2, ' s');
        if ($('temps-mouvement')) $('temps-mouvement').textContent = dataOrDefault(totalMoveTimeS, 2, ' s');
        if ($('distance-totale-3d')) $('distance-totale-3d').textContent = `${dataOrDefault(distMTotal / 1000, 3, ' km')} | ${dataOrDefault(distMTotal, 2, ' m')}`;
        const lorentzFactor = calculateLorentzFactor(currentSpd_mps);
        const restEnergy = currentMass * C_L * C_L;
        const momentum = currentMass * currentSpd_mps * lorentzFactor;
        const Rs = calculateSchwarzschildRadius(currentMass);
        if ($('pourcentage-vitesse-lumiere')) $('pourcentage-vitesse-lumiere').textContent = dataOrDefaultExp(currentSpd_mps / C_L * 100, 2, ' %');
        if ($('facteur-lorentz')) $('facteur-lorentz').textContent = dataOrDefault(lorentzFactor, 4);
        if ($('temps-dilation-vitesse')) $('temps-dilation-vitesse').textContent = dataOrDefault(Math.max(0, (lorentzFactor - 1) * 86400 * 1e9), 2, ' ns/j'); 
        if ($('temps-dilation-gravite')) $('temps-dilation-gravite').textContent = dataOrDefault(calculateGravitationalDilation(currentAlt, calculateWGS84Gravity(currentLat, currentAlt)), 2, ' ns/j'); 
        if ($('energie-relativiste')) $('energie-relativiste').textContent = dataOrDefaultExp(lorentzFactor * restEnergy, 2, ' J');
        if ($('energie-masse-repos')) $('energie-masse-repos').textContent = dataOrDefaultExp(restEnergy, 2, ' J');
        if ($('quantite-mouvement')) $('quantite-mouvement').textContent = dataOrDefaultExp(momentum, 2, ' kg·m/s');
        if ($('rayon-schwarzschild')) $('rayon-schwarzschild').textContent = dataOrDefaultExp(Rs, 3, ' m');
        
        // --- ASTRO (APPEL AUX LIBRAIRIES HAUTE PRÉCISION) ---
        // --- ASTRO (APPEL AUX LIBRAIRIES HAUTE PRÉCISION) ---
        
        // Les fonctions Astro dépendent de 'now' (Date) et d'une position (Lat/Lon).
        const now = getCDate(lServH, lLocH); 
        const lat = currentPosition ? currentPosition.lat : 0.0;
        const lon = currentPosition ? currentPosition.lon : 0.0;
        
        // 1. UTILISATION DES FONCTIONS ASTRO (Regroupe TST/MST/TSLV et Alt/Az)
        // La fonction calculateAstroDataHighPrec doit exister ET la date doit être valide.
        if (typeof calculateAstroDataHighPrec === 'function' && now instanceof Date && !isNaN(now.getTime())) {
            try {
                // Calcule toutes les données astro en une seule fois
                const fullAstroData = calculateAstroDataHighPrec(now, lat, lon); 

                // Mise à jour des IDs DOM pour le Temps Solaire & Sidéral
                if ($('date-display-astro')) $('date-display-astro').textContent = now.toLocaleDateString('fr-FR');
                
                // Champs Temps Solaire & Sidéral
                if ($('tst')) $('tst').textContent = fullAstroData.TST_HRS || 'N/A';
                if ($('mst')) $('mst').textContent = fullAstroData.MST_HRS || 'N/A';
                if ($('noon-solar')) $('noon-solar').textContent = fullAstroData.NOON_SOLAR_UTC.toTimeString().split(' ')[0] + ' UTC';
                if ($('eot')) $('eot').textContent = dataOrDefault(parseFloat(fullAstroData.EOT_MIN), 2, ' min');
                if ($('tslv')) $('tslv').textContent = getTSLV(now, lon) || 'N/A'; // TSLV doit être appelée séparément
                if ($('ecl-long')) $('ecl-long').textContent = dataOrDefault(parseFloat(fullAstroData.ECL_LONG), 4, '°');
                
                // Dates Solaires
                if ($('date-solar-true')) $('date-solar-true').textContent = fullAstroData.NOON_SOLAR_UTC.toLocaleDateString('fr-FR');
                if ($('date-solar-mean')) $('date-solar-mean').textContent = fullAstroData.NOON_SOLAR_UTC.toLocaleDateString('fr-FR');


                // --- Position du SOLEIL (Altitude / Azimut) ---
                const R2D = 180 / Math.PI;
                if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(fullAstroData.sun.altitude * R2D, 2, '°');
                if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(fullAstroData.sun.azimuth * R2D, 2, '°');
                
                // Heures de Lever/Coucher (Approximation)
                if ($('sunrise-times')) $('sunrise-times').textContent = fullAstroData.sun.sunrise.toLocaleTimeString('fr-FR');
                if ($('sunset-times')) $('sunset-times').textContent = fullAstroData.sun.sunset.toLocaleTimeString('fr-FR');
                if ($('day-duration')) $('day-duration').textContent = dataOrDefault((fullAstroData.sun.sunset.getTime() - fullAstroData.sun.sunrise.getTime()) / 3600000, 2, ' h');
                
                // --- Position de la LUNE (Simplifiée) ---
                if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(fullAstroData.moon.illumination.phase); 
                if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(fullAstroData.moon.illumination.fraction * 100, 1, ' %');
                if ($('moon-alt')) $('moon-alt').textContent = dataOrDefault(fullAstroData.moon.position.altitude * R2D, 2, '°');
                if ($('moon-azimuth')) $('moon-azimuth').textContent = dataOrDefault(fullAstroData.moon.position.azimuth * R2D, 2, '°');
                if ($('moon-distance')) $('moon-distance').textContent = dataOrDefault(fullAstroData.moon.position.distance / 1000, 0, ' km'); 

                // Lever/Coucher Lune (affichage simple)
                let moonTimesText = 'N/A';
                if (fullAstroData.moon.times.alwaysUp) {
                    moonTimesText = 'Toujours visible';
                } else if (fullAstroData.moon.times.alwaysDown) {
                    moonTimesText = 'Jamais visible';
                } else {
                    // Les valeurs de lever/coucher sont souvent nulles dans cette implémentation simplifiée
                    moonTimesText = 'Calcul indisponible';
                }
                if ($('moon-times')) $('moon-times').textContent = moonTimesText; 

            } catch (e) {
                console.error("🔴 ERREUR: L'appel à calculateAstroDataHighPrec a échoué. Cause: ", e);
                // Affichage des erreurs pour le débug
                if ($('tst')) $('tst').textContent = 'N/A (Erreur JS)';
                if ($('sun-alt')) $('sun-alt').textContent = 'N/A (Erreur JS)';
            }
        } else {
            // Affichage de secours si la date ou la fonction sont manquantes
            if ($('tst')) $('tst').textContent = 'N/A (Date ou Fonction Astro manquante)';
            if ($('sun-alt')) $('sun-alt').textContent = 'N/A (Date ou Fonction Astro manquante)';
        }

        // ... (le reste du code updateDOMSlow) ...
                // Mise à jour des IDs DOM pour le Temps Solaire & Sidéral
                if ($('date-display-astro')) $('date-display-astro').textContent = now.toLocaleDateString('fr-FR');
                
                // Champs de Temps Solaire Vrai (TST) et Équation du Temps (EOT)
                if ($('tst')) $('tst').textContent = fullAstroData.TST_HRS || 'N/A';
                if ($('eot')) $('eot').textContent = dataOrDefault(parseFloat(fullAstroData.EOT_MIN), 2, ' min');
                if ($('tslv')) $('tslv').textContent = getTSLV(now, currentLon) || 'N/A'; 
                if ($('ecl-long')) $('ecl-long').textContent = dataOrDefault(parseFloat(fullAstroData.ECL_LONG), 4, '°');
                
                // Nouveaux Champs complétés : MST et Midi Solaire Vrai
                if ($('mst')) $('mst').textContent = fullAstroData.MST_HRS || 'N/A';
                if ($('noon-solar')) $('noon-solar').textContent = fullAstroData.NOON_SOLAR_UTC.toTimeString().split(' ')[0] + ' UTC';
                
                // La "Date Solaire Vraie/Moyenne" correspond à la date du Midi Solaire
                if ($('date-solar-true')) $('date-solar-true').textContent = fullAstroData.NOON_SOLAR_UTC.toLocaleDateString('fr-FR');
                if ($('date-solar-mean')) $('date-solar-mean').textContent = fullAstroData.NOON_SOLAR_UTC.toLocaleDateString('fr-FR');


                // --- Position du SOLEIL (Altitude / Azimut) ---
                const R2D = 180 / Math.PI;
                if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(fullAstroData.sun.altitude * R2D, 2, '°');
                if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(fullAstroData.sun.azimuth * R2D, 2, '°');
                
                // Heures de Lever/Coucher (Approximation)
                if ($('sunrise-times')) $('sunrise-times').textContent = fullAstroData.sun.sunrise.toLocaleTimeString('fr-FR');
                if ($('sunset-times')) $('sunset-times').textContent = fullAstroData.sun.sunset.toLocaleTimeString('fr-FR');
                if ($('day-duration')) $('day-duration').textContent = dataOrDefault((fullAstroData.sun.sunset.getTime() - fullAstroData.sun.sunrise.getTime()) / 3600000, 2, ' h');
                
                // --- Position de la LUNE (Simplifiée) ---
                if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(fullAstroData.moon.illumination.phase); 
                if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(fullAstroData.moon.illumination.fraction * 100, 1, ' %');
                if ($('moon-alt')) $('moon-alt').textContent = dataOrDefault(fullAstroData.moon.position.altitude * R2D, 2, '°');
                if ($('moon-azimuth')) $('moon-azimuth').textContent = dataOrDefault(fullAstroData.moon.position.azimuth * R2D, 2, '°');
                if ($('moon-distance')) $('moon-distance').textContent = dataOrDefault(fullAstroData.moon.position.distance / 1000, 0, ' km'); 

                let moonTimesText = 'N/A';
                if (fullAstroData.moon.times.alwaysUp) {
                    moonTimesText = 'Toujours visible';
                } else if (fullAstroData.moon.times.alwaysDown) {
                    moonTimesText = 'Jamais visible';
                } else {
                    const rise = fullAstroData.moon.times.rise ? fullAstroData.moon.times.rise.toLocaleTimeString('fr-FR') : '--:--';
                    const set = fullAstroData.moon.times.set ? fullAstroData.moon.times.set.toLocaleTimeString('fr-FR') : '--:--';
                    moonTimesText = `${rise} / ${set}`;
                }
                if ($('moon-times')) $('moon-times').textContent = moonTimesText; 

            } catch (e) {
                console.error("🔴 ERREUR: L'appel à calculateAstroDataHighPrec a échoué.", e);
                if ($('tst')) $('tst').textContent = 'N/A (Erreur)';
                if ($('sun-alt')) $('sun-alt').textContent = 'N/A (Erreur)';
            }
        } else {
            if ($('tst')) $('tst').textContent = 'N/A (Fonction Astro manquante)';
            if ($('sun-alt')) $('sun-alt').textContent = 'N/A (Fonction Astro manquante)';
            }

        // 2. APPEL STANDARDISÉ POUR LES DONNÉES DE POSITION (Alt/Az/Lever/Coucher)
        // L'utilisateur doit implémenter cette fonction dans lib/astro.js ou lib/ephem.js
        if (typeof calculateAstroDataHighPrec === 'function') {
            try {
                // La fonction doit retourner un objet avec la structure attendue
                const fullAstroData = calculateAstroDataHighPrec(now, currentLat, currentLon); 

                // SOLEIL
                if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(fullAstroData.sun.altitude * R2D, 2, '°');
                if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(fullAstroData.sun.azimuth * R2D, 2, '°');
                if ($('day-duration')) $('day-duration').textContent = dataOrDefault((fullAstroData.sun.sunset.getTime() - fullAstroData.sun.sunrise.getTime()) / 3600000, 2, ' h');
                if ($('sunrise-times')) $('sunrise-times').textContent = fullAstroData.sun.sunrise.toLocaleTimeString('fr-FR');
                if ($('sunset-times')) $('sunset-times').textContent = fullAstroData.sun.sunset.toLocaleTimeString('fr-FR');
                
                // LUNE
                if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(fullAstroData.moon.illumination.phase); 
                if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(fullAstroData.moon.illumination.fraction * 100, 1, ' %');
                if ($('moon-alt')) $('moon-alt').textContent = dataOrDefault(fullAstroData.moon.position.altitude * R2D, 2, '°');
                if ($('moon-azimuth')) $('moon-azimuth').textContent = dataOrDefault(fullAstroData.moon.position.azimuth * R2D, 2, '°');
                
                let moonTimesText = 'N/A';
                if (fullAstroData.moon.times.alwaysUp) {
                    moonTimesText = 'Toujours visible';
                } else if (fullAstroData.moon.times.alwaysDown) {
                    moonTimesText = 'Jamais visible';
                } else {
                    const rise = fullAstroData.moon.times.rise ? fullAstroData.moon.times.rise.toLocaleTimeString('fr-FR') : '--:--';
                    const set = fullAstroData.moon.times.set ? fullAstroData.moon.times.set.toLocaleTimeString('fr-FR') : '--:--';
                    moonTimesText = `${rise} / ${set}`;
                }
                if ($('moon-times')) $('moon-times').textContent = moonTimesText; 
                if ($('moon-distance')) $('moon-distance').textContent = dataOrDefault(fullAstroData.moon.position.distance / 1000, 0, ' km'); 

            } catch (e) {
                console.warn("🟡 AVERTISSEMENT: Position Astro (Alt/Az/Times) manquante ou erreur dans calculateAstroDataHighPrec().", e);
                if ($('sun-alt')) $('sun-alt').textContent = 'N/A (Manque fonction Pos.)';
            }
        } else {
            if ($('sun-alt')) $('sun-alt').textContent = 'N/A (Manque fonction Pos.)';
            if ($('moon-alt')) $('moon-alt').textContent = 'N/A (Manque fonction Pos.)';
        }

        // --- IMU MOCK / BIO/SVT (Inchangé) ---
        if ($('lumiere-ambiante')) $('lumiere-ambiante').textContent = dataOrDefault(lightAmbientLux, 0, ' Lux');
        if ($('niveau-sonore')) $('niveau-sonore').textContent = dataOrDefault(soundLevelDb, 1, ' dB');
        
        if (lastKnownWeather) {
            const pressure_Pa = lastKnownWeather.pressure_hPa * 100;
            const bioSim = calculateBioSVT(lastKnownWeather.tempC, currentAlt, lastKnownWeather.humidity_perc, pressure_Pa);
            if ($('humidite-absolue-sim')) $('humidite-absolite-sim').textContent = dataOrDefault(bioSim.absoluteHumidity, 3, ' g/m³');
            if ($('temp-bulbe-humide-sim')) $('temp-bulbe-humide-sim').textContent = dataOrDefault(bioSim.wetBulbTemp, 1, ' °C');
            if ($('cape-sim')) $('cape-sim').textContent = dataOrDefault(bioSim.CAPE, 0, ' J/kg');
            if ($('saturation-o2-sim')) $('saturation-o2-sim').textContent = dataOrDefault(bioSim.saturationO2, 1, ' %');
        }

        // Fetch Météo/Polluants (Mocks)
        fetchWeather(currentLat, currentLon); 
        fetchPollutants(); 

        setTimeout(updateDOMSlow, DOM_SLOW_UPDATE_MS);
    };

    // --- CONTRÔLES ET INITIALISATION ---
    
    const handleControlEvents = () => {
        // ... (Logique de gestion des événements) ...
        if ($('marche-gps-btn')) $('marche-gps-btn').addEventListener('click', () => { 
            isGpsPaused = !isGpsPaused;
            $('marche-gps-btn').textContent = isGpsPaused ? '▶️ MARCHE GPS' : '⏸️ PAUSE GPS';
        });
        if ($('reinit-vmax-btn')) $('reinit-vmax-btn').addEventListener('click', () => vMaxSession = 0.0);
        if ($('reinit-dist-btn')) $('reinit-dist-btn').addEventListener('click', () => distMTotal = 0.0);
        // ... (autres contrôles) ...
    };

    const initDashboard = () => {
        
        handleControlEvents();

        // --- Démarrage GPS ---
        if (navigator.geolocation) {
            navigator.geolocation.watchPosition(handleGPS, handleGPSError, {
                enableHighAccuracy: true,
                maximumAge: 0, 
                timeout: 5000 
            });
            if ($('statut-gps')) $('statut-gps').textContent = 'Initialisation...';
        } else {
            handleGPSError({ code: 0, message: "Geolocation API non disponible dans ce navigateur." });
        }
        
        // --- Vérification et Initialisation des Dépendances ---
        if (typeof math !== 'undefined') {
            ukf = new ProfessionalUKF(); 
            requestMotionPermission(); 
            if ($('statut-ekf-fusion')) $('statut-ekf-fusion').textContent = 'Initialisation UKF...';
        } else {
            console.error("🔴 ERREUR: math.min.js n'est pas chargé. Le filtre UKF est désactivé.");
            if ($('statut-ekf-fusion')) $('statut-ekf-fusion').textContent = 'Désactivé (math.js manquant) 🔴';
        }

        if (typeof L !== 'undefined' && $('map-container')) {
            window.map = L.map('map-container').setView([currentPosition.lat, currentPosition.lon], 15);
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                attribution: '© OpenStreetMap contributors'
            }).addTo(window.map);
            window.marker = L.marker([currentPosition.lat, currentPosition.lon]).addTo(window.map)
                .bindPopup("Position UKF/GPS").openPopup();
            if ($('chargement-carte-statut')) $('chargement-carte-statut').textContent = 'Carte Chargée 🟢';
        } else {
             console.error("🔴 ERREUR: leaflet.js n'est pas chargé ou l'ID 'map-container' est manquant.");
             if ($('chargement-carte-statut')) $('chargement-carte-statut').textContent = 'Carte Désactivée 🔴';
        }

        if (typeof turf === 'undefined') {
             console.warn("🟡 AVERTISSEMENT: turf.min.js n'est pas chargé. Le calcul de la distance 3D sera moins précis.");
        }
        
        syncH(); 
        updateSensorsAndFilter(); 
        updateDOMSlow(); 
        
        if ($('gravite-base')) $('gravite-base').textContent = `${G_ACC_STD.toFixed(4)} m/s²`;
        if ($('masse-kg-display')) $('masse-kg-display').textContent = `${currentMass.toFixed(3)} kg`;
        if ($('vitesse-lumiere')) $('vitesse-lumiere').textContent = `${C_L.toFixed(0)} m/s`;
        if ($('gravitation-universelle')) $('gravitation-universelle').textContent = dataOrDefaultExp(G_U, 4, ' m³/kg/s²');
        if ($('statut-meteo')) $('statut-meteo').textContent = 'INACTIF 🔴';
    };

    document.addEventListener('DOMContentLoaded', initDashboard);
    
})(window);
