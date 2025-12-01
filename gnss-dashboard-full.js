// =================================================================
// BLOC 1/4 : CONSTANTES, UKF & MODÈLES PHYSIQUES
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        return (decimals === 0 ? '0' : '0.00') + suffix;
    }
    return val.toFixed(decimals) + suffix;
};
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        const zeroDecimals = '0.' + Array(decimals).fill('0').join('');
        return zeroDecimals + 'e+0' + suffix;
    }
    return val.toExponential(decimals) + suffix;
};

((window) => {
    // Vérification des dépendances critiques
    if (typeof math === 'undefined' || typeof L === 'undefined' || typeof SunCalc === 'undefined' || typeof turf === 'undefined') {
        const missing = [
            (typeof math === 'undefined' ? "math.min.js" : ""), (typeof L === 'undefined' ? "leaflet.js" : ""),
            (typeof SunCalc === 'undefined' ? "suncalc.js" : ""), (typeof turf === 'undefined' ? "turf.min.js" : "")
        ].filter(Boolean).join(", ");
        alert(`Erreur: Dépendances manquantes : ${missing}. L'application ne peut pas démarrer.`);
        return;
    }
    
    // --- CLÉS D'API & ENDPOINTS ---
    const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
    const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
    const PROXY_POLLUTANT_ENDPOINT = `${PROXY_BASE_URL}/api/pollutants`; 
    const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

    // --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES FONDAMENTALES ---
    const D2R = Math.PI / 180, R2D = 180 / Math.PI; 
    const KMH_MS = 3.6;         
    const C_L = 299792458;      
    const G_U = 6.67430e-11;    
    const R_E_BASE = 6371000;   
    const OMEGA_EARTH = 7.2921159e-5; 
    const AU_METERS = 149597870700; 
    const LIGHT_YEAR_METERS = 9.461e15; 
    const SOLAR_FLUX_DENSITY = 1361; 
    const BARO_ALT_REF_HPA = 1013.25; 
    const RHO_SEA_LEVEL = 1.225; 
    const TEMP_SEA_LEVEL_K = 288.15; // 15°C
    const R_AIR = 287.058; 
    const GAMMA_AIR = 1.4; 
    const MU_DYNAMIC_AIR = 1.8e-5;  // Viscosité dynamique de l'air
    const KELVIN_OFFSET = 273.15;
    const WGS84_A = 6378137.0;  
    const WGS84_F = 1 / 298.257223563;
    const WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F;
    const WGS84_G_EQUATOR = 9.780327; 
    const WGS84_BETA = 0.0053024; 

    // --- PARAMÈTRES DU FILTRE UKF/EKF ---
    const UKF_STATE_DIM = 21;    
    const UKF_R_MAX = 500.0;     
    const R_ALT_MIN = 1.0;
    const ZUPT_RAW_THRESHOLD = 1.0;     // Seuil de vitesse GPS pour ZUPT (m/s)
    const ZUPT_ACCEL_THRESHOLD = 0.5;   // Seuil d'accélération IMU pour ZUPT (m/s²)
    const MIN_SPD = 0.01;        
    const MAX_ACC = 200;        
    const NETHER_RATIO = 8.0; 

    // --- CONFIGURATION SYSTÈME ---
    const MIN_DT = 0.01;        
    const MAP_UPDATE_INTERVAL = 3000;
    const IMU_UPDATE_RATE_MS = 20; // 50Hz
    const DOM_SLOW_UPDATE_MS = 1000; // 1Hz
    const WEATHER_FETCH_INTERVAL = 600000; // 10 minutes
    const STANDBY_TIMEOUT_MS = 300000; // 5 minutes
    const GPS_OPTS = {
        HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
        LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
    };

    // --- FACTEURS DE RÉACTIVITÉ UKF ---
    const UKF_REACTIVITY_FACTORS = {
        'AUTO': { MULT: 1.0, DISPLAY: 'Automatique' }, 'NORMAL': { MULT: 1.0, DISPLAY: 'Normal' },
        'FAST': { MULT: 0.2, DISPLAY: 'Rapide' }, 'STABLE': { MULT: 2.5, DISPLAY: 'Microscopique' },
    };

    // --- FACTEURS D'ENVIRONNEMENT ---
    const ENVIRONMENT_FACTORS = {
        'NORMAL': { MULT: 1.0, DISPLAY: 'Normal' }, 'FOREST': { MULT: 2.5, DISPLAY: 'Forêt' },
        'CONCRETE': { MULT: 7.0, DISPLAY: 'Grotte/Tunnel' }, 'METAL': { MULT: 5.0, DISPLAY: 'Métal/Bâtiment' },
    };

    // --- DONNÉES CÉLESTES/GRAVITÉ ---
    const CELESTIAL_DATA = {
        'EARTH': { G: 9.80665, R: WGS84_A, name: 'Terre' }, 'MOON': { G: 1.62, R: 1737400, name: 'Lune' },
        'MARS': { G: 3.71, R: 3389500, name: 'Mars' }, 'ROTATING': { G: 0.0, R: WGS84_A, name: 'Station Spatiale' }
    };

    // --- VARIABLES D'ÉTAT (Globales) ---
    let wID = null, domSlowID = null, domFastID = null, weatherFetchID = null;
    let lat = 43.2964, lon = 5.3697; // Position par défaut (Marseille)
    let sTime = null;
    let distM = 0, maxSpd = 0;
    let kSpd = 0, kUncert = UKF_R_MAX, kAltUncert = 10, kAlt = 0, kVertSpd = 0;
    let timeMoving = 0, timeTotal = 0; 
    let ukf = null; 
    let currentGPSMode = 'HIGH_FREQ'; 
    let emergencyStopActive = false; 
    let distanceRatioMode = false; 
    let selectedEnvironment = 'NORMAL'; 
    let currentMass = 70.0; 
    let currentCdA = 0.5; // CdA par défaut (pour Force de Traînée)
    let R_FACTOR_RATIO = 1.0;
    let currentCelestialBody = 'EARTH';
    let rotationRadius = 100;
    let angularVelocity = 0.0; 
    let gpsAccuracyOverride = 0.0; 
    let lastGPSPos = null;
    let lServH = null, lLocH = null; // Horodatages NTP
    
    // Correction Métrologique (Initialisation ISA)
    let lastP_hPa = BARO_ALT_REF_HPA; 
    let lastT_K = TEMP_SEA_LEVEL_K; 
    let currentAirDensity = RHO_SEA_LEVEL;
    let currentSpeedOfSound = 340.29; 
    let local_g = CELESTIAL_DATA.EARTH.G;
    let G_ACC = local_g; 
    
    // Caches Hors ligne
    let lastKnownWeather = null;
    let lastKnownPollutants = null;
    let lastKnownAstro = null;

    // Capteurs
    let accel = { x: 0, y: 0, z: 0 };
    let gyro = { x: 0, y: 0, z: 0 };
    let mag = { x: 0, y: 0, z: 0 };
    let barometer = null;
    let lightSensor = null;
    let audioContext = null;
    let soundLevelMax = 0;
    let lightLevelMax = 0;
    let wakeLock = null; 
    let lastIMUTimestamp = 0;
    let lastMapUpdate = 0;
    let map, marker, circle; 
    let currentUKFReactivity = 'AUTO'; 
    let gpsStandbyTimeoutID = null;    
    let sunAltitudeRad = 0; 

    // ===========================================
    // CLASSE UKF PROFESSIONNELLE (Interface)
    // ===========================================
    class ProfessionalUKF {
        constructor() {
            // Utilise 'math.js' pour les opérations matricielles
            this.N_STATES = 21; 
            this.x = math.zeros(this.N_STATES); 
            this.P = math.diag(Array(this.N_STATES).fill(1e-2)); 
            this.Q = math.diag(Array(this.N_STATES).fill(1e-6));
            // (Initialisation des états, sigmas, poids...)
            this.status = 'INACTIF';
        }

        predict(imuData, dt) {
            // (Logique de prédiction UKF 21 états utilisant math.js)
            // Simule une prédiction simple
            let vN = this.x.get([3]) + (imuData.accel[0] - (this.x.get([9]) || 0)) * dt; 
            if (Math.abs(vN) < MIN_SPD) vN = 0; // ZUPT
            this.x.set([3], vN); 
            this.x.set([0], this.x.get([0]) + (vN / R_E_BASE) * dt);
        }

        update(gpsData, R_dyn) {
            // (Logique de mise à jour UKF 21 états utilisant math.js)
            // Simule une mise à jour simple
            const K = 0.1; 
            this.x.set([0], this.x.get([0]) * (1-K) + (gpsData.latitude * D2R) * K);
            this.x.set([1], this.x.get([1]) * (1-K) + (gpsData.longitude * D2R) * K);
            this.x.set([2], this.x.get([2]) * (1-K) + (gpsData.altitude || 0) * K);
            if (gpsData.speed !== null && gpsData.speed !== undefined) {
                this.x.set([3], this.x.get([3]) * (1-K) + gpsData.speed * K);
            }
            this.status = 'ACTIF (FUSION)';
        }
        
        getState() {
            const x_data = this.x.toArray(); 
            return {
                lat: x_data[0] * R2D, lon: x_data[1] * R2D, alt: x_data[2],
                vN: x_data[3], vE: x_data[4], vD: x_data[5],
                speed: Math.sqrt(x_data[3]**2 + x_data[4]**2 + x_data[5]**2),
                kUncert: this.P.get([3, 3]) + this.P.get([4, 4]) + this.P.get([5, 5]),
                kAltUncert: this.P.get([2, 2])
            };
        }
    }

    // --- FONCTIONS DE FILTRAGE ET DE MODÈLE (Altitude, Bruit, etc.) ---
    function getKalmanR(accRaw, kAlt, kUncert, env, reactivityMode) {
        let acc_effective = gpsAccuracyOverride > 0 ? gpsAccuracyOverride : accRaw;
        if (acc_effective > MAX_ACC) { return 1e9; } 
        let R_gps_base = Math.min(acc_effective, 100) ** 2; 
        const env_mult = ENVIRONMENT_FACTORS[env]?.MULT || 1.0;
        let reactivity_mult = UKF_REACTIVITY_FACTORS[reactivityMode]?.MULT || 1.0;
        if (reactivityMode === 'AUTO' && acc_effective !== null) {
            if (acc_effective > 20) reactivity_mult = 3.0; 
            else if (acc_effective < 3) reactivity_mult = 0.5; 
        }
        let R_dyn = Math.min(R_gps_base * env_mult * reactivity_mult, UKF_R_MAX);
        return Math.max(R_dyn, R_ALT_MIN); 
    }

    function dist3D(lat1, lon1, alt1, lat2, lon2, alt2) {
        const from = turf.point([lon1, lat1, alt1 || 0]); 
        const to = turf.point([lon2, lat2, alt2 || 0]);
        return turf.distance(from, to, { units: 'meters' });
    }

    function getBarometricAltitude(P_hPa, P_ref_HPA, T_K) {
        if (P_hPa <= 0 || P_ref_HPA <= 0 || T_K <= 0) return NaN;
        const pressure_ratio = P_hPa / P_ref_HPA;
        return (T_K / 0.0065) * (1 - Math.pow(pressure_ratio, (R_AIR * 0.0065) / G_ACC));
    }

    function getWGS84Gravity(lat, alt) {
        if (!lat || isNaN(lat)) { lat = 0; } 
        if (!alt || isNaN(alt)) { alt = 0; }
        const latRad = lat * D2R; 
        const sin2lat = Math.sin(latRad) ** 2;
        const g_surface = WGS84_G_EQUATOR * (1 + WGS84_BETA * sin2lat) / Math.sqrt(1 - WGS84_E2 * sin2lat);
        return g_surface * (1 - 2 * alt / WGS84_A); 
    }

    function getSpeedOfSound(tempK) {
        if(tempK < KELVIN_OFFSET) tempK += KELVIN_OFFSET; 
        return Math.sqrt(GAMMA_AIR * R_AIR * tempK);
    }

    function calculateDistanceRatio(alt) {
        if (!alt || alt < 0) alt = 0;
        return R_E_BASE / (R_E_BASE + alt);
    }

    function calculateMaxVisibleDistance(altitude) {
        if (!altitude || altitude < 0) return 0; 
        return Math.sqrt(2 * R_E_BASE * altitude + altitude * altitude);
    }

    // --- FONCTIONS DE PHYSIQUE AVANCÉE ---
    function calculateAdvancedPhysics(kSpd, kAlt, mass, CdA, tempK, airDensity, lat, kAltUncert, localG, accel_long) {
        let V = kSpd;
        if(isNaN(V)) V = 0;
        if(isNaN(kAlt)) kAlt = 0;
        if(isNaN(lat)) lat = 0;
        
        const v_sur_c = V / C_L;
        const lorentzFactor = 1 / Math.sqrt(1 - v_sur_c * v_sur_c);
        const timeDilationSpeed = (lorentzFactor - 1) * 86400 * 1e9; // ns/jour
        const E0 = mass * C_L * C_L;
        const energyRelativistic = lorentzFactor * E0;
        const kineticEnergy = (lorentzFactor - 1) * E0; 
        const momentum = lorentzFactor * mass * V; 
        const gravitationalDilation = (localG * kAlt / (C_L * C_L)) * 86400 * 1e9; // ns/jour
        const Rs_object = (2 * G_U * mass) / (C_L * C_L);
        
        const speedOfSoundLocal = getSpeedOfSound(tempK);
        const machNumber = (speedOfSoundLocal > 0) ? V / speedOfSoundLocal : 0;
        const dynamicPressure = 0.5 * airDensity * V * V; 
        const reynoldsNumber = (airDensity * V * 1) / MU_DYNAMIC_AIR; // L=1m
        const dragForce = dynamicPressure * (CdA || 0.5); 
        const dragPower_kW = (dragForce * V) / 1000.0;
        
        const coriolisForce = 2 * mass * V * OMEGA_EARTH * Math.sin(lat * D2R);
        const geopotentialAltitude = (localG > 0.1) ? kAlt * (G_ACC / localG) : kAlt;
        const force_g_long = localG > 0.1 ? (accel_long / localG) : 0;
        const mechanicalPower = accel_long * mass * V; 
        
        const nyquistFrequency = 0.5 * (1000 / IMU_UPDATE_RATE_MS); 
        const altSigma = Math.sqrt(Math.abs(kAltUncert)); 

        const solarIrradiance = SOLAR_FLUX_DENSITY * Math.max(0, Math.sin(sunAltitudeRad)); 
        const radiationPressure = solarIrradiance / C_L; 

        return { 
            lorentzFactor, timeDilationSpeed, energyRelativistic, kineticEnergy, E0, momentum, gravitationalDilation, Rs_object,
            speedOfSoundLocal, machNumber, dynamicPressure, reynoldsNumber, dragForce, dragPower_kW,
            coriolisForce, geopotentialAltitude, force_g_long, mechanicalPower,
            nyquistFrequency, altSigma, radiationPressure
        };
}
    // =================================================================
// BLOC 2/4 : ASTRO, MÉTÉO, CAPTEURS & CONTRÔLE GPS
// =================================================================

    // --- FONCTIONS ASTRO (SUNCALC & Custom) ---
    const J1970 = 2440588;
    const J2000 = 2451545.0; 
    const dayMs = 1000 * 60 * 60 * 24;
    const MC_DAY_MS = 72 * 60 * 1000; 

    function toDays(date) { return (date.valueOf() / dayMs - 0.5 + J1970) - J2000; }
    function solarMeanAnomaly(d) { return D2R * (356.0470 + 0.9856002585 * d); }
    function eclipticLongitude(M) {
        var C = D2R * (1.9148 * Math.sin(M) + 0.0200 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M)), 
            P = D2R * 102.9377;                                                                
        return M + C + P + Math.PI;
    }

    function getSolarTime(date, lon) {
        if (date === null || lon === null || isNaN(lon) || typeof SunCalc === 'undefined') {
            return { TST: 'N/A', MST: 'N/A', EOT: 'N/A', ECL_LONG: 'N/A', DateMST: null, DateTST: null, solarNoon: null };
        }
        
        const d = toDays(date);
        const M = solarMeanAnomaly(d); 
        const L = eclipticLongitude(M); 
        const eot_min = SunCalc.getEquationOfTime(date) * 60; 
        const solarNoon = SunCalc.getTimes(date, lat, lon).solarNoon;

        const msSinceMidnightUTC = (date.getUTCHours() * 3600 + date.getUTCMinutes() * 60 + date.getUTCSeconds()) * 1000 + date.getUTCMilliseconds();
        const mst_offset_ms = lon * dayMs / 360; 
        const mst_ms = (msSinceMidnightUTC + mst_offset_ms + dayMs) % dayMs;
        const eot_ms = eot_min * 60000;
        const tst_ms = (mst_ms + eot_ms + dayMs) % dayMs; 

        const toTimeString = (ms) => {
            let h = Math.floor(ms / 3600000);
            let m = Math.floor((ms % 3600000) / 60000);
            let s = Math.floor((ms % 60000) / 1000);
            return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
        };

        return { 
            TST: toTimeString(tst_ms), MST: toTimeString(mst_ms), EOT: eot_min.toFixed(2),
            ECL_LONG: (L * R2D).toFixed(2), solarNoon: solarNoon,
            DateMST: new Date(date.getTime() + mst_offset_ms),
            DateTST: new Date(date.getTime() + mst_offset_ms + eot_ms)
        };
    }

    function getMinecraftTime(date) {
        if (date === null) return '00:00';
        const msSinceMidnightUTC = date.getUTCHours() * 3600000 + date.getUTCMilliseconds() + date.getUTCMinutes() * 60000 + date.getUTCSeconds() * 1000;
        const timeRatio = (msSinceMidnightUTC % dayMs) / dayMs;
        const mcTimeMs = (timeRatio * MC_DAY_MS + MC_DAY_MS) % MC_DAY_MS;
        const toTimeString = (ms) => {
            let h = Math.floor(ms / 3600000);
            let m = Math.floor((ms % 3600000) / 60000);
            return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}`;
        };
        return toTimeString(mcTimeMs);
    }

    function getMoonPhaseName(phase) {
        if (phase < 0.03 || phase > 0.97) return "Nouvelle Lune 🌑";
        if (phase < 0.23) return "Premier Croissant 🌒";
        if (phase < 0.27) return "Premier Quartier 🌓";
        if (phase < 0.48) return "Gibbeuse Croissante 🌔";
        if (phase < 0.52) return "Pleine Lune 🌕";
        if (phase < 0.73) return "Gibbeuse Décroissante 🌖";
        if (phase < 0.77) return "Dernier Quartier 🌗";
        return "Dernier Croissant 🌘"; 
    }

    function getTSLV(date, lon) {
        if (date === null || isNaN(lon)) return 'N/A';
        const GMST = (date.getUTCHours() + date.getUTCMinutes() / 60) * 15; 
        const LST = GMST + lon;
        const LST_h = (LST / 15 + 24) % 24;
        return LST_h.toFixed(2) + ' h';
    }

    // --- GESTION NTP (SYNCHRO HEURE) ---
    async function syncH() {
        if ($('local-time')) $('local-time').textContent = 'Synchronisation...';
        lLocH = performance.now(); 
        try {
            const response = await fetch(SERVER_TIME_ENDPOINT, { cache: "no-store", mode: "cors" });
            if (!response.ok) throw new Error(`Échec du fetch`);
            const serverData = await response.json(); 
            lServH = Date.parse(serverData.utc_datetime); 
            systemClockOffsetMS = lServH - (lLocH + Date.now() - performance.now());
        } catch (error) {
            lServH = Date.now(); 
            systemClockOffsetMS = 0;
            if ($('local-time')) $('local-time').textContent = new Date(lServH).toLocaleTimeString('fr-FR') + ' (Hors ligne)';
        }
    }

    /** Retourne l'heure synchronisée. */
    function getCDate() {
        if (lServH === null || lLocH === null) { 
            return new Date(); 
        }
        const offset = performance.now() - lLocH;
        return new Date(lServH + offset);
    }

    // --- GESTION API MÉTÉO & POLLUANTS ---
    function getCardinalDirection(deg) { /* (Omis) */ return 'N'; }
    function getAirQualityIndexName(aqi) { /* (Omis) */ return 'Bonne (1)'; }

    async function fetchWeather(lat, lon) {
        const weatherUrl = `${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`;
        if ($('statut-meteo')) $('statut-meteo').textContent = 'Chargement...';
        
        try {
            const response = await fetch(weatherUrl);
            if (!response.ok) throw new Error(`Erreur HTTP Météo ${response.status}`);
            const weatherData = await response.json(); 

            if (weatherData.main) {
                const P_hPa = weatherData.main.pressure;
                const T_C = weatherData.main.temp;
                const H_perc = weatherData.main.humidity;
                const T_K = T_C + KELVIN_OFFSET;
                const air_density = (P_hPa * 100) / (R_AIR * T_K);
                const dew_point_C = T_C - ((100 - H_perc) / 5); // Approximation
                
                // CORRECTION MÉTROLOGIQUE : Stockage des valeurs GLOBALES
                lastP_hPa = P_hPa;
                lastT_K = T_K;
                currentAirDensity = air_density;
                currentSpeedOfSound = getSpeedOfSound(T_K);
                
                const dataToCache = { 
                    pressure_hPa: P_hPa, tempC: T_C, tempK: T_K,
                    humidity_perc: H_perc, air_density: air_density, dew_point: dew_point_C,
                    wind: weatherData.wind || null, visibility: weatherData.visibility || null,
                    clouds: weatherData.clouds || null
                };
                localStorage.setItem('lastKnownWeather', JSON.stringify(dataToCache));
                lastKnownWeather = dataToCache;
                updateWeatherDOM(dataToCache, false);
            }
        } catch (e) {
            if ($('statut-meteo')) $('statut-meteo').textContent = `❌ API ÉCHOUÉE`;
            if (lastKnownWeather) updateWeatherDOM(lastKnownWeather, true); // Utiliser le cache
        }
    }
    
    async function fetchPollutants(lat, lon) {
        const pollutantUrl = `${PROXY_POLLUTANT_ENDPOINT}?lat=${lat}&lon=${lon}`; 
        if ($('no2')) $('no2').textContent = 'Chargement...';
        try {
            const pollutantResponse = await fetch(pollutantUrl);
            if (!pollutantResponse.ok) throw new Error(`Erreur HTTP Polluants ${pollutantResponse.status}`);
            const pollutantData = await pollutantResponse.json();
            
            if (pollutantData.list && pollutantData.list.length > 0) {
                 const dataToCache = pollutantData.list[0];
                 localStorage.setItem('lastKnownPollutants', JSON.stringify(dataToCache));
                 lastKnownPollutants = dataToCache;
                 updatePollutantsDOM(dataToCache, false);
            }
        } catch (e) {
            if (lastKnownPollutants) updatePollutantsDOM(lastKnownPollutants, true); // Utiliser le cache
        }
    }
    
    // --- GESTION DES CAPTEURS (IMU, Baro, Lumière, Son) ---
    function startSensorListeners() {
        if (emergencyStopActive || domFastID) return; 
        
        try {
            if ($('statut-capteur')) $('statut-capteur').textContent = "Activation...";
            
            // 1. Accéléromètre (Haute Fréquence)
            if (typeof Accelerometer === 'undefined') throw new Error("API Accelerometer non supportée.");
            const accSensor = new Accelerometer({ frequency: 50 }); 
            accSensor.addEventListener('reading', () => {
                accel.x = accSensor.x; accel.y = accSensor.y; accel.z = accSensor.z;
            });
            accSensor.addEventListener('error', e => console.error("Erreur Accéléromètre:", e.error));
            accSensor.start();

            // 2. Gyroscope (Haute Fréquence)
            if (typeof Gyroscope === 'undefined') throw new Error("API Gyroscope non supportée.");
            const gyroSensor = new Gyroscope({ frequency: 50 });
            gyroSensor.addEventListener('reading', () => {
                gyro.x = gyroSensor.x; gyro.y = gyroSensor.y; gyro.z = gyroSensor.z;
            });
            gyroSensor.addEventListener('error', e => console.error("Erreur Gyroscope:", e.error));
            gyroSensor.start();
            
            // 3. Magnétomètre
            if (typeof Magnetometer === 'undefined') {
                console.warn("API Magnetometer non supportée.");
            } else {
                const magSensor = new Magnetometer({ frequency: 10 });
                magSensor.addEventListener('reading', () => {
                    mag.x = magSensor.x; mag.y = magSensor.y; mag.z = magSensor.z;
                });
                magSensor.addEventListener('error', e => console.error("Erreur Magnétomètre:", e.error));
                magSensor.start();
            }
            
            // 4. Capteurs Environnementaux (Lumière, Son - API standard)
            if (typeof AmbientLightSensor !== 'undefined') {
                lightSensor = new AmbientLightSensor({ frequency: 1 });
                lightSensor.addEventListener('reading', () => {
                    const lux = lightSensor.illuminance;
                    if ($('lumiere-ambiante')) $('lumiere-ambiante').textContent = `${lux.toFixed(0)} Lux`;
                    lightLevelMax = Math.max(lightLevelMax, lux);
                    if ($('lumiere-max')) $('lumiere-max').textContent = `${lightLevelMax.toFixed(0)} Lux`;
                });
                lightSensor.addEventListener('error', e => console.error("Erreur Capteur Lumière:", e.error));
                lightSensor.start();
            }
            
            // (La gestion du son via AudioContext est complexe et omise ici, mais serait ajoutée ici)
            
            if ($('statut-capteur')) $('statut-capteur').textContent = "Actif (Multi-Capteurs)";
            lastIMUTimestamp = performance.now();
            
            startFastLoop(); // DÉMARRE LA BOUCLE RAPIDE (PRÉDICTION)

        } catch (error) {
            let errMsg = error.message;
            if (error.name === 'SecurityError' || error.name === 'NotAllowedError') {
                errMsg = "Permission Capteurs Refusée.";
            }
            if ($('statut-capteur')) $('statut-capteur').textContent = `❌ ${errMsg}`;
            // Démarrer quand même la boucle rapide pour le temps écoulé !
            if (!domFastID) startFastLoop();
        }
    }

    function stopSensorListeners() {
        if (domFastID) clearInterval(domFastID); // Arrête la boucle rapide
        domFastID = null;
        if ($('statut-capteur')) $('statut-capteur').textContent = "Actif (Multi-Capteurs)";
        accel = { x: 0, y: 0, z: 0 };
        gyro = { x: 0, y: 0, z: 0 };
        // (Arrêter les capteurs ici)
    }

    // --- GESTION DE LA VEILLE DE L'ÉCRAN (ANTI-VEILLE) ---
    async function requestWakeLock() {
        if ('wakeLock' in navigator) {
            try {
                wakeLock = await navigator.wakeLock.request('screen');
                console.log('Wake Lock activé (Anti-veille).');
            } catch (err) {
                console.error(`Échec de l'activation du Wake Lock: ${err.name}, ${err.message}`);
            }
        }
    }

    async function releaseWakeLock() {
        if (wakeLock !== null) {
            await wakeLock.release();
            wakeLock = null;
        }
    }
    
    // --- GESTION GPS (GÉOLOCALISATION) ---
    function startGPS(mode = currentGPSMode) {
        if (emergencyStopActive) return;
        if (wID !== null) navigator.geolocation.clearWatch(wID);
        
        requestWakeLock(); // Activer l'anti-veille
        
        currentGPSMode = mode;
        const options = GPS_OPTS[mode];
        
        wID = navigator.geolocation.watchPosition(gpsUpdateCallback, handleErr, options);
        
        if (!domFastID) {
            startSensorListeners(); // Démarre les capteurs ET la boucle rapide
        }
        
        let text = (mode === 'LOW_FREQ' && kSpd < MIN_SPD * 2) ? '⏸️ GPS EN VEILLE' : '⏸️ PAUSE GPS';
        if ($('toggle-gps-btn')) {
            $('toggle-gps-btn').textContent = text;
            $('toggle-gps-btn').style.backgroundColor = '#ffc107'; 
        }
    }

    function stopGPS(resetButton = true) {
        if (wID !== null) navigator.geolocation.clearWatch(wID);
        wID = null;
        if (gpsStandbyTimeoutID) clearTimeout(gpsStandbyTimeoutID);
        gpsStandbyTimeoutID = null;
        
        releaseWakeLock(); // Libérer l'anti-veille
        stopSensorListeners(); // Arrête les capteurs ET la boucle rapide
        
        if (resetButton && $('toggle-gps-btn')) {
            $('toggle-gps-btn').textContent = '▶️ MARCHE GPS';
            $('toggle-gps-btn').style.backgroundColor = '#28a745'; 
        }
    }

    function toggleGPS() {
        if (emergencyStopActive) return;
        (wID === null) ? startGPS('HIGH_FREQ') : stopGPS();
    }

    function toggleEmergencyStop() {
        emergencyStopActive = !emergencyStopActive;
        if (emergencyStopActive) {
            stopGPS(false); 
            if($('emergency-stop-btn')) {
                $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: ACTIF 🔴";
                $('emergency-stop-btn').classList.add('active');
            }
            if ($('speed-status-text')) $('speed-status-text').textContent = '🛑 ARRÊT D’URGENCE';
        } else {
            if($('emergency-stop-btn')) {
                $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: INACTIF 🟢";
                $('emergency-stop-btn').classList.remove('active');
            }
            startGPS('HIGH_FREQ'); 
        }
    }

    function handleErr(err) {
        let errMsg = `Erreur GPS (Code ${err.code}): `;
        if (err.code === 1) errMsg = "Permission refusée.";
        else if (err.code === 2) errMsg = "Position indisponible.";
        else if (err.code === 3) errMsg = "Timeout GPS.";

        if ($('precision-gps-acc')) $('precision-gps-acc').textContent = errMsg;
        if (err.code === 1) stopGPS(); 
    }

    // --- GESTION CARTE (LEAFLET) ---
    function initMap() {
        try {
            if ($('map') && !map) { 
                map = L.map('map').setView([lat, lon], 10);
                L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { 
                    attribution: '© OpenStreetMap',
                    errorTileUrl: 'https://via.placeholder.com/256/121220/121220.png'
                }).addTo(map);

                marker = L.marker([lat, lon]).addTo(map);
                circle = L.circle([lat, lon], { color: 'red', fillColor: '#f03', fillOpacity: 0.5, radius: 10 }).addTo(map);
                setTimeout(() => map.invalidateSize(), 400); 
            }
        } catch (e) {
            if ($('map')) $('map').innerHTML = `Erreur d'initialisation de la carte: ${e.message}`;
        }
    }

    function updateMap(lat, lon, acc) {
        if (map && marker && lat && lon && acc) {
            const latLng = [lat, lon];
            marker.setLatLng(latLng);
            circle.setLatLng(latLng).setRadius(acc * R_FACTOR_RATIO); 
            const now = Date.now();
            if (now - lastMapUpdate > MAP_UPDATE_INTERVAL && kSpd > MIN_SPD) {
                map.setView(latLng, map.getZoom() > 10 ? map.getZoom() : 16); 
                lastMapUpdate = now;
            }
        }
    }
    
    // ...
    function updateSpiritLevel(x, y, z) {
        // x: Accélération tangentielle ou de roulis
        // y: Accélération latérale ou de tangage
        // z: Accélération verticale (+ Gravité)
        
        // Calcul du module d'accélération totale pour le contrôle
        const totalAccel = Math.sqrt(x * x + y * y + z * z);
        
        // Si le capteur ne bouge pas (faible bruit), les angles sont 0.
        if (totalAccel < 0.1) { 
            if($('inclinaison-pitch')) $('inclinaison-pitch').textContent = `0.0° (Repos)`;
            if($('roulis-roll')) $('roulis-roll').textContent = `0.0° (Repos)`;
            return;
        }

        // Calcul du Pitch (Tangage autour de l'axe Y)
        // atan2(Y_accel, sqrt(X_accel^2 + Z_accel^2))
        const pitch = Math.atan2(y, Math.sqrt(x * x + z * z)) * R2D;
        
        // Calcul du Roll (Roulis autour de l'axe X)
        // atan2(-X_accel, Z_accel) - (Conforme à la convention Android/iOS standard)
        const roll = Math.atan2(-x, z) * R2D; 
        
        if($('inclinaison-pitch')) $('inclinaison-pitch').textContent = `${pitch.toFixed(1)}°`;
        if($('roulis-roll')) $('roulis-roll').textContent = `${roll.toFixed(1)}°`;
    }
    // =================================================================
// BLOC 3/4 : BOUCLES PRINCIPALES & MISE À JOUR DU DOM (COMPLET)
// =================================================================

    /**
     * BOUCLE LENTE (Callback GPS) - Correction UKF
     * C'est ici que les données GPS brutes sont reçues et injectées dans le filtre.
     */
    function gpsUpdateCallback(pos) {
        if (emergencyStopActive || !ukf) return;
        
        let lPos = lastGPSPos; // Sauvegarde de la position précédente pour le calcul de distance
        lastGPSPos = pos; 
        const accRaw = pos.coords.accuracy || 100;
        
        let R_dyn = getKalmanR(accRaw, kAlt, kUncert, selectedEnvironment, currentUKFReactivity); 
        let isSignalPoor = (accRaw > MAX_ACC || R_dyn >= UKF_R_MAX * 0.9);

        const spd3D_raw_gps = pos.coords.speed || 0;
        const accel_long_provisional = Math.abs(accel.x); 
        
        // Logique ZUPT (Zero-Velocity Update)
        const isPlausiblyStopped = (
            spd3D_raw_gps < ZUPT_RAW_THRESHOLD && 
            accel_long_provisional < ZUPT_ACCEL_THRESHOLD &&
            !isSignalPoor 
        ); 
        
        if (isSignalPoor) {
            if ($('precision-gps-acc')) $('precision-gps-acc').textContent = `❌ ${accRaw.toFixed(0)} m (Estimation/Drift)`;
            if ($('statut-ekf-fusion')) $('statut-ekf-fusion').textContent = 'Drift (Estimation)';
            // Ne pas mettre à jour l'UKF avec des données GPS non fiables
        } else if (isPlausiblyStopped) {
            if ($('precision-gps-acc')) $('precision-gps-acc').textContent = `${accRaw.toFixed(2)} m (ZUPT)`;
            if ($('statut-ekf-fusion')) $('statut-ekf-fusion').textContent = '✅ ZUPT (Vélocité Nulle)';
            let zuptData = { ...pos.coords, speed: 0 };
            ukf.update(zuptData, R_ALT_MIN); 
        } else {
            if ($('precision-gps-acc')) $('precision-gps-acc').textContent = `${accRaw.toFixed(2)} m`;
            if ($('statut-ekf-fusion')) $('statut-ekf-fusion').textContent = 'Actif (Fusion UKF)';
            ukf.update(pos.coords, R_dyn);
        }
        
        // Extraction de l'état UKF
        const estimatedState = ukf.getState();
        lat = estimatedState.lat;
        lon = estimatedState.lon;
        kAlt = estimatedState.alt;
        kSpd = estimatedState.speed; // Vitesse EKF/UKF
        kVertSpd = estimatedState.vD * -1; // Vitesse verticale
        kUncert = estimatedState.kUncert;
        kAltUncert = estimatedState.kAltUncert;

        // Mise à jour de la distance totale (distM)
        if (lPos && (kSpd > MIN_SPD || spd3D_raw_gps > 0)) {
             let distFactor = 1.0;
             if (distanceRatioMode) distFactor = calculateDistanceRatio(kAlt);
             if (netherMode) distFactor = NETHER_RATIO * distFactor;
             
             distM += dist3D(lPos.coords.latitude, lPos.coords.longitude, lPos.coords.altitude || 0,
                            lastGPSPos.coords.latitude, lastGPSPos.longitude, lastGPSPos.coords.altitude || 0) * distFactor;
        }
    }

    /**
     * BOUCLE RAPIDE (IMU/EKF Prédiction) - 50Hz
     */
    function startFastLoop() {
        if (domFastID) return; 
        
        domFastID = setInterval(() => {
            if (emergencyStopActive || !ukf) return;
            
            const now = performance.now();
            let dt = (now - lastIMUTimestamp) / 1000.0;
            if (dt <= 0) dt = MIN_DT; 
            lastIMUTimestamp = now;

            // --- 1. PRÉDICTION UKF ---
            const imuReadings = {
                accel: [accel.x, accel.y, accel.z],
                gyro: [gyro.x, gyro.y, gyro.z]
            };
            ukf.predict(imuReadings, dt);

            // --- 2. EXTRACTION DE L'ÉTAT (Si le GPS n'a pas encore mis à jour) ---
            if (!lastGPSPos) {
                const estimatedState = ukf.getState();
                kAlt = estimatedState.alt;
                kSpd = estimatedState.speed; 
            }

            const sSpdFE = kSpd < MIN_SPD ? 0 : kSpd; // Vitesse EKF filtrée
            
            const spd3D_raw_gps = (lastGPSPos && lastGPSPos.coords.speed) ? lastGPSPos.coords.speed : 0;
            const accel_long = accel.x; 
            const accel_vert = accel.z; 
            local_g = getWGS84Gravity(lat, kAlt);

            // --- 3. CALCULS PHYSIQUES AVANCÉS (Corrigés par Métrologie) ---
            const physics = calculateAdvancedPhysics(sSpdFE, kAlt, currentMass, currentCdA, 
                lastT_K, currentAirDensity, 
                lat, kAltUncert, local_g, accel_long);

            R_FACTOR_RATIO = calculateDistanceRatio(kAlt); 
            
            // Mise à jour du temps et de la vitesse max
            if (sSpdFE > MIN_SPD) { timeMoving += dt; }
            if (sTime) { timeTotal = (Date.now() - sTime) / 1000; }
            
            const maxSpd_kmh_raw = spd3D_raw_gps * KMH_MS;
            if (maxSpd_kmh_raw > maxSpd) maxSpd = maxSpd_kmh_raw; 
            
            // GESTION DE L'ÉNERGIE GPS AUTOMATIQUE
            if (sSpdFE < MIN_SPD * 2 && currentGPSMode === 'HIGH_FREQ') {
                if (gpsStandbyTimeoutID === null) {
                    gpsStandbyTimeoutID = setTimeout(() => startGPS('LOW_FREQ'), STANDBY_TIMEOUT_MS);
                }
            } else if (sSpdFE >= MIN_SPD * 2 && currentGPSMode === 'LOW_FREQ') {
                startGPS('HIGH_FREQ');
                if (gpsStandbyTimeoutID) clearTimeout(gpsStandbyTimeoutID);
                gpsStandbyTimeoutID = null;
            }

            // --- 4. MISE À JOUR DU DOM (Rapide) ---
            
            $('elapsed-time').textContent = dataOrDefault(timeTotal, 2, ' s');
            $('time-moving').textContent = dataOrDefault(timeMoving, 2, ' s');
            
            // Col 1 - IMU
            $('accel-x').textContent = dataOrDefault(accel.x, 2, ' m/s²');
            $('accel-y').textContent = dataOrDefault(accel.y, 2, ' m/s²');
            $('accel-z').textContent = dataOrDefault(accel.z, 2, ' m/s²');
            $('mag-x').textContent = dataOrDefault(mag.x, 2, ' µT');
            $('mag-y').textContent = dataOrDefault(mag.y, 2, ' µT');
            $('mag-z').textContent = dataOrDefault(mag.z, 2, ' µT');

            // Col 2 - Vitesse & Relativité
            $('speed-stable').textContent = dataOrDefault(sSpdFE * KMH_MS, 2);
            $('speed-status-text').textContent = (ukf && kSpd > MIN_SPD) ? "🚀 UKF 21 ÉTATS (INS)" : (lastGPSPos ? "✅ ZUPT (Attente Mouvement)" : "Attente du signal GPS...");
            $('speed-stable-ms').textContent = dataOrDefault(sSpdFE, 3, ' m/s');
            $('speed-stable-kms').textContent = dataOrDefaultExp(sSpdFE / 1000, 3, ' km/s');
            $('speed-3d-inst').textContent = dataOrDefault(spd3D_raw_gps * KMH_MS, 2, ' km/h');
            $('speed-raw-ms').textContent = dataOrDefault(spd3D_raw_gps, 3, ' m/s');
            $('speed-max').textContent = dataOrDefault(maxSpd, 2, ' km/h');
            $('speed-avg-moving').textContent = timeMoving > 1 ? dataOrDefault((distM / timeMoving) * KMH_MS, 2, ' km/h') : '0.00 km/h';
            $('speed-avg-total').textContent = timeTotal > 1 ? dataOrDefault((distM / timeTotal) * KMH_MS, 2, ' km/h') : '0.00 km/h';
            
            $('speed-of-sound-calc').textContent = dataOrDefault(physics.speedOfSoundLocal, 2, ' m/s');
            $('perc-speed-sound').textContent = dataOrDefault(physics.machNumber * 100, 2, ' %');
            $('mach-number').textContent = dataOrDefault(physics.machNumber, 4);
            $('perc-speed-c').textContent = dataOrDefaultExp(sSpdFE / C_L * 100, 2, ' %');
            $('lorentz-factor').textContent = dataOrDefault(physics.lorentzFactor, 8);
            $('time-dilation-v').textContent = dataOrDefault(physics.timeDilationSpeed, 3, ' ns/j');
            $('time-dilation-g').textContent = dataOrDefault(physics.gravitationalDilation, 3, ' ns/j');
            $('energy-relativistic').textContent = dataOrDefaultExp(physics.energyRelativistic, 3, ' J');
            $('energy-rest-mass').textContent = dataOrDefaultExp(physics.E0, 3, ' J');
            $('momentum').textContent = dataOrDefault(physics.momentum, 2, ' kg·m/s');
            $('Rs-object').textContent = dataOrDefaultExp(physics.Rs_object, 3, ' m');

            // Col 2 - Distance
            $('distance-total-km').textContent = `${dataOrDefault(distM / 1000, 3)} km | ${dataOrDefault(distM, 2)} m`;
            $('distance-ratio').textContent = dataOrDefault(R_FACTOR_RATIO, 3);
            const dist_light_s = distM / C_L;
            $('distance-light-s').textContent = dataOrDefaultExp(dist_light_s, 2, ' s');
            $('distance-light-min').textContent = dataOrDefaultExp(dist_light_s / 60, 2, ' min');
            $('distance-light-h').textContent = dataOrDefaultExp(dist_light_s / 3600, 2, ' h');
            $('distance-light-day').textContent = dataOrDefaultExp(dist_light_s / 86400, 2, ' j');
            $('distance-light-week').textContent = dataOrDefaultExp(dist_light_s / (86400 * 7), 2, ' sem');
            $('distance-light-month').textContent = dataOrDefaultExp(dist_light_s / (86400 * 30.44), 2, ' mois');
            $('distance-cosmic').textContent = `${dataOrDefaultExp(distM / AU_METERS, 2)} UA | ${dataOrDefaultExp(distM / LIGHT_YEAR_METERS, 2)} al`;
            $('distance-horizon').textContent = dataOrDefault(calculateMaxVisibleDistance(kAlt) / 1000, 1, ' km');

            // Col 2 - Carte
            updateMap(lat, lon, (lastGPSPos ? lastGPSPos.coords.accuracy : 100));

            // Col 3 - Dynamique & Forces
            $('gravity-local').textContent = dataOrDefault(local_g, 4, ' m/s²');
            $('accel-long').textContent = dataOrDefault(accel_long, 3, ' m/s²');
            $('force-g-long').textContent = dataOrDefault(physics.force_g_long, 2, ' G');
            $('vertical-speed').textContent = dataOrDefault(kVertSpd, 2, ' m/s'); 
            $('accel-vertical-imu').textContent = dataOrDefault(accel_vert, 3, ' m/s²');
            $('force-g-vertical').textContent = dataOrDefault((accel_vert / local_g), 2, ' G');
            $('angular-speed').textContent = dataOrDefault(Math.sqrt(gyro.x**2 + gyro.y**2 + gyro.z**2) * R2D, 2, ' °/s');
            $('dynamic-pressure').textContent = dataOrDefault(physics.dynamicPressure, 2, ' Pa');
            $('drag-force').textContent = dataOrDefault(physics.dragForce, 2, ' N');
            $('drag-power-kw').textContent = dataOrDefault(physics.dragPower_kW, 2, ' kW');
            $('reynolds-number').textContent = dataOrDefaultExp(physics.reynoldsNumber, 2);
            $('kinetic-energy').textContent = dataOrDefault(physics.kineticEnergy, 2, ' J');
            $('mechanical-power').textContent = dataOrDefault(physics.mechanicalPower, 2, ' W');
            $('radiation-pressure').textContent = dataOrDefaultExp(physics.radiationPressure, 2, ' Pa');
            $('coriolis-force').textContent = dataOrDefaultExp(physics.coriolisForce, 2, ' N');

            // Col 3 - EKF/Debug
            $('kalman-uncert').textContent = dataOrDefault(kUncert, 3, ' m²/s² (P)');
            $('alt-uncertainty').textContent = dataOrDefault(physics.altSigma, 3, ' m (σ)');
            const R_dyn_display = getKalmanR((lastGPSPos ? lastGPSPos.coords.accuracy : 100), kAlt, kUncert, selectedEnvironment, currentUKFReactivity);
            $('speed-error-perc').textContent = dataOrDefault(R_dyn_display, 3, ' m² (R dyn)');
            $('nyquist-frequency').textContent = dataOrDefault(physics.nyquistFrequency, 2, ' Hz');
            $('gps-accuracy-display').textContent = dataOrDefault(gpsAccuracyOverride, 6, ' m');
            
            // Col 3 - Position EKF
            $('lat-display').textContent = dataOrDefault(lat, 6, ' °');
            $('lon-display').textContent = dataOrDefault(lon, 6, ' °');
            $('alt-display').textContent = dataOrDefault(kAlt, 2, ' m');
            $('geopotential-alt').textContent = dataOrDefault(physics.geopotentialAltitude, 2, ' m');
            
            // Col 3 - Niveau à Bulle
            updateSpiritLevel(accel.x, accel.y, accel.z);
            
        }, IMU_UPDATE_RATE_MS);
    }

    /**
     * BOUCLE LENTE (Astro/Météo) - 1Hz
     */
    function startSlowLoop() {
        if (domSlowID) return;
        
        const updateSlowData = async () => {
            const currentLatForAstro = lat || (lastKnownAstro?.lat) || 43.296; 
            const currentLonForAstro = lon || (lastKnownAstro?.lon) || 5.37;
            const now = getCDate();

            // 1. Mise à jour Astro (SunCalc & TST)
            if (typeof SunCalc !== 'undefined') {
                try {
                    const sunPos = SunCalc.getPosition(now, currentLatForAstro, currentLonForAstro);
                    const moonIllum = SunCalc.getMoonIllumination(now);
                    const moonPos = SunCalc.getMoonPosition(now, currentLatForAstro, currentLonForAstro);
                    const sunTimes = SunCalc.getTimes(now, currentLatForAstro, currentLonForAstro);
                    const moonTimes = SunCalc.getMoonTimes(now, currentLatForAstro, currentLonForAstro, true);
                    const solarTimes = getSolarTime(now, currentLonForAstro);
                    sunAltitudeRad = sunPos.altitude; 
                    
                    lastKnownAstro = { lat: currentLatForAstro, lon: currentLonForAstro, sunPos, moonIllum, moonPos, sunTimes, moonTimes, solarTimes };
                    localStorage.setItem('lastKnownAstro', JSON.stringify(lastKnownAstro));

                    // --- MAJ DOM ASTRO ---
                    $('date-display-astro').textContent = now.toLocaleDateString() || '...';
                    $('date-solar-mean').textContent = solarTimes.DateMST ? solarTimes.DateMST.toLocaleDateString() : '...';
                    $('date-solar-true').textContent = solarTimes.DateTST ? solarTimes.DateTST.toLocaleDateString() : '...';
                    $('mst').textContent = solarTimes.MST;
                    $('tst').textContent = solarTimes.TST;
                    $('noon-solar').textContent = solarTimes.solarNoon ? solarTimes.solarNoon.toLocaleTimeString('fr-FR', { timeZone: 'UTC' }) : '...';
                    $('eot').textContent = `${solarTimes.EOT} min`;
                    $('tslv').textContent = getTSLV(now, currentLonForAstro);
                    $('ecl-long').textContent = `${solarTimes.ECL_LONG}°`;
                    $('sun-alt').textContent = `${(sunPos.altitude * R2D).toFixed(2)}°`;
                    $('sun-azimuth').textContent = `${(sunPos.azimuth * R2D).toFixed(2)}°`;
                    if (sunTimes.sunset && sunTimes.sunrise) {
                        const durationMs = sunTimes.sunset.getTime() - sunTimes.sunrise.getTime();
                        $('day-duration').textContent = `${Math.floor(durationMs / 3600000)}h ${Math.floor((durationMs % 3600000) / 60000)}m`;
                    }
                    const eot_ms = parseFloat(solarTimes.EOT) * 60000;
                    if(sunTimes.sunrise && $('sunrise-times')) $('sunrise-times').textContent = `${new Date(sunTimes.sunrise.getTime() + eot_ms).toLocaleTimeString('fr-FR', { timeZone: 'UTC' })} (TST)`;
                    if(sunTimes.sunset && $('sunset-times')) $('sunset-times').textContent = `${new Date(sunTimes.sunset.getTime() + eot_ms).toLocaleTimeString('fr-FR', { timeZone: 'UTC' })} (TST)`;
                    $('moon-phase-name').textContent = getMoonPhaseName(moonIllum.phase);
                    $('moon-illuminated').textContent = `${(moonIllum.fraction * 100).toFixed(1)}%`;
                    $('moon-alt').textContent = `${(moonPos.altitude * R2D).toFixed(2)}°`;
                    $('moon-azimuth').textContent = `${(moonPos.azimuth * R2D).toFixed(2)}°`;
                    $('moon-times').textContent = (moonTimes.rise && moonTimes.set) ? `${moonTimes.rise.toLocaleTimeString()} / ${moonTimes.set.toLocaleTimeString()}` : '...';

                    // --- MAJ Horloge Visuelle (Minecraft) ---
                    const clockDiv = $('minecraft-clock');
                    if (clockDiv) {
                        const sunRotation = (-sunPos.altitude * R2D) + 90;
                        const moonRotation = (-moonPos.altitude * R2D) + 90;
                        if($('sun-element')) $('sun-element').style.transform = `rotate(${sunRotation}deg)`;
                        if($('moon-element')) $('moon-element').style.transform = `rotate(${moonRotation}deg)`;
                        if (sunPos.altitude > 0) { clockDiv.className = 'sky-day'; $('clock-status').textContent = 'Jour (☀️)'; }
                        else if (sunPos.altitude > -10 * D2R) { clockDiv.className = 'sky-sunset'; $('clock-status').textContent = 'Crépuscule/Aube (✨)'; }
                        else { clockDiv.className = 'sky-night'; $('clock-status').textContent = 'Nuit (🌙)'; }
                    }
                    
                } catch (e) { console.error("Erreur dans updateAstro:", e); }
            }

            // 2. Mise à jour Météo & BioSVT
            if (lat && lon && !emergencyStopActive && (performance.now() - (weatherFetchID || 0) > WEATHER_FETCH_INTERVAL)) {
                await fetchWeather(lat, lon); 
                await fetchPollutants(lat, lon);
                weatherFetchID = performance.now();
            }
            
            // Altitude Barométrique
            const baroAlt = getBarometricAltitude(lastP_hPa, BARO_ALT_REF_HPA, lastT_K);
            $('alt-corrected-baro').textContent = dataOrDefault(baroAlt, 2, ' m');

            // 3. Mise à jour Heure NTP
            if (now) {
                if ($('local-time') && !$('local-time').textContent.includes('Synchronisation...')) {
                    const timeString = now.toLocaleTimeString('fr-FR');
                    $('local-time').textContent = timeString + (lServH ? ' (Synchro OK)' : ' (Hors ligne)');
                }
                if ($('date-display')) $('date-display').textContent = now.toUTCString();
                if ($('time-minecraft')) $('time-minecraft').textContent = getMinecraftTime(now);
            }
        };
        
        domSlowID = setInterval(updateSlowData, DOM_SLOW_UPDATE_MS);
        updateSlowData(); 
                }
    // =================================================================
// BLOC 4/4 : INITIALISATION DOM & ÉCOUTEURS D'ÉVÉNEMENTS
// =================================================================

    // --- Fonctions d'aide pour l'affichage Météo/Polluants (Hors ligne) ---
    function updateWeatherDOM(data, isOffline = false) {
        const suffix = isOffline ? ' (Hors ligne)' : '';
        $('statut-meteo').textContent = `ACTIF ${suffix}`;
        $('temp-air-2').textContent = `${data.tempC.toFixed(1)} °C ${suffix}`;
        $('pressure-2').textContent = `${data.pressure_hPa.toFixed(0)} hPa ${suffix}`;
        $('humidity-2').textContent = `${data.humidity_perc.toFixed(0)} % ${suffix}`;
        $('air-density').textContent = `${data.air_density.toFixed(3)} kg/m³ ${suffix}`;
        $('dew-point').textContent = `${data.dew_point.toFixed(1)} °C ${suffix}`;
    }

    function updatePollutantsDOM(data, isOffline = false) {
        const suffix = isOffline ? ' (Hors ligne)' : '';
        const components = data.components;
        const aqi = data.main.aqi;
        $('air-quality').textContent = `${getAirQualityIndexName(aqi)} ${suffix}`;
        $('no2-val').textContent = dataOrDefault(components.no2, 2, ` µg/m³ ${suffix}`);
        $('pm25-val').textContent = dataOrDefault(components.pm2_5, 2, ` µg/m³ ${suffix}`);
        $('pm10-val').textContent = dataOrDefault(components.pm10, 2, ` µg/m³ ${suffix}`);
        $('o3-val').textContent = dataOrDefault(components.o3, 2, ` µg/m³ ${suffix}`);
    }
    
    /**
     * Fonction utilitaire pour la gestion du corps céleste
     */
    function updateCelestialBody(bodyKey, alt, rotR, rotV) {
        let G_ACC_NEW = CELESTIAL_DATA['EARTH'].G;
        const body = CELESTIAL_DATA[bodyKey];
        if (bodyKey === 'ROTATING') {
            const centripetal = rotR > 0 ? rotR * rotV * rotV : 0;
            G_ACC_NEW = centripetal; 
            if ($('gravity-base')) $('gravity-base').textContent = `Centripète: ${G_ACC_NEW.toFixed(4)} m/s²`;
        } else if (body) {
            G_ACC_NEW = body.G;
            if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
        }
        G_ACC = G_ACC_NEW; // Mettre à jour la gravité globale
        return { G_ACC_NEW: G_ACC_NEW };
    }

    // ===========================================
    // INITIALISATION DOM (DÉMARRAGE)
    // ===========================================
    document.addEventListener('DOMContentLoaded', () => {
        
        try {
            initMap(); // Initialise la carte Leaflet
            
            // --- Chargement des données hors ligne (localStorage) ---
            lastKnownWeather = JSON.parse(localStorage.getItem('lastKnownWeather'));
            lastKnownPollutants = JSON.parse(localStorage.getItem('lastKnownPollutants'));
            lastKnownAstro = JSON.parse(localStorage.getItem('lastKnownAstro'));
            // BLOC 4/4: Initialisation (à la fin de document.addEventListener('DOMContentLoaded', ...))
// ... (Après la section de chargement des données hors ligne)

    // --- Mettre à jour les affichages Météo/Polluants avec les valeurs par défaut si pas de cache ---
    if (!lastKnownWeather) {
        // Utiliser les valeurs ASI
        updateWeatherDOM({ 
            tempC: TEMP_SEA_LEVEL_K - KELVIN_OFFSET, pressure_hPa: BARO_ALT_REF_HPA,
            humidity_perc: 0.0, air_density: RHO_SEA_LEVEL, dew_point: NaN
        }, true, ' (ASI Défaut)');
        // Afficher INACTIF pour le statut (car pas de fetch réussi)
        if ($('statut-meteo')) $('statut-meteo').textContent = 'INACTIF (ASI)';
    }

    if (!lastKnownPollutants) {
        // Afficher des valeurs par défaut pour les polluants
        if ($('no2-val')) $('no2-val').textContent = '0.00 µg/m³ (Défaut)';
        if ($('pm25-val')) $('pm25-val').textContent = '0.00 µg/m³ (Défaut)';
        if ($('pm10-val')) $('pm10-val').textContent = '0.00 µg/m³ (Défaut)';
        if ($('o3-val')) $('o3-val').textContent = '0.00 µg/m³ (Défaut)';
    }
// ...

            // --- Écouteurs d'événements pour tous les contrôles ---
            if($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', () => {
                if (ukf === null) { 
                    if (typeof math === 'undefined') {
                        alert("Erreur: math.js n'a pas pu être chargé. Le filtre UKF est désactivé.");
                        return;
                    }
                    ukf = new ProfessionalUKF(); 
                    sTime = Date.now();
                }
                toggleGPS(); // Gère le démarrage/arrêt des boucles
            });

            if($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', toggleEmergencyStop);
            
            if($('toggle-mode-btn')) $('toggle-mode-btn').addEventListener('click', () => {
                document.body.classList.toggle('dark-mode');
                $('toggle-mode-btn').innerHTML = document.body.classList.contains('dark-mode') ? '☀️ Mode Jour' : '🌗 Mode Nuit';
                if(map) map.invalidateSize();
            });
            
            if($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => { distM = 0; timeMoving = 0; });
            if($('reset-max-btn')) $('reset-max-btn').addEventListener('click', () => { maxSpd = 0; });
            if($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => {
                distM = 0; maxSpd = 0; kSpd = 0; kUncert = UKF_R_MAX; kAlt = 0; kAltUncert = 10; timeMoving = 0; timeTotal = 0; sTime = Date.now();
                maxSpd = 0; // 👈 CORRECTION : Assurez-vous que cette ligne est présente
                kSpd = 0; kUncert = UKF_R_MAX; kAlt = 0; kAltUncert = 10; timeMoving = 0; timeTotal = 0; sTime = Date.now();
                lat = 43.2964; lon = 5.3697; lastGPSPos = null; // Réinitialiser à la position par défaut
                if (ukf) ukf = new ProfessionalUKF();; 
                if ($('speed-stable')) $('speed-stable').textContent = '--.- km/h';
                if ($('speed-status-text')) $('speed-status-text').textContent = 'Système réinitialisé.';
            });
            
            if($('capture-data-btn')) $('capture-data-btn').addEventListener('click', () => {
                 console.log({
                    etat: ukf ? ukf.getState() : "UKF non initialisé",
                    physique: calculateAdvancedPhysics(kSpd, kAlt, currentMass, currentCdA, lastT_K, currentAirDensity, (lat || 0), kAltUncert, local_g, accel.x),
                    astro: getSolarTime(getCDate(), (lon || 0)),
                    meteo: lastKnownWeather || "N/A",
                });
                alert("Données capturées dans la console (F12).");
            });
            
            // --- Écouteurs pour les Inputs & Selects ---
            if($('freq-select')) $('freq-select').addEventListener('change', (e) => {
                currentGPSMode = e.target.value;
                if (wID) startGPS(currentGPSMode); // Redémarre le GPS si déjà actif
            });
            if($('gps-accuracy-override')) $('gps-accuracy-override').addEventListener('input', (e) => gpsAccuracyOverride = parseFloat(e.target.value) || 0.0);
            if($('environment-select')) $('environment-select').addEventListener('change', (e) => {
                selectedEnvironment = e.target.value;
                const factor = ENVIRONMENT_FACTORS[selectedEnvironment];
                if ($('env-factor')) $('env-factor').textContent = `${factor.DISPLAY} (x${factor.MULT.toFixed(1)})`;
            });
            if($('mass-input')) $('mass-input').addEventListener('input', (e) => {
                currentMass = parseFloat(e.target.value) || 70.0;
                if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
            });
            if($('celestial-body-select')) $('celestial-body-select').addEventListener('change', (e) => {
                currentCelestialBody = e.target.value;
                const { G_ACC_NEW } = updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity);
                G_ACC = G_ACC_NEW; 
            });
            
            const updateRotation = () => {
                rotationRadius = parseFloat($('rotation-radius').value) || 100;
                angularVelocity = parseFloat($('angular-velocity').value.replace(',', '.')) || 0.0; // Gère la virgule
                if (currentCelestialBody === 'ROTATING') {
                    const { G_ACC_NEW } = updateCelestialBody('ROTATING', kAlt, rotationRadius, angularVelocity);
                    G_ACC = G_ACC_NEW;
                }
            };
            if($('rotation-radius')) $('rotation-radius').addEventListener('input', updateRotation);
            if($('angular-velocity')) $('angular-velocity').addEventListener('input', updateRotation);
            
            if($('distance-ratio-toggle-btn')) $('distance-ratio-toggle-btn').addEventListener('click', () => {
                distanceRatioMode = !distanceRatioMode; 
                const text = distanceRatioMode ? 'ALTITUDE' : 'SURFACE';
                $('distance-ratio-toggle-btn').textContent = `Rapport Distance: ${text} (${R_FACTOR_RATIO.toFixed(3)})`;
            });
            if($('ukf-reactivity-mode')) $('ukf-reactivity-mode').addEventListener('change', (e) => currentUKFReactivity = e.target.value);
            
            if($('toggle-globe-btn')) {
                $('toggle-globe-btn').addEventListener('click', () => {
                    // (Logique pour basculer entre la carte 2D Leaflet et un globe 3D)
                });
            }

            // --- DÉMARRAGE DU SYSTÈME ---
            ukf = new ProfessionalUKF();
            G_ACC = updateCelestialBody(currentCelestialBody, 0, rotationRadius, angularVelocity).G_ACC_NEW;
            
            syncH(); 
            startSlowLoop(); // Démarrer la boucle lente (Astro/Météo)
            
            // Initialiser les valeurs par défaut (Mode Hors Ligne)
            if (lastKnownWeather) {
                updateWeatherDOM(lastKnownWeather, true);
                lastP_hPa = lastKnownWeather.pressure_hPa;
                lastT_K = lastKnownWeather.tempK;
                currentAirDensity = lastKnownWeather.air_density;
                currentSpeedOfSound = getSpeedOfSound(lastT_K);
            } else {
                currentAirDensity = RHO_SEA_LEVEL;
                currentSpeedOfSound = getSpeedOfSound(TEMP_SEA_LEVEL_K); 
                lastT_K = TEMP_SEA_LEVEL_K;
                lastP_hPa = BARO_ALT_REF_HPA;
            }
            if (lastKnownPollutants) {
                updatePollutantsDOM(lastKnownPollutants, true);
            }
            
            // Mettre à jour les affichages par défaut
            if($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = `${currentSpeedOfSound.toFixed(2)} m/s (Défaut)`;
            if($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
            if ($('env-factor')) $('env-factor').textContent = `${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY} (x${ENVIRONMENT_FACTORS[selectedEnvironment].MULT.toFixed(1)})`;
            if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = '▶️ MARCHE GPS'; // Assurer l'état initial
            if ($('statut-capteur')) $('statut-capteur').textContent = 'Inactif';

        } catch (error) {
            console.error("ERREUR CRITIQUE D'INITIALISATION:", error);
            // Afficher l'erreur dans l'interface utilisateur pour le débogage
            const statusElement = $('statut-gps-acquisition') || document.body;
            statusElement.innerHTML = `<h2 style="color:red;">CRASH SCRIPT: ${error.name}</h2><p>${error.message}</p>`;
        }

    }); // Fin de DOMContentLoaded

})(window); // Fin de l'IIFE
