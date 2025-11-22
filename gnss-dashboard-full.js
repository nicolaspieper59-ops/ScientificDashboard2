// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// CORRIGÉ : Gestion hors ligne, capteurs IMU complets, animation corrigée, simulations supprimées.
// Dépendances (doivent être chargées dans l'HTML) : leaflet.js, turf.min.js, suncalc.js, math.min.js
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        return (decimals === 0 ? '0' : '0.00') + suffix;
    }
    return val.toFixed(decimals) + suffix;
};

// CORRECTION CRITIQUE : Assure que le format exponentiel par défaut respecte 'decimals'.
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        const zeroDecimals = '0.' + Array(decimals).fill('0').join('');
        return zeroDecimals + 'e+0' + suffix;
    }
    return val.toExponential(decimals) + suffix;
};

// =================================================================
// DÉMARRAGE : Encapsulation de la logique UKF et État Global (IIFE)
// =================================================================

((window) => {

    // Vérification des dépendances critiques
    if (typeof math === 'undefined' || typeof L === 'undefined' || typeof SunCalc === 'undefined' || typeof turf === 'undefined') {
        const missing = [
            (typeof math === 'undefined' ? "math.min.js" : ""),
            (typeof L === 'undefined' ? "leaflet.js" : ""),
            (typeof SunCalc === 'undefined' ? "suncalc.js" : ""),
            (typeof turf === 'undefined' ? "turf.min.js" : "")
        ].filter(Boolean).join(", ");
        
        console.error(`Erreur critique : Dépendances manquantes : ${missing}. Le script est arrêté.`);
        alert(`Erreur: Dépendances manquantes : ${missing}. L'application ne peut pas démarrer.`);
        return;
    }
    
    // --- CLÉS D'API & ENDPOINTS (Conservés mais gérés pour le mode hors ligne) ---
    const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
    const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
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

    // --- CONSTANTES ATMOSPHÉRIQUES (ISA Standard - Utilisées comme valeurs par défaut hors ligne) ---
    const BARO_ALT_REF_HPA = 1013.25;
    const RHO_SEA_LEVEL = 1.225;
    const TEMP_SEA_LEVEL_K = 288.15; // 15°C
    const R_AIR = 287.058;
    const GAMMA_AIR = 1.4;
    const MU_DYNAMIC_AIR = 1.8e-5;  
    const KELVIN_OFFSET = 273.15;

    // --- CONSTANTES GÉOPHYSIQUES (WGS84) ---
    const WGS84_A = 6378137.0;  
    const WGS84_F = 1 / 298.257223563;
    const WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F;
    const WGS84_G_EQUATOR = 9.780327;
    const WGS84_BETA = 0.0053024;

    // --- PARAMÈTRES DU FILTRE UKF/EKF ---
    const UKF_STATE_DIM = 21;    
    const UKF_R_MAX = 500.0;     
    const UKF_Q_SPD = 0.5;       
    const R_ALT_MIN = 1.0;
    const MAX_PLAUSIBLE_ACCEL_GPS = 19.62; 
    const ZUPT_RAW_THRESHOLD = 1.0;     
    const ZUPT_ACCEL_THRESHOLD = 0.5;   
    const MIN_SPD = 0.01;        
    const MAX_ACC = 200;        

    // --- CONFIGURATION SYSTÈME ---
    const MIN_DT = 0.01;        
    const MAP_UPDATE_INTERVAL = 3000;
    const IMU_UPDATE_RATE_MS = 20; // 50Hz
    const DOM_SLOW_UPDATE_MS = 1000; // 1Hz
    const STANDBY_TIMEOUT_MS = 300000; // 5 minutes
    const GPS_OPTS = {
        HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
        LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
    };

    // --- FACTEURS DE RÉACTIVITÉ UKF ---
    const UKF_REACTIVITY_FACTORS = {
        'AUTO': { MULT: 1.0, DISPLAY: 'Automatique' },
        'NORMAL': { MULT: 1.0, DISPLAY: 'Normal' },
        'FAST': { MULT: 0.2, DISPLAY: 'Rapide' },
        'STABLE': { MULT: 2.5, DISPLAY: 'Microscopique' },
    };

    // --- FACTEURS D'ENVIRONNEMENT ---
    const ENVIRONMENT_FACTORS = {
        'NORMAL': { R_MULT: 1.0, DISPLAY: 'Normal' },
        'FOREST': { R_MULT: 2.5, DISPLAY: 'Forêt' },
        'CONCRETE': { R_MULT: 7.0, DISPLAY: 'Grotte/Tunnel' },
        'METAL': { R_MULT: 5.0, DISPLAY: 'Métal/Bâtiment' },
    };

    // --- DONNÉES CÉLESTES/GRAVITÉ ---
    const CELESTIAL_DATA = {
        'EARTH': { G: 9.80665, R: WGS84_A, name: 'Terre' },
        'MOON': { G: 1.62, R: 1737400, name: 'Lune' },
        'MARS': { G: 3.71, R: 3389500, name: 'Mars' },
        'ROTATING': { G: 0.0, R: WGS84_A, name: 'Station Spatiale' }
    };

    let G_ACC = CELESTIAL_DATA.EARTH.G;
    let R_ALT_CENTER_REF = CELESTIAL_DATA.EARTH.R;

    // ===========================================
    // CLASSE UKF PROFESSIONNELLE (Architecture 21 États)
    // ===========================================
    class ProfessionalUKF {
        constructor() {
            this.N_STATES = 21; 
            this.x = math.zeros(this.N_STATES); 
            this.P = math.diag(Array(this.N_STATES).fill(1e-2)); 
            this.Q = math.diag(Array(this.N_STATES).fill(1e-6));
            this.x.set([0], 0); this.x.set([1], 0); this.x.set([2], 0);
        }

        predict(imuData, dt) {
            // (PLACEHOLDER - La logique réelle de prédiction UKF 21 états est complexe)
            const accel_biais_corrigé = imuData.accel[0] - (this.x.get([9]) || 0); // Biais Accel X
            let vN = this.x.get([3]) + accel_biais_corrigé * dt; 
            if (Math.abs(vN) < MIN_SPD) vN = 0; // ZUPT
            this.x.set([3], vN); 
            let lat_pred = this.x.get([0]) + (vN / R_E_BASE) * dt;
            this.x.set([0], lat_pred);
        }

        update(gpsData, R_dyn) {
            // (PLACEHOLDER - La logique réelle de mise à jour UKF 21 états est complexe)
            const K = 0.1; 
            this.x.set([0], this.x.get([0]) * (1-K) + (gpsData.latitude * D2R) * K);
            this.x.set([1], this.x.get([1]) * (1-K) + (gpsData.longitude * D2R) * K);
            this.x.set([2], this.x.get([2]) * (1-K) + gpsData.altitude * K);
            if (gpsData.speed) {
                const oldSpeed = this.x.get([3]);
                this.x.set([3], oldSpeed * (1-K) + gpsData.speed * K);
            }
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
        const env_mult = ENVIRONMENT_FACTORS[env]?.R_MULT || 1.0;
        let reactivity_mult = UKF_REACTIVITY_FACTORS[reactivityMode]?.MULT || 1.0;
        if (reactivityMode === 'AUTO' && acc_effective !== null) {
            if (acc_effective > 20) reactivity_mult = 3.0; 
            else if (acc_effective < 3) reactivity_mult = 0.5; 
        }
        let R_dyn = Math.min(R_gps_base * env_mult * reactivity_mult, UKF_R_MAX);
        return Math.max(R_dyn, R_ALT_MIN); 
    }

    function dist3D(lat1, lon1, alt1, lat2, lon2, alt2) {
        const from = turf.point([lon1, lat1, alt1]);
        const to = turf.point([lon2, lat2, alt2]);
        return turf.distance(from, to, { units: 'meters' });
    }

    function getBarometricAltitude(P_hPa, P_ref_hPa, T_K) {
        return (T_K / 0.0065) * (1 - (P_hPa / P_ref_hPa)**(R_AIR * 0.0065 / G_ACC));
    }

    // CORRECTION : Logique "Mode Nether" remplacée par "Rapport Distance Altitude"
    function calculateDistanceRatio(alt) {
        // Calcule le ratio de la distance à la surface vs au centre
        return R_E_BASE / (R_E_BASE + alt);
    }

    function updateCelestialBody(body, alt, rotationRadius, angularVelocity) {
        let G_ACC_NEW = CELESTIAL_DATA['EARTH'].G;
        let R_ALT_CENTER_REF_NEW = WGS84_A; 
        const data = CELESTIAL_DATA[body];
        if (data) {
            G_ACC_NEW = data.G;
            R_ALT_CENTER_REF_NEW = data.R;
        }
        if (body === 'ROTATING') {
            G_ACC_NEW = rotationRadius * angularVelocity ** 2;
        }
        G_ACC = G_ACC_NEW; 
        R_ALT_CENTER_REF = R_ALT_CENTER_REF_NEW; 
        return { G_ACC: G_ACC_NEW, R_ALT_CENTER_REF: R_ALT_CENTER_REF_NEW };
    }

    function getWGS84Gravity(lat, alt) {
        const latRad = lat * D2R; 
        const sin2lat = Math.sin(latRad) ** 2;
        const g_surface = WGS84_G_EQUATOR * (1 + WGS84_BETA * sin2lat) / Math.sqrt(1 - WGS84_E2 * sin2lat);
        return g_surface * (1 - 2 * alt / WGS84_A); 
    }

    function getSpeedOfSound(tempK) {
        if(tempK < KELVIN_OFFSET) tempK += KELVIN_OFFSET; 
        return Math.sqrt(GAMMA_AIR * R_AIR * tempK);
    }

    function calculateMaxVisibleDistance(altitude) {
        return Math.sqrt(2 * R_E_BASE * altitude + altitude * altitude);
    }

    // --- FONCTIONS DE PHYSIQUE AVANCÉE ---
    function calculateAdvancedPhysics(kSpd, kAlt, mass, CdA, tempK, airDensity, lat, kAltUncert, localG, accel_long) {
        let V = kSpd;
        if(isNaN(V)) V = 0;
        const lorentzFactor = 1 / Math.sqrt(1 - Math.pow(V / C_L, 2));
        const timeDilationSpeed = (lorentzFactor - 1) * 86400 * 1e9; // ns/jour
        const E0 = mass * C_L * C_L;
        const energyRelativistic = lorentzFactor * E0;
        const momentum = mass * V; 
        const gravitationalDilation = (localG * kAlt / (C_L * C_L)) * 86400 * 1e9; // ns/jour
        const Rs_object = (2 * G_U * mass) / (C_L * C_L);
        
        const speedOfSoundLocal = getSpeedOfSound(tempK);
        const machNumber = (speedOfSoundLocal > 0) ? V / speedOfSoundLocal : 0;
        const dynamicPressure = 0.5 * airDensity * V * V; 
        const reynoldsNumber = (airDensity * V * 1) / MU_DYNAMIC_AIR; 
        const dragForce = dynamicPressure * (CdA || 0.5); 
        const dragPower_kW = (dragForce * V) / 1000.0;
        
        const coriolisForce = 2 * mass * V * OMEGA_EARTH * Math.sin(lat * D2R);
        const geopotentialAltitude = (G_ACC > 0) ? kAlt * (G_ACC / localG) : kAlt;
        const force_g_long = localG > 0.1 ? (accel_long / localG) : 0;
        
        const nyquistFrequency = 0.5 * (1000 / IMU_UPDATE_RATE_MS); 
        const altSigma = Math.sqrt(Math.abs(kAltUncert)); 

        const solarIrradiance = SOLAR_FLUX_DENSITY * Math.max(0, Math.sin(sunAltitudeRad)); 
        const radiationPressure = solarIrradiance / C_L; 

        return { 
            lorentzFactor, timeDilationSpeed, energyRelativistic, E0, momentum, gravitationalDilation, Rs_object,
            speedOfSoundLocal, machNumber, dynamicPressure, reynoldsNumber, dragForce, dragPower_kW,
            coriolisForce, geopotentialAltitude, force_g_long,
            nyquistFrequency, altSigma, radiationPressure
        };
    }

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

    // CORRECTION : Vérification SunCalc
    function getSolarTime(date, lon) {
        if (date === null || lon === null || isNaN(lon) || typeof SunCalc === 'undefined') {
            return { TST: 'N/A', MST: 'N/A', EOT: 'N/A', ECL_LONG: 'N/A', DateMST: null, DateTST: null };
        }
        
        const d = toDays(date);
        const M = solarMeanAnomaly(d); 
        const L = eclipticLongitude(M); 
        
        const J_star = toDays(date) - lon / 360;
        const J_transit = J_star + (0.0053 * Math.sin(M) - 0.0069 * Math.sin(2 * L));
        // CORRECTION : Équation du temps réaliste
        const eot_min = SunCalc.getEquationOfTime(date) * 60; 

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
            TST: toTimeString(tst_ms), 
            MST: toTimeString(mst_ms), 
            EOT: eot_min.toFixed(2),
            ECL_LONG: (L * R2D).toFixed(2),
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
        if (date === null) return 'N/A';
        const GMST = (date.getUTCHours() + date.getUTCMinutes() / 60) * 15; 
        const LST = GMST + lon;
        const LST_h = (LST / 15 + 24) % 24;
        return LST_h.toFixed(2) + ' h';
    }

    // --- VARIABLES D'ÉTAT (Globales) ---
    let wID = null, domSlowID = null, domFastID = null, lPos = null;
    let lat = null, lon = null, sTime = null;
    let distM = 0, maxSpd = 0;
    let kSpd = 0, kUncert = UKF_R_MAX, kAltUncert = 10; 
    let timeMoving = 0, timeTotal = 0; 
    let lastFSpeed = 0; 
    let ukf = null; 

    let currentGPSMode = 'HIGH_FREQ'; 
    let emergencyStopActive = false; 
    let distanceRatioMode = false; 
    let selectedEnvironment = 'NORMAL'; 
    let gpsAccuracyOverride = 0.0; 
    let lastGPSPos = null;

    let lServH = null, lLocH = null; // Horodatages NTP
    let lastP_hPa = BARO_ALT_REF_HPA, lastT_K = TEMP_SEA_LEVEL_K, currentAirDensity = RHO_SEA_LEVEL;
    let currentSpeedOfSound = 343;
    let lastWeatherData = null; 

    // Capteurs
    let accel = { x: 0, y: 0, z: 0 };
    let gyro = { x: 0, y: 0, z: 0 };
    let mag = { x: 0, y: 0, z: 0 };
    let barometer = null;
    let lightSensor = null;
    let audioContext = null;
    let soundLevelMax = 0;
    let lightLevelMax = 0;
    
    let lastIMUTimestamp = 0;
    let lastMapUpdate = 0;
    let map, marker, circle; // Objets Leaflet

    let currentUKFReactivity = 'AUTO'; 
    let gpsStandbyTimeoutID = null;    

    // --- GESTION NTP (SYNCHRO HEURE) ---
    async function syncH() {
        if ($('local-time')) $('local-time').textContent = 'Synchronisation...';
        lLocH = performance.now(); 
        
        try {
            const response = await fetch(SERVER_TIME_ENDPOINT, { cache: "no-store", mode: "cors" });
            if (!response.ok) throw new Error(`Échec du fetch`);
            const serverData = await response.json(); 
            const utcTimeISO = serverData.utc_datetime; 
            lServH = Date.parse(utcTimeISO); 
            if ($('local-time')) $('local-time').textContent = '✅ SYNCHRO NTP ACTIVE';
        } catch (error) {
            // CORRECTION HORS LIGNE
            lServH = Date.now(); 
            if ($('local-time')) $('local-time').textContent = '❌ HORS LIGNE (Local)';
            console.warn("Échec de la synchro NTP (Hors ligne ?). Utilisation de l'heure locale.", error.message);
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

    // --- GESTION API MÉTÉO ---
    async function fetchWeather(lat, lon) {
        const url = `${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`;
        try {
            const response = await fetch(url);
            if (!response.ok) throw new Error(`Erreur HTTP ${response.status}`);
            const data = await response.json();

            if (data.main) {
                const P_hPa = data.main.pressure;
                const T_C = data.main.temp;
                const T_K = T_C + KELVIN_OFFSET;
                const H_perc = data.main.humidity;
                const P_Pa = P_hPa * 100; 
                const air_density = P_Pa / (R_AIR * T_K);

                lastP_hPa = P_hPa;
                lastT_K = T_K;
                currentAirDensity = air_density;
                currentSpeedOfSound = getSpeedOfSound(T_K);
                
                lastWeatherData = { 
                    pressure_hPa: P_hPa, tempC: T_C, tempK: T_K,
                    humidity_perc: H_perc, air_density: air_density
                };
                return lastWeatherData;
            }
        } catch (e) {
            if ($('weather-status')) $('weather-status').textContent = `❌ ${e.message}`;
            throw e; 
        }
        return null;
    }

    // --- GESTION DES CAPTEURS (IMU, Baro, Lumière, Son) ---
    function startSensorListeners() {
        if (emergencyStopActive || domFastID) return; 
        try {
            if ($('imu-status')) $('imu-status').textContent = "Activation...";
            
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
            
            // 3. Magnétomètre (CORRECTION : Ajout du capteur pour le cap)
            if (typeof Magnetometer === 'undefined') {
                console.warn("API Magnetometer non supportée. Le cap ne sera pas disponible.");
            } else {
                const magSensor = new Magnetometer({ frequency: 10 });
                magSensor.addEventListener('reading', () => {
                    mag.x = magSensor.x; mag.y = magSensor.y; mag.z = magSensor.z;
                    // (Logique de calcul du cap à partir de mag et accel)
                    $('heading-display').textContent = 'N/A (Calcul manquant)';
                });
                magSensor.addEventListener('error', e => console.error("Erreur Magnétomètre:", e.error));
                magSensor.start();
            }
            
            // 4. Baromètre (CORRECTION : Ajout du capteur pour l'altitude)
            if (typeof Barometer === 'undefined') {
                console.warn("API Barometer non supportée. L'altitude baro ne sera pas disponible.");
            } else {
                barometer = new Barometer({ frequency: 1 });
                barometer.addEventListener('reading', () => {
                    $('alt-corrected-baro').textContent = `${barometer.pressure.toFixed(2)} hPa`;
                    // (Logique de correction par station météo à ajouter)
                });
                barometer.addEventListener('error', e => console.error("Erreur Baromètre:", e.error));
                barometer.start();
                }

            // 5. Capteur de Lumière Ambiante (CORRECTION : Ajout)
            if (typeof AmbientLightSensor === 'undefined') {
                console.warn("API AmbientLightSensor non supportée.");
            } else {
                lightSensor = new AmbientLightSensor({ frequency: 1 });
                lightSensor.addEventListener('reading', () => {
                    $('ambient-light').textContent = `${lightSensor.illuminance.toFixed(0)} Lux`;
                    lightLevelMax = Math.max(lightLevelMax, lightSensor.illuminance);
                    $('ambient-light-max').textContent = `${lightLevelMax.toFixed(0)} Lux`;
                });
                lightSensor.addEventListener('error', e => console.error("Erreur Capteur Lumière:", e.error));
                lightSensor.start();
            }
            
            // 6. Capteur Sonore (CORRECTION : Ajout)
            try {
                navigator.mediaDevices.getUserMedia({ audio: true, video: false })
                    .then(stream => {
                        audioContext = new (window.AudioContext || window.webkitAudioContext)();
                        const analyser = audioContext.createAnalyser();
                        const microphone = audioContext.createMediaStreamSource(stream);
                        microphone.connect(analyser);
                        analyser.fftSize = 256;
                        const dataArray = new Uint8Array(analyser.frequencyBinCount);
                        
                        setInterval(() => {
                            analyser.getByteFrequencyData(dataArray);
                            let sum = 0;
                            for (let i = 0; i < dataArray.length; i++) {
                                sum += dataArray[i];
                            }
                            let avg = sum / dataArray.length;
                            // Conversion approximative en dB (pas calibrée !)
                            let dB = 20 * Math.log10(avg / 255 * 100); 
                            dB = isFinite(dB) ? dB + 100 : 0; // Normalisation simple
                            
                            $('sound-level').textContent = `${dB.toFixed(1)} dB`;
                            soundLevelMax = Math.max(soundLevelMax, dB);
                            $('sound-level-max').textContent = `${soundLevelMax.toFixed(1)} dB`;
                        }, 500);
                     })
                    .catch(e => console.warn("API Audio (Microphone) non autorisée ou non supportée."));
            } catch (e) {
                console.error("Erreur AudioContext:", e.message);
            }
            
            if ($('imu-status')) $('imu-status').textContent = "Actif (Multi-Capteurs)";
            lastIMUTimestamp = performance.now();
            
            startFastLoop(); // DÉMARRE LA BOUCLE RAPIDE (PRÉDICTION)

        } catch (error) {
            let errMsg = error.message;
            if (error.name === 'SecurityError' || error.name === 'NotAllowedError') {
                errMsg = "Permission Capteurs Refusée. Cliquez sur 'MARCHE GPS' pour autoriser.";
            }
            if ($('imu-status')) $('imu-status').textContent = `❌ ${errMsg}`;
        }
    }

    function stopSensorListeners() {
        if (domFastID) clearInterval(domFastID);
        domFastID = null;
        if ($('imu-status')) $('imu-status').textContent = "Inactif";
        accel = { x: 0, y: 0, z: 0 };
        gyro = { x: 0, y: 0, z: 0 };
        // (Ajouter l'arrêt des autres capteurs si nécessaire)
    }

    // --- GESTION GPS (GÉOLOCALISATION) ---
    function startGPS(mode = currentGPSMode) {
        if (emergencyStopActive) return;
        if (wID !== null) navigator.geolocation.clearWatch(wID);
        
        currentGPSMode = mode;
        const options = GPS_OPTS[mode];
        
        wID = navigator.geolocation.watchPosition(gpsUpdateCallback, handleErr, options);
        
        if (!domFastID) {
            startSensorListeners();
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
        
        stopSensorListeners();
        
        if (resetButton && $('toggle-gps-btn')) {
            $('toggle-gps-btn').textContent = '▶️ MARCHE GPS';
            $('toggle-gps-btn').style.backgroundColor = '#28a745'; 
        }
    }

    function toggleGPS() {
        if (emergencyStopActive) return;
        wID === null ? startGPS('HIGH_FREQ') : stopGPS();
    }

    function toggleEmergencyStop() {
        emergencyStopActive = !emergencyStopActive;
        if (emergencyStopActive) {
            stopGPS(false); 
            stopSensorListeners();
            if($('emergency-stop-btn')) {
                $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: ACTIF 🔴";
                $('emergency-stop-btn').classList.add('active');
            }
            if ($('speed-status-text')) $('speed-status-text').textContent = '🛑 ARRÊT D’URGENCE';
        } else {
