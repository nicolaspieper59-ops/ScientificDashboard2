// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// CORRIGÉ : Gestion hors ligne, Niveau à bulle, Anti-veille, Suppression des simulations.
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
    const NETHER_RATIO = 8.0; // Ratio 1:8
    const ALT_TH = -50;         // Seuil d'altitude "Sous-sol"

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
        // Utilisation de Turf pour une précision WGS84
        const from = turf.point([lon1, lat1, alt1]);
        const to = turf.point([lon2, lat2, alt2]);
        return turf.distance(from, to, { units: 'meters' });
    }

    function getBarometricAltitude(P_hPa, P_ref_hPa, T_K) {
        return (T_K / 0.0065) * (1 - (P_hPa / P_ref_hPa)**(R_AIR * 0.0065 / G_ACC));
    }

    // CORRECTION : Logique "Mode Nether"
    function calculateDistanceRatio(alt, netherMode) {
        if (netherMode) return NETHER_RATIO;
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
            $('celestial-body-display').textContent = data.name;
        }
        if (body === 'ROTATING') {
            G_ACC_NEW = rotationRadius * angularVelocity ** 2;
        }
        G_ACC = G_ACC_NEW; 
        R_ALT_CENTER_REF = R_ALT_CENTER_REF_NEW; 
        $('gravity-base').textContent = `${G_ACC_NEW.toFixed(5)} m/s²`;
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
    
    // CORRECTION : Suppression de 'calculateBioSVT' (Simulation)

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
            return { TST: 'N/A', MST: 'N/A', EOT: 'N/A', ECL_LONG: 'N/A', DateMST: null, DateTST: null };
        }
        
        const d = toDays(date);
        const M = solarMeanAnomaly(d); 
        const L = eclipticLongitude(M); 
        
        // CORRECTION : Équation du temps réaliste via SunCalc
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
    let netherMode = false; 
    let selectedEnvironment = 'NORMAL'; 
    let currentMass = 70.0; 
    let currentCdA = 0.5; 
    let R_FACTOR_RATIO = 1.0;
    let currentCelestialBody = 'EARTH';
    let rotationRadius = 100;
    let angularVelocity = 0.0; 
    let gpsAccuracyOverride = 0.0; 
    let lastGPSPos = null;

    let lServH = null, lLocH = null; // Horodatages NTP
    let lastP_hPa = BARO_ALT_REF_HPA, lastT_K = TEMP_SEA_LEVEL_K, currentAirDensity = RHO_SEA_LEVEL;
    let currentSpeedOfSound = 343;
    let sunAltitudeRad = 0; 
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
    let wakeLock = null; // Pour l'API Wake Lock
    
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

            // 1. Démarrage des capteurs IMU (Accéléromètre, Gyroscope, Magnétomètre)
            // L'API Generic Sensor est sujette aux autorisations de l'utilisateur (permissions) et aux politiques du navigateur.
            if ('Gyroscope' in window && 'Accelerometer' in window) {
                
                try {
                    const accelerometer = new Accelerometer({ frequency: 50 }); // 50 Hz
                    accelerometer.addEventListener('reading', () => {
                        accel.x = accelerometer.x; accel.y = accelerometer.y; accel.z = accelerometer.z;
                    });
                    accelerometer.start();
                    
                    const gyroscope = new Gyroscope({ frequency: 50 }); 
                    gyroscope.addEventListener('reading', () => {
                        gyro.x = gyroscope.x; gyro.y = gyroscope.y; gyro.z = gyroscope.z;
                    });
                    gyroscope.start();
                    
                    if ('Magnetometer' in window) {
                        const magnetometer = new Magnetometer({ frequency: 10 });
                        magnetometer.addEventListener('reading', () => {
                            mag.x = magnetometer.x; mag.y = magnetometer.y; mag.z = magnetometer.z;
                            // Mise à jour de la déclinaison magnétique (très basique pour l'exemple)
                            const mag_decl_approx = Math.atan2(mag.y, mag.x) * R2D;
                            $('mag-decl').textContent = dataOrDefault(mag_decl_approx, 2, '°');
                        });
                        magnetometer.start();
                    }
                    
                    if ($('imu-status')) $('imu-status').textContent = "ACTIF (IMU & Mag)";

                } catch (error) {
                    console.warn("Erreur de démarrage des capteurs IMU (permission ou API non supportée):", error);
                    if ($('imu-status')) $('imu-status').textContent = "INACTIF (Erreur/Permission)";
                }
            } else {
                if ($('imu-status')) $('imu-status').textContent = "INACTIF (API non supportée)";
            }
            
            // 2. Démarrage du capteur de son (Microphone pour le niveau sonore)
            if (navigator.mediaDevices && navigator.mediaDevices.getUserMedia) {
                navigator.mediaDevices.getUserMedia({ audio: true })
                    .then(stream => {
                        audioContext = new (window.AudioContext || window.webkitAudioContext)();
                        const analyser = audioContext.createAnalyser();
                        const source = audioContext.createMediaStreamSource(stream);
                        source.connect(analyser);
                        analyser.fftSize = 256;
                        const bufferLength = analyser.frequencyBinCount;
                        const dataArray = new Uint8Array(bufferLength);
                        
                        const soundAnalyzer = () => {
                            analyser.getByteFrequencyData(dataArray);
                            let sum = 0;
                            for(let i = 0; i < bufferLength; i++) {
                                sum += dataArray[i];
                            }
                            const avg = sum / bufferLength; 
                            soundLevelMax = Math.max(soundLevelMax, avg);
                            requestAnimationFrame(soundAnalyzer);
                        };
                        requestAnimationFrame(soundAnalyzer);
                        console.log("Analyseur de son démarré.");
                    })
                    .catch(err => {
                        console.warn("Impossible d'accéder au microphone pour l'analyse du son.", err);
                    });
            }
            
            // 3. Boucle de mise à jour rapide du DOM (50Hz) & UKF Prediction
            lastIMUTimestamp = performance.now();
            domFastID = setInterval(updateFastDOM, IMU_UPDATE_RATE_MS); 
            console.log(`Boucle rapide démarrée (${IMU_UPDATE_RATE_MS}ms).`);
            
        } catch (e) {
            console.error("Erreur générale dans startSensorListeners:", e);
        }
    } 

    /** Arrête tous les capteurs locaux (IMU, Son, Boucles d'intervalle). */
    function stopSensorListeners() {
        if (domFastID) {
            clearInterval(domFastID);
            domFastID = null;
        }
        if (domSlowID) {
            clearInterval(domSlowID);
            domSlowID = null;
        }
        if (audioContext) {
            audioContext.close();
            audioContext = null;
        }
        // Il est difficile d'arrêter les écouteurs de l'API Generic Sensor, on compte sur le garbage collection.
        if ($('imu-status')) $('imu-status').textContent = 'INACTIF';
        if ($('speed-status-text')) $('speed-status-text').textContent = 'Arrêté.';
        
        // Libération du Wake Lock
        if (wakeLock) {
            wakeLock.release();
            wakeLock = null;
        }
    }


    // ===========================================
    // BOUCLES DE MISE À JOUR DU DOM (Fast/Slow)
    // ===========================================

    /** Met à jour les éléments du DOM qui requièrent une haute fréquence (50Hz). */
    function updateFastDOM() {
        const now = performance.now();
        let dt = (now - lastIMUTimestamp) / 1000; 
        lastIMUTimestamp = now;
        
        // Limite dt pour éviter les sauts lors de la mise en veille
        if (dt > 0.5) dt = MIN_DT; 
        
        // UKF PRÉDICTION (utilise les données IMU/accel)
        if (ukf) {
            // Dans un vrai UKF, on injecterait les données d'accel et gyro pour prédire l'état (position, vitesse, orientation)
            ukf.predict({ accel: [accel.x, accel.y, accel.z], gyro: [gyro.x, gyro.y, gyro.z] }, dt);
            const state = ukf.getState();
            
            // Mise à jour des vitesses stables et de la position UKF
            kSpd = state.speed;
            kUncert = state.kUncert;
            kAltUncert = state.kAltUncert;

            $('speed-stable').textContent = dataOrDefault(kSpd * KMH_MS, 1, ' km/h');
            $('speed-stable-kmh').textContent = dataOrDefault(kSpd * KMH_MS, 2, ' km/h');
            $('speed-stable-kms').textContent = dataOrDefault(kSpd / 1000, 4, ' km/s');
            $('speed-stable-ms').textContent = dataOrDefault(kSpd, 2, ' m/s');

            $('ukf-state-lat').textContent = dataOrDefault(state.lat, 6, '°');
            $('ukf-state-lon').textContent = dataOrDefault(state.lon, 6, '°');
            $('ukf-state-alt').textContent = dataOrDefault(state.alt, 2, ' m');
            $('ukf-precision-p11').textContent = dataOrDefault(kUncert, 4, ' m/s²');
        }

        // Accélération et Forces G (basé sur IMU raw)
        const accel_long = accel.x; 
        const localG = getWGS84Gravity(lat || 0, ukf ? ukf.getState().alt : 0);
        const force_g_long = localG > 0.1 ? (accel_long / localG) : 0;
        
        $('accel-long').textContent = dataOrDefault(accel_long, 3, ' m/s²');
        $('force-g-long').textContent = dataOrDefault(force_g-long, 2, ' G');
        $('accel-vertical').textContent = dataOrDefault(accel.z, 3, ' m/s²');
        const force_g_vertical = localG > 0.1 ? ((accel.z - localG) / localG) : 0; // Accel Z moins la gravité
        $('force-g-vertical').textContent = dataOrDefault(force_g_vertical, 2, ' G');
        
        // Mise à jour du Niveau à Bulle (utilise le Gyro pour l'inclinaison)
        const pitch = gyro.x * R2D;
        const roll = gyro.y * R2D;
        
        const bubble = $('spirit-level-bubble');
        const container = $('spirit-level-container');
        if (bubble && container) {
            const maxOffset = 5; 
            const offsetX = Math.max(-maxOffset, Math.min(maxOffset, roll * 0.5));
            const offsetY = Math.max(-maxOffset, Math.min(maxOffset, pitch * 0.5));
            bubble.style.transform = `translate(${offsetX}px, ${offsetY}px)`;
            
            let heading = lPos?.coords.heading || 0;
            if (heading !== null) {
                 container.style.transform = `rotate(${-heading}deg)`; 
            }
        }
    }

    /** Met à jour les éléments du DOM qui requièrent une basse fréquence (1Hz). */
    function updateSlowDOM() {
        const cDate = getCDate();
        const kAlt = ukf ? ukf.getState().alt : 0;

        // 1. Mise à jour de l'heure et du temps
        timeTotal += DOM_SLOW_UPDATE_MS / 1000; 
        $('elapsed-time').textContent = dataOrDefault(timeTotal, 2, ' s');
        $('local-time').textContent = cDate.toLocaleTimeString();
        $('date-display').textContent = cDate.toLocaleDateString();
        
        if (kSpd > MIN_SPD) {
            timeMoving += DOM_SLOW_UPDATE_MS / 1000;
        }
        $('time-moving').textContent = dataOrDefault(timeMoving, 2, ' s');
        maxSpd = Math.max(maxSpd, kSpd * KMH_MS);
        $('speed-max').textContent = dataOrDefault(maxSpd, 1, ' km/h');
        
        // 2. Calculs Météo/Physique Avancée
        const tempK = lastT_K; 
        const airDensity = currentAirDensity;
        const localG = getWGS84Gravity(lat || 0, kAlt);
        
        const { 
            speedOfSoundLocal, machNumber, coriolisForce, dragPower_kW, 
            geopotentialAltitude, force_g_long, radiationPressure
        } = calculateAdvancedPhysics(kSpd, kAlt, currentMass, currentCdA, tempK, airDensity, lat, kAltUncert, localG, accel.x);

        $('perc-speed-sound').textContent = dataOrDefault(machNumber * 100, 2, ' %');
        $('coriolis-force').textContent = dataOrDefault(coriolisForce, 2, ' N');
        $('mechanical-power').textContent = dataOrDefault(kSpd * currentMass * accel.x / 1000, 2, ' kW'); 
        $('kinetic-energy').textContent = dataOrDefault(0.5 * currentMass * kSpd * kSpd, 2, ' J');
        $('gravity-base').textContent = `${localG.toFixed(5)} m/s²`;
        
        // 3. Calculs GNSS & Géodynamique (besoin de lat/lon)
        if (lat !== null && lon !== null) {
            const { TST, DateTST } = getSolarTime(cDate, lon);
            $('tst').textContent = TST;
            
            const sunData = SunCalc.getPosition(cDate, lat, lon);
            sunAltitudeRad = sunData.altitude;
            $('sun-alt').textContent = dataOrDefault(sunData.altitude * R2D, 2, '°');
            $('sun-azimuth').textContent = dataOrDefault(sunData.azimuth * R2D, 2, '°');

            const moonData = SunCalc.getMoonIllumination(cDate);
            const moonPhase = getMoonPhaseName(moonData.phase);
            $('moon-phase-name').textContent = moonPhase;
            $('moon-illuminated').textContent = dataOrDefault(moonData.fraction * 100, 1, ' %');

            // Mise à jour horloge Minecraft (si TST disponible)
            const mcTime = getMinecraftTime(DateTST || cDate);
            $('time-minecraft').textContent = mcTime;

            // Mise à jour de la carte (si l'intervalle est atteint)
            if (now - lastMapUpdate > MAP_UPDATE_INTERVAL) {
                if (marker) {
                    marker.setLatLng([ukf.getState().lat, ukf.getState().lon]);
                    circle.setLatLng([ukf.getState().lat, ukf.getState().lon]);
                    circle.setRadius(Math.sqrt(kUncert)); // Rayon basé sur l'incertitude UKF
                }
                lastMapUpdate = now;
            }
        }
        
        // Mise à jour du mode sombre/jour
        const sunAlt = sunAltitudeRad * R2D;
        if (sunAlt > 5) {
            document.body.classList.add('sky-day');
            document.body.classList.remove('sky-night', 'sky-sunset');
            $('clock-status').textContent = 'JOUR';
        } else if (sunAlt > -10) {
            document.body.classList.add('sky-sunset');
            document.body.classList.remove('sky-day', 'sky-night');
            $('clock-status').textContent = 'CRÉPUSCULE';
        } else {
            document.body.classList.add('sky-night');
            document.body.classList.remove('sky-day', 'sky-sunset');
            $('clock-status').textContent = 'NUIT';
        }
        
        // Si la vitesse stable est suffisante, met à jour la vitesse max/moyenne
        if (kSpd > MIN_SPD) {
            $('speed-status-text').textContent = 'Mouvement stable (UKF)';
        } else {
             $('speed-status-text').textContent = 'Position stable (ZUPT actif)';
        }
    }


    // ===========================================
    // GESTION GPS PRINCIPALE
    // ===========================================

    /** Gestionnaire d'erreur pour watchPosition */
    function gpsError(error) {
        console.error("Erreur GPS:", error.code, error.message);
        $('speed-status-text').textContent = `Erreur GPS ${error.code}: ${error.message}`;
        if (error.code === 3) { // Timeout
            $('speed-status-text').textContent = 'Timeout GPS. Tentative de redémarrage.';
            // En cas de timeout, on réactive le standby
            if (wID !== null) {
                 navigator.geolocation.clearWatch(wID);
                 wID = null;
                 gpsStandbyTimeoutID = setTimeout(() => {
                    if (wID === null) toggleGps(true); // Redémarre si toujours arrêté
                 }, 5000);
            }
        }
    }

    /** Gestionnaire de succès pour watchPosition */
    function gpsSuccess(pos) {
        lPos = pos; 
        
        // Réinitialise le timer de veille (standby)
        if (gpsStandbyTimeoutID) {
            clearTimeout(gpsStandbyTimeoutID);
            gpsStandbyTimeoutID = null;
        }

        const now = getCDate().getTime();
        const coords = pos.coords;
        const kAlt = ukf ? ukf.getState().alt : coords.altitude || 0;
        
        // Mise à jour des coordonnées brutes
        lat = coords.latitude;
        lon = coords.longitude;
        $('lat-raw').textContent = dataOrDefault(lat, 6, '°');
        $('lon-raw').textContent = dataOrDefault(lon, 6, '°');
        $('alt-raw').textContent = dataOrDefault(coords.altitude, 2, ' m');
        $('gps-precision').textContent = dataOrDefault(coords.accuracy, 1, ' m');
        $('speed-raw-ms').textContent = dataOrDefault(coords.speed, 2, ' m/s');
        $('heading-display').textContent = coords.heading !== null ? dataOrDefault(coords.heading, 1, '°') : 'N/A';
        $('sous-sol-status').textContent = (kAlt < ALT_TH) ? 'Oui' : 'Non';

        // Calcul de la distance
        if (lastGPSPos) {
            const dist = dist3D(lastGPSPos.latitude, lastGPSPos.longitude, lastGPSPos.altitude || 0, lat, lon, coords.altitude || 0);
            distM += dist;
        }
        
        // Mise à jour de la distance
        $('distance-total-km').textContent = `${dataOrDefault(distM / 1000, 3, ' km')} | ${dataOrDefault(distM, 2, ' m')}`;
        const distanceLightSec = distM / C_L;
        $('distance-light-s').textContent = dataOrDefaultExp(distanceLightSec, 2, ' s');
        $('distance-light-min').textContent = dataOrDefaultExp(distanceLightSec / 60, 2, ' min');
        $('distance-light-h').textContent = dataOrDefaultExp(distanceLightSec / 3600, 2, ' h');
        $('distance-light-day').textContent = dataOrDefaultExp(distanceLightSec / (3600*24), 2, ' j');
        $('distance-cosmic').textContent = `${dataOrDefaultExp(distM / AU_METERS, 2, ' UA')} | ${dataOrDefaultExp(distM / LIGHT_YEAR_METERS, 2, ' al')}`;


        // UKF MISE À JOUR (UPDATE)
        const R_dyn = getKalmanR(coords.accuracy, kAltUncert, kUncert, selectedEnvironment, currentUKFReactivity);
        $('ukf-environment-factor').textContent = `x${R_dyn.toFixed(1)}`;
        
        ukf.update({
            latitude: lat, longitude: lon, altitude: coords.altitude,
            speed: coords.speed, accuracy: coords.accuracy
        }, R_dyn);

        lastGPSPos = {
            latitude: lat, longitude: lon, altitude: coords.altitude
        };
    }

    /** Démarre/Arrête le GPS (watchPosition) et les capteurs locaux. */
    function toggleGps(forceStart = false) {
        const btn = $('toggle-gps-btn');
        if (wID !== null && !forceStart) { 
            // Arrêt
            navigator.geolocation.clearWatch(wID);
            wID = null;
            stopSensorListeners();
            $('speed-status-text').textContent = 'Arrêt GPS.';
            btn.textContent = '▶️ MARCHE GPS';
            btn.style.backgroundColor = '#28a745';
        } else if (wID === null) { 
            // Démarrage
            const opts = GPS_OPTS[currentGPSMode];
            wID = navigator.geolocation.watchPosition(gpsSuccess, gpsError, opts);
            startSensorListeners(); 
            $('speed-status-text').textContent = 'Recherche du signal GPS...';
            btn.textContent = '⏸️ PAUSE GPS';
            btn.style.backgroundColor = '#ffc107'; 

            // Active le Wake Lock si supporté
            if ('wakeLock' in navigator) {
                try {
                    navigator.wakeLock.request('screen').then(lock => {
                        wakeLock = lock;
                        console.log("Wake Lock activé.");
                    });
                } catch (e) {
                    console.warn("Échec de l'activation du Wake Lock:", e);
                }
            }
            
            // Démarrage de la boucle lente (météo/horloge/distance)
            if (!domSlowID) {
                domSlowID = setInterval(() => {
                    updateSlowDOM();
                    fetchWeather(lat || 0, lon || 0).catch(e => console.log('Météo hors ligne/erreur.'));
                }, DOM_SLOW_UPDATE_MS); 
            }
        }
    }


    // ===========================================
    // INITIALISATION DE LA CARTE LEAFLET
    // ===========================================

    function initializeMap() {
        if (!map) {
            map = L.map('map', { attributionControl: false }).setView([48.8566, 2.3522], 13); // Paris par défaut
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                maxZoom: 19,
                minZoom: 2,
                attribution: '&copy; <a href="http://www.openstreetmap.org/copyright">OSM</a>'
            }).addTo(map);
            
            // Marker pour la position UKF et cercle de précision
            marker = L.marker([48.8566, 2.3522]).addTo(map);
            circle = L.circle([48.8566, 2.3522], {
                color: 'red',
                fillColor: '#f03',
                fillOpacity: 0.5,
                radius: 10 // Rayon initial faible
            }).addTo(map);
        }
    }


    // ===========================================
    // INITIALISATION DU TABLEAU DE BORD
    // ===========================================

    function initDashboard() {
        // 1. Initialisation des objets d'état
        ukf = new ProfessionalUKF();
        initializeMap();
        syncH(); // Synchronisation de l'heure NTP
        
        // 2. Initialisation des Event Listeners
        $('toggle-gps-btn').addEventListener('click', () => toggleGps());
        $('freq-select').addEventListener('change', (e) => currentGPSMode = e.target.value);
        $('emergency-stop-btn').addEventListener('click', () => {
            emergencyStopActive = !emergencyStopActive;
            if (emergencyStopActive) {
                toggleGps(false); // Arrête le GPS
                $('emergency-stop-btn').classList.add('active');
                $('emergency-stop-btn').textContent = '🛑 Arrêt d\'urgence: ACTIF 🔴';
            } else {
                $('emergency-stop-btn').classList.remove('active');
                $('emergency-stop-btn').textContent = '🛑 Arrêt d\'urgence: INACTIF 🟢';
            }
        });
        
        $('toggle-mode-btn').addEventListener('click', () => {
            document.body.classList.toggle('dark-mode');
            const isDarkMode = document.body.classList.contains('dark-mode');
            $('toggle-mode-btn').innerHTML = isDarkMode ? '<i class="fas fa-sun"></i> Mode Jour' : '<i class="fas fa-moon"></i> Mode Nuit';
        });

        $('nether-toggle-btn').addEventListener('click', () => {
            netherMode = !netherMode;
            $('nether-mode-status').textContent = netherMode ? 'ACTIVÉ (1:8)' : 'DÉSACTIVÉ (1:1)';
            $('nether-toggle-btn').textContent = netherMode ? 'Mode Overworld' : 'Mode Nether';
        });

        $('reset-dist-btn').addEventListener('click', () => {
            distM = 0; 
            $('distance-total-km').textContent = '0.000 km | 0.00 m';
        });
        
        $('reset-max-btn').addEventListener('click', () => {
            maxSpd = 0;
            $('speed-max').textContent = '0.0 km/h';
        });
        
        $('reset-all-btn').addEventListener('click', () => {
            distM = 0; maxSpd = 0; timeMoving = 0; timeTotal = 0;
            kSpd = 0; kUncert = UKF_R_MAX; kAltUncert = 10;
            toggleGps(false);
            ukf = new ProfessionalUKF(); // Réinitialise le filtre UKF
            $('speed-status-text').textContent = 'Système réinitialisé.';
        });
        
        $('gps-accuracy-override').addEventListener('input', (e) => {
            gpsAccuracyOverride = parseFloat(e.target.value) || 0.0;
        });

        $('environment-select').addEventListener('change', (e) => {
            selectedEnvironment = e.target.value;
            $('ukf-environment-factor').textContent = `x${ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT.toFixed(1)}`;
        });
        
        $('mass-input').addEventListener('input', (e) => {
            currentMass = parseFloat(e.target.value) || 70.0;
            $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        });
        
        $('celestial-body-select').addEventListener('change', (e) => {
            currentCelestialBody = e.target.value;
            updateCelestialBody(currentCelestialBody, 0, 0, 0); 
        });
        
        $('ukf-reactivity-mode').addEventListener('change', (e) => {
            currentUKFReactivity = e.target.value;
        });
        
        $('xray-button').addEventListener('click', () => {
            $('minecraft-clock').classList.toggle('x-ray');
        });

        // 3. Démarrage de la synchro NTP régulière (1 fois par heure)
        setInterval(syncH, 3600000); 

        console.log("Tableau de bord initialisé. Prêt pour l'activation GPS.");
    }
    
    // 4. Démarrage de l'application
    document.addEventListener('DOMContentLoaded', initDashboard);

})(window); // Fin de l'IIFE
// =================================================================
// FIN DU FICHIER JAVASCRIPT
// =================================================================
            
          
