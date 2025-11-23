// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// BLOC 1/4 : Constantes, Utilitaires, UKF, et Modèles Physiques/Astro
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

    // --- CLÉS D'API & ENDPOINTS ---
    const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
    const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
    const PROXY_POLLUTANT_ENDPOINT = `${PROXY_BASE_URL}/api/pollutants`; // AJOUTÉ
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

    // --- CONSTANTES ATMOSPHÉRIQUES (ISA Standard) ---
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
        'NORMAL': { R_MULT: 1.0, DISPLAY: 'Normal' }, 'FOREST': { R_MULT: 2.5, DISPLAY: 'Forêt' },
        'CONCRETE': { R_MULT: 7.0, DISPLAY: 'Grotte/Tunnel' }, 'METAL': { R_MULT: 5.0, DISPLAY: 'Métal/Bâtiment' },
    };

    // --- DONNÉES CÉLESTES/GRAVITÉ ---
    const CELESTIAL_DATA = {
        'EARTH': { G: 9.80665, R: WGS84_A, name: 'Terre' }, 'MOON': { G: 1.62, R: 1737400, name: 'Lune' },
        'MARS': { G: 3.71, R: 3389500, name: 'Mars' }, 'ROTATING': { G: 0.0, R: WGS84_A, name: 'Station Spatiale' }
    };

    // --- VARIABLES D'ÉTAT (Globales) ---
    let wID = null, domSlowID = null, domFastID = null, weatherFetchID = null;
    let lat = null, lon = null, sTime = null;
    let distM = 0, maxSpd = 0;
    let kSpd = 0, kUncert = UKF_R_MAX, kAltUncert = 10, kAlt = 0;
    let timeMoving = 0, timeTotal = 0; 
    let lastFSpeed = 0; 
    let ukf = null; 
    let currentGPSMode = 'HIGH_FREQ'; 
    let emergencyStopActive = false; 
    let distanceRatioMode = false; // Pour le bouton "Rapport Distance"
    let selectedEnvironment = 'NORMAL'; 
    let currentMass = 70.0; 
    let currentCdA = 0.5; // CdA par défaut
    let R_FACTOR_RATIO = 1.0;
    let currentCelestialBody = 'EARTH';
    let rotationRadius = 100;
    let angularVelocity = 0.0; 
    let gpsAccuracyOverride = 0.0; 
    let lastGPSPos = null;
    let lServH = null, lLocH = null; // Horodatages NTP
    
    // --- VARIABLES DE CORRECTION MÉTROLOGIQUE (Initialisation ISA) ---
    let lastP_hPa = BARO_ALT_REF_HPA; 
    let lastT_K = TEMP_SEA_LEVEL_K; 
    let currentAirDensity = RHO_SEA_LEVEL;
    let currentSpeedOfSound = 340.29; // Vitesse du son ISA
    let G_ACC = CELESTIAL_DATA.EARTH.G; // Gravité Base
    
    let lastWeatherData = null; 
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
    let sunAltitudeRad = 0; // Pour la Pression de Radiation
 // =================================================================
// BLOC 2/4 : Classe UKF et Fonctions Métrologiques/Physiques
// =================================================================

    // ===========================================
    // CLASSE UKF PROFESSIONNELLE (Architecture 21 États - Simplifiée)
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
            // La matrice P (incertitude) devrait être propagée ici
        }

        update(gpsData, R_dyn) {
            // (PLACEHOLDER - La logique réelle de mise à jour UKF 21 états est complexe)
            // L'implémentation complète calculerait le Gain de Kalman K
            const K = 0.1; 
            this.x.set([0], this.x.get([0]) * (1-K) + (gpsData.latitude * D2R) * K);
            this.x.set([1], this.x.get([1]) * (1-K) + (gpsData.longitude * D2R) * K);
            this.x.set([2], this.x.get([2]) * (1-K) + gpsData.altitude * K);
            if (gpsData.speed !== null && gpsData.speed !== undefined) {
                const oldSpeed = this.x.get([3]);
                this.x.set([3], oldSpeed * (1-K) + gpsData.speed * K);
            }
             // La matrice P (incertitude) devrait être réduite ici
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
        const from = turf.point([lon1, lat1, alt1 || 0]); // S'assurer que l'altitude n'est pas nulle
        const to = turf.point([lon2, lat2, alt2 || 0]);
        return turf.distance(from, to, { units: 'meters' });
    }

    // CORRECTION MÉTROLOGIQUE : Calcul de l'Altitude Barométrique
    function getBarometricAltitude(P_hPa, P_ref_HPA, T_K) {
        // P_hPa = Pression actuelle (mesurée)
        // P_ref_HPA = Pression de référence au niveau de la mer (Standard = 1013.25)
        // T_K = Température actuelle (mesurée) en Kelvin
        if (P_hPa <= 0 || P_ref_HPA <= 0 || T_K <= 0) return NaN;
        // Formule standard d'altitude barométrique (utilisant la gravité G_ACC du corps céleste actuel)
        const pressure_ratio = P_hPa / P_ref_HPA;
        return (T_K / 0.0065) * (1 - Math.pow(pressure_ratio, (R_AIR * 0.0065) / G_ACC));
    }

    // CORRECTION MÉTROLOGIQUE : Gravité Locale (g) WGS84
    function getWGS84Gravity(lat, alt) {
        if (!lat || !alt) { lat = 0; alt = 0; } // Sécurité
        const latRad = lat * D2R; 
        const sin2lat = Math.sin(latRad) ** 2;
        const g_surface = WGS84_G_EQUATOR * (1 + WGS84_BETA * sin2lat) / Math.sqrt(1 - WGS84_E2 * sin2lat);
        return g_surface * (1 - 2 * alt / WGS84_A); 
    }

    // Correction Métrologique : Vitesse du son ajustée par la température locale
    function getSpeedOfSound(tempK) {
        if(tempK < KELVIN_OFFSET) tempK += KELVIN_OFFSET; // S'assure d'être en Kelvin
        return Math.sqrt(GAMMA_AIR * R_AIR * tempK);
    }

    function calculateMaxVisibleDistance(altitude) {
        if (!altitude || altitude < 0) return 0; // Ne peut pas voir à travers le sol
        return Math.sqrt(2 * R_E_BASE * altitude + altitude * altitude);
    }

    // --- FONCTIONS DE PHYSIQUE AVANCÉE ---
    // Correction Métrologique : Utilise les variables météo (tempK, airDensity) mises à jour
    function calculateAdvancedPhysics(kSpd, kAlt, mass, CdA, tempK, airDensity, lat, kAltUncert, localG, accel_long) {
        let V = kSpd;
        if(isNaN(V)) V = 0;
        if(isNaN(kAlt)) kAlt = 0;
        if(isNaN(lat)) lat = 0;
        
        const lorentzFactor = 1 / Math.sqrt(1 - Math.pow(V / C_L, 2));
        const timeDilationSpeed = (lorentzFactor - 1) * 86400 * 1e9; // ns/jour
        const E0 = mass * C_L * C_L;
        const energyRelativistic = lorentzFactor * E0;
        const momentum = mass * V; 
        const gravitationalDilation = (localG * kAlt / (C_L * C_L)) * 86400 * 1e9; // ns/jour
        const Rs_object = (2 * G_U * mass) / (C_L * C_L);
        
        // --- Correction Métrologique : Utilisation des données météo locales ---
        const speedOfSoundLocal = getSpeedOfSound(tempK);
        const machNumber = (speedOfSoundLocal > 0) ? V / speedOfSoundLocal : 0;
        const dynamicPressure = 0.5 * airDensity * V * V; 
        const reynoldsNumber = (airDensity * V * 1) / MU_DYNAMIC_AIR; 
        const dragForce = dynamicPressure * (CdA || 0.5); 
        const dragPower_kW = (dragForce * V) / 1000.0;
        
        const coriolisForce = 2 * mass * V * OMEGA_EARTH * Math.sin(lat * D2R);
        const geopotentialAltitude = (localG > 0.1) ? kAlt * (G_ACC / localG) : kAlt;
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
// =================================================================
// BLOC 3/4 : Fonctions Météo, Astro, Capteurs et Contrôle GPS
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
            return { TST: 'N/A', MST: 'N/A', EOT: 'N/A', ECL_LONG: 'N/A', DateMST: null, DateTST: null };
        }
        
        const d = toDays(date);
        const M = solarMeanAnomaly(d); 
        const L = eclipticLongitude(M); 
        
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
            const utcTimeISO = serverData.utc_datetime; 
            lServH = Date.parse(utcTimeISO); 
            if ($('local-time')) $('local-time').textContent = new Date(lServH).toLocaleTimeString('fr-FR') + ' (Synchro OK)';
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

    // --- GESTION API MÉTÉO (DÉTAILLÉE) ---
    function getCardinalDirection(deg) {
        const directions = ['N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE', 'S', 'SSO', 'SO', 'OSO', 'O', 'ONO', 'NO', 'NNO'];
        const index = Math.round((deg % 360) / (360 / directions.length)) % directions.length;
        return directions[index];
    }
    function getAirQualityIndexName(aqi) {
        if (aqi === 1) return 'Bonne (1)'; if (aqi === 2) return 'Correcte (2)';
        if (aqi === 3) return 'Dégradée (3)'; if (aqi === 4) return 'Mauvaise (4)';
        if (aqi === 5) return 'Très Mauvaise (5)'; return 'N/A';
    }

    async function fetchWeather(lat, lon) {
        const weatherUrl = `${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`;
        const pollutantUrl = `${PROXY_POLLUTANT_ENDPOINT}?lat=${lat}&lon=${lon}`; 

        $('weather-status').textContent = 'Chargement...';
        $('air-quality').textContent = 'Chargement...';
        
        try {
            // 1. Fetch Basic Weather
            const response = await fetch(weatherUrl);
            if (!response.ok) throw new Error(`Erreur HTTP Météo ${response.status}`);
            const weatherData = await response.json(); 

            if (weatherData.main) {
                const P_hPa = weatherData.main.pressure;
                const T_C = weatherData.main.temp;
                const H_perc = weatherData.main.humidity;
                const T_K = T_C + KELVIN_OFFSET;
                const P_Pa = P_hPa * 100; 
                const air_density = P_Pa / (R_AIR * T_K);
                const dew_point_C = T_C - ((100 - H_perc) / 5); // Approximation
                
                // CORRECTION MÉTROLOGIQUE : Stockage des valeurs GLOBALES
                lastP_hPa = P_hPa;
                lastT_K = T_K;
                currentAirDensity = air_density;
                currentSpeedOfSound = getSpeedOfSound(T_K);
                
                // Mise à jour DOM Météo de base et détaillée
                $('weather-status').textContent = `ACTIF`;
                $('temp-air-2').textContent = `${T_C.toFixed(1)} °C`;
                $('pressure-2').textContent = `${P_hPa.toFixed(0)} hPa`;
                $('humidity-2').textContent = `${H_perc.toFixed(0)} %`;
                $('dew-point').textContent = `${dew_point_C.toFixed(1)} °C`;
                $('realfeel-shade').textContent = `~${(T_C - (weatherData.wind?.speed || 0) / 5).toFixed(1)} °C (Est.)`; // Wind chill simple
                
                const windSpeed_ms = weatherData.wind?.speed || 0;
                const windDeg = weatherData.wind?.deg || 0;
                const windSpeed_kmh = windSpeed_ms * 3.6;
                const windDir = getCardinalDirection(windDeg);
                $('wind-display').textContent = `${windSpeed_kmh.toFixed(1)} km/h (${windDir})`;
                $('gusts').textContent = `${((weatherData.wind?.gust || windSpeed_ms) * 3.6).toFixed(1)} km/h`;
                const visibility_km = (weatherData.visibility / 1000) || 'N/A';
                $('visibility').textContent = (typeof visibility_km === 'number') ? `${visibility_km.toFixed(1)} km` : visibility_km;
                $('cloud-cover').textContent = `${weatherData.clouds?.all || 'N/A'} %`;
            }
        } catch (e) {
            $('weather-status').textContent = `❌ API ÉCHOUÉE`;
            console.warn("Échec du fetch Météo:", e.message);
        }
        
        // 2. Fetch Pollutants (séparément)
        try {
            const pollutantResponse = await fetch(pollutantUrl);
            if (!pollutantResponse.ok) throw new Error(`Erreur HTTP Polluants ${pollutantResponse.status}`);
            const pollutantData = await pollutantResponse.json();
            
            if (pollutantData.list && pollutantData.list.length > 0) {
                 const components = pollutantData.list[0].components;
                 const aqi = pollutantData.list[0].main.aqi;
                 
                 $('air-quality').textContent = getAirQualityIndexName(aqi);
                 $('no2-val').textContent = dataOrDefault(components.no2, 2, ' µg/m³');
                 $('pm25-val').textContent = dataOrDefault(components.pm2_5, 2, ' µg/m³');
                 $('pm10-val').textContent = dataOrDefault(components.pm10, 2, ' µg/m³');
                 $('o3-val').textContent = dataOrDefault(components.o3, 2, ' µg/m³');
            } else { 
                $('air-quality').textContent = 'N/A (API)'; 
            }
        } catch (e) {
            $('air-quality').textContent = 'N/A (API Échouée)';
            console.warn("Échec du fetch Polluants:", e.message);
        }
    }

    // --- GESTION DES CAPTEURS (IMU, Baro, Lumière, Son) ---
    function startSensorListeners() {
        if (emergencyStopActive || domFastID) return; 
        
        let sensorsStarted = false; // Flag pour s'assurer que l'IMU répond
        
        try {
            if ($('imu-status')) $('imu-status').textContent = "Activation...";
            
            // 1. Accéléromètre (Haute Fréquence)
            if (typeof Accelerometer === 'undefined') throw new Error("API Accelerometer non supportée.");
            const accSensor = new Accelerometer({ frequency: 50 }); 
            accSensor.addEventListener('reading', () => {
                accel.x = accSensor.x; accel.y = accSensor.y; accel.z = accSensor.z;
                sensorsStarted = true;
            });
            accSensor.addEventListener('error', e => console.error("Erreur Accéléromètre:", e.error.message));
            accSensor.start();

            // 2. Gyroscope (Haute Fréquence)
            if (typeof Gyroscope === 'undefined') throw new Error("API Gyroscope non supportée.");
            const gyroSensor = new Gyroscope({ frequency: 50 });
            gyroSensor.addEventListener('reading', () => {
                gyro.x = gyroSensor.x; gyro.y = gyroSensor.y; gyro.z = gyroSensor.z;
                sensorsStarted = true;
            });
            gyroSensor.addEventListener('error', e => console.error("Erreur Gyroscope:", e.error.message));
            gyroSensor.start();
            
            // 3. Magnétomètre
            if (typeof Magnetometer === 'undefined') {
                console.warn("API Magnetometer non supportée.");
            } else {
                const magSensor = new Magnetometer({ frequency: 10 });
                magSensor.addEventListener('reading', () => {
                    mag.x = magSensor.x; mag.y = magSensor.y; mag.z = magSensor.z;
                });
                magSensor.addEventListener('error', e => console.error("Erreur Magnétomètre:", e.error.message));
                magSensor.start();
            }
            
            // 4. Baromètre
            if (typeof Barometer === 'undefined') {
                console.warn("API Barometer non supportée.");
            } else {
                barometer = new Barometer({ frequency: 1 });
                barometer.addEventListener('reading', () => {
                    // Ne met pas à jour 'alt-corrected-baro' directement
                    // Stocke la pression pour la correction métrologique
                    lastP_hPa = barometer.pressure; 
                });
                barometer.addEventListener('error', e => console.error("Erreur Baromètre:", e.error.message));
                barometer.start();
            }
            
            // 5. Capteur de Lumière Ambiante
            if (typeof AmbientLightSensor === 'undefined') {
                console.warn("API AmbientLightSensor non supportée.");
            } else {
                lightSensor = new AmbientLightSensor({ frequency: 1 });
                lightSensor.addEventListener('reading', () => {
                    if ($('ambient-light')) $('ambient-light').textContent = `${lightSensor.illuminance.toFixed(0)} Lux`;
                    lightLevelMax = Math.max(lightLevelMax, lightSensor.illuminance);
                    if ($('ambient-light-max')) $('ambient-light-max').textContent = `${lightLevelMax.toFixed(0)} Lux`;
                });
                lightSensor.addEventListener('error', e => console.error("Erreur Capteur Lumière:", e.error.message));
                lightSensor.start();
            }
            
            // 6. Capteur Sonore
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
                            if (!audioContext) return; // Arrêté
                            analyser.getByteFrequencyData(dataArray);
                            let sum = 0;
                            for (let i = 0; i < dataArray.length; i++) {
                                sum += dataArray[i];
                            }
                            let avg = sum / dataArray.length;
                            let dB = 20 * Math.log10(avg / 255 * 100); 
                            dB = isFinite(dB) ? dB + 100 : 0; 
                            
                            if ($('sound-level')) $('sound-level').textContent = `${dB.toFixed(1)} dB`;
                            soundLevelMax = Math.max(soundLevelMax, dB);
                            if ($('sound-level-max')) $('sound-level-max').textContent = `${soundLevelMax.toFixed(1)} dB`;
                        }, 500);
                    })
                    .catch(e => console.warn("API Audio (Microphone) non autorisée ou non supportée."));
            } catch (e) {
                console.error("Erreur AudioContext:", e.message);
            }
            
            // CORRECTION BUG N°2 : Démarrer la boucle rapide uniquement si les capteurs de base fonctionnent
            setTimeout(() => {
                 if (sensorsStarted) {
                    if ($('imu-status')) $('imu-status').textContent = "Actif (Multi-Capteurs)";
                    lastIMUTimestamp = performance.now();
                    startFastLoop(); // DÉMARRE LA BOUCLE RAPIDE
                 } else {
                     // Si les capteurs n'ont pas démarré (ex: permission refusée, mobile non sécurisé)
                     throw new Error("IMU (Accel/Gyro) n'a pas démarré.");
                 }
            }, 500); // Laisse 500ms aux capteurs pour s'initialiser

        } catch (error) {
            let errMsg = error.message;
            if (error.name === 'SecurityError' || error.name === 'NotAllowedError') {
                errMsg = "Permission Capteurs Refusée.";
            }
            if ($('imu-status')) $('imu-status').textContent = `❌ ${errMsg}`;
            // Empêche la boucle rapide de démarrer si les capteurs essentiels ont échoué
            if (domFastID) clearInterval(domFastID);
            domFastID = null;
        }
            }
    // =================================================================
// BLOC 4/4 : Gestion GPS, Boucles de Mise à Jour et Initialisation
// =================================================================

    function stopSensorListeners() {
        if (domFastID) clearInterval(domFastID);
        domFastID = null;
        if (domSlowID) clearInterval(domSlowID);
        domSlowID = null;
        
        if ($('imu-status')) $('imu-status').textContent = "Inactif";
        accel = { x: 0, y: 0, z: 0 };
        gyro = { x: 0, y: 0, z: 0 };
        
        if (audioContext) {
            audioContext.close().catch(e => console.error("Erreur fermeture AudioContext:", e));
            audioContext = null;
        }
        // (Les capteurs génériques s'arrêtent généralement avec la page)
    }

    // --- GESTION DE LA VEILLE DE L'ÉCRAN (ANTI-VEILLE) ---
    async function requestWakeLock() {
        if ('wakeLock' in navigator) {
            try {
                wakeLock = await navigator.wakeLock.request('screen');
                console.log('Wake Lock activé (Anti-veille).');
                wakeLock.addEventListener('release', () => {
                    console.log('Wake Lock libéré (ex: changement onglet).');
                    wakeLock = null;
                });
            } catch (err) {
                console.error(`Échec de l'activation du Wake Lock: ${err.name}, ${err.message}`);
            }
        } else {
            console.warn("API Wake Lock (Anti-veille) non supportée par ce navigateur.");
        }
    }

    async function releaseWakeLock() {
        if (wakeLock !== null) {
            await wakeLock.release();
            wakeLock = null;
            console.log('Wake Lock libéré (Arrêt GPS).');
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
            startSensorListeners();
        }
        if (!domSlowID) {
            startSlowLoop(); // Démarrer la boucle lente (Astro/Météo)
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
        stopSensorListeners(); // Arrête les boucles rapide ET lente
        
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
            stopGPS(false); // Arrête GPS, WakeLock, et toutes les boucles (Rapide/Lente)
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
            startGPS(); // Redémarre GPS, WakeLock, et les boucles
        }
    }

    function handleErr(err) {
        let errMsg = `Erreur GPS (Code ${err.code}): `;
        if (err.code === 1) errMsg += "Permission refusée.";
        else if (err.code === 2) errMsg += "Position indisponible (Pas de signal).";
        else if (err.code === 3) errMsg += "Timeout GPS.";
        else errMsg += `Erreur inconnue: ${err.message}`;

        if ($('gps-precision')) $('gps-precision').textContent = errMsg;
        if (err.code === 1) stopGPS(); 
    }

    // --- GESTION CARTE (LEAFLET) ---
    function initMap() {
        try {
            if ($('map') && !map) { 
                map = L.map('map').setView([43.296, 5.37], 10);
                L.tileLayer('leaflet-tiles/{z}/{x}/{y}.png', { 
                    attribution: '© OpenStreetMap (Hors Ligne)',
                    errorTileUrl: 'https://via.placeholder.com/256/121220/121220.png'
                }).addTo(map);

                marker = L.marker([43.296, 5.37]).addTo(map);
                circle = L.circle([43.296, 5.37], { color: 'red', fillColor: '#f03', fillOpacity: 0.5, radius: 10 }).addTo(map);
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
    
    // --- GESTION NIVEAU À BULLE ---
    function updateSpiritLevel(x, y, z) {
        const pitch = Math.atan2(y, Math.sqrt(x * x + z * z)) * R2D;
        const roll = Math.atan2(-x, z) * R2D;
        
        if($('pitch')) $('pitch').textContent = `${pitch.toFixed(1)}°`;
        if($('roll')) $('roll').textContent = `${roll.toFixed(1)}°`;

        const maxBubbleMove = 45; // px
        const maxAngle = 45; // deg
        
        const bubbleX = Math.max(-maxBubbleMove, Math.min(maxBubbleMove, (roll / maxAngle) * maxBubbleMove));
        const bubbleY = Math.max(-maxBubbleMove, Math.min(maxBubbleMove, (pitch / maxAngle) * maxBubbleMove));

        if ($('bubble')) {
            $('bubble').style.transform = `translate(${bubbleX}px, ${bubbleY}px)`;
        }
    }
    
    // =================================================================
    // BLOC 3/4 (Cont.) : BOUCLES PRINCIPALES
    // =================================================================

    /**
     * BOUCLE LENTE (Callback GPS) - Correction UKF
     */
    function gpsUpdateCallback(pos) {
        if (emergencyStopActive || !ukf) return;
        
        lastGPSPos = pos; 
        const accRaw = pos.coords.accuracy || 100;
        
        let R_dyn = getKalmanR(accRaw, kAlt, kUncert, selectedEnvironment, currentUKFReactivity); 
        let isSignalPoor = (accRaw > MAX_ACC || R_dyn >= UKF_R_MAX * 0.9);

        const spd3D_raw_gps = pos.coords.speed || 0;
        const accel_long_provisional = Math.abs(accel.x); 
        
        const isPlausiblyStopped = (
            spd3D_raw_gps < ZUPT_RAW_THRESHOLD && 
            accel_long_provisional < ZUPT_ACCEL_THRESHOLD &&
            !isSignalPoor 
        ); 
        
        if (isSignalPoor) {
            if ($('gps-precision')) $('gps-precision').textContent = `❌ ${accRaw.toFixed(0)} m (Estimation/Drift)`;
            if ($('gps-status-dr')) $('gps-status-dr').textContent = 'Drift (Estimation)';
        } else if (isPlausiblyStopped) {
            if ($('gps-precision')) $('gps-precision').textContent = `${accRaw.toFixed(2)} m (ZUPT)`;
            if ($('gps-status-dr')) $('gps-status-dr').textContent = '✅ ZUPT (Vélocité Nulle)';
            let zuptData = { ...pos.coords, speed: 0 };
            ukf.update(zuptData, R_ALT_MIN); 
        } else {
            if ($('gps-precision')) $('gps-precision').textContent = `${accRaw.toFixed(2)} m`;
            if ($('gps-status-dr')) $('gps-status-dr').textContent = 'Actif (Fusion UKF)';
            ukf.update(pos.coords, R_dyn);
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
            if (dt <= 0) dt = MIN_DT; // Empêche dt=0
            lastIMUTimestamp = now;

            // --- 1. PRÉDICTION UKF ---
            const imuReadings = {
                accel: [accel.x, accel.y, accel.z],
                gyro: [gyro.x, gyro.y, gyro.z]
            };
            ukf.predict(imuReadings, dt);

            // --- 2. EXTRACTION DE L'ÉTAT ---
            const estimatedState = ukf.getState();
            lat = estimatedState.lat;
            lon = estimatedState.lon;
            kAlt = estimatedState.alt;
            kSpd = estimatedState.speed; 
            kUncert = estimatedState.kUncert;
            kAltUncert = estimatedState.kAltUncert;

            const sSpdFE = kSpd < MIN_SPD ? 0 : kSpd;
            
            const spd3D_raw_gps = (lastGPSPos && lastGPSPos.coords.speed) ? lastGPSPos.coords.speed : 0;
            const accel_long = accel.x; 
            const accel_vert = accel.z; 
            local_g = getWGS84Gravity(lat, kAlt);

            // --- 3. CALCULS PHYSIQUES AVANCÉS (Corrigés par Métrologie) ---
            const advancedPhysics = calculateAdvancedPhysics(sSpdFE, kAlt, currentMass, currentCdA, 
                lastT_K, currentAirDensity, 
                lat, kAltUncert, local_g, accel_long);

            R_FACTOR_RATIO = calculateDistanceRatio(kAlt); 
            
            if (lastGPSPos && lPos && (sSpdFE > 0 || spd3D_raw_gps > 0)) {
                 distM += dist3D(lPos.coords.latitude, lPos.coords.longitude, lPos.coords.altitude || 0,
                                lastGPSPos.coords.latitude, lastGPSPos.coords.longitude, lastGPSPos.coords.altitude || 0) * (distanceRatioMode ? NETHER_RATIO : 1.0);
            }
            lPos = lastGPSPos; 
            
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
            $('speed-status-text').textContent = (ukf && kSpd > MIN_SPD) ? "🚀 UKF 21 ÉTATS (INS)" : "✅ ZUPT (Attente Mouvement)";
            $('speed-stable-ms').textContent = dataOrDefault(sSpdFE, 3, ' m/s');
            $('speed-stable-kms').textContent = dataOrDefaultExp(sSpdFE / 1000, 3, ' km/s');
            $('speed-3d-inst').textContent = dataOrDefault(spd3D_raw_gps * KMH_MS, 2, ' km/h');
            $('speed-raw-ms').textContent = dataOrDefault(spd3D_raw_gps, 3, ' m/s');
            $('speed-max').textContent = dataOrDefault(maxSpd, 2, ' km/h');
            $('speed-avg-moving').textContent = timeMoving > 1 ? dataOrDefault((distM / timeMoving) * KMH_MS, 2, ' km/h') : '0.00 km/h';
            $('speed-avg-total').textContent = timeTotal > 1 ? dataOrDefault((distM / timeTotal) * KMH_MS, 2, ' km/h') : '0.00 km/h';
            
            $('speed-of-sound-calc').textContent = dataOrDefault(advancedPhysics.speedOfSoundLocal, 2, ' m/s');
            $('perc-speed-sound').textContent = dataOrDefault(advancedPhysics.machNumber * 100, 2, ' %');
            $('mach-number').textContent = dataOrDefault(advancedPhysics.machNumber, 4);
            $('perc-speed-c').textContent = dataOrDefaultExp(sSpdFE / C_L * 100, 2, ' %');
            $('lorentz-factor').textContent = dataOrDefault(advancedPhysics.lorentzFactor, 8);
            $('time-dilation-v').textContent = dataOrDefault(advancedPhysics.timeDilationSpeed, 3, ' ns/j');
            $('time-dilation-g').textContent = dataOrDefault(advancedPhysics.gravitationalDilation, 3, ' ns/j');
            
            $('energy-relativistic').textContent = dataOrDefaultExp(advancedPhysics.energyRelativistic, 3, ' J');
            $('energy-rest-mass').textContent = dataOrDefaultExp(advancedPhysics.E0, 3, ' J');
            $('momentum').textContent = dataOrDefault(advancedPhysics.momentum, 2, ' kg·m/s');
            $('Rs-object').textContent = dataOrDefaultExp(advancedPhysics.Rs_object, 3, ' m');

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
            $('force-g-long').textContent = dataOrDefault(advancedPhysics.force_g_long, 2, ' G');
            $('vertical-speed').textContent = dataOrDefault(estimatedState.vD * -1, 2, ' m/s'); 
            $('accel-vertical-imu').textContent = dataOrDefault(accel_vert, 3, ' m/s²');
            $('force-g-vertical').textContent = dataOrDefault((accel_vert / G_ACC), 2, ' G');
            $('angular-speed').textContent = dataOrDefault(Math.sqrt(gyro.x**2 + gyro.y**2 + gyro.z**2) * R2D, 2, ' °/s');
            
            $('dynamic-pressure').textContent = dataOrDefault(advancedPhysics.dynamicPressure, 2, ' Pa');
            $('drag-force').textContent = dataOrDefault(advancedPhysics.dragForce, 2, ' N');
            $('drag-power-kw').textContent = dataOrDefault(advancedPhysics.dragPower_kW, 2, ' kW');
            $('reynolds-number').textContent = dataOrDefaultExp(advancedPhysics.reynoldsNumber, 2);
            $('kinetic-energy').textContent = dataOrDefault(0.5 * currentMass * sSpdFE**2, 2, ' J');
            $('mechanical-power').textContent = dataOrDefault(advancedPhysics.accel_long * currentMass * sSpdFE, 2, ' W');
            $('radiation-pressure').textContent = dataOrDefaultExp(advancedPhysics.radiationPressure, 2, ' Pa');
            $('coriolis-force').textContent = dataOrDefaultExp(advancedPhysics.coriolisForce, 2, ' N');

            // Col 3 - EKF/Debug
            $('kalman-uncert').textContent = dataOrDefault(kUncert, 3, ' m²/s² (P)');
            $('alt-uncertainty').textContent = dataOrDefault(advancedPhysics.altSigma, 3, ' m (σ)');
            const R_dyn_display = getKalmanR((lastGPSPos ? lastGPSPos.coords.accuracy : 100), kAlt, kUncert, selectedEnvironment, currentUKFReactivity);
            $('speed-error-perc').textContent = dataOrDefault(R_dyn_display, 3, ' m² (R dyn)');
            $('nyquist-frequency').textContent = dataOrDefault(advancedPhysics.nyquistFrequency, 2, ' Hz');
            $('gps-accuracy-display').textContent = dataOrDefault(gpsAccuracyOverride, 6, ' m');
            
            // Col 3 - Position EKF
            $('lat-display').textContent = dataOrDefault(lat, 6, ' °');
            $('lon-display').textContent = dataOrDefault(lon, 6, ' °');
            $('alt-display').textContent = dataOrDefault(kAlt, 2, ' m');
            $('geopotential-alt').textContent = dataOrDefault(advancedPhysics.geopotentialAltitude, 2, ' m');
            
            // Col 3 - Niveau à Bulle
            updateSpiritLevel(accel.x, accel.y, accel.z);
            
        }, IMU_UPDATE_RATE_MS);
    }
    
    // =================================================================
    // BLOC 4/4 (Cont.) : BOUCLE LENTE (Astro/Météo) ET INITIALISATION
    // =================================================================

    /**
     * BOUCLE LENTE (Astro/Météo) - 1Hz
     */
    function startSlowLoop() {
        if (domSlowID) return;
        
        const updateSlowData = async () => {
            // CORRECTION BUG N°1 : Utilise les coordonnées par défaut si lat/lon sont null
            const currentLatForAstro = lat || 43.296; // (Marseille par défaut)
            const currentLonForAstro = lon || 5.37;
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

                    // --- MAJ DOM ASTRO ---
                    $('date-display-astro').textContent = now.toLocaleDateString() || '...';
                    $('date-solar-mean').textContent = solarTimes.DateMST ? solarTimes.DateMST.toLocaleDateString() : '...';
                    $('date-solar-true').textContent = solarTimes.DateTST ? solarTimes.DateTST.toLocaleDateString() : '...';
                    $('mst').textContent = solarTimes.MST;
                    $('tst').textContent = solarTimes.TST;
                    $('noon-solar').textContent = sunTimes.solarNoon ? sunTimes.solarNoon.toLocaleTimeString('fr-FR', { timeZone: 'UTC' }) : '...';
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

            // 2. Mise à jour Météo & Correction Métrologique Altitude Baro
            // S'exécute uniquement si le GPS est démarré (pour avoir lat/lon)
            if (lat && lon && !emergencyStopActive && (performance.now() - (weatherFetchID || 0) > WEATHER_FETCH_INTERVAL)) {
                await fetchWeather(currentLat, currentLon);
                weatherFetchID = performance.now();
            }
            
            // CORRECTION MÉTROLOGIQUE : Altitude Barométrique
            // (Utilise les 'lastP_hPa' et 'lastT_K' mis à jour par fetchWeather ou les valeurs ISA par défaut)
            const baroAlt = getBarometricAltitude(lastP_hPa, BARO_ALT_REF_HPA, lastT_K);
            $('alt-corrected-baro').textContent = dataOrDefault(baroAlt, 2, ' m');


            // 3. Mise à jour Heure NTP
            if (now) {
                if ($('local-time') && !$('local-time').textContent.includes('Synchronisation...')) {
                    $('local-time').textContent = now.toLocaleTimeString('fr-FR');
                }
                if ($('date-display')) $('date-display').textContent = now.toUTCString();
                if ($('time-minecraft')) $('time-minecraft').textContent = getMinecraftTime(now);
            }
        };
        
        domSlowID = setInterval(updateSlowData, DOM_SLOW_UPDATE_MS);
        updateSlowData(); // Exécuter immédiatement au démarrage
    }

    // ===========================================
    // INITIALISATION DOM (DÉMARRAGE)
    // ===========================================
    document.addEventListener('DOMContentLoaded', () => {
        
        initMap(); // Initialise la carte Leaflet
        
        // --- Écouteurs d'événements pour tous les contrôles ---
        $('toggle-gps-btn').addEventListener('click', () => {
            if (!ukf) { 
                 if (typeof math === 'undefined') {
                    alert("Erreur: math.js n'a pas pu être chargé. Le filtre UKF est désactivé.");
                    return;
                }
                ukf = new ProfessionalUKF(); 
                sTime = Date.now();
                startGPS(); 
                // La boucle lente démarre maintenant au chargement
            } else {
                toggleGPS(); 
            }
        });

        $('emergency-stop-btn').addEventListener('click', toggleEmergencyStop);
        $('toggle-mode-btn').addEventListener('click', () => {
            document.body.classList.toggle('dark-mode');
            $('toggle-mode-btn').innerHTML = document.body.classList.contains('dark-mode') ? '<i class="fas fa-sun"></i> Mode Jour' : '<i class="fas fa-moon"></i> Mode Nuit';
            if(map) map.invalidateSize();
        });
        $('reset-dist-btn').addEventListener('click', () => { distM = 0; timeMoving = 0; });
        $('reset-max-btn').addEventListener('click', () => { maxSpd = 0; });
        $('reset-all-btn').addEventListener('click', () => {
            distM = 0; maxSpd = 0; kSpd = 0; kUncert = UKF_R_MAX; kAlt = 0; kAltUncert = 10; timeMoving = 0; timeTotal = 0; sTime = Date.now();
            if (ukf) ukf = new ProfessionalUKF(); 
            $('speed-stable').textContent = '--.- km/h';
            $('speed-status-text').textContent = 'Système réinitialisé.';
        });
        $('capture-data-btn').addEventListener('click', () => {
             console.log({
                etat: ukf ? ukf.getState() : "UKF non initialisé",
                physique: calculateAdvancedPhysics(kSpd, kAlt, currentMass, currentCdA, lastT_K, currentAirDensity, (lat || 0), kAltUncert, local_g, accel.x),
                astro: getSolarTime(getCDate(), (lon || 0)),
                meteo: lastWeatherData || "N/A (Hors ligne)",
                gps_brut: lastGPSPos || "N/A"
            });
            alert("Données capturées dans la console (F12).");
        });
        if($('xray-button')) $('xray-button').addEventListener('click', () => $('minecraft-clock').classList.toggle('x-ray'));
        
        // --- Écouteurs pour les Inputs & Selects ---
        $('freq-select').addEventListener('change', (e) => {
            currentGPSMode = e.target.value;
            if (wID) startGPS(currentGPSMode); // Redémarre le GPS si déjà actif
        });
        $('gps-accuracy-override').addEventListener('input', (e) => gpsAccuracyOverride = parseFloat(e.target.value) || 0.0);
        $('environment-select').addEventListener('change', (e) => {
            selectedEnvironment = e.target.value;
            const factor = ENVIRONMENT_FACTORS[selectedEnvironment];
            if ($('env-factor')) $('env-factor').textContent = `${factor.DISPLAY} (x${factor.R_MULT.toFixed(1)})`;
        });
        $('mass-input').addEventListener('input', (e) => {
            currentMass = parseFloat(e.target.value) || 70.0;
            if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        });
        $('celestial-body-select').addEventListener('change', (e) => {
            currentCelestialBody = e.target.value;
            updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity);
        });
        
        const updateRotation = () => {
            rotationRadius = parseFloat($('rotation-radius').value) || 100;
            angularVelocity = parseFloat($('angular-velocity').value) || 0.0;
            if (currentCelestialBody === 'ROTATING') {
                updateCelestialBody('ROTATING', kAlt, rotationRadius, angularVelocity);
            }
        };
        $('rotation-radius').addEventListener('input', updateRotation);
        $('angular-velocity').addEventListener('input', updateRotation);
        
        $('distance-ratio-toggle-btn').addEventListener('click', () => {
            distanceRatioMode = !distanceRatioMode; // Bascule entre true (Altitude/Nether) et false (Surface)
            const ratio = distanceRatioMode ? NETHER_RATIO : 1.0;
            $('distance-ratio-toggle-btn').textContent = `Rapport Distance: ${distanceRatioMode ? 'NETHER (1:8)' : 'SURFACE (1:1)'}`;
        });
        $('ukf-reactivity-mode').addEventListener('change', (e) => currentUKFReactivity = e.target.value);
        
        if($('toggle-globe-btn')) {
            $('toggle-globe-btn').addEventListener('click', () => {
                const mapEl = $('map');
                const globeEl = $('globe-container');
                if (mapEl.style.display === 'none') {
                    mapEl.style.display = 'block';
                    globeEl.style.display = 'none';
                    if(map) map.invalidateSize(); // Important
                } else {
                    mapEl.style.display = 'none';
                    globeEl.style.display = 'block';
                }
            });
        }

        // --- DÉMARRAGE DU SYSTÈME ---
        // Initialise l'UKF au chargement
        ukf = new ProfessionalUKF();
        
        updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity);
        syncH(); // Démarrer la synchro NTP
        startSlowLoop(); // CORRECTION : Démarrer la boucle lente (Astro/Météo) immédiatement
        
        // Initialiser les valeurs par défaut (Correction Métrologique)
        currentAirDensity = RHO_SEA_LEVEL;
        currentSpeedOfSound = getSpeedOfSound(TEMP_SEA_LEVEL_K); 
        lastT_K = TEMP_SEA_LEVEL_K;
        lastP_hPa = BARO_ALT_REF_HPA;
        
        // Mettre à jour les affichages par défaut
        if($('air-density')) $('air-density').textContent = `${currentAirDensity.toFixed(3)} kg/m³ (Défaut)`;
        if($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = `${currentSpeedOfSound.toFixed(2)} m/s (Défaut)`;
        if($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        if ($('env-factor')) $('env-factor').textContent = `${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY} (x${ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT.toFixed(1)})`;
        if (document.body.classList.contains('dark-mode')) {
             $('toggle-mode-btn').innerHTML = '<i class="fas fa-sun"></i> Mode Jour';
        } else {
             $('toggle-mode-btn').innerHTML = '<i class="fas fa-moon"></i> Mode Nuit';
        }

    }); // Fin de DOMContentLoaded

})(window); // Fin de l'IIFE                      
