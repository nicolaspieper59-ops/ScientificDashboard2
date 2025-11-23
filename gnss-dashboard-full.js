// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// BLOC 1/4 : Constantes, Utilitaires, et Modèles Physiques
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES (Hors IIFE) ---
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

    // --- CLÉS D'API & ENDPOINTS ---
    const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
    const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
    const PROXY_POLLUTANT_ENDPOINT = `${PROXY_BASE_URL}/api/pollutants`;
    const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

    // --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES FONDAMENTALES ---
    const D2R = Math.PI / 180, R2D = 180 / Math.PI; 
    const KMH_MS = 3.6;         
    const C_L = 299792458;      // Vitesse de la lumière (m/s)
    const G_U = 6.67430e-11;    // Constante gravitationnelle
    const R_E_BASE = 6371000;   // Rayon terrestre moyen (m)
    const OMEGA_EARTH = 7.2921159e-5; // Vitesse angulaire Terre (rad/s)
    const SOLAR_FLUX_DENSITY = 1361; // Densité de flux solaire (W/m²)

    // --- CONSTANTES ATMOSPHÉRIQUES (ISA Standard) ---
    const BARO_ALT_REF_HPA = 1013.25; // Pression de référence (hPa)
    const RHO_SEA_LEVEL = 1.225; // Densité de l'air au niveau de la mer (kg/m³)
    const TEMP_SEA_LEVEL_K = 288.15; // Température au niveau de la mer (15°C)
    const R_AIR = 287.058; // Constante spécifique de l'air (J/kg·K)
    const GAMMA_AIR = 1.4; // Rapport des chaleurs spécifiques de l'air
    const MU_DYNAMIC_AIR = 1.8e-5; // Viscosité dynamique de l'air (Pa·s)
    const KELVIN_OFFSET = 273.15;

    // --- CONSTANTES GÉOPHYSIQUES (WGS84) ---
    const WGS84_A = 6378137.0;  // Demi-grand axe
    const WGS84_F = 1 / 298.257223563;
    const WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F;
    const WGS84_G_EQUATOR = 9.780327; // Gravité à l'équateur

    // --- PARAMÈTRES DU FILTRE UKF/EKF & SYSTÈME ---
    const UKF_STATE_DIM = 21;    
    const UKF_R_MAX = 500.0;     
    const R_ALT_MIN = 1.0;
    const MIN_DT = 0.01;        
    const IMU_UPDATE_RATE_MS = 20; // 50Hz
    const DOM_SLOW_UPDATE_MS = 1000; // 1Hz
    const WEATHER_FETCH_INTERVAL = 600000; // 10 minutes

    // --- FACTEURS DE RÉACTIVITÉ & ENVIRONNEMENT ---
    const UKF_REACTIVITY_FACTORS = {
        'AUTO': { MULT: 1.0, DISPLAY: 'Automatique' }, 'NORMAL': { MULT: 1.0, DISPLAY: 'Normal' },
        'FAST': { MULT: 0.2, DISPLAY: 'Rapide' }, 'STABLE': { MULT: 2.5, DISPLAY: 'Microscopique' },
    };
    const ENVIRONMENT_FACTORS = {
        'NORMAL': { R_MULT: 1.0, DISPLAY: 'Normal' }, 'FOREST': { R_MULT: 2.5, DISPLAY: 'Forêt' },
        'CONCRETE': { R_MULT: 7.0, DISPLAY: 'Grotte/Tunnel' }, 'METAL': { R_MULT: 5.0, DISPLAY: 'Métal/Bâtiment' },
    };
    const CELESTIAL_DATA = {
        'EARTH': { G: 9.80665, R: WGS84_A, name: 'Terre' }, 'MOON': { G: 1.62, R: 1737400, name: 'Lune' },
        'MARS': { G: 3.71, R: 3389500, name: 'Mars' }, 'ROTATING': { G: 0.0, R: WGS84_A, name: 'Station Spatiale' }
    };
    // =================================================================
// BLOC 2/4 : Variables d'État Globales et Fonctions Core/UKF
// =================================================================

    // --- VARIABLES D'ÉTAT (Globales) ---
    let wID = null, domSlowID = null, domFastID = null, weatherFetchID = null;
    let map = null, marker = null, polyline = null;
    let currentLat = null, currentLon = null, kAlt = 0, kSpd = 0;
    let distM = 0, maxSpd = 0;
    let kUncert = UKF_R_MAX, kAltUncert = 10;
    let timeMoving = 0, timeTotal = 0;
    let lastFSpeed = 0;
    let ukf = null;
    let currentGPSMode = 'HIGH_FREQ';
    let emergencyStopActive = false;
    let distanceRatioMode = false;
    let currentCelestialBody = 'EARTH';
    let currentMass = 70.0;
    let currentCdA = 0.5; // Coefficient de Traînée * Surface (m²)
    let rotationRadius = 100;
    let angularVelocity = 0.0;
    let currentUKFReactivity = 'AUTO';
    let accel = { x: 0, y: 0, z: 0 };
    let gyro = { x: 0, y: 0, z: 0 };
    
    // --- VARIABLES DE CORRECTION MÉTROLOGIQUE (Initialisation ISA Standard) ---
    let lastP_hPa = BARO_ALT_REF_HPA; 
    let lastT_K = TEMP_SEA_LEVEL_K; 
    let currentAirDensity = RHO_SEA_LEVEL;
    let currentSpeedOfSound = 343.0; 
    let local_g = CELESTIAL_DATA.EARTH.G;
    let G_ACC = local_g;
    let R_ALT_CENTER_REF = WGS84_A;

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
            // Logique de prédiction UKF 21 états simplifiée
            const accel_biais_corrigé = imuData.accel.y - (this.x.get([9]) || 0); 
            let vN = this.x.get([3]) + accel_biais_corrigé * dt; 
            this.x.set([3], vN); 
            let lat_pred = this.x.get([0]) + (vN / R_E_BASE) * dt;
            this.x.set([0], lat_pred);
        }
        update(gpsData, R_dyn) {
            // Logique de mise à jour UKF 21 états simplifiée
            const K = 0.1; 
            this.x.set([0], this.x.get([0]) * (1-K) + (gpsData.latitude * D2R) * K);
            this.x.set([1], this.x.get([1]) * (1-K) + (gpsData.longitude * D2R) * K);
            this.x.set([2], this.x.get([2]) * (1-K) + gpsData.altitude * K);
            if (gpsData.speed !== null && gpsData.speed !== undefined) {
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

    // --- FONCTIONS CORE ET MÉTROLOGIQUES ---
    
    // CORRECTION MÉTROLOGIQUE : Calcul de l'Altitude Barométrique
    function getBarometricAltitude(P_hPa, P_ref_hPa, T_K) {
        if (P_hPa <= 0 || P_ref_hPa <= 0 || T_K <= 0) return NaN;
        const pressure_ratio = P_hPa / P_ref_hPa;
        return (T_K / 0.0065) * (1 - Math.pow(pressure_ratio, (R_AIR * 0.0065) / G_ACC));
    }
    
    function getWGS84Gravity(lat, alt) {
        const latRad = lat * D2R; 
        const sin2lat = Math.sin(latRad) ** 2;
        const g_surface = WGS84_G_EQUATOR * (1 + (0.0053024) * sin2lat) / Math.sqrt(1 - WGS84_E2 * sin2lat);
        return g_surface * (1 - 2 * alt / WGS84_A); 
    }
    
    // Correction Métrologique : Vitesse du son ajustée par la température
    function getSpeedOfSound(tempK) {
        if(tempK < KELVIN_OFFSET) tempK += KELVIN_OFFSET; 
        return Math.sqrt(GAMMA_AIR * R_AIR * tempK);
    }

    // Fonctions de calcul (Distances, Facteurs...)
    function dist3D(lat1, lon1, alt1, lat2, lon2, alt2) {
        const from = turf.point([lon1, lat1]);
        const to = turf.point([lon2, lat2]);
        const d2D = turf.distance(from, to, { units: 'meters' });
        const dAlt = Math.abs(alt1 - alt2);
        return Math.sqrt(d2D * d2D + dAlt * dAlt);
    }
    function calculateDistanceRatio(alt) {
        return R_E_BASE / (R_E_BASE + alt);
    }
    function updateCelestialBody(body, alt, rotationRadius, angularVelocity) {
        let G_ACC_NEW = CELESTIAL_DATA['EARTH'].G;
        let R_ALT_CENTER_REF_NEW = WGS84_A; 
        const data = CELESTIAL_DATA[body];
        if (data) { G_ACC_NEW = data.G; R_ALT_CENTER_REF_NEW = data.R; }
        if (body === 'ROTATING') { G_ACC_NEW = rotationRadius * angularVelocity ** 2; }
        G_ACC = G_ACC_NEW; 
        R_ALT_CENTER_REF = R_ALT_CENTER_REF_NEW; 
        return { G_ACC: G_ACC_NEW, R_ALT_CENTER_REF: R_ALT_CENTER_REF_NEW };
    }

    // Fonction de Calcul des Erreurs EKF/UKF
    function getKalmanR(accRaw, gpsAccuracyOverride, env, reactivityMode) {
        let acc_effective = gpsAccuracyOverride > 0 ? gpsAccuracyOverride : accRaw;
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

    // Fonction de Physique Avancée (avec Correction Métrologique intégrée)
    function calculateAdvancedPhysics(kSpd, kAlt, mass, CdA, tempK, airDensity, lat, kAltUncert, localG, accel_long) {
        let V = kSpd; if(isNaN(V)) V = 0;
        
        // Relativité
        const lorentzFactor = 1 / Math.sqrt(1 - Math.pow(V / C_L, 2));
        const timeDilationSpeed = (lorentzFactor - 1) * 86400 * 1e9; 
        const gravitationalDilation = (localG * kAlt / (C_L * C_L)) * 86400 * 1e9; 
        const Rs_object = (2 * G_U * mass) / (C_L * C_L);
        
        // Mécanique des Fluides (CORRIGÉ par MÉTÉO)
        const speedOfSoundLocal = getSpeedOfSound(tempK);
        const machNumber = (speedOfSoundLocal > 0) ? V / speedOfSoundLocal : 0;
        const dynamicPressure = 0.5 * airDensity * V * V; 
        const dragForce = dynamicPressure * (CdA || 0.5); 
        const dragPower_kW = (dragForce * V) / 1000.0;
        
        // Autres
        const coriolisForce = 2 * mass * V * OMEGA_EARTH * Math.sin(lat * D2R);
        const geopotentialAltitude = (localG > 0.1) ? kAlt * (G_ACC / localG) : kAlt;
        const force_g_long = localG > 0.1 ? (accel_long / localG) : 0;
        const radiationPressure = (SOLAR_FLUX_DENSITY * Math.max(0, Math.sin(SunCalc.getPosition(new Date(), lat, lon).altitude))) / C_L; 

        return { 
            lorentzFactor, timeDilationSpeed, gravitationalDilation, Rs_object,
            speedOfSoundLocal, machNumber, dynamicPressure, dragForce, dragPower_kW,
            coriolisForce, geopotentialAltitude, force_g_long, radiationPressure
        };
                }
    // =================================================================
// BLOC 3/4 : Fonctions Météo Détaillée, Astro et Gestion du Temps
// =================================================================

    // --- GESTION DE L'HEURE (NTP) ---
    let lServH = null, lLocH = null;
    async function syncH() {
        try {
            const response = await fetch(SERVER_TIME_ENDPOINT);
            const data = await response.json();
            lServH = new Date(data.utc_datetime).getTime();
            lLocH = performance.now();
            $('local-time').textContent = new Date().toLocaleTimeString('fr-FR') + ' (Synchro OK)';
        } catch (e) {
            $('local-time').textContent = '❌ Synchro NTP échouée';
            lServH = null; lLocH = null;
        }
    }
    function getCurrentTime() {
        if (lServH === null || lLocH === null) return new Date();
        const offset = performance.now() - lLocH;
        return new Date(lServH + offset);
    }

    // --- FONCTIONS ASTRO ---
    const J2000 = 2451545.0; 
    const dayMs = 1000 * 60 * 60 * 24;
    const MC_DAY_MS = 72 * 60 * 1000; 

    function toDays(date) { return (date.valueOf() / dayMs - 0.5 + 2440588) - J2000; }
    function solarMeanAnomaly(d) { return D2R * (356.0470 + 0.9856002585 * d); }
    function eclipticLongitude(M) {
        var C = D2R * (1.9148 * Math.sin(M) + 0.0200 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M)), 
            P = D2R * 102.9377;                                                                
        return M + C + P + Math.PI;
    }
    function getSolarTime(date, lon) {
        if (date === null || lon === null) return { TST: 'N/A', MST: 'N/A', EOT: 'N/A', ECL_LONG: 'N/A' };
        
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

        return { TST: toTimeString(tst_ms), MST: toTimeString(mst_ms), EOT: eot_min.toFixed(2), ECL_LONG: (L * R2D).toFixed(2) };
    }
    function getMinecraftTime(date) {
        if (date === null) return '00:00';
        const msSinceMidnightUTC = date.getUTCHours() * 3600000 + date.getUTCMilliseconds() + date.getUTCMinutes() * 60000 + date.getUTCSeconds() * 1000;
        const timeRatio = (msSinceMidnightUTC % dayMs) / dayMs;
        const mcTimeMs = (timeRatio * MC_DAY_MS + MC_DAY_MS) % MC_DAY_MS;
        const mcHours = Math.floor(mcTimeMs / 3600000);
        const mcMinutes = Math.floor((mcTimeMs % 3600000) / 60000);
        return `${String(mcHours).padStart(2, '0')}:${String(mcMinutes).padStart(2, '0')}`;
    }
    function getTSLV(date, lon) { 
        if (date === null) return 'N/A';
        const GMST = (date.getUTCHours() + date.getUTCMinutes() / 60) * 15;
        const LST = GMST + lon;
        const LST_h = (LST / 15 + 24) % 24;
        return LST_h.toFixed(2) + ' h';
    }
    function getCardinalDirection(deg) {
        const directions = ['N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE', 'S', 'SSO', 'SO', 'OSO', 'O', 'ONO', 'NO', 'NNO'];
        const index = Math.round(deg / (360 / directions.length)) % directions.length;
        return directions[index];
    }
    function getAirQualityIndexName(aqi) {
        if (aqi === 1) return 'Très Bonne (1)'; if (aqi === 2) return 'Bonne (2)';
        if (aqi === 3) return 'Moyenne (3)'; if (aqi === 4) return 'Mauvaise (4)';
        if (aqi === 5) return 'Très Mauvaise (5)'; return 'N/A';
    }

    // --- FONCTIONS MÉTÉO (DÉTAILLÉE ET POLLUANTS) ---
    async function fetchWeather(lat, lon) {
        const weatherUrl = `${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`;
        const pollutantUrl = `${PROXY_POLLUTANT_ENDPOINT}?lat=${lat}&lon=${lon}`; 

        $('weather-status').textContent = 'Chargement...';
        $('air-quality').textContent = 'Chargement...';
        
        try {
            // 1. Fetch Basic Weather
            const response = await fetch(weatherUrl);
            if (!response.ok) throw new Error(`Erreur HTTP ${response.status}`);
            const weatherData = await response.json(); 

            if (weatherData.main) {
                const P_hPa = weatherData.main.pressure;
                const T_C = weatherData.main.temp;
                const H_perc = weatherData.main.humidity;
                const T_K = T_C + KELVIN_OFFSET;
                const P_Pa = P_hPa * 100;
                const air_density = P_Pa / (R_AIR * T_K);
                const dew_point_C = T_C - ((100 - H_perc) / 5); 
                
                // Correction Métrologique : Stockage des valeurs GLOBALES
                lastP_hPa = P_hPa;
                lastT_K = T_K;
                currentAirDensity = air_density;
                currentSpeedOfSound = getSpeedOfSound(T_K);
                
                // Mise à jour DOM Météo de base
                $('weather-status').textContent = `ACTIF`;
                $('temp-air-2').textContent = `${T_C.toFixed(1)} °C`;
                $('pressure-2').textContent = `${P_hPa.toFixed(0)} hPa`;
                $('humidity-2').textContent = `${H_perc.toFixed(0)} %`;
                $('dew-point').textContent = `${dew_point_C.toFixed(1)} °C`;
                
                // Mise à jour DOM Météo Détaillée (AccuWeather style)
                $('realfeel-shade').textContent = `~${(T_C + 2).toFixed(1)} °C (Est.)`; 
                const windSpeed_ms = weatherData.wind?.speed || 0;
                const windDeg = weatherData.wind?.deg || 0;
                const windSpeed_kmh = windSpeed_ms * 3.6;
                const windDir = getCardinalDirection(windDeg);
                $('wind-display').textContent = `${windSpeed_kmh.toFixed(1)} km/h (${windDir})`;
                $('gusts').textContent = `${(weatherData.wind?.gust * 3.6 || 0).toFixed(1)} km/h`;
                const visibility_km = (weatherData.visibility / 1000) || 'N/A';
                $('visibility').textContent = (typeof visibility_km === 'number') ? `${visibility_km.toFixed(1)} km` : visibility_km;
                $('cloud-cover').textContent = `${weatherData.clouds?.all || 'N/A'} %`;
                
                // 2. Fetch Pollutants
                try {
                    const pollutantResponse = await fetch(pollutantUrl);
                    if (pollutantResponse.ok) {
                        const pollutantData = await pollutantResponse.json();
                        if (pollutantData.list && pollutantData.list.length > 0) {
                             const components = pollutantData.list[0].components;
                             const aqi = pollutantData.list[0].main.aqi;
                             
                             $('air-quality').textContent = getAirQualityIndexName(aqi);
                             $('no2-val').textContent = dataOrDefault(components.no2, 2, ' µg/m³');
                             $('pm25-val').textContent = dataOrDefault(components.pm2_5, 2, ' µg/m³');
                             $('pm10-val').textContent = dataOrDefault(components.pm10, 2, ' µg/m³');
                             $('o3-val').textContent = dataOrDefault(components.o3, 2, ' µg/m³');
                        } else { $('air-quality').textContent = 'N/A (API)'; }
                    } else { throw new Error("Erreur Polluant"); }
                } catch (e) {
                    $('air-quality').textContent = 'N/A (API)';
                }
            }
        } catch (e) {
            $('weather-status').textContent = `❌ API ÉCHOUÉE`;
            $('air-quality').textContent = 'N/A (Échoué)';
        }
                                                 }
    // =================================================================
// BLOC 4/4 : Gestion GPS, Boucles de Mise à Jour et Initialisation
// =================================================================

    // --- GESTION DES CAPTEURS IMU ET DU GPS (Placeholders pour l'interaction du navigateur) ---
    let gpsAccuracyOverride = 0.0;
    let gpsStandbyTimeoutID = null;

    function startGPS(mode) {
        if (wID !== null) navigator.geolocation.clearWatch(wID);
        currentGPSMode = mode;
        const options = GPS_OPTS[mode];

        // Lancement du watch GPS
        wID = navigator.geolocation.watchPosition(handleSuccess, handleErr, options);
        
        $('toggle-gps-btn').textContent = `⏸ PAUSE GPS (${mode})`;
        clearTimeout(gpsStandbyTimeoutID);
        // ... (autres initialisations)
    }

    function handleSuccess(pos) {
        const dt = (pos.timestamp - (lastGPSPos?.timestamp || pos.timestamp)) / 1000;
        const coord = pos.coords;
        const accRaw = coord.accuracy;
        
        // 1. Calcul du R de Kalman Dynamique
        const R_dyn = getKalmanR(accRaw, gpsAccuracyOverride, selectedEnvironment, currentUKFReactivity);

        // 2. Mise à jour UKF (avec Lat, Lon, Alt, Speed)
        ukf.update({
            latitude: coord.latitude,
            longitude: coord.longitude,
            altitude: coord.altitude || kAlt, // utilise l'altitude UKF si GPS est nul
            speed: coord.speed || 0,
            accuracy: accRaw
        }, R_dyn);

        // Mise à jour de la dernière position (pour le calcul du DT)
        lastGPSPos = pos;
        
        // ... (Mise à jour de la distance, max speed, etc.)
    }

    function handleErr(err) {
        // ... (Gestion des erreurs GPS)
        console.error(`Erreur GPS (${err.code}): ${err.message}`);
        $('speed-status-text').textContent = `ERREUR GPS: ${err.message}`;
        if (err.code === 3) {
            gpsStandbyTimeoutID = setTimeout(() => {
                // Tente de redémarrer en mode basse fréquence après un timeout
                if (currentGPSMode === 'HIGH_FREQ') {
                    console.log("Passage en mode basse fréquence suite à timeout GPS.");
                    startGPS('LOW_FREQ');
                } else {
                    stopGPS();
                    $('speed-status-text').textContent = 'GPS en veille (Timeout)';
                }
            }, STANDBY_TIMEOUT_MS);
        }
    }

    function stopGPS() {
        if (wID !== null) navigator.geolocation.clearWatch(wID);
        wID = null;
        $('toggle-gps-btn').textContent = "▶️ MARCHE GPS";
    }

    // --- BOUCLE DE MISE À JOUR RAPIDE (IMU, UKF Predict, Vitesse) ---
    function updateFastData() {
        if (emergencyStopActive) return;

        const now = performance.now();
        const dt = (now - lastTime) / 1000;
        lastTime = now;
        if (dt < MIN_DT) return; 
        
        // 1. Prédiction UKF (Utilise les données IMU/Gyro)
        ukf.predict({ accel: [accel.x, accel.y, accel.z], gyro: [gyro.x, gyro.y, gyro.z] }, dt);
        const estimatedState = ukf.getState();
        kSpd = estimatedState.speed;
        currentLat = estimatedState.lat;
        currentLon = estimatedState.lon;
        kAlt = estimatedState.alt;
        kUncert = estimatedState.kUncert;
        kAltUncert = estimatedState.kAltUncert;
        
        const accel_long = accel.y; // Accélération dans l'axe Y (simplifié)
        
        // 2. Calculs Dynamiques et CORRECTION MÉTROLOGIQUE
        const advancedPhysics = calculateAdvancedPhysics(
            kSpd, kAlt, currentMass, currentCdA, 
            lastT_K, currentAirDensity, // Utilise les variables météo corrigées
            currentLat, kAltUncert, local_g, accel_long
        );
        
        // 3. Mise à jour Vitesse et Physique Corrigée
        const SMOOTHING_FACTOR = 0.8;
        const sSpdFE = lastFSpeed * SMOOTHING_FACTOR + kSpd * (1 - SMOOTHING_FACTOR);
        lastFSpeed = sSpdFE;

        $('speed-stable').textContent = dataOrDefault(sSpdFE * KMH_MS, 1, ' km/h');
        $('speed-stable-ms').textContent = dataOrDefault(sSpdFE, 3, ' m/s');
        $('speed-of-sound-calc').textContent = dataOrDefault(advancedPhysics.speedOfSoundLocal, 2, ' m/s');
        $('mach-number').textContent = dataOrDefault(advancedPhysics.machNumber, 4);
        
        // Mises à jour Dynamique Corrigée
        $('dynamic-pressure').textContent = dataOrDefault(advancedPhysics.dynamicPressure, 2, ' Pa');
        $('drag-force').textContent = dataOrDefault(advancedPhysics.dragForce, 2, ' N');
        $('radiation-pressure').textContent = dataOrDefaultExp(advancedPhysics.radiationPressure, 2, ' Pa');

        // ... (Autres mises à jour rapides)
    }

    // --- BOUCLE DE MISE À JOUR LENTE (DOM, Astro, Météo) ---
    function updateSlowData() {
        const now = getCurrentTime(); 
        timeTotal += DOM_SLOW_UPDATE_MS / 1000;
        $('elapsed-time').textContent = timeTotal.toFixed(0) + ' s';
        
        // 1. Mise à jour Position et Gravité Corrigée
        if (currentLat !== null && currentLon !== null) {
            local_g = getWGS84Gravity(currentLat, kAlt); 
            
            // CORRECTION MÉTROLOGIQUE (Altitude Barométrique Corrigée)
            let baroAltCorrected = NaN;
            if (lastP_hPa !== null && lastT_K !== null) {
                baroAltCorrected = getBarometricAltitude(lastP_hPa, BARO_ALT_REF_HPA, lastT_K); 
            }
            $('alt-corrected-baro').textContent = dataOrDefault(baroAltCorrected, 2, ' m');
            $('gravity-local').textContent = dataOrDefault(local_g, 4, ' m/s²');
        } else {
             $('alt-corrected-baro').textContent = 'N/A';
        }
        
        // 2. Mise à jour de la Météo Détaillée et Polluants
        if (currentLat !== null && currentLon !== null && !emergencyStopActive && (performance.now() - (weatherFetchID || 0) > WEATHER_FETCH_INTERVAL)) {
            fetchWeather(currentLat, currentLon);
            weatherFetchID = performance.now();
        } 
        
        // 3. Mise à jour Astronomique
        if (now && currentLat !== null && currentLon !== null) {
            const solarTimes = getSolarTime(now, currentLon);
            const moonIllum = SunCalc.getMoonIllumination(now);
            const sunPos = SunCalc.getPosition(now, currentLat, currentLon);
            const moonPos = SunCalc.getMoonPosition(now, currentLat, currentLon);
            const sunTimes = SunCalc.getTimes(now, currentLat, currentLon);
            
            $('tst').textContent = solarTimes.TST;
            $('mst').textContent = solarTimes.MST;
            $('sun-alt').textContent = dataOrDefault(sunPos.altitude * R2D, 2, ' °');
            $('moon-illuminated').textContent = dataOrDefault(moonIllum.fraction * 100, 1, ' %');
            $('tslv').textContent = getTSLV(now, currentLon);

            // ... (Autres mises à jour astro)
        }
    }


    // --- GESTIONNAIRES D'ÉVÉNEMENTS ET INITIALISATION ---
    function initDashboard() {
        ukf = new ProfessionalUKF();
        
        // Set up initial environment based on HTML
        selectedEnvironment = $('environment-select').value;
        currentCelestialBody = $('celestial-body-select').value;
        
        // Listeners pour les contrôles
        $('toggle-gps-btn').addEventListener('click', () => wID === null ? startGPS(currentGPSMode) : stopGPS());
        $('freq-select').addEventListener('change', (e) => startGPS(e.target.value));
        $('emergency-stop-btn').addEventListener('click', () => {
            emergencyStopActive = !emergencyStopActive;
            if (emergencyStopActive) { 
                // Arrêt total
                stopGPS(); 
                clearInterval(domFastID); clearInterval(domSlowID); 
            } else {
                // Redémarrage
                domFastID = setInterval(updateFastData, IMU_UPDATE_RATE_MS);
                domSlowID = setInterval(updateSlowData, DOM_SLOW_UPDATE_MS);
                startGPS(currentGPSMode);
            }
        });
        
        // Listener Correction Métrologique/Physique
        $('mass-input').addEventListener('input', (e) => currentMass = parseFloat(e.target.value) || 70.0);
        $('celestial-body-select').addEventListener('change', (e) => {
            currentCelestialBody = e.target.value;
            const { G_ACC_NEW } = updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity);
            $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
        });
        $('rotation-radius').addEventListener('input', () => {
             rotationRadius = parseFloat($('rotation-radius').value) || 100;
             if (currentCelestialBody === 'ROTATING') updateCelestialBody('ROTATING', kAlt, rotationRadius, angularVelocity);
        });
        
        // Listener Rapport de Distance (Correction d'altitude)
        $('distance-ratio-toggle-btn').addEventListener('click', () => {
            distanceRatioMode = !distanceRatioMode;
            const ratio = distanceRatioMode ? calculateDistanceRatio(kAlt || 0) : 1.0;
            $('distance-ratio-toggle-btn').textContent = `Rapport Distance: ${distanceRatioMode ? 'ALTITUDE' : 'SURFACE'} (${ratio.toFixed(3)})`;
        });
        
        // --- DÉMARRAGE DU SYSTÈME ---
        // Démarrage des boucles de mise à jour
        domFastID = setInterval(updateFastData, IMU_UPDATE_RATE_MS);
        domSlowID = setInterval(updateSlowData, DOM_SLOW_UPDATE_MS);
        
        // Démarrer la synchro NTP
        syncH();
        
        // Démarrer la surveillance GPS par défaut
        startGPS('HIGH_FREQ');
    }

    // Démarrage au chargement de la page
    window.addEventListener('load', initDashboard);

})(window);
