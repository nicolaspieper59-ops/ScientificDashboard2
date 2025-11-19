// =================================================================
// BLOC 1/3 : ekf_logic.js (Refonte EKF 15 États & Constantes)
// REQUIERT LA LIBRAIRIE EXTERNE 'math.js'
// =================================================================

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const R_AIR = 287.058;      // Constante spécifique de l'air sec (J/kg·K)
const GAMMA = 1.4;          // Rapport des chaleurs spécifiques pour l'air sec
const R_E_BASE = 6371000;   // Rayon terrestre moyen (m)
const KMH_MS = 3.6;         // Conversion m/s vers km/h

// --- PARAMÈTRES DU FILTRE DE KALMAN (EKF INS/GNSS - 15 ÉTATS) ---
const STATE_SIZE = 15; 
const Q_PROCESS = 1e-6; 
const R_MIN = 0.01;         
const R_MAX = 500.0;        
const ALT_TH = -50;         // SEUIL BAS (Souterrain / Tunnel)
const ALT_HIGH_TH = 3000;   // SEUIL HAUT (Avion / Montagne)
const MAX_ACC = 200;        
const MIN_SPD = 0.05;       
const NETHER_RATIO = 8.0;   

// --- VALEURS DE L'ATMOSPHÈRE HYPERLOOP (PROCHE-VIDE) ---
// Ces valeurs simulent une pression très faible (10 Pa = 0.1 hPa)
const P_HYPERLOOP_PA = 10.0;     // Pression Hyperloop (Pascals)
const T_HYPERLOOP_C = 20.0;     // Température interne contrôlée (°C)
const H_HYPERLOOP_PERC = 0.0;   // Humidité proche de zéro (%)

// Matrice de Covariance P initiale
let P_MATRIX = null; 

// --- FACTEURS ENVIRONNEMENTAUX --- 
const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0, DISPLAY: 'Normal' },
    'FOREST': { R_MULT: 2.5, DISPLAY: 'Forêt' },
    'CONCRETE': { R_MULT: 7.0, DISPLAY: 'Grotte/Tunnel' },
    'METAL': { R_MULT: 5.0, DISPLAY: 'Métal/Bâtiment' },
};

// --- DONNÉES CÉLESTES/GRAVITÉ ---
const CELESTIAL_DATA = {
    'EARTH': { G: 9.80665, R: R_E_BASE, name: 'Terre' },
    'MOON': { G: 1.62, R: 1737400, name: 'Lune' },
    'MARS': { G: 3.71, R: 3389500, name: 'Mars' },
    'ROTATING': { G: 0.0, R: R_E_BASE, name: 'Station Spatiale' }
};

// --- FONCTIONS MATHÉMATIQUES ET PHYSIQUES ---

const dist = (lat1, lon1, lat2, lon2, R_ref) => {
    // ... (Fonction dist) ...
    const R = R_ref || R_E_BASE; 
    const dLat = (lat2 - lat1) * D2R, dLon = (lon2 - lon1) * D2R;
    lat1 *= D2R; lat2 *= D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R * c;
};

function getGravityLocal(alt, bodyKey, r_rot, omega_rot) {
    // ... (Fonction getGravityLocal) ...
    if (bodyKey === 'ROTATING') {
        const centripetal_accel = r_rot * omega_rot ** 2;
        return centripetal_accel; 
    }
    
    if (alt === null) alt = 0;
    const g_base = CELESTIAL_DATA[bodyKey].G;
    const R_base = CELESTIAL_DATA[bodyKey].R;
    
    return g_base * (R_base / (R_base + alt)) ** 2;
}

function updateCelestialBody(bodyKey, alt, r_rot, omega_rot) {
    // ... (Fonction updateCelestialBody) ...
    let G_ACC_local = 0;
    let R_ALT_CENTER_REF_local = R_E_BASE;

    if (bodyKey === 'ROTATING') {
        G_ACC_local = getGravityLocal(alt, bodyKey, r_rot, omega_rot);
        R_ALT_CENTER_REF_local = R_E_BASE;
    } else {
        const data = CELESTIAL_DATA[bodyKey];
        if (data) {
            G_ACC_local = data.G;
            R_ALT_CENTER_REF_local = data.R;
        }
    }
    
    return { G_ACC: G_ACC_local, R_ALT_CENTER_REF: R_ALT_CENTER_REF_local };
}

function calculateMRF(alt, netherMode) {
    // ... (Fonction calculateMRF) ...
    if (netherMode) {
        return 1.0 / NETHER_RATIO;
    }
    if (alt !== null && alt < ALT_TH) {
        return 0.5;
    }
    return 1.0;
}

function getKalmanR(acc, alt, P_hPa, selectedEnv) {
    // ... (Fonction getKalmanR) ...
    let acc_effective = acc;
    if (acc > MAX_ACC) {
        return 1e9;
    }
    
    let R = acc_effective * acc_effective; 
    
    const envFactor = ENVIRONMENT_FACTORS[selectedEnv]?.R_MULT || 1.0;
    R *= envFactor;
    
    if (P_hPa !== null) {
        // Le facteur de correction pour la pression est important pour la densité de l'air/précision altimétrique
        const pressureFactor = 1.0 + (1013.25 - P_hPa) / 1013.25 * 0.1; 
        R *= Math.max(1.0, pressureFactor); 
    }
    
    if (alt !== null && alt < ALT_TH) { 
        R *= 2.0; // Augmente l'incertitude sous le seuil
    } 

    return Math.max(R_MIN, Math.min(R_MAX, R)); 
}


// --- NOUVELLE FONCTION EKF 15 ÉTATS (MATRICIELLE) ---
function gnssInsEKF_step(dt, imu, gps, stateVector, R_dyn, G_ACC) {
    if (typeof math === 'undefined' || dt === 0 || stateVector.length !== STATE_SIZE) {
        return { stateVector: stateVector, covariance_diag: P_MATRIX ? math.diag(P_MATRIX).toArray() : Array(STATE_SIZE).fill(1000) };
    }
    
    if (P_MATRIX === null) {
        P_MATRIX = math.identity(STATE_SIZE);
        P_MATRIX = P_MATRIX.map((val, index) => val * (index < 9 ? 100 : 1000));
    }

    let X_k = math.matrix(stateVector).resize([STATE_SIZE, 1]); 

    // --- 1. PRÉDICTION (INS) ---
    // Extraction des états P, V, et Biais Acc (B_a)
    const P = X_k.subset(math.index([0, 1, 2], 0));
    const V = X_k.subset(math.index([3, 4, 5], 0));
    const B_a = X_k.subset(math.index([9, 10, 11], 0));
    
    // Accélération corrigée
    const Acc_corr_array = [imu.real_accel_x, imu.real_accel_y, imu.real_accel_z];
    const Acc_corr = math.subtract(math.matrix(Acc_corr_array).resize([3, 1]), B_a); 
    const G_vector = math.matrix([0, 0, -G_ACC]).resize([3, 1]); 
    
    // Nouvelle Vitesse : V_k+1 = V_k + (Acc_corr + G) * dt
    const V_k_plus_1 = math.add(V, math.multiply(math.add(Acc_corr, G_vector), dt));
    
    // Nouvelle Position : P_k+1 = P_k + V_k * dt
    const P_k_plus_1 = math.add(P, math.multiply(V, dt));

    // Création de l'état prédit nominal (X_k+1)
    let X_k_plus_1_nominal = math.clone(X_k);
    X_k_plus_1_nominal.subset(math.index([0, 1, 2], 0), P_k_plus_1);
    X_k_plus_1_nominal.subset(math.index([3, 4, 5], 0), V_k_plus_1);

    // Propagation de la covariance (Noise Injection Simplifiée)
    const Q_MATRIX = math.identity(STATE_SIZE).map(val => val * Q_PROCESS * dt); 
    P_MATRIX = math.add(P_MATRIX, Q_MATRIX); 

    // ------------------------------------
    // 2. CORRECTION (GNSS)
    // ------------------------------------
    let X_k_final = X_k_plus_1_nominal;

    if (gps.lat !== null && R_dyn < R_MAX) { 
        const Y_GPS = math.matrix([gps.lat, gps.lon, gps.alt]).resize([3, 1]);
        const H_of_X = X_k_plus_1_nominal.subset(math.index([0, 1, 2], 0));
        const Z_RESIDUAL = math.subtract(Y_GPS, H_of_X);
        
        // H (3x15) = [ I_3x3 | 0_3x12 ]
        const H_MATRIX = math.matrix([
            [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        ]);
        
        const R_MATRIX = math.identity(3).map(val => val * R_dyn);

        // Calcul du Gain de Kalman (K)
        const H_P = math.multiply(H_MATRIX, P_MATRIX);
        const H_P_H_T = math.multiply(H_P, math.transpose(H_MATRIX));
        const S = math.add(H_P_H_T, R_MATRIX); 
        const K_num = math.multiply(P_MATRIX, math.transpose(H_MATRIX));
        const K_GAIN = math.multiply(K_num, math.inv(S)); 

        // Correction de l'état et de la Covariance
        X_k_final = math.add(X_k_plus_1_nominal, math.multiply(K_GAIN, Z_RESIDUAL));
        const I = math.identity(STATE_SIZE);
        const I_K_H = math.subtract(I, math.multiply(K_GAIN, H_MATRIX));
        P_MATRIX = math.multiply(I_K_H, P_MATRIX);
    }
    
    return { 
        stateVector: X_k_final.toArray().flat(), 
        covariance_diag: math.diag(P_MATRIX).toArray().flat()
    };
                                                 }
// =================================================================
// BLOC 2/3 : astro_weather.js
// Logique de services externes : Météo (API/Cache/ISA/Hyperloop), Temps (NTP local), et Astro (SunCalc).
// =================================================================
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc"; 

// --- VALEURS DE L'ATMOSPHÈRE STANDARD (ISA) à 0m ---
const WEATHER_CACHE_KEY = 'lastWeatherCache'; 
const P_ISA = 1013.25;    // Pression standard (hPa)
const T_ISA_C = 15.0;     // Température standard (°C)
const H_ISA_PERC = 50.0;  // Humidité par défaut (%)

// --- FONCTIONS TEMPS / SYNCHRO NTP (Mode Hors Ligne Prioritaire) ---
// Utilise l'heure locale comme référence principale
let lServH = Date.now(), lLocH = Date.now(); 

function getCDate(lServH, lLocH) {
    if (lServH === null || lLocH === null) { return null; }
    const timeOffset = Date.now() - lLocH;
    return new Date(lServH + timeOffset);
}

// Fonction synch locale pour le démarrage (utilise l'heure du système)
async function syncH() {
    lServH = Date.now();
    lLocH = lServH; 
    
    if (document.getElementById('local-time')) {
        document.getElementById('local-time').textContent = new Date(lServH).toLocaleTimeString('fr-FR') + ' (Local)';
    }
    
    return { lServH, lLocH };
}

// --- FONCTIONS MÉTÉO & ENVIRONNEMENT ---

/** Calcule la densité de l'air (rho) */
function calculateAirDensity(P_hPa, T_K) {
    if (P_hPa === null || T_K === null) return 1.225; 
    const P_Pa = P_hPa * 100; // Pression en Pascals
    return P_Pa / (R_AIR * T_K);
}

/** Calcule le point de rosée */
function calculateDewPoint(T_C, H_perc) {
    if (T_C === null || H_perc === null) return 0.0;
    const a = 17.27;
    const b = 237.7;
    const alpha = (a * T_C) / (b + T_C) + Math.log(H_perc / 100);
    return (b * alpha) / (a - alpha);
}

/** Calcule la vitesse du son (m/s) */
function calculateSpeedOfSound(T_K) {
    if (T_K === null) return 343.0;
    return Math.sqrt(GAMMA * R_AIR * T_K);
}

/** * Récupère la météo, en forçant l'usage d'un modèle (Hyperloop, Isolation, Cache ou ISA).
 */
async function fetchWeather(lat, lon, currentAlt, hyperloopMode) {
    const $ = id => document.getElementById(id); 
    if (!lat || !lon) {
        if ($('weather-status')) $('weather-status').textContent = 'En attente GPS...';
        return null;
    }
    
    let data = null;
    let isOffline = false;
    let statusText = 'ACTIF';
    let sourceTag = '';
    
    // --- LOGIQUE D'EXCLUSION MÉTÉO PAR ORDRE DE PRIORITÉ ---
    const isIsolatedByAltitude = currentAlt !== null && (currentAlt > ALT_HIGH_TH || currentAlt < ALT_TH);

    if (hyperloopMode) { 
        // 1. PRIORITÉ MAXIMALE: Hyperloop (Pression et Température forcées)
        data = {
            tempC: T_HYPERLOOP_C,
            pressure_hPa: P_HYPERLOOP_PA / 100, // Convert Pa to hPa
            humidity_perc: H_HYPERLOOP_PERC,
        };
        isOffline = true;
        statusText = 'FORCÉ (Hyperloop)';
        sourceTag = `(Hyperloop - ${P_HYPERLOOP_PA} Pa)`;

    } else if (isIsolatedByAltitude) {
        // 2. Priorité ISA (Avion / Souterrain)
        data = {
            tempC: T_ISA_C,
            pressure_hPa: P_ISA,
            humidity_perc: H_ISA_PERC,
        };
        isOffline = true;
        statusText = 'FORCÉ (Isolation)';
        sourceTag = `(ISA | Alt: ${currentAlt.toFixed(0)}m)`;

    } else {
        // 3. Priorité API / Cache (Surface normale)
        try {
            const url = `${PROXY_WEATHER_ENDPOINT}?lat=${lat.toFixed(4)}&lon=${lon.toFixed(4)}`;
            const response = await fetch(url);
            
            if (!response.ok) throw new Error("Erreur API Météo");
            
            data = await response.json();
            // Sauvegarde le cache
            localStorage.setItem(WEATHER_CACHE_KEY, JSON.stringify(data));
            sourceTag = ``;

        } catch (error) {
            // API échouée -> Utilisation du cache ou ISA
            const cachedData = localStorage.getItem(WEATHER_CACHE_KEY);
            if (cachedData) {
                data = JSON.parse(cachedData);
                isOffline = true;
                statusText = 'HORS LIGNE (Cache)';
                sourceTag = `(Cache)`;
            } else {
                data = { tempC: T_ISA_C, pressure_hPa: P_ISA, humidity_perc: H_ISA_PERC };
                isOffline = true;
                statusText = 'HORS LIGNE (ISA)';
                sourceTag = `(ISA)`;
            }
        }
    }
    
    // 1. Extraction et Conversion
    const T_C = data.tempC;
    const T_K = T_C + 273.15;
    const P_hPa = data.pressure_hPa;
    const H_perc = data.humidity_perc;

    // 2. Calculs Dérivés
    const air_density = calculateAirDensity(P_hPa, T_K);
    const dew_point = calculateDewPoint(T_C, H_perc);
    const speed_sound = calculateSpeedOfSound(T_K);
    
    // 3. Mise à jour du DOM
    if ($('temp-air')) $('temp-air').textContent = `${T_C.toFixed(1)} °C ${sourceTag}`;
    if ($('pressure-2')) $('pressure-2').textContent = `${P_hPa.toFixed(0)} hPa ${sourceTag}`;
    if ($('humidity-2')) $('humidity-2').textContent = `${H_perc} % ${sourceTag}`;
    if ($('air-density')) $('air-density').textContent = `${air_density.toFixed(3)} kg/m³`;
    if ($('dew-point')) $('dew-point').textContent = `${dew_point.toFixed(1)} °C`;
    if ($('weather-status')) $('weather-status').textContent = statusText;
    if ($('speed-sound')) $('speed-sound').textContent = `${speed_sound.toFixed(2)} m/s`;
    
    return {
        tempC: T_C, tempK: T_K, pressure_hPa: P_hPa, humidity_perc: H_perc,
        air_density: air_density, speed_sound: speed_sound, status: statusText
    };
}


// --- FONCTIONS ASTRO (SunCalc) ---

/** Retourne l'icône (emoji) de la phase lunaire */
function getMoonPhaseIcon(phase) {
    if (phase < 0.03 || phase > 0.97) return "🌑"; 
    if (phase < 0.23) return "🌒"; 
    if (phase < 0.27) return "🌓"; 
    if (phase < 0.48) return "🌔"; 
    if (phase < 0.52) return "🌕"; 
    if (phase < 0.73) return "🌖"; 
    if (phase < 0.77) return "🌗"; 
    return "🌘"; 
}

/** Retourne le nom de la phase lunaire */
function getMoonPhaseName(phase) {
    if (phase < 0.03 || phase > 0.97) return "Nouvelle Lune";
    if (phase < 0.23) return "Premier Croissant";
    if (phase < 0.27) return "Premier Quartier";
    if (phase < 0.48) return "Gibbeuse Croissante";
    if (phase < 0.52) return "Pleine Lune";
    if (phase < 0.73) return "Gibbeuse Décroissante";
    if (phase < 0.77) return "Dernier Quartier";
    return "Dernier Croissant"; 
}

// Fonction utilitaire pour obtenir l'heure solaire (non essentielle pour le moment)
function getSolarTime(now, lonA) { 
    // Simule TST (True Solar Time) : H - EOT + (lon/15) - TZ
    const hour = now.getUTCHours() + now.getUTCMinutes() / 60 + now.getUTCSeconds() / 3600;
    const tst = (hour + lonA / 15 + 24) % 24;
    return tst;
}

/** Fonction principale de mise à jour Astro (appelée par la boucle lente) */ 
function updateAstro(latA, lonA, lServH, lLocH) {
    const $ = id => document.getElementById(id);
    const now = getCDate(lServH, lLocH); 
    
    if (typeof SunCalc === 'undefined' || !latA || !lonA || !now) { 
        $('clock-status').textContent = 'Astro (En attente)'; 
        if ($('date-display')) $('date-display').textContent = 'En attente...';
        return; 
    }

    // --- CALCULS SUNCALC ---
    const sunPos = SunCalc.getPosition(now, latA, lonA); 
    const moonIllum = SunCalc.getMoonIllumination(now); 
    const moonPos = SunCalc.getMoonPosition(now, latA, lonA); 
    const sunTimes = SunCalc.getTimes(now, latA, lonA); 
    const moonTimes = SunCalc.getMoonTimes(now, latA, lonA, true); 
    const solarTimes = getSolarTime(now, lonA); 
    
    // --- MISE À JOUR DU DOM ASTRO ---
    if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
    if ($('date-astro')) $('date-astro').textContent = now.toLocaleDateString('fr-FR');

    if ($('sun-altitude')) $('sun-altitude').textContent = `${(sunPos.altitude * R2D).toFixed(2)} °`; 
    if ($('sun-azimuth')) $('sun-azimuth').textContent = `${(sunPos.azimuth * R2D).toFixed(2)} °`; 
    if ($('day-duration')) $('day-duration').textContent = sunTimes.sunset ? `${((sunTimes.sunset.getTime() - sunTimes.sunrise.getTime()) / 3600000).toFixed(2)} h` : 'N/A';

    // --- PHASE ET LUNE RÉALISTES ---
    const phaseName = getMoonPhaseName(moonIllum.phase);
    const phaseIcon = getMoonPhaseIcon(moonIllum.phase);
    
    if ($('moon-phase-name')) $('moon-phase-name').textContent = `${phaseName} ${phaseIcon}`;
    if ($('moon-illuminated')) $('moon-illuminated').textContent = `${(moonIllum.fraction * 100).toFixed(1)} %`;
    if ($('moon-alt')) $('moon-alt').textContent = `${(moonPos.altitude * R2D).toFixed(2)} °`;
    if ($('moon-azimuth')) $('moon-azimuth').textContent = `${(moonPos.azimuth * R2D).toFixed(2)} °`;
    
    let moonTimesDisplay = 'N/A';
    if (moonTimes.alwaysUp) { moonTimesDisplay = 'Toujours visible'; }
    else if (moonTimes.alwaysDown) { moonTimesDisplay = 'Jamais visible'; }
    else if (moonTimes.rise && moonTimes.set) { 
        moonTimesDisplay = `${moonTimes.rise.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit' })} / ${moonTimes.set.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit' })}`;
    }
    if ($('moon-times')) $('moon-times').textContent = moonTimesDisplay;
    
    // La fonction updateClockVisualization (visuel Soleil/Lune sur l'horloge) doit être définie séparément
    // ou être retirée si vous ne l'utilisez pas dans votre HTML.
    }
// =================================================================
// BLOC 3/3 : app.js (Logique principale, Capteurs, Boucle)
// (CORRIGÉ : Utilise l'EKF 15 États et la Météo Avancée)
// =================================================================

// Alias
const $ = id => document.getElementById(id);
const MIN_DT = 0.01; 
const DOM_SLOW_UPDATE_MS = 1000;
const WEATHER_UPDATE_MS = 30000; // Intervalle de mise à jour Météo

// --- VARIABLES D'ÉTAT (Globales) ---
let wID = null, domID = null, weatherID = null, lPos = null, lat = null, lon = null, sTime = null;
let distM = 0, maxSpd = 0, timeMoving = 0; 

// EKF State Vector (15 états)
let EKF_STATE_VECTOR = Array(STATE_SIZE).fill(0.0);
let EKF_COVARIANCE_DIAG = Array(STATE_SIZE).fill(1000.0); 

let kSpd = 0; 
let kUncert = 1000; 
let kAlt = null; // Altitude EKF LISÉE
let kAltUncert = 10; 

let emergencyStopActive = false;
let netherMode = false; 
let hyperloopMode = false; // NOUVEAU: Mode Hyperloop
let selectedEnvironment = 'NORMAL'; 
let currentMass = 70.0; 
let R_FACTOR_RATIO = 1.0;
let currentCelestialBody = 'EARTH';
let rotationRadius = 100;
let angularVelocity = 0.0; 
let gpsAccuracyOverride = 0.0; 

// Données externes et physiques (MÉTÉO)
let G_ACC = CELESTIAL_DATA['EARTH'].G;
let R_ALT_CENTER_REF = CELESTIAL_DATA['EARTH'].R;
let lastP_hPa = P_ISA;      
let lastT_K = T_ISA_C + 273.15;         
let lastAirDensity = calculateAirDensity(lastP_hPa, lastT_K); 
let lastSpeedOfSound = calculateSpeedOfSound(lastT_K);
let lastH_perc = H_ISA_PERC;

// Variables IMU
let real_accel_x = 0, real_accel_y = 0, real_accel_z = 0;
let imuError = null; 

// --- Fonctions GPS, IMU, Carte (Doivent exister ailleurs dans votre code) ---
// ... (startIMUListeners, startGPS, initMap, updateMap, etc.) ...

// ===========================================
// FONCTION PRINCIPALE DE MISE À JOUR GPS (updateDisp)
// ===========================================

function updateDisp(pos) {
    if (emergencyStopActive) return;
    if (typeof math === 'undefined') {
        if ($('speed-status-text')) $('speed-status-text').textContent = '❌ EKF HORS SERVICE (math.js manquant)';
        return;
    }
    
    // --- 1. ACQUISITION & DT ---
    const cTimePos = pos.timestamp;
    const cLat = pos.coords.latitude;
    const cLon = pos.coords.longitude;
    const altRaw = pos.coords.altitude;
    let accRaw = pos.coords.accuracy;
    
    const now = getCDate(lServH, lLocH); 
    if (now === null) { return; } 
    if (sTime === null) { sTime = now.getTime(); }
    if (gpsAccuracyOverride > 0.0) { accRaw = gpsAccuracyOverride; }
    
    let dt = 0;
    if (lPos) { dt = (cTimePos - lPos.timestamp) / 1000; } 
    else { 
        lPos = pos; lPos.speedMS_3D = 0; lPos.kAlt_old = altRaw; 
        updateMap(cLat, cLon, accRaw); return; 
    }
    if (dt < MIN_DT || dt > 10) { lPos = pos; return; }

    // --- 2. EKF GNSS/INS 15 ÉTATS ---
    
    // Initialisation du vecteur d'état
    if (EKF_STATE_VECTOR.every(val => val === 0.0) && cLat !== null) { 
        EKF_STATE_VECTOR[0] = cLat; EKF_STATE_VECTOR[1] = cLon; EKF_STATE_VECTOR[2] = altRaw;
        EKF_COVARIANCE_DIAG.fill(100.0); 
    }
    
    const gps_meas = { lat: cLat, lon: cLon, alt: altRaw, acc: accRaw };
    const imu_meas = { real_accel_x, real_accel_y, real_accel_z }; 
    let R_dyn = getKalmanR(accRaw, EKF_STATE_VECTOR[2], lastP_hPa, selectedEnvironment); 

    const { stateVector: newState, covariance_diag: newCovariance } = gnssInsEKF_step(
        dt, imu_meas, gps_meas, EKF_STATE_VECTOR, R_dyn, G_ACC
    );
    
    EKF_STATE_VECTOR = newState;
    EKF_COVARIANCE_DIAG = newCovariance;
    
    // --- 3. EXTRACTION ET CALCULS PHYSIQUES ---
    
    lat = EKF_STATE_VECTOR[0];
    lon = EKF_STATE_VECTOR[1];
    kAlt = EKF_STATE_VECTOR[2];
    
    const V_x = EKF_STATE_VECTOR[3];
    const V_y = EKF_STATE_VECTOR[4];
    const V_z = EKF_STATE_VECTOR[5];
    
    const V_ms = Math.sqrt(V_x**2 + V_y**2 + V_z**2);
    kSpd = V_ms; 
    const trace_V_cov = newCovariance[3] + newCovariance[4] + newCovariance[5];
    kUncert = trace_V_cov; 
    kAltUncert = Math.sqrt(newCovariance[2]);
    
    // Calcul de la distance
    const dist2D = dist(lPos.coords.latitude, lPos.coords.longitude, lat, lon, R_ALT_CENTER_REF);
    const dist3D = Math.sqrt(dist2D ** 2 + (kAlt - lPos.kAlt_old || 0) ** 2);
    const spd3D_raw = dist3D / dt; 
    
    let accel_long = 0;
    if (dt > 0.05) { 
        accel_long = (V_ms - lPos.speedMS_3D) / dt; 
        accel_long = (accel_long * 0.5) + (real_accel_x * 0.5); 
    }

    R_FACTOR_RATIO = calculateMRF(kAlt, netherMode); 
    distM += V_ms * dt * R_FACTOR_RATIO; 
    
    if (V_ms > MIN_SPD) { timeMoving += dt; }
    if (V_ms > maxSpd) maxSpd = V_ms; 
    
    const local_g = getGravityLocal(kAlt, currentCelestialBody, rotationRadius, angularVelocity); 
    const coriolis_force = 2 * currentMass * V_ms * OMEGA_EARTH * Math.sin(lat * D2R);

    // --- CORRECTION FINALE : CALCULS DE LA DYNAMIQUE (Drag, Mach) ---
    const currentAirDensity = lastAirDensity; // Utilise la densité calculée par fetchWeather
    const currentSpeedOfSound = lastSpeedOfSound; // Utilise la vitesse du son calculée par fetchWeather
    
    const dynamic_pressure_q = 0.5 * currentAirDensity * V_ms**2;
    const mach_number = V_ms / currentSpeedOfSound;
    
    // Coeff. de Traînée (Cd=1.2) et Surface Frontale (A=0.5m²) - Valeurs par défaut
    const Cd = 1.2; 
    const A_front = 0.5; 
    const drag_force = dynamic_pressure_q * Cd * A_front;
    
    // Mise à jour du statut EKF
    const accEKF_m = Math.sqrt(newCovariance[0] + newCovariance[1] + newCovariance[2]);
    let modeStatus = '';
    if (accEKF_m < 0.5) modeStatus = `🚀 FUSION TOTALE (INS Dominant)`; 
    else if (accEKF_m < 4.0) modeStatus = `🏡 FUSION MOYENNE (Équilibré)`; 
    else modeStatus = `🛰️ GPS/IMU Initialisation...`; 


    // --- 4. MISE À JOUR DU DOM ---
    
    // Vitesse & Distance
    if ($('speed-stable')) $('speed-stable').textContent = `${(V_ms * KMH_MS).toFixed(2)}`;
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${V_ms.toFixed(3)} m/s`;
    if ($('distance-total-km')) $('distance-total-km').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    
    // Dynamique & Forces
    if ($('%-speed-sound')) $('%-speed-sound').textContent = `${(V_ms / currentSpeedOfSound * 100).toFixed(2)} %`;
    if ($('mach-number')) $('mach-number').textContent = `${mach_number.toFixed(4)}`;
    if ($('dynamic-pressure')) $('dynamic-pressure').textContent = `${dynamic_pressure_q.toFixed(2)} Pa`;
    if ($('drag-force')) $('drag-force').textContent = `${drag_force.toFixed(2)} N`;
    if ($('accel-long')) $('accel-long').textContent = `${accel_long.toFixed(3)} m/s²`;
    if ($('coriolis-force')) $('coriolis-force').textContent = `${coriolis_force.toExponential(2)} N`;

    // EKF & Debug
    if ($('speed-status-text')) $('speed-status-text').textContent = modeStatus;
    if ($('kalman-uncert')) $('kalman-uncert').textContent = `${kUncert.toFixed(3)} m²/s² (P - V_3D)`;
    if ($('alt-uncertainty')) $('alt-uncertainty').textContent = `${kAltUncert.toFixed(3)} m (σ - Alt)`; 
    if ($('gps-status-dr')) { 
        $('gps-status-dr').textContent = `Souterrain: ${kAlt !== null && kAlt < ALT_TH ? 'OUI' : 'Non'} (${modeStatus} | Acc: ${accRaw.toFixed(1)}m | EKF Acc: ${accEKF_m.toFixed(2)}m)`;
    }

    // Position
    if ($('lat-display')) $('lat-display').textContent = `${lat.toFixed(6)} °`;
    if ($('lon-display')) $('lon-display').textContent = `${lon.toFixed(6)} °`;
    if ($('alt-display')) $('alt-display').textContent = kAlt !== null ? `${kAlt.toFixed(2)} m` : 'N/A';
    
    // --- 5. SAUVEGARDE & MISE À JOUR CARTE ---
    lPos = pos; 
    lPos.speedMS_3D = V_ms; 
    lPos.timestamp = cTimePos; 
    lPos.kAlt_old = kAlt; 

    updateMap(lat, lon, accRaw);
}

// ... (Les fonctions startIMUListeners, startGPS, initMap, updateMap, etc. doivent être définies ici) ...


document.addEventListener('DOMContentLoaded', () => {
    
    // ... (Initialisation de la carte, des contrôles mass/rotation/celestial, etc.) ...
    
    // --- NOUVEAU: GESTIONNAIRE HYPERLOOP ---
    const hyperloopToggle = $('hyperloop-toggle-btn');
    if (hyperloopToggle) {
        hyperloopToggle.addEventListener('click', () => {
            hyperloopMode = !hyperloopMode;
            hyperloopToggle.textContent = hyperloopMode ? 'HYPERLOOP ACTIF' : 'HYPERLOOP INACTIF';
            hyperloopToggle.classList.toggle('active', hyperloopMode);
            
            // Forcer une mise à jour météo immédiate pour changer les constantes physiques
            if (lat && lon) {
                fetchWeather(lat, lon, kAlt, hyperloopMode); 
            }
        });
    }

    // --- DÉMARRAGE DU SYSTÈME (Fonction corrigée pour les boucles) ---
    const startButton = $('init-system-btn');
    if (startButton) {
        startButton.addEventListener('click', () => {
            
            // 1. Initialise les valeurs de gravité
            const initVals = updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity);
            G_ACC = initVals.G_ACC;
            R_ALT_CENTER_REF = initVals.R_ALT_CENTER_REF;
            
            // 2. Démarre la synchro NTP locale, GPS et IMU
            syncH().then(newTimes => {
                lServH = newTimes.lServH;
                lLocH = newTimes.lLocH;
                startGPS(); 
                startIMUListeners(); 
            });
            
            // 3. Cache le bouton de démarrage
            startButton.style.display = 'none';
            
            // 4. Démarre la boucle lente (Astro/Temps)
            if (domID === null) {
                domID = setInterval(() => {
                    const currentLat = lat || 43.284; 
                    const currentLon = lon || 5.358;
                    
                    // Mise à jour Astro et Heure (à chaque seconde)
                    updateAstro(currentLat, currentLon, lServH, lLocH);
                    const now = getCDate(lServH, lLocH);
                    if (now) {
                        if ($('local-time') && !$('local-time').textContent.includes('SYNCHRO ÉCHOUÉE')) {
                            $('local-time').textContent = now.toLocaleTimeString('fr-FR');
                        }
                    }
                    
                }, DOM_SLOW_UPDATE_MS); 
            }
            
            // 5. Démarre l'intervalle Météo (toutes les 30s)
            if (weatherID === null) {
                weatherID = setInterval(async () => {
                    const currentLat = lat || 43.284; 
                    const currentLon = lon || 5.358;
                    const currentAlt = kAlt; 
                    
                    if (currentLat && currentLon && !emergencyStopActive) {
                        // PASSAGE DE L'ALTITUDE EKF ET DU MODE HYPERLOOP
                        const weatherData = await fetchWeather(currentLat, currentLon, currentAlt, hyperloopMode); 
                        
                        if (weatherData) {
                            // Sauvegarde les résultats Météo pour l'EKF et les calculs physiques
                            lastP_hPa = weatherData.pressure_hPa;
                            lastT_K = weatherData.tempK;
                            lastAirDensity = weatherData.air_density;
                            lastSpeedOfSound = weatherData.speed_sound;
                            lastH_perc = weatherData.humidity_perc;
                        }
                    }
                }, WEATHER_UPDATE_MS); 
                
                // Exécute la première fois immédiatement (avec kAlt initialement null)
                fetchWeather(lat || 43.284, lon || 5.358, kAlt, hyperloopMode).then(weatherData => {
                     if (weatherData) {
                        lastP_hPa = weatherData.pressure_hPa;
                        lastT_K = weatherData.tempK;
                        lastAirDensity = weatherData.air_density;
                        lastSpeedOfSound = weatherData.speed_sound;
                        lastH_perc = weatherData.humidity_perc;
                    }
                });
            }
            
        }, { once: true }); 
    }
});
