// =================================================================
// BLOC 1/4 : Constantes, État Global & Filtres EKF (Core Math)
// =================================================================

// --- CLÉS D'API & PROXY VERCEL ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";
const DOM_SLOW_UPDATE_MS = 2000; 

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES FONDAMENTALES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;          // Vitesse de la lumière (m/s)
const C_S_STD = 343;            // Vitesse du son standard (m/s)
const G_U = 6.67430e-11;        // Constante gravitationnelle universelle (N·m²/kg²)
const R_SPECIFIC_AIR = 287.058; // Constante spécifique de l'air sec (J/kg·K)
const GAMMA_AIR = 1.4;          // Indice adiabatique de l'air
const MU_DYNAMIC_AIR = 1.8e-5;  // Viscosité dynamique de l'air (Pa·s)
const R_E_BASE = 6371000;       // Rayon terrestre moyen (m)
const KMH_MS = 3.6;             // Conversion m/s vers km/h
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const C_DRAG_AREA = 0.5;        // CdA: Coefficient de traînée * Surface (valeur par défaut)
const SAT_O2_SEA_LEVEL = 97.5;  // Saturation O2 théorique au niveau de la mer (%)
const AU_METERS = 149597870700; // Unité Astronomique (m)
const LIGHT_YEAR_METERS = 9.461e15; // Année Lumière (m)
const NETHER_RATIO = 8;         // Ratio Minecraft (Nether)

// --- PARAMÈTRES DU FILTRE DE KALMAN (VITESSE/ALTITUDE) ---
const Q_NOISE = 0.1;        // Bruit de processus
const R_MIN = 0.01;         // Bruit de mesure minimum
const R_MAX = 500.0;        // Bruit de mesure maximum
const MAX_ACC = 200;        // Précision max (m) avant "Estimation Seule"
const MIN_SPD = 0.05;       // Vitesse minimale "en mouvement"
const ALT_TH = -50;         // Seuil d'altitude "Sous-sol"
const MIN_DT = 0.01;        // Temps minimum entre updates
const R_ALT_MIN = 0.1;      // R min pour l'altitude

// --- DONNÉES CÉLESTES/GRAVITÉ ---
const CELESTIAL_DATA = {
    'EARTH': { G: 9.80665, R: R_E_BASE, name: 'Terre' },
    'MOON': { G: 1.62, R: 1737400, name: 'Lune' },
    'MARS': { G: 3.71, R: 3389500, name: 'Mars' },
    'ROTATING': { G: 0.0, R: R_E_BASE, name: 'Station Spatiale' }
};

// --- FACTEURS ENVIRONNEMENTAUX ---
const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0, DISPLAY: 'Normal' },
    'FOREST': { R_MULT: 2.5, DISPLAY: 'Forêt' },
    'CONCRETE': { R_MULT: 7.0, DISPLAY: 'Grotte/Tunnel' },
    'METAL': { R_MULT: 5.0, DISPLAY: 'Métal/Bâtiment' },
};

// --- VARIABLES D'ÉTAT (Globales) ---
let wID = null, domID = null, lPos = null;
let lat = null, lon = null, sTime = null;
let distM = 0, maxSpd = 0;
let kSpd = 0, kUncert = 1000; 
let timeMoving = 0; 
let lServH = null, lLocH = null; 
let lastFSpeed = 0; 
let kAlt = null;      
let kAltUncert = 10;  
let emergencyStopActive = false;
let selectedEnvironment = 'NORMAL'; 
let currentMass = 70.0; 
let currentCdA = C_DRAG_AREA; 
let currentCelestialBody = 'EARTH';
let rotationRadius = 100;
let angularVelocity = 0.0; 
let G_ACC = CELESTIAL_DATA['EARTH'].G;
let lastWeatherData = null; 
let sunAltitudeRad = 0;
let real_accel_x = 0, real_accel_y = 0, real_accel_z = 0;
let map, marker, circle;
let isNetherMode = false;
let GPS_ACCURACY_OVERRIDE = 0;

// --- FONCTION UTILITAIRE DOM ---
const $ = id => document.getElementById(id);

/** Calcule la distance de Haversine en mètres */
const dist = (lat1, lon1, lat2, lon2, R_ref) => {
    const R = R_ref || R_E_BASE; 
    const dLat = (lat2 - lat1) * D2R, dLon = (lon2 - lon1) * D2R;
    lat1 *= D2R; lat2 *= D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R * c;
};

/** Calcule l'accélération gravitationnelle locale ou artificielle. */
function getGravityLocal(alt, bodyKey, r_rot, omega_rot) {
    if (bodyKey === 'ROTATING') {
        return r_rot * omega_rot ** 2; // Accélération centripète artificielle
    }
    if (alt === null) alt = 0;
    const g_base = CELESTIAL_DATA[bodyKey].G;
    const R_base = CELESTIAL_DATA[bodyKey].R;
    return g_base * (R_base / (R_base + alt)) ** 2;
}

/** Filtre de Kalman 1D pour la vitesse. */
function kFilter(kSpd_in, kUncert_in, nSpd, dt, R_dyn, accel_input = 0) {
    if (dt === 0 || dt > 5) return { kSpd: kSpd_in, kUncert: kUncert_in }; 
    const R = R_dyn;
    const Q = Q_NOISE * dt * dt; 

    // PRÉDICTION
    let pSpd = kSpd_in + (accel_input * dt); 
    let pUnc = kUncert_in + Q; 

    // CORRECTION
    let K = pUnc / (pUnc + R); 
    let kSpd = pSpd + K * (nSpd - pSpd); 
    let kUncert = (1 - K) * pUnc; 
    
    return { kSpd, kUncert };
}

/** Applique le filtre de Kalman à l'Altitude. */
function kFilterAltitude(kAlt_in, kAltUncert_in, nAlt, acc, dt) {
    if (nAlt === null) return { kAlt: kAlt_in, kAltUncert: kAltUncert_in };
    const R_alt = Math.max(R_ALT_MIN, acc * acc); 
    const Q_alt = Q_NOISE * dt; 
    
    let pAlt = kAlt_in === null ? nAlt : kAlt_in; 
    let pAltUncert = kAlt_in === null ? 1000 : kAltUncert_in + Q_alt;
    
    let K_alt = pAltUncert / (pAltUncert + R_alt);
    let kAlt = pAlt + K_alt * (nAlt - pAlt);
    let kAltUncert = (1 - K_alt) * pAltUncert;
    
    return { kAlt, kAltUncert };
}

/** Calcule le Facteur R (Confiance GPS) du filtre de Kalman. */
function getKalmanR(acc, alt, P_hPa, selectedEnv) {
    let acc_effective = GPS_ACCURACY_OVERRIDE > 0 ? GPS_ACCURACY_OVERRIDE : acc;
    if (acc_effective > MAX_ACC) { return 1e9; }
    
    let R = acc_effective * acc_effective; 
    const envFactor = ENVIRONMENT_FACTORS[selectedEnv]?.R_MULT || 1.0;
    R *= envFactor;
    
    if (P_hPa !== null) {
        const pressureFactor = 1.0 + (1013.25 - P_hPa) / 1013.25 * 0.1;
        R *= Math.max(1.0, pressureFactor); 
    }
    
    if (alt !== null && alt < ALT_TH) { 
        R *= 2.0; 
    } 

    return Math.max(R_MIN, Math.min(R_MAX, R)); 
}
// =================================================================
// BLOC 2/4 : Modèles de Physique Avancée et Biophysique
// =================================================================

/**
 * Calcule les grandeurs de Physique Avancée, Relativité et Fluides.
 */
function calculateAdvancedPhysics(kSpd, kAlt, mass, CdA, tempK, airDensity, lat, kAltUncert, localG, lastFSpeed, pos, lPos) {
    const V = kSpd;
    const alt = kAlt;

    // --- 1. Relativité et Énergie
    const lorentzFactor = 1 / Math.sqrt(1 - Math.pow(V / C_L, 2));
    const timeDilationSpeed = (lorentzFactor - 1) * 86400 * 1e9; // ns/jour
    const E0 = mass * C_L * C_L;
    const energyRelativistic = lorentzFactor * E0;
    const momentum = mass * V; // Quantité de mouvement

    // Correction de la Dilatation Temporelle Gravitationnelle (Relativité Générale)
    const DILATION_FACTOR = 1e-18; 
    const gravitationalDilation = alt * DILATION_FACTOR * 86400 * 1e9; // ns/jour

    // Rayon de Schwarzschild (pour la masse de l'objet)
    const Rs_object = (2 * G_U * mass) / (C_L * C_L);
    
    // --- 2. Mécanique des Fluides
    const speedOfSoundLocal = Math.sqrt(GAMMA_AIR * R_SPECIFIC_AIR * tempK); 
    const machNumber = V / speedOfSoundLocal;
    const dynamicPressure = 0.5 * airDensity * V * V; // q = 1/2 ρV²
    const viscosityCinematic = MU_DYNAMIC_AIR / airDensity;
    // Simplification du nombre de Reynolds (L=1m pour objet standard)
    const reynoldsNumber = (airDensity * V * 1) / MU_DYNAMIC_AIR; 
    const dragForce = dynamicPressure * CdA; // F_D = q * CdA

    // --- 3. Géophysique
    // Force de Coriolis
    const coriolisForce = 2 * mass * V * OMEGA_EARTH * Math.sin(lat * D2R);
    
    // Altitude Géopotentielle
    const G_ACC_ref = 9.80665;
    const geopotentialAltitude = alt * (G_ACC_ref / localG);
    
    // Force G (Longitudinale)
    const accel_long = (V - lastFSpeed) / (lPos ? (pos.timestamp - lPos.timestamp) / 1000 : 1);
    const force_g_long = accel_long / localG;

    // --- 4. EKF / Métrologie Avancée
    const EKF_UPDATE_RATE = 1000 / (lPos ? (pos.timestamp - lPos.timestamp) : 100); 
    const nyquistFrequency = 0.5 * EKF_UPDATE_RATE; 
    const altSigma = Math.sqrt(kAltUncert);

    return { 
        lorentzFactor, timeDilationSpeed, energyRelativistic, E0, momentum, gravitationalDilation, Rs_object,
        speedOfSoundLocal, machNumber, dynamicPressure, reynoldsNumber, viscosityCinematic, dragForce,
        coriolisForce, geopotentialAltitude, force_g_long, accel_long,
        nyquistFrequency, altSigma
    };
}

/**
 * Calcule les grandeurs de Bio-Physique et de Stress.
 */
function calculateBioSVT(tempC, alt, humidity_perc, pressurePa, sunAltitudeRad) {
    // 1. Point de Rosée (pour l'isolation)
    const a = 17.27, b = 237.7;
    const h_frac = humidity_perc / 100.0;
    const f = (a * tempC) / (b + tempC) + Math.log(h_frac);
    const dewPoint = (b * f) / (a - f);

    // 2. Température du Thermomètre Mouillé (Tw) - Formule empirique
    const wetBulbTemp = tempC * Math.atan(0.151977 * Math.sqrt(h_frac + 8.313659)) + 
                        Math.atan(tempC + h_frac) - 
                        Math.atan(h_frac - 1.67633) + 
                        0.00391838 * Math.pow(h_frac, 1.5) * Math.atan(0.023101 * h_frac) - 4.686035;

    // 3. CAPE (Convective Available Potential Energy) - Simplification
    const CAPE_sim = Math.max(0, 0.5 * (tempC - dewPoint) * 500); 

    // 4. Saturation en Oxygène (SVT) - Modèle d'altitude
    const O2Saturation = SAT_O2_SEA_LEVEL - (alt / 1000) * 2; 
    const O2SaturationClamped = Math.max(70, Math.min(100, O2Saturation));

    // 5. Taux de Photosynthèse (SVT) - Modèle simplifié
    const sunAngleFactor = Math.max(0, sunAltitudeRad * R2D) / 90; 
    const tempFactor = Math.exp(-0.5 * Math.pow((tempC - 25) / 10, 2)); 
    const photosynthesisRate = 0.05 * sunAngleFactor * tempFactor; 
    
    // 6. Rayonnement Solaire (Irradiance)
    const solarIrradiance = 1000 * Math.max(0, Math.sin(sunAltitudeRad)); 
    const radiationPressure = solarIrradiance / C_L; // Pression de radiation

    return { 
        dewPoint, wetBulbTemp, CAPE_sim, O2SaturationClamped, photosynthesisRate, 
        solarIrradiance, radiationPressure
    };
                                                   }
// =================================================================
// BLOC 3/4 : Services Externes, Météo et Modèles Astronomiques
// =================================================================

/** Synchronise l'horloge interne avec un serveur de temps (UTC/Atomique) */
async function syncH(lServH_in, lLocH_in) {
    let lServH = lServH_in;
    let lLocH = lLocH_in;
    const localStartPerformance = performance.now(); 

    try {
        const response = await fetch(SERVER_TIME_ENDPOINT, { cache: "no-store", mode: "cors" });
        if (!response.ok) throw new Error(`Server time sync failed: ${response.statusText}`);
        
        const localEndPerformance = performance.now(); 
        const serverData = await response.json(); 
        const utcTimeISO = serverData.utc_datetime; 
        const serverTimestamp = Date.parse(utcTimeISO); 
        const RTT = localEndPerformance - localStartPerformance;
        const latencyOffset = RTT / 2;

        lServH = serverTimestamp + latencyOffset; 
        lLocH = performance.now(); 
        
    } catch (error) {
        lServH = Date.now(); 
        lLocH = performance.now();
        if ($('local-time')) $('local-time').textContent = 'SYNCHRO ÉCHOUÉE (Local)';
        console.warn("Synchronisation NTP échouée:", error);
    }
    return { lServH, lLocH };
}

/** Retourne l'heure synchronisée. */
function getCDate(lServH, lLocH) { 
    if (lServH === null || lLocH === null) { return null; }
    const offsetSinceSync = performance.now() - lLocH;
    return new Date(lServH + offsetSinceSync); 
}

/** Récupère et traite les données météo via l'API */
async function fetchWeather(latA, lonA) {
    if (!latA || !lonA) return null; 
    
    const apiUrl = `${PROXY_WEATHER_ENDPOINT}?lat=${latA}&lon=${lonA}`;
    let weatherData = null;
    
    try {
        const response = await fetch(apiUrl);
        if (!response.ok) throw new Error(`Erreur HTTP: ${response.status}`);
        const data = await response.json();
        
        if (data.main) {
            const tempC = data.main.temp;
            const pressure_hPa = data.main.pressure;
            const humidity_perc = data.main.humidity;
            const tempK = tempC + 273.15;
            
            const pressure_pa = pressure_hPa * 100;
            const air_density = pressure_pa / (R_SPECIFIC_AIR * tempK);
            
            const P_sat_hPa = tC => 6.1078 * Math.pow(10, (7.5 * tC) / (237.3 + tC));
            const P_v = P_sat_hPa(tempC) * (humidity_perc / 100.0); 
            const absoluteHumidity = (P_v * 100 * 18.015) / (8.314 * tempK) * 1000; // g/m³
            
            weatherData = {
                tempC: tempC,
                pressure_hPa: pressure_hPa,
                humidity_perc: humidity_perc,
                tempK: tempK,
                air_density: air_density,
                absoluteHumidity: absoluteHumidity
            };
        } else {
             throw new Error(data.message || 'Données météo incomplètes');
        }
    } catch (err) {
        console.warn("Erreur de récupération météo:", err.message);
    }
    return weatherData; 
}


// --- FONCTIONS ASTRO (SUNCALC & Custom) ---
const dayMs = 1000 * 60 * 60 * 24;
const MC_DAY_MS = 72 * 60 * 1000;
const J1970 = 2440588, J2000 = 2451545;

function toDays(date) { return (date.valueOf() / dayMs - 0.5 + J1970) - J2000; }
function solarMeanAnomaly(d) { return D2R * (356.0470 + 0.9856002585 * d); }
function eclipticLongitude(M) {
    var C = D2R * (1.9148 * Math.sin(M) + 0.0200 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M)), 
        P = D2R * 102.9377;                                                                
    return M + C + P + Math.PI;
}

/** Calcule le Temps Solaire Vrai (TST). */
function getSolarTime(date, lon) {
    if (date === null || lon === null || isNaN(lon)) return { TST: 'N/A', MST: 'N/A', EOT: 'N/D', ECL_LONG: 'N/D' };
    
    const d = toDays(date);
    const M = solarMeanAnomaly(d); 
    const L = eclipticLongitude(M); 
    
    const J_star = toDays(date) - lon / 360;
    const J_transit = J_star + (0.0053 * Math.sin(M) - 0.0069 * Math.sin(2 * L));
    const eot_min = (J_star - J_transit) * 1440; 

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
        NoonSolar: toTimeString(J_transit * dayMs)
    };
}

/** Calcule le temps Minecraft. */
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

/** Calcule le Temps Sidéral Local Vrai (TSLV) (Simplification pour affichage) */
function getTSLV(date, lon) {
    if (date === null) return 'N/A';
    // Approximation du TSLV en heures
    const GMST = (date.getUTCHours() + date.getUTCMinutes() / 60) * 15; // GMST en degrés (très simplifié)
    const LST = GMST + lon;
    const LST_h = (LST / 15 + 24) % 24;
    return LST_h.toFixed(2) + ' h';
}

function getMoonPhaseName(fraction) {
    if (fraction === 0) return "Nouvelle Lune";
    if (fraction > 0 && fraction < 0.25) return "Croissant Montant";
    if (fraction === 0.25) return "Premier Quartier";
    if (fraction > 0.25 && fraction < 0.5) return "Gibbeuse Montante";
    if (fraction === 0.5) return "Pleine Lune";
    if (fraction > 0.5 && fraction < 0.75) return "Gibbeuse Décroissante";
    if (fraction === 0.75) return "Dernier Quartier";
    if (fraction > 0.75 && fraction < 1) return "Croissant Décroissant";
    return "N/A";
}

/** Fonction principale de mise à jour Astro (appelée par la boucle lente) */
function updateAstro(latA, lonA, lServH, lLocH) {
    const now = getCDate(lServH, lLocH); 
    
    if (now === null) { return { sunAltitudeRad: 0 }; }
    
    if ($('time-minecraft')) $('time-minecraft').textContent = getMinecraftTime(now);

    if (typeof SunCalc === 'undefined' || !latA || !lonA) {
        if ($('clock-status')) $('clock-status').textContent = 'Astro (Attente GPS)...';
        return { sunAltitudeRad: 0 };
    }
    
    const sunPos = SunCalc.getPosition(now, latA, lonA);
    const moonIllum = SunCalc.getMoonIllumination(now);
    const solarTimes = getSolarTime(now, lonA);
    const times = SunCalc.getTimes(now, latA, lonA);
    let sunAltitudeRad = sunPos.altitude;
    
    // Mise à jour des données Astro
    if ($('tst')) $('tst').textContent = solarTimes.TST;
    if ($('noon-solar')) $('noon-solar').textContent = solarTimes.NoonSolar;
    if ($('eot')) $('eot').textContent = solarTimes.EOT + ' min';
    if ($('tslv')) $('tslv').textContent = getTSLV(now, lonA);
    if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(moonIllum.fraction);
    
    // Mise à jour de l'horloge visuelle
    const hour = now.getHours() + now.getMinutes() / 60 + now.getSeconds() / 3600;
    const dayRatio = (hour + lonA / 15) / 24;
    const sunAngle = (dayRatio * 360 - 90) * D2R; 
    if ($('sun-element')) $('sun-element').style.transform = `rotate(${sunAngle}rad)`;
    if ($('moon-element')) $('moon-element').style.transform = `rotate(${sunAngle + Math.PI}rad)`;

    // Détermination du statut du ciel
    let skyClass = 'sky-night';
    if (times.sunsetEnd && times.sunrise) {
        if (now >= times.sunrise && now < times.sunset) { skyClass = 'sky-day'; }
        else if (now >= times.sunset && now < times.dusk) { skyClass = 'sky-sunset'; }
        else if (now >= times.dawn && now < times.sunrise) { skyClass = 'sky-sunset'; }
        else if (now >= times.dusk || now < times.dawn) { skyClass = 'sky-night'; }
    }
    
    document.body.className = document.body.className.split(' ').filter(c => !c.startsWith('sky-')).join(' ');
    document.body.classList.add(skyClass);
    if ($('minecraft-clock')) $('minecraft-clock').className = skyClass;

    if ($('clock-status')) $('clock-status').textContent = sunPos && sunPos.altitude > 0 ? 'Jour Solaire (☀️)' : 'Nuit/Crépuscule (🌙)';

    return { sunAltitudeRad };
            }
// =================================================================
// BLOC 4/4 : Logique Applicative, GPS/IMU et Initialisation
// =================================================================

// --- GESTION DES CAPTEURS IMU ---
function imuMotionHandler(event) {
    const accData = event.acceleration || event.accelerationIncludingGravity;

    if (accData) {
        real_accel_x = accData.x || 0;
        real_accel_y = accData.y || 0;
        real_accel_z = accData.z || 0;
        if ($('imu-accel-x')) $('imu-accel-x').textContent = real_accel_x.toFixed(2) + ' m/s²';
        if ($('imu-accel-y')) $('imu-accel-y').textContent = real_accel_y.toFixed(2) + ' m/s²';
        if ($('imu-accel-z')) $('imu-accel-z').textContent = real_accel_z.toFixed(2) + ' m/s²';
        if ($('imu-status')) $('imu-status').textContent = "Actif";
    } else {
        if ($('imu-status')) $('imu-status').textContent = "Erreur (Capteur N/A)";
    }
}

function startIMUListeners() {
    if (window.DeviceOrientationEvent) {
        window.addEventListener('devicemotion', imuMotionHandler, true);
    }
}

function stopIMUListeners() {
    window.removeEventListener('devicemotion', imuMotionHandler, true);
    if ($('imu-status')) $('imu-status').textContent = 'Inactif';
}

// --- Fonctions Carte ---
function initMap() {
    if (typeof L === 'undefined') {
        if ($('map')) $('map').textContent = "Erreur: Bibliothèque Leaflet non chargée.";
        return;
    }
    if ($('map')) {
        map = L.map('map').setView([48.8566, 2.3522], 13);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            maxZoom: 19,
            attribution: '© OpenStreetMap'
        }).addTo(map);
        marker = L.marker([0, 0]).addTo(map);
        circle = L.circle([0, 0], { radius: 10 }).addTo(map);
        map.lastMoveTime = Date.now();
        map.on('movestart', () => map.lastMoveTime = Date.now());
    }
}

function updateMap(lat, lon, acc) {
    if (marker && circle) {
        const newLatLng = L.latLng(lat, lon);
        marker.setLatLng(newLatLng);
        circle.setLatLng(newLatLng).setRadius(acc || 10);
        
        if (Date.now() - map.lastMoveTime > 3000) {
            map.panTo(newLatLng);
        }
    }
}

// --- FONCTIONS DE CONTRÔLE GPS ---

function startGPS() {
    stopGPS(false);
    startIMUListeners();

    if ($('gps-status-dr')) $('gps-status-dr').textContent = 'Démarrage...';

    wID = navigator.geolocation.watchPosition(pos => {
        if (emergencyStopActive) return;
        updateDisp(pos);
    }, handleErr, { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 });

    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = "⏸️ PAUSE GPS";
    if ($('emergency-stop-btn')) $('emergency-stop-btn').classList.remove('active');
}

function stopGPS(resetButton = true) {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    wID = null;
    stopIMUListeners();
    
    if ($('gps-status-dr')) $('gps-status-dr').textContent = 'Arrêté';
    if (resetButton && $('toggle-gps-btn')) $('toggle-gps-btn').textContent = "▶️ MARCHE GPS";
}

function emergencyStop() {
    emergencyStopActive = true;
    stopGPS(false);
    if ($('emergency-stop-btn')) $('emergency-stop-btn').textContent = '🛑 Arrêt d\'urgence: ACTIF 🔴';
    if ($('emergency-stop-btn')) $('emergency-stop-btn').classList.add('active');
    if ($('toggle-gps-btn')) $('toggle-gps-btn').disabled = true;
    if ($('gps-status-dr')) $('gps-status-dr').textContent = 'URGENCE (Mouvement Stoppé)';
}

function resumeSystem() {
    emergencyStopActive = false;
    if ($('emergency-stop-btn')) $('emergency-stop-btn').textContent = '🛑 Arrêt d\'urgence: INACTIF 🟢';
    if ($('emergency-stop-btn')) $('emergency-stop-btn').classList.remove('active');
    if ($('toggle-gps-btn')) $('toggle-gps-btn').disabled = false;
    startGPS();
}

function handleErr(err) {
    let msg;
    switch (err.code) {
        case err.PERMISSION_DENIED: msg = "Permission GPS refusée."; break;
        case err.POSITION_UNAVAILABLE: msg = "Position indisponible."; break;
        case err.TIMEOUT: msg = "Timeout GPS."; break;
        default: msg = "Erreur GPS inconnue: " + err.message;
    }
    if ($('gps-status-dr')) $('gps-status-dr').textContent = 'ERREUR: ' + msg;
    console.error(msg);
}

function resetDistance() {
    distM = 0;
    timeMoving = 0;
    if ($('distance-total-km')) $('distance-total-km').textContent = "0.000 km | 0.00 m";
    if ($('time-moving')) $('time-moving').textContent = "0.00 s";
}

function resetMaxSpeed() {
    maxSpd = 0;
    if ($('speed-max')) $('speed-max').textContent = "0.0 km/h";
}

function resetAll() {
    resetDistance();
    resetMaxSpeed();
    kSpd = 0; kUncert = 1000;
    kAlt = null; kAltUncert = 10;
    lastFSpeed = 0;
    lat = null; lon = null; lPos = null;
    if ($('gps-status-dr')) $('gps-status-dr').textContent = 'Réinitialisé';
}


// ===========================================
// FONCTION PRINCIPALE DE MISE À JOUR (updateDisp)
// ===========================================

function updateDisp(pos) {
    // --- 1. ACQUISITION DES DONNÉES & FILTRES ---
    const cTimePos = pos.timestamp;
    let cLat = pos.coords.latitude;
    let cLon = pos.coords.longitude;
    let altRaw = pos.coords.altitude || 0.0;
    let accRaw = pos.coords.accuracy || 10.0;
    let spdRaw = pos.coords.speed || 0.0;
    const now = getCDate(lServH, lLocH); 
    if (now === null) return; 
    if (sTime === null) { sTime = now.getTime(); }

    // Règle d'application du mode Nether
    const DISTANCE_RATIO = isNetherMode ? NETHER_RATIO : 1; 

    let isSignalLost = (accRaw > MAX_ACC);
    let R_dyn = getKalmanR(accRaw, kAlt, lastWeatherData ? lastWeatherData.pressure_hPa : null, selectedEnvironment); 
    
    if (!lPos) { // Initialisation
        lPos = pos; kAlt = altRaw; 
        lat = cLat; lon = cLon;
        updateMap(cLat, cLon, accRaw);
        if ($('gps-status-dr')) $('gps-status-dr').textContent = 'Actif';
        return;
    }

    if (isSignalLost) { 
        cLat = lat; cLon = lon; altRaw = kAlt; 
        if ($('underground-status')) $('underground-status').textContent = 'Perte de signal (Drift/Estimation)';
    } else {
        lat = cLat; lon = cLon;
        if ($('underground-status')) $('underground-status').textContent = kAlt < ALT_TH ? 'Sous-sol (signal faible)' : 'Signal stable';
    }
    
    let dt = (cTimePos - lPos.timestamp) / 1000;
    if (dt < MIN_DT || dt > 10) { lPos = pos; return; }

    // EKF ALTITUDE
    const { kAlt: kAlt_new, kAltUncert: kAltUncert_new } = kFilterAltitude(kAlt, kAltUncert, altRaw, pos.coords.altitudeAccuracy || R_ALT_MIN, dt);
    kAlt = kAlt_new;
    kAltUncert = kAltUncert_new;
    
    // ZUPT / FILTRE EKF VITESSE
    let spd3D_raw = pos.coords.speed || (dist(lPos.coords.latitude, lPos.coords.longitude, cLat, cLon, R_E_BASE) / dt) * DISTANCE_RATIO;
    
    const accel_sensor_input = Math.sqrt(real_accel_x**2 + real_accel_y**2) * (dt > 0.5 ? 0 : 1);

    const { kSpd: fSpd, kUncert: kUncert_new } = kFilter(kSpd, kUncert, spd3D_raw, dt, R_dyn, accel_sensor_input);
    kSpd = fSpd;
    kUncert = kUncert_new;
    const sSpdFE = fSpd < MIN_SPD ? 0 : fSpd; 
    
    // --- 2. CALCULS AVANCÉS ---
    
    const local_g = getGravityLocal(kAlt_new, currentCelestialBody, rotationRadius, angularVelocity); 
    
    // Le calcul de l'accélération longitudinale est déplacé dans calculateAdvancedPhysics pour la cohérence
    const advancedPhysics = calculateAdvancedPhysics(sSpdFE, kAlt_new, currentMass, currentCdA, 
        lastWeatherData ? lastWeatherData.tempK : 293.15, 
        lastWeatherData ? lastWeatherData.air_density : 1.225, 
        cLat, kAltUncert_new, local_g, lastFSpeed, pos, lPos);

    lastFSpeed = sSpdFE; // Mettre à jour après le calcul de l'accélération
        
    const bioSVT = calculateBioSVT(
        lastWeatherData ? lastWeatherData.tempC : 20.0, 
        kAlt_new, 
        lastWeatherData ? lastWeatherData.humidity_perc : 50.0, 
        lastWeatherData ? lastWeatherData.pressure_hPa * 100 : 101325, 
        sunAltitudeRad);
        
    const E_k = 0.5 * currentMass * sSpdFE * sSpdFE;
    const Power = advancedPhysics.accel_long * currentMass * sSpdFE;

    distM += sSpdFE * dt; 
    if (sSpdFE > MIN_SPD) { timeMoving += dt; }
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    
    // --- 3. MISE À JOUR DU DOM (Affichage) ---
    
    const timeElapsed = (now.getTime() - sTime) / 1000.0;
    if ($('elapsed-time')) $('elapsed-time').textContent = timeElapsed.toFixed(2) + ' s';
    
    // VITESSE
    if ($('speed-stable')) $('speed-stable').textContent = (sSpdFE * KMH_MS).toFixed(1);
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${sSpdFE.toFixed(2)} m/s`;
    if ($('speed-stable-kms')) $('speed-stable-kms').textContent = `${(sSpdFE / 1000).toExponential(2)} km/s`;
    if ($('speed-3d-inst')) $('speed-3d-inst').textContent = `${(spd3D_raw * KMH_MS).toFixed(5)} km/h`;
    if ($('speed-raw-ms')) $('speed-raw-ms').textContent = `${spdRaw.toFixed(2)} m/s`;
    if ($('speed-max')) $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    if ($('speed-avg-moving')) $('speed-avg-moving').textContent = `${(distM / (timeMoving || 1) * KMH_MS).toFixed(5)} km/h`;
    if ($('perc-speed-sound')) $('perc-speed-sound').textContent = `${(sSpdFE / advancedPhysics.speedOfSoundLocal * 100).toFixed(2)} %`;
    if ($('perc-speed-c')) $('perc-speed-c').textContent = `${(sSpdFE / C_L * 100).toExponential(2)} %`;

    // DISTANCE
    if ($('distance-total-km')) $('distance-total-km').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    if ($('mode-ratio')) $('mode-ratio').textContent = `${DISTANCE_RATIO.toFixed(3)}`;
    const dist_light_s = distM / C_L;
    if ($('distance-light-s')) $('distance-light-s').textContent = dist_light_s.toExponential(2) + ' s';
    if ($('distance-light-min')) $('distance-light-min').textContent = (dist_light_s / 60).toExponential(2) + ' min';
    if ($('distance-light-h')) $('distance-light-h').textContent = (dist_light_s / 3600).toExponential(2) + ' h';
    if ($('distance-light-day')) $('distance-light-day').textContent = (dist_light_s / 86400).toExponential(2) + ' j';
    if ($('distance-light-week')) $('distance-light-week').textContent = (dist_light_s / (86400 * 7)).toExponential(2) + ' sem';
    if ($('distance-light-month')) $('distance-light-month').textContent = (dist_light_s / (86400 * 30)).toExponential(2) + ' mois';
    if ($('distance-cosmic')) $('distance-cosmic').textContent = `${(distM / AU_METERS).toExponential(2)} UA | ${(distM / LIGHT_YEAR_METERS).toExponential(2)} al`;


    // POSITION & DYNAMIQUE
    if ($('lat-display')) $('lat-display').textContent = lat.toFixed(6);
    if ($('lon-display')) $('lon-display').textContent = lon.toFixed(6);
    if ($('alt-display')) $('alt-display').textContent = kAlt_new.toFixed(2) + ' m';
    if ($('geopotential-alt')) $('geopotential-alt').textContent = advancedPhysics.geopotentialAltitude.toFixed(2) + ' m';
    if ($('gravity-local')) $('gravity-local').textContent = local_g.toFixed(4) + ' m/s²';
    if ($('accel-long')) $('accel-long').textContent = `${advancedPhysics.accel_long.toFixed(2)} m/s²`;
    if ($('coriolis-force')) $('coriolis-force').textContent = advancedPhysics.coriolisForce.toExponential(2) + ' N';
    if ($('time-moving')) $('time-moving').textContent = `${timeMoving.toFixed(0)} s`;


    // EKF & DEBUG
    if ($('gps-precision')) $('gps-precision').textContent = accRaw.toFixed(2) + ' m';
    if ($('kalman-uncert')) $('kalman-uncert').textContent = kUncert.toExponential(2) + ' m²/s²';
    if ($('alt-uncertainty')) $('alt-uncertainty').textContent = advancedPhysics.altSigma.toFixed(3) + ' m (σ)';
    if ($('speed-error-perc')) $('speed-error-perc').textContent = R_dyn.toExponential(2) + ' m²/s²';
    if ($('nyquist-frequency')) $('nyquist-frequency').textContent = advancedPhysics.nyquistFrequency.toFixed(3) + ' Hz';
    if ($('gps-accuracy-forced')) $('gps-accuracy-forced').textContent = `${GPS_ACCURACY_OVERRIDE.toFixed(6)} m`;
    if ($('gps-status-dr') && isSignalLost) $('gps-status-dr').textContent = 'Drift (Estimation)';
    if ($('mass-display')) $('mass-display').textContent = currentMass.toFixed(3) + ' kg';
    if ($('gravity-base')) $('gravity-base').textContent = CELESTIAL_DATA[currentCelestialBody].G.toFixed(4) + ' m/s²';
    
    // AFFICHAGE AVANCÉ
    if ($('speed-of-sound-local')) $('speed-of-sound-local').textContent = advancedPhysics.speedOfSoundLocal.toFixed(2) + ' m/s';
    if ($('mach-number')) $('mach-number').textContent = advancedPhysics.machNumber.toFixed(4);
    if ($('lorentz-factor')) $('lorentz-factor').textContent = advancedPhysics.lorentzFactor.toExponential(4);
    if ($('time-dilation-speed')) $('time-dilation-speed').textContent = advancedPhysics.timeDilationSpeed.toFixed(3) + ' ns/j';
    if ($('grav-dilation')) $('grav-dilation').textContent = advancedPhysics.gravitationalDilation.toFixed(3) + ' ns/j';
    if ($('energy-relativistic')) $('energy-relativistic').textContent = advancedPhysics.energyRelativistic.toExponential(3) + ' J';
    if ($('energy-rest-mass')) $('energy-rest-mass').textContent = advancedPhysics.E0.toExponential(3) + ' J';
    if ($('Rs-object')) $('Rs-object').textContent = advancedPhysics.Rs_object.toExponential(3) + ' m';
    if ($('momentum')) $('momentum').textContent = advancedPhysics.momentum.toFixed(2) + ' kg·m/s';
    if ($('reynolds-number')) $('reynolds-number').textContent = advancedPhysics.reynoldsNumber.toExponential(2);
    if ($('dynamic-pressure')) $('dynamic-pressure').textContent = advancedPhysics.dynamicPressure.toFixed(2) + ' Pa';
    if ($('drag-force')) $('drag-force').textContent = advancedPhysics.dragForce.toFixed(2) + ' N';
    if ($('radiation-pressure')) $('radiation-pressure').textContent = bioSVT.radiationPressure.toExponential(2) + ' Pa';
    
    // BIO/MÉTÉO AVANCÉE
    if ($('abs-humidity-sim')) $('abs-humidity-sim').textContent = lastWeatherData ? lastWeatherData.absoluteHumidity.toFixed(3) + ' g/m³' : 'N/A';
    if ($('wet-bulb-temp-sim')) $('wet-bulb-temp-sim').textContent = bioSVT.wetBulbTemp.toFixed(1) + ' °C';
    if ($('CAPE-sim')) $('CAPE-sim').textContent = bioSVT.CAPE_sim.toFixed(0) + ' J/kg';
    if ($('O2-saturation-sim')) $('O2-saturation-sim').textContent = bioSVT.O2SaturationClamped.toFixed(1) + ' %';
    if ($('photosynthesis-rate-sim')) $('photosynthesis-rate-sim').textContent = bioSVT.photosynthesisRate.toExponential(2) + ' (taux)';


    // --- 4. SAUVEGARDE & MISE À JOUR CARTE ---
    lPos = pos; 
    lPos.timestamp = cTimePos; 

    updateMap(lat, lon, accRaw);
}


// ===========================================
// INITIALISATION DES ÉVÉNEMENTS DOM
// ===========================================

document.addEventListener('DOMContentLoaded', () => {
    
    initMap(); 

    // Gestionnaires d'événements
    if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', () => { 
        wID === null ? startGPS() : stopGPS(); 
    });
    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', () => {
        emergencyStopActive ? resumeSystem() : emergencyStop();
    });
    if ($('toggle-mode-btn')) $('toggle-mode-btn').addEventListener('click', () => {
        document.body.classList.toggle('dark-mode');
    });
    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', resetDistance);
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', resetMaxSpeed);
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', resetAll);
    if ($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', () => {
        isNetherMode = !isNetherMode;
        if ($('nether-toggle-btn')) $('nether-toggle-btn').textContent = `Mode Nether: ${isNetherMode ? 'ACTIVÉ (1:' + NETHER_RATIO + ')' : 'DÉSACTIVÉ (1:1)'}`;
        if ($('mode-nether')) $('mode-nether').textContent = isNetherMode ? `ACTIVÉ (1:${NETHER_RATIO})` : 'DÉSACTIVÉ (1:1)';
    });


    // Inputs et Sélecteurs
    if ($('mass-input')) $('mass-input').addEventListener('input', (e) => currentMass = parseFloat(e.target.value) || 70.0);
    if ($('gps-accuracy-override')) $('gps-accuracy-override').addEventListener('input', (e) => GPS_ACCURACY_OVERRIDE = parseFloat(e.target.value) || 0);
    if ($('environment-select')) $('environment-select').addEventListener('change', (e) => {
        selectedEnvironment = e.target.value;
        const factor = ENVIRONMENT_FACTORS[selectedEnvironment]?.R_MULT || 1.0;
        if ($('env-factor')) $('env-factor').textContent = `${ENVIRONMENT_FACTORS[selectedEnvironment]?.DISPLAY} (x${factor.toFixed(1)})`;
    });
    if ($('celestial-body-select')) $('celestial-body-select').addEventListener('change', (e) => {
        currentCelestialBody = e.target.value;
        G_ACC = getGravityLocal(kAlt, currentCelestialBody, rotationRadius, angularVelocity);
    });
    if ($('rotation-radius')) $('rotation-radius').addEventListener('input', (e) => {
        rotationRadius = parseFloat(e.target.value) || 100;
        G_ACC = getGravityLocal(kAlt, currentCelestialBody, rotationRadius, angularVelocity);
    });
    if ($('angular-velocity')) $('angular-velocity').addEventListener('input', (e) => {
        angularVelocity = parseFloat(e.target.value) || 0.0;
        G_ACC = getGravityLocal(kAlt, currentCelestialBody, rotationRadius, angularVelocity);
    });

    // --- DÉMARRAGE DU SYSTÈME ---
    G_ACC = getGravityLocal(kAlt, currentCelestialBody, rotationRadius, angularVelocity);
    
    syncH(lServH, lLocH).then(newTimes => {
        lServH = newTimes.lServH;
        lLocH = newTimes.lLocH;
        // startGPS(); // L'utilisateur doit cliquer
    });

    // Boucle de mise à jour lente (Astro/Météo/Horloge)
    if (domID === null) {
        domID = setInterval(() => {
            const currentLat = lat || 48.8566; 
            const currentLon = lon || 2.3522;
            
            // Met à jour l'astro (soleil, lune, horloge)
            const astro = updateAstro(currentLat, currentLon, lServH, lLocH);
            if (astro) sunAltitudeRad = astro.sunAltitudeRad;
            
            // Récupère la météo (si GPS actif ou mode arrêt d'urgence)
            if ((lat && lon) && !emergencyStopActive) {
                fetchWeather(lat, lon).then(data => {
                    if (data) {
                        lastWeatherData = data;
                        
                        // Met à jour le DOM météo
                        if ($('temp-air-2')) $('temp-air-2').textContent = `${data.tempC.toFixed(1)} °C`;
                        if ($('pressure-2')) $('pressure-2').textContent = `${data.pressure_hPa.toFixed(0)} hPa`;
                        if ($('humidity-2')) $('humidity-2').textContent = `${data.humidity_perc} %`;
                        if ($('air-density')) $('air-density').textContent = `${data.air_density.toFixed(3)} kg/m³`;
                        const bioSim = calculateBioSVT(data.tempC, kAlt || 0, data.humidity_perc, data.pressure_hPa * 100, sunAltitudeRad);
                        if ($('dew-point')) $('dew-point').textContent = `${bioSim.dewPoint.toFixed(1)} °C`;
                    }
                });
            }
            
            // Met à jour l'horloge locale (NTP)
            const now = getCDate(lServH, lLocH);
            if (now) {
                if ($('local-time') && !$('local-time').textContent.includes('SYNCHRO ÉCHOUÉE')) {
                    $('local-time').textContent = now.toLocaleTimeString('fr-FR');
                }
                if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
                if ($('date-display-astro')) $('date-display-astro').textContent = now.toLocaleDateString('fr-FR');
            }
            
        }, DOM_SLOW_UPDATE_MS); 
    }
});
