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
const GAMMA_AIR = 1.4;          // Indice adiabatique de l'air
const R_SPECIFIC_AIR = 287.058; // Constante spécifique de l'air sec (J/kg·K)
const R_E_BASE = 6371000;       // Rayon terrestre moyen (m)
const KMH_MS = 3.6;             // Conversion m/s vers km/h
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const C_DRAG_AREA = 0.5;        // CdA: Coefficient de traînée * Surface (valeur par défaut)
const T0 = 288.15;              // Température de référence (15°C) K
const P0 = 101325;              // Pression de référence (Niveau de la mer) Pa
const L_STD = 0.0065;           // Gradient de température standard K/m

// --- PARAMÈTRES DU FILTRE DE KALMAN (VITESSE/ALTITUDE) ---
const Q_NOISE = 0.1;        // Bruit de processus
const R_MIN = 0.01;         // Bruit de mesure minimum
const R_MAX = 500.0;        // Bruit de mesure maximum
const MAX_ACC = 200;        // Précision max (m) avant "Dead Reckoning"
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

// --- VARIABLES D'ÉTAT (Globales) ---
let wID = null, domID = null, lPos = null;
let lat = null, lon = null, sTime = null;
let distM = 0, maxSpd = 0, avgSpeedTotal = 0;
let kSpd = 0, kUncert = 1000; 
let timeMoving = 0, totalElapsedTime = 0; 
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
let capRaw = null; // Capteur de cap (Magnetomètre)
let map, marker, circle;

// Variables pour les nouveaux capteurs (Lumière, Sonore, Magnétique)
let maxLight = 0, avgLight = 0, lightSamples = 0; 
let maxSound = 0, avgSound = 0, soundSamples = 0; 
let maxMagnetic = 0, avgMagnetic = 0, magneticSamples = 0;
let lastP_hPa = 1013.25; // Dernière pression atmosphérique mesurée
let lastPressureAltitude = 0; // Altitude barométrique corrigée

// --- FONCTION UTILITAIRE DOM ---
const $ = id => document.getElementById(id);

/** Calcule la distance 3D Haversine en mètres (approximation de la longueur d'arc). */
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

/** Filtre de Kalman 1D pour la vitesse (Dead Reckoning Fusion). */
function kFilter(kSpd_in, kUncert_in, nSpd, dt, R_dyn, accel_input = 0) {
    if (dt === 0 || dt > 5) return { kSpd: kSpd_in, kUncert: kUncert_in }; 
    const R = R_dyn;
    const Q = Q_NOISE * dt * dt; 

    // PRÉDICTION (Modèle cinématique utilisant l'accélération IMU si disponible)
    let pSpd = kSpd_in + (accel_input * dt); 
    let pUnc = kUncert_in + Q; 

    // CORRECTION (Mesure GPS/GNSS)
    let K = pUnc / (pUnc + R); 
    let kSpd = pSpd + K * (nSpd - pSpd); 
    let kUncert = (1 - K) * pUnc; 
    
    // Si la mesure GPS est très incertaine (R est grand), le filtre se comporte comme un INS/Dead Reckoning.
    return { kSpd, kUncert };
}

/** Applique le filtre de Kalman à l'Altitude (Fusion GPS/Baromètre). */
function kFilterAltitude(kAlt_in, kAltUncert_in, nAlt_GPS, nAlt_Baro, acc_GPS, dt) {
    const R_GPS = Math.max(R_ALT_MIN, acc_GPS * acc_GPS);
    const R_BARO = 0.5 * 0.5; // Erreur typique d'un bon baromètre (~0.5m)
    const Q_alt = Q_NOISE * dt; 
    
    let pAlt = kAlt_in === null ? nAlt_GPS : kAlt_in; 
    let pAltUncert = kAlt_in === null ? 1000 : kAltUncert_in + Q_alt;
    
    // Correction 1: GPS
    let K_GPS = pAltUncert / (pAltUncert + R_GPS);
    pAlt = pAlt + K_GPS * (nAlt_GPS - pAlt);
    pAltUncert = (1 - K_GPS) * pAltUncert;
    
    // Correction 2: Baromètre (si disponible/valide)
    if (nAlt_Baro !== null && !isNaN(nAlt_Baro)) {
        let K_Baro = pAltUncert / (pAltUncert + R_BARO);
        pAlt = pAlt + K_Baro * (nAlt_Baro - pAlt);
        pAltUncert = (1 - K_Baro) * pAltUncert;
    }
    
    return { kAlt: pAlt, kAltUncert: pAltUncert };
}

/** Calcule le Facteur R (Confiance GPS) du filtre de Kalman. */
function getKalmanR(acc, alt, P_hPa, selectedEnv) {
    let acc_effective = GPS_ACCURACY_OVERRIDE > 0 ? GPS_ACCURACY_OVERRIDE : acc;
    if (acc_effective > MAX_ACC) { return 1e9; }
    
    let R = acc_effective * acc_effective; 
    const envFactor = ENVIRONMENT_FACTORS[selectedEnv]?.R_MULT || 1.0;
    R *= envFactor;
    
    // Augmentation de l'incertitude dans des conditions de basse pression (ex: orage)
    if (P_hPa !== null) {
        const pressureFactor = 1.0 + Math.abs(1013.25 - P_hPa) / 100.0 * 0.1;
        R *= Math.max(1.0, pressureFactor); 
    }
    
    if (alt !== null && alt < ALT_TH) { 
        R *= 2.0; // Pression accrue dans les tunnels/grottes
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
    const energyRelativistic = V > 0.0001 ? (lorentzFactor * E0) : E0;
    const momentum = mass * V; 
    const gravitationalDilation = alt * 1e-18 * 86400 * 1e9; 
    const Rs_object = (2 * G_U * mass) / (C_L * C_L);
    
    // --- 2. Mécanique des Fluides
    const speedOfSoundLocal = Math.sqrt(GAMMA_AIR * R_SPECIFIC_AIR * tempK); 
    const machNumber = V / speedOfSoundLocal;
    const dynamicPressure = 0.5 * airDensity * V * V; // q = 1/2 ρV²
    const dragForce = dynamicPressure * CdA; 

    // --- 3. Géophysique
    // Ratio de distance Geocentrique
    const geocentricRatio = (R_E_BASE + alt) / R_E_BASE; // R_terre / R_terre+alt
    
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
        speedOfSoundLocal, machNumber, dynamicPressure, dragForce,
        coriolisForce, geopotentialAltitude, force_g_long, accel_long, geocentricRatio,
        nyquistFrequency, altSigma
    };
}

/**
 * Calcule les grandeurs de Bio-Physique et de Stress (sans label "Sim.").
 */
function calculateBioSVT(tempC, alt, humidity_perc, pressurePa, sunAltitudeRad) {
    
    // 1. Point de Rosée 
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
    const CAPE = Math.max(0, 0.5 * (tempC - dewPoint) * 500); 

    // 4. Saturation en Oxygène (SVT) - Modèle d'altitude
    const O2Saturation = 97.5 - (alt / 1000) * 2; 
    const O2SaturationClamped = Math.max(70, Math.min(100, O2Saturation));

    // 5. Taux de Photosynthèse (SVT) - Modèle simplifié
    const sunAngleFactor = Math.max(0, sunAltitudeRad * R2D) / 90; 
    const tempFactor = Math.exp(-0.5 * Math.pow((tempC - 25) / 10, 2)); 
    const photosynthesisRate = 0.05 * sunAngleFactor * tempFactor; 
    
    // 6. Rayonnement Solaire (Irradiance)
    const solarIrradiance = 1000 * Math.max(0, Math.sin(sunAltitudeRad)); 
    const radiationPressure = solarIrradiance / C_L; 

    return { 
        dewPoint, wetBulbTemp, CAPE, O2SaturationClamped, photosynthesisRate, 
        solarIrradiance, radiationPressure
    };
        }
// =================================================================
// BLOC 3/4 : Services Externes, Météo et Modèles Astronomiques
// =================================================================

// --- FONCTIONS NTP ET HORLOGE ---
async function syncH(lServH_in, lLocH_in) {
    // ... (Logique de synchronisation NTP inchangée) ...
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

function getCDate(lServH, lLocH) { 
    if (lServH === null || lLocH === null) { return null; }
    const offsetSinceSync = performance.now() - lLocH;
    return new Date(lServH + offsetSinceSync); 
}

// --- FONCTIONS MÉTÉO ET BAROMÉTRIQUE (Correction d'Altitude) ---

/** Calcule l'altitude par pression (formule hypsométrique simplifiée). */
function getPressureAltitude(P_hPa, T_C, P_ref = 1013.25) {
    const T_K = T_C + 273.15;
    const P_Pa = P_hPa * 100;
    const P_ref_Pa = P_ref * 100;
    
    // Formule barométrique (pour l'atmosphère standard)
    // alt = (T0 / L_STD) * (1 - (P / P0)^(R_SPECIFIC_AIR * L_STD / G0))
    const tempRatio = T_K / T0;
    const pressureRatio = P_Pa / P0;
    const exponent = (R_SPECIFIC_AIR * L_STD) / 9.80665; 

    // Calcul de l'altitude au-dessus du niveau de la mer (basé sur la pression locale et le niveau de la mer standard)
    let alt_pressure = (T0 / L_STD) * (1 - Math.pow(P_Pa / P0, exponent));
    
    return alt_pressure;
}


/** Récupère et traite les données météo via l'API et met à jour l'altitude barométrique */
async function fetchWeather(latA, lonA, kAlt_current) {
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
            const absoluteHumidity = (P_v * 100 * 18.015) / (8.314 * tempK) * 1000; 
            
            // Correction d'Altitude Barométrique par Pression Météo
            lastPressureAltitude = getPressureAltitude(pressure_hPa, tempC); 
            
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


// --- FONCTIONS ASTRO (Révisées) ---
const dayMs = 1000 * 60 * 60 * 24;
const J1970 = 2440588, J2000 = 2451545;

function toDays(date) { return (date.valueOf() / dayMs - 0.5 + J1970) - J2000; }
function solarMeanAnomaly(d) { return D2R * (356.0470 + 0.9856002585 * d); }
function eclipticLongitude(M) {
    var C = D2R * (1.9148 * Math.sin(M) + 0.0200 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M)), 
        P = D2R * 102.9377;                                                                
    return M + C + P + Math.PI;
}

/** Calcule le Temps Solaire Vrai (TST) et Moyen (MST), ainsi que l'Équation du Temps (EOT). */
function getSolarTime(date, lon) {
    if (date === null || lon === null || isNaN(lon)) return { TST: 'N/A', MST: 'N/A', EOT: 'N/D', ECL_LONG: 'N/D' };
    
    const d = toDays(date);
    const M = solarMeanAnomaly(d); 
    const L = eclipticLongitude(M); 
    
    // Équation du Temps (EOT) en minutes
    const eot_min = (4.0 * R2D * (L - D2R * 180 - M)) / (15 * D2R);
    
    // Temps Solaire Moyen (MST)
    const msSinceMidnightUTC = (date.getUTCHours() * 3600 + date.getUTCMinutes() * 60 + date.getUTCSeconds()) * 1000 + date.getUTCMilliseconds();
    const mst_offset_ms = lon * dayMs / 360; 
    const mst_ms = (msSinceMidnightUTC + mst_offset_ms + dayMs) % dayMs;
    
    // Temps Solaire Vrai (TST)
    const eot_ms = eot_min * 60000;
    const tst_ms = (mst_ms + eot_ms + dayMs) % dayMs; 
    
    // Midi Solaire Vrai
    const noon_ms = (dayMs - eot_ms + dayMs) % dayMs; 

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
        NoonSolar: toTimeString(noon_ms)
    };
}


/** Fonction principale de mise à jour Astro (appelée par la boucle lente) */
function updateAstro(latA, lonA, lServH, lLocH) {
    const now = getCDate(lServH, lLocH); 
    
    if (now === null || typeof SunCalc === 'undefined' || !latA || !lonA) {
        if ($('clock-status')) $('clock-status').textContent = 'Astro (Attente GPS/Lib. SunCalc)...';
        return { sunAltitudeRad: 0 };
    }
    
    const sunPos = SunCalc.getPosition(now, latA, lonA);
    const moonIllum = SunCalc.getMoonIllumination(now);
    const solarTimes = getSolarTime(now, lonA);
    const times = SunCalc.getTimes(now, latA, lonA);
    let sunAltitudeRad = sunPos.altitude;
    
    const moonTimes = SunCalc.getMoonTimes(now, latA, lonA);
    
    // Heures de Lever/Coucher du Soleil (Heure Locale)
    if ($('sun-rise-local')) $('sun-rise-local').textContent = times.sunrise ? times.sunrise.toLocaleTimeString('fr-FR') : 'N/A';
    if ($('sun-set-local')) $('sun-set-local').textContent = times.sunset ? times.sunset.toLocaleTimeString('fr-FR') : 'N/A';
    
    // Calcul de l'Heure Solaire Vraie (TST) des événements
    const msOffset = (now.getUTCHours() * 3600 + now.getUTCMinutes() * 60 + now.getUTCSeconds()) * 1000 - solarTimes.TST_ms;
    const toSolarTime = (date) => date ? new Date(date.getTime() - msOffset) : null;
    
    // Mise à jour des données Astro
    if ($('tst')) $('tst').textContent = solarTimes.TST;
    if ($('mst')) $('mst').textContent = solarTimes.MST;
    if ($('noon-solar')) $('noon-solar').textContent = solarTimes.NoonSolar;
    if ($('eot')) $('eot').textContent = solarTimes.EOT + ' min';
    if ($('moon-phase-name')) $('moon-phase-name').textContent = `${getMoonPhaseName(moonIllum.fraction)} (${(moonIllum.fraction*100).toFixed(1)}%)`;
    
    // Animation du Ciel (Zénith en haut)
    const dayProgress = (solarTimes.TST_ms / dayMs) * 2; // 0 à 2, 12h = 1.0
    const skyAngle = (dayProgress * 180 + 90) % 360; 
    
    const isXray = document.body.classList.contains('x-ray-mode');
    if ($('sun-element')) $('sun-element').style.transform = `translateY(${Math.cos(sunPos.azimuth) * 50}px) rotate(${sunPos.azimuth}rad)`; // Animation de base
    
    const sunAltDeg = sunAltitudeRad * R2D;
    const altFactor = Math.min(1, Math.max(0, sunAltDeg / 90));
    const sunCSSAlt = 100 - (altFactor * 100);

    if ($('sun-element')) $('sun-element').style.top = `${sunCSSAlt}%`;
    if ($('moon-element')) $('moon-element').style.top = `${100 - (moonIllum.altitude * R2D / 90) * 100}%`;
    
    document.body.classList.toggle('zenith-sky', sunAltitudeRad > 0);
    
    if ($('clock-status')) $('clock-status').textContent = sunPos && sunPos.altitude > 0 ? 'Jour Solaire (☀️)' : 'Nuit/Crépuscule (🌙)';

    return { sunAltitudeRad, solarTimes };
}
// =================================================================
// BLOC 4/4 : Logique Applicative, GPS/IMU et Initialisation
// =================================================================

// --- GESTION DES CAPTEURS IMU ET SUPPLÉMENTAIRES ---
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
    }
}

function imuOrientationHandler(event) {
    // Cap: Utilise l'Alpha (direction vers le Nord magnétique)
    if (event.absolute && event.alpha !== null) {
        capRaw = event.alpha; // Cap en degrés (0-360)
        if ($('cap-display')) $('cap-display').textContent = capRaw.toFixed(1) + '°';
    }
    
    // Magnétomètre
    const magneticField = (event.webkitCompassHeading || capRaw) * 0.1; // Approximation pour l'affichage
    if (magneticField) {
        magneticSamples++;
        maxMagnetic = Math.max(maxMagnetic, magneticField);
        avgMagnetic = (avgMagnetic * (magneticSamples - 1) + magneticField) / magneticSamples;
        if ($('magnetic-field')) $('magnetic-field').textContent = magneticField.toFixed(2) + ' μT';
    }
}

function startIMUListeners() {
    if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', imuMotionHandler, true);
    }
    if (window.DeviceOrientationEvent) {
        window.addEventListener('deviceorientation', imuOrientationHandler, true);
    }
    // Remarque: Les API Sensor pour la lumière/son nécessitent des permissions/implémentations plus complexes
    // (Ambient Light Sensor API, Audio Context/Analyser). Nous gérons ici l'IMU et le Magnétomètre.
    if ($('imu-status')) $('imu-status').textContent = "Actif (IMU/Mag)";
}

function stopIMUListeners() {
    window.removeEventListener('devicemotion', imuMotionHandler, true);
    window.removeEventListener('deviceorientation', imuOrientationHandler, true);
    if ($('imu-status')) $('imu-status').textContent = 'Inactif';
}

// --- Fonctions Carte/GPS (inchangées) ---
function initMap() {
    // ... (Logique d'initialisation de Leaflet inchangée) ...
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
    // ... (Logique de mise à jour de la carte inchangée) ...
    if (marker && circle) {
        const newLatLng = L.latLng(lat, lon);
        marker.setLatLng(newLatLng);
        circle.setLatLng(newLatLng).setRadius(acc || 10);
        
        if (Date.now() - map.lastMoveTime > 3000) {
            map.panTo(newLatLng);
        }
    }
}

// --- FONCTIONS DE CONTRÔLE (inchangées) ---
// (startGPS, stopGPS, emergencyStop, resumeSystem, handleErr, resetDistance, resetMaxSpeed, resetAll)

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
    totalElapsedTime = 0;
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
    totalElapsedTime = 0;
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

    let isSignalLost = (accRaw > MAX_ACC);
    let R_dyn = getKalmanR(accRaw, kAlt, lastWeatherData ? lastWeatherData.pressure_hPa : null, selectedEnvironment); 
    
    if (!lPos) { // Initialisation
        lPos = pos; kAlt = altRaw; 
        lat = cLat; lon = cLon;
        updateMap(cLat, cLon, accRaw);
        if ($('gps-status-dr')) $('gps-status-dr').textContent = 'Actif';
        return;
    }

    // Dead Reckoning (Grottes)
    if (isSignalLost) { 
        // Si perte de signal, on estime la position et l'altitude uniquement avec le filtre Kalman (EKF/IMU Fusion)
        // Les coordonnées sont maintenues stables
        cLat = lat; cLon = lon; altRaw = kAlt; 
        if ($('underground-status')) $('underground-status').textContent = 'Dead Reckoning (Grottes/Estimation)';
    } else {
        lat = cLat; lon = cLon;
        if ($('underground-status')) $('underground-status').textContent = kAlt < ALT_TH ? 'Sous-sol (signal faible)' : 'Signal stable';
    }
    
    let dt = (cTimePos - lPos.timestamp) / 1000;
    if (dt < MIN_DT || dt > 10) { lPos = pos; return; }
    
    totalElapsedTime += dt;

    // EKF ALTITUDE (Fusion GPS/Baromètre)
    const { kAlt: kAlt_new, kAltUncert: kAltUncert_new } = kFilterAltitude(
        kAlt, kAltUncert, 
        altRaw, 
        lastPressureAltitude, // Altitude Barométrique
        pos.coords.altitudeAccuracy || R_ALT_MIN, dt
    );
    kAlt = kAlt_new;
    kAltUncert = kAltUncert_new;
    
    // ZUPT / FILTRE EKF VITESSE
    let spd3D_raw = pos.coords.speed || (dist(lPos.coords.latitude, lPos.coords.longitude, cLat, cLon, R_E_BASE) / dt);
    
    const accel_sensor_input = Math.sqrt(real_accel_x**2 + real_accel_y**2) * (dt > 0.5 ? 0 : 1);

    const { kSpd: fSpd, kUncert: kUncert_new } = kFilter(kSpd, kUncert, spd3D_raw, dt, R_dyn, accel_sensor_input);
    kSpd = fSpd;
    kUncert = kUncert_new;
    const sSpdFE = fSpd < MIN_SPD ? 0 : fSpd; 
    
    // --- 2. CALCULS AVANCÉS ---
    
    const local_g = getGravityLocal(kAlt_new, currentCelestialBody, rotationRadius, angularVelocity); 
    
    // Données météo pour les calculs physiques
    const T_K = lastWeatherData ? lastWeatherData.tempK : 293.15;
    const RHO = lastWeatherData ? lastWeatherData.air_density : 1.225;
    const H_PERC = lastWeatherData ? lastWeatherData.humidity_perc : 50.0;
    const P_PA = lastWeatherData ? lastWeatherData.pressure_hPa * 100 : 101325;
    
    const advancedPhysics = calculateAdvancedPhysics(sSpdFE, kAlt_new, currentMass, currentCdA, 
        T_K, RHO, cLat, kAltUncert_new, local_g, lastFSpeed, pos, lPos);

    lastFSpeed = sSpdFE; 
        
    const bioSVT = calculateBioSVT(
        lastWeatherData ? lastWeatherData.tempC : 20.0, 
        kAlt_new, H_PERC, P_PA, sunAltitudeRad);
        
    // Accumulation Vitesse/Distance
    distM += sSpdFE * dt; 
    if (sSpdFE > MIN_SPD) { timeMoving += dt; }
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    avgSpeedTotal = distM / (totalElapsedTime || 1);
    
    // --- 3. MISE À JOUR DU DOM (Affichage) ---
    
    const timeElapsed = totalElapsedTime;
    if ($('elapsed-time')) $('elapsed-time').textContent = timeElapsed.toFixed(2) + ' s';
    
    // VITESSE
    if ($('speed-stable')) $('speed-stable').textContent = (sSpdFE * KMH_MS).toFixed(1);
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${sSpdFE.toFixed(2)} m/s`;
    if ($('speed-stable-kms')) $('speed-stable-kms').textContent = `${(sSpdFE / 1000).toExponential(2)} km/s`;
    if ($('speed-3d-inst')) $('speed-3d-inst').textContent = `${(spd3D_raw * KMH_MS).toFixed(1)} km/h`;
    if ($('speed-raw-ms')) $('speed-raw-ms').textContent = `${spdRaw.toFixed(2)} m/s`;
    if ($('speed-max')) $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(1)} km/h`;
    if ($('speed-avg-moving')) $('speed-avg-moving').textContent = `${(distM / (timeMoving || 1) * KMH_MS).toFixed(1)} km/h`;
    if ($('speed-avg-total')) $('speed-avg-total').textContent = `${(avgSpeedTotal * KMH_MS).toFixed(1)} km/h`; // Nouvelle Vitesse Moyenne Totale
    if ($('perc-speed-sound')) $('perc-speed-sound').textContent = `${(sSpdFE / advancedPhysics.speedOfSoundLocal * 100).toFixed(2)} %`;
    if ($('perc-speed-c')) $('perc-speed-c').textContent = `${(sSpdFE / C_L * 100).toExponential(2)} %`;

    // DISTANCE (3D)
    if ($('distance-total-km')) $('distance-total-km').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    if ($('geocentric-ratio')) $('geocentric-ratio').textContent = `${advancedPhysics.geocentricRatio.toFixed(6)}`; // Nouveau Ratio Géocentrique

    // POSITION & ALTITUDE
    if ($('lat-display')) $('lat-display').textContent = lat.toFixed(6);
    if ($('lon-display')) $('lon-display').textContent = lon.toFixed(6);
    if ($('alt-display')) $('alt-display').textContent = kAlt_new.toFixed(2) + ' m';
    if ($('pressure-alt')) $('pressure-alt').textContent = lastPressureAltitude.toFixed(2) + ' m'; // Altitude Barométrique
    if ($('geopotential-alt')) $('geopotential-alt').textContent = advancedPhysics.geopotentialAltitude.toFixed(2) + ' m';

    // MÉTÉO & BIOPHYSIQUE (Sans "Sim")
    if ($('abs-humidity')) $('abs-humidity').textContent = lastWeatherData ? lastWeatherData.absoluteHumidity.toFixed(3) + ' g/m³' : 'N/A';
    if ($('wet-bulb-temp')) $('wet-bulb-temp').textContent = bioSVT.wetBulbTemp.toFixed(1) + ' °C';
    if ($('CAPE')) $('CAPE').textContent = bioSVT.CAPE.toFixed(0) + ' J/kg';
    if ($('O2-saturation')) $('O2-saturation').textContent = bioSVT.O2SaturationClamped.toFixed(1) + ' %';
    if ($('photosynthesis-rate')) $('photosynthesis-rate').textContent = bioSVT.photosynthesisRate.toExponential(2) + ' (taux)';
    if ($('speed-of-sound-local')) $('speed-of-sound-local').textContent = advancedPhysics.speedOfSoundLocal.toFixed(2) + ' m/s';
    
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
    if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', () => { wID === null ? startGPS() : stopGPS(); });
    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', () => { emergencyStopActive ? resumeSystem() : emergencyStop(); });
    if ($('toggle-mode-btn')) $('toggle-mode-btn').addEventListener('click', () => { document.body.classList.toggle('dark-mode'); });
    if ($('xray-mode-btn')) $('xray-mode-btn').addEventListener('click', () => { document.body.classList.toggle('x-ray-mode'); }); // Nouveau bouton X-Ray
    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', resetDistance);
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', resetMaxSpeed);
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', resetAll);
    
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
    if ($('rotation-radius')) $('rotation-radius').addEventListener('input', (e) => { rotationRadius = parseFloat(e.target.value) || 100; G_ACC = getGravityLocal(kAlt, currentCelestialBody, rotationRadius, angularVelocity); });
    if ($('angular-velocity')) $('angular-velocity').addEventListener('input', (e) => { angularVelocity = parseFloat(e.target.value) || 0.0; G_ACC = getGravityLocal(kAlt, currentCelestialBody, rotationRadius, angularVelocity); });

    // --- DÉMARRAGE DU SYSTÈME ---
    G_ACC = getGravityLocal(kAlt, currentCelestialBody, rotationRadius, angularVelocity);
    
    syncH(lServH, lLocH).then(newTimes => {
        lServH = newTimes.lServH;
        lLocH = newTimes.lLocH;
        // startGPS(); // L'utilisateur doit cliquer
    });

    // Boucle de mise à jour lente (Astro/Météo/Horloge/Capteurs Lents)
    if (domID === null) {
        domID = setInterval(() => {
            const currentLat = lat || 48.8566; 
            const currentLon = lon || 2.3522;
            
            // Met à jour l'astro (soleil, lune, horloge)
            const astro = updateAstro(currentLat, currentLon, lServH, lLocH);
            if (astro) sunAltitudeRad = astro.sunAltitudeRad;
            
            // Récupère la météo (si GPS actif ou mode arrêt d'urgence)
            if ((lat && lon) && !emergencyStopActive) {
                fetchWeather(lat, lon, kAlt).then(data => {
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
            
            // Mise à jour des Max/Moyenne des capteurs
            if ($('magnetic-field-max')) $('magnetic-field-max').textContent = maxMagnetic.toFixed(2) + ' μT';
            if ($('magnetic-field-avg')) $('magnetic-field-avg').textContent = avgMagnetic.toFixed(2) + ' μT';
            
        }, DOM_SLOW_UPDATE_MS); 
    }
});
