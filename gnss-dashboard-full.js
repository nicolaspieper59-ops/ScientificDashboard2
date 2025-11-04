// =================================================================
// FICHIER JS PARTIE 1 : gnss-dashboard-part1.js (Constantes & Kalman)
// =================================================================

const $ = (id) => document.getElementById(id);

// --- CONSTANTES GLOBALES (Physiques, GPS, Temps) ---
const C_L = 299792458; // Vitesse de la lumière (m/s)
const SPEED_SOUND = 343; // Vitesse du son (m/s)
const G_ACC = 9.80665; // Gravité standard (m/s²)
const KMH_MS = 3.6; 
const R_E = 6371000; // Rayon moyen de la Terre (m)
const R2D = 180 / Math.PI;
const D2R = Math.PI / 180;
const W_EARTH = 7.2921E-5; // Vitesse angulaire de la Terre (rad/s)
const NETHER_RATIO = 1 / 8; 

// Constantes Temps / Astro
const dayMs = 86400000;
const J1970 = 2440588; 
const J2000 = 2451545; 
const DOM_SLOW_UPDATE_MS = 1000;
const WEATHER_UPDATE_MS = 30000; 
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc"; 

// Constantes GPS
const MIN_DT = 0.05; 
const MIN_SPD = 0.01; 
const MAX_ACC = 20; 
const GOOD_ACC_THRESHOLD = 3.0; // Seuil de précision GPS (m) où l'on réduit le bruit IMU
const ALT_TH = -50; 
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 5000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 30000, timeout: 60000 }
};

// Constantes Kalman (Vitesse 3D & Altitude)
const Q_NOISE = 0.001; // Bruit de processus Vitesse
const Q_ALT_NOISE = 0.0005; // Bruit de processus Altitude
let kSpd = 0; // Estimation vitesse (m/s)
let kUncert = 1000; // Incertitude vitesse (m/s)²
let kAlt = 0; // Estimation altitude (m)
let kAltUncert = 1000; // Incertitude altitude (m)²
const ENVIRONMENT_FACTORS = {
    NORMAL: 1.0, FOREST: 1.5, CONCRETE: 3.0, METAL: 2.5
};

// Constantes ZVU / IMU-Only / Bruit
const R_GPS_DISABLED = 100000.0; // R élevé pour désactiver mathématiquement le GPS (Mode IMU-Only)
const VEL_NOISE_FACTOR = 10; // Utilisé pour seuil dynamique ZVU : acc/VEL_NOISE_FACTOR
const IMU_NOISE_FLOOR = 0.05; // Bruit électronique typique des accéléromètres (m/s²)
const ZVU_SAFETY_MARGIN = 0.15; // Marge pour dépasser le bruit réel
const STATIC_ACCEL_THRESHOLD = IMU_NOISE_FLOOR + ZVU_SAFETY_MARGIN; // CALCULÉ : ~0.2 m/s²
const LOW_SPEED_THRESHOLD = 1.0; // 👈 NOUVEAU : Seuil de vitesse (m/s) pour amortissement dynamique de R
const ACCEL_FILTER_ALPHA = 0.8; 
const ACCEL_MOVEMENT_THRESHOLD = 0.5; 
let kAccel = { x: 0, y: 0, z: 0 };
let G_STATIC_REF = { x: 0, y: 0, z: 0 };
let latestVerticalAccelIMU = 0; // Accélération verticale pour filtre altitude
let latestLinearAccelMagnitude = 0; // Magnitude de l'accélération pour contrôle EKF

// --- VARIABLES GLOBALES (État du système) ---
let wID = null;
let map = null;
let marker = null;
let tracePolyline = null;
let lat = 0, lon = 0;
let sTime = null; 
let distM = 0; 
let timeMoving = 0; 
let maxSpd = 0;
let lastFSpeed = 0;
let currentGPSMode = 'HIGH_FREQ';
let lPos = null; 
let emergencyStopActive = false;
let netherMode = false;
let selectedEnvironment = 'NORMAL';
let lServH = 0; 
let lLocH = 0; 
let domID = null; 
let weatherID = null; 
let lastP_hPa = 1013.25; 

// --- CONSTANTES API MÉTÉO ---
const OWM_API_KEY = "VOTRE_CLE_API_OPENWEATHERMAP"; // <-- REMPLACEZ CECI
const OWM_API_URL = "https://api.openweathermap.org/data/2.5/weather"; 


// ===========================================
// FONCTIONS UTILITAIRES ET KALMAN
// ===========================================

/** Calcule la distance de Haversine entre deux points (m). */
function dist(lat1, lon1, lat2, lon2) {
    const dLat = (lat2 - lat1) * D2R;
    const dLon = (lon2 - lon1) * D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1 * D2R) * Math.cos(lat2 * D2R) * Math.sin(dLon / 2) ** 2;
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R_E * c;
}

/** Filtre de Kalman 1D (Vitesse) avec entrée de contrôle Accélération IMU */
function kFilter(z, dt, R_dyn, u_accel = 0) {
    // 1. Prediction : Utilisation de l'accélération IMU (u_accel) comme entrée de contrôle
    const predSpd = kSpd + u_accel * dt; 
    
    // Le bruit de processus (Q_NOISE) augmente avec le temps (dt)
    const predUncert = kUncert + Q_NOISE * dt;
    
    // 2. Mesure (Correction par le GPS)
    const K = predUncert / (predUncert + R_dyn);
    kSpd = predSpd + K * (z - predSpd);
    kUncert = (1 - K) * predUncert;
    return kSpd;
}

/** Calcule le bruit de mesure dynamique R (Intègre la Vitesse EKF/Énergie Cinétique) */
function getKalmanR(acc, alt, pressure) { 
    let R_raw = acc * acc; 
    const envFactor = ENVIRONMENT_FACTORS[selectedEnvironment] || ENVIRONMENT_FACTORS.NORMAL;
    const MASS_PROXY = 0.05; 

    // 1. Facteur d'Environnement (Noise Multiplier)
    let noiseMultiplier = envFactor;
    if (alt !== null && alt < 0) {
        noiseMultiplier += Math.abs(alt / 100); 
    }
    
    // 2. Facteur de Vitesse/Énergie Cinétique (Réduction du bruit R aux hautes vitesses)
    const kSpd_squared = kSpd * kSpd;
    const speedFactor = 1 / (1 + MASS_PROXY * kSpd_squared); // Réduit R aux hautes vitesses

    // 3. NOUVEAU : Boost R aux basses vitesses (non-ZVU) pour fluidifier les mouvements lents.
    let lowSpeedBoost = 1.0;
    if (kSpd > MIN_SPD && kSpd < LOW_SPEED_THRESHOLD) {
        // Interpolation de 1.0 (à LOW_SPEED_THRESHOLD) à 2.0 (vers 0 m/s)
        const speed_ratio = (LOW_SPEED_THRESHOLD - kSpd) / LOW_SPEED_THRESHOLD; 
        lowSpeedBoost = 1.0 + speed_ratio * 1.0; // Boost R jusqu'à 2x à la vitesse minimale
    }
    
    // R final
    let R_dyn = R_raw * noiseMultiplier * speedFactor * lowSpeedBoost; // Application du boost
    
    // S'assurer que R n'est jamais trop petit
    R_dyn = Math.max(R_dyn, 0.01); 

    return R_dyn;
}

/** Filtre de Kalman 1D pour l'Altitude (Utilise u_accel IMU) */
function kFilterAltitude(z, acc, dt, u_accel = 0) { 
    if (z === null) return kAlt; 

    // 1. Prediction (avec Accélération IMU comme entrée de contrôle)
    const predAlt = kAlt + (0.5 * u_accel * dt * dt); 
    let predAltUncert = kAltUncert + Q_ALT_NOISE * dt;

    // 2. Mesure (R_alt est basé sur la précision brute du GPS)
    const R_alt = acc * acc * 2.0; 
    const K = predAltUncert / (predAltUncert + R_alt);
    kAlt = predAlt + K * (z - predAlt);
    kAltUncert = (1 - K) * predAltUncert;

    return kAlt;
}

/** Calcule le point de rosée (utilitaire pour la météo) */
function calculateDewPoint(tempC, humidity) {
    const a = 17.27;
    const b = 237.7;
    const alpha = (a * tempC) / (b + tempC) + Math.log(humidity / 100);
    return (b * alpha) / (a - alpha);
}

/** Calcule le nom de la phase de la Lune à partir du coefficient de phase (0.0 à 1.0) */
function getMoonPhaseName(phase) {
    if (phase < 0.03 || phase > 0.97) return "Nouvelle Lune 🌑";
    if (phase < 0.22) return "Premier Croissant 🌒";
    if (phase < 0.28) return "Premier Quartier 🌓";
    if (phase < 0.47) return "Gibbeuse Croissante 🌔";
    if (phase < 0.53) return "Pleine Lune 🌕";
    if (phase < 0.72) return "Gibbeuse Décroissante 🌖";
    if (phase < 0.78) return "Dernier Quartier 🌗";
    return "Dernier Croissant 🌘";
}

// ===========================================
// FONCTIONS ASTRO UTILITAIRES
// ===========================================

/** Obtient l'heure courante synchronisée (si syncH a réussi) */
function getCDate() {
    if (lLocH === 0) return new Date();
    const currentLocTime = performance.now();
    const offset = currentLocTime - lLocH;
    return new Date(lServH + offset);
}

/** Synchronisation horaire par serveur (UTC/Atomique) */
async function syncH() { 
    if ($('local-time')) $('local-time').textContent = 'Synchronisation UTC...';
    const localStartPerformance = performance.now(); 
    try {
        const response = await fetch(SERVER_TIME_ENDPOINT, { cache: "no-store", mode: "cors" });
        if (!response.ok) throw new Error(`Server time sync failed: ${response.statusText}`);
        const localEndPerformance = performance.now(); 
        const serverData = await response.json(); 
        const RTT = localEndPerformance - localStartPerformance;
        const latencyOffset = RTT / 2;
        lServH = Date.parse(serverData.datetime) + latencyOffset; 
        lLocH = performance.now(); 
        console.log(`Synchronisation UTC Atomique réussie.`);
    } catch (error) {
        console.warn("Échec de la synchronisation. Utilisation de l'horloge locale.", error);
        lServH = Date.now(); 
        lLocH = performance.now();
        if ($('local-time')) $('local-time').textContent = 'N/A (SYNCHRO ÉCHOUÉE)';
    }
}

/** Convertit la date en jours depuis J2000. */
function toDays(date) { return (date.valueOf() / dayMs - 0.5 + J1970) - J2000; }
/** Calcule l'anomalie solaire moyenne. */
function solarMeanAnomaly(d) { return D2R * (356.0470 + 0.9856002585 * d); }
/** Calcule la longitude écliptique. */
function eclipticLongitude(M) {
    // Calcul de la longitude écliptique du Soleil (L) et des corrections (C et P)
    var C = D2R * (1.9148 * Math.sin(M) + 0.0200 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M)), 
        P = D2R * 102.9377; // Longitude du périhélie                                                                
    return M + C + P + Math.PI;
}

/** Calcule le Temps Solaire Vrai (TST) normalisé. */
function getSolarTime(date, lon) {
    if (date === null || lon === null) return { TST: 'N/A', MST: 'N/A', EOT: 'N/D', ECL_LONG: 'N/D', TST_MS: 0 };
    const d = toDays(date);
    const M = solarMeanAnomaly(d); 
    const L = eclipticLongitude(M); 
    const epsilon = D2R * (23.4393 - 0.000000356 * d); // Obliquité de l'écliptique
    
    // Calcul de l'Ascension Droite (alpha)
    let alpha = Math.atan2(Math.cos(epsilon) * Math.sin(L), Math.cos(L));
    if (alpha < 0) alpha += 2 * Math.PI; 
    
    const meanLongitude = M + D2R * 102.9377 + Math.PI;
    
    // Équation du Temps (EOT = Ascension Droite Moyenne - Ascension Droite Vraie)
    let eot_rad_raw = alpha - meanLongitude; 
    eot_rad_raw = eot_rad_raw % (2 * Math.PI);
    if (eot_rad_raw > Math.PI) { eot_rad_raw -= 2 * Math.PI; } else if (eot_rad_raw < -Math.PI) { eot_rad_raw += 2 * Math.PI; }
    const eot_min = eot_rad_raw * 4 * R2D; // Conversion en minutes (15 deg = 1h, 1 deg = 4 min)
    
    let ecl_long_deg = (L * R2D) % 360; 
    const final_ecl_long = ecl_long_deg < 0 ? ecl_long_deg + 360 : ecl_long_deg;
    
    // Temps Solaire Moyen Local (MST)
    const msSinceMidnightUTC = (date.getUTCHours() * 3600 + date.getUTCMinutes() * 60 + date.getUTCSeconds()) * 1000 + date.getUTCMilliseconds();
    const mst_offset_ms = lon * dayMs / 360; 
    const mst_ms_raw = msSinceMidnightUTC + mst_offset_ms;
    const mst_ms = (mst_ms_raw % dayMs + dayMs) % dayMs; 
    
    // Temps Solaire Vrai (TST = MST + EOT)
    const eot_ms = eot_min * 60000;
    const tst_ms_raw = mst_ms + eot_ms;
    const tst_ms = (tst_ms_raw % dayMs + dayMs) % dayMs; 
    
    const toTimeString = (ms) => {
        let h = Math.floor(ms / 3600000);
        let m = Math.floor((ms % 3600000) / 60000);
        let s = Math.floor((ms % 60000) / 1000);
        return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
    };
    return { TST: toTimeString(tst_ms), TST_MS: tst_ms, MST: toTimeString(mst_ms), EOT: eot_min.toFixed(3), ECL_LONG: final_ecl_long.toFixed(2) };
}
// =======
// ===========================================
// VARIABLES GLOBALES (Si non définies dans part1.js)
// ===========================================
let wID = null;
let map = null;
let marker = null;
let tracePolyline = null;
let sTime = null;
let lPos = null;
let distM = 0;
let maxSpd = 0;
let timeMoving = 0;
let lastFSpeed = 0;
let currentGPSMode = 'HIGH_ACCURACY';
let latestLinearAccelMagnitude = 0.0;
let latestVerticalAccelIMU = 0.0;
let emergencyStopActive = false;
let netherMode = false;
let domID = null;
let weatherID = null; 
let lat = 0; 
let lon = 0;
let lastP_hPa = 1013.25; 
let kAccel = { x: 0.0, y: 0.0, z: 0.0 };
let G_STATIC_REF = { x: 0.0, y: 0.0, z: G_ACC }; // G_ACC doit être défini
let selectedEnvironment = 'NORMAL'; 
// kSpd, kUncert, kAlt, kAltUncert doivent être initialisés dans part1.js

// ===========================================
// FONCTION PRINCIPALE DE MISE À JOUR (GPS CALLBACK)
// ===========================================

function updateDisp(pos) {
    if (emergencyStopActive) return;
    lat = pos.coords.latitude; lon = pos.coords.longitude;
    const alt = pos.coords.altitude, acc = pos.coords.accuracy;
    const spd_raw_gps = pos.coords.speed;
    const cTimePos = pos.timestamp; 
    const now = getCDate(); // Fonction supposée définie
    const MASS = 70.0; 
    
    // --- Initialisation/Vérifications ---
    if (now === null) { updateAstro(lat, lon); return; } 
    if (sTime === null) { sTime = now.getTime(); }
    if (acc > MAX_ACC) { 
        if ($('gps-precision')) $('gps-precision').textContent = `❌ ${acc.toFixed(0)} m (Trop Imprécis)`; 
        if (lPos === null) lPos = pos; return; 
    }

    let effectiveAcc = acc;
    const accOverride = parseFloat($('gps-accuracy-override').value);
    if (accOverride > 0) { effectiveAcc = accOverride; }

    let spdH = spd_raw_gps ?? 0; 
    const dt = lPos ? (cTimePos - lPos.timestamp) / 1000 : MIN_DT;

    // 1. DÉTERMINATION DU FACTEUR D'AMORTISSEMENT IMU (Dampening)
    let imuDampeningFactor = 1.0; 
    if (effectiveAcc <= GOOD_ACC_THRESHOLD) { imuDampeningFactor = 0.5; } 
    else if (effectiveAcc >= MAX_ACC) { imuDampeningFactor = 1.0; } 
    else {
        const range = MAX_ACC - GOOD_ACC_THRESHOLD;
        const value = effectiveAcc - GOOD_ACC_THRESHOLD;
        imuDampeningFactor = 0.5 + 0.5 * (value / range); 
    }

    // 2. Entrées de Contrôle Amorties
    let accel_control_3D = latestLinearAccelMagnitude * imuDampeningFactor;
    let accel_control_V = latestVerticalAccelIMU * imuDampeningFactor;

    // 3. VITESSE 3D INSTANTANÉE BRUTE (Basée sur GPS)
    let spdV_raw = 0; 
    if (lPos && lPos.kAlt_old !== undefined && dt > MIN_DT && alt !== null) { 
        spdV_raw = (kAlt - lPos.kAlt_old) / dt; // kAlt est l'estimation EKF de l'itération précédente
    } 
    let spd3D_raw = Math.sqrt(spdH ** 2 + spdV_raw ** 2);

    // 4. LOGIQUE DE MISE À JOUR ZÉRO-VITESSE DYNAMIQUE (ZVU)
    const GPS_NOISE_SPEED = effectiveAcc / VEL_NOISE_FACTOR; 
    const isStaticByIMU = latestLinearAccelMagnitude < STATIC_ACCEL_THRESHOLD;
    const isStaticByDynamicSpeed = spd3D_raw < GPS_NOISE_SPEED;
    
    const isStatic = isStaticByIMU && isStaticByDynamicSpeed;
    
    let isIMUOnlyMode = false; 
    let spd3D = spd3D_raw; // Vitesse 3D Instantanée utilisée pour l'entrée EKF

    if (isStatic) {
        spd3D = 0.0; 
        isIMUOnlyMode = true; 
        
        // CRITIQUE ZVU: Force les entrées de contrôle IMU à ZÉRO
        accel_control_3D = 0.0; 
        accel_control_V = 0.0; 
        
        // CORRECTION CRITIQUE ZVU: Hard reset EKF pour une stabilité maximale
        kSpd = 0.0;
        kUncert = 0.000001; // Incertitude très faible pour forcer la stabilité
        lastFSpeed = 0.0; 
    }
    
    // Détection du Mode Intérieur/Haute Poursuite
    const HIGH_NOISE_ACC_THRESHOLD = 10.0; 
    const isHighNoise = effectiveAcc > HIGH_NOISE_ACC_THRESHOLD || selectedEnvironment === 'CONCRETE' || selectedEnvironment === 'METAL';
    const isInterior = isStatic && isHighNoise;

    // 5. FILTRAGE DE L'ALTITUDE (via Kalman)
    const kAlt_new = kFilterAltitude(alt, effectiveAcc, dt, accel_control_V); 
    
    // 6. VITESSE VERTICALE et HORIZONTALE après filtrage d'altitude/position
    let spdV = 0; 
    if (lPos && lPos.kAlt_old !== undefined && dt > MIN_DT && alt !== null) { 
        spdV = (kAlt_new - lPos.kAlt_old) / dt; 
        if (isStatic) spdV = 0.0; 
    } 
    
    if (lPos && dt > 0.05) { 
        const dH = dist(lPos.coords.latitude, lPos.coords.longitude, lat, lon); // Fonction supposée définie
        spdH = dH / dt; 
    } 
    
    // 7. FILTRE DE KALMAN FINAL (Vitesse 3D Stable)
    let R_dyn = getKalmanR(effectiveAcc, alt, lastP_hPa); // Fonction supposée définie
    
    if (isIMUOnlyMode) {
        R_dyn = R_GPS_DISABLED; // R_GPS_DISABLED doit être défini
    }

    const fSpd = kFilter(spd3D, dt, R_dyn, accel_control_3D); // Fonction supposée définie
    // VITESSE STABLE FINALE (sSpdFE): Vitesse 3D filtrée par EKF
    const sSpdFE = fSpd < MIN_SPD ? 0 : fSpd; 

    // --- CALCUL DES MÉTRIQUES BASÉ SUR LA VITESSE 3D INSTANTANÉE (spd3D) ---
    const speedForMetrics = spd3D < MIN_SPD ? 0 : spd3D; 

    // Calculs Physiques (sSpdFE pour l'énergie/accélération)
    let accel_long = (dt > 0.05) ? (sSpdFE - lastFSpeed) / dt : 0;
    lastFSpeed = sSpdFE;

    // DISTANCE 3D PARCOURUE: Intégration de la Vitesse 3D Instantanée
    distM += speedForMetrics * dt * (netherMode ? NETHER_RATIO : 1); // NETHER_RATIO doit être défini
    
    if (speedForMetrics > MIN_SPD) { timeMoving += dt; }
    // VITESSE MAX (Base spd3D)
    if (speedForMetrics > maxSpd) maxSpd = speedForMetrics; 
    // VITESSE MOYENNE (Base distM / timeMoving)
    const avgSpdMoving = timeMoving > 0 ? (distM / timeMoving) : 0;
    
    const coriolusForce = 2 * MASS * sSpdFE * W_EARTH * Math.sin(lat * D2R); // W_EARTH, D2R doivent être définis
    const kineticEnergy = 0.5 * MASS * sSpdFE ** 2;
    const mechanicalPower = accel_long * MASS * sSpdFE; 
    
    // Calcul de la densité de l'air
    let airDensity = "N/A";
    const tempElement = $('temp-air').textContent;
    const tempCMatch = tempElement.match(/(-?[\d.]+)\s*°C/);
    if (tempCMatch) {
        const tempC = parseFloat(tempCMatch[1]);
        const tempK = tempC + 273.15; 
        const pressurePa = lastP_hPa * 100; 
        const R_specific = 287.058; 
        if (!isNaN(tempK) && tempK > 0 && pressurePa > 0) {
            airDensity = (pressurePa / (R_specific * tempK)).toFixed(3);
        }
    }


    // --- MISE À JOUR DU DOM (GPS/Physique) ---
    $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(5)} km/h`; // KMH_MS doit être défini
    $('speed-stable-ms').textContent = `${sSpdFE.toFixed(3)} m/s | ${(sSpdFE * 1000).toFixed(0)} mm/s`; 
    $('speed-3d-inst').textContent = `${(spd3D * KMH_MS).toFixed(5)} km/h`;
    $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    $('speed-avg-moving').textContent = `${(avgSpdMoving * KMH_MS).toFixed(5)} km/h`;
    $('perc-speed-c').textContent = `${(spd3D / C_L * 100).toExponential(2)}%`; // C_L doit être défini
    $('perc-speed-sound').textContent = `${(spd3D / SPEED_SOUND * 100).toFixed(2)} %`; // SPEED_SOUND doit être défini
    $('distance-total-km').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    $('distance-cosmic').textContent = `${(distM / C_L).toExponential(2)} s lumière | ${(distM / C_L / (3600*24*365.25)).toExponential(2)} al`;
    
    $('latitude').textContent = lat.toFixed(6);
    $('longitude').textContent = lon.toFixed(6);
    $('altitude-gps').textContent = kAlt_new !== null ? `${kAlt_new.toFixed(2)} m` : 'N/A';
    $('gps-precision').textContent = `${acc.toFixed(2)} m`;
    $('gps-accuracy-effective').textContent = `${effectiveAcc.toFixed(2)} m`;
    $('speed-raw-ms').textContent = spd_raw_gps !== null ? `${spd_raw_gps.toFixed(2)} m/s` : 'N/A';
    
    $('vertical-speed').textContent = `${spdV.toFixed(2)} m/s`;
    $('accel-long').textContent = `${accel_long.toFixed(3)} m/s²`;
    $('force-g-long').textContent = `${(accel_long / G_ACC).toFixed(2)} G`;
    $('speed-error-perc').textContent = `${R_dyn.toFixed(3)} m² (R dyn)`; 
    
    $('kinetic-energy').textContent = `${kineticEnergy.toFixed(2)} J`;
    $('mechanical-power').textContent = `${mechanicalPower.toFixed(2)} W`;
    $('coriolis-force').textContent = `${coriolusForce.toExponential(2)} N`;
    
    $('air-density').textContent = airDensity + (airDensity !== "N/A" ? ' kg/m³' : '');
    
    // MISE À JOUR DU STATUT DÉTAILLÉ 
    const isSubterranean = (kAlt_new !== null && kAlt_new < ALT_TH); // ALT_TH doit être défini
    let statusText;
    
    if (isSubterranean) {
        statusText = `🟢 ACTIF (SOUTERRAIN/IMU) | GPS Acc: ${effectiveAcc.toFixed(0)} m`;
    } else {
        if (isInterior) {
            statusText = `🏡 INTÉRIEUR (GPS Élevé) | GPS Acc: ${effectiveAcc.toFixed(0)} m`;
        } else {
            statusText = `🔴 GPS+EKF | GPS Acc: ${effectiveAcc.toFixed(0)} m`;
        }
        
        if (isStatic) {
             const GPS_NOISE_SPEED_DISPLAY = effectiveAcc / VEL_NOISE_FACTOR; 
             statusText += ` | 🔒 ZVU ON (Seuil Dyn: ${(GPS_NOISE_SPEED_DISPLAY * KMH_MS).toFixed(2)} km/h)`;
             
             if (isIMUOnlyMode) {
                 statusText = statusText.replace('GPS+EKF', 'IMU-Only/EKF');
             }
        }
    }
    
    $('underground-status').textContent = isSubterranean ? `Oui (${statusText})` : `Non (${statusText})`;
    
    updateMap(lat, lon);
    updateAstro(lat, lon);
    
    // SAUVEGARDE DES VALEURS POUR LA PROCHAINE ITÉRATION
    lPos = pos; 
    lPos.timestamp = cTimePos; 
    lPos.kAlt_old = kAlt_new; 
    lPos.kAltUncert_old = kAltUncert; 
}


// ===========================================
// FONCTIONS UTILITAIRES (Astro)
// ===========================================

/** * Convertit un nombre de minutes depuis minuit (0 à 1440) en format HH:MM:SS. */
function formatTime(minutes) {
    if (isNaN(minutes)) return "N/D";
    let totalMinutes = minutes % 1440;
    if (totalMinutes < 0) totalMinutes += 1440;
    
    const h = Math.floor(totalMinutes / 60);
    const m = Math.floor(totalMinutes % 60);
    const s = Math.floor((totalMinutes * 60) % 60);

    const pad = (num) => num.toString().padStart(2, '0');
    return `${pad(h)}:${pad(m)}:${pad(s)}`;
}


// ===========================================
// FONCTIONS ASTRONOMIE (TST / SunCalc)
// ===========================================

function updateAstro(latA, lonA) {
    // SunCalc et getCDate() doivent être disponibles globalement
    const cDate = getCDate(); 
    if (!cDate || isNaN(latA) || isNaN(lonA)) { return; }

    const pos = SunCalc.getPosition(cDate, latA, lonA);
    const times = SunCalc.getTimes(cDate, latA, lonA);
    
    // 1. ÉLÉVATION / AZIMUT
    const sunElevationDeg = pos.altitude * 180 / Math.PI;
    const sunAzimuthDeg = pos.azimuth * 180 / Math.PI;

    // 2. MIDI SOLAIRE & TST
    const solarNoonLocal = times.solarNoon;
    const solarNoonUTCMs = solarNoonLocal.getTime() + solarNoonLocal.getTimezoneOffset() * 60 * 1000;
    const solarNoonUTC = new Date(solarNoonUTCMs); 
    
    const minutesSinceMidnight = cDate.getHours() * 60 + cDate.getMinutes() + cDate.getSeconds() / 60;
    const solarNoonMinutes = solarNoonLocal.getHours() * 60 + solarNoonLocal.getMinutes() + solarNoonLocal.getSeconds() / 60;
    
    // TST = Heure Actuelle + (12h - Heure de Midi Solaire)
    const TST_minutes = minutesSinceMidnight + (720 - solarNoonMinutes);
    
    // ÉOT = (Midi Solaire Local - 12:00:00) + Correction Fuseau Horaire
    const EOT_minutes = 720 - solarNoonMinutes + (cDate.getTimezoneOffset());
    
    // Durée du Jour
    const dayDurationH = (times.sunset.getTime() - times.sunrise.getTime()) / (1000 * 3600);


    // --- MISE À JOUR DU DOM (Astro) ---
    $('date-display-astro').textContent = cDate.toLocaleDateString();
    $('sun-elevation').textContent = `${sunElevationDeg.toFixed(2)} °`;
    
    $('tst').textContent = formatTime(TST_minutes);
    $('lsm').textContent = formatTime(TST_minutes - EOT_minutes); 
    $('noon-solar').textContent = solarNoonUTC.toTimeString().split(' ')[0] + ' UTC'; 
    $('day-duration').textContent = `${dayDurationH.toFixed(2)} h`; 
    
    const sunEcliptic = SunCalc.getEcliptic(cDate);
    $('ecliptic-long').textContent = `${(sunEcliptic.lon * 180 / Math.PI).toFixed(2)} °`; 
    
    $('eot').textContent = `${EOT_minutes.toFixed(2)} min`; 
    
    updateMinecraftClock(TST_minutes, sunElevationDeg, sunAzimuthDeg); 
}

/** * Met à jour l'affichage de l'horloge stylisée du Soleil. */
function updateMinecraftClock(TST_minutes, elevation, azimuth) {
    const $clock = $('minecraft-clock');
    const $status = $('clock-status');
    const $tstDisplay = $('time-minecraft');
    
    $tstDisplay.textContent = formatTime(TST_minutes);
    $status.textContent = `TST (Élévation: ${elevation.toFixed(2)}°)`;

    const clockRotation = (TST_minutes / 4) + 180; 
    const compassAngle = azimuth + 180; 
    
    $clock.style.transform = `rotate(${clockRotation.toFixed(2)}deg)`;
    // Code CSS pour l'horloge...
            }
// ===========================================
// FONCTIONS API MÉTÉO
// ===========================================

async function updateWeather(latA, lonA) {
    // OWM_API_KEY, OWM_API_URL, calculateDewPoint doivent être définis
    if (!OWM_API_KEY || OWM_API_KEY === "VOTRE_CLE_API_OPENWEATHERMAP") {
        $('temp-air').textContent = 'API CLÉ MANQUANTE';
        return;
    }
    
    if ($('temp-air')) $('temp-air').textContent = 'Chargement...';

    try {
        const url = `${OWM_API_URL}?lat=${latA.toFixed(4)}&lon=${lonA.toFixed(4)}&units=metric&appid=${OWM_API_KEY}`;
        const response = await fetch(url);
        if (!response.ok) throw new Error(`Weather API failed: ${response.statusText}`);
        const data = await response.json();
        
        const tempC = data.main.temp;
        const pressurehPa = data.main.pressure;
        const humidity = data.main.humidity;
        const windSpeedMs = data.wind.speed; 
        const windDeg = data.wind.deg;
        
        lastP_hPa = pressurehPa; 
        const dewPointC = calculateDewPoint(tempC, humidity); // Fonction supposée définie

        // ... calculs de Wind Chill et direction du vent ...

        $('temp-air').textContent = `${tempC.toFixed(1)} °C`;
        $('pressure').textContent = `${pressurehPa.toFixed(0)} hPa | ${(pressurehPa * 0.75006).toFixed(1)} mmHg`;
        // ... Mise à jour du reste des valeurs météo ...
        
    } catch (error) {
        console.error("Erreur lors de la récupération des données météo:", error);
        $('temp-air').textContent = 'API ERREUR';
    }
}


// ===========================================
// FONCTIONS CAPTEURS INERTIELS (IMU)
// ===========================================

function handleDeviceMotion(event) {
    if (emergencyStopActive) return;
    const acc = event.accelerationIncludingGravity;
    if (acc.x === null) return; 

    // ACCEL_FILTER_ALPHA doit être défini
    kAccel.x = ACCEL_FILTER_ALPHA * kAccel.x + (1 - ACCEL_FILTER_ALPHA) * acc.x;
    kAccel.y = ACCEL_FILTER_ALPHA * kAccel.y + (1 - ACCEL_FILTER_ALPHA) * acc.y;
    kAccel.z = ACCEL_FILTER_ALPHA * kAccel.z + (1 - ACCEL_FILTER_ALPHA) * acc.z; 

    const kAccel_mag = Math.sqrt(kAccel.x ** 2 + kAccel.y ** 2 + kAccel.z ** 2);
    let accel_vertical_lin = 0.0;
    let linear_x = 0.0, linear_y = 0.0, linear_z = 0.0;

    // STATIC_ACCEL_THRESHOLD doit être défini
    if (Math.abs(kAccel_mag - G_ACC) < STATIC_ACCEL_THRESHOLD) { // G_ACC doit être défini
        G_STATIC_REF.x = kAccel.x;
        G_STATIC_REF.y = kAccel.y;
        G_STATIC_REF.z = kAccel.z;
        accel_vertical_lin = 0.0; 
    } else {
        linear_x = kAccel.x - G_STATIC_REF.x;
        linear_y = kAccel.y - G_STATIC_REF.y;
        linear_z = kAccel.z - G_STATIC_REF.z;
        accel_vertical_lin = linear_z; 
    }

    latestVerticalAccelIMU = accel_vertical_lin; 
    latestLinearAccelMagnitude = Math.sqrt(linear_x ** 2 + linear_y ** 2 + linear_z ** 2); 
    
    if ($('accel-vertical-imu')) $('accel-vertical-imu').textContent = `${accel_vertical_lin.toFixed(3)} m/s²`;
    if ($('force-g-vertical')) $('force-g-vertical').textContent = `${(accel_vertical_lin / G_ACC).toFixed(2)} G`;
}

// ===========================================
// FONCTIONS CARTE ET CONTRÔLE GPS
// ===========================================

/** Initialise la carte Leaflet. */
function initMap(latA, lonA) {
    if (map) return;
    // La librairie Leaflet (L) doit être chargée dans index.html
    try {
        map = L.map('map-container').setView([latA, lonA], 15);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            maxZoom: 19,
            attribution: '© OpenStreetMap'
        }).addTo(map);

        marker = L.marker([latA, lonA]).addTo(map);
        tracePolyline = L.polyline([], { color: 'red' }).addTo(map);
    } catch (e) {
        if ($('map-container')) $('map-container').textContent = 'Erreur d\'initialisation de la carte (Leaflet manquant?).';
    }
}

/** Met à jour la carte. */
function updateMap(latA, lonA) {
    if (!map || !marker || !tracePolyline) { initMap(latA, lonA); return; }
    
    const newLatLng = [latA, lonA];
    marker.setLatLng(newLatLng);
    tracePolyline.addLatLng(newLatLng);
    map.setView(newLatLng);
}

function setGPSMode(mode) {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    currentGPSMode = mode;
    // GPS_OPTS doit être défini
    wID = navigator.geolocation.watchPosition(updateDisp, handleErr, GPS_OPTS[mode]); 
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = `⏸️ PAUSE GPS`;
    if ($('freq-select')) $('freq-select').value = mode; 
}

function startGPS() {
    if (wID === null) {
        if ($('freq-select')) $('freq-select').value = currentGPSMode; 
        setGPSMode(currentGPSMode);
        sTime = null; 
    }
}

function stopGPS(resetButton = true) {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
    }
    if (resetButton) {
        if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = `▶️ MARCHE GPS`;
    }
}

function emergencyStop() {
    emergencyStopActive = true;
    stopGPS(false);
    // ... Mise à jour des boutons d'urgence ...
}

function resumeSystem() {
    emergencyStopActive = false;
    // ... Mise à jour des boutons d'urgence ...
    startGPS();
}

function handleErr(err) {
    console.error(`Erreur GNSS (${err.code}): ${err.message}`);
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = `❌ ERREUR GPS`;
}


// ===========================================
// INITIALISATION DES ÉVÉNEMENTS ET INTERVALLES
// ===========================================

document.addEventListener('DOMContentLoaded', () => {
    
    // --- Gestion des événements des boutons (simplifié) ---
    if ($('environment-select')) {
        $('environment-select').value = selectedEnvironment;
        $('environment-select').addEventListener('change', (e) => { 
            if (emergencyStopActive) return;
            selectedEnvironment = e.target.value; 
        });
    }

    if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', () => {
        if (emergencyStopActive) return;
        wID === null ? startGPS() : stopGPS();
    });
    
    if ($('freq-select')) $('freq-select').addEventListener('change', (e) => {
        if (emergencyStopActive) return;
        setGPSMode(e.target.value);
    });
    
    // ... Écouteurs pour emergency-stop-btn, nether-toggle-btn, reset-dist-btn, reset-max-btn, reset-all-btn, etc. ...
    
    // Démarrage des capteurs IMU
    if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', handleDeviceMotion, true);
    } else {
        console.warn("DeviceMotion n'est pas supporté ou activé.");
    } 

    syncH(); // Fonction supposée définie (synchronisation de l'heure)
    startGPS(); // Démarrage initial du GPS

    // Intervalle lent pour les mises à jour Astro (1s) pour le cas sans signal GPS
    if (domID === null) {
        domID = setInterval(() => {
            // Utilise la dernière position connue
            if (lat !== 0 && lon !== 0) updateAstro(lat, lon); 
        }, DOM_SLOW_UPDATE_MS); // DOM_SLOW_UPDATE_MS doit être défini (ex: 1000)
    }
    
    // Intervalle pour la mise à jour Météo (30s)
    if (weatherID === null) {
        weatherID = setInterval(() => {
            if (lat !== 0 && lon !== 0) updateWeather(lat, lon);
        }, WEATHER_UPDATE_MS); // WEATHER_UPDATE_MS doit être défini (ex: 30000)
    }
});
