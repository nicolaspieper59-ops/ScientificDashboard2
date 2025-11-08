// =================================================================
// FICHIER 1/3 : gnss-dashboard-part1.js
// CŒUR DU SYSTÈME (Constantes, Variables d'État, EKF, Physique)
// =================================================================

const $ = id => document.getElementById(id);

// --- PARTIE 1 : CONSTANTES GLOBALES (Physiques, GPS, Temps, Astro) ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;

// Constantes Physiques
const C_L = 299792458;          // Vitesse de la lumière (m/s)
const C_S = 343;                // Vitesse du son dans l'air (m/s)
const G_ACC_STD = 9.80665;      // Gravité standard de la Terre (m/s²)
const M_EARTH = 5.9722e24;      // Masse de la Terre (kg)
const G_CONST = 6.67430e-11;    // Constante gravitationnelle (N(m/kg)²)
const R_E = 6371000;            // Rayon moyen de la Terre (m)
const NETHER_RATIO = 8;         // Ratio Nether (1:8)

// Constantes de Conversion
const KMH_MS = 3.6; 
const MIN_DT = 0.01; 

// Paramètres EKF/GPS Avancés
const R_MIN = 0.01;             // Plancher d'incertitude R (GPS), étalon théorique
const R_MAX = 50.0;             // Incertitude R maximale tolérée (pour ignorer le GPS trop bruité)
const K_UNCERT_FLOOR = 0.01;    // Plancher d'incertitude EKF
const T_GPS_REF = 1.0;          // Temps de référence (s) pour l'étalonnage
const ALT_TH = -50;             // Seuil d'altitude pour le sous-sol

// --- Étalonnage Automatique IMU (Q) ---
// Coefficient de Dérive (Marge d'erreur IMU en %) : 0.2 = 20%
const DRIFT_RATE_COEFFICIENT = 0.2; 
// Calcule Q_StdDev (m/s²) automatiquement : Q = Coefficient * (Racine carrée de R_MIN) / T_GPS_REF
const IMU_Q_STD_DEVIATION_BASE = (DRIFT_RATE_COEFFICIENT * Math.sqrt(R_MIN)) / T_GPS_REF;

// Q_ACCEL_NOISE sera ajusté dynamiquement dans updateDisp
let Q_ACCEL_NOISE = IMU_Q_STD_DEVIATION_BASE; 

// Facteurs Environnementaux (ajustés pour la robustesse métro)
const ENVIRONMENT_FACTORS = {
    // R_MULT : Pénalité sur la mesure GPS (pour interférences)
    // Q_MULT : Multiplicateur sur la dérive IMU (pour vibrations/bruit)
    'NORMAL': { R_MULT: 1.0, Q_MULT: 1.0 },
    'METAL': { R_MULT: 3.5, Q_MULT: 1.5 },      // Métro/Tunnel
    'FOREST': { R_MULT: 1.5, Q_MULT: 1.0 },     
    'CONCRETE': { R_MULT: 4.0, Q_MULT: 1.8 },   // Souterrain/Fortes vibrations
};

// Endpoints
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app"; 
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`; 
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc"; 

// --- PARTIE 2 : VARIABLES D'ÉTAT ---
let wID = null, domID = null, lPos = null, lat = null, lon = null, sTime = null;
let distM = 0, maxSpd = 0;
let kSpd = 0, kUncert = 1000; 
let timeMoving = 0; 
let lServH = null, lLocH = null; 
let kAlt = null;      
let kAltUncert = 10;  
let initialAlt = null;

let selectedEnvironment = 'NORMAL'; 
let emergencyStopActive = false;
let netherMode = false;

// Variables Météo (Récupérées par API)
let lastP_hPa = null;   // Pression en hPa
let lastT_K = null;     // Température en Kelvin
let lastH_perc = null;  // Humidité en pourcentage

// Capteurs (IMU)
let a_sensor_longitudinal = 0; // m/s² (Capteur)
let a_sensor_vertical_imu = 0; // m/s² (Capteur)


// --- PARTIE 3 : FONCTIONS DE BASE ---

/** Calcule la distance horizontale (Haversine). */
const dist = (lat1, lon1, lat2, lon2) => {
    const R = R_E, dLat = (lat2 - lat1) * D2R, dLon = (lon2 - lon1) * D2R;
    lat1 *= D2R; lat2 *= D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
};

/** Calcule la gravité locale en fonction de l'altitude (m). */
function getGravityAtAltitude(altitude_m) {
    if (altitude_m === null) return G_ACC_STD;
    const distance_centre = R_E + altitude_m;
    return (G_CONST * M_EARTH) / (distance_centre * distance_centre);
}

// SYNCHRONISATION HORAIRE PAR SERVEUR (UTC/Atomique)
let lastServerTimeUpdate = null;
let serverOffset = 0; // ms

async function syncH() {
    try {
        const localStart = performance.now();
        const response = await fetch(SERVER_TIME_ENDPOINT);
        if (!response.ok) throw new Error("Erreur de synchro NTP");
        const data = await response.json();
        const localEnd = performance.now();
        const serverTime = new Date(data.utc_datetime).getTime();
        const rtt = localEnd - localStart;
        serverOffset = serverTime - (localEnd - rtt / 2);
        lastServerTimeUpdate = new Date();
        console.log(`Synchro NTP réussie. Décalage: ${serverOffset.toFixed(0)} ms`);
    } catch (error) {
        console.warn("Échec de la synchronisation NTP:", error.message);
    }
}

function getCDate() {
    if (serverOffset === 0) return new Date(); // Fallback si non synchronisé
    return new Date(Date.now() + serverOffset);
}

// FILTRE DE KALMAN (EKF pour Vitesse)
function kFilter(nSpd_gps, dt, R_dyn, a_sensor_long, a_sensor_vert_imu) {
    if (dt === 0 || dt > 5) return kSpd; 
    
    const a_total_sensor = Math.sqrt(a_sensor_long ** 2 + a_sensor_vert_imu ** 2);
    
    // 1. PRÉDICTION
    let predictedSpd = kSpd + a_total_sensor * dt; 
    
    // Q est l'incertitude du PROCESSUS (Dérive IMU). Q_ACCEL_NOISE est ajusté dynamiquement.
    const Q = Q_ACCEL_NOISE ** 2 * dt; 
    let pUnc = kUncert + Q; 

    // 2. CORRECTION
    const R = R_dyn ?? R_MAX; // R est l'incertitude de la MESURE (GPS)
    const K = pUnc / (pUnc + R); 
    
    // Si R est grand (GPS infime/bruité dans le métro), K est proche de 0, la correction est minime (IMU domine).
    kSpd = predictedSpd + K * (nSpd_gps - predictedSpd); 
    kUncert = (1 - K) * pUnc; 
    
    kUncert = Math.max(kUncert, K_UNCERT_FLOOR); 
    
    return kSpd;
}

/** Applique le filtre de Kalman à l'Altitude. */
function kFilterAltitude(nAlt, acc, dt) {
    if (nAlt === null) return kAlt;
    const R = Math.max(R_MIN, acc); 
    const Q = Q_ACCEL_NOISE * dt; // Utilisation d'un bruit similaire
    let pAlt = kAlt === null ? nAlt : kAlt; 
    let pUnc = kAltUncert + Q; 
    const K = pUnc / (pUnc + R); 
    kAlt = pAlt + K * (nAlt - pAlt); 
    kAltUncert = (1 - K) * pUnc; 
    return kAlt;
}

/** Calcule le Facteur R (Précision 3D Dynamique) du filtre de Kalman. */
function getKalmanR(acc, alt, P_hPa) {
    let R = acc ** 2; 
    R = Math.max(R_MIN, Math.min(R_MAX, R));
    const envFactor = ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT;

    // --- CORRECTION GRACIEUSE PAR LA MÉTÉO (Pression) ---
    // Une basse pression augmente R, réduisant la confiance dans le GPS.
    if (P_hPa !== null) {
        const pressureFactor = 1.0 + (1013.25 - P_hPa) / 1013.25 * 0.1;
        R *= Math.max(1.0, pressureFactor);
    }
    
    // Application des facteurs d'environnement (métro/tunnel)
    R *= envFactor;
    if (alt < ALT_TH) { R *= 2.0; } 
    R = Math.max(R_MIN, Math.min(R_MAX, R));
    return R;
}
// =================================================================
// FICHIER 2A/3 : gnss-dashboard-part2a-astro.js
// Contient les fonctions de temps, Astro et la logique visuelle du cadran.
// NÉCESSITE gnss-dashboard-part1.js
// =================================================================

const MC_DAY_MS = 50 * 20 * 1000 * 60; // 20 minutes en ms
const MIN_IN_DAY = 1440; // 24 * 60

/** Calcule l'Équation du Temps (en minutes) */
function getEOT(date) {
    const J = date.getTime() / 86400000 + 2440587.5; // Jour Julien
    const n = J - 2451545.0;
    const L = (280.460 + 0.98564736 * n) % 360;
    const g = (357.528 + 0.98560030 * n) % 360;
    const lambda = L + 1.915 * Math.sin(g * D2R) + 0.020 * Math.sin(2 * g * D2R);
    const epsilon = 23.439 - 0.0000004 * n;
    const RA = Math.atan2(Math.sin(lambda * D2R) * Math.cos(epsilon * D2R), Math.cos(lambda * D2R)) * R2D;
    const EOT = L - RA;
    return EOT * 4; // Conversion en minutes
}

/** Met à jour l'affichage de l'heure Minecraft, Solaire et Astro. */
function updateAstro(lat, lon) {
    const date = getCDate();
    if (!date) return;

    // Calcul de l'heure Minecraft
    const totalMsToday = (date.getHours() * 3600 + date.getMinutes() * 60 + date.getSeconds()) * 1000 + date.getMilliseconds();
    const mcTimeMs = (totalMsToday / 86400000) * MC_DAY_MS;
    const mcHours = Math.floor(mcTimeMs / (1000 * 3600));
    const mcMinutes = Math.floor((mcTimeMs % (1000 * 3600)) / (1000 * 60));
    const mcSeconds = Math.floor((mcTimeMs % (1000 * 60)) / 1000);
    if ($('time-minecraft')) $('time-minecraft').textContent = 
        `${mcHours.toString().padStart(2, '0')}:${mcMinutes.toString().padStart(2, '0')}:${mcSeconds.toString().padStart(2, '0')}`;

    // Calculs SunCalc/Solaire
    if (lat !== null && lon !== null) {
        // Temps Solaire Vraie (TST)
        const eotMinutes = getEOT(date);
        const localTimeMinutes = date.getHours() * 60 + date.getMinutes() + date.getSeconds() / 60;
        const TST = localTimeMinutes + (eotMinutes + lon * 4) / 60;
        const tstHour = Math.floor(TST) % 24;
        const tstMin = Math.floor((TST - Math.floor(TST)) * 60);

        if ($('time-solar-true')) $('time-solar-true').textContent = `${tstHour.toString().padStart(2, '0')}:${tstMin.toString().padStart(2, '0')}`;
        
        try {
            const sunPos = SunCalc.getPosition(date, lat, lon);
            const moonIllum = SunCalc.getMoonIllumination(date);
            
            if ($('sun-elevation')) $('sun-elevation').textContent = `${(sunPos.altitude * R2D).toFixed(2)} °`;
            if ($('lunar-phase-perc')) $('lunar-phase-perc').textContent = `${(moonIllum.fraction * 100).toFixed(1)} %`;
        } catch (e) {
            console.warn("SunCalc non disponible ou données GPS manquantes.");
        }
    }
        }
// =================================================================
// FICHIER 3/3 : gnss-dashboard-part2b-logic.js
// Contient la logique principale de mise à jour GPS, Contrôles et DOM.
// =================================================================

const P_RECORDS_KEY = 'gnss_precision_records';
let maxGForce = 0;
let gpsWatchID = null;
let lastGForce = 0;

// --- PERSISTANCE LOCALE & CONTRÔLES ---

function loadPrecisionRecords() {
    try {
        const records = JSON.parse(localStorage.getItem(P_RECORDS_KEY));
        if (records && records.maxGForce) {
            maxGForce = records.maxGForce;
            if ($('max-g-force')) $('max-g-force').textContent = `${maxGForce.toFixed(2)} G`;
        }
    } catch (e) {
        console.error("Erreur lors du chargement des records:", e);
    }
}

function savePrecisionRecords() {
    try {
        const records = { maxGForce: maxGForce };
        localStorage.setItem(P_RECORDS_KEY, JSON.stringify(records));
    } catch (e) {
        console.error("Erreur lors de la sauvegarde des records:", e);
    }
}

function startGPS() {
    if (gpsWatchID) return;
    $('toggle-gps-btn').textContent = "⏸️ PAUSE GPS";
    const options = { enableHighAccuracy: true, timeout: 5000, maximumAge: 0 };
    gpsWatchID = navigator.geolocation.watchPosition(updateDisp, handleErr, options);
}

function stopGPS() {
    if (gpsWatchID) {
        navigator.geolocation.clearWatch(gpsWatchID);
        gpsWatchID = null;
    }
    $('toggle-gps-btn').textContent = "▶️ MARCHE GPS";
}

function handleErr(err) {
    console.warn(`GPS ERROR(${err.code}): ${err.message}`);
    if ($('gps-precision')) $('gps-precision').textContent = `Erreur GPS ${err.code}`;
}


// --- NOUVEAU : FONCTION DE SIMULATION CAPTEURS (IMU) ---

function getSensorData(dt) {
    // === SIMULATION : Pour un environnement réel, remplacer par l'API DeviceMotionEvent ===
    
    // Ajout d'un bruit basé sur le facteur d'environnement Q_MULT
    const q_mult = ENVIRONMENT_FACTORS[selectedEnvironment].Q_MULT;
    const noise_base = 0.05 * q_mult; 
    
    const noise_long = (Math.random() - 0.5) * noise_base; 
    const noise_vert = (Math.random() - 0.5) * noise_base * 0.5; 
    
    let a_long = 0;
    
    if (kSpd > 0.1) {
        // Simuler une résistance (décélération) proportionnelle à la vitesse
        a_long = -kSpd * 0.05 + noise_long; 
    } else {
        a_long = noise_long * 0.1; // Bruit faible à l'arrêt
    }
    
    a_sensor_longitudinal = a_long; 
    a_sensor_vertical_imu = noise_vert; 

    return { 
        a_long: a_long, 
        a_vert: noise_vert 
    };
}


// --- NOUVEAU : FONCTION DE RÉCUPÉRATION DES DONNÉES MÉTÉO (API) ---

async function fetchWeather(lat, lon) {
    if (lat === null || lon === null) return;
    try {
        const response = await fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`);
        if (!response.ok) throw new Error(`Weather fetch failed: ${response.statusText}`);
        const data = await response.json();

        // Mise à jour des variables globales pour l'EKF et le DOM
        lastP_hPa = data.pressure_hPa; 
        lastT_K = data.temp_K;
        lastH_perc = data.humidity_perc;
        
    } catch (error) {
        console.error("Impossible de récupérer les données météo:", error);
    }
}


// ===========================================
// FONCTION PRINCIPALE DE MISE À JOUR GPS 
// ===========================================

function updateDisp(pos) {
    if (emergencyStopActive) return;

    // Données GPS brutes
    lat = pos.coords.latitude; lon = pos.coords.longitude;
    const alt = pos.coords.altitude, acc = pos.coords.accuracy;
    const spd_gps_raw = pos.coords.speed; 
    const cTimePos = pos.timestamp; 

    const now = getCDate(); 
    if (now === null) { updateAstro(lat, lon); return; } 

    if (sTime === null) { sTime = now.getTime(); }
    
    // Calcul du DT
    const dt = lPos ? (cTimePos - lPos.timestamp) / 1000 : MIN_DT;

    // --- Étape 0 : MISE À JOUR DYNAMIQUE DE Q (Rob. Environnementale) ---
    const q_mult = ENVIRONMENT_FACTORS[selectedEnvironment].Q_MULT;
    Q_ACCEL_NOISE = IMU_Q_STD_DEVIATION_BASE * q_mult;
    
    // --- Étape 1 : Capteurs IMU ---
    const sensorData = getSensorData(dt);
    const a_long_imu = a_sensor_longitudinal;
    const a_vert_imu = a_sensor_vertical_imu;

    // Filtrage d'Altitude
    const kAlt_new = kAlt === null && alt !== null ? alt : kFilterAltitude(alt, acc, dt); 
    
    // Vitesse 3D (Mesure GPS)
    const spd3D_gps_mesure = spd_gps_raw !== null ? spd_gps_raw : 0; 

    // --- Étape 2 : FILTRE DE KALMAN (Capteur + GPS) ---
    // R_dyn utilise la pression (lastP_hPa) pour la correction gracieuse.
    const R_dyn = getKalmanR(acc, alt, lastP_hPa); 
    const fSpd = kFilter(spd3D_gps_mesure, dt, R_dyn, a_long_imu, a_vert_imu); 
    const sSpdFE = fSpd < 0.05 ? 0 : fSpd;
    
    // Calculs de l'accélération
    const accel_long = a_long_imu; 
    
    // Mise à jour de la distance et du temps de mouvement
    const dH = lPos ? dist(lPos.coords.latitude, lPos.coords.longitude, lat, lon) : 0;
    const dAlt = kAlt_new !== null && lPos && lPos.kAlt_old !== null ? Math.abs(kAlt_new - lPos.kAlt_old) : 0;
    const d3D = Math.sqrt(dH ** 2 + dAlt ** 2);
    distM += d3D;
    if (sSpdFE > 0.1) timeMoving += dt;
    maxSpd = Math.max(maxSpd, sSpdFE);

    // Calcul de G-Force
    const localGravity = getGravityAtAltitude(kAlt_new);
    lastGForce = Math.abs(accel_long) / localGravity;
    maxGForce = Math.max(maxGForce, lastGForce);

    // --- MISE À JOUR DU DOM ---
    
    // Temps / Coordonnées / Altitude
    if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString();
    if ($('date-display')) $('date-display').textContent = now.toLocaleDateString();
    if ($('time-elapsed')) $('time-elapsed').textContent = `${((now.getTime() - sTime) / 1000).toFixed(2)} s`;
    if ($('time-moving')) $('time-moving').textContent = `${timeMoving.toFixed(2)} s`;
    if ($('altitude-gps')) $('altitude-gps').textContent = kAlt_new !== null ? `${kAlt_new.toFixed(2)} m (±${kAltUncert.toFixed(2)} m)` : 'N/A';
    if ($('gps-precision')) $('gps-precision').textContent = `${acc.toFixed(2)} m`;
    if ($('underground-status')) $('underground-status').textContent = alt < ALT_TH ? 'Oui' : 'Non';
    
    // Vitesse & Distance
    if ($('speed-stable')) $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(5)} km/h`; 
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${sSpdFE.toFixed(3)} m/s`; 
    if ($('speed-max')) $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    if ($('distance-total-km')) $('distance-total-km').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    
    // Dynamique du Véhicule (IMU)
    if ($('accel-long')) $('accel-long').textContent = `${accel_long.toFixed(3)} m/s²`;
    if ($('force-g-long')) $('force-g-long').textContent = `${(accel_long / localGravity).toFixed(2)} G`;
    if ($('accel-vertical-imu')) $('accel-vertical-imu').textContent = `${a_vert_imu.toFixed(3)} m/s²`;
    if ($('force-g-vertical')) $('force-g-vertical').textContent = `${(a_vert_imu / localGravity).toFixed(2)} G`;
    if ($('max-g-force')) $('max-g-force').textContent = `${maxGForce.toFixed(2)} G`;
    if ($('gravity-local')) $('gravity-local').textContent = `${localGravity.toFixed(5)} m/s²`;

    // EKF/IMU (Q/R)
    if ($('speed-error-perc')) $('speed-error-perc').textContent = `${R_dyn.toFixed(5)} m² (R dyn)`;
    if ($('q-std-dev')) $('q-std-dev').textContent = `${Q_ACCEL_NOISE.toFixed(5)} m/s² (Q StdDev)`;
    if ($('q-perc-factor')) $('q-perc-factor').textContent = `${(DRIFT_RATE_COEFFICIENT * 100).toFixed(1)}% (Coeff. Dérive)`;
    if ($('q-mult-env')) $('q-mult-env').textContent = `x${q_mult.toFixed(1)} (Rob. Env.)`;

    // Météo (correction gracieuse)
    const temp_c = lastT_K !== null ? lastT_K - 273.15 : null;
    if ($('temp-air')) $('temp-air').textContent = temp_c !== null ? `${temp_c.toFixed(1)} °C` : 'N/A';
    if ($('pressure')) $('pressure').textContent = lastP_hPa !== null ? `${lastP_hPa.toFixed(2)} hPa` : 'N/A';
    if ($('humidity')) $('humidity').textContent = lastH_perc !== null ? `${lastH_perc.toFixed(1)} %` : 'N/A';
    
    // SAUVEGARDE DES VALEURS POUR LA PROCHAINE ITÉRATION
    lPos = pos; 
    lPos.timestamp = cTimePos; 
    lPos.kAlt_old = kAlt_new; 
}


// ===========================================
// INITIALISATION DES ÉVÉNEMENTS DOM
// ===========================================

document.addEventListener('DOMContentLoaded', () => {
    loadPrecisionRecords();
    syncH(); 
    
    // Initialisation de la carte (simplifiée)
    if ($('mapid')) {
        const map = L.map('mapid').setView([0, 0], 2);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: '© OpenStreetMap contributors'
        }).addTo(map);
    }
    
    // Boucle de mise à jour lente DOM (Énergie, Puissance, Astro)
    if (domID === null) {
        domID = setInterval(() => {
            if (lPos) {
                updateAstro(lPos.coords.latitude, lPos.coords.longitude);
                
                // Calculs d'Énergie/Puissance
                const speed_ms = Math.abs(kSpd);
                const massElement = document.getElementById('mass-input');
                const mass = massElement ? parseFloat(massElement.value) : 70; 
                if ($('kinetic-energy')) $('kinetic-energy').textContent = `${(0.5 * mass * speed_ms * speed_ms).toFixed(2)} J`;
                if ($('mechanical-power')) $('mechanical-power').textContent = `${(mass * a_sensor_longitudinal * speed_ms).toFixed(2)} W`;
            } else {
                updateAstro(null, null);
            }
            savePrecisionRecords();
        }, 1000); 
    }
    
    // --- BOUCLE DE MISE À JOUR MÉTÉO (Lente - toutes les 2 minutes) ---
    let weatherID = null;
    if (weatherID === null) {
        weatherID = setInterval(() => {
            if (lPos) fetchWeather(lPos.coords.latitude, lPos.coords.longitude);
        }, 120000); 
    }
    
    // Événements de contrôle
    $('toggle-gps-btn').addEventListener('click', () => gpsWatchID ? stopGPS() : startGPS());
    $('emergency-stop-btn').addEventListener('click', () => {
        emergencyStopActive = !emergencyStopActive;
        $('emergency-stop-btn').textContent = emergencyStopActive ? '🛑 Arrêt d\'urgence: ACTIF 🔴' : '🛑 Arrêt d\'urgence: INACTIF 🟢';
        $('emergency-stop-btn').style.backgroundColor = emergencyStopActive ? '#f8d7da' : '#dc3545';
        if (emergencyStopActive) stopGPS();
    });

    // Événement de sélection de l'environnement
    if ($('env-select')) $('env-select').addEventListener('change', (e) => { 
        if (emergencyStopActive) return;
        selectedEnvironment = e.target.value; 
        const factor = ENVIRONMENT_FACTORS[selectedEnvironment];
        if ($('env-factor')) $('env-factor').textContent = `${selectedEnvironment} (R:x${factor.R_MULT.toFixed(1)}, Q:x${factor.Q_MULT.toFixed(1)})`; 
    });

    // Autres boutons de réinitialisation (simplifiés)
    $('reset-dist-btn').addEventListener('click', () => { distM = 0; timeMoving = 0; });
    $('reset-max-btn').addEventListener('click', () => { maxSpd = 0; maxGForce = 0; savePrecisionRecords(); });
    $('reset-all-btn').addEventListener('click', () => { 
        if (confirm("Êtes-vous sûr de vouloir tout réinitialiser (dist/max/records) ?")) {
            localStorage.removeItem(P_RECORDS_KEY);
            distM = 0; timeMoving = 0; maxSpd = 0; maxGForce = 0;
            kSpd = 0; kUncert = 1000; kAltUncert = 1000; 
            location.reload(); 
        }
    });
});
