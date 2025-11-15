// =================================================================
// BLOC 1/4 : ekf_logic.js
// Constantes de base, filtres EKF (Vitesse/Altitude) et fonctions de calcul physique/mathématique.
// =================================================================

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const R_E_BASE = 6371000;   // Rayon terrestre moyen (m)
const KMH_MS = 3.6;         // Conversion m/s vers km/h
const C_S_BASE = 340.29;    // Vitesse du son standard (m/s)
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const R_AIR = 287.058;      // Constante spécifique de l'air sec (J/kg·K)
const KELVIN_OFFSET = 273.15; // Conversion Celsius vers Kelvin

// --- PARAMÈTRES DU FILTRE DE KALMAN (VITESSE/POSITION) ---
const Q_NOISE = 0.5;        // Bruit de processus (Réglage avionic: 0.5)
const R_MIN = 0.01;         // Bruit de mesure minimum
const R_MAX = 500.0;        // Bruit de mesure maximum
const MAX_ACC = 500.0;      // PRÉCISION MAX GPS (m) avant "Estimation Seule" (Corrigé à 500m)
const MIN_SPD = 0.05;       // Vitesse minimale pour considérer le mouvement (m/s)
const MIN_DT = 0.01;        // Intervalle de temps minimum (s)
const ALT_TH = 10.0;        // Seuil d'altitude pour le mode souterrain (m)
const ZUPT_RAW_THRESHOLD = 0.3;     // Seuil de vitesse EKF pour ZUPT (m/s)
const ZUPT_ACCEL_TOLERANCE = 0.5;   // Tolérance d'accélération pour ZUPT (m/s²)
const NETHER_RATIO = 8.0;   // Rapport de distance Nether:Overworld

// --- ÉTAT DU FILTRE EKF (Simplifié pour la démonstration) ---
let currentEKFState = {
    lat: 0.0,
    lon: 0.0,
    alt: 0.0,
    V_n: 0.0, 
    V_e: 0.0,
    V_d: 0.0, 
    acc_est: 10.0, 
    P: [[]] // Matrice de covariance (Simplifiée)
};

// --- STRUCTURES DE DONNÉES GLOBALES ---
const CELESTIAL_DATA = {
    'EARTH': { G: 9.80665, R: 6371000 },
    'MARS': { G: 3.72076, R: 3389500 },
    'MOON': { G: 1.625, R: 1737400 },
    'ROTATING': { G: 0.0, R: 0.0 }
};

const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0, DISPLAY: 'Normal' },
    'FOREST': { R_MULT: 2.5, DISPLAY: 'Forêt Dense' },
    'CONCRETE': { R_MULT: 7.0, DISPLAY: 'Canyon Urbain' },
    'METAL': { R_MULT: 5.0, DISPLAY: 'Structure Métallique' }
};

// --- FONCTIONS MATHÉMATIQUES SIMPLIFIÉES ---
const $ = id => document.getElementById(id);

function getCDate(lServH, lLocH) {
    if (lServH && lLocH) {
        // Retourne une Date ajustée par l'offset NTP
        return new Date(Date.now() - (lLocH - lServH));
    }
    return new Date(); // Retourne l'heure locale si l'NTP n'est pas synchro
}

// --- FONCTIONS DE CALCUL PHYSIQUE ---
function calculateLocalSpeed(tempC, speedMS) {
    const tempK = tempC + KELVIN_OFFSET;
    const C_S_local = Math.sqrt(1.4 * R_AIR * tempK); // Formule vitesse du son
    return {
        C_S_local: C_S_local,
        mach: speedMS / C_S_local
    };
}

function calculateMRF(alt, isNether) {
    // Calcul du Multiplier Ratio Factor (MRF)
    if (isNether) return NETHER_RATIO;
    if (alt < ALT_TH) return 0.5; // Exemple de facteur souterrain
    return 1.0;
}

function updateCelestialBody(bodyKey, alt, rotRadius, angVel) {
    let G_ACC = CELESTIAL_DATA[bodyKey].G;
    let R_ALT_CENTER_REF = CELESTIAL_DATA[bodyKey].R;
    
    if (bodyKey === 'ROTATING') {
        // Gravité effective dans un référentiel non inertiel (force centrifuge)
        G_ACC = G_ACC - Math.pow(angVel, 2) * rotRadius;
        R_ALT_CENTER_REF = R_E_BASE; 
    }
    return { G_ACC, R_ALT_CENTER_REF };
}

function getGravityLocal(alt, bodyKey, rotationRadius, angularVelocity) {
    if (bodyKey === 'ROTATING') {
        return CELESTIAL_DATA['EARTH'].G - (Math.pow(angularVelocity, 2) * rotationRadius);
    }
    const G_PLANET = CELESTIAL_DATA[bodyKey].G;
    const R_PLANET = CELESTIAL_DATA[bodyKey].R;
    // G locale décroît avec l'altitude (approximation)
    if (R_PLANET > 0) {
        return G_PLANET * Math.pow(R_PLANET / (R_PLANET + alt), 2);
    }
    return G_PLANET;
    }
// =================================================================
// BLOC 2/4 : astro_weather_ntp.js
// Fonctions pour la synchronisation NTP, l'heure Minecraft, les calculs Astro/Soleil/Lune et Météo.
// Dépendances: Fonctions externes (SunCalc, fetch)
// =================================================================

// --- FONCTIONS DE SYNCHRONISATION NTP ---

async function syncH(lServH, lLocH) {
    // NOTE: Simule la logique de synchronisation NTP sans API externe
    return new Promise((resolve) => {
        const localTimeBefore = Date.now();
        
        // Simuler un appel serveur pour l'heure (ici: localTime + 150ms d'offset)
        const serverTime = Date.now() + 150; 
        
        const localTimeAfter = Date.now();
        const latency = (localTimeAfter - localTimeBefore) / 2;
        
        // Estimation de l'heure du serveur ajustée
        const newServH = serverTime + latency;
        const newLocH = localTimeAfter;
        
        resolve({ lServH: newServH, lLocH: newLocH });
    });
}

// --- FONCTIONS MÉTÉO (Simulation d'Appel API) ---

async function fetchWeather(lat, lon) {
    // Simule des données de météo réalistes.
    try {
        const tempC = 19.0;
        const pressure_hPa = 1012.0;
        const humidity_perc = 82.0;
        
        const tempK = tempC + KELVIN_OFFSET;
        const pv = 6.1078 * Math.pow(10, (7.5 * tempC) / (tempC + 237.3)); 
        const rh_ratio = humidity_perc / 100.0;
        const p_v = rh_ratio * pv; 
        const p_d = pressure_hPa * 100 - p_v; 
        const air_density = (p_d / (R_AIR * tempK)) + (p_v / (461.495 * tempK));
        const dew_point = 237.3 * Math.log(rh_ratio * Math.exp((17.27 * tempC) / (237.3 + tempC))) / (17.27 - Math.log(rh_ratio * Math.exp((17.27 * tempC) / (237.3 + tempC))));

        return {
            tempC: tempC,
            tempK: tempK,
            pressure_hPa: pressure_hPa,
            humidity_perc: humidity_perc,
            air_density: air_density,
            dew_point: dew_point,
            status: "Dégagé" 
        };
    } catch (e) {
        console.error("Erreur de simulation de météo:", e);
        return null;
    }
}

// --- FONCTIONS ASTRO ET TEMPS MINECRAFT (Dépendance SunCalc) ---

function getMinecraftTime(now) {
    const minutesInMinecraftDay = 20;
    const msInMinecraftDay = minutesInMinecraftDay * 60 * 1000;
    const msSinceEpoch = now.getTime();
    
    const midnightOffsetMs = 6 * 3600 * 1000;
    
    const timeInCycle = (msSinceEpoch - midnightOffsetMs) % msInMinecraftDay;
    const mcHours = Math.floor(timeInCycle / (msInMinecraftDay / 24));
    const mcMinutes = Math.floor((timeInCycle % (msInMinecraftDay / 24)) / (msInMinecraftDay / (24 * 60)));
    
    return `${String(mcHours).padStart(2, '0')}:${String(mcMinutes).padStart(2, '0')}`;
}

function updateAstro(lat, lon, lServH, lLocH) {
    // NOTE: Nécessite l'objet global 'SunCalc' pour fonctionner
    if (typeof SunCalc === 'undefined' || lat === 0 || lon === 0) return null;
    
    const now = getCDate(lServH, lLocH); 
    const sunPos = SunCalc.getPosition(now, lat, lon);
    const moonIllum = SunCalc.getMoonIllumination(now);
    const moonPos = SunCalc.getMoonPosition(now, lat, lon);
    const sunTimes = SunCalc.getTimes(now, lat, lon);
    
    // Calculs solaires (simplifié pour EOT, MST, TST)
    const EOT_min = 15.55; // Simplification de l'équation du temps
    const EOT_hours = EOT_min / 60;
    const MST_hours = 12 + (lon / 15); 
    const TST_hours = MST_hours + EOT_hours; 
    const noonSolar = new Date(now.getFullYear(), now.getMonth(), now.getDate(), 12 - (lon/15) + EOT_hours).toTimeString().slice(0, 8); 
    
    const decimalToTime = (h) => {
        const h_int = Math.floor(h) % 24;
        const m_int = Math.floor((h % 1) * 60);
        const s_int = Math.floor((((h % 1) * 60) % 1) * 60);
        return `${String(h_int).padStart(2, '0')}:${String(m_int).padStart(2, '0')}:${String(s_int).padStart(2, '0')}`;
    }

    return {
        now, 
        sunPos, 
        moonIllum, 
        moonPos, 
        sunTimes,
        solarTimes: {
            TST: decimalToTime(TST_hours),
            MST: decimalToTime(MST_hours),
            EOT: EOT_min.toFixed(2),
            ECL_LONG: (sunPos.azimuth * R2D + 180).toFixed(2), 
            NoonSolar: noonSolar
        }
    };
}

function getMoonPhaseName(phase) {
    if (phase < 0.06 || phase > 0.94) return "Nouvelle Lune";
    if (phase < 0.25) return "Premier Croissant";
    if (phase < 0.44) return "Premier Quartier";
    if (phase < 0.56) return "Pleine Lune";
    if (phase < 0.75) return "Gibbeuse Décroissante";
    if (phase < 0.94) return "Dernier Quartier";
    return "Phase Inconnue";
                       }
// =================================================================
// BLOC 3/4 : ekf_core.js
// Implémentation du cœur du Filtre de Kalman Étendu (EKF)
// Dépendances: Constantes de BLOC 1/4 (D2R, Q_NOISE, R_MIN, R_E_BASE, etc.)
// =================================================================

// --- COVARIANCE ET INCERTITUDE (Simplification: P représenté par des scalaires) ---
let P_pos_2D = 100.0;  // Incertitude position (m²)
let P_vel_3D = 0.625;  // Incertitude vitesse (m²/s²)
let currentEnvFactor = 1.0; // Facteur environnemental (doit être mis à jour par 4B/4)

// --------------------------------------------------------------------------
// --- INITIALISATION DU FILTRE ---
// --------------------------------------------------------------------------

function initEKF(lat, lon, alt, acc) {
    // Initialise l'état EKF et la covariance
    currentEKFState.lat = lat;
    currentEKFState.lon = lon;
    currentEKFState.alt = alt;
    currentEKFState.V_n = 0.0;
    currentEKFState.V_e = 0.0;
    currentEKFState.V_d = 0.0;
    currentEKFState.acc_est = acc;

    // Initialisation des incertitudes
    P_pos_2D = acc * acc; 
    P_vel_3D = Q_NOISE * 2; 
}

// --------------------------------------------------------------------------
// --- ÉTAPE DE PRÉDICTION (Propagated State) ---
// --------------------------------------------------------------------------

function predictEKF(dt, acc_imu, gyro, G_ACC, R_ALT_CENTER_REF) {
    const Vn = currentEKFState.V_n;
    const Ve = currentEKFState.V_e;
    const lat = currentEKFState.lat;

    // --- 1. Prédiction de l'État (X) ---
    // Accélération dans le référentiel NED (Très simplifié: acc_imu[2] est Z)
    const accel_n = acc_imu[0]; 
    const accel_e = acc_imu[1]; 
    const accel_d = acc_imu[2] - G_ACC; // Accélération verticale nette (imu_z - g)

    // Mise à jour de la Vitesse (V = V + a * dt)
    currentEKFState.V_n += accel_n * dt;
    currentEKFState.V_e += accel_e * dt;
    currentEKFState.V_d += accel_d * dt;

    // Mise à jour de la Position (P = P + V * dt)
    // Conversion de la vitesse NED en changements de lat/lon/alt
    const R_MERIDIAN = R_ALT_CENTER_REF; 
    const R_TRANSVERSE = R_ALT_CENTER_REF * Math.cos(lat * D2R); 

    currentEKFState.lat += (Vn * dt) / R_MERIDIAN * R2D;
    currentEKFState.lon += (Ve * dt) / R_TRANSVERSE * R2D;
    currentEKFState.alt -= currentEKFState.V_d * dt; // 'D' est Down, donc la position 'alt' diminue

    // --- 2. Prédiction de l'Incertitude (P) ---
    // Le bruit de processus Q augmente l'incertitude avec le temps
    P_pos_2D += Q_NOISE * dt * 0.1; 
    P_vel_3D += Q_NOISE * dt; 
}

// --------------------------------------------------------------------------
// --- ÉTAPE DE CORRECTION GNSS ---
// --------------------------------------------------------------------------

function updateEKF_GNSS(gnss_pos, gnss_vel, accRaw, altAcc) {
    // Bruit de mesure (R) - Dépend de la précision GPS (accRaw)
    const R_dynamic = accRaw * accRaw * currentEnvFactor; 

    // --- 1. Correction de la Position ---
    // Gain de Kalman implicite (k = P / (P + R))
    const K_pos = P_pos_2D / (P_pos_2D + R_dynamic);
    
    // Mise à jour de l'état
    currentEKFState.lat += K_pos * (gnss_pos.lat - currentEKFState.lat);
    currentEKFState.lon += K_pos * (gnss_pos.lon - currentEKFState.lon);
    
    // L'altitude utilise une précision différente
    const K_alt = P_pos_2D / (P_pos_2D + altAcc * altAcc);
    currentEKFState.alt += K_alt * (gnss_pos.alt - currentEKFState.alt);

    // Mise à jour de l'Incertitude
    P_pos_2D = (1 - K_pos) * P_pos_2D;
    currentEKFState.acc_est = Math.sqrt(P_pos_2D); 

    // --- 2. Correction de la Vitesse (si Vitesse GPS est disponible) ---
    if (gnss_vel.V_n !== null && gnss_vel.V_n !== undefined) {
        const R_vel = R_dynamic * 0.1; // La mesure de vitesse est considérée 10x plus stable
        const K_vel = P_vel_3D / (P_vel_3D + R_vel);

        // Z_vel[0] est la vitesse GPS mesurée
        currentEKFState.V_n += K_vel * (gnss_vel.V_n - currentEKFState.V_n);
        
        P_vel_3D = (1 - K_vel) * P_vel_3D;
    }
}

// --------------------------------------------------------------------------
// --- ZUPT (Zero Velocity Update) ---
// --------------------------------------------------------------------------

function updateEKF_ZUPT() {
    // L'objet est immobile, forcer la vitesse à zéro
    currentEKFState.V_n = 0.0;
    currentEKFState.V_e = 0.0;
    currentEKFState.V_d = 0.0;

    // Réduire fortement l'incertitude de vitesse
    P_vel_3D = Q_NOISE * 0.1; 
}

// --------------------------------------------------------------------------
// --- FONCTIONS UTILITAIRES DE L'ÉTAT EKF ---
// --------------------------------------------------------------------------

function updateEKFReadableState() {
    // Fonction d'architecture: L'état est déjà lisible dans cette implémentation simplifiée.
}

function getEKFVelocity3D() {
    return Math.sqrt(currentEKFState.V_n * currentEKFState.V_n + 
                     currentEKFState.V_e * currentEKFState.V_e + 
                     currentEKFState.V_d * currentEKFState.V_d);
}

function getVelocityUncertainty() { 
    return P_vel_3D; 
}
// =================================================================
// BLOC 4A/4 : AppController.js (Logique Principale, EKF, Contrôles Rapides)
// =================================================================

// --- CONSTANTES DE CONFIGURATION SYSTÈME ---
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};
const DOM_SLOW_UPDATE_MS = 1000; 
let lastMapUpdate = 0; 
const MAP_UPDATE_INTERVAL = 3000; 
const DEFAULT_INIT_LAT = 45.749950;
const DEFAULT_INIT_LON = 4.850027;
const DEFAULT_INIT_ALT = 2.64;

// --- VARIABLES D'ÉTAT (Globales) ---
let wID = null, domID = null, lPos = null, sTime = null;
let distM_3D = 0, maxSpd = 0;
let timeMoving = 0; 
let lServH = null, lLocH = null;    // Horloge NTP
let lastFSpeed = 0; 

// État Physique et Contrôles
let currentGPSMode = 'HIGH_FREQ'; 
let emergencyStopActive = false;
let netherMode = false; 
let selectedEnvironment = 'NORMAL'; 
let currentMass = 70.0; 
let R_FACTOR_RATIO = 1.0;
let currentCelestialBody = 'EARTH';
let rotationRadius = 100;
let angularVelocity = 0.0; 
let gpsAccuracyOverride = 0.0; 
let G_ACC = CELESTIAL_DATA['EARTH'].G;      
let R_ALT_CENTER_REF = R_E_BASE;            
// currentEnvFactor est définie dans BLOC 3/4 mais est mise à jour ici

// Données externes et IMU
let lastP_hPa = null, lastT_K = null, lastH_perc = null; 
let real_accel_x = 0, real_accel_y = 0, real_accel_z = 0;
let lastAccelLong = 0;

// Objets Map (Leaflet)
let map, marker, circle;


// --------------------------------------------------------------------------
// --- GESTION DES CAPTEURS IMU ---
// --------------------------------------------------------------------------

function imuMotionHandler(event) {
    if (event.acceleration) {
        real_accel_x = event.acceleration.x || 0;
        real_accel_y = event.acceleration.y || 0;
        real_accel_z = event.acceleration.z || 0;
        if ($('imu-status')) $('imu-status').textContent = "Actif (Sans Gravité)";
    } 
    else if (event.accelerationIncludingGravity) {
        real_accel_x = event.accelerationIncludingGravity.x || 0;
        real_accel_y = event.accelerationIncludingGravity.y || 0;
        real_accel_z = event.accelerationIncludingGravity.z || 0;
        if ($('imu-status')) $('imu-status').textContent = "Actif (Avec Gravité)";
    } else {
        if ($('imu-status')) $('imu-status').textContent = "Erreur (Capteur N/A)";
    }
}

function startIMUListeners() {
    if (window.DeviceMotionEvent) {
        if (typeof DeviceMotionEvent.requestPermission === 'function') {
            DeviceMotionEvent.requestPermission().then(permissionState => {
                if (permissionState === 'granted') {
                    window.addEventListener('devicemotion', imuMotionHandler);
                }
            }).catch(console.error);
        } else {
            window.addEventListener('devicemotion', imuMotionHandler);
        }
    } else {
         if ($('imu-status')) $('imu-status').textContent = "Non supporté";
    }
}

function stopIMUListeners() {
    if (window.DeviceMotionEvent) {
        window.removeEventListener('devicemotion', imuMotionHandler);
    }
    if ($('imu-status')) $('imu-status').textContent = "Inactif";
}

// --------------------------------------------------------------------------
// --- GESTION DES CARTES ---
// --------------------------------------------------------------------------

function initMap() {
    try {
        if ($('map') && typeof L !== 'undefined') { 
            map = L.map('map').setView([DEFAULT_INIT_LAT, DEFAULT_INIT_LON], 12);
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                attribution: '© OpenStreetMap contributors'
            }).addTo(map);
            marker = L.marker([DEFAULT_INIT_LAT, DEFAULT_INIT_LON]).addTo(map);
            circle = L.circle([DEFAULT_INIT_LAT, DEFAULT_INIT_LON], { color: 'red', fillColor: '#f03', fillOpacity: 0.5, radius: 10 }).addTo(map);
        }
    } catch (e) {
        console.error("Erreur d'initialisation de Leaflet (Carte):", e);
        if ($('map')) $('map').innerHTML = "Erreur d'initialisation de la carte. (Leaflet N/A)";
    }
}

function updateMap(lat, lon, acc) {
    if (map && marker) {
        marker.setLatLng([lat, lon]);
        circle.setLatLng([lat, lon]).setRadius(acc * R_FACTOR_RATIO); 
        const now = Date.now();
        if (now - lastMapUpdate > MAP_UPDATE_INTERVAL && getEKFVelocity3D() > MIN_SPD) {
            map.setView([lat, lon], map.getZoom() > 10 ? map.getZoom() : 16); 
            lastMapUpdate = now;
        } else if (map.getZoom() < 10 && (Date.now() - lastMapUpdate > 5000)) {
            map.setView([lat, lon], 12);
            lastMapUpdate = now;
        }
    }
}

// --------------------------------------------------------------------------
// --- FONCTIONS DE CONTRÔLE GPS & SYSTÈME ---
// --------------------------------------------------------------------------

function setGPSMode(mode) {
    currentGPSMode = mode;
    if (wID !== null) {
        stopGPS(false); 
        startGPS();     
    }
    if ($('freq-select')) $('freq-select').value = mode; 
}

function startGPS() {
    if (wID !== null) return; 
    
    const options = (currentGPSMode === 'HIGH_FREQ') ? GPS_OPTS.HIGH_FREQ : GPS_OPTS.LOW_FREQ;
    
    wID = navigator.geolocation.watchPosition(updateDisp, handleErr, options); 
    startIMUListeners(); 
    
    if ($('toggle-gps-btn')) {
        $('toggle-gps-btn').textContent = '⏸️ PAUSE GPS';
        $('toggle-gps-btn').style.backgroundColor = '#ffc107'; 
    }
}

function stopGPS(resetButton = true) {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
    }
    stopIMUListeners(); 
    
    if (resetButton) {
        if ($('toggle-gps-btn')) {
            $('toggle-gps-btn').textContent = '▶️ MARCHE GPS';
            $('toggle-gps-btn').style.backgroundColor = '#28a745'; 
        }
    }
}

function emergencyStop() {
    emergencyStopActive = true;
    stopGPS(false);
    if ($('emergency-stop-btn')) {
        $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: ACTIF 🔴";
        $('emergency-stop-btn').classList.add('active');
    }
    ['speed-stable', 'speed-3d-inst', 'distance-total-km', 'local-time'].forEach(id => {
        if ($(id)) $(id).textContent = 'ARRÊT D’URGENCE';
    });
}

function resumeSystem() {
    emergencyStopActive = false;
    if ($('emergency-stop-btn')) {
        $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: INACTIF 🟢";
        $('emergency-stop-btn').classList.remove('active');
    }
    startGPS();
}

function handleErr(err) {
    console.warn(`ERREUR GPS (${err.code}): ${err.message}`);
    if ($('gps-precision')) $('gps-precision').textContent = `Erreur: ${err.message}`;
    
    if (err.code === 1) { 
        stopGPS();
        alert("Accès à la géolocalisation refusé. Veuillez l'activer.");
    }
}


// --------------------------------------------------------------------------
// --- FONCTION PRINCIPALE DE MISE À JOUR GPS/EKF (updateDisp) ---
// --------------------------------------------------------------------------

function updateDisp(pos) {
    if (emergencyStopActive) return;

    // --- 1. ACQUISITION DES DONNÉES ET INITIALISATION ---
    const cTimePos = pos.timestamp;
    const now = getCDate(lServH, lLocH); 
    
    if (now === null) return; 
    if (sTime === null) sTime = now.getTime();
    
    let accRaw = pos.coords.accuracy;
    if (gpsAccuracyOverride > 0.0) accRaw = gpsAccuracyOverride;

    let dt = 0;
    if (lPos) {
        dt = (cTimePos - lPos.timestamp) / 1000;
    } else {
        // Première position: Initialisation de l'EKF
        lPos = pos; 
        initEKF(pos.coords.latitude, pos.coords.longitude, pos.coords.altitude || DEFAULT_INIT_ALT, accRaw);
        updateMap(currentEKFState.lat, currentEKFState.lon, currentEKFState.acc_est);
        return; 
    }
    
    if (dt < MIN_DT || dt > 10) { 
        lPos = pos; 
        return; 
    }
    
    // 2. PRÉDICTION EKF (Basée sur l'IMU et le temps)
    predictEKF(dt, [real_accel_x, real_accel_y, real_accel_z], [0, 0, 0], G_ACC, R_ALT_CENTER_REF);
    
    // 3. LOGIQUE ZUPT
    const V_ekf = getEKFVelocity3D();
    const isPlausiblyStopped = (
        V_ekf < ZUPT_RAW_THRESHOLD && 
        Math.abs(lastAccelLong) < ZUPT_ACCEL_TOLERANCE
    ); 
    
    if (isPlausiblyStopped) { 
        updateEKF_ZUPT();
    }
    
    // 4. CORRECTION EKF (Fusion GNSS)
    const isSignalLost = (accRaw > MAX_ACC);
    currentEnvFactor = ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT;
    
    if (!isSignalLost) {
        const gnss_pos = { lat: pos.coords.latitude, lon: pos.coords.longitude, alt: pos.coords.altitude || currentEKFState.alt };
        const gnss_vel = { V_n: pos.coords.speed || pos.coords.velocity || 0, V_e: 0, V_d: 0 }; 
        
        updateEKF_GNSS(gnss_pos, gnss_vel, accRaw, pos.coords.altitudeAccuracy || 5.0);
    }
    
    // 5. MISE À JOUR DE L'ÉTAT LISIBLE ET CALCULS PHYSIQUES
    updateEKFReadableState(); 
    const sSpdFE = V_ekf < MIN_SPD ? 0 : V_ekf; 
    
    let accel_long = 0;
    if (dt > 0.05) { 
        accel_long = (sSpdFE - lastFSpeed) / dt;
    }
    lastFSpeed = sSpdFE;
    lastAccelLong = accel_long;

    R_FACTOR_RATIO = calculateMRF(currentEKFState.alt, netherMode); 
    distM_3D += sSpdFE * dt * R_FACTOR_RATIO; 
    
    if (sSpdFE > MIN_SPD) { timeMoving += dt; }
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    
    const local_g = getGravityLocal(currentEKFState.alt, currentCelestialBody, rotationRadius, angularVelocity); 
    const coriolis_force = 2 * currentMass * sSpdFE * OMEGA_EARTH * Math.sin(currentEKFState.lat * D2R);
    
    let mach_info = { C_S_local: C_S_BASE, mach: 0 };
    if (lastT_K !== null) {
        mach_info = calculateLocalSpeed(lastT_K - KELVIN_OFFSET, sSpdFE);
    }

    // 6. MISE À JOUR DU DOM (Affichage Rapide)
    const acc_est_m = currentEKFState.acc_est;
    const kalman_V_uncert = getVelocityUncertainty();
    const R_dyn = currentEnvFactor * accRaw ** 2; 
    const altStatusTxt = currentEKFState.alt < ALT_TH ? `OUI (< ${ALT_TH}m)` : 'Non';
    const status_mode = isPlausiblyStopped ? '✅ ZUPT' : (isSignalLost ? '⚠️ EST. SEULE' : '🚀 FUSION');

    if ($('time-elapsed')) $('time-elapsed').textContent = `${((Date.now() - sTime) / 1000).toFixed(2)} s`;
    if ($('time-moving')) $('time-moving').textContent = `${timeMoving.toFixed(2)} s`;
    if ($('mode-ratio')) $('mode-ratio').textContent = `${R_FACTOR_RATIO.toFixed(3)} (Ratio)`;
    if ($('env-factor')) $('env-factor').textContent = `${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY} (x${currentEnvFactor.toFixed(1)})`;
    if ($('speed-stable')) $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(5)}`;
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${sSpdFE.toFixed(3)} m/s | ${(sSpdFE * 1e6).toFixed(0)} µm/s | ${(sSpdFE * 1e9).toFixed(0)} nm/s`;
    
    // Vitesse Brute (Corrigé)
    const rawSpeed = pos.coords.speed !== null ? pos.coords.speed : (pos.coords.velocity !== null ? pos.coords.velocity : NaN);
    if ($('speed-raw-ms')) $('speed-raw-ms').textContent = !isNaN(rawSpeed) ? `${rawSpeed.toFixed(3)} m/s` : '-- m/s';
    
    if ($('speed-max')) $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    if ($('distance-total-km')) $('distance-total-km').textContent = `${(distM_3D / 1000).toFixed(3)} km | ${distM_3D.toFixed(2)} m`;
    if ($('latitude')) $('latitude').textContent = `${currentEKFState.lat.toFixed(6)} °`;
    if ($('longitude')) $('longitude').textContent = `${currentEKFState.lon.toFixed(6)} °`;
    if ($('altitude-gps')) $('altitude-gps').textContent = currentEKFState.alt !== null ? `${currentEKFState.alt.toFixed(2)} m` : 'N/A';
    if ($('heading-display')) $('heading-display').textContent = pos.coords.heading !== null ? `${pos.coords.heading.toFixed(1)} °` : 'N/A';
    if ($('underground-status')) $('underground-status').textContent = `Souterrain: ${altStatusTxt} (${status_mode} | Acc GPS: ${accRaw.toFixed(1)}m)`;
    if ($('gravity-local')) $('gravity-local').textContent = `${local_g.toFixed(5)} m/s²`;
    if ($('accel-long')) $('accel-long').textContent = `${accel_long.toFixed(3)} m/s²`;
    
    // Affichage des accélérations IMU (Corrigé: Ajout de Y et Z)
    if ($('imu-accel-x')) $('imu-accel-x').textContent = `${real_accel_x.toFixed(2)} m/s²`;
    if ($('imu-accel-y')) $('imu-accel-y').textContent = `${real_accel_y.toFixed(2)} m/s²`;
    if ($('imu-accel-z')) $('imu-accel-z').textContent = `${real_accel_z.toFixed(2)} m/s²`;
    
    if ($('kalman-uncert')) $('kalman-uncert').textContent = `${kalman_V_uncert.toFixed(3)} m²/s² (P)`;
    if ($('speed-error-perc')) $('speed-error-perc').textContent = `${R_dyn.toFixed(3)} m² (R dyn)`;
    if ($('gps-precision')) $('gps-precision').textContent = `${acc_est_m.toFixed(2)} m (Est.)`;
    if ($('coriolis-force')) $('coriolis-force').textContent = `${coriolis_force.toExponential(2)} N`;
    if ($('speed-sound-local')) $('speed-sound-local').textContent = `${mach_info.C_S_local.toFixed(2)} m/s`;
    if ($('mach-number')) $('mach-number').textContent = `${mach_info.mach.toFixed(4)}`;
    
    // 7. SAUVEGARDE & MISE À JOUR CARTE
    lPos = pos; 
    lPos.timestamp = cTimePos; 
    updateMap(currentEKFState.lat, currentEKFState.lon, acc_est_m);
}
// =================================================================
// BLOC 4B/4 : AppController.js (Fonctions DOM Lentes et Initialisation Globale)
// =================================================================

// --------------------------------------------------------------------------
// --- FONCTIONS DOM VISUELLES ET LENTES ---
// --------------------------------------------------------------------------

function updateClockVisualization(now, sunPos, moonPos, sunTimes) {
    const sunEl = $('sun-element');
    const moonEl = $('moon-element');
    const clockEl = $('minecraft-clock'); 

    if (!sunEl || !moonEl || !clockEl || !sunPos || !moonPos || !sunTimes) return;

    const sunIcon = sunEl.querySelector('.sun-icon');
    const moonIcon = moonEl.querySelector('.moon-icon');

    const altDegSun = sunPos.altitude * R2D;
    const aziDegSun = (sunPos.azimuth * R2D + 180) % 360; 
    sunEl.style.transform = `rotate(${aziDegSun}deg)`;
    const radialPercentSun = Math.min(50, Math.max(0, 50 * (90 - altDegSun) / 90));
    const altitudeOffsetPercentSun = 50 - radialPercentSun; 
    if (sunIcon) sunIcon.style.transform = `translateY(calc(-50% + ${altitudeOffsetPercentSun}%) )`; 
    sunEl.style.display = altDegSun > -0.83 ? 'flex' : 'none'; 

    const altDegMoon = moonPos.altitude * R2D;
    const aziDegMoon = (moonPos.azimuth * R2D + 180) % 360; 
    moonEl.style.transform = `rotate(${aziDegMoon}deg)`;
    const radialPercentMoon = Math.min(50, Math.max(0, 50 * (90 - altDegMoon) / 90));
    const altitudeOffsetPercentMoon = 50 - radialPercentMoon; 
    if (moonIcon) moonIcon.style.transform = `translateY(calc(-50% + ${altitudeOffsetPercentMoon}%) )`;
    moonEl.style.display = altDegMoon > 0 ? 'flex' : 'none';

    const body = document.body;
    body.classList.remove('sky-day', 'sky-sunset', 'sky-night', 'dark-mode', 'light-mode');
    clockEl.classList.remove('sky-day', 'sky-sunset', 'sky-night');

    const nowMs = now.getTime();
    let bodyClass;
    if (sunTimes.sunriseEnd && sunTimes.sunsetStart && sunTimes.dusk && sunTimes.dawn) {
        if (nowMs >= sunTimes.sunriseEnd.getTime() && nowMs < sunTimes.sunsetStart.getTime()) {
            bodyClass = 'sky-day';
        } else if (nowMs >= sunTimes.dusk.getTime() || nowMs < sunTimes.dawn.getTime()) {
            bodyClass = 'sky-night';
        } else {
            bodyClass = 'sky-sunset';
        }
    } else {
        bodyClass = 'sky-night';
    }
    
    body.classList.add(bodyClass);
    body.classList.add(bodyClass === 'sky-day' ? 'light-mode' : 'dark-mode');
    clockEl.classList.add(bodyClass); 

    $('clock-status').textContent = altDegSun > 0 ? 'Jour Solaire (☀️)' : 'Nuit/Crépuscule (🌙)';
}


// --------------------------------------------------------------------------
// --- INITIALISATION DES ÉVÉNEMENTS DOM ---
// --------------------------------------------------------------------------

document.addEventListener('DOMContentLoaded', () => {
    
    initMap(); 
    initEKF(DEFAULT_INIT_LAT, DEFAULT_INIT_LON, DEFAULT_INIT_ALT, 10.0);

    // --- Initialisation des Contrôles ---
    const massInput = $('mass-input'); 
    if (massInput) {
        massInput.addEventListener('input', () => { 
            currentMass = parseFloat(massInput.value) || 70.0; 
            if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        });
        currentMass = parseFloat(massInput.value); 
        if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
    }

    if ($('celestial-body-select')) {
        $('celestial-body-select').addEventListener('change', (e) => { 
            const newVals = updateCelestialBody(e.target.value, currentEKFState.alt, rotationRadius, angularVelocity);
            G_ACC = newVals.G_ACC;
            R_ALT_CENTER_REF = newVals.R_ALT_CENTER_REF;
            currentCelestialBody = e.target.value;
            if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC.toFixed(4)} m/s²`;
        });
    }

    const updateRotation = () => {
        rotationRadius = parseFloat($('rotation-radius')?.value) || 100;
        angularVelocity = parseFloat($('angular-velocity')?.value) || 0;
        if (currentCelestialBody === 'ROTATING') {
            const newVals = updateCelestialBody('ROTATING', currentEKFState.alt, rotationRadius, angularVelocity);
            G_ACC = newVals.G_ACC;
            if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC.toFixed(4)} m/s²`;
        }
    };
    if ($('rotation-radius')) $('rotation-radius').addEventListener('input', updateRotation);
    if ($('angular-velocity')) $('angular-velocity').addEventListener('input', updateRotation);

    if ($('environment-select')) {
        $('environment-select').addEventListener('change', (e) => { 
            if (emergencyStopActive) return;
            selectedEnvironment = e.target.value; 
            if ($('env-factor')) $('env-factor').textContent = `${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY} (x${ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT.toFixed(1)})`; 
        });
        if ($('env-factor')) $('env-factor').textContent = `${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY} (x${ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT.toFixed(1)})`;
    }
    
    if ($('gps-accuracy-override')) {
        $('gps-accuracy-override').addEventListener('change', (e) => {
            gpsAccuracyOverride = parseFloat(e.target.value) || 0.0;
        });
    }

    // --- Boutons Principaux ---
    if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', () => { 
        if (emergencyStopActive) { alert("Veuillez désactiver l'Arrêt d'urgence."); return; }
        wID === null ? startGPS() : stopGPS(); 
    });
    if ($('freq-select')) $('freq-select').addEventListener('change', (e) => setGPSMode(e.target.value));
    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', () => { 
        !emergencyStopActive ? emergencyStop() : resumeSystem(); 
    });
    if ($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        netherMode = !netherMode; 
        if ($('mode-nether')) $('mode-nether').textContent = netherMode ? `ACTIVÉ (1:${NETHER_RATIO}) 🔥` : "DÉSACTIVÉ (1:1)"; 
    });
    
    // --- Réinitialisations ---
    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        distM_3D = 0; timeMoving = 0; 
    });
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        maxSpd = 0; 
    });
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        if (confirm("Réinitialiser toutes les données de session, y compris l'EKF ?")) { 
            distM_3D = 0; maxSpd = 0; timeMoving = 0; lPos = null; sTime = null;
            initEKF(DEFAULT_INIT_LAT, DEFAULT_INIT_LON, DEFAULT_INIT_ALT, 10.0);
        } 
    });
    if ($('toggle-mode-btn')) $('toggle-mode-btn').addEventListener('click', () => {
        document.body.classList.toggle('dark-mode');
    });
    
    // --- DÉMARRAGE DU SYSTÈME ---
    
    const initVals = updateCelestialBody(currentCelestialBody, currentEKFState.alt, rotationRadius, angularVelocity);
    G_ACC = initVals.G_ACC;
    R_ALT_CENTER_REF = initVals.R_ALT_CENTER_REF;
    
    // CORRECTION NTP: Gestion de l'échec et démarrage garanti du GPS.
    syncH(lServH, lLocH).then(newTimes => {
        lServH = newTimes.lServH;
        lLocH = newTimes.lLocH;
        startGPS(); 
    }).catch(err => {
        console.warn("Échec de la synchronisation NTP. Démarrage du GPS avec heure locale.", err);
        if ($('local-time')) $('local-time').textContent = 'SYNCHRO ÉCHOUÉE';
        startGPS(); 
    });

    // Boucle de mise à jour lente (Astro/Météo/Horloge)
    if (domID === null) {
        domID = setInterval(async () => {
            const currentLat = currentEKFState.lat; 
            const currentLon = currentEKFState.lon;
            
            // 1. Météo : Récupération et Affichage
            if (currentLat !== 0 && currentLon !== 0 && !emergencyStopActive) {
                const data = await fetchWeather(currentLat, currentLon);
                
                if (data) {
                    lastP_hPa = data.pressure_hPa;
                    lastT_K = data.tempK;
                    lastH_perc = data.humidity_perc / 100.0;
                    
                    if ($('temp-air-2')) $('temp-air-2').textContent = `${data.tempC.toFixed(1)} °C`;
                    if ($('pressure-2')) $('pressure-2').textContent = `${data.pressure_hPa.toFixed(0)} hPa`;
                    if ($('humidity-2')) $('humidity-2').textContent = `${data.humidity_perc} %`;
                    if ($('air-density')) $('air-density').textContent = `${data.air_density.toFixed(3)} kg/m³`;
                    if ($('dew-point')) $('dew-point').textContent = `${data.dew_point.toFixed(1)} °C`;
                    if ($('weather-status')) $('weather-status').textContent = data.status; 
                } else {
                    if ($('weather-status')) $('weather-status').textContent = 'ÉCHEC RÉCUPÉRATION (Réseau ou API)';
                    lastP_hPa = lastT_K = lastH_perc = null; 
                }
            }
            
            // 2. Astro : Calculs et Affichage
            const astroData = updateAstro(currentLat, currentLon, lServH, lLocH);

            if (astroData) {
                const { now, sunPos, moonIllum, moonPos, sunTimes, solarTimes } = astroData;

                if ($('time-minecraft')) $('time-minecraft').textContent = getMinecraftTime(now);
                if ($('date-display-astro')) $('date-display-astro').textContent = now.toLocaleDateString('fr-FR');
                if ($('tst')) $('tst').textContent = solarTimes.TST;
                if ($('mst')) $('mst').textContent = solarTimes.MST;
                if ($('eot')) $('eot').textContent = solarTimes.EOT + ' min'; 
                if ($('ecl-long')) $('ecl-long').textContent = solarTimes.ECL_LONG + ' °'; 
                if ($('sun-alt')) $('sun-alt').textContent = `${(sunPos.altitude * R2D).toFixed(2)} °`;
                if ($('sun-azimuth')) $('sun-azimuth').textContent = `${(sunPos.azimuth * R2D).toFixed(2)} ° (S-O)`;
                if ($('moon-alt')) $('moon-alt').textContent = `${(moonPos.altitude * R2D).toFixed(2)} °`;
                if ($('moon-azimuth')) $('moon-azimuth').textContent = `${(moonPos.azimuth * R2D).toFixed(2)} ° (S-O)`;
                if ($('moon-illuminated')) $('moon-illuminated').textContent = `${(moonIllum.fraction * 100).toFixed(1)} %`;
                if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(moonIllum.phase);
                if ($('noon-solar')) $('noon-solar').textContent = solarTimes.NoonSolar;
                
                updateClockVisualization(now, sunPos, moonPos, sunTimes);
            } else {
                $('clock-status').textContent = 'Astro (Attente GPS/SunCalc)...';
            }
            
            // 3. Horloge NTP : Resynchronisation toutes les minutes
            if (Math.floor(Date.now() / 1000) % 60 === 0) {
                 syncH(lServH, lLocH).then(newTimes => {
                    lServH = newTimes.lServH;
                    lLocH = newTimes.lLocH;
                 }).catch(() => { /* Échec géré au démarrage */ });
            }
            
            // 4. Horloge locale (NTP) et UTC/GMT (Corrigé: Affichage UTC complet)
            const nowTime = getCDate(lServH, lLocH);
            if (nowTime) {
                // Heure Locale (NTP)
                if ($('local-time') && !$('local-time').textContent.includes('SYNCHRO ÉCHOUÉE')) {
                    $('local-time').textContent = nowTime.toLocaleTimeString('fr-FR');
                }
                
                // Date & Heure (UTC/GMT) - Corrigé pour afficher l'heure complète
                if ($('date-display')) {
                    const utcDateStr = nowTime.toLocaleDateString('fr-FR', { timeZone: 'UTC', day: '2-digit', month: '2-digit', year: 'numeric' });
                    const utcTimeStr = nowTime.toLocaleTimeString('fr-FR', { timeZone: 'UTC' });
                    $('date-display').textContent = `${utcDateStr} ${utcTimeStr}`;
                }
            }
            
        }, DOM_SLOW_UPDATE_MS); 
    }
});
