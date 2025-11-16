// =================================================================
// EKF CORE (1/4) : Logique de Filtrage Mathématique (15 États)
// =================================================================

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const G_EARTH = 9.80665;    // Gravité standard (m/s²)
const OMEGA_EARTH = 7.292115e-5; // Vitesse angulaire de la Terre (rad/s)
const R_E_BASE = 6371000;   // Rayon terrestre moyen (m)
const KMH_MS = 3.6;         // Conversion m/s vers km/h
const R_AIR = 287.058;      // Constante spécifique de l'air sec (J/kg·K)
const C_S_REF = 343.0;      // Vitesse du son de référence (m/s)

// --- PARAMÈTRES EKF (PROCESSUS ET BRUIT) ---
const NUM_STATES = 15;      // Taille du vecteur d'état (9 Nav + 6 Biais)
const Q_POS = 0.1;          // Bruit de processus Position (m²/s)
const Q_VEL = 0.5;          // Bruit de processus Vitesse (m²/s)
const Q_ATT = 0.001;        // Bruit de processus Attitude (rad²/s)
const Q_BIAS_ACC = 0.0001;  // Bruit de processus Biais Accél (m/s²/s)
const Q_BIAS_GYRO = 1e-6;   // Bruit de processus Biais Gyro (rad/s/s)
const R_POS_MIN = 2.0;      // Bruit de mesure min (GPS)
const R_VEL_MIN = 0.1;      // Bruit de mesure min (Vitesse GPS)
const MAX_GPS_ACCURACY = 25.0; // Précision max (m)
const MIN_DT = 0.01;        // Delta Temps minimum (s)

// --- STRUCTURE DE L'ÉTAT EKF ---
// Déclaré ici, initialisé dans main-initializer.js
const EKFState = {
    lat: 0.0, lon: 0.0, alt: 0.0,
    // X[0-2]: Pos_N, Pos_E, Pos_D
    // X[3-5]: Vel_N, Vel_E, Vel_D
    // X[6-8]: Roll, Pitch, Yaw
    // X[9-11]: Biais Accél. x, y, z
    // X[12-14]: Biais Gyro. x, y, z
    X: new Array(NUM_STATES).fill(0.0),
    P: Array.from({ length: NUM_STATES }, (_, i) => Array(NUM_STATES).fill(0.0).map((val, j) => {
        if (i !== j) return 0.0;
        if (i < 3) return 50.0; 
        if (i < 6) return 1.0;  
        if (i < 9) return 1.0;  
        if (i < 12) return 0.01; 
        if (i < 15) return 1e-4; 
        return 0.0;
    }))
};

/** Initialise l'état EKF (15 états). */
function initEKF(lat, lon, alt) {
    EKFState.lat = lat; EKFState.lon = lon; EKFState.alt = alt;
    EKFState.X.fill(0.0);
    EKFState.X[2] = -alt; 
    
    // Réinitialisation de la covariance (taille 15x15)
    for (let i = 0; i < NUM_STATES; i++) {
        for (let j = 0; j < NUM_STATES; j++) {
            EKFState.P[i][j] = i === j ? (i < 3 ? 50.0 : i < 6 ? 1.0 : i < 9 ? 1.0 : i < 12 ? 0.01 : 1e-4) : 0.0;
        }
    }
}

/** * ÉTAPE DE PRÉDICTION (IMU).
 * @param {number} dt Delta temps (s).
 * @param {number[]} a_b Accélération brute [ax, ay, az].
 * @param {number} G_ACC Gravité locale.
 */
function predictEKF_IMU(dt, a_b, G_ACC) {
    if (dt < MIN_DT) return;

    const X = EKFState.X;
    const phi = X[6], theta = X[7], psi = X[8]; 
    
    // Correction des mesures IMU avec les biais estimés (EKF 15 états)
    const a_corrected = [
        a_b[0] - X[9], 
        a_b[1] - X[10],
        a_b[2] - X[11] 
    ];

    // Matrice de rotation Corps vers Navigation (Simplified C_b^n)
    const C_bn = [
        [Math.cos(theta) * Math.cos(psi), Math.sin(phi) * Math.sin(theta) * Math.cos(psi) - Math.cos(phi) * Math.sin(psi), Math.cos(phi) * Math.sin(theta) * Math.cos(psi) + Math.sin(phi) * Math.sin(psi)],
        [Math.cos(theta) * Math.sin(psi), Math.sin(phi) * Math.sin(theta) * Math.sin(psi) + Math.cos(phi) * Math.cos(psi), Math.cos(phi) * Math.sin(theta) * Math.sin(psi) - Math.sin(phi) * Math.cos(psi)],
        [-Math.sin(theta), Math.sin(phi) * Math.cos(theta), Math.cos(phi) * Math.cos(theta)]
    ];

    // Accélération en référentiel Navigation (a_n)
    const g_n = [0, 0, G_ACC]; 
    const a_n = [0, 0, 0];
    for (let i = 0; i < 3; i++) {
        for (let j = 0; j < 3; j++) {
            a_n[i] += C_bn[i][j] * a_corrected[j]; 
        }
        a_n[i] += g_n[i]; 
    }

    // PROPAGATION DES ÉTATS DE NAVIGATION
    X[0] += X[3] * dt; X[1] += X[4] * dt; X[2] += X[5] * dt; 
    X[3] += a_n[0] * dt; X[4] += a_n[1] * dt; X[5] += a_n[2] * dt; 

    // PROPAGATION DE LA COVARIANCE P (Simplifiée : Matrice de bruit de processus Q diagonale)
    EKFState.P[0][0] += Q_POS * dt; EKFState.P[1][1] += Q_POS * dt; EKFState.P[2][2] += Q_POS * dt;
    EKFState.P[3][3] += Q_VEL * dt; EKFState.P[4][4] += Q_VEL * dt; EKFState.P[5][5] += Q_VEL * dt;
    EKFState.P[6][6] += Q_ATT * dt; EKFState.P[7][7] += Q_ATT * dt; EKFState.P[8][8] += Q_ATT * dt;
    
    // Propagation du bruit pour les Biais
    EKFState.P[9][9] += Q_BIAS_ACC * dt; EKFState.P[10][10] += Q_BIAS_ACC * dt; EKFState.P[11][11] += Q_BIAS_ACC * dt;
    EKFState.P[12][12] += Q_BIAS_GYRO * dt; EKFState.P[13][13] += Q_BIAS_GYRO * dt; EKFState.P[14][14] += Q_BIAS_GYRO * dt;
}

/** ÉTAPE DE CORRECTION (GNSS) */
function updateEKF_GNSS(gnss_m, R_pos_cov) {
    if (gnss_m.acc > MAX_GPS_ACCURACY) return;

    const X = EKFState.X;
    const P = EKFState.P;
    
    // Gains de Kalman simplifiés
    const K_pos = P[0][0] / (P[0][0] + R_pos_cov);
    const K_vel = P[3][3] / (P[3][3] + R_VEL_MIN);
    const K_bias = Math.sqrt(K_pos) * 0.005; // Facteur de correction pour les biais

    // Correction des coordonnées Lat/Lon/Alt (Affichage)
    EKFState.lat += K_pos * (gnss_m.lat - EKFState.lat);
    EKFState.lon += K_pos * (gnss_m.lon - EKFState.lon);
    EKFState.alt += K_pos * (gnss_m.alt - EKFState.alt); 
    
    // Correction de la Vitesse (NED)
    if (gnss_m.V_n !== null) {
        X[3] += K_vel * (gnss_m.V_n - X[3]);
        X[4] += K_vel * (gnss_m.V_e - X[4]);
        X[5] += K_vel * (gnss_m.V_d - X[5]);
    }
    
    // Correction des Biais Accél. (via l'erreur de vitesse)
    X[9] -= K_bias * (gnss_m.V_n - X[3]); 
    X[10] -= K_bias * (gnss_m.V_e - X[4]);
    X[11] -= K_bias * (gnss_m.V_d - X[5]);

    // Mise à jour de la Covariance P
    for (let i = 0; i < NUM_STATES; i++) {
        const K_effective = (i < 9) ? K_pos : K_bias; 
        P[i][i] = (1 - K_effective) * P[i][i]; 
        P[i][i] = Math.max(P[i][i], i < 9 ? 0.01 : 1e-8); 
    }
    
    // Correction de l'Attitude (Cap/Yaw) basé sur le Cap GPS
    if (gnss_m.heading !== null) {
        const K_yaw = P[8][8] / (P[8][8] + 0.01); 
        let heading_diff = (gnss_m.heading * D2R) - X[8];
        while (heading_diff > Math.PI) heading_diff -= 2 * Math.PI;
        while (heading_diff < -Math.PI) heading_diff += 2 * Math.PI;
        X[8] += K_yaw * heading_diff;
        P[8][8] = (1 - K_yaw) * P[8][8];
    }
}

/** Calcule la vitesse 3D estimée par l'EKF. */
function getEKFVelocity3D() {
    const X = EKFState.X;
    return Math.sqrt(X[3] * X[3] + X[4] * X[4] + X[5] * X[5]);
}

/** Calcule l'incertitude de position estimée. */
function getEKFAccuracy() {
    const P = EKFState.P;
    return Math.sqrt(P[0][0] + P[1][1] + P[2][2]);
}

/** Calcule le Bruit de Mesure (R) dynamique (basé sur l'acc. GPS brute). */
function getMeasurementNoiseR(accRaw, P_hPa) {
    let R = accRaw ** 2; 
    R = Math.min(R, MAX_GPS_ACCURACY ** 2 * 2); 
    if (P_hPa !== null) {
        const pressureFactor = 1.0 + Math.abs(1013.25 - P_hPa) / 1013.25 * 0.1;
        R *= Math.max(1.0, pressureFactor); 
    }
    return Math.max(R_POS_MIN, R); 
}
// =================================================================
// PHYSICS SERVICES (2/4) : Forces, Météo, Astro & Relativité
// =================================================================

// Dépendances de l'EKF Core
// (Accès aux variables globales EKFState, D2R, R_AIR, C_S_REF, OMEGA_EARTH, C_L)

// --- DONNÉES CÉLESTES/GRAVITÉ ---
const CELESTIAL_DATA = {
    'EARTH': { G: 9.80665, R: 6371000, name: 'Terre' },
    'MOON': { G: 1.62, R: 1737400, name: 'Lune' },
    'MARS': { G: 3.71, R: 3389500, name: 'Mars' },
};

/** Calcule l'accélération gravitationnelle locale. */
function getGravityLocal(alt, bodyKey, R_ref) {
    const data = CELESTIAL_DATA[bodyKey] || CELESTIAL_DATA['EARTH'];
    const g_base = data.G;
    const R_base = R_ref || data.R;
    if (alt === null) alt = 0;
    return g_base * (R_base / (R_base + alt)) ** 2;
}

/** Calcule la Vitesse du Son Local (basée sur la température). */
function calculateSpeedOfSound(tempK) {
    if (tempK === null || isNaN(tempK)) return C_S_REF; 
    return Math.sqrt(1.4 * R_AIR * tempK); // Gamma = 1.4 pour l'air sec
}

/** Calcule la force de Coriolis. */
function calculateCoriolisForce(lat, V_ekf, X_ekf, currentMass) {
    const V_vertical = -X_ekf[5]; 
    const latRad = lat * D2R;
    const V_horizontal = Math.sqrt(X_ekf[3]**2 + X_ekf[4]**2);
    
    const F_h = 2 * currentMass * OMEGA_EARTH * V_horizontal * Math.sin(latRad);
    const F_v = 2 * currentMass * OMEGA_EARTH * V_vertical * Math.cos(latRad);
    
    return Math.sqrt(F_h**2 + F_v**2); 
}

/** Calcule la Pression Dynamique. */
function calculateDynamicPressure(rho, V_ekf) {
    if (rho === null) rho = 1.225; 
    return 0.5 * rho * V_ekf ** 2; // q = 1/2 * rho * V^2
}

/** Calcule les facteurs relativistes (Lorentz). */
function calculateRelativisticFactors(V_ekf) {
    const V_C_ratio = V_ekf / C_L;
    if (V_C_ratio === 0) return { lorentzFactor: 1.0, percC: 0.0 };
    const lorentzFactor = 1.0 / Math.sqrt(1 - V_C_ratio ** 2);
    return { lorentzFactor, percC: V_C_ratio * 100 };
}

/** Calcule les positions du Soleil et de la Lune (utilisé dans main-initializer pour le DOM). */
function updateAstro(latA, lonA, date, $) {
    if (typeof SunCalc === 'undefined' || latA === null || lonA === null) return;
    
    const times = SunCalc.getTimes(date, latA, lonA);
    const sunPos = SunCalc.getPosition(date, latA, lonA);
    const moonPos = SunCalc.getMoonPosition(date, latA, lonA);
    const moonIllumination = SunCalc.getMoonIllumination(date);
    
    const sunAltDeg = sunPos.altitude * R2D;
    const sunAzDeg = (sunPos.azimuth * R2D + 180) % 360;
    const moonAltDeg = moonPos.altitude * R2D;
    const moonAzDeg = (moonPos.azimuth * R2D + 180) % 360;

    if ($('sun-alt')) $('sun-alt').textContent = `${sunAltDeg.toFixed(2)} °`;
    if ($('sun-azimuth')) $('sun-azimuth').textContent = `${sunAzDeg.toFixed(2)} °`;
    if ($('moon-alt')) $('moon-alt').textContent = `${moonAltDeg.toFixed(2)} °`;
    if ($('moon-azimuth')) $('moon-azimuth').textContent = `${moonAzDeg.toFixed(2)} °`;
    if ($('moon-illuminated')) $('moon-illuminated').textContent = `${(moonIllumination.fraction * 100).toFixed(1)} %`;
    
    const solarTimeData = getSolarTime(date, lonA);
    if ($('tst')) $('tst').textContent = solarTimeData.TST;
    if ($('mst')) $('mst').textContent = solarTimeData.MST;
    if ($('noon-solar')) $('noon-solar').textContent = solarTimeData.NoonSolar;
    if ($('eot')) $('eot').textContent = `${solarTimeData.EOT} min`;

    const bodyEl = document.body;
    if (sunAltDeg > 5) {
        bodyEl.className = bodyEl.className.replace('sky-night', 'sky-day');
    } else if (sunAltDeg < -0.5) {
        bodyEl.className = bodyEl.className.replace('sky-day', 'sky-night');
    }
}

/** Calcule le Temps Solaire Vrai (TST) et l'Équation du Temps (EOT). */
function getSolarTime(date, lon) {
    const dayMs = 1000 * 60 * 60 * 24;
    const J1970 = 2440588, J2000 = 2451545; 
    const toDays = (d) => (d.valueOf() / dayMs - 0.5 + J1970) - J2000;
    const solarMeanAnomaly = (d) => D2R * (356.0470 + 0.9856002585 * d); 
    const eclipticLongitude = (M) => {
        const C = D2R * (1.9148 * Math.sin(M) + 0.0200 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M)), P = D2R * 102.9377;                                                                
        return M + C + P + Math.PI;
    };
    
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

    return { TST: toTimeString(tst_ms), MST: toTimeString(mst_ms), EOT: eot_min.toFixed(2), NoonSolar: toTimeString(J_transit * dayMs) };
        }
// =================================================================
// APP CONTROLS (3/4) : Gestion des Capteurs, Carte et Boucle Principale
// =================================================================

// Dépendances : EKFState, initEKF, predictEKF_IMU, updateEKF_GNSS, getEKFVelocity3D, getMeasurementNoiseR
// (Accès aux variables globales : wID, lPos, lastUpdateTime, real_accel_x/y/z, G_ACC, lastP_hPa, map, marker, circle, sTime, distM, maxSpd, timeMoving, $, GPS_OPTS, emergencyStopActive)

/** Gestionnaire d'événements pour le mouvement IMU. */
function imuMotionHandler(event) {
    if (event.acceleration) {
        real_accel_x = event.acceleration.x || 0;
        real_accel_y = event.acceleration.y || 0;
        real_accel_z = event.acceleration.z || 0;
        if ($('imu-status')) $('imu-status').textContent = "Actif (Sans Gravité)";
    } else if (event.accelerationIncludingGravity) {
        real_accel_x = event.accelerationIncludingGravity.x || 0;
        real_accel_y = event.accelerationIncludingGravity.y || 0;
        real_accel_z = event.accelerationIncludingGravity.z || 0;
        if ($('imu-status')) $('imu-status').textContent = "Actif (Avec Gravité)";
    }
}

function startIMUListeners() {
    if (window.DeviceMotionEvent) {
        if (typeof DeviceMotionEvent.requestPermission === 'function') {
            DeviceMotionEvent.requestPermission().then(p => {
                if (p === 'granted') window.addEventListener('devicemotion', imuMotionHandler);
            });
        } else {
            window.addEventListener('devicemotion', imuMotionHandler);
        }
    }
}

function stopIMUListeners() {
    if (window.DeviceMotionEvent) {
        window.removeEventListener('devicemotion', imuMotionHandler);
    }
    if ($('imu-status')) $('imu-status').textContent = "Inactif";
}

function handleErr(err) {
    if (wID !== null) {
        $('gps-status-dr').textContent = `ERREUR ${err.code}: ${err.message}`;
    }
}

function toggleGPS() {
    if (wID === null) {
        $('toggle-gps-btn').innerHTML = '⏸️ PAUSE GNSS/IMU';
        $('toggle-gps-btn').style.backgroundColor = '#ffc107';
        wID = navigator.geolocation.watchPosition(handlePosition, handleErr, GPS_OPTS.HIGH_FREQ);
        startIMUListeners();
        $('gps-status-dr').textContent = 'Actif (Fusion EKF 15 États en cours)';
    } else {
        navigator.geolocation.clearWatch(wID);
        wID = null;
        stopIMUListeners();
        $('toggle-gps-btn').innerHTML = '▶️ MARCHE GNSS/IMU';
        $('toggle-gps-btn').style.backgroundColor = '#28a745';
        $('gps-status-dr').textContent = 'Pause';
    }
}

/** Mise à jour de la carte (Leaflet). */
function updateMap(lat, lon, acc) {
    if (typeof L === 'undefined' || !$('map')) return;

    if (!map) {
        map = L.map('map').setView([lat, lon], 16);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { attribution: '© OpenStreetMap contributors' }).addTo(map);
        marker = L.circleMarker([lat, lon], { radius: 6, color: 'white', fillColor: '#007bff', fillOpacity: 1 }).addTo(map);
        circle = L.circle([lat, lon], { radius: acc, color: '#007bff', fillColor: '#007bff', fillOpacity: 0.2, weight: 1 }).addTo(map);
    }

    const newLatLng = L.latLng(EKFState.lat, EKFState.lon);
    marker.setLatLng(newLatLng);
    circle.setLatLng(newLatLng);
    circle.setRadius(acc);
    map.setView(newLatLng, Math.max(map.getZoom(), 16));
}

/** BOUCLE PRINCIPALE : L'endroit où la magie EKF opère. */
function handlePosition(pos) {
    if (emergencyStopActive) return;

    const cTimePos = pos.timestamp;
    const dt = lastUpdateTime === 0 ? MIN_DT : Math.max(MIN_DT, (cTimePos - lastUpdateTime) / 1000); 
    lastUpdateTime = cTimePos;
    
    if (lPos === null) {
        EKFState.lat = pos.coords.latitude; EKFState.lon = pos.coords.longitude;
        initEKF(EKFState.lat, EKFState.lon, pos.coords.altitude || DEFAULT_ALT);
        sTime = Date.now();
        lPos = pos;
        updateMap(EKFState.lat, EKFState.lon, pos.coords.accuracy);
        return;
    }

    // EKF PRÉDICTION (IMU)
    const a_b = [real_accel_x, real_accel_y, real_accel_z]; 
    predictEKF_IMU(dt, a_b, G_ACC);

    // EKF CORRECTION (GNSS)
    const accRaw = pos.coords.accuracy;
    const rawSpeed = pos.coords.speed || 0.0;
    
    let V_n = 0.0, V_e = 0.0, V_d = 0.0;
    if (pos.coords.heading !== null && pos.coords.heading !== undefined) {
        const headingRad = pos.coords.heading * D2R;
        V_n = rawSpeed * Math.cos(headingRad);
        V_e = rawSpeed * Math.sin(headingRad);
        V_d = -(pos.coords.altitude - lPos.coords.altitude) / dt || 0.0; 
    }

    const R_cov = getMeasurementNoiseR(accRaw, lastP_hPa);

    const gnss_m = {
        lat: pos.coords.latitude, lon: pos.coords.longitude, alt: pos.coords.altitude || EKFState.alt, 
        acc: accRaw, V_n, V_e, V_d, heading: pos.coords.heading
    };

    updateEKF_GNSS(gnss_m, R_cov);
    
    // MISE À JOUR DES STATISTIQUES GLOBAL
    const V_ekf = getEKFVelocity3D();
    const sSpdFE = V_ekf < MIN_DT ? 0 : V_ekf; 

    if (sSpdFE > MIN_DT) {
        distM += sSpdFE * dt; 
        timeMoving += dt;
        maxSpd = Math.max(maxSpd, sSpdFE * KMH_MS);
    }
    
    updateDisplay(V_ekf, dt, accRaw, pos);
    
    lPos = pos;
            }
// =================================================================
// MAIN INITIALIZER (4/4) : Variables Globales, Sync et Affichage
// =================================================================

// Dépendances: EKFState, getCDate, updateAstro, fetchWeather, getEKFVelocity3D, getEKFAccuracy
// calculateSpeedOfSound, calculateRelativisticFactors, calculateDynamicPressure, calculateCoriolisForce, getGravityLocal
// (Accès à toutes les fonctions et constantes des autres fichiers)

const DOM_SLOW_UPDATE_MS = 1000;
const DEFAULT_LAT = 48.8566; 
const DEFAULT_LON = 2.3522;
const DEFAULT_ALT = 50.0;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 5000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 60000, timeout: 60000 }
};

// --- VARIABLES D'ÉTAT GLOBAL ---
let wID = null, domID = null, lPos = null, sTime = null;
let lastUpdateTime = 0; 
let distM = 0, maxSpd = 0, timeMoving = 0;
let lat = DEFAULT_LAT, lon = DEFAULT_LON;
let lServH = null, lLocH = null; 
let G_ACC = G_EARTH;
let R_ALT_CENTER_REF = R_E_BASE;
let currentCelestialBody = 'EARTH';
let currentMass = 70.0; 
let emergencyStopActive = false;

// Données externes et IMU
let lastP_hPa = null, lastT_K = null; 
let lastAirDensity = 1.225;
let real_accel_x = 0, real_accel_y = 0, real_accel_z = 0;
let map, marker, circle;

// Fonction utilitaire DOM
const $ = id => document.getElementById(id);


/** Récupère et compense l'heure NTP (RTT). */
async function syncH(lServH_in, lLocH_in) {
    const t0 = performance.now();
    try {
        const response = await fetch(SERVER_TIME_ENDPOINT);
        const data = await response.json();
        const t3 = performance.now();
        const serverTime = new Date(data.utc_datetime).getTime();
        const rtt = t3 - t0;
        const offset = rtt / 2;
        return { lServH: serverTime + offset, lLocH: t3 }; 
    } catch (err) {
        console.warn(`[SYNC] Échec de la synchronisation RTT. Heure locale utilisée.`);
        const now = Date.now();
        return { lServH: now, lLocH: performance.now() }; 
    }
}

/** Retourne l'heure synchronisée. */
function getCDate(lServH, lLocH) { 
    if (lServH === null || lLocH === null) { return new Date(); }
    const offsetSinceSync = performance.now() - lLocH;
    return new Date(lServH + offsetSinceSync); 
}

/** Récupère les données météo et calcule la densité de l'air. */
async function fetchWeather(latA, lonA) {
    const API_KEY = "YOUR_API_KEY_HERE"; 
    
    if (API_KEY === "YOUR_API_KEY_HERE") {
        return { tempC: 15.0, pressure_hPa: 1013.25, humidity_perc: 60, tempK: 288.15, air_density: 1.225, dew_point: 7.0, status: "Données par défaut (API non configurée)" };
    }
    // ... Logique d'appel API réelle et calcul de la densité de l'air ...
    return null; // Retourne null si API non configurée et non par défaut
}


/** Mise à jour de l'affichage DOM. */
function updateDisplay(V_ekf, dt, accRaw, pos) {
    const sSpdKMH = V_ekf * KMH_MS;
    const X = EKFState.X;
    
    // CALCULS DYNAMIQUES ET RELATIVISTES
    const V_sound = calculateSpeedOfSound(lastT_K);
    const machNumber = V_ekf / V_sound;
    const { lorentzFactor, percC } = calculateRelativisticFactors(V_ekf);
    const accel_long = (V_ekf - (lPos?.coords.speed || 0)) / dt;
    const dynamicPressure = calculateDynamicPressure(lastAirDensity, V_ekf);
    const coriolisForce = calculateCoriolisForce(EKFState.lat, V_ekf, X, currentMass);
    const forceLong = accel_long * currentMass;

    // --- VITESSE & DISTANCE ---
    $('speed-stable').textContent = `${sSpdKMH.toFixed(3)} km/h`;
    $('speed-stable-ms').textContent = `${V_ekf.toFixed(3)} m/s`;
    $('speed-raw-ms').textContent = `${(pos.coords.speed || 0).toFixed(3)} m/s`;
    $('speed-max').textContent = `${maxSpd.toFixed(3)} km/h`;
    $('distance-total-km').textContent = `${(distM / 1000).toFixed(3)} km`;
    $('time-moving').textContent = `${timeMoving.toFixed(2)} s`;
    
    // --- POSITION EKF & BRUTE ---
    $('lat-display').textContent = `${EKFState.lat.toFixed(6)} °`;
    $('lon-display').textContent = `${EKFState.lon.toFixed(6)} °`;
    $('alt-display').textContent = `${EKFState.alt.toFixed(2)} m`;
    $('vertical-speed').textContent = `${(-X[5]).toFixed(2)} m/s (EKF)`; 
    $('gps-precision').textContent = `${accRaw.toFixed(2)} m (Brut)`;
    
    // --- EKF/IMU & ATTITUDE ---
    $('kalman-uncert').textContent = `${getEKFAccuracy().toFixed(3)} m (EKF Pos)`;
    $('imu-accel-x').textContent = `${real_accel_x.toFixed(2)} m/s² (X)`;
    $('imu-accel-y').textContent = `${real_accel_y.toFixed(2)} m/s² (Y)`;
    $('imu-accel-z').textContent = `${real_accel_z.toFixed(2)} m/s² (Z)`;
    $('roll').textContent = `${(X[6] * R2D).toFixed(2)} °`;
    $('pitch').textContent = `${(X[7] * R2D).toFixed(2)} °`;
    $('yaw').textContent = `${(X[8] * R2D).toFixed(2)} °`;
    
    // --- BIAIS EKF 15 ÉTATS ---
    $('bias-accel-x').textContent = `${X[9].toFixed(6)} m/s²`;
    $('bias-accel-y').textContent = `${X[10].toFixed(6)} m/s²`;
    $('bias-accel-z').textContent = `${X[11].toFixed(6)} m/s²`;
    $('bias-gyro-x').textContent = `${X[12].toFixed(6)} rad/s`;
    $('bias-gyro-y').textContent = `${X[13].toFixed(6)} rad/s`;
    $('bias-gyro-z').textContent = `${X[14].toFixed(6)} rad/s`;

    // --- DYNAMIQUE ET RELATIVITÉ ---
    $('speed-of-sound-calc').textContent = `${V_sound.toFixed(2)} m/s`;
    $('mach-number').textContent = machNumber.toFixed(4);
    $('perc-speed-c').textContent = `${percC.toExponential(2)} %`;
    $('lorentz-factor').textContent = lorentzFactor.toFixed(4);
    $('dynamic-pressure').textContent = `${dynamicPressure.toFixed(2)} Pa`;
    $('coriolis-force').textContent = `${coriolisForce.toFixed(3)} N`;
    $('accel-long').textContent = `${accel_long.toFixed(3)} m/s²`;
    $('force-long').textContent = `${forceLong.toFixed(2)} N`;

    // --- PHYSIQUE & GRAVITÉ ---
    const local_g = getGravityLocal(EKFState.alt, currentCelestialBody, R_ALT_CENTER_REF);
    $('gravity-local').textContent = `${local_g.toFixed(5)} m/s²`;
    
    updateMap(EKFState.lat, EKFState.lon, accRaw);
}


document.addEventListener('DOMContentLoaded', () => {
    initEKF(DEFAULT_LAT, DEFAULT_LON, DEFAULT_ALT);
    updateMap(DEFAULT_LAT, DEFAULT_LON, 10.0);
    
    $('toggle-gps-btn').addEventListener('click', toggleGPS);
    $('toggle-mode-btn').addEventListener('click', () => document.body.classList.toggle('dark-mode'));
    
    if ($('celestial-body-select')) {
        $('celestial-body-select').addEventListener('change', (e) => {
            currentCelestialBody = e.target.value;
            const data = CELESTIAL_DATA[currentCelestialBody];
            G_ACC = data.G;
            R_ALT_CENTER_REF = data.R;
            if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC.toFixed(4)} m/s²`;
        });
        const initialData = CELESTIAL_DATA[currentCelestialBody];
        if ($('gravity-base')) $('gravity-base').textContent = `${initialData.G.toFixed(4)} m/s²`;
    }
    
    if ($('mass-input')) {
        $('mass-input').addEventListener('input', (e) => {
            currentMass = parseFloat(e.target.value) || 70.0;
            $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        });
        $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
    }

    // Démarrage de la synchro NTP
    syncH(lServH, lLocH).then(newTimes => {
        lServH = newTimes.lServH;
        lLocH = newTimes.lLocH;
        $('gps-status-dr').textContent = 'Prêt. Sync NTP réussie.';
    });

    // Boucle de mise à jour lente (Astro/Météo/Horloge)
    domID = setInterval(() => {
        const now = getCDate(lServH, lLocH);
        
        // HORLOGE
        if (now) {
            $('local-time').textContent = now.toLocaleTimeString('fr-FR');
            $('date-display').textContent = now.toLocaleDateString('fr-FR') + ' UTC';
            if (sTime) {
                $('elapsed-time').textContent = `${((Date.now() - sTime) / 1000).toFixed(2)} s`;
            }
        }
        
        // Resynchronise NTP toutes les 10 minutes
        if (Math.floor(Date.now() / 1000) % 600 === 0) {
             syncH(lServH, lLocH).then(newTimes => {
                lServH = newTimes.lServH;
                lLocH = newTimes.lLocH;
             });
        }
        
        // ASTRO
        updateAstro(EKFState.lat, EKFState.lon, now, $);
        
        // MÉTÉO (Appelé uniquement si GNSS/IMU est actif)
        if (wID !== null) {
            fetchWeather(EKFState.lat, EKFState.lon).then(data => {
                if (data) {
                    lastP_hPa = data.pressure_hPa;
                    lastT_K = data.tempK;
                    lastAirDensity = data.air_density;
                    
                    if ($('temp-air-2')) $('temp-air-2').textContent = `${data.tempC.toFixed(1)} °C`;
                    if ($('pressure-2')) $('pressure-2').textContent = `${data.pressure_hPa.toFixed(0)} hPa`;
                    if ($('humidity-2')) $('humidity-2').textContent = `${data.humidity_perc} %`;
                    if ($('air-density')) $('air-density').textContent = `${data.air_density.toFixed(3)} kg/m³`;
                    if ($('dew-point')) $('dew-point').textContent = `${data.dew_point.toFixed(1)} °C`;
                    if ($('weather-status')) $('weather-status').textContent = data.status;
                }
            });
        }

    }, DOM_SLOW_UPDATE_MS);
});
