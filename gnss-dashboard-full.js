// =================================================================
// BLOC 1/4 : Constantes Globales et Configuration
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        return (decimals === 0 ? '0' : '0.00') + suffix;
    }
    return val.toFixed(decimals) + suffix;
};

// --- CONSTANTES MATHÉMATIQUES ET PHYSIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const R_AIR = 287.058;      // Constante spécifique de l'air sec (J/kg·K)
const GAMMA = 1.4;          // Rapport des chaleurs spécifiques pour l'air sec
const KMH_MS = 3.6;         // Conversion m/s vers km/h
const MASS_KG = 70.0;       // Masse par défaut pour les calculs de force
const MIN_DT = 0.01; 
const DOM_SLOW_UPDATE_MS = 1000;

// --- PARAMÈTRES DU FILTRE DE KALMAN (EKF INS/GNSS - 15 ÉTATS) ---
const STATE_SIZE = 15; 
const Q_PROCESS = 1e-6; 
const R_MIN = 0.01;         
const R_MAX = 500.0;        
const ALT_TH = -50;         // SEUIL BAS (Souterrain / Tunnel)
const ALT_HIGH_TH = 3000;   // SEUIL HAUT (Avion / Montagne)
const MAX_ACC = 200;        
const MIN_SPD = 0.05;       
const REFERENCE_DRAG_AREA = 0.5; // Surface de référence (m²)
const DRAG_COEFFICIENT = 1.2; // Coefficient de Traînée
let P_MATRIX = null; // Matrice de Covariance P initiale

// --- GÉOPHYSIQUE (WGS84 et Gravité) ---
let G_ACC = 9.80665;         // Gravité locale (par défaut)
let R_ALT_CENTER_REF = 6371000; // Rayon terrestre moyen (m)
const CELESTIAL_DATA = {
    'EARTH': { G: 9.80665, R: 6371000, name: 'Terre' },
};

// --- VALEURS DE L'ATMOSPHÈRE STANDARD (ISA) & HYPERLOOP ---
const P_ISA = 1013.25;      // Pression standard (hPa)
const T_ISA_C = 15.0;       // Température standard (°C)
const H_ISA_PERC = 50.0;    // Humidité par défaut (%)
const P_HYPERLOOP_PA = 10.0;     // Pression Hyperloop (Pascals)
const RHO_VACUUM = 0.00001;      // Densité forcée pour Hyperloop
const T_HYPERLOOP_C = 20.0;
// =================================================================
// BLOC 2/4 : Fonctions Physiques, Météo et Astro
// =================================================================

// --- FONCTIONS MATHÉMATIQUES ET PHYSIQUES ---

/** Calcule la distance de Haversine entre deux points (m) */
const dist = (lat1, lon1, lat2, lon2, R_ref) => {
    const R = R_ref || R_ALT_CENTER_REF; 
    const dLat = (lat2 - lat1) * D2R, dLon = (lon2 - lon1) * D2R;
    lat1 *= D2R; lat2 *= D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R * c;
};

/** Gravité locale (simplifiée) */
function getGravityLocal(alt, bodyKey = 'EARTH') {
    if (alt === null) alt = 0;
    const g_base = CELESTIAL_DATA[bodyKey]?.G || CELESTIAL_DATA['EARTH'].G;
    const R_base = CELESTIAL_DATA[bodyKey]?.R || CELESTIAL_DATA['EARTH'].R;
    return g_base * (R_base / (R_base + alt)) ** 2;
}

/** Bruit de mesure R dynamique pour le Kalman */
function getKalmanR(acc, alt, P_hPa) {
    let acc_effective = acc;
    if (acc > MAX_ACC) return 1e9;
    
    let R = acc_effective * acc_effective; 
    if (alt !== null && alt < ALT_TH) R *= 4.0; 

    return Math.max(R_MIN, Math.min(R_MAX, R)); 
}


// --- FONCTIONS MÉTÉO & ENVIRONNEMENT ---
function calculateAirDensity(P_hPa, T_K, hyperloopMode) {
    if (hyperloopMode) return RHO_VACUUM;
    if (P_hPa === null || T_K === null) return 1.225; 
    const P_Pa = P_hPa * 100;
    return P_Pa / (R_AIR * T_K);
}
function calculateDewPoint(T_C, H_perc) {
    if (T_C === null || H_perc === null) return 0.0;
    const a = 17.27, b = 237.7;
    const alpha = (a * T_C) / (b + T_C) + Math.log(H_perc / 100);
    return (b * alpha) / (a - alpha);
}
function calculateSpeedOfSound(T_K) {
    if (T_K === null) return 343.0;
    return Math.sqrt(GAMMA * R_AIR * T_K);
}

// Fonction synchrone pour l'heure (simule NTP)
let lServH = Date.now(), lLocH = Date.now(); 
function getCDate(lServH, lLocH) {
    if (lServH === null || lLocH === null) return null; 
    const timeOffset = Date.now() - lLocH;
    return new Date(lServH + timeOffset);
}

// Fonction de récupération Météo (Simulée pour la démo)
async function fetchWeather(lat, lon, currentAlt, hyperloopMode) {
    const isIsolatedByAltitude = currentAlt !== null && (currentAlt > ALT_HIGH_TH || currentAlt < ALT_TH);

    if (hyperloopMode) { 
        // Mode Hyperloop
        return { 
            tempC: T_HYPERLOOP_C, tempK: T_HYPERLOOP_C + 273.15, 
            pressure_hPa: P_HYPERLOOP_PA / 100, humidity_perc: 0.0 
        };
    } else if (isIsolatedByAltitude) {
        // Mode Isolation (ISA par défaut)
        return { 
            tempC: T_ISA_C, tempK: T_ISA_C + 273.15, 
            pressure_hPa: P_ISA, humidity_perc: H_ISA_PERC
        };
    } else {
        // Simulation d'un appel API/Cache standard
        return { 
            tempC: T_ISA_C, tempK: T_ISA_C + 273.15, 
            pressure_hPa: P_ISA, humidity_perc: H_ISA_PERC
        };
    }
}

// --- FONCTIONS ASTRO (SunCalc) ---
function getMoonPhaseIcon(phase) {
    if (phase < 0.03 || phase > 0.97) return "🌑"; 
    if (phase < 0.27) return "🌓"; 
    if (phase < 0.52) return "🌕"; 
    if (phase < 0.77) return "🌗"; 
    return "🌘"; 
}
function getMoonPhaseName(phase) {
    if (phase < 0.03 || phase > 0.97) return "Nouvelle Lune";
    if (phase < 0.52) return "Pleine Lune";
    return "Autres Phases"; 
}
function updateAstro(latA, lonA, lServH, lLocH) {
    const now = getCDate(lServH, lLocH); 
    if (typeof SunCalc === 'undefined' || !latA || !lonA || !now) return; 
    
    const sunPos = SunCalc.getPosition(now, latA, lonA); 
    const moonIllum = SunCalc.getMoonIllumination(now); 
    const moonPos = SunCalc.getMoonPosition(now, latA, lonA); 
    const times = SunCalc.getTimes(now, latA, lonA);

    // Mise à jour DOM
    if ($('date-astro')) $('date-astro').textContent = now.toLocaleDateString('fr-FR');
    if ($('sun-altitude')) $('sun-altitude').textContent = `${(sunPos.altitude * R2D).toFixed(2)} °`; 
    if ($('sun-azimuth')) $('sun-azimuth').textContent = `${(sunPos.azimuth * R2D).toFixed(2)} °`; 
    if ($('day-duration')) $('day-duration').textContent = `${((times.sunset - times.sunrise) / 3600000).toFixed(2)} h`;
    
    const phaseName = getMoonPhaseName(moonIllum.phase);
    const phaseIcon = getMoonPhaseIcon(moonIllum.phase);
    
    if ($('moon-phase-name')) $('moon-phase-name').textContent = `${phaseName} ${phaseIcon}`;
    if ($('moon-illuminated')) $('moon-illuminated').textContent = `${(moonIllum.fraction * 100).toFixed(1)} %`;
    if ($('moon-alt')) $('moon-alt').textContent = `${(moonPos.altitude * R2D).toFixed(2)} °`;
    if ($('moon-azimuth')) $('moon-azimuth').textContent = `${(moonPos.azimuth * R2D).toFixed(2)} °`;
    if ($('moon-times')) $('moon-times').textContent = `N/A`; // Nécessite MoonCalc qui n'est pas inclus
              }
// =================================================================
// BLOC 3/4 : Logique du Filtre EKF (Matricielle)
// =================================================================

function gnssInsEKF_step(dt, imu, gps, stateVector, R_dyn, G_ACC) {
    if (typeof math === 'undefined' || dt === 0 || stateVector.length !== STATE_SIZE) {
        return { stateVector: stateVector, covariance_diag: P_MATRIX ? math.diag(P_MATRIX).toArray() : Array(STATE_SIZE).fill(1000) };
    }
    
    if (P_MATRIX === null) {
        P_MATRIX = math.identity(STATE_SIZE);
        P_MATRIX = P_MATRIX.map((val, index) => val * (index < 9 ? 100 : 1000));
    }

    let X_k = math.matrix(stateVector).resize([STATE_SIZE, 1]); 

    // 1. PRÉDICTION (INS)
    const P = X_k.subset(math.index([0, 1, 2], 0));
    const V = X_k.subset(math.index([3, 4, 5], 0));
    const B_a = X_k.subset(math.index([9, 10, 11], 0)); // Biais Accel. (simplifié, les autres états sont omis pour la démo)
    
    const Acc_corr_array = [imu.real_accel_x, imu.real_accel_y, imu.real_accel_z];
    const Acc_corr = math.subtract(math.matrix(Acc_corr_array).resize([3, 1]), B_a); 
    const G_vector = math.matrix([0, 0, -G_ACC]).resize([3, 1]); 
    
    const V_k_plus_1 = math.add(V, math.multiply(math.add(Acc_corr, G_vector), dt));
    const P_k_plus_1 = math.add(P, math.multiply(V, dt));

    let X_k_plus_1_nominal = math.clone(X_k);
    X_k_plus_1_nominal.subset(math.index([0, 1, 2], 0), P_k_plus_1);
    X_k_plus_1_nominal.subset(math.index([3, 4, 5], 0), V_k_plus_1);

    const Q_MATRIX = math.identity(STATE_SIZE).map(val => val * Q_PROCESS * dt); 
    P_MATRIX = math.add(P_MATRIX, Q_MATRIX); 

    // 2. CORRECTION (GNSS)
    let X_k_final = X_k_plus_1_nominal;

    if (gps.lat !== null && R_dyn < R_MAX) { 
        const Y_GPS = math.matrix([gps.lat, gps.lon, gps.alt]).resize([3, 1]);
        const H_of_X = X_k_plus_1_nominal.subset(math.index([0, 1, 2], 0));
        const Z_RESIDUAL = math.subtract(Y_GPS, H_of_X);
        
        // Matrice d'Observation H (pour Px, Py, Pz)
        const H_MATRIX = math.matrix([
            [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        ]);
        
        const R_MATRIX = math.identity(3).map(val => val * R_dyn);

        // Gain de Kalman K
        const H_P = math.multiply(H_MATRIX, P_MATRIX);
        const S = math.add(math.multiply(H_P, math.transpose(H_MATRIX)), R_MATRIX); 
        const K_num = math.multiply(P_MATRIX, math.transpose(H_MATRIX));
        const K_GAIN = math.multiply(K_num, math.inv(S)); 

        // Mise à jour de l'état final et de la covariance
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
// BLOC 4/4 : Logique Principale et Boucle
// =================================================================

// --- VARIABLES D'ÉTAT (Globales) ---
let wID = null, domID = null, lPos = null;
let distM = 0, maxSpd = 0, timeMoving = 0; 

let EKF_STATE_VECTOR = Array(STATE_SIZE).fill(0.0);
let EKF_COVARIANCE_DIAG = Array(STATE_SIZE).fill(1000.0); 

let kSpd = 0; 
let kUncert = 1000; 
let kAlt = null; 
let lat = 0, lon = 0;

let emergencyStopActive = false;
let hyperloopMode = false;

let lastP_hPa = P_ISA;      
let lastT_K = T_ISA_C + 273.15;         
let lastAirDensity = calculateAirDensity(lastP_hPa, lastT_K, hyperloopMode); 
let lastSpeedOfSound = calculateSpeedOfSound(lastT_K);

let real_accel_x = 0, real_accel_y = 0, real_accel_z = 0;

// --- BOILERPLATE CARTES ---
let map = null, marker = null, trace = [];
let traceLine = null;

function initMap(initialLat, initialLon) {
    if (typeof L === 'undefined') return;
    map = L.map('map').setView([initialLat, initialLon], 15);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
    }).addTo(map);
    marker = L.marker([initialLat, initialLon]).addTo(map);
    traceLine = L.polyline(trace, {color: 'red'}).addTo(map);
}
function updateMap(cLat, cLon) {
    if (!map || !marker) return;
    const latlng = [cLat, cLon];
    marker.setLatLng(latlng);
    map.setView(latlng, map.getZoom() > 15 ? map.getZoom() : 15); 
    trace.push(latlng);
    if (trace.length > 500) trace.shift(); 
    traceLine.setLatLngs(trace);
}

// --- GESTION CAPTEURS ---
let watchId = null; 
const gpsOptions = { enableHighAccuracy: true, timeout: 5000, maximumAge: 0 };
function gpsUpdateCallback(pos) { updateDisp(pos); }
function startGPS() {
    if (navigator.geolocation) {
        watchId = navigator.geolocation.watchPosition(gpsUpdateCallback, () => {
            if ($('speed-status-text')) $('speed-status-text').textContent = 'GPS ÉCHOUÉ';
        }, gpsOptions);
    } 
}
function handleMotionEvent(event) {
    // Utilise l'accélération en m/s²
    real_accel_x = event.accelerationIncludingGravity.x; 
    real_accel_y = event.accelerationIncludingGravity.y; 
    real_accel_z = event.accelerationIncludingGravity.z; 
}
function startIMUListeners() {
    if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', handleMotionEvent);
        if ($('speed-status-text')) $('speed-status-text').textContent = 'IMU ACTIF';
    }
}
function resetMaxValues() { distM = 0; maxSpd = 0; timeMoving = 0; }

// ===========================================
// FONCTION PRINCIPALE DE MISE À JOUR GPS (updateDisp)
// ===========================================

function updateDisp(pos) {
    if (emergencyStopActive || typeof math === 'undefined') return;

    // --- 1. ACQUISITION & DT ---
    const cTimePos = pos.timestamp;
    const cLat = pos.coords.latitude;
    const cLon = pos.coords.longitude;
    const altRaw = pos.coords.altitude;
    let accRaw = pos.coords.accuracy;
    
    if (altRaw !== null) { $('alt-raw').textContent = dataOrDefault(altRaw, 2, ' m'); }
    
    let dt = 0;
    if (lPos) { 
        dt = (cTimePos - lPos.timestamp) / 1000; 
    } 
    else { 
        // Initialisation de l'état EKF et de lPos
        EKF_STATE_VECTOR[0] = cLat; EKF_STATE_VECTOR[1] = cLon; EKF_STATE_VECTOR[2] = altRaw;
        lPos = { timestamp: cTimePos, speedMS_3D: 0, kAlt_old: altRaw, kLat_old: cLat, kLon_old: cLon }; 
        initMap(cLat, cLon); 
        updateMap(cLat, cLon); return; 
    }
    if (dt < MIN_DT || dt > 10) { lPos.timestamp = cTimePos; return; }

    // --- 2. EKF GNSS/INS 15 ÉTATS ---
    
    const gps_meas = { lat: cLat, lon: cLon, alt: altRaw, acc: accRaw };
    const imu_meas = { real_accel_x, real_accel_y, real_accel_z }; 
    let R_dyn = getKalmanR(accRaw, EKF_STATE_VECTOR[2], lastP_hPa); 

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
    const kAltUncert = Math.sqrt(newCovariance[2]);
    
    // CORRECTION CRITIQUE: Utilise les états EKF lissés pour le calcul de distance
    const dist2D = dist(lPos.kLat_old, lPos.kLon_old, lat, lon, R_ALT_CENTER_REF);
    distM += dist2D;
    
    if (V_ms > MIN_SPD) { timeMoving += dt; }
    if (V_ms > maxSpd) maxSpd = V_ms; 

    // Calculs de Dynamique et Forces
    const coriolis_force = 2 * MASS_KG * V_ms * OMEGA_EARTH * Math.sin(lat * D2R);
    const accel_long = (V_ms - lPos.speedMS_3D) / dt; 
    const dynamic_pressure_q = 0.5 * lastAirDensity * V_ms**2;
    const mach_number = V_ms / lastSpeedOfSound;
    const drag_force = dynamic_pressure_q * DRAG_COEFFICIENT * REFERENCE_DRAG_AREA;
    
    // --- 4. MISE À JOUR DU DOM ---
    if ($('speed-stable')) $('speed-stable').textContent = dataOrDefault(V_ms * KMH_MS, 2);
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = dataOrDefault(V_ms, 3, ' m/s');
    if ($('distance-total-km')) $('distance-total-km').textContent = `${dataOrDefault(distM / 1000, 3)} km | ${dataOrDefault(distM, 2)} m`;
    if ($('speed-max')) $('speed-max').textContent = dataOrDefault(maxSpd * KMH_MS, 2, ' km/h');
    if ($('time-moving')) $('time-moving').textContent = dataOrDefault(timeMoving, 2, ' s');
    if ($('mach-number')) $('mach-number').textContent = dataOrDefault(mach_number, 4);
    if ($('dynamic-pressure')) $('dynamic-pressure').textContent = dataOrDefault(dynamic_pressure_q, 2, ' Pa');
    if ($('drag-force')) $('drag-force').textContent = dataOrDefault(drag_force, 2, ' N');
    if ($('accel-long')) $('accel-long').textContent = dataOrDefault(accel_long, 3, ' m/s²');
    if ($('coriolis-force')) $('coriolis-force').textContent = dataOrDefault(coriolis_force, 2, ' N');
    if ($('kalman-uncert')) $('kalman-uncert').textContent = dataOrDefault(kUncert, 3, ' m²/s²');
    if ($('alt-uncertainty')) $('alt-uncertainty').textContent = dataOrDefault(kAltUncert, 3, ' m'); 
    if ($('lat-display')) $('lat-display').textContent = dataOrDefault(lat, 6, ' °');
    if ($('lon-display')) $('lon-display').textContent = dataOrDefault(lon, 6, ' °');
    if ($('alt-display')) $('alt-display').textContent = kAlt !== null ? dataOrDefault(kAlt, 2, ' m') : 'N/A';
    if ($('speed-sound')) $('speed-sound').textContent = dataOrDefault(lastSpeedOfSound, 2, ' m/s');

    // --- 5. SAUVEGARDE & MISE À JOUR CARTE ---
    // Enregistrement des états EKF lissés pour la prochaine itération
    lPos.kLat_old = lat; 
    lPos.kLon_old = lon; 
    lPos.kAlt_old = kAlt;
    lPos.speedMS_3D = kSpd; 
    lPos.timestamp = cTimePos; 

    updateMap(lat, lon);
}

// --- INITIALISATION DES ÉVÉNEMENTS ---
document.addEventListener('DOMContentLoaded', () => {
    $('hyperloop-toggle-btn').addEventListener('click', () => {
        hyperloopMode = !hyperloopMode;
        $('hyperloop-toggle-btn').textContent = hyperloopMode ? 'HYPERLOOP ACTIF' : 'HYPERLOOP INACTIF';
        $('hyperloop-toggle-btn').classList.toggle('active', hyperloopMode);
    });
    $('emergency-stop-btn').addEventListener('click', () => {
        emergencyStopActive = !emergencyStopActive;
        $('emergency-stop-btn').textContent = emergencyStopActive ? 'ARRÊT D\'URGENCE (ON)' : 'ARRÊT D\'URGENCE (OFF)';
    });
    $('reset-dist-btn').addEventListener('click', resetMaxValues);

    $('init-system-btn').addEventListener('click', () => {
        startGPS(); 
        startIMUListeners(); 
        
        $('init-system-btn').style.display = 'none';
        
        // Boucle Lente (Astro/Météo/Temps)
        domID = setInterval(async () => {
            const currentLat = lat || 43.284; 
            const currentLon = lon || 5.358;
            updateAstro(currentLat, currentLon, lServH, lLocH); 
            
            // Mise à jour Météo Périodique
            if (!emergencyStopActive) {
                const data = await fetchWeather(currentLat, currentLon, kAlt, hyperloopMode);
                if (data) {
                    lastP_hPa = data.pressure_hPa;
                    lastT_K = data.tempK;
                    lastAirDensity = calculateAirDensity(lastP_hPa, lastT_K, hyperloopMode);
                    lastSpeedOfSound = calculateSpeedOfSound(lastT_K);
                    
                    // Mise à jour DOM Météo
                    if ($('weather-status')) $('weather-status').textContent = `ACTIF`;
                    if ($('temp-air')) $('temp-air').textContent = `${data.tempC.toFixed(1)} °C`;
                    if ($('pressure-2')) $('pressure-2').textContent = `${data.pressure_hPa.toFixed(0)} hPa`;
                    if ($('humidity-2')) $('humidity-2').textContent = `${data.humidity_perc.toFixed(0)} %`;
                    if ($('air-density')) $('air-density').textContent = `${lastAirDensity.toFixed(3)} kg/m³`;
                    if ($('dew-point')) $('dew-point').textContent = `${calculateDewPoint(data.tempC, data.humidity_perc).toFixed(1)} °C`;
                }
            }
            
            // Mise à jour Horloge
            const now = getCDate(lServH, lLocH);
            if (now) $('local-time').textContent = now.toLocaleTimeString('fr-FR');
            if (now) $('date-display').textContent = now.toLocaleDateString('fr-FR');
            
        }, DOM_SLOW_UPDATE_MS); 
        
    }, { once: true });
});
