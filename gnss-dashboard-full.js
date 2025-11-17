// =================================================================
// BLOC 1/4 : Constantes Globales et Configuration
// =================================================================

// --- CONSTANTES MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI; 
const KMH_MS = 3.6;         
const C_L = 299792458;      

// --- CONSTANTES PHYSIQUES ET GÉOPHYSIQUES ---
let G_ACC = 9.80665;         // Doit être 'let' car modifié par WGS84 ou corps céleste
let R_ALT_CENTER_REF = 6371000; // Doit être 'let' car modifié par corps céleste
const OMEGA_EARTH = 7.2921159e-5; 

// --- CONSTANTES ATMOSPHÉRIQUES (ISA Standard) ---
const BARO_ALT_REF_HPA = 1013.25;
const RHO_SEA_LEVEL = 1.225;
const R_AIR = 287.058;
const GAMMA = 1.4;

// --- PARAMÈTRES DU FILTRE DE KALMAN NON PARFUMÉ (UKF) ---
const UKF_R_MAX = 500.0;     
const UKF_Q_SPD = 0.5;       
const KAPPA = 0.0;           
const MIN_SPD = 0.05;       
const R_ALT_MIN = 1.0;     

// --- CONFIGURATION SYSTÈME ---
const MIN_DT = 0.01;        
const MAP_UPDATE_INTERVAL = 3000;
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};

// --- FACTEURS D'ENVIRONNEMENT (Ajustent le bruit R du UKF) ---
const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0, DISPLAY: 'Surface Standard' },
    'URBAN': { R_MULT: 1.5, DISPLAY: 'Urbain Denser' },
    'MOUNTAIN': { R_MULT: 1.2, DISPLAY: 'Montagne' },
    'SUBTERRANEAN': { R_MULT: 3.0, DISPLAY: 'Souterrain/Tunnel' }
};
// =================================================================
// BLOC 2/4 : Filtres, Modèles Cinématiques et Quaternion
// Dépend de: app_constants.js (BLOC 1)
// =================================================================

// ===========================================
// CLASSE UKF (Unscented Kalman Filter - 1D Simplifié)
// ===========================================
class UKF {
    constructor(initialState, initialCovariance, processNoiseQ, measurementNoiseR, kappa) {
        this.x = initialState; 
        this.P = initialCovariance; 
        this.Q = processNoiseQ; 
        this.R_base = measurementNoiseR; 
        this.kappa = kappa;
        this.lambda = kappa; 
        this.W_m = [(this.lambda / (this.x.length + this.lambda)), 0.5 / (this.x.length + this.lambda), 0.5 / (this.x.length + this.lambda)];
        this.W_c = [this.W_m[0], this.W_m[1], this.W_m[2]]; 
    }

    generateSigmaPoints() {
        const sqrt_P = Math.sqrt(this.P[0]);
        const gamma = Math.sqrt(this.x.length + this.lambda);
        return [ this.x[0], this.x[0] + gamma * sqrt_P, this.x[0] - gamma * sqrt_P ];
    }

    // Modèle d'état (accel_input est la force/masse, i.e., m/s²)
    stateTransition(sigma, accel_input, dt) { return sigma + accel_input * dt; } 
    observationFunction(sigma) { return sigma; }
    
    predict(accel_input, dt) {
        const sigma_predicted = this.generateSigmaPoints().map(s => this.stateTransition(s, accel_input, dt));
        const x_bar = this.W_m[0] * sigma_predicted[0] + this.W_m[1] * sigma_predicted[1] + this.W_m[2] * sigma_predicted[2];
                      
        let P_bar = this.Q;
        for (let i = 0; i < 3; i++) { P_bar += this.W_c[i] * (sigma_predicted[i] - x_bar) ** 2; }

        this.x_pred = [x_bar];
        this.P_pred = [P_bar];
        return { x: this.x_pred, P: this.P_pred };
    }

    update(accel_input, measurement, dt) {
        this.predict(accel_input, dt);

        const sigma_predicted = this.generateSigmaPoints().map(s => this.stateTransition(s, accel_input, dt));
        const sigma_measurement = sigma_predicted.map(s => this.observationFunction(s));
        const y_bar = this.W_m[0] * sigma_measurement[0] + this.W_m[1] * sigma_measurement[1] + this.W_m[2] * sigma_measurement[2];

        let Pyy = measurement.R_dyn; 
        for (let i = 0; i < 3; i++) { Pyy += this.W_c[i] * (sigma_measurement[i] - y_bar) ** 2; }

        let Pxy = 0;
        for (let i = 0; i < 3; i++) { Pxy += this.W_c[i] * (sigma_predicted[i] - this.x_pred[0]) * (sigma_measurement[i] - y_bar); }
        
        const K = Pxy / Pyy;
        const y_diff = measurement.spd - y_bar;
        this.x[0] = this.x_pred[0] + K * y_diff;
        this.P[0] = this.P_pred[0] - K * Pyy * K;
        
        // ZUPT (Zero Velocity Update) si l'incertitude est faible et la vitesse aussi
        if (this.P[0] < this.R_base * 0.1 && this.x[0] < MIN_SPD) {
             this.x[0] = 0;
             this.P[0] = this.P[0] * 0.5;
        }

        return { kSpd: this.x[0], kUncert: this.P[0] };
    }
}


// ===========================================
// CLASSE QUATERNION (Pour gestion de l'IMU)
// ===========================================
class Quaternion {
    constructor(w = 1, x = 0, y = 0, z = 0) {
        this.w = w; this.x = x; this.y = y; this.z = z;
    }

    // Estime l'orientation à partir de l'accélération (assumant que le reste est la gravité)
    static fromAcc(ax, ay, az) {
        const g = Math.sqrt(ax * ax + ay * ay + az * az);
        if (g === 0) return new Quaternion(); 

        const gx = ax / g, gy = ay / g, gz = az / g;
        
        // Calcul des angles d'Euler à partir de l'accélération
        const roll = Math.atan2(gy, gz);
        const pitch = Math.atan2(-gx, Math.sqrt(gy * gy + gz * gz));
        const yaw = 0; // L'accéléromètre seul ne peut pas déterminer le lacet (yaw)

        // Conversion en Quaternion
        const c1 = Math.cos(roll / 2), s1 = Math.sin(roll / 2);
        const c2 = Math.cos(pitch / 2), s2 = Math.sin(pitch / 2);
        const c3 = Math.cos(yaw / 2), s3 = Math.sin(yaw / 2);

        const w = c1 * c2 * c3 - s1 * s2 * s3;
        const x = s1 * c2 * c3 + c1 * s2 * s3;
        const y = c1 * s2 * c3 - s1 * c2 * s3;
        const z = s1 * s2 * c3 + c1 * c2 * s3;

        return new Quaternion(w, x, y, z);
    }

    toEuler() {
        const x = this.x, y = this.y, z = this.z, w = this.w;
        // Roll (Rotation autour de X)
        const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
        // Pitch (Rotation autour de Y)
        let pitch = 2 * (w * y - z * x);
        pitch = Math.asin(Math.min(1, Math.max(-1, pitch)));
        // Yaw (Rotation autour de Z)
        const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

        return { roll: roll * R2D, pitch: pitch * R2D, yaw: yaw * R2D };
    }
}


// ===========================================
// FONCTIONS DE FILTRAGE ET DE MODÈLE
// ===========================================

function dist2D(lat1, lon1, lat2, lon2, R_earth) {
    // Formule Haversine
    const dLat = (lat2 - lat1) * D2R;
    const dLon = (lon2 - lon1) * D2R;
    const lat1Rad = lat1 * D2R;
    const lat2Rad = lat2 * D2R;

    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1Rad) * Math.cos(lat2Rad) * Math.sin(dLon / 2) ** 2;
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R_earth * c; 
}

function getKalmanR(accRaw, kAlt, lastP, env) {
    // Calcul de la covariance de mesure R, dépendante de la précision GPS brute (accRaw)
    let R_gps_base = Math.min(accRaw, 100) ** 2; 

    if (accRaw > 100) { R_gps_base = 100 ** 2 + (accRaw - 100) * 10; }
    
    // Facteur environnemental (influence le bruit en fonction du contexte sélectionné)
    const env_mult = ENVIRONMENT_FACTORS[env]?.R_MULT || 1.0;
    
    let R_dyn = Math.min(R_gps_base * env_mult, UKF_R_MAX);

    if (lastP < 800) { R_dyn *= 1.1; }

    return Math.max(R_dyn, R_ALT_MIN); 
}

function kFilterAltitude(kAlt_prev, P_prev, altRaw_gps, R_gps, dt, baroAlt) {
    // Filtre de Kalman 1D simple pour l'altitude (fusion GPS + Baro)
    const Q = 0.01;
    const kAlt_pred = kAlt_prev || altRaw_gps || 0;
    const P_pred = P_prev + Q;

    // Mise à jour 1 : GPS
    let H_gps = 1; 
    let R_eff_gps = R_gps || R_ALT_MIN; 
    let K_gps = P_pred * H_gps / (H_gps * P_pred * H_gps + R_eff_gps);
    let kAlt_1 = kAlt_pred + K_gps * (altRaw_gps - kAlt_pred);
    let P_1 = (1 - K_gps * H_gps) * P_pred;
    
    // Mise à jour 2 : Baromètre (si disponible)
    if (baroAlt !== null && P_1 > R_ALT_MIN * 0.1) {
        const R_baro = 5.0; 
        let H_baro = 1;
        let K_baro = P_1 * H_baro / (H_baro * P_1 * H_baro + R_baro);
        
        let kAlt_new = kAlt_1 + K_baro * (baroAlt - kAlt_1);
        let P_new = (1 - K_baro * H_baro) * P_1;
        
        return { kAlt: kAlt_new, kAltUncert: P_new };
    }

    return { kAlt: kAlt_1, kAltUncert: P_1 };
}

function getBarometricAltitude(P_hPa, P_ref_hPa, T_K) {
    // Formule de l'atmosphère standard ISA
    if (P_hPa === null || T_K === null) return null;
    
    const T0 = 288.15; // Température standard au niveau de la mer (K)
    const L = 0.0065;  // Taux de décroissance de la température (K/m)
    const g = G_ACC; 
    const R = R_AIR; 
    
    // Équation barométrique standard
    const alt = (T_K / L) * (1 - (P_hPa / P_ref_hPa)**(R * L / g));
    
    return alt;
}

function calculateMRF(alt, netherMode) {
    // Facteur de correction pour la distance parcourue (simulé)
    if (netherMode) { return 8.0; } // Mode Nether (Minecraft)
    if (alt < -20) { return 1.1; } 
    return 1.0;
}

function updateCelestialBody(body, alt, rotationRadius, angularVelocity) {
    // Mise à jour des constantes physiques de base
    let G_ACC_NEW = 9.80665;
    let R_ALT_CENTER_REF_NEW = 6371000;

    switch (body) {
        case 'MARS': G_ACC_NEW = 3.721; R_ALT_CENTER_REF_NEW = 3389500; break;
        case 'MOON': G_ACC_NEW = 1.62; R_ALT_CENTER_REF_NEW = 1737400; break;
        case 'ROTATING':
            // Simule un objet tournant (ex: station spatiale)
            G_ACC_NEW = rotationRadius * angularVelocity ** 2;
            R_ALT_CENTER_REF_NEW = 10000; break;
        case 'EARTH':
        default: break;
    }
    
    G_ACC = G_ACC_NEW;
    R_ALT_CENTER_REF = R_ALT_CENTER_REF_NEW;

    return { G_ACC: G_ACC_NEW, R_ALT_CENTER_REF: R_ALT_CENTER_REF_NEW };
}
// =================================================================
// BLOC 3/4 : Services Externes & Calculs Astro/Physique
// Dépend de: app_constants.js (BLOC 1)
// =================================================================

const NTP_API_URL = 'https://worldtimeapi.org/api/ip'; 
// NOTE: REMPLACEZ CETTE CLEF PAR VOTRE CLEF OPENWEATHERMAP
const WEATHER_API_KEY = 'YOUR_WEATHER_API_KEY_HERE'; 
const WEATHER_API_URL = 'https://api.openweathermap.org/data/2.5/weather'; 

// ===========================================
// GESTION DU TEMPS ET NTP
// ===========================================

async function syncH(lServH, lLocH) {
    // Synchronisation de l'heure via NTP
    if (lServH === null) {
        try {
            const response = await fetch(NTP_API_URL);
            const data = await response.json();
            
            if (data.unixtime) {
                const now_ms = Date.now();
                const serv_ms = data.unixtime * 1000;
                
                if (document.getElementById('local-time') && Math.abs(serv_ms - now_ms) > 1000) {
                     document.getElementById('local-time').textContent = `🕒 CORRECTION: ${(serv_ms - now_ms) / 1000}s`;
                }
                return { lServH: serv_ms, lLocH: now_ms };
            }
        } catch (e) {
            if (document.getElementById('local-time')) {
                document.getElementById('local-time').textContent = '❌ SYNCHRO ÉCHOUÉE (Local)';
            }
            return { lServH: Date.now(), lLocH: Date.now() };
        }
    }
    return { lServH: lServH, lLocH: lLocH };
}

function getCDate(lServH, lLocH) {
    // Retourne l'heure corrigée par l'offset NTP
    if (lServH === null || lLocH === null) { return new Date(); }
    const offset = lServH - lLocH;
    return new Date(Date.now() + offset);
}

// ===========================================
// CALCULS GÉOPHYSIQUES ET MÉTÉOROLOGIQUES
// ===========================================

function getWGS84Gravity(lat, alt) {
    // Calcul de l'accélération gravitationnelle selon le modèle WGS84
    const latRad = lat * D2R;
    const g0 = 9.780327 * (1 + 0.0053024 * Math.sin(latRad)**2 - 0.0000058 * Math.sin(2 * latRad)**2);
    const FAC = 3.086e-6 * alt; // Correction de l'altitude (Free-Air)
    
    return g0 - FAC; 
}

function getTrueAirspeed(spdMS, rhoAir) {
    // Calcul de la True Airspeed (TAS) à partir de la vitesse indiquée et de la densité de l'air
    const rho0 = RHO_SEA_LEVEL;
    if (rhoAir === 0 || rhoAir === null) return spdMS;
    
    return spdMS * Math.sqrt(rho0 / rhoAir);
}

function getSpeedOfSound(tempK) {
    // Calcul de la vitesse du son (fonction de la température)
    return Math.sqrt(GAMMA * R_AIR * tempK);
}


// ===========================================
// SERVICES MÉTÉO (OpenWeatherMap)
// ===========================================

async function fetchWeather(lat, lon) {
    // Récupération de la pression et de la température via API
    if (!WEATHER_API_KEY.includes('YOUR_WEATHER_API_KEY')) {
        const url = `${WEATHER_API_URL}?lat=${lat}&lon=${lon}&units=metric&appid=${WEATHER_API_KEY}`;
        try {
            const response = await fetch(url);
            const data = await response.json();

            if (data.main) {
                const P_hPa = data.main.pressure;
                const T_C = data.main.temp;
                const T_K = T_C + 273.15;
                const P_Pa = P_hPa * 100; 
                
                // Calcul de la densité de l'air (rho = P / (R * T))
                const air_density = P_Pa / (R_AIR * T_K);

                return {
                    pressure_hPa: P_hPa,
                    tempC: T_C,
                    tempK: T_K,
                    air_density: air_density
                };
            }
        } catch (e) {
            console.error("Erreur de récupération météo:", e);
        }
    }
    return null;
}

// ===========================================
// CALCULS ASTRONOMIQUES (Simplifiés)
// ===========================================

function updateAstro(lat, lon, lServH, lLocH) {
    const now = getCDate(lServH, lLocH);
    
    // Calcul de l'Heure Solaire Moyenne (MST)
    const utcHours = now.getUTCHours() + now.getUTCMinutes() / 60 + now.getUTCSeconds() / 3600;
    const mstHours = utcHours + lon / 15;
    
    const h = Math.floor(mstHours % 24);
    const m = Math.floor((mstHours * 60) % 60);
    const s = Math.floor((mstHours * 3600) % 60);
    
    if (document.getElementById('mst')) {
        document.getElementById('mst').textContent = 
            `${h.toString().padStart(2, '0')}:${m.toString().padStart(2, '0')}:${s.toString().padStart(2, '0')}`;
    }
    
    // Les calculs d'altitude et de phase sont omis ici pour la simplicité DOM
    if (document.getElementById('sun-alt')) document.getElementById('sun-alt').textContent = `~Calcul en cours...`;
    if (document.getElementById('moon-phase-name')) document.getElementById('moon-phase-name').textContent = `~Calcul en cours...`;
        }
// =================================================================
// BLOC 4/4 : Logique Applicative Principale (updateDisp & DOM/Init)
// Dépend de: BLOC 1, BLOC 2, BLOC 3
// =================================================================

// --- VARIABLES D'ÉTAT (Globales) ---
let wID = null, domID = null, lPos = null, lat = null, lon = null, sTime = null;
let distM = 0, maxSpd = 0;
let kSpd = 0, kUncert = UKF_R_MAX; 
let timeMoving = 0; 
let lServH = null, lLocH = null; 
let lastFSpeed = 0; 
let kAlt = null;      
let kAltUncert = 10;  
let ukfSpeed = null; 

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

// Données externes/fusionnées
let lastP_hPa = BARO_ALT_REF_HPA, lastT_K = 288.15, currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound = 343;

// IMU/Quaternion State
let real_accel_x = 0, real_accel_y = 0, real_accel_z = 0;
let currentAttitude = new Quaternion(); 
let lastAccelMagnitude = 0;

// Objets Map (Leaflet)
let map, marker, circle;
let lastMapUpdate = 0;


// --- FONCTION UTILITAIRE DOM ---
const $ = id => document.getElementById(id);


// ===========================================
// GESTION DES CAPTEURS ET CONTRÔLES
// ===========================================
function imuMotionHandler(event) {
    let accX = 0, accY = 0, accZ = 0;
    
    // Récupère l'accélération AVEC la gravité (plus fiable pour l'orientation)
    if (event.accelerationIncludingGravity) { 
        accX = event.accelerationIncludingGravity.x || 0;
        accY = event.accelerationIncludingGravity.y || 0;
        accZ = event.accelerationIncludingGravity.z || 0;
    } else {
        if ($('imu-status')) $('imu-status').textContent = "Erreur (Capteur N/A)";
        return;
    }
    
    // Normalisation si les données sont trop brutes
    if (Math.abs(accZ) > 30) { accX /= 9.81; accY /= 9.81; accZ /= 9.81; }

    real_accel_x = accX;
    real_accel_y = accY;
    real_accel_z = accZ;

    lastAccelMagnitude = Math.sqrt(accX**2 + accY**2 + accZ**2);
    // Utilise l'accélération pour estimer l'orientation
    currentAttitude = Quaternion.fromAcc(accX, accY, accZ); 
    
    if ($('imu-status')) $('imu-status').textContent = "Actif";
}

function startIMUListeners() {
    // Demande de permission pour iOS/Safari et ajout du listener
    const requestPermission = (EventClass, handler) => {
        if (typeof EventClass.requestPermission === 'function') {
            EventClass.requestPermission().then(permissionState => {
                if (permissionState === 'granted') {
                    window.addEventListener(EventClass.name.toLowerCase(), handler);
                }
            });
        } else {
            window.addEventListener(EventClass.name.toLowerCase(), handler);
        }
    };
    if (window.DeviceMotionEvent) requestPermission(DeviceMotionEvent, imuMotionHandler);
    if ($('imu-status')) $('imu-status').textContent = "Capteurs en attente...";
}

function stopIMUListeners() {
    if (window.DeviceMotionEvent) window.removeEventListener('devicemotion', imuMotionHandler);
    if ($('imu-status')) $('imu-status').textContent = "Inactif";
    real_accel_x = real_accel_y = real_accel_z = 0;
}

function startGPS() {
    if (wID !== null) return; 
    const options = GPS_OPTS[currentGPSMode]; // Utilise le mode sélectionné
    wID = navigator.geolocation.watchPosition(updateDisp, handleErr, options);
    startIMUListeners(); 
    if ($('toggle-gps-btn')) {
        $('toggle-gps-btn').textContent = '⏸️ PAUSE GPS';
        $('toggle-gps-btn').style.backgroundColor = '#ffc107'; 
    }
    emergencyStopActive = false; 
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

function toggleGPS() {
    // Fonction de bascule du bouton principal
    if (wID !== null) {
        stopGPS(true); 
    } else {
        startGPS(); 
    }
}

function handleErr(err) {
    if ($('gps-precision')) $('gps-precision').textContent = `Erreur: ${err.message}`;
    if (err.code === 1) { 
        stopGPS();
        alert("Accès à la géolocalisation refusé. Veuillez l'activer.");
    }
}


// --- MAP (Leaflet) ---

function initMap() {
    try {
        if ($('map') && typeof L !== 'undefined' && !map) { 
            map = L.map('map').setView([0, 0], 2);
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                attribution: '© OpenStreetMap contributors'
            }).addTo(map);
            marker = L.marker([0, 0]).addTo(map);
            circle = L.circle([0, 0], { color: 'red', fillColor: '#f03', fillOpacity: 0.5, radius: 10 }).addTo(map);
        }
    } catch (e) {
        if ($('map')) $('map').innerHTML = "Erreur d'initialisation de la carte (Leaflet non chargé).";
    }
}

function updateMap(lat, lon, acc) {
    if (map && marker) {
        marker.setLatLng([lat, lon]);
        circle.setLatLng([lat, lon]).setRadius(acc * R_FACTOR_RATIO); 
        const now = Date.now();
        if (now - lastMapUpdate > MAP_UPDATE_INTERVAL && kSpd > MIN_SPD) {
            map.setView([lat, lon], map.getZoom() > 10 ? map.getZoom() : 16); 
            lastMapUpdate = now;
        } else if (map.getZoom() < 10 && (Date.now() - lastMapUpdate > 5000)) {
            map.setView([lat, lon], 12);
            lastMapUpdate = now;
        }
    }
}


// ===========================================
// FONCTION PRINCIPALE DE MISE À JOUR GPS (updateDisp)
// ===========================================

function updateDisp(pos) {
    if (emergencyStopActive) return; 

    // --- 1. ACQUISITION & INIT ---
    const cTimePos = pos.timestamp;
    let cLat = pos.coords.latitude;
    let cLon = pos.coords.longitude;
    let altRaw = pos.coords.altitude;
    let accRaw = pos.coords.accuracy;
    let headingRaw = pos.coords.heading; 

    if (gpsAccuracyOverride > 0.0) { accRaw = gpsAccuracyOverride; }

    if (ukfSpeed === null) {
        // Initialisation du UKF de vitesse
        ukfSpeed = new UKF([0], [UKF_R_MAX], UKF_Q_SPD, UKF_R_MAX, KAPPA);
    }

    if (lPos === null) {
        lPos = pos; kAlt = altRaw; lat = cLat; lon = cLon;
        sTime = Date.now();
        updateMap(cLat, cLon, accRaw);
        return; 
    }
    
    const dt = (cTimePos - lPos.timestamp) / 1000;
    if (dt < MIN_DT || dt > 10) { lPos = pos; return; } 
    
    // --- 2. GESTION DU SIGNAL & BRUIT (R) ---
    // R dépend de l'environnement sélectionné par l'utilisateur
    let R_dyn = getKalmanR(accRaw, kAlt, lastP_hPa, selectedEnvironment); 
    let isSignalPoor = (accRaw > 200 || R_dyn >= UKF_R_MAX * 0.75);
    let modeStatus = '';
    
    if (isSignalPoor) { 
        // En mode signal faible, on utilise le dead reckoning et les valeurs filtrées
        modeStatus = `⚠️ ESTIMATION SEULE (${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY}) : Signal Faible`;
        cLat = lat; 
        cLon = lon;
        altRaw = kAlt; 
        if ($('gps-precision')) $('gps-precision').textContent = `❌ ${accRaw.toFixed(0)} m (Estimation)`; 
    } else {
        modeStatus = `🚀 UKF FUSION (Mode ${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY})`;
        lat = cLat; 
        lon = cLon;
        if ($('gps-precision')) $('gps-precision').textContent = `${accRaw.toFixed(2)} m`; 
    }
    
    // --- 3. FILTRAGE UKF ALTITUDE ---
    const baroAlt = getBarometricAltitude(lastP_hPa, BARO_ALT_REF_HPA, lastT_K);
    const { kAlt: kAlt_new, kAltUncert: kAltUncert_new } = kFilterAltitude(
        kAlt, kAltUncert, altRaw, pos.coords.altitudeAccuracy || R_ALT_MIN, dt, baroAlt
    );
    kAlt = kAlt_new;
    kAltUncert = kAltUncert_new;
    
    // --- 4. CALCUL VITESSE BRUTE 3D ---
    const dist2D_val = dist2D(lPos.coords.latitude, lPos.coords.longitude, cLat, cLon, R_ALT_CENTER_REF);
    const altDiff = (kAlt_new || 0) - (lPos.kAlt_old || 0);
    const dist3D = Math.sqrt(dist2D_val ** 2 + altDiff ** 2);
    let spd3D_raw = dist3D / dt; 
    
    // L'accélération inertielle selon l'axe Z (vers l'avant dans l'orientation du téléphone)
    const accel_sensor_input = real_accel_z; 

    // --- 5. LOGIQUE UKF VITESSE & ZUPT ---
    const ukf_measurement = { spd: spd3D_raw, R_dyn: R_dyn };
    const { kSpd: fSpd, kUncert: kUncert_new } = ukfSpeed.update(accel_sensor_input, ukf_measurement, dt);
    kSpd = fSpd;
    kUncert = kUncert_new;
    
    const sSpdFE = fSpd < MIN_SPD ? 0 : fSpd; 
    if (sSpdFE === 0 && R_dyn < UKF_R_MAX) { modeStatus = '✅ ZUPT (Vélocité Nulle Forcée)'; } 

    // --- 6. CALCULS AVANCÉS ---
    let accel_long = 0;
    if (dt > 0.05) { accel_long = (sSpdFE - lastFSpeed) / dt; }
    lastFSpeed = sSpdFE;

    R_FACTOR_RATIO = calculateMRF(kAlt_new, netherMode); 
    distM += sSpdFE * dt * R_FACTOR_RATIO; 
    
    if (sSpdFE > MIN_SPD) { timeMoving += dt; }
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    
    let local_g = G_ACC;
    if (currentCelestialBody === 'EARTH') {
        local_g = getWGS84Gravity(cLat, kAlt_new);
        G_ACC = local_g;
    }
    
    const tas_ms = getTrueAirspeed(sSpdFE, currentAirDensity);
    const coriolis_force = 2 * currentMass * OMEGA_EARTH * sSpdFE * Math.abs(Math.cos(lat * D2R));


    // --- 7. MISE À JOUR DU DOM ---
    if ($('speed-stable')) $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(5)}`;
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${sSpdFE.toFixed(3)} m/s`;
    if ($('speed-max')) $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    if ($('speed-3d-inst')) $('speed-3d-inst').textContent = `${(spd3D_raw * KMH_MS).toFixed(5)} km/h`; 
    if ($('perc-speed-sound')) $('perc-speed-sound').textContent = `${(sSpdFE / currentSpeedOfSound * 100).toFixed(2)} %`;
    if ($('distance-total-km')) $('distance-total-km').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    if ($('tas-display')) $('tas-display').textContent = `${tas_ms.toFixed(3)} m/s`;
    if ($('ukf-status-text')) $('ukf-status-text').textContent = modeStatus;
    
    // Position & Altitude
    if ($('lat-display')) $('lat-display').textContent = `${lat.toFixed(6)} °`;
    if ($('lon-display')) $('lon-display').textContent = `${lon.toFixed(6)} °`;
    if ($('alt-display')) $('alt-display').textContent = kAlt_new !== null ? `${kAlt_new.toFixed(2)} m (UKF Fusion)` : 'N/A';
    if ($('baro-alt-display')) $('baro-alt-display').textContent = baroAlt !== null ? `${baroAlt.toFixed(2)} m (Baro Corrigée)` : 'N/A';
    if ($('heading-display')) $('heading-display').textContent = headingRaw !== null ? `${headingRaw.toFixed(1)} °` : 'N/A';

    // Physique
    if ($('gravity-local')) $('gravity-local').textContent = `${local_g.toFixed(5)} m/s²`;
    if ($('accel-long')) $('accel-long').textContent = `${accel_long.toFixed(3)} m/s²`;
    if ($('kinetic-energy')) $('kinetic-energy').textContent = `${(0.5 * currentMass * sSpdFE ** 2).toFixed(2)} J`;
    if ($('coriolis-force')) $('coriolis-force').textContent = `${coriolis_force.toExponential(2)} N`;
    
    // IMU (QUATERNION)
    const euler = currentAttitude.toEuler();
    if ($('accel-x')) $('accel-x').textContent = `${real_accel_x.toFixed(2)} m/s²`;
    if ($('accel-y')) $('accel-y').textContent = `${real_accel_y.toFixed(2)} m/s²`;
    if ($('accel-z')) $('accel-z').textContent = `${real_accel_z.toFixed(2)} m/s²`;
    if ($('attitude-roll')) $('attitude-roll').textContent = `${euler.roll.toFixed(2)} °`;
    if ($('attitude-pitch')) $('attitude-pitch').textContent = `${euler.pitch.toFixed(2)} °`;
    if ($('attitude-yaw')) $('attitude-yaw').textContent = `${euler.yaw.toFixed(2)} ° (Approximation)`;


    // UKF Uncertainties (P)
    if ($('ukf-uncert-spd')) $('ukf-uncert-spd').textContent = `${kUncert.toFixed(5)} m²/s² (P)`;
    if ($('ukf-uncert-alt')) $('ukf-uncert-alt').textContent = `${kAltUncert_new.toFixed(3)} m² (P)`; 
    
    // --- 8. SAUVEGARDE & MISE À JOUR CARTE ---
    lPos = pos; 
    lPos.speedMS_3D = spd3D_raw; 
    lPos.timestamp = cTimePos; 
    lPos.kAlt_old = kAlt_new; 

    updateMap(lat, lon, accRaw);
}


// ===========================================
// INITIALISATION DOM ET GESTION DES BOUTONS/SÉLECTEURS
// ===========================================

document.addEventListener('DOMContentLoaded', () => {
    
    initMap(); 
    
    // --- 1. Gestion de la Masse ---
    const massInput = $('mass-input'); 
    if (massInput) {
        massInput.addEventListener('input', () => { 
            currentMass = parseFloat(massInput.value) || 70.0; 
            if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        });
        currentMass = parseFloat(massInput.value) || 70.0; 
        if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
    }

    // --- 2. Sélecteur de Corps Céleste ---
    if ($('celestial-body-select')) {
        $('celestial-body-select').addEventListener('change', (e) => { 
            const newVals = updateCelestialBody(e.target.value, kAlt, rotationRadius, angularVelocity);
            currentCelestialBody = e.target.value;
            if ($('gravity-base')) $('gravity-base').textContent = `${newVals.G_ACC.toFixed(4)} m/s²`;
        });
    }

    // ======================================================
    // --- GESTION DES BOUTONS ET SÉLECTEURS DYNAMIQUES (CORRIGÉ) ---
    // ======================================================
    
    // 3. Bouton GPS (Démarrer/Pause)
    if ($('toggle-gps-btn')) {
        $('toggle-gps-btn').addEventListener('click', toggleGPS);
    }
    
    // 4. Sélecteur de Mode GPS (Haute/Basse Fréquence)
    if ($('gps-mode-select')) {
        $('gps-mode-select').addEventListener('change', (e) => {
            currentGPSMode = e.target.value;
            // Redémarre le GPS pour appliquer les nouvelles options immédiatement
            if (wID !== null) {
                stopGPS(false); 
                startGPS();
            }
        });
    }
    
    // 5. Sélecteur d'Environnement (Affecte R du Kalman)
    if ($('environment-select')) {
        const environmentSelect = $('environment-select');
        environmentSelect.value = selectedEnvironment;
        
        environmentSelect.addEventListener('change', (e) => {
            selectedEnvironment = e.target.value;
            if ($('ukf-status-text')) {
                 $('ukf-status-text').textContent = `Mode: ${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY}`;
            }
        });
    }

    // 6. Bouton Arrêt d'Urgence
    if ($('toggle-emergency-stop')) {
        $('toggle-emergency-stop').addEventListener('click', () => {
            emergencyStopActive = !emergencyStopActive;
            if (emergencyStopActive) {
                stopGPS(false); 
                $('toggle-emergency-stop').textContent = '🟢 SYSTÈME ACTIF (CLIC POUR ARRÊT)';
                $('toggle-emergency-stop').style.backgroundColor = 'var(--accent-color)';
                if ($('ukf-status-text')) $('ukf-status-text').textContent = '🛑 ARRÊT D’URGENCE : SYSTÈME SUSPENDU';
            } else {
                startGPS(); 
                $('toggle-emergency-stop').textContent = '🛑 ARRÊT D\'URGENCE';
                $('toggle-emergency-stop').style.backgroundColor = 'var(--error-color)';
            }
        });
    }


    // --- DÉMARRAGE DU SYSTÈME (Sync NTP puis GPS) ---
    const initVals = updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity);
    
    syncH(lServH, lLocH).then(newTimes => {
        lServH = newTimes.lServH;
        lLocH = newTimes.lLocH;
        startGPS(); 
    });

    // Boucle de mise à jour lente (Astro/Météo/Horloge)
    const DOM_SLOW_UPDATE_MS = 1000;
    if (domID === null) {
        domID = setInterval(() => {
            const currentLat = lat || 43.296; 
            const currentLon = lon || 5.370;
            
            // Astro & Temps
            if (typeof updateAstro === 'function') { updateAstro(currentLat, currentLon, lServH, lLocH); }
            if (Math.floor(Date.now() / 1000) % 60 === 0) {
                 syncH(lServH, lLocH).then(newTimes => {
                    lServH = newTimes.lServH;
                    lLocH = newTimes.lLocH;
                 });
            }
            
            // Météo & Physique (si GPS actif)
            if (lat && lon && !emergencyStopActive && typeof fetchWeather === 'function') {
                fetchWeather(lat, lon).then(data => {
                    if (data) {
                        lastP_hPa = data.pressure_hPa;
                        lastT_K = data.tempK;
                        currentAirDensity = data.air_density; 
                        currentSpeedOfSound = getSpeedOfSound(data.tempK);
                        
                        if ($('weather-status')) $('weather-status').textContent = `Localisation (${(lat).toFixed(2)}, ${(lon).toFixed(2)})`;
                        if ($('temp-air-2')) $('temp-air-2').textContent = `${data.tempC.toFixed(1)} °C`;
                        if ($('pressure-2')) $('pressure-2').textContent = `${data.pressure_hPa.toFixed(0)} hPa`;
                        if ($('air-density')) $('air-density').textContent = `${data.air_density.toFixed(3)} kg/m³`;
                        if ($('speed-sound-calc')) $('speed-sound-calc').textContent = `${currentSpeedOfSound.toFixed(2)} m/s`;
                    }
                });
            }
            
            // Mise à jour de l'horloge locale
            const now = getCDate(lServH, lLocH);
            if (now) {
                if ($('local-time') && !$('local-time').textContent.includes('SYNCHRO ÉCHOUÉE')) {
                    $('local-time').textContent = now.toLocaleTimeString('fr-FR');
                }
                if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
            }
            
        }, DOM_SLOW_UPDATE_MS); 
    }
});
