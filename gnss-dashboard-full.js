// =================================================================
// BLOC 1/4 : Constantes, État Global & Utilitaires de Base (gnss-dashboard-full-final.js)
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);

const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        return (decimals === 0 ? '0' : '0.00') + suffix;
    }
    return val.toFixed(decimals) + suffix;
};

const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        const zeroDecimals = '0.' + Array(decimals).fill('0').join('');
        return zeroDecimals + 'e+0' + suffix;
    }
    return val.toExponential(decimals) + suffix;
};

// --- API Endpoints (Proxy Vercel pour Météo/Pollution/NTP) ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const PROXY_POLLUTANT_ENDPOINT = `${PROXY_BASE_URL}/api/pollutants`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES FONDAMENTALES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const KMH_MS = 3.6;          // Conversion m/s -> km/h
const C_L = 299792458;       // Vitesse de la lumière (m/s)
const R_AIR = 287.058;       // Constante spécifique de l'air sec (J/kg·K)
const GAMMA_AIR = 1.4;       // Indice adiabatique de l'air
const TEMP_SEA_LEVEL_K = 288.15; // 15°C en Kelvin
const BARO_ALT_REF_HPA = 1013.25; // Pression au niveau de la mer (hPa)
const RHO_SEA_LEVEL = 1.225; // Densité de l'air au niveau de la mer (kg/m³)
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const DOM_SLOW_UPDATE_MS = 2000; // 0.5 Hz
const DOM_FAST_UPDATE_MS = 50;   // 20 Hz (Pour l'UKF)

// --- CONSTANTES GÉOPHYSIQUES (WGS84) ---
let G_ACC = 9.80665;         // Gravité locale (mis à jour par WGS84)
const WGS84_A = 6378137.0;   // Rayon équatorial WGS84 (m)
const WGS84_F = 1 / 298.257223563; // Aplatissement WGS84
const WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F; // Excentricité au carré
const WGS84_G_EQUATOR = 9.780327; // Gravité à l'équateur
const WGS84_BETA = 0.0053024; // Facteur de gravité

// --- ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE ---
let ukf = null;             // Le filtre de Kalman
let wID = null;             // ID du watchPosition GPS
let domFastID = null;       // ID de l'intervalle d'affichage rapide
let domSlowID = null;       // ID de l'intervalle d'affichage lent
let lastGpsTimestamp = 0;   // Dernière mise à jour GPS (ms)
let lastIMUTimestamp = 0;   // Dernière mise à jour IMU (ms)
let lPos = null;            // Dernière position brute de l'API GPS
let currentPosition = { lat: 43.2964, lon: 5.3697, alt: 10.0, acc: 10.0, spd: 0.0, head: 0.0 }; // État UKF
let accel = { x: 0, y: 0, z: G_ACC }; // Accélération (m/s²) - Z est la gravité au repos
let gyro = { x: 0, y: 0, z: 0 };    // Rotation (rad/s)
let currentAirDensity = RHO_SEA_LEVEL; // Densité de l'air (kg/m³)
let currentSpeedOfSound = 343.0;     // Vitesse du son (m/s)
let systemClockOffsetMS = 0; // Décalage NTP (ms)
let lastP_hPa = BARO_ALT_REF_HPA; // Dernière pression (hPa)
let lastT_K = TEMP_SEA_LEVEL_K; // Dernière température (K)
let distM = 0; maxSpd = 0; timeMoving = 0; // Statistiques
let emergencyStopActive = false;
let map = null, marker = null, pathLine = null; // Leaflet
let lastKnownWeather = null;
let lastKnownPollutants = null;
let lastMapClickLatLon = null; // Pour ciblage
let targetLat = null, targetLon = null;

// --- CONFIGURATIONS GPS ---
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};

/** ⌚ Récupère l'heure corrigée (synchronisée NTP) */
function getCDate() { 
    return new Date(Date.now() + systemClockOffsetMS);
}

/** 🧮 Calcule la gravité locale basée sur le modèle WGS84 et l'altitude */
function calculateWGS84Gravity(latRad, altMeters) {
    const sinSqLat = Math.sin(latRad) ** 2;
    const g_surface = WGS84_G_EQUATOR * (1 + WGS84_BETA * sinSqLat);
    const g_alt = g_surface * (1 - 2 * altMeters / WGS84_A);
    
    G_ACC = g_alt; // Met à jour la variable globale
    return G_ACC;
}

/** 💨 Calcule la vitesse du son à partir de la température en Kelvin */
function getSpeedOfSound(tempK) {
    return Math.sqrt(GAMMA_AIR * R_AIR * tempK);
        }
// =================================================================
// BLOC 2/4 : Modèles UKF (Fusion) & Gestionnaire Capteurs IMU (DeviceMotionEvent)
// =================================================================

// --- PARAMÈTRES AVANCÉS DU FILTRE UKF ---
const UKF_CONSTS = {
    Q_PROCESS_NOISE_POS: 0.1,  // Bruit de processus (Position)
    Q_PROCESS_NOISE_VEL: 0.05, // Bruit de processus (Vitesse)
    R_GPS_MIN: 1.0,            // Bruit de mesure minimum GPS
    R_GPS_MAX: 50.0,           // Bruit de mesure maximum GPS
    R_ACC_NOISE: 0.1,          // Bruit de mesure Accéléromètre
    R_GYRO_NOISE: 0.05,        // Bruit de mesure Gyroscope
    MIN_SPD_MS: 0.05,          // Vitesse minimale pour être considéré en mouvement
    N_STATES: 21,              // États: POS(3), VEL(3), QUAT(4), BIAS_G(3), BIAS_A(3), MAG_ERR(5)
};

/**
 * Classe ProfessionalUKF (Unscented Kalman Filter - 21 États)
 * NOTE: L'implémentation complète des matrices UKF (mat.inv, mat.dot, etc.) 
 * est dépendante de 'math.min.js' et est très volumineuse. 
 * Le code ci-dessous est la structure fonctionnelle avec un 
 * **mode de repli simplifié** pour l'état de sortie, en attendant 
 * l'intégration de l'algèbre matricielle complète.
 */
class ProfessionalUKF {
    constructor(lat_init, lon_init, alt_init) {
        // Initialisation des matrices (Simplifié: doit être mat.zeros(N_STATES, 1) pour X et mat.eye(N_STATES) pour P)
        // this.X = math.matrix([...]); // État du système
        // this.P = math.matrix([...]); // Matrice de Covariance

        // État de sortie (pour l'affichage en attendant l'implémentation complète)
        this.kLat = lat_init;
        this.kLon = lon_init;
        this.kAlt = alt_init;
        this.kSpd = 0.0;
        this.kHeading = 0.0;
        this.kUncert = 1000.0; // Précision horizontale initiale
        this.kAltUncert = 1000.0; // Précision verticale initiale
        this.lastUpdateTime = performance.now();
        console.log(`UKF 21-états initialisé à Lat: ${lat_init}, Lon: ${lon_init}.`);
    }

    /** Mettre à jour le filtre avec les données GPS et IMU */
    update(gpsData, accelData, gyroData) {
        const now = performance.now();
        const dt = (now - this.lastUpdateTime) / 1000.0; // Temps écoulé en secondes
        this.lastUpdateTime = now;

        if (dt > 1.0) { console.warn("Grand saut DT. Reset UKF partiel."); }
        
        // ==========================================================
        // 1. ÉTAPE DE PRÉDICTION (PROPAGATION par Modèle de Mouvement / IMU)
        // ==========================================================
        // Si IMU actif, cette étape utilise accelData et gyroData pour prédire X et P.
        // math.js: X_pred = f(this.X, accelData, gyroData, dt);
        // math.js: P_pred = f(this.P, dt, Q);
        
        // --- LOGIQUE DE REPLI SIMPLIFIÉE (en l'absence de l'algèbre complète) ---
        if (dt > 0) {
            // Accélération nette (corrigée de la gravité)
            // Note: Ceci est une approximation très simple
            const acc_net = Math.sqrt(accelData.x ** 2 + accelData.y ** 2 + accelData.z ** 2) - G_ACC;
            
            // Mise à jour de la vitesse (V = V_prev + a * dt)
            this.kSpd = Math.max(0, this.kSpd + acc_net * dt); 
            
            // Mise à jour de la position (P = P_prev + V * dt)
            // L'implémentation UKF réelle ferait cela en coordonnées ECEF ou ENU.
            // Nous utilisons une approximation simplifiée ici.
            // Nécessite la vitesse (kSpd) et le cap (kHeading) pour le déplacement.
            const distance = this.kSpd * dt;
            const bearingRad = this.kHeading * D2R;

            if (distance > 0) {
                const destination = turf.destination([this.kLon, this.kLat], distance / 1000, this.kHeading, {units: 'kilometers'});
                this.kLat = destination.geometry.coordinates[1];
                this.kLon = destination.geometry.coordinates[0];
            }
            
            // L'incertitude augmente à chaque prédiction (P = P + Q)
            this.kUncert += UKF_CONSTS.Q_PROCESS_NOISE_POS * dt;
        }

        // ==========================================================
        // 2. ÉTAPE DE CORRECTION (MISE À JOUR par Mesure GPS)
        // ==========================================================
        if (gpsData) {
            // math.js: R = getRMatrix(gpsData.accuracy); // Bruit de mesure GPS
            // math.js: K = P_pred * H_T * inv(H * P_pred * H_T + R); // Gain de Kalman
            // math.js: X_corr = X_pred + K * (Z_GPS - h(X_pred)); // Correction de l'état
            // math.js: P_corr = (I - K * H) * P_pred; // Correction de la covariance

            // --- LOGIQUE DE REPLI SIMPLIFIÉE : Fuseau par pondération ---
            const alpha = 1.0 - Math.min(1.0, this.kUncert / (gpsData.accuracy || 10.0)); // Poids du GPS (plus GPS est précis, plus alpha est faible)
            
            this.kLat = this.kLat * alpha + gpsData.lat * (1.0 - alpha);
            this.kLon = this.kLon * alpha + gpsData.lon * (1.0 - alpha);
            this.kAlt = this.kAlt * alpha + gpsData.alt * (1.0 - alpha);
            this.kSpd = this.kSpd * alpha + (gpsData.speed || 0.0) * (1.0 - alpha);
            this.kHeading = gpsData.heading || this.kHeading;

            // L'incertitude est mise à jour pour refléter la mesure (P = P_corr)
            this.kUncert = gpsData.accuracy || this.kUncert; 
            this.kAltUncert = gpsData.altAccuracy || this.kAltUncert;
        }

        // 3. Mise à jour de l'état de sortie
        return { 
            lat: this.kLat, lon: this.kLon, alt: this.kAlt, 
            speed: this.kSpd, heading: this.kHeading, 
            accuracy: this.kUncert, altAccuracy: this.kAltUncert 
        };
    }
}
// Fin de la classe ProfessionalUKF


// --- GESTIONNAIRE DE SECOURS IMU (DeviceMotionEvent) ---

/** 👂 Gère l'événement DeviceMotionEvent pour mettre à jour les variables accel et gyro */
function handleDeviceMotion(event) {
    if (emergencyStopActive) return;

    const acc = event.accelerationIncludingGravity;
    if (acc.x === null) return; 

    // Mise à jour de l'état global 'accel' (Accélération avec gravité)
    accel.x = acc.x;
    accel.y = acc.y;
    accel.z = acc.z; // Z est généralement l'axe vertical/gravité

    // Lecture des données de rotation
    if (event.rotationRate) {
        const rot = event.rotationRate;
        gyro.x = rot.alpha || rot.x;
        gyro.y = rot.beta || rot.y;
        gyro.z = rot.gamma || rot.z;
    }
    
    if ($('imu-status')) $('imu-status').textContent = "Actif (DeviceMotion)";

    // Démarre la boucle rapide si l'IMU est le premier à se réveiller
    if (!domFastID) {
        startFastLoop();
    }
    
    lastIMUTimestamp = performance.now();
}
// =================================================================
// BLOC 3/4 : Logique GPS, API et Boucles de Mise à Jour
// =================================================================

// --- LOGIQUE NTP (Correction de l'heure) ---

/** ⌚ Récupère l'heure du serveur NTP pour corriger l'horloge locale */
function syncH() {
    if ($('local-time')) $('local-time').textContent = 'Synchronisation...';
    fetch(SERVER_TIME_ENDPOINT)
        .then(res => res.json())
        .then(data => {
            const serverTime = new Date(data.utc_datetime).getTime();
            const localTime = Date.now();
            systemClockOffsetMS = serverTime - localTime; 
            
            console.log("Synchronisation NTP réussie. Décalage:", systemClockOffsetMS.toFixed(0), "ms");
            if ($('local-time')) $('local-time').textContent = getCDate().toLocaleTimeString('fr-FR');
        })
        .catch(error => {
            console.error("Erreur de synchro NTP. Utilisation de l'heure locale.", error);
            if ($('local-time')) $('local-time').textContent = 'SYNCHRO ÉCHOUÉE.';
            systemClockOffsetMS = 0;
        });
}

// --- LOGIQUE GPS ---

/** ✅ Callback de succès de la géolocalisation */
function gpsUpdateCallback(pos) {
    if (emergencyStopActive) return;
    const now = performance.now();
    const { latitude, longitude, altitude, accuracy, altitudeAccuracy, speed, heading } = pos.coords;

    lPos = pos; 
    
    // Calcul de DT et mise à jour des stats
    const dt_gps = (now - lastGpsTimestamp) / 1000.0;
    if (lastGpsTimestamp !== 0 && ukf) {
        // Intégration de la distance basée sur la vitesse UKF
        if (ukf.kSpd > UKF_CONSTS.MIN_SPD_MS) {
            distM += ukf.kSpd * dt_gps;
            timeMoving += dt_gps;
            maxSpd = Math.max(maxSpd, ukf.kSpd);
        }
    }
    lastGpsTimestamp = now;

    // Données d'entrée pour l'UKF
    const gpsData = {
        lat: latitude, lon: longitude, alt: altitude || currentPosition.alt, 
        accuracy: accuracy, altAccuracy: altitudeAccuracy || currentPosition.acc,
        speed: speed || ukf.kSpd,
        heading: heading || ukf.kHeading
    };
    
    // 3. Exécution de l'UKF (fusion)
    const newUKFState = ukf.update(gpsData, accel, gyro);
    currentPosition = newUKFState;
    
    // 4. Mise à jour de la carte
    updateMap();
    
    // 5. Démarrage de la boucle rapide si le GPS fonctionne
    if (!domFastID) {
        startFastLoop();
    }
}

/** ❌ Callback d'erreur de la géolocalisation */
function gpsErrorCallback(err) {
    console.warn(`ERREUR GPS (${err.code}): ${err.message}`);
    let errMsg = `GPS Erreur ${err.code}: ${err.message}`;
    if (err.code === 1) errMsg = "Permission GPS refusée.";
    
    if ($('gps-precision')) $('gps-precision').textContent = errMsg;
    if (err.code === 1) stopGPS(true);
    
    // Si l'UKF est actif, il continue en mode INS (Inertial Navigation System)
    if (!domFastID) startSlowLoop(); 
}

/** ▶️ Démarre la géolocalisation et les capteurs IMU */
function startGPS(freq = 'HIGH_FREQ') {
    if (emergencyStopActive || wID) return;
    
    // 1. Démarrage des capteurs IMU (DeviceMotionEvent) avec gestion de permission iOS
    const startIMU = () => {
        if (window.DeviceMotionEvent) {
            window.addEventListener('devicemotion', handleDeviceMotion, true);
            if ($('imu-status')) $('imu-status').textContent = "En attente de données...";
        } else {
            if ($('imu-status')) $('imu-status').textContent = "Désactivé : DeviceMotion non supporté.";
        }
    };

    if (typeof DeviceMotionEvent.requestPermission === 'function') {
        DeviceMotionEvent.requestPermission()
            .then(permissionState => {
                if (permissionState === 'granted') startIMU();
                else if ($('imu-status')) $('imu-status').textContent = "Désactivé : Mouvement refusé.";
            })
            .catch(error => console.error("Échec demande permission IMU:", error));
    } else {
        startIMU(); // Android/Desktop
    }

    // 2. Démarrage de l'API GPS
    if (navigator.geolocation) {
        wID = navigator.geolocation.watchPosition(gpsUpdateCallback, gpsErrorCallback, GPS_OPTS[freq]);
        if ($('gps-status')) $('gps-status').textContent = "ACTIF";
        if ($('start-btn')) $('start-btn').textContent = "◼️ ARRÊT GPS";
    } else {
        alert("Géolocalisation non supportée par ce navigateur.");
        if ($('gps-status')) $('gps-status').textContent = "NON SUPPORTÉ";
    }
}

/** ◼️ Arrête la géolocalisation et les boucles */
function stopGPS(reset = false) {
    if (wID !== null && navigator.geolocation) navigator.geolocation.clearWatch(wID);
    wID = null;

    if (domFastID !== null) clearInterval(domFastID);
    domFastID = null;

    if (window.DeviceMotionEvent) {
        window.removeEventListener('devicemotion', handleDeviceMotion, true);
    }

    if ($('gps-status')) $('gps-status').textContent = "INACTIF";
    if ($('imu-status')) $('imu-status').textContent = "INACTIF";
    if ($('start-btn')) $('start-btn').textContent = "▶️ MARCHE GPS";
}

// --- LOGIQUE DES BOUCLES ---

/** 💨 Boucle rapide (UKF / 20 Hz) pour mettre à jour la fusion et la carte */
function startFastLoop() {
    if (domFastID) return;
    
    domFastID = setInterval(() => {
        if (emergencyStopActive) return;

        // Propagation UKF (Estimation de la position entre les mises à jour GPS)
        const newUKFState = ukf.update(null, accel, gyro);
        currentPosition = newUKFState;
        
        updateDOMFast(currentPosition, ukf);
        updateMap();

        // Mise à jour des statistiques de mouvement
        const dt_fast = DOM_FAST_UPDATE_MS / 1000.0;
        if (ukf.kSpd > UKF_CONSTS.MIN_SPD_MS) {
            distM += ukf.kSpd * dt_fast;
            timeMoving += dt_fast;
            maxSpd = Math.max(maxSpd, ukf.kSpd);
        }

    }, DOM_FAST_UPDATE_MS);
}

/** 🐢 Boucle lente (Météo/Astro/Gravité / 0.5 Hz) */
function startSlowLoop() {
    if (domSlowID) return;
    
    domSlowID = setInterval(() => {
        if (emergencyStopActive) return;
        
        const { lat, lon, alt } = currentPosition;
        const now = getCDate();

        // 1. Mise à jour de l'heure NTP
        if ($('local-time') && !$('local-time').textContent.includes('SYNCHRO ÉCHOUÉE')) {
            $('local-time').textContent = now.toLocaleTimeString('fr-FR');
            $('date-display').textContent = now.toLocaleDateString('fr-FR');
        }

        // 2. Mise à jour Astrologique/Géophysique
        updateAstro(lat, lon, now);
        calculateWGS84Gravity(lat * D2R, alt || 0);
        if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC.toFixed(4)} m/s²`;
        
        // 3. Mise à jour Météo/Pollution (Async)
        fetchWeather(lat, lon);
        fetchPollutants(lat, lon);
        
        updateDOMSlow();

    }, DOM_SLOW_UPDATE_MS);
}

// --- LOGIQUE API (Météo & Pollution) ---

/** ☁️ Récupère les données météo et met à jour les variables physiques */
function fetchWeather(lat, lon) {
    if (lat === 0 || lon === 0) return; 

    const url = `${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`;
    
    fetch(url)
        .then(res => res.json())
        .then(data => {
            if (data.tempK) {
                lastP_hPa = data.pressure_hPa;
                lastT_K = data.tempK;
                currentAirDensity = data.air_density;
                currentSpeedOfSound = getSpeedOfSound(data.tempK);
                lastKnownWeather = data; 
                
                if ($('weather-status')) $('weather-status').textContent = `ACTIF`;
                if ($('temp-air-2')) $('temp-air-2').textContent = `${data.tempC.toFixed(1)} °C`;
                if ($('pressure-2')) $('pressure-2').textContent = `${data.pressure_hPa.toFixed(0)} hPa`;
                if ($('humidity-2')) $('humidity-2').textContent = `${data.humidity_perc.toFixed(0)} %`;
                if ($('air-density')) $('air-density').textContent = `${data.air_density.toFixed(3)} kg/m³`;
                if ($('dew-point')) $('dew-point').textContent = `${data.dew_point.toFixed(1)} °C`;
                if ($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = `${currentSpeedOfSound.toFixed(2)} m/s`;
            } else {
                 if ($('weather-status')) $('weather-status').textContent = `❌ API Données invalides`;
            }
        }).catch(err => {
            console.error("Échec API Météo:", err);
            if ($('weather-status')) $('weather-status').textContent = `❌ API ÉCHOUÉE`;
        });
}

/** 🏭 Récupère les données de pollution (Air Quality Index) */
function fetchPollutants(lat, lon) {
    if (lat === 0 || lon === 0) return; 

    const url = `${PROXY_POLLUTANT_ENDPOINT}?lat=${lat}&lon=${lon}`;

    fetch(url)
        .then(res => res.json())
        .then(data => {
            if (data.aqi) {
                lastKnownPollutants = data;
                if ($('aqi')) $('aqi').textContent = `${data.aqi} (PM2.5: ${data.pm2_5} µg/m³)`;
                if ($('co-level')) $('co-level').textContent = `${data.co.toFixed(1)} µg/m³`;
                if ($('o3-level')) $('o3-level').textContent = `${data.o3.toFixed(1)} µg/m³`;
            } else {
                 if ($('aqi')) $('aqi').textContent = `N/A (API indisponible)`;
            }
        }).catch(err => {
            console.warn("Échec API Pollution:", err);
            if ($('aqi')) $('aqi').textContent = `N/A (Erreur API)`;
        });
        }
// =================================================================
// BLOC 4/4 : Mises à Jour DOM, Carte & Initialisation (DOMContentLoaded)
// =================================================================

// --- FONCTIONS DE MISE À JOUR DE L'AFFICHAGE ---

/** 💨 Mise à jour rapide des données de mouvement/UKF */
function updateDOMFast(state, ukfState) {
    // Vitesse / Accélération
    const speed_kmh = state.speed * KMH_MS;
    const accel_mag = Math.sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
    
    if ($('speed-ukf')) $('speed-ukf').textContent = dataOrDefault(speed_kmh, 4, ' km/h');
    if ($('accel-mag-ukf')) $('accel-mag-ukf').textContent = dataOrDefault(accel_mag, 3, ' m/s²');
    if ($('speed-max')) $('speed-max').textContent = dataOrDefault(maxSpd * KMH_MS, 4, ' km/h');
    
    // Position UKF
    if ($('latitude')) $('latitude').textContent = dataOrDefault(state.lat, 7);
    if ($('longitude')) $('longitude').textContent = dataOrDefault(state.lon, 7);
    if ($('altitude')) $('altitude').textContent = dataOrDefault(state.alt, 3, ' m');
    
    // Mach / Relativité
    const mach = state.speed / currentSpeedOfSound;
    if ($('mach-number')) $('mach-number').textContent = dataOrDefault(mach, 4);
    // Facteur de Lorentz (γ) = 1 / sqrt(1 - (v/c)²)
    const v_c_sq = (state.speed/C_L)**2;
    const lorentz_factor = 1 / Math.sqrt(1 - v_c_sq);
    if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(lorentz_factor, 8); 
    
    // Précision UKF
    if ($('ukf-precision')) $('ukf-precision').textContent = dataOrDefault(ukfState.kUncert, 2, ' m');
}

/** 🐢 Mise à jour lente des données Astro/Temps/Stats */
function updateDOMSlow() {
    // Distances / Temps
    if ($('distance-total-km')) $('distance-total-km').textContent = `${dataOrDefault(distM / 1000, 4)} km | ${dataOrDefault(distM, 2)} m`;
    if ($('time-moving')) $('time-moving').textContent = `${dataOrDefault(timeMoving / 3600, 2)} h (${dataOrDefault(timeMoving, 0)} s)`;
    
    if ($('speed-avg-moving')) {
        const avgSpd = timeMoving > 0 ? (distM / timeMoving) * KMH_MS : 0;
        $('speed-avg-moving').textContent = dataOrDefault(avgSpd, 4, ' km/h');
    }

    // Affichage des cibles
    if (targetLat !== null && targetLon !== null && typeof turf !== 'undefined') {
        const from = [currentPosition.lon, currentPosition.lat];
        const to = [targetLon, targetLat];
        const distance = turf.distance(from, to, { units: 'meters' });
        const bearing = turf.bearing(from, to);

        if ($('target-distance')) $('target-distance').textContent = dataOrDefault(distance, 2, ' m');
        if ($('target-cap')) $('target-cap').textContent = dataOrDefault(bearing, 1, ' °');
    } else {
        if ($('target-distance')) $('target-distance').textContent = 'Non définie';
        if ($('target-cap')) $('target-cap').textContent = 'Non définie';
    }
}

/** 🔭 Met à jour l'affichage astronomique */
function updateAstro(lat, lon, date) {
    if (typeof SunCalc === 'undefined' || !lat || !lon) return;

    const times = SunCalc.getTimes(date, lat, lon);
    const sunPos = SunCalc.getPosition(date, lat, lon);
    const moonPos = SunCalc.getMoonPosition(date, lat, lon);
    const moonPhase = SunCalc.getMoonIllumination(date);

    // Soleil
    if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(sunPos.altitude * R2D, 2, '°');
    if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(sunPos.azimuth * R2D + 180, 2, '°');
    
    // Lever/Coucher
    if ($('sunrise-times')) $('sunrise-times').textContent = times.sunrise.toLocaleTimeString('fr-FR');
    if ($('sunset-times')) $('sunset-times').textContent = times.sunset.toLocaleTimeString('fr-FR');

    // Lune
    if ($('moon-alt')) $('moon-alt').textContent = dataOrDefault(moonPos.altitude * R2D, 2, '°');
    if ($('moon-azimuth')) $('moon-azimuth').textContent = dataOrDefault(moonPos.azimuth * R2D + 180, 2, '°');
    if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(moonPhase.fraction * 100, 1, ' %');
    if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(moonPhase.phase);
}

/** 🌙 Fonction utilitaire pour le nom de la phase lunaire */
function getMoonPhaseName(phase) {
    if (phase < 0.03 || phase >= 0.97) return "Nouvelle Lune";
    if (phase < 0.22) return "Premier Croissant";
    if (phase < 0.28) return "Premier Quartier";
    if (phase < 0.47) return "Lune Gibbeuse Croissante";
    if (phase < 0.53) return "Pleine Lune";
    if (phase < 0.72) return "Lune Gibbeuse Décroissante";
    if (phase < 0.78) return "Dernier Quartier";
    if (phase < 0.97) return "Dernier Croissant";
    return "N/A";
}

// --- LOGIQUE CARTE (Leaflet) ---

/** 🗺️ Initialisation de la carte Leaflet */
function initMap(lat, lon) {
    if (!map && typeof L !== 'undefined' && $('map')) {
        map = L.map('map').setView([lat, lon], 16);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: '© OpenStreetMap'
        }).addTo(map);

        marker = L.marker([lat, lon]).addTo(map);
        pathLine = L.polyline([], { color: '#007bff', weight: 3 }).addTo(map);

        // Événement pour définir une cible
        map.on('click', (e) => {
            lastMapClickLatLon = e.latlng;
            alert(`Point cliqué : Lat ${e.latlng.lat.toFixed(4)}, Lon ${e.latlng.lng.toFixed(4)}. Utilisez "Définir Cible" pour confirmer.`);
        });
    }
}

/** 🔄 Mise à jour de la carte */
function updateMap() {
    if (map && marker && pathLine) {
        const latlng = [currentPosition.lat, currentPosition.lon];
        marker.setLatLng(latlng);
        pathLine.addLatLng(latlng);
    }
}

// --- INITIALISATION DU SYSTÈME (Au chargement de la page) ---
document.addEventListener('DOMContentLoaded', () => {

    // Vérification des dépendances critiques
    if (typeof math === 'undefined' || typeof L === 'undefined' || typeof SunCalc === 'undefined' || typeof turf === 'undefined') {
        const missing = [
            (typeof math === 'undefined' ? "math.min.js" : ""),
            (typeof L === 'undefined' ? "leaflet.js" : ""),
            (typeof SunCalc === 'undefined' ? "suncalc.js" : ""),
            (typeof turf === 'undefined' ? "turf.min.js" : "")
        ].filter(Boolean).join(", ");
        console.error(`Erreur critique : Dépendances manquantes : ${missing}.`);
        alert(`Erreur: Dépendances manquantes : ${missing}. L'application ne peut pas démarrer.`);
        return;
    }
    
    // 1. Initialisation des composants fondamentaux
    ukf = new ProfessionalUKF(currentPosition.lat, currentPosition.lon, currentPosition.alt);
    initMap(currentPosition.lat, currentPosition.lon);
    
    // 2. Réglage des valeurs par défaut pour l'affichage (Offline-First)
    if ($('air-density')) $('air-density').textContent = `${RHO_SEA_LEVEL.toFixed(3)} kg/m³ (Défaut)`;
    if ($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = `${getSpeedOfSound(TEMP_SEA_LEVEL_K).toFixed(2)} m/s (Défaut)`;
    
    // 3. Démarrage de la synchro NTP et de la boucle lente
    syncH();
    startSlowLoop();
    
    // 4. Raccrochage des événements principaux
    if ($('start-btn')) $('start-btn').addEventListener('click', () => startGPS('HIGH_FREQ'));
    if ($('stop-btn')) $('stop-btn').addEventListener('click', () => stopGPS(true));
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', () => maxSpd = 0);
    
    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', () => {
        emergencyStopActive = !emergencyStopActive;
        if (emergencyStopActive) stopGPS(true);
        if ($('emergency-status')) $('emergency-status').textContent = emergencyStopActive ? 'ACTIF (Mode Sécurité)' : 'INACTIF';
    });
    
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => {
        if (confirm("Êtes-vous sûr de vouloir tout réinitialiser (UKF, Distance, Max) ?")) {
            stopGPS(true);
            ukf = new ProfessionalUKF(currentPosition.lat, currentPosition.lon, currentPosition.alt);
            distM = 0; maxSpd = 0; timeMoving = 0;
            if(pathLine) pathLine.setLatLngs([]); 
            targetLat = null; targetLon = null;
            alert("Réinitialisation complète effectuée.");
        }
    });

    if ($('set-target-btn')) $('set-target-btn').addEventListener('click', () => {
        if (lastMapClickLatLon) {
            targetLat = lastMapClickLatLon.lat;
            targetLon = lastMapClickLatLon.lng;
            updateDOMSlow(); // Met à jour l'affichage de la cible
            alert(`Cible définie à Lat: ${targetLat.toFixed(4)}, Lon: ${targetLon.toFixed(4)}.`);
        } else {
            alert('Veuillez cliquer sur la carte pour choisir une cible.');
        }
    });

});
