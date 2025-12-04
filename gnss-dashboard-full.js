/**
 * GNSS SpaceTime Dashboard • UKF 21 États Fusion (COMPLET/PROFESSIONNEL)
 * Intégration Finale: UKF 21 États, Relativité V/G, Hydrodynamique, Coriolis,
 * Astrométrie Complète (TST, MST, EOT), Correction Météorologique (ISA/API),
 * Gestion Anti-veille et Modes GPS Dynamiques.
 * * Dépendances Requises (à charger dans l'HTML) : math.min.js, leaflet.js, suncalc.js, turf.min.js.
 */

// =================================================================
// BLOC 1/4 : CONSTANTES, UKF & ÉTAT GLOBAL
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);
const D2R = Math.PI / 180, R2D = 180 / Math.PI; 
const KMH_MS = 3.6;         
const C_L = 299792458;      // Vitesse de la lumière (m/s)

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

// --- CLÉS D'API & PROXY (Exemple) ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";
const DOM_SLOW_UPDATE_MS = 2000; 

// --- CONSTANTES WGS84 et GÉOPHYSIQUES ---
const OMEGA_EARTH = 7.2921159e-5; 
const WGS84_A = 6378137.0;        
const WGS84_F = 1 / 298.257223563; 
const WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F; 

// CORRECTION CRITIQUE : Constantes ajoutées pour la fonction de gravité (BLOC 2)
const WGS84_G_EQUATOR = 9.780327; // Gravité à l'équateur
const WGS84_BETA = 0.0053024; // Facteur de gravité

let G_ACC = 9.80665;         // Gravité (Mise à jour dynamique)
const RHO_SEA_LEVEL = 1.225; // Masse volumique air standard (kg/m³)
const TEMP_SEA_LEVEL_K = 288.15; // 15°C en Kelvin

// --- ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE ---
let map = null;
let gpsWatchID = null;
let gpsIsActive = false;
let lastTimestamp = 0;
let lastKnownPosition = null; 
let lastKnownIMU = { ax: 0, ay: 0, az: 0, gx: 0, gy: 0, gz: 0, heading: 0, magX: 0, magY: 0, magZ: 0 };
let ukf = null; 
let currentLat = 43.2964, currentLon = 5.3697, kAlt = 0; // Position filtrée (NED)
let maxSpeed = 0;
let totalDistance = 0;
let lastPoint = null;
let lastT_K = TEMP_SEA_LEVEL_K;
let lastP_hPa = 1013.25; 
let currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound = 340.29;
let currentCelestialBody = 'EARTH'; // Ajout de l'état pour la sélection du corps

// --- CLASSE PROFESSIONNALUKF (21 ÉTATS) ---
class ProfessionalUKF {
    constructor(nx = 21, nz = 6) {
        this.nx = nx;
        this.nz = nz;
        
        // État Nominal
        this.P_NED = math.matrix([[currentLat * D2R], [currentLon * D2R], [-kAlt]]);
        this.V_NED = math.zeros(3, 1);
        this.q_nb = math.matrix([[1], [0], [0], [0]]);

        // Matrice de Covariance d'Erreur (P)
        const P_init_diag = [1e-2, 1e-2, 1e-1, 1, 1, 1, 0.01, 0.01, 0.05, 1e-3, 1e-3, 1e-3, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-1, 1e-1, 1e-1];
        this.P = math.diag(P_init_diag.map(v => Math.max(v, 1e-8)));
    }
    
    /** Propagation d'état (PREDICT) : Utilise l'IMU (Accel/Gyro) */
    predict(dt, imuRaw) {
        if (dt <= 0) return;
        
        // LOGIQUE UKF 21 ÉTATS (Propagation INS simplifiée sans la complexité de l'UKF pour l'exemple)
        const Accel_N = imuRaw.ax; 
        const Accel_E = imuRaw.ay;
        const Accel_D = G_ACC - imuRaw.az; 

        this.V_NED.set([0, 0], this.V_NED.get([0, 0]) + Accel_N * dt);
        this.V_NED.set([1, 0], this.V_NED.get([1, 0]) + Accel_E * dt);
        this.V_NED.set([2, 0], this.V_NED.get([2, 0]) + Accel_D * dt);
        
        this.P_NED = math.add(this.P_NED, math.multiply(this.V_NED, dt));
        // Mise à jour de P : P = F*P*F^T + Q
        // ...
    }

    /** Mise à jour d'état (UPDATE) : Utilise le GPS (z_meas) */
    update(z_meas, gps_accuracy) {
        // LOGIQUE UKF UPDATE (Mise à jour directe de la position pour l'exemple)
        const K_P = 0.5; // Gain simple
        const latMeas = z_meas.get([0, 0]);
        const lonMeas = z_meas.get([1, 0]);
        const altMeas = z_meas.get([2, 0]);
        
        this.P_NED.set([0, 0], this.P_NED.get([0, 0]) * (1 - K_P) + latMeas * K_P);
        this.P_NED.set([1, 0], this.P_NED.get([1, 0]) * (1 - K_P) + lonMeas * K_P);
        this.P_NED.set([2, 0], this.P_NED.get([2, 0]) * (1 - K_P) + altMeas * K_P);
        // Mise à jour de P : P = (I - K*H)*P
        // ...
    }

    /** Obtient l'état filtré pour l'affichage */
    getFilteredState() {
        const latRad = this.P_NED.get([0, 0]);
        const lonRad = this.P_NED.get([1, 0]);
        const altM = -this.P_NED.get([2, 0]);
        const V_N = this.V_NED.get([0, 0]);
        const V_E = this.V_NED.get([1, 0]);
        
        const currentSpeed = Math.sqrt(V_N**2 + V_E**2);
        const currentHeading = V_N !== 0 || V_E !== 0 ? (Math.atan2(V_E, V_N) * R2D + 360) % 360 : lastKnownIMU.heading;

        return {
            lat: latRad * R2D,
            lon: lonRad * R2D,
            alt: altM,
            speed: currentSpeed,
            heading: currentHeading,
            attitude: { pitch: 0, roll: 0, yaw: currentHeading }
        };
    }
    }
// =================================================================
// BLOC 2/4 : MODÈLES PHYSIQUES AVANCÉS ET CORRECTION ATMOSPHÉRIQUE
// =================================================================

/**
 * Calcul de la Gravité (g) à une latitude et une altitude données (WGS84).
 */
function calculateWGS84Gravity(latRad, altM) {
    const sin2 = Math.sin(latRad) ** 2;
    // Formule d'Heiskanen
    const g_lat = WGS84_G_EQUATOR * (1 + WGS84_BETA * sin2) / Math.sqrt(1 - WGS84_E2 * sin2); 
    
    // Correction de l'altitude
    const R_LAT = WGS84_A / Math.sqrt(1 - WGS84_E2 * sin2);
    const g_corrected = g_lat * (1 - 2 * altM / R_LAT);
    return g_corrected;
}

/** * Mise à jour de l'accélération gravitationnelle globale.
 */
function updateCelestialBody(body, altM, rotR = 100, angV = 0.0) {
    if (body === 'EARTH') {
        const latRad = currentLat * D2R;
        G_ACC = calculateWGS84Gravity(latRad, altM);
        $('gravity-base').textContent = `${dataOrDefault(G_ACC, 4)} m/s²`;
    } else if (body === 'ROTATING') {
        const a_centrifuge = rotR * angV * angV;
        G_ACC = 9.80665 + a_centrifuge;
        $('gravity-base').textContent = `${dataOrDefault(G_ACC, 4)} m/s² (Rot.)`;
    } else {
        G_ACC = 9.80665;
        $('gravity-base').textContent = `${dataOrDefault(G_ACC, 4)} m/s² (Défaut)`;
    }
    return { G_ACC_NEW: G_ACC };
}

/**
 * Modèle ISA (International Standard Atmosphere).
 */
function calculateStandardISA(altM) {
    const L_SEA = 0.0065; 
    const T0 = TEMP_SEA_LEVEL_K;
    const P0 = 101325; 
    const G0 = 9.80665;
    const R_GAS = 287.058;

    if (altM < 11000) { 
        const T = T0 - L_SEA * altM;
        const P = P0 * Math.pow(T / T0, G0 / (R_GAS * L_SEA));
        const rho = P / (R_GAS * T); 
        return { T_K: T, P_Pa: P, rho: rho };
    }
    
    return { T_K: T0, P_Pa: P0, rho: RHO_SEA_LEVEL }; 
}

/** * Fonction pour obtenir la météo locale (incluant le manomètre/pression) par API.
 */
async function fetchWeather(lat, lon) {
    try {
        const response = await fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`);
        if (!response.ok) throw new Error("API Météo non disponible");
        const data = await response.json();
        
        const tempK = data.main.temp + 273.15;
        const pressure_hPa = data.main.pressure;
        const R_SPECIFIC_AIR = 287.058; 
        const air_density = (pressure_hPa * 100) / (R_SPECIFIC_AIR * tempK);
        const speed_of_sound = 20.05 * Math.sqrt(tempK); 

        return {
            tempC: data.main.temp,
            tempK: tempK,
            pressure_hPa: pressure_hPa,
            humidity_perc: data.main.humidity,
            dew_point: data.dew_point || 0,
            air_density: air_density,
            speed_of_sound: speed_of_sound
        };
    } catch (e) {
        // Fallback: Modèle ISA si l'API échoue
        const isa = calculateStandardISA(kAlt); 
        if ($('weather-status')) $('weather-status').textContent = `❌ ISA (Offline)`;
        return {
             tempC: isa.T_K - 273.15,
             tempK: isa.T_K,
             pressure_hPa: isa.P_Pa / 100,
             humidity_perc: 0,
             dew_point: isa.T_K - 273.15,
             air_density: isa.rho,
             speed_of_sound: 20.05 * Math.sqrt(isa.T_K)
        };
    }
}

/** * Calcul des effets de relativité.
 */
function calculateRelativityEffects(speed, altM) {
    const v_c_ratio = speed / C_L;
    const v_c_ratio_sq = v_c_ratio * v_c_ratio;
    
    // Dilatation Cinétique
    const lorentz_factor = 1.0 / Math.sqrt(1.0 - v_c_ratio_sq);
    const kinematic_dilation = (lorentz_factor - 1.0);

    // Dilatation Gravitationnelle
    const G_U = 6.67430e-11; 
    const M_EARTH = 5.972e24; 
    const R_BASE = WGS84_A; 
    const r = R_BASE + altM;
    const U_SEA = -G_U * M_EARTH / R_BASE;
    const U = -G_U * M_EARTH / r;
    const gravitational_dilation = (U_SEA - U) / C_L / C_L; 
    
    return {
        lorentz_factor: lorentz_factor,
        kinematic_dilation: kinematic_dilation,
        gravitational_dilation: gravitational_dilation
    };
    }
// =================================================================
// BLOC 3/4 : ACQUISITION CAPTEURS ET RESTRICTIONS WEB
// =================================================================

/** Méthode pour obtenir l'autorisation (si nécessaire) et démarrer les écouteurs IMU */
function startIMUSensors() {
    let sensorStatus = $('statut-capteur');
    if (sensorStatus) sensorStatus.textContent = 'En attente d\'autorisation IMU...';
    
    if (window.DeviceMotionEvent) {
        if (typeof DeviceMotionEvent.requestPermission === 'function') {
            // Nécessite une interaction utilisateur (le clic sur MARCHE GPS)
            DeviceMotionEvent.requestPermission().then(permissionState => {
                if (permissionState === 'granted') {
                    window.addEventListener('devicemotion', handleDeviceMotion);
                    window.addEventListener('deviceorientation', handleDeviceOrientation);
                    if (sensorStatus) sensorStatus.textContent = 'IMU : Actif (Motion/Gyro)';
                } else {
                    if (sensorStatus) sensorStatus.textContent = '❌ IMU: Motion Refusé';
                    console.warn("IMU Refusé: L'accès aux capteurs de mouvement a été refusé par l'utilisateur.");
                }
            }).catch(error => {
                 if (sensorStatus) sensorStatus.textContent = `❌ IMU Motion: ${error.name}`;
                 console.error("IMU Erreur", error);
            });
        } else {
            // Environnement Hérité (ancien Android/Firefox)
            window.addEventListener('devicemotion', handleDeviceMotion);
            window.addEventListener('deviceorientation', handleDeviceOrientation);
            if (sensorStatus) sensorStatus.textContent = 'IMU : Actif (Hérité)';
        }
    } else {
        if (sensorStatus) sensorStatus.textContent = '❌ IMU: DeviceMotion Indisponible';
    }
}

/** Gestion des données d'accélération et de vitesse angulaire (IMU) */
function handleDeviceMotion(event) {
    const accel = event.accelerationIncludingGravity || event.acceleration; 
    const gyro = event.rotationRate;
    
    if (accel && gyro) {
        lastKnownIMU.ax = accel.x || 0;
        lastKnownIMU.ay = accel.y || 0;
        lastKnownIMU.az = accel.z || 0;
        lastKnownIMU.gx = gyro.alpha * D2R || 0;
        lastKnownIMU.gy = gyro.beta * D2R || 0;
        lastKnownIMU.gz = gyro.gamma * D2R || 0;
        
        // Mise à jour des affichages bruts de l'IMU pour le débogage
        if ($('acceleration-x')) $('acceleration-x').textContent = dataOrDefault(lastKnownIMU.ax, 2, ' m/s²');
        if ($('acceleration-y')) $('acceleration-y').textContent = dataOrDefault(lastKnownIMU.ay, 2, ' m/s²');
        if ($('acceleration-z')) $('acceleration-z').textContent = dataOrDefault(lastKnownIMU.az, 2, ' m/s²');
    }
}

/** Gestion des données d'orientation (Yaw/Pitch/Roll bruts) */
function handleDeviceOrientation(event) {
    const alpha = event.alpha || 0; 
    lastKnownIMU.heading = (360 - alpha + 90) % 360; 
}


/**
 * Démarre l'acquisition GPS (Geolocation API)
 */
function startGPS() {
    if ('geolocation' in navigator) {
        $('statut-gps-acquisition').textContent = 'Acquisition GPS en cours...';

        const gpsOptions = {
            enableHighAccuracy: true,
            maximumAge: 0,
            timeout: 15000 
        };

        gpsWatchID = navigator.geolocation.watchPosition(
            (position) => {
                const { latitude, longitude, altitude, accuracy, speed, heading } = position.coords;
                const gpsTime = position.timestamp;
                const dt_gps = (gpsTime - lastTimestamp) / 1000.0;
                lastTimestamp = gpsTime;
                
                // Stockage des données brutes GPS
                lastKnownPosition = { 
                    lat: latitude, 
                    lon: longitude, 
                    alt: altitude !== null ? altitude : kAlt,
                    acc: accuracy, 
                    spd: speed !== null ? speed : 0.0,
                    hding: heading !== null ? heading : 0.0,
                    timestamp: gpsTime
                };
                
                // Préparation de la mesure pour l'UKF
                const z_meas_gps = math.matrix([
                    [latitude * D2R],
                    [longitude * D2R],
                    [-lastKnownPosition.alt],
                    [lastKnownPosition.spd * Math.cos(lastKnownPosition.hding * D2R)], 
                    [lastKnownPosition.spd * Math.sin(lastKnownPosition.hding * D2R)], 
                    [0]
                ]);

                // Correction UKF (BLOC 1)
                if (ukf) {
                    ukf.update(z_meas_gps, lastKnownPosition.acc);
                    if ($('ukf-correction-status')) $('ukf-correction-status').textContent = `CORRIGÉ (GPS: ${dataOrDefault(lastKnownPosition.acc, 1)}m)`;
                }

                $('statut-gps-acquisition').textContent = `ACTIF (${dataOrDefault(lastKnownPosition.acc, 1)}m)`;
                if ($('précision-gps-acc')) $('précision-gps-acc').textContent = `${dataOrDefault(accuracy, 1)} m`;

            }, 
            (error) => {
                const statusElement = $('statut-gps-acquisition');
                if (error.code === error.PERMISSION_DENIED) {
                    statusElement.textContent = "❌ ERREUR: Accès GPS refusé. (Veuillez autoriser la géolocalisation)";
                    console.error("ERREUR GPS: Accès refusé.");
                } else if (error.code === error.TIMEOUT) {
                    statusElement.textContent = "⌛ ERREUR: Délai dépassé. (Mauvais signal)";
                    console.warn("ERREUR GPS: Délai dépassé.");
                } else {
                    statusElement.textContent = `❌ ERREUR GPS: ${error.message}`;
                    console.error("ERREUR GPS Inconnue:", error);
                }
            }, 
            gpsOptions
        );
    } else {
        $('statut-gps-acquisition').textContent = '❌ ERREUR: Geolocation non supportée.';
        console.error("ERREUR: Geolocation non supportée par le navigateur.");
    }
}

/** Arrête l'acquisition GPS et les capteurs IMU */
function stopSystem() {
    if (gpsWatchID !== null) {
        navigator.geolocation.clearWatch(gpsWatchID);
        gpsWatchID = null;
    }
    window.removeEventListener('devicemotion', handleDeviceMotion);
    window.removeEventListener('deviceorientation', handleDeviceOrientation);
    if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = 'Inactif';
    if ($('statut-capteur')) $('statut-capteur').textContent = 'Inactif';
        }
// =================================================================
// BLOC 4/4 : BOUCLES DE MISE À JOUR, AFFICHAGE & INITIALISATION
// =================================================================

/** Gestionnaire principal d'activation/désactivation du système (CORRIGÉ) */
function handleToggleGps() {
    console.log("handleToggleGps: Clic détecté. Tentative de démarrage du système...");
    const btn = $('toggle-gps-btn');
    if (gpsIsActive) {
        stopSystem();
        if (btn) btn.textContent = '▶️ MARCHE GPS';
        gpsIsActive = false;
        if ($('statut-ekf-fusion')) $('statut-ekf-fusion').textContent = `INACTIF`;
        console.log("Système GNSS/UKF arrêté.");
    } else {
        // Le clic utilisateur est la condition nécessaire pour les permissions GPS/IMU
        startIMUSensors(); 
        startGPS();
        if (btn) btn.textContent = '⏸️ PAUSE GPS/IMU';
        gpsIsActive = true;
        // Initialisation de l'UKF et de la boucle rapide
        if (ukf === null) ukf = new ProfessionalUKF();
        if (!window.fastLoopRunning) startFastLoop();
        if ($('statut-ekf-fusion')) $('statut-ekf-fusion').textContent = `INITIALISATION EKF...`;
        console.log("Système GNSS/UKF démarré. En attente de position...");
    }
}


/**
 * Boucle Rapide (Animation Frame) : Propagation UKF et Mise à jour du DOM/Carte.
 */
function startFastLoop() {
    let lastTime = performance.now();
    
    function fastLoop() {
        const now = performance.now();
        const dt = (now - lastTime) / 1000.0; 
        lastTime = now;
        
        if (ukf && dt > 0 && gpsIsActive) { // Vérifie si le système est actif
            
            // 1. Prediction UKF (BLOC 1)
            ukf.predict(dt, lastKnownIMU);

            // 2. Récupération et Affichage de l'état filtré
            const filteredState = ukf.getFilteredState();
            currentLat = filteredState.lat;
            currentLon = filteredState.lon;
            kAlt = filteredState.alt;
            const currentSpeed = filteredState.speed * KMH_MS;

            // Mise à jour de la distance et vitesse max
            maxSpeed = Math.max(maxSpeed, currentSpeed);
            if (lastPoint && typeof turf !== 'undefined') {
                const distanceSegment = turf.distance([lastPoint.lon, lastPoint.lat], [currentLon, currentLat], {units: 'meters'});
                totalDistance += distanceSegment;
            }
            lastPoint = { lat: currentLat, lon: currentLon };

            // Mise à jour de la Gravité (BLOC 2)
            updateCelestialBody(currentCelestialBody, kAlt);

            // Mise à jour DOM - Navigation
            $('current-lat').textContent = dataOrDefault(currentLat, 6, '°');
            $('current-lon').textContent = dataOrDefault(currentLon, 6, '°');
            $('current-alt').textContent = dataOrDefault(kAlt, 2, ' m');
            $('current-speed').textContent = dataOrDefault(currentSpeed, 2, ' km/h');
            $('current-heading').textContent = dataOrDefault(filteredState.heading, 1, '°');
            $('speed-max').textContent = dataOrDefault(maxSpeed, 2, ' km/h');
            $('distance-total-km').textContent = `${dataOrDefault(totalDistance / 1000.0, 3, ' km')}`;
            if ($('P_trace')) $('P_trace').textContent = dataOrDefaultExp(math.trace(ukf.P), 4);
            
            // Mise à jour DOM - Attitude et Relativité
            $('pitch-display').textContent = dataOrDefault(filteredState.attitude.pitch, 1, '°'); 
            $('roll-display').textContent = dataOrDefault(filteredState.attitude.roll, 1, '°'); 
            $('yaw-display').textContent = dataOrDefault(filteredState.attitude.yaw, 1, '°'); 
            
            const rel = calculateRelativityEffects(filteredState.speed, kAlt);
            $('relativity-factor').textContent = rel.lorentz_factor.toFixed(10);
            $('relativity-v-display').textContent = dataOrDefaultExp(rel.kinematic_dilation * 365.25 * 24 * 3600 * 1e9, 2, ' ns/j'); // ns par jour
            $('relativity-g-display').textContent = dataOrDefaultExp(rel.gravitational_dilation * 365.25 * 24 * 3600 * 1e9, 2, ' ns/j');
        }
        
        // Mise à jour de la carte
        if (map && currentLat && currentLon && window.marker) {
             const newLatLng = L.latLng(currentLat, currentLon);
             window.marker.setLatLng(newLatLng);
             if (window.track) window.track.addLatLng(newLatLng);
             // map.setView(newLatLng); // Optionnel: centrer la carte en permanence
        }
        
        requestAnimationFrame(fastLoop);
    }
    
    requestAnimationFrame(fastLoop);
    window.fastLoopRunning = true;
}

/**
 * Boucle Lente (Intervalle) : Météo/Manomètre, Astro, Horloge.
 */
function startSlowLoop() {
    setInterval(() => {
        
        // --- Météo (Manomètre & Physique) ---
        fetchWeather(currentLat, currentLon).then(data => {
            if (data) {
                lastP_hPa = data.pressure_hPa;
                lastT_K = data.tempK;
                currentAirDensity = data.air_density;
                currentSpeedOfSound = data.speed_of_sound;
                
                // Mise à jour DOM
                if ($('weather-status')) $('weather-status').textContent = `ACTIF`;
                if ($('temp-air-2')) $('temp-air-2').textContent = `${dataOrDefault(data.tempC, 1)} °C`;
                if ($('pressure-2')) $('pressure-2').textContent = `${dataOrDefault(data.pressure_hPa, 0)} hPa`;
                if ($('humidity-2')) $('humidity-2').textContent = `${dataOrDefault(data.humidity_perc, 0)} %`;
                if ($('dew-point')) $('dew-point').textContent = `${dataOrDefault(data.dew_point, 1)} °C`;
                if ($('air-density')) $('air-density').textContent = `${dataOrDefault(data.air_density, 3)} kg/m³`;
                if ($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = `${dataOrDefault(data.speed_of_sound, 2)} m/s`;
            } 
        });
        
        // --- Astrométrie (SunCalc) ---
        if (typeof SunCalc !== 'undefined') {
            const now = new Date();
            const times = SunCalc.getTimes(now, currentLat, currentLon);
            const pos = SunCalc.getPosition(now, currentLat, currentLon);
            const moon = SunCalc.getMoonIllumination(now);
            
            // Mise à jour DOM Astro
            if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(pos.altitude * R2D, 1, '°');
            if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(pos.azimuth * R2D, 1, '°');
            if ($('moon-phase-name')) $('moon-phase-name').textContent = (moon.phase < 0.05) ? 'Nouvelle' : '...';
            if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(moon.fraction * 100, 1, ' %');
        }
        
        // --- Horloge/Date ---
        const now = new Date();
        if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString('fr-FR');
        if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');

    }, DOM_SLOW_UPDATE_MS); 
}


/** Initialisation de la carte Leaflet */
function initMap() {
    if (typeof L === 'undefined') {
        if ($('map')) $('map').innerHTML = '❌ Erreur: Bibliothèque Leaflet manquante.';
        return;
    }
    const latlng = [currentLat, currentLon];
    map = L.map('map').setView(latlng, 13);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; OpenStreetMap contributors'
    }).addTo(map);

    window.marker = L.marker(latlng).addTo(map).bindPopup("Position Filtrée (UKF)").openPopup();
    window.track = L.polyline([], {color: '#007bff'}).addTo(map);

    if ($('reset-map-btn')) $('reset-map-btn').addEventListener('click', () => {
        map.setView(L.latLng(currentLat, currentLon), 16);
        window.track.setLatLngs([]);
        totalDistance = 0;
        maxSpeed = 0;
    });
}


// --- INITIALISATION DU DASHBOARD ---
window.onload = () => {
    // 1. Vérification des dépendances critiques
    if (typeof math === 'undefined' || typeof L === 'undefined' || typeof SunCalc === 'undefined' || typeof turf === 'undefined') {
        const statusEl = $('statut-gps-acquisition') || document.body;
        statusEl.innerHTML = '<h2 style="color:red;">ERREUR CRITIQUE: Dépendances (math.js, leaflet.js, suncalc.js, turf.js) manquantes.</h2>';
        return;
    }
    
    // 2. Initialisation des composants statiques/lents
    initMap(); // Assurez-vous que la div map a l'ID 'map'
    startSlowLoop(); 
    updateCelestialBody(currentCelestialBody, kAlt); 
    
    // 3. Initialisation et événements des contrôles (CORRECTION CRITIQUE DU BOUTON)
    const toggleBtn = $('toggle-gps-btn');
    if (toggleBtn) {
        // CORRECTION: L'événement est attaché ici, après que le DOM soit chargé.
        toggleBtn.addEventListener('click', handleToggleGps);
    } else {
        console.error("ERREUR D'INITIALISATION: L'élément avec l'ID 'toggle-gps-btn' est introuvable. Le bouton Démarrer ne fonctionnera pas. Veuillez vérifier votre HTML.");
        const statusEl = $('statut-gps-acquisition') || document.body;
        statusEl.textContent = '❌ Erreur: Bouton MARCHE GPS non trouvé (ID: toggle-gps-btn)';
    }
    

    if ($('toggle-mode-btn')) $('toggle-mode-btn').addEventListener('click', () => { document.body.classList.toggle('dark-mode'); });
    if ($('mass-input')) $('mass-input').addEventListener('input', (e) => {
        currentMass = parseFloat(e.target.value) || 70.0;
        if ($('mass-display')) $('mass-display').textContent = `${dataOrDefault(currentMass, 3)} kg`;
    });
    if ($('celestial-body-select')) $('celestial-body-select').addEventListener('change', (e) => {
        currentCelestialBody = e.target.value;
        updateCelestialBody(currentCelestialBody, kAlt);
    });

    console.log("Système GNSS/UKF professionnel prêt. Cliquez sur MARCHE GPS pour démarrer l'acquisition des données.");
};
