// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
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
        return '0.00e+0' + suffix;
    }
    return val.toExponential(decimals) + suffix;
};

// =================================================================
// BLOC 1/4 : Constantes Globales et Configuration (MISE À JOUR)
// =================================================================

// --- CONSTANTES MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI; 
const KMH_MS = 3.6;         
const C_L = 299792458;      

// --- CONSTANTES GÉOPHYSIQUES (WGS84) ---
let G_ACC = 9.80665;         
let R_ALT_CENTER_REF = 6371000;
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const R_AIR = 287.058;      // Constante spécifique de l'air sec (J/kg·K)
const KAPPA = 1.4;          // Ratio des chaleurs spécifiques pour l'air

// --- ÉTATS GLOBAUX ---
let wID = null; // GPS Watch ID
let domSlowID = null; // ID du timer lent (Astro/Météo)
let domFastID = null; // ID du timer rapide (UKF)

let sTime = 0; // Temps de démarrage du GPS
let lServH = 0; // Décalage NTP (Server - Local)
let lLocH = 0; // Décalage NTP (Local - GMT)

let emergencyStopActive = false;
let isMoving = false;
let isZUPT = true;

// --- UKF/FILTRE ---
let ukf = null;
let lastUpdateTimestamp = 0;
let distanceTraveled = 0;
let timeMoving = 0;
let vMax = 0;
let lastP_hPa = 1013.25; // Pression atmosphérique de référence (hPa)
let lastT_K = 288.15; // Température de référence (K, ~15°C)
let currentAirDensity = 1.225; // Densité de l'air de référence (kg/m³)
let currentSpeedOfSound = 340.29; // Vitesse du son de référence (m/s)

// --- VITESSE/DISTANCE ---
const MIN_SPD = 0.05; // m/s
let speedInst = 0;
let lat = 0, lon = 0, alt = 0;
let speedEKF = 0;

// --- ASTRO/MÉTÉO ---
let currentCelestialBody = 'sun';
let kAlt = 0; // Altitude du soleil
let rotationRadius = 0;
let angularVelocity = 0;

// --- IMU (Generic Sensor API) ---
let accSensor = null;
let gyroSensor = null;
let imuData = { aX: 0, aY: 0, aZ: 0, wX: 0, wY: 0, wZ: 0 };
let imuError = null;
let lastAccelMag = 0; // Pour le ZUPT

// --- MAP ---
let map = null;
let marker = null;
let polyline = null;
const MAP_ZOOM_LEVEL = 17;

// --- CONFIG UKF REACTIVITY ---
let currentUKFReactivity = 'AUTO';
const UKF_REACTIVITY_FACTORS = {
    'AUTO': { MULT: 1.0, DISPLAY: 'Automatique' },
    'NORMAL': { MULT: 0.5, DISPLAY: 'Normal' },
    'FAST': { MULT: 2.0, DISPLAY: 'Rapide' },
    'STABLE': { MULT: 0.1, DISPLAY: 'Microscopique' }
};

// =================================================================
// BLOC 2/4 : Fonctions physiques, chronométriques et utilitaires
// =================================================================

// --- FONCTIONS TEMPS/CHRONOMÉTRIE ---
function getCDate(lServH, lLocH) {
    if (lServH === 0 && lLocH === 0) {
        return new Date();
    }
    const localNow = Date.now();
    const serverTimeUTC = localNow + lServH + lLocH;
    return new Date(serverTimeUTC);
}

function formatDuration(seconds) {
    const hours = Math.floor(seconds / 3600);
    const minutes = Math.floor((seconds % 3600) / 60);
    const secs = Math.floor(seconds % 60);
    return `${String(hours).padStart(2, '0')}:${String(minutes).padStart(2, '0')}:${String(secs).padStart(2, '0')}`;
}

async function syncH(lServH, lLocH) {
    let newTimes = { lServH: lServH, lLocH: lLocH };
    
    // Tente la synchro NTP (simulée ici par une API)
    try {
        const response = await fetch('https://worldtimeapi.org/api/ip');
        if (!response.ok) throw new Error('API non disponible');
        const data = await response.json();
        
        const localTime = Date.now();
        const serverTime = new Date(data.utc_datetime).getTime();
        
        newTimes.lServH = serverTime - localTime; // Décalage serveur - local
        newTimes.lLocH = data.utc_offset.includes('+') ? 
                         parseInt(data.utc_offset.split('+')[1]) * 3600 * 1000 :
                         -parseInt(data.utc_offset.split('-')[1]) * 3600 * 1000; // Décalage GMT - Local
        
        if ($('local-time')) $('local-time').textContent = getCDate(newTimes.lServH, newTimes.lLocH).toLocaleTimeString('fr-FR');
        if ($('date-display')) $('date-display').textContent = getCDate(newTimes.lServH, newTimes.lLocH).toLocaleDateString('fr-FR');
        
        console.log(`[NTP] Synchro réussie. Décalage: ${newTimes.lServH} ms`);

    } catch (error) {
        console.error(`[NTP] Échec de la synchro : ${error.message}. Utilisation de l'horloge locale.`);
        if ($('local-time')) $('local-time').textContent = '⚠️ SYNCHRO ÉCHOUÉE';
    }
    return newTimes;
}

// --- FONCTIONS PHYSIQUES/MÉTÉO ---
async function fetchWeather(lat, lon) {
    // API de météo très simplifiée pour l'exemple
    return {
        tempC: 15.0,
        tempK: 288.15,
        pressure_hPa: 1013.25,
        humidity_perc: 70,
        dew_point: 9.5,
        air_density: 1.225
    };
}

function getAirDensity(P_hPa, T_K) {
    const P_Pa = P_hPa * 100; // Pression en Pascals
    return P_Pa / (R_AIR * T_K);
}

function getSpeedOfSound(T_K) {
    return Math.sqrt(KAPPA * R_AIR * T_K);
}

// --- FONCTIONS ASTRO ---
function updateAstro(lat, lon, lServH, lLocH) {
    if (typeof SunCalc === 'undefined') return;

    const now = getCDate(lServH, lLocH);
    const times = SunCalc.getTimes(now, lat, lon);
    const posSun = SunCalc.getPosition(now, lat, lon);
    const posMoon = SunCalc.getMoonPosition(now, lat, lon);
    const moonPhase = SunCalc.getMoonIllumination(now);

    // Temps Solaire Vrai (TST) et Moyen (MST)
    const TST_ms = now.getTime() - times.solarNoon.getTime();
    const TST_sec = (TST_ms / 1000) + (12 * 3600); // TST est l'heure après minuit
    const TST_h = TST_sec / 3600;
    
    // Équation du Temps (EOT)
    const T_solar_deg = posSun.azimuth * R2D + 180;
    const EOT_sec = (T_solar_deg / 360 * 86400) - TST_sec;
    const EOT_min = EOT_sec / 60;
    
    // Calcul de la Durée du Jour
    const dayDurationMS = times.sunset.getTime() - times.sunrise.getTime();
    const dayDurationSec = dayDurationMS / 1000;

    // Affichage
    if ($('date-display-astro')) $('date-display-astro').textContent = now.toLocaleDateString('fr-FR');
    if ($('noon-solar')) $('noon-solar').textContent = times.solarNoon.toLocaleTimeString('fr-FR', {hour12: false});
    if ($('day-duration')) $('day-duration').textContent = formatDuration(dayDurationSec);
    
    // TST/MST
    if ($('tst')) $('tst').textContent = new Date(TST_h * 3600 * 1000).toISOString().substr(11, 8);
    if ($('mst')) $('mst').textContent = now.toLocaleTimeString('fr-FR', {hour12: false});
    if ($('eot')) $('eot').textContent = `${EOT_min.toFixed(2)} min`;
    
    // Soleil
    if ($('sun-alt')) $('sun-alt').textContent = `${(posSun.altitude * R2D).toFixed(2)} °`;
    if ($('sun-azimuth')) $('sun-azimuth').textContent = `${(posSun.azimuth * R2D).toFixed(2)} °`;
    if ($('ecl-long')) $('ecl-long').textContent = `${posSun.eclipticalLongitude.toFixed(2)} rad`;

    // Lune
    if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(moonPhase.phase);
    if ($('moon-illuminated')) $('moon-illuminated').textContent = `${(moonPhase.fraction * 100).toFixed(1)} %`;
    if ($('moon-alt')) $('moon-alt').textContent = `${(posMoon.altitude * R2D).toFixed(2)} °`;
    if ($('moon-azimuth')) $('moon-azimuth').textContent = `${(posMoon.azimuth * R2D).toFixed(2)} °`;
    if ($('moon-times')) $('moon-times').textContent = `${times.moonrise ? times.moonrise.toLocaleTimeString('fr-FR', {hour12: false}) : 'N/A'} / ${times.moonset ? times.moonset.toLocaleTimeString('fr-FR', {hour12: false}) : 'N/A'}`;
    
    // Temps Minecraft (MJ)
    const midnightJST = new Date(now.getFullYear(), now.getMonth(), now.getDate(), 0, 0, 0, 0).getTime();
    const diff = now.getTime() - midnightJST;
    const dayRatio = diff / (24 * 3600 * 1000); // Ratio du jour écoulé
    const minecraftTimeTicks = dayRatio * 24000;
    const minecraftHours = Math.floor((minecraftTimeTicks + 6000) / 1000) % 24;
    const minecraftMinutes = Math.floor(((minecraftTimeTicks + 6000) % 1000) * 0.06);

    if ($('time-minecraft')) $('time-minecraft').textContent = `${String(minecraftHours).padStart(2, '0')}:${String(minecraftMinutes).padStart(2, '0')}`;
}

function getMoonPhaseName(phase) {
    if (phase < 0.03 || phase > 0.97) return '🌑 Nouvelle Lune';
    if (phase < 0.22) return '🌒 Croissant (croissant)';
    if (phase < 0.28) return '🌓 Premier Quartier';
    if (phase < 0.47) return '🌔 Gibbeuse (croissante)';
    if (phase < 0.53) return '🌕 Pleine Lune';
    if (phase < 0.72) return '🌖 Gibbeuse (décroissante)';
    if (phase < 0.78) return '🌗 Dernier Quartier';
    return '🌘 Croissant (décroissant)';
}


// =================================================================
// BLOC 3/4 : UKF 21 États (ProfessionalUKF) (Nécessite math.js)
// =================================================================

/**
 * Classe UKF simplifiée pour la fusion 21 états
 * (Position, Vitesse, Attitude, Décalages Accel/Gyro, Gravité, etc.)
 */
class ProfessionalUKF {
    constructor() {
        if (typeof math === 'undefined') {
            console.error("Erreur: math.js est requis pour le ProfessionalUKF.");
            return;
        }

        // UKF État 21x1 : [p, v, q, ba, bg, G] (p=3, v=3, q=4, ba=3, bg=3, G=3)
        // L'état complet inclurait les dérives des capteurs, l'erreur d'échelle, etc.
        const STATE_SIZE = 21; 
        this.X = math.matrix(math.zeros(STATE_SIZE, 1));
        this.P = math.multiply(math.eye(STATE_SIZE), 1e-4); // Matrice de covariance

        // Bruits (A ajuster finement)
        this.Q = math.multiply(math.eye(STATE_SIZE), 1e-6); // Bruit de processus (Q)
        this.R_IMU = math.multiply(math.eye(6), 1e-3); // Bruit de mesure IMU (A/G)
        this.R_GPS = math.multiply(math.eye(3), 1.0); // Bruit de mesure GPS (p)
        this.R_ZUPT = math.multiply(math.eye(3), 1e-6); // Bruit ZUPT (v=0)
        
        // Initialisation de l'état
        this.X.set([10, 0], G_ACC); // Composante Z de la gravité
        this.X.set([11, 0], -OMEGA_EARTH); // Composante Z de la vitesse de rotation (pour l'exemple)

        this.lastTime = Date.now();
        this.vEKF = [0, 0, 0];
    }
    
    // Simulation simple des matrices F (Transition) et H (Observation)
    // En réalité, F est non-linéaire et H dépend du type de mesure.

    predict(dt, acc, gyro) {
        if (!this.X) return;

        // Étape de prédiction : X(k|k-1) = f(X(k-1|k-1), u(k-1))
        // P(k|k-1) = F * P(k-1|k-1) * F^T + Q
        
        // Simuler la nouvelle vitesse (simplifié à l'extrême pour le DOM)
        const accelMag = math.norm([acc.aX, acc.aY, acc.aZ]);
        const speedUpdate = accelMag * dt * (1/KMH_MS); // Très simpliste
        this.vEKF[0] += speedUpdate;
        
        // Simuler la croissance de l'incertitude (simplifié pour le DOM)
        const P_new = this.P.get([0, 0]) + 0.001 * dt;
        this.P.set([0, 0], P_new);
        
        // Mettre à jour le temps de la dernière prédiction
        this.lastTime = Date.now();
    }

    update(measurementType, Z, R) {
        if (!this.X) return;
        
        // Étape de mise à jour:
        // K = P * H^T * (H * P * H^T + R)^-1  (Gain de Kalman)
        // X(k|k) = X(k|k-1) + K * (Z - h(X(k|k-1)))
        // P(k|k) = (I - K * H) * P(k|k-1)
        
        // Simuler la correction de la vitesse (très simplifié)
        if (measurementType === 'GPS_POS' || measurementType === 'GPS_VEL') {
             this.vEKF[0] = this.vEKF[0] * 0.9 + Z[0] * 0.1; 
        } else if (measurementType === 'ZUPT') {
             this.vEKF[0] = this.vEKF[0] * 0.8; // Réinitialiser la vitesse si ZUPT
        }
        
        // Simuler la réduction de l'incertitude (simplifié pour le DOM)
        const P_new = this.P.get([0, 0]) * 0.8;
        this.P.set([0, 0], P_new);
    }
    
    // Obtient la vitesse stable EKF (pour le DOM)
    getStableSpeed() {
        return this.vEKF[0];
    }

    // Obtient l'incertitude vitesse (pour le DOM)
    getSpeedIncertitude() {
        return this.P.get([0, 0]);
    }
    
    // Obtient l'incertitude altitude (pour le DOM)
    getAltIncertitude() {
        return Math.sqrt(this.P.get([2, 2])) * 10; // Composante Z de la position
    }
}


// =================================================================
// BLOC 4/4 : Logique GPS/IMU, Cartographie et Affichage DOM
// =================================================================

// --- LOGIQUE GPS ---
function startGPS() {
    if (wID !== null) return;
    
    const options = {
        enableHighAccuracy: currentGPSMode === 'HIGH_FREQ',
        timeout: 5000,
        maximumAge: 0
    };

    wID = navigator.geolocation.watchPosition(updateGPS, handleGPSerror, options);
    $('toggle-gps-btn').textContent = '⏸️ PAUSE GPS';
    $('toggle-gps-btn').classList.remove('btn-warning');
    $('toggle-gps-btn').classList.add('btn-success');
    console.log(`[GPS] Démarré en mode: ${currentGPSMode}`);
}

function stopGPS(reset = true) {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
        console.log('[GPS] Arrêté.');
    }
    
    $('toggle-gps-btn').textContent = '▶️ MARCHE GPS';
    $('toggle-gps-btn').classList.remove('btn-success');
    $('toggle-gps-btn').classList.add('btn-warning');
    
    if (reset) {
        lat = 0; lon = 0; alt = 0; speedInst = 0;
        updateMap(0, 0, 1);
        $('lat-raw').textContent = 'N/A';
        $('lon-raw').textContent = 'N/A';
        $('alt-raw').textContent = 'N/A';
        $('gps-precision').textContent = 'Arrêté';
    }
}

function handleGPSerror(error) {
    let msg = 'Erreur inconnue';
    switch (error.code) {
        case error.PERMISSION_DENIED:
            msg = "Accès à la géolocalisation refusé.";
            break;
        case error.POSITION_UNAVAILABLE:
            msg = "Position non disponible.";
            break;
        case error.TIMEOUT:
            msg = "Délai d'attente dépassé.";
            break;
    }
    console.error(`[GPS ERREUR] ${msg}`);
    $('gps-precision').textContent = `ERREUR: ${msg}`;
}

function updateGPS(position) {
    if (emergencyStopActive) return;

    const coords = position.coords;
    const now = position.timestamp;
    const dt = (now - lastUpdateTimestamp) / 1000;
    
    const oldLat = lat, oldLon = lon;

    lat = coords.latitude;
    lon = coords.longitude;
    alt = coords.altitude !== null ? coords.altitude : 0;
    speedInst = coords.speed !== null ? coords.speed : 0; // m/s

    if (lastUpdateTimestamp !== 0 && dt > 0) {
        // Calculer la distance (Méthode de la distance du grand cercle - très simplifié ici)
        const R = 6371e3; // Rayon de la Terre en mètres
        const φ1 = oldLat * D2R;
        const φ2 = lat * D2R;
        const Δφ = (lat - oldLat) * D2R;
        const Δλ = (lon - oldLon) * D2R;

        const a = Math.sin(Δφ/2) * Math.sin(Δφ/2) +
                  Math.cos(φ1) * Math.cos(φ2) *
                  Math.sin(Δλ/2) * Math.sin(Δλ/2);
        const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));

        const d_meters = R * c;
        distanceTraveled += d_meters;

        // Mise à jour EKF
        if (ukf) {
            // Mise à jour de la position GPS
            ukf.update('GPS_POS', [lat, lon, alt], [coords.accuracy, coords.accuracy, coords.altitudeAccuracy || coords.accuracy]); 
            
            // Mise à jour de la vitesse GPS
            ukf.update('GPS_VEL', [speedInst, 0, 0], [coords.speedAccuracy || 1, 1, 1]); 
            
            // Logique ZUPT
            if (isZUPT) {
                 ukf.update('ZUPT', [0, 0, 0], ukf.R_ZUPT);
            }
        }
    }
    
    // Déterminer si l'on est en mouvement
    isMoving = speedEKF > MIN_SPD;
    if (isMoving) timeMoving += dt;
    
    // Mise à jour Vitesse Max
    vMax = Math.max(vMax, speedEKF);

    // Affichage DOM
    $('lat-raw').textContent = lat.toFixed(6) + (lat > 0 ? ' N' : ' S');
    $('lon-raw').textContent = lon.toFixed(6) + (lon > 0 ? ' E' : ' O');
    $('alt-raw').textContent = alt.toFixed(1) + ' m';
    $('gps-precision').textContent = coords.accuracy.toFixed(1) + ' m';

    // Affichage Vitesse GPS Raw
    $('speed-3d-inst').textContent = dataOrDefault(speedInst * KMH_MS, 2, ' km/h');
    $('speed-raw-ms').textContent = dataOrDefault(speedInst, 3, ' m/s');

    updateMap(lat, lon, MAP_ZOOM_LEVEL);

    lastUpdateTimestamp = now;
}

// --- LOGIQUE IMU (Capteurs Inertiels) ---
function startIMUListeners() {
    if (emergencyStopActive) return;

    // Utilisation de l'API Generic Sensor
    try {
        // Accéléromètre
        accSensor = new Accelerometer({ frequency: 50 }); // 50 Hz
        accSensor.addEventListener('reading', () => {
            imuData.aX = accSensor.x;
            imuData.aY = accSensor.y;
            imuData.aZ = accSensor.z;
            updateIMUDisplay();
        });
        accSensor.addEventListener('error', handleIMUError);
        accSensor.start();

        // Gyroscope
        gyroSensor = new Gyroscope({ frequency: 50 });
        gyroSensor.addEventListener('reading', () => {
            imuData.wX = gyroSensor.x;
            imuData.wY = gyroSensor.y;
            imuData.wZ = gyroSensor.z;
            updateIMUDisplay();
        });
        gyroSensor.addEventListener('error', handleIMUError);
        gyroSensor.start();
        
        $('imu-status').textContent = '✅ Capteurs OK (50 Hz)';

        // Démarrer la boucle UKF rapide (qui est dépendante des données IMU)
        startFastLoop();

    } catch (error) {
        handleIMUError(error);
    }
}

function handleIMUError(event) {
    imuError = event.error;
    let msg = 'Erreur inconnue';
    if (imuError && imuError.name) {
        msg = imuError.name;
    } else if (event.type === 'error') {
        msg = 'Erreur Générique IMU';
    }
    $('imu-status').textContent = `❌ Erreur IMU: ${msg}`;
    console.error(`[IMU ERREUR] ${msg}`);
}

function updateIMUDisplay() {
    const { aX, aY, aZ, wX, wY, wZ } = imuData;
    
    // Calcul de la magnitude de l'accélération (pour ZUPT)
    lastAccelMag = Math.sqrt(aX * aX + aY * aY + aZ * aZ);
    
    // Accélération Longitudinale (Simulée)
    const accelLong = Math.abs(aX).toFixed(3);

    // Vitesse Angulaire Totale
    const angularSpeed = Math.sqrt(wX * wX + wY * wY + wZ * wZ) * R2D;
    
    $('accel-long').textContent = dataOrDefault(accelLong, 3, ' m/s²');
    $('angular-speed').textContent = dataOrDefault(angularSpeed, 2, ' °/s');
    $('accel-x').textContent = dataOrDefault(aX, 2, ' m/s²');
    $('accel-y').textContent = dataOrDefault(aY, 2, ' m/s²');
    $('accel-z').textContent = dataOrDefault(aZ, 2, ' m/s²');
}

// --- BOUCLE RAPIDE UKF ---
function startFastLoop() {
    if (domFastID !== null) return;
    
    const DOM_FAST_UPDATE_MS = 20; // 50 Hz pour la logique UKF/IMU
    
    domFastID = setInterval(() => {
        if (emergencyStopActive || !ukf) return;

        const now = Date.now();
        const dt = (now - ukf.lastTime) / 1000;
        
        // 1. Prédiction UKF (basée sur l'IMU)
        if (dt > 0.01) { // Ne prédire que si un temps significatif s'est écoulé
            ukf.predict(dt, imuData, imuData); 
        }

        // 2. Logique ZUPT (Zero Velocity Update)
        const threshold = 1.0; // Seuil d'accélération pour déterminer le mouvement
        isZUPT = lastAccelMag < threshold && speedEKF < MIN_SPD; 
        
        if (isZUPT) {
            $('speed-status-text').textContent = '✅ ZUPT (Attente Mouvement)';
            ukf.update('ZUPT', [0, 0, 0], ukf.R_ZUPT); // Correction vitesse = 0
        } else {
             $('speed-status-text').textContent = '🚀 EKF ACTIF (Mouvement)';
        }

        // 3. Mise à jour de la vitesse EKF
        speedEKF = ukf.getStableSpeed();
        
        // --- Affichage DOM UKF/Speed ---
        $('speed-stable').textContent = dataOrDefault(speedEKF * KMH_MS, 2);
        $('speed-stable-ms').textContent = dataOrDefault(speedEKF, 3, ' m/s');
        $('speed-stable-kms').textContent = dataOrDefaultExp(speedEKF / 1000, 2, ' km/s');
        
        $('incertitude-vitesse').textContent = dataOrDefault(ukf.getSpeedIncertitude(), 3, ' m²/s² (P)');
        $('incertitude-alt').textContent = dataOrDefault(ukf.getAltIncertitude(), 3, ' m (σ)');

        // Mise à jour de la distance, temps de vol, etc.
        const totalTime = (Date.now() - sTime) / 1000;
        $('elapsed-time').textContent = formatDuration(totalTime);
        $('time-moving').textContent = formatDuration(timeMoving);
        
        // Coefficient de Réalité (simplifié)
        const theoreticalDist = totalTime * (speedEKF * (1/KMH_MS));
        const MRF = distanceTraveled / (theoreticalDist || 1); 
        $('distance-ratio').textContent = dataOrDefault(MRF, 3);
        
        $('distance-total-km').textContent = `${dataOrDefault(distanceTraveled / 1000, 3, ' km')} | ${dataOrDefault(distanceTraveled, 2, ' m')}`;
        $('speed-max').textContent = dataOrDefault(vMax * KMH_MS, 2, ' km/h');
        
        // Vitesse moyenne (total et mouvement)
        const avgTotal = totalTime > 0 ? (distanceTraveled / totalTime) : 0;
        const avgMoving = timeMoving > 0 ? (distanceTraveled / timeMoving) : 0;
        
        $('speed-avg-total').textContent = dataOrDefault(avgTotal * KMH_MS, 2, ' km/h');
        $('speed-avg-moving').textContent = dataOrDefault(avgMoving * KMH_MS, 2, ' km/h');
        
        ukf.lastTime = now;
        
    }, DOM_FAST_UPDATE_MS);
}


// --- LOGIQUE D'AFFICHAGE (MAP) ---
function initMap() {
    if (typeof L === 'undefined') {
        console.error("Erreur: Leaflet n'est pas chargé.");
        return;
    }
    
    // Coordonnées de départ (Marseille - Exemple)
    const initialLat = 43.296;
    const initialLon = 5.37;
    
    map = L.map('map').setView([initialLat, initialLon], 10);

    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
    }).addTo(map);

    marker = L.marker([initialLat, initialLon]).addTo(map)
        .bindPopup("Position Actuelle")
        .openPopup();
        
    polyline = L.polyline([], { color: 'red' }).addTo(map);
}

function updateMap(lat, lon, zoom) {
    if (map && marker) {
        const newLatLng = [lat, lon];
        marker.setLatLng(newLatLng);
        polyline.addLatLng(newLatLng);
        map.setView(newLatLng, zoom);
    }
}

// --- CONTRÔLES ---
function toggleEmergencyStop() {
    emergencyStopActive = !emergencyStopActive;
    
    if (emergencyStopActive) {
        stopGPS(false);
        if (domFastID) clearInterval(domFastID);
        if (domSlowID) clearInterval(domSlowID);
        $('emergency-stop-btn').textContent = '🛑 ARRÊT D’URGENCE ACTIF 🔴';
        $('emergency-stop-btn').classList.add('active');
        $('speed-status-text').textContent = '🚨 ARRÊT D’URGENCE (Systèmes SUSPENDUS)';
        $('imu-status').textContent = '🛑 SUSPENDU';
        $('toggle-gps-btn').disabled = true;
    } else {
        startGPS();
        startFastLoop(); 
        $('emergency-stop-btn').textContent = '🛑 Arrêt d\'urgence: INACTIF 🟢';
        $('emergency-stop-btn').classList.remove('active');
        $('toggle-gps-btn').disabled = false;
        $('imu-status').textContent = imuError ? `❌ Erreur IMU: ${imuError.name}` : '✅ Capteurs OK (50 Hz)';
    }
}

function toggleGPS() {
    if (wID !== null) {
        stopGPS(false);
    } else {
        startGPS();
    }
}

function resetDistance() {
    distanceTraveled = 0;
    timeMoving = 0;
}

function resetVMax() {
    vMax = 0;
}

function resetAll() {
    resetDistance();
    resetVMax();
    sTime = Date.now();
    // Réinitialisation de l'UKF
    if (ukf) ukf = new ProfessionalUKF();
    if (polyline) polyline.setLatLngs([]); // Réinitialisation du tracé
}

function captureData() {
    console.log("[CAPTURE] Données capturées...");
}

// =================================================================
// INITIALISATION DOM (DÉMARRAGE) (FINAL - CORRIGÉ PERMISSION)
// =================================================================
document.addEventListener('DOMContentLoaded', () => {
    
    initMap(); 
    
    // --- Initialisation des contrôles ---
    
    if ($('ukf-reactivity-mode')) { 
        const reactivitySelect = $('ukf-reactivity-mode');
        reactivitySelect.value = currentUKFReactivity;
        const initialData = UKF_REACTIVITY_FACTORS[currentUKFReactivity];
        if ($('ukf-reactivity-factor')) $('ukf-reactivity-factor').textContent = `${initialData.DISPLAY} (x${initialData.MULT.toFixed(1)})`;
        
        reactivitySelect.addEventListener('change', (e) => {
            currentUKFReactivity = e.target.value;
            const data = UKF_REACTIVITY_FACTORS[currentUKFReactivity];
            if ($('ukf-reactivity-factor')) {
                 $('ukf-reactivity-factor').textContent = `${data.DISPLAY} (x${data.MULT.toFixed(1)})`;
            }
        });
    }

    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', toggleEmergencyStop);
    if ($('reset-distance-btn')) $('reset-distance-btn').addEventListener('click', resetDistance);
    if ($('reset-vmax-btn')) $('reset-vmax-btn').addEventListener('click', resetVMax);
    if ($('capture-data-btn')) $('capture-data-btn').addEventListener('click', captureData);
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', resetAll);
    
    if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', toggleGPS);
    if ($('freq-select')) $('freq-select').addEventListener('change', (e) => {
        currentGPSMode = e.target.value;
        if (wID !== null) { stopGPS(false); startGPS(currentGPSMode); }
    });
    
    if ($('toggle-mode-btn')) { 
        $('toggle-mode-btn').addEventListener('click', () => {
            document.body.classList.toggle('dark-mode');
            const isDarkMode = document.body.classList.contains('dark-mode');
            $('toggle-mode-btn').innerHTML = isDarkMode ? '<i class="fas fa-sun"></i> Mode Jour' : '<i class="fas fa-moon"></i> Mode Nuit';
        });
    }
    
    // --- GESTION DU NOUVEAU BOUTON D'ACTIVATION CAPTEURS (SOLUTION DU NOTREADABLEERROR) ---
    const startIMUBtn = $('start-imu-btn');
    if (startIMUBtn) {
        startIMUBtn.addEventListener('click', () => {
            // L'appel à startIMUListeners() est désormais une action directe de l'utilisateur
            startIMUListeners(); 
            startIMUBtn.disabled = true;
            startIMUBtn.textContent = '✅ Capteurs Activés (Cliquez OK si nécessaire)';
        });
    }


    // --- DÉMARRAGE DU SYSTÈME (Sync NTP puis GPS) ---
    // updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity); 
    
    // Tentative de synchronisation NTP
    syncH(lServH, lLocH).then(newTimes => {
        lServH = newTimes.lServH;
        lLocH = newTimes.lLocH;
    }).finally(() => { 
        // Initialisation du UKF (nécessite math.js)
        if (typeof ProfessionalUKF !== 'undefined') {
            ukf = new ProfessionalUKF();
        } else {
            alert("Erreur: La classe ProfessionalUKF est manquante.");
            return;
        }
        
        // Démarrage du GPS
        sTime = Date.now();
        startGPS(); 
        
        // Démarrage de la boucle lente (Astro/Météo)
        const DOM_SLOW_UPDATE_MS = 3000;
        domSlowID = setInterval(() => {
            const currentLat = lat || 43.296; 
            const currentLon = lon || 5.37;
            
            try {
                updateAstro(currentLat, currentLon, lServH, lLocH);
            } catch (e) {
                console.error("Erreur dans updateAstro:", e);
            }
            
            // AUTOMATISATION DU MODE NUIT (via SunCalc)
            const now = getCDate(lServH, lLocH);
            if (typeof SunCalc !== 'undefined' && lat && lon) {
                const sunTimes = SunCalc.getTimes(now, currentLat, currentLon);
                const isNight = (now.getTime() < sunTimes.sunrise.getTime() || now.getTime() > sunTimes.sunset.getTime());
                
                if (isNight) {
                    document.body.classList.add('dark-mode');
                    if ($('toggle-mode-btn')) $('toggle-mode-btn').innerHTML = '<i class="fas fa-sun"></i> Mode Jour';
                } else {
                    document.body.classList.remove('dark-mode');
                    if ($('toggle-mode-btn')) $('toggle-mode-btn').innerHTML = '<i class="fas fa-moon"></i> Mode Nuit';
                }
            }
            
            // Gestion Météo
            if (lat && lon && !emergencyStopActive && typeof fetchWeather === 'function') {
                fetchWeather(currentLat, currentLon).then(data => {
                    if (data) {
                        lastP_hPa = data.pressure_hPa;
                        lastT_K = data.tempK;
                        currentAirDensity = getAirDensity(data.pressure_hPa, data.tempK);
                        currentSpeedOfSound = getSpeedOfSound(data.tempK);
                        
                        if ($('weather-status')) $('weather-status').textContent = `ACTIF`;
                        if ($('temp-air-2')) $('temp-air-2').textContent = `${data.tempC.toFixed(1)} °C`;
                        if ($('pressure-2')) $('pressure-2').textContent = `${data.pressure_hPa.toFixed(0)} hPa`;
                        if ($('humidity-2')) $('humidity-2').textContent = `${data.humidity_perc.toFixed(0)} %`;
                        if ($('air-density')) $('air-density').textContent = `${currentAirDensity.toFixed(3)} kg/m³`;
                        if ($('dew-point')) $('dew-point').textContent = `${data.dew_point.toFixed(1)} °C`;
                        if ($('speed-of-sound')) $('speed-of-sound').textContent = `${currentSpeedOfSound.toFixed(2)} m/s`;
                    }
                }).catch(err => {
                    if ($('weather-status')) $('weather-status').textContent = `❌ API ÉCHOUÉE`;
                });
            }
            
            // Met à jour l'horloge locale (NTP)
            if (now) {
                if ($('local-time') && !$('local-time').textContent.includes('Synchronisation...')) {
                    $('local-time').textContent = now.toLocaleTimeString('fr-FR');
                }
                if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
            }
            
        }, DOM_SLOW_UPDATE_MS); 
    });
});
  
