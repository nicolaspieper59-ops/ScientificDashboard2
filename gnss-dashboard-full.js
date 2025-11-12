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
const KMH_MS = 3.6;          
const C_L = 299792458;          // Vitesse de la lumière (m/s)
const C_S_STD = 343;            // Vitesse du son standard (m/s)
const R_SPECIFIC_AIR = 287.058; // Constante spécifique de l'air sec (J/kg·K)
const R_E_BASE = 6371000;       // Rayon terrestre moyen (m)

// --- CONSTANTES DE FRÉQUENCE IMU ---
const IMU_FREQUENCY_HZ = 10; // Fréquence de récupération des données IMU
const DT_IMU = 1 / IMU_FREQUENCY_HZ; 

// --- PARAMÈTRES EKF / DR ---
const R_MIN_GPS = 1.0;            
const MAX_ACC_ESTIMATION = 200.0; // Précision max (m) avant "Estimation Seule"
const MIN_SPD = 0.05;

// --- FACTEURS ENVIRONNEMENTAUX (POUR R) ---
const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0, DISPLAY: 'Normal' },
    'FOREST': { R_MULT: 2.5, DISPLAY: 'Forêt' },
    'CONCRETE': { R_MULT: 7.0, DISPLAY: 'Grotte/Tunnel' },
    'METAL': { R_MULT: 5.0, DISPLAY: 'Métal/Bâtiment' },
};

// --- DONNÉES CÉLESTES/GRAVITÉ ---
const CELESTIAL_DATA = {
    'EARTH': { G: 9.81, R: R_E_BASE, name: 'Terre' },
    'MOON': { G: 1.62, R: 1737400, name: 'Lune' },
    'MARS': { G: 3.71, R: 3389500, name: 'Mars' },
    'ROTATING': { G: 0.0, R: R_E_BASE, name: 'Station Spatiale' }
};

// --- VARIABLES D'ÉTAT GLOBALES ---
let wID = null, lPos = null, sTime = null; 
let imuIntervalID = null; 
let ekf6dof = null; // L'objet EKF INS 
let currentTransportMode = 'INS_6DOF_REALISTE'; 
let emergencyStopActive = false;
let map, marker, circle;

let lat = null, lon = null, kAlt = null; // Position filtrée EKF
let distM = 0, maxSpd = 0, timeMoving = 0;
let lastFSpeed = 0; // Vitesse filtrée précédente
let lServH = null, lLocH = null; // Temps NTP
let currentMass = 70.0; 
let currentCelestialBody = 'EARTH';
let rotationRadius = 100;
let angularVelocity = 0.0; 
let G_ACC = 9.81; 
let lastWeatherData = null;
let gpsAccuracyOverride = 0.0;
let selectedEnvironment = 'NORMAL';
let sunAltitudeRad = 0; // Altitude du soleil en radians pour calculs biologiques

// Variables IMU
let imuAccelX = 0, imuAccelY = 0, imuAccelZ = 0;
let imuGyroX = 0, imuGyroY = 0, imuGyroZ = 0;
let lastIMUTime = 0;

// Fonctions utilitaires rapides
const $ = id => document.getElementById(id); 

// --- EKF/INS 6DOF CONCEPTUEL (Simulé) ---
/** Classe très simplifiée pour simuler un EKF INS 21-États */
class EKF_INS_21_States {
    constructor() {
        // Incertitude initiale (P)
        this.P = 1000; 
        this.Q_VEL = 0.1; // Bruit de Processus Vitesse
        this.R_ALT = 100; // Bruit de Mesure GPS Altitude
        this.R_YAW = 100; // Bruit de Mesure GPS Cap
        this.kAltUncert = 100;

        this.lat = null;
        this.lon = null;
        this.alt = null;
        this.speedMS = 0;
        this.vX = 0; this.vY = 0; this.vZ = 0;
        this.roll = 0; this.pitch = 0; this.yaw = 0;
        this.biasGZ = 0; // Bias Gyro Z (Yaw)
    }

    // Fonction de prédiction INS (très simplifiée)
    predict(dt, accel, gyroZ) {
        if (dt <= 0) return;
        
        // 1. Mise à jour de l'orientation (Yaw)
        const gyroZ_corr = gyroZ - this.biasGZ;
        this.yaw += gyroZ_corr * dt * D2R; 

        // 2. Intégration de l'accélération (Corps -> Terre) vers la vitesse
        const accel_earth_x = accel.x * Math.cos(this.yaw); 
        const accel_earth_y = accel.x * Math.sin(this.yaw); 
        const accel_earth_z = accel.z; 

        this.vX += accel_earth_x * dt;
        this.vY += accel_earth_y * dt;
        this.vZ += (accel_earth_z - G_ACC) * dt; 

        // 3. Intégration de la position (Dead Reckoning)
        if (this.lat !== null) {
            const spd_horiz = Math.sqrt(this.vX**2 + this.vY**2);
            const dist_travelled = spd_horiz * dt;
            const heading_rad = this.yaw;
            const dLat = (dist_travelled * Math.cos(heading_rad)) / R_E_BASE * R2D;
            const dLon = (dist_travelled * Math.sin(heading_rad)) / (R_E_BASE * Math.cos(this.lat * D2R)) * R2D;
            
            this.lat += dLat;
            this.lon += dLon;
            this.alt += this.vZ * dt;
        }

        this.speedMS = Math.sqrt(this.vX**2 + this.vY**2);
        this.P += this.Q_VEL * dt; // Augmentation de l'incertitude
    }

    // Fonction de correction GNSS (simulée)
    correctGNSS(pos, accRaw, headingRaw, altRaw) {
        if (pos === null) return;
        
        const R_dyn = getKalmanR(accRaw, this.alt, lastWeatherData?.pressure_hPa, selectedEnvironment);
        
        // CORRECTION VITESSE 
        const K_vel = this.P / (this.P + R_dyn);
        const gnssSpd = pos.coords.speed || 0; 
        this.speedMS += K_vel * (gnssSpd - this.speedMS);
        this.P = (1 - K_vel) * this.P;
        
        // CORRECTION POSITION (Lat/Lon)
        const K_pos = R_dyn / (this.P + R_dyn);
        if (this.lat !== null) {
            this.lat += K_pos * (pos.coords.latitude - this.lat);
            this.lon += K_pos * (pos.coords.longitude - this.lon);
        } else {
            this.lat = pos.coords.latitude;
            this.lon = pos.coords.longitude;
        }

        // CORRECTION ALTITUDE
        const K_alt = this.kAltUncert / (this.kAltUncert + this.R_ALT);
        if (this.alt !== null) {
            this.alt += K_alt * (altRaw - this.alt);
        } else {
            this.alt = altRaw;
        }
        this.kAltUncert = (1 - K_alt) * this.kAltUncert;
        
        // CORRECTION YAW/CAP
        if (headingRaw !== null && headingRaw >= 0) {
            const headingRad = headingRaw * D2R;
            const K_yaw = this.R_YAW / (this.P + this.R_YAW);
            let yawError = headingRad - this.yaw;
            while (yawError > Math.PI) yawError -= 2 * Math.PI;
            while (yawError < -Math.PI) yawError += 2 * Math.PI;
            
            this.yaw += K_yaw * yawError;
            this.R_YAW = (1 - K_yaw) * this.R_YAW;
            
            this.biasGZ += 0.0001 * yawError; // Mise à jour très simplifiée du bias
        }
    }
}


// --- FONCTIONS MATHÉMATIQUES ET PHYSIQUES ---

/** Calcule le bruit de mesure R (pour EKF) */
function getKalmanR(acc, alt, P_hPa, selectedEnv) {
    let acc_effective = acc;
    if (acc > MAX_ACC_ESTIMATION) return 1e9; 
    
    let R = acc_effective * acc_effective; 
    
    const envFactor = ENVIRONMENT_FACTORS[selectedEnv]?.R_MULT || 1.0;
    R *= envFactor;
    
    return Math.max(R_MIN_GPS, R); 
}

/** Met à jour la gravité locale (y compris pour mode ROTATING) */
function updateCelestialBody(bodyKey, alt, r_rot, omega_rot) {
    let G_ACC_local = 0;
    let R_ALT_CENTER_REF_local = R_E_BASE;

    if (bodyKey === 'ROTATING') {
        // Gravité artificielle = Accélération Centripète
        G_ACC_local = r_rot * omega_rot ** 2;
    } else {
        const data = CELESTIAL_DATA[bodyKey];
        if (data) {
            G_ACC_local = data.G;
            R_ALT_CENTER_REF_local = data.R;
        }
        if (alt !== null) {
            // Gravité en fonction de l'altitude
            G_ACC_local = G_ACC_local * (R_ALT_CENTER_REF_local / (R_ALT_CENTER_REF_local + alt)) ** 2;
        }
    }
    
    G_ACC = G_ACC_local; // Mise à jour de la variable globale
    if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC.toFixed(4)} m/s²`;
    
    return { G_ACC: G_ACC_local, R_ALT_CENTER_REF: R_ALT_CENTER_REF_local };
}

/** Calcule la vitesse du son local */
function calculateLocalSoundSpeed(tempK) {
    const GAMMA_AIR = 1.4; // Indice adiabatique de l'air
    return Math.sqrt(GAMMA_AIR * R_SPECIFIC_AIR * tempK);
}

/** Calcule la pression dynamique (q) et la force de traînée (simplifiées) */
function calculateDrag(speedMS, airDensity, mass, dt) {
    if (airDensity === null || speedMS === 0) return { dynamicPressure: 0, dragForce: 0, CD_factor: 0 };
    
    const CD_factor = 0.01; 
    const A_ref = 1.0; 
    
    const dynamicPressure = 0.5 * airDensity * speedMS ** 2;
    const dragForce = dynamicPressure * CD_factor * A_ref;
    
    return { dynamicPressure, dragForce, CD_factor };
    }
// =================================================================
// BLOC 2/4 : Astro & Horloge (Animation Minecraft)
// =================================================================

// --- CONSTANTES DE TEMPS & CALENDRIER ---
const MC_DAY_MS = 72 * 60 * 1000; // Durée d'un jour Minecraft en ms

/** Retourne l'heure synchronisée (précision RTT compensée en UTC). */
function getCDate(lServH, lLocH) { 
    if (lServH === null || lLocH === null) { return null; }
    const offsetSinceSync = performance.now() - lLocH;
    return new Date(lServH + offsetSinceSync); 
}

/** Synchronise l'horloge interne avec un serveur de temps (UTC/Atomique) */
async function syncH() {
    let lServH_in = lServH;
    let lLocH_in = lLocH;
    
    if ($('local-time')) $('local-time').textContent = 'Synchronisation...';
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

        lServH_in = serverTimestamp + latencyOffset; 
        lLocH_in = performance.now(); 
        
        const now = getCDate(lServH_in, lLocH_in);
        if (now) {
            if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString('fr-FR');
            if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
        }

    } catch (error) {
        lServH_in = Date.now(); 
        lLocH_in = performance.now();
        if ($('local-time')) $('local-time').textContent = 'N/A (SYNCHRO ÉCHOUÉE)';
    }
    lServH = lServH_in;
    lLocH = lLocH_in;
}

/** Calcule le temps Minecraft. */
function getMinecraftTime(date) {
    if (date === null) return '00:00';
    const msSinceMidnightUTC = date.getUTCHours() * 3600000 + date.getUTCMilliseconds() + date.getUTCMinutes() * 60000 + date.getUTCSeconds() * 1000;
    
    // Le jour Minecraft commence à 6:00 UTC (0h dans le jeu)
    const offsetMs = 6 * 3600 * 1000;
    let msSinceStartDay = (msSinceMidnightUTC - offsetMs + (24 * 3600 * 1000)) % (24 * 3600 * 1000);

    const mcTimeMs = (msSinceStartDay / (24 * 3600 * 1000)) * MC_DAY_MS;
    
    const totalMCMinutes = (mcTimeMs / 60000) * (24 * 60 / 20); 
    
    const mcHour = Math.floor(totalMCMinutes / 60) % 24;
    const mcMinute = Math.floor(totalMCMinutes % 60);

    const toTimeString = (h, m) => {
        return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}`;
    };
    return toTimeString(mcHour, mcMinute);
}

/** Met à jour l'horloge visuelle et les couleurs du corps (Day/Night) */
function updateClockVisualization(now, sunPos, moonPos, sunTimes) {
    const sunEl = $('sun-element');
    const moonEl = $('moon-element');
    const clockEl = $('minecraft-clock'); 

    if (!sunEl || !moonEl || !clockEl) return;

    // 1. Position du Soleil (Animation Disque Solaire)
    if (sunPos) {
        const altRad = sunPos.altitude;
        const aziDeg = (sunPos.azimuth * R2D + 180) % 360; 
        sunAltitudeRad = altRad; 

        // Rotation Azimutale (Autour du centre)
        sunEl.style.transform = `rotate(${aziDeg}deg)`;

        // Déplacement Radial (De l'intérieur vers le bord)
        const offsetRange = 50; // Max 50% du rayon
        const altDeg = altRad * R2D;
        let radialOffset;

        // L'altitude de 90° (Zénith) est le centre (50% top). 
        // L'altitude de 0° (Horizon) est le bord (0% top).
        radialOffset = offsetRange * (90 - altDeg) / 90;
        radialOffset = Math.min(offsetRange, Math.max(0, radialOffset)); // Limite à 0% et 50%
        
        const sunIcon = sunEl.querySelector('.sun-icon');
        // top: 0% est le bord supérieur du conteneur. top: 50% est le centre.
        sunIcon.style.top = `${radialOffset}%`; 
        
        sunEl.style.display = altDeg > -5 ? 'flex' : 'none'; 
    } else {
        sunEl.style.display = 'none';
    }

    // 2. Position de la Lune (Même logique)
    if (moonPos) {
        const altRad = moonPos.altitude;
        const aziDeg = (moonPos.azimuth * R2D + 180) % 360; 
        
        moonEl.style.transform = `rotate(${aziDeg}deg)`;
        
        const offsetRange = 50; 
        const altDeg = altRad * R2D;
        let radialOffset;

        radialOffset = offsetRange * (90 - altDeg) / 90;
        radialOffset = Math.min(offsetRange, Math.max(0, radialOffset));

        const moonIcon = moonEl.querySelector('.moon-icon');
        moonIcon.style.top = `${radialOffset}%`; 
        
        moonEl.style.display = altDeg > -5 ? 'flex' : 'none'; 
    } else {
        moonEl.style.display = 'none';
    }
    
    // 3. Fond du corps et de l'horloge
    const body = document.body;
    body.classList.remove('sky-day', 'sky-sunset', 'sky-night');
    clockEl.classList.remove('sky-day', 'sky-sunset', 'sky-night');

    if (sunTimes) {
        const nowMs = now.getTime();
        let bodyClass;
        if (nowMs >= sunTimes.sunriseEnd.getTime() && nowMs < sunTimes.sunsetStart.getTime()) {
            bodyClass = 'sky-day';
        } else if (nowMs >= sunTimes.dusk.getTime() || nowMs < sunTimes.dawn.getTime()) {
            bodyClass = 'sky-night';
        } else {
            bodyClass = 'sky-sunset';
        }
        
        body.classList.add(bodyClass);
        body.classList.toggle('dark-mode', bodyClass !== 'sky-day');
        clockEl.classList.add(bodyClass); 

        $('clock-status').textContent = sunPos && sunPos.altitude > 0 ? 'Jour Solaire (☀️)' : 'Nuit/Crépuscule (🌙)';
    } else {
        $('clock-status').textContent = 'Position Solaire Indisponible';
    }
}

/** Fonction principale de mise à jour Astro (appelée par la boucle lente) */
function updateAstro(latA, lonA) {
    const now = getCDate(lServH, lLocH); 
    
    if (now === null) return; 
    
    if ($('time-minecraft')) $('time-minecraft').textContent = getMinecraftTime(now);

    if (typeof SunCalc === 'undefined' || !latA || !lonA) {
        $('clock-status').textContent = 'Astro (Attente GPS)...';
        return;
    }
    
    const sunPos = SunCalc.getPosition(now, latA, lonA);
    const moonIllum = SunCalc.getMoonIllumination(now);
    const moonPos = SunCalc.getMoonPosition(now, latA, lonA);
    const sunTimes = SunCalc.getTimes(now, latA, lonA);
    
    // Mise à jour du DOM
    if ($('sun-altitude')) $('sun-altitude').textContent = `${(sunPos.altitude * R2D).toFixed(2)} °`;
    if ($('sun-azimuth')) $('sun-azimuth').textContent = `${(sunPos.azimuth * R2D + 180).toFixed(2) % 360} °`;
    
    // Fonction d'aide pour la phase de lune (non incluse dans SunCalc par défaut)
    const getMoonPhaseName = (phase) => {
        if (phase < 0.0625 || phase >= 0.9375) return 'Nouvelle Lune';
        if (phase < 0.1875) return 'Premier Croissant';
        if (phase < 0.3125) return 'Premier Quartier';
        if (phase < 0.4375) return 'Lune Gibbeuse Croissante';
        if (phase < 0.5625) return 'Pleine Lune';
        if (phase < 0.6875) return 'Lune Gibbeuse Décroissante';
        if (phase < 0.8125) return 'Dernier Quartier';
        return 'Dernier Croissant';
    };
    if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(moonIllum.phase);
    
    if ($('day-duration') && sunTimes.sunrise && sunTimes.sunset) {
        const durationMs = sunTimes.sunset.getTime() - sunTimes.sunrise.getTime();
        const hours = Math.floor(durationMs / 3600000);
        const minutes = Math.floor((durationMs % 3600000) / 60000);
        $('day-duration').textContent = `${hours}h ${minutes}m`;
    } else if ($('day-duration')) {
        $('day-duration').textContent = 'N/A (Polaire/Nuit)';
    }

    updateClockVisualization(now, sunPos, moonPos, sunTimes);
}

// --- FONCTIONS MÉTÉO ---

/** Récupère et traite les données météo via l'API */
async function fetchWeather(latA, lonA) {
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
            
            // Calcul de la densité de l'air
            const pressure_pa = pressure_hPa * 100;
            const air_density = pressure_pa / (R_SPECIFIC_AIR * tempK);
            
            // Calcul du point de rosée
            const a = 17.27, b = 237.7;
            const h_frac = humidity_perc / 100.0;
            const f = (a * tempC) / (b + tempC) + Math.log(h_frac);
            const dew_point = (b * f) / (a - f);
            
            weatherData = {
                tempC: tempC,
                pressure_hPa: pressure_hPa,
                humidity_perc: humidity_perc,
                tempK: tempK,
                air_density: air_density,
                dew_point: dew_point
            };
        } else {
             throw new Error(data.message || 'Données météo incomplètes');
        }
    } catch (err) {
        console.warn("Erreur de récupération météo:", err.message);
    }
    return weatherData; 
}
// =================================================================
// BLOC 3/4 : Gestion des Capteurs (GPS, IMU) & Carte
// =================================================================

// --- Fonctions Carte ---
function initMap() {
    try {
        if ($('map') && typeof L !== 'undefined') { 
            map = L.map('map').setView([0, 0], 2);
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                attribution: '© OpenStreetMap contributors'
            }).addTo(map);
            marker = L.marker([0, 0]).addTo(map);
            circle = L.circle([0, 0], { color: 'red', fillColor: '#f03', fillOpacity: 0.5, radius: 10 }).addTo(map);
        }
    } catch (e) {
        if ($('map')) $('map').innerHTML = "Erreur d'initialisation de la carte.";
    }
}

function updateMap(latA, lonA, acc) {
    if (map && marker) {
        marker.setLatLng([latA, lonA]);
        circle.setLatLng([latA, lonA]).setRadius(acc); 
        
        if (ekf6dof && ekf6dof.speedMS > MIN_SPD || map.getZoom() < 10) {
            map.setView([latA, lonA], map.getZoom() > 10 ? map.getZoom() : 16); 
        }
    }
}


// --- GESTION DES CAPTEURS IMU ---
function imuMotionHandler(event) {
    if (event.accelerationIncludingGravity) {
        imuAccelX = event.accelerationIncludingGravity.x || 0;
        imuAccelY = event.accelerationIncludingGravity.y || 0;
        imuAccelZ = event.accelerationIncludingGravity.z || 0;
    }
    if (event.rotationRate) {
        imuGyroX = event.rotationRate.alpha || 0;
        imuGyroY = event.rotationRate.beta || 0;
        imuGyroZ = event.rotationRate.gamma || 0;
    }
    if ($('imu-status')) $('imu-status').textContent = "Actif";
}

function startIMUListeners() {
    if (window.DeviceMotionEvent) {
        if (typeof DeviceMotionEvent.requestPermission === 'function') {
            DeviceMotionEvent.requestPermission()
                .then(permissionState => {
                    if (permissionState === 'granted') {
                        window.addEventListener('devicemotion', imuMotionHandler);
                    }
                })
                .catch(console.error);
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


// --- LOGIQUE DE DÉMARRAGE INS/GNSS ---
function startFusion() {
    if (wID !== null) return; 

    // 1. Initialiser le filtre EKF 6DOF
    ekf6dof = new EKF_INS_21_States();
    if ($('nhc-status')) $('nhc-status').textContent = '✅ EKF INS ACTIF';

    // 2. Démarrer le capteur IMU
    startIMUListeners(); 

    // 3. Démarrer la boucle de prédiction IMU (INS)
    lastIMUTime = performance.now();
    imuIntervalID = setInterval(runIMUIntegration, DT_IMU * 1000);

    // 4. Démarrer l'écoute GPS (Correction GNSS)
    const options = { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 };
    wID = navigator.geolocation.watchPosition(runGNSSCorrection, handleErr, options);
    
    if ($('toggle-gps-btn')) {
        $('toggle-gps-btn').textContent = '⏸️ PAUSE FUSION';
        $('toggle-gps-btn').style.backgroundColor = '#ffc107'; 
    }
}

function stopGPS() {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    if (imuIntervalID !== null) clearInterval(imuIntervalID); 
    stopIMUListeners(); 
    wID = null;
    imuIntervalID = null;
    ekf6dof = null;
    
    if ($('nhc-status')) $('nhc-status').textContent = 'Arrêté';
    if ($('toggle-gps-btn')) {
        $('toggle-gps-btn').textContent = '▶️ Démarrer la Fusion (INS/GNSS)';
        $('toggle-gps-btn').style.backgroundColor = '#28a745'; 
    }
}

function emergencyStop() {
    emergencyStopActive = true;
    stopGPS(); 
    if ($('emergency-stop-btn')) {
        $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: ACTIF 🔴";
        $('emergency-stop-btn').classList.add('active');
    }
}

function resumeSystem() {
    emergencyStopActive = false;
    if ($('emergency-stop-btn')) {
        $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence";
        $('emergency-stop-btn').classList.remove('active');
    }
    startFusion();
}

function handleErr(err) {
    console.warn(`ERREUR GPS (${err.code}): ${err.message}`);
    if ($('gps-precision')) $('gps-precision').textContent = `Erreur: ${err.message}`;
    if (err.code === 1) { 
        stopGPS();
        alert("Accès à la géolocalisation refusé. Veuillez l'activer.");
    }
                }
// =================================================================
// BLOC 4/4 : Boucles de Traitement et Affichage
// =================================================================

/** Boucle de prédiction INS (Dead Reckoning) */
function runIMUIntegration() {
    if (!ekf6dof || emergencyStopActive) return;

    const cTime = performance.now();
    const dt = (cTime - lastIMUTime) / 1000;
    lastIMUTime = cTime;
    
    if (dt > 0.5) { 
        console.warn("Latence IMU trop élevée, saut de prédiction.");
        return;
    }
    
    ekf6dof.predict(dt, 
        { x: imuAccelX, y: imuAccelY, z: imuAccelZ }, 
        imuGyroZ
    );
    
    updateDisplayFast(dt);
}

/** Fonction de correction GNSS (Appelée par navigator.geolocation.watchPosition) */
function runGNSSCorrection(pos) {
    if (!ekf6dof || emergencyStopActive) return;

    const cTimePos = pos.timestamp;
    const dt_gnss = sTime === null ? 0 : (cTimePos - sTime) / 1000;
    
    if (sTime === null) sTime = cTimePos;

    let accRaw = pos.coords.accuracy;
    if (gpsAccuracyOverride > 0.0) {
        accRaw = gpsAccuracyOverride;
    }
    
    // Initialisation de l'EKF avec la première donnée GPS
    if (ekf6dof.lat === null) {
        ekf6dof.lat = pos.coords.latitude;
        ekf6dof.lon = pos.coords.longitude;
        ekf6dof.alt = pos.coords.altitude;
        ekf6dof.yaw = pos.coords.heading !== null ? pos.coords.heading * D2R : 0;
        lPos = pos;
        return;
    }
    
    ekf6dof.correctGNSS(pos, accRaw, pos.coords.heading, pos.coords.altitude);
    
    updateDisplaySlow(dt_gnss, pos.coords.speed || 0, accRaw);
    
    lPos = pos; 
    sTime = cTimePos; 
}


/** Mise à jour rapide (par IMU) */
function updateDisplayFast(dt) {
    if (!ekf6dof) return;
    const sSpdFE = ekf6dof.speedMS < MIN_SPD ? 0 : ekf6dof.speedMS;
    
    // 1. Calculs
    let accel_long = 0;
    if (dt > 0) { 
        accel_long = (sSpdFE - lastFSpeed) / dt;
    }
    lastFSpeed = sSpdFE;
    
    // 2. Mise à jour de la distance et temps
    distM += sSpdFE * dt; 
    if (sSpdFE > MIN_SPD) { timeMoving += dt; }
    if (sSpdFE * KMH_MS > maxSpd) maxSpd = sSpdFE * KMH_MS; 
    
    const dragData = calculateDrag(sSpdFE, lastWeatherData?.air_density || 1.225, currentMass, dt);
    
    // 3. DOM (Vitesse/Accélération/Distance/IMU)
    if ($('speed-stable')) $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(5)}`;
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${sSpdFE.toFixed(3)} m/s`;
    if ($('vertical-speed')) $('vertical-speed').textContent = `${ekf6dof.vZ.toFixed(2)} m/s`;
    if ($('distance-total-km')) $('distance-total-km').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    if ($('speed-max')) $('speed-max').textContent = `${maxSpd.toFixed(1)} km/h`;
    
    if ($('accel-long')) $('accel-long').textContent = `${accel_long.toFixed(3)} m/s²`;
    if ($('force-g-long')) $('force-g-long').textContent = G_ACC > 0.1 ? `${(accel_long / G_ACC).toFixed(2)} G` : '0.00 G';
    if ($('drag-force')) $('drag-force').textContent = `${dragData.dragForce.toFixed(2)} N`;
    if ($('dynamic-pressure')) $('dynamic-pressure').textContent = `${dragData.dynamicPressure.toFixed(2)} Pa`;

    // IMU
    if ($('imu-accel-x')) $('imu-accel-x').textContent = `${imuAccelX.toFixed(2)} m/s²`;
    if ($('imu-gyro-z')) $('imu-gyro-z').textContent = `${imuGyroZ.toFixed(2)} °/s`;
    if ($('bias-gyro-z')) $('bias-gyro-z').textContent = `${(ekf6dof.biasGZ * R2D).toFixed(3)} °/s`; // Le bias est en rad/s, on l'affiche en °/s
}

/** Mise à jour lente (par GPS) */
function updateDisplaySlow(dt, gnssSpdRaw, accRaw) {
    if (!ekf6dof) return;
    
    // Calculs lents
    const R_dyn = getKalmanR(accRaw, ekf6dof.alt, lastWeatherData?.pressure_hPa, selectedEnvironment);
    const localSoundSpeed = lastWeatherData ? calculateLocalSoundSpeed(lastWeatherData.tempK) : C_S_STD;
    const mach = ekf6dof.speedMS / localSoundSpeed;
    
    // Mise à jour de la carte
    updateMap(ekf6dof.lat, ekf6dof.lon, Math.sqrt(ekf6dof.P));

    // Mise à jour du corps céleste
    const newVals = updateCelestialBody(currentCelestialBody, ekf6dof.alt, rotationRadius, angularVelocity);
    G_ACC = newVals.G_ACC;
    
    // DOM
    if ($('time-elapsed')) $('time-elapsed').textContent = `${((Date.now() - sTime) / 1000).toFixed(2)} s`;
    if ($('time-moving')) $('time-moving').textContent = `${timeMoving.toFixed(2)} s`;
    if ($('latitude')) $('latitude').textContent = `${ekf6dof.lat.toFixed(6)} °`;
    if ($('longitude')) $('longitude').textContent = `${ekf6dof.lon.toFixed(6)} °`;
    if ($('altitude-gps')) $('altitude-gps').textContent = `${ekf6dof.alt.toFixed(2)} m`;
    if ($('heading-display')) $('heading-display').textContent = `${(ekf6dof.yaw * R2D).toFixed(1)} °`;
    if ($('roll-display')) $('roll-display').textContent = `${ekf6dof.roll.toFixed(2)} °`;
    if ($('pitch-display')) $('pitch-display').textContent = `${ekf6dof.pitch.toFixed(2)} °`;
    if ($('gps-precision')) $('gps-precision').textContent = `${accRaw.toFixed(2)} m`;
    if ($('speed-raw-ms')) $('speed-raw-ms').textContent = `${gnssSpdRaw.toFixed(3)} m/s`;
    
    if ($('speed-sound-local')) $('speed-sound-local').textContent = `${localSoundSpeed.toFixed(1)} m/s`;
    if ($('mach-number')) $('mach-number').textContent = `${mach.toFixed(2)}`;
    
    // Debug EKF
    if ($('kalman-uncert')) $('kalman-uncert').textContent = `${ekf6dof.P.toFixed(2)} m²/s² (P)`;
    if ($('alt-uncertainty')) $('alt-uncertainty').textContent = `${Math.sqrt(ekf6dof.kAltUncert).toFixed(2)} m (σ)`;
    if ($('speed-error-perc')) $('speed-error-perc').textContent = `${R_dyn.toFixed(2)} m² (R dyn)`;
    if ($('dead-reckoning-error')) $('dead-reckoning-error').textContent = `${Math.sqrt(ekf6dof.P).toFixed(2)} m`; 
    if ($('gravity-local')) $('gravity-local').textContent = `${G_ACC.toFixed(5)} m/s²`;
    
    // Cosmique
    if ($('perc-speed-c')) $('perc-speed-c').textContent = `${(ekf6dof.speedMS / C_L * 100).toExponential(2)}%`;
    if ($('distance-light-s')) $('distance-light-s').textContent = `${(distM / C_L).toExponential(2)} s`;
    if ($('distance-cosmic')) $('distance-cosmic').textContent = `${(distM / 149597870700).toExponential(2)} UA | ${(distM / 9460730472580800).toExponential(2)} al`;
}


// --- INITIALISATION DES ÉVÉNEMENTS DOM ---
document.addEventListener('DOMContentLoaded', () => {
    
    initMap(); 

    // --- Configuration des Contrôles ---
    
    const updateRotation = () => {
        rotationRadius = parseFloat($('rotation-radius')?.value) || 100;
        angularVelocity = parseFloat($('angular-velocity')?.value) || 0;
        if ($('rotation-radius-display')) $('rotation-radius-display').textContent = `${rotationRadius.toFixed(0)} m`;
        if ($('angular-velocity-display')) $('angular-velocity-display').textContent = `${angularVelocity.toFixed(4)} rad/s`;

        if (currentCelestialBody === 'ROTATING') {
            updateCelestialBody('ROTATING', ekf6dof?.alt, rotationRadius, angularVelocity);
        }
    };
    if ($('rotation-radius')) $('rotation-radius').addEventListener('input', updateRotation);
    if ($('angular-velocity')) $('angular-velocity').addEventListener('input', updateRotation);
    updateRotation(); 

    if ($('celestial-body-select')) {
        $('celestial-body-select').addEventListener('change', (e) => { 
            currentCelestialBody = e.target.value;
            updateCelestialBody(currentCelestialBody, ekf6dof?.alt, rotationRadius, angularVelocity);
        });
    }

    if ($('mass-input')) {
        $('mass-input').addEventListener('input', () => { 
            currentMass = parseFloat($('mass-input').value) || 70.0; 
            if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        });
        currentMass = parseFloat($('mass-input').value); 
        if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
    }

    if ($('environment-select')) {
        $('environment-select').addEventListener('change', (e) => { 
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

    // --- Gestion des Boutons ---
    if ($('toggle-gps-btn')) {
        $('toggle-gps-btn').addEventListener('click', () => { 
            if (emergencyStopActive) {
                alert("Veuillez désactiver l'Arrêt d'urgence avant d'utiliser ce contrôle.");
                return;
            }
            wID === null ? startFusion() : stopGPS(); 
        });
    }

    if ($('emergency-stop-btn')) {
        $('emergency-stop-btn').addEventListener('click', () => { 
            if (!emergencyStopActive) { emergencyStop(); } else { resumeSystem(); }
        });
    }
    
    // Boutons de Réinitialisation
    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => { distM = 0; timeMoving = 0; if ($('distance-total-km')) $('distance-total-km').textContent = `0.000 km | 0.00 m`; });
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', () => { maxSpd = 0; if ($('speed-max')) $('speed-max').textContent = `0.0 km/h`; });
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { 
        if (confirm("Réinitialiser toutes les données de session et le filtre EKF ?")) { 
            stopGPS(); 
            distM = 0; maxSpd = 0; timeMoving = 0; 
            if ($('distance-total-km')) $('distance-total-km').textContent = `0.000 km | 0.00 m`; 
            if ($('speed-max')) $('speed-max').textContent = `0.0 km/h`; 
        } 
    });
    
    if ($('toggle-mode-btn')) {
        $('toggle-mode-btn').addEventListener('click', () => {
            document.body.classList.toggle('dark-mode');
        });
    }
    
    // --- DÉMARRAGE DU SYSTÈME ---
    
    syncH().then(() => {
        startFusion(); 
    });

    // Boucle de mise à jour lente (Astro/Météo/Horloge)
    setInterval(() => {
        const currentLat = ekf6dof?.lat || lat || 43.296; 
        const currentLon = ekf6dof?.lon || lon || 5.370;
        
        updateAstro(currentLat, currentLon);
        
        // Resynchronise l'horloge NTP toutes les 60 secondes
        if (Math.floor(Date.now() / 1000) % 60 === 0) {
             syncH();
        }
        
        // Récupère la météo (si GPS actif)
        if (currentLat && currentLon && wID !== null) {
            fetchWeather(currentLat, currentLon).then(data => {
                if (data) {
                    lastWeatherData = data;
                    
                    // Met à jour le DOM météo
                    if ($('temp-air-2')) $('temp-air-2').textContent = `${data.tempC.toFixed(1)} °C`;
                    if ($('pressure-2')) $('pressure-2').textContent = `${data.pressure_hPa.toFixed(0)} hPa`;
                    if ($('humidity-2')) $('humidity-2').textContent = `${data.humidity_perc} %`;
                    if ($('air-density')) $('air-density').textContent = `${data.air_density.toFixed(3)} kg/m³`;
                    if ($('dew-point')) $('dew-point').textContent = `${data.dew_point.toFixed(1)} °C`;
                    if ($('speed-sound-local')) $('speed-sound-local').textContent = `${calculateLocalSoundSpeed(data.tempK).toFixed(1)} m/s`;
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
        }
        
    }, DOM_SLOW_UPDATE_MS); 
});
