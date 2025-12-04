// =================================================================
// BLOC 1/4 : CONSTANTES, ÉTAT GLOBAL, UTILITAIRES ET CLASSE UKF
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);
const D2R = Math.PI / 180, R2D = 180 / Math.PI; 
const KMH_MS = 3.6;         
const C_L = 299792458;      

const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        return (decimals === 0 ? '0' : ('0.' + Array(decimals).fill('0').join(''))) + suffix;
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

// --- CONSTANTES WGS84 et GÉOPHYSIQUES ---
const WGS84_A = 6378137.0;        
const WGS84_F = 1 / 298.257223563; 
const WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F; 
const WGS84_G_EQUATOR = 9.780327; 
const WGS84_BETA = 0.0053024; 
const RHO_SEA_LEVEL = 1.225; 
const TEMP_SEA_LEVEL_K = 288.15; 
const BARO_ALT_REF_HPA = 1013.25; 
let G_ACC = 9.80665; // Gravité locale (m/s²)

// --- ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE ---
let map = null;
let gpsWatchID = null;
let gpsIsActive = false;
let lastTimestamp = 0;
let lastServerTime = null;
let lastLocalTime = null;
let currentLat = 43.2964, currentLon = 5.3697, kAlt = 0; // Position initiale
let currentSpeed = 0.0;
let maxSpeed = 0;
let totalDistance = 0;
let lastUKFPosition = { lat: currentLat, lon: currentLon, alt: kAlt }; // CRITIQUE pour la distance 3D
let lastPolyline = null;
let lastPoint = null;
let lastKnownIMU = { ax: 0, ay: 0, az: 0, gx: 0, gy: 0, gz: 0, heading: 0 };
let ukf = null; 
let currentCelestialBody = 'EARTH'; 
let currentMass = 70.0; 
let currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound = 340.29; 
let currentUKFReactivity = 'NORMAL'; 
let netherMode = false;

const GPS_OPTIONS = { HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 } };

// --- CLASSE PROFESSIONALUKF (21 ÉTATS) ---
class ProfessionalUKF {
    constructor(nx = 21, nz = 6) {
        this.X = math.zeros(nx, 1);
        this.X.set([0, 0], currentLat * D2R);
        this.X.set([1, 0], currentLon * D2R);
        this.X.set([2, 0], -kAlt); // Z vers le bas (Convention NED)
        // Simplification: P (matrice de covariance) omise, mais nécessaire en production.
        this.P = math.diag(Array(nx).fill(1e-2));
    }
    
    predict(dt, imuRaw) { 
        if (dt <= 0) return;
        const Accel_N = imuRaw.ax; 
        const Accel_E = imuRaw.ay;
        const Accel_D = G_ACC - imuRaw.az; 
        
        // Propagation de la vitesse V
        this.X.set([3, 0], this.X.get([3, 0]) + Accel_N * dt);
        this.X.set([4, 0], this.X.get([4, 0]) + Accel_E * dt);
        this.X.set([5, 0], this.X.get([5, 0]) + Accel_D * dt);
        
        // Propagation de la position P
        this.X.set([0, 0], this.X.get([0, 0]) + this.X.get([3, 0]) * dt);
        this.X.set([1, 0], this.X.get([1, 0]) + this.X.get([4, 0]) * dt);
        this.X.set([2, 0], this.X.get([2, 0]) + this.X.get([5, 0]) * dt);
    }
    
    update(z_meas_ned, gps_accuracy) {
        const K_P_pos = Math.min(0.5, 10 / (gps_accuracy + 1e-6)); 
        this.X.set([0, 0], this.X.get([0, 0]) * (1 - K_P_pos) + z_meas_ned.get([0, 0]) * K_P_pos);
        this.X.set([1, 0], this.X.get([1, 0]) * (1 - K_P_pos) + z_meas_ned.get([1, 0]) * K_P_pos);
        this.X.set([2, 0], this.X.get([2, 0]) * (1 - K_P_pos) + z_meas_ned.get([2, 0]) * K_P_pos);
    }

    getFilteredState() {
        const latRad = this.X.get([0, 0]);
        const lonRad = this.X.get([1, 0]);
        const altM = -this.X.get([2, 0]);
        const V_N = this.X.get([3, 0]); 
        const V_E = this.X.get([4, 0]); 
        const speed = Math.sqrt(V_N**2 + V_E**2); // Vitesse horizontale
        const heading = V_N !== 0 || V_E !== 0 ? (Math.atan2(V_E, V_N) * R2D + 360) % 360 : 0;
        return {
            lat: latRad * R2D, lon: lonRad * R2D, alt: altM, speed: speed, heading: heading
        };
    }
        }
// =================================================================
// BLOC 2/4 : MODÈLES PHYSIQUES ET ENVIRONNEMENTAUX (ISA, RELATIVITÉ, API)
// =================================================================

function calculateWGS84Gravity(latRad, altM) {
    const sin2 = Math.sin(latRad) ** 2;
    const g_lat = WGS84_G_EQUATOR * (1 + WGS84_BETA * sin2) / Math.sqrt(1 - WGS84_E2 * sin2); 
    const R_LAT = WGS84_A / Math.sqrt(1 - WGS84_E2 * sin2);
    return g_lat * (1 - 2 * altM / R_LAT);
}

function updateCelestialBody(body, altM, rotR = 100, angV = 0.0) {
    if (body === 'EARTH') {
        const latRad = currentLat * D2R;
        G_ACC = calculateWGS84Gravity(latRad, altM);
    } else if (body === 'ROTATING') {
        const g_base = 9.80665;
        const F_CENTRIFUGE = rotR * (angV**2);
        G_ACC = g_base - F_CENTRIFUGE;
    } else {
        G_ACC = 9.80665; 
    }
    return { G_ACC_NEW: G_ACC };
}

function calculateStandardISA(altM) {
    if (altM < 11000) { 
        const T_K = TEMP_SEA_LEVEL_K - 0.0065 * altM;
        const P_Pa = 101325 * Math.pow(T_K / TEMP_SEA_LEVEL_K, 5.256); 
        return { T_K, P_Pa, tempC: T_K - 273.15, pressure_hPa: P_Pa / 100, humidity_perc: 0.0 };
    }
    return { T_K: TEMP_SEA_LEVEL_K, P_Pa: 101325, tempC: 15.0, pressure_hPa: 1013.25, humidity_perc: 0.0 };
}

function getAirDensity(tempK, pressure_hPa) {
    const pressure_Pa = pressure_hPa * 100;
    const R_SPECIFIC_AIR = 287.058;
    return pressure_Pa / (R_SPECIFIC_AIR * tempK);
}

function getSpeedOfSound(tempK) {
    const GAMMA_AIR = 1.4; 
    const R_SPECIFIC_AIR = 287.058;
    return Math.sqrt(GAMMA_AIR * R_SPECIFIC_AIR * tempK);
}

function calculateRelativityEffects(speed, altM) {
    const v_c_ratio = speed / C_L;
    const kinematic_dilation = 1.0 / Math.sqrt(1.0 - v_c_ratio * v_c_ratio);
    const G_U = 6.67430e-11; 
    const M_EARTH = 5.972e24; 
    const R_E_BASE = 6371000;
    const gravitational_dilation = 1.0 + (G_U * M_EARTH / (R_E_BASE * C_L**2) - G_U * M_EARTH / ((R_E_BASE + altM) * C_L**2));
    return { kinematic_dilation: kinematic_dilation, gravitational_dilation: gravitational_dilation };
}

function getCDate(serverTime, localTime) {
    if (serverTime && localTime) {
        const offset = serverTime.getTime() - localTime.getTime();
        return new Date(Date.now() + offset);
    }
    return new Date();
}

const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";
async function syncH() {
    const statusEl = $('local-time');
    if (statusEl) statusEl.textContent = 'Synchronisation...';
    try {
        const response = await fetch(SERVER_TIME_ENDPOINT);
        if (!response.ok) throw new Error('Échec de l\'API Time');
        const data = await response.json();
        lastServerTime = new Date(data.utc_datetime);
        lastLocalTime = new Date();
        if (statusEl) statusEl.textContent = lastLocalTime.toLocaleTimeString('fr-FR');
    } catch (error) {
        if (statusEl) statusEl.textContent = 'SYNCHRO ÉCHOUÉE (Local Time)';
    }
}

// NOTE: fetchWeather et updateWeatherDOM ont été omises ici pour la concision (API externe).
// Veuillez vous assurer qu'elles sont incluses dans votre fichier complet.
function updateWeatherDOM(data, isOffline = false) {
    // Remplacer par la logique d'affichage de la météo
    if ($('temp-air-2')) $('temp-air-2').textContent = `${data.tempC.toFixed(1)} °C`;
    if ($('pressure-2')) $('pressure-2').textContent = `${data.pressure_hPa.toFixed(0)} hPa`;
    if ($('air-density')) $('air-density').textContent = `${currentAirDensity.toFixed(3)} kg/m³`;
                                                                                }
// =================================================================
// BLOC 3/4 : ACQUISITION CAPTEURS, IMU ET CONTRÔLE SYSTÈME (CRITIQUE)
// =================================================================

// --- GESTION IMU (Device Motion) ---

function handleDeviceMotion(event) {
    const accel = event.accelerationIncludingGravity || event.acceleration; 
    const gyro = event.rotationRate;
    
    if (accel && gyro) {
        lastKnownIMU.ax = accel.x || 0;
        lastKnownIMU.ay = accel.y || 0;
        lastKnownIMU.az = accel.z || 0;
        lastKnownIMU.gx = gyro.alpha || 0;
        lastKnownIMU.gy = gyro.beta || 0;
        lastKnownIMU.gz = gyro.gamma || 0;
        
        // Mise à jour de l'affichage IMU brut
        if ($('acceleration-x')) $('acceleration-x').textContent = dataOrDefault(lastKnownIMU.ax, 2, ' m/s²');
        if ($('acceleration-y')) $('acceleration-y').textContent = dataOrDefault(lastKnownIMU.ay, 2, ' m/s²');
        if ($('acceleration-z')) $('acceleration-z').textContent = dataOrDefault(lastKnownIMU.az, 2, ' m/s²');
    }
}

function handleDeviceOrientation(event) {
    const alpha = event.alpha || 0; 
    lastKnownIMU.heading = (360 - alpha + 90) % 360; 
}

function startIMUSensors() {
    let sensorStatus = $('statut-capteur');
    if (!sensorStatus) return;
    
    // CRITIQUE : Gestion des permissions iOS 13+ (doit être déclenchée par un clic)
    if (typeof DeviceMotionEvent.requestPermission === 'function') {
        sensorStatus.textContent = 'En attente d\'autorisation IMU... (iOS)';
        DeviceMotionEvent.requestPermission().then(permissionState => {
            if (permissionState === 'granted') {
                window.addEventListener('devicemotion', handleDeviceMotion);
                window.addEventListener('deviceorientation', handleDeviceOrientation);
                sensorStatus.textContent = 'Actif (IMU/Fusion)';
            } else {
                sensorStatus.textContent = '❌ IMU: Refusé';
            }
        }).catch(error => {
             sensorStatus.textContent = `❌ IMU: Erreur (${error.name})`;
        });
    } else if (window.DeviceMotionEvent) {
        // Environnement Hérité (Android/Legacy)
        window.addEventListener('devicemotion', handleDeviceMotion);
        window.addEventListener('deviceorientation', handleDeviceOrientation);
        sensorStatus.textContent = 'Actif (IMU/Fusion)';
    } else {
        sensorStatus.textContent = '❌ IMU: Non supporté';
    }
}

// --- GESTION GPS (Geolocation API) ---

function startGPS() {
    const statusElement = $('statut-gps-acquisition');
    if (!('geolocation' in navigator)) {
        if (statusElement) statusElement.textContent = '❌ ERREUR: Geolocation non supportée.';
        return;
    }
    if (statusElement) statusElement.textContent = 'Acquisition GPS en cours...';

    gpsWatchID = navigator.geolocation.watchPosition(
        (position) => {
            const { latitude, longitude, altitude, accuracy } = position.coords;
            
            // Mise à jour de l'UKF avec les mesures GPS brutes
            const newAlt = altitude !== null ? altitude : kAlt;
            if (ukf) ukf.update(math.matrix([[latitude * D2R], [longitude * D2R], [-newAlt]]), accuracy);

            // Mise à jour de la carte (pour le tracé brut)
            if (map && lastPolyline) {
                lastPolyline.addLatLng(L.latLng(latitude, longitude));
            }

            // Mise à jour du DOM
            if (statusElement) statusElement.textContent = `ACTIF (${dataOrDefault(accuracy, 1)}m)`;
            if ($('précision-gps-acc')) $('précision-gps-acc').textContent = `${dataOrDefault(accuracy, 1)} m`;

        }, 
        (error) => {
            let msg = '';
            if (error.code === error.PERMISSION_DENIED) {
                msg = "❌ ERREUR: Accès GPS refusé. (HTTPS/Localhost requis)";
            } else if (error.code === error.TIMEOUT) {
                msg = "⌛ ERREUR: Délai dépassé. (Mauvais signal)";
            } else {
                msg = `❌ ERREUR GPS: ${error.message}`;
            }
            if (statusElement) statusElement.textContent = msg;
        }, 
        GPS_OPTIONS.HIGH_FREQ
    );
}

function stopSystem() {
    if (gpsWatchID !== null) {
        navigator.geolocation.clearWatch(gpsWatchID);
        gpsWatchID = null;
    }
    window.removeEventListener('devicemotion', handleDeviceMotion);
    window.removeEventListener('deviceorientation', handleDeviceOrientation);
    gpsIsActive = false;
    if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = 'Inactif';
    if ($('statut-capteur')) $('statut-capteur').textContent = 'Inactif';
}
// =================================================================
// BLOC 4/4 : BOUCLES DE MISE À JOUR, CARTE & INITIALISATION FINALE
// =================================================================

function initMap() {
    if (typeof L === 'undefined') {
        if ($('map')) $('map').textContent = "Erreur: Leaflet n'est pas disponible.";
        return;
    }
    if (!map) {
        map = L.map('map', { center: [currentLat, currentLon], zoom: 13, layers: [ L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19 }) ] });
        lastPolyline = L.polyline([], {color: '#007bff', weight: 3}).addTo(map);
    }
}

function updateAstro(lat, lon) {
    if (typeof SunCalc === 'undefined') return;
    const now = getCDate(lastServerTime, lastLocalTime);
    if (!now) return;
    const pos = SunCalc.getPosition(now, lat, lon);
    // Mise à jour de l'affichage Astro
    if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(pos.altitude * R2D, 2, '°');
    if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(pos.azimuth * R2D, 2, '°');
}

const DOM_SLOW_UPDATE_MS = 2000; 
function startSlowLoop() {
    if (window.slowLoopRunning) return;
    window.slowLoopRunning = true;
    
    setInterval(() => {
        updateAstro(currentLat, currentLon); 
        // NOTE: fetchWeather est omise, le bloc ISA est exécuté par défaut pour les valeurs météo
        const isa = calculateStandardISA(kAlt);
        currentAirDensity = getAirDensity(isa.T_K, isa.pressure_hPa);
        currentSpeedOfSound = getSpeedOfSound(isa.T_K);
        updateWeatherDOM({ ...isa, tempC: isa.tempC, pressure_hPa: isa.pressure_hPa }, true);
        
        // Mise à jour de l'heure
        const now = getCDate(lastServerTime, lastLocalTime);
        if (now) {
            if ($('local-time') && !$('local-time').textContent.includes('SYNCHRO ÉCHOUÉE')) {
                $('local-time').textContent = now.toLocaleTimeString('fr-FR');
            }
            if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
        }
        
        // Mise à jour de la gravité (lente)
        const { G_ACC_NEW } = updateCelestialBody(currentCelestialBody, kAlt);
        if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
        
    }, DOM_SLOW_UPDATE_MS); 
}

/** Boucle Rapide (Animation Frame) : Propagation UKF et Mise à jour du DOM/Carte (Haute Fréquence). */
function startFastLoop() {
    if (window.fastLoopRunning) return;
    window.fastLoopRunning = true;
    let lastTime = performance.now();
    
    function fastLoop(time) {
        const now = performance.now();
        const dt = (now - lastTime) / 1000.0;
        lastTime = now;
        
        if (gpsIsActive && ukf && dt > 0) { 
            // 1. Prédiction UKF 
            ukf.predict(dt, lastKnownIMU); 
            const filteredState = ukf.getFilteredState();
            
            // 2. Calcul de la distance 3D et Mise à jour de l'état
            const oldLat = lastUKFPosition.lat; 
            const oldLon = lastUKFPosition.lon;
            const oldAlt = lastUKFPosition.alt;

            currentLat = filteredState.lat;
            currentLon = filteredState.lon;
            kAlt = filteredState.alt;
            
            // Vitesse 3D complète (N, E, D)
            const V_N = ukf.X.get([3, 0]); 
            const V_E = ukf.X.get([4, 0]); 
            const V_D = ukf.X.get([5, 0]); 
            const speed3D = Math.sqrt(V_N**2 + V_E**2 + V_D**2); 
            currentSpeed = speed3D; 
            const speedKmh = currentSpeed * KMH_MS; 
            maxSpeed = Math.max(maxSpeed, speedKmh);

            // CALCUL DE LA DISTANCE 3D HAUTE FRÉQUENCE (Cohérence avec Vitesse 3D)
            const dLat = (currentLat - oldLat) * D2R * WGS84_A; 
            const dLon = (currentLon - oldLon) * D2R * WGS84_A * Math.cos(currentLat * D2R);
            const dAlt = kAlt - oldAlt; 
            
            const distance3D_frame = Math.sqrt(dLat**2 + dLon**2 + dAlt**2);
            totalDistance += distance3D_frame;
            
            // Mise à jour de la dernière position UKF pour la prochaine trame
            lastUKFPosition = { lat: currentLat, lon: currentLon, alt: kAlt };
            
            // 3. Mise à jour des affichages
            if ($('speed-current-kmh')) $('speed-current-kmh').textContent = dataOrDefault(speedKmh, 5, ' km/h');
            if ($('speed-max')) $('speed-max').textContent = dataOrDefault(maxSpeed, 5, ' km/h');
            if ($('distance-total-km')) $('distance-total-km').textContent = `${dataOrDefault(totalDistance / 1000, 3)} km | ${dataOrDefault(totalDistance, 2)} m`;
            if ($('current-lat')) $('current-lat').textContent = dataOrDefault(currentLat, 6);
            if ($('current-lon')) $('current-lon').textContent = dataOrDefault(currentLon, 6);
            if ($('current-alt')) $('current-alt').textContent = dataOrDefault(kAlt, 2, ' m');
            
            const relEffects = calculateRelativityEffects(currentSpeed, kAlt);
            if ($('kinematic-dilation')) $('kinematic-dilation').textContent = dataOrDefaultExp(relEffects.kinematic_dilation - 1, 10, ' s/s');
            if ($('gravitational-dilation')) $('gravitational-dilation').textContent = dataOrDefaultExp(relEffects.gravitational_dilation - 1, 10, ' s/s');

            if (map) map.setView(L.latLng(currentLat, currentLon));
            if ($('statut-ekf-fusion')) $('statut-ekf-fusion').textContent = `ACTIF (${dataOrDefault(dt*1000, 1)}ms)`;

        }
        requestAnimationFrame(fastLoop);
    }
    requestAnimationFrame(fastLoop);
}

/** Gestionnaire principal d'activation/désactivation du système (CRITIQUE) */
function handleToggleGps() {
    console.log("handleToggleGps: Clic détecté. Lancement de la procédure de démarrage/arrêt.");
    const btn = $('toggle-gps-btn');
    
    if (gpsIsActive) {
        stopSystem();
        if (btn) btn.textContent = '▶️ MARCHE GPS';
        if ($('statut-ekf-fusion')) $('statut-ekf-fusion').textContent = `PAUSE`;
    } else {
        // Le clic est la condition nécessaire pour les permissions (surtout IMU iOS)
        startIMUSensors(); 
        startGPS(); 

        if (btn) btn.textContent = '⏸️ PAUSE GPS/IMU';
        gpsIsActive = true;
        
        if (ukf === null && typeof ProfessionalUKF !== 'undefined') {
             ukf = new ProfessionalUKF();
        }
        
        // Démarrage du moteur haute fréquence (UKF)
        if (!window.fastLoopRunning) startFastLoop();
        
        if ($('statut-ekf-fusion')) $('statut-ekf-fusion').textContent = `INITIALISATION EKF...`;
    }
}

// Point d'entrée principal (garanti d'être exécuté après le chargement du DOM)
document.addEventListener('DOMContentLoaded', () => {
    
    console.log("DÉBOGAGE: 1. DOMContentLoaded déclenché. Début de l'initialisation du script.");

    try {
        // 1. Vérification des dépendances (CRITIQUE)
        if (typeof math === 'undefined' || typeof L === 'undefined' || typeof SunCalc === 'undefined' || typeof turf === 'undefined') {
            const statusEl = $('statut-gps-acquisition') || document.body;
            statusEl.innerHTML = '<h2 style="color:red;">CRASH: Dépendances manquantes.</h2>';
            return;
        }

        // 2. Initialisation statique
        initMap();
        syncH(); 
        startSlowLoop(); 
        updateCelestialBody(currentCelestialBody, kAlt); 
        
        if (ukf === null) ukf = new ProfessionalUKF();
        
        // 3. ATTACHEMENT CRITIQUE DU BOUTON MARCHE GPS
        const toggleBtn = $('toggle-gps-btn');
        if (toggleBtn) {
            toggleBtn.addEventListener('click', handleToggleGps);
            console.log("DÉBOGAGE: 2. Écouteur de clic attaché à 'toggle-gps-btn'.");
        } else {
            if ($('statut-gps-acquisition')) {
                $('statut-gps-acquisition').textContent = '❌ Erreur: Bouton Démarrer (ID: toggle-gps-btn) manquant.';
            }
        }
        
        // 4. Initialisation des autres contrôles (Reset, Mode Nuit, etc.)
        if ($('toggle-mode-btn')) $('toggle-mode-btn').addEventListener('click', () => { 
            document.body.classList.toggle('dark-mode'); 
            $('toggle-mode-btn').innerHTML = document.body.classList.contains('dark-mode') ? '<i class="fas fa-sun"></i> Mode Jour' : '<i class="fas fa-moon"></i> Mode Nuit';
        });

        if ($('reset-dashboard-btn')) $('reset-dashboard-btn').addEventListener('click', () => { 
            if (confirm("Voulez-vous vraiment réinitialiser le tableau de bord ?")) {
                stopSystem();
                totalDistance = 0; maxSpeed = 0; lastUKFPosition = { lat: currentLat, lon: currentLon, alt: kAlt };
                if (map) { 
                    map.removeLayer(lastPolyline); 
                    lastPolyline = L.polyline([], {color: '#007bff', weight: 3}).addTo(map);
                }
                if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = '▶️ MARCHE GPS';
                if ($('distance-total-km')) $('distance-total-km').textContent = `0.000 km | 0.00 m`; 
                if ($('speed-max')) $('speed-max').textContent = `0.00000 km/h`; 
            } 
        });
        
        // Mise à jour des affichages par défaut
        if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC.toFixed(4)} m/s² (WGS84)`;
        if ($('statut-capteur')) $('statut-capteur').textContent = `Inactif`;
        if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = `Prêt (HTTPS/Localhost requis)`;


    } catch (error) {
        console.error("DÉBOGAGE: ERREUR D'INITIALISATION MAJEURE (Avant le clic):", error);
        const statusEl = $('statut-gps-acquisition') || document.body;
        statusEl.innerHTML = `<h2 style="color:red;">CRASH INITIAL: ${error.name}</h2><p>Le script a planté avant le démarrage.</p>`;
    }
});
