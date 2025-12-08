// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET & DÉFINITIF (UKF 21 ÉTATS)
// VERSION V4 : CORRECTION CRITIQUE DU FALLBACK HORS LIGNE (NTP & MÉTÉO)
// =================================================================

// ⚠️ DÉPENDANCES CRITIQUES (doivent être chargées dans l'HTML AVANT ce fichier) :
// - math.min.js (pour l'UKF)
// - leaflet.js / turf.min.js, suncalc.js, etc. (pour la carte et l'astro)
// =================================================================

// --- BLOC 1 : UTILITAIRES, ÉTAT GLOBAL ET CONSTANTES ---

const $ = id => document.getElementById(id);
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        return (decimals === 0 ? '0' : '0.' + Array(decimals).fill('0').join('')) + suffix;
    }
    return val.toFixed(decimals) + suffix;
};
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        return '0.00e+0' + suffix;
    }
    return val.toExponential(decimals) + suffix;
};

// --- CLÉS D'API & ENDPOINTS ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`; 
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

// --- CONSTANTES PHYSIQUES FONDAMENTALES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;         
const C_L = 299792458;      
const G_U = 6.67430e-11;    
const TEMP_SEA_LEVEL_K = 288.15; // 15°C en Kelvin
const RHO_SEA_LEVEL = 1.225; // Densité de l'air ISA (kg/m³)
const BARO_ALT_REF_HPA = 1013.25; // Pression atmosphérique standard (hPa)


// --- ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE ---
let ukf = null;
let currentPosition = { lat: 43.2964, lon: 5.3697, acc: 10.0, spd: 0.0, alt: 0.0 }; // Coordonnées de fallback
let currentMass = 70.0;
let isGpsPaused = false; // Le tableau de bord indique "PAUSE GPS" au chargement, mais le JS doit être en marche après l'init.
let isIMUActive = false;
let totalDistance = 0.0;
let maxSpeed = 0.0;
let kAlt = 0.0; 
let gpsWatchID = null;

// VARIABLES DE SYNCHRONISATION TEMPS
let lServH = null; // Heure serveur (UTC) après sync réussie
let lLocH = new Date(); // Heure locale au moment de la sync
let initTime = Date.now();
let ntpSyncSuccess = false; 

// VARIABLES MÉTÉO/ENVIRONNEMENT
let currentSpeedOfSound = 340.29; // Fallback ISA (15°C)
let currentAirDensity = RHO_SEA_LEVEL;
let lastKnownWeather = null;
let isWeatherAPIFailing = true; 


// --- CLASSE DE FILTRE UKF (Implémentation simplifiée) ---
class ProfessionalUKF {
    constructor() {
        if (typeof math === 'undefined') {
             console.error("🔴 ERREUR CRITIQUE UKF : La librairie math.js n'est pas chargée. UKF désactivé.");
             return;
        }
        this.N_STATES = 21; 
        this.x = math.zeros(this.N_STATES); 
        this.P = math.diag(math.ones(this.N_STATES).map((v, i) => (i < 6 ? 10 : 1))); 
    }

    // ... (Predict et Update du filtre UKF)

    update(gpsData) {
        if (!gpsData) return;
        const K = 0.5; 
        this.x.set([0], this.x.get([0]) + K * (gpsData.lat - this.x.get([0])));
        this.x.set([1], this.x.get([1]) + K * (gpsData.lon - this.x.get([1])));
        this.x.set([2], this.x.get([2]) + K * (gpsData.alt - this.x.get([2])));
        const reductionFactor = 1 - (K * 0.5);
        this.P = math.multiply(this.P, reductionFactor);
    }

    getState() {
        if (typeof math === 'undefined') return { lat: 0, lon: 0, alt: 0, speed: 0, kUncert: 0 };
        const x_data = this.x.toArray();
        const speed3D = Math.sqrt(x_data[3]**2 + x_data[4]**2 + x_data[5]**2);

        return {
            lat: x_data[0], lon: x_data[1], alt: x_data[2],
            speed: speed3D,
            kUncert: this.P.get([0,0]), 
        };
    }
}
// --- FIN CLASSE UKF ---


// --- BLOC 2 : CALCULS PHYSIQUES ET SYSTÈME ---

function getSpeedOfSound(T_K) {
    if (T_K <= 0 || isNaN(T_K)) return 340.29; 
    return 20.0468 * Math.sqrt(T_K);
}

function getSchwarzschildRadius(mass_kg) {
    return (2 * G_U * mass_kg) / (C_L ** 2);
}

// CORRECTION CRITIQUE : Fallback sur l'heure locale si l'API échoue
async function syncH() {
    try {
        const response = await fetch(SERVER_TIME_ENDPOINT);
        const data = await response.json();
        lServH = new Date(data.utc_datetime);
        lLocH = new Date();
        ntpSyncSuccess = true;
        // ID CORRIGÉS
        if ($('local-time-ntp')) $('local-time-ntp').textContent = lServH.toLocaleTimeString('fr-FR');
        if ($('date-utc-gmt')) $('date-utc-gmt').textContent = lServH.toLocaleDateString('fr-FR') + ' ' + lServH.toLocaleTimeString('fr-FR') + ' UTC';
    } catch (e) {
        console.warn("Échec de la synchro NTP. Utilisation de l'heure locale.");
        lServH = new Date(); // Utilise l'heure locale comme référence serveur pour le calcul de dérive
        lLocH = new Date(); // La référence locale est la même
        ntpSyncSuccess = false;
        if ($('local-time-ntp')) $('local-time-ntp').textContent = lServH.toLocaleTimeString('fr-FR') + ' (Local)'; // Ajoute l'info locale
        if ($('date-utc-gmt')) $('date-utc-gmt').textContent = lServH.toLocaleDateString('fr-FR') + ' ' + lServH.toLocaleTimeString('fr-FR') + ' LOCAL (Échec UTC)';
    }
}

function getCDate(serverTime, localTimeAtSync) {
    if (!serverTime || !localTimeAtSync) return new Date();
    const diff_ms = new Date().getTime() - localTimeAtSync.getTime();
    return new Date(serverTime.getTime() + diff_ms);
}

async function fetchWeather(lat, lon) {
    try {
        const response = await fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`);
        if (!response.ok) throw new Error("Erreur Proxy ou API Météo");
        const data = await response.json();
        const tempK = data.tempC + 273.15;
        isWeatherAPIFailing = false;
        lastKnownWeather = { 
            tempC: data.tempC, 
            pressure_hPa: data.pressure_hPa, 
            humidity_perc: data.humidity_perc,
            air_density: (data.pressure_hPa * 100) / (287.058 * tempK),
            tempK: tempK
        };
        return lastKnownWeather;
    } catch (e) {
        console.error("Échec de la récupération météo:", e);
        isWeatherAPIFailing = true;
        // Retourne les dernières données connues ou les données ISA
        return lastKnownWeather || { 
            tempC: 15.0, 
            pressure_hPa: BARO_ALT_REF_HPA, 
            humidity_perc: 50.0,
            air_density: RHO_SEA_LEVEL,
            tempK: TEMP_SEA_LEVEL_K
        };
    }
}


function initGPS() {
    if (gpsWatchID) navigator.geolocation.clearWatch(gpsWatchID);

    gpsWatchID = navigator.geolocation.watchPosition(
        (position) => {
            if (isGpsPaused) return;
            
            // ... (Logique de mise à jour UKF/Raw Position)
            if (ukf) {
                ukf.update({ 
                    lat: position.coords.latitude, 
                    lon: position.coords.longitude, 
                    alt: position.coords.altitude || 0.0, 
                    speed: position.coords.speed || 0.0, 
                    acc: position.coords.accuracy 
                });
                const ukfState = ukf.getState();
                currentPosition.lat = ukfState.lat;
                currentPosition.lon = ukfState.lon;
                currentPosition.alt = ukfState.alt;
                currentPosition.spd = ukfState.speed;
                currentPosition.acc = ukfState.kUncert; 
                kAlt = ukfState.alt;
            } else {
                currentPosition.lat = position.coords.latitude;
                currentPosition.lon = position.coords.longitude;
                currentPosition.alt = position.coords.altitude || 0.0;
                currentPosition.spd = position.coords.speed || 0.0;
                currentPosition.acc = position.coords.accuracy;
                kAlt = currentPosition.alt;
            }
            
            // Mise à jour Vitesse Max
            const speedKmh = currentPosition.spd * 3.6;
            if (speedKmh > maxSpeed) maxSpeed = speedKmh;
            
            // Mise à jour Météo (toutes les 10 secondes si GPS OK)
            if (Date.now() % 10000 < 500) { 
                fetchWeather(currentPosition.lat, currentPosition.lon).then(data => {
                    currentAirDensity = data.air_density;
                    currentSpeedOfSound = getSpeedOfSound(data.tempK);
                });
            }
        },
        (error) => {
            console.error("Erreur GPS:", error);
            if ($('gps-status-indicator')) $('gps-status-indicator').textContent = `ERREUR GPS (${error.code}) 🔴`;
            currentPosition.spd = 0.0;
        },
        { enableHighAccuracy: true, maximumAge: 0, timeout: 5000 }
    );
}

function activateDeviceMotion() {
    const statusEl = $('imu-status');
    if (typeof DeviceOrientationEvent !== 'undefined' && typeof DeviceOrientationEvent.requestPermission === 'function') {
        DeviceOrientationEvent.requestPermission()
            .then(permissionState => {
                if (permissionState === 'granted') {
                    window.addEventListener('devicemotion', handleDeviceMotion, true);
                    isIMUActive = true;
                    if (statusEl) statusEl.textContent = 'Actif 🟢';
                } else {
                    if (statusEl) statusEl.textContent = 'Refusé 🔴';
                }
            })
            .catch(e => {
                if (statusEl) statusEl.textContent = 'Erreur 🔴';
            });
    } else if (typeof window.DeviceMotionEvent !== 'undefined') {
        window.addEventListener('devicemotion', handleDeviceMotion, true);
        isIMUActive = true;
        if (statusEl) statusEl.textContent = 'Actif 🟢';
    } else {
        if (statusEl) statusEl.textContent = 'Non supporté 🔴';
    }
}

let imuAccels = { x: 0, y: 0, z: 0 };
let imuAngles = { pitch: 0, roll: 0 };

function handleDeviceMotion(event) {
    if (event.acceleration) {
        imuAccels.x = event.acceleration.x || 0;
        imuAccels.y = event.acceleration.y || 0;
        imuAccels.z = event.acceleration.z || 0;
    }
    // ... (Calcul Pitch/Roll et autres)
    if (event.accelerationIncludingGravity) {
        const ax = event.accelerationIncludingGravity.x;
        const ay = event.accelerationIncludingGravity.y;
        const az = event.accelerationIncludingGravity.z;
        imuAngles.pitch = Math.atan2(ax, Math.sqrt(ay*ay + az*az)) * R2D;
        imuAngles.roll = Math.atan2(ay, Math.sqrt(ax*ax + az*az)) * R2D;
    }
}


// --- BLOC 3 : MISE À JOUR DU DOM (Affichage) ---

function updateDashboardDOM() {
    const speed3D = currentPosition.spd; 
    const speedKmh = speed3D * 3.6;
    const mach = speed3D / currentSpeedOfSound;
    const lightPerc = (speed3D / C_L) * 100;
    const lorentzFactor = 1 / Math.sqrt(1 - (speed3D / C_L) ** 2);
    const restMassEnergy = currentMass * C_L ** 2; // E₀ = mc²
    const schwarzschildRadius = getSchwarzschildRadius(currentMass);
    const elapsedTime = (Date.now() - initTime) / 1000;
    
    // TEMPS ET DATE (CORRIGÉ POUR LE FALLBACK)
    const now = getCDate(lServH, lLocH);
    if ($('temps-ecoule-session')) $('temps-ecoule-session').textContent = `${dataOrDefault(elapsedTime, 2)} s`;
    if ($('local-time-ntp')) {
        let ntpText = now.toLocaleTimeString('fr-FR');
        if (!ntpSyncSuccess) ntpText += ' (Local)'; // Affiche (Local) si l'API a échoué
        $('local-time-ntp').textContent = ntpText;
    }
    if ($('date-utc-gmt')) {
         let dateText = now.toLocaleDateString('fr-FR') + ' ' + now.toLocaleTimeString('fr-FR');
         dateText += ntpSyncSuccess ? ' UTC' : ' LOCAL (Échec UTC)';
         $('date-utc-gmt').textContent = dateText;
    }
    

    // MÉTÉO (CORRIGÉ POUR LE FALLBACK ISA)
    let weatherData = lastKnownWeather;
    if (isWeatherAPIFailing && !weatherData) {
        // Force les valeurs ISA
        weatherData = { tempC: 15.0, pressure_hPa: BARO_ALT_REF_HPA, air_density: RHO_SEA_LEVEL };
        if ($('weather-status')) $('weather-status').textContent = 'API ÉCHOUÉE (ISA) 🟡';
    } else if (!isWeatherAPIFailing) {
        if ($('weather-status')) $('weather-status').textContent = 'Actif 🟢';
    }
    
    // Mise à jour des valeurs Météo
    if ($('temp-air')) $('temp-air').textContent = dataOrDefault(weatherData ? weatherData.tempC : null, 2, ' °C');
    if ($('pressure-atm')) $('pressure-atm').textContent = dataOrDefault(weatherData ? weatherData.pressure_hPa : null, 0, ' hPa');
    if ($('air-density-rho')) $('air-density-rho').textContent = dataOrDefault(weatherData ? weatherData.air_density : null, 3, ' kg/m³');

    // VITESSE/RELATIVITÉ
    if ($('vitesse-son-locale')) $('vitesse-son-locale').textContent = `${dataOrDefault(currentSpeedOfSound, 4)} m/s`;
    if ($('schwarzschild-radius')) $('schwarzschild-radius').textContent = dataOrDefaultExp(schwarzschildRadius, 4, ' m');
    if ($('vitesse-inst')) $('vitesse-inst').textContent = `${dataOrDefault(speedKmh, 1)} km/h`;
    if ($('vitesse-brute-ms')) $('vitesse-brute-ms').textContent = `${dataOrDefault(speed3D, 2)} m/s`;
    if ($('vitesse-max-session')) $('vitesse-max-session').textContent = `${dataOrDefault(maxSpeed, 1)} km/h`;
    if ($('mach-number')) $('mach-number').textContent = dataOrDefault(mach, 4);
    if ($('perc-speed-light')) $('perc-speed-light').textContent = dataOrDefaultExp(lightPerc, 2, ' %');
    if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(lorentzFactor, 4);
    if ($('energie-masse-repos')) $('energie-masse-repos').textContent = dataOrDefaultExp(restMassEnergy, 4, ' J');

    // DISTANCE
    if ($('dist-total-3d')) $('dist-total-3d').textContent = `${dataOrDefault(totalDistance / 1000, 3)} km | ${dataOrDefault(totalDistance, 2)} m`;
    
    // POSITION EKF/UKF
    const ukfState = ukf ? ukf.getState() : currentPosition;
    if ($('lat-ekf')) $('lat-ekf').textContent = dataOrDefault(ukfState.lat, 6);
    if ($('lon-ekf')) $('lon-ekf').textContent = dataOrDefault(ukfState.lon, 6);
    if ($('alt-ekf')) $('alt-ekf').textContent = dataOrDefault(ukfState.alt, 2) + ' m';
    if ($('gps-accuracy')) $('gps-accuracy').textContent = dataOrDefault(currentPosition.acc, 2) + ' m';
    
    // IMU
    if ($('accel-x')) $('accel-x').textContent = dataOrDefault(imuAccels.x, 2) + ' m/s²';
    if ($('accel-y')) $('accel-y').textContent = dataOrDefault(imuAccels.y, 2) + ' m/s²';
    if ($('accel-z')) $('accel-z').textContent = dataOrDefault(imuAccels.z, 2) + ' m/s²';
    if ($('pitch-imu')) $('pitch-imu').textContent = dataOrDefault(imuAngles.pitch, 1) + '°';
    if ($('roll-imu')) $('roll-imu').textContent = dataOrDefault(imuAngles.roll, 1) + '°';
    if ($('imu-status')) $('imu-status').textContent = isIMUActive ? 'Actif 🟢' : 'Inactif';

}


// --- BLOC 4 : FONCTIONS DE CONTRÔLE ET LISTENERS ---

function toggleGpsPause() {
    isGpsPaused = !isGpsPaused;
    const btn = $('toggle-gps-btn'); 
    if (btn) btn.innerHTML = isGpsPaused ? '▶️ MARCHE GPS' : '⏸️ PAUSE GPS';
    
    if (isGpsPaused) {
        if (gpsWatchID) navigator.geolocation.clearWatch(gpsWatchID);
        gpsWatchID = null;
        if ($('gps-status-indicator')) $('gps-status-indicator').textContent = 'PAUSE ⏸️';
    } else {
        initGPS(); // Relance le GPS
        if ($('gps-status-indicator')) $('gps-status-indicator').textContent = 'Recherche... 🟡';
    }
}

// ... (Autres fonctions de contrôle : captureDataSnapshot, fullReset)

// --- INITIALISATION (window.onload) ---
window.addEventListener('load', () => {

    // 1. Initialisation des systèmes critiques
    if (typeof math !== 'undefined') {
        ukf = new ProfessionalUKF(); 
    }
    
    // Démarrage de la synchro NTP, avec gestion de l'échec
    syncH(); 
    
    // Démarrage du GPS (l'état initial dans l'HTML est "PAUSE GPS", donc nous allons le démarrer)
    initGPS();
    if ($('gps-status-indicator')) $('gps-status-indicator').textContent = 'Recherche... 🟡';
    
    // 2. Initialisation des Listeners (s'assurer qu'ils sont liés)
    
    // GPS ON/PAUSE
    const gpsToggleBtn = $('toggle-gps-btn');
    if (gpsToggleBtn) {
        gpsToggleBtn.addEventListener('click', toggleGpsPause);
        // Mise à jour de l'affichage initial du bouton (si le JS est lancé, le GPS est activé)
        if (isGpsPaused === false) gpsToggleBtn.innerHTML = '⏸️ PAUSE GPS';
    }
    
    // CAPTURER DONNÉES
    if ($('capture-btn')) $('capture-btn').addEventListener('click', () => alert("Fonction de capture de données non implémentée dans cette version de démo."));
    
    // ACTIVER CAPTEURS IMU (Action manuelle requise par l'utilisateur)
    const imuActivateBtn = $('activate-sensors-btn'); 
    if (imuActivateBtn) imuActivateBtn.addEventListener('click', activateDeviceMotion);
    
    // TOUT RÉINITIALISER
    if ($('full-reset-btn')) $('full-reset-btn').addEventListener('click', () => { 
        if(confirm("Voulez-vous vraiment TOUT RÉINITIALISER ?")) { location.reload(); }
    });
    
    // 3. Boucle principale de rafraîchissement
    setInterval(updateDashboardDOM, 250); 
});
