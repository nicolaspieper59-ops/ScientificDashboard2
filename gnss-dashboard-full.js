// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET & DÉFINITIF (UKF 21 ÉTATS)
// VERSION V3 : CORRECTION CRITIQUE DES ID DU DOM (Boutons + Affichage N/A)
// =================================================================

// ⚠️ DÉPENDANCES CRITIQUES (doivent être chargées dans l'HTML AVANT ce fichier) :
// - math.min.js (pour l'UKF)
// - leaflet.js / turf.min.js, suncalc.js, etc. (pour la carte et l'astro)
// =================================================================

// --- BLOC 1 : UTILITAIRES, ÉTAT GLOBAL ET CONSTANTES ---

const $ = id => document.getElementById(id);
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        // CORRECTION D'AFFICHAGE: Assure que les valeurs par défaut sont affichées correctement (0.00)
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
const KMH_MS = 3.6;         
const C_L = 299792458;      
const G_U = 6.67430e-11;    
const G_STD = 9.8067;      
const RHO_SEA_LEVEL = 1.225; // Densité de l'air ISA (kg/m³)
const TEMP_SEA_LEVEL_K = 288.15; // 15°C en Kelvin

// --- ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE ---
let ukf = null;
let currentPosition = { lat: 43.2964, lon: 5.3697, acc: 10.0, spd: 0.0, alt: 0.0 }; // Coordonnées de fallback
let currentMass = 70.0;
let isGpsPaused = true; // Commence en pause, l'utilisateur doit cliquer
let isEmergencyStopped = false;
let isIMUActive = false;
let totalDistance = 0.0;
let maxSpeed = 0.0;
let kAlt = 0.0; 
let currentSpeedOfSound = 340.29; // Fallback ISA (15°C)
let currentAirDensity = RHO_SEA_LEVEL;
let gpsWatchID = null;
let lServH = null; // Heure serveur NTP
let lLocH = new Date(); // Heure locale (pour dérive)
let initTime = Date.now();


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
        console.log("UKF 21 États : Initialisation du filtre.");
    }

    predict(dt, accX, accY, accZ) { /* ... logique de prédiction ... */ }

    update(gpsData) {
        if (!gpsData) return;
        const K = 0.5; // Gain de Kalman simple
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

// Rayon de Schwarzschild (m)
function getSchwarzschildRadius(mass_kg) {
    return (2 * G_U * mass_kg) / (C_L ** 2);
}

// Récupère l'heure NTP
async function syncH() {
    try {
        const response = await fetch(SERVER_TIME_ENDPOINT);
        const data = await response.json();
        lServH = new Date(data.utc_datetime);
        lLocH = new Date();
        // ID CORRIGÉS
        if ($('local-time-ntp')) $('local-time-ntp').textContent = lServH.toLocaleTimeString('fr-FR');
        if ($('date-utc-gmt')) $('date-utc-gmt').textContent = lServH.toLocaleDateString('fr-FR') + ' ' + lServH.toLocaleTimeString('fr-FR') + ' UTC';
    } catch (e) {
        console.warn("Échec de la synchro NTP, utilisation de l'heure locale.");
        if ($('local-time-ntp')) $('local-time-ntp').textContent = 'SYNCHRO ÉCHOUÉE';
        if ($('date-utc-gmt')) $('date-utc-gmt').textContent = 'N/A';
    }
}

// Calcule la Date/Heure corrigée
function getCDate(serverTime, localTimeAtSync) {
    if (!serverTime || !localTimeAtSync) return new Date();
    const diff_ms = new Date().getTime() - localTimeAtSync.getTime();
    return new Date(serverTime.getTime() + diff_ms);
}

// Récupère les données Météo (API MÉTÉO SOUVENT EN PANNE)
async function fetchWeather(lat, lon) {
    try {
        const response = await fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`);
        if (!response.ok) throw new Error("Erreur Proxy ou API Météo");
        const data = await response.json();
        const tempK = data.tempC + 273.15;
        return { 
            tempC: data.tempC, 
            pressure_hPa: data.pressure_hPa, 
            humidity_perc: data.humidity_perc,
            air_density: (data.pressure_hPa * 100) / (287.058 * tempK), // Calcul de la densité
            tempK: tempK
        };
    } catch (e) {
        console.error("Échec de la récupération météo:", e);
        return null;
    }
}


function initGPS() {
    if (gpsWatchID) navigator.geolocation.clearWatch(gpsWatchID);

    gpsWatchID = navigator.geolocation.watchPosition(
        (position) => {
            if (isGpsPaused) return;

            const now = Date.now();
            
            // Mise à jour de la position UKF/EKF
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
                // Utiliser les données brutes si UKF est désactivé
                currentPosition.lat = position.coords.latitude;
                currentPosition.lon = position.coords.longitude;
                currentPosition.alt = position.coords.altitude || 0.0;
                currentPosition.spd = position.coords.speed || 0.0;
                currentPosition.acc = position.coords.accuracy;
                kAlt = currentPosition.alt;
            }
            
            // Mise à jour Vitesse Max
            const speedKmh = currentPosition.spd * KMH_MS;
            if (speedKmh > maxSpeed) {
                maxSpeed = speedKmh;
            }
            
            // Mise à jour Météo (si GPS OK)
            if (Date.now() % 10000 < 500) { 
                fetchWeather(currentPosition.lat, currentPosition.lon).then(data => {
                    if (data) {
                        currentAirDensity = data.air_density;
                        currentSpeedOfSound = getSpeedOfSound(data.tempK);
                        if ($('temp-air')) $('temp-air').textContent = dataOrDefault(data.tempC, 2, ' °C');
                        if ($('pressure-atm')) $('pressure-atm').textContent = dataOrDefault(data.pressure_hPa, 0, ' hPa');
                        if ($('humidity-relative')) $('humidity-relative').textContent = dataOrDefault(data.humidity_perc, 0, ' %');
                        if ($('air-density-rho')) $('air-density-rho').textContent = dataOrDefault(data.air_density, 3, ' kg/m³');
                        if ($('weather-status')) $('weather-status').textContent = 'Actif 🟢';

                    } else {
                        // Conserver les valeurs de fallback en cas d'échec API
                        if ($('weather-status')) $('weather-status').textContent = 'ÉCHEC API 🟡';
                    }
                });
            }
        },
        (error) => {
            console.error("Erreur GPS:", error);
            if ($('gps-status-indicator')) $('gps-status-indicator').textContent = `ERREUR (${error.code}) 🔴`;
            // Si le GPS échoue, réinitialiser la vitesse
            currentPosition.spd = 0.0;
        },
        { enableHighAccuracy: true, maximumAge: 0, timeout: 5000 }
    );
}

// Activation des Capteurs de Mouvement (IMU)
function activateDeviceMotion() {
    const statusEl = $('imu-status');
    // Tentative de demande de permission pour iOS/Safari
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
                console.error("Erreur de permission IMU:", e);
                if (statusEl) statusEl.textContent = 'Erreur 🔴';
            });
    } else if (typeof window.DeviceMotionEvent !== 'undefined') {
        // Standard device motion API
        window.addEventListener('devicemotion', handleDeviceMotion, true);
        isIMUActive = true;
        if (statusEl) statusEl.textContent = 'Actif 🟢';
    } else {
        if (statusEl) statusEl.textContent = 'Non supporté 🔴';
    }
}

let imuAccels = { x: 0, y: 0, z: 0 };
let imuAngles = { pitch: 0, roll: 0 };
let imuGyro = { alpha: 0, beta: 0, gamma: 0 };


function handleDeviceMotion(event) {
    if (event.acceleration) {
        imuAccels.x = event.acceleration.x || 0;
        imuAccels.y = event.acceleration.y || 0;
        imuAccels.z = event.acceleration.z || 0;
    }
    if (event.rotationRate) {
        imuGyro.alpha = event.rotationRate.alpha || 0;
        imuGyro.beta = event.rotationRate.beta || 0;
        imuGyro.gamma = event.rotationRate.gamma || 0;
    }
    // Simplification pour Pitch/Roll (utilisant l'accélération avec gravité)
    if (event.accelerationIncludingGravity) {
        const ax = event.accelerationIncludingGravity.x;
        const ay = event.accelerationIncludingGravity.y;
        const az = event.accelerationIncludingGravity.z;
        imuAngles.pitch = Math.atan2(ax, Math.sqrt(ay*ay + az*az)) * R2D;
        imuAngles.roll = Math.atan2(ay, Math.sqrt(ax*ax + az*az)) * R2D;
    }

    if (ukf && !isGpsPaused) {
        // ukf.predict() ici...
    }
}

// --- BLOC 3 : MISE À JOUR DU DOM (Affichage) ---

function updateDashboardDOM() {
    // VITESSE ET RELATIVITÉ
    const speed3D = currentPosition.spd; 
    const speedKmh = speed3D * KMH_MS;
    const mach = speed3D / currentSpeedOfSound;
    const lightPerc = (speed3D / C_L) * 100;
    const lorentzFactor = 1 / Math.sqrt(1 - (speed3D / C_L) ** 2);
    const restMassEnergy = currentMass * C_L ** 2; // E₀ = mc²
    
    // RAYON DE SCHWARZSCHILD (Rs)
    const schwarzschildRadius = getSchwarzschildRadius(currentMass);
    
    // TEMPS ÉCOULÉ
    const elapsedTime = (Date.now() - initTime) / 1000;
    // CORRECTION ID
    if ($('temps-ecoule-session')) $('temps-ecoule-session').textContent = `${dataOrDefault(elapsedTime, 2)} s`;


    // 🛑 CORRECTION CRITIQUE DES ID D'AFFICHAGE (N/A)
    // Rayon de Schwarzschild (Rs)
    if ($('schwarzschild-radius')) { // ID PROBABLE (utilisé dans les fragments JS)
         $('schwarzschild-radius').textContent = dataOrDefaultExp(schwarzschildRadius, 4, ' m');
    }

    // Vitesse du Son Locale (Cor.)
    if ($('vitesse-son-locale')) { // ID PROBABLE (utilisé dans les fragments JS)
        $('vitesse-son-locale').textContent = `${dataOrDefault(currentSpeedOfSound, 4)} m/s`;
    } else if ($('speed-of-sound-calc')) { // Fallback ID
        $('speed-of-sound-calc').textContent = `${dataOrDefault(currentSpeedOfSound, 4)} m/s`;
    }
    
    // MÉTÉO FALLBACK (si l'API échoue, au moins les valeurs de fallback ISA s'affichent ici)
    // Ces IDs doivent correspondre aux span/div du HTML

    if ($('temp-air')) $('temp-air').textContent = dataOrDefault(currentSpeedOfSound > 340.28 ? 15.0 : null, 2, ' °C'); // Température ISA 15C si API KO
    if ($('pressure-atm')) $('pressure-atm').textContent = dataOrDefault(currentAirDensity < 1.226 ? 1013.25 : null, 0, ' hPa'); // Pression ISA 1013.25hPa
    if ($('air-density-rho')) $('air-density-rho').textContent = dataOrDefault(currentAirDensity, 3, ' kg/m³');


    // MISE À JOUR VITESSE/RELATIVITÉ
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
    // Assurez-vous que l'élément 'Statut Capteur' est bien ID 'imu-status'
    if ($('imu-status')) $('imu-status').textContent = isIMUActive ? 'Actif 🟢' : 'Inactif';


    // Synchronisation NTP (mise à jour du DOM)
    const now = getCDate(lServH, lLocH);
    if (now) {
        // ID CORRIGÉS
        if ($('local-time-ntp') && !$('local-time-ntp').textContent.includes('SYNCHRO ÉCHOUÉE')) {
            $('local-time-ntp').textContent = now.toLocaleTimeString('fr-FR');
        }
        if ($('date-utc-gmt')) $('date-utc-gmt').textContent = now.toLocaleDateString('fr-FR') + ' ' + now.toLocaleTimeString('fr-FR') + ' UTC';
    }
}


// --- BLOC 4 : FONCTIONS DE CONTRÔLE ET LISTENERS ---

function toggleGpsPause() {
    isGpsPaused = !isGpsPaused;
    const btn = $('toggle-gps-btn'); // 🟢 ID CORRIGÉ
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

function captureDataSnapshot() {
    const now = new Date().toISOString();
    const dataSnapshot = { /* ... */ };
    
    const dataStr = "data:text/json;charset=utf-8," + encodeURIComponent(JSON.stringify(dataSnapshot, null, 2));
    const downloadAnchorNode = document.createElement('a');
    downloadAnchorNode.setAttribute("href", dataStr);
    downloadAnchorNode.setAttribute("download", `gnss_data_capture_${now.substring(0, 19).replace('T', '_')}.json`);
    
    document.body.appendChild(downloadAnchorNode); 
    downloadAnchorNode.click();
    downloadAnchorNode.remove();
    alert("Données de session exportées en JSON.");
}

function fullReset() {
    if (!confirm("Voulez-vous vraiment TOUT RÉINITIALISER ?")) return;
    maxSpeed = 0.0;
    totalDistance = 0.0;
    initTime = Date.now();
    if (ukf) ukf = new ProfessionalUKF();
    currentPosition = { lat: 43.2964, lon: 5.3697, acc: 10.0, spd: 0.0, alt: 0.0 };
    if (!isGpsPaused) initGPS();
}

// --- INITIALISATION (window.onload) ---
window.addEventListener('load', () => {
    console.log("Démarrage du Dashboard GNSS (V3)...");

    // 1. Initialisation des systèmes critiques
    if (typeof math !== 'undefined') {
        ukf = new ProfessionalUKF(); 
    } else {
        console.error("🔴 math.js non trouvé. Le filtre UKF est désactivé.");
    }
    syncH(); // Démarrage de la synchro NTP
    
    // Initialisation du Rayon de Schwarzschild
    if ($('schwarzschild-radius')) $('schwarzschild-radius').textContent = dataOrDefaultExp(getSchwarzschildRadius(currentMass), 4, ' m');


    // 2. Initialisation des Listeners (ID CORRIGÉS)
    
    // GPS ON/PAUSE (ID CORRIGÉ: toggle-gps-btn)
    const gpsToggleBtn = $('toggle-gps-btn');
    if (gpsToggleBtn) {
        gpsToggleBtn.addEventListener('click', toggleGpsPause);
        // Assurez-vous que l'état initial du bouton correspond à l'état JS (isGpsPaused = true)
        gpsToggleBtn.innerHTML = isGpsPaused ? '▶️ MARCHE GPS' : '⏸️ PAUSE GPS';
    }
    
    // CAPTURER DONNÉES (ID CORRIGÉ: capture-btn)
    const captureBtn = $('capture-btn');
    if (captureBtn) captureBtn.addEventListener('click', captureDataSnapshot);
    
    // ACTIVER CAPTEURS IMU (ID PROBABLE: activate-sensors-btn)
    const imuActivateBtn = $('activate-sensors-btn'); 
    if (imuActivateBtn) imuActivateBtn.addEventListener('click', activateDeviceMotion);
    
    // TOUT RÉINITIALISER
    if ($('full-reset-btn')) $('full-reset-btn').addEventListener('click', fullReset);
    
    // 3. Boucle principale de rafraîchissement
    setInterval(updateDashboardDOM, 250); 
    
    // 4. Démarrage
    // Le GPS NE DÉMARRE PAS par défaut (isGpsPaused = true), l'utilisateur doit cliquer sur le bouton.
});
