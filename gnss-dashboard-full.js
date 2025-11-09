// =================================================================
// BLOC 1/2 : CONSTANTES, ÉTAT EKF ET LOGIQUE PRINCIPALE
// =================================================================

// PARTIE 1 : CONSTANTES GLOBALES ET PHYSIQUES

const KMH_MS = 3.6; // Conversion m/s en km/h
const MIN_SPD = 0.05; // M/s : vitesse minimale pour considérer un mouvement
const C = 299792458; // Vitesse de la lumière (m/s)

// --- PARAMÈTRES EKF ---
// Bruit du Processus Q (dérive IMU). Sera ajusté dynamiquement.
const Q_NOISE_INITIAL = 0.01; 
const Q_NOISE_MAX = 5.0; 
// Bruit de la Mesure R (précision GPS). Sera ajusté dynamiquement.
const R_FACTOR_INITIAL = 1.0; 

// --- SEUILS D'IMU / FILTRAGE ---
const MAX_IMU_ACCEL_INTEGRATION = 30.0; 
const MAX_GPS_ACCURACY_FOR_USE = 50.0; // Précision GPS max (en mètres) pour utiliser la correction

// ⬇️ AJUSTEMENT DYNAMIQUE
const ACCEL_FILTER_ALPHA = 0.2; // 🌟 Plus bas pour plus de dynamique (était à 0.5)
const MAX_ALLOWED_ACCEL_VARIANCE = 0.3; // 🌟 Seuil de bruit IMU pour suspendre dt (pour la marche/vibrations)
const ZVU_ACCEL_THRESHOLD = 0.15; // 🌟 Seuil d'accélération pour forcer Vitesse=0 à l'arrêt (en m/s²)

// --- INCERTITUDE TEMPORELLE / GPS ---
const MAX_SERVER_OFFSET_AGE_MS = 3600000; // 🌟 Âge max de la synchro horloge (1 heure)
const MAX_GPS_MEMORY_AGE_MS = 300000; // 🌟 Âge max de la mesure GPS pour être utilisée (5 minutes)

// --- CONSTANTES DIVERSES ---
const SERVER_TIME_ENDPOINT = 'YOUR_TIME_API_ENDPOINT'; 
const OWM_API_KEY = 'YOUR_OWM_API_KEY';

// PARTIE 2 : VARIABLES D'ÉTAT

let wID = null; // ID du watchPosition
let domID = null; // ID du setInterval DOM
let weatherID = null; // ID du setInterval Météo

// Position/Heure
let lat = 0; 
let lon = 0; 
let serverOffset = 0; 
let lastServerSyncTimestamp = 0; // Timestamp de la dernière synchro heure

// EKF (Filtre de Kalman Étendu)
let kSpd = 0.0; // Vitesse estimée (m/s)
let kUncert = 1.0; // Incertitude (m²/s²)
const MIN_UNCERT_FLOOR = 0.05; // Incertitude minimale
let lastTime = 0; // Dernier timestamp de l'EKF

// IMU
let latestIMULinearAccel = 0; // Accélération linéaire filtrée (m/s²)
let latestIMUGravity = [0, 0, 0];
let maxGForce = 0;
let maxGForceLat = 0;
let emergencyStopActive = false;

// Historique de l'accélération pour Q dynamique
const ACCEL_HISTORY_SIZE = 10; 
let accelHistory = []; 

// Vitesse et position mémorisée
let lastValidGpsSpeed = 0; 
let lastValidGpsAccuracy = MAX_GPS_ACCURACY_FOR_USE; 
let lastGpsSpeedTime = 0; // 🌟 Timestamp de la dernière vitesse GPS valide

// Mouvement
let distM = 0; 
let maxSpd = 0; 
let timeMoving = 0; 
let verticalSpeedRaw = 0; 
let smoothedSpeed = 0; 

// Variables secondaires
let netherMode = false;
let airTempC = 20; // 🌟 Température de l'air (mise à jour par getWeather)

// PARTIE 3 : FONCTIONS D'ACCESSOIRES ET UTILITAIRES

// Fonction d'accès DOM simplifiée
function $(id) { return document.getElementById(id); }

// Fonction de récupération de l'heure corrigée par le serveur
function getCDate() { return new Date(Date.now() + serverOffset); }

// Fonction de calcul de la variance de l'accélération
function calculateAccelVariance(history) {
    if (history.length < 2) return 0;
    const mean = history.reduce((a, b) => a + b, 0) / history.length;
    const variance = history.reduce((a, b) => a + Math.pow(b - mean, 2), 0) / history.length;
    return variance;
}

// PARTIE 4 : GESTIONNAIRE EKF ET GPS

/** Handler des données GPS et du filtre EKF. */
function updateDisp(pos) {
    if (emergencyStopActive) return;

    const now = Date.now();
    const dt = (now - lastTime) / 1000;
    if (dt === 0) return; // Éviter la division par zéro

    lastTime = now;
    lat = pos.coords.latitude || lat;
    lon = pos.coords.longitude || lon;

    // --- MISE À JOUR DU BRUIT Q ET DE LA CONFIANCE IMU ---
    
    // 1. Mise à jour de l'historique d'accélération
    if (accelHistory.length >= ACCEL_HISTORY_SIZE) {
        accelHistory.shift(); 
    }
    const clampedAccel = Math.min(latestIMULinearAccel, MAX_IMU_ACCEL_INTEGRATION);
    accelHistory.push(clampedAccel); 
    
    // 2. Calcul de la variance et ajustement du bruit Q (IMU)
    const accelVariance = calculateAccelVariance(accelHistory);

    // Initialisation du bruit Q
    let Q_NOISE = Q_NOISE_INITIAL * (pos.coords.accuracy || 1.0);
    const rFactor = R_FACTOR_INITIAL * (netherMode ? 8.0 : 1.0); // Pénalité Nether

    // Facteur de bruit dynamique Q : augmente avec la variance.
    const MAX_VAR_THRESHOLD = 0.5; 
    const Q_DYNAMIC_FACTOR = 1.0 + (accelVariance / MAX_VAR_THRESHOLD); 
    
    // 🌟 3. Pénalité d'Incertitude Temporelle
    let timeUncertaintyFactor = 1.0;
    const timeSinceLastSync = now - lastServerSyncTimestamp;
    
    if (lastServerSyncTimestamp === 0 || timeSinceLastSync > MAX_SERVER_OFFSET_AGE_MS) {
        timeUncertaintyFactor = 2.0; 

        // Pénalité de Température (si synchro perdue)
        const T_REF = 25; 
        const TEMP_COEF = 0.05; 
        const tempDeviation = Math.abs(airTempC - T_REF); 
        timeUncertaintyFactor += tempDeviation * TEMP_COEF;
    }
    
    // 4. Application finale du bruit Q
    Q_NOISE = Q_NOISE * Math.min(Q_DYNAMIC_FACTOR, 10.0) * timeUncertaintyFactor; 
    Q_NOISE = Math.min(Q_NOISE, Q_NOISE_MAX * 5); 

    // 5. DÉTERMINER LE TEMPS D'INTÉGRATION EFFECTIF (suspension de la dérive)
    let dt_eff = dt;
    let imu_is_trusted = true; 

    if (accelVariance > MAX_ALLOWED_ACCEL_VARIANCE) {
        dt_eff = 0; // Suspension de l'intégration si le bruit est trop haut
        imu_is_trusted = false;
    }
    
    // --- EKF: ÉTAPE DE PRÉDICTION (Modèle IMU) ---
    kSpd = kSpd + clampedAccel * dt_eff; 
    kUncert = kUncert + Q_NOISE * dt_eff; 
    kUncert = Math.max(kUncert, MIN_UNCERT_FLOOR); // Plancher d'incertitude

    // ⬆️ ZVU AMÉLIORÉ : N'appliquer le ZVU que si l'IMU est stable (imu_is_trusted)
    if (imu_is_trusted && clampedAccel < ZVU_ACCEL_THRESHOLD) { 
        kSpd = 0;
        kUncert = MIN_UNCERT_FLOOR;
    }

    // --- EKF: ÉTAPE DE CORRECTION (Mesure GPS de Vitesse) ---
    const gpsPositionValid = (pos.coords.accuracy !== null && pos.coords.accuracy < MAX_GPS_ACCURACY_FOR_USE);

    // 1. MISE À JOUR ET VIEILLISSEMENT DE LA MÉMOIRE GPS
    const currentTime = now; // now est en ms
    if (gpsPositionValid) { 
        lastValidGpsSpeed = pos.coords.speed;
        lastValidGpsAccuracy = pos.coords.accuracy;
        lastGpsSpeedTime = currentTime; 
    } 

    const ageOfGpsMemory = currentTime - lastGpsSpeedTime;
    
    if (ageOfGpsMemory > MAX_GPS_MEMORY_AGE_MS) {
        lastValidGpsAccuracy = MAX_GPS_ACCURACY_FOR_USE; 
    }
    
    // Correction GPS si mesure actuelle bonne OU si mesure mémorisée est encore jeune.
    const shouldRunGpsCorrection = (gpsPositionValid || lastValidGpsAccuracy < MAX_GPS_ACCURACY_FOR_USE);

    if (shouldRunGpsCorrection) { 
        
        const measuredSpd = gpsPositionValid ? pos.coords.speed : lastValidGpsSpeed;
        let usedAccuracy = gpsPositionValid ? pos.coords.accuracy : lastValidGpsAccuracy;
        
        // 🌟 APPLICATION DU VIEILLISSEMENT À LA PRÉCISION (PÉNALITÉ R)
        if (!gpsPositionValid && lastValidGpsAccuracy < MAX_GPS_ACCURACY_FOR_USE) { 
            const ageRatio = ageOfGpsMemory / MAX_GPS_MEMORY_AGE_MS; 
            const agingFactor = Math.pow(20, ageRatio); 
            usedAccuracy = usedAccuracy * agingFactor; 
        }

        // 🌟 CORRECTION EKF
        const lowSpeedFactor = (measuredSpd < 2.5) ? 0.5 : 1.0; 
        const R_MEASUREMENT_BASE = Math.max(1.0, usedAccuracy); 
        const R_MEASUREMENT = Math.pow(R_MEASUREMENT_BASE, 2) * rFactor * lowSpeedFactor; 

        const innovation = measuredSpd - kSpd; 
        const innovationCovariance = kUncert + R_MEASUREMENT; 
        const kalmanGain = kUncert / innovationCovariance;
        
        kSpd = kSpd + kalmanGain * innovation;
        kUncert = (1 - kalmanGain) * kUncert;
    }
    
    // --- CALCULS FINAUX ET AFFICHAGE ---
    const sSpdFE_3D = Math.sqrt(kSpd * kSpd + verticalSpeedRaw * verticalSpeedRaw);
    
    // 🌟 AFFICHAGE : Vitesse EKF brute pour le dynamisme
    const finalDisplaySpeed = sSpdFE_3D; 

    // ... (Le reste des calculs de distance, max speed, et mise à jour DOM est inchangé)
    
    if (sSpdFE_3D > maxSpd) maxSpd = sSpdFE_3D;
    if (finalDisplaySpeed > MIN_SPD) { 
        const d_3d = finalDisplaySpeed * dt; 
        distM += d_3d; 
        timeMoving += dt;
    }
    
    // Mise à jour de l'affichage DOM
    $('speed-stable').textContent = `${(finalDisplaySpeed * KMH_MS).toFixed(4)} km/h (3D)`;
    $('speed-stable-ms').textContent = `${(finalDisplaySpeed).toFixed(3)} m/s (3D)`;
    $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(4)} km/h`;
    
    $('latitude').textContent = lat.toFixed(6);
    $('longitude').textContent = lon.toFixed(6);
    $('altitude-gps').textContent = (pos.coords.altitude || 'N/A') + ' m';
    $('gps-precision').textContent = pos.coords.accuracy ? `${pos.coords.accuracy.toFixed(1)} m` : 'N/A';
    
    $('kalman-uncert').textContent = kUncert.toFixed(5) + ' m²';
    $('accel-long').textContent = latestIMULinearAccel.toFixed(3) + ' m/s²';
    $('accel-linear-corrected').textContent = clampedAccel.toFixed(3) + ' m/s² (Filtré)';
    
    // ... (Mise à jour d'autres éléments DOM)
    updateMap(lat, lon);
        }
// =================================================================
// BLOC 2/2 : IMU, FONCTIONS SECONDAIRES & INITIALISATION
// =================================================================

// --- GESTION IMU / FILTRAGE (DeviceMotion) ---

/** Handler des données du capteur de mouvement. */
function handleDeviceMotion(event) {
    if (!event.acceleration) return;

    // Accélération linéaire (sans gravité)
    const ax = event.acceleration.x || 0;
    const ay = event.acceleration.y || 0;
    const az = event.acceleration.z || 0;
    
    // Accélération totale (incluant gravité)
    const gx = event.accelerationIncludingGravity.x || 0;
    const gy = event.accelerationIncludingGravity.y || 0;
    const gz = event.accelerationIncludingGravity.z || 0;

    // Calcul de l'accélération linéaire brute (Norme vectorielle)
    const accelMagnitude = Math.sqrt(ax * ax + ay * ay + az * az);

    // 🌟 Filtre Passe-Bas sur l'accélération linéaire (Anti-vibration)
    // Utilise la constante ACCEL_FILTER_ALPHA définie dans le BLOC 1/2
    latestIMULinearAccel = latestIMULinearAccel * (1 - ACCEL_FILTER_ALPHA) + accelMagnitude * ACCEL_FILTER_ALPHA;

    // Mise à jour de la gravité résiduelle (pour l'affichage ou l'alignement)
    latestIMUGravity = [gx - ax, gy - ay, gz - az];

    // Calcul des forces G (pour l'affichage)
    const gNorm = Math.sqrt(gx * gx + gy * gy + gz * gz) / 9.81;
    if (gNorm > maxGForce) maxGForce = gNorm;
    
    $('force-g-max').textContent = maxGForce.toFixed(2) + ' G';
    $('gravity-calculated').textContent = `X:${latestIMUGravity[0].toFixed(2)}, Y:${latestIMUGravity[1].toFixed(2)}, Z:${latestIMUGravity[2].toFixed(2)}`;
}

/** Handler des données d'orientation. */
function handleDeviceOrientation(event) {
    // Les angles Pitch et Roll sont importants pour l'analyse des manèges
    const pitch = event.beta || 0; 
    const roll = event.gamma || 0;
    
    $('pitch-angle').textContent = pitch.toFixed(1) + ' °';
    $('roll-angle').textContent = roll.toFixed(1) + ' °';
}


// --- FONCTIONS SECONDAIRES (Astro, Météo, Horloge) ---

/** Met à jour les informations astronomiques (Heure Solaire). */
function updateAstro() {
    const cDate = getCDate();
    // Logique simplifiée : Heure Solaire Vraie (approximative)
    $('solar-time').textContent = `${cDate.getHours().toString().padStart(2, '0')}:${cDate.getMinutes().toString().padStart(2, '0')}:${cDate.getSeconds().toString().padStart(2, '0')} Local`;
}

/** Récupère les données météo pour la pénalité de température. */
function getWeather() {
    if (lat === 0 || lon === 0) return;

    // NOTE: Remplacez 'YOUR_OWM_API_KEY' par votre clé OpenWeatherMap
    const url = `https://api.openweathermap.org/data/2.5/weather?lat=${lat}&lon=${lon}&appid=${OWM_API_KEY}&units=metric&lang=fr`;

    fetch(url)
        .then(response => response.json())
        .then(data => {
            if (data.main) {
                // 🌟 MISE À JOUR DE LA TEMPÉRATURE (pour la pénalité de temps EKF)
                airTempC = data.main.temp; 
                $('temp-air').textContent = `${data.main.temp.toFixed(1)} °C`;
                $('pressure-baro').textContent = `${data.main.pressure || 'N/A'} hPa`;
                $('weather-status').textContent = data.weather[0].description;
            }
            if (data.wind) {
                 $('wind-speed').textContent = `${(data.wind.speed * KMH_MS).toFixed(1)} km/h`;
                 $('wind-dir').textContent = `${data.wind.deg || 'N/A'} °`;
            }
        })
        .catch(error => {
            console.error("Erreur Météo:", error);
            $('weather-status').textContent = 'Météo indisponible';
        });
}

/** Fonction de synchronisation de l'heure serveur (NTP). */
function syncH() {
    // NOTE: Remplacez 'YOUR_TIME_API_ENDPOINT' par un service de temps fiable
    fetch(SERVER_TIME_ENDPOINT)
        .then(response => response.json())
        .then(data => {
            // Exemple basé sur un JSON retournant 'utc_datetime'
            const serverTime = new Date(data.utc_datetime);
            const localTime = new Date();
            serverOffset = serverTime.getTime() - localTime.getTime();
            lastServerSyncTimestamp = Date.now(); // 🌟 MISE À JOUR DU TIMESTAMP DE SYNCHRO
            $('clock-utc').textContent = serverTime.toUTCString().slice(17, 25);
            console.log(`Synchronisation horaire effectuée. Décalage: ${serverOffset} ms`);
        })
        .catch(error => {
            console.error("Erreur de synchronisation horaire:", error);
            $('clock-utc').textContent = 'UTC: N/A';
        });
}

// --- GESTION GPS & IMU (Initialisation et Contrôles) ---

/** Fonction pour démarrer la lecture du GPS et des capteurs. */
function startGPS() {
    if (wID !== null) return; // Déjà démarré

    const options = {
        enableHighAccuracy: true,
        timeout: 5000,
        maximumAge: 0
    };

    wID = navigator.geolocation.watchPosition(updateDisp, (error) => {
        console.error("Erreur GPS:", error);
    }, options);

    lastTime = Date.now(); // Initialisation du temps EKF
    domID = setInterval(updateAstro, 500); // Mise à jour de l'heure

    // Tentative de synchronisation horaire et météo au démarrage
    syncH(); 
    weatherID = setInterval(getWeather, 600000); // Météo toutes les 10 minutes

    requestIMUPermissionAndStart();
    $('start-gnss').textContent = "EN COURS...";
    $('start-gnss').disabled = true;
}

/** Fonction pour arrêter la lecture. */
function stopGPS() {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
    }
    if (domID !== null) {
        clearInterval(domID);
        domID = null;
    }
    if (weatherID !== null) {
        clearInterval(weatherID);
        weatherID = null;
    }
    
    // Arrêt des écouteurs IMU
    window.removeEventListener('devicemotion', handleDeviceMotion);
    window.removeEventListener('deviceorientation', handleDeviceOrientation);
    
    $('start-gnss').textContent = "DÉMARRER GNSS/IMU";
    $('start-gnss').disabled = false;
}

/** Demande de permission IMU (iOS) et démarrage. */
function requestIMUPermissionAndStart() {
    if (typeof DeviceOrientationEvent.requestPermission === 'function') {
        DeviceOrientationEvent.requestPermission()
            .then(permissionState => {
                if (permissionState === 'granted') {
                    window.addEventListener('devicemotion', handleDeviceMotion);
                    window.addEventListener('deviceorientation', handleDeviceOrientation);
                } else {
                    console.error("Permission DeviceMotion/Orientation refusée.");
                }
            })
            .catch(console.error);
    } else {
        // Pour les navigateurs Android/PC
        window.addEventListener('devicemotion', handleDeviceMotion);
        window.addEventListener('deviceorientation', handleDeviceOrientation);
    }
}

/** Réinitialise les métriques de distance et de vitesse max. */
function resetMetrics() {
    distM = 0;
    maxSpd = 0;
    maxGForce = 0;
    kSpd = 0;
    kUncert = MIN_UNCERT_FLOOR;
    $('distance-total-km').textContent = '0.000 km';
    $('speed-max').textContent = '0.0000 km/h';
    $('force-g-max').textContent = '0.00 G';
    $('kalman-uncert').textContent = MIN_UNCERT_FLOOR.toFixed(5) + ' m²';
    emergencyStopActive = false;
    $('emergency-stop').classList.remove('error');
    $('emergency-stop').textContent = 'STOP';
}

// --- GESTION DOM ET ÉCOUTEURS D'ÉVÉNEMENTS ---

document.addEventListener('DOMContentLoaded', () => {
    // Initialisation de la carte (simplifiée)
    initMap(); 

    // Écouteurs pour les boutons de contrôle
    $('start-gnss').addEventListener('click', startGPS);
    $('emergency-stop').addEventListener('click', () => {
        emergencyStopActive = true;
        stopGPS();
        $('emergency-stop').classList.add('error');
        $('emergency-stop').textContent = 'STOP D’URGENCE (Reboot nécessaire)';
    });
    $('reset-metrics').addEventListener('click', resetMetrics);

    // Écouteur pour le mode Nether
    $('nether-mode-toggle').addEventListener('change', (e) => {
        netherMode = e.target.checked;
        $('mode-display').textContent = netherMode ? "NETHER" : "OVERWORLD";
    });

    // Écouteur pour la synchronisation manuelle
    $('sync-time').addEventListener('click', syncH);
    
    // Écouteur pour le sélecteur d'environnement (influence R factor via rFactor dans BLOC 1/2)
    $('environment-select').addEventListener('change', (e) => {
        const env = e.target.value;
        // Ceci est une implémentation simplifiée; il faudrait ajuster le R_FACTOR_INITIAL dans le BLOC 1/2
        // ou introduire une variable globale pour la pénalité d'environnement.
        console.log(`Environnement GPS sélectionné: ${env}`);
    });
});

// --- Fonctions de la Carte (Simplifiées) ---
let map;
let marker;

function initMap() {
    // Si vous utilisez Google Maps, le code d'initialisation irait ici.
    // Par souci de simplicité et de portabilité, nous affichons un placeholder.
    console.log("Carte initialisée (Placeholder)");
}

function updateMap(lat, lon) {
    // Logique de mise à jour du marqueur sur la carte.
    // Si vous n'utilisez pas de carte, ceci reste dans la console.
    // console.log(`Mise à jour de la carte à: ${lat}, ${lon}`);
}
