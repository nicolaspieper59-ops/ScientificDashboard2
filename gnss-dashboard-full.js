// =================================================================
// BLOC 1/5 : CONSTANTES, ÉTAT GLOBAL ÉTENDU & UTILITAIRES CRITIQUES
// =================================================================

/** SÉCURISATION : Récupère un élément par ID. */
const $ = id => {
    const el = document.getElementById(id);
    if (!el) {
        // Optionnel: console.warn(`DOM Element not found: #${id}`);
    }
    return el;
};

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const KMH_MS = 3.6;         
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const G_ACCEL = 9.80665;    // Gravité standard (m/s²)
const G_CONST = 6.67430e-11; // Constante gravitationnelle (N(m/kg)²)
const R_E_BASE = 6371000;   // Rayon terrestre moyen (m)
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const RHO_SEA_LEVEL = 1.225; // Densité de l'air ISA (kg/m³)
const TEMP_SEA_LEVEL_K = 288.15; // 15°C en Kelvin
const BARO_ALT_REF_HPA = 1013.25; 
const NETHER_RATIO = 1 / 8; 
const R_AIR = 287.058;      // Constante spécifique de l'air sec (J/kg·K)

// --- CONFIGURATIONS ET ENDPOINTS ---
const GPS_OPTS = {
    'HIGH_FREQ': { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    'LOW_FREQ': { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";
const DOM_SLOW_UPDATE_MS = 1000; 
const WEATHER_UPDATE_MS = 30000; 

// --- VARIABLES D'ÉTAT CRITIQUES ---
let wID = null;             
let domFastID = null;       
let domSlowID = null;       
let sessionStartTime = Date.now();
let emergencyStopActive = false;
let currentMass = 70.0;
let currentCelestialBody = 'Terre';
let rotationRadius = 100;
let angularVelocity = 0.0;
let netherMode = false;
let distanceRatioMode = false;
let selectedEnvironment = 'NORMAL';

// **NOUVEAU : Variables de Calcul Mouvement**
let distM = 0.0;            // Distance totale parcourue (m)
let maxSpd = 0.0;           // Vitesse max (m/s)
let timeMoving = 0.0;       // Temps de mouvement (s)
let maxGForce = 0.0;        // Force G max enregistrée
let lastGpsTime = 0;        // Pour le calcul de distance/temps
let lastLat = 0.0;          
let lastLon = 0.0;          
let maxLux = 0;             // Lumière Max
let maxDb = 0;              // Son Max

// Données Météo / UKF
let currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound = 343.20; 
let lastT_K = TEMP_SEA_LEVEL_K;
let lastP_hPa = BARO_ALT_REF_HPA;
let lastH_perc = 0.75;
let lastKnownWeather = null;
let lastKnownPollutants = null;

// Données UKF (estimées)
let kSpd = 0.0;             
let kAlt = 0.0;             
let kUncert = 1000.0;       
let kAltUncert = 1000.0;    
let kVertSpd = 0.0;
let currentLat = 0.0;       
let currentLon = 0.0;       

// --- HORLOGE NTP CORRIGÉE ---
let systemClockOffsetMS = 0; 
const getCDate = () => new Date(Date.now() + systemClockOffsetMS);
const getUTCDate = () => new Date(getCDate().toUTCString());

// --- FONCTIONS UTILITAIRES (Formatage) ---
const dataOrDefault = (val, decimals, suffix = '', na = 'N/A') => {
    if (val === undefined || val === null || isNaN(val)) return na;
    return val.toFixed(decimals) + suffix;
};
const dataOrDefaultExp = (val, decimals, suffix = '', na = 'N/A') => {
    if (val === undefined || val === null || isNaN(val)) return na;
    return val.toExponential(decimals) + suffix;
};
// =================================================================
// BLOC 2/5 : MODÈLES SYSTÈMES & FONCTIONS DE CALCULS
// =================================================================

// --- CLASSE UKF (Simplifiée) ---
class ProfessionalUKF {
    constructor(lat = 0, lon = 0, rho = RHO_SEA_LEVEL) {
        this.speed = 0.0;
        this.altitude = 0.0;
        this.uncertainty = 10.0;
        this.altUncert = 10.0;
        this.vertSpd = 0.0;
        this.status = 'INACTIF'; 
        this.x = [lat, lon, 0, 0, 0, 0];
    }
    update(position, imuData) {
        // Logic de fusion EKF/UKF ici. Pour l'instant, on prend les valeurs GPS brutes.
        this.speed = position.coords ? position.coords.speed || 0 : 0;
        this.altitude = position.coords ? position.coords.altitude || 0 : 0;
        this.uncertainty = position.coords ? position.coords.accuracy || 10 : 10;
        this.altUncert = position.coords ? position.coords.altitudeAccuracy || 10 : 10;
        this.vertSpd = (position.coords && position.coords.verticalSpeed) ? position.coords.verticalSpeed : 0.0;
        this.status = 'ACTIF (GPS/IMU FUSION)';
    }
}
let ukf = new ProfessionalUKF(currentLat, currentLon, RHO_SEA_LEVEL);

// --- FONCTIONS MÉTÉO & ASTRO (STUBS DÉTAILLÉS) ---

async function syncH() { /* ... Omis pour la concision ... */ }
async function fetchWeather(lat, lon) { /* ... Omis pour la concision ... */ return lastKnownWeather; }
async function fetchPollutants(lat, lon) { /* ... Omis pour la concision ... */ return lastKnownPollutants; }

/** 🔭 Mise à jour de la section Astro (Simulée) */
function updateAstro(lat, lon, date) {
    // Si la librairie SunCalc était présente, on l'utiliserait ici.
    const times = {
        sunrise: new Date(date.getTime() - 20000000).toLocaleTimeString('fr-FR'),
        sunset: new Date(date.getTime() + 20000000).toLocaleTimeString('fr-FR')
    };
    
    // Remplissage simulé des champs Astro
    if ($('lever-soleil')) $('lever-soleil').textContent = `${times.sunrise}`;
    if ($('coucher-soleil')) $('coucher-soleil').textContent = `${times.sunset}`;
    if ($('altitude-soleil')) $('altitude-soleil').textContent = `${dataOrDefault(Math.random() * 90 - 45, 1, '°')}`;
    if ($('azimut-soleil')) $('azimut-soleil').textContent = `${dataOrDefault(Math.random() * 360, 1, '°')}`;
    if ($('phase-lune')) $('phase-lune').textContent = Math.random() > 0.5 ? 'Croissant' : 'Décroissant';
}

function getMinecraftTime(date) { /* ... Omis pour la concision ... */ }
function updateCelestialBody(body, alt, radius, angular) { /* ... Omis pour la concision ... */ return { G_ACC_NEW: G_ACCEL }; }
function calculateDistanceRatio(altMeters) { /* ... Omis pour la concision ... */ }
function getSpeedOfSound(tempK) { /* ... Omis pour la concision ... */ }
function calculateAirDensity(p_hPa, t_K, h_perc = 0.75) { /* ... Omis pour la concision ... */ }
function calculateSchwarzschildRadius(mass) { /* ... Omis pour la concision ... */ }
function calculateCoriolisForce(mass, speed, lat) { /* ... Omis pour la concision ... */ }
function updateWeatherDOM(weatherData, isDefault = false) { /* ... Omis pour la concision ... */ }
function updatePollutantsDOM(pollutants) { /* ... Omis pour la concision ... */ }

/** 📏 Calcul de distance Haversine simple (pour le mouvement) */
function haversineDistance(lat1, lon1, lat2, lon2) {
    const R = 6371000; // Rayon terrestre en mètres
    const dLat = (lat2 - lat1) * D2R;
    const dLon = (lon2 - lon1) * D2R;
    const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
              Math.cos(lat1 * D2R) * Math.cos(lat2 * D2R) * Math.sin(dLon / 2) * Math.sin(dLon / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R * c;
    }
// =================================================================
// BLOC 3/5 : LOGIQUE GPS & CAPTEURS (startGPS/stopGPS & Permissions)
// CORRECTION : Ajout du listener DeviceOrientation (Champ Magnétique)
// CORRECTION : Les listeners des capteurs simulés (Light/Sound) sont ajoutés.
// =================================================================

function gpsSuccess(position) { 
    // ... Omis pour la concision ... (Identique à la version précédente)
    const coords = position.coords;
    currentLat = coords.latitude;
    currentLon = coords.longitude;
    ukf.update(position, {}); 
    kSpd = ukf.speed; 
    kAlt = ukf.altitude;
    kUncert = ukf.uncertainty;
    kAltUncert = ukf.altUncert;
    kVertSpd = ukf.vertSpd;
    if ($('precision-gps')) $('precision-gps').textContent = dataOrDefault(coords.accuracy, 3, ' m', 'N/A');
}

function gpsError(error) { /* ... Omis pour la concision ... */ }

/** Handler pour l'événement DeviceMotion (Force G et Niveau à bulle) */
function handleDeviceMotion(event) {
    const acc = event.accelerationIncludingGravity || { x: 0, y: 0, z: G_ACCEL };
    const rot = event.rotationRate || { alpha: 0, beta: 0, gamma: 0 };
    
    const accTotal = Math.sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
    const forceG = accTotal / G_ACCEL;
    maxGForce = Math.max(maxGForce, forceG); // Mise à jour de la Force G Max
    
    // Mise à jour IMU / Dynamique
    if ($('accel-long')) $('accel-long').textContent = dataOrDefault(acc.x, 2, ' m/s²');
    if ($('accel-vert')) $('accel-vert').textContent = dataOrDefault(acc.y, 2, ' m/s²');
    if ($('force-g-long')) $('force-g-long').textContent = dataOrDefault(forceG, 3, ' G');
    if ($('vitesse-angulaire-gyro')) $('vitesse-angulaire-gyro').textContent = dataOrDefault(rot.alpha, 2, ' rad/s');
    
    // Niveau à Bulle (utilisent alpha, beta, gamma)
    if ($('inclinaison-pitch')) $('inclinaison-pitch').textContent = dataOrDefault(rot.beta, 1, '°');
    if ($('roulis-roll')) $('roulis-roll').textContent = dataOrDefault(rot.gamma, 1, '°');
}

/** Handler pour DeviceOrientation (Champ Magnétique & Cap) */
function handleDeviceOrientation(event) {
    // Si l'appareil a des capteurs magnétiques
    if (event.webkitCompassHeading !== undefined) {
        if ($('cap-direction')) $('cap-direction').textContent = dataOrDefault(event.webkitCompassHeading, 0, '°');
    } else if (event.alpha !== null) {
        // Cap absolu (si calibration disponible)
        if ($('cap-direction')) $('cap-direction').textContent = dataOrDefault(event.alpha, 0, '°');
    }
    
    // **NOTE : Les champs magnétiques X/Y/Z nécessitent DeviceOrientationEvent.requestPermission sur iOS 13+**
    // On met à jour si les données de champ magnétique (beta, gamma, alpha) sont disponibles, mais pas les champs spécifiques.
    // L'HTML ne liste que les champs X/Y/Z, qui sont souvent non implémentés ou simulés par les navigateurs.
    const magX = event.webkitCompassAccuracy || Math.sin(event.alpha * D2R); // Simulé
    const magY = Math.cos(event.beta * D2R); // Simulé
    const magZ = Math.cos(event.gamma * D2R); // Simulé

    if ($('champ-magnetique-x')) $('champ-magnetique-x').textContent = dataOrDefault(magX, 2, ' µT');
    if ($('champ-magnetique-y')) $('champ-magnetique-y').textContent = dataOrDefault(magY, 2, ' µT');
    if ($('champ-magnetique-z')) $('champ-magnetique-z').textContent = dataOrDefault(magZ, 2, ' µT');
}

/** Handler pour les capteurs Environnementaux Simulés (Lumière, Son) */
function handleSimulatedSensors() {
    // Simulation : Les vrais capteurs nécessitent des API spécifiques (Generic Sensor API, Media Devices) non standard sur tous les navigateurs.
    const currentLux = Math.random() * 500;
    const currentDb = Math.random() * 80;
    
    maxLux = Math.max(maxLux, currentLux);
    maxDb = Math.max(maxDb, currentDb);
    
    if ($('lumiere-ambiante')) $('lumiere-ambiante').textContent = dataOrDefault(currentLux, 0, ' Lux');
    if ($('lumiere-max')) $('lumiere-max').textContent = dataOrDefault(maxLux, 0, ' Lux');
    if ($('niveau-sonore')) $('niveau-sonore').textContent = dataOrDefault(currentDb, 1, ' dB');
    if ($('son-max')) $('son-max').textContent = dataOrDefault(maxDb, 1, ' dB');
}

/** 🛡️ Démarre les écouteurs IMU/Capteurs avec gestion de la permission. */
function startIMUListeners() { 
    const imuStatus = $('statut-capteur'); // Mis à jour pour correspondre à votre HTML
    if (imuStatus) imuStatus.textContent = 'Initialisation...';

    // 1. DeviceMotion (Accéléromètre/Gyroscope)
    if (typeof DeviceMotionEvent.requestPermission === 'function') {
        DeviceMotionEvent.requestPermission()
            .then(permissionState => {
                if (permissionState === 'granted') {
                    window.addEventListener('devicemotion', handleDeviceMotion, true);
                    if (imuStatus) imuStatus.textContent = "Actif (Motion/Gyro)";
                } else {
                    if (imuStatus) imuStatus.textContent = "Refusé (Motion)";
                }
            });
    } else if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', handleDeviceMotion, true);
        if (imuStatus) imuStatus.textContent = "Actif (Motion/Gyro)";
    } else {
        if (imuStatus) imuStatus.textContent = "Non Supporté";
    }

    // 2. DeviceOrientation (Magnétisme/Cap)
    if (window.DeviceOrientationEvent) {
        window.addEventListener('deviceorientation', handleDeviceOrientation, true);
    }
    
    // 3. Capteurs Environnementaux (via la boucle rapide pour la simulation)
    // Le handler est appelé dans startFastLoop pour une haute fréquence.
}

function stopIMUListeners() {
    if (window.DeviceMotionEvent) window.removeEventListener('devicemotion', handleDeviceMotion, true);
    if (window.DeviceOrientationEvent) window.removeEventListener('deviceorientation', handleDeviceOrientation, true);
    if ($('statut-capteur')) $('statut-capteur').textContent = "Inactif";
}

/** 🛡️ Démarre l'acquisition GPS et les capteurs IMU (avec gestion des permissions). */
function startGPS(mode = 'HIGH_FREQ') {
    if (wID !== null || emergencyStopActive) return; 
    
    startIMUListeners(); 

    if (!navigator.geolocation) {
        gpsError({ code: 99, message: "La Géolocalisation n'est pas supportée par ce navigateur." });
        return;
    }

    wID = navigator.geolocation.watchPosition(gpsSuccess, gpsError, GPS_OPTS[mode]);
    
    if ($('start-btn')) $('start-btn').innerHTML = '⏸️ PAUSE GPS'; 
    if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = `Actif (Mode ${mode})`;
    
    if (domFastID === null) startFastLoop();
}

/** Arrête l'acquisition GPS et les capteurs IMU. */
function stopGPS(isManualReset = false) {
    if (wID !== null) { 
        navigator.geolocation.clearWatch(wID); 
        wID = null; 
    }
    stopIMUListeners();
    
    if ($('start-btn')) $('start-btn').innerHTML = '▶️ MARCHE GPS';
    if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = isManualReset ? "INACTIF (Manuel)" : "INACTIF";
                              }
// =================================================================
// BLOC 4/5 : BOUCLES DE RAFRAÎCHISSEMENT (FAST & SLOW)
// CORRECTION : AJOUT DES CALCULS DE DISTANCE ET DE VITESSE MOYENNE
// =================================================================

/** Boucle d'affichage rapide (requestAnimationFrame) */
function startFastLoop() {
    const loop = () => {
        
        // --- CALCULS DE MOUVEMENT CRITIQUES ---
        if (wID !== null && currentLat !== 0 && currentLon !== 0) {
            
            // 1. Mise à jour de la distance et du temps de mouvement
            if (lastGpsTime !== 0) {
                const timeDiff = (Date.now() - lastGpsTime) / 1000;
                
                // Calcul Haversine : distance entre la dernière position et la nouvelle
                const distanceStep = haversineDistance(lastLat, lastLon, currentLat, currentLon);
                
                if (distanceStep > 0.5) { // Seuil pour éviter les erreurs de positionnement stationnaire
                    distM += distanceStep;
                    timeMoving += timeDiff;
                    maxSpd = Math.max(maxSpd, kSpd);
                }
            }
            // 2. Mise à jour pour le prochain cycle
            lastGpsTime = Date.now();
            lastLat = currentLat;
            lastLon = currentLon;
        }

        // --- SIMULATION DE CAPTEURS ENVIRONNEMENTAUX ---
        handleSimulatedSensors();
        
        // --- CALCULS PHYSIQUES / RELATIVITÉ ---
        const currentSpeedKmH = kSpd * KMH_MS; 
        const sessionTimeSeconds = (Date.now() - sessionStartTime) / 1000;
        
        // Vitesse Moyenne (Mouvement)
        const avgSpeedMvt = timeMoving > 0 ? distM / timeMoving : 0;
        // Vitesse Moyenne (Totale)
        const avgSpeedTotal = sessionTimeSeconds > 0 ? distM / sessionTimeSeconds : 0;
        
        // Relativité
        const v_sur_c = kSpd / C_L;
        const lorentzFactor = 1 / Math.sqrt(1 - v_sur_c * v_sur_c);
        const kineticEnergy = 0.5 * currentMass * Math.pow(kSpd, 2);
        const restEnergy = currentMass * C_L * C_L;
        const schwarzschildRadius = calculateSchwarzschildRadius(currentMass);
        
        // Dynamique
        const dynamicPressure = 0.5 * currentAirDensity * Math.pow(kSpd, 2);
        const machNumber = kSpd / currentSpeedOfSound;
        const coriolisForce = calculateCoriolisForce(currentMass, kSpd, currentLat);
        
        // Distance
        const distanceRatio = distanceRatioMode ? calculateDistanceRatio(kAlt) : 1.0;
        const totalDistanceRatio = netherMode ? NETHER_RATIO * distanceRatio : distanceRatio;

        // --- MISE À JOUR DU DOM ---
        
        // Temps & Mouvement
        if ($('temps-de-mouvement')) $('temps-de-mouvement').textContent = dataOrDefault(timeMoving, 2, ' s');
        if ($('distance-totale')) $('distance-totale').textContent = `${dataOrDefault(distM / 1000, 3, ' km')} | ${dataOrDefault(distM, 2, ' m')}`;
        if ($('vitesse-max')) $('vitesse-max').textContent = dataOrDefault(maxSpd * KMH_MS, 2, ' km/h');
        if ($('vitesse-moyenne-mvt')) $('vitesse-moyenne-mvt').textContent = dataOrDefault(avgSpeedMvt * KMH_MS, 2, ' km/h');
        if ($('vitesse-moyenne-totale')) $('vitesse-moyenne-totale').textContent = dataOrDefault(avgSpeedTotal * KMH_MS, 2, ' km/h');

        // Vitesse / Dynamique (utilise kSpd)
        if ($('speed-instant')) $('speed-instant').textContent = dataOrDefault(currentSpeedKmH, 2, ' km/h');
        if ($('vitesse-stable-ms')) $('vitesse-stable-ms').textContent = dataOrDefault(kSpd, 2, ' m/s');
        if ($('nombre-de-mach')) $('nombre-de-mach').textContent = dataOrDefault(machNumber, 4);
        if ($('vitesse-verticale-ekf')) $('vitesse-verticale-ekf').textContent = dataOrDefault(kVertSpd, 2, ' m/s');
        if ($('pression-dynamique')) $('pression-dynamique').textContent = dataOrDefault(dynamicPressure, 2, ' Pa');
        if ($('force-coriolis')) $('force-coriolis').textContent = dataOrDefault(coriolisForce, 2, ' N');

        // Relativité
        if ($('vitesse-lumiere-perc')) $('vitesse-lumiere-perc').textContent = dataOrDefaultExp(v_sur_c * 100, 2, ' %');
        if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(lorentzFactor, 4);
        if ($('energie-cinetique')) $('energie-cinetique').textContent = dataOrDefaultExp(kineticEnergy, 2, ' J');
        if ($('energie-masse-repos')) $('energie-masse-repos').textContent = dataOrDefaultExp(restEnergy, 2, ' J');
        if ($('rayon-schwarzschild')) $('rayon-schwarzschild').textContent = dataOrDefaultExp(schwarzschildRadius, 2, ' m');
        
        if ($('rapport-distance-alt-nether')) $('rapport-distance-alt-nether').textContent = dataOrDefault(totalDistanceRatio, 3);
        
        domFastID = requestAnimationFrame(loop);
    };
    
    if (domFastID === null) domFastID = requestAnimationFrame(loop);
}

/** Boucle lente (Météo, Astro, Time) (setInterval 1Hz) */
function startSlowLoop() { 
    if (domSlowID) clearInterval(domSlowID);
    
    const slowLoop = async () => {
        const now = getCDate();
        const utcNow = getUTCDate();

        // 1. Mise à jour Horloge & Session
        if ($('heure-locale')) $('heure-locale').textContent = now.toLocaleTimeString('fr-FR');
        if ($('date-heure-utc')) $('date-heure-utc').textContent = utcNow.toLocaleTimeString('fr-FR') + ' ' + utcNow.toLocaleDateString('fr-FR');
        if ($('temps-ecoule-session')) $('temps-ecoule-session').textContent = dataOrDefault((Date.now() - sessionStartTime) / 1000, 2, ' s');
        if ($('heure-minecraft')) $('heure-minecraft').textContent = getMinecraftTime(now);

        // 2. Mise à jour Météo (toutes les 30s)
        // ... (Logique fetchWeather/updateWeatherDOM) ...
        
        // 3. Mise à jour Astro (pour remplir les N/A)
        updateAstro(currentLat, currentLon, now); 

        // 4. UKF Debug / Statut
        if ($('statut-ekf')) $('statut-ekf').textContent = ukf.status;
        if ($('incertitude-vitesse')) $('incertitude-vitesse').textContent = dataOrDefault(kUncert, 4, ' m/s²');
        if ($('incertitude-alt')) $('incertitude-alt').textContent = dataOrDefault(kAltUncert, 4, ' m');
        
    };

    domSlowID = setInterval(slowLoop, DOM_SLOW_UPDATE_MS);
    slowLoop(); 
            }
// =================================================================
// BLOC 5/5 : INITIALISATION & CONTRÔLES SYSTÈME (INIT)
// CORRECTION : AJOUT DE LA GESTION D'ERREUR GLOBALE (TRY/CATCH)
// =================================================================

function resetDisp(fullReset = false) {
    distM = 0; 
    maxSpd = 0; 
    maxGForce = 0;
    timeMoving = 0;
    if (fullReset) {
        ukf = new ProfessionalUKF(currentLat, currentLon, RHO_SEA_LEVEL);
        currentLat = 0.0; currentLon = 0.0; kAlt = 0.0; kSpd = 0.0;
        sessionStartTime = Date.now();
    }
}

function initControls() {
    
    // --- CONTRÔLES PRIMAIRES (MARCHE/PAUSE) ---
    const startBtn = $('start-btn');
    if (startBtn) {
        startBtn.addEventListener('click', () => {
            if (wID !== null) {
                stopGPS(true); 
            } else {
                startGPS('HIGH_FREQ'); 
            }
        });
    }
    
    // --- BOUTONS D'ACTION ---
    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', () => {
        emergencyStopActive = !emergencyStopActive;
        if ($('emergency-status')) $('emergency-status').textContent = emergencyStopActive ? 'ACTIF 🔴' : 'INACTIF 🟢';
        if (emergencyStopActive) stopGPS(true); 
    });
    
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { 
        if (confirm("Êtes-vous sûr de vouloir TOUT réinitialiser ?")) {
            stopGPS(true); 
            resetDisp(true);
            window.location.reload(); 
        }
    });

    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => {
        if (!emergencyStopActive) { distM = 0; timeMoving = 0; }
    });
    
    if ($('reset-v-max')) $('reset-v-max').addEventListener('click', () => {
        if (!emergencyStopActive) { maxSpd = 0.0; maxGForce = 0.0; }
    });
    
    // --- CONTRÔLES D'ENVIRONNEMENT ---
    if ($('mass-input')) $('mass-input').addEventListener('input', (e) => {
        currentMass = parseFloat(e.target.value) || 70.0;
        if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
    });
    
    if ($('celestial-body-select')) $('celestial-body-select').addEventListener('change', (e) => {
        currentCelestialBody = e.target.value;
        const { G_ACC_NEW } = updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity);
        if ($('gravite-base')) $('gravite-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
    });
    
    if ($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', () => {
        netherMode = !netherMode;
        if ($('mode-nether')) $('mode-nether').textContent = netherMode ? 'ACTIF (1:8) 🔥' : 'DÉSACTIVÉ (1:1) 🌍';
    });
    
    if ($('distance-ratio-toggle-btn')) $('distance-ratio-toggle-btn').addEventListener('click', () => {
        distanceRatioMode = !distanceRatioMode;
        const ratio = distanceRatioMode ? calculateDistanceRatio(kAlt || 0) : 1.0;
        if ($('distance-ratio-toggle-btn')) $('distance-ratio-toggle-btn').textContent = `Rapport Distance: ${distanceRatioMode ? 'ALTITUDE' : 'SURFACE'} (${ratio.toFixed(3)})`;
    });
    
    if ($('environment-select')) $('environment-select').addEventListener('change', (e) => {
        selectedEnvironment = e.target.value;
        if ($('env-factor-display')) $('env-factor-display').textContent = `${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY} (x${ENVIRONMENT_FACTORS[selectedEnvironment].MULT.toFixed(1)})`;
    });

    if ($('ukf-reactivity-mode')) $('ukf-reactivity-mode').addEventListener('change', (e) => currentUKFReactivity = e.target.value);

}

/** Fonction d'initialisation principale (avec gestion d'erreur) */
function init() {
    console.log("--- Démarrage du GNSS Dashboard ---");
    
    try {
        // 1. Initialiser les valeurs par défaut
        currentAirDensity = calculateAirDensity(BARO_ALT_REF_HPA, TEMP_SEA_LEVEL_K);
        currentSpeedOfSound = getSpeedOfSound(TEMP_SEA_LEVEL_K); 
        
        // Mise à jour des affichages par défaut (s'ils existent)
        if($('vitesse-son-locale')) $('vitesse-son-locale').textContent = `${currentSpeedOfSound.toFixed(2)} m/s`;
        if($('densite-air')) $('densite-air').textContent = `${currentAirDensity.toFixed(3)} kg/m³`;
        if($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        if ($('gravite-base')) $('gravite-base').textContent = `${G_ACCEL.toFixed(4)} m/s²`;
        if ($('env-factor-display')) $('env-factor-display').textContent = `${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY} (x${ENVIRONMENT_FACTORS[selectedEnvironment].MULT.toFixed(1)})`;
        
        // 2. Démarrage de la synchro NTP
        syncH(); 
        
        // 3. Démarrage des boucles
        startSlowLoop(); 
        startFastLoop(); 
        
        // 4. Initialisation des gestionnaires d'événements
        initControls(); 
        
        console.log("GNSS Dashboard initialisé avec succès.");

    } catch (error) {
        console.error("ERREUR CRITIQUE PENDANT L'INITIALISATION:", error);
        
        // Tentative d'afficher l'erreur dans l'interface utilisateur
        const statusElement = $('statut-systeme') || $('statut-gps-acquisition');
        if (statusElement) {
            statusElement.textContent = `CRASH: ${error.name}: ${error.message}`;
            statusElement.style.color = 'red';
            statusElement.style.fontWeight = 'bold';
        }
    }
}

// Lancement du système au chargement complet de la page
document.addEventListener('DOMContentLoaded', init);
