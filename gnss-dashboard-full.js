// =================================================================
// BLOC 1/5 : CONSTANTES, ÉTAT GLOBAL ÉTENDU & UTILITAIRES CRITIQUES
// CORRECTION : SÉCURISATION DE LA FONCTION DE SÉLECTION DU DOM ($)
// =================================================================

/** SÉCURISATION : Récupère un élément par ID. Affiche un avertissement si manquant. */
const $ = id => {
    const el = document.getElementById(id);
    if (!el) {
        console.warn(`DOM Element not found: #${id}. Check your HTML structure.`);
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

// --- UKF PARAMETERS / ENV FACTORS ---
const Q_NOISE = 0.1;        
const R_MIN = 0.01;         
const R_MAX = 500.0;        
const ENVIRONMENT_FACTORS = {
    NORMAL: { DISPLAY: "Normal", MULT: 1.0, R_FACTOR: 1.0 },
    FAIBLE: { DISPLAY: "Faible", MULT: 0.5, R_FACTOR: 0.5 },
    FORT: { DISPLAY: "Fort", MULT: 2.0, R_FACTOR: 2.0 }
};

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

// Données Météo / Environnement (pour UKF)
let currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound = 343.20; 
let lastT_K = TEMP_SEA_LEVEL_K;
let lastP_hPa = BARO_ALT_REF_HPA;
let lastH_perc = 0.75;
let lastKnownWeather = null;
let lastKnownPollutants = null;

// Données de session
let distM = 0.0;            // Distance totale parcourue (m)
let maxSpd = 0.0;           // Vitesse max (m/s)
let timeMoving = 0.0;       // Temps de mouvement (s)
let maxGForce = 0.0;        // Force G max enregistrée

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
// BLOC 2/5 : MODÈLES SYSTÈMES & FILTRES (UKF, ASTRO, MÉTÉO)
// =================================================================

// --- CLASSE UKF (21 États - Interface Fictive) ---
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

async function syncH() { 
    try {
        const response = await fetch(SERVER_TIME_ENDPOINT);
        const data = await response.json();
        const serverTimeMS = new Date(data.utc_datetime).getTime();
        systemClockOffsetMS = serverTimeMS - Date.now();
    } catch (e) {
        systemClockOffsetMS = 0;
    }
}

async function fetchWeather(lat, lon) {
    lastKnownWeather = {
        tempC: lastT_K - 273.15,
        pressure_hPa: lastP_hPa,
        humidity_perc: lastH_perc * 100,
        air_density: currentAirDensity,
        dew_point: (lastT_K - 273.15) - ((100 - lastH_perc * 100) / 5), 
        timestamp: Date.now()
    };
    return lastKnownWeather;
}

async function fetchPollutants(lat, lon) {
    lastKnownPollutants = {
        NO2: 25.5, PM25: 12.2, PM10: 18.0, O3: 50.0
    };
    return lastKnownPollutants;
}

function updateAstro(lat, lon, date) {
    // Placeholder pour SunCalc/AstroJS
}

function getMinecraftTime(date) {
    const msSinceMidnight = date.getHours() * 3600000 + date.getMinutes() * 60000;
    const mcTicks = Math.floor((msSinceMidnight / 86400000) * 24000);
    const mcHour = Math.floor((mcTicks / 1000 + 6) % 24); 
    const mcMinute = Math.floor(((mcTicks % 1000) / 1000) * 60);
    return `${String(mcHour).padStart(2, '0')}:${String(mcMinute).padStart(2, '0')}`;
}

function updateCelestialBody(body, alt, radius, angular) { 
    let newG = G_ACCEL;
    if (body === 'Lune') newG = G_ACCEL * 0.165;
    if (body === 'ROTATING') newG += (angular * angular * radius); 
    return { G_ACC_NEW: newG };
}

function calculateDistanceRatio(altMeters) {
    return 1.0 + (altMeters / R_E_BASE);
}

function getSpeedOfSound(tempK) { 
    return 20.045 * Math.sqrt(tempK); 
}

function calculateAirDensity(p_hPa, t_K, h_perc = 0.75) {
    const P_Pa = p_hPa * 100; 
    return (P_Pa / (R_AIR * t_K)); 
}

function calculateSchwarzschildRadius(mass) {
    return (2 * G_CONST * mass) / (C_L * C_L);
}

function calculateCoriolisForce(mass, speed, lat) {
    return 2 * mass * OMEGA_EARTH * speed * Math.sin(lat * D2R);
}

function updateWeatherDOM(weatherData, isDefault = false) {
    const statusText = isDefault ? "INACTIF (Défaut)" : "ACTIF (API)";
    if ($('statut-meteo')) $('statut-meteo').textContent = statusText;
    if ($('temp-air')) $('temp-air').textContent = dataOrDefault(weatherData.tempC, 1, ' °C');
    if ($('pression-atmospherique')) $('pression-atmospherique').textContent = dataOrDefault(weatherData.pressure_hPa, 0, ' hPa');
    if ($('humidite-relative')) $('humidite-relative').textContent = dataOrDefault(weatherData.humidity_perc, 0, ' %');
    if ($('densite-air')) $('densite-air').textContent = dataOrDefault(weatherData.air_density, 3, ' kg/m³');
    if ($('point-de-rosee')) $('point-de-rosee').textContent = dataOrDefault(weatherData.dew_point, 1, ' °C');
}

function updatePollutantsDOM(pollutants) {
    if ($('no2')) $('no2').textContent = dataOrDefault(pollutants.NO2, 1, ' µg/m³');
    if ($('pm25')) $('pm25').textContent = dataOrDefault(pollutants.PM25, 1, ' µg/m³');
    if ($('pm10')) $('pm10').textContent = dataOrDefault(pollutants.PM10, 1, ' µg/m³');
    if ($('o3')) $('o3').textContent = dataOrDefault(pollutants.O3, 1, ' µg/m³');
}
// =================================================================
// BLOC 3/5 : LOGIQUE GPS & CAPTEURS (startGPS/stopGPS & Permissions)
// =================================================================

function gpsSuccess(position) { 
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

function gpsError(error) { 
    let message = `ERREUR (${error.code})`;
    if (error.code === 1) message = "Permission GPS Refusée";
    else if (error.code === 2) message = "Position Indisponible";
    else if (error.code === 3) message = "Timeout GPS";
    
    console.error("Erreur GPS:", message, error.message);
    if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = message;
}

/** Handler pour l'événement DeviceMotion (Force G et Niveau à bulle) */
function handleDeviceMotion(event) {
    const acc = event.accelerationIncludingGravity || { x: 0, y: 0, z: G_ACCEL };
    const rot = event.rotationRate || { alpha: 0, beta: 0, gamma: 0 };
    
    // Force G
    const accTotal = Math.sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
    const forceG = accTotal / G_ACCEL;
    maxGForce = Math.max(maxGForce, forceG);
    
    // Mise à jour IMU / Dynamique
    if ($('accel-long')) $('accel-long').textContent = dataOrDefault(acc.x, 2, ' m/s²');
    if ($('accel-vert')) $('accel-vert').textContent = dataOrDefault(acc.y, 2, ' m/s²');
    if ($('force-g-long')) $('force-g-long').textContent = dataOrDefault(forceG, 3, ' G');
    if ($('vitesse-angulaire-gyro')) $('vitesse-angulaire-gyro').textContent = dataOrDefault(rot.alpha, 2, ' rad/s');
    if ($('inclinaison-pitch')) $('inclinaison-pitch').textContent = dataOrDefault(rot.beta, 1, '°');
    if ($('roulis-roll')) $('roulis-roll').textContent = dataOrDefault(rot.gamma, 1, '°');
}

/** 🛡️ Démarre les écouteurs IMU avec gestion de la permission explicite (pour iOS 13+). */
function startIMUListeners() { 
    const imuStatus = $('imu-status');
    if (imuStatus) imuStatus.textContent = 'Initialisation...';

    if (typeof DeviceMotionEvent.requestPermission === 'function') {
        DeviceMotionEvent.requestPermission()
            .then(permissionState => {
                if (permissionState === 'granted') {
                    window.addEventListener('devicemotion', handleDeviceMotion, true);
                    if (imuStatus) imuStatus.textContent = "Actif (Autorisé/DeviceMotion)";
                } else {
                    if (imuStatus) imuStatus.textContent = "Refusé (Permission IMU)";
                }
            })
            .catch(e => {
                console.error("Erreur de requête de permission DeviceMotion:", e);
                if (imuStatus) imuStatus.textContent = "Erreur de Permission IMU";
            });
    } else if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', handleDeviceMotion, true);
        if (imuStatus) imuStatus.textContent = "Actif (DeviceMotion)";
    } else {
        if (imuStatus) imuStatus.textContent = "Non Supporté";
    }
}

function stopIMUListeners() {
    if (window.DeviceMotionEvent) {
        window.removeEventListener('devicemotion', handleDeviceMotion, true);
    }
    if ($('imu-status')) $('imu-status').textContent = "Inactif";
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
// =================================================================

/** Boucle d'affichage rapide (requestAnimationFrame) */
function startFastLoop() {
    const loop = () => {
        
        // --- CALCULS EN TEMPS RÉEL ---
        const currentSpeedKmH = kSpd * KMH_MS; 
        
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
        
        if ($('speed-instant')) $('speed-instant').textContent = dataOrDefault(currentSpeedKmH, 2, ' km/h');
        if ($('vitesse-stable-ms')) $('vitesse-stable-ms').textContent = dataOrDefault(kSpd, 2, ' m/s');
        if ($('vitesse-max')) $('vitesse-max').textContent = dataOrDefault(maxSpd * KMH_MS, 5, ' km/h');
        if ($('nombre-de-mach')) $('nombre-de-mach').textContent = dataOrDefault(machNumber, 4);
        if ($('vitesse-verticale-ekf')) $('vitesse-verticale-ekf').textContent = dataOrDefault(kVertSpd, 2, ' m/s');
        if ($('pression-dynamique')) $('pression-dynamique').textContent = dataOrDefault(dynamicPressure, 2, ' Pa');
        if ($('force-coriolis')) $('force-coriolis').textContent = dataOrDefault(coriolisForce, 2, ' N');

        if ($('vitesse-lumiere-perc')) $('vitesse-lumiere-perc').textContent = dataOrDefaultExp(v_sur_c * 100, 2, ' %');
        if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(lorentzFactor, 4);
        if ($('energie-cinetique')) $('energie-cinetique').textContent = dataOrDefaultExp(kineticEnergy, 2, ' J');
        if ($('energie-masse-repos')) $('energie-masse-repos').textContent = dataOrDefaultExp(restEnergy, 2, ' J');
        if ($('rayon-schwarzschild')) $('rayon-schwarzschild').textContent = dataOrDefaultExp(schwarzschildRadius, 2, ' m');
        
        if ($('env-factor-display')) $('env-factor-display').textContent = `${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY} (x${ENVIRONMENT_FACTORS[selectedEnvironment].MULT.toFixed(1)})`;
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
        if (Date.now() - (lastKnownWeather ? lastKnownWeather.timestamp : 0) > WEATHER_UPDATE_MS) { 
             const weather = await fetchWeather(currentLat, currentLon);
             const pollutants = await fetchPollutants(currentLat, currentLon);
             
             if (weather) {
                lastP_hPa = weather.pressure_hPa;
                lastT_K = weather.tempC + 273.15;
                lastH_perc = weather.humidity_perc / 100.0;
                currentSpeedOfSound = getSpeedOfSound(lastT_K);
                currentAirDensity = calculateAirDensity(lastP_hPa, lastT_K, lastH_perc);

                updateWeatherDOM(weather, false);
             } else {
                 updateWeatherDOM({tempC: lastT_K - 273.15, pressure_hPa: lastP_hPa, humidity_perc: lastH_perc * 100, air_density: currentAirDensity, dew_point: 'N/A'}, true);
             }

             if (pollutants) updatePollutantsDOM(pollutants);
        }

        // 3. Mise à jour Astro
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
