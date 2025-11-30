// =================================================================
// BLOC 1/5 : CONSTANTES, ÉTAT GLOBAL ÉTENDU & UTILITAIRES CRITIQUES
// =================================================================

const $ = id => document.getElementById(id);

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const KMH_MS = 3.6;         
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const G_ACCEL = 9.80665;    // Gravité standard (m/s²)
const G_CONST = 6.67430e-11; // Constante gravitationnelle (N(m/kg)²)
const MASS_EARTH = 5.972e24; // Masse de la Terre (kg)
const R_EARTH = 6371000;    // Rayon terrestre moyen (m)
const RHO_SEA_LEVEL = 1.225; // Densité de l'air ISA (kg/m³)
const TEMP_SEA_LEVEL_K = 288.15; // 15°C en Kelvin
const BARO_ALT_REF_HPA = 1013.25; 
const NETHER_RATIO = 1 / 8; // Ratio Minecraft

// --- CONFIGURATIONS ET ENDPOINTS ---
const GPS_OPTS = {
    'HIGH_FREQ': { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    'LOW_FREQ': { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";
const DOM_SLOW_UPDATE_MS = 1000; 
const WEATHER_UPDATE_MS = 30000; 

// --- VARIABLES D'ÉTAT CRITIQUES ET EXTENDUES ---
let wID = null;             // ID de watchPosition
let domFastID = null;       // ID pour la boucle rapide
let domSlowID = null;       // ID pour la boucle lente
let sessionStartTime = Date.now();
let emergencyStopActive = false;
let isNightMode = false;
let netherMode = false;
let currentUKFReactivity = 'Automatique'; // UKF Debug
let currentMass = 70.0;
let currentCelestialBody = 'Terre';
let rotationRadius = 100;
let angularVelocity = 0.0;
let distanceRatioMode = false; // Toggle Rapport Distance

// Données Météo / Environnement (pour UKF)
let currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound = 343.20; 
let lastT_K = TEMP_SEA_LEVEL_K;
let lastP_hPa = BARO_ALT_REF_HPA;
let lastKnownWeather = null;
let lastKnownPollutants = null;

// Données de session
let distM = 0.0;            // Distance totale parcourue (m)
let maxSpd = 0.0;           // Vitesse max (m/s)
let timeMoving = 0.0;       // Temps de mouvement (s)
let maxGForce = 0.0;

// Données EKF/UKF (estimées)
let kSpd = 0.0;             // Vitesse UKF (m/s)
let kAlt = 0.0;             // Altitude UKF (m)
let kUncert = 1000.0;       // Incertitude Vitesse (P)
let kAltUncert = 1000.0;    // Incertitude Alt. (σ)
let currentLat = 0.0;       // Latitude EKF
let currentLon = 0.0;       // Longitude EKF
let kVertSpd = 0.0;         // Vitesse verticale EKF

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
        this.vertSpd = 0.0; // Vitesse verticale EKF
        this.status = 'INACTIF'; 
        this.x = [lat, lon, 0, 0, 0, 0]; // État minimal: Lat, Lon, V_N, V_E, Alt, V_Alt
    }
    // Méthode critique de mise à jour (fusion)
    update(position, imuData) {
        // Logique de fusion (fictive)
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

/** ⌚ Récupération de l'heure NTP */
async function syncH() { 
    try {
        const response = await fetch(SERVER_TIME_ENDPOINT);
        const data = await response.json();
        const serverTimeMS = new Date(data.utc_datetime).getTime();
        systemClockOffsetMS = serverTimeMS - Date.now();
    } catch (e) {
        systemClockOffsetMS = 0; // Utilisation de l'heure locale si échec
    }
}

/** 🌦️ Simulation d'une récupération Météo (pour éviter ReferenceError) */
async function fetchWeather(lat, lon) {
    // Dans la version complète, ceci ferait un fetch API
    lastKnownWeather = {
        tempC: lastT_K - 273.15,
        pressure_hPa: lastP_hPa,
        humidity_perc: 75,
        air_density: currentAirDensity,
        dew_point: 10.0 
    };
    return lastKnownWeather;
}

/** 🚨 Simulation d'une récupération Polluants (pour éviter ReferenceError) */
async function fetchPollutants(lat, lon) {
    lastKnownPollutants = {
        NO2: 25.5, PM25: 12.2, PM10: 18.0, O3: 50.0
    };
    return lastKnownPollutants;
}

/** 🔭 Mise à jour complète de la section Astro (Placeholder) */
function updateAstro(lat, lon, date) {
    // Cette fonction utiliserait SunCalc/AstroJS
    if ($('longitude-ecliptique')) $('longitude-ecliptique').textContent = 'N/A';
    if ($('altitude-soleil')) $('altitude-soleil').textContent = 'N/A';
    if ($('azimut-soleil')) $('azimut-soleil').textContent = 'N/A';
    if ($('lever')) $('lever').textContent = 'N/A';
    if ($('coucher')) $('coucher').textContent = 'N/A';
    // ... et tous les autres champs astro
}

/** Calcul de l'Heure Minecraft (Placeholder) */
function getMinecraftTime(date) {
    // Logique complète (24000 ticks/jour)
    const msSinceMidnight = date.getHours() * 3600000 + date.getMinutes() * 60000;
    const mcTicks = Math.floor((msSinceMidnight / 86400000) * 24000);
    const mcHour = Math.floor((mcTicks / 1000 + 6) % 24); 
    const mcMinute = Math.floor(((mcTicks % 1000) / 1000) * 60);
    return `${String(mcHour).padStart(2, '0')}:${String(mcMinute).padStart(2, '0')}`;
}

/** Fonction de mise à jour du corps céleste (Gravité) */
function updateCelestialBody(body, alt, radius, angular) { 
    // Logique pour Lune, Mars, ou corps rotatifs (Centrifuge)
    let newG = G_ACCEL;
    if (body === 'Lune') newG = G_ACCEL * 0.165;
    // La logique de force centrifuge/Coriolis serait dans startFastLoop, mais la référence de base est mise à jour ici.
    return { G_ACC_NEW: newG };
}

/** Calcul du Rayon de Schwarzschild (Rs) (Placeholder) */
function calculateSchwarzschildRadius(mass) {
    // Rs = 2 * G * M / c²
    return (2 * G_CONST * mass) / (C_L * C_L);
}

/** Calcul de la Force de Coriolis (Placeholder) */
function calculateCoriolisForce(mass, speed, lat) {
    const W_EARTH = 7.2921E-5; // Vitesse angulaire de la Terre (rad/s)
    return 2 * mass * W_EARTH * speed * Math.sin(lat * D2R);
        }
// =================================================================
// BLOC 3/5 : LOGIQUE DE CONTRÔLE GPS & IMU (startGPS/stopGPS & Capteurs)
// CORRECTION: Ajout de la gestion des permissions IMU (DeviceMotionEvent.requestPermission)
// =================================================================

// --- GESTION DES CAPTEURS ---

function gpsSuccess(position) { 
    const coords = position.coords;
    
    // 1. Mise à jour des variables globales de position
    currentLat = coords.latitude;
    currentLon = coords.longitude;

    // 2. Mise à jour du filtre UKF (Vitesse et Altitude estimées)
    ukf.update(position, {}); 
    kSpd = ukf.speed; 
    kAlt = ukf.altitude;
    kUncert = ukf.uncertainty;
    kAltUncert = ukf.altUncert;
    kVertSpd = ukf.vertSpd;
    
    // Mise à jour de la précision affichée
    if ($('precision-gps')) $('precision-gps').textContent = dataOrDefault(coords.accuracy, 3, ' m', 'N/A');
}

function gpsError(error) { 
    let message = `ERREUR (${error.code})`;
    if (error.code === 1) message = "Permission GPS Refusée";
    else if (error.code === 2) message = "Position Indisponible";
    else if (error.code === 3) message = "Timeout GPS";
    else if (error.code === 99) message = error.message; // Pour les erreurs de support

    console.error("Erreur GPS:", message, error.message);
    if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = message;
}

/** Handler pour l'événement DeviceMotion (pour la Force G et le niveau à bulle) */
function handleDeviceMotion(event) {
    const acc = event.accelerationIncludingGravity || { x: 0, y: 0, z: G_ACCEL };
    const rot = event.rotationRate || { alpha: 0, beta: 0, gamma: 0 };
    
    // Force G et Accélérations
    const accTotal = Math.sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
    const forceG = accTotal / G_ACCEL;
    maxGForce = Math.max(maxGForce, forceG);
    
    if ($('accel-long')) $('accel-long').textContent = dataOrDefault(acc.x, 2, ' m/s²');
    if ($('accel-vert')) $('accel-vert').textContent = dataOrDefault(acc.y, 2, ' m/s²');
    if ($('force-g-long')) $('force-g-long').textContent = dataOrDefault(forceG, 3, ' G');
    
    // Vitesse Angulaire et Niveau à Bulle
    if ($('vitesse-angulaire-gyro')) $('vitesse-angulaire-gyro').textContent = dataOrDefault(rot.alpha, 2, ' rad/s');
    if ($('inclinaison-pitch')) $('inclinaison-pitch').textContent = dataOrDefault(rot.beta, 1, '°');
    if ($('roulis-roll')) $('roulis-roll').textContent = dataOrDefault(rot.gamma, 1, '°');
}

/** 🛡️ Démarre les écouteurs IMU avec gestion de la permission explicite (iOS 13+). */
function startIMUListeners() { 
    const imuStatus = $('imu-status');
    imuStatus.textContent = 'Initialisation...';

    // 1. Vérification de la permission explicite (nécessaire pour iOS 13+ et certains navigateurs)
    if (typeof DeviceMotionEvent.requestPermission === 'function') {
        DeviceMotionEvent.requestPermission()
            .then(permissionState => {
                if (permissionState === 'granted') {
                    window.addEventListener('devicemotion', handleDeviceMotion, true);
                    imuStatus.textContent = "Actif (Autorisé/DeviceMotion)";
                } else {
                    imuStatus.textContent = "Refusé (Permission IMU)";
                }
            })
            .catch(e => {
                console.error("Erreur de requête de permission DeviceMotion:", e);
                imuStatus.textContent = "Erreur de Permission IMU";
            });
    } else if (window.DeviceMotionEvent) {
        // 2. Anciens navigateurs ou Android (où la permission est gérée par l'OS)
        window.addEventListener('devicemotion', handleDeviceMotion, true);
        imuStatus.textContent = "Actif (DeviceMotion)";
    } else {
        // 3. Fonction non supportée
        imuStatus.textContent = "Non Supporté";
    }
}

function stopIMUListeners() {
    if (window.DeviceMotionEvent) {
        window.removeEventListener('devicemotion', handleDeviceMotion, true);
    }
    if ($('imu-status')) $('imu-status').textContent = "Inactif";
}


// --- GESTION DU GPS (START/STOP) ---

/** 🛡️ Démarre l'acquisition GPS et les capteurs IMU (avec gestion des permissions). */
function startGPS(mode = 'HIGH_FREQ') {
    if (wID !== null || emergencyStopActive) return; 
    
    // 1. Démarrer les capteurs IMU (gestion des permissions)
    startIMUListeners(); 

    // 2. Démarrer la géolocalisation (Vérification de support)
    if (!navigator.geolocation) {
        gpsError({ code: 99, message: "La Géolocalisation n'est pas supportée par ce navigateur." });
        return;
    }

    // Le watchPosition gère la demande de permission Geolocation
    wID = navigator.geolocation.watchPosition(gpsSuccess, gpsError, GPS_OPTS[mode]);
    
    // 3. Mettre à jour l'affichage
    if ($('start-btn')) $('start-btn').innerHTML = '⏸️ PAUSE GPS'; 
    if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = `Actif (Mode ${mode})`;
    
    // 4. Assurer que la boucle d'affichage rapide est lancée
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
}
// =================================================================
// BLOC 4/5 : BOUCLES DE RAFRAÎCHISSEMENT (FAST & SLOW)
// =================================================================

/** Boucle d'affichage rapide (requestAnimationFrame) */
function startFastLoop() {
    const loop = () => {
        
        // --- CALCULS EN TEMPS RÉEL ---
        const currentSpeedKmH = kSpd * KMH_MS; 
        const currentSpeedKmS = kSpd / 1000;
        
        // Relativité
        const v_sur_c = kSpd / C_L;
        const lorentzFactor = 1 / Math.sqrt(1 - v_sur_c * v_sur_c);
        const kineticEnergy = 0.5 * currentMass * Math.pow(kSpd, 2);
        const restEnergy = currentMass * C_L * C_L;
        const schwarzschildRadius = calculateSchwarzschildRadius(currentMass);
        const momentum = currentMass * kSpd * lorentzFactor;
        
        // Dynamique
        const dynamicPressure = 0.5 * currentAirDensity * Math.pow(kSpd, 2);
        const machNumber = kSpd / currentSpeedOfSound;
        const coriolisForce = calculateCoriolisForce(currentMass, kSpd, currentLat);

        // --- MISE À JOUR DU DOM (VITESSE, RELATIVITÉ, DYNAMIQUE) ---
        
        // Vitesse
        if ($('speed-instant')) $('speed-instant').textContent = dataOrDefault(currentSpeedKmH, 2, ' km/h');
        if ($('vitesse-stable-ms')) $('vitesse-stable-ms').textContent = dataOrDefault(kSpd, 2, ' m/s');
        if ($('vitesse-stable-kms')) $('vitesse-stable-kms').textContent = dataOrDefault(currentSpeedKmS, 5, ' km/s');
        if ($('vitesse-max')) $('vitesse-max').textContent = dataOrDefault(maxSpd * KMH_MS, 5, ' km/h');

        // Relativité
        if ($('vitesse-son-locale')) $('vitesse-son-locale').textContent = dataOrDefault(currentSpeedOfSound, 2, ' m/s');
        if ($('vitesse-son-perc')) $('vitesse-son-perc').textContent = dataOrDefault((kSpd / currentSpeedOfSound) * 100, 2, ' %');
        if ($('nombre-de-mach')) $('nombre-de-mach').textContent = dataOrDefault(machNumber, 4);
        if ($('vitesse-lumiere-perc')) $('vitesse-lumiere-perc').textContent = dataOrDefaultExp(v_sur_c * 100, 2, ' %');
        if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(lorentzFactor, 4);
        if ($('energie-relativiste')) $('energie-relativiste').textContent = dataOrDefaultExp(kineticEnergy + restEnergy, 2, ' J');
        if ($('energie-masse-repos')) $('energie-masse-repos').textContent = dataOrDefaultExp(restEnergy, 2, ' J');
        if ($('quantite-de-mouvement')) $('quantite-de-mouvement').textContent = dataOrDefaultExp(momentum, 2, ' kg·m/s');
        if ($('rayon-schwarzschild')) $('rayon-schwarzschild').textContent = dataOrDefaultExp(schwarzschildRadius, 2, ' m');

        // Dynamique
        if ($('pression-dynamique')) $('pression-dynamique').textContent = dataOrDefault(dynamicPressure, 2, ' Pa');
        if ($('force-coriolis')) $('force-coriolis').textContent = dataOrDefault(coriolisForce, 2, ' N');
        if ($('vitesse-verticale-ekf')) $('vitesse-verticale-ekf').textContent = dataOrDefault(kVertSpd, 2, ' m/s');
        
        // Demande la prochaine frame
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

        // 2. Mise à jour Météo (toutes les 30s - simulation)
        if (Date.now() % WEATHER_UPDATE_MS < DOM_SLOW_UPDATE_MS) {
             const weather = await fetchWeather(currentLat, currentLon);
             const pollutants = await fetchPollutants(currentLat, currentLon);
             
             // Mise à jour Météo
             if ($('statut-meteo')) $('statut-meteo').textContent = 'ACTIF (API)';
             if ($('temp-air')) $('temp-air').textContent = dataOrDefault(weather.tempC, 1, ' °C');
             if ($('pression-atmospherique')) $('pression-atmospherique').textContent = dataOrDefault(weather.pressure_hPa, 0, ' hPa');
             if ($('densite-air')) $('densite-air').textContent = dataOrDefault(weather.air_density, 3, ' kg/m³');

             // Mise à jour Polluants
             if ($('no2')) $('no2').textContent = dataOrDefault(pollutants.NO2, 1, ' µg/m³');
             if ($('pm25')) $('pm25').textContent = dataOrDefault(pollutants.PM25, 1, ' µg/m³');
             if ($('pm10')) $('pm10').textContent = dataOrDefault(pollutants.PM10, 1, ' µg/m³');
             if ($('o3')) $('o3').textContent = dataOrDefault(pollutants.O3, 1, ' µg/m³');
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
// =================================================================

/** Réinitialise les variables d'affichage (distance, max speed) */
function resetDisp(fullReset = false) {
    distM = 0; 
    maxSpd = 0; 
    maxGForce = 0;
    timeMoving = 0;
    // Si reset total, réinitialiser UKF
    if (fullReset) {
        ukf = new ProfessionalUKF(currentLat, currentLon, RHO_SEA_LEVEL);
        currentLat = 0.0; currentLon = 0.0; kAlt = 0.0; kSpd = 0.0;
        sessionStartTime = Date.now();
    }
}

/** Inverse le mode nuit */
function toggleNightMode() {
    isNightMode = !isNightMode;
    document.body.classList.toggle('night-mode', isNightMode);
    if ($('mode-nuit-btn')) $('mode-nuit-btn').textContent = isNightMode ? "☀️ Mode Jour" : "🌗 Mode Nuit";
}


/** Configure tous les écouteurs d'événements pour les boutons et les inputs. */
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
    
    if ($('toggle-mode-btn')) $('toggle-mode-btn').addEventListener('click', toggleNightMode); // Mode Nuit
    
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
    
    // Mise à jour des champs de rotation
    const updateRotation = () => {
        rotationRadius = parseFloat($('rotation-radius').value) || 100;
        angularVelocity = parseFloat($('angular-velocity').value) || 0.0;
        if (currentCelestialBody === 'ROTATING') {
            const { G_ACC_NEW } = updateCelestialBody('ROTATING', kAlt, rotationRadius, angularVelocity);
            $('gravite-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
        }
    };
    if ($('rotation-radius')) $('rotation-radius').addEventListener('input', updateRotation);
    if ($('angular-velocity')) $('angular-velocity').addEventListener('input', updateRotation);
    
    if ($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', () => {
        netherMode = !netherMode;
        if ($('mode-nether')) $('mode-nether').textContent = netherMode ? 'ACTIF (1:8) 🔥' : 'DÉSACTIVÉ (1:1) 🌍';
    });
    
    if ($('ukf-reactivity-mode')) $('ukf-reactivity-mode').addEventListener('change', (e) => currentUKFReactivity = e.target.value);

    // --- DÉMARRAGE DU SYSTÈME ---
    // Initialisation de l'affichage par défaut
    if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
    if ($('gravite-base')) $('gravite-base').textContent = `${G_ACCEL.toFixed(4)} m/s²`;
    
}

/** Fonction d'initialisation principale */
function init() {
    // 1. Démarrage de la synchro NTP
    syncH(); 
    
    // 2. Démarrage des boucles lentes (Météo/Astro/1Hz)
    startSlowLoop(); 
    
    // 3. Démarrage de la boucle d'affichage rapide (pour les valeurs par défaut)
    startFastLoop(); 
    
    // 4. Initialisation des gestionnaires d'événements
    initControls(); 
}

// Lancement du système au chargement complet de la page
document.addEventListener('DOMContentLoaded', init);
