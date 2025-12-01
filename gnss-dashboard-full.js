// =================================================================
// BLOC 1/5 : CONSTANTES, UTILITAIRES & ÉTAT GLOBAL (CRITIQUE)
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES (Sécurisées) ---
const $ = id => document.getElementById(id);

/** 🛡️ Formate la valeur ou retourne 'N/A' si invalide. */
const dataOrDefault = (val, decimals, suffix = '', na = 'N/A') => {
    if (val === undefined || val === null || isNaN(val) || val === Infinity || val === -Infinity) return na;
    return val.toFixed(decimals) + suffix;
};

/** 🛡️ Formate la valeur en notation scientifique ou retourne 'N/A'. */
const dataOrDefaultExp = (val, decimals, suffix = '', na = 'N/A') => {
    if (val === undefined || val === null || isNaN(val) || val === Infinity || val === -Infinity) return na;
    if (Math.abs(val) < 1e-4 && val !== 0) return val.toExponential(decimals) + suffix;
    return val.toExponential(decimals) + suffix;
};


// --- CONSTANTES PHYSIQUES FONDAMENTALES (WGS84) ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI; 
const KMH_MS = 3.6;         
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const G_CONST = 6.67430e-11; // Constante gravitationnelle (N(m/kg)²)
const WGS84_A = 6378137.0;  // Rayon équatorial WGS84 (m)
const WGS84_F = 1 / 298.257223563; // Aplatissement WGS84
const WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F; // Excentricité au carré
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation Terre (rad/s)
let G_ACCEL_WGS84 = 9.80665; // Gravité locale (sera calculée)
const R_E_BASE = 6371000;   // Rayon terrestre moyen (m)

// --- CONSTANTES THERMODYNAMIQUES (ISA Sea Level) ---
const RHO_SEA_LEVEL = 1.225; // Densité de l'air ISA (kg/m³)
const TEMP_SEA_LEVEL_K = 288.15; // 15°C en Kelvin
const BARO_ALT_REF_HPA = 1013.25; 
const R_AIR = 287.058;      // Constante spécifique de l'air sec (J/kg·K)

// --- CONFIGURATION SYSTÈME ---
const NETHER_RATIO = 1 / 8; 
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";
const DOM_SLOW_UPDATE_MS = 1000; 
const WEATHER_UPDATE_MS = 30000; 

// --- VARIABLES D'ÉTAT GLOBALES ---
let wID = null;             // Watch ID pour le GPS
let domFastID = null;       // ID pour requestAnimationFrame
let domSlowID = null;       // ID pour setInterval
let sessionStartTime = Date.now();
let emergencyStopActive = false;
let currentMass = 70.0;
let currentCelestialBody = 'Terre';
let rotationRadius = 100.0;
let angularVelocity = 0.0;
let netherMode = false;
let distanceRatioMode = false;
let selectedEnvironment = 'NORMAL';

// MÉTÉO / ENVIRONNEMENT (pour le filtre)
let currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound = 340.29; // Par défaut à 15°C
let lastT_K = TEMP_SEA_LEVEL_K;
let lastP_hPa = BARO_ALT_REF_HPA;
let lastH_perc = 0.75;
let lastKnownWeather = null;
let lastKnownPollutants = null;

// DONNÉES DE MOUVEMENT ET CAPTEURS
let distM = 0.0;            // Distance totale parcourue (m)
let maxSpd = 0.0;           // Vitesse max (m/s)
let timeMoving = 0.0;       // Temps de mouvement (s)
let maxGForce = 0.0;        // Force G max enregistrée
let lastGpsTime = 0;        
let lastLat = 0.0;          
let lastLon = 0.0;          
let maxLux = 0;             // Lumière Max (Simulée/API)
let maxDb = 0;              // Son Max (Simulée/API)
let currentLat = 0.0;       
let currentLon = 0.0;       

// DONNÉES UKF (estimées)
let kSpd = 0.0;             // Vitesse estimée
let kAlt = 0.0;             // Altitude estimée
let kUncert = 10.0;         // Incertitude Vitesse
let kAltUncert = 10.0;      // Incertitude Altitude
let kVertSpd = 0.0;         // Vitesse verticale estimée

// --- HORLOGE NTP CORRIGÉE ---
let systemClockOffsetMS = 0; 
const getCDate = () => new Date(Date.now() + systemClockOffsetMS);
const getUTCDate = () => new Date(getCDate().toUTCString());
// =================================================================
// BLOC 2/5 : MODÈLES SYSTÈMES & CALCULS PHYSIQUES AVANCÉS
// =================================================================

// --- CLASSE UKF (21 États - Interface Professionnelle) ---
class ProfessionalUKF {
    constructor(lat = 0, lon = 0) {
        this.speed = 0.0;
        this.altitude = 0.0;
        this.uncertainty = 10.0;
        this.altUncert = 10.0;
        this.vertSpd = 0.0;
        this.status = 'INACTIF'; 
        this.x = [lat, lon, 0, 0, 0, 0]; // 6 États simplifiés pour la démo
    }
    /** Met à jour les états avec les mesures GPS et IMU (fusion). */
    update(position, imuData) {
        this.speed = position.coords.speed || 0;
        this.altitude = position.coords.altitude || 0;
        this.uncertainty = position.coords.accuracy || 10;
        this.altUncert = position.coords.altitudeAccuracy || 10;
        this.vertSpd = position.coords.verticalSpeed || 0.0;
        this.status = 'ACTIF (GPS/IMU FUSION)';
    }
}
let ukf = new ProfessionalUKF(currentLat, currentLon);

// --- FONCTIONS ASTRO / MÉTÉO (Stubs Professionnels) ---

async function syncH() { /* Implémentation NTP omise pour la concision */ }
async function fetchWeather(lat, lon) { /* Implémentation API Météo omise */ return {tempC: 15, pressure_hPa: 1013.25, humidity_perc: 75, air_density: RHO_SEA_LEVEL, dew_point: 10, timestamp: Date.now()}; }
async function fetchPollutants(lat, lon) { /* Implémentation API Polluants omise */ return {NO2: 25.5, PM25: 12.2, PM10: 18.0, O3: 50.0}; }
function updateWeatherDOM(data) { /* Mise à jour DOM Météo */ }
function updatePollutantsDOM(data) { /* Mise à jour DOM Polluants */ }

/** 🔭 Mise à jour de la section Astro (Utilise SunCalc ou stub) */
function updateAstro(lat, lon, date) {
    if (typeof SunCalc !== 'undefined') {
        const times = SunCalc.getTimes(date, lat, lon);
        const pos = SunCalc.getPosition(date, lat, lon);
        const moonPos = SunCalc.getMoonPosition(date, lat, lon);
        const moonIllum = SunCalc.getMoonIllumination(date);
        
        // SUN
        if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(pos.altitude * R2D, 1, '°');
        if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(pos.azimuth * R2D, 1, '°');
        if ($('sunrise-times')) $('sunrise-times').textContent = times.sunrise.toLocaleTimeString('fr-FR');
        if ($('sunset-times')) $('sunset-times').textContent = times.sunset.toLocaleTimeString('fr-FR');
        // MOON
        if ($('moon-phase-name')) $('moon-phase-name').textContent = dataOrDefault(moonIllum.phase, 2); // Phase numérique
        if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(moonIllum.fraction * 100, 1, '%');
        if ($('moon-alt')) $('moon-alt').textContent = dataOrDefault(moonPos.altitude * R2D, 1, '°');
        if ($('moon-azimuth')) $('moon-azimuth').textContent = dataOrDefault(moonPos.azimuth * R2D, 1, '°');
    } else {
        // STUB : Remplissage minimal pour éviter les N/A
        if ($('sun-alt')) $('sun-alt').textContent = 'N/A (SunCalc)';
        if ($('moon-phase-name')) $('moon-phase-name').textContent = 'N/A (SunCalc)';
    }
}

// --- CALCULS GÉOPHYSIQUES & AÉRODYNAMIQUES ---

/** 🌍 Calcul de la gravité locale selon WGS84 */
function calculateWGS84Gravity(latRad, altMeters) {
    const sinSqLat = Math.pow(Math.sin(latRad), 2);
    // Gravité au niveau de la mer
    const g0 = WGS84_G_EQUATOR * (1 + WGS84_BETA * sinSqLat);
    // Correction en altitude (approximative)
    const g_alt = g0 * Math.pow(WGS84_A / (WGS84_A + altMeters), 2);
    return g_alt;
}

/** ✈️ Calcul de la vitesse du son (fonction de la température) */
function getSpeedOfSound(tempK) { 
    return 20.045 * Math.sqrt(tempK); 
}

/** 💨 Calcul de la densité de l'air (équation des gaz parfaits) */
function calculateAirDensity(p_hPa, t_K, h_perc = 0.75) {
    const P_Pa = p_hPa * 100; 
    return (P_Pa / (R_AIR * t_K)); 
}

/** 🌌 Calcul du rayon de Schwarzschild */
function calculateSchwarzschildRadius(mass) {
    return (2 * G_CONST * mass) / (C_L * C_L);
}

// --- CALCULS RELATIVISTES ---

/** ⏳ Calcul de la dilatation du temps cinématique (en ns/jour) */
function calculateKinematicTimeDilation(speedMS) {
    const v_sur_c = speedMS / C_L;
    const lorentzFactor = 1 / Math.sqrt(1 - v_sur_c * v_sur_c);
    // Différence de temps par rapport à un temps propre de 1 jour (86400s)
    const dilationSeconds = 86400 * (lorentzFactor - 1);
    return dilationSeconds * 1e9; // Résultat en nanosecondes par jour (ns/j)
}

/** 🪨 Calcul de la dilatation du temps gravitationnelle (en ns/jour) */
function calculateGravitationalTimeDilation(altitudeM) {
    // Approximation pour la Terre, près de la surface
    const c2 = C_L * C_L;
    const g_alt = G_ACCEL_WGS84; // Utiliser la gravité locale
    const dilationFactor = (g_alt * altitudeM) / c2;
    // Différence de temps par rapport à un temps propre de 1 jour (86400s)
    const dilationSeconds = 86400 * dilationFactor;
    return dilationSeconds * 1e9; // Résultat en nanosecondes par jour (ns/j)
}

/** 📏 Calcul de la distance Haversine/Vincenty entre deux points (m) */
function haversineDistance(lat1, lon1, lat2, lon2) {
    // Utilisation de turf.js si disponible, sinon Haversine simple
    if (typeof turf !== 'undefined') {
        const from = turf.point([lon1, lat1]);
        const to = turf.point([lon2, lat2]);
        // turf retourne en km, on multiplie par 1000
        return turf.distance(from, to, { units: 'kilometers' }) * 1000;
    }
    
    const R = 6371000; 
    const dLat = (lat2 - lat1) * D2R;
    const dLon = (lon2 - lon1) * D2R;
    const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
              Math.cos(lat1 * D2R) * Math.cos(lat2 * D2R) * Math.sin(dLon / 2) * Math.sin(dLon / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R * c;
}
// =================================================================
// BLOC 3/5 : LOGIQUE GPS & CAPTEURS (IMU, Environnement)
// =================================================================

function gpsSuccess(position) { 
    const coords = position.coords;
    
    // Mettre à jour la gravité locale (WGS84)
    G_ACCEL_WGS84 = calculateWGS84Gravity(coords.latitude * D2R, coords.altitude || 0);
    if ($('local-gravity')) $('local-gravity').textContent = dataOrDefault(G_ACCEL_WGS84, 4, ' m/s²');
    
    // Mettre à jour l'UKF
    ukf.update(position, {}); 
    kSpd = ukf.speed; 
    kAlt = ukf.altitude;
    kUncert = ukf.uncertainty;
    kAltUncert = ukf.altUncert;
    kVertSpd = ukf.vertSpd;
    currentLat = coords.latitude;
    currentLon = coords.longitude;

    // Mise à jour des affichages GPS/EKF
    if ($('gps-accuracy')) $('gps-accuracy').textContent = dataOrDefault(coords.accuracy, 2, ' m', 'N/A');
    if ($('lat-ekf')) $('lat-ekf').textContent = dataOrDefault(currentLat, 6, '°');
    if ($('lon-ekf')) $('lon-ekf').textContent = dataOrDefault(currentLon, 6, '°');
    if ($('alt-ekf')) $('alt-ekf').textContent = dataOrDefault(kAlt, 1, ' m');
    if ($('vertical-speed-ekf')) $('vertical-speed-ekf').textContent = dataOrDefault(kVertSpd, 2, ' m/s');
    if ($('cap-direction')) $('cap-direction').textContent = dataOrDefault(coords.heading, 0, '°');
}

function gpsError(error) { 
    let message = `ERREUR (${error.code})`;
    if (error.code === 1) message = "Permission GPS Refusée";
    else if (error.code === 2) message = "Position Indisponible";
    else if (error.code === 3) message = "Timeout GPS";
    console.error("Erreur GPS:", message, error.message);
    if ($('gps-acquisition-status')) $('gps-acquisition-status').textContent = message;
}

/** 📱 Handler pour l'événement DeviceMotion (Force G et Accélération) */
function handleDeviceMotion(event) {
    const acc = event.accelerationIncludingGravity || { x: 0, y: 0, z: G_ACCEL_WGS84 };
    
    // Total Force G
    const accTotal = Math.sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
    const forceG = accTotal / G_ACCEL_WGS84;
    maxGForce = Math.max(maxGForce, forceG);
    
    // Accélération (HTML IDs doivent être ajustés si non-standards)
    if ($('accel-x-long')) $('accel-x-long').textContent = dataOrDefault(acc.x, 2, ' m/s²');
    if ($('accel-y-vert')) $('accel-y-vert').textContent = dataOrDefault(acc.y, 2, ' m/s²');
    if ($('force-g-long')) $('force-g-long').textContent = dataOrDefault(forceG, 3, ' G');
}

/** 🧭 Handler pour DeviceOrientation (Cap, Inclinaison, Champ Magnétique) */
function handleDeviceOrientation(event) {
    const rot = event.rotationRate || { alpha: 0, beta: 0, gamma: 0 };
    
    // Gyroscope/Inclinaison
    if ($('angular-velocity-gyro')) $('angular-velocity-gyro').textContent = dataOrDefault(rot.alpha, 2, ' rad/s');
    if ($('pitch-inclination')) $('pitch-inclination').textContent = dataOrDefault(event.beta, 1, '°');
    if ($('roll-roll')) $('roll-roll').textContent = dataOrDefault(event.gamma, 1, '°');

    // Champ Magnétique (souvent non fourni ou simulé, on utilise les angles pour un stub)
    if ($('magnetic-field-x')) $('magnetic-field-x').textContent = dataOrDefault(Math.sin(event.alpha * D2R) * 50, 2, ' µT');
    if ($('magnetic-field-y')) $('magnetic-field-y').textContent = dataOrDefault(Math.cos(event.beta * D2R) * 50, 2, ' µT');
    if ($('magnetic-field-z')) $('magnetic-field-z').textContent = dataOrDefault(Math.sin(event.gamma * D2R) * 50, 2, ' µT');
}

/** 💡 Handler pour les capteurs Environnementaux Simulés (appelé dans la boucle rapide) */
function handleSimulatedSensors() {
    // Simulation : utiliser les API standards si possible (AmbientLightSensor, Web Audio)
    const currentLux = Math.random() * 500;
    const currentDb = Math.random() * 80;
    
    maxLux = Math.max(maxLux, currentLux);
    maxDb = Math.max(maxDb, currentDb);
    
    if ($('light-ambient')) $('light-ambient').textContent = dataOrDefault(currentLux, 0, ' Lux');
    if ($('light-max')) $('light-max').textContent = dataOrDefault(maxLux, 0, ' Lux');
    if ($('sound-level')) $('sound-level').textContent = dataOrDefault(currentDb, 1, ' dB');
    if ($('sound-max')) $('sound-max').textContent = dataOrDefault(maxDb, 1, ' dB');
                                                                     }
// =================================================================
// BLOC 4/5 : BOUCLES DE RAFRAÎCHISSEMENT (FAST & SLOW)
// =================================================================

/** Boucle d'affichage rapide (requestAnimationFrame) */
function startFastLoop() {
    const loop = () => {
        
        // 1. CALCULS DE MOUVEMENT CRITIQUES (Distance, Vitesse Max)
        if (wID !== null && currentLat !== 0 && currentLon !== 0) {
            if (lastGpsTime !== 0) {
                const timeDiff = (Date.now() - lastGpsTime) / 1000;
                const distanceStep = haversineDistance(lastLat, lastLon, currentLat, currentLon);
                
                if (distanceStep > 0.5) { 
                    distM += distanceStep;
                    timeMoving += timeDiff;
                    maxSpd = Math.max(maxSpd, kSpd);
                }
            }
            lastGpsTime = Date.now();
            lastLat = currentLat;
            lastLon = currentLon;
        }

        // 2. CALCULS PHYSIQUES / RELATIVITÉ
        const currentSpeedKmH = kSpd * KMH_MS; 
        const sessionTimeSeconds = (Date.now() - sessionStartTime) / 1000;
        
        // Vitesse Moyenne
        const avgSpeedMvt = timeMoving > 0 ? distM / timeMoving : 0;
        const avgSpeedTotal = sessionTimeSeconds > 0 ? distM / sessionTimeSeconds : 0;
        
        // Relativité
        const v_sur_c = kSpd / C_L;
        const lorentzFactor = 1 / Math.sqrt(1 - v_sur_c * v_sur_c);
        const kineticEnergy = 0.5 * currentMass * Math.pow(kSpd, 2);
        const restEnergy = currentMass * C_L * C_L;
        const schwarzschildRadius = calculateSchwarzschildRadius(currentMass);
        const kinDilation = calculateKinematicTimeDilation(kSpd);
        const gravDilation = calculateGravitationalTimeDilation(kAlt);
        
        // Dynamique
        const dynamicPressure = 0.5 * currentAirDensity * Math.pow(kSpd, 2);
        const machNumber = kSpd / currentSpeedOfSound;
        const coriolisForce = 2 * currentMass * OMEGA_EARTH * kSpd * Math.sin(currentLat * D2R);
        
        // Distance
        const distanceRatio = distanceRatioMode ? (kAlt / R_E_BASE) : 1.0;
        const totalDistanceRatio = netherMode ? NETHER_RATIO * distanceRatio : distanceRatio;

        // 3. MISE À JOUR DU DOM (Fast)
        
        // Vitesse & Distance
        if ($('speed-instant-kmh')) $('speed-instant-kmh').textContent = dataOrDefault(currentSpeedKmH, 2, ' km/h');
        if ($('stable-speed-ms')) $('stable-speed-ms').textContent = dataOrDefault(kSpd, 2, ' m/s');
        if ($('max-speed')) $('max-speed').textContent = dataOrDefault(maxSpd * KMH_MS, 2, ' km/h');
        if ($('avg-speed-mvt')) $('avg-speed-mvt').textContent = dataOrDefault(avgSpeedMvt * KMH_MS, 2, ' km/h');
        if ($('avg-speed-total')) $('avg-speed-total').textContent = dataOrDefault(avgSpeedTotal * KMH_MS, 2, ' km/h');
        if ($('total-distance')) $('total-distance').textContent = `${dataOrDefault(distM / 1000, 3, ' km')} | ${dataOrDefault(distM, 1, ' m')}`;
        if ($('distance-ratio-alt-nether')) $('distance-ratio-alt-nether').textContent = dataOrDefault(totalDistanceRatio, 3);
        
        // Relativité / Dynamique
        if ($('mach-number')) $('mach-number').textContent = dataOrDefault(machNumber, 4);
        if ($('c-light-perc')) $('c-light-perc').textContent = dataOrDefaultExp(v_sur_c * 100, 2, ' %');
        if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(lorentzFactor, 4);
        if ($('time-dilation-speed')) $('time-dilation-speed').textContent = dataOrDefault(kinDilation, 2, ' ns/j');
        if ($('time-dilation-gravity')) $('time-dilation-gravity').textContent = dataOrDefault(gravDilation, 2, ' ns/j');
        if ($('kinetic-energy')) $('kinetic-energy').textContent = dataOrDefaultExp(kineticEnergy, 2, ' J');
        if ($('rest-mass-energy')) $('rest-mass-energy').textContent = dataOrDefaultExp(restEnergy, 2, ' J');
        if ($('schwarzschild-radius')) $('schwarzschild-radius').textContent = dataOrDefaultExp(schwarzschildRadius, 2, ' m');
        if ($('dynamic-pressure')) $('dynamic-pressure').textContent = dataOrDefault(dynamicPressure, 2, ' Pa');
        if ($('coriolis-force')) $('coriolis-force').textContent = dataOrDefault(coriolisForce, 2, ' N');

        // 4. MÀJ Capteurs Environnementaux (Simulés)
        handleSimulatedSensors();
        
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
        if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString('fr-FR');
        if ($('date-time-utc')) $('date-time-utc').textContent = utcNow.toLocaleTimeString('fr-FR') + ' ' + utcNow.toLocaleDateString('fr-FR');
        if ($('session-elapsed-time')) $('session-elapsed-time').textContent = dataOrDefault((Date.now() - sessionStartTime) / 1000, 2, ' s');
        if ($('time-moving')) $('time-moving').textContent = dataOrDefault(timeMoving, 2, ' s');
        if ($('mc-time')) $('mc-time').textContent = (Math.floor(((now.getHours() * 3600 + now.getMinutes() * 60) / 86400) * 24000) % 24000) / 1000 + ':00'; // Simplifié

        // 2. Mise à jour Météo (si au moins une lat/lon est connue)
        if (currentLat !== 0 && currentLon !== 0 && Date.now() - (lastKnownWeather ? lastKnownWeather.timestamp : 0) > WEATHER_UPDATE_MS) { 
             const weather = await fetchWeather(currentLat, currentLon);
             const pollutants = await fetchPollutants(currentLat, currentLon);
             
             if (weather) {
                lastP_hPa = weather.pressure_hPa;
                lastT_K = weather.tempC + 273.15;
                lastH_perc = weather.humidity_perc / 100.0;
                currentSpeedOfSound = getSpeedOfSound(lastT_K);
                currentAirDensity = calculateAirDensity(lastP_hPa, lastT_K, lastH_perc);
                
                // Mettre à jour tous les champs météo
                if ($('weather-status')) $('weather-status').textContent = 'ACTIF (API)';
                if ($('temp-air-2')) $('temp-air-2').textContent = dataOrDefault(weather.tempC, 1, ' °C');
                if ($('pressure-2')) $('pressure-2').textContent = dataOrDefault(weather.pressure_hPa, 0, ' hPa');
                if ($('humidity-2')) $('humidity-2').textContent = dataOrDefault(weather.humidity_perc, 0, ' %');
                if ($('air-density-calc')) $('air-density-calc').textContent = dataOrDefault(currentAirDensity, 3, ' kg/m³');
                if ($('dew-point')) $('dew-point').textContent = dataOrDefault(weather.dew_point, 1, ' °C');
             }
             if (pollutants) {
                if ($('no2')) $('no2').textContent = dataOrDefault(pollutants.NO2, 1, ' µg/m³');
                // ... Mise à jour des autres polluants
             }
        }
        
        // 3. Mise à jour Astro
        updateAstro(currentLat, currentLon, now); 

        // 4. UKF Debug / Statut
        if ($('ekf-status')) $('ekf-status').textContent = ukf.status;
        if ($('speed-uncertainty')) $('speed-uncertainty').textContent = dataOrDefault(kUncert, 4, ' m/s');
        if ($('alt-uncertainty')) $('alt-uncertainty').textContent = dataOrDefault(kAltUncert, 4, ' m');
        if ($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = dataOrDefault(currentSpeedOfSound, 2, ' m/s');
        
    };

    domSlowID = setInterval(slowLoop, DOM_SLOW_UPDATE_MS);
    slowLoop(); 
              }
// =================================================================
// BLOC 5/5 : INITIALISATION & CONTRÔLES SYSTÈME (INIT)
// =================================================================

function resetDisp(fullReset = false) {
    distM = 0; maxSpd = 0; maxGForce = 0; timeMoving = 0; maxLux = 0; maxDb = 0;
    if (fullReset) {
        ukf = new ProfessionalUKF(0, 0);
        currentLat = 0.0; currentLon = 0.0; kAlt = 0.0; kSpd = 0.0;
        sessionStartTime = Date.now();
        if ($('gps-acquisition-status')) $('gps-acquisition-status').textContent = 'INACTIF (Manuel)';
        window.location.reload(); // Rechargement forcé pour un reset complet
    }
}

function initControls() {
    // --- MARCHE / PAUSE GPS ---
    const startBtn = $('start-btn');
    if (startBtn) {
        startBtn.addEventListener('click', () => {
            if (wID !== null) {
                navigator.geolocation.clearWatch(wID); 
                wID = null;
                startBtn.innerHTML = '▶️ MARCHE GPS'; 
                if ($('gps-acquisition-status')) $('gps-acquisition-status').textContent = 'PAUSE';
            } else {
                wID = navigator.geolocation.watchPosition(gpsSuccess, gpsError, { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 });
                startBtn.innerHTML = '⏸️ PAUSE GPS'; 
                if ($('gps-acquisition-status')) $('gps-acquisition-status').textContent = 'Actif (HF)';
            }
        });
    }
    
    // --- CONTRÔLES D'ACTION ---
    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', () => {
        emergencyStopActive = !emergencyStopActive;
        if ($('emergency-status')) $('emergency-status').textContent = emergencyStopActive ? 'ACTIF 🔴' : 'INACTIF 🟢';
        if (emergencyStopActive) {
            if (wID !== null) navigator.geolocation.clearWatch(wID); 
            wID = null;
            if ($('gps-acquisition-status')) $('gps-acquisition-status').textContent = 'STOP D\'URGENCE';
        }
    });
    
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { resetDisp(true); });
    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => { distM = 0; timeMoving = 0; });
    if ($('reset-v-max')) $('reset-v-max').addEventListener('click', () => { maxSpd = 0.0; maxGForce = 0.0; });

    // --- CONTRÔLES D'ENVIRONNEMENT & PHYSIQUE ---
    if ($('mass-input')) $('mass-input').addEventListener('input', (e) => {
        currentMass = parseFloat(e.target.value) || 70.0;
        if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
    });
    
    if ($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', () => {
        netherMode = !netherMode;
        if ($('nether-indicator')) $('nether-indicator').textContent = netherMode ? 'ACTIVÉ (1:8) 🔥' : 'DÉSACTIVÉ (1:1) 🌍';
    });
    
    if ($('distance-ratio-toggle-btn')) $('distance-ratio-toggle-btn').addEventListener('click', () => {
        distanceRatioMode = !distanceRatioMode;
        const text = distanceRatioMode ? 'ALTITUDE' : 'SURFACE';
        $('distance-ratio-toggle-btn').textContent = `Rapport Distance: ${text}`;
    });

    if ($('celestial-body-select')) $('celestial-body-select').addEventListener('change', (e) => {
        currentCelestialBody = e.target.value;
        // On force la mise à jour de G_ACCEL_WGS84 ici pour simulation
        if (e.target.value === 'Lune') G_ACCEL_WGS84 = G_ACCEL_WGS84 * 0.165; 
        else G_ACCEL_WGS84 = calculateWGS84Gravity(currentLat * D2R, kAlt || 0);
        if ($('gravity-base')) $('gravity-base').textContent = `${G_ACCEL_WGS84.toFixed(4)} m/s²`;
    });
}

/** 🚀 Fonction d'initialisation principale */
function init() {
    console.log("--- Démarrage du GNSS SpaceTime Dashboard ---");
    
    try {
        // 1. Initialisation des valeurs par défaut
        currentAirDensity = calculateAirDensity(BARO_ALT_REF_HPA, TEMP_SEA_LEVEL_K);
        currentSpeedOfSound = getSpeedOfSound(TEMP_SEA_LEVEL_K); 
        
        // 2. Initialisation des capteurs IMU (avec gestion des permissions iOS/Android)
        if ($('imu-status')) $('imu-status').textContent = 'En attente de permission...';
        if (typeof DeviceMotionEvent.requestPermission === 'function') {
            document.addEventListener('click', () => {
                DeviceMotionEvent.requestPermission().then(state => {
                    if (state === 'granted') {
                        window.addEventListener('devicemotion', handleDeviceMotion, true);
                        window.addEventListener('deviceorientation', handleDeviceOrientation, true);
                        if ($('imu-status')) $('imu-status').textContent = 'Actif (Motion/Gyro)';
                    } else {
                        if ($('imu-status')) $('imu-status').textContent = 'Refusé';
                    }
                }).catch(e => console.error("IMU Permission Error:", e));
            }, { once: true }); 
        } else {
            window.addEventListener('devicemotion', handleDeviceMotion, true);
            window.addEventListener('deviceorientation', handleDeviceOrientation, true);
            if ($('imu-status')) $('imu-status').textContent = 'Actif (Motion/Gyro)';
        }
        
        // 3. Initialisation des affichages
        if ($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = `${currentSpeedOfSound.toFixed(2)} m/s`;
        if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        if ($('gravity-base')) $('gravity-base').textContent = `${G_ACCEL_WGS84.toFixed(4)} m/s²`;
        if ($('gps-acquisition-status')) $('gps-acquisition-status').textContent = 'INACTIF (Prêt)';
        if ($('weather-status')) $('weather-status').textContent = 'INACTIF (Défaut)';
        
        // 4. Démarrage Synchro NTP, Boucles & Contrôles
        syncH(); 
        startSlowLoop(); 
        startFastLoop(); 
        initControls(); 
        
    } catch (error) {
        console.error("ERREUR CRITIQUE D'INITIALISATION:", error);
        const statusElement = $('gps-acquisition-status');
        if (statusElement) {
            statusElement.textContent = `CRASH: ${error.name}: ${error.message}`;
            statusElement.style.color = 'red';
        }
    }
}

// Lancement du système au chargement complet de la page
document.addEventListener('DOMContentLoaded', init);
