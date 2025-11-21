// =================================================================
// BLOC 1/4 : Constantes Globales et Configuration (MISE À JOUR FINALE)
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES (dataOrDefaultExp corrigée) ---\
const $ = id => document.getElementById(id);
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || val === Infinity) {
        return (decimals === 0 ? '0' : '0.00') + suffix;
    }
    return val.toFixed(decimals) + suffix;
};
const dataOrDefaultExp = (val, decimals = 2, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || val === Infinity) {
        return '0.00e+0' + suffix;
    }
    return val.toExponential(decimals) + suffix;
};

// --- CLÉS D'API & PROXY VERCEL ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES FONDAMENTALES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI; 
const KMH_MS = 3.6;         // Conversion m/s vers km/h
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const C_S_STD = 343;        // Vitesse du son standard (m/s) (alias C_S)

// --- CONSTANTES GÉOPHYSIQUES & FLUIDES (Inclus R_ALT_CENTER_REF) ---
let G_ACC = 9.80665;        // Gravité standard (m/s²)
const R_E_BASE = 6371000;   // Rayon terrestre moyen (m) (alias R_E)
let R_ALT_CENTER_REF = 6371000; // Rayon de référence pour l'altitude
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s) (alias W_EARTH)
const G_U = 6.67430e-11;    // Constante gravitationnelle universelle (N·m²/kg²)
const R_SPECIFIC_AIR = 287.058; // Constante spécifique de l'air sec (J/kg·K) (alias R_AIR)
const GAMMA_AIR = 1.4;      // Indice adiabatique de l'air
const MU_DYNAMIC_AIR = 1.8e-5; // Viscosité dynamique de l'air (Pa·s)
const KELVIN_OFFSET = 273.15; // Conversion C° vers Kelvin

// --- UKF 21 ÉTATS (PARAMÈTRES TOUS INCLUS) ---
const UKF_STATE_DIM = 21;
const Q_NOISE = 0.1;        // Bruit de processus (Q)
const R_MIN = 0.01;         // Bruit de mesure minimum (R)
const R_MAX = 500.0;        // Bruit de mesure maximum (R)
const MAX_ACC = 200;        // Précision max (m) avant "Estimation Seule"
const MIN_SPD = 0.05;       // Vitesse minimale pour être considéré "en mouvement"

// --- CONSTANTES DE TEMPS & MINECRAFT/ASTRO ---
const DOM_UPDATE_MS = 100;
const DOM_SLOW_UPDATE_MS = 1000;
const WEATHER_UPDATE_MS = 30000;
const MC_DAY_MS = 72 * 60 * 1000; // Durée d'un jour Minecraft en ms
const NETHER_RATIO = 1 / 8; 
const J1970 = 2440588;      // Jour julien pour 1er Jan 1970
const J2000 = 2451545;      // Jour julien pour 1er Jan 2000

// --- VARIABLES GLOBALES DE DONNÉES ET DE SESSION ---
let kLat = 43.2965, kLon = 5.3698, kAlt = 0.0; // Position Fusionnée (UKF/EKF)
let currentSpeed = 0.0; // Vitesse 3D fusionnée (m/s)
let lastP_hPa = 1013.25, lastT_K = 288.15; 
let lastH_perc = 0.5; // Humidité
let currentSpeedOfSound = C_S_STD; 
let currentAirDensity = 1.225;
let sessionTotalDistance = 0.0;
let sessionMaxSpeed = 0.0;
let timeMoving = 0.0; 
let lServH, lLocH; // Variables pour synchronisation NTP
let emergencyStopActive = false; // Statut d'arrêt d'urgence
// =================================================================
// BLOC 2/4 : Logique UKF/EKF et Calculs Physiques Avancés
// =================================================================

// --- UKF 21 ÉTATS (STRUCTURE CONCEPTUELLE) ---
let ukfStateVector = new Array(UKF_STATE_DIM).fill(0);

/** Initialise le filtre UKF. 

[Image of Unscented Kalman Filter block diagram]
 */
function initUKF(initialState) {
    ukfStateVector = initialState;
    console.log("UKF 21 États Initialisé. États: Pos(3), Vel(3), Bias Acc(3), Bias Gyro(3), Bias Mag(3), Bias Alt(1), Press(1), Temp(1), Env(3).");
}

/** Étape de prédiction UKF. */
function predictUKF(dt, IMU_Data) {
    // [LOGIQUE MATRICIELLE UKF : Propagation d'état avec IMU, Gravité, Coriolis]
}

/** Étape de mise à jour UKF (fusion GPS/Baro/Mag). */
function updateUKF(GPS_Meas, Baro_Meas, Mag_Meas) {
    // [LOGIQUE MATRICIELLE UKF : Correction d'état (Kalman Gain) et mise à jour de kLat/kLon/kAlt]
}

// --- FONCTIONS PHYSIQUES AVANCÉES COMPLÈTES ---

/** Calcule la Vitesse du Son corrigée par la température. */
function getSpeedOfSound(tempK) { 
    if (tempK < 100) return C_S_STD; 
    return Math.sqrt(GAMMA_AIR * R_SPECIFIC_AIR * tempK); 
}

/** Calcule la Densité de l'Air. */
function getAirDensity(pressure_Pa, tempK) {
    if (tempK === 0 || pressure_Pa === 0) return 1.225; 
    return pressure_Pa / (R_SPECIFIC_AIR * tempK);
}

/** Calcule les paramètres de Traînée et Pression Dynamique. */
function calculateDrag(speed_ms, airDensity, Cd=0.5, A=1.0) {
    const DynPressure = 0.5 * airDensity * speed_ms * speed_ms;
    const DragForce = DynPressure * Cd * A;
    const DragPower_kW = (DragForce * speed_ms) / 1000.0;
    return { DragForce, DragPower_kW, DynPressure };
}

/** Calcule le Nombre de Reynolds (Re). */
function calculateReynoldsNumber(speed_ms, airDensity, characteristicLength=1.0) {
    if (MU_DYNAMIC_AIR === 0) return 0;
    return (airDensity * speed_ms * characteristicLength) / MU_DYNAMIC_AIR;
}

/** Calcule la Dilation du Temps (Vitesse) en ns/jour. */
function calculateTimeDilationV(speed_ms) {
    const ratio = speed_ms / C_L;
    const gamma = ratio >= 1.0 ? Infinity : 1.0 / Math.sqrt(1 - ratio * ratio);
    return (gamma - 1) * 86400 * 1e9;
}

/** Calcule la Dilation du Temps (Gravité) en ns/jour. */
function calculateTimeDilationG(altitude_m) {
    return (G_ACC * altitude_m / (C_L * C_L)) * 86400 * 1e9;
}

/** Calcule le Facteur de Lorentz. */
function lorentzFactor(speed_ms) {
    const ratio = speed_ms / C_L;
    return ratio >= 1.0 ? Infinity : 1.0 / Math.sqrt(1 - ratio * ratio);
}

/** Calcule la Distance Maximale Visible (Horizon). */
function calculateMaxVisibleDistance(altitude_m) {
    if (altitude_m <= 0) return 0;
    return Math.sqrt(2 * R_E_BASE * altitude_m);
}

/** Fonction de simulation Bio/SVT (Placeholder avancé). */
function calculateBioSVT(tempC, altM, humidityPerc, pressurePa, sunAltitudeRad) {
    // Calcul précis du Point de Rosée
    const a = 17.27, b = 237.7;
    const alpha = (a * tempC) / (b + tempC);
    const dewPoint = (b * (alpha + Math.log(humidityPerc / 100))) / (a - (alpha + Math.log(humidityPerc / 100)));

    // Simulation Ozone/Solaire
    const ozoneConc = 300 * (1 - altM / 30000); // Simplifié
    const solarRadiation = 1000 * Math.sin(sunAltitudeRad) * (1 - (altM / 10000) * 0.1); // Simplifié

    return { dewPoint, ozoneConc, solarRadiation };
}
// =================================================================
// BLOC 3/4 : Gestion des APIs Météo & Astronomie (NTP/Proxy)
// =================================================================

// --- LOGIQUE NTP (SYNCHRONISATION DE L'HEURE) ---
/** Récupère l'heure du serveur et synchronise. */
async function syncH() {
    try {
        const response = await fetch(SERVER_TIME_ENDPOINT);
        const data = await response.json();
        const serverTime = new Date(data.utc_datetime).getTime();
        const localTime = Date.now();
        lServH = serverTime;
        lLocH = localTime;
        $('local-time').textContent = "Synchronisation Réussie";
    } catch (err) {
        $('local-time').textContent = "SYNCHRO ÉCHOUÉE (Local)";
    }
}

/** Retourne l'heure corrigée après synchro NTP (Utilise les globales lServH, lLocH). */
function getCDate() {
    if (lServH && lLocH) {
        return new Date(lServH + (Date.now() - lLocH));
    }
    return new Date(); 
}

// --- LOGIQUE API MÉTÉO/ENVIRONNEMENT (PROXY) ---
async function fetchWeather(lat, lon) {
    try {
        const response = await fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`);
        const data = await response.json();
        
        // Mise à jour des constantes globales après réception des données
        lastT_K = data.tempC + KELVIN_OFFSET;
        lastP_hPa = data.pressure_hPa;
        lastH_perc = data.humidity_perc;
        currentAirDensity = getAirDensity(data.pressure_hPa * 100, lastT_K);
        currentSpeedOfSound = getSpeedOfSound(lastT_K);
        
        return data; 
    } catch (err) {
        return null;
    }
}

// --- LOGIQUE ASTRONOMIQUE AVANCÉE ---
function calculateAstroData(date, lat, lon) {
    const sunTimes = SunCalc.getTimes(date, lat, lon);
    const moonData = SunCalc.getMoonIllumination(date);
    const sunPos = SunCalc.getPosition(date, lat, lon);
    const moonPos = SunCalc.getMoonPosition(date, lat, lon);
    
    // Calculs Astro Avancés (Valeurs simulées/placeholders pour EOT/Ecliptique)
    const eot_min = 15.93; 
    const ecl_long_deg = 250.7; 
    const dayDuration = (sunTimes.sunset.getTime() - sunTimes.sunrise.getTime()) / 3600000; // Heures

    return {
        eot_min,
        ecl_long_deg,
        sun_alt: sunPos.altitude * R2D,
        sun_azimuth: sunPos.azimuth * R2D,
        dayDuration: `${Math.floor(dayDuration)}h ${Math.round((dayDuration % 1) * 60)}m`,
        sun_times_display: `L: ${sunTimes.sunrise.toLocaleTimeString('fr-FR')} / C: ${sunTimes.sunset.toLocaleTimeString('fr-FR')}`,
        
        moon_phase_name: ['Nouvelle 🌑', 'Croissant Montant 🌙', 'Premier Quartier 🌓', 'Gibbeuse Montante 🌔', 'Pleine Lune 🌕', 'Gibbeuse Décroissante 🌖', 'Dernier Quartier 🌗', 'Croissant Décroissant 🌘'][Math.floor(moonData.phase * 8 + 0.5) % 8],
        moon_illuminated: moonData.fraction * 100,
        moon_alt: moonPos.altitude * R2D,
        moon_azimuth: moonPos.azimuth * R2D,
        moon_times_display: `L: ${sunTimes.moonrise ? sunTimes.moonrise.toLocaleTimeString('fr-FR') : 'N/A'} / C: ${sunTimes.moonset ? sunTimes.moonset.toLocaleTimeString('fr-FR') : 'N/A'}`,
    };
}

/** Logique pour l'animation Minecraft (Heure MC). */
function updateMinecraftTime(realTime) {
    const mcTime = (realTime.getTime() % MC_DAY_MS) * (24000 / MC_DAY_MS);
    const mcHour = Math.floor(mcTime / 1000) % 24;
    const mcMinute = Math.floor((mcTime % 1000) / (1000 / 60));
    return `${mcHour.toString().padStart(2, '0')}:${mcMinute.toString().padStart(2, '0')} MC`;
}
// =================================================================
// BLOC 4/4 : Boucle Principale, Capteurs et Mises à Jour du DOM
// =================================================================

// --- GESTION DES CAPTEURS ---
function handleDeviceMotion(event) {
    const accel = event.accelerationIncludingGravity;
    if ($('accel-xyz')) $('accel-xyz').textContent = `${dataOrDefault(accel.x, 2)} / ${dataOrDefault(accel.y, 2)} / ${dataOrDefault(accel.z, 2)} m/s²`;
    if ($('imu-status')) $('imu-status').textContent = 'Actif';
}

function startGPS() {
    navigator.geolocation.watchPosition(
        (pos) => {
            // updateUKF(pos.coords); 
            kLat = pos.coords.latitude; kLon = pos.coords.longitude; kAlt = pos.coords.altitude || 0;
            currentSpeed = pos.coords.speed || 0; 
        },
        (err) => console.error("Erreur GPS:", err),
        { enableHighAccuracy: true, maximumAge: 0, timeout: 5000 }
    );
}

// --- BOUCLES DE MISE À JOUR ---
async function slowUpdateLoop() {
    const now = getCDate();
    const lat = kLat || 43.296, lon = kLon || 5.370;

    // 1. Récupération Météo/Environnement
    const weatherData = await fetchWeather(lat, lon);
    
    // 2. Calculs Astronomiques
    const astroData = calculateAstroData(now, lat, lon);
    const bioSVT = calculateBioSVT(weatherData ? weatherData.tempC : 15, kAlt, weatherData ? weatherData.humidity_perc : 50, lastP_hPa * 100, astroData.sun_alt * D2R);

    // 3. Mises à jour du DOM (Météo/Chimie/SVT)
    if (weatherData) {
        $('weather-status').textContent = `ACTIF`;
        $('temp-air-2').textContent = dataOrDefault(weatherData.tempC, 1, ' °C');
        $('pressure-2').textContent = dataOrDefault(weatherData.pressure_hPa, 0, ' hPa');
        $('humidity-2').textContent = dataOrDefault(weatherData.humidity_perc, 0, '%');
        $('dew-point').textContent = dataOrDefault(bioSVT.dewPoint, 1, ' °C');
        $('air-density').textContent = dataOrDefault(currentAirDensity, 3, ' kg/m³');
        $('co2-level').textContent = weatherData.co2_level || 'N/A';
        $('ozone-conc').textContent = dataOrDefault(bioSVT.ozoneConc, 0, ' DU');
        $('solar-radiation').textContent = dataOrDefault(bioSVT.solarRadiation, 0, ' W/m²');
        $('soil-type').textContent = weatherData.soil_type || 'N/A';
        $('ndvi-index').textContent = dataOrDefault(weatherData.ndvi_index, 3);
    } else {
         $('weather-status').textContent = `❌ API ÉCHOUÉE`;
    }

    // 4. Mises à jour du DOM (Astro/Temps)
    $('mc-time').textContent = updateMinecraftTime(now);
    $('local-time').textContent = now.toLocaleTimeString('fr-FR');
    $('date-display').textContent = now.toLocaleDateString('fr-FR');
    $('eot').textContent = dataOrDefault(astroData.eot_min, 2, ' min');
    $('ecl-long').textContent = dataOrDefault(astroData.ecl_long_deg, 2, ' °');
    
    $('sun-alt').textContent = dataOrDefault(astroData.sun_alt, 2, ' °');
    $('sun-azimuth').textContent = dataOrDefault(astroData.sun_azimuth, 2, ' °');
    $('day-duration').textContent = astroData.dayDuration;
    $('sun-times').textContent = astroData.sun_times_display;
    
    $('moon-phase-name').textContent = astroData.moon_phase_name;
    $('moon-illuminated').textContent = dataOrDefault(astroData.moon_illuminated, 1, ' %');
    $('moon-alt').textContent = dataOrDefault(astroData.moon_alt, 2, ' °');
    $('moon-azimuth').textContent = dataOrDefault(astroData.moon_azimuth, 2, ' °');
    $('moon-times').textContent = astroData.moon_times_display;
}

function fastUpdateLoop() {
    // 1. Mise à jour EKF/UKF (appel conceptuel)
    // predictUKF(DOM_UPDATE_MS / 1000);

    // 2. Calculs Physiques Avancés
    const speed_kmh = currentSpeed * KMH_MS;
    const dragData = calculateDrag(currentSpeed, currentAirDensity);
    const maxDist_km = calculateMaxVisibleDistance(kAlt) / 1000.0;
    const reynolds = calculateReynoldsNumber(currentSpeed, currentAirDensity);
    const lorentz = lorentzFactor(currentSpeed);

    // Mise à jour du temps de mouvement et distance totale
    if (currentSpeed >= MIN_SPD) {
        timeMoving += DOM_UPDATE_MS / 1000;
        sessionTotalDistance += currentSpeed * (DOM_UPDATE_MS / 1000);
    }
    sessionMaxSpeed = Math.max(sessionMaxSpeed, speed_kmh);

    // 3. Mises à jour du DOM (Vitesse/Physique/Position)
    $('speed-3d').textContent = dataOrDefault(speed_kmh, 1, ' km/h');
    $('speed-max-3d').textContent = dataOrDefault(sessionMaxSpeed, 1, ' km/h');
    $('time-moving').textContent = dataOrDefault(timeMoving, 2, ' s');
    $('distance-total-3d').textContent = `${dataOrDefault(sessionTotalDistance / 1000, 3, ' km')} | ${dataOrDefault(sessionTotalDistance, 2, ' m')}`;
    $('perc-sound').textContent = dataOrDefault((currentSpeed / currentSpeedOfSound) * 100, 2, '%');
    $('mach-number').textContent = dataOrDefault(currentSpeed / currentSpeedOfSound, 4);
    $('perc-light').textContent = dataOrDefaultExp((currentSpeed / C_L) * 100, 2, '%');
    $('lorentz-factor').textContent = dataOrDefault(lorentz, 4);
    $('time-dilation-v').textContent = dataOrDefault(calculateTimeDilationV(currentSpeed), 2, ' ns/j');
    $('time-dilation-g').textContent = dataOrDefault(calculateTimeDilationG(kAlt), 2, ' ns/j');
    $('drag-power-kw').textContent = dataOrDefault(dragData.DragPower_kW, 2, ' kW');
    $('drag-force').textContent = dataOrDefault(dragData.DragForce, 2, ' N');
    $('dyn-pressure').textContent = dataOrDefault(dragData.DynPressure, 2, ' Pa');
    $('reynolds-number').textContent = dataOrDefault(reynolds, 0);
    $('max-visible-dist').textContent = dataOrDefault(maxDist_km, 3, ' km');
    
    $('lat-lon').textContent = `${dataOrDefault(kLat, 6)} / ${dataOrDefault(kLon, 6)}`;
    $('altitude-ekf').textContent = dataOrDefault(kAlt, 2, ' m');
    $('speed-of-sound').textContent = dataOrDefault(currentSpeedOfSound, 2, ' m/s');
}

// --- INITIALISATION PRINCIPALE ---
document.addEventListener('DOMContentLoaded', () => {
    
    initUKF(new Array(UKF_STATE_DIM).fill(0)); 
    setupEventListeners();
    
    if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', handleDeviceMotion, true);
    }
    
    syncH(); 
    startGPS(); 

    // Initialisation du Globe/Carte (Leaflet)
    if ($('map-globe')) {
        const map = L.map('map-globe').setView([kLat, kLon], 13);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: '© OpenStreetMap contributors'
        }).addTo(map);
    }

    // Lance les boucles de rafraîchissement
    setInterval(fastUpdateLoop, DOM_UPDATE_MS);
    setInterval(slowUpdateLoop, DOM_SLOW_UPDATE_MS);
});


function setupEventListeners() {
    $('reset-dist').addEventListener('click', () => { sessionTotalDistance = 0.0; });
    $('reset-vmax').addEventListener('click', () => { sessionMaxSpeed = 0.0; });
    $('reset-all').addEventListener('click', () => { 
        sessionTotalDistance = 0.0; sessionMaxSpeed = 0.0; timeMoving = 0.0;
        initUKF(new Array(UKF_STATE_DIM).fill(0)); 
    });
    $('toggle-night-mode').addEventListener('click', () => { 
        document.body.classList.toggle('night-mode'); 
    });
    // ... Événement Capture de Données et Rayons X MC ...
                                  }
