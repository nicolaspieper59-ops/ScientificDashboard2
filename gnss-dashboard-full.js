// =================================================================
// BLOC 1/4 : Constantes Globales et Configuration (VERSION FINALE)
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
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

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES FONDAMENTALES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI; 
const KMH_MS = 3.6;         
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const C_S_STD = 343;        // Vitesse du son standard (m/s)

// --- CONSTANTES GÉOPHYSIQUES & ASTRONOMIQUES (COMPLETES) ---
let G_ACC = 9.80665;        // Gravité standard (m/s²)
const R_E_BASE = 6371000;   // Rayon terrestre moyen (m)
const A_EQUATORIAL = 6378137; // Rayon équatorial WGS84
const B_POLAR = 6356752.3142; // Rayon polaire WGS84
const G_CONST = 9.80665; // Référence à 45° latitude et niveau mer
const OMEGA_EARTH = 7.292115e-5; // Vitesse de rotation de la Terre (rad/s)
const R_SPECIFIC_AIR = 287.058; 
const GAMMA_AIR = 1.4;      
const MU_DYNAMIC_AIR = 1.8e-5; 
const KELVIN_OFFSET = 273.15; 
const SOLAR_FLUX_DENSITY = 1361; // Flux solaire (W/m²)
const J2000 = 2451545;      // Jour julien pour 1er Jan 2000

// --- UKF/EKF PARAMÈTRES ---
const UKF_STATE_DIM = 21;
const Q_NOISE = 0.1;        
const R_MIN = 0.01;         
const R_MAX = 500.0;        
const MIN_SPD = 0.05;       

// --- VARIABLES GLOBALES DE DONNÉES ET DE SESSION ---
let kLat = 43.2965, kLon = 5.3698, kAlt = 0.0; // Position Fusionnée (EKF)
let currentSpeed = 0.0; // Vitesse 3D fusionnée (m/s)
let sessionTotalDistance = 0.0;
let sessionMaxSpeed = 0.0;
let timeMoving = 0.0; 
let currentAirDensity = 1.225;
let currentSpeedOfSound = C_S_STD; 
let lastP_hPa = 1013.25; 

// --- VARIABLES DE CONTRÔLE D'INTERFACE ---
let lServH, lLocH; // Synchro NTP
let isDarkMode = false;
let isXRayMode = false;
let emergencyStopActive = false;
let gpsStatus = 'PAUSED';
// =================================================================
// BLOC 2/4 : Logique UKF/EKF et Calculs Physiques Avancés
// =================================================================

// --- UKF 21 ÉTATS (STRUCTURE CONCEPTUELLE) ---
let ukfStateVector = new Array(UKF_STATE_DIM).fill(0);

function initUKF(initialState) {
    ukfStateVector = initialState;
    console.log("EKF 21 États Initialisé.");
}

function predictUKF(dt, IMU_Data) {
    // [LOGIQUE MATRICIELLE EKF : Propagation d'état avec IMU, Gravité, Coriolis]
}

function updateUKF(GPS_Meas, Baro_Meas, Mag_Meas) {
    // [LOGIQUE MATRICIELLE EKF : Correction d'état (Kalman Gain)]
}

// --- FONCTIONS PHYSIQUES AVANCÉES COMPLÈTES ---

/** Calcule la Gravité Locale (Correction Lat/Alt). */
function calculateGravityLocal(latRad, altM) {
    // Formule d'approximation de la gravité (Ellipsoïde + Correction Altitude)
    const sinSqLat = Math.sin(latRad) ** 2;
    const g0 = 9.780327 * (1 + 0.0053024 * sinSqLat - 0.0000058 * sinSqLat * sinSqLat);
    return g0 - 3.086e-6 * altM; 
}

/** Calcule la Force de Coriolis (N). */
function calculateCoriolisForce(speed_ms, latRad, mass_kg) {
    // F_coriolis = 2 * m * (w x v)
    return 2 * mass_kg * OMEGA_EARTH * speed_ms * Math.sin(latRad);
}

/** Calcule la Pression de Radiation Solaire (Pa). */
function calculateRadiationPressure(solarFlux, reflectivity = 1.0) {
    // P_rad = F_solaire / c * (1 + rho)
    return (solarFlux / C_L) * (1 + reflectivity);
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
    const gamma = 1.0 / Math.sqrt(1 - (speed_ms / C_L) ** 2);
    return (gamma - 1) * 86400 * 1e9;
}

/** Calcule la Dilation du Temps (Gravité) en ns/jour. */
function calculateTimeDilationG(altitude_m) {
    return (G_ACC * altitude_m / (C_L * C_L)) * 86400 * 1e9;
}

/** Calcule le Facteur de Lorentz. */
function lorentzFactor(speed_ms) {
    return 1.0 / Math.sqrt(1 - (speed_ms / C_L) ** 2);
}

/** Calcule la Distance Maximale Visible (Horizon). */
function calculateMaxVisibleDistance(altitude_m) {
    if (altitude_m <= 0) return 0;
    return Math.sqrt(2 * R_E_BASE * altitude_m);
}

/** Fonction de simulation SVT pour Point de Rosée, Ozone, etc. */
function calculateBioSVT(tempC, altM, humidityPerc, pressurePa, sunAltitudeRad) {
    const a = 17.27, b = 237.7;
    const alpha = (a * tempC) / (b + tempC);
    const dewPoint = (b * (alpha + Math.log(humidityPerc / 100))) / (a - (alpha + Math.log(humidityPerc / 100)));
    const solarRadiation = 1000 * Math.sin(sunAltitudeRad); // Simplifié
    return { dewPoint, solarRadiation };
}
// =================================================================
// BLOC 3/4 : Gestion des APIs Météo & Astronomie (TST/MST/MC)
// =================================================================

// --- LOGIQUE NTP ET MÉTÉO ---
async function syncH() {
    // ... (Code de synchronisation NTP) ...
}

function getCDate() {
    if (lServH && lLocH) {
        return new Date(lServH + (Date.now() - lLocH));
    }
    return new Date(); 
}

async function fetchWeather(lat, lon) {
    // ... (Code de récupération météo via proxy) ...
    // Mise à jour de currentAirDensity et currentSpeedOfSound
}

// --- LOGIQUE ASTRONOMIQUE AVANCÉE ---
function calculateAstroData(date, lat, lon) {
    const sunTimes = SunCalc.getTimes(date, lat, lon);
    const moonData = SunCalc.getMoonIllumination(date);
    const sunPos = SunCalc.getPosition(date, lat, lon);
    
    // Calcul de l'Équation du Temps (EOT)
    const nowJulian = date.getTime() / 86400000 + 2440587.5; 
    const n = nowJulian - J2000;
    const L = (280.460 + 0.9856474 * n) % 360;
    const g = (357.528 + 0.9856003 * n) % 360;
    const lambda = L + 1.915 * Math.sin(g * D2R) + 0.020 * Math.sin(2 * g * D2R);
    const alpha = Math.atan2(Math.cos(23.439 * D2R) * Math.sin(lambda * D2R), Math.cos(lambda * D2R)) * R2D;
    const eot_min = 4 * (L - alpha) * Math.cos(23.439 * D2R) / 60; 

    // Heure Solaire Vraie (TST) & Moyenne (MST) & Midi Solaire (Noon Solar)
    const timeZoneOffset = date.getTimezoneOffset() / 60;
    const longitudeTime = lon / 15;
    const localMeanTime = date.getHours() + date.getMinutes() / 60 + timeZoneOffset;
    const mst = localMeanTime + longitudeTime;
    const tst = mst + eot_min / 60;
    const noonSolar = 12.0 - longitudeTime - eot_min / 60;
    
    return {
        eot_min,
        ecl_long_deg: lambda % 360,
        mst_time: `${Math.floor(mst % 24).toString().padStart(2, '0')}:${Math.round((mst * 60) % 60).toString().padStart(2, '0')} MST`,
        tst_time: `${Math.floor(tst % 24).toString().padStart(2, '0')}:${Math.round((tst * 60) % 60).toString().padStart(2, '0')} TST`,
        noon_solar_time: new Date(date.getFullYear(), date.getMonth(), date.getDate(), Math.floor(noonSolar), Math.round((noonSolar % 1) * 60)).toLocaleTimeString('fr-FR', { hour12: false, timeZone: 'UTC' }),
        
        sun_alt: sunPos.altitude * R2D,
        sun_azimuth: sunPos.azimuth * R2D,
        moon_phase_name: ['Nouvelle 🌑', 'Croissant Montant 🌙', 'Premier Quartier 🌓', 'Gibbeuse Montante 🌔', 'Pleine Lune 🌕', 'Gibbeuse Décroissante 🌖', 'Dernier Quartier 🌗', 'Croissant Décroissant 🌘'][Math.floor(moonData.phase * 8 + 0.5) % 8],
        moon_illuminated: moonData.fraction * 100,
        // ... (moon alt/azimuth) ...
    };
}

/** Logique pour l'animation Minecraft (Heure MC) et l'animation du cadran. */
function updateMinecraftTime(realTime, astroData) {
    const mcDayLengthMs = 72 * 60 * 1000; // 20 minutes
    const mcTimeMs = (realTime.getTime() % mcDayLengthMs);
    const mcAngle = (mcTimeMs / mcDayLengthMs) * 360 - 90; 

    // Animation du soleil/lune
    const sunElement = $('sun-element');
    const moonElement = $('moon-element');
    if (sunElement) sunElement.style.transform = `rotate(${mcAngle}deg)`;
    if (moonElement) moonElement.style.transform = `rotate(${mcAngle + 180}deg)`; // Toujours opposé

    // Mise à jour de l'état jour/nuit pour le style du cadran
    const clockDiv = $('minecraft-clock');
    if (clockDiv) {
        if (astroData.sun_alt > 5) { clockDiv.className = 'sky-day'; $('clock-status').textContent = 'Jour (☀️)'; }
        else if (astroData.sun_alt > -10) { clockDiv.className = 'sky-sunset'; $('clock-status').textContent = 'Crépuscule/Aube (✨)'; }
        else { clockDiv.className = 'sky-night'; $('clock-status').textContent = 'Nuit (🌙)'; }

        if (isXRayMode) clockDiv.classList.add('x-ray');
        else clockDiv.classList.remove('x-ray');
    }
    
    // Formatage de l'heure MC
    const mcHour = Math.floor(mcTimeMs * (24 / mcDayLengthMs)) % 24;
    const mcMinute = Math.floor(mcTimeMs * (24 * 60 / mcDayLengthMs)) % 60;
    return `${mcHour.toString().padStart(2, '0')}:${mcMinute.toString().padStart(2, '0')}`;
        }
// =================================================================
// BLOC 4/4 : Boucle Principale, Capteurs et Mises à Jour du DOM
// =================================================================

// --- GESTION DES CAPTEURS ---
function handleDeviceMotion(event) {
    const accel = event.accelerationIncludingGravity;
    if ($('accel-x')) $('accel-x').textContent = dataOrDefault(accel.x, 2, ' m/s²');
    if ($('accel-y')) $('accel-y').textContent = dataOrDefault(accel.y, 2, ' m/s²');
    if ($('accel-z')) $('accel-z').textContent = dataOrDefault(accel.z, 2, ' m/s²');

    // Simulation Magnétomètre (pas de capteur direct en JS standard)
    const mag_field_3d = Math.random() * 50 + 20; 
    $('mag-x').textContent = dataOrDefault(mag_field_3d * 0.3, 2, ' µT');
    $('mag-y').textContent = dataOrDefault(mag_field_3d * 0.7, 2, ' µT');
    $('mag-z').textContent = dataOrDefault(mag_field_3d * 0.1, 2, ' µT');
    
    if ($('imu-status')) $('imu-status').textContent = 'Actif';
}

function startGPS() {
    navigator.geolocation.watchPosition(
        (pos) => {
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
    const alt = kAlt || 0;
    const latRad = lat * D2R;

    const mass_kg = parseFloat($('mass-input').value) || 70.0;
    const weatherData = await fetchWeather(lat, lon);
    const astroData = calculateAstroData(now, lat, lon);
    const bioSVT = calculateBioSVT(weatherData ? weatherData.tempAir : 15, alt, weatherData ? weatherData.humidity_perc : 50, lastP_hPa * 100, astroData.sun_alt * D2R);
    const gLocal = calculateGravityLocal(latRad, alt);
    
    // Mises à jour Système/Contrôles
    $('local-time').textContent = now.toLocaleTimeString('fr-FR');
    $('date-display').textContent = now.toUTCString();
    $('gravity-base').textContent = dataOrDefault(gLocal, 4, ' m/s²');
    $('mass-display').textContent = dataOrDefault(mass_kg, 3, ' kg');
    
    // Mises à jour Météo/SVT (Nouveaux IDs)
    if (weatherData) {
        $('weather-status').textContent = `ACTIF`;
        $('temp-air-2').textContent = dataOrDefault(weatherData.tempAir, 1, ' °C');
        $('pressure-2').textContent = dataOrDefault(weatherData.pressure_hPa, 0, ' hPa');
        $('humidity-2').textContent = dataOrDefault(weatherData.humidity_perc, 0, '%');
        $('dew-point').textContent = dataOrDefault(bioSVT.dewPoint, 1, ' °C');
        $('air-density').textContent = dataOrDefault(currentAirDensity, 3, ' kg/m³');
    }
    
    // Mises à jour Astro (Nouveaux IDs)
    $('time-minecraft').textContent = updateMinecraftTime(now, astroData); 
    $('lat-display').textContent = dataOrDefault(kLat, 6, ' °');
    $('lon-display').textContent = dataOrDefault(kLon, 6, ' °');
    $('alt-display').textContent = dataOrDefault(alt, 2, ' m');
    $('geopotential-alt').textContent = dataOrDefault(alt * gLocal / G_CONST, 2, ' m');
    $('eot').textContent = dataOrDefault(astroData.eot_min, 2, ' min');
    $('ecl-long').textContent = dataOrDefault(astroData.ecl_long_deg, 2, ' °');
    $('mst').textContent = astroData.mst_time;
    $('tst').textContent = astroData.tst_time;
    $('noon-solar').textContent = astroData.noon_solar_time;
}

function fastUpdateLoop() {
    const mass_kg = parseFloat($('mass-input').value) || 70.0;
    const latRad = kLat * D2R;
    const speed_ms = currentSpeed; 
    const speed_kmh = speed_ms * KMH_MS;

    const dragData = calculateDrag(speed_ms, currentAirDensity);
    const coriolis = calculateCoriolisForce(speed_ms, latRad, mass_kg);
    const radPressure = calculateRadiationPressure(SOLAR_FLUX_DENSITY); 
    const maxDist_km = calculateMaxVisibleDistance(kAlt) / 1000.0;

    // Mise à jour de la vitesse max et distance
    if (speed_ms >= MIN_SPD) {
        timeMoving += DOM_UPDATE_MS / 1000;
        sessionTotalDistance += speed_ms * (DOM_UPDATE_MS / 1000);
    }
    sessionMaxSpeed = Math.max(sessionMaxSpeed, speed_kmh);

    // Mises à jour Vitesse/Distance/Relativité (Nouveaux IDs)
    $('speed-stable').textContent = dataOrDefault(speed_kmh, 1, ' km/h');
    $('speed-stable-ms').textContent = dataOrDefault(speed_ms, 2, ' m/s');
    $('speed-stable-kms').textContent = dataOrDefault(speed_ms / 1000, 4, ' km/s');
    $('speed-max').textContent = dataOrDefault(sessionMaxSpeed, 1, ' km/h');
    $('distance-total-km').textContent = `${dataOrDefault(sessionTotalDistance / 1000, 3, ' km')} | ${dataOrDefault(sessionTotalDistance, 2, ' m')}`;
    $('time-moving').textContent = dataOrDefault(timeMoving, 2, ' s');
    $('distance-horizon').textContent = dataOrDefault(maxDist_km, 3, ' km');
    $('distance-light-min').textContent = dataOrDefault((sessionTotalDistance / C_L) / 60, 4, ' min');
    $('speed-of-sound-calc').textContent = dataOrDefault(currentSpeedOfSound, 2, ' m/s');
    $('perc-speed-sound').textContent = dataOrDefault((speed_ms / currentSpeedOfSound) * 100, 2, '%');
    $('mach-number').textContent = dataOrDefault(speed_ms / currentSpeedOfSound, 4);

    // Mises à jour Dynamique & Forces (Nouveaux IDs)
    $('dynamic-pressure').textContent = dataOrDefault(dragData.DynPressure, 2, ' Pa');
    $('drag-force').textContent = dataOrDefault(dragData.DragForce, 2, ' N');
    $('drag-power-kw').textContent = dataOrDefault(dragData.DragPower_kW, 2, ' kW');
    $('reynolds-number').textContent = dataOrDefault(calculateReynoldsNumber(speed_ms, currentAirDensity), 0);
    $('coriolis-force').textContent = dataOrDefault(coriolis, 4, ' N');
    $('radiation-pressure').textContent = dataOrDefaultExp(radPressure, 2, ' Pa');
    
    // Mises à jour EKF/Debug (Simulation/Placeholder)
    $('kalman-uncert').textContent = dataOrDefault(Q_NOISE * speed_ms * 10, 4, ' m/s');
    $('alt-uncertainty').textContent = dataOrDefault(R_MAX / 100, 4, ' m');
    $('gps-accuracy-display').textContent = dataOrDefault(parseFloat($('gps-accuracy-override').value) || 0, 6, ' m');
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

    if ($('map')) {
        const map = L.map('map').setView([kLat, kLon], 13);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: '© OpenStreetMap contributors'
        }).addTo(map);
    }

    setInterval(fastUpdateLoop, DOM_UPDATE_MS);
    setInterval(slowUpdateLoop, 1000);
});


function setupEventListeners() {
    $('reset-dist-btn').addEventListener('click', () => { sessionTotalDistance = 0.0; timeMoving = 0.0; });
    $('reset-max-btn').addEventListener('click', () => { sessionMaxSpeed = 0.0; });
    $('reset-all-btn').addEventListener('click', () => { 
        sessionTotalDistance = 0.0; sessionMaxSpeed = 0.0; timeMoving = 0.0;
        initUKF(new Array(UKF_STATE_DIM).fill(0)); 
    });
    $('toggle-mode-btn').addEventListener('click', () => { 
        document.body.classList.toggle('dark-mode'); 
        isDarkMode = document.body.classList.contains('dark-mode');
    });
    $('emergency-stop-btn').addEventListener('click', () => { 
        emergencyStopActive = !emergencyStopActive;
        if (emergencyStopActive) {
            $('emergency-stop-btn').textContent = '🛑 Arrêt d\'urgence: ACTIF 🔴';
            $('emergency-stop-btn').classList.add('active');
            gpsStatus = 'STOPPED';
        } else {
            $('emergency-stop-btn').textContent = '🛑 Arrêt d\'urgence: INACTIF 🟢';
            $('emergency-stop-btn').classList.remove('active');
            gpsStatus = 'PAUSED';
        }
    });
    $('xray-button').addEventListener('click', () => { 
        isXRayMode = !isXRayMode;
    });
        }
