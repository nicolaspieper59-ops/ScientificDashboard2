// =================================================================
// FICHIER JS PARTIE 1 : gnss-dashboard-part1.js (Constantes & Utilitaires)
// =================================================================

// --- CLÉS API ET URLS ---
const OWM_API_KEY = "VOTRE_CLE_API_OPENWEATHERMAP"; // REMPLACER PAR VOTRE CLÉ RÉELLE !
const OWM_API_URL = "https://api.openweathermap.org/data/2.5/weather";

// --- CONSTANTES MATHÉMATIQUES ET GÉODÉSIQUES ---
const R2D = 180 / Math.PI; // Radians to Degrees
const D2R = Math.PI / 180; // Degrees to Radians
const G_ACC = 9.80665;     // Accélération standard de la gravité (m/s²)
const W_EARTH = 7.2921159e-5; // Vitesse angulaire de la Terre (rad/s)
const GEOID_SEPARATION_M = 40.0; // Séparation Géoïde/Ellipsoïde typique (m)

// --- CONSTANTES PHYSIQUES ET ASTRONOMIQUES ---
const C_L = 299792458; // Vitesse de la lumière (m/s)
const SPEED_SOUND = 343; // Vitesse du son dans l'air sec à 20°C (m/s)
const AU_M = 149597870700; // Unité Astronomique (m)
const KMH_MS = 3.6; // Multiplicateur m/s -> km/h
const dayMs = 86400000; // Millisecondes dans une journée

// --- CONFIGURATION DU SYSTÈME ET DES FILTRES ---
const MAX_ACC = 50.0; // Précision GPS max acceptable (m)
const GOOD_ACC_THRESHOLD = 5.0; // Seuil pour une bonne précision GPS (m)
const MIN_DT = 0.05; // Delta temps minimum pour les calculs (s)
const MIN_SPD = 0.05; // Vitesse minimale pour être considéré en mouvement (m/s)
const VEL_NOISE_FACTOR = 10.0; // Facteur pour le bruit de vitesse GPS
const ALT_TH = -50.0; // Seuil d'altitude pour le statut souterrain (m)
const NETHER_RATIO = 8.0; // Ratio distance Minecraft Nether

// Filtre Kalman EKF/IMU
const R_GPS_DISABLED = 10000.0; // Bruit R si GPS désactivé (m²)
const STATIC_ACCEL_THRESHOLD = 0.2; // Seuil Accel. pour ZVU (m/s²)
const ACCEL_FILTER_ALPHA = 0.8; // Alpha pour le filtre passe-bas Accel (plus haut = plus de lissage)

// Options GPS (watchPosition)
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, timeout: 5000, maximumAge: 0 },
    LOW_FREQ: { enableHighAccuracy: true, timeout: 10000, maximumAge: 3000 }
};

// --- VARIABLES D'ÉTAT GLOBALES ---
let currentGPSMode = 'HIGH_FREQ'; // Mode GPS actuel
let selectedEnvironment = 'NORMAL'; // Environnement pour le facteur Kalman
let wID = null; // ID de watchPosition
let sTime = null; // Temps de démarrage de la session (ms)
let lPos = null; // Dernière position GPS (pour dt)

let lat = 0.0, lon = 0.0; // Coordonnées globales
let distM = 0.0; // Distance totale parcourue (m)
let maxSpd = 0.0; // Vitesse max (m/s)
let timeMoving = 0.0; // Temps passé en mouvement (s)
let lastFSpeed = 0.0; // Dernière vitesse filtrée (pour accélération)

// Filtre Kalman Vitesse (k: Kalman, uncert: incertitude)
let kSpd = 0.0;
let kUncert = 1000.0; 

// Filtre Kalman Altitude
let kAlt = 0.0; 
let kAltUncert = 1000.0; 

// État IMU
let kAccel = { x: 0, y: 0, z: 0 }; // Accélération filtrée
let G_STATIC_REF = { x: 0, y: 0, z: 0 }; // Référence Gravité statique
let latestLinearAccelMagnitude = 0.0; // Magnitude Accel linéaire
let latestVerticalAccelIMU = 0.0; // Accel verticale corrigée
let global_pitch = 0.0; // Angle de tangage (rad)
let global_roll = 0.0; // Angle de roulis (rad)

// État Météo
let lastP_hPa = 1013.25; // Pression atmosphérique par défaut (hPa)
let lastAirDensity = 1.225; // Densité de l'air par défaut (kg/m³)

// État de contrôle
let netherMode = false;
let isZVUActive = false; // Zero Velocity Update
let zvuLockTime = 0; // Temps de verrouillage ZVU (s)
let emergencyStopActive = false; // Arrêt d'urgence

// DOM/Intervalle
let domID = null;
let weatherID = null;
const DOM_SLOW_UPDATE_MS = 500;
const WEATHER_UPDATE_MS = 30000;

let map = null; // Objet Leaflet Map
let marker = null; // Objet Leaflet Marker
let tracePolyline = null; // Objet Leaflet Polyline


// --- UTILITAIRES (Fonctions de base) ---

/** Simplification de l'accès au DOM */
const $ = (id) => document.getElementById(id);

/** Fonction de synchronisation du temps (pourrait être améliorée avec NTP) */
function getCDate() {
    return new Date();
}

/** Calcule le nombre de jours écoulés depuis le 1er janvier 2000 */
function toDays(date) {
    const epoch = new Date(2000, 0, 1, 12, 0, 0); // J2000.0 (GMT)
    return (date.getTime() - epoch.getTime()) / dayMs;
}

/** Calcule l'anomalie solaire moyenne */
function solarMeanAnomaly(d) {
    return D2R * (357.5291 + 0.98560028 * d);
}

/** Calcule la longitude écliptique */
function eclipticLongitude(M) {
    let C = D2R * (1.9148 * Math.sin(M) + 0.02 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M));
    let P = D2R * 102.9377;
    return M + C + P + Math.PI; 
}

/** Formatage du temps pour l'horloge Minecraft (minutes TST -> HH:MM:SS) */
function formatTime(totalMinutes) {
    const minutes = Math.floor(totalMinutes % 60);
    const totalHours = Math.floor(totalMinutes / 60);
    const hours = Math.floor(totalHours % 24);
    const seconds = Math.floor((totalMinutes * 60) % 60);
    return `${String(hours).padStart(2, '0')}:${String(minutes).padStart(2, '0')}:${String(seconds).padStart(2, '0')}`;
}

/** Calcule le point de rosée (approximation Magnus) */
function calculateDewPoint(T, RH) {
    const a = 17.27;
    const b = 237.7;
    const gamma = (a * T) / (b + T) + Math.log(RH / 100);
    return (b * gamma) / (a - gamma);
}

/** Nom de la phase lunaire */
function getMoonPhaseName(phase) {
    if (phase < 0.03 || phase > 0.97) return 'Nouvelle Lune';
    if (phase < 0.22) return 'Premier Croissant';
    if (phase < 0.28) return 'Premier Quartier';
    if (phase < 0.47) return 'Lune Gibbeuse Croissante';
    if (phase < 0.53) return 'Pleine Lune';
    if (phase < 0.72) return 'Lune Gibbeuse Décroissante';
    if (phase < 0.78) return 'Dernier Quartier';
    return 'Dernier Croissant';
}

/** Synchronisation de l'heure locale au démarrage */
function syncH() {
    // Dans un vrai projet, ceci synchroniserait l'heure via un serveur NTP/API
    console.log("Synchronisation de l'heure locale effectuée.");
}


// --- FILTRE KALMAN (EKF simplifiée) ---

/** Calcul dynamique de la covariance de bruit de mesure (R) pour l'EKF */
function getKalmanR(gpsAccuracy, alt, pressure) {
    let R_gps = gpsAccuracy * gpsAccuracy; 
    let R_env_factor = 1.0; 

    // Ajustement en fonction de l'environnement
    switch (selectedEnvironment) {
        case 'FOREST': R_env_factor = 1.5; break;
        case 'METAL': R_env_factor = 2.5; break;
        case 'CONCRETE': R_env_factor = 3.0; break;
    }

    // Le bruit est proportionnel à la précision GPS et au bruit environnemental
    return R_gps * R_env_factor; 
}

/** Filtre de Kalman 1D (Vitesse) */
function kFilter(z, dt, R_dyn, u) {
    // 1. Prédiction (avec Accélération comme contrôle 'u')
    let x_pred = kSpd + u * dt; 
    let P_pred = kUncert + R_dyn; // P = P + Q (On utilise R_dyn ici comme proxy pour Q)
    
    // 2. Mise à jour
    let K = P_pred / (P_pred + R_dyn); // Gain de Kalman
    kSpd = x_pred + K * (z - x_pred); // Nouvelle vitesse estimée
    kUncert = (1 - K) * P_pred; // Nouvelle incertitude
    
    return kSpd;
}

/** Filtre de Kalman 1D (Altitude) */
function kFilterAltitude(z, gpsAccuracy, dt, u) {
    const Q_alt = 0.5; // Bruit de processus pour l'altitude (m/s)
    let R_alt = gpsAccuracy * gpsAccuracy * 2; // R dépend de la précision GPS

    // 1. Prédiction (u est l'accélération verticale IMU pour le contrôle)
    let kAlt_vel_change = u * dt * 0.5; // Simplification : moitié de l'accélération
    let x_pred = kAlt + kAlt_vel_change * dt; 
    let P_pred = kAltUncert + Q_alt; 
    
    // 2. Mise à jour
    let K = P_pred / (P_pred + R_alt); // Gain de Kalman
    kAlt = x_pred + K * (z - x_pred); 
    kAltUncert = (1 - K) * P_pred; 
    
    return kAlt;
}

// =================================================================
// FIN FICHIER JS PARTIE 1
// =================================================================
    
// FICHIER JS PARTIE 2 : gnss-dashboard-part2.js (Logique Principale & UI)
// =================================================================

// ... (Nécessite toutes les constantes et utilitaires du Bloc 1)

// ===========================================
// FONCTIONS API MÉTÉO
// ===========================================

async function updateWeather(latA, lonA) {
    if (!OWM_API_KEY || OWM_API_KEY === "VOTRE_CLE_API_OPENWEATHERMAP") {
        if ($('temp-air')) $('temp-air').textContent = 'API CLÉ MANQUANTE';
        return;
    }
    
    if ($('temp-air')) $('temp-air').textContent = 'Chargement...';

    try {
        const url = `${OWM_API_URL}?lat=${latA.toFixed(4)}&lon=${lonA.toFixed(4)}&units=metric&appid=${OWM_API_KEY}&lang=fr`;
        const response = await fetch(url);
        if (!response.ok) throw new Error(`Weather API failed: ${response.statusText}`);
        const data = await response.json();
        
        const tempC = data.main.temp;
        const pressurehPa = data.main.pressure;
        const humidity = data.main.humidity;
        const windSpeedMs = data.wind.speed; 
        const windDeg = data.wind.deg;
        
        lastP_hPa = pressurehPa; 
        const dewPointC = calculateDewPoint(tempC, humidity);

        let windChillC = tempC;
        if (tempC < 10) { 
             windChillC = 13.12 + 0.6215 * tempC - 11.37 * (windSpeedMs**0.16) + 0.3965 * tempC * (windSpeedMs**0.16);
        }

        // Calcul de la densité de l'air
        const R_SPECIFIQUE_AIR = 287.05; 
        const tempK = tempC + 273.15;
        const pressurePa = pressurehPa * 100;
        const density = pressurePa / (R_SPECIFIQUE_AIR * tempK);
        lastAirDensity = density; 
        
        const directions = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSO", "SO", "OSO", "O", "ONO", "NO", "NNO"];
        const windDirection = directions[Math.round((windDeg / 22.5) + 0.5) % 16];

        if ($('temp-air')) $('temp-air').textContent = `${tempC.toFixed(1)} °C`;
        if ($('pressure')) $('pressure').textContent = `${pressurehPa.toFixed(0)} hPa | ${(pressurehPa * 0.75006).toFixed(1)} mmHg`;
        if ($('humidity')) $('humidity').textContent = `${humidity.toFixed(0)} %`;
        if ($('dew-point')) $('dew-point').textContent = dewPointC ? `${dewPointC.toFixed(1)} °C` : 'N/A';
        if ($('wind-speed-ms')) $('wind-speed-ms').textContent = `${windSpeedMs.toFixed(2)} m/s | ${(windSpeedMs * 3.6).toFixed(1)} km/h`;
        if ($('wind-direction')) $('wind-direction').textContent = `${windDirection} (${windDeg} °)`;
        if ($('temp-feels-like')) $('temp-feels-like').textContent = `${windChillC.toFixed(1)} °C`;
        if ($('visibility')) $('visibility').textContent = data.visibility ? `${(data.visibility / 1000).toFixed(1)} km` : 'N/A (API)';
        if ($('air-density')) $('air-density').textContent = `${density.toFixed(3)} kg/m³`; 

        if ($('precipitation-rate')) $('precipitation-rate').textContent = data.rain && data.rain['1h'] ? `${data.rain['1h']} mm/h` : (data.snow && data.snow['1h'] ? `${data.snow['1h']} mm/h (neige)` : '0 mm/h');
        
    } catch (error) {
        console.error("Erreur lors de la récupération des données météo:", error);
        if ($('temp-air')) $('temp-air').textContent = 'API ERREUR';
    }
}

// ===========================================
// FONCTIONS ASTRO (TST/MST/EOT)
// ===========================================

/** Calcule le Temps Solaire Vrai (TST) et l'Équation du Temps (EOT). */
function getSolarTime(date, lon) {
    if (date === null || lon === null) return { TST: 'N/A', MST: 'N/A', EOT: 'N/D', ECL_LONG: 'N/D', TST_MS: 0, DECL: 'N/D' }; 
    const d = toDays(date);
    const M = solarMeanAnomaly(d); 
    const L = eclipticLongitude(M); 
    const epsilon = D2R * (23.4393 - 0.000000356 * d); 
    
    const declination_rad = Math.asin(Math.sin(epsilon) * Math.sin(L));
    const final_declination = declination_rad * R2D; 

    let alpha = Math.atan2(Math.cos(epsilon) * Math.sin(L), Math.cos(L));
    if (alpha < 0) alpha += 2 * Math.PI; 
    
    const meanLongitude = M + D2R * 102.9377 + Math.PI;
    let eot_rad_raw = alpha - meanLongitude; 
    eot_rad_raw = eot_rad_raw % (2 * Math.PI);
    if (eot_rad_raw > Math.PI) { eot_rad_raw -= 2 * Math.PI; } else if (eot_rad_raw < -Math.PI) { eot_rad_raw += 2 * Math.PI; }
    
    const eot_min = eot_rad_raw * R2D * 4; 
    
    let ecl_long_deg = (L * R2D) % 360; 
    const final_ecl_long = ecl_long_deg < 0 ? ecl_long_deg + 360 : ecl_long_deg;
    
    const msSinceMidnightUTC = (date.getUTCHours() * 3600 + date.getUTCMinutes() * 60 + date.getUTCSeconds()) * 1000 + date.getUTCMilliseconds();
    const mst_offset_ms = lon * dayMs / 360; 
    const mst_ms_raw = msSinceMidnightUTC + mst_offset_ms;
    const mst_ms = (mst_ms_raw % dayMs + dayMs) % dayMs; 
    const eot_ms = eot_min * 60000;
    const tst_ms_raw = mst_ms + eot_ms;
    const tst_ms = (tst_ms_raw % dayMs + dayMs) % dayMs; 
    
    const toTimeString = (ms) => {
        let h = Math.floor(ms / 3600000);
        let m = Math.floor((ms % 3600000) / 60000);
        let s = Math.floor((ms % 60000) / 1000);
        return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
    };
    return { 
        TST: toTimeString(tst_ms), 
        TST_MS: tst_ms, 
        MST: toTimeString(mst_ms), 
        EOT: eot_min.toFixed(3), 
        ECL_LONG: final_ecl_long.toFixed(2), 
        DECL: final_declination.toFixed(2) 
    };
}

/** Met à jour les données Astro. (Lever/Coucher de soleil ajoutés) */
function updateAstro(latA, lonA) {
    const cDate = getCDate(); 
    if (!cDate || isNaN(latA) || isNaN(lonA) || !window.SunCalc) { return; }

    const { TST, TST_MS, MST, EOT, ECL_LONG, DECL } = getSolarTime(cDate, lonA); 

    const pos = SunCalc.getPosition(cDate, latA, lonA);
    const times = SunCalc.getTimes(cDate, latA, lonA);
    const moonIllum = SunCalc.getMoonIllumination(cDate);
    
    const sunElevationDeg = pos.altitude * R2D;
    const zenithAngleDeg = 90 - sunElevationDeg;

    const solarNoonLocal = times.solarNoon;
    const solarNoonUTCMs = solarNoonLocal.getTime() + (solarNoonLocal.getTimezoneOffset() * 60000);
    const solarNoonUTC = new Date(solarNoonUTCMs); 
    
    let dayDurationH = "N/A";
    if (times.sunrise && times.sunset) {
        dayDurationH = (times.sunset.getTime() - times.sunrise.getTime()) / (1000 * 3600);
    }
    
    // Formatage Lever/Coucher de soleil en UTC
    const formatUTCDate = (dateObj) => dateObj.toUTCString().match(/(\d{2}:\d{2}:\d{2})/)[0];
    const sunriseUTC = times.sunrise ? formatUTCDate(times.sunrise) : "N/A";
    const sunsetUTC = times.sunset ? formatUTCDate(times.sunset) : "N/A";

    // --- MISE À JOUR DU DOM (Astro) ---
    $('sun-elevation').textContent = `${sunElevationDeg.toFixed(2)} °`;
    $('zenith-angle').textContent = `${zenithAngleDeg.toFixed(2)} °`;
    $('tst').textContent = TST; 
    $('lsm').textContent = MST; 
    
    $('noon-solar').textContent = solarNoonUTC.toTimeString().split(' ')[0] + ' UTC'; 
    $('noon-solar-mst').textContent = '12:00:00 MST';
    
    $('day-duration').textContent = typeof dayDurationH === 'number' ? `${dayDurationH.toFixed(2)} h` : dayDurationH; 
    
    $('ecliptic-long').textContent = `${ECL_LONG} °`; 
    $('eot').textContent = `${EOT} min`;
    $('moon-phase-display').textContent = getMoonPhaseName(moonIllum.phase);
    $('sun-declination').textContent = `${DECL} °`; 
    
    // NOUVEAU : Affichage Lever/Coucher
    $('sunrise-time').textContent = `${sunriseUTC} UTC`;
    $('sunset-time').textContent = `${sunsetUTC} UTC`;
    
    updateMinecraftClock(TST_MS / 60000, sunElevationDeg, parseFloat(DECL)); 
}

/** Met à jour l'affichage de l'horloge stylisée du Soleil avec logique saisonnière. */
function updateMinecraftClock(TST_minutes, elevation, sunDeclination) {
    const $clock = $('minecraft-clock');
    if (!$clock) return;
    
    const $status = $('clock-status');
    
    if ($('time-minecraft')) {
        $('time-minecraft').textContent = formatTime(TST_minutes); 
    }
    
    $status.textContent = `TST (Élévation: ${elevation.toFixed(2)}°)`;
    
    const TST_hour = TST_minutes / 60;
    const TST_normalized = TST_hour % 24; 
    const TST_angle_deg_raw = (TST_normalized - 12) * 15;
    const TST_angle_deg = (TST_angle_deg_raw + 360) % 360; 

    // --- LOGIQUE SAISONNIÈRE ---
    const MAX_DECL = 23.44; 
    const normalizedDecl = Math.abs(sunDeclination) / MAX_DECL; 
    const sunSizeFactor = 1.0 + (normalizedDecl * 0.2); 
    const moonVerticalShift = -(sunDeclination / MAX_DECL) * 5; 

    // --- MISE À JOUR DES ÉLÉMENTS CÉLESTES ---
    let sunEl = $clock.querySelector('.sun-element');
    if (!sunEl) { sunEl = document.createElement('div'); sunEl.className = 'sun-element'; $clock.appendChild(sunEl); }
    sunEl.style.fontSize = `${1.0 * sunSizeFactor}em`; 
    sunEl.style.transform = `rotate(${TST_angle_deg}deg) translateY(-45px)`; 
    sunEl.textContent = '☀️'; 

    let moonEl = $clock.querySelector('.moon-element');
    if (!moonEl) { moonEl = document.createElement('div'); moonEl.className = 'moon-element'; $clock.appendChild(moonEl); }
    const moon_TST_angle_deg = (TST_angle_deg + 180) % 360;
    moonEl.style.transform = `rotate(${moon_TST_angle_deg}deg) translateY(${-45 + moonVerticalShift}px)`; 
    
    const moonIllum = SunCalc.getMoonIllumination(getCDate());
    const phase = moonIllum.phase;
    moonEl.textContent = phase < 0.03 || phase > 0.97 ? '🌑' : 
                         phase < 0.22 ? '🌒' :
                         phase < 0.28 ? '🌓' :
                         phase < 0.47 ? '🌔' :
                         phase < 0.53 ? '🌕' :
                         phase < 0.72 ? '🌖' :
                         phase < 0.78 ? '🌗' : '🌘';

    // Logique de couleur du ciel dynamique
    let sky_color = '#3f51b5';
    let opacity = 1; 
    if (elevation > 10) { sky_color = '#87ceeb'; opacity = 0; }
    else if (elevation > -6) { sky_color = '#ff9800'; opacity = 0.5; }
    else if (elevation > -18) { sky_color = '#00008b'; opacity = 0.8; }

    const style = document.createElement('style');
    style.innerHTML = `
        #minecraft-clock::before { 
            background-color: ${sky_color} !important; 
            opacity: ${opacity};
        }
    `;
    let oldStyle = $clock.querySelector('style');
    if (oldStyle) { $clock.removeChild(oldStyle); }
    $clock.appendChild(style);
}

// ===========================================
// FONCTIONS CAPTEURS INERTIELS (IMU) : CORRIGÉE
// ===========================================

function handleDeviceMotion(event) {
    if (emergencyStopActive) return;
    const acc = event.accelerationIncludingGravity;
    if (acc.x === null) return; 

    // --- 1. Filtrage Passe-Bas (lissage renforcé) ---
    kAccel.x = ACCEL_FILTER_ALPHA * kAccel.x + (1 - ACCEL_FILTER_ALPHA) * acc.x;
    kAccel.y = ACCEL_FILTER_ALPHA * kAccel.y + (1 - ACCEL_FILTER_ALPHA) * acc.y;
    kAccel.z = ACCEL_FILTER_ALPHA * kAccel.z + (1 - ACCEL_FILTER_ALPHA) * acc.z; 

    // --- 2. Correction de l'Inclinaison (Gravité projetée) ---
    const G_CORR_X = -G_ACC * Math.sin(global_pitch); 
    const G_CORR_Y = G_ACC * Math.sin(global_roll) * Math.cos(global_pitch); 
    const G_CORR_Z = G_ACC * Math.cos(global_roll) * Math.cos(global_pitch); 

    // --- 3. Extraction de l'Accélération Linéaire (corrigée) ---
    let accel_vertical_lin = 0.0;
    let linear_x = 0.0, linear_y = 0.0, linear_z = 0.0;

    linear_x = kAccel.x - G_CORR_X;
    linear_y = kAccel.y - G_CORR_Y;
    linear_z = kAccel.z - G_CORR_Z; 
    accel_vertical_lin = linear_z; 

    const magnitude_lin = Math.sqrt(linear_x ** 2 + linear_y ** 2 + linear_z ** 2);

    if (magnitude_lin < ACCEL_MOVEMENT_THRESHOLD) { 
        G_STATIC_REF.x = G_CORR_X;
        G_STATIC_REF.y = G_CORR_Y;
        G_STATIC_REF.z = G_CORR_Z;
        accel_vertical_lin = 0.0; 
        latestLinearAccelMagnitude = 0.0;
    } else {
        latestLinearAccelMagnitude = magnitude_lin;
    }

    latestVerticalAccelIMU = accel_vertical_lin; 
    
    if ($('accel-vertical-imu')) $('accel-vertical-imu').textContent = `${accel_vertical_lin.toFixed(3)} m/s²`;
    if ($('force-g-vertical')) $('force-g-vertical').textContent = `${(accel_vertical_lin / G_ACC).toFixed(2)} G`;
}

/** Met à jour les variables globales d'orientation (roulis et tangage) */
function handleDeviceOrientation(event) {
    if (emergencyStopActive) return;
    global_pitch = event.beta * D2R; 
    global_roll = event.gamma * D2R; 
}


// ===========================================
// FONCTION PRINCIPALE DE MISE À JOUR (GPS CALLBACK)
// ===========================================

function updateDisp(pos) {
    if (emergencyStopActive) return;
    lat = pos.coords.latitude; lon = pos.coords.longitude;
    const alt = pos.coords.altitude, acc = pos.coords.accuracy;
    const spd_raw_gps = pos.coords.speed;
    const cTimePos = pos.timestamp; 
    const now = getCDate(); 
    const MASS = 70.0; 

    if (now === null) { updateAstro(lat, lon); return; } 
    
    // CORRECTION : Réinitialise timeMoving avec sTime
    if (sTime === null) { 
        sTime = now.getTime(); 
        timeMoving = 0.0; 
    }

    // CORRECTION (NaN) : Gestion de l'imprécision GPS et initialisation de l'altitude
    if (acc > MAX_ACC) { 
        if ($('gps-precision')) $('gps-precision').textContent = `❌ ${acc.toFixed(0)} m (Trop Imprécis)`; 
        if (lPos === null) {
            lPos = pos; 
            if (alt !== null) {
                kAlt = alt; 
                lPos.kAlt_old = kAlt;
            }
        }
        return; 
    }

    let effectiveAcc = acc;
    const accOverride = parseFloat($('gps-accuracy-override').value);
    if (accOverride > 0) { effectiveAcc = accOverride; }

    let spdH = spd_raw_gps ?? 0; 
    const dt = lPos ? (cTimePos - lPos.timestamp) / 1000 : MIN_DT;

    if (lPos && lPos.kAlt_old === undefined) {
        lPos.kAlt_old = kAlt; 
    }

    // Facteur d'amortissement IMU
    let imuDampeningFactor = 1.0; 
    if (effectiveAcc <= GOOD_ACC_THRESHOLD) { imuDampeningFactor = 0.5; } 
    else if (effectiveAcc >= MAX_ACC) { imuDampeningFactor = 1.0; } 
    else {
        const range = MAX_ACC - GOOD_ACC_THRESHOLD;
        const value = effectiveAcc - GOOD_ACC_THRESHOLD;
        imuDampeningFactor = 0.5 + 0.5 * (value / range); 
    }

    let accel_control_3D = latestLinearAccelMagnitude * imuDampeningFactor;
    let accel_control_V = latestVerticalAccelIMU * imuDampeningFactor;

    // Vitesse Verticale Brute
    let spdV_raw = 0; 
    if (lPos && lPos.kAlt_old !== undefined && dt > MIN_DT && alt !== null) { 
        spdV_raw = (kAlt - lPos.kAlt_old) / dt; 
    } 
    let spd3D_raw = Math.sqrt(spdH ** 2 + spdV_raw ** 2);

    // LOGIQUE ZÉRO-VITESSE DYNAMIQUE (ZVU)
    const GPS_NOISE_SPEED = effectiveAcc / VEL_NOISE_FACTOR; 
    const isStaticByIMU = latestLinearAccelMagnitude < STATIC_ACCEL_THRESHOLD; 
    const isStaticByDynamicSpeed = spd3D_raw < GPS_NOISE_SPEED;
    
    const isStatic = isStaticByIMU && isStaticByDynamicSpeed; 
    let isIMUOnlyMode = false; 
    let spd3D = spd3D_raw; 

    if (isStatic) {
        spd3D = 0.0; 
        isIMUOnlyMode = true; 
        isZVUActive = true;
        zvuLockTime += dt;
        accel_control_3D = 0.0; accel_control_V = 0.0; 
        kSpd = 0.0; kUncert = 0.000001; lastFSpeed = 0.0; 
    } else {
        isZVUActive = false;
        zvuLockTime = 0;
    }
    
    const HIGH_NOISE_ACC_THRESHOLD = 10.0; 
    const isInterior = isStatic && (effectiveAcc > HIGH_NOISE_ACC_THRESHOLD || selectedEnvironment === 'CONCRETE' || selectedEnvironment === 'METAL');

    // Filtrage Kalman de la Vitesse et de l'Altitude
    let R_dyn = getKalmanR(effectiveAcc, alt, lastP_hPa);
    if (isIMUOnlyMode) { R_dyn = R_GPS_DISABLED; }

    const sSpdFE = kFilter(spd3D, dt, R_dyn, accel_control_3D);
    const kAlt_new = kFilterAltitude(alt, effectiveAcc, dt, accel_control_V);
    
    // NOUVEAU : Calcul de l'Altitude Géoïde (MSL)
    const kAlt_Geoid = kAlt_new - GEOID_SEPARATION_M;

    // Correction Distance : utilise spd3D_raw (vitesse brute)
    const speedForMetrics = spd3D_raw < MIN_SPD ? 0 : spd3D_raw; 

    const spd_kms = sSpdFE / 1000;
    const spd_mach = sSpdFE / SPEED_SOUND; 

    let accel_long = (dt > 0.05) ? (sSpdFE - lastFSpeed) / dt : 0;
    lastFSpeed = sSpdFE;

    distM += speedForMetrics * dt * (netherMode ? NETHER_RATIO : 1); 
    
    if (speedForMetrics > MIN_SPD) { timeMoving += dt; }
    if (speedForMetrics > maxSpd) maxSpd = speedForMetrics; 
    const avgSpdMoving = timeMoving > 0 ? (distM / timeMoving) : 0;
    
    const dist_s_light = distM / C_L;
    const dist_ua = distM / AU_M;
    
    const coriolusForce = 2 * MASS * sSpdFE * W_EARTH * Math.sin(lat * D2R); 
    const kineticEnergy = 0.5 * MASS * sSpdFE ** 2;
    const mechanicalPower = accel_long * MASS * sSpdFE; 
    
    const C_DRAG = 0.4; 
    const A_REF = 1.0; 
    const dragForce = 0.5 * lastAirDensity * sSpdFE * sSpdFE * C_DRAG * A_REF;

    // --- MISE À JOUR DU DOM (GPS/Physique) ---
    $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(5)} km/h`; 
    $('speed-stable-ms').textContent = `${sSpdFE.toFixed(3)} m/s`; 
    $('speed-stable-kms').textContent = `${spd_kms.toFixed(5)} km/s`; 
    $('speed-mach').textContent = `${spd_mach.toFixed(5)} Mach`; 
    $('speed-3d-inst').textContent = `${(spd3D_raw * KMH_MS).toFixed(5)} km/h`; 
    $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    $('speed-avg-moving').textContent = `${(avgSpdMoving * KMH_MS).toFixed(5)} km/h`;
    $('perc-speed-c').textContent = `${(sSpdFE / C_L * 100).toExponential(2)}%`; 
    $('perc-speed-sound').textContent = `${(sSpdFE / SPEED_SOUND * 100).toFixed(2)} %`; 
    
    $('distance-total-km').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    $('distance-ua').textContent = `${dist_ua.toExponential(3)} UA`; 
    $('distance-s-light').textContent = `${dist_s_light.toExponential(3)} s lumière`; 
    $('distance-j-light').textContent = `${(dist_s_light / (24 * 3600)).toExponential(3)} j lumière`; 
    $('distance-cosmic-al').textContent = `${(dist_s_light / (3600*24*365.25)).toExponential(3)} al`; 
    
    $('latitude').textContent = lat.toFixed(6);
    $('longitude').textContent = lon.toFixed(6);
    $('altitude-gps').textContent = `${kAlt_new.toFixed(2)} m (WGS84)`; // Précise WGS84
    $('altitude-geoid').textContent = `${kAlt_Geoid.toFixed(2)} m (MSL)`; // NOUVELLE ALTITUDE
    $('gps-precision').textContent = `${acc.toFixed(2)} m`;
    $('gps-accuracy-effective').textContent = `${effectiveAcc.toFixed(2)} m`;
    $('speed-raw-ms').textContent = spd_raw_gps !== null ? `${spd_raw_gps.toFixed(2)} m/s` : 'N/A';
    
    $('vertical-speed').textContent = `${spdV_raw.toFixed(2)} m/s`;
    $('accel-long').textContent = `${accel_long.toFixed(3)} m/s²`;
    $('force-g-long').textContent = `${(accel_long / G_ACC).toFixed(2)} G`;
    $('speed-error-perc').textContent = `${R_dyn.toFixed(3)} m² (R dyn)`; 
    
    $('kinetic-energy').textContent = `${kineticEnergy.toFixed(2)} J`;
    $('mechanical-power').textContent = `${mechanicalPower.toFixed(2)} W`;
    $('coriolis-force').textContent = `${coriolusForce.toExponential(2)} N`;
    $('drag-force').textContent = `${dragForce.toFixed(3)} N`; 
    
        
    // --- MISE À JOUR DU DOM (Physique - suite) ---
    // ... (Votre code précédent de mise à jour du DOM se termine ici) ...
    // Note : La nouvelle variable $('altitude-geoid') est utilisée ici pour la première fois.

    // MISE À JOUR DU STATUT DÉTAILLÉ 
    const isSubterranean = (kAlt_new !== null && kAlt_new < ALT_TH); 
    let statusText;
    
    if (isSubterranean) {
        statusText = `🟢 ACTIF (SOUTERRAIN/IMU) | GPS Acc: ${effectiveAcc.toFixed(0)} m`;
    } else {
        if (isInterior) {
            statusText = `🏡 INTÉRIEUR (GPS Élevé) | GPS Acc: ${effectiveAcc.toFixed(0)} m`;
        } else {
            // Complète la ligne de statut GPS/EKF
            statusText = `🔴 GPS+EKF | GPS Acc: ${effectiveAcc.toFixed(0)} m`; 
        }
        
        // Ajout de l'information ZVU (Zéro Vitesse) si statique
        if (isStatic) {
             const GPS_NOISE_SPEED_DISPLAY = effectiveAcc / VEL_NOISE_FACTOR; 
             statusText += ` | 🔒 ZVU ON (Seuil Dyn: ${(GPS_NOISE_SPEED_DISPLAY * KMH_MS).toFixed(2)} km/h)`;
             
             // Si le mode est IMU-Only (ZVU activé), on le reflète dans le statut
             if (isIMUOnlyMode) {
                 statusText = statusText.replace('GPS+EKF', 'IMU-Only/EKF');
             }
        }
    }
    
    // --- MISE À JOUR DU DOM (Statut) ---
    $('underground-status').textContent = isSubterranean ? `Oui (${statusText})` : `Non (${statusText})`;
    $('zvu-lock-status').textContent = isZVUActive ? `VERROUILLÉ` : `NON-VERROUILLÉ`;
    $('zvu-lock-time').textContent = `${zvuLockTime.toFixed(1)} s`;
    
    // Mise à jour de la carte et de l'astronomie
    updateMap(lat, lon);
    updateAstro(lat, lon);
    
    // SAUVEGARDE DES VALEURS POUR LA PROCHAINE ITÉRATION
    lPos = pos; 
    lPos.timestamp = cTimePos; 
    lPos.kAlt_old = kAlt_new; 
    lPos.kAltUncert_old = kAltUncert; 
}


// ===========================================
// FONCTIONS CARTE ET CONTRÔLE GPS
// ===========================================

// ... (Le reste du fichier, y compris les fonctions initMap, updateMap, 
// setGPSMode, startGPS, stopGPS, emergencyStop, resumeSystem, 
// handleErr, et le bloc document.addEventListener('DOMContentLoaded'), doit suivre ici, 
// tel que dans la dernière version fournie.)
// ... (Suite de la fonction updateDisp) ... 


// ===========================================
// FONCTIONS CARTE ET CONTRÔLE GPS
// ===========================================

/** Initialise la carte Leaflet. */
function initMap(latA, lonA) {
    if (map) return;
    try {
        map = L.map('map-container').setView([latA, lonA], 15);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            maxZoom: 19,
            attribution: '© OpenStreetMap'
        }).addTo(map);

        marker = L.marker([latA, lonA]).addTo(map);
        tracePolyline = L.polyline([], { color: 'red' }).addTo(map);
        console.log("Carte Leaflet initialisée.");
    } catch (e) {
        console.error("Échec de l'initialisation de la carte Leaflet:", e);
        if ($('map-container')) $('map-container').textContent = 'Erreur d\'initialisation de la carte.';
    }
}

/** Met à jour la carte. */
function updateMap(latA, lonA) {
    if (!map || !marker || !tracePolyline) { initMap(latA, lonA); return; }
    
    const newLatLng = [latA, lonA];
    marker.setLatLng(newLatLng);
    tracePolyline.addLatLng(newLatLng);
    map.setView(newLatLng);
}

function setGPSMode(mode) {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    currentGPSMode = mode;
    wID = navigator.geolocation.watchPosition(updateDisp, handleErr, GPS_OPTS[mode]); 
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = `⏸️ PAUSE GPS`;
    if ($('freq-select')) $('freq-select').value = mode; 
}

function startGPS() {
    if (wID === null) {
        if ($('freq-select')) $('freq-select').value = currentGPSMode; 
        setGPSMode(currentGPSMode);
        sTime = null; // Réinitialise le temps de session (et timeMoving sera réinitialisé dans updateDisp)
    }
}

function stopGPS(resetButton = true) {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
    }
    if (resetButton) {
        if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = `▶️ MARCHE GPS`;
    }
}

function emergencyStop() {
    emergencyStopActive = true;
    stopGPS(false);
    if ($('emergency-stop-btn')) {
        $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: ACTIF 🔴";
    }
}

function resumeSystem() {
    emergencyStopActive = false;
    if ($('emergency-stop-btn')) {
        $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: INACTIF 🟢";
    }
    startGPS();
}

function handleErr(err) {
    console.error(`Erreur GNSS (${err.code}): ${err.message}`);
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = `❌ ERREUR GPS`;
}


// ===========================================
// INITIALISATION DES ÉVÉNEMENTS ET INTERVALLES
// ===========================================

document.addEventListener('DOMContentLoaded', () => {
    
    // --- Initialisation des contrôles et de l'état ---
    if ($('environment-select')) {
        $('environment-select').value = selectedEnvironment;
        $('environment-select').addEventListener('change', (e) => { 
            if (emergencyStopActive) return;
            selectedEnvironment = e.target.value; 
        });
    }

    // --- Écouteurs pour les boutons et contrôles ---
    if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', () => {
        if (emergencyStopActive) return;
        wID === null ? startGPS() : stopGPS();
    });
    
    if ($('freq-select')) $('freq-select').addEventListener('change', (e) => {
        if (emergencyStopActive) return;
        setGPSMode(e.target.value);
    });
    
    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', () => {
        emergencyStopActive ? resumeSystem() : emergencyStop(); 
    });
    
    if ($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        netherMode = !netherMode; 
        if ($('mode-nether')) $('mode-nether').textContent = netherMode ? "ACTIVÉ (1:8) 🔥" : "DÉSACTIVÉ (1:1)"; 
    });

    // --- Écouteurs pour la réinitialisation des métriques ---
    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        distM = 0; timeMoving = 0; 
        if ($('distance-total-km')) $('distance-total-km').textContent = "0.000 km | 0.00 m";
    });
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        maxSpd = 0; 
        if ($('speed-max')) $('speed-max').textContent = "0.00000 km/h";
    });
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        if (confirm("Êtes-vous sûr de vouloir tout réinitialiser? (Distance, Max, Kalman)")) { 
            distM = 0; maxSpd = 0; kSpd = 0; kUncert = 1000; kAlt = 0; kAltUncert = 1000; timeMoving = 0; lastFSpeed = 0;
            if (tracePolyline) tracePolyline.setLatLngs([]); 
        } 
    });
    
    if ($('data-capture-btn')) $('data-capture-btn').addEventListener('click', () => {
        alert("Données actuelles capturées (logique de sauvegarde à implémenter)!");
    });
    
    if ($('toggle-mode-btn')) $('toggle-mode-btn').addEventListener('click', () => {
        document.body.classList.toggle('dark-mode');
        const isDarkMode = document.body.classList.contains('dark-mode');
        $('toggle-mode-btn').textContent = isDarkMode ? "☀️ Mode Jour" : "🌗 Mode Nuit";
    });

    // --- Démarrage des Capteurs IMU (Accélération) ---
    if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', handleDeviceMotion, true);
    } else {
        console.warn("DeviceMotion n'est pas supporté ou activé sur cet appareil/navigateur.");
    } 
    
    // --- Démarrage des Capteurs d'Orientation (pour correction verticale) ---
    if (window.DeviceOrientationEvent) {
        window.addEventListener('deviceorientation', handleDeviceOrientation, true);
    } else {
        console.warn("DeviceOrientation n'est pas supporté. La correction de l'accélération verticale ne sera pas appliquée.");
    }

    // --- Initialisation du Système ---
    syncH(); 
    startGPS(); 

    // --- Intervalle lent pour les mises à jour (Astro/Temps) ---
    if (domID === null) {
        domID = setInterval(() => {
            const now = getCDate();
            if (now) {
                if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString();
                if ($('date-display')) $('date-display').textContent = now.toLocaleDateString();
                if ($('time-elapsed')) $('time-elapsed').textContent = sTime ? ((now.getTime() - sTime) / 1000).toFixed(2) + ' s' : '0.00 s';
                if ($('time-moving')) $('time-moving').textContent = timeMoving.toFixed(2) + ' s';
            }
            
            if (lat !== 0 && lon !== 0) updateAstro(lat, lon); 
        }, DOM_SLOW_UPDATE_MS); 
    }
    
    // --- Intervalle pour la mise à jour Météo (30s) ---
    if (weatherID === null) {
        weatherID = setInterval(() => {
            if (lat !== 0 && lon !== 0) updateWeather(lat, lon);
        }, WEATHER_UPDATE_MS); 
    }
}); // Fin de document.addEventListener('DOMContentLoaded')
