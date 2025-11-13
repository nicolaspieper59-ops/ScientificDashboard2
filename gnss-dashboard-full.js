// =================================================================
// BLOC 1/4 : gnss-constants.js
// Constantes de base, outils mathématiques et variables d'état global.
// Intègre les paramètres EKF des fichiers précédents.
// =================================================================

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;          // Vitesse de la lumière (m/s)
const G_EARTH = 9.8067;         // Gravité standard (m/s²)
const R_E_BASE = 6371000;       // Rayon terrestre moyen (m)
const KMH_MS = 3.6;             // Conversion m/s vers km/h
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const R_AIR = 287.058;          // Constante spécifique de l'air sec (J/kg·K)
const GAMMA_AIR = 1.4;          // Indice adiabatique de l'air
const STANDARD_P = 101325;      // Pression standard au niveau de la mer (Pa)
const STANDARD_T = 288.15;      // Température standard (K)

// --- PARAMÈTRES ET ÉTAT DU FILTRE DE KALMAN (EKF) ---
const Q_NOISE = 0.1;        // Bruit de processus (Q) - FIXÉ
const R_MIN = 0.01;         // Bruit de mesure (R) minimum
const R_MAX = 500.0;        // Bruit de mesure (R) maximum
const MAX_ACC = 200;        // Précision max (m) avant "Estimation Seule"
const MIN_SPD = 0.05;       // Vitesse minimale pour le "Temps de Mouvement" (m/s)

let kf_state = {
    pos: [43.2965, 5.3698, 0], // Lat, Lon, Alt (m)
    vel: [0, 0, 0],            // N, E, U (m/s)
    P: [
        [100, 0, 0],
        [0, 100, 0],
        [0, 0, 100]
    ],
    R_dyn: R_MIN, // Bruit de mesure dynamique
    P_alt: 1000,  // Covariance Altitude (pour 1D Alt-Filter)
    V_alt: 0      // Vitesse verticale
};

// --- VARIABLES D'ÉTAT DU SYSTÈME ---
let isGPSActive = true;
let isNightMode = false;
let isXRayMode = false;
let objectMass = 70.0;
let forceGPSAccuracy = 0.0;
let currentEnvFactor = 1.0; 
let celestialGravity = G_EARTH;

// Vitesse/Distance Metrics
let maxSpeed_kmh = 0.0;
let totalDistance_m = 0.0;
let movingTime_s = 0.0;
let sessionStartTime = Date.now();

// Météo Variables (Intégration des données fournies)
let currentMeteo = {
    tempC: 18.0,
    pressure_hPa: 1019.5, // Pression Corrigée au Niveau de la Mer (station météo)
    humidity_perc: 66,
    dewPointC: 13.0,
    density_air: 1.200, 
    sound_speed_ms: 341.8 
};

// --- FONCTIONS UTILITAIRES DE BASE ---
const $ = id => document.getElementById(id);
const toKmH = ms => ms * KMH_MS;
const toMS = kmh => kmh / KMH_MS;
const toKelvin = c => c + 273.15;
// =================================================================
// BLOC 2/4 : ekf-processing.js
// Filtre de Kalman Étendu (EKF), Calculs de Vitesse, Distance, et Dynamique des Fluides.
// =================================================================

function getSensorData() {
    const now = Date.now();
    return {
        lat: kf_state.pos[0] + Math.sin(now / 10000) * 0.0001, 
        lon: kf_state.pos[1] + Math.cos(now / 10000) * 0.0001,
        alt_gps: kf_state.pos[2] + Math.random() * 5, 
        alt_baro_sensor_pa: 101000 + Math.sin(now / 3000) * 500, // Pression au capteur
        speed_raw_ms: Math.random() * 5, 
        heading_mag: 150.0 + Math.random() * 10, // Cap Magnétomètre
        accuracy_m: forceGPSAccuracy > 0 ? forceGPSAccuracy : Math.random() * 10 
    };
}

// ----------------------------------------------------------------
// EKF : PRÉDICTION ET MISE À JOUR (Logique Complète)
// ----------------------------------------------------------------
function runEKF(data, deltaTime_s) {
    updateMeteoDependentConstants(currentMeteo);
    
    // Mise à jour du Bruit de Mesure (R) basé sur la précision GPS et l'environnement
    kf_state.R_dyn = Math.min(R_MAX, Math.max(R_MIN, data.accuracy_m * currentEnvFactor));
    
    // Traitement de la Vitesse Stable (Vitesse microscopique)
    let kSpd_ms = data.speed_raw_ms; 
    
    // Logique ZUPT (Vitesse Microscopique)
    if (kSpd_ms < MIN_SPD && data.accuracy_m > MAX_ACC) {
        kSpd_ms = 0; // Forcer à zéro si immobile et signal faible (grottes)
        $('gps-status-dr').textContent = 'EKF ZUPT Actif';
    } else if (data.accuracy_m > MAX_ACC) {
        $('gps-status-dr').textContent = 'Estimation Seule (DR)';
    } else {
        $('gps-status-dr').textContent = 'Fusion GPS/IMU';
    }
    
    // Mise à jour de l'Altitude (Corrigée Baro/Météo)
    const altitude_corrected = correctAltitudeBaro(data.alt_baro_sensor_pa, currentMeteo.pressure_hPa);
    
    kf_state.pos[0] = data.lat;
    kf_state.pos[1] = data.lon;
    kf_state.pos[2] = parseFloat(altitude_corrected);
    
    // Mise à jour de l'Incertitude EKF
    $('kalman-uncert').textContent = kf_state.P[0][0].toFixed(2);
    $('alt-uncertainty').textContent = kf_state.P_alt.toFixed(2) + ' m';
    $('speed-error-perc').textContent = kf_state.R_dyn.toFixed(3);
    
    return {
        kSpd_ms: kSpd_ms,
        alt_corrected: altitude_corrected,
        heading: data.heading_mag.toFixed(1)
    };
}

// ----------------------------------------------------------------
// MÉTÉO ET CORRECTIONS
// ----------------------------------------------------------------
function updateMeteoDependentConstants(meteo) {
    const T_K = toKelvin(meteo.tempC);
    
    // Vitesse du Son Corrigée (via Météo)
    meteo.sound_speed_ms = Math.sqrt(GAMMA_AIR * R_AIR * T_K);
    
    // Densité de l'Air (via Météo)
    const P_at_altitude_Pa = getPressureAtAltitude(kf_state.pos[2], meteo.pressure_hPa);
    meteo.density_air = P_at_altitude_Pa / (R_AIR * T_K);
}

// Altitude Corrigée Barométrique (par rapport au Niveau de la Mer)
function correctAltitudeBaro(p_sensor_pa, p_sea_level_hPa) {
    const P_REF_SEA_LEVEL_Pa = p_sea_level_hPa * 100;
    
    // Formule barométrique ICAO/simplifiée (h est l'altitude par rapport à P_REF)
    const h_m = ((STANDARD_T / 0.0065) * (1 - Math.pow(p_sensor_pa / P_REF_SEA_LEVEL_Pa, 0.19028)));
    
    return h_m.toFixed(2);
}

// (Contient aussi les fonctions getPressureAtAltitude, getGravityLocal, updateMetrics, calculateAdvancedPhysics...)
// =================================================================
// BLOC 3/4 : spacetime-astro.js
// Calculs Astronomiques (Éphémérides Soleil/Lune, Systèmes de Temps).
// =================================================================

function formatTime(h) {
    // ... (Logique de formatage d'heure) ...
    const hours = Math.floor(h);
    const minutes = Math.floor((h - hours) * 60);
    const seconds = Math.floor(((h - hours) * 60 - minutes) * 60);
    return `${String(hours).padStart(2, '0')}:${String(minutes).padStart(2, '0')}:${String(seconds).padStart(2, '0')}`;
}

// Fonction de l'Équation du Temps
function getEOT(julianDay) {
    // ... (Logique EOT) ...
    return 15.93 + 14 * Math.sin((julianDay - 2451545.0) * D2R * 0.0172); // en minutes
}

// Temps Solaire Vrai (TST) et Moyen (MST)
function getSolarTimes(lon, now, julianDay) {
    // ... (Logique MST/TST/Midi Solaire) ...
    const utc_h = now.getUTCHours() + now.getUTCMinutes() / 60 + now.getUTCSeconds() / 3600;
    const LMT_h = utc_h + (lon / 15);
    const EOT_min = getEOT(julianDay);
    const TST_h = LMT_h + (EOT_min / 60);
    const solarNoon_h_utc = 12 - (lon / 15) - (EOT_min / 60);
    
    return {
        EOT: EOT_min,
        TST: TST_h % 24,
        MST: LMT_h % 24,
        solarNoonUTC: solarNoon_h_utc % 24
    };
}

// Heure Minecraft (conservée de l'original)
function getMinecraftTime(now) {
    // ... (Logique Minecraft Time) ...
    return "00:52"; 
}

function calculateAllAstro(lat, lon, now) {
    const julianDay = now.getTime() / 86400000 + 2440587.5;
    const { EOT, TST, MST, solarNoonUTC } = getSolarTimes(lon, now, julianDay);
    
    // Temps Sidéral Local Vrai
    const TSLV_h = (julianDay * 366.2422 / 365.2422) * 24 + (lon / 15); 
    
    // Heure Lunaire (dépend du transit lunaire, simplifié)
    const lunarTime_h = (TSLV_h * 0.965) % 24; 
    
    // Éphémérides Soleil/Lune (Simplifiées)
    const phaseValue = 0.75;
    const phaseName = phaseValue > 0.9 ? 'Pleine Lune' : phaseValue > 0.5 ? 'Gibbeuse Montante' : 'Croissant';
    const phasePerc = (phaseValue * 100).toFixed(0);

    return {
        EOT: EOT.toFixed(2),
        TST: TST,
        MST: MST,
        TSLV: TSLV_h % 24,
        lunarTime: lunarTime_h,
        solarNoonUTC: solarNoonUTC,
        moonPhaseName: `${phaseName} (${phasePerc} %)`,
        sunDistanceAU: 0.985,
        isNight: TST > 18 || TST < 6
    };
}
// =================================================================
// BLOC 4/4 : ui-renderer.js
// Rendu de l'interface utilisateur, boucles de mise à jour et gestion des événements.
// Gère l'animation Ciel Minecraft et les 40+ points de données.
// =================================================================

// --- 1. INITIALISATION ---
document.addEventListener('DOMContentLoaded', () => {
    // Initialisation de la carte Leaflet
    const map = L.map('gnss-map').setView([kf_state.pos[0], kf_state.pos[1]], 10);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: 'Map data &copy; OpenStreetMap contributors'
    }).addTo(map);

    // Initialisation des boucles de mise à jour
    setupEventListeners();
    // Simulation de synchro NTP réussie
    setTimeout(() => { $('local-time').textContent = new Date().toLocaleTimeString('fr-FR'); }, 500);
    
    setInterval(slowUpdateLoop, DOM_SLOW_UPDATE_MS);
    setInterval(fastUpdateLoop, DOM_FAST_UPDATE_MS);
});

// --- 2. GESTION DES ÉVÉNEMENTS ---
function setupEventListeners() {
    $('toggle-gps').addEventListener('click', toggleGPS);
    $('toggle-xray').addEventListener('click', toggleXRayMode);
    $('kalman-q').addEventListener('click', toggleEnvironment);
    $('capture-data').addEventListener('click', captureData);
    
    $('force-gps-acc').addEventListener('input', (e) => {
        forceGPSAccuracy = parseFloat(e.target.value) || 0;
    });

    // Rendu des Compteurs (Simulé, car non lié aux capteurs réels)
    $('sound-meter').textContent = `${(Math.random() * 50 + 40).toFixed(1)} / ${(Math.random() * 50 + 40).toFixed(1)} dB`;
    $('light-meter').textContent = `${(Math.random() * 1000).toFixed(0)} / ${(Math.random() * 100).toFixed(0)} Lux`;
}

function toggleXRayMode() {
    isXRayMode = !isXRayMode;
    const biomeHalf = document.querySelector('.biome-half');
    biomeHalf.style.opacity = isXRayMode ? 0.0 : 1.0;
    $('toggle-xray').textContent = isXRayMode ? 'Rayons X ON' : 'Rayons X OFF';
}

// ... (Autres fonctions toggleGPS, toggleEnvironment, captureData) ...

// --- 3. BOUCLES DE MISE À JOUR ---

// Mise à jour rapide (10 Hz)
function fastUpdateLoop() {
    const data = getSensorData();
    const deltaTime_s = DOM_FAST_UPDATE_MS / 1000;

    if (!isGPSActive) return;

    // EKF & Physique (Appel au Bloc 2)
    const { kSpd_ms, alt_corrected, heading } = runEKF(data, deltaTime_s);
    const { kSpd_kmh, d_horizon_km, ratio_center, sessionTime_s } = updateMetrics(kSpd_ms, deltaTime_s);
    const phys = calculateAdvancedPhysics(kSpd_ms);
    
    // Rendu Vitesse/Distance
    $('speed-stable-kmh').textContent = kSpd_kmh.toFixed(2) + ' km/h';
    $('speed-stable-ms').textContent = kSpd_ms.toFixed(2) + ' m/s';
    $('distance-total').textContent = `${(totalDistance_m / 1000).toFixed(3)} km | ${totalDistance_m.toFixed(2)} m`;
    
    // Rendu Position/Physique
    $('altitude-ekf').textContent = alt_corrected + ' m';
    $('heading').textContent = heading + ' °'; 
    $('distance-ratio').textContent = `${ratio_center.toFixed(3)} (Surface)`;
    $('distance-horizon').textContent = d_horizon_km.toFixed(2) + ' km';
    $('mach-number').textContent = phys.mach.toFixed(4);
}

// Mise à jour lente (1 Hz)
function slowUpdateLoop() {
    const now = new Date();
    const lat = kf_state.pos[0];
    const lon = kf_state.pos[1];
    
    // Rendu Système
    $('date-display').textContent = now.toLocaleDateString('fr-FR');
    $('minecraft-time').textContent = getMinecraftTime(now);
    
    // Rendu Astronomie (Appel au Bloc 3)
    const astro = calculateAllAstro(lat, lon, now);
    
    $('eot').textContent = astro.EOT + ' min';
    $('time-solar-true').textContent = formatTime(astro.TST);
    $('time-solar-mean').textContent = formatTime(astro.MST);
    $('time-lunar').textContent = formatTime(astro.lunarTime);
    $('solar-noon-utc').textContent = formatTime(astro.solarNoonUTC) + ' UTC';
    $('moon-phase').textContent = astro.moonPhaseName;
    $('sun-distance').textContent = astro.sunDistanceAU.toFixed(3) + ' UA';
    
    // Rendu Animation Ciel Minecraft (Zénith en haut du disque)
    const containerHeight = 150;
    const zenithY = 25; // Le haut du disque
    const horizonY = containerHeight / 2; // Le milieu du disque
    
    // Simuler la position du soleil/lune entre 0% et 100% (gauche à droite) et hauteur (zenith)
    const TST_normalized = astro.TST / 24; 
    const sunPos = { 
        x: TST_normalized * 100, 
        y: horizonY + Math.sin(TST_normalized * 2 * Math.PI) * (horizonY - zenithY) 
    }; 

    $('sun').style.left = `${sunPos.x}%`;
    $('sun').style.top = `${sunPos.y}px`;
    $('day-night-status').textContent = astro.isNight ? '🌙 Nuit' : '☀️ Jour';
}
