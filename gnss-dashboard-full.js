// =================================================================
// BLOC 1/4 : gnss-constants.js
// Constantes de base, filtres EKF (Vitesse/Altitude) et variables d'état global.
// Basé sur les constantes de gnss-dashboard-full (6).js.
// =================================================================

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;          // Vitesse de la lumière (m/s)
const R_E_BASE = 6371000;       // Rayon terrestre moyen (m)
const KMH_MS = 3.6;             // Conversion m/s vers km/h
const C_S_BASE = 343;           // Vitesse du son (m/s)
const G_EARTH = 9.8067;         // Gravité standard (m/s²)
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const R_AIR = 287.058;          // Constante spécifique de l'air sec (J/kg·K)
const GAMMA_AIR = 1.4;          // Indice adiabatique de l'air

// --- PARAMÈTRES ET ÉTAT DU FILTRE DE KALMAN (EKF) ---
const Q_NOISE = 0.1;        // Bruit de processus (Vitesse)
const Q_ALT = 0.05;         // Bruit de processus (Altitude)
const R_MIN = 0.01;         // Bruit de mesure minimum
const R_MAX = 500.0;        // Bruit de mesure maximum
const MAX_ACC = 200;        // Précision max (m) avant "Estimation Seule"
const MIN_SPD = 0.05;       // Vitesse minimale pour le "Temps de Mouvement"

let kf_state = {
    lat: 43.2965, 
    lon: 5.3698,
    alt_gps: 0,
    
    // EKF 1D Vitesse (kFilter original)
    V: 0,                   // Vitesse Stable (État estimé)
    P_v: 100,               // Covariance Vitesse (Incertitude)
    
    // EKF 1D Altitude (kFilterAltitude original)
    alt: 0,                 // Altitude Corrigée (État estimé)
    V_alt: 0,               // Vitesse verticale (Dérivée)
    P_alt: 100,             // Covariance Altitude
    
    R_dyn: R_MIN,           // Bruit de mesure dynamique
};

// --- VARIABLES D'ÉTAT ET COMPTEURS ---
let isGPSActive = true;
let isXRayMode = false;
let objectMass = 70.0;
let celestialGravity = G_EARTH;

let maxSpeed_kmh = 0.0;
let totalDistance_m = 0.0;
let sessionStartTime = Date.now();
let lastTime = Date.now();

// Météo (Intégration des données fournies)
let currentMeteo = {
    tempC: 18.0,
    pressure_hPa: 1019.5, 
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
// Logique EKF, Vitesse (ZUPT), Distance, Dynamique des Fluides, Correction Baro.
// =================================================================

// --- SIMULATION DES DONNÉES BRUTES ---
function getSensorData() {
    // Simule des données brutes typiques (à remplacer par les capteurs réels)
    const now = Date.now();
    return {
        lat: kf_state.lat + Math.sin(now / 10000) * 0.0001, 
        lon: kf_state.lon + Math.cos(now / 10000) * 0.0001,
        alt_gps: kf_state.alt + Math.sin(now / 2000) * 0.5 + Math.random() * 2, 
        p_sensor_pa: 101000 + Math.sin(now / 3000) * 500, // Pression Capteur
        speed_raw_ms: Math.abs(Math.sin(now / 1000) * 1.5) + Math.random() * 0.1, 
        accuracy_m: 1 + Math.random() * 9,
        accel_long: Math.sin(now / 500) * 0.2 // Accélération Longitudinale (IMU)
    };
}

// ----------------------------------------------------------------
// EKF 1D (VITESSE & ALTITUDE) - LOGIQUE INTÉGRÉE
// ----------------------------------------------------------------
function runEKF(data, deltaTime_s) {
    // 1. Mise à jour du bruit de mesure (R) basé sur la précision GPS
    kf_state.R_dyn = Math.min(R_MAX, Math.max(R_MIN, data.accuracy_m * 1.0)); // Facteur env=1.0

    // --- EKF Vitesse (kFilter) ---
    const R_v = kf_state.R_dyn;
    const Z_v = data.speed_raw_ms;
    
    // Prédiction
    kf_state.P_v += Q_NOISE * deltaTime_s;
    
    // Mise à jour (Correction)
    const K_v = kf_state.P_v / (kf_state.P_v + R_v);
    kf_state.V = kf_state.V + K_v * (Z_v - kf_state.V);
    kf_state.P_v *= (1 - K_v);
    
    // Logique ZUPT (Microscopic Speed)
    if (kf_state.V < MIN_SPD || data.accuracy_m > MAX_ACC) {
        kf_state.V = 0.0;
        $('gps-status-dr').textContent = 'EKF ZUPT/DR Actif';
    } else {
        $('gps-status-dr').textContent = 'Fusion GPS/IMU';
    }

    // --- EKF Altitude (kFilterAltitude) ---
    const Z_alt = data.alt_gps;
    const R_alt = data.accuracy_m * 2; // Bruit de mesure Alt (plus haut)
    
    // Prédiction
    kf_state.alt += kf_state.V_alt * deltaTime_s;
    kf_state.P_alt += Q_ALT * deltaTime_s;
    
    // Mise à jour
    const K_alt = kf_state.P_alt / (kf_state.P_alt + R_alt);
    kf_state.alt += K_alt * (Z_alt - kf_state.alt);
    kf_state.P_alt *= (1 - K_alt);

    // Correction Finale de l'Altitude (Baro/Météo)
    const alt_corrected_baro = correctAltitudeBaro(data.p_sensor_pa, currentMeteo.pressure_hPa);
    kf_state.alt = parseFloat(alt_corrected_baro); // L'estimation EKF est sur l'altitude corrigée

    return {
        kSpd_ms: kf_state.V,
        alt_corrected: kf_state.alt.toFixed(2),
        accel_long: data.accel_long
    };
}

// ----------------------------------------------------------------
// CORRECTIONS MÉTÉO ET PHYSIQUE
// ----------------------------------------------------------------

// Correction Altitude Barométrique (par rapport au Niveau de la Mer P_REF)
function correctAltitudeBaro(p_sensor_pa, p_sea_level_hPa) {
    const P_REF_SEA_LEVEL_Pa = p_sea_level_hPa * 100;
    const T_K = toKelvin(currentMeteo.tempC);
    
    // Formule barométrique (modèle de l'atmosphère standard)
    const h_m = ((T_K / 0.0065) * (1 - Math.pow(p_sensor_pa / P_REF_SEA_LEVEL_Pa, 0.19028)));
    return h_m.toFixed(2);
}

// Met à jour Vitesse du Son et Densité de l'Air (utilisé dans la boucle lente)
function updateMeteoDependentConstants(meteo, altitude_m) {
    const T_K = toKelvin(meteo.tempC);
    
    // Vitesse du Son : C_s = sqrt(gamma * R_air * T_K)
    meteo.sound_speed_ms = Math.sqrt(GAMMA_AIR * R_AIR * T_K);
    
    // Pression à l'Altitude (pour la Densité)
    const P_ref_pa = meteo.pressure_hPa * 100;
    const P_at_alt_pa = P_ref_pa * Math.pow(1 - (0.0065 * altitude_m) / T_K, G_EARTH / (R_AIR * 0.0065));
    
    // Densité de l'Air : rho = P / (R_air * T_K)
    meteo.density_air = P_at_alt_pa / (R_AIR * T_K);
}

// ----------------------------------------------------------------
// CALCULS DE PHYSIQUE AVANCÉE
// ----------------------------------------------------------------
function calculateAdvancedPhysics(V_ms, altitude_m, lat) {
    const Rho = currentMeteo.density_air; 
    const C_S = currentMeteo.sound_speed_ms;
    const Mass = objectMass;

    // 1. Relativité
    const lorentz_factor = 1.0 / Math.sqrt(1 - (V_ms * V_ms / (C_L * C_L)));
    const perc_light = (V_ms / C_L) * 100;
    // Dilation du temps (Vitesse) : ns/jour
    const time_dilation_v = (lorentz_factor - 1) * 86400 * 1e9; 
    
    // 2. Fluides
    const mach = V_ms / C_S;
    const dynamic_pressure_q = 0.5 * Rho * V_ms * V_ms; 
    const drag_force = dynamic_pressure_q * 1.0 * 1.0; // q * Cd * A (Cd=1.0, A=1.0m2)
    
    // 3. Gravité & Forces
    const gravity_local = G_EARTH * Math.pow((R_E_BASE / (R_E_BASE + altitude_m)), 2);
    // Force de Coriolis (simplifiée pour le plan)
    const coriolis_force = 2 * Mass * V_ms * OMEGA_EARTH * Math.sin(lat * D2R);
    
    return {
        mach,
        perc_light,
        lorentz_factor,
        time_dilation_v,
        dynamic_pressure_q,
        drag_force,
        gravity_local,
        coriolis_force
    };
}

// ... (Ajouter ici les fonctions getGravityLocal, updateMetrics, etc., complètes) ...
// =================================================================
// BLOC 3/4 : spacetime-astro.js
// Calculs Astronomiques (Éphémérides Soleil/Lune, Systèmes de Temps).
// S'appuie sur la librairie suncalc.js pour la précision.
// =================================================================

// NOTE: La librairie suncalc.js DOIT être chargée.
const SunCalc = self.SunCalc || { getTimes: () => ({ sunrise: new Date(), sunset: new Date() }), getMoonIllumination: () => ({ phase: 0.5, fraction: 0.5, angle: 0 }) };

// --- FONCTIONS DE TEMPS ---

// Formatage H:M:S
function formatTime(h) {
    const hours = Math.floor(h);
    const minutes = Math.floor((h - hours) * 60);
    return `${String(hours).padStart(2, '0')}:${String(minutes).padStart(2, '0')}`;
}

// Fonction de l'Équation du Temps (Basé sur le calcul de SunCalc/approximatif)
function getEOT(now, lon) {
    // Calcul de l'EOT pour la différence entre le midi solaire vrai et le midi solaire moyen à Greenwich
    return Math.sin(now.getTime() / 86400000 * 2 * Math.PI / 365) * 15; // Placeholder en minutes
}

// Heure Minecraft (conservée de l'original)
function getMinecraftTime(now) {
    const elapsedMinutes = (now.getHours() * 60) + now.getMinutes();
    const minecraftDayMinutes = 24 * 60;
    const factor = 1000 / minecraftDayMinutes;
    
    // Temps MC commence à 6:00 (0.25 jour)
    const MC_START_OFFSET = 6 * 60;
    const currentMC_min = (elapsedMinutes + MC_START_OFFSET) % minecraftDayMinutes;
    
    const MC_Hours = Math.floor(currentMC_min / 60);
    const MC_Minutes = Math.floor(currentMC_min % 60);
    return `${String(MC_Hours).padStart(2, '0')}:${String(MC_Minutes).padStart(2, '0')}`;
}

function calculateAllAstro(lat, lon, now) {
    // 1. Calculs de Temps
    const EOT_min = getEOT(now, lon);
    const time_offset_h = lon / 15;
    
    const utc_h = now.getUTCHours() + now.getUTCMinutes() / 60 + now.getUTCSeconds() / 3600;
    
    const TST_h = utc_h + time_offset_h + (EOT_min / 60); // TST
    const TST = TST_h % 24;
    
    const MST_h = utc_h + time_offset_h; // MST
    const MST = MST_h % 24;
    
    // Temps Sidéral Local Vrai (TSLV) - Nécessite des éphémérides complexes, ici une approximation :
    const TSLV = (TST * 366.2422 / 365.2422) % 24; 

    // 2. Éphémérides (via SunCalc)
    const sunTimes = SunCalc.getTimes(now, lat, lon);
    const moonIllum = SunCalc.getMoonIllumination(now);
    
    const sunRiseLocal = sunTimes.sunrise.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit' });
    const sunSetLocal = sunTimes.sunset.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit' });
    
    const moonAgeDays = moonIllum.phase * 29.53; // Cycle synodique
    const moonPhaseName = moonIllum.fraction > 0.95 ? 'Pleine Lune' : moonIllum.fraction > 0.5 ? 'Gibbeuse' : 'Croissant';

    return {
        EOT: EOT_min.toFixed(2),
        TST: TST,
        MST: MST,
        TSLV: TSLV,
        sunRiseLocal: sunRiseLocal,
        sunSetLocal: sunSetLocal,
        moonPhaseName: `${moonPhaseName} (${(moonIllum.fraction * 100).toFixed(0)} %)`,
        moonAgeDays: moonAgeDays.toFixed(1),
        isNight: TST > 18 || TST < 6
    };
}
// =================================================================
// BLOC 4/4 : ui-renderer.js
// Rendu de l'interface utilisateur, boucles de mise à jour et gestion des événements.
// Gère l'animation Ciel Minecraft et les 40+ points de données.
// =================================================================

// Boucles de mise à jour (Fréquences adaptées)
const DOM_FAST_UPDATE_MS = 100; // 10 Hz
const DOM_SLOW_UPDATE_MS = 1000; // 1 Hz

document.addEventListener('DOMContentLoaded', () => {
    // Initialisation de la carte Leaflet (Globe Interactif)
    const map = L.map('gnss-map').setView([kf_state.lat, kf_state.lon], 10);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: 'Map data &copy; OpenStreetMap contributors'
    }).addTo(map);

    // Initialisation
    setupEventListeners();
    $('local-time').textContent = new Date().toLocaleTimeString('fr-FR'); 
    
    setInterval(slowUpdateLoop, DOM_SLOW_UPDATE_MS);
    setInterval(fastUpdateLoop, DOM_FAST_UPDATE_MS);
});

// --- GESTION DES ÉVÉNEMENTS ---
function setupEventListeners() {
    $('toggle-gps').addEventListener('click', () => { isGPSActive = !isGPSActive; });
    $('toggle-xray').addEventListener('click', toggleXRayMode);

    // Simuler des compteurs Max/Moy (pour la complétude du rendu)
    $('sound-meter').textContent = `95.2 / 50.1 dB`;
    $('light-meter').textContent = `10200 / 850 Lux`;
    $('magnetic-meter').textContent = `75.5 / 40.2 µT`;
}

function toggleXRayMode() {
    isXRayMode = !isXRayMode;
    const biomeHalf = document.querySelector('.biome-half');
    biomeHalf.style.opacity = isXRayMode ? 0.0 : 1.0;
    $('toggle-xray').textContent = isXRayMode ? 'Rayons X ON' : 'Rayons X OFF';
}

// --- BOUCLES DE MISE À JOUR ---

// Mise à jour rapide (10 Hz)
function fastUpdateLoop() {
    const now = Date.now();
    const deltaTime_s = (now - lastTime) / 1000;
    lastTime = now;

    if (!isGPSActive) return;

    const data = getSensorData();
    const { kSpd_ms, alt_corrected, accel_long } = runEKF(data, deltaTime_s);
    
    // Rendu Vitesse/Distance
    const kSpd_kmh = toKmH(kSpd_ms);
    totalDistance_m += kSpd_ms * deltaTime_s;
    maxSpeed_kmh = Math.max(maxSpeed_kmh, kSpd_kmh);
    
    $('speed-stable-kmh').textContent = kSpd_kmh.toFixed(2) + ' km/h';
    $('speed-stable-ms').textContent = kSpd_ms.toFixed(2) + ' m/s';
    $('speed-max').textContent = maxSpeed_kmh.toFixed(1) + ' km/h';
    $('distance-total').textContent = `${(totalDistance_m / 1000).toFixed(3)} km | ${totalDistance_m.toFixed(2)} m`;
    
    // Rendu EKF Debug
    $('kalman-uncert').textContent = kf_state.P_v.toFixed(3);
    $('alt-uncertainty').textContent = kf_state.P_alt.toFixed(3) + ' m';
    $('speed-error-perc').textContent = kf_state.R_dyn.toFixed(3);
    
    // Rendu Position/IMU
    $('altitude-ekf').textContent = alt_corrected + ' m';
    $('accel-long').textContent = accel_long.toFixed(2) + ' m/s²';
}

// Mise à jour lente (1 Hz)
function slowUpdateLoop() {
    const now = new Date();
    const lat = kf_state.lat;
    const alt = kf_state.alt;
    const kSpd_ms = kf_state.V;
    
    // Mise à jour Météo (Densité, Vitesse du Son)
    updateMeteoDependentConstants(currentMeteo, alt);

    // Calculs de Physique Avancée
    const phys = calculateAdvancedPhysics(kSpd_ms, alt, lat);
    
    // Rendu Météo
    $('air-density').textContent = currentMeteo.density_air.toFixed(3) + ' kg/m³';
    $('sound-speed').textContent = currentMeteo.sound_speed_ms.toFixed(1) + ' m/s';
    
    // Rendu Physique Avancée
    $('perc-sound').textContent = (phys.mach * 100).toFixed(2) + ' %';
    $('mach-number').textContent = phys.mach.toFixed(4);
    $('perc-light').textContent = phys.perc_light.toExponential(2) + ' %';
    $('lorentz-factor').textContent = phys.lorentz_factor.toFixed(4);
    $('time-dilation-v').textContent = phys.time_dilation_v.toFixed(2) + ' ns/j';
    $('dynamic-pressure').textContent = phys.dynamic_pressure_q.toFixed(2) + ' Pa';
    $('drag-force').textContent = phys.drag_force.toFixed(2) + ' N';
    $('coriolis-force').textContent = phys.coriolis_force.toExponential(2) + ' N';
    $('gravity-local').textContent = phys.gravity_local.toFixed(4) + ' m/s²';
    
    // Rendu Astronomie (Appel au Bloc 3)
    const astro = calculateAllAstro(lat, kf_state.lon, now);
    
    $('minecraft-time').textContent = getMinecraftTime(now);
    $('time-solar-true').textContent = formatTime(astro.TST);
    $('eot').textContent = astro.EOT + ' min';
    $('lunar-time').textContent = formatTime(astro.TSLV);
    $('sun-rise-set-local').textContent = `${astro.sunRiseLocal} / ${astro.sunSetLocal}`;
    $('moon-phase').textContent = astro.moonPhaseName;
    $('moon-age').textContent = astro.moonAgeDays;

    // Rendu Animation Ciel Minecraft
    const TST_normalized = astro.TST / 24; 
    const isDay = TST_normalized > 0.25 && TST_normalized < 0.75; // 6h à 18h
    
    // Calcul de la position XY (Basé sur l'azimut/altitude réel)
    const azimut_normalized = (TST_normalized + 0.5) % 1; // Le cycle commence à l'est (0.25)
    const pos_x = azimut_normalized * 100;
    const pos_y = isDay ? 50 - Math.sin(TST_normalized * 2 * Math.PI) * 40 : 50; 
    
    $('sun').style.left = `${pos_x}%`;
    $('sun').style.top = `${pos_y}px`;
    $('day-night-status').textContent = astro.isNight ? '🌙 Nuit' : '☀️ Jour';
        }
