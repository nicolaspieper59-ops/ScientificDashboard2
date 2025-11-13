// =================================================================
// BLOC 1/3 : gnss-dashboard-1-core.js
// Définitions des Constantes et Initialisation de l'État Global (let).
// DOIT être chargé en premier.
// =================================================================

// --- 1. DÉCLARATIONS GLOBALES ET CONSTANTES ---

// Constantes Physiques et Mathématiques
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;          // Vitesse de la lumière (m/s)
const G_CONST = 6.67430e-11;    // Constante gravitationnelle (N(m/kg)²)
const M_EARTH = 5.972e24;       // Masse de la Terre (kg)
const R_E_BASE = 6371000;       // Rayon terrestre moyen (m)
const KMH_MS = 3.6;             // Conversion m/s vers km/h
const C_S_BASE = 343;           // Vitesse du son de base (m/s)
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const R_AIR = 287.058;          // Constante spécifique de l'air sec (J/kg·K)
const DYNAMIC_VISCOSITY_AIR = 1.81e-5; // Viscosité dynamique de l'air à 15°C (Pa·s)
const DOM_SLOW_UPDATE_MS = 1000; // 1 Hz

// Paramètres du Filtre de Kalman
const Q_NOISE = 0.1;        // Bruit de processus (Vitesse)
const Q_ALT_NOISE = 0.05;   // Bruit de processus (Altitude)
const R_MIN = 0.01;         // Bruit de mesure minimum
const R_MAX = 500.0;        // Bruit de mesure maximum
const MIN_SPD = 0.05;       // Vitesse minimale pour le mouvement
const MIN_DT = 0.05;        // Minimum Delta Time (s)
const ZUPT_RAW_THRESHOLD = 0.5; // Vitesse brute pour ZUPT (m/s)
const MAX_PLAUSIBLE_ACCEL = 50; // Accélération max anti-spike (m/s²)
const NETHER_RATIO = 8;     // Ratio Monde Réel / Nether

// État Global du Système (Variables LET)
let wID = null;             // ID du watchPosition
let sTime = null;           // Temps de début de session (timestamp)
let lPos = null;            // Position de l'itération précédente
let lastFSpeed = 0.0;       // Dernière vitesse filtrée (m/s)
let timeMoving = 0;         // Temps de mouvement (s)
let maxSpd = 0;             // Vitesse Max (m/s)
let distM = 0;              // Distance totale parcourue (m)
let V_avg_total = 0;        // Vitesse moyenne totale (m/s)
let emergencyStopActive = false; // Statut Arrêt d'Urgence

// EKF État
let kSpd = 0.0;             // Vitesse filtrée (m/s)
let kUncert = 1000.0;       // Incertitude Vitesse (P)
let kAlt = null;            // Altitude filtrée (m)
let kAltUncert = 10.0;      // Incertitude Altitude (P)

// Environnement et Physique
let currentMass = 70.0;
let currentCelestialBody = 'TERRE';
let G_ACC = 9.8067;         // Gravité de base
let R_ALT_CENTER_REF = R_E_BASE; // Rayon du corps céleste
let netherMode = false;
let gpsAccuracyOverride = 0.0;
let ENVIRONMENT_FACTORS = { NORMAL: { R_MULT: 1.0 } }; // Facteurs d'environnement pour R
let selectedEnvironment = 'NORMAL';

// Météo (Simulation initiale)
let lastT_K = 291.15; // Température initiale en Kelvin (18.0 °C)
let lastP_hPa = 1019.5; // Pression initiale
let lastH_perc = 0.66; // Humidité initiale
let currentMeteo = {
    tempC: 18.0, pressure_hPa: 1019.5, air_density: 1.200, sound_speed_ms: 341.8 
};

// Carte
let map = null;
let marker = null;
let line = null;
let pathCoords = [];

// Temps
let lServH = 0;             // Dernière heure serveur NTP
let lLocH = 0;              // Dernière heure locale
// =================================================================
// BLOC 2/3 : gnss-dashboard-2-logic.js
// Fonctions de Calcul, Filtres EKF et Logique Principale.
// DOIT être chargé après le BLOC 1.
// =================================================================

// --- 2. FONCTIONS UTILITAIRES ET MATHÉMATIQUES ---

const $ = id => document.getElementById(id);
const toKmH = ms => ms * KMH_MS;
const formatTime = h => {
    const hours = Math.floor(h);
    const minutes = Math.floor((h - hours) * 60);
    return `${String(hours).padStart(2, '0')}:${String(minutes).padStart(2, '0')}`;
};

function dist(lat1, lon1, lat2, lon2, radius) {
    const r = radius; 
    const dLat = (lat2 - lat1) * D2R;
    const dLon = (lon2 - lon1) * D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1 * D2R) * Math.cos(lat2 * D2R) * Math.sin(dLon / 2) ** 2;
    return r * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
}

// --- 3. FILTRES DE KALMAN 1D ---

function kFilter(kState, kUncert, Z_input, dt, R_input, accel_sensor_input = 0) {
    const F = 1;
    const u = accel_sensor_input * dt;
    // Prédiction
    kState = F * kState + u;
    kUncert += Q_NOISE * dt;
    
    // Mise à jour (Correction)
    const K = kUncert / (kUncert + R_input);
    kState = kState + K * (Z_input - kState);
    kUncert = kUncert * (1 - K);
    
    return { kSpd: kState, kUncert: kUncert };
}

function kFilterAltitude(kAlt, kAltUncert, altRaw, R_alt, dt) {
    // Prédiction (Altitude constante)
    kAltUncert += Q_ALT_NOISE * dt;
    
    // Mise à jour (Correction)
    const K_alt = kAltUncert / (kAltUncert + R_alt);
    kAlt = kAlt + K_alt * (altRaw - kAlt);
    kAltUncert = kAltUncert * (1 - K_alt);
    
    return { kAlt: kAlt, kAltUncert: kAltUncert };
}

// --- 4. FONCTIONS DE PHYSIQUE ET D'ASTRO ---

function calculateMRF(kAlt, netherMode) {
    if (netherMode) return 1 / NETHER_RATIO;
    return (R_ALT_CENTER_REF + kAlt) / R_ALT_CENTER_REF; 
}

function getGravityLocal(alt, body, R_ref, G_base) {
    if (body === 'TERRE') {
        return G_base * Math.pow((R_ref / (R_ref + alt)), 2);
    }
    return G_base;
}

function calculateAdvancedPhysics(kSpd, kAlt, currentMass, local_g) {
    const alt_m = kAlt || 0;
    const V_ms = kSpd;
    const Rho = currentMeteo.air_density;

    // Relativité
    const lorentz_factor = 1.0 / Math.sqrt(1 - (V_ms * V_ms / (C_L * C_L)));
    const time_dilation_v = (lorentz_factor - 1) * 86400 * 1e9; 
    const E_0 = currentMass * C_L ** 2; 
    const E = lorentz_factor * E_0;     
    const R_s = (2 * G_CONST * currentMass) / (C_L ** 2); 
    const time_dilation_grav = (G_CONST * M_EARTH / (C_L ** 2 * R_E_BASE ** 2)) * alt_m * 86400 * 1e9; 
    
    // Fluides
    const mach = V_ms / currentMeteo.sound_speed_ms;
    const dynamic_pressure_q = 0.5 * Rho * V_ms * V_ms; 
    const drag_force = dynamic_pressure_q * 1.0 * 1.0; 
    const reynolds_number = (Rho * V_ms * 1.0) / DYNAMIC_VISCOSITY_AIR; 
    
    // Géophysique
    const alt_geopotentielle = R_E_BASE * alt_m / (R_E_BASE + alt_m);

    return {
        lorentz_factor, time_dilation_v, time_dilation_grav, E_0, E, R_s,
        mach, dynamic_pressure_q, drag_force, reynolds_number, alt_geopotentielle
    };
}

// Fonctions de Temps/Astro
function getCDate(lServH, lLocH) {
    if (lServH === 0 || lLocH === 0) return null;
    const offset = Date.now() - lLocH;
    return new Date(lServH + offset);
}

async function syncH() {
    return new Promise(resolve => {
        const now = Date.now();
        const simulatedServerTime = now + Math.floor(Math.random() * 500) - 250;
        resolve({ lServH: simulatedServerTime, lLocH: now });
    });
}

function getMinecraftTime(now) {
    const totalMinutes = now.getHours() * 60 + now.getMinutes();
    const MC_MINUTES_PER_DAY = 1440;
    const minutesSinceMCStart = (totalMinutes + (6 * 60)) % MC_MINUTES_PER_DAY; 
    const MC_Hours = Math.floor(minutesSinceMCStart / 60);
    const MC_Minutes = Math.floor(minutesSinceMCStart % 60);
    return `${String(MC_Hours).padStart(2, '0')}:${String(MC_Minutes).padStart(2, '0')}`;
}

function getEOT(now) {
    const n = Math.floor((now - new Date(now.getFullYear(), 0, 0)) / 86400000); 
    const B = (2 * Math.PI / 365) * (n - 81);
    return (9.87 * Math.sin(2 * B) - 7.53 * Math.cos(B) - 1.5 * Math.sin(B)) / 60;
}

function calculateAllAstro(lat, lon, now) {
    if (typeof SunCalc === 'undefined' || !lat || !lon) {
        return {
            TST_h: 'N/A', MST_h: 'N/A', TSLV_h: 'N/A', EOT_min: 'N/A',
            sunRiseSet: 'N/A / N/A', moonPhase: 'N/A', moonAgeDays: 'N/A',
            moonDistance: 'N/A', sunDistanceAU: 'N/A'
        };
    }
    
    const EOT_min = getEOT(now);
    const time_offset_h = lon / 15;
    const utc_h = now.getUTCHours() + now.getUTCMinutes() / 60 + now.getUTCSeconds() / 3600;
    
    const TST_h = (utc_h + time_offset_h + (EOT_min / 60)) % 24; 
    const MST_h = (utc_h + time_offset_h) % 24;
    const TSLV = (MST_h * 366.2422 / 365.2422) % 24; 
    
    const sunTimes = SunCalc.getTimes(now, lat, lon);
    const moonIllum = SunCalc.getMoonIllumination(now);
    const moonPos = SunCalc.getMoonPosition(now, lat, lon);
    
    const sunRiseLocal = sunTimes.sunrise ? sunTimes.sunrise.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit' }) : 'N/A';
    const sunSetLocal = sunTimes.sunset ? sunTimes.sunset.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit' }) : 'N/A';
    
    const moonPhaseName = moonIllum.fraction > 0.95 ? 'Pleine Lune' : moonIllum.fraction > 0.5 ? 'Gibbeuse' : 'Croissant';
    
    return {
        TST_h: TST_h, MST_h: MST_h, TSLV_h: TSLV, EOT_min: EOT_min,
        sunRiseSet: `${sunRiseLocal} / ${sunSetLocal}`,
        moonPhase: `${moonPhaseName} (${(moonIllum.fraction * 100).toFixed(0)} %)`,
        moonAgeDays: (moonIllum.phase * 29.53).toFixed(1),
        moonDistance: (moonPos.distance * 1000).toFixed(0), 
        sunDistanceAU: (149597870700 / 149597870700).toFixed(4)
    };
}

// Fonction Météo (Simulation)
async function fetchWeather(lat, lon) {
    const lastT_K = currentMeteo.tempC + 273.15;
    const air_density = (lastP_hPa * 100) / (R_AIR * lastT_K);
    const sound_speed_ms = C_S_BASE + (lastT_K - 288.15) * 0.6;
    
    currentMeteo = {
        tempC: lastT_K - 273.15, pressure_hPa: lastP_hPa, humidity_perc: lastH_perc * 100,
        dewPointC: (lastT_K - 273.15) - ((100 - lastH_perc * 100) / 5), 
        air_density: air_density, sound_speed_ms: sound_speed_ms
    };
    
    return {
        tempC: currentMeteo.tempC, pressure_hPa: currentMeteo.pressure_hPa,
        humidity_perc: currentMeteo.humidity_perc, dew_point: currentMeteo.dewPointC,
        air_density: currentMeteo.air_density,
        wind_speed: 24, wind_gust: 44, wind_dir: 'SE', cloud_cover: 57, 
        cloud_ceiling: 9100, visibility: 16.0, uv_index: 0.6 
    };
}


// --- 5. FONCTION PRINCIPALE DE TRAITEMENT GPS (High Frequency) ---

function processGPSData(pos) {
    
    const cLat = pos.coords.latitude;
    const cLon = pos.coords.longitude;
    const altRaw = pos.coords.altitude;
    const accRaw = pos.coords.accuracy;
    const headingRaw = pos.coords.heading;
    const spd3D_gps_raw = pos.coords.speed || 0.0;
    const cTimePos = pos.timestamp;
    
    let modeStatus = 'EKF GPS Stable';
    
    let acc = (gpsAccuracyOverride > 0) ? gpsAccuracyOverride : accRaw;
    let R_dyn = ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT * acc * acc; 
    R_dyn = Math.min(R_dyn, R_MAX); 
    
    let dt;
    if (lPos && sTime) {
        dt = (cTimePos - lPos.timestamp) / 1000;
        V_avg_total = distM / ((cTimePos - sTime) / 1000);
    } else {
        lPos = pos; 
        lPos.speedMS_3D = 0;
        lPos.kAlt_old = altRaw;
        kAlt = altRaw;
        sTime = cTimePos;
        return; 
    }
    
    if (dt < MIN_DT || dt > 10) { 
        lPos = pos;
        return; 
    }

    // EKF Altitude
    const { kAlt: kAlt_new, kAltUncert: kAltUncert_new } = kFilterAltitude(
        kAlt, kAltUncert, altRaw, pos.coords.altitudeAccuracy || R_MIN, dt
    );
    kAlt = kAlt_new;
    kAltUncert = kAltUncert_new;
    
    // Calcul Vitesse Brute (si GPS Speed indisponible)
    const dist2D = dist(lPos.coords.latitude, lPos.coords.longitude, cLat, cLon, R_ALT_CENTER_REF);
    const distAlt = kAlt_new - (lPos.kAlt_old || kAlt_new);
    const dist3D = Math.sqrt(dist2D ** 2 + distAlt ** 2);
    let spd3D_raw = spd3D_gps_raw > 0 ? spd3D_gps_raw : dist3D / dt; 
    const spdV = distAlt / dt; 

    // Anti-Spike Vitesse
    if (lPos && lPos.speedMS_3D !== undefined) {
        const lastRawSpd = lPos.speedMS_3D;
        const accelSpike = Math.abs(spd3D_raw - lastRawSpd) / dt;
        if (accelSpike > MAX_PLAUSIBLE_ACCEL) {
            const maxPlausibleChange = MAX_PLAUSIBLE_ACCEL * dt;
            spd3D_raw = (spd3D_raw > lastRawSpd) ? (lastRawSpd + maxPlausibleChange) : (lastRawSpd - maxPlausibleChange);
        }
    }
    
    // Logique ZUPT
    let spd_kalman_input = spd3D_raw;
    let R_kalman_input = R_dyn;
    
    const isPlausiblyStopped = (spd3D_raw < ZUPT_RAW_THRESHOLD && R_dyn < R_MAX); 
    
    if (isPlausiblyStopped) { 
        spd_kalman_input = 0.0;
        R_kalman_input = R_MIN;
        modeStatus = '✅ ZUPT (Vélocité Nulle Forcée)';
    }

    // EKF Vitesse
    const { kSpd: fSpd, kUncert: kUncert_new } = kFilter(kSpd, kUncert, spd_kalman_input, dt, R_kalman_input, 0);
    kSpd = fSpd;
    kUncert = kUncert_new;
    
    const sSpdFE = fSpd < MIN_SPD ? 0 : fSpd; 
    
    // Calculs de distance/temps
    let accel_long = 0;
    if (dt > 0.05) { accel_long = (sSpdFE - lastFSpeed) / dt; }
    lastFSpeed = sSpdFE;

    const R_FACTOR_RATIO = calculateMRF(kAlt_new, netherMode); 
    distM += sSpdFE * dt * R_FACTOR_RATIO; 
    
    if (sSpdFE > MIN_SPD) { timeMoving += dt; }
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    
    const local_g = getGravityLocal(kAlt_new, currentCelestialBody, R_ALT_CENTER_REF, G_ACC); 
    const coriolis_force = 2 * currentMass * sSpdFE * OMEGA_EARTH * Math.sin(cLat * D2R);

    const phys = calculateAdvancedPhysics(sSpdFE, kAlt_new, currentMass, local_g);

    // Mise à jour de l'interface rapide (définie dans le BLOC 3)
    updateFastDOM(cTimePos, cLat, cLon, kAlt_new, spd3D_raw, headingRaw, spdV, accel_long, coriolis_force, local_g, phys, acc, R_dyn, sSpdFE, R_FACTOR_RATIO, modeStatus);

    // Sauvegarde de l'état
    lPos = pos; 
    lPos.speedMS_3D = spd3D_raw; 
    lPos.timestamp = cTimePos; 
    lPos.kAlt_old = kAlt_new; 
        }
// =================================================================
// BLOC 3/3 : gnss-dashboard-3-setup.js
// Fonctions DOM, Carte, Contrôle GPS et Initialisation finale.
// DOIT être chargé après le BLOC 2.
// =================================================================

// --- 6. GESTION DE LA CARTE ET DU GPS ---

function initMap() {
    if (!$('gnss-map') || typeof L === 'undefined') return;
    map = L.map('gnss-map').setView([43.2965, 5.3698], 13);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; OpenStreetMap contributors'
    }).addTo(map);
    marker = L.marker([43.2965, 5.3698]).addTo(map);
    line = L.polyline(pathCoords, {color: 'red'}).addTo(map);
}

function updateMap(lat, lon, accuracy) {
    if (!map || !marker) return;
    const latlng = L.latLng(lat, lon);
    marker.setLatLng(latlng);
    map.setView(latlng, map.getZoom() > 10 ? map.getZoom() : 15);
    
    pathCoords.push([lat, lon]);
    if (line) { line.setLatLngs(pathCoords); }
}

function startGPS() {
    if (wID !== null) return;
    const options = { enableHighAccuracy: true, maximumAge: 100, timeout: 5000 };
    
    wID = navigator.geolocation.watchPosition(
        (pos) => { if (!emergencyStopActive) processGPSData(pos); },
        (err) => { console.error('Erreur GPS:', err); }, 
        options
    );
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = '⏸️ PAUSE GPS';
    if ($('statut-gps-ekf')) $('statut-gps-ekf').textContent = 'EKF/GPS Actif';
}

function stopGPS() {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
    }
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = '▶️ DÉMARRER GPS';
    if ($('statut-gps-ekf')) $('statut-gps-ekf').textContent = 'Arrêté';
}


// --- 7. MISE À JOUR DOM RAPIDE (Appelée par processGPSData) ---

function updateFastDOM(cTimePos, cLat, cLon, kAlt_new, spd3D_raw, headingRaw, spdV, accel_long, coriolis_force, local_g, phys, acc, R_dyn, sSpdFE, R_FACTOR_RATIO, modeStatus) {
    const elapsed_s = (cTimePos - sTime) / 1000;
    
    // Contrôles & Système
    if ($('time-elapsed')) $('time-elapsed').textContent = `${elapsed_s.toFixed(2)} s`;
    if ($('time-moving')) $('time-moving').textContent = `${timeMoving.toFixed(2)} s`;
    if ($('rapport-dist-centre-terre')) $('rapport-dist-centre-terre').textContent = `${R_FACTOR_RATIO.toFixed(3)} (Surface)`;
    if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;

    // Vitesse & Distance
    if ($('speed-stable')) $('speed-stable').textContent = `${toKmH(sSpdFE).toFixed(2)}`;
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${sSpdFE.toFixed(3)} m/s`;
    if ($('speed-stable-kms')) $('speed-stable-kms').textContent = `${(sSpdFE / 1000).toExponential(3)} km/s`;
    if ($('speed-3d-inst')) $('speed-3d-inst').textContent = `${toKmH(spd3D_raw).toFixed(2)} km/h`; 
    if ($('speed-brute-ms')) $('speed-brute-ms').textContent = `${spd3D_raw.toFixed(3)} m/s`;
    if ($('speed-max')) $('speed-max').textContent = `${toKmH(maxSpd).toFixed(2)} km/h`;
    if ($('speed-avg-moving')) $('speed-avg-moving').textContent = timeMoving > 1 ? `${toKmH(distM / timeMoving).toFixed(2)} km/h` : '0.00 km/h';
    if ($('speed-avg-total')) $('speed-avg-total').textContent = elapsed_s > 1 ? `${toKmH(distM / elapsed_s).toFixed(2)} km/h` : '0.00 km/h';
    if ($('vertical-speed')) $('vertical-speed').textContent = `${spdV.toFixed(2)} m/s`;

    // Relativité & Fluides
    if ($('perc-speed-sound')) $('perc-speed-sound').textContent = `${(phys.mach * 100).toFixed(2)} %`;
    if ($('nombre-mach')) $('nombre-mach').textContent = phys.mach.toFixed(4);
    if ($('lorentz-factor')) $('lorentz-factor').textContent = phys.lorentz_factor.toFixed(5);
    if ($('time-dilation-v')) $('time-dilation-v').textContent = phys.time_dilation_v.toFixed(2) + ' ns/j';
    if ($('dynamic-pressure')) $('dynamic-pressure').textContent = phys.dynamic_pressure_q.toFixed(2) + ' Pa';
    if ($('force-trainee')) $('force-trainee').textContent = phys.drag_force.toFixed(2) + ' N';
    if ($('nombre-reynolds')) $('nombre-reynolds').textContent = phys.reynolds_number.toExponential(2);
    
    // Distance
    if ($('distance-total-3d')) $('distance-total-3d').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    if ($('distance-light-s')) $('distance-light-s').textContent = `${(distM / C_L).toExponential(2)} s`;
    if ($('distance-light-min')) $('distance-light-min').textContent = `${(distM / C_L / 60).toExponential(2)} min`;
    if ($('distance-light-h')) $('distance-light-h').textContent = `${(distM / C_L / 3600).toExponential(2)} h`;
    if ($('distance-light-j')) $('distance-light-j').textContent = `${(distM / C_L / 86400).toExponential(2)} j`;
    if ($('distance-ua-al')) $('distance-ua-al').textContent = `${(distM / 149597870700).toExponential(2)} UA | ${(distM / 9460730472580800).toExponential(2)} al`;
    
    // Position & Astro
    if ($('latitude-ekf')) $('latitude-ekf').textContent = `${cLat.toFixed(6)} °`;
    if ($('longitude-ekf')) $('longitude-ekf').textContent = `${cLon.toFixed(6)} °`;
    if ($('altitude-ekf-corrigee')) $('altitude-ekf-corrigee').textContent = kAlt_new !== null ? `${kAlt_new.toFixed(2)} m` : 'N/A';
    if ($('altitude-geopotentielle')) $('altitude-geopotentielle').textContent = phys.alt_geopotentielle.toFixed(2) + ' m';
    if ($('cap-direction')) $('cap-direction').textContent = headingRaw !== null ? `${headingRaw.toFixed(1)} °` : 'N/A';
    
    // Physique Avancée
    if ($('time-dilation-gravite')) $('time-dilation-gravite').textContent = phys.time_dilation_grav.toFixed(2) + ' ns/j';
    if ($('energie-relativiste')) $('energie-relativiste').textContent = phys.E.toExponential(3) + ' J';
    if ($('energie-masse-repos')) $('energie-masse-repos').textContent = phys.E_0.toExponential(3) + ' J';
    if ($('rayon-schwarzschild')) $('rayon-schwarzschild').textContent = phys.R_s.toExponential(2) + ' m';
    if ($('force-coriolis')) $('force-coriolis').textContent = coriolis_force.toExponential(2) + ' N';
    if ($('gravite-locale')) $('gravite-locale').textContent = local_g.toFixed(5) + ' m/s²';
    if ($('acceleration-long')) $('acceleration-long').textContent = accel_long.toFixed(3) + ' m/s²';
    if ($('quantite-mouvement')) $('quantite-mouvement').textContent = (currentMass * sSpdFE).toFixed(2) + ' kg·m/s';
    
    // IMU & Debug (Simulation des valeurs IMU)
    const time_ms = cTimePos;
    const real_accel_x = Math.sin(time_ms / 1000) * 0.1;
    const real_accel_y = Math.cos(time_ms / 1000) * 0.05;
    const real_accel_z = Math.sin(time_ms / 1000) * 0.1 + 0.1;
    if ($('acceleration-x')) $('acceleration-x').textContent = `${real_accel_x.toFixed(2)} m/s²`;
    if ($('acceleration-y')) $('acceleration-y').textContent = `${real_accel_y.toFixed(2)} m/s²`;
    if ($('acceleration-z')) $('acceleration-z').textContent = `${real_accel_z.toFixed(2)} m/s²`;
    
    // EKF Debug
    if ($('kalman-uncert')) $('kalman-uncert').textContent = `${kUncert.toFixed(3)} m²/s² (P)`;
    if ($('alt-uncertainty')) $('alt-uncertainty').textContent = `${Math.sqrt(kAltUncert).toFixed(3)} m (σ)`;
    if ($('bruit-mesure-vitesse-r')) $('bruit-mesure-vitesse-r').textContent = `${R_dyn.toFixed(3)} m² (R dyn)`;
    if ($('statut-gps-ekf')) $('statut-gps-ekf').textContent = modeStatus;
    if ($('forcer-precision-gps')) $('forcer-precision-gps').textContent = `${gpsAccuracyOverride.toFixed(6)} m`;
    if ($('gps-precision-acc')) $('gps-precision-acc').textContent = `${acc.toFixed(1)} m`;

    updateMap(cLat, cLon, accRaw);
}

// --- 8. BOUCLE DE MISE À JOUR LENTE (1 Hz) ---

function slowUpdateLoop() {
    const now = getCDate(lServH, lLocH);
    if (!now) return;

    const currentLat = lPos ? lPos.coords.latitude : 43.296; 
    const currentLon = lPos ? lPos.coords.longitude : 5.370;

    // Mise à jour Astro
    const astro = calculateAllAstro(currentLat, currentLon, now);
    
    if ($('heure-minecraft')) $('heure-minecraft').textContent = getMinecraftTime(now);
    if ($('date-locale-utc')) $('date-locale-utc').textContent = now.toLocaleDateString('fr-FR');
    if ($('date-astro')) $('date-astro').textContent = now.toLocaleDateString('fr-FR');
    if ($('heure-solaire-vraie')) $('heure-solaire-vraie').textContent = typeof astro.TST_h === 'number' ? formatTime(astro.TST_h) : astro.TST_h;
    if ($('heure-solaire-moyenne')) $('heure-solaire-moyenne').textContent = typeof astro.MST_h === 'number' ? formatTime(astro.MST_h) : astro.MST_h;
    if ($('temps-sideral-local-vrai')) $('temps-sideral-local-vrai').textContent = typeof astro.TSLV_h === 'number' ? formatTime(astro.TSLV_h) : astro.TSLV_h;
    if ($('equation-temps')) $('equation-temps').textContent = typeof astro.EOT_min === 'number' ? astro.EOT_min.toFixed(2) + ' min' : astro.EOT_min;
    if ($('lever-coucher-locale')) $('lever-coucher-locale').textContent = astro.sunRiseSet;
    if ($('phase-lune')) $('phase-lune').textContent = astro.moonPhase;
    if ($('lune-age')) $('lune-age').textContent = astro.moonAgeDays;
    if ($('lune-distance')) $('lune-distance').textContent = `${astro.moonDistance} km`;
    if ($('soleil-distance')) $('soleil-distance').textContent = `${astro.sunDistanceAU} UA`;
    
    const isNight = typeof astro.TST_h === 'number' ? (astro.TST_h > 18 || astro.TST_h < 6) : false;
    if ($('nuit-crepuscule')) $('nuit-crepuscule').textContent = isNight ? '🌙 Nuit' : '☀️ Jour';

    // Récupération Météo
    if (wID !== null && !emergencyStopActive) {
        fetchWeather(currentLat, currentLon).then(data => {
            if (data) {
                if ($('temp-air-reelle')) $('temp-air-reelle').textContent = `${data.tempC.toFixed(1)} °C`;
                if ($('pression-atm-corrigee')) $('pression-atm-corrigee').textContent = `${data.pressure_hPa.toFixed(0)} hPa`;
                if ($('humidite-relative')) $('humidite-relative').textContent = `${data.humidity_perc.toFixed(0)} %`;
                if ($('point-rosee')) $('point-rosee').textContent = `${data.dew_point.toFixed(1)} °C`;
                if ($('densite-air-corrigee')) $('densite-air-corrigee').textContent = `${currentMeteo.air_density.toFixed(3)} kg/m³`;
                if ($('vitesse-son-locale')) $('vitesse-son-locale').textContent = `${currentMeteo.sound_speed_ms.toFixed(1)} m/s`;
                
                if ($('vent-rafales')) $('vent-rafales').textContent = `${data.wind_speed} km/h (${data.wind_dir}) / ${data.wind_gust} km/h`;
                if ($('couverture-nuageuse')) $('couverture-nuageuse').textContent = `${data.cloud_cover} %`;
                if ($('plafond-nuageux')) $('plafond-nuageux').textContent = `≥ ${data.cloud_ceiling} m`;
                if ($('visibilite')) $('visibilite').textContent = `${data.visibility.toFixed(1)} km`;
            }
        });
    }

    // Mise à jour Horloge locale (NTP)
    if (now) {
        if ($('heure-locale-ntp') && !$('heure-locale-ntp').textContent.includes('SYNCHRO ÉCHOUÉE')) {
            $('heure-locale-ntp').textContent = now.toLocaleTimeString('fr-FR');
        }
    }
}

// --- 9. GESTION DES ÉVÉNEMENTS & INITIALISATION ---

document.addEventListener('DOMContentLoaded', () => {
    initMap(); 
    
    // Événements principaux
    if ($('toggle-gps-btn')) {
        $('toggle-gps-btn').addEventListener('click', () => { 
            if (!emergencyStopActive) wID === null ? startGPS() : stopGPS(); 
        });
    }

    if ($('emergency-stop-btn')) {
        $('emergency-stop-btn').addEventListener('click', () => { 
            emergencyStopActive = !emergencyStopActive;
            if (emergencyStopActive) {
                stopGPS();
                $('emergency-stop-btn').textContent = '▶️ REPRENDRE SYSTÈME';
                $('emergency-stop-btn').classList.add('active');
            } else {
                $('emergency-stop-btn').textContent = '🛑 Arrêt d\'urgence: INACTIF 🟢';
                $('emergency-stop-btn').classList.remove('active');
                startGPS();
            }
        });
        $('emergency-stop-btn').textContent = '🛑 Arrêt d\'urgence: INACTIF 🟢';
    }
    
    // Événements de réinitialisation
    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => { if (!emergencyStopActive) distM = 0; });
    if ($('reset-vmax-btn')) $('reset-vmax-btn').addEventListener('click', () => { if (!emergencyStopActive) maxSpd = 0; });
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => {
        if (!emergencyStopActive && confirm("Réinitialiser toutes les données de session ?")) { 
            distM = 0; maxSpd = 0; kSpd = 0; kUncert = 1000; timeMoving = 0; pathCoords = []; 
        } 
    });

    // Événements d'entrée utilisateur
    if ($('mass-input')) {
        $('mass-input').addEventListener('change', (e) => {
            currentMass = parseFloat(e.target.value) || 70.0;
            if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        });
    }
    if ($('gps-accuracy-override')) {
        $('gps-accuracy-override').addEventListener('change', (e) => {
            gpsAccuracyOverride = parseFloat(e.target.value) || 0.0;
        });
    }
    if ($('celestial-body-select')) {
         $('celestial-body-select').addEventListener('change', (e) => {
            currentCelestialBody = e.target.value;
            switch(currentCelestialBody) {
                case 'LUNE': G_ACC = 1.625; R_ALT_CENTER_REF = 1737400; break;
                case 'MARS': G_ACC = 3.721; R_ALT_CENTER_REF = 3389500; break;
                default: G_ACC = 9.8067; R_ALT_CENTER_REF = R_E_BASE; break;
            }
            if ($('corps-celeste-gravity')) $('corps-celeste-gravity').textContent = `${G_ACC.toFixed(4)} m/s²`;
        });
    }

    // Démarrage initial (Synchro temps)
    syncH().then(newTimes => {
        lServH = newTimes.lServH;
        lLocH = newTimes.lLocH;
        if ($('heure-locale-ntp')) $('heure-locale-ntp').textContent = getCDate(lServH, lLocH).toLocaleTimeString('fr-FR');
    }).catch(() => {
        if ($('heure-locale-ntp')) $('heure-locale-ntp').textContent = 'SYNCHRO ÉCHOUÉE';
    });

    // Démarrage de la boucle lente
    setInterval(slowUpdateLoop, DOM_SLOW_UPDATE_MS);
});
