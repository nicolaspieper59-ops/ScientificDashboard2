// =================================================================
// BLOC 1/4 : gnss-constants.js
// Constantes de base, outils mathématiques et variables d'état global.
// =================================================================

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
const D2R = Math.PI / 180;      // Degrés vers Radians
const R2D = 180 / Math.PI;      // Radians vers Degrés
const C_L = 299792458;          // Vitesse de la lumière (m/s)
const G_EARTH = 9.8067;         // Gravité standard (m/s²)
const R_E_BASE = 6371000;       // Rayon terrestre moyen (m)
const KMH_MS = 3.6;             // Conversion m/s vers km/h
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const R_AIR = 287.058;          // Constante spécifique de l'air sec (J/kg·K)
const GAMMA_AIR = 1.4;          // Indice adiabatique de l'air (pour Vitesse du Son)
const STANDARD_P = 101325;      // Pression standard au niveau de la mer (Pa)
const STANDARD_T = 288.15;      // Température standard (K)

// --- VARIABLES D'ÉTAT DU SYSTÈME (Global State) ---
let isGPSActive = true;
let isNightMode = false;
let isXRayMode = false;
let objectMass = 70.0;
let forceGPSAccuracy = 0.0;

// EKF State Variables (Simplifié)
let kf_state = {
    pos: [0, 0, 0],   // Lat, Lon, Alt (m)
    vel: [0, 0, 0],   // N, E, U (m/s)
    P: [             // Matrice de Covariance (Incertitude)
        [100, 0, 0],
        [0, 100, 0],
        [0, 0, 100]
    ],
    Q_factor: 1.0,  // Facteur de bruit de processus (environnement)
    R_factor: 1.0   // Facteur de bruit de mesure
};

// Vitesse/Distance Metrics
let maxSpeed_kmh = 0.0;
let totalDistance_m = 0.0;
let movingTime_s = 0.0;
let sessionStartTime = Date.now();
let totalSpeedSum_kmh = 0.0;
let totalSamples = 0;

// Météo Variables (Utilisation des données fournies)
let currentMeteo = {
    tempC: 18.0,
    pressure_hPa: 1019.5,
    humidity_perc: 66,
    dewPointC: 13.0,
    wind_kmh: 24,
    density_air: 1.200, // Calculée
    sound_speed_ms: 341.8 // Calculée
};

// --- TIMING ET MISES À JOUR ---
const DOM_FAST_UPDATE_MS = 100; // 10 Hz pour Vitesse, Accel, Heure
const DOM_SLOW_UPDATE_MS = 1000; // 1 Hz pour Astro, Météo, EKF Debug

// --- FONCTIONS UTILITAIRES DE BASE ---
const $ = id => document.getElementById(id); // Raccourci DOM
const toKmH = ms => ms * KMH_MS;
const toMS = kmh => kmh / KMH_MS;
const toKelvin = c => c + 273.15;
// =================================================================
// BLOC 2/4 : ekf-processing.js
// Filtre de Kalman Étendu (EKF), Calculs de Vitesse, Distance et Dynamique des Fluides.
// =================================================================

// Simule la réception des données GPS/IMU (remplacer par les hooks des capteurs)
function getSensorData() {
    // NOTE: Simuler les données brutes pour la structure.
    return {
        lat: 43.2965, // Exemple Marseille
        lon: 5.3698,
        alt_gps: 10.0 + Math.random(), // Altitude GPS (moins fiable)
        alt_baro: 5.0 + Math.random() * 2, // Altitude Baro (plus fiable)
        speed_raw_ms: Math.random() * 5, // Vitesse GPS brute
        accel_x: 0.1, // Données IMU
        accel_y: 0.0,
        accel_z: 0.1,
        accuracy_m: forceGPSAccuracy > 0 ? forceGPSAccuracy : Math.random() * 10
    };
}

// ----------------------------------------------------------------
// EKF : PRÉDICTION ET MISE À JOUR (Logique Simplifiée)
// ----------------------------------------------------------------
function runEKF(data, deltaTime_s) {
    // 1. Prédiction (Utilisation de l'IMU pour estimer l'état futur)
    // Etape non montrée pour simplifier, mais ici on utiliserait accel_x/y/z pour estimer kf_state.pos/vel

    // 2. Mise à jour (Correction de l'état avec les données GPS/Baro)
    const alt_corrected = correctAltitudeBaro(data.alt_baro, currentMeteo.pressure_hPa);
    
    // --- Mise à jour de la Vitesse (EKF Fusion) ---
    // La vitesse filtrée kSpd est la longueur du vecteur kf_state.vel
    const kSpd_ms = Math.sqrt(kf_state.vel[0]**2 + kf_state.vel[1]**2 + kf_state.vel[2]**2);
    
    // Si l'accélérateur est faible (ZUPT logic) et le GPS est mauvais: la vitesse est forcée près de zéro, mais l'accel reste dans la matrice.
    if (kSpd_ms < 0.1 && data.accuracy_m > 5) {
        // Logique ZUPT: Si presque à l'arrêt, forcer la vitesse près de 0
        kf_state.vel = [0, 0, 0];
        kf_state.P[0][0] = kf_state.P[1][1] = kf_state.P[2][2] = 0.5; // Réduire l'incertitude
        $('gps-status-dr').textContent = 'EKF ZUPT Actif';
    } else {
        // Mise à jour classique avec données GPS (Logique de correction K)
        // Simplifié: fusionner la vitesse brute avec l'estimation précédente
        const fusion_factor = 1.0 / (data.accuracy_m * kf_state.R_factor);
        kf_state.vel[0] = kf_state.vel[0] * (1 - fusion_factor) + toMS(data.speed_raw_ms) * fusion_factor;
        $('gps-status-dr').textContent = 'Fusion GPS/IMU';
    }
    
    // Mise à jour de la Position EKF
    kf_state.pos[0] = data.lat;
    kf_state.pos[1] = data.lon;
    kf_state.pos[2] = alt_corrected;
    
    return {
        kSpd_ms: kSpd_ms,
        alt_corrected: alt_corrected
    };
}

// ----------------------------------------------------------------
// CALCULS DE DISTANCE ET DE VITESSE
// ----------------------------------------------------------------
function updateMetrics(kSpd_ms, deltaTime_s) {
    const kSpd_kmh = toKmH(kSpd_ms);
    
    // 1. Vitesse Max et Moyennes
    maxSpeed_kmh = Math.max(maxSpeed_kmh, kSpd_kmh);

    if (kSpd_kmh > 0.05) {
        movingTime_s += deltaTime_s;
    }
    
    // Vitesse Moyenne Totale (Distance Totale / Temps Écoulé Session)
    const sessionTime_s = (Date.now() - sessionStartTime) / 1000;
    
    // 2. Distance Totale 3D (Intégration de la vitesse 3D)
    totalDistance_m += kSpd_ms * deltaTime_s;
    
    // 3. Distance Max Visible (Horizon)
    const h = kf_state.pos[2]; // Altitude EKF
    const d_horizon_km = 1.14 * Math.sqrt(h); // Formule simplifiée (km pour h en m)
    
    // 4. Rapport de Distance Centre-Terre (remplace le mode Nether)
    const ratio_center = (R_E_BASE + h) / R_E_BASE;
    
    return {
        kSpd_kmh: kSpd_kmh,
        d_horizon_km: d_horizon_km,
        ratio_center: ratio_center,
        sessionTime_s: sessionTime_s
    };
}

// ----------------------------------------------------------------
// DYNAMIQUE DES FLUIDES ET RELATIVITÉ
// ----------------------------------------------------------------
function calculateAdvancedPhysics(kSpd_ms) {
    const V = kSpd_ms;
    const C_S = currentMeteo.sound_speed_ms; // Vitesse du Son corrigée par Météo
    const Rho = currentMeteo.density_air;    // Densité de l'air corrigée
    const Mass = objectMass;
    
    // 1. Vitesse du Son et Mach
    const mach = V / C_S;
    const perc_sound = (V / C_S) * 100;
    
    // 2. Relativité
    const lorentz_factor = 1.0 / Math.sqrt(1 - (V * V / (C_L * C_L)));
    const perc_light = (V / C_L) * 100;
    const energy_e = Mass * C_L * C_L * lorentz_factor; // E = gamma * E0
    const energy_e0 = Mass * C_L * C_L;
    
    // 3. Fluides et Forces
    const dynamic_pressure_q = 0.5 * Rho * V * V; // 0.5 * rho * V^2
    const drag_force = dynamic_pressure_q * 1.0 * 1.0; // Simplifié: q * Cd * A (Cd=1.0, A=1.0m2)
    const momentum = Mass * V;
    
    // 4. Gravité et Dilation
    const gravity_local = G_EARTH * Math.pow((R_E_BASE / (R_E_BASE + kf_state.pos[2])), 2);
    const time_dilation_v = (lorentz_factor - 1) * 86400 * 1e9; // ns/jour
    const time_dilation_g = (gravity_local * kf_state.pos[2]) / (C_L * C_L) * 86400 * 1e9; // ns/jour
    
    return {
        mach,
        perc_sound,
        lorentz_factor,
        perc_light,
        dynamic_pressure_q,
        drag_force,
        momentum,
        energy_e,
        energy_e0,
        gravity_local,
        time_dilation_v,
        time_dilation_g
    };
}

// ----------------------------------------------------------------
// MÉTÉO ET CORRECTIONS
// ----------------------------------------------------------------
// NOTE: La logique de correction barométrique est ici, utilisant la pression d'une station de référence (P_REF)
function correctAltitudeBaro(altBaro_m, pressure_hPa) {
    // Pression de référence de la station météo à l'altitude h=0 (niveau de la mer)
    const P_REF_SEA_LEVEL_Pa = 1019.5 * 100; // 1019.5 hPa donné en entrée
    const P_MEASURED_Pa = pressure_hPa * 100;

    // Formule barométrique (Altitude par rapport au niveau de la mer)
    // h = (T0 / L) * ((P0/P)^(L*R/g) - 1) où T0=288.15, L=0.0065, R=287.058, g=9.8067
    const h_m = ((STANDARD_T / 0.0065) * (1 - Math.pow(P_MEASURED_Pa / P_REF_SEA_LEVEL_Pa, 0.19028)));
    
    return h_m.toFixed(2);
            }
// =================================================================
// BLOC 3/4 : spacetime-astro.js
// Calculs Astronomiques (Éphémérides Soleil/Lune, Systèmes de Temps).
// NOTE: Utilise des fonctions stub, la mise en œuvre complète nécessiterait une librairie d'éphémérides.
// =================================================================

function calculateAllAstro(lat, lon, now) {
    // Coordonnées J2000 (Jours Juliennes)
    const julianDay = now.getTime() / 86400000 + 2440587.5;
    
    // --- 1. Calculs de Temps ---
    
    // LST: Temps Sidéral Local (fonction de l'heure et de la longitude)
    const TSLV_h = (julianDay % 1) * 24 + (lon / 15); // Très simplifié
    
    // Équation du Temps (minutes)
    const EOT_min = 15.93 + (Math.sin(julianDay * D2R * 0.017) * 5); // Simplifié
    
    // Temps Solaire Vrai et Moyen
    const TST_h = (now.getUTCHours() + now.getUTCMinutes() / 60) + (EOT_min / 60) + (lon / 15);
    const MST_h = TST_h - (EOT_min / 60);
    
    // Heure Lunaire (dépend du transit lunaire)
    const lunarTime_h = (TSLV_h + 12) % 24; // Très simplifié
    
    // --- 2. Éphémérides (Soleil/Lune) ---
    
    // Placeholder pour les fonctions d'éphémérides
    const getSunEphemeris = (jd, lt, ln) => ({ rise: '07:30', set: '17:45', dist: 0.98 });
    const getMoonEphemeris = (jd, lt, ln) => ({ phase: 0.75, age: 10.5, dist: 384000 });
    
    const sunData = getSunEphemeris(julianDay, lat, lon);
    const moonData = getMoonEphemeris(julianDay, lat, lon);
    
    // Conversion de phase
    const phaseName = moonData.phase > 0.5 ? 'Gibbeuse Montante' : 'Croissant';
    const phasePerc = (moonData.phase * 100).toFixed(0);
    
    return {
        // Temps
        solarMeanDate: now.toLocaleDateString('fr-FR'), // À calculer correctement
        solarTrueDate: now.toLocaleDateString('fr-FR'), // À calculer correctement
        timeSolarTrue: new Date(now.setHours(MST_h)).toLocaleTimeString('fr-FR'),
        timeSolarMean: new Date(now.setHours(TST_h)).toLocaleTimeString('fr-FR'),
        timeLunar: lunarTime_h.toFixed(2) + ' h',
        solarNoonUTC: (12 - (lon / 15) * 60 - EOT_min).toFixed(2) + ' min UTC', // Simplifié
        eot: EOT_min.toFixed(2) + ' min',
        tslv: TSLV_h.toFixed(2) + ' h',
        
        // Soleil
        sunRiseLocal: sunData.rise,
        sunSetLocal: sunData.set,
        sunDistanceAU: sunData.dist.toFixed(3),
        
        // Lune
        moonPhaseName: `${phaseName} (${phasePerc} %)`,
        moonAgeDays: moonData.age.toFixed(1),
        moonDistanceKM: moonData.dist.toLocaleString(),
        // Les levers/couchers en HSL/HSM doivent être calculés à partir de la trigonométrie
        moonRiseLunar: 'N/A', 
        moonSetLunar: 'N/A',
        
        // Rendu Ciel
        isNight: TST_h > 18 || TST_h < 6 // Heure solaire vraie
    };
}
// =================================================================
// BLOC 4/4 : ui-renderer.js
// Rendu de l'interface utilisateur, boucles de mise à jour et gestion des événements.
// =================================================================

// --- 1. INITIALISATION ---
document.addEventListener('DOMContentLoaded', () => {
    // Initialiser la carte Leaflet (Globe Interactif)
    // NOTE: Remplacer par un rendu de globe 3D (ex: Cesium.js ou Three.js) pour l'effet Aurora/Météo
    const map = L.map('gnss-map').setView([43.2965, 5.3698], 10);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; OpenStreetMap contributors'
    }).addTo(map);

    // Initialisation des boucles de mise à jour
    setupEventListeners();
    setInterval(slowUpdateLoop, DOM_SLOW_UPDATE_MS);
    setInterval(fastUpdateLoop, DOM_FAST_UPDATE_MS);
});

// --- 2. GESTION DES ÉVÉNEMENTS (Boutons) ---
function setupEventListeners() {
    // Boutons de contrôle
    $('toggle-gps').addEventListener('click', toggleGPS);
    $('toggle-night-mode').addEventListener('click', toggleNightMode);
    $('toggle-xray').addEventListener('click', toggleXRayMode);
    $('reset-all').addEventListener('click', resetAllMetrics);
    
    // Saisie utilisateur
    $('force-gps-acc').addEventListener('input', (e) => {
        forceGPSAccuracy = parseFloat(e.target.value) || 0;
        $('force-gps-output').textContent = forceGPSAccuracy.toFixed(6) + ' m';
    });
    
    // Placeholder pour les compteurs (simule la lecture)
    $('sound-meter').textContent = `${(Math.random() * 50 + 40).toFixed(1)} / ${(Math.random() * 50 + 40).toFixed(1)} dB`;
    $('light-meter').textContent = `${(Math.random() * 1000).toFixed(0)} / ${(Math.random() * 100).toFixed(0)} Lux`;
    $('magnetic-meter').textContent = `${(Math.random() * 80 + 20).toFixed(1)} / ${(Math.random() * 50).toFixed(1)} µT`;
}

function toggleGPS() {
    isGPSActive = !isGPSActive;
    $('toggle-gps').innerHTML = isGPSActive ? '<i class="fas fa-pause"></i> PAUSE GPS' : '<i class="fas fa-play"></i> MARCHE GPS';
    $('toggle-gps').classList.toggle('status-red', !isGPSActive);
}

function toggleNightMode() {
    isNightMode = !isNightMode;
    document.body.classList.toggle('night-mode', isNightMode);
}

function toggleXRayMode() {
    isXRayMode = !isXRayMode;
    $('biome-half').style.opacity = isXRayMode ? 0.0 : 1.0;
    $('toggle-xray').textContent = isXRayMode ? 'Rayons X ON' : 'Rayons X OFF';
}

function resetAllMetrics() {
    if (confirm('Êtes-vous sûr de vouloir TOUT RÉINITIALISER ?')) {
        maxSpeed_kmh = 0.0;
        totalDistance_m = 0.0;
        movingTime_s = 0.0;
        sessionStartTime = Date.now();
        kf_state = { pos: [0, 0, 0], vel: [0, 0, 0], P: kf_state.P, Q_factor: 1.0, R_factor: 1.0 };
        // Mettre à jour l'affichage immédiatement
        $('speed-max').textContent = '0.0 km/h';
        $('distance-total').textContent = '0.000 km | 0.00 m';
        $('time-session').textContent = '0.00 s';
    }
}

// --- 3. BOUCLES DE MISE À JOUR ---

// Mise à jour rapide (10 Hz)
function fastUpdateLoop() {
    const data = getSensorData();
    const deltaTime_s = DOM_FAST_UPDATE_MS / 1000;

    // EKF & Physique
    const { kSpd_ms, alt_corrected } = runEKF(data, deltaTime_s);
    const { kSpd_kmh, d_horizon_km, ratio_center, sessionTime_s } = updateMetrics(kSpd_ms, deltaTime_s);
    const phys = calculateAdvancedPhysics(kSpd_ms);
    
    // --- Rendu Vitesse & Distance ---
    $('speed-stable-kmh').textContent = kSpd_kmh.toFixed(2) + ' km/h';
    $('speed-stable-ms').textContent = kSpd_ms.toFixed(2) + ' m/s';
    $('speed-stable-kms').textContent = (kSpd_ms / 1000).toFixed(4) + ' km/s';
    $('speed-max').textContent = maxSpeed_kmh.toFixed(1) + ' km/h';
    $('distance-total').textContent = `${(totalDistance_m / 1000).toFixed(3)} km | ${totalDistance_m.toFixed(2)} m`;
    $('distance-horizon').textContent = d_horizon_km.toFixed(2) + ' km';
    
    // Vitesse Moyenne Totale
    const avgTotalSpeed = totalDistance_m / sessionTime_s * KMH_MS;
    $('speed-avg-total').textContent = avgTotalSpeed.toFixed(1) + ' km/h';

    // Temps
    $('time-session').textContent = sessionTime_s.toFixed(2) + ' s';
    $('time-moving').textContent = movingTime_s.toFixed(2) + ' s';
    
    // --- Rendu Physique Avancée ---
    $('perc-sound').textContent = phys.perc_sound.toFixed(2) + ' %';
    $('mach-number').textContent = phys.mach.toFixed(4);
    $('perc-light').textContent = phys.perc_light.toExponential(2) + ' %';
    $('lorentz-factor').textContent = phys.lorentz_factor.toFixed(4);
    
    $('dynamic-pressure').textContent = phys.dynamic_pressure_q.toFixed(2) + ' Pa';
    $('drag-force').textContent = phys.drag_force.toFixed(2) + ' N';
    $('momentum').textContent = phys.momentum.toFixed(2) + ' kg·m/s';
    
    // --- Rendu IMU/Position ---
    $('accel-x').textContent = data.accel_x.toFixed(2) + ' m/s²';
    $('accel-y').textContent = data.accel_y.toFixed(2) + ' m/s²';
    $('accel-z').textContent = data.accel_z.toFixed(2) + ' m/s²';
    $('altitude-ekf').textContent = alt_corrected + ' m';
    $('latitude').textContent = data.lat.toFixed(5);
    $('longitude').textContent = data.lon.toFixed(5);
    $('distance-ratio').textContent = `${ratio_center.toFixed(3)} (Surface)`;
}

// Mise à jour lente (1 Hz)
function slowUpdateLoop() {
    const now = new Date();
    const lat = kf_state.pos[0];
    const lon = kf_state.pos[1];
    
    // --- Rendu Système ---
    $('local-time').textContent = now.toLocaleTimeString('fr-FR');
    $('date-display').textContent = now.toLocaleDateString('fr-FR');
    
    // --- Rendu Astronomie (Rendu du Bloc 3) ---
    const astro = calculateAllAstro(lat, lon, now);
    
    $('date-solar-mean').textContent = astro.solarMeanDate;
    $('date-solar-true').textContent = astro.solarTrueDate;
    $('time-solar-true').textContent = astro.timeSolarTrue;
    $('time-solar-mean').textContent = astro.timeSolarMean;
    $('time-lunar').textContent = astro.timeLunar;
    $('eot').textContent = astro.eot;
    $('lunar-time').textContent = astro.tslv;
    
    $('sun-rise-set-local').textContent = `${astro.sunRiseLocal} / ${astro.sunSetLocal}`;
    $('moon-rise-set-lunar').textContent = `${astro.moonRiseLunar} / ${astro.moonSetLunar}`;
    $('moon-phase').textContent = astro.moonPhaseName;
    $('moon-age').textContent = astro.moonAgeDays;
    $('moon-distance').textContent = astro.moonDistanceKM;
    $('sun-distance').textContent = astro.sunDistanceAU;
    
    // --- Rendu Animation Ciel Minecraft ---
    if (astro.isNight) {
        $('day-night-status').textContent = '🌙 Nuit';
        $('sky-animation-container').style.background = 'linear-gradient(to top, #001f3f, #004488)';
    } else {
        $('day-night-status').textContent = '☀️ Jour';
        $('sky-animation-container').style.background = 'linear-gradient(to top, #87CEEB, #ffffff)';
    }
    
    // NOTE: L'implémentation complète de la position Soleil/Lune sur le disque est complexe et dépend
    // des calculs d'azimut/altitude (non simulés) pour les transformer en coordonnées X/Y.
    const sunPos = { x: 50 + (astro.isNight ? 70 : -70), y: 75 + (astro.isNight ? 50 : -50) };
    $('sun').style.transform = `translate(-50%, -50%) translate(${sunPos.x}px, ${sunPos.y}px)`;
      }
