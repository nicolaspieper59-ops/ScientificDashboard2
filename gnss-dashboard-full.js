// =================================================================
// BLOC 1/4 : gnss-constants.js
// Constantes de base, variables d'état global et paramètres EKF.
// =================================================================

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const R_E_BASE = 6371000;   // Rayon terrestre moyen (m)
const KMH_MS = 3.6;         // Conversion m/s vers km/h
const C_S = 343;            // Vitesse du son (m/s) (Sera corrigée par la météo)
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const R_AIR = 287.058;      // Constante spécifique de l'air sec (J/kg·K)
const NETHER_RATIO = 8;     // Ratio Monde Réel/Nether (Minecraft)

// --- PARAMÈTRES DU FILTRE DE KALMAN (VITESSE/ALTITUDE) ---
const Q_NOISE = 0.1;        // Bruit de processus (Vitesse)
const Q_ALT_NOISE = 0.05;   // Bruit de processus (Altitude)
const R_MIN = 0.01;         // Bruit de mesure minimum
const R_MAX = 500.0;        // Bruit de mesure maximum
const MAX_ACC = 200;        // Précision max (m) avant "Estimation Seule"
const MIN_SPD = 0.05;       // Vitesse minimale pour le mouvement
const MIN_DT = 0.05;        // Minimum Delta Time (s)
const ZUPT_RAW_THRESHOLD = 0.5; // Vitesse brute pour ZUPT (m/s)
const ZUPT_ACCEL_THRESHOLD = 0.8; // Accélération pour ZUPT (m/s²)
const MAX_PLAUSIBLE_ACCEL = 50; // Accélération max anti-spike (m/s²)
const ALT_TH = 500;         // Seuil d'altitude pour le statut "Souterrain" (m)

// --- ÉTAT GLOBAL DU SYSTÈME ET EKF ---
let wID = null;             // ID du watchPosition
let domID = null;           // ID de l'intervalle lent
let emergencyStopActive = false;

// EKF État
let kSpd = 0.0;             // Vitesse filtrée (m/s)
let kUncert = 1000.0;       // Incertitude Vitesse (P)
let kAlt = null;            // Altitude filtrée (m)
let kAltUncert = 10.0;      // Incertitude Altitude (P)
let lastFSpeed = 0.0;       // Dernière vitesse filtrée

// Données de Position de l'itération précédente (lPos)
let lPos = null;
let sTime = null;           // Temps de début de session
let distM = 0;              // Distance totale parcourue (m)
let maxSpd = 0;             // Vitesse Max (m/s)
let timeMoving = 0;         // Temps de mouvement (s)

// Données NTP et Météo
let lServH = 0;             // Dernière heure serveur NTP
let lLocH = 0;              // Dernière heure locale
let lastP_hPa = 1013.25;    // Pression HPA (météo)
let lastT_K = 291.15;       // Température Kelvin (météo)
let lastH_perc = 0.7;       // Humidité (météo)

// Paramètres Utilisateur/Environnement
let currentMass = 70.0;
let currentCelestialBody = 'TERRE';
let netherMode = false;
let gpsAccuracyOverride = 0.0;
let G_ACC = 9.8067;         // Gravité de base
let R_ALT_CENTER_REF = R_E_BASE; // Rayon du corps céleste
let rotationRadius = 100;   // Pour corps céleste ROTATING
let angularVelocity = 0;    // Pour corps céleste ROTATING

// Facteurs Environnement (pour la modulation de R)
const ENVIRONMENT_FACTORS = { 
    NORMAL: { DISPLAY: 'Normal', R_MULT: 1.0 },
    FAIBLE: { DISPLAY: 'Faible GPS', R_MULT: 5.0 },
    GROTTE: { DISPLAY: 'Grotte/IMU', R_MULT: 0.1 }
};
let selectedEnvironment = 'NORMAL';

// --- Fonctions de base ---
const $ = id => document.getElementById(id);
// =================================================================
// BLOC 2/4 : ekf-processing.js
// Implémentation du Filtre de Kalman 1D (Vitesse et Altitude), Anti-Spike et Physique Avancée.
// =================================================================

// --- FONCTIONS FILTRES DE KALMAN 1D (Vitesse) ---
function kFilter(kState, kUncert, Z_input, dt, R_input, accel_sensor_input) {
    
    // 1. Prédiction
    const F = 1; // Transition Matrix
    const u = accel_sensor_input * dt; // Commande (accélération, ici souvent 0)
    kState = F * kState + u;
    kUncert += Q_NOISE * dt; // Q est le bruit de processus
    
    // 2. Mise à jour (Correction)
    const K = kUncert / (kUncert + R_input); // Gain de Kalman
    kState = kState + K * (Z_input - kState); // Nouvelle estimation
    kUncert = kUncert * (1 - K); // Nouvelle incertitude
    
    return { kSpd: kState, kUncert: kUncert };
}

// --- FONCTIONS FILTRES DE KALMAN 1D (Altitude) ---
function kFilterAltitude(kAlt, kAltUncert, altRaw, R_alt, dt) {
    
    // Simplification : le filtre d'altitude ne modélise pas la vitesse verticale
    // 1. Prédiction (État constant)
    kAltUncert += Q_ALT_NOISE * dt;
    
    // 2. Mise à jour (Correction)
    const K_alt = kAltUncert / (kAltUncert + R_alt);
    kAlt = kAlt + K_alt * (altRaw - kAlt);
    kAltUncert = kAltUncert * (1 - K_alt);
    
    return { kAlt: kAlt, kAltUncert: kAltUncert };
}

// --- FONCTIONS DE CALCUL PHYSIQUE/GEOGRAPHIQUE ---
function dist(lat1, lon1, lat2, lon2, radius) {
    const r = radius; 
    const dLat = (lat2 - lat1) * D2R;
    const dLon = (lon2 - lon1) * D2R;
    const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
              Math.cos(lat1 * D2R) * Math.cos(lat2 * D2R) * Math.sin(dLon / 2) * Math.sin(dLon / 2);
    return r * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
}

function calculateMRF(kAlt, netherMode) {
    if (netherMode) return 1 / NETHER_RATIO;
    // R_ALT_CENTER_REF est le rayon du corps céleste
    return (R_ALT_CENTER_REF + kAlt) / R_ALT_CENTER_REF;
}

function getGravityLocal(alt, body, rotRadius, angularVel) {
    if (body === 'TERRE') {
        return G_ACC * Math.pow((R_E_BASE / (R_E_BASE + alt)), 2);
    }
    if (body === 'ROTATING') {
        // Gravité effective = Gravité de base - Force centrifuge
        return G_ACC - rotRadius * angularVel * angularVel;
    }
    return G_ACC; // Gravité de base pour Lune/Mars/etc.
}

function updateCelestialBody(body, kAlt) {
    // Fonction d'initialisation du corps céleste (à implémenter)
    switch (body) {
        case 'LUNE': G_ACC = 1.625; R_ALT_CENTER_REF = 1737400; break;
        case 'MARS': G_ACC = 3.721; R_ALT_CENTER_REF = 3389500; break;
        case 'TERRE': default: G_ACC = 9.8067; R_ALT_CENTER_REF = R_E_BASE; break;
    }
    return { G_ACC: G_ACC, R_ALT_CENTER_REF: R_ALT_CENTER_REF };
}


// ----------------------------------------------------------------
// FONCTION PRINCIPALE DE TRAITEMENT GPS (processPosition)
// ----------------------------------------------------------------
function processGPSData(pos) {
    
    const cLat = pos.coords.latitude;
    const cLon = pos.coords.longitude;
    const altRaw = pos.coords.altitude;
    const accRaw = pos.coords.accuracy;
    const headingRaw = pos.coords.heading;
    const cTimePos = pos.timestamp;
    
    let modeStatus = 'EKF GPS Stable';
    
    // Correction de l'Accuracy (Override Utilisateur)
    let acc = (gpsAccuracyOverride > 0) ? gpsAccuracyOverride : accRaw;
    let R_dyn = ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT * acc * acc; 
    R_dyn = Math.min(R_dyn, R_MAX); 
    
    // --- 1. INITIALISATION OU CALCUL DT ---
    let dt;
    if (lPos && sTime) {
        dt = (cTimePos - lPos.timestamp) / 1000;
    } else {
        // Initialisation à la première exécution
        lPos = pos; 
        lPos.speedMS_3D = 0;
        lPos.kAlt_old = altRaw;
        kAlt = altRaw; // Initialisation du filtre altitude
        sTime = cTimePos;
        // updateMap(cLat, cLon, accRaw); // L'update map est dans la boucle lente pour alléger
        return; 
    }
    
    if (dt < MIN_DT || dt > 10) { 
        lPos = pos; // Réinitialise si dt est invalide
        return; 
    }

    // --- 2. FILTRAGE EKF ALTITUDE ---
    const { kAlt: kAlt_new, kAltUncert: kAltUncert_new } = kFilterAltitude(
        kAlt, kAltUncert, altRaw, pos.coords.altitudeAccuracy || R_MIN, dt
    );
    kAlt = kAlt_new;
    kAltUncert = kAltUncert_new;
    
    // --- 3. CALCUL VITESSE BRUTE 3D & ANTI-SPIKE ---
    const dist2D = dist(lPos.coords.latitude, lPos.coords.longitude, cLat, cLon, R_ALT_CENTER_REF);
    const dist3D = Math.sqrt(dist2D ** 2 + (kAlt_new - (lPos.kAlt_old || kAlt_new)) ** 2);
    let spd3D_raw = dist3D / dt; 
    const spdV = (kAlt_new - (lPos.kAlt_old || kAlt_new)) / dt; 

    let accel_long_provisional = 0;
    if (lPos && lPos.speedMS_3D !== undefined && dt > 0.05) { 
        accel_long_provisional = (spd3D_raw - lPos.speedMS_3D) / dt;
    }

    if (lPos && lPos.speedMS_3D !== undefined) {
        const lastRawSpd = lPos.speedMS_3D;
        const accelSpike = Math.abs(spd3D_raw - lastRawSpd) / dt;
        
        if (accelSpike > MAX_PLAUSIBLE_ACCEL) {
            console.warn(`Spike détecté: ${accelSpike.toFixed(2)} m/s². Correction appliquée.`);
            const maxPlausibleChange = MAX_PLAUSIBLE_ACCEL * dt;
            spd3D_raw = (spd3D_raw > lastRawSpd) ? (lastRawSpd + maxPlausibleChange) : (lastRawSpd - maxPlausibleChange);
        }
    }
    
    // --- 4. LOGIQUE ZUPT (Zero Velocity Update) ---
    let spd_kalman_input = spd3D_raw;
    let R_kalman_input = R_dyn;
    
    const isPlausiblyStopped = (
        spd3D_raw < ZUPT_RAW_THRESHOLD && 
        Math.abs(accel_long_provisional) < ZUPT_ACCEL_THRESHOLD &&
        R_dyn < R_MAX 
    ); 
    
    if (isPlausiblyStopped) { 
        spd_kalman_input = 0.0;     // Forcer la mesure à 0 m/s
        R_kalman_input = R_MIN;     // Confiance maximale dans la mesure ZUPT
        modeStatus = '✅ ZUPT (Vélocité Nulle Forcée)';
    }

    // --- 5. FILTRE EKF VITESSE ---
    let accel_sensor_input = 0; // (À remplacer par l'IMU réel)
    
    const { kSpd: fSpd, kUncert: kUncert_new } = kFilter(kSpd, kUncert, spd_kalman_input, dt, R_kalman_input, accel_sensor_input);
    kSpd = fSpd;
    kUncert = kUncert_new;
    
    const sSpdFE = fSpd < MIN_SPD ? 0 : fSpd; // Vitesse finale filtrée et seuillée
    
    // --- 6. CALCULS PHYSIQUES & DISTANCE ---
    let accel_long = 0;
    if (dt > 0.05) { 
        accel_long = (sSpdFE - lastFSpeed) / dt;
    }
    lastFSpeed = sSpdFE;

    const R_FACTOR_RATIO = calculateMRF(kAlt_new, netherMode); 
    distM += sSpdFE * dt * R_FACTOR_RATIO; 
    
    if (sSpdFE > MIN_SPD) { timeMoving += dt; }
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    
    const local_g = getGravityLocal(kAlt_new, currentCelestialBody, rotationRadius, angularVelocity); 
    const kineticEnergy = 0.5 * currentMass * sSpdFE ** 2;
    const mechanicalPower = currentMass * sSpdFE * accel_long;
    const coriolis_force = 2 * currentMass * sSpdFE * OMEGA_EARTH * Math.sin(cLat * D2R);

    // --- 7. MISE À JOUR DU DOM (Affichage des données rapides) ---
    // (Cette partie est conservée ici car elle dépend directement du traitement EKF/GPS)
    if ($('time-elapsed')) $('time-elapsed').textContent = `${((cTimePos - sTime) / 1000).toFixed(2)} s`;
    if ($('time-moving')) $('time-moving').textContent = `${timeMoving.toFixed(2)} s`;
    if ($('mode-ratio')) $('mode-ratio').textContent = `${R_FACTOR_RATIO.toFixed(3)} (Ratio)`;
    if ($('gps-accuracy-forced')) $('gps-accuracy-forced').textContent = `${gpsAccuracyOverride.toFixed(6)} m`;
    if ($('env-factor')) $('env-factor').textContent = `${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY} (x${ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT.toFixed(1)})`;
    if ($('mode-nether')) $('mode-nether').textContent = netherMode ? `ACTIVÉ (1:${NETHER_RATIO}) 🚀` : "DÉSACTIVÉ (1:1)";

    // Section Vitesse & Distance
    if ($('speed-stable')) $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(5)}`;
    if ($('speed-stable-kms')) $('speed-stable-kms').textContent = `${(sSpdFE / 1000).toExponential(3)} km/s`;
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${sSpdFE.toFixed(3)} m/s`;
    if ($('speed-max')) $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    if ($('speed-avg-moving')) $('speed-avg-moving').textContent = timeMoving > 1 ? `${(distM / timeMoving * KMH_MS).toFixed(5)} km/h` : '0.00000 km/h';
    if ($('speed-3d-inst')) $('speed-3d-inst').textContent = `${(spd3D_raw * KMH_MS).toFixed(5)} km/h`; 
    if ($('perc-speed-sound')) $('perc-speed-sound').textContent = `${(sSpdFE / C_S * 100).toFixed(2)} %`;
    if ($('perc-speed-c')) $('perc-speed-c').textContent = `${(sSpdFE / C_L * 100).toExponential(2)}%`;
    if ($('distance-total-km')) $('distance-total-km').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    if ($('distance-cosmic')) $('distance-cosmic').textContent = `${(distM / 149597870700).toExponential(2)} UA | ${(distM / 9460730472580800).toExponential(2)} al`;
    
    // Section GPS & Physique
    if ($('latitude')) $('latitude').textContent = `${cLat.toFixed(6)} °`;
    if ($('longitude')) $('longitude').textContent = `${cLon.toFixed(6)} °`;
    if ($('altitude-gps')) $('altitude-gps').textContent = kAlt_new !== null ? `${kAlt_new.toFixed(2)} m` : 'N/A';
    if ($('heading-display')) $('heading-display').textContent = headingRaw !== null ? `${headingRaw.toFixed(1)} °` : 'N/A';

    const altStatusTxt = kAlt_new !== null && kAlt_new < ALT_TH ? `OUI (< ${ALT_TH}m)` : 'Non';
    if ($('underground-status')) {
        $('underground-status').textContent = `Souterrain: ${altStatusTxt} (${modeStatus} | Acc: ${acc.toFixed(1)}m | R: ${R_dyn.toExponential(1)})`;
    }
    
    // Section Dynamique
    if ($('gravity-local')) $('gravity-local').textContent = `${local_g.toFixed(5)} m/s²`;
    if ($('accel-long')) $('accel-long').textContent = `${accel_long.toFixed(3)} m/s²`;
    if ($('force-g-long')) $('force-g-long').textContent = G_ACC > 0.1 ? `${(accel_long / local_g).toFixed(2)} G` : '0.00 G';
    if ($('vertical-speed')) $('vertical-speed').textContent = `${spdV.toFixed(2)} m/s`;
    
    // Section Champs & Énergie
    if ($('kinetic-energy')) $('kinetic-energy').textContent = `${kineticEnergy.toFixed(2)} J`;
    if ($('mechanical-power')) $('mechanical-power').textContent = `${mechanicalPower.toFixed(2)} W`;
    if ($('coriolis-force')) $('coriolis-force').textContent = `${coriolis_force.toExponential(2)} N`;

    // Section IMU (Simulation ici)
    const real_accel_x = Math.sin(cTimePos / 1000) * 0.1;
    const real_accel_y = Math.cos(cTimePos / 1000) * 0.1;
    const real_accel_z = -local_g;
    if ($('imu-accel-x')) $('imu-accel-x').textContent = `${real_accel_x.toFixed(2)} m/s²`;
    if ($('imu-accel-y')) $('imu-accel-y').textContent = `${real_accel_y.toFixed(2)} m/s²`;
    if ($('imu-accel-z')) $('imu-accel-z').textContent = `${real_accel_z.toFixed(2)} m/s²`;

    // Section Kalman
    if ($('kalman-uncert')) $('kalman-uncert').textContent = `${kUncert.toFixed(3)} m²/s² (P)`;
    if ($('alt-uncertainty')) $('alt-uncertainty').textContent = `${kAltUncert.toFixed(3)} m² (P alt)`;
    if ($('speed-error-perc')) $('speed-error-perc').textContent = `${R_dyn.toFixed(3)} m² (R dyn)`;
    
    // --- 8. SAUVEGARDE & MISE À JOUR CARTE ---
    lPos = pos; 
    lPos.speedMS_3D = spd3D_raw; 
    lPos.timestamp = cTimePos; 
    lPos.kAlt_old = kAlt_new; 
                              }
// =================================================================
// BLOC 3/4 : spacetime-astro.js
// Logique d'Astronomie (SunCalc), NTP et Météo (Fetch).
// =================================================================

// NOTE: La librairie suncalc.js DOIT être chargée.
const SunCalc = self.SunCalc || { getTimes: () => ({ sunrise: new Date(), sunset: new Date() }), getMoonIllumination: () => ({ phase: 0.5, fraction: 0.5, angle: 0 }) };

// --- FONCTIONS NTP & TEMPS ---

// Obtient l'heure locale corrigée (en supposant que lServH et lLocH sont définis)
function getCDate(lServH, lLocH) {
    if (lServH === 0 || lLocH === 0) return null;
    const offset = Date.now() - lLocH;
    return new Date(lServH + offset);
}

// Simule la synchronisation NTP (à remplacer par une requête serveur réelle)
async function syncH(lServH, lLocH) {
    return new Promise(resolve => {
        const now = Date.now();
        // Simuler un temps serveur NTP
        const simulatedServerTime = now + 123; 
        
        lServH = simulatedServerTime;
        lLocH = now;
        
        resolve({ lServH, lLocH });
    });
}

// --- FONCTIONS ASTRO ---
function updateAstro(lat, lon, lServH, lLocH) {
    const now = getCDate(lServH, lLocH);
    if (!now) return;

    // Calculs Astro (via SunCalc)
    const sunTimes = SunCalc.getTimes(now, lat, lon);
    const moonIllum = SunCalc.getMoonIllumination(now);
    
    // Calcul de l'Heure Solaire Vraie (TST)
    const TST_h = (now.getUTCHours() + now.getUTCMinutes()/60 + now.getUTCSeconds()/3600 + lon / 15) % 24; 

    // Rendu DOM des données lentes
    if ($('time-solar-true')) $('time-solar-true').textContent = `${Math.floor(TST_h).toString().padStart(2, '0')}:${Math.floor((TST_h % 1) * 60).toString().padStart(2, '0')}`;
    
    const moonPhaseName = moonIllum.fraction > 0.95 ? 'Pleine Lune' : moonIllum.fraction > 0.5 ? 'Gibbeuse' : 'Croissant';
    if ($('moon-phase')) $('moon-phase').textContent = `${moonPhaseName} (${(moonIllum.fraction * 100).toFixed(0)} %)`;

    // Logique d'animation du ciel (à affiner dans ui-renderer)
    const sun = $('sun');
    if (sun) {
        const tst_norm = TST_h / 24;
        sun.style.left = `${tst_norm * 100}%`;
    }
}

// --- FONCTIONS MÉTÉO ---
// Simule la récupération de la météo (à remplacer par votre proxy API)
async function fetchWeather(lat, lon) {
    // Simuler le résultat de l'API
    const data = {
        tempC: 18.0 + Math.sin(Date.now() / 3600000) * 5,
        pressure_hPa: 1019.5 + Math.cos(Date.now() / 7200000) * 10,
        humidity_perc: 66,
        tempK: 291.15,
        dew_point: 13.0,
        air_density: 1.225
    };
    return data;
}
// =================================================================
// BLOC 4/4 : ui-renderer.js
// Initialisation, gestion des événements DOM, boucles de mise à jour lentes.
// =================================================================

const DOM_SLOW_UPDATE_MS = 1000; // 1 Hz

let map = null;
let marker = null;

// --- FONCTIONS DE GESTION DU SYSTÈME GPS/MAP ---

function initMap() {
    map = L.map('gnss-map').setView([43.2965, 5.3698], 13);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; OpenStreetMap contributors'
    }).addTo(map);
    marker = L.marker([43.2965, 5.3698]).addTo(map);
}

function updateMap(lat, lon, accuracy) {
    if (!map || !marker) return;
    const latlng = L.latLng(lat, lon);
    marker.setLatLng(latlng);
    map.setView(latlng, map.getZoom() > 10 ? map.getZoom() : 15);
    // Ajoutez ici la logique de cercle de précision si nécessaire
}

function startGPS() {
    if (wID !== null) return;
    
    // Options de GPS (adaptées à votre code)
    const options = {
        enableHighAccuracy: true,
        maximumAge: 100, // Ms
        timeout: 5000 // Ms
    };
    
    // Démarre la surveillance GPS qui appelle processGPSData
    wID = navigator.geolocation.watchPosition(
        (pos) => {
            if (!emergencyStopActive) {
                processGPSData(pos); // Appel au Bloc 2
                updateMap(pos.coords.latitude, pos.coords.longitude, pos.coords.accuracy);
            }
        },
        (err) => {
            console.error('Erreur GPS:', err);
            // Mettre à jour le statut
        }, 
        options
    );
    $('toggle-gps-btn').textContent = '⏸️ PAUSE GPS';
}

function stopGPS() {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
    }
    $('toggle-gps-btn').textContent = '▶️ DÉMARRER GPS';
}

function emergencyStop() {
    emergencyStopActive = true;
    stopGPS();
    $('emergency-stop-btn').textContent = '▶️ REPRENDRE SYSTÈME';
    $('emergency-stop-btn').style.backgroundColor = 'red';
    // Mettre à jour tous les compteurs en pause...
}

function resumeSystem() {
    emergencyStopActive = false;
    $('emergency-stop-btn').textContent = '🛑 Arrêt d\'urgence';
    $('emergency-stop-btn').style.backgroundColor = '';
    // Redémarrer le GPS après la reprise
    startGPS();
}


// --- INITIALISATION DES ÉVÉNEMENTS DOM ---
document.addEventListener('DOMContentLoaded', () => {
    
    initMap(); // Initialise la carte

    // --- Initialisation des Contrôles ---
    // (Utilisation des IDs de votre code)
    if ($('mass-input')) {
        $('mass-input').addEventListener('input', (e) => { 
            currentMass = parseFloat(e.target.value) || 70.0; 
            if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        });
    }

    if ($('toggle-gps-btn')) {
        $('toggle-gps-btn').addEventListener('click', () => { 
            if (emergencyStopActive) return;
            wID === null ? startGPS() : stopGPS(); 
        });
    }

    if ($('emergency-stop-btn')) {
        $('emergency-stop-btn').addEventListener('click', () => { 
            !emergencyStopActive ? emergencyStop() : resumeSystem(); 
        });
    }
    
    // --- DÉMARRAGE DU SYSTÈME (Synchro NTP et Boucle Lente) ---
    
    // Démarrage de la synchro NTP
    syncH(lServH, lLocH).then(newTimes => {
        lServH = newTimes.lServH;
        lLocH = newTimes.lLocH;
        // Le GPS démarre généralement après, mais pour l'exemple on le lance via bouton.
    });

    // Boucle de mise à jour lente (Astro/Météo/Horloge)
    if (domID === null) {
        domID = setInterval(() => {
            const currentLat = lPos ? lPos.coords.latitude : 43.296; 
            const currentLon = lPos ? lPos.coords.longitude : 5.370;
            
            // 1. Mise à jour Astro (depuis spacetime-astro.js)
            if (typeof updateAstro === 'function') {
                updateAstro(currentLat, currentLon, lServH, lLocH);
            }
            
            // 2. Récupération Météo (si GPS actif)
            if (wID && !emergencyStopActive && typeof fetchWeather === 'function') {
                fetchWeather(currentLat, currentLon).then(data => {
                    if (data) {
                        // Stocke les valeurs pour le filtre EKF
                        lastP_hPa = data.pressure_hPa;
                        lastT_K = data.tempK;
                        lastH_perc = data.humidity_perc / 100.0;
                        
                        // Met à jour le DOM météo
                        if ($('temp-air-2')) $('temp-air-2').textContent = `${data.tempC.toFixed(1)} °C`;
                        if ($('pressure-2')) $('pressure-2').textContent = `${data.pressure_hPa.toFixed(0)} hPa`;
                        if ($('air-density')) $('air-density').textContent = `${data.air_density.toFixed(3)} kg/m³`;
                        if ($('dew-point')) $('dew-point').textContent = `${data.dew_point.toFixed(1)} °C`;
                    }
                });
            }
            
            // 3. Mise à jour Horloge locale (NTP)
            const now = getCDate(lServH, lLocH);
            if (now) {
                if ($('local-time') && !$('local-time').textContent.includes('SYNCHRO ÉCHOUÉE')) {
                    $('local-time').textContent = now.toLocaleTimeString('fr-FR');
                }
                if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
            }
            
        }, DOM_SLOW_UPDATE_MS); 
    }
});
