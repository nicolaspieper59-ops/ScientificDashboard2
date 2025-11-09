// =================================================================
// GNSS/IMU DASHBOARD - EKF ULTIME (Version Fusion et Astro)
// =================================================================

// --- CLÉS D'API & PROXY ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app"; 
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc"; // API NTP/UTC

// --- CONSTANTES GLOBALES ET PHYSIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const R_E_BASE = 6371000;   // Rayon terrestre moyen de base (m)
const KMH_MS = 3.6;         // Conversion m/s vers km/h
const C_S = 343;            // Vitesse du son dans l'air (m/s)
const MC_DAY_MS = 72 * 60 * 1000; // Durée d'un jour Minecraft en ms

const J1970 = 2440588, J2000 = 2451545; 
const dayMs = 1000 * 60 * 60 * 24;      
const MIN_DT = 0.01; 

// --- PARAMÈTRES EKF ---
const Q_NOISE = 0.01;       
const R_MIN = 0.05, R_MAX = 50.0; 
const MIN_SPD = 0.05;       
const ALT_TH = -50;         
const MAX_PLAUSIBLE_ACCEL = 20.0; 
const Q_ALT_NOISE = 0.1; 
const R_ALT_MIN = 0.5;
const ACC_DAMPEN_LOW = 5.0; 
const ACC_DAMPEN_HIGH = 50.0; 
const DOM_SLOW_UPDATE_MS = 1000; 

// --- DONNÉES CÉLESTES/GRAVITÉ ---
const CELESTIAL_DATA = {
    'EARTH': { G: 9.80665, R: R_E_BASE, name: 'Terre' },
    'MOON': { G: 1.62, R: 1737400, name: 'Lune' },
    'MARS': { G: 3.71, R: 3389500, name: 'Mars' },
    'ROTATING': { G: 0.0, R: R_E_BASE, name: 'Station Spatiale' } // G=0, sera remplacé par G_art
};
let G_ACC = CELESTIAL_DATA['EARTH'].G; // Gravité effective (m/s²)
let R_ALT_CENTER_REF = CELESTIAL_DATA['EARTH'].R; // Rayon de référence

// --- VARIABLES D'ÉTAT ---
let wID = null, domID = null, lPos = null, lat = null, lon = null, sTime = null;
let distM = 0, maxSpd = 0;
let kSpd = 0, kUncert = 1000; 
let timeMoving = 0; 
let lServH = null, lLocH = null; 
let lastFSpeed = 0; 
let kAlt = null;      
let kAltUncert = 10;  
let currentGPSMode = 'HIGH_FREQ'; 
let emergencyStopActive = false;
let selectedEnvironment = 'NORMAL'; 
let currentMass = 70; 
let R_FACTOR_RATIO = 1.0;   // Facteur de Rapport de Mouvement (MRF)
let currentCelestialBody = 'EARTH';
let rotationRadius = 100;
let angularVelocity = 0.0; 

// --- REFERENCES DOM ---
const $ = id => document.getElementById(id);

// --- FONCTIONS GÉO & UTILS ---

const dist = (lat1, lon1, lat2, lon2) => {
    // Utilise le rayon de la Terre sélectionnée ou par défaut
    const R = R_ALT_CENTER_REF; 
    const dLat = (lat2 - lat1) * D2R, dLon = (lon2 - lon1) * D2R;
    lat1 *= D2R; lat2 *= D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
};

// ... (initMap, updateMap, syncH, getCDate restent inchangés)

// --- FILTRE DE KALMAN (Fusion GPS/IMU) ---

/** Applique le filtre de Kalman à la vitesse 3D (Fusion GPS/Accélération). */
function kFilter(nSpd, dt, R_dyn, accel_input = 0) {
    if (dt === 0 || dt > 5) return kSpd; 
    
    // 1. PRÉDICTION (Basée sur l'accélération du capteur/modèle)
    const Q_MOTION = Q_NOISE * dt; 
    let pSpd = kSpd + (accel_input * dt); 
    let pUnc = kUncert + Q_MOTION; 
    
    // 2. MISE À JOUR (Correction par la mesure GPS)
    const R = R_dyn ?? R_MAX; 
    let K = pUnc / (pUnc + R); 
    kSpd = pSpd + K * (nSpd - pSpd); 
    kUncert = (1 - K) * pUnc; 
    
    return kSpd;
}

// ... (kFilterAltitude, getKalmanR restent inchangés)

function calculateMRF(kAltA) {
    if (kAltA === null || R_ALT_CENTER_REF === null) return 1.0;

    const R_current = R_ALT_CENTER_REF + kAltA; 
    const R_ref = R_ALT_CENTER_REF;
    const ratio = R_current / R_ref;

    return Math.max(0.001, Math.min(1.5, ratio));
}

// --- LOGIQUE CELESTE ET GRAVITE ---

function calculateArtificialGravity() {
    if (angularVelocity === 0 || rotationRadius === 0) return 0;
    // g_art = omega^2 * r
    const g_art = angularVelocity ** 2 * rotationRadius;
    return g_art;
}

/** Met à jour les constantes physiques pour le corps céleste sélectionné. */
function updateCelestialBody(body) {
    currentCelestialBody = body;
    let newG = 0;
    let newR = R_E_BASE;

    if (body === 'ROTATING') {
        // Gravité artificielle
        const g_art = calculateArtificialGravity();
        newG = g_art; 
        newR = R_E_BASE; // Garder une référence pour le MRF

        if ($('gravity-local')) $('gravity-local').textContent = `${newG.toFixed(4)} m/s² (Centrifuge)`;
    } else {
        // Gravité naturelle
        const data = CELESTIAL_DATA[body];
        newG = data.G;
        newR = data.R;
        if ($('gravity-local')) $('gravity-local').textContent = `${newG.toFixed(4)} m/s²`;
    }
    
    G_ACC = newG;
    R_ALT_CENTER_REF = newR; 
    
    // Réinitialiser les états EKF liés à la gravité/altitude
    kAlt = null; kAltUncert = 10;
    
    console.log(`Corps céleste réglé sur ${body}. G_ACC: ${G_ACC}, R_REF: ${R_ALT_CENTER_REF}`);
}

// --- FONCTIONS ASTRO & TEMPS (restent inchangées) ---

// ... (fonctions toDays, solarMeanAnomaly, eclipticLongitude, getSolarTime, getMinecraftTime inchangées)

// ... (updateAstro reste inchangée, utilise G_ACC via l'appel à updateDisp)

// --- FONCTIONS DE CONTRÔLE GPS & MÉTÉO (restent inchangées) ---

// ... (setGPSMode, startGPS, stopGPS, emergencyStop, resumeSystem, handleErr, fetchWeather restent inchangés)

// --- FONCTION PRINCIPALE DE MISE À JOUR GPS (EKF 3D) ---

function updateDisp(pos) {
    if (emergencyStopActive) return;

    // ... (Initialisation et Vérification du signal inchangées)

    // ... (Calculs de dt, usedAcc, kAlt_new, R_FACTOR_RATIO restent inchangés)
    
    // ... (Calculs de spdV, spdH, spd3D, Correction Anti-Spike restent inchangés)

    // 7. FILTRE DE KALMAN FINAL (Vitesse) - Fusion Inertielle/GPS
    const R_dyn = getKalmanR(usedAcc, alt, lastP_hPa); 
    
    // Entrée Accélération : Placeholder pour capteur réel
    // Si un accéléromètre réel était utilisé pour la fusion IMU/GPS, sa mesure irait ici
    const accel_sensor_input = 0; 
    
    const fSpd = kFilter(spd3D, dt, R_dyn, accel_sensor_input); 
    const sSpdFE = fSpd < MIN_SPD ? 0 : fSpd;
    
    // 8. Calculs d'accélération
    let accel_long = 0;
    if (dt > 0.05) {
        accel_long = (sSpdFE - lastFSpeed) / dt;
    }
    lastFSpeed = sSpdFE;

    // 9. Distance et Vitesse Max
    // Utilise R_FACTOR_RATIO (MRF)
    distM += sSpdFE * dt * R_FACTOR_RATIO; 
    if (sSpdFE > MIN_SPD) { timeMoving += dt; }
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    
    // --- MISE À JOUR DU DOM (Physique) ---
    const mass = currentMass;
    const kineticEnergy = 0.5 * mass * sSpdFE ** 2;
    const mechanicalPower = mass * sSpdFE * accel_long;
    
    // Mise à jour de la Force G en utilisant G_ACC variable
    if ($('force-g-long')) $('force-g-long').textContent = `${(accel_long / G_ACC).toFixed(2)} G`;
    
    // ... (Reste des mises à jour DOM inchangées)

    // SAUVEGARDE DES VALEURS POUR LA PROCHAINE ITÉRATION
    lPos = pos; 
    lPos.speedMS_3D = spd3D; 
    lPos.timestamp = cTimePos; 
    lPos.kAlt_old = kAlt_new; 
}

// --- INITIALISATION DES ÉVÉNEMENTS DOM ---

document.addEventListener('DOMContentLoaded', () => {
    
    initMap(); 
    
    // Écouteur pour la masse (inchangé)
    const massInput = document.createElement('input');
    massInput.type = 'number'; massInput.id = 'mass-input'; massInput.value = '70'; massInput.step = '0.1';
    massInput.style.width = '60px'; 
    massInput.onchange = () => { currentMass = parseFloat(massInput.value); $('mass-display').textContent = `${currentMass.toFixed(3)} kg`; };
    
    if ($('mass-display')) {
        const parent = $('mass-display').parentNode;
        parent.innerHTML = ''; 
        parent.appendChild(document.createTextNode('Masse (kg)'));
        const spanValue = document.createElement('span');
        spanValue.id = 'mass-display';
        spanValue.textContent = `${currentMass.toFixed(3)} kg`;
        parent.appendChild(spanValue);
        parent.appendChild(massInput); 
    }

    // Écouteur pour le sélecteur de Corps Céleste
    if ($('celestial-body-select')) $('celestial-body-select').addEventListener('change', (e) => { 
        updateCelestialBody(e.target.value); 
    });

    // Écouteurs pour la Gravité Artificielle (Rotation)
    const updateRotation = () => {
        rotationRadius = parseFloat($('rotation-radius')?.value) || 0;
        angularVelocity = parseFloat($('angular-velocity')?.value) || 0;
        if (currentCelestialBody === 'ROTATING') {
            updateCelestialBody('ROTATING'); 
        }
    };
    if ($('rotation-radius')) $('rotation-radius').addEventListener('change', updateRotation);
    if ($('angular-velocity')) $('angular-velocity').addEventListener('change', updateRotation);

    // Écouteur pour le sélecteur d'environnement (inchangé)
    if ($('environment-select')) $('environment-select').addEventListener('change', (e) => { 
        if (emergencyStopActive) return;
        selectedEnvironment = e.target.value; 
        if ($('env-factor')) $('env-factor').textContent = `${selectedEnvironment} (x${ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT})`; 
    });
    
    // Écouteur pour le rayon de référence (MRF)
    if ($('ref-radius-input')) $('ref-radius-input').addEventListener('change', (e) => {
        R_ALT_CENTER_REF = parseFloat(e.target.value);
        console.log(`Rayon de référence réglé à: ${R_ALT_CENTER_REF}`);
    });

    // ... (Reste des contrôles d'arrêt/reset/fréquence inchangés)

    syncH(); // Synchro NTP au démarrage

    if (domID === null) {
        domID = setInterval(() => {
            const fallbackLat = lat || 43.296;
            const fallbackLon = lon || 5.370;
            updateAstro(fallbackLat, fallbackLon);
        }, DOM_SLOW_UPDATE_MS); 
    }
    
    // Initialisation de la gravité
    updateCelestialBody(currentCelestialBody); 
    startGPS(); 
});

// ... (Le reste du code, notamment l'objet ENVIRONMENT_FACTORS et la logique de la carte/météo sont supposés être inclus ici)
