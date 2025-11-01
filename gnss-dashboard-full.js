// =================================================================
// FICHIER JS PARTIE 1/2 : gnss-dashboard-part1.js (Final V11.1)
// CORE, KALMAN, CONSTANTES & ÉTAT GLOBAL
// =================================================================

// --- CLÉS D'API & PROXY VERCEL (Non utilisés en mode Hors Ligne pour la Météo/NTP) ---
// *Si HORS LIGNE, ces appels échoueront et le dashboard utilisera l'horloge locale.*
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app"; 
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc"; 

// --- CONSTANTES GLOBALES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI; 
const C_L = 299792458; // Vitesse de la lumière (m/s)
const R_E = 6371000;   // Rayon terrestre moyen (m)
const KMH_MS = 3.6;    // Facteur m/s -> km/h
const C_S = 343;       // Vitesse du son standard (m/s)
const G_ACC = 9.80665; // Accélération de la gravité (m/s²)
const dayMs = 1000 * 60 * 60 * 24; 
const J1970 = 2440588, J2000 = 2451545; 
const MIN_DT = 0.01; 
const MSL_OFFSET_M = 40; 
const NETHER_RATIO = 8; 
const ALT_TH = -50; 
const DOM_SLOW_UPDATE_MS = 1000; 

// PARAMÈTRES AVANCÉS DU FILTRE DE KALMAN
const Q_NOISE = 0.01;       
const R_MIN = 0.05, R_MAX = 50.0; 
const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0 },
    'METAL': { R_MULT: 2.5 },      
    'FOREST': { R_MULT: 1.5 },     
    'CONCRETE': { R_MULT: 3.0 },   
};
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};

// --- VARIABLES D'ÉTAT ---
let wID = null, domID = null, lPos = null; 
let lat = null, lon = null, sTime = null; 
let distM = 0, distMStartOffset = 0, maxSpd = 0; 
let kSpd = 0, kUncert = 1000; 
let timeMoving = 0, timeElapsed = 0; 
let lServH = null, lLocH = null; 
let lastFSpeed = 0; 
let mslOffset = MSL_OFFSET_M; 
let lastP_hPa = null, lastT_K = null, lastH_perc = null; 
let emergencyStopActive = false;
let netherMode = false;
let selectedEnvironment = 'NORMAL'; 
let currentGPSMode = 'HIGH_FREQ'; 

// NOUVEAU : MASSE ET ACCÉLÉRATION
let DEVICE_MASS_KG = 70; // Masse par défaut (70 kg)
let LATERAL_ACCEL_MS2 = 0.0; // Accélération latérale estimée/mesurée

const $ = id => document.getElementById(id);


// ===========================================
// FONCTIONS GÉO & UTILS (dist, getCDate, correctAltitudeToMSL)
// ===========================================

/** ⌚ Obtient la date/heure corrigée (NTP si synchro ok) */
function getCDate() { 
    // Utilise l'horloge locale si la synchro NTP a échoué (mode hors ligne)
    return sTime ? new Date(Date.now() - (lLocH.getTime() - lServH.getTime())) : new Date();
}

/** Calcule la distance 3D Haversine entre deux positions. */
function dist(pos1, pos2) {
    const lat1 = pos1.coords.latitude * D2R;
    const lat2 = pos2.coords.latitude * D2R;
    const dLat = (pos2.coords.latitude - pos1.coords.latitude) * D2R;
    const dLon = (pos2.coords.longitude - pos1.coords.longitude) * D2R;

    const a = Math.sin(dLat / 2) ** 2 +
              Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    
    const distance2D = R_E * c;
    
    const dAlt = (pos2.coords.altitude || 0) - (pos1.coords.altitude || 0);
    return Math.sqrt(distance2D ** 2 + dAlt ** 2);
}

/** 🌐 Corrige l'altitude WGS84 (GPS) en Altitude MSL (Niveau de la mer) */
function correctAltitudeToMSL(wgs84Alt, geoidHeight) {
    if (wgs84Alt === null) return null;
    return wgs84Alt - (geoidHeight ?? mslOffset);
}

/** Calcule le facteur R (Précision 3D Dynamique) du filtre de Kalman. */
function getKalmanR(acc, alt, P_hPa) {
    let R = acc ** 2; 
    R = Math.max(R_MIN, Math.min(R_MAX, R));
    
    const envFactor = ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT;
    
    // CORRECTION MÉTÉO (ne s'applique pas si P_hPa est null en hors ligne)
    if (P_hPa !== null) {
        const pressureDeviation = Math.abs(1013.25 - P_hPa);
        const pressureFactor = 1.0 + pressureDeviation / 1013.25 * 0.1;
        R *= Math.max(1.0, pressureFactor);
    }
    
    R *= envFactor;
    if (alt < ALT_TH) { R *= 2.0; } 

    return Math.max(R_MIN, Math.min(R_MAX, R));
}

/** Applique le filtre de Kalman à la vitesse 3D. */
function kFilter(z, dt, acc, alt, P_hPa) {
    if (isNaN(z) || z === null || dt < MIN_DT) {
        kUncert += Q_NOISE * dt;
        return kSpd;
    }
    
    // 1. Prédiction
    const kSpdPred = kSpd + lastFSpeed * dt; 
    const kUncertPred = kUncert + Q_NOISE * dt;

    // 2. Mise à jour (Correction)
    const R_dyn = getKalmanR(acc, alt, P_hPa);
    const K = kUncertPred / (kUncertPred + R_dyn);
    
    kSpd = kSpdPred + K * (z - kSpdPred);
    kUncert = (1 - K) * kUncertPred;

    lastFSpeed = (kSpd - kSpdPred) / dt || 0; 
    return kSpd;
}

function getDynamicMinSpd() {
    return 0.1; // Vitesse minimale en m/s pour être considéré en mouvement
    }
// =================================================================
// FICHIER JS PARTIE 2/2 : gnss-dashboard-part2.js (Final V11.1)
// ASTRO, MÉTÉO, CARTE, LOGIQUE DOM & INITIALISATION
// NÉCESSITE gnss-dashboard-part1.js et Leaflet/SunCalc (si en ligne)
// =================================================================
// =================================================================
// Déclaration des variables pour les capteurs IMU (Réel)
// Ces variables stockeront les données brutes des accéléromètres
// =================================================================
let ACCEL_LATERAL_IMU = 0; // Accélération latérale (axe Y)
let ACCEL_LONG_IMU = 0;    // Accélération longitudinale (axe X)
let IMU_IS_ACTIVE = false;
let DEVICE_MASS_KG = parseFloat($('mass-input').value) || 70.000; 

// Nouvelle variable pour simuler la dernière synchronisation connue (GMT/UTC)
let LAST_KNOWN_SYNC_UTC; 

// =================================================================
// 1. GESTION DU TEMPS (SYNCHRONISATION HORS LIGNE GMT/UTC)
// =================================================================

function updateLocalTime() {
    const now = new Date();
    
    // Afficher l'heure en format UTC (équivalent GMT), basée sur l'horloge de l'appareil.
    const utcTime = now.toLocaleTimeString('fr-FR', {
        timeZone: 'UTC', 
        hour: '2-digit', 
        minute: '2-digit', 
        second: '2-digit'
    });
    
    // Affichage de l'heure GMT/UTC
    // Le libellé est mis à jour dans le code pour plus de clarté en mode hors ligne.
    $('local-time').textContent = utcTime + ' UTC (Horloge interne)';
    
    // Affichage de la date locale
    $('date-display').textContent = now.toLocaleDateString('fr-FR', {
        year: 'numeric', month: 'long', day: 'numeric'
    });
}

// =================================================================
// 2. GESTION DES CAPTEURS IMU (Accélération et Force Centrifuge RÉELLES)
// [Pas de changement ici par rapport à la réponse précédente, mais inclus pour le contexte]
// =================================================================

function requestDeviceMotionPermission() {
    // Demander la permission sur iOS 13+ ou démarrer directement
    if (typeof DeviceOrientationEvent !== 'undefined' && typeof DeviceOrientationEvent.requestPermission === 'function') {
        DeviceOrientationEvent.requestPermission()
            .then(permissionState => {
                if (permissionState === 'granted') {
                    startDeviceMotionTracking();
                } else {
                    console.error('Permission DeviceMotion refusée');
                    // Rétablir l'avertissement si la mesure réelle échoue
                    document.querySelector('.warning-note').style.display = 'block'; 
                }
            })
            .catch(console.error);
    } else {
        startDeviceMotionTracking();
    }
}

function startDeviceMotionTracking() {
    window.addEventListener('devicemotion', (event) => {
        if (event.acceleration || event.accelerationIncludingGravity) {
            const accelerationSource = event.acceleration || event.accelerationIncludingGravity;
            ACCEL_LONG_IMU = accelerationSource.x;
            ACCEL_LATERAL_IMU = accelerationSource.y;
            IMU_IS_ACTIVE = true;
            updateDynamicIMUDisplay();
        }
    }, true);
    
    if (IMU_IS_ACTIVE) {
        document.querySelector('.warning-note').style.display = 'none';
    }
}

function updateDynamicIMUDisplay() {
    $('current-mass').textContent = DEVICE_MASS_KG.toFixed(3);
    const accelLat = ACCEL_LATERAL_IMU;
    const forceCentrifugal = DEVICE_MASS_KG * Math.abs(accelLat); 
    const forceG = Math.abs(accelLat) / 9.80665; 

    $('accel-lat').textContent = accelLat.toFixed(3) + ' m/s² (MESURÉE)';
    $('force-centrifugal').textContent = forceCentrifugal.toFixed(2) + ' N (MESURÉE)';
    $('force-centrifugal-g').textContent = forceG.toFixed(2) + ' G (MESURÉE)';
    $('accel-long').textContent = ACCEL_LONG_IMU.toFixed(3) + ' m/s² (MESURÉE)';
    
    document.querySelector('.warning-note').style.display = 'none';
}

// =================================================================
// 3. GESTION DE L'ASTRONOMIE (SunCalc - Méthode PlanetCalc/Standard)
// [Pas de changement ici, mais inclus pour le contexte]
// =================================================================

function updateAstronomy(latitude, longitude) {
    const now = new Date();
    
    if (typeof SunCalc === 'undefined') {
        console.warn("La librairie SunCalc n'est pas chargée.");
        return;
    }

    const sunPos = SunCalc.getPosition(now, latitude, longitude);
    const moonIllumination = SunCalc.getMoonIllumination(now);
    const times = SunCalc.getTimes(now, latitude, longitude);

    const eclipticLong = sunPos.eclipticLng;
    $('ecliptic-long').textContent = (eclipticLong * 180 / Math.PI).toFixed(2) + ' °';
    $('sun-elevation').textContent = (sunPos.altitude * 180 / Math.PI).toFixed(2) + ' °';
    $('sun-azimuth').textContent = (sunPos.azimuth * 180 / Math.PI + 180).toFixed(2) + ' °';

    if (times.solarNoon && times.nadir) {
        const solarNoonOffsetMs = times.solarNoon.getTime() - new Date(now.getFullYear(), now.getMonth(), now.getDate(), 12, 0, 0).getTime();
        const eotMinutes = solarNoonOffsetMs / (1000 * 60);
        $('eot-min').textContent = eotMinutes.toFixed(2) + ' min';
    } else {
        $('eot-min').textContent = 'N/D min';
    }
    
    $('lunar-phase-perc').textContent = (moonIllumination.fraction * 100).toFixed(1) + ' %';
}


// =================================================================
// 4. INITIALISATION GLOBALE
// =================================================================

function initGeolocation() {
    
    // Initialisation des listeners pour les contrôles
    $('toggle-gps-btn').addEventListener('click', () => {
        // ... (Logique pour démarrer GPS) ...
        requestDeviceMotionPermission();
    });

    // Écouteur pour la masse (précision au gramme)
    const $massInput = $('mass-input');
    $massInput.addEventListener('change', (e) => {
        const newMass = parseFloat(e.target.value);
        if (!isNaN(newMass) && newMass >= 0.001) { 
            DEVICE_MASS_KG = newMass;
        } else {
            e.target.value = DEVICE_MASS_KG.toFixed(3); 
        }
        $('current-mass').textContent = DEVICE_MASS_KG.toFixed(3);
    });

    // Initialisation de la dernière synchronisation connue (GMT/UTC)
    // C'est le moment où le navigateur a chargé la page (basé sur l'horloge locale).
    LAST_KNOWN_SYNC_UTC = new Date().toUTCString();
    console.log("Dernière sync UTC connue (chargement page): " + LAST_KNOWN_SYNC_UTC);


    // Démarrage de la mise à jour de l'heure
    setInterval(updateLocalTime, 1000);
    updateLocalTime();
    
    // Démarrage de la mise à jour des données astronomiques 
    const defaultLat = 48.8566; // Utiliser la position GPS réelle lorsque disponible
    const defaultLng = 2.3522; 
    updateAstronomy(defaultLat, defaultLng); 
    setInterval(() => updateAstronomy(defaultLat, defaultLng), 60000); 
    
    // Tentative de démarrage immédiat des capteurs IMU
    requestDeviceMotionPermission();
    
    // ... (Reste de l'initialisation GPS/EKF) ...
}

document.addEventListener('DOMContentLoaded', initGeolocation);
