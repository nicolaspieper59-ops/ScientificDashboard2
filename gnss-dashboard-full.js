// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER FINAL COMPLET (V2.1 - UKF 21 ÉTATS)
// Fusion des fonctionnalités, corrections d'IMU, et calculs scientifiques avancés.
// =================================================================

// --- BLOC 1 : CONSTANTES, ÉTAT GLOBAL ET UTILITAIRES DE BASE ---
const $ = id => document.getElementById(id);

// Vérification des dépendances critiques (à des fins de robustesse)
if (typeof math === 'undefined') {
    console.error("🔴 ERREUR CRITIQUE: math.js n'a pas pu être chargé. Le filtre UKF est désactivé.");
}
if (typeof ProfessionalUKF === 'undefined') {
    console.error("🔴 ERREUR CRITIQUE: ProfessionalUKF n'est pas définie. Le filtre UKF est désactivé.");
}

// Constantes Mathématiques et Physiques
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const KMH_MS = 3.6;
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const G_U = 6.67430e-11;    // Constante gravitationnelle universelle
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const R_AIR = 287.058;      // Constante spécifique de l'air sec (J/kg·K)
const BARO_ALT_REF_HPA = 1013.25; // Pression standard (hPa)
const TEMP_SEA_LEVEL_K = 288.15; // Température standard (15°C) en Kelvin

// API Endpoints
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

// État Global
let currentPosition = { lat: 43.2964, lon: 5.3697, alt: 0.0, acc: 10.0, spd: 0.0, ts: 0 };
let lastIMU = { acc: { x: 0, y: 0, z: 0 }, pitch: 0, roll: 0 };
let ukf = null;
let currentUKFReactivity = 'NORMAL';
let lastP_hPa = BARO_ALT_REF_HPA;
let lastT_K = TEMP_SEA_LEVEL_K;
let lastH_perc = 0.5; // Humidité par défaut
let currentAirDensity = 1.225; // Densité de l'air standard (kg/m³)
let currentSpeedOfSound = 340.29; // Vitesse du son standard (m/s)
let currentMass = 70.0;
let isImuActive = false;
let imuListenersAttached = false; // Variable critique pour l'IMU

// Synchronisation NTP
let lastServerH = 0;
let lastLocalH = 0;
const DOM_SLOW_UPDATE_MS = 2000;

// Fonctions d'Affichage
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        return (decimals === 0 ? '0' : '0.00') + suffix;
    }
    return val.toFixed(decimals) + suffix;
};
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        const zeroDecimals = '0.' + Array(decimals).fill('0').join('');
        return zeroDecimals + 'e+0' + suffix;
    }
    return val.toExponential(decimals) + suffix;
};

// --- BLOC 2 : CALCULS SCIENTIFIQUES AVANCÉS ---

// Calcul de la Dilatation du Temps Relativiste (Lorentz Factor)
function getLorentzFactor(v_ms) {
    if (v_ms >= C_L) return Infinity;
    const ratio = v_ms / C_L;
    return 1.0 / Math.sqrt(1.0 - ratio * ratio);
}

// Calcul de la Densité de l'Air (à partir des données météo)
function calculateAirDensity(T_K, P_hPa, H_perc) {
    // Équation d'état de l'air humide (modèle simplifié pour R_air)
    const P_Pa = P_hPa * 100;
    const P_v = 6.112 * Math.exp((17.67 * (T_K - 273.15)) / (T_K - 29.65)) * H_perc * 100; // Pression de vapeur (Pa)
    const R_eff = R_AIR * (1 + 0.608 * P_v / P_Pa); // Constante du gaz effective
    return (P_Pa) / (R_eff * T_K); // ρ = P / (R_eff * T)
}

// Calcul de la Vitesse du Son
function getSpeedOfSound(T_K) {
    // Vitesse du son (m/s) : a = sqrt(γ * R * T), avec γ=1.4 pour l'air
    const GAMMA_AIR = 1.4;
    return Math.sqrt(GAMMA_AIR * R_AIR * T_K);
}

// Calcul de la Gravité Locale (WGS84 et Altitude)
window.getGravity = function(latRad, alt) {
    const G_E = 9.780327; // Gravité à l'équateur (m/s²)
    const WGS84_BETA = 0.0053024;
    const R_E_MEAN = 6371000; // Rayon terrestre moyen

    const sin2 = Math.sin(latRad) ** 2;
    const g_0 = G_E * (1 + WGS84_BETA * sin2); // Correction par latitude
    
    // Correction d'altitude (anomalie à l'air libre)
    const g_alt = g_0 * (1 - 2 * alt / R_E_MEAN); 
    return g_alt;
};


// --- BLOC 3 : LOGIQUE CRITIQUE DES CAPTEURS IMU (CORRECTION ROBUSTE V11.1) ---

/**
 * Tente d'initialiser les capteurs IMU (Accéléromètre et Gyroscope).
 * Implémente la logique de permission pour iOS/Chrome et un démarrage forcé pour Android.
 */
function initIMUSensors() {
    // Empêche l'attachement multiple des écouteurs
    if (window.imuListenersAttached) {
        if (!isImuActive) $('imu-status').textContent = 'En attente de données...';
        return;
    }
    
    const attachIMUListeners = () => {
        // ACCÉLÉROMÈTRE (devicemotion)
        if (window.DeviceMotionEvent) {
            window.addEventListener('devicemotion', (event) => {
                if (event.accelerationIncludingGravity) {
                    lastIMU.acc.x = event.accelerationIncludingGravity.x || 0;
                    lastIMU.acc.y = event.accelerationIncludingGravity.y || 0;
                    lastIMU.acc.z = event.accelerationIncludingGravity.z || 0;
                }
                isImuActive = true;
                $('imu-status').textContent = 'Actif (Mouvement)';
            }, { once: false });
        }
        
        // GYROSCOPE/ORIENTATION (deviceorientation)
        if (window.DeviceOrientationEvent) {
            window.addEventListener('deviceorientation', (event) => {
                lastIMU.pitch = event.beta || 0;
                lastIMU.roll = event.gamma || 0;
                isImuActive = true;
                $('imu-status').textContent = 'Actif (Orientation)';
            }, { once: false });
        }
        
        window.imuListenersAttached = true;
        if (!isImuActive) $('imu-status').textContent = 'En attente de données...'; 
    };

    // LOGIQUE DE PERMISSION (pour iOS et Chrome très récents)
    if (typeof DeviceMotionEvent !== 'undefined' && typeof DeviceMotionEvent.requestPermission === 'function') {
        $('imu-status').textContent = 'En attente de permission (Système)...';
        DeviceMotionEvent.requestPermission()
            .then(permissionState => {
                if (permissionState === 'granted') {
                    attachIMUListeners();
                } else {
                    $('imu-status').textContent = 'Refusé (Permission requise)';
                }
            })
            .catch(error => {
                // Tentative de démarrage forcé si la fonction requestPermission existe mais échoue
                console.warn("Échec de requestPermission. Tentative de démarrage forcé...");
                attachIMUListeners(); 
            });
    } else {
        // DÉMARRAGE FORCÉ/STANDARD (Path principal pour la majorité des Android/anciens navigateurs)
        attachIMUListeners();
    }
}


// --- BLOC 4 : GESTION GNSS (GPS) et UKF ---

// Fonction pour initialiser le GPS
function initGPS() {
    if ('geolocation' in navigator) {
        $('gps-status').textContent = 'Recherche...';
        const GPS_OPTS = {
            enableHighAccuracy: true, 
            maximumAge: 0, 
            timeout: 10000 
        };
        
        navigator.geolocation.watchPosition(
            (pos) => {
                const { latitude, longitude, altitude, speed, accuracy, timestamp } = pos.coords;
                
                // 1. Mise à jour des données brutes
                currentPosition.lat = latitude;
                currentPosition.lon = longitude;
                currentPosition.alt = altitude || 0.0;
                currentPosition.acc = accuracy || 10.0;
                currentPosition.spd = speed || 0.0;
                currentPosition.ts = timestamp;
                
                // 2. Mise à jour de l'UKF
                if (ukf && typeof ukf.update === 'function') {
                    // Les mesures UKF incluent GPS (position/vitesse) et IMU (accélération/attitude)
                    const g = window.getGravity(latitude * D2R, currentPosition.alt); 
                    ukf.update({ 
                        gps_lat: latitude, gps_lon: longitude, gps_alt: currentPosition.alt, 
                        gps_spd: currentPosition.spd, gps_acc: currentPosition.acc, 
                        imu_ax: lastIMU.acc.x, imu_ay: lastIMU.acc.y, imu_az: lastIMU.acc.z,
                        imu_pitch: lastIMU.pitch, imu_roll: lastIMU.roll,
                        gravity: g, // Gravité locale (pour l'intégration IMU)
                        reactivity: currentUKFReactivity
                    });
                } else {
                    // Affichage des données brutes si l'UKF n'est pas prêt
                    $('latitude-ekf').textContent = dataOrDefault(latitude, 6, '°');
                    $('longitude-ekf').textContent = dataOrDefault(longitude, 6, '°');
                    $('altitude-ekf').textContent = dataOrDefault(currentPosition.alt, 1, ' m');
                }
                
                $('gps-status').textContent = 'Actif';
            }, 
            (err) => {
                $('gps-status').textContent = `Erreur GPS: ${err.code} (${err.message})`;
                console.error(`Erreur GPS: ${err.code}, ${err.message}`);
            }, 
            GPS_OPTS
        );
    } else {
        $('gps-status').textContent = 'Non supporté';
    }
}


// --- BLOC 5 : TÉLÉCHARGEMENT DE DONNÉES (MÉTÉO, NTP) ---

// Synchronisation NTP (Correction du temps)
async function syncH() {
    try {
        const response = await fetch(SERVER_TIME_ENDPOINT);
        const data = await response.json();
        const serverTime = new Date(data.utc_datetime).getTime();
        const localTime = new Date().getTime();
        lastServerH = serverTime;
        lastLocalH = localTime;
        $('local-time').textContent = new Date(localTime).toLocaleTimeString('fr-FR') + ' (Synchro OK)';
    } catch (e) {
        $('local-time').textContent = 'SYNCHRO ÉCHOUÉE (Heure Locale Uniquement)';
    }
}

// Récupération de l'heure corrigée
function getCDate(serverH, localH) {
    if (serverH === 0 || localH === 0) return new Date();
    const now = new Date().getTime();
    const offset = serverH - localH;
    return new Date(now + offset);
}

// Récupération des données météo/polluants (via Proxy)
async function fetchWeather(lat, lon) {
    try {
        const url = `${PROXY_WEATHER_ENDPOINT}?lat=${lat.toFixed(4)}&lon=${lon.toFixed(4)}`;
        const response = await fetch(url);
        if (!response.ok) throw new Error("Erreur API Météo");
        const data = await response.json();
        
        // Calculs métrologiques essentiels
        data.tempK = data.tempC + 273.15;
        data.air_density = calculateAirDensity(data.tempK, data.pressure_hPa, data.humidity_perc / 100);
        
        return data;
    } catch (e) {
        console.error("Échec du fetch météo/polluants:", e);
        return null;
    }
}


// --- BLOC 6 : MISE À JOUR DOM (BOUCLE PRINCIPALE) ---

function updateDashboardDOM() {
    // Récupérer la position la plus stable (UKF ou GPS brute)
    let kLat, kLon, kAlt, kSpd, kGrav;
    
    if (ukf && ukf.getState && ukf.isInitialized()) {
        const state = ukf.getState();
        kLat = state.lat;
        kLon = state.lon;
        kAlt = state.alt;
        kSpd = state.speed_3d; // Vitesse 3D filtrée
        kGrav = window.getGravity(kLat * D2R, kAlt);
    } else {
        // Fallback GPS brut (moins stable)
        kLat = currentPosition.lat;
        kLon = currentPosition.lon;
        kAlt = currentPosition.alt;
        kSpd = currentPosition.spd;
        kGrav = window.getGravity(kLat * D2R, kAlt);
    }

    // --- 6.1 : Mise à jour GNSS/UKF (Rapide) ---
    $('latitude-ekf').textContent = dataOrDefault(kLat, 6, '°');
    $('longitude-ekf').textContent = dataOrDefault(kLon, 6, '°');
    $('altitude-ekf').textContent = dataOrDefault(kAlt, 1, ' m');
    $('speed-ekf').textContent = dataOrDefault(kSpd * KMH_MS, 2, ' km/h');
    $('accuracy-gps').textContent = dataOrDefault(currentPosition.acc, 1, ' m');
    $('vertical-speed-ekf').textContent = ukf ? dataOrDefault(ukf.getState().vertical_speed, 2, ' m/s') : 'N/A';
    
    // --- 6.2 : Calculs de Physique et Relativité (Rapide) ---
    const gamma = getLorentzFactor(kSpd);
    
    $('lorentz-factor').textContent = dataOrDefault(gamma, 8);
    $('time-dilation-speed').textContent = dataOrDefault((gamma - 1) * 86400 * 1000, 3, ' ms/jour');
    
    // Énergie relativiste
    const E_rest = currentMass * C_L * C_L;
    const E_total = gamma * E_rest;
    $('energy-rest-mass').textContent = dataOrDefaultExp(E_rest, 3, ' J');
    $('energy-relativiste').textContent = dataOrDefaultExp(E_total, 3, ' J');
    
    // Dynamique
    $('local-gravity').textContent = dataOrDefault(kGrav, 4, ' m/s²');
    
    // Traînée Aérodynamique (nécessite airDensity à jour)
    const dragForce = 0.5 * currentAirDensity * (kSpd * kSpd) * 1.0 * 1.0; // Simplifié: Cd=1.0, Surface=1.0 m²
    $('drag-force').textContent = dataOrDefault(dragForce, 3, ' N');
    
    // Mach Number (nécessite SpeedOfSound à jour)
    const machNumber = kSpd / currentSpeedOfSound;
    $('mach-number').textContent = dataOrDefault(machNumber, 3);
    
    // Coriolis (simplifié)
    const coriol_Force = 2 * currentMass * OMEGA_EARTH * kSpd * Math.sin(kLat * D2R);
    $('coriolis-force').textContent = dataOrDefault(coriol_Force, 6, ' N');

    // --- 6.3 : Calculs d'Astronomie (Lent, dépendance 'astro.js' ou 'ephem.js') ---
    if (typeof updateAstro === 'function') {
         updateAstro(kLat, kLon, getCDate(lastServerH, lastLocalH));
         // La fonction updateAstro doit gérer la mise à jour des éléments :
         // `TST_HRS`, `MST_HRS`, `EOT_MIN`, `noon-solar-utc`, `sun-alt`, `moon-phase-name`, etc.
    }
}

// Boucle Lente (Météo, Polluants, NTP)
function slowUpdateLoop() {
    const lat = currentPosition.lat;
    const lon = currentPosition.lon;
    
    // 1. Fetch Météo et Polluants
    fetchWeather(lat, lon).then(data => {
        if (data) {
            // Mise à jour des variables pour les calculs physiques
            lastP_hPa = data.pressure_hPa;
            lastT_K = data.tempK;
            lastH_perc = data.humidity_perc / 100.0;
            currentAirDensity = data.air_density;
            currentSpeedOfSound = getSpeedOfSound(lastT_K);
            
            // Mise à jour du DOM Météo/BioSVT
            $('temp-air-2').textContent = `${data.tempC.toFixed(1)} °C`;
            $('pressure-2').textContent = `${data.pressure_hPa.toFixed(0)} hPa`;
            $('humidity-2').textContent = `${data.humidity_perc} %`;
            $('air-density').textContent = `${data.air_density.toFixed(3)} kg/m³`;
            $('speed-of-sound-calc').textContent = `${currentSpeedOfSound.toFixed(2)} m/s`;
            $('dew-point').textContent = `${data.dew_point.toFixed(1)} °C`; // Assumer que l'API renvoie le dew point
            // Les autres éléments BioSVT/Polluants ('temp-bulb-humide', 'CAPE', 'air-quality-overall', etc.) doivent être mis à jour ici si les données sont dans 'data'.
        }
    });

    // 2. Mise à jour de l'horloge locale (NTP)
    const now = getCDate(lastServerH, lastLocalH);
    if (now) {
        if ($('local-time') && !$('local-time').textContent.includes('SYNCHRO ÉCHOUÉE')) {
            $('local-time').textContent = now.toLocaleTimeString('fr-FR');
        }
        if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
    }
}


// --- BLOC 7 : INITIALISATION DU SYSTÈME (onload) ---

window.addEventListener('load', () => {
    
    // Initialisation des systèmes critiques
    syncH(); // Démarrer la synchro NTP
    initGPS(); // Démarrer le GPS
    initIMUSensors(); // Démarrer les capteurs IMU (avec correction V11.1)

    // Initialisation de l'UKF après le chargement de math.js
    if (typeof math !== 'undefined' && typeof ProfessionalUKF !== 'undefined') {
        ukf = new ProfessionalUKF();
    } else {
         $('ukf-status').textContent = "UKF Désactivé (Dépendance manquante)";
    }

    // Attacher les Event Listeners pour les contrôles (si non gérés par un autre fichier)
    // Ex: Mass, Réactivité UKF, etc.
    if ($('mass-input')) {
        $('mass-input').addEventListener('input', (e) => {
            currentMass = parseFloat(e.target.value) || 70.0;
            $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        });
    }
    if ($('ukf-reactivity-mode')) {
         $('ukf-reactivity-mode').addEventListener('change', (e) => currentUKFReactivity = e.target.value);
    }

    // Premiers affichages par défaut et démarrage des boucles
    updateDashboardDOM();
    setInterval(updateDashboardDOM, 250); // Boucle Rapide (GNSS/UKF/Physique)
    
    slowUpdateLoop();
    setInterval(slowUpdateLoop, DOM_SLOW_UPDATE_MS); // Boucle Lente (Météo/Astro/NTP)
});
