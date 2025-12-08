// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET & ROBUSTE (UKF 21 ÉTATS)
// FUSION DE TOUTES LES MEILLEURES FONCTIONNALITÉS ET CORRECTIONS
// =================================================================

// ⚠️ DÉPENDANCES CRITIQUES (doivent être chargées dans l'HTML) :
// - math.min.js, leaflet.js, turf.min.js
// - lib/ukf-lib.js, lib/astro.js, lib/ephem.js
// =================================================================

// --- 1. CONSTANTES ET UTILITAIRES DE BASE ---

const $ = id => document.getElementById(id);

// --- CLÉS D'API & ENDPOINTS (Assumant un proxy Vercel) ---
const PROXY_WEATHER_ENDPOINT = "/api/weather"; // Relatif
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";
const DOM_SLOW_UPDATE_MS = 2000;
const DOM_FAST_UPDATE_MS = 50; // Pour les chronomètres

// --- CONSTANTES MATHÉMATIQUES & PHYSIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const KMH_MS = 3.6;         
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const G_U = 6.67430e-11;    // Constante gravitationnelle universelle
const G_STD = 9.80665;      // Gravité standard (m/s²)
const OMEGA_EARTH = 7.292115e-5; // Vitesse de rotation Terre (rad/s)

// WGS84 (Terre)
const WGS84_A = 6378137.0;  // Rayon équatorial
const WGS84_G_EQUATOR = 9.780327;
const WGS84_E2 = 0.00669437999014; // Excentricité au carré

// Atmosphère Standard (ISA)
const TEMP_SEA_LEVEL_K = 288.15; // 15°C
const RHO_SEA_LEVEL = 1.225;
const R_SPECIFIC_AIR = 287.058;
const GAMMA_AIR = 1.4;

// --- FONCTIONS UTILITAIRES DE FORMATAGE (Robustesse anti-NaN/N/A) ---
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || Math.abs(val) > 1e18) {
        return (decimals === 0 ? '0' : '0.00') + suffix;
    }
    return val.toFixed(decimals) + suffix;
};

const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || Math.abs(val) > 1e100) {
        const zeroDecimals = '0.' + Array(decimals).fill('0').join('');
        return zeroDecimals + 'e+0' + suffix;
    }
    return val.toExponential(decimals) + suffix;
};


// --- 2. ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE ---

let ukf = null;
let mapInstance = null;
let mapMarker = null;
let mapCircle = null;
let gpsWatchId = null;

// Variables de Session
let totalDistance = 0.0;    // Distance parcourue (m)
let maxSpeed = 0.0;         // Vitesse maximale (m/s)
let isEmergencyStopped = false; // Arrêt d'urgence
let isGpsPaused = false; 
let sessionStartTime = Date.now(); // Début de session
let movementStartTime = null; // Début du mouvement (pour vitesse moyenne)
let currentMovementTime = 0; // Temps de mouvement total (ms)

// Variables de Physique et Contrôle
let currentMass = 70.0;     // Masse de l'objet (kg)
let currentCelestialBody = 'TERRE'; // 'TERRE', 'MARS', 'ROTATING'
let netherMode = false;     // Mode Minecraft
let currentSpeed = 0.0;     // Vitesse filtrée (m/s)
let kAlt = 0.0;             // Altitude filtrée (m)
let G_ACC = G_STD;          // Gravité locale (m/s²)

// Position de travail (Marseille par défaut)
let currentPosition = { lat: 43.2964, lon: 5.3697, acc: 100.0, spd: 0.0, alt: 0.0 };
let lastPosition = { lat: 43.2964, lon: 5.3697 }; // Pour la distance Turf

// Météo (Valeurs ISA par défaut)
let lastWeatherData = {
    tempC: 15.0,
    pressure_hPa: 1013.25,
    humidity_perc: 50,
    tempK: 288.15,
    air_density: RHO_SEA_LEVEL,
    speed_of_sound: 340.29,
    timestamp: 0
};

let lServH = 0; // Temps serveur (NTP)
let lLocH = 0;  // Temps local


// --- 3. MODÈLES DE CALCULS AVANCÉS (Vérification et Correction) ---

// 3.1. Calcul de la Gravité (WGS84 et Simulé)
function updateCelestialBody(body, alt, radius = 100, omega = 0) {
    let G_BASE;
    let PLANET_MASS = 5.972e24; // Terre
    let PLANET_RADIUS = WGS84_A; // Terre

    switch (body) {
        case 'TERRE':
            // Utilisation du modèle WGS84 pour la gravité locale
            const phi = currentPosition.lat * D2R;
            const sinSq = Math.sin(phi) ** 2;
            const WGS84_BETA = (WGS84_G_EQUATOR * WGS84_A - 9.832) / WGS84_G_EQUATOR; // Simplification du facteur
            const factor = (1 + WGS84_BETA * sinSq) / Math.sqrt(1 - WGS84_E2 * sinSq);
            G_BASE = WGS84_G_EQUATOR * factor;
            // Correction d'altitude (Free Air)
            G_ACC = G_BASE * (1 - (2 * alt / WGS84_A));
            break;

        case 'MARS':
            G_ACC = 3.7207; // Gravité de surface de Mars
            PLANET_MASS = 6.39e23;
            PLANET_RADIUS = 3389500;
            break;

        case 'ROTATING':
            // Simulation de la force centrifuge sur un objet en rotation
            const radiusM = parseFloat(radius) || 100;
            const omegaRad = parseFloat(omega) || 0.0;
            const A_CENTRIFUGAL = (radiusM * omegaRad ** 2);
            // On soustrait l'accélération centrifuge à la gravité terrestre standard
            G_ACC = G_STD - A_CENTRIFUGAL; 
            if (G_ACC < 0) G_ACC = 0.001; // Éviter la gravité négative
            break;

        default:
            G_ACC = G_STD; // Fallback
    }

    if ($('gravity-local')) $('gravity-local').textContent = `${dataOrDefault(G_ACC, 4)} m/s²`;

    // Calcul du Rayon de Schwarzschild (Rs = 2GM/c²)
    const Rs = (2 * G_U * PLANET_MASS) / (C_L ** 2);
    if ($('schwarzschild-radius')) $('schwarzschild-radius').textContent = dataOrDefaultExp(Rs, 4, ' m');
    
    return { G_ACC_NEW: G_ACC };
}

// 3.2. Modèle Relativité (Facteur de Lorentz)
function calculateRelativity(v) {
    // Évite la division par zéro si v > C_L (impossible, mais sécurise)
    const v_safe = Math.min(v, C_L * 0.99999999999);
    const beta = v_safe / C_L;
    const gamma = 1.0 / Math.sqrt(1.0 - beta ** 2);

    // Dilatation du temps cinétique (ns/jour)
    const DILATION_FACTOR = 1000000000 * 86400 * (gamma - 1);

    // Énergie relativiste (E = γmc²)
    const E = gamma * currentMass * C_L ** 2;
    const E0 = currentMass * C_L ** 2;

    return {
        gamma,
        beta,
        timeDilation: DILATION_FACTOR,
        E,
        E0
    };
}

// 3.3. Modèle Aérodynamique (Forces et Mach)
function calculateAerodynamics(v) {
    const rho = lastWeatherData.air_density;
    const Cs = lastWeatherData.speed_of_sound;

    // Vitesse du Son (Correction ISA si Météo HS)
    const speedOfSound = Cs > 0 ? Cs : 340.29; 

    // Nombre de Mach
    const mach = v / speedOfSound;

    // Pression Dynamique (q = 0.5 * ρ * v²)
    const q = 0.5 * rho * v ** 2;

    // Force de Coriolis (simplifiée, uniquement composante horizontale F_corio = 2mωv sin(φ))
    const phi = currentPosition.lat * D2R;
    const F_coriolis = 2 * currentMass * OMEGA_EARTH * v * Math.sin(phi);

    return { mach, q, F_coriolis, speedOfSound };
}


// --- 4. GESTION TEMPS (NTP & Chronomètres) ---

// 4.1. Synchro NTP
function getCDate(serverTimeMs, localTimeMs) {
    if (serverTimeMs > 0 && localTimeMs > 0) {
        const offset = serverTimeMs - localTimeMs;
        return new Date(Date.now() + offset);
    }
    return new Date(); 
}

async function syncTime() {
    try {
        const res = await fetch(SERVER_TIME_ENDPOINT, { signal: AbortSignal.timeout(3000) });
        if (!res.ok) throw new Error("API NTP HS");
        const data = await res.json();
        lServH = new Date(data.utc_datetime).getTime();
        lLocH = Date.now();
    } catch (e) {
        lServH = 0;
        lLocH = 0;
        console.warn("NTP: Échec, utilisation heure système.");
    }
}

// 4.2. Boucle Rapide pour Chronomètres
function fastLoop() {
    const now = Date.now();
    
    // Temps écoulé (Session)
    const sessionTimeSec = (now - sessionStartTime) / 1000;
    if ($('temps-ecoule')) $('temps-ecoule').textContent = dataOrDefault(sessionTimeSec, 2, ' s');
    
    // Temps de Mouvement
    if (currentSpeed > 0.05 && movementStartTime === null) {
        movementStartTime = now;
    } else if (currentSpeed <= 0.05 && movementStartTime !== null) {
        currentMovementTime += (now - movementStartTime);
        movementStartTime = null;
    }
    const totalMovementTimeSec = currentMovementTime / 1000;
    if ($('temps-mouvement')) $('temps-mouvement').textContent = dataOrDefault(totalMovementTimeSec, 2, ' s');
    
    // Heure Minecraft (Correction: 20 minutes/jour, donc 72x plus rapide)
    const mcTimeMs = (now - sessionStartTime) * 72;
    const mcHour = Math.floor((mcTimeMs / 3600000) % 24);
    const mcMinute = Math.floor((mcTimeMs / 60000) % 60);
    if ($('heure-minecraft')) $('heure-minecraft').textContent = `${String(mcHour).padStart(2, '0')}:${String(mcMinute).padStart(2, '0')}`;
}


// --- 5. GESTION CARTE (Leaflet) ---

function initMap() {
    if (typeof L === 'undefined') return;
    mapInstance = L.map('map').setView([currentPosition.lat, currentPosition.lon], 16);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; OpenStreetMap contributors',
        maxZoom: 19
    }).addTo(mapInstance);
    mapMarker = L.marker([currentPosition.lat, currentPosition.lon]).addTo(mapInstance);
    mapCircle = L.circle([currentPosition.lat, currentPosition.lon], {
        color: '#007bff', fillColor: '#007bff', fillOpacity: 0.2, radius: currentPosition.acc
    }).addTo(mapInstance);
}

function updateMap(lat, lon, acc) {
    if (!mapInstance || !mapMarker) return;
    const newLatLng = new L.LatLng(lat, lon);
    mapMarker.setLatLng(newLatLng);
    mapCircle.setLatLng(newLatLng);
    mapCircle.setRadius(acc);
    // mapInstance.panTo(newLatLng); // Optionnel: centrage constant
}


// --- 6. GESTION CAPTEURS (GPS & IMU) ---

// 6.1. IMU (Accéléromètre/Gyroscope)
function handleMotion(event) {
    if (isEmergencyStopped) return;

    const acc = event.accelerationIncludingGravity;
    const gyro = event.rotationRate || {};

    // Affichage Accélération
    if (acc) {
        if ($('acceleration-x')) $('acceleration-x').textContent = dataOrDefault(acc.x, 2);
        if ($('acceleration-y')) $('acceleration-y').textContent = dataOrDefault(acc.y, 2);
        if ($('acceleration-z')) $('acceleration-z').textContent = dataOrDefault(acc.z, 2);
    }
    
    // Envoi des données à l'UKF
    if (ukf && acc) {
        const ax = acc.x || 0; const ay = acc.y || 0; const az = acc.z || 0;
        const gx = gyro.alpha || 0; const gy = gyro.beta || 0; const gz = gyro.gamma || 0;
        ukf.predict(ax, ay, az, gx, gy, gz);
    }

    // Affichage Niveau à Bulle (utilisant rotationRate pour le débogage)
    if ($('pitch-level')) $('pitch-level').textContent = dataOrDefault(gyro.beta, 1, '°');
    if ($('roll-level')) $('roll-level').textContent = dataOrDefault(gyro.gamma, 1, '°');
}

function activateDeviceMotion() {
    if ($('imu-status')) $('imu-status').textContent = 'Initialisation... 🟡';

    // Gestion des permissions iOS 13+ (nécessite HTTPS)
    if (typeof DeviceMotionEvent !== 'undefined' && typeof DeviceMotionEvent.requestPermission === 'function') {
        DeviceMotionEvent.requestPermission()
            .then(response => {
                if (response === 'granted') {
                    window.addEventListener('devicemotion', handleMotion);
                    if ($('imu-status')) $('imu-status').textContent = 'Actif 🟢';
                } else {
                    if ($('imu-status')) $('imu-status').textContent = 'Refusé 🔴';
                }
            })
            .catch(err => {
                if ($('imu-status')) $('imu-status').textContent = 'Erreur 🔴';
            });
    } else {
        // Pour Android / autres (pas de permission nécessaire)
        window.addEventListener('devicemotion', handleMotion);
        if ($('imu-status')) $('imu-status').textContent = 'Actif 🟢';
    }
}

// 6.2. GPS
function onPositionUpdate(pos) {
    if (isGpsPaused || isEmergencyStopped) return;

    const { latitude, longitude, accuracy, altitude, speed } = pos.coords;
    const currentLat = latitude;
    const currentLon = longitude;
    const currentAlt = altitude || kAlt; // Utilise l'UKF/précédent si l'alt est nulle

    // Calcul de la distance 3D depuis la dernière position
    if (typeof turf !== 'undefined' && turf.distance) {
        const from = turf.point([lastPosition.lon, lastPosition.lat]);
        const to = turf.point([currentLon, currentLat]);
        // Distance Haversine en km
        const haversineDistKm = turf.distance(from, to, { units: 'kilometers' });
        // Conversion en mètres
        const distM = haversineDistKm * 1000; 

        // Ajouter la composante verticale (altitude)
        const altChange = currentAlt - lastPosition.alt || 0;
        const dist3D = Math.sqrt(distM ** 2 + altChange ** 2);
        
        totalDistance += dist3D;
        lastPosition = { lat: currentLat, lon: currentLon, alt: currentAlt };
        
        // CORRECTION MODE NETHER
        const displayRatio = netherMode ? 1 / 8 : 1.0;
        if ($('distance-total-3d')) $('distance-total-3d').textContent = dataOrDefault((totalDistance * displayRatio) / 1000, 3, ' km');
    }

    // Mise à jour objet courant
    currentPosition = {
        lat: currentLat, lon: currentLon, acc: accuracy, alt: currentAlt, spd: speed || 0
    };

    // Mise à jour Vitesse Max
    maxSpeed = Math.max(maxSpeed, currentPosition.spd);

    // Pipeline UKF
    if (ukf) {
        ukf.update(currentPosition, 'GNSS');
        const state = ukf.getState();
        kAlt = state.alt;
        currentSpeed = state.speed;

        // Affichage UKF/EKF
        if ($('lat-ekf')) $('lat-ekf').textContent = dataOrDefault(state.lat, 6, ' °');
        if ($('lon-ekf')) $('lon-ekf').textContent = dataOrDefault(state.lon, 6, ' °');
        if ($('alt-ekf')) $('alt-ekf').textContent = dataOrDefault(kAlt, 2, ' m');
        if ($('ukf-v-uncert')) $('ukf-v-uncert').textContent = dataOrDefault(state.kUncert, 3, ' m');
        if ($('ekf-status')) $('ekf-status').textContent = 'Fusion 🟢';
        if ($('vitesse-verticale')) $('vitesse-verticale').textContent = dataOrDefault(state.vD, 2, ' m/s');
    } else {
        kAlt = currentPosition.alt;
        currentSpeed = currentPosition.spd;
    }

    // Mise à jour DOM Rapide
    if ($('lat-val')) $('lat-val').textContent = dataOrDefault(currentPosition.lat, 6, ' °');
    if ($('lon-val')) $('lon-val').textContent = dataOrDefault(currentPosition.lon, 6, ' °');
    if ($('speed-stable')) $('speed-stable').textContent = dataOrDefault(currentSpeed * KMH_MS, 1, ' km/h');
    if ($('acc-horiz')) $('acc-horiz').textContent = dataOrDefault(currentPosition.acc, 1, ' m');
    if ($('vitesse-max-session')) $('vitesse-max-session').textContent = dataOrDefault(maxSpeed * KMH_MS, 1, ' km/h');
    
    // Mise à jour Carte
    updateMap(currentPosition.lat, currentPosition.lon, currentPosition.acc);
    
    // Physique
    updateGravityWGS84(currentPosition.lat, kAlt);

    // Météo (Mise à jour immédiate de la vitesse du son si les données sont prêtes)
    const { mach, q, F_coriolis, speedOfSound } = calculateAerodynamics(currentSpeed);
    const { gamma, beta, timeDilation, E, E0 } = calculateRelativity(currentSpeed);

    // Affichage Physique
    if ($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = dataOrDefault(speedOfSound, 2, ' m/s');
    if ($('mach-number')) $('mach-number').textContent = dataOrDefault(mach, 4);
    if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(gamma, 4);
    if ($('time-dilation-v')) $('time-dilation-v').textContent = dataOrDefault(timeDilation, 2, ' ns/j');
    if ($('energy-relativiste')) $('energy-relativiste').textContent = dataOrDefaultExp(E, 4, ' J');
    if ($('energy-mass-repos')) $('energy-mass-repos').textContent = dataOrDefaultExp(E0, 4, ' J');
    if ($('dynamic-pressure')) $('dynamic-pressure').textContent = dataOrDefault(q, 2, ' Pa');
    if ($('force-coriolis')) $('force-coriolis').textContent = dataOrDefault(F_coriolis, 4, ' N');
    if ($('vitesse-lumiere-perc')) $('vitesse-lumiere-perc').textContent = dataOrDefault(beta * 100, 2, ' %');
}

function initGPS() {
    if (navigator.geolocation) {
        if (gpsWatchId) navigator.geolocation.clearWatch(gpsWatchId);
        if ($('gnss-status')) $('gnss-status').textContent = 'Recherche... 🟡';

        gpsWatchId = navigator.geolocation.watchPosition(
            onPositionUpdate,
            (err) => { if ($('gnss-status')) $('gnss-status').textContent = `Erreur GPS (${err.code}) 🔴`; },
            { enableHighAccuracy: true, maximumAge: 0, timeout: 5000 }
        );
    } else {
        if ($('gnss-status')) $('gnss-status').textContent = 'Non supporté ❌';
    }
}


// --- 7. GESTION DES CONTRÔLES UTILISATEUR ---

function toggleNightMode() {
    document.body.classList.toggle('dark-mode');
    const isDark = document.body.classList.contains('dark-mode');
    const btn = $('night-mode-btn') || $('toggle-mode-btn'); 
    if (btn) btn.innerHTML = isDark ? 'Mode Jour' : 'Mode Nuit';
}

function resetDistance() {
    totalDistance = 0.0;
    if ($('distance-total-3d')) $('distance-total-3d').textContent = dataOrDefault(totalDistance / 1000, 3, ' km');
    if ($('distance-mvt')) $('distance-mvt').textContent = dataOrDefault(totalDistance, 0, ' m');
}

function resetMaxSpeed() {
    maxSpeed = 0.0;
    if ($('vitesse-max-session')) $('vitesse-max-session').textContent = dataOrDefault(maxSpeed * KMH_MS, 1, ' km/h');
}

function toggleEmergencyStop() {
    isEmergencyStopped = !isEmergencyStopped;
    const btn = $('emergency-stop-btn');
    if (btn) btn.innerHTML = isEmergencyStopped 
        ? '▶️ MARCHE GPS' 
        : '🛑 Arrêt d\'urgence: INACTIF 🟢';

    if (isEmergencyStopped) {
        if (gpsWatchId) navigator.geolocation.clearWatch(gpsWatchId);
        if ($('gnss-status')) $('gnss-status').textContent = 'STOP D\'URGENCE 🔴';
        if ($('ekf-status')) $('ekf-status').textContent = 'HALT 🔴';
    } else {
        initGPS();
        if ($('ekf-status')) $('ekf-status').textContent = 'Initialisé 🟢';
    }
}

function fullReset() {
    if (!confirm("⚠️ CONFIRMATION : Êtes-vous sûr de vouloir TOUT RÉINITIALISER ?")) return;
    
    resetDistance();
    resetMaxSpeed();
    isEmergencyStopped = false;
    sessionStartTime = Date.now();
    currentMovementTime = 0;
    movementStartTime = null;

    if (ukf && typeof ukf.reset === 'function') ukf.reset(); 
    
    if (gpsWatchId) navigator.geolocation.clearWatch(gpsWatchId);
    initGPS();
    
    if ($('emergency-stop-btn')) $('emergency-stop-btn').innerHTML = '🛑 Arrêt d\'urgence: INACTIF 🟢';
    if ($('ekf-status')) $('ekf-status').textContent = 'Réinitialisé ⚪';
}


// --- 8. BOUCLE LENTE (Astro & Météo) ---

function slowLoop() {
    if (isEmergencyStopped) return;

    // 1. Horloge NTP/Système
    const now = getCDate(lServH, lLocH);
    if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString('fr-FR');
    if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');

    // 2. Astro (lib/astro.js)
    if (typeof getAstroData === 'function') {
        const astro = getAstroData(now, currentPosition.lat, currentPosition.lon);
        if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(astro.sun.altitude * R2D, 1, ' °');
        if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(astro.sun.azimuth * R2D, 1, ' °');
        if ($('moon-phase-name')) $('moon-phase-name').textContent = astro.moon.phaseName;
        if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(astro.moon.fraction * 100, 1, ' %');
        if ($('date-display-astro')) $('date-display-astro').textContent = now.toLocaleDateString('fr-FR');
    }

    // 3. Météo
    updateWeather();
}

async function updateWeather() {
    if (Date.now() - lastWeatherData.timestamp < 300000) return;

    try {
        if ($('statut-meteo')) $('statut-meteo').textContent = 'Mise à jour... 🟡';
        const res = await fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${currentPosition.lat}&lon=${currentPosition.lon}`);
        if (!res.ok) throw new Error("API Error");
        const data = await res.json();
        
        lastWeatherData = {
            tempC: data.tempC,
            pressure_hPa: data.pressure_hPa,
            humidity_perc: data.humidity_perc,
            tempK: data.tempC + 273.15,
            timestamp: Date.now()
        };

        const p_Pa = lastWeatherData.pressure_hPa * 100;
        lastWeatherData.air_density = getAirDensity(p_Pa, lastWeatherData.tempK, lastWeatherData.humidity_perc/100);
        lastWeatherData.speed_of_sound = calculateAerodynamics(0).speedOfSound; // Recalcul de Cs
        updateWeatherDOM();
        if ($('statut-meteo')) $('statut-meteo').textContent = 'Actif 🟢';

    } catch (e) {
        // Fallback ISA
        updateWeatherDOM(lastWeatherData, true);
        if ($('statut-meteo')) $('statut-meteo').textContent = 'Défaut (ISA) 🟡';
    }
}

function updateWeatherDOM() {
    const d = lastWeatherData;
    if ($('air-temp-c')) $('air-temp-c').textContent = dataOrDefault(d.tempC, 1) + ' °C';
    if ($('pressure-hpa')) $('pressure-hpa').textContent = dataOrDefault(d.pressure_hPa, 0) + ' hPa';
    if ($('humidity-perc')) $('humidity-perc').textContent = dataOrDefault(d.humidity_perc, 0) + ' %';
    if ($('air-density')) $('air-density').textContent = dataOrDefault(d.air_density, 3) + ' kg/m³';
}


// --- 9. INITIALISATION FINALE (window.onload) ---

window.addEventListener('load', () => {

    // 1. Initialiser UKF (Vérification critique des dépendances)
    if (typeof ProfessionalUKF !== 'undefined' && typeof math !== 'undefined') {
        try {
            ukf = new ProfessionalUKF();
            if ($('ekf-status')) $('ekf-status').textContent = 'Initialisé 🟢';
        } catch (e) {
            if ($('ekf-status')) $('ekf-status').textContent = 'Erreur Lib 🔴';
        }
    } else {
        if ($('ekf-status')) $('ekf-status').textContent = 'DÉSACTIVÉ 🔴';
    }
    
    // 2. Initialiser la Carte (Leaflet & Turf)
    initMap();
    if (typeof turf === 'undefined') console.error("Turf.js n'est pas chargé, la distance 3D sera limitée.");

    // 3. Initialisation de l'heure et des données par défaut
    syncTime();
    updateWeatherDOM();
    updateCelestialBody(currentCelestialBody, kAlt);
    if ($('mass-display')) $('mass-display').textContent = dataOrDefault(currentMass, 3, ' kg');

    // 4. Lancer les boucles
    setInterval(slowLoop, DOM_SLOW_UPDATE_MS);
    setInterval(fastLoop, DOM_FAST_UPDATE_MS);
    
    // 5. Démarrer GPS
    initGPS();

    // 6. Listeners d'événements CRITIQUES
    
    // Contrôles Système
    if ($('gps-toggle-btn')) $('gps-toggle-btn').addEventListener('click', toggleGpsPause); // Déjà défini, mais rappelé
    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', toggleEmergencyStop);
    if ($('full-reset-btn')) $('full-reset-btn').addEventListener('click', fullReset);
    if ($('capture-data-btn')) $('capture-data-btn').addEventListener('click', captureData);
    if ($('night-mode-btn')) $('night-mode-btn').addEventListener('click', toggleNightMode);

    // Réinitialisation des Mesures
    if ($('reset-vmax-btn')) $('reset-vmax-btn').addEventListener('click', resetMaxSpeed);
    if ($('reset-distance-btn')) $('reset-distance-btn').addEventListener('click', resetDistance);
    
    // Activation IMU
    const activateButton = document.getElementById('activate-sensors-btn');
    if (activateButton) activateButton.addEventListener('click', activateDeviceMotion);

    // Physique / Environnement
    if ($('celestial-body-select')) $('celestial-body-select').addEventListener('change', (e) => {
        currentCelestialBody = e.target.value;
        updateCelestialBody(currentCelestialBody, kAlt);
    });
    if ($('mass-input')) $('mass-input').addEventListener('input', (e) => {
        currentMass = parseFloat(e.target.value) || 70.0;
        if ($('mass-display')) $('mass-display').textContent = dataOrDefault(currentMass, 3, ' kg');
    });

    // Mode Nether
    if ($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', () => {
        netherMode = !netherMode;
        $('nether-toggle-btn').textContent = `Mode Nether: ${netherMode ? 'ACTIVÉ (1:8)' : 'DÉSACTIVÉ (1:1)'}`;
        if ($('nether-mode-ratio')) $('nether-mode-ratio').textContent = netherMode ? '1:8' : '1:1';
    });
});
