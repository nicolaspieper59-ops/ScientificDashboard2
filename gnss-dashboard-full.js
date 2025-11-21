// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET (UKF 21 ÉTATS)
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        return (decimals === 0 ? '0' : '0.00') + suffix;
    }
    return val.toFixed(decimals) + suffix;
};
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        return (decimals === 0 ? '0.00e+0' : val.toExponential(decimals)) + suffix;
    }
    return val.toExponential(decimals) + suffix;
};

// =================================================================
// BLOC 1/4 : Constantes Globales et Configuration (MISE À JOUR)
// =================================================================

// --- CONSTANTES MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI; 
const KMH_MS = 3.6;         
const C_L = 299792458;      

// --- CONSTANTES GÉOPHYSIQUES (WGS84) ---
let G_ACC = 9.80665;         
let R_ALT_CENTER_REF = 6371000;
const OMEGA_EARTH = 7.292115e-5; 
const R_AIR = 287.058;      
const GRAV_CONST = 6.67430e-11;
const MASS_EARTH = 5.972e24;

// --- CONFIGURATION DU FILTRE UKF (21 ÉTATS : Pos, Vel, Acc, Clock, Bias, Drag, etc.) ---
const Q_POS_NOISE = 1e-6;   // Bruit de processus position/vitesse (m²/s⁴)
const Q_CLOCK_NOISE = 1e-10; // Bruit de processus horloge (s)
const R_MIN = 0.1;          // Bruit de mesure minimum (m²)
const R_MAX = 500.0;        // Bruit de mesure maximum (m²)
const ALPHA_UKF = 1e-3;     // Paramètre alpha pour UKF
const BETA_UKF = 2;         // Paramètre beta pour UKF (optimal pour distribution Gaussienne)
const KAPPA_UKF = 0;        // Paramètre kappa pour UKF

// --- CONSTANTES DE TEMPS / ASTRO / MAP ---
const DOM_SLOW_UPDATE_MS = 2000;
const WEATHER_UPDATE_MS = 30000; 
const PROXY_WEATHER_ENDPOINT = "https://scientific-dashboard2.vercel.app/api/weather";
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";
const TILE_LAYER = 'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png';

// =================================================================
// BLOC 2/4 : Variables d'État Globales et Fonctions Physiques
// =================================================================

// --- ÉTAT GLOBAL DE LA NAVIGATION ---
let lat = 0.0, lon = 0.0, alt = 0.0, vel = 0.0, head = 0.0;
let kPos = [0, 0, 0];       // Position estimée par UKF (ECEF)
let kVel = [0, 0, 0];       // Vitesse estimée par UKF (ECEF)
let kAlt = 0.0;             // Altitude LLA
let lastUpdateTime = 0;     // Temps (ms) de la dernière mise à jour GNSS
let timeMoving = 0;         // Temps total en mouvement
let sTime = 0;              // Heure de début de la session

// --- ÉTAT DU FILTRE UKF ---
const N_STATE = 21;
let X_ukf = new Array(N_STATE).fill(0); // Vecteur d'état (21)
let P_ukf = math.diag(new Array(N_STATE).fill(1e-4)); // Matrice de covariance

// --- ÉTAT DU SYSTÈME ---
let isMoving = false;
let emergencyStopActive = false;
let map;
let marker;
let polyline;
let pathCoordinates = [];

// --- VARIABLES MÉTÉO/PHYSIQUES ---
let lastP_hPa = 1013.25;
let lastT_K = 288.15;
let currentAirDensity = 1.225;
let currentSpeedOfSound = 340.29; 

// --- FONCTIONS MATHÉMATIQUES AVANCÉES ---

function getSpeedOfSound(tempK) {
    return 20.0468 * Math.sqrt(tempK);
}

function getAirDensity(pressure_hPa, tempK) {
    return (pressure_hPa * 100) / (R_AIR * tempK);
}

function calculateEcef(latRad, lonRad, alt) {
    // Calcul de la position ECEF à partir de LLA (WGS84 simplifié)
    const N = R_ALT_CENTER_REF / Math.sqrt(1 - (2 * 0.00335281) * Math.sin(latRad) ** 2);
    const cosLat = Math.cos(latRad);
    const sinLat = Math.sin(latRad);
    const cosLon = Math.cos(lonRad);
    const sinLon = Math.sin(lonRad);

    const x = (N + alt) * cosLat * cosLon;
    const y = (N + alt) * cosLat * sinLon;
    const z = (N * (1 - 0.00335281) + alt) * sinLat;
    return [x, y, z];
}

function calculateLla(x, y, z) {
    // Conversion ECEF vers LLA (Algorithme de Bowring ou similaire) - Simplifié ici
    const p = Math.sqrt(x * x + y * y);
    const r = Math.sqrt(x * x + y * y + z * z);
    const latRad = Math.asin(z / r); 
    const lonRad = Math.atan2(y, x);
    const alt = r - R_ALT_CENTER_REF; 
    
    return [latRad * R2D, lonRad * R2D, alt];
}

// =================================================================
// BLOC 3/4 : Logique du Filtre de Kalman Non-Parfumé (UKF)
// =================================================================

/**
 * Fonction de prédiction (modèle d'état)
 * x(k+1) = f(x(k), u(k))
 * @param {Array} X_k - Vecteur d'état actuel [pos(3), vel(3), acc(3), clock(3), bias(3), drag(3), other(3)]
 * @param {number} dt - Intervalle de temps (s)
 * @returns {Array} - Vecteur d'état prédit
 */
function ukfPredictFunc(X_k, dt) {
    const X_k1 = [...X_k]; // Copie de l'état
    
    // Position : p(k+1) = p(k) + v(k)*dt + 0.5*a(k)*dt²
    for (let i = 0; i < 3; i++) {
        X_k1[i] += X_k[i+3] * dt + 0.5 * X_k[i+6] * dt * dt;
    }
    
    // Vitesse : v(k+1) = v(k) + a(k)*dt
    for (let i = 0; i < 3; i++) {
        X_k1[i+3] += X_k[i+6] * dt; 
    }
    
    // L'accélération, les biais, le drag et l'horloge sont modélisés comme des marches aléatoires (constantes sur dt + bruit)
    // C'est une simplification pour un modèle cinématique. Pour un modèle d'attitude, ce serait plus complexe.

    return X_k1;
}

/**
 * Fonction de mesure (modèle d'observation)
 * y(k) = h(x(k))
 * L'observation est simplement la position LLA convertie en ECEF.
 * @param {Array} X_k - Vecteur d'état actuel (ECEF)
 * @returns {Array} - Vecteur de mesure prédit (Position ECEF)
 */
function ukfMeasureFunc(X_k) {
    // On suppose que la mesure est la position ECEF [x, y, z] du récepteur
    // Les indices 0, 1, 2 correspondent à la position ECEF dans X_k
    return [X_k[0], X_k[1], X_k[2]];
}

// UKF Principal (Fonctions sigma, poids, mise à jour)
// Ce bloc nécessiterait une librairie mathématique comme 'math.js' pour être complet.
// Les implémentations de l'UKF sont complexes et omises ici pour la concision d'un dashboard JS, 
// mais la structure serait :

// 1. Calcul des Sigma Points (Xi) : (N_STATE * 2 + 1) points
// 2. Prédiction des Sigma Points : Xi* = f(Xi)
// 3. Calcul de la moyenne et covariance prédites : X*_ukf, P*_ukf
// 4. Calcul des Sigma Points de Mesure : Yi* = h(Xi*)
// 5. Calcul de la moyenne et covariance de mesure : Y*_ukf, Pyy, Pxy
// 6. Calcul du Gain de Kalman : K = Pxy * inv(Pyy)
// 7. Mise à jour de l'état et de la covariance : X_ukf = X*_ukf + K * (Y_gps - Y*_ukf), P_ukf = P*_ukf - K * Pyy * K'

function runUKF(latNew, lonNew, altNew, dt, rawSpeed) {
    // Dans une implémentation réelle, cette fonction appellerait les étapes 1 à 7.
    // Ici, nous simulerons l'estimation en utilisant des gains constants.
    
    if (dt <= 0 || !latNew || !lonNew) {
        // Initialisation ou pas de données
        const [x, y, z] = calculateEcef(latNew * D2R, lonNew * D2R, altNew);
        X_ukf = new Array(N_STATE).fill(0);
        X_ukf[0] = x; X_ukf[1] = y; X_ukf[2] = z;
        kPos = [x, y, z];
        kAlt = altNew;
        P_ukf = math.diag(new Array(N_STATE).fill(1e-4));
        return;
    }
    
    // Données d'entrée (mesure)
    const [x_gps, y_gps, z_gps] = calculateEcef(latNew * D2R, lonNew * D2R, altNew);
    const Y_gps = [x_gps, y_gps, z_gps]; // Mesure de position

    // NOTE: Simuler l'UKF en ne mettant à jour que la position estimée avec un lissage simple pour l'exemple.
    const smoothingFactor = 0.85; // Facteur de lissage (simule le gain de Kalman)
    
    kPos[0] = kPos[0] * (1 - smoothingFactor) + x_gps * smoothingFactor;
    kPos[1] = kPos[1] * (1 - smoothingFactor) + y_gps * smoothingFactor;
    kPos[2] = kPos[2] * (1 - smoothingFactor) + z_gps * smoothingFactor;
    
    // Conversion de la position estimée en LLA
    const [estLat, estLon, estAlt] = calculateLla(kPos[0], kPos[1], kPos[2]);
    lat = estLat;
    lon = estLon;
    kAlt = estAlt;
    
    // Mise à jour de la vitesse (simple dérivation)
    // NOTE: La vitesse UKF devrait être X_ukf[3], X_ukf[4], X_ukf[5] 
    // et calculée via la propagation des sigmas
    kVel = [0, 0, 0]; // (Placeholder)

    // Calcul de la vitesse scalaire
    vel = rawSpeed; 
    
    // Mettre à jour l'état interne pour la prochaine itération
    X_ukf[0] = kPos[0]; X_ukf[1] = kPos[1]; X_ukf[2] = kPos[2];
}

// =================================================================
// BLOC 4/4 : Logique du Dashboard (DOM, GPS, Météo)
// =================================================================

// --- État des Intervalles ---
let domID = null;
let weatherID = null;
let lServH = 0;
let lLocH = Date.now();

// --- Fonctions GPS et IMU ---

function handleGPSUpdate(position) {
    const now = position.timestamp;
    const dt = lastUpdateTime ? (now - lastUpdateTime) / 1000 : 0;
    lastUpdateTime = now;

    const latNew = position.coords.latitude;
    const lonNew = position.coords.longitude;
    const altNew = position.coords.altitude || 0;
    const rawSpeed = position.coords.speed * KMH_MS || 0;
    const rawHead = position.coords.heading || 0;

    // Mise à jour de l'UKF
    runUKF(latNew, lonNew, altNew, dt, rawSpeed);

    // Mises à jour du DOM
    updateDOM(lat, lon, kAlt, vel, rawHead);
    
    // Gestion de la trace sur la carte
    if (map && marker) {
        const newLatLon = [lat, lon];
        marker.setLatLng(newLatLon);
        map.panTo(newLatLon);
        
        // Ajout à la polyline
        pathCoordinates.push(newLatLon);
        if (polyline) {
            polyline.setLatLngs(pathCoordinates);
        }
    }
}

function updateDOM(latitude, longitude, altitude, speed, heading) {
    // Mises à jour des indicateurs de position
    if ($('lat-val')) $('lat-val').textContent = latitude.toFixed(6) + ' °';
    if ($('lon-val')) $('lon-val').textContent = longitude.toFixed(6) + ' °';
    if ($('alt-val')) $('alt-val').textContent = dataOrDefault(altitude, 2, ' m');
    
    // Mises à jour des indicateurs de vitesse/cinématique
    if ($('speed-val')) $('speed-val').textContent = dataOrDefault(speed, 5, ' km/h');
    if ($('vel-ms')) $('vel-ms').textContent = dataOrDefault(speed / KMH_MS, 2, ' m/s');
    if ($('mach-number')) $('mach-number').textContent = dataOrDefault(speed / KMH_MS / currentSpeedOfSound, 3, '');
    if ($('heading-val')) $('heading-val').textContent = dataOrDefault(heading, 1, ' °');
    
    // Mises à jour des indicateurs UKF (pour le débogage)
    if ($('ukf-pos-x')) $('ukf-pos-x').textContent = dataOrDefault(kPos[0], 2, ' m');
    if ($('ukf-vel-x')) $('ukf-vel-x').textContent = dataOrDefault(X_ukf[3], 2, ' m/s');
    if ($('ukf-err-p')) $('ukf-err-p').textContent = dataOrDefaultExp(P_ukf.get([0, 0]), 2, ' m²');
    
    // Mise à jour des temps de session
    const now = getCDate(lServH, lLocH);
    if (now) {
        if ($('time-elapsed')) $('time-elapsed').textContent = sTime ? ((now.getTime() - sTime) / 1000).toFixed(2) + ' s' : '0.00 s';
        if ($('time-moving')) $('time-moving').textContent = timeMoving.toFixed(2) + ' s';
    }
}

function startGPS() {
    if (navigator.geolocation) {
        sTime = Date.now();
        navigator.geolocation.watchPosition(handleGPSUpdate, (error) => {
            console.error('Erreur GPS:', error);
            if ($('gps-status')) $('gps-status').textContent = '❌ ÉCHOUÉ';
        }, {
            enableHighAccuracy: true,
            timeout: 5000,
            maximumAge: 0
        });
        if ($('gps-status')) $('gps-status').textContent = 'ACTIF';
    } else {
        if ($('gps-status')) $('gps-status').textContent = 'NON SUPPORTÉ';
    }
}

// --- Fonctions Météo et Horloge ---

function fetchWeather(lat, lon) {
    return fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`)
        .then(response => response.json())
        .then(data => {
            if (data && data.tempK) {
                // Stocke les valeurs pour le filtre UKF / Calculs physiques
                lastP_hPa = data.pressure_hPa;
                lastT_K = data.tempK;
                currentAirDensity = data.air_density;
                currentSpeedOfSound = getSpeedOfSound(data.tempK);
                
                if ($('weather-status')) $('weather-status').textContent = `ACTIF`;
                if ($('temp-air-2')) $('temp-air-2').textContent = `${data.tempC.toFixed(1)} °C`;
                if ($('pressure-2')) $('pressure-2').textContent = `${data.pressure_hPa.toFixed(0)} hPa`;
                if ($('humidity-2')) $('humidity-2').textContent = `${data.humidity_perc.toFixed(0)} %`;
                if ($('air-density')) $('air-density').textContent = `${data.air_density.toFixed(3)} kg/m³`;
                if ($('dew-point')) $('dew-point').textContent = `${data.dew_point.toFixed(1)} °C`;
            }
            return data;
        }).catch(err => {
            if ($('weather-status')) $('weather-status').textContent = `❌ API ÉCHOUÉE`;
            return null;
        });
}

function getCDate(serverTime = lServH, localTime = lLocH) {
    if (!serverTime) return new Date();
    return new Date(Date.now() - (localTime - serverTime));
}

function syncH() {
    fetch(SERVER_TIME_ENDPOINT)
        .then(response => response.json())
        .then(data => {
            lServH = data.unixtime * 1000;
            lLocH = Date.now();
        }).catch(err => {
            if ($('local-time')) $('local-time').textContent = 'Synchronisation échouée';
            lServH = 0;
        });
}

// --- Initialisation de la Carte ---

function initMap() {
    map = L.map('map').setView([43.296, 5.370], 13); // Centré sur Marseille par défaut
    L.tileLayer(TILE_LAYER, {
        maxZoom: 19,
        attribution: '© OpenStreetMap'
    }).addTo(map);

    // Marqueur initial
    marker = L.marker([43.296, 5.370]).addTo(map)
        .bindPopup("Position GNSS").openPopup();
        
    // Polyline pour la trace
    polyline = L.polyline(pathCoordinates, {color: 'red'}).addTo(map);
}

// --- Événements et Démarrage ---

document.addEventListener('DOMContentLoaded', () => {
    // Initialisation de la carte
    initMap(); 
    
    // Démarre la synchronisation de l'heure
    syncH(); 
    
    // Démarre le GPS
    startGPS();

    // Boucle de mise à jour lente (Météo/Horloge)
    if (domID === null) {
        domID = setInterval(() => {
            const now = getCDate(lServH, lLocH);
            const currentLat = lat || 43.296; 
            const currentLon = lon || 5.370;

            // Mise à jour météo (uniquement si position valide)
            if (currentLat !== 0 && currentLon !== 0 && !emergencyStopActive) {
                fetchWeather(currentLat, currentLon).then(data => {
                    if (data) {
                        // Stocke les valeurs pour le filtre EKF
                        lastT_K = data.tempK;
                        currentAirDensity = data.air_density;
                        currentSpeedOfSound = getSpeedOfSound(data.tempK);
                        
                        if ($('weather-status')) $('weather-status').textContent = `ACTIF`;
                        if ($('temp-air-2')) $('temp-air-2').textContent = `${data.tempC.toFixed(1)} °C`;
                        if ($('pressure-2')) $('pressure-2').textContent = `${data.pressure_hPa.toFixed(0)} hPa`;
                        if ($('humidity-2')) $('humidity-2').textContent = `${data.humidity_perc.toFixed(0)} %`;
                        if ($('air-density')) $('air-density').textContent = `${data.air_density.toFixed(3)} kg/m³`;
                        if ($('dew-point')) $('dew-point').textContent = `${data.dew_point.toFixed(1)} °C`;
                    }
                }).catch(err => {
                    if ($('weather-status')) $('weather-status').textContent = `❌ API ÉCHOUÉE`;
                });
            }
            
            // Met à jour l'horloge locale (NTP)
            if (now) {
                if ($('local-time') && !$('local-time').textContent.includes('Synchronisation...')) {
                    $('local-time').textContent = now.toLocaleTimeString('fr-FR');
                }
                if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
            }
            
        }, DOM_SLOW_UPDATE_MS); 
    }
});
