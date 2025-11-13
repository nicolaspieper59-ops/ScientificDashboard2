// =================================================================
// BLOC 1/3 : ekf_logic.js
// Constantes de base, filtres EKF (Vitesse/Altitude) et fonctions de calcul physique/mathématique.
// =================================================================

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const R_E_BASE = 6371000;   // Rayon terrestre moyen (m)
const KMH_MS = 3.6;         // Conversion m/s vers km/h
const C_S = 343;            // Vitesse du son (m/s)
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const R_AIR = 287.058;      // Constante spécifique de l'air sec (J/kg·K)

// --- PARAMÈTRES DU FILTRE DE KALMAN (VITESSE) ---
const Q_NOISE = 0.1;        // Bruit de processus
const R_MIN = 0.01;         // Bruit de mesure minimum
const R_MAX = 500.0;        // Bruit de mesure maximum
const MAX_ACC = 200;        // Précision max (m) avant "Estimation Seule"
const MIN_SPD = 0.05;       // Vitesse minimale "détectée"
let P_KF = Q_NOISE;         // Covariance de l'erreur initiale

// --- PARAMÈTRES DU FILTRE DE KALMAN (ALTITUDE) ---
const Q_ALT = 0.5;          // Bruit de processus pour l'altitude
const R_ALT_BASE = 50.0;    // Bruit de mesure de base pour l'altitude (m)
let P_ALT_KF = R_ALT_BASE;  // Covariance de l'erreur initiale pour l'altitude

// --- ÉTAT GLOBAL DU FILTRE ET DU SYSTÈME ---
let kAlt = 0;               // Altitude estimée par EKF
let kSpd = 0;               // Vitesse estimée par EKF
let kLat = 0, kLon = 0;     // Position estimée par EKF

// Fonction utilitaire pour récupérer un élément du DOM
const $ = (id) => document.getElementById(id);

// Calcule la force de Coriolis pour la vitesse estimée
function calculateCoriolisForce(latRad, speed) {
    return 2 * speed * OMEGA_EARTH * Math.sin(latRad);
}

// Modèle Atmosphérique (Approximation de la formule de l'altitude)
function calculatePressureAtAltitude(h, p_0 = 101325, t_0 = 288.15) {
    const L = 0.0065; // Taux de gradient de température (K/m)
    const G_0 = 9.80665; // Accélération de la gravité à la surface (m/s²)
    const P_Pa = p_0 * Math.pow(1 - (L * h) / t_0, (G_0 / (R_AIR * L)));
    return P_Pa / 100; // Retourne en hPa
}

// Calcule le point de rosée (approximation simple)
function calculateDewPoint(tempC, humidity_perc) {
    const a = 17.27;
    const b = 237.7;
    const alpha = ((a * tempC) / (b + tempC)) + Math.log(humidity_perc / 100.0);
    return (b * alpha) / (a - alpha);
}

// Calcule la densité de l'air (kg/m³)
function calculateAirDensity(P_Pa, T_K) {
    return P_Pa / (R_AIR * T_K);
}

// Filtre de Kalman (Vitesse)
function kalmanFilterSpeed(z, dt, acc, latRad) {
    // Prediction
    let P_predict = P_KF + Q_NOISE;
    let x_predict = kSpd + acc * dt;

    // Mise à jour (Correction)
    let K = P_predict / (P_predict + R_MIN); // Gain de Kalman. R est minimum si la réception GPS est bonne.
    let x_update = x_predict + K * (z - x_predict);
    P_KF = (1 - K) * P_predict;

    // Mise à jour de la vitesse globale
    kSpd = x_update;

    // Mise à jour de la Force de Coriolis
    const coriolis_force = calculateCoriolisForce(latRad, kSpd);

    return {
        speed: kSpd,
        P: P_KF,
        K: K,
        Coriolis: coriolis_force
    };
}

// Filtre de Kalman (Altitude)
function kalmanFilterAlt(z_gps, z_baro, dt) {
    // Prediction
    let P_alt_predict = P_ALT_KF + Q_ALT;
    let x_alt_predict = kAlt; // L'altitude est considérée comme constante sans accélération verticale

    // Mise à jour (Correction)
    // On combine la mesure GPS (z_gps) et la mesure barométrique (z_baro) pour l'innovation.
    // L'innovation est pondérée par leur bruit de mesure respectif (R).
    const R_GPS = 5.0; // Bruit de mesure GPS (fixe)
    const R_BARO = R_ALT_BASE; // Bruit de mesure barométrique

    // Utilisation de la mesure GPS
    let K_gps = P_alt_predict / (P_alt_predict + R_GPS);
    let alt_temp = x_alt_predict + K_gps * (z_gps - x_alt_predict);
    let P_temp = (1 - K_gps) * P_alt_predict;

    // Utilisation de la mesure Barométrique (si présente)
    if (z_baro !== null) {
        let K_baro = P_temp / (P_temp + R_BARO);
        kAlt = alt_temp + K_baro * (z_baro - alt_temp);
        P_ALT_KF = (1 - K_baro) * P_temp;
    } else {
        kAlt = alt_temp;
        P_ALT_KF = P_temp;
    }

    return {
        altitude: kAlt,
        P_alt: P_ALT_KF,
        K_gps: K_gps
    };
        }
// =================================================================
// BLOC 2/3 : map_handler.js
// Initialisation de la carte Leaflet et gestion des marqueurs/trace GPS.
// =================================================================

let map = null;
let marker = null;
let polyline = null;
let isFirstPos = true;
let trackPoints = [];
const TILE_URL = 'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png';

// Initialisation de la carte
function initMap(initialLat = 0, initialLon = 0) {
    if (map) map.remove();

    map = L.map('map').setView([initialLat, initialLon], 10);

    L.tileLayer(TILE_URL, {
        maxZoom: 19,
        attribution: '&copy; <a href="http://www.openstreetmap.org/copyright">OpenStreetMap</a>'
    }).addTo(map);

    marker = L.marker([initialLat, initialLon]).addTo(map);
    polyline = L.polyline([], {color: '#3498db', weight: 3}).addTo(map);
}

// Mise à jour de la position sur la carte
function updateMap(lat, lon, speed) {
    if (!map) return;

    const newLatLng = [lat, lon];
    marker.setLatLng(newLatLng);

    // Ajout du nouveau point au tracé si la position change significativement ou si c'est le premier point
    if (isFirstPos || trackPoints.length === 0 || L.latLng(trackPoints[trackPoints.length - 1]).distanceTo(newLatLng) > 1) {
        trackPoints.push(newLatLng);
        polyline.setLatLngs(trackPoints);
    }

    // Centrage de la carte sur la position actuelle (uniquement au premier point ou si l'utilisateur n'a pas bougé la carte)
    if (isFirstPos) {
        map.setView(newLatLng, 15);
        isFirstPos = false;
    }

    // Mise à jour du statut dans le DOM
    if ($('gps-status')) $('gps-status').textContent = 'Actif';
}

// Réinitialisation de la carte
function resetMap() {
    isFirstPos = true;
    trackPoints = [];
    if (polyline) polyline.setLatLngs([]);
    if (marker) marker.setLatLng([0, 0]);
    if (map) map.setView([0, 0], 10);
    if ($('gps-status')) $('gps-status').textContent = 'Arrêté';
}
// =================================================================
// BLOC 3/3 : main_logic.js
// Gestion des entrées GNSS (via WebSocket), appels API, et mise à jour du DOM.
// =================================================================

// --- CLÉS D'API & PROXY VERCEL ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";
const DOM_SLOW_UPDATE_MS = 2000;
const GPS_DATA_INTERVAL_MS = 1000;

// --- ÉTAT GLOBAL DU SYSTÈME ---
let currentInterval = null;
let lastWeatherData = null;
let lastP_hPa = 1013.25; // Pression par défaut
let lastT_K = 288.15;    // Température par défaut (15°C)
let lastH_perc = 0.6;    // Humidité par défaut (60%)

// Fonction pour récupérer l'heure du serveur
async function getServerTime() {
    try {
        const response = await fetch(SERVER_TIME_ENDPOINT);
        if (!response.ok) throw new Error("Erreur NTP");
        const data = await response.json();
        return new Date(data.utc_datetime);
    } catch (error) {
        console.error("Erreur de synchronisation NTP:", error);
        return null;
    }
}

// Fonction utilitaire pour calculer le temps local à partir du décalage
function getCDate(servTime, locOffsetMs) {
    if (!servTime) return null;
    return new Date(servTime.getTime() + locOffsetMs);
}

// Fonction pour récupérer les données météo
async function getWeatherData(lat, lon) {
    try {
        const endpoint = `${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`;
        const response = await fetch(endpoint);
        if (!response.ok) throw new Error("Erreur API Météo");
        const data = await response.json();
        data.tempK = data.tempC + 273.15;
        data.air_density = calculateAirDensity(data.pressure_hPa * 100, data.tempK);
        data.dew_point = calculateDewPoint(data.tempC, data.humidity_perc);
        return data;
    } catch (error) {
        console.error("Erreur de récupération météo:", error);
        return null;
    }
}

// Fonction de mise à jour rapide (appelée à chaque donnée GPS)
function updateFastData(data) {
    // 1. Dégager les données GPS de base
    const timestamp = data.ts;
    const lat = data.latitude;
    const lon = data.longitude;
    const gpsAlt = data.altitude_m;
    const gpsSpd = data.speed_ms;
    const gpsAcc = data.acceleration_m_s2;

    // 2. Traitement EKF (Vitesse & Altitude)
    const latRad = lat * D2R;
    const filteredSpeedData = kalmanFilterSpeed(gpsSpd, GPS_DATA_INTERVAL_MS / 1000, gpsAcc, latRad);
    const filteredAltData = kalmanFilterAlt(gpsAlt, null, GPS_DATA_INTERVAL_MS / 1000); // Pas d'alt. baro ici.

    const kSpdKMH = filteredSpeedData.speed * KMH_MS;

    // 3. Mise à jour de la Carte
    updateMap(lat, lon, kSpd);

    // 4. Mise à jour du DOM (Vitesse & GPS)
    if ($('latitude-display')) $('latitude-display').textContent = lat.toFixed(6) + ' °';
    if ($('longitude-display')) $('longitude-display').textContent = lon.toFixed(6) + ' °';
    if ($('altitude-display')) $('altitude-display').textContent = filteredAltData.altitude.toFixed(1) + ' m';
    if ($('speed-display')) $('speed-display').textContent = kSpdKMH.toFixed(1) + ' km/h';
    if ($('speed-ms-display')) $('speed-ms-display').textContent = filteredSpeedData.speed.toFixed(2) + ' m/s';

    // 5. Mise à jour du DOM (EKF & Physique)
    if ($('kalman-uncert')) $('kalman-uncert').textContent = filteredSpeedData.P.toFixed(3);
    if ($('alt-uncertainty')) $('alt-uncertainty').textContent = filteredAltData.P_alt.toFixed(1) + ' m';
    if ($('coriolis-force')) $('coriolis-force').textContent = filteredSpeedData.Coriolis.toFixed(5) + ' N';
    if ($('accel-long')) $('accel-long').textContent = gpsAcc.toFixed(2) + ' m/s²';

    // 6. Mise à jour du DOM (Air)
    // On utilise la dernière valeur météo connue pour les calculs physiques
    if (lastWeatherData) {
        const h = filteredAltData.altitude;
        const P_Pa = lastWeatherData.pressure_hPa * 100;
        const T_K = lastWeatherData.tempK;
        const speed_ms = filteredSpeedData.speed;
        const air_density = lastWeatherData.air_density;

        // Pression Dynamique q = 0.5 * rho * V²
        const q_pa = 0.5 * air_density * speed_ms * speed_ms;
        // Approximation de la Force de Traînée (avec un C_D*A arbitraire de 0.5)
        const C_D_A = 0.5;
        const drag_force = q_pa * C_D_A;

        if ($('dynamic-pressure')) $('dynamic-pressure').textContent = q_pa.toFixed(2) + ' Pa';
        if ($('drag-force')) $('drag-force').textContent = drag_force.toFixed(2) + ' N';

        // Gravité locale (Approximation simple g = 9.80665 * (1 - 2*h/R_E))
        const g_local = 9.80665 * (1 - 2 * h / R_E_BASE);
        if ($('gravity-local')) $('gravity-local').textContent = g_local.toFixed(4) + ' m/s²';
    }
}

// Fonction de démarrage
async function startMonitoring() {
    // 1. Récupérer l'heure du serveur pour la synchronisation NTP
    const lServH = await getServerTime();
    const lLocH = lServH ? (new Date()).getTime() - lServH.getTime() : 0;
    if (!lServH) {
        if ($('local-time')) $('local-time').textContent = 'SYNCHRO ÉCHOUÉE';
    }

    // 2. Initialiser la carte avec une position par défaut
    initMap(kLat, kLon);

    // 3. Simuler la réception des données GPS toutes les 1000 ms (1 Hz)
    // REMARQUE: Ceci serait un WebSocket ou un appel série dans une application réelle
    currentInterval = setInterval(() => {
        // Simulation de données (à remplacer par la logique de réception réelle)
        const mockData = {
            ts: Date.now(),
            latitude: kLat + (Math.random() - 0.5) * 0.00002, // Petite dérive pour le test
            longitude: kLon + (Math.random() - 0.5) * 0.00002,
            altitude_m: kAlt + (Math.random() - 0.5) * 0.1,
            speed_ms: Math.abs(kSpd) + (Math.random() - 0.5) * 0.05, // Vitesse simulée
            acceleration_m_s2: (Math.random() - 0.5) * 0.01 // Accélération simulée
        };
        kLat = mockData.latitude;
        kLon = mockData.longitude;

        updateFastData(mockData);

    }, GPS_DATA_INTERVAL_MS);

    // 4. Fonction de mise à jour lente (Météo/API)
    setInterval(() => {
        // Appeler l'API météo seulement si la position est connue
        if (kLat !== 0 && kLon !== 0) {
            getWeatherData(kLat, kLon).then((data) => {
                if (data) {
                    lastWeatherData = data;
                    
                    // Met à jour les valeurs pour le filtre EKF
                    lastP_hPa = data.pressure_hPa;
                    lastT_K = data.tempK;
                    lastH_perc = data.humidity_perc / 100.0;
                    
                    // Met à jour le DOM météo
                    if ($('temp-air-2')) $('temp-air-2').textContent = `${data.tempC.toFixed(1)} °C`;
                    if ($('pressure-2')) $('pressure-2').textContent = `${data.pressure_hPa.toFixed(0)} hPa`;
                    if ($('humidity-2')) $('humidity-2').textContent = `${data.humidity_perc} %`;
                    if ($('air-density')) $('air-density').textContent = `${data.air_density.toFixed(3)} kg/m³`;
                    if ($('dew-point')) $('dew-point').textContent = `${data.dew_point.toFixed(1)} °C`;
                }
            });
        }
        
        // Met à jour l'horloge locale (NTP)
        const now = getCDate(lServH, lLocH);
        if (now) {
            if ($('local-time') && !$('local-time').textContent.includes('SYNCHRO ÉCHOUÉE')) {
                $('local-time').textContent = now.toLocaleTimeString('fr-FR');
            }
            if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
        }
        
    }, DOM_SLOW_UPDATE_MS); 
}

// Démarrage de l'application une fois que le DOM est chargé
document.addEventListener('DOMContentLoaded', () => {
    // Écouteur pour le bouton de démarrage (si présent)
    const startButton = $('start-button');
    if (startButton) {
        startButton.addEventListener('click', () => {
            // Logique de démarrage (par exemple: demander la position initiale, etc.)
            startMonitoring();
            startButton.textContent = 'MONITORING ACTIF';
            startButton.disabled = true;
        });
    } else {
        // Démarrage automatique si pas de bouton de contrôle
        startMonitoring();
    }
});
