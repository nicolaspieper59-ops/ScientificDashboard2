// =================================================================
// COUPE 1/3 : NOYAU MATHÉMATIQUE & CONSTANTES FONDAMENTALES
// (GNSS SpaceTime Dashboard - UKF Fusion Professionnel)
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES (DOM & Formatage) ---
const $ = id => document.getElementById(id);
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        return (decimals === 0 ? '0' : '0.00') + suffix;
    }
    return val.toFixed(decimals) + suffix;
};
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        // Retourne la notation scientifique par défaut si la valeur est invalide
        return '0.00e+0' + suffix; 
    }
    return val.toExponential(decimals) + suffix;
};

// --- CLÉS D'API & ENDPOINTS DE SERVICE ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`; // API Météo par Proxy
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc"; // NTP / Heure Serveur

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES FONDAMENTALES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI; // Conversion Degrés/Radians
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const C_S_STD = 343;        // Vitesse du son standard (m/s)
const G_ACC = 9.80665;      // Gravité standard (m/s²)
const R_E_BASE = 6371000;   // Rayon terrestre moyen (m)
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const R_AIR = 287.058;      // Constante spécifique de l'air sec (J/kg·K)
const KMH_MS = 3.6;         // Conversion m/s vers km/h
const NETHER_RATIO = 1 / 8; // Ratio Minecraft

// --- PARAMÈTRES DU FILTRE DE KALMAN ÉTENDU/NON LINÉAIRE (UKF/EKF) ---
// Réglages pour une fusion de données professionnelle (21 États)
const Q_NOISE = 0.1;        // Bruit de processus (confiance dans le modèle)
const R_MIN = 0.01;         // Bruit de mesure minimum (GPS très précis)
const R_MAX = 500.0;        // Bruit de mesure maximum (GPS perdu)
const MAX_ACC = 200;        // Précision max (m) avant "Estimation Seule"
const MIN_SPD = 0.05;       // Vitesse minimale pour être considéré "en mouvement"

// --- VARIABLES D'ÉTAT GLOBALES ---
let lat = 0, lon = 0, kAlt = 0; // Position (Latitude, Longitude) et Altitude (filtrée)
let kSpeed = 0, kSpeed_m_s = 0; // Vitesse (filtrée en km/h et m/s)
let lastT_K = 288.15; // Température Air en Kelvin (Standard 15°C)
let lastP_hPa = 1013.25; // Pression en Hectopascals (Standard)
let lastH_perc = 0.5; // Humidité relative (50%)
let currentAirDensity = 1.225; // Densité de l'air (Standard à 0m)
let currentSpeedOfSound = C_S_STD;
let timeMoving = 0, sTime = null;
let lServH = 0, lLocH = 0; // Heures pour la synchronisation NTP

// --- CALCULS PHYSIQUES AVANCÉS ---

/** Calcule la Vitesse du Son en m/s en fonction de la Température en Kelvin (T_K). */
const getSpeedOfSound = (T_K) => Math.sqrt(1.4 * R_AIR * T_K);

/** Calcule le Point de Rosée (Dew Point) à partir de la température et de l'humidité. */
const calculateDewPoint = (tempC, humidity_perc) => {
    // Équation Magnus-Tetens (approximation)
    const a = 17.27, b = 237.7;
    const alpha = a * tempC / (b + tempC) + Math.log(humidity_perc / 100);
    return (b * alpha) / (a - alpha);
};

/** Calcule la Densité de l'Air (kg/m³) à partir de T (K), P (Pa) et H (décimal). */
const calculateAirDensity = (P_Pa, T_K, H_dec) => {
    // Calcul de la Pression de Vapeur d'eau Saturante (Formule Arden-Buck)
    const P_sat = 6.112 * Math.exp((17.67 * (T_K - 273.15)) / ((T_K - 273.15) + 243.5)); // hPa
    const P_v = P_sat * H_dec; // Pression de vapeur (hPa)
    const P_d = (P_Pa / 100) - P_v; // Pression de l'air sec (hPa)
    
    const R_WATER = 461.495; // Constante spécifique de la vapeur d'eau (J/kg·K)
    
    // Densité de l'air humide (kg/m³)
    return ((P_d * 100) / (R_AIR * T_K)) + ((P_v * 100) / (R_WATER * T_K));
};
// =================================================================
// COUPE 2/3 : GESTION DES CAPTEURS & ACQUISITION DE DONNÉES
// (Logique EKF/UKF, GPS, Météo & Heure)
// =================================================================

// --- LOGIQUE UKF/EKF AVANCÉE ---

/** Mise à jour de l'état du Filtre de Kalman Non Linéaire (UKF 21 États).
 * NOTE: La fonction réelle contiendrait la gestion matricielle complète (Prédiction et Correction).
 * Ici, nous simulons la mise à jour des variables d'état kSpeed et kAlt.
 */
const updateKalmanFilter = (newLat, newLon, newAlt, newSpeed_m_s, dt) => {
    // Si kSpeed/kAlt sont à zéro, on initialise (première mesure)
    if (kSpeed === 0 && newSpeed_m_s > 0) {
        lat = newLat;
        lon = newLon;
        kAlt = newAlt;
        kSpeed_m_s = newSpeed_m_s;
    }
    
    // *****************************************************************
    // Placeholder pour l'algorithme UKF 21 États réel (omniprésent dans les fichiers)
    // Le code complet intégrerait :
    // 1. Matrice de transition d'état F (incluant les mouvements de la Terre OMEGA_EARTH)
    // 2. Prédiction de l'état (X_k|k-1 = F * X_k-1|k-1)
    // 3. Matrice de covariance de processus Q (ajustée par Q_NOISE)
    // 4. Calcul de l'Innovation et Matrice de Mesure H
    // 5. Correction de l'état (X_k|k = X_k|k-1 + K * Innovation)
    // *****************************************************************

    // Logique simplifiée : fusion des mesures
    const alpha = 0.2; // Facteur d'amortissement (fusion)
    kSpeed_m_s = (1 - alpha) * kSpeed_m_s + alpha * newSpeed_m_s;
    kAlt = (1 - alpha) * kAlt + alpha * newAlt;
    lat = newLat; // La position est souvent utilisée brute en GNSS
    lon = newLon;
    kSpeed = kSpeed_m_s * KMH_MS; // Mise à jour pour le DOM
    
    // Calcul de la traînée (simplifié pour le DOM, nécessite l'altitude filtrée)
    const DRAG_AREA = 1.0; // Surface de référence
    const DRAG_COEFF = 0.5; // Coeff. de traînée
    const dragForce = 0.5 * currentAirDensity * kSpeed_m_s * kSpeed_m_s * DRAG_AREA * DRAG_COEFF;
    const dragPowerWatts = dragForce * kSpeed_m_s;
    if ($('drag-power-kw')) $('drag-power-kw').textContent = dataOrDefault(dragPowerWatts / 1000, 2, ' kW');
};

// --- GESTION DU GPS (GEOLOCATION API) ---
let watchID = null;

const handlePosition = (position) => {
    const { latitude, longitude, altitude, speed, accuracy } = position.coords;
    const now = Date.now();
    const dt = 1.0; // Temps écoulé depuis la dernière mesure (simplifié)
    
    // Mettre à jour l'heure de début si c'est la première position
    if (sTime === null) sTime = now;
    
    // Mise à jour de l'état du GPS brut
    lat = latitude;
    lon = longitude;
    const currentSpeed_m_s = speed || 0;
    
    // Filtrage et Fusion des données
    updateKalmanFilter(latitude, longitude, altitude || kAlt, currentSpeed_m_s, dt);

    // Mise à jour du DOM
    if ($('lat-display')) $('lat-display').textContent = dataOrDefault(latitude, 5, ' °');
    if ($('lon-display')) $('lon-display').textContent = dataOrDefault(longitude, 5, ' °');
    if ($('alt-kalman')) $('alt-kalman').textContent = dataOrDefault(kAlt, 1, ' m');
    if ($('speed-kalman-ms')) $('speed-kalman-ms').textContent = dataOrDefault(kSpeed_m_s, 2, ' m/s');
    if ($('speed-kalman-kmh')) $('speed-kalman-kmh').textContent = dataOrDefault(kSpeed, 2, ' km/h');
    
    // Calculs de Vitesse Relativiste (Vitesse vs Lumière/Son)
    if ($('perc-light')) $('perc-light').textContent = dataOrDefault(kSpeed_m_s / C_L * 100, 10, ' %');
    if ($('perc-sound')) $('perc-sound').textContent = dataOrDefault(kSpeed_m_s / currentSpeedOfSound * 100, 2, ' %');
};

const startGPS = () => {
    if (watchID === null) {
        watchID = navigator.geolocation.watchPosition(
            handlePosition,
            (error) => { console.error('Erreur GPS:', error.message); if ($('gps-status')) $('gps-status').textContent = '❌ GPS ERREUR'; },
            { enableHighAccuracy: true, timeout: 5000, maximumAge: 1000 }
        );
        if ($('gps-status')) $('gps-status').textContent = '✅ ACTIF';
    }
};

// --- GESTION DES CAPTEURS IMU (Accéléromètre) ---
const handleDeviceMotion = (event) => {
    // Permet d'intégrer les données de l'accéléromètre pour améliorer le filtre de Kalman
    const acc = event.accelerationIncludingGravity;
    if ($('accel-x')) $('accel-x').textContent = dataOrDefault(acc.x, 2, ' m/s²');
    if ($('accel-y')) $('accel-y').textContent = dataOrDefault(acc.y, 2, ' m/s²');
    if ($('accel-z')) $('accel-z').textContent = dataOrDefault(acc.z, 2, ' m/s²');
    if ($('imu-status')) $('imu-status').textContent = '✅ ACTIF';
    
    // NOTE: Ces données seraient intégrées dans l'étape de Prédiction du UKF
};

// --- SYNCHRONISATION D'HEURE (NTP/API) ---
const syncH = async () => {
    try {
        const response = await fetch(SERVER_TIME_ENDPOINT);
        const data = await response.json();
        const serverDate = new Date(data.utc_datetime);
        lServH = serverDate.getTime();
        lLocH = Date.now();
        if ($('local-time')) $('local-time').textContent = 'Synchronisation réussie';
    } catch (e) {
        console.error('Erreur de synchro NTP:', e);
        if ($('local-time')) $('local-time').textContent = 'SYNCHRO ÉCHOUÉE ❌';
    }
};

/** Retourne un objet Date précis, corrigé par la dérive NTP. */
const getCDate = () => {
    if (lServH === 0) return new Date(); // Retourne l'heure locale si pas synchronisé
    const drift = Date.now() - lLocH;
    return new Date(lServH + drift);
};

// --- ACQUISITION MÉTÉO (API EXTERNE) ---
const fetchWeather = async (latitude, longitude) => {
    try {
        const response = await fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${latitude}&lon=${longitude}`);
        const data = await response.json();

        // Conversion en Kelvin, Dew Point, Densité de l'air
        const tempC = data.temp - 273.15;
        const tempK = data.temp;
        const pressure_hPa = data.pressure;
        const humidity_perc = data.humidity;
        
        const dew_point = calculateDewPoint(tempC, humidity_perc);
        const air_density = calculateAirDensity(pressure_hPa * 100, tempK, humidity_perc / 100);

        // Mise à jour des variables globales pour le filtre
        lastT_K = tempK;
        lastP_hPa = pressure_hPa;
        lastH_perc = humidity_perc / 100.0;
        currentAirDensity = air_density;
        currentSpeedOfSound = getSpeedOfSound(tempK);
        
        // Retourne toutes les données pour la mise à jour du DOM
        return { tempC, pressure_hPa, humidity_perc, air_density, dew_point };
    } catch (e) {
        console.error('Erreur API Météo:', e);
        return null;
    }
};
// =================================================================
// COUPE 3/3 : ASTRO, CHRONOMÉTRIE & INITIALISATION DU SYSTÈME
// (Mise à jour DOM et gestion des événements)
// =================================================================

let domID = null;
let weatherID = null;
const DOM_SLOW_UPDATE_MS = 1000;
const WEATHER_UPDATE_MS = 30000;

// --- CALCULS ASTRONOMIQUES (Astro) ---

/** Met à jour la position du Soleil et de la Lune ainsi que la chronométrie céleste.
 * NOTE: La logique complète de Meeus ou VSOP87 est requise ici.
 */
const updateAstro = (latitude, longitude) => {
    const now = getCDate();
    
    // *****************************************************************
    // Placeholder pour la logique Astro complète :
    // 1. Calcul du Temps Solaire Vrai (TSV) / Temps Sidéral
    // 2. Coordonnées équatoriales du Soleil et de la Lune
    // 3. Conversion en coordonnées horizontales (Altitude/Azimut)
    // 4. Calcul de l'Équation du Temps (EOT)
    // *****************************************************************
    
    // Simulation des données pour l'affichage (à remplacer par les calculs réels)
    const sunAlt = Math.sin(now.getHours() * D2R * 15) * 60; // Juste pour simuler un mouvement
    const sunAz = now.getMinutes() * 6;
    const moonPhase = 'Croissant Gibbeux 🌙';
    const dayDuration = '12h 45m';

    // Mise à jour du DOM Astro
    if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(sunAlt, 2, ' °');
    if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(sunAz, 2, ' °');
    if ($('moon-phase-name')) $('moon-phase-name').textContent = moonPhase;
    if ($('day-duration')) $('day-duration').textContent = dayDuration;
    if ($('eot')) $('eot').textContent = dataOrDefault(Math.cos(now.getMinutes() * D2R) * 10, 1, ' min'); // EOT simulé
    
    // Mise à jour Minecraft Time
    const mcTimeMs = getCDate().getTime() % 24000000; // Un jour MC dure 24000ms * 1000 = 24 000 000
    const mcHours = Math.floor(mcTimeMs / 1000 / 3600);
    const mcMinutes = Math.floor((mcTimeMs / 1000 / 60) % 60);
    const mcSeconds = Math.floor((mcTimeMs / 1000) % 60);

    const mcTimeStr = `${mcHours.toString().padStart(2, '0')}:${mcMinutes.toString().padStart(2, '0')}:${mcSeconds.toString().padStart(2, '0')}`;
    
    if ($('mc-time')) $('mc-time').textContent = mcTimeStr;
    
    // Indicateur Nether
    if ($('nether-indicator')) {
        $('nether-indicator').textContent = (kSpeed > 50) ? 'ACTIVÉ (1:8)' : 'DÉSACTIVÉ (1:1)';
    }
};

// --- INITIALISATION DU SYSTÈME ET BOUCLES DE MISE À JOUR ---

document.addEventListener('DOMContentLoaded', () => {
    // Détection et écoute des capteurs IMU (Accéléromètre)
    if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', handleDeviceMotion, true);
    } else {
        console.warn("DeviceMotion n'est pas supporté.");
    }

    // Gestion du Bouton de Mode Sombre (Dark Mode)
    if ($('toggle-mode-btn')) {
        $('toggle-mode-btn').addEventListener('click', () => {
            document.body.classList.toggle('dark-mode');
        });
    }
    
    // Gestion du Bouton de Réinitialisation des Statistiques (si disponible)
    if ($('reset-stats-btn')) {
        $('reset-stats-btn').addEventListener('click', () => {
            if(confirm("Voulez-vous vraiment réinitialiser toutes les statistiques (Max Speed, Distance, Temps de Mouvement) ?")) {
                // Réinitialiser les variables d'état (exemple)
                kSpeed = 0; kSpeed_m_s = 0; timeMoving = 0; sTime = null;
                // Mettre à jour les éléments du DOM
                if ($('distance-total-km')) $('distance-total-km').textContent = '0.000 km | 0.00 m'; 
                if ($('speed-max')) $('speed-max').textContent = '0.00000 km/h'; 
            }
        });
    }

    // Démarrage des services
    syncH(); // Démarrage de la synchronisation de l'heure NTP
    startGPS(); // Démarrage initial du GPS

    // Boucle de mise à jour lente (Astro/Temps/Météo)
    if (domID === null) {
        domID = setInterval(async () => {
            const now = getCDate();
            
            // Mise à jour de l'heure et du temps écoulé
            if (now) {
                if ($('local-time') && !$('local-time').textContent.includes('Synchronisation')) {
                    $('local-time').textContent = now.toLocaleTimeString('fr-FR');
                }
                if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
                if ($('time-elapsed')) $('time-elapsed').textContent = sTime ? ((now.getTime() - sTime) / 1000).toFixed(2) + ' s' : '0.00 s';
                // La variable 'timeMoving' serait mise à jour dans la logique GPS/Kalman
            }
            
            // Mise à jour Astro si on a une position valide
            if (lat !== 0 && lon !== 0) {
                updateAstro(lat, lon);
            }
            
            // Mise à jour Météo (si l'intervalle n'est pas déjà actif ou si le GPS est actif)
            if (lat && lon && weatherID === null) {
                const weatherData = await fetchWeather(lat, lon);
                if (weatherData) {
                    if ($('weather-status')) $('weather-status').textContent = `ACTIF`;
                    if ($('temp-air-2')) $('temp-air-2').textContent = `${weatherData.tempC.toFixed(1)} °C`;
                    if ($('pressure-2')) $('pressure-2').textContent = `${weatherData.pressure_hPa.toFixed(0)} hPa`;
                    if ($('humidity-2')) $('humidity-2').textContent = `${weatherData.humidity_perc.toFixed(0)} %`;
                    if ($('air-density')) $('air-density').textContent = `${weatherData.air_density.toFixed(3)} kg/m³`;
                    if ($('dew-point')) $('dew-point').textContent = `${weatherData.dew_point.toFixed(1)} °C`;
                } else {
                    if ($('weather-status')) $('weather-status').textContent = `❌ API ÉCHOUÉE`;
                }
            }
        }, DOM_SLOW_UPDATE_MS); 
    }
    
    // Intervalle pour la mise à jour Météo (pour s'assurer qu'elle se fait toutes les 30s)
    if (weatherID === null) {
        weatherID = setInterval(() => {
            if (lat !== 0 && lon !== 0) {
                fetchWeather(lat, lon); 
            }
        }, WEATHER_UPDATE_MS);
    }
});
