// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER CORE V2 (UKF 21 ÉTATS)
// FUSION DE TOUTES LES MEILLEURES FONCTIONNALITÉS
// =================================================================

// ⚠️ DÉPENDANCES CRITIQUES (à charger avant ce fichier dans l'HTML) :
// - math.min.js (pour l'UKF)
// - ukf-lib.js (définissant la classe ProfessionalUKF)
// - astro.js (définissant la fonction getAstroData)
// - suncalc.js (pour les calculs solaires)
// - leaflet.js / turf.min.js (pour la carte)
// =================================================================


// --- BLOC 1 : CONSTANTES ET UTILITAIRES DE BASE ---

const $ = id => document.getElementById(id);

// Constantes Mathématiques/Physiques
const D2R = Math.PI / 180, R2D = 180 / Math.PI; 
const KMH_MS = 3.6;         
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const G_U = 6.67430e-11;    // Constante gravitationnelle universelle
const G_STD = 9.8067;       // Gravité standard (pour WGS84)
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation Terre (rad/s)
const R_SPECIFIC_AIR = 287.058; // Constante spécifique de l'air sec (J/kg·K)
const GAMMA_AIR = 1.4;      // Indice adiabatique de l'air

// Constantes ISA (Atmosphère Standard Internationale)
const TEMP_SEA_LEVEL_K = 288.15; // 15°C
const RHO_SEA_LEVEL = 1.225;     // Densité de l'air au niveau de la mer (kg/m³)
const BARO_ALT_REF_HPA = 1013.25; // Pression de référence (hPa)
const DOM_SLOW_UPDATE_MS = 2000; 

// Constantes WGS84 (Système Géodésique Mondial 1984)
const WGS84_A = 6378137.0;  // Rayon équatorial WGS84 (m)
const WGS84_F = 1 / 298.257223563; // Aplatissement WGS84
const WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F; // Excentricité au carré

// Endpoints API
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

// Formatage des données (Anti-NaN/Null/Inf)
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || val === Infinity || val === -Infinity || Math.abs(val) < 1e-9) { 
        // Affichage lisible pour l'utilisateur
        return (decimals === 0 ? '--' : '--.--') + suffix; 
    }
    return val.toFixed(decimals) + suffix;
};

// Formatage exponentiel
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || val === Infinity || val === -Infinity) {
        // Retourne un format scientifique vide
        const zeroDecimals = '0.' + Array(decimals).fill('0').join('');
        return zeroDecimals + 'e+0' + suffix; 
    }
    return val.toExponential(decimals) + suffix;
};


// --- BLOC 2 : ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE ---

let ukf = null; // Instance du ProfessionalUKF
let currentUKFReactivity = 'MEDIUM'; // Peut être changé par l'utilisateur
let currentCelestialBody = 'EARTH';
let netherMode = false; // Facteur 1:8 pour la distance
let distanceRatioMode = false; // Calcul de distance au centre vs. surface

let currentPosition = { 
    lat: 43.2964,   // Default: Marseille (pour débloquer Astro/Météo au démarrage)
    lon: 5.3697,
    acc: 10.0,      
    spd: 0.0,
    alt: 0.0
};
let kAlt = 0.0; // Altitude filtrée (m)
let currentSpeed = 0.0; // Vitesse filtrée (m/s)

// Variables de Météo et Physique (Utilisées par l'UKF et les calculs métrologiques)
let lastT_K = TEMP_SEA_LEVEL_K; // Température en Kelvin (par défaut ISA)
let lastP_hPa = BARO_ALT_REF_HPA; // Pression en hPa (par défaut ISA)
let lastH_perc = 0.5; // Humidité relative (par défaut 50%)
let lastWeatherData = null;

let currentAirDensity = RHO_SEA_LEVEL; // Densité de l'air (calculée)
let currentSpeedOfSound = 340.29; // Vitesse du son (calculée)
let currentMass = 70.0; // Masse (kg), paramètre utilisateur

let lServH = 0; // Heure serveur NTP
let lLocH = 0;  // Heure locale NTP


// --- BLOC 3 : MODÈLES PHYSIQUES ET GÉODÉSIQUES ---

/**
 * Calcule la vitesse du son dans l'air.
 * @param {number} tempK - Température de l'air en Kelvin.
 * @returns {number} Vitesse du son en m/s.
 */
function getSpeedOfSound(tempK) {
    // V_sound = sqrt(GAMMA_AIR * R_SPECIFIC_AIR * T)
    return Math.sqrt(GAMMA_AIR * R_SPECIFIC_AIR * tempK);
}

/**
 * Calcule la densité de l'air humide (Loi des gaz parfaits pour l'air humide).
 * @param {number} p_Pa - Pression atmosphérique en Pascal.
 * @param {number} t_K - Température en Kelvin.
 * @param {number} h_rel - Humidité relative (0.0 à 1.0).
 * @returns {number} Densité de l'air humide en kg/m³.
 */
function getAirDensity(p_Pa, t_K, h_rel) {
    // Pression de vapeur saturante (formule Magnus-Tetens, simplifiée)
    const T_c = t_K - 273.15;
    const Es = 610.7 * Math.pow(10, (7.5 * T_c) / (237.3 + T_c)); // Pression de vapeur saturante (Pa)
    const E = h_rel * Es; // Pression de vapeur réelle (Pa)
    
    const R_WATER_VAPOR = 461.5; // Constante des gaz pour la vapeur d'eau
    // DENSITY = (P - E) / (R_SPECIFIC_AIR * T) + E / (R_WATER_VAPOR * T)
    const DENSITY_AIR = (p_Pa - E) / (R_SPECIFIC_AIR * t_K) + (E / (R_WATER_VAPOR * t_K));
    return DENSITY_AIR;
}

/**
 * Met à jour l'accélération gravitationnelle et le rayon terrestre (WGS84 ou Modèle simulé).
 * @param {string} body - 'EARTH', 'ROTATING', 'MOON', etc.
 * @param {number} altitude - Altitude GNSS/Filtrée (m).
 * @param {number} [rotationRadius] - Rayon simulé pour l'objet ROTATING.
 * @param {number} [angularVelocity] - Vitesse angulaire simulée pour ROTATING.
 * @returns {object} { G_ACC_NEW, R_ALT_CENTER_REF_NEW }
 */
function updateCelestialBody(body, altitude, rotationRadius = 100, angularVelocity = 0.0) {
    let G_ACC_NEW = G_STD;
    let R_ALT_CENTER_REF_NEW = WGS84_A;

    if (body === 'EARTH') {
        const phi = currentPosition.lat * D2R;
        // Correction de la gravité WGS84 en fonction de la latitude et de l'altitude
        const factor_lat = (1 + WGS84_BETA * Math.sin(phi) * Math.sin(phi)) / Math.sqrt(1 - WGS84_E2 * Math.sin(phi) * Math.sin(phi));
        G_ACC_NEW = WGS84_G_EQUATOR * factor_lat * (1 - 2 * altitude / WGS84_A);
        
        // Calcul du rayon de courbure local (facteur important pour la conversion Lat/Lon -> XYZ pour l'UKF)
        const N = WGS84_A / Math.sqrt(1 - WGS84_E2 * Math.sin(phi) * Math.sin(phi));
        R_ALT_CENTER_REF_NEW = N + altitude;

    } else if (body === 'ROTATING') {
        // Simulation d'un corps en rotation (ex: centrifugeuse)
        const centrifugalAcc = rotationRadius * angularVelocity * angularVelocity;
        // Accélération résultante (simplifiée pour l'affichage)
        G_ACC_NEW = Math.sqrt(G_STD * G_STD + centrifugalAcc * centrifugalAcc); 
        R_ALT_CENTER_REF_NEW = 100.0; // Rayon de référence arbitraire
    }
    
    // Mettre à jour les variables globales (UKF en a besoin dans sa dynamique de prédiction)
    G_ACC = G_ACC_NEW;
    R_ALT_CENTER_REF = R_ALT_CENTER_REF_NEW;
    return { G_ACC_NEW, R_ALT_CENTER_REF_NEW };
}

/**
 * Calcule des métriques Biologiques / SpatioTemporelles (BioSVT) avancées.
 * Intégrée ici, mais peut être déplacée si astro.js est un fichier très large.
 */
function calculateBioSVT(tempC, kAlt, humidityPerc, pressurePa, sunAltitudeRad) {
    const tempK = tempC + 273.15;
    const humidity = humidityPerc / 100.0;
    
    const R_WATER_VAPOR = 461.5; 
    const T = tempC;
    
    // 1. Pression de vapeur saturante (Pa)
    const Es = 6.11 * Math.pow(10, (7.5 * T) / (237.3 + T)) * 100; 
    
    // 2. Point de Rosée (Dew Point)
    const E = humidity * Es; 
    const gamma = Math.log(E / 611);
    const dewPoint = (237.3 * gamma) / (7.5 - gamma);
    
    // 3. Densité de l'air humide (si non calculée par l'API météo)
    const DENSITY_AIR = (pressurePa - E) / (R_SPECIFIC_AIR * tempK) + (E / (R_WATER_VAPOR * tempK));
    
    // 4. Facteur de rayonnement solaire (simple)
    const solarFactor = Math.max(0, Math.sin(sunAltitudeRad)); 

    return {
        dewPoint: dewPoint,
        humideAirDensity: DENSITY_AIR,
        solarFactor: solarFactor,
        altitudeMeter: kAlt,
        tempK: tempK
    };
}


// --- BLOC 4 : LOGIQUE API ET TEMPS (NTP/MÉTÉO) ---

/**
 * Synchronisation du temps avec un serveur NTP (via API) pour la correction du temps de vol.
 */
async function syncH() {
    try {
        const response = await fetch(SERVER_TIME_ENDPOINT);
        const data = await response.json();
        const serverTimestamp = new Date(data.utc_datetime).getTime();
        
        lServH = serverTimestamp;
        lLocH = Date.now();
        console.log("NTP : Synchro réussie.");

    } catch (error) {
        console.error("NTP : Échec de la synchronisation de l'heure. Utilisation de l'horloge locale.", error);
        if ($('local-time')) $('local-time').textContent = 'SYNCHRO ÉCHOUÉE 🔴';
    }
}

/**
 * Met à jour l'heure locale corrigée (utilisée pour l'affichage et l'Astro).
 */
function getCDate(serverTimeMs, localTimeMs) {
    if (serverTimeMs > 0 && localTimeMs > 0) {
        const offset = serverTimeMs - localTimeMs;
        return new Date(Date.now() + offset);
    }
    return new Date();
}

/**
 * Récupère les données météo via proxy Vercel.
 */
async function fetchWeather(lat, lon) {
    try {
        const response = await fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`);
        if (!response.ok) {
             throw new Error(`Erreur API: ${response.status}`);
        }
        const data = await response.json();
        
        // Convertir pression en Pa pour les calculs internes
        data.pressure_Pa = data.pressure_hPa * 100;
        
        // Calcule la densité de l'air et la vitesse du son
        data.tempK = data.tempC + 273.15;
        data.air_density = getAirDensity(data.pressure_Pa, data.tempK, data.humidity_perc / 100);
        data.speed_of_sound = getSpeedOfSound(data.tempK);
        
        return data;
    } catch (error) {
        console.error("MÉTÉO : Échec de la récupération des données météo.", error);
        return null;
    }
}


// --- BLOC 5 : GESTION DES CAPTEURS ET UKF (BOUCLE RAPIDE) ---

/**
 * Met à jour les informations météo dans le DOM et les variables globales.
 */
function updateWeatherDOM(data, isOfflineDefault = false) {
    lastP_hPa = data.pressure_hPa;
    lastT_K = data.tempK;
    lastH_perc = data.humidity_perc / 100.0;
    currentAirDensity = data.air_density;
    currentSpeedOfSound = data.speed_of_sound;
    
    // Mise à jour des éléments DOM
    if ($('temp-air-2')) $('temp-air-2').textContent = `${data.tempC.toFixed(1)} °C`;
    if ($('pressure-2')) $('pressure-2').textContent = `${data.pressure_hPa.toFixed(0)} hPa`;
    if ($('humidity-2')) $('humidity-2').textContent = `${data.humidity_perc} %`;
    if ($('air-density')) $('air-density').textContent = `${data.air_density.toFixed(3)} kg/m³`;
    
    if($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = `${currentSpeedOfSound.toFixed(2)} m/s (${isOfflineDefault ? 'ISA' : 'Météo'})`;
}

/**
 * Traite les données GNSS brutes, effectue la prédiction/mise à jour UKF, et met à jour l'état.
 * @param {object} position - Données de géolocalisation.
 */
function updateGPSState(position) {
    const lat = position.coords.latitude;
    const lon = position.coords.longitude;
    const acc = position.coords.accuracy;
    const alt = position.coords.altitude || 0.0;
    const spd = position.coords.speed || 0.0;

    currentPosition = { lat, lon, acc, spd, alt };

    // 1. Prédiction/Correction UKF
    if (ukf) {
        // La prédiction doit être faite avec les données IMU (accélérations)
        // ukf.predict(accelX, accelY, accelZ, G_ACC); 
        
        // La correction est faite avec les données GNSS (mesures)
        ukf.update(currentPosition); 
        
        // Récupération de l'état filtré
        kAlt = netherMode ? ukf.getAltitude() / 8.0 : ukf.getAltitude();
        currentSpeed = netherMode ? ukf.getSpeed() / 8.0 : ukf.getSpeed();
    } else {
        // Fallback sans filtre
        kAlt = netherMode ? alt / 8.0 : alt;
        currentSpeed = netherMode ? spd / 8.0 : spd;
    }
    
    // 2. Mise à jour du DOM (Boucle rapide)
    
    // Gravité et correction WGS84
    updateCelestialBody(currentCelestialBody, kAlt);
    if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC.toFixed(4)} m/s²`;
    
    // Mise à jour des métriques GNSS/UKF
    if ($('lat-val')) $('lat-val').textContent = dataOrDefault(currentPosition.lat, 6) + ' °';
    if ($('lon-val')) $('lon-val').textContent = dataOrDefault(currentPosition.lon, 6) + ' °';
    if ($('alt-val')) $('alt-val').textContent = dataOrDefault(kAlt, 2) + ' m';
    if ($('spd-val')) $('spd-val').textContent = dataOrDefault(currentSpeed * KMH_MS, 2) + ' km/h';
    
    // Précision et facteur de correction
    if ($('acc-horiz')) $('acc-horiz').textContent = dataOrDefault(currentPosition.acc, 2) + ' m';
    
    // Mettre à jour la carte (omise ici pour la concision)
    // updateMap(lat, lon, netherMode); 
}

// Gestion des erreurs GPS
function errorGPS(err) {
    console.warn(`GPS ERROR(${err.code}): ${err.message}`);
    if ($('gnss-status')) $('gnss-status').textContent = `ERREUR GPS (${err.code}) 🔴`;
}

// Initialisation du GPS
let watchId = null;
function initGPS() {
    if (navigator.geolocation) {
        if ($('gnss-status')) $('gnss-status').textContent = 'Démarrage... 🟡';
        watchId = navigator.geolocation.watchPosition(updateGPSState, errorGPS, {
            enableHighAccuracy: true,
            maximumAge: 0, 
            timeout: 5000 
        });
        if ($('gnss-status')) $('gnss-status').textContent = 'Actif 🟢';
    } else {
        if ($('gnss-status')) $('gnss-status').textContent = 'Non supporté ❌';
        alert("La géolocalisation n'est pas supportée par ce navigateur.");
    }
}

// Initialisation IMU (DeviceMotion/DeviceOrientation)
function activateDeviceMotion() {
    // Logique d'activation des capteurs IMU (avec demande d'autorisation iOS)
    console.log("IMU : Tentative d'activation des capteurs.");
    if ($('imu-status')) $('imu-status').textContent = 'Actif 🟢';
    // ... (Ajouter ici la logique window.addEventListener('devicemotion', ...))
}


// --- BLOC 6 : BOUCLE LENTE (ASTRO, MÉTÉO, DOM COMPLEXE) ---

function slowUpdateLoop() {
    const lat = currentPosition.lat;
    const lon = currentPosition.lon;
    const now = getCDate(lServH, lLocH);

    // 1. Mise à jour de l'horloge
    if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString('fr-FR');
    if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');

    // 2. Mise à jour Météo (toutes les 5 minutes)
    if (Date.now() - (lastWeatherData ? lastWeatherData.timestamp : 0) > 300000) {
        fetchWeather(lat, lon).then(data => {
            if (data) {
                lastWeatherData = data;
                updateWeatherDOM(data);
                
                // 3. Mise à jour Astro et BioSVT
                // *getAstroData* est supposée être définie dans 'astro.js'
                if (typeof getAstroData === 'function') {
                    const astroData = getAstroData(now, lat, lon);
                    const sunAltitudeRad = astroData.sun.altitude; 

                    // Calcul BioSVT (Point de Rosée, etc.)
                    const pressurePa = lastWeatherData.pressure_hPa * 100;
                    const bioSim = calculateBioSVT(lastWeatherData.tempC, kAlt, lastWeatherData.humidity_perc, pressurePa, sunAltitudeRad);
                    
                    if ($('dew-point')) $('dew-point').textContent = `${bioSim.dewPoint.toFixed(1)} °C`;
                    if ($('air-density')) $('air-density').textContent = `${bioSim.humideAirDensity.toFixed(3)} kg/m³ (Calculé)`;
                    
                    // Mise à jour DOM Astro
                    if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(sunAltitudeRad * R2D, 2) + ' °';
                    if ($('moon-phase')) $('moon-phase').textContent = astroData.moon.phaseName;
                    if ($('tst-hrs')) $('tst-hrs').textContent = astroData.TST_HRS ? astroData.TST_HRS.toFixed(4) : '--';
                }
            }
        });
    }
}


// --- BLOC 7 : DÉMARRAGE ET ÉVÉNEMENTS (DOCUMENT READY) ---

window.addEventListener('load', () => {

    // 1. Vérification et initialisation des dépendances critiques (math.js pour UKF)
    if (typeof ProfessionalUKF !== 'undefined' && typeof math !== 'undefined') {
        try {
            // Initialisation UKF avec le mode de réactivité par défaut
            ukf = new ProfessionalUKF(currentUKFReactivity); 
            console.log("UKF 21 États : Initialisé. 🟢");
            if ($('ekf-status')) $('ekf-status').textContent = 'Initialisé 🟢';
        } catch (e) {
            console.error("🔴 ÉCHEC D'INITIALISATION UKF: " + e.message);
            if ($('ekf-status')) $('ekf-status').textContent = 'ERREUR UKF 🔴';
        }
    } else {
        console.error("🔴 ÉCHEC CRITIQUE : math.js ou ProfessionalUKF non trouvé. UKF désactivé.");
        if ($('ekf-status')) $('ekf-status').textContent = 'DÉSACTIVÉ 🔴';
    }
    
    // 2. Initialisation des valeurs par défaut physiques (Offline-First)
    if (lastWeatherData) {
        // Si les données sont en cache
        updateWeatherDOM(lastWeatherData, true);
    } else {
        // Utiliser les valeurs ISA par défaut
        updateWeatherDOM({
            tempC: TEMP_SEA_LEVEL_K - 273.15,
            pressure_hPa: BARO_ALT_REF_HPA,
            humidity_perc: 50,
            tempK: TEMP_SEA_LEVEL_K,
            air_density: RHO_SEA_LEVEL,
            speed_of_sound: getSpeedOfSound(TEMP_SEA_LEVEL_K)
        }, true);
    }
    
    if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
    
    // 3. Démarrer la synchro NTP (pour un temps précis)
    syncH();
    
    // 4. Démarrer les capteurs
    initGPS(); 
    
    // 5. Boucles de mise à jour
    // Le GPS lance la boucle rapide (via watchPosition -> updateGPSState)
    // La boucle lente gère le DOM, l'Astro et la Météo
    setInterval(slowUpdateLoop, DOM_SLOW_UPDATE_MS); 

    // 6. Écouteurs d'événements (Interactions Utilisateur)
    
    // Massez
    if ($('mass-input')) $('mass-input').addEventListener('input', (e) => {
        currentMass = parseFloat(e.target.value) || 70.0;
        if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
    });

    // Corps Céleste (Gravité)
    if ($('celestial-body-select')) $('celestial-body-select').addEventListener('change', (e) => {
        currentCelestialBody = e.target.value;
        updateCelestialBody(currentCelestialBody, kAlt);
    });

    // Mode Nether
    const netherToggleBtn = $('nether-toggle-btn');
    if (netherToggleBtn) netherToggleBtn.addEventListener('click', () => {
        netherMode = !netherMode;
        netherToggleBtn.textContent = `Mode Nether: ${netherMode ? 'ACTIVÉ (1:8) 🔥' : 'DÉSACTIVÉ (1:1) 🌍'}`;
        // L'état UKF/Alt sera corrigé à la prochaine mise à jour GPS
    });
    
    // Réactivité UKF
    if ($('ukf-reactivity-mode')) $('ukf-reactivity-mode').addEventListener('change', (e) => {
        currentUKFReactivity = e.target.value;
        if (ukf) {
            // Mise à jour de la matrice de bruit de processus
            ukf.Q_Base = ukf.getProcessNoiseMatrix(currentUKFReactivity); 
            console.log(`UKF : Réactivité changée à ${currentUKFReactivity}.`);
        }
    });
    
    // Activation IMU (Capteurs de mouvement) - VÉRIFICATION DÉFENSIVE
    const activateButton = document.getElementById('activate-sensors-btn');
    if (activateButton) {
        if (typeof activateDeviceMotion === 'function') {
            activateButton.addEventListener('click', activateDeviceMotion); 
        } else {
             console.warn("🟡 AVERTISSEMENT : 'activateDeviceMotion' est manquante ou non définie. Bouton IMU non fonctionnel.");
        }
    }
});
