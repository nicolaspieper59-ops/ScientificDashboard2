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
// Constantes WGS84 supplémentaires pour la gravité
const WGS84_G_EQUATOR = 9.7803253359; 
const WGS84_BETA = 0.0053024;      

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
let currentUKFReactivity = 'NORMAL'; // Peut être changé par l'utilisateur
let currentCelestialBody = 'EARTH';
let netherMode = false; // Facteur 1:8 pour la distance
let distanceRatioMode = false; // Calcul de distance au centre vs. surface

let currentPosition = { 
    lat: 43.2964,   // Default: Marseille 
    lon: 5.3697,
    acc: 10.0,      
    spd: 0.0,
    alt: 0.0
};

// Variables d'état filtré (remplacées par ukf.getState())
let kAlt = 0.0; 
let currentSpeed = 0.0; 
let G_ACC = G_STD; // Gravité locale mise à jour
let R_ALT_CENTER_REF = WGS84_A; // Rayon de courbure locale

// Variables de Météo et Physique
let lastT_K = TEMP_SEA_LEVEL_K; 
let lastP_hPa = BARO_ALT_REF_HPA; 
let lastH_perc = 0.5; 
let lastWeatherData = null;

let currentAirDensity = RHO_SEA_LEVEL; 
let currentSpeedOfSound = 340.29; 
let currentMass = 70.0; 

let lServH = 0; 
let lLocH = 0;  


// --- BLOC 3 : MODÈLES PHYSIQUES ET GÉODÉSIQUES ---

/**
 * Calcule la vitesse du son dans l'air.
 */
function getSpeedOfSound(tempK) {
    return Math.sqrt(GAMMA_AIR * R_SPECIFIC_AIR * tempK);
}

/**
 * Calcule la densité de l'air humide.
 */
function getAirDensity(p_Pa, t_K, h_rel) {
    const T_c = t_K - 273.15;
    const Es = 610.7 * Math.pow(10, (7.5 * T_c) / (237.3 + T_c)); 
    const E = h_rel * Es; 
    const R_WATER_VAPOR = 461.5; 
    const DENSITY_AIR = (p_Pa - E) / (R_SPECIFIC_AIR * t_K) + (E / (R_WATER_VAPOR * t_K));
    return DENSITY_AIR;
}

/**
 * Met à jour l'accélération gravitationnelle et le rayon terrestre (WGS84 ou Modèle simulé).
 */
function updateCelestialBody(body, altitude, rotationRadius = 100, angularVelocity = 0.0) {
    let G_ACC_NEW = G_STD;
    let R_ALT_CENTER_REF_NEW = WGS84_A;

    if (body === 'EARTH') {
        const phi = currentPosition.lat * D2R;
        // Correction de la gravité WGS84 en fonction de la latitude et de l'altitude
        const sin_phi = Math.sin(phi);
        const factor_lat = (1 + WGS84_BETA * sin_phi * sin_phi) / Math.sqrt(1 - WGS84_E2 * sin_phi * sin_phi);
        G_ACC_NEW = WGS84_G_EQUATOR * factor_lat * (1 - 2 * altitude / WGS84_A);
        
        // Calcul du rayon de courbure local (N)
        const N = WGS84_A / Math.sqrt(1 - WGS84_E2 * sin_phi * sin_phi);
        R_ALT_CENTER_REF_NEW = N + altitude;

    } else if (body === 'ROTATING') {
        const centrifugalAcc = rotationRadius * angularVelocity * angularVelocity;
        G_ACC_NEW = Math.sqrt(G_STD * G_STD + centrifugalAcc * centrifugalAcc); 
        R_ALT_CENTER_REF_NEW = 100.0; 
    }
    
    G_ACC = G_ACC_NEW;
    R_ALT_CENTER_REF = R_ALT_CENTER_REF_NEW;
    return { G_ACC_NEW, R_ALT_CENTER_REF_NEW };
}

/**
 * Calcule des métriques Biologiques / SpatioTemporelles (BioSVT) avancées.
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
    
    // 3. Densité de l'air humide 
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
 * Synchronisation du temps avec un serveur NTP (via API).
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
        if ($('heure-locale')) $('heure-locale').textContent = 'SYNCHRO ÉCHOUÉE 🔴';
    }
}

/**
 * Met à jour l'heure locale corrigée.
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
        
        data.pressure_Pa = data.pressure_hPa * 100;
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
    
    // Mise à jour des éléments DOM (Météo)
    if ($('air-temp-c')) $('air-temp-c').textContent = `${data.tempC.toFixed(1)} °C`;
    if ($('pressure-hpa')) $('pressure-hpa').textContent = `${data.pressure_hPa.toFixed(0)} hPa`;
    if ($('humidity-perc')) $('humidity-perc').textContent = `${data.humidity_perc} %`;
    if ($('air-density')) $('air-density').textContent = `${data.air_density.toFixed(3)} kg/m³`;
    
    if($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = `${currentSpeedOfSound.toFixed(2)} m/s (${isOfflineDefault ? 'ISA' : 'Météo'})`;
}

/**
 * Traite les données GNSS brutes, effectue la prédiction/mise à jour UKF, et met à jour l'état.
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
        if ($('statut-ekf-fusion')) $('statut-ekf-fusion').textContent = 'Actif 🟢';

        // 1.1. Prédiction UKF (Simulation IMU : Acc et Gyro nuls)
        // Les accélérations et les vitesses angulaires (gyro) doivent venir de l'IMU réel
        const accelX = 0, accelY = 0, accelZ = 0; 
        const gyroX = 0, gyroY = 0, gyroZ = 0; 
        ukf.predict(accelX, accelY, accelZ, gyroX, gyroY, gyroZ); 
        
        // 1.2. Correction UKF avec les données GNSS (Mesure)
        ukf.update(currentPosition, 'GNSS'); 
        
        // 1.3. Récupération de l'état filtré
        const state = ukf.getState();
        kAlt = netherMode ? state.alt / 8.0 : state.alt;
        currentSpeed = netherMode ? state.speed / 8.0 : state.speed;

        // Mise à jour des données d'état filtré (EKF/UKF)
        if ($('lat-ekf')) $('lat-ekf').textContent = dataOrDefault(state.lat, 6) + ' °';
        if ($('lon-ekf')) $('lon-ekf').textContent = dataOrDefault(state.lon, 6) + ' °';
        if ($('alt-ekf')) $('alt-ekf').textContent = dataOrDefault(kAlt, 2) + ' m';
        if ($('ukf-v-uncert')) $('ukf-v-uncert').textContent = dataOrDefault(state.kUncert, 6) + ' m/s';
        if ($('ukf-alt-sigma')) $('ukf-alt-sigma').textContent = dataOrDefault(state.kUncert, 6) + ' m'; // Simplification: réutiliser kUncert
        if ($('acc-horiz')) $('acc-horiz').textContent = dataOrDefault(currentPosition.acc, 2) + ' m';
        
    } else {
        if ($('statut-ekf-fusion')) $('statut-ekf-fusion').textContent = 'DÉSACTIVÉ 🔴';
        // Fallback sans filtre
        kAlt = netherMode ? alt / 8.0 : alt;
        currentSpeed = netherMode ? spd / 8.0 : spd;
    }
    
    // 2. Mise à jour du DOM (Boucle rapide)
    
    // Gravité et correction WGS84
    updateCelestialBody(currentCelestialBody, kAlt);
    if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC.toFixed(4)} m/s²`;
    if ($('gravite-wgs84')) $('gravite-wgs84').textContent = `${G_ACC.toFixed(4)} m/s²`;
    
    // Mise à jour des métriques GNSS/UKF
    if ($('lat-val')) $('lat-val').textContent = dataOrDefault(currentPosition.lat, 6) + ' °'; // Affichage Lat Brute
    if ($('lon-val')) $('lon-val').textContent = dataOrDefault(currentPosition.lon, 6) + ' °'; // Affichage Lon Brute
    if ($('alt-val')) $('alt-val').textContent = dataOrDefault(kAlt, 2) + ' m'; // Affichage Alt Filtrée
    
    // Vitesse (Filtrée)
    if ($('speed-stable')) $('speed-stable').textContent = dataOrDefault(currentSpeed * KMH_MS, 2) + ' km/h';
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = dataOrDefault(currentSpeed, 2) + ' m/s';
    
    // Vitesse Brute
    if ($('speed-raw-ms')) $('speed-raw-ms').textContent = dataOrDefault(currentPosition.spd, 2) + ' m/s';

    // Précision
    if ($('acc-gps')) $('acc-gps').textContent = dataOrDefault(currentPosition.acc, 2) + ' m';
    if ($('acc-horiz')) $('acc-horiz').textContent = dataOrDefault(currentPosition.acc, 2) + ' m';
    
    // Mettre à jour la carte (omise ici pour la concision)
    // updateMap(lat, lon, netherMode); 
}

// Gestion des erreurs GPS
function errorGPS(err) {
    console.warn(`GPS ERROR(${err.code}): ${err.message}`);
    if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = `ERREUR GPS (${err.code}) 🔴`;
}

// Initialisation du GPS
let watchId = null;
function initGPS() {
    if (navigator.geolocation) {
        if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = 'Démarrage... 🟡';
        watchId = navigator.geolocation.watchPosition(updateGPSState, errorGPS, {
            enableHighAccuracy: true,
            maximumAge: 0, 
            timeout: 5000 
        });
        if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = 'Actif 🟢';
    } else {
        if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = 'Non supporté ❌';
        alert("La géolocalisation n'est pas supportée par ce navigateur.");
    }
}

// Initialisation IMU (DeviceMotion/DeviceOrientation)
function activateDeviceMotion() {
    console.log("IMU : Tentative d'activation des capteurs.");
    if ($('statut-capteur')) $('statut-capteur').textContent = 'Actif 🟢';
    // ... (Ajouter ici la logique window.addEventListener('devicemotion', ...))
}


// --- BLOC 6 : BOUCLE LENTE (ASTRO, MÉTÉO, DOM COMPLEXE) ---

function slowUpdateLoop() {
    const lat = currentPosition.lat;
    const lon = currentPosition.lon;
    const now = getCDate(lServH, lLocH);

    // 1. Mise à jour de l'horloge
    if ($('heure-locale')) $('heure-locale').textContent = now.toLocaleTimeString('fr-FR');
    if ($('date-heure-utc')) $('date-heure-utc').textContent = now.toLocaleDateString('fr-FR');
    if ($('date-display-astro')) $('date-display-astro').textContent = now.toLocaleDateString('fr-FR');

    // 2. Mise à jour Météo (toutes les 5 minutes - 300000 ms)
    if (Date.now() - (lastWeatherData ? lastWeatherData.timestamp : 0) > 300000) {
        if ($('statut-meteo')) $('statut-meteo').textContent = 'Mise à jour... 🟡';
        fetchWeather(lat, lon).then(data => {
            if (data) {
                lastWeatherData = data;
                updateWeatherDOM(data);
                if ($('statut-meteo')) $('statut-meteo').textContent = 'Actif 🟢';
                
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
                    if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(astroData.sun.azimuth * R2D, 2) + ' °';
                    if ($('moon-phase-name')) $('moon-phase-name').textContent = astroData.moon.phaseName;
                    if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(astroData.moon.fraction * 100, 1) + ' %';
                    
                    // TST/MST
                    if ($('tst')) $('tst').textContent = astroData.TST_HRS ? astroData.TST_HRS.toFixed(4) : '--';
                    // MST non calculé ici (requiert EPHEM/complexité)
                    if ($('mst')) $('mst').textContent = '--'; 
                    
                }
            } else {
                 if ($('statut-meteo')) $('statut-meteo').textContent = 'ÉCHEC 🔴';
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
            ukf = new ProfessionalUKF(); 
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
        updateWeatherDOM(lastWeatherData, true);
    } else {
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
    if ($('const-c')) $('const-c').textContent = `${C_L} m/s`;
    if ($('const-G')) $('const-G').textContent = dataOrDefaultExp(G_U, 6) + ' m³/kg/s²';
    
    // 3. Démarrer la synchro NTP (pour un temps précis)
    syncH();
    
    // 4. Démarrer les capteurs
    initGPS(); 
    
    // 5. Boucles de mise à jour
    setInterval(slowUpdateLoop, DOM_SLOW_UPDATE_MS); 

    // 6. Écouteurs d'événements (Interactions Utilisateur)
    
    // Masse
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
        if ($('nether-mode-status')) $('nether-mode-status').textContent = netherMode ? '1/8' : '1.000';
    });
    
    // Réactivité UKF (Ajustement de la matrice Q)
    if ($('ukf-reactivity-mode')) $('ukf-reactivity-mode').addEventListener('change', (e) => {
        currentUKFReactivity = e.target.value;
        if (ukf) {
            // Utilise la méthode privée/interne pour reconstruire Q
            ukf.Q = ukf._getProcessNoiseMatrix(currentUKFReactivity); 
            console.log(`UKF : Réactivité changée à ${currentUKFReactivity}.`);
        }
    });
    
    // Activation IMU (Capteurs de mouvement)
    const activateButton = document.getElementById('activate-sensors-btn');
    if (activateButton) {
        if (typeof activateDeviceMotion === 'function') {
            activateButton.addEventListener('click', activateDeviceMotion); 
        } else {
             console.warn("🟡 AVERTISSEMENT : 'activateDeviceMotion' est manquante ou non définie. Bouton IMU non fonctionnel.");
        }
    }
});
