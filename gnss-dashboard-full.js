// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER CORE V2 (UKF 21 ÉTATS)
// FUSION DE TOUTES LES MEILLEURES FONCTIONNALITÉS
// VERSION : ROBUSTESSE NaN & CORRECTION ID
// =================================================================

// ⚠️ DÉPENDANCES CRITIQUES (à charger avant ce fichier dans l'HTML) :
// - math.min.js (pour l'UKF)
// - ukf-lib.js (définissant la classe ProfessionalUKF)
// - astro.js (définissant la fonction getAstroData)
// ...
// =================================================================


// --- BLOC 1 : CONSTANTES ET UTILITAIRES DE BASE ---

const $ = id => document.getElementById(id);

// Constantes Mathématiques/Physiques
const D2R = Math.PI / 180, R2D = 180 / Math.PI; 
const KMH_MS = 3.6;         
const C_L = 299792458;      
const G_U = 6.67430e-11;    
const G_STD = 9.8067;       
const R_SPECIFIC_AIR = 287.058; 
const GAMMA_AIR = 1.4;      

// Constantes ISA (Atmosphère Standard Internationale)
const TEMP_SEA_LEVEL_K = 288.15; // 15°C
const RHO_SEA_LEVEL = 1.225;     
const BARO_ALT_REF_HPA = 1013.25; 
const DOM_SLOW_UPDATE_MS = 2000; 

// Constantes WGS84
const WGS84_A = 6378137.0;  
const WGS84_F = 1 / 298.257223563; 
const WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F; 
const WGS84_G_EQUATOR = 9.7803253359; 
const WGS84_BETA = 0.0053024;      

// Endpoints API
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

// Formatage des données (Anti-NaN/Null/Inf)
const dataOrDefault = (val, decimals, suffix = '') => {
    // Vérification stricte des valeurs non valides
    if (val === undefined || val === null || isNaN(val) || Math.abs(val) > 1e18 || Math.abs(val) < 1e-12) { 
        return (decimals === 0 ? 'N/A' : '--.--') + suffix; 
    }
    return val.toFixed(decimals) + suffix;
};

const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || Math.abs(val) > 1e18 || Math.abs(val) < 1e-12) {
        return 'N/A'; 
    }
    return val.toExponential(decimals) + suffix;
};


// --- BLOC 2 : ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE ---

let ukf = null; 
let currentCelestialBody = 'EARTH';
let netherMode = false; 

let currentPosition = { 
    lat: 43.2964,   
    lon: 5.3697,
    acc: 10.0,      
    spd: 0.0,
    alt: 0.0
};
let kAlt = 0.0; // Altitude filtrée (m)
let currentSpeed = 0.0; // Vitesse filtrée (m/s)

let G_ACC = G_STD; // Gravité locale WGS84 (m/s²)

// Variables de Météo et Physique
let lastT_K = TEMP_SEA_LEVEL_K; 
let lastP_hPa = BARO_ALT_REF_HPA; 
let lastH_perc = 0.5; 
let lastWeatherData = null;

let currentAirDensity = RHO_SEA_LEVEL; 
let currentSpeedOfSound = getSpeedOfSound(TEMP_SEA_LEVEL_K); // Valeur ISA par défaut
let currentMass = 70.0; 

let lServH = 0; 
let lLocH = 0;  


// --- BLOC 3 : MODÈLES PHYSIQUES ET GÉODÉSIQUES (Fonctions inchangées) ---

function getSpeedOfSound(tempK) {
    // Protéger contre le NaN si tempK est invalide
    if (tempK <= 0 || isNaN(tempK)) return NaN; 
    return Math.sqrt(GAMMA_AIR * R_SPECIFIC_AIR * tempK);
}

function getAirDensity(p_Pa, t_K, h_rel) {
    if (t_K <= 0 || isNaN(p_Pa) || isNaN(t_K) || isNaN(h_rel)) return NaN; 
    const T_c = t_K - 273.15;
    const Es = 610.7 * Math.pow(10, (7.5 * T_c) / (237.3 + T_c)) * 100;
    const E = h_rel * Es; 
    const R_WATER_VAPOR = 461.5; 
    const DENSITY_AIR = (p_Pa - E) / (R_SPECIFIC_AIR * t_K) + (E / (R_WATER_VAPOR * t_K));
    return DENSITY_AIR;
}

function updateCelestialBody(body, altitude) {
    // ... (Logique WGS84 inchangée)
    let G_ACC_NEW = G_STD;
    if (body === 'EARTH') {
        const phi = currentPosition.lat * D2R;
        const sin_phi = Math.sin(phi);
        const factor_lat = (1 + WGS84_BETA * sin_phi * sin_phi) / Math.sqrt(1 - WGS84_E2 * sin_phi * sin_phi);
        G_ACC_NEW = WGS84_G_EQUATOR * factor_lat * (1 - 2 * altitude / WGS84_A);
    }
    G_ACC = G_ACC_NEW;
    // Mise à jour de l'affichage de la Gravité Locale
    if ($('gravity-local')) $('gravity-local').textContent = `${G_ACC.toFixed(4)} m/s²`;
    return { G_ACC_NEW };
}

function calculateBioSVT(tempC, kAlt, humidityPerc, pressurePa, sunAltitudeRad) {
    // ... (Logique inchangée)
    const tempK = tempC + 273.15;
    const humidity = humidityPerc / 100.0;
    
    const T = tempC;
    const Es = 6.11 * Math.pow(10, (7.5 * T) / (237.3 + T)) * 100; 
    const E = humidity * Es; 
    const gamma = Math.log(E / 611);
    const dewPoint = (237.3 * gamma) / (7.5 - gamma);
    
    const DENSITY_AIR = getAirDensity(pressurePa, tempK, humidity); 

    return {
        dewPoint: dewPoint,
        humideAirDensity: DENSITY_AIR,
        solarFactor: Math.max(0, Math.sin(sunAltitudeRad))
    };
}


// --- BLOC 4 : LOGIQUE API ET TEMPS (NTP/MÉTÉO) ---

async function syncH() {
    try {
        const response = await fetch(SERVER_TIME_ENDPOINT);
        const data = await response.json();
        lServH = new Date(data.utc_datetime).getTime();
        lLocH = Date.now();
        if ($('local-time')) $('local-time').textContent = getCDate(lServH, lLocH).toLocaleTimeString('fr-FR');

    } catch (error) {
        // En cas d'échec, utilise l'heure locale par défaut
        lServH = 0; 
        lLocH = 0; 
        if ($('local-time')) $('local-time').textContent = new Date().toLocaleTimeString('fr-FR') + ' (LOCAL)';
        if ($('date-display')) $('date-display').textContent = new Date().toLocaleDateString('fr-FR');
    }
}

function getCDate(serverTimeMs, localTimeMs) {
    if (serverTimeMs > 0 && localTimeMs > 0) {
        const offset = serverTimeMs - localTimeMs;
        return new Date(Date.now() + offset);
    }
    return new Date();
}

async function fetchWeather(lat, lon) {
    try {
        const response = await fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`);
        if (!response.ok) {
             throw new Error(`Erreur API: ${response.status}`);
        }
        const data = await response.json();
        
        // Calcul des métriques pour les valeurs globales
        data.pressure_Pa = data.pressure_hPa * 100;
        data.tempK = data.tempC + 273.15;
        data.air_density = getAirDensity(data.pressure_Pa, data.tempK, data.humidity_perc / 100);
        data.speed_of_sound = getSpeedOfSound(data.tempK);
        
        return data;
    } catch (error) {
        return null;
    }
}


// --- BLOC 5 : GESTION DES CAPTEURS ET UKF (BOUCLE RAPIDE) ---

/**
 * Mise à jour DOM Météo (utilise les IDs harmonisés)
 * @param {object} data - Données météo complètes
 * @param {boolean} isOfflineDefault - Indique si ce sont des valeurs ISA par défaut
 */
function updateWeatherDOM(data, isOfflineDefault = false) {
    lastP_hPa = data.pressure_hPa;
    lastT_K = data.tempK;
    lastH_perc = data.humidity_perc / 100.0;
    currentAirDensity = data.air_density;
    currentSpeedOfSound = data.speed_of_sound;
    
    // Utilisation des IDs DOM corrigés
    if ($('air-temp-c')) $('air-temp-c').textContent = `${data.tempC.toFixed(1)} °C`;
    if ($('pressure-hpa')) $('pressure-hpa').textContent = `${data.pressure_hPa.toFixed(0)} hPa`;
    if ($('humidity-perc')) $('humidity-perc').textContent = `${data.humidity_perc} %`;
    
    // Affichage robustes pour les valeurs calculées
    if ($('air-density')) $('air-density').textContent = dataOrDefault(data.air_density, 3) + ' kg/m³';
    if($('speed-of-sound-calc')) $('speed-of-sound-calc').textContent = dataOrDefault(currentSpeedOfSound, 2) + ` m/s (${isOfflineDefault ? 'ISA' : 'Météo'})`;
}

function updateGPSState(position) {
    // ... (Récupération des données GNSS)
    const lat = position.coords.latitude;
    const lon = position.coords.longitude;
    const acc = position.coords.accuracy;
    const alt = position.coords.altitude || 0.0;
    const spd = position.coords.speed || 0.0;

    currentPosition = { lat, lon, acc, spd, alt };

    // 1. Prédiction/Correction UKF
    if (ukf) {
        if ($('ekf-status')) $('ekf-status').textContent = 'Actif 🟢';

        const accelX = 0, accelY = 0, accelZ = 0; 
        const gyroX = 0, gyroY = 0, gyroZ = 0; 
        ukf.predict(accelX, accelY, accelZ, gyroX, gyroY, gyroZ); 
        
        ukf.update(currentPosition, 'GNSS'); 
        
        const state = ukf.getState();
        kAlt = netherMode ? state.alt / 8.0 : state.alt;
        currentSpeed = netherMode ? state.speed / 8.0 : state.speed;

        // Mise à jour des données d'état filtré (EKF/UKF)
        if ($('lat-ekf')) $('lat-ekf').textContent = dataOrDefault(state.lat, 6) + ' °';
        if ($('lon-ekf')) $('lon-ekf').textContent = dataOrDefault(state.lon, 6) + ' °';
        if ($('alt-ekf')) $('alt-ekf').textContent = dataOrDefault(kAlt, 2) + ' m';
        
        // Mise à jour des métriques de débogage UKF
        if ($('ukf-v-uncert')) $('ukf-v-uncert').textContent = dataOrDefault(state.kUncert, 6) + ' m/s';
        if ($('ukf-alt-sigma')) $('ukf-alt-sigma').textContent = dataOrDefault(state.kUncert, 6) + ' m';
        if ($('baro-bias')) $('baro-bias').textContent = dataOrDefault(state.baroBias, 4) + ' m'; // Le biais de l'UKF est en mètres

        // Affichage de la vitesse verticale (vD)
        if ($('vertical-speed-ekf')) $('vertical-speed-ekf').textContent = dataOrDefault(-state.vD, 2) + ' m/s';

        
    } else {
        if ($('ekf-status')) $('ekf-status').textContent = 'DÉSACTIVÉ 🔴';
        kAlt = netherMode ? alt / 8.0 : alt;
        currentSpeed = netherMode ? spd / 8.0 : spd;
    }
    
    // 2. Mise à jour du DOM (Boucle rapide)
    
    // Gravité et correction WGS84
    updateCelestialBody(currentCelestialBody, kAlt);
    if ($('gravity-base')) $('gravity-base').textContent = `${G_STD.toFixed(4)} m/s²`;
    
    // Vitesse (Filtrée)
    if ($('speed-stable')) $('speed-stable').textContent = dataOrDefault(currentSpeed * KMH_MS, 2) + ' km/h';
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = dataOrDefault(currentSpeed, 2) + ' m/s';
    if ($('speed-raw-ms')) $('speed-raw-ms').textContent = dataOrDefault(currentPosition.spd, 2) + ' m/s';

    // Précision GNSS
    if ($('acc-horiz')) $('acc-horiz').textContent = dataOrDefault(currentPosition.acc, 2) + ' m';
}

function errorGPS(err) {
    if ($('gnss-status')) $('gnss-status').textContent = `ERREUR GPS (${err.code}) 🔴`;
}

let watchId = null;
function initGPS() {
    // ... (Logique inchangée pour l'initialisation GPS)
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
    }
}

function activateDeviceMotion() {
    if ($('imu-status')) $('imu-status').textContent = 'Actif 🟢';
    // Logique d'activation des écouteurs 'devicemotion'
}


// --- BLOC 6 : BOUCLE LENTE (ASTRO, MÉTÉO, DOM COMPLEXE) ---

function slowUpdateLoop() {
    const lat = currentPosition.lat;
    const lon = currentPosition.lon;
    const now = getCDate(lServH, lLocH);

    // 1. Mise à jour de l'horloge
    if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString('fr-FR');
    if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
    if ($('date-display-astro')) $('date-display-astro').textContent = now.toLocaleDateString('fr-FR');


    // 2. Mise à jour Météo (toutes les 5 minutes)
    if (Date.now() - (lastWeatherData ? lastWeatherData.timestamp : 0) > 300000) {
        if ($('statut-meteo')) $('statut-meteo').textContent = 'Mise à jour... 🟡';
        fetchWeather(lat, lon).then(data => {
            if (data) {
                // Succès : met à jour les données et le statut
                lastWeatherData = { ...data, timestamp: Date.now() };
                updateWeatherDOM(data);
                if ($('statut-meteo')) $('statut-meteo').textContent = 'Actif 🟢';
                
                // 3. Mise à jour Astro et BioSVT
                if (typeof getAstroData === 'function') {
                    const astroData = getAstroData(now, lat, lon);
                    const sunAltitudeRad = astroData.sun.altitude; 

                    const pressurePa = lastWeatherData.pressure_hPa * 100;
                    const bioSim = calculateBioSVT(lastWeatherData.tempC, kAlt, lastWeatherData.humidity_perc, pressurePa, sunAltitudeRad);
                    
                    if ($('dew-point')) $('dew-point').textContent = `${dataOrDefault(bioSim.dewPoint, 1)} °C`;
                    if ($('air-density')) $('air-density').textContent = `${dataOrDefault(bioSim.humideAirDensity, 3)} kg/m³ (Calculé)`;
                    
                    // Mise à jour DOM Astro
                    if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(sunAltitudeRad * R2D, 2) + ' °';
                    if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(astroData.sun.azimuth * R2D, 2) + ' °';
                    if ($('moon-phase-name')) $('moon-phase-name').textContent = astroData.moon.phaseName;
                    if ($('moon-illuminated')) $('moon-illuminated').textContent = dataOrDefault(astroData.moon.fraction * 100, 1) + ' %';
                    if ($('tst')) $('tst').textContent = astroData.TST_HRS ? dataOrDefault(astroData.TST_HRS, 4) : 'N/A';
                }
            } else {
                 // Échec : Réinitialise l'affichage au statut d'échec, mais PRÉSERVE les valeurs globales (lastT_K, etc.)
                 if ($('statut-meteo')) $('statut-meteo').textContent = 'ÉCHEC (Proxy) 🔴';
            }
        });
    }
}


// --- BLOC 7 : DÉMARRAGE ET ÉVÉNEMENTS (DOCUMENT READY) ---

window.addEventListener('load', () => {

    // 1. Initialisation UKF (doit se faire avant l'utilisation)
    if (typeof ProfessionalUKF !== 'undefined' && typeof math !== 'undefined') {
        try {
            ukf = new ProfessionalUKF(); 
            if ($('ekf-status')) $('ekf-status').textContent = 'Initialisé 🟢';
        } catch (e) {
            if ($('ekf-status')) $('ekf-status').textContent = 'ERREUR UKF 🔴';
        }
    } else {
        if ($('ekf-status')) $('ekf-status').textContent = 'DÉSACTIVÉ 🔴';
    }
    
    // 2. Initialisation des valeurs par défaut ISA (ROBUSTESSE ANTI-NaN)
    const initialISA = {
        tempC: TEMP_SEA_LEVEL_K - 273.15, // 15.0
        pressure_hPa: BARO_ALT_REF_HPA, // 1013.25
        humidity_perc: 50,
        tempK: TEMP_SEA_LEVEL_K,
        air_density: RHO_SEA_LEVEL,
        speed_of_sound: getSpeedOfSound(TEMP_SEA_LEVEL_K)
    };
    updateWeatherDOM(initialISA, true); // Affiche 340.30 m/s (ISA)
    if ($('statut-meteo')) $('statut-meteo').textContent = 'Défaut (ISA) 🟡';
    
    // 3. Initialisation des constantes
    if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
    if ($('const-c')) $('const-c').textContent = `${C_L} m/s`;
    if ($('const-G')) $('const-G').textContent = dataOrDefaultExp(G_U, 6) + ' m³/kg/s²';
    updateCelestialBody(currentCelestialBody, kAlt); // Initialise l'affichage de la gravité locale
    
    // 4. Démarrer la synchro NTP
    syncH();
    
    // 5. Démarrer les capteurs
    initGPS(); 
    
    // 6. Boucles de mise à jour
    setInterval(slowUpdateLoop, DOM_SLOW_UPDATE_MS); 

    // 7. Écouteurs d'événements (Interactions Utilisateur - inchangés)
    if ($('mass-input')) $('mass-input').addEventListener('input', (e) => {
        currentMass = parseFloat(e.target.value) || 70.0;
        if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
    });

    if ($('celestial-body-select')) $('celestial-body-select').addEventListener('change', (e) => {
        currentCelestialBody = e.target.value;
        updateCelestialBody(currentCelestialBody, kAlt);
    });

    const netherToggleBtn = $('nether-toggle-btn');
    if (netherToggleBtn) netherToggleBtn.addEventListener('click', () => {
        netherMode = !netherMode;
        netherToggleBtn.textContent = `Mode Nether: ${netherMode ? 'ACTIVÉ (1:8) 🔥' : 'DÉSACTIVÉ (1:1) 🌍'}`;
    });
    
    if ($('ukf-reactivity-mode')) $('ukf-reactivity-mode').addEventListener('change', (e) => {
        // ... (Logique de changement de Q pour l'UKF)
    });
    
    const activateButton = document.getElementById('activate-sensors-btn');
    if (activateButton) {
        if (typeof activateDeviceMotion === 'function') {
            activateButton.addEventListener('click', activateDeviceMotion); 
        } 
    }
});
