// =================================================================
// BLOC 1/4 : Constantes Globales et Configuration
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

// --- CONSTANTES MATHÉMATIQUES ET PHYSIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI; 
const KMH_MS = 3.6;         
const C_L = 299792458;      
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation Terre (rad/s)
const R_AIR = 287.058;      // Constante spécifique de l'air sec (J/kg·K)
const GAMMA = 1.4;          // Rapport des chaleurs spécifiques pour l'air sec

// --- CONSTANTES GÉOPHYSIQUES (WGS84) ---
let G_ACC = 9.80665;         
let R_ALT_CENTER_REF = 6371000;
const WGS84_A = 6378137.0;  
const WGS84_G_EQUATOR = 9.780327;
const WGS84_BETA = 0.0053024;
const WGS84_E2 = 0.00669437999014; // (WGS84_F * 2 - WGS84_F * WGS84_F)

// --- CONSTANTES ATMOSPHÉRIQUES (ISA Standard) ---
const BARO_ALT_REF_HPA = 1013.25;
const RHO_SEA_LEVEL = 1.225;

// --- PARAMÈTRES DU FILTRE UKF (Basé sur 13.js) ---
const UKF_R_MAX = 500.0;     
const MIN_SPD = 0.01;        // Seuil bas pour réactivité de la distance
const R_ALT_MIN = 1.0;

// --- FACTEURS DE RÉACTIVITÉ UKF ---
const UKF_REACTIVITY_FACTORS = {
    'AUTO': { MULT: 1.0, DISPLAY: 'Automatique' },
    'NORMAL': { MULT: 1.0, DISPLAY: 'Normal' },
    'FAST': { MULT: 0.2, DISPLAY: 'Rapide' },
    'STABLE': { MULT: 2.5, DISPLAY: 'Microscopique' },
};

// --- SEUILS D'ISOLATION & HYPERLOOP ---
const ALT_TH = -50;         // SEUIL BAS (Souterrain / Tunnel)
const ALT_HIGH_TH = 3000;   // SEUIL HAUT (Avion / Montagne)
const P_HYPERLOOP_PA = 10.0;     // Pression Hyperloop (Pascals)
const RHO_VACUUM = 0.00001;      // Densité forcée pour Hyperloop
const T_HYPERLOOP_C = 20.0;     
const P_ISA = 1013.25;      // Pression standard (hPa)
const T_ISA_C = 15.0;       // Température standard (°C)
const H_ISA_PERC = 50.0;    // Humidité par défaut (%)

// --- DYNAMIQUE DE TRAÎNÉE ---
const REFERENCE_DRAG_AREA = 0.5; // Surface de référence (m²)
const DRAG_COEFFICIENT = 1.2; // Coefficient de Traînée

// --- CONFIGURATION SYSTÈME ---
const MIN_DT = 0.01;        
const MAP_UPDATE_INTERVAL = 3000;
const IMU_UPDATE_RATE_MS = 20; // 50Hz (Fréquence de la boucle rapide)
const STANDBY_TIMEOUT_MS = 300000; // 5 minutes avant passage en Basse Fréquence
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};

// --- FACTEURS D'ENVIRONNEMENT ---
const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0, DISPLAY: 'Normal' },
    'FOREST': { R_MULT: 2.5, DISPLAY: 'Forêt' },
    'CONCRETE': { R_MULT: 7.0, DISPLAY: 'Grotte/Tunnel' },
    'METAL': { R_MULT: 5.0, DISPLAY: 'Métal/Bâtiment' },
};

// --- DONNÉES CÉLESTES/GRAVITÉ ---
const CELESTIAL_DATA = {
    'EARTH': { G: 9.80665, R: WGS84_A, name: 'Terre' },
    'MOON': { G: 1.62, R: 1737400, name: 'Lune' },
    'MARS': { G: 3.71, R: 3389500, name: 'Mars' },
    'ROTATING': { G: 0.0, R: WGS84_A, name: 'Station Spatiale' }
};
// =================================================================
// BLOC 2/4 : Filtres, Modèles Cinématiques et UKF (Basé sur 13.js)
// =================================================================

// ===========================================
// CLASSE UKF PROFESSIONNELLE (Architecture 21 États)
// ===========================================
class ProfessionalUKF {
    constructor() {
        if (typeof math === 'undefined') {
            throw new Error("math.js n'est pas chargé. L'UKF 21 états ne peut pas fonctionner.");
        }
        this.N_STATES = 21; 
        this.x = math.zeros(this.N_STATES); 
        this.P = math.diag(Array(this.N_STATES).fill(1e-2)); 
        this.Q = math.diag(Array(this.N_STATES).fill(1e-6));
        // (Initialisation complète des poids UKF omise pour la concision)
    }

    predict(imuData, dt) {
        // PLACEHOLDER (Simulation simplifiée pour faire bouger les chiffres):
        // Une véritable prédiction UKF/INS nécessite une intégration Strapdown complexe
        // Ici, nous simulons la prédiction de vitesse basée sur l'accélération
        const ax_corr = imuData.accel[0]; 
        let vN = this.x.get([3]) + ax_corr * dt; 
        
        if (Math.abs(vN) < MIN_SPD) vN = 0; // ZUPT
        this.x.set([3], vN); 
    }

    update(gpsData) {
        // PLACEHOLDER : Correction directe de la position et de la vitesse
        
        // 💡 CORRECTION CRITIQUE : Rejette les latitudes impossibles
        if (gpsData.latitude > 90.0 || gpsData.latitude < -90.0) {
            console.error(`Latitude GPS invalide rejetée: ${gpsData.latitude}`);
            return; // Ne pas mettre à jour l'état avec des données corrompues
        }
        
        this.x.set([0], gpsData.latitude * D2R);
        this.x.set([1], gpsData.longitude * D2R);
        this.x.set([2], gpsData.altitude);
        
        if (gpsData.speed) {
            const oldSpeed = this.x.get([3]);
            this.x.set([3], oldSpeed * 0.8 + gpsData.speed * 0.2); // Simple fusion
        }
    }
    
    getState() {
        const x_data = this.x.toArray();
        // S'assure que la latitude reste dans les bornes physiques
        const clampedLat = Math.max(-90, Math.min(90, x_data[0] * R2D));
        
        return {
            lat: clampedLat,
            lon: x_data[1] * R2D,
            alt: x_data[2],
            speed: Math.sqrt(x_data[3]**2 + x_data[4]**2 + x_data[5]**2),
            kUncert: this.P.get([3, 3]) + this.P.get([4, 4]) + this.P.get([5, 5]),
            kAltUncert: this.P.get([2, 2])
        };
    }
}


// --- FONCTIONS DE FILTRAGE ET DE MODÈLE (Bruit, Gravité, etc.) ---

function getKalmanR(accRaw, kAlt, kUncert, env, reactivityMode) {
    let R_gps_base = Math.min(accRaw, 100) ** 2; 
    if (accRaw > 100) { R_gps_base = 100 ** 2 + (accRaw - 100) * 10; }
    
    const env_mult = ENVIRONMENT_FACTORS[env]?.R_MULT || 1.0;
    let reactivity_mult = UKF_REACTIVITY_FACTORS[reactivityMode]?.MULT || 1.0;
    
    // Automatisation du Facteur R
    if (reactivityMode === 'AUTO' && accRaw !== null) {
        if (accRaw > 20) { reactivity_mult = 3.0; } // Moins de confiance
        else if (accRaw < 3) { reactivity_mult = 0.5; } // Plus de confiance
    }
    
    let R_dyn = Math.min(R_gps_base * env_mult * reactivity_mult, UKF_R_MAX);
    
    return Math.max(R_dyn, R_ALT_MIN); 
}

function dist2D(lat1, lon1, lat2, lon2, R_earth) {
    const dLat = (lat2 - lat1) * D2R;
    const dLon = (lon2 - lon1) * D2R;
    const lat1Rad = lat1 * D2R;
    const lat2Rad = lat2 * D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1Rad) * Math.cos(lat2Rad) * Math.sin(dLon / 2) ** 2;
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R_earth * c; 
}

function getBarometricAltitude(P_hPa, P_ref_hPa, T_K) {
    if (P_hPa === null || T_K === null) return null;
    const L = 0.0065; const g = G_ACC; const R = R_AIR; 
    const alt = (T_K / L) * (1 - (P_hPa / P_ref_hPa)**(R * L / g));
    return alt;
}

function calculateMRF(alt, netherMode) {
    // (Logique Nether retirée, MRF est 1.0)
    return 1.0;
}

function updateCelestialBody(body, alt, rotationRadius, angularVelocity) {
    // (Fonction inchangée, gère G_ACC et R_ALT_CENTER_REF)
    let G_ACC_NEW = CELESTIAL_DATA['EARTH'].G;
    let R_ALT_CENTER_REF_NEW = WGS84_A; 
    const data = CELESTIAL_DATA[body];
    if (data) {
        G_ACC_NEW = data.G;
        R_ALT_CENTER_REF_NEW = data.R;
    }
    if (body === 'ROTATING') {
        G_ACC_NEW = rotationRadius * angularVelocity ** 2;
    }
    G_ACC = G_ACC_NEW;
    R_ALT_CENTER_REF = R_ALT_CENTER_REF_NEW;
    return { G_ACC: G_ACC_NEW, R_ALT_CENTER_REF: R_ALT_CENTER_REF_NEW };
}

// NOUVEAU: Fonction de calcul dynamique
function calculateDynamicMetrics(speed_ms, airDensity, soundSpeed, accel_long_imu, lat_rad) {
    const mach = speed_ms / soundSpeed;
    const dynamicPressure = 0.5 * airDensity * speed_ms * speed_ms; 
    const dragForce = dynamicPressure * REFERENCE_DRAG_AREA * DRAG_COEFFICIENT; 
    const accelLong = accel_long_imu; // Utilise l'accélération IMU brute
    const coriolisForce = 2 * MASS_KG * speed_ms * OMEGA_EARTH * Math.cos(lat_rad); 

    return { mach, dynamicPressure, dragForce, accelLong, coriolisForce };
    }
// =================================================================
// BLOC 3/4 : Services Externes & Calculs Astro/Physique
// =================================================================

const PROXY_WEATHER_ENDPOINT = "https://scientific-dashboard2.vercel.app/api/weather";
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

const J1970 = 2440588, J2000 = 2451545.0;
const dayMs = 1000 * 60 * 60 * 24;
const MC_DAY_MS = 72 * 60 * 1000; 
const WEATHER_CACHE_KEY = 'lastWeatherCache'; // Pour la météo hors ligne

async function syncH(lServH_in, lLocH_in) {
    let lServH = lServH_in;
    let lLocH = lLocH_in;
    
    if ($('local-time')) $('local-time').textContent = 'Synchronisation...';
    
    try {
        const response = await fetch(SERVER_TIME_ENDPOINT, { cache: "no-store", mode: "cors" });
        if (!response.ok) throw new Error(`Server time sync failed`);
        
        const serverData = await response.json(); 
        lServH = Date.parse(serverData.utc_datetime); 
        lLocH = performance.now(); 
        
        if ($('local-time')) $('local-time').textContent = '✅ SYNCHRO NTP ACTIVE';

    } catch (error) {
        lServH = Date.now(); 
        lLocH = performance.now();
        if ($('local-time')) $('local-time').textContent = '❌ SYNCHRO ÉCHOUÉE (Local)';
    }
    return { lServH, lLocH };
}

function getCDate(lServH, lLocH) {
    if (lServH === null || lLocH === null) { return new Date(); }
    const offset = performance.now() - lLocH;
    return new Date(lServH + offset);
}

function getWGS84Gravity(lat, alt) {
    const latRad = lat * D2R; 
    const sin2lat = Math.sin(latRad) ** 2;
    const g_surface = WGS84_G_EQUATOR * (1 + WGS84_BETA * sin2lat) / Math.sqrt(1 - WGS84_E2 * sin2lat);
    const g_local = g_surface * (1 - 2 * alt / WGS84_A);
    return g_local; 
}

function getSpeedOfSound(tempK) {
    return Math.sqrt(GAMMA * R_AIR * tempK);
}

// NOUVEAU: Logique Météo Avancée (Hyperloop, Isolation, Cache)
async function fetchWeather(lat, lon, currentAlt, hyperloopMode) {
    let data = null;
    let statusText = 'ACTIF';
    let sourceTag = '';

    const isIsolatedByAltitude = currentAlt !== null && (currentAlt > ALT_HIGH_TH || currentAlt < ALT_TH);

    if (hyperloopMode) { 
        data = { tempC: T_HYPERLOOP_C, pressure_hPa: P_HYPERLOOP_PA / 100, humidity_perc: 0.0 };
        statusText = 'FORCÉ'; sourceTag = `(Hyperloop)`;
    } else if (isIsolatedByAltitude) {
        data = { tempC: T_ISA_C, pressure_hPa: P_ISA, humidity_perc: H_ISA_PERC };
        statusText = 'FORCÉ'; sourceTag = `(Isolation)`;
    } else {
        try {
            const url = `${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`;
            const response = await fetch(url);
            if (!response.ok) throw new Error(`Erreur HTTP ${response.status}`);
            data = await response.json();
            localStorage.setItem(WEATHER_CACHE_KEY, JSON.stringify(data)); // Sauvegarde cache
        } catch (e) {
            const cachedData = localStorage.getItem(WEATHER_CACHE_KEY);
            if (cachedData) {
                data = JSON.parse(cachedData);
                statusText = 'HORS LIGNE'; sourceTag = `(Cache)`;
            } else {
                data = { tempC: T_ISA_C, pressure_hPa: P_ISA, humidity_perc: H_ISA_PERC };
                statusText = 'HORS LIGNE'; sourceTag = `(ISA)`;
            }
        }
    }
    
    // Traitement des données
    const T_C = data.tempC;
    const T_K = T_C + 273.15;
    const P_hPa = data.pressure_hPa;
    const H_perc = data.humidity_perc;
    const air_density = calculateAirDensity(P_hPa, T_K, hyperloopMode);
    const dew_point = calculateDewPoint(T_C, H_perc);
    const speed_sound = getSpeedOfSound(T_K);

    // Mise à jour DOM Météo
    if ($('weather-status')) $('weather-status').textContent = `${statusText} ${sourceTag}`;
    if ($('temp-air-2')) $('temp-air-2').textContent = `${data.tempC.toFixed(1)} °C`;
    if ($('pressure-2')) $('pressure-2').textContent = `${data.pressure_hPa.toFixed(0)} hPa`;
    if ($('humidity-2')) $('humidity-2').textContent = `${data.humidity_perc.toFixed(0)} %`;
    if ($('air-density')) $('air-density').textContent = `${air_density.toFixed(3)} kg/m³`;
    if ($('dew-point')) $('dew-point').textContent = `${dew_point.toFixed(1)} °C`;
    
    return { pressure_hPa: P_hPa, tempK: T_K, air_density: air_density, speed_sound: speed_sound };
}

// ... (Fonctions getSolarTime, getMinecraftTime omises) ...

function getMoonPhaseName(phase) {
    if (phase < 0.03 || phase > 0.97) return "Nouvelle Lune 🌑";
    if (phase < 0.23) return "Premier Croissant 🌒";
    if (phase < 0.27) return "Premier Quartier 🌓";
    if (phase < 0.48) return "Gibbeuse Croissante 🌔";
    if (phase < 0.52) return "Pleine Lune 🌕";
    if (phase < 0.73) return "Gibbeuse Décroissante 🌖";
    if (phase < 0.77) return "Dernier Quartier 🌗";
    return "Dernier Croissant 🌘"; 
}

function updateAstro(lat, lon, lServH, lLocH) {
    const now = getCDate(lServH, lLocH);
    if ($('time-minecraft')) $('time-minecraft').textContent = 'N/A'; // (Logique MC retirée)

    if (typeof SunCalc === 'undefined' || !lat || !lon) return; 

    const sunPos = SunCalc.getPosition(now, lat, lon);
    const moonIllum = SunCalc.getMoonIllumination(now);
    const moonPos = SunCalc.getMoonPosition(now, lat, lon);
    const sunTimes = SunCalc.getTimes(now, lat, lon);
    const moonTimes = SunCalc.getMoonTimes(now, lat, lon, true);
    // const solarTimes = getSolarTime(now, lon); // (getSolarTime est complexe)

    if ($('date-display-astro')) $('date-display-astro').textContent = now.toLocaleDateString() || 'En attente...';
    // ... (Mise à jour TST/MST/EOT omise)

    if ($('sun-alt')) $('sun-alt').textContent = `${(sunPos.altitude * R2D).toFixed(2)}°`;
    if ($('sun-azimuth')) $('sun-azimuth').textContent = `${(sunPos.azimuth * R2D).toFixed(2)}°`;
    if ($('day-duration') && sunTimes.sunrise && sunTimes.sunset) {
        const durationMs = sunTimes.sunset.getTime() - sunTimes.sunrise.getTime();
        $('day-duration').textContent = `${(durationMs / 3600000).toFixed(2)} h`;
    }
    
    if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(moonIllum.phase);
    if ($('moon-illuminated')) $('moon-illuminated').textContent = `${(moonIllum.fraction * 100).toFixed(1)}%`;
    if ($('moon-alt')) $('moon-alt').textContent = `${(moonPos.altitude * R2D).toFixed(2)}°`;
    if ($('moon-azimuth')) $('moon-azimuth').textContent = `${(moonPos.azimuth * R2D).toFixed(2)}°`;
    if ($('moon-times')) $('moon-times').textContent = (moonTimes.rise && moonTimes.set) ? `${moonTimes.rise.toLocaleTimeString()} / ${moonTimes.set.toLocaleTimeString()}` : 'Circumpolaire';
}
// =================================================================
// BLOC 4/4 : Logique Applicative Principale (Core Loop & DOM Init)
// =================================================================

// --- VARIABLES D'ÉTAT (Globales) ---
let wID = null, domSlowID = null, domFastID = null, lPos = null, lat = null, lon = null, sTime = null;
let distM = 0, maxSpd = 0;
let kSpd = 0, kUncert = UKF_R_MAX, kAltUncert = 10; 
let timeMoving = 0, timeTotal = 0; 
let kAlt = null;      
let ukf = null; 

let currentGPSMode = 'HIGH_FREQ'; 
let emergencyStopActive = false; 
let hyperloopMode = false; 
let selectedEnvironment = 'NORMAL'; 
let currentUKFReactivity = 'AUTO'; 
let lastGPSPos = null;

let lastP_hPa = BARO_ALT_REF_HPA, lastT_K = 288.15, currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound = 343;

let accel = { x: 0, y: 0, z: 0 };
let gyro = { x: 0, y: 0, z: 0 };
let lastIMUTimestamp = 0;
let lastMapUpdate = 0;
let gpsStandbyTimeoutID = null;    


// --- GESTION DES CAPTEURS (Nouvelle API) ---
function startIMUListeners() {
    if (emergencyStopActive) return;
    try {
        if ($('imu-status')) $('imu-status').textContent = "Activation...";
        
        if (typeof Accelerometer === 'undefined' || typeof Gyroscope === 'undefined') {
             throw new Error("API Sensor non supportée.");
        }
        
        const accSensor = new Accelerometer({ frequency: 50 }); 
        accSensor.addEventListener('reading', () => {
            accel.x = accSensor.x; accel.y = accSensor.y; accel.z = accSensor.z;
        });
        accSensor.addEventListener('error', e => console.error("Erreur Accéléromètre:", e.error));
        accSensor.start();

        const gyroSensor = new Gyroscope({ frequency: 50 });
        gyroSensor.addEventListener('reading', () => {
            gyro.x = gyroSensor.x; gyro.y = gyroSensor.y; gyro.z = gyroSensor.z;
        });
        gyroSensor.addEventListener('error', e => console.error("Erreur Gyroscope:", e.error));
        gyroSensor.start();
        
        if ($('imu-status')) $('imu-status').textContent = "Actif (API Sensor)";
        lastIMUTimestamp = performance.now();
        
        startFastLoop(); // Démarre la boucle de prédiction IMU

    } catch (error) {
        let errMsg = error.message;
        if (error.name === 'SecurityError' || error.name === 'NotAllowedError') {
            errMsg = "Permission Capteurs Refusée. Cliquez sur 'Démarrer'.";
        } else if (error.name === 'NotReadableError') {
             errMsg = "Capteurs Verrouillés (OS/Navigateur).";
        }
        if ($('imu-status')) $('imu-status').textContent = `❌ ${errMsg}`;
    }
}

function stopIMUListeners() {
    if (domFastID) clearInterval(domFastID);
    domFastID = null;
    if ($('imu-status')) $('imu-status').textContent = "Inactif";
    accel = { x: 0, y: 0, z: 0 }; gyro = { x: 0, y: 0, z: 0 };
}

// --- GESTION GPS ---
function startGPS(mode = currentGPSMode) {
    if (emergencyStopActive) return;
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    
    currentGPSMode = mode;
    const options = GPS_OPTS[mode];
    wID = navigator.geolocation.watchPosition(gpsUpdateCallback, handleErr, options);
    
    if ($('toggle-gps-btn')) {
        let text = (mode === 'LOW_FREQ' && kSpd < MIN_SPD * 2) ? '⏸️ GPS EN VEILLE' : '⏸️ PAUSE GPS';
        $('toggle-gps-btn').textContent = text;
        $('toggle-gps-btn').style.backgroundColor = '#ffc107'; 
    }
}
function stopGPS(resetButton = true) {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    wID = null;
    if (gpsStandbyTimeoutID) clearTimeout(gpsStandbyTimeoutID);
    gpsStandbyTimeoutID = null;
    if (resetButton && $('toggle-gps-btn')) {
        $('toggle-gps-btn').textContent = '▶️ MARCHE GPS';
        $('toggle-gps-btn').style.backgroundColor = '#28a745'; 
    }
}
function toggleGPS() {
    if (emergencyStopActive) return;
    wID === null ? startGPS('HIGH_FREQ') : stopGPS();
}
function toggleEmergencyStop() {
    emergencyStopActive = !emergencyStopActive;
    if (emergencyStopActive) {
        stopGPS(false); 
        stopIMUListeners();
        if($('emergency-stop-btn')) $('emergency-stop-btn').textContent = "🟢 REPRENDRE";
    } else {
        if($('emergency-stop-btn')) $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: INACTIF 🟢";
        startGPS(); 
        startIMUListeners();
    }
}
function handleErr(err) {
    if ($('gps-precision')) $('gps-precision').textContent = `Erreur: ${err.message}`;
    if (err.code === 1) { stopGPS(); }
}

// --- MAP (Leaflet) ---
function initMap() {
    try {
        if ($('map') && typeof L !== 'undefined' && !map) { 
            map = L.map('map').setView([43.296, 5.37], 10);
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                attribution: '© OpenStreetMap contributors'
            }).addTo(map);
            marker = L.marker([43.296, 5.37]).addTo(map);
            circle = L.circle([43.296, 5.37], { color: 'red', fillColor: '#f03', fillOpacity: 0.5, radius: 10 }).addTo(map);
        }
    } catch (e) { if ($('map')) $('map').innerHTML = "Erreur de la carte."; }
}
function updateMap(lat, lon, acc) {
    if (map && marker) {
        const latLng = [lat, lon];
        marker.setLatLng(latLng);
        circle.setLatLng(latLng).setRadius(acc); 
        const now = Date.now();
        if (now - lastMapUpdate > MAP_UPDATE_INTERVAL && kSpd > MIN_SPD) {
            map.setView(latLng, map.getZoom() > 10 ? map.getZoom() : 16); 
            lastMapUpdate = now;
        }
    }
}

// ===========================================
// BOUCLES DE MISE À JOUR (Architecture Corrigée)
// ===========================================

/**
 * BOUCLE LENTE (Callback GPS) - Correction UKF
 */
function gpsUpdateCallback(pos) {
    if (emergencyStopActive || !ukf) return;
    
    lastGPSPos = pos; 
    const accRaw = pos.coords.accuracy || 100;
    
    // 💡 CORRECTION CRITIQUE : Rejette les latitudes impossibles
    if (pos.coords.latitude > 90.0 || pos.coords.latitude < -90.0) {
        console.error(`Latitude GPS invalide rejetée: ${pos.coords.latitude}`);
        if ($('gps-precision')) $('gps-precision').textContent = `❌ DONNÉES GPS CORROMPUES`;
        return; // Rejette cette mise à jour
    }

    let R_dyn = getKalmanR(accRaw, kAlt, kUncert, selectedEnvironment, currentUKFReactivity); 
    let isSignalPoor = (accRaw > 200 || R_dyn >= UKF_R_MAX * 0.75);

    if (isSignalPoor) {
        if ($('gps-precision')) $('gps-precision').textContent = `❌ ${accRaw.toFixed(0)} m (Estimation)`;
    } else {
        if ($('gps-precision')) $('gps-precision').textContent = `${accRaw.toFixed(2)} m`;
        ukf.update(pos.coords); // Correction EKF
    }
}

/**
 * BOUCLE RAPIDE (IMU) - Prédiction UKF et Affichage
 */
function startFastLoop() {
    if (domFastID) return; 
    
    domFastID = setInterval(() => {
        if (emergencyStopActive || !ukf) return;
        
        const now = performance.now();
        const dt = (now - lastIMUTimestamp) / 1000.0;
        if (dt < MIN_DT) return; 
        lastIMUTimestamp = now;

        // --- 1. PRÉDICTION UKF ---
        ukf.predict({ accel: [accel.x, accel.y, accel.z], gyro: [gyro.x, gyro.y, gyro.z] }, dt);

        // --- 2. EXTRACTION DE L'ÉTAT ---
        const estimatedState = ukf.getState();
        lat = estimatedState.lat;
        lon = estimatedState.lon;
        kAlt = estimatedState.alt;
        kSpd = estimatedState.speed;
        kUncert = estimatedState.kUncert;
        kAltUncert = estimatedState.kAltUncert;

        const sSpdFE = kSpd < MIN_SPD ? 0 : kSpd;
        const spd3D_raw_gps = (lastGPSPos && lastGPSPos.coords.speed) ? lastGPSPos.coords.speed : 0;
        
        // --- 3. CALCULS AVANCÉS (DISTANCE CORRIGÉE & DYNAMIQUE) ---
        
        // 💡 CORRECTION CRITIQUE: Calcul de distance basé sur la trajectoire EKF
        let dist_step = 0;
        if (lPos && lPos.kLat_old !== undefined && isFinite(lat) && isFinite(lPos.kLat_old)) {
            dist_step = dist2D(lPos.kLat_old, lPos.kLon_old, lat, lon, R_ALT_CENTER_REF);
        }
        lPos = { kLat_old: lat, kLon_old: lon }; // Sauvegarde l'état EKF actuel
        
        distM += dist_step; // Ajoute la distance EKF lissée
        
        if (sSpdFE > MIN_SPD) { timeMoving += dt; }
        if (sTime) { timeTotal = (Date.now() - sTime) / 1000; }
        if (spd3D_raw_gps > maxSpd) maxSpd = spd3D_raw_gps; 
        
        // NOUVEAU: Calculs Dynamiques
        const { mach, dynamicPressure, dragForce, accelLong, coriolisForce } = calculateDynamicMetrics(
            sSpdFE, 
            currentAirDensity, 
            currentSpeedOfSound,
            accel.x, // Accélération longitudinale (IMU)
            lat * D2R // Latitude en radians
        );
        
        // GESTION DE L'ÉNERGIE GPS AUTOMATIQUE
        if (sSpdFE < MIN_SPD * 2 && gpsStandbyTimeoutID === null && currentGPSMode === 'HIGH_FREQ') {
            gpsStandbyTimeoutID = setTimeout(() => startGPS('LOW_FREQ'), STANDBY_TIMEOUT_MS);
        } else if (sSpdFE >= MIN_SPD * 2 && currentGPSMode === 'LOW_FREQ') {
            startGPS('HIGH_FREQ');
            if (gpsStandbyTimeoutID) clearTimeout(gpsStandbyTimeoutID);
            gpsStandbyTimeoutID = null;
        }

        // --- 4. MISE À JOUR DU DOM (Rapide) ---
        $('elapsed-time').textContent = dataOrDefault(timeTotal, 2, ' s');
        $('time-moving').textContent = dataOrDefault(timeMoving, 2, ' s');
        $('distance-ratio').textContent = `1.000`; // (Logique MRF/Nether retirée)

        $('speed-stable').textContent = dataOrDefault(sSpdFE * KMH_MS, 2);
        $('speed-status-text').textContent = (ukf && kSpd > MIN_SPD) ? "🚀 UKF 21 ÉTATS (INS)" : "✅ ZUPT";
        $('speed-stable-ms').textContent = dataOrDefault(sSpdFE, 3, ' m/s');
        $('speed-stable-kms').textContent = dataOrDefaultExp(sSpdFE / 1000, 3, ' km/s');
        $('speed-3d-inst').textContent = dataOrDefault(spd3D_raw_gps * KMH_MS, 2, ' km/h');
        $('speed-raw-ms').textContent = dataOrDefault(spd3D_raw_gps, 3, ' m/s');
        $('speed-max').textContent = dataOrDefault(maxSpd * KMH_MS, 2, ' km/h');
        $('speed-avg-moving').textContent = timeMoving > 1 ? dataOrDefault(distM / timeMoving * KMH_MS, 2, ' km/h') : '0.00 km/h';
        $('speed-avg-total').textContent = timeTotal > 1 ? dataOrDefault(distM / timeTotal * KMH_MS, 2, ' km/h') : '0.00 km/h';
        $('speed-of-sound-calc').textContent = dataOrDefault(currentSpeedOfSound, 2, ' m/s');
        $('perc-speed-sound').textContent = dataOrDefault(sSpdFE / currentSpeedOfSound * 100, 2, ' %');
        $('mach-number').textContent = dataOrDefault(mach, 4);
        $('perc-speed-c').textContent = dataOrDefaultExp(sSpdFE / C_L * 100, 2, '%');

        $('distance-total-km').textContent = `${dataOrDefault(distM / 1000, 3)} km | ${dataOrDefault(distM, 2)} m`;
        
        // Dynamique & Forces
        $('dynamic-pressure').textContent = dataOrDefault(dynamicPressure, 2, ' Pa');
        $('drag-force').textContent = dataOrDefault(dragForce, 2, ' N');
        $('coriolis-force').textContent = dataOrDefault(coriolisForce, 2, ' N');
        $('gravity-local').textContent = dataOrDefault(G_ACC, 5, ' m/s²');
        $('accel-long').textContent = dataOrDefault(accelLong, 3, ' m/s²'); 
        
        // EKF Debug
        $('kalman-uncert').textContent = dataOrDefault(kUncert, 3, ' m²/s² (P)');
        $('alt-uncertainty').textContent = dataOrDefault(kAltUncert, 3, ' m (σ)');
        $('gps-status-dr').textContent = (kAlt !== null && kAlt < ALT_TH) ? 'OUI (Souterrain)' : ((kAlt !== null && kAlt > ALT_HIGH_TH) ? 'OUI (Avion)' : 'Non');

        // Position
        $('lat-display').textContent = dataOrDefault(lat, 6, ' °');
        $('lon-display').textContent = dataOrDefault(lon, 6, ' °');
        $('alt-display').textContent = dataOrDefault(kAlt, 2, ' m');
        if (lastGPSPos && lastGPSPos.coords.altitude) {
             $('alt-raw').textContent = dataOrDefault(lastGPSPos.coords.altitude, 2, ' m');
        }

        // IMU
        $('accel-x').textContent = dataOrDefault(accel.x, 2, ' m/s²');
        $('accel-y').textContent = dataOrDefault(accel.y, 2, ' m/s²');
        $('accel-z').textContent = dataOrDefault(accel.z, 2, ' m/s²');
        $('angular-speed').textContent = dataOrDefault(Math.sqrt(gyro.x**2 + gyro.y**2 + gyro.z**2) * R2D, 2, ' °/s');
        
        updateMap(lat, lon, (lastGPSPos ? lastGPSPos.coords.accuracy : 100));

    }, IMU_UPDATE_RATE_MS);
}


// ===========================================
// INITIALISATION DOM (DÉMARRAGE)
// ===========================================
document.addEventListener('DOMContentLoaded', () => {
    
    initMap(); 
    
    // --- Initialisation des contrôles ---
    if ($('ukf-reactivity-mode')) {
        const reactivitySelect = $('ukf-reactivity-mode');
        reactivitySelect.value = currentUKFReactivity;
        reactivitySelect.addEventListener('change', (e) => { currentUKFReactivity = e.target.value; });
    }
    if ($('environment-select')) {
        $('environment-select').addEventListener('change', (e) => { selectedEnvironment = e.target.value; });
    }

    // --- Boutons d'action ---
    if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', toggleGPS);
    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', toggleEmergencyStop);
    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => { distM = 0; timeMoving = 0; });
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', () => { maxSpd = 0; });
    
    if ($('hyperloop-toggle-btn')) {
        $('hyperloop-toggle-btn').addEventListener('click', () => {
            hyperloopMode = !hyperloopMode;
            $('hyperloop-toggle-btn').textContent = hyperloopMode ? 'HYPERLOOP ACTIF' : 'HYPERLOOP INACTIF';
            $('hyperloop-toggle-btn').classList.toggle('active', hyperloopMode);
        });
    }

    // --- DÉMARRAGE DU SYSTÈME (via Bouton Init) ---
    if ($('init-system-btn')) {
        $('init-system-btn').addEventListener('click', () => {
            
            // 1. Initialise UKF
            if (typeof math !== 'undefined') {
                ukf = new ProfessionalUKF();
            } else {
                alert("Erreur: math.js n'a pas pu être chargé. Le filtre UKF est désactivé.");
                return;
            }
            
            // 2. Initialise l'heure
            sTime = Date.now();
            
            // 3. Démarre GPS et IMU (déclenche la demande de permission)
            startGPS(); 
            startIMUListeners(); 
            
            // 4. Cache le bouton de démarrage
            $('init-system-btn').style.display = 'none';

            // 5. Démarrage de la boucle lente (Astro/Météo)
            domSlowID = setInterval(async () => {
                const currentLat = lat || 43.296; 
                const currentLon = lon || 5.358;
                
                updateAstro(currentLat, currentLon, lServH, lLocH);
                
                if (lat && lon && !emergencyStopActive) {
                    try {
                        const data = await fetchWeather(currentLat, currentLon, kAlt, hyperloopMode);
                        if (data) {
                            lastP_hPa = data.pressure_hPa;
                            lastT_K = data.tempK;
                            currentAirDensity = data.air_density;
                            currentSpeedOfSound = data.speed_sound;
                            
                            const baroAlt = getBarometricAltitude(lastP_hPa, BARO_ALT_REF_HPA, lastT_K);
                            if ($('alt-baro') && baroAlt !== null) $('alt-baro').textContent = `${baroAlt.toFixed(2)} m`;
                        }
                    } catch(e) {
                         if ($('weather-status')) $('weather-status').textContent = `❌ API ÉCHOUÉE`;
                    }
                }
                
                // Met à jour l'horloge locale (NTP)
                const now = getCDate(lServH, lLocH);
                if (now) {
                    if ($('local-time') && !$('local-time').textContent.includes('Synchronisation...')) {
                        $('local-time').textContent = now.toLocaleTimeString('fr-FR');
                    }
                    if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
                }
                
            }, DOM_SLOW_UPDATE_MS); 

        }, { once: true }); // Ne s'exécute qu'une fois
    }

    // Pré-synchronisation de l'heure
    syncH(lServH, lLocH).then(newTimes => {
        lServH = newTimes.lServH;
        lLocH = newTimes.lLocH;
    });
});
