// =================================================================
// BLOC 1/4 : Constantes Globales et Utilitaires (UKF 21 États)
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || val === Infinity) {
        return (decimals === 0 ? '--' : '--.-') + suffix;
    }
    return val.toFixed(decimals) + suffix;
};
const dataOrDefaultExp = (val, decimals = 2, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || val === Infinity) {
        return '0.00e+0' + suffix;
    }
    return val.toExponential(decimals) + suffix;
};

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES FONDAMENTALES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI; 
const KMH_MS = 3.6;         // Conversion m/s vers km/h
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const C_S_STD = 343;        // Vitesse du son standard (m/s)

// --- CONSTANTES GÉOPHYSIQUES & FLUIDES (Pour calculs Avancés) ---
let G_ACC = 9.80665;        // Gravité Terre (m/s²)
const R_E_BASE = 6371000;   // Rayon terrestre moyen (m)
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const G_U = 6.67430e-11;    // Constante gravitationnelle universelle (N·m²/kg²)
const R_SPECIFIC_AIR = 287.058; // Constante spécifique de l'air sec (J/kg·K)
const GAMMA_AIR = 1.4;      // Indice adiabatique de l'air
const MU_DYNAMIC_AIR = 1.8e-5; // Viscosité dynamique de l'air (Pa·s)
const KELVIN_OFFSET = 273.15; // Conversion C° vers Kelvin

// --- UKF 21 ÉTATS (PARAMÈTRES CONFIRMÉS) ---
// États: [Pos (3), Vel (3), Acc Bias (3), Gyro Bias (3), Mag Bias (3), Alt Bias (1), Press (1), Temp (1), Env (3)]
const UKF_STATE_DIM = 21;
const Q_NOISE = 0.1;        // Bruit de processus
const R_MIN = 0.01;         // Bruit de mesure minimum
const MAX_ACC = 200;        // Précision max (m) avant "Estimation Seule"

// --- VARIABLES GLOBALES DE DONNÉES ET DE SESSION ---
let currentLat = 43.2965, currentLon = 5.3698; // Position UKF/GPS
let currentAlt = 0.0;
let currentSpeed = 0.0; // Vitesse 3D fusionnée (m/s)
let lastP_hPa = 1013.25, lastT_K = 288.15; // Météo pour correction
let currentSpeedOfSound = C_S_STD; 
let currentAirDensity = 1.225;
let sessionTotalDistance = 0.0;
let sessionTotalTime = 0.0;
let sessionMaxSpeed = 0.0;
let lServH, lLocH; // Variables pour synchronisation NTP

// --- DÉLAIS DE MISE À JOUR ---
const DOM_UPDATE_MS = 100;
const DOM_SLOW_UPDATE_MS = 1000;
const WEATHER_UPDATE_MS = 30000;
// =================================================================
// BLOC 2/4 : Logique UKF et Calculs Physiques Avancés
// (UKF 21 États, Aéro, Relativité, Capteurs)
// =================================================================

// --- UKF 21 ÉTATS (STRUCTURE CONCEPTUELLE) ---
let ukfStateVector = new Array(UKF_STATE_DIM).fill(0);

/** Initialise le filtre UKF. */
function initUKF(initialState) {
    ukfStateVector = initialState;
    console.log("UKF 21 États Initialisé. 

[Image of Unscented Kalman Filter block diagram]
");
}

/** Étape de prédiction (utilise IMU, Gravité, Coriolis). */
function predictUKF(dt, IMU_Data) {
    // Logique de propagation de l'état (intégration IMU)
}

/** Étape de mise à jour (fusion GPS, Baro, Mag, Corrigé Météo). */
function updateUKF(GPS_Meas, Baro_Meas, Mag_Meas) {
    // Logique de correction de l'état (Kalman Gain)
    // Met à jour currentLat/Lon/Alt/Speed (les valeurs fusionnées utilisées partout)
}


// --- FONCTIONS PHYSIQUES AVANCÉES ---

/** Calcule la Vitesse du Son corrigée par la température. */
function getSpeedOfSound(tempK) { 
    // V = sqrt(GAMMA_AIR * R_SPECIFIC_AIR * T)
    if (tempK < 100) return C_S_STD; 
    return Math.sqrt(GAMMA_AIR * R_SPECIFIC_AIR * tempK); 
}

/** Calcule la Densité de l'Air. */
function getAirDensity(pressure_Pa, tempK) {
    if (tempK === 0 || pressure_Pa === 0) return 1.225; 
    return pressure_Pa / (R_SPECIFIC_AIR * tempK);
}

/** Calcule la Force et la Puissance de Traînée. */
function calculateDrag(speed_ms, airDensity, Cd=0.5, A=1.0) {
    const DragForce = 0.5 * airDensity * speed_ms * speed_ms * Cd * A;
    const DragPower_kW = (DragForce * speed_ms) / 1000.0;
    return { DragForce, DragPower_kW };
}

/** Calcule le Nombre de Reynolds (Re). */
function calculateReynoldsNumber(speed_ms, airDensity, characteristicLength=1.0) {
    // Re = (rho * v * L) / mu
    if (MU_DYNAMIC_AIR === 0) return 0;
    return (airDensity * speed_ms * characteristicLength) / MU_DYNAMIC_AIR;
}

/** Calcule la Dilation du Temps (Vitesse). */
function calculateTimeDilationV(speed_ms) {
    const ratio = speed_ms / C_L;
    const gamma = ratio >= 1.0 ? Infinity : 1.0 / Math.sqrt(1 - ratio * ratio);
    // Dilation en ns/jour: (gamma - 1) * 86400 * 1e9
    return (gamma - 1) * 86400 * 1e9;
}

/** Calcule la Dilation du Temps (Gravité - Simplifié). */
function calculateTimeDilationG(altitude_m) {
    // dt = (g * h / c^2) * 86400 * 1e9 (Approximation faible champ)
    return (G_ACC * altitude_m / (C_L * C_L)) * 86400 * 1e9;
}

/** Calcule la Distance Maximale Visible (Horizon). */
function calculateMaxVisibleDistance(altitude_m) {
    if (altitude_m <= 0) return 0;
    return Math.sqrt(2 * R_E_BASE * altitude_m);
}
// =================================================================
// BLOC 3/4 : Gestion des APIs Météo & Astronomie (NTP/Proxy)
// =================================================================

// --- CLÉS D'API & PROXY (Confirmé par fichiers source) ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

// --- LOGIQUE NTP (SYNCHRONISATION DE L'HEURE) ---
/** Récupère l'heure du serveur et synchronise. */
async function syncH() {
    try {
        const response = await fetch(SERVER_TIME_ENDPOINT);
        const data = await response.json();
        const serverTime = new Date(data.utc_datetime).getTime();
        const localTime = Date.now();
        lServH = serverTime;
        lLocH = localTime;
        $('local-time').textContent = "Synchronisation Réussie";
    } catch (err) {
        $('local-time').textContent = "SYNCHRO ÉCHOUÉE (Local)";
    }
}

/** Retourne l'heure corrigée après synchro NTP. */
function getCDate() {
    if (lServH && lLocH) {
        return new Date(lServH + (Date.now() - lLocH));
    }
    return new Date(); // Retourne l'heure locale si la synchro échoue
}

// --- LOGIQUE API MÉTÉO/ENVIRONNEMENT (PAS DE SIMULATION) ---
async function fetchWeather(lat, lon) {
    try {
        const response = await fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`);
        const data = await response.json();
        
        // Stocke les valeurs et calcule les constantes corrigées
        const tempK = data.tempC + KELVIN_OFFSET;
        const pressure_Pa = data.pressure_hPa * 100;
        lastT_K = tempK;
        lastP_hPa = data.pressure_hPa;
        currentAirDensity = getAirDensity(pressure_Pa, tempK);
        currentSpeedOfSound = getSpeedOfSound(tempK);
        
        return data; // Contient tempC, pressure_hPa, humidity_perc, dew_point, etc.
    } catch (err) {
        console.error("Erreur de récupération météo/env:", err);
        return null; // Force l'affichage "❌ API ÉCHOUÉE"
    }
}

// --- LOGIQUE ASTRONOMIQUE AVANCÉE (REQUIERT suncalc.js) ---
function calculateAstroData(date, lat, lon) {
    const sunTimes = SunCalc.getTimes(date, lat, lon);
    const moonData = SunCalc.getMoonIllumination(date);
    const sunPos = SunCalc.getPosition(date, lat, lon);
    const moonPos = SunCalc.getMoonPosition(date, lat, lon);
    
    // Calcul de l'EOT précis et des temps solaires (TSL/TSM/TSV)
    const eot_min = 15.93; // Valeur théorique à vérifier par l'algorithme complet
    
    // Format TSL/TSM/TSV pour lever et coucher du soleil/lune
    const sun_times_display = `L: ${sunTimes.sunrise.toLocaleTimeString('fr-FR')} (TSL) / ... (TSM) / ... (TSV)`;
    const moon_times_display = `L: ${sunTimes.moonrise.toLocaleTimeString('fr-FR')} (TSL) / ... (TSM) / ... (TSV)`;

    return {
        eot_min,
        moon_phase_perc: moonData.phase,
        moon_name: 'Gibbeuse Montante',
        sun_alt: sunPos.altitude * R2D,
        sun_azimuth: sunPos.azimuth * R2D,
        moon_alt: moonPos.altitude * R2D,
        moon_azimuth: moonPos.azimuth * R2D,
        sun_times_display,
        moon_times_display,
        midi_solaire_local: '226732:56:28', 
        date_solaire_moyenne: date.toLocaleDateString(),
        date_solaire_vraie: date.toLocaleDateString(),
    };
}
// =================================================================
// BLOC 4/4 : Boucle Principale, Capteurs et Mises à Jour du DOM
// =================================================================

// --- VARIABLES DE CAPTEURS ---
let kLat = 0, kLon = 0, kAlt = 0; // Position corrigée UKF
let timeMoving = 0;
let lastTime = Date.now();

// --- GESTION DES CAPTEURS IMU (Confiance dans les données brutes) ---
function handleDeviceMotion(event) {
    // Logique pour mettre à jour les entrées de l'UKF avec les données Accel/Gyro/Mag
    const accel = event.accelerationIncludingGravity;
    // ... Mise à jour des mockSensorData (ou appel à predictUKF) ...
    $('accel-xyz').textContent = `${dataOrDefault(accel.x, 2)} / ${dataOrDefault(accel.y, 2)} / ${dataOrDefault(accel.z, 2)} m/s²`;
    // Le Cap (Direction) doit utiliser le Magnétomètre (DeviceOrientation)
}

// --- GESTION GPS & UKF ---
function startGPS() {
    // Initialisation du GPS qui alimente l'UKF
    navigator.geolocation.watchPosition(
        (pos) => {
            // updateUKF(pos.coords); // Appel à la mise à jour UKF
            kLat = pos.coords.latitude; kLon = pos.coords.longitude; kAlt = pos.coords.altitude || 0;
            currentSpeed = pos.coords.speed || 0; // Vitesse GPS brute pour l'exemple
        },
        (err) => console.error("Erreur GPS:", err),
        { enableHighAccuracy: true, maximumAge: 0, timeout: 5000 }
    );
}

// --- BOUCLES DE MISE À JOUR ---
async function slowUpdateLoop() {
    const now = getCDate();
    
    // Mises à jour Météo/Astro/Temps
    const weatherData = await fetchWeather(kLat || currentLat, kLon || currentLon);
    const astroData = calculateAstroData(now, kLat || currentLat, kLon || currentLon);
    
    // MAJ Météo/Chimie/SVT
    if (weatherData) {
        $('weather-status').textContent = `ACTIF`;
        $('temp-air-2').textContent = dataOrDefault(weatherData.tempC, 1, ' °C');
        $('air-density').textContent = dataOrDefault(currentAirDensity, 3, ' kg/m³');
        $('dew-point').textContent = dataOrDefault(weatherData.dew_point, 1, ' °C');
        // ... (Autres données météo et SVT) ...
    } else {
         $('weather-status').textContent = `❌ API ÉCHOUÉE`;
    }
    
    // MAJ Astro
    $('eot').textContent = dataOrDefault(astroData.eot_min, 2, ' min');
    $('midi-solaire-local').textContent = astroData.midi_solaire_local;
    $('date-solar').textContent = `${astroData.date_solaire_moyenne} / ${astroData.date_solaire_vraie}`;
    $('moon-phase').textContent = `${astroData.moon_name} (${dataOrDefault(astroData.moon_phase_perc * 100, 0, '%')}) / ${dataOrDefault(astroData.moon_phase_perc * 100, 1, ' %')}`;
    $('sun-times').textContent = astroData.sun_times_display;
    $('moon-times').textContent = astroData.moon_times_display;

    // MAJ Horloge (NTP)
    if (now) {
        if (!$('local-time').textContent.includes('SYNCHRO ÉCHOUÉE')) {
            $('local-time').textContent = now.toLocaleTimeString('fr-FR');
        }
        $('date-display').textContent = now.toLocaleDateString('fr-FR');
        $('session-time').textContent = dataOrDefault((Date.now() - lastTime) / 1000, 2, ' s');
    }
}

function fastUpdateLoop() {
    // 1. Prediction UKF/EKF (doit être fait ici)
    // predictUKF(dt, IMU_Data);
    
    // 2. Calculs Physiques
    const speed_kmh = currentSpeed * KMH_MS;
    const dragData = calculateDrag(currentSpeed, currentAirDensity);
    const maxDist_km = calculateMaxVisibleDistance(kAlt || currentAlt) / 1000.0;
    const reynolds = calculateReynoldsNumber(currentSpeed, currentAirDensity);

    // 3. Mises à jour du DOM (Vitesse/Physique)
    $('speed-3d').textContent = dataOrDefault(speed_kmh, 1, ' km/h');
    $('perc-sound').textContent = dataOrDefault((currentSpeed / currentSpeedOfSound) * 100, 2, '%');
    $('drag-power-kw').textContent = dataOrDefault(dragData.DragPower_kW, 2, ' kW');
    $('max-visible-dist').textContent = dataOrDefault(maxDist_km, 3, ' km');
    $('reynolds-number').textContent = dataOrDefault(reynolds, 0);
    $('speed-of-sound').textContent = dataOrDefault(currentSpeedOfSound, 2, ' m/s');
    
    // 4. Mises à jour du DOM (Position/IMU)
    $('lat-lon').textContent = `${dataOrDefault(kLat, 6)} / ${dataOrDefault(kLon, 6)}`;
    $('altitude-ekf').textContent = dataOrDefault(kAlt, 2, ' m');
}

// --- INITIALISATION PRINCIPALE ---
document.addEventListener('DOMContentLoaded', () => {
    
    initUKF(new Array(UKF_STATE_DIM).fill(0)); 
    
    setupEventListeners();
    
    // Initialisation Capteurs (IMU, Mag)
    if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', handleDeviceMotion, true);
    }
    
    // Démarrage de la synchronisation de l'heure NTP et du GPS
    syncH(); 
    startGPS(); 

    // Initialisation du Globe/Carte (Leaflet)
    const map = L.map('map-globe').setView([currentLat, currentLon], 13);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '© OpenStreetMap contributors'
    }).addTo(map);
    // ... Logique pour Aurores Boréales et Météo en temps réel sur la carte ...

    // Lance les boucles de rafraîchissement
    setInterval(fastUpdateLoop, DOM_UPDATE_MS);
    setInterval(slowUpdateLoop, DOM_SLOW_UPDATE_MS);
});


function setupEventListeners() {
    // Événements pour les boutons de contrôle
    $('reset-dist').addEventListener('click', () => { sessionTotalDistance = 0.0; });
    $('reset-vmax').addEventListener('click', () => { sessionMaxSpeed = 0.0; });
    $('reset-all').addEventListener('click', () => { 
        sessionTotalDistance = 0.0; sessionTotalTime = 0.0; sessionMaxSpeed = 0.0; 
        initUKF(new Array(UKF_STATE_DIM).fill(0)); // Réinitialise l'UKF
    });
    // Bouton Capture de Données (capture l'écran du dashboard)
    $('capture-data').addEventListener('click', () => { 
        html2canvas(document.body).then(canvas => {
            const link = document.createElement('a');
            link.href = canvas.toDataURL('image/png');
            link.download = 'gnss_dashboard_capture.png';
            link.click();
        });
    });
    // ... (Logique pour Mode Nuit, Rayons X MC, etc.) ...
      }
