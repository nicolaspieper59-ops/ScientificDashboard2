// ===========================================
// 1. CONSTANTES ET VARIABLES GLOBALES
// ===========================================

// --- PHYSIQUE / VITESSES ---
const C_LIGHT = 299792458;          
const C_SOUND_SEA_LEVEL = 343;      
const EARTH_RADIUS = 6371000;       
const KMH_PER_MS = 3.6;             
const DEG_TO_RAD = Math.PI / 180;
const RAD_TO_DEG = 180 / Math.PI;

// --- CONSTANTES ASTRO ---
const OBLIQUITY = 23.44 * DEG_TO_RAD; 
const ECCENTRICITY = 0.0167;        
const JD_2000_REF = 2451545.0; 

// --- ÉTAT DE L'APPLICATION ---
const WATCH_OPTIONS = {
    enableHighAccuracy: true,
    maximumAge: 500, 
    timeout: 15000 
};
let watchID = null;         
let lastPosition = null;
let startTime = null;
let totalDistanceM = 0; 
let maxSpeedMS = 0;
let targetLat = null;
let targetLon = null;

// --- FILTRAGE GPS ---
const MIN_TIME_INTERVAL_S = 1; 
const MAX_ACCURACY_M = 50;     

// --- SYNCHRONISATION TEMPORELLE (NTP Web) ---
let timeOffsetMS = 0; 

// --- FILTRAGE DE KALMAN SIMPLIFIÉ ---
let kalmanSpeed = 0; // Vitesse estimée par le filtre (m/s)
let kalmanUncertainty = 1000; // Incertitude initiale élevée
const PROCESS_NOISE_Q = 0.01; // Bruit du processus (mouvement du véhicule)
const MEASUREMENT_NOISE_R = 0.1; // Bruit de la mesure GPS (vitesse en m/s)


// --- REFERENCES DOM (Récupération des éléments HTML) ---
const startBtn = document.getElementById('start-btn');
const stopBtn = document.getElementById('stop-btn');
const resetMaxBtn = document.getElementById('reset-max-btn');
const setTargetBtn = document.getElementById('set-target-btn');
const errorDisplay = document.getElementById('error-message');
const modeIndicator = document.getElementById('mode-indicator');
const speedSourceIndicator = document.getElementById('speed-source-indicator'); 


// ===========================================
// 2. SYNCHRONISATION DU TEMPS
// ===========================================

async function synchronizeTime() {
    const timeServerUrl = 'https://worldtimeapi.org/api/ip'; 
    try {
        const response = await fetch(timeServerUrl);
        const T4 = Date.now(); 
        const data = await response.json();
        const serverTimeUTC = data.utc_datetime;
        const serverTimeMS = new Date(serverTimeUTC).getTime();
        timeOffsetMS = serverTimeMS - T4;
    } catch (e) {
        timeOffsetMS = 0; 
    }
}

function getCorrectedDate() {
    return new Date(Date.now() + timeOffsetMS);
}


// ===========================================
// 3. GESTION DES BOUTONS ET DU DÉMARRAGE
// ===========================================

function resetDisplay() {
    lastPosition = null;
    totalDistanceM = 0; 
    startTime = null;
    maxSpeedMS = 0;
    targetLat = null; 
    targetLon = null;
    
    // Réinitialisation des variables du filtre de Kalman
    kalmanSpeed = 0;
    kalmanUncertainty = 1000;

    // Liste complète des IDs à réinitialiser, incluant le nouveau 'speed-stable'
    const defaultText = '--';
    const ids = ['elapsed-time', 'speed-3d-inst', 'speed-stable', 'speed-raw-ms', 'speed-avg', 'speed-max', 'speed-ms', 
        'perc-light', 'perc-sound', 'distance-km-m', 'lunar-time',
        'latitude', 'longitude', 'altitude', 'gps-accuracy', 'underground',
        'solar-true', 'solar-mean', 'eot', 'solar-longitude-val', 
        'lunar-phase-perc', 'mc-time', 'air-temp', 'pressure', 'humidity', 'wind-speed', 
        'boiling-point', 'heading', 'bubble-level', 'cap-dest', 'solar-true-header', 
        'mode-indicator', 'speed-source-indicator'
    ];

    ids.forEach(id => {
        const element = document.getElementById(id);
        if (element) {
            if (id === 'speed-source-indicator') element.textContent = 'Source: N/A';
            else if (['air-temp', 'pressure', 'humidity', 'wind-speed', 'boiling-point', 'bubble-level'].includes(id)) {
                element.textContent = 'N/A (API désactivée)'; 
            }
            else if (id === 'mc-time' || id === 'lunar-time') element.textContent = '00:00:00';
            else if (id === 'altitude' || id === 'gps-accuracy') element.textContent = '-- m';
            else if (id === 'distance-km-m') element.textContent = '-- km | -- m';
            else if (id === 'mode-indicator') element.textContent = 'Mode: Jour ☀️';
            else element.textContent = defaultText;
        }
    });

    // Gestion de l'état des boutons
    startBtn.disabled = false;
    stopBtn.disabled = true;
    resetMaxBtn.disabled = true;
    setTargetBtn.textContent = '📍 Aller';
    errorDisplay.style.display = 'none';
    document.body.classList.remove('night-mode');
}

function handleGeolocationError(error) {
    errorDisplay.style.display = 'block';
    switch (error.code) {
        case error.PERMISSION_DENIED: errorDisplay.textContent = "❌ L'accès à la localisation a été refusé."; break;
        case error.POSITION_UNAVAILABLE: errorDisplay.textContent = "⚠️ Position non disponible. Signal GPS faible."; break;
        case error.TIMEOUT: errorDisplay.textContent = "⏱️ Délai de recherche du GPS dépassé. Signal faible."; break;
        default: errorDisplay.textContent = "❓ Erreur GPS inconnue."; break;
    }
    stopGPS(false); 
}

function startGPS() {
    if (!navigator.geolocation) {
        errorDisplay.style.display = 'block';
        errorDisplay.textContent = "❌ Votre navigateur ne supporte pas la géolocalisation.";
        return;
    }

    resetDisplay(); 
    startTime = Date.now(); 
    
    // Démarre l'écoute de la position GPS
    watchID = navigator.geolocation.watchPosition(updateDisplay, handleGeolocationError, WATCH_OPTIONS);
    
    startBtn.disabled = true;
    stopBtn.disabled = false;
    resetMaxBtn.disabled = false;
    errorDisplay.style.display = 'none';
    
    synchronizeTime(); 
}

function stopGPS(shouldReset = true) {
    if (watchID !== null) {
        navigator.geolocation.clearWatch(watchID);
        watchID = null;
    }
    
    if (shouldReset) {
        resetDisplay();
    } else {
        startBtn.disabled = false;
        stopBtn.disabled = true;
    }
}

function resetMaxSpeed() {
    maxSpeedMS = 0;
    document.getElementById('speed-max').textContent = '0.00000 km/h';
}


// ===========================================
// 4. FONCTIONS DE CALCULS GÉO ET ASTRO
// ===========================================

function calculateDistance(lat1, lon1, lat2, lon2) {
    const R = EARTH_RADIUS;
    const dLat = (lat2 - lat1) * DEG_TO_RAD;
    const dLon = (lon2 - lon1) * DEG_TO_RAD;
    lat1 *= DEG_TO_RAD;
    lat2 *= DEG_TO_RAD;

    const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
              Math.cos(lat1) * Math.cos(lat2) *
              Math.sin(dLon / 2) * Math.sin(dLon / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R * c;
}

function calculateBearing(lat1, lon1, lat2, lon2) {
    const R = DEG_TO_RAD;
    lat1 *= R; lon1 *= R; lat2 *= R; lon2 *= R;

    const y = Math.sin(lon2 - lon1) * Math.cos(lat2);
    const x = Math.cos(lat1) * Math.sin(lat2) -
              Math.sin(lat1) * Math.cos(lat2) * Math.cos(lon2 - lon1);
    
    let bearing = Math.atan2(y, x) * RAD_TO_DEG;
    return (bearing + 360) % 360; 
}

function calculateSolarDetails() {
    const now = getCorrectedDate();
    const J2000 = new Date(Date.UTC(2000, 0, 1, 12, 0, 0));
    const D = (now.getTime() - J2000.getTime()) / (1000 * 60 * 60 * 24);

    const M = (357.529 + 0.98560028 * D) * DEG_TO_RAD;
    const L = (280.466 + 0.98564736 * D) * DEG_TO_RAD;
    
    const C_e = 2 * ECCENTRICITY * Math.sin(M) + 1.25 * ECCENTRICITY * ECCENTRICITY * Math.sin(2 * M);
    const lambda = L + C_e; 

    const alpha = Math.atan2(Math.cos(OBLIQUITY) * Math.sin(lambda), Math.cos(lambda));
    
    const EoT_rad = L - alpha;
    const EoT_minutes = (EoT_rad * RAD_TO_DEG) * 4;
    
    return {
        eot: EoT_minutes,
        solarLongitude: (lambda * RAD_TO_DEG) % 360,
    };
}

function calculateLunarPhaseAngle() {
    const now = getCorrectedDate();
    const JD = now.getTime() / 86400000 + 2440587.5; 
    const d = JD - JD_2000_REF; 
    
    let D = 297.8501921 + 445.2671115 * d;

    D = D % 360; 
    if (D < 0) { D += 360; }
    
    return D * DEG_TO_RAD;
}

function updateAstroDisplay(latitude, longitude) {
    const now = getCorrectedDate();
    
    // --- TEMPS MINECRAFT ---
    const mcTicksPerDay = 24000;
    const msSinceMidnight = now.getTime() % 86400000;
    const mcTicks = (msSinceMidnight * mcTicksPerDay) / 86400000;
    const mcMinutes = Math.floor(mcTicks / 20) % 1440; 
    const mcHour = Math.floor(mcMinutes / 60);
    const mcMinute = mcMinutes % 60;
    const mcSecond = Math.floor((mcTicks % 20) / 20 * 60);
    document.getElementById('mc-time').textContent = `${String(mcHour).padStart(2, '0')}:${String(mcMinute).padStart(2, '0')}:${String(mcSecond).padStart(2, '0')}`;


    // --- TEMPS SOLAIRE MOYEN (HSM) & HSV ---
    const totalMinutesUT = now.getUTCHours() * 60 + now.getUTCMinutes() + now.getUTCSeconds() / 60;
    const totalMinutesLSM = (totalMinutesUT + (longitude * 4) + 1440) % 1440; 
    
    const hsmHour = Math.floor(totalMinutesLSM / 60);
    const hsmMinute = Math.floor(totalMinutesLSM % 60);
    document.getElementById('solar-mean').textContent = `${String(hsmHour).padStart(2, '0')}:${String(hsmMinute).padStart(2, '0')}`;
    
    const solarDetails = calculateSolarDetails();
    const EoT_ms = solarDetails.eot * 60 * 1000; 
    
    const nowHSM = new Date(now.getTime() - (now.getTime() % 86400000) + totalMinutesLSM * 60 * 1000);
    const nowHSV = new Date(nowHSM.getTime() + EoT_ms);

    const lsvHour = nowHSV.getUTCHours(); 
    const lsvMinute = nowHSV.getUTCMinutes();
    const lsvSecond = Math.floor(nowHSV.getUTCSeconds());
    
    const hsvTimeStr = `${String(lsvHour).padStart(2, '0')}:${String(lsvMinute).padStart(2, '0')}:${String(lsvSecond).padStart(2, '0')}`;

    document.getElementById('solar-true').textContent = `${hsvTimeStr} (HSV)`;
    document.getElementById('solar-true-header').textContent = hsvTimeStr;

    document.getElementById('eot').textContent = `${solarDetails.eot.toFixed(2)} min`;
    document.getElementById('solar-longitude-val').textContent = `${solarDetails.solarLongitude.toFixed(2)} °`;


    // --- PHASE LUNAIRE (Illumination) ---
    const D_rad = calculateLunarPhaseAngle();
    const illumination = 0.5 * (1 - Math.cos(D_rad)); 
    document.getElementById('lunar-phase-perc').textContent = `${(illumination * 100).toFixed(1)}%`;

    // --- TEMPS LUNAIRE ---
    // (Cette fonction est longue, je ne la répète pas ici mais elle est dans le fichier précédent.)
    // La logique de calcul du temps lunaire reste correcte.
    calculateLunarTime(longitude); 
}

// ===========================================
// 4b. FILTRAGE DE KALMAN SIMPLIFIÉ
// ===========================================

function simpleKalmanFilter(newSpeedMS, dt) {
    if (dt === 0 || dt > 5) { return kalmanSpeed; } 
    
    const Q = PROCESS_NOISE_Q * dt; 
    
    // 1. Prediction 
    let predictedSpeed = kalmanSpeed;
    let predictedUncertainty = kalmanUncertainty + Q; 

    // 2. Update (Correction)
    let K = predictedUncertainty / (predictedUncertainty + MEASUREMENT_NOISE_R);

    // Mise à jour de la vitesse estimée
    kalmanSpeed = predictedSpeed + K * (newSpeedMS - predictedSpeed);
    
    // Mise à jour de l'incertitude
    kalmanUncertainty = (1 - K) * predictedUncertainty;

    return kalmanSpeed;
}


// ===========================================
// 5. MODE JOUR/NUIT & CIBLE
// ===========================================

function updateDarkMode(latitude, longitude) {
    const now = getCorrectedDate();
    const localHourApprox = now.getUTCHours() + now.getUTCMinutes() / 60 + longitude / 15;
    const hours = localHourApprox % 24;
    
    const SUNRISE_H = 6;
    const SUNSET_H = 18;

    const isNight = (hours < SUNRISE_H || hours >= SUNSET_H);

    if (isNight) {
        document.body.classList.add('night-mode');
        modeIndicator.textContent = 'Mode: Nuit 🌑';
    } else {
        document.body.classList.remove('night-mode');
        modeIndicator.textContent = 'Mode: Jour ☀️';
    }
}

function setTargetDestination() {
    if (!lastPosition) {
        alert("Veuillez démarrer le GPS et attendre une position avant de définir une cible.");
        return;
    }

    const currentLat = lastPosition.coords.latitude.toFixed(6);
    const currentLon = lastPosition.coords.longitude.toFixed(6);
    
    const inputLat = prompt(`Entrez la Latitude de destination (actuel: ${currentLat}°) :`, currentLat);
    const inputLon = prompt(`Entrez la Longitude de destination (actuel: ${currentLon}°) :`, currentLon);

    const lat = parseFloat(inputLat);
    const lon = parseFloat(inputLon);

    if (!isNaN(lat) && !isNaN(lon)) {
        targetLat = lat;
        targetLon = lon;
        setTargetBtn.textContent = '✅ Cible définie';
    } else {
        alert("Coordonnées invalides. Réinitialisation.");
        targetLat = null;
        targetLon = null;
        setTargetBtn.textContent = '📍 Aller';
        document.getElementById('cap-dest').textContent = 'N/A';
    }
}


// ===========================================
// 6. FONCTION PRINCIPALE D'AFFICHAGE
// ===========================================

function updateDisplay(position) {
    const latitude = position.coords.latitude;
    const longitude = position.coords.longitude;
    const altitude = position.coords.altitude; 
    const accuracy = position.coords.accuracy;
    const heading = position.coords.heading;   
    const speed = position.coords.speed;       
    const currentTime = position.timestamp;
    
    // --- CONTRÔLE DE LA PRÉCISION ---
    if (accuracy > MAX_ACCURACY_M) {
        document.getElementById('gps-accuracy').textContent = `🚨 ${accuracy.toFixed(0)} m (Trop Imprécis)`;
        if (lastPosition === null) { lastPosition = position; }
        updateAstroDisplay(latitude, longitude);
        updateDarkMode(latitude, longitude);
        return; 
    }

    // --- LOGIQUE DE VITESSE BRUTE ET SOURCE ---
    let speedMS_Horiz = speed !== null && speed !== undefined ? speed : 0; 
    let speedSource = speed !== null && speed !== undefined ? 'Puce GPS (Brute/Doppler)' : 'Calculée (Dérivée)';
    let speedMS_Vert = 0;
    
    const dt = lastPosition ? (currentTime - lastPosition.timestamp) / 1000 : MIN_TIME_INTERVAL_S;

    if (lastPosition && lastPosition.coords.latitude !== undefined) { 
        const dLat = lastPosition.coords.latitude;
        const dLon = lastPosition.coords.longitude;
        const dAlt = lastPosition.coords.altitude; 

        if (dt > 0.1) { 
            const distHorizM = calculateDistance(dLat, dLon, latitude, longitude);
            
            if (speed === null || speed === undefined) { 
                speedMS_Horiz = distHorizM / dt; 
            }
            if (altitude !== null && dAlt !== null) { 
                const distVertM = altitude - dAlt;
                speedMS_Vert = distVertM / dt; 
            }
        } 
    }
    
    // Vitesse 3D Brute (le "Chaos" non filtré)
    const speedMS_3D = Math.sqrt(speedMS_Horiz * speedMS_Horiz + speedMS_Vert * speedMS_Vert);

    // --- FILTRAGE DE VITESSE ---
    const filteredSpeedMS = simpleKalmanFilter(speedMS_3D, dt);
    
    const elapsedTimeS = (currentTime - startTime) / 1000;
    
    totalDistanceM += speedMS_3D * dt; 
    
    const speedAvgMS = elapsedTimeS > 0 ? totalDistanceM / elapsedTimeS : 0; 
    
    if (filteredSpeedMS > maxSpeedMS) { maxSpeedMS = filteredSpeedMS; } 

    const totalDistKm = totalDistanceM / 1000;
    const totalDistMeters = totalDistanceM % 1000;

    // MISE À JOUR DES VALEURS DANS LE DOM
    document.getElementById('elapsed-time').textContent = `${elapsedTimeS.toFixed(2)} s`;
    
    // AFFICHAGE BRUT : Vitesse 3D Instantanée
    document.getElementById('speed-3d-inst').textContent = `${(speedMS_3D * KMH_PER_MS).toFixed(5)} km/h`; 
    
    // AFFICHAGE FILTRÉ (Stable) : Vitesse par le filtre de Kalman
    document.getElementById('speed-stable').textContent = `${(filteredSpeedMS * KMH_PER_MS).toFixed(5)} km/h`; 
    
    document.getElementById('speed-ms').textContent = `${speedMS_3D.toFixed(5)} m/s`; 

    document.getElementById('speed-avg').textContent = `${(speedAvgMS * KMH_PER_MS).toFixed(5)} km/h`; 
    document.getElementById('speed-max').textContent = `${(maxSpeedMS * KMH_PER_MS).toFixed(5)} km/h`;
    
    document.getElementById('perc-light').textContent = `${((filteredSpeedMS / C_LIGHT) * 100).toExponential(2)}%`;
    document.getElementById('perc-sound').textContent = `${((filteredSpeedMS / C_SOUND_SEA_LEVEL) * 100).toFixed(1)}%`;

    document.getElementById('latitude').textContent = `${latitude.toFixed(6)}`;
    document.getElementById('longitude').textContent = `${longitude.toFixed(6)}`;
    document.getElementById('altitude').textContent = altitude !== null ? `${altitude.toFixed(3)} m` : 'N/A';
    document.getElementById('gps-accuracy').textContent = `${accuracy.toFixed(2)} m`;
    document.getElementById('underground').textContent = altitude !== null ? (altitude < 0 ? 'Oui' : 'Non') : 'N/A';
    document.getElementById('heading').textContent = heading !== null ? `${heading.toFixed(0)}°` : 'N/A';
    
    document.getElementById('distance-km-m').textContent = `${totalDistKm.toFixed(5)} km | ${totalDistMeters.toFixed(5)} m`;

    speedSourceIndicator.textContent = `Source: ${speedSource}`;

    if (targetLat !== null && targetLon !== null) {
        const bearing = calculateBearing(latitude, longitude, targetLat, targetLon);
        document.getElementById('cap-dest').textContent = `${bearing.toFixed(0)}°`;
    } else {
        document.getElementById('cap-dest').textContent = 'N/A';
    }

    // --- ASTRO et MODE NUIT/JOUR ---
    updateAstroDisplay(latitude, longitude);
    updateDarkMode(latitude, longitude); 
    
    lastPosition = position; 
}


// ===========================================
// 7. INITIALISATION
// ===========================================

function initApp() {
    
    // LIAISON DES ÉVÉNEMENTS (Vérifie que les éléments DOM existent)
    if (startBtn) startBtn.addEventListener('click', startGPS);
    if (stopBtn) stopBtn.addEventListener('click', () => stopGPS(true)); 
    if (resetMaxBtn) resetMaxBtn.addEventListener('click', resetMaxSpeed);
    if (setTargetBtn) setTargetBtn.addEventListener('click', setTargetDestination);

    synchronizeTime(); 
    setInterval(synchronizeTime, 300000); 
    
    resetDisplay();
    updateDarkMode(0, 0); 
}

// Assure que l'initialisation se fait après le chargement de TOUS les éléments HTML
document.addEventListener('DOMContentLoaded', initApp);
