// =================================================================
// FICHIER COMPLET : dashboard.js (Fusion Vitesse & Astro)
// =================================================================

// --- CONSTANTES DE CONVERSION ---
const DEG_TO_RAD = Math.PI / 180;
const RAD_TO_DEG = 180 / Math.PI;

// --- PHYSIQUE / VITESSES ---
const C_LIGHT = 299792458;          
const C_SOUND_SEA_LEVEL = 343;      
const EARTH_RADIUS = 6371000;       
const KMH_PER_MS = 3.6;             

// --- CONSTANTES ASTRO ---
const OBLIQUITY = 23.44 * DEG_TO_RAD; 
const ECCENTRICITY = 0.0167;        
const JD_2000_REF = 2451545.0; 

// --- COORDONNÉES PAR DÉFAUT (Fallback) ---
const DEFAULT_LAT = 48.8566;
const DEFAULT_LON = 2.3522; 

// --- ÉTAT ET OPTIONS ---
const WATCH_OPTIONS = {
    enableHighAccuracy: true,
    maximumAge: 0,
    timeout: 20000 
};
const DOM_UPDATE_INTERVAL_MS = 17;
let watchID = null;         
let domIntervalID = null;
let lastPosition = null;
let latitude = null;
let longitude = null;
let startTime = null;
let totalDistanceM = 0; 
let maxSpeedMS = 0;
let targetLat = null;
let targetLon = null;
let lastDOMTime = null;

// --- SYNCHRONISATION UTC EN CACHE ---
let timeOffsetMS = 0; 
let lastServerTimeMS = null; 
let lastLocalTimeMS = null;  

// --- FILTRAGE GPS & SENSITIVITÉ ---
const MIN_TIME_INTERVAL_S = 1; 
const MAX_ACCURACY_M = 50;     
const MIN_SPEED_THRESHOLD_MS = 0.001;
const UNDERGROUND_ALT_THRESHOLD_M = -50; 

// --- FILTRAGE DE KALMAN ADAPTATIF ---
let kalmanSpeed = 0; 
let kalmanUncertainty = 1000; 
const PROCESS_NOISE_Q = 0.005; 
const KALMAN_R_MIN = 0.005;     
const KALMAN_R_MAX = 5.0;       
const LOW_PRECISION_THRESHOLD_M = 60;   


// --- REFERENCES DOM ---
const startBtn = document.getElementById('start-btn');
const stopBtn = document.getElementById('stop-btn');
const resetMaxBtn = document.getElementById('reset-max-btn');
const errorDisplay = document.getElementById('error-message');
const speedSourceIndicator = document.getElementById('speed-source-indicator'); 
const setTargetBtn = document.getElementById('set-target-btn');
const modeIndicator = document.getElementById('mode-indicator');


// ===========================================
// FONCTIONS UTILITAIRES DE BASE ET GÉO
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


// ===========================================
// TEMPS ET KALMAN
// ===========================================

async function synchronizeTime() {
    const timeZone = Intl.DateTimeFormat().resolvedOptions().timeZone;
    const url = `https://worldtimeapi.org/api/timezone/${timeZone}`;

    try {
        const response = await fetch(url);
        
        if (!response.ok) {
            throw new Error(`Erreur HTTP: ${response.status}`);
        }
        
        const data = await response.json();
        
        lastServerTimeMS = data.unixtime * 1000; 
        lastLocalTimeMS = Date.now(); 
        timeOffsetMS = lastServerTimeMS - lastLocalTimeMS;
        
    } catch (error) {
        if (lastServerTimeMS === null) {
             lastLocalTimeMS = Date.now();
             timeOffsetMS = 0;
        }
    }
}


function getCorrectedDate() {
    if (lastServerTimeMS !== null) {
        const timeElapsed = Date.now() - lastLocalTimeMS;
        const estimatedServerTimeMS = lastServerTimeMS + timeElapsed;
        
        return new Date(estimatedServerTimeMS);
    } 
    
    return new Date(); 
}

function simpleKalmanFilter(newSpeedMS, dt, R_dynamic) {
    if (dt === 0 || dt > 5) { return kalmanSpeed; } 
    
    const R = R_dynamic !== undefined ? R_dynamic : KALMAN_R_MAX; 
    const Q = PROCESS_NOISE_Q * dt; 
    
    let predictedSpeed = kalmanSpeed;
    let predictedUncertainty = kalmanUncertainty + Q; 

    let K = predictedUncertainty / (predictedUncertainty + R); 

    kalmanSpeed = predictedSpeed + K * (newSpeedMS - predictedSpeed);
    kalmanUncertainty = (1 - K) * predictedUncertainty;

    return kalmanSpeed;
}

// ===========================================
// CALCULS ASTRONOMIQUES
// ===========================================

function calculateSolarDetails() {
    const now = getCorrectedDate();

    const J2000_MS = 946728000000;
    const D = (now.getTime() - J2000_MS) / (1000 * 60 * 60 * 24);

    const M = (357.529 + 0.98560028 * D) * DEG_TO_RAD;
    const L = (280.466 + 0.98564736 * D) * DEG_TO_RAD;
    
    const C_e = 2 * ECCENTRICITY * Math.sin(M) + 1.25 * ECCENTRICITY * ECCENTRICITY * Math.sin(2 * M);
    const lambda = L + C_e; 

    const alpha = Math.atan2(Math.cos(OBLIQUITY) * Math.sin(lambda), Math.cos(lambda));
    
    const EoT_rad = L - alpha;
    let EoT_minutes = (EoT_rad * RAD_TO_DEG) * 4;
    
    if (EoT_minutes > 30) EoT_minutes -= 360 * 4;
    if (EoT_minutes < -30) EoT_minutes += 360 * 4;
    
    let solarLongitude = (lambda * RAD_TO_DEG) % 360;
    if (solarLongitude < 0) solarLongitude += 360;

    return {
        eot: EoT_minutes,
        solarLongitude: solarLongitude,
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

function calculateLunarTime(longitude) {
    const now = getCorrectedDate();
    const JD = now.getTime() / 86400000 + 2440587.5; 

    const T = (JD - JD_2000_REF) / 36525.0; 
    let GST = 280.4606 + 360.9856473 * (JD - JD_2000_REF) + 0.000388 * T * T;
    GST = GST % 360;
    if (GST < 0) GST += 360;

    let L_moon = 218.316 + 488204.661 * T; 
    L_moon = L_moon % 360;
    if (L_moon < 0) L_moon += 360;
    
    let HA_moon = GST + longitude - L_moon;
    HA_moon = HA_moon % 360;
    if (HA_moon < 0) HA_moon += 360;

    const LMT_hours = HA_moon / 15.0; 
    
    const LMT_total_seconds = LMT_hours * 3600;
    const lmtHour = Math.floor(LMT_total_seconds / 3600);
    const lmtMinute = Math.floor((LMT_total_seconds % 3600) / 60);
    const lmtSecond = Math.floor(LMT_total_seconds % 60);
    
    document.getElementById('lunar-time').textContent = 
        `${String(lmtHour).padStart(2, '0')}:${String(lmtMinute).padStart(2, '0')}:${String(lmtSecond).padStart(2, '0')}`;
}

// ===========================================
// LOGIQUE D'AFFICHAGE ET D'ÉTAT
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
        modeIndicator.textContent = 'Mode: Nuit 🌙';
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
        setTargetBtn.textContent = '✔️ Cible définie';
    } else {
        alert("Coordonnées invalides. Réinitialisation.");
        targetLat = null;
        targetLon = null;
        document.getElementById('cap-dest').textContent = 'N/A';
        setTargetBtn.textContent = '🗺️ Aller';
    }
}

function updateAstroDisplay(latitude, longitude) {
    const currentLat = latitude !== null ? latitude : DEFAULT_LAT; 
    const currentLon = longitude !== null ? longitude : DEFAULT_LON;
    
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
    const totalSecondsUT = now.getUTCHours() * 3600 + now.getUTCMinutes() * 60 + now.getUTCSeconds();
    const totalSecondsLSM = (totalSecondsUT + (currentLon * 4 * 60) + 86400) % 86400;
    
    const hsmHour = Math.floor(totalSecondsLSM / 3600);
    const hsmMinute = Math.floor((totalSecondsLSM % 3600) / 60);
    const hsmSecond = Math.floor(totalSecondsLSM % 60);
    
    document.getElementById('solar-mean').textContent = 
        `${String(hsmHour).padStart(2, '0')}:${String(hsmMinute).padStart(2, '0')}:${String(hsmSecond).padStart(2, '0')}`;
    
    const solarDetails = calculateSolarDetails();
    const EoT_seconds = solarDetails.eot * 60;
    
    const totalSecondsLSM_Corrected = totalSecondsLSM + EoT_seconds; 
    const totalSecondsHSV = (totalSecondsLSM_Corrected + 86400) % 86400;

    const lsvHour = Math.floor(totalSecondsHSV / 3600); 
    const lsvMinute = Math.floor((totalSecondsHSV % 3600) / 60);
    const lsvSecond = Math.floor(totalSecondsHSV % 60);
    
    const hsvTimeStr = `${String(lsvHour).padStart(2, '0')}:${String(lsvMinute).padStart(2, '0')}:${String(lsvSecond).padStart(2, '0')}`;

    document.getElementById('solar-true').textContent = `${hsvTimeStr} (HSV)`;
    document.getElementById('solar-true-header').textContent = hsvTimeStr;

    document.getElementById('eot').textContent = `${solarDetails.eot.toFixed(2)} min`;
    document.getElementById('solar-longitude-val').textContent = `${solarDetails.solarLongitude.toFixed(2)} °`;


    // --- PHASE LUNAIRE ---
    const D_rad = calculateLunarPhaseAngle();
    const illumination = 0.5 * (1 - Math.cos(D_rad)); 
    document.getElementById('lunar-phase-perc').textContent = `${(illumination * 100).toFixed(1)}%`;

    calculateLunarTime(currentLon); 
}

// ===========================================
// GESTION DU DOM ET DU GPS
// ===========================================

function resetDisplay() {
    lastPosition = null;
    latitude = null; 
    longitude = null;
    totalDistanceM = 0; 
    startTime = null;
    maxSpeedMS = 0;
    targetLat = null; 
    targetLon = null;
    kalmanSpeed = 0;
    kalmanUncertainty = 1000;
    lastDOMTime = null;

    const defaultText = '--';
    const ids = ['elapsed-time', 'speed-3d-inst', 'speed-stable', 'speed-stable-mm', 'speed-avg', 'speed-max', 'speed-ms', 
        'perc-light', 'perc-sound', 'distance-km-m', 'lunar-time',
        'latitude', 'longitude', 'altitude', 'gps-accuracy', 'underground',
        'solar-true', 'solar-mean', 'eot', 'solar-longitude-val', 
        'lunar-phase-perc', 'mc-time', 'air-temp', 'pressure', 'humidity', 'wind-speed', 
        'boiling-point', 'heading', 'bubble-level', 'cap-dest', 'solar-true-header', 
        'mode-indicator', 'speed-source-indicator', 'speed-error-perc', 'update-frequency'
    ];

    ids.forEach(id => {
        const element = document.getElementById(id);
        if (element) {
            if (id === 'speed-source-indicator') element.textContent = 'Source: N/A';
            else if (['air-temp', 'pressure', 'humidity', 'wind-speed', 'boiling-point', 'bubble-level'].includes(id)) {
                element.textContent = 'N/A (API désactivée)'; 
            }
            else if (id === 'altitude' || id === 'gps-accuracy') element.textContent = '-- m';
            else if (id === 'distance-km-m') element.textContent = '-- km | -- m';
            else if (id === 'mode-indicator') element.textContent = 'Mode: Jour ☀️';
            else if (id === 'speed-error-perc' || id === 'update-frequency') element.textContent = '--';
            else if (['mc-time', 'lunar-time', 'solar-mean', 'solar-true', 'solar-true-header'].includes(id)) element.textContent = '00:00:00';
            else element.textContent = defaultText;
        }
    });

    startBtn.disabled = false;
    stopBtn.disabled = true;
    resetMaxBtn.disabled = true;
    setTargetBtn.textContent = '🗺️ Aller';
    errorDisplay.style.display = 'none';
    document.body.classList.remove('night-mode');
    document.getElementById('gps-accuracy').classList.remove('max-precision');
}

function resetMaxSpeed() {
    maxSpeedMS = 0;
    document.getElementById('speed-max').textContent = '0.00000 km/h';
}

function fastDOMUpdate() {
    const latForAstro = latitude !== null ? latitude : DEFAULT_LAT;
    const lonForAstro = longitude !== null ? longitude : DEFAULT_LON;
    
    updateAstroDisplay(latForAstro, lonForAstro); 
    updateDarkMode(latForAstro, lonForAstro);
    
    if (!lastPosition || startTime === null) return;
    
    const currentSpeedMS_3D = lastPosition.speedMS_3D || 0;
    const currentSpeedKMH_3D = currentSpeedMS_3D * KMH_PER_MS;

    const stableSpeed = kalmanSpeed < MIN_SPEED_THRESHOLD_MS ? 0 : kalmanSpeed;
    const stableSpeedKMH = stableSpeed * KMH_PER_MS;

    document.getElementById('speed-3d-inst').textContent = `${currentSpeedKMH_3D.toFixed(5)} km/h`; 
    document.getElementById('speed-ms').textContent = `${currentSpeedMS_3D.toFixed(5)} m/s`; 
    document.getElementById('perc-light').textContent = `${(currentSpeedMS_3D / C_LIGHT * 100).toPrecision(5)}%`;
    document.getElementById('perc-sound').textContent = `${(currentSpeedMS_3D / C_SOUND_SEA_LEVEL * 100).toPrecision(5)}%`;

    document.getElementById('speed-stable').textContent = `${stableSpeedKMH.toFixed(5)} km/h`; 
    document.getElementById('speed-stable-mm').textContent = `${(stableSpeed * 1000).toFixed(2)} mm/s`;

    const now = performance.now();
    if (lastDOMTime) {
        const freq = 1000 / (now - lastDOMTime);
        document.getElementById('update-frequency').textContent = `${freq.toFixed(1)} Hz (DOM)`;
    }
    lastDOMTime = now;
}

function updateDisplay(position) {
    latitude = position.coords.latitude; 
    longitude = position.coords.longitude;
    const altitude = position.coords.altitude; 
    const accuracy = position.coords.accuracy;
    const heading = position.coords.heading;   
    const speed = position.coords.speed;       
    const currentTime = position.timestamp;
    
    if (startTime === null) { startTime = currentTime; }

    if (accuracy > MAX_ACCURACY_M) {
        document.getElementById('gps-accuracy').textContent = `❌ ${accuracy.toFixed(0)} m (Trop Imprécis)`;
        if (lastPosition === null) { lastPosition = position; }
        return; 
    }
    
    let speedMS_Horiz = speed !== null && speed !== undefined ? speed : 0; 
    let speedSource = speed !== null && speed !== undefined ? 'Puce GPS (Doppler)' : 'Calculée (Dérivée)';
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
    
    const speedMS_3D = Math.sqrt(speedMS_Horiz * speedMS_Horiz + speedMS_Vert * speedMS_Vert);
    lastPosition = position;
    lastPosition.speedMS_3D = speedMS_3D;

    let kalmanR; 
    let precisionIndicatorText = `${accuracy.toFixed(2)} m`;
    const gpsAccuracyElement = document.getElementById('gps-accuracy');

    if (accuracy <= 1.0) {
        kalmanR = KALMAN_R_MIN; 
        precisionIndicatorText += ' (Optimal)';
        gpsAccuracyElement.classList.add('max-precision');
        
    } else if (accuracy > LOW_PRECISION_THRESHOLD_M) {
        kalmanR = KALMAN_R_MAX; 
        precisionIndicatorText += ' (Très Faible)';
        gpsAccuracyElement.classList.remove('max-precision');
        
    } else {
        const normalizedAcc = (accuracy - 1.0) / (LOW_PRECISION_THRESHOLD_M - 1.0);
        kalmanR = KALMAN_R_MIN + (KALMAN_R_MAX - KALMAN_R_MIN) * Math.pow(normalizedAcc, 2);
        precisionIndicatorText += ' (Progressif)';
        gpsAccuracyElement.classList.remove('max-precision');
    }
    
    kalmanR = Math.max(KALMAN_R_MIN, Math.min(KALMAN_R_MAX, kalmanR));

    const filteredSpeedMS = simpleKalmanFilter(speedMS_3D, dt, kalmanR);
    const stableSpeedForError = filteredSpeedMS < MIN_SPEED_THRESHOLD_MS ? 0 : filteredSpeedMS;
    
    const elapsedTimeS = (currentTime - startTime) / 1000;
    
    totalDistanceM += stableSpeedForError * dt; 
    
    const speedAvgMS = elapsedTimeS > 0 ? totalDistanceM / elapsedTimeS : 0; 
    
    if (stableSpeedForError > maxSpeedMS) { maxSpeedMS = stableSpeedForError; } 

    let speedErrorPerc = 0;
    if (stableSpeedForError > MIN_SPEED_THRESHOLD_MS) {
        speedErrorPerc = (Math.abs(speedMS_3D - stableSpeedForError) / stableSpeedForError) * 100;
    }

    document.getElementById('elapsed-time').textContent = `${elapsedTimeS.toFixed(2)} s`;
    document.getElementById('speed-avg').textContent = `${(speedAvgMS * KMH_PER_MS).toFixed(5)} km/h`; 
    document.getElementById('speed-max').textContent = `${(maxSpeedMS * KMH_PER_MS).toFixed(5)} km/h`;
    
    document.getElementById('speed-error-perc').textContent = `${speedErrorPerc.toFixed(2)}%`; 
    
    document.getElementById('latitude').textContent = `${latitude.toFixed(6)}`;
    document.getElementById('longitude').textContent = `${longitude.toFixed(6)}`;
    document.getElementById('altitude').textContent = `${altitude !== null ? altitude.toFixed(2) : '--'} m`;
    document.getElementById('gps-accuracy').textContent = precisionIndicatorText;
    document.getElementById('underground').textContent = altitude !== null && altitude < UNDERGROUND_ALT_THRESHOLD_M ? 'Oui' : 'Non';
    document.getElementById('heading').textContent = heading !== null ? `${heading.toFixed(1)} °` : '--';
    
    document.getElementById('distance-km-m').textContent = 
        `${(totalDistanceM / 1000).toFixed(3)} km | ${totalDistanceM.toFixed(2)} m`;

    document.getElementById('speed-source-indicator').textContent = `Source: ${speedSource}`;

    if (targetLat !== null && targetLon !== null) {
        const bearingToTarget = calculateBearing(latitude, longitude, targetLat, targetLon);
        document.getElementById('cap-dest').textContent = `${bearingToTarget.toFixed(1)} °`;
       }
    // =================================================================
// BLOC 6/X : DÉMARRAGE, ARRÊT ET ÉCOUTEURS (Lignes BBB-FIN)
// =================================================================

function handleGeolocationError(error) {
    errorDisplay.style.display = 'block';
    switch (error.code) {
        case error.PERMISSION_DENIED: errorDisplay.textContent = "❌ L'accès à la localisation a été refusé."; break;
        case error.POSITION_UNAVAILABLE: errorDisplay.textContent = "🛰️ Position non disponible. Signal GPS faible."; break;
        case error.TIMEOUT: errorDisplay.textContent = "⏱️ Délai de recherche du GPS dépassé. Signal faible."; break;
        default: errorDisplay.textContent = "❌ Erreur GPS inconnue."; break;
    }
    stopGPS(false); 
}

function startGPS() {
    if (navigator.geolocation) {
        // Initialiser les états
        startTime = Date.now();
        resetMaxSpeed();
        totalDistanceM = 0;
        
        // Tenter la synchronisation UTC avant de démarrer la surveillance GPS
        synchronizeTime(); 

        // Démarrer la surveillance GPS
        watchID = navigator.geolocation.watchPosition(
            updateDisplay, 
            handleGeolocationError, 
            WATCH_OPTIONS
        );
        
        // Démarrer la mise à jour fluide du DOM (si pas déjà fait)
        if (domIntervalID === null) {
            domIntervalID = setInterval(fastDOMUpdate, DOM_UPDATE_INTERVAL_MS);
        }

        // Mettre à jour les boutons d'état
        startBtn.disabled = true;
        stopBtn.disabled = false;
        resetMaxBtn.disabled = false;
        document.getElementById('gps-accuracy').classList.remove('max-precision');
    } else {
        errorDisplay.textContent = "❌ Géolocalisation non supportée par votre navigateur.";
        errorDisplay.style.display = 'block';
    }
}

function stopGPS(clearTime = true) {
    if (watchID !== null) {
        navigator.geolocation.clearWatch(watchID);
        watchID = null;
    }
    if (clearTime) {
        startTime = null;
    }
    
    // Arrêter la mise à jour fluide du DOM (optionnel, peut continuer pour l'astro)
    // if (domIntervalID !== null) {
    //     clearInterval(domIntervalID);
    //     domIntervalID = null;
    // }
    
    // Mettre à jour les boutons d'état
    startBtn.disabled = false;
    stopBtn.disabled = true;
    errorDisplay.style.display = 'block';
    errorDisplay.textContent = "PAUSE : Géolocalisation arrêtée.";
}


document.addEventListener('DOMContentLoaded', () => {
    resetDisplay();
    // Démarrer la mise à jour fluide du DOM pour que l'heure astro/MC/lunaire s'affiche
    // même sans GPS actif.
    domIntervalID = setInterval(fastDOMUpdate, DOM_UPDATE_INTERVAL_MS); 
    
    // Écouteurs pour les boutons de contrôle
    startBtn.addEventListener('click', startGPS);
    stopBtn.addEventListener('click', stopGPS);
    resetMaxBtn.addEventListener('click', resetMaxSpeed);
    setTargetBtn.addEventListener('click', setTargetDestination);
});
