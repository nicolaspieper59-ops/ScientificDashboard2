// ===========================================
// Fichier JavaScript Complet - PARTIE 1/2
// ===========================================

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
let timeOffsetMS = 0; 

// --- ÉTAT DE L'APPLICATION ---
const WATCH_OPTIONS = {
    enableHighAccuracy: true,
    maximumAge: 100,
    timeout: 5000 
};
const DOM_UPDATE_INTERVAL_MS = 17;
let watchID = null;         
let domIntervalID = null;
let lastPosition = null;
let startTime = null;
let totalDistanceM = 0; 
let maxSpeedMS = 0;
let targetLat = null;
let targetLon = null;
let lastDOMTime = null;

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


// --- REFERENCES DOM (Réduites pour cette partie) ---
const setTargetBtn = document.getElementById('set-target-btn');
const modeIndicator = document.getElementById('mode-indicator');


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
// 3. FONCTIONS DE CALCULS GÉO ET ASTRO
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
// 4. FILTRAGE DE KALMAN ADAPTATIF
// ===========================================

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
// 5. GESTION DU MODE JOUR/NUIT & CIBLE
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
    const totalSecondsUT = now.getUTCHours() * 3600 + now.getUTCMinutes() * 60 + now.getUTCSeconds();
    const totalSecondsLSM = (totalSecondsUT + (longitude * 4 * 60) + 86400) % 86400;
    
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


    // --- PHASE LUNAIRE (Illumination) ---
    const D_rad = calculateLunarPhaseAngle();
    const illumination = 0.5 * (1 - Math.cos(D_rad)); 
    document.getElementById('lunar-phase-perc').textContent = `${(illumination * 100).toFixed(1)}%`;

    // --- TEMPS LUNAIRE ---
    calculateLunarTime(longitude); 
                                        }
// ===========================================
// Fichier JavaScript Complet - PARTIE 2/2
// ===========================================

// --- REFERENCES DOM (Complètes pour cette partie) ---
const startBtn = document.getElementById('start-btn');
const stopBtn = document.getElementById('stop-btn');
const resetMaxBtn = document.getElementById('reset-max-btn');
const errorDisplay = document.getElementById('error-message');
const speedSourceIndicator = document.getElementById('speed-source-indicator'); 
const setTargetBtn = document.getElementById('set-target-btn'); // Déjà défini en Partie 1

// ===========================================
// 6. GESTION DE L'AFFICHAGE ET DES ERREURS
// ===========================================

function resetDisplay() {
    // RÉINITIALISE TOUTES LES VALEURS ET L'AFFICHAGE
    lastPosition = null;
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
            // Réinitialisation des temps Astro avec secondes
            else if (['mc-time', 'lunar-time', 'solar-mean', 'solar-true', 'solar-true-header'].includes(id)) element.textContent = '00:00:00';
            else element.textContent = defaultText;
        }
    });

    startBtn.disabled = false;
    stopBtn.disabled = true;
    resetMaxBtn.disabled = true;
    setTargetBtn.textContent = '📍 Aller';
    errorDisplay.style.display = 'none';
    document.body.classList.remove('night-mode');
    document.getElementById('gps-accuracy').classList.remove('max-precision');
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

function resetMaxSpeed() {
    maxSpeedMS = 0;
    document.getElementById('speed-max').textContent = '0.00000 km/h';
}


// ===========================================
// 7. FONCTION DE MISE À JOUR DOM RAPIDE (60 Hz)
// ===========================================

function fastDOMUpdate() {
    
    const latitude = lastPosition ? lastPosition.coords.latitude : 0;
    const longitude = lastPosition ? lastPosition.coords.longitude : 0;
    
    // Mettre à jour les temps astronomiques et mode jour/nuit à chaque tick pour les secondes
    updateAstroDisplay(latitude, longitude); 
    updateDarkMode(latitude, longitude);
    
    if (!lastPosition || startTime === null) return;
    
    // Utiliser kalmanSpeed pour un affichage stable et fluide
    const stableSpeed = kalmanSpeed < MIN_SPEED_THRESHOLD_MS ? 0 : kalmanSpeed;
    const stableSpeedKMH = stableSpeed * KMH_PER_MS;

    // Mise à jour de la vitesse stable dans le DOM à 60Hz
    document.getElementById('speed-stable').textContent = `${stableSpeedKMH.toFixed(5)} km/h`; 
    document.getElementById('speed-stable-mm').textContent = `${(stableSpeed * 1000).toFixed(2)} mm/s`;

    // Mise à jour de la fréquence de rafraîchissement DOM
    const now = performance.now();
    if (lastDOMTime) {
        const freq = 1000 / (now - lastDOMTime);
        document.getElementById('update-frequency').textContent = `${freq.toFixed(1)} Hz (DOM)`;
    }
    lastDOMTime = now;
}


// ===========================================
// 8. FONCTION PRINCIPALE D'AFFICHAGE GPS (LENTE)
// ===========================================

function updateDisplay(position) {
    const latitude = position.coords.latitude;
    const longitude = position.coords.longitude;
    const altitude = position.coords.altitude; 
    const accuracy = position.coords.accuracy;
    const heading = position.coords.heading;   
    const speed = position.coords.speed;       
    const currentTime = position.timestamp;
    
    // 1. Vérification de la Précision
    if (accuracy > MAX_ACCURACY_M) {
        document.getElementById('gps-accuracy').textContent = `🚨 ${accuracy.toFixed(0)} m (Trop Imprécis)`;
        if (lastPosition === null) { lastPosition = position; }
        return; 
    }

    // 2. Calcul de la Vitesse Brute
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

    // 3. Logique Adaptative Kalman (R)
    let kalmanR; 
    let precisionIndicatorText = `${accuracy.toFixed(2)} m`;
    const gpsAccuracyElement = document.getElementById('gps-accuracy');

    // Détermination dynamique du bruit R
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

    // 4. Filtrage de Vitesse (Met à jour la variable globale `kalmanSpeed`)
    const filteredSpeedMS = simpleKalmanFilter(speedMS_3D, dt, kalmanR);
    const stableSpeedForError = filteredSpeedMS < MIN_SPEED_THRESHOLD_MS ? 0 : filteredSpeedMS;
    
    // 5. Mise à Jour des Statistiques (Distance et Vitesse Max/Moyenne)
    const elapsedTimeS = (currentTime - startTime) / 1000;
    
    // UTILISATION DE LA VITESSE FILTRÉE POUR UN CUMUL DE DISTANCE COHÉRENT
    totalDistanceM += stableSpeedForError * dt; 
    
    const speedAvgMS = elapsedTimeS > 0 ? totalDistanceM / elapsedTimeS : 0; 
    
    if (stableSpeedForError > maxSpeedMS) { maxSpeedMS = stableSpeedForError; } 

    // Calcul de l'écart en pourcentage (Erreur Relative)
    let speedErrorPerc = 0;
    if (stableSpeedForError > MIN_SPEED_THRESHOLD_MS) {
        speedErrorPerc = (Math.abs(speedMS_3D - stableSpeedForError) / stableSpeedForError) * 100;
    }

    // 6. MISE À JOUR DU DOM (Lente) - Informations GPS/Statistiques
    
    document.getElementById('elapsed-time').textContent = `${elapsedTimeS.toFixed(2)} s`;
    document.getElementById('speed-3d-inst').textContent = `${(speedMS_3D * KMH_PER_MS).toFixed(5)} km/h`; 
    document.getElementById('speed-ms').textContent = `${speedMS_3D.toFixed(5)} m/s`; 
    document.getElementById('speed-avg').textContent = `${(speedAvgMS * KMH_PER_MS).toFixed(5)} km/h`; 
    document.getElementById('speed-max').textContent = `${(maxSpeedMS * KMH_PER_MS).toFixed(5)} km/h`;
    
    document.getElementById('speed-error-perc').textContent = `${speedErrorPerc.toFixed(2)}%`; 
    
    // --- INFO GPS/POSITION ---
    document.getElementById('latitude').textContent = `${latitude.toFixed(6)}`;
    document.getElementById('longitude').textContent = `${longitude.toFixed(6)}`;
    document.getElementById('altitude').textContent = altitude !== null ? `${altitude.toFixed(3)} m` : 'N/A';
    gpsAccuracyElement.textContent = precisionIndicatorText; 
    
    let undergroundStatus = 'Non';
    if (altitude !== null && altitude < UNDERGROUND_ALT_THRESHOLD_M) {
        undergroundStatus = 'Oui (Très bas)';
    }
    if (accuracy > 100 && watchID !== null) { 
        undergroundStatus = 'Possible (Signal Dégradé)';
    } else if (altitude === null && watchID !== null) {
        undergroundStatus = 'N/A (Signal Perdu)';
    }
    document.getElementById('underground').textContent = undergroundStatus;
    
    document.getElementById('heading').textContent = heading !== null ? `${heading.toFixed(0)}°` : 'N/A';
    document.getElementById('distance-km-m').textContent = `${(totalDistanceM / 1000).toFixed(5)} km | ${totalDistanceM.toFixed(5)} m`;

    speedSourceIndicator.textContent = `Source: ${speedSource}`;

    // 7. Calcul et Affichage du Relèvement
    if (targetLat !== null && targetLon !== null) {
        const bearing = calculateBearing(latitude, longitude, targetLat, targetLon);
        document.getElementById('cap-dest').textContent = `${bearing.toFixed(0)}°`;
    } else { 
        document.getElementById('cap-dest').textContent = 'N/A';
    }
    
    // --- Mise à jour des données persistantes ---
    lastPosition = position; 
} 


// ===========================================
// 9. GESTION DU DÉMARRAGE ET ARRÊT DU GPS
// ===========================================

function startGPS() {
    if (!navigator.geolocation) {
        errorDisplay.style.display = 'block';
        errorDisplay.textContent = "❌ Votre navigateur ne supporte pas la géolocalisation.";
        return;
    }

    resetDisplay(); 
    startTime = Date.now(); 
    
    // Démarre la lecture GPS (lente)
    watchID = navigator.geolocation.watchPosition(updateDisplay, handleGeolocationError, WATCH_OPTIONS);
    
    // Le DOM_UPDATE_INTERVAL_MS est démarré dans DOMContentLoaded pour gérer les temps
    
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
    
    // La boucle fastDOMUpdate n'est pas arrêtée ici car elle gère les temps Astro indépendamment du GPS
    // (sauf si on désactive complètement le DOM_UPDATE_INTERVAL_MS). 
    // Pour l'instant, on la laisse tourner.

    if (shouldReset) {
        resetDisplay();
    } else {
        startBtn.disabled = false;
        stopBtn.disabled = true;
    }
}


// ===========================================
// 10. ÉCOUTEURS D'ÉVÉNEMENTS (INITIALISATION)
// ===========================================

// Écouteurs pour les boutons
const startBtn = document.getElementById('start-btn');
const stopBtn = document.getElementById('stop-btn');
const resetMaxBtn = document.getElementById('reset-max-btn');
const errorDisplay = document.getElementById('error-message');
const speedSourceIndicator = document.getElementById('speed-source-indicator'); 
const setTargetBtn = document.getElementById('set-target-btn');


startBtn.addEventListener('click', startGPS);
stopBtn.addEventListener('click', () => stopGPS(true));
resetMaxBtn.addEventListener('click', resetMaxSpeed);
setTargetBtn.addEventListener('click', setTargetDestination);

// Initialisation au chargement de la page
document.addEventListener('DOMContentLoaded', () => {
    resetDisplay();
    // Synchroniser l'heure pour les calculs astronomiques
    synchronizeTime(); 
    // Démarrer la mise à jour fluide du DOM (pour les temps qui courent)
    domIntervalID = setInterval(fastDOMUpdate, DOM_UPDATE_INTERVAL_MS);
});
