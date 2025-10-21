// ===========================================
// Fichier JavaScript - partie2.js
// Contient: Gestion du DOM, Logique GPS (updateDisplay), Boucle de rafraîchissement rapide (fastDOMUpdate), Initialisation et Événements.
// ===========================================

// --- REFERENCES DOM (Complètes pour cette partie) ---
// Note: Ces références DOM sont aussi utilisées par les fonctions définies dans partie1.js.
const startBtn = document.getElementById('start-btn');
const stopBtn = document.getElementById('stop-btn');
const resetMaxBtn = document.getElementById('reset-max-btn');
const errorDisplay = document.getElementById('error-message');
const speedSourceIndicator = document.getElementById('speed-source-indicator'); 
const setTargetBtn = document.getElementById('set-target-btn');


// ===========================================
// 6. GESTION DE L'AFFICHAGE ET DES ERREURS
// ===========================================

function resetDisplay() {
    // RÉINITIALISE TOUTES LES VALEURS ET L'AFFICHAGE
    // Réinitialisation des variables globales (définies dans partie1.js)
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
    
    // updateAstroDisplay et updateDarkMode sont définies dans partie1.js
    updateAstroDisplay(latitude, longitude); 
    updateDarkMode(latitude, longitude);
    
    if (!lastPosition || startTime === null) return;
    
    // Vitesse Brute (Instantanée 3D)
    const currentSpeedMS_3D = lastPosition.speedMS_3D || 0;
    const currentSpeedKMH_3D = currentSpeedMS_3D * KMH_PER_MS;

    // Vitesse Filtrée (Kalman)
    const stableSpeed = kalmanSpeed < MIN_SPEED_THRESHOLD_MS ? 0 : kalmanSpeed;
    const stableSpeedKMH = stableSpeed * KMH_PER_MS;

    // Affichage PRINCIPAL (Vitesse Brute)
    document.getElementById('speed-3d-inst').textContent = `${currentSpeedKMH_3D.toFixed(5)} km/h`; 
    document.getElementById('speed-ms').textContent = `${currentSpeedMS_3D.toFixed(5)} m/s`; 
    document.getElementById('perc-light').textContent = `${(currentSpeedMS_3D / C_LIGHT * 100).toPrecision(5)}%`;
    document.getElementById('perc-sound').textContent = `${(currentSpeedMS_3D / C_SOUND_SEA_LEVEL * 100).toPrecision(5)}%`;

    // Affichage SECONDAIRE (Vitesse Kalman)
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
            // calculateDistance et calculateBearing sont définies dans partie1.js
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

    // 4. Filtrage de Vitesse (Met à jour la variable globale `kalmanSpeed` - fonction dans partie1.js)
    const filteredSpeedMS = simpleKalmanFilter(speedMS_3D, dt, kalmanR);
    const stableSpeedForError = filteredSpeedMS < MIN_SPEED_THRESHOLD_MS ? 0 : filteredSpeedMS;
    
    // 5. Mise à Jour des Statistiques (Distance et Vitesse Max/Moyenne)
    const elapsedTimeS = (currentTime - startTime) / 1000;
    
    // La distance utilise la VITESSE FILTRÉE
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
    
    // --- Mise à jour des données persistantes (inclut speedMS_3D pour fastDOMUpdate) ---
    const newPosition = { ...position, speedMS_3D: speedMS_3D }; 
    lastPosition = newPosition; 
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
    
    startBtn.disabled = true;
    stopBtn.disabled = false;
    resetMaxBtn.disabled = false;
    errorDisplay.style.display = 'none';
    
    // synchronizeTime est définie dans partie1.js
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


// ===========================================
// 10. ÉCOUTEURS D'ÉVÉNEMENTS (INITIALISATION)
// ===========================================

startBtn.addEventListener('click', startGPS);
stopBtn.addEventListener('click', () => stopGPS(true));
resetMaxBtn.addEventListener('click', resetMaxSpeed);
setTargetBtn.addEventListener('click', setTargetDestination);

// Initialisation au chargement de la page
document.addEventListener('DOMContentLoaded', () => {
    resetDisplay();
    synchronizeTime(); 
    // Démarrer la mise à jour fluide du DOM (pour les temps qui courent)
    domIntervalID = setInterval(fastDOMUpdate, DOM_UPDATE_INTERVAL_MS);
});
