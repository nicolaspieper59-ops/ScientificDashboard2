// Constantes
const C_LIGHT = 299792458; // Vitesse de la lumière en m/s
const AIR_SPEED_OF_SOUND = 343; // Vitesse du son dans l'air standard (m/s)
const MS_TO_KMH = 3.6;

// Variables de suivi globales
let startTime = 0;
let maxSpeed = 0;
let totalDistance = 0;
let lastPosition = null;
let timerInterval = null; // Pour stocker l'ID du timer d'affichage du temps

// ===================================================================
// FONCTIONS UTILITAIRES ET DE CONVERSION
// ===================================================================

/** Convertit les mètres/seconde en kilomètres/heure. */
function msToKmh(ms) {
    return ms * MS_TO_KMH;
}

/** Calcule la distance euclidienne (3D) entre deux points GPS. */
function calculateDistance3D(pos1, pos2) {
    // Si l'une des positions n'a pas d'altitude, on force l'altitude à 0 pour le calcul 3D
    const alt1 = pos1.coords.altitude !== null ? pos1.coords.altitude : 0;
    const alt2 = pos2.coords.altitude !== null ? pos2.coords.altitude : 0;

    const R = 6371000; // Rayon de la Terre en mètres
    const lat1 = pos1.coords.latitude * (Math.PI / 180);
    const lat2 = pos2.coords.latitude * (Math.PI / 180);
    const dLat = (pos2.coords.latitude - pos1.coords.latitude) * (Math.PI / 180);
    const dLon = (pos2.coords.longitude - pos1.coords.longitude) * (Math.PI / 180);

    const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
              Math.cos(lat1) * Math.cos(lat2) *
              Math.sin(dLon / 2) * Math.sin(dLon / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    const distance2D = R * c; 

    const dAlt = alt2 - alt1;
    
    // Distance totale 3D (Pythagore)
    return Math.sqrt(distance2D * distance2D + dAlt * dAlt);
}

// ===================================================================
// MISE À JOUR DE L'AFFICHAGE
// ===================================================================

function updateDisplay(position) {
    const coords = position.coords;
    
    // --- 1. Position et Précision ---
    document.getElementById('gps-status-text').textContent = 'Actif / Fixe';
    document.getElementById('gps-status-text').style.color = 'green';

    document.getElementById('latitude').textContent = coords.latitude !== null ? coords.latitude.toFixed(6) : '--';
    document.getElementById('longitude').textContent = coords.longitude !== null ? coords.longitude.toFixed(6) : '--';
    
    const altitudeM = coords.altitude !== null ? coords.altitude : 0;
    document.getElementById('altitude-m').textContent = `${altitudeM.toFixed(2)} m`;
    document.getElementById('altitude-ft').textContent = `${(altitudeM * 3.28084).toFixed(2)} ft`;
    document.getElementById('accuracy-horiz').textContent = `${coords.accuracy.toFixed(2)} m`;

    // --- 2. Vitesse (Ground Speed) ---
    const speedMS = coords.speed !== null ? coords.speed : 0;
    const speedKmh = msToKmh(speedMS);
    
    // Vitesse verticale (simulée si possible)
    let verticalSpeed = 0;
    if (lastPosition && lastPosition.coords.altitude !== null && coords.altitude !== null) {
        const timeDiff = (position.timestamp - lastPosition.timestamp) / 1000;
        if (timeDiff > 0) {
             verticalSpeed = (coords.altitude - lastPosition.coords.altitude) / timeDiff;
        }
    }
    document.getElementById('speed-vert').textContent = `${verticalSpeed.toFixed(2)} m/s`;

    // Vitesse 3D (Totale)
    const speed3D_MS = Math.sqrt(speedMS * speedMS + verticalSpeed * verticalSpeed);
    
    // Mettre à jour les champs de vitesse
    document.getElementById('speed-horiz').textContent = `${speedKmh.toFixed(2)} km/h`;
    document.getElementById('speed-3d').textContent = `${msToKmh(speed3D_MS).toFixed(2)} km/h`;
    document.getElementById('speed-ms').textContent = `${speedMS.toFixed(2)} m/s`;
    document.getElementById('speed-mms').textContent = `${(speedMS * 1000).toFixed(0)} mm/s`;

    // --- 3. Maximum et Distance ---
    if (speedKmh > maxSpeed) {
        maxSpeed = speedKmh;
    }
    document.getElementById('speed-max').textContent = `${maxSpeed.toFixed(2)} km/h`;

    if (lastPosition) {
        const distanceIncrement = calculateDistance3D(lastPosition, position);
        totalDistance += distanceIncrement;

        // Vitesse moyenne : Distance totale / Temps total
        const elapsedTimeSec = (Date.now() - startTime) / 1000;
        const avgSpeedMS = elapsedTimeSec > 0 ? totalDistance / elapsedTimeSec : 0;
        document.getElementById('speed-avg').textContent = `${msToKmh(avgSpeedMS).toFixed(2)} km/h`;
        
        document.getElementById('distance-totale-km').textContent = `${(totalDistance / 1000).toFixed(3)} km`;
        document.getElementById('distance-m').textContent = `${totalDistance.toFixed(2)} m`;
    }
    
    // --- 4. Cosmique ---
    calculateCosmicData(speed3D_MS); // Utiliser la vitesse 3D pour les indicateurs cosmiques
    
    // Sauvegarder la position actuelle pour le prochain calcul
    lastPosition = position;
}

function updateTime() {
    if (startTime > 0) {
        const elapsedTimeSec = (Date.now() - startTime) / 1000;
        document.getElementById('elapsed-time').textContent = `${elapsedTimeSec.toFixed(1)} s`;
    }
}

function calculateCosmicData(speedMS) {
    const mach = speedMS / AIR_SPEED_OF_SOUND;
    document.getElementById('mach').textContent = mach.toFixed(3);

    const lightPercent = (speedMS / C_LIGHT) * 100;
    // Utiliser la notation scientifique pour les très petites valeurs
    document.getElementById('light-perc').textContent = `${lightPercent.toExponential(8)}%`;
}

// ===================================================================
// GESTIONNAIRES DE CONTRÔLE (Boutons)
// ===================================================================

function successCallback(position) {
    updateDisplay(position);
}

function errorCallback(error) {
    let message = 'Erreur GPS : ';
    switch(error.code) {
        case error.PERMISSION_DENIED:
            message += "L'utilisateur a refusé l'accès.";
            break;
        case error.POSITION_UNAVAILABLE:
            message += "Localisation indisponible.";
            break;
        case error.TIMEOUT:
            message += "La demande a expiré.";
            break;
        default:
            message += "Erreur inconnue.";
            break;
    }
    document.getElementById('gps-status-text').textContent = message;
    document.getElementById('gps-status-text').style.color = 'red';
    stopGPS(false); // Arrêter la surveillance en cas d'erreur
}

/** Démarre la surveillance GPS et le timer. */
function startGPS() {
    if ('geolocation' in navigator) {
        const options = {
            enableHighAccuracy: true, 
            timeout: 5000, 
            maximumAge: 0
        };
        
        // 1. Démarrer la surveillance et stocker l'ID
        const watchId = navigator.geolocation.watchPosition(successCallback, errorCallback, options);
        document.getElementById('watch-id').value = watchId;

        // 2. Démarrer le timer
        startTime = Date.now();
        timerInterval = setInterval(updateTime, 100);

        // 3. Mise à jour de l'UI
        document.getElementById('btn-start').disabled = true;
        document.getElementById('btn-stop').disabled = false;
        document.getElementById('btn-reset').disabled = false;
        document.getElementById('gps-status-text').textContent = 'Démarrage...';
        document.getElementById('gps-status-text').style.color = 'orange';

    } else {
        document.getElementById('gps-status-text').textContent = "La géolocalisation n'est pas supportée.";
        document.getElementById('gps-status-text').style.color = 'red';
    }
}

/** Arrête la surveillance GPS et le timer. */
function stopGPS(clearWatch = true) {
    const watchId = document.getElementById('watch-id').value;
    
    if (watchId && clearWatch) {
        // 1. Arrêter la surveillance
        navigator.geolocation.clearWatch(watchId);
        document.getElementById('watch-id').value = '';
    }

    // 2. Arrêter le timer
    clearInterval(timerInterval);

    // 3. Mise à jour de l'UI
    document.getElementById('btn-start').disabled = false;
    document.getElementById('btn-stop').disabled = true;
    document.getElementById('gps-status-text').textContent = 'Arrêté.';
    document.getElementById('gps-status-text').style.color = 'gray';
}

/** Réinitialise les variables de suivi (Max et Moyenne). */
function resetMax() {
    // Réinitialiser les variables
    maxSpeed = 0;
    totalDistance = 0;
    
    // Réinitialiser l'affichage
    document.getElementById('speed-max').textContent = '0.00 km/h';
    document.getElementById('speed-avg').textContent = '0.00 km/h';
    document.getElementById('distance-totale-km').textContent = '0.000 km';
    document.getElementById('distance-m').textContent = '0.00 m';

    // Réinitialiser le temps et la position de départ pour la moyenne
    if (document.getElementById('watch-id').value) {
        // Si la surveillance est active, redémarrer le timer et effacer la dernière position
        startTime = Date.now();
        lastPosition = null;
        document.getElementById('elapsed-time').textContent = '0 s';
    } else {
        // Si la surveillance est inactive, réinitialiser tout à zéro
        startTime = 0;
        document.getElementById('elapsed-time').textContent = '0 s';
    }
                                                     }
