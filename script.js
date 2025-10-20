// Constantes
const C_LIGHT = 299792458; // Vitesse de la lumière en m/s
const AIR_SPEED_OF_SOUND = 343; // Vitesse du son dans l'air standard (m/s)
const MS_TO_KMH = 3.6;

// Variables de suivi globales
let totalElapsedTime = 0; // Temps total d'action accumulé (en millisecondes)
let sessionStartTime = 0; // Timestamp du début de la session d'action actuelle (Date.now())
let maxSpeed = 0;
let totalDistance = 0;
let lastPosition = null;
let timerInterval = null; // ID pour le setInterval du chronomètre

// ===================================================================
// FONCTIONS UTILITAIRES ET DE CONVERSION
// ===================================================================

function msToKmh(ms) {
    return ms * MS_TO_KMH;
}

/** Calcule la distance euclidienne (3D) entre deux points GPS. */
function calculateDistance3D(pos1, pos2) {
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
    
    return Math.sqrt(distance2D * distance2D + dAlt * dAlt);
}

// ===================================================================
// GESTION DU CHRONOMÈTRE
// ===================================================================

/** Met à jour le chronomètre et retourne le temps total en secondes. */
function updateTime() {
    let totalTimeSec = totalElapsedTime / 1000;

    if (sessionStartTime > 0) {
        // Ajoute le temps de la session en cours
        totalTimeSec += (Date.now() - sessionStartTime) / 1000;
    }
    
    document.getElementById('elapsed-time').textContent = `${totalTimeSec.toFixed(1)} s`;
    
    return totalTimeSec;
}

// ===================================================================
// MISE À JOUR DE L'AFFICHAGE (SUCCESS CALLBACK GPS)
// ===================================================================

function updateDisplay(position) {
    const coords = position.coords;
    
    // 1. Mise à jour du statut
    document.getElementById('gps-status-text').textContent = 'Actif / Fixe';
    document.getElementById('gps-status-text').style.color = 'green';

    // 2. Position et Précision
    document.getElementById('latitude').textContent = coords.latitude !== null ? coords.latitude.toFixed(6) : '--';
    document.getElementById('longitude').textContent = coords.longitude !== null ? coords.longitude.toFixed(6) : '--';
    
    const altitudeM = coords.altitude !== null ? coords.altitude : 0;
    document.getElementById('altitude-m').textContent = `${altitudeM.toFixed(2)} m`;
    document.getElementById('altitude-ft').textContent = `${(altitudeM * 3.28084).toFixed(2)} ft`;
    document.getElementById('accuracy-horiz').textContent = `${coords.accuracy.toFixed(2)} m`;

    // 3. Vitesse (Ground Speed)
    const speedMS = coords.speed !== null ? coords.speed : 0;
    const speedKmh = msToKmh(speedMS);
    
    let verticalSpeed = 0;
    if (lastPosition && lastPosition.coords.altitude !== null && coords.altitude !== null) {
        const timeDiff = (position.timestamp - lastPosition.timestamp) / 1000;
        if (timeDiff > 0.1) {
             verticalSpeed = (coords.altitude - lastPosition.coords.altitude) / timeDiff;
        }
    }
    document.getElementById('speed-vert').textContent = `${verticalSpeed.toFixed(2)} m/s`;

    // Vitesse 3D (Totale) : Combinaison de la vitesse Horizontale et Verticale
    const speed3D_MS = Math.sqrt(speedMS * speedMS + verticalSpeed * verticalSpeed);
    
    document.getElementById('speed-horiz').textContent = `${speedKmh.toFixed(2)} km/h`;
    document.getElementById('speed-3d').textContent = `${msToKmh(speed3D_MS).toFixed(2)} km/h`;
    document.getElementById('speed-ms').textContent = `${speedMS.toFixed(2)} m/s`;
    document.getElementById('speed-mms').textContent = `${(speedMS * 1000).toFixed(0)} mm/s`;

    // 4. Maximum et Distance/Moyenne
    if (speedKmh > maxSpeed) {
        maxSpeed = speedKmh;
    }
    document.getElementById('speed-max').textContent = `${maxSpeed.toFixed(2)} km/h`;

    if (lastPosition) {
        const distanceIncrement = calculateDistance3D(lastPosition, position);
        totalDistance += distanceIncrement;

        // Utilise le temps total ACCUMULÉ pour la vitesse moyenne
        const totalTimeSec = updateTime(); 
        
        const avgSpeedMS = totalTimeSec > 0 ? totalDistance / totalTimeSec : 0;
        document.getElementById('speed-avg').textContent = `${msToKmh(avgSpeedMS).toFixed(2)} km/h`;
        
        document.getElementById('distance-totale-km').textContent = `${(totalDistance / 1000).toFixed(3)} km`;
        document.getElementById('distance-m').textContent = `${totalDistance.toFixed(2)} m`;
    }
    
    // 5. Cosmique
    calculateCosmicData(speed3D_MS);
    
    lastPosition = position;
}

function calculateCosmicData(speedMS) {
    const mach = speedMS / AIR_SPEED_OF_SOUND;
    document.getElementById('mach').textContent = mach.toFixed(3);

    const lightPercent = (speedMS / C_LIGHT) * 100;
    document.getElementById('light-perc').textContent = `${lightPercent.toExponential(8)}%`;
}

// ===================================================================
// GESTIONNAIRES DE CONTRÔLE (Boutons Marche/Arrêt/Réinitialisation)
// ===================================================================

function errorCallback(error) {
    let message = 'Erreur GPS : ';
    switch(error.code) {
        case error.PERMISSION_DENIED:
            message += "L'utilisateur a refusé l'accès. Veuillez autoriser la géolocalisation.";
            break;
        case error.POSITION_UNAVAILABLE:
            message += "Localisation indisponible. Signal faible.";
            break;
        case error.TIMEOUT:
            message += "La demande a expiré. Réessayez.";
            break;
        default:
            message += "Erreur inconnue.";
            break;
    }
    document.getElementById('gps-status-text').textContent = message;
    document.getElementById('gps-status-text').style.color = 'red';
    stopGPS(false); // Arrête le timer sans effacer watchId
}

/** Démarre la surveillance GPS et le timer. (Marche) */
function startGPS() {
    if ('geolocation' in navigator) {
        // Si le GPS est déjà actif, ne rien faire
        if (document.getElementById('watch-id').value) return; 

        const options = { enableHighAccuracy: true, timeout: 5000, maximumAge: 0 };
        
        // 1. Démarrer la surveillance et stocker l'ID
        const watchId = navigator.geolocation.watchPosition(updateDisplay, errorCallback, options);
        document.getElementById('watch-id').value = watchId;

        // 2. Démarrer le chronomètre
        sessionStartTime = Date.now(); // Début de la nouvelle session d'action
        timerInterval = setInterval(updateTime, 100);

        // 3. Mise à jour de l'UI
        document.getElementById('btn-start').disabled = true;
        document.getElementById('btn-stop').disabled = false;
        document.getElementById('btn-reset').disabled = false;
        document.getElementById('gps-status-text').textContent = 'Démarrage...';
        document.getElementById('gps-status-text').style.color = 'orange';

    } else {
        document.getElementById('gps-status-text').textContent = "La géolocalisation n'est pas supportée par ce navigateur.";
        document.getElementById('gps-status-text').style.color = 'red';
    }
}

/** Arrête la surveillance GPS et le timer. (Arrêt) */
function stopGPS(clearWatch = true) {
    const watchId = document.getElementById('watch-id').value;
    
    // 1. Accumuler le temps écoulé (même si clearWatch est false suite à une erreur)
    if (sessionStartTime > 0) {
        totalElapsedTime += (Date.now() - sessionStartTime); // Accumulation du temps
        sessionStartTime = 0; // Met le chronomètre en pause
    }
    clearInterval(timerInterval);

    if (watchId && clearWatch) {
        // 2. Arrêter la surveillance GPS
        navigator.geolocation.clearWatch(watchId);
        document.getElementById('watch-id').value = '';
    }

    // 3. Mise à jour de l'UI
    document.getElementById('btn-start').disabled = false;
    document.getElementById('btn-stop').disabled = true;
    document.getElementById('gps-status-text').textContent = 'Arrêté.';
    document.getElementById('gps-status-text').style.color = 'gray';
}

/** Réinitialise les variables de suivi (Max, Moyenne, Temps). */
function resetMax() {
    // 1. Réinitialiser toutes les variables de suivi
    maxSpeed = 0;
    totalDistance = 0;
    totalElapsedTime = 0;
    lastPosition = null;

    // 2. Redémarrer le temps de session si le GPS est actif (pour le nouveau décompte)
    if (document.getElementById('watch-id').value) {
        sessionStartTime = Date.now(); 
    } else {
        sessionStartTime = 0;
    }

    // 3. Réinitialiser l'affichage
    document.getElementById('speed-max').textContent = '0.00 km/h';
    document.getElementById('speed-avg').textContent = '0.00 km/h';
    document.getElementById('distance-totale-km').textContent = '0.000 km';
    document.getElementById('distance-m').textContent = '0.00 m';
    document.getElementById('elapsed-time').textContent = '0 s';
}

// Initialisation au chargement de la page
document.addEventListener('DOMContentLoaded', () => {
    document.getElementById('gps-status-text').textContent = 'Prêt. Appuyez sur Démarrer.';
    document.getElementById('gps-status-text').style.color = 'gray';
});
