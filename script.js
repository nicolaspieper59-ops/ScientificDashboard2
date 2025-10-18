// --- Variables Globales d'État ---
let maxSpeed = 0;              
let totalDistance = 0;         
let startTime = null;          // Initialisé à null, démarré au premier clic
let lastPosition = null;       
let watchId = null;            // ID du suivi pour pouvoir l'arrêter
let isTracking = false;        // État de suivi

// --- DOM Elements ---
const statusElement = document.getElementById('status');
const startStopBtn = document.getElementById('start-stop-btn');


// --- Fonctions Utilitaires (Identiques au précédent, omises ici pour la concision) ---

/**
 * Convertit les mètres par seconde (m/s) en kilomètres par heure (km/h).
 */
function msToKmh(ms) {
    if (ms === null || isNaN(ms)) return 0;
    return ms * 3.6;
}

/**
 * Formate un nombre de secondes en H:M:S.
 */
function formatTime(totalSeconds) {
    const hours = Math.floor(totalSeconds / 3600);
    const minutes = Math.floor((totalSeconds % 3600) / 60);
    const seconds = Math.floor(totalSeconds % 60);
    const pad = (num) => String(num).padStart(2, '0');
    return `${pad(hours)}:${pad(minutes)}:${pad(seconds)}`;
}

/**
 * Calcule la distance 3D entre deux points (lat, lon, alt).
 */
function calculateDistance(pos1, pos2) {
    const R = 6371000; 
    const lat1 = pos1.latitude * (Math.PI / 180);
    const lat2 = pos2.latitude * (Math.PI / 180);
    const dLat = (pos2.latitude - pos1.latitude) * (Math.PI / 180);
    const dLon = (pos2.longitude - pos1.longitude) * (Math.PI / 180);
    const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
              Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) * Math.sin(dLon / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    const distanceHorizontal = R * c;
    const dAlt = (pos2.altitude !== null && pos1.altitude !== null) ? 
                 (pos2.altitude - pos1.altitude) : 0;
    return Math.sqrt(distanceHorizontal * distanceHorizontal + dAlt * dAlt);
}


// --- Fonction de Mise à Jour (Identique au précédent) ---

function success(position) {
    const coords = position.coords;
    const currentTime = Date.now();

    // 1. Mise à jour de la Distance Totale
    if (lastPosition && isTracking) {
        const segmentDistance = calculateDistance(lastPosition, coords);
        totalDistance += segmentDistance;
    }
    lastPosition = coords;


    // 2. Vitesse Instantanée, Max et Moyenne
    const speedMS = coords.speed !== null ? coords.speed : 0;
    const speedKmh = msToKmh(speedMS);
    
    if (speedKmh > maxSpeed) {
        maxSpeed = speedKmh;
    }

    const totalTimeSeconds = startTime ? (currentTime - startTime) / 1000 : 0;
    let averageSpeedKmh = 0;
    
    if (totalTimeSeconds > 5 && isTracking) {
        averageSpeedKmh = (totalDistance / totalTimeSeconds) * 3.6;
    }


    // 3. Mise à jour de l'affichage
    statusElement.textContent = `Suivi ${isTracking ? 'ACTIF' : 'PAUSÉ'}. Précision H : ${coords.accuracy.toFixed(1)} m`;
    
    document.getElementById('speed-current').textContent = speedKmh.toFixed(2);
    document.getElementById('speed-max').textContent = maxSpeed.toFixed(2);
    document.getElementById('speed-average').textContent = averageSpeedKmh.toFixed(2);
    
    document.getElementById('distance-total').textContent = totalDistance.toFixed(2);
    document.getElementById('altitude').textContent = coords.altitude !== null ? coords.altitude.toFixed(1) : 'N/A';
    document.getElementById('accuracy-h').textContent = coords.accuracy.toFixed(1);

    document.getElementById('time-total').textContent = formatTime(totalTimeSeconds);
}

function error(err) {
    // ... (Logique d'erreur simple) ...
    statusElement.textContent = `Erreur GPS. Code: ${err.code}`;
}


// --- Fonctions de Contrôle ---

/**
 * Démarre ou arrête le suivi GPS.
 */
function toggleTracking() {
    if (isTracking) {
        // --- Arrêter le suivi ---
        if (watchId !== null) {
            navigator.geolocation.clearWatch(watchId);
            watchId = null;
        }
        isTracking = false;
        startStopBtn.textContent = "Reprendre le suivi";
        startStopBtn.classList.remove('running');
        statusElement.style.backgroundColor = '#ffc107'; // Jaune pour pause
        statusElement.textContent = "Suivi en PAUSE. Données figées.";

    } else {
        // --- Démarrer le suivi ---
        if (!navigator.geolocation) {
            statusElement.textContent = "La géolocalisation n'est pas supportée.";
            statusElement.style.backgroundColor = '#dc3545';
            return;
        }

        const options = {
            enableHighAccuracy: true,
            timeout: 300000,
            maximumAge: 0
        };
        
        // S'il n'y a pas de temps de départ (première fois), on initialise
        if (startTime === null) {
             startTime = Date.now();
        }

        // Démarrer la surveillance de la position (watchPosition)
        watchId = navigator.geolocation.watchPosition(success, error, options);
        isTracking = true;
        startStopBtn.textContent = "Arrêter le suivi";
        startStopBtn.classList.add('running');
        statusElement.style.backgroundColor = '#d4edda'; // Vert clair pour actif
        statusElement.textContent = "Suivi ACTIF...";
    }
}

/**
 * Réinitialise toutes les variables d'agrégats.
 */
function resetTracking() {
    // 1. Arrêter le suivi s'il est actif
    if (isTracking) {
        toggleTracking(); // Passe en mode PAUSE
    }

    // 2. Réinitialiser les variables
    maxSpeed = 0;
    totalDistance = 0;
    startTime = null;
    lastPosition = null;

    // 3. Réinitialiser l'affichage
    document.getElementById('speed-current').textContent = '0.00';
    document.getElementById('speed-max').textContent = '0.00';
    document.getElementById('speed-average').textContent = '0.00';
    document.getElementById('distance-total').textContent = '0.00';
    document.getElementById('time-total').textContent = '00:00:00';
    document.getElementById('accuracy-h').textContent = 'N/A';
    document.getElementById('altitude').textContent = 'N/A';
    
    statusElement.style.backgroundColor = '#ffe0b2'; // Orange pour prêt
    statusElement.textContent = "Réinitialisé. Prêt à démarrer le suivi.";
}

// Initialise l'état au chargement de la page
window.onload = () => {
    statusElement.style.backgroundColor = '#ffe0b2';
    // Le suivi ne démarre pas automatiquement, il faut cliquer sur le bouton.
};
