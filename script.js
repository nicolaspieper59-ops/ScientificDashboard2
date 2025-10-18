// --- Variables Globales pour le Calcul de la Moyenne et de la Distance ---
let maxSpeed = 0;              // Vitesse maximale enregistrée (km/h)
let totalDistance = 0;         // Distance totale accumulée (mètres)
let startTime = Date.now();    // Temps de début de l'activité (millisecondes)
let lastPosition = null;       // Dernière position connue pour le calcul de la distance

// --- Fonctions Utilitaires ---

/**
 * Convertit les mètres par seconde (m/s) en kilomètres par heure (km/h).
 * @param {number} ms Vitesse en m/s.
 * @returns {number} Vitesse en km/h.
 */
function msToKmh(ms) {
    if (ms === null || isNaN(ms)) return 0;
    // 1 m/s = 3.6 km/h
    return ms * 3.6;
}

/**
 * Calcule la distance 3D entre deux points (lat, lon, alt).
 * Utilise la formule Haversine pour la distance horizontale, et inclut le dénivelé.
 * @param {Object} pos1 La première position.
 * @param {Object} pos2 La deuxième position.
 * @returns {number} La distance en mètres.
 */
function calculateDistance(pos1, pos2) {
    // Rayon de la Terre en mètres
    const R = 6371000; 

    // Conversion en radians
    const lat1 = pos1.latitude * (Math.PI / 180);
    const lat2 = pos2.latitude * (Math.PI / 180);
    const dLat = (pos2.latitude - pos1.latitude) * (Math.PI / 180);
    const dLon = (pos2.longitude - pos1.longitude) * (Math.PI / 180);

    // Formule Haversine pour distance horizontale (d_h)
    const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
              Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) * Math.sin(dLon / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    const distanceHorizontal = R * c;

    // Calcul de la distance 3D (d_3D) avec le théorème de Pythagore
    // d_3D = sqrt(d_h^2 + d_v^2)
    const dAlt = pos2.altitude !== null && pos1.altitude !== null ? 
                 pos2.altitude - pos1.altitude : 0;
    
    // On retourne la distance euclidienne (3D) pour la distance parcourue
    return Math.sqrt(distanceHorizontal * distanceHorizontal + dAlt * dAlt);
}


// --- Fonctions de Géolocalisation ---

/**
 * Succès de la géolocalisation.
 * @param {Object} position Objet de position du navigateur.
 */
function success(position) {
    const coords = position.coords;
    
    // 1. Mise à jour de la distance totale
    if (lastPosition) {
        // Calcule la distance 3D entre la dernière position et la nouvelle
        const segmentDistance = calculateDistance(lastPosition, coords);
        totalDistance += segmentDistance;
    }
    // Met à jour la dernière position
    lastPosition = coords;


    // 2. Vitesse Instantanée (Utilisation de la vitesse Doppler fournie par le GPS)
    const speedMS = coords.speed !== null ? coords.speed : 0;
    const speedKmh = msToKmh(speedMS);
    
    // 3. Vitesse Maximale
    if (speedKmh > maxSpeed) {
        maxSpeed = speedKmh;
    }

    // 4. Vitesse Moyenne (CALCUL CORRIGÉ)
    const currentTime = Date.now();
    const totalTimeSeconds = (currentTime - startTime) / 1000; // Temps total écoulé en secondes
    
    let averageSpeedKmh = 0;
    if (totalTimeSeconds > 5) { // Évite les divisions par zéro et les pics initiaux
        // Vitesse Moyenne = (Distance Totale en mètres / Temps Total en secondes) * 3.6
        averageSpeedKmh = (totalDistance / totalTimeSeconds) * 3.6;
    }


    // 5. Mise à jour de l'affichage
    document.getElementById('status').textContent = 'Données GPS reçues.';
    
    document.getElementById('speed-current').textContent = speedKmh.toFixed(2);
    document.getElementById('speed-max').textContent = maxSpeed.toFixed(2);
    document.getElementById('distance-total').textContent = totalDistance.toFixed(2);
    document.getElementById('altitude').textContent = coords.altitude !== null ? coords.altitude.toFixed(1) : 'N/A';
    
    // Affichage corrigé de la vitesse moyenne
    document.getElementById('speed-average').textContent = averageSpeedKmh.toFixed(2);
}

/**
 * Erreur de géolocalisation.
 * @param {Object} err Objet d'erreur.
 */
function error(err) {
    let message;
    switch (err.code) {
        case err.PERMISSION_DENIED:
            message = "Vous avez refusé l'accès à la géolocalisation.";
            break;
        case err.POSITION_UNAVAILABLE:
            message = "Position GPS indisponible.";
            break;
        case err.TIMEOUT:
            message = "La demande de position a expiré.";
            break;
        default:
            message = "Erreur inconnue lors de la géolocalisation.";
            break;
    }
    document.getElementById('status').textContent = `Erreur: ${message}`;
}


// --- Lancement de l'Application ---
if (navigator.geolocation) {
    const options = {
        enableHighAccuracy: true,  // Utilise tous les systèmes GNSS disponibles pour une meilleure précision
        timeout: 300000,           // Laisse 5 minutes pour obtenir la position (très large)
        maximumAge: 0              // Force l'appareil à ne pas utiliser de position mise en cache
    };
    
    // Surveillance de la position (appel success() dès qu'une nouvelle position est disponible)
    navigator.geolocation.watchPosition(success, error, options);
} else {
    document.getElementById('status').textContent = "La géolocalisation n'est pas supportée par ce navigateur.";
                                  }
