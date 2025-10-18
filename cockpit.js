// Constantes
const C_LIGHT = 299792458; // Vitesse de la lumière en m/s
const AIR_SPEED_OF_SOUND = 343; // Vitesse du son dans l'air standard (m/s)
const MS_TO_KMH = 3.6;

// Variables de suivi
let startTime = Date.now();
let maxSpeed = 0;
let totalDistance = 0;
let lastPosition = null;

// ===================================================================
// FONCTIONS UTILITAIRES DE CONVERSION
// ===================================================================

/** Convertit les mètres/seconde en kilomètres/heure. */
function msToKmh(ms) {
    return ms * MS_TO_KMH;
}

/** Calcule la distance euclidienne (3D) entre deux points GPS.
 * C'est une simplification, une formule de grande-cercle (Haversine) serait plus précise
 * pour les longues distances, mais la géolocalisation est pour le 'Ground Speed' local. */
function calculateDistance3D(pos1, pos2) {
    const R = 6371000; // Rayon de la Terre en mètres
    const lat1 = pos1.coords.latitude * (Math.PI / 180);
    const lat2 = pos2.coords.latitude * (Math.PI / 180);
    const dLat = (pos2.coords.latitude - pos1.coords.latitude) * (Math.PI / 180);
    const dLon = (pos2.coords.longitude - pos1.coords.longitude) * (Math.PI / 180);

    const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
              Math.cos(lat1) * Math.cos(lat2) *
              Math.sin(dLon / 2) * Math.sin(dLon / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    const distance2D = R * c; // Distance horizontale sur la surface terrestre

    const dAlt = pos2.coords.altitude - pos1.coords.altitude;
    
    // Distance totale 3D (Pythagore dans un triangle formé par distance2D et dAlt)
    return Math.sqrt(distance2D * distance2D + dAlt * dAlt);
}

// ===================================================================
// LOGIQUE DE MISE À JOUR DU GPS
// ===================================================================

function updateDisplay(position) {
    const coords = position.coords;

    // --- 1. Statut et Position ---
    document.getElementById('gps-status').textContent = 'Actif / Fixe';
    document.getElementById('gps-status').style.color = 'green';
    document.getElementById('latitude').textContent = coords.latitude.toFixed(6);
    document.getElementById('longitude').textContent = coords.longitude.toFixed(6);
    
    // Si l'altitude n'est pas disponible, use 0
    const altitudeM = coords.altitude !== null ? coords.altitude : 0;
    document.getElementById('altitude-m').textContent = `${altitudeM.toFixed(2)} m`;
    document.getElementById('altitude-ft').textContent = `${(altitudeM * 3.28084).toFixed(2)} ft`;
    document.getElementById('accuracy-horiz').textContent = `${coords.accuracy.toFixed(2)} m`;

    // --- 2. Vitesse (Ground Speed) ---
    // La vitesse horizontale est donnée directement par coords.speed (en m/s)
    const speedMS = coords.speed !== null ? coords.speed : 0;
    
    const speedKmh = msToKmh(speedMS);
    document.getElementById('speed-horiz').textContent = `${speedKmh.toFixed(2)} km/h`;
    document.getElementById('speed-ms').textContent = `${speedMS.toFixed(2)} m/s`;
    document.getElementById('speed-mms').textContent = `${(speedMS * 1000).toFixed(0)} mm/s`;

    // Vitesse verticale
    // La géolocalisation ne fournit pas directement la vitesse verticale (climb rate).
    // On la simule en utilisant l'altitude.
    let verticalSpeed = 0;
    if (lastPosition && lastPosition.coords.altitude !== null && coords.altitude !== null) {
        const timeDiff = (position.timestamp - lastPosition.timestamp) / 1000; // en secondes
        if (timeDiff > 0) {
             verticalSpeed = (coords.altitude - lastPosition.coords.altitude) / timeDiff;
        }
    }
    document.getElementById('speed-vert').textContent = `${verticalSpeed.toFixed(2)} m/s`;

    // Vitesse 3D (Totale) : Pythagore : sqrt(Horiz^2 + Vert^2)
    const speed3D_MS = Math.sqrt(speedMS * speedMS + verticalSpeed * verticalSpeed);
    document.getElementById('speed-3d').textContent = `${msToKmh(speed3D_MS).toFixed(2)} km/h`;

    // --- 3. Maximum et Total ---
    maxSpeed = Math.max(maxSpeed, speedKmh);
    document.getElementById('speed-max').textContent = `${maxSpeed.toFixed(2)} km/h`;

    if (lastPosition) {
        // Ajouter la distance parcourue depuis la dernière mise à jour
        const distanceIncrement = calculateDistance3D(lastPosition, position);
        totalDistance += distanceIncrement;

        // Vitesse moyenne : Distance totale / Temps total
        const elapsedTimeSec = (Date.now() - startTime) / 1000;
        const avgSpeedMS = totalDistance / elapsedTimeSec;
        document.getElementById('speed-avg').textContent = `${msToKmh(avgSpeedMS).toFixed(2)} km/h`;
        
        document.getElementById('distance-totale-km').textContent = `${(totalDistance / 1000).toFixed(3)} km`;
        document.getElementById('distance-m').textContent = `${totalDistance.toFixed(2)} m`;
    }
    
    // Mettre à jour la dernière position pour le calcul suivant
    lastPosition = position;
}

function updateTime() {
    const elapsedTimeSec = (Date.now() - startTime) / 1000;
    document.getElementById('elapsed-time').textContent = `${elapsedTimeSec.toFixed(1)} s`;
}

function calculateCosmicData(speedMS) {
    // Nombre de Mach
    const mach = speedMS / AIR_SPEED_OF_SOUND;
    document.getElementById('mach').textContent = mach.toFixed(3);

    // Pourcentage de la vitesse de la lumière
    const lightPercent = (speedMS / C_LIGHT) * 100;
    document.getElementById('light-perc').textContent = `${lightPercent.toExponential(8)}%`;
}


// ===================================================================
// GESTIONNAIRE D'ÉVÉNEMENTS GPS
// ===================================================================

function successCallback(position) {
    document.getElementById('status-message').style.display = 'none';
    
    updateDisplay(position);
    calculateCosmicData(position.coords.speed || 0);
}

function errorCallback(error) {
    let message = 'Erreur GPS : ';
    switch(error.code) {
        case error.PERMISSION_DENIED:
            message += "L'utilisateur a refusé l'accès à la géolocalisation.";
            break;
        case error.POSITION_UNAVAILABLE:
            message += "Les informations de localisation sont indisponibles.";
            break;
        case error.TIMEOUT:
            message += "La demande de localisation a expiré.";
            break;
        default:
            message += "Erreur inconnue.";
            break;
    }
    document.getElementById('status-message').textContent = message;
    document.getElementById('gps-status').textContent = 'Erreur';
    document.getElementById('gps-status').style.color = 'red';
}

function initGeolocation() {
    if ('geolocation' in navigator) {
        // Options pour une meilleure précision (nécessaire pour la vitesse)
        const options = {
            enableHighAccuracy: true, 
            timeout: 5000, 
            maximumAge: 0
        };
        
        // Regarder la position en continu
        navigator.geolocation.watchPosition(successCallback, errorCallback, options);
        
        // Mettre à jour le temps écoulé toutes les 100ms
        setInterval(updateTime, 100);
    } else {
        document.getElementById('status-message').textContent = "La géolocalisation n'est pas supportée par ce navigateur.";
    }
}

// Démarrer l'application
document.addEventListener('DOMContentLoaded', initGeolocation);
