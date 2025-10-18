// --- CONSTANTES GLOBALES ---
const C_LIGHT = 299792458; // Vitesse de la lumière en m/s
const AIR_SPEED_OF_SOUND = 343; // Vitesse du son dans l'air standard (m/s)
const MS_TO_KMH = 3.6;
const FT_PER_METER = 3.28084;

// --- VARIABLES DE SUIVI GLOBALES ---
let totalElapsedTime = 0; // Temps total d'action accumulé (en millisecondes)
let sessionStartTime = 0; // Timestamp du début de la session d'action actuelle (Date.now())
let maxSpeed = 0;
let totalDistance = 0;
let lastPosition = null; // Dernière position complète (objet Position)
let timerInterval = null; // ID pour le setInterval du chronomètre
let watchId = null; // ID de la surveillance GPS (remplace l'input hidden)

// ===================================================================
// FONCTIONS UTILITAIRES ET DE CONVERSION
// ===================================================================

function msToKmh(ms) {
    return ms * MS_TO_KMH;
}

/** Formatte les secondes totales en HH:MM:SS */
function formatTime(totalSeconds) {
    const h = Math.floor(totalSeconds / 3600);
    const m = Math.floor((totalSeconds % 3600) / 60);
    const s = Math.floor(totalSeconds % 60);
    const ms = Math.floor((totalSeconds * 100) % 100);

    const pad = (num) => String(num).padStart(2, '0');
    return `${pad(h)}:${pad(m)}:${pad(s)}.${pad(ms)}`;
}

/** Calcule la distance euclidienne (3D) entre deux objets Position GPS. */
function calculateDistance3D(pos1, pos2) {
    // Utiliser les coords directement à partir de l'objet Position
    const alt1 = pos1.coords.altitude !== null ? pos1.coords.altitude : 0;
    const alt2 = pos2.coords.altitude !== null ? pos2.coords.altitude : 0;

    const R = 6371000;
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

/** Met à jour le chronomètre et retourne le temps total en secondes (précis). */
function updateTime() {
    let totalTimeMS = totalElapsedTime;

    if (sessionStartTime > 0) {
        // Ajoute le temps de la session en cours
        totalTimeMS += (Date.now() - sessionStartTime);
    }
    
    const totalTimeSec = totalTimeMS / 1000;

    document.getElementById('elapsed-time').textContent = formatTime(totalTimeSec);
    
    return totalTimeSec;
}


// ===================================================================
// MISE À JOUR DE L'AFFICHAGE (SUCCESS CALLBACK GPS)
// ===================================================================

function updateDisplay(position) {
    const coords = position.coords;
    
    // --- 1. Chronomètre et Temps ---
    // Le chronomètre est géré par setInterval, mais nous le mettons à jour ici pour le calcul de la moyenne.
    const totalTimeSec = updateTime(); 

    // --- 2. Vitesse et Distance ---
    const speedMS = coords.speed !== null ? coords.speed : 0;
    const speedKmh = msToKmh(speedMS);
    
    let verticalSpeed = 0;
    let timeDiff = 0;

    if (lastPosition) {
        // Calcul de l'incrément de distance
        const distanceIncrement = calculateDistance3D(lastPosition, position);
        totalDistance += distanceIncrement;
        
        // Calcul de la vitesse verticale (plus précis avec timestamp)
        timeDiff = (position.timestamp - lastPosition.timestamp) / 1000;
        
        if (timeDiff > 0.1 && lastPosition.coords.altitude !== null && coords.altitude !== null) {
             verticalSpeed = (coords.altitude - lastPosition.coords.altitude) / timeDiff;
        }
    }
    
    // Vitesse 3D (Totale)
    const speed3D_MS = Math.sqrt(speedMS * speedMS + verticalSpeed * verticalSpeed);
    
    // Vitesse Moyenne (Distance Totale / Temps Total d'Activité)
    const avgSpeedMS = totalTimeSec > 5 ? totalDistance / totalTimeSec : 0; // Évite les pics au démarrage

    // Maximum
    if (speedKmh > maxSpeed) {
        maxSpeed = speedKmh;
    }

    // --- 3. Mise à jour de l'affichage principal ---
    
    // Vitesse
    document.getElementById('speed-vert').textContent = `${verticalSpeed.toFixed(4)} m/s`;
    document.getElementById('speed-horiz').textContent = `${speedKmh.toFixed(4)} km/h`;
    document.getElementById('speed-3d').textContent = `${msToKmh(speed3D_MS).toFixed(4)} km/h`;
    document.getElementById('speed-ms').textContent = `${speedMS.toFixed(4)} m/s`;
    document.getElementById('speed-mms').textContent = `${(speedMS * 1000).toFixed(4)} mm/s`;
    
    // Agrégats
    document.getElementById('speed-max').textContent = `${maxSpeed.toFixed(4)} km/h`;
    document.getElementById('speed-avg').textContent = `${msToKmh(avgSpeedMS).toFixed(4)} km/h`;
    document.getElementById('distance-totale-km').textContent = `${(totalDistance / 1000).toFixed(3)} km`;
    document.getElementById('distance-m').textContent = `${totalDistance.toFixed(2)} m`;

    // --- 4. Mise à jour de la Position et précision ---
    const altitudeM = coords.altitude !== null ? coords.altitude : 0;
    
    document.getElementById('latitude').textContent = coords.latitude !== null ? coords.latitude.toFixed(6) : '--';
    document.getElementById('longitude').textContent = coords.longitude !== null ? coords.longitude.toFixed(6) : '--';
    document.getElementById('altitude-m').textContent = `${altitudeM.toFixed(2)} m`;
    document.getElementById('altitude-ft').textContent = `${(altitudeM * FT_PER_METER).toFixed(2)} ft`;
    document.getElementById('accuracy-horiz').textContent = `${coords.accuracy.toFixed(2)} m`;

    // --- 5. Données Cosmiques (utiliser speed3D_MS pour Mach/Lumière si possible) ---
    calculateCosmicData(speed3D_MS);
    
    // --- 6. Mises à jour Astro et Météo Simulé ---
    // Note: Ces fonctions devraient être implémentées dans un script externe comme dans votre HTML
    updateAstro(coords.latitude, coords.longitude);
    updateSimulatedMeteo(altitudeM);
    
    // --- 7. Statut et Sauvegarde ---
    document.getElementById('gps-status-text').textContent = 'Actif / Fixe';
    document.getElementById('gps-status-text').style.color = 'green';
    
    lastPosition = position;
}

// ===================================================================
// FONCTIONS DE CALCUL SECONDAIRE
// ===================================================================

function calculateCosmicData(speedMS) {
    const mach = speedMS / AIR_SPEED_OF_SOUND;
    document.getElementById('mach').textContent = mach.toFixed(3);

    const lightPercent = (speedMS / C_LIGHT) * 100;
    document.getElementById('light-perc').textContent = `${lightPercent.toExponential(8)}%`;
}

// Fonction placeholder pour l'Astro (nécessite SunCalc.js)
function updateAstro(lat, lon) {
    if (typeof SunCalc === 'undefined') return;

    const times = SunCalc.getTimes(new Date(), lat, lon);
    const moon = SunCalc.getMoonIllumination(new Date());

    const formatTimeAstro = (date) => date ? date.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit' }) : '--';

    document.getElementById('lever-soleil').textContent = formatTimeAstro(times.sunrise);
    document.getElementById('coucher-soleil').textContent = formatTimeAstro(times.sunset);
    document.getElementById('lune-phase').textContent = (moon.fraction * 100).toFixed(1) + '%';
    // ... autres champs Astro
}

// Fonction pour simuler des données météo basées sur l'altitude (très simplifiée)
function updateSimulatedMeteo(altitudeM) {
    // Simulation du Point d'Ébullition
    const altKm = altitudeM / 1000;
    const boilingPoint = 100 - (altKm * 3.2); // Environ 3.2°C de moins par km
    document.getElementById('boiling-point').textContent = boilingPoint.toFixed(1) + ' °C';
    
    // Les autres champs Météo sont laissés sur '--' car non mesurables.
}


// ===================================================================
// GESTIONNAIRES DE CONTRÔLE (Boutons Marche/Arrêt/Réinitialisation)
// ===================================================================

function errorCallback(error) {
    let message = 'Erreur GPS : Localisation indisponible ou expirée.';
    document.getElementById('gps-status-text').textContent = message;
    document.getElementById('gps-status-text').style.color = 'red';
    
    // Arrête la surveillance si une erreur critique se produit
    if (error.code !== error.TIMEOUT) {
        stopGPS(false);
    }
}

/** Démarre la surveillance GPS et le timer. (Marche) */
function startGPS() {
    if (watchId) return; // Déjà actif

    if ('geolocation' in navigator) {
        const options = { enableHighAccuracy: true, timeout: 30000, maximumAge: 0 };
        
        // 1. Démarrer la surveillance
        watchId = navigator.geolocation.watchPosition(updateDisplay, errorCallback, options);

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
        document.getElementById('gps-status-text').textContent = "La géolocalisation n'est pas supportée.";
        document.getElementById('gps-status-text').style.color = 'red';
    }
}

/** Arrête la surveillance GPS et le timer. (Arrêt) */
function stopGPS(clearGPSWatch = true) {
    // 1. Accumuler le temps écoulé
    if (sessionStartTime > 0) {
        totalElapsedTime += (Date.now() - sessionStartTime); // Accumulation du temps
        sessionStartTime = 0; // Met le chronomètre en pause
    }
    clearInterval(timerInterval);

    if (watchId && clearGPSWatch) {
        // 2. Arrêter la surveillance GPS
        navigator.geolocation.clearWatch(watchId);
        watchId = null;
    }

    // 3. Mise à jour de l'UI
    document.getElementById('btn-start').disabled = false;
    document.getElementById('btn-stop').disabled = true;
    document.getElementById('gps-status-text').textContent = 'Arrêté.';
    document.getElementById('gps-status-text').style.color = 'gray';
}

/** Réinitialise les variables de suivi (Max, Moyenne, Temps). */
function resetMax() {
    // 1. Sauvegarder l'état et arrêter le suivi si besoin
    const wasTracking = !!watchId;
    if (wasTracking) stopGPS();
    
    // 2. Réinitialiser toutes les variables
    maxSpeed = 0;
    totalDistance = 0;
    totalElapsedTime = 0;
    lastPosition = null;

    // 3. Redémarrer la session si le GPS était actif avant reset
    if (wasTracking) {
        startGPS(); 
    } else {
        // Mettre à jour le temps à 0 (pour l'affichage)
        updateTime();
    }

    // 4. Réinitialiser l'affichage
    document.getElementById('speed-max').textContent = '0.00 km/h';
    document.getElementById('speed-avg').textContent = '0.00 km/h';
    document.getElementById('distance-totale-km').textContent = '0.000 km';
    document.getElementById('distance-m').textContent = '0.00 m';
    
    // Statut
    document.getElementById('gps-status-text').textContent = 'Réinitialisé. Prêt.';
    document.getElementById('gps-status-text').style.color = 'gray';
}

// Initialisation au chargement de la page et exposition des fonctions
document.addEventListener('DOMContentLoaded', () => {
    // Initialisation des boutons et du statut
    document.getElementById('btn-start').onclick = startGPS;
    document.getElementById('btn-stop').onclick = stopGPS;
    document.getElementById('btn-reset').onclick = resetMax;

    document.getElementById('btn-stop').disabled = true;
    document.getElementById('btn-reset').disabled = true;

    document.getElementById('gps-status-text').textContent = 'Initialisation...';
    document.getElementById('gps-status-text').style.color = 'gray';
    
    // Remplissage initial des champs non-GPS
    document.getElementById('elapsed-time').textContent = formatTime(0);

    // Tentative d'exécution de la première mise à jour Astro (pour avoir les valeurs au démarrage)
    // Nécessite une position par défaut ou une demande initiale. 
    // Mieux vaut attendre le premier 'startGPS'.
});
