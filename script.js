// --- CONSTANTES GLOBALES ---
const C_LIGHT = 299792458;      // Vitesse de la lumière en m/s
const AIR_SPEED_OF_SOUND = 343; // Vitesse du son dans l'air standard (m/s)
const MS_TO_KMH = 3.6;
const FT_PER_METER = 3.28084;
const MINECRAFT_DAY_LENGTH_MS = 1200000;
const ACCURACY_THRESHOLD = 20;  // Seuil de précision GPS (en mètres) pour considérer le signal comme fiable.

// --- VARIABLES DE SUIVI GLOBALES ---
let watchId = null;             // ID de la surveillance GPS
let totalElapsedTime = 0;       // Temps total d'action accumulé (en millisecondes)
let sessionStartTime = 0;       // Timestamp du début de la session d'action actuelle (Date.now())
let maxSpeed = 0;               // Vitesse maximale (km/h)
let totalDistance = 0;          // Distance 3D totale accumulée (mètres)
let lastPosition = null;        // Dernière position complète (objet Position)
let timerInterval = null;       // ID pour le setInterval du chronomètre

// ===================================================================
// FONCTIONS UTILITAIRES ET DE CONVERSION
// ===================================================================

function msToKmh(ms) {
    return ms * MS_TO_KMH;
}

/** Formatte les secondes totales en HH:MM:SS.ms */
function formatTime(totalSeconds) {
    const totalTimeMS = totalSeconds * 1000;
    const h = Math.floor(totalTimeMS / 3600000);
    const m = Math.floor((totalTimeMS % 3600000) / 60000);
    const s = Math.floor((totalTimeMS % 60000) / 1000);
    const ms = Math.floor((totalTimeMS % 1000) / 10); // Affiche les centièmes de seconde

    const pad = (num) => String(num).padStart(2, '0');
    return `${pad(h)}:${pad(m)}:${pad(s)}.${pad(ms)}`;
}

/** Calcule la distance euclidienne (3D) entre deux objets Position GPS. */
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

/** Mise à jour de l'heure Minecraft (indépendante du GPS) */
function updateMinecraftTime() {
    const now = new Date();
    const msSinceMidnight = now.getHours() * 3600000 + now.getMinutes() * 60000 + now.getSeconds() * 1000 + now.getMilliseconds();
    const mcTimeRatio = 24000 / MINECRAFT_DAY_LENGTH_MS; 
    let mcTicks = (msSinceMidnight * mcTimeRatio + 6000) % 24000;

    let mcHours = Math.floor(mcTicks / 1000);
    let mcMinutes = Math.floor((mcTicks % 1000) * 0.06); 
    
    if (mcHours >= 18) {
        mcHours -= 18;
    } else {
        mcHours += 6;
    }
    
    const mcSeconds = Math.floor((mcTicks % 1000 * 0.06 * 60) % 60);

    const pad = (num) => String(num).padStart(2, '0');
    document.getElementById('minecraft-time').textContent = `${pad(mcHours)}:${pad(mcMinutes)}:${pad(mcSeconds)}`;
}
setInterval(updateMinecraftTime, 1000);


// ===================================================================
// MISE À JOUR DE L'AFFICHAGE (SUCCESS CALLBACK GPS)
// ===================================================================

function updateDisplay(position) {
    const coords = position.coords;
    
    // 1. --- LOGIQUE DE COUPE/REPRISE DU SIGNAL (GESTION DE LA PERTE GPS) ---
    // Si la précision est mauvaise, on met en pause l'accumulation de distance/temps
    if (coords.accuracy > ACCURACY_THRESHOLD) {
        stopGPS(false); // Arrête le chronomètre et l'accumulation, mais laisse watchId actif
        document.getElementById('gps-status-text').textContent = `Signal faible/imprécis (${coords.accuracy.toFixed(1)}m). PAUSE.`;
        document.getElementById('gps-status-text').style.color = 'orange';
        return; // Sort avant de calculer le mouvement
    }

    // Si le signal est bon ET que le chronomètre est en pause, on redémarre
    if (sessionStartTime === 0 && watchId !== null) {
        sessionStartTime = Date.now();
        timerInterval = setInterval(updateTime, 100);
        document.getElementById('gps-status-text').textContent = 'Actif / Fixe';
        document.getElementById('gps-status-text').style.color = 'green';
    }


    // 2. --- CALCUL DE LA VITESSE INSTANTANÉE RECONSTITUÉE (IGNORER coords.speed) ---
    let speedMS_Reconstituée_3D = 0; 
    let verticalSpeed = 0;
    
    if (lastPosition) {
        const distanceIncrement = calculateDistance3D(lastPosition, position);
        totalDistance += distanceIncrement;
        
        const timeDiff = (position.timestamp - lastPosition.timestamp) / 1000;
        
        if (timeDiff > 0.1) {
            // Vitesse Instantanée 3D = Distance 3D / Temps
            speedMS_Reconstituée_3D = distanceIncrement / timeDiff; 
            
            // Vitesse verticale
            if (lastPosition.coords.altitude !== null && coords.altitude !== null) {
                 verticalSpeed = (coords.altitude - lastPosition.coords.altitude) / timeDiff;
            }
        }
    }
    
    // Projection de la vitesse horizontale à partir de la 3D
    const speedMS_Horiz_Calc = Math.sqrt(
        Math.max(0, speedMS_Reconstituée_3D * speedMS_Reconstituée_3D - verticalSpeed * verticalSpeed)
    );
    const speedKmh_3D_Reconstituée = msToKmh(speedMS_Reconstituée_3D);
    const speedKmh_Horiz_Calc = msToKmh(speedMS_Horiz_Calc);


    // 3. --- CALCULS DES AGRÉGATS (MOYENNE & MAX) ---
    const totalTimeSec = updateTime(); 
    
    // Vitesse Moyenne (Distance Totale / Temps Total d'Activité)
    const avgSpeedMS = totalTimeSec > 5 ? totalDistance / totalTimeSec : 0; 
    
    // Maximum (basé sur la vitesse horizontale calculée)
    if (speedKmh_Horiz_Calc > maxSpeed) {
        maxSpeed = speedKmh_Horiz_Calc;
    }


    // 4. --- MISE À JOUR DE L'AFFICHAGE ---
    
    // Vitesse et Temps
    document.getElementById('speed-vert').textContent = `${verticalSpeed.toFixed(2)} m/s`;
    document.getElementById('speed-horiz').textContent = `${speedKmh_Horiz_Calc.toFixed(2)} km/h`;
    document.getElementById('speed-3d').textContent = `${speedKmh_3D_Reconstituée.toFixed(2)} km/h`;
    document.getElementById('speed-ms').textContent = `${speedMS_Reconstituée_3D.toFixed(2)} m/s`;
    document.getElementById('speed-mms').textContent = `${(speedMS_Reconstituée_3D * 1000).toFixed(0)} mm/s`;
    document.getElementById('speed-max').textContent = `${maxSpeed.toFixed(2)} km/h`;
    document.getElementById('speed-avg').textContent = `${msToKmh(avgSpeedMS).toFixed(2)} km/h`;

    // Distance et Cosmique
    document.getElementById('distance-totale-km').textContent = `${(totalDistance / 1000).toFixed(3)} km`;
    document.getElementById('distance-m').textContent = `${totalDistance.toFixed(2)} m`;
    calculateCosmicData(speedMS_Reconstituée_3D);

    // Position et Altitude
    const altitudeM = coords.altitude !== null ? coords.altitude : 0;
    document.getElementById('latitude').textContent = coords.latitude !== null ? coords.latitude.toFixed(6) : '--';
    document.getElementById('longitude').textContent = coords.longitude !== null ? coords.longitude.toFixed(6) : '--';
    document.getElementById('altitude-m').textContent = `${altitudeM.toFixed(2)} m`;
    document.getElementById('altitude-ft').textContent = `${(altitudeM * FT_PER_METER).toFixed(2)} ft`;
    document.getElementById('accuracy-horiz').textContent = `${coords.accuracy.toFixed(2)} m`;

    // Astro et Météo Simulé
    updateAstro(coords.latitude, coords.longitude);
    updateSimulatedMeteo(altitudeM);
    
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

function updateAstro(lat, lon) {
    if (typeof SunCalc === 'undefined') return;

    const now = new Date();
    const times = SunCalc.getTimes(now, lat, lon);
    const moon = SunCalc.getMoonIllumination(now);
    const moonTimes = SunCalc.getMoonTimes(now, lat, lon);

    const formatTimeAstro = (date) => date ? date.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit' }) : '--';

    document.getElementById('lever-soleil').textContent = formatTimeAstro(times.sunrise);
    document.getElementById('coucher-soleil').textContent = formatTimeAstro(times.sunset);
    document.getElementById('culmination-soleil').textContent = formatTimeAstro(times.solarNoon); 
    document.getElementById('lever-lune').textContent = formatTimeAstro(moonTimes.rise);
    document.getElementById('coucher-lune').textContent = formatTimeAstro(moonTimes.set);
    document.getElementById('culmination-lune').textContent = formatTimeAstro(SunCalc.getMoonTimes(now, lat, lon, true).transit);
    
    document.getElementById('lune-phase').textContent = (moon.fraction * 100).toFixed(1) + '%';
    document.getElementById('lune-magnitude').textContent = moon.phase.toFixed(2); 

    // Simulation de l'Heure Solaire Vraie/Moyenne et Équation du temps
    const diff_solar_noon = (times.solarNoon.getTime() - new Date(new Date().setHours(12, 0, 0, 0)).getTime()) / 60000;
    document.getElementById('eq-temps').textContent = diff_solar_noon.toFixed(1) + ' min';
    document.getElementById('hsv').textContent = formatTimeAstro(new Date(Date.now() + (diff_solar_noon * 60000)));
    document.getElementById('hsm').textContent = formatTimeAstro(now); 
}

function updateSimulatedMeteo(altitudeM) {
    // Simulation du Point d'Ébullition basé sur l'altitude
    const altKm = altitudeM / 1000;
    const boilingPoint = 100 - (altKm * 3.2); 
    document.getElementById('boiling-point').textContent = boilingPoint.toFixed(1) + ' °C';

    // Remplissage des autres champs Météo par défaut
    document.getElementById('temperature').textContent = '-- °C';
    document.getElementById('pression').textContent = '-- hPa';
    document.getElementById('humidite').textContent = '--%';
    document.getElementById('vent').textContent = '-- km/h';
    document.getElementById('nuages').textContent = '--%';
    document.getElementById('pluie').textContent = '-- mm';
    document.getElementById('neige').textContent = '-- mm';
    document.getElementById('uv-index').textContent = '--';
    document.getElementById('air-quality').textContent = '--';
    document.getElementById('niveau-bulle').textContent = '--°';
}


// ===================================================================
// GESTIONNAIRES DE CONTRÔLE (Boutons Marche/Arrêt/Réinitialisation)
// ===================================================================

function errorCallback(error) {
    let message = 'Erreur GPS : ';
    if (error.code === error.PERMISSION_DENIED) {
        message += "Accès refusé. Veuillez autoriser la géolocalisation.";
    } else {
        message += "Signal perdu ou localisation indisponible. Suivi en pause.";
    }
    
    document.getElementById('gps-status-text').textContent = message;
    document.getElementById('gps-status-text').style.color = 'red';
    
    // Arrête le chronomètre et l'accumulation, mais conserve watchId si c'est une erreur récupérable (pas DENIED)
    if (error.code !== error.PERMISSION_DENIED) {
        stopGPS(false); 
    }
}

function startGPS() {
    if (watchId) return;

    if ('geolocation' in navigator) {
        const options = { enableHighAccuracy: true, timeout: 30000, maximumAge: 0 };
        
        watchId = navigator.geolocation.watchPosition(updateDisplay, errorCallback, options);

        if (sessionStartTime === 0) { // Redémarrer le chronomètre s'il était en pause
            sessionStartTime = Date.now();
            timerInterval = setInterval(updateTime, 100);
        }

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
    // 1. Accumuler le temps écoulé (Mise en PAUSE du chronomètre)
    if (sessionStartTime > 0) {
        totalElapsedTime += (Date.now() - sessionStartTime); // Accumulation du temps
        sessionStartTime = 0; // Met le chronomètre en pause
    }
    clearInterval(timerInterval);

    if (watchId && clearGPSWatch) {
        // 2. Arrêter la surveillance GPS (arrêt complet)
        navigator.geolocation.clearWatch(watchId);
        watchId = null;
    }

    // 3. Mise à jour de l'UI
    document.getElementById('btn-start').disabled = false;
    document.getElementById('btn-stop').disabled = true;
    document.getElementById('gps-status-text').textContent = 'Arrêté.';
    document.getElementById('gps-status-text').style.color = 'gray';
}

function resetMax() {
    const wasTracking = !!watchId;
    if (wasTracking) stopGPS();
    
    // 1. Réinitialiser toutes les variables
    maxSpeed = 0;
    totalDistance = 0;
    totalElapsedTime = 0;
    lastPosition = null;

    // 2. Réinitialiser l'affichage
    document.getElementById('speed-max').textContent = '0.00 km/h';
    document.getElementById('speed-avg').textContent = '0.00 km/h';
    document.getElementById('distance-totale-km').textContent = '0.000 km';
    document.getElementById('distance-m').textContent = '0.00 m';
    document.getElementById('speed-horiz').textContent = '0.00 km/h';
    document.getElementById('speed-vert').textContent = '0.00 m/s';
    document.getElementById('speed-3d').textContent = '0.00 km/h';
    document.getElementById('speed-ms').textContent = '0.00 m/s';
    document.getElementById('speed-mms').textContent = '0 mm/s';
    updateTime(); // Met à jour le temps à 00:00:00.00

    // 3. Redémarrer si nécessaire
    if (wasTracking) {
        startGPS(); 
    }

    document.getElementById('gps-status-text').textContent = 'Réinitialisé. Prêt.';
    document.getElementById('gps-status-text').style.color = 'gray';
}

// Initialisation au chargement de la page
document.addEventListener('DOMContentLoaded', () => {
    // Lie les fonctions aux boutons (sécurité)
    document.getElementById('btn-start').onclick = startGPS;
    document.getElementById('btn-stop').onclick = stopGPS;
    document.getElementById('btn-reset').onclick = resetMax;

    // Initialisation des contrôles
    document.getElementById('btn-stop').disabled = true;
    document.getElementById('btn-reset').disabled = true;
    document.getElementById('gps-status-text').textContent = 'Prêt. Appuyez sur Démarrer (Marche).';
    
    // Remplissage initial des champs non-GPS/Astro
    updateSimulatedMeteo(0);
});
