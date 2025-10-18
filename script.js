// --- CONSTANTES GLOBALES ---
const SPEED_OF_LIGHT = 299792458; // Vitesse de la lumière en m/s
const MACH_1_MS = 343;            // Vitesse du son à 20°C au niveau de la mer (m/s)
const MINECRAFT_DAY_LENGTH_MS = 1200000; // 20 minutes en millisecondes
const FT_PER_METER = 3.28084;

// --- VARIABLES D'ÉTAT ---
let watchId = null;            // ID du suivi GPS
let isTracking = false;
let startTime = null;
let lastPosition = null;       // Dernière position (coords)
let totalDistance = 0;         // Distance 3D totale accumulée (mètres)
let maxSpeed = 0;              // Vitesse maximale (km/h)
let lastAltitude = null;       // Dernière altitude pour le calcul de vitesse verticale
let lastTimestamp = null;      // Dernier timestamp pour le calcul de vitesse verticale

// --- ELEMENTS DU DOM (Correspondance avec le HTML fourni) ---
const elements = {
    statusText: document.getElementById('gps-status-text'),
    btnStart: document.getElementById('btn-start'),
    btnStop: document.getElementById('btn-stop'),
    btnReset: document.getElementById('btn-reset'),
    
    // Vitesse et Temps
    elapsedTime: document.getElementById('elapsed-time'),
    speedHoriz: document.getElementById('speed-horiz'),
    speedVert: document.getElementById('speed-vert'),
    speed3d: document.getElementById('speed-3d'),
    speedAvg: document.getElementById('speed-avg'),
    speedMax: document.getElementById('speed-max'),
    speedMS: document.getElementById('speed-ms'),
    speedMMS: document.getElementById('speed-mms'),

    // Distance
    distanceKm: document.getElementById('distance-totale-km'),
    distanceM: document.getElementById('distance-m'),

    // GPS Brute / Aviation
    latitude: document.getElementById('latitude'),
    longitude: document.getElementById('longitude'),
    altitudeM: document.getElementById('altitude-m'),
    altitudeFt: document.getElementById('altitude-ft'),
    accuracyHoriz: document.getElementById('accuracy-horiz'),
    // Note: Niveau à bulle (niveau-bulle) n'est pas réalisable avec l'API Geolocation.

    // Cosmique
    mach: document.getElementById('mach'),
    lightPerc: document.getElementById('light-perc'),
    minecraftTime: document.getElementById('minecraft-time'),
    
    // Astro
    leverSoleil: document.getElementById('lever-soleil'),
    coucherSoleil: document.getElementById('coucher-soleil'),
    culminationSoleil: document.getElementById('culmination-soleil'),
    leverLune: document.getElementById('lever-lune'),
    coucherLune: document.getElementById('coucher-lune'),
    culminationLune: document.getElementById('culmination-lune'),
    lunePhase: document.getElementById('lune-phase'),
    luneMagnitude: document.getElementById('lune-magnitude'),
    
    // Solaire Vraie/Moyenne (Simulées)
    hsv: document.getElementById('hsv'),
    hsm: document.getElementById('hsm'),
    eqTemps: document.getElementById('eq-temps'),
    
    // Météo (Laissé avec -- car nécessite une API externe)
    temperature: document.getElementById('temperature'),
    pression: document.getElementById('pression'),
    humidite: document.getElementById('humidite'),
    vent: document.getElementById('vent'),
    nuages: document.getElementById('nuages'),
    pluie: document.getElementById('pluie'),
    neige: document.getElementById('neige'),
    uvIndex: document.getElementById('uv-index'),
    airQuality: document.getElementById('air-quality'),
    boilingPoint: document.getElementById('boiling-point'),
};


// --- FONCTIONS UTILITAIRES ---

function msToKmh(ms) {
    if (ms === null || isNaN(ms)) return 0;
    return ms * 3.6;
}

function formatTime(totalSeconds) {
    const h = Math.floor(totalSeconds / 3600);
    const m = Math.floor((totalSeconds % 3600) / 60);
    const s = Math.floor(totalSeconds % 60);
    return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
}

function calculateDistance3D(pos1, pos2) {
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

function updateMinecraftTime() {
    // Calcul de l'heure Minecraft basée sur l'heure réelle
    const now = new Date();
    const msSinceMidnight = now.getHours() * 3600000 + now.getMinutes() * 60000 + now.getSeconds() * 1000 + now.getMilliseconds();
    
    // Ratio de temps MC : 20 minutes réelles = 24 heures MC
    const mcTimeRatio = 24000 / MINECRAFT_DAY_LENGTH_MS; 
    
    // Le jour MC commence à l'aube (6000 ticks)
    let mcTicks = (msSinceMidnight * mcTimeRatio + 6000) % 24000;

    // Conversion des ticks en heures et minutes MC
    let mcHours = Math.floor(mcTicks / 1000);
    let mcMinutes = Math.floor((mcTicks % 1000) * 0.06); 
    
    // Ajustement de l'heure MC pour un affichage "logique" (18:00 ticks = 00:00 affichage)
    if (mcHours >= 18) {
        mcHours -= 18;
    } else {
        mcHours += 6;
    }
    
    elements.minecraftTime.textContent = `${String(mcHours).padStart(2, '0')}:${String(mcMinutes).padStart(2, '0')}:${String(Math.floor(mcTicks % 1000 * 0.0036)).padStart(2, '0')}`;
}

// Lancement du timer Minecraft (indépendant du GPS)
setInterval(updateMinecraftTime, 1000);


// --- FONCTION DE MISE À JOUR PRINCIPALE ---

function success(position) {
    const coords = position.coords;
    const now = position.timestamp; // Utilisation du timestamp fourni par le GPS
    const speedMS = coords.speed !== null ? coords.speed : 0;
    let totalTimeSeconds = startTime ? (now - startTime) / 1000 : 0;
    
    // --- 1. Calcul des Vitesses et Distances ---
    
    // Distance totale (3D)
    if (lastPosition && isTracking) {
        const segmentDistance = calculateDistance3D(lastPosition, coords);
        totalDistance += segmentDistance;
        
        // Vitesse verticale et 3D (seulement si le temps écoulé est positif)
        const timeDiff = (now - lastTimestamp) / 1000;
        let vertSpeed = 0;
        let speed3D_MS = speedMS; // Initialisation par défaut à la vitesse horizontale

        if (timeDiff > 0 && lastAltitude !== null && coords.altitude !== null) {
            vertSpeed = (coords.altitude - lastAltitude) / timeDiff;
            elements.speedVert.textContent = vertSpeed.toFixed(2) + ' m/s';
            
            // Vitesse 3D (Totale)
            speed3D_MS = Math.sqrt(speedMS * speedMS + vertSpeed * vertSpeed);
            elements.speed3d.textContent = msToKmh(speed3D_MS).toFixed(2) + ' km/h';
        } else {
            elements.speedVert.textContent = '0.00 m/s';
            elements.speed3d.textContent = msToKmh(speed3D_MS).toFixed(2) + ' km/h';
        }
    }
    
    // Mise à jour des dernières valeurs
    lastPosition = coords;
    lastAltitude = coords.altitude;
    lastTimestamp = now;


    // Vitesse Instantanée (Horizontale)
    const speedKmh = msToKmh(speedMS);
    elements.speedHoriz.textContent = speedKmh.toFixed(2) + ' km/h';
    elements.speedMS.textContent = speedMS.toFixed(2) + ' m/s';
    elements.speedMMS.textContent = Math.round(speedMS * 1000) + ' mm/s';

    // Vitesse Maximale
    if (speedKmh > maxSpeed) {
        maxSpeed = speedKmh;
    }
    elements.speedMax.textContent = maxSpeed.toFixed(2) + ' km/h';

    // Vitesse Moyenne (Distance Totale / Temps Total - CORRIGÉE)
    let averageSpeedKmh = 0;
    if (totalTimeSeconds > 5 && isTracking) {
        averageSpeedKmh = (totalDistance / totalTimeSeconds) * 3.6;
    }
    elements.speedAvg.textContent = averageSpeedKmh.toFixed(2) + ' km/h';
    
    // Mise à jour Distance Totale
    elements.distanceKm.textContent = (totalDistance / 1000).toFixed(3) + ' km';
    elements.distanceM.textContent = totalDistance.toFixed(2) + ' m';


    // --- 2. Mise à jour des Grandeurs Cosmiques ---

    elements.mach.textContent = (speedMS / MACH_1_MS).toFixed(3);
    elements.lightPerc.textContent = ((speedMS / SPEED_OF_LIGHT) * 100).toFixed(8) + '%';


    // --- 3. Mise à jour des Données Brutes GPS ---
    
    elements.latitude.textContent = coords.latitude.toFixed(6);
    elements.longitude.textContent = coords.longitude.toFixed(6);
    
    if (coords.altitude !== null) {
        elements.altitudeM.textContent = coords.altitude.toFixed(1) + ' m';
        elements.altitudeFt.textContent = (coords.altitude * FT_PER_METER).toFixed(1) + ' ft';
        
        // Simulation Point d'Ébullition
        const altKm = coords.altitude / 1000;
        const boilingPoint = 100 - (altKm * 3.2); 
        elements.boilingPoint.textContent = boilingPoint.toFixed(1) + ' °C';
    } else {
        elements.altitudeM.textContent = elements.altitudeFt.textContent = elements.boilingPoint.textContent = '--';
    }
    
    elements.accuracyHoriz.textContent = coords.accuracy.toFixed(1) + ' m';
    elements.elapsedTime.textContent = formatTime(totalTimeSeconds);


    // --- 4. Mise à jour de l'Astro (SunCalc) ---
    // On ne met à jour l'astro que si on est en suivi actif, ou si c'est la première lecture
    if (isTracking || totalTimeSeconds === 0) {
        updateAstro(coords.latitude, coords.longitude);
    }
    
    // --- 5. Mise à jour du Statut ---
    elements.statusText.textContent = `Données GNSS ACTIVES. Précision: ${coords.accuracy.toFixed(1)} m`;
    elements.statusText.style.color = 'green';
}

// Fonction pour mettre à jour les données astronomiques
function updateAstro(lat, lon) {
    const times = SunCalc.getTimes(new Date(), lat, lon);
    const moon = SunCalc.getMoonIllumination(new Date());
    const moonTimes = SunCalc.getMoonTimes(new Date(), lat, lon);

    const formatTimeAstro = (date) => date ? date.toLocaleTimeString('fr-FR') : '--';
    const formatTimeAstroSimple = (date) => date ? date.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit' }) : '--';


    elements.leverSoleil.textContent = formatTimeAstroSimple(times.sunrise);
    elements.coucherSoleil.textContent = formatTimeAstroSimple(times.sunset);
    elements.culminationSoleil.textContent = formatTimeAstroSimple(times.solarNoon); 

    elements.leverLune.textContent = formatTimeAstroSimple(moonTimes.rise);
    elements.coucherLune.textContent = formatTimeAstroSimple(moonTimes.set);
    
    // Culmination lune (transit)
    const moonTransit = SunCalc.getMoonTimes(new Date(), lat, lon, true).transit;
    elements.culminationLune.textContent = formatTimeAstroSimple(moonTransit);
    
    elements.lunePhase.textContent = (moon.fraction * 100).toFixed(1) + '%';
    elements.luneMagnitude.textContent = moon.phase.toFixed(2); 
    
    // Équation du temps (différence entre heure solaire vraie et moyenne)
    // C'est très complexe, on va simuler la différence entre midi solaire et midi UTC local
    const diff = (times.solarNoon.getTime() - new Date(new Date().setHours(12, 0, 0, 0)).getTime()) / 60000;
    
    elements.eqTemps.textContent = diff.toFixed(1) + ' min';

    // Heure solaire vraie/moyenne (approximation)
    elements.hsv.textContent = formatTimeAstro(new Date(Date.now() + (diff * 60000)));
    elements.hsm.textContent = formatTimeAstro(new Date()); 
}

// --- FONCTION D'ERREUR ---
function error(err) {
    elements.statusText.textContent = `ERREUR: ${err.message || 'Position non disponible.'} Code: ${err.code}`;
    elements.statusText.style.color = 'red';
}

// --- FONCTIONS DE CONTRÔLE (Boutons) ---

function startGPS() {
    if (isTracking) return;

    if (!navigator.geolocation) {
        elements.statusText.textContent = "La géolocalisation n'est pas supportée par ce navigateur.";
        elements.statusText.style.color = 'red';
        return;
    }

    const options = {
        enableHighAccuracy: true,
        timeout: 30000,
        maximumAge: 0
    };
    
    if (startTime === null) {
         startTime = Date.now();
    }

    watchId = navigator.geolocation.watchPosition(success, error, options);
    isTracking = true;

    elements.btnStart.disabled = true;
    elements.btnStop.disabled = false;
    elements.btnReset.disabled = false;
    elements.statusText.textContent = "Suivi GPS ACTIF. Attente des données...";
    elements.statusText.style.color = 'orange';
}

function stopGPS() {
    if (!isTracking) return;
    
    if (watchId !== null) {
        navigator.geolocation.clearWatch(watchId);
        watchId = null;
    }
    isTracking = false;

    elements.btnStart.disabled = false;
    elements.btnStop.disabled = true;
    elements.statusText.textContent = "Suivi GPS en PAUSE. Données figées.";
    elements.statusText.style.color = 'gray';
}

function resetMax() {
    // 1. Sauvegarder l'état (Stop si actif)
    const wasTracking = isTracking;
    if (wasTracking) stopGPS();
    
    // 2. Réinitialiser les variables
    maxSpeed = 0;
    totalDistance = 0;
    startTime = null;
    lastPosition = null;
    lastAltitude = null;
    lastTimestamp = null;
    
    // 3. Réinitialiser l'affichage
    elements.speedHoriz.textContent = '0.00 km/h';
    elements.speedVert.textContent = '0.00 m/s';
    elements.speed3d.textContent = '0.00 km/h';
    elements.speedAvg.textContent = '0.00 km/h';
    elements.speedMax.textContent = '0.00 km/h';
    elements.speedMS.textContent = '0.00 m/s';
    elements.speedMMS.textContent = '0 mm/s';
    elements.distanceKm.textContent = '0.000 km';
    elements.distanceM.textContent = '0.00 m';
    elements.elapsedTime.textContent = '00:00:00';
    elements.mach.textContent = '0.000';
    elements.lightPerc.textContent = '0.00000000%';
    elements.altitudeM.textContent = '-- m';
    elements.altitudeFt.textContent = '-- ft';
    elements.accuracyHoriz.textContent = '-- m';
    
    // Météo et Astro (les champs astro seront recalculés au prochain start)
    elements.boilingPoint.textContent = '-- °C';
    
    // 4. Remettre l'état si nécessaire
    if (wasTracking) startGPS();

    elements.statusText.textContent = "Données réinitialisées. Prêt à commencer.";
    elements.statusText.style.color = '#2c3e50';
    elements.btnReset.disabled = true;
}

// Exposer les fonctions de contrôle aux boutons
window.startGPS = startGPS;
window.stopGPS = stopGPS;
window.resetMax = resetMax;

// Initialisation au chargement de la page
window.onload = () => {
    elements.btnStop.disabled = true;
    elements.btnReset.disabled = true;
    elements.statusText.textContent = "Prêt à démarrer. Cliquez sur Démarrer (Marche).";
    elements.statusText.style.color = '#2c3e50';

    // Initialisation des champs Météo
    elements.temperature.textContent = '-- °C';
    elements.pression.textContent = '-- hPa';
    elements.humidite.textContent = '--%';
    elements.vent.textContent = '-- km/h';
    elements.nuages.textContent = '--%';
    elements.pluie.textContent = '-- mm';
    elements.neige.textContent = '-- mm';
    elements.uvIndex.textContent = '--';
    elements.airQuality.textContent = '--';
    
    // Initialisation du champ non supporté par l'API Geolocation
    document.getElementById('niveau-bulle').textContent = '--°';
};
