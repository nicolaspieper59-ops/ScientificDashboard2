// Constantes
const C_LIGHT = 299792458; // Vitesse de la lumière en m/s
const AIR_SPEED_OF_SOUND = 343; // Vitesse du son dans l'air standard (m/s)
const MS_TO_KMH = 3.6;

// Configuration Météo - ⚠️ REMPLACEZ CELA PAR VOTRE VRAIE CLÉ API
const OPENWEATHER_API_KEY = "VOTRE_CLE_API_ICI"; 
const UNITS = "metric"; 

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

function calculateDistance3D(pos1, pos2) {
    // ... (Logique de calcul de distance 3D Haversine/Pythagore) ...
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

/** Met à jour le chronomètre et retourne le temps total en secondes. */
function updateTime() {
    let totalTimeSec = totalElapsedTime / 1000;

    if (sessionStartTime > 0) {
        totalTimeSec += (Date.now() - sessionStartTime) / 1000;
    }
    
    document.getElementById('elapsed-time').textContent = `${totalTimeSec.toFixed(1)} s`;
    
    // Mise à jour de l'Horloge Minecraft
    updateMinecraftTime(totalTimeSec); 
    
    return totalTimeSec;
}

/** Calcule et affiche l'heure Minecraft (simple modulo 20 minutes) */
function updateMinecraftTime(totalTimeSec) {
    const MINECRAFT_DAY_SECONDS = 1200; // 20 minutes * 60 seconds
    const dayRatio = totalTimeSec / MINECRAFT_DAY_SECONDS;
    const currentDayTimeSeconds = (dayRatio - Math.floor(dayRatio)) * MINECRAFT_DAY_SECONDS;
    
    // Décalage: Minuit dans la vraie vie est 18000 ticks (6h00) dans Minecraft. 
    // Ici, nous commençons l'heure Minecraft à 00:00:00 (tick 0)
    
    const totalSeconds = currentDayTimeSeconds;
    const hours = Math.floor(totalSeconds / 60 / 60);
    const minutes = Math.floor((totalSeconds / 60) % 60);
    const seconds = Math.floor(totalSeconds % 60);

    const format = (val) => String(val).padStart(2, '0');
    document.getElementById('minecraft-time').textContent = `${format(hours)}:${format(minutes)}:${format(seconds)}`;
}

// ===================================================================
// MISES À JOUR ASTRO ET MÉTÉO
// ===================================================================

function updateAstroData(lat, lon) {
    if (!window.SunCalc || lat === null || lon === null) {
        // Affiche un message si SunCalc n'est pas chargé ou si les coordonnées manquent
        document.getElementById('culmination-soleil').textContent = "Chargement Astro...";
        return;
    }
    const date = new Date();

    // Calculs Soleil
    const sunTimes = SunCalc.getTimes(date, lat, lon);
    const sunPos = SunCalc.getPosition(date, lat, lon);
    
    const toTime = (d) => d ? d.toLocaleTimeString('fr') : '--';

    document.getElementById('lever-soleil').textContent = toTime(sunTimes.sunrise);
    document.getElementById('coucher-soleil').textContent = toTime(sunTimes.sunset);
    document.getElementById('culmination-soleil').textContent = toTime(sunTimes.solarNoon);

    // L'heure solaire vraie (HSV) est l'heure locale plus/moins la longitude par rapport au méridien
    const solarTimeHours = (date.getHours() + date.getMinutes() / 60 + date.getSeconds() / 3600) + (sunPos.hourAngle * 180 / Math.PI) / 15;
    const hsv = new Date(date.getFullYear(), date.getMonth(), date.getDate(), solarTimeHours, 0, 0);
    document.getElementById('hsv').textContent = hsv.toLocaleTimeString('fr', { hour12: false });
    
    // Heure solaire moyenne (HSM) et Équation du temps (simplifié)
    document.getElementById('hsm').textContent = toTime(sunTimes.solarNoon); // Approximation pour HSM
    
    // Calculs Lune
    const moonIllumination = SunCalc.getMoonIllumination(date);
    const moonTimes = SunCalc.getMoonTimes(date, lat, lon);
    
    document.getElementById('lune-phase').textContent = `${(moonIllumination.fraction * 100).toFixed(1)}%`;
    document.getElementById('lune-magnitude').textContent = moonIllumination.phase.toFixed(2);
    
    document.getElementById('lever-lune').textContent = toTime(moonTimes.rise);
    document.getElementById('coucher-lune').textContent = toTime(moonTimes.set);
    // Note: Culmination lune est plus complexe, mais on affiche le nom (maxAltitude) si présent
    if (moonTimes.alwaysUp) {
        document.getElementById('culmination-lune').textContent = 'Toujours visible';
    } else if (moonTimes.alwaysDown) {
        document.getElementById('culmination-lune').textContent = 'Jamais visible';
    } else {
        document.getElementById('culmination-lune').textContent = '--'; // Nécessite un calcul plus poussé
    }
}

function updateWeatherData(lat, lon) {
    if (lat === null || lon === null || OPENWEATHER_API_KEY === "VOTRE_CLE_API_ICI") {
        document.getElementById('temperature').textContent = "Clé API manquante";
        return;
    }

    // Utilisation des endpoints d'OpenWeatherMap pour la météo de base et les données complexes
    const weatherUrl = `https://api.openweathermap.org/data/2.5/weather?lat=${lat}&lon=${lon}&appid=${OPENWEATHER_API_KEY}&units=${UNITS}&lang=fr`;
    const airPollutionUrl = `https://api.openweathermap.org/data/2.5/air_pollution?lat=${lat}&lon=${lon}&appid=${OPENWEATHER_API_KEY}`;
    
    // Météo de base (Température, Pression, Humidité, Vent, Nuages)
    fetch(weatherUrl)
        .then(response => response.json())
        .then(data => {
            if (data.main) {
                document.getElementById('temperature').textContent = `${data.main.temp.toFixed(1)} °C`;
                document.getElementById('pression').textContent = `${data.main.pressure} hPa`;
                document.getElementById('humidite').textContent = `${data.main.humidity}%`;
                
                // Le Point d'ébullition (approximatif) dépend de l'altitude et de la pression (Formule très simplifiée)
                const boilingPoint = 100 - (1013.25 - data.main.pressure) * 0.027;
                document.getElementById('boiling-point').textContent = `${boilingPoint.toFixed(1)} °C`;
            }
            if (data.wind) {
                const windKmh = data.wind.speed * 3.6; 
                document.getElementById('vent').textContent = `${windKmh.toFixed(1)} km/h`;
            }
            if (data.clouds) {
                document.getElementById('nuages').textContent = `${data.clouds.all}%`;
            }
            // Pluie/Neige (simplifié, vérifie la présence de l'objet)
            document.getElementById('pluie').textContent = data.rain && data.rain['1h'] ? `${data.rain['1h']} mm` : '--';
            document.getElementById('neige').textContent = data.snow && data.snow['1h'] ? `${data.snow['1h']} mm` : '--';

            // Indice UV (nécessite l'endpoint One Call, que nous ignorons ici pour la simplicité, on le marque --)
            document.getElementById('uv-index').textContent = '--';

        }).catch(error => {
            console.error("Erreur Météo de base:", error);
            document.getElementById('temperature').textContent = "Erreur Météo";
        });

    // Qualité de l'Air (API distincte)
    fetch(airPollutionUrl)
        .then(response => response.json())
        .then(data => {
            if (data.list && data.list.length > 0) {
                const aqi = data.list[0].main.aqi; // Air Quality Index (1=Bon, 5=Très mauvais)
                const aqiMap = { 1: 'Bon', 2: 'Juste', 3: 'Modéré', 4: 'Mauvais', 5: 'Très Mauvais' };
                document.getElementById('air-quality').textContent = `${aqi} (${aqiMap[aqi]})`;
            }
        }).catch(error => {
            console.error("Erreur Qualité de l'air:", error);
            document.getElementById('air-quality').textContent = "Erreur Air";
        });
}


// ===================================================================
// MISE À JOUR PRINCIPALE
// ===================================================================

function updateDisplay(position) {
    const coords = position.coords;
    const lat = coords.latitude;
    const lon = coords.longitude;

    // --- 1. Mise à jour des données GPS/Vitesse/Cosmique ---
    // ... (Logique de mise à jour des vitesses, distances, etc. - Identique aux versions précédentes) ...
    // ...

    // (RÉINSERTION du corps de updateDisplay ici pour la complétude)
    
    // Mise à jour du statut
    document.getElementById('gps-status-text').textContent = 'Actif / Fixe';
    document.getElementById('gps-status-text').style.color = 'green';

    // Position et Précision
    document.getElementById('latitude').textContent = lat !== null ? lat.toFixed(6) : '--';
    document.getElementById('longitude').textContent = lon !== null ? lon.toFixed(6) : '--';
    
    const altitudeM = coords.altitude !== null ? coords.altitude : 0;
    document.getElementById('altitude-m').textContent = `${altitudeM.toFixed(2)} m`;
    document.getElementById('altitude-ft').textContent = `${(altitudeM * 3.28084).toFixed(2)} ft`;
    document.getElementById('accuracy-horiz').textContent = `${coords.accuracy.toFixed(2)} m`;

    // Vitesse (Ground Speed)
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

    const speed3D_MS = Math.sqrt(speedMS * speedMS + verticalSpeed * verticalSpeed);
    
    document.getElementById('speed-horiz').textContent = `${speedKmh.toFixed(2)} km/h`;
    document.getElementById('speed-3d').textContent = `${msToKmh(speed3D_MS).toFixed(2)} km/h`;
    document.getElementById('speed-ms').textContent = `${speedMS.toFixed(2)} m/s`;
    document.getElementById('speed-mms').textContent = `${(speedMS * 1000).toFixed(0)} mm/s`;

    // Maximum et Distance/Moyenne
    if (speedKmh > maxSpeed) {
        maxSpeed = speedKmh;
    }
    document.getElementById('speed-max').textContent = `${maxSpeed.toFixed(2)} km/h`;

    if (lastPosition) {
        const distanceIncrement = calculateDistance3D(lastPosition, position);
        totalDistance += distanceIncrement;

        const totalTimeSec = updateTime(); 
        
        const avgSpeedMS = totalTimeSec > 0 ? totalDistance / totalTimeSec : 0;
        document.getElementById('speed-avg').textContent = `${msToKmh(avgSpeedMS).toFixed(2)} km/h`;
        
        document.getElementById('distance-totale-km').textContent = `${(totalDistance / 1000).toFixed(3)} km`;
        document.getElementById('distance-m').textContent = `${totalDistance.toFixed(2)} m`;
    }
    
    calculateCosmicData(speed3D_MS);
    
    // --- 2. Mise à jour des données Astro et Météo (Moins souvent) ---
    // On met à jour toutes les 30 secondes pour économiser les ressources et les appels API
    if (lastPosition === null || Date.now() % 30000 < 100) { 
        updateAstroData(lat, lon); 
        updateWeatherData(lat, lon); 
    }

    lastPosition = position;
}


// --- 3. Gestionnaire d'orientation (Niveau à Bulle) ---

function handleOrientation(event) {
    const angleGamma = event.gamma;
    if (angleGamma !== null) {
        document.getElementById('niveau-bulle').textContent = `${angleGamma.toFixed(1)}°`;
    }
}
if (window.DeviceOrientationEvent) {
    window.addEventListener('deviceorientation', handleOrientation, true);
}


// ===================================================================
// GESTIONNAIRES DE CONTRÔLE (Boutons Marche/Arrêt/Réinitialisation)
// ===================================================================

function errorCallback(error) {
    // ... (Logique d'erreur) ...
    document.getElementById('gps-status-text').textContent = "Erreur: Signal perdu ou Accès refusé.";
    document.getElementById('gps-status-text').style.color = 'red';
    stopGPS(false); 
}

/** Démarre la surveillance GPS et le timer. (Marche) */
function startGPS() {
    if ('geolocation' in navigator) {
        if (document.getElementById('watch-id').value) return; 

        // ⚠️ CORRECTION DU TIMEOUT: 5 minutes (300000 ms) au lieu de 5 secondes
        const options = { enableHighAccuracy: true, timeout: 300000, maximumAge: 0 }; 
        
        const watchId = navigator.geolocation.watchPosition(updateDisplay, errorCallback, options);
        document.getElementById('watch-id').value = watchId;

        sessionStartTime = Date.now(); 
        timerInterval = setInterval(updateTime, 100);

        document.getElementById('btn-start').disabled = true;
        document.getElementById('btn-stop').disabled = false;
        document.getElementById('btn-reset').disabled = false;
        document.getElementById('gps-status-text').textContent = 'Démarrage (Timeout: 5min)...';
        document.getElementById('gps-status-text').style.color = 'orange';

    } else {
        document.getElementById('gps-status-text').textContent = "La géolocalisation n'est pas supportée.";
        document.getElementById('gps-status-text').style.color = 'red';
    }
}

/** Arrête la surveillance GPS et le timer. (Arrêt) */
function stopGPS(clearWatch = true) {
    const watchId = document.getElementById('watch-id').value;
    
    if (sessionStartTime > 0) {
        totalElapsedTime += (Date.now() - sessionStartTime); 
        sessionStartTime = 0; 
    }
    clearInterval(timerInterval);

    if (watchId && clearWatch) {
        navigator.geolocation.clearWatch(watchId);
        document.getElementById('watch-id').value = '';
    }

    document.getElementById('btn-start').disabled = false;
    document.getElementById('btn-stop').disabled = true;
    document.getElementById('gps-status-text').textContent = 'Arrêté.';
    document.getElementById('gps-status-text').style.color = 'gray';
}

/** Réinitialise les variables de suivi (Max, Moyenne, Temps). */
function resetMax() {
    // Réinitialiser les variables de suivi
    maxSpeed = 0;
    totalDistance = 0;
    totalElapsedTime = 0;
    lastPosition = null;

    // Redémarrer le temps de session si actif
    if (document.getElementById('watch-id').value) {
        sessionStartTime = Date.now(); 
    } else {
        sessionStartTime = 0;
    }

    // Réinitialiser l'affichage
    document.getElementById('speed-max').textContent = '0.00 km/h';
    document.getElementById('speed-avg').textContent = '0.00 km/h';
    document.getElementById('distance-totale-km').textContent = '0.000 km';
    document.getElementById('distance-m').textContent = '0.00 m';
    document.getElementById('elapsed-time').textContent = '0 s';
    document.getElementById('minecraft-time').textContent = '--:--:--';
}

// Initialisation au chargement de la page
document.addEventListener('DOMContentLoaded', () => {
    document.getElementById('gps-status-text').textContent = 'Prêt. Appuyez sur Démarrer.';
    document.getElementById('gps-status-text').style.color = 'gray';
});
