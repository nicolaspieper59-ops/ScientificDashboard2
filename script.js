// ===================================================================
// --- 1. CONSTANTES GLOBALES ET PHYSIQUES ---
// ===================================================================

const C_LIGHT = 299792458;      // Vitesse de la lumière en m/s
const AIR_SPEED_OF_SOUND = 343; // Vitesse du son dans l'air standard (m/s)
const MS_TO_KMH = 3.6;
const FT_PER_METER = 3.28084;
const MINECRAFT_DAY_LENGTH_MS = 1200000;
const ACCURACY_THRESHOLD = 30; // Seuil de précision GPS (en mètres) pour considérer les données fiables
const G_ACCELERATION = 9.80665; // Accélération standard de la gravité (m/s²)

// Constantes Terre et Météo
const EARTH_RADIUS = 6371000; // Rayon de la Terre pour la force centrifuge (m)
const EARTH_RADIUS_AVG_KM = 6371; // Rayon moyen de la Terre (km)
const OMEGA_EARTH = 7.292115e-5; // Vitesse angulaire de rotation de la Terre (rad/s)
const SEA_LEVEL_PRESSURE = 1013.25; // Pression au niveau de la mer (hPa)
const KELVIN_OFFSET = 273.15;   // 0°C en Kelvin
const OXYGEN_PERCENT = 0.2095;  // Pourcentage d'oxygène dans l'air sec
const R_SPECIFIC_AIR = 287.058; // R spécifique de l'air sec J/(kg·K)

// --- VARIABLES DE SUIVI ET FILTRAGE ---
let watchId = null;
let totalElapsedTime = 0;
let sessionStartTime = 0;
let maxSpeed = 0;
let totalDistance = 0;
let lastPosition = null;
let timerInterval = null;
let verticalSpeedSmoothed = 0; 
const SMOOTHING_FACTOR = 0.5; // Facteur de lissage pour la vitesse verticale

// --- VARIABLES PHYSIQUES ET ÉNERGIE ---
let lastSpeedMS = 0; 
let lastCalcTimestamp = 0; 
let totalEnergyJoules = 0;

// --- VARIABLES DE TEMPS UTC ATOMIQUE (SYNCHRONISATION) ---
let utcOffsetMs = 0;           // Décalage entre l'heure locale et l'heure UTC atomique (API)
let lastUtcSyncTime = 0;       // Timestamp local du moment de la dernière synchronisation
const SYNC_INTERVAL_MS = 3600000; // Resynchroniser toutes les heures (1h)
const TIME_API_URL = "https://worldtimeapi.org/api/timezone/Etc/UTC";


// ===================================================================
// --- 2. FONCTIONS UTILITAIRES ---
// ===================================================================

function msToKmh(ms) {
    return ms * MS_TO_KMH;
}

/** Récupère la masse saisie par l'utilisateur. */
function getCurrentMass() {
    const massInput = document.getElementById('mass-input');
    const mass = parseFloat(massInput?.value);
    return isNaN(mass) || mass <= 0 ? 70 : mass;
}

/** Obtient l'heure actuelle corrigée (UTC Atomique Estimée). */
function getAtomicTime() {
    // Heure locale actuelle (NTP) + Décalage corrigé
    const currentTimeMs = Date.now() + utcOffsetMs;
    return new Date(currentTimeMs);
}

/** Calcule la distance euclidienne (3D) entre deux objets Position GPS. */
function calculateDistance3D(pos1, pos2) {
    const alt1 = (pos1.coords.altitude !== null && pos1.coords.altitudeAccuracy < ACCURACY_THRESHOLD) ? pos1.coords.altitude : 0;
    const alt2 = (pos2.coords.altitude !== null && pos2.coords.altitudeAccuracy < ACCURACY_THRESHOLD) ? pos2.coords.altitude : 0;
    
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

    const dAlt = (alt1 !== 0 && alt2 !== 0) ? (alt2 - alt1) : 0;
    
    return Math.sqrt(distance2D * distance2D + dAlt * dAlt);
}

/** Formatte les secondes totales en HH:MM:SS.ms */
function formatTime(totalSeconds) {
    const totalTimeMS = totalSeconds * 1000;
    const h = Math.floor(totalTimeMS / 3600000);
    const m = Math.floor((totalTimeMS % 3600000) / 60000);
    const s = Math.floor((totalTimeMS % 60000) / 1000);
    const ms = Math.floor((totalTimeMS % 1000) / 10);

    const pad = (num) => String(num).padStart(2, '0');
    return `${pad(h)}:${pad(m)}:${pad(s)}.${pad(ms)}`;
}

// ===================================================================
// --- 3. GESTION DU TEMPS ET SYNCHRONISATION UTC ATOMIQUE ---
// ===================================================================

/** Tente de se synchroniser avec une source de temps UTC externe. */
async function syncWithAtomicTime() {
    console.log("Tentative de synchronisation de l'heure atomique via API...");
    
    try {
        const localBeforeFetch = Date.now();
        const response = await fetch(TIME_API_URL);
        const localAfterFetch = Date.now();
        
        if (!response.ok) throw new Error(`Erreur HTTP: ${response.status}`);
        
        const data = await response.json();
        
        const unixTimeApiSec = data.unixtime; 
        const unixTimeApiMs = unixTimeApiSec * 1000;
        
        const latency = (localAfterFetch - localBeforeFetch) / 2;
        const localTimeAdjusted = localBeforeFetch + latency;
        
        utcOffsetMs = unixTimeApiMs - localTimeAdjusted;
        lastUtcSyncTime = localAfterFetch;
        
        console.log(`Synchronisation réussie. Décalage local corrigé : ${utcOffsetMs.toFixed(0)} ms.`);
        return true;
        
    } catch (error) {
        console.error("Échec de la synchronisation de l'heure atomique. Utilisation de l'heure du système.");
        utcOffsetMs = 0; // Utilise l'heure du système (Date.now()) non corrigée
        return false;
    }
}

/** Met à jour le chronomètre et retourne le temps total en secondes. */
function updateTime() {
    // Le chronomètre utilise le temps local pour mesurer l'intervalle
    let totalTimeMS = totalElapsedTime;
    if (sessionStartTime > 0) {
        totalTimeMS += (Date.now() - sessionStartTime);
    }
    const totalTimeSec = totalTimeMS / 1000;
    document.getElementById('elapsed-time').textContent = formatTime(totalTimeSec);
    return totalTimeSec;
}

function updateMinecraftTime() {
    // Utilise l'heure corrigée (Atomic Time) pour la cohérence
    const now = getAtomicTime();
    const msSinceMidnight = now.getHours() * 3600000 + now.getMinutes() * 60000 + now.getSeconds() * 1000 + now.getMilliseconds();
    
    const mcTimeRatio = 24000 / MINECRAFT_DAY_LENGTH_MS; 
    let mcTicks = (msSinceMidnight * mcTimeRatio + 6000) % 24000; // +6000 pour que minuit réel soit l'aube du jeu

    let mcHours = Math.floor(mcTicks / 1000);
    let mcMinutes = Math.floor((mcTicks % 1000) * 0.06); 
    const mcSeconds = Math.floor((mcTicks % 1000 * 0.06 * 60) % 60);

    // Ajustement des heures pour le format standard (0 = 6h)
    if (mcHours >= 18) {
        mcHours -= 18;
    } else {
        mcHours += 6;
    }
    
    const pad = (num) => String(num).padStart(2, '0');
    document.getElementById('minecraft-time').textContent = `${pad(mcHours)}:${pad(mcMinutes)}:${pad(mcSeconds)}`;
}
// Mise à jour rapide du temps Minecraft (pas besoin d'être au 100ms)
setInterval(updateMinecraftTime, 1000);


// ===================================================================
// --- 4. CALCULS SECONDAIRES (Astro, Cosmique, Science Avancée) ---
// ===================================================================

function calculateCosmicData(speedMS) {
    const mach = speedMS / AIR_SPEED_OF_SOUND;
    document.getElementById('mach').textContent = mach.toFixed(3);

    const lightPercent = (speedMS / C_LIGHT) * 100;
    document.getElementById('light-perc').textContent = `${lightPercent.toExponential(8)}%`;
}

function updateAstro(lat, lon) {
    if (typeof SunCalc === 'undefined') return;

    // Utilise l'heure corrigée (Atomic Time) pour la précision Astro
    const now = getAtomicTime();

    const times = SunCalc.getTimes(now, lat, lon);
    const moon = SunCalc.getMoonIllumination(now);
    const moonTimes = SunCalc.getMoonTimes(now, lat, lon);

    // Affichage des heures dans le fuseau horaire de l'utilisateur pour la lecture
    const formatTimeAstro = (date) => date ? date.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit', second: '2-digit' }) : '--';
    const formatTimeSimple = (date) => date ? date.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit' }) : '--';

    document.getElementById('lever-soleil').textContent = formatTimeSimple(times.sunrise);
    document.getElementById('coucher-soleil').textContent = formatTimeSimple(times.sunset);
    
    // Heure Solaire Vraie (HSV) et Équation du Temps (EqT)
    const solarNoonTime = times.solarNoon.getTime();
    const localNoonTime = new Date(new Date().setHours(12, 0, 0, 0)).getTime();
    
    const eqT_ms = solarNoonTime - localNoonTime;
    const eqT_min = eqT_ms / 60000;
    document.getElementById('eq-temps').textContent = `${eqT_min.toFixed(1)} min`;
    
    document.getElementById('hsv').textContent = formatTimeAstro(new Date(now.getTime() + eqT_ms));
    document.getElementById('hsm').textContent = formatTimeAstro(now); // Affiche l'heure Atomique Estimée

    // Lune
    document.getElementById('lune-phase').textContent = (moon.fraction * 100).toFixed(1) + '%';
}

/** Met à jour les grandeurs de Physique, Chimie et SVT simulées. */
function updateAdvancedScience(lat, speedMS_3D, altitudeM) {
    // Si l'altitude n'est pas fiable, on utilise 0m
    const alt = altitudeM !== null ? altitudeM : 0; 
    const latRad = lat * (Math.PI / 180);

    // --- 1. PHYSIQUE & MÉCANIQUE DES FLUIDES ---
    
    const TEMP_SEA_LEVEL = 15; // °C
    const tempCelsius = TEMP_SEA_LEVEL - (6.5 * alt / 1000); 
    const tempKelvin = tempCelsius + KELVIN_OFFSET;
    
    // Pression Atmosphérique (Formule barométrique simplifiée)
    const pressureHpa = SEA_LEVEL_PRESSURE * Math.pow(1 - (0.0065 * alt / (TEMP_SEA_LEVEL + KELVIN_OFFSET)), 5.257); 
    
    // Masse Volumique de l'air
    const densityKgM3 = (pressureHpa * 100) / (R_SPECIFIC_AIR * tempKelvin); 

    // Portée de l'Horizon (Vue géométrique) : d = sqrt(2 * R * h)
    const porteeHorizonKm = Math.sqrt(2 * EARTH_RADIUS_AVG_KM * (alt / 1000));
    
    // Force de Coriolis
    const currentMass = getCurrentMass();
    const coriolisForceNewton = 2 * currentMass * OMEGA_EARTH * speedMS_3D * Math.sin(latRad);


    // --- 2. CHIMIE & SVT ---

    // Taux d'oxygène (Pression Partielle relative au niveau de la mer)
    const partialPressureOxy = pressureHpa * OXYGEN_PERCENT; 
    const tauxOxygene = (partialPressureOxy / SEA_LEVEL_PRESSURE) * 100;
    
    // Température au sol (SVT - Estimation de la température corporelle en réponse au froid/chaud)
    const tempSolSVT = 37.0 - (tempCelsius / 30); 
    
    // Durée de vie théorique (Simulée : temps écoulé de la session en minutes)
    const lifeTimeSimulated = updateTime() / 60;


    // --- 3. MISE À JOUR DE L'AFFICHAGE ---

    document.getElementById('pression-atm').textContent = `${pressureHpa.toFixed(2)} hPa`;
    document.getElementById('masse-volumique-air').textContent = `${densityKgM3.toFixed(4)} kg/m³`;
    document.getElementById('portee-horizon').textContent = `${porteeHorizonKm.toFixed(2)} km`;
    document.getElementById('coriolis-force').textContent = `${Math.abs(coriolisForceNewton).toExponential(2)} N`;

    document.getElementById('taux-oxygene').textContent = `${tauxOxygene.toFixed(1)} % (pp)`;
    document.getElementById('temp-sol').textContent = `${tempSolSVT.toFixed(1)} °C`;
    document.getElementById('duree-vie-theorique').textContent = `${lifeTimeSimulated.toFixed(2)} min`;
}

// ===================================================================
// --- 5. FONCTION PRINCIPALE : updateDisplay (Callback GPS) ---
// ===================================================================

function updateDisplay(position) {
    const coords = position.coords;
    const nowTimestamp = position.timestamp;
    
    // 1. LOGIQUE DE COUPE/REPRISE DU SIGNAL (Précision 3D)
    const isReliable = coords.accuracy <= ACCURACY_THRESHOLD && 
                       (coords.altitude === null || coords.altitudeAccuracy === null || coords.altitudeAccuracy <= ACCURACY_THRESHOLD);

    if (!isReliable) {
        stopGPS(false);
        document.getElementById('gps-status-text').textContent = `Signal faible/imprécis (H: ${coords.accuracy.toFixed(1)}m, V: ${coords.altitudeAccuracy?.toFixed(1) || '--'}m). PAUSE.`;
        document.getElementById('gps-status-text').style.color = 'orange';
        return;
    }

    if (sessionStartTime === 0 && watchId !== null) {
        sessionStartTime = Date.now();
        timerInterval = setInterval(updateTime, 100);
        document.getElementById('gps-status-text').textContent = 'Actif / Fixe';
        document.getElementById('gps-status-text').style.color = 'green';
    }


    // 2. CALCUL VITESSE/ACCÉLÉRATION/DISTANCES
    let speedMS_Reconstituée_3D = 0; 
    let verticalSpeedRaw = 0;
    let acceleration = 0;
    let timeDelta = 0;
    let distanceIncrement = 0;
    
    if (lastPosition) {
        distanceIncrement = calculateDistance3D(lastPosition, position);
        totalDistance += distanceIncrement;
        
        timeDelta = (nowTimestamp - lastPosition.timestamp) / 1000;
        
        if (timeDelta > 0.1) {
            speedMS_Reconstituée_3D = distanceIncrement / timeDelta; 
            acceleration = (speedMS_Reconstituée_3D - lastSpeedMS) / timeDelta;
            
            if (lastPosition.coords.altitude !== null && coords.altitude !== null) {
                 verticalSpeedRaw = (coords.altitude - lastPosition.coords.altitude) / timeDelta;
                 // Lissage pour stabiliser la vitesse verticale (Clé de la précision 3D)
                 verticalSpeedSmoothed = verticalSpeedSmoothed * (1 - SMOOTHING_FACTOR) + verticalSpeedRaw * SMOOTHING_FACTOR;
            }
        }
    }
    
    const verticalSpeed = verticalSpeedSmoothed;
    // Vitesse Horizontale par projection (cohérence 3D)
    const speedMS_Horiz_Calc = Math.sqrt(
        Math.max(0, speedMS_Reconstituée_3D * speedMS_Reconstituée_3D - verticalSpeed * verticalSpeed)
    );
    const speedKmh_3D_Reconstituée = msToKmh(speedMS_Reconstituée_3D);
    const speedKmh_Horiz_Calc = msToKmh(speedMS_Horiz_Calc);

    // 3. CALCULS PHYSIQUES (FORCE, ENERGIE, G)
    const currentMass = getCurrentMass();
    const forceNewton = Math.abs(currentMass * acceleration); 
    const gSensation = Math.abs(acceleration) / G_ACCELERATION;
    
    let powerWatt = 0;
    let workJoules = 0;

    if (timeDelta > 0 && acceleration !== 0) {
        workJoules = forceNewton * distanceIncrement;
        powerWatt = workJoules / timeDelta;
        if (workJoules > 0) { 
             totalEnergyJoules += workJoules; 
        }
    }
    
    const centrifugalForceNewton = currentMass * (speedMS_Reconstituée_3D * speedMS_Reconstituée_3D / EARTH_RADIUS);
    
    
    // 4. MISE À JOUR DE L'AFFICHAGE (PHYSIQUE)
    document.getElementById('force-newton').textContent = `${forceNewton.toFixed(2)} N`;
    document.getElementById('power-watt').textContent = `${powerWatt.toFixed(2)} W`;
    document.getElementById('energy-joule').textContent = `${totalEnergyJoules.toFixed(2)} J`;
    document.getElementById('g-sensation').textContent = `${gSensation.toFixed(3)} G`;
    document.getElementById('centrifugal-force').textContent = `${centrifugalForceNewton.toExponential(2)} N`;
    document.getElementById('acceleration-ms2').textContent = `${Math.abs(acceleration).toFixed(2)} m/s²`;


    // 5. MISE À JOUR DE L'AFFICHAGE (POSITION, VITESSE, DISTANCE)
    const totalTimeSec = updateTime(); 
    const avgSpeedMS = totalTimeSec > 5 ? totalDistance / totalTimeSec : 0; 
    if (speedKmh_Horiz_Calc > maxSpeed) { maxSpeed = speedKmh_Horiz_Calc; }

    document.getElementById('speed-vert').textContent = `${verticalSpeed.toFixed(2)} m/s`;
    document.getElementById('speed-horiz').textContent = `${speedKmh_Horiz_Calc.toFixed(2)} km/h`;
    document.getElementById('speed-3d').textContent = `${speedKmh_3D_Reconstituée.toFixed(2)} km/h`;
    document.getElementById('speed-ms').textContent = `${speedMS_Reconstituée_3D.toFixed(2)} m/s`;
    
    document.getElementById('speed-max').textContent = `${maxSpeed.toFixed(2)} km/h`;
    document.getElementById('speed-avg').textContent = `${msToKmh(avgSpeedMS).toFixed(2)} km/h`;

    document.getElementById('distance-totale-km').textContent = `${(totalDistance / 1000).toFixed(3)} km`;
    document.getElementById('distance-m').textContent = `${totalDistance.toFixed(2)} m`;
    
    const altitudeM = coords.altitude !== null && coords.altitudeAccuracy <= ACCURACY_THRESHOLD ? coords.altitude : null;
    document.getElementById('latitude').textContent = coords.latitude !== null ? coords.latitude.toFixed(6) : '--';
    document.getElementById('longitude').textContent = coords.longitude !== null ? coords.longitude.toFixed(6) : '--';

    if (altitudeM !== null) {
        document.getElementById('altitude-m').textContent = `${altitudeM.toFixed(2)} m`;
        document.getElementById('altitude-ft').textContent = `${(altitudeM * FT_PER_METER).toFixed(2)} ft`;
    } else {
        document.getElementById('altitude-m').textContent = '-- m (non fiable)';
        document.getElementById('altitude-ft').textContent = '-- ft (non fiable)';
    }

    document.getElementById('accuracy-horiz').textContent = `${coords.accuracy.toFixed(2)} m`;

    // 6. CALCULS AVANCÉS
    calculateCosmicData(speedMS_Reconstituée_3D);
    updateAstro(coords.latitude, coords.longitude);
    updateAdvancedScience(coords.latitude, speedMS_Reconstituée_3D, altitudeM);
    
    // 7. SAUVEGARDE POUR LE PROCHAIN CALCUL
    lastPosition = position;
    lastSpeedMS = speedMS_Reconstituée_3D;
    lastCalcTimestamp = nowTimestamp;
}

// ===================================================================
// --- 6. GESTIONNAIRES DE CONTRÔLE ET ERREUR ---
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
    
    if (error.code !== error.PERMISSION_DENIED) {
        stopGPS(false); 
    }
}

function startGPS() {
    if (watchId) return;

    if ('geolocation' in navigator) {
        // Synchroniser l'heure si la dernière synchro est trop ancienne
        if (Date.now() - lastUtcSyncTime > SYNC_INTERVAL_MS || lastUtcSyncTime === 0) {
            syncWithAtomicTime(); 
        }

        const options = { enableHighAccuracy: true, timeout: 30000, maximumAge: 0 }; 
        
        watchId = navigator.geolocation.watchPosition(updateDisplay, errorCallback, options);

        if (sessionStartTime === 0) { 
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

function stopGPS(clearGPSWatch = true) {
    if (sessionStartTime > 0) {
        totalElapsedTime += (Date.now() - sessionStartTime);
        sessionStartTime = 0;
    }
    clearInterval(timerInterval);

    if (watchId && clearGPSWatch) {
        navi
