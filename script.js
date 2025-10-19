// ===========================================
// 1. CONSTANTES ET VARIABLES GLOBALES
// ===========================================

// --- PHYSIQUE / VITESSES ---
const C_LIGHT = 299792458;          // Vitesse de la lumière (m/s)
const C_SOUND_SEA_LEVEL = 343;      // Vitesse du son dans l'air sec à 20°C (m/s)
const EARTH_RADIUS = 6371000;       // Rayon de la Terre (m)
const KMH_PER_MS = 3.6;             // Conversion m/s -> km/h
const M_TO_AL = 1 / 9.461e15;       // Conversion mètres -> années-lumière

// --- ÉTAT DE L'APPLICATION ---
const WATCH_OPTIONS = {
    enableHighAccuracy: true,
    maximumAge: 0,
    timeout: 5000 
};
let watchID = null;         // ID de suivi pour arrêter le GPS
let lastPosition = null;
let startTime = null;
let totalDistanceM = 0;
let maxSpeedMS = 0;

// --- CIBLE DE NAVIGATION ---
let targetLat = null;
let targetLon = null;

// --- REFERENCES DOM ---
const startBtn = document.getElementById('start-btn');
const stopBtn = document.getElementById('stop-btn');
const resetMaxBtn = document.getElementById('reset-max-btn');
const setTargetBtn = document.getElementById('set-target-btn');
const errorDisplay = document.getElementById('error-message');


// ===========================================
// 2. LOGIQUE DE CONTRÔLE ET D'ÉTAT
// ===========================================

/** Réinitialise tous les champs d'affichage à leur valeur par défaut ('--'). */
function resetDisplay() {
    // Réinitialisation des variables d'état
    lastPosition = null;
    totalDistanceM = 0;
    startTime = null;
    maxSpeedMS = 0;
    
    // Réinitialisation des affichages
    const ids = [
        'elapsed-time', 'speed-3d-inst', 'speed-avg', 'speed-max', 'speed-ms', 'speed-mms',
        'perc-light', 'perc-sound', 'distance-km-m', 'distance-mm', 'distance-cosmic-s', 'distance-cosmic-al',
        'latitude', 'longitude', 'altitude', 'gps-accuracy', 'light-sim', 'underground',
        'solar-true', 'sun-culmination', 'lunar-phase-perc', 'lunar-magnitude', 'moon-rise', 'moon-set', 'moon-culmination',
        'mc-time', 'air-temp', 'pressure', 'humidity', 'wind-speed', 'uv-index', 'boiling-point', 'air-quality',
        'heading', 'bubble-level', 'light-level', 'cap-dest', 'target-lat', 'target-lon'
    ];

    ids.forEach(id => {
        const element = document.getElementById(id);
        if (element) {
            // Affichage des valeurs par défaut
            element.textContent = '--';
        }
    });

    // Cas spéciaux
    document.getElementById('elapsed-time').textContent = '0.00 s';
    document.getElementById('mc-time').textContent = '00:00:00';
    document.getElementById('bubble-level').textContent = '--°';
    
    // Mise à jour de l'état des boutons
    startBtn.disabled = false;
    stopBtn.disabled = true;
    resetMaxBtn.disabled = true;
    setTargetBtn.textContent = '📍 Aller';
    errorDisplay.style.display = 'none';
}

/** Gère les erreurs de géolocalisation. */
function handleGeolocationError(error) {
    stopGPS(false); // Arrête le suivi sans réinitialiser l'affichage
    errorDisplay.style.display = 'block';

    switch (error.code) {
        case error.PERMISSION_DENIED:
            errorDisplay.textContent = "❌ Erreur : Accès à la géolocalisation refusé. Veuillez l'activer.";
            break;
        case error.POSITION_UNAVAILABLE:
            errorDisplay.textContent = "⚠️ Erreur : Position non disponible (Signal faible ou mode économie d'énergie).";
            break;
        case error.TIMEOUT:
            errorDisplay.textContent = "⏱️ Erreur : Délai d'attente dépassé. Connexion lente.";
            break;
        default:
            errorDisplay.textContent = "❓ Erreur GPS : Une erreur inconnue est survenue.";
            break;
    }
}

/** Démarre le suivi GPS. */
function startGPS() {
    if (!navigator.geolocation) {
        errorDisplay.style.display = 'block';
        errorDisplay.textContent = "❌ Erreur : Votre navigateur ne supporte pas la géolocalisation.";
        return;
    }

    resetDisplay(); // Réinitialise tout avant de démarrer
    startTime = Date.now();
    
    // Démarrage du suivi et stockage de l'ID
    watchID = navigator.geolocation.watchPosition(updateDisplay, handleGeolocationError, WATCH_OPTIONS);

    // Mise à jour des boutons
    startBtn.disabled = true;
    stopBtn.disabled = false;
    resetMaxBtn.disabled = false;
    errorDisplay.style.display = 'none';
}

/** Arrête le suivi GPS et réinitialise les données si demandé. */
function stopGPS(shouldReset = true) {
    if (watchID !== null) {
        navigator.geolocation.clearWatch(watchID);
        watchID = null;
    }
    
    if (shouldReset) {
        resetDisplay();
    } else {
        // Si non réinitialisé (cas d'erreur), met à jour seulement l'état du bouton
        startBtn.disabled = false;
        stopBtn.disabled = true;
    }
}

/** Réinitialise la vitesse max. */
function resetMaxSpeed() {
    maxSpeedMS = 0;
    document.getElementById('speed-max').textContent = '0.00 km/h';
}


// ===========================================
// 3. FONCTIONS DE CALCUL
// ===========================================

/** Calcule la distance entre deux points GPS (formule de Haversine). */
function calculateDistance(lat1, lon1, lat2, lon2) {
    const R = EARTH_RADIUS; // Rayon de la Terre en mètres
    const dLat = (lat2 - lat1) * (Math.PI / 180);
    const dLon = (lon2 - lon1) * (Math.PI / 180);
    
    // Convertir les latitudes en radians pour cos
    lat1 = lat1 * (Math.PI / 180);
    lat2 = lat2 * (Math.PI / 180);

    const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
              Math.cos(lat1) * Math.cos(lat2) *
              Math.sin(dLon / 2) * Math.sin(dLon / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R * c; // Distance en mètres
}

/** Calcule le cap entre deux points GPS (Bearing). */
function calculateBearing(lat1, lon1, lat2, lon2) {
    const R = Math.PI / 180;
    lat1 *= R;
    lon1 *= R;
    lat2 *= R;
    lon2 *= R;

    const y = Math.sin(lon2 - lon1) * Math.cos(lat2);
    const x = Math.cos(lat1) * Math.sin(lat2) -
              Math.sin(lat1) * Math.cos(lat2) * Math.cos(lon2 - lon1);
    
    let bearing = Math.atan2(y, x) / R;
    return (bearing + 360) % 360; // Assure une valeur entre 0 et 360
}


/** Calcule les données cosmiques et temporelles simplifiées. */
function calculateAstroData(latitude, longitude) {
    const now = new Date();
    const utcHours = now.getUTCHours();
    const utcMinutes = now.getUTCMinutes();
    
    // --- TEMPS MINECRAFT (Approximation 1s IRL = 20 ticks) ---
    // Un jour MC dure 24000 ticks (20 minutes IRL). 
    // Heures MC (0-23)
    const mcTicksPerDay = 24000;
    const msInMCMinute = 50 * 60;
    const msSinceMidnight = now.getTime() % 86400000; // ms depuis minuit UTC
    
    // Ticks depuis minuit
    const mcTicks = (msSinceMidnight * mcTicksPerDay) / 86400000;
    const mcMinutes = Math.floor(mcTicks / 20) % 1440; // 1440 minutes MC par jour
    const mcHour = Math.floor(mcMinutes / 60);
    const mcMinute = mcMinutes % 60;
    const mcSecond = Math.floor((mcTicks % 20) / 20 * 60);

    document.getElementById('mc-time').textContent = `${String(mcHour).padStart(2, '0')}:${String(mcMinute).padStart(2, '0')}:${String(mcSecond).padStart(2, '0')}`;

    // --- TEMPS SOLAIRE (Approximation) ---
    // Heure solaire moyenne (HSM) : basée sur le fuseau horaire idéal de la longitude
    // Chaque heure (15 degrés de longitude) est un fuseau.
    const solarHour = (utcHours + longitude / 15) % 24;

    document.getElementById('solar-true').textContent = `${Math.floor(solarHour).toString().padStart(2, '0')}:${String(utcMinutes).padStart(2, '0')} (HSM)`;
    document.getElementById('sun-culmination').textContent = '12:00:00 (HSM)';

    // --- LUNE (Très simplifié) ---
    // Cycle lunaire approximatif (29.53 jours)
    const newMoonDate = new Date('2000-01-06T18:14:00Z').getTime(); // Date arbitraire pour référence
    const diffDays = (now.getTime() - newMoonDate) / (1000 * 60 * 60 * 24);
    const lunarCycleDays = 29.530588853;
    const phaseDay = diffDays % lunarCycleDays;
    const phasePerc = (phaseDay / lunarCycleDays) * 100;
    
    document.getElementById('lunar-phase-perc').textContent = `${phasePerc.toFixed(1)}%`;
    document.getElementById('lunar-magnitude').textContent = '--'; // Nécessite des calculs complexes
    document.getElementById('moon-rise').textContent = '--';
    document.getElementById('moon-set').textContent = '--';
    document.getElementById('moon-culmination').textContent = '--';
}


// ===========================================
// 4. FONCTION PRINCIPALE D'AFFICHAGE
// ===========================================

/** Met à jour tous les affichages à partir des données GPS. */
function updateDisplay(position) {
    const { latitude, longitude, altitude, speed, heading, accuracy, timestamp } = position.coords;
    const currentTime = timestamp;

    // Calcul du temps écoulé total
    const elapsedTimeS = (currentTime - startTime) / 1000;

    // 1. CALCUL DE LA VITESSE 3D ET DISTANCE TOTALE
    let speedMS_Horiz_GPS = speed || 0; // Vitesse fournie par le GPS (horizontale)
    let speedMS_Vert_Calc = 0;           // Vitesse verticale (changement d'altitude)
    let speedMS_Horiz_Calc = speedMS_Horiz_GPS;

    if (lastPosition) {
        const dLat = lastPosition.coords.latitude;
        const dLon = lastPosition.coords.longitude;
        const dAlt = lastPosition.coords.altitude;
        const dt = (currentTime - lastPosition.timestamp) / 1000; // Delta temps en secondes

        if (dt > 0) {
            // Calcul de la distance horizontale et mise à jour de la distance totale
            const distHorizM = calculateDistance(dLat, dLon, latitude, longitude);
            totalDistanceM += distHorizM;

            // Si le GPS ne fournit pas la vitesse, la calculer à partir de la distance
            if (!speed) {
                speedMS_Horiz_Calc = distHorizM / dt;
            }

            // Vitesse verticale (nécessite l'altitude et un dt > 0)
            if (altitude !== null && dAlt !== null) {
                speedMS_Vert_Calc = (altitude - dAlt) / dt;
            }
        }
    }
    
    // Vitesse 3D INSTANTANÉE (Norme du vecteur vitesse)
    const speedMS_3D = Math.sqrt(
        speedMS_Horiz_Calc * speedMS_Horiz_Calc + 
        speedMS_Vert_Calc * speedMS_Vert_Calc
    );
    
    // Vitesse moyenne totale
    const speedAvgMS = elapsedTimeS > 0 ? totalDistanceM / elapsedTimeS : 0;
    
    // Mise à jour de la vitesse max
    if (speedMS_3D > maxSpeedMS) {
        maxSpeedMS = speedMS_3D;
    }


    // 2. CALCULS RELATIVISTES ET COSMOS
    const percLight = (speedMS_3D / C_LIGHT) * 100;
    const percSound = (speedMS_3D / C_SOUND_SEA_LEVEL) * 100;
    const distCosmicSL = totalDistanceM / C_LIGHT; // Distance en secondes-lumière
    const distCosmicAL = totalDistanceM * M_TO_AL; // Distance en années-lumière


    // 3. AFFICHAGE DES DONNÉES DE VITESSE ET DE DISTANCE
    document.getElementById('elapsed-time').textContent = `${elapsedTimeS.toFixed(2)} s`;
    document.getElementById('speed-3d-inst').textContent = `${(speedMS_3D * KMH_PER_MS).toFixed(2)} km/h`;
    document.getElementById('speed-avg').textContent = `${(speedAvgMS * KMH_PER_MS).toFixed(2)} km/h`;
    document.getElementById('speed-max').textContent = `${(maxSpeedMS * KMH_PER_MS).toFixed(2)} km/h`;
    document.getElementById('speed-ms').textContent = `${speedMS_3D.toFixed(3)} m/s`;
    document.getElementById('speed-mms').textContent = `${(speedMS_3D * 1000).toFixed(0)} mm/s`;

    document.getElementById('perc-light').textContent = `${percLight.toExponential(3)}%`;
    document.getElementById('perc-sound').textContent = `${percSound.toFixed(1)}%`;
    
    document.getElementById('distance-km-m').textContent = `${(totalDistanceM / 1000).toFixed(3)} km | ${(totalDistanceM % 1000).toFixed(2)} m`;
    document.getElementById('distance-mm').textContent = `${(totalDistanceM * 1000).toFixed(0)} mm`;
    document.getElementById('distance-cosmic-s').textContent = `${distCosmicSL.toExponential(3)} s lumière`;
    document.getElementById('distance-cosmic-al').textContent = `${distCosmicAL.toExponential(3)} al`;

    // 4. AFFICHAGE DE LA VUE BRUTE ET DES CAPTEURS (Simulés/GPS)
    document.getElementById('latitude').textContent = `${latitude.toFixed(6)}`;
    document.getElementById('longitude').textContent = `${longitude.toFixed(6)}`;
    document.getElementById('altitude').textContent = altitude !== null ? `${altitude.toFixed(1)} m` : '--';
    document.getElementById('gps-accuracy').textContent = accuracy !== null ? `${accuracy.toFixed(1)} m` : '--';
    
    document.getElementById('underground').textContent = altitude !== null ? (altitude < 0 ? 'Oui' : 'Non') : '--';
    
    // Capteur Lumière (Simulation simplifiée : jour/nuit)
    const hour = new Date().getHours();
    const isDaylight = hour > 6 && hour < 20;
    document.getElementById('light-sim').textContent = isDaylight ? 'Jour' : 'Nuit';
    document.getElementById('light-level').textContent = isDaylight ? '~5000 lux' : '~0 lux';
    document.getElementById('bubble-level').textContent = '--°'; // Nécessite DeviceOrientationEvent
    document.getElementById('heading').textContent = heading !== null ? `${heading.toFixed(0)}°` : '--°';


    // 5. BOUSSOLE / NAVIGATION
    if (targetLat !== null && targetLon !== null) {
        const bearing = calculateBearing(latitude, longitude, targetLat, targetLon);
        document.getElementById('cap-dest').textContent = `${bearing.toFixed(0)}°`;
        document.getElementById('target-lat').textContent = `${targetLat.toFixed(6)}`;
        document.getElementById('target-lon').textContent = `${targetLon.toFixed(6)}`;
    }

    // 6. ASTRO ET TEMPS MINECRAFT
    calculateAstroData(latitude, longitude);

    // 7. MÉTÉO (Tirets car l'API est désactivée)
    // Tous les champs météo sont gérés par le resetDisplay() et restent à '--'

    // Mise à jour pour la prochaine itération
    lastPosition = position;
}


// ===========================================
// 5. INITIALISATION
// ===========================================

/** Gère la définition de la cible de destination. */
function setTargetDestination() {
    if (!lastPosition) {
        alert("Veuillez d'abord démarrer le GPS et attendre une position.");
        return;
    }

    const currentLat = lastPosition.coords.latitude.toFixed(6);
    const currentLon = lastPosition.coords.longitude.toFixed(6);
    
    // Demande à l'utilisateur d'entrer les coordonnées cibles
    const inputLat = prompt(`Entrez la Latitude de destination (actuel: ${currentLat}°) :`, currentLat);
    const inputLon = prompt(`Entrez la Longitude de destination (actuel: ${currentLon}°) :`, currentLon);

    const lat = parseFloat(inputLat);
    const lon = parseFloat(inputLon);

    if (!isNaN(lat) && !isNaN(lon)) {
        targetLat = lat;
        targetLon = lon;
        document.getElementById('cap-dest').textContent = 'Calcul...';
        document.getElementById('target-lat').textContent = targetLat.toFixed(6);
        document.getElementById('target-lon').textContent = targetLon.toFixed(6);
        setTargetBtn.textContent = '✅ Destination définie';
    } else {
        alert("Coordonnées invalides.");
        targetLat = null;
        targetLon = null;
        setTargetBtn.textContent = '📍 Aller';
        document.getElementById('cap-dest').textContent = '--°';
    }
}

/** Initialisation de l'application (écoute des événements). */
function initApp() {
    startBtn.addEventListener('click', startGPS);
    stopBtn.addEventListener('click', () => stopGPS(true)); // Réinitialise à l'arrêt
    resetMaxBtn.addEventListener('click', resetMaxSpeed);
    setTargetBtn.addEventListener('click', setTargetDestination);
    
    resetDisplay(); // Affichage initial
}

// Démarrage de l'application une fois que le document est complètement chargé
document.addEventListener('DOMContentLoaded', initApp);
