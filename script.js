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
    maximumAge: 500,
    timeout: 10000 
};
let watchID = null;         
let lastPosition = null;
let startTime = null;
let totalDistanceM = 0;
let maxSpeedMS = 0;

// --- CIBLE DE NAVIGATION ---
let targetLat = null;
let targetLon = null;

// --- REFERENCES DOM (Inchangées) ---
const startBtn = document.getElementById('start-btn');
const stopBtn = document.getElementById('stop-btn');
const resetMaxBtn = document.getElementById('reset-max-btn');
const setTargetBtn = document.getElementById('set-target-btn');
const errorDisplay = document.getElementById('error-message');


// ===========================================
// 2. LOGIQUE DE CONTRÔLE ET D'ÉTAT (Inchangée)
// ===========================================

function resetDisplay() {
    lastPosition = null;
    totalDistanceM = 0;
    startTime = null;
    maxSpeedMS = 0;
    targetLat = null; 
    targetLon = null;
    
    // Réinitialisation de tous les champs
    const ids = ['elapsed-time', 'speed-3d-inst', 'speed-avg', 'speed-max', 'speed-ms', 'speed-mms',
        'perc-light', 'perc-sound', 'distance-km-m', 'distance-mm', 'distance-cosmic-s', 'distance-cosmic-al',
        'latitude', 'longitude', 'altitude', 'gps-accuracy', 'light-sim', 'underground',
        'solar-true', 'sun-culmination', 'lunar-phase-perc', 'lunar-magnitude', 'moon-rise', 'moon-set', 'moon-culmination',
        'mc-time', 'air-temp', 'pressure', 'humidity', 'wind-speed', 'uv-index', 'boiling-point', 'air-quality',
        'heading', 'bubble-level', 'light-level', 'cap-dest', 'target-lat', 'target-lon'
    ];

    ids.forEach(id => {
        const element = document.getElementById(id);
        if (element) {
            element.textContent = '--';
        }
    });

    document.getElementById('elapsed-time').textContent = '0.00 s';
    document.getElementById('mc-time').textContent = '00:00:00';
    document.getElementById('bubble-level').textContent = '--°';
    
    startBtn.disabled = false;
    stopBtn.disabled = true;
    resetMaxBtn.disabled = true;
    setTargetBtn.textContent = '📍 Aller';
    errorDisplay.style.display = 'none';
}

function handleGeolocationError(error) {
    stopGPS(false);
    errorDisplay.style.display = 'block';

    switch (error.code) {
        case error.PERMISSION_DENIED:
            errorDisplay.textContent = "❌ L'accès à la localisation a été refusé. Veuillez l'activer.";
            break;
        case error.POSITION_UNAVAILABLE:
            errorDisplay.textContent = "⚠️ Position non disponible. Assurez-vous d'être en extérieur.";
            break;
        case error.TIMEOUT:
            errorDisplay.textContent = "⏱️ Délai de recherche du GPS dépassé.";
            break;
        default:
            errorDisplay.textContent = "❓ Erreur inconnue du GPS.";
            break;
    }
}

function startGPS() {
    if (!navigator.geolocation) {
        errorDisplay.style.display = 'block';
        errorDisplay.textContent = "❌ Votre navigateur ne supporte pas la géolocalisation.";
        return;
    }

    resetDisplay(); 
    startTime = Date.now();
    
    // Si la première tentative échoue, l'erreur est gérée par handleGeolocationError
    watchID = navigator.geolocation.watchPosition(updateDisplay, handleGeolocationError, WATCH_OPTIONS);

    initOrientationSensors(); 

    startBtn.disabled = true;
    stopBtn.disabled = false;
    resetMaxBtn.disabled = false;
    errorDisplay.style.display = 'none';
}

function stopGPS(shouldReset = true) {
    if (watchID !== null) {
        navigator.geolocation.clearWatch(watchID);
        watchID = null;
    }
    stopOrientationSensors();
    
    if (shouldReset) {
        resetDisplay();
    } else {
        startBtn.disabled = false;
        stopBtn.disabled = true;
    }
}

function resetMaxSpeed() {
    maxSpeedMS = 0;
    document.getElementById('speed-max').textContent = '0.00 km/h';
}


// ===========================================
// 3. FONCTIONS DE CALCUL (Inchangées)
// ===========================================

function calculateDistance(lat1, lon1, lat2, lon2) {
    const R = EARTH_RADIUS;
    const dLat = (lat2 - lat1) * (Math.PI / 180);
    const dLon = (lon2 - lon1) * (Math.PI / 180);
    lat1 *= (Math.PI / 180);
    lat2 *= (Math.PI / 180);

    const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
              Math.cos(lat1) * Math.cos(lat2) *
              Math.sin(dLon / 2) * Math.sin(dLon / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R * c;
}

function calculateBearing(lat1, lon1, lat2, lon2) {
    const R = Math.PI / 180;
    lat1 *= R; lon1 *= R; lat2 *= R; lon2 *= R;

    const y = Math.sin(lon2 - lon1) * Math.cos(lat2);
    const x = Math.cos(lat1) * Math.sin(lat2) -
              Math.sin(lat1) * Math.cos(lat2) * Math.cos(lon2 - lon1);
    
    let bearing = Math.atan2(y, x) / R;
    return (bearing + 360) % 360; 
}


function calculateAstroData(latitude, longitude) {
    const now = new Date();
    
    // TEMPS MINECRAFT (Simplifié)
    const mcTicksPerDay = 24000;
    const msSinceMidnight = now.getTime() % 86400000;
    const mcTicks = (msSinceMidnight * mcTicksPerDay) / 86400000;
    const mcMinutes = Math.floor(mcTicks / 20) % 1440; 
    const mcHour = Math.floor(mcMinutes / 60);
    const mcMinute = mcMinutes % 60;
    const mcSecond = Math.floor((mcTicks % 20) / 20 * 60);

    document.getElementById('mc-time').textContent = `${String(mcHour).padStart(2, '0')}:${String(mcMinute).padStart(2, '0')}:${String(mcSecond).padStart(2, '0')}`;

    // TEMPS SOLAIRE (Simplifié)
    const solarHour = (now.getUTCHours() + longitude / 15) % 24;
    document.getElementById('solar-true').textContent = `${Math.floor(solarHour).toString().padStart(2, '0')}:${String(now.getUTCMinutes()).padStart(2, '0')} (HSM)`;
    document.getElementById('sun-culmination').textContent = '12:00:00 (HSM)';

    // LUNE (Simplifié)
    const newMoonDate = new Date('2000-01-06T18:14:00Z').getTime();
    const diffDays = (now.getTime() - newMoonDate) / (1000 * 60 * 60 * 24);
    const lunarCycleDays = 29.530588853;
    const phaseDay = diffDays % lunarCycleDays;
    const phasePerc = (phaseDay / lunarCycleDays) * 100;
    
    document.getElementById('lunar-phase-perc').textContent = `${phasePerc.toFixed(1)}%`;
}

// ===========================================
// 4. GESTION DES CAPTEURS SPÉCIFIQUES ANDROID (Inchangée)
// ===========================================

function handleDeviceOrientation(event) {
    const beta = event.beta;
    document.getElementById('bubble-level').textContent = `${beta.toFixed(1)}°`; 
}

function initOrientationSensors() {
    if (window.DeviceOrientationEvent) {
        window.addEventListener('deviceorientation', handleDeviceOrientation);
    } else {
        document.getElementById('bubble-level').textContent = 'Non supporté';
    }
}

function stopOrientationSensors() {
    window.removeEventListener('deviceorientation', handleDeviceOrientation);
}


// ===========================================
// 5. FONCTION PRINCIPALE D'AFFICHAGE (updateDisplay)
// ===========================================

/** Met à jour tous les affichages à partir des données GPS. */
function updateDisplay(position) {
    const { latitude, longitude, accuracy, timestamp } = position.coords;
    
    // Utiliser la vitesse fournie par le GPS (null si non fournie)
    const speed = position.coords.speed;
    const altitude = position.coords.altitude; 
    const heading = position.coords.heading || null;

    const currentTime = timestamp;
    const elapsedTimeS = (currentTime - startTime) / 1000;

    // --- CALCULS DE VITESSE ET DISTANCE (LOGIQUE RESTAURÉE) ---
    
    // Vitesse horizontale instantanée (en m/s)
    let speedMS_Horiz = speed !== null ? speed : 0; // Prioriser la vitesse GPS, sinon 0.
    
    // Vitesse verticale instantanée (en m/s)
    let speedMS_Vert = 0;
    
    if (lastPosition && lastPosition.coords.latitude !== undefined) { 
        const dLat = lastPosition.coords.latitude;
        const dLon = lastPosition.coords.longitude;
        const dAlt = lastPosition.coords.altitude; 
        const dt = (currentTime - lastPosition.timestamp) / 1000; 

        if (dt > 0) {
            const distHorizM = calculateDistance(dLat, dLon, latitude, longitude);
            totalDistanceM += distHorizM;

            // 2. CALCULER LA VITESSE HORIZONTALE à partir de la distance si la vitesse GPS était manquante (speed === null).
            if (speed === null) {
                 speedMS_Horiz = distHorizM / dt;
            }

            // 3. CALCULER LA VITESSE VERTICALE uniquement si les deux altitudes sont valides
            if (altitude !== null && dAlt !== null) { 
                speedMS_Vert = (altitude - dAlt) / dt;
            }
        }
    }
    
    // 4. VITESSE 3D (Vitesse Horizontale² + Vitesse Verticale²)
    const speedMS_3D = Math.sqrt(speedMS_Horiz * speedMS_Horiz + speedMS_Vert * speedMS_Vert);
    
    // Vitesse moyenne totale (basée sur la distance horizontale cumulée)
    const speedAvgMS = elapsedTimeS > 0 ? totalDistanceM / elapsedTimeS : 0;
    
    // Mise à jour de la vitesse max
    if (speedMS_3D > maxSpeedMS) { maxSpeedMS = speedMS_3D; }

    // --- CALCULS RELATIVISTES ET COSMOS (Basé sur speedMS_3D) ---
    const percLight = (speedMS_3D / C_LIGHT) * 100;
    const percSound = (speedMS_3D / C_SOUND_SEA_LEVEL) * 100;
    const distCosmicSL = totalDistanceM / C_LIGHT; 
    const distCosmicAL = totalDistanceM * M_TO_AL; 
    
    // --- AFFICHAGE TEMPS/VITESSE ---
    document.getElementById('elapsed-time').textContent = `${elapsedTimeS.toFixed(1)} s`;
    document.getElementById('speed-3d-inst').textContent = `${(speedMS_3D * KMH_PER_MS).toFixed(4)} km/h`;
    document.getElementById('speed-avg').textContent = `${(speedAvgMS * KMH_PER_MS).toFixed(4)} km/h`;
    document.getElementById('speed-max').textContent = `${(maxSpeedMS * KMH_PER_MS).toFixed(4)} km/h`;
    document.getElementById('speed-ms').textContent = `${speedMS_3D.toFixed(2)} m/s`;
    document.getElementById('speed-mms').textContent = `${(speedMS_3D * 1000).toFixed(0)} mm/s`;

    document.getElementById('perc-light').textContent = `${percLight.toExponential(2)}%`;
    document.getElementById('perc-sound').textContent = `${percSound.toFixed(1)}%`;
    
    document.getElementById('distance-km-m').textContent = `${(totalDistanceM / 1000).toFixed(2)} km | ${(totalDistanceM % 1000).toFixed(1)} m`;
    document.getElementById('distance-mm').textContent = `${(totalDistanceM * 1000).toFixed(0)} mm`;
    document.getElementById('distance-cosmic-s').textContent = `${distCosmicSL.toExponential(2)} s lumière`;
    document.getElementById('distance-cosmic-al').textContent = `${distCosmicAL.toExponential(2)} al`;
    
    // --- AFFICHAGE BRUT GPS / CAPTEURS ---
    document.getElementById('latitude').textContent = `${latitude.toFixed(6)}`;
    document.getElementById('longitude').textContent = `${longitude.toFixed(6)}`;
    
    // Affichage Altitude/Précision : affiche '--' si les données sont null
    document.getElementById('altitude').textContent = altitude !== null ? `${altitude.toFixed(0)} m` : '--';
    document.getElementById('gps-accuracy').textContent = accuracy !== null ? `${accuracy.toFixed(0)} m` : '--';
    document.getElementById('underground').textContent = altitude !== null ? (altitude < 0 ? 'Oui' : 'Non') : '--';
    
    const hour = new Date().getHours();
    const isDaylight = hour > 6 && hour < 20;
    document.getElementById('light-sim').textContent = isDaylight ? 'Jour' : 'Nuit';
    document.getElementById('light-level').textContent = isDaylight ? '≈ 5000 lux' : '≈ 0 lux';

    document.getElementById('heading').textContent = heading !== null ? `${heading.toFixed(0)}°` : '--°';


    // --- BOUSSOLE / NAVIGATION ---
    if (targetLat !== null && targetLon !== null) {
        const bearing = calculateBearing(latitude, longitude, targetLat, targetLon);
        document.getElementById('cap-dest').textContent = `${bearing.toFixed(0)}°`;
        document.getElementById('target-lat').textContent = `${targetLat.toFixed(6)}`;
        document.getElementById('target-lon').textContent = `${targetLon.toFixed(6)}`;
    }

    // --- ASTRO ET TEMPS MINECRAFT ---
    calculateAstroData(latitude, longitude);

    lastPosition = position;
}

// ===========================================
// 6. INITIALISATION (Inchangée)
// ===========================================

function setTargetDestination() {
    if (!lastPosition) {
        alert("Veuillez démarrer le GPS et attendre une position avant de définir une cible.");
        return;
    }

    const currentLat = lastPosition.coords.latitude.toFixed(6);
    const currentLon = lastPosition.coords.longitude.toFixed(6);
    
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
        alert("Coordonnées invalides. Réinitialisation.");
        targetLat = null;
        targetLon = null;
        setTargetBtn.textContent = '📍 Aller';
        document.getElementById('cap-dest').textContent = '--°';
    }
}

function initApp() {
    startBtn.addEventListener('click', startGPS);
    stopBtn.addEventListener('click', () => stopGPS(true)); 
    resetMaxBtn.addEventListener('click', resetMaxSpeed);
    setTargetBtn.addEventListener('click', setTargetDestination);
    
    resetDisplay();
}

document.addEventListener('DOMContentLoaded', initApp);
