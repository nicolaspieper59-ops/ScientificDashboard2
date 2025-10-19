// ===========================================
// 1. CONSTANTES GLOBALES ET PHYSIQUES
// ===========================================

// --- PHYSIQUE ---
const C_LIGHT = 299792458; // Vitesse de la lumière en m/s
const C_LIGHT_SQ = C_LIGHT * C_LIGHT; // c²
const EARTH_RADIUS = 6371000; // Rayon de la Terre en mètres (pour les calculs d'altitude)
const AU = 149597870700; // Unité Astronomique en mètres
const KMH_PER_MS = 3.6;     // Conversion m/s en km/h
const G_ACCEL = 9.80665; // Accélération standard de la gravité (m/s²)

// --- GÉOLOCALISATION / AFFICHAGE ---
const WATCH_OPTIONS = {
    enableHighAccuracy: true,
    maximumAge: 0,
    timeout: 5000 
};
let lastPosition = null;
let totalDistanceM = 0;
let watchID = null; // ID de suivi pour arrêter le GPS

// Références aux boutons
const startBtn = document.getElementById('start-btn');
const stopBtn = document.getElementById('stop-btn');
const statusDisplay = document.getElementById('gps-status');
const errorDisplay = document.getElementById('error-message');


// ===========================================
// 2. FONCTIONS DE CALCUL ET D'AFFICHAGE
// ===========================================

/** Réinitialise tous les champs d'affichage à leur valeur par défaut ('--'). */
function resetDisplay() {
    // Réinitialisation des variables de suivi
    lastPosition = null;
    totalDistanceM = 0;
    
    // Réinitialisation des affichages
    const ids = ['latitude', 'longitude', 'altitude', 'speed-horiz', 'speed-3d', 
                 'distance-total', 'lorentz-factor', 'light-perc', 
                 'time-accel-v', 'time-accel-h', 'time-accel-net', 
                 'hsm', 'dist-soleil', 'dist-lune', 
                 'lever-soleil', 'coucher-soleil', 'lunar-phase', 
                 'air-temp', 'wind-speed', 'wind-dir'];

    ids.forEach(id => {
        const element = document.getElementById(id);
        if (element) {
            element.textContent = '--';
        }
    });

    // Cas spéciaux
    document.getElementById('lorentz-factor').textContent = '1.0000000000';
    document.getElementById('light-perc').textContent = '0.00e-00%';
    document.getElementById('time-accel-v').textContent = '0 ns/s';
    document.getElementById('time-accel-h').textContent = '0 ns/s';
    document.getElementById('time-accel-net').textContent = '0 ns/s';

    statusDisplay.textContent = 'Inactif';
    errorDisplay.style.display = 'none';
}


/** Calcule la distance entre deux points GPS (formule de Haversine). */
function calculateDistance(lat1, lon1, lat2, lon2) {
    const R = EARTH_RADIUS; // Rayon de la Terre en mètres
    const dLat = (lat2 - lat1) * (Math.PI / 180);
    const dLon = (lon2 - lon1) * (Math.PI / 180);
    const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
              Math.cos(lat1 * (Math.PI / 180)) * Math.cos(lat2 * (Math.PI / 180)) *
              Math.sin(dLon / 2) * Math.sin(dLon / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R * c; // Distance en mètres
}

/** Calcule les effets relativistes et l'accélération du temps (4D). */
function calculate4DData(speedMS_3D, altitudeM) {
    const v = speedMS_3D;
    const h = altitudeM;

    // 1. Dilatation du Temps par la Vitesse (Relativité Restreinte)
    const v2 = v * v;
    let gamma = 1;
    let timeAccelV = 0; 

    if (v2 < C_LIGHT_SQ) {
        gamma = 1 / Math.sqrt(1 - (v2 / C_LIGHT_SQ));
        timeAccelV = (1 - gamma) * 1e9; // ns/s (négatif = ralentissement)
    }

    // 2. Dilatation du Temps par la Gravité (Relativité Générale)
    const timeAccelH = (G_ACCEL * h / C_LIGHT_SQ) * 1e9; // ns/s (positif = accélération)

    // 3. Bilan Net (Vitesse + Gravité)
    const timeAccelNet = timeAccelV + timeAccelH;

    return {
        gamma,
        timeAccelV: timeAccelV.toFixed(5),
        timeAccelH: timeAccelH.toFixed(5),
        timeAccelNet: timeAccelNet.toFixed(5)
    };
}

/** Calcule et affiche les données astronomiques (simplifié). */
function calculateAstroData(latitude, longitude) {
    const now = new Date();
    const date = now.getDate();
    const month = now.getMonth() + 1; 
    const year = now.getFullYear();
    const hour = now.getUTCHours();
    const minute = now.getUTCMinutes();
    const second = now.getUTCSeconds();

    // --- HEURE ---
    document.getElementById('hsm').textContent = `${String(hour).padStart(2, '0')}:${String(minute).padStart(2, '0')}:${String(second).padStart(2, '0')} UTC`;

    // --- CALCULS ASTRO SIMPLIFIÉS ---
    const dayOfYear = Math.floor((now - new Date(year, 0, 0)) / (1000 * 60 * 60 * 24));
    const distAU = 1.0000004209 + 0.0167086 * Math.cos(0.0172021 * dayOfYear - 0.005406); 
    document.getElementById('dist-soleil').textContent = `${distAU.toFixed(5)} UA`;
    
    // Distance Terre-Lune (approximation moyenne)
    const distLuneKm = 384400 + Math.sin(now.getTime() / 2360592000) * 20000;
    document.getElementById('dist-lune').textContent = `${(distLuneKm).toFixed(2)} km`; // Affiche en km pour la Lune

    // Lever/Coucher du Soleil (Approximation simple)
    const sunRiseHour = (12 - Math.floor(longitude / 15)) % 24;
    document.getElementById('lever-soleil').textContent = `${String(sunRiseHour).padStart(2, '0')}:30`;
    document.getElementById('coucher-soleil').textContent = `${String((sunRiseHour + 12) % 24).padStart(2, '0')}:30`;

    // Phase Lunaire (Approximation simple)
    const moonPhaseIndex = Math.floor((now.getTime() / 86400000) % 29.53);
    const phases = ["Nouvelle Lune", "Croissant", "Premier Quartier", "Gibbeuse Ascendante", "Pleine Lune", "Gibbeuse Descendante", "Dernier Quartier", "Vieux Croissant"];
    const phase = phases[Math.floor(moonPhaseIndex / (29.53 / phases.length)) % phases.length];
    document.getElementById('lunar-phase').textContent = phase;
}


/** Met à jour tous les affichages du tableau de bord. */
function updateDisplay(position) {
    statusDisplay.textContent = 'Actif (Mise à jour...)';
    errorDisplay.style.display = 'none';

    const { latitude, longitude, altitude, speed, timestamp } = position.coords;
    const currentTime = timestamp;
    
    // 1. CALCUL DE LA VITESSE 3D ET DISTANCE TOTALE
    let speedMS_Horiz_Calc = speed || 0; 
    let speedMS_Vert_Calc = 0;           

    if (lastPosition) {
        const dLat = lastPosition.coords.latitude;
        const dLon = lastPosition.coords.longitude;
        const dAlt = lastPosition.coords.altitude;
        const dt = (currentTime - lastPosition.timestamp) / 1000; 

        const distHorizM = calculateDistance(dLat, dLon, latitude, longitude);
        totalDistanceM += distHorizM;

        if (!speed) {
            speedMS_Horiz_Calc = distHorizM / dt;
        }

        if (altitude !== null && dAlt !== null && dt > 0) {
            speedMS_Vert_Calc = (altitude - dAlt) / dt;
        }
    }
    
    const speedMS_3D_Reconstituée = Math.sqrt(speedMS_Horiz_Calc * speedMS_Horiz_Calc + speedMS_Vert_Calc * speedMS_Vert_Calc);


    // 2. CALCULS RELATIVISTES (4D)
    const data4D = calculate4DData(speedMS_3D_Reconstituée, altitude || 0);
    const speedPercLight = (speedMS_3D_Reconstituée / C_LIGHT) * 100;


    // 3. AFFICHAGE DES DONNÉES EN TEMPS RÉEL (VITESSE/DISTANCE/TEMPS)
    document.getElementById('latitude').textContent = `${latitude.toFixed(6)} °`;
    document.getElementById('longitude').textContent = `${longitude.toFixed(6)} °`;
    document.getElementById('altitude').textContent = altitude !== null ? `${(altitude).toFixed(1)} m` : '--';
    
    document.getElementById('speed-horiz').textContent = `${(speedMS_Horiz_Calc * KMH_PER_MS).toFixed(2)} km/h`;
    document.getElementById('speed-3d').textContent = `${(speedMS_3D_Reconstituée * KMH_PER_MS).toFixed(2)} km/h`;
    document.getElementById('distance-total').textContent = `${(totalDistanceM / 1000).toFixed(3)} km`;


    // 4. AFFICHAGE DES DONNÉES RELATIVISTES (4D)
    document.getElementById('lorentz-factor').textContent = `${data4D.gamma.toFixed(10)}`;
    document.getElementById('light-perc').textContent = `${speedPercLight.toExponential(3)}%`;
    document.getElementById('time-accel-v').textContent = `${data4D.timeAccelV} ns/s (Ralentissement)`;
    document.getElementById('time-accel-h').textContent = `${data4D.timeAccelH} ns/s (Accélération)`;
    document.getElementById('time-accel-net').textContent = `${data4D.timeAccelNet} ns/s`;


    // 5. AFFICHAGE DES DONNÉES COSMOGRAPHIQUES
    calculateAstroData(latitude, longitude);


    // 6. AFFICHAGE DES DONNÉES MÉTÉO/SCIENCE RÉELLE (DÉSACTIVÉ)
    document.getElementById('air-temp').textContent = '-- °C';
    document.getElementById('wind-speed').textContent = '-- km/h';
    document.getElementById('wind-dir').textContent = '--';
    
    // Mise à jour pour la prochaine itération
    lastPosition = position;
}

// ===========================================
// 3. GESTION DES BOUTONS ET DU FLUX GPS
// ===========================================

/** Gère les erreurs de géolocalisation. */
function handleGeolocationError(error) {
    stopGPS(); // Arrête le suivi en cas d'erreur
    errorDisplay.style.display = 'block';
    statusDisplay.textContent = 'Erreur';

    switch (error.code) {
        case error.PERMISSION_DENIED:
            errorDisplay.textContent = "Erreur GPS : L'accès à la géolocalisation a été refusé. Le GPS est obligatoire.";
            break;
        case error.POSITION_UNAVAILABLE:
            errorDisplay.textContent = "Erreur GPS : Position non disponible (Signal faible).";
            break;
        case error.TIMEOUT:
            errorDisplay.textContent = "Erreur GPS : Délai d'attente dépassé. Connexion lente.";
            break;
        case error.UNKNOWN_ERROR:
            errorDisplay.textContent = "Erreur GPS : Une erreur inconnue est survenue.";
            break;
    }
}

/** Démarre le suivi GPS. */
function startGPS() {
    if (!navigator.geolocation) {
        errorDisplay.style.display = 'block';
        errorDisplay.textContent = "Erreur : Votre navigateur ne supporte pas la géolocalisation.";
        return;
    }

    // Réinitialisation de l'affichage avant le démarrage
    resetDisplay(); 
    statusDisplay.textContent = 'Localisation en cours...';

    // Démarrage du suivi et stockage de l'ID
    watchID = navigator.geolocation.watchPosition(updateDisplay, handleGeolocationError, WATCH_OPTIONS);

    // Mise à jour des boutons
    startBtn.disabled = true;
    stopBtn.disabled = false;
}

/** Arrête le suivi GPS et réinitialise les données. */
function stopGPS() {
    if (watchID !== null) {
        navigator.geolocation.clearWatch(watchID);
        watchID = null;
    }
    
    resetDisplay();
    
    // Mise à jour des boutons
    startBtn.disabled = false;
    stopBtn.disabled = true;
}

/** Initialisation de l'application (écoute des événements). */
function initApp() {
    startBtn.addEventListener('click', startGPS);
    stopBtn.addEventListener('click', stopGPS);
    
    // Affichage initial
    resetDisplay();
}

// Démarrage de l'application une fois que le document est complètement chargé
document.addEventListener('DOMContentLoaded', initApp);
