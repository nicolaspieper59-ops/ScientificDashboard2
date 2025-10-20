// ===========================================
// 1. CONSTANTES ET VARIABLES GLOBALES
// (Inchangé)
// ===========================================

// --- PHYSIQUE / VITESSES ---
const C_LIGHT = 299792458;          
const C_SOUND_SEA_LEVEL = 343;      
const EARTH_RADIUS = 6371000;       
const KMH_PER_MS = 3.6;             
const DEG_TO_RAD = Math.PI / 180;
const RAD_TO_DEG = 180 / Math.PI;

// --- CONSTANTES ASTRO ---
const OBLIQUITY = 23.44 * DEG_TO_RAD; 
const ECCENTRICITY = 0.0167;        

// --- ÉTAT DE L'APPLICATION ---
const WATCH_OPTIONS = {
    enableHighAccuracy: true,
    maximumAge: 500,
    timeout: 15000 
};
let watchID = null;         
let lastPosition = null;
let startTime = null;
let totalDistanceM = 0; 
let maxSpeedMS = 0;
let targetLat = null;
let targetLon = null;

// --- FILTRAGE GPS ---
const MIN_TIME_INTERVAL_S = 1; 
const MAX_ACCURACY_M = 50;     

// --- SYNCHRONISATION TEMPORELLE (NTP Web) ---
let timeOffsetMS = 0; 

// --- REFERENCES DOM ---
const startBtn = document.getElementById('start-btn');
const stopBtn = document.getElementById('stop-btn');
const resetMaxBtn = document.getElementById('reset-max-btn');
const setTargetBtn = document.getElementById('set-target-btn');
const errorDisplay = document.getElementById('error-message');
const modeIndicator = document.getElementById('mode-indicator');
// NOUVEAU CHAMP DANS LE DOM
const speedSourceIndicator = document.getElementById('speed-source-indicator'); 


// ===========================================
// 2. SYNCHRONISATION DU TEMPS
// (Inchangé)
// ===========================================

async function synchronizeTime() {
    const timeServerUrl = 'https://worldtimeapi.org/api/ip'; 
    try {
        const response = await fetch(timeServerUrl);
        const T4 = Date.now(); 
        const data = await response.json();
        const serverTimeUTC = data.utc_datetime;
        const serverTimeMS = new Date(serverTimeUTC).getTime();
        timeOffsetMS = serverTimeMS - T4;
    } catch (e) {
        timeOffsetMS = 0; 
    }
}

function getCorrectedDate() {
    return new Date(Date.now() + timeOffsetMS);
}


// ===========================================
// 3. GESTION DES BOUTONS ET DU DÉMARRAGE
// (Inchangé, sauf pour le reset d'un champ)
// ===========================================

function resetDisplay() {
    lastPosition = null;
    totalDistanceM = 0; 
    startTime = null;
    maxSpeedMS = 0;
    targetLat = null; 
    targetLon = null;
    
    // Réinitialisation simplifiée de l'affichage
    const defaultText = '--';
    const ids = ['elapsed-time', 'speed-3d-inst', 'speed-avg', 'speed-max', 'speed-ms', 
        'perc-light', 'perc-sound', 'distance-km-m', 'lunar-time',
        'latitude', 'longitude', 'altitude', 'gps-accuracy', 'underground',
        'solar-true', 'solar-mean', 'eot', 'solar-longitude-val', 
        'lunar-phase-perc', 'mc-time', 'air-temp', 'pressure', 'humidity', 'wind-speed', 
        'boiling-point', 'heading', 'bubble-level', 'cap-dest', 'solar-true-header', 'mode-indicator', 'speed-source-indicator'
    ];

    ids.forEach(id => {
        const element = document.getElementById(id);
        if (element) {
            if (id === 'speed-source-indicator') element.textContent = 'Source: N/A';
            else if (['air-temp', 'pressure', 'humidity', 'wind-speed', 'boiling-point', 'bubble-level'].includes(id)) {
                element.textContent = 'N/A (API désactivée)'; 
            }
            else if (id === 'mc-time' || id === 'lunar-time') element.textContent = '00:00:00';
            else if (id === 'altitude' || id === 'gps-accuracy') element.textContent = '-- m';
            else if (id === 'distance-km-m') element.textContent = '-- km | -- m';
            else if (id === 'mode-indicator') element.textContent = 'Mode: Jour ☀️';
            else element.textContent = defaultText;
        }
    });

    startBtn.disabled = false;
    stopBtn.disabled = true;
    resetMaxBtn.disabled = true;
    setTargetBtn.textContent = '📍 Aller';
    errorDisplay.style.display = 'none';
    document.body.classList.remove('night-mode');
}

function handleGeolocationError(error) {
    errorDisplay.style.display = 'block';
    switch (error.code) {
        case error.PERMISSION_DENIED: errorDisplay.textContent = "❌ L'accès à la localisation a été refusé."; break;
        case error.POSITION_UNAVAILABLE: errorDisplay.textContent = "⚠️ Position non disponible. Signal GPS faible."; break;
        case error.TIMEOUT: errorDisplay.textContent = "⏱️ Délai de recherche du GPS dépassé. Signal faible."; break;
        default: errorDisplay.textContent = "❓ Erreur GPS inconnue."; break;
    }
    stopGPS(false); 
}

function startGPS() {
    if (!navigator.geolocation) {
        errorDisplay.style.display = 'block';
        errorDisplay.textContent = "❌ Votre navigateur ne supporte pas la géolocalisation.";
        return;
    }

    resetDisplay(); 
    startTime = Date.now(); 
    
    watchID = navigator.geolocation.watchPosition(updateDisplay, handleGeolocationError, WATCH_OPTIONS);
    
    startBtn.disabled = true;
    stopBtn.disabled = false;
    resetMaxBtn.disabled = false;
    errorDisplay.style.display = 'none';
    
    synchronizeTime(); 
}

function stopGPS(shouldReset = true) {
    if (watchID !== null) {
        navigator.geolocation.clearWatch(watchID);
        watchID = null;
    }
    
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
// 4. FONCTIONS DE CALCULS ASTRO
// (Inchangé)
// ===========================================

function calculateDistance(lat1, lon1, lat2, lon2) {
    const R = EARTH_RADIUS;
    const dLat = (lat2 - lat1) * DEG_TO_RAD;
    const dLon = (lon2 - lon1) * DEG_TO_RAD;
    lat1 *= DEG_TO_RAD;
    lat2 *= DEG_TO_RAD;

    const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
              Math.cos(lat1) * Math.cos(lat2) *
              Math.sin(dLon / 2) * Math.sin(dLon / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R * c;
}

function calculateBearing(lat1, lon1, lat2, lon2) {
    const R = DEG_TO_RAD;
    lat1 *= R; lon1 *= R; lat2 *= R; lon2 *= R;

    const y = Math.sin(lon2 - lon1) * Math.cos(lat2);
    const x = Math.cos(lat1) * Math.sin(lat2) -
              Math.sin(lat1) * Math.cos(lat2) * Math.cos(lon2 - lon1);
    
    let bearing = Math.atan2(y, x) * RAD_TO_DEG;
    return (bearing + 360) % 360; 
}

function calculateSolarDetails() {
    const now = getCorrectedDate();
    const J2000 = new Date(Date.UTC(2000, 0, 1, 12, 0, 0));
    const D = (now.getTime() - J2000.getTime()) / (1000 * 60 * 60 * 24);

    const M = (357.529 + 0.98560028 * D) * DEG_TO_RAD;
    const L = (280.466 + 0.98564736 * D) * DEG_TO_RAD;
    
    const C_e = 2 * ECCENTRICITY * Math.sin(M) + 1.25 * ECCENTRICITY * ECCENTRICITY * Math.sin(2 * M);
    const lambda = L + C_e; 

    const alpha = Math.atan2(Math.cos(OBLIQUITY) * Math.sin(lambda), Math.cos(lambda));
    
    const EoT_rad = L - alpha;
    const EoT_minutes = (EoT_rad * RAD_TO_DEG) * 4;
    
    return {
        eot: EoT_minutes,
        solarLongitude: (lambda * RAD_TO_DEG) % 360,
    };
}

function calculateLunarTime(longitude) {
    const now = getCorrectedDate();
    const JD = now.getTime() / 86400000 + 2440587.5; 

    const T = (JD - 2451545.0) / 36525.0; 
    let GST = 280.4606 + 360.9856473 * (JD - 2451545.0) + 0.000388 * T * T;
    GST = GST % 360;
    if (GST < 0) GST += 360;

    let L_moon = 218.316 + 488204.661 * T; 
    L_moon = L_moon % 360;
    if (L_moon < 0) L_moon += 360;
    
    let HA_moon = GST + longitude - L_moon;
    HA_moon = HA_moon % 360;
    if (HA_moon < 0) HA_moon += 360;

    const LMT_hours = HA_moon / 15.0; 
    
    const LMT_total_seconds = LMT_hours * 3600;
    const lmtHour = Math.floor(LMT_total_seconds / 3600);
    const lmtMinute = Math.floor((LMT_total_seconds % 3600) / 60);
    const lmtSecond = Math.floor(LMT_total_seconds % 60);
    
    document.getElementById('lunar-time').textContent = 
        `${String(lmtHour).padStart(2, '0')}:${String(lmtMinute).padStart(2, '0')}:${String(lmtSecond).padStart(2, '0')}`;
}

function updateAstroDisplay(latitude, longitude) {
    const now = getCorrectedDate();
    
    // --- TEMPS MINECRAFT ---
    const mcTicksPerDay = 24000;
    const msSinceMidnight = now.getTime() % 86400000;
    const mcTicks = (msSinceMidnight * mcTicksPerDay) / 86400000;
    const mcMinutes = Math.floor(mcTicks / 20) % 1440; 
    const mcHour = Math.floor(mcMinutes / 60);
    const mcMinute = mcMinutes % 60;
    const mcSecond = Math.floor((mcTicks % 20) / 20 * 60);
    document.getElementById('mc-time').textContent = `${String(mcHour).padStart(2, '0')}:${String(mcMinute).padStart(2, '0')}:${String(mcSecond).padStart(2, '0')}`;


    // --- TEMPS SOLAIRE MOYEN (HSM) & HSV ---
    const totalMinutesUT = now.getUTCHours() * 60 + now.getUTCMinutes() + now.getUTCSeconds() / 60;
    const totalMinutesLSM = (totalMinutesUT + (longitude * 4) + 1440) % 1440; 
    
    const hsmHour = Math.floor(totalMinutesLSM / 60);
    const hsmMinute = Math.floor(totalMinutesLSM % 60);
    document.getElementById('solar-mean').textContent = `${String(hsmHour).padStart(2, '0')}:${String(hsmMinute).padStart(2, '0')}`;
    
    const solarDetails = calculateSolarDetails();
    const EoT_ms = solarDetails.eot * 60 * 1000; 
    
    const nowHSM = new Date(now.getTime() - (now.getTime() % 86400000) + totalMinutesLSM * 60 * 1000);
    const nowHSV = new Date(nowHSM.getTime() + EoT_ms);

    const lsvHour = nowHSV.getUTCHours(); 
    const lsvMinute = nowHSV.getUTCMinutes();
    const lsvSecond = Math.floor(nowHSV.getUTCSeconds());
    
    const hsvTimeStr = `${String(lsvHour).padStart(2, '0')}:${String(lsvMinute).padStart(2, '0')}:${String(lsvSecond).padStart(2, '0')}`;

    document.getElementById('solar-true').textContent = `${hsvTimeStr} (HSV)`;
    document.getElementById('solar-true-header').textContent = hsvTimeStr;

    document.getElementById('eot').textContent = `${solarDetails.eot.toFixed(2)} min`;
    document.getElementById('solar-longitude-val').textContent = `${solarDetails.solarLongitude.toFixed(2)} °`;


    // --- PHASE LUNAIRE (Illumination) ---
    const JD = now.getTime() / 86400000 + 2440587.5; 
    const N = (JD - 2451549.5) / 365.25; 
    const D_moon = (297.85 + 445267.111 * N) % 360 * DEG_TO_RAD; 
    
    const illumination = 0.5 * (1 - Math.cos(D_moon)); 
    document.getElementById('lunar-phase-perc').textContent = `${(illumination * 100).toFixed(1)}%`;

    // --- TEMPS LUNAIRE ---
    calculateLunarTime(longitude); 
}


// ===========================================
// 5. MODE JOUR/NUIT & CIBLE
// (Inchangé)
// ===========================================

function updateDarkMode(latitude, longitude) {
    const now = getCorrectedDate();
    const localHourApprox = now.getUTCHours() + now.getUTCMinutes() / 60 + longitude / 15;
    const hours = localHourApprox % 24;
    
    const SUNRISE_H = 6;
    const SUNSET_H = 18;

    const isNight = (hours < SUNRISE_H || hours >= SUNSET_H);

    if (isNight) {
        document.body.classList.add('night-mode');
        modeIndicator.textContent = 'Mode: Nuit 🌑';
    } else {
        document.body.classList.remove('night-mode');
        modeIndicator.textContent = 'Mode: Jour ☀️';
    }
}

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
        setTargetBtn.textContent = '✅ Cible définie';
    } else {
        alert("Coordonnées invalides. Réinitialisation.");
        targetLat = null;
        targetLon = null;
        setTargetBtn.textContent = '📍 Aller';
        document.getElementById('cap-dest').textContent = 'N/A';
    }
}


// ===========================================
// 6. FONCTION PRINCIPALE D'AFFICHAGE (MODIFIÉE)
// ===========================================

function updateDisplay(position) {
    const latitude = position.coords.latitude;
    const longitude = position.coords.longitude;
    const altitude = position.coords.altitude; 
    const accuracy = position.coords.accuracy;
    const heading = position.coords.heading;   
    const speed = position.coords.speed;       
    const currentTime = position.timestamp;
    
    // --- CONTRÔLE DE LA PRÉCISION ---
    if (accuracy > MAX_ACCURACY_M) {
        document.getElementById('gps-accuracy').textContent = `🚨 ${accuracy.toFixed(0)} m (Trop Imprécis)`;
        if (lastPosition === null) { lastPosition = position; }
        updateAstroDisplay(latitude, longitude);
        updateDarkMode(latitude, longitude);
        return; 
    }

    // --- LOGIQUE DE VITESSE ET SOURCE ---
    let speedMS_Horiz = 0; 
    let speedMS_Vert = 0;
    let speedSource = 'Calculée (Filtrée)'; // Hypothèse par défaut
    
    if (speed !== null && speed !== undefined) {
        // Le navigateur fournit la vitesse directement (probablement Doppler / Brute)
        speedMS_Horiz = speed;
        speedSource = 'Puce GPS (Brute/Doppler)';
    }

    if (lastPosition && lastPosition.coords.latitude !== undefined) { 
        const dLat = lastPosition.coords.latitude;
        const dLon = lastPosition.coords.longitude;
        const dAlt = lastPosition.coords.altitude; 
        const dt = (currentTime - lastPosition.timestamp) / 1000; 

        if (dt > MIN_TIME_INTERVAL_S) { 
            const distHorizM = calculateDistance(dLat, dLon, latitude, longitude);
            let distVertM = (altitude !== null && dAlt !== null) ? altitude - dAlt : 0;

            const dist3D = Math.sqrt(distHorizM * distHorizM + distVertM * distVertM);
            totalDistanceM += dist3D; 

            // Si le navigateur NE FOURNIT PAS la vitesse (speed === null), nous la calculons ici
            if (speed === null || speed === undefined) { 
                speedMS_Horiz = distHorizM / dt; 
                // Mise à jour de la source pour refléter le calcul
                speedSource = 'Calculée (Dérivée)';
            }
            if (altitude !== null && dAlt !== null) { speedMS_Vert = distVertM / dt; }
        } else {
             // Si l'intervalle est trop court ou la position est la première
             if (speed === null || speed === undefined) { 
                 speedMS_Horiz = 0; 
                 speedSource = 'Calculée (Statique)';
             }
             speedMS_Vert = 0;
        }
    }
    
    const elapsedTimeS = (currentTime - startTime) / 1000;
    const speedMS_3D = Math.sqrt(speedMS_Horiz * speedMS_Horiz + speedMS_Vert * speedMS_Vert);
    const speedAvgMS = elapsedTimeS > MIN_TIME_INTERVAL_S ? totalDistanceM / elapsedTimeS : 0; 
    if (speedMS_3D > maxSpeedMS) { maxSpeedMS = speedMS_3D; }

    const totalDistKm = totalDistanceM / 1000;
    const totalDistMeters = totalDistanceM % 1000;

    // MISE À JOUR DES VALEURS
    document.getElementById('elapsed-time').textContent = `${elapsedTimeS.toFixed(1)} s`;
    document.getElementById('speed-3d-inst').textContent = `${(speedMS_3D * KMH_PER_MS).toFixed(3)} km/h`;
    document.getElementById('distance-km-m').textContent = `${totalDistKm.toFixed(2)} km | ${totalDistMeters.toFixed(3)} m`;
    document.getElementById('speed-avg').textContent = `${(speedAvgMS * KMH_PER_MS).toFixed(3)} km/h`; 
    document.getElementById('speed-max').textContent = `${(maxSpeedMS * KMH_PER_MS).toFixed(3)} km/h`;
    document.getElementById('speed-ms').textContent = `${speedMS_3D.toFixed(4)} m/s`;
    
    document.getElementById('perc-light').textContent = `${((speedMS_3D / C_LIGHT) * 100).toExponential(2)}%`;
    document.getElementById('perc-sound').textContent = `${((speedMS_3D / C_SOUND_SEA_LEVEL) * 100).toFixed(1)}%`;

    document.getElementById('latitude').textContent = `${latitude.toFixed(6)}`;
    document.getElementById('longitude').textContent = `${longitude.toFixed(6)}`;
    document.getElementById('altitude').textContent = altitude !== null ? `${altitude.toFixed(2)} m` : 'N/A';
    document.getElementById('gps-accuracy').textContent = `${accuracy.toFixed(2)} m`;
    document.getElementById('underground').textContent = altitude !== null ? (altitude < 0 ? 'Oui' : 'Non') : 'N/A';
    document.getElementById('heading').textContent = heading !== null ? `${heading.toFixed(0)}°` : 'N/A';
    
    // NOUVEL AFFICHAGE DE LA SOURCE DE VITESSE
    speedSourceIndicator.textContent = `Source: ${speedSource}`;

    if (targetLat !== null && targetLon !== null) {
        const bearing = calculateBearing(latitude, longitude, targetLat, targetLon);
        document.getElementById('cap-dest').textContent = `${bearing.toFixed(0)}°`;
    } else {
        document.getElementById('cap-dest').textContent = 'N/A';
    }

    // --- ASTRO et MODE NUIT/JOUR ---
    updateAstroDisplay(latitude, longitude);
    updateDarkMode(latitude, longitude); 
    
    lastPosition = position; 
}


// ===========================================
// 7. INITIALISATION
// (Inchangé)
// ===========================================

function initApp() {
    if (startBtn) startBtn.addEventListener('click', startGPS);
    if (stopBtn) stopBtn.addEventListener('click', () => stopGPS(true)); 
    if (resetMaxBtn) resetMaxBtn.addEventListener('click', resetMaxSpeed);
    if (setTargetBtn) setTargetBtn.addEventListener('click', setTargetDestination);

    synchronizeTime(); 
    setInterval(synchronizeTime, 300000); 
    
    resetDisplay();
    updateDarkMode(0, 0); 
}

document.addEventListener('DOMContentLoaded', initApp);
