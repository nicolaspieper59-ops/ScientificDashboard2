// ===========================================
// 1. CONSTANTES ET VARIABLES GLOBALES
// ===========================================

// --- PHYSIQUE / VITESSES ---
const C_LIGHT = 299792458;          // Vitesse de la lumière (m/s)
const C_SOUND_SEA_LEVEL = 343;      // Vitesse du son (m/s)
const EARTH_RADIUS = 6371000;       // Rayon de la Terre (m)
const KMH_PER_MS = 3.6;             // Conversion m/s -> km/h
const DEG_TO_RAD = Math.PI / 180;
const RAD_TO_DEG = 180 / Math.PI;

// --- CONSTANTES ASTRO ---
const OBLIQUITY = 23.44 * DEG_TO_RAD; // Obliquité de l'écliptique (en radians)
const ECCENTRICITY = 0.0167;        // Excentricité de l'orbite terrestre

// --- ÉTAT DE L'APPLICATION ---
const WATCH_OPTIONS = {
    enableHighAccuracy: true,
    maximumAge: 500,
    timeout: 15000 // Augmentation du timeout à 15s pour les zones difficiles
};
let watchID = null;         
let lastPosition = null;
let startTime = null;
let totalDistanceM = 0; 
let maxSpeedMS = 0;
let targetLat = null;
let targetLon = null;

// --- API MÉTÉO ---
const OPENWEATHER_API_KEY = 'YOUR_API_KEY'; 
let lastWeatherFetchTime = 0;
const WEATHER_FETCH_INTERVAL_MS = 60000; // 60 secondes

// --- FILTRAGE GPS (pour la stabilité de la Vitesse 3D) ---
const MIN_TIME_INTERVAL_S = 1; // Intervalle minimum pour le calcul de vitesse (1s)
const MAX_ACCURACY_M = 50;     // Précision maximale acceptable (ignorons les relevés > 50m d'erreur)

// --- SYNCHRONISATION TEMPORELLE (NTP Web) ---
let timeOffsetMS = 0; // Décalage de l'horloge locale par rapport au temps UTC réel

// --- REFERENCES DOM ---
const startBtn = document.getElementById('start-btn');
const stopBtn = document.getElementById('stop-btn');
const resetMaxBtn = document.getElementById('reset-max-btn');
const setTargetBtn = document.getElementById('set-target-btn');
const errorDisplay = document.getElementById('error-message');


// ===========================================
// 2. SYNCHRONISATION DU TEMPS (NTP Web Précis)
// ===========================================

/**
 * Synchronise l'horloge locale avec un serveur de temps via HTTP pour
 * calculer un décalage ultra-précis, en se basant sur l'heure de fin de requête (T4).
 * Ceci rend l'application INDÉPENDANTE de l'heure système manipulée par l'utilisateur.
 */
async function synchronizeTime() {
    const timeServerUrl = 'https://worldtimeapi.org/api/ip'; 
    
    try {
        const response = await fetch(timeServerUrl);
        
        // 1. Enregistre l'heure locale EXACTE après la réception de la réponse (T4)
        const T4 = Date.now(); 
        
        const data = await response.json();
        
        const serverTimeUTC = data.utc_datetime;
        const serverTimeMS = new Date(serverTimeUTC).getTime();
        
        // 2. Le décalage (offset) est calculé : (Temps du Serveur) - (Temps Local de Réception)
        timeOffsetMS = serverTimeMS - T4;

        console.log(`Synchronisation NTP web à T4 : Décalage: ${timeOffsetMS.toFixed(2)} ms.`);

    } catch (e) {
        console.warn("Erreur de synchronisation NTP web. timeOffsetMS réinitialisé à 0.", e);
        timeOffsetMS = 0; 
    }
}

/**
 * Retourne l'heure actuelle corrigée par l'offset NTP. C'EST LA SOURCE DE TEMPS DE L'APP.
 */
function getCorrectedDate() {
    return new Date(Date.now() + timeOffsetMS);
}


// ===========================================
// 3. GESTION DES BOUTONS ET DU DÉMARRAGE
// ===========================================

function resetDisplay() {
    lastPosition = null;
    totalDistanceM = 0; 
    startTime = null;
    maxSpeedMS = 0;
    targetLat = null; 
    targetLon = null;
    
    const ids = ['elapsed-time', 'speed-3d-inst', 'speed-avg', 'speed-max', 'speed-ms', 
        'perc-light', 'perc-sound', 'distance-km-m',
        'latitude', 'longitude', 'altitude', 'gps-accuracy', 'underground',
        'solar-true', 'solar-mean', 'eot', 'solar-longitude-val', 
        'lunar-phase-perc', 'mc-time', 'air-temp', 'pressure', 'humidity', 'wind-speed', 
        'boiling-point', 'heading', 'bubble-level', 'cap-dest', 'solar-true-header'
    ];

    ids.forEach(id => {
        const element = document.getElementById(id);
        if (element) {
            // Utilisation de N/A pour les valeurs potentiellement non disponibles
            if (['altitude', 'gps-accuracy'].includes(id)) element.textContent = '-- m';
            else if (['eot'].includes(id)) element.textContent = '-- min';
            else if (['solar-longitude-val'].includes(id)) element.textContent = '-- °';
            else if (['heading', 'cap-dest', 'bubble-level'].includes(id)) element.textContent = 'N/A'; 
            else if (['lunar-phase-perc', 'humidity', 'perc-light', 'perc-sound'].includes(id)) element.textContent = '--%';
            else if (id === 'air-temp' || id === 'boiling-point') element.textContent = 'N/A';
            else if (id === 'pressure') element.textContent = 'N/A';
            else if (id === 'wind-speed') element.textContent = 'N/A';
            else if (id === 'underground') element.textContent = 'N/A';
            else element.textContent = '--';
        }
    });

    document.getElementById('elapsed-time').textContent = '0.00 s';
    document.getElementById('mc-time').textContent = '00:00:00';
    document.getElementById('solar-true-header').textContent = '--:--:--';
    document.getElementById('distance-km-m').textContent = '-- km | -- m';
    
    startBtn.disabled = false;
    stopBtn.disabled = true;
    resetMaxBtn.disabled = true;
    setTargetBtn.textContent = '📍 Aller';
    errorDisplay.style.display = 'none';
}

function handleGeolocationError(error) {
    errorDisplay.style.display = 'block';

    switch (error.code) {
        case error.PERMISSION_DENIED:
            errorDisplay.textContent = "❌ L'accès à la localisation a été refusé. Veuillez l'autoriser.";
            break;
        case error.POSITION_UNAVAILABLE:
            errorDisplay.textContent = "⚠️ Position non disponible. Signal GPS faible.";
            break;
        case error.TIMEOUT:
            errorDisplay.textContent = "⏱️ Délai de recherche du GPS dépassé (15s). Le signal est faible ou perdu.";
            break;
        default:
            errorDisplay.textContent = "❓ Erreur GPS inconnue. Code: " + error.code;
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
    startTime = Date.now(); // Note : On utilise Date.now() pour le temps écoulé local
    
    watchID = navigator.geolocation.watchPosition(updateDisplay, handleGeolocationError, WATCH_OPTIONS);

    initSensorListeners(); 
    
    startBtn.disabled = true;
    stopBtn.disabled = false;
    resetMaxBtn.disabled = false;
    errorDisplay.style.display = 'none';
    
    // Assurer que la synchronisation est lancée au démarrage
    synchronizeTime(); 
}

function stopGPS(shouldReset = true) {
    if (watchID !== null) {
        navigator.geolocation.clearWatch(watchID);
        watchID = null;
    }
    stopSensorListeners();
    
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
// 4. FONCTIONS DE CALCUL (Distance, Astro)
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
    
    // Recalcul de l'heure locale corrigée pour l'HSM (Heure Solaire Moyenne)
    const nowHSM = new Date(now.getTime() - (now.getTime() % 86400000) + totalMinutesLSM * 60 * 1000);
    const nowHSV = new Date(nowHSM.getTime() + EoT_ms);

    const lsvHour = nowHSV.getUTCHours(); 
    const lsvMinute = nowHSV.getUTCMinutes();
    const lsvSecond = Math.floor(nowHSV.getUTCSeconds());
    
    const hsvTimeStr = `${String(lsvHour).padStart(2, '0')}:${String(lsvMinute).padStart(2, '0')}:${String(lsvSecond).padStart(2, '0')}`;

    document.getElementById('solar-true').textContent = `${hsvTimeStr} (HSV)`;
    const headerElement = document.getElementById('solar-true-header');
    if (headerElement) {
        headerElement.textContent = hsvTimeStr;
    }

    document.getElementById('eot').textContent = `${solarDetails.eot.toFixed(2)} min`;
    document.getElementById('solar-longitude-val').textContent = `${solarDetails.solarLongitude.toFixed(2)} °`;


    // --- PHASE LUNAIRE (Illumination) ---
    const JD = now.getTime() / 86400000 + 2440587.5; 
    const N = (JD - 2451549.5) / 365.25; 
    const D_moon = (297.85 + 445267.111 * N) % 360 * DEG_TO_RAD; 
    
    const illumination = 0.5 * (1 - Math.cos(D_moon)); 
    document.getElementById('lunar-phase-perc').textContent = `${(illumination * 100).toFixed(1)}%`;
}


async function fetchWeatherData(latitude, longitude) {
    if (OPENWEATHER_API_KEY === 'YOUR_API_KEY') {
        document.getElementById('air-temp').textContent = `N/A`;
        document.getElementById('humidity').textContent = `N/A`;
        document.getElementById('wind-speed').textContent = `N/A`;
        document.getElementById('pressure').textContent = `N/A`;
        document.getElementById('boiling-point').textContent = `N/A`;
        return;
    }

    const now = Date.now();
    if (now - lastWeatherFetchTime < WEATHER_FETCH_INTERVAL_MS) {
        return;
    }
    lastWeatherFetchTime = now;

    try {
        const url = `https://api.openweathermap.org/data/2.5/weather?lat=${latitude}&lon=${longitude}&appid=${OPENWEATHER_API_KEY}&units=metric&lang=fr`;
        const response = await fetch(url);
        const data = await response.json();
        
        if (data && data.main && data.wind) {
            const tempC = data.main.temp;
            const windKmH = data.wind.speed * KMH_PER_MS;
            const humidity = data.main.humidity;
            const pressure = data.main.pressure;
            
            document.getElementById('air-temp').textContent = `${tempC.toFixed(1)} °C`;
            document.getElementById('humidity').textContent = `${humidity.toFixed(0)}%`;
            document.getElementById('wind-speed').textContent = `${windKmH.toFixed(1)} km/h`;
            document.getElementById('pressure').textContent = `${pressure.toFixed(0)} hPa`; 
            
            const boilingPoint = 100 - (1013.25 - pressure) * 0.033;
            document.getElementById('boiling-point').textContent = `${boilingPoint.toFixed(1)} °C`;
        }
    } catch (e) {
        console.error("Erreur API météo:", e);
    }
}


// ===========================================
// 5. GESTION DES CAPTEURS 3D (Niveau à Bulle)
// ===========================================

function handleDeviceMotion(event) {
    const acc = event.accelerationIncludingGravity;
    
    if (!acc || acc.x === null || acc.y === null || acc.z === null) {
        document.getElementById('bubble-level').textContent = 'N/A';
        return;
    }

    const pitch = Math.atan2(acc.z, acc.y) * RAD_TO_DEG; 
    const roll = Math.atan2(acc.z, acc.x) * RAD_TO_DEG; 
    
    const pitchDeviation = Math.abs(pitch - 90);
    const rollDeviation = Math.abs(roll - 90);

    document.getElementById('bubble-level').textContent = 
        `P: ${pitchDeviation.toFixed(1)}° | R: ${rollDeviation.toFixed(1)}°`;
}

function initSensorListeners() {
    if (window.DeviceMotionEvent && typeof window.DeviceMotionEvent.requestPermission === 'function') {
        // Demande de permission pour iOS 13+
        DeviceMotionEvent.requestPermission()
            .then(response => {
                if (response === 'granted') {
                    window.addEventListener('devicemotion', handleDeviceMotion);
                } else {
                    document.getElementById('bubble-level').textContent = 'Perm. Refusée';
                }
            })
            .catch(e => {
                console.error("Erreur de permission devicemotion:", e);
                document.getElementById('bubble-level').textContent = 'Erreur';
            });
    } else if (window.DeviceMotionEvent) {
        // Anciens navigateurs ou Android
        window.addEventListener('devicemotion', handleDeviceMotion);
    } else {
        document.getElementById('bubble-level').textContent = 'N/A';
    }
}

function stopSensorListeners() {
    window.removeEventListener('devicemotion', handleDeviceMotion);
}


// ===========================================
// 6. FONCTION PRINCIPALE D'AFFICHAGE (Filtrage 3D)
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
        fetchWeatherData(latitude, longitude);
        return; 
    }

    const elapsedTimeS = (currentTime - startTime) / 1000;

    // --- CALCULS VITESSE/DISTANCE ---
    let speedMS_Horiz = speed !== null ? speed : 0; 
    let speedMS_Vert = 0;
    
    if (lastPosition && lastPosition.coords.latitude !== undefined) { 
        const dLat = lastPosition.coords.latitude;
        const dLon = lastPosition.coords.longitude;
        const dAlt = lastPosition.coords.altitude; 
        const dt = (currentTime - lastPosition.timestamp) / 1000; 

        if (dt > MIN_TIME_INTERVAL_S) { 
            const distHorizM = calculateDistance(dLat, dLon, latitude, longitude);
            
            let distVertM = 0;
            if (altitude !== null && dAlt !== null) {
                distVertM = altitude - dAlt;
            }

            const dist3D = Math.sqrt(distHorizM * distHorizM + distVertM * distVertM);
            totalDistanceM += dist3D; 

            if (speed === null) {
                 speedMS_Horiz = distHorizM / dt;
            }

            if (altitude !== null && dAlt !== null) { 
                speedMS_Vert = distVertM / dt;
            }
        } else {
             speedMS_Horiz = 0;
             speedMS_Vert = 0;
        }
    }
    
    const speedMS_3D = Math.sqrt(speedMS_Horiz * speedMS_Horiz + speedMS_Vert * speedMS_Vert);
    
    const speedAvgMS = elapsedTimeS > MIN_TIME_INTERVAL_S ? totalDistanceM / elapsedTimeS : 0; 
    if (speedMS_3D > maxSpeedMS) { maxSpeedMS = speedMS_3D; }

    // --- MISE À JOUR AFFICHAGE ---
    const totalDistKm = totalDistanceM / 1000;
    const totalDistMeters = totalDistanceM % 1000;

    document.getElementById('elapsed-time').textContent = `${elapsedTimeS.toFixed(1)} s`;
    
    // Carte principale
    document.getElementById('speed-3d-inst').textContent = `${(speedMS_3D * KMH_PER_MS).toFixed(1)} km/h`;
    document.getElementById('distance-km-m').textContent = `${totalDistKm.toFixed(2)} km | ${totalDistMeters.toFixed(1)} m`;
    
    document.getElementById('speed-avg').textContent = `${(speedAvgMS * KMH_PER_MS).toFixed(1)} km/h`; 
    document.getElementById('speed-max').textContent = `${(maxSpeedMS * KMH_PER_MS).toFixed(1)} km/h`;
    
    document.getElementById('speed-ms').textContent = `${speedMS_3D.toFixed(2)} m/s`;

    const percLight = (speedMS_3D / C_LIGHT) * 100;
    const percSound = (speedMS_3D / C_SOUND_SEA_LEVEL) * 100;

    document.getElementById('perc-light').textContent = `${percLight.toExponential(2)}%`;
    document.getElementById('perc-sound').textContent = `${percSound.toFixed(1)}%`;

    // --- MISE À JOUR POSITION/CAPTEURS ---
    document.getElementById('latitude').textContent = `${latitude.toFixed(6)}`;
    document.getElementById('longitude').textContent = `${longitude.toFixed(6)}`;
    document.getElementById('altitude').textContent = altitude !== null ? `${altitude.toFixed(0)} m` : 'N/A';
    document.getElementById('gps-accuracy').textContent = `${accuracy.toFixed(0)} m`;
    document.getElementById('underground').textContent = altitude !== null ? (altitude < 0 ? 'Oui' : 'Non') : 'N/A';
    
    document.getElementById('heading').textContent = heading !== null ? `${heading.toFixed(0)}°` : 'N/A';

    if (targetLat !== null && targetLon !== null) {
        const bearing = calculateBearing(latitude, longitude, targetLat, targetLon);
        document.getElementById('cap-dest').textContent = `${bearing.toFixed(0)}°`;
    } else {
        document.getElementById('cap-dest').textContent = 'N/A';
    }

    // --- ASTRO DÉTAILLÉ et MÉTÉO ---
    updateAstroDisplay(latitude, longitude);
    fetchWeatherData(latitude, longitude);
    
    lastPosition = position; 
}


// ===========================================
// 7. INITIALISATION ET CIBLE
// ===========================================

function se
