// ===========================================
// 1. CONSTANTES ET VARIABLES GLOBALES
// ===========================================

// --- PHYSIQUE / VITESSES ---
const C_LIGHT = 299792458;          // Vitesse de la lumière (m/s)
const C_SOUND_SEA_LEVEL = 343;      // Vitesse du son dans l'air sec à 20°C (m/s)
const EARTH_RADIUS = 6371000;       // Rayon de la Terre (m)
const KMH_PER_MS = 3.6;             // Conversion m/s -> km/h
const M_TO_AL = 1 / 9.461e15;       // Conversion mètres -> années-lumière
const DEG_TO_RAD = Math.PI / 180;
const RAD_TO_DEG = 180 / Math.PI;

// --- CONSTANTES ASTRO ---
const OBLIQUITY = 23.44 * DEG_TO_RAD; // Obliquité de l'écliptique (en radians)
const ECCENTRICITY = 0.0167;        // Excentricité de l'orbite terrestre

// --- ÉTAT DE L'APPLICATION ---
const WATCH_OPTIONS = {
    enableHighAccuracy: true,
    maximumAge: 500,
    timeout: 10000 
};
let watchID = null;         
let lastPosition = null;
let startTime = null;
let totalDistanceM = 0; // CORRIGÉ: Distance totale en mètres
let maxSpeedMS = 0;

// --- API MÉTÉO ---
const OPENWEATHER_API_KEY = 'YOUR_API_KEY'; 
let lastWeatherFetchTime = 0;
const WEATHER_FETCH_INTERVAL_MS = 60000; // 60 secondes entre les appels API

// --- REFERENCES DOM (inchangées) ---
const startBtn = document.getElementById('start-btn');
const stopBtn = document.getElementById('stop-btn');
const resetMaxBtn = document.getElementById('reset-max-btn');
const setTargetBtn = document.getElementById('set-target-btn');
const errorDisplay = document.getElementById('error-message');


// ===========================================
// 2. LOGIQUE DE CONTRÔLE ET D'ÉTAT
// ===========================================

function resetDisplay() {
    lastPosition = null;
    totalDistanceM = 0; // Assuré que la distance est à zéro
    startTime = null;
    maxSpeedMS = 0;
    targetLat = null; 
    targetLon = null;
    
    // Réinitialisation des éléments DOM
    const ids = ['elapsed-time', 'speed-3d-inst', 'speed-avg', 'speed-max', 'speed-ms', 'speed-mms',
        'perc-light', 'perc-sound', 'distance-km-m', 'distance-mm', 'distance-cosmic-s', 'distance-cosmic-al',
        'latitude', 'longitude', 'altitude', 'gps-accuracy', 'underground',
        'solar-true', 'solar-mean', 'eot', 'eot-eccentricity', 'eot-obliquity', 'solar-longitude-val', 'solar-day-duration',
        'lunar-phase-perc', 'mc-time', 'air-temp', 'pressure', 'humidity', 'wind-speed', 'uv-index', 'boiling-point', 
        'heading', 'bubble-level', 'light-level', 'cap-dest'
    ];

    ids.forEach(id => {
        const element = document.getElementById(id);
        if (element) {
            // Logique de réinitialisation avec les bonnes unités...
            if (['altitude', 'gps-accuracy'].includes(id)) element.textContent = '-- m';
            else if (['eot', 'eot-eccentricity', 'eot-obliquity'].includes(id)) element.textContent = '-- min';
            else if (['solar-longitude-val'].includes(id)) element.textContent = '-- °';
            else if (['heading', 'cap-dest', 'bubble-level'].includes(id)) element.textContent = '--°';
            else if (['lunar-phase-perc', 'humidity', 'perc-light', 'perc-sound'].includes(id)) element.textContent = '--%';
            else if (id === 'air-temp' || id === 'boiling-point') element.textContent = '-- °C';
            else if (id === 'pressure') element.textContent = '-- hPa';
            else if (id === 'wind-speed') element.textContent = '-- km/h';
            else if (id === 'solar-day-duration') element.textContent = '-- s';
            else element.textContent = '--';
        }
    });

    document.getElementById('elapsed-time').textContent = '0.00 s';
    document.getElementById('mc-time').textContent = '00:00:00';
    
    startBtn.disabled = false;
    stopBtn.disabled = true;
    resetMaxBtn.disabled = true;
    setTargetBtn.textContent = '📍 Aller';
    errorDisplay.style.display = 'none';
}

// ... (handleGeolocationError, startGPS, stopGPS, resetMaxSpeed sont inchangées) ...

function handleGeolocationError(error) {
    stopGPS(false);
    errorDisplay.style.display = 'block';

    switch (error.code) {
        case error.PERMISSION_DENIED:
            errorDisplay.textContent = "❌ L'accès à la localisation a été refusé.";
            break;
        case error.POSITION_UNAVAILABLE:
            errorDisplay.textContent = "⚠️ Position non disponible. (Assurez-vous d'être en extérieur).";
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
    
    watchID = navigator.geolocation.watchPosition(updateDisplay, handleGeolocationError, WATCH_OPTIONS);

    initSensorListeners(); 
    
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
// 3. FONCTIONS DE CALCUL (Distance, Astro)
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
    const now = new Date();
    const J2000 = new Date(Date.UTC(2000, 0, 1, 12, 0, 0));
    const D = (now.getTime() - J2000.getTime()) / (1000 * 60 * 60 * 24);

    const M = (357.529 + 0.98560028 * D) * DEG_TO_RAD;
    const L = (280.466 + 0.98564736 * D) * DEG_TO_RAD;
    
    const C_e = 2 * ECCENTRICITY * Math.sin(M) + 1.25 * ECCENTRICITY * ECCENTRICITY * Math.sin(2 * M);
    const lambda = L + C_e; 

    const alpha = Math.atan2(Math.cos(OBLIQUITY) * Math.sin(lambda), Math.cos(lambda));
    
    const EoT_rad = L - alpha;
    const EoT_minutes = (EoT_rad * RAD_TO_DEG) * 4;
    
    const Ce_minutes = C_e * RAD_TO_DEG * 4; 
    const Co_minutes = EoT_minutes - Ce_minutes; 

    const solarDayDuration = 86400; 

    return {
        eot: EoT_minutes,
        eccentricityComponent: Ce_minutes,
        obliquityComponent: Co_minutes,
        solarLongitude: (lambda * RAD_TO_DEG) % 360,
        solarDayDuration: solarDayDuration
    };
}


function updateAstroDisplay(latitude, longitude) {
    const now = new Date();
    
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
    document.getElementById('solar-mean').textContent = `${String(hsmHour).padStart(2, '0')}:${String(hsmMinute).padStart(2, '0')} (HSM)`;
    
    const solarDetails = calculateSolarDetails();
    const EoT_ms = solarDetails.eot * 60 * 1000; 
    
    // Calcul de l'Heure Solaire Vraie (HSV)
    const nowHSM = new Date(now.getTime() - (now.getTime() % 86400000) + totalMinutesLSM * 60 * 1000);
    const nowHSV = new Date(nowHSM.getTime() + EoT_ms);

    const lsvHour = nowHSV.getUTCHours(); 
    const lsvMinute = nowHSV.getUTCMinutes();
    const lsvSecond = Math.floor(nowHSV.getUTCSeconds());
    
    document.getElementById('solar-true').textContent = `${String(lsvHour).padStart(2, '0')}:${String(lsvMinute).padStart(2, '0')}:${String(lsvSecond).padStart(2, '0')} (HSV)`;
    
    document.getElementById('eot').textContent = `${solarDetails.eot.toFixed(2)} min`;
    document.getElementById('eot-eccentricity').textContent = `${solarDetails.eccentricityComponent.toFixed(2)} min`;
    document.getElementById('eot-obliquity').textContent = `${solarDetails.obliquityComponent.toFixed(2)} min`;
    document.getElementById('solar-longitude-val').textContent = `${solarDetails.solarLongitude.toFixed(2)} °`;
    document.getElementById('solar-day-duration').textContent = `${solarDetails.solarDayDuration.toFixed(0)} s`;


    // --- PHASE LUNAIRE (Illumination Réaliste) ---
    const JD = now.getTime() / 86400000 + 2440587.5; 
    const N = (JD - 2451549.5) / 365.25; 
    const D_moon = (297.85 + 445267.111 * N) % 360 * DEG_TO_RAD; // Élongation moyenne
    
    const illumination = 0.5 * (1 - Math.cos(D_moon)); 
    
    document.getElementById('lunar-phase-perc').textContent = `${(illumination * 100).toFixed(1)}%`;
}


async function fetchWeatherData(latitude, longitude) {
    if (OPENWEATHER_API_KEY === 'YOUR_API_KEY') {
        // Affichage factice si la clé n'est pas configurée
        document.getElementById('air-temp').textContent = `20.5 °C (Factice)`;
        document.getElementById('humidity').textContent = `65 % (Factice)`;
        document.getElementById('wind-speed').textContent = `15.0 km/h (Factice)`;
        document.getElementById('pressure').textContent = `1013 hPa`;
        document.getElementById('boiling-point').textContent = `100.0 °C`;
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
// 4. GESTION DES CAPTEURS 3D (Niveau à Bulle)
// ===========================================

function handleDeviceMotion(event) {
    const acc = event.accelerationIncludingGravity;
    
    if (!acc.x || !acc.y || !acc.z) {
        document.getElementById('bubble-level').textContent = 'Accéléro. Non Dispo';
        return;
    }

    // Calcul du Pitch (Inclinaison Y) et du Roll (Inclinaison X) par rapport à la gravité Z
    const pitch = Math.atan2(acc.z, acc.y) * RAD_TO_DEG; 
    const roll = Math.atan2(acc.z, acc.x) * RAD_TO_DEG; 
    
    // Mesure de la déviation : 0° est parfaitement plat (pitch=90, roll=90)
    const pitchDeviation = Math.abs(pitch - 90);
    const rollDeviation = Math.abs(roll - 90);

    document.getElementById('bubble-level').textContent = 
        `P: ${pitchDeviation.toFixed(1)}° | R: ${rollDeviation.toFixed(1)}°`;
}

function initSensorListeners() {
    // Tente d'obtenir la permission pour les capteurs de mouvement (requis sur iOS 13+)
    if (window.DeviceMotionEvent && typeof window.DeviceMotionEvent.requestPermission === 'function') {
        DeviceMotionEvent.requestPermission()
            .then(response => {
                if (response === 'granted') {
                    window.addEventListener('devicemotion', handleDeviceMotion);
                } else {
                    document.getElementById('bubble-level').textContent = 'Permission Refusée';
                }
            })
            .catch(console.error);
    } else if (window.DeviceMotionEvent) {
        // Android / Anciens navigateurs
        window.addEventListener('devicemotion', handleDeviceMotion);
    } else {
        document.getElementById('bubble-level').textContent = 'Capteurs Non Supportés';
    }
}

function stopSensorListeners() {
    window.removeEventListener('devicemotion', handleDeviceMotion);
}


// ===========================================
// 5. FONCTION PRINCIPALE D'AFFICHAGE (Correction Distance/Vitesse)
// ===========================================

function updateDisplay(position) {
    const { latitude, longitude, accuracy, timestamp } = position.coords;
    
    const speed = position.coords.speed;
    const altitude = position.coords.altitude; 
    const heading = position.coords.heading || null;

    const currentTime = timestamp;
    const elapsedTimeS = (currentTime - startTime) / 1000;

    // --- CALCULS VITESSE/DISTANCE ---
    let speedMS_Horiz = speed !== null ? speed : 0; 
    let speedMS_Vert = 0;
    
    if (lastPosition && lastPosition.coords.latitude !== undefined) { 
        const dLat = lastPosition.coords.latitude;
        const dLon = lastPosition.coords.longitude;
        const dAlt = lastPosition.coords.altitude; 
        const dt = (currentTime - lastPosition.timestamp) / 1000; 

        // FIX STABILITÉ: Ne calcule la distance/vitesse que si un temps significatif s'est écoulé (anti-bruit)
        if (dt > 0.1) { 
            const distHorizM = calculateDistance(dLat, dLon, latitude, longitude);
            
            // Mise à jour de la distance totale (distance parcourue)
            totalDistanceM += distHorizM;

            // Fallback: Si le GPS ne donne pas de vitesse (souvent le cas), on la calcule.
            if (position.coords.speed === null) {
                 speedMS_Horiz = distHorizM / dt;
            }

            // Calcul de la vitesse verticale (composante 3D)
            if (altitude !== null && dAlt !== null) { 
                speedMS_Vert = (altitude - dAlt) / dt;
            }
        }
    }
    
    // Vitesse 3D = sqrt(Horizontale² + Verticale²)
    const speedMS_3D = Math.sqrt(speedMS_Horiz * speedMS_Horiz + speedMS_Vert * speedMS_Vert);
    
    // Vitesse moyenne = Distance totale / Temps total
    // FIX STABILITÉ: Ne calcule la moyenne que si l'elapsedTime est > 1s pour éviter les divisions par des valeurs très petites au début.
    const speedAvgMS = elapsedTimeS > 1 ? totalDistanceM / elapsedTimeS : 0; 
    if (speedMS_3D > maxSpeedMS) { maxSpeedMS = speedMS_3D; }

    // --- MISE À JOUR AFFICHAGE ---
    document.getElementById('elapsed-time').textContent = `${elapsedTimeS.toFixed(1)} s`;
    document.getElementById('speed-3d-inst').textContent = `${(speedMS_3D * KMH_PER_MS).toFixed(1)} km/h`;
    document.getElementById('speed-avg').textContent = `${(speedAvgMS * KMH_PER_MS).toFixed(1)} km/h`; 
    document.getElementById('speed-max').textContent = `${(maxSpeedMS * KMH_PER_MS).toFixed(1)} km/h`;
    
    document.getElementById('distance-km-m').textContent = `${(totalDistanceM / 1000).toFixed(2)} km | ${(totalDistanceM % 1000).toFixed(1)} m`;
    // ... (Le reste de l'affichage de la vitesse et de la distance est inchangé) ...
    document.getElementById('speed-ms').textContent = `${speedMS_3D.toFixed(2)} m/s`;
    document.getElementById('speed-mms').textContent = `${(speedMS_3D * 1000).toFixed(0)} mm/s`;

    const percLight = (speedMS_3D / C_LIGHT) * 100;
    const percSound = (speedMS_3D / C_SOUND_SEA_LEVEL) * 100;
    const distCosmicSL = totalDistanceM / C_LIGHT; 
    const distCosmicAL = totalDistanceM * M_TO_AL; 

    document.getElementById('perc-light').textContent = `${percLight.toExponential(2)}%`;
    document.getElementById('perc-sound').textContent = `${percSound.toFixed(1)}%`;
    document.getElementById('distance-mm').textContent = `${(totalDistanceM * 1000).toFixed(0)} mm`;
    document.getElementById('distance-cosmic-s').textContent = `${distCosmicSL.toExponential(2)} s lumière`;
    document.getElementById('distance-cosmic-al').textContent = `${distCosmicAL.toExponential(2)} al`;

    // --- MISE À JOUR POSITION/CAPTEURS ---
    document.getElementById('latitude').textContent = `${latitude.toFixed(6)}`;
    document.getElementById('longitude').textContent = `${longitude.toFixed(6)}`;
    document.getElementById('altitude').textContent = altitude !== null ? `${altitude.toFixed(0)} m` : '-- m';
    document.getElementById('gps-accuracy').textContent = accuracy !== null ? `${accuracy.toFixed(0)} m` : '-- m';
    document.getElementById('underground').textContent = altitude !== null ? (altitude < 0 ? 'Oui' : 'Non') : '--';
    document.getElementById('heading').textContent = heading !== null ? `${heading.toFixed(0)}°` : '--°';
    const hour = new Date().getHours();
    document.getElementById('light-level').textContent = (hour > 6 && hour < 20) ? 'Jour (Élevé)' : 'Nuit (Faible)';

    if (targetLat !== null && targetLon !== null) {
        const bearing = calculateBearing(latitude, longitude, targetLat, targetLon);
        document.getElementById('cap-dest').textContent = `${bearing.toFixed(0)}°`;
    }

    // --- ASTRO DÉTAILLÉ et MÉTÉO ---
    updateAstroDisplay(latitude, longitude);
    fetchWeatherData(latitude, longitude);
    
    lastPosition = position; // Sauvegarde de la position pour le prochain calcul
}

// ... (Les fonctions setTargetDestination et initApp sont inchangées) ...

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
        setTargetBtn.textContent = '✅ Destination définie';
    } else {
        
