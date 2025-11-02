// =================================================================
// FICHIER JS : gnss-dashboard-full.js (PARTIE 1/2)
// Constantes, Variables Globales, Moteur GPS/EKF & Contrôles
// =================================================================

// --- CONSTANTES GLOBALES ---
const R = 6371000; // Rayon de la Terre en mètres
const D2R = Math.PI / 180; // Degrés vers Radians
const MS_TO_KMH = 3.6; // Mètres par seconde vers Kilomètres par heure
const DOM_SLOW_UPDATE_MS = 200; // Fréquence de base (Haute Fréquence)
const G = 9.81; // Accélération de la gravité (m/s²)
const EARTH_OMEGA = 7.2921e-5; // Vitesse angulaire de la Terre (rad/s)
const NETHER_SCALE = 8.0; // Facteur du mode Nether (1:8)
const SPEED_OF_SOUND = 343; // m/s (à 20°C au niveau de la mer)
const SPEED_OF_LIGHT = 299792458; // m/s
const EKFQ = 1; // Bruit de processus EKF (base)
const EKFR = 1; // Bruit de mesure EKF (base)

// --- VARIABLES D'ÉTAT GLOBALES ---
window.lPos = null; 
window.lTime = 0; 
window.distTotal = 0; 
window.speedMax = 0; 
window.startTime = Date.now(); 
window.timeMoving = 0; 
window.EKFState = [0, 0, 0, 0]; // [Lat, Lon, Speed, Altitude]
window.EKFPCov = [[10, 0, 0, 0], [0, 10, 0, 0], [0, 0, 10, 0], [0, 0, 0, 10]];
window.serverTimeOffset = 0; 
window.mass = 70.0; 
window.isGPSRunning = false;
window.geoWatchID = null;
window.isNetherMode = false;
window.currentFreqMS = DOM_SLOW_UPDATE_MS;
window.kalmanR = EKFR; 
window.environmentFactor = 1.0; 
window.mcTime = 0; // Temps Solaire Vrai en MS
window.lMap = null; // Instance Leaflet Map
window.lMarker = null; // Instance Leaflet Marker

// --- FONCTIONS UTILITAIRES ---
function $(id) {
    return document.getElementById(id);
}

function msToTime(ms) {
    let seconds = Math.floor(ms / 1000);
    let hours = Math.floor(seconds / 3600);
    seconds -= hours * 3600;
    let minutes = Math.floor(seconds / 60);
    seconds -= minutes * 60;

    const pad = (n) => String(n).padStart(2, '0');
    return `${pad(hours)}:${pad(minutes)}:${pad(seconds)}`;
}

function getCorrectedDate() {
    return new Date(Date.now() + window.serverTimeOffset); 
}

// =================================================================
// 1. MOTEUR GPS & EKF & IMU
// =================================================================

function startGPS() {
    if (navigator.geolocation && !window.isGPSRunning) {
        window.geoWatchID = navigator.geolocation.watchPosition(
            (position) => { handleNewPosition(position); },
            (error) => { console.error("Erreur GPS:", error.message); },
            { enableHighAccuracy: true, timeout: 5000, maximumAge: 0 }
        );
        window.isGPSRunning = true;
        if ($('toggle-gps-btn')) {
            $('toggle-gps-btn').textContent = '⏸ PAUSE GPS';
            $('toggle-gps-btn').style.backgroundColor = '#ffc107'; 
        }
    }
}

function stopGPS() {
    if (window.geoWatchID !== null) {
        navigator.geolocation.clearWatch(window.geoWatchID);
        window.geoWatchID = null;
    }
    window.isGPSRunning = false;
    if ($('toggle-gps-btn')) {
        $('toggle-gps-btn').textContent = '▶️ MARCHE GPS';
        $('toggle-gps-btn').style.backgroundColor = '#28a745'; 
    }
}

function toggleGPS() {
    window.isGPSRunning ? stopGPS() : startGPS();
}

function handleNewPosition(position) {
    const now = position.timestamp;
    
    if (window.lPos) {
        const dt = (now - window.lTime) / 1000; 
        
        // Calculs Haversine
        const dLat = (position.coords.latitude - window.lPos.coords.latitude) * D2R;
        const dLon = (position.coords.longitude - window.lPos.coords.longitude) * D2R;
        const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) + Math.cos(window.lPos.coords.latitude * D2R) * Math.cos(position.coords.latitude * D2R) * Math.sin(dLon / 2) * Math.sin(dLon / 2);
        const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        let distance_meters = R * c;
        
        let dAlt = position.coords.altitude && window.lPos.coords.altitude ? (position.coords.altitude - window.lPos.coords.altitude) : 0;
        window.verticalSpeed = dt > 0 ? dAlt / dt : 0;
        
        const scale = window.isNetherMode ? NETHER_SCALE : 1.0;
        const distance_meters_scaled = distance_meters * scale;
        window.distTotal += distance_meters_scaled;

        let instantSpeed = dt > 0 ? distance_meters_scaled / dt : 0;
        if (instantSpeed < 0) instantSpeed = 0;
        
        // EKF Update (Speed)
        let R_effective = window.kalmanR * window.environmentFactor; 
        let predSpeed = window.EKFState[2]; 
        let K = window.EKFPCov[2][2] / (window.EKFPCov[2][2] + R_effective); 
        window.EKFState[2] = predSpeed + K * (instantSpeed - predSpeed);
        window.EKFPCov[2][2] = (1 - K) * window.EKFPCov[2][2] + EKFQ * dt * window.environmentFactor;
        let filteredSpeed = window.EKFState[2];
        if (filteredSpeed < 0) filteredSpeed = 0;
        
        let accelLong = dt > 0 ? (filteredSpeed - predSpeed) / dt : 0;
        
        // Mise à jour des métriques de mouvement
        const isMoving = filteredSpeed * MS_TO_KMH > 1.0; 
        if (isMoving) window.timeMoving += dt * 1000; 
        if (filteredSpeed * MS_TO_KMH > window.speedMax) window.speedMax = filteredSpeed * MS_TO_KMH;
        const totalTimeMovingS = window.timeMoving / 1000;
        window.speedAvg = totalTimeMovingS > 0 ? (window.distTotal / totalTimeMovingS) * MS_TO_KMH : 0;

        // Mise à jour de l'interface (Vitesse/Distance)
        if ($('speed-stable')) $('speed-stable').textContent = `${(filteredSpeed * MS_TO_KMH).toFixed(2)} km/h`;
        if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${filteredSpeed.toFixed(2)} m/s | ${(filteredSpeed * 1000).toFixed(0)} mm/s`;
        if ($('speed-avg-moving')) $('speed-avg-moving').textContent = `${window.speedAvg.toFixed(5)} km/h`;
        if ($('accel-long')) $('accel-long').textContent = `${accelLong.toFixed(3)} m/s²`;
        if ($('force-g-long')) $('force-g-long').textContent = `${(accelLong / G).toFixed(2)} G`;
        if ($('vertical-speed')) $('vertical-speed').textContent = `${window.verticalSpeed.toFixed(2)} m/s`;
        if ($('speed-max')) $('speed-max').textContent = `${window.speedMax.toFixed(5)} km/h`;
        
        // Vitesse 3D (Inst.)
        let rawSpeed = position.coords.speed !== null ? position.coords.speed * scale : instantSpeed;
        if (rawSpeed < 0) rawSpeed = 0;
        if ($('speed-3d-inst')) {
             $('speed-3d-inst').textContent = (rawSpeed * MS_TO_KMH > 0.01) ? `${(rawSpeed * MS_TO_KMH).toFixed(1)} km/h` : `-- km/h`;
        }
        
        // Affichage des données GPS brutes
        if ($('latitude')) $('latitude').textContent = position.coords.latitude.toFixed(6);
        if ($('longitude')) $('longitude').textContent = position.coords.longitude.toFixed(6);
        if ($('altitude-gps')) $('altitude-gps').textContent = `${position.coords.altitude ? position.coords.altitude.toFixed(2) : 'N/A'} m`;
        if ($('gps-precision')) $('gps-precision').textContent = `${position.coords.accuracy.toFixed(2)} m`;
        if ($('speed-raw-ms')) $('speed-raw-ms').textContent = `${(distance_meters / dt).toFixed(2)} m/s`;
        if ($('underground-status')) $('underground-status').textContent = position.coords.altitude && position.coords.altitude < -50 ? 'OUI (<-50m)' : 'Non';
    }

    window.lPos = position;
    window.lTime = now;
}

function handleIMU(event) {
    if (!window.lPos) return;

    const latA = window.lPos.coords.latitude;
    const speed = window.EKFState[2];
    
    // Calcul Force de Coriolis (simplifié)
    const coriolis = 2 * window.mass * speed * EARTH_OMEGA * Math.sin(latA * D2R);
    if ($('coriolis-force')) $('coriolis-force').textContent = `${coriolis.toExponential(2)} N`;

    const accIncGrav = event && event.accelerationIncludingGravity;
    if ($('accel-vertical-imu')) $('accel-vertical-imu').textContent = accIncGrav && accIncGrav.z ? `${accIncGrav.z.toFixed(3)} m/s²` : `N/A`;
    if ($('force-g-vertical')) $('force-g-vertical').textContent = accIncGrav && accIncGrav.z ? `${(accIncGrav.z / G).toFixed(2)} G` : `N/A`;
}

function startIMU() {
    if (window.DeviceOrientationEvent || window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', handleIMU, false);
    } 
    handleIMU(null); 
}

// =================================================================
// 2. FONCTIONS DE CONTRÔLE
// =================================================================

function resetDistance() {
    window.distTotal = 0;
    window.timeMoving = 0;
    console.log("Distance et temps de mouvement réinitialisés.");
}

function resetSpeedMax() {
    window.speedMax = 0;
    console.log("Vitesse maximale réinitialisée.");
}

function resetAll() {
    stopGPS();
    resetDistance();
    resetSpeedMax();
    window.EKFState = [0, 0, 0, 0];
    window.EKFPCov = [[10, 0, 0, 0], [0, 10, 0, 0], [0, 0, 10, 0], [0, 0, 0, 10]];
    window.startTime = Date.now();
    $('emergency-stop-btn').textContent = '🛑 Arrêt d\'urgence: INACTIF 🚨';
    $('emergency-stop-btn').style.backgroundColor = '#dc3545';
    console.log("Tableau de bord complètement réinitialisé.");
}

function emergencyStop() {
    if (window.isGPSRunning) {
        stopGPS();
        $('emergency-stop-btn').textContent = '🛑 ARRÊT D\'URGENCE ACTIF ⛔';
        $('emergency-stop-btn').style.backgroundColor = 'black';
        alert("Arrêt d'urgence déclenché ! GPS désactivé.");
    } else {
        $('emergency-stop-btn').textContent = '🛑 Arrêt d\'urgence: INACTIF 🚨';
        $('emergency-stop-btn').style.backgroundColor = '#dc3545';
        startGPS();
    }
}

function toggleNetherMode() {
    window.isNetherMode = !window.isNetherMode;
    const btn = $('nether-toggle-btn');
    const display = $('mode-nether');
    if (window.isNetherMode) {
        btn.textContent = 'Mode Normal';
        display.textContent = 'ACTIF (1:8)';
        btn.style.backgroundColor = '#ff6600';
    } else {
        btn.textContent = 'Nether Mode';
        display.textContent = 'DÉSACTIVÉ (1:1)';
        btn.style.backgroundColor = '#5bc0de'; 
    }
}

function updateKalmanParameters() {
    const overrideInput = $('gps-accuracy-override');
    const envSelect = $('environment-select');

    // 1. Gestion de la Précision GPS forcée
    const overrideValue = parseFloat(overrideInput.value);
    window.kalmanR = overrideValue > 0 ? overrideValue : EKFR; 
    
    // 2. Gestion du Facteur d'Environnement
    const factorMap = {
        'NORMAL': 1.0,
        'FOREST': 1.5,
        'CONCRETE': 3.0,
        'METAL': 2.5
    };
    window.environmentFactor = factorMap[envSelect.value] || 1.0;
    
    // Mise à jour visuelle (EKF R_Effective)
    if ($('gps-accuracy-effective')) $('gps-accuracy-effective').textContent = `${(window.kalmanR * window.environmentFactor).toFixed(2)} m`;
}
// =================================================================
// FICHIER JS : gnss-dashboard-full.js (PARTIE 2/2)
// Astro, Map, Météo & Boucle Principale
// =================================================================

// =================================================================
// 3. LOGIQUE ASTRO, TST & MAP
// =================================================================

function getSolarTime(date, lon) {
    // Calculs TST/EOT (répétition complète pour s'assurer que la fonction est définie)
    const J = window.SunCalc.getJDay(date); 
    const M = (357.5291 + 0.98560028 * J) * D2R; 
    const L = (280.4665 + 0.98564736 * J) * D2R; 
    const C = (1.9148 * Math.sin(M) + 0.0200 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M)) * D2R; 
    const lambda = L + C; 
    const epsilon = (23.439 - 0.00000036 * J) * D2R; 
    const ra = Math.atan2(Math.cos(epsilon) * Math.sin(lambda), Math.cos(lambda));
    const EOT_rad = L - ra; 
    const EOT_minutes = (EOT_rad / (2 * Math.PI)) * 1440; 
    const EOT_MS = EOT_minutes * 60000;
    const lon_time_ms = lon * 240 * 1000; 
    const date_ms_since_midnight = date.getUTCHours() * 3600000 + date.getUTCMinutes() * 60000 + date.getUTCSeconds() * 1000;
    const tst_ms_local = (date_ms_since_midnight + lon_time_ms + EOT_MS) % 86400000;
    
    const TST_MS = tst_ms_local; 
    const tst_date = new Date(TST_MS);
    const TST = tst_date.toLocaleTimeString('en-US', { hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false, timeZone: 'UTC' });
    
    const EOT_ABS = Math.abs(EOT_MS);
    const EOT_SIGN = EOT_MS >= 0 ? '+' : '-';
    const EOT_sec = Math.floor(EOT_ABS / 1000) % 60;
    const EOT_min = Math.floor(EOT_ABS / 60000);
    const EOT = `${EOT_SIGN}${String(EOT_min).padStart(2, '0')}:${String(EOT_sec).padStart(2, '0')}`;
    
    return { EOT_MS, EOT, TST_MS, TST, lambda };
}

function initSolarClock() {
    const container = $('minecraft-clock');
    if (container) {
        container.innerHTML = ''; 
        const canvas = document.createElement('canvas');
        canvas.id = 'mc-clock-canvas';
        canvas.width = 180; 
        canvas.height = 180; 
        canvas.style.width = '100%'; 
        canvas.style.height = '100%';
        container.appendChild(canvas);
    }
}

function drawSolarClock(sunPos) {
    const canvas = $('mc-clock-canvas');
    if (!canvas || !sunPos || window.mcTime === null) return; 
    
    const ctx = canvas.getContext('2d');
    const center = canvas.width / 2;
    const radius = center * 0.9;
    
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Illumination et Couleurs dynamiques
    const elevation = sunPos.altitude; 
    const dayFactor = Math.min(1, Math.max(0, (elevation / D2R + 6) / 40)); 
    const skyColorR = Math.round(30 + 170 * dayFactor);
    const skyColorG = Math.round(50 + 150 * dayFactor);
    const skyColorB = Math.round(100 + 155 * dayFactor);
    const skyColor = `rgb(${skyColorR}, ${skyColorG}, ${skyColorB})`;

    const waterColorR = Math.round(20 + 40 * dayFactor);
    const waterColorG = Math.round(40 + 70 * dayFactor);
    const waterColorB = Math.round(60 + 90 * dayFactor);
    const groundColor = `rgb(${waterColorR}, ${waterColorG}, ${waterColorB})`;

    // Rendu Ciel/Sol
    ctx.beginPath();
    ctx.arc(center, center, radius, -Math.PI, 0); ctx.lineTo(center + radius, center);
    ctx.fillStyle = skyColor; ctx.fill();
    ctx.beginPath();
    ctx.arc(center, center, radius, 0, Math.PI); ctx.lineTo(center - radius, center);
    ctx.fillStyle = groundColor; ctx.fill();
    ctx.strokeStyle = '#333'; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(center - radius, center); ctx.lineTo(center + radius, center); ctx.stroke();
    
    // Aiguille TST
    const total_seconds = window.mcTime / 1000;
    const angle_from_north = (total_seconds / 86400) * 2 * Math.PI;
    const TST_ANGLE = angle_from_north - Math.PI; 
    ctx.save();
    ctx.translate(center, center);
    ctx.rotate(TST_ANGLE); 
    ctx.strokeStyle = '#D9534F'; ctx.lineWidth = 4;
    ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(0, -radius * 0.9); ctx.stroke();
    ctx.restore();

    // Heure au centre
    ctx.fillStyle = '#000';
    ctx.font = `${center * 0.25}px Arial bold`; 
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    const TST_time = new Date(window.mcTime).toLocaleTimeString('en-US', { hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false, timeZone: 'UTC' });
    ctx.fillText(TST_time, center, center); 
}

function updateAstro(latA, lonA) {
    const now = getCorrectedDate(); 
    if (now === null || !window.SunCalc) return; 

    const sunPos = window.SunCalc.getPosition(now, latA, lonA);
    const solarTimes = getSolarTime(now, lonA); 
    window.mcTime = solarTimes.TST_MS; 

    // Mise à jour de l'affichage Astro
    if ($('tst')) $('tst').textContent = solarTimes.TST;
    if ($('eot')) $('eot').textContent = `${solarTimes.EOT} min`;
    if ($('sun-elevation')) $('sun-elevation').textContent = (sunPos.altitude / D2R).toFixed(2) + " °";
    if ($('time-minecraft')) $('time-minecraft').textContent = solarTimes.TST;
    if ($('clock-status')) $('clock-status').textContent = `TST: ${solarTimes.TST} (Élév: ${(sunPos.altitude / D2R).toFixed(2)}°)`;
    if ($('ecliptic-long')) $('ecliptic-long').textContent = (solarTimes.lambda / D2R).toFixed(2) + " °";

    drawSolarClock(sunPos); 
}

function initMap() {
    const defaultLat = 43.296482; 
    const defaultLon = 5.36978;
    
    if ($('map-container')) {
        if (window.lMap) window.lMap.remove(); 
        // L.map et L.tileLayer sont disponibles via leaflet.js (lien CDN dans l'HTML)
        window.lMap = L.map('map-container').setView([defaultLat, defaultLon], 13);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: '© OpenStreetMap contributors'
        }).addTo(window.lMap);
        window.lMarker = L.marker([defaultLat, defaultLon]).addTo(window.lMap);
    }
}

function updateMap() {
    if (window.lMap && window.lMarker && window.lPos) {
        const lat = window.lPos.coords.latitude;
        const lon = window.lPos.coords.longitude;
        const newLatLng = new L.LatLng(lat, lon);
        
        window.lMarker.setLatLng(newLatLng);
        window.lMap.setView(newLatLng, window.lMap.getZoom(), { animate: false }); 
    }
}


// =================================================================
// 4. API MÉTÉO & DONNÉES EXTERNES
// =================================================================

async function fetchWeatherData(lat, lon) {
    // Si votre API est déployée sur Vercel :
    const url = `api/weather.js?lat=${lat}&lon=${lon}`; 
    
    try {
        const response = await fetch(url);
        if (!response.ok) {
            throw new Error(`Erreur HTTP! status: ${response.status}`);
        }
        const data = await response.json();
        
        // Mise à jour des champs Météo
        if (data && data.main) {
            if ($('temp-air')) $('temp-air').textContent = `${data.main.temp.toFixed(1)} °C`;
            if ($('pressure')) $('pressure').textContent = `${data.main.pressure} hPa`;
            if ($('humidity')) $('humidity').textContent = `${data.main.humidity} %`;
            if ($('temp-feels-like')) $('temp-feels-like').textContent = `${data.main.feels_like.toFixed(1)} °C`;
            if ($('wind-speed-ms')) $('wind-speed-ms').textContent = `${data.wind.speed.toFixed(1)} m/s`;
            if ($('wind-direction')) $('wind-direction').textContent = `${data.wind.deg}°`;

        } else {
             throw new Error('Données météo non valides reçues.');
        }

    } catch (error) {
        console.error("Erreur lors de la récupération des données météo:", error);
        if ($('temp-air')) $('temp-air').textContent = `N/A (Erreur API)`;
    }
}

function updateAllExternalData() {
    const currentLat = window.lPos ? window.lPos.coords.latitude : 43.296482; 
    const currentLon = window.lPos ? window.lPos.coords.longitude : 5.36978;

    // Appel à l'API Météo
    fetchWeatherData(currentLat, currentLon); 

    // Mise à jour des champs basés sur des calculs internes (Physique)
    const filteredSpeed = window.EKFState[2];
    const kineticEnergy = 0.5 * window.mass * filteredSpeed * filteredSpeed;
    if ($('kinetic-energy')) $('kinetic-energy').textContent = `${kineticEnergy.toFixed(2)} J`;
    if ($('mechanical-power')) $('mechanical-power').textContent = `0.00 W`;
    
    // Tentative de réinitialisation/mise à jour d'autres champs
    [
        'dew-point', 'visibility', 'uv-index', 'precipitation-rate', 
        'magnetic-field', 'air-flow', 'co2-level', 
        'air-density', 'ozone-conc', 'ph-level', 'solar-radiation', 'noise-level', 
        'soil-type', 'ndvi-index', 'lsm', 'noon-solar', 'day-duration'
    ].forEach(id => {
        const elem = $(id);
        if (elem && elem.textContent.includes('N/A')) {
             // Laisser N/A si non mis à jour par l'API ou un calcul
        }
    });
}


// =================================================================
// 5. BOUCLE PRINCIPALE & DÉMARRAGE
// =================================================================

function syncTimeWithServer() {
    window.serverTimeOffset = 0; 
    const now = getCorrectedDate();
    const time_str = now.toLocaleTimeString('en-US', { hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false });
    
    if ($('local-time')) $('local-time').textContent = `${time_str} (${now.toLocaleTimeString('en-US', {timeZoneName: 'short', hour12: false}).split(' ')[1]})`;
    if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
    if ($('date-display-astro')) $('date-display-astro').textContent = now.toLocaleDateString('fr-FR');
}

function updateDOMTimers() {
    const elapsed = Date.now() - window.startTime;
    if ($('time-elapsed')) $('time-elapsed').textContent = msToTime(elapsed) + " (" + (elapsed / 1000).toFixed(2) + " s)";
    if ($('time-moving')) $('time-moving').textContent = msToTime(window.timeMoving) + " (" + (window.timeMoving / 1000).toFixed(2) + " s)";
    
    // Distance totale
    const dist_km = window.distTotal / 1000;
    if ($('distance-total-km')) $('distance-total-km').textContent = `${dist_km.toFixed(3)} km | ${window.distTotal.toFixed(2)} m`;
    
    // Vitesse de la lumière et du son
    const filteredSpeed = window.EKFState[2];
    if ($('perc-speed-sound')) $('perc-speed-sound').textContent = `${(filteredSpeed / SPEED_OF_SOUND * 100).toFixed(4)} %`;
    if ($('perc-speed-c')) $('perc-speed-c').textContent = `${(filteredSpeed / SPEED_OF_LIGHT * 100).toExponential(2)} %`;

    // Distance cosmique
    const cosmic_seconds = window.distTotal / SPEED_OF_LIGHT;
    if ($('distance-cosmic')) $('distance-cosmic').textContent = `${cosmic_seconds.toExponential(2)} s lumière | ${(cosmic_seconds / (60*60*24*365.25)).toExponential(2)} al`;
}

let domID = null; 

function startMainLoop() {
    if (domID === null) {
        domID = setInterval(() => {
            const currentLat = window.lPos ? window.lPos.coords.latitude : 43.296482; 
            const currentLon = window.lPos ? window.lPos.coords.longitude : 5.36978;
            
            updateDOMTimers(); 
            updateAstro(currentLat, currentLon); 
            
            if(window.isGPSRunning) updateMap(); 

            // SYNCHRO NTP : toutes les 5 minutes (300000ms)
            if (Date.now() % 300000 < window.currentFreqMS) { 
                syncTimeWithServer(); 
            }
            // Mise à jour des données externes (Météo, Physique, etc.) toutes les 10 secondes
            if (Date.now() % 10000 < window.currentFreqMS) { 
                updateAllExternalData(); 
            }
        }, window.currentFreqMS);
    }
}

function setFrequency() {
    const freqSelect = $('freq-select');
    if (!freqSelect) return;
    
    const isHigh = freqSelect.value === 'HIGH_FREQ';
    const newFreqMS = isHigh ? 200 : 1000;
    
    if (newFreqMS !== window.currentFreqMS) {
        window.currentFreqMS = newFreqMS;
        if (domID !== null) {
            clearInterval(domID);
            domID = null;
        }
        startMainLoop();
    }
}

function toggleNightMode() {
    document.body.classList.toggle('night-mode');
    const isNight = document.body.classList.contains('night-mode');
    if ($('toggle-mode-btn')) {
        $('toggle-mode-btn').textContent = isNight ? '☀️ Mode Jour' : '🌓 Mode Nuit';
    }
}

function captureData() {
    alert("Données capturées dans la console (Simulation de sauvegarde)...");
    console.log("--- CAPTURE DE DONNÉES (INSTANTANÉE) ---");
    console.table({
        'Temps (UTC)': getCorrectedDate().toUTCString(),
        'Latitude': window.lPos ? window.lPos.coords.latitude.toFixed(6) : 'N/A',
        'Vitesse Stable (m/s)': window.EKFState[2].toFixed(2),
        'Distance Totale (km)': (window.distTotal / 1000).toFixed(3),
        'EKF R Efficace': window.kalmanR * window.environmentFactor
    });
}

function initializeControlListeners() {
    if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', toggleGPS);
    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', emergencyStop);
    if ($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', toggleNetherMode);
    
    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', resetDistance);
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', resetSpeedMax);
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', resetAll);
    
    if ($('freq-select')) $('freq-select').addEventListener('change', setFrequency);
    
    if ($('gps-accuracy-override')) $('gps-accuracy-override').addEventListener('change', updateKalmanParameters);
    if ($('environment-select')) $('environment-select').addEventListener('change', updateKalmanParameters);

    if ($('toggle-mode-btn')) $('toggle-mode-btn').addEventListener('click', toggleNightMode);
    if ($('data-capture-btn')) $('data-capture-btn').addEventListener('click', captureData);

    updateKalmanParameters(); 
}


// --- INITIALISATION GLOBALE (ENTRY POINT) ---

document.addEventListener('DOMContentLoaded', () => {
    
    if (!window.SunCalc) {
        console.warn("Avertissement: suncalc.js n'est pas chargé. Assurez-vous d'avoir le CDN ou le fichier local.");
    }
    
    initMap(); 
    initSolarClock(); 
    
    initializeControlListeners();
    
    startGPS(); 
    startIMU(); 
    
    syncTimeWithServer(); 
    updateAllExternalData(); 
    
    startMainLoop(); 
});
