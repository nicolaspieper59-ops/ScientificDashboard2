// =================================================================
// FICHIER JS PARTIE 1/2 : gnss-dashboard-part1.js (FINAL)
// Moteur GPS, EKF et Contrôles d'état (Vérifié 2025-11-02)
// =================================================================

// --- CONSTANTES ---
const R = 6371000; // Rayon de la Terre en mètres
const D2R = Math.PI / 180; // Degrés vers Radians
const MS_TO_KMH = 3.6; // Mètres par seconde vers Kilomètres par heure
const DOM_SLOW_UPDATE_MS = 200; // Fréquence de base (Haute Fréquence)
const G = 9.81; // Accélération de la gravité (m/s²)
const EARTH_OMEGA = 7.2921e-5; // Vitesse angulaire de la Terre (rad/s)
const NETHER_SCALE = 8.0; // Facteur du mode Nether (1:8)
const SPEED_OF_SOUND = 343; // m/s (à 20°C au niveau de la mer)
const SPEED_OF_LIGHT = 299792458; // m/s

// --- VARIABLES EKF DE BASE ---
const EKFQ = 1; // Bruit de processus (base)
const EKFR = 1; // Bruit de mesure (base)

// --- VARIABLES D'ÉTAT GLOBALES ---
window.lPos = null; 
window.lTime = 0; 
window.distTotal = 0; 
window.speedMax = 0; 
window.startTime = Date.now(); 
window.timeMoving = 0; 
window.EKFState = [0, 0, 0, 0]; 
window.EKFPCov = [[10, 0, 0, 0], [0, 10, 0, 0], [0, 0, 10, 0], [0, 0, 0, 10]];
window.serverTimeOffset = 0; // Fixé à 0 (heure client)
window.speedAvg = 0; // Vitesse moyenne
window.verticalSpeed = 0; // Vitesse verticale
window.mass = 70.0; // Masse par défaut (correspond au HTML)

// --- VARIABLES DE CONTRÔLE ---
window.isGPSRunning = false;
window.geoWatchID = null;
window.isNetherMode = false;
window.isHighFreq = true; 
window.currentFreqMS = DOM_SLOW_UPDATE_MS;
window.kalmanR = EKFR; // R dynamique (forcé par l'utilisateur)
window.environmentFactor = 1.0; // Facteur d'environnement Kalman

// --- FONCTION UTILITAIRE ---
function $(id) {
    return document.getElementById(id);
}

// --- GESTION GPS/EKF ---

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
    if (window.isGPSRunning) {
        stopGPS();
    } else {
        startGPS();
    }
}

function handleNewPosition(position) {
    const now = position.timestamp;
    
    if (window.lPos) {
        const dt = (now - window.lTime) / 1000; 
        
        // Calcul Haversine (distance brute)
        const dLat = (position.coords.latitude - window.lPos.coords.latitude) * D2R;
        const dLon = (position.coords.longitude - window.lPos.coords.longitude) * D2R;
        const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) + Math.cos(window.lPos.coords.latitude * D2R) * Math.cos(position.coords.latitude * D2R) * Math.sin(dLon / 2) * Math.sin(dLon / 2);
        const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        let distance_meters = R * c;
        
        // Calcul Vitesse Verticale
        let dAlt = position.coords.altitude && window.lPos.coords.altitude ? (position.coords.altitude - window.lPos.coords.altitude) : 0;
        window.verticalSpeed = dt > 0 ? dAlt / dt : 0;
        if ($('vertical-speed')) $('vertical-speed').textContent = `${window.verticalSpeed.toFixed(2)} m/s`;

        // --- MISE À L'ÉCHELLE NETHER MODE ---
        const scale = window.isNetherMode ? NETHER_SCALE : 1.0;
        const distance_meters_scaled = distance_meters * scale;
        window.distTotal += distance_meters_scaled;

        // Vitesse instantanée brute (calcul delta)
        let instantSpeed = dt > 0 ? distance_meters_scaled / dt : 0;
        if (instantSpeed < 0) instantSpeed = 0;
        
        // --- FILTRE EKF (Vitesse Stable) ---
        let R_effective = window.kalmanR * window.environmentFactor; 
        let predSpeed = window.EKFState[2]; 
        let K = window.EKFPCov[2][2] / (window.EKFPCov[2][2] + R_effective); 
        window.EKFState[2] = predSpeed + K * (instantSpeed - predSpeed);
        window.EKFPCov[2][2] = (1 - K) * window.EKFPCov[2][2] + EKFQ * dt * window.environmentFactor;
        
        let filteredSpeed = window.EKFState[2];
        if (filteredSpeed < 0) filteredSpeed = 0;

        // Calcul de l'accélération longitudinale (simple delta EKF)
        let accelLong = dt > 0 ? (filteredSpeed - predSpeed) / dt : 0;
        if ($('accel-long')) $('accel-long').textContent = `${accelLong.toFixed(3)} m/s²`;
        if ($('force-g-long')) $('force-g-long').textContent = `${(accelLong / G).toFixed(2)} G`;
        
        // Mise à jour des métriques de mouvement
        const isMoving = filteredSpeed * MS_TO_KMH > 1.0; 
        if (isMoving) window.timeMoving += dt * 1000; 
        if (filteredSpeed * MS_TO_KMH > window.speedMax) window.speedMax = filteredSpeed * MS_TO_KMH;
        
        // Calcul Vitesse Moyenne (sur la distance totale / temps de mouvement)
        const totalTimeMovingS = window.timeMoving / 1000;
        window.speedAvg = totalTimeMovingS > 0 ? (window.distTotal / totalTimeMovingS) * MS_TO_KMH : 0;
        if ($('speed-avg-moving')) $('speed-avg-moving').textContent = `${window.speedAvg.toFixed(5)} km/h`;

        // --- AFFICHAGE ---
        const filteredSpeedKmh = (filteredSpeed * MS_TO_KMH).toFixed(2);
        if ($('speed-stable')) $('speed-stable').textContent = `${filteredSpeedKmh} km/h`;
        if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${filteredSpeed.toFixed(2)} m/s | ${(filteredSpeed * 1000).toFixed(0)} mm/s`;

        let rawSpeed = position.coords.speed !== null ? position.coords.speed * scale : instantSpeed;
        if (rawSpeed < 0) rawSpeed = 0;
        if ($('speed-3d-inst')) {
             $('speed-3d-inst').textContent = (rawSpeed * MS_TO_KMH > 0.01) ? `${(rawSpeed * MS_TO_KMH).toFixed(1)} km/h` : `-- km/h`;
        }
        
        // Pourcentage des Vitesses C et Son
        if ($('perc-speed-sound')) $('perc-speed-sound').textContent = `${((filteredSpeed / SPEED_OF_SOUND) * 100).toFixed(2)} %`;
        if ($('perc-speed-c')) $('perc-speed-c').textContent = `${((filteredSpeed / SPEED_OF_LIGHT) * 100).toExponential(3)} %`;

        // Mise à jour de la précision EKF et GPS
        const error_percentage = (window.EKFPCov[2][2] / R_effective) * 100;
        if ($('speed-error-perc')) $('speed-error-perc').textContent = `${error_percentage.toFixed(2)} % (P / R)`;
        if ($('gps-accuracy-effective')) $('gps-accuracy-effective').textContent = `${R_effective.toFixed(4)} m`;
        
        // Sous-sol (simulation simple: Altitude GPS < -50m)
        const isUnderground = position.coords.altitude && position.coords.altitude < -50;
        if ($('underground-status')) $('underground-status').textContent = isUnderground ? 'OUI (<-50m)' : 'Non';

        // Coordonnées brutes
        if ($('latitude')) $('latitude').textContent = position.coords.latitude.toFixed(6);
        if ($('longitude')) $('longitude').textContent = position.coords.longitude.toFixed(6);
        if ($('altitude-gps')) $('altitude-gps').textContent = `${position.coords.altitude ? position.coords.altitude.toFixed(2) : 'N/A'} m`;
        if ($('gps-precision')) $('gps-precision').textContent = `${position.coords.accuracy.toFixed(2)} m`;
        if ($('speed-raw-ms')) $('speed-raw-ms').textContent = `${(distance_meters / dt).toFixed(2)} m/s`;
    }

    // Sauvegarde de la nouvelle position
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

// --- LOGIQUE DES CONTRÔLES ---

function toggleNetherMode() {
    window.isNetherMode = !window.isNetherMode;
    const modeText = window.isNetherMode ? `ACTIVÉ (1:${NETHER_SCALE})` : 'DÉSACTIVÉ (1:1)';
    if ($('mode-nether')) $('mode-nether').textContent = modeText;
    if ($('nether-toggle-btn')) $('nether-toggle-btn').textContent = window.isNetherMode ? 'Nether Mode (ON)' : 'Nether Mode';
}

function updateKalmanParameters() {
    // 1. Facteur d'environnement (affecte R)
    const envSelect = $('environment-select');
    if (envSelect) {
        const selectedValue = envSelect.value;
        let factor;
        switch(selectedValue) {
            case 'FOREST': factor = 1.5; break;
            case 'CONCRETE': factor = 3.0; break;
            case 'METAL': factor = 2.5; break;
            default: factor = 1.0; 
        }
        window.environmentFactor = factor;
    }

    // 2. Précision GPS forcée (affecte R directement si > 0)
    const accOverride = $('gps-accuracy-override');
    if (accOverride) {
        const overrideValue = parseFloat(accOverride.value);
        window.kalmanR = overrideValue > 0 ? overrideValue : EKFR; 
    }
}

function resetDistance() {
    window.distTotal = 0;
    window.timeMoving = 0;
    window.speedAvg = 0;
    if ($('distance-total-km')) $('distance-total-km').textContent = '0.000 km | 0.00 m';
    if ($('time-moving')) $('time-moving').textContent = '0.00 s';
    if ($('speed-avg-moving')) $('speed-avg-moving').textContent = '0.00000 km/h';
    if ($('distance-cosmic')) $('distance-cosmic').textContent = '0.00e+00 s lumière | 0.00e+00 al';
}

function resetSpeedMax() {
    window.speedMax = 0;
    if ($('speed-max')) $('speed-max').textContent = '0.00000 km/h';
}

function resetAll() {
    resetDistance();
    resetSpeedMax();
    window.startTime = Date.now();
    window.EKFState = [0, 0, 0, 0]; 
    if ($('time-elapsed')) $('time-elapsed').textContent = '0.00 s';
    if ($('speed-error-perc')) $('speed-error-perc').textContent = 'N/A';
    if ($('gps-accuracy-effective')) $('gps-accuracy-effective').textContent = 'N/A';
    if ($('local-time')) $('local-time').textContent = 'N/A';
}

function initializeControlListeners() {
    // Boutons de contrôle
    if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', toggleGPS);
    if ($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', toggleNetherMode);
    
    // Réinitialisation
    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', resetDistance);
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', resetSpeedMax);
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', resetAll);

    // Contrôles EKF
    if ($('environment-select')) $('environment-select').addEventListener('change', updateKalmanParameters);
    if ($('gps-accuracy-override')) $('gps-accuracy-override').addEventListener('input', updateKalmanParameters);

    updateKalmanParameters(); // Initialiser les paramètres EKF
                                                 }
// =================================================================
// FICHIER JS PARTIE 2/2 : gnss-dashboard-part2.js (FINAL)
// Astro, Rendu, Météo (N/A) et Logique de Synchro (Vérifié 2025-11-02)
// =================================================================

// Les constantes et $(id) sont définies dans part1.js

// --- FONCTIONS DE SUPPORT ---

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

// --- FONCTION DE SYNCHRONISATION DU TEMPS NTP ---

function syncTimeWithServer() {
    window.serverTimeOffset = 0; // Utilise l'heure client (pas de simulation NTP)
        
    const now = getCorrectedDate();
    const time_str = now.toLocaleTimeString('en-US', { hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false });
    
    if ($('local-time')) $('local-time').textContent = `${time_str} (${now.toLocaleTimeString('en-US', {timeZoneName: 'short', hour12: false}).split(' ')[1]})`;
    if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
    if ($('date-display-astro')) $('date-display-astro').textContent = now.toLocaleDateString('fr-FR');
}

// --- FONCTIONS ASTRO (TST/EOT) ---

function getSolarTime(date, lon) {
    // Calculs TST/EOT (même logique que précédemment)
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

// --- FONCTIONS RENDU 2D (CADRAN TST DYNAMIQUE) ---

function initSolarClock() {
    const container = $('minecraft-clock');
    if (container) {
        const canvas = document.createElement('canvas');
        canvas.id = 'mc-clock-canvas';
        // Taille réduite pour correspondre au style CSS
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
    
    // Efface le cadran précédent
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // --- 1. Calcul de l'illumination et de l'environnement ---
    const elevation = sunPos.altitude; // en radians

    // Luminosité du jour (-0.3: crépuscule/aube civils, 0.0: horizon, 1.0: zénith)
    const dayFactor = Math.min(1, Math.max(0, (elevation / D2R + 6) / 40)); 
    
    // Couleur du Ciel (de nuit au bleu vif)
    const skyColorR = Math.round(30 + 170 * dayFactor);
    const skyColorG = Math.round(50 + 150 * dayFactor);
    const skyColorB = Math.round(100 + 155 * dayFactor);
    const skyColor = `rgb(${skyColorR}, ${skyColorG}, ${skyColorB})`;

    // Couleur du Sol (Simulée pour être Mer/Plaine près de Marseille)
    const waterColorR = Math.round(20 + 40 * dayFactor);
    const waterColorG = Math.round(40 + 70 * dayFactor);
    const waterColorB = Math.round(60 + 90 * dayFactor);
    const groundColor = `rgb(${waterColorR}, ${waterColorG}, ${waterColorB})`;


    // --- 2. Rendu de l'environnement (Cadran) ---

    // Moitié supérieure : Ciel 
    ctx.beginPath();
    ctx.arc(center, center, radius, -Math.PI, 0); 
    ctx.lineTo(center + radius, center);
    ctx.fillStyle = skyColor;
    ctx.fill();
    
    // Moitié inférieure : Sol/Mer
    ctx.beginPath();
    ctx.arc(center, center, radius, 0, Math.PI); 
    ctx.lineTo(center - radius, center);
    ctx.fillStyle = groundColor;
    ctx.fill();

    // Ligne d'horizon
    ctx.strokeStyle = '#333';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(center - radius, center);
    ctx.lineTo(center + radius, center);
    ctx.stroke();

    // --- 3. Simulation Météo/Lumière ---
    // Représente une légère brume ou couverture nuageuse (Couverture Nuageuse = 10%)
    const cloudOpacity = 0.1; 
    if (cloudOpacity > 0) {
        ctx.fillStyle = `rgba(180, 180, 180, ${cloudOpacity * (1 - dayFactor * 0.5)})`;
        ctx.beginPath();
        ctx.arc(center, center, radius, -Math.PI, Math.PI);
        ctx.fill();
    }


    // --- 4. Rendu de l'AIGUILLE TST ---
    const total_seconds = window.mcTime / 1000;
    const angle_from_north = (total_seconds / 86400) * 2 * Math.PI;
    const TST_ANGLE = angle_from_north - Math.PI; // 00:00:00 = Nord (Bas), 12:00:00 = Sud (Haut)

    ctx.save();
    ctx.translate(center, center);
    ctx.rotate(TST_ANGLE); 

    ctx.strokeStyle = '#D9534F'; 
    ctx.lineWidth = 4;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(0, -radius * 0.9); 
    ctx.stroke();

    ctx.restore();

    // --- 5. Rendu du TST (Heure au centre) ---
    ctx.fillStyle = '#000';
    ctx.font = `${center * 0.25}px Arial bold`; 
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    const TST_time = new Date(window.mcTime).toLocaleTimeString('en-US', { hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false, timeZone: 'UTC' });
    ctx.fillText(TST_time, center, center); 
}

// --- FONCTIONS MÉTÉO ET DONNÉES EXTERNES (N/A) ---

function fetchExternalData() {
    // Tous les champs Météo, Chimie, Capteurs sont mis à N/A (absence d'API)
    [
        'temp-air', 'pressure', 'humidity', 'dew-point', 'visibility', 
        'uv-index', 'wind-direction', 'precipitation-rate', 'temp-feels-like',
        'magnetic-field', 'air-flow', 'o2-level', 'co2-level', 'air-density',
        'ozone-conc', 'ph-level', 'solar-radiation', 'noise-level', 'wind-speed-ms',
        'soil-type', 'ndvi-index', 'lsm', 'noon-solar', 'day-duration', 'ecliptic-long'
    ].forEach(id => {
        if ($(id) && $(id).textContent.startsWith('N/A') === false && $(id).textContent.startsWith('0') === false) $(id).textContent = `N/A`;
    });
    
    // Calculs basés sur les données internes
    const filteredSpeed = window.EKFState[2];
    const kineticEnergy = 0.5 * window.mass * filteredSpeed * filteredSpeed;
    if ($('kinetic-energy')) $('kinetic-energy').textContent = `${kineticEnergy.toFixed(2)} J`;
    if ($('mechanical-power')) $('mechanical-power').textContent = `0.00 W`;
}


// --- FONCTIONS CARTE/RENDU/UPDATE ---

function initMap() {
    const defaultLat = 43.296482; 
    const defaultLon = 5.36978;
    
    if ($('map-container')) {
        if (window.lMap) window.lMap.remove(); 
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

function updateAstro(latA, lonA) {
    const now = getCorrectedDate(); 
    if (now === null || !window.SunCalc) return; 

    const sunPos = window.SunCalc.getPosition(now, latA, lonA);
    const solarTimes = getSolarTime(now, lonA); 
    window.mcTime = solarTimes.TST_MS; 

    // Mise à jour de l'affichage
    if ($('tst')) $('tst').textContent = solarTimes.TST;
    if ($('eot')) $('eot').textContent = `${solarTimes.EOT} min`;
    if ($('sun-elevation')) $('sun-elevation').textContent = (sunPos.altitude / D2R).toFixed(2) + " °";
    if ($('time-minecraft')) $('time-minecraft').textContent = solarTimes.TST;
    if ($('clock-status')) $('clock-status').textContent = `TST: ${solarTimes.TST} (Élév: ${(sunPos.altitude / D2R).toFixed(2)}°)`;
    
    // Élément "Longitude écliptique du Soleil" (lambda)
    if ($('ecliptic-long')) $('ecliptic-long').textContent = (solarTimes.lambda / D2R).toFixed(2) + " °";

    drawSolarClock(sunPos); 
}


// --- CONTRÔLES DU FLUX ---

let domID = null; 

function startMainLoop() {
    if (domID === null) {
        domID = setInterval(() => {
            const currentLat = window.lPos ? window.lPos.coords.latitude : 43.296482; 
            const currentLon = window.lPos ? window.lPos.coords.longitude : 5.36978;
            
            updateAstro(currentLat, currentLon); 
            updatePhysicsAndChemistry(); 
            if(window.isGPSRunning) updateMap(); 

            // SYNCHRO NTP : toutes les 5 minutes (300000ms)
            if (Date.now() % 300000 < window.currentFreqMS) { 
                syncTimeWithServer(); 
            }
            // Mise à jour des données externes (N/A ou Calculables) toutes les 10 secondes
            if (Date.now() % 10000 < window.currentFreqMS) { 
                fetchExternalData(); 
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
        
        // Redémarre l'intervalle principal à la nouvelle fréquence
        if (domID !== null) {
            clearInterval(domID);
            domID = null;
        }
        startMainLoop();
    }
}

function emergencyStop() {
    if (domID !== null) {
        clearInterval(domID);
        domID = null;
    }
    stopGPS(); 
    
    if ($('emergency-stop-btn')) {
        $('emergency-stop-btn').textContent = '🛑 Arrêt d’urgence: ACTIF 🔴';
        $('emergency-stop-btn').style.backgroundColor = '#dc3545';
        $('emergency-stop-btn').style.fontWeight = 'bold';
    }
}

function toggleNightMode() {
    document.body.classList.toggle('night-mode');
    const isNight = document.body.classList.contains('night-mode');
    if ($('toggle-mode-btn')) {
        $('toggle-mode-btn').textContent = isNight ? '☀️ Mode Jour' : '🌓 Mode Nuit';
    }
    // Ajouté pour garantir que la couleur du texte est visible sur le fond de nuit
    document.body.style.color = isNight ? '#f4f4f4' : '#333';
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
    console.log("---------------------------------------");
}

// --- INITIALISATION GLOBALE (ENTRY POINT) ---

document.addEventListener('DOMContentLoaded', () => {
    
    if (!window.SunCalc) {
        console.error("Erreur: SunCalc.js n'est pas chargé.");
        return;
    }
    
    initMap(); 
    initSolarClock(); 
    initializeControlListeners(); // Défini dans part1.js
    
    startGPS(); 
    startIMU(); 
    syncTimeWithServer(); 
    fetchExternalData(); 

    // Attachement des fonctions de contrôle spécifiques à la boucle
    if ($('freq-select')) $('freq-select').addEventListener('change', setFrequency);
    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', emergencyStop);
    if ($('toggle-mode-btn')) $('toggle-mode-btn').addEventListener('click', toggleNightMode);
    if ($('data-capture-btn')) $('data-capture-btn').addEventListener('click', captureData);

    // Démarrage de la boucle de rafraîchissement
    startMainLoop(); 
});
