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
    if (window.lPos) {
        const dt = (position.timestamp - window.lTime) / 1000; 
        
        // Calcul Haversine (distance brute)
        const dLat = (position.coords.latitude - window.lPos.coords.latitude) * D2R;
        const dLon = (position.coords.longitude - window.lPos.coords.longitude) * D2R;
        const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) + Math.cos(window.lPos.coords.latitude * D2R) * Math.cos(position.coords.latitude * D2R) * Math.sin(dLon / 2) * Math.sin(dLon / 2);
        const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        let distance_meters = R * c;
        
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
        
        // Mise à jour des métriques de mouvement
        const isMoving = filteredSpeed * MS_TO_KMH > 1.0; 
        if (isMoving) window.timeMoving += dt * 1000; 
        if (filteredSpeed * MS_TO_KMH > window.speedMax) window.speedMax = filteredSpeed * MS_TO_KMH;
        
        // --- AFFICHAGE ---
        const filteredSpeedKmh = (filteredSpeed * MS_TO_KMH).toFixed(2);
        if ($('speed-stable')) $('speed-stable').textContent = `${filteredSpeedKmh} km/h`;
        if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${filteredSpeed.toFixed(2)} m/s | ${(filteredSpeed * 1000).toFixed(0)} mm/s`;

        let rawSpeed = position.coords.speed !== null ? position.coords.speed * scale : instantSpeed;
        if (rawSpeed < 0) rawSpeed = 0;
        if ($('speed-3d-inst')) {
             $('speed-3d-inst').textContent = (rawSpeed * MS_TO_KMH > 0.01) ? `${(rawSpeed * MS_TO_KMH).toFixed(1)} km/h` : `-- km/h`;
        }
        
        // Mise à jour de la précision EKF et GPS
        const error_percentage = (window.EKFPCov[2][2] / R_effective) * 100;
        if ($('speed-error-perc')) $('speed-error-perc').textContent = `${error_percentage.toFixed(2)} %`;
        if ($('gps-accuracy-effective')) $('gps-accuracy-effective').textContent = `${R_effective.toFixed(4)} m (EKF R)`;
        
        // Mise à jour Nether Mode Display
        if ($('mode-nether')) $('mode-nether').textContent = window.isNetherMode ? `ACTIVÉ (1:${NETHER_SCALE})` : 'DÉSACTIVÉ (1:1)';

        // Coordonnées brutes
        if ($('latitude')) $('latitude').textContent = position.coords.latitude.toFixed(6);
        if ($('longitude')) $('longitude').textContent = position.coords.longitude.toFixed(6);
        if ($('altitude-gps')) $('altitude-gps').textContent = `${position.coords.altitude ? position.coords.altitude.toFixed(2) : 'N/A'} m`;
        if ($('gps-precision')) $('gps-precision').textContent = `${position.coords.accuracy.toFixed(2)} m`;
        if ($('speed-raw-ms')) $('speed-raw-ms').textContent = `${(distance_meters / dt).toFixed(2)} m/s`;
    }

    // Sauvegarde de la nouvelle position
    window.lPos = position;
    window.lTime = position.timestamp;
}

function handleIMU(event) {
    const latA = window.lPos ? window.lPos.coords.latitude : 43.296482;
    const speed = window.EKFState[2];
    const mass = 70.0; 
    
    const coriolis = 2 * mass * speed * EARTH_OMEGA * Math.sin(latA * D2R);
    if ($('coriolis-force')) $('coriolis-force').textContent = `${coriolis.toExponential(2)} N`;

    const accIncGrav = event && event.accelerationIncludingGravity;
    if ($('accel-vertical-imu')) $('accel-vertical-imu').textContent = accIncGrav && accIncGrav.z ? `${accIncGrav.z.toFixed(3)} m/s²` : `N/A`;
    if ($('force-g-vertical')) $('force-g-vertical').textContent = accIncGrav && accIncGrav.z ? `${(accIncGrav.z / G).toFixed(2)} G` : `N/A`;
    
    if ($('accel-long')) $('accel-long').textContent = `N/A`;
    if ($('force-g-long')) $('force-g-long').textContent = `N/A`;
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
    if ($('nether-toggle-btn')) {
        $('nether-toggle-btn').textContent = window.isNetherMode ? 'Nether Mode (ON)' : 'Nether Mode';
    }
    if ($('mode-nether')) $('mode-nether').textContent = window.isNetherMode ? `ACTIVÉ (1:${NETHER_SCALE})` : 'DÉSACTIVÉ (1:1)';
    console.log("Nether Mode:", window.isNetherMode ? 'ON' : 'OFF');
}

function updateKalmanParameters() {
    // 1. Facteur d'environnement (affecte Q et R)
    const envSelect = $('environment-select');
    if (envSelect) {
        const selectedOption = envSelect.options[envSelect.selectedIndex].text;
        const factorMatch = selectedOption.match(/\(x([\d.]+)\)/);
        const factor = factorMatch ? parseFloat(factorMatch[1]) : 1.0;
        window.environmentFactor = isNaN(factor) ? 1.0 : factor;
    }

    // 2. Précision GPS forcée (affecte R directement si > 0)
    const accOverride = $('gps-accuracy-override');
    if (accOverride) {
        const overrideValue = parseFloat(accOverride.value);
        window.kalmanR = overrideValue > 0 ? overrideValue : EKFR; 
    }
    console.log(`EKF Params mis à jour: R=${window.kalmanR.toFixed(2)}, Env Factor=${window.environmentFactor.toFixed(1)}`);
}

function resetDistance() {
    window.distTotal = 0;
    window.timeMoving = 0;
    if ($('distance-total-km')) $('distance-total-km').textContent = '0.000 km | 0.00 m';
    if ($('time-moving')) $('time-moving').textContent = '00:00:00';
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
    if ($('local-time')) $('local-time').innerHTML = `N/A UTC (<span style="color: #6c757d; font-weight: normal; font-size: 0.8em;">Synchro: Client Time</span>)`;
    if ($('speed-error-perc')) $('speed-error-perc').textContent = 'N/A';
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
// FICHIER JS PARTIE 2/2 : gnss-dashboard-part2.js (FINAL - Corrigé)
// Astro (TST Dynamique), Rendu, Météo (N/A) et Logique de Synchro
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
    // Utilise l'heure locale, car l'offset NTP est fixé à 0 (pas de simulation)
    return new Date(Date.now() + window.serverTimeOffset); 
}

// --- FONCTION DE SYNCHRONISATION DU TEMPS NTP ---

function syncTimeWithServer() {
    console.log("Synchronisation NTP (Client Time)...");
    
    window.serverTimeOffset = 0; // Utilise l'heure client (pas de simulation NTP)
        
    if ($('local-time')) {
        const now = getCorrectedDate();
        const gmt_time_str = now.toLocaleTimeString('en-US', { hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false, timeZone: 'UTC' });
        
        // Affichage de l'heure UTC et du statut (Synchro: Client Time)
        $('local-time').innerHTML = `${gmt_time_str} UTC (<span style="color: #6c757d; font-weight: normal; font-size: 0.8em;">Synchro: Client Time</span>)`;
    }
}


// --- FONCTIONS ASTRO (TST/EOT) ---

function getSolarTime(date, lon) {
    // RAPPEL: D2R (Degrés vers Radians) est défini dans part1.js
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
    
    return { EOT_MS, EOT, TST_MS, TST };
}

// --- FONCTIONS RENDU 2D (CADRAN TST DYNAMIQUE) ---

function initSolarClock() {
    const container = $('minecraft-clock');
    if (container) {
        const canvas = document.createElement('canvas');
        canvas.id = 'mc-clock-canvas';
        canvas.width = 300; 
        canvas.height = 300; 
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
    
    // --- 1. Calcul de l'illumination et de l'environnement ---
    const elevation = sunPos.altitude; // en radians
    const D2R = Math.PI / 180; 

    // Luminosité du jour (0.0 au milieu de la nuit à 1.0 au zénith)
    // Civils Twilight (~-6 degrés) est le début/fin de l'éclairage.
    const dayFactor = Math.min(1, Math.max(0, (elevation / D2R + 6) / 40)); 
    
    // Couleur du Ciel (de nuit au bleu vif)
    const skyColorR = Math.round(30 + 170 * dayFactor);
    const skyColorG = Math.round(50 + 150 * dayFactor);
    const skyColorB = Math.round(100 + 155 * dayFactor);
    const skyColor = `rgb(${skyColorR}, ${skyColorG}, ${skyColorB})`;

    // Couleur du Sol/Mer (Plus foncé la nuit)
    let groundColor;
    
    // Simulation du sol: Couleur Mer/Plaine (Basé sur la latitude de Marseille par défaut)
    const waterColorR = Math.round(20 + 40 * dayFactor);
    const waterColorG = Math.round(40 + 70 * dayFactor);
    const waterColorB = Math.round(60 + 90 * dayFactor);
    groundColor = `rgb(${waterColorR}, ${waterColorG}, ${waterColorB})`;


    // --- 2. Rendu de l'environnement (Cadran) ---

    // Moitié supérieure : Ciel (Adapté à l'éclairage solaire)
    ctx.beginPath();
    ctx.arc(center, center, radius, -Math.PI, 0); 
    ctx.lineTo(center + radius, center);
    ctx.fillStyle = skyColor;
    ctx.fill();
    
    // Moitié inférieure : Sol/Mer (Adapté à l'éclairage solaire)
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
    // Représente une légère brume ou couverture nuageuse (météo réelle ferait effet ici)
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
    const TST_ANGLE = angle_from_north - Math.PI; 

    ctx.save();
    ctx.translate(center, center);
    ctx.rotate(TST_ANGLE); 

    ctx.strokeStyle = '#D9534F'; 
    ctx.lineWidth = 6;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(0, -radius * 0.9); 
    ctx.stroke();

    ctx.restore();

    // --- 5. Affichage VITESSE INSTANTANÉE (Centre) ---
    const rawSpeed = window.lPos && window.lPos.coords.speed !== null ? window.lPos.coords.speed : window.EKFState[2];
    const speedKmh = (rawSpeed * MS_TO_KMH).toFixed(1);
    
    ctx.fillStyle = `rgba(255, 255, 255, ${0.8 + dayFactor * 0.1})`; 
    ctx.fillRect(center - 55, center - 30, 110, 60);
    
    ctx.fillStyle = '#007bff'; 
    ctx.font = `${center * 0.25}px Arial bold`; 
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(rawSpeed * MS_TO_KMH > 0.01 ? speedKmh : '--', center, center - 5); 
    
    ctx.font = `${center * 0.08}px Arial`; 
    ctx.fillStyle = '#555'; 
    ctx.fillText("km/h", center, center + 20); 
}

// --- FONCTIONS MÉTÉO ET DONNÉES EXTERNES (N/A) ---

function fetchExternalData() {
    // Tous les champs Météo, Chimie, Capteurs sont mis à N/A (absence d'API)
    [
        'temp-air', 'pressure', 'humidity', 'dew-point', 'visibility', 
        'uv-index', 'wind-direction', 'precipitation-rate', 'temp-feels-like',
        'magnetic-field', 'air-flow', 'o2-level', 'co2-level', 'air-density',
        'ozone-conc', 'ph-level', 'solar-radiation', 'noise-level', 'wind-speed-ms',
        'soil-type', 'ndvi-index'
    ].forEach(id => {
        if ($(id)) $(id).textContent = `N/A`;
    });
    
    // Calculs basés sur les données internes
    const filteredSpeed = window.EKFState[2];
    const mass = 70.0;
    const kineticEnergy = 0.5 * mass * filteredSpeed * filteredSpeed;
    if ($('kinetic-energy')) $('kinetic-energy').textContent = `${kineticEnergy.toFixed(2)} J`;
    if ($('mechanical-power')) $('mechanical-power').textContent = `0.00 W`;
}


// --- FONCTIONS CARTE/RENDU/UPDATE ---
// (initMap, updateMap, updateAstro, updatePhysicsAndChemistry, drawSolarClock sont inchangées ici)

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
    if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
    if ($('tst')) $('tst').textContent = solarTimes.TST;
    if ($('eot')) $('eot').textContent = solarTimes.EOT;
    if ($('sun-elevation')) $('sun-elevation').textContent = (sunPos.altitude / D2R).toFixed(2) + " °";
    if ($('time-minecraft')) $('time-minecraft').textContent = solarTimes.TST;
    if ($('date-display-astro')) $('date-display-astro').textContent = now.toLocaleDateString('fr-FR');
    if ($('clock-status')) $('clock-status').textContent = `TST (Élévation: ${ (sunPos.altitude / D2R).toFixed(2) }°)`;

    drawSolarClock(sunPos); 
}

function updatePhysicsAndChemistry() {
    const dist_km = window.distTotal / 1000;
    const dist_m = window.distTotal;
    
    if ($('distance-total-km')) $('distance-total-km').textContent = `${dist_km.toFixed(3)} km | ${dist_m.toFixed(2)} m`;
    if ($('speed-max')) $('speed-max').textContent = `${window.speedMax.toFixed(5)} km/h`;
    
    const elapsed_s = (Date.now() - window.startTime) / 1000;
    if ($('time-elapsed')) $('time-elapsed').textContent = `${elapsed_s.toFixed(2)} s`;
    if ($('time-moving')) $('time-moving').textContent = msToTime(window.timeMoving);
    
    // Conversion cosmique
    const speedOfLight = 299792458; 
    const cosmic_s_light = window.distTotal / speedOfLight;
    const cosmic_al = cosmic_s_light / (3600 * 24 * 365.25); 
    
    if ($('distance-cosmic')) {
        $('distance-cosmic').textContent = `${cosmic_s_light.toExponential(2)} s lumière | ${cosmic_al.toExponential(2)} al`;
    }
}


// --- CONTRÔLES DU FLUX ---

let domID = null; // Variable pour l'intervalle 

function startMainLoop() {
    if (domID === null) {
        domID = setInterval(() => {
            const currentLat = window.lPos ? window.lPos.coords.latitude : 43.296482; 
            const currentLon = window.lPos ? window.lPos.coords.longitude : 5.36978;
            
            updateAstro(currentLat, currentLon); 
            updatePhysicsAndChemistry(); 
            if(window.isGPSRunning) updateMap(); 

            // SYNCHRO NTP : toutes les 5 minutes (300000ms) - CORRECTION APPORTÉE
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

    // Démarrage de la boucle de rafraîchissement (utilise la fréquence par défaut: 200ms)
    startMainLoop(); 
});
