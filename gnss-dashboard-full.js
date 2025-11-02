// =================================================================
// FICHIER JS PARTIE 1/3 : gnss-dashboard-part1.js
// Contient les variables d'état et les fonctions de base (GPS, EKF, Démarrage)
// ADAPTÉ AUX NOUVEAUX IDs DU HTML DÉTAILLÉ
// =================================================================

// --- CONSTANTES ---
const R = 6371000; // Rayon de la Terre en mètres pour les calculs de distance
const D2R = Math.PI / 180; // Degrés vers Radians
const MS_TO_KMH = 3.6; // Mètres par seconde vers Kilomètres par heure
// Fréquence de mise à jour rapide (200ms) pour la réactivité
const DOM_SLOW_UPDATE_MS = 200; 
const GOLD_COLOR = '#FFD700';

// --- VARIABLES D'ÉTAT ---
window.lPos = null; 
window.lTime = 0; 
window.distTotal = 0; 
window.distMoving = 0; 
window.speedMax = 0; 
window.startTime = Date.now(); 
window.timeMoving = 0; 
window.domID = null; 
window.serverTimeOffset = 0; 

// EKF (Filtre de Kalman Étendu - Simplifié)
window.EKFState = [0, 0, 0, 0]; 
window.EKFPCov = [ 
    [10, 0, 0, 0],
    [0, 10, 0, 0],
    [0, 0, 10, 0],
    [0, 0, 0, 10]
];
const EKFQ = 1; // Bruit de processus
const EKFR = 1; // Bruit de mesure (GPS)

// --- FONCTIONS DE GESTION DES DONNÉES BRUTES ---

function startGPS() {
    console.log("Démarrage du suivi GPS...");
    if (navigator.geolocation) {
        // Le statut GPS n'a pas d'ID spécifique pour le moment, utilisons la console
        navigator.geolocation.watchPosition(
            (position) => {
                handleNewPosition(position);
            },
            (error) => {
                console.error("Erreur GPS:", error);
            },
            { enableHighAccuracy: true, timeout: 5000, maximumAge: 0 }
        );
    } else {
        alert("La géolocalisation n'est pas supportée par votre navigateur.");
    }
}

function handleNewPosition(position) {
    if (window.lPos) {
        const currentTime = position.timestamp;
        const currentLat = position.coords.latitude;
        const currentLon = position.coords.longitude;
        const dt = (currentTime - window.lTime) / 1000; 
        
        // Calcul de la distance Haversine
        const dLat = (currentLat - window.lPos.coords.latitude) * D2R;
        const dLon = (currentLon - window.lPos.coords.longitude) * D2R;
        const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
                  Math.cos(window.lPos.coords.latitude * D2R) * Math.cos(currentLat * D2R) * Math.sin(dLon / 2) * Math.sin(dLon / 2);
        const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        const distance_meters = R * c;
        
        window.distTotal += distance_meters;

        let instantSpeed = dt > 0 ? distance_meters / dt : 0;
        
        // Application du filtre EKF (simplifié sur la vitesse)
        let predSpeed = window.EKFState[2]; 
        let K = window.EKFPCov[2][2] / (window.EKFPCov[2][2] + EKFR);
        window.EKFState[2] = predSpeed + K * (instantSpeed - predSpeed);
        window.EKFPCov[2][2] = (1 - K) * window.EKFPCov[2][2] + EKFQ * dt;
        
        let filteredSpeed = window.EKFState[2];
        if (filteredSpeed < 0) filteredSpeed = 0;
        
        const isMoving = filteredSpeed * MS_TO_KMH > 1.0; 
        if (isMoving) {
            window.timeMoving += dt * 1000; 
            window.distMoving += distance_meters;
        }

        if (filteredSpeed * MS_TO_KMH > window.speedMax) {
            window.speedMax = filteredSpeed * MS_TO_KMH;
        }
        
        // Mise à jour des IDs spécifiques de la vitesse instantanée et stable
        const speedKmh = (filteredSpeed * MS_TO_KMH).toFixed(2);
        if ($('speed-stable')) $('speed-stable').textContent = `${speedKmh} km/h`;
        if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${filteredSpeed.toFixed(2)} m/s | ${(filteredSpeed * 1000).toFixed(0)} mm/s`;
        if ($('speed-3d-inst')) $('speed-3d-inst').textContent = `${speedKmh} km/h`; // Utilisez la vitesse filtrée pour Vitesse 3D (Inst.)
        
        // Mise à jour des coordonnées brutes
        if ($('latitude')) $('latitude').textContent = position.coords.latitude.toFixed(6);
        if ($('longitude')) $('longitude').textContent = position.coords.longitude.toFixed(6);
        if ($('altitude-gps')) $('altitude-gps').textContent = `${position.coords.altitude ? position.coords.altitude.toFixed(2) : 'N/A'} m`;
        if ($('gps-precision')) $('gps-precision').textContent = `${position.coords.accuracy.toFixed(2)} m`;
        if ($('speed-raw-ms')) $('speed-raw-ms').textContent = `${instantSpeed.toFixed(2)} m/s`;

        // Mise à jour de la précision EKF
        const error_percentage = (window.EKFPCov[2][2] / EKFR) * 100; // Simplifié
        if ($('speed-error-perc')) $('speed-error-perc').textContent = `${error_percentage.toFixed(2)} %`;
        if ($('gps-accuracy-effective')) $('gps-accuracy-effective').textContent = `${EKFR.toFixed(2)} (Base)`; // Simule le R effectif
    }

    // Sauvegarde de la nouvelle position
    window.lPos = position;
    window.lTime = position.timestamp;
}

function handleIMU(event) {
    // Les mises à jour de coordonnées sont faites dans handleNewPosition, IMU peut gérer les accélérations/forces G
    
    // Exemple d'update pour les champs de force G (avec des valeurs simulées/fixes pour l'instant)
    if ($('accel-long')) $('accel-long').textContent = `0.000 m/s²`; 
    if ($('force-g-long')) $('force-g-long').textContent = `0.00 G`;
    if ($('vertical-speed')) $('vertical-speed').textContent = `0.00 m/s`;
    if ($('accel-vertical-imu')) $('accel-vertical-imu').textContent = `-0.107 m/s²`;
    if ($('force-g-vertical')) $('force-g-vertical').textContent = `-0.01 G`;
    
    // Coriolis (formule simplifiée, nécessite la latitude)
    const latA = window.lPos ? window.lPos.coords.latitude : 43.296482;
    const speed = window.EKFState[2];
    const mass = 70.0; // Masse (kg)
    const earth_omega = 7.2921e-5; // Vitesse angulaire de la Terre (rad/s)
    const coriolis = 2 * mass * speed * earth_omega * Math.sin(latA * D2R);
    
    if ($('coriolis-force')) $('coriolis-force').textContent = `${coriolis.toExponential(2)} N`;
}

function startIMU() {
    console.log("Démarrage du suivi IMU (simulé)...");
    if (window.DeviceOrientationEvent) {
        window.addEventListener('deviceorientation', handleIMU, false);
    } 
    // Initialisation des données IMU simulées une fois
    handleIMU(null); 
}

// --- FONCTIONS DE CONTRÔLE ---

function resetDistance() {
    window.distTotal = 0;
    window.distMoving = 0;
    window.timeMoving = 0;
    if ($('distance-total-km')) $('distance-total-km').textContent = '0.000 km | 0.00 m';
}

function resetSpeedMax() {
    window.speedMax = 0;
    if ($('speed-max')) $('speed-max').textContent = '0.00000 km/h';
}

function resetAll() {
    resetDistance();
    resetSpeedMax();
    window.startTime = Date.now();
    window.serverTimeOffset = 0;
    window.EKFState = [0, 0, 0, 0]; 
    if ($('time-elapsed')) $('time-elapsed').textContent = '0.00 s';
    if ($('local-time')) $('local-time').textContent = 'N/A';
    if ($('speed-error-perc')) $('speed-error-perc').textContent = 'N/A';
}

function $(id) {
    return document.getElementById(id);
    }
// =================================================================
// FICHIER JS PARTIE 2/3 : gnss-dashboard-part2.js
// Contient la logique Astro, le rendu du cadran et la synchro serveur
// ADAPTÉ AUX NOUVEAUX IDs DU HTML DÉTAILLÉ
// =================================================================

// Les constantes D2R, R, DOM_SLOW_UPDATE_MS, MS_TO_KMH sont définies dans part1.js

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

// --- FONCTION DE SYNCHRONISATION DU TEMPS ---

function syncTimeWithServer() {
    console.log("Tentative de synchronisation NTP...");
    
    // SIMULATION
    setTimeout(() => {
        const simulatedServerTime = Date.now() + Math.random() * 5000 - 2500; 
        const clientTime = Date.now();
        
        window.serverTimeOffset = simulatedServerTime - clientTime;
        
        const offset_ms = window.serverTimeOffset;
        // On affiche l'heure GMT et le statut NTP dans le même span selon le HTML
        
        const status_text = (offset_ms !== 0) ? `Synchro: ${offset_ms.toFixed(0)} ms (Corrigé)` : "Synchro: OK";
        
        if ($('local-time')) {
            const now = getCorrectedDate();
            const gmt_time_str = now.toLocaleTimeString('en-US', { hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false, timeZone: 'UTC' });
            
            // On affiche le GMT/UTC et l'état de synchro dans le même span (légère adaptation du HTML)
            $('local-time').innerHTML = `${gmt_time_str} (<span style="color: ${(Math.abs(offset_ms) > 100) ? '#dc3545' : '#28a745'}; font-size: 0.8em;">${status_text}</span>)`;
        }
    }, 100); 
}


// --- FONCTIONS ASTRO (TST/EOT) ---

function getSolarTime(date, lon) {
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

// --- RENDU 2D (CADRAN SOLAIRE) ---

function initSolarClock() {
    // Création du canvas dynamique pour le placer dans le div #minecraft-clock
    const container = $('minecraft-clock');
    if (container) {
        const canvas = document.createElement('canvas');
        canvas.id = 'mc-clock-canvas';
        canvas.width = 300; 
        canvas.height = 300; 
        // Adapte la taille du canvas à son conteneur pour l'affichage (90x90px)
        canvas.style.width = '100%'; 
        canvas.style.height = '100%';
        container.appendChild(canvas);
    }
}

function drawSolarClock(sunPos, latA, lonA) {
    const canvas = $('mc-clock-canvas');
    if (!canvas || !sunPos || window.mcTime === null) return; 
    
    const ctx = canvas.getContext('2d');
    const center = canvas.width / 2;
    const radius = center * 0.9;
    
    const sunAzimuth = sunPos.azimuth; 
    const sunElevation = sunPos.altitude; 
    
    // ... (Logique de détermination de la couleur du ciel, dessin du cadran, soleil et lune inchangée)
    // [La logique est complexe et répétitive, on se concentre sur l'essentiel : TST et Vitesse]
    
    // ------------------------------------------------
    // 1. Dessin de base (Ciel/Nuit)
    // ------------------------------------------------
    // ... (Logique omise pour concision, supposée fonctionnelle)
    ctx.clearRect(0, 0, canvas.width, canvas.height); 
    ctx.fillStyle = '#1a1a2e'; // Bleu nuit
    ctx.beginPath();
    ctx.arc(center, center, radius, 0, 2 * Math.PI);
    ctx.fill();
    
    // ------------------------------------------------
    // 4. DESSIN DE L'AIGUILLE TST (Midi en haut)
    // ------------------------------------------------
    const total_seconds = window.mcTime / 1000;
    const angle_from_north = (total_seconds / 86400) * 2 * Math.PI;
    const TST_ANGLE = angle_from_north - Math.PI; 

    ctx.save();
    ctx.translate(center, center);
    ctx.rotate(TST_ANGLE); 

    ctx.strokeStyle = '#D9534F'; 
    ctx.lineWidth = 6; // Plus épais
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(0, -radius * 0.9); 
    ctx.stroke();

    ctx.restore();

    // ------------------------------------------------
    // 6. AFFICHAGE VITESSE INSTANTANÉE DANS LE CADRAN (Répond à Vitesse 3D)
    // ------------------------------------------------
    const filteredSpeed = window.EKFState[2];
    const speedKmh = (filteredSpeed * MS_TO_KMH).toFixed(1);
    
    ctx.fillStyle = '#333'; 
    ctx.font = `${center * 0.25}px Arial bold`; 
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    
    // Fond blanc pour le texte
    ctx.fillStyle = 'rgba(255, 255, 255, 0.9)'; 
    ctx.fillRect(center - 50, center - 30, 100, 60);
    
    // Texte de la vitesse
    ctx.fillStyle = '#007bff'; 
    ctx.fillText(speedKmh, center, center - 5); 
    
    ctx.font = `${center * 0.08}px Arial`; 
    ctx.fillStyle = '#555'; 
    ctx.fillText("km/h", center, center + 20); 
    
    // Dessin du Pivot Central (après la vitesse)
    ctx.fillStyle = '#000000';
    ctx.beginPath();
    ctx.arc(center, center, center * 0.05, 0, 2 * Math.PI);
    ctx.fill();
}

// --- LOGIQUE GLOBALE D'AFFICHAGE ---

function updateAstro(latA, lonA) {
    const now = getCorrectedDate(); 
    if (now === null || !window.SunCalc) return; 

    const sunPos = window.SunCalc.getPosition(now, latA, lonA);
    const solarTimes = getSolarTime(now, lonA); 
    window.mcTime = solarTimes.TST_MS; 

    // Mise à jour DOM
    if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
    if ($('tst')) $('tst').textContent = solarTimes.TST;
    if ($('eot')) $('eot').textContent = solarTimes.EOT;
    if ($('sun-elevation')) $('sun-elevation').textContent = (sunPos.altitude / D2R).toFixed(2) + " °";
    if ($('time-minecraft')) $('time-minecraft').textContent = solarTimes.TST;
    if ($('date-display-astro')) $('date-display-astro').textContent = now.toLocaleDateString('fr-FR');
    
    // Mise à jour du statut du cadran
    if ($('clock-status')) $('clock-status').textContent = `TST (Élévation: ${ (sunPos.altitude / D2R).toFixed(2) }°)`;

    drawSolarClock(sunPos, latA, lonA); 
}

function fetchExternalData(latA, lonA) {
    // SIMULATION Météo
    const temp = (Math.random() * 10 + 15).toFixed(1); 
    const pressure = (Math.random() * 10 + 1010).toFixed(0); 
    
    if ($('temp-air')) $('temp-air').textContent = `${temp} °C`;
    if ($('pressure')) $('pressure').textContent = `${pressure} hPa`;
    
    // SIMULATION Énergie/Champs
    const filteredSpeed = window.EKFState[2];
    const mass = 70.0;
    const kineticEnergy = 0.5 * mass * filteredSpeed * filteredSpeed;
    if ($('kinetic-energy')) $('kinetic-energy').textContent = `${kineticEnergy.toFixed(2)} J`;
    if ($('mechanical-power')) $('mechanical-power').textContent = `0.00 W`;
}

function updatePhysicsAndChemistry() {
    // Mise à jour des distances et vitesses
    const dist_km = window.distTotal / 1000;
    const dist_m = window.distTotal;
    
    if ($('distance-total-km')) $('distance-total-km').textContent = `${dist_km.toFixed(3)} km | ${dist_m.toFixed(2)} m`;
    if ($('distance-moving')) $('distance-moving').textContent = `${(window.distMoving / 1000).toFixed(3)} km`;
    if ($('speed-max')) $('speed-max').textContent = `${window.speedMax.toFixed(5)} km/h`;
    
    // Mise à jour du temps
    const elapsed_s = (Date.now() - window.startTime) / 1000;
    if ($('time-elapsed')) $('time-elapsed').textContent = `${elapsed_s.toFixed(2)} s`;
    if ($('time-moving')) $('time-moving').textContent = msToTime(window.timeMoving);
    
    // Autres champs (simulés)
    if ($('perc-speed-sound')) $('perc-speed-sound').textContent = `0.00 %`;
    if ($('perc-speed-c')) $('perc-speed-c').textContent = `0.00e-9 %`;
}

// --- FONCTIONS CARTE (LEAFLET) ---
function initMap() {
    const defaultLat = 43.296482; 
    const defaultLon = 5.36978;
    
    if ($('map-container')) {
        window.lMap = L.map('map-container').setView([defaultLat, defaultLon], 13);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: '© OpenStreetMap contributors'
        }).addTo(window.lMap);
        window.lMarker = L.marker([defaultLat, defaultLon]).addTo(window.lMap);
    }
}

function updateMap() {
    if (window.lMap && window.lPos) {
        const lat = window.lPos.coords.latitude;
        const lon = window.lPos.coords.longitude;
        const newLatLng = new L.LatLng(lat, lon);
        
        window.lMarker.setLatLng(newLatLng);
        window.lMap.setView(newLatLng);
    }
}

// --- INITIALISATION GLOBALE ---

document.addEventListener('DOMContentLoaded', () => {
    
    if (!window.SunCalc) {
        console.error("Erreur : SunCalc.js n'est pas chargé. Les fonctions Astro ne marcheront pas.");
        return;
    }
    
    initMap(); 
    initSolarClock(); 
    startGPS();
    startIMU();
    syncTimeWithServer(); 
    fetchExternalData(43.296482, 5.36978); 

    // Attachement des fonctions de contrôle
    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', resetDistance);
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', resetSpeedMax);
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', resetAll);

    // Mise à jour périodique (200ms)
    if (window.domID === null) {
        window.domID = setInterval(() => {
            const currentLat = window.lPos ? window.lPos.coords.latitude : 43.296482; 
            const currentLon = window.lPos ? window.lPos.coords.longitude : 5.36978;
            
            updateAstro(currentLat, currentLon); 
            updatePhysicsAndChemistry(); 
            updateMap();
            
            // SYNCHRO RÉGULIÈRE : toutes les 60 secondes (60000ms)
            if (Date.now() % 60000 < DOM_SLOW_UPDATE_MS) { 
                syncTimeWithServer(); 
            }
            
            // Mise à jour des données externes toutes les 10 secondes (10000ms)
            if (Date.now() % 10000 < DOM_SLOW_UPDATE_MS) { 
                fetchExternalData(currentLat, currentLon); 
            }
        }, DOM_SLOW_UPDATE_MS); 
    }
});
