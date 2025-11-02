// =================================================================
// FICHIER JS PARTIE 1/3 : gnss-dashboard-part1.js
// Contient les variables d'état et les fonctions de base (GPS, EKF, Démarrage)
// =================================================================

// --- CONSTANTES ---
const R = 6371000; // Rayon de la Terre en mètres pour les calculs de distance
const D2R = Math.PI / 180; // Degrés vers Radians
const MS_TO_KMH = 3.6; // Mètres par seconde vers Kilomètres par heure
const DOM_SLOW_UPDATE_MS = 1000; // Intervalle de mise à jour DOM (1 seconde)
const GOLD_COLOR = '#FFD700';

// --- VARIABLES D'ÉTAT ---
window.lPos = null; // Dernière position GPS (navigator.geolocation.getCurrentPosition)
window.lTime = 0; // Timestamp de la dernière position
window.distTotal = 0; // Distance totale parcourue
window.distMoving = 0; // Distance parcourue en mouvement
window.speedMax = 0; // Vitesse maximale
window.startTime = Date.now(); // Temps de démarrage du dashboard (heure locale brute)
window.timeMoving = 0; // Temps cumulé de mouvement
window.domID = null; // ID du setInterval pour la mise à jour DOM/Astro

// NOUVELLE VARIABLE : Décalage entre l'heure locale et l'heure serveur (millisecondes)
window.serverTimeOffset = 0; 

// VARIABLES EKF (Filtre de Kalman Étendu - Simplifié pour la vitesse)
// x: [position_x, position_y, vitesse_x, vitesse_y]
window.EKFState = [0, 0, 0, 0]; 
window.EKFPCov = [ // Matrice de covariance d'erreur simplifiée
    [10, 0, 0, 0],
    [0, 10, 0, 0],
    [0, 0, 10, 0],
    [0, 0, 0, 10]
];
const EKFQ = 1; // Bruit de processus (mouvement)
const EKFR = 1; // Bruit de mesure (GPS)

// --- FONCTIONS DE GESTION DES DONNÉES BRUTES ---

function startGPS() {
    console.log("Démarrage du suivi GPS...");
    if (navigator.geolocation) {
        navigator.geolocation.watchPosition(
            (position) => {
                handleNewPosition(position);
            },
            (error) => {
                console.error("Erreur GPS:", error);
                if ($('gps-status')) $('gps-status').textContent = `Erreur GPS: ${error.message}`;
            },
            { enableHighAccuracy: true, timeout: 5000, maximumAge: 0 }
        );
        if ($('gps-status')) $('gps-status').textContent = "GPS actif";
    } else {
        alert("La géolocalisation n'est pas supportée par votre navigateur.");
    }
}

function handleNewPosition(position) {
    if (window.lPos) {
        // Calcul des différences de temps et de distance
        const currentTime = position.timestamp;
        const currentLat = position.coords.latitude;
        const currentLon = position.coords.longitude;
        const dt = (currentTime - window.lTime) / 1000; // Différence de temps en secondes
        
        // Calcul de la distance (Formule de Haversine ou simplifiée)
        const dLat = (currentLat - window.lPos.coords.latitude) * D2R;
        const dLon = (currentLon - window.lPos.coords.longitude) * D2R;
        const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
                  Math.cos(window.lPos.coords.latitude * D2R) * Math.cos(currentLat * D2R) * Math.sin(dLon / 2) * Math.sin(dLon / 2);
        const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        const distance_meters = R * c;
        
        // Mise à jour de la distance totale
        window.distTotal += distance_meters;

        // Vitesse instantanée (m/s)
        let instantSpeed = dt > 0 ? distance_meters / dt : 0;
        
        // Application du filtre EKF (simplifié) pour lisser la vitesse
        // (Pour un EKF complet, les positions x et y devraient être dans l'état, ici on se concentre sur la vitesse)
        
        // 1. Prédiction (État = vitesse constante + bruit)
        let predSpeed = window.EKFState[2]; 
        
        // 2. Mise à jour (Correction par la mesure GPS)
        // Gain de Kalman (K)
        let K = window.EKFPCov[2][2] / (window.EKFPCov[2][2] + EKFR);
        // Mise à jour de l'état (vitesse)
        window.EKFState[2] = predSpeed + K * (instantSpeed - predSpeed);
        // Mise à jour de la covariance
        window.EKFPCov[2][2] = (1 - K) * window.EKFPCov[2][2] + EKFQ * dt;
        
        let filteredSpeed = window.EKFState[2];
        if (filteredSpeed < 0) filteredSpeed = 0; // La vitesse ne peut être négative
        
        // Mise à jour des métriques de mouvement
        const isMoving = filteredSpeed * MS_TO_KMH > 1.0; // Seuil de 1.0 km/h
        if (isMoving) {
            window.timeMoving += dt * 1000; // Ajoute le temps de déplacement en ms
            window.distMoving += distance_meters;
        }

        // Mise à jour de la vitesse max
        if (filteredSpeed * MS_TO_KMH > window.speedMax) {
            window.speedMax = filteredSpeed * MS_TO_KMH;
        }

        // Mise à jour de l'affichage de la vitesse instantanée
        if ($('instant-speed')) $('instant-speed').textContent = `${(filteredSpeed * MS_TO_KMH).toFixed(1)} km/h`;
    }

    // Sauvegarde de la nouvelle position pour la prochaine itération
    window.lPos = position;
    window.lTime = position.timestamp;
}

function handleIMU(event) {
    // Si la boussole ou l'accéléromètre était disponible:
    // const alpha = event.alpha; // Orientation (Boussole)
    // const gamma = event.gamma; // Inclinaison Latérale
    // const beta = event.beta; // Inclinaison Avant/Arrière
    
    // Simplification : Affichage de la position brute (si disponible)
    const latA = window.lPos ? window.lPos.coords.latitude : 43.296482; // Marseille par défaut
    const lonA = window.lPos ? window.lPos.coords.longitude : 5.36978; 
    
    if ($('coords')) $('coords').textContent = `Lat: ${latA.toFixed(4)}, Lon: ${lonA.toFixed(4)}`;
}

function startIMU() {
    console.log("Démarrage du suivi IMU (simulé)...");
    // Événements d'orientation du périphérique
    if (window.DeviceOrientationEvent) {
        window.addEventListener('deviceorientation', handleIMU, false);
    } else {
        console.warn("L'orientation du périphérique n'est pas supportée.");
    }
}

// --- FONCTIONS DE CONTRÔLE ---

function resetDistance() {
    window.distTotal = 0;
    window.distMoving = 0;
    window.timeMoving = 0;
    if ($('distance-total')) $('distance-total').textContent = '0.00 km';
    if ($('time-moving')) $('time-moving').textContent = '00:00:00';
}

function resetSpeedMax() {
    window.speedMax = 0;
    if ($('speed-max')) $('speed-max').textContent = '0.0 km/h';
}

function resetAll() {
    resetDistance();
    resetSpeedMax();
    window.startTime = Date.now();
    window.serverTimeOffset = 0;
    // Réinitialisation du filtre EKF
    window.EKFState = [0, 0, 0, 0]; 
    if ($('time-elapsed')) $('time-elapsed').textContent = '00:00:00';
    if ($('time-status')) $('time-status').textContent = 'Synchronisation: N/A';
}

// Fonction utilitaire pour éviter document.getElementById
function $(id) {
    return document.getElementById(id);
    }
// =================================================================
// FICHIER JS PARTIE 2/3 : gnss-dashboard-part2.js
// Contient la logique Astro, le rendu du cadran et la synchro serveur
// =================================================================

// Les constantes D2R et R sont définies dans part1.js
// Références globales : window.SunCalc, window.serverTimeOffset, etc.

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

/**
 * Obtient l'heure actuelle corrigée par le décalage du serveur.
 * C'est l'heure utilisée pour tous les calculs TST/GMT.
 * @returns {Date} L'objet Date corrigé.
 */
function getCorrectedDate() {
    return new Date(Date.now() + window.serverTimeOffset); 
}

// --- NOUVELLE FONCTION DE SYNCHRONISATION ---

function syncTimeWithServer() {
    console.log("Tentative de synchronisation de l'heure...");
    
    // SIMULATION : En production, cette fonction ferait une requête (ex: fetch)
    // à un serveur NTP ou une API (ex: worldtimeapi.org) pour obtenir l'heure UTC.
    // L'heure serveur 'réelle' est simulée ici.
    
    setTimeout(() => {
        // Simule un décalage aléatoire (pour prouver que la synchro fonctionne)
        const simulatedServerTime = Date.now() + Math.random() * 5000 - 2500; 
        const clientTime = Date.now();
        
        window.serverTimeOffset = simulatedServerTime - clientTime;
        
        console.log(`Synchronisation réussie. Décalage ajusté de : ${window.serverTimeOffset.toFixed(0)} ms`);
        
        if ($('time-status')) {
            const offset_ms = window.serverTimeOffset;
            const status_text = (offset_ms !== 0) ? `${offset_ms.toFixed(0)} ms (Corrigé)` : "Synchronisé";
            
            $('time-status').textContent = `Synchro: ${status_text}`;
            $('time-status').style.backgroundColor = (Math.abs(offset_ms) > 100) ? '#ffc107' : '#28a745'; // Jaune si > 100ms, vert sinon
            $('time-status').style.color = '#fff';
            $('time-status').style.padding = '2px 5px';
            $('time-status').style.borderRadius = '3px';
        }
    }, 100); 
}


// --- FONCTIONS ASTRO (TST/EOT) ---

/**
 * Calcule l'Équation du Temps (EOT) et le Temps Solaire Vrai (TST).
 * @param {Date} date - L'objet Date corrigé (GMT/UTC).
 * @param {number} lon - Longitude actuelle.
 * @returns {{EOT_MS: number, EOT: string, TST_MS: number, TST: string}}
 */
function getSolarTime(date, lon) {
    // La position du soleil inclut l'Équation du Temps (EOT) dans son calcul.
    // L'EOT est la différence entre le TST et l'heure moyenne.
    
    // 1. Heure UTC (Moyenne)
    const utc_ms = date.getTime();
    
    // 2. TST (Temps Solaire Vrai)
    // Le TST est calculé comme : UTC + Longitude (en temps) + EOT
    
    // Utilisation d'un calcul EOT standard pour plus de précision et de vérifiabilité
    const J = window.SunCalc.getJDay(date); // Jour Julien
    
    // Formule simplifiée de l'EOT (précise à quelques secondes près)
    const M = (357.5291 + 0.98560028 * J) * D2R; // Anomalie moyenne
    const L = (280.4665 + 0.98564736 * J) * D2R; // Longitude moyenne
    const C = (1.9148 * Math.sin(M) + 0.0200 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M)) * D2R; // Équation du centre
    const lambda = L + C; // Longitude écliptique
    const epsilon = (23.439 - 0.00000036 * J) * D2R; // Obliquité de l'écliptique
    
    const ra = Math.atan2(Math.cos(epsilon) * Math.sin(lambda), Math.cos(lambda));
    
    const EOT_rad = L - ra; // EOT en radians
    const EOT_minutes = (EOT_rad / (2 * Math.PI)) * 1440; // Conversion en minutes
    
    const EOT_MS = EOT_minutes * 60000;
    
    // Correction Longitude (Changement d'heure de longitude) : 1° = 4 minutes (ou 240 secondes)
    const lon_time_ms = lon * 240 * 1000; 

    // TST (Temps Solaire Vrai en ms depuis minuit UTC)
    // UTC (en ms) + Longitude (en ms) + EOT (en ms)
    const tst_ms_local = (date.getHours() * 3600000 + date.getMinutes() * 60000 + date.getSeconds() * 1000 + lon_time_ms + EOT_MS) % 86400000;
    
    const TST_MS = tst_ms_local; // TST en ms (0-86400000)

    // Conversion des MS en format HH:MM:SS
    const tst_date = new Date(TST_MS);
    const TST = tst_date.toLocaleTimeString('en-US', { hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false, timeZone: 'UTC' });
    
    // Conversion de l'EOT en format +/-MM:SS
    const EOT_ABS = Math.abs(EOT_MS);
    const EOT_SIGN = EOT_MS >= 0 ? '+' : '-';
    const EOT_sec = Math.floor(EOT_ABS / 1000) % 60;
    const EOT_min = Math.floor(EOT_ABS / 60000);
    const EOT = `${EOT_SIGN}${String(EOT_min).padStart(2, '0')}:${String(EOT_sec).padStart(2, '0')}`;
    
    return { EOT_MS, EOT, TST_MS, TST };
}

// --- RENDU 2D (CADRAN SOLAIRE) ---

function initSolarClock() {
    const canvas = $('mc-clock-canvas');
    if (canvas) {
        canvas.width = 300; 
        canvas.height = 300; 
    }
}

/**
 * Dessine le cadran solaire 2D corrigé.
 * @param {object} sunPos - Position du Soleil (azimuth, altitude).
 * @param {number} latA - Latitude.
 * @param {number} lonA - Longitude.
 */
function drawSolarClock(sunPos, latA, lonA) {
    const canvas = $('mc-clock-canvas');
    if (!canvas || !sunPos || window.mcTime === null) return; 
    
    const ctx = canvas.getContext('2d');
    const center = canvas.width / 2;
    const radius = center * 0.9;
    
    // Azimut : 0 (Nord) est en haut de la toile.
    // L'Azimut de SunCalc est en radians, compté depuis le Nord dans le sens horaire.
    const sunAzimuth = sunPos.azimuth; 
    const sunElevation = sunPos.altitude; 
    
    // Constantes de rendu
    const MOON_RADIUS = center * 0.08;
    const SUN_RADIUS = center * 0.12;
    
    // Seuils en radians pour le dégradé du ciel (crépuscules civil/astronomique)
    const TWI_CIVIL = -0.833 * D2R; 
    const TWI_ASTRO = -18 * D2R;    

    // Détermination de la couleur du ciel (Dégradé par élévation solaire)
    let skyColor = 'rgba(0,0,0,1)';
    let sunColor = '#FFD700'; // Or
    let nightColor = '#1a1a2e'; // Bleu nuit
    
    if (sunElevation > 0) {
        // JOUR
        const brightness = Math.min(1, sunElevation / (45 * D2R)); // 0 à 1 basé sur l'élévation jusqu'à 45°
        const r = Math.floor(135 + 120 * brightness);
        const g = Math.floor(180 + 75 * brightness);
        const b = Math.floor(255 * brightness);
        skyColor = `rgb(${r},${g},${b})`;
        sunColor = '#FFF59D';
    } else if (sunElevation > TWI_CIVIL) {
        // CRÉPUSCULE CIVIL
        skyColor = '#4682B4'; // Bleu Acier
        sunColor = '#FF8C00'; // Orange
    } else if (sunElevation > TWI_ASTRO) {
        // CRÉPUSCULE ASTRONOMIQUE
        skyColor = '#34495E'; // Bleu foncé
        sunColor = '#9400D3'; // Violet
    }

    // ------------------------------------------------
    // 1. DESSIN DU CADRAN AVEC MASQUE (CLIP)
    // ------------------------------------------------
    ctx.clearRect(0, 0, canvas.width, canvas.height); // Efface le canvas
    
    // A. Dessin du Fond Nuit
    ctx.fillStyle = nightColor; 
    ctx.beginPath();
    ctx.arc(center, center, radius, 0, 2 * Math.PI);
    ctx.fill();

    // B. Masque pour dessiner le secteur jour/ciel
    // Le Soleil est toujours au centre de l'arc jour. L'arc jour est dessiné entre 
    // l'horizon Est (Azimut - 90°) et l'horizon Ouest (Azimut + 90°).

    // La zone de clip permet de dessiner le ciel sur la partie de la toile où le Soleil est au-dessus de l'horizon.
    ctx.save();
    ctx.beginPath();
    ctx.rect(0, 0, canvas.width, canvas.height); 
    ctx.clip(); 

    // C. Dessin du Secteur Jour/Ciel
    // Le ciel est toujours dessiné comme un arc de cercle dont l'angle et le décalage 
    // dépendent de l'élévation du soleil (simulant l'horizon).
    
    // Simuler l'horizon en fonction de l'élévation solaire
    const horizon_offset = -radius * Math.sin(sunElevation); // Décalage vertical
    
    ctx.beginPath();
    ctx.arc(center, center + horizon_offset, radius * 2, 0, 2 * Math.PI); // Dessine un grand cercle centré en dessous du centre réel (pour simuler l'horizon)
    ctx.fillStyle = skyColor; 
    ctx.fill();
    
    ctx.restore(); 
    
    // ------------------------------------------------
    // 2. DESSIN DU SOLEIL (ASTRE) - Position par Azimut
    // ------------------------------------------------
    const r_sun_pos = radius * 0.7; // Rayon de positionnement du Soleil
    // Décalage de -Math.PI/2 pour que le Nord (Azimut 0) soit en haut de l'écran (0 rad)
    const x_sun = center + r_sun_pos * Math.sin(sunAzimuth); 
    const y_sun = center - r_sun_pos * Math.cos(sunAzimuth); // Utilise -cos pour que 0 (Nord) soit en haut

    // Dessin du Soleil (seulement s'il est au-dessus du crépuscule astronomique)
    if (sunElevation > TWI_ASTRO) {
        ctx.beginPath();
        ctx.arc(x_sun, y_sun, SUN_RADIUS, 0, 2 * Math.PI); 
        ctx.fillStyle = sunColor; 
        ctx.fill();
    }

    // ------------------------------------------------
    // 3. DESSIN DE LA LUNE (Position par Azimut Réel)
    // ------------------------------------------------
    const moonPos = window.SunCalc.getMoonPosition(getCorrectedDate(), latA, lonA);
    const moonIllum = window.SunCalc.getMoonIllumination(getCorrectedDate());

    const moonAzimuth = moonPos.azimuth;
    const moonElevation = moonPos.altitude;
    const illuminatedFraction = moonIllum.fraction; 

    // Dessin de la Lune (si elle est au-dessus du crépuscule astronomique)
    if (moonElevation > TWI_ASTRO) { 
        const r_moon_pos = radius * 0.7; 
        const x_moon = center + r_moon_pos * Math.sin(moonAzimuth);
        const y_moon = center - r_moon_pos * Math.cos(moonAzimuth);

        // Dessin du cercle total de la Lune
        ctx.beginPath();
        ctx.arc(x_moon, y_moon, MOON_RADIUS, 0, 2 * Math.PI);
        ctx.fillStyle = '#F0F0F0'; 
        ctx.fill();

        // Dessin de l'OMBRE (Phase Lunaire)
        if (illuminatedFraction < 1) {
            ctx.save();
            ctx.beginPath();
            ctx.arc(x_moon, y_moon, MOON_RADIUS, 0, 2 * Math.PI);
            ctx.clip(); 
            
            const phase = moonIllum.phase;
            // Calcule la distance que le cercle d'ombre doit être décalé
            const phaseOffset = MOON_RADIUS * 2 * (1 - illuminatedFraction); 
            
            let shadowCenterX;
            // Position de l'ombre pour la phase croissante (New Moon -> Full Moon)
            if (phase < 0.5) { 
                 shadowCenterX = x_moon + MOON_RADIUS - phaseOffset; 
            } else { 
                 shadowCenterX = x_moon - MOON_RADIUS + phaseOffset; 
            }

            ctx.beginPath();
            ctx.arc(shadowCenterX, y_moon, MOON_RADIUS * 1.5, 0, 2 * Math.PI);
            ctx.fillStyle = nightColor; 
            ctx.fill();
            
            ctx.restore(); 
        }
    }
    
    // ------------------------------------------------
    // 4. DESSIN DE L'AIGUILLE TST (CORRECTION TST MIDI HAUT)
    // ------------------------------------------------

    // 00:00 TST est en bas (Sud)
    // 06:00 TST est à gauche (Est)
    // 12:00 TST est en haut (Nord) - CORRECTION APPLIQUÉE
    // 18:00 TST est à droite (Ouest)
    
    const total_seconds = window.mcTime / 1000;
    const angle_from_north = (total_seconds / 86400) * 2 * Math.PI; // Angle TST 0-2PI
    
    // CORRECTION : L'angle 0 TST doit correspondre à l'angle Nord (haut, -PI/2)
    // Un angle de 0 (minuit) correspond à -PI/2 + PI = PI/2.
    // L'angle de 12:00 (midi) correspond à 0.
    
    // Angle pour que 12h (PI) soit en haut (0)
    const TST_ANGLE = angle_from_north - Math.PI; // Décalage pour aligner l'angle

    ctx.save();
    ctx.translate(center, center);
    ctx.rotate(TST_ANGLE); 

    // Dessin de la ligne de l'aiguille TST
    ctx.strokeStyle = '#D9534F'; // Rouge
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(0, -radius * 0.9); // Aiguille pointant vers le Nord (TST 12h)
    ctx.stroke();

    ctx.restore();

    // Dessin du Pivot Central
    ctx.fillStyle = '#000000';
    ctx.beginPath();
    ctx.arc(center, center, center * 0.05, 0, 2 * Math.PI);
    ctx.fill();

    // ------------------------------------------------
    // 5. MARQUES HORAIRES TST
    // ------------------------------------------------
    ctx.fillStyle = GOLD_COLOR;
    ctx.font = `${center * 0.1}px Arial`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    
    for (let h = 0; h < 24; h += 3) {
        // Position des heures : 12h en haut, 0h en bas
        let hour_angle = (h / 24) * 2 * Math.PI;
        hour_angle = hour_angle - Math.PI; // Décalage pour aligner 12h en haut
        
        const x = center + radius * 0.8 * Math.sin(hour_angle);
        const y = center - radius * 0.8 * Math.cos(hour_angle);
        
        ctx.fillText(h.toString().padStart(2, '0') + 'h', x, y);
    }
}

// --- LOGIQUE GLOBALE D'AFFICHAGE ---

function updateAstro(latA, lonA) {
    const now = getCorrectedDate(); // *** UTILISE L'HEURE CORRIGÉE ***
    if (now === null || !window.SunCalc) return; 

    // Calculs Astro
    const sunPos = window.SunCalc.getPosition(now, latA, lonA);
    const solarTimes = getSolarTime(now, lonA); 
    window.mcTime = solarTimes.TST_MS; // TST en ms

    // Affichage des données GMT et TST
    const gmt_time_str = now.toLocaleTimeString('en-US', { hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false, timeZone: 'UTC' });
    if ($('local-time')) $('local-time').textContent = gmt_time_str + " (GMT/UTC)";
    if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
    
    if ($('tst')) $('tst').textContent = solarTimes.TST;
    if ($('eot')) $('eot').textContent = solarTimes.EOT;
    if ($('sun-azimuth')) $('sun-azimuth').textContent = (sunPos.azimuth / D2R).toFixed(1) + "°";
    if ($('sun-elevation')) $('sun-elevation').textContent = (sunPos.altitude / D2R).toFixed(1) + "°";

    // Mise à jour du Cadran
    drawSolarClock(sunPos, latA, lonA); 
}

function fetchExternalData(latA, lonA) {
    // SIMULATION de la récupération de données externes (Air, Chimie)
    // En production, cette fonction ferait des requêtes à des API comme OpenWeatherMap, etc.
    const temp = (Math.random() * 10 + 15).toFixed(1); // 15.0 - 25.0 °C
    const pressure = (Math.random() * 10 + 1010).toFixed(0); // 1010 - 1020 hPa
    
    if ($('temperature')) $('temperature').textContent = `${temp}°C`;
    if ($('pressure')) $('pressure').textContent = `${pressure} hPa`;
    
    // Si l'IMU était réel, on pourrait calibrer le baromètre avec la pression atmosphérique
}

function updatePhysicsAndChemistry() {
    // Mise à jour des distances et vitesses
    if ($('distance-total')) $('distance-total').textContent = `${(window.distTotal / 1000).toFixed(2)} km`;
    if ($('distance-moving')) $('distance-moving').textContent = `${(window.distMoving / 1000).toFixed(2)} km`;
    if ($('speed-max')) $('speed-max').textContent = `${window.speedMax.toFixed(1)} km/h`;
    
    // L'affichage de la vitesse instantanée est mis à jour dans handleNewPosition
}

// --- FONCTIONS CARTE (LEAFLET) ---
function initMap() {
    // Initialisation de la carte Leaflet
    const defaultLat = 43.296482; // Marseille
    const defaultLon = 5.36978;
    
    if ($('map')) {
        window.lMap = L.map('map').setView([defaultLat, defaultLon], 13);
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

// --- INITIALISATION DES ÉVÉNEMENTS DOM ---

document.addEventListener('DOMContentLoaded', () => {
    
    // Vérification de la dépendance (SunCalc doit être chargé via un script <script> avant)
    if (!window.SunCalc) {
        console.error("Erreur : SunCalc.js n'est pas chargé. Les fonctions Astro ne marcheront pas.");
        return;
    }
    
    initMap(); 
    initSolarClock(); 
    startGPS();
    startIMU();
    syncTimeWithServer(); // *** PREMIÈRE SYNCHRO AU DÉMARRAGE ***
    fetchExternalData(43.296482, 5.36978); // Première récupération de données externes

    // Attachement des fonctions de contrôle aux boutons
    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', resetDistance);
    if ($('reset-speed-btn')) $('reset-speed-btn').addEventListener('click', resetSpeedMax);
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', resetAll);

    // Mise à jour périodique des données DOM/Astro/Physique (1x/seconde)
    if (window.domID === null) {
        window.domID = setInterval(() => {
            const currentLat = window.lPos ? window.lPos.coords.latitude : 43.296482; 
            const currentLon = window.lPos ? window.lPos.coords.longitude : 5.36978;
            
            const elapsed_ms = Date.now() - window.startTime; 
            if ($('time-elapsed')) $('time-elapsed').textContent = msToTime(elapsed_ms);
            if ($('time-moving')) $('time-moving').textContent = msToTime(window.timeMoving);
            
            updateAstro(currentLat, currentLon); 
            updatePhysicsAndChemistry(); 
            updateMap();
            
            // SYNCHRO RÉGULIÈRE : toutes les 60 secondes (60000ms)
            if (Date.now() % 60000 < DOM_SLOW_UPDATE_MS) { 
                syncTimeWithServer(); 
            }
            
            // Mise à jour des données externes toutes les 10 secondes (pour simuler)
            if (Date.now() % 10000 < DOM_SLOW_UPDATE_MS) { 
                fetchExternalData(currentLat, currentLon); 
            }
        }, DOM_SLOW_UPDATE_MS); 
    }
});
