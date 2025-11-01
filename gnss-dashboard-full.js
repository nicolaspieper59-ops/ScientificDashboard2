// =================================================================
// FICHIER JS PARTIE 1/2 : gnss-dashboard-part1.js
// Contient : Variables globales, Constantes, Initialisation EKF/IMU, Fonctions de Démarrage
// =================================================================

// --- CLÉS D'API & PROXY VERCEL ---
// ... (Utilisez vos clés d'API ici si nécessaire)

// --- CONSTANTES GLOBALES ET INITIALISATION ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;      // Vitesse de la Lumière (m/s)
const R_E = 6371000;        // Rayon de la Terre (m)
const KMH_MS = 3.6;         // Facteur de conversion
const C_S = 343;            // Vitesse du Son (m/s)
const G_ACC = 9.80665;      // Accélération de la gravité (m/s²)
const DOM_SLOW_UPDATE_MS = 1000; // Fréquence de rafraîchissement DOM lent (1s)
const dayMs = 86400000;     // 24 heures en millisecondes

// --- VARIABLES D'ÉTAT ---
window.lPos = null;         // Dernière position GPS (objet Position)
window.lVitesse = 0;        // Dernière vitesse GPS (m/s)
window.lTime = 0;           // Dernier timestamp
window.isMoving = false;    // Statut de mouvement
window.distanceTotal = 0;   // Distance totale parcourue (m)
window.speedMax = 0;        // Vitesse maximale (km/h)
window.timeMoving = 0;      // Temps passé en mouvement (s)
window.gpsActive = false;   // Statut GPS
window.imuActive = false;   // Statut IMU
window.domID = null;        // ID de l'intervalle de rafraîchissement DOM
window.startTime = Date.now(); // Temps de démarrage de la session

// VARIABLES EKF (Filtre de Kalman Étendu)
window.ekfState = { 
    x: 0, 
    y: 0, 
    vx: 0, 
    vy: 0, 
    ax: 0, 
    ay: 0 
};
window.ekfCov = [
    [100, 0, 0, 0, 0, 0], // Grande incertitude initiale
    [0, 100, 0, 0, 0, 0],
    [0, 0, 10, 0, 0, 0],
    [0, 0, 0, 10, 0, 0],
    [0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 1]
];
window.R_BASE = 0.5; // Erreur de mesure GPS de base (m²)
window.Q_BASE = 0.01; // Bruit de processus de base (m/s⁴)

// VARIABLES IMU
window.imuAccel = { x: 0, y: 0, z: 0 };
window.imuData = null; // Objet pour le dernier événement DeviceMotion

// VARIABLES DE CARTE
window.map = null;
window.userMarker = null;

// VARIABLES TEMPS TST
window.mcTime = null; // Sera TST en MS (Temps Solaire Vrai en millisecondes depuis minuit)

// ... (Ajouter ici les fonctions de support mathématiques comme msToTime, $ etc.)

// --- FONCTIONS DE GESTION GPS & EKF ---

/**
 * Fonction EKF (Filtre de Kalman Étendu) - Le cœur du filtrage.
 * (La logique complète doit être ici ou dans part2.js)
 */
function updateEKF(dt, accX, accY, gpsMeasurement) {
    // ... (Logique EKF de Prédiction et Mise à Jour - Doit être complète)
    // Placeholder: 
    // Mettre à jour ekfState, ekfCov, et retourner la vitesse stable (vx, vy)
    
    // Retourne une vitesse stable simulée pour éviter les erreurs d'exécution:
    return { 
        vx: window.ekfState.vx, 
        vy: window.ekfState.vy,
        uncertainty: window.ekfCov[2][2] + window.ekfCov[3][3] // Somme des variances de vitesse
    }; 
}

/**
 * Démarre l'écoute GPS.
 */
function startGPS() {
    if ("geolocation" in navigator) {
        window.gpsActive = true;
        // La fonction updateDisp (dans part2.js) sera appelée à chaque mise à jour.
        // Utiliser une fréquence élevée pour la meilleure précision
        navigator.geolocation.watchPosition(updateDisp, gpsError, {
            enableHighAccuracy: true,
            maximumAge: 500,
            timeout: 5000
        });
        $('toggle-gps-btn').textContent = "⏸️ PAUSE GPS";
    } else {
        alert("La géolocalisation n'est pas supportée par ce navigateur.");
    }
}

/**
 * Démarre l'écoute IMU (DeviceMotion).
 */
function startIMU() {
    if (window.DeviceMotionEvent) {
        window.imuActive = true;
        window.addEventListener('devicemotion', handleIMU, true);
        $('toggle-imu-btn').textContent = "⏸️ PAUSE IMU";
    } else {
        // Optionnel : Gérer les anciens navigateurs ou les restrictions
    }
}

// ... (Autres fonctions : startIMU, handleIMU, gpsError, toggleGPS, toggleIMU)// =================================================================
// FICHIER JS PARTIE 2/2 : gnss-dashboard-part2.js
// Contient : Fonctions de support (Astro, DOM), Logique Montre TST/Astro, Carte Leaflet
// =================================================================

// --- FONCTIONS DE SUPPORT GLOBALES ---

const $ = (id) => document.getElementById(id);
const D2R = Math.PI / 180, R2D = 180 / Math.PI; // Répété pour la clarté

function msToTime(ms) {
    const hours = Math.floor(ms / (1000 * 60 * 60));
    const minutes = Math.floor((ms % (1000 * 60 * 60)) / (1000 * 60));
    const seconds = Math.floor((ms % (1000 * 60)) / 1000);
    return `${String(hours).padStart(2, '0')}:${String(minutes).padStart(2, '0')}:${String(seconds).padStart(2, '0')}`;
}

/** Obtient l'heure actuelle, en gérant une synchronisation NTP simulée. */
function getCDate() {
    // Si vous utilisez une vraie synchro NTP, modifiez cette fonction.
    return new Date(); 
}

// --- FONCTIONS ASTRO (SunCalc/TST/EOT) ---

/** Calcule le Temps Solaire Vrai (TST) et l'Équation du Temps (EOT). */
function getSolarTime(date, lon) {
    // Simplification du calcul EOT/TST (La librairie SunCalc pourrait être utilisée pour l'EOT, mais la formule manuelle est courante)
    
    // Calcul de l'heure UTC en MS depuis minuit
    const msSinceMidnightUTC = (date.getUTCHours() * 3600 + date.getUTCMinutes() * 60 + date.getUTCMilliseconds()) * 1000;
    
    // Calcul de l'heure Solaire Moyenne Locale (MST)
    const mst_offset_ms = lon * dayMs / 360; 
    const mst_ms = (msSinceMidnightUTC + mst_offset_ms + dayMs) % dayMs;

    // Calcul de l'Équation du Temps (EOT) - Nécessite le Jour Julien, ici simulé.
    // Pour une EOT précise, il faut une librairie complète. Ici, on utilise SunCalc pour l'EOT, mais il n'expose pas directement la valeur.
    // **Pour l'exemple, l'EOT est mise à 0.0 min, et doit être calculée par une librairie externe (ou manuellement).**
    const eot_min = 0.0; // À CALCULER (La valeur de l'EOT est cruciale)
    const eot_ms = eot_min * 60000;
    
    // TST = MST + EOT
    const tst_ms = (mst_ms + eot_ms + dayMs) % dayMs; 
    
    return { 
        TST: msToTime(tst_ms), 
        TST_MS: tst_ms, 
        MST: msToTime(mst_ms), 
        EOT: eot_min.toFixed(3),
        // Le reste des valeurs astro doit venir du SunCalc.getPosition
    };
}


/**
 * Mise à jour des affichages Astro et appel du dessin de la montre.
 */
function updateAstro(latA, lonA) {
    const now = getCDate(); 
    if (now === null) return; 

    const sunPos = window.SunCalc.getPosition(now, latA, lonA);
    const moonPos = window.SunCalc.getMoonPosition(now, latA, lonA);
    const times = window.SunCalc.getTimes(now, latA, lonA);

    const solarTimes = getSolarTime(now, lonA); 
    window.mcTime = solarTimes.TST_MS; // Stockage du TST

    // --- MISE À JOUR DOM ASTRO/TEMPS ---
    $('local-time').textContent = now.toLocaleTimeString('fr-FR', { timeZone: 'UTC', hour12: false });
    $('date-display').textContent = now.toLocaleDateString('fr-FR');
    
    $('tst').textContent = solarTimes.TST;
    $('lsm').textContent = solarTimes.MST;
    $('eot').textContent = solarTimes.EOT + " min";
    
    $('sun-elevation').textContent = (sunPos.altitude * R2D).toFixed(2) + " °";
    $('ecliptic-long').textContent = (sunPos.eclipticLong * R2D).toFixed(2) + " °";
    
    // ... (Mise à jour des autres champs astro)
    
    // Appel du dessin de la montre TST/Astro
    drawSolarClock(sunPos); 
}


// --- FONCTIONS MONTRE TST/ASTRO ---

/** Initialise le canvas de la montre. */
function initSolarClock() {
    const container = $('minecraft-clock');
    if (!container) return;

    const canvas = document.createElement('canvas');
    canvas.id = 'mc-clock-canvas'; 
    canvas.width = 90;
    canvas.height = 90;
    container.appendChild(canvas);
}

/** Dessine la montre en utilisant les coordonnées solaires réelles (TST/SunCalc). */
function drawSolarClock(sunPos) {
    const canvas = $('mc-clock-canvas');
    if (!canvas || !sunPos || window.mcTime === null) return; 
    
    const ctx = canvas.getContext('2d');
    const center = canvas.width / 2;
    const radius = center * 0.9;
    
    const sunAzimuth = sunPos.azimuth; 
    const sunElevation = sunPos.altitude; 
    
    // Azimut (Angle de rotation) : Azimuth 0 (Nord) est en haut
    const rotationAngle = sunAzimuth + Math.PI / 2; 

    // Élévation (Position sur le rayon) : Zénith -> Bord, Horizon -> Centre
    const elevationFraction = Math.abs(sunElevation) / (Math.PI / 2); 
    const r_sun = radius * elevationFraction; 

    // --- Dessin du Cadran ---
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    
    // Fond de la montre (Or)
    ctx.beginPath();
    ctx.arc(center, center, radius, 0, 2 * Math.PI);
    ctx.fillStyle = '#C89A4D'; 
    ctx.fill();

    // Couleur du ciel/simulation jour-nuit (basée sur l'élévation réelle)
    let skyColor, sunColor;
    if (sunElevation > D2R * 10) { // Jour clair (> 10°)
        skyColor = '#4C7AD9'; 
        sunColor = 'yellow';
    } else if (sunElevation > D2R * -5) { // Crépuscule/Aube
        skyColor = '#E6994C'; 
        sunColor = 'orange';
    } else { // Nuit
        skyColor = '#212A3E'; 
        sunColor = 'darkorange';
    }
    
    // Projection Jour/Nuit (Ciel) : basé sur l'Azimut pour la rotation
    ctx.beginPath();
    // Dessin de la zone jour, couvrante, centrée sur l'azimut solaire
    ctx.arc(center, center, radius, rotationAngle - Math.PI/2, rotationAngle + Math.PI/2);
    ctx.lineTo(center, center);
    ctx.fillStyle = skyColor; 
    ctx.fill();
    
    // Dessin de l'arc de Nuit (opposé)
    ctx.beginPath();
    ctx.arc(center, center, radius, rotationAngle + Math.PI/2, rotationAngle - Math.PI/2 + 2 * Math.PI);
    ctx.lineTo(center, center);
    ctx.fillStyle = '#101828'; 
    ctx.fill();


    // Dessin de l'Astre (Soleil)
    const x_sun = center + r_sun * Math.cos(rotationAngle);
    const y_sun = center + r_sun * Math.sin(rotationAngle);

    ctx.beginPath();
    ctx.arc(x_sun, y_sun, center * 0.15, 0, 2 * Math.PI);
    ctx.fillStyle = sunColor; 
    ctx.fill();

    // Centre Or
    ctx.beginPath();
    ctx.arc(center, center, center * 0.1, 0, 2 * Math.PI);
    ctx.fillStyle = '#C89A4D'; 
    ctx.fill();
    
    // Mise à jour de l'affichage TST
    const tst_str = msToTime(window.mcTime);
    $('time-minecraft').textContent = tst_str;
    $('clock-status').textContent = `TST (Élévation: ${(sunElevation * R2D).toFixed(2)}°)`;
}


// --- FONCTIONS CARTE LEAFLET ---

function initMap() {
    if (window.map) {
        window.map.remove(); 
    }
    
    const initialLat = 43.296482; 
    const initialLon = 5.36978;
    
    window.map = L.map('map-container').setView([initialLat, initialLon], 16);
    
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        maxZoom: 19,
        attribution: '© OpenStreetMap'
    }).addTo(window.map);

    window.userMarker = L.marker([initialLat, initialLon]).addTo(window.map)
        .bindPopup("Position initiale").openPopup();
}

function updateMap(latA, lonA, altA, accA) {
    if (window.map && window.userMarker) {
        const newLatLon = L.latLng(latA, lonA);
        window.userMarker.setLatLng(newLatLon);
        
        window.map.panTo(newLatLon);

        window.userMarker.setPopupContent(
            `Lat: ${latA.toFixed(6)}<br>` + 
            `Lon: ${lonA.toFixed(6)}<br>` + 
            `Alt: ${altA.toFixed(2)} m<br>` +
            `Précision: ${accA.toFixed(2)} m`
        );
    }
}


// --- FONCTION PRINCIPALE DE MISE À JOUR GPS ---

/**
 * Fonction appelée par le watchPosition du GPS.
 */
function updateDisp(pos) {
    if (!window.gpsActive || !pos || !window.lPos) {
        window.lPos = pos; 
        return;
    }

    const dt = (pos.timestamp - window.lPos.timestamp) / 1000;
    if (dt <= 0) return;

    const lat = pos.coords.latitude;
    const lon = pos.coords.longitude;
    const altRaw = pos.coords.altitude;
    const acc = pos.coords.accuracy ?? 1000.0; // Précision brute
    
    // ... (Logique EKF, calcul de vitesse/distance) ...
    
    // Mise à jour de l'affichage DOM Vitesse/Position
    $('latitude').textContent = lat.toFixed(6);
    $('longitude').textContent = lon.toFixed(6);
    $('altitude-gps').textContent = altRaw.toFixed(2) + " m";
    $('gps-precision').textContent = acc.toFixed(2) + " m";
    
    // Mise à jour de la carte Leaflet
    updateMap(lat, lon, altRaw, acc);
    
    // SAUVEGARDE DES VALEURS POUR LA PROCHAINE ITÉRATION
    window.lPos = pos; 
}

// ... (Autres fonctions : handleIMU, gpsError, toggleGPS, toggleIMU, reset)

// --- INITIALISATION DES ÉVÉNEMENTS DOM ---

document.addEventListener('DOMContentLoaded', () => {
    
    // Initialisation de la carte Leaflet
    initMap(); 
    // Initialisation de l'horloge Solaire Vraie
    initSolarClock(); 

    // Mise à jour périodique des données DOM/Astro (1x/seconde)
    if (window.domID === null) {
        window.domID = setInterval(() => {
            // Utiliser une position par défaut si le GPS n'a pas encore de fix
            const currentLat = window.lPos ? window.lPos.coords.latitude : 43.296482; 
            const currentLon = window.lPos ? window.lPos.coords.longitude : 5.36978;
            updateAstro(currentLat, currentLon); 
        }, DOM_SLOW_UPDATE_MS); 
    }
    
    // DÉMARRAGE DES ÉCOUTES
    startGPS();   
    startIMU();   
    
    // ... (Ajouter ici les écouteurs d'événements pour les boutons)
});
