// =================================================================
// FICHIER JS PARTIE 1/2 : gnss-dashboard-part1.js
// Contient : Variables globales, Constantes, Fonctions de support, Initialisation EKF/IMU, Fonctions de Démarrage/Contrôle
// =================================================================

// --- FONCTIONS DE SUPPORT GLOBALES (Doivent être au début) ---
const $ = (id) => document.getElementById(id); // Ajout : Fonction de sélection DOM
const D2R = Math.PI / 180, R2D = 180 / Math.PI;

// --- CLÉS D'API & PROXY VERCEL ---
// ... (Utilisez vos clés d'API ici si nécessaire)

// --- CONSTANTES GLOBALES ET INITIALISATION ---
const C_L = 299792458;      // Vitesse de la Lumière (m/s)
const R_E = 6371000;        // Rayon de la Terre (m)
const KMH_MS = 3.6;         // Facteur de conversion
const C_S = 343;            // Vitesse du Son (m/s)
const G_ACC = 9.80665;      // Accélération de la gravité (m/s²)
const DOM_SLOW_UPDATE_MS = 1000; // Fréquence de rafraîchissement DOM lent (1s)
const dayMs = 86400000;     // 24 heures en millisecondes
const R_MAX = 1000.0;       // Précision max par défaut

// --- VARIABLES D'ÉTAT ---
window.lPos = null;         // Dernière position GPS (objet Position)
window.lVitesse = 0;        // Dernière vitesse GPS (m/s)
window.lTime = 0;           // Dernier timestamp
window.isMoving = false;    
window.distanceTotal = 0;   
window.speedMax = 0;        
window.timeMoving = 0;      
window.gpsActive = false;   
window.imuActive = false;   
window.domID = null;        
window.startTime = Date.now(); 

// VARIABLES EKF
// ... (ekfState, ekfCov, R_BASE, Q_BASE inchangées)
window.ekfState = { 
    x: 0, 
    y: 0, 
    vx: 0, 
    vy: 0, 
    ax: 0, 
    ay: 0 
};
window.ekfCov = [
    [100, 0, 0, 0, 0, 0], 
    [0, 100, 0, 0, 0, 0],
    [0, 0, 10, 0, 0, 0],
    [0, 0, 0, 10, 0, 0],
    [0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 1]
];
window.R_BASE = 0.5; 
window.Q_BASE = 0.01; 
window.gpsOverride = 0.0; // Précision GPS forcée

// VARIABLES IMU
window.imuAccel = { x: 0, y: 0, z: 0 };
window.imuData = null; 

// VARIABLES DE CARTE
window.map = null;
window.userMarker = null;

// VARIABLES TEMPS TST
window.mcTime = null; 

// --- FONCTIONS DE GESTION GPS & EKF ---

/**
 * Fonction EKF (Filtre de Kalman Étendu) - Placeholder.
 */
function updateEKF(dt, accX, accY, gpsMeasurement) {
    // La logique EKF complexe doit être implémentée ici.
    // Pour l'instant, on simule une mise à jour minime pour éviter le crash.
    window.ekfState.vx += accX * dt;
    window.ekfState.vy += accY * dt;

    return { 
        vx: window.ekfState.vx, 
        vy: window.ekfState.vy,
        uncertainty: window.ekfCov[2][2] + window.ekfCov[3][3] 
    }; 
}

/**
 * Gestion des erreurs GPS.
 */
function gpsError(error) {
    console.error("Erreur GPS:", error.code, error.message);
    // Mettre à jour l'affichage d'erreur si nécessaire
}

/**
 * Démarre l'écoute GPS.
 */
function startGPS() {
    if ("geolocation" in navigator) {
        window.gpsActive = true;
        // updateDisp est dans part2.js, ce qui est normal pour la séparation.
        navigator.geolocation.watchPosition(updateDisp, gpsError, {
            enableHighAccuracy: true,
            maximumAge: 500,
            timeout: 5000
        });
        if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = "⏸️ PAUSE GPS";
    } else {
        alert("La géolocalisation n'est pas supportée par ce navigateur.");
    }
}

/**
 * Gère les données IMU (accéléromètre, gyroscope).
 */
function handleIMU(event) {
    // Les accélérations brutes incluent la gravité (z = ~9.81)
    window.imuAccel = event.accelerationIncludingGravity;
    window.imuData = event;
    
    // Si la fonction updateDisp (part2.js) est exécutée rapidement, 
    // elle prendra ces données pour le calcul EKF.
}

/**
 * Démarre l'écoute IMU (DeviceMotion).
 */
function startIMU() {
    if (window.DeviceMotionEvent) {
        window.imuActive = true;
        window.addEventListener('devicemotion', handleIMU, true);
        // Vous devez ajouter un bouton 'toggle-imu-btn' ou l'intégrer au flux existant
    } else {
        console.warn("DeviceMotionEvent non supporté ou non autorisé.");
    }
}

/**
 * Fonctions de réinitialisation (pour les boutons)
 */
function resetDistance() {
    window.distanceTotal = 0;
    window.timeMoving = 0;
    // ... Mettre à jour le DOM
}

function resetSpeedMax() {
    window.speedMax = 0;
    // ... Mettre à jour le DOM
}

function resetAll() {
    resetDistance();
    resetSpeedMax();
    // ... Réinitialiser l'EKF, etc.
}

// ... (Ajouter les autres fonctions de contrôle nécessaires ici)
// =================================================================
// FICHIER JS PARTIE 2/2 : gnss-dashboard-part2.js
// Contient : Fonctions de support (Astro, DOM), Logique Montre TST/Astro, Carte Leaflet
// =================================================================

// --- FONCTIONS DE SUPPORT GLOBALES (Doivent être redéfinies si part1 n'est pas chargé en premier) ---
// On suppose que part1 est chargé en premier, donc $ et D2R/R2D sont disponibles.

/** Conversion ms en HH:MM:SS */
function msToTime(ms) {
    const hours = Math.floor(ms / (1000 * 60 * 60));
    const minutes = Math.floor((ms % (1000 * 60 * 60)) / (1000 * 60));
    const seconds = Math.floor((ms % (1000 * 60)) / 1000);
    return `${String(hours).padStart(2, '0')}:${String(minutes).padStart(2, '0')}:${String(seconds).padStart(2, '0')}`;
}

/** Obtient l'heure actuelle (simule NTP) */
function getCDate() {
    return new Date(); 
}

// --- FONCTIONS ASTRO (SunCalc/TST/EOT) ---

/** Calcule le Temps Solaire Vrai (TST) et l'Équation du Temps (EOT). */
function getSolarTime(date, lon) {
    // Calcul de l'heure UTC en MS depuis minuit
    const msSinceMidnightUTC = (date.getUTCHours() * 3600 + date.getUTCMinutes() * 60 + date.getUTCMilliseconds()) * 1000;
    
    // Heure Solaire Moyenne Locale (MST)
    const mst_offset_ms = lon * dayMs / 360; 
    const mst_ms = (msSinceMidnightUTC + mst_offset_ms + dayMs) % dayMs;

    // Calcul de l'Équation du Temps (EOT) - REQUIERT UNE LIBRAIRIE ASTRO PLUS COMPLÈTE
    // Pour l'instant, on simule l'EOT avec une approximation ou une valeur de base.
    const eot_min = 0.0; 
    const eot_ms = eot_min * 60000;
    
    // TST = MST + EOT
    const tst_ms = (mst_ms + eot_ms + dayMs) % dayMs; 
    
    return { 
        TST: msToTime(tst_ms), 
        TST_MS: tst_ms, 
        MST: msToTime(mst_ms), 
        EOT: eot_min.toFixed(3),
    };
}


/** Mise à jour des affichages Astro et appel du dessin de la montre. */
function updateAstro(latA, lonA) {
    const now = getCDate(); 
    if (now === null) return; 

    // Vérifie si SunCalc est chargé
    if (!window.SunCalc) {
        console.error("SunCalc n'est pas chargé.");
        return;
    }

    const sunPos = window.SunCalc.getPosition(now, latA, lonA);
    const times = window.SunCalc.getTimes(now, latA, lonA);

    const solarTimes = getSolarTime(now, lonA); 
    window.mcTime = solarTimes.TST_MS; // Stockage du TST

    // --- MISE À JOUR DOM ASTRO/TEMPS ---
    if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString('fr-FR', { timeZone: 'UTC', hour12: false });
    if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
    
    if ($('tst')) $('tst').textContent = solarTimes.TST;
    if ($('lsm')) $('lsm').textContent = solarTimes.MST;
    if ($('eot')) $('eot').textContent = solarTimes.EOT + " min";
    
    if ($('sun-elevation')) $('sun-elevation').textContent = (sunPos.altitude * R2D).toFixed(2) + " °";
    
    // Mise à jour de l'horloge TST/Astro
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
    
    // Calcul de la position de l'astre
    const rotationAngle = sunAzimuth + Math.PI / 2; // Azimut 0 (Nord) est en haut
    const elevationFraction = Math.abs(sunElevation) / (Math.PI / 2); 
    const r_sun = radius * elevationFraction; 

    // --- Dessin du Cadran (Identique au dernier échange) ---
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    
    ctx.beginPath();
    ctx.arc(center, center, radius, 0, 2 * Math.PI);
    ctx.fillStyle = '#C89A4D'; 
    ctx.fill();

    // Couleur du ciel/simulation jour-nuit
    let skyColor, sunColor;
    if (sunElevation > D2R * 10) { 
        skyColor = '#4C7AD9'; 
        sunColor = 'yellow';
    } else if (sunElevation > D2R * -5) { 
        skyColor = '#E6994C'; 
        sunColor = 'orange';
    } else { 
        skyColor = '#212A3E'; 
        sunColor = 'darkorange';
    }
    
    // Projection Jour/Nuit 
    ctx.beginPath();
    ctx.arc(center, center, radius, rotationAngle - Math.PI/2, rotationAngle + Math.PI/2);
    ctx.lineTo(center, center);
    ctx.fillStyle = skyColor; 
    ctx.fill();
    
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
    if ($('time-minecraft')) $('time-minecraft').textContent = tst_str;
    if ($('clock-status')) $('clock-status').textContent = `TST (Élévation: ${(sunElevation * R2D).toFixed(2)}°)`;
}


// --- FONCTIONS CARTE LEAFLET ---

function initMap() {
    if (window.map) {
        window.map.remove(); 
    }
    
    const initialLat = 43.296482; 
    const initialLon = 5.36978;
    
    if ($('map-container')) {
        window.map = L.map('map-container').setView([initialLat, initialLon], 16);
        
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            maxZoom: 19,
            attribution: '© OpenStreetMap'
        }).addTo(window.map);

        window.userMarker = L.marker([initialLat, initialLon]).addTo(window.map)
            .bindPopup("Position initiale").openPopup();
    }
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

/** Fonction appelée par le watchPosition du GPS. */
function updateDisp(pos) {
    // Si le GPS est inactif, on ne met à jour que la position initiale si elle est null
    if (!window.gpsActive) {
        if (!window.lPos) window.lPos = pos;
        return;
    }
    
    // ... (Logique de calcul dt, etc.)
    const dt = window.lPos ? (pos.timestamp - window.lPos.timestamp) / 1000 : 0.5;

    const lat = pos.coords.latitude;
    const lon = pos.coords.longitude;
    const altRaw = pos.coords.altitude;
    const acc = window.gpsOverride > 0 ? window.gpsOverride : (pos.coords.accuracy ?? R_MAX); 
    
    // ... (Mise à jour EKF)
    
    // Mise à jour de l'affichage DOM Vitesse/Position
    if ($('latitude')) $('latitude').textContent = lat.toFixed(6);
    if ($('longitude')) $('longitude').textContent = lon.toFixed(6);
    if ($('altitude-gps')) $('altitude-gps').textContent = altRaw.toFixed(2) + " m";
    if ($('gps-precision')) $('gps-precision').textContent = acc.toFixed(2) + " m";
    
    // Mise à jour de la carte Leaflet
    updateMap(lat, lon, altRaw, acc);
    
    window.lPos = pos; 
}


// --- INITIALISATION DES ÉVÉNEMENTS DOM ---

document.addEventListener('DOMContentLoaded', () => {
    
    // Initialisation des composants
    initMap(); 
    initSolarClock(); 

    // Événements des boutons de contrôle
    if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', startGPS);
    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', resetDistance);
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', resetSpeedMax);
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', resetAll);
    
    // Événement pour la précision forcée
    if ($('gps-accuracy-override')) {
        $('gps-accuracy-override').addEventListener('change', (e) => {
            window.gpsOverride = parseFloat(e.target.value) || 0.0;
            if ($('gps-accuracy-effective')) $('gps-accuracy-effective').textContent = window.gpsOverride > 0 ? `${window.gpsOverride.toFixed(6)} m (Forcé)` : 'N/A';
        });
    }

    // Mise à jour périodique des données DOM/Astro (1x/seconde)
    if (window.domID === null) {
        window.domID = setInterval(() => {
            const currentLat = window.lPos ? window.lPos.coords.latitude : 43.296482; 
            const currentLon = window.lPos ? window.lPos.coords.longitude : 5.36978;
            updateAstro(currentLat, currentLon); 
        }, DOM_SLOW_UPDATE_MS); 
    }
    
    // DÉMARRAGE DES ÉCOUTES (Le GPS est démarré par le bouton au clic, ici on initie juste IMU)
    startIMU();   
});
