// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET V9.1 (FIX VITESSE & MOYENNES)
// VERSION : FINALE ULTRA-ROBUSTE (Non Simulé / Calculs Moyennes Corrigés)
// DÉPENDANCES : math.min.js, ukf-lib.js, astro.js, leaflet.js, turf.min.js
// =================================================================

// =================================================================
// PARTIE 1 : UTILITAIRES ET FORMATAGE
// =================================================================

const $ = id => document.getElementById(id);

// Formatage robuste : Affiche 0.00 au lieu de N/A ou -- si la valeur est 0
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        // Si c'est null/undefined, on affiche 0 par défaut pour les vitesses/distances
        const zeroFormat = (decimals === 0 ? '0' : '0.' + Array(decimals).fill('0').join(''));
        return zeroFormat.replace('.', ',') + suffix;
    }
    return val.toFixed(decimals).replace('.', ',') + suffix;
};

const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) return '0,00e+0' + suffix;
    return val.toExponential(decimals).replace('.', ',').replace('e', 'e') + suffix;
};

// =================================================================
// PARTIE 2 : CONSTANTES ET ÉTAT GLOBAL
// =================================================================

// --- CONSTANTES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI; 
const KMH_MS = 3.6;         
const C_L = 299792458;      
const G_U = 6.67430e-11;    
const G_STD = 9.80665;
const OMEGA_EARTH = 7.2921159e-5; 

// Atmosphère Standard (ISA)
const TEMP_SEA_LEVEL_K = 288.15; 
const BARO_ALT_REF_HPA = 1013.25; 
const RHO_SEA_LEVEL = 1.225; 
const R_SPECIFIC_AIR = 287.058; 
const GAMMA_AIR = 1.4; 

// --- ÉTAT GLOBAL ---
let ukf = null;
let isGpsPaused = true; // Démarrage en PAUSE (attente action utilisateur)
let isGpsActive = false; // Devient true dès la première trame GPS

// Position & Mouvement
let currentPosition = { 
    lat: 43.284578, lon: 5.358713, alt: 100.00, // Coordonnées initiales (Marseille)
    acc: 0.0, spd_ms: 0.0, heading: 0.0 
};

// Statistiques de Session
let maxSpeedMs = 0.0;
let totalDistanceM = 0.0;
let totalMovementTimeS = 0.0;
let totalSessionTimeS = 0.0;
let lastTimestamp = Date.now();
let lastGPSPosition = null; // Pour le calcul de distance entre deux points

// Métrologie & Physique
let currentGravity = G_STD;
let currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound = 340.29;
let currentMass = 70.0;

// Variables Capteurs & Astro
let lastIMU = { acc: {x:0, y:0, z:0}, pitch: 0, roll: 0 };
let lServH = 0, lLocH = 0; // NTP
let gpsWatchID = null;
let map, marker, trackPolyline;
let isMapInitialized = false;

// =================================================================
// PARTIE 3 : LOGIQUE GPS & STATISTIQUES (COEUR DU PROBLÈME)
// =================================================================

function handleGPSUpdate(pos) {
    const now = Date.now();
    const dt = (now - lastTimestamp) / 1000;
    lastTimestamp = now;

    if (isGpsPaused) return;
    
    // 1. Mise à jour État Actif
    isGpsActive = true;
    $('gps-status').textContent = 'SIGNAL ACTIF (Reception)';

    // 2. Extraction des données (Avec protection contre les NULL)
    const p = pos.coords;
    const newLat = p.latitude;
    const newLon = p.longitude;
    const newAlt = p.altitude !== null ? p.altitude : currentPosition.alt;
    const newAcc = p.accuracy;
    
    // CRITIQUE : Si speed est null (appareil à l'arrêt), on force 0.0
    let newSpd = (p.speed !== null && p.speed >= 0) ? p.speed : 0.0;
    
    // Filtre de bruit à l'arrêt (si vitesse < 0.1 m/s, on considère 0 pour éviter les micro-calculs)
    if (newSpd < 0.1) newSpd = 0.0;

    // 3. Calcul de Distance (Accumulation)
    if (lastGPSPosition) {
        // Utilise turf.js si dispo pour précision, sinon approximation simple
        let dist = 0;
        if (typeof turf !== 'undefined' && turf.distance) {
            const from = turf.point([lastGPSPosition.lon, lastGPSPosition.lat]);
            const to = turf.point([newLon, newLat]);
            dist = turf.distance(from, to, { units: 'kilometers' }) * 1000;
        } else {
            // Fallback simple si turf absent (vitesse * temps)
            dist = newSpd * dt;
        }
        
        // On n'ajoute la distance que si la précision GPS est correcte ou si on bouge
        if (newAcc < 50 || newSpd > 0.5) {
            totalDistanceM += dist;
        }
    }

    // 4. Mise à jour Statistiques
    if (newSpd > maxSpeedMs) maxSpeedMs = newSpd;
    if (newSpd > 0.2) totalMovementTimeS += dt; // On bouge si > 0.2 m/s

    // 5. Sauvegarde État
    currentPosition = { 
        lat: newLat, lon: newLon, alt: newAlt, 
        acc: newAcc, spd_ms: newSpd, heading: p.heading || 0 
    };
    lastGPSPosition = { lat: newLat, lon: newLon };

    // 6. Injection dans UKF (Si disponible)
    if (ukf && ukf.update) {
        // ukf.update(...)
    }
}

function initGPS() {
    if (gpsWatchID) navigator.geolocation.clearWatch(gpsWatchID);
    
    const options = {
        enableHighAccuracy: true,
        maximumAge: 0,
        timeout: 10000
    };

    gpsWatchID = navigator.geolocation.watchPosition(
        handleGPSUpdate,
        (err) => {
            console.warn(`GPS Erreur: ${err.message}`);
            $('gps-status').textContent = `ERREUR: ${err.message}`;
        },
        options
    );
    $('gps-status').textContent = 'ACQUISITION...';
}

// =================================================================
// PARTIE 4 : CAPTEURS IMU (ACCEL/GYRO)
// =================================================================

function initIMUSensors() {
    if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', (event) => {
            if (event.accelerationIncludingGravity) {
                lastIMU.acc.x = event.accelerationIncludingGravity.x || 0;
                lastIMU.acc.y = event.accelerationIncludingGravity.y || 0;
                lastIMU.acc.z = event.accelerationIncludingGravity.z || 0;
            }
            if($('imu-status')) $('imu-status').textContent = 'Actif (Legacy)';
        });
    }
    if (window.DeviceOrientationEvent) {
        window.addEventListener('deviceorientation', (event) => {
            lastIMU.pitch = event.beta || 0;
            lastIMU.roll = event.gamma || 0;
        });
    }
}

// =================================================================
// PARTIE 5 : BOUCLE DE MISE À JOUR DOM (60Hz)
// =================================================================

function updateDashboard() {
    // Incrémenter le temps de session global (indépendamment du GPS)
    const dt = 1/60;
    if (!isGpsPaused) totalSessionTimeS += dt;

    const v_ms = currentPosition.spd_ms;
    const v_kmh = v_ms * KMH_MS;

    // --- 1. AFFICHAGE VITESSE & DISTANCE (CORRIGÉ) ---
    
    // Affichage principal (Gros chiffre)
    $('speed-stable').textContent = dataOrDefault(v_kmh, 1, ' km/h');
    
    // Texte de statut sous la vitesse
    let statusText = "Attente du signal...";
    if (isGpsPaused) statusText = "PAUSE GPS";
    else if (isGpsActive) statusText = (v_kmh < 1.0) ? "À l'arrêt" : "En mouvement";
    $('speed-status-text').textContent = statusText;

    // Détails Vitesse
    $('speed-stable-ms').textContent = dataOrDefault(v_ms, 2, ' m/s');
    $('speed-stable-kms').textContent = dataOrDefault(v_ms / 1000, 4, ' km/s');
    $('speed-3d-inst').textContent = dataOrDefault(v_kmh, 1, ' km/h');
    $('speed-raw-ms').textContent = dataOrDefault(v_ms, 2, ' m/s');
    $('vitesse-max-session').textContent = dataOrDefault(maxSpeedMs * KMH_MS, 1, ' km/h');

    // Moyennes (Correction Division par Zéro)
    const avgMove = totalMovementTimeS > 1 ? (totalDistanceM / totalMovementTimeS) * KMH_MS : 0;
    const avgTotal = totalSessionTimeS > 1 ? (totalDistanceM / totalSessionTimeS) * KMH_MS : 0;
    
    $('speed-avg-moving').textContent = dataOrDefault(avgMove, 1, ' km/h');
    $('speed-avg-total').textContent = dataOrDefault(avgTotal, 1, ' km/h');

    // Distance & Temps
    $('distance-totale').textContent = `${dataOrDefault(totalDistanceM / 1000, 3, ' km')} | ${dataOrDefault(totalDistanceM, 1, ' m')}`;
    $('elapsed-time').textContent = dataOrDefault(totalSessionTimeS, 1, ' s');
    $('movement-time').textContent = dataOrDefault(totalMovementTimeS, 1, ' s');

    // --- 2. AFFICHAGE PHYSIQUE & IMU ---
    
    // Gravité Locale
    const latRad = currentPosition.lat * D2R;
    const g_wgs84 = 9.780327 * (1 + 0.0053024 * Math.sin(latRad)**2) - (3.086e-6 * currentPosition.alt);
    $('local-gravity').textContent = dataOrDefault(g_wgs84, 4, ' m/s²');

    // IMU
    $('acceleration-x').textContent = dataOrDefault(lastIMU.acc.x, 2, ' m/s²');
    $('acceleration-y').textContent = dataOrDefault(lastIMU.acc.y, 2, ' m/s²');
    $('acceleration-z').textContent = dataOrDefault(lastIMU.acc.z, 2, ' m/s²');
    $('inclinaison-pitch').textContent = dataOrDefault(lastIMU.pitch, 1, '°');
    $('roulis-roll').textContent = dataOrDefault(lastIMU.roll, 1, '°');
    
    // Niveau à bulle (Visuel)
    const bubble = $('bubble');
    if (bubble) {
        const dx = Math.max(-30, Math.min(30, lastIMU.roll));
        const dy = Math.max(-30, Math.min(30, lastIMU.pitch));
        bubble.style.transform = `translate(${dx}px, ${dy}px)`;
    }

    // --- 3. AFFICHAGE ASTRO & POSITION ---
    
    // Position
    $('lat-ekf').textContent = dataOrDefault(currentPosition.lat, 6);
    $('lon-ekf').textContent = dataOrDefault(currentPosition.lon, 6);
    $('alt-ekf').textContent = dataOrDefault(currentPosition.alt, 2, ' m');
    $('acc-gps').textContent = dataOrDefault(currentPosition.acc, 1, ' m');

    // Astro (Appel Lib Externe)
    if (typeof window.getSolarData === 'function') {
        try {
            // Utilise l'heure actuelle, corrigée par NTP si dispo
            const now = getCDate(lServH, lLocH) || new Date();
            const astro = window.getSolarData(now, currentPosition.lat, currentPosition.lon, currentPosition.alt);
            
            $('sun-alt').textContent = dataOrDefault(astro.sun.position.altitude * R2D, 2, '°');
            $('sun-azimuth').textContent = dataOrDefault(astro.sun.position.azimuth * R2D, 2, '°');
            
            if (window.formatHours) {
                $('tst-time').textContent = window.formatHours(astro.TST_HRS);
                $('mst-time').textContent = window.formatHours(astro.MST_HRS);
            }
            // Date / Heure
            if ($('local-time-ntp')) $('local-time-ntp').textContent = now.toLocaleTimeString();
            if ($('utc-datetime')) $('utc-datetime').textContent = now.toUTCString();

        } catch (e) {
            // Fail silent
        }
    }
}

// =================================================================
// PARTIE 6 : SYNCHRO TEMPS & CARTE
// =================================================================

function syncH() {
    fetch("https://worldtimeapi.org/api/utc")
        .then(r => r.json())
        .then(d => {
            lServH = d.unixtime * 1000;
            lLocH = Date.now();
        })
        .catch(() => {
            lServH = Date.now(); // Fallback Local
            lLocH = Date.now();
        });
}

function initMap() {
    if (typeof L !== 'undefined' && !isMapInitialized && $('map')) {
        map = L.map('map').setView([currentPosition.lat, currentPosition.lon], 15);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(map);
        marker = L.marker([currentPosition.lat, currentPosition.lon]).addTo(map);
        isMapInitialized = true;
    }
}

function updateMap() {
    if (isMapInitialized && marker) {
        const newLL = [currentPosition.lat, currentPosition.lon];
        marker.setLatLng(newLL);
        if (!isGpsPaused && currentPosition.spd_ms > 1.0) map.panTo(newLL); // Suivre seulement si mouvement
    }
}

// =================================================================
// PARTIE 7 : INITIALISATION GLOBALE
// =================================================================

window.addEventListener('load', () => {
    console.log("🚀 Dashboard V9.1 Started");

    // 1. Gestionnaires Boutons
    if ($('toggle-gps-btn')) {
        $('toggle-gps-btn').addEventListener('click', () => {
            isGpsPaused = !isGpsPaused;
            $('toggle-gps-btn').textContent = isGpsPaused ? "▶️ REPRENDRE" : "⏸️ PAUSE";
            $('toggle-gps-btn').style.background = isGpsPaused ? "#ffc107" : "#28a745";
        });
    }
    
    if ($('reset-all-btn')) {
        $('reset-all-btn').addEventListener('click', () => {
            maxSpeedMs = 0; totalDistanceM = 0; totalMovementTimeS = 0; totalSessionTimeS = 0;
            alert("Statistiques réinitialisées.");
        });
    }

    // 2. Démarrage Services
    if (typeof window.ProfessionalUKF === 'function') ukf = new ProfessionalUKF();
    
    syncH();
    initIMUSensors();
    initGPS();
    initMap();

    // 3. Boucle Principale
    setInterval(() => {
        updateDashboard();
        updateMap();
    }, 1000 / 60); // 60 FPS
});
