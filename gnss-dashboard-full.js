// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET V9.0 (LEGACY API & FIX)
// VERSION : COMPATIBILITÉ MAXIMALE (Anciennes API Capteurs + GPS Force)
// =================================================================

// --- 1. UTILITAIRES & FORMATAGE ---
const $ = id => document.getElementById(id);

const dataOrDefault = (val, decimals, suffix = '') => {
    // Si val est strictement null, undefined ou NaN (mais on accepte 0)
    if (val === undefined || val === null || isNaN(val)) return 'N/A';
    return val.toFixed(decimals).replace('.', ',') + suffix;
};

const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) return 'N/A';
    return val.toExponential(decimals).replace('.', ',') + suffix;
};

// --- 2. CONSTANTES PHYSIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI; 
const C_L = 299792458;      
const G_STD = 9.80665; 
const RHO_AIR = 1.225; 
const V_SOUND_STD = 340.29; 

// --- 3. ÉTAT GLOBAL ---
let ukf = null;
let isGpsPaused = false; 
let lastTimestamp = Date.now();

// État des capteurs (Legacy Mode)
let imuData = { accX: 0, accY: 0, accZ: 0, rotAlpha: 0, rotBeta: 0, rotGamma: 0 };
let currentPos = { 
    lat: 43.284473, // Vos coordonnées initiales
    lon: 5.358518, 
    alt: 100.00, 
    spd: 0.0, 
    acc: 0 
};

// Variables Statistiques
let maxSpeedSession = 0.0;
let totalDistance = 0.0;
let movingTime = 0.0;
let sessionTime = 0.0;

// Variables Astro/Météo (Cache)
let lServH = 0; // Heure serveur
let lLocH = 0;
let currentGravity = G_STD;

// =================================================================
// PARTIE 4 : GESTION DES CAPTEURS (ANCIENNES API "LEGACY")
// =================================================================

function initSensorsLegacy() {
    // Écouteur standard pour l'accéléromètre (incluant gravité)
    if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', (event) => {
            if (event.accelerationIncludingGravity) {
                imuData.accX = event.accelerationIncludingGravity.x || 0;
                imuData.accY = event.accelerationIncludingGravity.y || 0;
                imuData.accZ = event.accelerationIncludingGravity.z || 0;
                
                // Mise à jour immédiate du DOM IMU pour débug
                if($('acceleration-x')) $('acceleration-x').textContent = dataOrDefault(imuData.accX, 2, ' m/s²');
                if($('acceleration-y')) $('acceleration-y').textContent = dataOrDefault(imuData.accY, 2, ' m/s²');
                if($('acceleration-z')) $('acceleration-z').textContent = dataOrDefault(imuData.accZ, 2, ' m/s²');
                if($('imu-status')) $('imu-status').textContent = 'Actif (Legacy)';
            }
        });
    } else {
        if($('imu-status')) $('imu-status').textContent = 'Non supporté';
    }

    // Écouteur standard pour l'orientation (Gyro/Boussole)
    if (window.DeviceOrientationEvent) {
        window.addEventListener('deviceorientation', (event) => {
            imuData.rotAlpha = event.alpha || 0; // Cap (Heading)
            imuData.rotBeta = event.beta || 0;   // Pitch
            imuData.rotGamma = event.gamma || 0; // Roll
            
            // Mise à jour Niveau à Bulle
            updateSpiritLevel(imuData.rotBeta, imuData.rotGamma);
            if($('heading-display')) $('heading-display').textContent = dataOrDefault(imuData.rotAlpha, 0, '°');
        });
    }
}

// Fonction Visuelle Niveau à Bulle
function updateSpiritLevel(pitch, roll) {
    const bubble = $('bubble');
    if (bubble) {
        const maxOffset = 35; 
        const dx = Math.max(-maxOffset, Math.min(maxOffset, roll)); 
        const dy = Math.max(-maxOffset, Math.min(maxOffset, pitch)); 
        bubble.style.transform = `translate(${dx}px, ${dy}px)`;
    }
    if($('inclinaison-pitch')) $('inclinaison-pitch').textContent = dataOrDefault(pitch, 1, '°');
    if($('roulis-roll')) $('roulis-roll').textContent = dataOrDefault(roll, 1, '°');
}

// =================================================================
// PARTIE 5 : GESTION GPS & UKF
// =================================================================

function initGPS() {
    if ("geolocation" in navigator) {
        navigator.geolocation.watchPosition(
            (position) => {
                if (isGpsPaused) return;
                
                // Récupération des données brutes
                const p = position.coords;
                
                // Calcul du delta temps
                const now = Date.now();
                const dt = (now - lastTimestamp) / 1000;
                lastTimestamp = now;

                // Mise à jour de l'état
                currentPos.lat = p.latitude;
                currentPos.lon = p.longitude;
                currentPos.alt = p.altitude !== null ? p.altitude : currentPos.alt; // Garde l'ancienne si null
                currentPos.acc = p.accuracy;
                
                // Calcul Vitesse (Si le GPS renvoie null, on met 0)
                let rawSpeed = p.speed !== null ? p.speed : 0;
                currentPos.spd = rawSpeed;

                // Stats Session
                if (rawSpeed > 0.5) { // Seuil de mouvement
                    maxSpeedSession = Math.max(maxSpeedSession, rawSpeed);
                    movingTime += dt;
                    totalDistance += rawSpeed * dt;
                }

                // Mise à jour UKF (Si disponible)
                if (ukf && ukf.update) {
                    // ukf.update(...) // À intégrer si vous utilisez la librairie complète
                }

                // Feedback Visuel Immédiat
                if($('gps-status')) $('gps-status').textContent = 'SIGNAL OK (Actif)';
                if($('acc-gps')) $('acc-gps').textContent = dataOrDefault(p.accuracy, 1, ' m');
                
            },
            (err) => {
                console.warn("Erreur GPS:", err);
                if($('gps-status')) $('gps-status').textContent = 'PERTE SIGNAL / ERREUR';
            },
            { enableHighAccuracy: true, maximumAge: 0, timeout: 5000 }
        );
    } else {
        if($('gps-status')) $('gps-status').textContent = 'GPS NON DISPONIBLE';
    }
}

// =================================================================
// PARTIE 6 : CALCULS PHYSIQUES & ASTRO (Update Loop)
// =================================================================

function updateDashboard() {
    const now = new Date();
    const dt = 1/60; // 60Hz
    sessionTime += dt;

    // --- 1. Calculs Gravité Locale (WGS84) ---
    // g = ge * (1 + beta*sin^2(lat)) - free_air_correction
    const latRad = currentPos.lat * D2R;
    const sin2 = Math.sin(latRad) ** 2;
    const g_0 = 9.780327 * (1 + 0.0053024 * sin2);
    currentGravity = g_0 - (3.086e-6 * currentPos.alt);

    // --- 2. Calculs Vitesse & Relativité ---
    const v = currentPos.spd; // m/s
    const v_kmh = v * 3.6;
    const mach = v / V_SOUND_STD;
    const gamma = 1 / Math.sqrt(1 - (v**2 / C_L**2));
    
    // --- 3. MISE À JOUR DOM (Affichage) ---
    
    // Vitesse
    if($('speed-stable')) $('speed-stable').textContent = dataOrDefault(v_kmh, 1, ' km/h');
    if($('speed-stable-ms')) $('speed-stable-ms').textContent = dataOrDefault(v, 2, ' m/s');
    if($('vitesse-max-session')) $('vitesse-max-session').textContent = dataOrDefault(maxSpeedSession * 3.6, 1, ' km/h');
    
    // Temps & Session
    if($('elapsed-time')) $('elapsed-time').textContent = dataOrDefault(sessionTime, 1, ' s');
    if($('movement-time')) $('movement-time').textContent = dataOrDefault(movingTime, 1, ' s');
    if($('local-time-ntp')) $('local-time-ntp').textContent = now.toLocaleTimeString();
    
    // Date UTC (Si possible)
    if($('utc-datetime')) {
        $('utc-datetime').textContent = now.toISOString().replace('T', ' ').split('.')[0] + ' UTC';
    }

    // Physique
    if($('local-gravity')) $('local-gravity').textContent = dataOrDefault(currentGravity, 4, ' m/s²');
    if($('mach-number')) $('mach-number').textContent = dataOrDefault(mach, 4);
    if($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(gamma, 8);
    if($('kinetic-energy')) $('kinetic-energy').textContent = dataOrDefault(0.5 * 70 * v*v, 1, ' J');

    // Position EKF (Données GPS brutes ou filtrées)
    if($('lat-ekf')) $('lat-ekf').textContent = currentPos.lat.toFixed(6);
    if($('lon-ekf')) $('lon-ekf').textContent = currentPos.lon.toFixed(6);
    if($('alt-ekf')) $('alt-ekf').textContent = dataOrDefault(currentPos.alt, 1, ' m');

    // --- 4. ASTRO (Appel Lib Externe) ---
    if (typeof window.getSolarData === 'function') {
        try {
            const astro = window.getSolarData(now, currentPos.lat, currentPos.lon, currentPos.alt);
            
            if($('tst-time')) $('tst-time').textContent = window.formatHours ? window.formatHours(astro.TST_HRS) : 'N/A';
            if($('mst-time')) $('mst-time').textContent = window.formatHours ? window.formatHours(astro.MST_HRS) : 'N/A';
            if($('sun-alt')) $('sun-alt').textContent = dataOrDefault(astro.sun.position.altitude * R2D, 2, '°');
            if($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(astro.sun.position.azimuth * R2D, 2, '°');
            
            // Lune
            if($('moon-phase-name') && window.getMoonPhaseName) $('moon-phase-name').textContent = window.getMoonPhaseName(astro.moon.illumination.phase);
            if($('moon-alt')) $('moon-alt').textContent = dataOrDefault(astro.moon.position.altitude * R2D, 2, '°');
            
        } catch (e) {
            // Silencieux pour ne pas spammer la console
        }
    }
}

// =================================================================
// PARTIE 7 : INITIALISATION
// =================================================================

window.addEventListener('load', () => {
    console.log("🚀 Démarrage Dashboard V9.0 (Legacy Mode)");

    // 1. Initialiser UKF si présent
    if (typeof window.ProfessionalUKF === 'function') {
        ukf = new ProfessionalUKF();
    }

    // 2. Initialiser les capteurs (Méthode Ancienne/Standard)
    initSensorsLegacy();

    // 3. Initialiser GPS
    initGPS();

    // 4. Initialiser la synchro temps (Simple Fetch)
    fetch("https://worldtimeapi.org/api/utc")
        .then(r => r.json())
        .then(d => { lServH = d.unixtime * 1000; lLocH = Date.now(); })
        .catch(e => console.log("Mode Hors Ligne (Temps)"));

    // 5. Lancer la boucle principale (60 FPS)
    setInterval(updateDashboard, 1000 / 60);
    
    // 6. Gestionnaires d'événements UI
    if($('toggle-gps-btn')) {
        $('toggle-gps-btn').addEventListener('click', () => {
            isGpsPaused = !isGpsPaused;
            $('toggle-gps-btn').textContent = isGpsPaused ? "▶️ REPRENDRE" : "⏸️ PAUSE";
        });
    }
    
    if($('reset-all-btn')) {
        $('reset-all-btn').addEventListener('click', () => {
            maxSpeedSession = 0;
            totalDistance = 0;
            sessionTime = 0;
            alert("Session réinitialisée");
        });
    }
});
