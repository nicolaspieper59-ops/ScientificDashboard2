// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET V8.0 (FUSION & CORRECTIONS CRITIQUES)
// CORRECTIONS: Fallback NTP/UTC, Activation IMU Legacy, Intégration Astro complète.
// DÉPENDANCES NÉCESSAIRES (doivent être chargées dans votre HTML):
// - math.min.js
// - ukf-lib.js (avec ProfessionalUKF)
// - astro.js (avec getSolarData et getMoonPhaseName)
// =================================================================

// --- BLOC 1 : CONSTANTES ET UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);
const D2R = Math.PI / 180, R2D = 180 / Math.PI; 
const C_L = 299792458;      
const KMH_MS = 3.6; 
const G_U = 6.67430e-11;
const G_STD = 9.80665; 
const V_SOUND_STD = 340.29; 
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";
const DOM_SLOW_UPDATE_MS = 2000; 

// Formatteurs robustes (issus de gnss-dashboard-full (18).js)
const dataOrDefault = (val, decimals, suffix = '', fallback = null, forceZero = true) => {
    if (val === 'N/A') return 'N/A'; 
    if (val === undefined || val === null || isNaN(val) || (typeof val === 'number' && Math.abs(val) < 1e-18)) {
        if (fallback !== null) return fallback;
        if (forceZero) {
            // S'assure d'avoir '0,00' ou '0'
            const zeroFormat = (decimals === 0 ? '0' : '0.' + Array(decimals).fill('0').join('')) + suffix;
            return zeroFormat.replace('.', ',');
        }
        return 'N/A';
    }
    return val.toFixed(decimals).replace('.', ',') + suffix;
};
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) return 'N/A';
    return val.toExponential(decimals).replace('.', ',') + suffix;
};

// --- BLOC 2 : ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE ---
let ukf = null;
let isGpsPaused = true; // Initialisé sur Pause
let lastTimestamp = Date.now();

// Coordonnées utilisateur mises à jour et état du filtre
let currentPosition = { 
    lat: 43.284501,   // VOTRE LATITUDE
    lon: 5.358674,    // VOTRE LONGITUDE
    alt: 100.00,      // VOTRE ALTITUDE
    acc: 10.0,        // Précision (m)
    spd: 0.0          // Vitesse (m/s)
};

// Variables Statistiques et État
let maxSpeedSession = 0.0;
let totalDistance = 0.0;
let movingTime = 0.0;
let sessionTime = 0.0;
let currentMass = 70.0;
let currentGravity = G_STD;

// Variables NTP/Astro
let lServH = null; // Timestamp UTC du serveur
let lLocH = null;  // Timestamp local de l'appel NTP
let imuData = { accX: 0, accY: 0, accZ: 0, rotBeta: 0, rotGamma: 0 };


// =================================================================
// BLOC 3 : SYNCHRONISATION HORLOGE (NTP) - FIX CRITIQUE
// Ajoute un fallback pour débloquer Astro même si l'API est inaccessible.
// =================================================================

function getCDate(serverH, localH) {
    if (serverH === null || localH === null) return null;
    return new Date(serverH + (Date.now() - localH));
}

function syncH() {
    return fetch(SERVER_TIME_ENDPOINT)
        .then(r => r.json())
        .then(d => { 
            lServH = d.unixtime * 1000; 
            lLocH = Date.now(); 
            if ($('utc-datetime')) {
                $('utc-datetime').textContent = getCDate(lServH, lLocH).toISOString().replace('T', ' ').split('.')[0] + ' UTC';
            }
            console.log("NTP Synchronisation OK.");
        })
        .catch(e => {
            console.warn("🔴 Échec de la synchro NTP. Utilisation de l'heure locale comme référence UTC.");
            // --- FALLBACK CRITIQUE ---
            const now = Date.now();
            lServH = now; // Utiliser l'heure locale comme référence UTC
            lLocH = now;
            if ($('utc-datetime')) {
                $('utc-datetime').textContent = new Date(now).toISOString().replace('T', ' ').split('.')[0] + ' UTC (FALLBACK)';
            }
        });
}


// =================================================================
// BLOC 4 : GESTION DES CAPTEURS (IMU Legacy / GPS)
// =================================================================

function initSensorsLegacy() {
    // IMU - Accéléromètre et Gravité (via devicemotion)
    if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', (event) => {
            if (event.accelerationIncludingGravity) {
                // Pour l'affichage, nous utilisons l'accélération + gravité
                imuData.accX = event.accelerationIncludingGravity.x || 0;
                imuData.accY = event.accelerationIncludingGravity.y || 0;
                imuData.accZ = event.accelerationIncludingGravity.z || 0;
            }
            if (event.rotationRate) {
                // Pour le gyroscope
                imuData.rotBeta = event.rotationRate.beta || 0;   // Pitch
                imuData.rotGamma = event.rotationRate.gamma || 0; // Roll
            }
             if($('imu-status')) $('imu-status').textContent = 'Actif (Legacy)';
        });
    } else {
        if($('imu-status')) $('imu-status').textContent = 'Non supporté';
    }
}

function initGPS() {
    if ("geolocation" in navigator) {
        navigator.geolocation.watchPosition(
            (position) => {
                if (isGpsPaused) return;
                
                const p = position.coords;
                const now = Date.now();
                const dt = (now - lastTimestamp) / 1000;
                lastTimestamp = now;

                currentPosition.lat = p.latitude;
                currentPosition.lon = p.longitude;
                currentPosition.alt = p.altitude !== null ? p.altitude : currentPosition.alt; 
                currentPosition.acc = p.accuracy;
                let rawSpeed = p.speed !== null ? p.speed : 0;
                currentPosition.spd = rawSpeed;

                // Mise à jour des stats
                if (rawSpeed > 0.05) { 
                    maxSpeedSession = Math.max(maxSpeedSession, rawSpeed);
                    movingTime += dt;
                    totalDistance += rawSpeed * dt;
                }

                if($('gps-status')) $('gps-status').textContent = 'SIGNAL OK (Actif)';
                if($('acc-gps')) $('acc-gps').textContent = dataOrDefault(p.accuracy, 1, ' m');
                
                // Ici, vous lanceriez ukf.update avec les données (p.latitude, p.longitude, rawSpeed, p.accuracy)

            },
            (err) => {
                if($('gps-status')) $('gps-status').textContent = 'PERTE SIGNAL / ERREUR';
            },
            { enableHighAccuracy: true, maximumAge: 0, timeout: 5000 }
        );
    } else {
        if($('gps-status')) $('gps-status').textContent = 'GPS NON DISPONIBLE';
    }
}


// =================================================================
// BLOC 5 : MISE À JOUR DU DOM (60 Hz)
// =================================================================

function updateDashboardDOM() {
    const now = new Date();
    const dt = 1/60; 
    sessionTime += dt;

    // --- 1. CALCULS PHYSIQUES ---
    // Gravité Locale (WGS84) - Réf: ukf-lib (3).js
    const latRad = currentPosition.lat * D2R;
    const sin2 = Math.sin(latRad) ** 2;
    const G_E = 9.780327; // Gravité à l'équateur (m/s²)
    const g_0 = G_E * (1 + 0.0053024 * sin2);
    currentGravity = g_0 - (3.086e-6 * currentPosition.alt);
    if($('local-gravity')) $('local-gravity').textContent = dataOrDefault(currentGravity, 4, ' m/s²');

    // Vitesse & Relativité
    const v = currentPosition.spd; // m/s
    const v_kmh = v * KMH_MS;
    const mach = v / V_SOUND_STD;
    const gamma = 1 / Math.sqrt(1 - (v**2 / C_L**2));
    
    // Énergie (E = 1/2*m*v^2, pour la partie non-relativiste)
    const kineticEnergy = 0.5 * currentMass * v * v;
    
    // --- 2. MISE À JOUR DOM ---

    // Temps & Session
    if($('local-time-ntp')) $('local-time-ntp').textContent = now.toLocaleTimeString('fr-FR');
    if($('elapsed-time')) $('elapsed-time').textContent = dataOrDefault(sessionTime, 1, ' s');
    if($('movement-time')) $('movement-time').textContent = dataOrDefault(movingTime, 1, ' s');

    // Vitesse & Distance
    if($('speed-stable')) $('speed-stable').textContent = dataOrDefault(v_kmh, 1, ' km/h');
    if($('vitesse-max-session')) $('vitesse-max-session').textContent = dataOrDefault(maxSpeedSession * KMH_MS, 1, ' km/h');
    if($('distance-total-3d')) $('distance-total-3d').textContent = dataOrDefault(totalDistance / 1000, 3, ' km') + ' | ' + dataOrDefault(totalDistance, 2, ' m');

    // Physique & Relativité
    if($('mach-number')) $('mach-number').textContent = dataOrDefault(mach, 4);
    if($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(gamma, 4);
    if($('kinetic-energy-j')) $('kinetic-energy-j').textContent = dataOrDefault(kineticEnergy, 2, ' J');
    if($('percentage-light-speed')) $('percentage-light-speed').textContent = dataOrDefault(v / C_L * 100, 2, 'e+0 %');

    // Position EKF (vos dernières coordonnées)
    if($('lat-ekf')) $('lat-ekf').textContent = dataOrDefault(currentPosition.lat, 6);
    if($('lon-ekf')) $('lon-ekf').textContent = dataOrDefault(currentPosition.lon, 6);
    if($('alt-ekf')) $('alt-ekf').textContent = dataOrDefault(currentPosition.alt, 2, ' m');

    // IMU
    if($('acceleration-x')) $('acceleration-x').textContent = dataOrDefault(imuData.accX, 2, ' m/s²');
    if($('acceleration-y')) $('acceleration-y').textContent = dataOrDefault(imuData.accY, 2, ' m/s²');
    if($('acceleration-z')) $('acceleration-z').textContent = dataOrDefault(imuData.accZ, 2, ' m/s²');
    if($('inclinaison-pitch')) $('inclinaison-pitch').textContent = dataOrDefault(imuData.rotBeta, 1, '°');
    if($('roulis-roll')) $('roulis-roll').textContent = dataOrDefault(imuData.rotGamma, 1, '°');


    // --- 3. CALCULS ASTRO (DÉPEND DE astro.js) ---
    const currentTime = getCDate(lServH, lLocH);

    if (currentTime && typeof window.getSolarData === 'function') {
        try {
            // Nécessite l'heure du serveur (lServH)
            const astroData = window.getSolarData(currentTime, currentPosition.lat, currentPosition.lon, currentPosition.alt);
            
            // Mise à jour Temps Solaire
            if ($('date-astro')) $('date-astro').textContent = currentTime.toLocaleDateString('fr-FR');
            if ($('tst-time')) $('tst-time').textContent = window.formatHours ? window.formatHours(astroData.TST_HRS) : dataOrDefault(astroData.TST_HRS, 4);
            if ($('mst-time')) $('mst-time').textContent = window.formatHours ? window.formatHours(astroData.MST_HRS) : dataOrDefault(astroData.MST_HRS, 4);
            if ($('equation-of-time')) $('equation-of-time').textContent = dataOrDefault(astroData.EOT_MIN, 2, ' min');
            if ($('noon-solar-utc')) $('noon-solar-utc').textContent = astroData.NOON_SOLAR_UTC; // Format H:M:S UTC

            // Mise à jour Soleil
            if ($('sun-alt')) $('sun-alt').textContent = dataOrDefault(astroData.sun.position.altitude * R2D, 2, '°');
            if ($('sun-azimuth')) $('sun-azimuth').textContent = dataOrDefault(astroData.sun.position.azimuth * R2D, 2, '°');
            
            // Mise à jour Lune
            if ($('moon-phase-name') && typeof window.getMoonPhaseName === 'function') {
                $('moon-phase-name').textContent = window.getMoonPhaseName(astroData.moon.illumination.phase);
            }
            if ($('moon-alt')) $('moon-alt').textContent = dataOrDefault(astroData.moon.position.altitude * R2D, 2, '°');
            
        } catch (e) {
             // Si une des fonctions Astro échoue, on loggue et laisse N/A
             console.error("Erreur dans les calculs Astro:", e);
        }
    } else {
         // Fallback Astro si lServH est toujours null ou astro.js est absent
         if ($('utc-datetime')) $('utc-datetime').textContent = 'N/A';
         // Note: Les champs Astro dans le DOM seront à N/A ou à 0.0 par défaut (selon votre HTML)
    }

}


// =================================================================
// BLOC 6 : INITIALISATION DU SYSTÈME
// =================================================================

window.addEventListener('load', () => {
    console.log("🚀 Démarrage Dashboard V8.0 Professionnel Complet.");

    // 1. Initialiser UKF
    if (typeof window.ProfessionalUKF === 'function' && typeof math !== 'undefined') { 
        ukf = new ProfessionalUKF();
    } else {
        console.warn("UKF désactivé: math.js ou ProfessionalUKF introuvable.");
    }

    // 2. Initialiser les capteurs (Legacy/Standard)
    initSensorsLegacy();

    // 3. Initialiser GPS
    initGPS();
    
    // 4. Démarrer la synchro NTP (avec son fallback critique)
    syncH();

    // 5. Lancer les boucles de mise à jour
    // Boucle rapide (60 FPS) pour les valeurs dynamiques (Vitesse, IMU, Temps)
    setInterval(updateDashboardDOM, 1000 / 60); 

    // Boucle lente (pour la météo, si implémentée)
    // setInterval(fetchWeather, DOM_SLOW_UPDATE_MS); 

    // 6. Gestionnaires d'événements UI
    // Toggle GPS
    const toggleGpsBtn = $('toggle-gps-btn');
    if(toggleGpsBtn) {
        toggleGpsBtn.addEventListener('click', () => {
            isGpsPaused = !isGpsPaused;
            toggleGpsBtn.textContent = isGpsPaused ? "▶️ REPRENDRE" : "⏸️ PAUSE";
        });
    }
    // Assurez-vous que l'état initial du bouton correspond à l'état de la variable
    if(toggleGpsBtn) toggleGpsBtn.textContent = isGpsPaused ? "▶️ REPRENDRE" : "⏸️ PAUSE";

    // Autres boutons de réinitialisation si vous les aviez dans l'HTML...

});
