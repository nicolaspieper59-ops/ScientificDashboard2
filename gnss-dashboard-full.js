// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET V10.0 (ULTIMATE FIX)
// CORRECTIONS: IMU Geste Utilisateur, Synchronisation UTC Immédiate, Initialisation DOM à 0.0.
// DÉPENDANCES CRITIQUES: math.min.js, ukf-lib.js, astro.js, leaflet.js, turf.min.js (à charger dans l'HTML)
// =================================================================

// =================================================================
// PARTIE 1 : UTILITAIRES ET FORMATAGE
// =================================================================

const $ = id => document.getElementById(id);

// Constantes fondamentales
const D2R = Math.PI / 180, R2D = 180 / Math.PI; 
const KMH_MS = 3.6;         
const C_L = 299792458;      
const G_STD = 9.80665;
const RHO_SEA_LEVEL = 1.225; 
const TEMP_SEA_LEVEL_K = 288.15; 
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

// Formatage robuste : Affiche 0.00 ou la valeur par défaut au lieu de N/A ou --
const dataOrDefault = (val, decimals, suffix = '') => {
    // Si la valeur est explicitement 'N/A' (chaîne)
    if (val === 'N/A') return 'N/A'; 
    
    // Si la valeur est undefined, null, NaN, ou très proche de zéro, on force l'affichage '0.00'
    if (val === undefined || val === null || isNaN(val) || (typeof val === 'number' && Math.abs(val) < 1e-18)) {
        const zeroFormat = (decimals === 0 ? '0' : '0.' + Array(decimals).fill('0').join(''));
        return zeroFormat.replace('.', ',') + suffix;
    }
    return val.toFixed(decimals).replace('.', ',') + suffix;
};

const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) return '0,00e+0' + suffix;
    return val.toExponential(decimals).replace('.', ',').replace('e', 'e') + suffix;
};

// Récupère l'heure corrigée UTC (Critique pour l'Astro)
function getCDate(serverH, localH) {
    if (serverH === 0 || localH === 0) return null;
    return new Date(serverH + (Date.now() - localH));
}

// =================================================================
// PARTIE 2 : ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE
// =================================================================

let ukf = null;
let isGpsPaused = true; 
let isGpsActive = false; 
let isImuActive = false; // Nouvelle variable d'état
let gpsWatchID = null;

let currentPosition = { 
    lat: 43.284578, lon: 5.358713, alt: 100.00, 
    acc: 0.0, spd_ms: 0.0, heading: 0.0 
};

let maxSpeedMs = 0.0;
let totalDistanceM = 0.0;
let totalMovementTimeS = 0.0;
let totalSessionTimeS = 0.0;
let lastTimestamp = Date.now();
let lastGPSPosition = null;

let lastIMU = { acc: {x:0, y:0, z:0}, pitch: 0.0, roll: 0.0 };
let lServH = 0, lLocH = 0; // NTP

// =================================================================
// PARTIE 3 : SYNCHRONISATION HORLOGE (UTC/NTP)
// =================================================================

function syncH() {
    // Utilise un fallback immédiat pour l'affichage UTC initial
    const now = Date.now();
    lServH = now; 
    lLocH = now;

    // Mise à jour immédiate du DOM avec l'heure locale comme fallback UTC
    const localFallbackDate = new Date(now).toISOString().replace('T', ' ').split('.')[0] + ' UTC (FALLBACK)';
    if ($('utc-datetime')) $('utc-datetime').textContent = localFallbackDate;

    // Tentative de synchronisation API
    fetch(SERVER_TIME_ENDPOINT)
        .then(r => r.json())
        .then(d => {
            lServH = d.unixtime * 1000; 
            lLocH = Date.now(); 
            const syncDate = getCDate(lServH, lLocH).toISOString().replace('T', ' ').split('.')[0] + ' UTC';
            if ($('utc-datetime')) $('utc-datetime').textContent = syncDate;
            console.log("NTP Synchronisation OK.");
        })
        .catch(e => {
            console.warn("🔴 Échec synchro NTP. Le fallback est maintenu.");
        });
}

// =================================================================
// PARTIE 4 : CAPTEURS IMU (GESTION DE L'ANCIENNE API)
// =================================================================

// NOTE: Cette fonction doit être appelée UNIQUEMENT par un geste utilisateur (ex: clic)
function initIMUSensors(retry = false) {
    if (isImuActive) return;

    const startIMU = () => {
        if (window.DeviceMotionEvent) {
            window.addEventListener('devicemotion', (event) => {
                if (event.accelerationIncludingGravity) {
                    lastIMU.acc.x = event.accelerationIncludingGravity.x || 0;
                    lastIMU.acc.y = event.accelerationIncludingGravity.y || 0;
                    lastIMU.acc.z = event.accelerationIncludingGravity.z || 0;
                }
                isImuActive = true;
                $('imu-status').textContent = 'Actif (Legacy)';
            }, { once: true }); // Écoute une seule fois pour garantir la première lecture
        }
        
        if (window.DeviceOrientationEvent) {
            window.addEventListener('deviceorientation', (event) => {
                lastIMU.pitch = event.beta || 0;
                lastIMU.roll = event.gamma || 0;
                isImuActive = true;
                $('imu-status').textContent = 'Actif (Legacy)';
            }, { once: true }); 
        }

        // Si l'écouteur a été ajouté sans erreur, on passe à actif (même si les données sont 0)
        if (!retry) $('imu-status').textContent = 'Actif (Initialisation)';
        
    };

    // Pour iOS 13+ sur Safari, il faut demander la permission
    if (typeof DeviceMotionEvent.requestPermission === 'function') {
        DeviceMotionEvent.requestPermission()
            .then(permissionState => {
                if (permissionState === 'granted') {
                    startIMU();
                } else {
                    $('imu-status').textContent = 'Refusé (Permission requise)';
                }
            })
            .catch(error => {
                 // Si la permission est rejetée ou si ce n'est pas iOS, on tente l'approche standard
                startIMU();
            });
    } else {
        // Navigateurs Android et ancienne API (la vôtre)
        startIMU();
    }
}


// =================================================================
// PARTIE 5 : GESTION GPS & STATISTIQUES (FIX AFFICHAGE VITESSE)
// =================================================================

function handleGPSUpdate(pos) {
    const now = Date.now();
    const dt = (now - lastTimestamp) / 1000;
    lastTimestamp = now;

    if (isGpsPaused) return;

    isGpsActive = true;
    $('statut-gps-acquisition').textContent = 'SIGNAL OK'; 

    const p = pos.coords;
    const newLat = p.latitude;
    const newLon = p.longitude;
    const newAlt = p.altitude !== null ? p.altitude : currentPosition.alt;
    const newAcc = p.accuracy;
    
    // CRITIQUE : Assure que newSpd est au moins 0.0 si null (pour éviter les '-- m/s')
    let newSpd = (p.speed !== null && p.speed >= 0) ? p.speed : 0.0;
    
    // Filtre de bruit : Si vitesse < 0.1 m/s (0.36 km/h), on considère l'arrêt pour la stabilité
    if (newSpd < 0.1) newSpd = 0.0;

    // --- Calcul de Distance/Temps ---
    if (newSpd > maxSpeedMs) maxSpeedMs = newSpd;
    if (newSpd > 0.1) totalMovementTimeS += dt; 

    // Simulation de distance simplifiée (en l'absence de turf.js)
    totalDistanceM += newSpd * dt;

    // Sauvegarde État
    currentPosition = { lat: newLat, lon: newLon, alt: newAlt, acc: newAcc, spd_ms: newSpd, heading: p.heading || 0 };
    lastGPSPosition = { lat: newLat, lon: newLon };

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
            $('statut-gps-acquisition').textContent = `ERREUR: ${err.code}`;
        },
        options
    );
    $('statut-gps-acquisition').textContent = 'ACQUISITION...';
}

// =================================================================
// PARTIE 6 : INITIALISATION DOM PAR DÉFAUT (FIX N/A)
// =================================================================

function initDOMDefaults() {
    // Initialise tous les champs importants à 0.0 pour éviter les 'N/A' et '--' au démarrage

    // Vitesse
    const speedFields = ['speed-stable', 'speed-stable-ms', 'speed-stable-kms', 'speed-3d-inst', 'speed-raw-ms', 'vitesse-max-session', 'speed-avg-moving', 'speed-avg-total'];
    speedFields.forEach(id => {
        if ($(id)) $(id).textContent = dataOrDefault(0, 1, (id.includes('km/h') ? ' km/h' : (id.includes('m/s') ? ' m/s' : '')));
    });

    // Dynamique & Forces (g/Accélération)
    if ($('local-gravity')) $('local-gravity').textContent = dataOrDefault(G_STD, 4, ' m/s²');
    const forceFields = ['force-g-long', 'acceleration-long', 'vertical-speed', 'acceleration-vert-imu', 'force-g-vert', 'angular-speed', 'kinetic-energy', 'mechanical-power', 'coriolis-force'];
    forceFields.forEach(id => {
        if ($(id)) $(id).textContent = dataOrDefault(0, 2, (id.includes('J') ? ' J' : (id.includes('N') ? ' N' : (id.includes('W') ? ' W' : ''))));
    });

    // IMU (Utilise N/A si pas de détection, mais 0.0 si détection et à l'arrêt)
    if ($('imu-status')) $('imu-status').textContent = 'Inactif';
    const imuFields = ['acceleration-x', 'acceleration-y', 'acceleration-z', 'mag-x', 'mag-y', 'mag-z'];
    imuFields.forEach(id => {
        if ($(id)) $(id).textContent = 'N/A'; // N/A est préférable ici tant que le capteur n'est pas actif
    });
}

// =================================================================
// PARTIE 7 : BOUCLE DE MISE À JOUR DOM (60Hz)
// =================================================================

function updateDashboard() {
    const dt = 1/60;
    if (!isGpsPaused) totalSessionTimeS += dt;

    const v_ms = currentPosition.spd_ms;
    const v_kmh = v_ms * KMH_MS;

    // --- 1. MISE À JOUR VITESSE & DISTANCE ---
    $('speed-stable').textContent = dataOrDefault(v_kmh, 1, ' km/h');
    $('speed-stable-ms').textContent = dataOrDefault(v_ms, 2, ' m/s');
    $('vitesse-max-session').textContent = dataOrDefault(maxSpeedMs * KMH_MS, 1, ' km/h');
    
    // Moyennes (Le totalMovementTimeS > 1 permet d'éviter la division par zéro dans le cas réel où totalMovementTimeS serait 0)
    const avgMove = totalMovementTimeS > 1 ? (totalDistanceM / totalMovementTimeS) * KMH_MS : 0;
    const avgTotal = totalSessionTimeS > 1 ? (totalDistanceM / totalSessionTimeS) * KMH_MS : 0;
    $('speed-avg-moving').textContent = dataOrDefault(avgMove, 1, ' km/h');
    $('speed-avg-total').textContent = dataOrDefault(avgTotal, 1, ' km/h');

    $('distance-totale').textContent = `${dataOrDefault(totalDistanceM / 1000, 3, ' km')} | ${dataOrDefault(totalDistanceM, 1, ' m')}`;
    $('elapsed-time').textContent = dataOrDefault(totalSessionTimeS, 1, ' s');
    $('movement-time').textContent = dataOrDefault(totalMovementTimeS, 1, ' s');

    // --- 2. MISE À JOUR PHYSIQUE & IMU ---
    
    // Gravité Locale
    const latRad = currentPosition.lat * D2R;
    const g_wgs84 = 9.780327 * (1 + 0.0053024 * Math.sin(latRad)**2) - (3.086e-6 * currentPosition.alt);
    $('local-gravity').textContent = dataOrDefault(g_wgs84, 4, ' m/s²');

    // Énergie Cinétique
    const kineticEnergy = 0.5 * currentMass * v_ms * v_ms;
    $('kinetic-energy').textContent = dataOrDefault(kineticEnergy, 2, ' J');

    // IMU (seulement si actif)
    if (isImuActive) {
        $('acceleration-x').textContent = dataOrDefault(lastIMU.acc.x, 2, ' m/s²');
        $('acceleration-y').textContent = dataOrDefault(lastIMU.acc.y, 2, ' m/s²');
        $('acceleration-z').textContent = dataOrDefault(lastIMU.acc.z, 2, ' m/s²');
        $('inclinaison-pitch').textContent = dataOrDefault(lastIMU.pitch, 1, '°');
        $('roulis-roll').textContent = dataOrDefault(lastIMU.roll, 1, '°');
    }

    // --- 3. MISE À JOUR ASTRO & POSITION (DÉPEND DE astro.js) ---
    $('lat-ekf').textContent = dataOrDefault(currentPosition.lat, 6);
    $('lon-ekf').textContent = dataOrDefault(currentPosition.lon, 6);
    $('alt-ekf').textContent = dataOrDefault(currentPosition.alt, 2, ' m');
    $('acc-gps').textContent = dataOrDefault(currentPosition.acc, 1, ' m');

    // Astro
    const now = getCDate(lServH, lLocH);
    if (now && typeof window.getSolarData === 'function') {
        try {
            const astro = window.getSolarData(now, currentPosition.lat, currentPosition.lon, currentPosition.alt);
            
            $('date-display-astro').textContent = now.toLocaleDateString('fr-FR');
            $('sun-alt').textContent = dataOrDefault(astro.sun.position.altitude * R2D, 2, '°');
            $('sun-azimuth').textContent = dataOrDefault(astro.sun.position.azimuth * R2D, 2, '°');

            // Le champ TST/MST nécessite le formatage spécifique de votre astro.js
            if (window.formatHours) {
                $('tst-time').textContent = window.formatHours(astro.TST_HRS);
                $('mst-time').textContent = window.formatHours(astro.MST_HRS);
            }
            if ($('moon-phase-name') && typeof window.getMoonPhaseName === 'function') {
                $('moon-phase-name').textContent = window.getMoonPhaseName(astro.moon.illumination.phase);
            }
            // Autres champs Astro...

        } catch (e) {
             // Si l'astro plante, on s'assure que les champs restent lisibles
             if ($('sun-alt')) $('sun-alt').textContent = 'N/A';
        }
    }
}

// =================================================================
// PARTIE 8 : INITIALISATION GLOBALE
// =================================================================

window.addEventListener('load', () => {
    console.log("🚀 Démarrage Dashboard V10.0");

    // 1. Initialiser le DOM avec des valeurs par défaut 0.0 (FIX N/A)
    initDOMDefaults();

    // 2. Gestion du bouton Pause/Reprendre
    const toggleGpsBtn = $('toggle-gps-btn');
    if (toggleGpsBtn) {
        toggleGpsBtn.addEventListener('click', () => {
            isGpsPaused = !isGpsPaused;
            toggleGpsBtn.textContent = isGpsPaused ? "▶️ REPRENDRE" : "⏸️ PAUSE GPS";
            
            if (!isGpsPaused) {
                // TENTEZ D'ACTIVER L'IMU (Geste utilisateur)
                initIMUSensors();
            }
        });
        toggleGpsBtn.textContent = isGpsPaused ? "▶️ REPRENDRE" : "⏸️ PAUSE GPS";
    }
    
    // 3. Initialisation des services
    if (typeof window.ProfessionalUKF === 'function') ukf = new ProfessionalUKF();
    syncH(); // Démarrage synchro NTP (avec fallback)
    initGPS(); // Démarrage GPS

    // 4. Boucle Principale
    setInterval(updateDashboard, 1000 / 60); 
});
