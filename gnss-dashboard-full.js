// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET V11.0 (ULTIMATE FIX)
// CORRECTIONS: IMU (Ancienne API + Geste Utilisateur/Permission),
// Synchronisation UTC Immédiate, Initialisation DOM à 0.0, Formatage Vitesse/Relativité.
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
const RHO_SEA_LEVEL = 1.225; // Densité de l'air à 15°C au niveau de la mer
const TEMP_SEA_LEVEL_K = 288.15; // 15°C en Kelvin
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

// Formatage robuste : Affiche 0.00 ou la valeur par défaut au lieu de N/A ou --
const dataOrDefault = (val, decimals, suffix = '', fallback = null, forceZero = true) => {
    // Si la valeur est explicitement 'N/A' (chaîne)
    if (val === 'N/A') return 'N/A'; 
    
    // Si la valeur est undefined, null, NaN, ou très proche de zéro
    if (val === undefined || val === null || isNaN(val) || (typeof val === 'number' && Math.abs(val) < 1e-18)) {
        if (fallback !== null) return fallback;
        if (forceZero) {
            const zeroFormat = (decimals === 0 ? '0' : '0.' + Array(decimals).fill('0').join('')) + suffix;
            return zeroFormat.replace('.', ',');
        }
        return 'N/A';
    }
    return val.toFixed(decimals).replace('.', ',') + suffix;
};

// Formatage exponentiel robuste (pour Rayon de Schwarzschild, etc.)
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        const zeroDecimals = '0.' + Array(decimals).fill('0').join('');
        return zeroDecimals.replace('.', ',') + 'e+0' + suffix;
    }
    // Utilise toExponential et remplace le point par une virgule pour le format FR
    return val.toExponential(decimals).replace('.', ',').replace('e', 'e') + suffix;
};

// Récupère l'heure corrigée UTC (Critique pour l'Astro)
function getCDate(serverH, localH) {
    if (serverH === 0 || localH === 0) return new Date(); // Fallback date
    return new Date(serverH + (Date.now() - localH));
}

// =================================================================
// PARTIE 2 : ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE
// =================================================================

let ukf = null;
let isGpsPaused = true; 
let isImuActive = false; // Nouvelle variable d'état
let gpsWatchID = null;

let currentMass = 70.0;
let currentPosition = { 
    lat: 43.284444, lon: 5.358599, alt: 100.00, // Coordonnées de l'utilisateur (Marseille)
    acc: 0.0, spd_ms: 0.0, heading: 0.0 
};

let maxSpeedMs = 0.0;
let totalDistanceM = 0.0;
let totalMovementTimeS = 0.0;
let totalSessionTimeS = 0.0;
let lastTimestamp = Date.now();

let lastIMU = { acc: {x:0, y:0, z:0}, pitch: 0.0, roll: 0.0 };
let lServH = 0, lLocH = 0; // NTP

// =================================================================
// PARTIE 3 : SYNCHRONISATION HORLOGE (UTC/NTP)
// =================================================================

function syncH() {
    const now = Date.now();
    lServH = now; 
    lLocH = now;

    // Mise à jour immédiate du DOM avec l'heure locale comme fallback UTC (FIX N/A UTC)
    const localFallbackDate = new Date(now).toISOString().replace('T', ' ').split('.')[0].replace('Z', '') + ' UTC (FALLBACK)';
    if ($('utc-datetime')) $('utc-datetime').textContent = localFallbackDate;

    // Tentative de synchronisation API
    fetch(SERVER_TIME_ENDPOINT)
        .then(r => r.json())
        .then(d => {
            lServH = d.unixtime * 1000; 
            lLocH = Date.now(); 
            const syncDate = getCDate(lServH, lLocH).toISOString().replace('T', ' ').split('.')[0].replace('Z', '') + ' UTC';
            if ($('utc-datetime')) $('utc-datetime').textContent = syncDate;
            console.log("NTP Synchronisation OK.");
        })
        .catch(e => {
            console.warn("🔴 Échec synchro NTP. Le fallback est maintenu.");
        });
}

// =================================================================
// PARTIE 4 : CAPTEURS IMU (IMU - Correction "Ancienne API")
// =================================================================

// NOTE: Cette fonction doit être appelée UNIQUEMENT par un geste utilisateur (ex: clic)
function initIMUSensors() {
    if (isImuActive) return;

    const startLegacyIMU = () => {
        // ANCIENNE API (DeviceMotionEvent pour Accéléromètre)
        if (window.DeviceMotionEvent) {
            window.addEventListener('devicemotion', (event) => {
                // Utilise event.accelerationIncludingGravity pour les axes (plus fréquent sur les anciens devices)
                if (event.accelerationIncludingGravity) {
                    lastIMU.acc.x = event.accelerationIncludingGravity.x || 0;
                    lastIMU.acc.y = event.accelerationIncludingGravity.y || 0;
                    lastIMU.acc.z = event.accelerationIncludingGravity.z || 0;
                }
                isImuActive = true;
                $('imu-status').textContent = 'Actif (Mouvement)';
            }, { once: false });
        }
        
        // ANCIENNE API (DeviceOrientationEvent pour Gyroscope/Pitch/Roll)
        if (window.DeviceOrientationEvent) {
            window.addEventListener('deviceorientation', (event) => {
                // event.beta (pitch), event.gamma (roll)
                lastIMU.pitch = event.beta || 0;
                lastIMU.roll = event.gamma || 0;
                isImuActive = true;
                $('imu-status').textContent = 'Actif (Orientation)';
            }, { once: false });
        }
        
        // Si l'initialisation s'est faite sans erreur visible
        if (!isImuActive) $('imu-status').textContent = 'Inactif (En attente de données)';
    };

    // Tenter l'API de permission (iOS 13+ et certains Chrome)
    if (typeof DeviceMotionEvent !== 'undefined' && typeof DeviceMotionEvent.requestPermission === 'function') {
        DeviceMotionEvent.requestPermission()
            .then(permissionState => {
                if (permissionState === 'granted') {
                    startLegacyIMU();
                } else {
                    $('imu-status').textContent = 'Refusé (Permission requise)';
                }
            })
            .catch(error => {
                // Si la fonction existe mais plante (ex: environnement non sécurisé), on tente quand même
                startLegacyIMU();
            });
    } else {
        // ANCIENNE API standard (Android, anciens navigateurs, non-HTTPS)
        startLegacyIMU();
    }
}

// =================================================================
// PARTIE 5 : GESTION GPS & STATISTIQUES (FIX VITESSE INSTANTANÉE)
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
    if (newSpd * KMH_MS > maxSpeedMs) maxSpeedMs = newSpd * KMH_MS; // Max speed in km/h
    if (newSpd > 0.1) totalMovementTimeS += dt; 

    // Calcul de distance brute si turf.js n'est pas utilisé (pour une valeur initiale)
    totalDistanceM += newSpd * dt;

    // Mise à jour de l'affichage de la vitesse brute/instantanée
    $('speed-3d-inst').textContent = dataOrDefault(newSpd * KMH_MS, 1, ' km/h');
    $('speed-raw-ms').textContent = dataOrDefault(newSpd, 2, ' m/s');
    
    // Sauvegarde État
    currentPosition = { lat: newLat, lon: newLon, alt: newAlt, acc: newAcc, spd_ms: newSpd, heading: p.heading || 0 };

    if (ukf && ukf.update) {
        // ukf.update(...) - La logique UKF doit être ici pour le filtrage
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
// PARTIE 6 : INITIALISATION DOM PAR DÉFAUT (FIX N/A AU DÉMARRAGE)
// =================================================================

function initDOMDefaults() {
    // Vitesse (Utilisation du dataOrDefault avec forceZero=true pour 0.00)
    const speedFields = ['speed-stable', 'speed-stable-ms', 'speed-stable-kms', 'speed-3d-inst', 'speed-raw-ms', 'vitesse-max-session', 'speed-avg-moving', 'speed-avg-total'];
    speedFields.forEach(id => {
        if ($(id)) $(id).textContent = dataOrDefault(0, 1, (id.includes('km/h') ? ' km/h' : (id.includes('m/s') ? ' m/s' : '')));
    });

    // Relativité/Physique (FIX N/A)
    $('lorentz-factor').textContent = dataOrDefault(1.0, 4); // Facteur de Lorentz initial à 1.0000
    $('speed-of-sound-calc').textContent = dataOrDefault(340.2900, 4, ' m/s');
    $('mach-number').textContent = dataOrDefault(0.0000, 4);
    $('percent-speed-light').textContent = dataOrDefaultExp(0.0, 2, ' %');
    $('energy-relativistic').textContent = 'N/A'; // Laisse N/A car dépend de l'UKF/Vitesse (ou 0.00 J)

    // Gravité et constantes (Gravité par défaut)
    $('gravity-base').textContent = dataOrDefault(G_STD, 4, ' m/s²');
    $('universal-gravity').textContent = dataOrDefaultExp(6.67430e-11, 6, ' m³/kg/s²');
    $('schwarzschild-radius').textContent = dataOrDefaultExp(1.0397e-25, 4, ' m'); // Calculé pour 70kg
    
    // IMU / Capteurs (Doit être N/A tant que non actif)
    if ($('imu-status')) $('imu-status').textContent = 'Inactif';
    const imuFields = ['acceleration-x', 'acceleration-y', 'acceleration-z', 'mag-x', 'mag-y', 'mag-z', 'light-ambient', 'sound-level'];
    imuFields.forEach(id => {
        if ($(id)) $(id).textContent = 'N/A';
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
    $('vitesse-max-session').textContent = dataOrDefault(maxSpeedMs, 1, ' km/h');
    
    // Temps
    $('elapsed-time').textContent = dataOrDefault(totalSessionTimeS, 2, ' s');
    $('movement-time').textContent = dataOrDefault(totalMovementTimeS, 2, ' s');
    
    // --- 2. MISE À JOUR PHYSIQUE & RELATIVITÉ ---
    
    // Facteur de Lorentz (gamma)
    const beta = v_ms / C_L;
    const lorentzFactor = 1.0 / Math.sqrt(1.0 - beta * beta);
    $('lorentz-factor').textContent = dataOrDefault(lorentzFactor, 4); 
    
    // % Vitesse Lumière
    const percentSpeedLight = beta * 100;
    $('percent-speed-light').textContent = dataOrDefaultExp(percentSpeedLight, 2, ' %');
    
    // Énergie de Masse au Repos (E₀ = m₀c²)
    const E0 = currentMass * C_L * C_L;
    $('energy-mass-rest').textContent = dataOrDefaultExp(E0, 2, ' J');

    // Énergie Relativiste (E = E₀ * γ)
    const E = E0 * lorentzFactor;
    $('energy-relativistic').textContent = dataOrDefaultExp(E, 2, ' J');

    // Quantité de Mouvement (p = m₀γv)
    const momentum = currentMass * lorentzFactor * v_ms;
    $('momentum').textContent = dataOrDefaultExp(momentum, 2, ' kg·m/s');

    // --- 3. MISE À JOUR IMU/ACCÉLÉRATION (Si actif) ---
    if (isImuActive) {
        $('acceleration-x').textContent = dataOrDefault(lastIMU.acc.x, 2, ' m/s²');
        $('acceleration-y').textContent = dataOrDefault(lastIMU.acc.y, 2, ' m/s²');
        $('acceleration-z').textContent = dataOrDefault(lastIMU.acc.z, 2, ' m/s²');
        // Roulis/Inclinaison (Pitch/Roll)
        $('inclinaison-pitch').textContent = dataOrDefault(lastIMU.pitch, 1, '°');
        $('roulis-roll').textContent = dataOrDefault(lastIMU.roll, 1, '°');
        // Force G (Verticale: z, Longitudinale: x/y. Ici on utilise z)
        const gVert = (lastIMU.acc.z / G_STD) - 1.0; // Retire g de la mesure z
        $('force-g-vert').textContent = dataOrDefault(gVert, 2, ' G');
        $('accel-vert-imu').textContent = dataOrDefault(lastIMU.acc.z, 2, ' m/s²');
    } else {
        // Maintient N/A si inactif pour la section IMU
        $('acceleration-x').textContent = 'N/A';
        $('acceleration-y').textContent = 'N/A';
        $('acceleration-z').textContent = 'N/A';
        $('inclinaison-pitch').textContent = dataOrDefault(0, 1, '°'); // 0.0° par défaut
        $('roulis-roll').textContent = dataOrDefault(0, 1, '°'); // 0.0° par défaut
    }

    // --- 4. MISE À JOUR ASTRO & POSITION ---
    $('lat-ekf').textContent = dataOrDefault(currentPosition.lat, 6);
    $('lon-ekf').textContent = dataOrDefault(currentPosition.lon, 6);
    $('alt-ekf').textContent = dataOrDefault(currentPosition.alt, 2, ' m');
    $('acc-gps').textContent = dataOrDefault(currentPosition.acc, 1, ' m');

    const now = getCDate(lServH, lLocH);
    if (now && typeof window.getSolarData === 'function') {
        try {
            const astro = window.getSolarData(now, currentPosition.lat, currentPosition.lon, currentPosition.alt);
            
            // Heures solaires
            if (window.formatHours) {
                $('tst-time').textContent = window.formatHours(astro.TST_HRS);
                $('mst-time').textContent = window.formatHours(astro.MST_HRS);
                $('local-solar-noon-utc').textContent = astro.NOON_SOLAR_UTC; // Format string est préférable
            }

            // Soleil
            $('sun-alt').textContent = dataOrDefault(astro.sun.position.altitude * R2D, 2, '°');
            $('sun-azimuth').textContent = dataOrDefault(astro.sun.position.azimuth * R2D, 2, '°');
            
            // Lune
            if (typeof window.getMoonPhaseName === 'function') {
                $('moon-phase-name').textContent = window.getMoonPhaseName(astro.moon.illumination.phase);
                $('moon-illuminated').textContent = dataOrDefault(astro.moon.illumination.fraction * 100, 1, ' %');
                $('moon-alt').textContent = dataOrDefault(astro.moon.position.altitude * R2D, 2, '°');
                $('moon-azimuth').textContent = dataOrDefault(astro.moon.position.azimuth * R2D, 2, '°');
                // Distance en km
                $('moon-distance').textContent = dataOrDefault(astro.moon.position.distance / 1000, 0, ' km');
            }
        } catch (e) {
             console.error("Erreur de rendu Astro:", e);
             // S'assure que les champs astro sont réinitialisés si la dépendance n'est pas complète
             if ($('sun-alt')) $('sun-alt').textContent = 'N/A'; 
        }
    }
}

// =================================================================
// PARTIE 8 : INITIALISATION GLOBALE
// =================================================================

window.addEventListener('load', () => {
    console.log("🚀 Démarrage Dashboard V11.0 - Correction IMU Ancienne API");

    // 1. Initialiser le DOM avec des valeurs par défaut 0.0/N/A (FIX N/A)
    initDOMDefaults();

    // 2. Gestion du bouton Pause/Reprendre
    const toggleGpsBtn = $('toggle-gps-btn');
    if (toggleGpsBtn) {
        toggleGpsBtn.addEventListener('click', () => {
            isGpsPaused = !isGpsPaused;
            toggleGpsBtn.textContent = isGpsPaused ? "▶️ REPRENDRE" : "⏸️ PAUSE GPS";
            
            if (!isGpsPaused) {
                // TENTE D'ACTIVER L'IMU (Geste utilisateur CRITIQUE)
                initIMUSensors();
            }
        });
        // Initialise l'état du bouton
        toggleGpsBtn.textContent = isGpsPaused ? "▶️ REPRENDRE" : "⏸️ PAUSE GPS";
    }
    
    // 3. Initialisation des services
    if (typeof window.ProfessionalUKF === 'function') {
         ukf = new ProfessionalUKF();
         $('statut-ekf-fusion').textContent = 'Initialisé';
    } else {
         $('statut-ekf-fusion').textContent = 'DÉSACTIVÉ (UKF manquant)';
    }
    
    syncH(); // Démarrage synchro NTP (avec fallback immédiat)
    initGPS(); // Démarrage GPS

    // 4. Boucle Principale
    // updateDashboard() doit être appelé immédiatement une première fois
    updateDashboard(); 
    setInterval(updateDashboard, 1000 / 60); // 60Hz
});
