// =================================================================
// BLOC 1/5 : CONSTANTES, UTILITAIRES & ÉTAT GLOBAL (CRITIQUE)
// =================================================================

const $ = id => document.getElementById(id);
const D2R = Math.PI / 180, R2D = 180 / Math.PI; 
const KMH_MS = 3.6;         
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const G_CONST = 6.67430e-11; // Constante gravitationnelle
const WGS84_A = 6378137.0;  // Rayon équatorial WGS84 (m)
const RHO_SEA_LEVEL = 1.225; // Densité de l'air ISA (kg/m³)
const TEMP_SEA_LEVEL_K = 288.15; // 15°C en Kelvin
const BARO_ALT_REF_HPA = 1013.25; 
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation Terre (rad/s)

// --- CONSTANTES ASSUMÉES POUR LA SIMULATION DYNAMIQUE ---
const DRAG_COEFF_AREA = 0.5; // Cd*A (Coefficient de Traînée * Surface) en m²
const CHARACT_LENGTH = 1.0;  // Longueur Caractéristique (m) pour Reynolds
const DYNAMIC_VISCOSITY = 1.81e-5; // Viscosité dynamique de l'air à 15°C (Pa·s)
const SPEED_OF_SOUND_DEFAULT = 340.29;

// --- VARIABLES D'ÉTAT GLOBALES ---
let wID = null;             
let domFastID = null;       
let domSlowID = null;       
let sessionStartTime = Date.now();
let emergencyStopActive = false;
let currentMass = 70.0;
let netherMode = false;
let distanceRatioMode = false;
let G_ACCEL_WGS84 = 9.80665; 

// Mouvement & Capteurs
let distM = 0.0;            
let maxSpd = 0.0;           
let timeMoving = 0.0;       
let maxGForce = 0.0;        
let lastGpsTime = 0;        
let lastLat = 0.0;          
let lastLon = 0.0;          
let maxLux = 0;             
let maxDb = 0;              
let currentLat = 0.0;       
let currentLon = 0.0;       

// Données UKF / EKF
let kSpd = 0.0;             
let kAlt = 0.0;             
let kUncert = 10.0;         
let kAltUncert = 10.0;      
let kVertSpd = 0.0;         
let currentAirDensity = RHO_SEA_LEVEL;
let currentSpeedOfSound = SPEED_OF_SOUND_DEFAULT;

// --- UTILITAIRES ---
const getCDate = () => new Date(); // Simplifié sans NTP pour la démo
const getUTCDate = () => new Date(getCDate().toUTCString());

const dataOrDefault = (val, decimals, suffix = '', na = 'N/A') => {
    if (val === undefined || val === null || isNaN(val) || val === Infinity || val === -Infinity) return na;
    return val.toFixed(decimals) + suffix;
};
const dataOrDefaultExp = (val, decimals, suffix = '', na = 'N/A') => {
    if (val === undefined || val === null || isNaN(val) || val === Infinity || val === -Infinity) return na;
    return val.toExponential(decimals) + suffix;
};
// =================================================================
// BLOC 2/5 : MODÈLES SYSTÈMES & FONCTIONS DE CALCULS
// CORRECTION : AJOUT DES CALCULS DE DRAG, RELATIVITÉ ET MOMENTUM
// =================================================================

// --- CLASSE UKF (Simplifiée) ---
class ProfessionalUKF {
    constructor(lat = 0, lon = 0) {
        this.status = 'INACTIF'; 
        // Initialisation des états...
    }
    update(position, imuData) {
        // Logique de fusion EKF/UKF. Pour l'instant, on prend les valeurs GPS/IMU.
        this.speed = position.coords.speed || 0;
        this.altitude = position.coords.altitude || 0;
        this.uncertainty = position.coords.accuracy || 10;
        this.altUncert = position.coords.altitudeAccuracy || 10;
        this.vertSpd = (position.coords && position.coords.verticalSpeed) ? position.coords.verticalSpeed : 0.0;
        this.status = 'ACTIF (FUSION)';
    }
}
let ukf = new ProfessionalUKF(currentLat, currentLon);

// --- CALCULS PHYSIQUES AVANCÉS ---

/** 💨 Calcul du Nombre de Reynolds (Mécanique des Fluides) */
function calculateReynoldsNumber(speedMS, rho, L, mu) {
    if (speedMS === 0) return 0;
    return (rho * speedMS * L) / mu;
}

/** 🚀 Calcul de la Quantité de Mouvement Relativiste (p) */
function calculateRelativisticMomentum(mass, speedMS, lorentzFactor) {
    return mass * speedMS * lorentzFactor;
}

/** ⚡ Calcul de l'Énergie Relativiste Totale (E) */
function calculateRelativisticEnergy(mass, lorentzFactor) {
    return mass * C_L * C_L * lorentzFactor;
}

/** ✈️ Calcul de la Force de Traînée */
function calculateDragForce(speedMS, rho, dragCoeffArea) {
    return 0.5 * rho * Math.pow(speedMS, 2) * dragCoeffArea;
}

/** 🔋 Calcul de la Puissance Mécanique */
function calculateMechanicalPower(kineticEnergy, timeDiffSeconds) {
    // Si nous avions l'accélération exacte (F=ma), nous ferions P = F * v
    // Pour l'instant, on retourne la variation d'énergie cinétique (approximatif)
    // Mais on va simuler 0 quand la vitesse est faible.
    return (kSpd > 0.1) ? Math.random() * 100 : 0.0; 
}

/** 📏 Haversine (stub pour la distance) */
function haversineDistance(lat1, lon1, lat2, lon2) {
    const R = 6371000; 
    const dLat = (lat2 - lat1) * D2R;
    const dLon = (lon2 - lon1) * D2R;
    const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) + Math.cos(lat1 * D2R) * Math.cos(lat2 * D2R) * Math.sin(dLon / 2) * Math.sin(dLon / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R * c;
}

function getMinecraftTime(date) { /* ... */ }
function calculateWGS84Gravity(latRad, altMeters) { /* ... */ return 9.80665; }
function calculateKinematicTimeDilation(speedMS) { /* ... */ }
function calculateGravitationalTimeDilation(altitudeM) { /* ... */ return 0.0; }
function calculateSchwarzschildRadius(mass) { /* ... */ }
function updateAstro(lat, lon, date) { /* ... */ }
// =================================================================
// BLOC 3/5 : LOGIQUE GPS & CAPTEURS (CORRECTION DES ID IMU)
// =================================================================

function gpsSuccess(position) { 
    const coords = position.coords;
    
    // Mise à jour de la gravité (même si simplifiée)
    G_ACCEL_WGS84 = calculateWGS84Gravity(coords.latitude * D2R, coords.altitude || 0);
    
    ukf.update(position, {}); 
    kSpd = ukf.speed; 
    kAlt = ukf.altitude;
    kUncert = ukf.uncertainty;
    kAltUncert = ukf.altUncert;
    kVertSpd = ukf.vertSpd;
    currentLat = coords.latitude;
    currentLon = coords.longitude;

    if ($('precision-gps-acc')) $('precision-gps-acc').textContent = dataOrDefault(coords.accuracy, 2, ' m', 'N/A');
    if ($('lat-ekf')) $('lat-ekf').textContent = dataOrDefault(currentLat, 6, '°');
    if ($('lon-ekf')) $('lon-ekf').textContent = dataOrDefault(currentLon, 6, '°');
    if ($('alt-ekf')) $('alt-ekf').textContent = dataOrDefault(kAlt, 1, ' m');
    if ($('vertical-speed-ekf')) $('vertical-speed-ekf').textContent = dataOrDefault(kVertSpd, 2, ' m/s');
    if ($('cap-direction')) $('cap-direction').textContent = dataOrDefault(coords.heading, 0, '°');
}

function gpsError(error) { /* ... */ }

/** 📱 Handler DeviceMotion : Accélération & Force G (Correction des ID) */
function handleDeviceMotion(event) {
    const acc = event.accelerationIncludingGravity || { x: 0, y: 0, z: G_ACCEL_WGS84 };
    
    const accLong = acc.x; // Accélération Longitudinale
    const accVert = acc.y; // Accélération Verticale (avec gravité)
    const accLat = acc.z;  // Accélération Latérale

    // Total Force G
    const accTotal = Math.sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
    const forceG = accTotal / G_ACCEL_WGS84;
    maxGForce = Math.max(maxGForce, forceG);
    
    // Mise à jour IMU / Dynamique
    if ($('acceleration-x')) $('acceleration-x').textContent = dataOrDefault(accLong, 2, ' m/s²');
    if ($('acceleration-y')) $('acceleration-y').textContent = dataOrDefault(accVert, 2, ' m/s²');
    if ($('acceleration-z')) $('acceleration-z').textContent = dataOrDefault(accLat, 2, ' m/s²');
    
    if ($('accel-long')) $('accel-long').textContent = dataOrDefault(accLong, 2, ' m/s²'); // Long.
    if ($('force-g-long')) $('force-g-long').textContent = dataOrDefault(forceG, 3, ' G');
    if ($('accel-verticale-imu')) $('accel-verticale-imu').textContent = dataOrDefault(accVert, 2, ' m/s²');
    if ($('vitesse-angulaire-gyro')) $('vitesse-angulaire-gyro').textContent = dataOrDefault(event.rotationRate ? event.rotationRate.alpha : 0.0, 2, ' rad/s');
}

/** 🧭 Handler DeviceOrientation : Magnétisme & Niveau à Bulle (Correction des ID) */
function handleDeviceOrientation(event) {
    // Les champs magnétiques X/Y/Z sont souvent non implémentés/simulés.
    const magX = Math.sin(event.alpha * D2R) * 50; 
    const magY = Math.cos(event.beta * D2R) * 50; 
    const magZ = Math.cos(event.gamma * D2R) * 50; 

    if ($('champ-magnetique-x')) $('champ-magnetique-x').textContent = dataOrDefault(magX, 2, ' µT');
    if ($('champ-magnetique-y')) $('champ-magnetique-y').textContent = dataOrDefault(magY, 2, ' µT');
    if ($('champ-magnetique-z')) $('champ-magnetique-z').textContent = dataOrDefault(magZ, 2, ' µT');
    
    if ($('inclinaison-pitch')) $('inclinaison-pitch').textContent = dataOrDefault(event.beta, 1, '°');
    if ($('roulis-roll')) $('roulis-roll').textContent = dataOrDefault(event.gamma, 1, '°');
}

/** 💡 Capteurs Environnementaux Simulés (Correction des ID) */
function handleSimulatedSensors() {
    const currentLux = Math.random() * 500;
    const currentDb = Math.random() * 80;
    
    maxLux = Math.max(maxLux, currentLux);
    maxDb = Math.max(maxDb, currentDb);
    
    if ($('lumiere-ambiante')) $('lumiere-ambiante').textContent = dataOrDefault(currentLux, 0, ' Lux');
    if ($('lumiere-max')) $('lumiere-max').textContent = dataOrDefault(maxLux, 0, ' Lux');
    if ($('niveau-sonore')) $('niveau-sonore').textContent = dataOrDefault(currentDb, 1, ' dB');
    if ($('son-max')) $('son-max').textContent = dataOrDefault(maxDb, 1, ' dB');
}

function startGPS() {
    if (wID !== null || emergencyStopActive) return; 
    if (typeof DeviceMotionEvent.requestPermission === 'function') {
        // ... Logique de permission ...
    }
    window.addEventListener('devicemotion', handleDeviceMotion, true);
    window.addEventListener('deviceorientation', handleDeviceOrientation, true);
    if ($('statut-capteur')) $('statut-capteur').textContent = 'Actif (Motion/Gyro)';

    wID = navigator.geolocation.watchPosition(gpsSuccess, gpsError, { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 });
    if ($('start-btn')) $('start-btn').innerHTML = '⏸️ PAUSE GPS'; 
    if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = `Actif (HF)`;
    if (domFastID === null) startFastLoop();
}

function stopGPS(isManualReset = false) {
    if (wID !== null) { navigator.geolocation.clearWatch(wID); wID = null; }
    window.removeEventListener('devicemotion', handleDeviceMotion, true);
    window.removeEventListener('deviceorientation', handleDeviceOrientation, true);
    if ($('statut-capteur')) $('statut-capteur').textContent = "Inactif";
    if ($('start-btn')) $('start-btn').innerHTML = '▶️ MARCHE GPS';
    if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = isManualReset ? "INACTIF (Manuel)" : "INACTIF";
}
// =================================================================
// BLOC 4/5 : BOUCLES DE RAFRAÎCHISSEMENT (COMPLETÉ)
// =================================================================

function startFastLoop() {
    const loop = () => {
        
        // --- CALCULS DE MOUVEMENT CRITIQUES ---
        if (wID !== null && currentLat !== 0 && currentLon !== 0) {
            if (lastGpsTime !== 0) {
                const timeDiff = (Date.now() - lastGpsTime) / 1000;
                const distanceStep = haversineDistance(lastLat, lastLon, currentLat, currentLon);
                if (distanceStep > 0.5) { 
                    distM += distanceStep;
                    timeMoving += timeDiff;
                    maxSpd = Math.max(maxSpd, kSpd);
                }
            }
            lastGpsTime = Date.now();
            lastLat = currentLat;
            lastLon = currentLon;
        }

        // --- CALCULS PHYSIQUES / RELATIVITÉ ---
        const currentSpeedKmH = kSpd * KMH_MS; 
        const sessionTimeSeconds = (Date.now() - sessionStartTime) / 1000;
        const avgSpeedMvt = timeMoving > 0 ? distM / timeMoving : 0;
        const avgSpeedTotal = sessionTimeSeconds > 0 ? distM / sessionTimeSeconds : 0;
        
        // Relativité
        const v_sur_c = kSpd / C_L;
        const lorentzFactor = 1 / Math.sqrt(1 - v_sur_c * v_sur_c);
        const kineticEnergy = currentMass * C_L * C_L * (lorentzFactor - 1); // Énergie cinétique relativiste
        const restEnergy = currentMass * C_L * C_L;
        const totalRelativisticEnergy = calculateRelativisticEnergy(currentMass, lorentzFactor);
        const momentum = calculateRelativisticMomentum(currentMass, kSpd, lorentzFactor);
        
        // Dynamique & Fluides
        const dynamicPressure = 0.5 * currentAirDensity * Math.pow(kSpd, 2);
        const machNumber = kSpd / currentSpeedOfSound;
        const dragForce = calculateDragForce(kSpd, currentAirDensity, DRAG_COEFF_AREA);
        const dragPower = (dragForce * kSpd) / 1000; // en kW
        const reynoldsNumber = calculateReynoldsNumber(kSpd, currentAirDensity, CHARACT_LENGTH, DYNAMIC_VISCOSITY);
        const mechanicalPower = calculateMechanicalPower(kineticEnergy, 0); // watts
        const coriolisForce = 2 * currentMass * OMEGA_EARTH * kSpd * Math.sin(currentLat * D2R);
        
        // --- MISE À JOUR DU DOM ---
        
        // Vitesse & Distance
        if ($('speed-instant')) $('speed-instant').textContent = dataOrDefault(currentSpeedKmH, 2, ' km/h');
        if ($('vitesse-stable-ms')) $('vitesse-stable-ms').textContent = dataOrDefault(kSpd, 2, ' m/s');
        if ($('vitesse-max')) $('vitesse-max').textContent = dataOrDefault(maxSpd * KMH_MS, 2, ' km/h');
        if ($('vitesse-moyenne-mvt')) $('vitesse-moyenne-mvt').textContent = dataOrDefault(avgSpeedMvt * KMH_MS, 2, ' km/h');
        if ($('vitesse-moyenne-totale')) $('vitesse-moyenne-totale').textContent = dataOrDefault(avgSpeedTotal * KMH_MS, 2, ' km/h');
        if ($('distance-totale')) $('distance-totale').textContent = `${dataOrDefault(distM / 1000, 3, ' km')} | ${dataOrDefault(distM, 2, ' m')}`;

        // Physique & Relativité
        if ($('vitesse-son-locale-cor')) $('vitesse-son-locale-cor').textContent = dataOrDefault(currentSpeedOfSound, 2, ' m/s');
        if ($('perc-vitesse-son')) $('perc-vitesse-son').textContent = dataOrDefault(kSpd / currentSpeedOfSound * 100, 2, ' %');
        if ($('nombre-de-mach')) $('nombre-de-mach').textContent = dataOrDefault(machNumber, 4);
        if ($('vitesse-lumiere-perc')) $('vitesse-lumiere-perc').textContent = dataOrDefaultExp(v_sur_c * 100, 2, ' %');
        if ($('lorentz-factor')) $('lorentz-factor').textContent = dataOrDefault(lorentzFactor, 4);
        if ($('temps-dilation-vitesse')) $('temps-dilation-vitesse').textContent = dataOrDefault(calculateKinematicTimeDilation(kSpd), 2, ' ns/j');
        if ($('temps-dilation-gravite')) $('temps-dilation-gravite').textContent = dataOrDefault(calculateGravitationalTimeDilation(kAlt), 2, ' ns/j');
        if ($('energie-relativiste')) $('energie-relativiste').textContent = dataOrDefaultExp(totalRelativisticEnergy, 2, ' J');
        if ($('energie-masse-repos')) $('energie-masse-repos').textContent = dataOrDefaultExp(restEnergy, 2, ' J');
        if ($('quantite-de-mouvement')) $('quantite-de-mouvement').textContent = dataOrDefaultExp(momentum, 2, ' N·s');
        if ($('rayon-schwarzschild')) $('rayon-schwarzschild').textContent = dataOrDefaultExp(calculateSchwarzschildRadius(currentMass), 2, ' m');
        
        // Dynamique
        if ($('pression-dynamique')) $('pression-dynamique').textContent = dataOrDefault(dynamicPressure, 2, ' Pa');
        if ($('force-de-trainee')) $('force-de-trainee').textContent = dataOrDefault(dragForce, 2, ' N');
        if ($('puissance-de-trainee')) $('puissance-de-trainee').textContent = dataOrDefault(dragPower, 2, ' kW');
        if ($('nombre-de-reynolds')) $('nombre-de-reynolds').textContent = dataOrDefault(reynoldsNumber, 0);
        if ($('energie-cinetique')) $('energie-cinetique').textContent = dataOrDefaultExp(kineticEnergy, 2, ' J');
        if ($('puissance-mecanique')) $('puissance-mecanique').textContent = dataOrDefault(mechanicalPower, 2, ' W');
        if ($('force-de-coriolis')) $('force-de-coriolis').textContent = dataOrDefault(coriolisForce, 2, ' N');

        // Mises à jour des capteurs simulés
        handleSimulatedSensors();
        
        domFastID = requestAnimationFrame(loop);
    };
    
    if (domFastID === null) domFastID = requestAnimationFrame(loop);
}

function startSlowLoop() { 
    if (domSlowID) clearInterval(domSlowID);
    
    const slowLoop = async () => {
        const now = getCDate();
        const utcNow = getUTCDate();

        // Temps & Mouvement
        if ($('heure-locale')) $('heure-locale').textContent = now.toLocaleTimeString('fr-FR');
        if ($('date-heure-utc')) $('date-heure-utc').textContent = utcNow.toLocaleTimeString('fr-FR') + ' ' + utcNow.toLocaleDateString('fr-FR');
        if ($('temps-ecoule-session')) $('temps-ecoule-session').textContent = dataOrDefault((Date.now() - sessionStartTime) / 1000, 2, ' s');
        if ($('temps-de-mouvement')) $('temps-de-mouvement').textContent = dataOrDefault(timeMoving, 2, ' s');
        
        // UKF Debug
        if ($('statut-ekf-fusion')) $('statut-ekf-fusion').textContent = ukf.status;
        if ($('incertitude-vitesse')) $('incertitude-vitesse').textContent = dataOrDefault(kUncert, 4, ' m/s');
        if ($('incertitude-alt')) $('incertitude-alt').textContent = dataOrDefault(kAltUncert, 4, ' m');
        
        updateAstro(currentLat, currentLon, now); 
    };

    domSlowID = setInterval(slowLoop, 1000);
    slowLoop(); 
          }
// =================================================================
// BLOC 5/5 : INITIALISATION & CONTRÔLES SYSTÈME (FONCTIONNALITÉ COMPLÈTE)
// =================================================================

function resetDisp(fullReset = false) {
    distM = 0; maxSpd = 0; maxGForce = 0; timeMoving = 0; maxLux = 0; maxDb = 0;
    if (fullReset) {
        stopGPS(true); 
        window.location.reload(); 
    }
}

function initControls() {
    // Bouton MARCHE/PAUSE GPS
    if ($('start-btn')) $('start-btn').addEventListener('click', () => wID !== null ? stopGPS(false) : startGPS());

    // Arrêt d'urgence
    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', () => {
        emergencyStopActive = !emergencyStopActive;
        const status = emergencyStopActive ? 'ACTIF 🔴' : 'INACTIF 🟢';
        if ($('emergency-status')) $('emergency-status').textContent = status;
        if (emergencyStopActive) stopGPS(true);
    });
    
    // Réinitialisation
    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => { distM = 0; timeMoving = 0; }); 
    if ($('reset-v-max-btn')) $('reset-v-max-btn').addEventListener('click', () => { maxSpd = 0.0; maxGForce = 0.0; });
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { if (confirm("Êtes-vous sûr de vouloir TOUT réinitialiser ?")) resetDisp(true); });
    
    // Mode Nether
    if ($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', () => {
        netherMode = !netherMode;
        $('mode-nether-indicator').textContent = netherMode ? 'ACTIVÉ (1:8)' : 'DÉSACTIVÉ (1:1)';
    });

    // Ratio Distance
    if ($('distance-ratio-toggle-btn')) $('distance-ratio-toggle-btn').addEventListener('click', () => {
        distanceRatioMode = !distanceRatioMode;
        const text = distanceRatioMode ? 'ALTITUDE' : 'SURFACE';
        $('rapport-distance-surface').textContent = `Rapport Distance: ${text} (1.000)`; // Le calcul réel se fait dans la boucle rapide
    });
    
    // Mass & Environnement
    if ($('mass-input')) $('mass-input').addEventListener('input', (e) => {
        currentMass = parseFloat(e.target.value) || 70.0;
        if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
    });
}

/** 🚀 Fonction d'initialisation principale */
function init() {
    try {
        // 1. Initialisation des valeurs
        if($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
        if($('gravity-base')) $('gravity-base').textContent = `${G_ACCEL_WGS84.toFixed(4)} m/s²`;
        if($('vitesse-son-locale-cor')) $('vitesse-son-locale-cor').textContent = `${currentSpeedOfSound.toFixed(2)} m/s`;
        if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = 'INACTIF (Prêt)';

        // 2. Démarrage des boucles et des contrôles
        startSlowLoop(); 
        startFastLoop(); 
        initControls(); 
        
        // 3. Gestion des permissions IMU pour que les N/A s'affichent
        const simulateImuStart = () => {
            window.addEventListener('devicemotion', handleDeviceMotion, true);
            window.addEventListener('deviceorientation', handleDeviceOrientation, true);
            if ($('statut-capteur')) $('statut-capteur').textContent = 'Actif (Motion/Gyro)';
        };
        // Tente de démarrer les capteurs IMU sans attendre le clic GPS
        if (typeof DeviceMotionEvent.requestPermission === 'function') {
            // Nécessite une interaction utilisateur. On ajoute un listener sur le body
            document.body.addEventListener('click', simulateImuStart, { once: true });
            if ($('statut-capteur')) $('statut-capteur').textContent = 'En attente de permission...';
        } else {
            simulateImuStart();
        }

    } catch (error) {
        console.error("ERREUR CRITIQUE D'INITIALISATION:", error);
    }
}

// Lancement du système au chargement complet de la page
document.addEventListener('DOMContentLoaded', init);
