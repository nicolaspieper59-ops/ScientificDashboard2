// --- CONSTANTES GLOBALES ET ASTRONOMIQUES ---
const C_LIGHT_MS = 299792458;  // Vitesse de la lumière en m/s
const C_SON_MS_SEA_LEVEL = 343; // Vitesse du son en m/s
const METER_TO_FEET = 3.28084;  
const EARTH_ROTATION_RATE = 15; // 15 degrés par heure (Rotation de la Terre)
const SYNODIC_MONTH = 29.53058867; // Durée moyenne d'un cycle lunaire

// *** SEUILS DE FILTRAGE CRITIQUES ***
// Si l'imprécision de l'altitude est supérieure à ce seuil (en mètres), on considère Vz = 0
const ALTITUDE_ACCURACY_THRESHOLD = 5.0; 
// Si la précision horizontale est supérieure à ce seuil (en mètres), on force Vx = 0
const HORIZONTAL_ACCURACY_THRESHOLD = 50.0; 

// --- Variables de contrôle et de navigation ---
let intervalId = null;
let timeElapsed = 0; 

let currentSpeedMS = 0; // Vitesse Horizontale (Ground Speed) en m/s
let maxSpeedKPH = 0;
let distanceTraveled = 0; // en km

let currentLat = null;
let currentLon = null;
let currentAlt = null;
let currentAccuracy = 9999; // Précision horizontale (mètres)
let currentAltAccuracy = 9999; // Précision verticale (mètres)

let previousAltitude = null; 
let verticalSpeedMS = 0; // Vitesse Verticale (Vz) en m/s
let previousTimestamp = null; // Pour le calcul temporel précis
let gpsWatchId = null;


// --- FONCTIONS UTILITAIRES DE TEMPS & ASTRONOMIE ---

function getAtomicTimeUTC() {
    const now = new Date();
    const totalHours = now.getUTCHours() + (now.getUTCMinutes() / 60) + (now.getUTCSeconds() / 3600) + (now.getUTCMilliseconds() / 3600000);
    return totalHours;
}

function updateAtomicTime() {
    // Utilise l'heure UTC du système, synchronisée par le réseau/GPS
    const now = new Date();
    const utcHours = String(now.getUTCHours()).padStart(2, '0');
    const utcMinutes = String(now.getUTCMinutes()).padStart(2, '0');
    const utcSeconds = String(now.getUTCSeconds()).padStart(2, '0');
    // Affiche les centièmes de seconde
    const utcMilliseconds = String(Math.floor(now.getUTCMilliseconds() / 10)).padStart(2, '0'); 
    
    document.getElementById('atomic-time').textContent = 
        `${utcHours}:${utcMinutes}:${utcSeconds}.${utcMilliseconds} UTC`;
}

function calculateLunarData() { 
    // Calcul de la phase lunaire (très approximatif et basé sur l'heure système)
    const newMoonEpoch = new Date('2000-01-06T18:14:00Z');
    const now = new Date();
    const totalDays = (now.getTime() - newMoonEpoch.getTime()) / (1000 * 60 * 60 * 24);
    let daysIntoCycle = totalDays % SYNODIC_MONTH;
    if (daysIntoCycle < 0) {
        daysIntoCycle += SYNODIC_MONTH;
    }
    const phasePercent = ((1 - Math.cos(2 * Math.PI * daysIntoCycle / SYNODIC_MONTH)) / 2) * 100;
    const magnitude = (phasePercent / 100) * (0.5) + 0.5;

    return { 
        phasePercent: phasePercent.toFixed(1), 
        magnitude: magnitude.toFixed(1) 
    };
}


function calculateLocalSolarTime(longitude) {
    const now = new Date();
    const utcTotalHours = getAtomicTimeUTC(); 
    const longitudeOffsetHours = longitude / EARTH_ROTATION_RATE; // Correction longitude vers temps

    const yearStart = new Date(now.getUTCFullYear(), 0, 1);
    const dayOfYear = (now.getTime() - yearStart.getTime()) / (1000 * 60 * 60 * 24);
    
    const B = (2 * Math.PI * (dayOfYear - 81) / 365.25); 

    // Composantes d'Excentricité et d'Obliquité pour l'Équation du Temps (très simplifié)
    const ECCENTRICITY_APPROX_FACTOR = -7.38217; 
    const OBLIQUITY_APPROX_FACTOR = 9.869;
    const EARTH_ORBITAL_ECCENTRICITY = 0.0167; 
    
    const eccSeconds = (ECCENTRICITY_APPROX_FACTOR * EARTH_ORBITAL_ECCENTRICITY * Math.sin(B)) * 60; 
    const oblSeconds = (OBLIQUITY_APPROX_FACTOR * Math.sin(2 * B)) * 60; 
    
    const edtSeconds = eccSeconds + oblSeconds; // Équation du temps

    // Heure Solaire Moyenne (HSM) = UTC + Longitude
    let hsmTotalHours = utcTotalHours + longitudeOffsetHours;
    hsmTotalHours = (hsmTotalHours % 24 + 24) % 24; 
    const hsmSecondsTotal = hsmTotalHours * 3600;
    const hsmTime = formatSecondsToTime(hsmSecondsTotal);

    // Heure Solaire Vraie (HSV) = HSM + Équation du temps
    let hsvTotalSeconds = hsmTotalHours * 3600 + edtSeconds;
    hsvTotalSeconds = (hsvTotalSeconds % 86400 + 86400) % 86400;
    const hsvTime = formatSecondsToTime(hsvTotalSeconds);

    // Calcul de la Culmination (Midi Solaire)
    const noonUTCSec = 12 * 3600; 
    const longitudeOffsetSeconds = longitudeOffsetHours * 3600;
    let culmTotalSeconds = noonUTCSec - longitudeOffsetSeconds - edtSeconds;
    const localOffset = now.getTimezoneOffset() * 60; // Fuseau horaire local
    let culmLocalSeconds = culmTotalSeconds - localOffset; 
    culmLocalSeconds = (culmLocalSeconds % 86400 + 86400) % 86400;
    const culmTime = formatSecondsToTime(culmLocalSeconds);

    const solarDayDurationSeconds = 86400 + edtSeconds * 0.005; // Durée du jour solaire (très simplifié)
    const solarDayDuration = formatSecondsToTime(solarDayDurationSeconds);

    return { 
        hsmTime, 
        hsvTime, 
        edtSeconds: edtSeconds.toFixed(8), 
        culmTime,
        solarDayDuration
    };
}

function formatSecondsToTime(totalSeconds) {
    const hours = Math.floor(totalSeconds / 3600);
    const minutes = Math.floor((totalSeconds % 3600) / 60);
    const seconds = Math.floor(totalSeconds % 60);
    return `${String(hours % 24).padStart(2, '0')}:${String(minutes).padStart(2, '0')}:${String(seconds).padStart(2, '0')}`;
}

// --- GESTION DES CAPTEURS ET GPS ---

function startBubbleLevel() { 
    if (!('DeviceOrientationEvent' in window)) {
        document.getElementById('niveau-bulle').textContent = '--° (N/A)';
        return;
    }
    
    window.addEventListener('deviceorientation', function(event) {
        const tiltX = event.gamma; 
        const tiltY = event.beta;

        if (tiltX !== null && tiltY !== null) {
            const resultX = tiltX.toFixed(1); 
            const resultY = tiltY.toFixed(1); 
            
            document.getElementById('niveau-bulle').textContent = 
                `Lat: ${resultX}° | Prof: ${resultY}°`;
        }
    }, true);
    
    // Demande de permission sur iOS 13+ (si nécessaire)
    if (typeof DeviceOrientationEvent.requestPermission === 'function') {
        DeviceOrientationEvent.requestPermission().catch(() => {}); 
    }
}


function getGeoLocation() {
    if (!("geolocation" in navigator)) {
        document.getElementById('gps-status').textContent = 'N/A (Non supporté)';
        return;
    }
    
    if (gpsWatchId) {
        navigator.geolocation.clearWatch(gpsWatchId);
    }
    
    gpsWatchId = navigator.geolocation.watchPosition(
        (position) => {
            const coords = position.coords;
            const currentTimestampMs = position.timestamp;
            
            currentAccuracy = coords.accuracy; 
            // Estime la précision verticale si non fournie (souvent 1.5x la précision horizontale)
            currentAltAccuracy = coords.altitudeAccuracy !== null ? coords.altitudeAccuracy : currentAccuracy * 1.5;

            // --- 1. FILTRE VITESSE HORIZONTALE (Ground Speed) ---
            if (coords.speed !== null && coords.speed !== undefined && currentAccuracy < HORIZONTAL_ACCURACY_THRESHOLD) {
                currentSpeedMS = Math.max(0, coords.speed); 
            } else {
                currentSpeedMS = 0; // Force à zéro si trop imprécis
            }
            
            // --- 2. FILTRE VITESSE VERTICALE (Vz) ---
            const newAltitude = coords.altitude;
            
            if (previousAltitude !== null && newAltitude !== null && previousTimestamp !== null) {
                const timeDiffSeconds = (currentTimestampMs - previousTimestamp) / 1000;
                
                if (timeDiffSeconds > 0) {
                    verticalSpeedMS = (newAltitude - previousAltitude) / timeDiffSeconds;
                } else {
                    verticalSpeedMS = 0;
                }
                
                // Si l'imprécision d'altitude est trop élevée, on force Vz à zéro
                if (currentAltAccuracy > ALTITUDE_ACCURACY_THRESHOLD) {
                    verticalSpeedMS = 0;
                    document.getElementById('vitesse-vert').classList.add('warning');
                } else {
                    document.getElementById('vitesse-vert').classList.remove('warning');
                }
                
            } else {
                verticalSpeedMS = 0;
            }
            
            previousAltitude = newAltitude;
            previousTimestamp = currentTimestampMs;
            // ----------------------------------------------------

            currentLat = coords.latitude;
            currentLon = coords.longitude;
            currentAlt = newAltitude; 

            document.getElementById('latitude').textContent = currentLat !== null ? currentLat.toFixed(6) : '--';
            document.getElementById('longitude').textContent = currentLon !== null ? currentLon.toFixed(6) : '--';
            
            const altMeters = currentAlt !== null ? currentAlt.toFixed(1) : '--';
            const altFeet = currentAlt !== null ? (currentAlt * METER_TO_FEET).toFixed(0) : '--';

            document.getElementById('altitude').textContent = `${altMeters} m`;
            document.getElementById('altitude-ft').textContent = `${altFeet} ft`; 
            
            document.getElementById('vitesse-vert').textContent = `${verticalSpeedMS.toFixed(2)} m/s`;
            
            document.getElementById('precision-m').textContent = `${currentAccuracy.toFixed(1)} m`;
            // Calcule la précision affichée (Précision GPS: 70% équivaut à 30m d'erreur environ)
            document.getElementById('prec-gps').textContent = `${(Math.max(0, 100 - (currentAccuracy / 10) * 10)).toFixed(0)} %`; 
            
            document.getElementById('gps-status').textContent = 'ACTIF';
            document.getElementById('gps-status').classList.remove('warning');
            
            updateCelestialData(); 
        },
        (error) => {
            // Gestion des erreurs GPS
            // ... (Réinitialisation des variables GPS) ...
            currentLat = null; currentLon = null; currentAlt = null; previousAltitude = null; 
            verticalSpeedMS = 0; currentSpeedMS = 0; 
            currentAccuracy = 9999; currentAltAccuracy = 9999; previousTimestamp = null;
            
            document.getElementById('vitesse-vert').textContent = '0.00 m/s';
            document.getElementById('vitesse-vert').classList.add('warning');
            document.getElementById('gps-status').textContent = `Erreur GPS: ${error.message} - Arrêté`;
            document.getElementById('gps-status').classList.add('warning');
            updateCelestialData(); 
        },
        { enableHighAccuracy: true, timeout: 10000, maximumAge: 0 } 
    );
}

/**
 * Mise à jour des données de navigation (Vitesse/Distance).
 */
function updateNavigationData() {
    if (!intervalId) return;
    
    // Calcul de la Vitesse Sol Totale (Vitesse 3D)
    const totalGroundSpeedSq = Math.pow(currentSpeedMS, 2) + Math.pow(verticalSpeedMS, 2);
    const TOTAL_GROUND_SPEED_MS = Math.sqrt(totalGroundSpeedSq);
    
    // Utilisation de la Vitesse 3D pour la distance totale parcourue
    // On suppose un intervalle de 1 seconde entre les mises à jour
    distanceTraveled += (TOTAL_GROUND_SPEED_MS / 1000); 

    // --- 3. PLANCHER À ZÉRO pour éviter les vitesses négatives microscopiques ---
    
    const V_KPH_INST = Math.max(0, currentSpeedMS * 3.6); 
    const V_KPH_3D = Math.max(0, TOTAL_GROUND_SPEED_MS * 3.6); 
    
    // Mise à jour de la vitesse max (basée sur la vitesse horizontale la plus fiable)
    if (V_KPH_INST > maxSpeedKPH) {
        maxSpeedKPH = V_KPH_INST;
    }
    
    const V_MMS = Math.max(0, TOTAL_GROUND_SPEED_MS * 1000);
    const V_LIGHT_RATIO = Math.max(0, TOTAL_GROUND_SPEED_MS / C_LIGHT_MS);
    const MACH_NUMBER = Math.max(0, TOTAL_GROUND_SPEED_MS / C_SON_MS_SEA_LEVEL);
    const AVG_SPEED_KPH = timeElapsed > 0 ? distanceTraveled / (timeElapsed / 3600) : 0;
    
    const avgSpeedKPH = Math.max(0, AVG_SPEED_KPH); 
    const DISTANCE_M = distanceTraveled * 1000;
    const DISTANCE_SL = DISTANCE_M / C_LIGHT_MS; 
    const DISTANCE_AL = DISTANCE_SL / (3600 * 24 * 365.25); 
    
    // Mise à jour de l'interface
    document.getElementById('time-s').textContent = `${(timeElapsed + 1).toFixed(0)} s`; // Temps total est incrémenté ici
    document.getElementById('vitesse-inst').textContent = `${V_KPH_INST.toFixed(2)} km/h`; 
    document.getElementById('vitesse-3d').textContent = `${V_KPH_3D.toFixed(2)} km/h`; 
    document.getElementById('vitesse-moy').textContent = `${avgSpeedKPH.toFixed(2)} km/h`;
    document.getElementById('vitesse-max').textContent = `${maxSpeedKPH.toFixed(2)} km/h`;
    
    document.getElementById('vitesse-ms').textContent = `${Math.max(0, TOTAL_GROUND_SPEED_MS).toFixed(2)} m/s`; 
    document.getElementById('vitesse-mms').textContent = `${V_MMS.toFixed(0)} mm/s`;

    document.getElementById('mach-number').textContent = `${MACH_NUMBER.toFixed(3)}`; 
    document.getElementById('pourcent-lumiere').textContent = `${(V_LIGHT_RATIO * 100).toFixed(8)}%`;
    
    document.getElementById('distance-km').textContent = `${distanceTraveled.toFixed(3)} km`;
    document.getElementById('distance-m').textContent = `${DISTANCE_M.toFixed(1)} m`;
    document.getElementById('distance-sl').textContent = `${DISTANCE_SL.toPrecision(4)} s lumière`;
    
    timeElapsed++; 
}

function updateCelestialData() {
    const now = new Date(); 
    
    let hsmTime = '--:--:--';
    let hsvTime = '--:--:--';
    let edtSeconds = '--';
    let culmTime = '--:--:--';
    let solarDayDuration = '--:--:--';

    const lunarData = calculateLunarData();
    
    if (currentLon !== null) {
        const solarTimes = calculateLocalSolarTime(currentLon);
        hsmTime = solarTimes.hsmTime;
        hsvTime = solarTimes.hsvTime;
        edtSeconds = solarTimes.edtSeconds;
        culmTime = solarTimes.culmTime;
        solarDayDuration = solarTimes.solarDayDuration;
    }

    document.getElementById('culm-soleil').textContent = culmTime;
    document.getElementById('hsm').textContent = hsmTime;
    document.getElementById('hsv').textContent = hsvTime;
    document.getElementById('edt').textContent = `${edtSeconds} s`; 
    document.getElementById('solar-day-duration').textContent = solarDayDuration;
    document.getElementById('phase-lune').textContent = `${lunarData.phasePercent}%`;
    document.getElementById('mag-lune').textContent = lunarData.magnitude;
    
    // Horloge Minecraft (simplifié)
    const totalSecondsInDay = (now.getHours() * 3600) + (now.getMinutes() * 60) + now.getSeconds();
    const minecraftTimeRatio = 1200 / 86400; 
    const minecraftSeconds = totalSecondsInDay * minecraftTimeRatio;
    const mcHours = Math.floor(minecraftSeconds / 50); 
    const mcMinutes = Math.floor((minecraftSeconds % 50) * 1.2); 
    const mcTimeStr = `${String(mcHours % 24).padStart(2, '0')}:${String(mcMinutes % 60).padStart(2, '0')}:00`;
    document.getElementById('horloge-minecraft').textContent = mcTimeStr;

    // Indicateur souterrain
    document.getElementById('souterrain').textContent = currentAlt === null ? 'Oui' : 'Non'; 
}

function toggleMovement(start) {
    if (start) {
        getGeoLocation(); 
        // Mises à jour principales de la navigation chaque seconde
        intervalId = setInterval(() => {
            updateCelestialData(); 
            updateNavigationData(); 
        }, 1000);
        
        document.getElementById('gps-status').textContent = 'ACTIF (Acquisition Vitesse)';
        document.getElementById('gps-status').classList.add('warning');
    } else {
        clearInterval(intervalId);
        intervalId = null;
        
        if (gpsWatchId) {
            navigator.geolocation.clearWatch(gpsWatchId);
            gpsWatchId = null;
        }

        document.getElementById('gps-status').textContent = 'Arrêté';
        document.getElementById('gps-status').classList.remove('warning');
        
        currentSpeedMS = 0; 
    }
    document.getElementById('start-btn').disabled = start;
    document.getElementById('stop-btn').disabled = !start;
}

function resetData() { 
    // Réinitialise toutes les variables de mouvement
    toggleMovement(false); 
    
    timeElapsed = 0;
    currentSpeedMS = 0;
    maxSpeedKPH = 0;
    distanceTraveled = 0;
    currentLat = null;
    currentLon = null;
    currentAlt = null;
    previousAltitude = null; 
    verticalSpeedMS = 0; 
    currentAccuracy = 9999;
    currentAltAccuracy = 9999;
    previousTimestamp = null;
    
    // Réinitialise les affichages
    document.getElementById('time-s').textContent = '0.00 s';
    document.getElementById('vitesse-inst').textContent = '0.00 km/h'; 
    document.getElementById('vitesse-3d').textContent = '0.00 km/h'; 
    document.getElementById('vitesse-moy').textContent = '0.00 km/h';
    document.getElementById('vitesse-max').textContent = '0.00 km/h';
    document.getElementById('vitesse-ms').textContent = '0.00 m/s';
    document.getElementById('vitesse-mms').textContent = '0 mm/s';
    document.getElementById('mach-number').textContent = '0.000';
    document.getElementById('pourcent-lumiere').textContent = '0.00000000%';
    document.getElementById('distance-km').textContent = '0.000 km';
    document.getElementById('distance-m').textContent = '-- m';
    document.getElementById('distance-sl').textContent = '-- s lumière';
    
    document.getElementById('vitesse-vert').textContent = '0.00 m/s'; 
    document.getElementById('vitesse-vert').classList.remove('warning');
    
    document.getElementById('gps-status').textContent = 'En attente...';
    document.getElementById('gps-status').classList.add('warning');
}

// --- INITIALISATION AU CHARGEMENT DE LA PAGE ---
function initializeCockpit() {
    getGeoLocation(); // Commence la surveillance GPS
    startBubbleLevel(); // Commence la surveillance de l'orientation
    updateCelestialData(); // Met à jour les données astronomiques initiales
    
    // Mises à jour en temps réel (Atomic Time)
    setInterval(updateAtomicTime, 10); 
}

window.onload = initializeCockpit;
