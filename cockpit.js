// --- CONSTANTES GLOBALES ET ASTRONOMIQUES (FINALISÉES) ---
const C_LIGHT_MS = 299792458;  
const C_SON_MS_SEA_LEVEL = 343; 
const METER_TO_FEET = 3.28084;  
const EARTH_ROTATION_RATE = 15; 
const SYNODIC_MONTH = 29.53058867; 

// *** CONSTANTES ÉTALONNÉES ***
const EARTH_ORBITAL_ECCENTRICITY = 0.0167;      
const EARTH_AXIAL_OBLIQUITY_DEG = 23.45;        
const ECCENTRICITY_APPROX_FACTOR = -7.38217; 
const OBLIQUITY_APPROX_FACTOR = 9.869;     

let intervalId = null;
let timeElapsed = 0; 

// Variables de Vitesse/Distance/GPS
let currentSpeedMS = 0; // Vitesse Horizontale (Ground Speed)
let maxSpeedKPH = 0;
let distanceTraveled = 0; 

let currentLat = null;
let currentLon = null;
let currentAlt = null;

let previousAltitude = null; 
let verticalSpeedMS = 0; // Vitesse Verticale (Vz)

let gpsWatchId = null;

// --- FONCTIONS UTILITAIRES DE TEMPS ---

function getAtomicTimeUTC() {
    const now = new Date();
    const totalHours = now.getUTCHours() + (now.getUTCMinutes() / 60) + (now.getUTCSeconds() / 3600) + (now.getUTCMilliseconds() / 3600000);
    return totalHours;
}

function updateAtomicTime() {
    const now = new Date();
    const utcHours = String(now.getUTCHours()).padStart(2, '0');
    const utcMinutes = String(now.getUTCMinutes()).padStart(2, '0');
    const utcSeconds = String(now.getUTCSeconds()).padStart(2, '0');
    const utcMilliseconds = String(Math.floor(now.getUTCMilliseconds() / 10)).padStart(2, '0'); 
    
    document.getElementById('atomic-time').textContent = 
        `${utcHours}:${utcMinutes}:${utcSeconds}.${utcMilliseconds} UTC`;
}


function calculateLunarData() { 
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


// --- FONCTIONS ASTRONOMIQUES CLÉS (Étalonnées) ---

function calculateLocalSolarTime(longitude) {
    const now = new Date();
    const utcTotalHours = getAtomicTimeUTC(); 
    const longitudeOffsetHours = longitude / EARTH_ROTATION_RATE;

    const yearStart = new Date(now.getUTCFullYear(), 0, 1);
    const dayOfYear = (now.getTime() - yearStart.getTime()) / (1000 * 60 * 60 * 24);
    
    const B = (2 * Math.PI * (dayOfYear - 81) / 365.25); 

    const eccSeconds = (ECCENTRICITY_APPROX_FACTOR * EARTH_ORBITAL_ECCENTRICITY * Math.sin(B)) * 60; 
    const oblSeconds = (OBLIQUITY_APPROX_FACTOR * Math.sin(2 * B)) * 60; 
    
    const edtSeconds = eccSeconds + oblSeconds;

    const solLon = ((dayOfYear / 365.25) * 360) % 360; 

    let hsmTotalHours = utcTotalHours + longitudeOffsetHours;
    hsmTotalHours = (hsmTotalHours % 24 + 24) % 24; 

    const hsmSecondsTotal = hsmTotalHours * 3600;
    const hsmHours = Math.floor(hsmSecondsTotal / 3600);
    const hsmMinutes = Math.floor((hsmSecondsTotal % 3600) / 60);
    const hsmSeconds = Math.floor(hsmSecondsTotal % 60);
    const hsmTime = `${String(hsmHours).padStart(2, '0')}:${String(hsmMinutes).padStart(2, '0')}:${String(hsmSeconds).padStart(2, '0')}`;

    let hsvTotalSeconds = hsmTotalHours * 3600 + edtSeconds;
    hsvTotalSeconds = (hsvTotalSeconds % 86400 + 86400) % 86400;

    const hsvHours = Math.floor(hsvTotalSeconds / 3600);
    const hsvMinutes = Math.floor((hsvTotalSeconds % 3600) / 60);
    const hsvSecondsFinal = Math.floor(hsvTotalSeconds % 60);
    const hsvTime = `${String(hsvHours).padStart(2, '0')}:${String(hsvMinutes).padStart(2, '0')}:${String(hsvSecondsFinal).padStart(2, '0')}`;

    const noonUTCSec = 12 * 3600; 
    const longitudeOffsetSeconds = longitudeOffsetHours * 3600;

    let culmTotalSeconds = noonUTCSec - longitudeOffsetSeconds - edtSeconds;
    
    const localOffset = now.getTimezoneOffset() * 60; 
    let culmLocalSeconds = culmTotalSeconds - localOffset; 
    culmLocalSeconds = (culmLocalSeconds % 86400 + 86400) % 86400;

    const culmLocalHours = Math.floor(culmLocalSeconds / 3600);
    const culmLocalMinutes = Math.floor((culmLocalSeconds % 3600) / 60);
    const culmLocalSecondsFinal = Math.floor(culmLocalSeconds % 60);
    const culmTime = `${String(culmLocalHours).padStart(2, '0')}:${String(culmLocalMinutes).padStart(2, '0')}:${String(culmLocalSecondsFinal).padStart(2, '0')}`;

    const solarDayDurationSeconds = 86400 + edtSeconds * 0.005; 
    
    const dayHours = Math.floor(solarDayDurationSeconds / 3600);
    const dayMinutes = Math.floor((solarDayDurationSeconds % 3600) / 60);
    const daySeconds = Math.floor(solarDayDurationSeconds % 60);
    const solarDayDuration = `${String(dayHours).padStart(2, '0')}:${String(dayMinutes).padStart(2, '0')}:${String(daySeconds).padStart(2, '0')}`;

    return { 
        hsmTime, 
        hsvTime, 
        edtSeconds: edtSeconds.toFixed(8), 
        culmTime,
        eccComp: eccSeconds.toFixed(8), 
        oblComp: oblSeconds.toFixed(8), 
        solLon: solLon.toFixed(8),
        solarDayDuration
    };
}


// --- GESTION DES CAPTEURS ET MISE À JOUR ---

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
    
    if (typeof DeviceOrientationEvent.requestPermission === 'function') {
        DeviceOrientationEvent.requestPermission()
            .then(permissionState => {
                if (permissionState !== 'granted') {
                    document.getElementById('niveau-bulle').textContent = '--° (Accès refusé)';
                }
            })
            .catch(error => {
                document.getElementById('niveau-bulle').textContent = '--° (Erreur capteur)';
                console.error("Erreur d'accès à DeviceOrientation:", error);
            });
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
            
            // --- Calcul de la Vitesse Verticale (Vz) ---
            const newAltitude = coords.altitude;
            
            // Calcul du Delta Alt / Delta Temps sur la base de l'intervalle de 1s de l'updateNavigationData
            if (previousAltitude !== null && newAltitude !== null) {
                // Nous utilisons 1.0 comme diviseur car updateNavigationData est appelé toutes les 1000ms (1s)
                verticalSpeedMS = (newAltitude - previousAltitude); 
            } else {
                verticalSpeedMS = 0;
            }
            previousAltitude = newAltitude;
            // ------------------------------------------

            currentLat = coords.latitude;
            currentLon = coords.longitude;
            currentAlt = newAltitude; 

            if (coords.speed !== null && coords.speed !== undefined) {
                currentSpeedMS = Math.max(0, coords.speed); 
            } else {
                currentSpeedMS = 0;
            }

            document.getElementById('latitude').textContent = currentLat.toFixed(6);
            document.getElementById('longitude').textContent = currentLon.toFixed(6);
            
            const altMeters = currentAlt !== null ? currentAlt.toFixed(1) : '--';
            const altFeet = currentAlt !== null ? (currentAlt * METER_TO_FEET).toFixed(0) : '--';

            document.getElementById('altitude').textContent = `${altMeters} m`;
            document.getElementById('altitude-ft').textContent = `${altFeet} ft`; 
            
            document.getElementById('vitesse-vert').textContent = `${verticalSpeedMS.toFixed(2)} m/s`;
            
            document.getElementById('precision-m').textContent = `${coords.accuracy.toFixed(1)} m`;
            document.getElementById('prec-gps').textContent = `${((100 - (coords.accuracy / 20) * 100).toFixed(0))} %`; 
            
            document.getElementById('gps-status').textContent = 'ACTIF';
            document.getElementById('gps-status').classList.remove('warning');
            
            updateCelestialData(); 
        },
        (error) => {
            currentLat = null;
            currentLon = null;
            currentAlt = null; 
            previousAltitude = null; 
            verticalSpeedMS = 0;

            document.getElementById('vitesse-vert').textContent = '-- m/s';
            
            document.getElementById('gps-status').textContent = `Erreur GPS: ${error.message} - Arrêté`;
            document.getElementById('gps-status').classList.add('warning');
            currentSpeedMS = 0; 
            if (error.code === 1) { 
                document.getElementById('gps-status').textContent = 'Erreur GPS: Autorisation Refusée !';
            }
            updateCelestialData(); 
        },
        { enableHighAccuracy: true, timeout: 10000, maximumAge: 0 } 
    );
}

/** * Mise à jour des données de navigation (Vitesse/Distance).
 */
function updateNavigationData() {
    if (!intervalId) return;
    
    // Calcul de la Vitesse Sol Totale (Vitesse 3D)
    const totalGroundSpeedSq = Math.pow(currentSpeedMS, 2) + Math.pow(verticalSpeedMS, 2);
    const TOTAL_GROUND_SPEED_MS = Math.sqrt(totalGroundSpeedSq);

    distanceTraveled += (TOTAL_GROUND_SPEED_MS / 1000); // Utiliser la vitesse 3D pour la distance totale

    const V_KPH_INST = currentSpeedMS * 3.6; // Vitesse Horizontale (instantanée)
    const V_KPH_3D = TOTAL_GROUND_SPEED_MS * 3.6; // Vitesse 3D (Totale)
    
    const V_MMS = TOTAL_GROUND_SPEED_MS * 1000;
    const V_LIGHT_RATIO = TOTAL_GROUND_SPEED_MS / C_LIGHT_MS;
    const MACH_NUMBER = TOTAL_GROUND_SPEED_MS / C_SON_MS_SEA_LEVEL;
    
    if (V_KPH_INST > maxSpeedKPH) {
        maxSpeedKPH = V_KPH_INST;
    }
    
    const avgSpeedKPH = timeElapsed > 0 ? distanceTraveled / (timeElapsed / 3600) : 0;
    const DISTANCE_M = distanceTraveled * 1000;
    const DISTANCE_SL = DISTANCE_M / C_LIGHT_MS; 
    const DISTANCE_AL = DISTANCE_SL / (3600 * 24 * 365.25); 
    
    document.getElementById('time-s').textContent = `${timeElapsed.toFixed(2)} s`;
    document.getElementById('vitesse-inst').textContent = `${V_KPH_INST.toFixed(2)} km/h`; // Vitesse Horizontale
    document.getElementById('vitesse-3d').textContent = `${V_KPH_3D.toFixed(2)} km/h`; // NOUVELLE VITESSE 3D
    document.getElementById('vitesse-moy').textContent = `${avgSpeedKPH.toFixed(2)} km/h`;
    document.getElementById('vitesse-max').textContent = `${maxSpeedKPH.toFixed(2)} km/h`;
    
    document.getElementById('vitesse-ms').textContent = `${TOTAL_GROUND_SPEED_MS.toFixed(2)} m/s`; // Affichage de Vitesse 3D ici
    document.getElementById('vitesse-mms').textContent = `${V_MMS.toFixed(0)} mm/s`;

    document.getElementById('mach-number').textContent = `${MACH_NUMBER.toFixed(3)}`; 
    document.getElementById('pourcent-lumiere').textContent = `${(V_LIGHT_RATIO * 100).toFixed(8)}%`;
    
    document.getElementById('distance-km').textContent = `${distanceTraveled.toFixed(3)} km`;
    document.getElementById('distance-m').textContent = `${DISTANCE_M.toFixed(1)} m`;
    document.getElementById('distance-mm').textContent = `${(DISTANCE_M * 1000).toFixed(0)} mm`;
    
    document.getElementById('distance-sl').textContent = `${DISTANCE_SL.toPrecision(4)} s lumière`;
    document.getElementById('distance-al').textContent = `${DISTANCE_AL.toPrecision(4)} al`;
    
    timeElapsed++; 
}

function updateCelestialData() {
    // ... (Logique inchangée) ...
    const now = new Date(); 
    
    let hsmTime = '--:--:--';
    let hsvTime = '--:--:--';
    let edtSeconds = '--';
    let culmTime = '--:--:--';
    let eccComp = '--';
    let oblComp = '--';
    let solLon = '--';
    let solarDayDuration = '--:--:--';

    const lunarData = calculateLunarData();
    
    if (currentLon !== null) {
        const solarTimes = calculateLocalSolarTime(currentLon);
        hsmTime = solarTimes.hsmTime;
        hsvTime = solarTimes.hsvTime;
        edtSeconds = solarTimes.edtSeconds;
        culmTime = solarTimes.culmTime;
        eccComp = solarTimes.eccComp;
        oblComp = solarTimes.oblComp;
        solLon = solarTimes.solLon;
        solarDayDuration = solarTimes.solarDayDuration;
    }

    // ... (Mise à jour DOM pour Céleste/EDT/Minecraft/Météo) ...
    document.getElementById('culm-soleil').textContent = culmTime;
    document.getElementById('hsm').textContent = hsmTime;
    document.getElementById('hsv').textContent = hsvTime;
    document.getElementById('edt').textContent = `${edtSeconds} s`; 

    document.getElementById('eccentricity-comp').textContent = `${eccComp} s`;
    document.getElementById('obliquity-comp').textContent = `${oblComp} s`;
    document.getElementById('solar-longitude').textContent = `${solLon}°`; 
    document.getElementById('solar-day-duration').textContent = solarDayDuration;
    
    document.getElementById('phase-lune').textContent = `${lunarData.phasePercent}%`;
    document.getElementById('mag-lune').textContent = lunarData.magnitude;
    
    document.getElementById('lever-lune').textContent = '--:--:--';
    document.getElementById('coucher-lune').textContent = '--:--:--';
    document.getElementById('culmination-lune').textContent = '--:--:--';
    
    const totalSecondsInDay = (now.getHours() * 3600) + (now.getMinutes() * 60) + now.getSeconds();
    const minecraftTimeRatio = 1200 / 86400; 
    const minecraftSeconds = totalSecondsInDay * minecraftTimeRatio;

    const mcHours = Math.floor(minecraftSeconds / 50); 
    const mcMinutes = Math.floor((minecraftSeconds % 50) * 1.2); 
    
    const mcTimeStr = `${String(mcHours % 24).padStart(2, '0')}:${String(mcMinutes % 60).padStart(2, '0')}:${String(Math.floor(minecraftSeconds * 20) % 60).padStart(2, '0')}`;
    document.getElementById('horloge-minecraft').textContent = mcTimeStr;

    document.getElementById('temp').textContent = '-- °C';
    document.getElementById('pression').textContent = '-- hPa';
    document.getElementById('humidite').textContent = '--%';
    document.getElementById('vent').textContent = '-- km/h';
    document.getElementById('nuages').textContent = '--%';
    document.getElementById('uv').textContent = '--';
    document.getElementById('air-qual').textContent = '--';
    document.getElementById('point-ebullition').textContent = '-- °C';
    document.getElementById('pluie').textContent = '-- mm';
    document.getElementById('neige').textContent = '-- mm';
    document.getElementById('lumiere').textContent = '-- lux';
    document.getElementById('son').textContent = '-- dB';
    document.getElementById('frequence').textContent = '-- Hz';
    document.getElementById('souterrain').textContent = currentAlt === null ? 'Oui' : 'Non'; 

    document.getElementById('cap').textContent = '--°';
    document.getElementById('vers-lat').textContent = '--';
    document.getElementById('vers-lon').textContent = '--';
    document.getElementById('cap-dest').textContent = '--°';
    document.getElementById('dist-dest').textContent = '-- km';
}

function toggleMovement(start) {
    if (start) {
        getGeoLocation(); 
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
    
    document.getElementById('time-s').textContent = '0.00 s';
    document.getElementById('gps-status').textContent = 'En attente...';
    document.getElementById('gps-status').classList.add('warning');
    
    const resetFields = ['vitesse-inst', 'vitesse-3d', 'vitesse-moy', 'vitesse-max', 'vitesse-ms', 'vitesse-mms', 
                       'mach-number', 'pourcent-lumiere', 'distance-km', 'distance-m', 
                       'distance-mm', 'distance-sl', 'distance-al', 'latitude', 'longitude', 
                       'altitude', 'altitude-ft', 'precision-m', 'prec-gps', 'edt', 'hsm', 'hsv', 'niveau-bulle',
                       'eccentricity-comp', 'obliquity-comp', 'solar-longitude', 'solar-day-duration',
                       'culm-soleil', 'lever-lune', 'coucher-lune', 'culmination-lune', 'phase-lune', 'mag-lune',
                       'temp', 'pression', 'humidite', 'vent', 'nuages', 'uv', 'air-qual', 'point-ebullition', 'pluie', 'neige',
                       'cap', 'vers-lat', 'vers-lon', 'cap-dest', 'dist-dest', 'lumiere', 'son', 'frequence', 
                       'vitesse-vert']; 
    
    resetFields.forEach(id => {
        let value = '--';
        if (id.includes('km/h') || id.includes('m/s') || id.includes('mms')) value = '-- km/h'; // Pour vitesse
        if (id.includes('s') && id !== 'vitesse-ms' && id !== 'vitesse-mms' && id !== 'distance-sl' && id !== 'vitesse-vert') value = '-- s';
        if (id === 'vitesse-vert' || id === 'vitesse-ms') value = '-- m/s'; 
        if (id === 'vitesse-mms') value = '-- mm/s'; 
        if (id === 'hsm' || id === 'hsv' || id === 'culm-soleil' || id === 'solar-day-duration' || id.includes('lune')) value = '--:--:--';
        if (id === 'niveau-bulle' || id.includes('cap')) value = '--°';
        if (id === 'altitude-ft') value = '-- ft';
        if (id === 'solar-longitude') value = '--°';
        if (id.includes('%')) value = '--%';
        
        document.getElementById(id).textContent = value;
    });

    document.getElementById('souterrain').textContent = 'Non';
}

// --- INITIALISATION AU CHARGEMENT DE LA PAGE ---
function initializeCockpit() {
    getGeoLocation(); 
    startBubbleLevel(); 
    updateCelestialData(); 
    
    setInterval(updateAtomicTime, 10);
    setInterval(updateCelestialData, 1000);
}

window.onload = initializeCockpit;
