// --- CONSTANTES ET VARIABLES GLOBALES ---
const C_LIGHT_MS = 299792458;  
const C_SON_MS_SEA_LEVEL = 343; 
const METER_TO_FEET = 3.28084;  
const EARTH_ROTATION_RATE = 15; // 15 degrés par heure
const SYNODIC_MONTH = 29.53058867;

let intervalId = null;
let timeElapsed = 0; // en secondes

// Variables de Vitesse/Distance/GPS
let currentSpeedMS = 0; 
let maxSpeedKPH = 0;
let distanceTraveled = 0; // en km

let currentLat = null;
let currentLon = null;
let currentAlt = null;
let gpsWatchId = null;

// --- DONNÉES STATIQUES / MOCK ---
const MockData = {
    // Les heures de Lever/Coucher de Lune restent en MOCK.
    leverLune: "18:00:00",
    coucherLune: "05:00:00",
    culmCune: "00:30:00",
    // Données Météo (Exemple statique)
    temp: "20.5",
    pression: "1012.3",
    humidite: "65",
    vent: "15",
    nuages: "40",
    uv: "3",
    airQual: "Bon",
    pointEbullition: "100.0",
    pluie: "0.0",
    neige: "0.0",
};

// --- FONCTIONS UTILITAIRES DE TEMPS ---

/** * Retourne l'heure UTC en heures totales (pour les calculs astronomiques)
 * Inclut les millisecondes pour une précision maximale.
 */
function getAtomicTimeUTC() {
    const now = new Date();
    const totalHours = now.getUTCHours() + (now.getUTCMinutes() / 60) + (now.getUTCSeconds() / 3600) + (now.getUTCMilliseconds() / 3600000);
    return totalHours;
}

/** * Met à jour l'horloge UTC à la milliseconde (style time.is).
 * La plus haute fréquence de mise à jour.
 */
function updateAtomicTime() {
    const now = new Date();
    const utcHours = String(now.getUTCHours()).padStart(2, '0');
    const utcMinutes = String(now.getUTCMinutes()).padStart(2, '0');
    const utcSeconds = String(now.getUTCSeconds()).padStart(2, '0');
    // Affiche les millisecondes avec deux décimales (centisecondes)
    const utcMilliseconds = String(Math.floor(now.getUTCMilliseconds() / 10)).padStart(2, '0'); 
    
    document.getElementById('atomic-time').textContent = 
        `${utcHours}:${utcMinutes}:${utcSeconds}.${utcMilliseconds} UTC`;
}


function calculateLunarData() { 
    // Calcul de la phase lunaire, base sur la date UTC
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


// --- FONCTIONS ASTRONOMIQUES CLÉS (Base sur l'Heure Atomique) ---

/**
 * Calcule HSLM, HSLV, EDT, Culmination et les composants orbitaux, 
 * en utilisant getAtomicTimeUTC() comme référence de temps.
 */
function calculateLocalSolarTime(longitude) {
    const now = new Date();
    // TEMPS DE DÉPART : Heure Atomique (UTC) complète
    const utcTotalHours = getAtomicTimeUTC(); 
    const longitudeOffsetHours = longitude / EARTH_ROTATION_RATE;

    // --- Calcul du Jour de l'Année (DoY) basé sur la date UTC ---
    const yearStart = new Date(now.getUTCFullYear(), 0, 1);
    const dayOfYear = (now.getTime() - yearStart.getTime()) / (1000 * 60 * 60 * 24);
    
    // Angle B, en radians
    const B = (2 * Math.PI * (dayOfYear - 81) / 365.25); 

    // --- 1. Longitude Solaire et EDT Composantes (Calcul précis) ---
    
    const eccSeconds = (-7.659 * Math.sin(B)) * 60; // en secondes
    const oblSeconds = (9.87 * Math.sin(2 * B)) * 60; // en secondes
    const edtSeconds = eccSeconds + oblSeconds;

    const solLon = ((dayOfYear / 365.25) * 360) % 360; 

    // --- 2. Heure Solaire Moyenne (HSLM) ---
    // UTC + Décalage Longitude
    let hsmTotalHours = utcTotalHours + longitudeOffsetHours;
    hsmTotalHours = (hsmTotalHours % 24 + 24) % 24; 

    const hsmSecondsTotal = hsmTotalHours * 3600;
    const hsmHours = Math.floor(hsmSecondsTotal / 3600);
    const hsmMinutes = Math.floor((hsmSecondsTotal % 3600) / 60);
    const hsmSeconds = Math.floor(hsmSecondsTotal % 60);
    const hsmTime = `${String(hsmHours).padStart(2, '0')}:${String(hsmMinutes).padStart(2, '0')}:${String(hsmSeconds).padStart(2, '0')}`;

    // --- 3. Heure Solaire Vraie (HSLV) ---
    // HSM + Équation du Temps
    let hsvTotalSeconds = hsmTotalHours * 3600 + edtSeconds;
    hsvTotalSeconds = (hsvTotalSeconds % 86400 + 86400) % 86400;

    const hsvHours = Math.floor(hsvTotalSeconds / 3600);
    const hsvMinutes = Math.floor((hsvTotalSeconds % 3600) / 60);
    const hsvSecondsFinal = Math.floor(hsvTotalSeconds % 60);
    const hsvTime = `${String(hsvHours).padStart(2, '0')}:${String(hsvMinutes).padStart(2, '0')}:${String(hsvSecondsFinal).padStart(2, '0')}`;

    // --- 4. Culmination (Midi Solaire Vrai - Heure locale de la machine) ---
    // Point de la journée où le soleil est au zénith local.
    const noonUTCSec = 12 * 3600; 
    const longitudeOffsetSeconds = longitudeOffsetHours * 3600;

    // Calcul de la culmination en secondes UTC
    let culmTotalSeconds = noonUTCSec - longitudeOffsetSeconds - edtSeconds;
    
    // CONVERSION EN HEURE LOCALE (fuseau horaire de la machine)
    const localOffset = now.getTimezoneOffset() * 60; 
    let culmLocalSeconds = culmTotalSeconds - localOffset; 
    culmLocalSeconds = (culmLocalSeconds % 86400 + 86400) % 86400;

    const culmLocalHours = Math.floor(culmLocalSeconds / 3600);
    const culmLocalMinutes = Math.floor((culmLocalSeconds % 3600) / 60);
    const culmLocalSecondsFinal = Math.floor(culmLocalSeconds % 60);
    const culmTime = `${String(culmLocalHours).padStart(2, '0')}:${String(culmLocalMinutes).padStart(2, '0')}:${String(culmLocalSecondsFinal).padStart(2, '0')}`;

    // --- 5. Durée du Jour Solaire ---
    const solarDayDurationSeconds = 86400 + edtSeconds * 0.005; 
    
    const dayHours = Math.floor(solarDayDurationSeconds / 3600);
    const dayMinutes = Math.floor((solarDayDurationSeconds % 3600) / 60);
    const daySeconds = Math.floor(solarDayDurationSeconds % 60);
    const solarDayDuration = `${String(dayHours).padStart(2, '0')}:${String(dayMinutes).padStart(2, '0')}:${String(daySeconds).padStart(2, '0')}`;

    return { 
        hsmTime, 
        hsvTime, 
        edtSeconds: edtSeconds.toFixed(2), // Ex: 14.30 s
        culmTime,
        eccComp: eccSeconds.toFixed(2), 
        oblComp: oblSeconds.toFixed(2), 
        solLon,
        solarDayDuration
    };
}


// --- RESTE DU CODE (inchangé dans sa logique) ---

function startBubbleLevel() { 
    // ... (Logique Niveau à Bulle inchangée) ...
    if (!('DeviceOrientationEvent' in window)) {
        document.getElementById('niveau-bulle').textContent = 'N/A (Capteur non supporté)';
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
                    document.getElementById('niveau-bulle').textContent = 'N/A (Accès refusé)';
                }
            })
            .catch(error => {
                document.getElementById('niveau-bulle').textContent = 'N/A (Erreur capteur)';
                console.error("Erreur d'accès à DeviceOrientation:", error);
            });
    }
}


function getGeoLocation() {
    // ... (Logique GPS inchangée) ...
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
            
            currentLat = coords.latitude;
            currentLon = coords.longitude;
            currentAlt = coords.altitude; 

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

            document.getElementById('precision-m').textContent = `${coords.accuracy.toFixed(1)} m`;
            document.getElementById('prec-gps').textContent = `${((100 - (coords.accuracy / 20) * 100).toFixed(0))} %`; 
            
            document.getElementById('gps-status').textContent = 'ACTIF';
            document.getElementById('gps-status').classList.remove('warning');
            
            updateCelestialAndMockData(); 
        },
        (error) => {
            currentLat = null;
            currentLon = null;
            currentAlt = null; 
            
            document.getElementById('gps-status').textContent = `Erreur GPS: ${error.message} - Arrêté`;
            document.getElementById('gps-status').classList.add('warning');
            currentSpeedMS = 0; 
            if (error.code === 1) { 
                document.getElementById('gps-status').textContent = 'Erreur GPS: Autorisation Refusée !';
            }
        },
        { enableHighAccuracy: true, timeout: 10000, maximumAge: 0 } 
    );
}

function updateNavigationData() {
    if (!intervalId) return;
    
    distanceTraveled += (currentSpeedMS / 1000); 

    const V_KPH = currentSpeedMS * 3.6;
    const V_MMS = currentSpeedMS * 1000;
    const V_LIGHT_RATIO = currentSpeedMS / C_LIGHT_MS;
    const MACH_NUMBER = currentSpeedMS / C_SON_MS_SEA_LEVEL;
    
    if (V_KPH > maxSpeedKPH) {
        maxSpeedKPH = V_KPH;
    }
    
    const avgSpeedKPH = timeElapsed > 0 ? distanceTraveled / (timeElapsed / 3600) : 0;
    const DISTANCE_M = distanceTraveled * 1000;
    const DISTANCE_SL = DISTANCE_M / C_LIGHT_MS; 
    const DISTANCE_AL = DISTANCE_SL / (3600 * 24 * 365.25); 
    
    document.getElementById('time-s').textContent = `${timeElapsed.toFixed(2)} s`;
    document.getElementById('vitesse-inst').textContent = `${V_KPH.toFixed(2)} km/h`;
    document.getElementById('vitesse-moy').textContent = `${avgSpeedKPH.toFixed(2)} km/h`;
    document.getElementById('vitesse-max').textContent = `${maxSpeedKPH.toFixed(2)} km/h`;
    document.getElementById('vitesse-ms').textContent = `${currentSpeedMS.toFixed(2)} m/s`;
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

function updateCelestialAndMockData() {
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

    // --- Heure Solaire et EDT ---
    document.getElementById('culm-soleil').textContent = culmTime;
    document.getElementById('hsm').textContent = hsmTime;
    document.getElementById('hsv').textContent = hsvTime;
    document.getElementById('edt').textContent = `${edtSeconds} s`; 

    // --- Dynamique Orbitale ---
    document.getElementById('eccentricity-comp').textContent = `${eccComp} s`;
    document.getElementById('obliquity-comp').textContent = `${oblComp} s`;
    document.getElementById('solar-longitude').textContent = `${solLon.toFixed(2)}°`;
    document.getElementById('solar-day-duration').textContent = solarDayDuration;
    
    // --- Lune (Dynamique) ---
    document.getElementById('phase-lune').textContent = `${lunarData.phasePercent}%`;
    document.getElementById('mag-lune').textContent = lunarData.magnitude;
    
    // --- Horloge Minecraft et Mocks (inchangé) ---
    const hour = now.getHours();
    const minute = now.getMinutes();
    const second = now.getSeconds();

    const totalSecondsInDay = (hour * 3600) + (minute * 60) + second;
    const minecraftTimeRatio = 1200 / 86400; 
    const minecraftSeconds = totalSecondsInDay * minecraftTimeRatio;

    const mcHours = Math.floor(minecraftSeconds / 50); 
    const mcMinutes = Math.floor((minecraftSeconds % 50) * 1.2); 
    
    const mcTimeStr = `${String(mcHours % 24).padStart(2, '0')}:${String(mcMinutes % 60).padStart(2, '0')}:${String(Math.floor(minecraftSeconds * 20) % 60).padStart(2, '0')}`;
    document.getElementById('horloge-minecraft').textContent = mcTimeStr;

    // Reste des Mocks 
    document.getElementById('lever-lune').textContent = MockData.leverLune;
    document.getElementById('coucher-lune').textContent = MockData.coucherLune;
    document.getElementById('culmination-lune').textContent = MockData.culmCune;
    
    document.getElementById('temp').textContent = `${MockData.temp} °C`;
    document.getElementById('pression').textContent = `${MockData.pression} hPa`;
    document.getElementById('humidite').textContent = `${MockData.humidite}%`;
    document.getElementById('vent').textContent = `${MockData.vent} km/h`;
    document.getElementById('nuages').textContent = `${MockData.nuages}%`;
    document.getElementById('uv').textContent = MockData.uv;
    document.getElementById('air-qual').textContent = MockData.airQual;
    document.getElementById('point-ebullition').textContent = `${MockData.pointEbullition} °C`;
    document.getElementById('pluie').textContent = `${MockData.pluie} mm`;
    document.getElementById('neige').textContent = `${MockData.neige} mm`;

    document.getElementById('lumiere').textContent = '-- lux (MOCK)';
    document.getElementById('son').textContent = '-- dB (MOCK)';
    document.getElementById('frequence').textContent = '-- Hz (MOCK)';
    document.getElementById('souterrain').textContent = currentAlt === null ? 'Oui' : 'Non'; 
}

function toggleMovement(start) {
    if (start) {
        getGeoLocation(); 
        intervalId = setInterval(() => {
            updateCelestialAndMockData(); 
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
    
    document.getElementById('time-s').textContent = '0.00 s';
    document.getElementById('gps-status').textContent = 'En attente...';
    document.getElementById('gps-status').classList.add('warning');
    
    const resetFields = ['vitesse-inst', 'vitesse-moy', 'vitesse-max', 'vitesse-ms', 'vitesse-mms', 
                       'mach-number', 'pourcent-lumiere', 'distance-km', 'distance-m', 
                       'distance-mm', 'distance-sl', 'distance-al', 'latitude', 'longitude', 
                       'altitude', 'altitude-ft', 'precision-m', 'prec-gps', 'edt', 'hsm', 'hsv', 'niveau-bulle',
                       'eccentricity-comp', 'obliquity-comp', 'solar-longitude', 'solar-day-duration'];
    
    resetFields.forEach(id => {
        let value = '--';
        if (id.includes('s') && id !== 'vitesse-ms' && id !== 'vitesse-mms' && id !== 'distance-sl') value = '-- s';
        if (id === 'hsm' || id === 'hsv' || id === 'culm-soleil' || id === 'solar-day-duration') value = '--:--:--';
        if (id === 'niveau-bulle') value = '--°';
        if (id === 'altitude-ft') value = '-- ft';
        if (id === 'solar-longitude') value = '--°';
        
        document.getElementById(id).textContent = value;
    });

    document.getElementById('souterrain').textContent = 'Non';
}

// --- INITIALISATION AU CHARGEMENT DE LA PAGE ---
function initializeCockpit() {
    getGeoLocation(); 
    startBubbleLevel(); 
    updateCelestialAndMockData(); 
    
    // Intervalle de 10ms (1 centiseconde) pour l'Horloge Atomique (time.is)
    setInterval(updateAtomicTime, 10);

    // Intervalle de 1s pour les calculs célestes/orbitaux plus lourds (heuresolaire.com)
    setInterval(updateCelestialAndMockData, 1000);
}

window.onload = initializeCockpit;
