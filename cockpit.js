// --- CONSTANTES ET VARIABLES GLOBALES ---
const C_LIGHT_MS = 299792458;  
const C_SON_MS = 343;           
const EARTH_ROTATION_RATE = 15; // Taux de rotation de la Terre en degrés par heure

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
    // Données Célestes
    culmSoleil: "13:00:00",
    leverLune: "18:00:00",
    coucherLune: "05:00:00",
    culmCune: "00:30:00",
    magLune: "0.9",
    phaseLune: "75",
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

// --- FONCTION UTILITAIRE : HEURE ATOMIQUE (UTC) ---

/**
 * Récupère l'heure atomique actuelle (UTC) et la retourne en heures décimales.
 * @returns {number} Heure UTC en heures décimales (ex: 13.015)
 */
function getAtomicTimeUTC() {
    const now = new Date();
    const utcHours = now.getUTCHours();
    const utcMinutes = now.getUTCMinutes();
    const utcSeconds = now.getUTCSeconds();
    
    // Retourne l'heure totale en heures décimales (ex: 13h + 0.015h)
    return utcHours + (utcMinutes / 60) + (utcSeconds / 3600);
}


// --- FONCTIONS ASTRONOMIQUES CLÉS ---

/**
 * Calcule l'Heure Solaire Locale Moyenne (HSLM) et l'Heure Solaire Locale Vraie (HSLV)
 * en utilisant l'heure atomique (UTC) et la longitude GPS.
 * @param {number} longitude - Longitude actuelle de l'appareil en degrés décimaux.
 * @returns {object} {hsmTime, hsvTime, edtSeconds}
 */
function calculateLocalSolarTime(longitude) {
    const now = new Date();
    
    // 1. Référence Atomique (UTC)
    const utcTotalHours = getAtomicTimeUTC();

    // 2. Décalage de la Longitude pour l'Heure Solaire Moyenne Locale (HSLM)
    const longitudeOffsetHours = longitude / EARTH_ROTATION_RATE;

    // HSLM = UTC + Longitude Offset
    let hsmTotalHours = utcTotalHours + longitudeOffsetHours;

    // Normalisation à 24 heures (gère les heures > 24 ou < 0)
    hsmTotalHours = (hsmTotalHours % 24 + 24) % 24; 

    const hsmHours = Math.floor(hsmTotalHours);
    const hsmMinutes = Math.floor((hsmTotalHours - hsmHours) * 60);
    const hsmSeconds = Math.floor(((hsmTotalHours - hsmHours) * 60 - hsmMinutes) * 60);
    
    const hsmTime = `${String(hsmHours).padStart(2, '0')}:${String(hsmMinutes).padStart(2, '0')}:${String(hsmSeconds).padStart(2, '0')}`;

    // 3. Équation du Temps (EDT) - Approximation dynamique
    const yearStart = new Date(now.getFullYear(), 0, 1);
    const dayOfYear = Math.floor((now - yearStart) / (1000 * 60 * 60 * 24));
    
    const edtMinutes = 7.3 * Math.sin(2 * Math.PI * (dayOfYear + 10) / 365) - 9.9 * Math.sin(4 * Math.PI * (dayOfYear + 10) / 365);
    const edtSeconds = Math.round(edtMinutes * 60);

    // 4. Heure Solaire Vraie Locale (HSLV)
    let hsvTotalSeconds = hsmTotalHours * 3600 + edtSeconds;
    hsvTotalSeconds = (hsvTotalSeconds % 86400 + 86400) % 86400;

    const hsvHours = Math.floor(hsvTotalSeconds / 3600);
    const hsvMinutes = Math.floor((hsvTotalSeconds % 3600) / 60);
    const hsvSecondsFinal = Math.floor(hsvTotalSeconds % 60);

    const hsvTime = `${String(hsvHours).padStart(2, '0')}:${String(hsvMinutes).padStart(2, '0')}:${String(hsvSecondsFinal).padStart(2, '0')}`;

    return { hsmTime, hsvTime, edtSeconds };
}


// --- GESTION DU NIVEAU À BULLE (DeviceOrientationEvent) ---

/**
 * Démarre la surveillance des capteurs d'orientation (gyroscope/accéléromètre).
 */
function startBubbleLevel() {
    if (!('DeviceOrientationEvent' in window)) {
        document.getElementById('niveau-bulle').textContent = 'N/A (Capteur non supporté)';
        return;
    }

    // Fonction de rappel pour l'événement d'orientation
    window.addEventListener('deviceorientation', function(event) {
        // 'gamma' (Latéral) et 'beta' (Profondeur) sont les inclinaisons
        const tiltX = event.gamma; 
        const tiltY = event.beta;

        if (tiltX !== null && tiltY !== null) {
            const resultX = tiltX.toFixed(1); 
            const resultY = tiltY.toFixed(1); 
            
            document.getElementById('niveau-bulle').textContent = 
                `Lat: ${resultX}° | Prof: ${resultY}°`;
        }
    }, true);
    
    // Gestion de l'autorisation des capteurs sur iOS 13+
    if (typeof DeviceOrientationEvent.requestPermission === 'function') {
        // Si l'accès n'a pas été accordé (ou est le premier chargement), demandez-le.
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


// --- GESTION DES CAPTEURS (GPS) ---

/** * Obtient la position GPS et met à jour les coordonnées. */
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
            
            currentLat = coords.latitude;
            currentLon = coords.longitude; // Variable clé pour le calcul solaire
            currentAlt = coords.altitude;

            if (coords.speed !== null && coords.speed !== undefined) {
                currentSpeedMS = Math.max(0, coords.speed); 
            } else {
                currentSpeedMS = 0;
            }

            document.getElementById('latitude').textContent = currentLat.toFixed(6);
            document.getElementById('longitude').textContent = currentLon.toFixed(6);
            document.getElementById('altitude').textContent = currentAlt !== null ? `${currentAlt.toFixed(1)} m` : '--';
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

// --- FONCTIONS DE MISE À JOUR PRINCIPALES ---

/** Met à jour la section Temps & Vitesse. */
function updateNavigationData() {
    if (!intervalId) return;

    distanceTraveled += (currentSpeedMS / 1000); 

    const V_KPH = currentSpeedMS * 3.6;
    const V_MMS = currentSpeedMS * 1000;
    const V_LIGHT_RATIO = currentSpeedMS / C_LIGHT_MS;
    const V_SON_RATIO = currentSpeedMS / C_SON_MS;

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

    document.getElementById('pourcent-lumiere').textContent = `${(V_LIGHT_RATIO * 100).toFixed(8)}%`;
    document.getElementById('pourcent-son').textContent = `${(V_SON_RATIO * 100).toFixed(2)}%`;
    
    document.getElementById('distance-km').textContent = `${distanceTraveled.toFixed(3)} km`;
    document.getElementById('distance-m').textContent = `${DISTANCE_M.toFixed(1)} m`;
    document.getElementById('distance-mm').textContent = `${(DISTANCE_M * 1000).toFixed(0)} mm`;
    
    document.getElementById('distance-sl').textContent = `${DISTANCE_SL.toPrecision(4)} s lumière`;
    document.getElementById('distance-al').textContent = `${DISTANCE_AL.toPrecision(4)} al`;
    
    timeElapsed++; 
}

/** Met à jour les données célestes, y compris la nouvelle HSLM et HSLV */
function updateCelestialAndMockData() {
    const now = new Date(); 
    
    let hsmTime = '--:--:--';
    let hsvTime = '--:--:--';
    let edtSeconds = '--';
    
    // Exécute le calcul de l'heure solaire SEULEMENT si nous avons une longitude GPS
    if (currentLon !== null) {
        const solarTimes = calculateLocalSolarTime(currentLon);
        hsmTime = solarTimes.hsmTime;
        hsvTime = solarTimes.hsvTime;
        edtSeconds = solarTimes.edtSeconds;
    }

    // --- Heure Solaire Moyenne (HSLM) et Vraie (HSLV)
    document.getElementById('hsm').textContent = hsmTime;
    document.getElementById('hsv').textContent = hsvTime;
    document.getElementById('edt').textContent = `${edtSeconds} s`;

    // --- Horloge Minecraft (simple conversion temporelle)
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


    // --- Données Statiques / Mock
    document.getElementById('culm-soleil').textContent = MockData.culmSoleil;
    document.getElementById('phase-lune').textContent = `${MockData.phaseLune}%`;
    document.getElementById('mag-lune').textContent = MockData.magLune;
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

    // Les capteurs non implémentés restent en MOCK
    document.getElementById('lumiere').textContent = '-- lux (MOCK)';
    document.getElementById('son').textContent = '-- dB (MOCK)';
    document.getElementById('frequence').textContent = '-- Hz (MOCK)';
    document.getElementById('souterrain').textContent = currentAlt === null ? 'Oui' : 'Non'; 
}


// --- CHRONOMÈTRE ET CONTRÔLES ---

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
                       'pourcent-lumiere', 'pourcent-son', 'distance-km', 'distance-m', 
                       'distance-mm', 'distance-sl', 'distance-al', 'latitude', 'longitude', 'altitude', 'precision-m', 'prec-gps', 'edt', 'hsm', 'hsv', 'niveau-bulle'];
    resetFields.forEach(id => {
        let value = '--';
        if (id.includes('m/s') || id.includes('km/h') || id.includes('%') || id.includes('km') || id.includes('m') || id.includes('mm') || id.includes('s lumière')) {
             value = id.includes('s') ? '-- s' : '--';
             value = value.replace('s', ''); 
        }
        if (id === 'edt') value = '-- s';
        if (id === 'hsm' || id === 'hsv') value = '--:--:--';
        if (id === 'niveau-bulle') value = '--°';
        
        document.getElementById(id).textContent = value;
    });

    document.getElementById('souterrain').textContent = 'Non';
}

// --- INITIALISATION AU CHARGEMENT DE LA PAGE ---
function initializeCockpit() {
    getGeoLocation(); 
    startBubbleLevel(); 
    setInterval(updateCelestialAndMockData, 1000);
}

window.onload = initializeCockpit;
