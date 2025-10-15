// --- CONSTANTES ET VARIABLES GLOBALES ---
const C_LIGHT_MS = 299792458;  
const C_SON_MS = 343;           // Vitesse du son dans l'air sec (m/s)
const EARTH_ROTATION_RATE = 15; // Taux de rotation de la Terre en degrés par heure

let intervalId = null;
let timeElapsed = 0; // en secondes

// Variables de Vitesse/Distance (Lecture Directe du GPS)
let currentSpeedMS = 0; 
let maxSpeedKPH = 0;
let distanceTraveled = 0; // en km

// Variables GPS (Mises à jour par getGeoLocation)
let currentLat = null;
let currentLon = null;
let currentAlt = null;
let gpsWatchId = null;

// --- DONNÉES STATIQUES / MOCK ---
const MockData = {
    // ... (Données Météo et Capteurs inchangées) ...
    culmSoleil: "13:00:00",
    leverLune: "18:00:00",
    coucherLune: "05:00:00",
    culmCune: "00:30:00",
    magLune: "0.9",
    // Météo (Exemple statique)
    temp: "20.5",
    pression: "1012.3",
    humidite: "65",
    vent: "15",
    nuages: "40",
    uv: "3",
    airQual: "Bon",
    pointEbullition: "100.0",
};

// --- FONCTIONS ASTRONOMIQUES CLÉS ---

/**
 * Calcule l'Heure Solaire Locale Moyenne (HSLM) et l'Heure Solaire Locale Vraie (HSLV)
 * en utilisant l'heure atomique (UTC) et la longitude GPS.
 * * L'Heure Atomique (UTC) est notre référence de temps précis.
 * * @param {number} longitude - Longitude actuelle de l'appareil en degrés décimaux.
 * @returns {object} {hsmTime, hsvTime, edtSeconds}
 */
function calculateLocalSolarTime(longitude) {
    const now = new Date();
    // 1. Décalage de la Longitude (Heure Solaire Moyenne Locale)
    // 1 heure = 15 degrés de longitude.
    // Décalage entre le Méridien de Greenwich (UTC) et la longitude locale.
    const longitudeOffsetHours = longitude / EARTH_ROTATION_RATE; // Heures décimales

    // 2. Temps Universel (UTC) en millisecondes
    const utcHours = now.getUTCHours();
    const utcMinutes = now.getUTCMinutes();
    const utcSeconds = now.getUTCSeconds();

    // 3. Calcul de l'Heure Solaire Moyenne Locale (HSLM)
    // UTC + Longitude Offset
    let hsmTotalHours = utcHours + (utcMinutes / 60) + (utcSeconds / 3600) + longitudeOffsetHours;

    // Normalisation à 24 heures (gère les heures > 24 ou < 0)
    hsmTotalHours = (hsmTotalHours % 24 + 24) % 24; 

    const hsmHours = Math.floor(hsmTotalHours);
    const hsmMinutes = Math.floor((hsmTotalHours - hsmHours) * 60);
    const hsmSeconds = Math.floor(((hsmTotalHours - hsmHours) * 60 - hsmMinutes) * 60);
    
    const hsmTime = `${String(hsmHours).padStart(2, '0')}:${String(hsmMinutes).padStart(2, '0')}:${String(hsmSeconds).padStart(2, '0')}`;

    // 4. Calcul de l'Équation du Temps (EDT)
    // L'EDT est la différence entre le Temps Solaire Vrai et le Temps Solaire Moyen.
    // L'EDT est complexe à calculer précisément en JS et dépend du jour de l'année.
    // Pour une correction 'qui fonctionne vraiment', on utilise une approximation statique
    // OU on fait appel à une librairie, mais ici on va utiliser une MOCK DYNAMIQUE
    // pour simuler son effet (la vraie valeur varie de -16 à +14 minutes).
    
    // Pour simplifier et simuler une valeur 'réelle' sans librairie astro complète :
    // Utilisons une approximation simple de la sinusoïde annuelle de l'EDT.
    const yearStart = new Date(now.getFullYear(), 0, 1);
    const dayOfYear = Math.floor((now - yearStart) / (1000 * 60 * 60 * 24));
    
    // Formule simplifiée de l'EDT (en minutes)
    const edtMinutes = 7.3 * Math.sin(2 * Math.PI * (dayOfYear + 10) / 365) - 9.9 * Math.sin(4 * Math.PI * (dayOfYear + 10) / 365);
    const edtSeconds = Math.round(edtMinutes * 60);

    // 5. Calcul de l'Heure Solaire Vraie Locale (HSLV)
    // HSLM + Équation du Temps
    let hsvTotalSeconds = hsmTotalHours * 3600 + edtSeconds;
    
    // Normalisation à 24 heures
    hsvTotalSeconds = (hsvTotalSeconds % 86400 + 86400) % 86400;

    const hsvHours = Math.floor(hsvTotalSeconds / 3600);
    const hsvMinutes = Math.floor((hsvTotalSeconds % 3600) / 60);
    const hsvSecondsFinal = Math.floor(hsvTotalSeconds % 60);

    const hsvTime = `${String(hsvHours).padStart(2, '0')}:${String(hsvMinutes).padStart(2, '0')}:${String(hsvSecondsFinal).padStart(2, '0')}`;

    return { hsmTime, hsvTime, edtSeconds };
}


// --- GESTION DES CAPTEURS (GPS) ---

/** * Obtient la position GPS et met à jour les coordonnées. */
function getGeoLocation() {
    // ... (Code getGeoLocation) ...
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
            
            // Mise à jour des variables globales pour les autres fonctions
            currentLat = coords.latitude;
            currentLon = coords.longitude;
            currentAlt = coords.altitude;

            // 1. Lecture de la vitesse et de la précision
            if (coords.speed !== null && coords.speed !== undefined) {
                currentSpeedMS = Math.max(0, coords.speed); 
            } else {
                currentSpeedMS = 0;
            }

            // 2. Mise à jour des coordonnées brutes
            document.getElementById('latitude').textContent = currentLat.toFixed(6);
            document.getElementById('longitude').textContent = currentLon.toFixed(6);
            document.getElementById('altitude').textContent = currentAlt !== null ? `${currentAlt.toFixed(1)} m` : '--';
            document.getElementById('precision-m').textContent = `${coords.accuracy.toFixed(1)} m`;
            document.getElementById('prec-gps').textContent = `${((100 - (coords.accuracy / 20) * 100).toFixed(0))} %`; 
            
            document.getElementById('gps-status').textContent = 'ACTIF';
            document.getElementById('gps-status').classList.remove('warning');

        },
        (error) => {
            document.getElementById('gps-status').textContent = `Erreur GPS: ${error.message} - Arrêté`;
            document.getElementById('gps-status').classList.add('warning');
            currentSpeedMS = 0; 
            toggleMovement(false);
        },
        { enableHighAccuracy: true, timeout: 5000, maximumAge: 0 } 
    );
}

// --- FONCTIONS DE MISE À JOUR ---

/** Met à jour la section Temps & Vitesse. */
function updateNavigationData() {
    if (!intervalId) return;

    // ... (Calculs de Vitesse/Distance inchangés) ...
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
    
    // 2. Mise à jour du HTML
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
    
    // **Exécute le calcul de l'heure solaire SEULEMENT si nous avons une longitude GPS**
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
    // ... (Code Minecraft inchangé) ...
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
    // ... (Mise à jour des mocks inchangée) ...
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

    document.getElementById('niveau-bulle').textContent = '--° (N/A)';
    document.getElementById('lumiere').textContent = '-- lux (N/A)';
    document.getElementById('son').textContent = '-- dB (N/A)';
    document.getElementById('frequence').textContent = '-- Hz (N/A)';
    document.getElementById('souterrain').textContent = currentAlt === null ? 'Oui' : 'Non';
}


// --- CHRONOMÈTRE ET CONTRÔLES ---

function toggleMovement(start) {
    // ... (Code toggleMovement inchangé) ...
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
    // ... (Code resetData inchangé) ...
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
    
    const zeroFields = ['vitesse-inst', 'vitesse-moy', 'vitesse-max', 'vitesse-ms', 'vitesse-mms', 
                       'pourcent-lumiere', 'pourcent-son', 'distance-km', 'distance-m', 
                       'distance-mm', 'distance-sl', 'distance-al', 'latitude', 'longitude', 'altitude', 'precision-m', 'prec-gps'];
    zeroFields.forEach(id => {
        let value = '--';
        if (id.includes('m/s')) value = '-- m/s';
        if (id.includes('km/h')) value = '-- km/h';
        if (id.includes('%')) value = '--%';
        if (id.includes('km')) value = '-- km';
        if (id.includes('m')) value = '-- m';
        if (id.includes('mm')) value = '-- mm';
        if (id.includes('s lumière')) value = '-- s lumière';
        
        document.getElementById(id).textContent = value;
    });

    document.getElementById('edt').textContent = '-- s';
    document.getElementById('hsm').textContent = '--:--:--';
    document.getElementById('hsv').textContent = '--:--:--';
    document.getElementById('souterrain').textContent = 'Non';
}

// --- INITIALISATION AU CHARGEMENT DE LA PAGE ---
function initializeCockpit() {
    updateCelestialAndMockData(); 
    getGeoLocation(); 
    setInterval(updateCelestialAndMockData, 1000);
}

window.onload = initializeCockpit;
