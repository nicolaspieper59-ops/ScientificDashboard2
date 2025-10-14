// --- CONSTANTES ET VARIABLES GLOBALES ---
const C_LIGHT_MS = 299792458;  
const C_SON_MS = 343;           // Vitesse du son dans l'air sec (m/s)

let intervalId = null;
let timeElapsed = 0; // en secondes

// Variables de Vitesse/Distance (Lecture Directe du GPS)
let currentSpeedMS = 0; 
let maxSpeedKPH = 0;
let distanceTraveled = 0; // en km

// Variables GPS
let lastLat = null;
let lastLon = null;
let lastAlt = null;
let gpsWatchId = null;

// --- DONNÉES STATIQUES / MOCK (pour les champs non GPS/Horloge) ---
const MockData = {
    // Soleil & Lune (Exemple statique)
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
    // Céleste (Calculés)
    edt: 107, // Équation du temps en secondes
    phaseLune: "55" // Phase en pourcentage
};

// --- GESTION DES CAPTEURS (GPS) ---

/** * Utilise watchPosition pour obtenir la position, la vitesse (directement) et les mises à jour en direct.
 * Cette méthode est la plus STABLE.
 */
function getGeoLocation() {
    if (!("geolocation" in navigator)) {
        document.getElementById('gps-status').textContent = 'N/A (Non supporté)';
        return;
    }
    
    // Si l'on démarre (intervalId est null), on arrête la surveillance précédente
    if (gpsWatchId) {
        navigator.geolocation.clearWatch(gpsWatchId);
    }
    
    // Démarre la surveillance de la position pour des mises à jour continues
    gpsWatchId = navigator.geolocation.watchPosition(
        (position) => {
            const coords = position.coords;
            
            // 1. Lecture de la vitesse et de la précision
            if (coords.speed !== null && coords.speed !== undefined) {
                // La vitesse est en m/s et est filtrée par le système d'exploitation
                currentSpeedMS = Math.max(0, coords.speed); 
            } else {
                currentSpeedMS = 0;
            }

            // Mise à jour des coordonnées brutes
            document.getElementById('latitude').textContent = coords.latitude.toFixed(6);
            document.getElementById('longitude').textContent = coords.longitude.toFixed(6);
            document.getElementById('altitude').textContent = coords.altitude !== null ? `${coords.altitude.toFixed(1)} m` : '--';
            document.getElementById('precision-m').textContent = `${coords.accuracy.toFixed(1)} m`;
            document.getElementById('prec-gps').textContent = `${((100 - (coords.accuracy / 20) * 100).toFixed(0))} %`; // Estimation simple
            
            document.getElementById('gps-status').textContent = 'ACTIF';
            document.getElementById('gps-status').classList.remove('warning');

            // Mise à jour de la position pour le calcul de la distance (si on en a besoin pour autre chose, mais on utilise ici currentSpeedMS)
            lastLat = coords.latitude;
            lastLon = coords.longitude;
            lastAlt = coords.altitude;
        },
        (error) => {
            document.getElementById('gps-status').textContent = `Erreur GPS: ${error.message} - Arrêté`;
            document.getElementById('gps-status').classList.add('warning');
            currentSpeedMS = 0; 
            // Arrête le chronomètre si le GPS échoue
            toggleMovement(false);
        },
        // Options pour la haute précision et une mise à jour rapide
        { enableHighAccuracy: true, timeout: 5000, maximumAge: 0 } 
    );
}

// --- FONCTIONS DE MISE À JOUR ---

/** Met à jour la section Temps & Vitesse. */
function updateNavigationData() {
    if (!intervalId) return;

    // Calcul de la distance parcourue pendant la dernière seconde (temps du chronomètre)
    // distance = vitesse (m/s) * 1 seconde / 1000 (pour km)
    distanceTraveled += (currentSpeedMS / 1000); 

    // 1. Calculs
    const V_KPH = currentSpeedMS * 3.6;
    const V_MMS = currentSpeedMS * 1000;
    const V_LIGHT_RATIO = currentSpeedMS / C_LIGHT_MS;
    const V_SON_RATIO = currentSpeedMS / C_SON_MS;

    if (V_KPH > maxSpeedKPH) {
        maxSpeedKPH = V_KPH;
    }
    
    // Vitesse moyenne basée sur la distance totale et le temps écoulé du chronomètre
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

/** Met à jour les données du soleil, de la lune et Minecraft */
function updateCelestialAndMockData() {
    const now = new Date(); 
    
    // --- Heure Solaire Moyenne (HSM) et Vraie (HSV)
    const hour = now.getHours();
    const minute = now.getMinutes();
    const second = now.getSeconds();

    const hsmTime = `${String(hour).padStart(2, '0')}:${String(minute).padStart(2, '0')}:${String(second).padStart(2, '0')}`;
    
    let hsvSeconds = (hour * 3600) + (minute * 60) + second + MockData.edt;
    const hsvDate = new Date(hsvSeconds * 1000); 
    const hsvTimeStr = `${String(hsvDate.getUTCHours()).padStart(2, '0')}:${String(hsvDate.getUTCMinutes()).padStart(2, '0')}:${String(hsvDate.getUTCSeconds()).padStart(2, '0')}`;
    
    document.getElementById('hsm').textContent = hsmTime;
    document.getElementById('hsv').textContent = hsvTimeStr;
    document.getElementById('edt').textContent = `${MockData.edt} s`;

    // --- Horloge Minecraft (simple conversion temporelle)
    // 1 jour Minecraft = 20 minutes réelles (1200 secondes)
    const totalSecondsInDay = (hour * 3600) + (minute * 60) + second;
    const minecraftTimeRatio = 1200 / 86400; // 20 minutes / 24 heures
    const minecraftSeconds = totalSecondsInDay * minecraftTimeRatio;

    const mcHours = Math.floor(minecraftSeconds / 50); // 50 secondes réelles = 1h Minecraft
    const mcMinutes = Math.floor((minecraftSeconds % 50) * 1.2); // 50s * 1.2 = 60s
    
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

    // Remplissage des capteurs avec des N/A (nécessite des API ou capteurs spécifiques)
    document.getElementById('niveau-bulle').textContent = '--° (N/A)';
    document.getElementById('lumiere').textContent = '-- lux (N/A)';
    document.getElementById('son').textContent = '-- dB (N/A)';
    document.getElementById('frequence').textContent = '-- Hz (N/A)';
    document.getElementById('souterrain').textContent = lastAlt === null ? 'Oui' : 'Non';
}


// --- CHRONOMÈTRE ET CONTRÔLES ---

/** Bascule le chronomètre (Démarrer/Arrêter). */
function toggleMovement(start) {
    if (start) {
        // Démarre la surveillance GPS pour la vitesse et la position
        getGeoLocation(); 
        
        // Démarre l'intervalle principal
        intervalId = setInterval(() => {
            updateCelestialAndMockData(); 
            updateNavigationData();
        }, 1000);
        
        document.getElementById('gps-status').textContent = 'ACTIF (Acquisition Vitesse)';
        document.getElementById('gps-status').classList.add('warning');
    } else {
        clearInterval(intervalId);
        intervalId = null;
        
        // Arrête la surveillance GPS pour économiser la batterie
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

/** Réinitialise toutes les données de navigation. */
function resetData() {
    toggleMovement(false); // Arrête le mouvement et le GPS
    
    // Réinitialisation des variables de navigation
    timeElapsed = 0;
    currentSpeedMS = 0;
    maxSpeedKPH = 0;
    distanceTraveled = 0;
    lastLat = null;
    lastLon = null;
    lastAlt = null;
    
    document.getElementById('time-s').textContent = '0.00 s';
    document.getElementById('gps-status').textContent = 'En attente...';
    document.getElementById('gps-status').classList.add('warning');
    
    const zeroFields = ['vitesse-inst', 'vitesse-moy', 'vitesse-max', 'vitesse-ms', 'vitesse-mms', 
                       'pourcent-lumiere', 'pourcent-son', 'distance-km', 'distance-m', 
                       'distance-mm', 'distance-sl', 'distance-al', 'latitude', 'longitude', 'altitude', 'precision-m', 'prec-gps'];
    zeroFields.forEach(id => {
        let value = '--';
        if (id.includes('s')) value = '-- m/s';
        if (id.includes('km/h')) value = '-- km/h';
        if (id.includes('lumiere')) value = '--%';
        if (id.includes('son')) value = '--%';
        if (id.includes('km')) value = '-- km';
        if (id.includes('m')) value = '-- m';
        if (id.includes('mm')) value = '-- mm';
        if (id.includes('sl') || id.includes('al')) value = '-- s lumière';
        
        document.getElementById(id).textContent = value;
    });

    document.getElementById('souterrain').textContent = 'Non';
}

// --- INITIALISATION AU CHARGEMENT DE LA PAGE ---
function initializeCockpit() {
    // Met à jour les données statiques/calculées (heure, météo, etc.) immédiatement
    updateCelestialAndMockData(); 

    // Démarre la surveillance GPS de base pour obtenir la position de départ (sans calculer la vitesse tant qu'on n'appuie pas sur Marche)
    getGeoLocation(); 
    
    // Garde le temps, les données célestes, et mock à jour
    setInterval(updateCelestialAndMockData, 1000);
}

// Cette ligne est CRUCIALE : elle s'assure que toutes les fonctions sont chargées avant d'être appelées par les boutons.
window.onload = initializeCockpit;
