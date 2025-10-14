// --- CONSTANTES ET VARIABLES GLOBALES ---
const C_LIGHT_KMS = 299792.458; 
const C_LIGHT_MS = 299792458;  
const MASS_KG = 70;             
const EARTH_RADIUS_M = 6371000; // Rayon moyen de la Terre en mètres

let intervalId = null;
let timeElapsed = 0; // en secondes

// Variables pour le calcul de la vitesse par changement de position (3D)
let currentSpeedMS = 0; 
let maxSpeedKPH = 0;
let distanceTraveled = 0; // en km

// Variables de la position précédente pour le calcul de la vitesse 3D
let lastLat = null;
let lastLon = null;
let lastAlt = null;
let lastTimestamp = null;
let gpsWatchId = null;

// --- DONNÉES ASTRONOMIQUES (Statiques) ---
const AstroData = {
    leverSoleil: "07:54:00",    
    coucherSoleil: "18:41:00",  
    dureeJourSolaire: "10:47:00",
    lonSolaire: 201.2,           
    edt: 107,                   
    phaseLune: "Lune Croissante 🌔",
    culminationLune: "20:30:00",
    saison: "Automne 🍂" 
};

// --- FONCTIONS UTILITAIRES ---

/** Convertit les degrés en radians. */
function toRad(degrees) {
    return degrees * (Math.PI / 180);
}

/** * Calcule la distance horizontale (2D) entre deux points GPS (formule Haversine) 
 * et combine avec la différence d'altitude pour obtenir la distance 3D totale en mètres.
 */
function calculate3dDistance(lat1, lon1, alt1, lat2, lon2, alt2) {
    // 1. Calcul de la distance horizontale (Haversine)
    const dLat = toRad(lat2 - lat1);
    const dLon = toRad(lon2 - lon1);
    
    const a = 
        Math.sin(dLat / 2) * Math.sin(dLat / 2) +
        Math.cos(toRad(lat1)) * Math.cos(toRad(lat2)) * Math.sin(dLon / 2) * Math.sin(dLon / 2);
    
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    const distance2d = EARTH_RADIUS_M * c; // Distance en mètres

    // 2. Calcul de la distance verticale
    // Si l'une des altitudes est manquante, la distance verticale est 0.
    // L'altitude est soustraite pour obtenir la différence entre les deux points.
    const altDiff = (alt1 !== null && alt2 !== null) ? Math.abs(alt2 - alt1) : 0;

    // 3. Calcul de la distance 3D (Pythagore dans l'espace)
    return Math.sqrt(distance2d * distance2d + altDiff * altDiff);
}


// --- GESTION DES CAPTEURS (GPS & Accéléromètre) ---

let accelListenerActive = false;

/** Initialise l'accès à l'accéléromètre. */
function initAccelerometer() {
    if ('DeviceMotionEvent' in window) {
        if (typeof DeviceOrientationEvent.requestPermission === 'function') {
            document.getElementById('accel-status').textContent = 'Autorisation requise (cliquez)';
            document.getElementById('accel-status').onclick = () => {
                DeviceOrientationEvent.requestPermission()
                .then(permissionState => {
                    if (permissionState === 'granted') {
                        window.addEventListener('devicemotion', handleMotion);
                        document.getElementById('accel-status').textContent = 'Actif (m/s²)';
                        document.getElementById('accel-status').classList.remove('warning');
                        document.getElementById('accel-status').onclick = null;
                        accelListenerActive = true;
                    } else {
                        document.getElementById('accel-status').textContent = 'Refusé';
                    }
                })
                .catch(console.error);
            };
        } else {
            window.addEventListener('devicemotion', handleMotion);
            document.getElementById('accel-status').textContent = 'Actif (m/s²)';
            document.getElementById('accel-status').classList.remove('warning');
            accelListenerActive = true;
        }
    } else {
        document.getElementById('accel-status').textContent = 'N/A (Non supporté)';
        document.getElementById('accel-status').classList.add('warning');
    }
}

/** Gère les données de mouvement (lecture brute de l'accélération). */
function handleMotion(event) {
    // Cette fonction ne calcule PAS la vitesse (qui vient du GPS)
    // Elle est conservée pour la lecture brute de l'accélération si besoin.
}

/** Tente d'obtenir le niveau de la batterie. */
function updateBatteryStatus() {
    if ('getBattery' in navigator) {
        navigator.getBattery().then(function(battery) {
            const level = (battery.level * 100).toFixed(0);
            document.getElementById('batterie').textContent = `${level}%`;
            document.getElementById('batterie').classList.remove('warning');
        });
    } else {
         document.getElementById('batterie').textContent = 'N/A (API)';
         document.getElementById('batterie').classList.add('warning');
    }
}

/** * Utilise watchPosition pour calculer la vitesse et la distance en 3D (comme Google Maps) 
 * en utilisant la différence de position et le temps écoulé entre les mises à jour.
 */
function getGeoLocation() {
    if (!("geolocation" in navigator)) {
        document.getElementById('gps-status').textContent = 'N/A (Non supporté)';
        return;
    }
    
    // Arrête la surveillance précédente si elle existe
    if (gpsWatchId) {
        navigator.geolocation.clearWatch(gpsWatchId);
    }
    
    // Démarre la surveillance de la position
    gpsWatchId = navigator.geolocation.watchPosition(
        (position) => {
            const currentTimestamp = position.timestamp;
            const currentLat = position.coords.latitude;
            const currentLon = position.coords.longitude;
            const currentAlt = position.coords.altitude !== null ? position.coords.altitude : null; 
            
            // Affichage de la position
            const altDisplay = currentAlt !== null ? `, ${currentAlt.toFixed(1)} m` : '';
            document.getElementById('rendez-vous').textContent = `${currentLat.toFixed(4)}, ${currentLon.toFixed(4)}${altDisplay}`;
            document.getElementById('gps-status').textContent = `OK (${currentLat.toFixed(4)}, ${currentLon.toFixed(4)})`;
            document.getElementById('gps-status').classList.remove('warning');

            // Calcul de la vitesse et de la distance si le mouvement est en marche
            if (intervalId && lastTimestamp !== null && lastLat !== null) {
                // Temps écoulé en secondes
                const deltaTime = (currentTimestamp - lastTimestamp) / 1000; 

                if (deltaTime > 0) {
                    // Calcul de la distance 3D parcourue en mètres
                    const distanceMeters = calculate3dDistance(
                        lastLat, lastLon, lastAlt, 
                        currentLat, currentLon, currentAlt
                    );
                    
                    // Vitesse = Distance / Temps (en m/s)
                    const calculatedSpeed = distanceMeters / deltaTime;

                    // Mise à jour de la vitesse instantanée (filtrage minimal)
                    currentSpeedMS = Math.max(0, calculatedSpeed); 
                    
                    // Accumulation de la distance totale (en km)
                    distanceTraveled += distanceMeters / 1000; 
                }
            }

            // Mise à jour de la position précédente pour le prochain calcul
            lastLat = currentLat;
            lastLon = currentLon;
            lastAlt = currentAlt;
            lastTimestamp = currentTimestamp;
        },
        (error) => {
            document.getElementById('gps-status').textContent = `Erreur GPS: ${error.message}`;
            currentSpeedMS = 0; 
            lastTimestamp = null; // Réinitialise l'état pour éviter les faux calculs
        },
        // Options pour la haute précision (nécessaire pour l'altitude) et des mises à jour optimales
        { enableHighAccuracy: true, timeout: 5000, maximumAge: 0 } 
    );
}

// --- FONCTIONS DE MISE À JOUR & RELATIVITÉ ---

/** Calcule le Facteur de Lorentz (Dilatation du Temps). */
function calculateLorentzFactor(v_ms) {
    const v_ratio_squared = (v_ms * v_ms) / (C_LIGHT_MS * C_LIGHT_MS);
    if (v_ratio_squared >= 1) {
        return Infinity;
    }
    return 1 / Math.sqrt(1 - v_ratio_squared);
}

/** Met à jour la section Vitesse & Navigation. */
function updateNavigationData() {
    if (!intervalId) return;

    // 1. Calculs
    const V_KPH = currentSpeedMS * 3.6;
    const V_MMS = currentSpeedMS * 1000;
    const V_LIGHT_RATIO = currentSpeedMS / C_LIGHT_MS;
    const LORENTZ_FACTOR = calculateLorentzFactor(currentSpeedMS);

    if (V_KPH > maxSpeedKPH) {
        maxSpeedKPH = V_KPH;
    }
    
    // Vitesse moyenne basée sur la distance totale et le temps écoulé du chronomètre
    const avgSpeedKPH = timeElapsed > 0 ? distanceTraveled / (timeElapsed / 3600) : 0;
    const E_KINETIC = 0.5 * MASS_KG * (currentSpeedMS * currentSpeedMS);
    
    const DISTANCE_M = distanceTraveled * 1000;
    const DISTANCE_SL = DISTANCE_M / C_LIGHT_MS; 
    const DISTANCE_AL = DISTANCE_SL / (3600 * 24 * 365.25); 
    
    // 2. Mise à jour du HTML
    document.getElementById('time-s').textContent = `${timeElapsed.toFixed(0)} s`;
    document.getElementById('vitesse-inst').textContent = `${V_KPH.toFixed(2)} km/h`;
    document.getElementById('vitesse-moy').textContent = `${avgSpeedKPH.toFixed(2)} km/h`;
    document.getElementById('vitesse-max').textContent = `${maxSpeedKPH.toFixed(2)} km/h`;
    document.getElementById('vitesse-ms').textContent = `${currentSpeedMS.toFixed(2)} m/s`;
    document.getElementById('vitesse-mms').textContent = `${V_MMS.toFixed(0)} mm/s`;

    document.getElementById('gamma-factor').textContent = LORENTZ_FACTOR.toFixed(4);
    document.getElementById('pourcent-lumiere').textContent = `${(V_LIGHT_RATIO * 100).toFixed(4)}%`;
    document.getElementById('pourcent-lumiere-precise').textContent = `${V_LIGHT_RATIO.toFixed(8)} c`;
    
    document.getElementById('distance-km').textContent = `${distanceTraveled.toFixed(2)} km`;
    document.getElementById('distance-m').textContent = `${DISTANCE_M.toFixed(0)} m`;
    document.getElementById('distance-mm').textContent = `${(DISTANCE_M * 1000).toFixed(0)} mm`;
    
    document.getElementById('distance-sl').textContent = `${DISTANCE_SL.toPrecision(4)} s lumière`;
    document.getElementById('distance-al').textContent = `${DISTANCE_AL.toPrecision(4)} al`;
    document.getElementById('energie-cinetique').textContent = `${E_KINETIC.toFixed(0)} J`;
    
    timeElapsed++; 
}

// --- CHRONOMÈTRE ET CONTRÔLES ---

/** Bascule le chronomètre (Démarrer/Arrêter). */
function toggleMovement(start) {
    if (start) {
        // Démarre la surveillance GPS et le calcul de vitesse/distance
        getGeoLocation(); 
        
        intervalId = setInterval(() => {
            updateCelestialData(); 
            updateNavigationData();
        }, 1000);
        
        document.getElementById('gps-status').textContent = 'Acquisition GPS (Calcul 3D)...';
        document.getElementById('gps-status').classList.add('warning');
    } else {
        clearInterval(intervalId);
        intervalId = null;
        
        // Arrête la surveillance GPS
        if (gpsWatchId) {
            navigator.geolocation.clearWatch(gpsWatchId);
            gpsWatchId = null;
        }

        document.getElementById('gps-status').textContent = 'Arrêté';
        document.getElementById('gps-status').classList.remove('warning');
        
        currentSpeedMS = 0; 
        lastTimestamp = null; // Important : réinitialise l'état précédent à l'arrêt
        lastLat = null;
        lastLon = null;
        lastAlt = null;
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
    lastTimestamp = null;
    
    document.getElementById('time-s').textContent = '0 s';
    document.getElementById('gps-status').textContent = 'En attente...';
    document.getElementById('gps-status').classList.add('warning');
    
    const zeroFields = ['vitesse-inst', 'vitesse-moy', 'vitesse-max', 'vitesse-ms', 'vitesse-mms', 
                       'pourcent-lumiere', 'pourcent-lumiere-precise', 'distance-km', 'distance-m', 
                       'distance-mm', 'distance-sl', 'distance-al', 'energie-cinetique'];
    zeroFields.forEach(id => {
        let value = '0';
        if (id.includes('lumiere')) value = '0%';
        if (id.includes('sl') || id.includes('al')) value = '0 s lumière';
        if (id.includes('energie-cinetique')) value = '0 J';
        
        document.getElementById(id).textContent = value;
    });
    document.getElementById('gamma-factor').textContent = '1.0000';
}

// --- FONCTIONS CÉLESTES (inchangées) ---

/** Parse un temps HH:MM:SS en secondes depuis minuit. */
function parseTime(timeStr) {
    const parts = timeStr.split(':');
    return parseInt(parts[0]) * 3600 + parseInt(parts[1]) * 60 + parseInt(parts[2]);
}

/** Met à jour le temps et les données célestes. */
function updateCelestialData() {
    const now = new Date(); 
    
    // 1. Heure Atomique (UTC) et Date Locale
    document.getElementById('utc-time').textContent = now.toUTCString().split(' ')[4] + " UTC";
    document.getElementById('date').textContent = now.toLocaleDateString('fr-FR');
    
    // 2. Calcul Heure Solaire Moyenne (HSM) et Vraie (HSV)
    const hour = now.getHours();
    const minute = now.getMinutes();
    const second = now.getSeconds();

    const hsmTime = `${String(hour).padStart(2, '0')}:${String(minute).padStart(2, '0')}:${String(second).padStart(2, '0')}`;
    
    let hsvSeconds = (hour * 3600) + (minute * 60) + second + AstroData.edt;
    const hsvDate = new Date(hsvSeconds * 1000); 
    const hsvTimeStr = `${String(hsvDate.getUTCHours()).padStart(2, '0')}:${String(hsvDate.getUTCMinutes()).padStart(2, '0')}:${String(hsvDate.getUTCSeconds()).padStart(2, '0')}`;
    
    document.getElementById('hsm').textContent = hsmTime;
    document.getElementById('hsv').textContent = hsvTimeStr;
    document.getElementById('edt').textContent = `+${AstroData.edt} s`;

    // 3. Statut Jour/Nuit et Médaillon
    const sunrise = parseTime(AstroData.leverSoleil);
    const sunset = parseTime(AstroData.coucherSoleil);
    const currentTimeSeconds = (hour * 3600) + (minute * 60) + second; 
    
    let isDay = (currentTimeSeconds > sunrise) && (currentTimeSeconds < sunset);
    let jourNuitStatus = isDay ? "Jour ☀️" : "Nuit 🌑";
    document.getElementById('jour-nuit').textContent = jourNuitStatus;
    document.getElementById('horloge-cosmique').textContent = `${AstroData.saison} ${jourNuitStatus}`;
    
    // 4. Autres données
    const isVisible = (currentTimeSeconds < sunrise) || (currentTimeSeconds > sunset);
    document.getElementById('polaire').textContent = isVisible ? "Visible" : "N/A (Invisible)";
    document.getElementById('lon-solaire').textContent = `${AstroData.lonSolaire.toFixed(1)}°`;
    document.getElementById('djs').textContent = AstroData.dureeJourSolaire;
    document.getElementById('lever-soleil').textContent = AstroData.leverSoleil;
    document.getElementById('coucher-soleil').textContent = AstroData.coucherSoleil;
    document.getElementById('phase-lune').textContent = AstroData.phaseLune + (isDay ? " (Visible de Jour !)" : "");
    document.getElementById('culmination-lune').textContent = AstroData.culminationLune;
}


// --- INITIALISATION AU CHARGEMENT DE LA PAGE ---
function initializeCockpit() {
    updateCelestialData(); 

    getGeoLocation(); // Démarre la surveillance GPS de base pour la position initiale
    updateBatteryStatus();
    initAccelerometer(); 
    
    // Garde le temps et les données célestes à jour chaque seconde
    setInterval(updateCelestialData, 1000);

    // Mise à jour de la batterie chaque minute
    setInterval(updateBatteryStatus, 60000); 
}

window.onload = initializeCockpit;
