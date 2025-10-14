// --- CONSTANTES ET VARIABLES GLOBALES ---
const C_LIGHT_KMS = 299792.458; 
const C_LIGHT_MS = 299792458;  
const MASS_KG = 70;             // Masse du voyageur (pour l'énergie cinétique)

let intervalId = null;
let timeElapsed = 0; // en secondes
let startTime = 0;

// Utilisation de l'heure système réelle (pas de simulation)
let currentLat = 43.2965; // Marseille Lat (Défaut)
let currentLon = 5.3698;  // Marseille Lon (Défaut) 

let currentSpeedMS = 0; 
let maxSpeedKPH = 0;
let distanceTraveled = 0; // en km

// --- DONNÉES ASTRONOMIQUES (Basé sur le 14 Octobre 2025, Marseille) ---
// Utilisation de données statiques pour les calculs célestes car la librairie SunCalc est absente.
const AstroData = {
    leverSoleil: "07:54:00",    // Heure de lever du soleil (CEST)
    coucherSoleil: "18:41:00",  // Heure de coucher du soleil (CEST)
    dureeJourSolaire: "10:47:00",
    lonSolaire: 201.2,           
    edt: 107,                    // Équation du Temps (en secondes)
    phaseLune: "Lune Croissante 🌔",
    culminationLune: "20:30:00",
    saison: "Automne 🍂" 
};

// --- GESTION DES CAPTEURS (Prise en charge au mieux) ---

let lastAccelTime = 0;

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
        }
    } else {
        document.getElementById('accel-status').textContent = 'N/A (Non supporté)';
        document.getElementById('accel-status').classList.add('warning');
    }
}

/** Gère les données de mouvement et intègre la vitesse. */
function handleMotion(event) {
    if (!lastAccelTime || !intervalId) return; 

    const currentAccelTime = performance.now();
    const deltaTime = (currentAccelTime - lastAccelTime) / 1000;
    
    const acc = event.accelerationIncludingGravity || event.acceleration;
    
    if (acc) {
        const accX = acc.x || 0;
        const accY = acc.y || 0;
        const accZ = acc.z || 0;
        
        const acceleration = Math.sqrt(accX * accX + accY * accY + accZ * accZ);
        
        currentSpeedMS += acceleration * deltaTime;
        distanceTraveled += (currentSpeedMS / 1000) * deltaTime;
    }

    lastAccelTime = currentAccelTime;
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

/** Tente d'obtenir la position GPS actuelle et met à jour les coordonnées. */
function getGeoLocation() {
    if ("geolocation" in navigator) {
        navigator.geolocation.getCurrentPosition(
            (position) => {
                currentLat = position.coords.latitude.toFixed(4);
                currentLon = position.coords.longitude.toFixed(4);
                document.getElementById('gps-status').textContent = `OK (${currentLat}, ${currentLon})`;
                document.getElementById('rendez-vous').textContent = `${currentLat}, ${currentLon}`;
                document.getElementById('gps-status').classList.remove('warning');
            },
            (error) => {
                document.getElementById('gps-status').textContent = `Erreur GPS (Utilisation par défaut)`;
            },
            { enableHighAccuracy: false, timeout: 5000, maximumAge: 0 }
        );
    } else {
        document.getElementById('gps-status').textContent = 'N/A (Non supporté)';
    }
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
    
    const avgSpeedKPH = timeElapsed > 0 ? distanceTraveled / (timeElapsed / 3600) : 0;
    const E_KINETIC = 0.5 * MASS_KG * (currentSpeedMS * currentSpeedMS);
    
    const DISTANCE_SL = distanceTraveled * 1000 / C_LIGHT_MS; 
    const DISTANCE_AL = DISTANCE_SL / (3600 * 24 * 365.25); 
    
    // 2. Mise à jour du HTML
    document.getElementById('time-s').textContent = `${timeElapsed.toFixed(0)} s`;
    document.getElementById('vitesse-inst').textContent = `${V_KPH.toFixed(2)} km/h`;
    document.getElementById('vitesse-moy').textContent = `${avgSpeedKPH.toFixed(2)} km/h`;
    document.getElementById('vitesse-max').textContent = `${maxSpeedKPH.toFixed(2)} km/h`;
    document.getElementById('vitesse-ms').textContent = `${currentSpeedMS.toFixed(2)} m/s`;
    document.getElementById('vitesse-mms').textContent = `${V_MMS.toFixed(0)} mm/s`;

    // RELATIVITÉ
    document.getElementById('gamma-factor').textContent = LORENTZ_FACTOR.toFixed(4);
    document.getElementById('pourcent-lumiere').textContent = `${(V_LIGHT_RATIO * 100).toFixed(4)}%`;
    document.getElementById('pourcent-lumiere-precise').textContent = `${V_LIGHT_RATIO.toFixed(8)} c`;
    
    document.getElementById('distance-km').textContent = `${distanceTraveled.toFixed(2)} km`;
    document.getElementById('distance-m').textContent = `${(distanceTraveled * 1000).toFixed(0)} m`;
    document.getElementById('distance-mm').textContent = `${(distanceTraveled * 1000000).toFixed(0)} mm`;
    
    document.getElementById('distance-sl').textContent = `${DISTANCE_SL.toPrecision(4)} s lumière`;
    document.getElementById('distance-al').textContent = `${DISTANCE_AL.toPrecision(4)} al`;
    document.getElementById('energie-cinetique').textContent = `${E_KINETIC.toFixed(0)} J`;
    
    timeElapsed++; 
}

/** Parse un temps HH:MM:SS en secondes depuis minuit. */
function parseTime(timeStr) {
    const parts = timeStr.split(':');
    return parseInt(parts[0]) * 3600 + parseInt(parts[1]) * 60 + parseInt(parts[2]);
}

/** Met à jour le temps et les données célestes en utilisant l'heure système réelle. */
function updateCelestialData() {
    const now = new Date(); // Utilise l'heure système réelle
    
    // 1. Heure Atomique (UTC) et Date Locale
    document.getElementById('utc-time').textContent = now.toUTCString().split(' ')[4] + " UTC";
    document.getElementById('date').textContent = now.toLocaleDateString('fr-FR');
    
    // 2. Calcul Heure Solaire Moyenne (HSM) et Vraie (HSV)
    // L'heure locale est utilisée pour le calcul du temps solaire moyen (HSM)
    const hour = now.getHours();
    const minute = now.getMinutes();
    const second = now.getSeconds();

    const hsmTime = `${String(hour).padStart(2, '0')}:${String(minute).padStart(2, '0')}:${String(second).padStart(2, '0')}`;
    
    // HSV = HSM + Équation du Temps (EDT)
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


// --- CHRONOMÈTRE ET CONTRÔLES ---

/** Bascule le chronomètre (Démarrer/Arrêter). */
function toggleMovement(start) {
    if (start) {
        if (!lastAccelTime) lastAccelTime = performance.now();
        startTime = Date.now() - (timeElapsed * 1000); 
        
        // Un seul intervalle pour tout mettre à jour chaque seconde
        intervalId = setInterval(() => {
            updateCelestialData(); 
            updateNavigationData();
        }, 1000);
        
        document.getElementById('gps-status').textContent = 'Acquisition GPS...';
        document.getElementById('gps-status').classList.add('warning');
    } else {
        clearInterval(intervalId);
        intervalId = null;
        document.getElementById('gps-status').textContent = 'Arrêté';
        document.getElementById('gps-status').classList.remove('warning');
    }
    document.getElementById('start-btn').disabled = start;
    document.getElementById('stop-btn').disabled = !start;
}

/** Réinitialise toutes les données de navigation. */
function resetData() {
    toggleMovement(false);
    timeElapsed = 0;
    currentSpeedMS = 0;
    maxSpeedKPH = 0;
    distanceTraveled = 0;
    
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


// --- INITIALISATION AU CHARGEMENT DE LA PAGE ---
function initializeCockpit() {
    updateCelestialData(); // Mise à jour immédiate du temps et des astres

    getGeoLocation();
    updateBatteryStatus();
    initAccelerometer(); 
    
    // Timer pour la mise à jour constante du temps/astres
    setInterval(updateCelestialData, 1000);

    // Timer pour la mise à jour des capteurs non critiques (batterie)
    setInterval(updateBatteryStatus, 60000); 
}

window.onload = initializeCockpit;
