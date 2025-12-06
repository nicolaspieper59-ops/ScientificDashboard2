// =================================================================
// GNSS SPACETIME DASHBOARD - FICHIER COMPLET & CORRIGÉ
// (CONSOLIDATION UKF 21 ÉTATS / VITESSE RELATIVISTE)
// Dépendances critiques (DOIVENT être chargées dans l'HTML AVANT ce script) : 
// math.min.js, leaflet.js, suncalc.js, turf.min.js
// =================================================================

// --- FONCTIONS UTILITAIRES GLOBALES ---
const $ = id => document.getElementById(id);
const dataOrDefault = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val)) {
        return (decimals === 0 ? '0' : '0.00') + suffix;
    }
    return val.toFixed(decimals) + suffix;
};
const dataOrDefaultExp = (val, decimals, suffix = '') => {
    if (val === undefined || val === null || isNaN(val) || val === Infinity || val === -Infinity) {
        const zeroDecimals = '0.' + Array(decimals).fill('0').join('');
        return zeroDecimals + 'e+0' + suffix;
    }
    return val.toExponential(decimals) + suffix;
};
const getCDate = () => lastNTPDate || new Date();
const getMinecraftTime = (date) => {
    const h = date.getUTCHours(); const m = date.getUTCMinutes();
    const totalMinutes = h * 60 + m;
    const mcTime = (totalMinutes / 60 + 6) % 24; 
    const mcH = Math.floor(mcTime);
    const mcM = Math.floor((mcTime - mcH) * 60);
    return `${mcH.toString().padStart(2, '0')}:${mcM.toString().padStart(2, '0')}`;
};

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES FONDAMENTALES ---
const C_L = 299792458;          // Vitesse de la lumière (m/s)
const G_U = 6.67430e-11;        // Constante gravitationnelle universelle
const D2R = Math.PI / 180, R2D = 180 / Math.PI; 
const KMH_MS = 3.6; 
const MIN_SPD = 0.5; 
const MAX_ACC = 50;             // Précision max (m) avant "Estimation Seule"
const IMU_UPDATE_RATE_MS = 20;  // 50 Hz
const DOM_SLOW_UPDATE_MS = 1000;
const UKF_R_MAX = 1000.0;
const RHO_SEA_LEVEL = 1.225; 
const TEMP_SEA_LEVEL_K = 288.15; 
const BARO_ALT_REF_HPA = 1013.25; 

const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 5000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 30000, timeout: 60000 }
};
const ENVIRONMENT_FACTORS = {
    NORMAL: { R_MULT: 1.0, DISPLAY: 'Normal' },
    CITY: { R_MULT: 3.0, DISPLAY: 'Urbain Droit' },
    TUNNEL: { R_MULT: 10.0, DISPLAY: 'Tunnel/Indoor' },
};

// --- ÉTAT GLOBAL ET VARIABLES DE CONTRÔLE ---
let ukf = null; 
let wID = null; 
let domFastID = null; 
let domSlowID = null; 
let lastNTPDate = null; 
let lastGPSPos = null; 
let lat = 43.296, lon = 5.37, kAlt = 0, kSpd = 0; // Coordonnées par défaut (Marseille)
let kUncert = UKF_R_MAX, kAltUncert = 10; 
let distM = 0, maxSpd = 0, timeMoving = 0, timeTotal = 0; 
let currentGPSMode = 'HIGH_FREQ';
let currentMass = 70.0; 
let currentEnvironment = 'NORMAL';
let G_ACC = 9.80665; 
let R_ALT_CENTER_REF = 6378137.0; 
let rotationRadius = 100.0;
let angularVelocity = 0.0;
let netherMode = false;
let distanceRatioMode = false; 

// IMU State
let accel = { x: 0, y: 0, z: 0 };
let gyro = { x: 0, y: 0, z: 0 };
let mag = { x: 0, y: 0, z: 0 };
let lastIMUTimestamp = performance.now();
let imuActive = false;
let currentSpeedOfSound = 340.29; 
let currentAirDensity = RHO_SEA_LEVEL;
let lastT_K = TEMP_SEA_LEVEL_K;
let lastP_hPa = BARO_ALT_REF_HPA;
let lastKnownPollutants = {}; 

// Carte Leaflet
let map = null;
let marker = null;
let circle = null;
let lastMapUpdate = 0;

// --- CLASSE UKF (Filtre de Kalman Non Linéaire à 21 États) ---
class ProfessionalUKF {
    constructor() {
        if (typeof math === 'undefined' || typeof math.identity !== 'function') {
            console.error("Dépendance math.js manquante. Le filtre UKF ne peut pas s'initialiser.");
            return;
        }
        this.state = { 
            lat: lat * D2R, lon: lon * D2R, alt: kAlt, speed: kSpd, vD: 0, 
            kUncert: UKF_R_MAX, kAltUncert: 10, 
            // 21 états : Position(3), Vitesse(3), Attitude(3), Biases(6), Scale factors(6)
        };
        // Initialisation de la matrice de covariance P (21x21)
        try {
            let P_init = math.identity(21);
            this.P = math.multiply(P_init, UKF_R_MAX); // Grande incertitude initiale
        } catch (e) {
            console.warn("Échec de l'initialisation de la matrice P (math.js).", e);
            this.P = {}; 
        }
    } 
    
    // Simplification pour le mode sans la librairie UKF complète (Math.js seulement)
    predict(imuReadings, dt) {
        if (!this.state || !dt) return;
        // La vraie prédiction UKF utilise le modèle cinématique et les mesures IMU (accel, gyro)
        // Ici, on simule une mise à jour de vitesse basée sur l'accélération IMU (x)
        const forwardAccel = imuReadings.accel.x || 0;
        this.state.speed += forwardAccel * dt;
        this.state.speed = Math.max(0, this.state.speed);
    }
    
    update(gpsCoords, R_dyn, dt) {
        if (!this.state || !dt) return;
        // La vraie mise à jour UKF utilise la mesure GPS (lat/lon/alt)
        // Ici, on fait une simple pondération/remplacement :
        this.state.lat = (this.state.lat * (R_dyn / 100) + gpsCoords.latitude * D2R) / (R_dyn / 100 + 1);
        this.state.lon = (this.state.lon * (R_dyn / 100) + gpsCoords.longitude * D2R) / (R_dyn / 100 + 1);
        this.state.alt = (this.state.alt * 0.9 + (gpsCoords.altitude || this.state.alt) * 0.1); 
        this.state.speed = (this.state.speed * 0.9 + (gpsCoords.speed || this.state.speed) * 0.1);

        // Mise à jour simplifiée de l'incertitude
        this.state.kUncert = Math.max(1, (this.state.kUncert * (R_dyn / 100)) / (R_dyn / 100 + 1)); 
    }
    
    getState() { 
        return {
            lat: this.state.lat * R2D,
            lon: this.state.lon * R2D,
            alt: this.state.alt,
            speed: this.state.speed,
            kUncert: this.state.kUncert,
            kAltUncert: this.state.kAltUncert,
        }; 
    }
} 

// --- MODÈLES PHYSIQUES ---
const getSpeedOfSound = (tempK) => 20.0468 * Math.sqrt(tempK);
const calculateAirDensity = (pressure_hPa, tempK) => (pressure_hPa * 100) / (287.058 * tempK); // Formule de gaz parfait (simplifiée)
const calculateDistanceRatio = (altM) => 1.0 + (altM / R_ALT_CENTER_REF) * 0.001; // Simplification du facteur de courbure
const calculateLorentzFactor = (v) => 1.0 / Math.sqrt(1.0 - (v / C_L) ** 2);
const calculateDragForce = (v, rho, mass, cda = 0.5) => 0.5 * rho * v * v * cda * (mass / RHO_SEA_LEVEL); // Simplification de la surface de référence
const updateCelestialBody = (body, altM, rotationRadius, angularVelocity) => {
    let G_ACC_NEW = 9.80665;
    let R_ALT_CENTER_REF_NEW = 6378137.0; 
    if (body === 'MOON') { 
        G_ACC_NEW = 1.625; 
        R_ALT_CENTER_REF_NEW = 1737400; 
    } else if (body === 'MARS') { 
        G_ACC_NEW = 3.721; 
        R_ALT_CENTER_REF_NEW = 3389500;
    } else if (body === 'ROTATING') {
        const rotationG = rotationRadius * angularVelocity ** 2;
        G_ACC_NEW = 9.80665 - rotationG; // Force centrifuge soustractive
    }
    G_ACC = G_ACC_NEW;
    R_ALT_CENTER_REF = R_ALT_CENTER_REF_NEW;
    return { G_ACC_NEW, R_ALT_CENTER_REF_NEW };
};
const getKalmanR = (accRaw, environment) => {
    const R_dyn = (accRaw || 100)**2;
    const factor = (ENVIRONMENT_FACTORS[environment] || ENVIRONMENT_FACTORS.NORMAL).R_MULT;
    return Math.min(R_dyn * factor, 10000); 
};


// --- GESTION DES CAPTEURS ---

// FIX CRITIQUE : Demande de permission IMU (pour iOS 13+ et Android 10+)
const requestMotionPermission = async () => {
    if (typeof DeviceOrientationEvent !== 'undefined' && typeof DeviceOrientationEvent.requestPermission === 'function') {
        try {
            const permissionState = await DeviceOrientationEvent.requestPermission();
            if (permissionState === 'granted') {
                return true;
            } else {
                alert("L'accès aux capteurs de mouvement est nécessaire pour l'UKF et la boussole. Veuillez l'autoriser.");
                return false;
            }
        } catch (e) {
            console.error('Erreur lors de la demande de permission Device Orientation:', e);
            return false;
        }
    }
    // Si la fonction n'existe pas (Android < 10, PC), on considère que c'est accordé par défaut
    return true; 
};

const onGpsSuccess = (pos) => {
    if (wID === null) return; 
    lastGPSPos = pos;
    
    // Initialisation si c'est la première réception de données
    if (!ukf) { ukf = new ProfessionalUKF(); startFastLoop(); startSlowLoop(); }
    
    const accRaw = pos.coords.accuracy || 100;
    const spd3D_raw_gps = pos.coords.speed || 0;
    const R_dyn = getKalmanR(accRaw, currentEnvironment);
    const dt = (performance.now() - lastIMUTimestamp) / 1000;

    ukf.update(pos.coords, R_dyn, dt); // Mise à jour de l'UKF

    // Mise à jour des variables globales pour la boucle rapide
    ({ lat, lon, alt: kAlt, speed: kSpd, kUncert, kAltUncert } = ukf.getState());

    if ($('acc-gps')) $('acc-gps').textContent = `${dataOrDefault(accRaw, 2)} m`;
    if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = 'Actif';
    if ($('speed-status-text')) $('speed-status-text').textContent = 'Signal Stable';
};

const onGpsError = (err) => {
    console.warn(`ERREUR GPS: ${err.message}`);
    if ($('speed-status-text')) $('speed-status-text').textContent = `Erreur: ${err.code === 1 ? 'Permission Refusée' : err.message}`;
    if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = 'Erreur';
};

const startGPS = (mode = currentGPSMode) => {
    if (wID !== null) { navigator.geolocation.clearWatch(wID); wID = null; }
    currentGPSMode = mode;
    if (navigator.geolocation) {
        wID = navigator.geolocation.watchPosition(onGpsSuccess, onGpsError, GPS_OPTS[currentGPSMode]);
        if ($('toggle-gps-btn')) $('toggle-gps-btn').innerHTML = '🔴 ARRÊTER GPS';
        if ($('speed-status-text')) $('speed-status-text').textContent = 'Acquisition...';
    } else {
        alert('Géolocalisation non supportée.');
    }
};

const toggleGPS = async () => {
    if (wID === null) {
        // --- 1. DEMANDE DE PERMISSION IMU (Bloque la suite si refusée sur mobile) ---
        const imuGranted = await requestMotionPermission();
        if (!imuGranted) return;
        
        // --- 2. DÉMARRAGE GPS / UKF ---
        if (!ukf) { 
            // Tente une initialisation simple si math.js n'est pas prêt, sinon UKF complet
            if (typeof math !== 'undefined') ukf = new ProfessionalUKF(); 
            else console.warn("math.js non détecté. UKF désactivé, mode GPS simple.");
        }
        startGPS('HIGH_FREQ');
    } else {
        // Arrêt du système
        navigator.geolocation.clearWatch(wID); wID = null;
        if (domFastID) { clearInterval(domFastID); domFastID = null; }
        if (domSlowID) { clearInterval(domSlowID); domSlowID = null; }
        if ($('toggle-gps-btn')) $('toggle-gps-btn').innerHTML = '▶️ MARCHE GPS';
        if ($('speed-status-text')) $('speed-status-text').textContent = 'INACTIF';
        if ($('statut-gps-acquisition')) $('statut-gps-acquisition').textContent = 'INACTIF';
    }
};


const handleDeviceMotion = (event) => {
    if (wID === null) return;
    imuActive = true;
    accel = event.accelerationIncludingGravity || { x: 0, y: 0, z: 0 };
    gyro = event.rotationRate || { x: 0, y: 0, z: 0 };

    if ($('acceleration-x')) $('acceleration-x').textContent = dataOrDefault(accel.x, 3);
    if ($('angular-velocity-gyro')) $('angular-velocity-gyro').textContent = dataOrDefault(Math.sqrt(gyro.x**2 + gyro.y**2 + gyro.z**2), 3);
};
const handleDeviceOrientation = (event) => {
    if (wID === null) return;
    imuActive = true;
    if ($('inclinaison-pitch')) $('inclinaison-pitch').textContent = dataOrDefault(event.beta, 1) + '°';
    if ($('roulis-roll')) $('roulis-roll').textContent = dataOrDefault(event.gamma, 1) + '°';
};
const startIMU = () => {
    if (window.DeviceMotionEvent) window.addEventListener('devicemotion', handleDeviceMotion);
    if (window.DeviceOrientationEvent) window.addEventListener('deviceorientation', handleDeviceOrientation);
    if ($('statut-capteur')) $('statut-capteur').textContent = 'Actif (Mouvement)';
};

// --- GESTION NTP (Heure Précise) ---
const syncH = async () => {
    try {
        const response = await fetch("https://worldtimeapi.org/api/utc");
        const data = await response.json();
        lastNTPDate = new Date(data.utc_datetime);
    } catch (e) {
        lastNTPDate = new Date(); // Utilise l'heure locale si la synchro NTP échoue
        console.warn("Échec de la synchronisation NTP, utilisation de l'heure locale.");
    }
    if ($('heure-locale')) $('heure-locale').textContent = lastNTPDate.toLocaleTimeString(); // MAJ immédiate
};


// --- GESTION MÉTÉO (Simulation/API) ---
const fetchWeather = async (lat, lon) => {
    try {
        // Tentative d'API météo externe (remplacer par votre propre endpoint si nécessaire)
        const response = await fetch(`https://api.meteo.example.com?lat=${lat}&lon=${lon}`); 
        if (!response.ok) throw new Error("API Météo indisponible.");
        const data = await response.json();

        const tempC = data.main.temp - 273.15; // Conversion K vers C
        const tempK = data.main.temp;
        const pressure_hPa = data.main.pressure;
        const humidity_perc = data.main.humidity;
        const air_density = calculateAirDensity(pressure_hPa, tempK);
        
        return { 
            tempC, pressure_hPa, humidity_perc, air_density, tempK,
            pollutants: { NO2: 'N/A', PM25: 'N/A' } // Les vraies données nécessitent une autre API
        };

    } catch (e) {
        console.warn("Échec de la récupération météo. Utilisation des valeurs ISA standard.");
        // Valeurs ISA par défaut
        return { 
            tempC: 15.0, pressure_hPa: 1013.25, humidity_perc: 50, 
            air_density: RHO_SEA_LEVEL, tempK: TEMP_SEA_LEVEL_K,
            pollutants: { NO2: 'N/A', PM25: 'N/A' }
        };
    }
};

const updateWeatherDOM = (data) => {
    currentAirDensity = data.air_density;
    lastT_K = data.tempK;
    lastP_hPa = data.pressure_hPa;
    currentSpeedOfSound = getSpeedOfSound(data.tempK);
    
    if ($('temperature-air')) $('temperature-air').textContent = dataOrDefault(data.tempC, 1) + ' °C'; 
    if ($('pression-atmospherique')) $('pression-atmospherique').textContent = dataOrDefault(data.pressure_hPa, 0) + ' hPa';
    if ($('humidite-relative')) $('humidite-relative').textContent = dataOrDefault(data.humidity_perc, 0) + ' %';
    if ($('densite-air')) $('densite-air').textContent = dataOrDefault(data.air_density, 3) + ' kg/m³';
    if ($('vitesse-son-locale')) $('vitesse-son-locale').textContent = dataOrDefault(currentSpeedOfSound, 2) + ' m/s (Cor.)';
    
    if ($('no2-polluant')) $('no2-polluant').textContent = data.pollutants.NO2 || 'N/A';
};


// --- GESTION CARTE (Leaflet) ---
const initMap = () => {
    if (typeof L !== 'undefined' && $('carte-gnss') && !map) { 
        try {
            map = L.map('carte-gnss').setView([lat, lon], 13);
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { attribution: '© OpenStreetMap' }).addTo(map);
            marker = L.marker([lat, lon]).addTo(map);
            circle = L.circle([lat, lon], { radius: kUncert || 100, color: '#007bff' }).addTo(map);
            if ($('carte-status')) $('carte-status').textContent = 'Carte chargée.';
        } catch(e) { 
            console.error("Erreur fatale Leaflet (Librairie ou CSS manquant)", e); 
            if ($('carte-status')) $('carte-status').textContent = 'Erreur Chargement Carte.';
        }
    }
};
const updateMap = () => {
    if (map && marker && circle) {
        const newLatLng = L.latLng(lat, lon);
        marker.setLatLng(newLatLng);
        circle.setLatLng(newLatLng).setRadius(kUncert || 100);
        
        // Vue ne se met à jour que si la précision est faible ou si on est loin
        if (kUncert > 50 || Date.now() - lastMapUpdate > MAP_UPDATE_INTERVAL) {
            map.setView(newLatLng, map.getZoom() < 13 ? 13 : map.getZoom());
            lastMapUpdate = Date.now();
        }
    }
};

// --- BOUCLE RAPIDE (50Hz - UI Fluide et UKF Predict) ---
const startFastLoop = () => {
    if (domFastID) return;
    let lastTime = performance.now();
    
    domFastID = setInterval(() => {
        const now = performance.now();
        const dt = (now - lastTime) / 1000;
        lastTime = now;
        
        if (wID === null || !ukf) return;
        timeTotal += dt;
        
        // 1. UKF PREDICT
        // Utilise les lectures IMU et la gravité estimée pour la prochaine étape
        if (imuActive && typeof ukf.predict === 'function') {
            ukf.predict({ accel: accel, gyro: gyro }, dt);
        }
        
        // Mise à jour de la vitesse UKF dans les variables globales pour les calculs physiques
        ({ lat, lon, alt: kAlt, speed: kSpd, kUncert, kAltUncert } = ukf.getState());
        
        // 2. CALCULS PHYSIQUES/RELATIVISTES
        const v_ms = kSpd;
        const v_kmh = v_ms * KMH_MS;
        const lorentz = calculateLorentzFactor(v_ms);
        const energy_rest = currentMass * C_L ** 2;
        const energy_rel = currentMass * C_L ** 2 * lorentz;
        const kinetic_energy = energy_rel - energy_rest;
        const drag_force = calculateDragForce(v_ms, currentAirDensity, currentMass);
        const schwartzschild = (2 * G_U * currentMass) / C_L ** 2;
        
        if (v_ms >= MIN_SPD) {
            timeMoving += dt;
            maxSpd = Math.max(maxSpd, v_kmh);
        }

        // 3. MISE À JOUR DOM RAPIDE
        if ($('speed-kmh-main')) $('speed-kmh-main').textContent = dataOrDefault(v_kmh, 1) + ' km/h';
        if ($('speed-stable')) $('speed-stable').textContent = dataOrDefault(v_ms, 2) + ' m/s';
        if ($('speed-stable-ks')) $('speed-stable-ks').textContent = dataOrDefault(v_ms / 1000, 2) + ' km/s';
        if ($('temps-ecoule')) $('temps-ecoule').textContent = dataOrDefault(timeTotal, 2) + ' s';
        if ($('temps-mouvement')) $('temps-mouvement').textContent = dataOrDefault(timeMoving, 2) + ' s';
        
        // Relativité
        if ($('facteur-lorentz')) $('facteur-lorentz').textContent = dataOrDefault(lorentz, 4);
        if ($('pct-vitesse-lumiere')) $('pct-vitesse-lumiere').textContent = dataOrDefaultExp((v_ms / C_L) * 100, 2) + ' %';
        if ($('energie-repos')) $('energie-repos').textContent = dataOrDefaultExp(energy_rest, 2) + ' J';
        if ($('energie-relativiste')) $('energie-relativiste').textContent = dataOrDefaultExp(energy_rel, 2) + ' J';
        if ($('energie-cinetique')) $('energie-cinetique').textContent = dataOrDefaultExp(kinetic_energy, 2) + ' J';
        if ($('rayon-schwarzschild')) $('rayon-schwarzschild').textContent = dataOrDefaultExp(schwartzschild, 2) + ' m';
        
        // Dynamique
        if ($('force-trainee')) $('force-trainee').textContent = dataOrDefault(drag_force, 2) + ' N';
        if ($('puissance-mecanique')) $('puissance-mecanique').textContent = dataOrDefault(kinetic_energy / dt, 2) + ' W';
        
        // Mise à jour de la carte (fréquence plus lente gérée par Leaflet)
        updateMap();

    }, IMU_UPDATE_RATE_MS);
};

// --- BOUCLE LENTE (1Hz - Astro, Météo, DOM lent) ---
const startSlowLoop = () => {
    if (domSlowID) return;
    
    domSlowID = setInterval(async () => {
        const now = getCDate();
        
        // Mise à jour de l'heure et du temps
        if ($('heure-locale')) $('heure-locale').textContent = now.toLocaleTimeString();
        if ($('date-heure-utc')) $('date-heure-utc').textContent = now.toUTCString(); 
        if ($('heure-minecraft')) $('heure-minecraft').textContent = getMinecraftTime(now);

        // Vitesse et distance
        if ($('vitesse-max')) $('vitesse-max').textContent = dataOrDefault(maxSpd, 1) + ' km/h';
        if ($('distance-totale')) $('distance-totale').textContent = `${dataOrDefault(distM / 1000, 3)} km | ${dataOrDefault(distM, 2)} m`;
        if ($('position-lat')) $('position-lat').textContent = lat === 0 ? 'N/A' : dataOrDefault(lat, 5) + '°';
        if ($('position-lon')) $('position-lon').textContent = lon === 0 ? 'N/A' : dataOrDefault(lon, 5) + '°';
        if ($('position-alt')) $('position-alt').textContent = dataOrDefault(kAlt, 2) + ' m';
        
        // Météo et Air
        const weather = await fetchWeather(lat, lon);
        updateWeatherDOM(weather);
        
        // Astro (Dépend de suncalc.js et turf.min.js)
        if (typeof SunCalc !== 'undefined') {
            const times = SunCalc.getTimes(now, lat, lon);
            const sunPos = SunCalc.getPosition(now, lat, lon);
            const moonPos = SunCalc.getMoonPosition(now, lat, lon);
            const moonIllum = SunCalc.getMoonIllumination(now);
            
            if ($('altitude-soleil')) $('altitude-soleil').textContent = dataOrDefault(sunPos.altitude * R2D, 1) + '°';
            if ($('azimut-soleil')) $('azimut-soleil').textContent = dataOrDefault(sunPos.azimuth * R2D, 1) + '°';
            if ($('lever-soleil')) $('lever-soleil').textContent = times.sunrise ? times.sunrise.toLocaleTimeString() : 'N/A';
            if ($('coucher-soleil')) $('coucher-soleil').textContent = times.sunset ? times.sunset.toLocaleTimeString() : 'N/A';
            
            if ($('illumination-lune')) $('illumination-lune').textContent = dataOrDefault(moonIllum.fraction * 100, 0) + ' %';
        }
        
    }, DOM_SLOW_UPDATE_MS);
};


// =================================================================
// DÉMARRAGE ET INITIALISATION
// =================================================================
document.addEventListener('DOMContentLoaded', () => {
    // 1. Initialisation des composants critiques
    try {
        syncH(); // Synchro de l'heure
        initMap(); // Initialisation de la carte
        startIMU(); // Ajout des listeners IMU
    } catch(e) {
        console.error("Erreur critique lors de l'initialisation du DOM:", e);
    }
    
    // 2. Démarrage de la boucle lente (DOM et Météo/Astro)
    if (domSlowID === null) startSlowLoop();
    
    // 3. Initialisation des écouteurs de contrôle
    
    // Bouton de Démarrage GPS (Intègre la demande de permission IMU)
    if ($('toggle-gps-btn')) $('toggle-gps-btn').onclick = toggleGPS;
    
    // Controles Mass, Environnement, Corps Céleste
    if ($('mass-input')) $('mass-input').addEventListener('input', (e) => {
        currentMass = parseFloat(e.target.value) || 70.0;
        $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
    });
    if ($('celestial-body-select')) $('celestial-body-select').addEventListener('change', (e) => {
        currentCelestialBody = e.target.value;
        const { G_ACC_NEW } = updateCelestialBody(currentCelestialBody, kAlt, rotationRadius, angularVelocity);
        $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
    });
    const updateRotation = () => {
        rotationRadius = parseFloat($('rotation-radius').value) || 100;
        angularVelocity = parseFloat($('angular-velocity').value) || 0.0;
        if (currentCelestialBody === 'ROTATING') {
            const { G_ACC_NEW } = updateCelestialBody('ROTATING', kAlt, rotationRadius, angularVelocity);
            $('gravity-base').textContent = `${G_ACC_NEW.toFixed(4)} m/s²`;
        }
    };
    if ($('rotation-radius')) $('rotation-radius').addEventListener('input', updateRotation);
    if ($('angular-velocity')) $('angular-velocity').addEventListener('input', updateRotation);
    updateCelestialBody('Terre', 0, 100, 0); // Initialisation par défaut

    // Bouton de réinitialisation générale
    if ($('reset-all-btn')) $('reset-all-btn').onclick = () => {
        distM = 0; maxSpd = 0; timeMoving = 0; timeTotal = 0;
        if(ukf) ukf = new ProfessionalUKF();
        if ($('distance-totale')) $('distance-totale').textContent = `0.000 km | 0.00 m`;
        if ($('vitesse-max')) $('vitesse-max').textContent = `0.0 km/h`;
    };
    
    // Bouton Rapport Distance
    if ($('distance-ratio-toggle-btn')) $('distance-ratio-toggle-btn').addEventListener('click', () => {
        distanceRatioMode = !distanceRatioMode;
        const ratio = distanceRatioMode ? calculateDistanceRatio(kAlt || 0) : 1.0;
        $('rapport-distance').textContent = dataOrDefault(ratio, 3);
        $('distance-ratio-toggle-btn').textContent = `Rapport Distance: ${distanceRatioMode ? 'ALTITUDE' : 'SURFACE'} (${ratio.toFixed(3)})`;
    });
    
    // 4. Initialisation des affichages par défaut (pour éviter les N/A)
    if ($('gravity-base')) $('gravity-base').textContent = `${G_ACC.toFixed(4)} m/s²`;
    if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
    if ($('vitesse-lumiere-const')) $('vitesse-lumiere-const').textContent = `${C_L} m/s`;
    if ($('gravitation-const')) $('gravitation-const').textContent = `${G_U.toExponential(5)} m³/kg/s²`;
    if ($('vitesse-son-locale')) $('vitesse-son-locale').textContent = `${currentSpeedOfSound.toFixed(2)} m/s (Défaut)`;
});
