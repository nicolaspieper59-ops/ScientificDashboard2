// =================================================================
// FICHIER JS PARTIE 1/2 : gnss-dashboard-part1.js
// Contient : Variables globales, Constantes, Fonctions de support, Initialisation EKF/IMU, Fonctions de Démarrage/Contrôle
// =================================================================

// --- FONCTIONS DE SUPPORT GLOBALES ---
const $ = (id) => document.getElementById(id); 
const D2R = Math.PI / 180, R2D = 180 / Math.PI;

// --- CONSTANTES GLOBALES ET INITIALISATION ---
const C_L = 299792458;      // Vitesse de la Lumière (m/s)
const R_E = 6371000;        // Rayon de la Terre (m)
const KMH_MS = 3.6;         // Facteur de conversion (m/s vers km/h)
const C_S = 343;            // Vitesse du Son (m/s)
const G_ACC = 9.80665;      // Accélération de la gravité (m/s²)
const DOM_SLOW_UPDATE_MS = 1000; // Fréquence de rafraîchissement DOM lent (1s)
const dayMs = 86400000;     // 24 heures en millisecondes
const R_MAX = 1000.0;       // Précision max par défaut

// --- VARIABLES D'ÉTAT ---
window.lPos = null;         // Dernière position GPS
window.lVitesse = 0;        
window.lTime = 0;           
window.isMoving = false;    
window.distanceTotal = 0;   
window.speedMax = 0;        
window.timeMoving = 0;      
window.gpsActive = false;   
window.imuActive = false;   
window.domID = null;        
window.startTime = Date.now(); 

// VARIABLES EKF (Filtre de Kalman Étendu)
window.ekfState = { 
    x: 0, 
    y: 0, 
    vx: 0, 
    vy: 0, 
    ax: 0, 
    ay: 0 
};
window.ekfCov = [
    [100, 0, 0, 0, 0, 0], 
    [0, 100, 0, 0, 0, 0],
    [0, 0, 10, 0, 0, 0],
    [0, 0, 0, 10, 0, 0],
    [0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 1]
];
window.R_BASE = 0.5; 
window.Q_BASE = 0.01; 
window.gpsOverride = 0.0; 

// VARIABLES IMU
window.imuAccel = { x: 0, y: 0, z: 0 };
window.imuData = null; 

// VARIABLES DE CARTE
window.map = null;
window.userMarker = null;

// VARIABLES TEMPS TST
window.mcTime = null; 

// VARIABLES CAPTEURS/APIs EXTERNES (Placeholder pour l'intégration future des API)
window.externalData = {
    temp_air: null,         
    pressure: null,         
    humidity: null,         
    magnetic_field: null,   
    solar_radiation: null,  
    wind_speed: null,       
    co2_level: null,
    ph_level: null,
    noise_level: null
};

// --- FONCTIONS DE GESTION GPS, IMU & EKF ---

function updateEKF(dt, accX, accY, gpsMeasurement) {
    // *** LOGIQUE EKF PLACEHOLDER ***
    // Mise à jour simplifiée de l'état pour éviter le crash.
    window.ekfState.vx += accX * dt;
    window.ekfState.vy += accY * dt;

    return { 
        vx: window.ekfState.vx, 
        vy: window.ekfState.vy,
        uncertainty: window.ekfCov[2][2] + window.ekfCov[3][3] 
    }; 
}

function gpsError(error) {
    console.error("Erreur GPS:", error.code, error.message);
}

function startGPS() {
    if ("geolocation" in navigator) {
        window.gpsActive = true;
        navigator.geolocation.watchPosition(updateDisp, gpsError, {
            enableHighAccuracy: true,
            maximumAge: 500,
            timeout: 5000
        });
        if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = "⏸️ PAUSE GPS";
    } else {
        alert("La géolocalisation n'est pas supportée par ce navigateur.");
    }
}

function handleIMU(event) {
    window.imuAccel = event.accelerationIncludingGravity;
    window.imuData = event;
}

function startIMU() {
    if (window.DeviceMotionEvent) {
        window.imuActive = true;
        window.addEventListener('devicemotion', handleIMU, true);
    } else {
        console.warn("DeviceMotionEvent non supporté ou non autorisé.");
    }
}

// --- FONCTIONS DE CONTRÔLE (Réinitialisation) ---

function resetDistance() {
    window.distanceTotal = 0;
    window.timeMoving = 0;
    // Mettre à jour le DOM dans part2.js
}

function resetSpeedMax() {
    window.speedMax = 0;
    // Mettre à jour le DOM dans part2.js
}

function resetAll() {
    resetDistance();
    resetSpeedMax();
    // Réinitialiser l'EKF et d'autres variables d'état
    window.startTime = Date.now();
    window.lPos = null;
    window.lVitesse = 0;
    window.ekfState = { x: 0, y: 0, vx: 0, vy: 0, ax: 0, ay: 0 };
}
// =================================================================
// FICHIER JS PARTIE 2/2 : gnss-dashboard-part2.js
// Contient : Fonctions de support, Logique Astro/TST/Montre, Carte Leaflet, Calculs P/C/SVT, Boucles DOM
// =================================================================

// --- FONCTIONS DE SUPPORT ---

function msToTime(ms) {
    const hours = Math.floor(ms / (1000 * 60 * 60));
    const minutes = Math.floor((ms % (1000 * 60 * 60)) / (1000 * 60));
    const seconds = Math.floor((ms % (1000 * 60)) / 1000);
    return `${String(hours).padStart(2, '0')}:${String(minutes).padStart(2, '0')}:${String(seconds).padStart(2, '0')}`;
}

function getCDate() {
    return new Date(); 
}

// --- FONCTIONS ASTRO (TST/EOT) ---

function getSolarTime(date, lon) {
    const msSinceMidnightUTC = (date.getUTCHours() * 3600 + date.getUTCMinutes() * 60 + date.getUTCMilliseconds()) * 1000;
    
    // Heure Solaire Moyenne Locale (MST)
    const mst_offset_ms = lon * dayMs / 360; 
    const mst_ms = (msSinceMidnightUTC + mst_offset_ms + dayMs) % dayMs;

    // Équation du Temps (EOT) - Placeholder de 0.0, doit être calculée précisément
    const eot_min = 0.0; 
    const eot_ms = eot_min * 60000;
    
    // TST = MST + EOT
    const tst_ms = (mst_ms + eot_ms + dayMs) % dayMs; 
    
    return { 
        TST: msToTime(tst_ms), 
        TST_MS: tst_ms, 
        MST: msToTime(mst_ms), 
        EOT: eot_min.toFixed(3),
    };
}

// --- SIMULATION D'APPEL API EXTERNE (A remplacer par un vrai fetch() en production) ---

function fetchExternalData(lat, lon) {
    setTimeout(() => {
        // Météo et Géomagnétisme (Simulés, à remplacer par des vrais appels API)
        window.externalData.temp_air = (Math.random() * 20 + 5).toFixed(1) + " °C"; 
        window.externalData.pressure = (Math.random() * 20 + 1000).toFixed(0) + " hPa"; 
        window.externalData.humidity = (Math.random() * 40 + 40).toFixed(0) + " %"; 
        window.externalData.wind_speed = (Math.random() * 10).toFixed(1); 
        window.externalData.magnetic_field = (Math.random() * 10 + 40).toFixed(1) + " μT"; 
        
        // Radiation Solaire (Calculé en fonction de l'élévation simulée)
        const sunPos = window.SunCalc.getPosition(getCDate(), lat, lon);
        let rad = sunPos.altitude > 0 ? (Math.sin(sunPos.altitude) * 1000) : 0;
        window.externalData.solar_radiation = rad.toFixed(0) + " W/m²";
    }, 500); 
}

// --- FONCTION DE MISE À JOUR PHYSIQUE/CHIMIE/SVT (Utilise les données des capteurs/APIs) ---

function updatePhysicsAndChemistry() {
    const mass = parseFloat($('mass-display').textContent) || 70.0;
    const vx = window.ekfState.vx; 
    const vy = window.ekfState.vy;
    const speed_stable_ms = Math.sqrt(vx * vx + vy * vy);
    const accel_long = window.ekfState.ax; 
    
    // 1. Grandeurs Physiques
    const kinetic_energy = 0.5 * mass * speed_stable_ms * speed_stable_ms;
    if ($('kinetic-energy')) $('kinetic-energy').textContent = kinetic_energy.toFixed(2) + " J";

    const mechanical_power = mass * accel_long * speed_stable_ms;
    if ($('mechanical-power')) $('mechanical-power').textContent = mechanical_power.toFixed(2) + " W";

    const omega = 7.2921e-5; 
    const coriolis_force_mag = 2 * mass * omega * speed_stable_ms; 
    if ($('coriolis-force')) $('coriolis-force').textContent = coriolis_force_mag.toExponential(2) + " N";

    if ($('magnetic-field')) $('magnetic-field').textContent = window.externalData.magnetic_field || "N/A (API)";
    
    // Accélération Verticale (IMU - MESURE RÉELLE)
    if (window.imuAccel.z) {
        const accel_vertical_real = window.imuAccel.z - G_ACC; 
        if ($('accel-vertical-imu')) $('accel-vertical-imu').textContent = accel_vertical_real.toFixed(3) + " m/s²";
        if ($('force-g-vertical')) $('force-g-vertical').textContent = (accel_vertical_real / G_ACC).toFixed(2) + " G";
    }

    // 2. Grandeurs Chimiques & Météo
    if ($('temp-air')) $('temp-air').textContent = window.externalData.temp_air || "N/A (API)";
    if ($('pressure')) $('pressure').textContent = window.externalData.pressure || "N/A (API)";
    if ($('humidity')) $('humidity').textContent = window.externalData.humidity || "N/A (API)";
    
    // Densité de l'Air (Calcul basé sur l'API)
    let air_density = "N/A";
    const temp_match = window.externalData.temp_air ? window.externalData.temp_air.match(/([\d\.]+)/) : null;
    const pressure_match = window.externalData.pressure ? window.externalData.pressure.match(/([\d\.]+)/) : null;
    
    if (temp_match && pressure_match) {
        const temp_c = parseFloat(temp_match[1]);
        const temp_k = temp_c + 273.15; 
        const pressure_pa = parseFloat(pressure_match[1]) * 100; // hPa vers Pa
        const R_air = 287.05; 
        air_density = (pressure_pa / (R_air * temp_k)).toFixed(3) + " kg/m³";
    }
    if ($('air-density')) $('air-density').textContent = air_density;

    // Placeholders (Capteurs/API spécialisés requis)
    if ($('o2-level')) $('o2-level').textContent = "20.9 % vol (Capteur requis)";
    if ($('co2-level')) $('co2-level').textContent = window.externalData.co2_level || "N/A (Capteur/API)";
    if ($('ph-level')) $('ph-level').textContent = window.externalData.ph_level || "N/A (Capteur)";
    if ($('ozone-conc')) $('ozone-conc').textContent = "N/A (Capteur/API)";

    // 3. Grandeurs SVT
    if ($('solar-radiation')) $('solar-radiation').textContent = window.externalData.solar_radiation || "N/A (API)";
    if ($('wind-speed-ms')) $('wind-speed-ms').textContent = (window.externalData.wind_speed ? window.externalData.wind_speed + " m/s" : "N/A (API)");
    
    if ($('noise-level')) $('noise-level').textContent = window.externalData.noise_level || "N/A (Capteur)";
    if ($('soil-type')) $('soil-type').textContent = "N/A (API Géo/Capteur)";
    if ($('ndvi-index')) $('ndvi-index').textContent = "N/A (API Satellite)";
}


// --- FONCTIONS MONTRE TST/ASTRO ---

function initSolarClock() {
    const container = $('minecraft-clock');
    if (!container) return;

    const canvas = document.createElement('canvas');
    canvas.id = 'mc-clock-canvas'; 
    canvas.width = 90;
    canvas.height = 90;
    container.appendChild(canvas);
}

function drawSolarClock(sunPos) {
    const canvas = $('mc-clock-canvas');
    if (!canvas || !sunPos || window.mcTime === null) return; 
    
    const ctx = canvas.getContext('2d');
    const center = canvas.width / 2;
    const radius = center * 0.9;
    
    const sunAzimuth = sunPos.azimuth; 
    const sunElevation = sunPos.altitude; 
    
    const rotationAngle = sunAzimuth + Math.PI / 2; 
    const elevationFraction = Math.abs(sunElevation) / (Math.PI / 2); 
    const r_sun = radius * elevationFraction; 

    // Dessin du Cadran et des Astres (Logique Jour/Nuit basée sur l'élévation réelle)
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.beginPath();
    ctx.arc(center, center, radius, 0, 2 * Math.PI);
    ctx.fillStyle = '#C89A4D'; 
    ctx.fill();

    let skyColor, sunColor;
    if (sunElevation > D2R * 10) { 
        skyColor = '#4C7AD9'; 
        sunColor = 'yellow';
    } else if (sunElevation > D2R * -5) { 
        skyColor = '#E6994C'; 
        sunColor = 'orange';
    } else { 
        skyColor = '#212A3E'; 
        sunColor = 'darkorange';
    }
    
    ctx.beginPath();
    ctx.arc(center, center, radius, rotationAngle - Math.PI/2, rotationAngle + Math.PI/2);
    ctx.lineTo(center, center);
    ctx.fillStyle = skyColor; 
    ctx.fill();
    
    ctx.beginPath();
    ctx.arc(center, center, radius, rotationAngle + Math.PI/2, rotationAngle - Math.PI/2 + 2 * Math.PI);
    ctx.lineTo(center, center);
    ctx.fillStyle = '#101828'; 
    ctx.fill();

    const x_sun = center + r_sun * Math.cos(rotationAngle);
    const y_sun = center + r_sun * Math.sin(rotationAngle);

    ctx.beginPath();
    ctx.arc(x_sun, y_sun, center * 0.15, 0, 2 * Math.PI);
    ctx.fillStyle = sunColor; 
    ctx.fill();

    ctx.beginPath();
    ctx.arc(center, center, center * 0.1, 0, 2 * Math.PI);
    ctx.fillStyle = '#C89A4D'; 
    ctx.fill();
    
    const tst_str = msToTime(window.mcTime);
    if ($('time-minecraft')) $('time-minecraft').textContent = tst_str;
    if ($('clock-status')) $('clock-status').textContent = `TST (Élévation: ${(sunElevation * R2D).toFixed(2)}°)`;
}

function updateAstro(latA, lonA) {
    const now = getCDate(); 
    if (now === null || !window.SunCalc) return; 

    const sunPos = window.SunCalc.getPosition(now, latA, lonA);
    const solarTimes = getSolarTime(now, lonA); 
    window.mcTime = solarTimes.TST_MS; 

    if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString('fr-FR', { timeZone: 'UTC', hour12: false });
    if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
    
    if ($('tst')) $('tst').textContent = solarTimes.TST;
    if ($('lsm')) $('lsm').textContent = solarTimes.MST;
    if ($('eot')) $('eot').textContent = solarTimes.EOT + " min";
    
    if ($('sun-elevation')) $('sun-elevation').textContent = (sunPos.altitude * R2D).toFixed(2) + " °";
    
    drawSolarClock(sunPos); 
}


// --- FONCTIONS CARTE LEAFLET ---

function initMap() {
    if (window.map) {
        window.map.remove(); 
    }
    
    const initialLat = 43.296482; 
    const initialLon = 5.36978;
    
    if (window.L && $('map-container')) {
        window.map = L.map('map-container').setView([initialLat, initialLon], 16);
        
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            maxZoom: 19,
            attribution: '© OpenStreetMap'
        }).addTo(window.map);

        window.userMarker = L.marker([initialLat, initialLon]).addTo(window.map)
            .bindPopup("Position initiale").openPopup();
    }
}

function updateMap(latA, lonA, altA, accA) {
    if (window.map && window.userMarker) {
        const newLatLon = L.latLng(latA, lonA);
        window.userMarker.setLatLng(newLatLon);
        window.map.panTo(newLatLon);
        window.userMarker.setPopupContent(
            `Lat: ${latA.toFixed(6)}<br>` + 
            `Lon: ${lonA.toFixed(6)}<br>` + 
            `Alt: ${altA.toFixed(2)} m<br>` +
            `Précision: ${accA.toFixed(2)} m`
        );
    }
}


// --- FONCTION PRINCIPALE DE MISE À JOUR GPS ---

function updateDisp(pos) {
    if (!window.gpsActive) {
        if (!window.lPos) window.lPos = pos;
        return;
    }
    
    const dt = window.lPos ? (pos.timestamp - window.lPos.timestamp) / 1000 : 0.5;
    if (dt <= 0) return;

    const lat = pos.coords.latitude;
    const lon = pos.coords.longitude;
    const altRaw = pos.coords.altitude;
    const acc = window.gpsOverride > 0 ? window.gpsOverride : (pos.coords.accuracy ?? R_MAX); 
    
    // Appel à l'EKF (logique dans part1.js)
    // updateEKF(dt, window.imuAccel.x, window.imuAccel.y, {lat: lat, lon: lon, acc: acc});

    if ($('latitude')) $('latitude').textContent = lat.toFixed(6);
    if ($('longitude')) $('longitude').textContent = lon.toFixed(6);
    if ($('altitude-gps')) $('altitude-gps').textContent = altRaw.toFixed(2) + " m";
    if ($('gps-precision')) $('gps-precision').textContent = acc.toFixed(2) + " m";
    
    updateMap(lat, lon, altRaw, acc);
    
    window.lPos = pos; 
}


// --- INITIALISATION DES ÉVÉNEMENTS DOM ---

document.addEventListener('DOMContentLoaded', () => {
    
    initMap(); 
    initSolarClock(); 

    // Événements des boutons de contrôle
    if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', startGPS);
    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', resetDistance);
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', resetSpeedMax);
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', resetAll);
    
    // Précision forcée
    if ($('gps-accuracy-override')) {
        $('gps-accuracy-override').addEventListener('change', (e) => {
            window.gpsOverride = parseFloat(e.target.value.replace(',', '.')) || 0.0; // Gère la virgule
            if ($('gps-accuracy-effective')) $('gps-accuracy-effective').textContent = window.gpsOverride > 0 ? `${window.gpsOverride.toFixed(6)} m (Forcé)` : 'N/A';
        });
    }

    // Mise à jour périodique des données DOM/Astro/Physique (1x/seconde)
    if (window.domID === null) {
        window.domID = setInterval(() => {
            const currentLat = window.lPos ? window.lPos.coords.latitude : 43.296482; 
            const currentLon = window.lPos ? window.lPos.coords.longitude : 5.36978;
            updateAstro(currentLat, currentLon); 
            updatePhysicsAndChemistry(); 
            
            // Récupération des données externes toutes les 10 secondes (simulation d'API)
            if (Date.now() % 10000 < 1000) { 
                fetchExternalData(currentLat, currentLon); 
            }
        }, DOM_SLOW_UPDATE_MS); 
    }
    
    startIMU();   
});
