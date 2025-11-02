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
window.lVitesse = 0;        // Dernière vitesse (m/s)
window.lTime = 0;           
window.isMoving = false;    
window.distanceTotal = 0;   
window.speedMax = 0;        
window.timeMoving = 0;      
window.gpsActive = false;   
window.imuActive = false;   
window.domID = null;        
window.startTime = Date.now(); 

// VARIABLES EKF (Filtre de Kalman Étendu - Simplifié)
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

// VARIABLES CAPTEURS/APIs EXTERNES (Simulations d'API Météo/Géo)
window.externalData = {
    temp_air: null,         
    pressure: null,         
    humidity: null,         
    magnetic_field: null,   
    solar_radiation: null,  
    wind_speed: null,       
    co2_level: null,
    ph_level: null,
    noise_level: null,
    dew_point: null,         // NOUVEAU MÉTÉO
    visibility: null,        // NOUVEAU MÉTÉO
    uv_index: null,          // NOUVEAU MÉTÉO
    wind_direction: null,    // NOUVEAU MÉTÉO
    precipitation_rate: null,// NOUVEAU MÉTÉO
    temp_feels_like: null    // NOUVEAU MÉTÉO
};

// --- FONCTIONS DE GESTION GPS, IMU & EKF ---

function updateEKF(dt, accX, accY, gpsMeasurement) {
    // *** LOGIQUE EKF PLACEHOLDER ***
    // Met à jour l'état et la covariance pour simuler la stabilité de la vitesse.
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
        window.gpsActive = !window.gpsActive; // Toggle
        if (window.gpsActive) {
            navigator.geolocation.watchPosition(updateDisp, gpsError, {
                enableHighAccuracy: true,
                maximumAge: 500,
                timeout: 5000
            });
            if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = "⏸️ PAUSE GPS";
        } else {
            if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = "▶️ MARCHE GPS";
            // Nettoyer la surveillance GPS si nécessaire (nécessiterait de stocker l'ID de la surveillance)
        }
    } else {
        alert("La géolocalisation n'est pas supportée par ce navigateur.");
    }
}

function handleIMU(event) {
    // Les données d'accélération IMU sont utilisées pour la Force G Verticale
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
}

function resetSpeedMax() {
    window.speedMax = 0;
}

function resetAll() {
    resetDistance();
    resetSpeedMax();
    window.startTime = Date.now();
    window.lPos = null;
    window.lVitesse = 0;
    window.ekfState = { x: 0, y: 0, vx: 0, vy: 0, ax: 0, ay: 0 };
    if ($('toggle-gps-btn').textContent === "⏸️ PAUSE GPS") {
        window.gpsActive = true; // Reste en marche
    }
     }
// =================================================================
// FICHIER JS PARTIE 2/2 : gnss-dashboard-part2.js
// Contient : Fonctions de support, Logique Astro/TST/Montre, Carte Leaflet, Calculs P/C/SVT, Boucles DOM
// =================================================================

// --- FONCTIONS DE SUPPORT ---

function msToTime(ms) {
    const totalSeconds = Math.floor(ms / 1000);
    const hours = Math.floor(totalSeconds / 3600);
    const minutes = Math.floor((totalSeconds % 3600) / 60);
    const seconds = totalSeconds % 60;
    return `${String(hours).padStart(2, '0')}:${String(minutes).padStart(2, '0')}:${String(seconds).padStart(2, '0')}`;
}

function getCDate() {
    return new Date(); 
}

// --- FONCTIONS ASTRO (TST/EOT) ---

function getSolarTime(date, lon) {
    // Calcul précis de l'Heure Solaire Vraie (TST)
    
    const msSinceMidnightUTC = (date.getUTCHours() * 3600 + date.getUTCMinutes() * 60 + date.getUTCSeconds()) * 1000 + date.getUTCMilliseconds();
    
    // Heure Solaire Moyenne Locale (MST)
    const mst_offset_ms = lon * dayMs / 360; 
    const mst_ms = (msSinceMidnightUTC + mst_offset_ms + dayMs) % dayMs;

    // Équation du Temps (EOT) - Simulation basée sur le Jour de l'Année (plus précis qu'une constante)
    const start = new Date(date.getFullYear(), 0, 0);
    const diff = date - start;
    const oneDay = 1000 * 60 * 60 * 24;
    const dayOfYear = Math.floor(diff / oneDay); // jour de l'année (1 à 365)
    
    // Formule approximative pour l'EOT (en minutes)
    const eot_min = 7.3 * Math.sin((dayOfYear + 10) / 365 * 2 * Math.PI) + 
                    -9.8 * Math.sin((dayOfYear - 80) / 365 * 2 * Math.PI); 
    
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
        // Température et Humidité pour calculs de densité/point de rosée
        const temp_c = Math.random() * 20 + 5; // Entre 5 et 25°C
        const humidity = Math.random() * 40 + 40; // Entre 40 et 80 %
        
        window.externalData.temp_air = temp_c.toFixed(1) + " °C"; 
        window.externalData.pressure = (Math.random() * 20 + 1000).toFixed(0) + " hPa"; 
        window.externalData.humidity = humidity.toFixed(0) + " %"; 
        window.externalData.wind_speed = (Math.random() * 10).toFixed(1); // 0-10 m/s
        
        // Géomagnétisme (API NOAA par ex.)
        window.externalData.magnetic_field = (Math.random() * 10 + 40).toFixed(1) + " μT"; // 40-50 μT

        // Radiation Solaire (Calculé en fonction de l'élévation simulée)
        const sunPos = window.SunCalc.getPosition(getCDate(), lat, lon);
        let rad = sunPos.altitude > 0 ? (Math.sin(sunPos.altitude) * 1000) : 0;
        window.externalData.solar_radiation = rad.toFixed(0) + " W/m²";
        
        // CHAMPS MÉTÉO COMPLETS (Simulation)
        // Point de Rosée (Formule simplifiée : Td = T - (100 - H) / 5)
        const dew_point_c = temp_c - (100 - humidity) / 5;
        window.externalData.dew_point = dew_point_c.toFixed(1) + " °C";
        window.externalData.visibility = (Math.random() * 15 + 5).toFixed(1) + " km"; 
        window.externalData.uv_index = Math.floor(Math.random() * 8).toString(); 
        window.externalData.wind_direction = ["Nord", "Nord-Est", "Est", "Sud-Est", "Sud", "Sud-Ouest", "Ouest", "Nord-Ouest"][Math.floor(Math.random() * 8)]; 
        window.externalData.precipitation_rate = (Math.random() * 0.5).toFixed(2) + " mm/h"; 
        window.externalData.temp_feels_like = (temp_c - (Math.random() * 2)).toFixed(1) + " °C";
    }, 500); 
}

// --- FONCTION DE MISE À JOUR PHYSIQUE/CHIMIE/SVT ---

function updatePhysicsAndChemistry() {
    const mass = parseFloat($('mass-display').textContent) || 70.0;
    
    // Données de l'EKF et IMU
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
    const temp_match = window.externalData.temp_air ? window.externalData.temp_air.match(/([\d\.]+)/) : null;
    const pressure_match = window.externalData.pressure ? window.externalData.pressure.match(/([\d\.]+)/) : null;

    // Affichage Météo
    if ($('temp-air')) $('temp-air').textContent = window.externalData.temp_air || "N/A (API)";
    if ($('pressure')) $('pressure').textContent = window.externalData.pressure || "N/A (API)";
    if ($('humidity')) $('humidity').textContent = window.externalData.humidity || "N/A (API)";
    if ($('dew-point')) $('dew-point').textContent = window.externalData.dew_point || "N/A (API)";
    if ($('visibility')) $('visibility').textContent = window.externalData.visibility || "N/A (API)";
    if ($('uv-index')) $('uv-index').textContent = window.externalData.uv_index || "N/A (API)";
    if ($('wind-direction')) $('wind-direction').textContent = window.externalData.wind_direction || "N/A (API)";
    if ($('precipitation-rate')) $('precipitation-rate').textContent = window.externalData.precipitation_rate || "N/A (API)";
    if ($('temp-feels-like')) $('temp-feels-like').textContent = window.externalData.temp_feels_like || "N/A (API)";
    if ($('wind-speed-ms')) $('wind-speed-ms').textContent = (window.externalData.wind_speed ? window.externalData.wind_speed + " m/s" : "N/A (API)");

    // Densité de l'Air (Calcul basé sur l'API)
    let air_density = "N/A";
    if (temp_match && pressure_match) {
        const temp_c = parseFloat(temp_match[1]);
        const temp_k = temp_c + 273.15; 
        const pressure_pa = parseFloat(pressure_match[1]) * 100; // hPa vers Pa
        const R_air = 287.05; 
        air_density = (pressure_pa / (R_air * temp_k)).toFixed(3) + " kg/m³";
    }
    if ($('air-density')) $('air-density').textContent = air_density;

    // Placeholders Chimie & SVT
    if ($('co2-level')) $('co2-level').textContent = window.externalData.co2_level || "N/A (Capteur/API)";
    if ($('ph-level')) $('ph-level').textContent = window.externalData.ph_level || "N/A (Capteur)";
    if ($('solar-radiation')) $('solar-radiation').textContent = window.externalData.solar_radiation || "N/A (API)";
    // ... (autres Placeholders inchangés)
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

function drawSolarClock(sunPos, latA, lonA) {
    const canvas = $('mc-clock-canvas');
    if (!canvas || !sunPos || window.mcTime === null) return; 
    
    const ctx = canvas.getContext('2d');
    const center = canvas.width / 2;
    const radius = center * 0.9;
    
    const sunAzimuth = sunPos.azimuth; 
    const sunElevation = sunPos.altitude; 
    
    const rotationAngle = sunAzimuth + Math.PI / 2; // Angle de rotation du soleil
    
    // Constantes de rendu
    const MOON_RADIUS = center * 0.1;
    const SUN_RADIUS = center * 0.15;
    const GOLD_COLOR = '#FFD700'; 
    
    // Seuils en radians pour le dégradé du ciel (Toutes les minutes)
    const TWI_CIVIL = -0.833 * D2R; // Coucher/Lever civil 
    const TWI_ASTRO = -18 * D2R;    // Crépuscule Astronomique

    // Détermination de la couleur du ciel
    let skyColor, sunColor, nightColor;

    if (sunElevation > TWI_CIVIL) { 
        skyColor = '#4C7AD9'; // Bleu vif (Jour)
        sunColor = 'yellow';
        nightColor = '#101828';
        
        // Dégradé pour l'aube/crépuscule civil
        if (sunElevation < 10 * D2R) {
            const transition = (sunElevation - TWI_CIVIL) / (10 * D2R - TWI_CIVIL); // 0 à 1
            const r = Math.floor(76 + (230 - 76) * (1 - transition)); 
            const g = Math.floor(122 + (153 - 122) * (1 - transition));
            const b = Math.floor(217 + (76 - 217) * (1 - transition));
            skyColor = `rgb(${r}, ${g}, ${b})`; // Bleu au Orange/Rouge
            sunColor = 'orange';
        }
    } else if (sunElevation > TWI_ASTRO) { 
        skyColor = '#212A3E'; // Bleu nuit foncé (Crépuscule)
        sunColor = 'darkorange';
        nightColor = '#101828';
    } else { 
        skyColor = '#101828'; // Noir (Pleine Nuit)
        sunColor = 'darkorange';
        nightColor = '#101828';
    }

    // 1. Dessin du Fond Noir (Nuit Astronomique)
    ctx.clearRect(0, 0, canvas.width, canvas.height); 
    ctx.fillStyle = '#101828'; 
    ctx.beginPath();
    ctx.arc(center, center, radius, 0, 2 * Math.PI);
    ctx.fill();

    // 2. Dessin du Secteur JOUR (avec dégradé)
    ctx.beginPath();
    ctx.arc(center, center, radius, rotationAngle - Math.PI/2, rotationAngle + Math.PI/2);
    ctx.lineTo(center, center);
    ctx.fillStyle = skyColor; 
    ctx.fill();

    // 3. Dessin du CADRE (Inspiration Minecraft : Bords Dorés)
    ctx.strokeStyle = GOLD_COLOR;
    ctx.lineWidth = center * 0.1; 
    ctx.beginPath();
    ctx.arc(center, center, radius - ctx.lineWidth / 2, 0, 2 * Math.PI);
    ctx.stroke();

    // 4. Dessin du SOLEIL
    const elevationFraction = Math.abs(sunElevation) / (Math.PI / 2); 
    const r_sun_pos = radius * elevationFraction; 
    const x_sun = center + r_sun_pos * Math.cos(rotationAngle);
    const y_sun = center + r_sun_pos * Math.sin(rotationAngle);
    
    ctx.beginPath();
    ctx.arc(x_sun, y_sun, SUN_RADIUS, 0, 2 * Math.PI); 
    ctx.fillStyle = sunColor; 
    ctx.fill();

    // 5. Dessin de la LUNE (avec Phase réaliste)
    const moonPos = window.SunCalc.getMoonPosition(getCDate(), latA, lonA);
    const moonIllum = window.SunCalc.getMoonIllumination(getCDate());

    const moonAzimuth = moonPos.azimuth;
    const moonElevation = moonPos.altitude;

    if (moonElevation > TWI_ASTRO) { 
        const r_moon_pos = radius * (1 - Math.abs(moonElevation) / (Math.PI / 2)); 
        const x_moon = center + r_moon_pos * Math.cos(moonAzimuth + Math.PI / 2);
        const y_moon = center + r_moon_pos * Math.sin(moonAzimuth + Math.PI / 2);

        // Dessin du cercle total de la Lune
        ctx.beginPath();
        ctx.arc(x_moon, y_moon, MOON_RADIUS, 0, 2 * Math.PI);
        ctx.fillStyle = '#F0F0F0'; 
        ctx.fill();

        // Dessin de l'OMBRE (Phase Lunaire)
        const phase = moonIllum.phase; 
        const illuminatedFraction = moonIllum.fraction; 
        
        if (illuminatedFraction < 1) {
            ctx.save();
            // Découper la zone à l'intérieur du cercle de la lune
            ctx.beginPath();
            ctx.arc(x_moon, y_moon, MOON_RADIUS, 0, 2 * Math.PI);
            ctx.clip(); 
            
            // Calcul du décalage de l'ombre
            let shadowOffset = MOON_RADIUS * 2 * (1 - 2 * Math.abs(phase - 0.5));
            let shadowX = x_moon + (phase > 0.5 ? -shadowOffset : shadowOffset);
            
            // Dessiner le cercle d'ombre (en utilisant la couleur de nuit)
            ctx.beginPath();
            ctx.arc(shadowX, y_moon, MOON_RADIUS * 1.5, 0, 2 * Math.PI);
            ctx.fillStyle = nightColor; 
            ctx.fill();
            
            ctx.restore(); 
        }
    }

    // 6. Dessin du Pivot Central
    ctx.beginPath();
    ctx.arc(center, center, center * 0.1, 0, 2 * Math.PI);
    ctx.fillStyle = GOLD_COLOR; 
    ctx.fill();
    
    // Mise à jour du TST
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

    // Mise à jour DOM
    if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString('fr-FR', { timeZone: 'UTC', hour12: false });
    if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
    if ($('tst')) $('tst').textContent = solarTimes.TST;
    if ($('lsm')) $('lsm').textContent = solarTimes.MST;
    if ($('eot')) $('eot').textContent = solarTimes.EOT + " min";
    if ($('sun-elevation')) $('sun-elevation').textContent = (sunPos.altitude * R2D).toFixed(2) + " °";
    
    drawSolarClock(sunPos, latA, lonA); 
}


// --- FONCTIONS CARTE LEAFLET (Inchagée) ---

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
    
    const lat = pos.coords.latitude;
    const lon = pos.coords.longitude;
    const altRaw = pos.coords.altitude;
    const acc = window.gpsOverride > 0 ? window.gpsOverride : (pos.coords.accuracy ?? R_MAX); 
    
    // Temps écoulé
    const dt = window.lPos ? (pos.timestamp - window.lPos.timestamp) / 1000 : 0.5;
    if (dt <= 0 || !window.lPos) {
        window.lPos = pos;
        return;
    }

    // -------------------------------------------------------------
    // 1. CALCUL DE DISTANCE ET VITESSE BRUTE
    // -------------------------------------------------------------
    const R_EARTH = 6371e3; // Rayon de la Terre en mètres

    const lat1 = window.lPos.coords.latitude * D2R;
    const lon1 = window.lPos.coords.longitude * D2R;
    const lat2 = lat * D2R;
    const lon2 = lon * D2R;

    // Formule Haversine pour distance (m)
    const dLat = lat2 - lat1;
    const dLon = lon2 - lon1;
    const a = Math.sin(dLat/2) * Math.sin(dLat/2) +
              Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon/2) * Math.sin(dLon/2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
    const distance_segment = R_EARTH * c; // Distance en mètres

    // Vitesse Brute
    const speed_raw_ms = distance_segment / dt;
    window.lVitesse = speed_raw_ms; 

    // -------------------------------------------------------------
    // 2. MISE À JOUR CUMULÉE
    // -------------------------------------------------------------
    window.distanceTotal += distance_segment;

    // Si la vitesse dépasse un seuil (ex: 0.5 m/s)
    if (speed_raw_ms > 0.5) { 
        window.timeMoving += dt * 1000; // en ms
        window.isMoving = true;
    } else {
        window.isMoving = false;
    }

    // Vitesse Max (Session)
    if (speed_raw_ms * KMH_MS > window.speedMax) {
        window.speedMax = speed_raw_ms * KMH_MS;
        if ($('speed-max')) $('speed-max').textContent = window.speedMax.toFixed(5) + " km/h";
    }

    // -------------------------------------------------------------
    // 3. MISE À JOUR DOM
    // -------------------------------------------------------------
    if ($('latitude')) $('latitude').textContent = lat.toFixed(6);
    if ($('longitude')) $('longitude').textContent = lon.toFixed(6);
    if ($('altitude-gps')) $('altitude-gps').textContent = altRaw.toFixed(2) + " m";
    if ($('gps-precision')) $('gps-precision').textContent = acc.toFixed(2) + " m";
    if ($('speed-raw-ms')) $('speed-raw-ms').textContent = speed_raw_ms.toFixed(2) + " m/s";
    if ($('distance-total-km')) $('distance-total-km').textContent = (window.distanceTotal / 1000).toFixed(3) + " km | " + window.distanceTotal.toFixed(2) + " m";
    
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
            window.gpsOverride = parseFloat(e.target.value.replace(',', '.')) || 0.0; 
            if ($('gps-accuracy-effective')) $('gps-accuracy-effective').textContent = window.gpsOverride > 0 ? `${window.gpsOverride.toFixed(6)} m (Forcé)` : 'N/A';
        });
    }

    // Mise à jour périodique des données DOM/Astro/Physique (1x/seconde)
    if (window.domID === null) {
        window.domID = setInterval(() => {
            const currentLat = window.lPos ? window.lPos.coords.latitude : 43.296482; 
            const currentLon = window.lPos ? window.lPos.coords.longitude : 5.36978;
            
            // Mise à jour du temps de session et de mouvement
            const elapsed_ms = Date.now() - window.startTime;
            if ($('time-elapsed')) $('time-elapsed').textContent = msToTime(elapsed_ms);
            if ($('time-moving')) $('time-moving').textContent = msToTime(window.timeMoving);
            
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
