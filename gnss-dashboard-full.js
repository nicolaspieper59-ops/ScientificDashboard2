// =================================================================
// FICHIER JS PARTIE 1/2 : gnss-dashboard-part1.js
// Contient : Variables globales, Constantes, Fonctions de support, Initialisation EKF/IMU, Fonctions de Démarrage/Contrôle
// =================================================================

// --- FONCTIONS DE SUPPORT GLOBALES ---
const $ = (id) => document.getElementById(id); 
const D2R = Math.PI / 180, R2D = 180 / Math.PI;

// --- CONSTANTES GLOBALES ET INITIALISATION ---
const C_L = 299792458;      // Vitesse de la Lumière (m/s)
const C_S = 343;            // Vitesse du Son (m/s)
const R_E = 6371000;        // Rayon de la Terre (m)
const KMH_MS = 3.6;         // Facteur de conversion (m/s vers km/h)
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

// VARIABLES EKF (Filtre de Kalman Étendu - Simplifié pour la vitesse)
window.ekfState = { 
    vx: 0, 
    vy: 0, 
    ax: 0, 
    ay: 0 
};
window.ekfCov = [
    [100, 0, 0, 0], 
    [0, 100, 0, 0],
    [0, 0, 10, 0],
    [0, 0, 0, 10]
];
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
    temp_air: null, pressure: null, humidity: null, magnetic_field: null,   
    solar_radiation: null, wind_speed: null, co2_level: null, ph_level: null,
    noise_level: null, dew_point: null, visibility: null, uv_index: null,          
    wind_direction: null, precipitation_rate: null, temp_feels_like: null    
};

// --- FONCTIONS DE GESTION GPS, IMU & EKF ---

/**
 * Mise à jour de l'EKF pour lisser la vitesse. (Filtre passe-bas très simplifié)
 * @param {number} dt - Différence de temps (secondes)
 * @param {number} accX - Accélération X (non utilisée ici pour simplification)
 * @param {number} accY - Accélération Y (non utilisée ici pour simplification)
 * @param {object} gpsMeasurement - { speed: vitesse brute m/s, accuracy: R (bruit de mesure corrigé) }
 */
function updateEKF(dt, accX, accY, gpsMeasurement) {
    // EKF TRÈS SIMPLIFIÉ (filtre passe-bas)
    const tau = 0.5; // Constante de temps (0.5 seconde pour le lissage)
    const alpha = dt / (dt + tau); // Facteur de lissage
    
    // Le bruit de mesure (R) est déjà corrigé par la météo dans updateDisp.
    const R = gpsMeasurement.accuracy; 

    // Prédiction (pas de prédiction complexe, on suppose vitesse constante)
    // Pour une vraie EKF, il faudrait prédire l'état (x, y, vx, vy)
    
    // Mise à jour (Correction - Lissage simple)
    // On lissage la vitesse Vx (car on ne calcule qu'une vitesse stable scalaire)
    window.ekfState.vx = (1 - alpha) * window.ekfState.vx + alpha * gpsMeasurement.speed;
    
    // Simplification de l'incertitude (Plus la précision R est grande, plus l'incertitude finale est grande)
    const uncertainty_simulated = Math.max(0.01, R / 50); // Un minimum de 0.01m/s²
    
    return { 
        vx: window.ekfState.vx, 
        vy: 0,
        uncertainty: uncertainty_simulated // en m²/s² (variance)
    }; 
}

function gpsError(error) {
    console.error("Erreur GPS:", error.code, error.message);
}

function startGPS() {
    if ("geolocation" in navigator) {
        window.gpsActive = !window.gpsActive; // Toggle
        if (window.gpsActive) {
            // ... (démarrage du GPS inchangé)
            navigator.geolocation.watchPosition(updateDisp, gpsError, {
                enableHighAccuracy: true,
                maximumAge: 500,
                timeout: 5000
            });
            if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = "⏸️ PAUSE GPS";
        } else {
            if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = "▶️ MARCHE GPS";
        }
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
    window.ekfState.vx = 0;
    window.ekfState.vy = 0;
    if ($('toggle-gps-btn').textContent === "⏸️ PAUSE GPS") {
        window.gpsActive = true; 
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
    const msSinceMidnightUTC = (date.getUTCHours() * 3600 + date.getUTCMinutes() * 60 + date.getUTCSeconds()) * 1000 + date.getUTCMilliseconds();
    
    // Heure Solaire Moyenne Locale (MST)
    const mst_offset_ms = lon * dayMs / 360; 
    const mst_ms = (msSinceMidnightUTC + mst_offset_ms + dayMs) % dayMs;

    // Équation du Temps (EOT) - Simulation basée sur le Jour de l'Année
    const start = new Date(date.getFullYear(), 0, 0);
    const diff = date - start;
    const oneDay = 1000 * 60 * 60 * 24;
    const dayOfYear = Math.floor(diff / oneDay); 
    
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

// --- SIMULATION D'APPEL API EXTERNE (MÉTÉO) ---

function fetchExternalData(lat, lon) {
    setTimeout(() => {
        const temp_c = Math.random() * 20 + 5; 
        const humidity = Math.random() * 40 + 40; 
        const wind_speed = (Math.random() * 10).toFixed(1); 
        
        window.externalData.temp_air = temp_c.toFixed(1) + " °C"; 
        window.externalData.pressure = (Math.random() * 20 + 1000).toFixed(0) + " hPa"; 
        window.externalData.humidity = humidity.toFixed(0) + " %"; 
        window.externalData.wind_speed = wind_speed; 
        window.externalData.magnetic_field = (Math.random() * 10 + 40).toFixed(1) + " μT"; 
        
        const sunPos = window.SunCalc.getPosition(getCDate(), lat, lon);
        let rad = sunPos.altitude > 0 ? (Math.sin(sunPos.altitude) * 1000) : 0;
        window.externalData.solar_radiation = rad.toFixed(0) + " W/m²";
        
        // CHAMPS MÉTÉO COMPLETS (Simulation)
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
    
    // Utilisation de la vitesse stable EKF pour les calculs physiques
    const speed_stable_ms = window.ekfState.vx; 
    
    // 1. Grandeurs Physiques
    const kinetic_energy = 0.5 * mass * speed_stable_ms * speed_stable_ms;
    if ($('kinetic-energy')) $('kinetic-energy').textContent = kinetic_energy.toFixed(2) + " J";

    // Note: Accélération Longitudinale (ax) est un placeholder dans l'EKF simplifié
    const accel_long = 0; 
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

    // 2. Grandeurs Chimiques & Météo (Affichage des données Météo complètes)
    // ... (Affichage des données Météo/Chimie inchangé)
    const temp_match = window.externalData.temp_air ? window.externalData.temp_air.match(/([\d\.]+)/) : null;
    const pressure_match = window.externalData.pressure ? window.externalData.pressure.match(/([\d\.]+)/) : null;
    
    if ($('temp-air')) $('temp-air').textContent = window.externalData.temp_air || "N/A (API)";
    if ($('pressure')) $('pressure').textContent = window.externalData.pressure || "N/A (API)";
    if ($('humidity')) $('humidity').textContent = window.externalData.humidity || "N/A (API)";
    if ($('dew-point')) $('dew-point').textContent = window.externalData.dew_point || "N/A (API)";
    if ($('visibility')) $('visibility').textContent = window.externalData.visibility || "N/A (API)";
    if ($('uv-index')) $('uv-index').textContent = window.externalData.uv_index || "N/A (API)";
    if ($('wind-direction')) $('wind-direction').textContent = window.externalData.wind_direction || "N/A (API)";
    if ($('precipitation-rate')) $('precipitation-rate').textContent = window.externalData.precipitation_rate || "N/A (API)";
    if ($('temp-feels-like')) $('temp-feels-like').textContent = window.externalData.temp_feels-like || "N/A (API)";
    if ($('wind-speed-ms')) $('wind-speed-ms').textContent = (window.externalData.wind_speed ? window.externalData.wind_speed + " m/s" : "N/A (API)");
    
    let air_density = "N/A";
    if (temp_match && pressure_match) {
        const temp_c = parseFloat(temp_match[1]);
        const temp_k = temp_c + 273.15; 
        const pressure_pa = parseFloat(pressure_match[1]) * 100; 
        const R_air = 287.05; 
        air_density = (pressure_pa / (R_air * temp_k)).toFixed(3) + " kg/m³";
    }
    if ($('air-density')) $('air-density').textContent = air_density;
    if ($('solar-radiation')) $('solar-radiation').textContent = window.externalData.solar_radiation || "N/A (API)";
    // ... (autres Placeholders inchangés)
}


// --- FONCTIONS MONTRE TST/ASTRO (CORRIGÉE) ---

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
    
    // CORRECTION DE L'ORIENTATION TST : Utilisation du temps solaire lui-même pour l'angle de rotation
    // 00:00 TST correspond à 0 rotation (Minuit, en bas)
    // Pi/2 correspond à 06:00 TST (Gauche)
    // 3*Pi/2 correspond à 18:00 TST (Droite)
    const rotationAngle = (window.mcTime / dayMs * 2 * Math.PI) - Math.PI / 2; 
    
    // Constantes de rendu
    const MOON_RADIUS = center * 0.1;
    const SUN_RADIUS = center * 0.15;
    const GOLD_COLOR = '#FFD700'; 
    
    // Seuils en radians pour le dégradé du ciel
    const TWI_CIVIL = -0.833 * D2R; 
    const TWI_ASTRO = -18 * D2R;    

    // Détermination de la couleur du ciel (Dégradé toutes les minutes)
    let skyColor, sunColor, nightColor;

    if (sunElevation > TWI_CIVIL) { 
        skyColor = '#4C7AD9'; 
        sunColor = 'yellow';
        nightColor = '#101828';
        
        // Dégradé pour l'aube/crépuscule civil
        if (sunElevation < 10 * D2R) {
            const transition = (sunElevation - TWI_CIVIL) / (10 * D2R - TWI_CIVIL);
            const r = Math.floor(76 + (230 - 76) * (1 - transition)); 
            const g = Math.floor(122 + (153 - 122) * (1 - transition));
            const b = Math.floor(217 + (76 - 217) * (1 - transition));
            skyColor = `rgb(${r}, ${g}, ${b})`; 
            sunColor = 'orange';
        }
    } else if (sunElevation > TWI_ASTRO) { 
        skyColor = '#212A3E'; 
        sunColor = 'darkorange';
        nightColor = '#101828';
    } else { 
        skyColor = '#101828'; 
        sunColor = 'darkorange';
        nightColor = '#101828';
    }

    // ------------------------------------------------
    // 1. DESSIN DU CADRAN AVEC MASQUE (CLIP)
    // ------------------------------------------------

    // A. Dessin du cadre (non masqué)
    ctx.strokeStyle = GOLD_COLOR;
    ctx.lineWidth = center * 0.1; 
    ctx.beginPath();
    ctx.arc(center, center, radius - ctx.lineWidth / 2, 0, 2 * Math.PI);
    ctx.stroke();

    // B. Masque pour ne montrer que la **moitié supérieure** (y < center)
    ctx.save();
    ctx.beginPath();
    ctx.rect(0, 0, canvas.width, center); // Zone de découpe : de (0,0) à (width, center)
    ctx.clip(); 

    // C. Dessin du Fond Noir (Nuit Astronomique) dans la zone masquée
    ctx.fillStyle = '#101828'; 
    ctx.beginPath();
    ctx.arc(center, center, radius, 0, 2 * Math.PI);
    ctx.fill();

    // D. Dessin du Secteur Jour (avec dégradé)
    ctx.beginPath();
    ctx.arc(center, center, radius, rotationAngle - Math.PI/2, rotationAngle + Math.PI/2);
    ctx.lineTo(center, center);
    ctx.fillStyle = skyColor; 
    ctx.fill();

    // Restauration du contexte pour annuler le masque
    ctx.restore(); 

    // ------------------------------------------------
    // 2. DESSIN DU SOLEIL (ASTRE)
    // ------------------------------------------------
    // Position des astres centrée sur le pivot, angle TST
    const x_sun = center + (radius * 0.7) * Math.cos(rotationAngle + Math.PI/2);
    const y_sun = center + (radius * 0.7) * Math.sin(rotationAngle + Math.PI/2);
    
    ctx.beginPath();
    ctx.arc(x_sun, y_sun, SUN_RADIUS, 0, 2 * Math.PI); 
    ctx.fillStyle = sunColor; 
    ctx.fill();

    // 3. DESSIN DE LA LUNE (avec Phase réaliste)
    const moonPos = window.SunCalc.getMoonPosition(getCDate(), latA, lonA);
    const moonIllum = window.SunCalc.getMoonIllumination(getCDate());

    const moonAzimuth = moonPos.azimuth;
    const moonElevation = moonPos.altitude;

    // Utilisation de l'angle TST pour la rotation de la lune
    const moonRotationAngle = (moonPos.parallacticAngle - Math.PI / 2) + moonAzimuth;

    if (moonElevation > TWI_ASTRO) { 
        const r_moon_pos = radius * 0.7; 
        const x_moon = center + r_moon_pos * Math.cos(moonRotationAngle);
        const y_moon = center + r_moon_pos * Math.sin(moonRotationAngle);

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
            ctx.beginPath();
            ctx.arc(x_moon, y_moon, MOON_RADIUS, 0, 2 * Math.PI);
            ctx.clip(); 
            
            let shadowOffset = MOON_RADIUS * 2 * (1 - 2 * Math.abs(phase - 0.5));
            let shadowX = x_moon + (phase > 0.5 ? -shadowOffset : shadowOffset);
            
            ctx.beginPath();
            ctx.arc(shadowX, y_moon, MOON_RADIUS * 1.5, 0, 2 * Math.PI);
            ctx.fillStyle = nightColor; 
            ctx.fill();
            
            ctx.restore(); 
        }
    }

    // 4. Dessin du Pivot Central
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

    // Mise à jour DOM - CORRECTION GMT
    const gmt_time_str = now.toLocaleTimeString('en-US', { hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false, timeZone: 'UTC' });
    if ($('local-time')) $('local-time').textContent = gmt_time_str + " (GMT/UTC)";
    if ($('date-display')) $('date-display').textContent = now.toLocaleDateString('fr-FR');
    
    // ... (affichage des autres champs astro inchangé)
    if ($('tst')) $('tst').textContent = solarTimes.TST;
    if ($('lsm')) $('lsm').textContent = solarTimes.MST;
    if ($('eot')) $('eot').textContent = solarTimes.EOT + " min";
    if ($('sun-elevation')) $('sun-elevation').textContent = (sunPos.altitude * R2D).toFixed(2) + " °";
    
    drawSolarClock(sunPos, latA, lonA); 
}

// ... (Fonctions initMap et updateMap inchangées) ...

// --- FONCTION PRINCIPALE DE MISE À JOUR GPS (CORRIGÉE) ---

function updateDisp(pos) {
    if (!window.gpsActive) {
        if (!window.lPos) window.lPos = pos;
        return;
    }
    
    const lat = pos.coords.latitude;
    const lon = pos.coords.longitude;
    const altRaw = pos.coords.altitude;
    const acc = window.gpsOverride > 0 ? window.gpsOverride : (pos.coords.accuracy ?? R_MAX); 
    
    const dt = window.lPos ? (pos.timestamp - window.lPos.timestamp) / 1000 : 0.5;
    if (dt <= 0 || !window.lPos) {
        window.lPos = pos;
        return;
    }

    // -------------------------------------------------------------
    // 1. CALCUL DE DISTANCE ET VITESSE BRUTE
    // -------------------------------------------------------------
    const R_EARTH = 6371e3; 
    const lat1 = window.lPos.coords.latitude * D2R;
    const lon1 = window.lPos.coords.longitude * D2R;
    const lat2 = lat * D2R;
    const lon2 = lon * D2R;
    const dLat = lat2 - lat1;
    const dLon = lon2 - lon1;
    const a = Math.sin(dLat/2) * Math.sin(dLat/2) +
              Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon/2) * Math.sin(dLon/2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
    const distance_segment = R_EARTH * c; 

    const speed_raw_ms = distance_segment / dt;
    window.lVitesse = speed_raw_ms; 

    // -------------------------------------------------------------
    // 2. CORRECTION DE VITESSE PAR EKF ET MISE À JOUR CUMULÉE
    // -------------------------------------------------------------
    
    // Calcul de R (bruit de mesure) et CORRECTION MÉTÉO
    let R = acc * acc / 4; // R est la variance (m²)

    const wind_speed_ms = parseFloat(window.externalData.wind_speed) || 0;
    
    // Si la vitesse brute est faible (< 0.5 m/s) et qu'il y a du vent, augmenter l'incertitude R
    if (speed_raw_ms < 0.5 && wind_speed_ms > 5) {
        R *= 1.5; // Augmenter l'incertitude (R) par vent
        if ($('gps-accuracy-effective')) $('gps-accuracy-effective').textContent = R.toFixed(2) + " (Corrigé Météo)";
    } else {
        if ($('gps-accuracy-effective')) $('gps-accuracy-effective').textContent = R.toFixed(2) + " (Base)";
    }

    // EKF
    const ekf_result = updateEKF(dt, 0, 0, { speed: speed_raw_ms, accuracy: R }); 
    const speed_stable_ms = ekf_result.vx; 
    const speed_stable_kmh = speed_stable_ms * KMH_MS;
    
    // Vitesse Max (Session)
    if (speed_stable_kmh > window.speedMax) {
        window.speedMax = speed_stable_kmh;
    }
    if ($('speed-max')) $('speed-max').textContent = window.speedMax.toFixed(5) + " km/h";

    // Mise à jour de la distance, du temps de mouvement, etc.
    window.distanceTotal += distance_segment;
    if (speed_stable_ms > 0.5) { 
        window.timeMoving += dt * 1000; 
        window.isMoving = true;
    } else {
        window.isMoving = false;
    }

    // -------------------------------------------------------------
    // 3. AFFICHAGE DES VITESSES STABLES
    // -------------------------------------------------------------
    if ($('speed-stable')) $('speed-stable').textContent = speed_stable_kmh.toFixed(2) + " km/h";
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = speed_stable_ms.toFixed(2) + " m/s | " + (speed_stable_ms * 1000).toFixed(0) + " mm/s";

    if ($('perc-speed-sound')) $('perc-speed-sound').textContent = (speed_stable_ms / C_S * 100).toFixed(2) + " %";
    if ($('perc-speed-c')) $('perc-speed-c').textContent = (speed_stable_ms / C_L * 100).toExponential(2) + " %";

    // Affichage Précision EKF (racine carrée de la variance)
    if ($('speed-error-perc')) $('speed-error-perc').textContent = (Math.sqrt(ekf_result.uncertainty) * 100).toFixed(2) + " %";


    // -------------------------------------------------------------
    // 4. MISE À JOUR DOM GPS BRUT
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
    
    // ... (initialisation MAP et CLOCK inchangée)
    initMap(); 
    initSolarClock(); 

    // Événements des boutons de contrôle (inchangé)
    if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', startGPS);
    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', resetDistance);
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', resetSpeedMax);
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', resetAll);
    
    if ($('gps-accuracy-override')) {
        $('gps-accuracy-override').addEventListener('change', (e) => {
            window.gpsOverride = parseFloat(e.target.value.replace(',', '.')) || 0.0; 
        });
    }

    // Mise à jour périodique des données DOM/Astro/Physique (1x/seconde)
    if (window.domID === null) {
        window.domID = setInterval(() => {
            const currentLat = window.lPos ? window.lPos.coords.latitude : 43.296482; 
            const currentLon = window.lPos ? window.lPos.coords.longitude : 5.36978;
            
            const elapsed_ms = Date.now() - window.startTime;
            if ($('time-elapsed')) $('time-elapsed').textContent = msToTime(elapsed_ms);
            if ($('time-moving')) $('time-moving').textContent = msToTime(window.timeMoving);
            
            updateAstro(currentLat, currentLon); 
            updatePhysicsAndChemistry(); 
            
                // ... (suite de la fonction setInterval)
            if (Date.now() % 10000 < 1000) { 
                fetchExternalData(currentLat, currentLon); 
            }
        }, DOM_SLOW_UPDATE_MS); 
    }
    
    startIMU();   
});
// FIN DU CODE POUR LA MISE À JOUR DOM/ASTRO/PHYSIQUE


// -----------------------------------------------------------------
// --- FONCTIONS CARTE (ASSUMANT L'UTILISATION DE LEAFLET) ---
// -----------------------------------------------------------------

function initMap() {
    // Initialisation de la carte Leaflet (remplacer les coordonnées par défaut si nécessaire)
    if ($('map') && window.L) {
        window.map = window.L.map('map').setView([43.296482, 5.36978], 13);
        
        window.L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
        }).addTo(window.map);
        
        // Marker de l'utilisateur
        window.userMarker = window.L.circleMarker([0, 0], {
            radius: 8,
            color: 'red',
            fillColor: '#f03',
            fillOpacity: 0.5
        }).addTo(window.map);
    }
}

function updateMap(lat, lon, altRaw, acc) {
    if (window.map && window.userMarker) {
        const latLng = [lat, lon];
        
        // Déplacer le marqueur
        window.userMarker.setLatLng(latLng);
        
        // Ajuster le rayon du cercle pour représenter la précision (accuracy)
        // La librairie Leaflet utilise `setRadius` pour les cercles
        window.userMarker.setRadius(Math.max(5, acc * 0.5)); // Rayon minimum de 5m

        // Centrer la vue sur l'utilisateur une seule fois au démarrage
        if (!window.map.isCentered) {
            window.map.setView(latLng, 16);
            window.map.isCentered = true;
        }
    }
}

// -----------------------------------------------------------------
// --- FIN DU FICHIER gnss-dashboard-part2.js ---
// -----------------------------------------------------------------
