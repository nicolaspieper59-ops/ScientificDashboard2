// =================================================================
// FICHIER 1/2 : gnss-dashboard-core.js
// CŒUR DU SYSTÈME (Variables, EKF avec Prédiction IMU, Capteurs, Logique GPS/IMU)
// =================================================================

const $ = (id) => document.getElementById(id);

// --- PARTIE 1 : CONSTANTES GLOBALES ---
const C_L = 299792458; // Vitesse de la lumière (m/s)
const SPEED_SOUND = 343; // Vitesse du son (m/s)
const G_ACC_STD = 9.80665; // Gravité standard de la Terre (m/s²)
const M_EARTH = 5.972e24; // Masse de la Terre (kg)
const G_CONST = 6.67430e-11; // Constante gravitationnelle (N(m/kg)²)
const R_E = 6371000; // Rayon moyen de la Terre (m)
const R2D = 180 / Math.PI;
const D2R = Math.PI / 180;

// Constantes de Conversion
const KMH_MS = 3.6; 
const KMS_MS = 1000; 
const AU_TO_M = 149597870700; 
const LIGHT_YEAR_TO_M = 9.461e15; 
const SEC_LIGHT = 1; 
const MIN_LIGHT = 60;
const HOUR_LIGHT = 3600;
const DAY_LIGHT = 86400;
const WEEK_LIGHT = 604800;
const MONTH_LIGHT = 2592000; 

// Constantes Planétaires (utilisées pour la gravité dynamique)
const G_SURFACE_MOON = 1.625; const R_MOON = 1737400; const M_MOON = 7.34767e22; 
const G_SURFACE_MARS = 3.721; const R_MARS = 3389500; const M_MARS = 6.4171e23; 

// Constantes GPS et EKF
const MIN_DT = 0.05; 
const Q_NOISE = 0.001; // Bruit de processus pour l'EKF (EKF Process Noise)
const MIN_SPD = 0.001; 
const MIN_UNCERT_FLOOR = Q_NOISE * MIN_DT; 
const OWM_API_KEY = "VOTRE_CLE_API_OPENWEATHERMAP"; // !!! À REMPLACER !!!
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc"; 

// --- VARIABLES GLOBALES (EKF & État) ---
let lat = 0, lon = 0, alt = 0, speed = 0, gpsTS = 0;
let kSpd = 0, kUncert = 1000, kAlt = 0, kAltUncert = 1000;
let lastTS = 0, lastFSpeed = 0, distM = 0;
let lastPos = null, lastAlt = 0; 
let sTime = null, timeMoving = 0, maxSpd = 0; 
let latestAccelZ = 0, latestLinearAccelMagnitude = 0, maxGForce = 0;
let wID = null, domID = null, weatherID = null;
let currentGPSMode = 'HIGH_FREQ'; 
let emergencyStopActive = false;
let currentCelestialBody = 'EARTH'; 
let G_ACC_LOCAL = G_ACC_STD; // Gravité locale dynamique

let P_RECORDS = { max_kUncert_min: 1000, max_acc_min: 1000, max_g_force_max: 0 };
const P_RECORDS_KEY = 'gnss_precision_records'; 

// ===========================================
// FONCTIONS UTILITAIRES ET GRAVITÉ DYNAMIQUE
// ===========================================

/** Calcule la gravité locale en fonction de l'altitude et du corps céleste. */
function calculateLocalGravity(altitude) {
    let R, M, g_surface;
    switch (currentCelestialBody) {
        case 'MOON': R = R_MOON; M = M_MOON; g_surface = G_SURFACE_MOON; break;
        case 'MARS': R = R_MARS; M = M_MARS; g_surface = G_SURFACE_MARS; break;
        case 'EARTH': default: R = R_E; M = M_EARTH; g_surface = G_ACC_STD; break;
    }
    
    if (altitude === null || isNaN(altitude) || currentCelestialBody !== 'EARTH') {
        G_ACC_LOCAL = g_surface; 
        return g_surface; 
    }
    
    const h = altitude;
    const g_local = G_CONST * M / Math.pow(R + h, 2);
    
    G_ACC_LOCAL = g_local; 
    return g_local;
}

/** Met à jour le corps céleste actuel et recalcule la gravité. */
function updateCelestialBody(body) {
    currentCelestialBody = body;
    console.log(`Corps céleste réglé sur : ${body}`);
    calculateLocalGravity(alt); 
    $('gravity-local').textContent = `${G_ACC_LOCAL.toFixed(5)} m/s²`;
}

function loadPrecisionRecords() {
    try {
        const stored = localStorage.getItem(P_RECORDS_KEY);
        if (stored) {
            const loaded = JSON.parse(stored);
            P_RECORDS = { ...P_RECORDS, ...loaded };
            maxGForce = P_RECORDS.max_g_force_max;
        }
    } catch (e) { console.error("Erreur de chargement des records:", e); }
}

function savePrecisionRecords() {
    P_RECORDS.max_g_force_max = maxGForce; 
    try {
        localStorage.setItem(P_RECORDS_KEY, JSON.stringify(P_RECORDS));
    } catch (e) { console.error("Erreur de sauvegarde des records:", e); }
}

function distanceCalc(lat1, lon1, lat2, lon2) {
    if (lat1 === 0 && lon1 === 0) return 0;
    const R = R_E; 
    const dLat = (lat2 - lat1) * D2R;
    const dLon = (lon2 - lon1) * D2R;
    const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
              Math.cos(lat1 * D2R) * Math.cos(lat2 * D2R) * Math.sin(dLon / 2) * Math.sin(dLon / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R * c;
}

function getEnvironmentFactor() {
    const env = $('environment-select').value;
    switch (env) {
        case 'FOREST': return 1.5;
        case 'METAL': return 2.5;
        case 'CONCRETE': return 3.0; 
        default: return 1.0; 
    }
}

function toReadableScientific(num) {
    if (num === 0) return "0.00";
    const exponent = Math.floor(Math.log10(Math.abs(num)));
    const mantissa = num / Math.pow(10, exponent);
    return `${mantissa.toFixed(2)}e+${exponent}`;
}

/** Calcule l'heure corrigée en utilisant le décalage NTP/GPS. */
function getCDate() {
    return sTime ? new Date(new Date().getTime() - (sTime.getTime() - gpsTS)) : new Date();
}

// ===========================================
// CŒUR EKF AVEC PRÉDICTION IMU HAUTE FRÉQUENCE
// ===========================================

/** Handler des données GPS et du filtre EKF. */
function updateDisp(pos) {
    if (emergencyStopActive) return;
    
    // Vérification de la dépendance (gnss-dashboard-ui-init.js)
    if (typeof updateMap !== 'function') return; 

    const acc = $('gps-accuracy-override').value !== '0.000000' ? parseFloat($('gps-accuracy-override').value) : pos.coords.accuracy;

    // --- MISE À JOUR DES POSITIONS ET TEMPS ---
    lat = pos.coords.latitude; 
    lon = pos.coords.longitude;
    const currentAlt = pos.coords.altitude; 
    alt = currentAlt; 
    gpsTS = pos.timestamp;
    
    const nowTS = getCDate().getTime();
    const dt = lastTS === 0 ? MIN_DT : (nowTS - lastTS) / 1000;
    
    if (dt === 0) return; 

    // --- CORRECTION VITESSE : Calcul manuel de la vitesse brute ---
    let speedRaw;
    if (lastPos && dt > 0) {
        const d_horiz_raw = distanceCalc(lastPos.latitude, lastPos.longitude, lat, lon);
        speedRaw = d_horiz_raw / dt; 
    } else {
        speedRaw = pos.coords.speed !== null ? pos.coords.speed : 0; 
    }

    const currentHeading = pos.coords.heading !== null && !isNaN(pos.coords.heading) ? pos.coords.heading : 'N/A';
    lastTS = nowTS;

    const g_dynamic = calculateLocalGravity(currentAlt);
    $('gravity-local').textContent = `${g_dynamic.toFixed(5)} m/s²`;
    
    // --- ACCÉLÉRATION IMU SIGNÉE pour PRÉDICTION EKF ---
    const IMU_ACCEL_THRESHOLD = 0.05; 
    const accel_ekf = (dt > MIN_DT) ? (kSpd - lastFSpeed) / dt : 0; 
    
    let accel_imu_sign = Math.sign(pos.coords.speed !== null ? pos.coords.speed : kSpd);
    if (Math.abs(accel_imu_sign) < 0.1) accel_imu_sign = Math.sign(accel_ekf || 1);
    
    let accel_imu_signed = 0;
    if (latestLinearAccelMagnitude > IMU_ACCEL_THRESHOLD) { 
        accel_imu_signed = latestLinearAccelMagnitude * accel_imu_sign; 
    }

    // --- NOUVEAU EKF : PRÉDICTION HAUTE FRÉQUENCE (IMU) & CORRECTION (GPS) ---
    
    // 1. PRÉDICTION: Utiliser l'accélération IMU pour estimer le nouvel état (Vitesse et Incertitude)
    let kSpd_pred = kSpd;
    let kUncert_pred = kUncert;

    if (Math.abs(accel_imu_signed) > 0.05) { 
        kSpd_pred = kSpd + accel_imu_signed * dt; // Utilisation de l'accélération IMU
        kUncert_pred = kUncert + Q_NOISE * dt; 
    } else {
        kUncert_pred = kUncert + Q_NOISE * dt; // Prédiction classique (seulement bruit Q)
    }

    // 2. CORRECTION: Correction de la prédiction par la mesure GPS brute
    
    let kR = acc * acc * getEnvironmentFactor(); 
    let kGain = kUncert_pred / (kUncert_pred + kR);

    kSpd = kSpd_pred + kGain * (speedRaw - kSpd_pred); 
    kUncert = (1 - kGain) * kUncert_pred;             

    kUncert = Math.max(kUncert, MIN_UNCERT_FLOOR); 
    
    // --- ZVU ÉTALONNAGE ---
    if (speedRaw < MIN_SPD * 10 && kSpd < MIN_SPD) { 
        kSpd = 0; 
    }
    
    // --- CALCUL ACCÉLÉRATION FINALE ---
    let sSpdFE = Math.abs(kSpd);
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    
    // Fusion pour l'affichage : IMU prend le dessus si il y a mouvement significatif
    const FUSION_FACTOR = 0.7; 
    let accel_long;
    if (Math.abs(accel_imu_signed) > Math.abs(accel_ekf) && sSpdFE > 0.1) {
        accel_long = (accel_ekf * (1 - FUSION_FACTOR)) + (accel_imu_signed * FUSION_FACTOR);
    } else {
        accel_long = accel_ekf;
    }
    
    lastFSpeed = sSpdFE; 

    const currentGForceLong = Math.abs(accel_long / g_dynamic); 
    if (currentGForceLong > maxGForce) maxGForce = currentGForceLong; 

    // --- CALCUL VITESSE VERTICALE ET 3D ---
    let verticalSpeedRaw = 0;
    let speed3DInst = sSpdFE; 
    
    if (currentAlt !== null && lastAlt !== 0 && dt > 0) {
        verticalSpeedRaw = (currentAlt - lastAlt) / dt;
        speed3DInst = Math.sqrt(sSpdFE * sSpdFE + verticalSpeedRaw * verticalSpeedRaw);
    }
    
    // --- CALCUL DE LA DISTANCE (3D) ET VITESSE MOYENNE ---
    if (lastPos) {
        if (sSpdFE > MIN_SPD) { 
            const d_horiz_ekf = sSpdFE * dt;
            let d_vert = (verticalSpeedRaw) * dt;
            const d_3d = Math.sqrt(d_horiz_ekf * d_horiz_ekf + d_vert * d_vert); 
            distM += d_3d; 
            timeMoving += dt;
        }
    } 
    
    if (currentAlt !== null) lastAlt = currentAlt; 
    lastPos = { latitude: lat, longitude: lon };

    // --- MISE À JOUR DOM : VITESSE, DISTANCE, ACCÉLÉRATION, etc. ---
    $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(4)} km/h`;
    $('speed-3d-inst').textContent = `${(speed3DInst * KMH_MS).toFixed(4)} km/h`; 
    $('accel-long').textContent = `${(accel_long).toFixed(3)} m/s ²`; 
    $('force-g-long').textContent = `${(accel_long / g_dynamic).toFixed(2)} G | Max: ${maxGForce.toFixed(2)} G`;
    $('distance-total-km').textContent = `${(distM/1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    
    // ... (autres mises à jour DOM) ...
    
    if (lat !== 0 && lon !== 0) updateMap(lat, lon); 
}

// --- HANDLERS CAPTEURS IMU ---
function handleDeviceMotion(event) {
    if (emergencyStopActive) return;

    const accel = event.accelerationIncludingGravity;
    latestAccelZ = accel.z || 0; 
    
    const linearAccel = event.acceleration;
    if (linearAccel) {
        // Magnitude de l'accélération linéaire 3D
        latestLinearAccelMagnitude = Math.sqrt(linearAccel.x*linearAccel.x + linearAccel.y*linearAccel.y + linearAccel.z*linearAccel.z);
    } else {
        latestLinearAccelMagnitude = 0;
    }
    
    const g_dynamic = G_ACC_LOCAL; 
    const totalGForce = latestLinearAccelMagnitude / g_dynamic; 

    if (totalGForce > maxGForce) maxGForce = totalGForce; 
    
    $('accel-vertical-imu').textContent = `${(latestAccelZ - g_dynamic).toFixed(3)} m/s²`;
    $('force-g-vertical').textContent = `${(latestAccelZ / g_dynamic).toFixed(2)} G`;

    savePrecisionRecords(); 
}

// ... (fonctions continueGPSStart, requestIMUPermissionAndStart, startGPS, stopGPS restent inchangées) ...

function continueGPSStart() {
    const opts = { enableHighAccuracy: currentGPSMode === 'HIGH_FREQ', timeout: 5000, maximumAge: 0 };
    
    if (wID !== null) navigator.geolocation.clearWatch(wID);

    wID = navigator.geolocation.watchPosition(updateDisp, (error) => {
        console.warn(`ERREUR GPS(${error.code}): ${error.message}`);
        if (error.code === 1) {
            alert("Accès à la géolocalisation refusé. Le tableau de bord ne peut fonctionner.");
        }
    }, opts);

    if ($('freq-select')) $('freq-select').value = currentGPSMode; 
    sTime = sTime === null ? getCDate() : sTime; 
}

/** Gère la demande de permission de l'IMU (nécessite un clic utilisateur) */
function requestIMUPermissionAndStart() {
    if (typeof DeviceOrientationEvent !== 'undefined' && typeof DeviceOrientationEvent.requestPermission === 'function') {
        
        DeviceOrientationEvent.requestPermission()
            .then(permissionState => {
                if (permissionState === 'granted') {
                    window.addEventListener('devicemotion', handleDeviceMotion, true);
                    console.log("Permission IMU accordée.");
                } else {
                    console.warn("Accès aux capteurs de mouvement refusé.");
                }
                continueGPSStart(); 
            })
            .catch(err => {
                console.error("Erreur d'autorisation DeviceMotion:", err);
                continueGPSStart(); 
            });
    } else {
        if (window.DeviceMotionEvent) {
             window.addEventListener('devicemotion', handleDeviceMotion, true);
        }
        continueGPSStart();
    }
}

function startGPS() {
    if (wID === null) {
        requestIMUPermissionAndStart(); 
    }
    $('toggle-gps-btn').textContent = ' 7œ4„1‚5 ARRÊT GPS';
    $('toggle-gps-btn').style.backgroundColor = '#dc3545';
}

function stopGPS() {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
    }
    $('toggle-gps-btn').textContent = ' 7œ4„1‚5 MARCHE GPS';
    $('toggle-gps-btn').style.backgroundColor = '#28a745';
        }
// =================================================================
// FICHIER 2/2 : gnss-dashboard-ui-init.js
// MODULES UI, ASTRO & INITIALISATION (Projection Stellarium, DOM)
// =================================================================

// Note: Ce fichier dépend des variables et fonctions globales définies dans gnss-dashboard-core.js

let map = null;
let marker = null;
const MAX_RADIUS_PX = 100; // Rayon max du disque Astro (à ajuster avec le CSS)

// ===========================================
// LOGIQUE ASTRO ZÉNITHALE ET COULEUR DU CIEL
// ===========================================

/** Convertit les coordonnées sphériques (Azimut, Élévation) en coordonnées X,Y pour le disque. */
function projectAstroPosition(pos) {
    // 1. Convertir les radians en degrés
    const altitudeDeg = pos.altitude * R2D; // Élévation
    let azimuthDeg = pos.azimuth * R2D;    // Azimut (0=Nord)

    // S'assurer que l'azimut est entre 0 et 360
    if (azimuthDeg < 0) azimuthDeg += 360; 

    // Si l'astre est sous l'horizon, le masquer
    if (altitudeDeg < 0) {
        return { x: NaN, y: NaN }; 
    }

    // 2. Calculer l'angle zénithal (distance au centre)
    // Zénith (90 deg) = 0 distance ; Horizon (0 deg) = MAX_RADIUS_PX distance
    const zenithAngle = 90 - altitudeDeg; 
    const distanceToCenter = MAX_RADIUS_PX * (zenithAngle / 90); 

    // 3. Calcul de l'Angle de Rotation sur le Disque (phi)
    // Nord (0 deg) doit correspondre à l'axe Y- (haut)
    const angleRotationDeg = azimuthDeg - 90; // Rotation pour aligner N=0 avec le Haut du cercle
    const angleRotationRad = angleRotationDeg * D2R; 
    
    // 4. Coordonnées Cartésiennes (X, Y)
    const x = distanceToCenter * Math.cos(angleRotationRad);
    const y = distanceToCenter * Math.sin(angleRotationRad);

    return { x: x, y: y };
}

/** Définit la couleur du ciel en fonction de l'élévation du Soleil. */
function getSkyColor(sunAltitudeDeg) {
    let skyColor = '#000000'; // Noir (Nuit astronomique)

    if (sunAltitudeDeg > 10) {
        skyColor = '#87CEEB'; // Jour (Light Sky Blue)
    } else if (sunAltitudeDeg > 0) {
        skyColor = '#00BFFF'; // Jour standard (Deep Sky Blue)
    } else if (sunAltitudeDeg > -6) {
        skyColor = '#FF8C00'; // Crépuscule Civil (Orange/Rose)
    } else if (sunAltitudeDeg > -12) {
        skyColor = '#483D8B'; // Crépuscule Nautique (Violet/Gris foncé)
    } else if (sunAltitudeDeg > -18) {
        skyColor = '#191970'; // Crépuscule Astronomique (Midnight Blue)
    } else {
        skyColor = '#000000'; // Nuit Astronomique (Noir)
    }
    return skyColor;
}

/** Calcule et affiche l'heure solaire vraie (TST), la position des astres et la couleur du ciel. */
function updateAstro(latitude, longitude) {
    if (typeof getCDate !== 'function' || typeof SunCalc === 'undefined') return;

    const now = getCDate();
    const times = SunCalc.getTimes(now, latitude, longitude);
    const pos = SunCalc.getPosition(now, latitude, longitude); // Position du Soleil
    const moonIllumination = SunCalc.getMoonIllumination(now);
    const moonPos = SunCalc.getMoonPosition(now, latitude, longitude); // Position de la Lune
    
    // --- NOUVEAU : MISE À JOUR DE LA COULEUR DU CIEL ---
    const sunAltitudeDeg = pos.altitude * R2D;
    const skyElement = $('sky-display'); // Assurez-vous d'avoir ce div dans votre HTML
    
    if (skyElement) {
        skyElement.style.backgroundColor = getSkyColor(sunAltitudeDeg);
    }
    
    // --- NOUVEAU : PROJECTION ZÉNITHALE DES ASTRES ---
    const sunCoords = projectAstroPosition(pos);
    const moonCoords = projectAstroPosition(moonPos);

    const sunEl = $('sun-element');
    const moonEl = $('moon-element');
    
    // Déplacer les éléments (centerOffset = MAX_RADIUS_PX pour centrer le disque)
    const centerOffset = MAX_RADIUS_PX; 

    if (!isNaN(sunCoords.x)) {
        sunEl.style.display = 'block';
        // Utilisation de translate pour positionner l'astre
        sunEl.style.transform = `translate(${sunCoords.x + centerOffset}px, ${sunCoords.y + centerOffset}px)`;
    } else {
        sunEl.style.display = 'none'; 
    }

    if (!isNaN(moonCoords.x)) {
        moonEl.style.display = 'block';
        moonEl.style.transform = `translate(${moonCoords.x + centerOffset}px, ${moonCoords.y + centerOffset}px)`;
    } else {
        moonEl.style.display = 'none';
    }

    // --- CALCUL TST (Heure Solaire Vraie) ---
    // (Cette logique reste la même pour l'affichage numérique du TST)
    let fractionalDay = (now.getTime() - times.solarMidnight.getTime()) / (86400000);
    let tstHours = fractionalDay * 24;
    tstHours = tstHours % 24;

    const h = Math.floor(tstHours);
    const m = Math.floor((tstHours - h) * 60);
    const s = Math.floor(((tstHours - h) * 60 - m) * 60);

    // Si lat/lon sont 0 (initialisation), affiche l'heure locale, sinon TST
    if (latitude === 0 && longitude === 0) {
        $('tst').textContent = now.toLocaleTimeString(); 
    } else {
        $('tst').textContent = `${h.toString().padStart(2, '0')}:${m.toString().padStart(2, '0')}:${s.toString().padStart(2, '0')}`;
    }
    
    // ... (Mise à jour des autres champs Astro - Noon, Duration, etc.) ...
    $('sun-elevation').textContent = `${sunAltitudeDeg.toFixed(2)} °`;
    $('noon-solar').textContent = times.solarNoon.toLocaleTimeString();
    $('day-duration').textContent = `${((times.sunset.getTime() - times.sunrise.getTime()) / 3600000).toFixed(2)} h`;
    $('moon-phase-display').textContent = `${(moonIllumination.phase * 100).toFixed(1)}% | ${getMoonPhaseName(moonIllumination.phase)}`;
}

/** Nomme la phase lunaire. */
function getMoonPhaseName(phase) {
    if (phase < 0.03 || phase > 0.97) return "Nouvelle Lune";
    if (phase < 0.28) return "Premier Quartier";
    if (phase < 0.53) return "Pleine Lune";
    if (phase < 0.78) return "Dernier Quartier";
    return "Croissant/Gibbeuse";
}

// --- MÉTÉO ---
// (Fonction getWeather reste inchangée - Nécessite la clé API)

function getWeather() {
    if (lat === 0 || lon === 0 || OWM_API_KEY === "VOTRE_CLE_API_OPENWEATHERMAP") {
        // ... (Messages API Requis) ...
        $('temp-air').textContent = 'API Requis';
        $('pressure').textContent = 'API Requis';
        $('humidity').textContent = 'API Requis';
        $('wind-speed-ms').textContent = 'API Requis';
        return;
    }
    
    // ... (Logique fetch inchangée) ...
    const url = `https://api.openweathermap.org/data/2.5/weather?lat=${lat}&lon=${lon}&appid=${OWM_API_KEY}&units=metric`;

    fetch(url)
        .then(response => response.json())
        .then(data => {
            if (data.main) {
                $('temp-air').textContent = `${data.main.temp.toFixed(1)} °C`;
                $('pressure').textContent = `${data.main.pressure.toFixed(0)} hPa`;
                $('humidity').textContent = `${data.main.humidity}%`;
                $('wind-speed-ms').textContent = `${data.wind.speed.toFixed(1)} m/s`;
            }
        })
        .catch(err => console.error("Erreur de récupération météo :", err));
}

// --- CARTE LEAFLET ---
// (Fonctions initMap et updateMap restent inchangées)

function initMap() {
    if (typeof L === 'undefined') {
        console.error("Leaflet n'est pas chargé.");
        return;
    }
    map = L.map('map-container').setView([0, 0], 2);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
    }).addTo(map);

    marker = L.marker([0, 0]).addTo(map)
        .bindPopup("Position Actuelle")
        .openPopup();
}

function updateMap(latitude, longitude) {
    if (map) {
        map.setView([latitude, longitude], map.getZoom() < 12 ? 12 : map.getZoom());
        marker.setLatLng([latitude, longitude]);
    }
}

// --- SYNCHRONISATION HORLOGE (NTP) ---
// (Fonction syncH reste inchangée)

function syncH() {
    fetch(SERVER_TIME_ENDPOINT)
        .then(response => response.json())
        .then(data => {
            const apiTime = new Date(data.utc_datetime).getTime();
            const localTime = new Date().getTime();
            gpsTS = apiTime + (localTime - apiTime); 
            sTime = new Date();
        })
        .catch(err => {
            console.warn("Échec de la synchronisation NTP, utilise l'heure locale.", err);
            sTime = new Date();
        });
}


// --- INITIALISATION FINALE DU SYSTÈME ET LISTENERS ---

document.addEventListener('DOMContentLoaded', () => {
    loadPrecisionRecords();
    initMap();
    syncH(); 
    
    updateCelestialBody($('celestial-body-select').value);
    
    // --- Intervalle de mise à jour lente DOM/Astro ---
    domID = setInterval(() => {
        const now = getCDate();
        if (now) {
            $('local-time').textContent = now.toLocaleTimeString();
            $('date-display').textContent = now.toLocaleDateString();
            $('time-elapsed').textContent = sTime ? ((now.getTime() - sTime.getTime()) / 1000).toFixed(2) + ' s' : '0.00 s';
            $('time-moving').textContent = timeMoving.toFixed(2) + ' s';
        }
        
        // Appelle updateAstro, qui gère la couleur du ciel et la projection des astres
        updateAstro(lat, lon); 
    }, 1000); 
    
    // --- Intervalle pour la mise à jour Météo (30s) ---
    weatherID = setInterval(getWeather, 30000); 

    // --- Événements Utilisateur (Restent inchangés) ---
    $('toggle-gps-btn').addEventListener('click', () => wID === null ? startGPS() : stopGPS());
    
    $('emergency-stop-btn').addEventListener('click', () => {
        emergencyStopActive = !emergencyStopActive;
        $('emergency-stop-btn').textContent = emergencyStopActive ? '•0“5 Arrêt d\'urgence: ACTIF •0 4' : '•0“5 Arrêt d\'urgence: INACTIF •0 4';
        $('emergency-stop-btn').style.backgroundColor = emergencyStopActive ? '#f8d7da' : '#dc3545';
        if (emergencyStopActive) stopGPS();
    });
    // ... (Autres listeners de boutons) ...
    $('reset-dist-btn').addEventListener('click', () => { distM = 0; timeMoving = 0; });
    $('reset-max-btn').addEventListener('click', () => { maxSpd = 0; maxGForce = 0; savePrecisionRecords(); });
    $('toggle-mode-btn').addEventListener('click', () => {
        document.body.classList.toggle('dark-mode');
        $('toggle-mode-btn').classList.toggle('active');
    });

    $('freq-select').addEventListener('change', (e) => {
        currentGPSMode = e.target.value;
        if (wID !== null) {
            stopGPS();
            startGPS();
        }
    });
});
