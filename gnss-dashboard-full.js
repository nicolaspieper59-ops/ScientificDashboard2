// =================================================================
// FICHIER 1/2 : gnss-dashboard-core.js
// CŒUR DU SYSTÈME (Variables, EKF, Capteurs, Logique GPS/IMU)
// =================================================================

const $ = (id) => document.getElementById(id);

// --- PARTIE 1 : CONSTANTES GLOBALES (Physiques, GPS, Temps, Astro) ---
const C_L = 299792458; // Vitesse de la lumière (m/s)
const SPEED_SOUND = 343; // Vitesse du son (m/s)
const G_ACC_STD = 9.80665; // Gravité standard de la Terre (m/s²)
const M_EARTH = 5.972e24; // Masse de la Terre (kg)
const G_CONST = 6.67430e-11; // Constante gravitationnelle (N(m/kg)²)
const R_E = 6371000; // Rayon moyen de la Terre (m)
const R2D = 180 / Math.PI;
const D2R = Math.PI / 180;
const NETHER_RATIO = 1 / 8; 

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

// Constantes Planétaires
const G_SURFACE_MOON = 1.625; const R_MOON = 1737400; const M_MOON = 7.34767e22; 
const G_SURFACE_MARS = 3.721; const R_MARS = 3389500; const M_MARS = 6.4171e23; 

// Constantes GPS et EKF
const MIN_DT = 0.05; 
const Q_NOISE = 0.001; 
const MIN_SPD = 0.001; 
const MIN_UNCERT_FLOOR = Q_NOISE * MIN_DT; 
const OWM_API_KEY = "VOTRE_CLE_API_OPENWEATHERMAP"; // À REMPLACER
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
// CŒUR EKF, ZVU ET MISE À JOUR (updateDisp)
// ===========================================

/** Simule la correction GPS théorique parfaite. */
function simulateBestCorrection() {
    if (lat === 0 && lon === 0) {
        kUncert = MIN_UNCERT_FLOOR; 
        kAltUncert = MIN_UNCERT_FLOOR; 
        return; 
    }
    kUncert = MIN_UNCERT_FLOOR; 
    kAltUncert = MIN_UNCERT_FLOOR; 
    
    const mockBestCorrectionPos = {
        coords: { latitude: lat, longitude: lon, altitude: kAlt, accuracy: 0.00001, speed: kSpd, altitudeAccuracy: 0.00001 },
        timestamp: new Date().getTime()
    };
    updateDisp(mockBestCorrectionPos); 
    
    console.log(`Simulateur de Correction Théorique activé : EKF réglé sur l'incertitude minimale (${MIN_UNCERT_FLOOR.toExponential(2)} m²).`);
}

/** Handler des données GPS et du filtre EKF. */
function updateDisp(pos) {
    if (emergencyStopActive) return;
    
    // Les fonctions updateMap et updateAstro sont définies dans gnss-dashboard-ui-init.js
    if (typeof updateMap !== 'function') {
        console.warn("updateDisp() appelé avant le chargement complet des modules UI. Saut...");
        return;
    }

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
        speedRaw = d_horiz_raw / dt; // Vitesse = distance / temps
    } else {
        speedRaw = pos.coords.speed !== null ? pos.coords.speed : 0; 
    }

    const currentHeading = pos.coords.heading !== null && !isNaN(pos.coords.heading) ? pos.coords.heading : 'N/A';
    lastTS = nowTS;

    const g_dynamic = calculateLocalGravity(currentAlt);
    $('gravity-local').textContent = `${g_dynamic.toFixed(5)} m/s²`;
    
    // --- EKF (Filtre de Kalman pour la vitesse 2D) ---
    let kR = acc * acc * getEnvironmentFactor(); 
    let kGain = kUncert / (kUncert + kR);
    kSpd = kSpd + kGain * (speedRaw - kSpd); 
    kUncert = (1 - kGain) * kUncert;
    kUncert = Math.max(kUncert, MIN_UNCERT_FLOOR); 
    
    // --- ZVU ÉTALONNAGE ---
    if (speedRaw < MIN_SPD * 10 && kSpd < MIN_SPD) { 
        kSpd = 0; 
    }
    
    // --- ACCÉLÉRATION & G-FORCE (Fusion Hybride IMU/EKF) ---
    let sSpdFE = Math.abs(kSpd);
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    
    const accel_ekf = (dt > MIN_DT) ? (sSpdFE - lastFSpeed) / dt : 0;
    
    let accel_imu_signed = 0;
    const IMU_ACCEL_THRESHOLD = 0.1; 
    const FUSION_FACTOR = 0.7; 

    if (latestLinearAccelMagnitude > IMU_ACCEL_THRESHOLD) { 
        accel_imu_signed = latestLinearAccelMagnitude * Math.sign(accel_ekf || 1); 
    }

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
    $('speed-stable-kms').textContent = `${(sSpdFE / KMS_MS).toFixed(7)} km/s`; 
    $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(4)} km/h`;
    
    const avgSpdMoving = timeMoving > 0 ? (distM / timeMoving) : 0;
    $('speed-avg-moving').textContent = `${(avgSpdMoving * KMH_MS).toFixed(4)} km/h`;
    $('speed-stable-ms').textContent = `${sSpdFE.toFixed(3)} m/s | ${(sSpdFE * 1e6).toFixed(0)} µm/s | ${(sSpdFE * 1e9).toFixed(0)} nm/s`; 
    $('speed-3d-inst').textContent = `${(speed3DInst * KMH_MS).toFixed(4)} km/h`; 
    $('vertical-speed').textContent = `${verticalSpeedRaw.toFixed(2)} m/s`;
    
    $('perc-speed-c').textContent = `${(sSpdFE / C_L * 100).toExponential(2)}%`;
    $('perc-speed-sound').textContent = `${(sSpdFE / SPEED_SOUND * 100).toFixed(2)}%`;
    $('accel-long').textContent = `${(accel_long).toFixed(3)} m/s ²`; 
    $('force-g-long').textContent = `${(accel_long / g_dynamic).toFixed(2)} G | Max: ${maxGForce.toFixed(2)} G`;
    $('distance-total-km').textContent = `${(distM/1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    
    const distLightSeconds = distM / C_L;
    $('distance-light-s').textContent = `${toReadableScientific(distLightSeconds / SEC_LIGHT)} s lumière`;
    $('distance-light-min').textContent = `${toReadableScientific(distLightSeconds / MIN_LIGHT)} min lumière`;
    $('distance-light-h').textContent = `${toReadableScientific(distLightSeconds / HOUR_LIGHT)} h lumière`;
    $('distance-light-day').textContent = `${toReadableScientific(distLightSeconds / DAY_LIGHT)} j lumière`;
    $('distance-light-week').textContent = `${toReadableScientific(distLightSeconds / WEEK_LIGHT)} sem lumière`;
    $('distance-light-month').textContent = `${toReadableScientific(distLightSeconds / MONTH_LIGHT)} mois lumière`;
    const distAU = distM / AU_TO_M;
    const distLightYears = distM / LIGHT_YEAR_TO_M;
    $('distance-cosmic').textContent = `${toReadableScientific(distAU)} UA | ${toReadableScientific(distLightYears)} al`;
    
    $('latitude').textContent = lat.toFixed(6);
    $('longitude').textContent = lon.toFixed(6);
    $('altitude-gps').textContent = currentAlt !== null ? `${currentAlt.toFixed(2)} m` : 'N/A';
    $('gps-precision').textContent = acc !== null ? `${acc.toFixed(3)} m` : 'N/A';
    $('speed-raw-ms').textContent = speedRaw !== null ? `${speedRaw.toFixed(3)} m/s` : 'N/A';
    $('heading-display').textContent = currentHeading !== 'N/A' ? `${currentHeading.toFixed(1)} °` : 'N/A';
    $('underground-status').textContent = currentAlt !== null && currentAlt < -50 ? 'OUI' : 'Non';

    const mass = parseFloat($('mass-display').textContent) || 70;
    $('kinetic-energy').textContent = `${(0.5 * mass * sSpdFE * sSpdFE).toFixed(2)} J`;
    $('mechanical-power').textContent = `${(mass * accel_long * sSpdFE).toFixed(2)} W`;

    if (acc !== null && acc < P_RECORDS.max_acc_min) P_RECORDS.max_acc_min = acc;
    if (kUncert < P_RECORDS.max_kUncert_min) P_RECORDS.max_kUncert_min = kUncert;
    savePrecisionRecords();

    if (lat !== 0 && lon !== 0) updateMap(lat, lon); 
}

// --- HANDLERS CAPTEURS IMU ---
function handleDeviceMotion(event) {
    if (emergencyStopActive) return;

    const accel = event.accelerationIncludingGravity;
    latestAccelZ = accel.z || 0; 
    
    const linearAccel = event.acceleration;
    if (linearAccel) {
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

// --- LOGIQUE GPS ET DÉMARRAGE ---

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
    // Force la fenêtre d'autorisation IMU si la méthode est supportée (iOS Safari/Chrome récent)
    if (typeof DeviceOrientationEvent !== 'undefined' && typeof DeviceOrientationEvent.requestPermission === 'function') {
        
        DeviceOrientationEvent.requestPermission()
            .then(permissionState => {
                if (permissionState === 'granted') {
                    // SI LA PERMISSION EST ACCORDÉE : On attache l'écouteur de mouvement
                    window.addEventListener('devicemotion', handleDeviceMotion, true);
                    console.log("Permission IMU (Accélération Linéaire) accordée.");
                } else {
                    console.warn("Accès aux capteurs de mouvement refusé. Accélération horizontale IMU inactive.");
                }
                continueGPSStart(); 
            })
            .catch(err => {
                console.error("Erreur d'autorisation DeviceMotion:", err);
                continueGPSStart(); 
            });
    } else {
        // Ancien navigateur/Plateforme où la permission n'est pas nécessaire
        if (window.DeviceMotionEvent) {
             window.addEventListener('devicemotion', handleDeviceMotion, true);
        }
        continueGPSStart();
    }
}

function startGPS() {
    if (wID === null) {
        // Déclencheur du clic utilisateur pour forcer la fenêtre d'autorisation IMU
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
// MODULES UI & INITIALISATION (Astro, Carte, DOM)
// =================================================================

// Note: Ce fichier dépend des variables et fonctions globales définies dans gnss-dashboard-core.js

let map = null;
let marker = null;

/** Calcule et affiche l'heure solaire vraie (TST) et l'animation astro. */
function updateAstro(latitude, longitude) {
    if (latitude === 0 && longitude === 0 || typeof getCDate !== 'function' || typeof SunCalc === 'undefined') return;

    const now = getCDate();
    const times = SunCalc.getTimes(now, latitude, longitude);
    const pos = SunCalc.getPosition(now, latitude, longitude);
    const moonIllumination = SunCalc.getMoonIllumination(now);
    const moonPos = SunCalc.getMoonPosition(now, latitude, longitude);

    // 1. Calcul de l'Heure Solaire Vraie (TST)
    let fractionalDay = (now.getTime() - times.solarMidnight.getTime()) / (86400000);
    let tstHours = fractionalDay * 24;
    tstHours = tstHours % 24;

    const h = Math.floor(tstHours);
    const m = Math.floor((tstHours - h) * 60);
    const s = Math.floor(((tstHours - h) * 60 - m) * 60);

    $('tst').textContent = `${h.toString().padStart(2, '0')}:${m.toString().padStart(2, '0')}:${s.toString().padStart(2, '0')}`;
    $('time-minecraft').textContent = $('tst').textContent; 

    // 2. Animation Jour/Nuit
    let clockRotation = (tstHours / 24) * 360; 
    
    const sunEl = $('sun-element');
    const moonEl = $('moon-element');

    sunEl.style.transform = `rotate(${clockRotation - 90}deg)`; 
    moonEl.style.transform = `rotate(${(moonPos.azimuth + Math.PI) * R2D - 90}deg)`; 

    const sunAltDeg = pos.altitude * R2D;
    let skyClass = '';
    const elevationThreshold = 0; 
    const isNight = sunAltDeg < elevationThreshold;

    if (!isNight) {
        skyClass = sunAltDeg > 20 ? 'sky-day' : 'sky-sunset'; 
    } else {
        skyClass = sunAltDeg > -12 ? 'sky-night-light' : 'sky-night'; 
    }
    if (!$('toggle-mode-btn').classList.contains('active')) {
        document.body.className = skyClass;
    }

    // 3. Affichages Astro supplémentaires
    $('sun-elevation').textContent = `${sunAltDeg.toFixed(2)} °`;
    $('noon-solar').textContent = times.solarNoon.toLocaleTimeString();
    $('day-duration').textContent = `${((times.sunset.getTime() - times.sunrise.getTime()) / 3600000).toFixed(2)} h`;
    $('moon-phase-display').textContent = `${(moonIllumination.phase * 100).toFixed(1)}% | ${getMoonPhaseName(moonIllumination.phase)}`;
    $('ecliptic-long').textContent = `${(pos.eclipticLng * R2D).toFixed(2)} °`;
    
    const meanSolarTime = (now.getTime() - times.solarMidnight.getTime()) / (86400000) * 24;
    const tstSec = h * 3600 + m * 60 + s;
    const lstSec = (meanSolarTime % 24) * 3600;
    const eotMin = (tstSec - lstSec) / 60;
    $('eot').textContent = `${eotMin.toFixed(2)} min`;
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

function getWeather() {
    if (lat === 0 || lon === 0 || OWM_API_KEY === "VOTRE_CLE_API_OPENWEATHERMAP") {
        $('temp-air').textContent = 'API Requis';
        $('pressure').textContent = 'API Requis';
        $('humidity').textContent = 'API Requis';
        $('wind-speed-ms').textContent = 'API Requis';
        return;
    }
    
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
        
        if (lat !== 0 && lon !== 0) updateAstro(lat, lon); 
    }, 1000); 
    
    // --- Intervalle pour la mise à jour Météo (30s) ---
    weatherID = setInterval(getWeather, 30000); 

    // --- Événements Utilisateur ---
    // Clic qui déclenche la demande de permission IMU dans startGPS()
    $('toggle-gps-btn').addEventListener('click', () => wID === null ? startGPS() : stopGPS());
    
    $('emergency-stop-btn').addEventListener('click', () => {
        emergencyStopActive = !emergencyStopActive;
        $('emergency-stop-btn').textContent = emergencyStopActive ? '•0“5 Arrêt d\'urgence: ACTIF •0 4' : '•0“5 Arrêt d\'urgence: INACTIF •0 4';
        $('emergency-stop-btn').style.backgroundColor = emergencyStopActive ? '#f8d7da' : '#dc3545';
        if (emergencyStopActive) stopGPS();
    });
    $('nether-toggle-btn').addEventListener('click', () => {
        const isActive = $('mode-nether').textContent.includes('ACTIF');
        $('mode-nether').textContent = isActive ? 'DÉSACTIVÉ (1:1)' : `ACTIF (1:${1/NETHER_RAT})`;
    });

    $('reset-dist-btn').addEventListener('click', () => { distM = 0; timeMoving = 0; });
    $('reset-max-btn').addEventListener('click', () => { maxSpd = 0; maxGForce = 0; savePrecisionRecords(); });
    $('reset-all-btn').addEventListener('click', () => { 
        if (confirm("Êtes-vous sûr de vouloir tout réinitialiser (dist/max/records) ?")) {
            localStorage.removeItem(P_RECORDS_KEY);
            distM = 0; timeMoving = 0; maxSpd = 0; maxGForce = 0;
            kSpd = 0; kUncert = 1000; kAltUncert = 1000; 
            location.reload(); 
        }
    });

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
