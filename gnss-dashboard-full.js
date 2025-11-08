// =================================================================
// FICHIER 1/2 (BLOC 1/2) : gnss-dashboard-core-part1.js
// Contient constantes, variables d'état, fonctions utilitaires et gravité dynamique.
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

// Constantes Planétaires (Ajout de la masse/rayon pour la gravité dynamique)
const G_SURFACE_MOON = 1.625; const R_MOON = 1737400; const M_MOON = 7.34767e22; 
const G_SURFACE_MARS = 3.721; const R_MARS = 3389500; const M_MARS = 6.4171e23; 

// Constantes GPS et EKF (Plancher d'Incertitude EKF)
const MIN_DT = 0.05; 
const Q_NOISE = 0.001; 
const MIN_SPD = 0.001; 
const MIN_UNCERT_FLOOR = Q_NOISE * MIN_DT; 
const OWM_API_KEY = "VOTRE_CLE_API_OPENWEATHERMAP"; 
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc"; 

// --- VARIABLES GLOBALES (EKF & État) ---
let lat = 0, lon = 0, alt = 0, speed = 0, gpsTS = 0;
let kSpd = 0, kUncert = 1000, kAlt = 0, kAltUncert = 1000;
let lastTS = 0, lastFSpeed = 0, distM = 0;
let lastPos = null, lastAlt = 0; 
let sTime = null, timeMoving = 0, maxSpd = 0; 
let latestLinearAccelMagnitude = 0, maxGForce = 0; 
let latestRotationBeta = 0, latestRotationGamma = 0; 
let wID = null, domID = null, weatherID = null;
let currentGPSMode = 'HIGH_FREQ'; 
let emergencyStopActive = false;
let currentCelestialBody = 'EARTH'; 
let G_ACC_LOCAL = G_ACC_STD; // Gravité locale dynamique
let netherMode = false; 

// ===========================================
// FONCTIONS UTILITAIRES ET GRAVITÉ DYNAMIQUE
// ===========================================

/** Calcule la gravité locale en fonction du corps céleste et de l'altitude (Fonction de l'altitude). */
function calculateLocalGravity(altitude) {
    let R, M, g_surface;
    switch (currentCelestialBody) {
        case 'MOON': R = R_MOON; M = M_MOON; g_surface = G_SURFACE_MOON; break;
        case 'MARS': R = R_MARS; M = M_MARS; g_surface = G_SURFACE_MARS; break;
        case 'EARTH': default: R = R_E; M = M_EARTH; g_surface = G_ACC_STD; break;
    }
    
    // Gravité en fonction de l'altitude: G * M / (R + h)^2
    if (altitude === null || isNaN(altitude) || currentCelestialBody !== 'EARTH') {
        G_ACC_LOCAL = g_surface; 
    } else {
        const h = altitude;
        G_ACC_LOCAL = G_CONST * M / Math.pow(R + h, 2);
    }
    
    if($('gravity-local')) $('gravity-local').textContent = `${G_ACC_LOCAL.toFixed(5)} m/s²`;
    return G_ACC_LOCAL;
}

function updateCelestialBody(body) {
    currentCelestialBody = body;
    calculateLocalGravity(alt); 
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
    if (num === 0) return "0.00e+00";
    if (isNaN(num) || !isFinite(num)) return "N/A";
    const exponent = Math.floor(Math.log10(Math.abs(num)));
    const mantissa = num / Math.pow(10, exponent);
    return `${mantissa.toFixed(2)}e${exponent >= 0 ? '+' : ''}${exponent}`;
}

function getCDate() {
    return sTime ? new Date(new Date().getTime() - (sTime.getTime() - gpsTS)) : new Date();
        }
// =================================================================
// FICHIER 1/2 (BLOC 2/2) : gnss-dashboard-core-part2.js
// Contient le Cœur EKF, l'IMU, la logique GPS, et les mises à jour DOM.
// ASSUME QUE LE BLOC 1/2 EST DÉJÀ CHARGÉ (accès aux variables et fonctions).
// =================================================================

// ===========================================
// CŒUR EKF, ZVU ET MISE À JOUR (updateDisp)
// ===========================================

function updateDisp(pos) {
    // Vérification de l'arrêt d'urgence et de la fonction updateMap (définie dans gnss-dashboard-ui-init.js)
    if (emergencyStopActive || typeof updateMap !== 'function') return;

    const acc = $('gps-accuracy-override').value !== '0.000000' ? parseFloat($('gps-accuracy-override').value) : pos.coords.accuracy;

    lat = pos.coords.latitude; 
    lon = pos.coords.longitude;
    const currentAlt = pos.coords.altitude; 
    alt = currentAlt; 
    gpsTS = pos.timestamp;
    
    const nowTS = getCDate().getTime();
    const dt = lastTS === 0 ? MIN_DT : (nowTS - lastTS) / 1000;
    
    if (dt <= 0) return; 

    // 1. CALCUL VITESSE BRUTE (GPS)
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
    
    // 2. DÉTERMINATION DE L'ACCÉLÉRATION IMU (POUR PRÉDICTION)
    const accel_ekf_estimate = (dt > MIN_DT) ? (kSpd - lastFSpeed) / dt : 0;
    let accel_imu_sign = Math.sign(accel_ekf_estimate || speedRaw || kSpd || 1);
    const accel_imu_signed = latestLinearAccelMagnitude * accel_imu_sign;

    // --- EKF (AVEC PRÉDICTION IMU pour le Dead Reckoning) ---
    
    // 3. ÉTAPE DE PRÉDICTION 
    let kSpd_pred = kSpd + (accel_imu_signed * dt);
    let kUncert_pred = kUncert + Q_NOISE * dt; 

    // 4. ÉTAPE DE CORRECTION (GPS)
    let kR = acc * acc * getEnvironmentFactor(); 
    let kGain = kUncert_pred / (kUncert_pred + kR);
    
    kSpd = kSpd_pred + kGain * (speedRaw - kSpd_pred);
    kUncert = (1 - kGain) * kUncert_pred;
    
    kUncert = Math.max(kUncert, MIN_UNCERT_FLOOR); // Plancher d'Incertitude EKF
    
    // 5. ZVU (Mise à jour Zéro Vitesse)
    if (speedRaw < 0.1 && latestLinearAccelMagnitude < 0.1) { 
        kSpd = 0; 
        kUncert = 1; 
    }
    
    // --- CALCULS FINAUX ---
    let sSpdFE = Math.abs(kSpd);
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    
    // Vitesse Verticale
    let verticalSpeedRaw = 0;
    if (currentAlt !== null && lastAlt !== 0 && dt > 0) {
        verticalSpeedRaw = (currentAlt - lastAlt) / dt;
    }

    // Accélération Longitudinale
    const accel_long = (dt > MIN_DT) ? (sSpdFE - lastFSpeed) / dt : 0;
    lastFSpeed = sSpdFE; 
    
    // Vitesse 3D (Instantanée)
    let speed3DInst = Math.sqrt(sSpdFE * sSpdFE + verticalSpeedRaw * verticalSpeedRaw);
    
    // Distance Totale (Distance 3D)
    if (lastPos && sSpdFE > MIN_SPD) { 
        const d_horiz_ekf = sSpdFE * dt;
        let d_vert = (verticalSpeedRaw) * dt;
        const d_3d = Math.sqrt(d_horiz_ekf * d_horiz_ekf + d_vert * d_vert); 
        distM += d_3d * (netherMode ? NETHER_RATIO : 1); 
        timeMoving += dt;
    }
    
    if (currentAlt !== null) lastAlt = currentAlt; 
    lastPos = { latitude: lat, longitude: lon };

    // --- MISE À JOUR DU DOM (Données principales) ---
    
    // Vitesse (Vitesse en km/s et nm/s)
    $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(4)} km/h`;
    $('speed-stable-kms').textContent = `${(sSpdFE / KMS_MS).toFixed(7)} km/s`; 
    $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(4)} km/h`; 
    const avgSpdMoving = timeMoving > 0 ? (distM / timeMoving) : 0;
    $('speed-avg-moving').textContent = `${(avgSpdMoving * KMH_MS).toFixed(4)} km/h`; 
    $('speed-stable-ms').textContent = `${sSpdFE.toFixed(3)} m/s | ${(sSpdFE * 1e6).toFixed(0)} µm/s | ${(sSpdFE * 1e9).toFixed(0)} nm/s`; 
    $('speed-3d-inst').textContent = `${(speed3DInst * KMH_MS).toFixed(4)} km/h`; 
    
    // % Vitesse
    $('perc-speed-sound').textContent = `${((sSpdFE / SPEED_SOUND) * 100).toFixed(2)} %`;
    $('perc-speed-c').textContent = `${((sSpdFE / C_L) * 100).toExponential(2)} %`;

    // Distance Totale (3D)
    $('distance-total-km').textContent = `${(distM/1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    
    // Distances Cosmologiques (Ajout des unités de temps-lumière et UA)
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
    
    // Accélération et Forces
    $('accel-long').textContent = `${(accel_long).toFixed(3)} m/s ²`; 
    $('force-g-long').textContent = `${(accel_long / g_dynamic).toFixed(2)} G`; 
    $('vertical-speed').textContent = `${verticalSpeedRaw.toFixed(2)} m/s`;
    
    // Énergie (Corrigé)
    const mass = parseFloat($('mass-display').textContent) || 70;
    $('kinetic-energy').textContent = `${(0.5 * mass * sSpdFE * sSpdFE).toFixed(2)} J`;
    $('mechanical-power').textContent = `${(mass * accel_long * sSpdFE).toFixed(2)} W`;

    // Vue Brute GPS
    $('latitude').textContent = lat.toFixed(6);
    $('longitude').textContent = lon.toFixed(6);
    $('altitude-gps').textContent = currentAlt !== null ? `${currentAlt.toFixed(2)} m` : 'N/A';
    $('gps-precision').textContent = acc !== null ? `${acc.toFixed(3)} m` : 'N/A';
    $('speed-raw-ms').textContent = speedRaw !== null ? `${speedRaw.toFixed(3)} m/s` : 'N/A';
    $('heading-display').textContent = currentHeading !== 'N/A' ? `${currentHeading.toFixed(1)} °` : 'N/A';
    $('underground-status').textContent = currentAlt !== null && currentAlt < -50 ? 'OUI' : 'Non';

    if (lat !== 0 && lon !== 0) updateMap(lat, lon); 
}

// ===========================================
// HANDLERS CAPTEURS IMU 
// ===========================================

function handleDeviceMotion(event) {
    if (emergencyStopActive) return;

    const linearAccel = event.acceleration;
    
    if (linearAccel && linearAccel.x !== null) {
        // Magnitude 3D (pour fusion EKF horizontale/longitudinale)
        latestLinearAccelMagnitude = Math.sqrt(linearAccel.x*linearAccel.x + linearAccel.y*linearAccel.y + linearAccel.z*linearAccel.z);
        
        // Accélération Z linéaire
        const linearZ = linearAccel.z;
        $('accel-vertical-imu').textContent = `${linearZ.toFixed(3)} m/s ²`;
        $('force-g-vertical').textContent = `${(linearZ / G_ACC_LOCAL).toFixed(2)} G`;

    } else {
        latestLinearAccelMagnitude = 0; 
        $('accel-vertical-imu').textContent = 'N/A';
        $('force-g-vertical').textContent = 'N/A';
    }
}

function handleDeviceOrientation(event) {
    if (emergencyStopActive) return;
    latestRotationBeta = event.beta || 0;   // Tangage (Pitch)
    latestRotationGamma = event.gamma || 0; // Roulis (Roll)
    
    // Mise à jour du DOM (Niveau à bulle)
    if($('pitch-angle')) $('pitch-angle').textContent = `${latestRotationBeta.toFixed(1)} °`;
    if($('roll-angle')) $('roll-angle').textContent = `${latestRotationGamma.toFixed(1)} °`;
}

// ===========================================
// LOGIQUE GPS ET DÉMARRAGE 
// ===========================================

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

/** Gère la demande de permission de l'IMU (Accélération ET Orientation) */
function requestIMUPermissionAndStart() {
    let motionGranted = false;
    let orientationGranted = false;

    const startListeners = () => {
        if (motionGranted) {
            window.addEventListener('devicemotion', handleDeviceMotion, true);
        }
        if (orientationGranted) {
            window.addEventListener('deviceorientation', handleDeviceOrientation, true);
        }
        continueGPSStart();
    };

    const requestMotion = () => {
        if (typeof DeviceMotionEvent !== 'undefined' && typeof DeviceMotionEvent.requestPermission === 'function') {
            DeviceMotionEvent.requestPermission()
                .then(permissionState => {
                    motionGranted = (permissionState === 'granted');
                    requestOrientation();
                })
                .catch(() => requestOrientation());
        } else {
            motionGranted = true; 
            requestOrientation();
        }
    };

    const requestOrientation = () => {
        if (typeof DeviceOrientationEvent !== 'undefined' && typeof DeviceOrientationEvent.requestPermission === 'function') {
            DeviceOrientationEvent.requestPermission()
                .then(permissionState => {
                    orientationGranted = (permissionState === 'granted');
                    startListeners(); 
                })
                .catch(() => startListeners());
        } else {
            orientationGranted = true; 
            startListeners();
        }
    };

    requestMotion();
}

function startGPS() {
    if (wID === null) {
        requestIMUPermissionAndStart(); 
    }
    $('toggle-gps-btn').textContent = ' 7œ4„1‚5 ARRÊT GPS';
    $('toggle-gps-btn').style.backgroundColor = '#dc3545';
}

function stopGPS() {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    wID = null;
    
    window.removeEventListener('devicemotion', handleDeviceMotion, true);
    window.removeEventListener('deviceorientation', handleDeviceOrientation, true);

    $('toggle-gps-btn').textContent = ' 7œ4„1‚5 MARCHE GPS';
    $('toggle-gps-btn').style.backgroundColor = '#28a745';
        }
// =================================================================
// FICHIER 2/2 : gnss-dashboard-ui-init.js
// MODULES UI & INITIALISATION (Astro, Carte, DOM)
// =================================================================

// Dépendances de gnss-dashboard-core.js: $, getCDate, lat, lon, R2D, NETHER_RATIO

let map = null;
let marker = null;

/** Calcule et affiche l'heure solaire vraie (TST) et l'animation astro. */
function updateAstro(latitude, longitude) {
    if (latitude === 0 && longitude === 0 || typeof getCDate !== 'function' || typeof SunCalc === 'undefined') return;

    const now = getCDate();
    if (!now) return; 
    
    const times = SunCalc.getTimes(now, latitude, longitude);
    const pos = SunCalc.getPosition(now, latitude, longitude);
    const moonIllumination = SunCalc.getMoonIllumination(now);

    // 1. Calcul de l'Heure Solaire Vraie (TST)
    let fractionalDay = (now.getTime() - times.solarMidnight.getTime()) / (86400000);
    let tstHours = fractionalDay * 24;
    tstHours = tstHours % 24;

    const h = Math.floor(tstHours);
    const m = Math.floor((tstHours - h) * 60);
    const s = Math.floor(((tstHours - h) * 60 - m) * 60);

    // Affichage TST pour l'horloge
    if($('time-minecraft')) $('time-minecraft').textContent = `${h.toString().padStart(2, '0')}:${m.toString().padStart(2, '0')}:${s.toString().padStart(2, '0')}`;
    if($('tst')) $('tst').textContent = `${h.toString().padStart(2, '0')}:${m.toString().padStart(2, '0')}:${s.toString().padStart(2, '0')}`;

    // 2. Animation Horloge (Style Minecraft TST)
    
    // Logique de rotation TST: 12h TST (Midi Solaire / Zénith) = 0° (Top)
    let clockRotation = ((tstHours - 12) / 24) * 360; 
    clockRotation = (clockRotation % 360 + 360) % 360; // Assure que l'angle est entre 0 et 360
    
    const sunEl = $('sun-element');
    const moonEl = $('moon-element');
    const moonRotation = (clockRotation + 180) % 360; // La lune est à l'opposition du soleil
    

    if(sunEl) sunEl.style.transform = `rotate(${clockRotation}deg)`; 
    if(moonEl) moonEl.style.transform = `rotate(${moonRotation}deg)`; 

    // 3. Logique de Couleur du Ciel
    const sunAltDeg = pos.altitude * R2D;
    let skyClass = '';
    const elevationThreshold = 0; 
    const isNight = sunAltDeg < elevationThreshold;

    if (!isNight) {
        skyClass = sunAltDeg > 20 ? 'sky-day' : 'sky-sunset'; 
    } else {
        skyClass = sunAltDeg > -12 ? 'sky-night-light' : 'sky-night'; 
    }
    
    if ($('toggle-mode-btn') && !$('toggle-mode-btn').classList.contains('active')) {
        document.body.className = skyClass;
    }

    // 4. Affichages Astro supplémentaires
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

    $('date-display-astro').textContent = now.toLocaleDateString();
}

/** Nomme la phase lunaire. */
function getMoonPhaseName(phase) {
    if (phase < 0.03 || phase > 0.97) return "Nouvelle Lune";
    if (phase < 0.28) return "Premier Quartier";
    if (phase < 0.53) return "Pleine Lune";
    if (phase < 0.78) return "Dernier Quartier";
    return "Croissant/Gibbeuse";
}

// --- MÉTÉO (Simulation d'API) ---

function getWeather() {
    if (lat === 0 || lon === 0 || OWM_API_KEY === "VOTRE_CLE_API_OPENWEATHERMAP") {
        $('temp-air').textContent = 'API Requis';
        $('pressure').textContent = 'API Requis';
        $('humidity').textContent = 'API Requis';
        $('wind-speed-ms').textContent = 'API Requis';
        return;
    }
    
    // Logique d'appel API réelle
    const url = `https://api.openweathermap.org/data/2.5/weather?lat=${lat}&lon=${lon}&appid=${OWM_API_KEY}&units=metric`;

    fetch(url)
        .then(response => response.json())
        .then(data => {
            if (data.main) {
                $('temp-air').textContent = `${data.main.temp.toFixed(1)} °C`;
                $('pressure').textContent = `${data.main.pressure.toFixed(0)} hPa`;
                $('humidity').textContent = `${data.main.humidity}%`;
                if(data.wind) $('wind-speed-ms').textContent = `${data.wind.speed.toFixed(1)} m/s`;
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
            // Utilise l'heure NTP corrigée pour gpsTS (point de référence)
            gpsTS = apiTime + (localTime - apiTime); 
            sTime = new Date(); // L'heure locale de la synchronisation
        })
        .catch(err => {
            console.warn("Échec de la synchronisation NTP, utilise l'heure locale.", err);
            sTime = new Date();
        });
}


// --- INITIALISATION FINALE DU SYSTÈME ET LISTENERS ---

document.addEventListener('DOMContentLoaded', () => {
    initMap();
    syncH(); 
    
    if($('celestial-body-select')) updateCelestialBody($('celestial-body-select').value);
    
    // --- Intervalle de mise à jour lente DOM/Astro (1s) ---
    domID = setInterval(() => {
        const now = getCDate();
        if (now) {
            $('local-time').textContent = now.toLocaleTimeString();
            $('date-display').textContent = now.toLocaleDateString();
            if(sTime) $('time-elapsed').textContent = ((now.getTime() - sTime.getTime()) / 1000).toFixed(2) + ' s';
            $('time-moving').textContent = timeMoving.toFixed(2) + ' s';
        }
        
        updateAstro(lat, lon); 
        
    }, 1000); 
    
    // --- Intervalle pour la mise à jour Météo (30s) ---
    weatherID = setInterval(getWeather, 30000); 

    // --- Événements Utilisateur ---
    $('toggle-gps-btn').addEventListener('click', () => wID === null ? startGPS() : stopGPS());
    
    $('emergency-stop-btn').addEventListener('click', () => {
        emergencyStopActive = !emergencyStopActive;
        $('emergency-stop-btn').textContent = emergencyStopActive ? '•0“5 Arrêt d\'urgence: ACTIF •0 4' : '•0“5 Arrêt d\'urgence: INACTIF •0 4';
        $('emergency-stop-btn').style.backgroundColor = emergencyStopActive ? '#f8d7da' : '#dc3545';
        if (emergencyStopActive) stopGPS();
    });
    
    if($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', () => {
        netherMode = !netherMode; 
        $('mode-nether').textContent = netherMode ? `ACTIF (1:${1/NETHER_RATIO})` : 'DÉSACTIVÉ (1:1)';
    });

    if($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => { distM = 0; timeMoving = 0; });
    if($('reset-max-btn')) $('reset-max-btn').addEventListener('click', () => { maxSpd = 0; maxGForce = 0; });
    
    if($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { 
        if (confirm("Êtes-vous sûr de vouloir tout réinitialiser (dist/max/records) ?")) {
            distM = 0; timeMoving = 0; maxSpd = 0; maxGForce = 0;
            kSpd = 0; kUncert = 1000; kAltUncert = 1000; 
            location.reload(); 
        }
    });

    if($('toggle-mode-btn')) $('toggle-mode-btn').addEventListener('click', () => {
        document.body.classList.toggle('dark-mode');
        $('toggle-mode-btn').classList.toggle('active');
    });

    if($('freq-select')) $('freq-select').addEventListener('change', (e) => {
        currentGPSMode = e.target.value;
        if (wID !== null) {
            stopGPS();
            startGPS();
        }
    });
});
