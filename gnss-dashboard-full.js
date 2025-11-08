// =================================================================
// FICHIER 1/2 : gnss-dashboard-core.js
// CŒUR DU SYSTÈME : EKF, IMU (Correction Gravité), GPS et Temps.
// =================================================================

// --- CLÉS D'API & PROXY VERCEL ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app"; 
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc"; 

// --- CONSTANTES GLOBALES ET INITIALISATION ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458; // Vitesse de la lumière (m/s)
const R_E = 6371000;   // Rayon terrestre moyen (m)
const KMH_MS = 3.6;    // Conversion m/s vers km/h
const G_ACC = 9.80665; // Gravité standard (m/s²)

const MIN_DT = 0.01; 
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};

// PARAMÈTRES AVANCÉS DU FILTRE DE KALMAN
const Q_NOISE = 0.01;       
const R_MIN = 0.05, R_MAX = 50.0; 
const MAX_ACC = 200, MIN_SPD = 0.05, ALT_TH = -50; 
const NETHER_RATIO = 8; 
const MAX_PLAUSIBLE_ACCEL = 20.0; 
const Q_ALT_NOISE = 0.1; 
const R_ALT_MIN = 0.1;  

const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0 }, 'METAL': { R_MULT: 2.5 },      
    'FOREST': { R_MULT: 1.5 }, 'CONCRETE': { R_MULT: 3.0 },   
};

// --- VARIABLES D'ÉTAT (GLOBALES) ---
let wID = null, lPos = null, lat = 0, lon = 0, sTime = null;
let distM = 0, maxSpd = 0, kSpd = 0, kUncert = 1000, timeMoving = 0, lastFSpeed = 0; 
let kAlt = null, kAltUncert = 10;  
let currentGPSMode = 'HIGH_FREQ'; 
let emergencyStopActive = false;
let netherMode = false;
let selectedEnvironment = 'NORMAL'; 
let lastP_hPa = null; // Utilisé pour le facteur R du Kalman
let currentLinearAccelMagnitude = 0; // Magnitude 3D de l'accélération linéaire (Gravité corrigée)
let latestRotationBeta = 0, latestRotationGamma = 0; // Angles Pitch et Roll (pour affichage Niveau à Bulle)


// --- FONCTIONS GÉO & UTILS ---
const dist = (lat1, lon1, lat2, lon2) => {
    if (lat1 === 0 && lon1 === 0) return 0;
    const R = R_E, dLat = (lat2 - lat1) * D2R, dLon = (lon2 - lon1) * D2R;
    lat1 *= D2R; lat2 *= D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R * c;
};

// --- SYNCHRONISATION HORAIRE ---
let lServH = null, lLocH = null; 
async function syncH() { 
    const localStartPerformance = performance.now(); 
    try {
        const response = await fetch(SERVER_TIME_ENDPOINT, { cache: "no-store", mode: "cors" });
        const localEndPerformance = performance.now(); 
        const serverData = await response.json(); 
        const serverTimestamp = Date.parse(serverData.datetime); 
        const RTT = localEndPerformance - localStartPerformance;
        lServH = serverTimestamp + RTT / 2; 
        lLocH = performance.now(); 
    } catch (error) {
        lServH = Date.now(); lLocH = performance.now();
    }
}
function getCDate() { 
    if (lServH === null || lLocH === null) return null;
    return new Date(lServH + (performance.now() - lLocH)); 
}

// --- FILTRE DE KALMAN ---
function kFilter(nSpd, dt, R_dyn) {
    if (dt === 0 || dt > 5) return kSpd; 
    const R = R_dyn ?? R_MAX, Q = Q_NOISE * dt; 
    let pSpd = kSpd, pUnc = kUncert + Q; 
    let K = pUnc / (pUnc + R); 
    kSpd = pSpd + K * (nSpd - pSpd); 
    kUncert = (1 - K) * pUnc; 
    return kSpd;
}

function kFilterAltitude(nAlt, acc, dt) {
    if (nAlt === null) return kAlt;
    const R = Math.max(R_ALT_MIN, acc); 
    const Q = Q_ALT_NOISE * dt; 
    let pAlt = kAlt === null ? nAlt : kAlt; 
    let pUnc = kAltUncert + Q; 
    const K = pUnc / (pUnc + R); 
    kAlt = pAlt + K * (nAlt - pAlt); 
    kAltUncert = (1 - K) * pUnc; 
    return kAlt;
}

function getKalmanR(acc) {
    let R = acc ** 2; 
    R = Math.max(R_MIN, Math.min(R_MAX, R));
    
    // Correction par le facteur environnemental
    R *= ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT;
    
    // Correction par la pression atmosphérique si disponible
    if (lastP_hPa !== null) {
        const pressureFactor = 1.0 + (1013.25 - lastP_hPa) / 1013.25 * 0.1;
        R *= Math.max(1.0, pressureFactor);
    }
    
    return R;
}

// ===========================================
// HANDLERS CAPTEURS IMU (ACCÉLÉRATION LINÉAIRE & NIVEAU À BULLE)
// ===========================================

// Fonction qui récupère l'accélération linéaire corrigée de la gravité par l'OS
function handleDeviceMotion(event) {
    if (emergencyStopActive) return;

    const linearAccel = event.acceleration; // Accélération linéaire (Gravité soustraite)
    
    if (linearAccel) {
        // Magnitude 3D de l'accélération linéaire (pour l'EKF)
        currentLinearAccelMagnitude = Math.sqrt(linearAccel.x**2 + linearAccel.y**2 + linearAccel.z**2);
        
        // Affichage de la composante verticale (Z) corrigée de la gravité (Pour l'affichage vertical)
        if ($('accel-vertical-imu')) $('accel-vertical-imu').textContent = `${linearAccel.z.toFixed(3)} m/s²`;
        if ($('force-g-vertical')) $('force-g-vertical').textContent = `${(linearAccel.z / G_ACC).toFixed(2)} G`;
    } else {
        currentLinearAccelMagnitude = 0;
    }
}

// Fonction qui récupère l'orientation (Niveau à Bulle)
function handleDeviceOrientation(event) {
    if (emergencyStopActive) return;
    latestRotationBeta = event.beta || 0;   // X (Pitch/Tangage)
    latestRotationGamma = event.gamma || 0; // Y (Roll/Roulis)

    // Affichage dans le DOM (utilisé par gnss-dashboard-ui-init.js)
    if (typeof updateDOMIMU === 'function') {
        updateDOMIMU(latestRotationBeta, latestRotationGamma);
    }
}

// ===========================================
// FONCTION PRINCIPALE DE MISE À JOUR GPS (EKF)
// ===========================================

function updateDisp(pos) {
    if (emergencyStopActive) return;

    lat = pos.coords.latitude; lon = pos.coords.longitude;
    const alt = pos.coords.altitude, acc = pos.coords.accuracy;
    const spd = pos.coords.speed;
    const cTimePos = pos.timestamp; 

    const now = getCDate(); 
    if (now === null || acc > MAX_ACC) { lPos = pos; return; } 

    if (sTime === null) sTime = now.getTime();
    
    let spdH = spd ?? 0;
    let spdV = 0; 
    const dt = lPos ? (cTimePos - lPos.timestamp) / 1000 : MIN_DT;
    if (dt === 0 || dt > 5) { lPos = pos; return; }

    // 1. FILTRAGE DE L'ALTITUDE
    const kAlt_new = kFilterAltitude(alt, acc, dt);
    if (lPos && lPos.kAlt_old !== undefined) spdV = (kAlt_new - lPos.kAlt_old) / dt;

    // 2. CALCUL DE LA VITESSE HORIZONTALE BRUTE
    if (lPos) { 
        const dH = dist(lPos.coords.latitude, lPos.coords.longitude, lat, lon); 
        spdH = dH / dt; 
    } else if (spd !== null) {
        spdH = spd;
    }
    
    // 3. VITESSE 3D BRUTE
    let spd3D = Math.sqrt(spdH ** 2 + spdV ** 2);
    
    // 4. CORRECTION ANTI-SPIKE DE VITESSE (optionnel)
    if (lPos && lPos.speedMS_3D !== undefined) {
        const lastRawSpd = lPos.speedMS_3D;
        const accelSpike = Math.abs(spd3D - lastRawSpd) / dt;
        if (accelSpike > MAX_PLAUSIBLE_ACCEL) {
            const maxPlausibleChange = MAX_PLAUSIBLE_ACCEL * dt;
            spd3D = spd3D > lastRawSpd ? lastRawSpd + maxPlausibleChange : lastRawSpd - maxPlausibleChange;
        }
    }

    // 5. PRÉDICTION/CORRECTION EKF (Vitesse 3D)
    
    // Calcul de l'accélération IMU signée pour la prédiction (composante longitudinale)
    let accel_imu_sign = Math.sign(kSpd || (spd3D - lastFSpeed));
    const accel_imu_signed = currentLinearAccelMagnitude * accel_imu_sign;

    let kSpd_pred = kSpd, kUncert_pred = kUncert + Q_NOISE * dt;
    if (currentLinearAccelMagnitude > 0.05) { 
        kSpd_pred = kSpd + accel_imu_signed * dt; 
    }

    const R_dyn = getKalmanR(acc); 
    let kGain = kUncert_pred / (kUncert_pred + R_dyn);
    const fSpd = kSpd_pred + kGain * (spd3D - kSpd_pred); 
    kUncert = (1 - kGain) * kUncert_pred;             

    const sSpdFE = fSpd < MIN_SPD ? 0 : fSpd;
    kSpd = sSpdFE;
    
    // 6. CALCUL ACCÉLÉRATION FINALE (Fusion EKF et IMU)
    let accel_ekf_raw = (dt > 0.05) ? (sSpdFE - lastFSpeed) / dt : 0;
    const FUSION_FACTOR = 0.7;
    let accel_long;
    if (currentLinearAccelMagnitude > 0.05) {
        accel_long = (accel_ekf_raw * (1 - FUSION_FACTOR)) + (accel_imu_signed * FUSION_FACTOR);
    } else {
        accel_long = accel_ekf_raw;
    }
    
    lastFSpeed = sSpdFE;

    // Mise à jour de la distance et du temps
    distM += sSpdFE * dt * (netherMode ? NETHER_RATIO : 1); 
    if (sSpdFE > MIN_SPD) { timeMoving += dt; }
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    
    // Appel à la mise à jour de l'UI (définie dans gnss-dashboard-ui-init.js)
    if (typeof updateDOMAndMap === 'function') {
        updateDOMAndMap(sSpdFE, spd3D, spdV, accel_long, acc, kAlt_new, pos.coords.heading);
    }
    
    // SAUVEGARDE DES VALEURS POUR LA PROCHAINE ITÉRATION
    lPos = pos; 
    lPos.speedMS_3D = spd3D; 
    lPos.kAlt_old = kAlt_new; 
}

// --- FONCTIONS DE CONTRÔLE ---
function setGPSMode(mode) {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    currentGPSMode = mode;
    wID = navigator.geolocation.watchPosition(updateDisp, handleErr, GPS_OPTS[mode]);
    if (typeof startButtonUpdate === 'function') startButtonUpdate();
}

function startGPS() {
    if (wID === null) {
        window.addEventListener('devicemotion', handleDeviceMotion, true);
        window.addEventListener('deviceorientation', handleDeviceOrientation, true);
        setGPSMode(currentGPSMode);
    }
}

function stopGPS(resetButton = true) {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
    }
    if (resetButton && typeof stopButtonUpdate === 'function') stopButtonUpdate();
}

function handleErr(err) {
    console.error(`Erreur GNSS (${err.code}): ${err.message}`);
    if (typeof handleGPSErrorUI === 'function') handleGPSErrorUI();
    }
// =================================================================
// FICHIER 2/2 : gnss-dashboard-ui-init.js
// MODULES UI, ASTRO (Projection Zénithale) & INITIALISATION.
// =================================================================

const $ = id => document.getElementById(id);

// --- ASTRO CONSTANTES ---
const MAX_RADIUS_PX = 45; // Rayon du cadran (90px de large) / 2
const DOM_SLOW_UPDATE_MS = 1000; 

// --- CARTE LEAFLET ---
let map = null;
let marker = null;

function initMap() {
    if (typeof L === 'undefined') return;
    map = L.map('map-container').setView([lat, lon], 2);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
    }).addTo(map);
    marker = L.marker([lat, lon]).addTo(map).bindPopup("Position Actuelle").openPopup();
}

function updateMap(latitude, longitude) {
    if (map) {
        map.setView([latitude, longitude], map.getZoom() < 12 ? 12 : map.getZoom());
        marker.setLatLng([latitude, longitude]);
    }
}

// --- LOGIQUE ASTRO ZÉNITHALE ET COULEUR DU CIEL ---

function projectAstroPosition(pos) {
    const altitudeDeg = pos.altitude * R2D;
    let azimuthDeg = pos.azimut * R2D;    
    if (azimuthDeg < 0) azimuthDeg += 360; 

    if (altitudeDeg < -18) return { x: NaN, y: NaN }; 

    // Calcul de la Distance au Centre (D) : 0 au Zénith, MAX_RADIUS_PX à l'horizon (0°)
    const zenithAngle = 90 - altitudeDeg; 
    const distanceToCenter = MAX_RADIUS_PX * (zenithAngle / 90); 

    // Nord (Azimut 0) doit correspondre à l'axe Y- (Haut)
    const angleRotationDeg = azimuthDeg - 90; 
    const angleRotationRad = angleRotationDeg * D2R; 
    
    const x = distanceToCenter * Math.cos(angleRotationRad);
    const y = distanceToCenter * Math.sin(angleRotationRad);

    return { x: x, y: y };
}

function updateAstro(latA, lonA) {
    const now = getCDate(); 
    if (now === null || latA === 0) return;
    
    const sunPos = window.SunCalc ? SunCalc.getPosition(now, latA, lonA) : null;
    const moonIllum = window.SunCalc ? SunCalc.getMoonIllumination(now) : null;
    const sunTimes = window.SunCalc ? SunCalc.getTimes(now, latA, lonA) : null;
    const moonPos = window.SunCalc ? SunCalc.getMoonPosition(now, latA, lonA) : null;
    
    // Heure Solaire Vraie (TST)
    const solarTimes = getSolarTime(now, lonA); // Fonction définie dans Core
    if ($('tst')) $('tst').textContent = solarTimes.TST;
    if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString('fr-FR', { timeZone: 'UTC', hour12: false });
    if ($('ecliptic-long')) $('ecliptic-long').textContent = solarTimes.ECL_LONG + ' °';
    if ($('eot')) $('eot').textContent = solarTimes.EOT + ' min'; 
    if ($('lsm')) $('lsm').textContent = solarTimes.MST;

    // --- NOUVEAU : COULEUR DU CIEL DYNAMIQUE (sur le BODY via classes CSS) ---
    const body = document.body;
    body.classList.remove('sky-day', 'sky-sunset', 'sky-night-light', 'sky-night');

    const sunAltitudeDeg = sunPos ? sunPos.altitude * R2D : -90; 
    
    if (sunAltitudeDeg > 10) body.classList.add('sky-day');
    else if (sunAltitudeDeg > 0) body.classList.add('sky-sunset');
    else if (sunAltitudeDeg > -6) body.classList.add('sky-sunset');
    else if (sunAltitudeDeg > -18) body.classList.add('sky-night-light');
    else body.classList.add('sky-night');
    
    // --- NOUVEAU : PROJECTION ZÉNITHALE DES ASTRES ---
    if (sunPos) {
        const sunCoords = projectAstroPosition(sunPos);
        const moonCoords = projectAstroPosition(moonPos);
        const sunEl = $('sun-element'), moonEl = $('moon-element');
        const centerOffset = MAX_RADIUS_PX; 

        const updateAstroEl = (el, coords) => {
            if (!isNaN(coords.x)) {
                el.style.display = 'flex'; 
                el.style.transform = `translate(${coords.x + centerOffset}px, ${coords.y + centerOffset}px)`;
            } else {
                el.style.display = 'none';
            }
        };

        updateAstroEl(sunEl, sunCoords);
        updateAstroEl(moonEl, moonCoords);

        // --- MISE À JOUR DES DONNÉES ASTRO ---
        if ($('sun-elevation')) $('sun-elevation').textContent = `${sunAltitudeDeg.toFixed(2)} °`;
        if ($('noon-solar')) $('noon-solar').textContent = sunTimes && sunTimes.solarNoon ? sunTimes.solarNoon.toLocaleTimeString() : 'N/D';
        if ($('moon-phase-display')) $('moon-phase-display').textContent = moonIllum ? `${(moonIllum.fraction * 100).toFixed(1)} % (${getMoonPhaseName(moonIllum.phase)})` : 'N/A';
        if ($('day-duration') && sunTimes && sunTimes.sunrise && sunTimes.sunset) {
            const durationMs = sunTimes.sunset.getTime() - sunTimes.sunrise.getTime();
            const hours = Math.floor(durationMs / 3600000);
            const minutes = Math.floor((durationMs % 3600000) / 60000);
            $('day-duration').textContent = `${hours}h ${minutes}m`;
        }
    }
}

function getMoonPhaseName(phase) {
    if (phase < 0.03 || phase > 0.97) return "Nouvelle";
    if (phase < 0.28) return "Premier Quartier";
    if (phase < 0.53) return "Pleine";
    if (phase < 0.78) return "Dernier Quartier";
    return "Croissant/Gibbeuse";
}

// --- MISE À JOUR DOM GÉNÉRALE ---

let domID = null;
function updateDOMAndMap(sSpdFE, spd3D, spdV, accel_long, acc, kAlt_new, heading) {
    // Coordonnées et Précision
    if ($('latitude')) $('latitude').textContent = lat.toFixed(6);
    if ($('longitude')) $('longitude').textContent = lon.toFixed(6);
    if ($('altitude-gps')) $('altitude-gps').textContent = kAlt_new !== null ? `${kAlt_new.toFixed(2)} m` : 'N/A';
    if ($('gps-precision')) $('gps-precision').textContent = `${acc.toFixed(2)} m`; 

    // Vitesse et Accélération
    if ($('speed-stable')) $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(5)} km/h`; 
    if ($('speed-3d-inst')) $('speed-3d-inst').textContent = `${(spd3D * KMH_MS).toFixed(5)} km/h`; 
    if ($('vertical-speed')) $('vertical-speed').textContent = `${spdV.toFixed(2)} m/s`;
    if ($('accel-long')) $('accel-long').textContent = `${accel_long.toFixed(3)} m/s²`;
    if ($('force-g-long')) $('force-g-long').textContent = `${(accel_long / G_ACC).toFixed(2)} G`;
    if ($('speed-max')) $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;

    // Distance et Temps
    if ($('distance-total-km')) $('distance-total-km').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    if ($('speed-avg-moving')) $('speed-avg-moving').textContent = timeMoving > 1 ? `${(distM / timeMoving * KMH_MS).toFixed(5)} km/h` : '0.00000 km/h';

    // Cap
    if ($('heading-display')) $('heading-display').textContent = heading !== null && !isNaN(heading) ? `${heading.toFixed(0)} °` : 'N/A';

    updateMap(lat, lon);
}

function updateDOMIMU(pitch, roll) {
    if ($('pitch-angle')) $('pitch-angle').textContent = `${pitch.toFixed(1)} °`;
    if ($('roll-angle')) $('roll-angle').textContent = `${roll.toFixed(1)} °`;
}

// --- MÉTÉO ---

async function fetchWeather() {
    if (!lat || !lon || PROXY_BASE_URL.includes('scientific-dashboard2')) {
        if ($('temp-air')) $('temp-air').textContent = 'API Requis';
        return; 
    }

    try {
        const response = await fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`);
        const data = await response.json();
        
        if (data.main) {
            lastP_hPa = data.main.pressure; // Mise à jour pour le Kalman
            if ($('temp-air')) $('temp-air').textContent = `${(data.main.temp).toFixed(1)} °C`;
            if ($('pressure')) $('pressure').textContent = `${data.main.pressure.toFixed(1)} hPa`;
            if ($('humidity')) $('humidity').textContent = `${data.main.humidity}%`;
            if ($('wind-speed-ms')) $('wind-speed-ms').textContent = `${data.wind.speed.toFixed(1)} m/s`;
        }

    } catch (error) {
        console.error("Échec de la récupération des données météo :", error);
    }
}

// --- UI CONTRÔLES ---

function startButtonUpdate() {
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = `⏸️ PAUSE GPS`;
    if ($('freq-select')) $('freq-select').value = currentGPSMode;
    if ($('nether-toggle-btn')) $('nether-toggle-btn').textContent = netherMode ? "ACTIVÉ (1:8) 🔥" : "DÉSACTIVÉ (1:1)";
}

function stopButtonUpdate() {
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = `▶️ MARCHE GPS`;
}

function handleGPSErrorUI() {
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = `❌ ERREUR GPS`;
    emergencyStop();
}

// --- INITIALISATION ---

document.addEventListener('DOMContentLoaded', () => {
    initMap();
    syncH(); 
    startButtonUpdate(); // Initialise les boutons dans l'état "PAUSE" (avant le start réel)
    
    // --- Listeners de Contrôle (Appelle les fonctions du Core) ---
    if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', () => wID === null ? startGPS() : stopGPS());
    if ($('freq-select')) $('freq-select').addEventListener('change', (e) => setGPSMode(e.target.value));

    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', () => emergencyStopActive ? resumeSystem() : emergencyStop());
    if ($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', () => netherMode = !netherMode);
    
    // Listeners de Réinitialisation
    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => { distM = 0; timeMoving = 0; });
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', () => { maxSpd = 0; });
    
    // Démarrage des intervalles
    domID = setInterval(() => updateAstro(lat, lon), DOM_SLOW_UPDATE_MS); 
    setInterval(fetchWeather, 60000); 

    // Démarrage du GPS (avec écouteurs IMU)
    startGPS();
});
