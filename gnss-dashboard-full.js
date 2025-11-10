// =================================================================
// FICHIER JS PARTIE 1/2 : gnss-dashboard-part1.js
// Contient constantes, variables d'état, UTC/NTP et Kalman.
// =================================================================

// --- CLÉS D'API & PROXY VERCEL ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app"; 
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc"; // API NTP/UTC

// --- CONSTANTES GLOBALES ET PHYSIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const R_E_BASE = 6371000;   // Rayon terrestre moyen de base (m)
const KMH_MS = 3.6;         // Conversion m/s vers km/h
const C_S = 343;            // Vitesse du son dans l'air (m/s)
const MC_DAY_MS = 72 * 60 * 1000; // Durée d'un jour Minecraft en ms

const J1970 = 2440588, J2000 = 2451545; 
const dayMs = 1000 * 60 * 60 * 24;      
const MIN_DT = 0.01; 

// --- PARAMÈTRES EKF ---
const Q_NOISE = 0.01;       
const R_MIN = 0.05, R_MAX = 50.0; 
const MIN_SPD = 0.05;       
const ALT_TH = -50;         
const MAX_PLAUSIBLE_ACCEL = 20.0; 
const Q_ALT_NOISE = 0.1; 
const R_ALT_MIN = 0.5;
const ACC_DAMPEN_LOW = 5.0; 
const ACC_DAMPEN_HIGH = 50.0; 
const DOM_SLOW_UPDATE_MS = 1000; 

// Facteurs d'environnement pour l'incertitude GPS (R)
const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0, DISPLAY: 'NORMAL' },
    'FOREST': { R_MULT: 1.5, DISPLAY: 'FORÊT' },
    'CONCRETE': { R_MULT: 3.0, DISPLAY: 'GROTTE/TUNNEL (NETHER)' },
    'METAL': { R_MULT: 2.5, DISPLAY: 'MÉTAL/BÂTIMENT' },
};

// --- DONNÉES CÉLESTES/GRAVITÉ ---
const CELESTIAL_DATA = {
    'EARTH': { G: 9.80665, R: R_E_BASE, name: 'Terre' },
    'MOON': { G: 1.62, R: 1737400, name: 'Lune' },
    'MARS': { G: 3.71, R: 3389500, name: 'Mars' },
    'ROTATING': { G: 0.0, R: R_E_BASE, name: 'Station Spatiale' }
};
let G_ACC = CELESTIAL_DATA['EARTH'].G; // Gravité effective (m/s²)
let R_ALT_CENTER_REF = CELESTIAL_DATA['EARTH'].R; // Rayon de référence

// --- VARIABLES D'ÉTAT ---
let wID = null, domID = null, lPos = null, lat = null, lon = null, sTime = null;
let distM = 0, maxSpd = 0;
let kSpd = 0, kUncert = 1000; 
let timeMoving = 0; 
let lServH = null, lLocH = null; 
let lastFSpeed = 0; 
let kAlt = null;      
let kAltUncert = 10;  
let currentGPSMode = 'HIGH_FREQ'; 
let emergencyStopActive = false;
let selectedEnvironment = 'NORMAL'; 
let currentMass = 70; 
let R_FACTOR_RATIO = 1.0;   // Facteur de Rapport de Mouvement (MRF)
let currentCelestialBody = 'EARTH';
let rotationRadius = 100;
let angularVelocity = 0.0; 
let lastP_hPa = 1013.25; 
let gpsAccuracyOverride = 0.0; 
let distMStartOffset = 0;
let lastUpdateMap = 0;
let map, marker, circle;


// --- REFERENCES DOM & FONCTIONS UTILS ---
const $ = id => document.getElementById(id);

const dist = (lat1, lon1, lat2, lon2) => {
    const R = R_ALT_CENTER_REF; 
    const dLat = (lat2 - lat1) * D2R, dLon = (lon2 - lon1) * D2R;
    lat1 *= D2R; lat2 *= D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
};

const getCDate = () => {
    if (lServH && lLocH) {
        return new Date(lServH + (Date.now() - lLocH));
    }
    return new Date();
};

// --- FILTRE DE KALMAN (VITESSE) ---
/** Applique le filtre de Kalman à la vitesse 3D (Fusion GPS/Accélération). */
function kFilter(nSpd, dt, R_dyn, accel_input = 0) {
    if (dt === 0 || dt > 5) return kSpd; 
    
    // 1. PRÉDICTION
    const Q_MOTION = Q_NOISE * dt; 
    let pSpd = kSpd + (accel_input * dt); 
    let pUnc = kUncert + Q_MOTION; 
    
    // 2. MISE À JOUR
    const R = R_dyn ?? R_MAX; 
    let K = pUnc / (pUnc + R); 
    kSpd = pSpd + K * (nSpd - pSpd); 
    kUncert = (1 - K) * pUnc; 
    
    return kSpd;
}

// --- FILTRE DE KALMAN (ALTITUDE) ---
function kFilterAltitude(nAlt, dt, R_alt_raw) {
    if (dt === 0 || nAlt === null) return kAlt;

    // 1. Prédiction
    const Q_alt = Q_ALT_NOISE * dt;
    let pAlt = kAlt === null ? nAlt : kAlt; 
    let pAltUncert = kAlt === null ? 1000 : kAltUncert + Q_alt;
    
    // 2. Mise à jour
    const R_alt = Math.max(R_ALT_MIN, R_alt_raw ** 2); 
    let K_alt = pAltUncert / (pAltUncert + R_alt);
    kAlt = pAlt + K_alt * (nAlt - pAlt);
    kAltUncert = (1 - K_alt) * pAltUncert;
    
    return kAlt;
}

// --- FACTEUR D'INCERTITUDE EKF (R dynamique) ---
function getKalmanR(usedAcc, alt, P_hPa) {
    let R = R_MAX;
    if (usedAcc > 0.0) {
        R = usedAcc ** 2;
        const envFactor = ENVIRONMENT_FACTORS[selectedEnvironment]?.R_MULT || 1.0;
        R *= envFactor; 
    }
    return Math.max(R_MIN, Math.min(R_MAX, R));
}

// --- LOGIQUE MRF (Facteur de Rapport de Mouvement) ---
function calculateMRF(kAltA) {
    if (kAltA === null || R_ALT_CENTER_REF === null) return 1.0;

    const R_current = R_ALT_CENTER_REF + kAltA; 
    const R_ref = R_ALT_CENTER_REF;
    const ratio = R_current / R_ref;

    return Math.max(0.001, Math.min(1.5, ratio));
}

// --- LOGIQUE CELESTE ET GRAVITÉ ---
function calculateArtificialGravity() {
    if (angularVelocity === 0 || rotationRadius === 0) return 0;
    // g_art = omega^2 * r
    return angularVelocity ** 2 * rotationRadius;
}

/** Met à jour les constantes physiques pour le corps céleste sélectionné. */
function updateCelestialBody(body) {
    currentCelestialBody = body;
    let newG = 0;
    let newR = R_E_BASE;

    if (body === 'ROTATING') {
        const g_art = calculateArtificialGravity();
        newG = g_art; 
        newR = R_E_BASE; 

        if ($('gravity-local')) $('gravity-local').textContent = `${newG.toFixed(4)} m/s² (Centrifuge)`;
    } else {
        const data = CELESTIAL_DATA[body];
        newG = data.G;
        newR = data.R;
        if ($('gravity-local')) $('gravity-local').textContent = `${newG.toFixed(4)} m/s²`;
    }
    
    G_ACC = newG;
    R_ALT_CENTER_REF = newR; 
    
    kAlt = null; kAltUncert = 10;
    
    console.log(`Corps céleste réglé sur ${body}. G_ACC: ${G_ACC}, R_REF: ${R_ALT_CENTER_REF}`);
}
// Le reste de la logique doit être dans la partie 2
// =================================================================
// FICHIER JS PARTIE 2/2 : gnss-dashboard-part2.js
// Contient les fonctions Astro, Météo, GPS (updateDisp) et DOM.
// =================================================================

// NOTE: Ce fichier dépend des variables globales et fonctions (dist, $, kFilter...) définies dans gnss-dashboard-part1.js.

// --- FONCTIONS GÉO, MÉTÉO & NTP ---

function initMap() {
    if ($('map-container')) {
        map = L.map('map-container').setView([0, 0], 2);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: '© OpenStreetMap contributors'
        }).addTo(map);
        marker = L.marker([0, 0]).addTo(map);
        circle = L.circle([0, 0], { color: 'red', fillColor: '#f03', fillOpacity: 0.5, radius: 10 }).addTo(map);
    }
}

function updateMap(lat, lon, acc) {
    if (map && marker) {
        marker.setLatLng([lat, lon]);
        circle.setLatLng([lat, lon]).setRadius(acc * R_FACTOR_RATIO); 
        const now = Date.now();
        if (now - lastUpdateMap > 3000 && kSpd > MIN_SPD) {
            map.setView([lat, lon], map.getZoom() > 10 ? map.getZoom() : 16); 
            lastUpdateMap = now;
        } else if (map.getZoom() < 10) {
            map.setView([lat, lon], 12);
        }
    }
}

function syncH() {
    fetch(SERVER_TIME_ENDPOINT)
        .then(r => r.json())
        .then(data => {
            const serverTime = new Date(data.utc_datetime);
            lServH = serverTime.getTime();
            lLocH = Date.now();
            if ($('local-time')) $('local-time').textContent = serverTime.toLocaleTimeString();
            if ($('date-display')) $('date-display').textContent = serverTime.toLocaleDateString();
        })
        .catch(err => {
            console.error("Erreur de synchro NTP/UTC:", err);
            if ($('local-time')) $('local-time').textContent = new Date().toLocaleTimeString() + ' (Local)';
        });
}

function fetchWeather() {
    if (!lat || !lon) {
        setTimeout(fetchWeather, 5000); 
        return;
    }

    const apiUrl = `${PROXY_WEATHER_ENDPOINT}?lat=${lat}&lon=${lon}`;
    
    fetch(apiUrl)
        .then(r => r.json())
        .then(data => {
            if (data.main) {
                const tempC = data.main.temp - 273.15;
                const pressure_hPa = data.main.pressure;
                lastP_hPa = pressure_hPa; 
                
                if ($('temp-air')) $('temp-air').textContent = `${tempC.toFixed(1)} °C`;
                if ($('pressure')) $('pressure').textContent = `${pressure_hPa.toFixed(2)} hPa`;
                if ($('humidity')) $('humidity').textContent = `${data.main.humidity}%`;
                
                if ($('o2-level')) $('o2-level').textContent = `20.9 % vol`; 
                if ($('co2-level')) $('co2-level').textContent = `420 ppm`; 
                if ($('noise-level')) $('noise-level').textContent = `N/A`; 
            } else {
                 throw new Error(data.message || 'Données météo incomplètes');
            }
        })
        .catch(err => {
            console.warn("Erreur de récupération météo:", err);
            if ($('temp-air')) $('temp-air').textContent = `N/A`;
            if ($('pressure')) $('pressure').textContent = `N/A`;
            if ($('humidity')) $('humidity').textContent = `N/A`;
        });
}


// --- FONCTIONS ASTRO & TEMPS ---

function toDays(date) { return date.getTime() / dayMs - (J1970 - J2000); }
function solarMeanAnomaly(d) { return D2R * (357.5291 + 0.98560028 * d); }
function eclipticLongitude(M) {
    const C = D2R * (1.9148 * Math.sin(M) + 0.02 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M)); 
    const P = D2R * 102.9372; 
    return M + C + P + Math.PI;
}

function getSolarTime(d, lon, lat) {
    const sunPos = SunCalc.getPosition(d, lat, lon);
    const date = new Date(d);
    
    const M = solarMeanAnomaly(toDays(date));
    const L = eclipticLongitude(M);
    
    const J_star = toDays(date) - lon / 360;
    const J_transit = J_star + (0.0053 * Math.sin(M) - 0.0069 * Math.sin(2 * L));
    
    const TST_days = (toDays(date) - J_transit) % 1; 
    const TST_ms = TST_days * dayMs;

    return {
        TST_ms, 
        elevation: sunPos.altitude * R2D,
        azimuth: sunPos.azimuth * R2D,
        LSM: new Date(J_star * dayMs).toUTCString().split(' ')[4],
        noonSolar: new Date(J_transit * dayMs).toUTCString().split(' ')[4],
        eclipticLong: L * R2D,
        eot: (J_star - J_transit) * 24 * 60
    };
}

function getMinecraftTime(solarElevationDeg) {
    const sunTimes = SunCalc.getTimes(new Date(), lat, lon);
    const currentTime = getCDate().getTime();
    
    const noon = sunTimes.solarNoon.getTime();
    const midnight = sunTimes.nadir.getTime(); 
    
    let cycleMs;
    if (currentTime >= noon || currentTime < midnight) {
        cycleMs = currentTime - noon;
    } else {
        cycleMs = currentTime - midnight;
    }
    
    const ticks = Math.floor((cycleMs % dayMs) / (dayMs / 24000));
    let mcTime = (ticks + 18000) % 24000;
    
    const hours = Math.floor(mcTime / 1000);
    const minutes = Math.floor((mcTime % 1000) / (1000/60));
    const seconds = Math.floor(((mcTime % 1000) % (1000/60)) / (1000/3600));

    return `${String(hours).padStart(2, '0')}:${String(minutes).padStart(2, '0')}:${String(seconds).padStart(2, '0')}`;
}

function getMoonPhaseName(phase) {
    if (phase === 0 || phase === 1) return "Nouvelle Lune 🌑";
    if (phase > 0 && phase < 0.25) return "Croissant Jeune 🌒";
    if (phase === 0.25) return "Premier Quartier 🌓";
    if (phase > 0.25 && phase < 0.5) return "Gibbeuse Croissante 🌔";
    if (phase === 0.5) return "Pleine Lune 🌕";
    if (phase > 0.5 && phase < 0.75) return "Gibbeuse Décroissante 🌖";
    if (phase === 0.75) return "Dernier Quartier 🌗";
    return "Croissant Vieux 🌘";
}

function updateClockVisual(elevation, moonPhase) {
    const sun = $('sun-element'), moonEl = $('moon-element');
    const moonIcon = $('moon-icon-display');
    const clockStatus = $('clock-status');

    if (!sun || !moonEl) return;

    const angle = (elevation + 90) * 180 / 180;
    
    sun.style.transform = `rotate(${angle - 90}deg)`; 
    moonEl.style.transform = `rotate(${angle + 90}deg)`; 

    sun.style.opacity = elevation > 0 ? 1 : 0;
    moonEl.style.opacity = elevation <= 0 ? 1 : 0;
    
    if (moonPhase === 0 || moonPhase === 1) moonIcon.textContent = '🌑';
    else if (moonPhase < 0.25) moonIcon.textContent = '🌒';
    else if (moonPhase < 0.5) moonIcon.textContent = '🌓';
    else if (moonPhase === 0.5) moonIcon.textContent = '🌕';
    else if (moonPhase < 0.75) moonIcon.textContent = '🌖';
    else moonIcon.textContent = '🌗';

    clockStatus.textContent = elevation > 0 ? `Jour (Élévation: ${elevation.toFixed(1)}°)` : `Nuit (Lune: ${getMoonPhaseName(moonPhase)})`;
}

function updateAstro(lat, lon) {
    const cDate = getCDate();
    const astro = getSolarTime(cDate, lon, lat);
    const times = SunCalc.getTimes(cDate, lat, lon);
    const moon = SunCalc.getMoonIllumination(cDate);
    
    if ($('tst')) $('tst').textContent = new Date(astro.TST_ms).toUTCString().split(' ')[4];
    if ($('sun-elevation')) $('sun-elevation').textContent = `${astro.elevation.toFixed(2)} °`;
    if ($('moon-phase-display')) $('moon-phase-display').textContent = `${(moon.fraction * 100).toFixed(1)}% / ${getMoonPhaseName(moon.phase)}`;
    if ($('lsm')) $('lsm').textContent = astro.LSM;
    if ($('noon-solar')) $('noon-solar').textContent = astro.noonSolar;
    if ($('ecliptic-long')) $('ecliptic-long').textContent = `${astro.eclipticLong.toFixed(2)} °`;
    if ($('eot')) $('eot').textContent = `${astro.eot.toFixed(2)} min`;
    
    if (times.sunrise && times.sunset) {
        const dayDuration = (times.sunset.getTime() - times.sunrise.getTime()) / 3600000;
        if ($('day-duration')) $('day-duration').textContent = `${dayDuration.toFixed(2)} h`;
    }
    
    if ($('time-minecraft')) $('time-minecraft').textContent = getMinecraftTime(astro.elevation);
    updateClockVisual(astro.elevation, moon.phase);
    
    let skyClass = 'sky-day';
    if (astro.elevation < 0) {
        skyClass = 'sky-night-light';
        if (astro.elevation < -18) skyClass = 'sky-night';
    } else if (astro.elevation < 5) {
        skyClass = 'sky-sunset'; 
    }
    document.body.className = skyClass; 
}


// --- CONTRÔLES GPS ---

function setGPSMode(mode) {
    currentGPSMode = mode;
    if (wID !== null) {
        stopGPS();
        startGPS();
    }
}

function startGPS() {
    if (wID !== null) return;
    
    const options = {
        enableHighAccuracy: currentGPSMode === 'HIGH_FREQ',
        timeout: 5000,
        maximumAge: currentGPSMode === 'HIGH_FREQ' ? 0 : 3000
    };
    
    wID = navigator.geolocation.watchPosition(updateDisp, handleErr, options);
    
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = ' 7œ4„1‚5 MARCHE GPS';
    if ($('toggle-gps-btn')) $('toggle-gps-btn').style.backgroundColor = '#28a745';
    
    if (sTime === null) sTime = Date.now();
    fetchWeather();
}

function stopGPS() {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
    }
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = ' 7ß8„1‚5 ARRÊT GPS';
    if ($('toggle-gps-btn')) $('toggle-gps-btn').style.backgroundColor = '#dc3545';
}

function emergencyStop() {
    emergencyStopActive = true;
    stopGPS();
    if ($('emergency-stop-btn')) {
        $('emergency-stop-btn').textContent = '•0“5 ARRÊT D\'URGENCE: ACTIF 🔴';
        $('emergency-stop-btn').classList.add('active');
    }
}

function resumeSystem() {
    emergencyStopActive = false;
    startGPS();
    if ($('emergency-stop-btn')) {
        $('emergency-stop-btn').textContent = '•0“5 Arrêt d\'urgence: INACTIF 🟢';
        $('emergency-stop-btn').classList.remove('active');
    }
}

function handleErr(err) {
    console.warn(`ERREUR GPS (${err.code}): ${err.message}`);
    if ($('gps-precision')) $('gps-precision').textContent = `Erreur: ${err.message}`;
    
    if (err.code === 1) { 
        stopGPS();
        alert("Accès à la géolocalisation refusé. Veuillez l'activer.");
    }
}

// --- FONCTION PRINCIPALE DE MISE À JOUR GPS (EKF 3D) ---

function updateDisp(pos) {
    if (emergencyStopActive) return;

    const cTimePos = pos.timestamp;
    const cLat = pos.coords.latitude;
    const cLon = pos.coords.longitude;
    const altRaw = pos.coords.altitude;
    let accRaw = pos.coords.accuracy;
    
    // 0. Gérer l'override de précision GPS
    if (gpsAccuracyOverride > 0.0) {
        accRaw = gpsAccuracyOverride;
    }
    let usedAcc = accRaw;

    // 1. Calcul de dt et initialisation
    let dt = 0;
    if (lPos) {
        dt = (cTimePos - lPos.timestamp) / 1000;
    } else {
        lPos = pos;
        lPos.speedMS_3D = 0;
        lPos.kAlt_old = altRaw;
        kAlt = altRaw; 
        lat = cLat; lon = cLon;
        updateMap(cLat, cLon, usedAcc);
        return; 
    }
    
    if (dt < MIN_DT || dt > 10) return; 

    // 2. Mise à jour de l'altitude EKF et MRF
    const kAlt_new = kFilterAltitude(altRaw, dt, pos.coords.altitudeAccuracy || R_ALT_MIN);
    kAlt = kAlt_new; 
    R_FACTOR_RATIO = calculateMRF(kAlt_new);

    // 3. Calculs de distance, vitesse brutes
    const dist2D = dist(lat, lon, cLat, cLon);
    const dist3D = Math.sqrt(dist2D ** 2 + (kAlt_new - lPos.kAlt_old) ** 2);
    const spd3D = dist3D / dt;
    const spdV = (kAlt_new - lPos.kAlt_old) / dt; 

    // 4. Correction Anti-Spike (limitée)
    const accPrev = Math.abs((lPos.speedMS_3D - kSpd) / dt);
    const maxAccel = Math.min(MAX_PLAUSIBLE_ACCEL, (accPrev * ACC_DAMPEN_LOW + ACC_DAMPEN_HIGH));
    if (Math.abs(spd3D - lPos.speedMS_3D) / dt > maxAccel) {
        console.warn(`SPIKE DETECTÉ! Vitesse brute rejetée: ${spd3D.toFixed(2)} m/s`);
        lPos.speedMS_3D = kSpd; 
    }
    
    // 5. FILTRE DE KALMAN FINAL (Vitesse) - Fusion Inertielle/GPS
    const R_dyn = getKalmanR(usedAcc, altRaw, lastP_hPa); 
    const accel_sensor_input = 0; 
    const fSpd = kFilter(spd3D, dt, R_dyn, accel_sensor_input); 
    const sSpdFE = fSpd < MIN_SPD ? 0 : fSpd;
    
    // 6. Calculs d'accélération, Distance et Vitesse Max
    let accel_long = 0;
    if (dt > 0.05) {
        accel_long = (sSpdFE - lastFSpeed) / dt;
    }
    lastFSpeed = sSpdFE;

    distM += sSpdFE * dt * R_FACTOR_RATIO; 
    if (sSpdFE > MIN_SPD) { timeMoving += dt; }
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    
    // --- MISE À JOUR DU DOM (Cinématique et Physique) ---
    lat = cLat; lon = cLon;
    const sSpdKMH = sSpdFE * KMH_MS;
    const maxSpdKMH = maxSpd * KMH_MS;
    const distM_disp = distM;
    const distKM_disp = distM / 1000;
    const mass = currentMass;
    const kineticEnergy = 0.5 * mass * sSpdFE ** 2;
    const mechanicalPower = mass * sSpdFE * accel_long;
    
    if ($('local-time')) $('local-time').textContent = getCDate().toLocaleTimeString();
    if ($('time-elapsed')) $('time-elapsed').textContent = `${((Date.now() - sTime) / 1000).toFixed(2)} s`;
    if ($('time-moving')) $('time-moving').textContent = `${timeMoving.toFixed(2)} s`;
    
    if ($('speed-stable')) $('speed-stable').textContent = `${sSpdKMH.toFixed(5)} km/h`;
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${sSpdFE.toFixed(3)} m/s | ${(sSpdFE * 1000).toFixed(0)} mm/s`;
    if ($('speed-max')) $('speed-max').textContent = `${maxSpdKMH.toFixed(5)} km/h`;
    if ($('speed-avg-moving')) $('speed-avg-moving').textContent = timeMoving > 0 ? `${(distM / timeMoving * KMH_MS).toFixed(5)} km/h` : '0.00000 km/h';

    if ($('latitude')) $('latitude').textContent = `${cLat.toFixed(5)} °`;
    if ($('longitude')) $('longitude').textContent = `${cLon.toFixed(5)} °`;
    if ($('altitude-gps')) $('altitude-gps').textContent = `${kAlt_new !== null ? kAlt_new.toFixed(2) : '--'} m`;
    if ($('gps-precision')) $('gps-precision').textContent = `${usedAcc.toFixed(2)} m`;
    
    if ($('vertical-speed')) $('vertical-speed').textContent = `${spdV.toFixed(2)} m/s`;
    if ($('speed-raw-ms')) $('speed-raw-ms').textContent = `${spd3D.toFixed(3)} m/s`;
    if ($('heading-display')) $('heading-display').textContent = pos.coords.heading !== null ? `${pos.coords.heading.toFixed(1)} °` : 'N/A';
    if ($('underground-status')) $('underground-status').textContent = kAlt_new !== null && kAlt_new < ALT_TH ? 'Oui 🔻' : 'Non';

    if ($('accel-long')) $('accel-long').textContent = `${accel_long.toFixed(3)} m/s ²`;
    if ($('force-g-long')) $('force-g-long').textContent = `${(accel_long / G_ACC).toFixed(2)} G`;
    if ($('kinetic-energy')) $('kinetic-energy').textContent = `${kineticEnergy.toFixed(2)} J`;
    if ($('mechanical-power')) $('mechanical-power').textContent = `${mechanicalPower.toFixed(2)} W`;

    if ($('distance-total-km')) $('distance-total-km').textContent = `${distKM_disp.toFixed(3)} km | ${distM_disp.toFixed(2)} m`;
    if ($('distance-light-s')) $('distance-light-s').textContent = `${(distM / C_L).toExponential(2)} s lumière`;
    if ($('distance-cosmic')) $('distance-cosmic').textContent = `${(distM / 149597870700).toExponential(2)} UA | ${(distM / 9460730472580800).toExponential(2)} al`;
    
    if ($('perc-speed-sound')) $('perc-speed-sound').textContent = `${(sSpdFE / C_S * 100).toFixed(2)} %`;
    if ($('perc-speed-c')) $('perc-speed-c').textContent = `${(sSpdFE / C_L * 100).toExponential(2)} %`;
    
    if ($('kalman-uncert')) $('kalman-uncert').textContent = `${kUncert.toFixed(3)} m²/s² (P)`;
    if ($('speed-error-perc')) $('speed-error-perc').textContent = `${R_dyn.toFixed(3)} m² (R dyn)`;
    if ($('mode-ratio')) $('mode-ratio').textContent = `${R_FACTOR_RATIO.toFixed(3)} (Ratio)`;
    if ($('env-factor')) $('env-factor').textContent = `${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY} (x${ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT.toFixed(1)})`;
    if ($('gps-accuracy-forced')) $('gps-accuracy-forced').textContent = `${gpsAccuracyOverride.toFixed(6)} m`;
    
    updateMap(cLat, cLon, usedAcc);
    
    // SAUVEGARDE DES VALEURS POUR LA PROCHAINE ITÉRATION
    lPos = pos; 
    lPos.speedMS_3D = spd3D; 
    lPos.timestamp = cTimePos; 
    lPos.kAlt_old = kAlt_new; 
}

// --- INITIALISATION DES ÉVÉNEMENTS DOM ---

document.addEventListener('DOMContentLoaded', () => {
    
    initMap(); 
    
    // Écouteur pour la masse
    const massInput = document.createElement('input');
    massInput.type = 'number'; massInput.id = 'mass-input'; massInput.value = '70'; massInput.step = '0.1';
    massInput.style.width = '60px'; 
    massInput.onchange = () => { currentMass = parseFloat(massInput.value); $('mass-display').textContent = `${currentMass.toFixed(3)} kg`; };
    
    if ($('mass-display')) {
        const parent = $('mass-display').parentNode;
        parent.innerHTML = ''; 
        parent.appendChild(document.createTextNode('Masse (kg)'));
        const spanValue = document.createElement('span');
        spanValue.id = 'mass-display';
        spanValue.textContent = `${currentMass.toFixed(3)} kg`;
        parent.appendChild(spanValue);
        parent.appendChild(massInput); 
    }

    // Écouteur pour le sélecteur de Corps Céleste
    if ($('celestial-body-select')) $('celestial-body-select').addEventListener('change', (e) => { 
        updateCelestialBody(e.target.value); 
    });

    // Écouteurs pour la Gravité Artificielle (Rotation)
    const updateRotation = () => {
        rotationRadius = parseFloat($('rotation-radius')?.value) || 0;
        angularVelocity = parseFloat($('angular-velocity')?.value) || 0;
        if (currentCelestialBody === 'ROTATING') {
            updateCelestialBody('ROTATING'); 
        }
    };
    if ($('rotation-radius')) $('rotation-radius').addEventListener('change', updateRotation);
    if ($('angular-velocity')) $('angular-velocity').addEventListener('change', updateRotation);

    // Écouteurs pour les contrôles EKF
    if ($('environment-select')) $('environment-select').addEventListener('change', (e) => { 
        if (emergencyStopActive) return;
        selectedEnvironment = e.target.value; 
        if ($('env-factor')) $('env-factor').textContent = `${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY} (x${ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT.toFixed(1)})`; 
    });
    if ($('ref-radius-input')) $('ref-radius-input').addEventListener('change', (e) => {
        R_ALT_CENTER_REF = parseFloat(e.target.value);
        console.log(`Rayon de référence réglé à: ${R_ALT_CENTER_REF}`);
    });
    if ($('gps-accuracy-override')) $('gps-accuracy-override').addEventListener('change', (e) => {
        gpsAccuracyOverride = parseFloat(e.target.value);
    });

    // Contrôles d'état
    if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', () => { 
        if (wID === null) { startGPS(); } else { stopGPS(); }
    });
    if ($('freq-select')) $('freq-select').addEventListener('change', (e) => setGPSMode(e.target.value));

    // Contrôles d'arrêt et de réinitialisation
    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', () => { 
        if (!emergencyStopActive) { emergencyStop(); } else { resumeSystem(); }
    });
    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        distM = 0; timeMoving = 0; 
        if ($('distance-total-km')) $('distance-total-km').textContent = `0.000 km | 0.00 m`; 
        if ($('speed-avg-moving')) $('speed-avg-moving').textContent = `0.00000 km/h`; 
        if ($('time-moving')) $('time-moving').textContent = `0.00 s`; 
    });
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        maxSpd = 0; 
        if ($('speed-max')) $('speed-max').textContent = `0.00000 km/h`; 
    });
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { 
        if (emergencyStopActive) return; 
        if (confirm("Êtes-vous sûr de vouloir tout réinitialiser?")) { 
            distM = 0; maxSpd = 0; timeMoving = 0; 
            kSpd = 0; kUncert = 1000; 
            kAlt = null; kAltUncert = 10;
        } 
    });
    
    // Contrôle du mode Nuit/Jour
    if ($('toggle-mode-btn')) $('toggle-mode-btn').addEventListener('click', () => {
        document.body.classList.toggle('dark-mode');
        // Fallback pour la localisation Astro si le GPS n'a pas encore de position
        const fallbackLat = lat || 43.296; 
        const fallbackLon = lon || 5.370;
        updateAstro(fallbackLat, fallbackLon);
    });
    
    // 🌐 Synchronisation initiale de l'heure NTP/UTC
    syncH(); 

    // 🔄 Boucle de mise à jour lente du DOM (Météo et Astro)
    if (domID === null) {
        domID = setInterval(() => {
            const fallbackLat = lat || 43.296;
            const fallbackLon = lon || 5.370;
            updateAstro(fallbackLat, fallbackLon);
            fetchWeather();
        }, DOM_SLOW_UPDATE_MS); 
    }
    
    // 🚀 Démarrage du système
    updateCelestialBody(currentCelestialBody); // Initialise la gravité et le rayon de référence
    startGPS(); // Démarre la surveillance de la position
});
