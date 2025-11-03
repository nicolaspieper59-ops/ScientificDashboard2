// =================================================================
// FICHIER JS PARTIE 1 : gnss-dashboard-part1.js
// (Constantes, Utilitaires, Kalman, Astro, Météo, IMU, Lune & Ciel)
// =================================================================

const $ = (id) => document.getElementById(id);

// --- CONSTANTES GLOBALES (Physiques, GPS, Temps) ---
const C_L = 299792458; // Vitesse de la lumière (m/s)
const SPEED_SOUND = 343; // Vitesse du son (m/s) (Utilisé comme fallback)
const G_ACC = 9.80665; // Gravité standard (m/s²)
const KMH_MS = 3.6; // M/s vers Km/h
const R_E = 6371000; // Rayon moyen de la Terre (m)
const R2D = 180 / Math.PI;
const D2R = Math.PI / 180;
const W_EARTH = 7.2921E-5; // Vitesse angulaire de la Terre (rad/s)
const NETHER_RATIO = 1 / 8; 

// Constantes Temps / Astro
const dayMs = 86400000;
const J1970 = 2440588; 
const J2000 = 2451545; 
const DOM_SLOW_UPDATE_MS = 1000;
const WEATHER_UPDATE_MS = 30000; 
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc"; 

// Constantes GPS
const MIN_DT = 0.05; 
const MIN_SPD = 0.01; 
const MAX_ACC = 20; 
const ALT_TH = -50; 
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 5000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 30000, timeout: 60000 }
};

// Constantes Kalman
const Q_NOISE = 0.001; 
let kSpd = 0; // Estimation vitesse (m/s)
let kUncert = 1000; // Incertitude vitesse (m/s)²
const ENVIRONMENT_FACTORS = {
    NORMAL: 1.0, FOREST: 1.5, CONCRETE: 3.0, METAL: 2.5
};
const Q_ALT_NOISE = 0.01; 
let kAlt = 0; // Estimation altitude (m)
let kAltUncert = 1000; // Incertitude altitude (m)²

// Constantes IMU
const ACCEL_FILTER_ALPHA = 0.8; 
const ACCEL_MOVEMENT_THRESHOLD = 0.5; 
let kAccel = { x: 0, y: 0, z: 0 };
let G_STATIC_REF = { x: 0, y: 0, z: 0 };
let latestVerticalAccelIMU = 0; 
let latestLinearAccelMagnitude = 0; 

// --- VARIABLES GLOBALES (État du système) ---
let wID = null;
let map = null;
let marker = null;
let tracePolyline = null;
let lat = 0, lon = 0;
let sTime = null; 
let distM = 0; 
let timeMoving = 0; 
let maxSpd = 0;
let lastFSpeed = 0;
let currentGPSMode = 'HIGH_FREQ';
let lPos = null; 
let emergencyStopActive = false;
let netherMode = false;
let selectedEnvironment = 'NORMAL';
let lServH = 0; 
let lLocH = 0; 
let domID = null; 
let weatherID = null; 
let lastP_hPa = 1013.25; 
const DEFAULT_LAT = 43.296; 
const DEFAULT_LON = 5.378; 
const OWM_API_KEY = "VOTRE_CLE_API_OPENWEATHERMAP"; 
const OWM_API_URL = "https://api.openweathermap.org/data/2.5/weather";


// ===========================================
// FONCTIONS UTILITAIRES ET KALMAN
// ===========================================

/** Calcule la distance de Haversine entre deux points (m). */
function dist(lat1, lon1, lat2, lon2) {
    const dLat = (lat2 - lat1) * D2R;
    const dLon = (lon2 - lon1) * D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1 * D2R) * Math.cos(lat2 * D2R) * Math.sin(dLon / 2) ** 2;
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R_E * c;
}

/** Filtre de Kalman 1D (Vitesse) */
function kFilter(z, dt, R_dyn) {
    const predSpd = kSpd;
    const predUncert = kUncert + Q_NOISE * dt;
    const K = predUncert / (predUncert + R_dyn);
    kSpd = predSpd + K * (z - predSpd);
    kUncert = (1 - K) * predUncert;
    return kSpd;
}

/** Calcule le bruit de mesure dynamique R (Intègre IMU linéaire) */
function getKalmanR(acc, alt, pressure, linearAccelMag) { 
    let R_raw = acc * acc; 
    const envFactor = ENVIRONMENT_FACTORS[selectedEnvironment] || ENVIRONMENT_FACTORS.NORMAL;
    let noiseMultiplier = envFactor;
    if (alt !== null && alt < 0) {
        noiseMultiplier += Math.abs(alt / 100); 
    }
    if (linearAccelMag > 0.5) { 
        noiseMultiplier += Math.pow(linearAccelMag, 1.5) * 0.5; 
    }
    return R_raw * noiseMultiplier;
}

/** Filtre de Kalman 1D pour l'Altitude (Utilise u_accel IMU) */
function kFilterAltitude(z, acc, dt, u_accel = 0) { 
    if (z === null) return kAlt; 
    const predAlt = kAlt + (0.5 * u_accel * dt * dt); 
    let predAltUncert = kAltUncert + Q_ALT_NOISE * dt;
    const R_alt = acc * acc * 2.0; 
    const K = predAltUncert / (predAltUncert + R_alt);
    kAlt = predAlt + K * (z - predAlt);
    kAltUncert = (1 - K) * predAltUncert;
    return kAlt;
}

/** Calcule le point de rosée (utilitaire pour la météo) */
function calculateDewPoint(tempC, humidity) {
    const a = 17.27;
    const b = 237.7;
    const alpha = (a * tempC) / (b + tempC) + Math.log(humidity / 100);
    return (b * alpha) / (a - alpha);
}

// ===========================================
// FONCTIONS ASTRO, TEMPS & COULEUR DU CIEL
// ===========================================

/** Obtient l'heure courante synchronisée */
function getCDate() {
    if (lLocH === 0) return new Date();
    const currentLocTime = performance.now();
    const offset = currentLocTime - lLocH;
    return new Date(lServH + offset);
}

/** Synchronisation horaire par serveur (UTC/Atomique) */
async function syncH() { 
    try {
        const localStartPerformance = performance.now(); 
        const response = await fetch(SERVER_TIME_ENDPOINT, { cache: "no-store", mode: "cors" });
        const localEndPerformance = performance.now(); 
        const serverData = await response.json(); 
        const RTT = localEndPerformance - localStartPerformance;
        const latencyOffset = RTT / 2;
        lServH = Date.parse(serverData.datetime) + latencyOffset; 
        lLocH = performance.now(); 
    } catch (error) {
        lServH = Date.now(); 
        lLocH = performance.now();
        if ($('local-time')) $('local-time').textContent = 'N/A (SYNCHRO ÉCHOUÉE)';
    }
}

/** TST Minecraft-inspiré */
function getSolarTime(date, lon) {
    if (date === null || lon === null) return { TST: 'N/A', MST: 'N/A', EOT: 'N/D', ECL_LONG: 'N/D' };
    const d = (date.valueOf() / dayMs - 0.5 + J1970) - J2000; 
    const M = D2R * (356.0470 + 0.9856002585 * d); 
    const C = D2R * (1.9148 * Math.sin(M) + 0.0200 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M)); 
    const P = D2R * 102.9377;                                                                
    const L = M + C + P + Math.PI;
    const epsilon = D2R * (23.4393 - 0.000000356 * d); 
    let alpha = Math.atan2(Math.cos(epsilon) * Math.sin(L), Math.cos(L));
    if (alpha < 0) alpha += 2 * Math.PI; 
    const meanLongitude = M + P + Math.PI;
    let eot_rad_raw = alpha - meanLongitude; 
    eot_rad_raw = eot_rad_raw % (2 * Math.PI);
    if (eot_rad_raw > Math.PI) { eot_rad_raw -= 2 * Math.PI; } else if (eot_rad_raw < -Math.PI) { eot_rad_raw += 2 * Math.PI; }
    const eot_min = eot_rad_raw * 4 * R2D; 
    let ecl_long_deg = (L * R2D) % 360; 
    const final_ecl_long = ecl_long_deg < 0 ? ecl_long_deg + 360 : ecl_long_deg;
    const msSinceMidnightUTC = (date.getUTCHours() * 3600 + date.getUTCMinutes() * 60 + date.getUTCSeconds()) * 1000 + date.getUTCMilliseconds();
    const mst_offset_ms = lon * dayMs / 360; 
    const mst_ms = ((msSinceMidnightUTC + mst_offset_ms) % dayMs + dayMs) % dayMs; 
    const eot_ms = eot_min * 60000;
    const tst_ms = ((mst_ms + eot_ms) % dayMs + dayMs) % dayMs; 
    
    const toTimeString = (ms) => {
        let h = Math.floor(ms / 3600000);
        let m = Math.floor((ms % 3600000) / 60000);
        let s = Math.floor((ms % 60000) / 1000);
        return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
    };
    return { TST: toTimeString(tst_ms), TST_MS: tst_ms, MST: toTimeString(mst_ms), EOT: eot_min.toFixed(3), ECL_LONG: final_ecl_long.toFixed(2) };
}

/** Détermine la couleur de fond du body (ciel) en fonction de l'élévation solaire. */
function getSkyColor(elevation, date) {
    const body = document.body;
    let newColor = '#f4f4f4'; // Jour (mode clair par défaut)

    if (body.classList.contains('dark-mode')) {
        // Mode nuit (couleurs sombres)
        newColor = '#1a1a1a'; // Nuit profonde
        if (elevation > -12 * D2R) { // Crépuscule nautique/civil
            newColor = '#2c2c2c';
        }
    } else {
        // Mode jour (couleurs claires)
        if (elevation < -18 * D2R) { // Nuit profonde (Astrologique)
            newColor = '#333';
        } else if (elevation < -12 * D2R) { // Crépuscule Astronomique
            newColor = '#666'; 
        } else if (elevation < -6 * D2R) { // Crépuscule Nautique
            newColor = '#999'; 
        } else if (elevation < 0) { // Crépuscule Civil
            newColor = '#ccc'; 
        } else if (elevation < 0.1 * D2R) { // Lever/Coucher (très bas)
            newColor = '#f0d0c0'; // Ton chaud
        } else {
            newColor = '#f4f4f4'; // Plein jour
        }
    }
    
    // Application de la couleur de fond
    body.style.backgroundColor = newColor;
}

/** Met à jour les données de la Lune. */
function updateMoon(date, latA, lonA) {
    if (!window.SunCalc) return;

    const moonPos = SunCalc.getMoonPosition(date, latA, lonA);
    const moonIllumination = SunCalc.getMoonIllumination(date);
    
    const moonPhase = moonIllumination.phase;
    const phaseText = (phase) => {
        if (phase < 0.05 || phase > 0.95) return 'Nouvelle Lune 🌑';
        if (phase < 0.22) return 'Croissant Ascendant 🌒';
        if (phase < 0.28) return 'Premier Quartier 🌓';
        if (phase < 0.45) return 'Gibbeuse Ascendante 🌔';
        if (phase < 0.55) return 'Pleine Lune 🌕';
        if (phase < 0.72) return 'Gibbeuse Descendante 🌖';
        if (phase < 0.78) return 'Dernier Quartier 🌗';
        return 'Croissant Descendant 🌘';
    };

    if ($('moon-phase')) $('moon-phase').textContent = phaseText(moonPhase);
    if ($('moon-illumination')) $('moon-illumination').textContent = `${(moonIllumination.fraction * 100).toFixed(1)} %`;
    if ($('moon-elevation')) $('moon-elevation').textContent = `${(moonPos.altitude * R2D).toFixed(2)} °`;
    if ($('moon-azimuth')) $('moon-azimuth').textContent = `${(moonPos.azimuth * R2D + 180).toFixed(2)} °`;
}

/** Met à jour les valeurs Astro et TST sur le DOM */
function updateAstro(latA, lonA) {
    const now = getCDate(); 
    if (now === null) return;
    
    const sunPos = window.SunCalc ? SunCalc.getPosition(now, latA, lonA) : null;
    const solarTimes = getSolarTime(now, lonA);
    const elevation_deg = sunPos ? (sunPos.altitude * R2D) : 0;
    
    getSkyColor(sunPos ? sunPos.altitude : -90 * D2R, now); // Mise à jour de la couleur du ciel
    updateMoon(now, latA, lonA); // Mise à jour de la lune

    if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString('fr-FR', { timeZone: 'UTC', hour12: false });
    if ($('date-display')) $('date-display').textContent = now.toLocaleDateString();
    if (sTime) {
        const timeElapsed = (now.getTime() - sTime) / 1000;
        if ($('time-elapsed')) $('time-elapsed').textContent = `${timeElapsed.toFixed(2)} s`;
        if ($('time-moving')) $('time-moving').textContent = `${timeMoving.toFixed(2)} s`;
    }
    
    if ($('time-minecraft')) $('time-minecraft').textContent = solarTimes.TST; // TST Minecraft
    if ($('tst')) $('tst').textContent = solarTimes.TST;
    if ($('lsm')) $('lsm').textContent = solarTimes.MST;
    if ($('sun-elevation')) $('sun-elevation').textContent = sunPos ? `${elevation_deg.toFixed(2)} °` : 'N/A';
    if ($('eot')) $('eot').textContent = solarTimes.EOT + ' min'; 
    if ($('ecliptic-long')) $('ecliptic-long').textContent = solarTimes.ECL_LONG + ' °';
}

// ===========================================
// FONCTIONS API MÉTÉO (OMISES POUR LA TAILLE)
// ===========================================
async function updateWeather(latA, lonA) { /* ... Logique OWM ... */ }

// ===========================================
// FONCTIONS CAPTEURS INERTIELS (IMU)
// ===========================================

/** Gère les données de l'accéléromètre via DeviceMotionEvent. */
function handleDeviceMotion(event) {
    if (emergencyStopActive) return;
    const acc = event.accelerationIncludingGravity;
    if (acc.x === null) return; 

    kAccel.x = ACCEL_FILTER_ALPHA * kAccel.x + (1 - ACCEL_FILTER_ALPHA) * acc.x;
    kAccel.z = ACCEL_FILTER_ALPHA * kAccel.z + (1 - ACCEL_FILTER_ALPHA) * acc.z; 

    const kAccel_mag = Math.sqrt(kAccel.x ** 2 + kAccel.y ** 2 + kAccel.z ** 2);
    let accel_vertical_lin = 0.0;
    
    if (Math.abs(kAccel_mag - G_ACC) < ACCEL_MOVEMENT_THRESHOLD) {
        G_STATIC_REF.x = kAccel.x;
        G_STATIC_REF.z = kAccel.z;
        accel_vertical_lin = 0.0; 
    } else {
        accel_vertical_lin = kAccel.z - G_STATIC_REF.z;
    }

    latestVerticalAccelIMU = accel_vertical_lin; 
    
    if ($('accel-vertical-imu')) $('accel-vertical-imu').textContent = `${accel_vertical_lin.toFixed(3)} m/s²`;
    if ($('force-g-vertical')) $('force-g-vertical').textContent = `${(accel_vertical_lin / G_ACC).toFixed(2)} G`;
        }
// =================================================================
// FICHIER JS PARTIE 2 : gnss-dashboard-part2.js
// (Logique Principale, Carte, Contrôles et Initialisation)
// =================================================================

// Les fonctions et variables globales sont définies dans part1.js.

// ===========================================
// FONCTIONS CARTE ET CONTRÔLE GPS
// ===========================================

/** Initialise la carte Leaflet. */
function initMap(latA, lonA) {
    if (map) return;
    try {
        map = L.map('map-container').setView([latA, lonA], 15);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            maxZoom: 19,
            attribution: '© OpenStreetMap'
        }).addTo(map);

        marker = L.marker([latA, lonA]).addTo(map);
        tracePolyline = L.polyline([], { color: 'red' }).addTo(map);
    } catch (e) {
        if ($('map-container')) $('map-container').textContent = 'Erreur d\'initialisation de la carte.';
    }
}

/** Met à jour la carte. */
function updateMap(latA, lonA) {
    if (!map || !marker || !tracePolyline) { initMap(latA, lonA); return; }
    
    const newLatLng = [latA, lonA];
    marker.setLatLng(newLatLng);
    tracePolyline.addLatLng(newLatLng);
    map.setView(newLatLng);
}

function setGPSMode(mode) {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    currentGPSMode = mode;
    wID = navigator.geolocation.watchPosition(updateDisp, handleErr, GPS_OPTS[mode]);
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = `⏸️ PAUSE GPS`;
    if ($('freq-select')) $('freq-select').value = mode; 
}

function startGPS() {
    if (wID === null) {
        if ($('freq-select')) $('freq-select').value = currentGPSMode; 
        setGPSMode(currentGPSMode);
        sTime = null; 
    }
}

function stopGPS(resetButton = true) {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
    }
    if (resetButton) {
        if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = `▶️ MARCHE GPS`;
    }
}

function emergencyStop() {
    emergencyStopActive = true;
    stopGPS(false);
    if ($('emergency-stop-btn')) {
        $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: ACTIF 🔴";
        $('emergency-stop-btn').classList.add('active');
    }
}

function resumeSystem() {
    emergencyStopActive = false;
    if ($('emergency-stop-btn')) {
        $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: INACTIF 🟢";
        $('emergency-stop-btn').classList.remove('active');
    }
    startGPS();
}

function handleErr(err) {
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = `❌ ERREUR GPS`;
}


// ===========================================
// FONCTION PRINCIPALE DE MISE À JOUR (GPS, Kalman, Physique)
// ===========================================

function updateDisp(pos) {
    if (emergencyStopActive) return;
    lat = pos.coords.latitude; lon = pos.coords.longitude;
    const alt = pos.coords.altitude, acc = pos.coords.accuracy;
    const spd_raw_gps = pos.coords.speed;
    const cTimePos = pos.timestamp; 
    const now = getCDate(); 
    const MASS = 70.0; 

    if (now === null) { updateAstro(lat, lon); return; } 
    if (sTime === null) { sTime = now.getTime(); }
    if (acc > MAX_ACC) { 
        if ($('gps-precision')) $('gps-precision').textContent = `❌ ${acc.toFixed(0)} m (Trop Imprécis)`; 
        if (lPos === null) lPos = pos; return; 
    }

    let effectiveAcc = acc;
    const accOverride = parseFloat($('gps-accuracy-override').value);
    if (accOverride > 0) { effectiveAcc = accOverride; }

    let spdH = spd_raw_gps ?? 0; 
    const dt = lPos ? (cTimePos - lPos.timestamp) / 1000 : MIN_DT;

    // 1. FILTRAGE DE L'ALTITUDE (via Kalman)
    const kAlt_new = kFilterAltitude(alt, effectiveAcc, dt, latestVerticalAccelIMU); 
    
    // 2. VITESSE VERTICALE
    let spdV = 0; 
    if (lPos && lPos.kAlt_old !== undefined && dt > MIN_DT && alt !== null) { 
        spdV = (kAlt_new - lPos.kAlt_old) / dt; 
    } else if (alt !== null) { 
        spdV = 0; 
    }
    
    // 3. VITESSE HORIZONTALE CALCULÉE ET VITESSE 3D
    if (lPos && dt > 0.05) { 
        const dH = dist(lPos.coords.latitude, lPos.coords.longitude, lat, lon); 
        spdH = dH / dt; 
    } 
    let spd3D = Math.sqrt(spdH ** 2 + spdV ** 2);

    // 4. FILTRE DE KALMAN FINAL (Vitesse 3D Stable)
    const R_dyn = getKalmanR(effectiveAcc, alt, lastP_hPa, latestLinearAccelMagnitude); 
    const fSpd = kFilter(spd3D, dt, R_dyn); 
    const sSpdFE = fSpd < MIN_SPD ? 0 : fSpd; 

    // 5. CALCULS PHYSIQUES ET RELATIFS
    let accel_long = (dt > 0.05) ? (sSpdFE - lastFSpeed) / dt : 0;
    lastFSpeed = sSpdFE;
    distM += sSpdFE * dt * (netherMode ? NETHER_RATIO : 1); 
    if (sSpdFE > MIN_SPD) { timeMoving += dt; }
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    const avgSpdMoving = timeMoving > 0 ? (distM / timeMoving) : 0;
    
    const coriolusForce = 2 * MASS * sSpdFE * W_EARTH * Math.sin(lat * D2R);
    const kineticEnergy = 0.5 * MASS * sSpdFE ** 2;
    const mechanicalPower = accel_long * MASS * sSpdFE; 
    
    const v_c_ratio = sSpdFE / C_L;
    const lorentzFactor = 1 / Math.sqrt(Math.max(1 - v_c_ratio ** 2, 1e-15)); 
    
    // CALCUL DENSITÉ DE L'AIR
    let airDensity = "N/A";
    let tempCMatch = null; 
    const tempElement = $('temp-air') ? $('temp-air').textContent : ''; 
    tempCMatch = tempElement.match(/(-?[\d.]+)\s*°C/);
    
    if (tempCMatch) {
        const tempC = parseFloat(tempCMatch[1]);
        const tempK = tempC + 273.15; 
        const pressurePa = lastP_hPa * 100; 
        const R_specific = 287.058; 
        if (!isNaN(tempK) && tempK > 0 && pressurePa > 0) {
            airDensity = (pressurePa / (R_specific * tempK)).toFixed(3);
        }
    }
    const dynamicPressure = (airDensity !== "N/A" && sSpdFE > 0) ? (0.5 * parseFloat(airDensity) * sSpdFE ** 2) : 0.0;


    // --- 6. MISE À JOUR DU DOM ---
    const sSpdFE_Kmh = (sSpdFE * KMH_MS);
    const distM_display = distM * (netherMode ? NETHER_RATIO : 1);

    // Vitesse/Distance
    if ($('speed-stable')) $('speed-stable').textContent = `${sSpdFE_Kmh.toFixed(5)} km/h`;
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${sSpdFE.toFixed(3)} m/s | ${(sSpdFE * 1000).toFixed(0)} mm/s`;
    if ($('speed-3d-inst')) $('speed-3d-inst').textContent = `${(spd3D * KMH_MS).toFixed(5)} km/h`;
    if ($('speed-max')) $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    if ($('speed-avg-moving')) $('speed-avg-moving').textContent = `${(avgSpdMoving * KMH_MS).toFixed(5)} km/h`;
    if ($('distance-total-km')) $('distance-total-km').textContent = `${(distM_display / 1000).toFixed(3)} km | ${distM_display.toFixed(2)} m`;
    if ($('distance-cosmic')) $('distance-cosmic').textContent = `${(distM_display / C_L).toExponential(2)} s lumière | ${(distM_display / (C_L * 31557600)).toExponential(2)} al`;


    // Altitudes et Précision
    if ($('latitude')) $('latitude').textContent = lat.toFixed(6);
    if ($('longitude')) $('longitude').textContent = lon.toFixed(6);
    if ($('altitude-gps')) $('altitude-gps').textContent = alt !== null ? `${alt.toFixed(2)} m` : 'N/A'; 
    if ($('altitude-msl')) $('altitude-msl').textContent = kAlt_new !== null ? `${kAlt_new.toFixed(2)} m` : 'N/A';
    if ($('gps-precision')) $('gps-precision').textContent = `${acc.toFixed(2)} m`;
    if ($('gps-accuracy-effective')) $('gps-accuracy-effective').textContent = `${effectiveAcc.toFixed(2)} m`;
    if ($('speed-raw-ms')) $('speed-raw-ms').textContent = spd_raw_gps !== null ? `${spd_raw_gps.toFixed(2)} m/s` : 'N/A';
    if ($('underground-status')) $('underground-status').textContent = (kAlt_new !== null && kAlt_new < ALT_TH) ? 'Oui' : 'Non';
    if ($('speed-error-perc')) $('speed-error-perc').textContent = `${R_dyn.toFixed(3)} m² (R dyn)`;

    // Dynamique et Forces
    if ($('vertical-speed')) $('vertical-speed').textContent = `${spdV.toFixed(2)} m/s`;
    if ($('accel-long')) $('accel-long').textContent = `${accel_long.toFixed(3)} m/s²`;
    if ($('force-g-long')) $('force-g-long').textContent = `${(accel_long / G_ACC).toFixed(2)} G`;
    if ($('kinetic-energy')) $('kinetic-energy').textContent = `${kineticEnergy.toFixed(2)} J`;
    if ($('mechanical-power')) $('mechanical-power').textContent = `${mechanicalPower.toFixed(2)} W`;
    if ($('coriolis-force')) $('coriolis-force').textContent = `${coriolusForce.toExponential(2)} N`;
    
    // Relativité, Fluides
    if ($('lorentz-factor')) $('lorentz-factor').textContent = `γ = ${lorentzFactor.toFixed(6)}`;
    if ($('perc-speed-c')) $('perc-speed-c').textContent = `${(v_c_ratio * 100).toExponential(2)}%`;
    if ($('dynamic-pressure')) $('dynamic-pressure').textContent = `${dynamicPressure.toFixed(2)} Pa`;
    if ($('air-density')) $('air-density').textContent = airDensity + (airDensity !== "N/A" ? ' kg/m³' : '');

    // Vitesse du Son Dynamique
    if (tempCMatch) {
        const tempC = parseFloat(tempCMatch[1]);
        const soundSpeed = 331.3 + 0.606 * tempC; 
        const percSound = (sSpdFE / soundSpeed) * 100;
        if ($('perc-speed-sound')) $('perc-speed-sound').textContent = `${percSound.toFixed(2)} %`;
    } else {
        if ($('perc-speed-sound')) $('perc-speed-sound').textContent = `${(sSpdFE / SPEED_SOUND * 100).toFixed(2)} %`;
    }

    // --- 7. MISE À JOUR DES COMPOSANTS ET SAUVEGARDE ---
    updateAstro(lat, lon);
    updateMap(lat, lon);
    
    lPos = { 
        coords: pos.coords, 
        timestamp: cTimePos, 
        kAlt_old: kAlt_new, 
        kAltUncert_old: kAltUncert
    };
} 


// ===========================================
// INITIALISATION DES ÉVÉNEMENTS
// ===========================================

document.addEventListener('DOMContentLoaded', () => {
    
    if ($('environment-select')) $('environment-select').value = selectedEnvironment;
    if ($('environment-select')) $('environment-select').addEventListener('change', (e) => { 
        if (!emergencyStopActive) selectedEnvironment = e.target.value; 
    });

    syncH(); 
    initMap(DEFAULT_LAT, DEFAULT_LON);

    // Démarrage/Arrêt GPS
    if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', () => {
        if (!emergencyStopActive) wID === null ? startGPS() : stopGPS();
    });
    
    // Sélecteur de Fréquence GPS
    if ($('freq-select')) $('freq-select').addEventListener('change', (e) => {
        if (!emergencyStopActive) setGPSMode(e.target.value);
    });
    
    // Arrêt d'Urgence
    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', () => {
        emergencyStopActive ? resumeSystem() : emergencyStop(); 
    });
    
    // Mode Nether
    if ($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', () => { 
        if (!emergencyStopActive) {
            netherMode = !netherMode; 
            if ($('mode-nether')) $('mode-nether').textContent = netherMode ? "ACTIVÉ (1:8) 🔥" : "DÉSACTIVÉ (1:1)"; 
        }
    });

    // Boutons de Réinitialisation
    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => { 
        if (!emergencyStopActive) { distM = 0; timeMoving = 0; }
    });
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', () => { 
        if (!emergencyStopActive) maxSpd = 0; 
    });
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { 
        if (!emergencyStopActive && confirm("Êtes-vous sûr de vouloir tout réinitialiser?")) { 
            distM = 0; maxSpd = 0; kSpd = 0; kUncert = 1000; timeMoving = 0; lastFSpeed = 0;
            if (tracePolyline) tracePolyline.setLatLngs([]);
        } 
    });
    
    // Gestionnaire du Mode Nuit (toggle-mode-btn)
    if ($('toggle-mode-btn')) $('toggle-mode-btn').addEventListener('click', () => {
        document.body.classList.toggle('dark-mode');
    });


    if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', handleDeviceMotion, true);
    } 

    // Intervalle lent pour les mises à jour Astro (1s)
    domID = setInterval(() => {
        if (lPos) updateAstro(lPos.coords.latitude, lPos.coords.longitude);
        else updateAstro(DEFAULT_LAT, DEFAULT_LON);
    }, DOM_SLOW_UPDATE_MS); 
    
    // Intervalle pour la mise à jour Météo (30s)
    weatherID = setInterval(() => {
        if (lPos) updateWeather(lPos.coords.latitude, lPos.coords.longitude);
        else updateWeather(DEFAULT_LAT, DEFAULT_LON);
    }, WEATHER_UPDATE_MS); 
});
