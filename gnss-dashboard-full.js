// =================================================================
// FICHIER JS PARTIE 1/2 : gnss-dashboard-part1.js (Final V11.1)
// CORE, KALMAN, CONSTANTES & ÉTAT GLOBAL
// =================================================================

// --- CLÉS D'API & PROXY VERCEL (Non utilisés en mode Hors Ligne pour la Météo/NTP) ---
// *Si HORS LIGNE, ces appels échoueront et le dashboard utilisera l'horloge locale.*
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app"; 
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc"; 

// --- CONSTANTES GLOBALES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI; 
const C_L = 299792458; // Vitesse de la lumière (m/s)
const R_E = 6371000;   // Rayon terrestre moyen (m)
const KMH_MS = 3.6;    // Facteur m/s -> km/h
const C_S = 343;       // Vitesse du son standard (m/s)
const G_ACC = 9.80665; // Accélération de la gravité (m/s²)
const dayMs = 1000 * 60 * 60 * 24; 
const J1970 = 2440588, J2000 = 2451545; 
const MIN_DT = 0.01; 
const MSL_OFFSET_M = 40; 
const NETHER_RATIO = 8; 
const ALT_TH = -50; 
const DOM_SLOW_UPDATE_MS = 1000; 

// PARAMÈTRES AVANCÉS DU FILTRE DE KALMAN
const Q_NOISE = 0.01;       
const R_MIN = 0.05, R_MAX = 50.0; 
const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0 },
    'METAL': { R_MULT: 2.5 },      
    'FOREST': { R_MULT: 1.5 },     
    'CONCRETE': { R_MULT: 3.0 },   
};
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};

// --- VARIABLES D'ÉTAT ---
let wID = null, domID = null, lPos = null; 
let lat = null, lon = null, sTime = null; 
let distM = 0, distMStartOffset = 0, maxSpd = 0; 
let kSpd = 0, kUncert = 1000; 
let timeMoving = 0, timeElapsed = 0; 
let lServH = null, lLocH = null; 
let lastFSpeed = 0; 
let mslOffset = MSL_OFFSET_M; 
let lastP_hPa = null, lastT_K = null, lastH_perc = null; 
let emergencyStopActive = false;
let netherMode = false;
let selectedEnvironment = 'NORMAL'; 
let currentGPSMode = 'HIGH_FREQ'; 

// NOUVEAU : MASSE ET ACCÉLÉRATION
let DEVICE_MASS_KG = 70; // Masse par défaut (70 kg)
let LATERAL_ACCEL_MS2 = 0.0; // Accélération latérale estimée/mesurée

const $ = id => document.getElementById(id);


// ===========================================
// FONCTIONS GÉO & UTILS (dist, getCDate, correctAltitudeToMSL)
// ===========================================

/** ⌚ Obtient la date/heure corrigée (NTP si synchro ok) */
function getCDate() { 
    // Utilise l'horloge locale si la synchro NTP a échoué (mode hors ligne)
    return sTime ? new Date(Date.now() - (lLocH.getTime() - lServH.getTime())) : new Date();
}

/** Calcule la distance 3D Haversine entre deux positions. */
function dist(pos1, pos2) {
    const lat1 = pos1.coords.latitude * D2R;
    const lat2 = pos2.coords.latitude * D2R;
    const dLat = (pos2.coords.latitude - pos1.coords.latitude) * D2R;
    const dLon = (pos2.coords.longitude - pos1.coords.longitude) * D2R;

    const a = Math.sin(dLat / 2) ** 2 +
              Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    
    const distance2D = R_E * c;
    
    const dAlt = (pos2.coords.altitude || 0) - (pos1.coords.altitude || 0);
    return Math.sqrt(distance2D ** 2 + dAlt ** 2);
}

/** 🌐 Corrige l'altitude WGS84 (GPS) en Altitude MSL (Niveau de la mer) */
function correctAltitudeToMSL(wgs84Alt, geoidHeight) {
    if (wgs84Alt === null) return null;
    return wgs84Alt - (geoidHeight ?? mslOffset);
}

/** Calcule le facteur R (Précision 3D Dynamique) du filtre de Kalman. */
function getKalmanR(acc, alt, P_hPa) {
    let R = acc ** 2; 
    R = Math.max(R_MIN, Math.min(R_MAX, R));
    
    const envFactor = ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT;
    
    // CORRECTION MÉTÉO (ne s'applique pas si P_hPa est null en hors ligne)
    if (P_hPa !== null) {
        const pressureDeviation = Math.abs(1013.25 - P_hPa);
        const pressureFactor = 1.0 + pressureDeviation / 1013.25 * 0.1;
        R *= Math.max(1.0, pressureFactor);
    }
    
    R *= envFactor;
    if (alt < ALT_TH) { R *= 2.0; } 

    return Math.max(R_MIN, Math.min(R_MAX, R));
}

/** Applique le filtre de Kalman à la vitesse 3D. */
function kFilter(z, dt, acc, alt, P_hPa) {
    if (isNaN(z) || z === null || dt < MIN_DT) {
        kUncert += Q_NOISE * dt;
        return kSpd;
    }
    
    // 1. Prédiction
    const kSpdPred = kSpd + lastFSpeed * dt; 
    const kUncertPred = kUncert + Q_NOISE * dt;

    // 2. Mise à jour (Correction)
    const R_dyn = getKalmanR(acc, alt, P_hPa);
    const K = kUncertPred / (kUncertPred + R_dyn);
    
    kSpd = kSpdPred + K * (z - kSpdPred);
    kUncert = (1 - K) * kUncertPred;

    lastFSpeed = (kSpd - kSpdPred) / dt || 0; 
    return kSpd;
}

function getDynamicMinSpd() {
    return 0.1; // Vitesse minimale en m/s pour être considéré en mouvement
    }
// =================================================================
// FICHIER JS PARTIE 2/2 : gnss-dashboard-part2.js (Final V11.1)
// ASTRO, MÉTÉO, CARTE, LOGIQUE DOM & INITIALISATION
// NÉCESSITE gnss-dashboard-part1.js et Leaflet/SunCalc (si en ligne)
// =================================================================

// --- CARTE LEAFLET (Variables globales) ---
let map = null;
let marker = null;

// ===========================================
// FONCTIONS ASTRO
// ===========================================

/** Convertit la date en jours depuis J2000. */
function toDays(date) { return (date.valueOf() / dayMs - 0.5 + J1970) - J2000; }

/** Calcule l'anomalie solaire moyenne (en radians). */
function solarMeanAnomaly(d) { return D2R * (356.0470 + 0.98560028 * d); }

/**
 * ☀️ CORRIGÉ (V11.1) : Calcule TST, LSM, EOT Total et ses Composantes (Meeus's method).
 */
function getSolarTime(date, lon) {
    // ... (Logique getSolarTime V11.1 inchangée) ...
    if (date === null || lon === null) return { TST: 'N/A', MST: 'N/A', EOT: 'N/D', ECL_LONG: 'N/D' };

    const d = toDays(date);
    const M = solarMeanAnomaly(d); 
    const P = D2R * 102.9377;      

    const C_rad = D2R * (1.9148 * Math.sin(M) + 0.0200 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M));
    const eot_ecc_min = C_rad * R2D * 4; 
    
    let L = M + C_rad + P + Math.PI;
    L = L % (2 * Math.PI); 
    if (L < 0) L += 2 * Math.PI;

    let L_mean = M + P + Math.PI;
    L_mean = L_mean % (2 * Math.PI); 
    if (L_mean < 0) L_mean += 2 * Math.PI;

    const epsilon = D2R * (23.4393 - 0.000000356 * d);
    let alpha = Math.atan2(Math.cos(epsilon) * Math.sin(L), Math.cos(L));
    if (alpha < 0) alpha += 2 * Math.PI;

    let eot_rad = L_mean - alpha;
    eot_rad = eot_rad % (2 * Math.PI);
    if (eot_rad > Math.PI) eot_rad -= 2 * Math.PI;
    if (eot_rad < -Math.PI) eot_rad += 2 * Math.PI;
    
    const eot_min = eot_rad * R2D * 4; 

    const eot_obliq_min = eot_min - eot_ecc_min;

    const msSinceMidnightUTC = (date.getUTCHours() * 3600 + date.getUTCMinutes() * 60 + date.getUTCSeconds()) * 1000 + date.getUTCMilliseconds();
    const mst_offset_ms = (lon / 15) * 3600000; 
    const mst_ms = (msSinceMidnightUTC + mst_offset_ms + dayMs) % dayMs;
    const eot_ms = eot_min * 60000;
    const tst_ms = (mst_ms + eot_ms + dayMs) % dayMs;

    const toTimeString = (ms) => {
        let h = Math.floor(ms / 3600000);
        let m = Math.floor((ms % 3600000) / 60000);
        let s = Math.floor((ms % 60000) / 1000);
        return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
    };

    return { 
        TST: toTimeString(tst_ms), MST: toTimeString(mst_ms),
        EOT: eot_min.toFixed(4), ECL_LONG: (L * R2D).toFixed(4),
        EOT_ECC: eot_ecc_min.toFixed(4), EOT_OBLIQ: eot_obliq_min.toFixed(4)
    };
}


// ===========================================
// FONCTIONS CARTE (Leaflet)
// ===========================================

/** Initialise la carte Leaflet */
function initMap(latA, lonA) {
    if (typeof L === 'undefined' || map !== null) return; 

    // Tentative d'initialisation de la carte avec la première position
    try {
        map = L.map('map', { zoomControl: false }).setView([latA, lonA], 13);

        // Tuile OpenStreetMap (NÉCESSITE INTERNET)
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: '© OpenStreetMap contributors',
            maxZoom: 19
        }).addTo(map);

        marker = L.marker([latA, lonA]).addTo(map)
            .bindPopup("Position GPS").openPopup();
            
        // Zoom par défaut
        map.setZoom(17);

    } catch (e) {
        console.warn("Erreur d'initialisation Leaflet (Vérifiez la connexion Internet ou l'inclusion locale des tuiles):", e);
    }
}

/** Met à jour le marqueur et la vue de la carte */
function updateMap(latA, lonA, spd) {
    if (map === null || marker === null) {
        initMap(latA, lonA); // Tente l'initialisation si elle a échoué
        return;
    }
    
    const newLatLng = L.latLng(latA, lonA);

    // Déplace le marqueur
    marker.setLatLng(newLatLng);
    marker.setPopupContent(`Vitesse (EKF): ${(spd * KMH_MS).toFixed(2)} km/h`).openPopup();
    
    // Centre la vue sur le marqueur
    map.panTo(newLatLng);

    // Optionnel: Trace la trajectoire (non implémenté ici pour rester simple)
}


// ===========================================
// FONCTIONS MÉTÉO & SYNCHRONISATION (Mode HORS LIGNE)
// ===========================================

let lastWeatherFetch = 0;

/** Récupère les données météo et le temps (Échoue si Hors Ligne) */
async function fetchExternalData(latA, lonA) {
    // 1. Synchro NTP (Mode Hors Ligne : Échoue)
    try {
        const responseNTP = await fetch(SERVER_TIME_ENDPOINT);
        const dataNTP = await responseNTP.json();
        lLocH = new Date();
        lServH = new Date(dataNTP.utc_datetime);
        sTime = lServH; 
    } catch (e) {
        // En mode hors ligne, cette erreur est attendue. Utilise l'horloge locale.
        sTime = null; 
    }
    
    // 2. Météo (Mode Hors Ligne : Échoue)
    if (Date.now() - lastWeatherFetch < 60000) return; 
    
    try {
        const responseWeather = await fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${latA}&lon=${lonA}`);
        const dataWeather = await responseWeather.json();

        lastP_hPa = dataWeather.main.pressure;
        lastT_K = dataWeather.main.temp + 273.15; // Celsius à Kelvin
        lastH_perc = dataWeather.main.humidity / 100;

        if ($('temp-air')) $('temp-air').textContent = `${dataWeather.main.temp.toFixed(1)} °C`;
        if ($('pressure')) $('pressure').textContent = `${lastP_hPa.toFixed(0)} hPa`;
        if ($('humidity')) $('humidity').textContent = `${dataWeather.main.humidity.toFixed(0)} %`;
        
        lastWeatherFetch = Date.now();
    } catch (e) {
        console.warn("Météo et/ou NTP indisponibles (Mode Hors Ligne/Erreur):", e.message);
        if ($('temp-air')) $('temp-air').textContent = 'N/A (Hors Ligne)';
    }
}


// ===========================================
// BOUCLES ET MISE À JOUR DOM
// ===========================================

let lastUpdateTS = 0;

function updateDOMFast() {
    // ... (Logique updateDOMFast inchangée - calcul du temps et affichage vitesse) ...
    const now = getCDate();
    const dt_sec = (now.getTime() - lastUpdateTS) / 1000;
    
    if (!lastUpdateTS || dt_sec > 1) { 
        lastUpdateTS = now.getTime();
        return;
    }
    
    timeElapsed += dt_sec;

    const formatTime = (totalSeconds) => {
        const h = Math.floor(totalSeconds / 3600);
        const m = Math.floor((totalSeconds % 3600) / 60);
        const s = (totalSeconds % 60).toFixed(2);
        return `${String(h).padStart(2, '0')}h ${String(m).padStart(2, '0')}m ${String(Math.floor(s)).padStart(2, '0')}s`;
    };
    
    if (kSpd >= getDynamicMinSpd()) timeMoving += dt_sec;

    if ($('time-elapsed')) $('time-elapsed').textContent = formatTime(timeElapsed);
    if ($('time-moving')) $('time-moving').textContent = formatTime(timeMoving);
    
    const microMS = (kSpd * 1e6).toFixed(0);
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${kSpd.toFixed(2)} m/s | ${microMS} µm/s`;

    lastUpdateTS = now.getTime();
}

function updateAstro(latA, lonA) {
    // ... (Logique updateAstro inchangée - utilise getSolarTime V11.1) ...
    const now = getCDate(); 
    if (now === null || latA === null || lonA === null) {
        if ($('date-display')) $('date-display').textContent = 'N/A';
        return;
    }

    if ($('date-display')) $('date-display').textContent = now.toLocaleDateString();
    
    const solarTimes = getSolarTime(now, lonA); 
    const sunPos = window.SunCalc ? SunCalc.getPosition(now, latA, lonA) : null;
    const moonPos = window.SunCalc ? SunCalc.getMoonPosition(now, latA, lonA) : null;
    const moonIllum = window.SunCalc ? SunCalc.getMoonIllumination(now) : null;
    const sunTimes = window.SunCalc ? SunCalc.getTimes(now, latA, lonA) : null;
    
    if ($('time-solar-true')) $('time-solar-true').textContent = solarTimes.TST;
    if ($('culmination-lsm')) $('culmination-lsm').textContent = solarTimes.MST;
    if ($('eot-min')) $('eot-min').textContent = solarTimes.EOT + ' min'; 
    if ($('ecliptic-long')) $('ecliptic-long').textContent = solarTimes.ECL_LONG + ' °';
    if ($('eot-ecc-comp')) $('eot-ecc-comp').textContent = solarTimes.EOT_ECC + ' min';
    if ($('eot-obliq-comp')) $('eot-obliq-comp').textContent = solarTimes.EOT_OBLIQ + ' min';

    if (sunPos) {
        if ($('sun-elevation')) $('sun-elevation').textContent = (sunPos.altitude * R2D).toFixed(2) + ' °';
        if ($('sun-azimuth')) $('sun-azimuth').textContent = (sunPos.azimuth * R2D).toFixed(2) + ' °';
    }
    
    // ... (Durée du Jour et Lune) ...
    if (moonPos && moonIllum) {
        if ($('lunar-phase-perc')) $('lunar-phase-perc').textContent = (moonIllum.fraction * 100).toFixed(1) + ' %';
        if ($('moon-elevation')) $('moon-elevation').textContent = (moonPos.altitude * R2D).toFixed(2) + ' °';
        if ($('moon-azimuth')) $('moon-azimuth').textContent = (moonPos.azimuth * R2D).toFixed(2) + ' °';
    }
    
    if (window.SunCalc && latA !== null) {
        const moonRise = SunCalc.getMoonTimes(now, latA, lonA).rise;
        const moonSet = SunCalc.getMoonTimes(now, latA, lonA).set;
        const riseStr = moonRise ? moonRise.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit' }) : '--:--';
        const setStr = moonSet ? moonSet.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit' }) : '--:--';
        if ($('moon-times')) $('moon-times').textContent = `↑ ${riseStr} / ↓ ${setStr}`;
    }
}

/** Mise à jour principale des données GPS/EKF/Astro/Map */
function updateDisp(pos) {
    if (emergencyStopActive) return;

    const now = getCDate();
    let dt = 0;
    let speedRawMS = 0;
    
    lat = pos.coords.latitude;
    lon = pos.coords.longitude;
    
    const alt_gps_wgs = pos.coords.altitude || 0;
    const geoid_correction = mslOffset; 
    const alt_msl = correctAltitudeToMSL(alt_gps_wgs, geoid_correction);
    
    if (lPos) {
        dt = (pos.timestamp - lPos.timestamp) / 1000;
        const dist_step = dist(lPos, pos);
        
        if (dt > MIN_DT) {
            speedRawMS = dist_step / dt;
            distM += dist_step;
        }
    }
    lPos = pos;
    
    // Filtre EKF et Accélération Longitudinale
    const accuracy = pos.coords.accuracy || 5; 
    const kSpd_prev = kSpd;
    kSpd = kFilter(speedRawMS, dt, accuracy, alt_msl, lastP_hPa);
    const accel_long = (kSpd - kSpd_prev) / dt || 0;
    
    // NOUVEAU : Calculs de Dynamique du Véhicule (Force Centrifuge)
    // NOTE: LATERAL_ACCEL_MS2 doit être rempli par un capteur IMU ou une estimation GPS
    const LATERAL_ACCEL_MS2 = 0.0; // Placez ici la lecture d'un capteur (à implémenter)
    const centrifugalForce_N = LATERAL_ACCEL_MS2 * DEVICE_MASS_KG;

    // Mise à jour Max/Avg et Nether
    maxSpd = Math.max(maxSpd, kSpd);
    const dist_total_final = netherMode ? distM / NETHER_RATIO : distM;

    // --- MISE À JOUR DOM ---
    if ($('local-time')) $('local-time').textContent = now.toLocaleTimeString('fr-FR');
    if ($('speed-stable')) $('speed-stable').textContent = (kSpd * KMH_MS).toFixed(5);
    if ($('speed-max')) $('speed-max').textContent = (maxSpd * KMH_MS).toFixed(5);
    if ($('distance-total-km')) $('distance-total-km').textContent = `${(dist_total_final / 1000).toFixed(3)} km | ${dist_total_final.toFixed(2)} m`;
    if ($('latitude')) $('latitude').textContent = lat.toFixed(6);
    if ($('longitude')) $('longitude').textContent = lon.toFixed(6);
    if ($('altitude-gps')) $('altitude-gps').textContent = alt_msl.toFixed(2) + ' m (MSL)';
    if ($('gps-precision')) $('gps-precision').textContent = accuracy.toFixed(2) + ' m';
    if ($('accel-long')) $('accel-long').textContent = accel_long.toFixed(3) + ' m/s²';
    
    // Affichage Dynamique du Véhicule
    if ($('accel-lat')) $('accel-lat').textContent = LATERAL_ACCEL_MS2.toFixed(3) + ' m/s²';
    if ($('force-centrifugal')) $('force-centrifugal').textContent = centrifugalForce_N.toFixed(2) + ' N';
    if ($('force-centrifugal-g')) $('force-centrifugal-g').textContent = (LATERAL_ACCEL_MS2 / G_ACC).toFixed(3) + ' G';

    const R_dyn = getKalmanR(accuracy, alt_msl, lastP_hPa);
    if ($('speed-error-perc')) $('speed-error-perc').textContent = `${(kUncert / R_dyn * 100).toFixed(1)} % (${R_dyn.toFixed(2)})`;
    
    // Lancement des fonctions lentes (Astro/Météo/NTP)
    updateAstro(lat, lon);
    fetchExternalData(lat, lon);
    
    // Mise à jour de la carte Leaflet
    updateMap(lat, lon, kSpd);
}

// ... (fonctions handleGeolocationError, startGPS, stopGPS, resetDisp, etc. - inchangées) ...

function handleGeolocationError(error) {
    console.error("Erreur GPS:", error);
    // ...
}

function startGPS() {
    if ('geolocation' in navigator) {
        if (wID !== null) stopGPS(false);
        
        const options = GPS_OPTS[currentGPSMode];

        wID = navigator.geolocation.watchPosition(updateDisp, handleGeolocationError, options);
        domID = setInterval(updateDOMFast, 17); // 60 FPS
        
        // La synchro NTP est faite dans fetchExternalData pour le mode hors ligne
        
        if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = '⏸️ PAUSE GPS';
    } else {
        alert("La géolocalisation n'est pas supportée par ce navigateur.");
    }
}

function stopGPS(shouldReset = true) {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
    }
    if (domID !== null) {
        clearInterval(domID);
        domID = null;
    }

    if (shouldReset) {
        resetDisp(true);
    }
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = '▶️ MARCHE GPS';
}

function resetDisp(fullReset = false) {
    if (fullReset) {
        distM = 0; maxSpd = 0;
        timeElapsed = 0; timeMoving = 0;
    }
    kSpd = 0; kUncert = 1000;
    lastFSpeed = 0;
}

// ===========================================
// INITIALISATION ET ÉVÉNEMENTS
// ===========================================

function initGeolocation() {
    if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', () => wID === null ? startGPS() : stopGPS(false));
    
    if ($('freq-select')) $('freq-select').addEventListener('change', (e) => {
        currentGPSMode = e.target.value;
        if (wID !== null) startGPS();
    });
    
    if ($('mass-select')) $('mass-select').addEventListener('change', (e) => {
        // Mise à jour de la masse globale pour les calculs de force
        DEVICE_MASS_KG = parseFloat(e.target.value);
    });

    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', () => {
        emergencyStopActive = !emergencyStopActive;
        const btn = $('emergency-stop-btn');
        btn.textContent = emergencyStopActive ? '🛑 Arrêt d\'urgence: ACTIF 🔴' : '🛑 Arrêt d\'urgence: INACTIF 🟢';
        if (emergencyStopActive) stopGPS(false);
    });

    if ($('reset-dist-btn')) $('reset-dist-btn').addEventListener('click', () => { distM = 0; maxSpd = 0; });
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', () => { maxSpd = 0; });
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { if (confirm("Tout réinitialiser?")) stopGPS(true); });
    
    if ($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', () => {
        netherMode = !netherMode;
        $('mode-nether').textContent = netherMode ? 'ACTIVÉ (1:8) 🔥' : 'DÉSACTIVÉ (1:1)';
    });
}

document.addEventListener('DOMContentLoaded', initGeolocation);
