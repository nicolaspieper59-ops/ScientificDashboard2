// =================================================================
// FICHIER JS PARTIE 1/3 : gnss-dashboard-part1.js
// Contient les constantes, variables d'état, et la synchronisation NTP.
// =================================================================

// --- CLÉS D'API & PROXY VERCEL ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app";
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc";

// --- CONSTANTES PHYSIQUES ET MATHÉMATIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458;      // Vitesse de la lumière (m/s)
const R_E_BASE = 6371000;   // Rayon terrestre moyen (m)
const KMH_MS = 3.6;         // Conversion m/s vers km/h
const C_S = 343;            // Vitesse du son (m/s)
const OMEGA_EARTH = 7.2921159e-5; // Vitesse de rotation de la Terre (rad/s)
const R_AIR = 287.058;      // Constante spécifique de l'air sec (J/kg·K)

// --- CONSTANTES DE TEMPS & CALENDRIER ---
const MC_DAY_MS = 72 * 60 * 1000; // Durée d'un jour Minecraft en ms
const J1970 = 2440588, J2000 = 2451545;
const dayMs = 1000 * 60 * 60 * 24;

// --- CONSTANTES DE CONFIGURATION SYSTÈME ---
const MIN_DT = 0.01; // Temps minimum (en sec) pour une mise à jour
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};
const DOM_SLOW_UPDATE_MS = 1000; // Rafraîchissement des données non-critiques (1 sec)
const MAP_UPDATE_INTERVAL = 3000; // Rafraîchir la vue carte toutes les 3 sec si en mouvement

// --- PARAMÈTRES DU FILTRE DE KALMAN (VITESSE) ---
const Q_NOISE = 0.05;       // Bruit de processus (incertitude du modèle de mouvement)
const R_MIN = 0.01;         // Bruit de mesure minimum (GPS parfait)
const R_MAX = 50.0;         // Bruit de mesure maximum (GPS très bruité)
const MAX_ACC = 200;        // Précision max (m) avant de passer en "Estimation Seule"
const MIN_SPD = 0.05;       // Vitesse minimale pour être considéré "en mouvement"
const ALT_TH = -50;         // Seuil d'altitude pour détection "Sous-sol"
const MAX_PLAUSIBLE_ACCEL = 20.0; // Accélération max (m/s²) pour filtrage des "spikes"

// --- PARAMÈTRES EKF (ALTITUDE) ---
const Q_ALT_NOISE = 0.1;
const R_ALT_MIN = 0.1;

// --- FACTEURS ENVIRONNEMENTAUX (POUR R) ---
const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0, DISPLAY: 'Normal' },
    'FOREST': { R_MULT: 2.5, DISPLAY: 'Forêt' },
    'CONCRETE': { R_MULT: 7.0, DISPLAY: 'Grotte/Tunnel' },
    'METAL': { R_MULT: 5.0, DISPLAY: 'Métal/Bâtiment' },
};

// --- DONNÉES CÉLESTES/GRAVITÉ ---
const CELESTIAL_DATA = {
    'EARTH': { G: 9.80665, R: R_E_BASE, name: 'Terre' },
    'MOON': { G: 1.62, R: 1737400, name: 'Lune' },
    'MARS': { G: 3.71, R: 3389500, name: 'Mars' },
    'ROTATING': { G: 0.0, R: R_E_BASE, name: 'Station Spatiale' }
};
let G_ACC = CELESTIAL_DATA['EARTH'].G;
let R_ALT_CENTER_REF = CELESTIAL_DATA['EARTH'].R;

// --- VARIABLES D'ÉTAT (Globales) ---
let wID = null, domID = null, lPos = null, lat = null, lon = null, sTime = null;
let distM = 0, distMStartOffset = 0, maxSpd = 0;
let kSpd = 0, kUncert = 1000; 
let timeMoving = 0; 
let lServH = null, lLocH = null; 
let lastFSpeed = 0; 
let kAlt = null;      
let kAltUncert = 10;  

let currentGPSMode = 'HIGH_FREQ'; 
let emergencyStopActive = false;
let netherMode = false; // Conservé pour le bouton, mais la logique utilise R_FACTOR_RATIO
let selectedEnvironment = 'NORMAL'; 
let currentMass = 70.0; 
let R_FACTOR_RATIO = 1.0;   // Facteur de Rapport de Mouvement (MRF)
let currentCelestialBody = 'EARTH';
let rotationRadius = 100;
let angularVelocity = 0.0; 
let gpsAccuracyOverride = 0.0; 

// Données externes
let lastP_hPa = null, lastT_K = null, lastH_perc = null; 

// Objets Map
let map, marker, circle;
let lastUpdateMap = 0;

// --- FONCTIONS UTILITAIRES ---
const $ = id => document.getElementById(id);

/** Calcule la distance de Haversine en mètres */
const dist = (lat1, lon1, lat2, lon2) => {
    const R = R_ALT_CENTER_REF; 
    const dLat = (lat2 - lat1) * D2R, dLon = (lon2 - lon1) * D2R;
    lat1 *= D2R; lat2 *= D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
};

/** Synchronise l'horloge interne avec un serveur de temps (UTC/Atomique) */
async function syncH() { 
    if ($('local-time')) $('local-time').textContent = 'Synchronisation...';
    const localStartPerformance = performance.now(); 

    try {
        const response = await fetch(SERVER_TIME_ENDPOINT, { cache: "no-store", mode: "cors" });
        if (!response.ok) throw new Error(`Server time sync failed: ${response.statusText}`);
        
        const localEndPerformance = performance.now(); 
        const serverData = await response.json(); 
        
        const utcTimeISO = serverData.utc_datetime; 
        const serverTimestamp = Date.parse(utcTimeISO); 
        
        const RTT = localEndPerformance - localStartPerformance;
        const latencyOffset = RTT / 2;

        lServH = serverTimestamp + latencyOffset; 
        lLocH = performance.now(); 
        console.log(`Synchronisation UTC Atomique réussie. Latence corrigée: ${latencyOffset.toFixed(1)} ms.`);

    } catch (error) {
        console.warn("Échec de la synchronisation. Utilisation de l'horloge locale.", error);
        lServH = Date.now(); 
        lLocH = performance.now();
        if ($('local-time')) $('local-time').textContent = 'N/A (SYNCHRO ÉCHOUÉE)';
    }
}

/** Retourne l'heure synchronisée (précision RTT compensée en UTC). */
function getCDate() { 
    if (lServH === null || lLocH === null) { return null; }
    // Utilisation de performance.now() pour un delta plus précis que Date.now()
    const offsetSinceSync = performance.now() - lLocH;
    return new Date(lServH + offsetSinceSync); 
    }
// =================================================================
// FICHIER JS PARTIE 2/3 : gnss-dashboard-part2.js
// Contient les filtres de Kalman, la gestion Météo/Map, et les calculs Astro.
// NÉCESSITE gnss-dashboard-part1.js
// =================================================================

// ===========================================
// FILTRE DE KALMAN & FACTEUR R
// ===========================================

/**
 * Filtre de Kalman 1D pour la vitesse (Fusion IMU/GNSS).
 * @param {number} nSpd - Nouvelle mesure de vitesse (du GPS).
 * @param {number} dt - Delta-temps depuis la dernière mesure.
 * @param {number} R_dyn - Bruit de la mesure (calculé depuis la précision GPS).
 * @param {number} accel_input - Accélération mesurée (par un IMU, si disponible).
 */
function kFilter(nSpd, dt, R_dyn, accel_input = 0) {
    if (dt === 0 || dt > 5) return kSpd; 
    const R = R_dyn ?? R_MAX, Q = Q_NOISE * dt * dt; // Bruit de processus (incertitude du modèle)

    // Étape de PRÉDICTION (Modèle + Accéléromètre IMU)
    let pSpd = kSpd + (accel_input * dt); 
    let pUnc = kUncert + Q; 

    // Calcul du Gain de Kalman (K)
    let K = pUnc / (pUnc + R); 
    
    // Étape de CORRECTION (Fusion : combine prédiction et mesure GPS)
    kSpd = pSpd + K * (nSpd - pSpd); 
    kUncert = (1 - K) * pUnc; 
    
    return kSpd;
}

/** Applique le filtre de Kalman à l'Altitude. */
function kFilterAltitude(nAlt, acc, dt) {
    if (nAlt === null) return kAlt;
    const R_alt = Math.max(R_ALT_MIN, acc * acc); // Variance de la mesure
    const Q_alt = Q_ALT_NOISE * dt; 
    
    let pAlt = kAlt === null ? nAlt : kAlt; 
    let pAltUncert = kAlt === null ? 1000 : kAltUncert + Q_alt;
    
    let K_alt = pAltUncert / (pAltUncert + R_alt);
    kAlt = pAlt + K_alt * (nAlt - pAlt);
    kAltUncert = (1 - K_alt) * pAltUncert;
    
    return kAlt;
}

/** Calcule le Facteur R (Confiance GPS) du filtre de Kalman. */
function getKalmanR(acc, alt, P_hPa) {
    // CORRECTION : Mode Estimation Seule (Dead Reckoning)
    if (acc > MAX_ACC) {
        return 1e9; // R = infini (1 milliard). K (Gain) = 0. Force le filtre à ignorer le GPS.
    }
    
    let R = acc * acc; // Le bruit de mesure (variance) est le carré de l'écart-type (accuracy)
    
    // Application des facteurs environnementaux (R_MULT)
    const envFactor = ENVIRONMENT_FACTORS[selectedEnvironment]?.R_MULT || 1.0;
    R *= envFactor;
    
    // Facteur de Pression (Optionnel)
    if (P_hPa !== null) {
        const pressureFactor = 1.0 + (1013.25 - P_hPa) / 1013.25 * 0.1;
        R *= Math.max(1.0, pressureFactor); // Le mauvais temps (basse pression) dégrade le signal
    }
    
    // Pénalité sous-sol
    if (alt !== null && alt < ALT_TH) { 
        R *= 2.0; 
    } 

    return Math.max(R_MIN, Math.min(R_MAX, R)); // Plafonnement
}

// ===========================================
// FONCTIONS GÉO, MÉTÉO & CARTE
// ===========================================

function initMap() {
    try {
        if ($('map')) { // ID 'map' (corrigé de 'map-container')
            map = L.map('map').setView([0, 0], 2);
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                attribution: '© OpenStreetMap contributors'
            }).addTo(map);
            marker = L.marker([0, 0]).addTo(map);
            circle = L.circle([0, 0], { color: 'red', fillColor: '#f03', fillOpacity: 0.5, radius: 10 }).addTo(map);
        }
    } catch (e) {
        console.error("Erreur d'initialisation de Leaflet (Carte):", e);
        if ($('map')) $('map').innerHTML = "Erreur d'initialisation de la carte.";
    }
}

function updateMap(lat, lon, acc) {
    if (map && marker) {
        marker.setLatLng([lat, lon]);
        circle.setLatLng([lat, lon]).setRadius(acc * R_FACTOR_RATIO); 
        const now = Date.now();
        if (now - lastMapUpdate > MAP_UPDATE_INTERVAL && kSpd > MIN_SPD) {
            map.setView([lat, lon], map.getZoom() > 10 ? map.getZoom() : 16); 
            lastMapUpdate = now;
        } else if (map.getZoom() < 10 && (Date.now() - lastMapUpdate > 5000)) {
            map.setView([lat, lon], 12);
            lastMapUpdate = now;
        }
    }
}

async function fetchWeather(latA, lonA) {
    if (!latA || !lonA) return; 
    
    const apiUrl = `${PROXY_WEATHER_ENDPOINT}?lat=${latA}&lon=${lonA}`;
    
    try {
        const response = await fetch(apiUrl);
        if (!response.ok) throw new Error(`Erreur HTTP: ${response.status}`);
        const data = await response.json();
        
        if (data.main) {
            const tempC = data.main.temp;
            lastP_hPa = data.main.pressure; 
            lastT_K = tempC + 273.15; // Convertir Celsius en Kelvin
            lastH_perc = data.main.humidity / 100.0;
            
            if ($('temp-air')) $('temp-air').textContent = `${tempC.toFixed(1)} °C`;
            if ($('pressure')) $('pressure').textContent = `${lastP_hPa.toFixed(0)} hPa`;
            if ($('humidity')) $('humidity').textContent = `${data.main.humidity} %`;
            
            // Calcul de la densité de l'air
            const pressure_pa = lastP_hPa * 100;
            const air_density = pressure_pa / (R_AIR * lastT_K);
            if ($('air-density')) $('air-density').textContent = `${air_density.toFixed(3)} kg/m³`;
            
            // Calcul du point de rosée
            const a = 17.27, b = 237.7;
            const f = (a * tempC) / (b + tempC) + Math.log(lastH_perc);
            const dew_point = (b * f) / (a - f);
            if ($('dew-point')) $('dew-point').textContent = `${dew_point.toFixed(1)} °C`;
            
            // Remplissage des autres champs
            if ($('visibility')) $('visibility').textContent = data.visibility ? `${(data.visibility / 1000).toFixed(1)} km` : 'N/A';
            if ($('wind-speed-ms')) $('wind-speed-ms').textContent = data.wind ? `${data.wind.speed.toFixed(1)} m/s` : 'N/A';
            if ($('wind-direction')) $('wind-direction').textContent = data.wind ? `${data.wind.deg.toFixed(0)} °` : 'N/A';
            if ($('temp-feels-like')) $('temp-feels-like').textContent = data.main.feels_like ? `${data.main.feels_like.toFixed(1)} °C` : 'N/A';
        } else {
             throw new Error(data.message || 'Données météo incomplètes');
        }
    } catch (err) {
        console.warn("Erreur de récupération météo:", err.message);
        if ($('temp-air')) $('temp-air').textContent = `N/A`;
        if ($('pressure')) $('pressure').textContent = `N/A`;
        if ($('humidity')) $('humidity').textContent = `N/A`;
        if ($('air-density')) $('air-density').textContent = `N/A`;
        if ($('dew-point')) $('dew-point').textContent = `N/A`;
    }
}


// ===========================================
// FONCTIONS ASTRO & TEMPS
// (Inclus SunCalc.js dans le HTML)
// ===========================================

/** Convertit la date en jours depuis J2000. */
function toDays(date) { return (date.valueOf() / dayMs - 0.5 + J1970) - J2000; }
/** Calcule l'anomalie solaire moyenne. */
function solarMeanAnomaly(d) { return D2R * (356.0470 + 0.9856002585 * d); }
/** Calcule la longitude écliptique. */
function eclipticLongitude(M) {
    var C = D2R * (1.9148 * Math.sin(M) + 0.0200 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M)), 
        P = D2R * 102.9377;                                                                
    return M + C + P + Math.PI;
}
/** Calcule le Temps Solaire Vrai (TST). */
function getSolarTime(date, lon) {
    if (date === null || lon === null || isNaN(lon)) return { TST: 'N/A', MST: 'N/A', EOT: 'N/D', ECL_LONG: 'N/D' };
    
    const d = toDays(date);
    const M = solarMeanAnomaly(d); 
    const L = eclipticLongitude(M); 
    
    // Calcul de l'équation du temps (EOT)
    const J_star = toDays(date) - lon / 360;
    const J_transit = J_star + (0.0053 * Math.sin(M) - 0.0069 * Math.sin(2 * L));
    const eot_min = (J_star - J_transit) * 1440; // 1440 minutes par jour

    // Calcul du TST
    const msSinceMidnightUTC = (date.getUTCHours() * 3600 + date.getUTCMinutes() * 60 + date.getUTCSeconds()) * 1000 + date.getUTCMilliseconds();
    const mst_offset_ms = lon * dayMs / 360; 
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
        TST: toTimeString(tst_ms), 
        MST: toTimeString(mst_ms), 
        EOT: eot_min.toFixed(2),
        ECL_LONG: (L * R2D).toFixed(2),
        NoonSolar: toTimeString(J_transit * dayMs) // Corrigé : Utilise J_transit
    };
        }
// =================================================================
// FICHIER JS PARTIE 3/3 : gnss-dashboard-part3.js
// Contient la boucle principale (updateDisp) et l'initialisation DOM.
// NÉCESSITE gnss-dashboard-part1.js et gnss-dashboard-part2.js
// =================================================================

/** Calcule le temps Minecraft. */
function getMinecraftTime(date) {
    if (date === null) return '00:00:00';
    const msSinceMidnightUTC = date.getUTCHours() * 3600000 + date.getUTCMilliseconds() + date.getUTCMinutes() * 60000 + date.getUTCSeconds() * 1000;
    const timeRatio = (msSinceMidnightUTC % dayMs) / dayMs;
    const mcTimeMs = (timeRatio * MC_DAY_MS + MC_DAY_MS) % MC_DAY_MS;
    const toTimeString = (ms) => {
        let h = Math.floor(ms / 3600000);
        let m = Math.floor((ms % 3600000) / 60000);
        return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}`;
    };
    return toTimeString(mcTimeMs);
}

// Fonction pour déterminer le nom de la phase lunaire (0.0 à 1.0)
function getMoonPhaseName(phase) {
    if (phase < 0.03 || phase > 0.97) return "Nouvelle Lune 🌑";
    if (phase < 0.23) return "Premier Croissant 🌒";
    if (phase < 0.27) return "Premier Quartier 🌓";
    if (phase < 0.48) return "Gibbeuse Croissante 🌔";
    if (phase < 0.52) return "Pleine Lune 🌕";
    if (phase < 0.73) return "Gibbeuse Décroissante 🌖";
    if (phase < 0.77) return "Dernier Quartier 🌗";
    return "Dernier Croissant 🌘"; // (phase > 0.77)
}

// Mise à jour de l'horloge visuelle et des couleurs du corps (Day/Night)
function updateClockVisualization(now, sunPos, moonPos, sunTimes) {
    const sunEl = $('sun-element');
    const moonEl = $('moon-element');
    const clockEl = $('minecraft-clock'); 

    if (!sunEl || !moonEl || !clockEl) return;

    const sunIcon = sunEl.querySelector('.sun-icon');
    const moonIcon = moonEl.querySelector('.moon-icon');

    // 1. Position du Soleil
    if (sunPos) {
        const altDeg = sunPos.altitude * R2D;
        const aziDeg = (sunPos.azimuth * R2D + 180) % 360; 
        sunEl.style.transform = `rotate(${aziDeg}deg)`;
        const radialPercent = Math.min(50, Math.max(0, 50 * (90 - altDeg) / 90));
        const altitudeOffsetPercent = 50 - radialPercent; 
        sunIcon.style.transform = `translateY(calc(-50% + ${altitudeOffsetPercent}%) )`; 
        sunEl.style.display = altDeg > -0.83 ? 'flex' : 'none'; 
    } else {
        sunEl.style.display = 'none';
    }

    // 2. Position de la Lune
    if (moonPos) {
        const altDeg = moonPos.altitude * R2D;
        const aziDeg = (moonPos.azimuth * R2D + 180) % 360; 
        moonEl.style.transform = `rotate(${aziDeg}deg)`;
        const radialPercent = Math.min(50, Math.max(0, 50 * (90 - altDeg) / 90));
        const altitudeOffsetPercent = 50 - radialPercent; 
        moonIcon.style.transform = `translateY(calc(-50% + ${altitudeOffsetPercent}%) )`;
        moonEl.style.display = altDeg > 0 ? 'flex' : 'none';
    } else {
        moonEl.style.display = 'none';
    }
    
    // 3. Fond du corps et de l'horloge
    const body = document.body;
    body.classList.remove('sky-day', 'sky-sunset', 'sky-night', 'sky-night-light', 'dark-mode', 'light-mode');
    clockEl.classList.remove('sky-day', 'sky-sunset', 'sky-night', 'sky-night-light');

    if (sunTimes) {
        const nowMs = now.getTime();
        let bodyClass;
        if (nowMs >= sunTimes.sunriseEnd.getTime() && nowMs < sunTimes.sunsetStart.getTime()) {
            bodyClass = 'sky-day';
        } else if (nowMs >= sunTimes.dusk.getTime() || nowMs < sunTimes.dawn.getTime()) {
            bodyClass = 'sky-night';
        } else {
            bodyClass = 'sky-sunset';
        }
        
        body.classList.add(bodyClass);
        body.classList.add(bodyClass === 'sky-day' ? 'light-mode' : 'dark-mode');
        clockEl.classList.add(bodyClass); 

        $('clock-status').textContent = sunPos && sunPos.altitude > 0 ? 'Jour Solaire (☀️)' : 'Nuit/Crépuscule (🌙)';
    } else {
        $('clock-status').textContent = 'Position Solaire Indisponible';
    }
}


function updateAstro(latA, lonA) {
    const now = getCDate(); 
    if (now === null) {
        if ($('local-time') && !$('local-time').textContent.includes('Synchronisation')) {
             $('local-time').textContent = 'Synchronisation...';
        }
        return;
    }
    
    // Vérification que SunCalc est chargé et que les coordonnées sont valides
    if (typeof SunCalc === 'undefined' || !latA || !lonA) {
        $('clock-status').textContent = 'Astro (Attente GPS)...';
        return;
    }
    
    if ($('date-display-astro')) $('date-display-astro').textContent = now.toLocaleDateString('fr-FR');

    const sunPos = SunCalc.getPosition(now, latA, lonA);
    const moonIllum = SunCalc.getMoonIllumination(now);
    const moonPos = SunCalc.getMoonPosition(now, latA, lonA);
    const sunTimes = SunCalc.getTimes(now, latA, lonA);
    const moonTimes = SunCalc.getMoonTimes(now, latA, lonA, true);
    const solarTimes = getSolarTime(now, lonA);
    
    // Mise à jour du DOM
    if ($('tst')) $('tst').textContent = solarTimes.TST;
    if ($('lsm')) $('lsm').textContent = solarTimes.MST;
    if ($('eot')) $('eot').textContent = solarTimes.EOT + ' min'; 
    if ($('ecliptic-long')) $('ecliptic-long').textContent = solarTimes.ECL_LONG + ' °';

    if ($('sun-altitude')) $('sun-altitude').textContent = `${(sunPos.altitude * R2D).toFixed(2)} °`;
    if ($('sun-azimuth')) $('sun-azimuth').textContent = `${(sunPos.azimuth * R2D).toFixed(2)} ° (S-O)`;
    
    if ($('moon-altitude')) $('moon-altitude').textContent = `${(moonPos.altitude * R2D).toFixed(2)} °`;
    if ($('moon-azimuth')) $('moon-azimuth').textContent = `${(moonPos.azimuth * R2D).toFixed(2)} ° (S-O)`;
    
    if ($('moon-illum-fraction')) $('moon-illum-fraction').textContent = `${(moonIllum.fraction * 100).toFixed(1)} %`;
    if ($('moon-phase-name')) $('moon-phase-name').textContent = getMoonPhaseName(moonIllum.phase);
    
    if ($('noon-solar')) $('noon-solar').textContent = sunTimes.solarNoon ? sunTimes.solarNoon.toLocaleTimeString() : 'N/D';
    if ($('day-duration') && sunTimes.sunrise && sunTimes.sunset) {
        const durationMs = sunTimes.sunset.getTime() - sunTimes.sunrise.getTime();
        const hours = Math.floor(durationMs / 3600000);
        const minutes = Math.floor((durationMs % 3600000) / 60000);
        $('day-duration').textContent = `${hours}h ${minutes}m`;
    } else if ($('day-duration')) {
        $('day-duration').textContent = 'N/A (Polaire/Nuit)';
    }

    if ($('moon-times')) $('moon-times').textContent = moonTimes ? 
        `↑ ${moonTimes.rise ? moonTimes.rise.toLocaleTimeString() : 'N/A'} / ↓ ${moonTimes.set ? moonTimes.set.toLocaleTimeString() : 'N/A'}` : 'N/D';

    updateClockVisualization(now, sunPos, moonPos, sunTimes);
}


// ===========================================
// FONCTIONS DE CONTRÔLE GPS & MÉTÉO
// ===========================================
function initMap() {
    try {
        if ($('map')) { // ID 'map'
            map = L.map('map').setView([0, 0], 2);
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                attribution: '© OpenStreetMap contributors'
            }).addTo(map);
            marker = L.marker([0, 0]).addTo(map);
            circle = L.circle([0, 0], { color: 'red', fillColor: '#f03', fillOpacity: 0.5, radius: 10 }).addTo(map);
        }
    } catch (e) {
        console.error("Erreur d'initialisation de Leaflet (Carte):", e);
        if ($('map')) $('map').innerHTML = "Erreur d'initialisation de la carte. Vérifiez la connexion et l'ordre des scripts.";
    }
}

function updateMap(lat, lon, acc) {
    if (map && marker) {
        marker.setLatLng([lat, lon]);
        circle.setLatLng([lat, lon]).setRadius(acc * R_FACTOR_RATIO); 
        const now = Date.now();
        if (now - lastMapUpdate > 3000 && kSpd > MIN_SPD) {
            map.setView([lat, lon], map.getZoom() > 10 ? map.getZoom() : 16); 
            lastMapUpdate = now;
        } else if (map.getZoom() < 10) {
            map.setView([lat, lon], 12);
        }
    }
}

function setGPSMode(mode) {
    currentGPSMode = mode;
    if (wID !== null) {
        stopGPS(false); // Arrête sans changer le texte du bouton
        startGPS();     // Redémarre avec les nouvelles options
    }
    if ($('freq-select')) $('freq-select').value = mode; 
}

function startGPS() {
    if (wID !== null) return; // Déjà en cours
    
    const options = (currentGPSMode === 'HIGH_FREQ') ? GPS_OPTS.HIGH_FREQ : GPS_OPTS.LOW_FREQ;
    
    wID = navigator.geolocation.watchPosition(updateDisp, handleErr, options);
    
    if ($('toggle-gps-btn')) {
        $('toggle-gps-btn').textContent = '⏸️ PAUSE GPS';
        $('toggle-gps-btn').style.backgroundColor = '#ffc107'; // Jaune/Orange
    }
    
    if (sTime === null) sTime = Date.now();
    // Tente de récupérer la météo dès le début
    if (lat && lon) fetchWeather(lat, lon);
}

function stopGPS(resetButton = true) {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
    }
    if (resetButton) {
        if ($('toggle-gps-btn')) {
            $('toggle-gps-btn').textContent = '▶️ MARCHE GPS';
            $('toggle-gps-btn').style.backgroundColor = '#28a745'; // Vert
        }
    }
}

function emergencyStop() {
    emergencyStopActive = true;
    stopGPS(false);
    if ($('emergency-stop-btn')) {
        $('emergency-stop-btn').textContent = "🛑 Arrêt d'urgence: ACTIF 🔴";
        $('emergency-stop-btn').classList.add('active');
    }
    // Geler les affichages
    ['speed-stable', 'speed-3d-inst', 'distance-total-km', 'local-time'].forEach(id => {
        if ($(id)) $(id).textContent = 'ARRÊT D’URGENCE';
    });
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
    console.warn(`ERREUR GPS (${err.code}): ${err.message}`);
    if ($('gps-precision')) $('gps-precision').textContent = `Erreur: ${err.message}`;
    
    if (err.code === 1) { // Permission refusée
        stopGPS();
        alert("Accès à la géolocalisation refusé. Veuillez l'activer.");
    }
    // Pour les autres erreurs (ex: timeout), le mode estimation (Dead Reckoning) prendra le relais.
}

async function fetchWeather(latA, lonA) {
    // Si la position n'est pas encore définie, ne rien faire
    if (!latA || !lonA) {
        return; 
    }

    const apiUrl = `${PROXY_WEATHER_ENDPOINT}?lat=${latA}&lon=${lonA}`;
    
    try {
        const response = await fetch(apiUrl);
        if (!response.ok) throw new Error(`Erreur HTTP: ${response.status}`);
        
        const data = await response.json();
        
        if (data.main) {
            const tempC = data.main.temp; // Supposons que l'API renvoie en Celsius
            lastP_hPa = data.main.pressure; 
            lastT_K = tempC + 273.15; // Convertir en Kelvin
            lastH_perc = data.main.humidity / 100.0; // Convertir % en fraction
            
            if ($('temp-air')) $('temp-air').textContent = `${tempC.toFixed(1)} °C`;
            if ($('pressure')) $('pressure').textContent = `${lastP_hPa.toFixed(0)} hPa`;
            if ($('humidity')) $('humidity').textContent = `${data.main.humidity} %`;
            if ($('visibility')) $('visibility').textContent = data.visibility ? `${(data.visibility / 1000).toFixed(1)} km` : 'N/A';
            if ($('wind-speed-ms')) $('wind-speed-ms').textContent = data.wind ? `${data.wind.speed.toFixed(1)} m/s` : 'N/A';
            if ($('wind-direction')) $('wind-direction').textContent = data.wind ? `${data.wind.deg.toFixed(0)} °` : 'N/A';
            if ($('temp-feels-like')) $('temp-feels-like').textContent = data.main.feels_like ? `${data.main.feels_like.toFixed(1)} °C` : 'N/A';

            // Calcul de la densité de l'air (Physique)
            const pressure_pa = lastP_hPa * 100;
            const air_density = pressure_pa / (R_AIR * lastT_K); // R_AIR = 287.058
            if ($('air-density')) $('air-density').textContent = `${air_density.toFixed(3)} kg/m³`;
            
            // Calcul du point de rosée (Chimie/Physique)
            const a = 17.27, b = 237.7;
            const f = (a * tempC) / (b + tempC) + Math.log(lastH_perc);
            const dew_point = (b * f) / (a - f);
            if ($('dew-point')) $('dew-point').textContent = `${dew_point.toFixed(1)} °C`;
            
        } else {
             throw new Error(data.message || 'Données météo incomplètes');
        }
    } catch (err) {
        console.warn("Erreur de récupération météo:", err.message);
        if ($('temp-air')) $('temp-air').textContent = `N/A`;
        if ($('pressure')) $('pressure').textContent = `N/A`;
        if ($('humidity')) $('humidity').textContent = `N/A`;
    }
}


// ===========================================
// FONCTION PRINCIPALE DE MISE À JOUR GPS (updateDisp)
// ===========================================

function updateDisp(pos) {
    if (emergencyStopActive) return;

    // 1. Récupération des données brutes
    const cTimePos = pos.timestamp;
    let cLat = pos.coords.latitude;
    let cLon = pos.coords.longitude;
    let altRaw = pos.coords.altitude;
    let accRaw = pos.coords.accuracy;
    let spdRawGPS = pos.coords.speed; // Vitesse brute reportée par le GPS
    let headingRaw = pos.coords.heading;

    // 2. Vérification de l'heure et initialisation
    const now = getCDate(); 
    if (now === null) { return; } // Attend la synchro NTP
    if (sTime === null) { sTime = now.getTime(); }
    
    // 3. Gestion de l'override et de la perte de signal (Mode Estimation)
    if (gpsAccuracyOverride > 0.0) {
        accRaw = gpsAccuracyOverride;
    }

    let isSignalLost = (accRaw > MAX_ACC);

    if (isSignalLost) { 
        if ($('gps-precision')) $('gps-precision').textContent = `❌ ${accRaw.toFixed(0)} m (Signal Perdu/Estimation)`; 
        if (lPos === null) { lPos = pos; return; } // Ne peut pas estimer sans position précédente
        // En mode estimation, on utilise les dernières coordonnées valides
        cLat = lat;
        cLon = lon;
        altRaw = kAlt; // Utilise la dernière altitude filtrée
    } else {
        // Le signal est bon, on met à jour les coordonnées globales
        lat = cLat; 
        lon = cLon;
        if ($('gps-precision')) $('gps-precision').textContent = `${accRaw.toFixed(2)} m`; 
    }
    
    // 4. Calcul du Delta-Temps (dt)
    let dt = 0;
    if (lPos) {
        dt = (cTimePos - lPos.timestamp) / 1000;
    } else {
        lPos = pos; // Premier point de données
        lPos.speedMS_3D = 0;
        lPos.kAlt_old = altRaw;
        kAlt = altRaw;
        updateMap(cLat, cLon, accRaw);
        return; 
    }
    
    if (dt < MIN_DT || dt > 10) { // Ignorer les sauts temporels
        lPos = pos; 
        return; 
    }

    // 5. FILTRAGE DE L'ALTITUDE (via Kalman)
    const kAlt_new = kFilterAltitude(altRaw, pos.coords.altitudeAccuracy || R_ALT_MIN, dt);
    
    // 6. CALCUL DE LA VITESSE 3D BRUTE (Instantanée)
    const dist2D = dist(lPos.coords.latitude, lPos.coords.longitude, cLat, cLon);
    const dist3D = Math.sqrt(dist2D ** 2 + (kAlt_new - (lPos.kAlt_old || kAlt_new)) ** 2);
    let spd3D_raw = dist3D / dt; 
    const spdV = (kAlt_new - (lPos.kAlt_old || kAlt_new)) / dt; 

    // 7. CORRECTION ANTI-SPIKE DE VITESSE (Garde-fou)
    if (lPos && lPos.speedMS_3D !== undefined) {
        const lastRawSpd = lPos.speedMS_3D;
        const accelSpike = Math.abs(spd3D_raw - lastRawSpd) / dt;
        
        if (accelSpike > MAX_PLAUSIBLE_ACCEL) {
            console.warn(`Spike détecté: ${accelSpike.toFixed(2)} m/s². Correction appliquée.`);
            const maxPlausibleChange = MAX_PLAUSIBLE_ACCEL * dt;
            spd3D_raw = (spd3D_raw > lastRawSpd) ? (lastRawSpd + maxPlausibleChange) : (lastRawSpd - maxPlausibleChange);
        }
    }

    // 8. FILTRE DE KALMAN FINAL (Fusion IMU/GNSS)
    const R_dyn = getKalmanR(accRaw, kAlt_new, lastP_hPa); 
    
    // PLACEHOLDER : L'accélération doit venir d'un capteur IMU.
    // Sans IMU, en mode "Estimation Seule", la vitesse prédite sera la vitesse précédente (kSpd),
    // ce qui signifie que la vitesse va lentement dériver vers 0 à cause du bruit Q_NOISE.
    // Si nous avions un IMU, nous utiliserions son accélération ici.
    let accel_sensor_input = 0; 
    
    const fSpd = kFilter(spd3D_raw, dt, R_dyn, accel_sensor_input); 
    const sSpdFE = fSpd < MIN_SPD ? 0 : fSpd; // Vitesse stable finale (EKF)
    
    // 9. Calculs d'accélération et distance (basés sur la vitesse stable)
    let accel_long = 0;
    if (dt > 0.05) { // Évite les pics d'accélération sur des dt trop courts
        accel_long = (sSpdFE - lastFSpeed) / dt;
    }
    lastFSpeed = sSpdFE;

    R_FACTOR_RATIO = calculateMRF(kAlt_new); // Mise à jour du MRF
    distM += sSpdFE * dt * R_FACTOR_RATIO; // Distance cumulée
    
    if (sSpdFE > MIN_SPD) { timeMoving += dt; }
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    
    // 10. Calculs de physique (Gravité, Coriolis, Énergie)
    const local_g = getGravityLocal(kAlt_new); 
    const kineticEnergy = 0.5 * currentMass * sSpdFE ** 2;
    const mechanicalPower = currentMass * sSpdFE * accel_long;
    const coriolis_force = 2 * currentMass * sSpdFE * OMEGA_EARTH * Math.sin(lat * D2R);

    // --- MISE À JOUR DU DOM (Affichage) ---
    
    // Section Contrôle
    if ($('time-elapsed')) $('time-elapsed').textContent = `${((Date.now() - sTime) / 1000).toFixed(2)} s`;
    if ($('time-moving')) $('time-moving').textContent = `${timeMoving.toFixed(2)} s`;
    if ($('mode-ratio')) $('mode-ratio').textContent = `${R_FACTOR_RATIO.toFixed(3)} (Ratio)`;
    if ($('gps-accuracy-forced')) $('gps-accuracy-forced').textContent = `${gpsAccuracyOverride.toFixed(6)} m`;
    if ($('env-factor')) $('env-factor').textContent = `${ENVIRONMENT_FACTORS[selectedEnvironment].DISPLAY} (x${ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT.toFixed(1)})`;

    // Section Vitesse & Distance
    if ($('speed-stable')) $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(5)} km/h`;
    if ($('speed-stable-kms')) $('speed-stable-kms').textContent = `${(sSpdFE / 1000).toExponential(3)} km/s`;
    if ($('speed-stable-ms')) $('speed-stable-ms').textContent = `${sSpdFE.toFixed(3)} m/s | ${(sSpdFE * 1e6).toFixed(0)} µm/s | ${(sSpdFE * 1e9).toFixed(0)} nm/s`;
    if ($('speed-max')) $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    if ($('speed-avg-moving')) $('speed-avg-moving').textContent = timeMoving > 1 ? `${(distM / timeMoving * KMH_MS).toFixed(5)} km/h` : '0.00000 km/h';
    if ($('speed-3d-inst')) $('speed-3d-inst').textContent = `${(spd3D_raw * KMH_MS).toFixed(5)} km/h`; 
    if ($('perc-speed-sound')) $('perc-speed-sound').textContent = `${(sSpdFE / C_S * 100).toFixed(2)} %`;
    if ($('perc-speed-c')) $('perc-speed-c').textContent = `${(sSpdFE / C_L * 100).toExponential(2)}%`;
    if ($('distance-total-km')) $('distance-total-km').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    if ($('distance-light-s')) $('distance-light-s').textContent = `${(distM / C_L).toExponential(2)} s`;
    if ($('distance-light-min')) $('distance-light-min').textContent = `${(distM / C_L / 60).toExponential(2)} min`;
    if ($('distance-light-h')) $('distance-light-h').textContent = `${(distM / C_L / 3600).toExponential(2)} h`;
    if ($('distance-light-day')) $('distance-light-day').textContent = `${(distM / C_L / 86400).toExponential(2)} jours`;
    if ($('distance-light-week')) $('distance-light-week').textContent = `${(distM / C_L / 604800).toExponential(2)} sem`;
    if ($('distance-light-month')) $('distance-light-month').textContent = `$
