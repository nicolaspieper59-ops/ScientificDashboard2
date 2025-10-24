// =================================================================
// DÉBUT DU FICHIER JAVASCRIPT COMPLET
// =================================================================

// =================================================================
// PARTIE 1/3 : Core Engine, Constantes, Utilitaires, et Init
// (Contient constantes, variables d'état, fonctions de base, init capteurs, map)
// =================================================================

// --- CLÉS D'API ---
const API_KEYS = {
    WEATHER_API: 'VOTRE_CLE_API_METEO_ICI', 
    MAP_API: 'VOTRE_CLE_API_CARTE_ICI' // Clé pour Google Maps/Mapbox (utilisez Leaflet/OSM si non nécessaire)
};

// --- CONSTANTES GLOBALES ET INITIALISATION --
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458, C_S = 343, R_E = 6371000, KMH_MS = 3.6;
const OBLIQ = 23.44 * D2R, ECC = 0.0167, JD_2K = 2451545.0; 
let D_LAT = 48.8566, D_LON = 2.3522; // Latitude et Longitude par défaut
const MIN_DT = 0.01; 

// CONFIGURATIONS GPS
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};

// PARAMÈTRES AVANCÉS DU FILTRE DE KALMAN (V3.2)
const Q_NOISE = 0.01;       
const R_MIN = 0.05, R_MAX = 50.0; 
const MAX_ACC = 50, MIN_SPD = 0.001, ALT_TH = -50;
const SPEED_THRESHOLD = 0.5; 

// Fréquences pour la boucle principale fastDOM
const DOM_HIGH_FREQ_MS = 17;   
const DOM_LOW_FREQ_MS = 250;   
const DOM_SLOW_UPDATE_MS = 1000; 

// CONSTANTES POUR LES MODÈLES PHYSIQUES (V3.2)
const AIR_DENSITY = 1.225; 
const G_ACCEL = 9.80665;   
const CDA_EST = 0.6;       
const TROPO_K2 = 382000; 
const SUN_NIGHT_TH = -12; 
const LUX_NIGHT_TH = 50; 
const NETHER_RATIO = 8; 

// FACTEURS POUR MODÉLISATION V3.2
const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0, DRAG_MULT: 1.0 }, 'METAL': { R_MULT: 2.5, DRAG_MULT: 1.0 },      
    'FOREST': { R_MULT: 1.5, DRAG_MULT: 1.2 }, 'CONCRETE': { R_MULT: 3.0, DRAG_MULT: 1.0 },   
};
const WEATHER_FACTORS = {
    'CLEAR': 1.0, 'RAIN': 1.2, 'SNOW': 1.5, 'STORM': 2.0,                                  
};

// --- VARIABLES D'ÉTAT ---
let wID = null, domID = null, lPos = null, lat = null, lon = null, sTime = null;
let distM = 0, distMStartOffset = 0, maxSpd = 0, tLat = null, tLon = null, lDomT = null;
let kSpd = 0, kUncert = 1000; 
let lServH = null, lLocH = null; 
let als = null; 
let lastLux = null; 
let manualMode = null; 
let netherMode = false; 
let lastReliableHeading = null; 
let lastP_hPa = 1013.25, lastT_K = 293.15, lastH_perc = 0.5; 
let lastAltitudeBaro = null; 
let currentGPSMode = 'LOW_FREQ'; 
let currentDOMFreq = DOM_LOW_FREQ_MS; 
let manualFreqMode = false;      
let forcedFreqState = 'HIGH_FREQ'; 
let emergencyStopActive = false;
let selectedEnvironment = 'NORMAL'; 
let selectedWeather = 'CLEAR'; 
let currentBatteryLevel = 100;
let externalBatteryLevel = 99.999; // Simulé pour l'exemple
let mapObject = null;
let mapMarker = null;

const $ = id => document.getElementById(id); // Raccourci DOM

// ===========================================
// FONCTIONS GÉO & UTILS
// ===========================================

/** Calcule la distance de Haversine entre deux points. */
const dist = (lat1, lon1, lat2, lon2) => {
    const R = R_E, dLat = (lat2 - lat1) * D2R, dLon = (lon2 - lon1) * D2R;
    lat1 *= D2R; lat2 *= D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
};

/** Calcule le cap (bearing) entre deux points. */
const bearing = (lat1, lon1, lat2, lon2) => {
    lat1 *= D2R; lon1 *= D2R; lat2 *= D2R; lon2 *= D2R;
    const y = Math.sin(lon2 - lon1) * Math.cos(lat2);
    const x = Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1) * Math.cos(lat2) * Math.cos(lon2 - lon1);
    let b = Math.atan2(y, x) * R2D;
    return (b + 360) % 360; 
};

/** Synchronise l'heure locale avec l'heure serveur. */
async function syncH() { 
    const tz = Intl.DateTimeFormat().resolvedOptions().timeZone;
    try {
        const res = await fetch(`https://worldtimeapi.org/api/timezone/${tz}`, { signal: AbortSignal.timeout(5000) });
        if (!res.ok) throw new Error(res.status);
        const data = await res.json();
        lServH = data.unixtime * 1000; 
        lLocH = Date.now();            
    } catch (e) {
        if (lServH === null) lLocH = Date.now();
    }
}

/** Obtient l'heure corrigée. */
function getCDate() { 
    const currentLocalTime = Date.now(); 
    if (lServH !== null && lLocH !== null) {
        const offsetSinceSync = currentLocalTime - lLocH;
        return new Date(lServH + offsetSinceSync); 
    } else {
        return new Date(currentLocalTime); 
    }
}

/** Filtre de Kalman 1D. */
function kFilter(nSpd, dt, R_dyn) {
    if (dt === 0 || dt > 5) return kSpd; 
    const R = R_dyn ?? R_MAX, Q = Q_NOISE * dt; 
    let pSpd = kSpd, pUnc = kUncert + Q; 
    let K = pUnc / (pUnc + R); 
    kSpd = pSpd + K * (nSpd - pSpd);
    kUncert = (1 - K) * pUnc;
    return kSpd;
}

// ===========================================
// CAPTEURS AVANCÉS & SYSTÈME (INIT)
// ===========================================

function initALS() {
    if (!('AmbientLightSensor' in window)) {
        if ($('illuminance-lux')) $('illuminance-lux').textContent = 'N/A (API Manquante)';
        return;
    }
    try {
        als = new AmbientLightSensor({ frequency: 5 }); 
        als.onreading = () => {
            lastLux = als.illuminance;
            const luxEl = $('illuminance-lux');
            if (luxEl) luxEl.textContent = `${lastLux.toFixed(2)} Lux`;
        };
        als.onerror = (event) => {
            lastLux = null;
            if ($('illuminance-lux')) $('illuminance-lux').textContent = 'Erreur/Non Autorisé';
            if (als) als.stop();
        };
        als.start();
    } catch (error) {
        if ($('illuminance-lux')) $('illuminance-lux').textContent = 'N/A (Navigateur)';
    }
}

function initAdvancedSensors() {
    if ('DeviceOrientationEvent' in window) {
        window.addEventListener('deviceorientation', (event) => {
            const beta = event.beta, gamma = event.gamma; 
            const bubbleEl = $('bubble-level');
            if (bubbleEl) bubbleEl.textContent = `${beta !== null ? beta.toFixed(1) : '--'}°/${gamma !== null ? gamma.toFixed(1) : '--'}°`;
        }, true);
    } 

    if ('DeviceMotionEvent' in window) {
        window.addEventListener('devicemotion', (event) => {
            const accel = event.acceleration;
            if (accel && accel.x !== null) {
                const a_long = accel.y ?? 0; 
                const g_force = a_long / G_ACCEL;
                const gForceEl = $('g-force'), accelLongEl = $('accel-long');
                if (gForceEl) gForceEl.textContent = `${g_force.toFixed(2)} G`;
                if (accelLongEl) accelLongEl.textContent = `${a_long.toFixed(3)} m/s²`;
            }
        }, true);
    }
}

function updateBatteryStatus(battery) {
    const level = battery.level;
    const levelPerc = (level * 100).toFixed(3); // x.xxx%
    const charging = battery.charging;
    currentBatteryLevel = level;
    
    const batInd = $('battery-indicator');
    if (batInd) batInd.textContent = `${Math.floor(level * 100)}% ${charging ? '🔌' : ''} (${charging ? 'Charge' : 'Décharge'})`;
    if ($('battery-level-perc')) $('battery-level-perc').textContent = `${levelPerc}%`;

    const batExt = $('external-battery-level');
    // Simulation simple de la batterie externe
    const extLevelPerc = externalBatteryLevel.toFixed(3);
    if (batExt) batExt.textContent = `${extLevelPerc}%`;
}

async function initBattery() {
    if (!('getBattery' in navigator)) {
        if ($('battery-indicator')) $('battery-indicator').textContent = 'N/A (API Manquante)';
        return;
    }
    try {
        const battery = await navigator.getBattery();
        updateBatteryStatus(battery);
        
        battery.addEventListener('levelchange', () => updateBatteryStatus(battery));
        battery.addEventListener('chargingchange', () => updateBatteryStatus(battery));
    } catch (e) {
        if ($('battery-indicator')) $('battery-indicator').textContent = 'N/A (Accès Refusé)';
    }
}

function initMap() {
    const defaultCoords = [D_LAT, D_LON];
    const mapEl = $('map');
    
    if (mapEl && window.L) {
        mapObject = L.map('map').setView(defaultCoords, 13);
        
        // Utilisation d'OpenStreetMap pour un meilleur fonctionnement hors ligne/sans clé API
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: '© OpenStreetMap contributors',
            maxZoom: 19
        }).addTo(mapObject);

        mapMarker = L.marker(defaultCoords).addTo(mapObject)
            .bindPopup("Position Actuelle").openPopup();
        
        mapObject.on('locationfound', (e) => {
            // Centre la carte sur la position GPS si elle démarre
            mapObject.setView(e.latlng, 15);
        });
        
        mapObject.on('locationerror', (e) => {
            console.warn("Erreur de localisation pour la carte:", e.message);
        });
        
        // Tentative de localisation (pour centrer la carte au début)
        mapObject.locate({setView: false, maxZoom: 16});
    } else if (mapEl) {
        mapEl.textContent = "Erreur: Leaflet (Carte) non chargé ou L n'est pas défini.";
    }
}

// =================================================================
// FIN DE LA PARTIE 1/3 (Coupure Artificielle) - Env. 175 Lignes
// =================================================================
// =================================================================
// PARTIE 2/3 : Astro, Météo & Traitement de Données GPS
// (Dépend des variables et fonctions de la Partie 1)
// =================================================================

// ===========================================
// CORRECTIONS MÉTÉO/TROPOSPHÈRE
// ===========================================

function getTroposphericDelay(P_hPa, T_K, H_frac, alt_m, lat_deg) {
    const ZHD = 0.0022768 * P_hPa / (1 - 0.00266 * Math.cos(2 * lat_deg * D2R) - 0.00028 * alt_m / 1000);
    const T_C = T_K - 273.15;
    const SVP = 6.11 * Math.exp(19.7 * T_C / (T_C + 273.15)); 
    const e = H_frac * SVP; 
    const ZWD = 0.002277 * (TROPO_K2 / T_K) * (e / 100); 
    return ZHD + ZWD;
}

function getKalmanR(acc, alt, ztd_m) {
    let R = acc ** 2; 
    R = Math.max(R_MIN, Math.min(R_MAX, R));
    const envFactor = ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT;
    const weatherFactor = WEATHER_FACTORS[selectedWeather];
    R *= envFactor; R *= weatherFactor;
    if (ztd_m > 2.5) { R *= 1.1; } 
    R = Math.max(R_MIN, Math.min(R_MAX, R));
    if (emergencyStopActive) { R = Math.min(R, 10.0); } 
    return R;
}

function getCurrentDragMultiplier() {
    return ENVIRONMENT_FACTORS[selectedEnvironment].DRAG_MULT;
}

async function fetchWeather(latA, lonA) {
    const weatherCondEl = $('weather-condition-api');
    
    if (API_KEYS.WEATHER_API === 'VOTRE_CLE_API_METEO_ICI') {
        // [SIMULATION] (Fonctionne hors ligne)
        lastP_hPa = 1013.25 + Math.sin(Date.now() / 100000) * 1.5;
        lastT_K = 293.15 + Math.sin(Date.now() / 500000) * 5; 
        lastH_perc = 0.5 + Math.cos(Date.now() / 300000) * 0.1;
        lastAltitudeBaro = (lPos && lPos.coords.altitude !== null) ? lPos.coords.altitude + Math.sin(Date.now()/50000)*10 : 100;
        if (weatherCondEl) weatherCondEl.textContent = 'Simulées (Hors Ligne)';
        return; 
    }

    // [APPEL API RÉEL - OpenWeatherMap]
    const API_URL = `https://api.openweathermap.org/data/2.5/weather?lat=${latA}&lon=${lonA}&units=metric&appid=${API_KEYS.WEATHER_API}`;
    
    try {
        const res = await fetch(API_URL, { signal: AbortSignal.timeout(5000) });
        if (!res.ok) throw new Error(`HTTP error! status: ${res.status}`);
        const data = await res.json();
        
        lastP_hPa = data.main.pressure;            
        lastT_K = data.main.temp + 273.15;         
        lastH_perc = data.main.humidity / 100;     
        if (lastAltitudeBaro === null) {
            lastAltitudeBaro = (lPos && lPos.coords.altitude !== null) ? lPos.coords.altitude : 0; 
        }
        const condition = data.weather[0].description;
        if (weatherCondEl) weatherCondEl.textContent = condition;
        
    } catch (e) {
        if (weatherCondEl) weatherCondEl.textContent = 'Erreur API/Hors Ligne';
    }
}

// ===========================================
// CALCULS ASTRO
// ===========================================

function calcSolar() { 
    const now = getCDate(), J2K_MS = 946728000000;
    const D = (now.getTime() - J2K_MS) / 86400000;
    const M = (357.529 + 0.98560028 * D) * D2R; 
    const L = (280.466 + 0.98564736 * D) * D2R; 
    const Ce = 2 * ECC * Math.sin(M) + 1.25 * ECC ** 2 * Math.sin(2 * M);
    const lambda = L + Ce; 
    const delta = Math.asin(Math.sin(OBLIQ) * Math.sin(lambda)); 
    const alpha = Math.atan2(Math.cos(OBLIQ) * Math.sin(lambda), Math.cos(lambda)); 
    const JD = now.getTime() / 86400000 + 2440587.5;
    const T = (JD - JD_2K) / 36525.0; 
    let GST = 280.4606 + 360.9856473 * (JD - JD_2K) + 0.000388 * T ** 2;
    GST = (GST % 360 + 360) % 360; 
    const LST = GST + (lon ?? D_LON); 
    const HA_rad = ((LST % 360) * D2R) - alpha; 
    const lat_rad = (lat ?? D_LAT) * D2R;
    const h = Math.asin(Math.sin(lat_rad) * Math.sin(delta) + Math.cos(lat_rad) * Math.cos(delta) * Math.cos(HA_rad));
    
    let EoT_deg = (L - alpha) * R2D;
    while (EoT_deg > 180) EoT_deg -= 360;
    while (EoT_deg < -180) EoT_deg += 360;
    const EoT_m = EoT_deg * 4; 
    
    let sLon = (lambda * R2D) % 360;
    if (sLon < 0) sLon += 360;
    
    const noonLSM_sec = 12 * 3600;
    const EoT_sec = EoT_m * 60;
    const culmination_LSM_sec = (noonLSM_sec + EoT_sec) % 86400;
    
    const culminH_h = Math.floor(culmination_LSM_sec / 3600);
    const culminH_m = Math.floor((culmination_LSM_sec % 3600) / 60);
    const culminH_s = Math.floor(culmination_LSM_sec % 60);
    
    const culminationStr = `${String(culminH_h).padStart(2, '0')}:${String(culminH_m).padStart(2, '0')}:${String(culminH_s).padStart(2, '0')}`;

    return { eot: EoT_m, solarLongitude: sLon, elevation: h * R2D, culmination: culminationStr };
}

function calcLunarPhase() { 
    const now = getCDate();
    const JD = now.getTime() / 86400000 + 2440587.5; 
    const d = JD - JD_2K; 
    let D = 297.8501921 + 445.2671115 * d; 
    D = D % 360; 
    if (D < 0) D += 360;
    return D * D2R; 
}

function calcMiddayMetrics() {
    const now = getCDate();
    const year = now.getFullYear(), month = now.getMonth(), day = now.getDate();
    const utcMidday = new Date(Date.UTC(year, month, day, 12, 0, 0));
    
    const J2K_MS = 946728000000;
    const D = (utcMidday.getTime() - J2K_MS) / 86400000;
    const M = (357.529 + 0.98560028 * D) * D2R; 
    const L = (280.466 + 0.98564736 * D) * D2R; 
    const Ce = 2 * ECC * Math.sin(M) + 1.25 * ECC ** 2 * Math.sin(2 * M);
    const lambda = L + Ce; 
    
    let sLon = (lambda * R2D) % 360;
    if (sLon < 0) sLon += 360;
    
    const alpha = Math.atan2(Math.cos(OBLIQ) * Math.sin(lambda), Math.cos(lambda));
    let EoT_deg = (L - alpha) * R2D;
    while (EoT_deg > 180) EoT_deg -= 360;
    while (EoT_deg < -180) EoT_deg += 360;
    const EoT_m = EoT_deg * 4; 
    
    return { solarLongitude: sLon, eot: EoT_m };
}

// ===========================================
// GESTIONNAIRE DE DONNÉES GPS PRINCIPAL
// ===========================================

function updateDisp(pos) {
    lat = pos.coords.latitude; lon = pos.coords.longitude;
    const alt = pos.coords.altitude, acc = pos.coords.accuracy;
    const spd = pos.coords.speed, cTime = pos.timestamp; 

    syncH(); 

    if (sTime === null) { sTime = getCDate().getTime(); distMStartOffset = distM; }

    if (acc > MAX_ACC) { 
        if ($('gps-accuracy')) $('gps-accuracy').textContent = `❌ ${acc.toFixed(0)} m (Trop Imprécis)`; 
        if (lPos === null) lPos = pos; return; 
    }
    
    let spdH = spd ?? 0, spdV = 0;
    const dt = lPos ? (cTime - lPos.timestamp) / 1000 : MIN_DT; 

    if (lPos && dt > 0.1) { 
        const dH = dist(lPos.coords.latitude, lPos.coords.longitude, lat, lon); 
        if (spd === null || spd === undefined) spdH = dH / dt; 
        if (alt !== null && lPos.coords.altitude !== null) spdV = (alt - lPos.coords.altitude) / dt; 
    }
    
    const spd3D = Math.sqrt(spdH ** 2 + spdV ** 2); // Vitesse 3D (m/s)

    // --- CORRECTION GNSS AVANCÉE ---
    const ztd_m = getTroposphericDelay(lastP_hPa, lastT_K, lastH_perc, alt ?? 0, lat); 
    const R_dyn = getKalmanR(acc, alt, ztd_m); 
    
    const fSpd = kFilter(spd3D, dt, R_dyn), sSpdFE = fSpd < MIN_SPD ? 0 : fSpd;
    
    let hdg = pos.coords.heading;
    let hdg_corr = hdg;
    if (sSpdFE > SPEED_THRESHOLD && hdg !== null) { lastReliableHeading = hdg; } else { hdg_corr = lastReliableHeading; }
    
    // --- MISE À JOUR DES ÉTATS ET MÉTRIQUES ---
    const lastSpd3D = lPos ? (lPos.speedMS_3D_LAST ?? 0) : 0;
    const accellLong = dt > 0 ? (spd3D - lastSpd3D) / dt : 0;
    
    const dragMult = getCurrentDragMultiplier(); 
    const dragForce = 0.5 * AIR_DENSITY * CDA_EST * dragMult * sSpdFE ** 2; 
    const dragPower = dragForce * sSpdFE; 
    
    lPos = pos; lPos.speedMS_3D = spd3D; lPos.timestamp = cTime; 
    lPos.speedMS_3D_LAST = spd3D; lPos.kalman_R_val = R_dyn; 
    lPos.kalman_kSpd_LAST = fSpd; 

    checkGPSFrequency(sSpdFE); 

    distM += sSpdFE * dt * (netherMode ? NETHER_RATIO : 1); 
    
    const elapS = (getCDate().getTime() - sTime) / 1000;
    const sessionDistM = distM - distMStartOffset;
    const spdAvg = elapS > 0 ? sessionDistM / elapS : 0; 
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    
    // --- MISE À JOUR CARTE LEAFLET ---
    if (mapMarker && mapObject) {
        const newLatLng = L.latLng(lat, lon);
        mapMarker.setLatLng(newLatLng);
        mapObject.setView(newLatLng);
    }

    // --- AFFICHAGE BASIQUE (Le reste est fait dans fastDOM) ---
    let pText = `${acc.toFixed(2)} m`;
    pText += R_dyn <= R_MIN * 1.5 ? ' (Optimal/Corrigé)' : ' (Ajusté)'; 

    if ($('gps-accuracy')) $('gps-accuracy').textContent = pText;
    if ($('speed-avg')) $('speed-avg').textContent = `${(spdAvg * KMH_MS).toFixed(5)} km/h`; 
    if ($('latitude')) $('latitude').textContent = `${lat.toFixed(6)}`;
    if ($('longitude')) $('longitude').textContent = `${lon.toFixed(6)}`;
    if ($('altitude')) $('altitude').textContent = `${alt !== null ? alt.toFixed(2) : '--'} m`;
    if ($('underground')) $('underground').textContent = alt !== null && alt < ALT_TH ? 'Oui' : 'Non'; 
    if ($('heading')) $('heading').textContent = hdg_corr !== null ? `${hdg_corr.toFixed(1)} °` : '--';
    if ($('vertical-speed')) $('vertical-speed').textContent = `${spdV.toFixed(2)} m/s`;
    if ($('drag-force')) $('drag-force').textContent = `${dragForce.toFixed(1)} N`;
    if ($('drag-power-kw')) $('drag-power-kw').textContent = `${(dragPower / 1000).toFixed(2)} kW`;
    if ($('g-force')) $('g-force').textContent = `${(accellLong / G_ACCEL).toFixed(2)} G`;
    if ($('accel-long')) $('accel-long').textContent = `${accellLong.toFixed(3)} m/s²`;
    if ($('ztd')) $('ztd').textContent = `${ztd_m.toFixed(3)} m`; 

    if (Date.now() - (updateDisp.lastWeatherFetch ?? 0) > 10000) {
        fetchWeather(lat, lon); 
        updateDisp.lastWeatherFetch = Date.now();
    }
}

/** Gestionnaire des erreurs GPS. */
function handleErr(err) {
    console.error(`Erreur GNSS (${err.code}): ${err.message}`);
    const errEl = $('error-message');
    if(errEl) {
        errEl.style.display = 'block';
        errEl.textContent = `❌ Erreur GNSS (${err.code}): Signal perdu. Bascule vers la prédiction/dernière position.`;
    }
    stopGPS(false);
}

// =================================================================
// FIN DE LA PARTIE 2/3 (Coupure Artificielle) - Env. 220 Lignes
// =================================================================
// =================================================================
// PARTIE 3/3 : DOM, Boucles & Contrôle Système
// (Dépend des fonctions des Parties 1 et 2)
// =================================================================

// ===========================================
// LOGIQUE D'AFFICHAGE & DOM
// ===========================================


    
    // Fonctions de contrôle et utilitaires (Raccourcis
// =================================================================
// PARTIE 3/3 : DOM, Boucles & Contrôle Système
// (Dépend des fonctions des Parties 1 et 2)
// =================================================================

// ===========================================
// FONCTIONS DE PERSISTENCE D'ÉTAT (NOUVEAU)
// ===========================================

const STATE_KEY = 'qdgnss_state_v4';

function saveState() {
    const state = {
        distM: distM,
        maxSpd: maxSpd,
        tLat: tLat,
        tLon: tLon,
        D_LAT: lat ?? D_LAT, 
        D_LON: lon ?? D_LON,
        netherMode: netherMode,
        externalBatteryLevel: externalBatteryLevel,
        manualMode: manualMode
    };
    try {
        localStorage.setItem(STATE_KEY, JSON.stringify(state));
    } catch (e) {
        console.warn("Échec de la sauvegarde de l'état.", e);
    }
}

function loadState() {
    try {
        const savedState = localStorage.getItem(STATE_KEY);
        if (savedState) {
            const state = JSON.parse(savedState);
            distM = state.distM || 0;
            distMStartOffset = state.distM || 0;
            maxSpd = state.maxSpd || 0;
            tLat = state.tLat || null;
            tLon = state.tLon || null;
            netherMode = state.netherMode || false;
            externalBatteryLevel = state.externalBatteryLevel || 99.999;
            // Charge la dernière position GPS connue comme position par défaut
            D_LAT = state.D_LAT || D_LAT; 
            D_LON = state.D_LON || D_LON;
            manualMode = state.manualMode === true || state.manualMode === false ? state.manualMode : null;
            
            // Mise à jour initiale de l'affichage
            if ($('nether-indicator')) $('nether-indicator').textContent = netherMode ? "ACTIVÉ (1:8) 🔥" : "DÉSACTIVÉ (1:1)";
            if ($('nether-toggle-btn')) $('nether-toggle-btn').textContent = netherMode ? "🌍 Overworld" : "🔥 Nether";
            document.body.classList.toggle('night-mode', manualMode !== null ? manualMode : false);
            return true;
        }
    } catch (e) {
        console.error("Échec du chargement de l'état.", e);
    }
    return false;
}

// ===========================================
// LOGIQUE D'AFFICHAGE & DOM
// ===========================================

function calcLunarTime(lon) { 
    const now = getCDate(), JD = now.getTime() / 86400000 + 2440587.5; 
    const T = (JD - JD_2K) / 36525.0; 
    let GST = 280.4606 + 360.9856473 * (JD - JD_2K) + 0.000388 * T ** 2;
    GST = GST % 360; if (GST < 0) GST += 360;
    let Lm = 218.316 + 488204.661 * T; 
    Lm = Lm % 360; if (Lm < 0) Lm += 360;
    let HAm = GST + lon - Lm; 
    HAm = HAm % 360; if (HAm < 0) HAm += 360;
    
    const LMT_h = HAm / 15.0; 
    const LMT_sec = LMT_h * 3600;
    const h = Math.floor(LMT_sec / 3600) % 24;
    const m = Math.floor((LMT_sec % 3600) / 60);
    const s = Math.floor(LMT_sec % 60);
    const lunarTimeEl = $('lunar-time');
    if (lunarTimeEl) lunarTimeEl.textContent = `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
}

/** Met à jour tous les éléments d'affichage astronomiques (DOM). */
function updateAstro(latA, lonA) {
    const now = getCDate(), sData = calcSolar(); 
    const hDeg = sData.elevation;
    
    // Temps Solaire Vrai et Moyen
    let sTimeH = (now.getUTCHours() * 3600 + now.getUTCMinutes() * 60 + now.getUTCSeconds()) / 3600;
    let trueSolarTimeH = sTimeH + (lonA / 15) + (sData.eot / 60);
    trueSolarTimeH = (trueSolarTimeH % 24 + 24) % 24;
    const tsH = Math.floor(trueSolarTimeH), tsM = Math.floor((trueSolarTimeH * 60) % 60), tsS = Math.floor((trueSolarTimeH * 3600) % 60);
    if ($('solar-true')) $('solar-true').textContent = `${String(tsH).padStart(2, '0')}:${String(tsM).padStart(2, '0')}:${String(tsS).padStart(2, '0')}`;
    let meanSolarTimeH = sTimeH + (lonA / 15);
    meanSolarTimeH = (meanSolarTimeH % 24 + 24) % 24;
    const msH = Math.floor(meanSolarTimeH), msM = Math.floor((meanSolarTimeH * 60) % 60), msS = Math.floor((meanSolarTimeH * 3600) % 60);
    if ($('solar-mean')) $('solar-mean').textContent = `${String(msH).padStart(2, '0')}:${String(msM).padStart(2, '0')}:${String(msS).padStart(2, '0')}`;
    
    if ($('eot')) $('eot').textContent = `${sData.eot.toFixed(2)} min`;
    if ($('sun-elevation')) $('sun-elevation').textContent = `${hDeg.toFixed(2)} °`;
    if ($('solar-longitude-val')) $('solar-longitude-val').textContent = `${sData.solarLongitude.toFixed(2)} °`;
    if ($('solar-culmination')) $('solar-culmination').textContent = sData.culmination;

    const D_rad = calcLunarPhase(); 
    const phasePerc = (1 + Math.cos(D_rad)) / 2 * 100; 
    if ($('lunar-phase-perc')) $('lunar-phase-perc').textContent = `${phasePerc.toFixed(1)}%`;
    calcLunarTime(lonA); 

    const autoNight = (hDeg < SUN_NIGHT_TH) || (lastLux !== null && lastLux < LUX_NIGHT_TH); 
    const isNight = manualMode !== null ? manualMode : autoNight;
    
    document.body.classList.toggle('night-mode', isNight);
    
    let modeText = '';
    const modeInd = $('mode-indicator');
    if (manualMode !== null) modeText = manualMode ? 'Nuit 🌙 (Manuel)' : 'Jour ☀️ (Manuel)';
    else modeText = isNight ? 'Nuit 🌙 (Auto)' : 'Jour ☀️ (Auto)';
    if (modeInd) modeInd.textContent = modeText;
}

/** Met à jour l'affichage de la distance, de la vitesse max et de la cible. */
function updateDM(latA, lonA) {
    if ($('distance-km-m')) $('distance-km-m').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    if ($('speed-max')) $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    
    if (tLat !== null && tLon !== null) {
        const distTarget = dist(latA, lonA, tLat, tLon);
        const capDest = bearing(latA, lonA, tLat, tLon);
        if ($('cap-dest')) $('cap-dest').textContent = `${capDest.toFixed(1)} °`;
        if ($('dist-target')) $('dist-target').textContent = `${(distTarget / 1000).toFixed(3)} km`;
    }
}

/** Boucle principale pour les mises à jour rapides du DOM. */
function fastDOM() {
    const latA = lat ?? D_LAT, lonA = lon ?? D_LON;
    const pNow = performance.now();
    
    const updateFrequencyEl = $('update-frequency');
    if (lDomT && updateFrequencyEl) updateFrequencyEl.textContent = `${(1000 / (pNow - lDomT)).toFixed(1)} Hz (DOM)`;
    lDomT = pNow;

    if ($('local-time')) $('local-time').textContent = getCDate().toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit', second: '2-digit' });

    updateAstro(latA, lonA); 
    
    // Contrôle de la fréquence lente (1 Hz) et Sauvegarde de l'état
    if (fastDOM.lastSlowT && (pNow - fastDOM.lastSlowT) < DOM_SLOW_UPDATE_MS) return;
    
    fastDOM.lastSlowT = pNow;
    saveState(); // <<< SAUVEGARDE DE L'ÉTAT

    updateDM(latA, lonA); 
    
    const middayData = calcMiddayMetrics(); 
    if ($('solar-longitude-midday')) $('solar-longitude-midday').textContent = `${middayData.solarLongitude.toFixed(8)} °`;
    
    if (!lPos && sTime !== null) { return; } // Retourne si GPS en cours mais sans données récentes
    
    const spd3D = lPos?.speedMS_3D || 0, spd3DKMH = spd3D * KMH_MS; 
    const sSpd = kSpd < MIN_SPD ? 0 : kSpd, sSpdKMH = sSpd * KMH_MS; 
    
    const spd4D_perc = (spd3D / C_L * 100); 

    if ($('speed-3d-inst')) $('speed-3d-inst').textContent = `${spd3DKMH.toFixed(5)} km/h`; 
    if ($('speed-4d-perc')) $('speed-4d-perc').textContent = `${spd4D_perc.toPrecision(5)} %`; 
    if ($('perc-sound')) $('perc-sound').textContent = `${(spd3D / C_S * 100).toFixed(5)} %`; 
    if ($('speed-stable')) $('speed-stable').textContent = `${sSpdKMH.toFixed(5)} km/h`; 

    const kR = lPos?.kalman_R_val || R_MAX; 
    const uncertainty_ratio = Math.min(kUncert / 1.0, 1.0); 
    const coherence_perc = 100 * (1 - uncertainty_ratio);
    if ($('speed-error-perc')) $('speed-error-perc').textContent = `${coherence_perc.toFixed(1)}% (R:${kR.toFixed(3)})`;
    
    const P_hPa = lastP_hPa;
    const T_C = lastT_K - 273.15;
    
    if ($('pressure')) $('pressure').textContent = `${P_hPa.toFixed(2)} hPa`;
    if ($('air-temp')) $('air-temp').textContent = `${T_C.toFixed(1)} °C`;
    if ($('humidity')) $('humidity').textContent = `${(lastH_perc * 100).toFixed(1)} %`;
    if ($('alt-baro') && lastAltitudeBaro !== null) {
        $('alt-baro').textContent = `${lastAltitudeBaro.toFixed(2)} m (Baro)`;
    } else if ($('alt-baro')) {
        $('alt-baro').textContent = 'N/A (Baro)';
    }
}

// ===========================================
// FONCTIONS DE CONTRÔLE GPS & SYSTÈME
// ===========================================

function stopGPS(clearT = true) {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    wID = null; currentGPSMode = 'OFF';
    if ($('speed-source-indicator')) $('speed-source-indicator').textContent = 'Source: OFF';
    if ($('start-btn')) $('start-btn').disabled = false;
    if ($('stop-btn')) $('stop-btn').disabled = true;
    if ($('reset-max-btn')) $('reset-max-btn').disabled = false;
    if (clearT) { sTime = null; lPos = null; }
}

function setGPSMode(newMode) {
    if (emergencyStopActive) { newMode = 'LOW_FREQ'; }
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    currentGPSMode = newMode;
    const opts = GPS_OPTS[newMode]; 
    wID = navigator.geolocation.watchPosition(updateDisp, handleErr, opts); 
    
    const newDOMFreq = newMode === 'HIGH_FREQ' ? DOM_HIGH_FREQ_MS : (emergencyStopActive ? DOM_LOW_FREQ_MS * 4 : DOM_LOW_FREQ_MS);
    if (newDOMFreq !== currentDOMFreq) {
        currentDOMFreq = newDOMFreq;
        if (domID !== null) clearInterval(domID);
        domID = setInterval(fastDOM, currentDOMFreq); 
    }
    
    const speedSourceIndicator = $('speed-source-indicator');
    if (speedSourceIndicator) speedSourceIndicator.textContent = `Source: ${newMode} (${manualFreqMode ? 'Forcé' : 'Auto'})`;
    const freqManualBtn = $('freq-manual-btn');
    if (freqManualBtn && !manualFreqMode) freqManualBtn.textContent = `⚡ Fréquence: Auto`;
}

function startGPS() {
    if (wID !== null) stopGPS(false);
    sTime = null; 
    setGPSMode(emergencyStopActive ? 'LOW_FREQ' : 'LOW_FREQ');
    
    if ($('start-btn')) $('start-btn').disabled = true;
    if ($('stop-btn')) $('stop-btn').disabled = false;
    if ($('reset-max-btn')) $('reset-max-btn').disabled = false;
    if ($('error-message')) $('error-message').style.display = 'none';
}

function checkGPSFrequency(currentSpeed) {
    if (manualFreqMode || emergencyStopActive) {
        let newMode = emergencyStopActive ? 'LOW_FREQ' : forcedFreqState;
        if (newMode !== currentGPSMode) setGPSMode(newMode);
        return;
    }
    
    const isMovingFast = currentSpeed >= SPEED_THRESHOLD; 
    let newMode = isMovingFast ? 'HIGH_FREQ' : 'LOW_FREQ';

    if (newMode !== currentGPSMode) { setGPSMode(newMode); }
}

function emergencyStop() {
    emergencyStopActive = !emergencyStopActive;
    const stopBtn = $('emergency-stop-btn');
    const freqBtn = $('freq-manual-btn');
    const stopInd = $('emergency-status');
    
    if (emergencyStopActive) {
        stopGPS(false); 
        if (als && als.stop) als.stop(); 
        
        if (domID !== null) { clearInterval(domID); currentDOMFreq = DOM_LOW_FREQ_MS * 4; domID = setInterval(fastDOM, currentDOMFreq); }
        
        if (stopBtn) { stopBtn.textContent = '🟢 Démarrer Système'; stopBtn.style.backgroundColor = '#4CAF50'; }
        if (freqBtn) freqBtn.disabled = true;
        if (stopInd) { stopInd.textContent = 'ACTIF (Mode Sécurité/Batterie)'; stopInd.style.color = '#ff6666'; }
    } else {
        startGPS(); initALS(); 
        
        if (domID !== null) { clearInterval(domID); currentDOMFreq = manualFreqMode ? (forcedFreqState === 'HIGH_FREQ' ? DOM_HIGH_FREQ_MS : DOM_LOW_FREQ_MS) : DOM_LOW_FREQ_MS; domID = setInterval(fastDOM, currentDOMFreq); }
        
        if (stopBtn) { stopBtn.textContent = '🚨 Arrêt Urgence Batterie'; stopBtn.style.backgroundColor = '#f44336'; }
        if (freqBtn) freqBtn.disabled = false;
        if (stopInd) { stopInd.textContent = 'INACTIF'; stopInd.style.color = '#00ff99'; }
    }
}

// Fonctions de contrôle et utilitaires (Raccourcis)
function changeDisplaySize(size) {
    const root = document.documentElement;
    let factor = (size === 'SMALL') ? 0.8 : (size === 'LARGE') ? 1.2 : 1.0;
    root.style.setProperty('--global-font-factor', factor);
    if ($('display-size-indicator')) $('display-size-indicator').textContent = size;
}

function handleEnvironmentChange() { selectedEnvironment = $('environment-select').value; if ($('selected-environment-ind')) $('selected-environment-ind').textContent = selectedEnvironment; }
function handleWeatherChange() { selectedWeather = $('weather-select').value; if ($('selected-weather-ind')) $('selected-weather-ind').textContent = selectedWeather; }
function toggleManualMode() {
    manualMode = manualMode === null ? (document.body.classList.contains('night-mode') ? false : true) : (manualMode === true ? false : true);
    document.body.classList.toggle('night-mode', manualMode);
    if ($('auto-mode-btn')) $('auto-mode-btn').style.display = 'inline-block';
    updateAstro(lat ?? D_LAT, lon ?? D_LON);
}
function setAutoMode() { manualMode = null; if ($('auto-mode-btn')) $('auto-mode-btn').style.display = 'none'; updateAstro(lat ?? D_LAT, lon ?? D_LON); }
function toggleManualFreq() {
    manualFreqMode = !manualFreqMode;
    if (manualFreqMode) { forcedFreqState = 'HIGH_FREQ'; setGPSMode('HIGH_FREQ'); if ($('freq-manual-btn')) { $('freq-manual-btn').style.backgroundColor = '#007bff'; $('freq-manual-btn').textContent = '⚡ MANUEL: MAX'; } }
    else { checkGPSFrequency(kSpd); if ($('freq-manual-btn')) { $('freq-manual-btn').style.backgroundColor = '#ff4500'; $('freq-manual-btn').textContent = '⚡ Fréquence: Auto'; } }
}
function cycleForcedFreq() {
    if (!manualFreqMode) return; 
    forcedFreqState = (forcedFreqState === 'HIGH_FREQ') ? 'LOW_FREQ' : 'HIGH_FREQ';
    setGPSMode(forcedFreqState);
    if ($('freq-manual-btn')) {
        $('freq-manual-btn').textContent = `⚡ MANUEL: ${forcedFreqState === 'HIGH_FREQ' ? 'MAX' : 'MIN'}`;
        $('freq-manual-btn').style.backgroundColor = forcedFreqState === 'HIGH_FREQ' ? '#007bff' : '#4CAF50';
    }
}

/** Réinitialisation de la session (garde les réglages permanents sauf si fullReset). */
function resetDisp(fullReset = true) {
    stopGPS(false);
    if (fullReset) { 
        distM = 0; distMStartOffset = 0; maxSpd = 0; 
        tLat = null; tLon = null;
        externalBatteryLevel = 99.999;
        manualMode = null;
        localStorage.removeItem(STATE_KEY); // Efface les données persistantes
    }
    kSpd = 0; kUncert = 1000;
    lastReliableHeading = null; sTime = null; lPos = null; 
}
function resetMax() { maxSpd = 0; }
function setTarget() {
    const defaultLat = lat ?? D_LAT;
    const defaultLon = lon ?? D_LON;
    const newLatStr = prompt(`Entrez la Latitude cible (actuel: ${tLat ?? defaultLat.toFixed(6)}) :`);
    if (newLatStr !== null && !isNaN(parseFloat(newLatStr))) { tLat = parseFloat(newLatStr); }
    const newLonStr = prompt(`Entrez la Longitude cible (actuel: ${tLon ?? defaultLon.toFixed(6)}) :`);
    if (newLonStr !== null && !isNaN(parseFloat(newLonStr))) { tLon = parseFloat(newLonStr); }
    if ($('cap-dest')) $('cap-dest').textContent = 'Calcul en cours...';
}

function captureScreenshot() {
    const controls = document.querySelector('.controls');
    if (controls) controls.style.display = 'none'; 
    if (typeof html2canvas === 'function') {
        html2canvas(document.body).then(canvas => {
            if (controls) controls.style.display = 'block'; 
            const link = document.createElement('a');
            link.href = canvas.toDataURL('image/png');
            link.download = 'dashboard_capture.png';
            link.click();
        });
    } else { if (controls) controls.style.display = 'block'; alert("Erreur: Librairie html2canvas manquante."); }
}

function initAll() { 
    // Initialisation
    loadState(); // <<< CHARGEMENT DE L'ÉTAT AU DÉMARRAGE
    resetDisp(false); // Réinitialise les variables GPS/Kalman mais garde l'état de session
    syncH(); initALS(); initAdvancedSensors(); 
    fetchWeather(D_LAT, D_LON); initBattery(); changeDisplaySize('NORMAL'); initMap();

    if (domID === null) { domID = setInterval(fastDOM, DOM_LOW_FREQ_MS); fastDOM.lastSlowT = 0; currentDOMFreq = DOM_LOW_FREQ_MS; }
    
    // Événements (Aucun changement)
    if ($('start-btn')) $('start-btn').addEventListener('click', startGPS);
    if ($('stop-btn')) $('stop-btn').addEventListener('click', () => stopGPS(true));
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', resetMax);
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { if (confirm("Réinitialiser toutes les données de session ?")) { stopGPS(true); resetDisp(true); } });
    if ($('set-default-loc-btn')) $('set-default-loc-btn').addEventListener('click', () => { 
        const newLatStr = prompt(`Lat (actuel: ${D_LAT}) :`);
        if (newLatStr !== null && !isNaN(parseFloat(newLatStr))) { D_LAT = parseFloat(newLatStr); }
        const newLonStr = prompt(`Lon (actuel: ${D_LON}) :`);
        if (newLonStr !== null && !isNaN(parseFloat(newLonStr))) { D_LON = parseFloat(newLonStr); }
        alert(`Nouvelle position par défaut : Lat=${D_LAT.toFixed(4)}, Lon=${D_LON.toFixed(4)}.`);
        resetDisp(false);
    });

    if ($('set-target-btn')) $('set-target-btn').addEventListener('click', setTarget);
    if ($('toggle-mode-btn')) $('toggle-mode-btn').addEventListener('click', toggleManualMode);
    if ($('auto-mode-btn')) $('auto-mode-btn').addEventListener('click', setAutoMode);
    if ($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', () => {
        netherMode = !netherMode;
        distM = distMStartOffset; maxSpd = 0; 
        if ($('nether-indicator')) $('nether-indicator').textContent = netherMode ? "ACTIVÉ (1:8) 🔥" : "DÉSACTIVÉ (1:1)";
        $('nether-toggle-btn').textContent = netherMode ? "🌍 Overworld" : "🔥 Nether";
    });
    
    if ($('freq-manual-btn')) {
        $('freq-manual-btn').addEventListener('click', () => { if (manualFreqMode) { cycleForcedFreq(); } else { toggleManualFreq(); } });
        $('freq-manual-btn').addEventListener('dblclick', toggleManualFreq); 
    }

    if ($('capture-btn')) $('capture-btn').addEventListener('click', captureScreenshot);
    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', emergencyStop);
    if ($('size-normal-btn')) $('size-normal-btn').addEventListener('click', () => changeDisplaySize('NORMAL'));
    if ($('size-small-btn')) $('size-small-btn').addEventListener('click', () => changeDisplaySize('SMALL'));
    if ($('size-large-btn')) $('size-large-btn').addEventListener('click', () => changeDisplaySize('LARGE'));

    if ($('environment-select')) $('environment-select').addEventListener('change', handleEnvironmentChange);
    if ($('weather-select')) $('weather-select').addEventListener('change', handleWeatherChange);

    handleEnvironmentChange(); handleWeatherChange();
} 

document.addEventListener('DOMContentLoaded', initAll);

// =================================================================
// FIN DU FICHIER JAVASCRIPT COMPLET
// =================================================================
