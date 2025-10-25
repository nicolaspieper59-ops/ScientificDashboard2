// =================================================================
// BLOC 1/3 : CONSTANTES, INITIALISATION, PERSISTANCE ET SYNCHRONISATION
// =================================================================

// --- CLÉS D'API (AJOUTEZ VOTRE CLÉ METEO) ---
const API_KEYS = {
    WEATHER_API: 'VOTRE_CLE_API_METEO_ICI' 
};

// --- CONSTANTES GLOBALES ET INITIALISATION ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458, R_E = 6371000, KMH_MS = 3.6;
const MIN_DT = 0.01; 

// --- HORLOGE MAÎTRESSE ET CORRECTION DE DÉRIVE ---
let systemClockOffsetMS = 0; // Décalage entre l'horloge locale et l'horloge NTP
let lastNtpSync = 0;

function getCDate() { 
    // Retourne l'heure corrigée (synchronisée en ligne) pour tous les calculs astronomiques et temporels
    return new Date(Date.now() + systemClockOffsetMS);
}

// CONFIGURATIONS GPS POUR L'OPTIMISATION BATTERIE
// HIGH_FREQ est la "fréquence normale" pour un suivi de vitesse actif.
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};

// PARAMÈTRES AVANCÉS DU FILTRE DE KALMAN (V3.3)
const Q_NOISE = 0.01;       
const R_MIN = 0.05, R_MAX = 50.0; 
const MAX_ACC = 100, MIN_SPD = 0.001, ALT_TH = -50; 
const SPEED_THRESHOLD = 0.5; 
const G_MASS_DEFAULT = 75; // kg (Masse par défaut) 

// CONSTANTES POUR LES MODÈLES PHYSIQUES (V3.3)
const AIR_DENSITY = 1.225; 
const G_ACCEL = 9.80665;   
const CDA_EST = 0.6;       
const NETHER_RATIO = 8; 

// --- FACTEURS ENVIRONNEMENTAUX ---
const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0, DRAG_MULT: 1.0 },
    'METAL': { R_MULT: 2.5, DRAG_MULT: 1.0 },      
    'FOREST': { R_MULT: 1.5, DRAG_MULT: 1.2 },     
    'CONCRETE': { R_MULT: 3.0, DRAG_MULT: 1.0 },   
};
const WEATHER_FACTORS = {
    'CLEAR': 1.0, 'RAIN': 1.2, 'SNOW': 1.5, 'STORM': 2.0,                                   
};

// Fréquences pour la boucle principale fastDOM (Optimisation de la performance)
const DOM_LOW_FREQ_MS = 250;   
const DOM_SLOW_UPDATE_MS = 1000; 

// --- DONNÉES ÉPHÉMERIDES (Pour 12:00:00 UTC) ---
const EPHEMERIS_DATA = [
    { date: new Date(Date.UTC(2025, 9, 25, 12, 0, 0)).getTime(), eot: 15.995479772833402, solar_lon: 212.33130066388185 },
    { date: new Date(Date.UTC(2025, 9, 26, 12, 0, 0)).getTime(), eot: 16.10206103298588, solar_lon: 213.32855548364387 },
    { date: new Date(Date.UTC(2025, 9, 27, 12, 0, 0)).getTime(), eot: 16.19616300661873, solar_lon: 214.32628408755045 },
    { date: new Date(Date.UTC(2025, 9, 28, 12, 0, 0)).getTime(), eot: 16.2775677305359, solar_lon: 215.32447552860685 },
    { date: new Date(Date.UTC(2025, 9, 29, 12, 0, 0)).getTime(), eot: 16.346068511216895, solar_lon: 216.32312209682095 }
];

// --- DONNÉES CÉLESTES DÉTAILLÉES ---
const CELESTIAL_DATA = {
    '2025-10-25': {
        sun: {
            mag: '-26.75', dist: '0.99 AU', radius: '696000 Km',
            radec: '14h 00m43.9s / -12°17\'55.1"', azalt: '203°41\'04.5" / +24°29\'08.3"',
            rise: '06:26', set: '16:36', elevation: '24.49°' 
        },
        moon: {
            mag: '-10.42', dist: '404410.71 km', radius: '1737.4 Km',
            radec: '16h 58m22.6s / -28°40\'32.0"', azalt: '160°12\'47.8" / +08°36\'09.1"',
            phase: '14%', rise: '11:02', set: '17:58', time_h_l: '12:31:00'
        }
    }
};


// --- VARIABLES D'ÉTAT ---
let wID = null, domID = null, lPos = null, lat = null, lon = null, sTime = null;
let distM = 0, distMStartOffset = 0, maxSpd = 0;
let tLat = null, tLon = null;
let kSpd = 0, kUncert = 1000; 
let lastAltitudeBaro = null; 
let currentGPSMode = 'LOW_FREQ'; 
let emergencyStopActive = false;
let selectedEnvironment = 'NORMAL'; 
let selectedWeather = 'CLEAR'; 
let manualTractionForce = 0; 
let calculatedMassKg = G_MASS_DEFAULT; 
let netherMode = false; 
let lastReliableHeading = null; 

// Batteries et Autonomie (Mis à jour pour la précision)
let currentBatteryLevelPrecise = null; // Niveau précis de la batterie de l'appareil (0.000 à 100.000)
let deviceAutonomyS = Infinity;        // Autonomie en secondes (navigator.getBattery)
let externalBatteryLevel = null;       // Niveau précis de la batterie externe (null si non renseigné)
let externalAutonomyS = Infinity;      // Autonomie Externe simulée ou calculée (en secondes)

// Fréquence DOM personnalisée (par défaut 16 ms / 62.5 Hz)
let customDOMIntervalMS = 16; 

// --- VARIABLES MÉTÉO (Base) ---
let lastP_hPa = 1013.25, lastT_K = 293.15, lastH_perc = 0.5; 

// --- VARIABLES MÉTÉO (Détaillées) ---
let lastWeather = { wind_kmh: 0, clouds_perc: 0, rain_mm: 0, snow_mm: 0, uv_index: 0, air_quality: '--', boiling_point: 100 };

// --- REFERENCES DOM (Alias) ---
const $ = id => document.getElementById(id);

// ===========================================
// FONCTIONS DE PERSISTENCE D'ÉTAT
// ===========================================

const STATE_KEY = 'qdgnss_state_v4';

function saveState() {
    const state = {
        distM: distM,
        maxSpd: maxSpd,
        tLat: tLat,
        tLon: tLon,
        D_LAT: lat ?? 48.8566,
        D_LON: lon ?? 2.3522,
        netherMode: netherMode,
        externalBatteryLevel: externalBatteryLevel, // Ajout de l'état
        externalAutonomyS: externalAutonomyS,       // Ajout de l'état
        manualTractionForce: manualTractionForce,
        calculatedMassKg: calculatedMassKg,
        systemClockOffsetMS: systemClockOffsetMS,
        lastNtpSync: lastNtpSync,
        customDOMIntervalMS: customDOMIntervalMS
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
            externalBatteryLevel = state.externalBatteryLevel ?? null; // Chargement de l'état
            externalAutonomyS = state.externalAutonomyS ?? Infinity;     // Chargement de l'état
            lat = state.D_LAT;
            lon = state.D_LON; 
            manualTractionForce = state.manualTractionForce || 0;
            calculatedMassKg = state.calculatedMassKg || G_MASS_DEFAULT;
            systemClockOffsetMS = state.systemClockOffsetMS || 0;
            lastNtpSync = state.lastNtpSync || 0;
            customDOMIntervalMS = state.customDOMIntervalMS || 16;
            return true;
        }
    } catch (e) {
        console.error("Échec du chargement de l'état.", e);
    }
    return false;
}

// ===========================================
// FONCTIONS GÉO & UTILS
// ===========================================

const dist = (lat1, lon1, lat2, lon2) => {
    const R = R_E, dLat = (lat2 - lat1) * D2R, dLon = (lon2 - lon1) * D2R;
    lat1 *= D2R; lat2 *= D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
};

const bearing = (lat1, lon1, lat2, lon2) => {
    lat1 *= D2R; lon1 *= D2R; lat2 *= D2R; lon2 *= D2R;
    const y = Math.sin(lon2 - lon1) * Math.cos(lat2);
    const x = Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1) * Math.cos(lat2) * Math.cos(lon2 - lon1);
    let b = Math.atan2(y, x) * R2D;
    return (b + 360) % 360; 
};

// ===========================================
// FONCTION DE SYNCHRONISATION INTERNET (RECHARGE)
// ===========================================

/**
 * Recalibre l'horloge interne (NTP) et met à jour les données météo distantes.
 */
async function syncRemoteData() {
    if (!navigator.onLine) {
        alert("Hors ligne. Impossible de recalibrer via Internet.");
        return;
    }
    const currentLat = lat ?? 48.8566;
    const currentLon = lon ?? 2.3522;
    let successCount = 0;
    let ntpSuccess = false;

    // 1. SYNCHRONISATION NTP (Horloge Maîtresse)
    try {
        const controller = new AbortController();
        const timeoutId = setTimeout(() => controller.abort(), 5000); 

        const timeResponse = await fetch('https://worldtimeapi.org/api/ip', { cache: 'no-store', signal: controller.signal });
        clearTimeout(timeoutId);
        
        if (!timeResponse.ok) throw new Error(`Erreur HTTP: ${timeResponse.status}`);

        const data = await timeResponse.json();
        const serverTime = new Date(data.utc_datetime).getTime();
        const localTime = Date.now();
        
        systemClockOffsetMS = serverTime - localTime;
        lastNtpSync = getCDate().getTime();
        successCount++;
        ntpSuccess = true;
        console.log(`[NTP Sync] Dérive corrigée: ${systemClockOffsetMS} ms. Nouveau temps maître établi.`);
    } catch (error) {
        if (error.name === 'AbortError') {
            console.warn(`[NTP Sync] Échec de la synchronisation NTP: Timeout (5s).`);
        } else {
            console.warn(`[NTP Sync] Échec de la synchronisation NTP: ${error.message}`);
        }
    }
    
    // 2. FETCH MÉTÉO (Pour les corrections atmosphériques et l'affichage)
    try {
        if (API_KEYS.WEATHER_API !== 'VOTRE_CLE_API_METEO_ICI') {
            const weather = await fetchWeather(currentLat, currentLon);
            if (weather) {
                lastP_hPa = weather.pressure; 
                lastT_K = weather.temperature;
                lastH_perc = weather.humidity;
                
                // Stockage des données météo détaillées
                lastWeather.wind_kmh = weather.wind_kmh;
                lastWeather.clouds_perc = weather.clouds_perc;
                lastWeather.rain_mm = weather.rain_mm;
                lastWeather.snow_mm = weather.snow_mm;
                lastWeather.uv_index = weather.uv_index;
                lastWeather.air_quality = weather.air_quality;
                lastWeather.boiling_point = weather.boiling_point;
                
                successCount++;
                console.log(`[Météo Sync] Données atmosphériques mises à jour.`);
            }
        }
    } catch (error) {
        console.warn(`[Météo Sync] Échec de la mise à jour météo: ${error.message}`);
    }

    if (successCount > 0) {
        alert(`Recalibrage Internet terminé: ${successCount} points synchronisés.${ntpSuccess ? ' (Temps NTP OK)' : ' (Temps: Échec ou Timeout)'}`);
    } else {
        alert("Recalibrage Internet échoué ou aucune clé d'API fournie/valide.");
    }
    fastDOM();
}

// --- GESTION ÉVÉNEMENTIELLE DE BASE ---

/**
 * Met à jour l'état et l'autonomie de la batterie de l'appareil
 */
function updateBatteryStatus(battery) {
    const levelPrecise = battery.level * 100;
    const charging = battery.charging;
    const autonomy = battery.dischargingTime; // en secondes, ou Infinity pour charge/plein

    currentBatteryLevelPrecise = levelPrecise;
    deviceAutonomyS = autonomy;

    // Mise à jour de l'indicateur de statut pour le feedback immédiat
    const batInd = $('battery-indicator');
    if (batInd) {
        let status = 'Décharge';
        if (charging) {
            status = '🔌 Recharge';
        } else if (autonomy === Infinity) {
            status = 'Pleine/Infinie';
        } else if (autonomy === -1) {
            status = 'Inconnu';
        }
        batInd.textContent = status;
    }
}

async function initBattery() {
    if ('getBattery' in navigator) {
        try {
            const battery = await navigator.getBattery();
            updateBatteryStatus(battery);
            // Ajout des écouteurs pour la précision et l'autonomie
            battery.addEventListener('levelchange', () => updateBatteryStatus(battery));
            battery.addEventListener('chargingchange', () => updateBatteryStatus(battery));
            battery.addEventListener('dischargingtimechange', () => updateBatteryStatus(battery));
        } catch (e) {
            if ($('battery-indicator')) $('battery-indicator').textContent = 'Erreur API';
        }
    } else {
        if ($('battery-indicator')) $('battery-indicator').textContent = 'API N/A';
    }
}
// =================================================================
// BLOC 2/3 : CALCULS GÉO, LOGIQUE EKF ET MISE À JOUR DE POSITION GPS
// =================================================================

// --- ASTRO CALCULS (SYNCHRONISÉS & CORRIGÉS) ---

/**
 * Interpole l'Équation du Temps (EoT) et la Longitude Solaire pour l'instant UTC donné.
 */
function getAstroData(targetDateMS) {
    let p1 = EPHEMERIS_DATA[0], p2 = EPHEMERIS_DATA[0];
    let found = false;
    
    for (let i = 0; i < EPHEMERIS_DATA.length - 1; i++) {
        if (targetDateMS >= EPHEMERIS_DATA[i].date && targetDateMS < EPHEMERIS_DATA[i+1].date) {
            p1 = EPHEMERIS_DATA[i];
            p2 = EPHEMERIS_DATA[i+1];
            found = true;
            break;
        }
    }
    
    if (!found) {
        return (targetDateMS >= EPHEMERIS_DATA[EPHEMERIS_DATA.length - 1].date) ? EPHEMERIS_DATA[EPHEMERIS_DATA.length - 1] : EPHEMERIS_DATA[0]; 
    }

    const t = (targetDateMS - p1.date) / (p2.date - p1.date);
    
    const eot = p1.eot + (p2.eot - p1.eot) * t;
    const solar_lon = p1.solar_lon + (p2.solar_lon - p1.solar_lon) * t;

    return { eot: eot, solar_lon: solar_lon };
}

/**
 * Mise à jour des informations astronomiques.
 */
function updateAstro(latA, lonA) {
    if (latA === null || lonA === null) { latA = 48.8566; lonA = 2.3522; }

    const now = getCDate(); 
    const timeFormatOptions = { hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false };
    
    const dateKey = now.getFullYear() + '-' + ('0' + (now.getMonth() + 1)).slice(-2) + '-' + ('0' + now.getDate()).slice(-2);
    const celestial_day_data = CELESTIAL_DATA[dateKey];


    // -------------------------------------------------------------
    // CALCULS SOLAIRES (Dynamique)
    // -------------------------------------------------------------
    
    const nowUTC_ms = now.getTime() + now.getTimezoneOffset() * 60 * 1000;
    const astroData = getAstroData(nowUTC_ms);
    
    const EoT_minutes = astroData.eot; 
    const SolarLongitude = astroData.solar_lon;

    const MS_PER_DEGREE = 240000;
    const longitude_correction_ms = lonA * MS_PER_DEGREE; 
    const EoT_ms = EoT_minutes * 60 * 1000;

    const LMST_ms = nowUTC_ms + longitude_correction_ms; 
    const LMST = new Date(LMST_ms); 
    
    const LTST_ms = LMST_ms - EoT_ms;
    const LTST = new Date(LTST_ms);

    const midday_utc_ms = Date.UTC(now.getUTCFullYear(), now.getUTCMonth(), now.getUTCDate(), 12, 0, 0);
    const culmination_utc_ms = midday_utc_ms - longitude_correction_ms - EoT_ms;
    const culminationTime = new Date(culmination_utc_ms); 
    
    // Mise à jour des champs Solaires Dynamiques
    if ($('solar-true')) $('solar-true').textContent = LTST.toLocaleTimeString('fr-FR', timeFormatOptions); 
    if ($('solar-mean')) $('solar-mean').textContent = LMST.toLocaleTimeString('fr-FR', timeFormatOptions); 
    if ($('solar-culmination')) $('solar-culmination').textContent = culminationTime.toLocaleTimeString('fr-FR', timeFormatOptions);
    if ($('eot')) $('eot').textContent = `${EoT_minutes.toFixed(4)} min`;
    if ($('solar-longitude-val')) $('solar-longitude-val').textContent = `${SolarLongitude.toFixed(3)} °`;
    if ($('solar-longitude-midday')) $('solar-longitude-midday').textContent = `0 °`; 
    
    const LTST_hours = LTST.getHours() + LTST.getMinutes() / 60 + LTST.getSeconds() / 3600;
    const isDay = LTST_hours >= 6 && LTST_hours < 18;
    if ($('mode-indicator')) $('mode-indicator').textContent = isDay ? 'JOUR ☀️' : 'NUIT 🌙';

    // -------------------------------------------------------------
    // DONNÉES CÉLESTES DÉTAILLÉES (Statiques + Culmination Lunaire)
    // -------------------------------------------------------------
    const sunData = celestial_day_data ? celestial_day_data.sun : null;
    const moonData = celestial_day_data ? celestial_day_data.moon : null;
    const PLACEHOLDER = '--';
    
    // SOLEIL
    if ($('sun-magnitude')) $('sun-magnitude').textContent = sunData ? sunData.mag : PLACEHOLDER;
    if ($('sun-distance')) $('sun-distance').textContent = sunData ? sunData.dist : PLACEHOLDER;
    if ($('sun-radius')) $('sun-radius').textContent = sunData ? sunData.radius : PLACEHOLDER;
    if ($('sun-radec')) $('sun-radec').textContent = sunData ? sunData.radec : PLACEHOLDER;
    if ($('sun-azalt')) $('sun-azalt').textContent = sunData ? sunData.azalt : PLACEHOLDER;
    if ($('sun-rise-set')) $('sun-rise-set').textContent = sunData ? `${sunData.rise} / ${sunData.set}` : PLACEHOLDER;
    if ($('sun-elevation')) $('sun-elevation').textContent = sunData ? sunData.elevation : 'Calcul nécessaire...';

    // LUNE
    if ($('moon-magnitude')) $('moon-magnitude').textContent = moonData ? moonData.mag : PLACEHOLDER;
    if ($('moon-distance')) $('moon-distance').textContent = moonData ? moonData.dist : PLACEHOLDER;
    if ($('moon-radius')) $('moon-radius').textContent = moonData ? moonData.radius : PLACEHOLDER;
    if ($('moon-radec')) $('moon-radec').textContent = moonData ? moonData.radec : PLACEHOLDER;
    if ($('moon-azalt')) $('moon-azalt').textContent = moonData ? moonData.azalt : PLACEHOLDER;
    if ($('lunar-phase-perc')) $('lunar-phase-perc').textContent = moonData ? moonData.phase : PLACEHOLDER;
    if ($('moon-rise-set')) $('moon-rise-set').textContent = moonData ? `${moonData.rise} / ${moonData.set}` : PLACEHOLDER;
    if ($('lunar-time')) $('lunar-time').textContent = now.toLocaleTimeString('fr-FR', timeFormatOptions);

    if (moonData && moonData.rise && moonData.set) {
        const riseParts = moonData.rise.split(':').map(Number);
        const setParts = moonData.set.split(':').map(Number);
        let riseMins = riseParts[0] * 60 + riseParts[1];
        let setMins = setParts[0] * 60 + setParts[1];
        if (setMins < riseMins) setMins += 24 * 60; 

        const culminationMins = (riseMins + setMins) / 2;
        let culmHour = Math.floor(culminationMins % (24 * 60) / 60);
        let culmMin = Math.round(culminationMins % 60);

        const culmTimeStr = `${String(culmHour).padStart(2, '0')}:${String(culmMin).padStart(2, '0')}`;
        if ($('moon-culmination')) $('moon-culmination').textContent = culmTimeStr;
    } else {
        if ($('moon-culmination')) $('moon-culmination').textContent = PLACEHOLDER;
    }

    if (celestial_day_data === null) {
         if ($('sun-azalt')) $('sun-azalt').textContent = "Données jour actuel N/A";
         if ($('moon-azalt')) $('moon-azalt').textContent = "Données jour actuel N/A";
    }
}

// --- CORRECTIONS MÉTÉO/TROPOSPHÈRE ---

function getTroposphericDelay(P_hPa, T_K, H_frac, alt_m, lat_deg) {
    return 2.3 * (1 + (P_hPa - 1013.25) / 1000); 
}

function getKalmanR(acc, alt, ztd_m) {
    let R = acc ** 2; 
    R = Math.max(R_MIN, Math.min(R_MAX, R));
    const envFactor = ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT;
    const weatherFactor = WEATHER_FACTORS[selectedWeather];
    R *= envFactor;
    R *= weatherFactor;
    if (ztd_m > 2.5) { R *= 1.1; }
    R = Math.max(R_MIN, Math.min(R_MAX, R));
    if (emergencyStopActive) { R = Math.min(R, 10.0); }
    return R;
}

function getCurrentDragMultiplier() {
    return ENVIRONMENT_FACTORS[selectedEnvironment].DRAG_MULT;
}

/**
 * Simulation du retour météo avec toutes les données demandées.
 */
async function fetchWeather(latA, lonA) {
    const P = 1012.0;
    const T_K = 288.15; // 15 C en K
    
    return {
        pressure: P,
        temperature: T_K, 
        humidity: 0.6,
        condition: 'Clear',
        wind_kmh: 15.5,
        clouds_perc: 20,
        rain_mm: 0,
        snow_mm: 0,
        uv_index: 5,
        air_quality: 'Moyenne (AQI: 65)',
        boiling_point: 100 + ((1013.25 - P) / 33.3) 
    };
}


/**
 * Gestionnaire principal de l'événement GPS watchPosition (Mise à jour EKF/Physique)
 */
function updateDisp(pos) {
    lat = pos.coords.latitude; lon = pos.coords.longitude;
    const alt = pos.coords.altitude, acc = pos.coords.accuracy;
    const spd_gps = pos.coords.speed, cTime = pos.timestamp; 

    if (sTime === null) { sTime = getCDate().getTime(); distMStartOffset = distM; }

    // --- Validation de l'Acquisition ---
    if (acc > MAX_ACC) { 
        if ($('gps-accuracy')) $('gps-accuracy').textContent = `❌ ${acc.toFixed(0)} m`; 
        if (lPos === null) lPos = pos; return; 
    }
    
    // --- Calcul de Vitesse et Accélération Instantanée ---
    let spdH = spd_gps ?? 0, spdV = 0;
    const dt = lPos ? (cTime - lPos.timestamp) / 1000 : MIN_DT; 

    if (lPos && dt > 0.05) { 
        const dH = dist(lPos.coords.latitude, lPos.coords.longitude, lat, lon); 
        if (spd_gps === null || spd_gps === undefined) spdH = dH / dt; 
        if (alt !== null && lPos.coords.altitude !== null) spdV = (alt - lPos.coords.altitude) / dt; 
    }
    
    const spd3D = Math.sqrt(spdH ** 2 + spdV ** 2); 
    const lastSpd3D = lPos ? (lPos.speedMS_3D_LAST ?? 0) : 0;
    const accellLong = dt > 0 ? (spd3D - lastSpd3D) / dt : 0; 

    // --- LOGIQUE EKF (Filtre de Kalman Étendu) ---
    const fSpd_PREV = lPos ? (lPos.kalman_kSpd_LAST ?? kSpd) : 0;
    const kSpd_PREDICTED = fSpd_PREV + accellLong * dt;
    const kUncert_PREDICTED = kUncert + Q_NOISE * dt;
    const ztd_m = getTroposphericDelay(lastP_hPa, lastT_K, lastH_perc, alt ?? 0, lat); 
    const R_dyn = getKalmanR(acc, alt, ztd_m); 
    let K = kUncert_PREDICTED / (kUncert_PREDICTED + R_dyn); 
    kSpd = kSpd_PREDICTED + K * (spd3D - kSpd_PREDICTED); 
    kUncert = (1 - K) * kUncert_PREDICTED;
    const sSpdFE = kSpd < MIN_SPD ? 0 : kSpd;

    // ===============================================
    // LOGIQUE D'ESTIMATION DE MASSE (V3.3)
    // ===============================================
    const dragMult = getCurrentDragMultiplier(); 
    const dragForce = 0.5 * AIR_DENSITY * CDA_EST * dragMult * sSpdFE ** 2; 
    const rollingResistance = 0.005 * G_ACCEL * calculatedMassKg; 
    const totalResistance = dragForce + rollingResistance;
    const netForce = manualTractionForce - totalResistance; 
    
    if (Math.abs(accellLong) > 0.1 && manualTractionForce !== 0) { 
        calculatedMassKg = netForce / accellLong;
        calculatedMassKg = Math.max(0.5, Math.min(2000, calculatedMassKg));
    }
    // ===============================================

    let hdg = pos.coords.heading;
    let hdg_corr = hdg;
    if (sSpdFE > SPEED_THRESHOLD && hdg !== null) { lastReliableHeading = hdg; } 
    else { hdg_corr = lastReliableHeading; }
    
    // --- Stockage des états ---
    lPos = pos; 
    lPos.speedMS_3D = spd3D; 
    lPos.timestamp = cTime; 
    lPos.speedMS_3D_LAST = spd3D; 
    lPos.kalman_R_val = R_dyn; 
    lPos.kalman_kSpd_LAST = kSpd; 

    // Gestion de la fréquence GPS dynamique (la "mise à jour de vitesse")
    checkGPSFrequency(sSpdFE); 

    distM += sSpdFE * dt * (netherMode ? NETHER_RATIO : 1); 
    
    if (spd3D * KMH_MS > maxSpd) maxSpd = spd3D * KMH_MS; 

    // --- MISE À JOUR DOM INSTANTANÉE (M/S et Mètres d'Accuracy) ---
    if ($('gps-accuracy')) $('gps-accuracy').textContent = acc.toFixed(2) + ' m';
    if ($('altitude')) $('altitude').textContent = alt !== null ? alt.toFixed(2) : '--';
    if ($('heading')) $('heading').textContent = hdg_corr !== null ? `${hdg_corr.toFixed(1)} °` : '--';
    if ($('g-force')) $('g-force').textContent = `${(accellLong / G_ACCEL).toFixed(2)}`;
    if ($('accel-long')) $('accel-long').textContent = `${accellLong.toFixed(3)}`;
    if ($('speed-3d-inst-kmh')) $('speed-3d-inst-kmh').textContent = `${(spd3D * KMH_MS).toFixed(3)}`;
    if ($('speed-3d-inst')) $('speed-3d-inst').textContent = `${spd3D.toFixed(3)}`;

}

function handleErr(err) {
    console.error(`Erreur GNSS (${err.code}): ${err.message}`);
    const errEl = $('error-message');
    if(errEl) {
        errEl.style.display = 'block';
        errEl.textContent = `❌ Erreur GNSS (${err.code}): Signal perdu. Bascule vers la prédiction/dernière position.`;
    }
}

/**
 * Ajuste la fréquence d'acquisition GPS entre LOW_FREQ et HIGH_FREQ
 * en fonction de la vitesse pour optimiser la batterie (la "mise à jour de vitesse").
 */
function checkGPSFrequency(currentSpeed) {
    const targetMode = (currentSpeed * KMH_MS > 50) ? 'HIGH_FREQ' : 'LOW_FREQ';
    if (targetMode !== currentGPSMode) {
        setGPSMode(targetMode);
    }
                }
// =================================================================
// BLOC 3/3 : CONTRÔLES, BOUCLES DOM ET FONCTIONS D'INITIALISATION
// =================================================================

/**
 * Formate l'autonomie en secondes en une chaîne lisible (h, min, sec).
 */
function formatAutonomy(seconds) {
    if (seconds === Infinity) return 'Pleine (Inf.)';
    if (seconds <= 0 || seconds === -1) return 'Inconnu / Épuisé';

    const h = Math.floor(seconds / 3600);
    const m = Math.floor((seconds % 3600) / 60);
    const s = Math.round(seconds % 60);

    if (h > 0) return `${h} h ${String(m).padStart(2, '0')} min`;
    if (m > 0) return `${m} min ${String(s).padStart(2, '0')} sec`;
    return `${s} sec`;
}

/**
 * Met à jour l'intervalle d'actualisation DOM basé sur la valeur du slider (en ms).
 * @param {string} msStr Intervalle en millisecondes (min 16, max 1000).
 */
function updateFreqFromRange(msStr) {
    const newIntervalMS = parseInt(msStr, 10);
    customDOMIntervalMS = newIntervalMS;
    
    // 1. Mettre à jour les affichages du slider
    const newFreqHz = (1000 / newIntervalMS).toFixed(1);
    if ($('current-freq-display')) $('current-freq-display').textContent = newFreqHz;
    if ($('update-frequency-ms')) $('update-frequency-ms').textContent = newIntervalMS;
    
    // 2. Redémarrer la boucle fastDOM
    if (domID && domID.intervalMS !== newIntervalMS) {
        clearInterval(domID);
        domID = setInterval(fastDOM, newIntervalMS);
        domID.intervalMS = newIntervalMS;
        if ($('update-frequency')) $('update-frequency').textContent = `${newFreqHz} Hz`; 
    }
}

/**
 * Permet à l'utilisateur de saisir manuellement le niveau de la batterie externe.
 */
function setExternalBatteryLevel() {
    const currentLevel = externalBatteryLevel === null ? 0 : externalBatteryLevel.toFixed(3);
    const newLevelStr = prompt(`Entrez le niveau de la batterie externe (en pourcentage, ex: 99.999). (Actuel: ${currentLevel} %) :`);

    if (newLevelStr !== null && !isNaN(parseFloat(newLevelStr))) {
        const newLevel = parseFloat(newLevelStr);
        externalBatteryLevel = Math.max(0, Math.min(100, newLevel)); // Clamp entre 0 et 100

        // Simulation d'une autonomie basée sur le niveau pour l'affichage
        // (Autonomie max simulée à 3 heures = 10800 secondes)
        if (externalBatteryLevel > 0) {
            externalAutonomyS = (externalBatteryLevel / 100) * 10800; 
        } else {
            externalAutonomyS = 0;
        }
    } else if (newLevelStr !== null) {
        alert("Valeur invalide. Veuillez entrer un nombre.");
    }
}


function setGPSMode(newMode) {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    currentGPSMode = newMode;
    // La fréquence d'acquisition GPS est gérée ici.
    const opts = GPS_OPTS[newMode]; 
    wID = navigator.geolocation.watchPosition(updateDisp, handleErr, opts);
    if ($('speed-source-indicator')) $('speed-source-indicator').textContent = `Source: ${newMode}`;
}

function updateAvgSpeed() {
    if (sTime === null || distM === 0) return 0;
    const elapS = (getCDate().getTime() - sTime) / 1000;
    return (distM / elapS) * KMH_MS; // Vitesse moyenne en KM/H
}

/**
 * Calcule l'heure Minecraft basée sur un cycle de 20 minutes réelles.
 */
function calculateMinecraftTime() {
    const REAL_MS_PER_MC_DAY = 20 * 60 * 1000;
    const now = getCDate();
    const startTime = new Date(0).getTime(); 
    const elapsedRealTime = now.getTime() - startTime; 
    
    const mcDayTimeMS = (elapsedRealTime % REAL_MS_PER_MC_DAY) / REAL_MS_PER_MC_DAY * (24 * 60 * 60 * 1000); 
    
    const totalSeconds = Math.floor(mcDayTimeMS / 1000);
    const seconds = totalSeconds % 60;
    const minutes = Math.floor(totalSeconds / 60) % 60;
    const hours = Math.floor(totalSeconds / 3600) % 24;
    
    return `${String(hours).padStart(2, '0')}:${String(minutes).padStart(2, '0')}:${String(seconds).padStart(2, '0')}`;
}

/**
 * Simule les données des capteurs (Lumière, Son, Niveau à Bulle).
 */
function simulateSensorData() {
    return {
        tilt: (Math.sin(performance.now() / 3000) * 5).toFixed(2), 
        light: Math.floor(Math.random() * 5000) + 100, 
        sound: (Math.random() * 40 + 50).toFixed(1)
    };
}

function fastDOM() {
    const now = getCDate().getTime(); 
    const pNow = performance.now();
    
    const sSpd = kSpd < MIN_SPD ? 0 : kSpd;
    const sSpdKMH = sSpd * KMH_MS;
    const avgSpdKMH = updateAvgSpeed(); 
    const elapS = sTime !== null ? (now - sTime) / 1000 : 0;
    const sensorData = simulateSensorData();
    
    // Mise à jour de l'heure locale et du temps écoulé
    if ($('local-time')) $('local-time').textContent = getCDate().toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit', second: '2-digit' });
    if ($('elapsed-time')) $('elapsed-time').textContent = `${elapS.toFixed(2)}`;
    
    // Mise à jour Astro 
    updateAstro(lat ?? 48.8566, lon ?? 2.3522); 

    // -----------------------------------------------------------------
    // NOUVEL AFFICHAGE PRÉCIS DE LA BATTERIE
    // -----------------------------------------------------------------
    if (currentBatteryLevelPrecise !== null) {
        if ($('battery-level-perc')) $('battery-level-perc').textContent = `${currentBatteryLevelPrecise.toFixed(3)} %`;
        const autonomyStr = formatAutonomy(deviceAutonomyS);
        if ($('device-autonomy')) $('device-autonomy').textContent = autonomyStr;
    } else {
        if ($('battery-level-perc')) $('battery-level-perc').textContent = '--.--- %';
        if ($('device-autonomy')) $('device-autonomy').textContent = 'API N/A';
    }

    if (externalBatteryLevel !== null) {
        if ($('external-battery-level')) $('external-battery-level').textContent = `${externalBatteryLevel.toFixed(3)} %`;
        const autonomyStrEx = formatAutonomy(externalAutonomyS);
        if ($('external-autonomy')) $('external-autonomy').textContent = autonomyStrEx;
        if ($('external-battery-raw')) $('external-battery-raw').textContent = `OK (${externalBatteryLevel.toFixed(1)} V)`; // Simu tension
    } else {
        if ($('external-battery-level')) $('external-battery-level').textContent = 'N/A';
        if ($('external-autonomy')) $('external-autonomy').textContent = '-- h -- min';
        if ($('external-battery-raw')) $('external-battery-raw').textContent = 'N/A';
    }
    // -----------------------------------------------------------------

    // Mise à jour des vitesses et de l'écart
    if ($('speed-stable')) $('speed-stable').textContent = `${sSpdKMH.toFixed(3)}`; 
    if ($('speed-max-kmh')) $('speed-max-kmh').textContent = `${maxSpd.toFixed(2)}`;
    if ($('speed-avg-kmh')) $('speed-avg-kmh').textContent = `${avgSpdKMH.toFixed(2)}`;

    // Vitesse détaillée 
    if ($('speed-3d-inst-mms')) $('speed-3d-inst-mms').textContent = `${(sSpd * 1000).toFixed(0)}`;

    // Distance détaillée et cosmique
    if ($('distance-m')) $('distance-m').textContent = `${distM.toFixed(2)}`;
    if ($('distance-mm')) $('distance-mm').textContent = `${(distM * 1000).toFixed(0)}`;
    
    const LIGHT_SPEED_M_S = C_L;
    const LIGHT_YEAR_M = 9.461e15;
    if ($('dist-light-s')) $('dist-light-s').textContent = `${(distM / LIGHT_SPEED_M_S).toFixed(10)}`;
    if ($('dist-al')) $('dist-al').textContent = `${(distM / LIGHT_YEAR_M).toExponential(3)}`;

    // Précision GPS (Cohérence Kalman)
    const coherence_perc = Math.min(100 * (1 - Math.min(kUncert / 1.0, 1.0)), 100);
    if ($('speed-error-perc')) $('speed-error-perc').textContent = `${coherence_perc.toFixed(1)}`;
    if ($('gps-accuracy-perc')) $('gps-accuracy-perc').textContent = `${coherence_perc.toFixed(1)}`;

    // Mise à jour des autres valeurs
    if ($('calculated-mass')) $('calculated-mass').textContent = `${calculatedMassKg.toFixed(2)}`;
    if ($('traction-force')) $('traction-force').textContent = `${manualTractionForce.toFixed(1)} N`;
    if ($('distance-km-m')) $('distance-km-m').textContent = `${(distM / 1000).toFixed(6)} km`;
    
    // Horloge Minecraft
    if ($('minecraft-time')) $('minecraft-time').textContent = calculateMinecraftTime();

    // Mise à jour des capteurs
    if ($('sensor-tilt')) $('sensor-tilt').textContent = `${sensorData.tilt} °`;
    if ($('sensor-light')) $('sensor-light').textContent = `${sensorData.light} lux`;
    if ($('sensor-sound')) $('sensor-sound').textContent = `${sensorData.sound} dB`;
    if ($('light-perc')) $('light-perc').textContent = `${(sensorData.light / 5000 * 100).toFixed(0)} %`; 
    if ($('sound-perc')) $('sound-perc').textContent = `${((sensorData.sound - 50) / 40 * 100).toFixed(0)} %`; 

    // Mise à jour de la boussole (Cible)
    if ($('target-lat')) $('target-lat').textContent = tLat !== null ? tLat.toFixed(6) : '--';
    if ($('target-lon')) $('target-lon').textContent = tLon !== null ? tLon.toFixed(6) : '--';

    if (tLat !== null && tLon !== null && lat !== null && lon !== null) {
        const destBearing = bearing(lat, lon, tLat, tLon);
        if ($('cap-dest')) $('cap-dest').textContent = `${destBearing.toFixed(1)} °`;
    }

    if ($('nether-indicator')) $('nether-indicator').textContent = netherMode ? 'ACTIVÉ (1:8)' : 'DÉSACTIVÉ (1:1)';

    // Mise à jour lente (toutes les 1s)
    if (fastDOM.lastSlowT && (pNow - fastDOM.lastSlowT) < DOM_SLOW_UPDATE_MS) return;
    fastDOM.lastSlowT = pNow;
    saveState(); 

    // Mise à jour des données météo 
    if ($('weather-temp')) $('weather-temp').textContent = `${(lastT_K - 273.15).toFixed(1)} °C`;
    if ($('weather-pressure')) $('weather-pressure').textContent = `${lastP_hPa.toFixed(2)} hPa`;
    if ($('weather-humidity')) $('weather-humidity').textContent = `${(lastH_perc * 100).toFixed(0)} %`;
    if ($('weather-wind')) $('weather-wind').textContent = `${lastWeather.wind_kmh.toFixed(1)} km/h`;
    if ($('weather-clouds')) $('weather-clouds').textContent = `${lastWeather.clouds_perc.toFixed(0)} %`;
    if ($('weather-rain')) $('weather-rain').textContent = `${lastWeather.rain_mm.toFixed(1)} mm`;
    if ($('weather-snow')) $('weather-snow').textContent = `${lastWeather.snow_mm.toFixed(1)} mm`;
    if ($('weather-uv')) $('weather-uv').textContent = `${lastWeather.uv_index.toFixed(0)}`;
    if ($('weather-air-quality')) $('weather-air-quality').textContent = `${lastWeather.air_quality}`;
    if ($('weather-boiling-point')) $('weather-boiling-point').textContent = `${lastWeather.boiling_point.toFixed(2)} °C`;
    
}

function startGPS() {
    if (wID !== null) stopGPS(false);
    sTime = null; 
    // Au démarrage, on initialise en LOW_FREQ, l'acquisition passera en HIGH_FREQ si la vitesse dépasse le seuil (cf. checkGPSFrequency)
    setGPSMode('LOW_FREQ');
    
    if ($('start-btn')) $('start-btn').disabled = true;
    if ($('stop-btn')) $('stop-btn').disabled = false;
    if ($('error-message')) $('error-message').style.display = 'none';
}

function stopGPS(clearT = true) {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    wID = null;
    currentGPSMode = 'OFF';
    
    if ($('start-btn')) $('start-btn').disabled = false;
    if ($('stop-btn')) $('stop-btn').disabled = true;

    if (clearT) { sTime = null; lPos = null; }
}

function resetDisp(fullReset = true) {
    stopGPS(false);
    if (fullReset) { 
        distM = 0; distMStartOffset = 0; maxSpd = 0; 
        tLat = null; tLon = null;
        manualTractionForce = 0;
        calculatedMassKg = G_MASS_DEFAULT;
        systemClockOffsetMS = 0;
        externalBatteryLevel = null; // Réinitialisation
        externalAutonomyS = Infinity; // Réinitialisation
        localStorage.removeItem(STATE_KEY); 
    }
    kSpd = 0; kUncert = 1000;
    lastReliableHeading = null; sTime = null; lPos = null; 
}

function resetMax() { maxSpd = 0; }

function setTarget() {
    const defaultLat = lat ?? 48.8566;
    const defaultLon = lon ?? 2.3522;
    const newLatStr = prompt(`Entrez la Latitude cible (actuel: ${tLat ?? defaultLat.toFixed(6)}) :`);
    if (newLatStr !== null && !isNaN(parseFloat(newLatStr))) { tLat = parseFloat(newLatStr); }
    const newLonStr = prompt(`Entrez la Longitude cible (actuel: ${tLon ?? defaultLon.toFixed(6)}) :`);
    if (newLonStr !== null && !isNaN(parseFloat(newLonStr))) { tLon = parseFloat(newLonStr); }
    if ($('cap-dest')) $('cap-dest').textContent = 'Calcul en cours...';
}

function setManualTraction() {
    const currentF = manualTractionForce;
    const newFStr = prompt(`Entrez la force de traction/poussée constante (en Newtons, N). (Actuel: ${currentF.toFixed(1)} N) :`);

    if (newFStr !== null && !isNaN(parseFloat(newFStr))) {
        manualTractionForce = parseFloat(newFStr);
    }
}

function emergencyStop() {
    emergencyStopActive = !emergencyStopActive;
    const stopBtn = $('emergency-stop-btn');
    const stopInd = $('emergency-status');
    
    if (emergencyStopActive) {
        stopGPS(false); 
        if (stopBtn) { stopBtn.textContent = '🟢 Démarrer Système'; stopBtn.style.backgroundColor = '#4CAF50'; }
        if (stopInd) { stopInd.textContent = 'ACTIF (Mode Sécurité/Batterie)'; stopInd.style.color = '#ff6666'; }
    } else {
        startGPS(); 
        if (stopBtn) { stopBtn.textContent = '🚨 Arrêt Urgence Batterie'; stopBtn.style.backgroundColor = '#f44336'; }
        if (stopInd) { stopInd.textContent = 'INACTIF'; stopInd.style.color = '#00ff99'; }
    }
}

// Fonctions de changement d'environnement/météo
if ($('environment-select')) {
    $('environment-select').addEventListener('change', (e) => {
        selectedEnvironment = e.target.value;
        if ($('selected-environment-ind')) $('selected-environment-ind').textContent = selectedEnvironment;
    });
}
if ($('weather-select')) {
    $('weather-select').addEventListener('change', (e) => {
        selectedWeather = e.target.value;
        if ($('selected-weather-ind')) $('selected-weather-ind').textContent = selectedWeather;
    });
}
if ($('nether-toggle-btn')) {
    $('nether-toggle-btn').addEventListener('click', () => {
        netherMode = !netherMode;
        $('nether-toggle-btn').textContent = netherMode ? '🧊 Overworld' : '🔥 Nether';
    });
}

if ($('capture-btn')) {
    $('capture-btn').addEventListener('click', () => {
        html2canvas(document.body, { allowTaint: true, useCORS: true }).then(canvas => {
            const link = document.createElement('a');
            link.download = 'dashboard-capture.png';
            link.href = canvas.toDataURL();
            link.click();
        });
    });
}

function initAll() { 
    loadState(); 
    resetDisp(false); 
    initBattery(); // Initialisation de l'API batterie interne
    
    // Initialisation de la boucle DOM (Mise à jour de vitesse d'affichage)
    if (domID === null) {
        const initialInterval = customDOMIntervalMS;
        domID = setInterval(fastDOM, initialInterval); 
        domID.intervalMS = initialInterval;
        fastDOM.lastSlowT = 0; 
        
        const initialFreqHz = (1000 / initialInterval).toFixed(1);
        if ($('update-frequency')) $('update-frequency').textContent = `${initialFreqHz} Hz`;
        
        // Événement pour le slider de fréquence
        const freqRangeSlider = $('freq-range');
        if (freqRangeSlider) {
            // Mettre à jour l'état initial du slider
            freqRangeSlider.value = initialInterval;
            $('current-freq-display').textContent = initialFreqHz;
            $('update-frequency-ms').textContent = initialInterval;
            
            // Écouteur d'événement principal
            freqRangeSlider.addEventListener('input', (e) => updateFreqFromRange(e.target.value));
        }
    }
    
    // Raccrochage des événements principaux
    if ($('start-btn')) $('start-btn').addEventListener('click', startGPS);
    if ($('stop-btn')) $('stop-btn').addEventListener('click', () => stopGPS(true));
    if ($('reset-max-btn')) $('reset-max-btn').addEventListener('click', resetMax);
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', () => { 
        if (confirm("Êtes-vous sûr de vouloir tout réinitialiser (Distance, Max, Cible) ?")) {
            stopGPS(true);
            resetDisp(true);
        }
    });
    if ($('set-target-btn')) $('set-target-btn').addEventListener('click', setTarget);
    if ($('set-mass-btn')) $('set-mass-btn').addEventListener('click', setManualTraction);
    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', emergencyStop);
    if ($('recharge-internet-btn')) $('recharge-internet-btn').addEventListener('click', syncRemoteData);
    
    // Événement pour la batterie externe
    if ($('set-external-level-btn')) $('set-external-level-btn').addEventListener('click', setExternalBatteryLevel);
    
    // Initialisation de l'état d'urgence
    if (emergencyStopActive) emergencyStop();
    
    // Premier appel de syncRemoteData si aucune synchro récente (> 24h)
    if (lastNtpSync === 0 || (getCDate().getTime() - lastNtpSync) > (24 * 3600 * 1000)) {
        syncRemoteData();
    }
}

document.addEventListener('DOMContentLoaded', initAll);
