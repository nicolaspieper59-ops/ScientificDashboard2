// =================================================================
// FICHIER JS PARTIE 1/2 : dashboard_part1.js (V3.2 Core & Init)
// Contient constantes, variables d'état, calculs Geo/Astro/Kalman,
// et les fonctions d'initialisation des capteurs/systèmes.
// Doit être chargé avant dashboard_part2.js
// =================================================================

// --- CLÉS D'API ---
const API_KEYS = {
    WEATHER_API: 'VOTRE_CLE_API_METEO_ICI' 
};

// --- CONSTANTES GLOBALES ET INITIALISATION ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458, C_S = 343, R_E = 6371000, KMH_MS = 3.6;
const OBLIQ = 23.44 * D2R, ECC = 0.0167, JD_2K = 2451545.0; 
let D_LAT = 48.8566, D_LON = 2.3522; 
const MIN_DT = 0.01; 

// CONFIGURATIONS GPS POUR L'OPTIMISATION BATTERIE
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};

// PARAMÈTRES AVANCÉS DU FILTRE DE KALMAN (V3.1)
const Q_NOISE = 0.01;       
const R_MIN = 0.05, R_MAX = 50.0; 
const MAX_ACC = 50, MIN_SPD = 0.001, ALT_TH = -50;
const SPEED_THRESHOLD = 0.5; 

// Fréquences pour la boucle principale fastDOM
const DOM_HIGH_FREQ_MS = 17;   
const DOM_LOW_FREQ_MS = 250;   
const DOM_SLOW_UPDATE_MS = 1000; 

// CONSTANTES POUR LES MODÈLES PHYSIQUES (V3.1)
const AIR_DENSITY = 1.225; 
const G_ACCEL = 9.80665;   
const CDA_EST = 0.6;       
const TROPO_K2 = 382000; 
const SUN_NIGHT_TH = -12; 
const LUX_NIGHT_TH = 50; 
const NETHER_RATIO = 8; 

// --- NOUVELLES CONSTANTES POUR FILTRAGE AMÉLIORÉ (V3.2) ---
const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0, DRAG_MULT: 1.0 },
    'METAL': { R_MULT: 2.5, DRAG_MULT: 1.0 },      
    'FOREST': { R_MULT: 1.5, DRAG_MULT: 1.2 },     
    'CONCRETE': { R_MULT: 3.0, DRAG_MULT: 1.0 },   
};
const WEATHER_FACTORS = {
    'CLEAR': 1.0,
    'RAIN': 1.2,                                   
    'SNOW': 1.5,                                   
    'STORM': 2.0,                                  
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

// --- NOUVELLES VARIABLES D'ÉTAT (V3.2) ---
let emergencyStopActive = false;
let selectedEnvironment = 'NORMAL'; 
let selectedWeather = 'CLEAR'; 
let currentBatteryLevel = 100;

// --- REFERENCES DOM ---
const $ = id => document.getElementById(id);

// ===========================================
// FONCTIONS GÉO & UTILS (V3.0/V3.1)
// ===========================================

/**
 * Calcule la distance entre deux points géographiques (Formule de Haversine).
 */
const dist = (lat1, lon1, lat2, lon2) => {
    const R = R_E, dLat = (lat2 - lat1) * D2R, dLon = (lon2 - lon1) * D2R;
    lat1 *= D2R; lat2 *= D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
};

/**
 * Calcule le cap (bearing) entre deux points géographiques.
 */
const bearing = (lat1, lon1, lat2, lon2) => {
    lat1 *= D2R; lon1 *= D2R; lat2 *= D2R; lon2 *= D2R;
    const y = Math.sin(lon2 - lon1) * Math.cos(lat2);
    const x = Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1) * Math.cos(lat2) * Math.cos(lon2 - lon1);
    let b = Math.atan2(y, x) * R2D;
    return (b + 360) % 360; 
};

/**
 * Synchronise l'heure locale avec un serveur de temps (WorldTimeAPI) pour plus de précision.
 */
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

/**
 * Renvoie l'objet Date courant, corrigé par la synchronisation serveur si disponible.
 */
function getCDate() { 
    const currentLocalTime = Date.now(); 
    if (lServH !== null && lLocH !== null) {
        const offsetSinceSync = currentLocalTime - lLocH;
        return new Date(lServH + offsetSinceSync); 
    } else {
        return new Date(currentLocalTime); 
    }
}

/**
 * Applique le filtre de Kalman à la vitesse pour la stabilisation.
 * @param {number} nSpd - Vitesse mesurée (bruitée).
 * @param {number} dt - Intervalle de temps.
 * @param {number} R_dyn - Incertitude de la mesure (Dynamique).
 * @returns {number} Vitesse filtrée (kSpd).
 */
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
// CORRECTIONS MÉTÉO/TROPOSPHÈRE (V3.1/V3.2)
// ===========================================

/**
 * Estime le délai troposphérique (ZTD) basé sur les conditions météo.
 */
function getTroposphericDelay(P_hPa, T_K, H_frac, alt_m, lat_deg) {
    const ZHD = 0.0022768 * P_hPa / (1 - 0.00266 * Math.cos(2 * lat_deg * D2R) - 0.00028 * alt_m / 1000);
    const T_C = T_K - 273.15;
    const SVP = 6.11 * Math.exp(19.7 * T_C / (T_C + 273.15)); 
    const e = H_frac * SVP; 
    const ZWD = 0.002277 * (TROPO_K2 / T_K) * (e / 100); 
    return ZHD + ZWD;
}

/**
 * Calcule l'incertitude dynamique (R) du filtre de Kalman en utilisant
 * la précision GPS, l'altitude, les corrections météo et les facteurs utilisateur.
 */
function getKalmanR(acc, alt, ztd_m) {
    let R = acc ** 2; 
    R = Math.max(R_MIN, Math.min(R_MAX, R));
    
    // Application des facteurs utilisateur (V3.2)
    const envFactor = ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT;
    const weatherFactor = WEATHER_FACTORS[selectedWeather];

    R *= envFactor;
    R *= weatherFactor;
    
    // Correction Météo (Troposphère - ZTD)
    if (ztd_m > 2.5) { R *= 1.1; }

    R = Math.max(R_MIN, Math.min(R_MAX, R));
    
    // Limitation supérieure lors de l'arrêt d'urgence
    if (emergencyStopActive) {
        R = Math.min(R, 10.0); 
    }

    return R;
}

/**
 * Renvoie le multiplicateur de traînée aérodynamique pour les calculs physiques.
 */
function getCurrentDragMultiplier() {
    return ENVIRONMENT_FACTORS[selectedEnvironment].DRAG_MULT;
}

/**
 * Simule ou récupère les données météo pour les corrections.
 */
async function fetchWeather(latA, lonA) {
    if (API_KEYS.WEATHER_API === 'VOTRE_CLE_API_METEO_ICI') {
        // Simulation des données météo si l'API est désactivée
        lastP_hPa = 1013.25 + Math.sin(Date.now() / 100000) * 1.5;
        lastT_K = 293.15 + Math.sin(Date.now() / 500000) * 5; 
        lastH_perc = 0.5 + Math.cos(Date.now() / 300000) * 0.1;
        lastAltitudeBaro = 100 + Math.sin(Date.now() / 50000) * 10; 
        return; 
    }
    // VRAI APPEL API MÉTÉO (À implémenter si l'API est fournie)
}

// ===========================================
// CALCULS ASTRO & ENV (V3.0/V3.2)
// ===========================================

/**
 * Calcule les données solaires (élévation, longitude, heure de culmination).
 */
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

/**
 * Calcule la phase lunaire (angle d'élongation du Soleil à la Lune).
 */
function calcLunarPhase() { 
    const now = getCDate();
    const JD = now.getTime() / 86400000 + 2440587.5; 
    const d = JD - JD_2K; 
    let D = 297.8501921 + 445.2671115 * d; 
    D = D % 360; 
    if (D < 0) D += 360;
    return D * D2R; 
}

/**
 * Calcule et affiche le Temps Lunaire Moyen (LMT).
 */
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

/**
 * Fonction utilitaire pour obtenir midi UTC du jour donné.
 */
function getUTCMidday(date) {
    const year = date.getFullYear();
    const month = date.getMonth();
    const day = date.getDate();
    return new Date(Date.UTC(year, month, day, 12, 0, 0));
}

/**
 * Calcule les métriques astronomiques pour midi UTC (point de référence stable).
 */
function calcMiddayMetrics() {
    const now = getCDate();
    const utcMidday = getUTCMidday(now);
    
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
// CAPTEURS AVANCÉS & SYSTÈME (INIT) (V3.1/V3.2)
// ===========================================

/**
 * Initialise le capteur de lumière ambiante (AmbientLightSensor).
 */
function initALS() {
    if ('AmbientLightSensor' in window) {
        try {
            als = new AmbientLightSensor({ frequency: 5 }); 
            als.onreading = () => {
                lastLux = als.illuminance;
                const luxEl = $('illuminance-lux');
                if (luxEl) luxEl.textContent = `${lastLux.toFixed(2)} Lux`;
            };
            als.onerror = (event) => {
                lastLux = null;
                const luxEl = $('illuminance-lux');
                if (luxEl) luxEl.textContent = 'Erreur/Non Autorisé';
                if (als) als.stop();
            };
            als.start();
        } catch (error) {
            lastLux = null;
            const luxEl = $('illuminance-lux');
            if (luxEl) luxEl.textContent = 'N/A (Navigateur)';
        }
    } else {
        lastLux = null;
        const luxEl = $('illuminance-lux');
        if (luxEl) luxEl.textContent = 'N/A (API Manquante)';
    }
}

/**
 * Initialise les capteurs d'orientation et de mouvement pour le niveau à bulle et la force G.
 */
function initAdvancedSensors() {
    if ('DeviceOrientationEvent' in window) {
        window.addEventListener('deviceorientation', (event) => {
            const alpha = event.webkitCompassHeading ?? event.alpha; 
            const beta = event.beta, gamma = event.gamma; 
            const bubbleEl = $('bubble-level'), magFieldEl = $('mag-field');
            // Affichage du niveau à bulle (beta/gamma) et du champ magnétique (alpha)
            if (bubbleEl) bubbleEl.textContent = `${beta !== null ? beta.toFixed(1) : '--'}°/${gamma !== null ? gamma.toFixed(1) : '--'}°`;
            if (magFieldEl && alpha !== null) magFieldEl.textContent = `${alpha.toFixed(1)} ° (Mag)`; 
        }, true);
    } 

    if ('DeviceMotionEvent' in window) {
        window.addEventListener('devicemotion', (event) => {
            const accel = event.acceleration;
            if (accel && accel.x !== null) {
                const a_long = accel.y ?? 0; // Accélération longitudinale (y)
                const g_force = a_long / G_ACCEL;
                const gForceEl = $('g-force'), accelLongEl = $('accel-long');
                if (gForceEl) gForceEl.textContent = `${g_force.toFixed(2)} G`;
                if (accelLongEl) accelLongEl.textContent = `${a_long.toFixed(3)} m/s²`;
            }
        }, true);
    }
}

// ===========================================
// GESTION DU SYSTÈME (V3.2)
// ===========================================

/**
 * Ajuste la taille de la police pour l'affichage (mode petit, normal, grand).
 * @param {string} size - 'SMALL', 'NORMAL', ou 'LARGE'.
 */
function changeDisplaySize(size) {
    const root = document.documentElement;
    let factor = 1.0;
    
    if (size === 'SMALL') factor = 0.8;
    else if (size === 'LARGE') factor = 1.2;
    else factor = 1.0; 
    
    root.style.setProperty('--global-font-factor', factor);
    
    const sizeInd = $('display-size-indicator');
    if (sizeInd) sizeInd.textContent = size;
}

/**
 * Gère le changement de l'environnement sélectionné par l'utilisateur.
 */
function handleEnvironmentChange() {
    const select = $('environment-select');
    selectedEnvironment = select.value;
    const envInd = $('selected-environment-ind');
    if (envInd) envInd.textContent = selectedEnvironment;
}

/**
 * Gère le changement des conditions météo sélectionnées par l'utilisateur.
 */
function handleWeatherChange() {
    const select = $('weather-select');
    selectedWeather = select.value;
    const weatherInd = $('selected-weather-ind');
    if (weatherInd) weatherInd.textContent = selectedWeather;
}

/**
 * Met à jour l'indicateur de niveau de batterie.
 */
function updateBatteryStatus(battery) {
    const level = Math.floor(battery.level * 100);
    const charging = battery.charging;
    currentBatteryLevel = level;
    
    const batInd = $('battery-indicator');
    if (batInd) batInd.textContent = `${level}% ${charging ? '🔌' : ''} (${charging ? 'Charge' : 'Décharge'})`;
    batInd.style.color = level < 20 && !charging ? '#ff6666' : '#00ff99';
}

/**
 * Initialise l'API de la batterie et écoute les changements.
 */
async function initBattery() {
    if ('getBattery' in navigator) {
        try {
            const battery = await navigator.getBattery();
            updateBatteryStatus(battery);
            
            battery.addEventListener('levelchange', () => updateBatteryStatus(battery));
            battery.addEventListener('chargingchange', () => updateBatteryStatus(battery));
        } catch (e) {
            if ($('battery-indicator')) $('battery-indicator').textContent = 'N/A (Accès Refusé)';
        }
    } else {
        if ($('battery-indicator')) $('battery-indicator').textContent = 'N/A (API Manquante)';
    }
}

/**
 * Active/désactive le mode d'arrêt d'urgence (arrêt GPS, ralenti DOM, R Kalman ajusté).
 */
function emergencyStop() {
    emergencyStopActive = !emergencyStopActive;
    
    const stopBtn = $('emergency-stop-btn');
    const freqBtn = $('freq-manual-btn');
    const stopInd = $('emergency-status');
    
    // Les fonctions stopGPS, startGPS et la gestion de la boucle DOM 
    // sont dans dashboard_part2.js, elles doivent être déclarées là-bas.
    
    if (emergencyStopActive) {
        // Fonctions dans Part 2 (doivent être définies globalement)
        if (typeof stopGPS === 'function') stopGPS(false); 
        if (als && als.stop) als.stop(); 
        
        if (domID !== null && typeof clearInterval === 'function' && typeof fastDOM === 'function') {
            clearInterval(domID);
            currentDOMFreq = DOM_LOW_FREQ_MS * 4; 
            domID = setInterval(fastDOM, currentDOMFreq);
        }
        
        if (stopBtn) { 
            stopBtn.textContent = '🟢 Démarrer Système';
            stopBtn.style.backgroundColor = '#4CAF50';
        }
        if (freqBtn) freqBtn.disabled = true;
        if (stopInd) {
            stopInd.textContent = 'ACTIF (Mode Sécurité/Batterie)';
            stopInd.style.color = '#ff6666';
        }
    } else {
        // Fonctions dans Part 2 (doivent être définies globalement)
        if (typeof startGPS === 'function') startGPS(); 
        initALS();  // Redémarre le capteur de lumière
        
        if (domID !== null && typeof clearInterval === 'function' && typeof fastDOM === 'function') {
            clearInterval(domID);
            currentDOMFreq = DOM_LOW_FREQ_MS; 
            domID = setInterval(fastDOM, currentDOMFreq);
        }
        
        if (stopBtn) {
            stopBtn.textContent = '🚨 Arrêt Urgence Batterie';
            stopBtn.style.backgroundColor = '#f44336';
        }
        if (freqBtn) freqBtn.disabled = false;
        if (stopInd) {
            stopInd.textContent = 'INACTIF';
            stopInd.style.color = '#00ff99';
        }
    }
}

// =================================================================
// FICHIER JS PARTIE 2/2 : dashboard_part2.js (V3.2 Logic & DOM)
// Contient les fonctions de mise à jour du DOM, la boucle principale,
// le traitement des données GPS (updateDisp) et les handlers d'événements.
// Nécessite dashboard_part1.js pour les constantes et les fonctions de calcul.
// =================================================================

// ===========================================
// LOGIQUE PRINCIPALE (V3.0/V3.2)
// ===========================================

/**
 * Bascule le mode jour/nuit entre manuel et automatique.
 */
function toggleManualMode() {
    manualMode = manualMode === null ? (document.body.classList.contains('night-mode') ? false : true) : (manualMode === true ? false : true);
    document.body.classList.toggle('night-mode', manualMode);
    const autoModeBtn = $('auto-mode-btn');
    if (autoModeBtn) autoModeBtn.style.display = 'inline-block';
    updateAstro(lat ?? D_LAT, lon ?? D_LON);
}

/**
 * Force le retour au mode jour/nuit automatique.
 */
function setAutoMode() {
    manualMode = null;
    const autoModeBtn = $('auto-mode-btn');
    if (autoModeBtn) autoModeBtn.style.display = 'none';
    updateAstro(lat ?? D_LAT, lon ?? D_LON);
}

/**
 * Met à jour tous les éléments d'affichage astronomiques (DOM).
 * Utilise calcSolar, calcLunarPhase, calcLunarTime de la partie 1.
 */
function updateAstro(latA, lonA) {
    const now = getCDate(); // Fonction de la partie 1
    const dateM = new Date(now.getFullYear(), now.getMonth(), now.getDate(), now.getHours(), now.getMinutes(), now.getSeconds());
    const mTime = dateM.getUTCHours() * 3600 + dateM.getUTCMinutes() * 60 + dateM.getUTCSeconds();
    const mcTime = (mTime * 10) % 24000; 
    const mcH = Math.floor(mcTime / 1000) % 24, mcM = Math.floor((mcTime % 1000) / (1000/60));
    const mcTimeEl = $('mc-time');
    if (mcTimeEl) mcTimeEl.textContent = `${String(mcH).padStart(2, '0')}:${String(mcM).padStart(2, '0')}:--`;

    const sData = calcSolar(); // Fonction de la partie 1
    const hDeg = sData.elevation;
    
    let sTimeH = (now.getUTCHours() * 3600 + now.getUTCMinutes() * 60 + now.getUTCSeconds()) / 3600;
    let trueSolarTimeH = sTimeH + (lonA / 15) + (sData.eot / 60);
    trueSolarTimeH = (trueSolarTimeH % 24 + 24) % 24;
    
    const tsH = Math.floor(trueSolarTimeH), tsM = Math.floor((trueSolarTimeH * 60) % 60), tsS = Math.floor((trueSolarTimeH * 3600) % 60);
    const tsStr = `${String(tsH).padStart(2, '0')}:${String(tsM).padStart(2, '0')}:${String(tsS).padStart(2, '0')}`;
    const solarTrueHeaderEl = $('solar-true-header'), solarTrueEl = $('solar-true');
    if (solarTrueHeaderEl) solarTrueHeaderEl.textContent = tsStr + ' (HSV)';
    if (solarTrueEl) solarTrueEl.textContent = tsStr;

    let meanSolarTimeH = sTimeH + (lonA / 15);
    meanSolarTimeH = (meanSolarTimeH % 24 + 24) % 24;
    const msH = Math.floor(meanSolarTimeH), msM = Math.floor((meanSolarTimeH * 60) % 60), msS = Math.floor((meanSolarTimeH * 3600) % 60);
    const msStr = `${String(msH).padStart(2, '0')}:${String(msM).padStart(2, '0')}:${String(msS).padStart(2, '0')}`;
    const solarMeanHeaderEl = $('solar-mean-header'), solarMeanEl = $('solar-mean');
    if (solarMeanHeaderEl) solarMeanHeaderEl.textContent = msStr + ' (LSM)';
    if (solarMeanEl) solarMeanEl.textContent = msStr;
    
    const eotEl = $('eot');
    if (eotEl) eotEl.textContent = `${sData.eot.toFixed(2)} min`;
    const sunElevationEl = $('sun-elevation');
    if (sunElevationEl) sunElevationEl.textContent = `${hDeg.toFixed(2)} °`;
    const solarLongitudeValEl = $('solar-longitude-val');
    if (solarLongitudeValEl) solarLongitudeValEl.textContent = `${sData.solarLongitude.toFixed(2)} °`;
    const solarCulminationEl = $('solar-culmination');
    if (solarCulminationEl) solarCulminationEl.textContent = sData.culmination;

    const D_rad = calcLunarPhase(); // Fonction de la partie 1
    const phasePerc = (1 + Math.cos(D_rad)) / 2 * 100; 
    const lunarPhasePercEl = $('lunar-phase-perc');
    if (lunarPhasePercEl) lunarPhasePercEl.textContent = `${phasePerc.toFixed(1)}%`;
    calcLunarTime(lonA); // Fonction de la partie 1

    const autoNight = (hDeg < SUN_NIGHT_TH) || (lastLux !== null && lastLux < LUX_NIGHT_TH); // Constantes de la partie 1
    const isNight = manualMode !== null ? manualMode : autoNight;
    
    document.body.classList.toggle('night-mode', isNight);
    
    let modeText = '';
    const modeInd = $('mode-indicator');
    if (manualMode !== null) modeText = manualMode ? 'Nuit 🌙 (Manuel)' : 'Jour ☀️ (Manuel)';
    else modeText = isNight ? 'Nuit 🌙 (Auto)' : 'Jour ☀️ (Auto)';
    if (modeInd) modeInd.textContent = modeText;
}

/**
 * Met à jour l'affichage de la distance, de la vitesse max et de la cible.
 * Utilise dist et bearing de la partie 1.
 */
function updateDM(latA, lonA) {
    if ($('distance-km-m')) $('distance-km-m').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    if ($('speed-max')) $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    
    if (tLat !== null && tLon !== null) {
        const distTarget = dist(latA, lonA, tLat, tLon);
        const capDest = bearing(latA, lonA, tLat, tLon);
        if ($('cap-dest')) $('cap-dest').textContent = `${capDest.toFixed(1)} °`;
        const distTargetEl = $('dist-target');
        if (distTargetEl) distTargetEl.textContent = `${(distTarget / 1000).toFixed(3)} km`;
    }
}

/**
 * Démarre le watchPosition du GPS avec les options de fréquence appropriées.
 */
function setGPSMode(newMode) {
    if (emergencyStopActive) { 
        newMode = 'LOW_FREQ'; 
    }
    
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    currentGPSMode = newMode;
    
    const opts = GPS_OPTS[newMode]; // Constantes de la partie 1
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

/**
 * Détermine s'il faut passer du mode LOW_FREQ au mode HIGH_FREQ (ou vice-versa) en fonction de la vitesse.
 */
function checkGPSFrequency(currentSpeed) {
    if (manualFreqMode || emergencyStopActive) {
        let newMode = emergencyStopActive ? 'LOW_FREQ' : forcedFreqState;
        if (newMode !== currentGPSMode) setGPSMode(newMode);
        return;
    }
    
    const isMovingFast = currentSpeed >= SPEED_THRESHOLD; // Constante de la partie 1
    let newMode = isMovingFast ? 'HIGH_FREQ' : 'LOW_FREQ';

    if (newMode !== currentGPSMode) {
        setGPSMode(newMode);
    }
}

/**
 * Active/désactive le mode de contrôle manuel de la fréquence GPS.
 */
function toggleManualFreq() {
    const freqManualBtn = $('freq-manual-btn');
    manualFreqMode = !manualFreqMode;

    if (manualFreqMode) {
        forcedFreqState = 'HIGH_FREQ'; 
        setGPSMode('HIGH_FREQ');
        if (freqManualBtn) {
            freqManualBtn.style.backgroundColor = '#007bff';
            freqManualBtn.textContent = '⚡ MANUEL: MAX';
        }
    } else {
        checkGPSFrequency(kSpd); 
        if (freqManualBtn) {
            freqManualBtn.style.backgroundColor = '#ff4500';
            freqManualBtn.textContent = '⚡ Fréquence: Auto';
        }
    }
}

/**
 * Cycle la fréquence forcée entre MAX et MIN en mode manuel.
 */
function cycleForcedFreq() {
    const freqManualBtn = $('freq-manual-btn');
    if (!manualFreqMode) return; 

    if (forcedFreqState === 'HIGH_FREQ') {
        forcedFreqState = 'LOW_FREQ';
        setGPSMode('LOW_FREQ');
        if (freqManualBtn) {
            freqManualBtn.textContent = '⚡ MANUEL: MIN';
            freqManualBtn.style.backgroundColor = '#4CAF50';
        }
    } else {
        forcedFreqState = 'HIGH_FREQ';
        setGPSMode('HIGH_FREQ');
        if (freqManualBtn) {
            freqManualBtn.textContent = '⚡ MANUEL: MAX';
            freqManualBtn.style.backgroundColor = '#007bff';
        }
    }
}

/**
 * Boucle principale pour les mises à jour rapides du DOM.
 */
function fastDOM() {
    const latA = lat ?? D_LAT, lonA = lon ?? D_LON;
    const now = getCDate().getTime(); // Fonction de la partie 1
    const pNow = performance.now();
    
    const updateFrequencyEl = $('update-frequency');
    if (lDomT && updateFrequencyEl) updateFrequencyEl.textContent = `${(1000 / (pNow - lDomT)).toFixed(1)} Hz (DOM)`;
    lDomT = pNow;

    updateAstro(latA, lonA); 
    
    // Contrôle de la fréquence lente (1 Hz)
    if (fastDOM.lastSlowT && (pNow - fastDOM.lastSlowT) < DOM_SLOW_UPDATE_MS) return;
    
    fastDOM.lastSlowT = pNow;

    updateDM(latA, lonA); 
    
    // Calcul et affichage des métriques de midi UTC (V3.2)
    const middayData = calcMiddayMetrics(); // Fonction de la partie 1
    const solarLongitudeMiddayEl = $('solar-longitude-midday');
    if (solarLongitudeMiddayEl) solarLongitudeMiddayEl.textContent = `${middayData.solarLongitude.toFixed(8)} °`;
    const eotMiddayEl = $('eot-midday');
    if (eotMiddayEl) eotMiddayEl.textContent = `${middayData.eot.toFixed(4)} min`;
    
    if (!lPos || sTime === null) { return; }
    
    const spd3D = lPos.speedMS_3D || 0, spd3DKMH = spd3D * KMH_MS; // Constantes de la partie 1
    const sSpd = kSpd < MIN_SPD ? 0 : kSpd, sSpdKMH = sSpd * KMH_MS; // Constantes de la partie 1
    const elapS = (now - sTime) / 1000;
    
    // Affichage des vitesses
    if ($('speed-3d-inst')) $('speed-3d-inst').textContent = `${spd3DKMH.toFixed(5)} km/h`; 
    if ($('speed-ms')) $('speed-ms').textContent = `${spd3D.toFixed(5)} m/s`; 
    if ($('perc-light')) $('perc-light').textContent = `${(spd3D / C_L * 100).toPrecision(5)}%`; // Constantes de la partie 1
    if ($('perc-sound')) $('perc-sound').textContent = `${(spd3D / C_S * 100).toPrecision(5)}%`; // Constantes de la partie 1
    if ($('speed-stable')) $('speed-stable').textContent = `${sSpdKMH.toFixed(5)} km/h`; 
    if ($('speed-stable-mm')) $('speed-stable-mm').textContent = `${(sSpd * 1000).toFixed(2)} mm/s`;
    if ($('elapsed-time')) $('elapsed-time').textContent = `${elapS.toFixed(2)} s`;

    // Affichage Cohérence Kalman
    const kR = lPos.kalman_R_val || R_MAX; // Constantes de la partie 1
    const uncertainty_ratio = Math.min(kUncert / 1.0, 1.0); 
    const coherence_perc = 100 * (1 - uncertainty_ratio);
    if ($('speed-error-perc')) $('speed-error-perc').textContent = `${coherence_perc.toFixed(1)}% (R:${kR.toFixed(3)})`;
    
    // Affichage des données Météo/Troposphère
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

/**
 * Gestionnaire principal de l'événement GPS watchPosition.
 * C'est ici que le filtre de Kalman est appliqué et que toutes les métriques sont calculées.
 */
function updateDisp(pos) {
    lat = pos.coords.latitude; lon = pos.coords.longitude;
    const alt = pos.coords.altitude, acc = pos.coords.accuracy;
    const spd = pos.coords.speed, cTime = pos.timestamp; 

    syncH(); // Fonction de la partie 1

    if (sTime === null) { 
        sTime = getCDate().getTime(); // Fonction de la partie 1
        distMStartOffset = distM; 
    }

    if (acc > MAX_ACC) { // Constantes de la partie 1
        if ($('gps-accuracy')) $('gps-accuracy').textContent = `❌ ${acc.toFixed(0)} m (Trop Imprécis)`; 
        if (lPos === null) lPos = pos; return; 
    }
    
    let spdH = spd ?? 0, spdV = 0;
    const dt = lPos ? (cTime - lPos.timestamp) / 1000 : MIN_DT; // Constantes de la partie 1

    if (lPos && dt > 0.1) { 
        const dH = dist(lPos.coords.latitude, lPos.coords.longitude, lat, lon); // Fonction de la partie 1
        if (spd === null || spd === undefined) spdH = dH / dt; 
        if (alt !== null && lPos.coords.altitude !== null) spdV = (alt - lPos.coords.altitude) / dt; 
    }
    
    const spd3D = Math.sqrt(spdH ** 2 + spdV ** 2);

    // --- CORRECTION GNSS AVANCÉE ---
    // Fonctions et constantes de la partie 1
    const ztd_m = getTroposphericDelay(lastP_hPa, lastT_K, lastH_perc, alt ?? 0, lat); 
    const R_dyn = getKalmanR(acc, alt, ztd_m); 
    
    const fSpd = kFilter(spd3D, dt, R_dyn), sSpdFE = fSpd < MIN_SPD ? 0 : fSpd;
    
    let hdg = pos.coords.heading;
    let hdg_corr = hdg;
    if (sSpdFE > SPEED_THRESHOLD && hdg !== null) { // Constantes de la partie 1
        lastReliableHeading = hdg; 
    } else {
        hdg_corr = lastReliableHeading; 
    }
    
    // --- MISE À JOUR DES ÉTATS ET MÉTRIQUES ---
    const lastSpd3D = lPos ? (lPos.speedMS_3D_LAST ?? 0) : 0;
    const accellLong = dt > 0 ? (spd3D - lastSpd3D) / dt : 0;
    const gForce = accellLong / G_ACCEL; // Constantes de la partie 1
    
    // Nouvelle force de traînée avec multiplicateur d'environnement (V3.2)
    const dragMult = getCurrentDragMultiplier(); // Fonction de la partie 1
    const dragForce = 0.5 * AIR_DENSITY * CDA_EST * dragMult * sSpdFE ** 2; // Constantes de la partie 1
    const dragPower = dragForce * sSpdFE; 
    
    lPos = pos; lPos.speedMS_3D = spd3D; lPos.timestamp = cTime; 
    lPos.speedMS_3D_LAST = spd3D; 
    lPos.kalman_R_val = R_dyn; 
    lPos.kalman_kSpd_LAST = fSpd; 

    checkGPSFrequency(sSpdFE); 

    distM += sSpdFE * dt * (netherMode ? NETHER_RATIO : 1); // Constantes de la partie 1
    
    const elapS = (getCDate().getTime() - sTime) / 1000;
    const sessionDistM = distM - distMStartOffset;
    const spdAvg = elapS > 0 ? sessionDistM / elapS : 0; 
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    
    // --- AFFICHAGE BASIQUE (Le reste est fait dans fastDOM) ---
    let pText = `${acc.toFixed(2)} m`;
    pText += R_dyn <= R_MIN * 1.5 ? ' (Optimal/Corrigé)' : ' (Ajusté)'; // Constantes de la partie 1

    if ($('gps-accuracy')) $('gps-accuracy').textContent = pText;
    if ($('speed-avg')) $('speed-avg').textContent = `${(spdAvg * KMH_MS).toFixed(5)} km/h`; 
    if ($('speed-max')) $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    if ($('latitude')) $('latitude').textContent = `${lat.toFixed(6)}`;
    if ($('longitude')) $('longitude').textContent = `${lon.toFixed(6)}`;
    if ($('altitude')) $('altitude').textContent = `${alt !== null ? alt.toFixed(2) : '--'} m`;
    if ($('underground')) $('underground').textContent = alt !== null && alt < ALT_TH ? 'Oui' : 'Non'; // Constantes de la partie 1
    if ($('heading')) $('heading').textContent = hdg_corr !== null ? `${hdg_corr.toFixed(1)} °` : '--';
    if ($('distance-km-m')) $('distance-km-m').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    if ($('vertical-speed')) $('vertical-speed').textContent = `${spdV.toFixed(2)} m/s`;
    if ($('drag-force')) $('drag-force').textContent = `${dragForce.toFixed(1)} N`;
    if ($('drag-power-kw')) $('drag-power-kw').textContent = `${(dragPower / 1000).toFixed(2)} kW`;
    if ($('g-force')) $('g-force').textContent = `${gForce.toFixed(2)} G`;
    if ($('accel-long')) $('accel-long').textContent = `${accellLong.toFixed(3)} m/s²`;
    
    if (Date.now() - (updateDisp.lastWeatherFetch ?? 0) > 10000) {
        fetchWeather(lat, lon); // Fonction de la partie 1
        updateDisp.lastWeatherFetch = Date.now();
    }
}

/**
 * Gestionnaire des erreurs GPS.
 */
function handleErr(err) {
    console.error(`Erreur GNSS (${err.code}): ${err.message}`);
    const errEl = $('error-message');
    if(errEl) {
        errEl.style.display = 'block';
        errEl.textContent = `❌ Erreur GNSS (${err.code}): Signal perdu. Bascule vers la prédiction/dernière position.`;
    }
    stopGPS(false);
}

/**
 * Démarre le suivi GPS.
 */
function startGPS() {
    if (wID !== null) stopGPS(false);
    sTime = null; 
    setGPSMode(emergencyStopActive ? 'LOW_FREQ' : 'LOW_FREQ');
    
    if ($('start-btn')) $('start-btn').disabled = true;
    if ($('stop-btn')) $('stop-btn').disabled = false;
    if ($('reset-max-btn')) $('reset-max-btn').disabled = false;
    if ($('error-message')) $('error-message').style.display = 'none';
}

/**
 * Arrête le suivi GPS.
 */
function stopGPS(clearT = true) {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    wID = null;
    currentGPSMode = 'OFF';
    
    if ($('speed-source-indicator')) $('speed-source-indicator').textContent = 'Source: OFF';

    if ($('start-btn')) $('start-btn').disabled = false;
    if ($('stop-btn')) $('stop-btn').disabled = true;
    if ($('reset-max-btn')) $('reset-max-btn').disabled = false;

    if (clearT) { 
        sTime = null;
        lPos = null;
    }
}

/**
 * Réinitialise toutes les variables de session.
 */
function resetDisp() {
    stopGPS(false);
    distM = 0; distMStartOffset = 0; maxSpd = 0; 
    kSpd = 0; kUncert = 1000;
    tLat = null; tLon = null;
    lastReliableHeading = null;
    sTime = null;
    lPos = null; 
}

/**
 * Réinitialise uniquement la vitesse maximale.
 */
function resetMax() { maxSpd = 0; }

/**
 * Définit une nouvelle latitude/longitude cible via une invite.
 */
function setTarget() {
    const defaultLat = lat ?? D_LAT;
    const defaultLon = lon ?? D_LON;
    const newLatStr = prompt(`Entrez la Latitude cible (actuel: ${tLat ?? defaultLat.toFixed(6)}) :`);
    if (newLatStr !== null && !isNaN(parseFloat(newLatStr))) { tLat = parseFloat(newLatStr); }
    const newLonStr = prompt(`Entrez la Longitude cible (actuel: ${tLon ?? defaultLon.toFixed(6)}) :`);
    if (newLonStr !== null && !isNaN(parseFloat(newLonStr))) { tLon = parseFloat(newLonStr); }
    if ($('cap-dest')) $('cap-dest').textContent = 'Calcul en cours...';
}

/**
 * Capture l'écran du tableau de bord (nécessite html2canvas dans le HTML).
 */
function captureScreenshot() {
    const controls = document.querySelector('.controls');
    if (controls) controls.style.display = 'none'; 
    
    // html2canvas doit être chargé dans le fichier HTML
    if (typeof html2canvas === 'function') {
        html2canvas(document.body).then(canvas => {
            if (controls) controls.style.display = 'block'; 
            const link = document.createElement('a');
            link.href = canvas.toDataURL('image/png');
            link.download = 'dashboard_capture.png';
            link.click();
        });
    } else {
        if (controls) controls.style.display = 'block'; 
        alert("Erreur: La librairie html2canvas n'est pas chargée.");
    }
}

// ===========================================
// INITIALISATION DES ÉVÉNEMENTS DOM
// ===========================================

document.addEventListener('DOMContentLoaded', () => {
    // Références DOM
    const resetAllBtn = $('reset-all-btn'), startBtn = $('start-btn'), stopBtn = $('stop-btn');
    const resetMaxBtn = $('reset-max-btn'), setTargetBtn = $('set-target-btn'), toggleModeBtn = $('toggle-mode-btn');
    const autoModeBtn = $('auto-mode-btn'), netherToggleBtn = $('nether-toggle-btn'), freqManualBtn = $('freq-manual-btn');
    const captureBtn = $('capture-btn'), setDefaultLocBtn = $('set-default-loc-btn');
    const emergencyStopBtn = $('emergency-stop-btn');
    const sizeNormalBtn = $('size-normal-btn'), sizeSmallBtn = $('size-small-btn'), sizeLargeBtn = $('size-large-btn');
    const environmentSelect = $('environment-select');
    const weatherSelect = $('weather-select');

    resetDisp();
    
    // Fonctions de la partie 1
    syncH(); 
    initALS(); 
    initAdvancedSensors(); 
    fetchWeather(D_LAT, D_LON); 
    initBattery(); 
    changeDisplaySize('NORMAL'); 

    if (domID === null) {
        domID = setInterval(fastDOM, DOM_LOW_FREQ_MS); 
        fastDOM.lastSlowT = 0; 
        currentDOMFreq = DOM_LOW_FREQ_MS;
    }
    
    // Connexion des événements principaux
    if (startBtn) startBtn.addEventListener('click', startGPS);
    if (stopBtn) stopBtn.addEventListener('click', stopGPS);
    if (resetMaxBtn) resetMaxBtn.addEventListener('click', resetMax);
    
    if (resetAllBtn) resetAllBtn.addEventListener('click', () => { 
        if (confirm("Êtes-vous sûr de vouloir tout réinitialiser (Distance, Max, Cible) ?")) {
            stopGPS(true);
            resetDisp();
        }
    });

    if (setDefaultLocBtn) setDefaultLocBtn.addEventListener('click', () => { 
        const newLatStr = prompt(`Entrez la nouvelle Latitude par défaut (actuel: ${D_LAT}) :`);
        if (newLatStr !== null && !isNaN(parseFloat(newLatStr))) { D_LAT = parseFloat(newLatStr); }
        const newLonStr = prompt(`Entrez la nouvelle Longitude par défaut (actuel: ${D_LON}) :`);
        if (newLonStr !== null && !isNaN(parseFloat(newLonStr))) { D_LON = parseFloat(newLonStr); }
        alert(`Nouvelle position par défaut : Lat=${D_LAT.toFixed(4)}, Lon=${D_LON.toFixed(4)}.`);
        resetDisp();
    });

    if (setTargetBtn) setTargetBtn.addEventListener('click', setTarget);
    if (toggleModeBtn) toggleModeBtn.addEventListener('click', toggleManualMode);
    if (autoModeBtn) autoModeBtn.addEventListener('click', setAutoMode);
    
    if (netherToggleBtn) netherToggleBtn.addEventListener('click', () => {
        netherMode = !netherMode;
        distM = distMStartOffset; 
        maxSpd = 0; 
        if ($('nether-indicator')) $('nether-indicator').textContent = netherMode ? "ACTIVÉ (1:8) 🔥" : "DÉSACTIVÉ (1:1)";
        netherToggleBtn.textContent = netherMode ? "🌍 Overworld" : "🔥 Nether";
    });
    
    if (freqManualBtn) {
        freqManualBtn.addEventListener('click', () => {
            if (manualFreqMode) { cycleForcedFreq(); } else { toggleManualFreq(); }
        });
        freqManualBtn.addEventListener('dblclick', toggleManualFreq); 
    }

    if (captureBtn) captureBtn.addEventListener('click', captureScreenshot);
    
    // ÉVÉNEMENTS V3.2 (Fonctions dans Part 1)
    if (emergencyStopBtn) emergencyStopBtn.addEventListener('click', emergencyStop);
    
    if (sizeNormalBtn) sizeNormalBtn.addEventListener('click', () => changeDisplaySize('NORMAL'));
    if (sizeSmallBtn) sizeSmallBtn.addEventListener('click', () => changeDisplaySize('SMALL'));
    if (sizeLargeBtn) sizeLargeBtn.addEventListener('click', () => changeDisplaySize('LARGE'));

    if (environmentSelect) environmentSelect.addEventListener('change', handleEnvironmentChange);
    if (weatherSelect) weatherSelect.addEventListener('change', handleWeatherChange);

    // Initialisation des sélections pour les indicateurs
    handleEnvironmentChange();
    handleWeatherChange();
});
