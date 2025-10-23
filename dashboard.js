// =================================================================
// FICHIER COMPLET ET STABLE : dashboard.js (V3.1 - Fusion Avancée)
// BLOC 1 SUR 2 (CORE, KALMAN, ASTRO, CAPTEURS)
// =================================================================

// --- CLÉS D'API (Ajouté) ---
const API_KEYS = {
    WEATHER_API: 'VOTRE_CLE_API_METEO_ICI' 
};

// --- CONSTANTES GLOBALES ET INITIALISATION ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458, C_S = 343, R_E = 6371000, KMH_MS = 3.6;
const OBLIQ = 23.44 * D2R, ECC = 0.0167, JD_2K = 2451545.0; 
let D_LAT = 48.8566, D_LON = 2.3522; 
const MIN_DT = 0.01; // Temps minimum pour les calculs de vitesse

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

// --- REFERENCES DOM ---
const $ = id => document.getElementById(id);

// ===========================================
// FONCTIONS GÉO & UTILS (V3.0/V3.1)
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

function getCDate() { 
    const currentLocalTime = Date.now(); 
    if (lServH !== null && lLocH !== null) {
        const offsetSinceSync = currentLocalTime - lLocH;
        return new Date(lServH + offsetSinceSync); 
    } else {
        return new Date(currentLocalTime); 
    }
}

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
// CORRECTIONS MÉTÉO/TROPOSPHÈRE (V3.1)
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
    if (acc > 5.0 && alt !== null && alt < 100) { R *= 1.5; }
    if (ztd_m > 2.5) { R *= 1.2; }
    R = Math.max(R_MIN, Math.min(R_MAX, R));
    return R;
}

async function fetchWeather(latA, lonA) {
    if (API_KEYS.WEATHER_API === 'VOTRE_CLE_API_METEO_ICI') {
        lastP_hPa = 1013.25 + Math.sin(Date.now() / 100000) * 1.5;
        lastT_K = 293.15 + Math.sin(Date.now() / 500000) * 5; 
        lastH_perc = 0.5 + Math.cos(Date.now() / 300000) * 0.1;
        lastAltitudeBaro = 100 + Math.sin(Date.now() / 50000) * 10; 
        return; 
    }
    // VRAI APPEL API MÉTÉO ICI (Non inclus pour la concision)
}

// ===========================================
// CALCULS ASTRO & ENV (V3.0)
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

// ===========================================
// CALCULS ASTRO & ENV (V3.0) - NOUVELLE LOGIQUE
// ===========================================

// Fonction utilitaire pour obtenir midi UTC pour une date locale donnée
function getUTCMidday(date) {
    const year = date.getFullYear();
    const month = date.getMonth();
    const day = date.getDate();
    // Créer une nouvelle Date à midi (12h00:00) en UTC
    const midday = new Date(Date.UTC(year, month, day, 12, 0, 0));
    return midday;
}

// Fonction de calcul des métriques de midi UTC
function calcMiddayMetrics() {
    // 1. Déterminer la date d'aujourd'hui (Locale)
    const now = getCDate();
    
    // 2. Créer une Date de référence à 12:00:00 UTC pour cette journée
    const utcMidday = getUTCMidday(now);
    
    // 3. Effectuer le calcul solaire avec la Date de midi UTC
    // REMARQUE: Nous dupliquons/adaptons calcSolar pour utiliser un temps spécifique
    const J2K_MS = 946728000000;
    const D = (utcMidday.getTime() - J2K_MS) / 86400000;
    const M = (357.529 + 0.98560028 * D) * D2R; 
    const L = (280.466 + 0.98564736 * D) * D2R; 
    const Ce = 2 * ECC * Math.sin(M) + 1.25 * ECC ** 2 * Math.sin(2 * M);
    const lambda = L + Ce; // Longitude Solaire
    
    let sLon = (lambda * R2D) % 360;
    if (sLon < 0) sLon += 360;
    
    const alpha = Math.atan2(Math.cos(OBLIQ) * Math.sin(lambda), Math.cos(lambda));
    let EoT_deg = (L - alpha) * R2D;
    while (EoT_deg > 180) EoT_deg -= 360;
    while (EoT_deg < -180) EoT_deg += 360;
    const EoT_m = EoT_deg * 4; 
    
    return { 
        solarLongitude: sLon, 
        eot: EoT_m 
}
// ===========================================
// CAPTEURS AVANCÉS (V3.0/V3.1)
// ===========================================

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

function initAdvancedSensors() {
    // 1. Device Orientation (Boussole/Inclinaison)
    if ('DeviceOrientationEvent' in window) {
        window.addEventListener('deviceorientation', (event) => {
            const alpha = event.webkitCompassHeading ?? event.alpha; 
            const beta = event.beta, gamma = event.gamma; 
            const bubbleEl = $('bubble-level'), magFieldEl = $('mag-field');
            if (bubbleEl) bubbleEl.textContent = `${beta !== null ? beta.toFixed(1) : '--'}°/${gamma !== null ? gamma.toFixed(1) : '--'}°`;
            if (magFieldEl && alpha !== null) magFieldEl.textContent = `${alpha.toFixed(1)} μT (Mag)`; 
        }, true);
    } 

    // 2. Proximité et Accélération (DeviceMotionEvent)
    if ('DeviceMotionEvent' in window) {
        window.addEventListener('devicemotion', (event) => {
            const accel = event.acceleration;
            if (accel && accel.x !== null) {
                const a_long = accel.y ?? 0; 
                const g_force = a_long / G_ACCEL;
                const gForceEl = $('g-force'), accelLongEl = $('accel-long');
                if (gForceEl) gForceEl.textContent = `${g_force.toFixed(2)} G`;
                if (accelLongEl) accelLongEl.textContent = `${a_long.toFixed(3)} m/s²`;
                updateDisp.g_lat_read = accel.x / G_ACCEL; 
            }
        }, true);
    }
}

// FIN DU BLOC 1/2
// =================================================================
// =================================================================
// FICHIER COMPLET ET STABLE : dashboard.js (V3.1 - Fusion Avancée)
// BLOC 2 SUR 2 (LOGIQUE, AFFICHAGE, TRAITEMENT GPS, CYCLE DE VIE)
// =================================================================

// Les variables et fonctions du Bloc 1 sont accessibles ici.

// ===========================================
// GESTION DU MODE JOUR/NUIT ET AFFICHAGE ASTRO (V3.0)
// ===========================================

function toggleManualMode() {
    manualMode = manualMode === null ? (document.body.classList.contains('night-mode') ? false : true) : (manualMode === true ? false : true);
    document.body.classList.toggle('night-mode', manualMode);
    const autoModeBtn = $('auto-mode-btn');
    if (autoModeBtn) autoModeBtn.style.display = 'inline-block';
    updateAstro(lat ?? D_LAT, lon ?? D_LON);
}

function setAutoMode() {
    manualMode = null;
    const autoModeBtn = $('auto-mode-btn');
    if (autoModeBtn) autoModeBtn.style.display = 'none';
    updateAstro(lat ?? D_LAT, lon ?? D_LON);
}

function updateAstro(latA, lonA) {
    const now = getCDate();
    const dateM = new Date(now.getFullYear(), now.getMonth(), now.getDate(), now.getHours(), now.getMinutes(), now.getSeconds());
    const mTime = dateM.getUTCHours() * 3600 + dateM.getUTCMinutes() * 60 + dateM.getUTCSeconds();
    const mcTime = (mTime * 10) % 24000; 
    const mcH = Math.floor(mcTime / 1000) % 24, mcM = Math.floor((mcTime % 1000) / (1000/60));
    const mcTimeEl = $('mc-time');
    if (mcTimeEl) mcTimeEl.textContent = `${String(mcH).padStart(2, '0')}:${String(mcM).padStart(2, '0')}:--`;

    const sData = calcSolar(latA, lonA);
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

    const D_rad = calcLunarPhase();
    const phasePerc = (1 + Math.cos(D_rad)) / 2 * 100; 
    const lunarPhasePercEl = $('lunar-phase-perc');
    if (lunarPhasePercEl) lunarPhasePercEl.textContent = `${phasePerc.toFixed(1)}%`;
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

// ===========================================
// GESTION FRÉQUENCE (V3.1)
// ===========================================

function setGPSMode(newMode) {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    currentGPSMode = newMode;
    
    const opts = GPS_OPTS[newMode];
    wID = navigator.geolocation.watchPosition(updateDisp, handleErr, opts);
    
    const newDOMFreq = newMode === 'HIGH_FREQ' ? DOM_HIGH_FREQ_MS : DOM_LOW_FREQ_MS;
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

function checkGPSFrequency(currentSpeed) {
    if (manualFreqMode) {
        let newMode = forcedFreqState;
        if (newMode !== currentGPSMode) setGPSMode(newMode);
        return;
    }
    
    const isMovingFast = currentSpeed >= SPEED_THRESHOLD;
    let newMode = isMovingFast ? 'HIGH_FREQ' : 'LOW_FREQ';

    if (newMode !== currentGPSMode) {
        setGPSMode(newMode);
    }
}

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

// ===========================================
// BOUCLES ET MISE À JOUR DOM (V3.0/V3.1)
// ===========================================

function fastDOM() {
    const latA = lat ?? D_LAT, lonA = lon ?? D_LON;
    const now = getCDate().getTime(); 
    const pNow = performance.now();
    
    const updateFrequencyEl = $('update-frequency');
    if (lDomT && updateFrequencyEl) updateFrequencyEl.textContent = `${(1000 / (pNow - lDomT)).toFixed(1)} Hz (DOM)`;
    lDomT = pNow;

    updateAstro(latA, lonA); 
    
    if (fastDOM.lastSlowT && (pNow - fastDOM.lastSlowT) < DOM_SLOW_UPDATE_MS) return;
    
    fastDOM.lastSlowT = pNow;
    // Dans la fonction fastDOM() (BLOC 2/2) :

function fastDOM() {
    // ... (Début de la fonction fastDOM() inchangé)
    
    updateAstro(latA, lonA); 
    
    if (fastDOM.lastSlowT && (pNow - fastDOM.lastSlowT) < DOM_SLOW_UPDATE_MS) return;
    
    // --- Mise à jour lente (1 Hz) ---
    fastDOM.lastSlowT = pNow;

    updateDM(latA, lonA); 
    
    // NOUVEAU: Calcul et affichage des métriques de midi UTC
    const middayData = calcMiddayMetrics();
    
    const solarLongitudeMiddayEl = $('solar-longitude-midday');
    if (solarLongitudeMiddayEl) solarLongitudeMiddayEl.textContent = `${middayData.solarLongitude.toFixed(8)} °`;

    const eotMiddayEl = $('eot-midday');
    if (eotMiddayEl) eotMiddayEl.textContent = `${middayData.eot.toFixed(4)} min`;
    
    // ... (Reste de la fonction fastDOM() inchangé)
        }

    updateDM(latA, lonA); 
    
    if (!lPos || sTime === null) { return; }
    
    const spd3D = lPos.speedMS_3D || 0, spd3DKMH = spd3D * KMH_MS;
    const sSpd = kSpd < MIN_SPD ? 0 : kSpd, sSpdKMH = sSpd * KMH_MS;
    const elapS = (now - sTime) / 1000;
    
    // Affichage des vitesses
    if ($('speed-3d-inst')) $('speed-3d-inst').textContent = `${spd3DKMH.toFixed(5)} km/h`; 
    if ($('speed-ms')) $('speed-ms').textContent = `${spd3D.toFixed(5)} m/s`; 
    if ($('perc-light')) $('perc-light').textContent = `${(spd3D / C_L * 100).toPrecision(5)}%`;
    if ($('perc-sound')) $('perc-sound').textContent = `${(spd3D / C_S * 100).toPrecision(5)}%`;
    if ($('speed-stable')) $('speed-stable').textContent = `${sSpdKMH.toFixed(5)} km/h`; 
    if ($('speed-stable-mm')) $('speed-stable-mm').textContent = `${(sSpd * 1000).toFixed(2)} mm/s`;
    if ($('elapsed-time')) $('elapsed-time').textContent = `${elapS.toFixed(2)} s`;

    // Affichage Cohérence Kalman
    const kR = lPos.kalman_R_val || R_MAX; 
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

// ===========================================
// TRAITEMENT GPS / GNSS / CORRECTIONS (V3.1)
// ===========================================

function updateDisp(pos) {
    lat = pos.coords.latitude; lon = pos.coords.longitude;
    const alt = pos.coords.altitude, acc = pos.coords.accuracy;
    const spd = pos.coords.speed, cTime = pos.timestamp; 
    let hdg = pos.coords.heading;

    syncH(); 

    if (sTime === null) { 
        sTime = getCDate().getTime();
        distMStartOffset = distM; 
    }

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
    
    const spd3D = Math.sqrt(spdH ** 2 + spdV ** 2);

    // --- CORRECTION GNSS AVANCÉE ---
    const ztd_m = getTroposphericDelay(lastP_hPa, lastT_K, lastH_perc, alt ?? 0, lat);
    const R_dyn = getKalmanR(acc, alt, ztd_m);
    
    const fSpd = kFilter(spd3D, dt, R_dyn), sSpdFE = fSpd < MIN_SPD ? 0 : fSpd;
    
    let hdg_corr = hdg;
    if (sSpdFE > SPEED_THRESHOLD && hdg !== null) {
        lastReliableHeading = hdg; 
    } else {
        hdg_corr = lastReliableHeading; 
    }
    
    // --- MISE À JOUR DES ÉTATS ET MÉTRIQUES ---
    
    const lastSpd3D = lPos ? (lPos.speedMS_3D_LAST ?? 0) : 0;
    const accellLong = dt > 0 ? (spd3D - lastSpd3D) / dt : 0;
    const gForce = accellLong / G_ACCEL; 
    const dragForce = 0.5 * AIR_DENSITY * CDA_EST * sSpdFE ** 2;
    const dragPower = dragForce * sSpdFE; 
    
    lPos = pos; lPos.speedMS_3D = spd3D; lPos.timestamp = cTime; 
    lPos.speedMS_3D_LAST = spd3D; 
    lPos.kalman_R_val = R_dyn; 
    lPos.kalman_kSpd_LAST = fSpd; 

    checkGPSFrequency(sSpdFE); 

    distM += sSpdFE * dt * (netherMode ? NETHER_RATIO : 1);
    
    const elapS = (getCDate().getTime() - sTime) / 1000;
    const sessionDistM = distM - distMStartOffset;
    const spdAvg = elapS > 0 ? sessionDistM / elapS : 0; 
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    
    // --- AFFICHAGE BASIQUE (Le reste est fait dans fastDOM) ---
    let pText = `${acc.toFixed(2)} m`;
    pText += R_dyn <= R_MIN * 1.5 ? ' (Optimal/Corrigé)' : ' (Ajusté)';

    if ($('gps-accuracy')) $('gps-accuracy').textContent = pText;
    if ($('speed-avg')) $('speed-avg').textContent = `${(spdAvg * KMH_MS).toFixed(5)} km/h`; 
    if ($('speed-max')) $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    if ($('latitude')) $('latitude').textContent = `${lat.toFixed(6)}`;
    if ($('longitude')) $('longitude').textContent = `${lon.toFixed(6)}`;
    if ($('altitude')) $('altitude').textContent = `${alt !== null ? alt.toFixed(2) : '--'} m`;
    if ($('underground')) $('underground').textContent = alt !== null && alt < ALT_TH ? 'Oui' : 'Non';
    if ($('heading')) $('heading').textContent = hdg_corr !== null ? `${hdg_corr.toFixed(1)} °` : '--';
    if ($('distance-km-m')) $('distance-km-m').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    if ($('vertical-speed')) $('vertical-speed').textContent = `${spdV.toFixed(2)} m/s`;
    if ($('drag-force')) $('drag-force').textContent = `${dragForce.toFixed(1)} N`;
    if ($('drag-power-kw')) $('drag-power-kw').textContent = `${(dragPower / 1000).toFixed(2)} kW`;
    if ($('g-force')) $('g-force').textContent = `${gForce.toFixed(2)} G`;
    if ($('accel-long')) $('accel-long').textContent = `${accellLong.toFixed(3)} m/s²`;
    
    // Taux de rafraîchissement des données météo
    if (Date.now() - (updateDisp.lastWeatherFetch ?? 0) > 10000) {
        fetchWeather(lat, lon);
        updateDisp.lastWeatherFetch = Date.now();
    }
}

// ===========================================
// GESTION DU CYCLE DE VIE ET ÉVÉNEMENTS (V3.0/V3.1)
// ===========================================

function handleErr(err) {
    console.error(`Erreur GNSS (${err.code}): ${err.message}`);
    const errEl = $('error-message');
    errEl.style.display = 'block';
    errEl.textContent = `❌ Erreur GNSS (${err.code}): Signal perdu. Bascule vers la prédiction/dernière position.`;
    stopGPS(false);
}

function startGPS() {
    if (wID !== null) stopGPS(false);
    sTime = null; 
    setGPSMode('LOW_FREQ'); 
    
    if ($('start-btn')) $('start-btn').disabled = true;
    if ($('stop-btn')) $('stop-btn').disabled = false;
    if ($('error-message')) $('error-message').style.display = 'none';
}

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

function resetDisp() {
    // Réinitialisation de toutes les métriques de session
    stopGPS(false);
    distM = 0; distMStartOffset = 0; maxSpd = 0; 
    kSpd = 0; kUncert = 1000;
    tLat = null; tLon = null;
    lastReliableHeading = null;
    sTime = null;
    lPos = null; 
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
    
    html2canvas(document.body).then(canvas => {
        if (controls) controls.style.display = 'block'; 
        const link = document.createElement('a');
        link.href = canvas.toDataURL('image/png');
        link.download = 'dashboard_capture.png';
        link.click();
    });
}

document.addEventListener('DOMContentLoaded', () => {
    // Références DOM pour les boutons (omises pour la concision)
    const resetAllBtn = $('reset-all-btn'), startBtn = $('start-btn'), stopBtn = $('stop-btn');
    const resetMaxBtn = $('reset-max-btn'), setTargetBtn = $('set-target-btn'), toggleModeBtn = $('toggle-mode-btn');
    const autoModeBtn = $('auto-mode-btn'), netherToggleBtn = $('nether-toggle-btn'), freqManualBtn = $('freq-manual-btn');
    const captureBtn = $('capture-btn');
    const setDefaultLocBtn = $('set-default-loc-btn');

    resetDisp();
    syncH(); 
    initALS(); 
    initAdvancedSensors(); 
    fetchWeather(D_LAT, D_LON); 
    
    if (domID === null) {
        domID = setInterval(fastDOM, DOM_LOW_FREQ_MS); 
        fastDOM.lastSlowT = 0; 
        currentDOMFreq = DOM_LOW_FREQ_MS;
    }
    
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
});

// FIN DU BLOC 2/2
// =================================================================
