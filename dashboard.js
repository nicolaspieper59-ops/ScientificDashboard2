// =================================================================
// FICHIER COMPLET ET STABLE : dashboard.js (V3.0)
// =================================================================

// --- CONSTANTES GLOBALES ET INITIALISATION ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458, C_S = 343, R_E = 6371000, KMH_MS = 3.6;
const OBLIQ = 23.44 * D2R, ECC = 0.0167, JD_2K = 2451545.0; 
const D_LAT = 48.8566, D_LON = 2.3522; 

// CONFIGURATIONS GPS POUR L'OPTIMISATION BATTERIE
const GPS_OPTS = {
    // Mode Haute Fréquence (HIGH_FREQ) : Mouvement rapide
    HIGH_FREQ: {
        enableHighAccuracy: true,
        maximumAge: 0,
        timeout: 10000 // Timeout plus court, on attend une réponse rapide
    },
    // Mode Économie (LOW_FREQ) : À l'arrêt ou très lent
    LOW_FREQ: {
        enableHighAccuracy: false, 
        maximumAge: 120000,         // Accepte position en cache jusqu'à 2 minutes (120s)
        timeout: 120000             // Long timeout (120s)
    }
};

// Fréquences pour la boucle principale fastDOM (sollicite le CPU)
const DOM_HIGH_FREQ_MS = 17;   // ~60 Hz (Pour un affichage très fluide en mouvement)
const DOM_LOW_FREQ_MS = 250;   // 4 Hz (Pour les mises à jour de l'heure et des secondes à l'arrêt)
const DOM_SLOW_UPDATE_MS = 1000; // 1 Hz (Pour les données non critiques)

const SPEED_THRESHOLD = 0.5; // Seuil de vitesse (m/s) pour basculer entre les modes
const MIN_DT = 1, MAX_ACC = 50, MIN_SPD = 0.001, ALT_TH = -50;
const Q_NOISE = 0.005, R_MIN = 0.005, R_MAX = 5.0, L_PREC_TH = 60;
const SUN_NIGHT_TH = -12; 
const LUX_NIGHT_TH = 50; 
const NETHER_RATIO = 8; 

let wID = null, domID = null, lPos = null, lat = null, lon = null, sTime = null;
let distM = 0, distMStartOffset = 0, maxSpd = 0, tLat = null, tLon = null, lDomT = null;
let kSpd = 0, kUncert = 1000; 
let lServH = null, lLocH = null; 
let als = null; 
let lastLux = null; 
let manualMode = null; 
let netherMode = false; 

// ÉTATS DE LA FRÉQUENCE DYNAMIQUE
let currentGPSMode = 'LOW_FREQ'; 
let currentDOMFreq = DOM_LOW_FREQ_MS; // Initialisation en mode éco
let manualFreqMode = false;      
let forcedFreqState = 'HIGH_FREQ'; 

// --- REFERENCES DOM ---
const $ = id => document.getElementById(id);
const startBtn = $('start-btn'), stopBtn = $('stop-btn'), resetMaxBtn = $('reset-max-btn');
const errorDisplay = $('error-message'), speedSrc = $('speed-source-indicator'); 
const setTargetBtn = $('set-target-btn'), modeInd = $('mode-indicator');
const toggleModeBtn = $('toggle-mode-btn'), autoModeBtn = $('auto-mode-btn');
const netherToggleBtn = $('nether-toggle-btn'), resetAllBtn = $('reset-all-btn');
const freqManualBtn = $('freq-manual-btn'); 

// ===========================================
// FONCTIONS GÉO & UTILS
// ===========================================

const dist = (lat1, lon1, lat2, lon2) => {
    // Calcul de la distance de Haversine
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
// CALCULS ASTRO & ENV
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
    $('lunar-time').textContent = `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
}

function initALS() {
    if ('AmbientLightSensor' in window) {
        try {
            als = new AmbientLightSensor({ frequency: 5 }); 
            als.onreading = () => {
                lastLux = als.illuminance;
                $('illuminance-lux').textContent = `${lastLux.toFixed(2)} Lux`;
            };
            als.onerror = (event) => {
                lastLux = null;
                $('illuminance-lux').textContent = 'Erreur/Non Autorisé';
                if (als) als.stop();
            };
            als.start();
        } catch (error) {
            lastLux = null;
            $('illuminance-lux').textContent = 'N/A (Navigateur)';
        }
    } else {
        lastLux = null;
        $('illuminance-lux').textContent = 'N/A (API Manquante)';
    }
}

// ===========================================
// GESTION DU MODE JOUR/NUIT
// ===========================================

function updateAstro(latA, lonA) {
    const now = getCDate();
    const dateM = new Date(now.getFullYear(), now.getMonth(), now.getDate(), now.getHours(), now.getMinutes(), now.getSeconds());
    const mTime = dateM.getUTCHours() * 3600 + dateM.getUTCMinutes() * 60 + dateM.getUTCSeconds();
    const mcTime = (mTime * 10) % 24000; 
    const mcH = Math.floor(mcTime / 1000) % 24, mcM = Math.floor((mcTime % 1000) / (1000/60));
    $('mc-time').textContent = `${String(mcH).padStart(2, '0')}:${String(mcM).padStart(2, '0')}:--`;

    const sData = calcSolar(latA, lonA);
    const hDeg = sData.elevation;
    
    // TEMPS SOLAIRE VRAI (HSV)
    let sTimeH = (now.getUTCHours() * 3600 + now.getUTCMinutes() * 60 + now.getUTCSeconds()) / 3600;
    let trueSolarTimeH = sTimeH + (lonA / 15) + (sData.eot / 60);
    trueSolarTimeH = (trueSolarTimeH % 24 + 24) % 24;
    
    const tsH = Math.floor(trueSolarTimeH), tsM = Math.floor((trueSolarTimeH * 60) % 60), tsS = Math.floor((trueSolarTimeH * 3600) % 60);
    const tsStr = `${String(tsH).padStart(2, '0')}:${String(tsM).padStart(2, '0')}:${String(tsS).padStart(2, '0')}`;
    $('solar-true-header').textContent = tsStr + ' (HSV)';
    $('solar-true').textContent = tsStr;

    // TEMPS SOLAIRE MOYEN (LSM)
    let meanSolarTimeH = sTimeH + (lonA / 15);
    meanSolarTimeH = (meanSolarTimeH % 24 + 24) % 24;
    const msH = Math.floor(meanSolarTimeH), msM = Math.floor((meanSolarTimeH * 60) % 60), msS = Math.floor((meanSolarTimeH * 3600) % 60);
    const msStr = `${String(msH).padStart(2, '0')}:${String(msM).padStart(2, '0')}:${String(msS).padStart(2, '0')}`;
    $('solar-mean-header').textContent = msStr + ' (LSM)';
    $('solar-mean').textContent = msStr;
    
    // ... (affichage EoT, etc.)
    $('eot').textContent = `${sData.eot.toFixed(2)} min`;
    $('sun-elevation').textContent = `${hDeg.toFixed(2)} °`;
    $('solar-longitude-val').textContent = `${sData.solarLongitude.toFixed(2)} °`;
    $('solar-culmination').textContent = sData.culmination;

    // PHASE LUNAIRE
    const D_rad = calcLunarPhase();
    const phasePerc = (1 + Math.cos(D_rad)) / 2 * 100; // Illumination
    $('lunar-phase-perc').textContent = `${phasePerc.toFixed(1)}%`;
    calcLunarTime(lonA);

    // LOGIQUE JOUR/NUIT
    const autoNight = (hDeg < SUN_NIGHT_TH) || (lastLux !== null && lastLux < LUX_NIGHT_TH);
    const isNight = manualMode !== null ? manualMode : autoNight;
    
    document.body.classList.toggle('night-mode', isNight);
    
    let modeText = '';
    if (manualMode !== null) modeText = manualMode ? 'Nuit 🌙 (Manuel)' : 'Jour ☀️ (Manuel)';
    else modeText = isNight ? 'Nuit 🌙 (Auto)' : 'Jour ☀️ (Auto)';
    modeInd.textContent = modeText;
}

function toggleManualMode() {
    if (manualMode === null) { 
        // Passer en mode manuel (commence par l'état actuel)
        const isNight = document.body.classList.contains('night-mode');
        manualMode = isNight;
        toggleModeBtn.textContent = '🔄 Basculer (Jour)'; 
        autoModeBtn.style.display = 'inline-block';
    } else {
        // Basculer l'état manuel
        manualMode = !manualMode;
        toggleModeBtn.textContent = manualMode ? '🔄 Basculer (Jour)' : '🔄 Basculer (Nuit)';
    }
}

function setAutoMode() {
    manualMode = null;
    toggleModeBtn.textContent = '🌗 Bascule Manuelle';
    autoModeBtn.style.display = 'none';
    // Re-calculer immédiatement le mode jour/nuit basé sur l'astro/lux
    if (lat !== null && lon !== null) updateAstro(lat, lon); 
}

// ===========================================
// LOGIQUE DE DISTANCE ET CIBLE
// ===========================================

function updateDM(latA, lonA) {
    if (tLat !== null && tLon !== null) {
        const d = dist(latA, lonA, tLat, tLon);
        $('cap-dest').textContent = `${d < 1000 ? d.toFixed(1) + ' m' : (d / 1000).toFixed(3) + ' km'}`;
    } else {
        $('cap-dest').textContent = 'N/A';
    }
}

function setTarget() {
    if (tLat === null) {
        const latStr = prompt("Entrez la Latitude Cible:", lat ?? D_LAT);
        if (latStr === null || isNaN(parseFloat(latStr))) return;
        const lonStr = prompt("Entrez la Longitude Cible:", lon ?? D_LON);
        if (lonStr === null || isNaN(parseFloat(lonStr))) return;
        tLat = parseFloat(latStr);
        tLon = parseFloat(lonStr);
        setTargetBtn.textContent = '🎯 Cible (Supprimer)';
    } else {
        tLat = null; tLon = null;
        setTargetBtn.textContent = '🗺️ Cible';
    }
    updateDM(lat ?? D_LAT, lon ?? D_LON);
}

// ===========================================
// GESTION FRÉQUENCE MANUELLE ET AUTOMATIQUE
// ===========================================

/** APPLIQUE LES NOUVELLES FRÉQUENCES GPS ET DOM */
function forceNewFrequency(mode) {
    // Mode doit être 'HIGH_FREQ' ou 'LOW_FREQ'
    const newGPSMode = mode;
    const newDOMFreq = mode === 'HIGH_FREQ' ? DOM_HIGH_FREQ_MS : DOM_LOW_FREQ_MS;

    // 1. GESTION DU GPS
    if (newGPSMode !== currentGPSMode && wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;

        const opts = GPS_OPTS[newGPSMode];
        wID = navigator.geolocation.watchPosition(updateDisp, handleErr, opts);
        currentGPSMode = newGPSMode;
        $('speed-source-indicator').textContent = `Source: ${newGPSMode} (Forcé)`;
    }

    // 2. GESTION DE LA FRÉQUENCE DOM/CPU
    if (newDOMFreq !== currentDOMFreq) {
        if (domID !== null) { clearInterval(domID); domID = null; }
        domID = setInterval(fastDOM, newDOMFreq);
        currentDOMFreq = newDOMFreq;
    }
}

/** FORCE LE SYSTÈME EN MODE ÉCONOMIE MAXIMAL (GPS + DOM) */
function forceEcoMode() {
    // 1. GPS : Forcer LOW_FREQ (Timeout 120s, Age 120s)
    if (currentGPSMode !== 'LOW_FREQ' && wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
        
        const opts = GPS_OPTS['LOW_FREQ'];
        wID = navigator.geolocation.watchPosition(updateDisp, handleErr, opts);
        currentGPSMode = 'LOW_FREQ';
        $('speed-source-indicator').textContent = `Source: LOW_FREQ (Forcé)`;
    }

    // 2. DOM/CPU : Forcer la fréquence la plus basse (250ms)
    if (currentDOMFreq !== DOM_LOW_FREQ_MS) {
        if (domID !== null) { clearInterval(domID); domID = null; }
        domID = setInterval(fastDOM, DOM_LOW_FREQ_MS);
        currentDOMFreq = DOM_LOW_FREQ_MS;
    }
}

/** FONCTION D'OPTIMISATION DE LA FRÉQUENCE GPS ET DOM (Automatique) */
function checkGPSFrequency(currentSpeed) {
    
    // Si le mode manuel est actif, ignorer l'ajustement par la vitesse
    if (manualFreqMode) return; 
    
    const isMovingFast = currentSpeed >= SPEED_THRESHOLD;
    
    // 1. GESTION DU MODE GPS
    let newGPSMode = isMovingFast ? 'HIGH_FREQ' : 'LOW_FREQ';
    if (newGPSMode !== currentGPSMode) {
        // console.log(`Bascule GPS: ${currentGPSMode} -> ${newGPSMode}.`);
        forceNewFrequency(newGPSMode);
        $('speed-source-indicator').textContent = `Source: ${newGPSMode} (Auto)`;
    }

    // 2. GESTION DE LA FRÉQUENCE DOM/CPU
    let newDOMFreq = isMovingFast ? DOM_HIGH_FREQ_MS : DOM_LOW_FREQ_MS;
    if (newDOMFreq !== currentDOMFreq) {
        // console.log(`Bascule DOM/CPU: ${currentDOMFreq}ms -> ${newDOMFreq}ms.`);
        if (domID !== null) { clearInterval(domID); domID = null; }
        domID = setInterval(fastDOM, newDOMFreq);
        currentDOMFreq = newDOMFreq;
    }
}

/** TOGGLE MANUEL DE LA FRÉQUENCE (MAX <-> MIN) */
function toggleManualFreq() {
    manualFreqMode = !manualFreqMode;
    const btn = freqManualBtn;

    if (manualFreqMode) {
        forcedFreqState = 'HIGH_FREQ'; 
        forceNewFrequency('HIGH_FREQ');
        btn.style.backgroundColor = '#007bff';
        btn.textContent = '⚡ MANUEL: MAX';
        
    } else {
        checkGPSFrequency(kSpd); 
        btn.style.backgroundColor = '#ff4500';
        btn.textContent = '⚡ Fréquence: Auto';
    }
}

/** BASULE L'ÉTAT LORSQUE LE MODE MANUEL EST ACTIF */
function cycleForcedFreq() {
    if (!manualFreqMode) return; 

    if (forcedFreqState === 'HIGH_FREQ') {
        forcedFreqState = 'LOW_FREQ';
        forceNewFrequency('LOW_FREQ');
        freqManualBtn.textContent = '⚡ MANUEL: MIN';
        freqManualBtn.style.backgroundColor = '#4CAF50';
    } else {
        forcedFreqState = 'HIGH_FREQ';
        forceNewFrequency('HIGH_FREQ');
        freqManualBtn.textContent = '⚡ MANUEL: MAX';
        freqManualBtn.style.backgroundColor = '#007bff';
    }
}

// ===========================================
// CAPTURE D'ÉCRAN
// ===========================================

function captureScreenshot() {
    const captureTarget = document.body;
    const captureBtn = $('capture-btn');
    captureBtn.disabled = true;
    captureBtn.textContent = 'Capture en cours...';

    // 1. FORCER LE MODE ÉCO AVANT L'OPÉRATION COÛTEUSE
    const originalSpeed = kSpd; 
    forceEcoMode(); 

    html2canvas(captureTarget, {
        scale: 2, 
        useCORS: true, 
        logging: false,
        ignoreElements: (element) => element.tagName === 'BUTTON' || element.tagName === 'INPUT' 
    }).then(canvas => {
        // Logique de téléchargement
        const imageURL = canvas.toDataURL('image/png');
        const a = document.createElement('a');
        const now = new Date();
        const timestamp = now.toISOString().replace(/[:.]/g, '-');
        a.download = `dashboard_capture_${timestamp}.png`;
        a.href = imageURL;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);

        captureBtn.textContent = '📸 Capturer';

        // 2. RÉTABLIR LE MODE AUTOMATIQUE APRÈS DÉLAI
        setTimeout(() => {
            checkGPSFrequency(originalSpeed); // Utilise la vitesse sauvegardée
            captureBtn.disabled = false;
        }, 2000); 

    }).catch(error => {
        console.error("Erreur lors de la capture:", error);
        alert("Une erreur est survenue lors de la capture d'écran.");
        
        checkGPSFrequency(kSpd); 
        captureBtn.disabled = false;
        captureBtn.textContent = '📸 Capturer';
    });
}

// -------------------------------------------------------------------
// DÉBUT DU MORCEAU DE FICHIER DEMANDÉ (Suite de resetDisp et fonctions suivantes)
// -------------------------------------------------------------------

function resetDisp() {
    lPos = null; lat = null; lon = null; distM = 0; distMStartOffset = 0; sTime = null; maxSpd = 0; tLat = null; tLon = null;
    kSpd = 0; kUncert = 1000; lDomT = null; lServH = null; lLocH = null; lastLux = null;
    manualMode = null; netherMode = false;

    // Réinitialisation des états de fréquence au défaut (LOW_FREQ, Auto)
    currentGPSMode = 'LOW_FREQ'; 
    currentDOMFreq = DOM_LOW_FREQ_MS; 
    manualFreqMode = false;      
    forcedFreqState = 'HIGH_FREQ'; 
    freqManualBtn.textContent = '⚡ Fréquence: Auto';
    freqManualBtn.style.backgroundColor = '#ff4500';

    const defT = '--', ids = ['elapsed-time', 'speed-3d-inst', 'speed-stable', 'speed-stable-mm', 'speed-avg', 'speed-max', 'speed-ms', 'perc-light', 'perc-sound', 'distance-km-m', 'lunar-time', 'latitude', 'longitude', 'altitude', 'gps-accuracy', 'underground', 'solar-true', 'solar-mean', 'eot', 'solar-longitude-val', 'lunar-phase-perc', 'mc-time', 'air-temp', 'pressure', 'humidity', 'wind-speed', 'boiling-point', 'heading', 'bubble-level', 'cap-dest', 'solar-true-header', 'mode-indicator', 'speed-source-indicator', 'speed-error-perc', 'update-frequency', 'sun-elevation', 'illuminance-lux', 'mag-field', 'accel-long', 'g-force', 'drag-force', 'drag-power-kw', 'alt-baro', 'o2-perc', 'time-shift-rate', 'speed-coherence', 'vertical-speed', 'solar-culmination'];

    ids.forEach(id => {
        const el = $(id);
        if (el) {
            if (id === 'speed-source-indicator') el.textContent = 'Source: N/A';
            else if (['air-temp', 'pressure', 'humidity', 'wind-speed', 'boiling-point', 'bubble-level'].includes(id)) el.textContent = 'N/A (API désactivée)'; 
            else if (id === 'altitude' || id === 'gps-accuracy' || id === 'alt-baro') el.textContent = '-- m';
            else if (id === 'distance-km-m') el.textContent = '-- km | -- m';
            else if (id === 'mode-indicator') el.textContent = 'Mode: Jour ☀️';
            else if (id === 'speed-error-perc' || id === 'update-frequency' || id === 'time-shift-rate') el.textContent = '--';
            else if (id === 'sun-elevation') el.textContent = '-- °';
            else if (id === 'illuminance-lux') el.textContent = 'Initialisation...';
            else if (['mc-time', 'lunar-time', 'solar-mean', 'solar-true', 'solar-true-header', 'solar-culmination'].includes(id)) el.textContent = '00:00:00';
            else if (id === 'mag-field') el.textContent = '-- \mu\text{T}';
            else if (id === 'accel-long') el.textContent = '0.000 m/s²';
            else if (id === 'g-force') el.textContent = '0.00 G';
            else if (id === 'drag-force') el.textContent = '0.0 N';
            else if (id === 'drag-power-kw') el.textContent = '0.00 kW';
            else if (id === 'o2-perc') el.textContent = '20.95 %';
            else if (id === 'nether-indicator') el.textContent = 'DÉSACTIVÉ (1:1)';
            else if (id === 'vertical-speed') el.textContent = '0.00 m/s';
            else el.textContent = defT;
        }
    });

    startBtn.disabled = false; stopBtn.disabled = true; resetMaxBtn.disabled = true; setTargetBtn.textContent = '🗺️ Cible';
    toggleModeBtn.textContent = '🌗 Bascule Manuelle';
    autoModeBtn.style.display = 'none';
    netherToggleBtn.textContent = '🔥 Nether';

    errorDisplay.style.display = 'none';
    document.body.classList.remove('night-mode'); $('gps-accuracy').classList.remove('max-precision');
}

// -------------------------------------------------------------------
// FIN DE resetDisp()
// -------------------------------------------------------------------

function resetMax() { maxSpd = 0; $('speed-max').textContent = '0.00000 km/h'; }

function fastDOM() {
    const latA = lat ?? D_LAT, lonA = lon ?? D_LON;
    const now = getCDate().getTime(); 
    const pNow = performance.now();
    
    // Fréquence d'affichage (Mise à jour à la fréquence de DOM_LOW/HIGH_FREQ_MS)
    if (lDomT) $('update-frequency').textContent = `${(1000 / (pNow - lDomT)).toFixed(1)} Hz (DOM)`;
    lDomT = pNow;

    // --- Mise à jour ultra-rapide (Gérée par la fréquence actuelle, pour l'heure) ---
    updateAstro(latA, lonA); 
    
    // Si la dernière mise à jour lente est trop récente, on s'arrête là (économie CPU/DOM)
    if (fastDOM.lastSlowT && (pNow - fastDOM.lastSlowT) < DOM_SLOW_UPDATE_MS) return;
    
    // --- Mise à jour lente (1 Hz) ---
    fastDOM.lastSlowT = pNow;

    updateDM(latA, lonA); 
    
    if (!lPos || sTime === null) {
         return; 
    }
    
    const spd3D = lPos.speedMS_3D || 0, spd3DKMH = spd3D * KMH_MS;
    const sSpd = kSpd < MIN_SPD ? 0 : kSpd, sSpdKMH = sSpd * KMH_MS;

    $('speed-3d-inst').textContent = `${spd3DKMH.toFixed(5)} km/h`; 
    $('speed-ms').textContent = `${spd3D.toFixed(5)} m/s`; 
    $('perc-light').textContent = `${(spd3D / C_L * 100).toPrecision(5)}%`;
    $('perc-sound').textContent = `${(spd3D / C_S * 100).toPrecision(5)}%`;
    $('speed-stable').textContent = `${sSpdKMH.toFixed(5)} km/h`; 
    $('speed-stable-mm').textContent = `${(sSpd * 1000).toFixed(2)} mm/s`;

    const elapS = (now - sTime) / 1000;
    $('elapsed-time').textContent = `${elapS.toFixed(2)} s`;
}

function updateDisp(pos) {
    lat = pos.coords.latitude; lon = pos.coords.longitude;
    const alt = pos.coords.altitude, acc = pos.coords.accuracy, hdg = pos.coords.heading;   
    const spd = pos.coords.speed, cTime = pos.timestamp; 
    
    syncH(); 

    if (sTime === null) { 
        sTime = getCDate().getTime();
        distMStartOffset = distM; 
    }

    if (acc > MAX_ACC) { $('gps-accuracy').textContent = `❌ ${acc.toFixed(0)} m (Trop Imprécis)`; if (lPos === null) lPos = pos; return; }
    
    let spdH = spd ?? 0, spdSrc = spd !== null && spd !== undefined ? 'Puce GPS (Doppler)' : 'Calculée (Dérivée)';
    let spdV = 0;
    
    const dt = lPos ? (cTime - lPos.timestamp) / 1000 : MIN_DT;

    if (lPos && dt > 0.1) { 
        const dH = dist(lPos.coords.latitude, lPos.coords.longitude, lat, lon);
        if (spd === null || spd === undefined) spdH = dH / dt; 
        if (alt !== null && lPos.coords.altitude !== null) spdV = (alt - lPos.coords.altitude) / dt; 
    }
    
    const spd3D = Math.sqrt(spdH ** 2 + spdV ** 2);
    lPos = pos; lPos.speedMS_3D = spd3D; lPos.timestamp = cTime; 

    let kR, pText = `${acc.toFixed(2)} m`, accEl = $('gps-accuracy');

    if (acc <= 1.0) {
        kR = R_MIN; pText += ' (Optimal)'; accEl.classList.add('max-precision');
    } else if (acc > L_PREC_TH) {
        kR = R_MAX; pText += ' (Très Faible)'; accEl.classList.remove('max-precision');
    } else {
        const normAcc = (acc - 1.0) / (L_PREC_TH - 1.0);
        kR = R_MIN + (R_MAX - R_MIN) * Math.pow(normAcc, 2);
        pText += ' (Progressif)'; accEl.classList.remove('max-precision');
    }
    kR = Math.max(R_MIN, Math.min(R_MAX, kR));

    const fSpd = kFilter(spd3D, dt, kR), sSpdFE = fSpd < MIN_SPD ? 0 : fSpd;
    
    // NOUVEL APPEL : Vérifier la fréquence GPS en fonction de la vitesse stable
    checkGPSFrequency(sSpdFE); 

    distM += sSpdFE * dt * (netherMode ? NETHER_RATIO : 1);
    
    const elapS = (getCDate().getTime() - sTime) / 1000;
    const sessionDistM = distM - distMStartOffset;
    const spdAvg = elapS > 0 ? sessionDistM / elapS : 0; 
    
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    let spdErr = sSpdFE > MIN_SPD ? (Math.abs(spd3D - sSpdFE) / sSpdFE) * 100 : 0;

    $('speed-avg').textContent = `${(spdAvg * KMH_MS).toFixed(5)} km/h`; 
    $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    $('speed-error-perc').textContent = `${spdErr.toFixed(2)}%`; 
    $('latitude').textContent = `${lat.toFixed(6)}`; $('longitude').textContent = `${lon.toFixed(6)}`;
    $('altitude').textContent = `${alt !== null ? alt.toFixed(2) : '--'} m`;
    $('gps-accuracy').textContent = pText;
    $('underground').textContent = alt !== null && alt < ALT_TH ? 'Oui' : 'Non';
    $('heading').textContent = hdg !== null ? `${hdg.toFixed(1)} °` : '--';
    $('distance-km-m').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    speedSrc.textContent = `Source: ${currentGPSMode} (${manualFreqMode ? 'Forcé' : 'Auto'})`;
    $('vertical-speed').textContent = `${spdV.toFixed(2)} m/s`;
    
    if (tLat !== null && tLon !== null) {
        $('cap-dest').textContent = `${bearing(lat, lon, tLat, tLon).toFixed(1)} °`;
    }
    
    // ===============================================
    // DÉBUT : Métriques physiques
    // ===============================================
    
    const AIR_DENSITY = 1.225; 
    const CDA_EST = 0.6;       
    const v_ms = sSpdFE;       
    
    const dragForce = 0.5 * AIR_DENSITY * CDA_EST * v_ms ** 2;
    const dragPower = dragForce * v_ms; 
    
    $('drag-force').textContent = `${dragForce.toFixed(1)} N`;
    $('drag-power-kw').textContent = `${(dragPower / 1000).toFixed(2)} kW`;
    
    const lastSpd3D = lPos.speedMS_3D_LAST ?? 0;
    const accellLong = dt > 0 ? (spd3D - lastSpd3D) / dt : 0;
    lPos.speedMS_3D_LAST = spd3D; 
    
    const gForce = accellLong / 9.80665; 
    
    $('g-force').textContent = `${gForce.toFixed(2)} G`;
    $('accel-long').textContent = `${accellLong.toFixed(3)} m/s²`;
    
    // ===============================================
    // FIN : Métriques physiques
    // ===============================================
}

function handleErr(err) {
    syncH(); 
    errorDisplay.style.display = 'block';
    let msg = "❌ Erreur GPS inconnue. Utilisation du temps Internet/Local.";
    if (err.code === err.PERMISSION_DENIED) msg = "❌ L'accès à la localisation a été refusé. Utilisation du temps Internet/Local.";
    else if (err.code === err.POSITION_UNAVAILABLE) msg = "🛰️ Position non disponible. Signal GPS faible. Utilisation du temps Internet/Local.";
    else if (err.code === err.TIMEOUT) msg = "⏱️ Délai de recherche du GPS dépassé. Signal faible. Utilisation du temps Internet/Local.";
    errorDisplay.textContent = msg;
    stopGPS(false); 
}

function startGPS() {
    if (navigator.geolocation) {
        syncH(); 
        
        distMStartOffset = distM; 
        sTime = getCDate().getTime(); 
        resetMax(); 
        
        // Démarrage initial en mode LOW_FREQ (GPS)
        const opts = GPS_OPTS['LOW_FREQ'];
        wID = navigator.geolocation.watchPosition(updateDisp, handleErr, opts);
        currentGPSMode = 'LOW_FREQ';

        // Démarrage initial en mode faible fréquence (DOM/CPU)
        if (domID !== null) clearInterval(domID);
        domID = setInterval(fastDOM, DOM_LOW_FREQ_MS);
        currentDOMFreq = DOM_LOW_FREQ_MS;
        fastDOM.lastSlowT = 0; 
        
        startBtn.disabled = true; stopBtn.disabled = false; resetMaxBtn.disabled = false;
        $('gps-accuracy').classList.remove('max-precision');
        $('speed-source-indicator').textContent = `Source: LOW_FREQ (Auto)`; 
    } else {
        errorDisplay.textContent = "❌ Géolocalisation non supportée par votre navigateur.";
        errorDisplay.style.display = 'block';
    }
}

function stopGPS(clearT = true) {
    if (wID !== null) { navigator.geolocation.clearWatch(wID); wID = null; }
    
    if (clearT) sTime = null;
    
    // S'assurer que le mode d'affichage est au moins LOW_FREQ
    if (currentDOMFreq !== DOM_LOW_FREQ_MS && domID !== null) {
        clearInterval(domID);
        domID = setInterval(fastDOM, DOM_LOW_FREQ_MS);
        currentDOMFreq = DOM_LOW_FREQ_MS;
    }

    startBtn.disabled = false; stopBtn.disabled = true; 
    errorDisplay.style.display = 'block';
    errorDisplay.textContent = "PAUSE : Géolocalisation arrêtée. La distance totale est conservée.";
}

// --- DÉMARRAGE INITIAL ---
document.addEventListener('DOMContentLoaded', () => {
    resetDisp();
    syncH(); 
    initALS(); 
    
    // Démarrage initial de la boucle DOM en mode éco (LOW_FREQ_MS)
    if (domID === null) {
        domID = setInterval(fastDOM, DOM_LOW_FREQ_MS); 
        fastDOM.lastSlowT = 0; 
        currentDOMFreq = DOM_LOW_FREQ_MS;
    }
    
    const resetAllBtn = $('reset-all-btn');
    startBtn.addEventListener('click', startGPS);
    stopBtn.addEventListener('click', stopGPS);
    resetMaxBtn.addEventListener('click', resetMax);
    
    // Événement RESET TOUT
    resetAllBtn.addEventListener('click', () => { 
        if (confirm("Êtes-vous sûr de vouloir tout réinitialiser (Distance, Max, Cible) ?")) {
            stopGPS(true);
            resetDisp();
        }
    });

    $('set-default-loc-btn').addEventListener('click', () => { 
        const newLatStr = prompt(`Entrez la nouvelle Latitude par défaut (actuel: ${D_LAT}) :`);
        if (newLatStr !== null && !isNaN(parseFloat(newLatStr))) { D_LAT = parseFloat(newLatStr); }
        const newLonStr = prompt(`Entrez la nouvelle Longitude par défaut (actuel: ${D_LON}) :`);
        if (newLonStr !== null && !isNaN(parseFloat(newLonStr))) { D_LON = parseFloat(newLonStr); }
        alert(`Nouvelle position par défaut : Lat=${D_LAT.toFixed(4)}, Lon=${D_LON.toFixed(4)}.`);
        resetDisp();
    });
    setTargetBtn.addEventListener('click', setTarget);
    toggleModeBtn.addEventListener('click', toggleManualMode);
    autoModeBtn.addEventListener('click', setAutoMode);
    
    netherToggleBtn.addEventListener('click', () => {
        netherMode = !netherMode;
        distM = distMStartOffset; 
        maxSpd = 0; 
        $('nether-indicator').textContent = netherMode ? "ACTIVÉ (1:8) 🔥" : "DÉSACTIVÉ (1:1)";
        netherToggleBtn.textContent = netherMode ? "🌍 Overworld" : "🔥 Nether";
    });
    
    // ÉVÉNEMENT FRÉQUENCE MANUELLE
    freqManualBtn.addEventListener('click', () => {
        if (manualFreqMode) {
            cycleForcedFreq(); 
        } else {
            toggleManualFreq(); 
        }
    });
    freqManualBtn.addEventListener('dblclick', toggleManualFreq); 

    // ÉVÉNEMENT CAPTURE
    $('capture-btn').addEventListener('click', captureScreenshot);
});
