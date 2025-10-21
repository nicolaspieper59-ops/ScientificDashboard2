// =================================================================
// FICHIER COMPLET CONDENSÉ : dashboard.js
// (Correction pour Empêcher le Gel de l'Heure/Astro en Pause GPS)
// =================================================================

// --- CONSTANTES GLOBALES ET INITIALISATION (Inchangées) ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458, C_S = 343, R_E = 6371000, KMH_MS = 3.6;
const OBLIQ = 23.44 * D2R, ECC = 0.0167, JD_2K = 2451545.0;
const D_LAT = 48.8566, D_LON = 2.3522; 
const W_OPTS = { enableHighAccuracy: true, maximumAge: 0, timeout: 20000 };
const DOM_MS = 17, MIN_DT = 1, MAX_ACC = 50, MIN_SPD = 0.001, ALT_TH = -50;
const Q_NOISE = 0.005, R_MIN = 0.005, R_MAX = 5.0, L_PREC_TH = 60;
const SUN_NIGHT_TH = -12; 
const LUX_NIGHT_TH = 50; 

let wID = null, domID = null, lPos = null, lat = null, lon = null, sTime = null;
let distM = 0, maxSpd = 0, tLat = null, tLon = null, lDomT = null;
let kSpd = 0, kUncert = 1000;
let lServH = null, lLocH = null;

let als = null; 
let lastLux = null; 
let manualMode = null; 

const $ = id => document.getElementById(id);
const startBtn = $('start-btn'), stopBtn = $('stop-btn'), resetMaxBtn = $('reset-max-btn');
const errorDisplay = $('error-message'), speedSrc = $('speed-source-indicator'); 
const setTargetBtn = $('set-target-btn'), modeInd = $('mode-indicator');
const toggleModeBtn = $('toggle-mode-btn'), autoModeBtn = $('auto-mode-btn');

// --- FONCTIONS UTILS (Inchangées) ---

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
    let estT = Date.now();
    if (lServH !== null) {
        estT = lServH + (Date.now() - lLocH);
    } 
    return new Date(estT);
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
    
    return { eot: EoT_m, solarLongitude: sLon, elevation: h * R2D };
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
    const h = Math.floor(LMT_sec / 3600);
    const m = Math.floor((LMT_sec % 3600) / 60);
    const s = Math.floor(LMT_sec % 60);
    $('lunar-time').textContent = `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
}

function initALS() { /* ... (Inchangée) ... */ }
function toggleManualMode() { /* ... (Inchangée) ... */ }
function setAutoMode() { /* ... (Inchangée) ... */ }
function updateDM(lat, lon) { /* ... (Inchangée) ... */ }
function setTarget() { /* ... (Inchangée) ... */ }
function updateAstro(lat, lon) { /* ... (Inchangée) ... */ }
function resetDisp() { /* ... (Inchangée) ... */ }
function resetMax() { maxSpd = 0; $('speed-max').textContent = '0.00000 km/h'; }

function fastDOM() {
    const latA = lat ?? D_LAT, lonA = lon ?? D_LON;
    updateAstro(latA, lonA); updateDM(latA, lonA); 
    if (!lPos || sTime === null) return;
    
    const spd3D = lPos.speedMS_3D || 0, spd3DKMH = spd3D * KMH_MS;
    const sSpd = kSpd < MIN_SPD ? 0 : kSpd, sSpdKMH = sSpd * KMH_MS;

    $('speed-3d-inst').textContent = `${spd3DKMH.toFixed(5)} km/h`; 
    $('speed-ms').textContent = `${spd3D.toFixed(5)} m/s`; 
    $('perc-light').textContent = `${(spd3D / C_L * 100).toPrecision(5)}%`;
    $('perc-sound').textContent = `${(spd3D / C_S * 100).toPrecision(5)}%`;
    $('speed-stable').textContent = `${sSpdKMH.toFixed(5)} km/h`; 
    $('speed-stable-mm').textContent = `${(sSpd * 1000).toFixed(2)} mm/s`;

    const now = getCDate().getTime(); 
    
    const elapS = (now - sTime) / 1000;
    $('elapsed-time').textContent = `${elapS.toFixed(2)} s`;
    
    const pNow = performance.now();
    if (lDomT) $('update-frequency').textContent = `${(1000 / (pNow - lDomT)).toFixed(1)} Hz (DOM)`;
    lDomT = pNow;
}

function updateDisp(pos) { /* ... (Inchangée) ... */ }
function handleErr(err) { /* ... (Inchangée) ... */ }

function startGPS() {
    if (navigator.geolocation) {
        syncH(); 
        sTime = null; 
        resetMax(); distM = 0;
        
        wID = navigator.geolocation.watchPosition(updateDisp, handleErr, W_OPTS);
        
        // CORRECTION: Assure que le DOM tourne, même s'il ne devrait jamais être null ici.
        if (domID === null) domID = setInterval(fastDOM, DOM_MS); 

        startBtn.disabled = true; stopBtn.disabled = false; resetMaxBtn.disabled = false;
        $('gps-accuracy').classList.remove('max-precision');
    } else {
        errorDisplay.textContent = "❌ Géolocalisation non supportée par votre navigateur.";
        errorDisplay.style.display = 'block';
    }
}

// 🔥 CORRECTION CLÉ: On retire l'arrêt de domID pour que l'heure/astro continuent
function stopGPS(clearT = true) {
    // Stoppe uniquement la surveillance GPS
    if (wID !== null) { navigator.geolocation.clearWatch(wID); wID = null; }
    
    // L'intervalle DOM (domID) continue de tourner
    
    if (clearT) sTime = null;
    
    startBtn.disabled = false; stopBtn.disabled = true; 
    errorDisplay.style.display = 'block';
    errorDisplay.textContent = "PAUSE : Géolocalisation arrêtée. Heure UTC basée sur la dernière synchro Internet/Locale.";
}

document.addEventListener('DOMContentLoaded', () => {
    resetDisp();
    syncH(); 
    initALS(); 
    
    // Initialisation UNIVERSELLE de l'intervalle DOM. Il tournera en continu.
    if (domID === null) domID = setInterval(fastDOM, DOM_MS); 
    
    startBtn.addEventListener('click', startGPS);
    stopBtn.addEventListener('click', stopGPS);
    resetMaxBtn.addEventListener('click', resetMax);
    setTargetBtn.addEventListener('click', setTarget);
    toggleModeBtn.addEventListener('click', toggleManualMode);
    autoModeBtn.addEventListener('click', setAutoMode);
});
