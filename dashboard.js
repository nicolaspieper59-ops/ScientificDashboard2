// =================================================================
// FICHIER COMPLET CONDENSÉ : dashboard.js
// (Moins de 533 lignes - Priorité Temps GPS + Kalman + Astro)
// =================================================================

// --- CONSTANTES GLOBALES ET INITIALISATION ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458, C_S = 343, R_E = 6371000, KMH_MS = 3.6;
const OBLIQ = 23.44 * D2R, ECC = 0.0167, JD_2K = 2451545.0;
const D_LAT = 48.8566, D_LON = 2.3522; 
const W_OPTS = { enableHighAccuracy: true, maximumAge: 0, timeout: 20000 };
const DOM_MS = 17, MIN_DT = 1, MAX_ACC = 50, MIN_SPD = 0.001, ALT_TH = -50;
const Q_NOISE = 0.005, R_MIN = 0.005, R_MAX = 5.0, L_PREC_TH = 60;

let wID = null, domID = null, lPos = null, lat = null, lon = null, sTime = null;
let distM = 0, maxSpd = 0, tLat = null, tLon = null, lDomT = null;
let kSpd = 0, kUncert = 1000;
let offH = 0, lServH = null, lLocH = null, lGPST = null;

// --- REFERENCES DOM ---
const $ = id => document.getElementById(id);
const startBtn = $('start-btn'), stopBtn = $('stop-btn'), resetMaxBtn = $('reset-max-btn');
const errorDisplay = $('error-message'), speedSrc = $('speed-source-indicator'); 
const setTargetBtn = $('set-target-btn'), modeInd = $('mode-indicator');

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
// TEMPS & KALMAN
// ===========================================

async function syncH() {
    if (lGPST !== null) return;
    const tz = Intl.DateTimeFormat().resolvedOptions().timeZone;
    try {
        const res = await fetch(`https://worldtimeapi.org/api/timezone/${tz}`);
        if (!res.ok) throw new Error(res.status);
        const data = await res.json();
        lServH = data.unixtime * 1000; 
        lLocH = Date.now(); 
        offH = lServH - lLocH;
    } catch (e) {
        if (lServH === null) { lLocH = Date.now(); offH = 0; }
    }
}

function getCDate() {
    let estT = Date.now();
    if (lGPST !== null && lPos) {
        estT = lGPST + (Date.now() - lPos.timestamp);
    } else if (lServH !== null) {
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
    const alpha = Math.atan2(Math.cos(OBLIQ) * Math.sin(lambda), Math.cos(lambda));
    let EoT_m = (L - alpha) * R2D * 4;
    if (EoT_m > 30) EoT_m -= 360 * 4;
    if (EoT_m < -30) EoT_m += 360 * 4;
    let sLon = (lambda * R2D) % 360;
    if (sLon < 0) sLon += 360;
    return { eot: EoT_m, solarLongitude: sLon };
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

// ===========================================
// LOGIQUE D'AFFICHAGE ET GESTION GPS
// ===========================================

function updateDM(lat, lon) {
    const now = getCDate();
    const hA = (now.getUTCHours() + now.getUTCMinutes() / 60 + lon / 15) % 24;
    const isN = (hA < 6 || hA >= 18);
    document.body.classList.toggle('night-mode', isN);
    modeInd.textContent = `Mode: ${isN ? 'Nuit 🌙' : 'Jour ☀️'}`;
}

function setTarget() {
    if (!lPos) { alert("Attendre une position avant de définir une cible."); return; }
    const cLat = lPos.coords.latitude.toFixed(6), cLon = lPos.coords.longitude.toFixed(6);
    const iLat = prompt(`Lat (actuel: ${cLat}°):`, cLat), iLon = prompt(`Lon (actuel: ${cLon}°):`, cLon);
    const la = parseFloat(iLat), lo = parseFloat(iLon);
    if (!isNaN(la) && !isNaN(lo)) {
        tLat = la; tLon = lo;
        setTargetBtn.textContent = '✔️ Cible définie';
    } else {
        alert("Coordonnées invalides. Réinitialisation.");
        tLat = null; tLon = null;
        $('cap-dest').textContent = 'N/A';
        setTargetBtn.textContent = '🗺️ Aller';
    }
}

function updateAstro(lat, lon) {
    const cLat = lat ?? D_LAT, cLon = lon ?? D_LON, now = getCDate();
    
    // MC Time
    const mcTicksPD = 24000, msSinceM = now.getTime() % 86400000;
    const mcTicks = (msSinceM * mcTicksPD) / 86400000;
    const mcM = Math.floor(mcTicks / 20) % 1440; 
    const h = Math.floor(mcM / 60), m = mcM % 60, s = Math.floor((mcTicks % 20) / 20 * 60);
    $('mc-time').textContent = `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;

    // LSM & HSV
    const sUT = now.getUTCHours() * 3600 + now.getUTCMinutes() * 60 + now.getUTCSeconds();
    const sLSM = (sUT + (cLon * 4 * 60) + 86400) % 86400;
    const hsmH = Math.floor(sLSM / 3600), hsmM = Math.floor((sLSM % 3600) / 60), hsmS = Math.floor(sLSM % 60);
    $('solar-mean').textContent = `${String(hsmH).padStart(2, '0')}:${String(hsmM).padStart(2, '0')}:${String(hsmS).padStart(2, '0')}`;
    
    const sD = calcSolar(), EoT_s = sD.eot * 60;
    const sLSM_C = sLSM + EoT_s; 
    const sHSV = (sLSM_C + 86400) % 86400;
    const lsvH = Math.floor(sHSV / 3600), lsvM = Math.floor((sHSV % 3600) / 60), lsvS = Math.floor(sHSV % 60);
    const hsvStr = `${String(lsvH).padStart(2, '0')}:${String(lsvM).padStart(2, '0')}:${String(lsvS).padStart(2, '0')}`;
    $('solar-true').textContent = `${hsvStr} (HSV)`;
    $('solar-true-header').textContent = hsvStr;
    $('eot').textContent = `${sD.eot.toFixed(2)} min`;
    $('solar-longitude-val').textContent = `${sD.solarLongitude.toFixed(2)} °`;

    // Lunar Phase
    const D_rad = calcLunarPhase();
    const ill = 0.5 * (1 - Math.cos(D_rad)); 
    $('lunar-phase-perc').textContent = `${(ill * 100).toFixed(1)}%`;
    calcLunarTime(cLon); 
}

function resetDisp() {
    lPos = null; lat = null; lon = null; distM = 0; sTime = null; maxSpd = 0; tLat = null; tLon = null;
    kSpd = 0; kUncert = 1000; lDomT = null;
    lGPST = null; lServH = null; lLocH = null; offH = 0;

    const defT = '--', ids = ['elapsed-time', 'speed-3d-inst', 'speed-stable', 'speed-stable-mm', 'speed-avg', 'speed-max', 'speed-ms', 'perc-light', 'perc-sound', 'distance-km-m', 'lunar-time', 'latitude', 'longitude', 'altitude', 'gps-accuracy', 'underground', 'solar-true', 'solar-mean', 'eot', 'solar-longitude-val', 'lunar-phase-perc', 'mc-time', 'air-temp', 'pressure', 'humidity', 'wind-speed', 'boiling-point', 'heading', 'bubble-level', 'cap-dest', 'solar-true-header', 'mode-indicator', 'speed-source-indicator', 'speed-error-perc', 'update-frequency'];

    ids.forEach(id => {
        const el = $(id);
        if (el) {
            if (id === 'speed-source-indicator') el.textContent = 'Source: N/A';
            else if (['air-temp', 'pressure', 'humidity', 'wind-speed', 'boiling-point', 'bubble-level'].includes(id)) el.textContent = 'N/A (API désactivée)'; 
            else if (id === 'altitude' || id === 'gps-accuracy') el.textContent = '-- m';
            else if (id === 'distance-km-m') el.textContent = '-- km | -- m';
            else if (id === 'mode-indicator') el.textContent = 'Mode: Jour ☀️';
            else if (id === 'speed-error-perc' || id === 'update-frequency') el.textContent = '--';
            else if (['mc-time', 'lunar-time', 'solar-mean', 'solar-true', 'solar-true-header'].includes(id)) el.textContent = '00:00:00';
            else el.textContent = defT;
        }
    });

    startBtn.disabled = false; stopBtn.disabled = true; resetMaxBtn.disabled = true; setTargetBtn.textContent = '🗺️ Aller';
    errorDisplay.style.display = 'none';
    document.body.classList.remove('night-mode'); $('gps-accuracy').classList.remove('max-precision');
}

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

    const now = performance.now();
    if (lDomT) $('update-frequency').textContent = `${(1000 / (now - lDomT)).toFixed(1)} Hz (DOM)`;
    lDomT = now;
}

function updateDisp(pos) {
    lat = pos.coords.latitude; lon = pos.coords.longitude;
    const alt = pos.coords.altitude, acc = pos.coords.accuracy, hdg = pos.coords.heading;   
    const spd = pos.coords.speed, cTime = pos.timestamp;
    
    lGPST = cTime; 
    if (sTime === null) sTime = cTime;

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
    lPos = pos; lPos.speedMS_3D = spd3D;

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
    const elapS = (cTime - sTime) / 1000;
    distM += sSpdFE * dt; 
    const spdAvg = elapS > 0 ? distM / elapS : 0; 
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    let spdErr = sSpdFE > MIN_SPD ? (Math.abs(spd3D - sSpdFE) / sSpdFE) * 100 : 0;

    $('elapsed-time').textContent = `${elapS.toFixed(2)} s`;
    $('speed-avg').textContent = `${(spdAvg * KMH_MS).toFixed(5)} km/h`; 
    $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    $('speed-error-perc').textContent = `${spdErr.toFixed(2)}%`; 
    $('latitude').textContent = `${lat.toFixed(6)}`; $('longitude').textContent = `${lon.toFixed(6)}`;
    $('altitude').textContent = `${alt !== null ? alt.toFixed(2) : '--'} m`;
    $('gps-accuracy').textContent = pText;
    $('underground').textContent = alt !== null && alt < ALT_TH ? 'Oui' : 'Non';
    $('heading').textContent = hdg !== null ? `${hdg.toFixed(1)} °` : '--';
    $('distance-km-m').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    speedSrc.textContent = `Source: ${spdSrc}`;

    if (tLat !== null && tLon !== null) {
        $('cap-dest').textContent = `${bearing(lat, lon, tLat, tLon).toFixed(1)} °`;
    }
}

function handleErr(err) {
    if (lGPST === null) syncH();
    errorDisplay.style.display = 'block';
    let msg = "❌ Erreur GPS inconnue. Utilisation du Fallback.";
    if (err.code === err.PERMISSION_DENIED) msg = "❌ L'accès à la localisation a été refusé. Utilisation du Fallback.";
    else if (err.code === err.POSITION_UNAVAILABLE) msg = "🛰️ Position non disponible. Signal GPS faible. Utilisation du Fallback.";
    else if (err.code === err.TIMEOUT) msg = "⏱️ Délai de recherche du GPS dépassé. Signal faible. Utilisation du Fallback.";
    errorDisplay.textContent = msg;
    stopGPS(false); 
}

function startGPS() {
    if (navigator.geolocation) {
        syncH(); 
        sTime = null; lGPST = null;
        resetMax(); distM = 0;
        
        wID = navigator.geolocation.watchPosition(updateDisp, handleErr, W_OPTS);
        
        if (domID === null) domID = setInterval(fastDOM, DOM_MS);

        startBtn.disabled = true; stopBtn.disabled = false; resetMaxBtn.disabled = false;
        $('gps-accuracy').classList.remove('max-precision');
    } else {
        errorDisplay.textContent = "❌ Géolocalisation non supportée par votre navigateur.";
        errorDisplay.style.display = 'block';
    }
}

function stopGPS(clearT = true) {
    if (wID !== null) { navigator.geolocation.clearWatch(wID); wID = null; }
    if (clearT) sTime = null;
    
    startBtn.disabled = false; stopBtn.disabled = true; 
    errorDisplay.style.display = 'block';
    errorDisplay.textContent = "PAUSE : Géolocalisation arrêtée. Heure UTC basée sur la dernière synchro/position connue.";
}

document.addEventListener('DOMContentLoaded', () => {
    resetDisp();
    syncH(); 
    domID = setInterval(fastDOM, DOM_MS); 
    startBtn.addEventListener('click', startGPS);
    stopBtn.addEventListener('click', stopGPS);
    resetMaxBtn.addEventListener('click', resetMax);
    setTargetBtn.addEventListener('click', setTarget);
});
