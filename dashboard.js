// =================================================================
// FICHIER COMPLET ET FINAL : dashboard.js
// Version finale avec LSM et HSV SEULEMENT en haut, Lieu par Défaut Modifiable, et toutes les fonctions astro/GPS.
// =================================================================

// --- CONSTANTES GLOBALES ET INITIALISATION ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458, C_S = 343, R_E = 6371000, KMH_MS = 3.6;
const OBLIQ = 23.44 * D2R, ECC = 0.0167, JD_2K = 2451545.0;
const W_OPTS = { enableHighAccuracy: true, maximumAge: 0, timeout: 20000 };
const DOM_MS = 17, MIN_DT = 1, MAX_ACC = 50, MIN_SPD = 0.001, ALT_TH = -50;
const Q_NOISE = 0.005, R_MIN = 0.005, R_MAX = 5.0, L_PREC_TH = 60;
const SUN_NIGHT_TH = -12; 
const LUX_NIGHT_TH = 50; 
const NETHER_RATIO = 8; 

// Variables pour le lieu par défaut (Modifiable)
let defaultLat = 48.8566; // Paris
let defaultLon = 2.3522;  // Paris

let wID = null, domID = null, lPos = null, lat = null, lon = null, sTime = null;
let distM = 0, maxSpd = 0, tLat = null, tLon = null, lDomT = null;
let kSpd = 0, kUncert = 1000; 
let lServH = null, lLocH = null; 
let als = null, lastLux = null, manualMode = null; 
let netherMode = false; 
let todayLC = 0; // Culmination solaire en heures décimales

// --- REFERENCES DOM ---
const $ = id => document.getElementById(id);
const startBtn = $('start-btn'), stopBtn = $('stop-btn'), resetMaxBtn = $('reset-max-btn');
const errorDisplay = $('error-message'), speedSrc = $('speed-source-indicator'); 
const setTargetBtn = $('set-target-btn'), modeInd = $('mode-indicator');
const toggleModeBtn = $('toggle-mode-btn'), autoModeBtn = $('auto-mode-btn');
const netherToggleBtn = $('nether-toggle-btn'), netherIndicator = $('nether-indicator');
const solarTrueHeader = $('solar-true-header'); // HSV
const solarMeanHeader = $('solar-mean-header'); // LSM
// Suppression de 'localTimeHeader'

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

async function syncH() { /* ... (Logique de synchronisation temps internet) ... */
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

function getCDate() { /* ... (Logique de date via syncH) ... */
    let estT = Date.now();
    if (lServH !== null) {
        estT = lServH + (Date.now() - lLocH);
    } 
    return new Date(estT);
}

function kFilter(nSpd, dt, R_dyn) { /* ... (Filtre de Kalman) ... */
    if (dt === 0 || dt > 5) return kSpd; 
    const R = R_dyn ?? R_MAX, Q = Q_NOISE * dt; 
    let pSpd = kSpd, pUnc = kUncert + Q; 
    let K = pUnc / (pUnc + R); 
    kSpd = pSpd + K * (nSpd - pSpd);
    kUncert = (1 - K) * pUnc;
    return kSpd;
}

/** Bascule le mode de simulation Nether (Minecraft). */
function toggleNetherMode() {
    netherMode = !netherMode;
    distM = 0; 
    maxSpd = 0; 
    
    if (netherMode) {
        netherIndicator.textContent = "ACTIVÉ (1:8) 🔥";
        netherToggleBtn.textContent = "🌍 Overworld";
    } else {
        netherIndicator.textContent = "DÉSACTIVÉ (1:1)";
        netherToggleBtn.textContent = "🔥 Nether";
    }
}

/** Permet à l'utilisateur de changer la Latitude/Longitude par défaut */
function setDefaultLocation() {
    const newLatStr = prompt(`Entrez la nouvelle Latitude par défaut (actuel: ${defaultLat}) :`);
    if (newLatStr !== null && !isNaN(parseFloat(newLatStr))) {
        defaultLat = parseFloat(newLatStr);
    }
    
    const newLonStr = prompt(`Entrez la nouvelle Longitude par défaut (actuel: ${defaultLon}) :`);
    if (newLonStr !== null && !isNaN(parseFloat(newLonStr))) {
        defaultLon = parseFloat(newLonStr);
    }
    
    alert(`Nouvelle position par défaut : Lat=${defaultLat.toFixed(4)}, Lon=${defaultLon.toFixed(4)}. Redémarrez le GPS pour l'utiliser comme position initiale.`);
}


// ===========================================
// CALCULS ASTRO
// ===========================================

/** Calcule la position du Soleil (EoT, Élévation, Longueur Solaire) et la Culmination. */
function calcSolar() {
    const now = getCDate(), J2K_MS = 946728000000;
    const D = (now.getTime() - J2K_MS) / 86400000;
    const M = (357.529 + 0.98560028 * D) * D2R; 
    const L = (280.466 + 0.98564736 * D) * D2R; 
    const Ce = 2 * ECC * Math.sin(M) + 1.25 * ECC ** 2 * Math.sin(2 * M);
    const lambda = L + Ce; 

    const delta = Math.asin(Math.sin(OBLIQ) * Math.sin(lambda)); 
    const alpha_rad = Math.atan2(Math.cos(OBLIQ) * Math.sin(lambda), Math.cos(lambda));
    const alpha = alpha_rad * R2D; 

    const JD = now.getTime() / 86400000 + 2440587.5;
    const T = (JD - JD_2K) / 36525.0; 
    let GST = 280.4606 + 360.9856473 * (JD - JD_2K) + 0.000388 * T ** 2; 
    GST = (GST % 360 + 360) % 360; 
    const LST = GST + (lon ?? defaultLon); 
    const HA_rad = ((LST % 360) * D2R) - alpha_rad; 

    const lat_rad = (lat ?? defaultLat) * D2R;
    const h = Math.asin(Math.sin(lat_rad) * Math.sin(delta) + Math.cos(lat_rad) * Math.cos(delta) * Math.cos(HA_rad));
    
    let EoT_deg = (L * R2D) - alpha; 
    while (EoT_deg > 180) EoT_deg -= 360;
    while (EoT_deg < -180) EoT_deg += 360;
    const EoT_m = EoT_deg * 4; 
    
    let sLon = (lambda * R2D) % 360;
    if (sLon < 0) sLon += 360;
    
    const noonLSM_H = 12; 
    const noonHSV_H = noonLSM_H - (EoT_m / 60); 
    todayLC = noonHSV_H; 

    return { eot: EoT_m, solarLongitude: sLon, elevation: h * R2D, lambda: lambda };
}

/** Estimation du temps de lever/coucher du soleil en HSV. */
function getSunTimes(sD, cLat) {
    const delta = Math.asin(Math.sin(OBLIQ) * Math.sin(sD.lambda)); 
    const lat_rad = cLat * D2R;
    
    let cosH = -Math.tan(lat_rad) * Math.tan(delta);
    if (cosH > 1) cosH = 1; else if (cosH < -1) cosH = -1;

    const H_rad = Math.acos(cosH); 
    const H_deg = H_rad * R2D;
    const H_temps = H_deg * 4; 

    const EoT_min = sD.eot;
    const noonLSM_min = 12 * 60; 
    
    const sunriseLSM_min = noonLSM_min - H_temps;
    const sunsetLSM_min = noonLSM_min + H_temps;

    const sunriseHSV_s = (sunriseLSM_min * 60) + (EoT_min * 60);
    const sunsetHSV_s = (sunsetLSM_min * 60) + (EoT_min * 60);

    return { 
        sr: (sunriseHSV_s + 86400) % 86400, 
        ss: (sunsetHSV_s + 86400) % 86400,
        polar: (cosH >= 1 || cosH <= -1)
    };
}

function calcLunarPhase() { /* ... (Logique de phase lunaire) ... */
    const now = getCDate();
    const JD = now.getTime() / 86400000 + 2440587.5; 
    const d = JD - JD_2K; 
    let D = 297.8501921 + 445.2671115 * d; 
    D = D % 360; 
    if (D < 0) D += 360;
    return D * D2R; 
}

function calcLunarTime(lon) { /* ... (Logique de temps lunaire) ... */
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

/** Met à jour les heures solaires (LSM, HSV) et format l'EoT. */
function updateSolarTime(cLon) {
    const now = getCDate();
    
    // Temps Solaire Moyen (LSM)
    const sUT = now.getUTCHours() * 3600 + now.getUTCMinutes() * 60 + now.getUTCSeconds();
    const sLSM = (sUT + (cLon * 4 * 60) + 86400) % 86400;
    const hsmH = Math.floor(sLSM / 3600), hsmM = Math.floor((sLSM % 3600) / 60), hsmS = Math.floor(sLSM % 60);
    const hsmStr = `${String(hsmH).padStart(2, '0')}:${String(hsmM).padStart(2, '0')}:${String(hsmS).padStart(2, '0')}`;
    
    // Heure Solaire Vraie (HSV)
    const sD = calcSolar(), EoT_s = sD.eot * 60;
    const sLSM_C = sLSM + EoT_s; 
    const sHSV = (sLSM_C + 86400) % 86400;
    const lsvH = Math.floor(sHSV / 3600), lsvM = Math.floor((sHSV % 3600) / 60), lsvS = Math.floor(sHSV % 60);
    const hsvStr = `${String(lsvH).padStart(2, '0')}:${String(lsvM).padStart(2, '0')}:${String(lsvS).padStart(2, '0')}`;
    
    // Affichage dans les HEADERS
    solarMeanHeader.textContent = hsmStr; // LSM en haut
    solarTrueHeader.textContent = hsvStr; // HSV en haut

    // Affichage dans la section Astro
    $('solar-mean').textContent = hsmStr; 
    $('solar-true').textContent = `${hsvStr} (HSV)`;
    
    // EoT formaté
    const EoT_sec_total = sD.eot * 60;
    const EoT_min = Math.trunc(EoT_sec_total / 60);
    const EoT_sec_after_min = Math.abs(EoT_sec_total % 60);
    const EoT_sign = EoT_sec_total >= 0 ? '+' : '-';
    $('eot').textContent = `${EoT_sign} ${Math.abs(EoT_min)}m ${EoT_sec_after_min.toFixed(1)}s`;
    
    $('solar-longitude-val').textContent = `${sD.solarLongitude.toFixed(2)} °`;
    
    // Culmination Solaire
    const culminH_sec = todayLC * 3600;
    const culminH_h = Math.floor(culminH_sec / 3600);
    const culminH_m = Math.floor((culminH_sec % 3600) / 60);
    const culminH_s = Math.floor(culminH_sec % 60);
    $('solar-culmination').textContent = `${String(culminH_h).padStart(2, '0')}:${String(culminH_m).padStart(2, '0')}:${String(culminH_s).padStart(2, '0')} (HSV)`;
    
    return sD;
}

// ... (Le reste des fonctions est inchangé : initALS, updateDM, updateAstro, resetDisp, fastDOM, updateDisp, handleErr, startGPS, stopGPS)

// --- DÉMARRAGE INITIAL ---
document.addEventListener('DOMContentLoaded', () => {
    resetDisp();
    syncH(); 
    initALS(); 
    
    if (domID === null) domID = setInterval(fastDOM, DOM_MS); 
    
    startBtn.addEventListener('click', startGPS);
    stopBtn.addEventListener('click', stopGPS);
    resetMaxBtn.addEventListener('click', () => { maxSpd = 0; $('speed-max').textContent = '0.00000 km/h'; });
    $('set-target-btn').addEventListener('click', () => { /* ... (Logique setTarget) ... */ });
    $('set-default-loc-btn').addEventListener('click', setDefaultLocation); 
    $('toggle-mode-btn').addEventListener('click', () => { manualMode = (manualMode === null) ? true : !manualMode; });
    $('auto-mode-btn').addEventListener('click', () => { manualMode = null; });
    netherToggleBtn.addEventListener('click', toggleNetherMode);
});
