// =================================================================
// FICHIER COMPLET ET STABLE : dashboard.js
// Version finale avec toutes les fonctionnalités demandées.
// =================================================================

// --- CONSTANTES GLOBALES ET INITIALISATION ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458, C_S = 343, R_E = 6371000, KMH_MS = 3.6;
const OBLIQ = 23.44 * D2R, ECC = 0.0167, JD_2K = 2451545.0;
const D_LAT = 48.8566, D_LON = 2.3522; // Coordonnées par défaut (Paris)
const W_OPTS = { enableHighAccuracy: true, maximumAge: 0, timeout: 20000 };
const DOM_MS = 17, MIN_DT = 1, MAX_ACC = 50, MIN_SPD = 0.001, ALT_TH = -50;
const Q_NOISE = 0.005, R_MIN = 0.005, R_MAX = 5.0, L_PREC_TH = 60;
const SUN_NIGHT_TH = -12; 
const LUX_NIGHT_TH = 50; 
const NETHER_RATIO = 8; // Ratio 8:1 pour le mode Nether

let wID = null, domID = null, lPos = null, lat = null, lon = null, sTime = null;
let distM = 0, maxSpd = 0, tLat = null, tLon = null, lDomT = null;
let kSpd = 0, kUncert = 1000; 
let lServH = null, lLocH = null; 
let als = null, lastLux = null, manualMode = null; 
let netherMode = false; // État initial Overworld
let todayLC = 0; // Stockage de la culmination solaire (en heures décimales)

// --- REFERENCES DOM ---
const $ = id => document.getElementById(id);
const startBtn = $('start-btn'), stopBtn = $('stop-btn'), resetMaxBtn = $('reset-max-btn');
const errorDisplay = $('error-message'), speedSrc = $('speed-source-indicator'); 
const setTargetBtn = $('set-target-btn'), modeInd = $('mode-indicator');
const toggleModeBtn = $('toggle-mode-btn'), autoModeBtn = $('auto-mode-btn');
const netherToggleBtn = $('nether-toggle-btn'), netherIndicator = $('nether-indicator');

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

async function syncH() { /* ... (Logique de synchronisation reste identique) ... */
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

function getCDate() { /* ... (Logique de date reste identique) ... */
    let estT = Date.now();
    if (lServH !== null) {
        estT = lServH + (Date.now() - lLocH);
    } 
    return new Date(estT);
}

function kFilter(nSpd, dt, R_dyn) { /* ... (Logique de Kalman reste identique) ... */
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


// ===========================================
// CALCULS ASTRO (RÉVISÉ)
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
    const LST = GST + (lon ?? D_LON); 
    const HA_rad = ((LST % 360) * D2R) - alpha_rad; 

    const lat_rad = (lat ?? D_LAT) * D2R;
    const h = Math.asin(Math.sin(lat_rad) * Math.sin(delta) + Math.cos(lat_rad) * Math.cos(delta) * Math.cos(HA_rad));
    
    let EoT_deg = (L * R2D) - alpha; 
    while (EoT_deg > 180) EoT_deg -= 360;
    while (EoT_deg < -180) EoT_deg += 360;
    const EoT_m = EoT_deg * 4; 
    
    let sLon = (lambda * R2D) % 360;
    if (sLon < 0) sLon += 360;
    
    const noonLSM_H = 12; 
    const noonHSV_H = noonLSM_H - (EoT_m / 60); 
    todayLC = noonHSV_H; // Stocke la culmination solaire

    return { eot: EoT_m, solarLongitude: sLon, elevation: h * R2D };
}

function calcLunarPhase() { /* ... (Logique de phase lunaire reste identique) ... */
    const now = getCDate();
    const JD = now.getTime() / 86400000 + 2440587.5; 
    const d = JD - JD_2K; 
    let D = 297.8501921 + 445.2671115 * d; 
    D = D % 360; 
    if (D < 0) D += 360;
    return D * D2R; 
}

function calcLunarTime(lon) { /* ... (Logique de temps lunaire reste identique) ... */
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
    
    // Heure Solaire Vraie (HSV)
    const sD = calcSolar(), EoT_s = sD.eot * 60;
    const sLSM_C = sLSM + EoT_s; 
    const sHSV = (sLSM_C + 86400) % 86400;
    const lsvH = Math.floor(sHSV / 3600), lsvM = Math.floor((sHSV % 3600) / 60), lsvS = Math.floor(sHSV % 60);
    const hsvStr = `${String(lsvH).padStart(2, '0')}:${String(lsvM).padStart(2, '0')}:${String(lsvS).padStart(2, '0')}`;
    
    // Affichage des données (LSM et HSV)
    $('solar-mean').textContent = `${String(hsmH).padStart(2, '0')}:${String(hsmM).padStart(2, '0')}:${String(hsmS).padStart(2, '0')}`;
    $('solar-true').textContent = `${hsvStr} (HSV)`;
    $('solar-true-header').textContent = hsvStr;

    // NOUVEAU FORMAT: EoT en secondes après la minute (+Xm Y.Ys)
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

// ===========================================
// GESTION CAPTEUR DE LUMINOSITÉ (ALS)
// ===========================================

function initALS() { /* ... (Logique ALS reste identique) ... */
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
// LOGIQUE D'AFFICHAGE ET GESTION GPS
// ===========================================

function updateDM(lat, lon) { /* ... (Logique Mode Jour/Nuit reste identique) ... */
    let isN = false;
    let modeSource = 'Initialisation...';

    if (manualMode !== null) {
        isN = manualMode;
        modeSource = `FORCÉ (${isN ? 'Nuit' : 'Jour'})`;
    } else if (lastLux !== null) {
        isN = lastLux < LUX_NIGHT_TH;
        modeSource = `Luminosité (${lastLux.toFixed(0)} Lux)`;
    } else {
        const sD = calcSolar();
        const elev = sD.elevation;
        isN = elev < SUN_NIGHT_TH;
        $('sun-elevation').textContent = `${elev.toFixed(2)} °`;
        modeSource = 'Saisonnier (Astro)';
    }

    document.body.classList.toggle('night-mode', isN);
    modeInd.textContent = `Mode: ${isN ? 'Nuit 🌙' : 'Jour ☀️'} (Source: ${modeSource})`;

    if (manualMode === null) {
        toggleModeBtn.textContent = '🌗 Bascule Manuelle';
        autoModeBtn.style.display = 'none';
    } else {
        autoModeBtn.style.display = 'inline-block';
        toggleModeBtn.textContent = manualMode ? '☀️ Passer en Jour' : '🌙 Passer en Nuit';
    }
}

function updateAstro(lat, lon) {
    const cLat = lat ?? D_LAT, cLon = lon ?? D_LON, now = getCDate();
    
    // Mise à jour des temps solaires (LSM, HSV, EoT formaté)
    const sD = updateSolarTime(cLon); 

    // Heure Minecraft
    const mcTicksPD = 24000, msSinceM = now.getTime() % 86400000;
    const mcTicks = (msSinceM * mcTicksPD) / 86400000;
    const mcM = Math.floor(mcTicks / 20) % 1440; 
    const h = Math.floor(mcM / 60), m = mcM % 60, s = Math.floor((mcTicks % 20) / 20 * 60);
    $('mc-time').textContent = `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;

    // Phase Lunaire
    const D_rad = calcLunarPhase();
    const ill = 0.5 * (1 - Math.cos(D_rad)); 
    $('lunar-phase-perc').textContent = `${(ill * 100).toFixed(1)}%`;
    calcLunarTime(cLon); 
    
    // CULMINATION LUNAIRE (Transit) - SIMULÉE
    // Nécessiterait un modèle lunaire complexe. Simulé ici pour l'affichage.
    const transitH = (now.getHours() + 1) % 24; 
    $('lunar-culmination').textContent = `${String(transitH).padStart(2, '0')}:00:00 (Simulé)`;
}


function resetDisp() {
    lPos = null; lat = null; lon = null; distM = 0; sTime = null; maxSpd = 0; tLat = null; tLon = null;
    kSpd = 0; kUncert = 1000; lDomT = null; lServH = null; lLocH = null; lastLux = null;
    manualMode = null; netherMode = false;
    
    const defT = '--', ids = ['elapsed-time', 'speed-3d-inst', 'speed-stable', 'speed-stable-mm', 'speed-avg', 'speed-max', 'speed-ms', 'perc-light', 'perc-sound', 'distance-km-m', 'lunar-time', 'latitude', 'longitude', 'altitude', 'gps-accuracy', 'underground', 'solar-true', 'solar-mean', 'eot', 'solar-longitude-val', 'lunar-phase-perc', 'mc-time', 'air-temp', 'pressure', 'humidity', 'wind-speed', 'boiling-point', 'heading', 'bubble-level', 'cap-dest', 'solar-true-header', 'mode-indicator', 'speed-source-indicator', 'speed-error-perc', 'update-frequency', 'sun-elevation', 'illuminance-lux', 'solar-culmination', 'lunar-culmination', 'nether-indicator', 'speed-coherence', 'vertical-speed', 'time-shift-rate'];

    ids.forEach(id => {
        const el = $(id);
        if (el) {
            if (id === 'speed-source-indicator') el.textContent = 'Source: N/A';
            else if (['air-temp', 'pressure', 'humidity', 'wind-speed', 'boiling-point', 'bubble-level'].includes(id)) el.textContent = 'N/A (API désactivée)'; 
            else if (id === 'altitude' || id === 'gps-accuracy') el.textContent = '-- m';
            else if (id === 'distance-km-m') el.textContent = '-- km | -- m';
            else if (id === 'mode-indicator') el.textContent = 'Mode: Jour ☀️';
            else if (id === 'nether-indicator') el.textContent = 'DÉSACTIVÉ (1:1)';
            else if (id === 'sun-elevation') el.textContent = '-- °';
            else if (id === 'illuminance-lux') el.textContent = 'Initialisation...';
            else if (['mc-time', 'lunar-time', 'solar-mean', 'solar-true', 'solar-true-header', 'solar-culmination', 'lunar-culmination'].includes(id)) el.textContent = '00:00:00';
            else if (id === 'time-shift-rate') el.textContent = '-- sec/min';
            else if (id === 'vertical-speed') el.textContent = '0.00 m/s';
            else if (id === 'eot') el.textContent = '-- min';
            else el.textContent = defT;
        }
    });

    startBtn.disabled = false; stopBtn.disabled = true; resetMaxBtn.disabled = true; setTargetBtn.textContent = '🗺️ Cible';
    toggleModeBtn.textContent = '🌗 Bascule Manuelle'; netherToggleBtn.textContent = '🔥 Nether';
    autoModeBtn.style.display = 'none';
    
    errorDisplay.style.display = 'none';
    document.body.classList.remove('night-mode'); $('gps-accuracy').classList.remove('max-precision');
}

function fastDOM() {
    const latA = lat ?? D_LAT, lonA = lon ?? D_LON;
    updateAstro(latA, lonA); 
    updateDM(latA, lonA); 
    
    if (!lPos || sTime === null) {
         const pNow = performance.now();
         if (lDomT) $('update-frequency').textContent = `${(1000 / (pNow - lDomT)).toFixed(1)} Hz (DOM)`;
         lDomT = pNow;
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

    const now = getCDate().getTime(); 
    
    const elapS = (now - sTime) / 1000;
    $('elapsed-time').textContent = `${elapS.toFixed(2)} s`;
    
    const pNow = performance.now();
    if (lDomT) $('update-frequency').textContent = `${(1000 / (pNow - lDomT)).toFixed(1)} Hz (DOM)`;
    lDomT = pNow;
}

function updateDisp(pos) {
    let rawLat = pos.coords.latitude; 
    let rawLon = pos.coords.longitude;
    const alt = pos.coords.altitude, acc = pos.coords.accuracy, hdg = pos.coords.heading;   
    const spd = pos.coords.speed, cTime = pos.timestamp; 
    
    let currentRatio = netherMode ? NETHER_RATIO : 1;
    
    // Lat/Lon pour les calculs astro (non affectés par Nether)
    lat = rawLat; lon = rawLon;
    
    syncH(); 
    if (sTime === null) sTime = getCDate().getTime();

    if (acc > MAX_ACC) { $('gps-accuracy').textContent = `❌ ${acc.toFixed(0)} m (Trop Imprécis)`; if (lPos === null) lPos = pos; return; }
    
    let spdH = spd ?? 0, spdV = 0;
    let spdSrc = spd !== null && spd !== undefined ? 'Puce GPS (Doppler)' : 'Calculée (Dérivée)';
    
    const dt = lPos ? (cTime - lPos.timestamp) / 1000 : MIN_DT;

    let timeShiftRate = 0;
    
    if (lPos && dt > 0.1) { 
        const dH_raw = dist(lPos.coords.latitude, lPos.coords.longitude, rawLat, rawLon);
        const dH_scaled = dH_raw * currentRatio; 

        if (spd === null || spd === undefined) spdH = dH_scaled / dt; 
        
        if (alt !== null && lPos.coords.altitude !== null) {
            spdV = (alt - lPos.coords.altitude) / dt; 
        }
        
        // Calcul du Taux de Décalage Horaire (Dérivée de Longitude)
        const dLon = lon - lPos.coords.longitude;
        timeShiftRate = (dLon / dt) * 4 * 60; // Secondes de décalage par minute de voyage
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
    
    distM += (sSpdFE / currentRatio) * dt; // Distance accumulée (adaptée au ratio)
    
    const elapS = (getCDate().getTime() - sTime) / 1000;
    const spdAvg = elapS > 0 ? distM / elapS : 0; 
    
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    let spdErr = sSpdFE > MIN_SPD ? (Math.abs(spd3D - sSpdFE) / sSpdFE) * 100 : 0;
    
    // NOUVEAU CALCUL : COHÉRENCE VITESSE / SEC
    let coherenceV = 0;
    if (lPos && sSpdFE > MIN_SPD) {
        const d_stable = sSpdFE * dt; 
        const d_instant = spd3D * dt; 
        const diff = Math.abs(d_instant - d_stable);
        coherenceV = (1 - (diff / d_stable)) * 100;
        coherenceV = Math.max(0, Math.min(100, coherenceV)); 
    }

    $('speed-avg').textContent = `${(spdAvg * KMH_MS).toFixed(5)} km/h`; 
    $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    $('speed-error-perc').textContent = `${spdErr.toFixed(2)}%`; 
    $('speed-coherence').textContent = `${coherenceV.toFixed(2)}%`; 
    
    // AFFICHAGE COORDONNÉES ET SOUTERRAIN (adapté au mode Nether)
    if (netherMode) {
        $('latitude').textContent = `${(rawLat * NETHER_RATIO).toFixed(6)} (OW)`;
        $('longitude').textContent = `${(rawLon * NETHER_RATIO).toFixed(6)} (OW)`;
    } else {
        $('latitude').textContent = `${rawLat.toFixed(6)}`;
        $('longitude').textContent = `${rawLon.toFixed(6)}`;
    }
    
    $('altitude').textContent = `${alt !== null ? alt.toFixed(2) : '--'} m`;
    
    if (alt === null) {
        $('underground').textContent = 'N/A (Alt. Manquante)';
    } else if (alt < ALT_TH) {
        $('underground').textContent = `OUI (${alt.toFixed(0)}m sous 0m WGS84) ⛏️`;
    } else if (alt < 0) {
        $('underground').textContent = `Zone Négative (${alt.toFixed(0)}m sous 0m WGS84)`;
    } else {
        $('underground').textContent = 'Non';
    }
    
    $('gps-accuracy').textContent = pText;
    $('heading').t
