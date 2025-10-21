// =================================================================
// FICHIER COMPLET ET FONCTIONNEL : dashboard.js (Version Stable)
// =================================================================

// --- CONSTANTES GLOBALES ET INITIALISATION ---
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

// --- REFERENCES DOM ---
const $ = id => document.getElementById(id);
const startBtn = $('start-btn'), stopBtn = $('stop-btn'), resetMaxBtn = $('reset-max-btn');
const errorDisplay = $('error-message'), speedSrc = $('speed-source-indicator'); 
const setTargetBtn = $('set-target-btn'), modeInd = $('mode-indicator');
const toggleModeBtn = $('toggle-mode-btn'), autoModeBtn = $('auto-mode-btn');

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

// ===========================================
// GESTION CAPTEUR DE LUMINOSITÉ (ALS)
// ===========================================

function initALS() {
    if ('AmbientLightSensor' in window) {
        try {
            // Demande une fréquence de 5 Hz (5 mises à jour par seconde)
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

function toggleManualMode() {
    if (manualMode === null) {
        manualMode = true; 
        toggleModeBtn.textContent = '☀️ Passer en Jour';
        autoModeBtn.style.display = 'inline-block';
    } else if (manualMode === true) {
        manualMode = false;
        toggleModeBtn.textContent = '🌙 Passer en Nuit';
    } else if (manualMode === false) {
        manualMode = true;
        toggleModeBtn.textContent = '☀️ Passer en Jour';
    }
}

function setAutoMode() {
    manualMode = null;
    toggleModeBtn.textContent = '🌗 Bascule Manuelle';
    autoModeBtn.style.display = 'none';
}

function updateDM(lat, lon) {
    let isN = false;
    let modeSource = 'Initialisation...';

    if (manualMode !== null) {
        isN = manualMode;
        modeSource = `FORCÉ (${isN ? 'Nuit' : 'Jour'})`;
    } else if (lastLux !== null) {
        // Utilise le capteur de luminosité si disponible
        isN = lastLux < LUX_NIGHT_TH;
        modeSource = `Luminosité (${lastLux.toFixed(0)} Lux)`;
    } else {
        // Fallback: Utilise l'angle du Soleil (lever/coucher)
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
    }
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
    
    // Heure Minecraft (basée sur 24000 ticks par jour)
    const mcTicksPD = 24000, msSinceM = now.getTime() % 86400000;
    const mcTicks = (msSinceM * mcTicksPD) / 86400000;
    const mcM = Math.floor(mcTicks / 20) % 1440; 
    const h = Math.floor(mcM / 60), m = mcM % 60, s = Math.floor((mcTicks % 20) / 20 * 60);
    $('mc-time').textContent = `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;

    // Temps Solaire Moyen (LSM)
    const sUT = now.getUTCHours() * 3600 + now.getUTCMinutes() * 60 + now.getUTCSeconds();
    const sLSM = (sUT + (cLon * 4 * 60) + 86400) % 86400;
    const hsmH = Math.floor(sLSM / 3600), hsmM = Math.floor((sLSM % 3600) / 60), hsmS = Math.floor(sLSM % 60);
    $('solar-mean').textContent = `${String(hsmH).padStart(2, '0')}:${String(hsmM).padStart(2, '0')}:${String(hsmS).padStart(2, '0')}`;
    
    // Temps Solaire Vrai (HSV)
    const sD = calcSolar(), EoT_s = sD.eot * 60;
    const sLSM_C = sLSM + EoT_s; 
    const sHSV = (sLSM_C + 86400) % 86400;
    const lsvH = Math.floor(sHSV / 3600), lsvM = Math.floor((sHSV % 3600) / 60), lsvS = Math.floor(sHSV % 60);
    const hsvStr = `${String(lsvH).padStart(2, '0')}:${String(lsvM).padStart(2, '0')}:${String(lsvS).padStart(2, '0')}`;
    $('solar-true').textContent = `${hsvStr} (HSV)`;
    $('solar-true-header').textContent = hsvStr;
    $('eot').textContent = `${sD.eot.toFixed(2)} min`;
    $('solar-longitude-val').textContent = `${sD.solarLongitude.toFixed(2)} °`;

    // Phase Lunaire
    const D_rad = calcLunarPhase();
    const ill = 0.5 * (1 - Math.cos(D_rad)); 
    $('lunar-phase-perc').textContent = `${(ill * 100).toFixed(1)}%`;
    calcLunarTime(cLon); 
}

function resetDisp() {
    lPos = null; lat = null; lon = null; distM = 0; sTime = null; maxSpd = 0; tLat = null; tLon = null;
    kSpd = 0; kUncert = 1000; lDomT = null; lServH = null; lLocH = null; lastLux = null;
    manualMode = null; 
    
    const defT = '--', ids = ['elapsed-time', 'speed-3d-inst', 'speed-stable', 'speed-stable-mm', 'speed-avg', 'speed-max', 'speed-ms', 'perc-light', 'perc-sound', 'distance-km-m', 'lunar-time', 'latitude', 'longitude', 'altitude', 'gps-accuracy', 'underground', 'solar-true', 'solar-mean', 'eot', 'solar-longitude-val', 'lunar-phase-perc', 'mc-time', 'air-temp', 'pressure', 'humidity', 'wind-speed', 'boiling-point', 'heading', 'bubble-level', 'cap-dest', 'solar-true-header', 'mode-indicator', 'speed-source-indicator', 'speed-error-perc', 'update-frequency', 'sun-elevation', 'illuminance-lux'];

    ids.forEach(id => {
        const el = $(id);
        if (el) {
            if (id === 'speed-source-indicator') el.textContent = 'Source: N/A';
            else if (['air-temp', 'pressure', 'humidity', 'wind-speed', 'boiling-point', 'bubble-level'].includes(id)) el.textContent = 'N/A (API désactivée)'; 
            else if (id === 'altitude' || id === 'gps-accuracy') el.textContent = '-- m';
            else if (id === 'distance-km-m') el.textContent = '-- km | -- m';
            else if (id === 'mode-indicator') el.textContent = 'Mode: Jour ☀️';
            else if (id === 'speed-error-perc' || id === 'update-frequency') el.textContent = '--';
            else if (id === 'sun-elevation') el.textContent = '-- °';
            else if (id === 'illuminance-lux') el.textContent = 'Initialisation...';
            else if (['mc-time', 'lunar-time', 'solar-mean', 'solar-true', 'solar-true-header'].includes(id)) el.textContent = '00:00:00';
            else el.textContent = defT;
        }
    });

    startBtn.disabled = false; stopBtn.disabled = true; resetMaxBtn.disabled = true; setTargetBtn.textContent = '🗺️ Aller';
    toggleModeBtn.textContent = '🌗 Bascule Manuelle';
    autoModeBtn.style.display = 'none';
    
    errorDisplay.style.display = 'none';
    document.body.classList.remove('night-mode'); $('gps-accuracy').classList.remove('max-precision');
}

function resetMax() { maxSpd = 0; $('speed-max').textContent = '0.00000 km/h'; }

function fastDOM() {
    // ⚠️ Cette fonction doit TOUJOURS tourner pour rafraîchir l'heure et l'astro (à ~60Hz)
    const latA = lat ?? D_LAT, lonA = lon ?? D_LON;
    updateAstro(latA, lonA); 
    updateDM(latA, lonA); 
    
    if (!lPos || sTime === null) {
         // Si aucune position GPS n'a jamais été reçue ou n'est active, on ne met à jour que l'heure/astro
         const pNow = performance.now();
         if (lDomT) $('update-frequency').textContent = `${(1000 / (pNow - lDomT)).toFixed(1)} Hz (DOM)`;
         lDomT = pNow;
         return; 
    }
    
    // Calculs de vitesse et distance basés sur la DERNIÈRE position reçue (lPos)
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
    lat = pos.coords.latitude; lon = pos.coords.longitude;
    const alt = pos.coords.altitude, acc = pos.coords.accuracy, hdg = pos.coords.heading;   
    const spd = pos.coords.speed, cTime = pos.timestamp; 
    
    syncH(); // Synchronisation de l'heure
    if (sTime === null) sTime = getCDate().getTime();

    // Gestion de la précision
    if (acc > MAX_ACC) { $('gps-accuracy').textContent = `❌ ${acc.toFixed(0)} m (Trop Imprécis)`; if (lPos === null) lPos = pos; return; }
    
    let spdH = spd ?? 0, spdSrc = spd !== null && spd !== undefined ? 'Puce GPS (Doppler)' : 'Calculée (Dérivée)';
    let spdV = 0;
    
    const dt = lPos ? (cTime - lPos.timestamp) / 1000 : MIN_DT;

    // Calcul de la vitesse et distance si les données précédentes existent
    if (lPos && dt > 0.1) { 
        const dH = dist(lPos.coords.latitude, lPos.coords.longitude, lat, lon);
        if (spd === null || spd === undefined) spdH = dH / dt; 
        if (alt !== null && lPos.coords.altitude !== null) spdV = (alt - lPos.coords.altitude) / dt; 
    }
    
    const spd3D = Math.sqrt(spdH ** 2 + spdV ** 2);
    lPos = pos; lPos.speedMS_3D = spd3D; lPos.timestamp = cTime; 

    // Ajustement du filtre de Kalman en fonction de la précision
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
    
    distM += sSpdFE * dt; 
    
    const elapS = (getCDate().getTime() - sTime) / 1000;
    const spdAvg = elapS > 0 ? distM / elapS : 0; 
    
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    let spdErr = sSpdFE > MIN_SPD ? (Math.abs(spd3D - sSpdFE) / sSpdFE) * 100 : 0;

    // Mise à jour des valeurs du DOM
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
    syncH(); 
    errorDisplay.style.display = 'block';
    let msg = "❌ Erreur GPS inconnue. Utilisation du temps Internet/Local.";
    if (err.code === err.PERMISSION_DENIED) msg = "❌ L'accès à la localisation a été refusé. Utilisation du temps Internet/Local.";
    else if (err.code === err.POSITION_UNAVAILABLE) msg = "🛰️ Position non disponible. Signal GPS faible. Utilisation du temps Internet/Local.";
    else if (err.code === err.TIMEOUT) msg = "⏱️ Délai de recherche du GPS dépassé. Signal faible. Utilisation du temps Internet/Local.";
    errorDisplay.textContent = msg;
    stopGPS(false); // Stop le watchPosition, mais conserve les données s'il y en a.
}

function startGPS() {
    if (navigator.geolocation) {
        syncH(); 
        sTime = null; 
        resetMax(); distM = 0;
        
        wID = navigator.geolocation.watchPosition(updateDisp, handleErr, W_OPTS);
        
        // S'assurer que le DOM tourne, au cas où il aurait été stoppé par un bug antérieur
        if (domID === null) domID = setInterval(fastDOM, DOM_MS); 

        startBtn.disabled = true; stopBtn.disabled = false; resetMaxBtn.disabled = false;
        $('gps-accuracy').classList.remove('max-precision');
    } else {
        errorDisplay.textContent = "❌ Géolocalisation non supportée par votre navigateur.";
        errorDisplay.style.display = 'block';
    }
}

function stopGPS(clearT = true) {
    // 🛑 Arrête uniquement la surveillance GPS (wID)
    if (wID !== null) { navigator.geolocation.clearWatch(wID); wID = null; }
    
    // 🔥 Le rafraîchissement DOM (fastDOM/domID) CONTINUE de tourner pour l'heure et l'astro.
    
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
    autoModeBtn.addEventListener('click', set
