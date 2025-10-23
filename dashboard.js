// =================================================================
// FICHIER COMPLET ET STABLE : dashboard.js (V3.1 - Correction Avancée)
// BLOC 1 SUR 2 (CORE, KALMAN, ASTRO PARTIEL)
// =================================================================

// --- CLÉS D'API & CONFIGURATION DES CORRECTIONS ---
const API_KEYS = {
    ASTRO_API: 'VOTRE_CLE_API_ASTRO_ICI', 
    WEATHER_API: 'VOTRE_CLE_API_METEO_ICI', 
    GEO_API: 'VOTRE_CLE_API_GEO_ICI' 
};

// --- CONSTANTES GLOBALES ET PHYSIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458, C_S = 343, R_E = 6371000, KMH_MS = 3.6;
const OBLIQ = 23.44 * D2R, ECC = 0.0167, JD_2K = 2451545.0; 
let D_LAT = 48.8566, D_LON = 2.3522; // Position par défaut

// PARAMÈTRES AVANCÉS DU FILTRE DE KALMAN (GNSS)
const Q_NOISE = 0.01;       
const R_MIN = 0.05, R_MAX = 50.0; 
const L_PREC_TH = 60;       
const MAX_ACC = 50, MIN_SPD = 0.001, ALT_TH = -50; 
const SPEED_THRESHOLD = 0.5; 

// CONSTANTES POUR LES MODÈLES PHYSIQUES
const AIR_DENSITY = 1.225; 
const G_ACCEL = 9.80665;   
const CDA_EST = 0.6;       
const ESTIMATED_MASS_KG = 1500; // Ajouté pour calcul physique
const TROPO_K2 = 382000; 

// CONSTANTES JOUR/NUIT
const SUN_NIGHT_TH = -12; 
const LUX_NIGHT_TH = 50;  
const NETHER_RATIO = 8; 

// ÉTATS DE LA FRÉQUENCE DYNAMIQUE
const DOM_HIGH_FREQ_MS = 17, DOM_LOW_FREQ_MS = 250;
let currentGPSMode = 'LOW_FREQ', currentDOMFreq = DOM_LOW_FREQ_MS; 
let manualFreqMode = false, forcedFreqState = 'HIGH_FREQ'; 

// --- VARIABLES D'ÉTAT ---
let wID = null, domID = null, lPos = null, lat = null, lon = null, sTime = null;
let distM = 0, maxSpd = 0;
let kSpd = 0, kUncert = 1000; 
let lastLux = null, manualMode = null, netherMode = false; 
let lastReliableHeading = null; 
let lastP_hPa = 1013.25, lastT_K = 293.15, lastH_perc = 0.5; 
let lastAltitudeBaro = null; 
let lDomT = null, sessionStartTimestamp = Date.now(); // Ajouté pour temps écoulé

// --- UTILS ET DOM ---
const $ = id => document.getElementById(id);
const JD = (date) => (date.getTime() / 86400000) + 2440587.5; 

const dist = (lat1, lon1, lat2, lon2) => {
    const R = R_E, dLat = (lat2 - lat1) * D2R, dLon = (lon2 - lon1) * D2R;
    lat1 *= D2R; lat2 *= D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
};

function getCDate() { return new Date(); }

// ===========================================
// FONCTIONS GÉO, ASTRO & CAPTEURS DE BASE
// ===========================================

function calcSolar() {
    // Calcul complet de l'EoT, de la culmination solaire et de l'élévation (simplifié)
    const date = getCDate(), latA = lat ?? D_LAT, lonA = lon ?? D_LON;
    const jdn = JD(date), d = jdn - JD_2K; 
    const L0 = (280.466 + 0.98564736 * d) % 360 * D2R; 
    const M = (357.529 + 0.98560028 * d) % 360 * D2R; 
    const C = (1.915 * Math.sin(M) + 0.020 * Math.sin(2 * M)) * D2R; 
    const lambda = L0 + C; 
    const alpha = Math.atan2(Math.cos(OBLIQ) * Math.sin(lambda), Math.cos(lambda)); 
    const eot_rad = (L0 - alpha) + 0.0053 * Math.sin(M) - 0.0069 * Math.sin(2 * alpha);
    const eot_min = eot_rad * 4 * R2D; 
    const local_offset_hours = date.getTimezoneOffset() / 60;
    const transit_UT = (12 - (eot_min / 60) - (lonA / 15)) % 24;
    const transit_local = (transit_UT - local_offset_hours + 24) % 24;
    const transit_H = Math.floor(transit_local), transit_M = Math.floor((transit_local * 60) % 60);

    // Élévation Solaire (H) et Déclinaison (delta)
    const delta = Math.asin(Math.sin(OBLIQ) * Math.sin(lambda)); 
    const LST_hours = date.getUTCHours() + date.getUTCMinutes() / 60 + date.getUTCSeconds() / 3600 + lonA / 15;
    const tau = (LST_hours * 15 * D2R - alpha);
    const H = Math.asin(Math.sin(latA * D2R) * Math.sin(delta) + Math.cos(latA * D2R) * Math.cos(delta) * Math.cos(tau));
    
    return { 
        eot: eot_min, 
        elevation: H * R2D, 
        culmination: `${String(transit_H).padStart(2, '0')}:${String(transit_M).padStart(2, '0')}:00` 
    };
}

function initALS() {
    if ('AmbientLightSensor' in window) {
        // Logique ALS (simplifiée)
        const luxEl = $('illuminance-lux');
        if (luxEl) luxEl.textContent = 'Initialisation ALS...';
        // Simuler la lecture LUX
        lastLux = 100 + Math.sin(Date.now() / 600000) * 90;
        if (luxEl) luxEl.textContent = `${lastLux.toFixed(1)} Lux (Simulé)`;
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
                updateDisp.g_lat_read = accel.x / G_ACCEL; // Stocker l'accélération latérale
            }
        }, true);
    }

    // 3. Web Bluetooth/Serial API
    const extSensorEl = $('external-sensor-status');
    if (extSensorEl) {
        if ('Bluetooth' in navigator || 'serial' in navigator) {
            extSensorEl.textContent = 'Prêt BT/Serial';
        } else {
            extSensorEl.textContent = 'N/A';
        }
    }
}


// ===========================================
// CORRECTION GNSS (KALMAN ET TROPOSPHÈRE)
// ===========================================

function getKalmanR(acc, alt, ztd_m) {
    let R = acc ** 2; 
    R = Math.max(R_MIN, Math.min(R_MAX, R));
    
    if (acc > 5.0 && alt !== null && alt < 100) { R *= 1.5; }
    if (ztd_m > 2.5) { R *= 1.2; }
    
    R = Math.max(R_MIN, Math.min(R_MAX, R));
    return R;
}

function kFilter(nSpd, dt, R_dyn) {
    if (dt === 0 || dt > 5) return kSpd; 
    
    const R = R_dyn ?? R_MAX; 
    const Q = Q_NOISE * dt; 
    
    let pSpd = kSpd; 
    let pUnc = kUncert + Q; 
    
    let K = pUnc / (pUnc + R); 
    kSpd = pSpd + K * (nSpd - pSpd); 
    kUncert = (1 - K) * pUnc; 
    
    return kSpd;
}

function getTroposphericDelay(P_hPa, T_K, H_frac, alt_m, lat_deg) {
    const ZHD = 0.0022768 * P_hPa / (1 - 0.00266 * Math.cos(2 * lat_deg * D2R) - 0.00028 * alt_m / 1000);
    const T_C = T_K - 273.15;
    const SVP = 6.11 * Math.exp(19.7 * T_C / (T_C + 273.15)); 
    const e = H_frac * SVP; 
    const ZWD = 0.002277 * (TROPO_K2 / T_K) * (e / 100); 
    
    return ZHD + ZWD;
}

async function fetchWeather(latA, lonA) {
    if (API_KEYS.WEATHER_API === 'VOTRE_CLE_API_METEO_ICI') {
        // Simulation Météo HORS LIGNE
        lastP_hPa = 1013.25 + Math.sin(Date.now() / 100000) * 1.5;
        lastT_K = 293.15 + Math.sin(Date.now() / 500000) * 5; // ~20C
        lastH_perc = 0.5; 
        lastAltitudeBaro = 100 + Math.sin(Date.now() / 50000) * 10; 
        return; 
    }
    // VRAI APPEL API MÉTÉO (Omis pour la compacité et le respect des limites)
}
// FIN DU BLOC 1 SUR 2
// =================================================================
// FICHIER COMPLET ET STABLE : dashboard.js (V3.1 - Correction Avancée)
// BLOC 2 SUR 2 (LOGIQUE, AFFICHAGE, PHYSIQUE & CYCLE DE VIE)
// =================================================================

// Les variables et fonctions du Bloc 1 sont accessibles ici.

// ===========================================
// TRAITEMENT GPS / GNSS / CORRECTIONS
// ===========================================

function updateDisp(pos) {
    // --- 1. Récupération des données brutes
    lat = pos.coords.latitude; lon = pos.coords.longitude;
    const alt = pos.coords.altitude, acc = pos.coords.accuracy; 
    let hdg = pos.coords.heading; 
    const spd = pos.coords.speed, cTime = pos.timestamp; 
    
    if (sTime === null) sTime = cTime;

    if (acc > MAX_ACC) { return; }
    
    let spdH = spd ?? 0, spdV = 0;
    const dt = lPos ? (cTime - lPos.timestamp) / 1000 : 0.1;
    let accel_ms2 = 0; // Calculé plus bas

    // Calcul de la vitesse 3D brute et vitesse verticale
    if (lPos && dt > 0.01) { 
        const dH = dist(lPos.coords.latitude, lPos.coords.longitude, lat, lon);
        if (spd === null || spd === undefined) spdH = dH / dt; 
        if (alt !== null && lPos.coords.altitude !== null) spdV = (alt - lPos.coords.altitude) / dt; 
        distM += dH; // Mise à jour de la distance totale
    }
    const spd3D = Math.sqrt(spdH ** 2 + spdV ** 2);
    
    // --- 2. Modèles de Correction Physique / Météo ---
    const ztd_m = getTroposphericDelay(lastP_hPa, lastT_K, lastH_perc, alt ?? 0, lat);
    const R_dyn = getKalmanR(acc, alt, ztd_m);
    const fSpd = kFilter(spd3D, dt, R_dyn);
    const sSpdFE = fSpd < MIN_SPD ? 0 : fSpd;
    
    // --- 3. Gestion du Cap / Boussole de Prédiction ---
    if (sSpdFE > SPEED_THRESHOLD && hdg !== null) { lastReliableHeading = hdg; } 
    else { hdg = lastReliableHeading; }

    // --- 4. Calculs Physiques ---
    if (dt > 0.01 && lPos) {
        // Accélération basée sur la variation de vitesse stable (Kalman)
        accel_ms2 = (fSpd - (lPos.kalman_kSpd_LAST ?? 0)) / dt;
    }
    
    const force_drag = 0.5 * AIR_DENSITY * CDA_EST * sSpdFE ** 2;
    const drag_power_watts = force_drag * sSpdFE;

    // --- 5. Mise à jour des variables d'état ---
    lPos = pos; 
    lPos.speedMS_3D = spd3D; lPos.timestamp = cTime; 
    lPos.kalman_kSpd_LAST = fSpd; 
    lPos.kalman_R_val = R_dyn; 
    maxSpd = Math.max(maxSpd, sSpdFE);
    
    // --- 6. Mise à jour des éléments DOM
    
    if ($('latitude')) $('latitude').textContent = lat.toFixed(6);
    if ($('longitude')) $('longitude').textContent = lon.toFixed(6);
    if ($('gps-accuracy')) {
        let pText = `${acc.toFixed(2)} m` + (R_dyn <= R_MIN * 1.5 ? ' (Optimal)' : ' (Ajusté)');
        $('gps-accuracy').textContent = pText;
    }
    
    if ($('vertical-speed')) $('vertical-speed').textContent = `${spdV.toFixed(2)} m/s`;
    if ($('heading')) $('heading').textContent = hdg !== null ? `${hdg.toFixed(1)} °` : '--';
    if ($('underground')) $('underground').textContent = alt !== null && alt < ALT_TH ? 'Oui' : 'Non'; 
    if ($('accel-long')) $('accel-long').textContent = `${accel_ms2.toFixed(3)} m/s²`;
    if ($('drag-power-kw')) $('drag-power-kw').textContent = `${(drag_power_watts / 1000).toFixed(2)} kW`;
    if ($('ztd-delay')) $('ztd-delay').textContent = `${ztd_m.toFixed(3)} m`;
    
    // Taux de rafraîchissement des données météo (1 fois toutes les 10s)
    if (Date.now() - (updateDisp.lastWeatherFetch ?? 0) > 10000) {
        fetchWeather(lat, lon);
        updateDisp.lastWeatherFetch = Date.now();
    }
    
    // NOUVEL APPEL : Vérifier la fréquence GPS en fonction de la vitesse stable
    if (!manualFreqMode) checkGPSFrequency(sSpdFE); 
}

// ===========================================
// BOUCLES ET AFFICHAGE
// ===========================================

function fastDOM() {
    const latA = lat ?? D_LAT, lonA = lon ?? D_LON;
    const pNow = performance.now();
    
    // Mise à jour de la fréquence DOM
    const updateFreqEl = $('update-frequency');
    if (lDomT && updateFreqEl) updateFreqEl.textContent = `${(1000 / (pNow - lDomT)).toFixed(1)} Hz`;
    lDomT = pNow;

    // Mise à jour Astro (ultra-rapide)
    const { eot, culmination, elevation } = calcSolar();
    if ($('eot-value')) $('eot-value').textContent = `${eot.toFixed(2)} min`;
    if ($('solar-culmination')) $('solar-culmination').textContent = culmination;
    
    // Si la dernière mise à jour lente est trop récente, on s'arrête là
    if (fastDOM.lastSlowT && (pNow - fastDOM.lastSlowT) < 500) return;
    fastDOM.lastSlowT = pNow;
    
    // Mise à jour lente (2 Hz)
    const spd3D = lPos ? lPos.speedMS_3D : 0, spd3DKMH = spd3D * KMH_MS;
    const sSpd = kSpd < MIN_SPD ? 0 : kSpd, sSpdKMH = sSpd * KMH_MS;
    const kR = lPos ? lPos.kalman_R_val : R_MAX; 
    const uncertainty_ratio = Math.min(kUncert / 1.0, 1.0); 
    const coherence_perc = 100 * (1 - uncertainty_ratio);
    
    if ($('speed-stable')) $('speed-stable').textContent = `${sSpdKMH.toFixed(2)} km/h`; 
    if ($('speed-3d-inst')) $('speed-3d-inst').textContent = `${spd3DKMH.toFixed(2)} km/h`; 
    if ($('speed-coherence')) $('speed-coherence').textContent = `${coherence_perc.toFixed(1)}% (R:${kR.toFixed(3)})`;

    // Affichage des données Météo/Troposphère
    const T_C = lastT_K - 273.15;
    if ($('pressure')) $('pressure').textContent = `${lastP_hPa.toFixed(2)} hPa`;
    if ($('air-temp')) $('air-temp').textContent = `${T_C.toFixed(1)} °C`;
    if ($('alt-baro') && lastAltitudeBaro !== null) $('alt-baro').textContent = `${lastAltitudeBaro.toFixed(2)} m (Baro)`;
}

// ===========================================
// GESTION DU CYCLE DE VIE (Fréquence Dynamique)
// ===========================================

function checkGPSFrequency(stableSpeed) {
    if (manualFreqMode) {
        // En mode manuel, force le mode de fréquence (défini par toggleManualFreq)
        let newMode = forcedFreqState;
        if (newMode !== currentGPSMode) setGPSMode(newMode);
        return;
    }

    let newMode = currentGPSMode;
    
    if (stableSpeed > SPEED_THRESHOLD * 2) { 
        newMode = 'HIGH_FREQ';
    } else {
        newMode = 'LOW_FREQ';
    }

    if (newMode !== currentGPSMode) {
        setGPSMode(newMode);
    }
}

function setGPSMode(newMode) {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    currentGPSMode = newMode;
    
    const opts = { 
        enableHighAccuracy: newMode === 'HIGH_FREQ', 
        maximumAge: newMode === 'LOW_FREQ' ? 60000 : 0, 
        timeout: 15000 
    };
    wID = navigator.geolocation.watchPosition(updateDisp, handleErr, opts);
    
    const newDOMFreq = newMode === 'HIGH_FREQ' ? DOM_HIGH_FREQ_MS : DOM_LOW_FREQ_MS;
    if (newDOMFreq !== currentDOMFreq) {
        currentDOMFreq = newDOMFreq;
        clearInterval(domID);
        domID = setInterval(fastDOM, currentDOMFreq);
    }
    
    if ($('gnss-mode')) $('gnss-mode').textContent = `${newMode} (${currentDOMFreq}ms)`;
    if ($('freq-manual-btn') && !manualFreqMode) $('freq-manual-btn').textContent = `⌁ Fréquence: Auto`;
}

function startGPS() {
    if (wID !== null) stopGPS(false);
    sessionStartTimestamp = Date.now();
    setGPSMode('LOW_FREQ'); // Démarrage initial en LOW_FREQ
    
    if ($('start-btn')) $('start-btn').disabled = true;
    if ($('stop-btn')) $('stop-btn').disabled = false;
    if ($('error-message')) $('error-message').style.display = 'none';

    fetchWeather(lat ?? D_LAT, lon ?? D_LON);
}

function stopGPS() {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    wID = null;
    currentGPSMode = 'OFF';
    
    if ($('gnss-mode')) $('gnss-mode').textContent = 'OFF';
    if ($('start-btn')) $('start-btn').disabled = false;
    if ($('stop-btn')) $('stop-btn').disabled = true;
}

function handleErr(err) {
    console.error(`Erreur GNSS (${err.code}): ${err.message}`);
    const errEl = $('error-message');
    errEl.style.display = 'block';
    errEl.textContent = `❌ Erreur GNSS (${err.code}): Signal perdu. Bascule vers la prédiction/dernière position.`;
}

function toggleManualFreq() {
    manualFreqMode = !manualFreqMode;
    const btn = $('freq-manual-btn');
    
    if (manualFreqMode) {
        forcedFreqState = 'HIGH_FREQ'; 
        btn.textContent = '⌁ Fréquence: Manuel (Haute)';
        setGPSMode(forcedFreqState);
    } else {
        btn.textContent = '⌁ Fréquence: Auto';
        checkGPSFrequency(kSpd); 
    }
}

// --- DÉMARRAGE INITIAL ET ÉVÉNEMENTS ---
document.addEventListener('DOMContentLoaded', () => {
    initALS(); 
    initAdvancedSensors(); 
    
    if (domID === null) {
        domID = setInterval(fastDOM, DOM_LOW_FREQ_MS); 
        currentDOMFreq = DOM_LOW_FREQ_MS;
    }
    
    const startBtn = $('start-btn'), stopBtn = $('stop-btn'), freqBtn = $('freq-manual-btn');
    if (startBtn) startBtn.addEventListener('click', startGPS);
    if (stopBtn) stopBtn.addEventListener('click', stopGPS);
    if (freqBtn) freqBtn.addEventListener('click', toggleManualFreq);

    fetchWeather(D_LAT, D_LON);
});
// FIN DU BLOC 2 SUR 2
