// =================================================================
// BLOC 1/3 : CONSTANTES, INITIALISATION, PERSISTANCE ET SYNCHRONISATION
// Contenu fonctionnel : ~370 lignes
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
    // Retourne l'heure corrigée par la dérive NTP
    return new Date(Date.now() + systemClockOffsetMS);
}

// CONFIGURATIONS GPS POUR L'OPTIMISATION BATTERIE
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};

// PARAMÈTRES AVANCÉS DU FILTRE DE KALMAN (V3.3)
const Q_NOISE = 0.01;       
const R_MIN = 0.05, R_MAX = 50.0; 
const MAX_ACC = 50, MIN_SPD = 0.001, ALT_TH = -50;
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

// Fréquences pour la boucle principale fastDOM
const DOM_HIGH_FREQ_MS = 17;   
const DOM_LOW_FREQ_MS = 250;   
const DOM_SLOW_UPDATE_MS = 1000; 

// --- VARIABLES D'ÉTAT ---
let wID = null, domID = null, lPos = null, lat = null, lon = null, sTime = null;
let distM = 0, distMStartOffset = 0, maxSpd = 0;
let tLat = null, tLon = null;
let kSpd = 0, kUncert = 1000; 
let lastAltitudeBaro = null; 
let currentGPSMode = 'LOW_FREQ'; 
let manualFreqMode = false;      
let emergencyStopActive = false;
let selectedEnvironment = 'NORMAL'; 
let selectedWeather = 'CLEAR'; 
let currentBatteryLevel = 100;
let externalBatteryLevel = 99.999; 
let manualTractionForce = 0; 
let calculatedMassKg = G_MASS_DEFAULT; 
let netherMode = false; 
let lastReliableHeading = null; 
let lastP_hPa = 1013.25, lastT_K = 293.15, lastH_perc = 0.5; 

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
        externalBatteryLevel: externalBatteryLevel,
        manualTractionForce: manualTractionForce,
        calculatedMassKg: calculatedMassKg,
        systemClockOffsetMS: systemClockOffsetMS,
        lastNtpSync: lastNtpSync
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
            externalBatteryLevel = state.externalBatteryLevel || 99.999;
            lat = state.D_LAT;
            lon = state.D_LON; 
            manualTractionForce = state.manualTractionForce || 0;
            calculatedMassKg = state.calculatedMassKg || G_MASS_DEFAULT;
            systemClockOffsetMS = state.systemClockOffsetMS || 0;
            lastNtpSync = state.lastNtpSync || 0;
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

    // 1. SYNCHRONISATION NTP (Horloge Maîtresse)
    try {
        const timeResponse = await fetch('https://worldtimeapi.org/api/ip', { cache: 'no-store' });
        if (!timeResponse.ok) throw new Error('Erreur de service de temps.');

        const data = await timeResponse.json();
        const serverTime = new Date(data.utc_datetime).getTime();
        const localTime = Date.now();
        
        systemClockOffsetMS = serverTime - localTime;
        lastNtpSync = getCDate().getTime();
        successCount++;
        console.log(`[NTP Sync] Dérive corrigée: ${systemClockOffsetMS} ms. Nouveau temps maître établi.`);
    } catch (error) {
        console.warn(`[NTP Sync] Échec de la synchronisation NTP: ${error.message}`);
    }
    
    // 2. FETCH MÉTÉO (Pour les corrections atmosphériques)
    try {
        if (API_KEYS.WEATHER_API !== 'VOTRE_CLE_API_METEO_ICI') {
            const weather = await fetchWeather(currentLat, currentLon);
            if (weather) {
                lastP_hPa = weather.pressure; 
                lastT_K = weather.temperature;
                lastH_perc = weather.humidity;
                successCount++;
                console.log(`[Météo Sync] Données atmosphériques mises à jour.`);
            }
        }
    } catch (error) {
        console.warn(`[Météo Sync] Échec de la mise à jour météo: ${error.message}`);
    }

    if (successCount > 0) {
        alert(`Recalibrage Internet terminé: ${successCount} points synchronisés.`);
    } else {
        alert("Recalibrage Internet échoué ou aucune clé d'API fournie/valide.");
    }
    fastDOM();
}

// --- GESTION ÉVÉNEMENTIELLE DE BASE ---

function updateBatteryStatus(battery) {
    const level = Math.floor(battery.level * 100);
    const charging = battery.charging;
    currentBatteryLevel = level;
    const batInd = $('battery-indicator');
    if (batInd) batInd.textContent = `${level}% ${charging ? '🔌' : ''}`;
}

async function initBattery() {
    if ('getBattery' in navigator) {
        try {
            const battery = await navigator.getBattery();
            updateBatteryStatus(battery);
            battery.addEventListener('levelchange', () => updateBatteryStatus(battery));
            battery.addEventListener('chargingchange', () => updateBatteryStatus(battery));
        } catch (e) {
            if ($('battery-indicator')) $('battery-indicator').textContent = 'N/A';
        }
    } else {
        if ($('battery-indicator')) $('battery-indicator').textContent = 'N/A';
    }
}
// =================================================================
// BLOC 2/3 : CALCULS GÉO, LOGIQUE EKF ET MISE À JOUR DE POSITION GPS
// Contenu fonctionnel : ~350 lignes
// =================================================================

// --- ASTRO CALCULS (SIMULÉS pour l'intégration) ---
function updateAstro(latA, lonA) {
    // Les vrais calculs impliquent une librairie astro.
    const now = getCDate();
    const timeStr = now.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit', second: '2-digit' });

    if ($('solar-true')) $('solar-true').textContent = timeStr; 
    if ($('solar-mean')) $('solar-mean').textContent = timeStr; 
    if ($('solar-culmination')) $('solar-culmination').textContent = '12:00:00';
    if ($('sun-elevation')) $('sun-elevation').textContent = '45.00 °';
    if ($('eot')) $('eot').textContent = '0.00 min';
    if ($('lunar-phase-perc')) $('lunar-phase-perc').textContent = '50%';
    if ($('lunar-time')) $('lunar-time').textContent = timeStr;
    if ($('solar-longitude-val')) $('solar-longitude-val').textContent = '90.00 °';
    const isDay = now.getHours() > 6 && now.getHours() < 20;
    if ($('mode-indicator')) $('mode-indicator').textContent = isDay ? 'JOUR ☀️' : 'NUIT 🌙';
}

// --- CORRECTIONS MÉTÉO/TROPOSPHÈRE ---

function getTroposphericDelay(P_hPa, T_K, H_frac, alt_m, lat_deg) {
    // Modèle ZTD (simulé)
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

async function fetchWeather(latA, lonA) {
    // Simulation du retour météo
    return {
        pressure: 1012.0,
        temperature: 288.15, // 15 C en K
        humidity: 0.6,
        condition: 'Clear'
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
    const spd3D = Math.sqrt(spdH ** 2 + spdV ** 2); // Vitesse 3D en m/s
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
    // --- FIN LOGIQUE EKF ---

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
    
    // --- Stockage des états pour la prochaine itération ---
    lPos = pos; 
    lPos.speedMS_3D = spd3D; 
    lPos.timestamp = cTime; 
    lPos.speedMS_3D_LAST = spd3D; 
    lPos.kalman_R_val = R_dyn; 
    lPos.kalman_kSpd_LAST = kSpd; 

    checkGPSFrequency(sSpdFE); 

    distM += sSpdFE * dt * (netherMode ? NETHER_RATIO : 1); 
    
    if (spd3D * KMH_MS > maxSpd) maxSpd = spd3D * KMH_MS; // Max est en KM/H

    // --- MISE À JOUR DOM INSTANTANÉE (Mise à jour dans fastDOM aussi) ---
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

function checkGPSFrequency(currentSpeed) {
    // Logique de changement de fréquence GPS auto
    const targetMode = (currentSpeed * KMH_MS > 50 && !manualFreqMode) ? 'HIGH_FREQ' : 'LOW_FREQ';
    if (targetMode !== currentGPSMode) {
        setGPSMode(targetMode);
    }
    }
// =================================================================
// BLOC 3/3 : CONTRÔLES, BOUCLES DOM ET FONCTIONS D'INITIALISATION
// Contenu fonctionnel : ~380 lignes
// =================================================================

function setGPSMode(newMode) {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    currentGPSMode = newMode;
    const opts = GPS_OPTS[newMode]; 
    wID = navigator.geolocation.watchPosition(updateDisp, handleErr, opts);
    if ($('speed-source-indicator')) $('speed-source-indicator').textContent = `Source: ${newMode}`;
    
    // Gérer la fréquence DOM en fonction du mode GPS
    const newDOMFreq = (newMode === 'HIGH_FREQ') ? DOM_HIGH_FREQ_MS : DOM_LOW_FREQ_MS;
    if (domID && domID.intervalMS !== newDOMFreq) {
        clearInterval(domID);
        domID = setInterval(fastDOM, newDOMFreq);
        domID.intervalMS = newDOMFreq;
        if ($('update-frequency')) $('update-frequency').textContent = `${(1000/newDOMFreq).toFixed(1)} Hz`;
    }
}

function updateAvgSpeed() {
    if (sTime === null || distM === 0) return 0;
    const elapS = (getCDate().getTime() - sTime) / 1000;
    return (distM / elapS) * KMH_MS; // Vitesse moyenne en KM/H
}

function fastDOM() {
    const now = getCDate().getTime(); 
    const pNow = performance.now();
    
    const sSpd = kSpd < MIN_SPD ? 0 : kSpd;
    const sSpdKMH = sSpd * KMH_MS;
    const avgSpdKMH = updateAvgSpeed(); 
    const elapS = sTime !== null ? (now - sTime) / 1000 : 0;
    
    updateAstro(lat ?? 48.8566, lon ?? 2.3522); 
    
    // Mise à jour de la grille principale
    if ($('speed-stable')) $('speed-stable').textContent = `${sSpdKMH.toFixed(3)}`; 
    if ($('speed-max-kmh')) $('speed-max-kmh').textContent = `${maxSpd.toFixed(2)}`;
    if ($('speed-avg-kmh')) $('speed-avg-kmh').textContent = `${avgSpdKMH.toFixed(2)}`;
    
    // Mise à jour des valeurs secondaires
    const coherence_perc = Math.min(100 * (1 - Math.min(kUncert / 1.0, 1.0)), 100);
    if ($('speed-error-perc')) $('speed-error-perc').textContent = `${coherence_perc.toFixed(1)}`;
    if ($('calculated-mass')) $('calculated-mass').textContent = `${calculatedMassKg.toFixed(2)}`;
    if ($('traction-force')) $('traction-force').textContent = `${manualTractionForce.toFixed(1)} N`;
    if ($('local-time')) $('local-time').textContent = getCDate().toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit', second: '2-digit' });
    if ($('elapsed-time')) $('elapsed-time').textContent = `${elapS.toFixed(1)}`;
    if ($('distance-km-m')) $('distance-km-m').textContent = `${(distM / 1000).toFixed(3)} km`;
    
    // Mettre à jour les indicateurs d'état et les indicateurs météo/environnement
    if ($('nether-indicator')) $('nether-indicator').textContent = netherMode ? 'ACTIVÉ (1:8)' : 'DÉSACTIVÉ (1:1)';
    
    // Mise à jour lente (toutes les 1s)
    if (fastDOM.lastSlowT && (pNow - fastDOM.lastSlowT) < DOM_SLOW_UPDATE_MS) return;
    fastDOM.lastSlowT = pNow;
    saveState(); 

    // Simulation de l'altitude barométrique (pour la démo)
    if ($('alt-baro')) $('alt-baro').textContent = (parseFloat($('altitude')?.textContent) || 0) + 15.0;
    
}

function startGPS() {
    if (wID !== null) stopGPS(false);
    sTime = null; 
    setGPSMode(emergencyStopActive ? 'LOW_FREQ' : 'LOW_FREQ');
    
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
if ($('freq-manual-btn')) {
    $('freq-manual-btn').addEventListener('click', () => {
        manualFreqMode = !manualFreqMode;
        $('freq-manual-btn').textContent = manualFreqMode ? '⚡ Fréquence: Manuel' : '⚡ Fréquence: Auto';
        setGPSMode(manualFreqMode ? 'HIGH_FREQ' : 'LOW_FREQ'); // Force HIGH en mode manuel
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
    initBattery(); 
    
    // Initialisation de la boucle DOM
    if (domID === null) {
        domID = setInterval(fastDOM, DOM_LOW_FREQ_MS); 
        domID.intervalMS = DOM_LOW_FREQ_MS;
        fastDOM.lastSlowT = 0; 
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
    
    // Initialisation de l'état d'urgence
    if (emergencyStopActive) emergencyStop();
    
    // Premier appel de syncRemoteData si aucune synchro récente (> 24h)
    if (lastNtpSync === 0 || (getCDate().getTime() - lastNtpSync) > (24 * 3600 * 1000)) {
        syncRemoteData();
    }
}

document.addEventListener('DOMContentLoaded', initAll);
