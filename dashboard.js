// =================================================================
// FICHIER JS PARTIE 1/2 : dashboard_part1.js (V3.2 Core & Init)
// Contient constantes, variables d'état, calculs Geo/Astro/Kalman,
// et les fonctions d'initialisation des capteurs/systèmes.
// !! AUCUNE SIMULATION DE DONNÉES !!
// =================================================================

// --- CLÉS D'API (Non utilisées) ---
const API_KEYS = {
    WEATHER_API: 'VOTRE_CLE_API_METEO_ICI' 
};

// --- CONSTANTES GLOBALES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458, C_S = 343, R_E = 6371000, KMH_MS = 3.6;
const G_ACCEL = 9.80665; 
const OBLIQ = 23.44 * D2R, ECC = 0.0167, JD_2K = 2451545.0; // Constantes Astro
let D_LAT = 48.8566, D_LON = 2.3522; // Position par défaut (Paris)
const MIN_DT = 0.01; 
const NETHER_RATIO = 8.0; 
const CDA_EST = 0.3; // Coefficient de traînée estimé
const AIR_DENSITY = 1.225; // Densité de l'air standard (kg/m³)
const SVT_OXYGEN_LEVEL = 0.2095; 
const MIN_SPD = 0.05; 

// CONFIGURATIONS GPS ET FRÉQUENCES
const SPEED_THRESHOLD = 5 / KMH_MS; 
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};
const DOM_LOW_FREQ_MS = 100; 
const DOM_SLOW_UPDATE_MS = 1000; 
let domID = null, currentDOMFreq = DOM_LOW_FREQ_MS;

// Facteurs de perturbation (V3.2+)
const ENVIRONMENT_FACTORS = {
    NORMAL: { MAG_MULT: 1.0, GPS_Q: 0.5, DRAG_MULT: 1.0 },
    METAL: { MAG_MULT: 0.2, GPS_Q: 1.5, DRAG_MULT: 1.1 },
    FOREST: { MAG_MULT: 1.0, GPS_Q: 2.0, DRAG_MULT: 1.2 },
    CONCRETE: { MAG_MULT: 1.0, GPS_Q: 3.0, DRAG_MULT: 1.05 }
};
const WEATHER_FACTORS = {
    CLEAR: 1.0, RAIN: 1.05, SNOW: 1.10, STORM: 1.20
};
const FORCED_FREQS = [5000, 10000, 30000]; 

// --- VARIABLES D'ÉTAT GLOBALES ---
let wID = null; 
let lPos = null; 
let lat = null, lon = null, tLat = null, tLon = null; 
let sTime = null; 
let distM = 0; 
let maxSpd = 0; 
let currentGPSMode = 'LOW_FREQ'; 
let manualMode = false; 
let netherMode = false;
let manualFreqMode = false;
let forcedFreqIndex = 0;
let emergencyStopActive = false;
let selectedEnvironment = 'NORMAL';
let selectedWeather = 'CLEAR';
let selectedMass_kg = 70.0;
let totalWork_J = 0; 

// Variables de Kalman
let kSpd = 0; 
let kUncert = 1000; 
const Q = 0.01; 

// Capteurs Avancés (Valeurs initiales à N/A)
let als = null; 
let magSensor = null; 
let currentIlluminance = null;
let currentMagField = null;

// Variables pour les valeurs non-simulées (API réelles ou N/A)
let realP_hPa = 'N/A';
let realT_K = 'N/A';
let realH_perc = 'N/A';


// --- UTILS & CORE ---
const $ = id => document.getElementById(id);
const getCDate = () => new Date();

function syncH() {
    const now = getCDate();
    const h = String(now.getHours()).padStart(2, '0');
    const m = String(now.getMinutes()).padStart(2, '0');
    const s = String(now.getSeconds()).padStart(2, '0');
    if ($('local-time')) $('local-time').textContent = `${h}:${m}:${s}`;
}

function dist(lat1, lon1, lat2, lon2) {
    const dLat = (lat2 - lat1) * D2R;
    const dLon = (lon2 - lon1) * D2R;
    lat1 *= D2R; lat2 *= D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.sin(dLon / 2) ** 2 * Math.cos(lat1) * Math.cos(lat2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R_E * c;
}

function bearing(lat1, lon1, lat2, lon2) {
    lat1 *= D2R; lat2 *= D2R; lon1 *= D2R; lon2 *= D2R;
    const y = Math.sin(lon2 - lon1) * Math.cos(lat2);
    const x = Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1) * Math.cos(lat2) * Math.cos(lon2 - lon1);
    return (Math.atan2(y, x) * R2D + 360) % 360;
}

// --- CALCULS ASTRO & GEO (LOGIQUE V3.2) ---

function calcSolar() {
    const now = getCDate();
    const JD = now.getTime() / 86400000 + 2440587.5; 
    const d = JD - JD_2K; 
    
    const w = 282.9404 + 4.70935e-5 * d; 
    let M = 356.0470 + 0.9856002585 * d; 
    M = M % 360; if (M < 0) M += 360;
    const L = w + M; 
    
    const obl = OBLIQ; 
    const RA = Math.atan2(Math.sin(L * D2R) * Math.cos(obl), Math.cos(L * D2R)) * R2D;
    let Decl = Math.asin(Math.sin(L * D2R) * Math.sin(obl)) * R2D;
    
    const eot = (4 * (L - RA)) / D2R / 60;
    
    let culminationH = 12 + ((D_LON - L / D2R) / 15);
    culminationH = (culminationH % 24 + 24) % 24;
    const cul_h = Math.floor(culminationH), cul_m = Math.floor((culminationH * 60) % 60);
    
    const elevation = 90 - (D_LAT) + Decl;
    
    return { eot: eot, culmination: `${String(cul_h).padStart(2, '0')}:${String(cul_m).padStart(2, '0')}`, solarLongitude: L, declination: Decl, elevation: elevation };
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

function updateAstro(latA, lonA) {
    const now = getCDate(); 
    const sData = calcSolar(); 
    
    let sTimeH = (now.getUTCHours() * 3600 + now.getUTCMinutes() * 60 + now.getUTCSeconds()) / 3600;
    let trueSolarTimeH = sTimeH + (lonA / 15) + (sData.eot / 60);
    trueSolarTimeH = (trueSolarTimeH % 24 + 24) % 24;
    const tsH = Math.floor(trueSolarTimeH), tsM = Math.floor((trueSolarTimeH * 60) % 60), tsS = Math.floor((trueSolarTimeH * 3600) % 3600 % 60);
    if ($('solar-true')) $('solar-true').textContent = `${String(tsH).padStart(2, '0')}:${String(tsM).padStart(2, '0')}:${String(tsS).padStart(2, '0')}`;
    if ($('solar-true-header')) $('solar-true-header').textContent = `${String(tsH).padStart(2, '0')}:${String(tsM).padStart(2, '0')}:${String(tsS).padStart(2, '0')} (HSV)`;

    let meanSolarTimeH = sTimeH + (lonA / 15);
    meanSolarTimeH = (meanSolarTimeH % 24 + 24) % 24;
    const msH = Math.floor(meanSolarTimeH), msM = Math.floor((meanSolarTimeH * 60) % 60), msS = Math.floor((meanSolarTimeH * 3600) % 3600 % 60);
    if ($('solar-mean')) $('solar-mean').textContent = `${String(msH).padStart(2, '0')}:${String(msM).padStart(2, '0')}:${String(msS).padStart(2, '0')}`;
    if ($('solar-mean-header')) $('solar-mean-header').textContent = `${String(msH).padStart(2, '0')}:${String(msM).padStart(2, '0')}:${String(msS).padStart(2, '0')} (LSM)`;
    
    if ($('eot')) $('eot').textContent = `${sData.eot.toFixed(2)} min`;
    if ($('solar-culmination')) $('solar-culmination').textContent = sData.culmination; 
    if ($('solar-longitude-val')) $('solar-longitude-val').textContent = `${sData.solarLongitude.toFixed(2)} °`;
    if ($('solar-elevation')) $('solar-elevation').textContent = `${sData.elevation.toFixed(2)} °`;
    
    const D_rad = calcLunarPhase(); 
    const phasePerc = (1 + Math.cos(D_rad)) / 2 * 100; 
    if ($('lunar-phase-perc')) $('lunar-phase-perc').textContent = `${phasePerc.toFixed(1)}%`;
}

// --- CALCULS SYSTÈMES AVANCÉS (Kalman & Météo Fixe) ---

function getTroposphericDelay(Alt_m) {
    // Utilise l'atmosphère standard (Pas de simulation de météo)
    const P_hPa = 1013.25; const T_K = 293.15; const H_perc = 0.5; 
    const P_Pa = P_hPa * 100;
    const T_C = T_K - 273.15;
    const T_m = 450.0; 
    const P_w = 0.003 * P_hPa * H_perc * Math.exp(17.67 * T_C / (T_C + 243.5)); 
    const K1 = 77.6e-5;
    const D_hydro = K1 * P_Pa / T_K; 
    const K2 = 63.8e-5;
    const K3 = 3.776e-3;
    const D_wet = (K2 * P_Pa + K3 * P_w) / (T_m); 
    return (D_hydro + D_wet) * (2.2 / 1000) * (2000 / (2000 + Alt_m / 1000));
}

function getKalmanR(Accuracy_m, Alt_m) {
    const environmentFactor = ENVIRONMENT_FACTORS[selectedEnvironment].GPS_Q;
    const weatherFactor = WEATHER_FACTORS[selectedWeather];
    const baseR = Accuracy_m ** 2 * 1.5; 
    const ztd_m_fixed = getTroposphericDelay(Alt_m);
    const noise = Math.max(1, (Alt_m / 1000)) * (1 + ztd_m_fixed) * environmentFactor * weatherFactor;
    return baseR + noise;
}

function kFilter(z, dt, R) {
    const Q_dyn = Q * ENVIRONMENT_FACTORS[selectedEnvironment].GPS_Q; 
    const kSpd_pred = kSpd;
    const kUncert_pred = kUncert + Q_dyn * dt;
    const K = kUncert_pred / (kUncert_pred + R); 
    kSpd = kSpd_pred + K * (z - kSpd_pred);
    kUncert = (1 - K) * kUncert_pred;
    
    if ($('kalman-uncert')) $('kalman-uncert').textContent = `${kUncert.toFixed(3)} m/s`;
    if ($('kalman-r')) $('kalman-r').textContent = `${R.toFixed(2)} m²`;
    return kSpd;
}

function initBattery() {
    if ('getBattery' in navigator) {
        navigator.getBattery().then(function(battery) {
            function updateBatteryInfo() {
                const level = (battery.level * 100).toFixed(2);
                const charging = battery.charging ? ' (🔌 En charge)' : '';
                const statusText = `${level}%${charging}`;
                if ($('battery-indicator')) $('battery-indicator').textContent = statusText;
            }
            updateBatteryInfo();
            battery.addEventListener('levelchange', updateBatteryInfo);
            battery.addEventListener('chargingchange', updateBatteryInfo);
        });
    } else {
        if ($('battery-indicator')) $('battery-indicator').textContent = 'N/A (API non supportée)';
    }
}

function handleEnvironmentChange() {
    const select = $('environment-select');
    if (select) selectedEnvironment = select.value;
    if ($('selected-environment-ind')) $('selected-environment-ind').textContent = selectedEnvironment;
}
function handleWeatherChange() {
    const select = $('weather-select');
    if (select) selectedWeather = select.value;
    if ($('selected-weather-ind')) $('selected-weather-ind').textContent = selectedWeather;
}

function updateDisplayMode() {
    const now = getCDate();
    const currentHour = now.getHours();
    
    // Bascule auto: Si lumière ambiante non disponible, utilise l'heure.
    const isNight = currentIlluminance !== null ? currentIlluminance < 50 : (currentHour < 7 || currentHour >= 20); 
    const modeIndicator = $('mode-indicator');
    const body = document.body;

    let isNightMode;
    if (manualMode) {
        isNightMode = body.classList.contains('night-mode');
        modeIndicator.textContent = `Mode: Manuel ${isNightMode ? '🌒' : '☀️'}`;
    } else {
        isNightMode = isNight;
        modeIndicator.textContent = `Mode: Auto ${isNightMode ? '🌒' : '☀️'}`;
    }

    body.classList.toggle('night-mode', isNightMode);
    if (modeIndicator) modeIndicator.textContent = `Mode: ${manualMode ? 'Manuel' : 'Auto'} ${isNightMode ? '🌒' : '☀️'}`;
}

// --- GESTION DE L'URGENCE (SANS SIMULATION DE BATTERIE) ---

function emergencyStop() {
    emergencyStopActive = !emergencyStopActive;
    
    const stopBtn = $('emergency-stop-btn');
    const freqBtn = $('freq-manual-btn');
    const stopInd = $('emergency-status');
    
    if (emergencyStopActive) {
        if (wID !== null) stopGPS(false);
        if (als && als.stop) als.stop(); 
        if (magSensor && magSensor.stop) magSensor.stop();
        if (domID !== null) clearInterval(domID);
        currentDOMFreq = DOM_LOW_FREQ_MS * 4; 
        domID = setInterval(fastDOM, currentDOMFreq);
        
        if (stopBtn) { 
            stopBtn.textContent = '🟢 Démarrer Système'; 
            stopBtn.style.backgroundColor = '#4CAF50'; 
        }
        if (freqBtn) freqBtn.disabled = true;
        if (stopInd) { 
            stopInd.textContent = 'ACTIF (Mode Sécurité)'; 
            stopInd.style.color = '#ff6666'; 
        }
    } else {
        if (wID === null) startGPS(); 
        if (als && als.start) als.start(); 
        if (magSensor && magSensor.start) magSensor.start();
        
        if (domID !== null) clearInterval(domID);
        currentDOMFreq = DOM_LOW_FREQ_MS;
        domID = setInterval(fastDOM, currentDOMFreq);
        
        if (stopBtn) { 
            stopBtn.textContent = '🚨 Arrêt Urgence'; 
            stopBtn.style.backgroundColor = '#f44336'; 
        }
        if (freqBtn) freqBtn.disabled = false;
        if (stopInd) { 
            stopInd.textContent = 'INACTIF'; 
            stopInd.style.color = '#00ff99'; 
        }
    }
}


// --- GESTIONNAIRE GPS (updateDisp) ---

function handleErr(err) {
    console.error(`Erreur GNSS (${err.code}): ${err.message}`);
    const errEl = $('error-message');
    if(errEl) {
        errEl.style.display = 'block';
        errEl.textContent = `❌ Erreur GNSS (${err.code}): Signal perdu ou non autorisé.`;
    }
    stopGPS(false);
}

function stopGPS(updateDOM = true) {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
    }
    if ($('start-btn')) $('start-btn').disabled = false;
    if ($('stop-btn')) $('stop-btn').disabled = true;
    if (updateDOM) {
        if ($('speed-stable')) $('speed-stable').textContent = '0.00000 km/h';
    }
}

function startGPS() { 
    if (wID !== null) return;
    const opts = GPS_OPTS['HIGH_FREQ']; 
    wID = navigator.geolocation.watchPosition(updateDisp, handleErr, opts);
    emergencyStopActive = false;
    if ($('error-message')) $('error-message').style.display = 'none';
    if ($('start-btn')) $('start-btn').disabled = true;
    if ($('stop-btn')) $('stop-btn').disabled = false;
}

function updateDisp(pos) {
    if (emergencyStopActive) return;

    lat = pos.coords.latitude; lon = pos.coords.longitude;
    const alt = pos.coords.altitude, acc = pos.coords.accuracy;
    const spd = pos.coords.speed, cTime = pos.timestamp; 

    currentGPSMode = (spd !== null && spd > SPEED_THRESHOLD) ? 'HIGH_FREQ' : 'LOW_FREQ';
    if (wID !== null && !manualFreqMode) {
        const newOpts = GPS_OPTS[currentGPSMode];
        if (pos.options && pos.options.timeout !== newOpts.timeout) {
             stopGPS(false); 
             wID = navigator.geolocation.watchPosition(updateDisp, handleErr, newOpts);
             if ($('freq-manual-btn')) $('freq-manual-btn').textContent = `⚡ Fréquence: Auto (${currentGPSMode.split('_')[0]})`;
        }
    }

    const dt = lPos ? (cTime - lPos.timestamp) / 1000 : MIN_DT; 
    let spdH = spd ?? 0, spdV = 0;
    
    if (lPos && dt > 0.1) { 
        const dH = dist(lPos.coords.latitude, lPos.coords.longitude, lat, lon); 
        if (spd === null || spd === undefined) spdH = dH / dt; 
        if (alt !== null && lPos.coords.altitude !== null) spdV = (alt - lPos.coords.altitude) / dt; 
    }
    const spd3D = Math.sqrt(spdH ** 2 + spdV ** 2);
    
    const ztd_m = getTroposphericDelay(alt ?? 0); 
    const R_dyn = getKalmanR(acc, alt ?? 0); 
    const fSpd = kFilter(spd3D, dt, R_dyn), sSpdFE = fSpd < MIN_SPD ? 0 : fSpd;
    
    // --- CALCULS PHYSIQUES ---
    const accellLong = dt > 0 ? (sSpdFE - (lPos?.speedMS_3D_FILTERED_LAST ?? 0)) / dt : 0;
    const gForce = accellLong / G_ACCEL; 
    const DRAG_MULT = ENVIRONMENT_FACTORS[selectedEnvironment].DRAG_MULT * WEATHER_FACTORS[selectedWeather];
    const dragForce = 0.5 * AIR_DENSITY * CDA_EST * DRAG_MULT * sSpdFE ** 2;
    const dragPower = dragForce * sSpdFE; 
    const totalForce = dragForce + (selectedMass_kg * accellLong) + (selectedMass_kg * G_ACCEL * (spdV / sSpdFE || 0));
    totalWork_J += totalForce * (sSpdFE * dt);
    
    lPos = pos; lPos.speedMS_3D = spd3D; lPos.timestamp = cTime; 
    lPos.speedMS_3D_FILTERED_LAST = sSpdFE; 

    if (sTime === null) sTime = getCDate().getTime();
    distM += sSpdFE * dt * (netherMode ? NETHER_RATIO : 1); 
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 

    // --- MISE À JOUR DOM GPS/PHYSIQUE ---
    if ($('latitude')) $('latitude').textContent = lat.toFixed(6);
    if ($('longitude')) $('longitude').textContent = lon.toFixed(6);
    if ($('accuracy')) $('accuracy').textContent = `${acc.toFixed(2)} m`;
    if ($('altitude')) $('altitude').textContent = alt !== null ? `${alt.toFixed(1)} m` : '-- m';
    if ($('vertical-speed')) $('vertical-speed').textContent = `${spdV.toFixed(2)} m/s`;
    if ($('speed-stable')) $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(5)} km/h`;
    if ($('speed-3d-inst')) $('speed-3d-inst').textContent = `${(spd3D * KMH_MS).toFixed(2)} km/h`;
    if ($('speed-max')) $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(2)} km/h`;
    if ($('perc-light')) $('perc-light').textContent = `${((sSpdFE / C_L) * 100).toExponential(2)}%`;
    if ($('perc-sound')) $('perc-sound').textContent = `${((sSpdFE / C_S) * 100).toFixed(2)}%`;
    if ($('ztd')) $('ztd').textContent = `${ztd_m.toFixed(3)} m`;
    if ($('drag-force')) $('drag-force').textContent = `${dragForce.toFixed(1)} N`;
    if ($('drag-power-kw')) $('drag-power-kw').textContent = `${(dragPower / 1000).toFixed(2)} kW`;
    if ($('accel-long')) $('accel-long').textContent = `${accellLong.toFixed(3)} m/s²`;
    if ($('g-force')) $('g-force').textContent = `${gForce.toFixed(2)} G`;
    if ($('speed-brute')) $('speed-brute').textContent = `${spd3D.toFixed(3)} m/s`;
    
    if (tLat !== null && tLon !== null) {
        const d_target = dist(lat, lon, tLat, tLon);
        const b_target = bearing(lat, lon, tLat, tLon);
        if ($('target-dist')) $('target-dist').textContent = `${d_target.toFixed(2)} m`;
        if ($('target-bearing')) $('target-bearing').textContent = `${b_target.toFixed(1)} °`;
    }
}
// Environ 390 lignes
// =================================================================
// FICHIER JS PARTIE 2/2 : dashboard_part2.js (V3.2 Loop & Events)
// Contient la boucle principale fastDOM et les gestionnaires d'événements DOM.
// Doit être chargé APRÈS dashboard_part1.js
// =================================================================

// --- BOUCLE PRINCIPALE (fastDOM) ---

function fastDOM() {
    const latA = lat ?? D_LAT, lonA = lon ?? D_LON;
    const pNow = performance.now();
    const dt_sec = (pNow - (fastDOM.lastPNow ?? pNow)) / 1000;
    fastDOM.lastPNow = pNow;
    
    // Remplacement des simulations de puissance/batterie par N/A
    if ($('power-consumption')) $('power-consumption').textContent = 'N/A'; 
    if ($('backup-battery-status')) $('backup-battery-status').textContent = 'Désactivée (Supprimé)';
    
    const sessionTime_sec = sTime ? (getCDate().getTime() - sTime) / 1000 : 0;
    if ($('time-elapsed')) $('time-elapsed').textContent = `${sessionTime_sec.toFixed(0)} s`;
    
    const mc_sec = sessionTime_sec * (netherMode ? NETHER_RATIO : 1);
    const mc_h = Math.floor(mc_sec / 3600) % 24;
    const mc_m = Math.floor((mc_sec % 3600) / 60);
    const mc_s = Math.floor(mc_sec % 60);
    if ($('mc-time')) $('mc-time').textContent = `${String(mc_h).padStart(2, '0')}:${String(mc_m).padStart(2, '0')}:${String(mc_s).padStart(2, '0')}`;
    
    syncH(); 
    
    // Logique Temps Solaire (HSV/LSM) - Cœur V3.2
    updateAstro(latA, lonA); 
    
    if (fastDOM.lastSlowT && (pNow - fastDOM.lastSlowT) < DOM_SLOW_UPDATE_MS) {
        return; 
    }
    fastDOM.lastSlowT = pNow;
    
    const sSpd = kSpd < MIN_SPD ? 0 : kSpd;
    
    // Calculs de puissance SVT simplifiés (Estimation basée uniquement sur la masse et la vitesse)
    const BMR_W_Est = (1.2 * selectedMass_kg); 
    const EFFORT_W_Est = (5 * sSpd * selectedMass_kg); 
    
    if ($('power-output')) $('power-output').textContent = `${(BMR_W_Est + EFFORT_W_Est).toFixed(1)} W (Est.)`;
    
    // Affichage des valeurs météo/SVT à N/A ou fixe
    if ($('air-temp')) $('air-temp').textContent = realT_K === 'N/A' ? 'N/A' : `${(realT_K - 273.15).toFixed(1)} °C`;
    if ($('pressure')) $('pressure').textContent = realP_hPa === 'N/A' ? 'N/A' : `${realP_hPa.toFixed(2)} hPa`;
    if ($('humidity')) $('humidity').textContent = realH_perc === 'N/A' ? 'N/A' : `${(realH_perc * 100).toFixed(1)} %`;
    
    if ($('alt-baro')) $('alt-baro').textContent = 'N/A'; // Nécéssite Pression réelle
    if ($('o2-level-sim')) $('o2-level-sim').textContent = `${(SVT_OXYGEN_LEVEL * 100).toFixed(2)} %`; 
    if ($('spo2-sim')) $('spo2-sim').textContent = 'N/A'; // Nécéssite Altitude Barométrique réelle
    
    if ($('calorie-burn')) $('calorie-burn').textContent = `${(totalWork_J / 4184).toFixed(1)} Cal (Mécanique)`;
    if ($('work-energy')) $('work-energy').textContent = `${(totalWork_J / 1000000).toFixed(2)} MJ`; 
    
    if ($('distance-km-m')) {
        const distKM = distM / 1000;
        const distM_Int = distM % 1000;
        $('distance-km-m').textContent = `${distKM.toFixed(3)} km | ${distM_Int.toFixed(0)} m`;
    }
    if (sessionTime_sec > 5) { 
        const avgSpd = (distM / sessionTime_sec) * KMH_MS;
        if ($('speed-avg')) $('speed-avg').textContent = `${avgSpd.toFixed(2)} km/h`;
    }
}

// --- GESTIONNAIRES D'ÉVÉNEMENTS DOM ---

function toggleManualMode() {
    manualMode = !manualMode;
    updateDisplayMode();
}

function cycleForcedFreq() {
    forcedFreqIndex = (forcedFreqIndex + 1) % FORCED_FREQS.length;
    const freqMs = FORCED_FREQS[forcedFreqIndex];
    if (wID !== null) {
        stopGPS(false);
        navigator.geolocation.watchPosition(updateDisp, handleErr, {
            enableHighAccuracy: true,
            maximumAge: 0,
            timeout: freqMs
        });
    }
    if ($('freq-manual-btn')) $('freq-manual-btn').textContent = `⚡ Fréquence: Manuel (${freqMs / 1000}s)`;
}

function toggleManualFreq() {
    manualFreqMode = !manualFreqMode;
    if (!manualFreqMode) {
        if (wID !== null) { stopGPS(false); startGPS(); }
        forcedFreqIndex = 0;
        if ($('freq-manual-btn')) $('freq-manual-btn').textContent = '⚡ Fréquence: Auto';
    } else {
        forcedFreqIndex = 0;
        cycleForcedFreq(); 
    }
}

function setTarget() {
    const defaultLat = lat ?? D_LAT;
    const defaultLon = lon ?? D_LON;
    
    const newLatStr = prompt(`Entrez la Latitude Cible (actuel: ${tLat ?? 'N/A'}):`, tLat ?? defaultLat.toFixed(6));
    if (newLatStr !== null && !isNaN(parseFloat(newLatStr))) { tLat = parseFloat(newLatStr); }
    else return;

    const newLonStr = prompt(`Entrez la Longitude Cible (actuel: ${tLon ?? 'N/A'}):`, tLon ?? defaultLon.toFixed(6));
    if (newLonStr !== null && !isNaN(parseFloat(newLonStr))) { tLon = parseFloat(newLonStr); }
    else return;
    
    if (tLat !== null && tLon !== null) {
        if ($('target-coords')) $('target-coords').textContent = `${tLat.toFixed(4)}, ${tLon.toFixed(4)}`;
    }
}

function initAdvancedSensors() {
    try {
        als = new AmbientLightSensor({ frequency: 1 });
        als.onreading = () => {
            currentIlluminance = als.illuminance;
            if ($('illuminance-lux')) $('illuminance-lux').textContent = `${currentIlluminance.toFixed(1)} lux`;
            updateDisplayMode(); 
        };
        als.onerror = (event) => { console.log(`ALS Error: ${event.error.name}`); currentIlluminance = null; if ($('illuminance-lux')) $('illuminance-lux').textContent = 'N/A (Erreur)'; };
        als.start();
    } catch (e) { if ($('illuminance-lux')) $('illuminance-lux').textContent = 'N/A (Non supporté)'; }

    try {
        magSensor = new Magnetometer({ frequency: 1 });
        magSensor.onreading = () => {
            const magVal = Math.sqrt(magSensor.x ** 2 + magSensor.y ** 2 + magSensor.z ** 2) * 1000;
            currentMagField = magVal;
            if ($('mag-field')) $('mag-field').textContent = `${magVal.toFixed(1)} µT`;
        };
        magSensor.onerror = (event) => { console.log(`Mag Sensor Error: ${event.error.name}`); currentMagField = null; if ($('mag-field')) $('mag-field').textContent = 'N/A (Erreur)'; };
        magSensor.start();
    } catch (e) { if ($('mag-field')) $('mag-field').textContent = 'N/A (Non supporté)'; }
}

function changeMass(newMassStr) {
    const newMass = parseFloat(newMassStr);
    if (!isNaN(newMass) && newMass > 1) {
        selectedMass_kg = newMass;
        if ($('user-mass')) $('user-mass').textContent = `${selectedMass_kg.toFixed(3)} kg`;
    } else {
        alert("Masse invalide. Veuillez entrer un nombre supérieur à 000.1 kg.");
    }
}

function changeDisplaySize(size) {
    let factor = 1.0;
    if (size === 'SMALL') factor = 0.8;
    if (size === 'LARGE') factor = 1.2;
    document.documentElement.style.setProperty('--global-font-factor', factor);
}

function resetDisp() {
    stopGPS(false);
    sTime = null; 
    lPos = null;
    distM = 0; 
    maxSpd = 0;
    kSpd = 0; kUncert = 1000;
    totalWork_J = 0;
    
    if ($('time-elapsed')) $('time-elapsed').textContent = '-- s';
    if ($('distance-km-m')) $('distance-km-m').textContent = '-- km | -- m';
    if ($('speed-max')) $('speed-max').textContent = '--';
    if ($('target-dist')) $('target-dist').textContent = 'N/A';
    if ($('target-bearing')) $('target-bearing').textContent = 'N/A';
    if ($('speed-avg')) $('speed-avg').textContent = '--';
    if ($('work-energy')) $('work-energy').textContent = '0.00 MJ';
    if ($('calorie-burn')) $('calorie-burn').textContent = '0.0 Cal (Mécanique)';
    if ($('backup-battery-status')) $('backup-battery-status').textContent = 'Désactivée (Supprimé)';
    if ($('speed-stable')) $('speed-stable').textContent = '--';
    if ($('speed-3d-inst')) $('speed-3d-inst').textContent = '--';

    if ($('air-temp')) $('air-temp').textContent = 'N/A';
    if ($('pressure')) $('pressure').textContent = 'N/A';
    if ($('humidity')) $('humidity').textContent = 'N/A';
    if ($('alt-baro')) $('alt-baro').textContent = 'N/A';
    if ($('spo2-sim')) $('spo2-sim').textContent = 'N/A';
    if ($('power-consumption')) $('power-consumption').textContent = 'N/A';
    if ($('power-output')) $('power-output').textContent = 'N/A';
}

function initMapInterface() {
    const placeholder = $('map-placeholder');
    if (placeholder) {
        placeholder.addEventListener('click', () => {
            const newLatStr = prompt(`Entrez la nouvelle Latitude par défaut (actuel: ${D_LAT}) :`);
            if (newLatStr !== null && !isNaN(parseFloat(newLatStr))) { D_LAT = parseFloat(newLatStr); }
            const newLonStr = prompt(`Entrez la nouvelle Longitude par défaut (actuel: ${D_LON}) :`);
            if (newLonStr !== null && !isNaN(parseFloat(newLonStr))) { D_LON = parseFloat(newLonStr); }
            placeholder.innerHTML = `
                🗺️ **Carte Interactive (Cliquez pour Définir D_LAT/D_LON)** 🌍
                <br>Position Actuelle par Défaut: Lat=${D_LAT.toFixed(4)}, Lon=${D_LON.toFixed(4)}
                <br><small>*(Nouvelles coordonnées enregistrées. Redémarrage GPS recommandé)*</small>
            `;
            resetDisp();
        });
    }
}

function captureScreenshot() {
    html2canvas(document.body, {
        allowTaint: true,
        useCORS: true,
        scale: 1, 
        backgroundColor: document.body.classList.contains('night-mode') ? '#000033' : '#121212',
    }).then(function(canvas) {
        const timestamp = new Date().toISOString().replace(/[:.]/g, '-');
        const a = document.createElement('a');
        a.href = canvas.toDataURL('image/png');
        a.download = `dashboard_capture_${timestamp}.png`;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
    });
}

document.addEventListener('DOMContentLoaded', () => {
    
    const startBtn = $('start-btn'), stopBtn = $('stop-btn'), resetAllBtn = $('reset-all-btn');
    const resetMaxBtn = $('reset-max-btn'), emergencyStopBtn = $('emergency-stop-btn');
    const setMassBtn = $('set-mass-btn'), setTargetBtn = $('set-target-btn');
    const environmentSelect = $('environment-select'), weatherSelect = $('weather-select');
    const netherToggleBtn = $('nether-toggle-btn'), toggleModeBtn = $('toggle-mode-btn');
    const freqManualBtn = $('freq-manual-btn'), captureBtn = $('capture-btn');
    
    // 1. Boutons de Contrôle de Base
    if (startBtn) startBtn.addEventListener('click', startGPS);
    if (stopBtn) stopBtn.addEventListener('click', stopGPS);
    if (resetAllBtn) resetAllBtn.addEventListener('click', resetDisp);
    if (resetMaxBtn) resetMaxBtn.addEventListener('click', () => { maxSpd = 0; if ($('speed-max')) $('speed-max').textContent = '--'; });
    if (emergencyStopBtn) emergencyStopBtn.addEventListener('click', emergencyStop);
    
    // 2. Paramètres Utilisateur
    if (setMassBtn) setMassBtn.addEventListener('click', () => {
        const newMassStr = prompt(`Entrez la nouvelle Masse (kg) :`);
        changeMass(newMassStr);
    });
    if (setTargetBtn) setTargetBtn.addEventListener('click', setTarget);
    if (environmentSelect) environmentSelect.addEventListener('change', handleEnvironmentChange);
    if (weatherSelect) weatherSelect.addEventListener('change', handleWeatherChange);
    
    // 3. Contrôles d'Affichage
    if ($('size-normal-btn')) $('size-normal-btn').addEventListener('click', () => changeDisplaySize('NORMAL'));
    if ($('size-small-btn')) $('size-small-btn').addEventListener('click', () => changeDisplaySize('SMALL'));
    if ($('size-large-btn')) $('size-large-btn').addEventListener('click', () => changeDisplaySize('LARGE'));
    if (toggleModeBtn) toggleModeBtn.addEventListener('click', toggleManualMode);
    
    if ($('set-default-loc-btn')) $('set-default-loc-btn').addEventListener('click', () => { 
        const newLatStr = prompt(`Entrez la nouvelle Latitude par défaut (actuel: ${D_LAT}) :`);
        if (newLatStr !== null && !isNaN(parseFloat(newLatStr))) { D_LAT = parseFloat(newLatStr); }
        const newLonStr = prompt(`Entrez la nouvelle Longitude par défaut (actuel: ${D_LON}) :`);
        if (newLonStr !== null && !isNaN(parseFloat(newLonStr))) { D_LON = parseFloat(newLonStr); }
        alert(`Nouvelle position par défaut : Lat=${D_LAT.toFixed(4)}, Lon=${D_LON.toFixed(4)}.`);
        resetDisp();
    });

    if (netherToggleBtn) netherToggleBtn.addEventListener('click', () => {
        netherMode = !netherMode;
        distM = 0; 
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

    // 4. Initialisation Finale
    syncH();
    initBattery();
    initAdvancedSensors();
    initMapInterface();
    changeMass(selectedMass_kg);
    handleEnvironmentChange(); 
    handleWeatherChange();     
    resetDisp(); 
    updateDisplayMode(); 


    if (domID === null) {
        domID = setInterval(fastDOM, DOM_LOW_FREQ_MS); 
        fastDOM.lastSlowT = 0; 
    }
});
// Environ 360 lignes
