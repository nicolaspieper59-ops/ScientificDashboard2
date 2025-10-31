// =================================================================
// FICHIER JS PARTIE 1/2 : gnss-dashboard-part1.js (CONSTANTES, UTILS & INIT)
// DOIT ÊTRE CHARGÉ AVANT gnss-dashboard-part2.js
// =================================================================

// --- CLÉS D'API & PROXY VERCEL ---
// 🚨 IMPORTANT : REMPLACER AVEC L'URL HTTPS VERCEL DE VOTRE PROXY NODE.JS
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app"; 
// const PROXY_BASE_URL = "VOTRE-URL-VERCEL-HTTPS-ICI"; // <-- Utilisez votre URL ici

const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;

// --- CONSTANTES GLOBALES ET INITIALISATION ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458, C_S = 343, R_E = 6371000, KMH_MS = 3.6;
const OBLIQ = 23.44 * D2R, ECC = 0.0167, JD_2K = 2451545.0; 
let D_LAT = 48.8566, D_LON = 2.3522; 
const MIN_DT = 0.01; 
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};

// PARAMÈTRES AVANCÉS DU FILTRE DE KALMAN
const Q_NOISE = 0.01;       
const R_MIN = 0.05, R_MAX = 50.0; 
const MAX_ACC = 50, MIN_SPD = 0.001, ALT_TH = -50;
const SPEED_THRESHOLD = 0.5; 

// Fréquences pour la boucle principale fastDOM
const DOM_HIGH_FREQ_MS = 17;   
const DOM_LOW_FREQ_MS = 250;   
const DOM_SLOW_UPDATE_MS = 1000; 

// CONSTANTES POUR LES MODÈLES PHYSIQUES
const AIR_DENSITY = 1.225; 
const G_ACCEL = 9.80665;   
const CDA_EST = 0.6;       
const TROPO_K2 = 382000; 
const SUN_NIGHT_TH = -12; 
const LUX_NIGHT_TH = 50; 
const NETHER_RATIO = 8; 

// CONSTANTES POUR FILTRAGE AMÉLIORÉ
const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0, DRAG_MULT: 1.0 },
    'METAL': { R_MULT: 2.5, DRAG_MULT: 1.0 },      
    'FOREST': { R_MULT: 1.5, DRAG_MULT: 1.2 },     
    'CONCRETE': { R_MULT: 3.0, DRAG_MULT: 1.0 },   
};
const WEATHER_FACTORS = {
    'CLEAR': 1.0,
    'RAIN': 1.2,                                   
    'SNOW': 1.5,                                   
    'STORM': 2.0,                                  
};

// --- VARIABLES D'ÉTAT (ACCESSIBLES GLOBALEMENT) ---
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
let emergencyStopActive = false;
let selectedEnvironment = 'NORMAL'; 
let selectedWeather = 'CLEAR'; 
let currentBatteryLevel = 100;

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
        const controller = new AbortController();
        const timeoutId = setTimeout(() => controller.abort(), 5000);
        
        const res = await fetch(`https://worldtimeapi.org/api/timezone/${tz}`, { signal: controller.signal });
        clearTimeout(timeoutId);

        if (!res.ok) throw new Error(res.status);
        const data = await res.json();
        lServH = data.unixtime * 1000; 
        lLocH = Date.now();            
    } catch (e) {
        if (e.name !== 'AbortError' && lServH === null) lLocH = Date.now();
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
// CORRECTIONS MÉTÉO/TROPOSPHÈRE (V3.1/V3.2)
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
    
    const envFactor = ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT;
    const weatherFactor = WEATHER_FACTORS[selectedWeather];

    R *= envFactor;
    R *= weatherFactor;
    
    if (ztd_m > 2.5) { R *= 1.1; }

    R = Math.max(R_MIN, Math.min(R_MAX, R));
    
    if (emergencyStopActive) {
        R = Math.min(R, 10.0); 
    }

    return R;
}

function getCurrentDragMultiplier() {
    return ENVIRONMENT_FACTORS[selectedEnvironment].DRAG_MULT;
}

/**
 * Récupère les données météo du proxy Vercel.
 * Met à jour lastP_hPa, lastT_K, lastH_perc.
 */
async function fetchWeather(latA, lonA) {
    if (!PROXY_BASE_URL || PROXY_BASE_URL.includes("VOTRE-URL-VERCEL-HTTPS-ICI")) {
        // Simulation si l'URL Vercel n'est pas configurée
        lastP_hPa = 1013.25 + Math.sin(Date.now() / 100000) * 1.5;
        lastT_K = 293.15 + Math.sin(Date.now() / 500000) * 5; 
        lastH_perc = 0.5 + Math.cos(Date.now() / 300000) * 0.1;
        // Mise à jour des données météo simulées dans le DOM
        const T_C_sim = lastT_K - 273.15;
        if ($('temp-air')) $('temp-air').textContent = `${T_C_sim.toFixed(1)} °C (Simulé)`;
        if ($('pressure')) $('pressure').textContent = `${lastP_hPa.toFixed(2)} hPa (Simulé)`;
        if ($('humidity')) $('humidity').textContent = `${(lastH_perc * 100).toFixed(1)} % (Simulé)`;
        return; 
    }

    const url = `${PROXY_WEATHER_ENDPOINT}?lat=${latA}&lon=${lonA}`;

    try {
        const res = await fetch(url);
        const data = await res.json();

        if (!res.ok || data.error) {
            console.error("Erreur Proxy Météo:", data.error || res.statusText);
            return;
        }

        // Mise à jour des variables d'état avec les données OpenWeatherMap
        if (data.main.temp !== undefined) lastT_K = data.main.temp + 273.15; 
        lastP_hPa = data.main.pressure; 
        lastH_perc = data.main.humidity / 100; 
        
        // Mise à jour des données météo réelles dans le DOM
        const T_C_real = lastT_K - 273.15;
        if ($('temp-air')) $('temp-air').textContent = `${T_C_real.toFixed(1)} °C`;
        if ($('pressure')) $('pressure').textContent = `${lastP_hPa.toFixed(2)} hPa`;
        if ($('humidity')) $('humidity').textContent = `${(lastH_perc * 100).toFixed(1)} %`;
        if ($('wind-speed-est')) $('wind-speed-est').textContent = `${(data.wind.speed * KMH_MS).toFixed(1)} km/h`;
        if ($('cloud-cover')) $('cloud-cover').textContent = `${data.clouds.all} %`;

    } catch (e) {
        console.error("Échec de l'appel au serveur proxy Vercel:", e);
    }
}

// ===========================================
// CALCULS ASTRO & ENV (V3.0/V3.2)
// Ces fonctions sont utilisées par updateAstro() dans PARTIE 2
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
// CAPTEURS AVANCÉS & SYSTÈME (INIT)
// ===========================================

function initALS() {
    if ('AmbientLightSensor' in window) {
        try {
            als = new AmbientLightSensor({ frequency: 5 }); 
            als.onreading = () => {
                lastLux = als.illuminance;
                const luxEl = $('luminosity'); 
                if (luxEl) luxEl.textContent = `${lastLux.toFixed(2)} lux`;
            };
            als.onerror = (event) => {
                lastLux = null;
                const luxEl = $('luminosity');
                if (luxEl) luxEl.textContent = 'Erreur/Non Autorisé';
                if (als) als.stop();
            };
            als.start();
        } catch (error) {
            lastLux = null;
            const luxEl = $('luminosity');
            if (luxEl) luxEl.textContent = 'N/A (Navigateur)';
        }
    } else {
        lastLux = null;
        const luxEl = $('luminosity');
        if (luxEl) luxEl.textContent = 'N/A (API Manquante)';
    }
}

function initAdvancedSensors() {
    if ('DeviceOrientationEvent' in window) {
        window.addEventListener('deviceorientation', (event) => {
            const alpha = event.webkitCompassHeading ?? event.alpha; 
            const beta = event.beta, gamma = event.gamma; 
            const bubbleEl = $('bubble-level'), magFieldEl = $('magnetic-field');
            
            if (bubbleEl) bubbleEl.textContent = `${beta !== null ? beta.toFixed(1) : '--'}° / ${gamma !== null ? gamma.toFixed(1) : '--'}°`;
            if (magFieldEl && alpha !== null) magFieldEl.textContent = `${alpha.toFixed(1)} µT (Mag)`; 
        }, true);
    } 

    if ('DeviceMotionEvent' in window) {
        window.addEventListener('devicemotion', (event) => {
            const accel = event.acceleration;
            if (accel && accel.x !== null) {
                const a_long = accel.y ?? 0; 
                const g_force = a_long / G_ACCEL;
                const gForceEl = $('g-force'), accelLongEl = $('accel-long');
                if (gForceEl) gForceEl.textContent = `${g_force.toFixed(2)} G`;
                if (accelLongEl) accelLongEl.textContent = `${a_long.toFixed(3)} m/s²`;
            }
        }, true);
    }
}

function updateBatteryStatus(battery) {
    const level = Math.floor(battery.level * 100);
    const charging = battery.charging;
    currentBatteryLevel = level;
    // L'ID 'battery-indicator' n'est pas dans le HTML, on log l'info.
    console.log(`Batterie: ${level}% ${charging ? '🔌' : '🔋'}`);
}

async function initBattery() {
    if ('getBattery' in navigator) {
        try {
            const battery = await navigator.getBattery();
            updateBatteryStatus(battery);
            
            battery.addEventListener('levelchange', () => updateBatteryStatus(battery));
            battery.addEventListener('chargingchange', () => updateBatteryStatus(battery));
        } catch (e) {
            console.warn('Accès à l\'API Batterie refusé.');
        }
    }
}

function changeDisplaySize(size) {
    const root = document.body;
    let factor = 1.0;
    
    if (size === 'SMALL') factor = 0.8;
    else if (size === 'LARGE') factor = 1.2;
    else factor = 1.0; 
    
    root.style.fontSize = `${factor * 16}px`; 
}

function toggleManualMode() {
    manualMode = manualMode === null ? (document.body.classList.contains('night-mode') ? false : true) : (manualMode === true ? false : true);
    document.body.classList.toggle('night-mode', manualMode);
    updateAstro(lat ?? D_LAT, lon ?? D_LON);
}

function toggleManualFreq() {
    manualFreqMode = !manualFreqMode;

    if (manualFreqMode) {
        forcedFreqState = 'HIGH_FREQ'; 
        setGPSMode('HIGH_FREQ');
    } else {
        checkGPSFrequency(kSpd); 
    }
}

function cycleForcedFreq() {
    if (!manualFreqMode) return; 

    if (forcedFreqState === 'HIGH_FREQ') {
        forcedFreqState = 'LOW_FREQ';
        setGPSMode('LOW_FREQ');
    } else {
        forcedFreqState = 'HIGH_FREQ';
        setGPSMode('HIGH_FREQ');
    }
        }
// =================================================================
// FICHIER JS PARTIE 2/2 : gnss-dashboard-part2.js (LOGIQUE & DOM)
// NÉCESSITE gnss-dashboard-part1.js POUR FONCTIONNER
// =================================================================

// Les variables et fonctions du PARTIE 1 ($, lat, lon, kFilter, setGPSMode, dist, bearing, getCDate, etc.)
// sont accessibles ici car l'encapsulation a été retirée dans PARTIE 1.

// ===========================================
// LOGIQUE PRINCIPALE (V3.0/V3.2)
// ===========================================

function updateAstro(latA, lonA) {
    const now = getCDate(); 
    const dateM = new Date(now.getFullYear(), now.getMonth(), now.getDate(), now.getHours(), now.getMinutes(), now.getSeconds());
    const mTime = dateM.getUTCHours() * 3600 + dateM.getUTCMinutes() * 60 + dateM.getUTCSeconds();
    const mcTime = (mTime * 10) % 24000; 
    const mcH = Math.floor(mcTime / 1000) % 24, mcM = Math.floor((mcTime % 1000) / (1000/60));
    
    if ($('time-minecraft')) $('time-minecraft').textContent = `${String(mcH).padStart(2, '0')}:${String(mcM).padStart(2, '0')}:00`;

    const sData = calcSolar(); 
    const hDeg = sData.elevation;
    
    let sTimeH = (now.getUTCHours() * 3600 + now.getUTCMinutes() * 60 + now.getUTCSeconds()) / 3600;
    let trueSolarTimeH = sTimeH + (lonA / 15) + (sData.eot / 60);
    trueSolarTimeH = (trueSolarTimeH % 24 + 24) % 24;
    
    const tsH = Math.floor(trueSolarTimeH), tsM = Math.floor((trueSolarTimeH * 60) % 60), tsS = Math.floor((trueSolarTimeH * 3600) % 60);
    const tsStr = `${String(tsH).padStart(2, '0')}:${String(tsM).padStart(2, '0')}:${String(tsS).padStart(2, '0')}`;
    if ($('solar-true')) $('solar-true').textContent = tsStr;

    let meanSolarTimeH = sTimeH + (lonA / 15);
    meanSolarTimeH = (meanSolarTimeH % 24 + 24) % 24;
    const msH = Math.floor(meanSolarTimeH), msM = Math.floor((meanSolarTimeH * 60) % 60), msS = Math.floor((meanSolarTimeH * 3600) % 60);
    const msStr = `${String(msH).padStart(2, '0')}:${String(msM).padStart(2, '0')}:${String(msS).padStart(2, '0')}`;
    if ($('solar-mean')) $('solar-mean').textContent = msStr;
    
    const eotEl = $('eot');
    if (eotEl) eotEl.textContent = `${sData.eot.toFixed(2)} min`;
    const sunElevationEl = $('sun-elevation');
    if (sunElevationEl) sunElevationEl.textContent = `${hDeg.toFixed(2)} °`;
    const solarLongitudeValEl = $('solar-longitude'); 
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
}

function updateDM(latA, lonA) {
    if ($('distance-total-km')) $('distance-total-km').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    if ($('speed-max')) $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    
    if (tLat !== null && tLon !== null) {
        const distTarget = dist(latA, lonA, tLat, tLon);
        const capDest = bearing(latA, lonA, tLat, tLon);
        if ($('target-heading')) $('target-heading').textContent = `${capDest.toFixed(1)} °`;
        const distTargetEl = $('distance-cible'); 
        if (distTargetEl) distTargetEl.textContent = `${(distTarget / 1000).toFixed(3)} km`;
    }
}

function checkGPSFrequency(currentSpeed) {
    if (manualFreqMode || emergencyStopActive) {
        let newMode = emergencyStopActive ? 'LOW_FREQ' : forcedFreqState;
        if (newMode !== currentGPSMode) setGPSMode(newMode);
        return;
    }
    
    const isMovingFast = currentSpeed >= SPEED_THRESHOLD; 
    let newMode = isMovingFast ? 'HIGH_FREQ' : 'LOW_FREQ';

    if (newMode !== currentGPSMode) {
        setGPSMode(newMode);
    }
}

function fastDOM() {
    const latA = lat ?? D_LAT, lonA = lon ?? D_LON;
    const now = getCDate().getTime(); 
    const pNow = performance.now();
    
    const localTimeEl = $('local-time');
    if (localTimeEl) localTimeEl.textContent = getCDate().toLocaleTimeString();

    // Contrôle de la fréquence lente (1 Hz)
    if (fastDOM.lastSlowT && (pNow - fastDOM.lastSlowT) < DOM_SLOW_UPDATE_MS) return;
    
    fastDOM.lastSlowT = pNow;

    updateAstro(latA, lonA); 
    updateDM(latA, lonA); 
    
    if (!lPos || sTime === null) { return; }
    
    const spd3D = lPos.speedMS_3D || 0; 
    const sSpd = kSpd < MIN_SPD ? 0 : kSpd; 
    const elapS = (now - sTime) / 1000;
    
    if ($('speed-3d-inst')) $('speed-3d-inst').textContent = `${(spd3D * KMH_MS).toFixed(5)} km/h`; 
    if ($('perc-speed-c')) $('perc-speed-c').textContent = `${(spd3D / C_L * 100).toPrecision(5)}%`; 
    if ($('perc-sound')) $('perc-sound').textContent = `${(spd3D / C_S * 100).toPrecision(5)}%`; 
    if ($('speed-stable')) $('speed-stable').textContent = `${(sSpd * KMH_MS).toFixed(5)} km/h`; 
    if ($('speed-ms-mms')) $('speed-ms-mms').textContent = `${sSpd.toFixed(3)} m/s | ${(sSpd * 1000).toFixed(0)} mm/s`;

    if ($('time-elapsed')) $('time-elapsed').textContent = `${elapS.toFixed(2)} s`;
}

function updateDisp(pos) {
    lat = pos.coords.latitude; lon = pos.coords.longitude;
    const alt = pos.coords.altitude, acc = pos.coords.accuracy;
    const spd = pos.coords.speed, cTime = pos.timestamp; 

    syncH(); 

    if (sTime === null) { 
        sTime = getCDate().getTime(); 
        distMStartOffset = distM; 
    }

    if (acc > MAX_ACC) { 
        if ($('gps-precision')) $('gps-precision').textContent = `❌ ${acc.toFixed(0)} m (Trop Imprécis)`; 
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
    
    let hdg = pos.coords.heading;
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
    
    const dragMult = getCurrentDragMultiplier(); 
    const dragForce = 0.5 * AIR_DENSITY * CDA_EST * dragMult * sSpdFE ** 2; 
    const dragPower = dragForce * sSpdFE; 
    
    lPos = pos; lPos.speedMS_3D = spd3D; lPos.timestamp = cTime; 
    lPos.speedMS_3D_LAST = spd3D; 
    lPos.kalman_R_val = R_dyn; 
    lPos.kalman_kSpd_LAST = fSpd; 

    checkGPSFrequency(sSpdFE); 

    distM += sSpdFE * dt * (netherMode ? NETHER_RATIO : 1); 
    
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    
    // --- AFFICHAGE ---
    let pText = `${acc.toFixed(2)} m`;
    pText += R_dyn <= R_MIN * 1.5 ? ' (Optimal/Corrigé)' : ' (Ajusté)'; 

    if ($('gps-precision')) $('gps-precision').textContent = pText;
    if ($('speed-raw-ms')) $('speed-raw-ms').textContent = `${spd3D.toFixed(3)} m/s`; 
    if ($('speed-avg')) $('speed-avg').textContent = `${((distM - distMStartOffset) / ((getCDate().getTime() - sTime) / 1000) * KMH_MS).toFixed(5)} km/h`; 
    
    if ($('latitude')) $('latitude').textContent = `${lat.toFixed(6)}`;
    if ($('longitude')) $('longitude').textContent = `${lon.toFixed(6)}`;
    if ($('altitude-gps')) $('altitude-gps').textContent = `${alt !== null ? alt.toFixed(2) : '--'} m`; 
    if ($('is-underground')) $('is-underground').textContent = alt !== null && alt < ALT_TH ? 'Oui' : 'Non'; 
    if ($('heading')) $('heading').textContent = hdg_corr !== null ? `${hdg_corr.toFixed(1)} °` : '--';
    
    if ($('force-trainee')) $('force-trainee').textContent = `${dragForce.toFixed(1)} N`; 
    if ($('puissance-trainee')) $('puissance-trainee').textContent = `${(dragPower / 1000).toFixed(2)} kW`; 
    if ($('g-force')) $('g-force').textContent = `${gForce.toFixed(2)} G`;
    if ($('accel-long')) $('accel-long').textContent = `${accellLong.toFixed(3)} m/s²`;
    if ($('vertical-speed')) $('vertical-speed').textContent = `${spdV.toFixed(2)} m/s`;
    
    if (Date.now() - (updateDisp.lastWeatherFetch ?? 0) > 60000) {
        fetchWeather(lat, lon); 
        updateDisp.lastWeatherFetch = Date.now();
    }
}

function handleErr(err) {
    console.error(`Erreur GNSS (${err.code}): ${err.message}`);
    stopGPS(false);
}

function startGPS() {
    if (wID !== null) stopGPS(false);
    sTime = null; 
    setGPSMode(emergencyStopActive ? 'LOW_FREQ' : 'LOW_FREQ');
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = '⏸️ PAUSE GPS';
}

function stopGPS(clearT = true) {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    wID = null;
    currentGPSMode = 'OFF';
    if ($('toggle-gps-btn')) $('toggle-gps-btn').textContent = '▶️ MARCHE GPS';

    if (clearT) { 
        sTime = null;
        lPos = null;
    }
}

function emergencyStop() {
    emergencyStopActive = !emergencyStopActive;
    
    const stopBtn = $('emergency-stop-btn');
    const stopInd = $('emergency-status');
    
    if (emergencyStopActive) {
        stopGPS(false); 
        if (als && als.stop) als.stop(); 
        
        if (domID !== null) {
            clearInterval(domID);
            currentDOMFreq = DOM_LOW_FREQ_MS * 4; 
            domID = setInterval(fastDOM, currentDOMFreq);
        }
        
        if (stopBtn) { 
            stopBtn.textContent = '🟢 Démarrer Système';
            stopBtn.classList.remove('active'); 
            stopBtn.style.backgroundColor = '#4CAF50';
        }
        if (stopInd) {
            stopInd.textContent = 'ACTIF (Mode Sécurité)';
            stopInd.style.color = '#ff6666';
        }
    } else {
        startGPS(); 
        initALS();  
        
        if (domID !== null) {
            clearInterval(domID);
            currentDOMFreq = DOM_LOW_FREQ_MS; 
            domID = setInterval(fastDOM, currentDOMFreq);
        }
        
        if (stopBtn) {
            stopBtn.textContent = '🛑 Arrêt d'urgence: INACTIF';
            stopBtn.classList.add('active'); 
            stopBtn.style.backgroundColor = '#dc3545';
        }
        if (stopInd) {
            stopInd.textContent = '🟢';
            stopInd.style.color = '#00ff99';
        }
    }
}

function resetDisp() {
    stopGPS(false);
    distM = 0; distMStartOffset = 0; maxSpd = 0; 
    kSpd = 0; kUncert = 1000;
    tLat = null; tLon = null;
    lastReliableHeading = null;
    sTime = null;
    lPos = null; 
    if ($('distance-total-km')) $('distance-total-km').textContent = `0.000 km | 0.00 m`;
}

function resetMax() { maxSpd = 0; }

function setTarget() {
    const defaultLat = lat ?? D_LAT;
    const defaultLon = lon ?? D_LON;
    const newLatStr = prompt(`Entrez la Latitude cible (actuel: ${tLat ?? defaultLat.toFixed(6)}) :`);
    if (newLatStr !== null && !isNaN(parseFloat(newLatStr))) { tLat = parseFloat(newLatStr); }
    const newLonStr = prompt(`Entrez la Longitude cible (actuel: ${tLon ?? defaultLon.toFixed(6)}) :`);
    if (newLonStr !== null && !isNaN(parseFloat(newLonStr))) { tLon = parseFloat(newLonStr); }
    if ($('target-heading')) $('target-heading').textContent = 'Calcul en cours...';
}

function captureScreenshot() {
    alert("Fonction de capture non implémentée (nécessite html2canvas).");
}


// ===========================================
// INITIALISATION DES ÉVÉNEMENTS DOM
// ===========================================

document.addEventListener('DOMContentLoaded', () => {
    const toggleGpsBtn = $('toggle-gps-btn');
    const resetAllBtn = $('reset-all-btn');
    const resetMaxBtn = $('reset-max-btn');
    const resetDistBtn = $('reset-dist-btn'); // ID ajouté pour la réinitialisation de la distance
    const setTargetBtn = $('set-target-btn');
    const toggleModeBtn = $('toggle-mode-btn');
    const netherToggleBtn = $('nether-toggle-btn');
    const dataCaptureBtn = $('data-capture-btn');
    const emergencyStopBtn = $('emergency-stop-btn');
    const freqSelect = $('freq-select'); 

    resetDisp();
    
    syncH(); 
    initALS(); 
    initAdvancedSensors(); 
    fetchWeather(D_LAT, D_LON); 
    initBattery(); 
    changeDisplaySize('NORMAL'); 

    if (domID === null) {
        domID = setInterval(fastDOM, DOM_LOW_FREQ_MS); 
        fastDOM.lastSlowT = 0; 
        currentDOMFreq = DOM_LOW_FREQ_MS;
    }
    
    // Connexion des événements
    if (toggleGpsBtn) toggleGpsBtn.addEventListener('click', () => {
        if (wID === null) { startGPS(); } else { stopGPS(); }
    });
    
    if (resetMaxBtn) resetMaxBtn.addEventListener('click', resetMax);
    
    if (resetDistBtn) resetDistBtn.addEventListener('click', () => {
        distM = 0; 
        distMStartOffset = distM; 
        if ($('distance-total-km')) $('distance-total-km').textContent = `0.000 km | 0.00 m`;
    });
    
    if (resetAllBtn) resetAllBtn.addEventListener('click', () => { 
        if (confirm("Êtes-vous sûr de vouloir tout réinitialiser (Distance, Max, Cible) ?")) {
            stopGPS(true);
            resetDisp();
        }
    });
    
    if (setTargetBtn) setTargetBtn.addEventListener('click', setTarget);
    if (toggleModeBtn) toggleModeBtn.addEventListener('click', toggleManualMode);
    
    if (netherToggleBtn) netherToggleBtn.addEventListener('click', () => {
        netherMode = !netherMode;
        distM = distMStartOffset; 
        maxSpd = 0; 
        if ($('mode-nether')) $('mode-nether').textContent = netherMode ? "ACTIVÉ (1:8) 🔥" : "DÉSACTIVÉ (1:1)";
        netherToggleBtn.textContent = netherMode ? "🌍 Overworld" : "🔥 Nether";
    });
    
    if (dataCaptureBtn) dataCaptureBtn.addEventListener('click', captureScreenshot);
    
    if (emergencyStopBtn) emergencyStopBtn.addEventListener('click', emergencyStop);
    
    if (freqSelect) freqSelect.addEventListener('change', (e) => {
        const newMode = e.target.value;
        manualFreqMode = true; 
        forcedFreqState = newMode;
        setGPSMode(newMode);
    });

});
