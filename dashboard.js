// =================================================================
// FICHIER JS PARTIE 1/2 : dashboard_part1.js (V4.2 Core & Init)
// Contient : Constantes, Variables d'État, Kalman, GPS, Batterie.
// LIGNES : ~250 (Contrainte < 400 respectée)
// =================================================================

// --- FONCTIONS UTILITAIRES DE BASE ---
const $ = id => document.getElementById(id); 

// --- CLÉS D'API ET CONSTANTES ---
const API_KEYS = { WEATHER_API: 'VOTRE_CLE_API_METEO_ICI' };
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458, C_S = 343, R_E = 6371000, KMH_MS = 3.6;
const OBLIQ = 23.44 * D2R, ECC = 0.0167, JD_2K = 2451545.0; 
const G_ACCEL = 9.80665;
let D_LAT = 48.8566, D_LON = 2.3522; 
const MIN_DT = 0.01; 

// CONFIGURATIONS GPS
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};

// PÉNALITÉS ENVIRONNEMENTALES POUR LE FILTRE DE KALMAN
const ENV_NOISE = {
    NORMAL: 0.1, METAL: 0.3, FOREST: 0.2, CONCRETE: 0.15
};

// --- VARIABLES D'ÉTAT PRINCIPALES ---
let wID = null; 
let lat = D_LAT, lon = D_LON, alt = 0, spd = 0, acc = 'N/A';
let maxSpd = 0, avgSpd = 0, totalDistM = 0, startTime = 0;
let userMassKg = 0.001; // Masse par défaut à 1 gramme (0.001 kg)
let currentEnvironment = 'NORMAL';
let currentWeather = 'CLEAR';
let isDayMode = true;
let emergencyActive = false;
let manualFreqMode = false;
let currentFreqMode = 'AUTO';
let netherMode = false;
let distMStartOffset = 0; 

// --- VARIABLES KALMAN ---
let kSpd = 0; 
let kUncert = 1000; 
let kPred = 0; 
let kalmanInitialized = false;
let lastTime = 0;
let lastSpd = 0;
const Q = 0.01; // Process Noise (Bruit de processus)

// --- VARIABLES D'ÉTAT BATTERIE ---
let batteryLevel = 'N/A';
let batteryCharging = false;
let backupBatteryLevel = 0.95; 
let backupAutonomyHours = 'N/A';

// --- CONSTANTES POUR LE CALCUL D'AUTONOMIE SIMULÉE ---
const BACKUP_CAPACITY_WH = 50.0;
const BACKUP_CONSUMPTION_W = 5.0;

// =================================================================
// --- LOGIQUE KALMAN ---
// =================================================================

function getKalmanR(hpe, env) {
    let noiseFactor = ENV_NOISE[env] || ENV_NOISE['NORMAL'];
    // R est l'incertitude de la mesure GPS (HPE) au carré, ajustée.
    return Math.pow(hpe, 2) * noiseFactor * 0.1; 
}

function runKalmanFilter(measurementSpd, measurementHpe) {
    if (!kalmanInitialized) {
        kSpd = measurementSpd;
        kUncert = measurementHpe * 2; 
        kalmanInitialized = true;
        lastTime = performance.now(); // Initialisation du temps
        return kSpd;
    }

    // 1. Prédiction
    const now = performance.now();
    const dt = (now - lastTime) / 1000;
    const acceleration = (kSpd - lastSpd) / (dt || MIN_DT); 
    kPred = kSpd + (acceleration * dt); 
    let kPredUncert = kUncert + Q; 

    // 2. Correction
    const R = getKalmanR(measurementHpe, currentEnvironment);
    const K = kPredUncert / (kPredUncert + R); 
    
    kSpd = kPred + K * (measurementSpd - kPred); 
    kUncert = kPredUncert * (1 - K); 

    lastSpd = kSpd;
    lastTime = now;
    return kSpd;
}

// =================================================================
// --- GESTION GPS ---
// =================================================================

function handleErr(err) {
    if ($('error-message')) {
        let msg = `❌ Erreur GNSS (code ${err.code}): `;
        switch (err.code) {
            case 1: msg += "Permission refusée. Utilisez un serveur web (http/https)."; break;
            case 2: msg += "Position non disponible ou signal perdu."; break;
            case 3: msg += "Délai d'attente dépassé."; break;
            default: msg += "Erreur inconnue."; break;
        }
        $('error-message').textContent = msg;
        $('error-message').style.display = 'block';
    }
    stopGPS(false);
}

function updateDisp(pos) {
    $('error-message').style.display = 'none';

    const coords = pos.coords;
    const now = performance.now();
    const currentTime = pos.timestamp;
    
    const newSpdMS = coords.speed !== null ? coords.speed : 0; 
    const dt = (now - lastTime) / 1000;

    // Calcul de la distance
    if (kalmanInitialized) {
        const distanceStep = kSpd * dt; 
        totalDistM += distanceStep;
    }

    // Mise à jour des variables globales
    lat = coords.latitude;
    lon = coords.longitude;
    alt = coords.altitude !== null ? coords.altitude : 0;
    spd = newSpdMS;
    acc = coords.accuracy;

    runKalmanFilter(newSpdMS, acc);
    
    if (kSpd * KMH_MS > maxSpd) {
        maxSpd = kSpd * KMH_MS;
    }

    if (startTime === 0) startTime = currentTime;
}

// =================================================================
// --- LOGIQUE BATTERIE ET AUTONOMIE DE SECOURS ---
// =================================================================

function updateBackupBatteryInfo() {
    if (typeof backupBatteryLevel !== 'number') return;
    
    const remainingWh = backupBatteryLevel * BACKUP_CAPACITY_WH;
    const autonomy = remainingWh / BACKUP_CONSUMPTION_W;
    backupAutonomyHours = autonomy; 
    // Le DOM est mis à jour dans updateDOM de la Partie 2
}

function simulateBackupDischarge() {
    if (backupBatteryLevel > 0) {
        const dischargeRate = (BACKUP_CONSUMPTION_W / BACKUP_CAPACITY_WH) * (60 / 3600); 
        backupBatteryLevel = Math.max(0, backupBatteryLevel - dischargeRate);
        updateBackupBatteryInfo();
    }
}

function initBattery() {
    // 1. Initialisation et simulation de la batterie de secours
    updateBackupBatteryInfo(); 
    setInterval(simulateBackupDischarge, 60000); 

    // 2. Surveillance de la batterie principale (API Web)
    if ('getBattery' in navigator) {
        navigator.getBattery().then(function(battery) {
            
            function updateBatteryInfo() {
                batteryLevel = battery.level; 
                batteryCharging = battery.charging;
                
                // Affichage direct de la batterie principale (avec 3 décimales)
                const percentage = batteryLevel * 100;
                const levelDisplay = percentage.toFixed(3); 
                const chargingText = batteryCharging ? ' (🔌 En charge)' : '';
                const statusText = `${levelDisplay} %${chargingText}`;
                
                if ($('battery-indicator')) {
                    const indicatorElement = $('battery-indicator');
                    indicatorElement.textContent = statusText;
                    
                    if (percentage <= 15) { indicatorElement.style.color = '#f44336'; } 
                    else if (percentage <= 30) { indicatorElement.style.color = '#ff9800'; } 
                    else { indicatorElement.style.color = '#00ff99'; } 
                }
            }
            
            updateBatteryInfo();
            battery.addEventListener('levelchange', updateBatteryInfo);
            battery.addEventListener('chargingchange', updateBatteryInfo);
            
        }).catch(error => {
            console.error("Erreur d'accès à l'API de batterie:", error);
            if ($('battery-indicator')) $('battery-indicator').textContent = 'API Rejetée';
        });
        
    } else {
        if ($('battery-indicator')) $('battery-indicator').textContent = 'N/A (API absente)';
    }
}

// =================================================================
// --- CONTRÔLE GPS & INITIALISATION FINALE ---
// =================================================================

function startGPS() {
    if (wID === null) {
        $('error-message').style.display = 'none';
        const opts = (currentFreqMode === 'LOW') ? GPS_OPTS.LOW_FREQ : GPS_OPTS.HIGH_FREQ;
        wID = navigator.geolocation.watchPosition(updateDisp, handleErr, opts);
        $('start-btn').disabled = true;
        $('stop-btn').disabled = false;
        if (typeof window.startFastDOM === 'function') window.startFastDOM();
    }
}

function stopGPS(resetID = true) {
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        if (resetID) wID = null;
    }
    $('start-btn').disabled = false;
    $('stop-btn').disabled = true;
    if (typeof window.stopFastDOM === 'function') window.stopFastDOM();
}

// La fonction d'arrêt d'urgence est définie dans la PARTIE 2
window.emergencyStop = function() {
    if (emergencyActive) {
        if (typeof window.resetAllState === 'function') window.resetAllState();
        $('emergency-status').textContent = 'INACTIF';
        $('emergency-status').style.color = '#00ff99';
        $('emergency-stop-btn').textContent = '🚨 Arrêt Urgence';
        emergencyActive = false;
    } else {
        stopGPS(true); 
        $('emergency-status').textContent = 'ACTIF (GPS Off)';
        $('emergency-status').style.color = '#f44336';
        $('emergency-stop-btn').textContent = '🟢 Démarrer Système';
        emergencyActive = true;
    }
};

function initAll() {
    initBattery();
    if (typeof window.initControls === 'function') window.initControls();
}
// =================================================================
// FICHIER JS PARTIE 2/2 : dashboard_part2.js (V4.2 DOM & Avancé)
// Contient : Logique DOM, Astro, ZTD, Contrôles utilisateur.
// LIGNES : ~350 (Contrainte < 400 respectée)
// =================================================================

// --- CONSTANTES DE TEMPS ---
const DOM_HIGH_FREQ_MS = 100;
const DOM_LOW_FREQ_MS = 400;

// --- VARIABLES DE TEMPS/DOM ---
let domID = null;
let domFreqMS = DOM_HIGH_FREQ_MS;
let lastUpdateDOM = 0;
let totalAvgSpd = 0;
let totalAvgCount = 0;

// =================================================================
// --- FONCTIONS ASTRONOMIQUES (V3.2) ---
// =================================================================

function toJulianDay(date) {
    const time = date.getTime();
    return (time / 86400000.0) + 2440587.5; 
}

function formatHours(hours) {
    const h = Math.floor(hours);
    const m = Math.floor((hours - h) * 60);
    const s = Math.floor(((hours - h) * 60 - m) * 60);
    return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
}

function getAstroData(date) {
    const J = toJulianDay(date);
    const n = J - JD_2K;
    const M = (357.5291 + 0.98560028 * n) * D2R; 
    const L = (280.459 + 0.98564736 * n) * D2R; 
    const C = (1.914 * Math.sin(M) + 0.02 * Math.sin(2 * M)) * D2R; 
    const lambda = L + C; 
    const epsilon = OBLIQ; 
    const nu = lambda - L; 
    const EoT_min = nu * R2D * 4; 

    const alpha = Math.atan2(Math.cos(epsilon) * Math.sin(lambda), Math.cos(lambda)); 

    const TST_hrs = (12 + (alpha * R2D / 15) + (lon / 15)) % 24;
    const TSM_hrs = (12 + (L * R2D / 15) + (lon / 15)) % 24;
    const culmination_LSM = 12 - (lon / 15) + (EoT_min / 60);

    const J_moon = J - 5.5; 
    const synodic = 29.5305886;
    const lunarAge = J_moon % synodic;
    const lunarPhasePerc = (lunarAge / synodic) * 100;

    return { TST_hrs, TSM_hrs, EoT_min, culmination_LSM: culmination_LSM % 24, lunarPhasePerc };
}

// =================================================================
// --- CALCULS AVANCÉS (ZTD & Physique) ---
// =================================================================

function calculateZTD(h, P, T, H) {
    const T_K = T + 273.15; 
    const P_Pa = P * 100; 
    const ZHD = 10e-5 * 0.002277 * P_Pa / (1 - 0.00266 * Math.cos(2 * lat * D2R) - 0.00028 * h); 
    const ZWD = 10e-5 * 0.000018 * (H / 100) * 0.000227 * (1255 / T_K - 0.05) * P_Pa; 
    return ZHD + ZWD; 
}

function getDragCoefficient(weather) {
    switch (weather) {
        case 'RAIN': return 1.1;
        case 'SNOW': return 1.2;
        case 'STORM': return 1.3;
        default: return 1.0;
    }
}

// =================================================================
// --- LOGIQUE DOM & BOUCLE PRINCIPALE ---
// =================================================================

function updateDOM() {
    const date = new Date();
    const elapsedTimeS = startTime > 0 ? (date.getTime() - startTime) / 1000 : 0;
    const stableSpdKmh = kSpd * KMH_MS;
    
    // Vitesse moyenne cumulée
    if (kSpd > 0) {
        totalAvgSpd += kSpd;
        totalAvgCount++;
        avgSpd = totalAvgSpd / totalAvgCount * KMH_MS;
    }

    // Calculs dérivés
    const currentAccel = (kSpd - lastSpd) / (performance.now() - lastTime) / 1000 || 0;
    const dragCoeff = getDragCoefficient(currentWeather);
    const dragForce = 0.5 * dragCoeff * 1.225 * Math.pow(kSpd, 2);
    const dragPower = dragForce * kSpd; 
    const gForce = currentAccel / G_ACCEL;
    
    const accelerationForce = userMassKg * currentAccel;
    const netForce = accelerationForce + dragForce; 
    const workEnergyMJ = (totalDistM * netForce) / 1000000;
    const calorieBurn = workEnergyMJ * 239.006; 

    // Données astronomiques et ZTD (T, P, H fixés pour la simulation)
    const astro = getAstroData(date);
    const ZTD = calculateZTD(alt, 1013.25, 15, 50); 

    // --- MISE À JOUR DU DOM ---
    $('local-time').textContent = date.toLocaleTimeString();

    // Vitesse & Kalman
    $('speed-stable').textContent = stableSpdKmh.toFixed(3);
    $('kalman-uncert').textContent = kUncert.toFixed(3) + ' m/s';
    $('kalman-r').textContent = getKalmanR(acc, currentEnvironment).toFixed(4) + ' m²';
    $('speed-3d-inst').textContent = (spd * KMH_MS).toFixed(3);
    $('speed-max').textContent = maxSpd.toFixed(3);
    $('speed-avg').textContent = avgSpd.toFixed(3);
    $('distance-km-m').textContent = `${(totalDistM / 1000).toFixed(3)} km | ${totalDistM.toFixed(1)} m`;
    $('time-elapsed').textContent = elapsedTimeS.toFixed(1) + ' s';
    $('perc-light').textContent = (stableSpdKmh / (C_L * KMH_MS) * 100).toFixed(6) + ' %';
    
    // GNSS & Corrections
    $('latitude').textContent = lat.toFixed(6);
    $('longitude').textContent = lon.toFixed(6);
    $('altitude').textContent = alt.toFixed(1) + ' m';
    $('accuracy').textContent = typeof acc === 'number' ? acc.toFixed(2) + ' m' : acc;
    $('ztd').textContent = ZTD.toFixed(3) + ' m';
    
    // Astro
    $('solar-true-header').textContent = formatHours(astro.TST_hrs) + ' (HSV)';
    $('solar-mean-header').textContent = formatHours(astro.TSM_hrs) + ' (LSM)';
    $('solar-true').textContent = formatHours(astro.TST_hrs);
    $('solar-mean').textContent = formatHours(astro.TSM_hrs);
    $('solar-culmination').textContent = formatHours(astro.culmination_LSM);
    $('eot').textContent = astro.EoT_min.toFixed(3) + ' min';
    $('lunar-phase-perc').textContent = astro.lunarPhasePerc.toFixed(1) + ' %';
    
    // Physique
    $('accel-long').textContent = currentAccel.toFixed(3) + ' m/s²';
    $('g-force').textContent = gForce.toFixed(3) + ' G';
    $('drag-power-kw').textContent = (dragPower / 1000).toFixed(3) + ' kW';
    $('work-energy').textContent = workEnergyMJ.toFixed(3) + ' MJ';
    $('calorie-burn').textContent = calorieBurn.toFixed(1) + ' Cal';
    $('user-mass').textContent = userMassKg.toFixed(3) + ' kg';
    
    // Batterie de Secours (avec précision à 3 décimales)
    if ($('backup-battery-level') && typeof backupBatteryLevel === 'number') {
        const backupPercentage = backupBatteryLevel * 100;
        const backupLevelDisplay = backupPercentage.toFixed(3);
        $('backup-battery-level').textContent = `${backupLevelDisplay} %`;
    }
    if ($('backup-autonomy') && typeof backupAutonomyHours === 'number') {
        $('backup-autonomy').textContent = backupAutonomyHours.toFixed(3) + ' h';
    }

    if ($('nether-indicator')) {
        $('nether-indicator').textContent = netherMode ? `ACTIVÉ (1:8) 🔥` : `DÉSACTIVÉ (1:1)`;
    }
    
    // Changement de mode nuit/jour
    if (astro.TSM_hrs > 6 && astro.TSM_hrs < 19) {
        if (!isDayMode) toggleMode();
    } else {
        if (isDayMode) toggleMode();
    }
}

// =================================================================
// --- GESTION DU DOM ET DES CONTRÔLES ---
// =================================================================

window.startFastDOM = function() {
    if (domID === null) {
        domID = setInterval(updateDOM, domFreqMS);
        lastUpdateDOM = performance.now();
        updateDOM(); 
    }
};

window.stopFastDOM = function() {
    if (domID !== null) {
        clearInterval(domID);
        domID = null;
    }
};

function toggleMode() {
    isDayMode = !isDayMode;
    document.body.classList.toggle('night-mode', !isDayMode);
    $('mode-indicator').textContent = isDayMode ? 'Mode: Jour ☀️' : 'Mode: Nuit 🌙';
}

window.resetAllState = function() {
    stopGPS(true);
    lat = D_LAT; lon = D_LON; alt = 0; spd = 0; acc = 'N/A';
    maxSpd = 0; avgSpd = 0; totalDistM = 0; startTime = 0;
    totalAvgSpd = 0; totalAvgCount = 0;
    kSpd = 0; kUncert = 1000; kalmanInitialized = false;
    lastTime = 0; lastSpd = 0;
    updateDOM();
    startFastDOM();
};

function changeDisplaySize(size) {
    let factor = 1.0;
    if (size === 'SMALL') factor = 0.8;
    else if (size === 'LARGE') factor = 1.2;
    document.documentElement.style.setProperty('--global-font-factor', factor.toString());
}

function setMass() {
    const newMass = prompt("Entrez la nouvelle masse de l'utilisateur en kg :", userMassKg.toFixed(3));
    if (newMass !== null && !isNaN(parseFloat(newMass)) && parseFloat(newMass) > 0) {
        userMassKg = parseFloat(newMass);
        $('user-mass').textContent = userMassKg.toFixed(3) + ' kg';
    } else if (newMass !== null) {
        alert("Saisie non valide. La masse doit être un nombre positif.");
    }
}

function initControls() {
    // Événements GPS
    if ($('start-btn')) $('start-btn').addEventListener('click', startGPS);
    if ($('stop-btn')) $('stop-btn').addEventListener('click', () => stopGPS(true));
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', resetAllState);
    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', emergencyStop);
    
    // Événements de contrôle
    if ($('toggle-mode-btn')) $('toggle-mode-btn').addEventListener('click', toggleMode);
    if ($('set-mass-btn')) $('set-mass-btn').addEventListener('click', setMass);
    if ($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', () => { netherMode = !netherMode; updateDOM(); });
    
    // Événements d'environnement (Mise à jour des variables globales de la Partie 1)
    if ($('environment-select')) $('environment-select').addEventListener('change', (e) => {
        currentEnvironment = e.target.value;
        $('selected-environment-ind').textContent = currentEnvironment;
    });
    if ($('weather-select')) $('weather-select').addEventListener('change', (e) => {
        currentWeather = e.target.value;
        $('selected-weather-ind').textContent = currentWeather;
    });

    // Événements de taille d'affichage
    if ($('size-normal-btn')) $('size-normal-btn').addEventListener('click', () => changeDisplaySize('NORMAL'));
    if ($('size-small-btn')) $('size-small-btn').addEventListener('click', () => changeDisplaySize('SMALL'));
    if ($('size-large-btn')) $('size-large-btn').addEventListener('click', () => changeDisplaySize('LARGE'));

    // Démarrage initial de la boucle DOM
    startFastDOM();
}

window.initControls = initControls;
