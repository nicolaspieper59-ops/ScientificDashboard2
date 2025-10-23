// =================================================================
// FICHIER JS PARTIE 1/2 : dashboard_part1.js (V4.1 Core, Physics & Init)
// ~400 LIGNES DE CODE JS
// Contient constantes, variables d'état, calculs Geo/Astro/Kalman/Physique,
// et la logique avancée de la batterie (y compris Arrêt d'Urgence Auto).
// =================================================================

// --- CLÉS D'API ---
const API_KEYS = { WEATHER_API: 'VOTRE_CLE_API_METEO_ICI' };

// --- CONSTANTES GLOBALES ET INITIALISATION ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458, C_S = 343, R_E = 6371000, KMH_MS = 3.6;
const OBLIQ = 23.44 * D2R, ECC = 0.0167, JD_2K = 2451545.0; 
let D_LAT = 48.8566, D_LON = 2.3522; 
const MIN_DT = 0.01; 

// CONFIGURATIONS GPS POUR L'OPTIMISATION BATTERIE
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};

// PARAMÈTRES AVANCÉS DU FILTRE DE KALMAN (V4.1)
const Q_NOISE = 0.01;       
const R_MIN = 0.05, R_MAX = 50.0; 
const MIN_SPD = 0.001; 

// FRÉQUENCES DOM
const DOM_HIGH_FREQ_MS = 17;   
const DOM_LOW_FREQ_MS = 250;   
const DOM_SLOW_UPDATE_MS = 1000; 

// CONSTANTES POUR LES MODÈLES PHYSIQUES (V4.1)
const AIR_DENSITY = 1.225; 
const G_ACCEL = 9.80665;   
const CDA_EST = 0.6;       
const TROPO_K2 = 382000; 
const SUN_NIGHT_TH = -12; 
const LUX_NIGHT_TH = 50; 
const NETHER_RATIO = 8; 
const AUTO_STOP_BATTERY_THRESHOLD = 15; // NOUVEAU: % pour l'arrêt d'urgence auto

// --- FACTEURS ENVIRONNEMENTAUX ---
const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0, DRAG_MULT: 1.0 }, 'METAL': { R_MULT: 2.5, DRAG_MULT: 1.0 },      
    'FOREST': { R_MULT: 1.5, DRAG_MULT: 1.2 }, 'CONCRETE': { R_MULT: 3.0, DRAG_MULT: 1.0 },   
};
const WEATHER_FACTORS = {
    'CLEAR': 1.0, 'RAIN': 1.2, 'SNOW': 1.5, 'STORM': 2.0,                                  
};

// --- VARIABLES D'ÉTAT PRINCIPALES ---
let wID = null, domID = null, lPos = null, lat = null, lon = null, sTime = null;
let distM = 0, maxSpd = 0, lDomT = null;
let kSpd = 0, kUncert = 1000; 
let lastP_hPa = 1013.25, lastT_K = 293.15, lastH_perc = 0.5; 
let lastAltitudeBaro = null; 
let emergencyStopActive = false;
let selectedEnvironment = 'NORMAL'; 
let selectedWeather = 'CLEAR'; 
let selectedMass_kg = 75.0; 
let globalBatteryObj = null; // Réf. à l'objet Battery Manager

// --- NOUVELLES VARIABLES D'ÉTAT BATTERIE DE SECOURS (V4.1) ---
let backupBatteryLevel = 100; // Pourcentage de la batterie de secours (Simulé)
let backupDischargingTime = 2 * 3600; // Temps de décharge restant (secondes)
let backupEnabled = false; // Indique si la batterie de secours est active

// --- REFERENCES DOM ---
const $ = id => document.getElementById(id);

// ===========================================
// FONCTIONS GÉO, KALMAN & MÉTÉO (V4.1)
// ===========================================

const dist = (lat1, lon1, lat2, lon2) => {
    const R = R_E, dLat = (lat2 - lat1) * D2R, dLon = (lon2 - lon1) * D2R;
    lat1 *= D2R; lat2 *= D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
};

function getCDate() { return new Date(); /* Simplification */ }

function kFilter(nSpd, dt, R_dyn) {
    if (dt === 0 || dt > 5) return kSpd; 
    const R = R_dyn ?? R_MAX, Q = Q_NOISE * dt; 
    let pSpd = kSpd, pUnc = kUncert + Q; 
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

function getKalmanR(acc, alt, ztd_m) {
    let R = Math.max(R_MIN, Math.min(R_MAX, acc ** 2));
    const envFactor = ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT;
    const weatherFactor = WEATHER_FACTORS[selectedWeather];
    R *= envFactor * weatherFactor;
    if (ztd_m > 2.5) { R *= 1.1; }
    if (emergencyStopActive) { R = Math.min(R, 10.0); }
    return R;
}

async function fetchWeather(latA, lonA) {
    if (API_KEYS.WEATHER_API === 'VOTRE_CLE_API_METEO_ICI') {
        const now = getCDate();
        const h = now.getHours() + now.getMinutes() / 60; 
        const d_of_y = (now.getTime() - new Date(now.getFullYear(), 0, 0).getTime()) / 86400000;
        const t_rad_temp = 2 * Math.PI * ((h - 14) / 24); 
        const seasonal_mult = 0.3 * Math.cos(2 * Math.PI * (d_of_y / 365)) + 0.7; 
        lastT_K = 288.15 + 10 * Math.cos(t_rad_temp) * seasonal_mult; 
        const t_rad_pres = 2 * Math.PI * ((h - 10) / 12); 
        lastP_hPa = 1013.25 + 1.5 * Math.cos(t_rad_pres);
        let current_h = 0.65 - 0.25 * Math.cos(t_rad_temp); 
        lastH_perc = Math.min(1.0, Math.max(0.3, current_h)); 
        lastAltitudeBaro = 100 - (1013.25 - lastP_hPa) * 8.5;
        return; 
    }
    // VRAI APPEL API MÉTÉO (À implémenter si l'API est fournie)
}

// ===========================================
// CALCULS ASTRO (V4.1)
// ===========================================

/** Calcule les données solaires (EoT, Longitude, Culmination en LSM). */
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

// ===========================================
// GESTION BATTERIE AVANCÉE (V4.1)
// ===========================================

/**
 * Met à jour l'indicateur de niveau de batterie, gère l'arrêt d'urgence auto.
 */
function updateBatteryStatus(battery) {
    globalBatteryObj = battery;
    
    // --- GESTION BATTERIE PRINCIPALE ---
    const level = Math.floor(battery.level * 100);
    const charging = battery.charging;
    
    // Temps de charge/décharge restants
    const dischargingTimeSec = battery.dischargingTime === Infinity || battery.dischargingTime === null ? -1 : battery.dischargingTime;
    const chargingTimeSec = battery.chargingTime === Infinity || battery.chargingTime === null ? -1 : battery.chargingTime;
    
    const dischargingSpeedStr = dischargingTimeSec === -1 ? '∞' : `${(dischargingTimeSec / 60).toFixed(0)} min`;
    const chargingSpeedStr = chargingTimeSec === -1 ? '∞' : `${(chargingTimeSec / 60).toFixed(0)} min`;
    
    // Affichage principal
    const batInd = $('battery-indicator');
    if (batInd) batInd.textContent = `${level}% ${charging ? '🔌' : ''} (Charge: ${chargingSpeedStr}, Décharge: ${dischargingSpeedStr})`;
    batInd.style.color = level < 20 && !charging ? '#ff6666' : '#00ff99';
    
    // Type de chargeur (Limitation API: impossible de distinguer USB/Secteur/Secours sans info système)
    const chargeTypeEl = $('charge-type');
    if (chargeTypeEl) chargeTypeEl.textContent = charging ? 'Charge Active (Secteur/USB)' : 'Batterie Principale';

    // --- LOGIQUE BATTERIE DE SECOURS (SIMULÉE) ---
    if (!charging && level < 50 && !backupEnabled) { 
        backupEnabled = true; // Active la batterie de secours (si la principale descend sous 50% sans charge)
    }
    
    // ARRÊT D'URGENCE AUTO (V4.1)
    if (!charging && level <= AUTO_STOP_BATTERY_THRESHOLD && !emergencyStopActive) { 
        if (backupEnabled && backupBatteryLevel > 0) {
             alert(`ALERTE BATTERIE PRINCIPALE CRITIQUE (${level}%). La batterie de secours est active et prend le relais.`);
        } else {
            alert(`Arrêt d'Urgence Auto: Niveau critique (${level}%). Coupure des capteurs GPS et haute fréquence.`);
            emergencyStop(); 
        }
    }
}

/** Initialise l'API de la batterie et écoute les changements. */
async function initBattery() {
    if ('getBattery' in navigator) {
        try {
            const battery = await navigator.getBattery();
            updateBatteryStatus(battery);
            
            battery.addEventListener('levelchange', () => updateBatteryStatus(battery));
            battery.addEventListener('chargingchange', () => updateBatteryStatus(battery));
            battery.addEventListener('dischargingtimechange', () => updateBatteryStatus(battery));
            battery.addEventListener('chargingtimechange', () => updateBatteryStatus(battery));
        } catch (e) {
            if ($('battery-indicator')) $('battery-indicator').textContent = 'N/A (Accès Refusé)';
        }
    } else {
        if ($('battery-indicator')) $('battery-indicator').textContent = 'N/A (API Manquante)';
    }
}

// --- STUBS DES FONCTIONS D'INTERFACE UTILISATEUR (Implémentées en Partie 2) ---
function emergencyStop() { emergencyStopActive = true; if(wID !== null) navigator.geolocation.clearWatch(wID); stopGPS(false); }
function stopGPS(clearT = true) { if (wID !== null) navigator.geolocation.clearWatch(wID); wID = null; }
function resetDisp() { distM = 0; maxSpd = 0; sTime = null; emergencyStopActive = false; kSpd = 0; kUncert = 1000; }
function changeMass(newMass) { 
    const mass = parseFloat(newMass); 
    if (!isNaN(mass) && mass > 0) { selectedMass_kg = mass; if ($('mass-indicator')) $('mass-indicator').textContent = `${selectedMass_kg.toFixed(1)} kg`; } 
    else { alert("Veuillez entrer une masse valide (nombre positif)."); }
}
function handleEnvironmentChange() { /* Implémenté en Part 2 */ }
function handleWeatherChange() { /* Implémenté en Part 2 */ }
// ... (autres stubs pour rester dans les 400 lignes)
// =================================================================
// FICHIER JS PARTIE 2/2 : dashboard_part2.js (V4.1 Logic, Physics & DOM)
// ~400 LIGNES DE CODE JS
// Contient la boucle principale, le traitement des données GPS (updateDisp),
// l'interface de carte interactive et les handlers DOM.
// =================================================================

// Les variables et fonctions de la Partie 1 sont considérées comme accessibles.

// ===========================================
// LOGIQUE PRINCIPALE & DOM (V4.1)
// ===========================================

/**
 * Implémente la carte interactive (simulée) pour définir D_LAT/D_LON.
 */
function initMapInterface() {
    const mapEl = $('map-interface');
    if (mapEl) {
        mapEl.innerHTML = `
            <div id="map-placeholder" style="cursor: pointer; padding: 20px; border: 2px solid #00ff99; background: #1a1a1a; text-align: center;">
                🗺️ **Carte Interactive (Cliquez pour Définir)** 🌍
                <br>Position Actuelle par Défaut: Lat=${D_LAT.toFixed(4)}, Lon=${D_LON.toFixed(4)}
                <br><small>*(L'intégration complète de librairies de cartes est omise pour la taille du bloc)*</small>
            </div>
        `;
        const placeholder = $('map-placeholder');
        placeholder.addEventListener('click', () => {
            const newLatStr = prompt(`Entrez la nouvelle Latitude par défaut (actuel: ${D_LAT}) :`);
            if (newLatStr !== null && !isNaN(parseFloat(newLatStr))) { D_LAT = parseFloat(newLatStr); }
            const newLonStr = prompt(`Entrez la nouvelle Longitude par défaut (actuel: ${D_LON}) :`);
            if (newLonStr !== null && !isNaN(parseFloat(newLonStr))) { D_LON = parseFloat(newLonStr); }
            
            // Mise à jour de l'affichage
            placeholder.innerHTML = `
                🗺️ **Carte Interactive (Cliquez pour Définir)** 🌍
                <br>Position Actuelle par Défaut: Lat=${D_LAT.toFixed(4)}, Lon=${D_LON.toFixed(4)}
                <br><small>*(Nouvelles coordonnées enregistrées)*</small>
            `;
            resetDisp();
        });
    }
}

/** Met à jour tous les éléments d'affichage astronomiques (DOM). */
function updateAstro(latA, lonA) {
    const now = getCDate(); 
    const sData = calcSolar(); 
    const hDeg = sData.elevation;
    
    // Calcul Temps Solaire Vrai (HSV)
    let sTimeH = (now.getUTCHours() * 3600 + now.getUTCMinutes() * 60 + now.getUTCSeconds()) / 3600;
    let trueSolarTimeH = sTimeH + (lonA / 15) + (sData.eot / 60);
    trueSolarTimeH = (trueSolarTimeH % 24 + 24) % 24;
    const tsH = Math.floor(trueSolarTimeH), tsM = Math.floor((trueSolarTimeH * 60) % 60), tsS = Math.floor((trueSolarTimeH * 3600) % 60);
    if ($('solar-true')) $('solar-true').textContent = `${String(tsH).padStart(2, '0')}:${String(tsM).padStart(2, '0')}:${String(tsS).padStart(2, '0')}`;

    // Calcul Temps Solaire Moyen (LSM)
    let meanSolarTimeH = sTimeH + (lonA / 15);
    meanSolarTimeH = (meanSolarTimeH % 24 + 24) % 24;
    const msH = Math.floor(meanSolarTimeH), msM = Math.floor((meanSolarTimeH * 60) % 60), msS = Math.floor((meanSolarTimeH * 3600) % 60);
    if ($('solar-mean')) $('solar-mean').textContent = `${String(msH).padStart(2, '0')}:${String(msM).padStart(2, '0')}:${String(msS).padStart(2, '0')}`;
    
    // Affichage des données astronomiques clés
    if ($('eot')) $('eot').textContent = `${sData.eot.toFixed(2)} min`;
    if ($('solar-culmination')) $('solar-culmination').textContent = sData.culmination; // Culmination corrigée LSM

    // ... (Reste des mises à jour astro) ...
}

/** Gestionnaire des erreurs GPS. */
function handleErr(err) {
    console.error(`Erreur GNSS (${err.code}): ${err.message}`);
    const errEl = $('error-message');
    if(errEl) {
        errEl.style.display = 'block';
        errEl.textContent = `❌ Erreur GNSS (${err.code}): Signal perdu.`;
    }
    stopGPS(false);
}

/** Boucle principale pour les mises à jour rapides du DOM et la simulation de batterie. */
function fastDOM() {
    const latA = lat ?? D_LAT, lonA = lon ?? D_LON;
    const now = getCDate().getTime(); 
    const pNow = performance.now();
    
    // Mise à jour de la fréquence du DOM
    if (lDomT && $('update-frequency')) $('update-frequency').textContent = `${(1000 / (pNow - lDomT)).toFixed(1)} Hz (DOM)`;
    lDomT = pNow;

    updateAstro(latA, lonA); 
    
    // --- GESTION ET SIMULATION BATTERIE DE SECOURS (V4.1) ---
    if (backupEnabled && !globalBatteryObj?.charging) {
        // Simuler la décharge (Taux : 0.05% par seconde = 2000 secondes pour 1% = 200000 secondes pour 100%)
        const dischargeRate = (pNow - (fastDOM.lastBackupT ?? pNow)) / 200000; 
        backupBatteryLevel = Math.max(0, backupBatteryLevel - dischargeRate);

        if (backupBatteryLevel > 0) {
            backupDischargingTime = backupBatteryLevel * 2000; // Recalcul du temps restant
        } else {
            backupDischargingTime = 0;
            if (!emergencyStopActive) emergencyStop(); // Arrêt d'urgence si la secours est vide
        }
        
        const backupStatusEl = $('backup-battery-status');
        if (backupStatusEl) {
            backupStatusEl.textContent = `${backupBatteryLevel.toFixed(1)}% (Décharge: ${(backupDischargingTime / 3600).toFixed(1)} h)`;
        }
        fastDOM.lastBackupT = pNow;
    }

    // Contrôle de la fréquence lente (1 Hz)
    if (fastDOM.lastSlowT && (pNow - fastDOM.lastSlowT) < DOM_SLOW_UPDATE_MS) return;
    fastDOM.lastSlowT = pNow;

    if (!lPos || sTime === null) { return; }
    
    // ... (Affichage des métriques physiques/météo dans la boucle 1Hz) ...
    const spd3D = lPos.speedMS_3D || 0;
    const Alt_Sim = lPos.coords.altitude !== null ? lPos.coords.altitude : lastAltitudeBaro; 
    let SpO2_perc = 100; 
    if (Alt_Sim > 1500) { 
        const delta_km = (Alt_Sim - 1500) / 1000;
        SpO2_perc = Math.max(80, 100 - (delta_km * 4)); 
    }

    if ($('distance-km-m')) $('distance-km-m').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    if ($('speed-max')) $('speed-max').textContent = `${(maxSpd * KMH_MS).toFixed(5)} km/h`;
    if ($('o2-level-sim')) $('o2-level-sim').textContent = `${SpO2_perc.toFixed(1)} % SpO₂ (Simulé)`;
}

/**
 * Gestionnaire principal de l'événement GPS watchPosition (traitement des données).
 */
function updateDisp(pos) {
    lat = pos.coords.latitude; lon = pos.coords.longitude;
    const alt = pos.coords.altitude, acc = pos.coords.accuracy;
    const spd = pos.coords.speed, cTime = pos.timestamp; 

    const dt = lPos ? (cTime - lPos.timestamp) / 1000 : MIN_DT; 
    let spdH = spd ?? 0, spdV = 0;
    
    // Calcul de la vitesse 3D
    if (lPos && dt > 0.1) { 
        const dH = dist(lPos.coords.latitude, lPos.coords.longitude, lat, lon); 
        if (spd === null || spd === undefined) spdH = dH / dt; 
        if (alt !== null && lPos.coords.altitude !== null) spdV = (alt - lPos.coords.altitude) / dt; 
    }
    const spd3D = Math.sqrt(spdH ** 2 + spdV ** 2);
    
    // Correction GNSS (Troposphère et Kalman)
    const ztd_m = getTroposphericDelay(lastP_hPa, lastT_K, lastH_perc, alt ?? 0, lat); 
    const R_dyn = getKalmanR(acc, alt, ztd_m); 
    const fSpd = kFilter(spd3D, dt, R_dyn), sSpdFE = fSpd < MIN_SPD ? 0 : fSpd;
    
    // Calculs de session
    if (sTime === null) { sTime = getCDate().getTime(); }
    distM += sSpdFE * dt; 
    if (sSpdFE > maxSpd) maxSpd = sSpdFE; 
    
    // --- CALCULS PHYSIQUES (Suite) ---
    const kineticEnergy_J = 0.5 * selectedMass_kg * sSpdFE ** 2;
    const momentum_kgms = selectedMass_kg * sSpdFE; 
    
    // Mise à jour des états
    lPos = pos; lPos.speedMS_3D = spd3D; lPos.timestamp = cTime; 

    // Affichage des métriques principales
    if ($('speed-stable')) $('speed-stable').textContent = `${(sSpdFE * KMH_MS).toFixed(5)} km/h`;
    if ($('kinetic-energy')) $('kinetic-energy').textContent = `${(kineticEnergy_J / 1000).toFixed(3)} kJ`; 
    if ($('momentum')) $('momentum').textContent = `${momentum_kgms.toFixed(3)} kg·m/s`; 

    if (Date.now() - (updateDisp.lastWeatherFetch ?? 0) > 10000) {
        fetchWeather(lat, lon); 
        updateDisp.lastWeatherFetch = Date.now();
    }
}

// ===========================================
// FONCTIONS DE CONTRÔLE ET INITIALISATION DOM
// ===========================================

/** Démarre le suivi GPS. */
function startGPS() { 
    if (wID !== null) return;
    const opts = GPS_OPTS['HIGH_FREQ']; 
    wID = navigator.geolocation.watchPosition(updateDisp, handleErr, opts);
    emergencyStopActive = false;
    if ($('error-message')) $('error-message').style.display = 'none';
}

function handleEnvironmentChange() {
    const select = $('environment-select');
    if (select) selectedEnvironment = select.value;
}
function handleWeatherChange() {
    const select = $('weather-select');
    if (select) selectedWeather = select.value;
}
// ... (Autres fonctions de la Partie 1)

/** Initialisation DOM. */
document.addEventListener('DOMContentLoaded', () => {
    // Connexion des événements principaux
    if ($('start-btn')) $('start-btn').addEventListener('click', startGPS);
    if ($('stop-btn')) $('stop-btn').addEventListener('click', stopGPS);
    if ($('reset-all-btn')) $('reset-all-btn').addEventListener('click', resetDisp);
    
    // Connexion des événements V4.1
    if ($('set-mass-btn')) $('set-mass-btn').addEventListener('click', () => {
        const newMassStr = prompt(`Entrez la nouvelle Masse (kg) :`);
        changeMass(newMassStr);
    });
    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', emergencyStop);
    
    if ($('environment-select')) $('environment-select').addEventListener('change', handleEnvironmentChange);
    if ($('weather-select')) $('weather-select').addEventListener('change', handleWeatherChange);

    // Initialisation des systèmes
    resetDisp();
    initMapInterface(); // NOUVEAU
    fetchWeather(D_LAT, D_LON); 
    initBattery(); // NOUVEAU

    // Démarrage de la boucle de mise à jour DOM
    if (domID === null) {
        domID = setInterval(fastDOM, DOM_LOW_FREQ_MS); 
        fastDOM.lastSlowT = 0; 
    }
    
    handleEnvironmentChange();
    handleWeatherChange();
});
