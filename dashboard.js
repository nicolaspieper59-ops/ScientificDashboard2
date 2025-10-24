// =================================================================
// FICHIER JS PARTIE 1/2 : dashboard_part1.js (V4.2 Core & Init)
// Contient constantes, variables d'état, calculs Geo/Astro/Kalman,
// et les fonctions d'initialisation des capteurs/systèmes.
// Doit être chargé avant dashboard_part2.js
// =================================================================

// --- FONCTIONS UTILITAIRES DE BASE (pour accès rapide) ---
const $ = id => document.getElementById(id); 

// --- CLÉS D'API (Non utilisées dans les calculs physiques) ---
const API_KEYS = {
    WEATHER_API: 'VOTRE_CLE_API_METEO_ICI' 
};

// --- CONSTANTES GLOBALES ET INITIALISATION ---\
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458, C_S = 343, R_E = 6371000, KMH_MS = 3.6;
const OBLIQ = 23.44 * D2R, ECC = 0.0167, JD_2K = 2451545.0; 
const G_ACCEL = 9.80665;
let D_LAT = 48.8566, D_LON = 2.3522; // Destination par défaut (Paris)
const MIN_DT = 0.01; 

// CONFIGURATIONS GPS POUR L'OPTIMISATION BATTERIE
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};

// --- VARIABLES D'ÉTAT PRINCIPALES ---
let wID = null; // ID de la surveillance GPS
let lat = D_LAT, lon = D_LON, alt = 0, spd = 0, acc = 'N/A'; // Geo
let maxSpd = 0, avgSpd = 0, totalDistM = 0, startTime = 0; // Stats
let userMassKg = 70.0; // Masse utilisateur par défaut
let currentEnvironment = 'NORMAL';
let currentWeather = 'CLEAR';
let isDayMode = true; // Mode d'affichage jour/nuit
let emergencyActive = false;
let manualFreqMode = false;
let currentFreqMode = 'AUTO';

// --- VARIABLES KALMAN (Filtre de stabilisation de la vitesse) ---
let kSpd = 0; 
let kUncert = 1000; // Incertitude initiale élevée
let kalmanInitialized = false;

// --- VARIABLES D'ÉTAT BATTERIE ---
let batteryLevel = 'N/A'; // Niveau principal (0.0 à 1.0 ou 'N/A')
let batteryCharging = false;

// --- VARIABLES BATTERIE DE SECOURS SIMULÉE (V4.2) ---
let backupBatteryLevel = 0.95; // Niveau initial de la batterie de secours (95% -> 0.95)
let backupAutonomyHours = 'N/A'; // Autonomie estimée (heures)

// --- CONSTANTES POUR LE CALCUL D'AUTONOMIE SIMULÉE ---
const BACKUP_CAPACITY_WH = 50.0; // Capacité totale de la batterie de secours (Watt-heures)
const BACKUP_CONSUMPTION_W = 5.0; // Consommation du système en mode secours (Watts)

// =================================================================
// --- LOGIQUE BATTERIE ET AUTONOMIE DE SECOURS ---
// =================================================================

// Calcule et met à jour l'autonomie de la batterie de secours.
function updateBackupBatteryInfo() {
    if (typeof backupBatteryLevel !== 'number') return;
    
    // Calcul de l'autonomie estimée en heures: Heures = Wh restants / Consommation W
    const remainingWh = backupBatteryLevel * BACKUP_CAPACITY_WH;
    const autonomy = remainingWh / BACKUP_CONSUMPTION_W;
    backupAutonomyHours = autonomy; // Stocke la valeur brute

    // Mise à jour du DOM pour la batterie de secours (avec 3 décimales)
    if ($('backup-battery-level')) {
        const backupPercentage = backupBatteryLevel * 100;
        const backupLevelDisplay = backupPercentage.toFixed(3);
        $('backup-battery-level').textContent = `${backupLevelDisplay} %`;
    }
    
    // Mise à jour du DOM pour l'autonomie estimée
    if ($('backup-autonomy')) {
        $('backup-autonomy').textContent = autonomy.toFixed(2) + ' h';
    }
}

// Déclenche une simulation de décharge (à appeler par exemple toutes les 60s)
function simulateBackupDischarge() {
    if (backupBatteryLevel > 0) {
        // Taux de décharge simulé (basé sur une heure, divisé par les secondes entre les appels)
        const dischargeRate = (BACKUP_CONSUMPTION_W / BACKUP_CAPACITY_WH) * (60 / 3600); // Décharge de 1 minute
        backupBatteryLevel = Math.max(0, backupBatteryLevel - dischargeRate);
        updateBackupBatteryInfo();
    }
}

// Initialise la surveillance de la batterie (Principale & Secours)
function initBattery() {
    // 1. Initialisation de la batterie de secours simulée et de la simulation
    updateBackupBatteryInfo(); 
    // Démarrer la simulation de décharge toutes les 60 secondes (60000ms)
    setInterval(simulateBackupDischarge, 60000); 

    // 2. Surveillance de la batterie principale (API Web)
    if ('getBattery' in navigator) {
        navigator.getBattery().then(function(battery) {
            
            function updateBatteryInfo() {
                // Mise à jour des variables globales
                batteryLevel = battery.level; 
                batteryCharging = battery.charging;
                
                // --- Mise à jour du DOM DIRECTE pour la batterie principale (3 décimales) ---
                const percentage = batteryLevel * 100;
                const levelDisplay = percentage.toFixed(3); // Formatage à trois décimales
                const chargingText = batteryCharging ? ' (🔌 En charge)' : '';
                const statusText = `${levelDisplay} %${chargingText}`;
                
                if ($('battery-indicator')) {
                    const indicatorElement = $('battery-indicator');
                    indicatorElement.textContent = statusText;
                    
                    // Gestion de la couleur
                    if (percentage <= 15) {
                        indicatorElement.style.color = '#f44336'; 
                    } else if (percentage <= 30) {
                        indicatorElement.style.color = '#ff9800'; 
                    } else {
                        indicatorElement.style.color = '#00ff99'; 
                    }
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
        // Fallback si l'API n'est pas supportée
        if ($('battery-indicator')) $('battery-indicator').textContent = 'N/A (API absente)';
    }
}

// =================================================================
// --- FONCTIONS DE GÉOLOCALISATION ET DE MISES À JOUR GPS ---
// =================================================================

function handleErr(err) {
    if ($('error-message')) {
        let msg = `❌ Erreur GNSS (code ${err.code}): `;
        switch (err.code) {
            case 1: msg += "Permission refusée par l'utilisateur."; break;
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
    // La logique de mise à jour du GPS est trop longue pour être incluse ici
    // et sera contenue dans la suite de dashboard_part1.js
    // Elle doit mettre à jour lat, lon, spd, etc., et appeler la fonction Kalman
    // [LOGIQUE GPS ET KALMAN À CONTINUER ICI]
}

function startGPS() {
    if (wID === null) {
        $('error-message').style.display = 'none';
        const opts = (currentFreqMode === 'LOW') ? GPS_OPTS.LOW_FREQ : GPS_OPTS.HIGH_FREQ;
        wID = navigator.geolocation.watchPosition(updateDisp, handleErr, opts);
        $('start-btn').disabled = true;
        $('stop-btn').disabled = false;
        // Démarrer la boucle DOM (dans la Partie 2)
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
    // Arrêter la boucle DOM (dans la Partie 2)
    if (typeof window.stopFastDOM === 'function') window.stopFastDOM();
}

// =================================================================
// --- FONCTIONS DE CONTRÔLE ET D'INITIALISATION FINALE ---
// =================================================================

// Cette fonction doit être appelée par le 'onload' du body
function initAll() {
    // Initialisation des capteurs
    initBattery();
    // initAmbientLightSensor(); // (Si vous l'ajoutez)

    // Initialisation des contrôles (dans la Partie 2)
    // initControls(); 

    // Tentative de démarrage initial de la surveillance GPS
    // Le démarrage est souvent refusé si l'utilisateur n'interagit pas, 
    // donc nous laissons l'utilisateur cliquer sur 'Démarrer GPS'.
}

// ... Le reste du fichier dashboard_part1.js, y compris la logique GPS/Kalman 
// et les autres fonctions de contrôle, devrait suivre ici.
                                                                                              

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
