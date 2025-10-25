// =================================================================
// BLOC 1/3 : CONSTANTES ET VARIABLES GLOBALES
// =================================================================

// Constantes Physiques et Facteurs
const C_L = 299792458;          // Vitesse de la lumière (m/s)
const KMH_MS = 3.6;             // Facteur de conversion m/s -> km/h
const G_ACCEL = 9.80665;        // Accélération de la gravité (m/s²)
const G_MASS_DEFAULT = 70;      // Masse par défaut (kg)
const AIR_DENSITY = 1.225;      // Densité de l'air (kg/m³)
const CDA_EST = 0.25;           // Coefficient de traînée estimé * Surface frontale (m²)
const MIN_SPD = 0.001;          // Vitesse minimale pour être considéré en mouvement (m/s)
const DOM_SLOW_UPDATE_MS = 1000; // Fréquence de sauvegarde de l'état (1 seconde)
const STATE_KEY = 'gnss_dashboard_state'; // Clé pour localStorage
const LIGHT_YEAR_KM = 9.461e12; // 1 Année lumière en km

// Options GPS (Haute et Basse Fréquence)
const GPS_OPTS = {
    'HIGH_FREQ': { enableHighAccuracy: true, timeout: 5000, maximumAge: 0 },
    'LOW_FREQ': { enableHighAccuracy: false, timeout: 10000, maximumAge: 60000 },
    'OFF': { enableHighAccuracy: false, timeout: 1, maximumAge: Infinity }
};

// Variables d'état
let lat = null;                 // Dernière latitude connue
let lon = null;                 // Dernière longitude connue
let kSpd = 0;                   // Vitesse Stable (Filtre Kalman) m/s
let kUncert = 1000;             // Incertitude Kalman (m)
let distM = 0;                  // Distance totale parcourue (m)
let maxSpd = 0;                 // Vitesse maximale de la session (km/h)
let sTime = null;               // Timestamp de début de session
let wID = null;                 // ID du watcher Geolocation
let domID = null;               // ID de la boucle d'actualisation DOM lente (1 Hz)
let sensorID = null;            // ID de la boucle ultra-rapide (62.5 Hz)
let currentGPSMode = 'OFF';
let lPos = null;                // Dernière position GPS reçue
let currentBatteryLevelPrecise = null; // Niveau batterie (0-100)
let deviceAutonomyS = -1;       // Autonomie estimée (secondes)
let externalBatteryLevel = null;// Niveau batterie externe (0-100)
let externalAutonomyS = -1;     // Autonomie externe (secondes)
let emergencyStopActive = false;
let isNightMode = false;
let netherMultiplier = 1;
let currentFontFactor = 1.0;
let lastReliableHeading = null;
let tLat = null; tLon = null;   // Coordonnées de la cible
let selectedEnvironment = 'NORMAL';
let selectedWeather = 'CLEAR';
let lastT_K = 293.15;           // Température de l'air (K)
let lastP_hPa = 1013.25;        // Pression atmosphérique (hPa)
let lastH_perc = 0.5;           // Humidité (0-1)
let calculatedMassKg = G_MASS_DEFAULT;
let manualTractionForce = 0;
let lastNtpSync = 0;
let distMStartOffset = 0;       // Distance accumulée avant le début de la session

// Références DOM
const $ = id => document.getElementById(id);
// =================================================================
// BLOC 2/3 : LOGIQUE GPS, KALMAN, ASTRO ET MÉTÉO
// =================================================================

/**
 * Fonctions Astro simplifiées (équivalent de SunCalc / MoonCalc)
 * Ces fonctions nécessitent une librairie complète pour être précises, 
 * mais fournissent une simulation réaliste pour le dashboard.
 */
function getJulianDay(date) {
    const time = date.getTime();
    return (time / 86400000) - 0.5 + 2440587.5;
}

function normalizeAngle(angle) {
    return angle - 360 * Math.floor(angle / 360);
}

function getLunarPhase(JD) {
    const K = (JD - 2451550.1) / 29.530588853;
    const phase = normalizeAngle(K * 360) / 360;
    return (phase * 100); 
}

function getMoonTimes(date, lat, lon) {
    const JD = getJulianDay(date);
    // Simuler Lever/Coucher/Culmination Lune
    const h = date.getHours();
    const rise = new Date(date);
    const set = new Date(date);
    const culmination = new Date(date);
    
    // Simuler une rotation d'environ 24.8 heures
    const moonTimeOffset = (JD % 1) * 24.8; 
    
    rise.setHours(Math.floor(normalizeAngle(h + moonTimeOffset + 6) % 24), Math.floor(Math.random() * 60));
    set.setHours(Math.floor(normalizeAngle(h + moonTimeOffset + 18) % 24), Math.floor(Math.random() * 60));
    culmination.setHours(Math.floor(normalizeAngle(h + moonTimeOffset + 12) % 24), Math.floor(Math.random() * 60));
    
    return {
        rise: rise,
        set: set,
        culmination: culmination,
        magnitude: (Math.abs(Math.cos(getLunarPhase(JD) * Math.PI / 50)) * 5).toFixed(2), // Simu magnitude
    };
}

function getSolarTimes(date, lat, lon) {
    // Simuler Heure Solaire Vraie (Approx)
    const offsetMin = lon * 4;
    const meanTime = new Date(date.getTime() + offsetMin * 60000);
    const solarMean = meanTime.toLocaleTimeString('fr-FR');
    
    // Simuler Équation du temps (EoT)
    const JD = getJulianDay(date);
    const n = JD - 2451545.0;
    const M = normalizeAngle(357.5291 + 0.98560028 * n);
    const C = 1.9148 * Math.sin(M * Math.PI / 180) + 0.02 * Math.sin(2 * M * Math.PI / 180) + 0.0003 * Math.sin(3 * M * Math.PI / 180);
    const EoT = (C).toFixed(4); // En minutes

    // Heure solaire vraie
    const trueTime = new Date(meanTime.getTime() + C * 60000);
    const solarTrue = trueTime.toLocaleTimeString('fr-FR');
    
    // Simuler Culmination
    const culmination = new Date(date);
    const midDayS = 12 * 3600 - offsetMin * 60 + C * 60;
    culmination.setHours(Math.floor(midDayS / 3600), Math.floor((midDayS % 3600) / 60), Math.floor(midDayS % 60));

    // Élévation du Soleil (approximation simple)
    const T = (date.getHours() + date.getMinutes()/60) - 12;
    const elev = 90 - lat + 23.45 * Math.sin((n + 284) * 2 * Math.PI / 365.25);
    const elevation = (Math.cos(T * Math.PI / 12) * elev).toFixed(2);
    
    return {
        solarTrue,
        solarMean,
        culmination: culmination.toLocaleTimeString('fr-FR'),
        eot: EoT,
        elevation: elevation,
    };
}

function updateAstro(currentLat, currentLon) {
    const now = getCDate();
    
    // Données Solaires
    const solarData = getSolarTimes(now, currentLat, currentLon);
    if ($('solar-true')) $('solar-true').textContent = solarData.solarTrue;
    if ($('solar-mean')) $('solar-mean').textContent = solarData.solarMean;
    if ($('solar-culmination')) $('solar-culmination').textContent = solarData.culmination;
    if ($('eot')) $('eot').textContent = `${(solarData.eot * 1).toFixed(4)} min`;
    if ($('sun-elevation')) $('sun-elevation').textContent = `${solarData.elevation} °`;
    
    // Données Lunaires Détaillées
    const moonData = getMoonTimes(now, currentLat, currentLon);
    const lunarPhasePerc = getLunarPhase(getJulianDay(now));
    
    if ($('lunar-phase-perc')) $('lunar-phase-perc').textContent = `${lunarPhasePerc.toFixed(0)} %`;
    if ($('moon-magnitude')) $('moon-magnitude').textContent = moonData.magnitude;
    if ($('moon-rise')) $('moon-rise').textContent = moonData.rise.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit' });
    if ($('moon-set')) $('moon-set').textContent = moonData.set.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit' });
    if ($('moon-culmination')) $('moon-culmination').textContent = moonData.culmination.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit' });
    
    // Placeholder - Non calculé dans cette implémentation simplifiée
    if ($('lunar-time')) $('lunar-time').textContent = now.toLocaleTimeString('fr-FR'); 
}

function updateGPSPrecisionPerc(accuracyMeters) {
    // Simuler une précision maximale de 20m pour 100% de la barre
    const MAX_ACC = 20; 
    const precisionPerc = Math.max(0, 100 - (accuracyMeters / MAX_ACC) * 100);
    if ($('gps-precision-perc')) $('gps-precision-perc').textContent = `${precisionPerc.toFixed(1)} %`;
}


function kalmanFilter(newVal, newAcc, dt) {
    // Simuler le filtre Kalman (EKF simplifié)
    const Q = 0.01; // Bruit de processus (faible, car le GPS est déjà filtré)
    const R = newAcc * newAcc; // Bruit de mesure (basé sur la précision GPS)

    if (lPos === null) {
        kSpd = newVal;
        kUncert = R;
        return;
    }

    // Prédiction (assume une vitesse constante)
    const predictedSpd = kSpd;
    let predictedUncert = kUncert + Q;

    // Correction (mise à jour du gain de Kalman)
    const K = predictedUncert / (predictedUncert + R);
    kSpd = predictedSpd + K * (newVal - predictedSpd);
    kUncert = (1 - K) * predictedUncert;
}

function updateDisp(pos) {
    if (emergencyStopActive) return;
    
    const now = getCDate().getTime();
    if (sTime === null) sTime = now;
    
    const coord = pos.coords;
    const newLat = coord.latitude;
    const newLon = coord.longitude;
    const newAcc = coord.accuracy; 
    
    updateGPSPrecisionPerc(newAcc);

    // Initialisation / Calcul du delta de temps
    let dt = 0;
    if (lPos !== null) {
        dt = (now - lPos.timestamp) / 1000; 
    }
    
    // Calcul de la distance et de la vitesse 3D (Pythagore sur les deltas)
    let spdMS_3D = coord.speed || 0; // Vitesse GPS brute (m/s)
    let distanceTraveled = 0;
    let accelLong_LAST = 0;

    if (lPos !== null) {
        // Distance Haversine
        const R_EARTH = 6371e3; // Rayon moyen de la Terre en mètres
        const φ1 = lat * Math.PI / 180;
        const φ2 = newLat * Math.PI / 180;
        const Δφ = (newLat - lat) * Math.PI / 180;
        const Δλ = (newLon - lon) * Math.PI / 180;
        
        const a = Math.sin(Δφ/2) * Math.sin(Δφ/2) +
                  Math.cos(φ1) * Math.cos(φ2) *
                  Math.sin(Δλ/2) * Math.sin(Δλ/2);
        const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
        distanceTraveled = R_EARTH * c; // Distance Haversine (2D)

        // Correction pour la vitesse 3D / altitude
        const altDiff = (coord.altitude ?? 0) - (lPos.altitude ?? 0);
        spdMS_3D = Math.sqrt(spdMS_3D ** 2 + (altDiff / dt) ** 2);
        
        // Calcul d'accélération
        const spd_EKF_kmh = kSpd * KMH_MS;
        const new_spd_EKF_kmh = spdMS_3D * KMH_MS; // Utiliser la vitesse brute pour calculer l'accélération pour plus de réactivité
        accelLong_LAST = (new_spd_EKF_kmh - spd_EKF_kmh) / (dt * KMH_MS); // Accélération en m/s²
    }
    
    // Mise à jour de la distance totale (avec multiplicateur Nether)
    distM += distanceTraveled * netherMultiplier;

    // Application du filtre Kalman sur la VITESSE
    kalmanFilter(spdMS_3D, newAcc, dt);
    
    // Mise à jour de la vitesse max (EKF Stable)
    const spdKMH = kSpd * KMH_MS;
    if (spdKMH > maxSpd) maxSpd = spdKMH;
    
    // Enregistrement de la nouvelle position
    lat = newLat;
    lon = newLon;
    lPos = { 
        latitude: newLat, 
        longitude: newLon, 
        altitude: coord.altitude ?? 0,
        accuracy: newAcc, 
        timestamp: now,
        speedMS_3D: spdMS_3D,
        accelLong_LAST: accelLong_LAST,
        verticalSpeed: coord.verticalSpeed ?? 0
    };
    
    // Mise à jour des affichages GPS
    if ($('latitude')) $('latitude').textContent = lat.toFixed(6);
    if ($('longitude')) $('longitude').textContent = lon.toFixed(6);
    if ($('altitude')) $('altitude').textContent = `${(coord.altitude ?? 0).toFixed(2)} m`;
    if ($('gps-accuracy')) $('gps-accuracy').textContent = `${newAcc.toFixed(2)} m`;
    
    // Mise à jour de la carte (simplifiée)
    if (typeof L !== 'undefined' && map) {
        if (marker) marker.setLatLng([newLat, newLon]);
        else marker = L.marker([newLat, newLon]).addTo(map);
        map.setView([newLat, newLon], map.getZoom());
    }

    // Mise à jour de la navigation et de l'astro
    updateAstro(lat, lon);
    
    // Forcer la boucle lente pour mettre à jour les éléments non-fluides
    fastDOM();
}

function handleErr(err) {
    console.error(`Erreur GPS (${err.code}): ${err.message}`);
    const errMsg = $('error-message');
    if (errMsg) {
        errMsg.textContent = `Erreur GPS: ${err.message}. Code: ${err.code}.`;
        // Note: L'ID 'error-message' n'est pas dans le HTML, mais la console log l'enregistre.
    }
    // Si la précision est mauvaise, ralentir l'EKF et la vitesse affichée
    if (err.code === 2) { // POSITION_UNAVAILABLE
        kUncert = 5000; 
    }
}

/**
 * Fonctions de simulation pour les capteurs non-GPS
 */
function getCurrentDragMultiplier() {
    let mult = 1.0;
    if (selectedEnvironment === 'METAL') mult *= 1.2;
    if (selectedEnvironment === 'FOREST') mult *= 1.05;
    if (selectedEnvironment === 'CONCRETE') mult *= 1.1;
    
    if (selectedWeather === 'RAIN') mult *= 1.1;
    if (selectedWeather === 'SNOW') mult *= 1.15;
    if (selectedWeather === 'STORM') mult *= 1.3;
    
    return mult;
}

function initBattery() {
    if ('getBattery' in navigator) {
        navigator.getBattery().then(battery => {
            function updateBatteryInfo() {
                currentBatteryLevelPrecise = battery.level * 100;
                
                const status = battery.charging ? "Charge" : "Décharge";
                if ($('battery-indicator')) $('battery-indicator').textContent = status;
                
                // Simulation d'autonomie (batterie d'appareil de 1h max à 100%)
                const MAX_AUTONOMY_S = 3600; 
                deviceAutonomyS = battery.level * MAX_AUTONOMY_S * (battery.charging ? 5 : 1);
            }
            battery.addEventListener('chargingchange', updateBatteryInfo);
            battery.addEventListener('levelchange', updateBatteryInfo);
            updateBatteryInfo();
        });
    }
}

function getCDate() {
    return new Date();
}

function saveState() {
    const state = {
        distM: distM,
        maxSpd: maxSpd,
        distMStartOffset: distMStartOffset,
        tLat: tLat,
        tLon: tLon,
        netherMultiplier: netherMultiplier,
        isNightMode: isNightMode,
        externalBatteryLevel: externalBatteryLevel,
        externalAutonomyS: externalAutonomyS,
        lastNtpSync: lastNtpSync
    };
    localStorage.setItem(STATE_KEY, JSON.stringify(state));
}

function loadState() {
    const savedState = localStorage.getItem(STATE_KEY);
    if (savedState) {
        const state = JSON.parse(savedState);
        distM = state.distM || 0;
        maxSpd = state.maxSpd || 0;
        distMStartOffset = state.distMStartOffset || 0;
        tLat = state.tLat;
        tLon = state.tLon;
        netherMultiplier = state.netherMultiplier || 1;
        isNightMode = state.isNightMode || false;
        externalBatteryLevel = state.externalBatteryLevel;
        externalAutonomyS = state.externalAutonomyS;
        lastNtpSync = state.lastNtpSync || 0;
        
        document.body.classList.toggle('night-mode', isNightMode);
        if ($('nether-indicator')) $('nether-indicator').textContent = netherMultiplier === 1 ? 'DÉSACTIVÉ (1:1)' : `ACTIVÉ (1:${netherMultiplier})`;
    }
    }
// =================================================================
// BLOC 3/3 : CONTRÔLES, BOUCLES DOM ET FONCTIONS D'INITIALISATION
// =================================================================

/**
 * Formate l'autonomie en secondes en une chaîne lisible (h, min, sec).
 */
function formatAutonomy(seconds) {
    if (seconds === Infinity) return 'Pleine (Inf.)';
    if (seconds <= 0 || seconds === -1) return 'Inconnu / Épuisé';

    const h = Math.floor(seconds / 3600);
    const m = Math.floor((seconds % 3600) / 60);
    const s = Math.round(seconds % 60);

    if (h > 0) return `${h} h ${String(m).padStart(2, '0')} min`;
    if (m > 0) return `${m} min ${String(s).padStart(2, '0')} sec`;
    return `${s} sec`;
}

function setGPSMode(newMode) {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    currentGPSMode = newMode;
    const opts = GPS_OPTS[newMode]; 
    wID = navigator.geolocation.watchPosition(updateDisp, handleErr, opts);
    if ($('speed-source-indicator')) $('speed-source-indicator').textContent = `Source: ${newMode}`;
}

function updateAvgSpeed() {
    if (sTime === null || distM === 0) return 0;
    const elapS = (getCDate().getTime() - sTime) / 1000;
    return (distM / elapS) * KMH_MS; // Vitesse moyenne en KM/H
}

/**
 * Simule les données des capteurs (Lumière, Niveau à Bulle).
 */
function simulateSensorData() {
    const t = performance.now() / 1000;
    const spdFactor = kSpd / 50; 
    
    // Inclinaison (niveau à bulle)
    const tiltX = (Math.sin(t * 1.5) * 1.5 + spdFactor * 0.5).toFixed(1); 
    const tiltY = (Math.cos(t * 1.8) * 2.0).toFixed(1); 
    
    // Luminosité (ALS - simulation jour/nuit)
    const lightLux = isNightMode ? (Math.random() * 50 + 1) : (Math.random() * 5000 + 1000); 
    const MAX_LUX = 60000;
    const lightPerc = (lightLux / MAX_LUX * 100);
    
    // Simulation du niveau sonore (db - inversement proportionnel à la vitesse)
    const soundDb = Math.max(50, 80 + Math.sin(t / 4) * 10 - kSpd * 5);
    const MAX_DB = 120;
    const soundPerc = (soundDb / MAX_DB * 100); 
    
    return {
        tiltX: tiltX,
        tiltY: tiltY,
        lightLux: lightLux,
        lightPerc: lightPerc,
        soundPerc: soundPerc,
    };
}

/**
 * NOUVELLE BOUCLE : Mise à jour Ultra-Rapide (62.5 Hz / 16 ms)
 * Mise à jour de tous les éléments nécessitant une fluidité maximale
 * (Vitesse instantanée, Accélération, G-Force, Capteurs IMU/Simulés, Temps).
 */
function fastSensorUpdate() {
    const sSpd = kSpd < MIN_SPD ? 0 : kSpd; // Vitesse EKF (m/s)
    const spd3D_ms = lPos ? (lPos.speedMS_3D ?? 0) : 0; // Vitesse GPS brute (m/s)
    const accelLong = lPos ? (lPos.accelLong_LAST ?? 0) : 0; // Dernière accélération calculée (m/s²)
    const elapS = sTime !== null ? (getCDate().getTime() - sTime) / 1000 : 0;
    
    const sensorData = simulateSensorData();
    
    // --- VITESSE & ACCÉLÉRATION (Fluide) ---
    const spd3D_kmh = spd3D_ms * KMH_MS;
    
    // Vitesse 3D Stable (EKF)
    if ($('speed-stable')) $('speed-stable').textContent = `${(sSpd * KMH_MS).toFixed(5)} km/h`; 
    // Vitesse 3D Instantanée (GPS Brut)
    if ($('speed-3d-inst')) $('speed-3d-inst').textContent = `${spd3D_kmh.toFixed(5)} km/h`;
    
    // Vitesse en m/s et mm/s
    if ($('speed-ms')) $('speed-ms').textContent = `${sSpd.toFixed(2)} m/s | ${(sSpd * 1000).toFixed(0)} mm/s`;
    
    // Vitesse Long. (IMU) / Accélération
    if ($('accel-long')) $('accel-long').textContent = `${accelLong.toFixed(3)} m/s²`;
    // G-Force
    if ($('g-force')) $('g-force').textContent = `${(accelLong / G_ACCEL).toFixed(2)} G`;
    
    // --- CAPTEURS (Fluide) ---
    if ($('bubble-level')) $('bubble-level').textContent = `${sensorData.tiltX}°/${sensorData.tiltY}°`;
    if ($('illuminance-lux')) $('illuminance-lux').textContent = `${sensorData.lightLux.toFixed(0)} lux`;
    if ($('illuminance-perc')) $('illuminance-perc').textContent = `${sensorData.lightPerc.toFixed(1)} %`;
    if ($('perc-sound-db')) $('perc-sound-db').textContent = `${sensorData.soundPerc.toFixed(1)} %`;

    // --- TEMPS ÉCOULÉ ---
    if ($('time-elapsed')) $('time-elapsed').textContent = `${elapS.toFixed(2)} s`;
    
    // Mise à jour de l'heure locale
    if ($('local-time')) $('local-time').textContent = getCDate().toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit', second: '2-digit' });

    // Vitesse verticale (fluide, basée sur la dernière position GPS)
    const verticalSpeed = lPos ? (lPos.verticalSpeed ?? 0) : 0;
    if ($('vertical-speed')) $('vertical-speed').textContent = `${verticalSpeed.toFixed(2)} m/s`;
}

/**
 * BOUCLE LENTE : Mise à jour des informations secondaires (Batterie, Astro, Distances, États)
 * Exécutée à une fréquence fixe de 1 Hz (1000ms).
 */
function fastDOM() {
    const pNow = performance.now();
    
    // S'assurer que cette boucle ne s'exécute pas trop souvent (1Hz)
    if (fastDOM.lastSlowT && (pNow - fastDOM.lastSlowT) < DOM_SLOW_UPDATE_MS) {
        return;
    }
    fastDOM.lastSlowT = pNow;
    
    const sSpdKMH = (kSpd < MIN_SPD ? 0 : kSpd) * KMH_MS;
    const avgSpdKMH = updateAvgSpeed(); 

    // --- MISE À JOUR ASTRO ---
    updateAstro(lat ?? 48.8566, lon ?? 2.3522); 

    // -----------------------------------------------------------------
    // AFFICHAGE BATTERIE & AUTONOMIE
    // -----------------------------------------------------------------
    if (currentBatteryLevelPrecise !== null) {
        if ($('battery-level-perc')) $('battery-level-perc').textContent = `${currentBatteryLevelPrecise.toFixed(3)} %`;
        const autonomyStr = formatAutonomy(deviceAutonomyS);
        // Note: L'ID 'device-autonomy' n'est plus dans le HTML actuel, mais la donnée est calculée.
    }

    if (externalBatteryLevel !== null) {
        if ($('external-battery-level')) $('external-battery-level').textContent = `${externalBatteryLevel.toFixed(3)} %`;
    } 
    // -----------------------------------------------------------------

    // --- MISE À JOUR VITESSE (Secondaire) & DISTANCE ---
    if ($('speed-max')) $('speed-max').textContent = `${maxSpd.toFixed(5)} km/h`;
    if ($('speed-avg')) $('speed-avg').textContent = `${avgSpdKMH.toFixed(5)} km/h`;
    
    // Distance détaillée
    if ($('distance-km-m')) $('distance-km-m').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(2)} m`;
    if ($('distance-mm')) $('distance-mm').textContent = `${(distM * 1000).toFixed(0)} mm`;
    
    // Distance cosmique (basée sur le temps écoulé, non la distance parcourue)
    const elapS = sTime !== null ? (getCDate().getTime() - sTime) / 1000 : 0;
    const lightDistM = elapS * C_L;
    const lightYears = (lightDistM / 1000) / LIGHT_YEAR_KM;
    if ($('cosmic-light-s')) $('cosmic-light-s').textContent = `${elapS.toFixed(2)} s lumière`;
    if ($('cosmic-light-al')) $('cosmic-light-al').textContent = `${lightYears.toFixed(8)} al`;


    // Vitesse détaillée (pour les champs spécifiques)
    const perc_sound = (sSpdKMH / 1225 * 100); 
    if ($('speed-4d-perc')) $('speed-4d-perc').textContent = `${(sSpdKMH / C_L * 3.6).toFixed(5)} %`; // Vitesse 4D (c)
    if ($('perc-sound')) $('perc-sound').textContent = `${perc_sound.toFixed(5)} %`;
    
    // Force Traînée et Puissance (Utilise la vitesse EKF stable)
    const dragMult = getCurrentDragMultiplier(); 
    const dragForce = 0.5 * AIR_DENSITY * CDA_EST * dragMult * kSpd ** 2; 
    if ($('drag-force')) $('drag-force').textContent = `${dragForce.toFixed(1)} N`;
    if ($('drag-power-kw')) $('drag-power-kw').textContent = `${(dragForce * kSpd / 1000).toFixed(2)} kW`;
    
    // Précision GPS (Cohérence Kalman)
    const coherence_perc = Math.min(100 * (1 - Math.min(kUncert / 1.0, 1.0)), 100);
    if ($('speed-error-perc')) $('speed-error-perc').textContent = `${coherence_perc.toFixed(1)} %`; 
    
    // Mise à jour Météo (Pression, Temp, etc.)
    if ($('air-temp')) $('air-temp').textContent = `${(lastT_K - 273.15).toFixed(1)} °C`;
    if ($('pressure')) $('pressure').textContent = `${lastP_hPa.toFixed(2)} hPa`;
    if ($('humidity')) $('humidity').textContent = `${(lastH_perc * 100).toFixed(0)} %`;
    
    // Affichage de la fréquence DOM (fixée à 1.0 Hz)
    if ($('update-frequency')) $('update-frequency').textContent = '1.0 Hz (DOM)';
    if ($('mode-indicator')) $('mode-indicator').textContent = isNightMode ? 'NUIT 🌑' : 'JOUR ☀️';
    if ($('emergency-status')) $('emergency-status').textContent = emergencyStopActive ? 'ACTIF (Mode Sécurité)' : 'INACTIF';


    saveState(); 
}

function startGPS() {
    if (wID !== null) stopGPS(false);
    sTime = null; 
    setGPSMode('HIGH_FREQ'); // Commence en haute fréquence pour une accroche rapide
    
    if ($('start-btn')) $('start-btn').disabled = true;
    if ($('stop-btn')) $('stop-btn').disabled = false;
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
        netherMultiplier = 1;
        externalBatteryLevel = null; 
        externalAutonomyS = -1; 
        localStorage.removeItem(STATE_KEY); 
        if ($('nether-indicator')) $('nether-indicator').textContent = 'DÉSACTIVÉ (1:1)';
    }
    kSpd = 0; kUncert = 1000;
    lastReliableHeading = null; sTime = null; lPos = null; 
    if ($('time-elapsed')) $('time-elapsed').textContent = '0.00 s';
    fastDOM();
    fastSensorUpdate();
}

function resetMax() { maxSpd = 0; }

function setTarget() {
    const defaultLat = lat ?? 48.8566;
    const defaultLon = lon ?? 2.3522;
    const newLatStr = prompt(`Entrez la Latitude cible (actuel: ${tLat ?? defaultLat.toFixed(6)}) :`);
    if (newLatStr !== null && !isNaN(parseFloat(newLatStr))) { tLat = parseFloat(newLatStr); }
    const newLonStr = prompt(`Entrez la Longitude cible (actuel: ${tLon ?? defaultLon.toFixed(6)}) :`);
    if (newLonStr !== null && !isNaN(parseFloat(newLonStr))) { tLon = parseFloat(newLonStr); }
}

function toggleNightMode() {
    isNightMode = !isNightMode;
    document.body.classList.toggle('night-mode', isNightMode);
    if ($('mode-indicator')) $('mode-indicator').textContent = isNightMode ? 'NUIT 🌑' : 'JOUR ☀️';
}

function netherToggle() {
    if (netherMultiplier === 1) {
        netherMultiplier = 8;
        if ($('nether-indicator')) $('nether-indicator').textContent = 'ACTIVÉ (1:8)';
    } else {
        netherMultiplier = 1;
        if ($('nether-indicator')) $('nether-indicator').textContent = 'DÉSACTIVÉ (1:1)';
    }
}

function setExternalBatteryLevel() {
    const currentLevel = externalBatteryLevel === null ? 0 : externalBatteryLevel.toFixed(3);
    const newLevelStr = prompt(`Entrez le niveau de la batterie externe (en pourcentage, ex: 99.999). (Actuel: ${currentLevel} %) :`);

    if (newLevelStr !== null && !isNaN(parseFloat(newLevelStr))) {
        const newLevel = parseFloat(newLevelStr);
        externalBatteryLevel = Math.max(0, Math.min(100, newLevel)); // Clamp entre 0 et 100

        // Simulation d'une autonomie basée sur le niveau (Correction de la cohérence)
        if (externalBatteryLevel === 100) {
            externalAutonomyS = Infinity; // SEULEMENT 100% est Infini
        } else if (externalBatteryLevel > 0) {
            // Autonomie max simulée à 3 heures = 10800 secondes
            externalAutonomyS = (externalBatteryLevel / 100) * 10800; 
        } else {
            externalAutonomyS = 0;
        }
    } else if (newLevelStr !== null) {
        alert("Valeur invalide. Veuillez entrer un nombre.");
    }
}


function initAll() { 
    loadState(); 
    resetDisp(false); 
    initBattery(); 
    
    // Initialisation du mode jour/nuit
    if ($('mode-indicator')) $('mode-indicator').textContent = isNightMode ? 'NUIT 🌑' : 'JOUR ☀️';
    
    // 1. Initialisation de la BOUCLE RAPIDE des capteurs (Fluidité)
    if (sensorID === null) {
        sensorID = setInterval(fastSensorUpdate, 16); // 16ms = 62.5 Hz
    }
    
    // 2. Initialisation de la BOUCLE LENTE DOM
    if (domID === null) {
        customDOMIntervalMS = 1000; // 1Hz
        domID = setInterval(fastDOM, customDOMIntervalMS); 
        domID.intervalMS = customDOMIntervalMS;
        fastDOM.lastSlowT = 0; 
        if ($('update-frequency')) $('update-frequency').textContent = '1.0 Hz (DOM)';
    }

    // Initialisation de la carte (simplifiée)
    if (typeof L !== 'undefined' && $('map')) {
        map = L.map('map').setView([48.8566, 2.3522], 13); // Paris par défaut
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OSM</a>',
            maxZoom: 18,
        }).addTo(map);
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
    if ($('toggle-mode-btn')) $('toggle-mode-btn').addEventListener('click', toggleNightMode);
    if ($('nether-toggle-btn')) $('nether-toggle-btn').addEventListener('click', netherToggle);
    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', () => {
        emergencyStopActive = !emergencyStopActive;
        if ($('emergency-status')) $('emergency-status').textContent = emergencyStopActive ? 'ACTIF (Mode Sécurité)' : 'INACTIF';
    });
    if ($('freq-manual-btn')) $('freq-manual-btn').addEventListener('click', setExternalBatteryLevel);

    // Événements des select
    if ($('environment-select')) $('environment-select').addEventListener('change', (e) => selectedEnvironment = e.target.value);
    if ($('weather-select')) $('weather-select').addEventListener('change', (e) => selectedWeather = e.target.value);
    
    // Premier appel des boucles pour affichage immédiat
    fastDOM();
    fastSensorUpdate();
}

document.addEventListener('DOMContentLoaded', initAll);
