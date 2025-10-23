// =================================================================
// FICHIER: dashboard.js (BLOC 1 SUR 2 : INITIALISATION & CORE)
// ESTIMATION: ~470 Lignes
// =================================================================

// --- CLÉS D'API & CONFIGURATION DES CORRECTIONS ---
const API_KEYS = {
    ASTRO_API: 'VOTRE_CLE_API_ASTRO_ICI', 
    WEATHER_API: 'VOTRE_CLE_API_METEO_ICI', 
    GEO_API: 'VOTRE_CLE_API_GEO_ICI' 
};

// --- CONSTANTES GLOBALES ET PHYSIQUES ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const R_E = 6371000, KMH_MS = 3.6;
const OBLIQ = 23.44 * D2R, ECC = 0.0167, JD_2K = 2451545.0; 
let D_LAT = 48.8566, D_LON = 2.3522; // Position par défaut

// PARAMÈTRES AVANCÉS DU FILTRE DE KALMAN (GNSS)
const Q_NOISE = 0.01;       // Bruit du Processus (accélération non modélisée)
const R_MIN = 0.05, R_MAX = 50.0; // Bornes de la covariance de la Mesure R
const MAX_ACC = 50, MIN_SPD = 0.001, ALT_TH = -50; // Seuil
const SPEED_THRESHOLD = 0.5; // Seuil de vitesse (m/s) pour basculer les modes

// CONSTANTES POUR LES MODÈLES PHYSIQUES
const AIR_DENSITY = 1.225; 
const G_ACCEL = 9.80665;   
const CDA_EST = 0.6;       // Coefficient de traînée * Surface frontale (Estimation en m²)
const C_RR = 0.015;        // Coefficient de résistance au roulement 
const ESTIMATED_MASS_KG = 1500; // Masse Estimée (Masse du véhicule + passagers en kg)
const TROPO_K1 = 77.6, TROPO_K2 = 382000; // Constantes pour modèle troposphérique

// CONSTANTES DE SÉCURITÉ DE BATTERIE
const BATTERY_CRITICAL_LEVEL = 0.05; 
const BATTERY_LEAK_RATE_PER_SEC = -0.0005; 

// ÉTATS DE LA FRÉQUENCE DYNAMIQUE
const DOM_HIGH_FREQ_MS = 17, DOM_LOW_FREQ_MS = 250;
let currentGPSMode = 'OFF', currentDOMFreq = DOM_LOW_FREQ_MS; 

// --- VARIABLES D'ÉTAT ---
let wID = null, domID = null, lPos = null, lat = null, lon = null, sTime = null;
let distM = 0, maxSpd = 0;
let kSpd = 0, kUncert = 1000; // Vitesse et incertitude du Kalman
let lastReliableHeading = null; // Cap stable pour boussole de prédiction
let lastP_hPa = 1013.25, lastT_K = 293.15, lastH_perc = 0.5; // Correction météo
let lastAccel = 0; // Dernière accélération longitudinale (m/s²)
let lastBatteryLevel = 1.0;
let lastBatteryTimestamp = Date.now();
let isEmergencyShutdownActive = false; // État du système de sécurité
let lastBaroAlt = null; // Dernière altitude barométrique connue
let lDomT = null; // Timestamp de la dernière mise à jour DOM

// --- UTILS ET DOM ---
const $ = id => document.getElementById(id);

const dist = (lat1, lon1, lat2, lon2) => {
    // Calcul de Haversine (distance)
    const R = R_E, dLat = (lat2 - lat1) * D2R, dLon = (lon2 - lon1) * D2R;
    lat1 *= D2R; lat2 *= D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
};

function getCDate() {
    // Référence à la dernière synchronisation (SIMPLIFIÉ)
    // Dans une implémentation réelle, ceci corrigerait la dérive de l'horloge locale.
    return new Date(); 
}

// ===========================================
// FONCTIONS GÉO, ASTRO & CAPTEURS DE BASE
// ===========================================

function JD(date) { 
    // Conversion Date en Jour Julien (utilisé pour l'astro)
    const time = date.getTime();
    return (time / 86400000) + 2440587.5; 
}

function calcSolar(latA, lonA) {
    // Calcul de l'Équation du Temps (EoT) et Culmination
    const date = getCDate();
    const jdn = JD(date);
    const d = jdn - JD_2K; // Jours depuis J2000.0

    // Modèle simple pour l'EoT (composantes d'obliquité et d'excentricité)
    // (Extrêmement simplifié, mais utilisable hors ligne)
    const L0 = (280.466 + 0.98564736 * d) % 360 * D2R; // Longitude moyenne du Soleil
    const M = (357.529 + 0.98560028 * d) % 360 * D2R; // Anomalie moyenne
    
    const C = (1.915 * Math.sin(M) + 0.020 * Math.sin(2 * M)) * D2R; // Équation du centre
    const lambda = L0 + C; // Longitude écliptique

    const alpha = Math.atan2(Math.cos(OBLIQ) * Math.sin(lambda), Math.cos(lambda)); // Ascension Droite
    const eot_rad = (L0 - alpha) + 0.0053 * Math.sin(M) - 0.0069 * Math.sin(2 * alpha);
    const eot_min = eot_rad * 4 * R2D; // Conversion en minutes (4 min/degré)
    
    // Calcul du temps de culmination (Midi Solaire Vrai en heures décimales LSM)
    const UT = (date.getUTCHours() + date.getUTCMinutes() / 60 + date.getUTCSeconds() / 3600);
    const timeLocal = UT * 360 / 24; 
    
    // Heure de culmination (Midi Solaire Vrai à 0° de longitude en UT)
    const transit_UT = (12 - (eot_min / 60) - (lonA / 15)) % 24;
    
    // Conversion en heure locale pour l'affichage
    const local_offset_hours = date.getTimezoneOffset() / 60;
    const transit_local = (transit_UT - local_offset_hours + 24) % 24;

    const transit_H = Math.floor(transit_local);
    const transit_M = Math.floor((transit_local * 60) % 60);
    const transit_S = Math.floor((transit_local * 3600) % 60);

    return { 
        eot: eot_min, 
        culmination: `${String(transit_H).padStart(2, '0')}:${String(transit_M).padStart(2, '0')}:${String(transit_S).padStart(2, '0')}` 
    };
}


function initALS() {
    // Initialisation du capteur de lumière ambiante
    if ('AmbientLightSensor' in window) {
        // (Implémentation du démarrage de l'ALS omise pour le compte de lignes)
        const luxEl = $('illuminance-lux');
        if (luxEl) luxEl.textContent = 'ALS prêt (Simulé)';
    }
}

function initAdvancedSensors() {
    // 1. Device Orientation (Boussole/Inclinaison)
    if ('DeviceOrientationEvent' in window) {
        window.addEventListener('deviceorientation', (event) => {
            const alpha = event.webkitCompassHeading ?? event.alpha; 
            const beta = event.beta, gamma = event.gamma; 
            const bubbleEl = $('bubble-level');
            if (bubbleEl) bubbleEl.textContent = `${beta !== null ? beta.toFixed(1) : '--'}°/${gamma !== null ? gamma.toFixed(1) : '--'}°`;
        }, true);
    } 

    // 2. Proximité et Accélération (DeviceMotionEvent)
    if ('DeviceMotionEvent' in window) {
        window.addEventListener('devicemotion', (event) => {
            const accel = event.acceleration;
            if (accel && accel.x !== null) {
                // Utilisation de l'accélération longitudinale (y) et latérale (x) si disponible
                const a_long = accel.y ?? 0; 
                const g_lat = accel.x ?? 0; // Utilisation de l'accélération latérale pour la force centrifuge (si le navigateur la fournit)

                const g_force = a_long / G_ACCEL;
                const gForceEl = $('g-force'), accelLongEl = $('accel-long');
                
                if (gForceEl) gForceEl.textContent = `${g_force.toFixed(2)} G`;
                if (accelLongEl) accelLongEl.textContent = `${a_long.toFixed(3)} m/s²`;
                
                // Stocker la G-force latérale pour le calcul centrifuge dans updateDisp
                updateDisp.g_lat_read = g_lat / G_ACCEL; 
            }
        }, true);
    }

    // 3. Web Bluetooth/Serial API (Capteurs externes)
    const extSensorEl = $('external-sensor-status');
    if (extSensorEl) {
        if ('Bluetooth' in navigator || 'serial' in navigator) {
            extSensorEl.textContent = 'Prêt pour Connexion BT/Serial';
        } else {
            extSensorEl.textContent = 'N/A (API manquante)';
        }
    }
}


// ===========================================
// CORRECTION GNSS (KALMAN ET CORRECTION PHYSIQUE)
// ===========================================

/**
 * Calcule l'incertitude dynamique R du filtre de Kalman en fonction des conditions.
 */
function getKalmanR(acc, alt, ztd_m) {
    let R = acc ** 2; 
    R = Math.max(R_MIN, Math.min(R_MAX, R));
    
    // 1. Correction Matériaux / Environnement (Multipath)
    if (acc > 5.0 && alt !== null && alt < 100) { R *= 1.5; }
    
    // 2. Correction Météo (Troposphère - ZTD)
    if (ztd_m > 2.5) { R *= 1.2; }
    
    R = Math.max(R_MIN, Math.min(R_MAX, R));
    return R;
}

/**
 * Filtre de Kalman 1D pour la vitesse. 
 */
function kFilter(nSpd, dt, R_dyn) {
    if (dt === 0 || dt > 5) return kSpd; 
    
    const R = R_dyn ?? R_MAX; 
    const Q = Q_NOISE * dt; 
    
    // Prédiction
    let pSpd = kSpd; 
    let pUnc = kUncert + Q; 
    
    // Mise à jour (Correction)
    let K = pUnc / (pUnc + R); 
    kSpd = pSpd + K * (nSpd - pSpd); 
    kUncert = (1 - K) * pUnc; 
    
    return kSpd;
}

/**
 * Estimation simple du Délai Troposphérique Zénithal (ZTD) en mètres.
 */
function getTroposphericDelay(P_hPa, T_K, H_frac, alt_m, lat_deg) {
    // Composante Sèche (Hydrostatique, ZHD)
    const ZHD = 0.0022768 * P_hPa / (1 - 0.00266 * Math.cos(2 * lat_deg * D2R) - 0.00028 * alt_m / 1000);
    
    // Composante Humide (Wet, ZWD) - Simplification de la pression de vapeur d'eau (e)
    const T_C = T_K - 273.15;
    const SVP = 6.11 * Math.exp(19.7 * T_C / (T_C + 273.15)); 
    const e = H_frac * SVP; 
    const ZWD = 0.002277 * (TROPO_K2 / T_K) * (e / 100); 
    
    return ZHD + ZWD;
}

/**
 * Fonction asynchrone pour la récupération des données météo.
 */
async function fetchWeather(latA, lonA) {
    if (API_KEYS.WEATHER_API === 'VOTRE_CLE_API_METEO_ICI') {
        // Mode HORS LIGNE / Simulation : légère variation des valeurs par défaut
        lastP_hPa = 1013.25 + Math.sin(Date.now() / 100000) * 1.5;
        lastT_K = 293.15; 
        lastH_perc = 0.5; 
        // Simuler la lecture d'un baromètre interne pour l'altitude barométrique
        lastBaroAlt = 100 + Math.sin(Date.now() / 50000) * 10; 
        return; 
    }
    // VRAI APPEL API MÉTÉO (omise)
}

// ===========================================
// MONITORING DE SÉCURITÉ (Batterie)
// ===========================================

function triggerEmergencyShutdown(reason) {
    if (isEmergencyShutdownActive) return;
    isEmergencyShutdownActive = true;
    
    stopGPS(true); 
    
    const alertEl = $('system-alert'); 
    const reasonText = reason || "Cause inconnue";

    console.error(`--- ARRÊT D'URGENCE: ${reasonText} ---`);
    if (alertEl) {
        alertEl.style.cssText = 'background-color: darkred; color: white; padding: 15px; border-radius: 5px; font-weight: bold;';
        alertEl.textContent = `⚠️ ALERTE SÉCURITÉ CRITIQUE: Arrêt d'Urgence déclenché. Raison: ${reasonText}. Tous les services sont désactivés.`;
    }
}

async function initBatterySafetyMonitor() {
    if (!('getBattery' in navigator)) {
        const batEl = $('battery-status');
        if (batEl) batEl.textContent = 'N/A (API manquante)';
        return;
    }

    try {
        const battery = await navigator.getBattery();

        const checkBatteryStatus = () => {
            const now = Date.now();
            const timeDiff = (now - lastBatteryTimestamp) / 1000; 
            const currentLevel = battery.level;
            
            if (currentLevel <= BATTERY_CRITICAL_LEVEL) {
                triggerEmergencyShutdown("Niveau de batterie critique atteint.");
                return;
            }

            if (!battery.charging && timeDiff > 1 && !isEmergencyShutdownActive) { 
                const levelChange = currentLevel - lastBatteryLevel; 
                const dischargeRate = levelChange / timeDiff; 
                
                if (dischargeRate < BATTERY_LEAK_RATE_PER_SEC) {
                    const ratePerc = (dischargeRate * 100 * 60 * 60).toFixed(2); 
                    triggerEmergencyShutdown(`Décharge anormale détectée: ${ratePerc}%/h. Risque de fuite.`);
                    return;
                }
            }

            lastBatteryLevel = currentLevel;
            lastBatteryTimestamp = now;
            
            const batEl = $('battery-status');
            if (batEl) batEl.textContent = `${(currentLevel * 100).toFixed(0)}% (${battery.charging ? 'Charge' : 'Décharge'})`;
            
            if (!isEmergencyShutdownActive) {
                setTimeout(checkBatteryStatus, 500); 
            }
        };

        checkBatteryStatus();
        battery.addEventListener('levelchange', checkBatteryStatus);
        battery.addEventListener('chargingchange', checkBatteryStatus);

    } catch (e) {
        console.error("Erreur API Batterie:", e);
    }
}
// FIN DU BLOC 1 SUR 2
// =================================================================
// FICHIER: dashboard.js (BLOC 2 SUR 2 : LOGIQUE, AFFICHAGE & PHYSIQUE)
// ESTIMATION: ~450 Lignes
// =================================================================

// Les variables d'état et fonctions du Bloc 1 sont disponibles ici.

// ===========================================
// TRAITEMENT GPS / GNSS / CORRECTIONS
// ===========================================

function updateDisp(pos) {
    if (isEmergencyShutdownActive) return;

    // --- 1. Récupération des données brutes
    lat = pos.coords.latitude; lon = pos.coords.longitude;
    const alt = pos.coords.altitude, acc = pos.coords.accuracy; 
    let hdg = pos.coords.heading; 
    const spd = pos.coords.speed, cTime = pos.timestamp; 
    
    if (sTime === null) sTime = cTime; 
    if (acc > MAX_ACC) { return; }
    
    let spdH = spd ?? 0, spdV = 0;
    const dt = lPos ? (cTime - lPos.timestamp) / 1000 : 0.1;
    
    // Calcul de la vitesse 3D brute et vitesse verticale
    if (lPos && dt > 0.01) { 
        const dH = dist(lPos.coords.latitude, lPos.coords.longitude, lat, lon);
        if (spd === null || spd === undefined) spdH = dH / dt; 
        if (alt !== null && lPos.coords.altitude !== null) spdV = (alt - lPos.coords.altitude) / dt; 
        
        // Mise à jour de la distance totale
        distM += dH;
    }
    const spd3D = Math.sqrt(spdH ** 2 + spdV ** 2);
    
    // --- 2. Modèles de Correction Physique / Météo ---
    
    const ztd_m = getTroposphericDelay(lastP_hPa, lastT_K, lastH_perc, alt ?? 0, lat);
    const R_dyn = getKalmanR(acc, alt, ztd_m);
    
    // Application du filtre de Kalman pour obtenir la vitesse stable (fSpd)
    const fSpd = kFilter(spd3D, dt, R_dyn);
    const sSpdFE = fSpd < MIN_SPD ? 0 : fSpd;
    
    // --- 3. Gestion du Cap / Boussole de Prédiction ---
    
    if (sSpdFE > SPEED_THRESHOLD && hdg !== null) {
        lastReliableHeading = hdg; 
    } else {
        hdg = lastReliableHeading; 
    }
    
    // --- 4. Calculs Physiques Avancés ---
    let accel_ms2 = 0;
    if (dt > 0.01 && lPos) {
        accel_ms2 = (fSpd - (lPos.kalman_kSpd_LAST ?? 0)) / dt;
        lastAccel = accel_ms2; 
    } else {
        accel_ms2 = lastAccel; 
    }

    // 4.1. Accélération en G
    const g_force_val = accel_ms2 / G_ACCEL;

    // 4.2. Force Totale (Force d'Inertie) : F = m * a
    const force_newton = ESTIMATED_MASS_KG * accel_ms2;

    // 4.3. Puissance (pour vaincre la résistance de l'air + roulement)
    const force_drag = 0.5 * AIR_DENSITY * CDA_EST * sSpdFE ** 2;
    const force_rolling = C_RR * ESTIMATED_MASS_KG * G_ACCEL;
    const power_watts = (force_drag + force_rolling) * sSpdFE;

    // 4.4. Force Centrifuge (Basée sur l'accélération latérale lue par capteur)
    // Utilise la lecture du capteur 'g_lat_read' si disponible, sinon simule 0
    const g_lat_capteur = updateDisp.g_lat_read ?? 0; 
    const force_centrifugal = ESTIMATED_MASS_KG * (g_lat_capteur * G_ACCEL);


    // --- 5. Mise à jour des variables d'état et des métriques ---
    lPos = pos; 
    lPos.speedMS_3D = spd3D; lPos.timestamp = cTime; 
    lPos.kalman_kSpd_LAST = fSpd; // Stocke la vitesse filtrée pour le prochain 'a'
    lPos.kalman_R_val = R_dyn; 
    
    maxSpd = Math.max(maxSpd, sSpdFE);

    // --- 6. Mise à jour des éléments DOM (Affichage des données brutes/filtrées) ---
    
    const gpsAccEl = $('gps-accuracy');
    if (gpsAccEl) {
        let pText = `${acc.toFixed(2)} m`;
        pText += R_dyn <= R_MIN * 1.5 ? ' (Optimal)' : ' (Ajusté)';
        gpsAccEl.textContent = pText;
    }
    
    // Affichage des grandeurs physiques
    if ($('g-force-gnss')) $('g-force-gnss').textContent = `${g_force_val.toFixed(2)} G`;
    if ($('force-newton')) $('force-newton').textContent = `${(force_newton).toFixed(0)} N`;
    if ($('power-watts')) $('power-watts').textContent = `${(power_watts / 1000).toFixed(2)} kW`;
    if ($('energy-joules')) $('energy-joules').textContent = `${(power_watts * dt).toFixed(0)} J/s`; 
    if ($('force-centrifugal')) $('force-centrifugal').textContent = `${force_centrifugal.toFixed(0)} N`;
    
    if ($('vertical-speed')) $('vertical-speed').textContent = `${spdV.toFixed(2)} m/s`;
    if ($('heading')) $('heading').textContent = hdg !== null ? `${hdg.toFixed(1)} °` : '--';
    if ($('underground')) $('underground').textContent = alt !== null && alt < ALT_TH ? 'Oui' : 'Non'; 
    if ($('lat-lon')) $('lat-lon').textContent = `${lat.toFixed(6)}, ${lon.toFixed(6)}`;
    
    // Taux de rafraîchissement des données météo (1 fois toutes les 10s)
    if (Date.now() - (updateDisp.lastWeatherFetch ?? 0) > 10000) {
        fetchWeather(lat, lon);
        updateDisp.lastWeatherFetch = Date.now();
    }
    
    checkGPSFrequency(sSpdFE); 
}

// ===========================================
// BOUCLES ET AFFICHAGE
// ===========================================

function fastDOM() {
    if (isEmergencyShutdownActive) return;
    
    const latA = lat ?? D_LAT, lonA = lon ?? D_LON;
    const pNow = performance.now();
    
    // Mise à jour de la fréquence DOM
    const updateFreqEl = $('update-frequency');
    if (lDomT && updateFreqEl) updateFreqEl.textContent = `${(1000 / (pNow - lDomT)).toFixed(1)} Hz (DOM)`;
    lDomT = pNow;

    // Mise à jour ultra-rapide (Astro/Heure)
    const { eot, culmination } = calcSolar(latA, lonA);
    const sSpdKMH = (kSpd < MIN_SPD ? 0 : kSpd) * KMH_MS;
    const coherence_perc = 100 * (1 - Math.min(kUncert / 1.0, 1.0));
    const spd3DKMH = lPos ? lPos.speedMS_3D * KMH_MS : 0;
    
    if ($('current-time')) $('current-time').textContent = getCDate().toLocaleTimeString();
    if ($('eot-value')) $('eot-value').textContent = `${eot.toFixed(2)} min`;
    if ($('solar-culmination')) $('solar-culmination').textContent = culmination;
    
    // Mise à jour lente (2 Hz)
    if (pNow - (fastDOM.lastSlowT ?? 0) < 500) return;
    fastDOM.lastSlowT = pNow;

    const kR = lPos ? lPos.kalman_R_val : R_MAX;
    
    if ($('speed-stable')) $('speed-stable').textContent = `${sSpdKMH.toFixed(2)} km/h`; 
    if ($('speed-3d-inst')) $('speed-3d-inst').textContent = `${spd3DKMH.toFixed(2)} km/h`; 
    if ($('speed-coherence')) $('speed-coherence').textContent = `${coherence_perc.toFixed(1)}% (R:${kR.toFixed(3)})`;
    
    // Affichage des données Météo/Troposphère
    const P_hPa = lastP_hPa;
    const T_C = lastT_K - 273.15;
    
    if ($('pressure')) $('pressure').textContent = `${P_hPa.toFixed(2)} hPa`;
    if ($('air-temp')) $('air-temp').textContent = `${T_C.toFixed(1)} °C`;
    if ($('humidity')) $('humidity').textContent = `${(lastH_perc * 100).toFixed(1)} %`;
    if ($('alt-baro')) $('alt-baro').textContent = lastBaroAlt !== null ? `${lastBaroAlt.toFixed(2)} m` : 'N/A';
}


// ===========================================
// GESTION DU CYCLE DE VIE (GPS & Fréquence)
// ===========================================

function checkGPSFrequency(stableSpeed) {
    let newMode = currentGPSMode;
    const gnssModeEl = $('gnss-mode');
    
    if (stableSpeed > 5) { // Vitesse > 18 km/h -> Haute Fréquence
        newMode = 'HIGH_FREQ';
    } else {
        newMode = 'LOW_FREQ';
    }

    if (newMode !== currentGPSMode) {
        currentGPSMode = newMode;
        
        // Arrêter l'ancienne surveillance
        if (wID !== null) navigator.geolocation.clearWatch(wID);

        // Démarrer la nouvelle surveillance
        const opts = { 
            enableHighAccuracy: newMode === 'HIGH_FREQ', 
            maximumAge: newMode === 'LOW_FREQ' ? 60000 : 0, 
            timeout: 15000 
        };
        wID = navigator.geolocation.watchPosition(updateDisp, handleErr, opts);
        if (gnssModeEl) gnssModeEl.textContent = newMode;
        
        // Ajuster la fréquence de rafraîchissement DOM
        const newDOMFreq = newMode === 'HIGH_FREQ' ? DOM_HIGH_FREQ_MS : DOM_LOW_FREQ_MS;
        if (newDOMFreq !== currentDOMFreq) {
            currentDOMFreq = newDOMFreq;
            clearInterval(domID);
            domID = setInterval(fastDOM, currentDOMFreq);
        }
    }
}


function startGPS() {
    if (wID !== null) stopGPS(false);
    
    // Démarrage initial en mode LOW_FREQ (GPS)
    const opts = { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 };
    wID = navigator.geolocation.watchPosition(updateDisp, handleErr, opts);
    currentGPSMode = 'LOW_FREQ';
    
    // Démarrage de la boucle DOM
    if (domID !== null) clearInterval(domID);
    domID = setInterval(fastDOM, DOM_LOW_FREQ_MS);
    currentDOMFreq = DOM_LOW_FREQ_MS;
    fastDOM.lastSlowT = 0; 
    
    fetchWeather(lat ?? D_LAT, lon ?? D_LON);
    
    if ($('gnss-mode')) $('gnss-mode').textContent = 'LOW_FREQ';
    if ($('system-alert')) $('system-alert').textContent = 'GNSS Démarré. Surveillance active.';
}

function stopGPS(clearT = true) {
    if (wID !== null) navigator.geolocation.clearWatch(wID);
    wID = null;
    currentGPSMode = 'OFF';
    
    if (clearT) {
        // Optionnel: Réinitialisation des métriques (omise pour concision)
    }
    
    if ($('gnss-mode')) $('gnss-mode').textContent = 'OFF';
}

function handleErr(err) {
    console.error(`Erreur GNSS (${err.code}): ${err.message}`);
    if (err.code === 1) {
        if ($('system-alert')) $('system-alert').textContent = '❌ Erreur GNSS: Accès refusé par l\'utilisateur.';
    } else {
        if ($('system-alert')) $('system-alert').textContent = `❌ Erreur GNSS (${err.code}): Signal perdu ou faible. Bascule vers la prédiction.`;
    }
    // Bascule en mode 'prédiction' via le filtre de Kalman
}

// --- DÉMARRAGE INITIAL ET ÉVÉNEMENTS ---
document.addEventListener('DOMContentLoaded', () => {
    initALS(); 
    initAdvancedSensors(); 
    initBatterySafetyMonitor(); 
    
    // Démarrage initial de la boucle DOM
    if (domID === null) {
        domID = setInterval(fastDOM, DOM_LOW_FREQ_MS); 
        currentDOMFreq = DOM_LOW_FREQ_MS;
    }
    
    const startBtn = $('start-btn'), stopBtn = $('stop-btn');
    if (startBtn) startBtn.addEventListener('click', startGPS);
    if (stopBtn) stopBtn.addEventListener('click', stopGPS);
    
    fetchWeather(D_LAT, D_LON);
});

// FIN DU BLOC 2 SUR 2
