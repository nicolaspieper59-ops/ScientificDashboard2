// =================================================================
// FICHIER JS PARTIE 1/2 : gnss-dashboard-part1.js
// Contient constantes, variables d'état, NTP, et les filtres de Kalman.
// =================================================================

// --- CLÉS D'API & PROXY VERCEL ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app"; 
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc"; 

// --- CONSTANTES GLOBALES ET INITIALISATION ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458; // Vitesse de la lumière (m/s)
const R_E = 6371000;   // Rayon terrestre moyen (m)
const KMH_MS = 3.6;    // Conversion m/s vers km/h
const C_S = 343;       // Vitesse du son dans l'air (m/s)
const G_ACC = 9.80665; // Gravité standard (m/s²)
const MC_DAY_MS = 72 * 60 * 1000; // Durée d'un jour Minecraft en ms
const VEHICLE_MASS_KG_DEFAULT = 70.0; // Masse du véhicule par défaut

const J1970 = 2440588, J2000 = 2451545; 
const dayMs = 1000 * 60 * 60 * 24;      
const MIN_DT = 0.01; 
const GPS_OPTS = {
    HIGH_FREQ: { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
    LOW_FREQ: { enableHighAccuracy: false, maximumAge: 120000, timeout: 120000 }
};

// PARAMÈTRES AVANCÉS DU FILTRE DE KALMAN
const Q_NOISE = 0.01;       
const R_MIN = 0.05, R_MAX = 50.0; 
const MAX_ACC = 200, MIN_SPD = 0.05, ALT_TH = -50; 
const NETHER_RATIO = 8; 
const MAX_PLAUSIBLE_ACCEL = 20.0; 
const Q_ALT_NOISE = 0.1; 
const R_ALT_MIN = 0.1;  
const ACC_DAMPEN_LOW = 5.0; 
const ACC_DAMPEN_HIGH = 50.0; 

// FACTEURS ENVIRONNEMENTAUX POUR LA CORRECTION KALMAN (Matériaux)
const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0 },
    'METAL': { R_MULT: 2.5 },      
    'FOREST': { R_MULT: 1.5 },     
    'CONCRETE': { R_MULT: 3.0 },   
};
const DOM_SLOW_UPDATE_MS = 1000; 

// --- VARIABLES D'ÉTAT (AJOUT IMU & Contrôles) ---
let wID = null, domID = null, lPos = null, lat = null, lon = null, sTime = null;
let distM = 0, distMStartOffset = 0, maxSpd = 0;
let kSpd = 0, kUncert = 1000; 
let timeMoving = 0; 
let lServH = null, lLocH = null; 
let lastFSpeed = 0; 
let kAlt = null;      // Altitude filtrée par Kalman
let kAltUncert = 10;  // Incertitude de l'altitude

let currentGPSMode = 'HIGH_FREQ'; 
let emergencyStopActive = false;
let netherMode = false;
let selectedEnvironment = 'NORMAL'; 

let lastP_hPa = null, lastT_K = null, lastH_perc = null; 

// NOUVELLES VARIABLES IMU
let imuID = null; // ID du listener DeviceMotion
let lastAccelX = 0, lastAccelY = 0; // Dernières accélérations
let vehicleMass = VEHICLE_MASS_KG_DEFAULT; // Masse du véhicule actuelle

// --- REFERENCES DOM ---
const $ = id => document.getElementById(id);

// ===========================================
// FONCTIONS GÉO & UTILS
// ===========================================

// ... (dist, syncH, getCDate, kFilter, kFilterAltitude, getKalmanR restent inchangés)

/** Calcule la distance horizontale (Haversine). */
const dist = (lat1, lon1, lat2, lon2) => {
    const R = R_E, dLat = (lat2 - lat1) * D2R, dLon = (lon2 - lon1) * D2R;
    lat1 *= D2R; lat2 *= D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
};

// ===========================================
// SYNCHRONISATION HORAIRE PAR SERVEUR (UTC/Atomique)
// ===========================================

async function syncH() { 
    if ($('local-time')) $('local-time').textContent = 'Synchronisation UTC...';
    const localStartPerformance = performance.now(); 

    try {
        const response = await fetch(SERVER_TIME_ENDPOINT, { cache: "no-store", mode: "cors" });
        if (!response.ok) throw new Error(`Server time sync failed: ${response.statusText}`);
        
        const localEndPerformance = performance.now(); 
        const serverData = await response.json(); 
        
        const utcTimeISO = serverData.datetime; 
        const serverTimestamp = Date.parse(utcTimeISO); 
        
        const RTT = localEndPerformance - localStartPerformance;
        const latencyOffset = RTT / 2;

        lServH = serverTimestamp + latencyOffset; 
        lLocH = performance.now(); 
        console.log(`Synchronisation UTC Atomique réussie. Latence corrigée: ${latencyOffset.toFixed(1)} ms.`);

    } catch (error) {
        console.warn("Échec de la synchronisation. Utilisation de l'horloge locale.", error);
        lServH = Date.now(); 
        lLocH = performance.now();
        if ($('local-time')) $('local-time').textContent = 'N/A (SYNCHRO ÉCHOUÉE)';
    }
}

/** Retourne l'heure synchronisée (précision RTT compensée en UTC). */
function getCDate() { 
    if (lServH === null || lLocH === null) { return null; }
    const offsetSinceSync = performance.now() - lLocH;
    return new Date(lServH + offsetSinceSync); 
}

// ===========================================
// FILTRE DE KALMAN (Vitesse & Altitude)
// ===========================================

/** Applique le filtre de Kalman à la vitesse 3D. */
function kFilter(nSpd, dt, R_dyn) {
    if (dt === 0 || dt > 5) return kSpd; 
    const R = R_dyn ?? R_MAX, Q = Q_NOISE * dt; 
    let pSpd = kSpd, pUnc = kUncert + Q; 
    let K = pUnc / (pUnc + R); 
    kSpd = pSpd + K * (nSpd - pSpd); 
    kUncert = (1 - K) * pUnc; 
    return kSpd;
}

/** Applique le filtre de Kalman à l'Altitude. */
function kFilterAltitude(nAlt, acc, dt) {
    if (nAlt === null) return kAlt;
    
    if (kAlt === null) {
        kAlt = nAlt;
        return kAlt;
    }

    const R = Math.max(R_ALT_MIN, acc); 
    const Q = Q_ALT_NOISE * dt; 
    
    // Prédiction (Approximation pour la structure simplifiée du Kalman)
    let pAlt = kAlt; 
    const spdV_pred = (kAlt - (lPos?.kAlt_old ?? kAlt)) / (lPos ? (lPos.timestamp - lPos.oldTimestamp) / 1000 : dt);
    pAlt = kAlt + spdV_pred * dt;
    let pUnc = kAltUncert + Q; 
    
    // Correction
    const K = pUnc / (pUnc + R); 
    kAlt = pAlt + K * (nAlt - pAlt); 
    kAltUncert = (1 - K) * pUnc; 
    
    return kAlt;
}

/** Calcule le Facteur R (Précision 3D Dynamique) du filtre de Kalman. */
function getKalmanR(acc, alt, P_hPa) {
    let R = acc ** 2; 
    R = Math.max(R_MIN, Math.min(R_MAX, R));
    
    const envFactor = ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT;
    
    if (P_hPa !== null) {
        const pressureFactor = 1.0 + (1013.25 - P_hPa) / 1013.25 * 0.1;
        R *= Math.max(1.0, pressureFactor);
    }
    
    R *= envFactor;
    if (alt < ALT_TH) { R *= 2.0; } 

    R = Math.max(R_MIN, Math.min(R_MAX, R));
    
    return R;
}
// =================================================================
// FICHIER JS PARTIE 2/2 : gnss-dashboard-part2.js
// Contient la logique principale de mise à jour GPS, Astro, Météo et DOM.
// NÉCESSITE gnss-dashboard-part1.js
// =================================================================

// ===========================================
// FONCTIONS ASTRO & TEMPS
// ===========================================

/** Convertit la date en jours depuis J2000. */
function toDays(date) { return (date.valueOf() / dayMs - 0.5 + J1970) - J2000; }

/** Calcule l'anomalie solaire moyenne. */
function solarMeanAnomaly(d) { return D2R * (356.0470 + 0.9856002585 * d); }

/** Calcule la longitude écliptique. */
function eclipticLongitude(M) {
    var C = D2R * (1.9148 * Math.sin(M) + 0.0200 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M)), 
        P = D2R * 102.9377;                                                                
    return M + C + P + Math.PI;
}

/** Format les ms en HH:MM:SS. */
function msToTime(ms) {
    if (isNaN(ms)) return 'N/A';
    ms = (ms + dayMs) % dayMs; // Assure une valeur positive dans la journée
    let h = Math.floor(ms / 3600000);
    let m = Math.floor((ms % 3600000) / 60000);
    let s = Math.floor((ms % 60000) / 1000);
    return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
}

/** Calcule le Temps Solaire Vrai (TST) et l'Équation du Temps (EOT). */
function getSolarTime(date, lon) {
    if (date === null || lon === null) return { TST: 'N/A', MST: 'N/A', EOT: 'N/D', ECL_LONG: 'N/D' };
    const d = toDays(date);
    const M = solarMeanAnomaly(d); 
    const L = eclipticLongitude(M); 
    const epsilon = D2R * (23.4393 - 0.000000356 * d); 
    let alpha = Math.atan2(Math.cos(epsilon) * Math.sin(L), Math.cos(L));
    if (alpha < 0) alpha += 2 * Math.PI; 
    const eot_rad = alpha - M - D2R * 102.9377 - Math.PI;
    const eot_min = eot_rad * 4 * R2D; 
    
    const msSinceMidnightUTC = (date.getUTCHours() * 3600 + date.getUTCMinutes() * 60 + date.getUTCSeconds()) * 1000 + date.getUTCMilliseconds();
    const mst_offset_ms = lon * dayMs / 360; 
    const mst_ms = (msSinceMidnightUTC + mst_offset_ms + dayMs) % dayMs;
    const eot_ms = eot_min * 60000;
    const tst_ms = (mst_ms + eot_ms + dayMs) % dayMs; 
    
    return { 
        TST: msToTime(tst_ms), 
        MST: msToTime(mst_ms), 
        EOT: eot_min.toFixed(3),
        ECL_LONG: (L * R2D).toFixed(2),
        EOT_MS: eot_ms // Exporté pour conversion d'heure
    };
}

/** Convertit une heure UTC (Date object) en Heure Solaire Vraie (TST). */
function toSolarTime(utcDate, lon, eotMs) {
    if (!utcDate || !lon || eotMs === undefined) return 'N/A';
    
    const msSinceMidnightUTC = utcDate.getUTCHours() * 3600000 + utcDate.getUTCMinutes() * 60000 + utcDate.getUTCSeconds() * 1000;
    const mst_offset_ms = lon * dayMs / 360; 
    const tst_ms = (msSinceMidnightUTC + mst_offset_ms + eotMs + dayMs) % dayMs;

    return msToTime(tst_ms);
}

/** Calcule le temps Minecraft. */
function getMinecraftTime(date) {
    if (date === null) return '00:00:00';
    const msSinceMidnightUTC = date.getUTCHours() * 3600000 + date.getUTCMilliseconds() + date.getUTCMinutes() * 60000 + date.getUTCSeconds() * 1000;
    const timeRatio = (msSinceMidnightUTC % dayMs) / dayMs;
    const mcTimeMs = (timeRatio * MC_DAY_MS + MC_DAY_MS) % MC_DAY_MS;
    return msToTime(mcTimeMs);
}

function updateAstro(latA, lonA) {
    const now = getCDate(); 
    if (now === null) {
        if ($('local-time') && !$('local-time').textContent.includes('Synchronisation')) {
             $('local-time').textContent = 'Synchronisation...';
        }
        return;
    }
    
    const sunPos = window.SunCalc ? SunCalc.getPosition(now, latA, lonA) : null;
    const moonIllum = window.SunCalc ? SunCalc.getMoonIllumination(now) : null;
    const sunTimes = window.SunCalc ? SunCalc.getTimes(now, latA, lonA) : null;
    const moonTimes = window.SunCalc ? SunCalc.getMoonTimes(now, latA, lonA, true) : null;
    const solarTimes = getSolarTime(now, lonA);

    // Mise à jour de l'heure
    $('local-time').textContent = now.toLocaleTimeString('fr-FR', { timeZone: 'UTC', hour12: false });
    if ($('date-display')) $('date-display').textContent = now.toLocaleDateString();
    
    // Mise à jour Astro (TST/LSM)
    if ($('time-solar-true')) $('time-solar-true').textContent = solarTimes.TST;
    if ($('culmination-lsm')) $('culmination-lsm').textContent = solarTimes.MST;
    if ($('eot-min')) $('eot-min').textContent = solarTimes.EOT + ' min'; 
    if ($('ecliptic-long')) $('ecliptic-long').textContent = solarTimes.ECL_LONG + ' °';

    if (sunPos) {
        if ($('sun-elevation')) $('sun-elevation').textContent = `${(sunPos.altitude * R2D).toFixed(2)} °`;
        if ($('sun-azimuth')) $('sun-azimuth').textContent = `${(sunPos.azimuth * R2D).toFixed(2)} °`;
    } else {
        if ($('sun-elevation')) $('sun-elevation').textContent = 'N/A';
        if ($('sun-azimuth')) $('sun-azimuth').textContent = 'N/A';
    }
    
    // --- LEVER/COUCHER EN HEURE SOLAIRE VRAIE (TST) ---
    let sunRiseTST = 'N/A', sunSetTST = 'N/A';
    if (sunTimes && sunTimes.sunrise && sunTimes.sunset) {
        sunRiseTST = toSolarTime(sunTimes.sunrise, lonA, solarTimes.EOT_MS);
        sunSetTST = toSolarTime(sunTimes.sunset, lonA, solarTimes.EOT_MS);
    }
    if ($('sun-times-tst')) $('sun-times-tst').textContent = `↑ ${sunRiseTST} / ↓ ${sunSetTST}`;

    if ($('day-duration') && sunTimes && sunTimes.sunrise && sunTimes.sunset) {
        const durationMs = sunTimes.sunset.getTime() - sunTimes.sunrise.getTime();
        const hours = Math.floor(durationMs / 3600000);
        const minutes = Math.floor((durationMs % 3600000) / 60000);
        $('day-duration').textContent = `${hours}h ${minutes}m`;
    } else if ($('day-duration')) {
        $('day-duration').textContent = 'N/A (Polaire/Nuit)';
    }

    // --- CULMINATION LUNAIRE (TST) ---
    let moonRiseCulm = 'N/A', moonSetCulm = 'N/A';
    if (moonTimes) {
        if (moonTimes.rise) moonRiseCulm = toSolarTime(moonTimes.rise, lonA, solarTimes.EOT_MS);
        if (moonTimes.set) moonSetCulm = toSolarTime(moonTimes.set, lonA, solarTimes.EOT_MS);
    }

    if ($('moon-culmination-tst')) $('moon-culmination-tst').textContent = `↑ ${moonRiseCulm} / ↓ ${moonSetCulm}`;
    
    // Mise à jour Lune
    if ($('lunar-phase-perc')) $('lunar-phase-perc').textContent = moonIllum ? `${(moonIllum.fraction * 100).toFixed(1)} %` : 'N/A';
    if (moonTimes && moonTimes.alwaysUp === false && moonTimes.alwaysDown === false) {
        if ($('moon-elevation')) $('moon-elevation').textContent = moonTimes.rise ? `${(SunCalc.getMoonPosition(now, latA, lonA).altitude * R2D).toFixed(2)} °` : 'N/A';
        if ($('moon-azimuth')) $('moon-azimuth').textContent = moonTimes.rise ? `${(SunCalc.getMoonPosition(now, latA, lonA).azimuth * R2D).toFixed(2)} °` : 'N/A';
    } else {
        if ($('moon-elevation')) $('moon-elevation').textContent = moonTimes?.alwaysUp ? 'UP 24/7' : 'DOWN 24/7';
        if ($('moon-azimuth')) $('moon-azimuth').textContent = 'N/A';
    }
    
    // Mise à jour du temps (suite)
    if (sTime) {
        const timeElapsed = (now.getTime() - sTime) / 1000;
        $('time-elapsed').textContent = `${timeElapsed.toFixed(2)} s`;
        $('time-moving').textContent = `${timeMoving.toFixed(2)} s`;
        if ($('time-minecraft')) $('time-minecraft').textContent = getMinecraftTime(now);
    }
}

// ===========================================
// FONCTIONS DE CONTRÔLE GPS & MÉTÉO
// ===========================================

// ... (setGPSMode, startGPS, stopGPS, emergencyStop, resumeSystem, handleErr restent inchangés)

// Fonction météo (avec proxy Vercel potentiel)
async function fetchWeather(latA, lonA) {
    // Si latA/lonA sont nuls ou si on est en mode 'localhost' (proxy par défaut), on ignore l'API
    if (!latA || !lonA || PROXY_BASE_URL.includes('scientific-dashboard2')) {
        if ($('env-factor')) $('env-factor').textContent = `${selectedEnvironment} (x${ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT})`;
        return; 
    }

    try {
        const response = await fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${latA}&lon=${lonA}`);
        const data = await response.json();
        
        if (data.main) {
            lastP_hPa = data.main.pressure; 
            lastT_K = data.main.temp + 273.15; 
            lastH_perc = data.main.humidity / 100; 
        }

        if ($('temp-air')) $('temp-air').textContent = lastT_K ? `${(lastT_K - 273.15).toFixed(1)} °C` : 'N/A';
        if ($('pressure')) $('pressure').textContent = lastP_hPa ? `${lastP_hPa.toFixed(1)} hPa` : 'N/A';
        if ($('humidity')) $('humidity').textContent = lastH_perc ? `${(lastH_perc * 100).toFixed(0)} %` : 'N/A';
        if ($('env-factor')) $('env-factor').textContent = `${selectedEnvironment} (x${ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT})`;

    } catch (error) {
        console.error("Échec de la récupération des données météo:", error);
    }
}

// ... (handleCapture, handleMaterials restent inchangés)


// ===========================================
// FONCTIONS DE CONTRÔLE IMU (DeviceMotion)
// ===========================================

function updateIMU(event) {
    if (emergencyStopActive) return;

    // Utilisation de accelerationIncludingGravity car acceleration seule peut être nulle
    const accX = event.accelerationIncludingGravity.x ?? 0;
    const accY = event.accelerationIncludingGravity.y ?? 0;
    
    // Tente d'estimer l'accélération longitudinale (selon l'orientation de l'appareil)
    // Pour un smartphone tenu à plat : Y est latéral, X est longitudinal.
    // L'accélération latérale (Y) est utilisée pour la force centrifuge/latérale ici.

    // Lit la masse actuelle du DOM ou utilise la valeur par défaut si l'ID DOM n'existe pas
    const currentMass = parseFloat($('mass-input')?.value) || vehicleMass;
    
    // Mise à jour de la variable globale pour la persistance si le champ DOM n'est pas utilisé
    vehicleMass = currentMass;

    // Force Latérale (Centrifuge) et Longitudinale (Freinage/Accélération)
    const lateralForce = currentMass * accY;
    const longitudinalAccel = accX; // Accélération X est utilisée pour l'affichage longitudinal
    
    // Mise à jour des variables pour la sauvegarde et l'affichage (Utilisation de l'accélération corrigée)
    lastAccelX = longitudinalAccel;
    lastAccelY = accY;

    // Mise à jour du DOM pour l'IMU (Doit être fait ici pour la réactivité)
    if ($('accel-long-imu')) $('accel-long-imu').textContent = `${longitudinalAccel.toFixed(3)} m/s²`;
    if ($('accel-lateral-imu')) $('accel-lateral-imu').textContent = `${accY.toFixed(3)} m/s²`;
    if ($('force-centrifugal-n')) $('force-centrifugal-n').textContent = `${lateralForce.toFixed(2)} N`;
    if ($('force-g-lateral')) $('force-g-lateral').textContent = `${(accY / G_ACC).toFixed(2)} G`;
    if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
}

function startIMU() {
    if (!window.DeviceMotionEvent) {
        console.warn("DeviceMotionEvent non supporté par ce navigateur.");
        if ($('accel-long-imu')) $('accel-long-imu').textContent = 'N/A (Non Supporté)';
        return;
    }
    
    if (imuID === null) {
        if (typeof DeviceMotionEvent.requestPermission === 'function') {
            DeviceMotionEvent.requestPermission().then(permissionState => {
                if (permissionState === 'granted') {
                    imuID = window.addEventListener('devicemotion', updateIMU);
                    console.log("DeviceMotion démarré.");
                } else {
                    console.warn("DeviceMotion autorisation refusée.");
                }
            }).catch(console.error);
        } else {
            imuID = window.addEventListener('devicemotion', updateIMU);
            console.log("DeviceMotion démarré (sans permission explicite).");
        }
    }
}

function stopIMU() {
    if (imuID !== null) {
        window.removeEventListener('devicemotion', updateIMU);
        imuID = null;
        console.log("DeviceMotion arrêté.");
    }
}


// ===========================================
// FONCTION PRINCIPALE DE MISE À JOUR GPS 
// ===========================================

function updateDisp(pos) {
    if (emergencyStopActive) return;

    // ... (Logique GPS et Kalman inchangée, utilise les mêmes variables globales)

    // ... (MISE À JOUR DU DOM GPS/Physique)

    // Mise à jour de la météo (toutes les 60 secondes)
    if (Date.now() - (updateDisp.lastWeatherFetch ?? 0) > 60000) {
        fetchWeather(lat, lon); 
        updateDisp.lastWeatherFetch = Date.now();
    }
    
    // SAUVEGARDE DES VALEURS POUR LA PROCHAINE ITÉRATION
    lPos = pos; 
    lPos.speedMS_3D = spd3D; 
    lPos.timestamp = cTimePos; 
    lPos.kAlt_old = kAlt_new; 
}


// ===========================================
// INITIALISATION DES ÉVÉNEMENTS DOM
// ===========================================

document.addEventListener('DOMContentLoaded', () => {
    
    // ... (Initialisation du sélecteur d'environnement)
    
    syncH(); 

    // Événements de contrôle
    // ... (Logique de contrôle GPS, arrêt d'urgence, nether, reset inchangée)

    // Événement SÉLECTEUR ENVIRONNEMENT (Matériaux)
    if ($('env-select')) $('env-select').addEventListener('change', (e) => { 
        if (emergencyStopActive) return;
        selectedEnvironment = e.target.value; 
        if ($('env-factor')) $('env-factor').textContent = `${selectedEnvironment} (x${ENVIRONMENT_FACTORS[selectedEnvironment].R_MULT})`; 
    });
    // Événement Matériaux (pour le bouton)
    if ($('materials-btn')) $('materials-btn').addEventListener('click', handleMaterials);

    // Initialisation du champ de masse
    if ($('mass-input')) {
        $('mass-input').value = vehicleMass.toFixed(3);
        $('mass-input').addEventListener('change', () => {
            const newMass = parseFloat($('mass-input').value);
            if (!isNaN(newMass) && newMass > 0) {
                vehicleMass = newMass;
                if ($('mass-display')) $('mass-display').textContent = `${vehicleMass.toFixed(3)} kg`;
            } else {
                $('mass-input').value = vehicleMass.toFixed(3);
            }
        });
    }


    startGPS(); 
    startIMU(); // Démarrage des capteurs IMU

    // Mise à jour périodique des données DOM/Astro (1x/seconde)
    if (domID === null) {
        domID = setInterval(() => {
            const currentLat = lPos ? lPos.coords.latitude : 48.8566; 
            const currentLon = lPos ? lPos.coords.longitude : 2.3522;
            updateAstro(currentLat, currentLon); 
        }, DOM_SLOW_UPDATE_MS); 
    }
    
    // La fonction initMap() doit être définie dans le HTML ou une librairie JS externe (Leaflet)
    if (typeof initMap !== 'undefined') initMap();
});
