// =================================================================
// FICHIER JS PARTIE 1/2 : gnss-dashboard-part1.js
// Contient constantes, variables d'état (avec window. pour le scope), NTP, et les filtres de Kalman.
// =================================================================

// --- CLÉS D'API & PROXY VERCEL ---
const PROXY_BASE_URL = "https://scientific-dashboard2.vercel.app"; 
const PROXY_WEATHER_ENDPOINT = `${PROXY_BASE_URL}/api/weather`;
const SERVER_TIME_ENDPOINT = "https://worldtimeapi.org/api/utc"; 

// --- CONSTANTES GLOBALES ET INITIALISATION ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458; 
const R_E = 6371000;   
const KMH_MS = 3.6;    
const C_S = 343;       
const G_ACC = 9.80665; 
const MC_DAY_MS = 72 * 60 * 1000; 
const VEHICLE_MASS_KG_DEFAULT = 70.0; 

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
const MIN_SPD = 0.05, ALT_TH = -50; 
const NETHER_RATIO = 8; 
const Q_ALT_NOISE = 0.1; 
const R_ALT_MIN = 0.1;  
const DOM_SLOW_UPDATE_MS = 1000; 

// FACTEURS ENVIRONNEMENTAUX POUR LA CORRECTION KALMAN (Matériaux)
const ENVIRONMENT_FACTORS = {
    'NORMAL': { R_MULT: 1.0 },
    'METAL': { R_MULT: 2.5 },      
    'FOREST': { R_MULT: 1.5 },     
    'CONCRETE': { R_MULT: 3.0 },   
};

// --- VARIABLES D'ÉTAT (TOUTES ATTACHÉES À WINDOW POUR LE SCOPE GLOBAL) ---
window.wID = null;
window.domID = null;
window.lPos = null;
window.lat = null;
window.lon = null;
window.sTime = null;

window.distM = 0;
window.distMStartOffset = 0;
window.maxSpd = 0;
window.kSpd = 0;
window.kUncert = 1000; 
window.timeMoving = 0; 
window.lServH = null;
window.lLocH = null;
window.lastFSpeed = 0;
window.kAlt = null;     
window.kAltUncert = 10; 

window.currentGPSMode = 'HIGH_FREQ'; 
window.emergencyStopActive = false;
window.netherMode = false;
window.selectedEnvironment = 'NORMAL'; 

window.lastP_hPa = null;
window.lastT_K = null;
window.lastH_perc = null; 

// VARIABLES IMU
window.imuID = null; 
window.lastAccelX = 0;
window.lastAccelY = 0;
window.vehicleMass = VEHICLE_MASS_KG_DEFAULT; 

// --- REFERENCES DOM (Rendue globale par sécurité) ---
window.$ = id => document.getElementById(id);

// ===========================================
// FONCTIONS GÉO & SYNCHRONISATION
// ===========================================

/** Calcule la distance horizontale (Haversine). */
function dist(lat1, lon1, lat2, lon2) {
    const R = R_E, dLat = (lat2 - lat1) * D2R, dLon = (lon2 - lon1) * D2R;
    lat1 *= D2R; lat2 *= D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
}

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

        window.lServH = serverTimestamp + latencyOffset; 
        window.lLocH = performance.now(); 
        console.log(`Synchronisation UTC Atomique réussie. Latence corrigée: ${latencyOffset.toFixed(1)} ms.`);

    } catch (error) {
        console.warn("Échec de la synchronisation. Utilisation de l'horloge locale.", error);
        window.lServH = Date.now(); 
        window.lLocH = performance.now();
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

function kFilter(nSpd, dt, R_dyn) {
    if (dt === 0 || dt > 5) return kSpd; 
    const R = R_dyn ?? R_MAX, Q = Q_NOISE * dt; 
    let pSpd = kSpd, pUnc = kUncert + Q; 
    let K = pUnc / (pUnc + R); 
    window.kSpd = pSpd + K * (nSpd - pSpd); 
    window.kUncert = (1 - K) * pUnc; 
    return window.kSpd;
}

function kFilterAltitude(nAlt, acc, dt) {
    if (nAlt === null) return kAlt;
    
    if (kAlt === null) {
        window.kAlt = nAlt;
        return kAlt;
    }

    const R = Math.max(R_ALT_MIN, acc); 
    const Q = Q_ALT_NOISE * dt; 
    
    let pAlt = kAlt; 
    const spdV_pred = (kAlt - (lPos?.kAlt_old ?? kAlt)) / (lPos ? (lPos.timestamp - lPos.oldTimestamp) / 1000 : dt);
    pAlt = kAlt + spdV_pred * dt;
    let pUnc = kAltUncert + Q; 
    
    const K = pUnc / (pUnc + R); 
    window.kAlt = pAlt + K * (nAlt - pAlt); 
    window.kAltUncert = (1 - K) * pUnc; 
    
    return window.kAlt;
}

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
// NÉCESSITE gnss-dashboard-part1.js POUR LES VARIABLES ET FONCTIONS DE BASE.
// =================================================================

// --- FONCTIONS ASTRO & TEMPS (NÉCESSITENT SUN CALC) ---

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
    ms = (ms + dayMs) % dayMs; 
    let h = Math.floor(ms / 3600000);
    let m = Math.floor((ms % 3600000) / 60000);
    let s = Math.floor((ms % 60000) / 1000);
    return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
}

/** Calcule le Temps Solaire Vrai (TST) et l'Équation du Temps (EOT). */
function getSolarTime(date, lon) {
    if (date === null || lon === null) return { TST: 'N/A', MST: 'N/A', EOT: 'N/D', ECL_LONG: 'N/D', EOT_MS: 0 };
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
        EOT_MS: eot_ms 
    };
}

/** Convertit une heure UTC (Date object) en Heure Solaire Vraie (TST). */
function toSolarTime(utcDate, lon, eotMs) {
    if (!utcDate || lon === null || eotMs === undefined) return 'N/A';
    
    const msSinceMidnightUTC = utcDate.getUTCHours() * 3600000 + utcDate.getUTCMilliseconds() + utcDate.getUTCMinutes() * 60000 + utcDate.getUTCSeconds() * 1000;
    const mst_offset_ms = lon * dayMs / 360; 
    const tst_ms = (msSinceMidnightUTC + mst_offset_ms + eotMs + dayMs) % dayMs;

    return msToTime(tst_ms);
}

function updateAstro(latA, lonA) {
    const now = getCDate(); 
    if (now === null) return;
    
    const sunPos = window.SunCalc ? SunCalc.getPosition(now, latA, lonA) : null;
    const moonIllum = window.SunCalc ? SunCalc.getMoonIllumination(now) : null;
    const sunTimes = window.SunCalc ? SunCalc.getTimes(now, latA, lonA) : null;
    const moonTimes = window.SunCalc ? SunCalc.getMoonTimes(now, latA, lonA, true) : null;
    const solarTimes = getSolarTime(now, lonA);

    // Mise à jour de l'heure
    $('local-time').textContent = now.toLocaleTimeString('fr-FR', { timeZone: 'UTC', hour12: false });
    if ($('date-display')) $('date-display').textContent = now.toLocaleDateString();
    
    // ... (Mise à jour DOM Astro TST/LSM)

    if (sunPos) {
        if ($('sun-elevation')) $('sun-elevation').textContent = `${(sunPos.altitude * R2D).toFixed(2)} °`;
        if ($('sun-azimuth')) $('sun-azimuth').textContent = `${(sunPos.azimuth * R2D).toFixed(2)} °`;
    } 
    
    let sunRiseTST = 'N/A', sunSetTST = 'N/A';
    if (sunTimes && sunTimes.sunrise && sunTimes.sunset) {
        sunRiseTST = toSolarTime(sunTimes.sunrise, lonA, solarTimes.EOT_MS);
        sunSetTST = toSolarTime(sunTimes.sunset, lonA, solarTimes.EOT_MS);
    }
    if ($('sun-times-tst')) $('sun-times-tst').textContent = `↑ ${sunRiseTST} / ↓ ${sunSetTST}`;

    let moonRiseCulm = 'N/A', moonSetCulm = 'N/A';
    if (moonTimes) {
        if (moonTimes.rise) moonRiseCulm = toSolarTime(moonTimes.rise, lonA, solarTimes.EOT_MS);
        if (moonTimes.set) moonSetCulm = toSolarTime(moonTimes.set, lonA, solarTimes.EOT_MS);
    }
    if ($('moon-culmination-tst')) $('moon-culmination-tst').textContent = `↑ ${moonRiseCulm} / ↓ ${moonSetCulm}`;
    
    // ... (Mise à jour Lune)
}


// ===========================================
// FONCTIONS DE CONTRÔLE GPS & MÉTÉO
// ===========================================

function setGPSMode(mode) {
    window.currentGPSMode = mode;
    if (wID !== null) {
        stopGPS();
        startGPS();
    }
}

function startGPS() {
    if (wID !== null) return;
    
    const options = GPS_OPTS[currentGPSMode];
    window.wID = navigator.geolocation.watchPosition(updateDisp, handleErr, options);
    
    $('toggle-gps-btn').textContent = '⏸️ PAUSE GPS';
    $('toggle-gps-btn').style.backgroundColor = '#FFC107';
    
    if (sTime === null) window.sTime = getCDate()?.getTime() ?? Date.now();
}

function stopGPS() {
    if (wID === null) return;
    navigator.geolocation.clearWatch(wID);
    window.wID = null;
    
    $('toggle-gps-btn').textContent = '▶️ MARCHE GPS';
    $('toggle-gps-btn').style.backgroundColor = '#4CAF50';
}

function emergencyStop() {
    window.emergencyStopActive = true;
    stopGPS();
    stopIMU();
    
    $('emergency-stop-btn').textContent = '🛑 ARRÊT D’URGENCE : ACTIF 🔴';
    $('emergency-stop-btn').classList.add('active');
}

function resumeSystem() {
    window.emergencyStopActive = false;
    startGPS();
    startIMU();
    
    $('emergency-stop-btn').textContent = '🛑 Arrêt d\'urgence: INACTIF 🟢';
    $('emergency-stop-btn').classList.remove('active');
}

function handleErr(err) {
    console.error(`Erreur GPS (${err.code}): ${err.message}`);
    if (err.code === 3 && currentGPSMode === 'HIGH_FREQ') {
        stopGPS();
        setTimeout(startGPS, 1000); 
    }
    if ($('gps-precision')) $('gps-precision').textContent = `Erreur: ${err.code}`;
}

async function fetchWeather(latA, lonA) {
    if (!latA || !lonA) return; 

    try {
        const response = await fetch(`${PROXY_WEATHER_ENDPOINT}?lat=${latA}&lon=${lonA}`);
        const data = await response.json();
        
        if (data.main) {
            window.lastP_hPa = data.main.pressure; 
            window.lastT_K = data.main.temp + 273.15; 
            window.lastH_perc = data.main.humidity / 100; 
        }

        if ($('temp-air')) $('temp-air').textContent = lastT_K ? `${(lastT_K - 273.15).toFixed(1)} °C` : 'N/A';
        if ($('pressure')) $('pressure').textContent = lastP_hPa ? `${lastP_hPa.toFixed(1)} hPa` : 'N/A';
        if ($('humidity')) $('humidity').textContent = lastH_perc ? `${(lastH_perc * 100).toFixed(0)} %` : 'N/A';
        
    } catch (error) {
        console.error("Échec de la récupération des données météo:", error);
    }
}

function handleMaterials() {
    // ... (Logique de gestion des matériaux inchangée)
}

// --- FONCTIONS IMU (appelées ici pour utiliser les fonctions de part1.js) ---

function updateIMU(event) {
    if (emergencyStopActive) return;

    const accX = event.accelerationIncludingGravity.x ?? 0;
    const accY = event.accelerationIncludingGravity.y ?? 0;
    
    const currentMass = parseFloat($('mass-input')?.value) || vehicleMass;
    window.vehicleMass = currentMass;

    const lateralForce = currentMass * accY;
    const longitudinalAccel = accX; 
    
    window.lastAccelX = longitudinalAccel;
    window.lastAccelY = accY;

    if ($('accel-long-imu')) $('accel-long-imu').textContent = `${longitudinalAccel.toFixed(3)} m/s²`;
    if ($('accel-lateral-imu')) $('accel-lateral-imu').textContent = `${accY.toFixed(3)} m/s²`;
    if ($('force-centrifugal-n')) $('force-centrifugal-n').textContent = `${lateralForce.toFixed(2)} N`;
    if ($('force-g-lateral')) $('force-g-lateral').textContent = `${(accY / G_ACC).toFixed(2)} G`;
    if ($('mass-display')) $('mass-display').textContent = `${currentMass.toFixed(3)} kg`;
}

function startIMU() {
    if (!window.DeviceMotionEvent) {
        console.warn("DeviceMotionEvent non supporté par ce navigateur.");
        return;
    }
    
    if (imuID === null) {
        if (typeof DeviceMotionEvent.requestPermission === 'function') {
            DeviceMotionEvent.requestPermission().then(permissionState => {
                if (permissionState === 'granted') {
                    window.imuID = window.addEventListener('devicemotion', updateIMU);
                }
            }).catch(console.error);
        } else {
            window.imuID = window.addEventListener('devicemotion', updateIMU);
        }
    }
}

function stopIMU() {
    if (imuID !== null) {
        window.removeEventListener('devicemotion', updateIMU);
        window.imuID = null;
    }
}

// ===========================================
// FONCTION PRINCIPALE DE MISE À JOUR GPS 
// ===========================================

function updateDisp(pos) {
    if (emergencyStopActive) return;

    const cTimePos = pos.timestamp;
    const acc = pos.coords.accuracy ?? R_MAX;
    const altRaw = pos.coords.altitude;
    const spdRaw = pos.coords.speed ?? 0;
    const spdVRaw = pos.coords.altitudeSpeed ?? 0;

    const dt = lPos ? (cTimePos - lPos.timestamp) / 1000 : MIN_DT;
    if (dt < MIN_DT) return; 

    const lat = pos.coords.latitude;
    const lon = pos.coords.longitude;
    const dH = lPos ? dist(lPos.coords.latitude, lPos.coords.longitude, lat, lon) : 0;
    const dV = lPos ? (altRaw - lPos.coords.altitude) : 0;
    const dist3D = Math.sqrt(dH ** 2 + dV ** 2);
    const spd3D = dist3D / dt;

    window.distM += dH;
    if (netherMode) window.distMStartOffset += dH * (NETHER_RATIO - 1);

    if (spd3D > MIN_SPD) window.timeMoving += dt;
    
    const R_dyn = getKalmanR(acc, altRaw, lastP_hPa);
    const spdFiltered = kFilter(spd3D, dt, R_dyn);
    const kAlt_new = kFilterAltitude(altRaw, acc, dt);
    
    const accelEKF = (spdFiltered - lastFSpeed) / dt; 
    window.lastFSpeed = spdFiltered; 

    window.maxSpd = Math.max(maxSpd, spdFiltered);

    // ... (MISE À JOUR DU DOM VITESSE, DISTANCE, GNSS BRUT, PHYSIQUE)
    
    if ($('speed-stable')) $('speed-stable').textContent = `${(spdFiltered * KMH_MS).toFixed(4)} km/h`;
    if ($('accel-long')) $('accel-long').textContent = `${accelEKF.toFixed(3)} m/s²`;
    if ($('force-g-long')) $('force-g-long').textContent = `${(accelEKF / G_ACC).toFixed(2)} G`;
    if ($('latitude')) $('latitude').textContent = lat.toFixed(6);
    if ($('longitude')) $('longitude').textContent = lon.toFixed(6);
    // ... (autres mises à jour DOM)

    // Appel de la fonction de mise à jour de la carte (doit être définie dans le HTML)
    if (typeof window.updateMap === 'function') {
        window.updateMap(lat, lon);
    }
    
    // Mise à jour de la météo (toutes les 60 secondes)
    if (Date.now() - (updateDisp.lastWeatherFetch ?? 0) > 60000) {
        fetchWeather(lat, lon); 
        updateDisp.lastWeatherFetch = Date.now();
    }
    
    // SAUVEGARDE DES VALEURS POUR LA PROCHAINE ITÉRATION
    window.lPos = pos; 
    window.lPos.speedMS_3D = spd3D; 
    window.lPos.timestamp = cTimePos; 
    window.lPos.kAlt_old = kAlt_new; 
}


// ===========================================
// INITIALISATION DES ÉVÉNEMENTS DOM
// ===========================================

document.addEventListener('DOMContentLoaded', () => {
    
    syncH(); 

    // Événements de contrôle principaux
    if ($('toggle-gps-btn')) $('toggle-gps-btn').addEventListener('click', () => {
        if (wID === null) { startGPS(); } else { stopGPS(); }
    });
    if ($('freq-select')) $('freq-select').addEventListener('change', (e) => setGPSMode(e.target.value));
    if ($('emergency-stop-btn')) $('emergency-stop-btn').addEventListener('click', () => {
        emergencyStopActive ? resumeSystem() : emergencyStop();
    });
    // ... (autres écouteurs de boutons)

    // Initialisation du champ de masse
    const massInput = $('mass-input');
    if (massInput) {
        massInput.value = vehicleMass.toFixed(3);
        massInput.addEventListener('change', () => {
            const newMass = parseFloat(massInput.value);
            if (!isNaN(newMass) && newMass > 0) {
                window.vehicleMass = newMass;
                if ($('mass-display')) $('mass-display').textContent = `${vehicleMass.toFixed(3)} kg`;
            } else {
                massInput.value = vehicleMass.toFixed(3);
            }
        });
    }

    startGPS();   
    startIMU();   

    // Mise à jour périodique des données DOM/Astro (1x/seconde)
    if (domID === null) {
        window.domID = setInterval(() => {
            const currentLat = lPos ? lPos.coords.latitude : 43.296482; 
            const currentLon = lPos ? lPos.coords.longitude : 5.36978;
            updateAstro(currentLat, currentLon); 
        }, DOM_SLOW_UPDATE_MS); 
    }
    
    // Initialisation de la carte
    if (typeof initMap !== 'undefined') initMap();
});
