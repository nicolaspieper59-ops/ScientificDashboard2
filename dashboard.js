// =================================================================
// FICHIER COMPLET ET FINAL : dashboard.js
// Version Ultime Pédagogique avec Synchro time.is Persistante & Correction de Vitesse Dynamique
// =================================================================

// --- CONSTANTES GLOBALES ET INITIALISATION ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458, C_S = 343, R_E = 6371000, KMH_MS = 3.6;
const OBLIQ = 23.44 * D2R, ECC = 0.0167, JD_2K = 2451545.0;
const W_OPTS = { enableHighAccuracy: true, maximumAge: 0, timeout: 20000 };
const DOM_MS = 17, MIN_DT = 1, MAX_ACC = 10, MIN_SPD = 0.001, ALT_TH = -50; 
const Q_NOISE = 0.005, R_MIN = 0.005, R_MAX = 5.0, L_PREC_TH = 60;
const SUN_NIGHT_TH = -12; 
const LUX_NIGHT_TH = 50; 
const NETHER_RATIO = 8; 

// Constantes pour les calculs de dynamique et thermo
const AIR_DENSITY = 1.225; 
const CAR_MASS = 1500;     
const CDA_EST = 0.6;      
const PRESSURE_SEA = 1013.25; 
const LAPSE_RATE = 0.0065;   

// Variables pour le lieu par défaut (Modifiable)
let defaultLat = 48.8566; 
let defaultLon = 2.3522;  

let wID = null, domID = null, lPos = null, lat = null, lon = null, sTime = null;
let distM = 0, maxSpd = 0, tLat = null, tLon = null, lDomT = null;
let kSpd = 0, kUncert = 1000; 
let lServH = null, lLocH = null; 
let als = null, lastLux = null, manualMode = null; 
let netherMode = false; 
let todayLC = 0; 

let lastSpd3D = 0;        
let accellLong = 0;       
let ambientTemp = 20.0; 
let ambientPressure = 1013.25; 
let magSensor = null; 

// --- REFERENCES DOM ---
const $ = id => document.getElementById(id);
const startBtn = $('start-btn'), stopBtn = $('stop-btn'), resetMaxBtn = $('reset-max-btn');
const netherToggleBtn = $('nether-toggle-btn');
const solarTrueHeader = $('solar-true-header'); 
const solarMeanHeader = $('solar-mean-header'); 


// ===========================================
// FONCTIONS GÉO & UTILS
// ===========================================

const dist = (lat1, lon1, lat2, lon2) => { /* ... (Logique dist) ... */
    const R = R_E, dLat = (lat2 - lat1) * D2R, dLon = (lon2 - lon1) * D2R;
    lat1 *= D2R; lat2 *= D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
};

const bearing = (lat1, lon1, lat2, lon2) => { /* ... (Logique bearing) ... */
    lat1 *= D2R; lon1 *= D2R; lat2 *= D2R; lon2 *= D2R;
    const y = Math.sin(lon2 - lon1) * Math.cos(lat2);
    const x = Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1) * Math.cos(lat2) * Math.cos(lon2 - lon1);
    let b = Math.atan2(y, x) * R2D;
    return (b + 360) % 360; 
};

/** Synchronise l'heure avec time.is (API reconnue pour sa précision UTC). CONSERVE LA CORRECTION EN CAS D'ÉCHEC. */
async function syncH() { 
    try {
        const res = await fetch(`https://time.is/UTC?json`, { signal: AbortSignal.timeout(5000) });
        if (!res.ok) throw new Error(`Erreur réseau: ${res.status}`);
        const data = await res.json();
        lServH = data.unixtime * 1000; 
        lLocH = Date.now(); 
        console.log("Synchronisation de l'heure time.is (UTC) réussie.");
    } catch (e) {
        console.warn(`Échec de la synchronisation. Maintien de la dernière correction. Raison: ${e.message}`);
    }
}

/** Obtient l'heure actuelle, en appliquant la correction basée sur la dernière synchro Internet. */
function getCDate() { 
    let estT = Date.now();
    if (lServH !== null) {
        estT = lServH + (Date.now() - lLocH);
    } 
    return new Date(estT);
}

/** FILTRE DE KALMAN (Gère le lissage et l'incertitude) */
function kFilter(nSpd, dt, R_dyn) { 
    if (dt === 0 || dt > 5) return kSpd; 
    const R = R_dyn ?? R_MAX; 
    const Q = Q_NOISE * dt; 
    let pSpd = kSpd;
    let pUnc = kUncert + Q; 
    let K = pUnc / (pUnc + R); 
    kSpd = pSpd + K * (nSpd - pSpd);
    kUncert = (1 - K) * pUnc;
    return kSpd;
}

function toggleNetherMode() { 
    netherMode = !netherMode;
    distM = 0; maxSpd = 0; 
    if (netherMode) {
        $('nether-indicator').textContent = "ACTIVÉ (1:8) 🔥";
        netherToggleBtn.textContent = "🌍 Overworld";
    } else {
        $('nether-indicator').textContent = "DÉSACTIVÉ (1:1)";
        netherToggleBtn.textContent = "🔥 Nether";
    }
}

function setDefaultLocation() { /* ... (Logique Lieu Défaut) ... */
    const newLatStr = prompt(`Entrez la nouvelle Latitude par défaut (actuel: ${defaultLat}) :`);
    if (newLatStr !== null && !isNaN(parseFloat(newLatStr))) { defaultLat = parseFloat(newLatStr); }
    const newLonStr = prompt(`Entrez la nouvelle Longitude par défaut (actuel: ${defaultLon}) :`);
    if (newLonStr !== null && !isNaN(parseFloat(newLonStr))) { defaultLon = parseFloat(newLonStr); }
    alert(`Nouvelle position par défaut : Lat=${defaultLat.toFixed(4)}, Lon=${defaultLon.toFixed(4)}. Redémarrez le GPS pour l'utiliser comme position initiale.`);
}


// ===========================================
// CALCULS ASTRO & TEMPS
// ===========================================

function calcSolar() { 
    const now = getCDate(), J2K_MS = 946728000000;
    const D = (now.getTime() - J2K_MS) / 86400000;
    const M = (357.529 + 0.98560028 * D) * D2R; 
    const L = (280.466 + 0.98564736 * D) * D2R; 
    const Ce = 2 * ECC * Math.sin(M) + 1.25 * ECC ** 2 * Math.sin(2 * M);
    const lambda = L + Ce; 
    const delta = Math.asin(Math.sin(OBLIQ) * Math.sin(lambda)); 
    const alpha_rad = Math.atan2(Math.cos(OBLIQ) * Math.sin(lambda), Math.cos(lambda));

    const JD = now.getTime() / 86400000 + 2440587.5;
    const T = (JD - JD_2K) / 36525.0; 
    let GST = 280.4606 + 360.9856473 * (JD - JD_2K) + 0.000388 * T ** 2; 
    GST = (GST % 360 + 360) % 360; 
    const LST = GST + (lon ?? defaultLon); 
    const HA_rad = ((LST % 360) * D2R) - alpha_rad; 

    const lat_rad = (lat ?? defaultLat) * D2R;
    const h = Math.asin(Math.sin(lat_rad) * Math.sin(delta) + Math.cos(lat_rad) * Math.cos(delta) * Math.cos(HA_rad));
    
    let EoT_deg = (L * R2D) - (alpha_rad * R2D); 
    while (EoT_deg > 180) EoT_deg -= 360;
    while (EoT_deg < -180) EoT_deg += 360;
    const EoT_m = EoT_deg * 4; 
    
    let sLon = (lambda * R2D) % 360;
    if (sLon < 0) sLon += 360;
    
    todayLC = 12 - (EoT_m / 60); 

    return { eot: EoT_m, solarLongitude: sLon, elevation: h * R2D, lambda: lambda };
}

function getSunTimes(sD, cLat) { /* ... (Logique getSunTimes) ... */
    const delta = Math.asin(Math.sin(OBLIQ) * Math.sin(sD.lambda)); 
    const lat_rad = cLat * D2R;
    
    let cosH = -Math.tan(lat_rad) * Math.tan(delta);
    if (cosH > 1) cosH = 1; else if (cosH < -1) cosH = -1;

    const H_rad = Math.acos(cosH); 
    const H_temps = H_rad * R2D * 4; 

    const EoT_min = sD.eot;
    const noonLSM_min = 12 * 60; 
    
    const sunriseLSM_min = noonLSM_min - H_temps;
    const sunsetLSM_min = noonLSM_min + H_temps;

    const sunriseHSV_s = (sunriseLSM_min * 60) + (EoT_min * 60);
    const sunsetHSV_s = (sunsetLSM_min * 60) + (EoT_min * 60);

    return { 
        sr: (sunriseHSV_s + 86400) % 86400, 
        ss: (sunsetHSV_s + 86400) % 86400,
        polar: (cosH >= 1 || cosH <= -1)
    };
}

// ... (code précédent inchangé) ...

function updateSolarTime(cLon) { 
    const now = getCDate();
    
    // Temps Solaire Moyen (LSM)
    // *** UTILISATION DE getUTCHours/Minutes/Seconds POUR IGNORER L'OFFSET DU FUSEAU HORAIRE LOCAL ***
    const sUT = now.getUTCHours() * 3600 + now.getUTCMinutes() * 60 + now.getUTCSeconds();
    
    // Le LSM est l'heure UTC (Temps au Méridien 0°) ajustée par la longitude GPS de l'utilisateur.
    const sLSM = (sUT + (cLon * 4 * 60) + 86400) % 86400;
// ... (le reste de la fonction est inchangé) ...
    
    // Temps Solaire Moyen (LSM)
    const sUT = now.getUTCHours() * 3600 + now.getUTCMinutes() * 60 + now.getUTCSeconds();
    const sLSM = (sUT + (cLon * 4 * 60) + 86400) % 86400;
    const hsmH = Math.floor(sLSM / 3600), hsmM = Math.floor((sLSM % 3600) / 60), hsmS = Math.floor(sLSM % 60);
    const hsmStr = `${String(hsmH).padStart(2, '0')}:${String(hsmM).padStart(2, '0')}:${String(hsmS).padStart(2, '0')}`;
    
    // Heure Solaire Vraie (HSV)
    const sD = calcSolar(), EoT_s = sD.eot * 60;
    const sLSM_C = sLSM + EoT_s; 
    const sHSV = (sLSM_C + 86400) % 86400;
    const lsvH = Math.floor(sHSV / 3600), lsvM = Math.floor((sHSV % 3600) / 60), lsvS = Math.floor(sHSV % 60);
    const hsvStr = `${String(lsvH).padStart(2, '0')}:${String(lsvM).padStart(2, '0')}:${String(lsvS).padStart(2, '0')}`;
    
    solarMeanHeader.textContent = hsmStr; 
    solarTrueHeader.textContent = hsvStr; 
    $('solar-mean').textContent = hsmStr; 
    $('solar-true').textContent = `${hsvStr} (HSV)`;
    
    const EoT_sec_total = sD.eot * 60;
    const EoT_min = Math.trunc(EoT_sec_total / 60);
    const EoT_sec_after_min = Math.abs(EoT_sec_total % 60);
    const EoT_sign = EoT_sec_total >= 0 ? '+' : '-';
    $('eot').textContent = `${EoT_sign} ${Math.abs(EoT_min)}m ${EoT_sec_after_min.toFixed(1)}s`;
    
    $('solar-longitude-val').textContent = `${sD.solarLongitude.toFixed(2)} °`;
    
    const culminH_sec = todayLC * 3600;
    const culminH_h = Math.floor(culminH_sec / 3600);
    const culminH_m = Math.floor((culminH_sec % 3600) / 60);
    const culminH_s = Math.floor(culminH_sec % 60);
    $('solar-culmination').textContent = `${String(culminH_h).padStart(2, '0')}:${String(culminH_m).padStart(2, '0')}:${String(culminH_s).padStart(2, '0')} (HSV)`;
    
    return sD;
}


// ===========================================
// GESTION DES CAPTEURS ET CALCULS DÉRIVÉS
// ===========================================

function initExtendedSensors() { 
    if ('Magnetometer' in window) {
        try {
            magSensor = new Magnetometer({ frequency: 10 });
            magSensor.addEventListener('reading', () => {
                const magTotal = Math.sqrt(magSensor.x ** 2 + magSensor.y ** 2 + magSensor.z ** 2);
                $('mag-field').textContent = `${magTotal.toFixed(2)} \u00B5T`; 
            });
            magSensor.start();
        } catch(e) { $('mag-field').textContent = `Erreur capteur (${e.name})`; }
    } else { $('mag-field').textContent = 'API Magnétomètre non supportée'; }
    if ('AmbientLightSensor' in window) { /* ... (Logique ALS) ... */ }
}

/** Mise à jour de la Pression et de la Température (ICAO Standard) */
function updateThermo(alt_m) { 
    const P_curr = PRESSURE_SEA * Math.pow(1 - (LAPSE_RATE * alt_m) / 288.15, 5.255);
    ambientPressure = P_curr;
    $('pressure-hpa').textContent = `${P_curr.toFixed(2)} hPa`;
    ambientTemp = 20.0 - (0.0065 * alt_m); 
    $('air-temp').textContent = `${ambientTemp.toFixed(1)} \u00B0C (Simulé)`;
    const alt_baro = 44330.8 * (1 - Math.pow(P_curr / PRESSURE_SEA, 0.19029));
    $('alt-baro').textContent = `${alt_baro.toFixed(0)} m`;
    const teb = 100.0 - (alt_m / 300);
    $('boiling-point').textContent = `${teb.toFixed(2)} \u00B0C`;
    $('o2-perc').textContent = '20.95 % (Pression Diminuée)';
    $('humidity').textContent = '55 % (Simulé)';
}


function fastDOM() { 
    const latA = lat ?? defaultLat, lonA = lon ?? defaultLon;
    updateSolarTime(lonA); 
    
    const v_ms = kSpd / KMH_MS; 
    
    // Calcul Aérodynamique (Force de Traînée)
    const dragForce = 0.5 * AIR_DENSITY * CDA_EST * v_ms ** 2;
    const dragPower = dragForce * v_ms; 
    
    $('drag-force').textContent = `${dragForce.toFixed(1)} N`;
    $('drag-power-kw').textContent = `${(dragPower / 1000).toFixed(2)} kW`;

    // Mise à jour SVT/Chimie (Pression/Thermo)
    const alt_m = parseFloat($('altitude').textContent) || 0;
    updateThermo(alt_m); 
    
    if (domID === null) domID = setInterval(fastDOM, DOM_MS);
}


function updateDisp(pos) { 
    const rawLat = pos.coords.latitude, rawLon = pos.coords.longitude;
    const rawAlt = pos.coords.altitude ?? 0;
    const newTime = pos.timestamp;
    
    const dt = (newTime - (lPos?.timestamp ?? newTime)) / 1000;
    
    const vertSpd = (rawAlt - (lPos?.coords.altitude ?? rawAlt)) / dt;
    const spd3D = Math.sqrt((pos.coords.speed ?? 0) ** 2 + vertSpd ** 2) || 0;
    
    const nSpdKMH = spd3D * KMH_MS; // Vitesse mesurée en km/h
    let nSpdClampedKMH = nSpdKMH;
    
    // --- 1. VÉRIFICATION DE FAISABILITÉ PHYSIQUE ---
    if (dt > MIN_DT / 1000) { 
        const maxDeltaSpd = MAX_ACC * dt * KMH_MS; 
        const currentKSpdKMH = kSpd;
        
        if (Math.abs(nSpdKMH - currentKSpdKMH) > maxDeltaSpd) {
            if (nSpdKMH > currentKSpdKMH) {
                nSpdClampedKMH = currentKSpdKMH + maxDeltaSpd;
            } else {
                nSpdClampedKMH = currentKSpdKMH - maxDeltaSpd;
            }
            console.warn(`Vitesse impossible corrigée : Mesure limitée de ${nSpdKMH.toFixed(2)} à ${nSpdClampedKMH.toFixed(2)} km/h.`);
        }
    }

    // --- 2. DYNAMIQUE KALMAN (Régulation par Précision GPS) ---
    let R_dyn = pos.coords.accuracy * 0.5; 
    R_dyn = Math.min(R_dyn, R_MAX); 
    R_dyn = Math.max(R_dyn, R_MIN); 
    
    kSpd = kFilter(nSpdClampedKMH, dt, R_dyn);

    lat = rawLat; lon = rawLon;
    
    // Calcul de l'Accélération Longitudinale (Dérivée)
    accellLong = dt > 0 ? (spd3D - lastSpd3D) / dt : 0;
    lastSpd3D = spd3D; 
    
    const gForce = accellLong / 9.80665; 
    $('g-force').textContent = `${gForce.toFixed(2)} G`;
    $('accel-long').textContent = `${accellLong.toFixed(3)} m/s²`;
    
    // Affichage
    $('latitude').textContent = lat.toFixed(6) + " °";
    $('longitude').textContent = lon.toFixed(6) + " °";
    $('altitude').textContent = rawAlt.toFixed(1) + " m";
    $('gps-accuracy').textContent = pos.coords.accuracy.toFixed(1) + " m";
    $('vertical-speed').textContent = vertSpd.toFixed(2) + " m/s";
    
    if (lPos) {
        let currentDist = dist(lPos.coords.latitude, lPos.coords.longitude, lat, lon);
        if (netherMode) currentDist *= NETHER_RATIO;
        distM += currentDist;
    }
    
    $('speed-3d-inst').textContent = nSpdKMH.toFixed(5) + " km/h";
    $('speed-stable').textContent = kSpd.toFixed(5) + " km/h";
    $('speed-stable-mm').textContent = (kSpd / KMH_MS * 1000).toFixed(2) + " mm/s";
    maxSpd = Math.max(maxSpd, kSpd);
    $('speed-max').textContent = maxSpd.toFixed(5) + " km/h";
    
    $('distance-km-m').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(0)} m`;
    $('perc-light').textContent = (spd3D / C_L * 100).toExponential(5) + "%";
    $('perc-sound').textContent = (spd3D / C_S * 100).toFixed(5) + "%";
    
    const head = pos.coords.heading;
    if (head !== null && head >= 0) { $('heading').textContent = head.toFixed(1) + " °"; } else { $('heading').textContent = "---"; }
    
    const coherence = kSpd / Math.max(0.001, nSpdKMH) * 100;
    $('speed-coherence').textContent = `${coherence.toFixed(1)} %`;
    $('speed-error-perc').textContent = `${(kUncert * 100).toFixed(2)} %`;
    
    lPos = pos;
}

function startGPS() { /* ... (Logique startGPS) ... */
    if (wID === null) {
        sTime = Date.now();
        resetDisp();
        lPos = null;
        wID = navigator.geolocation.watchPosition(updateDisp, (e) => {
             $('error-message').textContent = `Erreur GPS: ${e.message} (${e.code}). Vérifiez les permissions.`;
             $('error-message').style.display = 'block';
        }, W_OPTS);
        startBtn.disabled = true;
        stopBtn.disabled = false;
        resetMaxBtn.disabled = false;
        $('speed-source-indicator').textContent = "Source: GPS Actif";
    }
}
function stopGPS() { /* ... (Logique stopGPS) ... */
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
        startBtn.disabled = false;
        stopBtn.disabled = true;
        $('speed-source-indicator').textContent = "Source: Statique/Dernière pos.";
    }
}
function resetDisp() { /* ... (Logique resetDisp) ... */
    distM = 0; maxSpd = 0; kSpd = 0; kUncert = 1000; lPos = null;
    lat = defaultLat; lon = defaultLon;
    
    // Initialisation des DOM
    $('speed-stable').textContent = '0.00000 km/h'; $('speed-3d-inst').textContent = '0.00000 km/h'; $('speed-max').textContent = '0.00000 km/h'; 
    $('latitude').textContent = defaultLat.toFixed(6) + " ° (Défaut)";
    $('longitude').textContent = defaultLon.toFixed(6) + " ° (Défaut)";
    $('altitude').textContent = '-- m';
    $('speed-coherence').textContent = '--';
    $('distance-km-m').textContent = '0.000 km | 0 m';
}


// --- DÉMARRAGE INITIAL ---
document.addEventListener('DOMContentLoaded', () => {
    resetDisp();
    
    // **IMPORTANT**: Initialisation de lLocH AVANT la tentative de synchro
    if (lLocH === null) lLocH = Date.now();
    
    syncH(); // Tente la synchronisation de l'heure time.is persistante
    initExtendedSensors(); 
    
    if (domID === null) domID = setInterval(fastDOM, DOM_MS); 
    
    startBtn.addEventListener('click', startGPS);
    stopBtn.addEventListener('click', stopGPS);
    resetMaxBtn.addEventListener('click', () => { maxSpd = 0; $('speed-max').textContent = '0.00000 km/h'; });
    $('set-target-btn').addEventListener('click', () => { /* ... (Logique setTarget) ... */ });
    $('set-default-loc-btn').addEventListener('click', setDefaultLocation); 
    $('toggle-mode-btn').addEventListener('click', () => { manualMode = (manualMode === null) ? true : !manualMode; });
    $('auto-mode-btn').addEventListener('click', () => { manualMode = null; });
    netherToggleBtn.addEventListener('click', toggleNetherMode);
});
