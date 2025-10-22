// =================================================================
// FICHIER COMPLET ET FINAL : dashboard.js (VERSION UTC CORRIGÉE, KALMAN, ASTRO UNIFIÉE)
// Logique getCDate() raffinée pour clarifier la dépendance à l'offset externe.
// =================================================================

// --- CONSTANTES GLOBALES ET INITIALISATION ---
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const C_L = 299792458, C_S = 343, R_E = 6371000, KMH_MS = 3.6;
const OBLIQ = 23.44 * D2R, ECC = 0.0167, JD_2K = 2451545.0; // Constantes Astronomiques
const W_OPTS = { enableHighAccuracy: true, maximumAge: 0, timeout: 20000 };
const DOM_MS = 17, MIN_DT = 1, MAX_ACC = 10; 
const Q_NOISE = 0.005, R_MIN = 0.005, R_MAX = 5.0; // Paramètres Kalman
const NETHER_RATIO = 8; 
const AIR_DENSITY = 1.225; 
const CDA_EST = 0.6;      
const PRESSURE_SEA = 1013.25; 
const LAPSE_RATE = 0.0065;   
const B_EARTH_AVG = 50.0; 

let defaultLat = 48.8566; 
let defaultLon = 2.3522;  

let wID = null, domID = null, lPos = null, lat = null, lon = null, sTime = null;
let distM = 0, maxSpd = 0, lDomT = null;
let kSpd = 0, kUncert = 1000; 
let lServH = null, lLocH = null; // Heure time.is pour correction UTC
let netherMode = false; 

let lastSpd3D = 0;        
let accellLong = 0;
let todayLC = 0; // Heure de culmination (Midi solaire vrai)

// --- REFERENCES DOM ---
const $ = id => document.getElementById(id);
const startBtn = $('start-btn'), stopBtn = $('stop-btn'), resetMaxBtn = $('reset-max-btn');
const solarTrueHeader = $('solar-true-header'); 
const solarMeanHeader = $('solar-mean-header'); 

// ===========================================
// FONCTIONS GÉO & UTILS
// ===========================================

const dist = (lat1, lon1, lat2, lon2) => { 
    // Calcul de la distance de Haversine
    const R = R_E, dLat = (lat2 - lat1) * D2R, dLon = (lon2 - lon1) * D2R;
    lat1 *= D2R; lat2 *= D2R;
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
};

async function syncH() { 
    // Tente de synchroniser l'heure UTC via time.is
    try {
        const res = await fetch(`https://time.is/UTC?json`, { signal: AbortSignal.timeout(5000) });
        if (!res.ok) throw new Error(`Erreur réseau: ${res.status}`);
        const data = await res.json();
        lServH = data.unixtime * 1000; // Timestamp UTC du serveur
        lLocH = Date.now();            // Timestamp local au moment de la réception
    } catch (e) {
        console.warn(`Échec de la synchronisation de l'heure UTC. Raison: ${e.message}. Utilisation de l'horloge locale.`);
    }
}

/** * Récupère le timestamp UTC actuel, corrigé si la synchronisation time.is a réussi.
 * C'est le point d'intégration de la correction de l'offset.
 */
function getCDate() { 
    const currentLocalTime = Date.now(); 
    
    if (lServH !== null && lLocH !== null) {
        // Utilise l'offset calculé : Heure Serveur + Temps écoulé depuis la synchro
        const offsetSinceSync = currentLocalTime - lLocH;
        return new Date(lServH + offsetSinceSync); 
    } else {
        // Repli : utilise l'heure locale brute du système (moins fiable)
        return new Date(currentLocalTime); 
    }
}

/** FILTRE DE KALMAN (Lissage de la vitesse) */
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

function setDefaultLocation() { 
    const newLatStr = prompt(`Entrez la nouvelle Latitude par défaut (actuel: ${defaultLat}) :`);
    if (newLatStr !== null && !isNaN(parseFloat(newLatStr))) { defaultLat = parseFloat(newLatStr); }
    const newLonStr = prompt(`Entrez la nouvelle Longitude par défaut (actuel: ${defaultLon}) :`);
    if (newLonStr !== null && !isNaN(parseFloat(newLonStr))) { defaultLon = parseFloat(newLonStr); }
    alert(`Nouvelle position par défaut : Lat=${defaultLat.toFixed(4)}, Lon=${defaultLon.toFixed(4)}.`);
}

// ===========================================
// CALCULS ASTRO & TEMPS 
// ===========================================

/** Calcule la position du Soleil (EoT, Élévation, Longueur Solaire) et la Culmination. */
function calcSolar() { 
    const now = getCDate(), J2K_MS = 946728000000;
    const D = (now.getTime() - J2K_MS) / 86400000;
    
    const M = (357.529 + 0.98560028 * D) * D2R; 
    const L = (280.466 + 0.98564736 * D) * D2R; 
    const Ce = 2 * ECC * Math.sin(M) + 1.25 * ECC ** 2 * Math.sin(2 * M);
    const lambda = L + Ce; 

    const delta = Math.asin(Math.sin(OBLIQ) * Math.sin(lambda)); 
    const alpha_rad = Math.atan2(Math.cos(OBLIQ) * Math.sin(lambda), Math.cos(lambda));
    const alpha = alpha_rad * R2D; 

    const JD = now.getTime() / 86400000 + 2440587.5;
    const T = (JD - JD_2K) / 36525.0; 
    let GST = 280.4606 + 360.9856473 * (JD - JD_2K) + 0.000388 * T ** 2; 
    GST = (GST % 360 + 360) % 360; 
    const LST = GST + (lon ?? defaultLon); 
    const HA_rad = ((LST % 360) * D2R) - alpha_rad; 

    const lat_rad = (lat ?? defaultLat) * D2R;
    const h = Math.asin(Math.sin(lat_rad) * Math.sin(delta) + Math.cos(lat_rad) * Math.cos(delta) * Math.cos(HA_rad));
    
    let EoT_deg = (L * R2D) - alpha; 
    while (EoT_deg > 180) EoT_deg -= 360;
    while (EoT_deg < -180) EoT_deg += 360;
    const EoT_m = EoT_deg * 4; 

    let sLon = (lambda * R2D) % 360;
    if (sLon < 0) sLon += 360;
    
    const noonLSM_H = 12; 
    const noonHSV_H = noonLSM_H - (EoT_m / 60); 
    
    todayLC = noonHSV_H; 

    return { eot: EoT_m, solarLongitude: sLon, elevation: h * R2D };
}

/** Calcule et affiche l'Heure Solaire Vraie (HSV) et le temps de Culmination (culmination) */
function updateSolarTime(cLon) { 
    const now = getCDate();
    
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
    
    // Affichage des données (LSM et HSV)
    $('solar-mean').textContent = hsmStr;
    $('solar-true').textContent = `${hsvStr} (HSV)`;
    solarTrueHeader.textContent = hsvStr; 
    solarMeanHeader.textContent = hsmStr; 
    
    // Format EoT
    const EoT_sec_total = sD.eot * 60;
    const EoT_min = Math.trunc(EoT_sec_total / 60);
    const EoT_sec_after_min = Math.abs(EoT_sec_total % 60);
    const EoT_sign = EoT_sec_total >= 0 ? '+' : '-';
    $('eot').textContent = `${EoT_sign} ${Math.abs(EoT_min)}m ${EoT_sec_after_min.toFixed(1)}s`;
    
    $('solar-longitude-val').textContent = `${sD.solarLongitude.toFixed(2)} °`;
    $('sun-elevation').textContent = `${sD.elevation.toFixed(2)} °`;
    
    // Stockage et formatage du temps de culmination
    const culminH_sec = todayLC * 3600;
    const culminH_h = Math.floor(culminH_sec / 3600);
    const culminH_m = Math.floor((culminH_sec % 3600) / 60);
    const culminH_s = Math.floor(culminH_sec % 60);
    $('solar-culmination').textContent = `${String(culminH_h).padStart(2, '0')}:${String(culminH_m).padStart(2, '0')}:${String(culminH_s).padStart(2, '0')} (LSM)`;

    return sD; 
}


// ===========================================
// GESTION DES CAPTEURS ET CALCULS DÉRIVÉS
// ===========================================

function initExtendedSensors() { 
    // Magnétomètre
    if ('Magnetometer' in window) {
        try {
            const magSensor = new Magnetometer({ frequency: 10 });
            magSensor.addEventListener('reading', () => {
                const magTotal = Math.sqrt(magSensor.x ** 2 + magSensor.y ** 2 + magSensor.z ** 2);
                $('mag-field').textContent = `${magTotal.toFixed(2)} \u00B5T`; 
            });
            magSensor.addEventListener('error', (e) => {
                console.error(`Erreur Magnetometer : ${e.message}`);
                $('mag-field').textContent = `${B_EARTH_AVG.toFixed(2)} \u00B5T (Erreur Capteur)`; 
            });
            magSensor.start();
        } catch(e) { 
            $('mag-field').textContent = `${B_EARTH_AVG.toFixed(2)} \u00B5T (API Erreur)`; 
        }
    } else {
        $('mag-field').textContent = `${B_EARTH_AVG.toFixed(2)} \u00B5T (Estimé)`;
    }

    // Luminosité (ALS)
    if ('AmbientLightSensor' in window) { 
        $('illuminance-lux').textContent = "Capteur ALS non actif.";
    } else {
        $('illuminance-lux').textContent = "API ALS non supportée.";
    }
}

/** Mise à jour de la Pression et de la Température (ICAO Standard) */
function updateThermo(alt_m) { 
    const P_curr = PRESSURE_SEA * Math.pow(1 - (LAPSE_RATE * alt_m) / 288.15, 5.255);
    $('pressure-hpa').textContent = `${P_curr.toFixed(2)} hPa`;
    
    const ambientTemp = 20.0 - (LAPSE_RATE * alt_m); 
    $('air-temp').textContent = `${ambientTemp.toFixed(1)} \u00B0C (Simulé)`;
    
    const teb = 100.0 - (alt_m / 300);
    $('boiling-point').textContent = `${teb.toFixed(2)} \u00B0C`;
}

/** Mise à jour de la Phase Lunaire (Simplifiée) et Culmination Lunaire (Simulée) */
function updateLunarInfo() {
    const now = getCDate();
    const D = (now.getTime() / 86400000) - 10957.5; 
    const N = D / 29.530588853; 
    const D_rad = (N - Math.floor(N)) * 2 * Math.PI;
    const ill = 0.5 * (1 - Math.cos(D_rad)); 
    $('lunar-phase-perc').textContent = `${(ill * 100).toFixed(1)}%`;

    const transitH = (now.getUTCHours() + 1) % 24; 
    $('lunar-culmination').textContent = `${String(transitH).padStart(2, '0')}:00:00 (Simulé UTC)`;
}


function fastDOM() { 
    const latA = lat ?? defaultLat, lonA = lon ?? defaultLon;
    
    updateSolarTime(lonA); 
    updateLunarInfo();

    const v_ms = kSpd / KMH_MS; 
    const dragForce = 0.5 * AIR_DENSITY * CDA_EST * v_ms ** 2;
    const dragPower = dragForce * v_ms; 
    
    $('drag-force').textContent = `${dragForce.toFixed(1)} N`;
    $('drag-power-kw').textContent = `${(dragPower / 1000).toFixed(2)} kW`;

    const alt_m = parseFloat($('altitude').textContent) || 0;
    updateThermo(alt_m); 
    
    const now = performance.now();
    if (lDomT) {
        const freq = 1000 / (now - lDomT);
        $('update-frequency').textContent = `${freq.toFixed(1)} Hz (${DOM_MS} ms)`;
    }
    lDomT = now;
    
    if (domID === null) domID = setInterval(fastDOM, DOM_MS);
}


function updateDisp(pos) { 
    const rawLat = pos.coords.latitude, rawLon = pos.coords.longitude;
    const rawAlt = pos.coords.altitude ?? 0;
    const newTime = pos.timestamp;
    
    const dt = (newTime - (lPos?.timestamp ?? newTime)) / 1000;
    
    const vertSpd = dt > 0 ? (rawAlt - (lPos?.coords.altitude ?? rawAlt)) / dt : 0;
    const spd3D = Math.sqrt((pos.coords.speed ?? 0) ** 2 + vertSpd ** 2) || 0;
    
    const nSpdKMH = spd3D * KMH_MS; 
    let nSpdClampedKMH = nSpdKMH;
    
    if (dt > MIN_DT / 1000) { 
        const maxDeltaSpd = MAX_ACC * dt * KMH_MS; 
        const currentKSpdKMH = kSpd;
        
        if (Math.abs(nSpdKMH - currentKSpdKMH) > maxDeltaSpd) {
            nSpdClampedKMH = nSpdKMH > currentKSpdKMH ? currentKSpdKMH + maxDeltaSpd : currentKSpdKMH - maxDeltaSpd;
        }
    }

    let R_dyn = pos.coords.accuracy * 0.5; 
    R_dyn = Math.min(R_dyn, R_MAX); 
    R_dyn = Math.max(R_dyn, R_MIN); 
    kSpd = kFilter(nSpdClampedKMH, dt, R_dyn); 

    lat = rawLat; lon = rawLon;
    
    accellLong = dt > 0 ? (spd3D - lastSpd3D) / dt : 0;
    lastSpd3D = spd3D; 
    
    const gForce = accellLong / 9.80665; 
    $('g-force').textContent = `${gForce.toFixed(2)} G`;
    $('accel-long').textContent = `${accellLong.toFixed(3)} m/s²`;
    
    if (lPos) {
        let currentDist = dist(lPos.coords.latitude, lPos.coords.longitude, lat, lon);
        distM += currentDist * (netherMode ? NETHER_RATIO : 1);
    }
    
    if (sTime) {
        const elapsedSec = (getCDate().getTime() - sTime) / 1000; 
        $('elapsed-time').textContent = `${elapsedSec.toFixed(1)} s`;
    }
    
    $('speed-stable').textContent = kSpd.toFixed(5) + " km/h";
    $('speed-3d-inst').textContent = nSpdKMH.toFixed(5) + " km/h";
    $('speed-stable-mm').textContent = (kSpd / KMH_MS * 1000).toFixed(2) + " mm/s";
    maxSpd = Math.max(maxSpd, kSpd);
    $('speed-max').textContent = maxSpd.toFixed(5) + " km/h";
    $('distance-km-m').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(0)} m`;
    
    $('latitude').textContent = lat.toFixed(6) + " °";
    $('longitude').textContent = lon.toFixed(6) + " °";
    $('altitude').textContent = rawAlt.toFixed(1) + " m";
    $('gps-accuracy').textContent = pos.coords.accuracy.toFixed(1) + " m";
    $('vertical-speed').textContent = vertSpd.toFixed(2) + " m/s";

    $('perc-light').textContent = `${(spd3D / C_L * 100).toExponential(3)} %`;
    $('perc-sound').textContent = `${(spd3D / C_S * 100).toFixed(2)} %`;
    
    const coherence = kSpd / Math.max(0.001, nSpdKMH) * 100;
    $('speed-coherence').textContent = `${coherence.toFixed(1)} %`;
    $('speed-error-perc').textContent = `${(kUncert * 100).toFixed(2)} %`;
    
    lPos = pos;
}

function startGPS() { 
    if (wID === null) {
        sTime = getCDate().getTime(); 
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
function stopGPS() { 
    if (wID !== null) {
        navigator.geolocation.clearWatch(wID);
        wID = null;
        startBtn.disabled = false;
        stopBtn.disabled = true;
        $('speed-source-indicator').textContent = "Source: Statique/Dernière pos.";
    }
}
function resetDisp() { 
    distM = 0; maxSpd = 0; kSpd = 0; kUncert = 1000; lPos = null;
    lat = defaultLat; lon = defaultLon;
    sTime = null; 
    $('latitude').textContent = defaultLat.toFixed(6) + " ° (Défaut)";
    $('longitude').textContent = defaultLon.toFixed(6) + " ° (Défaut)";
    $('distance-km-m').textContent = '0.000 km | 0 m';
    $('elapsed-time').textContent = '-- s';
    $('speed-stable').textContent = '0.00000 km/h';
    $('speed-max').textContent = '0.00000 km/h';
}

// --- DÉMARRAGE INITIAL ---
document.addEventListener('DOMContentLoaded', () => {
    resetDisp();
    // On lance la synchro immédiatement. Le DOM se rafraîchira avec l'heure corrigée dès qu'elle est disponible.
    syncH(); 
    initExtendedSensors(); 
    
    if (domID === null) domID = setInterval(fastDOM, DOM_MS); 
    
    startBtn.addEventListener('click', startGPS);
    stopBtn.addEventListener('click', stopGPS);
    $('reset-max-btn').addEventListener('click', () => { maxSpd = 0; $('speed-max').textContent = '0.00000 km/h'; });
    $('set-default-loc-btn').addEventListener('click', setDefaultLocation); 
    $('nether-toggle-btn').addEventListener('click', () => {
        netherMode = !netherMode;
        distM = 0; maxSpd = 0; 
        $('nether-indicator').textContent = netherMode ? "ACTIVÉ (1:8) 🔥" : "DÉSACTIVÉ (1:1)";
        $('nether-toggle-btn').textContent = netherMode ? "🌍 Overworld" : "🔥 Nether";
    });
});
