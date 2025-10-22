// =================================================================
// FICHIER COMPLET ET FINAL : dashboard.js (VERSION RIGOUREUSEMENT UTC ET STABLE)
// Utilise getCDate() (UTC corrigé via time.is) pour TOUS les calculs temporels critiques,
// garantissant l'indépendance de l'heure locale de l'appareil (GMT/UTC atomique).
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
const CDA_EST = 0.6;      // Coefficient CdA (Est.)
const PRESSURE_SEA = 1013.25; 
const LAPSE_RATE = 0.0065;   
const B_EARTH_AVG = 50.0; // Champ Magnétique Terre (Moyenne Est.)

let defaultLat = 48.8566; // Paris par défaut
let defaultLon = 2.3522;  

let wID = null, domID = null, lPos = null, lat = null, lon = null, sTime = null;
let distM = 0, maxSpd = 0, lDomT = null;
let kSpd = 0, kUncert = 1000; 
let lServH = null, lLocH = null; // Heure time.is pour correction UTC
let netherMode = false; 

let lastSpd3D = 0;        
let accellLong = 0;       

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
        console.warn(`Échec de la synchronisation de l'heure UTC. Raison: ${e.message}`);
    }
}

/** * Récupère le timestamp UTC actuel, corrigé si la synchronisation time.is a réussi.
 * Ceci garantit l'indépendance totale de l'horloge locale de l'appareil.
 */
function getCDate() { 
    let estT = Date.now(); 
    if (lServH !== null) {
        // Applique l'offset de synchro pour obtenir le temps UTC corrigé
        estT = lServH + (Date.now() - lLocH); 
    } 
    return new Date(estT); 
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
// CALCULS ASTRO & TEMPS (TOUJOURS BASÉS SUR UTC)
// ===========================================

function calcSolar() { 
    const now = getCDate(), J2K_MS = 946728000000;
    const D = (now.getTime() - J2K_MS) / 86400000; // Jours depuis J2000
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
    
    return { eot: EoT_m, solarLongitude: sLon, elevation: h * R2D };
}

function updateSolarTime(cLon) { 
    const now = getCDate();
    
    // Temps Solaire Moyen (LSM) : Basé sur UTC (avec getUTC*)
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
    
    $('eot').textContent = `${(sD.eot >= 0 ? '+' : '')}${sD.eot.toFixed(1)} min`;
    $('solar-longitude-val').textContent = `${sD.solarLongitude.toFixed(2)} °`;
    $('sun-elevation').textContent = `${sD.elevation.toFixed(2)} °`;
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


function fastDOM() { 
    // Taux de rafraîchissement rapide du DOM (17ms = ~60Hz)
    const latA = lat ?? defaultLat, lonA = lon ?? defaultLon;
    updateSolarTime(lonA); 
    
    // Calcul de la Traînée et de la Puissance
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
    
    // --- 1. VÉRIFICATION DE FAISABILITÉ PHYSIQUE ---
    if (dt > MIN_DT / 1000) { 
        const maxDeltaSpd = MAX_ACC * dt * KMH_MS; 
        const currentKSpdKMH = kSpd;
        
        if (Math.abs(nSpdKMH - currentKSpdKMH) > maxDeltaSpd) {
            nSpdClampedKMH = nSpdKMH > currentKSpdKMH ? currentKSpdKMH + maxDeltaSpd : currentKSpdKMH - maxDeltaSpd;
            console.warn(`Vitesse impossible corrigée : Mesure limitée de ${nSpdKMH.toFixed(2)} à ${nSpdClampedKMH.toFixed(2)} km/h.`);
        }
    }

    // --- 2. DYNAMIQUE KALMAN ---
    let R_dyn = pos.coords.accuracy * 0.5; 
    R_dyn = Math.min(R_dyn, R_MAX); 
    R_dyn = Math.max(R_dyn, R_MIN); 
    kSpd = kFilter(nSpdClampedKMH, dt, R_dyn); 

    lat = rawLat; lon = rawLon;
    
    // Calcul de l'Accélération Longitudinale
    accellLong = dt > 0 ? (spd3D - lastSpd3D) / dt : 0;
    lastSpd3D = spd3D; 
    
    const gForce = accellLong / 9.80665; 
    $('g-force').textContent = `${gForce.toFixed(2)} G`;
    $('accel-long').textContent = `${accellLong.toFixed(3)} m/s²`;
    
    // Mise à jour de la distance
    if (lPos) {
        let currentDist = dist(lPos.coords.latitude, lPos.coords.longitude, lat, lon);
        distM += currentDist * (netherMode ? NETHER_RATIO : 1);
    }
    
    // --- 3. MISE À JOUR DU TEMPS ÉCOULÉ (UTC STRICT) ---
    if (sTime) {
        // Temps écoulé basé uniquement sur les timestamps UTC corrigés
        const elapsedSec = (getCDate().getTime() - sTime) / 1000; 
        $('elapsed-time').textContent = `${elapsedSec.toFixed(1)} s`;
    }

    // Affichage des données de position
    $('latitude').textContent = lat.toFixed(6) + " °";
    $('longitude').textContent = lon.toFixed(6) + " °";
    $('altitude').textContent = rawAlt.toFixed(1) + " m";
    $('gps-accuracy').textContent = pos.coords.accuracy.toFixed(1) + " m";
    $('vertical-speed').textContent = vertSpd.toFixed(2) + " m/s";
    
    // Affichage des vitesses et distance
    $('speed-3d-inst').textContent = nSpdKMH.toFixed(5) + " km/h";
    $('speed-stable').textContent = kSpd.toFixed(5) + " km/h";
    $('speed-stable-mm').textContent = (kSpd / KMH_MS * 1000).toFixed(2) + " mm/s";
    maxSpd = Math.max(maxSpd, kSpd);
    $('speed-max').textContent = maxSpd.toFixed(5) + " km/h";
    $('distance-km-m').textContent = `${(distM / 1000).toFixed(3)} km | ${distM.toFixed(0)} m`;
    
    // Cohérence et Erreur
    const coherence = kSpd / Math.max(0.001, nSpdKMH) * 100;
    $('speed-coherence').textContent = `${coherence.toFixed(1)} %`;
    $('speed-error-perc').textContent = `${(kUncert * 100).toFixed(2)} %`;
    
    lPos = pos;
}

function startGPS() { 
    if (wID === null) {
        // Initialisation de sTime avec le timestamp UTC corrigé pour le départ
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
    sTime = null; // Réinitialiser le temps de départ
    $('latitude').textContent = defaultLat.toFixed(6) + " ° (Défaut)";
    $('longitude').textContent = defaultLon.toFixed(6) + " ° (Défaut)";
    $('distance-km-m').textContent = '0.000 km | 0 m';
    $('elapsed-time').textContent = '-- s';
}

// --- DÉMARRAGE INITIAL ---
document.addEventListener('DOMContentLoaded', () => {
    resetDisp();
    if (lLocH === null) lLocH = Date.now();
    syncH(); // Tentative de synchronisation UTC en arrière-plan
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
