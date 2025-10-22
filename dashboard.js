// ====================================================================
// --- CONSTANTES GLOBALES ET INITIALISATION ---
// ====================================================================

const D2R = Math.PI / 180; // Degrés vers Radians
const MIN_SPD = 0.5; // Vitesse minimale (m/s) pour considérer que l'on est en mouvement
const C_L = 299792458; // Vitesse de la lumière dans le vide (m/s)
const EARTH_RADIUS = 6371000; // Rayon moyen de la Terre en mètres

// Kalman State: [position, vitesse] (Nous filtrons uniquement la vitesse spd3D pour la simplicité)
let kX = 0; // State: Vitesse Estimée
let kP = 1; // Covariance d'Erreur
let kQ = 0.05; // Covariance de Bruit de Processus (Dynamique)

// Position et Temps
let lPos = null;       // Dernière position GPS brute reçue
let lastTime = 0;      // Dernier timestamp de mise à jour GPS
let sTime = 0;         // Temps écoulé total (s)
let lat = null, lon = null, alt = null; // Position actuelle (filtrée ou figée)

// Trajet
let totalDistance = 0; // Distance totale accumulée (m)
let lastAlt = null;    // Dernière altitude pour le dénivelé
let totalClimb = 0;    // Dénivelé positif accumulé (m)
let totalDescent = 0;  // Dénivelé négatif accumulé (m)

// IMU (Accélération 3D)
let lastAcceleration = { x: 0, y: 0, z: 0 };
let lastAccelTime = 0;

// Corrections de Biais (Multipath)
const MULTIPATH_BIAS = {
    'NONE': { lat_offset_m: 0, lon_offset_m: 0, desc: "0.0m / 0.0m" },
    'METAL': { lat_offset_m: 5.0, lon_offset_m: 5.0, desc: "5.0m / 5.0m" },
    'CONCRETE': { lat_offset_m: 3.5, lon_offset_m: 3.5, desc: "3.5m / 3.5m" },
    'WET_GROUND': { lat_offset_m: 2.0, lon_offset_m: 2.0, desc: "2.0m / 2.0m" },
    'FOLIAGE': { lat_offset_m: 1.0, lon_offset_m: 1.0, desc: "1.0m / 1.0m" }
};
let currentMaterialBias = 'NONE';
let isStationary = false; // État de figement de la position

// Constantes pour la correction atmosphérique
const K1 = 77.6; // K/mbar
const K2 = 6.45; // K/mbar
const K3 = 3.77 * 10**5; // K^2/mbar


// ====================================================================
// --- FONCTIONS MATHÉMATIQUES ET FILTRE ---
// ====================================================================

function $(id) { return document.getElementById(id); }

/**
 * Filtre de Kalman 1D (pour la vitesse).
 * @param {number} Z - Mesure brute (vitesse GPS).
 * @param {number} dt - Intervalle de temps (s).
 * @param {number} R - Covariance de Bruit de Mesure (dépend de l'accuracy GPS).
 * @returns {number} La vitesse filtrée.
 */
function kFilter(Z, dt, R) {
    // Étape de Prédiction
    kX = kX; // Modèle simple : vitesse constante
    kP = kP + kQ * dt;

    // Étape de Mise à Jour (Correction)
    const K = kP / (kP + R); // Gain de Kalman
    kX = kX + K * (Z - kX); // Nouvelle estimation
    kP = (1 - K) * kP; // Nouvelle covariance d'erreur

    return kX;
}

/**
 * Calcule la distance en mètres entre deux coordonnées (Formule d'Haversine simplifiée).
 */
function haversine(lat1, lon1, lat2, lon2) {
    const dLat = (lat2 - lat1) * D2R;
    const dLon = (lon2 - lon1) * D2R;
    const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
              Math.cos(lat1 * D2R) * Math.cos(lat2 * D2R) *
              Math.sin(dLon / 2) * Math.sin(dLon / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return EARTH_RADIUS * c;
}

/**
 * Convertit un décalage en mètres en degrés de Lat/Lon.
 */
function metersToDegrees(meters, lat) {
    const dLat = meters / (EARTH_RADIUS * D2R);
    const dLon = meters / (EARTH_RADIUS * D2R * Math.cos(lat * D2R));
    return { dLat: dLat, dLon: dLon };
}

/**
 * Applique le décalage Multipath sélectionné aux coordonnées.
 */
function applyMaterialBias(rawLat, rawLon) {
    const bias = MULTIPATH_BIAS[currentMaterialBias];
    
    if (!bias) return { correctedLat: rawLat, correctedLon: rawLon };
    
    // Convertir les mètres en degrés à la latitude actuelle
    const offsetLatLon = metersToDegrees(bias.lat_offset_m, rawLat);
    
    // Appliquer un simple décalage Nord-Est (pour la démo)
    const correctedLat = rawLat + offsetLatLon.dLat;
    const correctedLon = rawLon + offsetLatLon.dLon;

    return { correctedLat, correctedLon };
}

/**
 * Calcule la vitesse de la lumière (v) et l'indice de réfraction (n) dans l'air.
 */
function calculateAtmosphericSpeed(P_hPa, T_C, H_perc) {
    const T_K = T_C + 273.15;
    const Es = 6.1078 * Math.exp(17.27 * T_C / (T_C + 237.3));
    const Pw = Es * (H_perc / 100.0); 
    const P_dry = P_hPa - Pw;

    // N = indice de réfractivité troposphérique
    const N = K1 * (P_dry / T_K) + K2 * (Pw / T_K) + K3 * (Pw / (T_K * T_K));
    
    const n = 1 + (N * 1e-6);
    const v = C_L / n; 

    return { vitesse: v, n: n, refractivity: N };
}


// ====================================================================
// --- GESTION DES CAPTEURS ET MISE À JOUR ---
// ====================================================================

/**
 * Gère les données de position GPS.
 */
function handleGPS(pos) {
    const currentTime = pos.timestamp;
    const acc = pos.coords.accuracy || 100;

    // Calcul du temps écoulé (dt)
    const dt = (lastTime > 0) ? (currentTime - lastTime) / 1000 : 0;
    
    // Calcul de la vitesse GPS 3D (Z = mesure brute)
    const spd3D = pos.coords.speed !== null ? pos.coords.speed : 0;
    
    // --- 1. Calcul de l'Accélération 3D (pour le filtre IMU/GPS) ---
    const accelerationMagnitude = Math.sqrt(
        lastAcceleration.x**2 + 
        lastAcceleration.y**2 + 
        lastAcceleration.z**2
    );
    const ACCEL_THRESHOLD = 0.5;
    
    // --- 2. Définir le R (Bruit de Mesure) basé sur l'incertitude du GPS et l'IMU ---
    // R est proportionnel au carré de l'incertitude.
    let kR = acc * acc; 
    
    // Si l'IMU dit que nous sommes très stables, on augmente R pour lisser davantage
    if (accelerationMagnitude < ACCEL_THRESHOLD) {
        kR = kR * 10; 
    }
    
    const R_MIN = 0.01; 
    const R_MAX = 5000.0;
    kR = Math.min(Math.max(kR, R_MIN), R_MAX);
    
    // --- 3. Filtrage de Kalman pour la Vitesse ---
    const fSpd = kFilter(spd3D, dt, kR);
    const sSpdFE = fSpd < MIN_SPD ? 0 : fSpd; // Vitesse Stable (0 si < MIN_SPD)

    // --- 4. Calcul de la Distance et du Dénivelé ---
    if (lPos) {
        // Distance totale
        const dist_delta = haversine(lPos.coords.latitude, lPos.coords.longitude, pos.coords.latitude, pos.coords.longitude);
        totalDistance += dist_delta;

        // Dénivelé (si altitude est fournie)
        const currentAlt = pos.coords.altitude;
        if (currentAlt !== null && lastAlt !== null && !isNaN(currentAlt)) {
            const altDiff = currentAlt - lastAlt;
            if (altDiff > 0) totalClimb += altDiff;
            else if (altDiff < 0) totalDescent -= altDiff;
        }
        lastAlt = currentAlt;
    }

    // --- 5. LOGIQUE DE FIGEMENT DE POSITION (Stabilité) ---
    const currentlyStationary = sSpdFE === 0;

    if (!currentlyStationary) {
        // En mouvement : on met à jour la position brute
        lat = pos.coords.latitude; 
        lon = pos.coords.longitude;
        alt = pos.coords.altitude;
        isStationary = false;
    } else if (!isStationary) {
        // Vient de s'arrêter : on fige la dernière position connue
        // lat, lon, et alt conservent leurs dernières valeurs de mouvement
        isStationary = true;
    } 
    // Si isStationary est déjà true, lat/lon/alt sont conservés (figés).
    
    // Mise à jour des variables globales pour la prochaine itération
    lPos = pos;
    lastTime = currentTime;
    sTime += dt;
    
    // Mise à jour de l'affichage rapide
    updateDisp(pos, spd3D, fSpd, sSpdFE, dt, accelerationMagnitude);
}

/**
 * Met à jour l'affichage des données
 */
function updateDisp(pos, spd3D, fSpd, sSpdFE, dt, accelerationMagnitude) {
    
    // --- Application du Biais de position (Multipath) ---
    const currentLat = lat;
    const currentLon = lon;
    
    const { correctedLat, correctedLon } = applyMaterialBias(currentLat, currentLon);
    
    // Affichage des données de position
    $('latitude').textContent = `${correctedLat !== null ? correctedLat.toFixed(6) : '--'}`; 
    $('longitude').textContent = `${correctedLon !== null ? correctedLon.toFixed(6) : '--'}`;
    $('altitude').textContent = `${alt !== null ? alt.toFixed(1) : '--'} m`; 
    $('accuracy').textContent = `${pos.coords.accuracy.toFixed(2)} m`;
    $('heading').textContent = `${pos.coords.heading !== null ? pos.coords.heading.toFixed(1) : '--'} °`;

    // Affichage des données cinématiques
    $('spd-raw').textContent = `${spd3D.toFixed(3)}`;
    $('spd-stable').textContent = `${sSpdFE.toFixed(3)}`;
    $('accel-total').textContent = `${accelerationMagnitude.toFixed(3)}`;
    $('time-elapsed').textContent = `${sTime.toFixed(1)}`;
    $('dt-gps').textContent = `${(dt * 1000).toFixed(0)}`;
    
    // Affichage des données de trajet
    const avgSpd = sTime > 0 ? totalDistance / sTime : 0;
    $('dist-total').textContent = `${totalDistance.toFixed(1)} m`;
    $('avg-speed').textContent = `${avgSpd.toFixed(3)} m/s`;
    $('climb-total').textContent = `${totalClimb.toFixed(1)} m`;
    $('descent-total').textContent = `${totalDescent.toFixed(1)} m`;
    
    // Biais Multipath
    $('bias-applied').textContent = MULTIPATH_BIAS[currentMaterialBias].desc;
}

/**
 * Mise à jour lente (pour les données météo et l'ETA)
 */
function slowUpdate() {
    // 1. Correction Atmosphérique (Théorique)
    const P = parseFloat($('press-input').value);
    const T = parseFloat($('temp-input').value);
    const H = parseFloat($('hum-input').value);

    if (!isNaN(P) && !isNaN(T) && !isNaN(H)) {
        const astroSpeed = calculateAtmosphericSpeed(P, T, H);

        $('c-air').textContent = `${(astroSpeed.vitesse / 1e6).toFixed(3)} Mm/s`; 
        $('refraction-index').textContent = `${astroSpeed.n.toFixed(6)}`;
    }
    
    // 2. Calcul de l'ETA (Exemple : Distance figée)
    // Pour une vraie ETA, il faudrait une cible lat/lon et une distance à vol d'oiseau
    const distToTarget = 1000; // Simuler une cible à 1km
    if (totalDistance > 0 && totalDistance < distToTarget) {
        const remainingDist = distToTarget - totalDistance;
        const sSpd = parseFloat($('spd-stable').textContent);
        const etaSeconds = sSpd > 0.1 ? remainingDist / sSpd : Infinity;
        
        if (etaSeconds !== Infinity) {
            const min = Math.floor(etaSeconds / 60);
            const sec = Math.round(etaSeconds % 60);
            $('eta-target').textContent = `${min}m ${sec}s`;
        } else {
            $('eta-target').textContent = '--';
        }
    } else {
        $('eta-target').textContent = '--';
    }
}


// ====================================================================
// --- DÉMARRAGE ET ÉVÉNEMENTS ---
// ====================================================================

document.addEventListener('DOMContentLoaded', () => {
    // 1. Démarrage de la Géolocalisation
    if (navigator.geolocation) {
        navigator.geolocation.watchPosition(
            handleGPS, 
            (error) => { console.error('Erreur GPS:', error); $('accuracy').textContent = 'GPS ERROR'; },
            { enableHighAccuracy: true, timeout: 5000, maximumAge: 0 }
        );
    } else {
        alert("La géolocalisation n'est pas supportée par ce navigateur.");
    }
    
    // 2. Écouteur pour l'Accélération (IMU)
    if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', (event) => {
            const accel = event.accelerationIncludingGravity;
            if (accel) {
                lastAcceleration.x = accel.x || 0;
                lastAcceleration.y = accel.y || 0;
                lastAcceleration.z = accel.z || 0;
                lastAccelTime = Date.now();
            }
        });
    }

    // 3. Événement pour le Biais Multipath Manuel
    const materialSelect = $('material-select');
    const applyBiasBtn = $('apply-bias-btn');

    applyBiasBtn.addEventListener('click', () => {
        currentMaterialBias = materialSelect.value;
        // Forcer la mise à jour de l'affichage pour appliquer immédiatement le biais
        if (lPos) {
            // Recalculer le dernier état figé ou brut avec le nouveau biais
            const spd3D = lPos.coords.speed !== null ? lPos.coords.speed : 0;
            const fSpd = kFilter(spd3D, (lPos.timestamp - lastTime) / 1000, lPos.coords.accuracy * lPos.coords.accuracy);
            const sSpdFE = fSpd < MIN_SPD ? 0 : fSpd;
            updateDisp(lPos, spd3D, fSpd, sSpdFE, (lPos.timestamp - lastTime) / 1000, 
                Math.sqrt(lastAcceleration.x**2 + lastAcceleration.y**2 + lastAcceleration.z**2));
        }
    });

    // 4. Lancement de la mise à jour lente
    setInterval(slowUpdate, 2000); // Mise à jour toutes les 2 secondes
});
