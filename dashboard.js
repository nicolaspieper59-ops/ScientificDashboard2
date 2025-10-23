// ====================================================================
// --- CONSTANTES GLOBALES ET INITIALISATION ---
// ====================================================================

const D2R = Math.PI / 180; // Degrés vers Radians
const C_L = 299792458; // Vitesse de la lumière dans le vide (m/s)
const EARTH_RADIUS = 6371000; // Rayon moyen de la Terre en mètres

// **CRITIQUE** : MICRO-PRÉCISION (1 mm/s)
const MIN_SPD = 0.001; // 1 mm/s : Vitesse minimale pour être considéré en mouvement.

// Kalman State: [position, vitesse] (Filtrage 1D de la vitesse)
let kX = 0; // State: Vitesse Estimée
let kP = 1; // Covariance d'Erreur (incertitude de la vitesse estimée)
let kQ = 0.000001; // Bruit de Processus (très faible pour stabilité maximale)

// Position et Temps
let lPos = null;
let lastTime = 0;
let sTime = 0;
let lat = null, lon = null, alt = null;

// Trajet et Dénivelé
let totalDistance = 0;
let lastAlt = null;
let totalClimb = 0;
let totalDescent = 0;

// IMU (Accélération 3D & Orientation)
let lastAcceleration = { x: 0, y: 0, z: 0 };
let currentHeading = 0; // Cap actuel (degré)
let imuVelocity = 0; // Vitesse intégrée par IMU (m/s)
let isIMUSaturated = false; // Indicateur de saturation/erreur IMU (lecture a=0)
const ACCEL_THRESHOLD = 0.5; // Seuil pour détecter un mouvement réel (m/s²)

// Contrôle de Mode
let currentMode = 'HYBRID';

// Corrections de Biais (Multipath)
const MULTIPATH_BIAS = {
    'NONE': { lat_offset_m: 0, lon_offset_m: 0, desc: "0.0m / 0.0m" },
    'METAL': { lat_offset_m: 5.0, lon_offset_m: 5.0, desc: "5.0m / 5.0m" },
    'CONCRETE': { lat_offset_m: 3.5, lon_offset_m: 3.5, desc: "3.5m / 3.5m" },
    'WET_GROUND': { lat_offset_m: 2.0, lon_offset_m: 2.0, desc: "2.0m / 2.0m" },
    'FOLIAGE': { lat_offset_m: 1.0, lon_offset_m: 1.0, desc: "1.0m / 1.0m" }
};
let currentMaterialBias = 'NONE';
let isStationary = false; 

// Constantes pour la correction atmosphérique
const K1 = 77.6; const K2 = 6.45; const K3 = 3.77 * 10**5;


// ====================================================================
// --- FONCTIONS MATHÉMATIQUES ET FILTRE ---
// ====================================================================

function $(id) { return document.getElementById(id); }

/**
 * Calcul de la distance en mètres (Haversine).
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
 * Convertit les mètres en degrés de Lat/Lon pour le biais.
 */
function metersToDegrees(meters, lat) {
    const dLat = meters / (EARTH_RADIUS * D2R);
    const dLon = meters / (EARTH_RADIUS * D2R * Math.cos(lat * D2R));
    return { dLat: dLat, dLon: dLon };
}

/**
 * Applique le décalage Multipath sélectionné.
 */
function applyMaterialBias(rawLat, rawLon) {
    const bias = MULTIPATH_BIAS[currentMaterialBias];
    if (!bias) return { correctedLat: rawLat, correctedLon: rawLon };
    const offsetLatLon = metersToDegrees(bias.lat_offset_m, rawLat);
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
    const N = K1 * (P_dry / T_K) + K2 * (Pw / T_K) + K3 * (Pw / (T_K * T_K));
    const n = 1 + (N * 1e-6);
    const v = C_L / n; 
    return { vitesse: v, n: n, refractivity: N };
}


// ====================================================================
// --- GESTION DES CAPTEURS ET LOGIQUE DE MODE ---
// ====================================================================

/**
 * Calcule la position par navigation à l'estime (Dead Reckoning)
 */
function deadReckoning(dt, accMagnitude) {
    // La vitesse intégrée dérive très rapidement sans correction.
    imuVelocity += accMagnitude * dt;
    imuVelocity = Math.max(0, imuVelocity); 

    const stableImuVelocity = imuVelocity < MIN_SPD ? 0 : imuVelocity;
    
    if (stableImuVelocity > 0 && lat !== null && lon !== null) {
        // Mise à jour de la position par intégration de vitesse et cap
        const dist_delta = stableImuVelocity * dt;
        const bearingRad = currentHeading * D2R;
        
        // Calcul des composantes Nord/Est
        const dist_lat_m = dist_delta * Math.cos(bearingRad);
        const dist_lon_m = dist_delta * Math.sin(bearingRad);
        
        // Conversion de mètres en degrés et mise à jour
        const { dLat } = metersToDegrees(dist_lat_m, lat);
        const { dLat: dLonDeg } = metersToDegrees(dist_lon_m, lat); // Utilisation dLonMeters pour la simplicité
        
        lat += dLat;
        lon += dLonDeg; 

        totalDistance += dist_delta;
    }
    
    return stableImuVelocity;
}

/**
 * Gère les données de position GPS et/ou IMU
 */
function handleGPS(pos) {
    const currentTime = pos.timestamp;
    const dt = (lastTime > 0) ? (currentTime - lastTime) / 1000 : 0;
    
    // 1. Calcul de l'Accélération 3D et gestion de la saturation
    let accelerationMagnitude;

    if (isIMUSaturated) {
        // Saturation : on suppose qu'il y a mouvement (> ACCEL_THRESHOLD) pour ne pas figer la position 
        // basée sur une fausse lecture a=0, et on force le système à se fier au GPS.
        accelerationMagnitude = ACCEL_THRESHOLD + 0.1; 
    } else {
        accelerationMagnitude = Math.sqrt(
            lastAcceleration.x**2 + 
            lastAcceleration.y**2 + 
            lastAcceleration.z**2
        );
    }
    
    let spd3D = 0;
    let sSpdFE = 0;
    let acc = 0;
    let coherencePercent = 0;

    if (currentMode === 'HYBRID') {
        // --- MODE HYBRIDE (GPS + IMU + KALMAN) ---
        
        // **CRITIQUE : FORCER L'ACCURACY À UNE VALEUR MINIMALE (1m)**
        acc = pos.coords.accuracy || 7.0; 
        if (acc < 1.0) {
            acc = 1.0; // Évite R=0, garantit un lissage minimal
        }
        
        spd3D = pos.coords.speed !== null ? pos.coords.speed : 0;

        // 2. Définir le R (Bruit de Mesure) basé sur l'incertitude du GPS et l'IMU
        let kR = acc * acc; 
        
        // Si l'IMU dit que nous sommes très stables.
        if (accelerationMagnitude < ACCEL_THRESHOLD) {
            kR = kR * 5000; // Multiplicateur agressif pour le MIN_SPD = 0.001
        }
        
        // Limites de R
        const R_MIN = 1.0;     
        const R_MAX = 50000.0;
        kR = Math.min(Math.max(kR, R_MIN), R_MAX); 
        
        // 3. Filtrage de Kalman
        const kP_pred = kP + kQ * dt; 
        const kalmanGain = kP_pred / (kP_pred + kR); 
        
        kX = kX + kalmanGain * (spd3D - kX); 
        kP = (1 - kalmanGain) * kP_pred; 

        // Vitesse Stable (Objectif 1mm/s)
        sSpdFE = kX < MIN_SPD ? 0 : kX; 
        
        // 4. Calcul de la Cohérence (basée sur la confiance du filtre kP)
        const KP_MAX_UNCERTAINTY = 0.5;
        coherencePercent = 100 * (1 - Math.min(kP, KP_MAX_UNCERTAINTY) / KP_MAX_UNCERTAINTY);
        coherencePercent = Math.max(0, Math.min(100, coherencePercent));
        
        // 5. Logique de Figement et de Trajet (basée sur le GPS réel)
        const currentlyStationary = sSpdFE === 0;
        if (!currentlyStationary) {
            if (lPos) {
                const dist_delta = haversine(lPos.coords.latitude, lPos.coords.longitude, pos.coords.latitude, pos.coords.longitude);
                totalDistance += dist_delta;
            }
            lat = pos.coords.latitude; 
            lon = pos.coords.longitude;
            alt = pos.coords.altitude;
            isStationary = false;
        } else if (!isStationary) {
            isStationary = true;
        } 
        
    } else {
        // --- MODE IMU SEUL (DEAD RECKONING) ---
        acc = 999.0;
        coherencePercent = 10; // Cohérence minimale due à la dérive

        if (isIMUSaturated) {
            sSpdFE = 0;
            spd3D = 0;
            // On ne met pas à jour la position pour éviter la dérive à cause d'une mauvaise accélération.
        } else {
            // Logique Dead Reckoning
            sSpdFE = deadReckoning(dt, accelerationMagnitude);
            spd3D = sSpdFE;
        }
    }

    // Mise à jour de l'altitude et du dénivelé (indépendant du mode)
    const currentAlt = pos.coords.altitude;
    if (currentAlt !== null && lastAlt !== null && !isNaN(currentAlt)) {
        const altDiff = currentAlt - lastAlt;
        if (altDiff > 0) totalClimb += altDiff;
        else if (altDiff < 0) totalDescent -= altDiff;
    }
    lastAlt = currentAlt;


    // Mise à jour des variables globales
    lPos = pos;
    lastTime = currentTime;
    sTime += dt;
    
    // Mise à jour de l'affichage rapide
    updateDisp(pos, spd3D, sSpdFE, dt, accelerationMagnitude, acc, coherencePercent);
}

/**
 * Met à jour l'affichage des données
 */
function updateDisp(pos, spd3D, sSpdFE, dt, accelerationMagnitude, currentAcc, coherencePercent) {
    const currentLat = lat;
    const currentLon = lon;
    
    const { correctedLat, correctedLon } = applyMaterialBias(currentLat, currentLon);
    
    // Affichage des données de position
    $('current-mode').textContent = currentMode === 'HYBRID' ? 'HYBRIDE' : 'IMU SEUL';
    $('signal-coherence').textContent = `${coherencePercent.toFixed(1)} %`; 
    $('latitude').textContent = `${correctedLat !== null ? correctedLat.toFixed(6) : '--'}`; 
    $('longitude').textContent = `${correctedLon !== null ? correctedLon.toFixed(6) : '--'}`;
    $('altitude').textContent = `${alt !== null ? alt.toFixed(1) : '--'} m`; 
    $('accuracy').textContent = `${currentAcc.toFixed(2)} m`;
    $('heading').textContent = `${currentHeading.toFixed(1)} °`; 
    
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
 * Mise à jour lente (météo et ETA)
 */
function slowUpdate() {
    // Correction Atmosphérique (Théorique)
    const P = parseFloat($('press-input').value);
    const T = parseFloat($('temp-input').value);
    const H = parseFloat($('hum-input').value);

    if (!isNaN(P) && !isNaN(T) && !isNaN(H)) {
        const astroSpeed = calculateAtmosphericSpeed(P, T, H);
        $('c-air').textContent = `${(astroSpeed.vitesse / 1e6).toFixed(3)} Mm/s`; 
        $('refraction-index').textContent = `${astroSpeed.n.toFixed(6)}`;
    }
    
    // Calcul de l'ETA (Exemple)
    const distToTarget = 1000;
    if (totalDistance < distToTarget) {
        const remainingDist = distToTarget - totalDistance;
        const sSpd = parseFloat($('spd-stable').textContent);
        const etaSeconds = sSpd > MIN_SPD ? remainingDist / sSpd : Infinity;
        
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
            (pos) => { 
                // Initialisation de la position pour le mode IMU_ONLY
                if (lat === null) {
                    lat = pos.coords.latitude; 
                    lon = pos.coords.longitude;
                    alt = pos.coords.altitude;
                }
                handleGPS(pos);
            }, 
            (error) => { console.error('Erreur GPS:', error); $('accuracy').textContent = 'GPS ERROR'; },
            { enableHighAccuracy: true, timeout: 5000, maximumAge: 0 }
        );
    } else {
        alert("La géolocalisation n'est pas supportée par ce navigateur.");
    }
    
    // 2. Écouteur pour l'Accélération (IMU) & Détection de saturation
    if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', (event) => {
            const accel = event.accelerationIncludingGravity;
            if (accel) {
                const ax = accel.x || 0;
                const ay = accel.y || 0;
                const az = accel.z || 0;
                
                // Si toutes les lectures sont 0, on flag la saturation (comportement max pression = 0)
                if (ax === 0 && ay === 0 && az === 0) {
                     isIMUSaturated = true;
                     lastAcceleration.x = 0;
                     lastAcceleration.y = 0;
                     lastAcceleration.z = 0;
                } else {
                     isIMUSaturated = false;
                     lastAcceleration.x = ax;
                     lastAcceleration.y = ay;
                     lastAcceleration.z = az;
                }
            }
        });
    }

    // 3. Écouteur pour l'Orientation (Cap / Boussole)
    if (window.DeviceOrientationEvent) {
        window.addEventListener('deviceorientation', (event) => {
            if (event.alpha !== null) {
                currentHeading = 360 - event.alpha; 
            }
        });
    }

    // 4. Événement pour le Changement de Mode
    const modeSelect = $('mode-select');
    modeSelect.addEventListener('change', () => {
        currentMode = modeSelect.value;
        // Réinitialiser l'état du filtre Kalman et l'IMU à 0 lors du changement de mode
        kX = 0; kP = 1; imuVelocity = 0; 
        console.log(`Mode changé en: ${currentMode}`);
    });
    
    // 5. Événement pour le Biais Multipath Manuel
    const materialSelect = $('material-select');
    const applyBiasBtn = $('apply-bias-btn');

    applyBiasBtn.addEventListener('click', () => {
        currentMaterialBias = materialSelect.value;
        if (lPos) handleGPS(lPos); 
    });

    // 6. Lancement de la mise à jour lente
    setInterval(slowUpdate, 2000); 
});
