// kalman_logic.js - ACCÉLÉROMÈTRE ET GYROSCOPE (Meilleure Précision Inertielle)

// ========================
// ÉTAT GLOBAL
// ========================
let watchId = null;
let modeGPSActif = true;

// Capteurs
let accel = { x: 0, y: 0, z: 0 }; // Accélération linéaire (sans gravité, si dispo)
let orientation = { alpha: 0, beta: 0, gamma: 0 }; // Orientation (Gyroscope)

// Estimation Inertielle
let inertialState = { vx: 0, vy: 0, vz: 0, lastTime: 0 };
let vitesseMax = 0; // en km/h
let distanceEstimée = 0; // en mètres


// ========================
// UTILITAIRES DOM & MATHS
// ========================
function safeSetText(id, text) {
  const e = document.getElementById(id);
  if (e) e.textContent = text;
}
function toRad(degrees) { return degrees * Math.PI / 180; }
const G = 9.81; // Gravité standard


// ========================
// LOGIQUE DE DÉTECTION ET DE CORRECTION
// ========================

// Tente de soustraire la gravité pour obtenir l'accélération linéaire pure
function getLinearAcceleration(accelInclGravity) {
    if (!accelInclGravity) return { x: 0, y: 0, z: 0 };

    // Si le navigateur fournit déjà l'accélération sans gravité
    if ('acceleration' in accelInclGravity && accelInclGravity.acceleration) {
        return { 
            x: accelInclGravity.acceleration.x || 0,
            y: accelInclGravity.acceleration.y || 0,
            z: accelInclGravity.acceleration.z || 0
        };
    }
    
    // Si nous avons l'orientation (gyro), nous pouvons tenter la soustraction (méthode simplifiée)
    // NOTE: Ceci nécessite une calibration précise et est très imprécis sur de longues périodes.
    const radBeta = toRad(orientation.beta);
    const radGamma = toRad(orientation.gamma);
    
    // Projection simplifiée de la gravité sur les axes du téléphone (x et y)
    // C'est la partie la plus difficile sans un filtre de fusion comme un Madgwick.
    const gravX = G * Math.sin(radGamma);
    const gravY = G * Math.sin(radBeta); 
    // La gravité en Z est souvent la plus stable pour l'altitude, mais nous nous concentrons sur le plan horizontal.

    return {
        x: accelInclGravity.accelerationIncludingGravity.x - gravX,
        y: accelInclGravity.accelerationIncludingGravity.y - gravY,
        z: accelInclGravity.accelerationIncludingGravity.z - G // très simpliste pour Z
    };
}


// Estimation de la position et de la vitesse par intégration
function estimateInertialMovement() {
    const now = performance.now();
    
    if (inertialState.lastTime === 0) {
        inertialState.lastTime = now;
        return 0;
    }

    const dt = (now - inertialState.lastTime) / 1000;
    inertialState.lastTime = now;

    // 1. Obtenir l'accélération sans gravité
    const ax = accel.x;
    const ay = accel.y;
    
    // 2. Seuil de bruit (filtration)
    const ACC_THRESHOLD = 0.08; // m/s² (toute accélération inférieure est ignorée)
    
    const horizAcc = Math.sqrt(ax * ax + ay * ay);
    const effectiveAcc = Math.max(0, horizAcc - ACC_THRESHOLD);

    // 3. Mise à jour de la vitesse (Intégration)
    // La vitesse intégrale est utilisée comme notre Vitesse Filtrée (vitesse inertielle).
    const prevSpeed = Math.sqrt(inertialState.vx**2 + inertialState.vy**2);
    const newSpeed = prevSpeed + effectiveAcc * dt;

    // 4. Damping (pour contrer la dérive)
    // La vitesse doit décroître si aucune accélération significative n'est détectée.
    const DAMPING_FACTOR = 0.999;
    inertialState.vx = effectiveAcc > 0 ? newSpeed : newSpeed * DAMPING_FACTOR;
    inertialState.vy = 0; // Simplification pour ne garder qu'une vitesse 1D

    // 5. Mise à jour de la distance
    distanceEstimée += newSpeed * dt;

    return Math.abs(newSpeed); // Vitesse estimée en m/s
}


// ========================
// MISE À JOUR PRINCIPALE (Capteurs uniquement)
// ========================
function miseAJourInertielle(now) {
    
    if (!modeGPSActif) {
        // --- MODE SANS GPS (Inertiel) ---
        const v_inertial_ms = estimateInertialMovement();
        const v_inertial_kmh = v_inertial_ms * 3.6;

        // Mise à jour des statistiques
        vitesseMax = Math.max(vitesseMax, v_inertial_kmh);
        
        // Affichage
        safeSetText('vitesse-kalman', `Vitesse Filtrée (Inertiel) : ${v_inertial_kmh.toFixed(2)} km/h`);
        safeSetText('vitesse-raw', `Vitesse GPS Brute : -- km/h`);
        safeSetText('vitesse-max', `Vitesse max : ${vitesseMax.toFixed(2)} km/h`);
        safeSetText('distance', `Distance totale : ${(distanceEstimée/1000).toFixed(3)} km`);
        safeSetText('temps', `Temps écoulé : ${tempsDebut ? ((now - tempsDebut)/1000).toFixed(2) : '0.00'} s`);
        safeSetText('gps', `GPS : **INACTIF** | Gyro: ${orientation.alpha.toFixed(1)}°`);
        safeSetText('accel-z', `Accélération Linéaire X/Y/Z : ${accel.x.toFixed(2)}, ${accel.y.toFixed(2)}, ${accel.z.toFixed(2)} m/s²`);
        safeSetText('kalman-gain', `Gain de Kalman (K) : N/A`);
        safeSetText('kalman-error', `Erreur Estimée (P) : N/A`);
    } 
    // NOTE: Le code pour le mode GPS a été retiré pour se concentrer uniquement sur la demande (sans GPS).
}


// ========================
// CAPTEURS & CONTRÔLE
// ========================
function activerCapteurs() {
    // 1. Accéléromètre et Accélération Linéaire
    if ('DeviceMotionEvent' in window) {
        window.addEventListener('devicemotion', e => {
            const linearAcc = getLinearAcceleration(e);
            accel.x = linearAcc.x;
            accel.y = linearAcc.y;
            accel.z = linearAcc.z;
        });
    }

    // 2. Gyroscope (Orientation pour correction de gravité)
    if ('DeviceOrientationEvent' in window) {
        window.addEventListener('deviceorientation', e => {
            orientation.alpha = e.alpha || 0;
            orientation.beta = e.beta || 0;
            orientation.gamma = e.gamma || 0;
        });
    }
}

function demarrerCockpitSansGPS() {
    if (window.location.protocol !== 'https:' && window.location.hostname !== 'localhost') {
        alert("⚠️ Le mode inertiel nécessite l'accès aux capteurs et donc HTTPS.");
        return;
    }
    
    modeGPSActif = false;
    // Réinitialisation de l'état
    inertialState = { vx: 0, vy: 0, vz: 0, lastTime: 0 };
    vitesseMax = 0; distanceEstimée = 0;
    
    activerCapteurs(); // Active l'écoute des capteurs
    
    // Le 'watchId' est utilisé ici pour l'intervalle de temps, pas pour le GPS
    if (watchId) clearInterval(watchId);
    tempsDebut = Date.now();
    
    console.log("Démarrage du mode inertiel (100ms intervalle)...");
    
    // Utilise setInterval pour forcer les mises à jour régulières
    watchId = setInterval(() => miseAJourInertielle(Date.now()), 100); 

    safeSetText('avertissement', '⚠️ Mode Inertiel Actif. La **dérive** sera rapide. Agitez le téléphone!');
    safeSetText('vitesse-kalman', `Vitesse Filtrée (Inertiel) : 0.00 km/h`);
}


function arreterCockpit() {
    if (watchId!==null) clearInterval(watchId); // Arrête l'intervalle en mode inertiel
    
    watchId = null; 
    
    // Réinitialisation de l'affichage
    safeSetText('temps', 'Temps écoulé : 0.00 s');
    safeSetText('vitesse-kalman','Vitesse Filtrée (Kalman) : -- km/h');
    safeSetText('vitesse-raw','Vitesse GPS Brute : -- km/h');
    safeSetText('vitesse-max','Vitesse max (Filtrée) : 0.00 km/h');
    safeSetText('distance','Distance totale : 0.000 km');
    safeSetText('gps','GPS : Lat: -- | Lon: -- | Alt: -- m | Précision: -- m');
    safeSetText('kalman-gain', 'Gain de Kalman (K) : --');
    safeSetText('kalman-error', 'Erreur Estimée (P) : --');
    safeSetText('accel-z', 'Accélération Z : 0.00 m/s²');
    safeSetText('avertissement', '⚠️ Géolocalisation et Capteurs nécessitent **HTTPS**');
}

function resetVitesseMax() { 
    vitesseMax=0; 
    safeSetText('vitesse-max','Vitesse max (Filtrée) : 0.00 km/h'); 
}

// ========================
// Événements DOM
// ========================
document.addEventListener('DOMContentLoaded',()=>{
    const marcheBtn = document.getElementById('marche');
    if(marcheBtn) {
        // Le bouton par défaut démarre le mode inertiel
        marcheBtn.textContent = '▶️ Démarrer (Mode Inertiel)';
        marcheBtn.removeEventListener('click', demarrerCockpit); // Retire l'ancien listener GPS
        marcheBtn.addEventListener('click', demarrerCockpitSansGPS);
    }
    
    document.getElementById('arreter').addEventListener('click',arreterCockpit);
    document.getElementById('reset').addEventListener('click',resetVitesseMax);
    
    arreterCockpit();
});
