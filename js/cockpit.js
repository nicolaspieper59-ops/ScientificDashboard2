// fusion_logic.js - COMPLET (GPS + Capteurs + Connexion)

// ========================
// PARAMÈTRES ET CONSTANTES
// ========================
// Paramètres de Kalman pour la fusion GPS/Accel
const KALMAN_Q = 0.0001;
const KALMAN_P_INIT = 1000;

// Paramètres ZUPT pour le mode Inertiel
const ACC_THRESHOLD = 0.15; // Seuil d'accélération pour le mouvement (m/s²)
const ZUPT_TIME_THRESHOLD = 200; // Temps minimum où l'accélération est sous le seuil pour forcer la vitesse à zéro (ms)
const DAMPING_FACTOR = 0.990; // Amortissement de la vitesse en phase de décélération

// Géographie
const R_TERRE = 6371e3; // Rayon de la Terre en mètres
const G = 9.81; // Gravité standard

// ========================
// ÉTAT GLOBAL
// ========================
let watchId = null;
let positionPrecedente = null; 
let vitesseMax = 0; // en km/h
let distanceTotale = 0; // en mètres
let tempsDebut = null;

let modeActif = 'INACTIF'; // 'GPS_KALMAN' ou 'INERTIAL_ZUPT'

// Capteurs
let accel = { x: 0, y: 0, z: 0 }; // Accélération linéaire (m/s²)
let orientation = { alpha: 0, beta: 0, gamma: 0 }; // Orientation (Gyroscope)

// État de Kalman (Position 1D) pour la fusion GPS
let kalmanState = { x: 0, P: KALMAN_P_INIT, v: 0 };
let lastKalmanTime = 0; 

// État Inertiel pour le mode ZUPT
let inertialState = { vx: 0, vy: 0, lastTime: 0, lastMovementTime: 0 };

// ========================
// UTILITAIRES & MATHS
// ========================
function safeSetText(id, text) {
  const e = document.getElementById(id);
  if (e) e.textContent = text;
}
function toRad(degrees) { return degrees * Math.PI / 180; }

function haversine3D(p1, p2) {
  // Calcul de la distance 3D Haversine (utilisé par le GPS)
  const φ1 = toRad(p1.latitude);
  const φ2 = toRad(p2.latitude);
  const Δφ = toRad(p2.latitude - p1.latitude);
  const Δλ = toRad(p2.longitude - p1.longitude);
  const a = Math.sin(Δφ/2)**2 + Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2;
  const c = 2*Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
  const d2D = R_TERRE * c;
  const dz = (p2.altitude ?? 0) - (p1.altitude ?? 0);
  return Math.sqrt(d2D*d2D + dz*dz);
}

function getLinearAcceleration(e) {
    // Tente d'obtenir l'accélération sans gravité
    if ('acceleration' in e && e.acceleration) {
        return { 
            x: e.acceleration.x || 0,
            y: e.acceleration.y || 0,
            z: e.acceleration.z || 0
        };
    }
    // Si non disponible, retourne l'accélération avec gravité
    return {
        x: e.accelerationIncludingGravity?.x || 0,
        y: e.accelerationIncludingGravity?.y || 0,
        z: e.accelerationIncludingGravity?.z || 0
    };
}


// ========================
// LOGIQUE KALMAN (FUSION GPS/ACCEL)
// ========================
function kalmanFilterUpdate(posGPS_m, dt_kalman, accZ, accGPS) {
    // Phase 1: Prédiction
    kalmanState.x = kalmanState.x + kalmanState.v * dt_kalman;
    kalmanState.v = kalmanState.v + accZ * dt_kalman; 
    kalmanState.P = kalmanState.P + KALMAN_Q;

    // Phase 2: Correction (Mesure GPS)
    const R_current = accGPS * accGPS; 
    const K = kalmanState.P / (kalmanState.P + R_current);

    kalmanState.x = kalmanState.x + K * (posGPS_m - kalmanState.x);
    kalmanState.P = (1 - K) * kalmanState.P;
    
    return { K: K, v_ms: kalmanState.v }; 
}

// ========================
// LOGIQUE INERTIELLE (ZUPT)
// ========================
function estimateInertialMovement() {
    const now = performance.now();
    
    if (inertialState.lastTime === 0) {
        inertialState.lastTime = now;
        return { v_ms: 0, etat: 'INITIALISATION' };
    }

    const dt = (now - inertialState.lastTime) / 1000;
    inertialState.lastTime = now;
    
    const currentSpeed = Math.sqrt(inertialState.vx**2 + inertialState.vy**2);

    // 1. Détection de l'Accélération
    const ax = accel.x;
    const ay = accel.y;
    const horizAcc = Math.sqrt(ax * ax + ay * ay);
    const isMoving = horizAcc >= ACC_THRESHOLD;

    if (isMoving) {
        // Mouvement détecté: Intégrer la vitesse
        inertialState.lastMovementTime = now;
        const newSpeed = currentSpeed + horizAcc * dt;
        
        inertialState.vx = newSpeed; 
        inertialState.vy = 0; 
        distanceTotale += newSpeed * dt;
        return { v_ms: Math.abs(inertialState.vx), etat: 'MOUVEMENT' };
        
    } else {
        // 2. Mouvement NON détecté : ZUPT
        const timeSinceLastMove = now - inertialState.lastMovementTime;
        
        if (timeSinceLastMove > ZUPT_TIME_THRESHOLD) {
            // ZUPT: FORCER VITESSE ZÉRO
            inertialState.vx = 0;
            inertialState.vy = 0;
            return { v_ms: 0, etat: 'IMMOBILE (ZUPT)' };
        } else {
            // Phase de décélération (amortissement)
            inertialState.vx *= DAMPING_FACTOR;
            inertialState.vy *= DAMPING_FACTOR;
            const dampedSpeed = Math.sqrt(inertialState.vx**2 + inertialState.vy**2);
            distanceTotale += dampedSpeed * dt;
            return { v_ms: dampedSpeed, etat: 'DÉCÉLÉRATION' };
        }
    }
}


// ========================
// MISE À JOUR GPS (MODE GPS_KALMAN)
// ========================
function miseAJourGPS(pos) {
    if (modeActif !== 'GPS_KALMAN') return;
    
    const now = pos.timestamp;
    const gpsData = {
        latitude: pos.coords.latitude, longitude: pos.coords.longitude,
        altitude: pos.coords.altitude, accuracy: pos.coords.accuracy,
        timestamp: now, speed: pos.coords.speed 
    };
    
    // 1. Calculs Bruts GPS
    const v_raw_ms = gpsData.speed || 0; 
    const v_raw_kmh = v_raw_ms * 3.6;

    let d_delta = 0;
    if (positionPrecedente) {
        d_delta = haversine3D(positionPrecedente, gpsData);
    }
    distanceTotale += d_delta;
    positionPrecedente = gpsData;
    
    
    // 2. Exécution du Kalman (Fusion)
    const posGPS_m = distanceTotale; 
    let dt_kalman = (lastKalmanTime !== 0) ? (now - lastKalmanTime) / 1000 : 0;
    lastKalmanTime = now;
    
    const { K, v_ms: v_kalman_ms } = kalmanFilterUpdate(posGPS_m, dt_kalman, accel.z, gpsData.accuracy);
    const v_kalman_kmh = v_kalman_ms * 3.6;
    
    
    // 3. Affichage
    vitesseMax = Math.max(vitesseMax, v_kalman_kmh);
    safeSetText('vitesse-filtree', `Vitesse Estimée : ${v_kalman_kmh.toFixed(2)} km/h`);
    safeSetText('vitesse-raw', `Vitesse GPS Brute : ${v_raw_kmh.toFixed(2)} km/h`);
    safeSetText('gps', `GPS : Lat: ${gpsData.latitude.toFixed(6)} | Lon: ${gpsData.longitude.toFixed(6)} | Alt: ${gpsData.altitude?.toFixed(1) ?? '--'} m | Précision: ${gpsData.accuracy?.toFixed(0) ?? '--'} m`);
    safeSetText('kalman-etat', `Kalman: Gain K=${K.toFixed(4)}`);
}


// ========================
// MISE À JOUR INERTIELLE (MODE INERTIAL_ZUPT)
// ========================
function miseAJourInertielle() {
    if (modeActif !== 'INERTIAL_ZUPT') return;
    
    const now = Date.now();
    const { v_ms: v_inertial_ms, etat: zuptEtat } = estimateInertialMovement();
    const v_inertial_kmh = v_inertial_ms * 3.6;

    // Affichage
    vitesseMax = Math.max(vitesseMax, v_inertial_kmh);
    safeSetText('vitesse-filtree', `Vitesse Estimée : ${v_inertial_kmh.toFixed(2)} km/h`);
    safeSetText('vitesse-raw', `Acc. Horiz. Brute : ${Math.sqrt(accel.x**2 + accel.y**2).toFixed(2)} m/s²`);
    safeSetText('gps', `GPS : **INACTIF** | Précision Inertielle très limitée.`);
    safeSetText('kalman-etat', `ZUPT: ${zuptEtat}`);
}

// ========================
// MISE À JOUR UI COMMUNE (À appeler par les deux modes)
// ========================
function miseAJourUI(now) {
    if (modeActif === 'INACTIF') return;
    
    // Mise à jour de l'affichage commun
    safeSetText('mode-actuel', `Mode actuel : ${modeActif.replace('_', '/')}`);
    safeSetText('vitesse-max', `Vitesse max : ${vitesseMax.toFixed(2)} km/h`);
    safeSetText('distance', `Distance totale : ${(distanceTotale/1000).toFixed(3)} km`);
    safeSetText('temps', `Temps écoulé : ${tempsDebut ? ((now - tempsDebut)/1000).toFixed(2) : '0.00'} s`);
    safeSetText('accel-data', `Accélération Linéaire (X,Y,Z) : ${accel.x.toFixed(2)}, ${accel.y.toFixed(2)}, ${accel.z.toFixed(2)} m/s²`);
    
    // Statut de la connexion
    safeSetText('connexion-etat', `Statut Réseau : ${navigator.onLine ? 'EN LIGNE (OK)' : 'HORS LIGNE (Peut Affecter l\'A-GPS)'}`);
}


// ========================
// CONTRÔLE ET INITIALISATION
// ========================
function activerCapteurs() {
    if ('DeviceMotionEvent' in window) {
        window.addEventListener('devicemotion', e => {
            const linearAcc = getLinearAcceleration(e);
            accel.x = linearAcc.x;
            accel.y = linearAcc.y;
            accel.z = linearAcc.z;
        });
    }
    if ('DeviceOrientationEvent' in window) {
        window.addEventListener('deviceorientation', e => {
            orientation.alpha = e.alpha || 0;
            orientation.beta = e.beta || 0;
            orientation.gamma = e.gamma || 0;
        });
    }
}

function demarrerCockpit(mode) {
    if (window.location.protocol !== 'https:' && window.location.hostname !== 'localhost') {
        alert("⚠️ Le GPS et les capteurs nécessitent une connexion sécurisée (HTTPS) ou localhost.");
        return;
    }
    if (watchId !== null) arreterCockpit(); // Arrête l'ancien mode
    
    modeActif = mode;
    tempsDebut = Date.now();
    
    // Réinitialisation des états
    vitesseMax = 0; distanceTotale = 0; positionPrecedente = null;
    kalmanState = { x: 0, P: KALMAN_P_INIT, v: 0 };
    inertialState = { vx: 0, vy: 0, lastTime: 0, lastMovementTime: 0 };
    lastKalmanTime = 0;
    
    activerCapteurs();
    
    if (mode === 'GPS_KALMAN') {
        if (!navigator.geolocation) { safeSetText('gps','GPS non disponible'); return; }
        
        safeSetText('avertissement', 'Mode GPS/KALMAN: Acquisition de la position...');
        watchId = navigator.geolocation.watchPosition(
            (pos) => {
                miseAJourGPS(pos);
                miseAJourUI(pos.timestamp);
            },
            err => {
                 console.error(`Erreur GPS: Code ${err.code} - ${err.message}`);
                 safeSetText('gps',`Erreur GPS (${err.code}): ${err.message}.`);
            },
            // Timeout généreux pour le premier fix
            { enableHighAccuracy:true, maximumAge:0, timeout:30000 } 
        );
    } else if (mode === 'INERTIAL_ZUPT') {
        safeSetText('avertissement', 'Mode INERTIEL/ZUPT: La dérive est inévitable sur le long terme.');
        // Utilise setInterval pour forcer les mises à jour régulières des capteurs
        watchId = setInterval(() => {
            const now = Date.now();
            miseAJourInertielle();
            miseAJourUI(now);
        }, 50); // Mises à jour rapides pour la dynamique inertielle
    }
}


function arreterCockpit() {
    if (watchId!==null) {
        if (modeActif === 'GPS_KALMAN' && navigator.geolocation) {
             navigator.geolocation.clearWatch(watchId);
        } else if (modeActif === 'INERTIAL_ZUPT') {
            clearInterval(watchId);
        }
    }
    
    watchId = null; 
    modeActif = 'INACTIF';
    
    // Réinitialisation de l'affichage
    safeSetText('mode-actuel', 'Mode actuel : INACTIF');
    safeSetText('temps', 'Temps écoulé : 0.00 s');
    safeSetText('vitesse-filtree','Vitesse Estimée : -- km/h');
    safeSetText('vitesse-raw','Vitesse GPS/Inertiel Brute : -- km/h');
    safeSetText('vitesse-max','Vitesse max : 0.00 km/h');
    safeSetText('distance','Distance totale : 0.000 km');
    safeSetText('gps','GPS : Lat: -- | Lon: -- | Alt: -- m | Précision: -- m');
    safeSetText('accel-data', 'Accélération Linéaire (X,Y,Z) : 0.00, 0.00, 0.00 m/s²');
    safeSetText('kalman-etat', 'Kalman/ZUPT État : --');
    safeSetText('connexion-etat', 'Statut Réseau : --');
    safeSetText('avertissement', '⚠️ Géolocalisation et Capteurs nécessitent **HTTPS**');
}

function resetVitesseMax() { 
    vitesseMax=0; 
    safeSetText('vitesse-max','Vitesse max : 0.00 km/h'); 
}

// ========================
// Événements DOM
// ========================
document.addEventListener('DOMContentLoaded',()=>{
    document.getElementById('marche').addEventListener('click', () => demarrerCockpit('GPS_KALMAN'));
    document.getElementById('marche-inertiel').addEventListener('click', () => demarrerCockpit('INERTIAL_ZUPT'));
    document.getElementById('arreter').addEventListener('click',arreterCockpit);
    document.getElementById('reset').addEventListener('click',resetVitesseMax);
    
    arreterCockpit();
});
