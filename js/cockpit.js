// kalman.js

// ========================
// PARAMÈTRES DU FILTRE DE KALMAN
// ========================
// Q (Process Noise): Bruit du modèle d'accélération (Petit)
const KALMAN_Q = 0.0001;
// P (Estimation Error): Erreur initiale de l'état (Position), grande pour faire confiance au premier GPS
const KALMAN_P_INIT = 1000;

// ========================
// ÉTAT GLOBAL
// ========================
let watchId = null;
let positionPrecedente = null; // {latitude, longitude, altitude, timestamp}
let vitesseMax = 0;
let vitessesFiltrees = [];
let distanceTotale = 0; // en mètres
let tempsDebut = null;

let accel = { z: 0 }; // accélération Z (m/s²) pour la correction d'altitude

// État du Filtre de Kalman (pour la position 1D)
let kalmanState = {
    x: 0,   // Position estimée (distance totale en mètres)
    P: KALMAN_P_INIT, // Erreur de covariance (m²)
    v: 0    // Vitesse estimée (m/s)
};
let lastKalmanTime = 0; 


// ========================
// UTILS DOM & MATHS
// ========================
function safeSetText(id, text) {
  const e = document.getElementById(id);
  if (e) e.textContent = text;
}

const R_TERRE = 6371e3; // Rayon de la Terre en mètres

function toRad(degrees) {
    return degrees * Math.PI / 180;
}

function haversine3D(p1, p2) {
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

function calculerVitesse3D(pos) {
  if (!positionPrecedente) {
    positionPrecedente = pos;
    return { vitesse: 0, distance: 0, dt: 0 };
  }
  const dt = (pos.timestamp - positionPrecedente.timestamp)/1000;
  if (dt <= 0) return { vitesse: 0, distance: 0, dt: 0 };

  const dist = haversine3D(positionPrecedente, pos);
  
  // Met à jour la position précédente *après* avoir calculé la distance
  positionPrecedente = pos; 
  return { vitesse: dist/dt, distance: dist, dt: dt }; // Vitesse en m/s
}


// ========================
// FILTRE DE KALMAN (1D pour Position)
// ========================
function kalmanFilterUpdate(posGPS, dt, accZ, accGPS) {
    // --- Phase 1: Prédiction (Estimation de l'état futur) ---
    
    // Prediction de la Position: x_k = x_{k-1} + v_{k-1} * dt
    kalmanState.x = kalmanState.x + kalmanState.v * dt;
    
    // Prediction de la Vitesse: v_k = v_{k-1} + Accélération * dt
    // L'accélération 'accZ' est l'entrée de contrôle du filtre.
    kalmanState.v = kalmanState.v + accZ * dt; 

    // Prediction de la Covariance (incertitude): P_k = P_{k-1} + Q
    kalmanState.P = kalmanState.P + KALMAN_Q;

    
    // --- Phase 2: Correction (Intégration de la mesure GPS) ---
    
    // Erreur R (Bruit de la mesure GPS). R = accuracy^2
    const R_current = accGPS * accGPS; 

    // Calcul du gain de Kalman (K): K = P_k / (P_k + R)
    const K = kalmanState.P / (kalmanState.P + R_current);

    // Mise à jour de la Position (Correction): x_k = x_k + K * (z_k - x_k)
    // z_k est la mesure (posGPS)
    kalmanState.x = kalmanState.x + K * (posGPS - kalmanState.x);
    
    // Mise à jour de la Covariance: P_k = (1 - K) * P_k
    kalmanState.P = (1 - K) * kalmanState.P;
    
    return K; // Retourne le Gain pour l'affichage
}


// ========================
// MISE À JOUR GPS (NAVIGATOR.GEOLOCATION)
// ========================
function miseAJour(pos) {
    const now = pos.timestamp;
    const gpsData = {
        latitude: pos.coords.latitude,
        longitude: pos.coords.longitude,
        altitude: pos.coords.altitude,
        accuracy: pos.coords.accuracy, // Précision GPS (m)
        timestamp: now,
        speed: pos.coords.speed // Vitesse native (m/s)
    };

    // 1. Calculs Bruts pour la Distance Totale
    // On utilise les données brutes pour mettre à jour la distance totale parcourue
    const { vitesse: v_raw_ms, distance: d_delta } = calculerVitesse3D(gpsData);
    const v_raw_kmh = v_raw_ms * 3.6;
    distanceTotale += d_delta;
    
    
    // --- 2. Préparation pour le Kalman ---
    
    const posGPS_m = distanceTotale; // Distance totale = Position 1D de la mesure
    let dt_kalman = 0;
    
    if (lastKalmanTime !== 0) {
        dt_kalman = (now - lastKalmanTime) / 1000;
    } else {
        // Initialisation de l'état du filtre
        kalmanState.x = posGPS_m;
        kalmanState.v = v_raw_ms;
    }
    lastKalmanTime = now;
    
    
    // --- 3. Exécution du Filtre de Kalman ---
    
    // Utilise l'accélération Z pour la prédiction d'altitude/verticalité.
    const K = kalmanFilterUpdate(posGPS_m, dt_kalman, accel.z, gpsData.accuracy);

    // Vitesse Filtrée (m/s) et conversion en km/h
    const v_kalman_kmh = kalmanState.v * 3.6;
    
    
    // --- 4. Mise à jour de l'affichage ---
    
    vitesseMax = Math.max(vitesseMax, v_kalman_kmh);
    vitessesFiltrees.push(v_kalman_kmh);
    if (vitessesFiltrees.length > 60) vitessesFiltrees.shift();
    
    // Affichage des résultats
    safeSetText('vitesse-kalman', `Vitesse Filtrée (Kalman) : ${v_kalman_kmh.toFixed(2)} km/h`);
    safeSetText('vitesse-raw', `Vitesse GPS Brute : ${v_raw_kmh.toFixed(2)} km/h`);
    safeSetText('vitesse-max', `Vitesse max (Filtrée) : ${vitesseMax.toFixed(2)} km/h`);
    safeSetText('distance', `Distance totale : ${(distanceTotale/1000).toFixed(3)} km`);
    safeSetText('temps', `Temps écoulé : ${tempsDebut ? ((now - tempsDebut)/1000).toFixed(2) : '0.00'} s`);

    safeSetText('gps', `GPS : Lat: ${gpsData.latitude.toFixed(6)} | Lon: ${gpsData.longitude.toFixed(6)} | Alt: ${gpsData.altitude?.toFixed(1) ?? '--'} m | Précision: ${gpsData.accuracy?.toFixed(0) ?? '--'} m`);
    safeSetText('kalman-gain', `Gain de Kalman (K) : ${K.toFixed(4)}`);
    safeSetText('kalman-error', `Erreur Estimée (P) : ${kalmanState.P.toFixed(2)} m²`);
    safeSetText('accel-z', `Accélération Z : ${accel.z.toFixed(2)} m/s²`);
}


// ========================
// CAPTEURS & CONTRÔLE
// ========================
function activerAccelerometre() {
    if ('DeviceMotionEvent' in window) {
        window.addEventListener('devicemotion', e => {
            // Utilise Z (vertical) comme entrée de contrôle pour la correction d'altitude/vitesse
            accel.z = e.accelerationIncludingGravity?.z ?? 0; 
        });
    } else {
        safeSetText('accel-z', 'Accélération Z : **NON SUPPORTÉ**');
    }
}

function demarrerCockpit() {
    if (window.location.protocol !== 'https:') {
        alert("⚠️ ERREUR : Le GPS et les capteurs nécessitent une connexion sécurisée (HTTPS).");
        safeSetText('gps', 'GPS : **HTTPS REQUIS**');
        return;
    }
    if (!navigator.geolocation) { safeSetText('gps','GPS non disponible'); return; }
    if (watchId!==null) return;

    tempsDebut = Date.now();
    
    // Réinitialisation de l'état du filtre
    kalmanState.x = 0; 
    kalmanState.P = KALMAN_P_INIT;
    kalmanState.v = 0;
    lastKalmanTime = 0;
    
    vitesseMax = 0; vitessesFiltrees = []; distanceTotale = 0; positionPrecedente = null;

    // Utilisation de navigator.geolocation.watchPosition
    watchId = navigator.geolocation.watchPosition(
        miseAJour,
        err => safeSetText('gps',`Erreur GPS (${err.code}): ${err.message}`),
        { enableHighAccuracy:true, maximumAge:0, timeout:10000 }
    );
    
    activerAccelerometre();
}

function arreterCockpit() {
    if (watchId!==null) navigator.geolocation.clearWatch(watchId);
    
    // Nettoyage des états
    watchId = null; 
    positionPrecedente = null; 
    tempsDebut = null;

    // Réinitialisation de l'affichage aux valeurs par défaut
    safeSetText('temps', 'Temps écoulé : 0.00 s');
    safeSetText('vitesse-kalman','Vitesse Filtrée (Kalman) : -- km/h');
    safeSetText('vitesse-raw','Vitesse GPS Brute : -- km/h');
    safeSetText('vitesse-max','Vitesse max (Filtrée) : 0.00 km/h');
    safeSetText('distance','Distance totale : 0.000 km');
    safeSetText('gps','GPS : Lat: -- | Lon: -- | Alt: -- m | Précision: -- m');
    safeSetText('kalman-gain', 'Gain de Kalman (K) : --');
    safeSetText('kalman-error', 'Erreur Estimée (P) : --');
    safeSetText('accel-z', 'Accélération Z : 0.00 m/s²');
}

function resetVitesseMax() { 
    vitesseMax=0; 
    safeSetText('vitesse-max','Vitesse max (Filtrée) : 0.00 km/h'); 
}

// ========================
// Événements DOM
// ========================
document.addEventListener('DOMContentLoaded',()=>{
    document.getElementById('marche').addEventListener('click',demarrerCockpit);
    document.getElementById('arreter').addEventListener('click',arreterCockpit);
    document.getElementById('reset').addEventListener('click',resetVitesseMax);
    
    // Initialisation de l'affichage
    arreterCockpit(); 
});
