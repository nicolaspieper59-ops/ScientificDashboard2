// kalman_logic.js

// ========================
// PARAMÈTRES DU FILTRE DE KALMAN
// ========================
const KALMAN_Q = 0.0001;
const KALMAN_P_INIT = 1000;

// ========================
// ÉTAT GLOBAL
// ========================
let watchId = null;
let positionPrecedente = null; 
let vitesseMax = 0; // en km/h
let vitessesFiltrees = []; // en km/h
let distanceTotale = 0; // en mètres
let tempsDebut = null;

let accel = { z: 0 }; // accélération Z (m/s²)

// État du Filtre de Kalman (position 1D)
let kalmanState = {
    x: 0,   // Position estimée (distance totale en mètres)
    P: KALMAN_P_INIT, // Erreur de covariance (m²)
    v: 0    // Vitesse estimée (m/s)
};
let lastKalmanTime = 0; 


// ========================
// UTILITAIRES DOM & MATHS
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
  
  positionPrecedente = pos; 
  return { vitesse: dist/dt, distance: dist, dt: dt }; // Vitesse en m/s
}


// ========================
// FILTRE DE KALMAN (1D pour Position/Vitesse)
// ========================
function kalmanFilterUpdate(posGPS, dt, accZ, accGPS) {
    // --- Phase 1: Prédiction ---
    kalmanState.x = kalmanState.x + kalmanState.v * dt;
    kalmanState.v = kalmanState.v + accZ * dt; 
    kalmanState.P = kalmanState.P + KALMAN_Q;

    // --- Phase 2: Correction ---
    const R_current = accGPS * accGPS; 
    const K = kalmanState.P / (kalmanState.P + R_current);

    kalmanState.x = kalmanState.x + K * (posGPS - kalmanState.x);
    kalmanState.P = (1 - K) * kalmanState.P;
    
    return K; 
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
        accuracy: pos.coords.accuracy,
        timestamp: now,
        speed: pos.coords.speed // vitesse native en m/s
    };
    
    // Si la vitesse native est nulle et la précision mauvaise, on ne fait rien
    if (gpsData.speed === 0 && gpsData.accuracy > 50 && vitesseMax === 0) {
        // Cela évite les faux départs et les mises à jour inutiles à l'arrêt
        return;
    }

    // 1. Calculs Bruts pour la Distance Totale
    const { vitesse: v_raw_ms, distance: d_delta } = calculerVitesse3D(gpsData);
    const v_raw_kmh = v_raw_ms * 3.6;
    distanceTotale += d_delta;
    
    
    // --- 2. Préparation et Exécution du Kalman ---
    
    const posGPS_m = distanceTotale; 
    let dt_kalman = 0;
    
    if (lastKalmanTime !== 0) {
        dt_kalman = (now - lastKalmanTime) / 1000;
    } else {
        // Initialisation de l'état du filtre
        kalmanState.x = posGPS_m;
        kalmanState.v = v_raw_ms;
    }
    lastKalmanTime = now;
    
    const K = kalmanFilterUpdate(posGPS_m, dt_kalman, accel.z, gpsData.accuracy);

    // Vitesse Filtrée (m/s) et conversion en km/h
    const v_kalman_ms = kalmanState.v;
    const v_kalman_kmh = v_kalman_ms * 3.6;
    
    
    // --- 3. Mise à jour de l'affichage ---
    
    vitesseMax = Math.max(vitesseMax, v_kalman_kmh);
    
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
            accel.z = e.accelerationIncludingGravity?.z ?? 0; 
        });
    }
}

function demarrerCockpit() {
    if (window.location.protocol !== 'https:' && window.location.hostname !== 'localhost') {
        alert("⚠️ ERREUR : Le GPS et les capteurs nécessitent une connexion sécurisée (HTTPS) ou localhost.");
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
    
    vitesseMax = 0; distanceTotale = 0; positionPrecedente = null;

    watchId = navigator.geolocation.watchPosition(
        miseAJour,
        err => safeSetText('gps',`Erreur GPS (${err.code}): ${err.message}`),
        { enableHighAccuracy:true, maximumAge:0, timeout:10000 }
    );
    
    activerAccelerometre();
}

function arreterCockpit() {
    if (watchId!==null) navigator.geolocation.clearWatch(watchId);
    
    watchId = null; 
    positionPrecedente = null; 
    tempsDebut = null;

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
    
    arreterCockpit(); // Initialisation de l'affichage
});
