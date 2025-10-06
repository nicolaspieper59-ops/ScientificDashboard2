// kalman.js
// Filtre de Kalman 1D + GPS + acc Z (version propre, autonome)

// ========================
// PARAMÈTRES DU FILTRE DE KALMAN
// ========================
const KALMAN_Q = 0.0001;
const KALMAN_P_INIT = 1000;

// ========================
// ÉTAT GLOBAL
// ========================
let watchId = null;
let positionPrecedente = null; // { latitude, longitude, altitude, timestamp }
let vitesseMax = 0;
let vitessesFiltrees = [];
let distanceTotale = 0; // m
let tempsDebut = null;

let accel = { z: 0 }; // acc Z (m/s²)

let kalmanState = { x: 0, P: KALMAN_P_INIT, v: 0 };
let lastKalmanTime = 0;

// ========================
// UTILITAIRES DOM & MATHS
// ========================
function safeSetText(id, text) {
  const e = document.getElementById(id);
  if (e) e.textContent = text;
}

const R_TERRE = 6371e3;
function toRad(deg) { return deg * Math.PI / 180; }

function haversine3D(p1, p2) {
  const φ1 = toRad(p1.latitude), φ2 = toRad(p2.latitude);
  const Δφ = toRad(p2.latitude - p1.latitude), Δλ = toRad(p2.longitude - p1.longitude);
  const a = Math.sin(Δφ/2)**2 + Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ/2)**2;
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
  const d2D = R_TERRE * c;
  const dz = (p2.altitude ?? 0) - (p1.altitude ?? 0);
  return Math.sqrt(d2D*d2D + dz*dz);
}

function calculerVitesse3D(pos) {
  if (!positionPrecedente) {
    positionPrecedente = pos;
    return { vitesse: 0, distance: 0, dt: 0 };
  }
  const dt = (pos.timestamp - positionPrecedente.timestamp) / 1000;
  if (dt <= 0) return { vitesse: 0, distance: 0, dt: 0 };
  const dist = haversine3D(positionPrecedente, pos);
  positionPrecedente = pos;
  return { vitesse: dist / dt, distance: dist, dt };
}

// ========================
// FILTRE DE KALMAN 1D
// ========================
function kalmanFilterUpdate(posGPS, dt, accZ, accGPS) {
  // Prediction
  kalmanState.x += kalmanState.v * dt;
  kalmanState.v += (accZ || 0) * dt;
  kalmanState.P += KALMAN_Q;

  // Measurement noise R (variance) — R = (accuracy)^2. Use fallback minimal variance
  const R_current = Math.max((accGPS || 10) ** 2, 0.5);

  // Kalman gain
  const K = kalmanState.P / (kalmanState.P + R_current);

  // Correction with measurement posGPS
  kalmanState.x += K * (posGPS - kalmanState.x);
  kalmanState.P *= (1 - K);

  return K;
}

// ========================
// MISE À JOUR GPS
// ========================
function miseAJour(pos) {
  const now = pos.timestamp;
  const gpsData = {
    latitude: pos.coords.latitude,
    longitude: pos.coords.longitude,
    altitude: pos.coords.altitude,
    accuracy: pos.coords.accuracy,
    timestamp: now,
    speed: pos.coords.speed
  };

  // raw (brute) speed & distance
  const { vitesse: v_raw_ms, distance: d_delta } = calculerVitesse3D(gpsData);
  const v_raw_kmh = v_raw_ms * 3.6;
  distanceTotale += d_delta;

  // posGPS_m = cumulative distance (1D proxy)
  const posGPS_m = distanceTotale;
  let dt_kalman = 0;
  if (lastKalmanTime !== 0) dt_kalman = (now - lastKalmanTime) / 1000;
  else { kalmanState.x = posGPS_m; kalmanState.v = v_raw_ms; }
  lastKalmanTime = now;

  // Apply Kalman
  const K = kalmanFilterUpdate(posGPS_m, dt_kalman, accel.z, gpsData.accuracy);
  const v_kalman_kmh = kalmanState.v * 3.6;

  vitesseMax = Math.max(vitesseMax, v_kalman_kmh);
  vitessesFiltrees.push(v_kalman_kmh);
  if (vitessesFiltrees.length > 60) vitessesFiltrees.shift();

  // Update UI
  safeSetText('vitesse-kalman', `Vitesse Filtrée (Kalman) : ${isFinite(v_kalman_kmh) ? v_kalman_kmh.toFixed(2) : '--'} km/h`);
  safeSetText('vitesse-raw', `Vitesse GPS Brute : ${isFinite(v_raw_kmh) ? v_raw_kmh.toFixed(2) : '--'} km/h`);
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
  } else {
    safeSetText('accel-z', 'Accélération Z : **NON SUPPORTÉ**');
  }
}

function demarrerCockpit() {
  // HTTPS check: geolocation & many sensors require secure origin; localhost allowed
  if (window.location.protocol !== 'https:' && window.location.hostname !== 'localhost') {
    alert('⚠️ Le GPS et les capteurs nécessitent HTTPS (ou localhost). Héberge en HTTPS ou utilisez localhost.');
    safeSetText('gps', 'GPS : **HTTPS REQUIS**');
    return;
  }
  if (!navigator.geolocation) { safeSetText('gps', 'GPS non disponible'); return; }
  if (watchId !== null) return;

  tempsDebut = Date.now();
  kalmanState = { x: 0, P: KALMAN_P_INIT, v: 0 };
  lastKalmanTime = 0;
  vitesseMax = 0; vitessesFiltrees = []; distanceTotale = 0; positionPrecedente = null;

  watchId = navigator.geolocation.watchPosition(
    miseAJour,
    err => safeSetText('gps', `Erreur GPS (${err.code}): ${err.message}`),
    { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 }
  );

  activerAccelerometre();
}

function arreterCockpit() {
  if (watchId !== null) navigator.geolocation.clearWatch(watchId);
  watchId = null; positionPrecedente = null; tempsDebut = null;
  safeSetText('temps', 'Temps écoulé : 0.00 s');
  safeSetText('vitesse-kalman', 'Vitesse Filtrée (Kalman) : -- km/h');
  safeSetText('vitesse-raw', 'Vitesse GPS Brute : -- km/h');
  safeSetText('vitesse-max', 'Vitesse max (Filtrée) : 0.00 km/h');
  safeSetText('distance', 'Distance totale : 0.000 km');
  safeSetText('gps', 'GPS : Lat: -- | Lon: -- | Alt: -- m | Précision: -- m');
  safeSetText('kalman-gain', 'Gain de Kalman (K) : --');
  safeSetText('kalman-error', 'Erreur Estimée (P) : --');
  safeSetText('accel-z', 'Accélération Z : 0.00 m/s²');
}

function resetVitesseMax() {
  vitesseMax = 0;
  safeSetText('vitesse-max', 'Vitesse max (Filtrée) : 0.00 km/h');
}

// ========================
// ÉVÉNEMENTS DOM
// ========================
document.addEventListener('DOMContentLoaded', () => {
  document.getElementById('marche').addEventListener('click', demarrerCockpit);
  document.getElementById('arreter').addEventListener('click', arreterCockpit);
  document.getElementById('reset').addEventListener('click', resetVitesseMax);

  // Initial display reset
  arreterCockpit();
});
