// cockpit.js
// Cockpit Cosmique 3D — vitesse brute 3D + Kalman (position cumulée) + mode souterrain + détection capteurs

// ---------- CONFIG (ajustables) ----------
const DT_MIN = 0.25;          // ignore updates GPS plus rapprochés que ça (s)
const ACCURACY_THRESHOLD = 20; // si accuracy > threshold (m) => mode souterrain/inertiel
const KALMAN_Q = 0.0001;
const KALMAN_P_INIT = 1000;
const VITESSE_MIN_OR_ZERO = 0.15; // en m/s : en dessous on considère immobile

// ---------- ETAT GLOBAL ----------
let watchId = null;
let positionPrecedente = null; // {latitude, longitude, altitude, timestamp}
let distanceTotale = 0; // en mètres
let tempsDebut = null;

let accel = { x:0, y:0, z:0 }; // accels
let gyro = { alpha:0, beta:0, gamma:0 };
let magneto = { x:0, y:0, z:0 };

let capteursDisponibles = {
  gps: false,
  accelerometre: false,
  gyroscope: false,
  magnetometre: false
};

// Kalman 1D sur la position cumulative (m)
let kalmanState = { x: 0, P: KALMAN_P_INIT };
let lastKalmanX = 0;
let lastKalmanTime = 0;

let vitesseMax = 0;
let vitessesFiltrees = [];

// ---------- UTILITAIRES DOM ----------
function $id(id){ return document.getElementById(id); }
function safeSetText(id, text){ const e = $id(id); if(e) e.textContent = text; }
function setCapteurIcon(id, ok){ const el = $id(id); if(!el) return; el.textContent = ok ? '✅' : '❌'; el.className = ok ? 'ok' : 'nok'; }

// ---------- GÉO / MATH ----------
const R_TERRE = 6371e3;
function toRad(deg){ return deg * Math.PI / 180; }
function haversine3D(p1, p2){
  const φ1 = toRad(p1.latitude), φ2 = toRad(p2.latitude);
  const Δφ = toRad(p2.latitude - p1.latitude), Δλ = toRad(p2.longitude - p1.longitude);
  const a = Math.sin(Δφ/2)**2 + Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2;
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
  const d2D = R_TERRE * c;
  const dz = ( (p2.altitude ?? 0) - (p1.altitude ?? 0) );
  return Math.sqrt(d2D*d2D + dz*dz);
}

// ---------- KALMAN 1D (position cumulative) ----------
function kalmanUpdate(posGPS_m, R_measure){
  // Prediction: here trivial (no dynamic model on x besides Q)
  kalmanState.P += KALMAN_Q;
  // Measurement update
  const Rvar = Math.max((R_measure || 10), 0.5) ** 2; // variance from accuracy (m^2)
  const K = kalmanState.P / (kalmanState.P + Rvar);
  kalmanState.x = kalmanState.x + K * (posGPS_m - kalmanState.x);
  kalmanState.P = (1 - K) * kalmanState.P;
  return K;
}

// ---------- CALCULS VITESSE BRUTE 3D ----------
function processGPSPosition(pos){
  const now = pos.timestamp;
  const gps = {
    latitude: pos.coords.latitude,
    longitude: pos.coords.longitude,
    altitude: pos.coords.altitude ?? 0,
    accuracy: pos.coords.accuracy ?? 999,
    timestamp: now
  };

  capteursDisponibles.gps = true;

  // Ignore updates trop rapprochés (contrôle fréquence)
  if(positionPrecedente){
    let dt = (now - positionPrecedente.timestamp)/1000;
    if(dt < DT_MIN){
      // On ignore mais on met à jour GPS display if needed
      safeSetText('gps', `Lat: ${gps.latitude.toFixed(6)} | Lon: ${gps.longitude.toFixed(6)} | Alt: ${gps.altitude.toFixed(1)} m | Acc: ${gps.accuracy.toFixed(1)} m (update ignored dt=${dt.toFixed(3)}s)`);
      return;
    }
  }

  // Calcul brut distance & vitesse
  const { vitesse_ms, dist_m, dt } = (() => {
    if(!positionPrecedente) return { vitesse_ms: 0, dist_m: 0, dt: 0 };
    const dtLocal = (now - positionPrecedente.timestamp)/1000;
    if(dtLocal <= 0) return { vitesse_ms: 0, dist_m: 0, dt: 0 };
    const d = haversine3D(positionPrecedente, gps);
    const v = d / dtLocal;
    return { vitesse_ms: v, dist_m: d, dt: dtLocal };
  })();

  // Update cumulative distance (use raw distance for cumulative proxy)
  distanceTotale += dist_m;

  // Mode souterrain si accuracy mauvaise
  const underground = gps.accuracy > ACCURACY_THRESHOLD;

  // Kalman update: use cumulative distance as measurement (posGPS_m)
  const posGPS_m = distanceTotale;
  if(lastKalmanTime === 0){
    // init
    kalmanState.x = posGPS_m;
    lastKalmanX = kalmanState.x;
    lastKalmanTime = now;
  }
  const dtKal = (now - lastKalmanTime)/1000 || DT_MIN;
  // R measure: if underground -> large R to trust GPS less
  const R_measure = underground ? gps.accuracy * 3 : gps.accuracy; // tuneable
  const K = kalmanUpdate(posGPS_m, R_measure);

  // filtered speed computed from delta of kalman x
  const v_filtered_ms = (kalmanState.x - lastKalmanX) / dtKal;
  const v_filtered_kmh = v_filtered_ms * 3.6;
  lastKalmanX = kalmanState.x;
  lastKalmanTime = now;

  // raw speed (brute) in km/h
  const v_raw_kmh = vitesse_ms * 3.6;

  // threshold tiny speeds -> zero
  const raw_final_kmh = (v_raw_kmh < VITESSE_MIN_OR_ZERO*3.6) ? 0 : v_raw_kmh;
  const filt_final_kmh = (Math.abs(v_filtered_kmh) < VITESSE_MIN_OR_ZERO*3.6) ? 0 : v_filtered_kmh;

  // store record
  vitesseMax = Math.max(vitesseMax, Math.max(raw_final_kmh, filt_final_kmh));
  vitessesFiltrees.push(filt_final_kmh);
  if(vitessesFiltrees.length > 200) vitessesFiltrees.shift();

  // display
  safeSetText('vitesse-raw', `Vitesse brute 3D : ${raw_final_kmh.toFixed(2)} km/h`);
  safeSetText('vitesse-kalman', `Vitesse filtrée (Kalman) : ${filt_final_kmh.toFixed(2)} km/h`);
  safeSetText('vitesse-max', `Record : ${vitesseMax.toFixed(2)} km/h`);
  safeSetText('distance', `Distance totale : ${(distanceTotale/1000).toFixed(3)} km`);
  safeSetText('gps', `Lat: ${gps.latitude.toFixed(6)} | Lon: ${gps.longitude.toFixed(6)} | Alt: ${gps.altitude.toFixed(1)} m | Acc: ${gps.accuracy.toFixed(1)} m`);
  safeSetText('kalman-gain', `K : ${K.toFixed(4)}`);
  safeSetText('kalman-error', `P : ${kalmanState.P.toFixed(2)} m²`);
  safeSetText('accel-z', `Acc Z : ${accel.z.toFixed(2)} m/s²`);
  safeSetText('temps', `Temps écoulé : ${tempsDebut ? ((now - tempsDebut)/1000).toFixed(2) : '0.00'} s`);
  safeSetText('mode', `Mode : ${underground ? 'Souterrain / Inertiel' : 'GNSS normal'}`);

  // update capteur icons externally via interval
  updateCapteursIcons();
  // update previous pos for next raw calc
  positionPrecedente = gps;
}

// ---------- SENSORS Activation ----------
function activateAccelerometer(){
  if('DeviceMotionEvent' in window){
    capteursDisponibles.accelerometre = true;
    window.addEventListener('devicemotion', e => {
      // use accelerationIncludingGravity to detect vertical changes quickly;
      // subtract gravity if desired or apply filter later
      accel.x = e.accelerationIncludingGravity?.x ?? 0;
      accel.y = e.accelerationIncludingGravity?.y ?? 0;
      accel.z = e.accelerationIncludingGravity?.z ?? 0;
    });
  } else {
    capteursDisponibles.accelerometre = false;
  }
}

function activateGyroscope(){
  // use DeviceOrientationEvent as fallback; Gyroscope Sensor API is not supported everywhere
  if('Gyroscope' in window){
    try{
      const g = new Gyroscope({frequency:60});
      g.addEventListener('reading', () => {
        capteursDisponibles.gyroscope = true;
        gyro.x = g.x; gyro.y = g.y; gyro.z = g.z;
      });
      g.start();
      capteursDisponibles.gyroscope = true;
    } catch(err){
      capteursDisponibles.gyroscope = false;
    }
  } else if('DeviceOrientationEvent' in window){
    capteursDisponibles.gyroscope = true;
    window.addEventListener('deviceorientation', ev => {
      gyro.alpha = ev.alpha || 0; gyro.beta = ev.beta || 0; gyro.gamma = ev.gamma || 0;
    });
  } else capteursDisponibles.gyroscope = false;
}

function activateMagnetometer(){
  if('Magnetometer' in window){
    try{
      const m = new Magnetometer({frequency:10});
      m.addEventListener('reading', () => {
        capteursDisponibles.magnetometre = true;
        magneto.x = m.x; magneto.y = m.y; magneto.z = m.z;
      });
      m.start();
    } catch(e){ capteursDisponibles.magnetometre = false; }
  } else capteursDisponibles.magnetometre = false;
}

function updateCapteursIcons(){
  setCapteurIcon('cap-gps', capteursDisponibles.gps);
  setCapteurIcon('cap-accel', capteursDisponibles.accelerometre);
  setCapteurIcon('cap-gyro', capteursDisponibles.gyroscope);
  setCapteurIcon('cap-mag', capteursDisponibles.magnetometre);
}

// ---------- START / STOP / RESET ----------
function startCockpit(){
  // HTTPS check (localhost allowed)
  const secure = (location.protocol === 'https:') || (location.hostname === 'localhost' || location.hostname === '127.0.0.1');
  if(!secure){
    alert('⚠️ Le GPS et certains capteurs nécessitent HTTPS (ou localhost). Héberge la page en HTTPS ou utilisez localhost.');
    safeSetText('avertissement','⚠️ HTTPS requis (ou lance en localhost).');
    // still allow starting but warn
  }
  if(!('geolocation' in navigator)){ safeSetText('gps','GPS non disponible'); return; }
  if(watchId !== null) return; // déjà démarré

  // initialisation
  tempsDebut = Date.now();
  positionPrecedente = null;
  distanceTotale = 0;
  kalmanState = { x: 0, P: KALMAN_P_INIT };
  lastKalmanX = 0;
  lastKalmanTime = 0;
  vitesseMax = 0;
  vitessesFiltrees = [];

  // activate sensors
  activateAccelerometer();
  activateGyroscope();
  activateMagnetometer();

  // start GPS watch
  watchId = navigator.geolocation.watchPosition(
    pos => {
      capteursDisponibles.gps = true;
      processGPSPosition(pos);
    },
    err => {
      capteursDisponibles.gps = false;
      safeSetText('gps', `Erreur GPS (${err.code}): ${err.message}`);
      updateCapteursIcons();
    },
    { enableHighAccuracy: true, maximumAge: 0, timeout: 15000 }
  );
}

function stopCockpit(){
  if(watchId !== null){
    navigator.geolocation.clearWatch(watchId);
    watchId = null;
  }
  // not stopping sensors event listeners (they are lightweight), but could remove them if wanted
  capteursDisponibles.gps = false;
  updateCapteursIcons();

  // reset UI (keeps maxima)
  safeSetText('vitesse-raw','Vitesse brute 3D : -- km/h');
  safeSetText('vitesse-kalman','Vitesse filtrée (Kalman) : -- km/h');
  safeSetText('distance','Distance totale : 0.000 km');
  safeSetText('temps','Temps écoulé : 0.00 s');
  safeSetText('gps','GPS : --');
  safeSetText('accel-z','Acc Z : 0.00 m/s²');
  safeSetText('kalman-gain','K : --');
  safeSetText('kalman-error','P : --');
  safeSetText('mode','Mode : --');
}

function resetMax(){
  vitesseMax = 0;
  safeSetText('vitesse-max', `Record : ${vitesseMax.toFixed(2)} km/h`);
}

// ---------- DOM bindings ----------
document.addEventListener('DOMContentLoaded', () => {
  $id('marche').addEventListener('click', startCockpit);
  $id('arreter').addEventListener('click', stopCockpit);
  $id('reset').addEventListener('click', resetMax);
  // refresh icons periodically
  setInterval(updateCapteursIcons, 500);
});
  
