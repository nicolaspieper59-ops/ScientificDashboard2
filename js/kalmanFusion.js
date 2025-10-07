// kalmanFusion.js
// Intégration : GPS 3D + capteurs (accel, gyro, baro, magneto) + filtre Kalman 1D sur distance cumulative
// Conçu pour fonctionner avec index.html + utils.js

// ---------- Configuration ----------
const KALMAN_Q = 0.0001;      // bruit du modèle
const KALMAN_P_INIT = 1000;   // covariance initiale (forte incertitude)

// ---------- Etats globaux ----------
let watchId = null;
let positionPrev = null;
let distanceTotale = 0;
let vitesseMax = 0;
let vitessesFiltrees = [];
let tempsDebut = null;
let lastKalmanTime = 0;

let accel = { x: 0, y: 0, z: 0 };
let gyro = { x: 0, y: 0, z: 0 };
let magneto = { x: 0, y: 0, z: 0 };
let baro = null;

// Capteurs détectés / utilisés (affichage)
const detected = {
  accelerometer: false,
  gyroscope: false,
  magnetometer: false,
  barometer: false,
  ambientLight: false,
  microphone: false
};

// KalmanState (1D : position cumulative en m & vitesse m/s)
let kalmanState = { x: 0, v: 0, P: KALMAN_P_INIT };

// ---------- UTIL (importées depuis utils.js) ----------
/* safeSetText, msToKmh, haversine3D are available from utils.js */

// ---------- Fonctions de calcul ----------
function calculerVitesse3DAndDelta(pos) {
  if (!positionPrev) {
    positionPrev = pos;
    return { v: 0, d: 0, dt: 0 };
  }
  const dt = (pos.timestamp - positionPrev.timestamp) / 1000;
  if (dt <= 0) {
    positionPrev = pos;
    return { v: 0, d: 0, dt: 0 };
  }
  const d = haversine3D(positionPrev, pos); // metres
  const v = d / dt; // m/s
  positionPrev = pos;
  return { v, d, dt };
}

function kalmanPredict(dt, accAlong = 0) {
  // x = x + v*dt + 0.5*acc*dt^2  (we use accAlong to nudge v)
  kalmanState.x += kalmanState.v * dt + 0.5 * accAlong * dt * dt;
  kalmanState.v += accAlong * dt;
  kalmanState.P += KALMAN_Q;
}

function kalmanUpdate(measValue, measVariance) {
  const R = Math.max(1e-6, measVariance); // avoid 0
  const K = kalmanState.P / (kalmanState.P + R);
  kalmanState.x += K * (measValue - kalmanState.x);
  // Joseph form simplified
  kalmanState.P = (1 - K) * kalmanState.P;
  return K;
}

// ---------- Capteurs : détection et listeners ----------
function startSensors() {
  // Accelerometer (DeviceMotion)
  if ('DeviceMotionEvent' in window) {
    detected.accelerometer = true;
    window.addEventListener('devicemotion', e => {
      accel.x = e.accelerationIncludingGravity?.x ?? 0;
      accel.y = e.accelerationIncludingGravity?.y ?? 0;
      accel.z = e.accelerationIncludingGravity?.z ?? 0;
      safeSetText('accel-z', `Accélération Z : ${accel.z.toFixed(2)} m/s²`);
    });
  }

  // Gyroscope (Generic Sensor API)
  if ('Gyroscope' in window) {
    try {
      detected.gyroscope = true;
      const g = new Gyroscope({ frequency: 60 });
      g.addEventListener('reading', () => {
        gyro.x = g.x; gyro.y = g.y; gyro.z = g.z;
        // optionally show gyro values if you add UI
      });
      g.start();
    } catch (e) {
      detected.gyroscope = false;
    }
  }

  // Magnetometer
  if ('Magnetometer' in window) {
    try {
      detected.magnetometer = true;
      const m = new Magnetometer({ frequency: 10 });
      m.addEventListener('reading', () => {
        magneto.x = m.x; magneto.y = m.y; magneto.z = m.z;
        safeSetText('cap-magneto', `${Math.round(Math.sqrt(m.x*m.x + m.y*m.y + m.z*m.z))} µT`);
      });
      m.start();
    } catch (e) {
      detected.magnetometer = false;
    }
  } else if ('ondeviceorientationabsolute' in window) {
    // fallback to deviceorientation for coarse magneto / heading
    window.addEventListener('deviceorientationabsolute', ev => {
      detected.magnetometer = true;
      magneto.x = ev.alpha ?? 0;
      magneto.y = ev.beta ?? 0;
      magneto.z = ev.gamma ?? 0;
      safeSetText('cap-magneto', `heading:${Math.round(magneto.x)}`);
    });
  }

  // Ambient light
  if ('AmbientLightSensor' in window) {
    try {
      detected.ambientLight = true;
      const light = new AmbientLightSensor();
      light.addEventListener('reading', () => safeSetText('cap-lumiere', `${Math.round(light.illuminance)} lux`));
      light.start();
    } catch (e) {
      detected.ambientLight = false;
    }
  }

  // Microphone (sound level via analyser)
  if (navigator.mediaDevices && navigator.mediaDevices.getUserMedia) {
    navigator.mediaDevices.getUserMedia({ audio: true }).then(stream => {
      detected.microphone = true;
      const audioCtx = new (window.AudioContext || window.webkitAudioContext)();
      const source = audioCtx.createMediaStreamSource(stream);
      const analyser = audioCtx.createAnalyser();
      analyser.fftSize = 1024;
      source.connect(analyser);
      const data = new Uint8Array(analyser.frequencyBinCount);
      function sampleSound() {
        analyser.getByteFrequencyData(data);
        const avg = data.reduce((a,b)=>a+b,0) / data.length;
        safeSetText('cap-son', `${Math.round(20 * Math.log10(avg || 1))} dB`);
        requestAnimationFrame(sampleSound);
      }
      sampleSound();
    }).catch(()=>{ detected.microphone = false; });
  }

  // Barometer (experimental - not available in many browsers)
  if ('PressureSensor' in window) {
    try {
      detected.barometer = true;
      const b = new PressureSensor({ frequency: 1 });
      b.addEventListener('reading', () => {
        baro = b.pressure;
        safeSetText('cap-baro', `${baro.toFixed(1)} hPa`);
      });
      b.start();
    } catch (e) {
      detected.barometer = false;
    }
  }

  // Update detected list UI
  updateDetectedUI();
}

function updateDetectedUI() {
  const used = Object.entries(detected).filter(([k,v]) => v).map(([k]) => k).join(', ') || '--';
  safeSetText('capteurs-utilises', used);
}

// ---------- Main: mise à jour GPS callback ----------
function onPosition(pos) {
  // pos is GeolocationPosition
  const now = pos.timestamp;
  const gpsPoint = {
    latitude: pos.coords.latitude,
    longitude: pos.coords.longitude,
    altitude: pos.coords.altitude ?? 0,
    accuracy: pos.coords.accuracy ?? 999,
    timestamp: now,
    speed: pos.coords.speed ?? null
  };

  // 1) brut : distance/time
  const { v: v_raw_ms, d: delta_m, dt } = calculerVitesse3DAndDelta(gpsPoint);
  distanceTotale += delta_m;
  const v_raw_kmh = msToKmh(v_raw_ms);

  // 2) kalman predict+update
  const measPos = distanceTotale;           // position 1D = distance cumulative (m)
  const measVar = (gpsPoint.accuracy ?? 10) ** 2;
  const dtKal = lastKalmanTime ? (now - lastKalmanTime)/1000 : 0.05;
  lastKalmanTime = now;

  // choose accAlong: project accel on travel direction — simple approx: use accel.z magnitude
  const accAlong = accel.z || 0;

  // Predict
  kalmanPredict(dtKal, accAlong);

  // Update with GNSS measurement (distance cumulative)
  const K = kalmanUpdate(measPos, measVar);

  // Filtered speed (m/s) from Kalman state
  const v_filt_kmh = msToKmh(kalmanState.v);

  // Update stats
  vitesseMax = Math.max(vitesseMax, v_filt_kmh);
  vitessesFiltrees.push(v_filt_kmh);
  if (vitessesFiltrees.length > 60) vitessesFiltrees.shift();

  // Percentage of "perfect" speed : compare filtered vs raw (if raw non-zero)
  const percentPerfect = v_raw_kmh > 0 ? Math.min(100, Math.max(0, (v_filt_kmh / v_raw_kmh) * 100)) : 100;

  // UI updates
  safeSetText('vitesse-raw', `Vitesse GPS Brute : ${v_raw_kmh.toFixed(2)} km/h`);
  safeSetText('vitesse-kalman', `Vitesse Filtrée : ${v_filt_kmh.toFixed(2)} km/h`);
  safeSetText('vitesse-max', `Vitesse max : ${vitesseMax.toFixed(2)} km/h`);
  safeSetText('distance', `Distance totale : ${(distanceTotale/1000).toFixed(3)} km`);
  safeSetText('temps', `Temps écoulé : ${tempsDebut ? ((now - tempsDebut)/1000).toFixed(2) : '0.00'} s`);
  safeSetText('vitesse-precision', `Pourcentage de vitesse parfaite : ${percentPerfect.toFixed(1)}%`);
  safeSetText('kalman-gain', `Gain Kalman K : ${K.toFixed(4)}`);
  safeSetText('kalman-error', `Erreur P : ${kalmanState.P.toFixed(2)} m²`);
  safeSetText('gps', `GPS : Lat:${gpsPoint.latitude.toFixed(6)} | Lon:${gpsPoint.longitude.toFixed(6)} | Alt:${gpsPoint.altitude.toFixed(1)} m | Acc:${gpsPoint.accuracy.toFixed(1)} m`);
  // capteurs detail will be filled by sensor listeners
  updateDetectedUI();
}

// ---------- start / stop / reset ----------
function startCockpit() {
  if (location.protocol !== 'https:' && location.hostname !== 'localhost') {
    alert('⚠️ Pour accéder aux capteurs et au GPS, utilise HTTPS ou localhost.');
    return;
  }
  if (!navigator.geolocation) {
    alert('GPS non disponible dans ce navigateur.');
    return;
  }
  if (watchId !== null) return;

  // reset state
  tempsDebut = Date.now();
  positionPrev = null;
  distanceTotale = 0;
  vitesseMax = 0;
  vitessesFiltrees = [];
  kalmanState = { x: 0, v: 0, P: KALMAN_P_INIT };
  lastKalmanTime = 0;

  // start sensors
  startSensors();

  // start watchPosition
  watchId = navigator.geolocation.watchPosition(onPosition, err => {
    safeSetText('gps', `Erreur GPS: ${err.message || err.code}`);
  }, {
    enableHighAccuracy: true,
    maximumAge: 0,
    timeout: 10000
  });
}

function stopCockpit() {
  if (watchId !== null) navigator.geolocation.clearWatch(watchId);
  watchId = null;
  positionPrev = null;
  tempsDebut = null;
  distanceTotale = 0;
  vitesseMax = 0;
  vitessesFiltrees = [];
  kalmanState = { x: 0, v: 0, P: KALMAN_P_INIT };
  lastKalmanTime = 0;

  // reset UI
  safeSetText('vitesse-raw', 'Vitesse GPS Brute : -- km/h');
  safeSetText('vitesse-kalman', 'Vitesse Filtrée : -- km/h');
  safeSetText('vitesse-max', 'Vitesse max : -- km/h');
  safeSetText('distance', 'Distance totale : -- km');
  safeSetText('temps', 'Temps écoulé : 0.00 s');
  safeSetText('vitesse-precision', 'Pourcentage de vitesse parfaite : --%');
  safeSetText('kalman-gain', 'Gain Kalman K : --');
  safeSetText('kalman-error', 'Erreur P : --');
  safeSetText('accel-z', 'Accélération Z : 0.00 m/s²');
  safeSetText('capteurs-utilises', '--');
}

function resetVitesseMax() {
  vitesseMax = 0;
  safeSetText('vitesse-max', 'Vitesse max : 0.00 km/h');
}

// ---------- events ----------
document.addEventListener('DOMContentLoaded', () => {
  document.getElementById('marche').addEventListener('click', startCockpit);
  document.getElementById('arreter').addEventListener('click', stopCockpit);
  document.getElementById('reset').addEventListener('click', resetVitesseMax);
  // initial UI blank
  stopCockpit();
});
      
