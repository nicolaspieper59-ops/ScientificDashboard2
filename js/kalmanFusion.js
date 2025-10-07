let watchID = null;
let lastPos = null;
let totalDistance = 0;
let maxSpeed = 0;
let startTime = null;
let souterrain = false;

// Capteurs activables
let capteurs = {
  gps: true,
  accel: true,
  gyro: true,
  baro: true
};

// Kalman
class Kalman {
  constructor(r = 0.2, q = 1.0) {
    this.R = r; this.Q = q; this.A = 1; this.C = 1;
    this.cov = NaN; this.x = NaN;
  }
  filter(z) {
// --- Kalman class (complète) ---
class Kalman {
  constructor(r = 0.2, q = 1.0) {
    // r : variance processus (how much we trust model)
    // q : variance mesure (how noisy measurements are)
    this.R = r; // processus noise (we'll treat as process covariance increment)
    this.Q = q; // measurement noise
    this.A = 1;
    this.C = 1;
    this.cov = NaN;
    this.x = NaN;
  }

  // z : measurement (we use speed in km/h as measurement here)
  // returns filtered value
  filter(z) {
    if (isNaN(this.x)) {
      // first measurement initialization
      this.x = z;
      this.cov = this.Q;
    } else {
      // Predict (here trivial because A=1 and no control input)
      const predX = this.A * this.x;
      const predCov = this.A * this.cov * this.A + this.R;

      // Kalman gain
      const K = predCov * this.C / (this.C * predCov * this.C + this.Q);

      // Update
      this.x = predX + K * (z - this.C * predX);
      this.cov = (1 - K * this.C) * predCov;
    }
    return this.x;
  }

  // optional: allow external reset
  reset(x0 = NaN) {
    this.x = x0;
    this.cov = isNaN(x0) ? NaN : this.Q;
  }
}

const kalman = new Kalman(0.02, 0.5); // params tuned for smoothing behavior

// === Inertial dead-reckoning state (for souterrain mode) ===
let inertial = {
  enabled: false,
  lastTime: null,
  vx: 0, // m/s horizontal estimated speed
  vy: 0, // if needed vertical
  x: 0,  // cumulative inertial distance (m)
  y: 0
};

// === Sensor state ===
let accelState = { ax: 0, ay: 0, az: 0 }; // m/s² (accelerationIncludingGravity or acceleration if available)
let hasLinearAcceleration = false;
let barometerPressure = null;
let gyroState = { gx: 0, gy: 0, gz: 0 };
let sensorListenersActive = false;

// === Timers & thresholds ===
const GPS_TIMEOUT_MS = 3000; // si pas de mise à jour GPS pendant 3s => mode souterrain
let lastGpsTimestamp = 0;
let gpsTimeoutHandle = null;

// === Utility small functions ===
function nowMs() { return performance.now(); }
function clamp(v, a, b) { return Math.max(a, Math.min(b, v)); }

// === Sensor starters ===
function startSensors() {
  if (sensorListenersActive) return;
  sensorListenersActive = true;

  // DeviceMotion: prefer .acceleration (without gravity) if available
  if ('DeviceMotionEvent' in window) {
    window.addEventListener('devicemotion', e => {
      // prefer e.acceleration (non-gravity) if present; fallback to accelerationIncludingGravity
      const accObj = e.acceleration && (e.acceleration.x !== null) ? e.acceleration : e.accelerationIncludingGravity;
      if (accObj) {
        accelState.ax = accObj.x || 0;
        accelState.ay = accObj.y || 0;
        accelState.az = accObj.z || 0;
        hasLinearAcceleration = !!(e.acceleration && (e.acceleration.x !== null));
      }
    });
  }

  // Gyroscope
  if ('Gyroscope' in window) {
    try {
      const g = new Gyroscope({ frequency: 60 });
      g.addEventListener('reading', () => {
        gyroState.gx = g.x || 0;
        gyroState.gy = g.y || 0;
        gyroState.gz = g.z || 0;
      });
      g.start();
    } catch (e) {
      // ignore if not permitted
    }
  }

  // Barometer / PressureSensor (experimental)
  if ('PressureSensor' in window) {
    try {
      const p = new PressureSensor({ frequency: 1 });
      p.addEventListener('reading', () => {
        barometerPressure = p.pressure;
        safeSetText('altitude', `Altitude : (baro) ${barometerPressure.toFixed(1)} hPa`);
      });
      p.start();
    } catch (e) {
      // not available
    }
  }

  // Note: other sensors (Infrared, Ultrasonic, RayonsX) typically not accessible from browser
  // and require external hardware + native app. We keep placeholders in the UI.
}

// === Stop sensors (listeners cleanup) ===
function stopSensors() {
  // We don't remove DeviceMotion/other listeners aggressively to keep example simple.
  // For production, keep references and removeEventListener appropriately.
  sensorListenersActive = false;
}

// === GPS callbacks & mode switching ===
function enterSouterrainMode() {
  if (souterrain) return;
  souterrain = true;
  inertial.enabled = true;
  inertial.lastTime = nowMs();
  inertial.vx = 0;
  inertial.x = 0;
  safeSetText('mode', 'Mode : 🕳️ Souterrain (inertiel)');
  // start inertial integration loop
  inertialLoop();
}

function exitSouterrainMode() {
  if (!souterrain) return;
  souterrain = false;
  inertial.enabled = false;
  inertial.lastTime = null;
  safeSetText('mode', 'Mode : 🌐 Surface');
  // On exit, we will reconcile inertial state with GPS in onPosition
}

// inertial integration using accelState; called by requestAnimationFrame
function inertialLoop() {
  if (!inertial.enabled) return;
  const t = nowMs();
  const dt = (inertial.lastTime) ? (t - inertial.lastTime) / 1000 : 0;
  inertial.lastTime = t;

  // approximate horizontal acceleration magnitude
  // If device provides linear acceleration (no gravity) use that, else try to remove gravity roughly.
  let ax = accelState.ax || 0;
  let ay = accelState.ay || 0;

  if (!hasLinearAcceleration) {
    // attempt to remove gravity using device orientation is complex; we approximate by subtracting 0 on Z
    // This is a rough approach — expect drift over time.
    // Use az solely for vertical; horizontals are less reliable without sensor fusion.
  }

  // integrate speed: simple Euler integration (vx += ax*dt)
  inertial.vx += ax * dt; // m/s
  inertial.vy += ay * dt || 0;

  // integrate position: x += vx*dt
  inertial.x += inertial.vx * dt;
  inertial.y += inertial.vy * dt || 0;

  // update UI to show inertial estimate
  const inertialSpeedKmh = inertial.vx * 3.6;
  safeSetText('vitesse-filtrée', `Vitesse filtrée (inertiel): ${inertialSpeedKmh.toFixed(2)} km/h (est.)`);
  safeSetText('vitesse-precision', `Vitesse parfaite : --% (inertiel)`);

  // schedule next
  requestAnimationFrame(inertialLoop);
}

// Called when geolocation gives a new position
function onPosition(pos) {
  lastGpsTimestamp = nowMs();
  // clear any pending timeout that would enter souterrain mode
  if (gpsTimeoutHandle) {
    clearTimeout(gpsTimeoutHandle);
    gpsTimeoutHandle = null;
  }
  // set a new timeout to detect GPS loss
  gpsTimeoutHandle = setTimeout(() => {
    // if no GPS update in GPS_TIMEOUT_MS ms, enter souterrain mode
    enterSouterrainMode();
  }, GPS_TIMEOUT_MS);

  // if souterrain was active, we'll exit and reconcile
  if (souterrain) {
    // exit early from souterrain active (we have GPS again)
    exitSouterrainMode();
    // reconcile: we have inertial.x (meters) and distance since start; we can nudge Kalman state
    // Simple strategy: set kalman state position to cumulative GPS distance (we will compute below)
    // and set kalman.x = totalDistance to remove drift
  }

  // compute raw speed from pos.coords.speed if available else from distance/time
  const coords = pos.coords;
  const gpsPoint = {
    latitude: coords.latitude,
    longitude: coords.longitude,
    altitude: coords.altitude || 0,
    accuracy: coords.accuracy || 999,
    timestamp: pos.timestamp
  };

  // compute 3D delta distance & dt
  const dt = lastPos && lastPos.timestamp ? (pos.timestamp - lastPos.timestamp) / 1000 : 0;
  let delta_m = 0;
  if (lastPos && dt > 0) {
    delta_m = distance3D(lastPos.latitude, lastPos.longitude, lastPos.altitude || 0,
                        gpsPoint.latitude, gpsPoint.longitude, gpsPoint.altitude || 0);
    totalDistance += delta_m;
  }
  lastPos = { latitude: gpsPoint.latitude, longitude: gpsPoint.longitude, altitude: gpsPoint.altitude, timestamp: pos.timestamp };

  // speed in m/s: prefer coords.speed if available, otherwise delta/dt
  let speedMs = (typeof coords.speed === 'number' && !isNaN(coords.speed)) ? coords.speed : (dt > 0 ? delta_m / dt : 0);
  let speedKmh = speedMs * 3.6;

  // If souterrain was active and we have inertial estimates, blend inertial and GPS speeds gently
  if (inertial.enabled && inertial.x) {
    // simple blending weight based on how long GPS was lost (shorter loss => trust GPS more)
    const lostDuration = (nowMs() - lastGpsTimestamp) || 0; // but lastGpsTimestamp was just set above, so approx 0
    // We'll use GPS primarily because it just returned; but nudge Kalman with inertial velocity to avoid jump
    const inertialSpeedKmh = inertial.vx * 3.6;
    // blending factor alpha in [0..1] (1 => GPS), we trust GPS more as accuracy is better
    const gpsReliability = clamp(1 - (gpsPoint.accuracy / 50), 0, 1); // accuracy 0..50 -> 1..0
    const alpha = 0.7 * gpsReliability + 0.3; // bias toward GPS
    speedKmh = alpha * speedKmh + (1 - alpha) * inertialSpeedKmh;
    speedMs = speedKmh / 3.6;
    // reset inertial integration to avoid double-counting
    inertial.vx = speedMs;
    inertial.x = 0;
  }

  // Kalman filtering: use speedKmh as measurement
  const filteredSpeed = kalman.filter(speedKmh);

  // Update stats & UI
  if (!startTime) startTime = Date.now();
  const elapsed = (Date.now() - startTime) / 1000;
  safeSetText('temps', `Temps écoulé : ${elapsed.toFixed(2)} s`);
  safeSetText('vitesse-brute', `Vitesse brute : ${speedKmh.toFixed(2)} km/h`);
  safeSetText('vitesse-filtrée', `Vitesse filtrée : ${filteredSpeed.toFixed(2)} km/h`);
  safeSetText('distance', `Distance totale : ${(totalDistance / 1000).toFixed(3)} km`);
  maxSpeed = Math.max(maxSpeed, speedKmh, filteredSpeed);
  safeSetText('vitesse-max', `Vitesse max : ${maxSpeed.toFixed(2)} km/h`);
  safeSetText('gps', `GPS : Lat:${gpsPoint.latitude.toFixed(6)} | Lon:${gpsPoint.longitude.toFixed(6)} | Alt:${gpsPoint.altitude.toFixed(1)} m`);
  safeSetText('precision', `Précision : ${gpsPoint.accuracy.toFixed(1)} m`);
  safeSetText('kalman-gain', `Gain K : ${(kalman.cov || 0).toFixed(4)}`);
  safeSetText('kalman-error', `Erreur P : ${(kalman.Q || 0).toFixed(4)}`);

  // percentege of "perfect" speed: compare filtered vs brute
  const percentPerfect = speedKmh > 0 ? clamp((filteredSpeed / speedKmh) * 100, 0, 200) : 100;
  safeSetText('vitesse-precision', `Vitesse parfaite : ${percentPerfect.toFixed(1)}%`);

  // update capteurs UI
  let used = ['GPS'];
  if (capteurs.accel) used.push('Accéléro');
  if (capteurs.gyro) used.push('Gyro');
  if (capteurs.baro) used.push('Baro');
  safeSetText('capteurs', `Capteurs actifs : ${used.join(', ')}`);
}

// === GPS error handler ===
function onPositionError(err) {
  console.warn('GPS error', err);
  // if GPS error occurs, schedule entering souterrain mode
  if (!gpsTimeoutHandle) {
    gpsTimeoutHandle = setTimeout(() => enterSouterrainMode(), GPS_TIMEOUT_MS);
  }
}

// === Toggle functions for sensors (buttons) ===
document.getElementById('toggle-gps').onclick = (e) => {
  capteurs.gps = !capteurs.gps;
  e.target.textContent = `GPS : ${capteurs.gps ? '✅' : '❌'}`;
  if (!capteurs.gps && watchID) {
    navigator.geolocation.clearWatch(watchID);
    watchID = null;
  }
  if (capteurs.gps && !watchID) {
    // re-enable if previously disabled
    startWatchingGPS();
  }
};

document.getElementById('toggle-accel').onclick = (e) => {
  capteurs.accel = !capteurs.accel;
  e.target.textContent = `Accéléromètre : ${capteurs.accel ? '✅' : '❌'}`;
  if (capteurs.accel) startSensors();
};

document.getElementById('toggle-gyro').onclick = (e) => {
  capteurs.gyro = !capteurs.gyro;
  e.target.textContent = `Gyroscope : ${capteurs.gyro ? '✅' : '❌'}`;
  if (capteurs.gyro) startSensors();
};

document.getElementById('toggle-baro').onclick = (e) => {
  capteurs.baro = !capteurs.baro;
  e.target.textContent = `Baromètre : ${capteurs.baro ? '✅' : '❌'}`;
  if (capteurs.baro) startSensors();
};

// === Start / stop GPS watching ===
function startWatchingGPS() {
  if (!('geolocation' in navigator)) {
    alert('Géolocalisation non disponible dans ce navigateur.');
    return;
  }
  if (watchID) return;
  startSensors(); // start sensors when we start GPS
  watchID = navigator.geolocation.watchPosition(onPosition, onPositionError, {
    enableHighAccuracy: true,
    maximumAge: 0,
    timeout: 10000
  });
}

function stopWatchingGPS() {
  if (watchID) {
    navigator.geolocation.clearWatch(watchID);
    watchID = null;
  }
  // stopSensors(); // keep sensors active — user may want to test inertial
}

// === Start / Stop buttons (main control) ===
document.getElementById('demarrer').onclick = () => {
  if (!capteurs.gps) {
    alert('Activer le GPS pour démarrer');
    return;
  }
  if (!watchID) {
    startTime = null;
    startWatchingGPS();
    startTime = Date.now();
    safeSetText('mode', 'Mode : 🌐 Surface');
  }
};

document.getElementById('arreter').onclick = () => {
  if (watchID) navigator.geolocation.clearWatch(watchID);
  watchID = null;
  if (gpsTimeoutHandle) { clearTimeout(gpsTimeoutHandle); gpsTimeoutHandle = null; }
  // stop inertial
  inertial.enabled = false;
  souterrain = false;
  // reset UI or leave last values — we reset here
  safeSetText('vitesse-brute', 'Vitesse brute : -- km/h');
  safeSetText('vitesse-filtrée', 'Vitesse filtrée (Kalman) : -- km/h');
  safeSetText('distance', 'Distance totale : -- km');
  safeSetText('vitesse-max', 'Vitesse max : -- km/h');
  safeSetText('mode', 'Mode : ⛔ Arrêté');
};

document.getElementById('reset').onclick = () => {
  totalDistance = 0;
  maxSpeed = 0;
  kalman.reset(NaN);
  inertial.x = 0;
  inertial.vx = 0;
  safeSetText('vitesse-max', 'Vitesse max : -- km/h');
  safeSetText('distance', 'Distance totale : 0.000 km');
  safeSetText('vitesse-filtrée', 'Vitesse filtrée (Kalman) : -- km/h');
  safeSetText('vitesse-brute', 'Vitesse brute : -- km/h');
};

// === on load: initialize UI and sensors availability display ===
document.addEventListener('DOMContentLoaded', () => {
  // set initial button labels (reflect default true values)
  document.getElementById('toggle-gps').textContent = `GPS : ${capteurs.gps ? '✅' : '❌'}`;
  document.getElementById('toggle-accel').textContent = `Accéléromètre : ${capteurs.accel ? '✅' : '❌'}`;
  document.getElementById('toggle-gyro').textContent = `Gyroscope : ${capteurs.gyro ? '✅' : '❌'}`;
  document.getElementById('toggle-baro').textContent = `Baromètre : ${capteurs.baro ? '✅' : '❌'}`;
  safeSetText('mode', 'Mode : ⛳ Prêt');

  // quick detection of sensor availability for UI hints
  let available = [];
  if ('DeviceMotionEvent' in window) available.push('Accéléromètre');
  if ('Gyroscope' in window) available.push('Gyroscope');
  if ('Magnetometer' in window || 'ondeviceorientationabsolute' in window) available.push('Magnétomètre');
  if ('AmbientLightSensor' in window) available.push('Luminosité');
  if ('PressureSensor' in window) available.push('Baromètre');
  safeSetText('capteurs', `Capteurs détectés : ${available.join(', ') || '--'}`);

  // ensure sensors started if toggles true
  if (capteurs.accel || capteurs.gyro || capteurs.baro) startSensors();
});
                 
  
