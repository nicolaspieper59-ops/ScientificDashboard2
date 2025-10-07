// ========================
// cockpit.js – COMPLET
// - vitesse brute (GPS distance/time 3D)
// - vitesse filtrée (Kalman/simple filter) référencée sur la brute
// - capteurs activables/désactivables
// - réglage Hz
// - % perfection vitesse (cohérence capteurs)
// - calibration automatique (ajuste gain & Hz)
// ========================

// ---- Etat global ----
let watchId = null;
let positionPrecedente = null;
let distanceTotale = 0;          // en mètres
let vitesseMax = 0;             // en m/s
let vitessesFiltrees = [];      // en m/s (historique pour lissage)
let tempsDebut = null;
let lastUpdateTime = 0;
let refreshHz = 5;              // par défaut (modifiable)
let filterGain = 0.15;          // 0..1 (pour lissage simple)
let kalmanCov = 1.0;            // paramètre pour information (optionnel)

// ---- Capteurs activables ----
const capteursActifs = {
  gps: true,
  accel: true,
  baro: false,
  gyro: false,
  magnet: false,
  lum: false,
  mic: false,
  infra: false
};

// ---- Etats capteurs ----
let accel = { x: 0, y: 0, z: 0 }; // m/s² (accelerationIncludingGravity)
let altitudeBaro = null;          // hPa or meters (simulé if not present)

// ---- Utilitaires DOM ----
function safeSetText(id, text) {
  const e = document.getElementById(id);
  if (e) e.textContent = text;
}

// ---- Géodésie simple 3D ----
const R_EARTH = 6371e3;
function toRad(d) { return d * Math.PI / 180; }
function haversine3D(p1, p2) {
  const φ1 = toRad(p1.latitude), φ2 = toRad(p2.latitude);
  const Δφ = toRad(p2.latitude - p1.latitude);
  const Δλ = toRad(p2.longitude - p1.longitude);
  const a = Math.sin(Δφ / 2) ** 2 + Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ / 2) ** 2;
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
  const d2D = R_EARTH * c;
  const dz = (p2.altitude ?? 0) - (p1.altitude ?? 0);
  return Math.sqrt(d2D * d2D + dz * dz);
}

// ---- Calcul vitesse brute 3D ----
function calculerVitesse3D(pos) {
  if (!positionPrecedente) {
    positionPrecedente = pos;
    return { vitesse: 0, distance: 0, vertical: 0, dt: 0 };
  }
  const dt = (pos.timestamp - positionPrecedente.timestamp) / 1000;
  if (dt <= 0) return { vitesse: 0, distance: 0, vertical: 0, dt: 0 };
  const d = haversine3D(positionPrecedente, pos); // m
  const dz = (pos.altitude ?? 0) - (positionPrecedente.altitude ?? 0);
  positionPrecedente = pos;
  return { vitesse: d / dt, distance: d, vertical: dz / dt, dt };
}

// ---- Filtre simple (référence vitesse brute) ----
function filtrerVitesse(vBrute) {
  // we keep speeds in m/s domain for accuracy
  const prev = vitessesFiltrees.length ? vitessesFiltrees[vitessesFiltrees.length - 1] : vBrute;
  const vFilt = prev + filterGain * (vBrute - prev);
  vitessesFiltrees.push(vFilt);
  if (vitessesFiltrees.length > 120) vitessesFiltrees.shift();
  return vFilt;
}

// ---- Calcul du score / % perfection de la vitesse ----
function computeSensorConfidence(gpsAccuracyM) {
  // Basic per-sensor confidence 0..1
  const conf = {};
  // GPS confidence: based on accuracy (smaller = better). 1.0 at <=3m, 0 at >=100m
  conf.gps = 1 - Math.min(1, Math.max(0, (gpsAccuracyM - 3) / (100 - 3)));
  conf.accel = capteursActifs.accel ? 0.8 : 0.0;
  conf.baro = capteursActifs.baro ? 0.6 : 0.0;
  conf.gyro = capteursActifs.gyro ? 0.5 : 0.0;
  conf.magnet = capteursActifs.magnet ? 0.4 : 0.0;
  conf.lum = capteursActifs.lum ? 0.2 : 0.0;
  conf.mic = capteursActifs.mic ? 0.2 : 0.0;
  conf.infra = capteursActifs.infra ? 0.1 : 0.0;
  return conf;
}

function computePerfectionPercentage(gpsAccuracyM, vBrute, vInertialEstimate) {
  // Components:
  // - sensor quality average (weighted)
  // - agreement between GPS speed and inertial estimate (if accel available)
  const conf = computeSensorConfidence(gpsAccuracyM);
  // weighted average of available confs
  let sum = 0, count = 0;
  for (const k in conf) {
    if (conf[k] > 0) { sum += conf[k]; count++; }
  }
  const sensorQuality = count ? (sum / count) : 0;

  // agreement score: compare vBrute (m/s) vs vInertialEstimate (m/s)
  let agreement = 1.0;
  if (vInertialEstimate != null && vBrute > 0) {
    const relErr = Math.abs(vBrute - vInertialEstimate) / Math.max(0.1, vBrute);
    // map relative error to [0,1] (0 = totally wrong, 1 = perfect)
    agreement = Math.max(0, 1 - Math.min(1, relErr / 1.0)); // if relErr=1 -> 0
  }

  // final percentage: sensorQuality * 0.6 + agreement * 0.4 scaled to 100
  const perc = (0.6 * sensorQuality + 0.4 * agreement) * 100;
  return Math.round(perc * 10) / 10; // 1 decimal
}

// ---- Estimation inertielle simple (speed from accel magnitude) ----
let inertialState = { vx: 0, vy: 0, lastT: null };
// integrate accel magnitude -> approximate horizontal speed (very rough)
function estimateSpeedFromAccel() {
  if (!capteursActifs.accel) return null;
  const t = performance.now();
  if (!inertialState.lastT) { inertialState.lastT = t; return null; }
  const dt = (t - inertialState.lastT) / 1000;
  inertialState.lastT = t;
  // project accel on horizontal plane magnitude
  const ax = accel.x || 0, ay = accel.y || 0;
  const horizAcc = Math.sqrt(ax * ax + ay * ay);
  inertialState.vx += horizAcc * dt;
  // simple damping to mimic friction/noise removal
  inertialState.vx *= 0.999;
  return Math.abs(inertialState.vx);
}

// ---- Calibration automatique (ajuste filterGain & refreshHz) ----
function autoCalibrate(gpsAccuracyM, currentBruteSpeed, perfectionPct) {
  // Rules (heuristic):
  // - If GPS accuracy poor (>30m), reduce Hz and increase smoothing
  // - If speed low (<0.5 m/s), decrease filter aggressiveness (more smoothing) to avoid jitter
  // - If perfectionPct high, can lower smoothing and increase Hz for more responsiveness
  // - If in "sudden change" (big diff between brute and filtered), increase Hz for shorter response

  // baseline
  let targetHz = 5;
  let targetGain = 0.12;

  // GPS accuracy influence
  if (gpsAccuracyM <= 5) { targetHz = 10; targetGain = 0.10; }
  else if (gpsAccuracyM <= 15) { targetHz = 8; targetGain = 0.12; }
  else if (gpsAccuracyM <= 30) { targetHz = 6; targetGain = 0.18; }
  else { targetHz = 3; targetGain = 0.28; } // bad GPS -> more smoothing, less updates

  // speed influence
  if (currentBruteSpeed < 0.5) { targetGain = Math.max(targetGain, 0.22); targetHz = Math.min(targetHz, 5); }
  if (currentBruteSpeed > 10) { targetGain = Math.min(targetGain, 0.08); targetHz = Math.max(targetHz, 10); }

  // perfection influence
  if (perfectionPct > 85) { targetHz = Math.max(targetHz, 12); targetGain = Math.max(0.07, targetGain - 0.02); }
  if (perfectionPct < 50) { targetGain = Math.min(0.35, targetGain + 0.05); targetHz = Math.max(2, Math.min(targetHz, 6)); }

  // apply smoothing to changes (avoid abrupt jumps)
  filterGain = filterGain + 0.2 * (targetGain - filterGain);
  refreshHz = Math.round(refreshHz + 0.2 * (targetHz - refreshHz));

  // clamp
  filterGain = Math.max(0.02, Math.min(0.6, filterGain));
  refreshHz = Math.max(1, Math.min(60, refreshHz));

  // show calibration hints in UI
  safeSetText('frequence', `${refreshHz} Hz`);
  safeSetText('precision', `Précision globale : ${typeof currentBruteSpeed === 'number' ? (computePerfectionPercentage(lastGpsAccuracy||999, currentBruteSpeed, inertialState.vx||0)) : '--'}%`);
}

// ---- Sensor listeners ----
function startSensors() {
  if (capteursActifs.accel && 'DeviceMotionEvent' in window) {
    window.addEventListener('devicemotion', e => {
      accel.x = e.acceleration?.x ?? e.accelerationIncludingGravity?.x ?? 0;
      accel.y = e.acceleration?.y ?? e.accelerationIncludingGravity?.y ?? 0;
      accel.z = e.acceleration?.z ?? e.accelerationIncludingGravity?.z ?? 0;
    });
  }

  // baro/other sensors: not widely available in browsers — placeholders
}

// ---- Tracking control & update ----
let lastGpsAccuracy = 999;
let lastBruteSpeed = 0;

function startTracking() {
  if (watchId) return;
  if (!('geolocation' in navigator)) { alert('Géolocalisation non disponible'); return; }
  if (location.protocol !== 'https:' && location.hostname !== 'localhost') {
    alert('Utilise HTTPS ou localhost pour accéder aux capteurs');
    return;
  }

  tempsDebut = Date.now();
  positionPrecedente = null;
  distanceTotale = 0;
  vitesseMax = 0;
  vitessesFiltrees = [];
  inertialState = { vx: 0, vy: 0, lastT: null };
  lastUpdateTime = 0;
  startSensors();

  watchId = navigator.geolocation.watchPosition(onPosition, err => {
    console.warn('GPS error', err);
    safeSetText('gps', `Erreur GPS: ${err.message || err.code}`);
  }, {
    enableHighAccuracy: true,
    maximumAge: 0,
    timeout: 10000
  });
}

function stopTracking() {
  if (watchId) navigator.geolocation.clearWatch(watchId);
  watchId = null;
  safeSetText('vitesse-brute', '--');
  safeSetText('vitesse-filtree', '--');
  safeSetText('vitesse-verticale', '--');
  safeSetText('vitesse-max', '--');
  safeSetText('distance', '--');
  safeSetText('frequence', `${refreshHz} Hz`);
  safeSetText('precision', '--');
}

function onPosition(pos) {
  const now = Date.now();
  if (now - lastUpdateTime < 1000 / refreshHz) {
    // still update some minimal UI values if needed but skip heavy calc
    return;
  }
  lastUpdateTime = now;

  const gpsPoint = {
    latitude: pos.coords.latitude,
    longitude: pos.coords.longitude,
    altitude: pos.coords.altitude ?? altitudeBaro ?? 0,
    accuracy: pos.coords.accuracy ?? 999,
    timestamp: pos.timestamp
  };

  lastGpsAccuracy = gpsPoint.accuracy;

  // compute brute velocity
  const { vitesse: vBrute, distance: dDelta, vertical: vVert } = calculerVitesse3D(gpsPoint);
  lastBruteSpeed = vBrute;

  distanceTotale += dDelta;

  // inertial estimate
  const vInertial = estimateSpeedFromAccel(); // m/s or null

  // compute perfection % using current data
  const perfectionPct = computePerfectionPercentage(gpsPoint.accuracy, vBrute, vInertial ?? null);

  // automatic calibration
  autoCalibrate(gpsPoint.accuracy, vBrute, perfectionPct);

  // filtered velocity (based on brute + filterGain)
  const vFilt = filtrerVitesse(vBrute);

  // update stats
  if (vFilt > vitesseMax) vitesseMax = vFilt;

  // Update UI: convert m/s -> km/h for readability except internal displays
  safeSetText('vitesse-brute', `${(vBrute * 3.6).toFixed(2)} km/h`);
  safeSetText('vitesse-filtree', `${(vFilt * 3.6).toFixed(2)} km/h`);
  safeSetText('vitesse-verticale', `${(vVert).toFixed(3)} m/s`);
  safeSetText('vitesse-max', `${(vitesseMax * 3.6).toFixed(2)} km/h`);
  safeSetText('distance', `${(distanceTotale / 1000).toFixed(3)} km`);
  safeSetText('gps', `Lat:${gpsPoint.latitude.toFixed(6)} Lon:${gpsPoint.longitude.toFixed(6)} Alt:${gpsPoint.altitude.toFixed(1)} m Acc:${gpsPoint.accuracy.toFixed(1)} m`);
  safeSetText('frequence', `${refreshHz} Hz`);
  safeSetText('precision', `${perfectionPct.toFixed(1)} %`);
  // reflect active sensors on UI (buttons must exist)
  const activeList = Object.keys(capteursActifs).filter(k => capteursActifs[k]);
  safeSetText('capteurs-list', activeList.join(', ') || 'aucun');
}

// ---- UI helpers (toggle & frequency change) ----
function toggleCapteur(name) {
  if (!(name in capteursActifs)) return;
  capteursActifs[name] = !capteursActifs[name];
  const btn = document.getElementById('btn-' + name);
  if (btn) btn.classList.toggle('active', capteursActifs[name]);
  // if enabling accel, ensure sensor listener attached
  if (name === 'accel' && capteursActifs.accel) startSensors();
}

function changerFrequenceFromInput() {
  const el = document.getElementById('hz');
  if (!el) return;
  const v = Number(el.value);
  if (!isNaN(v) && v >= 1 && v <= 60) {
    refreshHz = Math.round(v);
    safeSetText('frequence', `${refreshHz} Hz`);
  }
}

// ---- init DOM listeners ----
document.addEventListener('DOMContentLoaded', () => {
  // start/stop buttons expected to exist in HTML
  const startBtn = document.querySelector('[onclick="startTracking()"]');
  if (!startBtn) {
    // fallback: attach to known id if present
    const sb = document.getElementById('start-btn');
    if (sb) sb.addEventListener('click', startTracking);
  }
  // map common named functions to window for inline onclick HTML usage
  window.startTracking = startTracking;
  window.stopTracking = stopTracking;
  window.toggleCapteur = toggleCapteur;
  window.changerFrequence = changerFrequenceFromInput;
  window.changerFrequenceFromInput = changerFrequenceFromInput;

  // set default UI texts
  safeSetText('vitesse-brute', '--');
  safeSetText('vitesse-filtree', '--');
  safeSetText('vitesse-verticale', '--');
  safeSetText('vitesse-max', '--');
  safeSetText('distance', '--');
  safeSetText('frequence', `${refreshHz} Hz`);
  safeSetText('precision', '--');
});
