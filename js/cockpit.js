// cockpit.js
// Module unique pour le "Cockpit Cosmique" — GPS, capteurs, astronomie, météo, grandeurs physiques.
// Usage: <script type="module" src="cockpit.js"></script>

// ---------- Dépendances ----------
import * as SunCalc from 'https://cdn.jsdelivr.net/npm/suncalc@1.9.0/suncalc.js';

// ---------- Variables globales ----------
let watchId = null;
let positionPrecedente = null;
let vitesses = [];
let vitesseMax = 0;
let distanceTotale = 0;
let t0 = null;

let destination = { latitude: null, longitude: null };
let masseSimulee = 70; // kg (modifiable par l'UI)
const applicateurs = { physique: true, chimie: true, svt: true };
let rituelActif = true;

// Internal handles to stop loops
let audioAnalyserHandle = null;
let audioStream = null;
let deviceOrientationHandler = null;
let magnetometerInstance = null;
let gyroscopeInstance = null;
let medaillonRAF = null;

// Small helpers for DOM
const set = (id, txt) => { const el = document.getElementById(id); if (el) el.textContent = txt; };
const appendLimited = (id, txt, limit = 800) => {
  const el = document.getElementById(id);
  if (!el) return;
  // keep base separate to avoid runaway growth (we store base in data attribute)
  const base = el.dataset.base ?? '';
  const current = el.textContent || '';
  const newText = current.length > limit ? base + ' ' + txt : current + txt;
  el.textContent = newText;
  // store base on first set
  if (!el.dataset.base) el.dataset.base = base || newText.slice(0, 200);
};
const safeGet = id => document.getElementById(id);

// ---------- Exported API (optional) ----------
export function definirDestination(lat, lon) {
  destination.latitude = lat;
  destination.longitude = lon;
  set('boussole', `Cap vers destination définie : ${lat.toFixed(6)}, ${lon.toFixed(6)}`);
}
export function setMasseSimulee(kg) {
  masseSimulee = Number(kg) || masseSimulee;
  afficherGrandeurs();
}
export function demarrerCockpit() {
  startAll();
}
export function arreterCockpit() {
  stopAll();
}
export function resetVitesseMax() {
  vitesseMax = 0;
  set('vitesse-max', 'Vitesse max : -- km/h (réinitialisée)');
}

// ---------- Start / Stop Orchestrators ----------
function startAll() {
  if (watchId !== null) return; // déjà démarré
  t0 = performance.now();

  // get current position once for faster first display
  if ('geolocation' in navigator) {
    navigator.geolocation.getCurrentPosition(
      pos => traiterPosition(pos.coords, pos.timestamp),
      err => set('gps', `Erreur GPS : ${err.message || err.code}`),
      { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 }
    );

    // then watch continuously
    try {
      watchId = navigator.geolocation.watchPosition(
        pos => traiterPosition(pos.coords, pos.timestamp),
        err => set('gps', `Erreur GPS : ${err.message || err.code}`),
        { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 }
      );
    } catch (err) {
      set('gps', `Erreur watchPosition : ${err.message}`);
    }
  } else {
    set('gps', 'Geolocation non supportée');
  }

  loopTemps();
  activerHorloge();
  synchroniserHeureAtomique();
  activerCapteurs();
  activerBoussole();
  afficherMedaillon(); // init medaillon and RAF loop
  chargerMeteo();
  afficherGrandeurs();
}

function stopAll() {
  if (watchId !== null && 'geolocation' in navigator) {
    navigator.geolocation.clearWatch(watchId);
  }
  watchId = null;
  positionPrecedente = null;
  vitesses = [];
  distanceTotale = 0;
  vitesseMax = 0;

  // stop audio analyser
  if (audioAnalyserHandle) { cancelAnimationFrame(audioAnalyserHandle); audioAnalyserHandle = null; }
  if (audioStream) {
    audioStream.getTracks().forEach(t => t.stop());
    audioStream = null;
  }
  // stop magnetometer
  if (magnetometerInstance && typeof magnetometerInstance.stop === 'function') {
    try { magnetometerInstance.stop(); } catch (e) {}
    magnetometerInstance = null;
  }
  // stop gyroscope
  if (gyroscopeInstance && typeof gyroscopeInstance.stop === 'function') {
    try { gyroscopeInstance.stop(); } catch (e) {}
    gyroscopeInstance = null;
  }
  // stop medaillon RAF
  if (medaillonRAF) { cancelAnimationFrame(medaillonRAF); medaillonRAF = null; }
}

// ---------- Time & relative timer ----------
function loopTemps() {
  const el = safeGet('temps');
  if (!el) return;
  function tick() {
    if (!t0) t0 = performance.now();
    const t = performance.now() - t0;
    el.textContent = `Temps : ${(t / 1000).toFixed(2)} s`;
    requestAnimationFrame(tick);
  }
  tick();
}

// ---------- GPS / position handling ----------
function traiterPosition(coords, timestamp) {
  // coords: GeolocationCoordinates
  const lat = coords.latitude;
  const lon = coords.longitude;
  const altitude = coords.altitude ?? 0;
  const accuracy = coords.accuracy ?? 0;
  const speed = (typeof coords.speed === 'number' && coords.speed !== null) ? coords.speed * 3.6 : null;

  // compute speed (km/h)
  const v = (speed !== null) ? speed : calculerVitesse(coords, timestamp);
  if (v != null && v >= 0 && v < 300) {
    vitesseMax = Math.max(vitesseMax, v);
    vitesses.push(v);

    const moy = vitesses.length ? (vitesses.reduce((a, b) => a + b, 0) / vitesses.length) : 0;
    const mps = v / 3.6;
    const mmps = mps * 1000;
    const dkm = distanceTotale / 1000;
    const ds = distanceTotale / 299792458;
    const dal = ds / (3600 * 24 * 365.25);

    set('vitesse', `Vitesse instantanée : ${v.toFixed(2)} km/h`);
    set('vitesse-moy', `Vitesse moyenne : ${moy.toFixed(2)} km/h`);
    set('vitesse-max', `Vitesse max : ${vitesseMax.toFixed(2)} km/h`);
    set('vitesse-ms', `Vitesse : ${mps.toFixed(2)} m/s | ${mmps.toFixed(0)} mm/s`);
    set('pourcentage', `% Lumière : ${(mps / 299792458 * 100).toExponential(2)}% | % Son : ${(mps / 343 * 100).toFixed(2)}%`);
    set('distance', `Distance : ${dkm.toFixed(3)} km | ${distanceTotale.toFixed(1)} m | ${(distanceTotale * 1000).toFixed(0)} mm`);
    set('distance-cosmique', `Distance cosmique : ${ds.toFixed(6)} s lumière | ${dal.toExponential(3)} al`);
    set('gps', `Lat ${lat.toFixed(6)} | Lon ${lon.toFixed(6)} | Alt ${altitude.toFixed(1)} m | Précision ${accuracy.toFixed(0)} m`);

    // store last position with timestamp for velocity calc and Minecraft coords
    positionPrecedente = { latitude: lat, longitude: lon, altitude: altitude, timestamp };

    // update medaillon + astro with current coords
    afficherMedaillon(lat, lon);
    afficherAstronomie(lat, lon);

    // update grandeurs (dependent on vitesse & masse)
    afficherGrandeurs();
  }
}

function calculerVitesse(coords, timestamp) {
  if (!positionPrecedente) {
    // set initial position and return 0
    positionPrecedente = { latitude: coords.latitude, longitude: coords.longitude, altitude: coords.altitude ?? 0, timestamp };
    return 0;
  }
  const dt = (timestamp - (positionPrecedente.timestamp || timestamp)) / 1000; // seconds
  if (dt <= 0) return 0;
  const d = calculerDistance(coords, positionPrecedente); // meters
  if (!Number.isFinite(d) || d < 0) return 0;
  distanceTotale += d;
  // update previous position for next calc
  positionPrecedente = { latitude: coords.latitude, longitude: coords.longitude, altitude: coords.altitude ?? 0, timestamp };
  return (d / dt) * 3.6; // km/h
}

// Haversine distance in meters
function calculerDistance(a, b) {
  const R = 6371e3;
  const φ1 = a.latitude * Math.PI / 180;
  const φ2 = b.latitude * Math.PI / 180;
  const Δφ = (b.latitude - a.latitude) * Math.PI / 180;
  const Δλ = (b.longitude - a.longitude) * Math.PI / 180;
  const sinΔφ2 = Math.sin(Δφ / 2);
  const sinΔλ2 = Math.sin(Δλ / 2);
  const aa = sinΔφ2 * sinΔφ2 + Math.cos(φ1) * Math.cos(φ2) * sinΔλ2 * sinΔλ2;
  const c = 2 * Math.atan2(Math.sqrt(aa), Math.sqrt(1 - aa));
  return R * c;
}

// ---------- Astro: SunCalc wrapper ----------
function afficherAstronomie(lat = 43.3, lon = 5.4) {
  try {
    const now = new Date();
    const times = SunCalc.getTimes(now, lat, lon);
    const sunPos = SunCalc.getPosition(now, lat, lon);
    const eqMin = computeEquationOfTimeMinutes(now); // more precise minute offset
    // solar noon and eq time displayed
    set('culmination-soleil', `☀️ Culmination : ${times.solarNoon?.toLocaleTimeString() ?? '--'}`);
    set('heure-vraie', `Heure solaire vraie (sun transit) : ${times.solarNoon?.toLocaleTimeString() ?? '--'}`);
    // mean solar time approximated by local clock:
    set('heure-moyenne', `Heure locale : ${now.toLocaleTimeString()}`);
    set('equation-temps', `Équation du temps : ${eqMin.toFixed(2)} min`);
  } catch (e) {
    set('culmination-soleil', '--');
    console.warn('Erreur astro', e);
  }
}

function computeEquationOfTimeMinutes(date = new Date()) {
  // approximate equation of time in minutes using SunCalc internals (or using simple approx).
  // We'll use formula based on Earth mean anomaly etc. (approx, good within a minute)
  // Source: NOAA approximate formula
  const rad = Math.PI / 180;
  const day = (Date.UTC(date.getUTCFullYear(), date.getUTCMonth(), date.getUTCDate()) - Date.UTC(date.getUTCFullYear(), 0, 0)) / 86400000;
  const B = 360 / 365 * (day - 81) * rad;
  const E = 9.87 * Math.sin(2 * B) - 7.53 * Math.cos(B) - 1.5 * Math.sin(B);
  return E; // minutes
}

// ---------- Médaillon (canvas) ----------
function afficherMedaillon(lat = 43.3, lon = 5.4) {
  const container = safeGet('medaillon');
  if (!container) return;

  // if canvas present reuse it
  let canvas = container.querySelector('canvas');
  if (!canvas) {
    canvas = document.createElement('canvas');
    canvas.width = 480;
    canvas.height = 480;
    canvas.style.display = 'block';
    canvas.style.margin = '0 auto';
    container.innerHTML = '';
    container.appendChild(canvas);
  }
  const ctx = canvas.getContext('2d');

  // animation loop draw
  const draw = () => {
    const now = new Date();
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    // background
    ctx.fillStyle = '#000';
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    const cx = canvas.width / 2;
    const cy = canvas.height / 2;
    const r = Math.min(canvas.width, canvas.height) * 0.42;

    // sky circle
    ctx.strokeStyle = '#444';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.stroke();

    // graticule
    ctx.strokeStyle = '#222';
    ctx.lineWidth = 1;
    for (let a = 0; a < 360; a += 30) {
      const radA = a * Math.PI / 180;
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.lineTo(cx + Math.cos(radA) * r, cy + Math.sin(radA) * r);
      ctx.stroke();
    }

    // Sun & Moon positions using SunCalc
    try {
      const sunPos = SunCalc.getPosition(now, lat, lon);
      const moonPos = SunCalc.getMoonPosition(now, lat, lon);
      // convert azimuth/altitude to canvas polar coords (simple projection)
      const azimuthToCanvas = az => -az + Math.PI / 2;
      const proj = (pos, radiusFactor = 0.65) => {
        // pos.azimuth (radians), pos.altitude (radians)
        const az = azimuthToCanvas(pos.azimuth);
        const alt = pos.altitude; // [-pi/2, pi/2]
        // altitude -> radius: high altitude close to center
        const radial = r * radiusFactor * (1 - (alt / (Math.PI / 2))); // alt high -> small radial
        return { x: cx + Math.cos(az) * radial, y: cy + Math.sin(az) * radial };
      };

      // Sun
      const sp = proj(sunPos, 0.7);
      ctx.fillStyle = '#ffd700';
      ctx.beginPath();
      ctx.arc(sp.x, sp.y, 8, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = '#fff';
      ctx.font = '12px monospace';
      ctx.fillText('☀️ Soleil', sp.x + 12, sp.y - 6);

      // Moon
      const mp = proj(moonPos, 0.6);
      ctx.fillStyle = '#ddd';
      ctx.beginPath();
      ctx.arc(mp.x, mp.y, 6, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = '#fff';
      ctx.fillText('🌙 Lune', mp.x - 24, mp.y - 10);
    } catch (e) {
      // fallthrough: if SunCalc fails
    }

    // random-ish star field (seeded by date so subtle variation)
    ctx.fillStyle = '#0ff';
    const seed = Math.floor((new Date()).getTime() / (1000 * 60 * 60));
    for (let i = 0; i < 40; i++) {
      const x = (Math.abs(Math.sin(seed + i * 17.13)) * (canvas.width - 40)) + 20;
      const y = (Math.abs(Math.cos(seed + i * 13.7)) * (canvas.height - 40)) + 20;
      ctx.beginPath();
      ctx.arc(x, y, Math.abs(Math.sin(i * 7.3)) * 1.8 + 0.8, 0, Math.PI * 2);
      ctx.fill();
    }

    // annotate center
    ctx.fillStyle = '#fff';
    ctx.font = '11px monospace';
    ctx.fillText(`Lat: ${lat.toFixed(4)} Lon: ${lon.toFixed(4)}`, 10, canvas.height - 10);

    medaillonRAF = requestAnimationFrame(draw);
  };

  // start RAF if not running
  if (!medaillonRAF) medaillonRAF = requestAnimationFrame(draw);
}

// ---------- Astronomical details (sun, moon times, moon phase) ----------
function afficherAstronomie(lat = 43.3, lon = 5.4) {
  try {
    const now = new Date();
    const sunTimes = SunCalc.getTimes(now, lat, lon);
    const moonTimes = SunCalc.getMoonTimes(now, lat, lon);
    const moonPos = SunCalc.getMoonPosition(now, lat, lon);
    const moonIll = SunCalc.getMoonIllumination(now);

    set('culmination-soleil', `☀️ Culmination : ${sunTimes.solarNoon?.toLocaleTimeString() ?? '--'}`);
    set('heure-vraie', `Heure solaire vraie (sun transit) : ${sunTimes.solarNoon?.toLocaleTimeString() ?? '--'}`);
    set('heure-moyenne', `Heure locale : ${now.toLocaleTimeString()}`);
    set('equation-temps', `Équation du temps (approx) : ${computeEquationOfTimeMinutes(now).toFixed(2)} min`);
    set('lune-phase', `🌗 Phase : ${(moonIll.fraction * 100).toFixed(2)}%`);
    set('lune-mag', `🔭 Altitude (rad) : ${moonPos.altitude?.toFixed(3) ?? '--'}`);
    set('lune-lever', `🌙 Lever : ${moonTimes.rise ? moonTimes.rise.toLocaleTimeString() : '--'}`);
    set('lune-coucher', `🌙 Coucher : ${moonTimes.set ? moonTimes.set.toLocaleTimeString() : '--'}`);
    set('lune-culmination', `${moonTimes.alwaysUp ? 'Toujours visible' : moonTimes.alwaysDown ? 'Invisible' : '--'}`);
  } catch (e) {
    console.warn('Erreur afficherAstronomie', e);
  }
}

// ---------- Horloge & atomique ----------
function activerHorloge() {
  const el = safeGet('horloge-minecraft');
  if (!el) return;
  function tick() {
    const d = new Date();
    el.textContent = `${String(d.getHours()).padStart(2, '0')}:${String(d.getMinutes()).padStart(2, '0')}:${String(d.getSeconds()).padStart(2, '0')}`;
    requestAnimationFrame(tick);
  }
  tick();
}

function synchroniserHeureAtomique() {
  fetch('https://worldtimeapi.org/api/ip')
    .then(r => r.json())
    .then(data => {
      const utc = new Date(data.utc_datetime);
      set('horloge-atomique', `Heure atomique (UTC) : ${utc.toLocaleTimeString()}`);
    })
    .catch(() => set('horloge-atomique', 'Heure atomique indisponible'));
}

// ---------- Capteurs (robustes, évitent duplication infinie) ----------
function activerCapteurs() {
  const el = safeGet('capteurs');
  if (!el) return;
  // initialize base
  el.dataset.base = 'Lumière : -- lux | Son : -- dB | Niveau : --° | Gyro : -- | Magnétomètre : -- µT | Batterie : --% | Réseau : --';
  el.textContent = el.dataset.base;

  // Ambient Light
  if ('AmbientLightSensor' in window) {
    try {
      const light = new AmbientLightSensor();
      light.addEventListener('reading', () => {
        updateCapteurField('Lumière', `${Math.round(light.illuminance)} lux`);
      });
      light.start();
    } catch (err) { console.warn('AmbientLightSensor error', err); }
  } else {
    // fallback: no standard fallback available — leave placeholder
  }

  // Microphone analyser (update "Son" field)
  if (navigator.mediaDevices?.getUserMedia) {
    navigator.mediaDevices.getUserMedia({ audio: true }).then(stream => {
      audioStream = stream;
      try {
        const AudioCtx = window.AudioContext || window.webkitAudioContext;
        const ctx = new AudioCtx();
        const analyser = ctx.createAnalyser();
        analyser.fftSize = 2048;
        const src = ctx.createMediaStreamSource(stream);
        src.connect(analyser);
        const data = new Uint8Array(analyser.frequencyBinCount);

        const audioLoop = () => {
          analyser.getByteFrequencyData(data);
          const avg = data.reduce((a, b) => a + b, 0) / data.length || 1;
          const dB = 20 * Math.log10(avg);
          updateCapteurField('Son', `${Math.round(dB)} dB`);
          audioAnalyserHandle = requestAnimationFrame(audioLoop);
        };
        audioLoop();
      } catch (err) {
        console.warn('Audio analyser error', err);
      }
    }).catch(err => {
      console.warn('Microphone access denied or unavailable', err);
    });
  }

  // Device orientation -> Nivel à bulle
  if ('DeviceOrientationEvent' in window) {
    deviceOrientationHandler = e => {
      if (typeof e.beta === 'number') updateCapteurField('Niveau', `${e.beta.toFixed(1)}°`);
      if (typeof e.alpha === 'number') updateCapteurField('Gyro', `${e.alpha.toFixed(1)}°`);
    };
    // some browsers require permission (iOS); we simply attach handler
    window.addEventListener('deviceorientation', deviceOrientationHandler);
  }

  // Magnetometer (if available)
  if ('Magnetometer' in window) {
    try {
      magnetometerInstance = new Magnetometer({ frequency: 10 });
      magnetometerInstance.addEventListener('reading', () => {
        const champ = Math.sqrt(magnetometerInstance.x ** 2 + magnetometerInstance.y ** 2 + magnetometerInstance.z ** 2);
        updateCapteurField('Magnétomètre', `${Math.round(champ)} µT`);
      });
      magnetometerInstance.start();
    } catch (err) {
      console.warn('Magnetometer failed to start', err);
    }
  }

  // Gyroscope (if available)
  if ('Gyroscope' in window) {
    try {
      gyroscopeInstance = new Gyroscope({ frequency: 10 });
      gyroscopeInstance.addEventListener('reading', () => {
        updateCapteurField('Gyro', `${gyroscopeInstance.x.toFixed(1)},${gyroscopeInstance.y.toFixed(1)},${gyroscopeInstance.z.toFixed(1)}`);
      });
      gyroscopeInstance.start();
    } catch (err) {
      console.warn('Gyroscope start failed', err);
    }
  }

  // Battery
  if ('getBattery' in navigator) {
    navigator.getBattery().then(b => {
      const updateBattery = () => updateCapteurField('Batterie', `${Math.round(b.level * 100)}%`);
      updateBattery();
      b.addEventListener('levelchange', updateBattery);
    });
  }

  // Network (navigator.connection)
  if (navigator.connection) {
    const conn = navigator.connection;
    const updateConn = () => updateCapteurField('Réseau', `${conn.effectiveType ?? conn.type ?? '—'}`);
    updateConn();
    conn.addEventListener('change', updateConn);
  }

  // helper updates specific field inside capteurs element by replacing pattern
  function updateCapteurField(label, newValue) {
    const el = safeGet('capteurs');
    if (!el) return;
    // el.dataset.base guaranteed above
    const base = el.dataset.base;
    // parse base into fields map or reconstruct by token
    // simple approach: keep a JSON map in data-map to avoid brittle regex
    if (!el.dataset.map) {
      // initialize map keys from base template
      const keys = ['Lumière', 'Son', 'Niveau', 'Gyro', 'Magnétomètre', 'Batterie', 'Réseau'];
      const map = {};
      keys.forEach(k => map[k] = '--');
      el.dataset.map = JSON.stringify(map);
    }
    const map = JSON.parse(el.dataset.map);
    map[label] = newValue;
    el.dataset.map = JSON.stringify(map);

    // compose display string
    const display = `Lumière : ${map['Lumière']} | Son : ${map['Son']} | Niveau : ${map['Niveau']} | Gyro : ${map['Gyro']} | Magnétomètre : ${map['Magnétomètre']} | Batterie : ${map['Batterie']} | Réseau : ${map['Réseau']}`;
    el.textContent = display;
  }
}

// ---------- Météo & Qualité de l'air ----------
function chargerMeteo(lat = 43.3, lon = 5.4) {
  const meteoEl = safeGet('meteo');
  const airEl = safeGet('qualite-air');
  if
