let watchId = null, positionPrecedente = null, vitesses = [], vitesseMax = 0, distanceTotale = 0, t0 = null;
let modeSouterrain = false;

const set = (id, txt) => document.getElementById(id).textContent = txt;

document.getElementById('marche').onclick = () => {
  if (watchId !== null) return;
  t0 = performance.now();
  navigator.geolocation.getCurrentPosition(
    pos => traiterPosition(pos.coords, pos.timestamp),
    err => {
      modeSouterrain = true;
      set('gps', 'Mode souterrain activé 🌑');
    }
  );
  watchId = navigator.geolocation.watchPosition(
    pos => traiterPosition(pos.coords, pos.timestamp),
    err => {
      modeSouterrain = true;
      set('gps', 'Mode souterrain activé 🌑');
    },
    { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 }
  );
  loopTemps(); activerCapteurs();
};

document.getElementById('stop').onclick = () => {
  if (watchId !== null) {
    navigator.geolocation.clearWatch(watchId);
    watchId = null;
    set('vitesse', '⏹️ Suivi arrêté');
  }
};

document.getElementById('reset').onclick = () => {
  vitesses = []; vitesseMax = 0; distanceTotale = 0; positionPrecedente = null;
  ['vitesse','vitesse-moy','vitesse-max','vitesse-ms','pourcentage','distance','distance-cosmique','gps'].forEach(id => set(id, '--'));
};

function loopTemps() {
  const el = document.getElementById('temps');
  function tick() {
    const t = performance.now() - t0;
    el.textContent = `EtTemps : ${(t / 1000).toFixed(2)} s`;
    requestAnimationFrame(tick);
  }
  tick();
}

function traiterPosition(coords, timestamp) {
  if (modeSouterrain || !coords.latitude || !coords.longitude || coords.accuracy > 1000) {
    coords.latitude = 43.3; coords.longitude = 5.4;
    set('gps', 'Mode souterrain activé 🌑');
  } else {
    set('gps', `Précision GPS : ${(coords.accuracy ?? 0).toFixed(0)}%`);
  }

  let v = coords.speed != null ? coords.speed * 3.6 : calculerVitesse(coords, timestamp);
  if (!isFinite(v) || v < 0) v = 0;

  vitesseMax = Math.max(vitesseMax, v);
  vitesses.push(v);
  const moy = vitesses.reduce((a, b) => a + b, 0) / vitesses.length;
  const mps = v / 3.6;
  const mmps = mps * 1000;
  const dkm = distanceTotale / 1000;
  const ds = distanceTotale / 299792458;
  const dal = ds / (3600 * 24 * 365.25);
  const pctLumiere = (mps / 299792458 * 100).toExponential(2);
  const pctSon = (mps / 343 * 100).toFixed(2);

  set('vitesse', `Vitesse instantanée : ${v.toFixed(2)} km/h`);
  set('vitesse-moy', `Vitesse moyenne : ${moy.toFixed(2)} km/h`);
  set('vitesse-max', `Vitesse max : ${vitesseMax.toFixed(2)} km/h`);
  set('vitesse-ms', `Vitesse : ${mps.toFixed(2)} m/s | ${mmps.toFixed(0)} mm/s`);
  set('pourcentage', `% Lumière : ${pctLumiere}% | % Son : ${pctSon}%`);
  set('distance', `Distance : ${dkm.toFixed(3)} km | ${distanceTotale.toFixed(1)} m | ${(distanceTotale * 1000).toFixed(0)} mm`);
  set('distance-cosmique', `Distance cosmique : ${ds.toFixed(6)} s lumière | ${dal.toExponential(3)} al`);

  if (mps > 299792458) {
    document.body.style.background = 'radial-gradient(circle, #ff00ff, #000)';
    set('vitesse', `🚀 Vitesse supraluminique : ${v.toFixed(2)} km/h`);
  }

  positionPrecedente = { ...coords, timestamp };
}

function calculerVitesse(c, t) {
  if (!positionPrecedente) return 0;
  const dt = (t - positionPrecedente.timestamp) / 1000;
  const d = calculerDistance(c, positionPrecedente);
  distanceTotale += d;
  return dt > 0 ? (d / dt) * 3.6 : 0;
}

function calculerDistance(a, b) {
  const R = 6371e3;
  const φ1 = a.latitude * Math.PI / 180;
  const φ2 = b.latitude * Math.PI / 180;
  const Δφ = (b.latitude - a.latitude) * Math.PI / 180;
  const Δλ = (b.longitude - a.longitude) * Math.PI / 180;
  const aVal = Math.sin(Δφ / 2) ** 2 + Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ / 2) ** 2;
  const c = 2 * Math.atan2(Math.sqrt(aVal), Math.sqrt(1 - aVal));
  return R * c;
}

function activerCapteurs() {
  if ('AmbientLightSensor' in window) {
    try {
      const light = new AmbientLightSensor();
      light.addEventListener('reading', () => {
        const pct = Math.min(100, Math.round(light.illuminance / 100));
        const txt = document.getElementById('pourcentage').textContent;
        set('pourcentage', txt.replace(/% Lumière : .*?%/, `% Lumière : ${pct}%`));
      });
      light.start();
    } catch {}
  }

  if (navigator.mediaDevices?.getUserMedia) {
    navigator.mediaDevices.getUserMedia({ audio: true }).then(stream => {
      const ctx = new AudioContext();
      const analyser = ctx
  function initialiserGPS() {
  if (!navigator.geolocation) {
    set('gps', '🌐 GPS non disponible sur ce navigateur');
    return;
  }

  // Lecture initiale
  navigator.geolocation.getCurrentPosition(
    pos => traiterPosition(pos.coords, pos.timestamp),
    err => {
      modeSouterrain = true;
      set('gps', 'Mode souterrain activé 🌑');
    }
  );

  // Suivi en continu
  watchId = navigator.geolocation.watchPosition(
    pos => traiterPosition(pos.coords, pos.timestamp),
    err => {
      modeSouterrain = true;
      set('gps', 'Mode souterrain activé 🌑');
    },
    {
      enableHighAccuracy: true,
      maximumAge: 0,
      timeout: 10000
    }
  );
  }
      
