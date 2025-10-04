let watchId = null, positionPrecedente = null, vitesses = [], vitesseMax = 0, distanceTotale = 0, t0 = null;
let modeSouterrain = false;
let boucleActive = false;

const set = (id, txt) => document.getElementById(id).textContent = txt;

document.getElementById('toggle').onclick = () => {
  if (watchId === null) {
    t0 = performance.now();
    initialiserGPS();
    boucle24Hz();
    activerCapteurs();
    document.getElementById('toggle').textContent = '⏹️ Arrêt';
  } else {
    navigator.geolocation.clearWatch(watchId);
    watchId = null;
    boucleActive = false;
    document.getElementById('toggle').textContent = '▶️ Marche';
    set('vitesse', '⏹️ Suivi arrêté');
  }
};

document.getElementById('reset').onclick = () => {
  vitesses = []; vitesseMax = 0; distanceTotale = 0; positionPrecedente = null;
  ['vitesse','vitesse-moy','vitesse-max','vitesse-ms','pourcentage','distance','distance-cosmique','gps'].forEach(id => set(id, '--'));
};

function boucle24Hz() {
  boucleActive = true;
  function tick() {
    if (!boucleActive) return;
    const t = performance.now() - t0;
    set('temps', `EtTemps : ${(t / 1000).toFixed(2)} s`);
    if (positionPrecedente) traiterPosition(positionPrecedente, performance.now());
    setTimeout(tick, 1000 / 24);
  }
  tick();
}

function initialiserGPS() {
  if (!navigator.geolocation) {
    set('gps', '🌐 GPS non disponible');
    modeSouterrain = true;
    return;
  }
  navigator.geolocation.getCurrentPosition(
    pos => traiterPosition(pos.coords, pos.timestamp),
    err => activerModeSouterrain()
  );
  watchId = navigator.geolocation.watchPosition(
    pos => { positionPrecedente = pos.coords; },
    err => activerModeSouterrain(),
    { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 }
  );
}

function activerModeSouterrain() {
  modeSouterrain = true;
  set('gps', 'Mode souterrain activé 🌑');
  document.body.style.background = 'radial-gradient(circle, #222, #000)';
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
  const c = 2 * Math
