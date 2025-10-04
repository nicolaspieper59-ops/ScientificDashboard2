let watchId = null;
let positionPrecedente = null;
let vitesses = [];
let vitesseMax = 0;
let distanceTotale = 0;
let suiviActif = false;
let t0 = null;

const set = (id, txt) => {
  const el = document.getElementById(id);
  if (el) el.textContent = txt;
};

document.getElementById('toggle').onclick = () => {
  if (!('geolocation' in navigator)) return set('gps', '🌐 GPS non disponible');

  if (!suiviActif) {
    t0 = performance.now();
    navigator.geolocation.getCurrentPosition(pos => traiter(pos.coords, pos.timestamp));
    watchId = navigator.geolocation.watchPosition(
      pos => traiter(pos.coords, pos.timestamp),
      err => set('gps', 'Erreur GPS'),
      { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 }
    );
    suiviActif = true;
    document.getElementById('toggle').textContent = '⏹️ Arrêt';
    boucleTemps();
  } else {
    navigator.geolocation.clearWatch(watchId);
    watchId = null;
    suiviActif = false;
    document.getElementById('toggle').textContent = '▶️ Marche';
    set('vitesse', '⏹️ Suivi arrêté');
  }
};

document.getElementById('reset').onclick = () => {
  vitesses = [];
  vitesseMax = 0;
  distanceTotale = 0;
  positionPrecedente = null;
  set('vitesse', 'Vitesse instantanée : -- km/h');
  set('vitesse-moy', 'Vitesse moyenne : -- km/h');
  set('vitesse-max', 'Vitesse max : -- km/h');
  set('vitesse-ms', 'Vitesse : -- m/s | -- mm/s');
  set('pourcentage', '% Lumière : --% | % Son : --%');
  set('distance', 'Distance : -- km | -- m | -- mm');
  set('distance-cosmique', 'Distance cosmique : -- s lumière | -- al');
  set('gps', 'Précision GPS : -- m');
};

function boucleTemps() {
  function tick() {
    if (!suiviActif) return;
    const t = performance.now() - t0;
    set('temps', `Temps : ${(t / 1000).toFixed(2)} s`);
    requestAnimationFrame(tick);
  }
  tick();
}

function traiter(coords, timestamp) {
  const v = coords.speed != null ? coords.speed * 3.6 : calculerVitesse(coords, timestamp);
  if (!isFinite(v) || v < 0) return;

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
  set('gps', `Précision GPS : ${coords.accuracy?.toFixed(1) ?? '--'} m`);

  positionPrecedente = { ...coords, timestamp };
}

function calculerVitesse(c, t) {
  if (!positionPrecedente) {
    positionPrecedente = { ...c, timestamp: t };
    return 0;
  }
  const dt = (t - positionPrecedente.timestamp) / 1000;
  const d = calculerDistance(c, positionPrecedente);
  distanceTotale += d;
  positionPrecedente = { ...c, timestamp: t };
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
  
