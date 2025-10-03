let watchId = null;
let positionPrecedente = null;
let vitesses = [];
let vitesseMax = 0;
let distanceTotale = 0;

const set = (id, txt) => document.getElementById(id).textContent = txt;

document.getElementById('marche').onclick = () => {
  if (!('geolocation' in navigator)) return set('vitesse', 'GPS non disponible');
  if (watchId !== null) return;

  navigator.geolocation.getCurrentPosition(pos => traiter(pos.coords, pos.timestamp));
  watchId = navigator.geolocation.watchPosition(
    pos => traiter(pos.coords, pos.timestamp),
    err => set('vitesse', 'Erreur GPS'),
    { enableHighAccuracy: true }
  );
};

document.getElementById('stop').onclick = () => {
  if (watchId !== null) {
    navigator.geolocation.clearWatch(watchId);
    watchId = null;
    set('vitesse', '⏹️ Suivi arrêté');
  }
};

document.getElementById('reset').onclick = () => {
  vitesses = [];
  vitesseMax = 0;
  distanceTotale = 0;
  positionPrecedente = null;
  set('vitesse', 'Vitesse : --');
  set('distance', 'Distance : --');
  set('pourcentage', '% Lumière / Son : --');
};

function traiter(coords, timestamp) {
  const v = coords.speed != null ? coords.speed * 3.6 : calculerVitesse(coords, timestamp);
  if (v >= 0 && v < 300) {
    vitesseMax = Math.max(vitesseMax, v);
    vitesses.push(v);
    const moy = vitesses.reduce((a, b) => a + b, 0) / vitesses.length;
    set('vitesse', `Vitesse : ${v.toFixed(2)} km/h | Moy : ${moy.toFixed(2)} | Max : ${vitesseMax.toFixed(2)}`);
    set('distance', `Distance : ${(distanceTotale / 1000).toFixed(3)} km`);
    const mps = v / 3.6;
    set('pourcentage', `% Lumière : ${(mps / 299792458 * 100).toExponential(2)}% | % Son : ${(mps / 343 * 100).toFixed(2)}%`);
  }
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
