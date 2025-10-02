import * as SunCalc from 'https://cdn.jsdelivr.net/npm/suncalc@1.9.0/suncalc.js';

let watchId = null;
let positionPrecedente = null;
let vitesses = [];
let vitesseMax = 0;
let distanceTotale = 0;
let t0 = null;
let destination = { latitude: null, longitude: null };

function set(id, txt) {
  const el = document.getElementById(id);
  if (el) el.textContent = txt;
}

document.getElementById('marche').onclick = () => {
  if (watchId !== null) return;
  t0 = performance.now();
  navigator.geolocation.getCurrentPosition(pos => traiter(pos.coords, pos.timestamp));
  watchId = navigator.geolocation.watchPosition(pos => traiter(pos.coords, pos.timestamp), err => set('gps', 'Erreur GPS'), { enableHighAccuracy: true });
  loopTemps();
  afficherMedaillon();
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
  set('vitesse', '--');
  set('distance', '--');
  set('pourcentage', '--');
};

function loopTemps() {
  const el = document.getElementById('temps');
  function tick() {
    const t = performance.now() - t0;
    el.textContent = `Temps : ${(t / 1000).toFixed(2)} s`;
    requestAnimationFrame(tick);
  }
  tick();
}

function traiter(coords, timestamp) {
  const v = coords.speed != null ? coords.speed * 3.6 : calculerVitesse(coords, timestamp);
  if (v >= 0 && v < 300) {
    vitesseMax = Math.max(vitesseMax, v);
    vitesses.push(v);
    const moy = vitesses.reduce((a, b) => a + b, 0) / vitesses.length;
    const mps = v / 3.6;
    const mmps = mps * 1000;
    set('vitesse', `Vitesse instantanée : ${v.toFixed(2)} km/h`);
    set('vitesse-moy', `Vitesse moyenne : ${moy.toFixed(2)} km/h`);
    set('vitesse-max', `Vitesse max : ${vitesseMax.toFixed(2)} km/h`);
    set('vitesse-ms', `Vitesse : ${mps.toFixed(2)} m/s | ${mmps.toFixed(0)} mm/s`);
    set('pourcentage', `% Lumière : ${(mps / 299792458 * 100).toExponential(2)}% | % Son : ${(mps / 343 * 100).toFixed(2)}%`);
    const dkm = distanceTotale / 1000;
    const ds = distanceTotale / 299792458;
    const dal = ds / (3600 * 24 * 365.25);
    set('distance', `Distance : ${dkm.toFixed(3)} km | ${(distanceTotale).toFixed(1)} m | ${(distanceTotale * 1000).toFixed(0)} mm`);
    set('distance-cosmique', `Distance cosmique : ${ds.toFixed(6)} s lumière | ${dal.toExponential(3)} al`);
    set('gps', `Précision GPS : ${(coords.accuracy ?? 0).toFixed(0)}%`);
    afficherVueBrute(coords);
    afficherMedaillon();
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

function afficherVueBrute(g) {
  set('gps-brut', `Latitude : ${g.latitude.toFixed(6)} | Longitude : ${g.longitude.toFixed(6)} | Altitude : ${(g.altitude ?? 0).toFixed(1)} m`);
}

function afficherMedaillon() {
  const lat = positionPrecedente?.latitude ?? 43.3;
  const lon = positionPrecedente?.longitude ?? 5.4;
  const now = new Date();
  const soleil = SunCalc.getPosition(now, lat, lon);
  const lune = SunCalc.getMoonPosition(now, lat, lon);
  const phase = SunCalc.getMoonIllumination(now);
  const times = SunCalc.getMoonTimes(now, lat, lon);
  const solarTimes = SunCalc.getTimes(now, lat, lon);

  set('culmination-soleil', `Culmination soleil : ${solarTimes.solarNoon.toLocaleTimeString()}`);
  set('heure-vraie', `Heure solaire vraie : ${solarTimes.sunrise.toLocaleTimeString()}`);
  set('heure-moyenne', `Heure solaire moyenne : ${solarTimes.sunset.toLocaleTimeString()}`);
  const eqTemps = (solarTimes.sunset - solarTimes.sunrise) / 60000 - 720;
  set('equation-temps', `Équation du temps : ${eqTemps.toFixed(2)} min`);

  set('lune-phase', `Lune phase : ${(phase.fraction * 100).toFixed(1)}%`);
  set('lune-mag', `Lune magnitude : ${lune.altitude.toFixed(2)} rad`);
  set('lune-lever', `Lever lune : ${times.rise?.toLocaleTimeString() ?? '--'}`);
  set('lune-coucher', `Coucher lune : ${times.set?.toLocaleTimeString() ?? '--'}`);
  set('lune-culmination', `Culmination lune : ${times.alwaysUp ? 'Toujours visible' : times.alwaysDown ? 'Invisible' : '--'}`);
    }
    
