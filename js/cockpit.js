import * as SunCalc from 'https://cdn.jsdelivr.net/npm/suncalc@1.9.0/suncalc.js';

let watchId = null, positionPrecedente = null, vitesses = [], vitesseMax = 0, distanceTotale = 0, t0 = null;
const applicateurs = { physique: true, chimie: true, svt: true };
let rituelActif = true;

const set = (id, txt) => { const el = document.getElementById(id); if (el) el.textContent = txt; };
const safeSetText = set;

document.getElementById('marche').onclick = () => {
  if (watchId !== null) return;
  t0 = performance.now();
  navigator.geolocation.getCurrentPosition(
    pos => traiterPosition(pos.coords, pos.timestamp),
    err => safeSetText('gps', 'Erreur GPS: ' + err.message)
  );
  watchId = navigator.geolocation.watchPosition(
    pos => traiterPosition(pos.coords, pos.timestamp),
    err => safeSetText('gps', 'Erreur GPS: ' + err.message),
    { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 }
  );
  loopTemps(); activerHorloge(); synchroniserHeureAtomique(); activerCapteurs(); afficherMedaillon(); chargerGrandeurs(); chargerMeteo();
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

document.getElementById('toggle-rituel').onclick = () => {
  rituelActif = !rituelActif;
  document.body.classList.toggle('rituel-off', !rituelActif);
  set('toggle-rituel', rituelActif ? '✨ Rituel cosmique : ON' : '🔋 Rituel cosmique : OFF');
};

['physique','chimie','svt'].forEach(type => {
  document.getElementById(`toggle-${type}`).onclick = () => {
    applicateurs[type] = !applicateurs[type];
    set(`toggle-${type}`, `${type === 'physique' ? '🧲' : type === 'chimie' ? '🧪' : '🌱'} ${type.charAt(0).toUpperCase() + type.slice(1)} : ${applicateurs[type] ? 'ON' : 'OFF'}`);
  };
});

function loopTemps() {
  const el = document.getElementById('temps');
  function tick() {
    const t = performance.now() - t0;
    el.textContent = `Temps : ${(t / 1000).toFixed(2)} s`;
    requestAnimationFrame(tick);
  }
  tick();
}

function traiterPosition(coords, timestamp) {
  const v = coords.speed != null ? coords.speed * 3.6 : calculerVitesse(coords, timestamp);
  if (v >= 0 && v < 300) {
    vitesseMax = Math.max(vitesseMax, v);
    vitesses.push(v);
    const moy = vitesses.reduce((a, b) => a + b, 0) / vitesses.length;
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
    set('gps', `Précision GPS : ${(coords.accuracy ?? 0).toFixed(0)}%`);
    positionPrecedente = { ...coords, timestamp };
    afficherMedaillon(coords.latitude, coords.longitude);
    chargerGrandeurs();
  }
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

function afficherMedaillon(lat = 43.3, lon = 5.4) {
  const now = new Date();
  const soleilTimes = SunCalc.getTimes(now, lat, lon);
  const luneTimes = SunCalc.getMoonTimes(now, lat, lon);
  const lune = SunCalc.getMoonPosition(now, lat, lon);
  const phase = SunCalc.getMoonIllumination(now);
  const eqTemps = (soleilTimes.sunset - soleilTimes.sunrise) / 60000 - 720;

  set('culmination-soleil', `☀️ Culmination : ${soleilTimes.solarNoon.toLocaleTimeString()}`);
  set('heure-vraie', `Heure solaire vraie : ${soleilTimes.sunrise.toLocaleTimeString()}`);
  set('heure-moyenne', `Heure solaire moyenne : ${soleilTimes.sunset.toLocaleTimeString()}`);
  set('equation-temps', `Équation du temps : ${eqTemps.toFixed(2)} min`);
  set('lune-phase', `🌗 Phase : ${(phase.fraction * 100).toFixed(2)}%`);
  set('lune-mag', `🔭 Altitude : ${lune.altitude.toFixed(2)} rad`);
  set('lune-lever', `🌙 Lever : ${luneTimes.rise?.toLocaleTimeString() ?? '--'}`);
  set('lune-coucher', `🌙 Coucher : ${luneTimes.set?.toLocaleTimeString() ?? '--'}`);
  set('lune-culmination', `🌙 Culmination : ${luneTimes.alwaysUp ? 'Toujours visible' : luneTimes.alwaysDown ? 'Invisible' : '--'}`);
}

function activerHorloge() {
  const h = document.getElementById('horloge-minecraft');
  function tick() {
    const d = new Date();
    h.textContent = `${String(d.getHours()).padStart(2, '0')}:${String(d.getMinutes()).padStart(2, '0')}:${String(d.getSeconds()).padStart(2, '0')}`;
    requestAnimationFrame(tick);
  }
  tick();
}

function synchroniserHeureAtomique() {
  fetch('https://worldtime
