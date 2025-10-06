// ===================== ÉTAT GLOBAL =====================
let watchId = null;
let positionPrecedente = null;
let vitesses = [];
let distanceTotale = 0;
let vitesseMax = 0;
let tempsDebut = null;
let vitesseVerticaleAcc = 0;
let accelPrecedente = null;
let altitudeBaro = null;

const R_TERRE = 6371e3;
const P0 = 1013.25; // hPa référence niveau mer

// ===================== OUTILS =====================
const setText = (id, txt) => {
  const e = document.getElementById(id);
  if (e) e.textContent = txt;
};

function haversine(p1, p2) {
  const φ1 = p1.coords.latitude * Math.PI / 180;
  const φ2 = p2.coords.latitude * Math.PI / 180;
  const Δφ = (p2.coords.latitude - p1.coords.latitude) * Math.PI / 180;
  const Δλ = (p2.coords.longitude - p1.coords.longitude) * Math.PI / 180;
  const a = Math.sin(Δφ / 2) ** 2 + Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ / 2) ** 2;
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
  return R_TERRE * c;
}

// ===================== ALTITUDE (GPS + BARO) =====================
let lastAltitude = null;

function altitudeBarometrique(pression_hPa) {
  // Formule internationale de l’atmosphère
  // h = 44330 * (1 - (p / P0)^(1/5.255))
  return 44330 * (1 - Math.pow(pression_hPa / P0, 1 / 5.255));
}

function altitudeFiltree(alt_gps, pression) {
  let alt = alt_gps;
  if (pression) {
    altitudeBaro = altitudeBarometrique(pression);
    alt = (alt_gps && altitudeBaro)
      ? (alt_gps * 0.5 + altitudeBaro * 0.5)
      : (altitudeBaro || alt_gps);
  }
  if (lastAltitude === null) lastAltitude = alt;
  // Filtre passe-bas pour lisser
  const filtrée = 0.8 * lastAltitude + 0.2 * alt;
  lastAltitude = filtrée;
  return filtrée;
}

// ===================== GPS 3D =====================
function vitesse3D(pos, prec, pression) {
  if (!prec) return { total: 0, horiz: 0, vert: 0 };
  const dt = (pos.timestamp - prec.timestamp) / 1000;
  if (dt <= 0) return { total: 0, horiz: 0, vert: 0 };

  const dh = haversine(pos, prec);
  let alt1 = altitudeFiltree(pos.coords.altitude ?? 0, pression);
  let alt2 = altitudeFiltree(prec.coords.altitude ?? 0, pression);
  let dv = alt1 - alt2;
  if (Math.abs(dv) < 0.5) dv = 0; // ignore petites variations

  const vh = dh / dt;
  const vv = dv / dt;
  const vt = Math.sqrt(vh ** 2 + vv ** 2);
  return { total: vt, horiz: vh, vert: vv };
}

// ===================== ACCÉLÉROMÈTRE =====================
function miseAJourVitesseVerticale(accel) {
  const dt = 0.05;
  if (accelPrecedente) {
    vitesseVerticaleAcc += ((accel.y || 0) + (accelPrecedente.y || 0)) / 2 * dt;
    vitesseVerticaleAcc *= 0.98; // filtre drift
  }
  accelPrecedente = accel;
  return vitesseVerticaleAcc;
}

// ===================== MISE À JOUR =====================
function miseAJourPosition(pos) {
  const pression = window.lastPressure ?? null; // prise du capteur si dispo
  const v = vitesse3D(pos, positionPrecedente, pression);
  if (positionPrecedente) distanceTotale += haversine(pos, positionPrecedente);
  positionPrecedente = pos;

  const vLissée = vitesses.length ? 0.8 * vitesses[vitesses.length - 1] + 0.2 * v.total : v.total;
  vitesses.push(vLissée);
  if (vitesses.length > 60) vitesses.shift();
  vitesseMax = Math.max(vitesseMax, vLissée * 3.6);
  const moyenne = vitesses.reduce((a, b) => a + b, 0) / vitesses.length;

  // Affichage
  setText('vitesse', `Instant : ${(vLissée * 3.6).toFixed(2)} km/h`);
  setText('vitesse-h', `Horizontale : ${(v.horiz * 3.6).toFixed(2)} km/h`);
  setText('vitesse-v', `Verticale : ${(v.vert * 3.6).toFixed(2)} km/h`);
  setText('vitesse-moy', `Moyenne : ${(moyenne * 3.6).toFixed(2)} km/h`);
  setText('vitesse-max', `Max : ${vitesseMax.toFixed(2)} km/h`);
  setText('distance', `Distance : ${(distanceTotale / 1000).toFixed(3)} km`);
  setText(
    'gps',
    `Lat : ${pos.coords.latitude.toFixed(6)} | Lon : ${pos.coords.longitude.toFixed(6)} | Alt : ${(pos.coords.altitude ?? '--').toFixed(1)} m | ±${pos.coords.accuracy?.toFixed(0)} m`
  );
  if (pression) setText('pression', `Pression : ${pression.toFixed(1)} hPa`);
  const t = (Date.now() - tempsDebut) / 1000;
  setText('temps', `Temps : ${t.toFixed(2)} s`);
}

// ===================== CAPTEURS =====================
function activerCapteurs() {
  if ('DeviceMotionEvent' in window) {
    window.addEventListener('devicemotion', e => {
      const a = e.accelerationIncludingGravity;
      const v = miseAJourVitesseVerticale(a);
      setText('capteurs', `Acc vertical : ${v.toFixed(2)} m/s`);
    });
  }

  // Capteur de pression barométrique
  if ('AbsoluteOrientationSensor' in window || 'Barometer' in window) {
    try {
      const baro = new Barometer({ frequency: 1 });
      baro.addEventListener('reading', () => {
        window.lastPressure = baro.pressure;
      });
      baro.start();
      setText('pression', 'Baromètre activé');
    } catch (err) {
      console.warn('Baromètre non disponible', err);
      setText('pression', 'Pas de baromètre');
    }
  } else {
    setText('pression', 'Baromètre non supporté');
  }
}

// ===================== CONTRÔLES =====================
function demarrerCockpit() {
  if (!navigator.geolocation) {
    setText('gps', 'GPS indisponible');
    return;
  }
  if (watchId !== null) return;
  tempsDebut = Date.now();
  watchId = navigator.geolocation.watchPosition(
    miseAJourPosition,
    err => setText('gps', 'Erreur GPS : ' + err.message),
    { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 }
  );
  activerCapteurs();
  document.getElementById('marche').textContent = '⏹️ Arrêt';
}

function arreterCockpit() {
  if (watchId !== null) navigator.geolocation.clearWatch(watchId);
  watchId = null;
  positionPrecedente = null;
  vitesses = [];
  distanceTotale = 0;
  vitesseMax = 0;
  vitesseVerticaleAcc = 0;
  setText('vitesse', '--');
  setText('vitesse-h', '--');
  setText('vitesse-v', '--');
  setText('vitesse-moy', '--');
  setText('vitesse-max', '--');
  setText('distance', '--');
  setText('gps', '--');
  setText('temps', '0.00 s');
  setText('capteurs', '--');
  setText('pression', '--');
  document.getElementById('marche').textContent = '▶️ Marche';
}

function resetVitesse() {
  vitesseMax = 0;
  vitesses = [];
  setText('vitesse-max', 'Max : 0.00 km/h');
}

// ===================== INIT =====================
document.addEventListener('DOMContentLoaded', () => {
  document.getElementById('marche').addEventListener('click', () => {
    if (watchId === null) demarrerCockpit();
    else arreterCockpit();
  });
  document.getElementById('reset').addEventListener('click', resetVitesse);
});
  
