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

// ============================
// Capteurs physiques
// ============================
function activerCapteurs() {
  const cap = document.getElementById('capteurs');
  cap.textContent = '--';

  if ('AmbientLightSensor' in window) {
    try {
      const light = new AmbientLightSensor();
      light.addEventListener('reading', () => {
        cap.textContent = `Lumière : ${Math.round(light.illuminance)} lux`;
      });
      light.start();
    } catch {}
  }

  if (navigator.mediaDevices?.getUserMedia) {
    navigator.mediaDevices.getUserMedia({ audio: true }).then(stream => {
      const ctx = new AudioContext();
      const analyser = ctx.createAnalyser();
      const source = ctx.createMediaStreamSource(stream);
      source.connect(analyser);
      const data = new Uint8Array(analyser.frequencyBinCount);
      (function loop() {
        analyser.getByteFrequencyData(data);
        const dB = 20 * Math.log10(data.reduce((a, b) => a + b, 0) / data.length || 1);
        cap.textContent += ` | Son : ${Math.round(dB)} dB | Fréquence : ${analyser.frequencyBinCount} Hz`;
        requestAnimationFrame(loop);
      })();
    });
  }

  if ('DeviceOrientationEvent' in window) {
    window.addEventListener('deviceorientation', e => {
      if (e.beta != null) {
        cap.textContent += ` | Niveau à bulle : ${e.beta.toFixed(1)}°`;
      }
    });
  }

  if ('Magnetometer' in window) {
    try {
      const mag = new Magnetometer();
      mag.addEventListener('reading', () => {
        const champ = Math.sqrt(mag.x ** 2 + mag.y ** 2 + mag.z ** 2);
        cap.textContent += ` | Magnétisme : ${champ.toFixed(0)} µT`;
      });
      mag.start();
    } catch {}
  }
}

// ============================
// Boussole Minecraft
// ============================
function activerBoussole() {
  if ('DeviceOrientationEvent' in window) {
    window.addEventListener('deviceorientationabsolute', e => {
      if (e.alpha != null) {
        const cap = e.alpha.toFixed(0);
        const minecraft = positionPrecedente
          ? `X:${Math.round(positionPrecedente.longitude * 1000)} Z:${Math.round(positionPrecedente.latitude * 1000)}`
          : '--';
        const capDest = destination.latitude
          ? calculerCapVersDestination(positionPrecedente, destination).toFixed(0) + '°'
          : '--';
        set('boussole', `Cap : ${cap}° | Coordonnées Minecraft : ${minecraft} | Cap vers destination : ${capDest}`);
      }
    });
  }
}

function calculerCapVersDestination(pos, dest) {
  if (!pos || !dest.latitude) return 0;
  const φ1 = pos.latitude * Math.PI / 180;
  const φ2 = dest.latitude * Math.PI / 180;
  const Δλ = (dest.longitude - pos.longitude) * Math.PI / 180;
  const y = Math.sin(Δλ) * Math.cos(φ2);
  const x = Math.cos(φ1) * Math.sin(φ2) - Math.sin(φ1) * Math.cos(φ2) * Math.cos(Δλ);
  const θ = Math.atan2(y, x);
  return (θ * 180 / Math.PI + 360) % 360;
}

// ============================
// Horloge Minecraft
// ============================
function activerHorlogeMinecraft() {
  const h = document.getElementById('horloge-minecraft');
  function tick() {
    const d = new Date();
    h.textContent = `${String(d.getHours()).padStart(2, '0')}:${String(d.getMinutes()).padStart(2, '0')}:${String(d.getSeconds()).padStart(2, '0')}`;
    requestAnimationFrame(tick);
  }
  tick();
}

// ============================
// Météo & qualité de l’air
// ============================
function chargerMeteo() {
  const meteoEl = document.getElementById('meteo');
  const airEl = document.getElementById('qualite-air');
  if (!navigator.onLine) {
    meteoEl.textContent = 'Pas de connexion';
    airEl.textContent = 'Indisponible';
    return;
  }

  fetch("https://api.open-meteo.com/v1/forecast?latitude=43.3&longitude=5.4&current_weather=true")
    .then(r => r.json())
    .then(data => {
      const w = data.current_weather;
      meteoEl.textContent =
        `Température : ${w.temperature}°C | Pression : ${w.pressure ?? '--'} hPa | Humidité : ${w.relative_humidity ?? '--'}% | Vent : ${w.windspeed} km/h | Nuages : ${w.cloudcover ?? '--'}% | Pluie : ${w.precipitation ?? '0'} mm | Neige : ${w.snowfall ?? '0'} mm | Indice UV : ${w.uv_index ?? '--'}`;
    })
    .catch(() => {
      meteoEl.textContent = 'Météo indisponible';
    });

  fetch("https://api.openaq.org/v2/latest?coordinates=43.3,5.4")
    .then(r => r.json())
    .then(d => {
      const m = d.results[0]?.measurements ?? [];
      const pm25 = m.find(a => a.parameter === "pm25")?.value ?? "--";
      const no2 = m.find(a => a.parameter === "no2")?.value ?? "--";
      const co = m.find(a => a.parameter === "co")?.value ?? "--";
      const so2 = m.find(a => a.parameter === "so2")?.value ?? "--";
      airEl.textContent = `Qualité air : PM2.5 ${pm25} µg/m³ | NO₂ ${no2} µg/m³ | CO ${co} µg/m³ | SO₂ ${so2} µg/m³`;
    })
    .catch(() => {
      airEl.textContent = 'Indisponible';
    });
}

// ============================
// Grandeurs physiques
// ============================
function afficherGrandeurs() {
  const gravite = 9.81;
  const masse = 70;
  const vitesse = vitesses.length ? vitesses[vitesses.length - 1] / 3.6 : 0;
  const force = masse * gravite;
  const cinetique = 0.5 * masse * vitesse ** 2;
  const puissance = force * vitesse;
  const ebullition = 100 + (masse / 10);
  set('grandeurs', `Point d’ébullition : ${ebullition.toFixed(1)}°C | Gravité : ${gravite} m/s² | Force : ${force.toFixed(1)} N | Puissance : ${puissance.toFixed(1)} J/s | Cinétique : ${cinetique.toFixed(1)} J`);
}

let rituelActif = true;

document.getElementById('toggle-rituel').addEventListener('click', () => {
  rituelActif = !rituelActif;
  document.body.classList.toggle('rituel-off', !rituelActif);
  document.getElementById('toggle-rituel').textContent =
    rituelActif ? '✨ Rituel cosmique : ON' : '🔋 Rituel cosmique : OFF';
});

