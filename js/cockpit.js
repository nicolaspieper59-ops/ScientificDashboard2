// ========================
// ETAT GLOBAL
// ========================
let watchId = null;
let positionPrecedente = null;
let vitesseMax = 0;
let vitesses = [];
let distanceTotale = 0;
let tempsDebut = null;

let capteursEtat = {
  niveauBulle: '--',
  lumiere: '--',
  son: '--',
  magnetisme: '--'
};

// ========================
// UTILS DOM
// ========================
function safeSetText(id, text) {
  const e = document.getElementById(id);
  if (e) e.textContent = text;
}

// ========================
// CONSTANTES PHYSIQUES
// ========================
const VITESSE_LUMIERE = 299792458;
const VITESSE_SON = 343;
const R_TERRE = 6371e3;
const ANNEE_LUMIERE_SECONDES = 3600 * 24 * 365.25;

// ========================
// CALCUL DISTANCE / VITESSE
// ========================
function calculerDistanceHaversine(p1, p2) {
  const φ1 = p1.latitude * Math.PI / 180;
  const φ2 = p2.latitude * Math.PI / 180;
  const Δφ = (p2.latitude - p1.latitude) * Math.PI / 180;
  const Δλ = (p2.longitude - p1.longitude) * Math.PI / 180;
  const a = Math.sin(Δφ / 2) ** 2 + Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ / 2) ** 2;
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
  return R_TERRE * c;
}

function calculerVitesseManuelle(gps) {
  if (!positionPrecedente) {
    positionPrecedente = gps;
    return 0;
  }
  const dt = (gps.timestamp - positionPrecedente.timestamp) / 1000;
  const d = calculerDistanceHaversine(gps, positionPrecedente);
  if (!Number.isFinite(d) || d <= 0 || dt <= 0) {
    positionPrecedente = gps;
    return 0;
  }
  distanceTotale += d;
  positionPrecedente = gps;
  return (d / dt) * 3.6; // m/s -> km/h
}

// ========================
// AFFICHAGE
// ========================
function afficherVitesse(v_kmh) {
  const mps = v_kmh / 3.6;
  const mmps = mps * 1000;
  const moyenne_kmh = vitesses.length
    ? vitesses.reduce((a, b) => a + b, 0) / vitesses.length
    : 0;

  safeSetText('vitesse', `Vitesse instantanée : ${v_kmh.toFixed(2)} km/h`);
  safeSetText('vitesse-moy', `Vitesse moyenne : ${moyenne_kmh.toFixed(2)} km/h`);
  safeSetText('vitesse-max', `Vitesse max : ${vitesseMax.toFixed(2)} km/h`);
  safeSetText('vitesse-ms', `Vitesse : ${mps.toFixed(2)} m/s | ${mmps.toFixed(0)} mm/s`);
  safeSetText('pourcentage',
    `% Lumière : ${(mps / VITESSE_LUMIERE * 100).toExponential(2)}% | % Son : ${(mps / VITESSE_SON * 100).toFixed(2)}%`);
}

function afficherDistance() {
  const km = distanceTotale / 1000;
  const m = distanceTotale;
  const secLumiere = m / VITESSE_LUMIERE;
  const al = secLumiere / ANNEE_LUMIERE_SECONDES;

  safeSetText('distance', `Distance : ${km.toFixed(3)} km | ${m.toFixed(0)} m | ${Math.round(m * 1000)} mm`);
  safeSetText('distance-cosmique', `Distance cosmique : ${secLumiere.toFixed(3)} s lumière | ${al.toExponential(3)} al`);
}

function afficherTemps() {
  if (!tempsDebut) return;
  const t = (Date.now() - tempsDebut) / 1000;
  safeSetText('temps', `Temps : ${t.toFixed(2)} s`);
}

function afficherGPS(g) {
  safeSetText('gps',
    `GPS : Latitude ${g.latitude.toFixed(6)} | Longitude ${g.longitude.toFixed(6)} | Altitude ${g.altitude?.toFixed(0) ?? '--'} m | Précision ${g.accuracy?.toFixed(0) ?? '--'} m`);
}

// ========================
// CAPTEURS
// ========================
function activerCapteurs() {
  if (window.DeviceOrientationEvent) {
    window.addEventListener('deviceorientation', e => {
      capteursEtat.niveauBulle = e.beta ? e.beta.toFixed(1) : '--';
      afficherCapteurs();
    });
  }
  if ('AmbientLightSensor' in window) {
    try {
      const l = new AmbientLightSensor();
      l.onreading = () => {
        capteursEtat.lumiere = Math.round(l.illuminance);
        afficherCapteurs();
      };
      l.start();
    } catch (e) { console.warn(e); }
  }
  if (navigator.mediaDevices?.getUserMedia) {
    navigator.mediaDevices.getUserMedia({ audio: true })
      .then(stream => {
        const ctx = new AudioContext();
        const src = ctx.createMediaStreamSource(stream);
        const analyser = ctx.createAnalyser();
        src.connect(analyser);
        const data = new Uint8Array(analyser.frequencyBinCount);
        setInterval(() => {
          analyser.getByteFrequencyData(data);
          const val = data.reduce((a, b) => a + b, 0) / data.length;
          capteursEtat.son = Math.round(val);
          afficherCapteurs();
        }, 500);
      })
      .catch(err => console.warn('Micro refusé', err));
  }
}

function afficherCapteurs() {
  safeSetText('capteurs',
    `Lumière : ${capteursEtat.lumiere} lux | Son : ${capteursEtat.son} dB | Niveau : ${capteursEtat.niveauBulle}° | Magnétisme : ${capteursEtat.magnetisme} µT`);
}

// ========================
// HORLOGE & MEDAILLON
// ========================
function activerHorloge() {
  const h = document.getElementById('horloge');
  function loop() {
    const t = new Date();
    h.textContent = `${t.getHours().toString().padStart(2, '0')}:${t.getMinutes().toString().padStart(2, '0')}:${t.getSeconds().toString().padStart(2, '0')}`;
    requestAnimationFrame(loop);
  }
  loop();
}

function afficherMedaillon() {
  const m = document.getElementById('medaillon');
  m.innerHTML = '';
  const c = document.createElement('canvas');
  c.width = 300; c.height = 300;
  m.appendChild(c);
  const ctx = c.getContext('2d');
  ctx.fillStyle = '#000';
  ctx.fillRect(0, 0, c.width, c.height);
  ctx.fillStyle = '#ffd700';
  ctx.beginPath();
  ctx.arc(150, 100, 10, 0, 2 * Math.PI);
  ctx.fill();
  ctx.fillStyle = '#ccc';
  ctx.beginPath();
  ctx.arc(150, 200, 8, 0, 2 * Math.PI);
  ctx.fill();
}

// ========================
// GPS / DEMARRAGE / RESET
// ========================
function miseAJourVitesse(pos) {
  const gps = {
    latitude: pos.coords.latitude,
    longitude: pos.coords.longitude,
    altitude: pos.coords.altitude,
    accuracy: pos.coords.accuracy,
    timestamp: pos.timestamp
  };

  const vitesse_kmh = calculerVitesseManuelle(gps);
  if (vitesse_kmh >= 0 && vitesse_kmh < 3000000) {
    vitesseMax = Math.max(vitesseMax, vitesse_kmh);
    vitesses.push(vitesse_kmh);
    if (vitesses.length > 60) vitesses.shift();
    afficherVitesse(vitesse_kmh);
    afficherDistance();
    afficherTemps();
  }
  afficherGPS(gps);
}

function demarrerCockpit() {
  if (!("geolocation" in navigator)) {
    safeSetText('gps', 'GPS non disponible');
    return;
  }
  if (watchId !== null) return;

  tempsDebut = Date.now();
  afficherTemps();
  activerHorloge();
  afficherMedaillon();

  watchId = navigator.geolocation.watchPosition(
    miseAJourVitesse,
    err => { console.warn('GPS refusé', err); safeSetText('gps', 'Erreur GPS'); },
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
  tempsDebut = null;
  afficherVitesse(0);
  afficherDistance();
  safeSetText('temps', 'Temps : 0.00 s');
  safeSetText('gps', 'GPS : --');
  document.getElementById('marche').textContent = '▶️ Marche';
}

function resetVitesseMax() {
  vitesseMax = 0;
  safeSetText('vitesse-max', 'Vitesse max : 0.00 km/h');
}

// ========================
// INIT
// ========================
document.addEventListener('DOMContentLoaded', () => {
  afficherVitesse(0);
  afficherDistance();
  safeSetText('temps', 'Temps : 0.00 s');
  document.getElementById('marche').addEventListener('click', () => {
    if (watchId) arreterCockpit(); else demarrerCockpit();
  });
  document.getElementById('reset').addEventListener('click', resetVitesseMax);
});
  
