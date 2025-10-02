// ============================
// cockpit.js - GPS et vitesse
// ============================

let watchId = null;
let positionPrecedente = null;
let vitesses = [];
let vitesseMax = 0;
let distanceTotale = 0;

// === Utilitaires DOM ===
function safeSetText(id, text) {
  const el = document.getElementById(id);
  if (el) el.textContent = text;
}

// === Boutons de contrôle ===
document.getElementById('marche').addEventListener('click', demarrerCockpit);
document.getElementById('reset').addEventListener('click', () => {
  vitesseMax = 0;
  safeSetText('vitesse', '--');
});

// === Démarrage cockpit ===
function demarrerCockpit() {
  if (!('geolocation' in navigator)) {
    safeSetText('gps', 'GPS non disponible');
    return;
  }
  if (watchId !== null) return;

  navigator.geolocation.getCurrentPosition(
    pos => traiterPosition(pos.coords, pos.timestamp),
    err => safeSetText('gps', 'Erreur GPS : ' + err.message)
  );

  watchId = navigator.geolocation.watchPosition(
    pos => traiterPosition(pos.coords, pos.timestamp),
    err => safeSetText('gps', 'Erreur GPS : ' + err.message),
    { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 }
  );
}

// === Traitement position GPS ===
function traiterPosition(coords, timestamp) {
  const vitesse = coords.speed != null ? coords.speed * 3.6 : calculerVitesse(coords, timestamp);
  if (vitesse >= 0 && vitesse < 300) {
    vitesseMax = Math.max(vitesseMax, vitesse);
    vitesses.push(vitesse);
    afficherVitesse(vitesse);
    afficherDistance();
    afficherPourcentage(vitesse);
    afficherGPS(coords);
  }
}

// === Calcul vitesse si pas fournie ===
function calculerVitesse(coords, timestamp) {
  if (!positionPrecedente) {
    positionPrecedente = { ...coords, timestamp };
    return 0;
  }
  const dt = (timestamp - positionPrecedente.timestamp) / 1000;
  const d = calculerDistance(coords, positionPrecedente);
  distanceTotale += d;
  positionPrecedente = { ...coords, timestamp };
  return dt > 0 ? (d / dt) * 3.6 : 0;
}

// === Calcul distance entre deux points GPS ===
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

// === Affichages ===
function afficherVitesse(v) {
  const moyenne = vitesses.length ? vitesses.reduce((a, b) => a + b, 0) / vitesses.length : 0;
  safeSetText('vitesse', `Vitesse : ${v.toFixed(2)} km/h | Moyenne : ${moyenne.toFixed(2)} | Max : ${vitesseMax.toFixed(2)}`);
}

function afficherDistance() {
  safeSetText('distance', `Distance : ${(distanceTotale / 1000).toFixed(3)} km`);
}

function afficherPourcentage(v) {
  const mps = v / 3.6;
  const pctL = (mps / 299792458) * 100;
  const pctS = (mps / 343) * 100;
  safeSetText('pourcentage', `% Lumière : ${pctL.toExponential(2)}% | % Son : ${pctS.toFixed(2)}%`);
}

function afficherGPS(g) {
  safeSetText('gps', `Latitude : ${g.latitude.toFixed(6)} | Longitude : ${g.longitude.toFixed(6)} | Altitude : ${(g.altitude ?? 0).toFixed(1)} m | Précision : ${(g.accuracy ?? 0).toFixed(0)} m`);
              }
      
