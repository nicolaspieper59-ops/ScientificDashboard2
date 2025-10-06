// ========================
// ETAT GLOBAL
// ========================
let watchId = null;
let positionPrecedente = null;
let vitesseMax = 0;
let vitesses = [];
let distanceTotale = 0;
let tempsDebut = null;

// Capteurs physiques
let capteursEtat = {
  niveauBulle: '--',
  lumiere: '--',
  son: '--',
  magnetisme: '--'
};

let vitesseVerticaleAcc = 0;
let accelPrecedente = null;

// ========================
// UTILITAIRES DOM
// ========================
function safeSetText(id, text) {
  const e = document.getElementById(id);
  if (e) e.textContent = text;
}

// ========================
// GPS 3D
// ========================
const R_TERRE = 6371e3;

function distanceHaversine(p1, p2) {
  const φ1 = p1.coords.latitude * Math.PI / 180;
  const φ2 = p2.coords.latitude * Math.PI / 180;
  const Δφ = (p2.coords.latitude - p1.coords.latitude) * Math.PI / 180;
  const Δλ = (p2.coords.longitude - p1.coords.longitude) * Math.PI / 180;

  const a = Math.sin(Δφ/2)**2 + Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ/2)**2;
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
  return R_TERRE * c;
}

function vitesse3D(pos, posPrec) {
  if (!posPrec) return 0;
  const dt = (pos.timestamp - posPrec.timestamp)/1000;
  if (dt <= 0) return 0;

  // Distance horizontale
  const dh = distanceHaversine(pos, posPrec);

  // Distance verticale
  let dy = pos.coords.altitude ?? 0;
  let dyPrec = posPrec.coords.altitude ?? 0;
  const dv = dy - dyPrec;

  return Math.sqrt(dh**2 + dv**2) / dt; // m/s
}

// ========================
// ACCELEROMETRE VERTICAL
// ========================
function miseAJourVitesseVerticale(accel) {
  const dt = 0.05; // approximatif, peut être recalculé avec timestamps
  if (accelPrecedente !== null) {
    vitesseVerticaleAcc += (accel.y + accelPrecedente.y)/2 * dt;
  }
  accelPrecedente = accel;
  return vitesseVerticaleAcc; // m/s
}

// ========================
// AFFICHAGE
// ========================
function afficherVitesseTotale(v_mps) {
  const v_kmh = v_mps * 3.6;
  vitesseMax = Math.max(vitesseMax, v_kmh);
  vitesses.push(v_kmh);
  if (vitesses.length > 60) vitesses.shift();
  const moyenne = vitesses.reduce((a,b)=>a+b,0)/vitesses.length;

  safeSetText('vitesse', `Instant : ${v_kmh.toFixed(2)} km/h`);
  safeSetText('vitesse-moy', `Moyenne : ${moyenne.toFixed(2)} km/h`);
  safeSetText('vitesse-max', `Max : ${vitesseMax.toFixed(2)} km/h`);
}

function afficherDistanceTotale() {
  const km = distanceTotale/1000;
  safeSetText('distance', `Distance : ${km.toFixed(3)} km`);
}

// ========================
// MISE A JOUR GPS
// ========================
function miseAJourPosition(pos) {
  let v_mps = vitesse3D(pos, positionPrecedente);
  positionPrecedente = pos;

  // Combinaison avec accéléromètre vertical
  if (typeof vitesseVerticaleAcc === 'number') {
    v_mps = Math.sqrt(v_mps**2 + vitesseVerticaleAcc**2);
  }

  // Mise à jour distance horizontale pour affichage
  if (pos.coords.latitude && pos.coords.longitude && positionPrecedente) {
    distanceTotale += distanceHaversine(pos, positionPrecedente);
  }

  afficherVitesseTotale(v_mps);
  afficherDistanceTotale();
  safeSetText('gps', `Lat : ${pos.coords.latitude.toFixed(6)} | Lon : ${pos.coords.longitude.toFixed(6)} | Alt : ${(pos.coords.altitude ?? '--').toFixed(1)} m`);
}

// ========================
// COCKPIT START / STOP
// ========================
function demarrerCockpit() {
  if (!navigator.geolocation) {
    safeSetText('gps','GPS indisponible');
    return;
  }
  if (watchId !== null) return;

  tempsDebut = Date.now();

  // GPS
  watchId = navigator.geolocation.watchPosition(
    miseAJourPosition,
    err => safeSetText('gps','Erreur GPS : '+err.code),
    { enableHighAccuracy:true, maximumAge:0, timeout:10000 }
  );

  // Capteurs
  activerCapteurs();

  document.getElementById('marche').textContent = '⏹️ Arrêt';
}

function arreterCockpit() {
  if (watchId !== null) navigator.geolocation.clearWatch(watchId);
  watchId = null;
  positionPrecedente = null;
  vitesses = [];
  distanceTotale = 0;
  vitesseVerticaleAcc = 0;
  accelPrecedente = null;
  vitesseMax = 0;

  // Reset affichage
  safeSetText('vitesse','--');
  safeSetText('vitesse-moy','--');
  safeSetText('vitesse-max','--');
  safeSetText('distance','--');
  safeSetText('gps','--');

  document.getElementById('marche').textContent = '▶️ Marche';
}

// ========================
// CAPTEURS
// ========================
function activerCapteurs() {
  // Accéléromètre vertical
  if ('DeviceMotionEvent' in window) {
    window.addEventListener('devicemotion', e => {
      miseAJourVitesseVerticale(e.accelerationIncludingGravity);
      capteursEtat.niveauBulle = (e.beta ?? '--').toFixed(1);
      afficherCapteurs();
    });
  }
}

function afficherCapteurs() {
  safeSetText('capteurs', 
    `Niveau : ${capteursEtat.niveauBulle}° | Lumière : ${capteursEtat.lumiere} | Son : ${capteursEtat.son} | Magnétisme : ${capteursEtat.magnetisme}`
  );
}

// ========================
// INIT
// ========================
document.addEventListener('DOMContentLoaded', () => {
  document.getElementById('marche').addEventListener('click', ()=> {
    if (watchId === null) demarrerCockpit();
    else arreterCockpit();
  });
});
  
