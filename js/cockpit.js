// ========================
// ETAT GLOBAL
// ========================
let watchId = null;
let positionPrecedente = null;
let vitesseMax = 0;
let vitesses = [];
let distanceTotale = 0;
let tempsDebut = null;

// Etat des capteurs inertiels
let acc = { x: 0, y: 0, z: 0 };
let vitesseAcc = { x: 0, y: 0, z: 0 };

// ========================
// UTILS DOM
// ========================
function safeSetText(id, text) {
  const e = document.getElementById(id);
  if (e) e.textContent = text;
}

// ========================
// CALCUL DISTANCE / VITESSE 3D
// ========================
const R_TERRE = 6371e3; // rayon Terre en m

function calculerDistance3D(p1, p2) {
  const φ1 = p1.latitude * Math.PI/180;
  const φ2 = p2.latitude * Math.PI/180;
  const Δφ = (p2.latitude - p1.latitude) * Math.PI/180;
  const Δλ = (p2.longitude - p1.longitude) * Math.PI
  
