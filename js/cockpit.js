// ========================
// État global
// ========================
let watchId = null;
let positionPrecedente = null;
let vitesseMax = 0;
let vitesses = [];
let distanceTotale = 0;
let tempsDebut = null;

let accel = { x: 0, y: 0, z: 0 }; // accélération pour correction vitesse

// ========================
// Utilitaires DOM
// ========================
function safeSetText(id, text) {
  const e = document.getElementById(id);
  if (e) e.textContent = text;
}

// ========================
// Calcul distance et vitesse 3D
// ========================
const R_TERRE = 6371e3;

function haversine3D(p1, p2) {
  // distance 2D
  const φ1 = p1.latitude * Math.PI/180;
  const φ2 = p2.latitude * Math.PI/180;
  const Δφ = (p2.latitude - p1.latitude) * Math.PI/180;
  const Δλ = (p2.longitude - p1.longitude) * Math.PI/180;
  const a = Math.sin(Δφ/2)**2 + Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2;
  const c = 2*Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
  const d2D = R_TERRE * c;

  // distance verticale
  const dz = (p2.altitude ?? 0) - (p1.altitude ?? 0);

  // distance 3D
  return Math.sqrt(d2D*d2D + dz*dz);
}

function calculerVitesse3D(pos) {
  if (!positionPrecedente) {
    positionPrecedente = pos;
    return 0;
  }
  const dt = (pos.timestamp - positionPrecedente.timestamp)/1000;
  if (dt <= 0) return 0;

  const dist = haversine3D(positionPrecedente, pos);
  distanceTotale += dist;
  positionPrecedente = pos;

  return (dist/dt) * 3.6; // km/h
}

// ========================
// GPS / Accéléromètre
// ========================
function miseAJour(pos) {
  const gps = {
    latitude: pos.coords.latitude,
    longitude: pos.coords.longitude,
    altitude: pos.coords.altitude ?? 0,
    accuracy: pos.coords.accuracy,
    timestamp: pos.timestamp,
    speed: pos.coords.speed
  };

  // Correction vitesse avec accélération verticale
  let vitesse_kmh = (gps.speed >= 0 ? gps.speed : calculerVitesse3D(gps)/3.6) * 3.6;
  // simple correction avec accel.z (approximative)
  vitesse_kmh += (accel.z || 0) * 3.6; 

  vitesseMax = Math.max(vitesseMax, vitesse_kmh);
  vitesses.push(vitesse_kmh);
  if (vitesses.length>60) vitesses.shift();

  const moyenne = vitesses.reduce((a,b)=>a+b,0)/vitesses.length;

  safeSetText('vitesse', `Vitesse instantanée : ${vitesse_kmh.toFixed(2)} km/h`);
  safeSetText('vitesse-moy', `Vitesse moyenne : ${moyenne.toFixed(2)} km/h`);
  safeSetText('vitesse-max', `Vitesse max : ${vitesseMax.toFixed(2)} km/h`);
  safeSetText('distance', `Distance totale : ${(distanceTotale/1000).toFixed(3)} km`);
  safeSetText('pourcentage', `% Lumière : ${(vitesse_kmh/299792458*100).toExponential(2)}% | % Son : ${(vitesse_kmh/343*100).toFixed(2)}%`);
  safeSetText('gps', `Lat: ${gps.latitude.toFixed(6)} | Lon: ${gps.longitude.toFixed(6)} | Alt: ${gps.altitude.toFixed(1)} m | Précision: ${gps.accuracy?.toFixed(0)} m`);
}

// ========================
// Capteurs
// ========================
function activerAccelerometre() {
  if (window.DeviceMotionEvent) {
    window.addEventListener('devicemotion', e => {
      accel.x = e.accelerationIncludingGravity?.x ?? 0;
      accel.y = e.accelerationIncludingGravity?.y ?? 0;
      accel.z = e.accelerationIncludingGravity?.z ?? 0;
    });
  }
}

// ========================
// Démarrage / arrêt
// ========================
function demarrerCockpit() {
  if (!navigator.geolocation) { safeSetText('gps','GPS non disponible'); return; }
  if (watchId!==null) return;

  tempsDebut = Date.now();
  vitesses=[]; distanceTotale=0; positionPrecedente=null;

  watchId = navigator.geolocation.watchPosition(
    miseAJour,
    err => safeSetText('gps','Erreur GPS: '+err.code),
    { enableHighAccuracy:true, maximumAge:0, timeout:10000 }
  );

  activerAccelerometre();
}

function arreterCockpit() {
  if (watchId!==null) navigator.geolocation.clearWatch(watchId);
  watchId = null; positionPrecedente=null; vitesses=[]; distanceTotale=0; vitesseMax=0;
  safeSetText('vitesse','Vitesse instantanée : -- km/h');
  safeSetText('vitesse-moy','Vitesse moyenne : -- km/h');
  safeSetText('vitesse-max','Vitesse max : -- km/h');
  safeSetText('distance','Distance totale : -- km');
  safeSetText('gps','Lat: -- | Lon: -- | Alt: -- m | Précision: -- m');
}

// ========================
// Réinitialiser max
// ========================
function resetVitesseMax() { vitesseMax=0; safeSetText('vitesse-max','Vitesse max : 0.00 km/h'); }

// ========================
// Événements DOM
// ========================
document.addEventListener('DOMContentLoaded',()=>{
  document.getElementById('marche').addEventListener('click',demarrerCockpit);
  document.getElementById('arreter').addEventListener('click',arreterCockpit);
  document.getElementById('reset').addEventListener('click',resetVitesseMax);
});
