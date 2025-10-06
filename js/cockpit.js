// cockpit3d_kalman.js
let watchId = null;
let positionPrecedente = null;
let vitesseMax = 0;
let vitesses = [];
let distanceTotale = 0;
let tempsDebut = null;

// Etat des capteurs
let capteursEtat = { x: 0, y: 0, z: 0 };

// Constantes physiques
const VITESSE_LUMIERE = 299792458;
const VITESSE_SON = 343;
const R_TERRE = 6371e3;
const ANNEE_LUMIERE_SECONDES = 3600 * 24 * 365.25;

// =========================
// UTILITAIRES DOM
// =========================
function safeSetText(id, text) {
  const e = document.getElementById(id);
  if (e) e.textContent = text;
}

// =========================
// CALCUL DISTANCE & VITESSE 3D
// =========================
function calculerDistanceHaversine(p1, p2) {
  const φ1 = p1.latitude * Math.PI / 180;
  const φ2 = p2.latitude * Math.PI / 180;
  const Δφ = (p2.latitude - p1.latitude) * Math.PI / 180;
  const Δλ = (p2.longitude - p1.longitude) * Math.PI / 180;
  const a = Math.sin(Δφ / 2) ** 2 + Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ / 2) ** 2;
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
  return R_TERRE * c; // distance horizontale en mètres
}

function calculerVitesse3D(gps) {
  if (!positionPrecedente) {
    positionPrecedente = gps;
    return 0;
  }
  const dt = (gps.timestamp - positionPrecedente.timestamp) / 1000;
  const dHorizontal = calculerDistanceHaversine(gps, positionPrecedente);
  const dz = (gps.altitude ?? 0) - (positionPrecedente.altitude ?? 0);
  const distance = Math.sqrt(dHorizontal ** 2 + dz ** 2);
  distanceTotale += distance;
  positionPrecedente = gps;
  return dt > 0 ? (distance / dt) * 3.6 : 0; // km/h
}

// =========================
// AFFICHAGE
// =========================
function afficherVitesse(v) {
  const moyenne = vitesses.length ? vitesses.reduce((a, b) => a + b, 0) / vitesses.length : 0;
  safeSetText('vitesse', `Vitesse instantanée : ${v.toFixed(2)} km/h`);
  safeSetText('vitesse-moy', `Vitesse moyenne : ${moyenne.toFixed(2)} km/h`);
  safeSetText('vitesse-max', `Vitesse max : ${vitesseMax.toFixed(2)} km/h`);
  const mps = v / 3.6;
  safeSetText('vitesse-ms', `Vitesse : ${mps.toFixed(2)} m/s | ${(mps*1000).toFixed(0)} mm/s`);
  safeSetText('pourcentage', `% Lumière : ${(mps/VITESSE_LUMIERE*100).toExponential(2)}% | % Son : ${(mps/VITESSE_SON*100).toFixed(2)}%`);
}

function afficherDistance() {
  const km = distanceTotale/1000;
  safeSetText('distance', `Distance : ${km.toFixed(3)} km`);
  const secLumiere = distanceTotale / VITESSE_LUMIERE;
  const al = secLumiere / ANNEE_LUMIERE_SECONDES;
  safeSetText('distance-cosmique', `Distance cosmique : ${secLumiere.toFixed(3)} s lumière | ${al.toExponential(3)} al`);
}

function afficherTemps() {
  if (!tempsDebut) return;
  const t = (Date.now() - tempsDebut)/1000;
  safeSetText('temps', `Temps : ${t.toFixed(2)} s`);
}

function afficherGPS(g) {
  safeSetText('gps', `GPS : Lat ${g.latitude.toFixed(6)} | Lon ${g.longitude.toFixed(6)} | Alt ${(g.altitude??'--')} m | Précision ${(g.accuracy??'--')} m`);
}

function afficherCapteurs() {
  safeSetText('capteurs', `Acc XYZ : ${capteursEtat.x.toFixed(2)}, ${capteursEtat.y.toFixed(2)}, ${capteursEtat.z.toFixed(2)}`);
}

// =========================
// GESTION GPS
// =========================
function miseAJourPosition(pos) {
  const gps = {
    latitude: pos.coords.latitude,
    longitude: pos.coords.longitude,
    altitude: pos.coords.altitude ?? 0,
    accuracy: pos.coords.accuracy,
    timestamp: pos.timestamp
  };
  const vitesse = calculerVitesse3D(gps);
  if (vitesse >=0 && vitesse < 3000000) {
    vitesseMax = Math.max(vitesseMax, vitesse);
    vitesses.push(vitesse);
    if (vitesses.length>60) vitesses.shift();
    afficherVitesse(vitesse);
    afficherDistance();
    afficherTemps();
  }
  afficherGPS(gps);
}

// =========================
// DÉMARRAGE / ARRÊT
// =========================
export function demarrerCockpit() {
  if (!("geolocation" in navigator)) { safeSetText('gps','GPS non dispo'); return; }
  if (watchId !== null) return;
  tempsDebut = Date.now();
  afficherTemps();

  watchId = navigator.geolocation.watchPosition(
    miseAJourPosition,
    err => { safeSetText('gps','Erreur GPS : '+err.message); },
    { enableHighAccuracy:true, maximumAge:0, timeout:10000 }
  );

  // Capteurs de mouvement
  if ('DeviceMotionEvent' in window) {
    window.addEventListener('devicemotion', e => {
      capteursEtat.x = e.accelerationIncludingGravity?.x ?? 0;
      capteursEtat.y = e.accelerationIncludingGravity?.y ?? 0;
      capteursEtat.z = e.accelerationIncludingGravity?.z ?? 0;
      afficherCapteurs();
    });
  }

  document.getElementById('marche').textContent = '⏹️ Arrêt';
}

export function arreterCockpit() {
  if (watchId!==null) navigator.geolocation.clearWatch(watchId);
  watchId=null; positionPrecedente=null; vitesses=[]; distanceTotale=0; tempsDebut=null;
  vitesseMax=0;
  afficherVitesse(0); afficherDistance(); afficherTemps();
  safeSetText('gps','GPS : --'); afficherCapteurs();
  document.getElementById('marche').textContent='▶️ Marche';
}

export function resetVitesseMax() {
  vitesseMax=0;
  safeSetText('vitesse-max','Vitesse max : 0 km/h');
}
