// ========================
// ETAT GLOBAL
// ========================
let watchId = null;
let positionPrecedente = null;
let vitesseMax = 0;
let vitesses = [];
let distanceTotale = 0;
let tempsDebut = null;

// Etat capteurs
let capteursEtat = {
  niveauBulle: '--',
  lumiere: '--',
  son: '--',
  magnetisme: '--',
  acceleration: 0
};

// Constantes
const VITESSE_LUMIERE = 299792458;
const VITESSE_SON = 343;
const R_TERRE = 6371e3;
const ANNEE_LUMIERE_SECONDES = 3600 * 24 * 365.25;

// ========================
// UTILS DOM
// ========================
function safeSetText(id, text) {
  const el = document.getElementById(id);
  if (el) el.textContent = text;
}

// ========================
// DISTANCE & VITESSE
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

function calculerVitesseCorrigee(gps, acceleration) {
  if (!positionPrecedente) {
    positionPrecedente = gps;
    return 0;
  }

  const dt = (gps.timestamp - positionPrecedente.timestamp) / 1000;
  const d = calculerDistanceHaversine(gps, positionPrecedente);
  let vitesse_brute = dt > 0 ? (d / dt) * 3.6 : 0; // km/h

  // Coefficient de précision GPS
  const coef_precision = d > 0 ? d / (d + (gps.accuracy || 0)) : 0;

  // Coefficient accéléromètre
  const seuil_acc = 0.1; // m/s²
  const coef_mouvement = Math.min(1, acceleration / seuil_acc);

  const vitesse_corrigee = vitesse_brute * coef_precision * coef_mouvement;

  // Mise à jour distance et position
  distanceTotale += d;
  positionPrecedente = gps;

  return vitesse_corrigee;
}

// ========================
// AFFICHAGE
// ========================
function afficherVitesse(v_kmh) {
  const mps = v_kmh / 3.6;
  const mmps = mps * 1000;
  const moyenne_kmh = vitesses.length ? vitesses.reduce((a, b) => a + b, 0) / vitesses.length : 0;

  safeSetText('vitesse', `Vitesse instantanée : ${v_kmh.toFixed(2)} km/h`);
  safeSetText('vitesse-moy', `Vitesse moyenne : ${moyenne_kmh.toFixed(2)} km/h`);
  safeSetText('vitesse-max', `Vitesse max : ${vitesseMax.toFixed(2)} km/h`);
  safeSetText('vitesse-ms', `Vitesse : ${mps.toFixed(2)} m/s | ${mmps.toFixed(0)} mm/s`);
  safeSetText('pourcentage', `% Lumière : ${(mps/VITESSE_LUMIERE*100).toExponential(2)}% | % Son : ${(mps/VITESSE_SON*100).toFixed(2)}%`);
}

function afficherDistance() {
  const km = distanceTotale / 1000;
  const m = distanceTotale;
  const secLumiere = m / VITESSE_LUMIERE;
  const al = secLumiere / ANNEE_LUMIERE_SECONDES;

  safeSetText('distance', `Distance : ${km.toFixed(3)} km | ${m.toFixed(0)} m | ${Math.round(m*1000)} mm`);
  safeSetText('distance-cosmique', `Distance cosmique : ${secLumiere.toFixed(3)} s lumière | ${al.toExponential(3)} al`);
}

function afficherTemps() {
  if (!tempsDebut) return;
  const tempsEcoule = (Date.now() - tempsDebut)/1000;
  safeSetText('temps', `Temps : ${tempsEcoule.toFixed(2)} s`);
}

function afficherGPS(g) {
  safeSetText('gps', `GPS : Lat ${g.latitude.toFixed(6)} | Lon ${g.longitude.toFixed(6)} | Alt ${(g.altitude??'--')} m | Précision ${g.accuracy?.toFixed(0)??'--'} m`);
}

function afficherCapteurs() {
  safeSetText('capteurs', 
    `Niveau : ${capteursEtat.niveauBulle}° | Lumière : ${capteursEtat.lumiere} lux | Son : ${capteursEtat.son} dB | Magnétisme : ${capteursEtat.magnetisme} µT`
  );
}

// ========================
// CAPTEURS PHYSIQUES
// ========================
function activerCapteurs() {
  if ('DeviceMotionEvent' in window) {
    window.addEventListener('devicemotion', e => {
      const a = e.accelerationIncludingGravity;
      if(a) capteursEtat.acceleration = Math.sqrt((a.x||0)**2 + (a.y||0)**2 + (a.z||0)**2);
    });
  }

  // Autres capteurs (niveau, lumière, magnétisme) peuvent être ajoutés ici
}

// ========================
// MISE À JOUR VITESSE
// ========================
function miseAJourVitesse(pos) {
  const gps = {
    latitude: pos.coords.latitude,
    longitude: pos.coords.longitude,
    altitude: pos.coords.altitude,
    accuracy: pos.coords.accuracy,
    timestamp: pos.timestamp
  };

  const vitesse_kmh = calculerVitesseCorrigee(gps, capteursEtat.acceleration);

  if(vitesse_kmh >= 0 && vitesse_kmh < 3000) {
    vitesseMax = Math.max(vitesseMax, vitesse_kmh);
    vitesses.push(vitesse_kmh);
    if(vitesses.length>60) vitesses.shift();

    afficherVitesse(vitesse_kmh);
    afficherDistance();
    afficherTemps();
  }
  afficherGPS(gps);
  afficherCapteurs();
}

// ========================
// DEMARRAGE / ARRET
// ========================
function demarrerCockpit() {
  if(!("geolocation" in navigator)) {
    safeSetText('gps','GPS non disponible');
    return;
  }
  if(watchId !== null) return;

  tempsDebut = Date.now();
  afficherTemps();

  watchId = navigator.geolocation.watchPosition(
    miseAJourVitesse,
    err => safeSetText('gps','Erreur GPS : '+err.code),
    {enableHighAccuracy:true, maximumAge:0, timeout:10000}
  );

  activerCapteurs();
  document.getElementById('marche').textContent='⏹️ Arrêt';
}

function arreterCockpit() {
  if(watchId !== null) navigator.geolocation.clearWatch(watchId);
  watchId = null;
  positionPrecedente=null;
  vitesses=[];
  distanceTotale=0;
  vitesseMax=0;
  tempsDebut=null;

  afficherVitesse(0);
  afficherDistance();
  safeSetText('gps','GPS : --');
  afficherCapteurs();
  document.getElementById('marche').textContent='▶️ Marche';
}

function resetVitesseMax() {
  vitesseMax=0;
  safeSetText('vitesse-max','Vitesse max : 0.00 km/h');
}

// ========================
// INIT
// ========================
document.addEventListener('DOMContentLoaded', () => {
  afficherVitesse(0);
  afficherDistance();
  safeSetText('temps','Temps : 0.00 s');

  document.getElementById('marche').addEventListener('click', ()=>{
    if(watchId===null) demarrerCockpit(); else arreterCockpit();
  });
  document.getElementById('reset').addEventListener('click', resetVitesseMax);
});
