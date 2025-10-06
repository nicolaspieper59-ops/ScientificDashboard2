// cockpit3d_kalman.js

// =========================
// ETAT GLOBAL
// =========================
let watchId = null;
let positionPrecedente = null; // {latitude, longitude, altitude, timestamp}
let vitesseMax = 0;
let vitesses = [];
let distanceTotale = 0; // en mètres
let tempsDebut = null;
let isRunning = false;

// Etat des capteurs
let capteursEtat = { x: 0, y: 0, z: 0 };

// Constantes physiques
const VITESSE_LUMIERE = 299792458; // m/s
const VITESSE_SON = 343; // m/s (approximatif)
const R_TERRE = 6371e3; // Rayon de la Terre en mètres
const SECONDES_PAR_ANNEE = 3600 * 24 * 365.25;

// =========================
// UTILITAIRES DOM
// =========================
function safeSetText(id, text) {
  const e = document.getElementById(id);
  // Utilisation de innerHTML car certains textes contiennent des retours à la ligne ou balises simulées
  if (e) e.innerHTML = text; 
}

// =========================
// CALCUL DISTANCE & VITESSE 3D
// =========================
function toRad(degrees) {
    return degrees * Math.PI / 180;
}

function calculerDistanceHaversine(p1, p2) {
  const φ1 = toRad(p1.latitude);
  const φ2 = toRad(p2.latitude);
  const Δφ = toRad(p2.latitude - p1.latitude);
  const Δλ = toRad(p2.longitude - p1.longitude);
  
  const a = Math.sin(Δφ / 2) ** 2 + 
            Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ / 2) ** 2;
            
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
  return R_TERRE * c; // distance horizontale en mètres
}

function calculerVitesse3D(gps) {
  if (!positionPrecedente) {
    positionPrecedente = gps;
    return 0;
  }
  const dt = (gps.timestamp - positionPrecedente.timestamp) / 1000;
  
  if (dt <= 0) {
      // Ignorer les mises à jour trop rapides ou les erreurs de timestamp
      positionPrecedente = gps;
      return 0;
  }
  
  const dHorizontal = calculerDistanceHaversine(gps, positionPrecedente);
  
  // Utiliser 0 si l'altitude n'est pas disponible pour éviter NaN
  const alt1 = positionPrecedente.altitude ?? 0;
  const alt2 = gps.altitude ?? 0;
  const dz = alt2 - alt1;
  
  // Distance totale (3D) = sqrt(dx² + dy² + dz²)
  const distance = Math.sqrt(dHorizontal ** 2 + dz ** 2);
  
  distanceTotale += distance;
  positionPrecedente = gps;
  
  return (distance / dt) * 3.6; // Vitesse en km/h
}

// =========================
// AFFICHAGE
// =========================
function afficherVitesse(v) {
  const moyenne = vitesses.length ? vitesses.reduce((a, b) => a + b, 0) / vitesses.length : 0;
  vitesseMax = Math.max(vitesseMax, v);
  
  safeSetText('vitesse', `Vitesse instantanée : ${v.toFixed(2)} km/h`);
  safeSetText('vitesse-moy', `Vitesse moyenne : ${moyenne.toFixed(2)} km/h`);
  safeSetText('vitesse-max', `Vitesse max : ${vitesseMax.toFixed(2)} km/h`);
  
  const mps = v / 3.6;
  safeSetText('vitesse-ms', `Vitesse : ${mps.toFixed(2)} m/s | ${(mps*1000).toFixed(0)} mm/s`);
  
  const pctLumiere = (mps / VITESSE_LUMIERE * 100);
  const pctSon = (mps / VITESSE_SON * 100);
  safeSetText('pourcentage', `% Lumière : ${pctLumiere.toExponential(2)}% | % Son : ${pctSon.toFixed(2)}%`);
}

function afficherDistance() {
  const km = distanceTotale / 1000;
  safeSetText('distance', `Distance parcourue : ${km.toFixed(3)} km`);
  
  const secLumiere = distanceTotale / VITESSE_LUMIERE;
  const al = secLumiere / SECONDES_PAR_ANNEE;
  safeSetText('distance-cosmique', `Distance cosmique : ${secLumiere.toFixed(3)} s lumière | ${al.toExponential(3)} al`);
}

function afficherTemps() {
  if (!tempsDebut) return;
  const t = (Date.now() - tempsDebut) / 1000;
  safeSetText('temps', `Temps : ${t.toFixed(2)} s`);
}

function afficherGPS(g) {
  safeSetText('gps', `GPS : Lat ${g.latitude.toFixed(6)} | Lon ${g.longitude.toFixed(6)} | Alt ${g.altitude ? g.altitude.toFixed(2) : '--'} m | Précision ${g.accuracy ? g.accuracy.toFixed(1) : '--'} m`);
}

function afficherCapteurs() {
  safeSetText('capteurs', `Acc XYZ : ${capteursEtat.x.toFixed(2)}, ${capteursEtat.y.toFixed(2)}, ${capteursEtat.z.toFixed(2)}`);
}

// =========================
// GESTION GPS & CAPTEURS
// =========================
function miseAJourPosition(pos) {
  const gps = {
    latitude: pos.coords.latitude,
    longitude: pos.coords.longitude,
    altitude: pos.coords.altitude,
    accuracy: pos.coords.accuracy,
    timestamp: pos.timestamp
  };
  
  // Calculer la vitesse 3D et mettre à jour la distance
  const vitesse = calculerVitesse3D(gps);
  
  // Limite haute pour éviter les valeurs erronées (vitesse > lumière)
  if (vitesse >= 0 && vitesse < VITESSE_LUMIERE * 3.6) {
    vitesses.push(vitesse);
    if (vitesses.length > 60) vitesses.shift(); // Moyenne sur 60 points
    
    afficherVitesse(vitesse);
    afficherDistance();
    // Le temps est mis à jour par l'intervalle, mais on peut le rafraîchir ici aussi.
    afficherTemps();
  }
  
  afficherGPS(gps);
}

function handleDeviceMotion(e) {
  capteursEtat.x = e.accelerationIncludingGravity?.x ?? 0;
  capteursEtat.y = e.accelerationIncludingGravity?.y ?? 0;
  capteursEtat.z = e.accelerationIncludingGravity?.z ?? 0;
  afficherCapteurs();
}

// =========================
// DÉMARRAGE / ARRÊT / RESET
// =========================
function demarrerCockpit() {
  if (!("geolocation" in navigator)) { 
    safeSetText('gps','GPS non disponible'); 
    return; 
  }
  
  // Sécurité HTTPS
  if (window.location.protocol !== 'https:') {
    alert("⚠️ Le GPS et les capteurs exigent une connexion sécurisée (HTTPS). Veuillez utiliser un serveur sécurisé.");
    safeSetText('gps', 'GPS : **HTTPS REQUIS**');
    return;
  }
  
  if (isRunning) {
      arreterCockpit();
      return;
  }
  
  isRunning = true;
  tempsDebut = Date.now();
  
  // Démarrage du suivi GPS
  watchId = navigator.geolocation.watchPosition(
    miseAJourPosition,
    err => { 
      safeSetText('gps',`Erreur GPS (${err.code}): ${err.message}`); 
    },
    { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 }
  );
  
  // Capteurs de mouvement (Accéléromètre)
  if ('DeviceMotionEvent' in window) {
      window.addEventListener('devicemotion', handleDeviceMotion);
  } else {
      safeSetText('capteurs', 'Acc XYZ : **API non supportée**');
  }

  // Mettre à jour le temps et les capteurs (pour la batterie/simulations)
  setInterval(afficherTemps, 100);
  
  document.getElementById('marche').textContent = '⏹️ Arrêt';
}

function arreterCockpit() {
  if (watchId !== null) navigator.geolocation.clearWatch(watchId);
  if ('DeviceMotionEvent' in window) {
      window.removeEventListener('devicemotion', handleDeviceMotion);
  }
  
  watchId = null; 
  positionPrecedente = null; 
  isRunning = false;
  
  safeSetText('vitesse', 'Vitesse instantanée : 0.00 km/h (ARRÊT)'); 
  safeSetText('gps', 'GPS : --');
  
  document.getElementById('marche').textContent = '▶️ Marche';
}

function resetVitesseMax() {
  vitesseMax = 0;
  safeSetText('vitesse-max','Vitesse max : 0.00 km/h');
}

// =========================
// INITIALISATION DU DOM
// =========================
document.addEventListener('DOMContentLoaded', () => {
    document.getElementById('marche').addEventListener('click', demarrerCockpit);
    document.getElementById('reset-max').addEventListener('click', resetVitesseMax);
    
    // Initialisation des affichages
    afficherVitesse(0);
    afficherDistance();
    safeSetText('temps', 'Temps : 0.00 s');
    afficherCapteurs();
    
    // Alerte si l'utilisateur n'est pas en HTTPS
    if (window.location.protocol !== 'https:') {
        safeSetText('gps', 'GPS : **HTTPS REQUIS** - Cliquez sur Marche pour vérifier');
    }
});
