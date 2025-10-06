// cockpit_3d.js

// ========================
// État global
// ========================
let watchId = null;
let positionPrecedente = null; // {latitude, longitude, altitude, timestamp}
let vitesseMax = 0;
let vitesses = [];
let distanceTotale = 0; // en mètres
let tempsDebut = null;
let intervalleTemps = null;

let accel = { x: 0, y: 0, z: 0 }; // accélération (m/s²)

// Constantes physiques
const VITESSE_LUMIERE_MS = 299792458; // m/s
const VITESSE_SON_MS = 343; // m/s (approximatif)
const R_TERRE = 6371e3; // Rayon de la Terre en mètres

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
function toRad(degrees) {
    return degrees * Math.PI / 180;
}

function haversine3D(p1, p2) {
  // distance 2D (Haversine)
  const φ1 = toRad(p1.latitude);
  const φ2 = toRad(p2.latitude);
  const Δφ = toRad(p2.latitude - p1.latitude);
  const Δλ = toRad(p2.longitude - p1.longitude);
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
// Mise à jour de l'affichage
// ========================
function updateAffichage(vitesse_kmh) {
    const moyenne = vitesses.length ? vitesses.reduce((a,b)=>a+b,0)/vitesses.length : 0;
    const mps = vitesse_kmh / 3.6;
    const tempsEcoule = tempsDebut ? (Date.now() - tempsDebut)/1000 : 0;
    
    // Affichage des vitesses
    safeSetText('vitesse', `Vitesse instantanée : ${vitesse_kmh.toFixed(2)} km/h`);
    safeSetText('vitesse-moy', `Vitesse moyenne : ${moyenne.toFixed(2)} km/h`);
    safeSetText('vitesse-max', `Vitesse max : ${vitesseMax.toFixed(2)} km/h`);
    
    // Affichage distance et temps
    safeSetText('distance', `Distance totale : ${(distanceTotale/1000).toFixed(3)} km`);
    safeSetText('temps', `Temps écoulé : ${tempsEcoule.toFixed(2)} s`);
    
    // Affichage pourcentages
    const pctLumiere = (mps / VITESSE_LUMIERE_MS) * 100;
    const pctSon = (mps / VITESSE_SON_MS) * 100;
    safeSetText('pourcentage', `% Lumière : ${pctLumiere.toExponential(2)}% | % Son : ${pctSon.toFixed(2)}%`);
    
    // Affichage accéléromètre
    safeSetText('accelerometre', `Accélération Z (Correction) : ${accel.z.toFixed(2)} m/s²`);
}

function miseAJour(pos) {
  const gps = {
    latitude: pos.coords.latitude,
    longitude: pos.coords.longitude,
    altitude: pos.coords.altitude,
    accuracy: pos.coords.accuracy,
    timestamp: pos.timestamp,
    speed: pos.coords.speed // vitesse native en m/s
  };

  // 1. Calcul de la vitesse de base (km/h)
  let vitesse_kmh;
  if (gps.speed >= 0 && gps.speed !== null) {
      vitesse_kmh = gps.speed * 3.6;
      // Il faut quand même calculer la distance manuelle pour la distanceTotale
      if (positionPrecedente) {
          distanceTotale += haversine3D(positionPrecedente, gps);
      }
      positionPrecedente = gps; // Mise à jour de la position précédente
  } else {
      vitesse_kmh = calculerVitesse3D(gps); // Calcule et met à jour positionPrecedente/distanceTotale
  }

  // 2. Correction simple par accélération verticale
  // Une accélération positive constante augmente la vitesse au fil du temps (simplification).
  if (tempsDebut) {
      // (a * dt) -> correction vitesse en m/s
      const dt = (Date.now() - tempsDebut) / 1000;
      // On utilise accel.z * dt comme une correction simple du delta v.
      // NOTE: Ce n'est pas un Filtre de Kalman réel, mais une simple correction basée sur l'accélération
      vitesse_kmh += (accel.z * dt * 3.6); 
  }

  // 3. Mise à jour des statistiques
  vitesseMax = Math.max(vitesseMax, vitesse_kmh);
  vitesses.push(vitesse_kmh);
  if (vitesses.length>60) vitesses.shift();

  // 4. Affichage
  updateAffichage(vitesse_kmh);
  safeSetText('gps', `GPS : Lat: ${gps.latitude.toFixed(6)} | Lon: ${gps.longitude.toFixed(6)} | Alt: ${gps.altitude?.toFixed(1) ?? '--'} m | Précision: ${gps.accuracy?.toFixed(0) ?? '--'} m`);
}

// ========================
// Capteurs
// ========================
function activerAccelerometre() {
    if ('DeviceMotionEvent' in window) {
        window.addEventListener('devicemotion', e => {
            // L'accélération incluant la gravité est souvent la seule disponible
            accel.x = e.accelerationIncludingGravity?.x ?? 0;
            accel.y = e.accelerationIncludingGravity?.y ?? 0;
            accel.z = e.accelerationIncludingGravity?.z ?? 0; // Utiliser Z pour la correction Alt/Vitesse
        });
    } else {
        safeSetText('accelerometre', 'Accéléromètre : **NON SUPPORTÉ**');
    }
}

// ========================
// Démarrage / arrêt
// ========================
function demarrerCockpit() {
  if (window.location.protocol !== 'https:') {
    alert("⚠️ ERREUR : La géolocalisation et les capteurs nécessitent une connexion sécurisée (HTTPS).");
    safeSetText('gps', 'GPS : **HTTPS REQUIS**');
    return;
  }
  if (!navigator.geolocation) { safeSetText('gps','GPS non disponible'); return; }
  if (watchId!==null) return;

  tempsDebut = Date.now();
  vitesses=[]; distanceTotale=0; positionPrecedente=null;

  watchId = navigator.geolocation.watchPosition(
    miseAJour,
    err => safeSetText('gps',`Erreur GPS (${err.code}): ${err.message}`),
    { enableHighAccuracy:true, maximumAge:0, timeout:10000 }
  );
  
  // Démarrage de l'accéléromètre
  activerAccelerometre();
  
  // Démarrage du compteur de temps
  if (!intervalleTemps) {
      intervalleTemps = setInterval(() => {
          if (tempsDebut) safeSetText('temps', `Temps écoulé : ${((Date.now() - tempsDebut)/1000).toFixed(2)} s`);
      }, 1000);
  }
}

function arreterCockpit() {
  if (watchId!==null) navigator.geolocation.clearWatch(watchId);
  if (intervalleTemps !== null) clearInterval(intervalleTemps);
  
  watchId = null; 
  intervalleTemps = null;
  positionPrecedente = null; 
  
  // Réinitialisation des données
  vitesseMax = 0;
  vitesses = []; 
  distanceTotale = 0; 
  tempsDebut = null;

  // Réinitialisation de l'affichage
  safeSetText('temps', 'Temps écoulé : 0.00 s');
  safeSetText('vitesse','Vitesse instantanée : -- km/h');
  safeSetText('vitesse-moy','Vitesse moyenne : -- km/h');
  safeSetText('vitesse-max','Vitesse max : -- km/h');
  safeSetText('distance','Distance totale : -- km');
  safeSetText('pourcentage','% Lumière : --% | % Son : --%');
  safeSetText('gps','GPS : Lat: -- | Lon: -- | Alt: -- m | Précision: -- m');
  safeSetText('accelerometre', 'Accélération Z (Correction) : 0.00 m/s²');
}

// ========================
// Réinitialiser max
// ========================
function resetVitesseMax() { 
    vitesseMax=0; 
    safeSetText('vitesse-max','Vitesse max : 0.00 km/h'); 
}

// ========================
// Événements DOM
// ========================
document.addEventListener('DOMContentLoaded',()=>{
  document.getElementById('marche').addEventListener('click',demarrerCockpit);
  document.getElementById('arreter').addEventListener('click',arreterCockpit);
  document.getElementById('reset').addEventListener('click',resetVitesseMax);
  
  // Initialisation de l'affichage
  arreterCockpit(); // Affiche les valeurs par défaut
});
