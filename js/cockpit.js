// ========================
// ETAT GLOBAL
// ========================
let watchId = null;
let positionPrecedente = null;
let vitesseMax = 0;
let vitesses = []; // Pour calculer la moyenne
let distanceTotale = 0;
let tempsDebut = null; // Pour le calcul du temps total

// Objet pour stocker l'état des capteurs physiques (pour affichage unifié)
let capteursEtat = {
  niveauBulle: '--',
  lumiere: '--',
  son: '--',
  magnetisme: '--'
};

// ========================
// UTILS BASE TEXT & DOM
// ========================
function safeSetText(id, text) {
  const e = document.getElementById(id);
  if (e) e.textContent = text;
}

// ========================
// GPS / VITESSE / DISTANCE
// ========================

// Constantes
const VITESSE_LUMIERE = 299792458; // m/s
const VITESSE_SON = 343; // m/s (approximatif)
const R_TERRE = 6371e3; // Rayon de la Terre en mètres
const ANNEE_LUMIERE_SECONDES = 3600 * 24 * 365.25;

function calculerDistanceHaversine(p1, p2) {
  const φ1 = p1.latitude * Math.PI / 180;
  const φ2 = p2.latitude * Math.PI / 180;
  const Δφ = (p2.latitude - p1.latitude) * Math.PI / 180;
  const Δλ = (p2.longitude - p1.longitude) * Math.PI / 180;
  const a = Math.sin(Δφ / 2) ** 2 + Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ / 2) ** 2;
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
  return R_TERRE * c; // Distance en mètres
}

function calculerVitesseManuelle(gps) {
  if (!positionPrecedente) {
    positionPrecedente = gps;
    return 0;
  }
  const dt = (gps.timestamp - positionPrecedente.timestamp) / 1000; // temps en secondes
  const d = calculerDistanceHaversine(gps, positionPrecedente); // distance en mètres

  if (!Number.isFinite(d) || d < 0 || dt <= 0) {
    positionPrecedente = gps;
    return 0;
  }

  // Si l'imprécision est trop grande, on ignore la position pour le calcul
  // Une simple vérification de l'accuracy GPS peut être ajoutée ici si nécessaire.

  distanceTotale += d;
  positionPrecedente = gps;
  return (d / dt) * 3.6; // Vitesse en km/h
}

function afficherVitesse(v_kmh) {
  const mps = v_kmh / 3.6;
  const mmps = mps * 1000;
  const moyenne_kmh = vitesses.length ? vitesses.reduce((a, b) => a + b, 0) / vitesses.length : 0;

  // Affichage Vitesse
  safeSetText('vitesse', `Vitesse instantanée : ${v_kmh.toFixed(4)} km/h`);
  safeSetText('vitesse-moy', `Vitesse moyenne : ${moyenne_kmh.toFixed(4)} km/h`);
  safeSetText('vitesse-max', `Vitesse max : ${vitesseMax.toFixed(4)} km/h`);
  safeSetText('vitesse-ms', `Vitesse : ${mps.toFixed(4)} m/s | ${mmps.toFixed(2)} mm/s`);

  // Affichage Pourcentage
  safeSetText('pourcentage', `% Lumière : ${(mps / VITESSE_LUMIERE * 100).toExponential(2)}% | % Son : ${(mps / VITESSE_SON * 100).toFixed(2)}%`);
}

function afficherDistance() {
  const km = distanceTotale / 1000;
  const m = distanceTotale;
  const secLumiere = m / VITESSE_LUMIERE;
  const al = secLumiere / ANNEE_LUMIERE_SECONDES;

  // Affichage Distance
  safeSetText('distance', `Distance : ${km.toFixed(3)} km | ${m.toFixed(3)} m | ${Math.round(m * 1000)} mm`);
  safeSetText('distance-cosmique', `Distance cosmique : ${secLumiere.toFixed(3)} s lumière | ${al.toExponential(3)} al`);
}

function afficherTemps() {
  if (!tempsDebut) return;
  const tempsEcoule = (Date.now() - tempsDebut) / 1000;
  safeSetText('temps', `Temps : ${tempsEcoule.toFixed(2)} s`);
}


function miseAJourVitesse(pos) {
  const gps = {
    latitude: pos.coords.latitude,
    longitude: pos.coords.longitude,
    altitude: pos.coords.altitude,
    accuracy: pos.coords.accuracy,
    timestamp: pos.timestamp,
    speed: pos.coords.speed
  };

  // Priorité à la vitesse native (m/s) convertie en km/h
  const vitesse_ms = (typeof gps.speed === 'number' && gps.speed >= 0) ? gps.speed : calculerVitesseManuelle(gps) / 3.6;
  const vitesse_kmh = vitesse_ms * 3.6;

  // Mise à jour de la distance si vitesse native est utilisée.
  // NOTE: On ne peut pas calculer la distance précisément SANS la position précédente,
  // donc si gps.speed est utilisé, on DOIT aussi faire le calcul manuel pour la distance.
  if (typeof gps.speed === 'number' && gps.speed >= 0) {
    if (positionPrecedente) {
      const d = calculerDistanceHaversine(gps, positionPrecedente);
      distanceTotale += d;
    }
    positionPrecedente = gps; // Met à jour pour le prochain calcul de distance
  }
  // Si vitesse manuelle, distanceTotale est déjà mise à jour dans calculerVitesseManuelle

  if (vitesse_kmh >= 0 && vitesse_kmh < 3000000) { // On met une limite haute raisonnable
    vitesseMax = Math.max(vitesseMax, vitesse_kmh);
    vitesses.push(vitesse_kmh);
    // Limite l'array de vitesse pour la moyenne (par exemple, aux 60 dernières)
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
  
  // Utilise miseAJourVitesse comme callback
  watchId = navigator.geolocation.watchPosition(
    miseAJourVitesse,
    err => {
      console.warn('GPS refusé ou erreur', err);
      safeSetText('gps', 'Erreur GPS: ' + err.code);
    }, {
      enableHighAccuracy: true,
      maximumAge: 0,
      timeout: 10000
    }
  );
  // Démarre aussi les capteurs physiques
  activerCapteurs();
  document.getElementById('marche').textContent = '⏹️ Arrêt';
  document.getElementById('stop').style.display = 'none'; // Cache le bouton Arrêt, on utilise Marche/Arrêt
}

function arreterCockpit() {
  if (watchId !== null) navigator.geolocation.clearWatch(watchId);
  watchId = null;
  positionPrecedente = null;
  vitesses = [];
  distanceTotale = 0;
  tempsDebut = null; // Stoppe le compteur de temps
  
  // Réinitialisation des affichages
  const defaultGPS = 'GPS : --';
  safeSetText('temps', 'Temps : 0.00 s');
  afficherVitesse(0);
  afficherDistance();
  safeSetText('gps', defaultGPS);
  
  // Réinitialisation des capteurs virtuels/physiques
  capteursEtat = {
    niveauBulle: '--',
    lumiere: '--',
    son: '--',
    magnetisme: '--'
  };
  afficherCapteurs();
  
  document.getElementById('marche').textContent = '▶️ Marche';
  document.getElementById('stop').style.display = 'inline-block'; // Réaffiche le bouton Arrêt
}

function resetVitesseMax() {
  vitesseMax = 0;
  // Met à jour l'affichage Max
  safeSetText('vitesse-max', 'Vitesse max : 0.00 km/h');
}

// ... (Les fonctions afficherGPS, afficherCapteurs, activerCapteurs, activerHorloge, afficherMedaillon
//     doivent être conservées ou adaptées pour utiliser safeSetText sur les IDs corrects) ...

function afficherGPS(g) {
  safeSetText('gps', `GPS : Latitude : ${g.latitude.toFixed(6)} | Longitude : ${g.longitude.toFixed(6)} | Altitude : ${g.altitude?.toFixed(0)??'--'} m | Précision GPS : ${g.accuracy?.toFixed(0)??'--'} m`);
}

// ========================
// AFFICHAGE CAPTEURS PHYSIQUES (Simplifié pour l'exemple, utiliser les IDs du HTML)
// ========================
function afficherCapteurs() {
  safeSetText('capteurs', 
    `Lumière : ${capteursEtat.lumiere} lux | Son : ${capteursEtat.son} dB | Niveau : ${capteursEtat.niveauBulle}° | Magnétomètre : ${capteursEtat.magnetisme} µT | Batterie : --% | Réseau : --`
  );
}

// NOTE: La fonction 'activerCapteurs' n'a pas été modifiée car elle était déjà fonctionnelle 
// pour la démo, mais elle doit être bien intégrée dans le DOMContentLoaded.

// ========================
// INIT CLICK & START
// ========================
document.addEventListener('DOMContentLoaded', () => {
  // Initialisation des affichages
  afficherVitesse(0);
  afficherDistance();
  safeSetText('temps', 'Temps : 0.00 s');

  // Mise en place des événements click
  document.getElementById('marche').addEventListener('click', demarrerCockpit);
  document.getElementById('stop').addEventListener('click', arreterCockpit);
  document.getElementById('reset').addEventListener('click', resetVitesseMax);
  
  // On peut retirer l'affichage du médaillon et l'horloge des fonctions de base 
  // si on veut les activer au démarrage.
});
