// cockpit.js
import SunCalc from 'https://cdn.jsdelivr.net/npm/suncalc@1.9.0/suncalc.js';

let watchId = null;
let positionPrecedente = null;
let distanceTotale = 0;
let vitesseMax = 0;
let vitesses = [];
let destination = { latitude: null, longitude: null };

// Pour simulation physique
let masseSimulee = 70; // kg par défaut

/* ========================
   GPS & Vitesse
======================== */
export function demarrerCockpit() {
  if (!('geolocation' in navigator)) {
    safeSetText('gps', 'GPS non disponible');
    return;
  }
  if (watchId !== null) return;

  watchId = navigator.geolocation.watchPosition(
    pos => {
      const gps = pos.coords;
      const vitesse = (gps.speed != null ? gps.speed * 3.6 : calculerVitesse(pos)) || 0;
      if (vitesse >= 0 && vitesse < 300) {
        vitesseMax = Math.max(vitesseMax, vitesse);
        vitesses.push(vitesse);

        afficherVitesse(vitesse);
        afficherDistance();
        afficherPourcentage(vitesse);
        afficherGPS(gps);
        afficherGPSBrut(gps);
        afficherSimulationPhysique(vitesse);
        afficherCoordMinecraft(gps);
      }
    },
    err => safeSetText('gps', 'Erreur GPS : ' + (err.message || err.code || 'inconnue')),
    { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 }
  );

  activerCapteurs();
  activerBoussole();
  activerHorloge();
  afficherMedaillon();
  chargerMeteo();
  afficherGrandeurs();
}

export function arreterCockpit() {
  if (watchId !== null && 'geolocation' in navigator) {
    navigator.geolocation.clearWatch(watchId);
  }
  watchId = null;
  positionPrecedente = null;
  vitesses = [];
  distanceTotale = 0;
  vitesseMax = 0;
}

export function resetVitesseMax() {
  vitesseMax = 0;
  appendText('vitesse', ' | Max réinitialisé');
}

export function definirDestination(lat, lon) {
  destination.latitude = lat;
  destination.longitude = lon;
}

function calculerVitesse(pos) {
  if (!positionPrecedente) {
    positionPrecedente = pos.coords;
    return 0;
  }
  const dt = (pos.timestamp - positionPrecedente.timestamp) / 1000;
  const d = calculerDistance(pos.coords, positionPrecedente);
  distanceTotale += d;
  positionPrecedente = pos.coords;
  return dt > 0 ? (d / dt) * 3.6 : 0;
