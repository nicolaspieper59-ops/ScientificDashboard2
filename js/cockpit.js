let watchId = null;
let positionPrecedente = null;
let vitesses = [];
let distanceTotale = 0;
let vitesseMax = 0;
let destination = { latitude: null, longitude: null };

// Simulation paramètres
let masseSimulee = 70; // kg
let vitesseSimulee = 0; // m/s
let simulationActive = false;

/* ========================
   UTILITAIRES DOM
======================== */
function safeSetText(id, text) {
  const el = document.getElementById(id);
  if (el) el.textContent = text;
}

function appendText(id, text) {
  const el = document.getElementById(id);
  if (el) el.textContent += text;
}

/* ========================
   GPS & VITESSE
======================== */
export function demarrerCockpit() {
  if ("geolocation" in navigator && !simulationActive) {
    if (watchId !== null) return;
    watchId = navigator.geolocation.watchPosition(
      pos => {
        traiterPosition(pos.coords, pos.timestamp);
      },
      err => safeSetText('gps', 'Erreur GPS : ' + (err.message || err.code || 'inconnue')),
      { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 }
    );
  } else if (simulationActive) {
    // GPS simulé
    setInterval(() => {
      const gpsSim = {
        latitude: 
       
