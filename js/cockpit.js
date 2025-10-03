// cockpit.js
import * as SunCalc from 'https://cdn.jsdelivr.net/npm/suncalc@1.9.0/suncalc.js';

// ---------- Variables globales ----------
let watchId = null;
let positionPrecedente = null;
let vitesses = [];
let vitesseMax = 0;
let distanceTotale = 0;
let t0 = null;
let medaillonRAF = null;

let audioAnalyserHandle = null;
let audioStream = null;
let deviceOrientationHandler = null;
let magnetometerInstance = null;
let gyroscopeInstance = null;

// Helper simple pour DOM
const set = (id, txt) => { const el = document.getElementById(id); if(el) el.textContent = txt; };
const safeGet = id => document.getElementById(id);

// ---------- Démarrer / arrêter cockpit ----------
export function demarrerCockpit() { startAll(); }
export function arreterCockpit() { stopAll(); }

function startAll() {
  if (watchId !== null) return;
  t0 = performance.now();

  // GPS
  if ('geolocation' in navigator) {
    watchId = navigator.geolocation.watchPosition(
      pos => traiterPosition(pos.coords, pos.timestamp),
      err => set('gps', `Erreur GPS : ${err.message || err.code}`),
      { enableHighAccuracy: true, maximumAge:0, timeout:10000 }
    );
  } else { set('gps','Geolocation non supportée'); }

  loopTemps();
  activerHorloge();
  synchroniserHeureAtomique();
  activerCapteurs();
  afficherMedaillon();
}

// ---------- Arrêt ----------
function stopAll() {
  if (watchId !== null) navigator.geolocation.clearWatch(watchId);
  watchId = null; positionPrecedente = null; vitesses=[]; distanceTotale=0; vitesseMax=0;

  if(audioAnalyserHandle) { cancelAnimationFrame(audioAnalyserHandle); audioAnalyserHandle=null; }
  if(audioStream) { audioStream.getTracks().forEach(t=>t.stop()); audioStream=null; }
  if(deviceOrientationHandler) window.removeEventListener('deviceorientation', deviceOrientationHandler);
  if(magnetometerInstance) magnetometerInstance.stop();
  if(gyroscopeInstance) gyroscopeInstance.stop();
  if(medaillonRAF) cancelAnimationFrame(medaillonRAF);
}

// ---------- Temps relatif ----------
function loopTemps() {
  const el = safeGet('temps');
  if(!el) return;
  function tick() {
    if(!t0) t0=performance.now();
    const t = performance.now()-t0;
    el.textContent = `Temps : ${(t/1000).toFixed(2)} s`;
    requestAnimationFrame(tick);

    
