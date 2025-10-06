let watchId = null;
let posPrev = null;
let vitesseMax = 0;
let vitesses = [];
let distanceTotale = 0;
let tempsDebut = null;

const VITESSE_LUMIERE = 299792458;
const VITESSE_SON = 343;
const R_TERRE = 6371e3;
const ANNEE_LUMIERE_SECONDES = 3600*24*365.25;

function safeSetText(id, text) {
  const el = document.getElementById(id);
  if(el) el.textContent = text;
}

function calculerDistance(p1, p2) {
  const φ1 = p1.latitude * Math.PI / 180;
  const φ2 = p2.latitude * Math.PI / 180;
  const Δφ = (p2.latitude - p1.latitude) * Math.PI / 180;
  const Δλ = (p2.longitude - p1.longitude) * Math.PI / 180;
  const a = Math.sin(Δφ/2)**2 + Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2;
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
  return R_TERRE * c;
}

function calculerVitesse(p) {
  if(!posPrev) { posPrev = p; return 0; }
  const dt = (p.timestamp - posPrev.timestamp)/1000;
  const d = calculerDistance(p,posPrev);
  if(!Number.isFinite(d) || d<0 || dt<=0) { posPrev = p; return 0; }
  distanceTotale += d;
  posPrev = p;
  return (d/dt)*3.6;
}

function afficherVitesse(v) {
  const mps = v/3.6;
  const mmps = mps*1000;
  const moy = vitesses.length ? vitesses.reduce((a,b)=>a+b,0)/vitesses.length : 0;
  safeSetText('vitesse', `Instant : ${v.toFixed(2)} km/h`);
  safeSetText('vitesse-moy', `Moyenne : ${moy.toFixed(2)} km/h`);
  safeSetText('vitesse-max', `Max : ${vitesseMax.toFixed(2)} km/h`);
  safeSetText('vitesse-h', `Horizontale : ${mps.toFixed(2)} m/s`);
  safeSetText('vitesse-v', `Verticale : ${mmps.toFixed(0)} mm/s`);
  safeSetText('pourcentage', `% Lumière : ${(mps/VITESSE_LUMIERE*100).toExponential(2)}% | % Son : ${(mps/VITESSE_SON*100).toFixed(2)}%`);
}

function afficherDistance() {
  const km = distanceTotale/1000;
  const secLumiere = distanceTotale/VITESSE_LUMIERE;
  const al = secLumiere/ANNEE_LUMIERE_SECONDES;
  safeSetText('distance', `Distance : ${km.toFixed(3)} km | ${distanceTotale.toFixed(0)} m`);
  safeSetText('distance-cosmique', `Cosmique : ${secLumiere.toFixed(3)} s lumière | ${al.toExponential(3)} al`);
}

function afficherTemps() {
  if(!tempsDebut) return;
  const t = (Date.now()-tempsDebut)/1000;
  safeSetText('temps', `Temps : ${t.toFixed(2)} s`);
}

function miseAJourVitesse(pos) {
  const gps = { latitude: pos.coords.latitude, longitude: pos.coords.longitude, altitude: pos.coords.altitude, accuracy: pos.coords.accuracy, timestamp: pos.timestamp, speed: pos.coords.speed };
  const vitesse_ms = (typeof gps.speed==='number' && gps.speed>=0) ? gps.speed : calculerVitesse(gps)/3.6;
  const vitesse_kmh = vitesse_ms*3.6;

  vitesseMax = Math.max(vitesseMax, vitesse_kmh);
  vitesses.push(vitesse_kmh);
  if(vitesses.length>60) vitesses.shift();

  afficherVitesse(vitesse_kmh);
  afficherDistance();
  afficherTemps();
  safeSetText('gps', `Lat : ${gps.latitude.toFixed(6)} | Lon : ${gps.longitude.toFixed(6)} | Alt : ${gps.altitude??'--'} m | Prec : ${gps.accuracy??'--'} m`);
}

export function demarrerCockpit() {
  if(!('geolocation' in navigator)) { safeSetText('gps','GPS non disponible'); return; }
  if(watchId!==null) return;
  tempsDebut = Date.now();
  watchId = navigator.geolocation.watchPosition(miseAJourVitesse, err=>safeSetText('gps','Erreur GPS'), {enableHighAccuracy:true,maximumAge:0,timeout:10000});
  document.getElementById('marche').textContent='⏹️ Arrêt';
}

export function arreterCockpit() {
  if(watchId!==null) navigator.geolocation.clearWatch(watchId);
  watchId=null;
  posPrev=null;
  vitesses=[];
  distanceTotale=0;
  vitesseMax=0;
  tempsDebut=null;
  afficherVitesse(0);
  afficherDistance();
  safeSetText('gps','GPS : --');
  document.getElementById('marche').textContent='▶️ Marche';
}

export function resetVitesseMax() {
  vitesseMax=0;
  safeSetText('vitesse-max','Max : 0.00 km/h');
}

document.addEventListener('DOMContentLoaded',()=>{
  document.getElementById('marche').addEventListener('click',()=>{ if(watchId===null) demarrerCockpit(); else arreterCockpit(); });
  document.getElementById('reset').addEventListener('click',resetVitesseMax);
});
