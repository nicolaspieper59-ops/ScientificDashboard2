// ========================
// PARAMÈTRES KALMAN
// ========================
const KALMAN_Q = 0.0001;
const KALMAN_P_INIT = 1000;

// ========================
// ETAT GLOBAL
// ========================
let watchId = null;
let positionPrecedente = null;
let vitesseMax = 0;
let vitessesFiltrees = [];
let distanceTotale = 0;
let tempsDebut = null;
let accel = { x:0, y:0, z:0 };
let gyro = { x:0, y:0, z:0 };
let magnetometre = { x:0, y:0, z:0 };

// Etat Kalman 1D
let kalmanState = { x:0, P:KALMAN_P_INIT, v:0 };
let lastKalmanTime = 0;

// Suivi des capteurs
let capteursUtilises = {
  gps: false,
  accel: false,
  gyro: false,
  magnetometre: false
};

// ========================
// UTILS
// ========================
function safeSetText(id, text) {
  const e = document.getElementById(id);
  if(e) e.textContent = text;
}

function toRad(degrees) { return degrees*Math.PI/180; }

function haversine3D(p1,p2) {
  const φ1 = toRad(p1.latitude), φ2 = toRad(p2.latitude);
  const Δφ = toRad(p2.latitude-p1.latitude), Δλ = toRad(p2.longitude-p1.longitude);
  const a = Math.sin(Δφ/2)**2 + Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2;
  const c = 2*Math.atan2(Math.sqrt(a),Math.sqrt(1-a));
  const d2D = 6371e3 * c;
  const dz = (p2.altitude??0)-(p1.altitude??0);
  return Math.sqrt(d2D*d2D + dz*dz);
}

function calculerVitesse3D(pos) {
  if(!positionPrecedente){
    positionPrecedente = pos;
    return {vitesse:0,distance:0,dt:0};
  }
  const dt = (pos.timestamp-positionPrecedente.timestamp)/1000;
  if(dt<=0) return {vitesse:0,distance:0,dt:0};
  const dist = haversine3D(positionPrecedente,pos);
  positionPrecedente = pos;
  return {vitesse:dist/dt,distance:dist,dt:dt};
}

// ========================
// FILTRE KALMAN
// ========================
function kalmanFilterUpdate(posGPS, dt, accZ, accGPS){
  kalmanState.x += kalmanState.v*dt;
  kalmanState.v += accZ*dt;
  kalmanState.P += KALMAN_Q;
  const R = accGPS*accGPS;
  const K = kalmanState.P/(kalmanState.P+R);
  kalmanState.x += K*(posGPS-kalmanState.x);
  kalmanState.P *= (1-K);
  return K;
}

// ========================
// MISE A JOUR GPS
// ========================
function miseAJour(pos){
  const now = pos.timestamp;
  capteursUtilises.gps = true;

  const gpsData = {
    latitude: pos.coords.latitude,
    longitude: pos.coords.longitude,
    altitude: pos.coords.altitude,
    accuracy: pos.coords.accuracy,
    timestamp: now,
    speed: pos.coords.speed
  };

  const {vitesse:v_raw_ms,distance:d_delta} = calculerVitesse3D(gpsData);
  distanceTotale += d_delta;

  const posGPS_m = distanceTotale;
  let dt_kalman = lastKalmanTime? (now-lastKalmanTime)/1000:0;
  if(!lastKalmanTime){ kalmanState.x=posGPS_m; kalmanState.v=v_raw_ms; }
  lastKalmanTime = now;

  const K = kalmanFilterUpdate(posGPS_m,dt_kalman,accel.z,gpsData.accuracy);
  const v_kmh = kalmanState.v*3.6;

  vitesseMax = Math.max(vitesseMax,v_kmh);
  vitessesFiltrees.push(v_kmh);
  if(vitessesFiltrees.length>60) vitessesFiltrees.shift();

  // Affichage
  safeSetText('vitesse-kalman',`Vitesse Filtrée : ${v_kmh.toFixed(2)} km/h`);
  safeSetText('vitesse-raw',`Vitesse GPS Brute : ${(v_raw_ms*3.6).toFixed(2)} km/h`);
  safeSetText('vitesse-max',`Vitesse max : ${vitesseMax.toFixed(2)} km/h`);
  safeSetText('distance',`Distance totale : ${(distanceTotale/1000).toFixed(3)} km`);
  safeSetText('temps',`Temps écoulé : ${tempsDebut?((now-tempsDebut)/1000).toFixed(2):'0.00'} s`);
  safeSetText('gps',`GPS : Lat:${gpsData.latitude.toFixed(6)} Lon:${gpsData.longitude.toFixed(6)} Alt:${gpsData.altitude?.toFixed(1)??'--'} m Précision:${gpsData.accuracy?.toFixed(0)??'--'} m`);
  safeSetText('kalman-gain',`Gain Kalman : ${K.toFixed(4)}`);
  safeSetText('kalman-error',`Erreur P : ${kalmanState.P.toFixed(2)} m²`);
  afficherEtatCapteurs();
}

// ========================
// CAPTEURS
// ========================
function activerAccelerometre(){
  if('DeviceMotionEvent' in window){
    capteursUtilises.accel=true;
    window.addEventListener('devicemotion',e=>{accel.x=e.accelerationIncludingGravity?.x??0; accel.y=e.accelerationIncludingGravity?.y??0; accel.z=e.accelerationIncludingGravity?.z??0;});
  }
}

function activerGyroscope(){
  if('Gyroscope' in window){
    capteursUtilises.gyro=true;
    const g = new Gyroscope({frequency:60});
    g.addEventListener('reading',()=>{gyro.x=g.x; gyro.y=g.y; gyro.z=g.z;});
    g.start();
  }
}

function activerMagnetometre(){
  if('Magnetometer' in window){
    capteursUtilises.magnetometre=true;
    const m = new Magnetometer({frequency:60});
    m.addEventListener('reading',()=>{magnetometre.x=m.x; magnetometre.y=m.y; magnetometre.z=m.z;});
    m.start();
  }
}

// ========================
// CONTROLE
// ========================
function demarrerCockpit(){
  if(window.location.protocol!=='https:'){ alert("HTTPS requis"); return; }
  if(!navigator.geolocation){ safeSetText('gps','GPS non disponible'); return; }
  if(watchId!==null) return;

  tempsDebut = Date.now();
  kalmanState={x:0,P:KALMAN_P_INIT,v:0}; lastKalmanTime=0;
  vitesseMax=0; vitessesFiltrees=[]; distanceTotale=0; positionPrecedente=null;

  watchId = navigator.geolocation.watchPosition(miseAJour, err=>safeSetText('gps',`Erreur GPS (${err.code}): ${err.message}`),{enableHighAccuracy:true,maximumAge:0,timeout:10000});
  activerAccelerometre(); activerGyroscope(); activerMagnetometre();
}

function arreterCockpit(){
  if(watchId!==null) navigator.geolocation.clearWatch(watchId);
  watchId=null; positionPrecedente=null; tempsDebut=null;

  // Reset affichage
  safeSetText('temps','Temps écoulé : 0.00 s');
  safeSetText('vitesse-kalman','Vitesse Filtrée : -- km/h');
  safeSetText('vitesse-raw','Vitesse GPS Brute : -- km/h');
  safeSetText('vitesse-max','Vitesse max : 0.00 km/h');
  safeSetText('distance','Distance totale : 0.000 km');
  safeSetText('gps','GPS : Lat: -- | Lon: -- | Alt: -- m | Précision: -- m');
  safeSetText('kalman-gain','Gain Kalman : --'); safeSetText('kalman-error','Erreur P : --');
  safeSetText('accel-x','Accélération X : 0.00'); safeSetText('accel-y','Accélération Y : 0.00'); safeSetText('accel-z','Accélération Z : 0.00');
  safeSetText('gyro-x','Gyro X : 0.00'); safeSetText('gyro-y','Gyro Y : 0.00'); safeSetText('gyro-z','Gyro Z : 0.00');
  safeSetText('magnet-x','Magnet X : 0.00'); safeSetText('magnet-y','Magnet Y : 0.00'); safeSetText('magnet-z','Magnet Z : 0.00');

  afficherEtatCapteurs();
}

function resetVitesseMax(){ vitesseMax=0; safeSetText('vitesse-max','Vitesse max : 0.00 km/h'); }

function afficherEtatCapteurs(){
  safeSetText('etat-capteurs',
    `GPS:${capteursUtilises.gps?'✅':'❌'} | Accel:${capteursUtilises.accel?'✅':'❌'} | Gyro:${capteursUtilises.gyro?'✅':'❌'} | Magnet:${capteursUtilises.magnetometre?'✅':'❌'}`
  );
}

// ========================
// DOM READY
// ========================
document.addEventListener('DOMContentLoaded',()=>{
  document.getElementById('marche').addEventListener('click',demarrerCockpit);
  document.getElementById('arreter').addEventListener('click',arreterCockpit);
  document.getElementById('reset').addEventListener('click',resetVitesseMax);
  arreterCockpit();
});
