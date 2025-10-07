// ========================
// ETAT GLOBAL
// ========================
let watchId = null;
let positionPrecedente = null;
let distanceTotale = 0;
let vitesseMax = 0;
let vitessesFiltrees = [];
let tempsDebut = null;

let accel = { x:0, y:0, z:0 };
let pression = null; // baromètre
let capteursEtat = {
  lumiere: '--',
  son: '--',
  magnetisme: '--',
  ultrasons: '--',
  rayonsX: '--'
};

// ========================
// KALMAN 1D
// ========================
const KALMAN_Q = 0.0001;
const KALMAN_P_INIT = 1000;
let kalmanState = { x:0, P: KALMAN_P_INIT, v:0 };
let lastKalmanTime = 0;

// ========================
// UTILS
// ========================
function safeSetText(id, text) {
  const e = document.getElementById(id);
  if(e) e.textContent = text;
}
function toRad(d){return d*Math.PI/180;}
const R_TERRE = 6371e3;
function haversine3D(p1,p2){
  const φ1 = toRad(p1.latitude), φ2 = toRad(p2.latitude);
  const Δφ = toRad(p2.latitude-p1.latitude);
  const Δλ = toRad(p2.longitude-p1.longitude);
  const a = Math.sin(Δφ/2)**2+Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2;
  const c=2*Math.atan2(Math.sqrt(a),Math.sqrt(1-a));
  const d2D = R_TERRE*c;
  const dz = (p2.altitude??0)-(p1.altitude??0);
  return Math.sqrt(d2D*d2D+dz*dz);
}

// ========================
// VITESSE BRUTE
// ========================
function calculerVitesse3D(pos){
  if(!positionPrecedente){positionPrecedente=pos;return {v:0,d:0,dt:0};}
  const dt = (pos.timestamp-positionPrecedente.timestamp)/1000;
  if(dt<=0) return {v:0,d:0,dt:0};
  const d = haversine3D(positionPrecedente,pos);
  positionPrecedente=pos;
  return {v:d/dt,d: d,dt: dt};
}

// ========================
// FILTRE KALMAN
// ========================
function kalmanFilterUpdate(posGPS, dt, accZ, accGPS){
  kalmanState.x += kalmanState.v*dt;
  kalmanState.v += accZ*dt;
  kalmanState.P += KALMAN_Q;
  const R = accGPS**2;
  const K = kalmanState.P/(kalmanState.P+R);
  kalmanState.x += K*(posGPS-kalmanState.x);
  kalmanState.P *= (1-K);
  return K;
}

// ========================
// MISE A JOUR
// ========================
function miseAJour(pos){
  const now = pos.timestamp;
  const gpsData = {
    latitude: pos.coords.latitude,
    longitude: pos.coords.longitude,
    altitude: pos.coords.altitude,
    accuracy: pos.coords.accuracy,
    timestamp: now,
    speed: pos.coords.speed
  };
  
  const {v:dVitesse,d:dDistance,dt} = calculerVitesse3D(gpsData);
  distanceTotale += dDistance;
  const vRawKmH = dVitesse*3.6;

  // Kalman
  const posGPS_m = distanceTotale;
  let dtKalman = lastKalmanTime? (now-lastKalmanTime)/1000:0;
  if(!lastKalmanTime){kalmanState.x=posGPS_m; kalmanState.v=dVitesse;}
  lastKalmanTime=now;
  const K = kalmanFilterUpdate(posGPS_m,dtKalman,accel.z,gpsData.accuracy);
  const vKalmanKmH = kalmanState.v*3.6;

  vitesseMax=Math.max(vitesseMax,vKalmanKmH);
  vitessesFiltrees.push(vKalmanKmH);
  if(vitessesFiltrees.length>60) vitessesFiltrees.shift();

  // Pourcentage vitesse parfaite (basé sur GPS vs Kalman)
  const pourcentPerfect = Math.min(100,Math.max(0,100*(vRawKmH?vKalmanKmH/vRawKmH:1)));

  // Mise à jour affichage
  safeSetText('vitesse-raw',`Vitesse GPS Brute : ${vRawKmH.toFixed(2)} km/h`);
  safeSetText('vitesse-kalman',`Vitesse Filtrée : ${vKalmanKmH.toFixed(2)} km/h`);
  safeSetText('vitesse-max',`Vitesse max : ${vitesseMax.toFixed(2)} km/h`);
  safeSetText('distance',`Distance totale : ${(distanceTotale/1000).toFixed(3)} km`);
  safeSetText('temps',`Temps écoulé : ${tempsDebut?((now-tempsDebut)/1000).toFixed(2):'0.00'} s`);
  safeSetText('vitesse-precision',`Pourcentage de vitesse parfaite : ${pourcentPerfect.toFixed(2)}%`);
  safeSetText('gps',`GPS : Lat:${gpsData.latitude.toFixed(6)} | Lon:${gpsData.longitude.toFixed(6)} | Alt:${gpsData.altitude??'--'} m | Acc:${gpsData.accuracy??'--'} m`);
  safeSetText('kalman-gain',`Gain Kalman K : ${K.toFixed(4)}`);
  safeSetText('kalman-error',`Erreur P : ${kalmanState.P.toFixed(2)} m²`);
  safeSetText('accel-z',`Accélération Z : ${accel.z.toFixed(2)} m/s²`);
  safeSetText('capteurs-utilises',`Capteurs utilisés : Lumière:${capteursEtat.lumiere}, Son:${capteursEtat.son}, Magnétisme:${capteursEtat.magnetisme}, Ultrasons:${capteursEtat.ultrasons}, Rayons X:${capteursEtat.rayonsX}`);
}

// ========================
// CAPTEURS
// ========================
function activerCapteurs(){
  if('DeviceMotionEvent' in window){
    window.addEventListener('devicemotion',e=>{accel.z=e.accelerationIncludingGravity?.z??0;});
  }
  if('AmbientLightSensor' in window){
    try{const s=new AmbientLightSensor(); s.onreading=()=>capteursEtat.lumiere=s.illuminance; s.start();}
    catch{capteursEtat.lumiere='--';}
  }
  if('Magnetometer' in window){
    try{const m=new Magnetometer(); m.onreading=()=>capteursEtat.magnetisme=Math.round(Math.sqrt(m.x*m.x+m.y*m.y+m.z*m.z)); m.start();}
    catch{capteursEtat.magnetisme='--';}
  }
  if('Barometer' in window){ /* API expérimentale */ }
  // Ultrasons et Rayons X simulés si non accessibles
}

// ========================
// DÉMARRER / ARRÊTER
// ========================
function demarrerCockpit(){
  if(window.location.protocol!=='https:'){alert("⚠️ HTTPS requis"); return;}
  if(!navigator.geolocation){safeSetText('gps','GPS non dispo'); return;}
  if(watchId!==null) return;

  tempsDebut=Date.now();
  kalmanState={x:0,P:KALMAN_P_INIT,v:0};
  lastKalmanTime=0;
  vitesseMax=0; vitessesFiltrees=[]; distanceTotale=0; positionPrecedente=null;

  watchId=navigator.geolocation.watchPosition(miseAJour,err=>safeSetText('gps',`Erreur GPS(${err.code})`),{enableHighAccuracy:true,maximumAge:0,timeout:10000});
  activerCapteurs();
}
function arreterCockpit(){
  if(watchId!==null) navigator.geolocation.clearWatch(watchId);
  watchId=null; positionPrecedente=null; tempsDebut=null;
  vitesseMax=0; vitessesFiltrees=[]; distanceTotale=0;
  safeSetText('temps','Temps écoulé : 0.00 s');
  safeSetText('vitesse-raw','Vitesse GPS Brute : -- km/h');
  safeSetText('vitesse-kalman','Vitesse Filtrée : -- km/h');
  safeSetText('vitesse-max','Vitesse max : 0.00 km/h');
  safeSetText('distance','Distance totale : 0.000 km');
  safeSetText('vitesse-precision','Pourcentage de vitesse parfaite : --%');
  safeSetText('gps','GPS : Lat: -- | Lon: -- | Alt: -- m | Acc: -- m');
  safeSetText('kalman-gain','Gain Kalman K : --');
  safeSetText('kalman-error','Erreur P : --');
  safeSetText('accel-z','Accélération Z : 0.00 m/s²');
  safeSetText('capteurs-utilises','Capteurs utilisés : --');
}
function resetVitesseMax(){vitesseMax=0; safeSetText('vitesse-max','Vitesse max : 0.00 km/h');}

// ========================
// DOM READY
// ========================
document.addEventListener('DOMContentLoaded',()=>{
  document.getElementById('marche').addEventListener('click',demarrerCockpit);
  document.getElementById('arreter').addEventListener('click',arreterCockpit);
  document.getElementById('reset').addEventListener('click',resetVitesseMax);
  arreterCockpit();
});
                            
