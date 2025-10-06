// ========================
// PARAMÈTRES KALMAN
// ========================
const KALMAN_Q = 0.0001;
const KALMAN_P_INIT = 1000;

// ========================
// ÉTAT GLOBAL
// ========================
let watchId = null;
let positionPrecedente = null;
let distanceTotale = 0;
let vitesseMax = 0;
let vitessesFiltrees = [];
let tempsDebut = null;
let accel = { z: 0 };

let kalmanState = { x: 0, P: KALMAN_P_INIT };
let lastKalmanX = 0;
let lastKalmanTime = 0;

// ========================
// UTILS
// ========================
function safeSetText(id,text){const e=document.getElementById(id);if(e)e.textContent=text;}

function haversine3D(p1,p2){
  const R=6371e3;
  const φ1=p1.latitude*Math.PI/180;
  const φ2=p2.latitude*Math.PI/180;
  const Δφ=(p2.latitude-p1.latitude)*Math.PI/180;
  const Δλ=(p2.longitude-p1.longitude)*Math.PI/180;
  const a=Math.sin(Δφ/2)**2+Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2;
  const c=2*Math.atan2(Math.sqrt(a),Math.sqrt(1-a));
  const d2D=R*c;
  const dz=(p2.altitude??0)-(p1.altitude??0);
  return Math.sqrt(d2D*d2D + dz*dz);
}

function kalmanFilterUpdate(posGPS_m, dt, R_measure){
  kalmanState.P += KALMAN_Q;
  const K = kalmanState.P/(kalmanState.P+R_measure*R_measure);
  kalmanState.x = kalmanState.x + K*(posGPS_m - kalmanState.x);
  kalmanState.P = (1-K)*kalmanState.P;
  return K;
}

// ========================
// MISE À JOUR GPS
// ========================
function miseAJour(pos){
  const now=pos.timestamp;
  const gpsData=pos.coords;

  if(positionPrecedente){
    const dt=(now-positionPrecedente.timestamp)/1000;
    if(dt<=0){positionPrecedente=pos; return;}
    const d=haversine3D(positionPrecedente,pos);
    distanceTotale+=d;

    if(lastKalmanTime===0){ lastKalmanTime=now; lastKalmanX=distanceTotale; kalmanState.x=distanceTotale; }
    const dtKal=(now-lastKalmanTime)/1000;
    const K=kalmanFilterUpdate(distanceTotale,dtKal,gpsData.accuracy);

    const vitesseKalman=(kalmanState.x-lastKalmanX)/dtKal;
    const vitesseKalmanKmh=vitesseKalman*3.6;
    lastKalmanX=kalmanState.x;
    lastKalmanTime=now;

    vitesseMax=Math.max(vitesseMax,vitesseKalmanKmh);
    vitessesFiltrees.push(vitesseKalmanKmh);
    if(vitessesFiltrees.length>60) vitessesFiltrees.shift();

    safeSetText('vitesse-raw',`Vitesse GPS brute : ${(d/dt*3.6).toFixed(2)} km/h`);
    safeSetText('vitesse-kalman',`Vitesse filtrée : ${vitesseKalmanKmh.toFixed(2)} km/h`);
    safeSetText('vitesse-max',`Vitesse max : ${vitesseMax.toFixed(2)} km/h`);
    safeSetText('distance',`Distance totale : ${(distanceTotale/1000).toFixed(3)} km`);
    safeSetText('kalman-gain',`Gain de Kalman (K) : ${K.toFixed(4)}`);
    safeSetText('kalman-error',`Erreur estimée (P) : ${kalmanState.P.toFixed(2)} m²`);
  }

  positionPrecedente=pos;
  safeSetText('gps',`Lat: ${gpsData.latitude.toFixed(6)} | Lon: ${gpsData.longitude.toFixed(6)} | Alt: ${(gpsData.altitude??0).toFixed(1)} m | Acc: ${gpsData.accuracy.toFixed(0)} m`);
}

// ========================
// CAPTEURS
// ========================
function activerAccelerometre(){
  if('DeviceMotionEvent' in window){
    window.addEventListener('devicemotion',e=>{accel.z=e.accelerationIncludingGravity?.z??0;});
  }else safeSetText('accel-z','Accélération Z : NON SUPPORTÉ');
}

// ========================
// CONTROLE
// ========================
function demarrerCockpit(){
  if(window.location.protocol!=='https:'){alert("⚠️ HTTPS requis"); safeSetText('gps','GPS : HTTPS REQUIS'); return;}
  if(!navigator.geolocation){ safeSetText('gps','GPS non disponible'); return;}
  if(watchId!==null) return;

  tempsDebut=Date.now();
  kalmanState={x:0,P:KALMAN_P_INIT};
  lastKalmanX=0; lastKalmanTime=0;
  vitesseMax=0; vitessesFiltrees=[]; distanceTotale=0; positionPrecedente=null;

  watchId=navigator.geolocation.watchPosition(miseAJour,err=>safeSetText('gps',`Erreur GPS (${err.code}): ${err.message}`),{enableHighAccuracy:true,maximumAge:0,timeout:10000});

  activerAccelerometre();
}

function arreterCockpit(){
  if(watchId!==null) navigator.geolocation.clearWatch(watchId);
  watchId=null; positionPrecedente=null; tempsDebut=null;

  safeSetText('temps','Temps écoulé : 0.00 s');
  safeSetText('vitesse-raw','Vitesse GPS brute : -- km/h');
  safeSetText('vitesse-kalman','Vitesse filtrée : -- km/h');
  safeSetText('vitesse-max','Vitesse max : 0.00 km/h');
  safeSetText('distance','Distance totale : 0.000 km');
  safeSetText('gps','Lat: -- | Lon: -- | Alt: -- m | Acc: -- m');
  safeSetText('kalman-gain','Gain de Kalman (K) : --');
  safeSetText('kalman-error','Erreur estimée (P) : --');
  safeSetText('accel-z','Accélération Z : 0.00 m/s²');
}

function resetVitesseMax(){
  vitesseMax=0;
  safeSetText('vitesse-max','Vitesse max : 0.00 km/h');
}

// ========================
// ÉVÉNEMENTS DOM
// ========================
document.addEventListener('DOMContentLoaded',()=>{
  document.getElementById('marche').addEventListener('click',demarrerCockpit);
  document.getElementById('arreter').addEventListener('click',arreterCockpit);
  document.getElementById('reset').addEventListener('click',resetVitesseMax);
  arreterCockpit();
});
      
