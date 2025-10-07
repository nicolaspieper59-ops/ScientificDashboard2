let watchId = null;
let positionPrev = null;
let distanceTotale = 0;
let vitesseMax = 0;
let tempsDebut = null;

function safeSetText(id,text){ const e=document.getElementById(id); if(e)e.textContent=text; }

function haversine3D(p1,p2){
  const R = 6371e3;
  const toRad = d=>d*Math.PI/180;
  const φ1=toRad(p1.latitude), φ2=toRad(p2.latitude);
  const Δφ=toRad(p2.latitude-p1.latitude), Δλ=toRad(p2.longitude-p1.longitude);
  const a=Math.sin(Δφ/2)**2+Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2;
  const c=2*Math.atan2(Math.sqrt(a),Math.sqrt(1-a));
  const d2D = R*c;
  const dz=(p2.altitude||0)-(p1.altitude||0);
  return Math.sqrt(d2D*d2D+dz*dz);
}

function miseAJour(pos){
  const now = pos.timestamp;
  const gpsData={
    latitude: pos.coords.latitude,
    longitude: pos.coords.longitude,
    altitude: pos.coords.altitude,
    accuracy: pos.coords.accuracy,
    timestamp: now,
    speed: pos.coords.speed
  };

  let dt = 0;
  if(positionPrev) dt=(now-positionPrev.timestamp)/1000;
  const dist = positionPrev? haversine3D(positionPrev,gpsData):0;
  positionPrev=gpsData;
  distanceTotale+=dist;
  const vitesseBrute = dt>0? dist/dt : 0;
  const accelZ = Sensors.getAccelZ();

  // Kalman
  if(!tempsDebut) Kalman2.reset();
  Kalman2.predict(dt, accelZ);
  const K = Kalman2.update(distanceTotale, gpsData.accuracy**2);
  const {vitesse: vitesseKalman} = Kalman2.getState();

  vitesseMax=Math.max(vitesseMax, vitesseKalman*3.6);

  safeSetText('vitesse-kalman',`Vitesse Kalman : ${(vitesseKalman*3.6).toFixed(2)} km/h`);
  safeSetText('vitesse-raw',`Vitesse brute : ${(vitesseBrute*3.6).toFixed(2)} km/h`);
  safeSetText('vitesse-max',`Vitesse max : ${vitesseMax.toFixed(2)} km/h`);
  safeSetText('distance',`Distance : ${(distanceTotale/1000).toFixed(3)} km`);
  safeSetText('temps',`Temps : ${((now-tempsDebut)/1000).toFixed(2)} s`);
  safeSetText('gps',`GPS : Lat: ${gpsData.latitude.toFixed(6)} | Lon: ${gpsData.longitude.toFixed(6)} | Alt: ${gpsData.altitude?.toFixed(1)||'--'} m | Acc: ${gpsData.accuracy?.toFixed(0)||'--'} m`);
  safeSetText('kalman-gain',`Gain : ${K.toFixed(4)}`);
}

function demarrerCockpit(){
  if(location.protocol!=='https:'){ alert('HTTPS requis'); return; }
  if(!navigator.geolocation){ safeSetText('gps','GPS non dispo'); return; }
  if(watchId!==null) return;

  tempsDebut=Date.now(); distanceTotale=0; vitesseMax=0; positionPrev=null;
  Sensors.startAccelerometer();
  watchId = navigator.geolocation.watchPosition(miseAJour, e=>safeSetText('gps','Erreur GPS'), {enableHighAccuracy:true,maximumAge:0,timeout:10000});
}

function arreterCockpit(){
  if(watchId!==null) navigator.geolocation.clearWatch(watchId);
  watchId=null; positionPrev=null; tempsDebut=null; distanceTotale=0; vitesseMax=0;
  ['temps','vitesse-kalman','vitesse-raw','vitesse-max','distance','gps','kalman-gain'].forEach(id=>safeSetText(id,'--'));
}

function resetVitesseMax(){ vitesseMax=0; safeSetText('vitesse-max','0.00 km/h'); }

document.addEventListener('DOMContentLoaded',()=>{
  document.getElementById('marche').addEventListener('click',demarrerCockpit);
  document.getElementById('arreter').addEventListener('click',arreterCockpit);
  document.getElementById('reset').addEventListener('click',resetVitesseMax);
});
