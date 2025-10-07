// ========================
// PARAMÈTRES
// ========================
let watchId=null;
let positionPrecedente=null;
let distanceTotale=0;
let vitesseMax=0;
let vitessesFiltrees=[];
let tempsDebut=null;
let accel={z:0};
let refreshHz=10; // fréquence en Hz
let lastUpdateTime=0;

let capteursActifs={
  accel:true,
  baro:true,
  gps:true,
  orientation:true
};

// ========================
// UTILS DOM & MATHS
// ========================
function safeSetText(id,text){
  const e=document.getElementById(id);
  if(e)e.textContent=text;
}

const R_TERRE=6371e3;
function toRad(deg){return deg*Math.PI/180;}
function haversine3D(p1,p2){
  const φ1=toRad(p1.latitude), φ2=toRad(p2.latitude);
  const Δφ=toRad(p2.latitude-p1.latitude), Δλ=toRad(p2.longitude-p1.longitude);
  const a=Math.sin(Δφ/2)**2+Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2;
  const d2D=R_TERRE*2*Math.atan2(Math.sqrt(a),Math.sqrt(1-a));
  const dz=(p2.altitude??0)-(p1.altitude??0);
  return Math.sqrt(d2D*d2D+dz*dz);
}

// ========================
// VITESSE
// ========================
function calculerVitesse3D(pos){
  if(!positionPrecedente){positionPrecedente=pos; return {vitesse:0,distance:0,dt:0};}
  const dt=(pos.timestamp-positionPrecedente.timestamp)/1000;
  if(dt<=0)return {vitesse:0,distance:0,dt:0};
  const dist=haversine3D(positionPrecedente,pos);
  positionPrecedente=pos;
  return {vitesse:dist/dt,distance:dist,dt:dt};
}

function vitesseFiltrée(vBrute){
  const alpha=0.3;
  if(vitessesFiltrees.length===0){vitessesFiltrees.push(vBrute); return vBrute;}
  const vPrev=vitessesFiltrees[vitessesFiltrees.length-1];
  const vFilt=alpha*vBrute+(1-alpha)*vPrev;
  vitessesFiltrees.push(vFilt);
  if(vitessesFiltrees.length>60)vitessesFiltrees.shift();
  return vFilt;
}

// ========================
// MISE À JOUR GPS
// ========================
function miseAJour(pos){
  const now=pos.timestamp;
  if(now-lastUpdateTime<1000/refreshHz)return;
  lastUpdateTime=now;

  const gpsData={
    latitude:pos.coords.latitude,
    longitude:pos.coords.longitude,
    altitude:pos.coords.altitude,
    accuracy:pos.coords.accuracy,
    timestamp:now
  };

  // Vitesse brute
  const {vitesse:vBruteMS,distance:dDelta}=calculerVitesse3D(gpsData);
  const vBruteKMH=vBruteMS*3.6;
  distanceTotale+=dDelta;

  // Vitesse filtrée
  const vFiltKMH=vitesseFiltrée(vBruteKMH);

  // Vitesse max
  vitesseMax=Math.max(vitesseMax,vFiltKMH);

  // Précision (approx)
  const precisionPct=100-Math.min(gpsData.accuracy,100);
  
  // Affichage
  safeSetText('vitesse-brute',`Vitesse brute : ${vBruteKMH.toFixed(2)} km/h`);
  safeSetText('vitesse-filtre',`Vitesse filtrée : ${vFiltKMH.toFixed(2)} km/h`);
  safeSetText('vitesse-max',`Vitesse max : ${vitesseMax.toFixed(2)} km/h`);
  safeSetText('distance',`Distance totale : ${(distanceTotale/1000).toFixed(3)} km`);
  safeSetText('temps',`Temps écoulé : ${tempsDebut?((now-tempsDebut)/1000).toFixed(2):'0.00'} s`);
  safeSetText('precision',`Précision : ${precisionPct.toFixed(1)} %`);
  safeSetText('gps',`GPS : Lat: ${gpsData.latitude.toFixed(6)} | Lon: ${gpsData.longitude.toFixed(6)} | Alt: ${gpsData.altitude??'--'} m | Précision: ${gpsData.accuracy??'--'} m`);
  safeSetText('accel-z',`Accélération Z : ${accel.z.toFixed(2)} m/s²`);
  safeSetText('capteurs',`Capteurs activés : ${Object.keys(capteursActifs).filter(k=>capteursActifs[k]).join(', ')}`);
}

// ========================
// CAPTEURS
// ========================
function activerAccelerometre(){
  if(!capteursActifs.accel) return;
  if('DeviceMotionEvent' in window){
    window.addEventListener('devicemotion',e=>{
      accel.z=e.accelerationIncludingGravity?.z??0;
    });
  }else safeSetText('accel-z','Accélération Z : **NON SUPPORTÉ**');
}

// ========================
// COCKPIT
// ========================
function demarrerCockpit(){
  if(window.location.protocol!=='https:'){ alert("⚠️ HTTPS requis"); safeSetText('gps','GPS : **HTTPS REQUIS**'); return; }
  if(!navigator.geolocation){ safeSetText('gps','GPS non disponible'); return; }
  if(watchId!==null) return;

  tempsDebut=Date.now(); positionPrecedente=null; distanceTotale=0; vitesseMax=0; vitessesFiltrees=[]; accel.z=0;

  watchId=navigator.geolocation.watchPosition(
    miseAJour,
    err=>safeSetText('gps',`Erreur GPS (${err.code}): ${err.message}`),
    {enableHighAccuracy:true,maximumAge:0,timeout:10000}
  );

  activerAccelerometre();
}

function arreterCockpit(){
  if(watchId!==null) navigator.geolocation.clearWatch(watchId);
  watchId=null; tempsDebut=null; positionPrecedente=null; distanceTotale=0; vitesseMax=0; vitessesFiltrees=[];
  safeSetText('vitesse-brute','Vitesse brute : -- km/h');
  safeSetText('vitesse-filtre','Vitesse filtrée : -- km/h');
  safeSetText('vitesse-max','Vitesse max : 0.00 km/h');
  safeSetText('distance','Distance totale : 0.000 km');
  safeSetText('temps','Temps écoulé : 0.00 s');
  safeSetText('precision','Précision : -- %');
  safeSetText('gps','GPS : Lat: -- | Lon: -- | Alt: -- m | Précision: -- m');
  safeSetText('accel-z','Accélération Z : 0.00 m/s²');
  safeSetText('capteurs','Capteurs activés : --');
}

function resetVitesseMax(){
  vitesseMax=0;
  safeSetText('vitesse-max','Vitesse max : 0.00 km/h');
}

// ========================
// INIT
// ========================
document.addEventListener('DOMContentLoaded',()=>{
  document.getElementById('marche').addEventListener('click',demarrerCockpit);
  document.getElementById('arreter').addEventListener('click',arreterCockpit);
  document.getElementById('reset').addEventListener('click',resetVitesseMax);
  arreterCockpit();
});
