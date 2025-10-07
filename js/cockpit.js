// ========================
// CONFIG & ETAT GLOBAL
// ========================
let watchId = null;
let positionPrecedente = null;
let distanceTotale = 0;
let vitessesFiltrees = [];
let vitesseMax = 0;
let tempsDebut = null;
let lastUpdateTime = 0;
let refreshHz = 5; // fréquence en Hz

// Capteurs actifs/désactivables
const capteursActifs = {
  gps: true,
  accel: true,
  gyro: true,
  magnet: true,
  baro: true,
  lumière: true,
  son: true,
  infra: false // exemple
};

// Etat des capteurs
let accel = {x:0,y:0,z:0};
let gyro = {x:0,y:0,z:0};
let magnet = {x:0,y:0,z:0};
let baro = 0;
let lumiere = 0;
let son = 0;

// ========================
// UTILS
// ========================
function safeSetText(id,text){
  const e=document.getElementById(id);
  if(e) e.textContent=text;
}

function toRad(deg){ return deg*Math.PI/180; }

function haversine3D(p1,p2){
  const φ1=toRad(p1.latitude),φ2=toRad(p2.latitude);
  const Δφ=toRad(p2.latitude-p1.latitude),Δλ=toRad(p2.longitude-p1.longitude);
  const a=Math.sin(Δφ/2)**2+Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2;
  const c=2*Math.atan2(Math.sqrt(a),Math.sqrt(1-a));
  const d2D=6371000*c;
  const dz=(p2.altitude??0)-(p1.altitude??0);
  return Math.sqrt(d2D*d2D+dz*dz);
}

function calculerVitesse3D(pos){
  if(!positionPrecedente){ positionPrecedente=pos; return {vitesse:0,distance:0}; }
  const dt=(pos.timestamp-positionPrecedente.timestamp)/1000;
  if(dt<=0) return {vitesse:0,distance:0};
  const dist=haversine3D(positionPrecedente,pos);
  positionPrecedente=pos;
  return {vitesse:dist/dt,distance:dist};
}

function calculerPrecisionGlobale(gpsAcc){
  let total=0, actifs=0;
  for(const k in capteursActifs){
    if(capteursActifs[k]){
      actifs++;
      if(k==='gps') total+=Math.max(0,100-Math.min(100,gpsAcc));
      else total+=100;
    }
  }
  return actifs? (total/actifs).toFixed(2):0;
}

// ========================
// CAPTEURS
// ========================
function activerAccelerometre(){
  if(!capteursActifs.accel) return;
  if('DeviceMotionEvent' in window){
    window.addEventListener('devicemotion', e=>{
      accel.x=e.accelerationIncludingGravity?.x??0;
      accel.y=e.accelerationIncludingGravity?.y??0;
      accel.z=e.accelerationIncludingGravity?.z??0;
    });
  }
}

function activerGyroscope(){
  if(!capteursActifs.gyro) return;
  if('DeviceOrientationEvent' in window){
    window.addEventListener('deviceorientation', e=>{
      gyro.x=e.alpha??0;
      gyro.y=e.beta??0;
      gyro.z=e.gamma??0;
    });
  }
}

function activerMagnetometre(){ 
  // Placeholder si accès natif possible
}

function activerBarometre(){ 
  // Placeholder: Android sensor API via WebView ou plugin
}

function activerLumiere(){ 
  // Placeholder: si capteur exposé
}

function activerSon(){ 
  // Placeholder: micro
}

function activerInfra(){ 
  // Placeholder: non standard
}

// ========================
// MISE A JOUR
// ========================
function miseAJour(pos){
  const now=pos.timestamp;
  if(lastUpdateTime && (now-lastUpdateTime)<(1000/refreshHz)) return;
  lastUpdateTime=now;

  const gpsData={
    latitude:pos.coords.latitude,
    longitude:pos.coords.longitude,
    altitude:pos.coords.altitude,
    accuracy:pos.coords.accuracy,
    timestamp:now,
    speed:pos.coords.speed
  };

  const {vitesse:vBrute_ms,distance:dDelta}=calculerVitesse3D(gpsData);
  const vBrute_kmh=vBrute_ms*3.6;
  distanceTotale+=dDelta;

  if(vBrute_kmh>0){
    vitessesFiltrees.push(vBrute_kmh);
    if(vitessesFiltrees.length>10) vitessesFiltrees.shift();
  }
  const vFiltre=vitessesFiltrees.length?vitessesFiltrees.reduce((a,b)=>a+b,0)/vitessesFiltrees.length:0;
  vitesseMax=Math.max(vitesseMax,vFiltre);

  const precision=calculerPrecisionGlobale(gpsData.accuracy);

  // Affichage
  safeSetText('vitesse-brute',`Vitesse brute : ${vBrute_kmh.toFixed(2)} km/h`);
  safeSetText('vitesse-filtre',`Vitesse filtrée : ${vFiltre.toFixed(2)} km/h`);
  safeSetText('vitesse-max',`Vitesse max : ${vitesseMax.toFixed(2)} km/h`);
  safeSetText('distance',`Distance totale : ${(distanceTotale/1000).toFixed(3)} km`);
  safeSetText('temps',`Temps écoulé : ${tempsDebut?((now-tempsDebut)/1000).toFixed(2):'0.00'} s`);
  safeSetText('gps',`GPS : Lat:${gpsData.latitude.toFixed(6)} Lon:${gpsData.longitude.toFixed(6)} Alt:${gpsData.altitude??'--'} m | Acc:${gpsData.accuracy??'--'} m`);
  safeSetText('accel',`Accel : x:${accel.x.toFixed(2)} y:${accel.y.toFixed(2)} z:${accel.z.toFixed(2)}`);
  safeSetText('gyro',`Gyro : x:${gyro.x.toFixed(2)} y:${gyro.y.toFixed(2)} z:${gyro.z.toFixed(2)}`);
  safeSetText('precision',`Précision globale : ${precision} %`);
  safeSetText('capteurs',`Capteurs actifs : ${Object.keys(capteursActifs).filter(k=>capteursActifs[k]).join(', ')}`);
}

// ========================
// DEMARRER / ARRETER / RESET
// ========================
function demarrerCockpit(){
  if(window.location.protocol!=='https:'){ alert("⚠️ HTTPS requis"); return; }
  if(!navigator.geolocation){ safeSetText('gps','GPS non disponible'); return; }
  if(watchId!==null) return;

  tempsDebut=Date.now(); distanceTotale=0; vitessesFiltrees=[]; vitesseMax=0; positionPrecedente=null;

  activerAccelerometre();
  activerGyroscope();
  activerMagnetometre();
  activerBarometre();
  activerLumiere();
  activerSon();
  activerInfra();

  watchId=navigator.geolocation.watchPosition(
    miseAJour,
    err=>safeSetText('gps',`Erreur GPS (${err.code}): ${err.message}`),
    {enableHighAccuracy:true,maximumAge:0,timeout:10000}
  );
}

function arreterCockpit(){
  if(watchId!==null) navigator.geolocation.clearWatch(watchId);
  watchId=null; positionPrecedente=null; tempsDebut=null; vitessesFiltrees=[]; distanceTotale=0; vitesseMax=0;

  ['vitesse-brute','vitesse-filtre','vitesse-max','distance','temps','gps','accel','gyro','precision','capteurs'].forEach(id=>safeSetText(id,`--`));
}

function resetVitesseMax(){
  vitesseMax=0;
  safeSetText('vitesse-max','0.00 km/h');
}

// ========================
// DOM READY
// ========================
document.addEventListener('DOMContentLoaded',()=>{
  document.getElementById('marche').addEventListener('click',demarrerCockpit);
  document.getElementById('arreter').addEventListener('click',arreterCockpit);
  document.getElementById('reset').addEventListener('click',resetVitesseMax);
});
  
