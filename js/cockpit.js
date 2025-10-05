// ========================
// ÉTAT GLOBAL
// ========================
let watchId = null;
let positionPrecedente = null;
let vitesseMax = 0;
let vitesses = [];
let distanceTotale = 0;
let tempsDebut = null;

// Accéléromètre
let vitesseAccel = 0;
let tempsDerniereMesureAccel = null;

// Capteurs physiques (affichage)
let capteursEtat = { niveauBulle:'--', lumiere:'--', son:'--', magnetisme:'--' };

// Constantes
const VITESSE_LUMIERE = 299792458; // m/s
const VITESSE_SON = 343; // m/s
const R_TERRE = 6371e3; // m
const ANNEE_LUMIERE_SECONDES = 3600*24*365.25;

// ========================
// UTILITAIRES DOM
// ========================
function safeSetText(id,text){
  const e = document.getElementById(id);
  if(e) e.textContent = text;
}

// ========================
// GPS / VITESSE / DISTANCE
// ========================
function calculerDistanceHaversine(p1,p2){
  const φ1=p1.latitude*Math.PI/180;
  const φ2=p2.latitude*Math.PI/180;
  const Δφ=(p2.latitude-p1.latitude)*Math.PI/180;
  const Δλ=(p2.longitude-p1.longitude)*Math.PI/180;
  const a=Math.sin(Δφ/2)**2 + Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2;
  const c=2*Math.atan2(Math.sqrt(a),Math.sqrt(1-a));
  return R_TERRE*c;
}

function calculerVitesseManuelle(gps){
  if(!positionPrecedente){ positionPrecedente=gps; return 0; }
  const dt=(gps.timestamp-positionPrecedente.timestamp)/1000;
  const d=calculerDistanceHaversine(gps,positionPrecedente);
  if(!Number.isFinite(d)||d<0||dt<=0){ positionPrecedente=gps; return 0; }
  distanceTotale+=d;
  positionPrecedente=gps;
  return (d/dt)*3.6; // km/h
}

function afficherVitesse(v_kmh){
  const mps=v_kmh/3.6;
  const mmps=mps*1000;
  const moyenne_kmh = vitesses.length? vitesses.reduce((a,b)=>a+b,0)/vitesses.length : 0;
  safeSetText('vitesse', `Vitesse instantanée : ${v_kmh.toFixed(2)} km/h`);
  safeSetText('vitesse-moy', `Vitesse moyenne : ${moyenne_kmh.toFixed(2)} km/h`);
  safeSetText('vitesse-max', `Vitesse max : ${vitesseMax.toFixed(2)} km/h`);
  safeSetText('vitesse-ms', `Vitesse : ${mps.toFixed(2)} m/s | ${mmps.toFixed(0)} mm/s`);
  safeSetText('pourcentage', `% Lumière : ${(mps/VITESSE_LUMIERE*100).toExponential(2)}% | % Son : ${(mps/VITESSE_SON*100).toFixed(2)}%`);
}

function afficherDistance(){
  const km=distanceTotale/1000;
  const m=distanceTotale;
  const secLumiere=m/VITESSE_LUMIERE;
  const al=secLumiere/ANNEE_LUMIERE_SECONDES;
  safeSetText('distance',`Distance : ${km.toFixed(3)} km | ${m.toFixed(0)} m | ${Math.round(m*1000)} mm`);
  safeSetText('distance-cosmique',`Distance cosmique : ${secLumiere.toFixed(3)} s lumière | ${al.toExponential(3)} al`);
}

function afficherTemps(){
  if(!tempsDebut) return;
  const t=(Date.now()-tempsDebut)/1000;
  safeSetText('temps',`Temps : ${t.toFixed(2)} s`);
}

function afficherGPS(g){
  safeSetText('gps',`Latitude : ${g.latitude.toFixed(6)} | Longitude : ${g.longitude.toFixed(6)} | Altitude : ${g.altitude?.toFixed(0)??'--'} m | Précision : ${g.accuracy?.toFixed(0)??'--'} m`);
}

function afficherCapteurs(){
  safeSetText('capteurs',`Lumière : ${capteursEtat.lumiere} lux | Son : ${capteursEtat.son} dB | Niveau : ${capteursEtat.niveauBulle}° | Magnétomètre : ${capteursEtat.magnetisme} µT`);
}

// ========================
// ACCÉLÉROMÈTRE
// ========================
function activerAccelerometre(){
  if('DeviceMotionEvent' in window){
    window.addEventListener('devicemotion', e=>{
      const a=e.accelerationIncludingGravity;
      if(!a) return;
      const now=Date.now();
      if(tempsDerniereMesureAccel){
        const dt=(now-tempsDerniereMesureAccel)/1000;
        const magnitude=Math.sqrt(a.x**2 + a.y**2 + a.z**2);
        vitesseAccel += magnitude*dt*3.6; // km/h
      }
      tempsDerniereMesureAccel = now;
    });
  }
}

// ========================
// UPDATE VITESSE COMBINÉE
// ========================
function miseAJourVitesse(pos){
  const gps = {
    latitude: pos.coords.latitude,
    longitude: pos.coords.longitude,
    altitude: pos.coords.altitude,
    accuracy: pos.coords.accuracy,
    timestamp: pos.timestamp,
    speed: pos.coords.speed
  };

  const vitesseGPS = (typeof gps.speed==='number' && gps.speed>=0)? gps.speed*3.6 : calculerVitesseManuelle(gps);
  const vitesseComb = vitesseGPS? (vitesseGPS+vitesseAccel)/2 : vitesseAccel;

  vitesseMax = Math.max(vitesseMax, vitesseComb);
  vitesses.push(vitesseComb);
  if(vitesses.length>60) vitesses.shift();

  afficherVitesse(vitesseComb);
  afficherDistance();
  afficherTemps();
  afficherGPS(gps);
}

// ========================
// COCKPIT START / STOP
// ========================
function demarrerCockpit(){
  if(!("geolocation" in navigator)){ safeSetText('gps','GPS non disponible'); return; }
  if(watchId!==null) return;

  tempsDebut = Date.now();
  afficherTemps();

  activerAccelerometre();

  watchId = navigator.geolocation.watchPosition(
    miseAJourVitesse,
    err=>{ safeSetText('gps','Erreur GPS: '+err.code); },
    { enableHighAccuracy:true, maximumAge:0, timeout:10000 }
  );

  document.getElementById('marche').textContent='⏹️ Arrêt';
}

function arreterCockpit(){
  if(watchId!==null) navigator.geolocation.clearWatch(watchId);
  watchId=null;
  positionPrecedente=null;
  vitesses=[];
  distanceTotale=0;
  tempsDebut=null;
  vitesseAccel=0;

  afficherVitesse(0);
  afficherDistance();
  safeSetText('temps','Temps : 0.00 s');
  afficherCapteurs();

  document.getElementById('marche').textContent='▶️ Marche';
}

function resetVitesseMax(){
  vitesseMax=0;
  safeSetText('vitesse-max','Vitesse max : 0.00 km/h');
}

// ========================
// INIT DOM
// ========================
document.addEventListener('DOMContentLoaded',()=>{
  afficherVitesse(0);
  afficherDistance();
  safeSetText('temps','Temps : 0.00 s');

  document.getElementById('marche').addEventListener('click',()=>{
    if(watchId===null) demarrerCockpit();
    else arreterCockpit();
  });
  document.getElementById('reset').addEventListener('click',resetVitesseMax);
});
          
