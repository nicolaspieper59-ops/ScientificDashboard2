// ========================
// État global
// ========================
let watchId = null;
let positionPrecedente = null;
let vitesseMax = 0;
let vitesses = [];
let distanceTotale = 0;
let tempsDebut = null;
let modeSouterrainActif = false;
let targetCoords = { latitude: 48.8584, longitude: 2.2945 };
let modulesActive = { rituel:true, physique:true, chimie:true, svt:true };

// Constantes physiques
const VITESSE_LUMIERE = 299792458;
const VITESSE_SON = 343;
const R_TERRE = 6371e3;
const ANNEE_LUMIERE_SECONDES = 3600*24*365.25;
const MASSE_EXPLORATEUR = 70;

// ========================
// Utilitaires
// ========================
function safeSetText(id,text){ const e=document.getElementById(id); if(e) e.textContent=text; }
function toRadians(d){return d*Math.PI/180;}
function toDegrees(r){return r*180/Math.PI;}
function calculerDistanceHaversine(p1,p2){
  const φ1=toRadians(p1.latitude), φ2=toRadians(p2.latitude);
  const Δφ=toRadians(p2.latitude-p1.latitude), Δλ=toRadians(p2.longitude-p1.longitude);
  const a=Math.sin(Δφ/2)**2+Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2;
  return R_TERRE*2*Math.atan2(Math.sqrt(a),Math.sqrt(1-a));
}

// ========================
// GPS & vitesse
// ========================
function miseAJourVitesse(pos){
  if(modeSouterrainActif) return;
  const gps={latitude:pos.coords.latitude,longitude:pos.coords.longitude,altitude:pos.coords.altitude||0,accuracy:pos.coords.accuracy,speed:pos.coords.speed||0,heading:pos.coords.heading};
  let vitesse_ms=0;
  if(positionPrecedente){
    const dt=(pos.timestamp-positionPrecedente.timestamp)/1000;
    const dist=calculerDistanceHaversine(gps,positionPrecedente);
    distanceTotale+=dist;
    vitesse_ms=(gps.speed>0)?gps.speed:dist/dt;
  }
  positionPrecedente=gps;
  const vitesse_kmh=vitesse_ms*3.6;
  vitesseMax=Math.max(vitesseMax,vitesse_kmh);
  vitesses.push(vitesse_kmh); if(vitesses.length>30) vitesses.shift();
  afficherVitesse(vitesse_kmh,vitesse_ms);
  afficherDistance();
  afficherTemps();
  afficherGPS(gps);
}

function afficherVitesse(v_kmh,v_ms){
  const moyenne_kmh=vitesses.reduce((a,b)=>a+b,0)/vitesses.length||0;
  safeSetText('vitesse',`Vitesse instantanée : ${v_kmh.toFixed(2)} km/h`);
  safeSetText('vitesse-moy',`Vitesse moyenne : ${moyenne_kmh.toFixed(2)} km/h`);
  safeSetText('vitesse-max',`Vitesse max : ${vitesseMax.toFixed(2)} km/h`);
  safeSetText('vitesse-ms',`Vitesse : ${v_ms.toFixed(2)} m/s | ${(v_ms*1000).toFixed(0)} mm/s`);
  safeSetText('pourcentage',`% Lumière : ${(v_ms/VITESSE_LUMIERE*100).toExponential(2)}% | % Son : ${(v_ms/VITESSE_SON*100).toFixed(2)}%`);
}

function afficherDistance(){
  const km=distanceTotale/1000, m=distanceTotale, mm=m*1000;
  const secLumiere=m/VITESSE_LUMIERE, al=secLumiere/ANNEE_LUMIERE_SECONDES;
  safeSetText('distance',`Distance : ${km.toFixed(3)} km | ${m.toFixed(0)} m | ${mm.toFixed(0)} mm`);
  safeSetText('distance-cosmique',`Distance cosmique : ${secLumiere.toFixed(3)} s lumière | ${al.toExponential(3)} al`);
}

function afficherTemps(){
  if(!tempsDebut) return;
  const t=(Date.now()-tempsDebut)/1000;
  safeSetText('temps',`Temps : ${t.toFixed(2)} s`);
}

function afficherGPS(g){
  safeSetText('gps',`GPS : Lat : ${g.latitude.toFixed(6)} | Lon : ${g.longitude.toFixed(6)} | Alt : ${g.altitude.toFixed(0)} m | Préc. : ${g.accuracy.toFixed(0)} m`);
}

// ========================
// Médaillon céleste (simplifié)
// ========================
function calculerHeuresSolaires(now,lat,lon){
  // Simplifié: on peut reprendre ton code complet ici
  safeSetText('heure-vraie','--'); safeSetText('heure-moyenne','--'); safeSetText('equation-temps','--');
  safeSetText('soleil-lever','--'); safeSetText('soleil-coucher','--');
  safeSetText('lune-phase','--'); safeSetText('lune-lever','--'); safeSetText('lune-coucher','--'); safeSetText('lune-mag','--'); safeSetText('lune-culmination','--');
  drawMinecraftClock(12, 'Nouvelle Lune');
}

function drawMinecraftClock(hour,phase){const canvas=document.getElementById('minecraft-clock'); if(!canvas) return; const ctx=canvas.getContext('2d'); const size=100,center=size/2,radius=size*0.4; ctx.clearRect(0,0,size,size); ctx.strokeStyle='#00ffcc'; ctx.lineWidth=3; ctx.beginPath(); ctx.arc(center,center,radius,0,2*Math.PI); ctx.stroke(); const angle_rad=(hour/24*2*Math.PI)-Math.PI/2; const x=center+radius*Math.cos(angle_rad),y=center+radius*Math.sin(angle_rad); ctx.beginPath(); ctx.arc(x,y,8,0,2*Math.PI); ctx.fillStyle='#ffd700'; ctx.fill();}

// ========================
// Contrôles cockpit
// ========================
function demarrerCockpit(){
  if(watchId!==null) return;
  if(!tempsDebut) tempsDebut=Date.now();
  const options={enableHighAccuracy:true,timeout:5000,maximumAge:0};
  watchId=navigator.geolocation.watchPosition(miseAJourVitesse,(e)=>{console.error(e);safeSetText('gps',`GPS : Erreur (${e.code})`)},options);
  calculerHeuresSolaires(new Date(),positionPrecedente?.latitude||48.8566,positionPrecedente?.longitude||2.3522);
}

function arreterCockpit(){
  if(watchId!==null){navigator.geolocation.clearWatch(watchId); watchId=null;}
  safeSetText('vitesse','Vitesse instantanée : **ARRÊT**');
}

function resetCockpit(){
  arreterCockpit(); positionPrecedente=null; vitesseMax=0; vitesses=[]; distanceTotale=0; tempsDebut=null;
  safeSetText('temps','Temps : 0.00 s'); safeSetText('vitesse','-- km/h'); safeSetText('vitesse-moy','-- km/h'); safeSetText('vitesse-max','-- km/h');
  safeSetText('vitesse-ms','-- m/s | -- mm/s'); safeSetText('pourcentage','% Lumière : --% | % Son : --%'); safeSetText('distance','--'); safeSetText('distance-cosmique','--');
  safeSetText('gps','--');
}

// ========================
// Cible GPS
// ========================
function setTargetCoords(){
  const input=document.getElementById('target-coord').value.split(',').map(p=>parseFloat(p.trim()));
  if(input.length===2 && !isNaN(input[0]) && !isNaN(input[1])){
    targetCoords.latitude=input[0]; targetCoords.longitude=input[1];
    alert(`Nouvelle cible : Lat ${targetCoords.latitude}, Lon ${targetCoords.longitude}`);
  } else alert('Format invalide: Lat,Lon');
}

// ========================
// Boutons
// ========================
document.getElementById('marche').addEventListener('click',()=>{if(watchId===null) demarrerCockpit();else arreterCockpit();});
document.getElementById('reset').addEventListener('click',()=>resetCockpit());
document.getElementById('toggle-souterrain').addEventListener('click',()=>{
  modeSouterrainActif=!modeSouterrainActif;
  document.getElementById('toggle-souterrain').textContent=modeSouterrainActif?'✅ Mode Souterrain : ON':'🚫 Mode Souterrain : OFF';
  if(modeSouterrainActif) arreterCockpit(); else if(!watchId) demarrerCockpit();
});
