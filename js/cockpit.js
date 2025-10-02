import SunCalc from 'https://cdn.jsdelivr.net/npm/suncalc@1.9.0/suncalc.js';
import * as Medaillon from './js/medaillon.js';

let watchId=null,positionPrecedente=null,distanceTotale=0,vitesseMax=0,vitesses=[],startTime=null;
let destination={latitude:null,longitude:null};

// ======================
// DOM UTIL
// ======================
const safeSet=(id,text)=>{const el=document.getElementById(id); if(el) el.textContent=text;};

// ======================
// GPS & VITESSE
// ======================
function traiterPosition(coords,timestamp){
  if(!startTime) startTime=timestamp;
  const lat=coords.latitude, lon=coords.longitude, alt=coords.altitude??0, acc=coords.accuracy??0;
  const speed = coords.speed!==null? coords.speed*3.6 : calculerVitesse(coords,timestamp);

  if(positionPrecedente){
    const d=calculerDistance(coords,positionPrecedente);
    distanceTotale+=d;
  }
  positionPrecedente=coords;
  vitesses.push(speed);
  vitesseMax=Math.max(vitesseMax,speed);

  afficherVitesse(speed);
  afficherDistance();
  afficherPourcentage(speed);
  afficherGPS(coords);
  afficherMinecraft(coords);
  afficherBoussole(coords);
  afficherHeureSolaire(coords);
}

function calculerVitesse(coords,timestamp){
  if(!positionPrecedente) return 0;
  const dt=(timestamp-startTime)/1000;
  const d=calculerDistance(coords,positionPrecedente);
  return dt>0? (d/dt)*3.6 : 0;
}

function calculerDistance(a,b){
  const R=6371e3, φ1=a.latitude*Math.PI/180, φ2=b.latitude*Math.PI/180;
  const Δφ=(b.latitude-a.latitude)*Math.PI/180;
  const Δλ=(b.longitude-a.longitude)*Math.PI/180;
  const c=2*Math.atan2(Math.sqrt(Math.sin(Δφ/2)**2+Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2),
                        Math.sqrt(1-(Math.sin(Δφ/2)**2+Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2)));
  return R*c;
}

function afficherVitesse(v){
  const moyenne=vitesses.length? vitesses.reduce((a,b)=>a+b,0)/vitesses.length:0;
  const mps=v/3.6, mmps=mps*1000;
  safeSet('vitesse',`Temps : ${(performance.now()/1000).toFixed(2)} s | Vitesse instantanée : ${v.toFixed(2)} km/h | Moyenne : ${moyenne.toFixed(2)} km/h | Max : ${vitesseMax.toFixed(2)} km/h | ${mps.toFixed(2)} m/s | ${mmps.toFixed(0)} mm/s`);
}

function afficherDistance(){
  const km=distanceTotale/1000, m=distanceTotale, mm=m*1000;
  const secL=m/299792458, al=secL/(3600*24*365.25);
  safeSet('distance',`Distance : ${km.toFixed(3)} km | ${m.toFixed(0)} m | ${mm.toFixed(0)} mm | Cosmique : ${secL.toFixed(3)} s lumière | ${al.toExponential(3)} al`);
}

function afficherPourcentage(v){
  const mps=v/3.6, pctL=(mps/299792458)*100, pctS=(mps/343)*100;
  safeSet('pourcentage',`% Lumière : ${pctL.toExponential(2)}% | % Son : ${pctS.toFixed(2)}%`);
}

function afficherGPS(coords){
  safeSet('gps',`Latitude : ${coords.latitude.toFixed(6)} | Longitude : ${coords.longitude.toFixed(6)} | Altitude : ${(coords.altitude??0).toFixed(1)} m | Précision GPS : ${(coords.accuracy??0).toFixed(0)}%`);
}

function afficherMinecraft(coords){
  const x=Math.round(coords.longitude*1000), y=Math.round(coords.altitude??0), z=Math.round(coords.latitude*1000);
  safeSet('boussole',`Coordonnées Minecraft : X:${x} Y:${y} Z:${z}`);
}

function afficherBoussole(coords){
  // simplifié : cap vers destination
  if(destination.latitude && destination.longitude){
    const dx=destination.longitude-coords.longitude, dz=destination.latitude-coords.latitude;
    const angle=(Math.atan2(dz,dx)*180/Math.PI+360)%360;
    safeSet('boussole',document.getElementById('boussole').textContent+` | Cap vers destination : ${angle.toFixed(0)}°`);
  }
}

// ======================
// MÉDAILLON COSMIQUE
// ======================
function afficherMedaillonComplet(){
  const med=document.getElementById('medaillon');
  med.innerHTML='';
  const c=document.createElement('canvas'); c.width=400; c.height=400;
  med.appendChild(c);
  const ctx=c.getContext('2d');
  const cx=200,cy=200,r=180;

  ctx.fillStyle="#000"; ctx.fillRect(0,0,400,400);
  ctx.strokeStyle="#0ff"; ctx.lineWidth=2; ctx.beginPath(); ctx.arc(cx,cy,r,0,2*Math.PI); ctx.stroke();

  ctx.fillStyle="#ffd700"; ctx.beginPath(); ctx.arc(cx+80,cy-80,12,0,2*Math.PI); ctx.fill(); ctx.fillText("☀️ Soleil",cx+60,cy-100);
  ctx.fillStyle="#ccc"; ctx.beginPath(); ctx.arc(cx-80,cy-60,10,0,2*Math.PI); ctx.fill(); ctx.fillText("🌙 Lune",cx-110,cy-80);

  const stars=[{name:"Orion",x:cx+30,y:cy+80},{name:"Cassiopeia",x:cx-50,y:cy+60},{name:"Scorpius",x:cx+100,y:cy+30}];
  ctx.fillStyle="#0ff"; stars.forEach(s=>{ctx.beginPath(); ctx.arc(s.x,s.y,3,0,2*Math.PI); ctx.fill(); ctx.fillText(s.name,s.x+5,s.y+5);});
  ctx.fillStyle="#f0f"; ctx.beginPath(); ctx.arc(cx,cy+140,4,0,2*Math.PI); ctx.fill(); ctx.fillText("🌌 Galaxie",cx-30,cy+160);
}

// ======================
// HORLOGE & HEURE SOLAIRE
// ======================
function activerHorloge(){
  const el=document.getElementById('horloge');
  function loop(){ el.textContent=new Date().toLocaleTimeString(); requestAnimationFrame(loop);}
  loop();
}

function afficherHeureSolaire(coords){
  const now=new Date();
  const sun=SunCalc.getTimes(now,coords.latitude,coords.longitude);
  const eqTime=(sun.solarNoon-now)/60000; // approximation
  const solaireVrai=now.getHours()+now.getMinutes()/60+now.getSeconds()/3600+eqTime/60;
  const solaireMoy=now.getHours()+now.getMinutes()/60+now.getSeconds()/3600;
  safeSet('heureSolaire',`Heure solaire vraie : ${solaireVrai.toFixed(2)} h | Heure solaire moyenne : ${solaireMoy.toFixed(2)} h | Equation du temps : ${eqTime.toFixed(2)} min`);
}

// ======================
// CAPTEURS
// ======================
function activerCapteurs(){
  const cap=document.getElementById('capteurs'); cap.textContent='';
  if('AmbientLightSensor' in window){ try{ const light=new AmbientLightSensor(); light.addEventListener('reading',()=>cap.textContent+=` | Lux : ${light.illuminance.toFixed(0)}`); light.start(); }catch{} }
  if(navigator.mediaDevices?.getUserMedia){ navigator.mediaDevices.getUserMedia({audio:true}).then(stream=>{ const ctx=new AudioContext(); const analyser=ctx.createAnalyser(); const src=ctx.createMediaStreamSource(stream); src.connect(analyser); const data=new Uint8Array(analyser.frequencyBinCount); (function loop(){ analyser.getByteFrequencyData(data); const dB=20*Math.log10(data.reduce((a,b)=>a+b,0)/data.length||1); cap.textContent+=` | dB : ${dB.toFixed(0)} | Hz : ${analyser.frequencyBinCount}`; requestAnimationFrame(loop); })(); }).catch(()=>{}); }
  if('DeviceOrientationEvent' in window){ window.addEventListener('deviceorientation',e=>{if(e.beta!=null) cap.textContent+=` | Niveau : ${e.beta.toFixed(1)}°`;}); }
  if('Magnetometer' in window){ try{ const m=new Magnetometer(); m.addEventListener('reading',()=>cap.textContent+=` | Magnétisme : ${Math.sqrt(m.x**2+m.y**2+m.z**2).toFixed(0)} µT`); m.start(); }catch{} }
}

// ======================
// INITIALISATION
// ======================
document.getElementById('marche').addEventListener('click',()=>{
  if(!watchId){
    navigator.geolocation.getCurrentPosition(pos=>traiterPosition(pos.coords,pos.timestamp),err=>console.error(err));
    watchId=navigator.geolocation.watchPosition(pos=>traiterPosition(pos.coords,pos.timestamp),err=>console.error(err),{enableHighAccuracy:true,maximumAge:0,timeout:10000});
    activerHorloge(); activerCapteurs(); afficherMedaillonComplet();
    document.getElementById('marche').textContent='⏹️ Arrêter';
  }else{ navigator.geolocation.clearWatch(watchId); watchId=null; document.getElementById('marche').textContent='▶️ Démarrer'; }
});

document.getElementById('reset').addEventListener('click',()=>{vitesseMax=0; vitesses=[]; distanceTotale=0; startTime=null;});

document.getElementById('defDest').addEventListener('click',()=>{
  destination.latitude=parseFloat(document.getElementById('destLat').value);
  destination.longitude=parseFloat(document.getElementById('destLon').value);
});
