// ============================
// cockpit.js - GPS et capteurs
// ============================

import * as SunCalc from 'https://cdn.jsdelivr.net/npm/suncalc@1.9.0/suncalc.js';

let watchId = null;
let positionPrecedente = null;
let vitesses = [];
let vitesseMax = 0;
let distanceTotale = 0;
let destination = { latitude: null, longitude: null };
let masseSimulee = 70;

// ============================
// DOM Utils
// ============================
function safeSetText(id, text) { const el = document.getElementById(id); if(el) el.textContent=text; }
function appendText(id,text){ const el=document.getElementById(id); if(el) el.textContent+=text; }

// ============================
// GPS réel & suivi
// ============================
export function demarrerCockpit(){
  if(!("geolocation" in navigator)){ safeSetText('gps','GPS non disponible'); return; }
  if(watchId!==null) return;

  // Position actuelle
  navigator.geolocation.getCurrentPosition(
    pos => traiterPosition(pos.coords,pos.timestamp),
    err => safeSetText('gps','Erreur GPS: '+err.message)
  );

  // Suivi en continu
  watchId = navigator.geolocation.watchPosition(
    pos => traiterPosition(pos.coords,pos.timestamp),
    err => safeSetText('gps','Erreur GPS: '+err.message),
    { enableHighAccuracy:true, maximumAge:0, timeout:10000 }
  );

  activerCapteurs();
  activerBoussole();
  activerHorloge();
  afficherMedaillon();
  chargerMeteo();
  afficherGrandeurs();
}

// ============================
// Arrêt GPS et reset
// ============================
export function arreterCockpit(){
  if(watchId!==null) navigator.geolocation.clearWatch(watchId);
  watchId=null; positionPrecedente=null; vitesses=[]; distanceTotale=0; vitesseMax=0;
}

// ============================
// Réinitialiser vitesse max
// ============================
export function resetVitesseMax(){ vitesseMax=0; }

// ============================
// Définir destination
// ============================
export function definirDestination(lat,lon){ destination.latitude=lat; destination.longitude=lon; }
export function setMasseSimulee(masse){ masseSimulee=masse; afficherGrandeurs(); }

// ============================
// Traitement position
// ============================
function traiterPosition(coords,timestamp){
  const vitesse = coords.speed!=null ? coords.speed*3.6 : calculerVitesse(coords,timestamp);
  if(vitesse>=0 && vitesse<300){ 
    vitesseMax=Math.max(vitesseMax,vitesse); vitesses.push(vitesse); 
    afficherVitesse(vitesse); afficherDistance(); afficherPourcentage(vitesse); afficherGPS(coords);
  }
}

// ============================
// Calculs distance et vitesse
// ============================
function calculerVitesse(coords,timestamp){
  if(!positionPrecedente){ positionPrecedente=coords; return 0; }
  const dt=(timestamp-positionPrecedente.timestamp)/1000;
  const d=calculerDistance(coords,positionPrecedente);
  distanceTotale+=d; positionPrecedente=coords;
  return dt>0?(d/dt)*3.6:0;
}

function calculerDistance(a,b){
  const R=6371e3;
  const φ1=a.latitude*Math.PI/180, φ2=b.latitude*Math.PI/180;
  const Δφ=(b.latitude-a.latitude)*Math.PI/180;
  const Δλ=(b.longitude-a.longitude)*Math.PI/180;
  const c=2*Math.atan2(Math.sqrt(Math.sin(Δφ/2)**2+Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2),Math.sqrt(1-(Math.sin(Δφ/2)**2+Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2)));
  return R*c;
}

// ============================
// Affichage Vitesse / Distance / GPS
// ============================
function afficherVitesse(v){
  const moyenne = vitesses.length ? vitesses.reduce((a,b)=>a+b,0)/vitesses.length:0;
  safeSetText('vitesse',`Vitesse : ${v.toFixed(2)} km/h | Moyenne : ${moyenne.toFixed(2)} | Max : ${vitesseMax.toFixed(2)}`);
}

function afficherDistance(){
  safeSetText('distance',`Distance : ${(distanceTotale/1000).toFixed(3)} km`);
}

function afficherPourcentage(v){
  const mps=v/3.6;
  const pctL=(mps/299792458)*100;
  const pctS=(mps/343)*100;
  safeSetText('pourcentage',`% Lumière : ${pctL.toExponential(2)}% | % Son : ${pctS.toFixed(2)}%`);
}

function afficherGPS(g){
  safeSetText('gps',`Latitude : ${g.latitude.toFixed(6)} | Longitude : ${g.longitude.toFixed(6)} | Altitude : ${(g.altitude??0).toFixed(1)} m | Précision : ${(g.accuracy??0).toFixed(0)}%`);
}

// ============================
// Capteurs physiques
// ============================
function activerCapteurs(){
  const cap = document.getElementById('capteurs');
  cap.textContent='--';
  
  if('AmbientLightSensor' in window){
    try{
      const light=new AmbientLightSensor();
      light.addEventListener('reading',()=>cap.textContent=`Lumière : ${Math.round(light.illuminance)} lux`);
      light.start();
    }catch{}
  }

  if(navigator.mediaDevices?.getUserMedia){
    navigator.mediaDevices.getUserMedia({audio:true}).then(stream=>{
      const ctx=new AudioContext();
      const analyser=ctx.createAnalyser();
      const source=ctx.createMediaStreamSource(stream);
      source.connect(analyser);
      const data=new Uint8Array(analyser.frequencyBinCount);
      (function loop(){
        analyser.getByteFrequencyData(data);
        const dB=20*Math.log10(data.reduce((a,b)=>a+b,0)/data.length||1);
        cap.textContent+=` | Son : ${Math.round(dB)} dB | Hz : ${analyser.frequencyBinCount}`;
        requestAnimationFrame(loop);
      })();
    }).catch(()=>{});
  }

  if('DeviceOrientationEvent' in window){
    window.addEventListener('deviceorientation', e=>{
      if(e.beta!=null) cap.textContent+=` | Niveau : ${e.beta.toFixed(1)}°`;
    });
  }

  if('Magnetometer' in window){
    try{
      const mag=new Magnetometer();
      mag.addEventListener('reading',()=>cap.textContent+=` | Magnétisme : ${Math.sqrt(mag.x**2+mag.y**2+mag.z**2).toFixed(0)} µT`);
      mag.start();
    }catch{}
  }
}

// ============================
// Boussole et destination
// ============================
function activerBoussole(){
  if('DeviceOrientationEvent' in window){
    window.addEventListener('deviceorientationabsolute', e=>{
      if(e.alpha!=null){
        const cap = e.alpha.toFixed(0);
        const minecraft = positionPrecedente?`X:${Math.round(positionPrecedente.longitude*1000)} Z:${Math.round(positionPrecedente.latitude*1000)}`:'--';
        const capDest = destination.latitude?`${cap}°`:'--';
        safeSetText('boussole',`Cap : ${cap}° | Coordonnées Minecraft : ${minecraft} | Cap vers destination : ${capDest}`);
      }
    });
  }
}

// ============================
// Horloge
// ============================
function activerHorloge(){
  const h=document.getElementById('horloge');
  (function loop(){
    const d=new Date();
    h.textContent=`${String(d.getHours()).padStart(2,'0')}:${String(d.getMinutes()).padStart(2,'0')}:${String(d.getSeconds()).padStart(2,'0')}`;
    requestAnimationFrame(loop);
  })();
}

// ============================
// Médaillon cosmique
// ============================
function afficherMedaillon(){
  const med=document.getElementById('medaillon'); med.innerHTML='';
  const c=document.createElement('canvas'); c.width=500; c.height=500; med.appendChild(c);
  const ctx=c.getContext('2d');
  ctx.fillStyle='#000'; ctx.fillRect(0,0,500,500);

  // Cercle extérieur
  ctx.strokeStyle='#0ff'; ctx.lineWidth=2; ctx.beginPath(); ctx.arc(250,250,240,0,2*Math.PI); ctx.stroke();

  // Soleil
  ctx.fillStyle='#ffd700'; ctx.beginPath(); ctx.arc(370,130,12,0,2*Math.PI); ctx.fill(); ctx.fillText("☀️ Soleil",355,120);
  // Lune
  ctx.fillStyle='#ccc'; ctx.beginPath(); ctx.arc(130,170,10,0,2*Math.PI); ctx.fill(); ctx.fillText("🌙 Lune",115,160);
  // Constellations
  const stars=[{name:"Orion",x:270,y:390},{name:"Cassiopeia",x:220,y:350},{name:"Scorpius",x:320,y:330}];
  ctx.fillStyle='#0ff'; stars.forEach(s=>{ctx.beginPath();ctx.arc(s.x,s.y,3,0,2*Math.PI);ctx.fill(); ctx.fillText(s.name,s.x+5,s.y+5);});
  ctx.fillStyle='#f0f'; ctx.beginPath(); ctx.arc(250,450,4,0,2*Math.PI); ctx.fill(); ctx.fillText("🌌 Galaxie",230,460);
}

// ============================
// Météo
// ============================
function chargerMeteo(){
  const meteoEl=document.getElementById('meteo');
  const airEl=document.getElementById('qualite-air');
  if(!navigator.onLine){ meteoEl.textContent="Pas de connexion"; airEl.textContent="Indisponible"; return; }

  fetch("https://api.open-meteo.com/v1/forecast?latitude=43.3&longitude=5.4&current_weather=true")
    .then(r=>r.json()).then(data=>{
      const w=data.current_weather;
      meteoEl.textContent=`Temp : ${w.temperature}°C | Vent : ${w.windspeed} km/h`;
    }).catch(()=>{ meteoEl.textContent="Météo indisponible"; });

  fetch("https://api.openaq.org/v2/latest?coordinates=43.3,5.4")
    .then(r=>r.json()).then(d=>{
      const m=d.results[0]?.measurements??[];
      const pm25=m.find(a=>a.parameter==="pm25")?.value??"--";
      const uv=m.find(a=>a.parameter==="uv")?.value??"--";
      airEl.textContent=`Qualité air : PM2.5 ${pm25} µg/m³ | Indice UV : ${uv}`;
    }).catch(()=>{ airEl.textContent="Indisponible"; });
}

// ============================
// Grandeurs scientifiques
// ============================
function afficherGrandeurs(){
  const gravite = 9.81; // m/s²
  const masse = masseSimulee;
  const force = masse*gravite; // N
  const vitesse = vitesses.length ? vitesses[vitesses.length-1]/3.6 : 0; // m/s
  const cinetique = 0.5*masse*vitesse**2; // J
  const puissance = force*vitesse; // J/s
  safeSetText('grandeurs',`Point ébullition : 100°C | Gravité : ${gravite} m/s² | Force : ${force.toFixed(1)} N | Puissance : ${puissance.toFixed(1)} J | Cinétique : ${cinetique.toFixed(1)} J`);
}
