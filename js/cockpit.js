// cockpitCosmique.js
let watchId = null, positionPrecedente = null, vitesseMax = 0, vitesses = [], distanceTotale = 0;

// ========================
// UTILITAIRES DOM
// ========================
function safeSetText(id,text){const e=document.getElementById(id);if(e)e.textContent=text;}
function appendText(id,text){const e=document.getElementById(id);if(e)e.textContent=(e.textContent.length>1000)?text:e.textContent+text;}

// ========================
// GPS / VITESSE
// ========================
function demarrerCockpit(){
  if(!("geolocation" in navigator)){safeSetText('gps','GPS non disponible');return;}
  if(watchId!==null)return;
  watchId=navigator.geolocation.watchPosition(pos=>{
    const gps={latitude:pos.coords.latitude,longitude:pos.coords.longitude,altitude:pos.coords.altitude,accuracy:pos.coords.accuracy,timestamp:pos.timestamp,speed:pos.coords.speed};
    const vitesse=(typeof gps.speed==='number' && gps.speed!=null)?gps.speed*3.6:calculerVitesse(gps);
    if(vitesse>=0 && vitesse<300){
      vitesseMax=Math.max(vitesseMax,vitesse);vitesses.push(vitesse);
      afficherVitesse(vitesse);afficherDistance();afficherPourcentage(vitesse);afficherGPS(gps);
    }
  },err=>{console.warn('GPS refusé ou erreur',err);safeSetText('gps','Erreur GPS');},{enableHighAccuracy:true,maximumAge:0,timeout:10000});
}
function arreterCockpit(){if(watchId!==null)navigator.geolocation.clearWatch(watchId);watchId=null;positionPrecedente=null;vitesses=[];distanceTotale=0;vitesseMax=0;}
function resetVitesseMax(){vitesseMax=0;}

// ========================
// CALCULS
// ========================
function calculerVitesse(gps){if(!positionPrecedente){positionPrecedente=gps;return 0;}const dt=(gps.timestamp-positionPrecedente.timestamp)/1000;const d=calculerDistance(gps,positionPrecedente);if(!Number.isFinite(d)||d<0){positionPrecedente=gps;return 0;}distanceTotale+=d;positionPrecedente=gps;return dt>0?(d/dt)*3.6:0;}
function calculerDistance(p1,p2){const R=6371e3,φ1=p1.latitude*Math.PI/180,φ2=p2.latitude*Math.PI/180,Δφ=(p2.latitude-p1.latitude)*Math.PI/180,Δλ=(p2.longitude-p1.longitude)*Math.PI/180,a=Math.sin(Δφ/2)**2+Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2,c=2*Math.atan2(Math.sqrt(a),Math.sqrt(1-a));return R*c;}

// ========================
// AFFICHAGE
// ========================
function afficherVitesse(v){const m=vitesses.length?vitesses.reduce((a,b)=>a+b,0)/vitesses.length:0;const mps=v/3.6,mmps=mps*1000;safeSetText('vitesse',`Temps : ${new Date().toLocaleTimeString()} | Vitesse : ${v.toFixed(2)} km/h | Moyenne : ${m.toFixed(2)} km/h | Max : ${vitesseMax.toFixed(2)} km/h | ${mps.toFixed(2)} m/s | ${mmps.toFixed(0)} mm/s`);}
function afficherDistance(){const km=distanceTotale/1000,m=distanceTotale,mm=m*1000,secLumiere=m/299792458,al=secLumiere/(3600*24*365.25);safeSetText('distance',`Distance : ${km.toFixed(3)} km | ${m.toFixed(0)} m | ${mm.toFixed(0)} mm | Cosmique : ${secLumiere.toFixed(3)} s lumière | ${al.toExponential(3)} al`);}
function afficherPourcentage(v){const mps=v/3.6;safeSetText('pourcentage',`% Lumière : ${(mps/299792458*100).toExponential(2)}% | % Son : ${(mps/343*100).toFixed(2)}%`);}
function afficherGPS(g){safeSetText('gps',`Latitude : ${g.latitude.toFixed(6)} | Longitude : ${g.longitude.toFixed(6)} | Altitude : ${g.altitude??'--'} | Précision GPS : ${g.accuracy??'--'}%`);}

// ========================
// CAPTEURS PHYSIQUES
// ========================
async function activerCapteurs(){
  setElementTextBase('capteurs','Niveau à bulle : --° | Lumière : -- lux');
  // Lumière
  if('AmbientLightSensor' in window){try{const s=new AmbientLightSensor();s.addEventListener('reading',()=>appendText('capteurs',` | Lumière : ${Math.round(s.illuminance)} lux`));s.start();}catch(e){console.warn(e);}}
  // Micro
  if(navigator.mediaDevices?.getUserMedia){try{const stream=await navigator.mediaDevices.getUserMedia({audio:true});const ctx=new (window.AudioContext||window.webkitAudioContext)();const analyser=ctx.createAnalyser();const source=ctx.createMediaStreamSource(stream);source.connect(analyser);analyser.fftSize=2048;const data=new Uint8Array(analyser.frequencyBinCount);function loop(){analyser.getByteFrequencyData(data);const dB=20*Math.log10(data.reduce((a,b)=>a+b,0)/data.length||1);safeSetText('capteurs',getElementText('capteurs')+` | Son : ${Math.round(dB)} dB`);requestAnimationFrame(loop);}loop();}catch(e){console.warn(e);}}
  // Orientation
  if(typeof DeviceOrientationEvent!=='undefined'){if(typeof DeviceOrientationEvent.requestPermission==='function'){try{const p=await DeviceOrientationEvent.requestPermission();if(p==='granted')window.addEventListener('deviceorientation',e=>appendText('capteurs',` | Niveau à bulle : ${e.beta?.toFixed(1)}°`));}catch(e){console.warn(e);}}else{window.addEventListener('deviceorientation',e=>appendText('capteurs',` | Niveau à bulle : ${e.beta?.toFixed(1)}°`));}}
  // Magnétomètre
  if('Magnetometer' in window){try{const m=new Magnetometer();m.addEventListener('reading',()=>appendText('capteurs',` | Magnétisme : ${Math.round(Math.sqrt(m.x**2+m.y**2+m.z**2))} µT`));m.start();}catch(e){console.warn(e);}}
}

// ========================
// HORLOGE & MEDAILLON
// ========================
function activerHorloge(){const h=document.getElementById('horloge');function loop(){const t=new Date();h.textContent=`⏰ ${t.getHours().toString().padStart(2,'0')}:${t.getMinutes().toString().padStart(2,'0')}:${t.getSeconds().toString().padStart(2,'0')}`;requestAnimationFrame(loop);}loop();}
function afficherMedaillon(){const m=document.getElementById('medaillon');if(!m)return;m.innerHTML='';const c=document.createElement('canvas');c.width=300;c.height=300;m.appendChild(c);const ctx=c.getContext('2d');ctx.fillStyle='#000';ctx.fillRect(0,0,c.width,c.height);ctx.fillStyle='#ffd700';ctx.beginPath();ctx.arc(150,100,10,0,2*Math.PI);ctx.fill();ctx.fillStyle='#ccc';ctx.beginPath();ctx.arc(150,200,8,0,2*Math.PI);ctx.fill();}

// ========================
// UTILS BASE TEXT
// ========================
const _baseTexts={};
function setElementTextBase(id,base){_baseTexts[id]=base;const e=document.getElementById(id);if(e)e.textContent=base;}
function getElementText(id){const e=document.getElementById(id);return e?e.textContent:_baseTexts[id]||'';}

// ========================
// INIT CLICK
// ========================
document.addEventListener('DOMContentLoaded',()=>{
  const btn=document.getElementById('marche');
  const reset=document.getElementById('reset');
  btn.addEventListener('click',async()=>{
    demarrerCockpit();
    await activerCapteurs();
    activerHorloge();
    afficherMedaillon();
    btn.textContent='⏹️ Arrêter';
  });
  reset.addEventListener('click',()=>{resetVitesseMax();appendText('vitesse',' | Max réinitialisé');});
});
      
