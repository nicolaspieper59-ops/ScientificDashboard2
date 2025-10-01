let watchId = null;
let positionPrecedente = null;
let distanceTotale = 0;
let vitesseMax = 0;
let vitesses = [];
let destination = { latitude: null, longitude: null };
let startTime = null;

/* ======================
   UTILITAIRES DOM
====================== */
const safeSet = (id, val) => { const el=document.getElementById(id); if(el) el.textContent=val; };

/* ======================
   GPS & VITESSE
====================== */
export function demarrerCockpit() {
  if (!("geolocation" in navigator)) { safeSet('gps','GPS non disponible'); return; }
  if (watchId !== null) return;
  startTime = Date.now();

  watchId = navigator.geolocation.watchPosition(pos => {
    const gps = pos.coords;
    const dt = (Date.now()-startTime)/1000;
    const vitesse = gps.speed!=null ? gps.speed*3.6 : calculerVitesse(pos);

    if(vitesse>=0 && vitesse<300){
      vitesseMax=Math.max(vitesseMax,vitesse);
      vitesses.push(vitesse);
      afficherVitesse(vitesse);
      afficherDistance();
      afficherPourcentage(vitesse);
      afficherGPS(gps);
    }
  }, err=> safeSet('gps','Erreur GPS : '+(err.message||err.code)), {enableHighAccuracy:true,maximumAge:0,timeout:10000});

  activerCapteurs();
  activerBoussole();
  activerHorloge();
  afficherMedaillon();
  chargerMeteo();
  afficherGrandeurs();
}

export function arreterCockpit() {
  if(watchId!=null) navigator.geolocation.clearWatch(watchId);
  watchId=null;
  positionPrecedente=null;
  vitesses=[];
  distanceTotale=0;
  vitesseMax=0;
}

/* ======================
   CALCULS
====================== */
function calculerVitesse(pos) {
  if(!positionPrecedente){ positionPrecedente=pos.coords; return 0; }
  const dt = (pos.timestamp - positionPrecedente.timestamp)/1000;
  const d = calculerDistance(pos.coords,positionPrecedente);
  distanceTotale += d;
  positionPrecedente=pos.coords;
  return dt>0 ? (d/dt)*3.6 : 0;
}

function calculerDistance(a,b){
  const R=6371e3;
  const φ1=a.latitude*Math.PI/180;
  const φ2=b.latitude*Math.PI/180;
  const Δφ=(b.latitude-a.latitude)*Math.PI/180;
  const Δλ=(b.longitude-a.longitude)*Math.PI/180;
  const c=2*Math.atan2(Math.sqrt(Math.sin(Δφ/2)**2+Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2), Math.sqrt(1-(Math.sin(Δφ/2)**2+Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2)));
  return R*c;
}

/* ======================
   AFFICHAGE
====================== */
function afficherVitesse(v){
  const moyenne=vitesses.length? vitesses.reduce((a,b)=>a+b,0)/vitesses.length :0;
  safeSet('vitesse', v.toFixed(2)+' km/h');
  safeSet('moyenne', moyenne.toFixed(2)+' km/h');
  safeSet('vmax', vitesseMax.toFixed(2)+' km/h');
}

function afficherDistance(){
  safeSet('distance',(distanceTotale/1000).toFixed(3)+' km');
  const distCosmique = distanceTotale/299792458;
  safeSet('distcosmique', distCosmique.toExponential(3)+' al');
}

function afficherPourcentage(v){
  const mps = v/3.6;
  safeSet('plumiere',((mps/299792458)*100).toExponential(2));
  safeSet('pson',((mps/343)*100).toFixed(2));
}

function afficherGPS(g){
  safeSet('boussole', `Lat:${g.latitude.toFixed(6)} Lon:${g.longitude.toFixed(6)}`);
}

/* ======================
   CAPTEURS
====================== */
function activerCapteurs(){
  const cap=document.getElementById('capteurs');
  cap.dataset.base='Niveau à bulle : --° | Lumière : -- lux | Son : -- dB | Fréquence : -- Hz | Magnétisme : -- µT';
  cap.textContent=cap.dataset.base;

  // Lumière
  if('AmbientLightSensor' in window){
    try{
      const light = new AmbientLightSensor();
      light.addEventListener('reading',()=>cap.textContent=`${cap.dataset.base} | Lumière : ${Math.round(light.illuminance)} lux`);
      light.start();
    }catch{}
  }

  // Son
  if(navigator.mediaDevices && navigator.mediaDevices.getUserMedia){
    navigator.mediaDevices.getUserMedia({audio:true}).then(stream=>{
      const audioCtx = new AudioContext();
      const analyser = audioCtx.createAnalyser();
      const source = audioCtx.createMediaStreamSource(stream);
      source.connect(analyser);
      analyser.fftSize = 2048;
      const data = new Uint8Array(analyser.frequencyBinCount);
      function loop(){
        analyser.getByteFrequencyData(data);
        const dB = 20*Math.log10(data.reduce((a,b)=>a+b,0)/data.length||1);
        cap.textContent=`${cap.dataset.base} | Son : ${dB.toFixed(0)} dB | Fréquence : ${analyser.frequencyBinCount} Hz`;
        requestAnimationFrame(loop);
      }
      loop();
    }).catch(()=>{});
  }

  // Orientation
  if('DeviceOrientationEvent' in window){
    window.addEventListener('deviceorientation', e=>{
      if(e.beta!=null) cap.textContent=`${cap.dataset.base} | Niveau : ${e.beta.toFixed(1)}°`;
    });
  }

  // Magnétisme
  if('Magnetometer' in window){
    try{
      const mag=new Magnetometer();
      mag.addEventListener('reading',()=>cap.textContent=`${cap.dataset.base} | Magnétisme : ${Math.round(Math.sqrt(mag.x**2+mag.y**2+mag.z**2))} µT`);
      mag.start();
    }catch{}
  }
}

/* ======================
   Boussole
====================== */
function activerBoussole(){
  if('DeviceOrientationEvent' in window){
    window.addEventListener('deviceorientationabsolute', e=>{
      if(e.alpha!=null) safeSet('boussole', 'Cap : '+e.alpha.toFixed(0)+'°');
    });
  }
}

/* ======================
   Horloge
====================== */
function activerHorloge(){
  const h=document.getElementById('horloge');
  (function loop(){
    const d=new Date();
    h.textContent=`${String(d.getHours()).padStart(2,'0')}:${String(d.getMinutes()).padStart(2,'0')}:${String(d.getSeconds()).padStart(2,'0')}`;
    requestAnimationFrame(loop);
  })();
}

/* ======================
   Médaillon
====================== */
function afficherMedaillon(){
  const c=document.getElementById('medaillon');
  c.width=400; c.height=400;
  const ctx=c.getContext('2d');
  ctx.fillStyle='#000'; ctx.fillRect(0,0,c.width,c.height);
  ctx.strokeStyle='#0ff'; ctx.lineWidth=2;
  ctx.beginPath(); ctx.arc(c.width/2,c.height/2,180,0,2*Math.PI); ctx.stroke();
  ctx.fillStyle='#ffd700';
  ctx.beginPath(); ctx.arc(c.width/2+100,c.height/2-100,12,0,2*Math.PI); ctx.fill();
  ctx.fillStyle='#ccc';
  ctx.beginPath(); ctx.arc(c.width/2-120,c.height/2-80,10,0,2*Math.PI); ctx.fill();
}

/* ======================
   Météo
====================== */
export async function chargerMeteo(lat=43.3,lon=5.4){
  try{
    const url=`https://api.open-meteo.com/v1/forecast?latitude=${lat}&longitude=${lon}&current_weather=true`;
    const res=await fetch(url); const data=await res.json();
    const w=data.current_weather;
    safeSet('temp',w.temperature); safeSet('vent',w.windspeed); safeSet('pression',w.pressure);
    safeSet('humid','--'); safeSet('nuages','--'); safeSet('uv','--');
  }catch(e){console.warn('Météo offline',e);}
}

/* ======================
   Grandeurs physiques
====================== */
function afficherGrandeurs(){
  safeSet('grandeurs','Point ébullition : 100°C | Gravité : 9.81 m/s² | Vitesse son : 343 m/s | Vitesse lumière : 299792458 m/s');
}

/* ======================
   EXPORTS
====================== */
export { demarrerCockpit, arreterCockpit };
    
