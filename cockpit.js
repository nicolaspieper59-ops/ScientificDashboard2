// cockpitCosmique.js
let watchId = null;
let positionPrecedente = null;
let vitesseMax = 0;
let vitesses = [];
let distanceTotale = 0;
let destination = { latitude: null, longitude: null };
const _baseTexts = {};

/* ========================
   UTILITAIRES DOM
======================== */
function safeSetText(id, text) {
  const el = document.getElementById(id);
  if (el) el.textContent = text;
}
function appendText(id, text) {
  const el = document.getElementById(id);
  if (el) el.textContent = (_baseTexts[id] || '') + text;
}
function setElementTextBase(id, base) {
  _baseTexts[id] = base;
  safeSetText(id, base);
}
function getElementText(id) {
  return _baseTexts[id] || '';
}

/* ========================
   GPS & VITESSE
======================== */
export function demarrerCockpit() {
  if (!("geolocation" in navigator)) { safeSetText('gps','GPS non disponible'); return; }
  if (watchId !== null) return;

  watchId = navigator.geolocation.watchPosition(pos => {
    const gps = pos.coords;
    const vitesse = (typeof gps.speed==='number' && gps.speed!==null) ? gps.speed*3.6 : calculerVitesse({latitude:gps.latitude, longitude:gps.longitude, altitude:gps.altitude, timestamp:pos.timestamp});
    if (vitesse>=0 && vitesse<300) {
      vitesseMax=Math.max(vitesseMax,vitesse);
      vitesses.push(vitesse);
      afficherVitesse(vitesse);
      afficherDistance();
      afficherPourcentage(vitesse);
      afficherGPS({latitude:gps.latitude, longitude:gps.longitude, altitude:gps.altitude, accuracy:gps.accuracy});
    }
  }, err=>{safeSetText('gps','Erreur GPS');},{enableHighAccuracy:true, maximumAge:0, timeout:10000});
}

export function arreterCockpit() {
  if (watchId!==null) navigator.geolocation.clearWatch(watchId);
  watchId=null; positionPrecedente=null; vitesses=[]; distanceTotale=0; vitesseMax=0;
}

export function resetVitesseMax() { vitesseMax=0; appendText('vitesse',' | Max réinitialisé'); }

function calculerVitesse(gps) {
  if (!positionPrecedente) { positionPrecedente=gps; return 0; }
  const dt=(gps.timestamp-positionPrecedente.timestamp)/1000;
  const d=calculerDistance(gps,positionPrecedente);
  distanceTotale+=d; positionPrecedente=gps;
  return dt>0?(d/dt)*3.6:0;
}
function calculerDistance(a,b){const R=6371e3; const φ1=a.latitude*Math.PI/180; const φ2=b.latitude*Math.PI/180; const Δφ=(b.latitude-a.latitude)*Math.PI/180; const Δλ=(b.longitude-a.longitude)*Math.PI/180; const c=2*Math.atan2(Math.sqrt(Math.sin(Δφ/2)**2+Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2), Math.sqrt(1-(Math.sin(Δφ/2)**2+Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2))); return R*c; }

function afficherVitesse(v) {
  const moyenne = vitesses.length ? vitesses.reduce((a,b)=>a+b,0)/vitesses.length :0;
  const mps=v/3.6, mmps=mps*1000;
  safeSetText('vitesse', `Temps : ${new Date().toLocaleTimeString()} | Vitesse instantanée : ${v.toFixed(2)} km/h | Moyenne : ${moyenne.toFixed(2)} km/h | Max : ${vitesseMax.toFixed(2)} km/h | ${mps.toFixed(2)} m/s | ${mmps.toFixed(0)} mm/s`);
}
function afficherDistance() {
  const km=distanceTotale/1000, m=distanceTotale, mm=m*1000;
  const secL=m/299792458, al=secL/(3600*24*365.25);
  safeSetText('distance', `Distance : ${km.toFixed(3)} km | ${m.toFixed(0)} m | ${mm.toFixed(0)} mm | Cosmique : ${secL.toFixed(3)} s lumière | ${al.toExponential(3)} al`);
}
function afficherPourcentage(v) {
  const mps=v/3.6;
  safeSetText('pourcentage', `% Lumière : ${(mps/299792458).toExponential(2)}% | % Son : ${(mps/343).toFixed(2)}%`);
}
function afficherGPS(gps) {
  safeSetText('gps',`Latitude : ${gps.latitude.toFixed(6)} | Longitude : ${gps.longitude.toFixed(6)} | Altitude : ${(gps.altitude??'--')} | Précision GPS : ${gps.accuracy??'--'}%`);
}

/* ========================
   CAPTEURS
======================== */
function activerCapteurs() {
  setElementTextBase('capteurs','Niveau à bulle : --° | Lumière : -- lux | Son : -- dB | Fréquence : -- Hz | Magnétisme : -- µT');

  // Light
  if ('AmbientLightSensor' in window){try{const s=new AmbientLightSensor(); s.onreading=()=>appendText('capteurs',` | Lumière : ${Math.round(s.illuminance)} lux`); s.start();}catch{}}
  else if('ondevicelight' in window) window.addEventListener('devicelight', e=>appendText('capteurs',` | Lumière : ${Math.round(e.value)} lux`));

  // Microphone
  if(navigator.mediaDevices&&navigator.mediaDevices.getUserMedia){
    navigator.mediaDevices.getUserMedia({audio:true}).then(stream=>{
      const ctx=new (window.AudioContext||window.webkitAudioContext)();
      const analyser=ctx.createAnalyser();
      analyser.fftSize=2048;
      const source=ctx.createMediaStreamSource(stream);
      source.connect(analyser);
      const data=new Uint8Array(analyser.frequencyBinCount);
      function loop(){
        analyser.getByteFrequencyData(data);
        const dB=20*Math.log10(data.reduce((a,b)=>a+b,0)/data.length||1);
        safeSetText('capteurs',getElementText('capteurs')+` | Son : ${Math.round(dB)} dB | Fréquence : ${analyser.frequencyBinCount} Hz`);
        requestAnimationFrame(loop);
      }
      loop();
    }).catch(()=>{});
  }

  // Niveau à bulle
  if('DeviceOrientationEvent' in window) window.addEventListener('deviceorientation', e=>{if(typeof e.beta==='number') appendText('capteurs',` | Niveau à bulle : ${e.beta.toFixed(1)}°`);});

  // Magnetometer
  if('Magnetometer' in window){try{const m=new Magnetometer(); m.onreading=()=>appendText('capteurs',` | Magnétisme : ${Math.round(Math.sqrt(m.x**2+m.y**2+m.z**2))} µT`); m.start();}catch{}}
}

/* ========================
   Boussole & Destination
======================== */
function activerBoussole() {
  if(!('DeviceOrientationEvent' in window)) return;
  const evt='deviceorientationabsolute' in window?'deviceorientationabsolute':'deviceorientation';
  window.addEventListener(evt,e=>{
    if(typeof e.alpha==='number'){
      const cap=e.alpha;
      const coordMC=positionPrecedente?`X:${Math.round(positionPrecedente.longitude*1000)} Z:${Math.round(positionPrecedente.latitude*1000)}`:'--';
      const capDest='--';
      safeSetText('boussole',`Cap : ${cap.toFixed(0)}° | Coordonnées Minecraft : ${coordMC} | Cap vers destination : ${capDest}`);
    }
  });
}
export function definirDestination(lat,lon){destination.latitude=lat; destination.longitude=lon;}

/* ========================
   Horloge & Médaillon
======================== */
function activerHorlogeMinecraft(){
  const h=document.getElementById('horloge');
  function loop(){const d=new Date(); h.textContent=`⏰ Horloge Minecraft\n${String(d.getHours()).padStart(2,'0')}:${String(d.getMinutes()).padStart(2,'0')}:${String(d.getSeconds()).padStart(2,'0')}`; requestAnimationFrame(loop);}
  loop();
}

function afficherMedaillon(){
  const med=document.getElementById('medaillon'); if(!med) return;
  med.innerHTML='';
  const c=document.createElement('canvas'); c.width=300; c.height=300; med.appendChild(c);
  const ctx=c.getContext('2d');
  const cx=c.width/2, cy=c.height/2, r=140;
  ctx.fillStyle='#000'; ctx.fillRect(0,0,c.width,c.height);
  ctx.strokeStyle='#444'; ctx.lineWidth=2; ctx.beginPath(); ctx.arc(cx,cy,r,0,2*Math.PI); ctx.stroke();
  ctx.fillStyle='#fff'; ctx.font='10px monospace'; ctx.fillText('Zénith',cx-20,cy-r+10); ctx.fillText('0°',cx-10,cy+4);
  ctx.fillStyle='#ffd700'; ctx.beginPath(); ctx.arc(cx+60,cy-60,8,0,2*Math.PI); ctx.fill(); ctx.fillText('☀️ Soleil',cx+50,cy-70);
  ctx.fillStyle='#ccc'; ctx.beginPath(); ctx.arc(cx-60,cy-40,6,0,2*Math.PI); ctx.fill(); ctx.fillText('🌙 Lune',cx-70,cy-50);
  const stars=[{name:'Orion',x:cx+20,y:cy+60},{name:'Cassiopeia',x:cx-30,y:cy+40},{name:'Scorpius',x:cx+70,y:cy+10}];
  ctx.fillStyle='#0ff'; stars.forEach(s=>{ctx.beginPath();ctx.arc(s.x,s.y,2,0,2*Math.PI);ctx.fill();ctx.fillText(s.name,s.x+5,s.y+5);});
  ctx.fillStyle='#f0f'; ctx.beginPath(); ctx.arc(cx,cy+100,3,0,2*Math.PI); ctx.fill(); ctx.fillText('🌌 Galaxie',cx-20,cy+110);
}

/* ========================
   Météo & Qualité Air
======================== */
function chargerMeteo(){
  if(!navigator.onLine){safeSetText('meteo','Pas de connexion'); safeSetText('qualite-air','--'); return;}
  fetch('https://api.open-meteo.com/v1/forecast?latitude=43.3&longitude=5.4&current_weather=true').then(r=>r.json()).then(d=>{const w=d.current_weather||{}; safeSetText('meteo',`Temp : ${w.temperature??'--'} °C | Vent : ${w.windspeed??'--'} km/h`);}).catch(()=>safeSetText('meteo','Météo indisponible'));
  fetch('https://api.openaq.org/v2/latest?coordinates=43.3,5.4').then(r=>r.json()).then(d=>{const m=d.results?.[0]?.measurements??[]; const pm25=m.find(a=>a.parameter==='pm25')?.value??'--'; const uv=m.find(a=>a.parameter==='uv')?.value??'--'; safeSetText('qualite-air',`Qualité air : PM2.5 ${pm25} µg/m³ | Indice UV : ${uv}`);}).catch(()=>safeSetText('qualite-air','Qualité air indisponible'));
}

/* ========================
   Grandeurs scientifiques
======================== */
function afficherGrandeurs(){
  safeSetText('grandeurs','Point d’ébullition : 100 °C | Gravité : 9.81 m/s² | Vitesse son : 343 m/s | Vitesse lumière : 2.99792458e8 m/s');
}

/* ========================
   INIT DOM
======================== */
document.addEventListener('DOMContentLoaded',()=>{
  const btnMarche=document.getElementById('marche');
  const btnReset=document.getElementById('reset');
  let actif=false;

  if(btnMarche){
    btnMarche.addEventListener('click',()=>{
      actif=!actif;
      if(actif){
        demarrerCockpit();
        activerCapteurs();
        activerBoussole();
        activerHorlogeMinecraft();
        afficherMedaillon();
        chargerMeteo();
        afficherGrandeurs();
        btnMarche.classList.add('active');
        btnMarche.textContent='⏹️ Arrêter';
      } else {
        arreterCockpit();
        btnMarche.classList.remove('active');
        btnMarche.textContent='▶️ Démarrer';
      }
    });
  }

  if(btnReset) btnReset.addEventListener('click', ()=>resetVitesseMax());
});
                            
