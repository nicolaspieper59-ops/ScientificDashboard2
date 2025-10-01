let watchId = null;
let positionPrecedente = null;
let distanceTotale = 0;
let vitesseMax = 0;
let vitesses = [];
let destination = { latitude: null, longitude: null };

const _baseTexts = {};

function safeSetText(id, text) {
  const el = document.getElementById(id);
  if (el) el.textContent = text;
}

function appendText(id, text) {
  const el = document.getElementById(id);
  if (el) el.textContent = _baseTexts[id] ? _baseTexts[id] + text : text;
}

function setElementTextBase(id, base) {
  _baseTexts[id] = base;
  safeSetText(id, base);
}

/* ==========================
   Cockpit GPS / vitesse
========================== */
export function demarrerCockpit() {
  if (!("geolocation" in navigator)) {
    safeSetText('gps','GPS non disponible');
    return;
  }
  if (watchId !== null) return;

  watchId = navigator.geolocation.watchPosition(
    pos => {
      const gps = pos.coords;
      const vitesse = gps.speed != null ? gps.speed * 3.6 : calculerVitesse(pos);
      if (vitesse >= 0 && vitesse < 300) {
        vitesseMax = Math.max(vitesseMax, vitesse);
        vitesses.push(vitesse);
        afficherVitesse(vitesse);
        afficherDistance();
        afficherPourcentage(vitesse);
        afficherGPS(gps);
      }
    },
    err => safeSetText('gps', 'Erreur GPS : ' + (err.message||err.code)),
    { enableHighAccuracy:true, maximumAge:0, timeout:10000 }
  );

  activerCapteurs();
  activerBoussole();
  activerHorloge();
  afficherMedaillon();
  chargerMeteo();
  afficherGrandeurs();
}

export function arreterCockpit() {
  if (watchId != null) navigator.geolocation.clearWatch(watchId);
  watchId = null;
  positionPrecedente = null;
  vitesses = [];
  distanceTotale = 0;
  vitesseMax = 0;
}

export function resetVitesseMax() { vitesseMax=0; }

export function definirDestination(lat, lon) { destination.latitude=lat; destination.longitude=lon; }

/* ==========================
   Calculs GPS / distance
========================== */
function calculerVitesse(pos) {
  if (!positionPrecedente) { positionPrecedente = pos.coords; return 0; }
  const dt = (pos.timestamp - positionPrecedente.timestamp)/1000;
  const d = calculerDistance(pos.coords, positionPrecedente);
  distanceTotale += d;
  positionPrecedente = pos.coords;
  return dt>0?(d/dt)*3.6:0;
}

function calculerDistance(a,b){
  const R=6371e3, φ1=a.latitude*Math.PI/180, φ2=b.latitude*Math.PI/180;
  const Δφ=(b.latitude-a.latitude)*Math.PI/180, Δλ=(b.longitude-a.longitude)*Math.PI/180;
  const c=2*Math.atan2(Math.sqrt(Math.sin(Δφ/2)**2 + Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2), 
                       Math.sqrt(1-(Math.sin(Δφ/2)**2 + Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2)));
  return R*c;
}

/* ==========================
   Affichages
========================== */
function afficherVitesse(v){
  const moyenne = vitesses.length ? vitesses.reduce((a,b)=>a+b,0)/vitesses.length : 0;
  const mps=v/3.6, mmps=mps*1000;
  safeSetText('vitesse', `Temps : ${((distanceTotale>0?distanceTotale/1:0).toFixed(2))} s | Vitesse instantanée : ${v.toFixed(2)} km/h | Moyenne : ${moyenne.toFixed(2)} km/h | Max : ${vitesseMax.toFixed(2)} km/h | ${mps.toFixed(2)} m/s | ${mmps.toFixed(0)} mm/s`);
}

function afficherDistance(){
  const km = distanceTotale/1000, m=distanceTotale, mm=m*1000;
  const sL = m/299792458, al = sL/(3600*24*365.25);
  safeSetText('distance', `Distance : ${km.toFixed(3)} km | ${m.toFixed(0)} m | ${mm.toFixed(0)} mm | Cosmique : ${sL.toFixed(3)} s lumière | ${al.toExponential(3)} al`);
}

function afficherPourcentage(v){
  const mps=v/3.6;
  safeSetText('pourcentage', `% Lumière : ${(mps/299792458*100).toExponential(2)}% | % Son : ${(mps/343*100).toFixed(2)}%`);
}

function afficherGPS(g){
  safeSetText('gps', `Latitude : ${g.latitude.toFixed(6)} | Longitude : ${g.longitude.toFixed(6)} | Altitude : ${(g.altitude??0).toFixed(1)} m | Précision GPS : ${(g.accuracy??0).toFixed(0)}%`);
}

/* ==========================
   Capteurs
========================== */
function activerCapteurs(){
  const capEl=document.getElementById('capteurs');
  setElementTextBase('capteurs','Niveau à bulle : --° | Lumière : -- lux | Son : -- dB | Fréquence : -- Hz | Magnétisme : -- µT');

  // Lumière ambiante
  if('AmbientLightSensor' in window){
    try{
      const sensor=new AmbientLightSensor();
      sensor.addEventListener('reading',()=>safeSetText('capteurs', _baseTexts['capteurs']+` | Lumière : ${Math.round(sensor.illuminance)} lux`));
      sensor.start();
    }catch{}
  }

  // Microphone
  if(navigator.mediaDevices?.getUserMedia){
    navigator.mediaDevices.getUserMedia({audio:true}).then(stream=>{
      const ctx=new AudioContext();
      const analyser=ctx.createAnalyser();
      const source=ctx.createMediaStreamSource(stream);
      source.connect(analyser);
      analyser.fftSize=2048;
      const data=new Uint8Array(analyser.frequencyBinCount);
      function loop(){
        analyser.getByteFrequencyData(data);
        const moyenne=data.reduce((a,b)=>a+b,0)/data.length;
        const dB=20*Math.log10(moyenne||1);
        safeSetText('capteurs', _baseTexts['capteurs']+` | Son : ${Math.round(dB)} dB | Fréquence : ${analyser.frequencyBinCount} Hz`);
        requestAnimationFrame(loop);
      }
      loop();
    }).catch(){}
  }

  // Niveau à bulle
  if('DeviceOrientationEvent' in window){
    window.addEventListener('deviceorientation', e=>{
      if(e.beta!=null) safeSetText('capteurs', _baseTexts['capteurs']+` | Niveau à bulle : ${e.beta.toFixed(1)}°`);
    });
  }

  // Magnétisme
  if('Magnetometer' in window){
    try{
      const mag=new Magnetometer();
      mag.addEventListener('reading', ()=>safeSetText('capteurs', _baseTexts['capteurs']+` | Magnétisme : ${Math.round(Math.sqrt(mag.x**2+mag.y**2+mag.z**2))} µT`));
      mag.start();
    }catch{}
  }
}

/* ==========================
   Boussole
========================== */
function activerBoussole(){
  if('DeviceOrientationEvent' in window){
    window.addEventListener('deviceorientationabsolute', e=>{
      if(e.alpha!=null){
        const cap=e.alpha.toFixed(0);
        const minecraft=positionPrecedente?`X:${Math.round(positionPrecedente.longitude*1000)} Z:${Math.round(positionPrecedente.latitude*1000)}`:"--";
        const capDest=destination.latitude?`${cap}°`:"--";
        safeSetText('boussole', `Cap : ${cap}° | Coordonnées Minecraft : ${minecraft} | Cap vers destination : ${capDest}`);
      }
    });
  }
}

/* ==========================
   Horloge et médaillon
========================== */
function activerHorloge(){
  const h=document.getElementById('horloge');
  function loop(){ const d=new Date(); h.textContent=`${String(d.getHours()).padStart(2,'0')}:${String(d.getMinutes()).padStart(2,'0')}:${String(d.getSeconds()).padStart(2,'0')}`; requestAnimationFrame(loop);}
  loop();
}

function afficherMedaillon(){
  const med=document.getElementById('medaillon');
  med.innerHTML='';
  const canvas=document.createElement('canvas');
  canvas.width=400; canvas.height=400; med.appendChild(canvas);
  const ctx=canvas.getContext('2d');

  // Fond
  ctx.fillStyle="#000"; ctx.fillRect(0,0,canvas.width,canvas.height);

  // Cercle principal
  ctx.strokeStyle="#0ff"; ctx.lineWidth=2; ctx.beginPath(); ctx.arc(200,200,180,0,2*Math.PI); ctx.stroke();

  // Soleil
  ctx.fillStyle="#ffd700"; ctx.beginPath(); ctx.arc(300,100,12,0,2*Math.PI); ctx.fill(); ctx.fillText("☀️ Soleil",280,90);

  // Lune
  ctx.fillStyle="#ccc"; ctx.beginPath(); ctx.arc(100,150,10,0,2*Math.PI); ctx.fill(); ctx.fillText("🌙 Lune",80,140);

  // Constellations
  const stars=[{name:"Orion",x:240,y:300},{name:"Cassiopeia",x:160,y:280},{name:"Scorpius",x:280,y:250}];
  ctx.fillStyle="#0ff"; stars.forEach(s=>{ctx.beginPath(); ctx.arc(s.x,s.y,3,0,2*Math.PI); ctx.fill(); ctx.fillText(s.name,s.x+5,s.y+5);});

  // Galaxie
  ctx.fillStyle="#f0f"; ctx.beginPath(); ctx.arc(200,350,5,0,2*Math.PI); ctx.fill(); ctx.fillText("🌌 Galaxie",180,360);
}

/* ==========================
   Météo & qualité air
========================== */
function chargerMeteo(){
  const meteoEl=document.getElementById("meteo");
  const airEl=document.getElementById("qualite-air");
  if(!navigator.onLine){meteoEl.textContent="Pas de connexion"; airEl.textContent="Indisponible"; return;}

  fetch("https://api.open-meteo.com/v1/forecast?latitude=43.3&longitude=5.4&current_weather=true")
    .then(r=>r.json()).then(data=>{
      const w=data.current_weather;
      meteoEl.textContent=`Température : ${w.temperature ?? '--'} °C | Vent : ${w.windspeed ?? '--'} km/h | Pression : ${w.pressure ?? '--'} hPa | Humidité : ${w.humidity ?? '--'}%`;
    }).catch(()=>{meteoEl.textContent="Météo indisponible";});

  fetch("https://api.openaq.org/v2/latest?coordinates=43.3,5.4")
    .then(r=>r.json()).then(d=>{
      const m=d.results[0]?.measurements??[];
      const pm25=m.find(a=>a.parameter==="pm25")?.value??"--";
      const uv=m.find(a=>a.parameter==="uv")?.value??"--";
      airEl.textContent=`Qualité air : PM2.5 ${pm25} µg/m³ | Indice UV : ${uv}`;
    }).catch(()=>{airEl.textContent="Indisponible";});
}

/* ==========================
   Grandeurs scientifiques
========================== */
function afficherGrandeurs(){
  safeSetText('grandeurs', `Point ébullition : 100 °C | Gravité : 9.81 m/s² | Vitesse son : 343 m/s | Vitesse lumière : 299792458 m/s`);
}

/* ==========================
   Init boutons
========================== */
document.addEventListener('DOMContentLoaded', ()=>{
  const boutonMarche=document.getElementById('marche');
  const boutonReset=document.getElementById('reset');
  let actif=false;

  boutonMarche.addEventListener('click', ()=>{
    actif=!actif;
    boutonMarche.classList.toggle('active', actif);
    if(actif) demarrerCockpit();
    else arreterCockpit();
  });

  boutonReset.addEventListener('click', ()=>{
    resetVitesseMax();
    appendText('vitesse', ' | Max réinitialisé');
  });
});
  
