let watchId = null;
let positionPrecedente = null;
let vitesseMax = 0;
let vitesses = [];
let distanceTotale = 0;
let destination = { latitude: null, longitude: null };

/* ========================
   UTILITAIRES DOM
======================== */
function safeSetText(id, text) {
  const el = document.getElementById(id);
  if (el) el.textContent = text;
}
function appendText(id, text) {
  const el = document.getElementById(id);
  if (el) el.textContent = (el.textContent.length > 1000) ? text : (el.textContent + text);
}
const _baseTexts = {};
function setElementTextBase(id, base) { _baseTexts[id] = base; safeSetText(id, base); }
function getElementText(id) { const el = document.getElementById(id); return el ? el.textContent : (_baseTexts[id] || ''); }

/* ========================
   GPS & VITESSE
======================== */
export function demarrerCockpit() {
  if (!("geolocation" in navigator)) { safeSetText('gps', 'GPS non disponible'); return; }
  if (watchId !== null) return;

  try {
    watchId = navigator.geolocation.watchPosition(
      pos => {
        const gps = {
          latitude: pos.coords.latitude,
          longitude: pos.coords.longitude,
          altitude: pos.coords.altitude,
          accuracy: pos.coords.accuracy,
          timestamp: pos.timestamp,
          speed: pos.coords.speed
        };
        const vitesse = (typeof gps.speed === 'number' && gps.speed !== null) ? gps.speed*3.6 : calculerVitesse(gps);

        if (vitesse >= 0 && vitesse < 300) {
          vitesseMax = Math.max(vitesseMax, vitesse);
          vitesses.push(vitesse);
          afficherVitesse(vitesse);
          afficherDistance();
          afficherPourcentage(vitesse);
          afficherGPS(gps);
        }
      },
      err => { safeSetText('gps','Erreur GPS : ' + (err.message||err.code||'inconnue')); },
      { enableHighAccuracy:true, maximumAge:0, timeout:10000 }
    );
  } catch(e) { safeSetText('gps','GPS indisponible (exception)'); }
}
export function arreterCockpit() {
  if(watchId!==null) navigator.geolocation.clearWatch(watchId);
  watchId=null; positionPrecedente=null; vitesses=[]; distanceTotale=0; vitesseMax=0;
}
export function resetVitesseMax() { vitesseMax=0; }

/* ========================
   CALCULS
======================== */
function calculerVitesse(gps){
  if(!positionPrecedente){ positionPrecedente=gps; return 0; }
  const dt=(gps.timestamp-positionPrecedente.timestamp)/1000;
  const d=calculerDistance(gps, positionPrecedente);
  if(!Number.isFinite(d)||d<0){ positionPrecedente=gps; return 0; }
  distanceTotale+=d;
  positionPrecedente=gps;
  return dt>0 ? (d/dt)*3.6 : 0;
}
function calculerDistance(pos1,pos2){
  const R=6371e3, φ1=pos1.latitude*Math.PI/180, φ2=pos2.latitude*Math.PI/180;
  const Δφ=(pos2.latitude-pos1.latitude)*Math.PI/180, Δλ=(pos2.longitude-pos1.longitude)*Math.PI/180;
  const a=Math.sin(Δφ/2)**2+Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2;
  const c=2*Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
  return R*c;
}

/* ========================
   AFFICHAGE
======================== */
function afficherVitesse(v){
  const moyenne=vitesses.length? vitesses.reduce((a,b)=>a+b,0)/vitesses.length :0;
  const mps=v/3.6, mmps=mps*1000;
  safeSetText('vitesse',
    `Temps : ${new Date().toLocaleTimeString()} | Vitesse : ${v.toFixed(2)} km/h | Moyenne : ${moyenne.toFixed(2)} km/h | Max : ${vitesseMax.toFixed(2)} km/h | ${mps.toFixed(2)} m/s | ${mmps.toFixed(0)} mm/s`
  );
}
function afficherDistance(){
  const km=distanceTotale/1000, m=distanceTotale, mm=m*1000;
  const secLumiere=m/299792458, al=secLumiere/(3600*24*365.25);
  safeSetText('distance', `Distance : ${km.toFixed(3)} km | ${m.toFixed(0)} m | ${mm.toFixed(0)} mm | Cosmique : ${secLumiere.toFixed(3)} s lumière | ${al.toExponential(3)} al`);
}
function afficherPourcentage(v){
  const mps=v/3.6;
  safeSetText('pourcentage', `% Lumière : ${(mps/299792458*100).toExponential(2)}% | % Son : ${(mps/343*100).toFixed(2)}%`);
}
function afficherGPS(g){ safeSetText('gps',`Latitude : ${g.latitude.toFixed(6)} | Longitude : ${g.longitude.toFixed(6)} | Altitude : ${(g.altitude??'--')==='--'?'--':g.altitude.toFixed(1)+' m'} | Précision GPS : ${g.accuracy!=null?g.accuracy.toFixed(0)+'%':'--'}`); }

/* ========================
   CAPTEURS (au clic)
======================== */
function activerCapteurs(){
  setElementTextBase('capteurs','Niveau à bulle : --° | Lumière : -- lux');
  
  // Lumière
  if('AmbientLightSensor' in window){
    try{ const sensor=new AmbientLightSensor(); sensor.addEventListener('reading',()=>{ appendText('capteurs', ` | Lumière : ${Math.round(sensor.illuminance)} lux`); }); sensor.start(); }
    catch(err){ console.warn('AmbientLightSensor indisponible',err); }
  }

  // Microphone
  if(navigator.mediaDevices?.getUserMedia){
    navigator.mediaDevices.getUserMedia({audio:true}).then(stream=>{
      try{
        const audioCtx=new (window.AudioContext||window.webkitAudioContext)();
        const analyser=audioCtx.createAnalyser();
        const source=audioCtx.createMediaStreamSource(stream);
        source.connect(analyser);
        analyser.fftSize=2048;
        const data=new Uint8Array(analyser.frequencyBinCount);
        function analyserSon(){
          analyser.getByteFrequencyData(data);
          const moyenne=data.reduce((a,b)=>a+b,0)/data.length;
          const dB=20*Math.log10(moyenne||1);
          const Hz=analyser.frequencyBinCount;
          safeSetText('capteurs', getElementText('capteurs')+` | Son : ${Math.round(dB)} dB | Fréquence : ${Hz} Hz`);
          requestAnimationFrame(analyserSon);
        }
        analyserSon();
      }catch(e){ console.warn('Erreur audio analyser',e); }
    }).catch(err=>console.warn('Microphone refusé',err));
  }

  // Niveau à bulle
  if('DeviceOrientationEvent' in window){
    window.addEventListener('deviceorientation', e=>{
      if(typeof e.beta==='number') appendText('capteurs', ` | Niveau à bulle : ${e.beta.toFixed(1)}°`);
    });
  }

  // Magnétisme
  if('Magnetometer' in window){
    try{ const magneto=new Magnetometer(); magneto.addEventListener('reading',()=>{ const champ=Math.sqrt(magneto.x**2+magneto.y**2+magneto.z**2); appendText('capteurs', ` | Magnétisme : ${Math.round(champ)} µT`); }); magneto.start(); }
    catch(err){ console.warn('Magnetometer indisponible',err); }
  }
}

/* ========================
   BOUTONS & INIT
======================== */
document.addEventListener('DOMContentLoaded', ()=>{
  const boutonMarche=document.getElementById('marche');
  const boutonReset=document.getElementById('reset');

  let actif=false;
  if(boutonMarche){
    boutonMarche.addEventListener('click', async ()=>{
      actif=!actif;
      if(actif){
        demarrerCockpit();
        activerCapteurs();
        boutonMarche.textContent='⏹️ Arrêter';
      }else{
        arreterCockpit();
        boutonMarche.textContent='▶️ Marche';
        safeSetText('vitesse','Temps : -- | Vitesse instantanée : -- km/h | Moyenne : -- | Max : --');
        safeSetText('distance','Distance : -- km | -- m | -- mm | Cosmique : --');
        safeSetText('pourcentage','% Lumière : --% | % Son : --%');
        safeSetText('gps','Latitude : -- | Longitude : -- | Altitude : -- | Précision GPS : --%');
        safeSetText('capteurs','Niveau à bulle : --° | Lumière : -- lux | Son : -- dB | Fréquence : -- Hz | Magnétisme : -- µT');
      }
    });
  }

  if(boutonReset){
    boutonReset.addEventListener('click', ()=>{
      resetVitesseMax();
      appendText('vitesse',' | Max réinitialisé');
    });
  }
});
   
