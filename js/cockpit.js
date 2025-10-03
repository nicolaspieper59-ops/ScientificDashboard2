import * as SunCalc from "https://cdn.jsdelivr.net/npm/suncalc@1.9.0/suncalc.js";

let watchId = null, positionPrecedente = null, vitesses = [], vitesseMax = 0, distanceTotale = 0;

const set = (id, txt) => document.getElementById(id).textContent = txt;

// === CONTROLES GPS ===
document.getElementById('marche').onclick = () => {
  if (!('geolocation' in navigator)) return set('vitesse', '❌ GPS non disponible');
  if (watchId !== null) return;

  navigator.geolocation.getCurrentPosition(pos => traiter(pos.coords, pos.timestamp));
  watchId = navigator.geolocation.watchPosition(
    pos => traiter(pos.coords, pos.timestamp),
    err => set('gps', 'Erreur GPS'),
    { enableHighAccuracy: true }
  );

  activerHorloge();
  synchroniserHeureAtomique();
  activerCapteurs();
  afficherMedaillon();
  chargerMeteo();
  chargerGrandeurs();
};

document.getElementById('stop').onclick = () => {
  if (watchId !== null) {
    navigator.geolocation.clearWatch(watchId);
    watchId = null;
    set('vitesse', '⏹️ Suivi arrêté');
  }
};

document.getElementById('reset').onclick = () => {
  vitesses = []; vitesseMax = 0; distanceTotale = 0; positionPrecedente = null;
  ['vitesse','distance','pourcentage','gps'].forEach(id => set(id,'--'));
};

// === TRAITEMENT GPS ===
function traiter(coords, timestamp) {
  const v = coords.speed != null ? coords.speed * 3.6 : calculerVitesse(coords, timestamp);
  if (v >= 0 && v < 300) {
    vitesseMax = Math.max(vitesseMax, v);
    vitesses.push(v);
    const moy = vitesses.reduce((a,b)=>a+b,0)/vitesses.length;
    const mps = v / 3.6;

    set('vitesse', `Vitesse : ${v.toFixed(2)} km/h | Moy : ${moy.toFixed(2)} | Max : ${vitesseMax.toFixed(2)}`);
    set('distance', `Distance : ${(distanceTotale/1000).toFixed(3)} km`);
    set('pourcentage', `% Lumière : ${(mps/299792458*100).toExponential(2)}% | % Son : ${(mps/343*100).toFixed(2)}%`);
    set('gps', `Précision GPS : ${(coords.accuracy ?? 0).toFixed(0)} m`);

    afficherMedaillon(coords.latitude, coords.longitude);
    chargerGrandeurs();
  }
}

function calculerVitesse(c,t) {
  if (!positionPrecedente) { positionPrecedente = {...c,timestamp:t}; return 0; }
  const dt = (t - positionPrecedente.timestamp)/1000;
  const d = calculerDistance(c, positionPrecedente);
  distanceTotale += d;
  positionPrecedente = {...c,timestamp:t};
  return dt>0 ? (d/dt)*3.6 : 0;
}

function calculerDistance(a,b) {
  const R=6371e3;
  const φ1=a.latitude*Math.PI/180, φ2=b.latitude*Math.PI/180;
  const Δφ=(b.latitude-a.latitude)*Math.PI/180;
  const Δλ=(b.longitude-a.longitude)*Math.PI/180;
  const aVal=Math.sin(Δφ/2)**2+Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2;
  return R*2*Math.atan2(Math.sqrt(aVal),Math.sqrt(1-aVal));
}

// === HORLOGE ===
function activerHorloge() {
  const h=document.getElementById('horloge-minecraft');
  function tick(){const d=new Date();h.textContent=`${String(d.getHours()).padStart(2,'0')}:${String(d.getMinutes()).padStart(2,'0')}:${String(d.getSeconds()).padStart(2,'0')}`;requestAnimationFrame(tick);}
  tick();
}
function synchroniserHeureAtomique() {
  fetch('https://worldtimeapi.org/api/ip').then(r=>r.json()).then(data=>{
    const utc=new Date(data.utc_datetime);
    set('horloge-atomique',`Heure atomique (UTC) : ${utc.toLocaleTimeString()}`);
  }).catch(()=>set('horloge-atomique','Heure atomique indisponible'));
}

// === CAPTEURS ===
function activerCapteurs(){
  // Lumière
  if('AmbientLightSensor' in window){
    try{const l=new AmbientLightSensor();l.addEventListener('reading',()=>{majCapteurs(`Lumière : ${Math.round(l.illuminance)} lux`)});l.start();}catch{}
  }
  // Son
  if(navigator.mediaDevices?.getUserMedia){
    navigator.mediaDevices.getUserMedia({audio:true}).then(stream=>{
      const ctx=new AudioContext();const analyser=ctx.createAnalyser();
      const src=ctx.createMediaStreamSource(stream);src.connect(analyser);
      const data=new Uint8Array(analyser.frequencyBinCount);
      (function loop(){analyser.getByteFrequencyData(data);
        const dB=20*Math.log10(data.reduce((a,b)=>a+b,0)/data.length||1);
        majCapteurs(`Son : ${Math.round(dB)} dB`);requestAnimationFrame(loop);})();
    });
  }
  // Orientation
  if('DeviceOrientationEvent' in window){
    window.addEventListener('deviceorientation',e=>{
      majCapteurs(`Orientation : ${e.alpha?.toFixed(1)}°`);
    });
  }
}
function majCapteurs(txt){
  let base=document.getElementById('capteurs').textContent;
  if(base.includes('|')) base=base.split('|')[0];
  set('capteurs',`${base} | ${txt}`);
}

// === MEDAILLON ===
function afficherMedaillon(lat=43.3,lon=5.4){
  const now=new Date();
  const soleil=SunCalc.getTimes(now,lat,lon);
  const lune=SunCalc.getMoonPosition(now,lat,lon);
  const luneTimes=SunCalc.getMoonTimes(now,lat,lon);
  const phase=SunCalc.getMoonIllumination(now);

  set('culmination-soleil',`☀️ Culmination : ${soleil.solarNoon.toLocaleTimeString()}`);
  set('heure-vraie',`Heure solaire vraie : ${soleil.sunrise.toLocaleTimeString()}`);
  set('heure-moyenne',`Heure solaire moyenne : ${soleil.sunset.toLocaleTimeString()}`);
  set('equation-temps',`Équation du temps : ${(((soleil.sunset-soleil.sunrise)/60000)-720).toFixed(2)} min`);
  set('lune-phase',`🌗 Phase : ${(phase.fraction*100).toFixed(2)}%`);
  set('lune-mag',`🔭 Altitude : ${lune.altitude.toFixed(2)} rad`);
  set('lune-lever',`🌙 Lever : ${luneTimes.rise?.toLocaleTimeString() ?? '--'}`);
  set('lune-coucher',`🌙 Coucher : ${luneTimes.set?.toLocaleTimeString() ?? '--'}`);
  set('lune-culmination',`🌙 Culmination : ${luneTimes.alwaysUp?'Toujours visible':luneTimes.alwaysDown?'Invisible':'--'}`);
}

// === METEO ===
function chargerMeteo(){
  fetch("https://api.open-meteo.com/v1/forecast?latitude=43.3&longitude=5.4&current_weather=true")
    .then(r=>r.json()).then(d=>{
      const w=d.current_weather;
      console.log("Météo :",w);
    });
}

// === GRANDEURS ===
function chargerGrandeurs(){
  const masse=70;
  const v=vitesses.length?vitesses.at(-1)/3.6:0;
  const F=masse*9.81,E=0.5*masse*v**2,P=F*v;
  set('grandeurs',`🧲 Force : ${F.toFixed(1)} N | 🔋 Énergie : ${E.toFixed(1)} J | ⚡ Puissance : ${P.toFixed(1)} W`);
        }
          
