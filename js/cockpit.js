let watchId = null;
let positionPrecedente = null;
let distanceTotale = 0;
let vitesseMax = 0;
let vitesses = [];
let startTime = null;
let destination = {latitude:null, longitude:null};

// ======================
// UTILITAIRES DOM
// ======================
function safeSetText(id,text){
  const el = document.getElementById(id);
  if(el) el.textContent = text;
}

// ======================
// GPS et VITESSE
// ======================
function traiterPosition(coords,timestamp){
  if(!startTime) startTime = timestamp;
  const lat = coords.latitude;
  const lon = coords.longitude;
  const alt = coords.altitude ?? 0;
  const acc = coords.accuracy ?? 0;
  const speed = coords.speed !== null ? coords.speed*3.6 : calculerVitesse(coords,timestamp);

  if(positionPrecedente){
    const d = calculerDistance(coords,positionPrecedente);
    distanceTotale += d;
  }
  positionPrecedente = coords;
  vitesses.push(speed);
  vitesseMax = Math.max(vitesseMax,speed);

  afficherVitesse(speed);
  afficherDistance();
  afficherPourcentage(speed);
  afficherGPS(coords);
  afficherMinecraft(coords);
}

function calculerVitesse(coords,timestamp){
  if(!positionPrecedente) return 0;
  const dt = (timestamp - startTime)/1000;
  const d = calculerDistance(coords,positionPrecedente);
  return dt>0? (d/dt)*3.6 : 0;
}

function calculerDistance(a,b){
  const R = 6371e3;
  const φ1 = a.latitude*Math.PI/180;
  const φ2 = b.latitude*Math.PI/180;
  const Δφ = (b.latitude-a.latitude)*Math.PI/180;
  const Δλ = (b.longitude-a.longitude)*Math.PI/180;
  const c = 2*Math.atan2(Math.sqrt(Math.sin(Δφ/2)**2+Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2),
                          Math.sqrt(1-(Math.sin(Δφ/2)**2+Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2)));
  return R*c;
}

function afficherVitesse(v){
  const moyenne = vitesses.length ? vitesses.reduce((a,b)=>a+b,0)/vitesses.length : 0;
  const mps = v/3.6;
  const mmps = mps*1000;
  safeSetText('vitesse',`Temps : ${(performance.now()/1000).toFixed(2)} s | Vitesse instantanée : ${v.toFixed(2)} km/h | Moyenne : ${moyenne.toFixed(2)} km/h | Max : ${vitesseMax.toFixed(2)} km/h | ${mps.toFixed(2)} m/s | ${mmps.toFixed(0)} mm/s`);
}

function afficherDistance(){
  const km = distanceTotale/1000;
  const m = distanceTotale;
  const mm = m*1000;
  const secL = m/299792458;
  const al = secL/(3600*24*365.25);
  safeSetText('distance',`Distance : ${km.toFixed(3)} km | ${m.toFixed(0)} m | ${mm.toFixed(0)} mm | Cosmique : ${secL.toFixed(3)} s lumière | ${al.toExponential(3)} al`);
}

function afficherPourcentage(v){
  const mps = v/3.6;
  const pctL = (mps/299792458)*100;
  const pctS = (mps/343)*100;
  safeSetText('pourcentage',`% Lumière : ${pctL.toExponential(2)}% | % Son : ${pctS.toFixed(2)}%`);
}

function afficherGPS(coords){
  safeSetText('gps',`Latitude : ${coords.latitude.toFixed(6)} | Longitude : ${coords.longitude.toFixed(6)} | Altitude : ${(coords.altitude??0).toFixed(1)} m | Précision GPS : ${(coords.accuracy??0).toFixed(0)}%`);
}

function afficherMinecraft(coords){
  const x = Math.round(coords.longitude*1000);
  const y = Math.round(coords.altitude ?? 0);
  const z = Math.round(coords.latitude*1000);
  safeSetText('boussole',`Coordonnées Minecraft : X:${x} Y:${y} Z:${z}`);
}

// ======================
// MÉDAILLON COSMIQUE
// ======================
function afficherMedaillonComplet(){
  const med = document.getElementById('medaillon');
  med.innerHTML = "";
  const c = document.createElement('canvas'); c.width=400; c.height=400;
  med.appendChild(c);
  const ctx = c.getContext('2d');

  const cx=200,cy=200,r=180;
  ctx.fillStyle="#000"; ctx.fillRect(0,0,400,400);
  ctx.strokeStyle="#0ff"; ctx.lineWidth=2;
  ctx.beginPath(); ctx.arc(cx,cy,r,0,2*Math.PI); ctx.stroke();

  ctx.fillStyle="#ffd700"; ctx.beginPath(); ctx.arc(cx+80,cy-80,12,0,2*Math.PI); ctx.fill(); ctx.fillText("☀️ Soleil",cx+60,cy-100);
  ctx.fillStyle="#ccc"; ctx.beginPath(); ctx.arc(cx-80,cy-60,10,0,2*Math.PI); ctx.fill(); ctx.fillText("🌙 Lune",cx-110,cy-80);

  const stars=[{name:"Orion",x:cx+30,y:cy+80},{name:"Cassiopeia",x:cx-50,y:cy+60},{name:"Scorpius",x:cx+100,y:cy+30}];
  ctx.fillStyle="#0ff"; stars.forEach(s=>{ctx.beginPath(); ctx.arc(s.x,s.y,3,0,2*Math.PI); ctx.fill(); ctx.fillText(s.name,s.x+5,s.y+5);});

  ctx.fillStyle="#f0f"; ctx.beginPath(); ctx.arc(cx,cy+140,4,0,2*Math.PI); ctx.fill(); ctx.fillText("🌌 Galaxie",cx-30,cy+160);
}

// ======================
// HORLOGE
// ======================
function activerHorloge(){
  const el = document.getElementById('horloge');
  function loop(){
    const d = new Date();
    el.textContent = d.toLocaleTimeString();
    requestAnimationFrame(loop);
  }
  loop();
}

// ======================
// INITIALISATION
// ======================
document.getElementById('marche').addEventListener('click',()=>{
  if(!watchId){
    navigator.geolocation.getCurrentPosition(
      pos=>traiterPosition(pos.coords,pos.timestamp),
      err=>console.error(err)
    );
    watchId = navigator.geolocation.watchPosition(
      pos=>traiterPosition(pos.coords,pos.timestamp),
      err=>console.error(err),
      {enableHighAccuracy:true, maximumAge:0, timeout:10000}
    );
    activerHorloge();
    activerCapteurs();
    afficherMedaillonComplet();
    document.getElementById('marche').textContent = '⏹️ Arrêter';
  }else{
    navigator.geolocation.clearWatch(watchId);
    watchId=null;
    document.getElementById('marche').textContent = '▶️ Démarrer';
  }
});

document.getElementById('reset').addEventListener('click',()=>{vitesseMax=0; vitesses=[]; distanceTotale=0; startTime=null;});
   
