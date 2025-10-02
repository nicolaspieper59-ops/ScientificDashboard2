let watchId = null;
let positionPrecedente = null;
let distanceTotale = 0;
let vitesseMax = 0;
let vitesses = [];
let destination = { latitude: null, longitude: null };
let masseSimulee = 70; // kg par défaut

/* ======================
   GPS réel & suivi
====================== */
export function demarrerCockpit() {
  if (!('geolocation' in navigator)) return alert('GPS non disponible');

  if (watchId !== null) return;

  // Position initiale
  navigator.geolocation.getCurrentPosition(
    pos => traiterPosition(pos.coords, pos.timestamp),
    err => console.error(err),
    { enableHighAccuracy:true }
  );

  // Suivi continu
  watchId = navigator.geolocation.watchPosition(
    pos => traiterPosition(pos.coords, pos.timestamp),
    err => console.error(err),
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
  if (watchId !== null) navigator.geolocation.clearWatch(watchId);
  watchId = null;
  positionPrecedente = null;
  vitesses = [];
  distanceTotale = 0;
  vitesseMax = 0;
}

export function resetVitesseMax() { vitesseMax = 0; }
export function definirDestination(lat, lon) { destination.latitude = lat; destination.longitude = lon; }
export function definirMasse(m) { masseSimulee = m; }

/* ======================
   Traitement GPS
====================== */
function traiterPosition(coords, timestamp){
  const gps = {
    latitude: coords.latitude,
    longitude: coords.longitude,
    altitude: coords.altitude ?? 0,
    accuracy: coords.accuracy ?? 0,
    speed: coords.speed ?? 0
  };

  const vitesse = gps.speed ? gps.speed*3.6 : calculerVitesse(gps, timestamp);
  if(vitesse>=0 && vitesse<300){
    vitesseMax = Math.max(vitesseMax, vitesse);
    vitesses.push(vitesse);
    afficherVitesse(vitesse);
    afficherDistance();
    afficherPourcentage(vitesse);
    afficherGPS(gps);
  }
  positionPrecedente = gps;
}

/* ======================
   Calculs
====================== */
function calculerVitesse(gps, timestamp){
  if(!positionPrecedente) return 0;
  const dt = (timestamp - (positionPrecedente.timestamp||timestamp))/1000;
  const d = calculerDistance(gps, positionPrecedente);
  distanceTotale += d;
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
   Affichage
====================== */
function afficherVitesse(v){
  const moyenne = vitesses.length? vitesses.reduce((a,b)=>a+b,0)/vitesses.length:0;
  document.getElementById("vitesse").textContent=
    `Vitesse instantanée : ${v.toFixed(2)} km/h | Moyenne : ${moyenne.toFixed(2)} | Max : ${vitesseMax.toFixed(2)} | ${ (v/3.6).toFixed(2)} m/s | ${(v/3.6*1000).toFixed(0)} mm/s`;
}

function afficherDistance(){
  const km = distanceTotale/1000;
  const m = distanceTotale;
  const mm = m*1000;
  const sLumiere = m/299792458;
  const aL = sLumiere/(3600*24*365.25);
  document.getElementById("distance").textContent=
    `Distance : ${km.toFixed(3)} km | ${m.toFixed(0)} m | ${mm.toFixed(0)} mm | Cosmique : ${sLumiere.toFixed(3)} s lumière | ${aL.toExponential(3)} al`;
}

function afficherPourcentage(v){
  const mps = v/3.6;
  document.getElementById("pourcentage").textContent=
    `% Lumière : ${(mps/299792458*100).toExponential(2)}% | % Son : ${(mps/343*100).toFixed(2)}%`;
}

function afficherGPS(g){
  document.getElementById("gps").textContent=
    `Latitude : ${g.latitude.toFixed(6)} | Longitude : ${g.longitude.toFixed(6)} | Altitude : ${g.altitude.toFixed(1)} m | Précision GPS : ${g.accuracy.toFixed(0)}%`;
}

/* ======================
   Capteurs
====================== */
function activerCapteurs(){
  const cap = document.getElementById("capteurs");
  cap.textContent="";

  // Lumière
  if('AmbientLightSensor' in window){
    try{
      const sensor = new AmbientLightSensor();
      sensor.addEventListener('reading',()=> cap.textContent = `Lumière : ${sensor.illuminance} lux`);
      sensor.start();
    }catch{}
  }

  // Microphone
  if(navigator.mediaDevices?.getUserMedia){
    navigator.mediaDevices.getUserMedia({audio:true}).then(stream=>{
      const ctx = new AudioContext();
      const analyser = ctx.createAnalyser();
      const source = ctx.createMediaStreamSource(stream);
      source.connect(analyser);
      const data = new Uint8Array(analyser.frequencyBinCount);
      (function loop(){
        analyser.getByteFrequencyData(data);
        const dB = 20*Math.log10(data.reduce((a,b)=>a+b,0)/data.length||1);
        cap.textContent += ` | Son : ${dB.toFixed(0)} dB | Fréquence : ${analyser.frequencyBinCount} Hz`;
        requestAnimationFrame(loop);
      })();
    }).catch(){}
  }

  // Orientation
  if('DeviceOrientationEvent' in window){
    window.addEventListener('deviceorientation', e=>{
      if(e.beta!=null) cap.textContent += ` | Niveau : ${e.beta.toFixed(1)}°`;
    });
  }

  // Magnétomètre
  if('Magnetometer' in window){
    try{
      const mag = new Magnetometer();
      mag.addEventListener('reading',()=> cap.textContent += ` | Magnétisme : ${Math.sqrt(mag.x**2+mag.y**2+mag.z**2).toFixed(0)} µT`);
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
      if(e.alpha!=null){
        const cap = e.alpha.toFixed(0);
        const minecraft = positionPrecedente?`X:${Math.round(positionPrecedente.longitude*1000)} Z:${Math.round(positionPrecedente.latitude*1000)}`:"--";
        const capDest = destination.latitude? `${cap}°`:"--";
        document.getElementById("boussole").textContent=`Cap : ${cap}° | Coordonnées Minecraft : ${minecraft} | Cap vers destination : ${capDest}`;
      }
    });
  }
}

/* ======================
   Horloge
====================== */
function activerHorloge(){
  const h = document.getElementById("horloge");
  (function loop(){
    const d = new Date();
    h.textContent=`${String(d.getHours()).padStart(2,'0')}:${String(d.getMinutes()).padStart(2,'0')}:${String(d.getSeconds()).padStart(2,'0')}`;
    requestAnimationFrame(loop);
  })();
}

/* ======================
   Médaillon cosmique
====================== */
function afficherMedaillon(){
  const med=document.getElementById("medaillon");
  if(!med) return;
  med.innerHTML="";
  const c=document.createElement("canvas");
  c.width=400; c.height=400;
  med.appendChild(c);
  const ctx=c.getContext("2d");

  const cx=c.width/2, cy=c.height/2, r=180;
  ctx.fillStyle="#000"; ctx.fillRect(0,0,c.width,c.height);
  ctx.strokeStyle="#0ff"; ctx.lineWidth=2;
  ctx.beginPath(); ctx.arc(cx,cy,r,0,2*Math.PI); ctx.stroke();

  // Soleil
  const sunX = cx + r*0.6*Math.cos(Date.now()/1e7);
  const sunY = cy + r*0.6*Math.sin(Date.now()/1e7);
  ctx.fillStyle="#ffd700"; ctx.beginPath(); ctx.arc(sunX,sunY,10,0,2*Math.PI); ctx.fill();
  ctx.fillStyle="#fff"; ctx.fillText("☀️ Soleil", sunX+10, sunY-10);

  // Lune
  const moonX = cx + r*0.5*Math.cos(Date.now()/1.3e7);
  const moonY = cy + r*0.5*Math.sin(Date.now()/1.3e7);
  ctx.fillStyle="#ccc"; ctx.beginPath(); ctx.arc(moonX,moonY,8,0,2*Math.PI); ctx.fill();
  ctx.fillText("🌙 Lune", moonX-20, moonY-10);

  // Constellations
  const stars=[{name:"Orion",x:cx+50,y:cy+100},{name:"Cassiopeia",x:cx-70,y:cy+40},{name:"Scorpius",x:cx+90,y:cy-20}];
  ctx.fillStyle="#0ff"; stars.forEach(s=>{ctx.beginPath();ctx.arc(s.x,s.y,2,0,2*Math.PI);ctx.fill(); ctx.fillText(s.name,s.x+5,s.y+5);});

  // Galaxie
  ctx.fillStyle="#f0f"; ctx.beginPath(); ctx.arc(cx,cy+150,4,0,2*Math.PI); ctx.fill();
  ctx.fillText("🌌 Galaxie", cx-30, cy+165);

  requestAnimationFrame(afficherMedaillon);
}

/* ======================
   Météo & Air
====================== */
function chargerMeteo(){
  if(!navigator.onLine) return;
  fetch("https://api.open-meteo.com/v1/forecast?latitude=43.3&longitude=5.4&current_weather=true")
    .then(r=>r.json()).then(data=>{
      const w=data.current_weather;
      document.getElementById("meteo").textContent=`Temp : ${w.temperature}°C | Vent : ${w.windspeed} km/h`;
    }).catch(()=>{document.getElementById("meteo").textContent="Météo indisponible";});

  fetch("https://api.openaq.org/v2/latest?coordinates=43.3,5.4")
    .then(r=>r.json()).then(d=>{
      const m=d.results[0]?.measurements??[];
      const pm25=m.find(a=>a.parameter==="pm25")?.value??"--";
      const uv=m.find(a=>a.parameter==="uv")?.value??"--";
      document.getElementById("qualite-air").textContent=`Qualité air : PM2.5 ${pm25} µg/m³ | Indice UV : ${uv}`;
    }).catch(()=>{document.getElementById("qualite-air").textContent="Indisponible";});
}

/* ======================
   Grandeurs scientifiques
====================== */
function afficherGrandeurs(){
  const gravite = 9.81;
  const vitesse = vitesses.length? vitesses[vitesses.length-1]/3.6:0;
  const force = masseSimulee * gravite;
  const energie = masseSimulee * gravite * 1; // hauteur simulée 1m
  const cinetique = 0.5 * masseSimulee * vitesse**2;

  document.getElementById("grandeurs").textContent=
    `Point ébullition : 100°C | Gravité : ${gravite.toFixed(2)} m/s² | Force : ${force.toFixed(2)} N | Energie : ${energie.toFixed(2)} J | Cinétique : ${cinetique.toFixed(2)} J`;
                       }
