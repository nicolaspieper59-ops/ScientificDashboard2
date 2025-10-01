// === Variables globales ===
let watchId = null;
let positionPrecedente = null;
let vitesseMax = 0;
let vitesses = [];
let distanceTotale = 0;
let destination = { latitude: null, longitude: null };
let intensitesSon = [];
let intensitesLumiere = [];

/* ========================
   GESTION GPS / VITESSE
======================== */
export function demarrerCockpit() {
  if (!("geolocation" in navigator)) {
    document.getElementById("gps").textContent = "GPS non disponible";
    return;
  }
  if (watchId !== null) return;

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
      const vitesse = gps.speed !== null ? gps.speed*3.6 : calculerVitesse(gps);
      if(vitesse>=0 && vitesse<300){
        vitesseMax = Math.max(vitesseMax,vitesse);
        vitesses.push(vitesse);
        afficherVitesse(vitesse);
        afficherDistance();
        afficherPourcentage(vitesse);
        afficherGPS(gps);
      }
    },
    err => {
      console.error("Erreur GPS :", err);
      document.getElementById("gps").textContent = "Erreur GPS : " + err.message;
    },
    { enableHighAccuracy:true, maximumAge:0, timeout:10000 }
  );
}

export function arreterCockpit(){
  if(watchId!==null){
    navigator.geolocation.clearWatch(watchId);
    watchId = null;
    positionPrecedente = null;
    vitesses = [];
    distanceTotale = 0;
  }
}

export function resetVitesseMax(){ vitesseMax=0; }

function calculerVitesse(gps){
  if(!positionPrecedente){ positionPrecedente=gps; return 0; }
  const dt=(gps.timestamp-positionPrecedente.timestamp)/1000;
  const d=calculerDistance(gps,positionPrecedente);
  distanceTotale+=d;
  positionPrecedente=gps;
  return dt>0?(d/dt)*3.6:0;
}

function calculerDistance(pos1,pos2){
  const R=6371e3,φ1=pos1.latitude*Math.PI/180,φ2=pos2.latitude*Math.PI/180;
  const Δφ=(pos2.latitude-pos1.latitude)*Math.PI/180;
  const Δλ=(pos2.longitude-pos1.longitude)*Math.PI/180;
  const a=Math.sin(Δφ/2)**2+Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2;
  const c=2*Math.atan2(Math.sqrt(a),Math.sqrt(1-a));
  return R*c;
}

/* ========================
   AFFICHAGE
======================== */
function afficherVitesse(vitesse){
  const moyenne=vitesses.length?vitesses.reduce((a,b)=>a+b,0)/vitesses.length:0;
  const mps=vitesse/3.6, mmps=mps*1000;
  document.getElementById("vitesse").textContent=
    `Temps : ${new Date().toLocaleTimeString()} | Vitesse : ${vitesse.toFixed(2)} km/h | Moy : ${moyenne.toFixed(2)} km/h | Max : ${vitesseMax.toFixed(2)} km/h | ${mps.toFixed(2)} m/s | ${mmps.toFixed(0)} mm/s`;
}

function afficherDistance(){
  const km=distanceTotale/1000, m=distanceTotale, mm=m*1000;
  const secondesLumiere=m/299792458;
  const anneesLumiere=secondesLumiere/(3600*24*365.25);
  document.getElementById("distance").textContent=
    `Distance : ${km.toFixed(3)} km | ${m.toFixed(0)} m | ${mm.toFixed(0)} mm | Cosmique : ${secondesLumiere.toFixed(3)} s lumière | ${anneesLumiere.toExponential(3)} al`;
}

function afficherPourcentage(vitesse){
  const mps=vitesse/3.6;
  const pourcentLumiere=(mps/299792458)*100;
  const pourcentSon=(mps/343)*100;
  document.getElementById("pourcentage").textContent=
    `% Lumière : ${pourcentLumiere.toExponential(2)}% | % Son : ${pourcentSon.toFixed(2)}%`;
}

function afficherGPS(gps){
  document.getElementById("gps").textContent=
    `Latitude : ${gps.latitude.toFixed(6)} | Longitude : ${gps.longitude.toFixed(6)} | Altitude : ${(gps.altitude??0).toFixed(1)} m | Précision : ${gps.accuracy.toFixed(0)}%`;
}

/* ========================
   CAPTEURS PHYSIQUES
======================== */
function activerCapteurs(){
  if("AmbientLightSensor" in window){
    try{
      const lightSensor=new AmbientLightSensor();
      lightSensor.addEventListener("reading",()=>{
        intensitesLumiere.push(lightSensor.illuminance);
        const max=Math.max(...intensitesLumiere);
        const moy=intensitesLumiere.reduce((a,b)=>a+b,0)/intensitesLumiere.length;
        document.getElementById("capteurs").textContent=`Lumière : ${lightSensor.illuminance.toFixed(0)} lux | Moy : ${moy.toFixed(0)} | Max : ${max.toFixed(0)}`;
      });
      lightSensor.start();
    }catch(e){console.warn("Capteur lumière indisponible :",e);}
  }

  if(navigator.mediaDevices?.getUserMedia){
    navigator.mediaDevices.getUserMedia({audio:true}).then(stream=>{
      const ctx=new AudioContext(), analyser=ctx.createAnalyser();
      const source=ctx.createMediaStreamSource(stream);
      source.connect(analyser);
      const data=new Uint8Array(analyser.frequencyBinCount);
      function analyserSon(){
        analyser.getByteFrequencyData(data);
        const moy=data.reduce((a,b)=>a+b,0)/data.length;
        const dB=20*Math.log10(moy||1);
        intensitesSon.push(dB);
        const max=Math.max(...intensitesSon);
        document.getElementById("capteurs").textContent+=` | Son : ${dB.toFixed(0)} dB | Moy : ${moy.toFixed(0)} | Max : ${max.toFixed(0)} dB`;
        requestAnimationFrame(analyserSon);
      }
      analyserSon();
    }).catch(err=>console.warn("Microphone indisponible :",err));
  }

  if("DeviceOrientationEvent" in window){
    window.addEventListener("deviceorientation",e=>{
      if(e.beta!=null) document.getElementById("capteurs").textContent+=` | Niveau : ${e.beta.toFixed(1)}°`;
    });
  }

  if("Magnetometer" in window){
    try{
      const magneto=new Magnetometer();
      magneto.addEventListener("reading",()=>{
        const champ=Math.sqrt(magneto.x**2+magneto.y**2+magneto.z**2);
        document.getElementById("capteurs").textContent+=` | Magnétisme : ${champ.toFixed(0)} µT`;
      });
      magneto.start();
    }catch(e){console.warn("Magnétomètre indisponible :",e);}
  }
}

/* ========================
   BOUSSOLE & DESTINATION
======================== */
function activerBoussole(){
  if("DeviceOrientationEvent" in window){
    const eventName="deviceorientationabsolute" in window?"deviceorientationabsolute":"deviceorientation";
    window.addEventListener(eventName,e=>{
      if(e.alpha!=null){
        const cap=e.alpha;
        const coordMC=convertirCoordonneesMinecraft();
        const capDest=calculerCapVersDestination(cap);
        document.getElementById("boussole").textContent=`Cap : ${cap.toFixed(0)}° | Coord Minecraft : ${coordMC} | Cap vers dest : ${capDest}`;
      }
    });
  }
}

function convertirCoordonneesMinecraft(){
  if(!positionPrecedente) return "--";
  const x=Math.round(positionPrecedente.longitude*1000);
  const z=Math.round(positionPrecedente.latitude*1000);
  return `X:${x} Z:${z}`;
}

function calculerCapVersDestination(capActuel){
  if(!positionPrecedente||!destination.latitude||!destination.longitude) return "--";
  const φ1=positionPrecedente.latitude*Math.PI/180;
  const φ2=destination.latitude*Math.PI/180;
  const Δλ=(destination.longitude-positionPrecedente.longitude)*Math.PI/180;
  const y=Math.sin(Δλ)*Math.cos(φ2);
  const x=Math.cos(φ1)*Math.sin(φ2)-Math.sin(φ1)*Math.cos(φ2)*Math.cos(Δλ);
  const θ=Math.atan2(y,x);
  const capDest=(θ*180/Math.PI+360)%360;
  const delta=Math.abs(capActuel-capDest);
  return `${capDest.toFixed(0)}° (${delta.toFixed(0)}° écart)`;
}

export function definirDestination(lat,lon){ destination.latitude=lat; destination.longitude=lon; }

/* ========================
   HORLOGE & MÉDAILLON
======================== */
function activerHorlogeMinecraft(){
  const horloge=document.getElementById("horloge");
  function maj(){ 
    const d=new Date(),h=String(d.getHours()).padStart(2,"0"),
          m=String(d.getMinutes()).padStart(2,"0"),
          s=String(d.getSeconds()).padStart(2,"0");
    horloge.textContent=`⏰ Horloge Minecraft\n${h}:${m}:${s}`;
    requestAnimationFrame(maj);
  }
  maj();
}

function afficherMedaillon(){
  const canvas=document.createElement("canvas"); canvas.width=300; canvas.height=300;
  const ctx=canvas.getContext("2d");
  const medaillon=document.getElementById("medaillon");
  medaillon.innerHTML=""; medaillon.appendChild(canvas);
  ctx.fillStyle="#000"; ctx.fillRect(0,0,canvas.width,canvas.height);
  const cx=canvas.width/2, cy=canvas.height/2, r=140;
  ctx.strokeStyle="#444"; ctx.lineWidth=2; ctx.beginPath(); ctx.arc(cx,cy,r,0,2*Math.PI); ctx.stroke();
  ctx.fillStyle="#fff"; ctx.font="10px monospace"; ctx.fillText("Zénith",cx-20,cy-r+10); ctx.fillText("0°",cx-10,cy+4);
  // Soleil & Lune
  ctx.fillStyle="#ffd700"; ctx.beginPath(); ctx.arc(cx+60,cy-60,8,0,2*Math.PI); ctx.fill(); ctx.fillText("☀️ Soleil",cx+50,cy-70);
  ctx.fillStyle="#ccc"; ctx.beginPath(); ctx.arc(cx-60,cy-40,6,0,2*Math.PI); ctx.fill(); ctx.fillText("🌙 Lune",cx-70,cy-50);
  // Constellations
  const constellations=[{name:"Orion",x:cx+20,y:cy+60},{name:"Cassiopeia",x:cx-30,y:cy+40},{name:"Scorpius",x:cx+70,y:cy+10}];
  ctx.fillStyle="#0ff"; constellations.forEach(c=>{ ctx.beginPath(); ctx.arc(c.x,c.y,2,0,2*Math.PI); ctx.fill(); ctx.fillText(c.name,c.x+5,c.y+5);});
  // Galaxie
  ctx.fillStyle="#f0f"; ctx.beginPath(); ctx.arc(cx,cy+100,3,0,2*Math.PI); ctx.fill(); ctx.fillText("🌌 Galaxie",cx-20,cy+110);
}

/* ========================
   MÉTÉO & QUALITÉ AIR
======================== */
function chargerMeteo(){
  if(!navigator.onLine){ document.getElementById("meteo").textContent="Pas de connexion"; return; }
  fetch("https://api.open-meteo.com/v1/forecast?latitude=43.3&longitude=5.4&current_weather=true").then(r=>r.json()).then(data=>{
    const m=data.current_weather;
    document.getElementById("meteo").textContent=`Temp : ${m.temperature} °C | Vent : ${m.windspeed} km/h | Pression : ${m.pressure??"--"} hPa | Humidité : ${m.humidity??"--"}%`;
  }).catch(()=>{ document.getElementById("meteo").textContent="Météo indisponible"; });

  fetch("https://api.openaq.org/v2/latest?coordinates=43.3,5.4").then(r=>r.json()).then(data=>{
    const air=data.results[0]?.measurements??[];
    const pm25=air.find(m=>m.parameter==="pm25")?.value??"--";
    const uv=air.find(m=>m.parameter==="uv")?.value??"--";
    document.getElementById("qualite-air").textContent=`Qualité air : PM2.5 ${pm25} µg/m³ | UV : ${uv}`;
  }).catch(()=>{ document.getElementById("qualite-air").textContent="Qualité air indisponible"; });
}

/* ========================
   GRANDEURS SCIENTIFIQUES
======================== */
function afficherGrandeurs(){
  const g=9.81, vs=343, vl=299792458, eb=100;
  document.getElementById("grandeurs").textContent=`Point ébullition : ${eb} °C | Gravité : ${g} m/s² | Vitesse son : ${vs} m/s | Vitesse lumière : ${vl.toExponential(2)} m/s`;
}

/* ========================
   INIT DOM
======================== */
document.addEventListener("DOMContentLoaded",()=>{
  const boutonMarche=document.getElementById("marche");
  let actif=false;
  boutonMarche.addEventListener("click",()=>{
    actif=!actif;
    if(actif){
      demarrerCockpit();
      activerCapteurs();
      activerBoussole();
      activerHorlogeMinecraft();
      afficherMedaillon();
      chargerMeteo();
      afficherGrandeurs();
      boutonMarche.textContent="⏹️ Arrêter";
    }else{
      arreterCockpit();
      boutonMarche.textContent="▶️ Marche";
      ["vitesse","distance","pourcentage","gps","capteurs","boussole","horloge","medaillon","meteo","qualite-air","grandeurs"].forEach(id=>{
        document.getElementById(id).textContent="--";
      });
    }
  });
});
     
