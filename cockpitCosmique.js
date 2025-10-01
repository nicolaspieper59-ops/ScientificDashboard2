// === Variables globales ===
let watchId = null;
let positionPrecedente = null;
let vitesseMax = 0;
let vitesses = [];
let distanceTotale = 0;
let rituelActif = true;
let destination = { latitude: null, longitude: null };
let intensitesSon = [];
let intensitesLumiere = [];

// === Gestion rituel cosmique ===
export function toggleRituel() {
  rituelActif = !rituelActif;
  document.body.classList.toggle('rituel-off', !rituelActif);
  const btn = document.getElementById('toggle-rituel');
  btn.textContent = rituelActif ? '✨ Rituel cosmique : ON' : '🔋 Rituel cosmique : OFF';
}

// === Cockpit GPS / vitesse / distance ===
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
        altitude: pos.coords.altitude ?? 0,
        accuracy: pos.coords.accuracy,
        timestamp: pos.timestamp,
        speed: pos.coords.speed
      };
      const vitesse = gps.speed !== null ? gps.speed*3.6 : calculerVitesse(gps);
      if(vitesse >=0 && vitesse<300){
        vitesseMax = Math.max(vitesseMax, vitesse);
        vitesses.push(vitesse);
        afficherVitesse(vitesse);
        afficherDistance();
        afficherPourcentage(vitesse);
        afficherGPS(gps);
      }
    },
    err => { console.error("Erreur GPS :", err); document.getElementById("gps").textContent = "Erreur GPS : "+err.message; },
    { enableHighAccuracy:true, maximumAge:0, timeout:10000 }
  );
}

export function arreterCockpit() {
  if(watchId!==null){
    navigator.geolocation.clearWatch(watchId);
    watchId=null;
    positionPrecedente=null;
    vitesses=[];
    distanceTotale=0;
  }
}

export function resetVitesseMax() { vitesseMax=0; }
export function definirDestination(lat,lon){ destination.latitude=lat; destination.longitude=lon; }

// === Calculs vitesse / distance ===
function calculerVitesse(gps){
  if(!positionPrecedente){ positionPrecedente=gps; return 0; }
  const dt=(gps.timestamp-positionPrecedente.timestamp)/1000;
  const d=calculerDistance(gps,positionPrecedente);
  distanceTotale+=d;
  positionPrecedente=gps;
  return dt>0?(d/dt)*3.6:0;
}

function calculerDistance(p1,p2){
  const R=6371e3;
  const φ1=p1.latitude*Math.PI/180;
  const φ2=p2.latitude*Math.PI/180;
  const Δφ=(p2.latitude-p1.latitude)*Math.PI/180;
  const Δλ=(p2.longitude-p1.longitude)*Math.PI/180;
  const a=Math.sin(Δφ/2)**2 + Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2;
  const c=2*Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
  return R*c;
}

// === Affichage cockpit ===
function afficherVitesse(v){
  const moy = vitesses.length ? vitesses.reduce((a,b)=>a+b,0)/vitesses.length : 0;
  const mps=v/3.6;
  const mmps=mps*1000;
  document.getElementById('vitesse').textContent=
    `Temps : ${new Date().toLocaleTimeString()} | Vitesse : ${v.toFixed(2)} km/h | Moy : ${moy.toFixed(2)} km/h | Max : ${vitesseMax.toFixed(2)} km/h | ${mps.toFixed(2)} m/s | ${mmps.toFixed(0)} mm/s`;
}

function afficherDistance(){
  const km=distanceTotale/1000, m=distanceTotale, mm=m*1000;
  const secL=m/299792458;
  const anL=secL/(3600*24*365.25);
  document.getElementById('distance').textContent=
    `Distance : ${km.toFixed(3)} km | ${m.toFixed(0)} m | ${mm.toFixed(0)} mm | Cosmique : ${secL.toFixed(3)} s lumière | ${anL.toExponential(3)} al`;
}

function afficherPourcentage(v){
  const mps=v/3.6;
  const pctL=(mps/299792458)*100;
  const pctS=(mps/343)*100;
  document.getElementById('pourcentage').textContent=`% Lumière : ${pctL.toExponential(2)}% | % Son : ${pctS.toFixed(2)}%`;
}

function afficherGPS(gps){
  document.getElementById('gps').textContent=
    `Latitude : ${gps.latitude.toFixed(6)} | Longitude : ${gps.longitude.toFixed(6)} | Altitude : ${gps.altitude.toFixed(1)} m | Précision GPS : ${gps.accuracy.toFixed(0)}%`;
}

// === Capteurs physiques ===
export function activerCapteurs(){
  // Lumière ambiante
  if("AmbientLightSensor" in window){
    try{
      const light=new AmbientLightSensor();
      light.addEventListener("reading",()=>{ intensitesLumiere.push(light.illuminance);
        const moy=intensitesLumiere.reduce((a,b)=>a+b,0)/intensitesLumiere.length;
        const max=Math.max(...intensitesLumiere);
        document.getElementById("capteurs").textContent=`Lumière : ${light.illuminance.toFixed(0)} lux | Moy : ${moy.toFixed(0)} | Max : ${max.toFixed(0)}`;
      });
      light.start();
    }catch(e){ console.warn("Capteur lumière indisponible",e);}
  }
  // Microphone
  if(navigator.mediaDevices?.getUserMedia){
    navigator.mediaDevices.getUserMedia({audio:true}).then(stream=>{
      const ctx=new AudioContext();
      const analyser=ctx.createAnalyser();
      const source=ctx.createMediaStreamSource(stream);
      source.connect(analyser);
      const data=new Uint8Array(analyser.frequencyBinCount);
      function analyserSon(){
        analyser.getByteFrequencyData(data);
        const moy=data.reduce((a,b)=>a+b,0)/data.length;
        const dB=20*Math.log10(moy||1);
        intensitesSon.push(dB);
        const max=Math.max(...intensitesSon);
        document.getElementById("capteurs").textContent+=` | Son : ${dB.toFixed(0)} dB`;
        requestAnimationFrame(analyserSon);
      }
      analyserSon();
    }).catch(e=>console.warn("Microphone indisponible",e));
  }
  // Orientation / magnétomètre
  if("DeviceOrientationEvent" in window){
    window.addEventListener("deviceorientation",e=>{
      if(e.beta!=null){
        document.getElementById("capteurs").textContent+=` | Inclinaison : ${e.beta.toFixed(1)}°`;
      }
    });
  }
  if("Magnetometer" in window){
    try{
      const mag=new Magnetometer();
      mag.addEventListener("reading",()=>{
        const champ=Math.sqrt(mag.x**2+mag.y**2+mag.z**2);
        document.getElementById("capteurs").textContent+=` | Magnétisme : ${champ.toFixed(0)} µT`;
      });
      mag.start();
    }catch(e){ console.warn("Magnétomètre indisponible",e);}
  }
}

// === Boussole & destination ===
export function activerBoussole(){
  if("DeviceOrientationEvent" in window){
    const evName="deviceorientationabsolute" in window?"deviceorientationabsolute":"deviceorientation";
    window.addEventListener(evName,e=>{
      if(e.alpha!=null){
        const cap=e.alpha;
        const capDest=calculerCapVersDestination(cap);
        document.getElementById("boussole").textContent=`Cap : ${cap.toFixed(0)}° | Cap vers destination : ${capDest}`;
      }
    });
  }
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
  return `${capDest.toFixed(0)}° (${delta.toFixed(0)}° d'écart)`;
}

// === Horloge cosmique ===
export function activerHorloge(){
  const horloge=document.getElementById("horloge");
  function miseAJour(){
    const t=new Date();
    horloge.textContent=`⏰ ${t.getHours().toString().padStart(2,'0')}:${t.getMinutes().toString().padStart(2,'0')}:${t.getSeconds().toString().padStart(2,'0')}`;
    requestAnimationFrame(miseAJour);
  }
  miseAJour();
}

// === Médaillon céleste ===
export function afficherMedaillon(){
  const canvas=document.getElementById("medaillon");
  const ctx=canvas.getContext("2d");
  const cx=canvas.width/2, cy=canvas.height/2, r=140;
  ctx.fillStyle="#000"; ctx.fillRect(0,0,canvas.width,canvas.height);
  ctx.strokeStyle="#0ff"; ctx.lineWidth=2;
  ctx.beginPath(); ctx.arc(cx,cy,r,0,2*Math.PI); ctx.stroke();
  ctx.fillStyle="#fff"; ctx.font="10px monospace"; ctx.fillText("Zénith", cx-20, cy-r+10);
  // Soleil
  ctx.fillStyle="#ffd700"; ctx.beginPath(); ctx.arc(cx+60,cy-60,8,0,2*Math.PI); ctx.fill(); ctx.fillText("☀️ Soleil", cx+50,cy-70);
  // Lune
  ctx.fillStyle="#ccc"; ctx.beginPath(); ctx.arc(cx-60,cy-40,6,0,2*Math.PI); ctx.fill(); ctx.fillText("🌙 Lune", cx-70,cy-50);
}

// === Météo & qualité de l'air ===
export function chargerMeteo(){
  if(!navigator.onLine){ document.getElementById("meteo").textContent="Pas de connexion Internet"; return; }
  fetch("https://api.open-meteo.com/v1/forecast?latitude=43.3&longitude=5.4&current_weather=true")
    .then(r=>r.json())
    .then(data=>{
      const m=data.current_weather;
      document.getElementById("meteo").textContent=`Température : ${m.temperature} °C | Vent : ${m.windspeed} km/h`;
    }).catch(()=>{document.getElementById("meteo").textContent="Météo indisponible";});
  fetch("https://api.openaq.org/v2/latest?coordinates=43.3,5.4")
    .then(r=>r.json())
    .then(data=>{
      const air=data.results[0]?.measurements??[];
      const pm25=air.find(x=>x.parameter==="pm25")?.value??"--";
      const uv=air.find(x=>x.parameter==="uv")?.value??"--";
      document.getElementById("qualite-air").textContent=`Qualité air : PM2.5 ${pm25} µg/m³ | Indice UV : ${uv}`;
    }).catch(()=>{document.getElementById("qualite-air").textContent="Qualité air indisponible";});
}

// === Grandeurs scientifiques ===
export function afficherGrandeurs(){
  const ptEbull=100, g=9.81, vSon=343, vLum=299792458;
  document.getElementById("grandeurs").textContent=`Point d’ébullition : ${ptEbull} °C | Gravité : ${g} m/s² | Vitesse son : ${vSon} m/s | Vitesse lumière : ${vLum.toExponential(2)} m/s`;
}
  
