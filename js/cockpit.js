let watchId = null;
let positionPrecedente = null;
let vitesses = [];
let distanceTotale = 0;
let vitesseMax = 0;
let destination = { latitude: null, longitude: null };

/* ======================
   INIT DOM
====================== */
document.addEventListener("DOMContentLoaded", () => {
  const boutonMarche = document.getElementById("marche");
  const boutonReset = document.getElementById("reset");
  let actif = false;

  boutonMarche.addEventListener("click", () => {
    actif = !actif;
    if (actif) {
      boutonMarche.classList.add("actif");
      boutonMarche.textContent = "⏹️ Arrêter";
      demarrerCockpit();
    } else {
      boutonMarche.classList.remove("actif");
      boutonMarche.textContent = "▶️ Démarrer";
      arreterCockpit();
    }
  });

  boutonReset.addEventListener("click", () => {
    resetVitesseMax();
    document.getElementById("vitesse").textContent += " | Max réinitialisé";
  });
});

/* ======================
   GPS & Vitesse
====================== */
export function demarrerCockpit() {
  if (!("geolocation" in navigator)) return;

  if (watchId !== null) return;

  watchId = navigator.geolocation.watchPosition(pos => {
    const gps = pos.coords;
    const vitesse = (gps.speed != null) ? gps.speed * 3.6 : calculerVitesse(pos);
    if (vitesse >= 0 && vitesse < 300) {
      vitesseMax = Math.max(vitesseMax, vitesse);
      vitesses.push(vitesse);
      afficherVitesse(vitesse);
      afficherDistance();
      afficherPourcentage(vitesse);
      afficherGPS(gps);
    }
  }, err => {
    document.getElementById("gps").textContent = "Erreur GPS : " + (err.message || "inconnue");
  }, { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 });

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

export function resetVitesseMax() {
  vitesseMax = 0;
}

export function definirDestination(lat, lon) {
  destination.latitude = lat;
  destination.longitude = lon;
}

/* ======================
   Calculs
====================== */
function calculerVitesse(pos) {
  if (!positionPrecedente) {
    positionPrecedente = pos.coords;
    return 0;
  }
  const dt = (pos.timestamp - positionPrecedente.timestamp) / 1000;
  const d = calculerDistance(pos.coords, positionPrecedente);
  distanceTotale += d;
  positionPrecedente = pos.coords;
  return dt > 0 ? (d / dt) * 3.6 : 0;
}

function calculerDistance(a, b) {
  const R = 6371e3;
  const φ1 = a.latitude * Math.PI/180;
  const φ2 = b.latitude * Math.PI/180;
  const Δφ = (b.latitude - a.latitude) * Math.PI/180;
  const Δλ = (b.longitude - a.longitude) * Math.PI/180;
  const c = 2 * Math.atan2(Math.sqrt(Math.sin(Δφ/2)**2 + Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2),
                           Math.sqrt(1-(Math.sin(Δφ/2)**2 + Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2)));
  return R*c;
}

/* ======================
   Affichage GPS / Vitesse
====================== */
function afficherVitesse(v) {
  const moyenne = vitesses.length ? vitesses.reduce((a,b)=>a+b,0)/vitesses.length : 0;
  document.getElementById("vitesse").textContent =
    `Vitesse : ${v.toFixed(2)} km/h | Moyenne : ${moyenne.toFixed(2)} | Max : ${vitesseMax.toFixed(2)}`;
}

function afficherDistance() {
  document.getElementById("distance").textContent = `Distance : ${(distanceTotale/1000).toFixed(3)} km`;
}

function afficherPourcentage(v) {
  const mps = v/3.6;
  const pctL = (mps/299792458)*100;
  const pctS = (mps/343)*100;
  document.getElementById("pourcentage").textContent = `% Lumière : ${pctL.toExponential(2)}% | % Son : ${pctS.toFixed(2)}%`;
}

function afficherGPS(g) {
  document.getElementById("gps").textContent =
    `Latitude : ${g.latitude.toFixed(6)} | Longitude : ${g.longitude.toFixed(6)} | Altitude : ${(g.altitude??0).toFixed(1)} m`;
}

/* ======================
   Capteurs physiques
====================== */
let capteursBaseText = "Niveau à bulle : --° | Lumière : -- lux | Son : -- dB | Fréquence : -- Hz | Magnétisme : -- µT";
function activerCapteurs() {
  const el = document.getElementById("capteurs");
  el.textContent = capteursBaseText;

  if("AmbientLightSensor" in window){
    try{
      const light = new AmbientLightSensor();
      light.addEventListener("reading", ()=>{
        el.textContent = capteursBaseText.replace("-- lux", `${Math.round(light.illuminance)} lux`);
      });
      light.start();
    }catch{}
  }

  if(navigator.mediaDevices?.getUserMedia){
    navigator.mediaDevices.getUserMedia({audio:true}).then(stream=>{
      const audioCtx = new AudioContext();
      const analyser = audioCtx.createAnalyser();
      const source = audioCtx.createMediaStreamSource(stream);
      source.connect(analyser);
      analyser.fftSize = 2048;
      const data = new Uint8Array(analyser.frequencyBinCount);

      function loopAudio() {
        analyser.getByteFrequencyData(data);
        const dB = 20 * Math.log10(data.reduce((a,b)=>a+b,0)/data.length||1);
        el.textContent = capteursBaseText.replace("-- dB", `${Math.round(dB)} dB`).replace("-- Hz", `${analyser.frequencyBinCount} Hz`);
        requestAnimationFrame(loopAudio);
      }
      loopAudio();
    }).catch(()=>{});
  }

  if("DeviceOrientationEvent" in window){
    window.addEventListener("deviceorientation", e=>{
      if(e.beta!=null){
        el.textContent = capteursBaseText.replace("--°", `${e.beta.toFixed(1)}°`);
      }
    });
  }

  if("Magnetometer" in window){
    try{
      const mag = new Magnetometer();
      mag.addEventListener("reading", ()=>{
        const champ = Math.sqrt(mag.x**2+mag.y**2+mag.z**2);
        el.textContent = capteursBaseText.replace("-- µT", `${Math.round(champ)} µT`);
      });
      mag.start();
    }catch{}
  }
}

/* ======================
   Boussole
====================== */
function activerBoussole(){
  if("DeviceOrientationEvent" in window){
    window.addEventListener("deviceorientationabsolute", e=>{
      if(e.alpha!=null){
        const cap = e.alpha.toFixed(0);
        const minecraft = positionPrecedente?`X:${Math.round(positionPrecedente.longitude*1000)} Z:${Math.round(positionPrecedente.latitude*1000)}`:"--";
        const capDest = destination.latitude?`${cap}°`:"--";
        document.getElementById("boussole").textContent = `Cap : ${cap}° | Coordonnées Minecraft : ${minecraft} | Cap vers destination : ${capDest}`;
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
    h.textContent = `${String(d.getHours()).padStart(2,'0')}:${String(d.getMinutes()).padStart(2,'0')}:${String(d.getSeconds()).padStart(2,'0')}`;
    requestAnimationFrame(loop);
  })();
}

/* ======================
   Médaillon
====================== */
function afficherMedaillon(){
  const med = document.getElementById("medaillon");
  med.innerHTML = "";
  const c = document.createElement("canvas");
  c.width = 300; c.height = 300;
  med.appendChild(c);
  const ctx = c.getContext("2d");

  ctx.fillStyle = "#000"; ctx.fillRect(0,0,300,300);
  ctx.strokeStyle = "#0ff"; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.arc(150,150,140,0,2*Math.PI); ctx.stroke();

  ctx.fillStyle = "#ffd700"; ctx.beginPath(); ctx.arc(210,90,8,0,2*Math.PI); ctx.fill(); ctx.fillText("☀️ Soleil",200,80);
  ctx.fillStyle = "#ccc"; ctx.beginPath(); ctx.arc(90,110,6,0,2*Math.PI); ctx.fill(); ctx.fillText("🌙 Lune",70,100);

  const stars=[{name:"Orion",x:170,y:210},{name:"Cassiopeia",x:120,y:190},{name:"Scorpius",x:220,y:180}];
  ctx.fillStyle="#0ff"; stars.forEach(s=>{ctx.beginPath(); ctx.arc(s.x,s.y,2,0,2*Math.PI); ctx.fill(); ctx.fillText(s.name,s.x+5,s.y+5);});
  ctx.fillStyle="#f0f"; ctx.beginPath(); ctx.arc(150,250,3,0,2*Math.PI); ctx.fill(); ctx.fillText("🌌 Galaxie",130,260);
}

/* ======================
   Météo
====================== */
function chargerMeteo(){
  const meteoEl=document.getElementById("meteo");
  const airEl=document.getElementById("qualite-air");
  if(!navigator.onLine){meteoEl.textContent="Pas de connexion";airEl.textContent="Indisponible";return;}

  fetch("https://api.open-meteo.com/v1/forecast?latitude=43.3&longitude=5.4&current_weather=true")
    .then(r=>r.json()).then(data=>{
      const w=data.current_weather;
      meteoEl.textContent=`Temp : ${w.temperature ?? "--"} °C | Vent : ${w.windspeed ?? "--"} km/h`;
    }).catch(()=>{meteoEl.textContent="Météo indisponible";});

  fetch("https://api.openaq.org/v2/latest?coordinates=43.3,5.4")
    .then(r=>r.json()).then(d=>{
      const m=d.results[0]?.measurements ?? [];
      const pm25=m.find(a=>a.parameter==="pm25")?.value??"--";
      const uv=m.find(a=>a.parameter==="uv")?.value??"--";
      airEl.textContent=`Qualité air : PM2.5 ${pm25} µg/m³ | Indice UV : ${uv}`;
    }).catch(()=>{airEl.textContent="Indisponible";});
}

/* ======================
   Grandeurs scientifiques
====================== */
function afficherGrandeurs(){
  document.getElementById("grandeurs").textContent=
    `Point ébullition : 100°C | Gravité : 9.81 m/s² | Vitesse son : 343 m/s | Vitesse lumière : 299792458 m/s`;
}

// Exemple de destination par défaut
definirDestination(43.3, 5.4);
