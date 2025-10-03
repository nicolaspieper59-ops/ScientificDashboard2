// === Librairie astro ===
import * as SunCalc from "https://cdn.jsdelivr.net/npm/suncalc@1.9.0/suncalc.js";

let watchId = null, positionPrecedente = null;
let vitesses = [], vitesseMax = 0, distanceTotale = 0, t0 = null;

const applicateurs = { physique: true, chimie: true, svt: true };
let rituelActif = true;

// ===== Utils =====
const set = (id, txt) => {
  const el = document.getElementById(id);
  if (el) el.textContent = txt;
};
const saveLocal = (key, data) => localStorage.setItem(key, JSON.stringify(data));
const loadLocal = (key) => JSON.parse(localStorage.getItem(key) || "null");

// ===== GPS initial =====
navigator.geolocation.getCurrentPosition(
  pos => traiterPosition(pos.coords, pos.timestamp),
  err => console.error("Erreur GPS:", err),
  { enableHighAccuracy: true, timeout: 10000 }
);

// ===== Boutons =====
document.getElementById("marche").onclick = () => {
  if (watchId !== null) return;
  t0 = performance.now();
  watchId = navigator.geolocation.watchPosition(
    pos => traiterPosition(pos.coords, pos.timestamp),
    err => console.error("Erreur GPS:", err),
    { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 }
  );
  loopTemps();
  activerHorloge();
  synchroniserHeureAtomique();
  activerCapteurs();
  chargerMeteo();
};

document.getElementById("stop").onclick = () => {
  if (watchId !== null) {
    navigator.geolocation.clearWatch(watchId);
    watchId = null;
    set("gps", "📡 Suivi arrêté");
  }
};

document.getElementById("reset").onclick = () => {
  vitesses = []; vitesseMax = 0; distanceTotale = 0; positionPrecedente = null;
  ["vitesse","vitesse-moy","vitesse-max","vitesse-ms","pourcentage","distance","distance-cosmique","gps"].forEach(id => set(id, "--"));
};

// ===== Chrono =====
function loopTemps() {
  const el = document.getElementById("temps");
  function tick() {
    if (!t0) return;
    const t = performance.now() - t0;
    el.textContent = `Temps : ${(t/1000).toFixed(2)} s`;
    requestAnimationFrame(tick);
  }
  tick();
}

// ===== Traitement GPS =====
function traiterPosition(coords, timestamp) {
  const v = coords.speed != null ? coords.speed * 3.6 : calculerVitesse(coords, timestamp);
  if (v >= 0 && v < 1000) {
    vitesseMax = Math.max(vitesseMax, v);
    vitesses.push(v);
    const moy = vitesses.reduce((a,b)=>a+b,0)/vitesses.length;
    const mps = v/3.6;
    const mmps = mps*1000;
    const dkm = distanceTotale/1000;
    const ds = distanceTotale/299792458;
    const dal = ds/(3600*24*365.25);

    set("vitesse", `Vitesse instantanée : ${v.toFixed(2)} km/h`);
    set("vitesse-moy", `Vitesse moyenne : ${moy.toFixed(2)} km/h`);
    set("vitesse-max", `Vitesse max : ${vitesseMax.toFixed(2)} km/h`);
    set("vitesse-ms", `Vitesse : ${mps.toFixed(2)} m/s | ${mmps.toFixed(0)} mm/s`);
    set("pourcentage", `% Lumière : ${(mps/299792458*100).toExponential(2)}% | % Son : ${(mps/343*100).toFixed(2)}%`);
    set("distance", `Distance : ${dkm.toFixed(3)} km | ${distanceTotale.toFixed(1)} m | ${(distanceTotale*1000).toFixed(0)} mm`);
    set("distance-cosmique", `Distance cosmique : ${ds.toFixed(6)} s lumière | ${dal.toExponential(3)} al`);
    set("gps", `Précision GPS : ${(coords.accuracy ?? 0).toFixed(0)} m`);

    positionPrecedente = { ...coords, timestamp };

    afficherMedaillon(coords.latitude, coords.longitude);
    chargerGrandeurs();
    saveLocal("lastGPS", coords);
  }
}

function calculerVitesse(c, t) {
  if (!positionPrecedente) return 0;
  const dt = (t - positionPrecedente.timestamp)/1000;
  const d = calculerDistance(c, positionPrecedente);
  distanceTotale += d;
  return dt>0 ? (d/dt)*3.6 : 0;
}
function calculerDistance(a,b) {
  const R=6371e3;
  const φ1=a.latitude*Math.PI/180, φ2=b.latitude*Math.PI/180;
  const Δφ=(b.latitude-a.latitude)*Math.PI/180;
  const Δλ=(b.longitude-a.longitude)*Math.PI/180;
  const A=Math.sin(Δφ/2)**2 + Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2;
  const C=2*Math.atan2(Math.sqrt(A),Math.sqrt(1-A));
  return R*C;
}

// ===== Soleil & Lune =====
function afficherMedaillon(lat,lon) {
  const now=new Date();
  const soleil=SunCalc.getTimes(now,lat,lon);
  const lune=SunCalc.getMoonTimes(now,lat,lon);
  const phase=SunCalc.getMoonIllumination(now);

  set("culmination-soleil", `☀️ Culmination : ${soleil.solarNoon.toLocaleTimeString()}`);
  set("heure-vraie", `Heure solaire vraie : ${soleil.sunrise.toLocaleTimeString()}`);
  set("heure-moyenne", `Heure solaire moyenne : ${soleil.sunset.toLocaleTimeString()}`);
  set("equation-temps", `Équation du temps : ${(soleil.sunset-soleil.sunrise)/60000-720} min`);
  set("lune-phase", `🌗 Phase : ${(phase.fraction*100).toFixed(1)}%`);
  set("lune-lever", `🌙 Lever : ${lune.rise?.toLocaleTimeString()??"--"}`);
  set("lune-coucher", `🌙 Coucher : ${lune.set?.toLocaleTimeString()??"--"}`);
}

// ===== Horloges =====
function activerHorloge() {
  const h=document.getElementById("horloge-minecraft");
  function tick() {
    const d=new Date();
    h.textContent=`${String(d.getHours()).padStart(2,"0")}:${String(d.getMinutes()).padStart(2,"0")}:${String(d.getSeconds()).padStart(2,"0")}`;
    requestAnimationFrame(tick);
  }
  tick();
}
function synchroniserHeureAtomique() {
  fetch("https://worldtimeapi.org/api/ip").then(r=>r.json()).then(data=>{
    const utc=new Date(data.utc_datetime);
    set("horloge-atomique", `Heure atomique (UTC) : ${utc.toLocaleTimeString()}`);
  }).catch(()=>set("horloge-atomique","Heure atomique indisponible"));
}

// ===== Capteurs =====
function activerCapteurs() {
  // Lumière
  if ("AmbientLightSensor" in window) {
    try {
      const light=new AmbientLightSensor();
      light.onreading=()=>set("capteurs", `💡 Lumière : ${Math.round(light.illuminance)} lux`);
      light.start();
    } catch(e){console.warn("Capteur lumière non dispo");}
  }
  // Son
  if (navigator.mediaDevices?.getUserMedia) {
    navigator.mediaDevices.getUserMedia({audio:true}).then(stream=>{
      const ctx=new AudioContext();
      const analyser=ctx.createAnalyser();
      const src=ctx.createMediaStreamSource(stream);
      src.connect(analyser);
      const data=new Uint8Array(analyser.frequencyBinCount);
      (function loop(){
        analyser.getByteFrequencyData(data);
        const dB=20*Math.log10(data.reduce((a,b)=>a+b,0)/data.length||1);
        set("capteurs", `🎤 Son : ${Math.round(dB)} dB`);
        requestAnimationFrame(loop);
      })();
    });
  }
  // Orientation
  if ("DeviceOrientationEvent" in window) {
    window.addEventListener("deviceorientation", e=>{
      set("capteurs", `📐 Inclinaison : ${e.beta?.toFixed(1)}°`);
    });
  }
}

// ===== Météo =====
async function chargerMeteo() {
  const coords=loadLocal("lastGPS") || {latitude:43.3, longitude:5.4};
  try {
    const meteoRes=await fetch(`https://api.open-meteo.com/v1/forecast?latitude=${coords.latitude}&longitude=${coords.longitude}&current_weather=true`);
    const meteoData=await meteoRes.json();
    const w=meteoData.current_weather;
    set("meteo", `Température : ${w.temperature}°C | Vent : ${w.windspeed} km/h | Code météo : ${w.weathercode}`);
    saveLocal("meteo",w);
  } catch {
    const w=loadLocal("meteo");
    if(w) set("meteo", `(Offline) Température : ${w.temperature}°C | Vent : ${w.windspeed} km/h`);
  }
  try {
    const airRes=await fetch(`https://api.openaq.org/v2/latest?coordinates=${coords.latitude},${coords.longitude}`);
    const d=await airRes.json();
    const m=d.results[0]?.measurements??[];
    const pm25=m.find(a=>a.parameter==="pm25")?.value??"--";
    set("air", `Qualité air : ${pm25} µg/m³`);
    saveLocal("air",pm25);
  } catch {
    const pm25=loadLocal("air")??"--";
    set("air", `(Offline) Qualité air : ${pm25} µg/m³`);
  }
}

// ===== Grandeurs scientifiques =====
function chargerGrandeurs() {
  const masse=70;
  const v=vitesses.length?vitesses.at(-1)/3.6:0;
  const g=9.81;
  let texte="";
  if(applicateurs.physique) {
    const F=masse*g;
    const E=0.5*masse*v**2;
    const P=F*v;
    texte+=`🧲 Force : ${F.toFixed(1)} N | 🔋 Énergie : ${E.toFixed(1)} J | ⚡ Puissance : ${P.toFixed(1)} W\n`;
  }
  if(applicateurs.chimie) {
    const R=8.314, T=298;
    const Vm=(R*T/101325).toFixed(3);
    texte+=`🧪 Volume molaire : ${Vm} m³/mol | Avogadro : 6.022×10²³\n`;
  }
  if(applicateurs.svt) {
    const bpm=Math.round(60+v*2);
    const temp=36.5+v*0.02;
    texte+=`🫀 Fréquence cardiaque : ${bpm} bpm | 🌡️ Température : ${temp.toFixed(1)}°C\n`;
  }
  set("grandeurs",texte);
  }
  
