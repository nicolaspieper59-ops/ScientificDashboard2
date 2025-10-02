import SunCalc from 'https://cdn.jsdelivr.net/npm/suncalc@1.9.0/suncalc.js';

let watchId = null;
let positionPrecedente = null;
let vitesseMax = 0;
let vitesses = [];
let distanceTotale = 0;
let destination = { latitude: null, longitude: null };
let masseSimulee = 70; // kg

/* =========================
   GPS et Vitesse
========================= */
export function demarrerCockpit() {
  if (!('geolocation' in navigator)) {
    safeSetText('gps', 'GPS non disponible');
    return;
  }

  watchId = navigator.geolocation.watchPosition(
    pos => traiterPosition(pos.coords, pos.timestamp),
    err => safeSetText('gps', 'Erreur GPS : ' + err.message),
    { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 }
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

export function resetVitesseMax() {
  vitesseMax = 0;
  appendText('vitesse', ' | Max réinitialisé');
}

export function definirDestination(lat, lon) {
  destination.latitude = lat;
  destination.longitude = lon;
}

/* =========================
   Calculs GPS
========================= */
function traiterPosition(coords, timestamp) {
  const gps = {
    latitude: coords.latitude,
    longitude: coords.longitude,
    altitude: coords.altitude ?? 0,
    accuracy: coords.accuracy,
    speed: coords.speed ?? 0,
    timestamp
  };

  const vitesse = gps.speed >= 0 ? gps.speed * 3.6 : calculerVitesse(gps);

  if (vitesse >= 0 && vitesse < 300) {
    vitesseMax = Math.max(vitesseMax, vitesse);
    vitesses.push(vitesse);

    afficherVitesse(vitesse);
    afficherDistance();
    afficherPourcentage(vitesse);
    afficherGPS(gps);
    afficherAstronomie(gps.latitude, gps.longitude);
  }
}

function calculerVitesse(gps) {
  if (!positionPrecedente) {
    positionPrecedente = gps;
    return 0;
  }
  const dt = (gps.timestamp - positionPrecedente.timestamp) / 1000;
  const d = calculerDistance(gps, positionPrecedente);
  distanceTotale += d;
  positionPrecedente = gps;
  return dt > 0 ? (d / dt) * 3.6 : 0;
}

function calculerDistance(p1, p2) {
  const R = 6371e3;
  const φ1 = p1.latitude * Math.PI / 180;
  const φ2 = p2.latitude * Math.PI / 180;
  const Δφ = (p2.latitude - p1.latitude) * Math.PI / 180;
  const Δλ = (p2.longitude - p1.longitude) * Math.PI / 180;

  const a = Math.sin(Δφ/2)**2 + Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2;
  const c = 2*Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
  return R * c;
}

/* =========================
   Affichage
========================= */
function afficherVitesse(v) {
  const moyenne = vitesses.length ? vitesses.reduce((a,b)=>a+b,0)/vitesses.length : 0;
  safeSetText('vitesse', 
    `Vitesse instantanée : ${v.toFixed(2)} km/h | Moyenne : ${moyenne.toFixed(2)} | Max : ${vitesseMax.toFixed(2)}`
  );
}

function afficherDistance() {
  const km = distanceTotale/1000;
  safeSetText('distance', `Distance : ${km.toFixed(3)} km | Cosmique : ${(distanceTotale/299792458).toFixed(3)} s lumière`);
}

function afficherPourcentage(v) {
  const mps = v/3.6;
  const pctL = (mps/299792458)*100;
  const pctS = (mps/343)*100;
  safeSetText('pourcentage', `% Lumière : ${pctL.toExponential(2)}% | % Son : ${pctS.toFixed(2)}%`);
}

function afficherGPS(g) {
  safeSetText('gps', `Lat : ${g.latitude.toFixed(6)} | Lon : ${g.longitude.toFixed(6)} | Alt : ${g.altitude.toFixed(1)} m | Précision : ${g.accuracy.toFixed(1)}%`);
}

/* =========================
   Astronomie
========================= */
function afficherAstronomie(lat, lon) {
  const now = new Date();
  const sunTimes = SunCalc.getTimes(now, lat, lon);
  const sunPos = SunCalc.getPosition(now, lat, lon);
  const moonPos = SunCalc.getMoonPosition(now, lat, lon);
  const moonIll = SunCalc.getMoonIllumination(now);

  safeSetText('medaillon', 
    `☀️ Soleil : Lever ${sunTimes.sunrise.toLocaleTimeString()} | Coucher ${sunTimes.sunset.toLocaleTimeString()} | Culmination ${sunTimes.solarNoon.toLocaleTimeString()}
🌙 Lune : Phase ${(moonIll.fraction*100).toFixed(1)}% | Magnitude ${moonPos.altitude.toFixed(2)} | Lever : ${SunCalc.getMoonTimes(now, lat, lon).rise?.toLocaleTimeString()||'--'} | Coucher : ${SunCalc.getMoonTimes(now, lat, lon).set?.toLocaleTimeString()||'--'}
Équation du temps : ${(sunPos.altitude - (now.getHours() + now.getMinutes()/60)).toFixed(2)} h`);
}

/* =========================
   Capteurs physiques
========================= */
function activerCapteurs() {
  const el = document.getElementById('capteurs');
  el.textContent = 'Chargement capteurs...';

  // Lumière
  if ('AmbientLightSensor' in window) {
    try {
      const sensor = new AmbientLightSensor();
      sensor.addEventListener('reading', () => el.textContent = `Lumière : ${Math.round(sensor.illuminance)} lux`);
      sensor.start();
    } catch {}
  }

  // Microphone
  if (navigator.mediaDevices?.getUserMedia) {
    navigator.mediaDevices.getUserMedia({audio:true}).then(stream => {
      const ctx = new AudioContext();
      const analyser = ctx.createAnalyser();
      const src = ctx.createMediaStreamSource(stream);
      src.connect(analyser);
      const data = new Uint8Array(analyser.frequencyBinCount);
      (function loop(){
        analyser.getByteFrequencyData(data);
        const dB = 20*Math.log10(data.reduce((a,b)=>a+b,0)/data.length||1);
        el.textContent += ` | Son : ${dB.toFixed(0)} dB | Fréquence : ${analyser.frequencyBinCount} Hz`;
        requestAnimationFrame(loop);
      })();
    }).catch(()=>{});
  }

  // Orientation (niveau à bulle)
  if ('DeviceOrientationEvent' in window) {
    window.addEventListener('deviceorientation', e => {
      if(e.beta!=null) el.textContent += ` | Niveau : ${e.beta.toFixed(1)}°`;
    });
  }

  // Magnétomètre
  if ('Magnetometer' in window) {
    try {
      const mag = new Magnetometer();
      mag.addEventListener('reading', () => {
        const champ = Math.sqrt(mag.x**2+mag.y**2+mag.z**2);
        el.textContent += ` | Magnétisme : ${Math.round(champ)} µT`;
      });
      mag.start();
    } catch {}
  }
}

/* =========================
   Boussole
========================= */
function activerBoussole() {
  if ('DeviceOrientationEvent' in window) {
    window.addEventListener('deviceorientationabsolute', e => {
      if(e.alpha!=null){
        const cap = e.alpha.toFixed(0);
        const minecraft = positionPrecedente ? `X:${Math.round(positionPrecedente.longitude*1000)} Z:${Math.round(positionPrecedente.latitude*1000)} Y:${Math.round(positionPrecedente.altitude??0)}` : '--';
        safeSetText('boussole', `Cap : ${cap}° | Coordonnées Minecraft : ${minecraft} | Cap vers destination : ${destination.latitude?destination.latitude+'°':''}`);
      }
    });
  }
}

/* =========================
   Horloge
========================= */
function activerHorloge() {
  const h = document.getElementById('horloge');
  (function loop(){
    const d = new Date();
    h.textContent = `${String(d.getHours()).padStart(2,'0')}:${String(d.getMinutes()).padStart(2,'0')}:${String(d.getSeconds()).padStart(2,'0')}`;
    requestAnimationFrame(loop);
  })();
}

/* =========================
   Médaillon
========================= */
function afficherMedaillon() {
  const med = document.getElementById('medaillon');
  med.innerHTML = '';
  const c = document.createElement('canvas');
  c.width = 300; c.height = 300;
  med.appendChild(c);
  const ctx = c.getContext('2d');

  ctx.fillStyle = '#000'; ctx.fillRect(0,0,300,300);
  ctx.strokeStyle = '#0ff'; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.arc(150,150,140,0,2*Math.PI); ctx.stroke();

  ctx.fillStyle = '#ffd700'; ctx.beginPath(); ctx.arc(210,90,8,0,2*Math.PI); ctx.fill(); ctx.fillText("☀️ Soleil",200,80);
  ctx.fillStyle = '#ccc'; ctx.beginPath(); ctx.arc(90,110,6,0,2*Math.PI); ctx.fill(); ctx.fillText("🌙 Lune",70,100);

  const stars=[{name:"Orion",x:170,y:210},{name:"Cassiopeia",x:120,y:190},{name:"Scorpius",x:220,y:180}];
  ctx.fillStyle="#0ff"; stars.forEach(s=>{ctx.beginPath();ctx.arc(s.x,s.y,2,0,2*Math.PI);ctx.fill(); ctx.fillText(s.name,s.x+5,s.y+5);});
  ctx.fillStyle="#f0f"; ctx.beginPath();ctx.arc(150,250,3,0,2*Math.PI);ctx.fill();ctx.fillText("🌌 Galaxie",130,260);
}

/* =========================
   Météo
========================= */
function chargerMeteo() {
  const meteoEl = document.getElementById('meteo');
  const airEl = document.getElementById('qualite-air');
  if(!navigator.onLine){meteoEl.textContent="Pas de connexion";airEl.textContent="Indisponible";return;}

  fetch("https://api.open-meteo.com/v1/forecast?latitude=43.3&longitude=5.4&current_weather=true")
    .then(r=>r.json()).then(data=>{
      const w=data.current_weather;
      meteoEl.textContent=`Temp : ${w.temperature}°C | Vent : ${w.windspeed} km/h`;
    }).catch(()=>{meteoEl.textContent="Météo indisponible";});

  fetch("https://api.openaq.org/v2/latest?coordinates=43.3,5.4")
    .then(r=>r.json()).then(d=>{
      const m=d.results[0]?.measurements??[];
      const pm25=m.find(a=>a.parameter==="pm25")?.value??"--";
      const uv=m.find(a=>a.parameter==="uv")?.value??"--";
      airEl.textContent=`Qualité air : PM2.5 ${pm25} µg/m³ | Indice UV : ${uv}`;
    }).catch(()=>{airEl.textContent="Indisponible";});
}

/* =========================
   Grandeurs scientifiques
========================= */
function afficherGrandeurs() {
  // Force = m*g ; Énergie = m*g*h ; Énergie cinétique = 0.5*m*v²
  const g = 9.81;
  const h = 10; // mètre simulé
  const v = vitesses.length? vitesses[vitesses.length-1]/3.6:0; // m/s
  const F = masseSimulee*g;
  const E_pot = masseSimulee*g*h;
  const E_cin = 0.5*masseSimulee*v*v;

  safeSetText('grandeurs',
    `Force : ${F.toFixed(2)} N | Énergie : ${E_pot.toFixed(2)} J | Cinétique : ${E_cin.toFixed(2)} J`
  );
}

/* =========================
   Utils
========================= */
function safeSetText(id, text){document.getElementById(id).textContent=text;}
function appendText(id, text){document.getElementById(id).textContent += text;}

/* =========================
   Initialisation
========================= */
document.getElementById('marche').addEventListener('click',()=>{
  if(watchId) {arreterCockpit();document.getElementById('marche').textContent='▶️ Démarrer';}
  else {demarrerCockpit();document.getElementById('marche').textContent='⏹️ Arrêter';}
});
document.getElementById('reset').addEventListener('click',()=>resetVitesseMax());
