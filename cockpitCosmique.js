// === Variables globales ===
let watchId = null;
let positionPrecedente = null;
let vitesseMax = 0;
let vitesses = [];
let distanceTotale = 0;
let rituelActif = true;
let destination = { latitude: null, longitude: null };
let intensitesSon = [], intensitesLumiere = [];

// === Mode rituel ===
document.getElementById('toggle-rituel')?.addEventListener('click', () => {
  rituelActif = !rituelActif;
  document.body.classList.toggle('rituel-off', !rituelActif);
  document.getElementById('toggle-rituel').textContent =
    rituelActif ? '✨ Rituel cosmique : ON' : '🔋 Rituel cosmique : OFF';
});

// === Démarrage / arrêt cockpit ===
export function demarrerCockpit() {
  if (watchId !== null) return;

  watchId = navigator.geolocation.watchPosition(pos => {
    const gps = {
      latitude: pos.coords.latitude,
      longitude: pos.coords.longitude,
      accuracy: pos.coords.accuracy,
      timestamp: pos.timestamp,
      speed: pos.coords.speed
    };

    const vitesse = gps.speed !== null ? gps.speed * 3.6 : calculerVitesse(gps);

    if (vitesse >= 0 && vitesse < 300) {
      vitesseMax = Math.max(vitesseMax, vitesse);
      vitesses.push(vitesse);
      afficherVitesse(vitesse);
      afficherDistance();
      afficherPourcentage(vitesse);
      afficherGPS(gps);
    }
  }, err => console.error("Erreur GPS :", err), {
    enableHighAccuracy: true,
    maximumAge: 0,
    timeout: 10000
  });
}

export function arreterCockpit() {
  if (watchId !== null) {
    navigator.geolocation.clearWatch(watchId);
    watchId = null;
    positionPrecedente = null;
    vitesses = [];
    distanceTotale = 0;
  }
}

export function resetVitesseMax() {
  vitesseMax = 0;
}

// === Calculs vitesse / distance ===
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

function calculerDistance(pos1, pos2) {
  const R = 6371e3;
  const φ1 = pos1.latitude * Math.PI / 180;
  const φ2 = pos2.latitude * Math.PI / 180;
  const Δφ = (pos2.latitude - pos1.latitude) * Math.PI / 180;
  const Δλ = (pos2.longitude - pos1.longitude) * Math.PI / 180;
  const a = Math.sin(Δφ/2)**2 + Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2;
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
  return R * c;
}

// === Affichage cockpit ===
function afficherVitesse(vitesse) {
  const moyenne = vitesses.reduce((a,b)=>a+b,0)/vitesses.length;
  const mps = vitesse / 3.6;
  const mmps = mps*1000;
  document.getElementById('vitesse').textContent =
    `Temps : ${new Date().toLocaleTimeString()} | Vitesse : ${vitesse.toFixed(2)} km/h | Moy : ${moyenne.toFixed(2)} km/h | Max : ${vitesseMax.toFixed(2)} km/h | ${mps.toFixed(2)} m/s | ${mmps.toFixed(0)} mm/s`;
}

function afficherDistance() {
  const km = distanceTotale / 1000;
  const m = distanceTotale;
  const mm = m * 1000;
  const secondesLumiere = m / 299792458;
  const anneesLumiere = secondesLumiere / (3600*24*365.25);
  document.getElementById('distance').textContent =
    `Distance : ${km.toFixed(3)} km | ${m.toFixed(0)} m | ${mm.toFixed(0)} mm | Cosmique : ${secondesLumiere.toFixed(3)} s lumière | ${anneesLumiere.toExponential(3)} al`;
}

function afficherPourcentage(vitesse) {
  const mps = vitesse / 3.6;
  const pourcentLumiere = (mps / 299792458)*100;
  const pourcentSon = (mps / 343)*100;
  document.getElementById('pourcentage').textContent =
    `% Lumière : ${pourcentLumiere.toExponential(2)}% | % Son : ${pourcentSon.toFixed(2)}%`;
}

function afficherGPS(gps) {
  document.getElementById('gps').textContent =
    `Latitude : ${gps.latitude.toFixed(6)} | Longitude : ${gps.longitude.toFixed(6)} | Précision : ${gps.accuracy.toFixed(0)}%`;
}

// === Capteurs physiques ===
if ('AmbientLightSensor' in window) {
  try {
    const lightSensor = new AmbientLightSensor();
    lightSensor.addEventListener('reading', () => {
      intensitesLumiere.push(lightSensor.illuminance);
      const max = Math.max(...intensitesLumiere);
      const moy = intensitesLumiere.reduce((a,b)=>a+b,0)/intensitesLumiere.length;
      document.getElementById('capteurs').textContent =
        `Lumière : ${lightSensor.illuminance.toFixed(0)} lux | Moy : ${moy.toFixed(0)} | Max : ${max.toFixed(0)}`;
    });
    lightSensor.start();
  } catch (err) { console.warn("Capteur lumière indisponible :", err); }
}

if (navigator.mediaDevices?.getUserMedia) {
  navigator.mediaDevices.getUserMedia({audio:true}).then(stream=>{
    const ctx = new AudioContext();
    const analyser = ctx.createAnalyser();
    const source = ctx.createMediaStreamSource(stream);
    source.connect(analyser);
    const data = new Uint8Array(analyser.frequencyBinCount);
    setInterval(()=>{
      analyser.getByteFrequencyData(data);
      const moy = data.reduce((a,b)=>a+b,0)/data.length;
      const dB = 20*Math.log10(moy);
      intensitesSon.push(dB);
      const max = Math.max(...intensitesSon);
      document.getElementById('capteurs').textContent =
        `Son : ${dB.toFixed(0)} dB | Moy : ${moy.toFixed(0)} | Max : ${max.toFixed(0)} dB`;
    },1000);
  }).catch(err=>console.warn("Microphone indisponible :",err));
}

// === Accélération, inertie, puissance ===
window.addEventListener('devicemotion', e=>{
  const ax=e.accelerationIncludingGravity.x;
  const ay=e.accelerationIncludingGravity.y;
  const az=e.accelerationIncludingGravity.z;
  const g=Math.sqrt(ax**2+ay**2+az**2)/9.81;
  const masse=70;
  const force=masse*9.81*g;
  const puissance=force*e.interval/1000;
  document.getElementById('grandeurs').textContent =
    `Accélération : ${g.toFixed(2)} g | Force : ${force.toFixed(0)} N | Puissance : ${puissance.toFixed(0)} J`;
});

// === Médaillon céleste vivant ===
if (window.Celestial) {
  Celestial.display({
    projection: "aitoff",
    transform: "equatorial",
    center: [0,0,0],
    follow: "zenith",
    location: true,
    interactive: true,
    stars:{show:true,limit:6,colors:true},
    constellations:{names:true,lines:true,bounds:true},
    planets:{show:true,which:["mercury","venus","mars","jupiter","saturn","uranus","neptune","moon"]},
    dsos:{show:true,limit:8},
    meteors:{show:true},
    galaxies:{show:true}
  });
}

// === Synchronisation horaire atomique ===
async function synchroniserHeureAtomique(){
  try{
    const res=await fetch('https://worldtimeapi.org/api/ip');
    const data=await res.json();
    const dateUTC=new Date(data.utc_datetime);
    document.getElementById('horloge').textContent =
      `⏰ Heure atomique : ${dateUTC.toLocaleTimeString()}`;
  }catch(e){console.warn('Erreur NTP',e);}
}
synchroniserHeureAtomique();

// === Horloge cosmique relative ===
const t0=performance.now();
setInterval(()=>{
  const t=performance.now()-t0;
  document.getElementById('horloge').textContent =
    `⏱️ Temps relatif : ${(t/1000).toFixed(2)} s`;
},1000);

// === Météo & qualité de l’air ===
async function chargerMeteo(){
  try{
    const res=await fetch('https://api.weatherapi.com/v1/current.json?key=YOUR_KEY&q=Marseille');
    const meteo=await res.json();
    const m=meteo.current;
    document.getElementById('meteo').textContent=
      `Temp : ${m.temp_c}°C | Humidité : ${m.humidity}% | Vent : ${m.wind_kph} km/h | Rafales : ${m.gust_kph} km/h | Point rosée : ${m.dewpoint_c}°C | Visibilité : ${m.vis_km} km`;
  }catch(e){console.warn('Erreur météo',e);}
}
chargerMeteo();

// === Destination & boussole ===
export function definirDestination(lat, lon){
  destination.latitude = lat;
  destination.longitude = lon;
}
