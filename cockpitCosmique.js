/* =====================================
   COCKPIT COSMIQUE COMPLET - VERSION ULTIME
===================================== */

/* === VARIABLES GLOBALES === */
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

      const vitesse = gps.speed !== null ? gps.speed * 3.6 : calculerVitesse(gps);

      if (vitesse >= 0 && vitesse < 300) {
        vitesseMax = Math.max(vitesseMax, vitesse);
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
    {
      enableHighAccuracy: true,
      maximumAge: 0,
      timeout: 10000
    }
  );
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

  const a = Math.sin(Δφ / 2) ** 2 +
            Math.cos(φ1) * Math.cos(φ2) *
            Math.sin(Δλ / 2) ** 2;
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
  return R * c;
}

/* ========================
   AFFICHAGE DES DONNÉES
======================== */
function afficherVitesse(vitesse) {
  const moyenne = vitesses.length
    ? vitesses.reduce((a, b) => a + b, 0) / vitesses.length
    : 0;

  const mps = vitesse / 3.6;
  const mmps = mps * 1000;

  document.getElementById("vitesse").textContent =
    `Temps : ${new Date().toLocaleTimeString()} | ` +
    `Vitesse instantanée : ${vitesse.toFixed(2)} km/h | ` +
    `Moyenne : ${moyenne.toFixed(2)} km/h | ` +
    `Max : ${vitesseMax.toFixed(2)} km/h | ` +
    `${mps.toFixed(2)} m/s | ${mmps.toFixed(0)} mm/s`;
}

function afficherDistance() {
  const km = distanceTotale / 1000;
  const m = distanceTotale;
  const mm = m * 1000;
  const secondesLumiere = m / 299792458;
  const anneesLumiere = secondesLumiere / (3600 * 24 * 365.25);

  document.getElementById("distance").textContent =
    `Distance : ${km.toFixed(3)} km | ${m.toFixed(0)} m | ` +
    `${mm.toFixed(0)} mm | Cosmique : ${secondesLumiere.toFixed(3)} s lumière | ` +
    `${anneesLumiere.toExponential(3)} al`;
}

function afficherPourcentage(vitesse) {
  const mps = vitesse / 3.6;
  const pourcentLumiere = (mps / 299792458) * 100;
  const pourcentSon = (mps / 343) * 100;

  document.getElementById("pourcentage").textContent =
    `% Lumière : ${pourcentLumiere.toExponential(2)}% | % Son : ${pourcentSon.toFixed(2)}%`;
}

function afficherGPS(gps) {
  document.getElementById("gps").textContent =
    `Latitude : ${gps.latitude.toFixed(6)} | ` +
    `Longitude : ${gps.longitude.toFixed(6)} | ` +
    `Altitude : ${(gps.altitude ?? 0).toFixed(1)} m | ` +
    `Précision GPS : ${gps.accuracy.toFixed(0)}%`;
}

/* ========================
   CAPTEURS PHYSIQUES
======================== */
function activerCapteurs() {
  // Lumière ambiante
  if ("AmbientLightSensor" in window) {
    try {
      const lightSensor = new AmbientLightSensor();
      lightSensor.addEventListener("reading", () => {
        intensitesLumiere.push(lightSensor.illuminance);
        const max = Math.max(...intensitesLumiere);
        const moy = intensitesLumiere.reduce((a,b)=>a+b,0)/intensitesLumiere.length;
        document.getElementById("capteurs").textContent =
          `Lumière : ${lightSensor.illuminance.toFixed(0)} lux | Moy : ${moy.toFixed(0)} | Max : ${max.toFixed(0)}`;
      });
      lightSensor.start();
    } catch (err) { console.warn("Capteur lumière indisponible :", err); }
  }

  // Microphone
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
        const dB = 20*Math.log10(moy || 1);
        intensitesSon.push(dB);
        const max = Math.max(...intensitesSon);
        document.getElementById("capteurs").textContent =
          `Son : ${dB.toFixed(0)} dB | Moy : ${moy.toFixed(0)} | Max : ${max.toFixed(0)} dB`;
      },1000);
    }).catch(err=>console.warn("Microphone indisponible :",err));
  }

  // Niveau à bulle
  if ("DeviceOrientationEvent" in window) {
    window.addEventListener("deviceorientation", e => {
      if (e.beta != null) {
        document.getElementById("capteurs").textContent +=
          ` | Niveau à bulle : ${e.beta.toFixed(1)}°`;
      }
    });
  }

  // Champ magnétique
  if ("Magnetometer" in window) {
    try {
      const magneto = new Magnetometer();
      magneto.addEventListener("reading", () => {
        const champ = Math.sqrt(magneto.x**2 + magneto.y**2 + magneto.z**2);
        document.getElementById("capteurs").textContent +=
          ` | Magnétisme : ${champ.toFixed(0)} µT`;
      });
      magneto.start();
    } catch (err) { console.warn("Magnétomètre indisponible :", err); }
  }

  // Accélération & puissance
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
}

/* ========================
   BOUSSOLE & DESTINATION
======================== */
export function definirDestination(lat, lon){
  destination.latitude = lat;
  destination.longitude = lon;
}

/* ========================
   HORLOGE, MÉDAILLON, MÉTÉO
======================== */
function activerHorloge() {
  const horloge = document.getElementById("horloge");
  setInterval(() => {
    const t = new Date();
    horloge.textContent = `⏰ ${t.toLocaleTimeString()}`;
  },1000);
}

function afficherMedaillon() {
  const canvas = document.getElementById("medaillon");
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  ctx.clearRect(0,0,canvas.width,canvas.height);
  ctx.fillStyle="#000"; ctx.fillRect(0,0,canvas.width,canvas.height);
  // Soleil
  ctx.fillStyle="#ffd700"; ctx.beginPath(); ctx.arc(150,50,10,0,2*Math.PI); ctx.fill();
  // Lune
  ctx.fillStyle="#ccc"; ctx.beginPath(); ctx.arc(50,80,6,0,2*Math.PI); ctx.fill();
  // Constellations
  ctx.fillStyle="#0ff"; ctx.fillRect(200,200,2,2);
  ctx.fillRect(220,180,2,2);
  ctx.fillRect(180,220,2,2);
  ctx.fillStyle="#fff"; ctx.font="10px monospace"; ctx.fillText("☀️ Soleil",160,50); ctx.fillText("🌙 Lune",40,80);
}

/* ========================
   Météo & qualité de l'air
======================== */
function chargerMeteo() {
  fetch("https://api.open-meteo.com/v1/forecast?latitude=43.3&longitude=5.4&current_weather=true")
  .then(r=>r.json())
  .then(data=>{
    const m=data.current_weather;
    document.getElementById("meteo").textContent=
      `Temp : ${m.temperature}°C | Vent : ${m.windspeed} km/h`;
  }).catch(()=>document.getElementById("meteo").textContent="Météo indisponible");
}

/* ========================
   INIT DOM
======================== */
document.addEventListener("DOMContentLoaded",()=>{
  const bouton=document.getElementById("marche");
  let actif=false;
  bouton.addEventListener("click",()=>{
    actif=!actif;
    if(actif){
      demarrerCockpit();
      activerCapteurs();
      activerHorloge();
      afficherMedaillon();
      chargerMeteo();
      bouton.textContent="⏹️ Arrêter";
    }else{
      arreterCockpit();
      bouton.textContent="▶️ Marche";
    }
  });
});
