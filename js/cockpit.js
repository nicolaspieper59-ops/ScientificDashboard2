let watchId = null;
let positionPrecedente = null;
let distanceTotale = 0;
let vitesseMax = 0;
let vitesses = [];

/* ======================
   GPS & Vitesse
====================== */
export function demarrerCockpit() {
  if (!("geolocation" in navigator)) {
    document.getElementById("gps").textContent = "GPS non disponible";
    return;
  }
  if (watchId !== null) return;

  watchId = navigator.geolocation.watchPosition(
    pos => {
      const gps = pos.coords;
      const vitesse = gps.speed !== null ? gps.speed * 3.6 : calculerVitesse(pos);
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
      console.error(err);
      document.getElementById("gps").textContent = "Erreur GPS";
    },
    { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 }
  );

  activerHorloge();
  afficherMedaillon();
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
  const φ1 = a.latitude * Math.PI / 180;
  const φ2 = b.latitude * Math.PI / 180;
  const Δφ = (b.latitude - a.latitude) * Math.PI / 180;
  const Δλ = (b.longitude - a.longitude) * Math.PI / 180;
  const c = 2 * Math.atan2(Math.sqrt(Math.sin(Δφ /2)**2 + Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2), Math.sqrt(1 - (Math.sin(Δφ /2)**2 + Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2)));
  return R * c;
}

function afficherVitesse(v) {
  const moyenne = vitesses.length ? vitesses.reduce((a,b)=>a+b,0)/vitesses.length : 0;
  document.getElementById("vitesse").textContent = `Vitesse : ${v.toFixed(2)} km/h | Moyenne : ${moyenne.toFixed(2)} | Max : ${vitesseMax.toFixed(2)}`;
}

function afficherDistance() {
  const km = distanceTotale / 1000;
  document.getElementById("distance").textContent = `Distance : ${km.toFixed(3)} km`;
}

function afficherPourcentage(v) {
  const mps = v/3.6;
  const pctL = (mps/299792458)*100;
  const pctS = (mps/343)*100;
  document.getElementById("pourcentage").textContent = `% Lumière : ${pctL.toExponential(2)}% | % Son : ${pctS.toFixed(2)}%`;
}

function afficherGPS(gps) {
  document.getElementById("gps").textContent = `Latitude : ${gps.latitude.toFixed(6)} | Longitude : ${gps.longitude.toFixed(6)} | Altitude : ${(gps.altitude??0).toFixed(1)} m`;
}

/* ======================
   Horloge
====================== */
function activerHorloge() {
  const horloge = document.getElementById("horloge");
  function update() {
    const d = new Date();
    horloge.textContent = `${String(d.getHours()).padStart(2,'0')}:${String(d.getMinutes()).padStart(2,'0')}:${String(d.getSeconds()).padStart(2,'0')}`;
    requestAnimationFrame(update);
  }
  update();
}

/* ======================
   Médaillon
====================== */
function afficherMedaillon() {
  const medaillon = document.getElementById("medaillon");
  medaillon.innerHTML = "";
  const canvas = document.createElement("canvas");
  canvas.width = 300; canvas.height = 300;
  medaillon.appendChild(canvas);
  const ctx = canvas.getContext("2d");

  ctx.fillStyle = "#000"; ctx.fillRect(0,0,300,300);

  // Cercle
  ctx.strokeStyle = "#0ff"; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.arc(150,150,140,0,2*Math.PI); ctx.stroke();

  // Soleil
  ctx.fillStyle="#ffd700"; ctx.beginPath(); ctx.arc(210,90,8,0,2*Math.PI); ctx.fill(); ctx.fillText("☀️ Soleil", 200,80);

  // Lune
  ctx.fillStyle="#ccc"; ctx.beginPath(); ctx.arc(90,110,6,0,2*Math.PI); ctx.fill(); ctx.fillText("🌙 Lune", 70,100);

  // Constellations
  const stars = [{name:"Orion",x:170,y:210},{name:"Cassiopeia",x:120,y:190}];
  ctx.fillStyle="#0ff";
  stars.forEach(s=>{ctx.beginPath();ctx.arc(s.x,s.y,2,0,2*Math.PI);ctx.fill();ctx.fillText(s.name,s.x+5,s.y+5);});
}
