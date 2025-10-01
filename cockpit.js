let watchId = null;
let positionPrecedente = null;
let vitesseMax = 0;
let vitesses = [];
let distanceTotale = 0;
let destination = { latitude: null, longitude: null };

/* ========================
   GPS & VITESSE
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
        altitude: pos.coords.altitude || 0,
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
    { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 }
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

export function resetVitesseMax() { vitesseMax = 0; }

function calculerVitesse(gps) {
  if (!positionPrecedente) { positionPrecedente = gps; return 0; }
  const dt = (gps.timestamp - positionPrecedente.timestamp)/1000;
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
  return R*c;
}

/* ========================
   AFFICHAGE
======================== */
function afficherVitesse(v) {
  const moyenne = vitesses.length ? vitesses.reduce((a,b)=>a+b,0)/vitesses.length : 0;
  const mps = v/3.6;
  const mmps = mps*1000;
  document.getElementById("vitesse").textContent =
    `Temps : ${new Date().toLocaleTimeString()} | Vitesse : ${v.toFixed(2)} km/h | Moy : ${moyenne.toFixed(2)} km/h | Max : ${vitesseMax.toFixed(2)} km/h | ${mps.toFixed(2)} m/s | ${mmps.toFixed(0)} mm/s`;
}

function afficherDistance() {
  const km = distanceTotale/1000;
  const m = distanceTotale;
  const mm = m*1000;
  const secondesLumiere = m/299792458;
  const al = secondesLumiere/(3600*24*365.25);
  document.getElementById("distance").textContent =
    `Distance : ${km.toFixed(3)} km | ${m.toFixed(0)} m | ${mm.toFixed(0)} mm | Cosmique : ${secondesLumiere.toFixed(3)} s lumière | ${al.toExponential(3)} al`;
}

function afficherPourcentage(v) {
  const mps = v/3.6;
  const pl = (mps/299792458)*100;
  const ps = (mps/343)*100;
  document.getElementById("pourcentage").textContent = `% Lumière : ${pl.toExponential(2)}% | % Son : ${ps.toFixed(2)}%`;
}

function afficherGPS(gps) {
  document.getElementById("gps").textContent =
    `Latitude : ${gps.latitude.toFixed(6)} | Longitude : ${gps.longitude.toFixed(6)} | Altitude : ${gps.altitude.toFixed(1)} m | Précision : ${gps.accuracy.toFixed(0)}%`;
}

/* ========================
   INIT
======================== */
document.addEventListener("DOMContentLoaded", () => {
  const boutonMarche = document.getElementById("marche");
  const boutonReset = document.getElementById("reset");

  boutonMarche.addEventListener("click", () => {
    if (boutonMarche.textContent.includes("▶️")) {
      demarrerCockpit();
      boutonMarche.textContent = "⏹️ Arrêter";
    } else {
      arreterCockpit();
      boutonMarche.textContent = "▶️ Marche";
      // Reset affichage
      document.getElementById("vitesse").textContent = "-- km/h";
      document.getElementById("distance").textContent = "-- km";
      document.getElementById("pourcentage").textContent = "-- %";
      document.getElementById("gps").textContent = "Latitude : -- | Longitude : -- | Altitude : -- | Précision : --%";
    }
  });

  boutonReset.addEventListener("click", resetVitesseMax);
});
