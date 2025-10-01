// === Variables globales avec cache ===
let watchId = null;
let positionPrecedente = null;
let vitesseMax = parseFloat(localStorage.getItem("vitesseMax")) || 0;
let vitesses = [];
let distanceTotale = parseFloat(localStorage.getItem("distanceTotale")) || 0;
let rituelActif = JSON.parse(localStorage.getItem("rituelActif")) ?? true;

let destination = JSON.parse(localStorage.getItem("destination")) || { latitude: null, longitude: null };

// === Mode rituel cosmique ===
const boutonRituel = document.getElementById("toggle-rituel");
if (boutonRituel) {
  boutonRituel.textContent = rituelActif ? "✨ Rituel cosmique : ON" : "🔋 Rituel cosmique : OFF";
  boutonRituel.addEventListener("click", () => {
    rituelActif = !rituelActif;
    localStorage.setItem("rituelActif", rituelActif);
    document.body.classList.toggle("rituel-off", !rituelActif);
    boutonRituel.textContent = rituelActif ? "✨ Rituel cosmique : ON" : "🔋 Rituel cosmique : OFF";
  });
}

// === GPS : démarrage / arrêt ===
export function demarrerCockpit() {
  if (!("geolocation" in navigator)) {
    document.getElementById("statut-gps").textContent = "❌ GPS non disponible sur cet appareil.";
    return;
  }
  if (watchId !== null) return;

  document.getElementById("statut-gps").textContent = "📡 Recherche GPS...";
  watchId = navigator.geolocation.watchPosition(
    pos => {
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
        localStorage.setItem("vitesseMax", vitesseMax);

        vitesses.push(vitesse);
        afficherVitesse(vitesse);
        afficherDistance();
        afficherPourcentage(vitesse);
        afficherGPS(gps);

        localStorage.setItem("distanceTotale", distanceTotale);
        localStorage.setItem("destination", JSON.stringify(destination));
      }
    },
    err => {
      document.getElementById("statut-gps").textContent = "❌ Erreur GPS : " + err.message;
    },
    { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 }
  );
}

export function arreterCockpit() {
  if (watchId !== null) {
    navigator.geolocation.clearWatch(watchId);
    watchId = null;
    document.getElementById("statut-gps").textContent = "🛑 GPS arrêté.";
  }
}

export function resetVitesseMax() {
  vitesseMax = 0;
  localStorage.setItem("vitesseMax", 0);
}

// === Calcul vitesse et distance ===
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
  const a = Math.sin(Δφ / 2) ** 2 + Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ / 2) ** 2;
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
  return R * c;
}

// === Affichages ===
function afficherVitesse(vitesse) {
  const moyenne = vitesses.length ? vitesses.reduce((a, b) => a + b, 0) / vitesses.length : 0;
  document.getElementById("vitesse-gps").textContent =
    `⏱️ ${new Date().toLocaleTimeString()} | ` +
    `Vit : ${vitesse.toFixed(1)} km/h | Moy : ${moyenne.toFixed(1)} | Max : ${vitesseMax.toFixed(1)}`;
}

function afficherDistance() {
  const km = distanceTotale / 1000;
  document.getElementById("distance").textContent =
    `📏 ${km.toFixed(2)} km parcourus`;
}

function afficherPourcentage(vitesse) {
  const mps = vitesse / 3.6;
  const pourcentLumiere = (mps / 299792458) * 100;
  const pourcentSon = (mps / 343) * 100;
  document.getElementById("pourcentage").textContent =
    `% Lumière : ${pourcentLumiere.toExponential(2)} | % Son : ${pourcentSon.toFixed(2)}`;
}

function afficherGPS(gps) {
  document.getElementById("statut-gps").textContent =
    `📍 Lat : ${gps.latitude.toFixed(6)} | Lon : ${gps.longitude.toFixed(6)} | Précision : ±${gps.accuracy}m`;
}

// === Détection des capteurs ===
if ("AmbientLightSensor" in window) {
  try {
    const sensor = new AmbientLightSensor();
    sensor.addEventListener("reading", () => {
      document.getElementById("capteurs").textContent = `💡 Lumière : ${sensor.illuminance.toFixed(1)} lux`;
    });
    sensor.start();
  } catch (e) {
    document.getElementById("capteurs").textContent = "❌ Capteur de lumière non disponible";
  }
} else {
  document.getElementById("capteurs").textContent = "❌ Pas de capteur de lumière";
}

if (navigator.mediaDevices?.getUserMedia) {
  navigator.mediaDevices.getUserMedia({ audio: true }).then(stream => {
    const ctx = new AudioContext();
    const analyser = ctx.createAnalyser();
    const source = ctx.createMediaStreamSource(stream);
    source.connect(analyser);
    const data = new Uint8Array(analyser.frequencyBinCount);
    setInterval(() => {
      analyser.getByteFrequencyData(data);
      const moy = data.reduce((a, b) => a + b, 0) / data.length;
      const dB = 20 * Math.log10(moy || 1);
      document.getElementById("capteurs").textContent = `🎤 Son : ${dB.toFixed(1)} dB`;
    }, 1000);
  }).catch(() => {
    document.getElementById("capteurs").textContent = "❌ Micro non disponible";
  });
          }
