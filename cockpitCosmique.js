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

// === Mode rituel ===
export function toggleRituel() {
  rituelActif = !rituelActif;
  document.body.classList.toggle('rituel-off', !rituelActif);
  document.getElementById('toggle-rituel').textContent =
    rituelActif ? '✨ Rituel cosmique : ON' : '🔋 Rituel cosmique : OFF';
}

// === Démarrage / arrêt cockpit ===
export function demarrerCockpit() {
  if (watchId !== null) return;

  if (!("geolocation" in navigator)) {
    document.getElementById("gps").textContent = "GPS non disponible";
    return;
  }

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

      const vitesse = gps.speed !== null
        ? gps.speed * 3.6
        : calculerVitesse(gps);

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

  const a = Math.sin(Δφ / 2) ** 2 +
            Math.cos(φ1) * Math.cos(φ2) *
            Math.sin(Δλ / 2) ** 2;
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
  return R * c;
}

// === Affichage cockpit ===
function afficherVitesse(vitesse) {
  const moyenne = vitesses.length
    ? vitesses.reduce((a, b) => a + b, 0) / vitesses.length
    : 0;

  const mps = vitesse / 3.6;
  const mmps = mps * 1000;

  document.getElementById('vitesse').textContent =
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

  document.getElementById('distance').textContent =
    `Distance : ${km.toFixed(3)} km | ${m.toFixed(0)} m | ` +
    `${mm.toFixed(0)} mm | Cosmique : ${secondesLumiere.toFixed(3)} s lumière | ` +
    `${anneesLumiere.toExponential(3)} al`;
}

function afficherPourcentage(vitesse) {
  const mps = vitesse / 3.6;
  const pourcentLumiere = (mps / 299792458) * 100;
  const pourcentSon = (mps / 343) * 100;

  document.getElementById('pourcentage').textContent =
    `% Lumière : ${pourcentLumiere.toExponential(2)}% | ` +
    `% Son : ${pourcentSon.toFixed(2)}%`;
}

function afficherGPS(gps) {
  document.getElementById('gps').textContent =
    `Latitude : ${gps.latitude.toFixed(6)} | ` +
    `Longitude : ${gps.longitude.toFixed(6)} | ` +
    `Altitude : ${(gps.altitude ?? 0).toFixed(1)} m | ` +
    `Précision GPS : ${gps.accuracy.toFixed(0)}%`;
}

// === Capteurs physiques ===
export function activerCapteurs() {
  // Lumière
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

  // Micro
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
        const dB = 20*Math.log10(moy||1);
        intensitesSon.push(dB);
        const max = Math.max(...intensitesSon);
        document.getElementById('capteurs').textContent =
          `Son : ${dB.toFixed(0)} dB | Moy : ${moy.toFixed(0)} | Max : ${max.toFixed(0)}`;
      },1000);
    }).catch(err=>console.warn("Microphone indisponible :",err));
  }
}

// === Boussole & destination ===
export function activerBoussole() {
  if ("DeviceOrientationEvent" in window) {
    const eventName = "deviceorientationabsolute" in window
      ? "deviceorientationabsolute"
      : "deviceorientation";
    window.addEventListener(eventName, e => {
      if (e.alpha != null) {
        const cap = e.alpha;
        const capDest = calculerCapVersDestination(cap);
        document.getElementById("boussole").textContent =
          `Cap : ${cap.toFixed(0)}° | Cap vers destination : ${capDest}`;
      }
    });
  }
}

function calculerCapVersDestination(capActuel) {
  if (!positionPrecedente || !destination.latitude || !destination.longitude) return "--";
  const φ1 = positionPrecedente.latitude * Math.PI/180;
  const φ2 = destination.latitude * Math.PI/180;
  const Δλ = (destination.longitude - positionPrecedente.longitude)*Math.PI/180;
  const y = Math.sin(Δλ)*Math.cos(φ2);
  const x = Math.cos(φ1)*Math.sin(φ2)-Math.sin(φ1)*Math.cos(φ2)*Math.cos(Δλ);
  const θ = Math.atan2(y, x);
  const capDestination = (θ*180/Math.PI+360)%360;
  const delta = Math.abs(capActuel - capDestination);
  return `${capDestination.toFixed(0)}° (${delta.toFixed(0)}° d'écart)`;
}

export function definirDestination(lat, lon){
  destination.latitude = lat;
  destination.longitude = lon;
}
   
