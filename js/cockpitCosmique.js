let watchId = null;
let positionPrecedente = null;
let vitesseMax = 0;
let vitesses = [];
let distanceTotale = 0;

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

    const vitesse = gps.speed !== null
      ? gps.speed * 3.6
      : calculerVitesse(gps);

    if (vitesse >= 0 && vitesse < 300) {
      vitesseMax = Math.max(vitesseMax, vitesse);
      vitesses.push(vitesse);
      afficherVitesse(vitesse);
      afficherDistance(gps);
      afficherPourcentage(vitesse);
      afficherGPS(gps);
    }
  }, err => {
    console.error("Erreur GPS :", err);
  }, {
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

function afficherVitesse(vitesse) {
  const moyenne = vitesses.reduce((a, b) => a + b, 0) / vitesses.length;
  const mps = vitesse / 3.6;
  const mmps = mps * 1000;

  document.getElementById('vitesse').textContent =
    `Temps : ${new Date().toLocaleTimeString()} | Vitesse instantanée : ${vitesse.toFixed(2)} km/h | Moyenne : ${moyenne.toFixed(2)} km/h | Max : ${vitesseMax.toFixed(2)} km/h | ${mps.toFixed(2)} m/s | ${mmps.toFixed(0)} mm/s`;
}

function afficherDistance(gps) {
  const km = distanceTotale / 1000;
  const m = distanceTotale;
  const mm = m * 1000;
  const secondesLumiere = m / 299792458;
  const anneesLumiere = secondesLumiere / (3600 * 24 * 365.25);

  document.getElementById('distance').textContent =
    `Distance : ${km.toFixed(3)} km | ${m.toFixed(0)} m | ${mm.toFixed(0)} mm | Cosmique : ${secondesLumiere.toFixed(3)} s lumière | ${anneesLumiere.toExponential(3)} al`;
}

function afficherPourcentage(vitesse) {
  const mps = vitesse / 3.6;
  const pourcentLumiere = (mps / 299792458) * 100;
  const pourcentSon = (mps / 343) * 100;

  document.getElementById('pourcentage').textContent =
    `% Lumière : ${pourcentLumiere.toExponential(2)}% | % Son : ${pourcentSon.toFixed(2)}%`;
}

function afficherGPS(gps) {
  document.getElementById('gps').textContent =
    `Latitude : ${gps.latitude.toFixed(6)} | Longitude : ${gps.longitude.toFixed(6)} | Précision GPS : ${gps.accuracy.toFixed(0)}%`;
    }
function activerCapteurs() {
  // Lumière ambiante
  if ('AmbientLightSensor' in window) {
    try {
      const lightSensor = new AmbientLightSensor();
      lightSensor.addEventListener('reading', () => {
        document.getElementById('capteurs').textContent =
          `Lumière : ${lightSensor.illuminance.toFixed(0)} lux`;
      });
      lightSensor.start();
    } catch (err) {
      console.warn("Capteur lumière indisponible :", err);
    }
  }

  // Son ambiant (simulation via microphone)
  if (navigator.mediaDevices && navigator.mediaDevices.getUserMedia) {
    navigator.mediaDevices.getUserMedia({ audio: true }).then(stream => {
      const audioCtx = new AudioContext();
      const analyser = audioCtx.createAnalyser();
      const source = audioCtx.createMediaStreamSource(stream);
      source.connect(analyser);
      const data = new Uint8Array(analyser.frequencyBinCount);

      function analyserSon() {
        analyser.getByteFrequencyData(data);
        const moyenne = data.reduce((a, b) => a + b, 0) / data.length;
        const dB = 20 * Math.log10(moyenne);
        const Hz = analyser.frequencyBinCount;

        document.getElementById('capteurs').textContent +=
          ` | Son : ${dB.toFixed(0)} dB | Fréquence : ${Hz} Hz`;
        requestAnimationFrame(analyserSon);
      }
      analyserSon();
    }).catch(err => {
      console.warn("Microphone indisponible :", err);
    });
  }

  // Niveau à bulle (inclinaison)
  if ('DeviceOrientationEvent' in window) {
    window.addEventListener('deviceorientation', e => {
      const inclinaison = e.beta; // axe Y
      document.getElementById('capteurs').textContent +=
        ` | Niveau à bulle : ${inclinaison.toFixed(1)}°`;
    });
  }

  // Champ magnétique (si disponible)
  if ('Magnetometer' in window) {
    try {
      const magneto = new Magnetometer();
      magneto.addEventListener('reading', () => {
        const champ = Math.sqrt(
          magneto.x ** 2 + magneto.y ** 2 + magneto.z ** 2
        );
        document.getElementById('capteurs').textContent +=
          ` | Magnétisme : ${champ.toFixed(0)} µT`;
      });
      magneto.start();
    } catch (err) {
      console.warn("Magnétomètre indisponible :", err);
    }
  }
          }
let destination = { latitude: null, longitude: null };

function activerBoussole() {
  if ('DeviceOrientationEvent' in window) {
    window.addEventListener('deviceorientationabsolute' in window ? 'deviceorientationabsolute' : 'deviceorientation', e => {
      const cap = e.alpha; // azimut
      const coordMinecraft = convertirCoordonneesMinecraft();
      const capDest = calculerCapVersDestination(cap);

      document.getElementById('boussole').textContent =
        `Cap : ${cap?.toFixed(0) ?? '--'}° | Coordonnées Minecraft : ${coordMinecraft} | Cap vers destination : ${capDest}`;
    });
  }
}

function convertirCoordonneesMinecraft() {
  if (!positionPrecedente) return '--';
  const x = Math.round(positionPrecedente.longitude * 1000);
  const z = Math.round(positionPrecedente.latitude * 1000);
  return `X:${x} Z:${z}`;
}

function calculerCapVersDestination(capActuel) {
  if (!positionPrecedente || !destination.latitude || !destination.longitude) return '--';

  const φ1 = positionPrecedente.latitude * Math.PI / 180;
  const φ2 = destination.latitude * Math.PI / 180;
  const Δλ = (destination.longitude - positionPrecedente.longitude) * Math.PI / 180;

  const y = Math.sin(Δλ) * Math.cos(φ2);
  const x = Math.cos(φ1) * Math.sin(φ2) - Math.sin(φ1) * Math.cos(φ2) * Math.cos(Δλ);
  const θ = Math.atan2(y, x);
  const capDestination = (θ * 180 / Math.PI + 360) % 360;

  const delta = Math.abs(capActuel - capDestination);
  return `${capDestination.toFixed(0)}° (${delta.toFixed(0)}° d'écart)`;
}

export function definirDestination(lat, lon) {
  destination.latitude = lat;
  destination.longitude = lon;
          }
  
                                    
