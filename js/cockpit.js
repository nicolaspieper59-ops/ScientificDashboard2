// cockpitCosmique.js
let watchId = null;
let positionPrecedente = null;
let vitesseMax = 0;
let vitesses = [];
let distanceTotale = 0;

let destination = { latitude: null, longitude: null };

/* ========================
   GESTION GPS / VITESSE
======================== */
export function demarrerCockpit() {
  if (!("geolocation" in navigator)) {
    safeSetText('gps', 'GPS non disponible');
    return;
  }
  if (watchId !== null) return;

  try {
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

        const vitesse = typeof gps.speed === 'number' && gps.speed !== null
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
        console.error('Erreur GPS :', err);
        safeSetText('gps', 'Erreur GPS : ' + (err.message || err.code || 'inconnue'));
      },
      {
        enableHighAccuracy: true,
        maximumAge: 0,
        timeout: 10000
      }
    );
  } catch (err) {
    console.error('Impossible de démarrer le GPS :', err);
    safeSetText('gps', 'GPS indisponible (exception)');
  }
}

export function arreterCockpit() {
  if (watchId !== null && 'geolocation' in navigator) {
    navigator.geolocation.clearWatch(watchId);
  }
  watchId = null;
  positionPrecedente = null;
  vitesses = [];
  distanceTotale = 0;
  vitesseMax = 0;
}

export function resetVitesseMax() {
  vitesseMax = 0;
}

/* ========================
   CALCULS (vitesse / distance)
======================== */
function calculerVitesse(gps) {
  if (!positionPrecedente) {
    positionPrecedente = gps;
    return 0;
  }

  const dt = (gps.timestamp - positionPrecedente.timestamp) / 1000; // s
  const d = calculerDistance(gps, positionPrecedente); // m
  if (!Number.isFinite(d) || d < 0) {
    positionPrecedente = gps;
    return 0;
  }
  distanceTotale += d;
  positionPrecedente = gps;

  return dt > 0 ? (d / dt) * 3.6 : 0;
}

function calculerDistance(pos1, pos2) {
  // Haversine, retourne la distance en mètres
  const R = 6371e3;
  const φ1 = pos1.latitude * Math.PI / 180;
  const φ2 = pos2.latitude * Math.PI / 180;
  const Δφ = (pos2.latitude - pos1.latitude) * Math.PI / 180;
  const Δλ = (pos2.longitude - pos1.longitude) * Math.PI / 180;

  const a = Math.sin(Δφ / 2) ** 2 +
    Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ / 2) ** 2;
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

  safeSetText('vitesse',
    `Temps : ${new Date().toLocaleTimeString()} | ` +
    `Vitesse instantanée : ${vitesse.toFixed(2)} km/h | ` +
    `Moyenne : ${moyenne.toFixed(2)} km/h | ` +
    `Max : ${vitesseMax.toFixed(2)} km/h | ` +
    `${mps.toFixed(2)} m/s | ${mmps.toFixed(0)} mm/s`
  );
}

function afficherDistance() {
  const km = distanceTotale / 1000;
  const m = distanceTotale;
  const mm = m * 1000;
  const secondesLumiere = m / 299792458;
  const anneesLumiere = secondesLumiere / (3600 * 24 * 365.25);

  safeSetText('distance',
    `Distance : ${km.toFixed(3)} km | ${m.toFixed(0)} m | ` +
    `${mm.toFixed(0)} mm | Cosmique : ${secondesLumiere.toFixed(3)} s lumière | ` +
    `${anneesLumiere.toExponential(3)} al`
  );
}

function afficherPourcentage(vitesse) {
  const mps = vitesse / 3.6;
  const pourcentLumiere = (mps / 299792458) * 100;
  const pourcentSon = (mps / 343) * 100;

  safeSetText('pourcentage',
    `% Lumière : ${pourcentLumiere.toExponential(2)}% | ` +
    `% Son : ${pourcentSon.toFixed(2)}%`
  );
}

function afficherGPS(gps) {
  safeSetText('gps',
    `Latitude : ${gps.latitude.toFixed(6)} | ` +
    `Longitude : ${gps.longitude.toFixed(6)} | ` +
    `Altitude : ${(gps.altitude ?? '--') === '--' ? '--' : (gps.altitude).toFixed(1) + ' m'} | ` +
    `Précision GPS : ${gps.accuracy != null ? gps.accuracy.toFixed(0) + '%' : '--'}`
  );
}

/* ========================
   CAPTEURS PHYSIQUES
   (garde-fous + fallbacks)
======================== */
function activerCapteurs() {
  // Initial placeholder
  safeSetText('capteurs', 'Niveau à bulle : --° | Lumière : -- lux | Son : -- dB | Fréquence : -- Hz | Magnétisme : -- µT');

  // Ambient Light
  if ('AmbientLightSensor' in window) {
    try {
      const sensor = new AmbientLightSensor();
      sensor.addEventListener('reading', () => {
        appendText('capteurs', ` | Lumière : ${Math.round(sensor.illuminance)} lux`);
      });
      sensor.start();
    } catch (err) {
      console.warn('AmbientLightSensor indisponible :', err);
    }
  } else if ('ondevicelight' in window) {
    // Ancien fallback (non standard)
    window.addEventListener('devicelight', e => {
      appendText('capteurs', ` | Lumière : ${Math.round(e.value)} lux`);
    });
  } else {
    // non disponible
  }

  // Microphone / niveau sonore (analyser)
  if (navigator.mediaDevices && navigator.mediaDevices.getUserMedia) {
    navigator.mediaDevices.getUserMedia({ audio: true }).then(stream => {
      try {
        const audioCtx = new (window.AudioContext || window.webkitAudioContext)();
        const analyser = audioCtx.createAnalyser();
        const source = audioCtx.createMediaStreamSource(stream);
        source.connect(analyser);
        analyser.fftSize = 2048;
        const data = new Uint8Array(analyser.frequencyBinCount);

        function analyserSon() {
          analyser.getByteFrequencyData(data);
          const moyenne = data.reduce((a, b) => a + b, 0) / data.length;
          const dB = 20 * Math.log10(moyenne || 1);
          const Hz = analyser.frequencyBinCount;
          // Remplacement : on ne concatène pas à l'infini (on écrase)
          safeSetText('capteurs', getElementText('capteursBase') + ` | Son : ${Math.round(dB)} dB | Fréquence : ${Hz} Hz`);
          requestAnimationFrame(analyserSon);
        }

        // store a base initial text to avoid infinite growth
        setElementTextBase('capteurs', 'Niveau à bulle : --° | Lumière : -- lux');
        analyserSon();
      } catch (err) {
        console.warn('Erreur audio analyser :', err);
      }
    }).catch(err => {
      console.warn('Microphone indisponible :', err);
    });
  } else {
    // not supported
  }

  // Device orientation (niveau à bulle)
  if ('DeviceOrientationEvent' in window) {
    window.addEventListener('deviceorientation', e => {
      if (typeof e.beta === 'number') {
        // Use base text for capteurs so we don't append forever
        setElementTextBase('capteurs', `Lumière : -- lux | Son : -- dB | Fréquence : -- Hz | Magnétisme : -- µT`);
        appendText('capteurs', ` | Niveau à bulle : ${e.beta.toFixed(1)}°`);
      }
    });
  }

  // Magnetometer
  if ('Magnetometer' in window) {
    try {
      const magneto = new Magnetometer();
      magneto.addEventListener('reading', () => {
        const champ = Math.sqrt(magneto.x ** 2 + magneto.y ** 2 + magneto.z ** 2);
        appendText('capteurs', ` | Magnétisme : ${Math.round(champ)} µT`);
      });
      magneto.start();
    } catch (err) {
      console.warn('Magnetometer indisponible :', err);
    }
  } else {
    // fallback possible: use DeviceOrientation / compass heading if available in other events
  }
}

/* ========================
   BOUSSOLE & DESTINATION
======================== */
function activerBoussole() {
  if (!('DeviceOrientationEvent' in window)) return;

  const eventName = 'deviceorientationabsolute' in window ? 'deviceorientationabsolute' : 'deviceorientation';
  window.addEventListener(eventName, e => {
    if (typeof e.alpha === 'number') {
      const cap = e.alpha;
      const coordMinecraft = convertirCoordonneesMinecraft();
      const capDest = calculerCapVersDestination(cap);
      safeSetText('boussole', `Cap : ${cap.toFixed(0)}° | Coordonnées Minecraft : ${coordMinecraft} | Cap vers destination : ${capDest}`);
    }
  });
}

function convertirCoordonneesMinecraft() {
  if (!positionPrecedente) return '--';
  const x = Math.round(positionPrecedente.longitude * 1000);
  const z = Math.round(positionPrecedente.latitude * 1000);
  return `X:${x} Z:${z}`;
}

function calculerCapVersDestination(capActuel) {
  if (!positionPrecedente || destination.latitude == null || destination.longitude == null) return '--';
  const φ1 = positionPrecedente.latitude * Math.PI / 180;
  const φ2 = destination.latitude * Math.PI / 180;
  const Δλ = (destination.longitude - positionPrecedente.longitude) * Math.PI / 180;

  const y = Math.sin(Δλ) * Math.cos(φ2);
  const x = Math.cos(φ1) * Math.sin(φ2) - Math.sin(φ1) * Math.cos(φ2) * Math.cos(Δλ);
  const θ = Math.atan2(y, x);
  const capDestination = (θ * 180 / Math.PI + 360) % 360;
  const delta = Math.abs((capActuel - capDestination + 540) % 360 - 180); // différence angulaire courte
  return `${capDestination.toFixed(0)}° (${delta.toFixed(0)}° d'écart)`;
}

export function definirDestination(lat, lon) {
  destination.latitude = lat;
  destination.longitude = lon;
}

/* ========================
   HORLOGE & MÉDAILLON
======================== */
function activerHorlogeMinecraft() {
  const horloge = document.getElementById('horloge');
  if (!horloge) return;
  function miseAJour() {
    const maintenant = new Date();
    const h = String(maintenant.getHours()).padStart(2, '0');
    const m = String(maintenant.getMinutes()).padStart(2, '0');
    const s = String(maintenant.getSeconds()).padStart(2, '0');
    horloge.textContent = `⏰ Horloge Minecraft\n${h}:${m}:${s}`;
    requestAnimationFrame(miseAJour);
  }
  miseAJour();
}

function afficherMedaillon() {
  const medaillon = document.getElementById('medaillon');
  if (!medaillon) return;

  medaillon.innerHTML = ''; // reset
  const canvas = document.createElement('canvas');
  canvas.width = 300;
  canvas.height = 300;
  medaillon.appendChild(canvas);
  const ctx = canvas.getContext('2d');

  // Fond cosmique simple
  ctx.fillStyle = '#000';
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  const cx = canvas.width / 2;
  const cy = canvas.height / 2;
  const r = 140;

  ctx.strokeStyle = '#444';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(cx, cy, r, 0, 2 * Math.PI);
  ctx.stroke();

  ctx.fillStyle = '#fff';
  ctx.font = '10px monospace';
  ctx.fillText('Zénith', cx - 20, cy - r + 10);
  ctx.fillText('0°', cx - 10, cy + 4);

  ctx.beginPath();
  ctx.arc(cx, cy, r, Math.PI, 0);
  ctx.strokeStyle = '#888';
  ctx.stroke();

  // Soleil
  ctx.fillStyle = '#ffd700';
  ctx.beginPath();
  ctx.arc(cx + 60, cy - 60, 8, 0, 2 * Math.PI);
  ctx.fill();
  ctx.fillText('☀️ Soleil', cx + 50, cy - 70);

  // Lune
  ctx.fillStyle = '#ccc';
  ctx.beginPath();
  ctx.arc(cx - 60, cy - 40, 6, 0, 2 * Math.PI);
  ctx.fill();
  ctx.fillText('🌙 Lune', cx - 70, cy - 50);

  // Constellations
  const constellations = [
    { name: 'Orion', x: cx + 20, y: cy + 60 },
    { name: 'Cassiopeia', x: cx - 30, y: cy + 40 },
    { name: 'Scorpius', x: cx + 70, y: cy + 10 }
  ];
  ctx.fillStyle = '#0ff';
  constellations.forEach(c => {
    ctx.beginPath();
    ctx.arc(c.x, c.y, 2, 0, 2 * Math.PI);
    ctx.fill();
    ctx.fillText(c.name, c.x + 5, c.y + 5);
  });

  // Petite galaxie
  ctx.fillStyle = '#f0f';
  ctx.beginPath();
  ctx.arc(cx, cy + 100, 3, 0, 2 * Math.PI);
  ctx.fill();
  ctx.fillText('🌌 Galaxie', cx - 20, cy + 110);
}

/* ========================
   MÉTÉO & QUALITÉ AIR (avec fallbacks)
======================== */
function chargerMeteo() {
  if (!navigator.onLine) {
    safeSetText('meteo', 'Pas de connexion Internet');
    safeSetText('qualite-air', 'Qualité air : --');
    return;
  }

  // Open-Meteo : current weather (latitude/longitude par défaut, adapter si besoin)
  fetch('https://api.open-meteo.com/v1/forecast?latitude=43.3&longitude=5.4&current_weather=true')
    .then(r => r.json())
    .then(data => {
      const meteo = data.current_weather || {};
      safeSetText('meteo',
        `Température : ${meteo.temperature ?? '--'} °C | Vent : ${meteo.windspeed ?? '--'} km/h | Pression : ${meteo.pressure ?? '--'} hPa | Humidité : ${meteo.humidity ?? '--'}%`
      );
    })
    .catch(err => {
      console.warn('Erreur météo :', err);
      safeSetText('meteo', 'Météo indisponible');
    });

  // OpenAQ pour qualité de l'air (ex: coordinates Marseille approximatives)
  fetch('https://api.openaq.org/v2/latest?coordinates=43.3,5.4')
    .then(r => r.json())
    .then(data => {
      const air = data.results?.[0]?.measurements ?? [];
      const pm25 = air.find(m => m.parameter === 'pm25')?.value ?? '--';
      const uv = air.find(m => m.parameter === 'uv')?.value ?? '--';
      safeSetText('qualite-air', `Qualité air : PM2.5 ${pm25} µg/m³ | Indice UV : ${uv}`);
    })
    .catch(err => {
      console.warn('Erreur qualité air :', err);
      safeSetText('qualite-air', 'Qualité air indisponible');
    });
}

/* ========================
   GRANDEURS SCIENTIFIQUES
======================== */
function afficherGrandeurs() {
  const pointEbullition = 100;
  const gravite = 9.81;
  const vitesseSon = 343;
  const vitesseLumiere = 299792458;

  safeSetText('grandeurs',
    `Point d’ébullition : ${pointEbullition} °C | Gravité : ${gravite} m/s² | ` +
    `Vitesse son : ${vitesseSon} m/s | Vitesse lumière : ${vitesseLumiere.toExponential(2)} m/s`
  );
}

/* ========================
   UTILITAIRES DOM (sécurisés)
======================== */
function safeSetText(id, text) {
  const el = document.getElementById(id);
  if (el) el.textContent = text;
}

function appendText(id, text) {
  const el = document.getElementById(id);
  if (el) {
    // Evite la croissance infinie : si contenu trop long, on remplace
    el.textContent = (el.textContent.length > 1000) ? text : (el.textContent + text);
  }
}

// Utilitaires pour gérer un "base" text pour des éléments qui se mettent à jour en boucle.
const _baseTexts = {};
function setElementTextBase(id, base) {
  _baseTexts[id] = base;
  const el = document.getElementById(id);
  if (el) el.textContent = base;
}
function getElementText(id) {
  const el = document.getElementById(id);
  return el ? el.textContent : (_baseTexts[id] || '');
}

/* ========================
   INIT DOM
======================== */
document.addEventListener('DOMContentLoaded', () => {
  const boutonMarche = document.getElementById('marche');
  const boutonReset = document.getElementById('reset');

  // Valeurs initiales cohérentes (fallbacks visibles)
  safeSetText('vitesse', 'Temps : -- | Vitesse instantanée : -- km/h | Moyenne : -- km/h | Max : -- km/h');
  safeSetText('pourcentage', '% Lumière : --% | % Son : --%');
  safeSetText('distance', 'Distance : -- km | -- m | -- mm | Cosmique : --');
  safeSetText('gps', 'Latitude : -- | Longitude : -- | Altitude : -- | Précision GPS : --%');
  safeSetText('capteurs', 'Niveau à bulle : --° | Lumière : -- lux | Son : -- dB | Fréquence : -- Hz | Magnétisme : -- µT');
  safeSetText('boussole', 'Cap : --° | Coordonnées Minecraft : -- | Cap vers destination : --');
  safeSetText('horloge', '🕒 00:00:00');
  safeSetText('medaillon', '☀️🌙 Médaillon cosmique');
  safeSetText('meteo', 'Température : -- °C | Vent : -- km/h | Pression : -- hPa | Humidité : --%');
  safeSetText('qualite-air', 'Qualité air : --');
  safeSetText('grandeurs', 'Point d’ébullition : -- °C | Gravité : -- m/s²');

  let actif = false;

  if (boutonMarche) {
    boutonMarche.addEventListener('click', () => {
      actif = !actif;
      if (actif) {
        demarrerCockpit();
        activerCapteurs();
        activerBoussole();
        activerHorlogeMinecraft();
        afficherMedaillon();
        chargerMeteo();
        afficherGrandeurs();
        boutonMarche.textContent = '⏹️ Arrêter';
      } else {
        arreterCockpit();
        boutonMarche.textContent = '▶️ Marche';
        // Reset affichage (fallbacks)
        safeSetText('vitesse', 'Temps : -- | Vitesse instantanée : -- km/h | Moyenne : -- km/h | Max : -- km/h');
        safeSetText('distance', 'Distance : -- km | -- m | -- mm | Cosmique : --');
        safeSetText('pourcentage', '% Lumière : --% | % Son : --%');
        safeSetText('gps', 'Latitude : -- | Longitude : -- | Altitude : -- | Précision GPS : --%');
        safeSetText('capteurs', 'Niveau à bulle : --° | Lumière : -- lux | Son : -- dB | Fréquence : -- Hz | Magnétisme : -- µT');
        safeSetText('boussole', 'Cap : --° | Coordonnées Minecraft : -- | Cap vers destination : --');
        safeSetText('horloge', '🕒 00:00:00');
        safeSetText('medaillon', '☀️🌙 Médaillon cosmique');
        safeSetText('meteo', 'Température : -- °C | Vent : -- km/h | Pression : -- hPa | Humidité : --%');
        safeSetText('qualite-air', 'Qualité air : --');
        safeSetText('grandeurs', 'Point d’ébullition : -- °C | Gravité : -- m/s²');
      }
    });
  }

  if (boutonReset) {
    boutonReset.addEventListener('click', () => {
      resetVitesseMax();
      // Indiquer visuellement la réinitialisation
      appendText('vitesse', ' | Max réinitialisé');
    });
  }
});
            
