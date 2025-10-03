import * as SunCalc from 'https://cdn.jsdelivr.net/npm/suncalc@1.9.0/suncalc.js';

let watchId = null;
let vitesses = [], vitesseMax = 0, distanceTotale = 0, positionPrecedente = null, t0 = null;
let destination = null;
let map, marker, destMarker;

// Raccourci pour maj d’un élément
const set = (id, txt) => { const el = document.getElementById(id); if (el) el.textContent = txt; };

// === GPS ===
function startGPS() {
  if (watchId !== null) return;
  t0 = performance.now();
  watchId = navigator.geolocation.watchPosition(
    pos => traiterPosition(pos.coords, pos.timestamp),
    err => set("gps", "Erreur GPS : " + err.message),
    { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 }
  );
  loopTemps();
}

function stopGPS() {
  if (watchId !== null) {
    navigator.geolocation.clearWatch(watchId);
    watchId = null;
    set("gps", "⏹️ Suivi arrêté");
  }
}

function resetGPS() {
  vitesses = [];
  vitesseMax = 0;
  distanceTotale = 0;
  positionPrecedente = null;
  ["vitesse","vitesse-moy","vitesse-max","distance"].forEach(id => set(id,"--"));
}

function loopTemps() {
  const el = document.getElementById("temps");
  function tick() {
    if (t0) {
      const t = (performance.now() - t0) / 1000;
      el.textContent = `Temps : ${t.toFixed(2)} s`;
    }
    requestAnimationFrame(tick);
  }
  tick();
}

function traiterPosition(coords, timestamp) {
  const v = coords.speed ? coords.speed * 3.6 : 0;
  vitesseMax = Math.max(vitesseMax, v);
  vitesses.push(v);
  const moy = vitesses.reduce((a,b) => a+b,0) / vitesses.length;
  set("vitesse", `Vitesse instantanée : ${v.toFixed(2)} km/h`);
  set("vitesse-moy", `Vitesse moyenne : ${moy.toFixed(2)} km/h`);
  set("vitesse-max", `Vitesse max : ${vitesseMax.toFixed(2)} km/h`);
  set("gps", `📍 Lat:${coords.latitude.toFixed(4)} Lon:${coords.longitude.toFixed(4)} | Alt:${coords.altitude??'--'}`);

  // Carte
  if (!map) initMap(coords.latitude, coords.longitude);
  if (marker) marker.setLatLng([coords.latitude, coords.longitude]);
  map.setView([coords.latitude, coords.longitude]);
  
  afficherMedaillon(coords.latitude, coords.longitude);
}

// === Carte Leaflet ===
function initMap(lat, lon) {
  map = L.map("map").setView([lat, lon], 13);
  L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
    attribution: "© OpenStreetMap contributors"
  }).addTo(map);
  marker = L.marker([lat, lon]).addTo(map).bindPopup("Vous êtes ici").openPopup();
  map.on("click", e => {
    destination = e.latlng;
    if (destMarker) destMarker.remove();
    destMarker = L.marker(destination).addTo(map).bindPopup("Destination").openPopup();
    set("destination", `Destination : ${destination.lat.toFixed(4)}, ${destination.lng.toFixed(4)}`);
  });
}

// === Médaillon ===
function afficherMedaillon(lat, lon) {
  const now = new Date();
  const soleil = SunCalc.getTimes(now, lat, lon);
  const lune = SunCalc.getMoonIllumination(now);

  set("culmination-soleil", `☀️ Midi solaire : ${soleil.solarNoon.toLocaleTimeString()}`);
  set("heure-vraie", `Lever : ${soleil.sunrise.toLocaleTimeString()}`);
  set("heure-moyenne", `Coucher : ${soleil.sunset.toLocaleTimeString()}`);
  set("equation-temps", `Éq. temps ≈ ${(soleil.sunset - soleil.sunrise)/60000 - 720} min`);
  set("lune-phase", `🌗 Phase : ${(lune.fraction*100).toFixed(1)}%`);
}

// === Capteurs ===
function activerCapteurs() {
  if ("AmbientLightSensor" in window) {
    try {
      const light = new AmbientLightSensor();
      light.addEventListener("reading", () => set("capteurs", `Lumière : ${Math.round(light.illuminance)} lux`));
      light.start();
    } catch {}
  }
  if ("DeviceOrientationEvent" in window) {
    window.addEventListener("deviceorientation", e => {
      const base = document.getElementById("capteurs").textContent;
      set("capteurs", base.replace(/Orientation : .*°/,"") + ` | Orientation : ${e.alpha?.toFixed(1)}°`);
    });
  }
}

// === Boutons ===
document.getElementById("marche").onclick = () => { startGPS(); activerCapteurs(); };
document.getElementById("stop").onclick = stopGPS;
document.getElementById("reset").onclick = resetGPS;
      
