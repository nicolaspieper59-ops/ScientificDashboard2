let watchId = null;
let t0 = null;
let prevPos = null;
let distanceTotale = 0;
let vitesses = [];
let vmax = 0;
let lastAstroFetch = 0;
let lastMeteoFetch = 0;

function set(id, txt) { const el = document.getElementById(id); if(el) el.textContent = txt; }

function demarrer() {
  if (!navigator.geolocation) return set("gps","Géolocalisation non disponible");
  if (watchId!==null) return set("gps","Déjà en cours");
  t0 = performance.now(); prevPos = null; distanceTotale = 0; vitesses = []; vmax = 0;
  lastAstroFetch = 0; lastMeteoFetch = 0;
  watchId = navigator.geolocation.watchPosition(onPosition,onGeoError,{enableHighAccuracy:true,maximumAge:5000,timeout:20000});
  requestAnimationFrame(tick);
}

function stop() { if(watchId!==null){navigator.geolocation.clearWatch(watchId); watchId=null; set("gps","GPS arrêté");} }

function resetAll() {
  stop(); t0=null; prevPos=null; distanceTotale=0; vitesses=[]; vmax=0;
  lastAstroFetch=0; lastMeteoFetch=0;
  ["temps","gps","vitesse","vitesse-moy","vitesse-max","vitesse-ms","pourcentage","distance","distance-cosmique",
  "culmination-soleil","heure-solaire-vraie","heure-solaire-moyenne","equation-temps","lune-phase","lune-magnitude",
  "lever-lune","coucher-lune","culmination-lune","horloge-minecraft","heure-atomique","temperature","pression","humidite",
  "vent","nuages","pluie","neige","uv","qualite-air","ebullition","latitude","longitude","altitude","gps-brut"]
  .forEach(id=>set(id,id.replace(/-/g," ")+" : --"));
}

// … reste de cockpit.js identique à la version sécurisée précédente …
