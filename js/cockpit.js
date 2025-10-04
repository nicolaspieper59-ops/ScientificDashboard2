let watchId = null;
let prevPos = null;
let distanceTotale = 0;
let vitesses = [];
let vmax = 0;

function set(id, txt) {
  const el = document.getElementById(id);
  if(el) el.textContent = txt;
}

function demarrer() {
  if (!navigator.geolocation) return set("gps", "Géolocalisation non disponible");
  if (watchId !== null) return set("gps", "GPS déjà actif");

  distanceTotale = 0; vitesses = []; vmax = 0; prevPos = null;

  watchId = navigator.geolocation.watchPosition(
    onPosition,
    onGeoError,
    { enableHighAccuracy: true, maximumAge: 5000, timeout: 20000 }
  );

  set("gps", "Recherche position...");
}

function stop() {
  if (watchId !== null) {
    navigator.geolocation.clearWatch(watchId);
    watchId = null;
    set("gps", "GPS arrêté");
  }
}

function resetAll() {
  stop();
  const ids = ["latitude","longitude","altitude","gps","gps-brut","vitesse","vitesse-moy","vitesse-max","distance"];
  ids.forEach(id => set(id, `${id.replace(/-/g," ")} : --`));
  prevPos = null; distanceTotale = 0; vitesses = []; vmax = 0;
}

function onPosition(pos) {
  const c = pos.coords;
  set("latitude", `Latitude : ${c.latitude.toFixed(6)}`);
  set("longitude", `Longitude : ${c.longitude.toFixed(6)}`);
  set("altitude", `Altitude : ${c.altitude !== null ? c.altitude.toFixed(1) : "--"} m`);
  set("gps", `GPS : précision ${c.accuracy.toFixed(1)} m`);
  set("gps-brut", `Précision GPS : ${c.accuracy.toFixed(1)} m`);

  if(prevPos){
    const dt = Math.max((pos.timestamp-prevPos.timestamp)/1000,0.001);
    const d = calculerDistance(prevPos,c);
    distanceTotale += d;

    const v = (d/dt)*3.6; // km/h
    vitesses.push(v);
    if(v>vmax) vmax=v;
    if(vitesses.length>1000) vitesses.shift();
    const moy = vitesses.reduce((a,b)=>a+b,0)/vitesses.length;

    set("vitesse", `Vitesse instantanée : ${v.toFixed(4)} km/h`);
    set("vitesse-moy", `Vitesse moyenne : ${moy.toFixed(4)} km/h`);
    set("vitesse-max", `Vitesse max : ${vmax.toFixed(4)} km/h`);
    set("distance", `Distance : ${distanceTotale.toFixed(4)} m`);
  }

  prevPos = {...c, timestamp: pos.timestamp};
}

function onGeoError(err) {
  set("gps", `Erreur géolocalisation : ${err.message}`);
}

function calculerDistance(a,b){
  const R = 6371e3;
  const φ1 = a.latitude*Math.PI/180;
  const φ2 = b.latitude*Math.PI/180;
  const Δφ = (b.latitude - a.latitude)*Math.PI/180;
  const Δλ = (b.longitude - a.longitude)*Math.PI/180;
  const s = Math.sin(Δφ/2)**2 + Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2;
  return 2*R*Math.atan2(Math.sqrt(s), Math.sqrt(1-s));
    }
    
