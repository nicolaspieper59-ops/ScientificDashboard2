let watchId = null;
let t0 = null;
let prevPos = null;
let distanceTotale = 0;
let vitesses = [];
let vmax = 0;
let lastAstroFetch = 0;
let lastMeteoFetch = 0;

function set(id, txt) { 
  const el = document.getElementById(id); 
  if(el) el.textContent = txt; 
}

function demarrer() {
  if(!navigator.geolocation) return set("gps","Géolocalisation non disponible");
  if(watchId!==null) return set("gps","GPS déjà actif");

  t0 = performance.now();
  prevPos = null; distanceTotale=0; vitesses=[]; vmax=0;
  lastAstroFetch=0; lastMeteoFetch=0;

  watchId = navigator.geolocation.watchPosition(onPosition, onGeoError, {enableHighAccuracy:true, maximumAge:5000, timeout:20000});
  requestAnimationFrame(tick);
}

function stop() {
  if(watchId!==null) {
    navigator.geolocation.clearWatch(watchId);
    watchId = null;
    set("gps","GPS arrêté");
  }
}

function resetAll() {
  stop(); t0=null; prevPos=null; distanceTotale=0; vitesses=[]; vmax=0;
  lastAstroFetch=0; lastMeteoFetch=0;
  const ids = ["temps","gps","vitesse","vitesse-moy","vitesse-max","vitesse-ms","pourcentage","distance","distance-cosmique",
               "culmination-soleil","heure-solaire-vraie","heure-solaire-moyenne","equation-temps","lune-phase","lune-magnitude",
               "lever-lune","coucher-lune","culmination-lune","horloge-minecraft","heure-atomique","temperature","pression",
               "humidite","vent","nuages","pluie","neige","uv","qualite-air","ebullition","latitude","longitude","altitude","gps-brut"];
  ids.forEach(id=>set(id,id.replace(/-/g," ")+" : --"));
}

function onGeoError(err) {
  set("gps",`Erreur géolocalisation : ${err.message}`);
}

function onPosition(pos) {
  const c = pos.coords; const t = pos.timestamp;
  set("latitude",`Latitude : ${c.latitude.toFixed(6)}`);
  set("longitude",`Longitude : ${c.longitude.toFixed(6)}`);
  set("altitude",`Altitude : ${c.altitude!==null?c.altitude.toFixed(1):"--"} m`);
  set("gps",`GPS : ${c.accuracy?.toFixed(1)??"--"} m`);
  set("gps-brut",`Précision GPS : ${c.accuracy?.toFixed(1)??"--"} m`);

  if(prevPos){
    const dt = Math.max((t-prevPos.timestamp)/1000,0.001);
    const d = calculerDistance(prevPos,c);
    distanceTotale += d;
    const v = (d/dt)*3.6; 
    vitesses.push(v); if(v>vmax)vmax=v;
    if(vitesses.length>1000) vitesses.shift();
    const moy = vitesses.reduce((a,b)=>a+b,0)/vitesses.length;
    const mps = v/3.6, mmps = mps*1000;
    const ds = distanceTotale/299792458, pctL = (mps/299792458*100), pctS=(mps/343*100);
    set("vitesse",`Vitesse instantanée : ${v.toFixed(2)} km/h`);
    set("vitesse-moy",`Vitesse moyenne : ${moy.toFixed(2)} km/h`);
    set("vitesse-max",`Vitesse max : ${vmax.toFixed(2)} km/h`);
    set("vitesse-ms",`Vitesse : ${mps.toFixed(2)} m/s | ${Math.round(mmps)} mm/s`);
    set("pourcentage",`% Lumière : ${pctL.toExponential?pctL.toExponential(2):pctL.toFixed(6)}% | % Son : ${pctS.toFixed(2)}%`);
    set("distance",`Distance : ${(distanceTotale/1000).toFixed(3)} km | ${Math.round(distanceTotale)} m`);
    set("distance-cosmique",`Distance cosmique : ${ds.toFixed(6)} s lumière`);
  }
  prevPos = {...c, timestamp:t};

  const now = Date.now();
  if(now - lastMeteoFetch > 60000){ lastMeteoFetch=now; meteoCosmique(c.latitude,c.longitude); }
  if(now - lastAstroFetch > 60000){ lastAstroFetch=now; medaillonCeleste(c.latitude,c.longitude); }
}

function tick(){
  if(t0!==null) set("temps",`Temps : ${((performance.now()-t0)/1000).toFixed(2)} s`);
  set("horloge-minecraft",new Date().toLocaleTimeString());
  set("heure-atomique",`Heure atomique (UTC) : ${new Date().toISOString().split('T')[1].split('.')[0]}`);
  requestAnimationFrame(tick);
}

function calculerDistance(a,b){
  const R=6371e3, φ1=a.latitude*Math.PI/180, φ2=b.latitude*Math.PI/180;
  const Δφ=(b.latitude-a.latitude)*Math.PI/180, Δλ=(b.longitude-a.longitude)*Math.PI/180;
  const s=Math.sin(Δφ/2)**2 + Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2;
  return 2*R*Math.atan2(Math.sqrt(s), Math.sqrt(1-s));
      }
  
